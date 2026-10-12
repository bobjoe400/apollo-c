#!/usr/bin/env python3
"""xref.py -- AGC label -> C function -> file:line cross-reference.

Usage (from the repo root):
    python3 -I tools/xref.py [--out docs/overview/xref.md] [--tsv FILE]

Sources (no hand-maintained table):
  * Universal Ctags (`ctags --output-format=json --fields=+ne`) for every
    function defined under src/: name, file, first and last line;
  * the boxed prolog directly above each definition: the AGC labels the
    function translates are the rope labels named in its head, i.e. the
    "AGC: LABEL (...)" line when there is one, otherwise the text of the
    first prolog paragraph up to the first citation parenthesis
    ("WTLST4 + WTLST2 (C pp.1227,1230 / L pp.1123,1127): ...",
    "Waitlist power-on: STARTSUB / STARTSB2 (FRESH_START...)"),
    or up to the first ':' / ' -- ' when there is no citation;
  * the block comments inside each function body that open with a label
    and a page citation ("/* BIGAOS (pp. 1450-1451) */", "/* FAIL- .. DBFUN
    (p. ...) */"): the section of an AGC routine inlined into that function;
  * the AGC sources (AGC_SRC, as tools/agc_label.py): only tokens that are
    labels in Comanche055 or Luminary099 count, and each label's file and
    page come from the last "# Page N" line above its definition.

A token counts in a prolog head when it is any rope label (code or data, so
constants such as ZEROVEC are indexed) and in a body section comment only
when it is a code label.  Common words that are also labels (LABEL_STOP in
tools/callgraph.py) never count.  "AGC: none" prologs name no label.

Output: docs/overview/xref.md -- one table per initial letter, a row per
(label, C function) pair: label, rope file and page (C = Comanche055,
L = Luminary099), C function, file:line, and whether the function's prolog
names the label (prolog) or a section comment in its body does (section).
"""
import argparse
import collections
import glob
import json
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SRC = ('/tmp/claude-1000/-home-cooperdesktop-git-apollo-c/'
               '89c6630f-d76a-459d-b8fc-f5cc06c736bc/scratchpad/src/Apollo-11')
ROPES = (('C', 'Comanche055'), ('L', 'Luminary099'))
CAPS_RE = re.compile(r'[A-Z0-9][A-Z0-9+\-./*]*[A-Z][A-Z0-9+\-./*]*|[A-Z][A-Z0-9+\-./*]{1,}')
CITE_RE = re.compile(r'\((?=[^)]*(?:\.agc|\bpp?\.\s*\d|\b[CL]\s*pp?\.|\bpages?\s+\d|Comanche|Luminary))')
SECTION_RE = re.compile(r'^\s*/\*\s+([^a-z(]*?)\s*\((?=[^)]*(?:\bpp?\.\s*\d|\.agc|\b[CL]\s*pp?\.))')
DATA_OPS = {'EQUALS', '=', 'ERASE', 'OCT', 'OCTAL', '2OCT', 'DEC', 'DEC*', '2DEC', '2DEC*', 'VN',
            'NV', 'MM', '2CADR', '2BCADR', '2FCADR', 'GENADR', 'ECADR', 'CADR', 'FCADR', 'ADRES',
            'REMADR', 'BBCON', 'BBCON*', 'DNPTR', 'DNCHAN', '1DNADR', '2DNADR', '3DNADR', '4DNADR',
            '5DNADR', '6DNADR', '=MINUS', '=ECADR', 'MEMORY', 'SETLOC', 'BANK', 'BLOCK',
            'EBANK=', 'SBANK=', 'COUNT', 'COUNT*', 'CHECK=', '=PLUS', 'SUBRO', 'OCT*'}
LABEL_STOP = {'ONE', 'TWO', 'THREE', 'FOUR', 'FIVE', 'SIX', 'SEVEN', 'TEN', 'ZERO', 'HALF',
              'NEG', 'DONE', 'BANK', 'AND', 'NOT', 'OUT', 'END', 'TIME', 'BIT', 'SIGN', 'TEMP',
              'AGC', 'DSKY', 'IMU', 'CDU', 'DAP', 'RCS', 'SPS', 'LGC', 'CMC', 'LM', 'CSM', 'MPAC',
              'TRUE', 'FALSE', 'NULL', 'DP', 'SP', 'TP', 'OK', 'ON', 'OFF', 'SET', 'CLEAR', 'FIX',
              'FUNCTION', 'NAME', 'ARGUMENTS', 'RETURNS', 'NOTES', 'DEVIATION', 'NO', 'YES', 'GOTO'}


def rope_labels(src):
    """label -> {'code': bool, 'where': [(rope letter, file, page)]}."""
    out = {}
    for letter, rope in ROPES:
        for p in sorted(glob.glob(os.path.join(src, rope, '*.agc'))):
            page = 0
            with open(p, encoding='utf-8', errors='replace') as fh:
                for ln in fh:
                    m = re.match(r'^#\s*Page\s+(\d+)', ln)
                    if m:
                        page = int(m.group(1))
                        continue
                    if not ln or ln[0].isspace() or ln[0] in '#$':
                        continue
                    f = ln.split('#')[0].split()
                    if not f:
                        continue
                    e = out.setdefault(f[0], {'code': False, 'where': []})
                    if len(f) == 1 or f[1] not in DATA_OPS:
                        e['code'] = True
                    e['where'].append((letter, os.path.basename(p)[:-4], page))
    return out


def functions():
    out = subprocess.run(['ctags', '--output-format=json', '--fields=+ne', '--c-kinds=f',
                          '-R', '-f', '-', 'src'], cwd=ROOT, capture_output=True, text=True,
                         check=True).stdout
    tags = [json.loads(l) for l in out.splitlines() if l.startswith('{')]
    return sorted((t for t in tags if t.get('kind') == 'function'),
                  key=lambda t: (t['path'], t['line']))


def prolog_text(lines, line):
    """The comment block ending directly above 1-based `line`, as text lines
    with the comment frame removed."""
    i = line - 2
    while i >= 0 and lines[i].strip() == '':
        i -= 1
    if i < 0 or not lines[i].rstrip().endswith('*/'):
        return []
    j = i
    while j >= 0 and not lines[j].lstrip().startswith('/*'):
        j -= 1
    if j < 0:
        return []
    body = []
    for x in lines[j:i + 1]:
        s = x.strip()
        if set(s) <= set('/* '):
            body.append('')
            continue
        s = re.sub(r'^/\*+\s?', '', s)
        s = re.sub(r'\s*\*+/$', '', s)
        s = re.sub(r'^\*\s?', '', s)
        body.append(s)
    return body


def tokens(text, labels, code_only):
    out = []
    for t in CAPS_RE.findall(text):
        cands = [t.rstrip('.:,*')]
        if cands[0] not in labels and '/' in cands[0]:
            cands = [x.rstrip('.:,*') for x in cands[0].split('/')]
        for c in cands:
            e = labels.get(c)
            if not e or c in LABEL_STOP or len(c) < 3:
                continue
            if code_only and not e['code']:
                continue
            if c not in out:
                out.append(c)
    return out


def head_segment(text):
    """The part of a prolog head that names labels: up to the first citation
    parenthesis, after the last ': ' before it; without a citation, up to
    the first ':' / ' -- ' / '.' sentence end; then up to the first
    lower-case word.  "label X" inside the citation overrides the head."""
    m = CITE_RE.search(text)
    if m:
        close = text.find(')', m.start())
        named = re.findall(r'\blabels?\s+([A-Z0-9][A-Z0-9+\-./*, ]*)', text[m.start():close])
        if named:
            return ' '.join(named)
        seg = text[:m.start()]
        if ': ' in seg:
            seg = seg.rsplit(': ', 1)[1]
    else:
        seg = re.split(r':\s|\s--\s|\.\s', text, maxsplit=1)[0]
    # prose after the labels ("P370ALRM after PROCEED") names other code
    return re.split(r'\b[a-z][a-z]+\b', seg, maxsplit=1)[0]


def prolog_labels(body, labels):
    """Labels named by a prolog (list of text lines)."""
    for s in body:
        m = re.match(r'^AGC:\s*(.*)', s)
        if m:
            rest = m.group(1)
            if re.match(r'none\b', rest, re.I):
                return []
            return tokens(head_segment(rest), labels, False)
    para = []
    for s in body:
        if s == '' and para:
            break
        if s:
            para.append(s)
        if re.match(r'^(FUNCTION NAME|ARGUMENTS|RETURNS)\b', s):
            para.pop()
            break
    head = ' '.join(para)
    return tokens(head_segment(head), labels, False)


def section_labels(lines, start, end, labels):
    """(label, line) for section comments inside a function body."""
    out = []
    for n in range(start, min(end, len(lines))):
        m = SECTION_RE.match(lines[n])
        if not m:
            continue
        seg = m.group(1)
        if re.search(r'[a-z]{3,}', seg):
            seg = CAPS_RE.match(seg.strip()).group(0) if CAPS_RE.match(seg.strip()) else ''
        for t in tokens(seg, labels, True):
            out.append((t, n + 1))
    return out


def where_text(e):
    by_rope = collections.OrderedDict()
    for letter, f, page in e['where']:
        by_rope.setdefault((letter, f), []).append(page)
    parts = []
    for (letter, f), pages in by_rope.items():
        pg = ','.join(str(p) for p in sorted(set(pages)))
        parts.append('%s %s p.%s' % (letter, f, pg))
    return '; '.join(parts)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default=os.path.join(ROOT, 'docs', 'overview', 'xref.md'))
    ap.add_argument('--tsv', default=None)
    args = ap.parse_args()
    labels = rope_labels(os.environ.get('AGC_SRC', DEFAULT_SRC))
    if not labels:
        sys.exit('no AGC sources found (set AGC_SRC)')
    rows = []
    unlabelled = 0
    cache = {}
    funcs = functions()
    for t in funcs:
        p = t['path']
        lines = cache.setdefault(p, open(os.path.join(ROOT, p), encoding='utf-8').read().split('\n'))
        own = prolog_labels(prolog_text(lines, t['line']), labels)
        for lab in own:
            rows.append((lab, t['name'], p, t['line'], 'prolog'))
        sec = section_labels(lines, t['line'], t.get('end', t['line']), labels)
        for lab, n in sec:
            if lab not in own:
                rows.append((lab, t['name'], p, n, 'section'))
        if not own and not sec:
            unlabelled += 1
    seen, uniq = set(), []
    for r in rows:
        k = (r[0], r[1], r[2], r[4])
        if k not in seen:
            seen.add(k)
            uniq.append(r)
    uniq.sort(key=lambda r: (r[0], r[2], r[3]))
    if args.tsv:
        with open(args.tsv, 'w', encoding='utf-8') as fh:
            fh.write('label\tagc\tfunction\tfile\tline\tkind\n')
            for lab, fn, p, n, kind in uniq:
                fh.write('%s\t%s\t%s\t%s\t%d\t%s\n' % (lab, where_text(labels[lab]), fn, p, n, kind))
    nlab = len({r[0] for r in uniq})
    nfn = len({(r[1], r[2]) for r in uniq})
    out = ['# Cross-reference: AGC label -> C function',
           '',
           '_Generated by `python3 -I tools/xref.py` from the function prologs and the',
           'in-body section comments of src/ (method in the script docstring); do not',
           'edit by hand. Regenerate after any change under src/._',
           '',
           '%d AGC labels -> %d C functions (%d rows); %d of %d functions name no rope'
           % (nlab, nfn, len(uniq), unlabelled, len(funcs)),
           'label (helpers, word-structure and "AGC: none" functions). Rope column:',
           'C = Comanche055, L = Luminary099, file and listing page of the label',
           'definition. Location is relative to src/. Kind: `prolog` = the function',
           'translates the label;',
           '`section` = a block comment inside the function marks the label\'s code.',
           'A label translated in several places (once per vehicle, or a shared',
           'routine and a vehicle variant) has one row per function.',
           '']
    groups = collections.OrderedDict()
    for r in uniq:
        k = r[0][0] if r[0][0].isalpha() else '0-9'
        groups.setdefault(k, []).append(r)
    out.append(' '.join('[%s](#%s)' % (k, k.lower()) for k in groups))
    out.append('')
    for k, rs in groups.items():
        out.append('## %s' % k)
        out.append('')
        out.append('| AGC label | Rope (file, page) | C function | Location | Kind |')
        out.append('|---|---|---|---|---|')
        for lab, fn, p, n, kind in rs:
            out.append('| `%s` | %s | `%s` | [%s:%d](../../%s#L%d) | %s |'
                       % (lab, where_text(labels[lab]).replace('|', '\\|'), fn, p[4:], n, p, n, kind))
        out.append('')
    with open(args.out, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(out))
    print('%s: %d labels, %d functions, %d rows' % (os.path.relpath(args.out, ROOT), nlab, nfn, len(uniq)))


if __name__ == '__main__':
    main()
