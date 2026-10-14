#!/usr/bin/env python3
"""callgraph.py -- compiler-derived call graph and collapse candidates.

Usage (from the repo root; `make callgraph` does all of this):
    python3 -I tools/callgraph.py --ci-dir build/callgraph \
        --cflow build/callgraph/cflow.txt --out build/overview

Inputs
  * GCC -fcallgraph-info=su,da VCG files (one .ci per TU, src/ and tests/,
    compiled at -O0 -fno-inline so no call is folded away).  Authoritative
    for nodes (a static function's title is "file:name") and edges (one
    edge per call site, labelled file:line:col).
  * cflow -b --no-main output for src/: cross-check of the extern->extern
    edges (cflow does not tell same-named statics apart, so only externs
    are compared).  The differences go to <ci-dir>/crosscheck.txt.
  * libclang (tools/cgast.py): function extents, code-line counts, header
    comments, call-site shapes and the AST of every AGC_ASSERT guard.
  * AGC sources (AGC_SRC, as tools/agc_label.py) for the label list.

Outputs (in --out)
  callgraph.tsv            one row per function defined in src/
  callgraph.dot            Graphviz digraph of src/ (candidates in red,
                           dispatch entries as boxes)
  collapse_candidates.tsv  single-caller helpers that may be inlined, with
                           the merged length after guard de-duplication
"""
import argparse
import collections
import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import cgast  # noqa: E402

K = cgast.K
DEFAULT_SRC = ('/tmp/claude-1000/-home-cooperdesktop-git-apollo-c/'
               '89c6630f-d76a-459d-b8fc-f5cc06c736bc/scratchpad/src/Apollo-11')
NODE_RE = re.compile(r'^node: \{ title: "([^"]+)" label: "([^"]*)"(.*)\}$')
EDGE_RE = re.compile(r'^edge: \{ sourcename: "([^"]+)" targetname: "([^"]+)" label: "([^"]*)" \}$')
GRAPH_RE = re.compile(r'^graph: \{ title: "([^"]+)"')
STMT_PARENTS = (K.COMPOUND_STMT, K.CASE_STMT, K.DEFAULT_STMT)
PAGE_RE = re.compile(r'\.agc\b|\bpp?\.\s*\d|\blabel\b|\bpages?\s+\d')
CAPS_RE = re.compile(r'[A-Z][A-Z0-9+\-./]{2,}')
# AGC labels that are also ordinary words or constants: not a cross-reference
LABEL_STOP = {'ONE', 'TWO', 'THREE', 'FOUR', 'FIVE', 'SIX', 'SEVEN', 'TEN', 'ZERO', 'HALF',
              'NEG', 'DONE', 'BANK', 'AND', 'NOT', 'OUT', 'END', 'TIME', 'BIT', 'SIGN', 'TEMP',
              'AGC', 'DSKY', 'IMU', 'CDU', 'DAP', 'RCS', 'SPS', 'LGC', 'CMC', 'LM', 'CSM', 'MPAC',
              'TRUE', 'FALSE', 'NULL', 'DP', 'SP', 'TP', 'OK', 'ON', 'OFF', 'SET', 'CLEAR', 'FIX'}


# ---- GCC .ci --------------------------------------------------------------

def parse_ci(ci_dir):
    nodes = {}      # title -> {name, file, line, static, defined}
    edges = []      # (src title, dst title, site, tu)
    for path in sorted(glob.glob(os.path.join(ci_dir, '**', '*.ci'), recursive=True)):
        tu = None
        with open(path, encoding='utf-8', errors='replace') as fh:
            for line in fh:
                line = line.rstrip('\n')
                m = GRAPH_RE.match(line)
                if m:
                    tu = os.path.relpath(m.group(1), ROOT) if os.path.isabs(m.group(1)) else m.group(1)
                    continue
                m = NODE_RE.match(line)
                if m:
                    title, label, rest = m.groups()
                    parts = label.split('\\n')
                    loc = parts[1] if len(parts) > 1 else ''
                    f, _, ln = loc.partition(':')
                    defined = 'bytes' in label and 'ellipse' not in rest
                    rec = nodes.get(title)
                    if rec is None or (defined and not rec['defined']):
                        nodes[title] = {'name': parts[0], 'file': os.path.normpath(f) if f else '',
                                        'line': int(ln.split(':')[0]) if ln else 0,
                                        'static': ':' in title, 'defined': defined,
                                        'tu': tu if defined else None}
                    continue
                m = EDGE_RE.match(line)
                if m:
                    edges.append((m.group(1), m.group(2), m.group(3), tu))
    return nodes, edges


# ---- cflow ----------------------------------------------------------------

def parse_cflow(path):
    edges = set()
    stack = []
    with open(path, encoding='utf-8', errors='replace') as fh:
        for line in fh:
            if not line.strip():
                continue
            ind = (len(line) - len(line.lstrip(' '))) // 4
            name = line.strip().split('(')[0].strip()
            del stack[ind:]
            if stack:
                edges.add((stack[-1], name))
            stack.append(name)
    return edges


# ---- AGC labels -------------------------------------------------------------

# yaYUL operations that define data, not code: a label on one of these is an
# erasable, constant or address word, not a routine or paragraph
DATA_OPS = {'EQUALS', '=', 'ERASE', 'OCT', 'OCTAL', '2OCT', 'DEC', 'DEC*', '2DEC', '2DEC*', 'VN',
            'NV', 'MM', '2CADR', '2BCADR', '2FCADR', 'GENADR', 'ECADR', 'CADR', 'FCADR', 'ADRES',
            'REMADR', 'BBCON', 'BBCON*', 'DNPTR', 'DNCHAN', '1DNADR', '2DNADR', '3DNADR', '4DNADR',
            '5DNADR', '6DNADR', '=MINUS', '=ECADR', 'MEMORY', 'SETLOC', 'BANK', 'BLOCK',
            'EBANK=', 'SBANK=', 'COUNT', 'COUNT*', 'CHECK=', '=PLUS', 'SUBRO', 'OCT*'}


def agc_labels():
    """(all labels, code labels) of both ropes; POS gets label -> [(file, line)]."""
    src = os.environ.get('AGC_SRC', DEFAULT_SRC)
    labels, code = set(), set()
    for rope in ('Comanche055', 'Luminary099'):
        for p in glob.glob(os.path.join(src, rope, '*.agc')):
            with open(p, encoding='utf-8', errors='replace') as fh:
                for n, ln in enumerate(fh, 1):
                    if ln and not ln[0].isspace() and ln[0] != '#':
                        f = ln.split('#')[0].split()
                        if not f or f[0].startswith('$'):
                            continue
                        labels.add(f[0])
                        if len(f) == 1 or f[1] not in DATA_OPS:
                            code.add(f[0])
                            POS.setdefault(f[0], []).append((os.path.basename(p), n))
    return labels, code


POS = {}
REFS = {}


def agc_label_refs():
    """label -> references in the rope that uses it most: every operand
    token equal to the label (TC, TCF, BZF, BZMF, CCS/INDEX tables, CADR,
    2CADR, ADRES, interpretive GOTO/CALL/BON operands ...), the defining
    label field and comments excluded.  The refined collapse rule
    (collapse candidates): a label entered from fewer than 2 places
    protects nothing."""
    if REFS:
        return REFS
    src = os.environ.get('AGC_SRC', DEFAULT_SRC)
    for rope in ('Comanche055', 'Luminary099'):
        count = collections.Counter()
        for p in glob.glob(os.path.join(src, rope, '*.agc')):
            with open(p, encoding='utf-8', errors='replace') as fh:
                for ln in fh:
                    code = ln.split('#')[0]
                    if not code.strip() or code.lstrip().startswith('$'):
                        continue
                    f = code.split()
                    if code and not code[0].isspace():
                        f = f[1:]  # the label field defines, it does not refer
                    for tok in re.split(r'[\s,]+', ' '.join(f)):
                        tok = tok.lstrip('-+').rstrip('.:')
                        if tok:
                            count[tok] += 1
        for k, v in count.items():
            REFS[k] = max(REFS.get(k, 0), v)
    return REFS


def _clean(t):
    return t.rstrip('.:,')


def _label_tokens(text, code):
    """Code labels in text; "MAJ+/MAJ-/GMERGE" is split at '/' unless the
    whole token is a label (GOMAXL/D)."""
    out = []
    for t in CAPS_RE.findall(text):
        t = _clean(t)
        if t in code:
            out.append(t)
        else:
            out.extend(_clean(x) for x in t.split('/') if _clean(x) in code)
    return [t for t in out if t not in LABEL_STOP]


def head_labels(comment, code):
    """Code labels the comment names as its own: the opening label, every
    label of an all-capitals head ("CM V06N9933 / V6N99INP: ..."), and
    explicit "label X" references."""
    if not comment:
        return set()
    first = cgast.first_comment_line(comment).lstrip('-').strip()
    head = re.split(r'[:(]|--| - ', first, maxsplit=1)[0]
    if re.search(r'[a-z]', head):
        m = CAPS_RE.match(first)
        text = m.group(0) if m else ''
    else:
        text = head
    text += ' ' + ' '.join(re.findall(r'\blabels?\s+([A-Z0-9+\-/.*]+)', comment))
    return set(_label_tokens(text, code))


def caller_cover(comment, code):
    """What a caller's comment claims as its own: its head labels and the AGC
    lines spanned by "A .. B" / "A to B" ranges in its first line.  Routines it merely names
    in prose ("NCOARSE; GYROTRIM; GOTO SURFDISP") stay distinct."""
    if not comment:
        return set(), []
    labels = head_labels(comment, code)
    spans = []
    for a, b in re.findall(r'([A-Z][A-Z0-9+\-./]*)\s*(?:\.\.|\bto\b|--)\s*([A-Z][A-Z0-9+\-./]*)',
                          cgast.first_comment_line(comment)):
        a, b = _clean(a), _clean(b)
        for fa, la in POS.get(a, []):
            for fb, lb in POS.get(b, []):
                if fa == fb and la <= lb:
                    spans.append((fa, la, lb))
    return labels, spans


def own_labels(helper_comment, caller_comment, code):
    """Helper labels that are neither the caller's nor inside its range."""
    mine = head_labels(helper_comment, code)
    labels, spans = caller_cover(caller_comment, code)
    out = set()
    for t in mine - labels:
        if any(f == fs and lo <= ln <= hi for f, ln in POS.get(t, []) for fs, lo, hi in spans):
            continue
        out.add(t)
    return out


def entry_label(comment, own, code):
    """The helper's entry label: the first of its own labels in its
    comment's head ("BDZCHECK, TAUCHECK, NOBDZ" -> BDZCHECK); the later
    ones are targets inside the routine."""
    first = cgast.first_comment_line(comment or '')
    for t in _label_tokens(first, code):
        if t in own:
            return t
    return sorted(own)[0] if own else None


def has_agc_label(comment, labels):
    if not comment:
        return False
    if PAGE_RE.search(comment):
        return True
    # the comment must open with the label ("GSELECT (p.1433) ...",
    # "NEEDLES3: ..."); a label named later in the prose ("One IGNALOOP
    # iteration") marks a fragment of that routine, not its own label.
    # An all-capitals head before ':' or '(' is a label list ("CM
    # V06N9933 / V6N99INP: ..."): any real label in it counts.
    first = cgast.first_comment_line(comment)
    head = re.split(r'[:(]|--| - ', first, maxsplit=1)[0]
    if re.search(r'[a-z]', head):
        m = CAPS_RE.match(first)
        cands = [m.group(0)] if m else []
    else:
        cands = CAPS_RE.findall(head)
    return any(t.rstrip('.:,') in labels and t.rstrip('.:,') not in LABEL_STOP for t in cands)


# ---- libclang pass ----------------------------------------------------------

def header_decls():
    """Public-header names, and comments on private-header prototypes."""
    public = set()
    proto_comment = {}
    for h in sorted(glob.glob('include/agc/**/*.h', recursive=True)):
        tu = cgast.parse(h, ['-x', 'c'])
        for c in tu.cursor.get_children():
            if c.kind == K.FUNCTION_DECL and cgast.in_file(c, h):
                public.add(c.spelling)
    for h in sorted(glob.glob('src/**/*.h', recursive=True)):
        with open(h, encoding='utf-8') as fh:
            lines = fh.readlines()
        tu = cgast.parse(h, ['-x', 'c'])
        for c in tu.cursor.get_children():
            if c.kind == K.FUNCTION_DECL and cgast.in_file(c, h):
                proto_comment[c.spelling] = (h, cgast.header_comment(lines, c.extent.start.line))
    return public, proto_comment


def helper_summary(fn, src_lines):
    ctx = cgast.Ctx(fn)
    b = cgast.body(fn)
    stmts = []
    for s in cgast.kids(b):
        facts = cgast.guard_facts(s, ctx)
        eff = cgast.summarize(s, ctx)
        stmts.append({'guard': facts,
                      'lines': cgast.span_code_lines(src_lines, s.extent.start.line, s.extent.end.line),
                      'first': s.extent.start.line, 'last': s.extent.end.line,
                      'eff': eff, 'returns': eff['returns'], 'is_return': s.kind == K.RETURN_STMT,
                      'void_return': s.kind == K.RETURN_STMT and not cgast.kids(s)})
    whole = cgast.summarize(b, ctx)
    return {'name': fn.spelling, 'ret': fn.result_type.spelling,
            'params': [p.spelling for p in cgast.params(fn)],
            'stmts': stmts, 'locals': whole['locals'], 'globals': whole['globals'],
            'lines': cgast.body_code_lines(src_lines, fn)}


def classify_arg(a):
    a = cgast.strip(a)
    if a.kind in (K.INTEGER_LITERAL, K.FLOATING_LITERAL, K.CHARACTER_LITERAL,
                  K.STRING_LITERAL, K.CXX_BOOL_LITERAL_EXPR):
        return 'const', None
    if a.kind == K.DECL_REF_EXPR and a.referenced is not None:
        if cgast.ref_class(a.referenced) == 'C':
            return 'const', None
        return 'ident', a.spelling
    return 'expr', None


def site_info(fn, call, src_lines):
    """Shape of the call in its caller and the guard facts established there."""
    ctx = cgast.Ctx()
    b = cgast.body(fn)
    chain = cgast.path_to(b, call)
    shape, stmt = 'expr', None
    # walk up from the call through parentheses / implicit conversions
    i = len(chain) - 1
    j = i - 1
    while j >= 0 and chain[j].kind in cgast.SKIP:
        j -= 1
    up = chain[j] if j >= 0 else None
    if up is not None:
        if up.kind in STMT_PARENTS:
            shape, stmt = 'stmt', chain[j + 1]
        elif up.kind == K.CSTYLE_CAST_EXPR and j > 0 and chain[j - 1].kind in STMT_PARENTS:
            shape, stmt = 'stmt', up
        elif up.kind == K.RETURN_STMT:
            shape, stmt = 'return', up
        elif up.kind == K.VAR_DECL:
            shape, stmt = 'init', chain[j - 1] if j > 0 else up
        elif up.kind == K.BINARY_OPERATOR and cgast.op_of(up) == '=' and j > 0 \
                and chain[j - 1].kind in STMT_PARENTS:
            shape, stmt = 'assign', up
    # the statement's parent: anything but a block needs braces when inlined
    k = chain.index(stmt) if stmt is not None else 0
    braces = k > 0 and chain[k - 1].kind != K.COMPOUND_STMT
    tail = False
    if stmt is not None and shape == 'stmt':
        bk = cgast.kids(b)
        tail = bool(bk) and bk[-1] == stmt
    call_lines = 0
    if stmt is not None:
        call_lines = cgast.span_code_lines(src_lines, stmt.extent.start.line, stmt.extent.end.line)
    args = cgast.kids(call)[1:]
    whole = cgast.summarize(b, ctx)
    names = set(whole['locals']) | {p.spelling for p in cgast.params(fn)}
    pkeys = {cgast.ref_key(p) for p in cgast.params(fn)}
    return {'shape': shape, 'tail': tail, 'call_lines': call_lines, 'stmt': stmt,
            'braces': braces,
            'args': [cgast.canon(a, ctx) for a in args],
            'argkind': [classify_arg(a) for a in args],
            'est': cgast.established_at(fn, call, ctx),
            'names': names, 'locals': set(whole['locals']),
            'stable_params': pkeys - whole['writes'],
            'ret': fn.result_type.spelling,
            'lines': cgast.body_code_lines(src_lines, fn), 'line': call.location.line}


def find_calls(fn, helper_name):
    out = []

    def go(n):
        if n.kind == K.CALL_EXPR and n.spelling == helper_name:
            out.append(n)
        for c in cgast.kids(n):
            go(c)

    go(cgast.body(fn))
    return out


# ---- combining helper and site ----------------------------------------------

def merge(h, s, sites):
    """Guard de-duplication and the merged length estimate for one edge."""
    args = s['args']
    argkeys = []
    for a in args:
        argkeys.append(a[1] if isinstance(a, tuple) and a and a[0] == 'ref' else None)
    est = list(s['est'])
    removed = removed_lines = kept = 0
    returns = 0
    hoist = []
    param_written = False
    removed_idx = []
    for idx, st in enumerate(h['stmts']):
        if st['guard'] is not None:
            facts = [cgast.subst(f, args) for f in st['guard']]
            if all(f in est for f in facts):
                removed_idx.append(idx)
                removed += 1
                removed_lines += st['lines']
                continue
            kept += 1
            returns += st['returns']
            est.extend(f for f in facts if f not in est)
            if all(_hoistable(f, s) for f in facts):
                hoist.append((frozenset(facts), st['lines']))
            continue
        eff = dict(st['eff'])
        w = set()
        for key in eff['writes']:
            if isinstance(key, tuple) and key[0] == 'param':
                param_written = True
                if key[1] < len(argkeys) and argkeys[key[1]]:
                    w.add(argkeys[key[1]])
            else:
                w.add(key)
        eff['writes'] = w
        est = [f for f in est if not cgast.invalidated(f, eff)]
        returns += st['returns']
    last = h['stmts'][-1] if h['stmts'] else None
    trailing = bool(last and last['is_return'])
    early = returns - (1 if trailing else 0)
    merged = s['lines'] + h['lines'] - removed_lines
    if s['shape'] in ('stmt', 'return'):
        merged -= s['call_lines']
    if s['shape'] == 'stmt' and last is not None and last['void_return']:
        merged -= last['lines']

    reasons = []
    if sites > 1:
        reasons.append('multi-site')
    if s['shape'] in ('expr', 'init', 'assign'):
        reasons.append('value-' + s['shape'])
    helper_void = h['ret'] == 'void'
    if s['shape'] == 'stmt' and not helper_void:
        reasons.append('result-unused')
    if s['shape'] == 'return' and h['ret'] != s['ret']:
        reasons.append('return-type')
    for (kind, name), p in zip(s['argkind'], h['params']):
        if kind == 'expr' or (kind == 'ident' and name != p):
            reasons.append('renamed-args')
            break
    tail_ok = s['shape'] == 'return' or (s['shape'] == 'stmt' and s['tail'] and helper_void
                                         and s['ret'] == 'void')
    if early > 0 and not tail_ok:
        reasons.append('early-return')
    if param_written:
        reasons.append('param-written')
    coll = (h['locals'] & s['names']) | (h['globals'] & s['locals'])
    if coll:
        reasons.append('collision:' + ','.join(sorted(coll)))
    return {'removed': removed, 'removed_idx': removed_idx, 'removed_lines': removed_lines,
            'kept': kept,
            'early': early, 'merged': merged, 'reasons': reasons, 'tail_ok': tail_ok,
            'hoist': hoist}


def _hoistable(f, s):
    keys, mem, call = cgast.fact_meta(f)
    return bool(keys) and not mem and not call and keys <= s['stable_params']


# ---- main ---------------------------------------------------------------------

def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument('--ci-dir', default='build/callgraph')
    ap.add_argument('--cflow', default='build/callgraph/cflow.txt')
    ap.add_argument('--out', default='build/overview')
    opt = ap.parse_args(argv[1:])
    os.chdir(ROOT)

    nodes, edges = parse_ci(opt.ci_dir)
    defined = {t: n for t, n in nodes.items() if n['defined']}
    callers = collections.defaultdict(collections.Counter)
    for src, dst, site, tu in edges:
        if dst in defined:
            callers[dst][src] += 1

    def is_test(t):
        return defined.get(t, {}).get('file', '').startswith('tests/')

    def in_dir(t, d):
        return defined.get(t, {}).get('file', '').startswith(d)

    labels, code_labels = agc_labels()
    public, proto_comment = header_decls()

    # libclang: extents, lines, comments for every src definition
    info = {}
    files = sorted({n['file'] for n in defined.values() if n['file'].startswith('src/')})
    tus_needed = collections.defaultdict(list)
    for f in files:
        with open(f, encoding='utf-8') as fh:
            src_lines = fh.readlines()
        tu = cgast.parse(f)
        for fn in cgast.functions(tu, f):
            title = f'{f}:{fn.spelling}' if fn.storage_class == cgast.ci.StorageClass.STATIC \
                else fn.spelling
            com = cgast.header_comment(src_lines, fn.extent.start.line)
            if not com and fn.spelling in proto_comment:
                com = proto_comment[fn.spelling][1]
            info[title] = {'lines': cgast.body_code_lines(src_lines, fn), 'comment': com,
                           'agc_label': has_agc_label(com, labels),
                           'first': fn.extent.start.line,
                           'last': fn.extent.end.line}

    # candidates
    cand = {}
    kept_label = 0
    single = 0
    protected = {}
    for t, n in defined.items():
        if not n['file'].startswith('src/') or n['file'].startswith('src/sim/') or n['name'] == 'main':
            continue
        cs = callers.get(t, {})
        if len(cs) != 1:
            continue
        (c, sites), = cs.items()
        if c not in defined or is_test(c) or in_dir(c, 'src/sim/') or in_dir(c, 'src/exec/dispatch.c'):
            continue
        if n['name'] in public:
            continue
        single += 1
        # protected only by an AGC label of its own, distinct from the
        # caller's (lead decision 2026-10-10): a page reference or the
        # parent's label/page range does not protect a helper
        own = own_labels(info.get(t, {}).get('comment', ''), info.get(c, {}).get('comment', ''),
                         code_labels)
        old = bool(info.get(t, {}).get('agc_label'))
        refs = agc_label_refs()
        entry = entry_label(info.get(t, {}).get('comment', ''), own, code_labels)
        entered = {entry} if entry is not None and refs.get(entry, 0) >= 2 else set()
        if entered:
            kept_label += 1
            protected[t] = {'caller': c, 'sites': sites, 'labels': entered}
            continue
        prot = (f'label<2refs:{entry}={refs.get(entry, 0)}') if own \
            else ('page-only' if old else 'none')
        cand[t] = {'caller': c, 'sites': sites, 'protection': prot}
        tus_needed[n['file']].append(('helper', t))
        tus_needed[defined[c]['file']].append(('site', t))

    # libclang: helper summaries and call-site shapes
    hsum, ssum = {}, {}
    for f, jobs in sorted(tus_needed.items()):
        with open(f, encoding='utf-8') as fh:
            src_lines = fh.readlines()
        tu = cgast.parse(f)
        fns = {fn.spelling: fn for fn in cgast.functions(tu, f)}
        for kind, t in jobs:
            if kind == 'helper':
                hsum[t] = helper_summary(fns[defined[t]['name']], src_lines)
            else:
                cfn = fns[defined[cand[t]['caller']]['name']]
                calls = find_calls(cfn, defined[t]['name'])
                if not calls:
                    continue
                si = site_info(cfn, calls[0], src_lines)
                si.pop('stmt')  # a cursor would keep the whole TU alive
                si['found'] = len(calls)
                ssum[t] = si

    rows = []
    for t, c in cand.items():
        h, s = hsum.get(t), ssum.get(t)
        if h is None or s is None:
            continue
        m = merge(h, s, c['sites'])
        if defined[t]['file'] != defined[c['caller']]['file']:
            m['reasons'].insert(0, 'cross-file')
        c.update(m)
        c['shape'] = s['shape'] + ('-tail' if s['tail'] or s['shape'] == 'return' else '')
        c['caller_lines'] = s['lines']
        c['site_line'] = s['line']
        c['guards'] = sum(1 for st in h['stmts'] if st['guard'] is not None)
        rows.append(t)

    # roots and whole-tree estimates (chains of candidates collapse into one root)
    def root_of(t):
        seen = set()
        while t in cand and t not in seen:
            seen.add(t)
            t = cand[t]['caller']
        return t

    tree = collections.defaultdict(list)
    for t in rows:
        tree[root_of(t)].append(t)
    root_est = {}
    for r, members in tree.items():
        base = info.get(r, {}).get('lines', 0)
        delta = sum(cand[t]['merged'] - cand[t]['caller_lines'] for t in members)
        seen = {}
        for t in members:
            for key, ln in cand[t]['hoist']:
                k2 = (cand[t]['caller'], key)
                if k2 in seen:
                    delta -= ln
                seen[k2] = True
        root_est[r] = base + delta

    os.makedirs(opt.out, exist_ok=True)
    disp = lambda t: defined[t]['name'] if t in defined else t.split(':')[-1]  # noqa: E731

    # callgraph.tsv
    with open(os.path.join(opt.out, 'callgraph.tsv'), 'w', encoding='utf-8') as fh:
        fh.write('callee\tfile\tline\tcallers\tsites\tlines\tstatic\tagc_label\tdispatch\t'
                 'tests\tpublic\tcandidate\n')
        for t in sorted(defined, key=lambda x: (defined[x]['file'], defined[x]['line'])):
            n = defined[t]
            if not n['file'].startswith('src/'):
                continue
            cs = callers.get(t, {})
            names = sorted(('test:' if is_test(c) else '') + disp(c) for c in cs)
            fh.write('\t'.join([n['name'], n['file'], str(n['line']), ','.join(names) or '-',
                                str(sum(cs.values())), str(info.get(t, {}).get('lines', '')),
                                'static' if n['static'] else 'extern',
                                'yes' if info.get(t, {}).get('agc_label') else 'no',
                                'yes' if any(in_dir(c, 'src/exec/dispatch.c') for c in cs) else 'no',
                                'yes' if any(is_test(c) for c in cs) else 'no',
                                'yes' if n['name'] in public else 'no',
                                'yes' if t in rows else 'no']) + '\n')

    # callgraph.dot
    with open(os.path.join(opt.out, 'callgraph.dot'), 'w', encoding='utf-8') as fh:
        fh.write('// generated by tools/callgraph.py (make callgraph)\n')
        fh.write('digraph callgraph {\n  rankdir=LR;\n  node [fontsize=9, shape=ellipse];\n')
        ids = {}
        for t in sorted(defined):
            n = defined[t]
            if not n['file'].startswith('src/'):
                continue
            ids[t] = f'n{len(ids)}'
            attrs = [f'label="{n["name"]}"', f'tooltip="{n["file"]}:{n["line"]}"']
            if any(in_dir(c, 'src/exec/dispatch.c') for c in callers.get(t, {})):
                attrs.append('shape=box')
            if t in rows:
                attrs.append('color=red')
            if n['static']:
                attrs.append('style=dashed')
            fh.write(f'  {ids[t]} [{", ".join(attrs)}];\n')
        for dst, cs in sorted(callers.items()):
            for src, k in sorted(cs.items()):
                if src in ids and dst in ids:
                    lab = f' [label="{k}"]' if k > 1 else ''
                    fh.write(f'  {ids[src]} -> {ids[dst]}{lab};\n')
        fh.write('}\n')

    # collapse_candidates.tsv, smallest helper first
    # the candidates of the first rule first, then the delta of the label
    # rule: page-only (now candidates) and label-distinct (still protected)
    rows.sort(key=lambda t: (cand[t]['protection'].split(':')[0] != 'none', info[t]['lines'],
                             defined[t]['file'], defined[t]['name']))
    with open(os.path.join(opt.out, 'collapse_candidates.tsv'), 'w', encoding='utf-8') as fh:
        fh.write('helper\tfile\tline\tcaller\tsite_line\tsites\thelper_lines\tcaller_lines\t'
                 'guards\tguards_dedup\tdedup_lines\tmerged_lines\tfits60\tshape\tmech\trefuse\t'
                 'root\troot_merged\tprotection\thelper_comment\n')
        for t in rows:
            c = cand[t]
            r = root_of(t)
            fh.write('\t'.join([
                defined[t]['name'], defined[t]['file'], str(defined[t]['line']), disp(c['caller']),
                str(c['site_line']), str(c['sites']), str(info[t]['lines']), str(c['caller_lines']),
                str(c['guards']), str(c['removed']), str(c['removed_lines']), str(c['merged']),
                'yes' if c['merged'] <= 60 else 'no', c['shape'],
                'yes' if not c['reasons'] else 'no', ';'.join(c['reasons']) or '-',
                disp(r), str(root_est[r]), c['protection'],
                cgast.first_comment_line(info[t]['comment']).replace('\t', ' ')[:100] or '-']) + '\n')
        for t in sorted(protected, key=lambda x: (info[x]['lines'], defined[x]['file'])):
            if not info[t]['agc_label']:
                continue
            p = protected[t]
            fh.write('\t'.join([
                defined[t]['name'], defined[t]['file'], str(defined[t]['line']), disp(p['caller']),
                '-', str(p['sites']), str(info[t]['lines']), str(info.get(p['caller'], {}).get('lines', '')),
                '-', '-', '-', '-', '-', '-', 'kept',
                'label-entered:' + ','.join(f'{lb}={agc_label_refs().get(lb, 0)}' for lb in sorted(p['labels'])),
                '-', '-', 'label-distinct',
                cgast.first_comment_line(info[t]['comment']).replace('\t', ' ')[:100] or '-']) + '\n')

    # cflow cross-check of extern -> extern edges in src/
    if os.path.exists(opt.cflow):
        ext = {n['name'] for t, n in defined.items() if not n['static'] and n['file'].startswith('src/')}
        g = {(defined[s]['name'], defined[d]['name']) for s, d, _, _ in edges
             if s in defined and d in defined and not defined[s]['static'] and not defined[d]['static']
             and defined[s]['file'].startswith('src/') and defined[d]['file'].startswith('src/')}
        cf = {(a, b) for a, b in parse_cflow(opt.cflow) if a in ext and b in ext}
        with open(os.path.join(opt.ci_dir, 'crosscheck.txt'), 'w', encoding='utf-8') as fh:
            for e in sorted(g - cf):
                fh.write(f'gcc-only\t{e[0]}\t{e[1]}\n')
            for e in sorted(cf - g):
                fh.write(f'cflow-only\t{e[0]}\t{e[1]}\n')
        print(f'cross-check extern->extern: both {len(g & cf)}, gcc-only {len(g - cf)}, '
              f'cflow-only {len(cf - g)} (details {opt.ci_dir}/crosscheck.txt)')

    nsrc = sum(1 for n in defined.values() if n['file'].startswith('src/'))
    print(f'functions in src/: {nsrc}; edges: {len(edges)}; single-caller helpers: {single} '
          f'(kept for a distinct AGC label: {kept_label}); candidates: {len(rows)} '
          f'(page-only: {sum(1 for t in rows if cand[t]["protection"] == "page-only")}); '
          f'mechanical: {sum(1 for t in rows if not cand[t]["reasons"])}; '
          f'fits 60 after merge: {sum(1 for t in rows if cand[t]["merged"] <= 60)}')
    missing = [t for t in cand if t not in rows]
    if missing:
        print(f'candidates without a libclang call site: {len(missing)}: '
              + ', '.join(disp(t) for t in missing[:10]))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
