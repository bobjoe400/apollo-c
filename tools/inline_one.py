#!/usr/bin/env python3
"""inline_one.py -- splice one single-use helper into its only caller.

Usage (from the repo root):
    python3 -I tools/inline_one.py HELPER CALLER [--file src/x.c] [--dry-run]

The file defaults to the helper's row in docs/overview/collapse_candidates.tsv.
Helper and caller must be defined in the same file.  The script refuses
(exit 2, reason on stdout) unless the shape is mechanical:

  * exactly one call site, and the call is a full statement (void helper),
    or `return (helper(...));` with the caller's return type;
  * every argument is the identifier the helper names its parameter, or a
    constant (literal or enum constant; substituted into the body);
  * no `return` is left in the helper except its trailing one after the
    guard de-duplication below -- unless the call is in tail position
    (`return (helper(...))`, or the last statement of a void caller), where
    the helper's returns are the caller's returns;
  * a call under a case label is replaced by a braced block;
  * the helper does not assign its parameters, and none of its locals or
    the file-scope names it uses collides with a name in the caller.

Guard de-duplication: a helper guard `if (!AGC_ASSERT(c)) { return ...; }`
is dropped when every conjunct of c (after replacing the helper's
parameters by the caller's arguments) is already asserted by a caller guard
that dominates the call and has not been invalidated since (assignment to a
variable it reads, a write through a pointer or a call when it reads memory
or calls).  Comparison is on the libclang AST, not on text.

The merged caller must stay <= --max-lines (60) code lines.

Then: the helper's header comment becomes a paragraph comment at the call
site (with the call's own trailing comment, if any), the body is
re-indented to the call's depth, the helper definition and any prototype
(same file or a private src/ header) are deleted, the files are written,
the .c file is compiled with the Makefile flags, and every file is
restored if the compile fails (exit 1).  --dry-run compiles the new text
from stdin and prints the diff without writing.
"""
import argparse
import csv
import difflib
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import callgraph  # noqa: E402
import cgast  # noqa: E402

K = cgast.K
CFLAGS = ['-std=c11', '-Wall', '-Wextra', '-Wpedantic', '-Wconversion', '-Wsign-conversion',
          '-Wshadow', '-Wstrict-prototypes', '-Wmissing-prototypes', '-Wundef', '-Wcast-qual',
          '-Wswitch-enum', '-Werror', '-O2', '-Iinclude']


class Refuse(Exception):
    pass


def lookup_file(helper, caller):
    path = os.path.join('docs', 'overview', 'collapse_candidates.tsv')
    if not os.path.exists(path):
        raise Refuse('no --file and no docs/overview/collapse_candidates.tsv')
    with open(path, encoding='utf-8') as fh:
        for row in csv.DictReader(fh, delimiter='\t'):
            if row['helper'] == helper and row['caller'] == caller:
                return row['file']
    raise Refuse(f'{helper} -> {caller} is not in collapse_candidates.tsv; pass --file')


def comment_start(lines, first_line):
    """1-based first line of the comment block attached above first_line."""
    c = cgast.header_comment(lines, first_line)
    return first_line - c.count('\n') if c else first_line


def stmt_end(lines, cur):
    """(last line, column after ';') of the statement whose expression is cur."""
    ln, col = cur.extent.end.line, cur.extent.end.column - 1
    for i in range(ln - 1, min(ln + 3, len(lines))):
        start = col if i == ln - 1 else 0
        j = lines[i].find(';', start)
        if j >= 0:
            return i + 1, j + 1
    raise Refuse('layout: statement end not found')


def indent_of(s):
    return len(s) - len(s.lstrip(' '))


def reindent(block, delta):
    out = []
    for ln in block:
        if not ln.strip():
            out.append('\n')
        elif delta >= 0:
            out.append(' ' * delta + ln)
        else:
            cut = min(-delta, indent_of(ln))
            out.append(ln[cut:])
    return out


def build(path, helper, caller, max_lines=60):
    with open(path, encoding='utf-8') as fh:
        lines = fh.readlines()
    tu = cgast.parse(path)
    fns = {fn.spelling: fn for fn in cgast.functions(tu, path)}
    if helper not in fns:
        raise Refuse(f'cross-file or missing: {helper} not defined in {path}')
    if caller not in fns:
        raise Refuse(f'cross-file or missing: {caller} not defined in {path}')
    hfn, cfn = fns[helper], fns[caller]
    others = [n for n, f in fns.items() if n != caller and callgraph.find_calls(f, helper)]
    if others:
        raise Refuse('more callers: ' + ','.join(others))
    calls = callgraph.find_calls(cfn, helper)
    if len(calls) != 1:
        raise Refuse(f'multi-site: {len(calls)} calls in {caller}')
    call = calls[0]

    h = callgraph.helper_summary(hfn, lines)
    s = callgraph.site_info(cfn, call, lines)
    m = callgraph.merge(h, s, 1)
    if m['reasons']:
        raise Refuse(';'.join(m['reasons']))
    if m['merged'] > max_lines:
        raise Refuse(f'too-long: {caller} would have {m["merged"]} code lines (> {max_lines})')
    stmt = s['stmt']

    # ---- the inlined block ----------------------------------------------
    b = cgast.body(hfn)
    b0, b1 = b.extent.start.line, b.extent.end.line
    if lines[b0 - 1].strip() != '{' or lines[b1 - 1].strip() != '}':
        raise Refuse('layout: helper body braces not on their own lines')
    drop = set()
    for idx in m['removed_idx']:
        st = h['stmts'][idx]
        drop.update(range(st['first'], st['last'] + 1))
        nxt = st['last'] + 1
        if nxt < b1 and not lines[nxt - 1].strip():
            drop.add(nxt)
    last = h['stmts'][-1] if h['stmts'] else None
    if s['shape'] == 'stmt' and last is not None and last['void_return']:
        drop.update(range(last['first'], last['last'] + 1))
    # constant arguments: substitute the constant's text for the parameter
    subst = {}
    pnames = h['params']
    for i, (kind, _) in enumerate(s['argkind']):
        if kind == 'const':
            a = cgast.kids(call)[1 + i]
            subst[pnames[i]] = cgast.spelled(cgast.strip(a))
    edits = {}
    if subst:
        ctx = cgast.Ctx(hfn)

        def walk(n):
            if n.kind == K.DECL_REF_EXPR and n.referenced is not None \
                    and cgast.ref_key(n.referenced) in ctx.pmap and n.spelling in subst:
                ln, col = n.extent.start.line, n.extent.start.column
                if n.extent.end.line != ln:
                    raise Refuse('layout: parameter use spans lines')
                edits.setdefault(ln, []).append((col - 1, n.extent.end.column - 1,
                                                 subst[n.spelling]))
            for c in cgast.kids(n):
                walk(c)

        walk(b)
    block = []
    for ln in range(b0 + 1, b1):
        if ln in drop:
            continue
        text = lines[ln - 1]
        for c0, c1, rep in sorted(edits.get(ln, []), reverse=True):
            text = text[:c0] + rep + text[c1:]
        block.append(text)
    while block and not block[0].strip():
        block.pop(0)
    while block and not block[-1].strip():
        block.pop()

    # ---- prototypes: same file or a private header under src/ ----------------
    proto_edits, proto_comments = [], []
    for c in tu.cursor.walk_preorder():
        if c.kind != K.FUNCTION_DECL or c.spelling != helper or c.is_definition():
            continue
        f = c.location.file
        if f is None:
            continue
        fp = os.path.relpath(os.path.realpath(f.name), ROOT)
        if fp.startswith('include/'):
            raise Refuse('declared in a public header')
        if not fp.startswith('src/'):
            continue
        with open(fp, encoding='utf-8') as fh:
            flines = fh.readlines()
        p0 = comment_start(flines, c.extent.start.line)
        p1, _ = stmt_end(flines, c)
        if p1 < len(flines) and not flines[p1].strip() and (p0 == 1 or not flines[p0 - 2].strip()):
            p1 += 1
        proto_edits.append((fp, (p0, p1, [])))
        proto_comments.append(cgast.header_comment(flines, c.extent.start.line))

    # ---- the call site ----------------------------------------------------
    s0 = stmt.extent.start.line
    if lines[s0 - 1][:stmt.extent.start.column - 1].strip():
        raise Refuse('layout: call statement does not start its line')
    s1, endcol = stmt_end(lines, stmt)
    trail = lines[s1 - 1][endcol:].strip()
    if trail and not (trail.startswith('/*') and trail.endswith('*/')):
        raise Refuse('layout: code after the call statement')
    ind = indent_of(lines[s0 - 1])
    new = []
    if trail:
        new.append(' ' * ind + trail + '\n')
    hc = cgast.header_comment(lines, hfn.extent.start.line)
    if not hc:
        hc = next((c for c in proto_comments if c), '')
    if hc:
        new.extend(reindent(hc.splitlines(True), ind - indent_of(hc.splitlines()[0])))
    if s['braces']:
        # under a case label (or an unbraced branch): give the locals a block
        new.append(' ' * ind + '{\n')
        new.extend(reindent(block, ind))
        new.append(' ' * ind + '}\n')
    else:
        new.extend(reindent(block, ind - 4))

    # ---- delete the helper definition (with its comment and one blank) ------
    d0 = comment_start(lines, hfn.extent.start.line)
    d1 = hfn.extent.end.line
    if d1 < len(lines) and not lines[d1].strip():
        d1 += 1
    if d0 <= s0 <= d1:
        raise Refuse('layout: call inside the helper span')
    file_edits = {path: [(s0, s1, new), (d0, d1, [])]}
    for fp, ed in proto_edits:
        file_edits.setdefault(fp, []).append(ed)

    return file_edits


def apply_edits(file_edits):
    out = {}
    for fp, eds in file_edits.items():
        with open(fp, encoding='utf-8') as fh:
            lines = fh.readlines()
        eds = sorted({(a, b): rep for a, b, rep in eds}.items(), reverse=True)
        for ((a, b), _), ((c0, c1), _) in zip(eds, eds[1:]):
            if c1 >= a:
                raise Refuse(f'layout: overlapping edits in {fp} at lines {c0}-{c1} and {a}-{b}')
        new = list(lines)
        for (a, b), rep in eds:
            new[a - 1:b] = rep
        out[fp] = (lines, new)
    return out


def compile_text(path, text):
    cmd = ['gcc'] + CFLAGS + ['-iquote', os.path.dirname(path), '-x', 'c', '-c', '-o', os.devnull, '-']
    r = subprocess.run(cmd, input=text, capture_output=True, text=True)
    return r.returncode == 0, r.stderr


def compile_file(path):
    r = subprocess.run(['gcc'] + CFLAGS + ['-c', '-o', os.devnull, path], capture_output=True, text=True)
    return r.returncode == 0, r.stderr


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument('helper')
    ap.add_argument('caller')
    ap.add_argument('--file')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--max-lines', type=int, default=60,
                    help='refuse when the merged caller would exceed this (rule 4)')
    opt = ap.parse_args(argv[1:])
    os.chdir(ROOT)
    try:
        path = opt.file or lookup_file(opt.helper, opt.caller)
        changes = apply_edits(build(path, opt.helper, opt.caller, opt.max_lines))
    except Refuse as e:
        print(f'REFUSED {opt.helper} -> {opt.caller}: {e}')
        return 2
    for fp, (old, new) in changes.items():
        sys.stdout.writelines(difflib.unified_diff(old, new, 'a/' + fp, 'b/' + fp))
    if opt.dry_run:
        ok, err = compile_text(path, ''.join(changes[path][1]))
        print('DRY-RUN compile', 'OK' if ok else 'FAILED\n' + err)
        return 0 if ok else 1
    for fp, (_, new) in changes.items():
        with open(fp, 'w', encoding='utf-8') as fh:
            fh.writelines(new)
    ok, err = compile_file(path)
    if not ok:
        for fp, (old, _) in changes.items():
            with open(fp, 'w', encoding='utf-8') as fh:
                fh.writelines(old)
        print(f'REVERTED {opt.helper} -> {opt.caller}: compile failed\n{err}')
        return 1
    print(f'INLINED {opt.helper} -> {opt.caller} in {path}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
