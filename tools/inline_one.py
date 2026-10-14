#!/usr/bin/env python3
"""inline_one.py -- splice one single-use helper into its only caller.

Usage (from the repo root):
    python3 -I tools/inline_one.py HELPER CALLER [--file src/x.c]
                                   [--caller-file src/y.c] [--dry-run]
                                   [--max-lines 60]

--file (the helper's file) defaults to the helper's row in
history/data/collapse_candidates.tsv; --caller-file defaults to the
helper's file, else the caller's row in history/data/callgraph.tsv.
Everything is decided on the libclang AST; the script refuses (exit 2,
reason on stdout) whatever it cannot transform exactly.

Shapes
  stmt    `helper(args);` (void helper): replaced by the body.
  return  `return (helper(args));`: replaced by the body; the helper's
          returns become the caller's (same return type).
  value   `T x = helper(args);`, `x = helper(args);` or the call inside
          the controlling expression of an if/switch, an expression
          statement or a return: the body (without its trailing return)
          goes in front of that statement and the call becomes the
          returned expression.  Refused when the call is under && || ?:,
          in a loop condition, or next to another call in the statement.

Arguments
  * same identifier as the parameter, or the parameter not otherwise
    needed: used as is;
  * a constant (literal, enum constant) or another variable, with the
    parameter never assigned in the helper: substituted for the parameter;
  * anything else: `T param = <arg>;` at the top of the inlined block.

Returns
  * The helper's trailing return goes (stmt) or becomes the value (value
    shapes).  In return/tail position all returns stay.
  * A helper guard `if (!AGC_ASSERT(c)) { return ...; }` whose facts the
    caller has already asserted at the call (structural comparison after
    substituting the arguments, invalidated by writes/calls in between) is
    dropped.  A guard that checks something new stays and returns what the
    caller's own guards return (its first guard; else `return;`,
    `return (AGC_STATUS_BAD_ARG);` or `return (false);` by return type).
  * Any other early return is refused.

Names: helper locals go in unbraced when no caller name collides; in a
braced block when the clash is only with names not visible at the call
(and the statement is not a declaration); otherwise refused.

The helper's header comment becomes a paragraph comment at the call site
(with the call's own trailing comment), the helper definition and its
prototypes (its file, a private src/ header) are deleted, the merged caller
must stay <= --max-lines code lines (rule 4), the files are written, every
changed .c is compiled with the Makefile flags, and all files are restored
if one fails (exit 1).  Indentation is approximate: run uncrustify after.
"""
import argparse
import csv
import difflib
import os
import re
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
SIMPLE = re.compile(r'^[A-Za-z_]\w*$|^[0-9][0-9A-Za-z_.]*$')


class Refuse(Exception):
    pass


# ---- lookups ------------------------------------------------------------------

def _tsv(path):
    if not os.path.exists(path):
        return []
    with open(path, encoding='utf-8') as fh:
        return list(csv.DictReader(fh, delimiter='\t'))


def lookup_files(helper, caller, hfile, cfile):
    if hfile is None:
        for row in _tsv(os.path.join('history', 'data', 'collapse_candidates.tsv')):
            if row['helper'] == helper and row['caller'] == caller:
                hfile = row['file']
                break
    if hfile is None:
        raise Refuse(f'{helper} -> {caller} not in collapse_candidates.tsv; pass --file')
    if cfile is None:
        rows = [r for r in _tsv(os.path.join('history', 'data', 'callgraph.tsv'))
                if r['callee'] == caller]
        same = [r for r in rows if r['file'] == hfile]
        if same or not rows:
            cfile = hfile
        elif len(rows) == 1:
            cfile = rows[0]['file']
        else:
            raise Refuse(f'{caller} is defined in several files; pass --caller-file')
    return hfile, cfile


# ---- text helpers (byte offsets, as libclang reports them) -------------------

class Text:
    def __init__(self, path):
        self.path = path
        with open(path, 'rb') as fh:
            self.b = fh.read()

    def line_start(self, off):
        return self.b.rfind(b'\n', 0, off) + 1

    def line_end(self, off):
        """Offset just after the newline of the line holding off."""
        e = self.b.find(b'\n', off)
        return len(self.b) if e < 0 else e + 1

    def semi_after(self, off):
        j = self.b.find(b';', off)
        if j < 0:
            raise Refuse('layout: missing ;')
        return j + 1

    def rest_of_line(self, off):
        return self.b[off:self.line_end(off)].decode().strip()

    def indent_at(self, off):
        ls = self.line_start(off)
        ln = self.b[ls:self.line_end(off)].decode()
        return len(ln) - len(ln.lstrip(' '))

    def render(self, lo, hi, edits):
        """Bytes [lo, hi) with the edits (start, end, bytes) inside applied."""
        out, pos = [], lo
        sel = [e for e in edits if lo <= e[0] and e[1] <= hi]
        # an edit inside a deleted / replaced span (a parameter use inside a
        # dropped guard) goes with it
        sel = [e for e in sel if not any(o is not e and o[0] <= e[0] and e[1] <= o[1]
                                         and (o[0], o[1]) != (e[0], e[1]) for o in sel)]
        for a, b, rep in sorted(sel):
            if a < pos:
                raise Refuse('layout: overlapping edits')
            out.append(self.b[pos:a])
            out.append(rep)
            pos = b
        out.append(self.b[pos:hi])
        return b''.join(out)


def full_lines(t, a, b):
    """Expand [a, b) to whole lines, plus one following blank line."""
    a = t.line_start(a)
    b = t.line_end(b - 1 if b > a else b)
    nxt = t.line_end(b) if b < len(t.b) else b
    if b < len(t.b) and not t.b[b:nxt].strip():
        b = nxt
    return a, b


def stmt_span(t, s):
    """[start, end) of a statement including its ';' and trailing comment."""
    a = s.extent.start.offset
    b = s.extent.end.offset
    if s.kind not in (K.IF_STMT, K.COMPOUND_STMT, K.SWITCH_STMT, K.FOR_STMT, K.WHILE_STMT) \
            and t.b[b - 1:b] != b';':
        b = t.semi_after(b)
    return a, b


def strip_parens(s):
    s = s.strip()
    while s.startswith('(') and s.endswith(')'):
        depth = 0
        for i, ch in enumerate(s):
            depth += ch == '('
            depth -= ch == ')'
            if depth == 0 and i < len(s) - 1:
                return s
        s = s[1:-1].strip()
    return s


def reindent(text, delta):
    out = []
    for ln in text.splitlines(True):
        if not ln.strip():
            out.append('\n')
        elif delta >= 0:
            out.append(' ' * delta + ln)
        else:
            cut = min(-delta, len(ln) - len(ln.lstrip(' ')))
            out.append(ln[cut:])
    return ''.join(out)


# ---- analysis -------------------------------------------------------------------

def visible_names(cfn, stmt):
    """Caller names in scope at stmt: parameters and earlier declarations of
    the enclosing blocks (and for-loop variables)."""
    names = {p.spelling for p in cgast.params(cfn)}
    chain = cgast.path_to(cgast.body(cfn), stmt)
    for parent, child in zip(chain, chain[1:]):
        if parent.kind in (K.COMPOUND_STMT, K.FOR_STMT):
            for s in cgast.kids(parent):
                if s == child:
                    break
                for d in s.walk_preorder() if s.kind in (K.DECL_STMT,) else []:
                    if d.kind == K.VAR_DECL:
                        names.add(d.spelling)
    return names


def all_names(cfn):
    names = {p.spelling for p in cgast.params(cfn)}
    for d in cgast.body(cfn).walk_preorder():
        if d.kind == K.VAR_DECL:
            names.add(d.spelling)
    return names


def recovery(ct, cfn):
    """What the caller's own guards return."""
    for s in cgast.kids(cgast.body(cfn)):
        if cgast.is_guard(s) is not None:
            r = [c for c in s.walk_preorder() if c.kind == K.RETURN_STMT][0]
            # only a value that means the same anywhere (no locals, no params)
            if any(c.kind == K.DECL_REF_EXPR and c.referenced is not None
                   and cgast.ref_class(c.referenced) in ('L', 'P') for c in r.walk_preorder()):
                continue
            a, b = r.extent.start.offset, ct.semi_after(r.extent.end.offset)
            return ct.b[a:b].decode()
    rt = cfn.result_type.spelling
    if rt == 'void':
        return 'return;'
    if rt == 'agc_status_t':
        return 'return (AGC_STATUS_BAD_ARG);'
    if rt == 'bool':
        return 'return (false);'
    raise Refuse(f'guard-recovery: no caller guard to copy for return type {rt}')


def is_pure(n):
    """A postfix lvalue chain with no calls: v, v.m, p->m, a[i], a[2].m ..."""
    n = cgast.strip(n)
    if n.kind == K.DECL_REF_EXPR:
        return True
    if n.kind == K.MEMBER_REF_EXPR:
        ch = cgast.kids(n)
        return bool(ch) and is_pure(ch[0])
    if n.kind == K.ARRAY_SUBSCRIPT_EXPR:
        a, i = cgast.kids(n)
        i = cgast.strip(i)
        return is_pure(a) and (i.kind in (K.DECL_REF_EXPR, K.INTEGER_LITERAL))
    return False


def refs_of(n):
    return {c.spelling for c in n.walk_preorder() if c.kind == K.DECL_REF_EXPR}


def dead_after(cfn, call, stmt, ref):
    """The caller's variable `ref` (a local or parameter) is not read after
    the statement holding the call, and the call is not inside a loop, so the
    helper's assignments to its copy may land on the variable itself."""
    r = ref.referenced
    if r is None or cgast.ref_class(r) not in ('L', 'P'):
        return False
    chain = cgast.path_to(cgast.body(cfn), call)
    if any(n.kind in cgast.LOOPS for n in chain):
        return False
    end = stmt.extent.end.offset
    key = cgast.ref_key(r)
    for n in cgast.body(cfn).walk_preorder():
        if n.kind == K.DECL_REF_EXPR and n.referenced is not None \
                and cgast.ref_key(n.referenced) == key and n.extent.start.offset > end:
            return False
    return True


def value_statement(cfn, call):
    """(statement, parent-needs-parens) for a call whose value is used."""
    chain = cgast.path_to(cgast.body(cfn), call)
    i = len(chain) - 1
    j = i - 1
    while j >= 0 and chain[j].kind in cgast.SKIP:
        j -= 1
    up = chain[j]
    parens = not (up.kind in (K.VAR_DECL, K.RETURN_STMT, K.CALL_EXPR, K.PAREN_EXPR)
                  or (up.kind == K.BINARY_OPERATOR and cgast.op_of(up) == '='))
    k = i
    while k > 0 and chain[k - 1].kind not in callgraph.STMT_PARENTS:
        k -= 1
        n = chain[k]
        if n.kind == K.CONDITIONAL_OPERATOR or (
                n.kind == K.BINARY_OPERATOR and cgast.op_of(n) in ('&&', '||', ',')):
            raise Refuse('value-conditional: the call is under && || ?: or ,')
    stmt = chain[k]
    region = stmt
    if stmt.kind in (K.IF_STMT, K.SWITCH_STMT):
        region = cgast.kids(stmt)[0]
        if call not in list(region.walk_preorder()):
            raise Refuse('value-branch: the call is inside a branch, not the condition')
    elif stmt.kind in (K.FOR_STMT, K.WHILE_STMT, K.DO_STMT):
        raise Refuse('value-loop: the call is in a loop statement')
    inside = {c.hash for c in call.walk_preorder()}
    above = {c.hash for c in chain}
    for c in region.walk_preorder():
        if c.kind == K.CALL_EXPR and c.hash not in inside and c.hash not in above \
                and c.spelling != 'AGC_ASSERT':
            raise Refuse(f'value-order: another call ({c.spelling}) in the same statement')
    braces = chain[k - 1].kind != K.COMPOUND_STMT
    return stmt, parens, braces


def build(hfile, cfile, helper, caller, max_lines=60):
    cgast._SRC_CACHE.clear()
    ht, ct = Text(hfile), (Text(cfile) if cfile != hfile else None)
    ct = ct or ht
    htu = cgast.parse(hfile)
    ctu = htu if cfile == hfile else cgast.parse(cfile)
    hfns = {fn.spelling: fn for fn in cgast.functions(htu, hfile)}
    cfns = {fn.spelling: fn for fn in cgast.functions(ctu, cfile)}
    if helper not in hfns:
        raise Refuse(f'missing: {helper} not defined in {hfile}')
    if caller not in cfns:
        raise Refuse(f'missing: {caller} not defined in {cfile}')
    hfn, cfn = hfns[helper], cfns[caller]
    others = [n for n, f in cfns.items() if n != caller and callgraph.find_calls(f, helper)]
    if others:
        raise Refuse('more callers: ' + ','.join(others))
    calls = callgraph.find_calls(cfn, helper)
    if len(calls) != 1:
        raise Refuse(f'multi-site: {len(calls)} calls in {caller}')
    call = calls[0]
    with open(hfile, encoding='utf-8') as fh:
        hlines = fh.readlines()
    with open(cfile, encoding='utf-8') as fh:
        clines = fh.readlines()
    h = callgraph.helper_summary(hfn, hlines)
    s = callgraph.site_info(cfn, call, clines)
    # an argument &v is never NULL: `p != NULL` holds for it
    for ac in s['args']:
        if isinstance(ac, tuple) and ac[:2] == ('un', '&'):
            for null in (('cast', 'void *', ('lit', 'NULL')), ('lit', '0'),
                         ('cast', 'void *', ('lit', '0')), ('lit', 'NULL')):
                s['est'].append(('bin', '!=', ac, null))
                s['est'].append(('bin', '!=', null, ac))
    m = callgraph.merge(h, s, 1)
    shape = s['shape']
    if shape == 'stmt' and h['ret'] != 'void':
        raise Refuse('result-unused: non-void helper called as a statement')
    if shape == 'return' and h['ret'] != s['ret']:
        raise Refuse('return-type')
    value = shape in ('init', 'assign', 'expr')
    tail = shape == 'return' or (shape == 'stmt' and s['tail'] and s['ret'] == 'void')

    hstmts = cgast.kids(cgast.body(hfn))
    last = hstmts[-1] if hstmts else None
    trailing = last is not None and last.kind == K.RETURN_STMT
    if value and not trailing:
        raise Refuse('value: no trailing return')

    # ---- returns: dropped guards, kept guards, early returns ---------------
    removed = set(m['removed_idx'])
    hedits = []
    for idx in sorted(removed):
        a, b = stmt_span(ht, hstmts[idx])
        hedits.append(full_lines(ht, a, b) + (b'',))
    rec = None
    for idx, st in enumerate(hstmts):
        if idx in removed or (trailing and st == last):
            continue
        rets = [c for c in st.walk_preorder() if c.kind == K.RETURN_STMT]
        if not rets or tail:
            continue
        if cgast.is_guard(st) is None:
            raise Refuse('early-return: a return outside a guard')
        if rec is None:
            rec = recovery(ct, cfn)
        r = rets[0]
        a, b = r.extent.start.offset, ht.semi_after(r.extent.end.offset)
        rest = ht.b[b:ht.line_end(b)].decode()
        if rest.strip().startswith('/*') and rest.strip().endswith('*/'):
            b = ht.line_end(b) - 1  # the old return's comment goes with it
        hedits.append((a, b, rec.encode()))
    rvalue = None
    if trailing and not tail:
        if value:
            ex = cgast.kids(last)
            if not ex:
                raise Refuse('value: bare return')
            rvalue = (ex[0].extent.start.offset, ex[0].extent.end.offset)
        a, b = stmt_span(ht, last)
        hedits.append(full_lines(ht, a, b) + (b'',))

    hctx = cgast.Ctx(hfn)
    params = cgast.params(hfn)
    args = cgast.kids(call)[1:]
    writes = cgast.summarize(cgast.body(hfn), hctx)['writes']
    hlocals = set(h['locals'])
    # ---- names visible at the call ------------------------------------------------
    if value:
        stmt, parens, braces = value_statement(cfn, call)
    else:
        stmt, parens, braces = s['stmt'], False, s['braces']
    vis = visible_names(cfn, stmt)
    cnames = all_names(cfn)
    # `T x = helper(..)` where the helper returns its local L of type T: L
    # becomes x and the declaration goes (no `T x = L;` copy)
    merge_into = None
    if value and stmt.kind == K.DECL_STMT and trailing and cgast.kids(last):
        vds = [c for c in cgast.kids(stmt) if c.kind == K.VAR_DECL]
        rex = cgast.strip(cgast.kids(last)[0])
        L = rex.referenced if rex.kind == K.DECL_REF_EXPR else None
        if len(vds) == 1 and L is not None and L.kind == K.VAR_DECL \
                and cgast.ref_class(L) == 'L' and L.type.spelling == vds[0].type.spelling \
                and sum(1 for d in cgast.body(cfn).walk_preorder()
                        if d.kind == K.VAR_DECL and d.spelling == vds[0].spelling) == 1 \
                and vds[0].spelling not in (hlocals - {L.spelling}):
            merge_into = (cgast.ref_key(L), L.spelling, vds[0].spelling)
            hlocals = (hlocals - {L.spelling}) | {vds[0].spelling}
            cnames = cnames - {vds[0].spelling}
    if h['globals'] & (vis - {p.spelling for p in cgast.params(cfn)}):
        raise Refuse('collision: a caller local hides a file-scope name the helper uses')

    # ---- arguments ------------------------------------------------------------------
    decls, subst, addr = [], {}, {}
    heff = cgast.summarize(cgast.body(hfn), hctx)
    hcalls = [c for c in cgast.body(hfn).walk_preorder()
              if c.kind == K.CALL_EXPR and c.spelling != 'AGC_ASSERT']
    pure_helper = not heff['memwrite'] and not hcalls
    for i, (p, a) in enumerate(zip(params, args)):
        kind, name = callgraph.classify_arg(a)
        written = ('param', i) in writes
        atext = ct.b[a.extent.start.offset:a.extent.end.offset].decode()
        if kind == 'ident' and name == p.spelling:
            if written and not dead_after(cfn, call, stmt, cgast.strip(a)):
                raise Refuse(f'param-written: {p.spelling} is assigned in the helper '
                             'and the caller reads it afterwards')
            continue
        if not written and (kind == 'const' or (kind == 'ident' and name not in hlocals)):
            subst[i] = cgast.spelled(cgast.strip(a)) if kind == 'const' else name
            continue
        sa_ = cgast.strip(a)
        if not written and pure_helper and is_pure(sa_) and not (refs_of(sa_) & hlocals):
            subst[i] = atext  # a postfix expression read in a helper that changes nothing
            continue
        if not written and sa_.kind == K.UNARY_OPERATOR and cgast.op_of(sa_) == '&':
            v = cgast.strip(cgast.kids(sa_)[0])
            if v.kind == K.DECL_REF_EXPR and v.spelling not in hlocals:
                addr[i] = v.spelling  # &v: p->m becomes v.m, *p becomes v, p becomes &v
                continue
        decls.append([p, atext])
        hlocals.add(p.spelling)

    # ---- clashes: a braced block when nothing visible clashes, else rename -------
    rename = {}
    clash_vis = hlocals & vis
    clash_any = hlocals & cnames
    if clash_any and not clash_vis and not (value and stmt.kind == K.DECL_STMT):
        braces = True
    else:
        taken = cnames | hlocals | set(h['globals'])
        words = [w for w in helper.split('_') if w and w not in ('agc', 'csm', 'lm')]
        for nm in sorted(clash_any):
            for k in range(1, len(words) + 1):
                cand = nm + '_' + '_'.join(words[-k:])
                if cand not in taken:
                    break
            else:
                raise Refuse(f'collision:{nm}')
            rename[nm] = cand
            taken.add(cand)
    pidx = {cgast.ref_key(p): i for i, p in enumerate(params)}
    hbody = cgast.body(hfn)
    local_keys = {}
    if merge_into:
        rename.pop(merge_into[2], None)
    for d in hbody.walk_preorder():
        if merge_into and d.kind == K.VAR_DECL and cgast.ref_key(d) == merge_into[0]:
            local_keys[merge_into[0]] = merge_into[2]
            if merge_into[1] != merge_into[2]:
                o = d.location.offset
                hedits.append((o, o + len(d.spelling), merge_into[2].encode()))
            continue
        if d.kind == K.VAR_DECL and d.spelling in rename:
            local_keys[cgast.ref_key(d)] = rename[d.spelling]
            o = d.location.offset
            hedits.append((o, o + len(d.spelling), rename[d.spelling].encode()))
    for p, _ in decls:
        if p.spelling in rename:
            local_keys[cgast.ref_key(p)] = rename[p.spelling]

    def walk(n, parent):
        if n.kind == K.DECL_REF_EXPR and n.referenced is not None:
            key = cgast.ref_key(n.referenced)
            i = pidx.get(key)
            a0, a1 = n.extent.start.offset, n.extent.end.offset
            if key in local_keys:
                hedits.append((a0, a1, local_keys[key].encode()))
            elif i in subst:
                hedits.append((a0, a1, subst[i].encode()))
            elif i in addr:
                v = addr[i]
                if parent is not None and parent.kind == K.MEMBER_REF_EXPR \
                        and cgast.is_arrow(parent, n):
                    hedits.append((a0, parent.location.offset, (v + '.').encode()))
                elif parent is not None and parent.kind == K.UNARY_OPERATOR \
                        and cgast.op_of(parent) == '*':
                    hedits.append((parent.extent.start.offset, a1, v.encode()))
                else:
                    hedits.append((a0, a1, ('&' + v).encode()))
        for c in cgast.kids(n):
            # implicit conversions sit between a use and its real parent
            walk(c, parent if n.kind == K.UNEXPOSED_EXPR else n)

    walk(hbody, None)
    decl_lines = []
    for p, atext in decls:
        ptext = ht.b[p.extent.start.offset:p.extent.end.offset].decode()
        if '[' in ptext:
            ty = p.type.spelling
            ptext = (ty if ty.endswith('*') else ty + ' ') + p.spelling
        if p.spelling in rename:
            ptext = re.sub(r'\b' + re.escape(p.spelling) + r'$', rename[p.spelling], ptext)
        decl_lines.append(f'{ptext} = {atext};\n')
    decls = decl_lines

    if value and braces and stmt.kind == K.DECL_STMT:
        raise Refuse('layout: a declaration under a case label would lose its scope')

    # ---- the inlined text ------------------------------------------------------------
    b = cgast.body(hfn)
    lo, hi = b.extent.start.offset + 1, b.extent.end.offset - 1
    body = ht.render(lo, hi, hedits).decode().strip('\n') + '\n'
    body = '\n'.join(ln for ln in body.split('\n')).strip('\n')
    lines = body.split('\n')
    while lines and not lines[0].strip():
        lines.pop(0)
    body = '\n'.join(lines) + '\n' if lines else ''
    if decls:
        body = ''.join(' ' * 4 + d for d in decls) + '\n' + body
    sa, sb = stmt_span(ct, stmt)
    if ct.b[ct.line_start(sa):sa].strip():
        raise Refuse('layout: the statement does not start its line')
    ind = ct.indent_at(sa)
    comment = ''
    trail = ct.rest_of_line(sb)
    if trail and not (trail.startswith('/*') and trail.endswith('*/')):
        raise Refuse('layout: code after the statement')
    hc = cgast.header_comment(hlines, hfn.extent.start.line)
    proto_edits, proto_comments = prototypes(htu, ctu, helper)
    if not hc:
        hc = next((c for c in proto_comments if c), '')
    if not hc:
        # a comment on the definition line: `static int f(int n) /* ... */`
        dl = hlines[hfn.extent.start.line - 1]
        mm = re.search(r'\)\s*(/\*.*\*/)\s*$', dl)
        if mm:
            hc = mm.group(1) + '\n'
    if hc:
        comment += reindent(hc, ind - (len(hc) - len(hc.lstrip(' '))))
    inner = ind + 4 if braces else ind
    block = reindent(body, inner - 4)

    cedits = []
    if value:
        r = ht.render(rvalue[0], rvalue[1], hedits).decode()
        r = strip_parens(r)
        if parens and not SIMPLE.match(r):
            r = '(' + r + ')'
        stmt_text = ct.render(stmt.extent.start.offset, sb,
                              [(call.extent.start.offset, call.extent.end.offset, r.encode())]).decode()
        if trail:
            stmt_text += ' ' + trail
        stmt_text = ' ' * inner + stmt_text + '\n'
        if merge_into:
            stmt_text = (' ' * inner + trail + '\n') if trail else ''
        new = comment + (' ' * ind + '{\n' if braces else '') + block + stmt_text \
            + (' ' * ind + '}\n' if braces else '')
    else:
        if trail:
            comment = ' ' * ind + trail + '\n' + comment
        new = comment + (' ' * ind + '{\n' + block + ' ' * ind + '}\n' if braces else block)
    a = ct.line_start(sa)
    e = ct.line_end(sb)
    cedits.append((a, e, new.encode()))

    # ---- delete the helper (comment, definition, one blank line) and prototypes ----
    d0 = hfn.extent.start.offset
    hc_def = cgast.header_comment(hlines, hfn.extent.start.line)
    if hc_def:
        d0 = ht.line_start(d0)
        for _ in range(hc_def.count('\n')):
            d0 = ht.line_start(d0 - 1)
    hdel = full_lines(ht, ht.line_start(d0), hfn.extent.end.offset)
    file_edits = {}
    file_edits.setdefault(cfile, []).extend(cedits)
    file_edits.setdefault(hfile, []).append(hdel + (b'',))
    for fp, ed in proto_edits:
        file_edits.setdefault(fp, []).append(ed)

    texts = {fp: (Text(fp) if fp not in (hfile, cfile) else (ht if fp == hfile else ct))
             for fp in file_edits}
    out = {}
    for fp, eds in file_edits.items():
        t = texts[fp]
        out[fp] = (t.b.decode(), t.render(0, len(t.b), eds).decode())

    # ---- rule 4 on the merged caller ---------------------------------------------
    ntu = cgast.parse(cfile, text=out[cfile][1])
    nf = {fn.spelling: fn for fn in cgast.functions(ntu, cfile)}
    nl = out[cfile][1].splitlines(True)
    if caller not in nf:
        raise Refuse('internal: caller lost after the edit')
    n = cgast.body_code_lines(nl, nf[caller])
    if n > max_lines:
        raise Refuse(f'too-long: {caller} would have {n} code lines (> {max_lines})')
    depth = max_depth(nf[caller])
    if depth > MAX_DEPTH:
        raise Refuse(f'too-deep: {caller} would nest {depth} blocks deep (> {MAX_DEPTH})')
    return out, n


MAX_DEPTH = 5  # nested blocks below the function body (readability; Uncrustify stops at 9 levels)


def max_depth(fn, d=0):
    """Deepest nesting of compound statements below the function body."""
    best = d
    for c in fn.get_children():
        nd = d + 1 if c.kind == K.COMPOUND_STMT else d
        best = max(best, max_depth(c, nd))
    return best - 1 if d == 0 and best > 0 else best


def prototypes(htu, ctu, helper):
    edits, comments = [], []
    seen = set()
    for tu in (htu, ctu):
        for c in tu.cursor.walk_preorder():
            if c.kind != K.FUNCTION_DECL or c.spelling != helper or c.is_definition():
                continue
            f = c.location.file
            if f is None:
                continue
            fp = os.path.relpath(os.path.realpath(f.name), ROOT)
            if fp.startswith('include/'):
                raise Refuse('declared in a public header')
            if not fp.startswith('src/') or (fp, c.extent.start.offset) in seen:
                continue
            seen.add((fp, c.extent.start.offset))
            t = Text(fp)
            with open(fp, encoding='utf-8') as fh:
                fl = fh.readlines()
            com = cgast.header_comment(fl, c.extent.start.line)
            a = t.line_start(c.extent.start.offset)
            for _ in range(com.count('\n')):
                a = t.line_start(a - 1)
            b = t.semi_after(c.extent.end.offset)
            edits.append((fp, full_lines(t, a, b) + (b'',)))
            comments.append(com)
    return edits, comments


def includers(header):
    out = []
    for dp, _, fs in os.walk('src'):
        for f in fs:
            if not f.endswith('.c'):
                continue
            c = os.path.join(dp, f)
            with open(c, encoding='utf-8', errors='replace') as fh:
                for m in re.finditer(r'#include "([^"]+)"', fh.read()):
                    if os.path.normpath(os.path.join(dp, m.group(1))) == os.path.normpath(header):
                        out.append(c)
                        break
    return out


def compile_file(path):
    r = subprocess.run(['gcc'] + CFLAGS + ['-c', '-o', os.devnull, path],
                       capture_output=True, text=True)
    return r.returncode == 0, r.stderr


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument('helper')
    ap.add_argument('caller')
    ap.add_argument('--file')
    ap.add_argument('--caller-file')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--quiet', action='store_true', help='no diff')
    ap.add_argument('--max-lines', type=int, default=60,
                    help='refuse when the merged caller would exceed this (rule 4)')
    opt = ap.parse_args(argv[1:])
    os.chdir(ROOT)
    try:
        hfile, cfile = lookup_files(opt.helper, opt.caller, opt.file, opt.caller_file)
        changes, n = build(hfile, cfile, opt.helper, opt.caller, opt.max_lines)
    except Refuse as e:
        print(f'REFUSED {opt.helper} -> {opt.caller}: {e}')
        return 2
    if not opt.quiet:
        for fp, (old, new) in changes.items():
            sys.stdout.writelines(difflib.unified_diff(old.splitlines(True), new.splitlines(True),
                                                       'a/' + fp, 'b/' + fp))
    if opt.dry_run:
        print(f'DRY-RUN {opt.helper} -> {opt.caller}: {n} code lines')
        return 0
    for fp, (_, new) in changes.items():
        with open(fp, 'w', encoding='utf-8') as fh:
            fh.write(new)
    todo = [fp for fp in changes if fp.endswith('.c')]
    for fp in changes:
        if fp.endswith('.h'):
            todo += includers(fp)  # every TU that includes a changed private header
    for fp in sorted(set(todo)):
        if True:
            ok, err = compile_file(fp)
            if not ok:
                for fp2, (old, _) in changes.items():
                    with open(fp2, 'w', encoding='utf-8') as fh:
                        fh.write(old)
                print(f'REVERTED {opt.helper} -> {opt.caller}: compile failed in {fp}\n{err}')
                return 1
    print(f'INLINED {opt.helper} -> {opt.caller} in {cfile} ({n} code lines) '
          f'files: {" ".join(sorted(changes))}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
