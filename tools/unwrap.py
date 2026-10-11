#!/usr/bin/env python3
"""unwrap.py -- find and remove forwarding wrappers.

Usage (from the repo root):
    python3 -I tools/unwrap.py --list
    python3 -I tools/unwrap.py WRAPPER --file src/x.c [--move N]... [--dry-run]

Rule (user direction, 2026-10-10): a function in src/ (not src/sim/) whose
body is only
  (a) zero or more `if (!AGC_ASSERT(c)) { return ...; }` guards, and
  (b) exactly one statement that calls another function with its own
      parameters as the arguments (in any order, each possibly cast, so a
      wrapper that only renames them counts too), optionally returning
      that call's value,
is a forwarding wrapper, whatever its caller count.  It is deleted and
every call is redirected to the callee (the casts go with the arguments).

Guards: a wrapper guard whose conjuncts the callee already asserts on the
same parameters (structural AST comparison; a callee guard that calls a
one-line predicate `return (expr);` of its own file is expanded first) is
dropped.  Any other guard is listed by --list and the call is refused until
the caller decides: --move N inserts guard N into the callee once (its text
with the parameters renamed, returning what the callee's guards return);
--drop N drops it (the callee asserts the fact in another form, which the
note must say).

Kept (reported by --list, never touched): wrappers declared in a public
header (include/agc: other lanes link against them; most are the dispatch
registry's job/task entry points), referenced from tests/ or src/sim/, a
void wrapper around a non-void callee (the result would be dropped), a
return type that differs from the callee's, a parameter forwarded twice or
not at all, or a file-private callee with callers in other files.

After the edit every changed .c and every includer of a changed header is
compiled with the Makefile flags; all files are restored if one fails.
"""
import argparse
import difflib
import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import cgast  # noqa: E402
import inline_one as io1  # noqa: E402

K = cgast.K


class Refuse(Exception):
    pass


def lib_files():
    return sorted(f for f in glob.glob('src/**/*.c', recursive=True) if not f.startswith('src/sim/'))


def forwarded_call(fn):
    """(guards, stmt, call, mapping) when fn is a forwarding wrapper, else None.
    mapping[j] = (wrapper param index, cast text or None) for callee arg j."""
    b = cgast.body(fn)
    if b is None:
        return None
    st = cgast.kids(b)
    guards = [s for s in st if cgast.is_guard(s) is not None]
    rest = [s for s in st if cgast.is_guard(s) is None]
    if len(rest) != 1:
        return None
    s = rest[0]
    if s.kind == K.RETURN_STMT and st[-1] != s:
        return None
    e = s
    if s.kind == K.RETURN_STMT:
        ch = cgast.kids(s)
        if not ch:
            return None
        e = ch[0]
    e = cgast.strip(e)
    if e.kind == K.CSTYLE_CAST_EXPR and s.kind == K.RETURN_STMT:
        e = cgast.strip(cgast.kids(e)[-1])  # `return ((T) f(..))`: the type check below keeps it
    if e.kind != K.CALL_EXPR or e.spelling == 'AGC_ASSERT' or e.referenced is None:
        return None
    pidx = {cgast.ref_key(p): i for i, p in enumerate(cgast.params(fn))}
    mapping = []
    for a in cgast.kids(e)[1:]:
        a2 = cgast.strip(a)
        cast = None
        if a2.kind == K.CSTYLE_CAST_EXPR:
            cast = a2.type.spelling
            a2 = cgast.strip(cgast.kids(a2)[-1])
        if a2.kind != K.DECL_REF_EXPR or a2.referenced is None \
                or cgast.ref_key(a2.referenced) not in pidx:
            return None
        mapping.append((pidx[cgast.ref_key(a2.referenced)], cast))
    return guards, s, e, mapping


def public_names():
    names = set()
    for h in sorted(glob.glob('include/agc/**/*.h', recursive=True)):
        tu = cgast.parse(h, ['-x', 'c'])
        for c in tu.cursor.get_children():
            if c.kind == K.FUNCTION_DECL and cgast.in_file(c, h):
                names.add(c.spelling)
    return names


def outside_refs(name):
    """Files in tests/ or src/sim/ that mention name (textual: these trees
    are other lanes' and are not parsed here)."""
    pat = re.compile(r'\b' + re.escape(name) + r'\b')
    out = []
    for f in glob.glob('tests/**/*.[ch]', recursive=True) + glob.glob('src/sim/**/*.[ch]', recursive=True):
        with open(f, encoding='utf-8', errors='replace') as fh:
            if pat.search(fh.read()):
                out.append(f)
    return out


_EXTERN = None


def extern_file(name):
    """The src/ file defining extern function name (docs/overview/callgraph.tsv)."""
    global _EXTERN
    if _EXTERN is None:
        _EXTERN = {}
        for row in io1._tsv(os.path.join('docs', 'overview', 'callgraph.tsv')):
            if row['static'] == 'extern':
                _EXTERN[row['callee']] = row['file']
    return _EXTERN.get(name)


def predicate_body(fn):
    """expr cursor if fn's body is `return (expr);`."""
    b = cgast.body(fn)
    if b is None:
        return None
    st = cgast.kids(b)
    if len(st) == 1 and st[0].kind == K.RETURN_STMT and cgast.kids(st[0]):
        return cgast.kids(st[0])[0]
    return None


def callee_facts(callee, tu, path):
    """Conjuncts the callee asserts in its top-level guards (callee-param space)."""
    ctx = cgast.Ctx(callee, cgast.macros_of(path))
    preds = {f.spelling: f for f in cgast.functions(tu, path)}
    facts = []
    for s in cgast.kids(cgast.body(callee)):
        terms = cgast.is_guard(s)
        if terms is None:
            continue
        for t in terms:
            for f in cgast.conjuncts(cgast.canon(t, ctx)):
                facts.append(f)
                # a predicate call of this file: also its body, parameters substituted
                if f[0] == 'call' and f[1] in preds:
                    pb = predicate_body(preds[f[1]])
                    if pb is not None:
                        inner = cgast.canon(pb, cgast.Ctx(preds[f[1]], ctx.macros))
                        for g in cgast.conjuncts(cgast.subst(inner, list(f[2:]))):
                            facts.append(g)
    return facts


def analyse(w, wtu, wpath, public):
    """Decision record for one wrapper."""
    fc = forwarded_call(w)
    if fc is None:
        return None
    guards, stmt, call, mapping = fc
    callee = call.referenced
    rec = {'_call': call, 'name': w.spelling, 'file': wpath, 'line': w.location.line, 'callee': callee.spelling,
           'static': w.storage_class == cgast.ci.StorageClass.STATIC, 'keep': None,
           'guards': [], 'mapping': mapping, 'callee_static':
           callee.storage_class == cgast.ci.StorageClass.STATIC}
    params = cgast.params(w)
    used = [m[0] for m in mapping]
    if w.spelling in public:
        rec['keep'] = 'public API in include/agc (other lanes link against it)'
    elif sorted(used) != list(range(len(params))):
        rec['keep'] = 'a parameter is forwarded twice or not at all (not a pure forward)'
    elif stmt.kind != K.RETURN_STMT and callee.result_type.spelling != 'void':
        rec['keep'] = 'void wrapper drops the callee\'s result'
    elif stmt.kind == K.RETURN_STMT and w.result_type.spelling != callee.result_type.spelling:
        rec['keep'] = (f'return type {w.result_type.spelling} differs from the callee\'s '
                       f'{callee.result_type.spelling}')
    else:
        refs = outside_refs(w.spelling)
        if refs:
            rec['keep'] = 'referenced from ' + ', '.join(sorted(refs)[:3])
    # guards: which does the callee already assert?
    cdef = callee.get_definition()
    cfacts = []
    cpath = None
    if cdef is not None and cdef.location.file is not None:
        cpath = os.path.relpath(os.path.realpath(cdef.location.file.name), ROOT)
        if cpath == wpath:
            cfacts = callee_facts(cdef, wtu, wpath)
        else:
            ctu = cgast.parse(cpath)
            cd = {f.spelling: f for f in cgast.functions(ctu, cpath)}.get(callee.spelling)
            cfacts = callee_facts(cd, ctu, cpath) if cd is not None else []
    if cpath is None:
        cpath = extern_file(callee.spelling)
        if cpath is not None:
            ctu = cgast.parse(cpath)
            cd = {f.spelling: f for f in cgast.functions(ctu, cpath)}.get(callee.spelling)
            cfacts = callee_facts(cd, ctu, cpath) if cd is not None else []
    rec['callee_file'] = cpath
    wctx = cgast.Ctx(w, cgast.macros_of(wpath))
    to_callee = {}
    for j, (i, _) in enumerate(mapping):
        to_callee.setdefault(i, ('param', j))
    wt = io1.Text(wpath)
    for g in guards:
        facts = cgast.guard_facts(g, wctx)
        mapped = [cgast.subst(f, [to_callee.get(i, ('param', 99)) for i in range(len(params))])
                  for f in facts]
        missing = [f for f in mapped if f not in cfacts]
        cond = cgast.is_guard(g)
        text = ' && '.join(wt.b[t.extent.start.offset:t.extent.end.offset].decode() for t in cond)
        rec['guards'].append({'cursor': g, 'dup': not missing, 'text': text,
                              'cond': cond})
    return rec


def scan(only=None):
    public = public_names()
    out = []
    for f in lib_files():
        tu = cgast.parse(f)
        for fn in cgast.functions(tu, f):
            if only and (fn.spelling != only[0] or f != only[1]):
                continue
            r = analyse(fn, tu, f, public)
            if r is not None:
                r['_tu'] = tu
                r['_fn'] = fn
                out.append(r)
    return out


def callers_of(rec):
    """[(file, [call cursors])] of every call of the wrapper in src/ (not sim)."""
    files = [rec['file']] if rec['static'] else lib_files()
    out = []
    pat = re.compile(r'\b' + re.escape(rec['name']) + r'\s*\(')
    for f in files:
        with open(f, encoding='utf-8', errors='replace') as fh:
            text = fh.read()
        if not pat.search(text):
            continue
        tu = rec['_tu'] if f == rec['file'] else cgast.parse(f)
        calls = []
        for fn in cgast.functions(tu, f):
            if fn.spelling == rec['name']:
                continue
            for c in fn.walk_preorder():
                if c.kind == K.CALL_EXPR and c.spelling == rec['name']:
                    calls.append(c)
        if calls:
            out.append((f, calls, tu))
    return out


def build(rec, moves, drops):
    if rec['keep']:
        raise Refuse('kept: ' + rec['keep'])
    for k, g in enumerate(rec['guards']):
        if not g['dup'] and k not in moves and k not in drops:
            raise Refuse(f'guard {k} `{g["text"]}` is not asserted by {rec["callee"]}: '
                         f'pass --move {k} or --drop {k}')
    sites = callers_of(rec)
    edits = {}
    if rec['callee_static']:
        for f, _, _ in sites:
            if f != rec['callee_file']:
                raise Refuse(f'callee {rec["callee"]} is file-private and {f} calls the wrapper')
    for f, calls, _ in sites:
        t = io1.Text(f)
        for c in calls:
            args = cgast.kids(c)[1:]
            name = cgast.kids(c)[0]
            texts = [t.b[a.extent.start.offset:a.extent.end.offset].decode() for a in args]
            ident = all(i == j and cast is None for j, (i, cast) in enumerate(rec['mapping'])) \
                and len(args) == len(rec['mapping'])
            if ident:
                n0 = name.extent.start.offset
                edits.setdefault(f, []).append((n0, n0 + len(rec['name']), rec['callee'].encode()))
                continue
            new = []
            for i, cast in rec['mapping']:
                a = texts[i]
                if cast:
                    a = f'({cast}) ' + (a if io1.SIMPLE.match(a) else f'({a})')
                new.append(a)
            edits.setdefault(f, []).append((c.extent.start.offset, c.extent.end.offset,
                                            f'{rec["callee"]}({", ".join(new)})'.encode()))
    # moved guards: into the callee, after its own guards
    for k in moves:
        if rec['guards'][k]['cursor'].extent.start.offset > rec['_call'].extent.start.offset:
            raise Refuse(f'guard {k} is a post-condition of the call; it cannot open the callee')
    if moves:
        cpath = rec['callee_file']
        ctu = rec['_tu'] if cpath == rec['file'] else cgast.parse(cpath)
        cfn = {f.spelling: f for f in cgast.functions(ctu, cpath)}[rec['callee']]
        ct = io1.Text(cpath)
        rec_ret = io1.recovery(ct, cfn)
        cparams = [p.spelling for p in cgast.params(cfn)]
        wparams = [p.spelling for p in cgast.params(rec['_fn'])]
        ren = {}
        for j, (i, _) in enumerate(rec['mapping']):
            ren.setdefault(wparams[i], cparams[j])
        st = cgast.kids(cgast.body(cfn))
        last_guard = None
        for s in st:
            if cgast.is_guard(s) is None:
                break
            last_guard = s
        at = (io1.full_lines(ct, last_guard.extent.start.offset, last_guard.extent.end.offset)[1]
              if last_guard is not None else ct.line_end(cgast.body(cfn).extent.start.offset))
        ins = ''
        for k in sorted(moves):
            cond = re.sub(r'\b(' + '|'.join(map(re.escape, ren)) + r')\b',
                          lambda m: ren[m.group(1)], rec['guards'][k]['text'])
            ins += (f'    if (!AGC_ASSERT({cond}))\n    {{\n        {rec_ret}\n    }}\n\n')
        edits.setdefault(cpath, []).append((at, at, ins.encode()))
    # delete the wrapper and its prototypes
    w = rec['_fn']
    wt = io1.Text(rec['file'])
    with open(rec['file'], encoding='utf-8') as fh:
        wl = fh.readlines()
    d0 = wt.line_start(w.extent.start.offset)
    hc = cgast.header_comment(wl, w.extent.start.line)
    for _ in range(hc.count('\n')):
        d0 = wt.line_start(d0 - 1)
    a, b = io1.full_lines(wt, d0, w.extent.end.offset)
    edits.setdefault(rec['file'], []).append((a, b, b''))
    pe, _ = io1.prototypes(rec['_tu'], rec['_tu'], rec['name'])
    for fp, ed in pe:
        edits.setdefault(fp, []).append(ed)
    out = {}
    for fp, eds in edits.items():
        t = io1.Text(fp)
        out[fp] = (t.b.decode(), t.render(0, len(t.b), eds).decode())
    return out


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument('wrapper', nargs='?')
    ap.add_argument('--file')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--move', type=int, action='append', default=[])
    ap.add_argument('--drop', type=int, action='append', default=[])
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--quiet', action='store_true')
    opt = ap.parse_args(argv[1:])
    os.chdir(ROOT)
    if opt.list:
        for r in scan():
            g = '; '.join(f'[{k}]{"dup" if x["dup"] else "NEW"}: {x["text"]}'
                          for k, x in enumerate(r['guards']))
            print('\t'.join([r['name'], f'{r["file"]}:{r["line"]}', r['callee'],
                             'KEEP: ' + r['keep'] if r['keep'] else 'remove', g or '-']))
        return 0
    try:
        recs = scan((opt.wrapper, opt.file))
        if not recs:
            raise Refuse(f'{opt.wrapper} in {opt.file} is not a forwarding wrapper')
        changes = build(recs[0], set(opt.move), set(opt.drop))
    except Refuse as e:
        print(f'REFUSED {opt.wrapper}: {e}')
        return 2
    if not opt.quiet:
        for fp, (old, new) in changes.items():
            sys.stdout.writelines(difflib.unified_diff(old.splitlines(True), new.splitlines(True),
                                                       'a/' + fp, 'b/' + fp))
    if opt.dry_run:
        return 0
    for fp, (_, new) in changes.items():
        with open(fp, 'w', encoding='utf-8') as fh:
            fh.write(new)
    todo = [fp for fp in changes if fp.endswith('.c')]
    for fp in changes:
        if fp.endswith('.h'):
            todo += io1.includers(fp)
    for fp in sorted(set(todo)):
        ok, err = io1.compile_file(fp)
        if not ok:
            for fp2, (old, _) in changes.items():
                with open(fp2, 'w', encoding='utf-8') as fh:
                    fh.write(old)
            print(f'REVERTED {opt.wrapper}: compile failed in {fp}\n{err}')
            return 1
    print(f'REMOVED {opt.wrapper} -> {recs[0]["callee"]} files: {" ".join(sorted(changes))}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
