#!/usr/bin/env python3
"""guards.py -- classify every AGC_ASSERT guard (docs/agents/status-handling-2.md).

Usage (repo root):
    python3 -I tools/prove/guards.py index      # libclang index -> build/prove/index.pkl
    python3 -I tools/prove/guards.py classify   # + EVA verdicts -> docs/overview/guards.tsv

A guard is `if (!AGC_ASSERT(c) [|| !AGC_ASSERT(d)]) { ...; return ...; }`.
Each conjunct of its condition is a fact.  A fact is GUARANTEED when

  * Frama-C EVA proves the assertion VALID on every path of every entry
    point that reaches it (tools/prove/eva_run.py, build/prove/
    eva_verdicts.tsv), or
  * it is a fact about the function's own parameters and every call site
    in src/ and tests/ satisfies it structurally: the argument is `&x`, an
    array or a string (non-NULL), a constant expression that evaluates to
    true, a fact the caller established before the call (an earlier guard,
    the condition of an enclosing `if`, a `switch` case, the bound of an
    enclosing counted `for`), or a fact about the caller's own parameters
    that is in turn guaranteed at every call site of the caller (fixpoint
    over the acyclic call graph; rule 1 forbids recursion).

Everything else is FALLIBLE (or UNCALLED when the function has no caller).
The why column names the evidence.
"""
import collections
import glob
import os
import pickle
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import cgast  # noqa: E402

K = cgast.K
ci = cgast.ci
OUT = os.path.join(ROOT, 'build', 'prove')
INDEX = os.path.join(OUT, 'index.pkl')
GLOBALS = {}
ENUM_CONSTS = {}
ENUM_TYPES = {}


def c_files():
    out = []
    for top in ('src', 'tests'):
        for dp, _, files in os.walk(os.path.join(ROOT, top)):
            out += [os.path.relpath(os.path.join(dp, f), ROOT) for f in files if f.endswith('.c')]
    return sorted(out)


def walk(n):
    yield n
    for c in n.get_children():
        yield from walk(c)


def guard_then_return(stmt):
    """The guard's return statement (then-branch = [stmts;] return)."""
    ch = cgast.kids(stmt)
    if stmt.kind != K.IF_STMT or len(ch) != 2:
        return None
    then = ch[1]
    if then.kind == K.COMPOUND_STMT:
        tk = cgast.kids(then)
        if not tk:
            return None
        then = tk[-1]
    return then if then.kind == K.RETURN_STMT else None


def src_text(n):
    f = n.extent.start.file
    if f is None:
        return ''
    return cgast._src(f.name)[n.extent.start.offset:n.extent.end.offset].decode('utf-8', 'replace')


def loop_facts(loop, ctx):
    """Facts a counted `for (T i = a; i < b; i++)` body may assume."""
    ch = cgast.kids(loop)
    if loop.kind != K.FOR_STMT or len(ch) < 4:
        return []
    init, cond, inc = ch[0], ch[1], ch[2]
    var = None
    lo = None
    if init.kind == K.DECL_STMT:
        vd = cgast.kids(init)
        if len(vd) == 1 and vd[0].kind == K.VAR_DECL:
            vk = cgast.kids(vd[0])
            if vk:
                var = ('ref', cgast.ref_key(vd[0]), 'L', vd[0].spelling)
                lo = cgast.canon(vk[-1], ctx)
    if var is None:
        return []
    c = cgast.canon(cond, ctx)
    op = cgast.op_of(inc) if inc.kind == K.UNARY_OPERATOR else None
    facts = []
    if c[0] == 'bin' and c[2] == var and c[1] in ('<', '<=') and op in ('++', 'post++'):
        facts.append(c)
        if lo is not None:
            facts.append(('bin', '>=', var, lo))
    return facts


def established(fn, call, ctx):
    """cgast.established_at plus enclosing if/switch/for facts."""
    chain = cgast.path_to(cgast.body(fn), call)
    if chain is None:
        return []
    est = []

    def apply(eff):
        est[:] = [f for f in est if not cgast.invalidated(f, eff)]

    def add(fs):
        est.extend(f for f in fs if f not in est)

    sw = []
    for parent, child in zip(chain, chain[1:]):
        if parent.kind == K.COMPOUND_STMT:
            for s in cgast.kids(parent):
                if s == child:
                    break
                facts = cgast.guard_facts(s, ctx)
                if facts is None and s.kind == K.IF_STMT and guard_then_return(s) is not None:
                    terms = cgast._assert_terms(cgast.kids(s)[0])
                    if terms is not None:
                        facts = [f for t in terms for f in cgast.conjuncts(cgast.canon(t, ctx))]
                if facts is not None:
                    add(facts)
                elif s.kind == K.IF_STMT and guard_then_return(s) is not None and len(cgast.kids(s)) == 2:
                    # if (c) { ...; return; }  -> !c afterwards
                    add([('un', '!', cgast.canon(cgast.kids(s)[0], ctx))])
                else:
                    apply(cgast.summarize(s, ctx))
        elif parent.kind in cgast.LOOPS:
            apply(cgast.summarize(parent, ctx))
            body = cgast.kids(parent)[-1]
            if child == body:
                add(loop_facts(parent, ctx))
        elif parent.kind == K.SWITCH_STMT:
            apply(cgast.summarize(parent, ctx))
            sw.append(cgast.canon(cgast.kids(parent)[0], ctx))
        elif parent.kind == K.CASE_STMT and sw:
            ch = cgast.kids(parent)
            if ch and child != ch[0]:
                add([('bin', '==', sw[-1], cgast.canon(ch[0], ctx))])
        elif parent.kind == K.IF_STMT:
            ch = cgast.kids(parent)
            cond = ch[0]
            if child != cond:
                apply(cgast.summarize(cond, ctx))
                c = cgast.canon(cond, ctx)
                if len(ch) >= 2 and child == ch[1]:
                    add(cgast.conjuncts(c))
                elif len(ch) == 3 and child == ch[2]:
                    add([('un', '!', c)])
        elif parent.kind == K.BINARY_OPERATOR and cgast.op_of(parent) == '&&':
            ch = cgast.kids(parent)
            if child == ch[1]:
                add(cgast.conjuncts(cgast.canon(ch[0], ctx)))
        elif parent.kind == K.CONDITIONAL_OPERATOR:
            ch = cgast.kids(parent)
            c = cgast.canon(ch[0], ctx)
            if child == ch[1]:
                add(cgast.conjuncts(c))
            elif child == ch[2]:
                add([('un', '!', c)])
    return est


def arg_info(a):
    """Shape of a call argument: nonnull / array names."""
    a = cgast.strip(a)
    while a.kind == K.CSTYLE_CAST_EXPR:
        a = cgast.strip(cgast.kids(a)[-1])
    if a.kind == K.UNARY_OPERATOR and cgast.op_of(a) == '&':
        return 'addr'
    if a.kind == K.STRING_LITERAL:
        return 'addr'
    if a.type.kind in (ci.TypeKind.CONSTANTARRAY, ci.TypeKind.INCOMPLETEARRAY):
        return 'addr'
    if a.kind in (K.DECL_REF_EXPR, K.MEMBER_REF_EXPR, K.ARRAY_SUBSCRIPT_EXPR):
        if a.type.get_canonical().kind in (ci.TypeKind.CONSTANTARRAY, ci.TypeKind.INCOMPLETEARRAY):
            return 'addr'
    if a.kind == K.CALL_EXPR:
        return 'call:' + a.spelling
    return ''


def static_root(n):
    """Key of the file-scope variable an lvalue is rooted in (through `.`
    and [] only), else None."""
    n = cgast.strip(n)
    while True:
        if n.kind == K.MEMBER_REF_EXPR:
            ch = cgast.kids(n)
            if not ch or cgast.is_arrow(n, ch[0]):
                return None
            n = cgast.strip(ch[0])
        elif n.kind == K.ARRAY_SUBSCRIPT_EXPR:
            n = cgast.strip(cgast.kids(n)[0])
        elif n.kind == K.DECL_REF_EXPR and n.referenced is not None:
            r = n.referenced
            if r.kind == K.VAR_DECL and cgast.ref_class(r) == 'G':
                return cgast.ref_key(r)
            return None
        else:
            return None


def index_enums(tu):
    for c in walk(tu.cursor):
        if c.kind == K.ENUM_DECL:
            vals = [e.enum_value for e in c.get_children() if e.kind == K.ENUM_CONSTANT_DECL]
            for e in c.get_children():
                if e.kind == K.ENUM_CONSTANT_DECL:
                    ENUM_CONSTS[cgast.ref_key(e)] = e.enum_value
            if vals:
                ENUM_TYPES[c.type.spelling] = (min(vals), max(vals))
                ENUM_TYPES[c.type.get_canonical().spelling] = (min(vals), max(vals))
        elif c.kind == K.TYPEDEF_DECL:
            u = c.underlying_typedef_type.get_canonical()
            if u.kind == ci.TypeKind.ENUM:
                vals = [e.enum_value for e in u.get_declaration().get_children()
                        if e.kind == K.ENUM_CONSTANT_DECL]
                if vals:
                    ENUM_TYPES[c.spelling] = (min(vals), max(vals))


def index_globals(tu, path, macros):
    """File-scope variables: key -> initial value canon (None: zero)."""
    out = {}
    ctx = cgast.Ctx(None, macros)
    index_enums(tu)
    for c in tu.cursor.get_children():
        if c.kind == K.VAR_DECL and cgast.in_file(c, os.path.join(ROOT, path)):
            ch = [k for k in cgast.kids(c) if k.kind not in (K.TYPE_REF,)]
            init = None
            if ch:
                last = ch[-1]
                init = 'list' if last.kind == K.INIT_LIST_EXPR else cgast.canon(last, ctx)
            out[cgast.ref_key(c)] = {'name': c.spelling, 'init': init, 'file': path,
                                     'type': c.type.spelling}
    return out


def tmap_of(n, ctx, out=None):
    """canon -> declared type spelling for every subexpression of n."""
    out = {} if out is None else out
    for x in walk(n):
        if x.kind.is_expression() and x.kind != K.CALL_EXPR or x.kind in (K.CALL_EXPR, K.DECL_REF_EXPR, K.MEMBER_REF_EXPR):
            try:
                c = cgast.canon(x, ctx)
            except Exception:
                continue
            ty = x.type.spelling
            if ty and c not in out:
                out[c] = ty
    return out


def index_file(path):
    extra = ['-DAGC_HOST_DEBUG'] if path.startswith('tests/') else []
    tu = cgast.parse(os.path.join(ROOT, path), extra)
    macros = cgast.macros_of(os.path.join(ROOT, path))
    funcs = []
    GLOBALS.update(index_globals(tu, path, macros))
    for fn in cgast.functions(tu, os.path.join(ROOT, path)):
        ctx = cgast.Ctx(fn, macros)
        b = cgast.body(fn)
        rt = fn.result_type.spelling
        rec = {'usr': fn.get_usr(), 'name': fn.spelling, 'file': path, 'line': fn.location.line,
               'end': fn.extent.end.line,
               'static': fn.storage_class == ci.StorageClass.STATIC, 'ret': rt,
               'params': [(p.spelling, p.type.spelling) for p in cgast.params(fn)],
               'guards': [], 'calls': [], 'returns': [], 'body_stmts': 0, 'writes': [],
               'decays': [], 'subscripted': [], 'pred': None}
        if b is None:
            continue
        rec['body_stmts'] = len(cgast.kids(b))
        safe = set()  # &g / g[] passed to a const-pointer parameter: read only
        for n in walk(b):
            if n.kind == K.IF_STMT:
                ret = guard_then_return(n)
                if ret is None:
                    continue
                terms = cgast._assert_terms(cgast.kids(n)[0])
                if terms is None:
                    continue
                facts = []
                for t in terms:
                    for f in cgast.conjuncts(cgast.canon(t, ctx)):
                        facts.append((f, src_text(t)))
                rec['guards'].append({'line': n.location.line, 'end': n.extent.end.line,
                                      'facts': facts, 'ret': src_text(ret),
                                      'tmap': tmap_of(cgast.kids(n)[0], ctx),
                                      'top': n.semantic_parent is not None and any(
                                          c == n for c in cgast.kids(b))})
            elif n.kind == K.CALL_EXPR and n.referenced is not None and n.spelling != 'AGC_ASSERT':
                args = cgast.kids(n)[1:]
                ptypes = [a.type for a in n.referenced.get_arguments()] if n.referenced.kind == K.FUNCTION_DECL else []
                for a, pt in zip(args, ptypes):
                    if pt.kind == ci.TypeKind.POINTER and pt.get_pointee().is_const_qualified():
                        x = cgast.strip(a)
                        while x.kind == K.CSTYLE_CAST_EXPR:
                            x = cgast.strip(cgast.kids(x)[-1])
                        safe.add((x.location.line, x.location.column))
                        if x.kind == K.UNARY_OPERATOR:
                            y = cgast.strip(cgast.kids(x)[0])
                            safe.add((y.location.line, y.location.column))
                rec['calls'].append({'callee': n.referenced.get_usr(), 'name': n.spelling,
                                     'line': n.location.line,
                                     'args': [cgast.canon(a, ctx) for a in args],
                                     'info': [arg_info(a) for a in args],
                                     'types': [a.type.spelling for a in args],
                                     'tmap': {k: v for a in args for k, v in tmap_of(a, ctx).items()},
                                     'est': established(fn, n, ctx)})
            elif n.kind == K.RETURN_STMT:
                ch = cgast.kids(n)
                if ch:
                    rec['returns'].append(arg_info(ch[0]))
            elif (n.kind in (K.BINARY_OPERATOR, K.COMPOUND_ASSIGNMENT_OPERATOR)
                  and cgast.op_of(n) in cgast.ASSIGN_OPS):
                lhs, rhs = cgast.kids(n)
                g = static_root(lhs)
                if g is not None:
                    op = cgast.op_of(n)
                    rec['writes'].append({'g': g, 'lhs': cgast.canon(lhs, ctx), 'line': n.location.line,
                                          'rhs': cgast.canon(rhs, ctx) if op == '=' else ('rmw', op),
                                          'tmap': tmap_of(rhs, ctx),
                                          'est': established(fn, n, ctx)})
            elif n.kind == K.UNARY_OPERATOR and cgast.op_of(n) in ('++', '--', 'post++', 'post--', '&'):
                g = static_root(cgast.kids(n)[0])
                if g is not None and not (cgast.op_of(n) == '&' and (n.location.line, n.location.column) in safe):
                    rec['writes'].append({'g': g, 'lhs': cgast.canon(cgast.kids(n)[0], ctx),
                                          'line': n.location.line,
                                          'rhs': ('rmw', cgast.op_of(n)), 'est': []})
            elif n.kind == K.DECL_REF_EXPR and n.type.kind in (ci.TypeKind.CONSTANTARRAY,):
                g = static_root(n)
                p = n.semantic_parent
                if g is not None:
                    # an array name used other than under [] decays: escape
                    rec['decays'].append((g, n.location.line, n.location.column))
            if n.kind == K.ARRAY_SUBSCRIPT_EXPR:
                base = cgast.strip(cgast.kids(n)[0])
                if base.kind == K.DECL_REF_EXPR:
                    rec['subscripted'].append((base.location.line, base.location.column))
        if len(cgast.kids(b)) == 1 and cgast.kids(b)[0].kind == K.RETURN_STMT and cgast.kids(cgast.kids(b)[0]):
            rec['pred'] = cgast.canon(cgast.kids(cgast.kids(b)[0])[0], ctx)
        rec['decays'] = [d for d in rec['decays'] if (d[1], d[2]) not in set(rec['subscripted'])
                         and (d[1], d[2]) not in safe]
        del rec['subscripted']
        funcs.append(rec)
    return funcs


def cmd_index():
    os.makedirs(OUT, exist_ok=True)
    allf = []
    files = c_files()
    for i, p in enumerate(files):
        allf += index_file(p)
        if i % 20 == 0:
            print(f'{i}/{len(files)} {p}', flush=True)
    with open(INDEX, 'wb') as fh:
        pickle.dump({'funcs': allf, 'globals': GLOBALS, 'enum_consts': ENUM_CONSTS,
                     'enum_types': ENUM_TYPES}, fh)
    print(f'{len(allf)} functions indexed')


# ---- classification -------------------------------------------------------

TYPE_RANGE = {
    'unsigned short': (0, 65535), 'short': (-32768, 32767), '_Bool': (0, 1),
    'unsigned char': (0, 255), 'signed char': (-128, 127), 'char': (-128, 127),
    'unsigned int': (0, 2 ** 32 - 1), 'int': (-2 ** 31, 2 ** 31 - 1),
}


def lit_val(t):
    if isinstance(t, tuple) and t and t[0] == 'lit' and isinstance(t[1], str) and t[1].startswith('#'):
        try:
            return int(t[1][1:])
        except ValueError:
            return None
    if isinstance(t, tuple) and t and t[0] == 'ref' and t[2] == 'C':
        return ENUMS.get(t[1])
    return None


ENUMS = {}
ENUM_TYPES = {}
NEG = {'<': '>=', '<=': '>', '>': '<=', '>=': '<', '==': '!=', '!=': '=='}


def negate(t):
    if isinstance(t, tuple) and t and t[0] == 'bin' and t[1] in NEG:
        return ('bin', NEG[t[1]], t[2], t[3])
    return ('un', '!', t)


def ev(t):
    """Integer value of a constant expression tree, else None."""
    v = lit_val(t)
    if v is not None:
        return v
    if not isinstance(t, tuple) or not t:
        return None
    if t[0] == 'cast':
        return ev(t[2])
    if t[0] == 'un':
        x = ev(t[2])
        if x is None:
            return None
        return {'-': -x, '+': x, '!': int(not x), '~': ~x}.get(t[1])
    if t[0] == 'bin':
        a, b = ev(t[2]), ev(t[3])
        if a is None or b is None:
            if t[1] == '&&' and (a == 0 or b == 0):
                return 0
            if t[1] == '||' and (a not in (None, 0) or b not in (None, 0)):
                return 1
            return None
        ops = {'<': a < b, '<=': a <= b, '>': a > b, '>=': a >= b, '==': a == b, '!=': a != b,
               '&&': bool(a and b), '||': bool(a or b), '+': a + b, '-': a - b, '*': a * b,
               '&': a & b, '|': a | b, '^': a ^ b,
               '<<': a << b if 0 <= b < 64 else None, '>>': a >> b if 0 <= b < 64 else None}
        r = ops.get(t[1])
        return None if r is None else int(r)
    if t[0] == '?:' and len(t) == 4:
        c = ev(t[1])
        if c is None:
            return None
        return ev(t[2]) if c else ev(t[3])
    return None


WORD_RANGE = {
    'agc_sp_t': (-0x3FFF, 0x3FFF), 'agc_dp_t': (-0x0FFFFFFF, 0x0FFFFFFF),
    'agc_tp_t': (-0x3FFFFFFFFFF, 0x3FFFFFFFFFF), 'agc_ovf_t': (-1, 1),
}
C_RANGE = dict(TYPE_RANGE)
C_RANGE.update({'uint16_t': (0, 65535), 'int16_t': (-32768, 32767), 'uint8_t': (0, 255),
                'int8_t': (-128, 127), 'uint32_t': (0, 2 ** 32 - 1), 'int32_t': (-2 ** 31, 2 ** 31 - 1),
                'bool': (0, 1), 'size_t': (0, 2 ** 64 - 1), 'unsigned long': (0, 2 ** 64 - 1)})
ATOMS = ('param', 'ref', 'mem', 'idx', 'call')
USED_INV = [False]


def type_range(t, types, inv):
    ty = types.get(t)
    if ty is None:
        return None
    ty = ty.replace('const ', '').strip()
    if inv and t[0] in ATOMS + ('un',):
        if ty in WORD_RANGE:
            USED_INV[0] = True
            return WORD_RANGE[ty]
        if ty in ENUM_TYPES:
            USED_INV[0] = True
            return ENUM_TYPES[ty]
    return C_RANGE.get(ty)


def rng(t, types, inv=False):
    """Interval of an expression; types: canon -> declared C type.  With inv,
    an atom of a word type (agc_sp_t ...) or an enum type takes the type's
    documented range (the type invariant)."""
    v = ev(t)
    if v is not None:
        return (v, v)
    if not isinstance(t, tuple) or not t:
        return None
    base = type_range(t, types, inv) if t[0] != 'cast' else None
    r = None
    if t[0] == 'cast':
        inner = rng(t[2], types, inv)
        ty = t[1].replace('const ', '')
        tr = C_RANGE.get(ty) or (ENUM_TYPES.get(ty) if inv else None)
        if inner and (tr is None or (tr[0] <= inner[0] and inner[1] <= tr[1])):
            r = inner
        else:
            r = tr
    elif t[0] == 'bin':
        a, b = rng(t[2], types, inv), rng(t[3], types, inv)
        if t[1] == '&':
            for x in (a, b):
                if x and x[0] >= 0:
                    r = (0, x[1]) if r is None else (0, min(r[1], x[1]))
        elif a and b:
            if t[1] == '+':
                r = (a[0] + b[0], a[1] + b[1])
            elif t[1] == '-':
                r = (a[0] - b[1], a[1] - b[0])
            elif t[1] == '*':
                c = [a[0] * b[0], a[0] * b[1], a[1] * b[0], a[1] * b[1]]
                r = (min(c), max(c))
            elif t[1] == '>>' and b[0] == b[1] and 0 <= b[0] < 64:
                r = (a[0] >> b[0], a[1] >> b[0])
            elif t[1] == '%' and b[0] == b[1] and b[0] > 0:
                r = (0, b[0] - 1) if a[0] >= 0 else (-(b[0] - 1), b[0] - 1)
            elif t[1] == '|' and a[0] >= 0 and b[0] >= 0:
                r = (0, (1 << max(a[1], b[1]).bit_length()) - 1)
        if t[1] in ('<', '<=', '>', '>=', '==', '!=', '&&', '||'):
            r = (0, 1)
    elif t[0] == 'un' and t[1] == '!':
        r = (0, 1)
    elif t[0] == 'un' and t[1] == '-':
        a = rng(t[2], types, inv)
        r = (-a[1], -a[0]) if a else None
    elif t[0] == '?:' and len(t) == 4:
        a, b = rng(t[2], types, inv), rng(t[3], types, inv)
        r = (min(a[0], b[0]), max(a[1], b[1])) if a and b else None
    if r is None:
        return base
    if base is not None:
        r = (max(r[0], base[0]), min(r[1], base[1]))
    return r


def range_holds(g, types, inv=False):
    if not isinstance(g, tuple) or g[0] != 'bin' or g[1] not in ('<', '<=', '>', '>=', '==', '!='):
        return False
    a, b = rng(g[2], types, inv), rng(g[3], types, inv)
    if not a or not b:
        return False
    op = g[1]
    return {'<': a[1] < b[0], '<=': a[1] <= b[0], '>': a[0] > b[1], '>=': a[0] >= b[1],
            '==': a[0] == a[1] == b[0] == b[1], '!=': a[1] < b[0] or a[0] > b[1]}[op]


def has(t, kinds):
    if not isinstance(t, tuple) or not t:
        return False
    if t[0] in kinds:
        return True
    return any(has(x, kinds) for x in t[1:])


def refs_of(t, out=None):
    out = set() if out is None else out
    if isinstance(t, tuple) and t:
        if t[0] == 'ref':
            out.add((t[1], t[2]))
        else:
            for x in t[1:]:
                refs_of(x, out)
    return out


def only_params(t):
    """True when t reads nothing but the function's parameters and constants."""
    if not isinstance(t, tuple) or not t:
        return True
    if t[0] == 'ref':
        return t[2] == 'C'
    if t[0] in ('other', 'id', 'call', 'idx'):
        return t[0] == 'other' and t[1] == 'CXX_UNARY_EXPR'
    if t[0] == 'un' and t[1] == '*':
        return all(only_params(x) for x in t[2:])
    return all(only_params(x) for x in t[1:] if isinstance(x, tuple))


def gpath(t):
    """(G key, path) of an lvalue canon rooted in a file-scope variable."""
    path = []
    while isinstance(t, tuple) and t:
        if t[0] == 'ref':
            return (t[1], tuple(reversed(path))) if t[2] == 'G' else None
        if t[0] == 'mem' and not t[2]:
            path.append(t[1])
            t = t[3]
        elif t[0] == 'idx':
            path.append('[]')
            t = t[1]
        elif t[0] == 'cast':
            t = t[2]
        else:
            return None
    return None


def replace_paths(t, key, path, val):
    """Replace every subtree of t that reads (key, path) by val; None when t
    reads key through another path that overlaps (prefix either way)."""
    if not isinstance(t, tuple) or not t:
        return t
    gp = gpath(t) if t[0] in ('ref', 'mem', 'idx') else None
    if gp is not None and gp[0] == key:
        if gp[1] == path:
            return val
        n = min(len(gp[1]), len(path))
        if gp[1][:n] == path[:n]:
            # whole-object read or a write to the whole object
            if len(gp[1]) < len(path):
                return None
            return None
        return t
    out = [t[0]]
    for x in t[1:]:
        r = replace_paths(x, key, path, val)
        if r is None and x is not None:
            return None
        out.append(r)
    return tuple(out)


class Classifier:
    def __init__(self, data):
        funcs = data['funcs']
        self.globals = data['globals']
        ENUMS.update(data.get('enum_consts', {}))
        ENUM_TYPES.update(data.get('enum_types', {}))
        self.funcs = {f['usr']: f for f in funcs}
        self.callers = collections.defaultdict(list)
        self.nonnull_ret = set()
        self.preds = collections.defaultdict(dict)  # file -> name -> canon
        self.extern_preds = {}
        self.writes = collections.defaultdict(list)
        self.escaped = set()
        for f in funcs:
            for c in f['calls']:
                self.callers[c['callee']].append((f, c))
            for w in f['writes']:
                self.writes[w['g']].append((f, w))
                if w['rhs'][0] == 'rmw' and w['rhs'][1] == '&':
                    self.escaped.add(w['g'])
            for d in f['decays']:
                self.escaped.add(d[0])
            if f['pred'] is not None and only_params(f['pred']) or (f['pred'] is not None and self._pure(f['pred'])):
                self.preds[f['file']][f['name']] = f['pred']
                if not f['static']:
                    self.extern_preds[f['name']] = f['pred']
        for f in funcs:
            if f['returns'] and all(r == 'addr' for r in f['returns']):
                self.nonnull_ret.add(f['name'])
        self.memo = {}

    @staticmethod
    def _pure(t):
        """A predicate body over params, constants and calls (expanded later)."""
        if not isinstance(t, tuple) or not t:
            return True
        if t[0] == 'ref':
            return t[2] == 'C'
        if t[0] in ('other', 'id', 'idx'):
            return False
        return all(Classifier._pure(x) for x in t[1:] if isinstance(x, tuple))

    def expand(self, t, file, depth=0):
        """Expand calls of one-line predicates (`return expr;`) over params."""
        if not isinstance(t, tuple) or not t or depth > 8:
            return t
        if t[0] == 'call':
            body = self.preds[file].get(t[1]) or self.extern_preds.get(t[1])
            args = tuple(self.expand(x, file, depth) for x in t[2:])
            if body is not None and self._pure(body):
                return self.expand(cgast.subst(body, list(args)), file, depth + 1)
            return (t[0], t[1]) + args
        return tuple([t[0]] + [self.expand(x, file, depth) if isinstance(x, tuple) else x for x in t[1:]])

    def norm(self, facts, file):
        out = []
        for f in facts:
            for c in cgast.conjuncts(self.expand(f, file)):
                if c not in out:
                    out.append(c)
        return out

    def nonnull(self, t, info):
        if t[0] == 'un' and t[1] == '&':
            return True
        if t[0] == 'lit' and isinstance(t[1], str) and t[1].startswith('"'):
            return True
        if info == 'addr':
            return True
        if info.startswith('call:') and info[5:] in self.nonnull_ret:
            return True
        return False

    def fact_at(self, fn, est, g, ginfo, types, depth):
        """Does fact g (in fn's space) hold at a point with established facts est?"""
        if g in est:
            return 'est'
        v = ev(g)
        if v is not None:
            return 'const' if v else None
        if g[0] == 'bin' and g[1] == '&&':
            a = self.fact_at(fn, est, g[2], ginfo, types, depth)
            b = a and self.fact_at(fn, est, g[3], ginfo, types, depth)
            return b and (a if a == b else 'mixed')
        if g[0] == 'bin' and g[1] == '||':
            return (self.fact_at(fn, est, g[2], ginfo, types, depth)
                    or self.fact_at(fn, est, g[3], ginfo, types, depth))
        if g[0] == 'bin' and g[1] in ('!=', '>') and lit_val(g[3]) == 0:
            if self.nonnull(g[2], ginfo.get(g[2], '')):
                return 'addr'
        if g[0] == 'bin' and g[1] == '!=' and lit_val(g[2]) == 0:
            if self.nonnull(g[3], ginfo.get(g[3], '')):
                return 'addr'
        if g[0] in ('call', 'ref', 'mem', 'un') and self.nonnull(g, ginfo.get(g, '')):
            return 'addr'
        if range_holds(g, types):
            return 'range'
        if g[0] == 'un' and g[1] == '!' and range_holds(negate(g[2]), types):
            return 'range'
        # a range fact implied by a stronger established one: x < 3 from x <= 1
        if g[0] == 'bin' and g[1] in ('<', '<=', '>', '>=') and ev(g[3]) is not None:
            for e in est:
                if e[0] == 'bin' and e[2] == g[2] and ev(e[3]) is not None:
                    c, d = ev(e[3]), ev(g[3])
                    lo_e = {'>': c + 1, '>=': c}.get(e[1])
                    hi_e = {'<': c - 1, '<=': c}.get(e[1])
                    if g[1] in ('>', '>=') and lo_e is not None and lo_e >= (d + 1 if g[1] == '>' else d):
                        return 'est'
                    if g[1] in ('<', '<=') and hi_e is not None and hi_e <= (d - 1 if g[1] == '<' else d):
                        return 'est'
        if only_params(g) and has(g, ('param',)) and depth < 40:
            r = self.holds(fn, g, depth + 1)
            if r:
                return 'caller:' + r
        if range_holds(g, types, inv=True):
            return 'type'
        return None

    def site(self, caller, site, fact, depth):
        g = cgast.subst(fact, site['args'])
        ginfo = {a: i for a, i in zip(site['args'], site['info'])}
        types = dict(site.get('tmap', {}))
        types.update({a: t for a, t in zip(site['args'], site.get('types', []))})
        est = self.norm(site['est'], caller['file'])
        g = self.expand(g, caller['file'])
        return self.fact_at(caller, est, g, ginfo, types, depth)

    def holds(self, f, fact, depth=0):
        """Is `fact` (in f's parameter space) true at every call site of f?"""
        key = (f['usr'], fact)
        if key in self.memo:
            return self.memo[key]
        self.memo[key] = None  # cycle guard
        sites = self.callers.get(f['usr'], [])
        if not sites:
            return None
        kinds = set()
        for caller, site in sites:
            r = self.site(caller, site, fact, depth)
            if not r:
                self.memo[key] = None
                return None
            kinds.add(r.split(':')[0])
        res = '+'.join(sorted(kinds))
        self.memo[key] = res
        return res

    def failing_sites(self, f, fact):
        return [f"{caller['file']}:{site['line']}" for caller, site in self.callers.get(f['usr'], [])
                if not self.site(caller, site, fact, 0)]

    def invariant(self, fact, file):
        """A fact over file-scope variables (and constants) that holds after
        every write: the initial value and every assignment establish it."""
        gs = {k for k, c in refs_of(fact) if c == 'G'}
        if not gs or not all(k in self.globals for k in gs):
            return None
        if has(fact, ('param',)) or any(c == 'L' for _, c in refs_of(fact)):
            return None
        if has(fact, ('call',)):
            return None
        for k in gs:
            if k in self.escaped:
                return None
            init = self.globals[k]['init']
            if init == 'list':
                return None
            val = ('lit', '#0') if init is None else init
            path = None
            # initial value: substitute every read of k
            t = self.subst_all(fact, k, val)
            if t is None or not ev(t):
                return None
            for wf, w in self.writes.get(k, []):
                if w['rhs'][0] == 'rmw':
                    return None
                gp = gpath(w['lhs'])
                if gp is None:
                    return None
                path = gp[1]
                t = replace_paths(fact, k, path, w['rhs'])
                if t is None:
                    return None
                if t == fact:
                    continue  # the write does not touch what the fact reads
                if has(t, (('ref',),)) and any(x == k for x, _ in refs_of(t)):
                    return None
                est = self.norm(w['est'], wf['file'])
                t = self.expand(t, wf['file'])
                if not self.fact_at(wf, est, t, {}, w.get('tmap', {}), 0):
                    return None
        return 'invariant'

    @staticmethod
    def subst_all(t, key, val):
        if not isinstance(t, tuple) or not t:
            return t
        gp = gpath(t) if t[0] in ('ref', 'mem', 'idx') else None
        if gp is not None and gp[0] == key:
            return val
        return tuple([t[0]] + [Classifier.subst_all(x, key, val) if isinstance(x, tuple) else x
                               for x in t[1:]])


EVA_RANK = {'INVALID': 3, 'UNKNOWN': 2, 'VALID': 1, 'DEAD': 0}


def load_eva():
    """Merged EVA verdicts of build/prove (make prove) and any earlier run
    kept in build/prove_*/ on the same tree: the worst verdict wins, and a
    site reached by any run is not DEAD."""
    v = {}
    paths = [os.path.join(OUT, 'eva_verdicts.tsv')]
    paths += sorted(glob.glob(os.path.join(ROOT, 'build', 'prove_*', 'eva_verdicts.tsv')))
    for path in paths:
        if not os.path.exists(path):
            continue
        with open(path) as fh:
            next(fh)
            for ln in fh:
                f, line, verdict, mains = ln.rstrip('\n').split('\t')
                key = (f, int(line))
                old = v.get(key)
                if old is None or EVA_RANK[verdict] > EVA_RANK[old[0]]:
                    ms = mains if old is None or old[1] == '-' else old[1] + ',' + mains
                    v[key] = (verdict, ms)
                elif mains != '-':
                    v[key] = (old[0], old[1] + ',' + mains if old[1] != '-' else mains)
    return v


def classify_guard(cl, f, g, eva):
    ev_v, mains = eva.get((f['file'], g['line']), ('DEAD', '-'))
    if ev_v == 'DEAD':
        for ln in range(g['line'], g['end'] + 1):
            if (f['file'], ln) in eva:
                ev_v, mains = eva[(f['file'], ln)]
                break
    whys = []
    ok = True
    texts = []
    for fact, text in g['facts']:
        if text not in texts:
            texts.append(text)
    for fact in cl.norm([x for x, _ in g['facts']], f['file']):
        r = None
        if range_holds(fact, g.get('tmap', {})):
            r = 'range'
            whys.append('range of the C type')
        elif range_holds(fact, g.get('tmap', {}), inv=True):
            r = 'type'
            whys.append('word/enum type invariant')
        elif only_params(fact) and has(fact, ('param',)):
            r = cl.holds(f, fact)
            if r:
                whys.append(f'param fact at every call site ({r})')
            else:
                bad = cl.failing_sites(f, fact)
                whys.append('param fact not shown at ' + (','.join(bad[:3]) + ('...' if len(bad) > 3 else '') if bad else 'any site (no caller)'))
        elif ev(fact):
            r = 'const'
            whys.append('constant')
        else:
            r = cl.invariant(fact, f['file'])
            whys.append('module invariant (initial value and every write establish it)' if r else 'state/local fact')
        ok = ok and bool(r)
    if ok:
        cls, why = 'GUARANTEED', 'structural: ' + '; '.join(whys)
    elif ev_v == 'VALID':
        cls, why = 'GUARANTEED', f'EVA VALID ({mains}); ' + '; '.join(whys)
    else:
        cls = 'FALLIBLE' if cl.callers.get(f['usr']) else 'UNCALLED'
        why = f'EVA {ev_v}; ' + '; '.join(whys)
    return ' && '.join(texts), ev_v, cls, why


def cmd_classify(out_path, index=INDEX):
    with open(index, 'rb') as fh:
        data = pickle.load(fh)
    cl = Classifier(data)
    eva = load_eva()
    rows = []
    for f in data['funcs']:
        if not f['file'].startswith('src/'):
            continue
        kind = 'void' if f['ret'] == 'void' else ('status' if f['ret'] == 'agc_status_t' else 'value')
        for g in f['guards']:
            cond, ev_v, cls, why = classify_guard(cl, f, g, eva)
            rows.append((f['file'], g['line'], f['name'], kind, cond, ev_v, cls, why))
    with open(out_path, 'w') as fh:
        fh.write('file:line\tfunction\tkind\tcondition\teva\tclass\twhy\n')
        for r in rows:
            fh.write(f'{r[0]}:{r[1]}\t{r[2]}\t{r[3]}\t{r[4]}\t{r[5]}\t{r[6]}\t{r[7]}\n')
    c = collections.Counter((r[3], r[6]) for r in rows)
    for k in sorted(c):
        print(k, c[k])


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    if argv[1] == 'index':
        cmd_index()
    elif argv[1] == 'classify':
        cmd_classify(argv[2] if len(argv) > 2 else os.path.join(ROOT, 'docs', 'overview', 'guards.tsv'),
                     argv[3] if len(argv) > 3 else INDEX)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
