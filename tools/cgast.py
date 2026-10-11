#!/usr/bin/env python3
"""cgast.py -- libclang helpers shared by callgraph.py and inline_one.py.

Everything here works on one parsed translation unit and returns plain
Python data (tuples, sets, dicts), so the call-graph script can combine
facts from different files without keeping ASTs alive.

* canon(): a structural canonical form of an expression.  Parentheses and
  implicit conversions are dropped, names are resolved to their
  declarations (USR), so `(x)` == `x` and two different locals both named
  `n` are different.  Helper parameters become ('param', i) so a helper's
  guard can be re-expressed in the caller's terms by substitution.
* guard_facts(): the conjuncts asserted by an `if (!AGC_ASSERT(c)) {
  return ...; }` guard (also `!AGC_ASSERT(a) || !AGC_ASSERT(b)`).
* summarize(): what a statement may change (written variables, writes
  through pointers, calls), used to decide whether an earlier guard still
  holds at a later point.
"""
import os
import re

import clang.cindex as ci

K = ci.CursorKind
SKIP = (K.UNEXPOSED_EXPR, K.PAREN_EXPR)
ASSIGN_OPS = {'=', '+=', '-=', '*=', '/=', '%=', '<<=', '>>=', '&=', '|=', '^='}
LOOPS = (K.FOR_STMT, K.WHILE_STMT, K.DO_STMT)
PARSE_ARGS = ['-std=c11', '-Iinclude', '-fparse-all-comments',
              '-include', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cgast_assert.h')]
_INDEX = None


def parse(path, extra=(), text=None):
    """Parse path (or `text` standing in for its contents)."""
    global _INDEX
    if _INDEX is None:
        _INDEX = ci.Index.create()
    unsaved = [(path, text)] if text is not None else None
    return _INDEX.parse(path, args=PARSE_ARGS + list(extra), unsaved_files=unsaved)


def kids(n):
    return list(n.get_children())


def toks(n):
    return [t.spelling for t in n.get_tokens()]


def strip(n):
    while n.kind in SKIP:
        ch = kids(n)
        if len(ch) != 1:
            break
        n = ch[0]
    return n


def in_file(n, path):
    f = n.location.file
    return f is not None and os.path.realpath(f.name) == os.path.realpath(path)


def functions(tu, path):
    """Function definitions spelled in `path`, in source order."""
    out = []
    for c in tu.cursor.get_children():
        if c.kind == K.FUNCTION_DECL and c.is_definition() and in_file(c, path):
            out.append(c)
    return out


def body(fn):
    for c in kids(fn):
        if c.kind == K.COMPOUND_STMT:
            return c
    return None


def params(fn):
    return [c for c in kids(fn) if c.kind == K.PARM_DECL]


def ref_class(r):
    if r.kind == K.PARM_DECL:
        return 'P'
    if r.kind == K.ENUM_CONSTANT_DECL:
        return 'C'
    if r.kind == K.FUNCTION_DECL:
        return 'F'
    if r.kind == K.VAR_DECL:
        sp = r.semantic_parent
        if sp is None or sp.kind == K.TRANSLATION_UNIT or r.storage_class == ci.StorageClass.STATIC:
            return 'G'
        return 'L'
    return 'O'


def ref_key(r):
    u = r.get_usr()
    return u if u else f'{r.spelling}@{r.location.line}:{r.location.column}'


_MACROS = {}


def macros_of(path):
    """Object-like macros visible in path (gcc -dM), name -> replacement."""
    if path not in _MACROS:
        import subprocess
        r = subprocess.run(['gcc', '-std=c11', '-Iinclude', '-dM', '-E', path],
                           capture_output=True, text=True)
        m = {}
        for ln in r.stdout.splitlines():
            parts = ln.split(None, 2)
            if len(parts) >= 2 and parts[0] == '#define' and '(' not in parts[1]:
                m[parts[1]] = parts[2] if len(parts) > 2 else ''
        _MACROS[path] = m
    return _MACROS[path]


def norm_lit(text, macros):
    """A literal or object-like macro as its integer value ('#32767') when it
    has one, so WORD15, 077777u and 32767 compare equal; NULL is 0."""
    t = text
    for _ in range(8):
        t = t.strip()
        while t.startswith('(') and t.endswith(')'):
            t = t[1:-1].strip()
        # a cast to a plain type name: ((agc_dp_t) -0x0FFFFFFF), (void *) 0
        t = re.sub(r'^\(\s*(?:const\s+)?[A-Za-z_]\w*\s*\*?\s*\)\s*', '', t)
        if macros and t in macros:
            t = macros[t]
            continue
        break
    t = re.sub(r'(?<=[0-9a-fA-F])[uUlL]+$', '', t.strip())
    try:
        if re.fullmatch(r'0[0-7]+', t):
            return '#' + str(int(t, 8))
        return '#' + str(int(t, 0))
    except ValueError:
        return text


class Ctx:
    """Canonicalisation context: maps a function's parameters to indices;
    `macros` (macros_of) lets literals compare by value."""

    def __init__(self, fn=None, macros=None):
        self.macros = macros
        self.pmap = {}
        if fn is not None:
            for i, p in enumerate(params(fn)):
                self.pmap[ref_key(p)] = i


_SRC_CACHE = {}


def _src(path):
    if path not in _SRC_CACHE:
        with open(path, encoding='utf-8', errors='replace') as fh:
            _SRC_CACHE[path] = fh.read().encode('utf-8')
    return _SRC_CACHE[path]


def _text_between(a, b):
    """Source text between the end of cursor a and the start of cursor b."""
    fa, fb = a.extent.end.file, b.extent.start.file
    if fa is None or fb is None or fa.name != fb.name:
        return None
    lo, hi = a.extent.end.offset, b.extent.start.offset
    if hi < lo:
        return None
    return _src(fa.name)[lo:hi].decode('utf-8', 'replace')


def _binop(n, ch):
    t = toks(n)
    lt = toks(ch[0])
    if t and len(t) > len(lt) and t[:len(lt)] == lt:
        return t[len(lt)]
    txt = _text_between(ch[0], ch[1])
    if txt is not None:
        op = txt.strip().strip('()').strip()
        if op:
            return op
    return f'?{n.location.line}:{n.location.column}'


def spelled(n):
    """Token text of a leaf; falls back to the source at its location (a
    literal that comes from a macro such as `true` has no tokens)."""
    t = toks(n)
    if len(t) == 1:
        return t[0]
    f = n.extent.start.file
    if f is not None:
        m = re.match(rb'[A-Za-z_0-9.]+', _src(f.name)[n.extent.start.offset:])
        if m:
            return m.group(0).decode()
    return f'?{n.location.line}:{n.location.column}'


def _unop(n, ch):
    t = toks(n)
    ct = toks(ch[0]) if ch else []
    if not t:
        f = n.extent.start.file
        if f is not None and ch and ch[0].extent.start.offset > n.extent.start.offset:
            pre = _src(f.name)[n.extent.start.offset:ch[0].extent.start.offset].decode().strip()
            if pre:
                return pre
        return f'?{n.location.line}:{n.location.column}'
    if ct and t[-len(ct):] == ct and len(t) > len(ct):
        return t[0]
    if ct and t[:len(ct)] == ct and len(t) > len(ct):
        return 'post' + t[-1]
    return t[0]


def op_of(n):
    ch = kids(n)
    if n.kind in (K.BINARY_OPERATOR, K.COMPOUND_ASSIGNMENT_OPERATOR):
        return _binop(n, ch)
    if n.kind == K.UNARY_OPERATOR:
        return _unop(n, ch)
    return None


def canon(n, ctx):
    n = strip(n)
    k = n.kind
    ch = kids(n)
    macros = getattr(ctx, 'macros', None)
    if ch and k not in (K.CALL_EXPR, K.DECL_REF_EXPR) and n.extent.start.file is not None:
        # a whole expression from one object-like macro (AGC_DP_MIN): by value
        a, b = n.extent.start.offset, n.extent.end.offset
        m = re.match(rb'[A-Za-z_]\w*', _src(n.extent.start.file.name)[a:a + 64])
        if m and (b == a or b == a + len(m.group(0))) and (
                macros is None or m.group(0).decode() in macros):
            return ('lit', norm_lit(m.group(0).decode(), macros))
    if k == K.CSTYLE_CAST_EXPR and n.type.spelling == 'void *':
        inner = canon(ch[-1], ctx)
        if inner[0] == 'lit':
            return inner  # (void *) 0 is NULL is 0
    if k == K.DECL_REF_EXPR:
        r = n.referenced
        if r is None:
            return ('id', n.spelling)
        key = ref_key(r)
        if key in ctx.pmap:
            return ('param', ctx.pmap[key])
        return ('ref', key, ref_class(r), n.spelling)
    if k in (K.INTEGER_LITERAL, K.FLOATING_LITERAL, K.CHARACTER_LITERAL, K.STRING_LITERAL,
             K.CXX_BOOL_LITERAL_EXPR):
        return ('lit', norm_lit(spelled(n), getattr(ctx, 'macros', None)))
    if k in (K.BINARY_OPERATOR, K.COMPOUND_ASSIGNMENT_OPERATOR) and len(ch) == 2:
        return ('bin', op_of(n), canon(ch[0], ctx), canon(ch[1], ctx))
    if k == K.UNARY_OPERATOR and len(ch) == 1:
        return ('un', op_of(n), canon(ch[0], ctx))
    if k == K.CSTYLE_CAST_EXPR:
        return ('cast', n.type.spelling, canon(ch[-1], ctx))
    if k == K.MEMBER_REF_EXPR and ch:
        return ('mem', n.spelling, is_arrow(n, ch[0]), canon(ch[0], ctx))
    if k == K.ARRAY_SUBSCRIPT_EXPR and len(ch) == 2:
        return ('idx', canon(ch[0], ctx), canon(ch[1], ctx))
    if k == K.CALL_EXPR:
        return ('call', n.spelling) + tuple(canon(a, ctx) for a in ch[1:])
    if k == K.CONDITIONAL_OPERATOR:
        return ('?:',) + tuple(canon(c, ctx) for c in ch)
    if k == K.CXX_UNARY_EXPR:
        return ('sizeof', ' '.join(toks(n)))
    return ('other', k.name, ' '.join(toks(n))) + tuple(canon(c, ctx) for c in ch)


def conjuncts(c):
    if c[0] == 'bin' and c[1] == '&&':
        return conjuncts(c[2]) + conjuncts(c[3])
    return [c]


def _assert_arg(n):
    """n is `!AGC_ASSERT(x)`: return the cursor of x, else None.

    AGC_ASSERT is a function here (cgast_assert.h), so x is a call argument
    with its real extents."""
    n = strip(n)
    if n.kind != K.UNARY_OPERATOR or op_of(n) != '!':
        return None
    ch = kids(n)
    if not ch:
        return None
    c = strip(ch[0])
    if c.kind != K.CALL_EXPR or c.spelling != 'AGC_ASSERT':
        return None
    args = kids(c)[1:]
    return args[0] if len(args) == 1 else None


def _assert_terms(n):
    a = _assert_arg(n)
    if a is not None:
        return [a]
    n = strip(n)
    if n.kind == K.BINARY_OPERATOR and op_of(n) == '||':
        ch = kids(n)
        left, right = _assert_terms(ch[0]), _assert_terms(ch[1])
        if left is not None and right is not None:
            return left + right
    return None


def is_guard(stmt):
    """`if (!AGC_ASSERT(c) [|| !AGC_ASSERT(d)]) { return ...; }` without else."""
    if stmt.kind != K.IF_STMT:
        return None
    ch = kids(stmt)
    if len(ch) != 2:
        return None
    terms = _assert_terms(ch[0])
    if terms is None:
        return None
    then = ch[1]
    if then.kind == K.COMPOUND_STMT:
        tk = kids(then)
        if len(tk) != 1:
            return None
        then = tk[0]
    if then.kind != K.RETURN_STMT:
        return None
    return terms


def guard_facts(stmt, ctx):
    terms = is_guard(stmt)
    if terms is None:
        return None
    facts = []
    for t in terms:
        for f in conjuncts(canon(t, ctx)):
            if f not in facts:
                facts.append(f)
    return facts


def is_arrow(n, base):
    t = toks(n)
    bt = toks(base)
    if t and len(t) > len(bt) and t[:len(bt)] == bt:
        return t[len(bt)] == '->'
    f = base.extent.end.file
    if f is not None and n.location.offset > base.extent.end.offset:
        return '->' in _src(f.name)[base.extent.end.offset:n.location.offset].decode()
    return True  # unknown: treat as a write through a pointer (conservative)


def _lhs_target(n, ctx):
    """(key, memwrite) for an assigned / incremented / address-taken lvalue."""
    n = strip(n)
    while True:
        if n.kind == K.MEMBER_REF_EXPR:
            ch = kids(n)
            if not ch or is_arrow(n, ch[0]):
                return None, True
            n = strip(ch[0])
        elif n.kind == K.ARRAY_SUBSCRIPT_EXPR:
            base = strip(kids(n)[0])
            if base.kind == K.DECL_REF_EXPR and base.type.kind == ci.TypeKind.POINTER:
                return None, True
            n = base
        elif n.kind == K.UNARY_OPERATOR and op_of(n) == '*':
            return None, True
        elif n.kind == K.DECL_REF_EXPR:
            r = n.referenced
            if r is None:
                return None, True
            key = ref_key(r)
            if key in ctx.pmap:
                return ('param', ctx.pmap[key]), False
            return key, False
        else:
            return None, True


def summarize(n, ctx):
    """Effects of a statement subtree: writes, memwrite, call, returns, locals."""
    s = {'writes': set(), 'memwrite': False, 'call': False, 'returns': 0, 'locals': set(),
         'globals': set()}

    def visit(x):
        k = x.kind
        if k in (K.BINARY_OPERATOR, K.COMPOUND_ASSIGNMENT_OPERATOR):
            if op_of(x) in ASSIGN_OPS:
                key, mw = _lhs_target(kids(x)[0], ctx)
                if key is not None:
                    s['writes'].add(key)
                s['memwrite'] |= mw
        elif k == K.UNARY_OPERATOR:
            op = op_of(x)
            if op in ('++', '--', 'post++', 'post--', '&'):
                key, mw = _lhs_target(kids(x)[0], ctx)
                if key is not None:
                    s['writes'].add(key)
                s['memwrite'] |= mw
        elif k == K.CALL_EXPR:
            s['call'] = True
            for a in kids(x)[1:]:
                a = strip(a)
                if a.kind == K.DECL_REF_EXPR and a.type.kind in (
                        ci.TypeKind.CONSTANTARRAY, ci.TypeKind.INCOMPLETEARRAY):
                    key, _ = _lhs_target(a, ctx)
                    if key is not None:
                        s['writes'].add(key)
        elif k == K.RETURN_STMT:
            s['returns'] += 1
        elif k == K.VAR_DECL:
            s['locals'].add(x.spelling)
        elif k == K.DECL_REF_EXPR and x.referenced is not None:
            if ref_class(x.referenced) == 'G':
                s['globals'].add(x.spelling)
        for c in kids(x):
            visit(c)

    visit(n)
    return s


def fact_meta(f):
    """(keys, volatile): keys = variables it reads; volatile = depends on memory/calls."""
    keys = set()
    vol = [False, False]  # [mem/global, call]

    def walk(t):
        if not isinstance(t, tuple) or not t:
            return
        if t[0] == 'ref':
            keys.add(t[1])
            if t[2] == 'G':
                vol[0] = True
            return
        if t[0] == 'param':
            keys.add(t)
            return
        if t[0] in ('mem', 'idx') or (t[0] == 'un' and t[1] == '*'):
            vol[0] = True
        if t[0] == 'call':
            vol[1] = True
        for x in t[1:]:
            walk(x)

    walk(f)
    return keys, vol[0], vol[1]


def invalidated(f, eff):
    keys, mem, call = fact_meta(f)
    if keys & eff['writes']:
        return True
    if eff['memwrite'] and (mem or call):
        return True
    if eff['call'] and (mem or call):
        return True
    return False


def subst(t, args):
    """Replace ('param', i) by args[i] (caller-space canonical forms)."""
    if not isinstance(t, tuple) or not t:
        return t
    if t[0] == 'param':
        i = t[1]
        return args[i] if i < len(args) else t
    return tuple(subst(x, args) for x in t)


def path_to(root, target):
    """Ancestor chain [root, ..., target] or None."""
    stack = [root]

    def go(n):
        if n == target:
            return True
        for c in kids(n):
            stack.append(c)
            if go(c):
                return True
            stack.pop()
        return False

    return list(stack) if go(root) else None


def established_at(fn, call, ctx):
    """Guard facts (caller space) that hold where `call` is evaluated."""
    chain = path_to(body(fn), call)
    if chain is None:
        return []
    est = []

    def apply(eff):
        est[:] = [f for f in est if not invalidated(f, eff)]

    for parent, child in zip(chain, chain[1:]):
        if parent.kind == K.COMPOUND_STMT:
            for s in kids(parent):
                if s == child:
                    break
                facts = guard_facts(s, ctx)
                if facts is not None:
                    est.extend(f for f in facts if f not in est)
                else:
                    apply(summarize(s, ctx))
        elif parent.kind in LOOPS or parent.kind == K.SWITCH_STMT:
            apply(summarize(parent, ctx))
        elif parent.kind == K.IF_STMT:
            cond = kids(parent)[0]
            if child != cond:
                apply(summarize(cond, ctx))
    return est


# ---- line counting (same rule as tools/check_rules.py) --------------------

def code_lines(lines):
    n = 0
    for raw in lines:
        s = raw.strip()
        if s and s not in ('{', '}') and not s.startswith(('/*', '*', '//')):
            n += 1
    return n


def span_code_lines(src_lines, first, last):
    """Code lines in 1-based inclusive line range."""
    return code_lines(src_lines[first - 1:last])


def body_code_lines(src_lines, fn):
    b = body(fn)
    return span_code_lines(src_lines, b.extent.start.line, b.extent.end.line)


def header_comment(src_lines, first_line):
    """The comment block ending on the line just above `first_line`."""
    i = first_line - 2
    if i < 0 or not src_lines[i].rstrip().endswith('*/'):
        return ''
    j = i
    while j >= 0 and '/*' not in src_lines[j]:
        j -= 1
    # a block comment of its own: it opens its line, and the lines after the
    # opener are comment text (a trailing comment on code is not one)
    if j < 0 or not src_lines[j].lstrip().startswith('/*'):
        return ''
    for k in range(j + 1, i + 1):
        t = src_lines[k].lstrip()
        if '/*' in t and not t.startswith(('*', '/*')):
            return ''
    return ''.join(src_lines[j:i + 1])


def first_comment_line(c):
    for ln in c.splitlines():
        s = re.sub(r'^\s*/?\*+\s?', '', ln).replace('*/', '').strip()
        if s:
            return s
    return ''
