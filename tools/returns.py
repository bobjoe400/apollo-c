#!/usr/bin/env python3
"""returns.py -- dead and normal-flow early returns (docs/CODING_STANDARD.md,
"Returns"; history/assignments/status-handling-2.md step 8).

Usage (repo root):
    python3 -I tools/returns.py report [FILES...]   # counts and a list
    python3 -I tools/returns.py fix [FILES...]      # rewrite in place

On the libclang AST of every function defined in src/ (or FILES):

  trailing  a `return;` in a void function with nothing after it on its
            path (it is the last statement of its block, and so is every
            enclosing if/else/block up to the function body): deleted.
            An `if (c) { return; }` left empty by that is deleted when c
            has no side effects.
  early     a `return;` in a void function, not trailing, that is not an
            error path, in an `if` without `else` directly in the function
            body: `if (c) { A; return; } T` becomes `if (c) { A; } else {
            T }` (`if (!(c)) { T }` when A is empty) when the tail T has at
            most --tail code lines.  Returns of a status or value function
            are left alone (their early returns are the error paths rule 1
            allows, or carry the result).

Every rewritten file is compiled with the Makefile flags; a file that does
not compile is restored.  Run `make format` afterwards.
"""
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import cgast  # noqa: E402

K = cgast.K
CFLAGS = ['-std=c11', '-Wall', '-Wextra', '-Wpedantic', '-Wconversion', '-Wsign-conversion',
          '-Wshadow', '-Wstrict-prototypes', '-Wmissing-prototypes', '-Wundef', '-Wcast-qual',
          '-Wswitch-enum', '-Werror', '-O2', '-Iinclude']
TAIL_MAX = 12
CALL_RE = re.compile(r'\b[A-Za-z_]\w*\s*\(')


def walk_parent(n, parent=None):
    yield n, parent
    for c in n.get_children():
        yield from walk_parent(c, n)


def parents_map(fn):
    m = {}
    for n, p in walk_parent(fn):
        m[(n.extent.start.offset, n.extent.end.offset, n.kind)] = p
    return m


def key(n):
    return (n.extent.start.offset, n.extent.end.offset, n.kind)


def is_trailing(ret, par, body):
    """Nothing follows ret on its path up to the function body."""
    n = ret
    while True:
        p = par.get(key(n))
        if p is None:
            return False
        if p.kind == K.COMPOUND_STMT:
            ch = cgast.kids(p)
            if key(ch[-1]) != key(n):
                return False
            if key(p) == key(body):
                return True
            n = p
            continue
        if p.kind == K.IF_STMT:
            ch = cgast.kids(p)
            if key(ch[0]) == key(n):
                return False  # the condition
            n = p
            continue
        return False  # a loop, switch case, ... : something may follow


def code_lines(lines, a, b):
    return cgast.span_code_lines(lines, a, b)


def analyse(path):
    with open(path, encoding='utf-8') as fh:
        lines = fh.readlines()
    tu = cgast.parse(path)
    out = []
    for fn in cgast.functions(tu, path):
        void = fn.result_type.spelling == 'void'
        body = cgast.body(fn)
        if body is None:
            continue
        par = parents_map(fn)
        top = cgast.kids(body)
        final = None  # value function: the text of its closing `return (C);`
        if not void:
            if not top or top[-1].kind != K.RETURN_STMT:
                continue
            final = ' '.join(cgast.toks(top[-1]))
            if not re.fullmatch(r'return \( ?[A-Za-z_0-9]+ ?\) ?;?', final):
                continue  # only a constant or a plain variable closes the function
        for n, _ in walk_parent(body):
            if n.kind != K.RETURN_STMT:
                continue
            if not void:
                if key(n) == key(top[-1]) or ' '.join(cgast.toks(n)) != final:
                    continue  # the closing return, or an early return of another value
            if void and is_trailing(n, par, body):
                out.append(('trailing', fn.spelling, n.extent.start.line, None))
                continue
            # early: the enclosing if is a top-level statement of the body
            p = par.get(key(n))
            blk = p if p is not None and p.kind == K.COMPOUND_STMT else None
            ifs = par.get(key(blk)) if blk is not None else None
            if ifs is None or ifs.kind != K.IF_STMT or len(cgast.kids(ifs)) != 2:
                out.append(('early-other', fn.spelling, n.extent.start.line, None))
                continue
            if key(cgast.kids(blk)[-1]) != key(n) or par.get(key(ifs)) is None or key(par.get(key(ifs))) != key(body):
                out.append(('early-other', fn.spelling, n.extent.start.line, None))
                continue
            cond = cgast.kids(ifs)[0]
            ctext = ' '.join(cgast.toks(cond))
            if 'AGC_ASSERT' in ctext:
                out.append(('early-error', fn.spelling, n.extent.start.line, None))
                continue
            i = [key(s) for s in top].index(key(ifs))
            tail = top[i + 1:] if void else top[i + 1:-1]
            if not void and not tail:
                continue
            tl = code_lines(lines, tail[0].extent.start.line, tail[-1].extent.end.line) if tail else 0
            out.append(('early', fn.spelling, n.extent.start.line,
                        {'if': ifs, 'blk': blk, 'ret': n, 'tail': tail, 'tail_lines': tl, 'cond': cond,
                         'end': body.extent.end.line if void else top[-1].extent.start.line,
                         'void': void}))
    return out, lines


def line_of(n):
    return n.extent.start.line


RET_LINE = re.compile(r'^\s*return(\s*\([^;]*\))?;\s*(/\*.*\*/)?\s*$')


INVERSE = {'==': '!=', '!=': '==', '<': '>=', '>=': '<', '>': '<=', '<=': '>'}


def top_level_ops(c):
    """Binary operators outside parentheses, with their offsets."""
    d = 0
    ops = []
    i = 0
    while i < len(c):
        ch = c[i]
        if ch in '([':
            d += 1
        elif ch in ')]':
            d -= 1
        elif d == 0 and c.startswith('->', i):
            i += 1  # member access, not an operator
        elif d == 0:
            for op in ('&&', '||', '==', '!=', '<=', '>=', '<<', '>>', '<', '>', '?'):
                if c.startswith(op, i):
                    ops.append((i, op))
                    i += len(op) - 1
                    break
        i += 1
    return ops


def wrapped(c):
    """c is one parenthesised expression."""
    if not (c.startswith('(') and c.endswith(')')):
        return False
    d = 0
    for i, ch in enumerate(c):
        d += (ch == '(') - (ch == ')')
        if d == 0 and i < len(c) - 1:
            return False
    return True


def negate(c):
    """The negation of condition text c, as plainly as it can be written."""
    if c.startswith('!') and not c.startswith('!=') and not top_level_ops(c):
        inner = c[1:].strip()
        return inner[1:-1].strip() if wrapped(inner) else inner
    if wrapped(c):
        return negate(c[1:-1].strip())
    ops = top_level_ops(c)
    if not ops:
        return '!' + c
    if len(ops) == 1 and ops[0][1] in INVERSE:
        i, op = ops[0]
        return c[:i].rstrip() + ' ' + INVERSE[op] + ' ' + c[i + len(op):].lstrip()
    return '!(' + c + ')'


def comment_of(text):
    m = re.search(r'/\*.*\*/', text)
    return m.group(0) if m else None


def fix_file(path, tail_max):
    """Apply the trailing deletions and short-tail if/else rewrites, bottom
    up so line numbers stay valid; returns (lines, trailing, early)."""
    found, lines = analyse(path)
    edits = []  # (first_line, last_line, replacement lines, kind) 1-based inclusive
    for kind, fn, ln, d in found:
        if kind == 'trailing':
            text = lines[ln - 1]
            if not RET_LINE.match(text):
                continue
            com = comment_of(text)
            prev = ln - 2
            while prev >= 0 and not lines[prev].strip():
                prev -= 1
            if com and prev >= 0 and lines[prev].rstrip().endswith(';') and comment_of(lines[prev]) is None:
                # the return's comment moves to the statement before it
                edits.append((prev + 1, ln, [lines[prev].rstrip('\n') + ' ' + com + '\n'], 'trailing'))
            elif com:
                ind = re.match(r'^(\s*)', text).group(1)
                edits.append((ln, ln, [ind + com + '\n'], 'trailing'))
            else:
                edits.append((prev + 2, ln, [], 'trailing'))  # with the blank lines before it
        elif kind == 'early' and d['tail'] and d['tail_lines'] <= tail_max:
            ifs, blk, ret = d['if'], d['blk'], d['ret']
            if ret.extent.start.line != ret.extent.end.line or not RET_LINE.match(lines[ret.extent.start.line - 1]):
                continue
            close = blk.extent.end.line  # the '}' of the then-block
            if lines[close - 1].strip() != '}':
                continue
            head = lines[ifs.extent.start.line - 1]
            if head.count('(') != head.count(')') or '{' in head:
                continue  # a condition over several lines, or a brace on the if line
            last = d['end'] - 1  # the function's closing brace, or its closing return
            while last > close and not lines[last - 1].strip():
                last -= 1  # blank lines before the closing return stay outside
            tail = lines[close:last]
            ind = re.match(r'^(\s*)', head).group(1)
            body = ['    ' + t if t.strip() else t for t in tail]
            while body and not body[0].strip():
                body.pop(0)
            if len(cgast.kids(blk)) == 1:
                # if (c) { return; } T  ->  if (!(c)) { T }
                m = re.match(r'^\s*if \((.*)\)\s*(/\*.*\*/)?\s*$', head)
                if m is None:
                    continue
                cond = negate(m.group(1).strip())
                rcom = comment_of(lines[ret.extent.start.line - 1])
                new_head = ind + 'if (' + cond + ')' + (' ' + rcom if rcom else '') + '\n'
                new = [new_head, ind + '{\n'] + body + [ind + '}\n']
            else:
                then = [l for i, l in enumerate(lines[blk.extent.start.line - 1:close], blk.extent.start.line)
                        if i != ret.extent.start.line]
                while len(then) >= 2 and not then[-2].strip():
                    del then[-2]
                new = lines[ifs.extent.start.line - 1:blk.extent.start.line - 1] + then
                new += [ind + 'else\n', ind + '{\n'] + body + [ind + '}\n']
            edits.append((ifs.extent.start.line, last, new, 'early'))
    edits.sort(key=lambda e: e[0], reverse=True)
    floor = None
    nt = ne = 0
    for a, b, rep, kind in edits:
        if floor is not None and b >= floor:
            continue
        lines[a - 1:b] = rep
        floor = a
        if kind == 'trailing':
            nt += 1
        else:
            ne += 1
    return lines, nt, ne


def compile_ok(path):
    r = subprocess.run(['gcc'] + CFLAGS + ['-c', '-o', '/dev/null', path], cwd=ROOT,
                       capture_output=True, text=True)
    return r.returncode == 0, r.stderr


def main(argv):
    os.chdir(ROOT)
    mode = argv[1] if len(argv) > 1 else 'report'
    files = argv[2:] or sorted(f for f in __import__('glob').glob('src/**/*.c', recursive=True))
    if mode == 'report':
        import collections
        c = collections.Counter()
        short = 0
        for f in files:
            found, _ = analyse(f)
            for kind, fn, ln, d in found:
                c[kind] += 1
                if kind == 'early' and d['tail_lines'] <= TAIL_MAX:
                    short += 1
                print(f'{kind}\t{f}:{ln}\t{fn}' + (f"\ttail {d['tail_lines']}" if d else ''))
        print(dict(c), f'early with a tail <= {TAIL_MAX}: {short}', file=sys.stderr)
        return 0
    tot = [0, 0]
    for f in files:
        for _ in range(4):  # one if per function per pass
            with open(f, encoding='utf-8') as fh:
                orig = fh.read()
            lines, nt, ne = fix_file(f, TAIL_MAX)
            if nt + ne == 0:
                break
            with open(f, 'w', encoding='utf-8') as fh:
                fh.writelines(lines)
            ok, err = compile_ok(f)
            if not ok:
                with open(f, 'w', encoding='utf-8') as fh:
                    fh.write(orig)
                print(f'RESTORED {f}: {err.splitlines()[0] if err else ""}')
                break
            tot[0] += nt
            tot[1] += ne
    print(f'trailing returns deleted: {tot[0]}; early returns rewritten as if/else: {tot[1]}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
