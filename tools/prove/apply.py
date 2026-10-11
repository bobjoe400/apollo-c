#!/usr/bin/env python3
"""apply.py -- delete the guards a guards.tsv classifies GUARANTEED.

Usage (repo root):
    python3 -I tools/prove/apply.py GUARDS.tsv [--structural-only] [--dry-run]
        [--kinds void,value,status] [--files PREFIX...]

Each row names the `if (!AGC_ASSERT(...)) { ... return ...; }` statement at
file:line in the tree the index was built from (build/prove/tree for an
EVA snapshot, else the working tree before any edit).  The statement is
removed whole, bottom-up per file, with a blank line it leaves doubled.
`--structural-only` skips rows whose evidence is EVA alone.
"""
import argparse
import collections
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def guard_extent(lines, i):
    """0-based [i, j] of the if-statement starting at line i (brace counting)."""
    assert 'AGC_ASSERT' in lines[i], (i, lines[i])
    depth = 0
    seen = False
    paren = 0
    j = i
    while j < len(lines):
        s = re.sub(r'/\*.*?\*/', '', lines[j])
        for ch in s:
            if ch == '(':
                paren += 1
            elif ch == ')':
                paren -= 1
            elif ch == '{':
                depth += 1
                seen = True
            elif ch == '}':
                depth -= 1
        if seen and depth == 0:
            return i, j
        if not seen and paren == 0 and s.rstrip().endswith(';') and j > i:
            return i, j  # braceless: if (...) return x;
        j += 1
    raise ValueError(f'no end for guard at {i + 1}')


def assert_args(text):
    """The AGC_ASSERT(...) arguments of a guard's condition, in order."""
    out = []
    for m in re.finditer(r'AGC_ASSERT\(', text):
        i = m.end()
        depth = 1
        j = i
        while depth:
            if text[j] == '(':
                depth += 1
            elif text[j] == ')':
                depth -= 1
            j += 1
        out.append(text[i:j - 1])
    return out


def norm(t):
    return re.sub(r'\s+', '', t)


def function_span(lines, name):
    """0-based [start, end] of the definition of `name` (brace counting)."""
    for k, l in enumerate(lines):
        if re.match(r'^(static\s+)?(inline\s+)?[A-Za-z_][\w\s\*]*\b' + re.escape(name) + r'\s*\(', l) \
                and not l.rstrip().endswith(';'):
            j = k
            while '{' not in lines[j]:
                j += 1
            depth = 0
            e = j
            while True:
                depth += lines[e].count('{') - lines[e].count('}')
                if depth == 0 and e >= j:
                    return k, e
                e += 1
    return None


def locate(lines, func, cond, used):
    """Current 1-based line of the guard of `func` whose AGC_ASSERT
    arguments joined by ' && ' equal `cond` (whitespace-insensitive); the
    first one not in `used`."""
    span = function_span(lines, func)
    if span is None:
        return None
    k, e = span
    i = k
    while i <= e:
        if lines[i].lstrip().startswith('if (!AGC_ASSERT'):
            a, b = guard_extent(lines, i)
            head = ''.join(lines[a:b + 1])
            hdr = head.split('{')[0] if '{' in head else head
            args = assert_args(hdr)
            if norm(' && '.join(args)) == norm(cond) and (i + 1) not in used:
                return i + 1
            i = b + 1
            continue
        i += 1
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('tsv')
    ap.add_argument('--structural-only', action='store_true')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--kinds', default='void,value,status')
    ap.add_argument('--files', nargs='*')
    ap.add_argument('--match', action='store_true',
                    help='find each guard by function and condition in the current tree '
                         '(the tsv line numbers refer to an older tree)')
    ap.add_argument('--eva-only', action='store_true', help='only rows whose evidence is EVA')
    a = ap.parse_args()
    kinds = set(a.kinds.split(','))
    todo = collections.defaultdict(list)
    with open(a.tsv) as fh:
        next(fh)
        for ln in fh:
            r = ln.rstrip('\n').split('\t')
            loc, fn, kind, cond, ev, cls, why = r[:7]
            if cls != 'GUARANTEED' or kind not in kinds:
                continue
            if a.structural_only and not why.startswith('structural'):
                continue
            if a.eva_only and not why.startswith('EVA VALID'):
                continue
            f, line = loc.rsplit(':', 1)
            if a.files and not any(f.startswith(p) for p in a.files):
                continue
            todo[f].append((int(line), fn, cond))
    total = 0
    for f, lines_ in sorted(todo.items()):
        path = os.path.join(ROOT, f)
        with open(path) as fh:
            lines = fh.readlines()
        targets = []
        used = set()
        for line, fn, cond in lines_:
            if a.match:
                cur = locate(lines, fn, cond, used)
                if cur is None:
                    print(f'NOT FOUND {f}:{line} {fn}: {cond}')
                    continue
                used.add(cur)
                targets.append(cur)
            else:
                targets.append(line)
        for line in sorted(set(targets), reverse=True):
            i, j = guard_extent(lines, line - 1)
            del lines[i:j + 1]
            # a blank line left doubled, or a blank right after `{` / before `}`
            if i < len(lines) and i > 0 and not lines[i].strip() and (
                    not lines[i - 1].strip() or lines[i - 1].strip().endswith('{')):
                del lines[i]
            elif i < len(lines) and i > 0 and lines[i].strip() == '}' and not lines[i - 1].strip():
                del lines[i - 1]
            total += 1
        if not a.dry_run:
            with open(path, 'w') as fh:
                fh.writelines(lines)
    print(f'{total} guards deleted in {len(todo)} files' + (' (dry run)' if a.dry_run else ''))


if __name__ == '__main__':
    sys.exit(main())
