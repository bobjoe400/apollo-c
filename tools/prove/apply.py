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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('tsv')
    ap.add_argument('--structural-only', action='store_true')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--kinds', default='void,value,status')
    ap.add_argument('--files', nargs='*')
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
            f, line = loc.rsplit(':', 1)
            if a.files and not any(f.startswith(p) for p in a.files):
                continue
            todo[f].append(int(line))
    total = 0
    for f, lines_ in sorted(todo.items()):
        path = os.path.join(ROOT, f)
        with open(path) as fh:
            lines = fh.readlines()
        for line in sorted(set(lines_), reverse=True):
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
