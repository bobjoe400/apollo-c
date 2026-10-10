#!/usr/bin/env python3
"""check_rules.py -- mechanical checks for the Power of Ten rules.

Usage: python3 -I tools/check_rules.py <dir>...
Checks: rule 1 (goto/setjmp/longjmp, direct recursion), rule 3 (heap),
rule 4 (function length <= 60 code lines), rule 5 (assertion density),
rule 8 (preprocessor: token pasting, variadic macros, #if count),
rule 9 (function pointers, double dereference).
Exit status 1 if any hard violation is found. Density is reported as a
warning per file (rule 5 is an average, not a per-function rule).
"""
import os
import re
import sys

MAX_FUNC_LINES = 60
FUNC_RE = re.compile(r'^[A-Za-z_][\w\s\*]*\s\**([A-Za-z_]\w*)\s*\([^;]*\)\s*\{?\s*$')
HARD = []
WARN = []


def check_file(path):
    with open(path, encoding='utf-8', errors='replace') as fh:
        lines = fh.readlines()
    n_funcs = 0
    n_asserts = 0
    ifs = 0
    in_func = None
    depth = 0
    start = 0
    code_lines = 0
    for i, raw in enumerate(lines, 1):
        s = raw.strip()
        if re.search(r'\bgoto\b|\bsetjmp\b|\blongjmp\b', s) and not s.startswith(('/*', '*', '//')):
            HARD.append(f'{path}:{i}: rule 1: goto/setjmp/longjmp')
        if re.search(r'\b(malloc|calloc|realloc|free|strdup)\s*\(', s) and not s.startswith(('/*', '*', '//')):
            HARD.append(f'{path}:{i}: rule 3: heap allocation')
        if s.startswith('#define') and ('##' in s or '...' in s):
            HARD.append(f'{path}:{i}: rule 8: token pasting / variadic macro')
        if s.startswith(('#if', '#ifdef', '#ifndef')):
            ifs += 1
        if re.search(r'\(\s*void\s*\)\s*agc_', s) and not s.startswith(('/*', '*', '//')) and '/src/' in '/' + path:
            HARD.append(f'{path}:{i}: rule 7: (void)-cast of an agc_ call; check it, handle it as the AGC does, or make the callee void')
        if re.search(r'\(\s*\*\s*\w+\s*\)\s*\(', s) and not s.startswith(('/*', '*', '//')):
            HARD.append(f'{path}:{i}: rule 9: function pointer')
        if re.search(r'\*\*\s*[A-Za-z_]', s) and not s.startswith(('/*', '*', '//')):
            HARD.append(f'{path}:{i}: rule 9: double dereference')
        if 'AGC_ASSERT(' in s:
            n_asserts += 1
        if in_func is None:
            m = FUNC_RE.match(s)
            if m and not s.startswith(('#', 'typedef', 'return')) and path.endswith('.c'):
                if s.endswith('{') or (i < len(lines) and lines[i].strip() == '{'):
                    in_func = m.group(1)
                    start = i
                    depth = s.count('{') - s.count('}')
                    code_lines = 0
                    n_funcs += 1
                    continue
        else:
            depth += s.count('{') - s.count('}')
            if s and not s.startswith(('/*', '*', '//')):
                code_lines += 1
            if re.search(r'\b' + re.escape(in_func) + r'\s*\(', s) and not s.startswith(('/*', '*', '//')):
                HARD.append(f'{path}:{i}: rule 1: direct recursion in {in_func}')
            if depth <= 0:
                if code_lines > MAX_FUNC_LINES:
                    HARD.append(f'{path}:{start}: rule 4: {in_func} has {code_lines} code lines (> {MAX_FUNC_LINES})')
                in_func = None
    if ifs > 1 + (1 if path.endswith('agc_assert.h') else 0):
        WARN.append(f'{path}: rule 8: {ifs} conditional-compilation directives')
    if n_funcs and n_asserts / n_funcs < 2.0:
        WARN.append(f'{path}: rule 5: assertion density {n_asserts}/{n_funcs} functions = {n_asserts / n_funcs:.2f} (< 2.0)')


def main(argv):
    for root in argv[1:]:
        for dp, _, files in os.walk(root):
            for f in sorted(files):
                if f.endswith(('.c', '.h')):
                    check_file(os.path.join(dp, f))
    for w in WARN:
        print('WARN', w)
    for h in HARD:
        print('FAIL', h)
    print(f'{len(HARD)} hard violations, {len(WARN)} warnings')
    return 1 if HARD else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
