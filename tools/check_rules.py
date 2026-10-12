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
STDLIB_VOID_OK = {'fprintf', 'printf', 'snprintf', 'fflush', 'fclose', 'memset', 'memcpy', 'memmove', 'nanosleep', 'setvbuf', 'fputs', 'puts'}
FUNC_RE = re.compile(r'^[A-Za-z_][\w\s\*]*\s\**([A-Za-z_]\w*)\s*\([^;]*\)\s*\{?\s*$')
# a statement such as `if (x)` inside a function whose multi-line signature
# FUNC_RE did not recognise is not a function definition
C_KEYWORDS = {'if', 'while', 'for', 'switch', 'return', 'sizeof', 'else', 'do'}
HARD = []
WARN = []
TOTAL_FUNCS = 0
TOTAL_ASSERTS = 0


def check_file(path):
    with open(path, encoding='utf-8', errors='replace') as fh:
        lines = fh.readlines()
    rule4_exempt = any('GENERATED-DISPATCH: rule 4 exempt' in l for l in lines[:5])
    if rule4_exempt and not path.endswith('src/exec/dispatch.c'):
        HARD.append(f'{path}:1: rule 4 exemption marker is only allowed in src/exec/dispatch.c')
        rule4_exempt = False
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
        vm = re.search(r'\(\s*void\s*\)\s*([A-Za-z_][A-Za-z_0-9]*)\s*\(', s)
        if vm and not s.startswith(('/*', '*', '//')) and '/src/' in '/' + path:
            if vm.group(1) not in STDLIB_VOID_OK or '/*' not in s:
                HARD.append(f'{path}:{i}: rule 7: (void)-cast of {vm.group(1)}(); check the result, handle it as the AGC does, or make the callee void (stdlib calls need a justifying comment)')
        if re.search(r'\(\s*\*\s*\w+\s*\)\s*\(', s) and not s.startswith(('/*', '*', '//')):
            HARD.append(f'{path}:{i}: rule 9: function pointer')
        if re.search(r'\*\*\s*[A-Za-z_]', s) and not s.startswith(('/*', '*', '//')):
            HARD.append(f'{path}:{i}: rule 9: double dereference')
        # Count every assertion: one-statement-per-line formatting puts
        # `!AGC_ASSERT(a) || !AGC_ASSERT(b)` on one line (format2).
        n_asserts += len(re.findall(r'\bAGC_ASSERT\(', s))
        if in_func is None:
            m = FUNC_RE.match(s)
            if m and m.group(1) not in C_KEYWORDS and not s.startswith(('#', 'typedef', 'return')) and path.endswith('.c'):
                if s.endswith('{') or (i < len(lines) and lines[i].strip() == '{'):
                    in_func = m.group(1)
                    func_is_void = bool(re.match(r'^(static\s+)?(inline\s+)?void\b', s))
                    func_is_status = 'agc_status_t' in s.split(in_func)[0]
                    seen_asserts = set()  # conditions guarded since the last real statement
                    start = i
                    depth = s.count('{') - s.count('}')
                    code_lines = 0
                    n_funcs += 1
                    continue
        else:
            depth += s.count('{') - s.count('}')
            if s and s not in ('{', '}') and not s.startswith(('/*', '*', '//')):
                code_lines += 1  # brace-only lines carry no statement (Allman style)
            if s.startswith('if (!AGC_ASSERT') and not func_is_status:
                # the whole guard block (to its closing brace), not just the
                # next line: `{ agc_relint(); return; }` is as silent
                blk, d, k = [], 0, i - 1
                while k < len(lines) and k < i + 12:
                    t = lines[k].strip()
                    blk.append(t)
                    d += t.count('{') - t.count('}')
                    if d <= 0 and '{' in ' '.join(blk) or (d == 0 and t.endswith(';') and k > i - 1):
                        break
                    k += 1
                nxt = ' '.join(blk[1:])
                rm = re.search(r'return\s*(\(?[^;]*\)?)\s*;', nxt)
                if rm and re.search(r'agc_(alarm|bailout|poodoo|curtains|ccshole|exec_varalarm|prioalarm)\(', nxt):
                    rm = None  # the failure is raised as the AGC raises it
                if rm:
                    if func_is_void:
                        HARD.append(f'{path}:{i}: rule 7: silent failure: void {in_func}() returns on a failed assertion; delete a guaranteed guard or return agc_status_t')
                    else:
                        HARD.append(f'{path}:{i}: rule 7: sentinel failure: {in_func}() returns {rm.group(1) or "a value"} on a failed assertion; delete a guaranteed guard or return agc_status_t with an out-parameter')
            am = re.search(r'AGC_ASSERT\((.*)\)\s*\)?\s*$', s)
            if am and s.startswith('if (!AGC_ASSERT'):
                cond = re.sub(r'\s+', '', am.group(1))
                if cond in seen_asserts:
                    HARD.append(f'{path}:{i}: rule 5: back-to-back duplicate assertion in {in_func}')
                seen_asserts.add(cond)
            elif s and s not in ('{', '}') and not s.startswith(('/*', '*', '//', 'return')):
                seen_asserts = set()  # a real statement may change the fact; a later re-check is a postcondition
            if re.search(r'\b' + re.escape(in_func) + r'\s*\(', s) and not s.startswith(('/*', '*', '//')):
                HARD.append(f'{path}:{i}: rule 1: direct recursion in {in_func}')
            if depth <= 0:
                if code_lines > MAX_FUNC_LINES and not rule4_exempt:
                    HARD.append(f'{path}:{start}: rule 4: {in_func} has {code_lines} code lines (> {MAX_FUNC_LINES})')
                in_func = None
    if ifs > 1 + (1 if path.endswith('agc_assert.h') else 0):
        WARN.append(f'{path}: rule 8: {ifs} conditional-compilation directives')
    global TOTAL_FUNCS, TOTAL_ASSERTS
    TOTAL_FUNCS += n_funcs
    TOTAL_ASSERTS += n_asserts


def main(argv):
    for root in argv[1:]:
        for dp, _, files in os.walk(root):
            for f in sorted(files):
                if f.endswith(('.c', '.h')):
                    low = f.lower()
                    if re.match(r'^(v|p|r|n)[0-9]{2}([_a-z0-9]*)?\.(c|h)$', low):
                        HARD.append(f'{os.path.join(dp, f)}: naming: file name carries a program/verb/routine number (docs/NAMING.md)')
                    if any(k in low for k in ('glue', 'shim', '_ext.', 'pending', 'adapter', 'wrapper')) or low == 'ext.h':
                        WARN.append(f'{os.path.join(dp, f)}: scaffolding file (glue/shim/ext/pending); must be gone after integration')
                    check_file(os.path.join(dp, f))
    print(f'rule 5 (informational): project assertion density {TOTAL_ASSERTS}/{max(TOTAL_FUNCS,1)} = {TOTAL_ASSERTS / max(TOTAL_FUNCS,1):.2f}')
    for w in WARN:
        print('WARN', w)
    for h in HARD:
        print('FAIL', h)
    print(f'{len(HARD)} hard violations, {len(WARN)} warnings')
    return 1 if HARD else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
