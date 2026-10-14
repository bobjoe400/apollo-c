#!/usr/bin/env python3
"""one_liners.py -- classify every one-line function (docs/CODING_STANDARD.md,
"One-line functions"; docs/history/assignments/status-handling-2.md step 6).

Usage (repo root):
    python3 -I tools/one_liners.py [OUT.tsv]   # default docs/overview/data/one_liners.tsv

A one-line function has a body of exactly one statement: a return of an
expression or one expression statement.  Each is classified on the
libclang AST:

  ACCESSOR  the statement reads or writes file-scope or static storage, or
            calls a function that does (transitively): the module boundary
            of rule 6, where the entry check of a value lives.  Kept.
  NAMED     no state access, but the header comment names an AGC label
            (the name carries an AGC meaning the expression does not show:
            an instruction, an overflow-corrected store, a scale
            conversion, a channel bit).  Kept.
  PLAIN     an expression over its own parameters and constants (calls
            only to state-free functions) with no AGC label: inlined at
            every caller and deleted, whatever the caller count.

Pointer parameters count as the function's own parameters (agc_interp_t
*ip, a caller's struct): touching them is not module state.  Callers are
counted over src/ and tests/.
"""
import collections
import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import callgraph  # noqa: E402
import cgast  # noqa: E402

K = cgast.K
# Library functions without hidden state (the rest of libc is impure).
PURE_LIB = {'abs', 'labs', 'llabs', 'floor', 'ceil', 'ldexp', 'fabs', 'sqrt', 'strlen', 'strcmp',
            'strncmp', 'memcmp', 'AGC_ASSERT', 'agc_assert_report'}


# Hand decisions (one line each): one-line functions whose name carries an
# AGC meaning their header comment states without a code label.
WORDS = 'word structure: the AGC word(s) the value is held in'
INSTR = 'instruction semantics'
HAND = {
    'agc_w_join': WORDS, 'agc_w3_value': WORDS, 'agc_dp_high': WORDS, 'agc_dp_low': WORDS,
    'agc_dp_from_sp': WORDS, 'agc_tp_from_dp': WORDS, 'agc_tp_to_dp': WORDS,
    'agc_dp_from_words': WORDS, 'agc_tp_from_words': WORDS, 'noun_raw_word': WORDS,
    'pin_ones_negative': WORDS, 'to16': WORDS, 'lm_dap_dpval': WORDS,
    'agc_lm_words_value': WORDS, 'rz_word_of': WORDS, 'dp_word': WORDS + ' (downlist order)',
    'vec_word': WORDS + ' (downlist order)',
    'agc_sp_add': INSTR + ' (AD)', 'agc_sp_mul': INSTR + ' (MP)',
    'agc_dp_add_raw': INSTR + ' (DAS)', 'agc_dp_sub_raw': INSTR + ' (DAS of the complement)',
    'csm_dap_mp_hi': INSTR + ' (MP, A)', 'csm_dap_mp_lo': INSTR + ' (MP, L)',
    'csm_dap_words_mp': INSTR + ' (MP, A and L)', 'dmp_shl': INSTR + ' (DMP SLn)',
    'pin_dmp_tp': INSTR + ' (DMP)', 'lm_dap_mp': INSTR + ' (MP)',
    'lm_dap_ovf': INSTR + ' (TS overflow skip)', 'lm_dap_ddoubl': INSTR + ' (DDOUBL)',
    'beyond': 'ENTERQR breakout test',
    'agc_sp_to_double': 'scale conversion with its scale 2^N',
    'agc_dp_to_double': 'scale conversion with its scale 2^N',
    'pin_frac_of_digits': 'decimal display conversion (digits to the fraction pinball shows)',
    'agc_flag_word': 'flagword index of FLAGWRDn', 'agc_flag_bit': 'AGC bit number of a flag',
    'agc_dsky_vn': 'the CAF VxxNyy coding of a verb-noun pair',
}


def walk(n):
    yield n
    for c in n.get_children():
        yield from walk(c)


def is_const(t):
    """A const object (a table of constants is not state)."""
    while t.kind in (cgast.ci.TypeKind.CONSTANTARRAY, cgast.ci.TypeKind.INCOMPLETEARRAY):
        t = t.element_type
    return t.is_const_qualified()


def is_state(d):
    """A variable with static storage (file scope, or a static local) that
    is not a const table."""
    if d is None or d.kind != K.VAR_DECL or is_const(d.type):
        return False
    if d.semantic_parent is not None and d.semantic_parent.kind == K.TRANSLATION_UNIT:
        return True
    return d.storage_class == ci_static()


def ci_static():
    return cgast.ci.StorageClass.STATIC


def one_statement(fn):
    b = cgast.body(fn)
    if b is None:
        return None
    st = cgast.kids(b)
    if len(st) != 1:
        return None
    s = st[0]
    if s.kind == K.RETURN_STMT:
        return s if cgast.kids(s) else None
    if s.kind in (K.CALL_EXPR, K.BINARY_OPERATOR, K.COMPOUND_ASSIGNMENT_OPERATOR, K.UNARY_OPERATOR,
                  K.UNEXPOSED_EXPR):
        return s
    return None


def scan():
    """Per function: file, line, one-statement flag, direct state access,
    callees, header comment; and call counts per callee."""
    funcs = {}
    callers = collections.Counter()
    files = sorted(glob.glob('src/**/*.c', recursive=True)) + sorted(glob.glob('tests/**/*.c', recursive=True))
    for path in files:
        with open(path, encoding='utf-8') as fh:
            lines = fh.readlines()
        tu = cgast.parse(path)
        for fn in cgast.functions(tu, path):
            b = cgast.body(fn)
            calls = set()
            state = False
            for n in walk(b) if b is not None else []:
                if n.kind == K.CALL_EXPR and n.referenced is not None:
                    calls.add(n.referenced.spelling)
                    callers[n.referenced.spelling] += 1
                elif n.kind == K.DECL_REF_EXPR and is_state(n.referenced):
                    state = True
            if not path.startswith('src/'):
                continue
            s = one_statement(fn)
            funcs[fn.spelling] = {
                'file': path, 'line': fn.extent.start.line, 'one': s is not None,
                'state': state, 'calls': calls,
                'comment': cgast.header_comment(lines, fn.extent.start.line),
                'static': fn.storage_class == ci_static(),
            }
    return funcs, callers


def impure_set(funcs):
    """Functions that touch module state, directly or through a callee
    (fixpoint; unknown external functions are impure unless PURE_LIB)."""
    bad = {n for n, f in funcs.items() if f['state']}
    changed = True
    while changed:
        changed = False
        for n, f in funcs.items():
            if n in bad:
                continue
            for c in f['calls']:
                if c in bad or (c not in funcs and c not in PURE_LIB and not c.startswith('__builtin')):
                    bad.add(n)
                    changed = True
                    break
    return bad


def main(argv):
    out = argv[1] if len(argv) > 1 else os.path.join(ROOT, 'docs', 'overview', 'data', 'one_liners.tsv')
    os.chdir(ROOT)
    labels, _ = callgraph.agc_labels()
    funcs, callers = scan()
    bad = impure_set(funcs)
    rows = []
    for name, f in sorted(funcs.items(), key=lambda kv: (kv[1]['file'], kv[1]['line'])):
        if not f['one']:
            continue
        if name in bad:
            cls = 'ACCESSOR'
            why = 'reads/writes module state' if f['state'] else 'calls ' + ', '.join(sorted(c for c in f['calls'] if c in bad or c not in funcs)[:3])
        elif callgraph.has_agc_label(f['comment'], labels):
            cls = 'NAMED'
            why = 'AGC label: ' + re.sub(r'\s+', ' ', cgast.first_comment_line(f['comment']))[:80]
        elif name in HAND:
            cls = 'NAMED'
            why = 'by hand: ' + HAND[name]
        else:
            cls = 'PLAIN'
            why = 'expression over its parameters and constants'
        rows.append((f['file'], f['line'], name, cls, callers.get(name, 0), why))
    with open(out, 'w') as fh:
        fh.write('file:line\tfunction\tclass\tcalls\twhy\n')
        for r in rows:
            fh.write(f'{r[0]}:{r[1]}\t{r[2]}\t{r[3]}\t{r[4]}\t{r[5]}\n')
    c = collections.Counter(r[3] for r in rows)
    print(' '.join(f'{k} {c[k]}' for k in sorted(c)), f'(of {len(rows)} one-line functions)')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
