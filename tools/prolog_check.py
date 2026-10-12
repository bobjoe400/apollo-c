"""Prolog-versus-body consistency (report only).

The boxed prologs were written at translation time; later passes
(collapse, guard deletion, status propagation, void conversions) rewrote
bodies.  This tool reads each function's prolog and body and reports:

  returns-void     prolog has a RETURNS line naming agc_status_t, but the
                   function is void (or the reverse)
  returns-missing  prolog RETURNS names an AGC_STATUS_* (BAD_ARG, FULL,
                   ...) that neither the body nor any status it propagates
                   can return -- stale failure text.  What a function can
                   return is computed over the whole tree: the AGC_STATUS_*
                   literals it returns or assigns, plus, when it returns a
                   variable or a call, everything its status-returning
                   callees can return.  A function that takes an
                   agc_status_t (or passes one by address) can return
                   anything and is not checked.
  returns-extra    body returns an AGC_STATUS_* the prolog does not name
  flag-missing     prolog names an AGC flag (UPFLAG/DOWNFLAG text or an
                   AGC_FLAG_* name) the body never touches
  call-missing     prolog says the function calls/continues in/goes to
                   `name()` and the body has no call to `name`
  ctx-stale        prolog ARGUMENTS table lists a parameter the signature
                   no longer has
  empty-body       body has no statements (only comments)

Usage: python3 -I tools/prolog_check.py [--tsv OUT] FILE... (default src)
"""
import os
import re
import sys

import clang.cindex as ci

STATUS_RE = re.compile(r'AGC_STATUS_([A-Z_0-9]+)')
FLAG_RE = re.compile(r'\bAGC_FLAG_([A-Z0-9_]+)\b')
CALL_RE = re.compile(r'\b([a-z][a-z0-9_]+)\(\)')
STATUS_WORDS = ('BAD_ARG', 'FULL', 'BUSY', 'NOT_FOUND', 'POODOO', 'BAILOUT', 'ALARM', 'NO_CONVERGE')
ARG_ROW_RE = re.compile(r'^\s*\*\s+([A-Za-z_][A-Za-z_0-9]*)\s+\S.*\b(I|O|I/O)\b')


def call_claims(pro):
    """Names the prolog says the function calls.

    A `name()` mention is provenance, not a call claim, when it sits in an
    ARGUMENTS row (where the caller got the argument: `d  lm_dap_t *  I/O
    DAP erasables (lm_dap_state())`), names an object the function uses
    (`agc_align_ram()->qmaj`), or is the agent of a passive ("the vehicle
    selected by agc_state_init()"): in all three someone else made the call.
    """
    names = set()
    in_args = False
    for l in pro:
        if 'ARGUMENTS:' in l:
            in_args = True
            continue
        if 'RETURNS:' in l:
            in_args = False
        if in_args and ARG_ROW_RE.match(l):
            continue
        for m in CALL_RE.finditer(l):
            if l[m.end():m.end() + 2] == '->':
                continue
            if re.search(r'\bby\s+$', l[:m.start()]):
                continue
            names.add(m.group(1))
    return names


def prolog_before(lines, start_line):
    """Return the boxed comment immediately above `start_line` (1-based)."""
    i = start_line - 2
    if i < 0 or not lines[i].strip().endswith('*/'):
        return []
    j = i
    while j >= 0 and not lines[j].lstrip().startswith('/*'):
        j -= 1
    return lines[j:i + 1] if j >= 0 else []


def body_text(tu, cursor, src):
    ext = cursor.extent
    return src[ext.start.offset:ext.end.offset]


def strip_comments(text):
    return re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)


def check_function(cursor, lines, src, out):
    name = cursor.spelling
    line = cursor.location.line
    pro = prolog_before(lines, line)
    if not pro:
        return
    ptext = '\n'.join(pro)
    body = strip_comments(body_text(None, cursor, src))
    is_void = cursor.result_type.spelling == 'void'
    ret_line = [l for l in pro if 'RETURNS:' in l]
    ret_text = '\n'.join(pro[pro.index(ret_line[0]):]) if ret_line else ''
    # RETURNS vs signature
    if ret_line:
        says_status = 'agc_status_t' in ret_text
        if says_status and is_void:
            out.append((name, line, 'returns-void', 'RETURNS says agc_status_t, function is void'))
        if not says_status and not is_void and cursor.result_type.spelling == 'agc_status_t' and re.search(r'RETURNS:\s*none\b', ret_text, re.I):
            out.append((name, line, 'returns-void', 'RETURNS says none, function returns agc_status_t'))
    # statuses
    pro_st = set(STATUS_RE.findall(ret_text)) | {w for w in STATUS_WORDS if re.search(r'\b' + w + r'\b', ret_text)}
    if cursor.result_type.spelling == 'agc_status_t':
        can = POSSIBLE.get(func_key(cursor))
        if can is not None and 'ANY' not in can:
            for s in sorted(pro_st - can - {'OK'}):
                out.append((name, line, 'returns-missing', f'RETURNS names {s}, which neither the body nor any status it propagates can return'))
    # flags
    for f in sorted(set(FLAG_RE.findall(ptext))):
        if 'AGC_FLAG_' + f not in body:
            out.append((name, line, 'flag-missing', f'prolog names AGC_FLAG_{f}, body never touches it'))
    # calls
    for c in sorted(call_claims(pro)):
        if c == name or c in ('main',):
            continue
        if not re.search(r'\b' + re.escape(c) + r'\s*\(', body):
            out.append((name, line, 'call-missing', f'prolog says {c}(), body has no such call'))
    # ARGUMENTS table vs parameters
    params = {p.spelling for p in cursor.get_arguments()}
    in_args = False
    for l in pro:
        if 'ARGUMENTS:' in l:
            in_args = 'none' not in l.lower()
            continue
        if 'RETURNS:' in l:
            in_args = False
        if in_args:
            m = ARG_ROW_RE.match(l)
            if m and m.group(1) not in params and m.group(1) not in ('Argument',):
                out.append((name, line, 'ctx-stale', f'ARGUMENTS lists {m.group(1)}, not a parameter'))
    # empty body
    inner = body.strip()
    if inner.startswith('{') and inner.endswith('}') and not inner[1:-1].strip():
        out.append((name, line, 'empty-body', 'no statements'))


POSSIBLE = {}  # func_key -> set of status names the function can return


def func_key(cursor):
    """Static functions are keyed by name and file, external ones by name."""
    if cursor.linkage == ci.LinkageKind.INTERNAL:
        return (cursor.spelling, cursor.location.file.name if cursor.location.file else '')
    return cursor.spelling


def status_facts(fn_cursor):
    """Literal statuses, whether a non-literal return propagates, callees."""
    lits, callees, prop = set(), set(), False
    stack = list(fn_cursor.get_children())
    while stack:
        c = stack.pop()
        if c.kind == ci.CursorKind.RETURN_STMT:
            toks = ' '.join(t.spelling for t in c.get_tokens())
            lits.update(STATUS_RE.findall(toks))
            if re.search(r'[a-z_][a-z_0-9]*', re.sub(r'AGC_STATUS_[A-Z_0-9]+|\breturn\b', '', toks)):
                prop = True
        elif c.kind == ci.CursorKind.UNARY_OPERATOR and c.type.spelling.startswith('agc_status_t *'):
            lits.add('ANY')  # a status delivered through an out-parameter
        elif c.kind == ci.CursorKind.CALL_EXPR and c.referenced is not None and c.referenced.result_type.spelling == 'agc_status_t':
            callees.add(func_key(c.referenced))
        stack.extend(c.get_children())
    toks = [t.spelling for t in fn_cursor.get_tokens()]
    if prop:  # statuses assigned to the variable it returns
        for n, t in enumerate(toks):
            m = STATUS_RE.fullmatch(t)
            if m and not (n > 0 and toks[n - 1] in ('==', '!=')):
                lits.add(m.group(1))
    if any(a.type.spelling == 'agc_status_t' for a in fn_cursor.get_arguments()):
        lits.add('ANY')
    return lits, prop, callees


def parse(path):
    return ci.Index.create().parse(path, args=['-std=c11', '-Iinclude', '-DAGC_HOST_DEBUG'])


def definitions(tu, path):
    for c in tu.cursor.get_children():
        if c.kind == ci.CursorKind.FUNCTION_DECL and c.is_definition() and c.location.file and c.location.file.name == path:
            yield c


def build_possible(tus):
    facts = {}
    for path, tu in tus.items():
        for c in definitions(tu, path):
            if c.result_type.spelling == 'agc_status_t':
                facts[func_key(c)] = status_facts(c)
    POSSIBLE.update({k: set(v[0]) for k, v in facts.items()})
    for _ in range(len(facts)):  # fixpoint; bounded by the function count
        changed = False
        for k, (lits, prop, callees) in facts.items():
            if not prop:
                continue
            for cal in callees:
                add = POSSIBLE.get(cal, set()) - POSSIBLE[k]
                if add:
                    POSSIBLE[k] |= add
                    changed = True
        if not changed:
            break


def check_file(path, tu, out):
    src = open(path).read()
    lines = src.split('\n')
    for c in definitions(tu, path):
        check_function(c, lines, src, out)


def main():
    args = sys.argv[1:]
    tsv = None
    if args and args[0] == '--tsv':
        tsv, args = args[1], args[2:]
    tree = sorted(os.path.join(d, f) for d, _, fs in os.walk('src') for f in fs if f.endswith('.c'))
    files = args or tree
    tus = {p: parse(p) for p in sorted(set(tree) | set(files))}
    build_possible(tus)
    out = []
    for p in sorted(files):
        file_out = []
        check_file(p, tus[p], file_out)
        out.extend((p,) + r for r in file_out)
    kinds = {}
    for r in out:
        kinds[r[3]] = kinds.get(r[3], 0) + 1
    for p, name, line, kind, msg in out:
        print(f'{p}:{line}: {kind}: {name}: {msg}')
    print(f'--- {len(out)} findings: ' + ', '.join(f'{k} {v}' for k, v in sorted(kinds.items())))
    if tsv:
        with open(tsv, 'w') as fh:
            fh.write('file\tline\tkind\tfunction\tmessage\n')
            for p, name, line, kind, msg in out:
                fh.write(f'{p}\t{line}\t{kind}\t{name}\t{msg}\n')


if __name__ == '__main__':
    main()
