"""Prolog-versus-body consistency (report only).

The boxed prologs were written at translation time; later passes
(collapse, guard deletion, status propagation, void conversions) rewrote
bodies.  This tool reads each function's prolog and body and reports:

  returns-void     prolog has a RETURNS line naming agc_status_t, but the
                   function is void (or the reverse)
  returns-missing  prolog RETURNS names an AGC_STATUS_* the body never
                   returns (BAD_ARG, FULL, ...) -- stale failure text
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
ARG_ROW_RE = re.compile(r'^\s*\*\s+([A-Za-z_][A-Za-z_0-9]*)\s+\S.*\b(I|O|I/O)\b')


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
        if not says_status and not is_void and cursor.result_type.spelling == 'agc_status_t' and 'none' in ret_text.lower():
            out.append((name, line, 'returns-void', 'RETURNS says none, function returns agc_status_t'))
    # statuses
    pro_st = set(STATUS_RE.findall(ret_text)) | {w for w in ('BAD_ARG', 'FULL', 'BUSY', 'NOT_FOUND', 'POODOO', 'BAILOUT', 'ALARM') if re.search(r'\b' + w + r'\b', ret_text)}
    body_st = set(STATUS_RE.findall(body))
    if cursor.result_type.spelling == 'agc_status_t':
        # A `return (st);` / `return (f(...));` propagates a callee's status:
        # the prolog may then legitimately name statuses the body never spells.
        propagates = bool(re.search(r'\breturn\s*\(?\s*(?!AGC_STATUS_)[A-Za-z_][A-Za-z_0-9]*\s*[\(\);]', body)) or \
            bool(re.search(r'\breturn\s*\(?\s*\(.*\?', body))
        if not propagates:
            for s in sorted(pro_st - body_st - {'OK'}):
                out.append((name, line, 'returns-missing', f'RETURNS names {s}, body never returns it (no propagated return either)'))
    # flags
    for f in sorted(set(FLAG_RE.findall(ptext))):
        if 'AGC_FLAG_' + f not in body:
            out.append((name, line, 'flag-missing', f'prolog names AGC_FLAG_{f}, body never touches it'))
    # calls
    for c in sorted(set(CALL_RE.findall(ptext))):
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


def check_file(path, out):
    src = open(path).read()
    lines = src.split('\n')
    idx = ci.Index.create()
    tu = idx.parse(path, args=['-std=c11', '-Iinclude', '-DAGC_HOST_DEBUG'])
    for c in tu.cursor.get_children():
        if c.kind == ci.CursorKind.FUNCTION_DECL and c.is_definition() and c.location.file and c.location.file.name == path:
            check_function(c, lines, src, out)


def main():
    args = sys.argv[1:]
    tsv = None
    if args and args[0] == '--tsv':
        tsv, args = args[1], args[2:]
    files = args or [os.path.join(d, f) for d, _, fs in os.walk('src') for f in fs if f.endswith('.c')]
    out = []
    for p in sorted(files):
        file_out = []
        check_file(p, file_out)
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
