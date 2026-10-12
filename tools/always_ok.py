"""Report agc_status_t functions that can only return AGC_STATUS_OK.

Rule 7 (docs/CODING_STANDARD.md): a routine that cannot fail is void;
status plumbing at its callers is noise.  A function is reported when
every `return` in its body is the literal AGC_STATUS_OK and no return
propagates a value (a variable, a call, a conditional).  Report only.

Usage: python3 -I tools/always_ok.py [--tsv OUT] [FILE...]  (default src)
"""
import os
import re
import sys

import clang.cindex as ci

RET_RE = re.compile(r'\breturn\b\s*([^;]*);')


def strip_comments(text):
    return re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)


def returns_of(body):
    return [m.group(1).strip() for m in RET_RE.finditer(body)]


def is_ok_literal(expr):
    return re.fullmatch(r'\(?\s*AGC_STATUS_OK\s*\)?', expr) is not None


def check_file(path, out):
    src = open(path).read()
    idx = ci.Index.create()
    tu = idx.parse(path, args=['-std=c11', '-Iinclude', '-DAGC_HOST_DEBUG'])
    for c in tu.cursor.get_children():
        if c.kind != ci.CursorKind.FUNCTION_DECL or not c.is_definition():
            continue
        if not c.location.file or c.location.file.name != path:
            continue
        if c.result_type.spelling != 'agc_status_t':
            continue
        ext = c.extent
        body = strip_comments(src[ext.start.offset:ext.end.offset])
        rets = returns_of(body)
        if rets and all(is_ok_literal(r) for r in rets):
            static = 'static' if c.storage_class == ci.StorageClass.STATIC else 'public'
            out.append((path, c.location.line, c.spelling, static, len(rets)))


def main():
    args = sys.argv[1:]
    tsv = None
    if args and args[0] == '--tsv':
        tsv, args = args[1], args[2:]
    files = args or [os.path.join(d, f) for d, _, fs in os.walk('src') for f in fs if f.endswith('.c')]
    out = []
    for p in sorted(files):
        check_file(p, out)
    for p, line, name, static, n in out:
        print(f'{p}:{line}: {name} ({static}) always returns AGC_STATUS_OK ({n} return(s))')
    pub = sum(1 for r in out if r[3] == 'public')
    print(f'--- {len(out)} always-OK status functions ({pub} public, {len(out) - pub} static)')
    if tsv:
        with open(tsv, 'w') as fh:
            fh.write('file\tline\tfunction\tlinkage\treturns\n')
            for r in out:
                fh.write('\t'.join(str(x) for x in r) + '\n')


if __name__ == '__main__':
    main()
