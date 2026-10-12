"""Turn function prologs pasted inside function bodies (an inline_one.py
artefact) into section comments.

A prolog box (`/****...` ... `****/`) indented inside a body keeps only its
description lines (those before FUNCTION NAME:) as a block comment at the
same indentation; ARGUMENTS/RETURNS belong to a function, not a section.

Usage: python3 -I tools/unpaste_prologs.py FILE...   (edits in place, prints counts)
"""
import re
import sys

OPEN = re.compile(r'^( {3,})/\*{10,}\s*$')
CLOSE = re.compile(r'^ {3,}\*{10,}/\s*$')


def rewrite(path):
    lines = open(path).read().split('\n')
    out, i, n = [], 0, 0
    while i < len(lines):
        m = OPEN.match(lines[i])
        if m:
            j = i + 1
            while j < len(lines) and not CLOSE.match(lines[j]):
                j += 1
            block = lines[i + 1:j]
            if j < len(lines) and any('FUNCTION NAME:' in b for b in block):
                ind = m.group(1)
                desc = []
                for b in block:
                    if 'FUNCTION NAME:' in b:
                        break
                    t = re.sub(r'^\s*\*\s?', '', b).rstrip()
                    if t:
                        desc.append(t)
                if desc:
                    out.append(f'{ind}/* ' + desc[0])
                    for d in desc[1:]:
                        out.append(f'{ind} * ' + d)
                    out[-1] += ' */'
                n += 1
                i = j + 1
                continue
        out.append(lines[i])
        i += 1
    if n:
        open(path, 'w').write('\n'.join(out))
    return n


def main():
    total = 0
    for p in sys.argv[1:]:
        k = rewrite(p)
        if k:
            print(f'{p}: {k}')
            total += k
    print(f'total: {total}')


if __name__ == '__main__':
    main()
