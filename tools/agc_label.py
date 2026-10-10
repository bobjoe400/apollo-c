#!/usr/bin/env python3
"""agc_label.py -- exact AGC label lookup across both ropes.

Usage: python3 -I tools/agc_label.py LABEL [LABEL...] [--after N] [--rope NAME]
       python3 -I tools/agc_label.py --noun 61 | --verb 97 | --flag MOONFLAG
       python3 -I tools/agc_label.py --grep REGEX   (comment-aware search)

Labels in yaYUL source start in column 0, so the match is exact, not a
substring guess. Prints: rope, file:line, the comment block immediately
above the label, and the next N lines (default 25). AGC_SRC env var or the
default scratchpad clone locates the sources.
"""
import os
import re
import sys

DEFAULT_SRC = ('/tmp/claude-1000/-home-cooperdesktop-git-apollo-c/'
               '89c6630f-d76a-459d-b8fc-f5cc06c736bc/scratchpad/src/Apollo-11')
ROPES = ('Comanche055', 'Luminary099')


def files(src, rope):
    d = os.path.join(src, rope)
    return [os.path.join(d, f) for f in sorted(os.listdir(d)) if f.endswith('.agc')]


def show(path, lines, i, after, rope):
    start = i
    while start > 0 and lines[start - 1].startswith('#'):
        start -= 1
    print(f'=== {rope} {os.path.basename(path)}:{i + 1}')
    for k in range(start, min(len(lines), i + after + 1)):
        print(f'{k + 1:6d}  {lines[k].rstrip()}')
    print()


def main(argv):
    src = os.environ.get('AGC_SRC', DEFAULT_SRC)
    after, rope_filter, mode, targets = 25, None, 'label', []
    it = iter(argv[1:])
    for a in it:
        if a == '--after':
            after = int(next(it))
        elif a == '--rope':
            rope_filter = next(it)
        elif a in ('--noun', '--verb', '--flag', '--grep'):
            mode = a[2:]
            targets.append(next(it))
        else:
            targets.append(a)
    if not targets:
        print(__doc__)
        return 2
    pats = []
    for t in targets:
        if mode == 'label':
            pats.append(re.compile(r'^' + re.escape(t) + r'(\s|$)'))
        elif mode == 'noun':
            pats.append(re.compile(r'^#.*\b(N' + t + r'|NOUN\s+' + t + r')\b', re.I))
        elif mode == 'verb':
            pats.append(re.compile(r'^#.*\b(V' + t + r'|VERB\s+' + t + r')\b', re.I))
        elif mode == 'flag':
            pats.append(re.compile(r'^' + re.escape(t) + r'\s|^#.*\b' + re.escape(t) + r'\b'))
        else:
            pats.append(re.compile(t))
    hits = 0
    for rope in ROPES:
        if rope_filter and rope != rope_filter:
            continue
        for path in files(src, rope):
            with open(path, encoding='utf-8', errors='replace') as fh:
                lines = fh.readlines()
            for i, line in enumerate(lines):
                if any(p.search(line) for p in pats):
                    show(path, lines, i, after if mode == 'label' else 3, rope)
                    hits += 1
                    if hits > 200:
                        print('... truncated')
                        return 0
    if hits == 0:
        print('no match')
    return 0 if hits else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv))
