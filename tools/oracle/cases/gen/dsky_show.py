#!/usr/bin/env python3
"""Render DSPTAB words as the DSKY shows them, AGC (dsky*.txt) next to C.
    ORACLE_VERBOSE=1 build/tests/oracle/dsky | python3 -I dsky_show.py [tag...]
(run from tests/oracle so the test finds its case files)."""
import os, sys
RELAY = {0o25: '0', 0o3: '1', 0o31: '2', 0o33: '3', 0o17: '4', 0o36: '5',
         0o34: '6', 0o23: '7', 0o35: '8', 0o37: '9', 0: ' '}

def show(words):
    w = [x if x < 0o40000 else 0o77777 - x for x in words]
    ch = lambda x: RELAY.get(x, '?')
    lt = lambda k: ch((w[k] >> 5) & 31)
    rt = lambda k: ch(w[k] & 31)
    sg = lambda kp, km: '+' if (w[kp] >> 10) & 1 else ('-' if (w[km] >> 10) & 1 else ' ')
    r1 = sg(6, 5) + rt(7) + lt(6) + rt(6) + lt(5) + rt(5)
    r2 = sg(4, 3) + lt(4) + rt(4) + lt(3) + rt(3) + lt(2)
    r3 = sg(1, 0) + rt(2) + lt(1) + rt(1) + lt(0) + rt(0)
    return 'V%s%s N%s%s |%s|%s|%s|' % (lt(9), rt(9), lt(8), rt(8), r1, r2, r3)

def agc_words():
    here = os.path.dirname(os.path.abspath(__file__))
    res, cur = {}, None
    for f in ('dsky.txt', 'dsky_lm.txt'):
        for line in open(os.path.join(here, '..', f)):
            k, _, v = line.strip().partition(' ')
            if k == 'CASE':
                cur = res.setdefault(v, [0] * 11)
            elif k.startswith('DSPTAB+'):
                cur[int(k[7:])] = int(v, 8)
    return res

def main():
    agc = agc_words()
    want = set(sys.argv[1:])
    for line in sys.stdin:
        f = line.split()
        if not f or f[0] != 'CDSPTAB' or (want and f[1] not in want):
            continue
        c = [int(x, 8) for x in f[2:]]
        a = agc.get(f[1])
        mark = '' if c == a else '   <-- differs'
        print('%-14s AGC %s' % (f[1], show(a)))
        print('%-14s C   %s%s' % ('', show(c), mark))

if __name__ == '__main__':
    main()
