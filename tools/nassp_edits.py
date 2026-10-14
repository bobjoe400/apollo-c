#!/usr/bin/env python3
"""nassp_edits.py -- `make nassp-mission`: the NASSP milestone tables, generated from the edit CSVs.

A NASSP mission run (docs/modules/nassp-integration.md, "How to run it") writes one edit CSV per
vehicle and run: every second the GET, the LGC/CMC words (MODREG, FAILREG, channels 11 12 14 30 33,
TIME, DSPTAB) and the vessel's truth from Orbiter (altitude, speeds, position, mass, engine level,
attitude, ...). For each milestone this script pairs the stock run's CSV with ours and prints

  1. the milestone table: the events found the same way in both files (ignition, throttle-up,
     program changes, displays at marks, cut-off, contact, orbit),
  2. per quantity of sim/nassp/edits.tsv: the tolerance, max |ours - stock| and where, and the
     first time out of tolerance (rows paired by time since ignition, nearest within 0.6 s, from
     ignition on),
  3. the major-mode changes and alarms of each run.

Usage: nassp_edits.py STOCK.csv OURS.csv MILESTONE [--tolerances sim/nassp/edits.tsv]
       nassp_edits.py --runs DIR            (every milestone found in DIR: ref_<m>_H/ and
                                             agc_<m>_H/ holding apollo-c-<m>-edits.csv)
Exit status 1 when a quantity is out of tolerance or a milestone event is missing in ours.
Standard library only; run with python3 -I.
"""
import csv
import os
import sys

# DSKY relay codes of the digits (DSPTAB words: bits 10-6 and 5-1).
DIGIT = {0: ' ', 21: '0', 3: '1', 25: '2', 27: '3', 15: '4', 30: '5', 28: '6', 19: '7', 29: '8', 31: '9'}


def dsky(words):
    """PROG/VERB/NOUN/R1-R3 of the 12 DSPTAB words (DSPTAB+10 is PROG, +9 VERB, +8 NOUN)."""
    rows = {i + 1: w for i, w in enumerate(words[:11])}

    def c(r):
        return DIGIT.get((rows.get(r, 0) >> 5) & 31, '?')

    def d(r):
        return DIGIT.get(rows.get(r, 0) & 31, '?')

    def sign(rp, rm):
        p, m = rows.get(rp, 0) & 0o2000, rows.get(rm, 0) & 0o2000
        return '+' if p and not m else '-' if m and not p else ' '
    r1 = sign(7, 6) + d(8) + c(7) + d(7) + c(6) + d(6)
    r2 = sign(5, 4) + c(5) + d(5) + c(4) + d(4) + c(3)
    r3 = sign(2, 1) + d(3) + c(2) + d(2) + c(1) + d(1)
    return 'PROG %s%s VERB %s%s NOUN %s%s R1 %s R2 %s R3 %s' % (c(11), d(11), c(10), d(10), c(9), d(9), r1, r2, r3)


def load(path):
    rows = list(csv.DictReader(open(path)))
    for r in rows:
        r['_t'] = float(r['t'])
    return rows


def num(r, k):
    v = r.get(k, '')
    if v in ('', None):
        return None
    if k in ('cdux', 'cduy', 'cduz'):
        c = int(float(v))
        c = c - 32768 if c >= 16384 else c
        return c * 360.0 / 32768
    try:
        return float(v)
    except ValueError:
        return None


def oct_(r, k):
    v = r.get(k, '')
    return int(v, 8) if v not in ('', None) else 0


def display(r):
    return dsky([oct_(r, 'dsptab%d' % i) for i in range(12)])


def first(rows, f):
    for r in rows:
        if f(r):
            return r
    return None


def nearest(rows, t):
    best = min(rows, key=lambda r: abs(r['_t'] - t))
    return best if abs(best['_t'] - t) <= 0.6 else None


def engine(rows):
    """The engine column of the milestone (dps for the descent, eng for the ascent)."""
    return 'dps' if 'dps' in rows[0] else 'eng'


def ignition(rows):
    e = engine(rows)
    return first(rows, lambda r: (num(r, e) or 0) > 0.05)


def ev_pdi(rows):
    ev = {}
    ign = ignition(rows)
    if not ign:
        return ev
    t0 = ign['_t']
    ev['ignition (DPS > 5%), GET s'] = '%.1f' % float(ign['get'])
    thr = first(rows, lambda r: (num(r, 'dps') or 0) > 0.5)
    if thr:
        ev['throttle-up (DPS > 50%), TIG+ s'] = '%.1f' % (thr['_t'] - t0)
    for mm, name in ((64, 'P64'), (65, 'P65'), (66, 'P66')):
        r = first(rows, lambda r, mm=mm: int(r['modreg'], 8) == mm)
        if r:
            ev[name + ', TIG+ s'] = '%.1f (galt %.0f m, vh %.1f m/s)' % (r['_t'] - t0, num(r, 'galt'), num(r, 'vh'))
    for m in (5, 8):
        r = nearest(rows, t0 + 60 * m)
        if r:
            ev['DSKY at TIG+%dm' % m] = display(r)
    c = first(rows, lambda r: r.get('gc') == '1' or (num(r, 'galt') is not None and 0 <= num(r, 'galt') < 1.0))
    if c:
        p = rows[max(rows.index(c) - 1, 0)]
        ev['ground contact, TIG+ s'] = '%.1f (hdot %.2f m/s, vh %.2f m/s 1 s before)' % (c['_t'] - t0, num(p, 'hdot'), num(p, 'vh'))
        last = rows[-1]
        ev['touchdown lng / lat, deg'] = '%.4f / %.4f' % (num(last, 'lng'), num(last, 'lat'))
    return ev


def ev_liftoff(rows):
    ev = {}
    ign = ignition(rows)
    if not ign:
        return ev
    t0 = ign['_t']
    ev['ignition (APS > 5%), GET s'] = '%.1f' % float(ign['get'])
    off = first(rows, lambda r: r['_t'] > t0 and (num(r, 'eng') or 0) < 0.5)
    if off:
        ev['cut-off, TIG+ s'] = '%.1f (vh %.1f m/s, hdot %.1f m/s)' % (off['_t'] - t0, num(off, 'vh'), num(off, 'hdot'))
        o = nearest(rows, off['_t'] + 10) or off
        ev['orbit radius peri x apo, km'] = '%.2f x %.2f' % (num(o, 'peri') / 1000, num(o, 'apo') / 1000)
        ev['mass after cut-off, kg'] = '%.1f' % num(o, 'mass')
        r = nearest(rows, off['_t'] - 30)
        if r:
            ev['DSKY 30 s before cut-off'] = display(r)
    return ev


EVENTS = {'pdi': ev_pdi, 'liftoff': ev_liftoff}


def tolerances(path, milestone):
    tol = {}
    for ln in open(path):
        if ln.startswith('#') or not ln.strip():
            continue
        f = ln.rstrip('\n').split('\t')
        if f[0] == milestone:
            tol[f[1]] = float(f[2])
    return tol


def changes(rows, key):
    out, last = [], None
    for r in rows:
        v = key(r)
        if v != last:
            out.append((r['_t'], v))
            last = v
    return out


def compare(stock, ours, milestone, tolfile):
    """Print the tables; return the number of failures."""
    fails = 0
    ea, eb = EVENTS[milestone](stock), EVENTS[milestone](ours)
    print('## %s\n\n| milestone | stock | ours |\n|---|---|---|' % milestone)
    for k in list(dict.fromkeys(list(ea) + list(eb))):
        print('| %s | %s | %s |' % (k, ea.get(k, 'not reached'), eb.get(k, 'not reached')))
        fails += 0 if k in eb else 1
    ia, ib = ignition(stock), ignition(ours)
    if not (ia and ib):
        return fails + 1
    print('\n| quantity | tolerance | max abs diff | at TIG+ s | first TIG+ s out |\n|---|---|---|---|---|')
    for k, tol in tolerances(tolfile, milestone).items():
        if k not in stock[0] or k not in ours[0]:
            continue
        mx, at, out = 0.0, None, None
        for r in stock:
            dt = r['_t'] - ia['_t']
            if dt < 0:
                continue  # before ignition the runs differ by their checklist timing only
            s = nearest(ours, ib['_t'] + dt)
            x, y = num(r, k), (num(s, k) if s else None)
            if x is None or y is None:
                continue
            d = abs(y - x)
            if k in ('lng', 'pitch', 'bank', 'yaw', 'cdux', 'cduy', 'cduz') and d > 180:
                d = 360 - d
            if d > mx:
                mx, at = d, dt
            if d > tol and out is None:
                out = dt
        fails += 1 if out is not None else 0
        print('| %s | %g | %.3f | %s | %s |' % (k, tol, mx, '%.0f' % at if at is not None else '-', '%.0f' % out if out is not None else 'none'))
    for name, rows, i in (('stock', stock, ia), ('ours', ours, ib)):
        mm = changes(rows, lambda r: int(r['modreg'], 8))
        al = [(t, f) for t, f in changes(rows, lambda r: ','.join(r['failreg%d' % j] for j in range(3))) if f != '0,0,0']
        print('\n%s: major modes %s' % (name, ' '.join('P%02d@TIG%+.0f' % (m, t - i['_t']) for t, m in mm)))
        print('%s: alarms %s' % (name, ' '.join('%s@TIG%+.0f' % (f, t - i['_t']) for t, f in al) or 'none'))
    print()
    return fails


def main():
    args = sys.argv[1:]
    tolfile = os.path.join('sim', 'nassp', 'edits.tsv')
    if '--tolerances' in args:
        i = args.index('--tolerances')
        tolfile = args[i + 1]
        del args[i:i + 2]
    pairs = []
    if args and args[0] == '--runs':
        d = args[1]
        for m in EVENTS:
            a = os.path.join(d, 'ref_%s_H' % m, 'apollo-c-%s-edits.csv' % m)
            b = os.path.join(d, 'agc_%s_H' % m, 'apollo-c-%s-edits.csv' % m)
            if os.path.isfile(a) and os.path.isfile(b):
                pairs.append((a, b, m))
        if not pairs:
            print('nassp-mission: no milestone runs in %s' % d)
            return 0
    elif len(args) == 3 and args[2] in EVENTS:
        pairs.append((args[0], args[1], args[2]))
    else:
        print(__doc__)
        return 2
    fails = sum(compare(load(a), load(b), m, tolfile) for a, b, m in pairs)
    print('nassp-mission: %d milestone(s), %d check(s) out of tolerance' % (len(pairs), fails))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
