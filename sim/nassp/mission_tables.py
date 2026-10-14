#!/usr/bin/env python3
"""mission_tables.py -- `make nassp-mission`: the NASSP milestone tables, generated from the edit CSVs.

A NASSP mission run (docs/modules/nassp-integration.md, "How to run it") writes one edit CSV per
vehicle and run: every second the GET, the LGC/CMC words (MODREG, FAILREG, channels 11 12 14 30 33,
TIME, DSPTAB) and the vessel's truth from Orbiter (altitude, speeds, position, mass, engine level,
attitude, ...). For each milestone this script pairs the stock run's CSV with ours and prints

  1. the milestone table: the events found the same way in both files (ignition, throttle-up,
     program changes, displays at marks, cut-off, contact, orbit),
  2. per quantity of sim/nassp/mission_tolerances.tsv: the tolerance, max |ours - stock| and where, and the
     first time out of tolerance (rows paired by time since ignition, nearest within 0.6 s, from
     ignition on),
  3. the major-mode changes and alarms of each run.

Usage: mission_tables.py STOCK.csv OURS.csv MILESTONE [--tolerances sim/nassp/mission_tolerances.tsv]
       mission_tables.py --runs DIR            (every milestone found in DIR: ref_<m>_H/ and
                                             agc_<m>_H/ holding apollo-c-<m>-edits.csv)
       mission_tables.py --derive DIR MILESTONE (print the milestone's tolerance rows from the two
                                             STOCK runs ref_<m>_H (-f 0.02) and ref_<m>_H01
                                             (-f 0.01) only: every numeric column, exactly 1.5 x
                                             their max difference, no rounding; for mission_tolerances.tsv,
                                             written before the dev run is compared)
Exit status 1 when a quantity is out of tolerance or a milestone event is missing in ours.
Standard library only; run with python3 -I.
"""
import bisect
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


_TIMES = {}


def nearest(rows, t):
    """The row of rows (in time order) nearest to sim time t, if within 0.6 s."""
    ts = _TIMES.get(id(rows))
    if ts is None or len(ts) != len(rows):
        ts = _TIMES[id(rows)] = [r['_t'] for r in rows]
    i = bisect.bisect_left(ts, t)
    cand = [rows[j] for j in (i - 1, i) if 0 <= j < len(rows)]
    if not cand:
        return None
    best = min(cand, key=lambda r: abs(r['_t'] - t))
    return best if abs(best['_t'] - t) <= 0.6 else None


def engine(rows):
    """The engine column of the milestone (dps for the descent, eng for the ascent)."""
    return 'dps' if 'dps' in rows[0] else 'eng'


def ignition(rows):
    """The row the milestone's times count from: ignition, entry interface (400,000 ft), or the
    scenario load (rendezvous: both builds start from the same post-insertion state)."""
    if 'markctr' in rows[0]:
        return rows[0]
    if 'vair' in rows[0]:
        return first(rows, lambda r: (num(r, 'alt') or 1e9) < 121920.0)
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


def ev_entry(rows):
    ev = {}
    ei = ignition(rows)
    if not ei:
        return ev
    t0 = ei['_t']
    ev['entry interface (400,000 ft), GET s'] = '%.1f' % float(ei['get'])
    for mm in (63, 64, 65, 66, 67):
        r = first(rows, lambda r, mm=mm: int(r['modreg'], 8) == mm and r['_t'] >= t0 - 600)
        if r:
            ev['P%d, EI+ s' % mm] = '%.1f' % (r['_t'] - t0)
    g = max((r for r in rows if r['_t'] >= t0), key=lambda r: num(r, 'g') or 0)
    ev['max deceleration, g at EI+ s'] = '%.2f at %.1f' % (num(g, 'g'), g['_t'] - t0)
    n0 = num(ei, 'nobj')
    d = first(rows, lambda r: r['_t'] > t0 and (num(r, 'alt') or 1e9) < 12192.0 and num(r, 'nobj') != n0)
    if d:
        ev['apex cover / drogues, EI+ s'] = '%.1f (alt %.0f m)' % (d['_t'] - t0, num(d, 'alt'))
        ev['DSKY at the drogues'] = display(d)
    w = first(rows, lambda r: r['_t'] > t0 and (num(r, 'alt') or 1e9) < 3.0)
    if w:
        ev['splashdown, EI+ s'] = '%.1f' % (w['_t'] - t0)
        ev['splashdown lng / lat, deg'] = '%.4f / %.4f' % (num(w, 'lng'), num(w, 'lat'))
    return ev


def regs(r):
    """R1 R2 R3 of a row's DSKY, as shown."""
    d = display(r)
    return '%s %s %s' % (d[d.index('R1 ') + 3:][:6], d[d.index('R2 ') + 3:][:6], d[d.index('R3 ') + 3:][:6])


def noun(r):
    d = display(r)
    return d[d.index('NOUN ') + 5:][:2]


def verb(r):
    d = display(r)
    return d[d.index('VERB ') + 5:][:2]


def ev_rendezvous(rows):
    """Rendezvous from the post-insertion start (times in sim s from the scenario load, the same GET
    on both builds): P20 and the RR lock-on, then per burn (CSI in P32, CDH in P33, TPI in P34) the
    targeting displays the crew last saw before it (N75 / N81 / N82, N37 / N58), the burn as the
    script nulled it (start, duration, RCS used, residual N85), and the relative state at the end."""
    ev = {}
    t = lambda r: '%.0f' % r['_t']
    r = first(rows, lambda r: int(r['modreg'], 8) == 20)
    if r:
        ev['P20 selected, s'] = t(r)
    r = first(rows, lambda r: noun(r) == '72' and verb(r) == '50')
    if r:
        ev['RR lock-on (F50N72), s'] = '%s (range %.1f km)' % (t(r), num(r, 'range') / 1000)
    for prog, b, nouns in ((32, 'csi', ('11', '75', '81', '82')), (33, 'cdh', ('13', '75', '81')), (34, 'tpi', ('37', '58', '59', '81'))):
        p = first(rows, lambda r, prog=prog: int(r['modreg'], 8) == prog)
        if not p:
            continue
        ev['P%d, s (marks)' % prog] = '%s (%s)' % (t(p), p.get('markctr', '?'))
        s = first(rows, lambda r, b=b: r.get('phase', '').startswith(b + ' burn'))
        end = s['_t'] if s else rows[-1]['_t']
        for n in nouns:
            last = None
            for r in rows:
                if p['_t'] <= r['_t'] < end and noun(r) == n and verb(r) in ('06', '16'):
                    last = r
            if last:
                ev['%s N%s (last before the burn)' % (b.upper(), n)] = regs(last)
        if not s:
            continue
        burn = [r for r in rows if r.get('phase', '').startswith(b + ' ')]
        e = burn[-1]
        ev['%s burn start s, nulling s' % b.upper()] = '%s, %.0f' % (t(s), e['_t'] - s['_t'])
        ev['%s RCS used, kg' % b.upper()] = '%.1f' % (num(s, 'mass') - num(e, 'mass'))
        clean = [r for r in burn if noun(r) == '85' and '?' not in regs(r)]
        ev['%s N85 after nulling' % b.upper()] = regs(clean[-1]) if clean else '(%s)' % display(e)
        o = nearest(rows, e['_t'] + 60) or e
        ev['%s orbit peri x apo, km (+60 s)' % b.upper()] = '%.2f x %.2f' % (num(o, 'peri') / 1000, num(o, 'apo') / 1000)
    s = first(rows, lambda r: r.get('phase', '').startswith('tpi burn'))
    e = nearest(rows, s['_t'] + 600) if s else None
    if e:
        ev['TPI burn + 600 s: range km, range rate m/s'] = '%.2f, %.2f' % (num(e, 'range') / 1000, num(e, 'rrate'))
    return ev


EVENTS = {'pdi': ev_pdi, 'liftoff': ev_liftoff, 'entry': ev_entry, 'rendezvous': ev_rendezvous}


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


ANGLES = ('lng', 'pitch', 'bank', 'yaw', 'cdux', 'cduy', 'cduz', 'cdut', 'cdus')  # wrap at 360 deg
META = ('get', 't', 'modreg', 'time1', 'time2', 'radmodes')        # clocks and mode words


def numeric_columns(rows):
    """The truth / DAP columns compared by number: every column whose values all parse as one
    number (channel, alarm and DSKY words, clocks and multi-word erasable dumps excluded)."""
    cols = []
    for k in rows[0]:
        if k.startswith('_') or k in META or k.startswith(('ch', 'failreg', 'dsptab')):
            continue
        vals = [r[k] for r in rows if r.get(k) not in ('', None)]
        try:
            [float(v) for v in vals]
        except ValueError:
            continue
        if vals:
            cols.append(k)
    return cols


def diff(a, b, k, tol=None):
    """max |b - a| of column k, rows paired by time since ignition (a's rows from ignition on,
    b's nearest within 0.6 s); returns (max, at s, first s above tol or None)."""
    ia, ib = ignition(a), ignition(b)
    mx, at, out = 0.0, None, None
    for r in a:
        dt = r['_t'] - ia['_t']
        if dt < 0:
            continue  # before ignition the runs differ by their checklist timing only
        s = nearest(b, ib['_t'] + dt)
        x, y = num(r, k), (num(s, k) if s else None)
        if x is None or y is None:
            continue
        d = abs(y - x)
        if k in ANGLES and d > 180:
            d = 360 - d
        if d > mx:
            mx, at = d, dt
        if tol is not None and d > tol and out is None:
            out = dt
    return mx, at, out


def derive(d, milestone):
    """mission_tolerances.tsv rows of a milestone from its two stock runs (the rule of the file's header)."""
    a = load(os.path.join(d, 'ref_%s_H' % milestone, 'apollo-c-%s-edits.csv' % milestone))
    b = load(os.path.join(d, 'ref_%s_H01' % milestone, 'apollo-c-%s-edits.csv' % milestone))
    for k in numeric_columns(a):
        if k in b[0]:
            mx = diff(a, b, k)[0]
            print('%s\t%s\t%r\tstock spread %r (1.5 x)' % (milestone, k, 1.5 * mx, mx))
    return 0


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
    tols = tolerances(tolfile, milestone)
    for k in numeric_columns(stock):
        if k in ours[0] and k not in tols:
            print('| %s | none (no tolerance row: not judged) | %.3f | - | - |' % (k, diff(stock, ours, k)[0]))
    for k, tol in tols.items():
        if k not in stock[0] or k not in ours[0]:
            continue
        mx, at, out = diff(stock, ours, k, tol)
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
    tolfile = os.path.join('sim', 'nassp', 'mission_tolerances.tsv')
    if '--tolerances' in args:
        i = args.index('--tolerances')
        tolfile = args[i + 1]
        del args[i:i + 2]
    pairs = []
    if len(args) == 3 and args[0] == '--derive' and args[2] in EVENTS:
        return derive(args[1], args[2])
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
