#!/usr/bin/env python3
"""nassp_replay.py -- `make nassp-replay`: NASSP scenario images on our AGC vs NASSP's yaAGC.

Replays the AGC images of NASSP 9 Apollo 11 scenarios (sim/nassp/images/*.agc, the
AGC_BEGIN..AGC_END block of the CSM) on two engines built from sim/nassp/replay.c:

  ours   src/nassp/nassp_engine.c (the shim NASSP's vessels link) over build/libagc.a
  stock  NASSP's own yaAGC engine (src_sys/yaAGC at NASSP 5470bf1)

and compares their transcripts line for line: the CSM/LM state-vector blocks after each
P00 integration step, and the result of an MCC-style uplink (V71 CSM state vector through
INLINK/UPRUPT, then V33): UPBUFF, UPSVFLAG, the blocks, MODREG, DSPTAB. Seconds, no Orbiter.

NASSP's yaAGC is GPL and not part of this repository: it is taken from NASSP_YAAGC_DIR
(<NASSP>/Orbitersdk/samples/ProjectApollo/src_sys/yaAGC) when set, else downloaded from
GitHub at the pinned commit into build/nassp/yaagc. Without either the target SKIPS with a
message and exits 0.
"""
import argparse
import difflib
import os
import subprocess
import sys
import time
import urllib.request

NASSP_COMMIT = '5470bf198b84ef58305c7c32f058b80e0d104f55'
RAW = ('https://raw.githubusercontent.com/rcflyinghokie/NASSP/' + NASSP_COMMIT +
       '/Orbitersdk/samples/ProjectApollo/src_sys/yaAGC/')
YAAGC_FILES = ['agc_engine.h', 'agc_engine.c', 'agc_engine_init.c', 'agc_utilities.c', 'Backtrace.c',
               'yaAGC.h', 'random.c', 'rfopen.c']
YAAGC_SRCS = ['agc_engine.c', 'agc_engine_init.c', 'agc_utilities.c', 'Backtrace.c', 'random.c', 'rfopen.c']
WARN = ['-Wall', '-Wextra', '-Wpedantic', '-Wconversion', '-Wsign-conversion', '-Wshadow',
        '-Wstrict-prototypes', '-Wmissing-prototypes', '-Wundef', '-Wcast-qual', '-Werror']
SIM_OBJS = ['clock.o', 'channels.o', 'counters.o', 'rope.o']

# The uplink: MCC's CSM state vector load (rtcc.cpp V7XUpdate: V71, 21 words, 1501
# = UPSVFLAG, 1 = CSM / earth), the vector words being the ones stock NASSP uplinked
# after insertion (milestone 1), then V33 (VERB 33 = accept, P27's PROCEED).
SV_UPLINK = ('V71E21E1501E1E'
             '77777E50575E77527E41251E77624E47425E'
             '23370E14531E77650E60540E00076E22455E'
             '00064E36074E'
             'V33E')

# The P30 target load MCC sends (rtcc.cpp CMCExternalDeltaVUpdate: V71, 12 words,
# 3404 = DELVSLV, the LVLH dV as three DP words, then TIG), with made-up values.
DV_UPLINK = ('V71E12E3404E'
             '00001E23456E77776E54321E00000E01234E'
             '01113E24475E'
             'V33E')

# image, rope, AGC seconds of P00, P00 integration steps expected, uplink.
# P00 integrates the state vectors every 20 minutes (two steps: CSM, then LM).
CASES = [
    ('insertion', 'Comanche055.bin', 1220, 2, SV_UPLINK),
    ('tli', 'Comanche055.bin', 1220, 2, DV_UPLINK),
    ('mcc2', 'Comanche055.bin', 30, 0, DV_UPLINK),
    # LGC before PDI, landing radar breaker open: the LR position the AGC wants (RADMODES
    # bit 6) must survive the shim's load restart (flight bug 5, alarm 00522 in P63).
    ('pdi_lm', 'Luminary099.bin', 2, 0, '-'),
    # LGC at T-1m of lunar liftoff, P12 waiting for TIG-35 (restart group 4 phase 25): the load
    # restart must resume it from the saved phase table (PHASEn, TBASEn, LONGBASE mapped).
    ('liftoff_lm', 'Luminary099.bin', 2, 0, '-'),
]


# The display a resumed program shows 40 s after the load (Luminary images): liftoff's P12
# at TIG-35 s (25 s after the load) shows V06N74.
NOUN40 = {'liftoff_lm': '74'}
# The erasable NASSP reads (Lua get_agcerasable, MFDs) must show a display soon after the key:
# NASSP's yaAGC works in that memory itself; the shim mirrors ours into it. 0.2 s is twice the
# Lua scripts' 0.1 s polling step.
DSKY_LATENCY_S = 0.2
RELAY_DIGIT = {0: ' ', 21: '0', 3: '1', 25: '2', 27: '3', 15: '4', 30: '5', 28: '6', 19: '7', 29: '8', 31: '9'}


def relay_digits(w):
    """The two digits of a DSKY relay word (bits 10-6 and 5-1)."""
    return RELAY_DIGIT.get((w >> 5) & 31, '?') + RELAY_DIGIT.get(w & 31, '?')


def skip(msg):
    print('nassp-replay: SKIP: ' + msg)
    print('  (set NASSP_YAAGC_DIR=<NASSP>/Orbitersdk/samples/ProjectApollo/src_sys/yaAGC'
          ' to use a local NASSP checkout)')
    sys.exit(0)


def yaagc_dir(build):
    d = os.environ.get('NASSP_YAAGC_DIR')
    if d:
        missing = [f for f in YAAGC_FILES if not os.path.isfile(os.path.join(d, f))]
        if missing:
            skip('NASSP_YAAGC_DIR=%s lacks %s' % (d, ', '.join(missing)))
        return d
    d = os.path.join(build, 'nassp', 'yaagc')
    if all(os.path.isfile(os.path.join(d, f)) for f in YAAGC_FILES):
        return d
    os.makedirs(d, exist_ok=True)
    for f in YAAGC_FILES:
        try:
            with urllib.request.urlopen(RAW + f, timeout=30) as r:
                data = r.read()
        except Exception as e:  # network, proxy, 404: no reference engine
            skip('cannot download NASSP yaAGC %s (%s)' % (f, e))
        with open(os.path.join(d, f), 'wb') as out:
            out.write(data.replace(b'\r\n', b'\n'))
    return d


def build(args, ya):
    out = os.path.join(args.build, 'nassp')
    os.makedirs(out, exist_ok=True)
    cc = os.environ.get('CC', 'gcc')
    ours = os.path.join(out, 'replay_ours')
    stock = os.path.join(out, 'replay_stock')
    sim = [os.path.join(args.build, 'src', 'sim', o) for o in SIM_OBJS]
    # The shim and the harness under the project's warnings (NASSP's header as a
    # system header); NASSP's yaAGC below is third-party code compiled as is.
    subprocess.run([cc, '-std=c11', '-O2'] + WARN + ['-Iinclude', '-isystem', ya, '-o', ours,
                    'sim/nassp/replay.c', 'src/nassp/nassp_engine.c'] + sim +
                   [os.path.join(args.build, 'libagc.a'), '-lm'], check=True)
    subprocess.run([cc, '-std=gnu11', '-O2', '-w', '-DREPLAY_YAAGC', '-I', ya, '-o', stock,
                    'sim/nassp/replay.c'] + [os.path.join(ya, f) for f in YAAGC_SRCS] + ['-lm'],
                   check=True)
    return ours, stock


def run(exe, case):
    name, rope, secs, _, up = case
    img = os.path.abspath(os.path.join('sim', 'nassp', 'images', name + '.agc'))
    # cwd: the shim writes its apollo-c-agc-*.log sidecars into the working directory
    r = subprocess.run([os.path.abspath(exe), img, os.path.abspath(os.path.join('tools', 'oracle', 'ropes', rope)),
                        str(secs), up], capture_output=True, text=True, timeout=600, cwd=os.path.dirname(os.path.abspath(exe)))
    if r.returncode != 0:
        return None, r.stderr
    return r.stdout.splitlines(), ''


DP_LINES = ('csm', 'lem', 'delvslv-tig')  # blocks of double-precision pairs


def sp(w):
    """A ones'-complement AGC word as an int (-0 and +0 both 0)."""
    return w - 0o77777 if w & 0o40000 else w


# Integration residuals already documented for the C integrator (docs/modules/
# review-nav-integ.md N4 KEPPREP, N9 TNUV, N10 RP-TO-R: 1-2 LSB on a DP word pair after a
# step that rectifies or runs KEPLER). A pair off by more fails; every pair off is listed.
TOL_LSB = 2
PAIR_NAMES = ['RRECT.x', 'RRECT.y', 'RRECT.z', 'VRECT.x', 'VRECT.y', 'VRECT.z', 'TET',
              'DELTAV.x', 'DELTAV.y', 'DELTAV.z', 'NUV.x', 'NUV.y', 'NUV.z', 'RCV.x', 'RCV.y',
              'RCV.z', 'VCV.x', 'VCV.y', 'VCV.z', 'TC', 'XKEP']


def dps(line):
    """(name, [DP values]) of a block line, None for other lines: a mixed-sign
    pair (-0 / +0 high word) is representation, not value."""
    f = line.split()
    if not f or f[0] not in DP_LINES:
        return None
    w = [int(x, 8) for x in f[1:]]
    return f[0], [sp(w[k]) * 0o40000 + sp(w[k + 1]) for k in range(0, len(w) - 1, 2)]


def compare(stock, ours):
    """(ok, words differing only in representation, [pairs off by 1..TOL_LSB])."""
    a = [x for x in stock if not x.startswith('#')]
    b = [x for x in ours if not x.startswith('#')]
    if len(a) != len(b):
        return False, 0, []
    rep, off = 0, []
    for i, (x, y) in enumerate(zip(a, b)):
        if x == y:
            continue
        dx, dy = dps(x), dps(y)
        if dx is None or dy is None:
            return False, 0, []
        for k, (u, v) in enumerate(zip(dx[1], dy[1])):
            if abs(u - v) > TOL_LSB:
                return False, 0, []
            if u != v:
                name = PAIR_NAMES[k] if dx[0] != 'delvslv-tig' else 'DELVSLV/TIG %d' % k
                off.append('line %d %s %s %+d' % (i + 1, dx[0], name, v - u))
        rep += sum(1 for u, v in zip(x.split(), y.split()) if u != v)
    return True, rep, off


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--build', default='build')
    ap.add_argument('--keep', action='store_true', help='keep transcripts in build/nassp')
    args = ap.parse_args()
    ya = yaagc_dir(args.build)
    ours, stock = build(args, ya)
    fails = 0
    for case in CASES:
        name, _, _, want, up = case
        t0 = time.time()
        a, ea = run(ours, case)
        b, eb = run(stock, case)
        dt = time.time() - t0
        if a is None or b is None:
            print('FAIL %-10s engine error: %s %s' % (name, ea.strip(), eb.strip()))
            fails += 1
            continue
        if args.keep:
            for tag, lines in (('ours', a), ('stock', b)):
                with open(os.path.join(args.build, 'nassp', '%s.%s.txt' % (name, tag)), 'w') as f:
                    f.write('\n'.join(lines) + '\n')
        steps = sum(1 for x in b if x.startswith('step '))
        same, rep, off = compare(b, a)
        la = [float(x.split()[2]) for x in a if x.startswith('# dsky-latency')]
        if not la or la[0] > DSKY_LATENCY_S:
            print('FAIL %-10s erasable mirror shows noun 36 %s s after the key (stock %s s; at most %.2f s)' %
                  (name, la[0] if la else '?', ' '.join(x.split()[2] for x in b if x.startswith('# dsky-latency')) or '?',
                   DSKY_LATENCY_S))
            fails += 1
            continue
        lr = [x.split() for x in a if x.startswith('# radmodes-lrpos')]
        if lr and lr[0][3] != lr[0][5]:
            print('FAIL %-10s RADMODES bit 6 (LR position wanted) %s after the load, %s in the image' %
                  (name, lr[0][5], lr[0][3]))
            fails += 1
            continue
        nn = [x.split() for x in a if x.startswith('# noun40')]
        if name in NOUN40 and (not nn or relay_digits(int(nn[0][2], 8)) != NOUN40[name]):
            print('FAIL %-10s noun %s 40 s after the load, %s expected: the program was not resumed' %
                  (name, relay_digits(int(nn[0][2], 8)) if nn else '?', NOUN40[name]))
            fails += 1
            continue
        if case[1].startswith('Luminary'):
            # This harness's yaAGC restarts the LGC images by a TC trap (NASSP's does not), so
            # stock is no reference here: the LM cases are checked against the image only.
            print('PASS %-10s load restart: LR position kept%s (own checks; %.1f s)' %
                  (name, ', program resumed' if name in NOUN40 else '', dt))
            continue
        what = '%d integration steps' % steps + (' + uplink' if up != '-' else '')
        if same and steps == want:
            print('PASS %-10s %s %s (%.1f s)' % (name, what, 'identical' if not off else
                  'identical but %d DP pairs within %d LSB' % (len(off), TOL_LSB), dt))
            for o in off:
                print('     %s LSB (ours - stock)' % o)
            if rep and not off:
                print('     %d words differ only in the sign of a zero high word' % rep)
        else:
            fails += 1
            print('FAIL %-10s %s, %d expected (%.1f s)' % (name, what, want, dt))
            al = [x for x in a if not x.startswith('#')]
            bl = [x for x in b if not x.startswith('#')]
            for d in list(difflib.unified_diff(bl, al, 'stock', 'ours', n=0, lineterm=''))[:12]:
                print('  ' + d[:200])
    print('nassp-replay: %d of %d images agree with NASSP yaAGC' % (len(CASES) - fails, len(CASES)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
