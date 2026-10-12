#!/usr/bin/env python3
"""lm_sim.py -- Virtual AGC's LM Simulator flown on build/agc_sim and on yaAGC.

Runs Stephan Hotto's LM Simulator (virtualagc/Contributed/LM_Simulator,
unmodified, through tools/lm_sim_harness.tcl) against an AGC on the LM
peripheral ports, plays a sequence of crew actions on the simulator's own
controls (DSKY Lite keys, crew switches, RHC), and records checkpoints:
the DSKY (decoded from a second peripheral connection, as yaDSKY2 does),
the simulator's model state (gimbal angles, body rates, FDAI needle
errors, PIPA counts, RCS propellant) and the RCS jet firings the AGC
commanded (channels 5 / 6). With --agc both the same sequence is flown on
build/agc_sim and on tools/oracle/bin/yaAGC and the checkpoints are
compared; that comparison is the test (exit 1 on a mismatch).

Usage:
  lm_sim.py --lmsim DIR [--agc ours|yaagc|both] [--sequence FILE]
            [--gui] [--out DIR] [--port N] [--sim PATH] [--yaagc PATH]
            [--repeat-jets]
DIR is virtualagc/Contributed/LM_Simulator. Without --gui the simulator
runs under tclsh (no windows); with --gui under wish, and `shot` saves
its windows (needs python3 PIL and libXtst).

Sequence language (one command per line, `#` comments, times in s):
  keys V37E00E        DSKY Lite buttons: digits V N E C(LR) R(SET)
                      K(EY REL) + -
  pro                 the DSKY Lite PRO button
  crew NAME           crew switch on the simulator's AGC Crew Inputs window:
                      ISS (ISS turn-on requested), CAGE, ATTHOLD, AUTO,
                      DAPOFF, AGCCTL, ARMED
  rhc AXIS N          RHC deflection (ROLL PITCH YAW, -57..57), as the
                      simulator's scale does
  rhc DETENT          RHC back to detent
  wait S              let S seconds pass
  until FIELD VALUE [within S]   wait until the DSKY shows it (default 30 s)
  expect FIELD VALUE  check the DSKY now (FIELD: PROG VERB NOUN R1 R2 R3,
                      FLASH on|off, LAMP name on|off)
  snap NAME [TOL...]  checkpoint; compared across AGCs with tolerances
                      deg=D (gimbal, default 0.5) rate=R (deg/s, default
                      0.2) needle=N (default 30 counts) jets=F (relative,
                      default 0.5) reg=N (R1-R3 numeric, default 0) or
                      reg=skip, display=skip
  jets reset          zero the jet statistics
  shot NAME           (--gui) save the simulator windows as NAME_*.png
  tcl SCRIPT          evaluate SCRIPT in the simulator
  say TEXT            print TEXT
"""
import argparse
import json
import os
import socket
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_client  # noqa: E402  (same directory; python3 -I drops it from the path)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
ROPE = os.path.join(ROOT, 'tools', 'oracle', 'ropes', 'Luminary099.bin')

KEY_BUTTON = {'+': 'b1', '7': 'b2', '8': 'b3', '9': 'b4', 'C': 'b5',
              '-': 'b6', '4': 'b7', '5': 'b8', '6': 'b9', '0': 'b11',
              '1': 'b12', '2': 'b13', '3': 'b14', 'K': 'b15', 'V': 'b16',
              'N': 'b17', 'E': 'b18', 'R': 'b19'}
CREW_BUTTON = {'ISS': 'cb21b', 'CAGE': 'cb4b', 'ATTHOLD': 'cb6b',
               'AUTO': 'cb7b', 'DAPOFF': 'cb61b', 'AGCCTL': 'cb51c',
               'ARMED': 'cb1a'}
SIM_VARS = ('IMUX_ANGLE', 'IMUY_ANGLE', 'IMUZ_ANGLE', 'Omega_Roll',
            'Omega_Pitch', 'Omega_Yaw', 'error_x', 'error_y', 'error_z',
            'PIPAX_COUNT', 'PIPAY_COUNT', 'PIPAZ_COUNT',
            'RCS_Propellant_Mass_KG', 'gimbalLock')
# channel 5 bits 1-8 and channel 6 bits 1-8, the simulator's names
JET_NAMES = {(5, 1): 'Q4U', (5, 2): 'Q4D', (5, 3): 'Q3U', (5, 4): 'Q3D',
             (5, 5): 'Q2U', (5, 6): 'Q2D', (5, 7): 'Q1U', (5, 8): 'Q1D',
             (6, 1): 'Q3A', (6, 2): 'Q4F', (6, 3): 'Q1F', (6, 4): 'Q2A',
             (6, 5): 'Q2L', (6, 6): 'Q3R', (6, 7): 'Q4R', (6, 8): 'Q1L'}
WINDOWS = {'DSKY Lite': 'dsky', 'FDAI / IMU': 'fdai', 'AGC Outputs': 'outputs',
           'AGC Crew Inputs': 'crew', 'Simulation Monitor': 'monitor'}


def port_free(p):
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(('', p))
        return True
    except OSError:
        return False
    finally:
        s.close()


def free_base(want):
    """A base port with the ten ports base..base+9 free (LM: 19797)."""
    bases = [want] * 30 if want else [19797 + 2000 * i for i in range(10)]
    for base in bases:
        if all(port_free(base + i) for i in list(range(10)) + [150, 153]):
            return base
        if want:
            time.sleep(1.0)     # the last run's sockets may still linger
    sys.exit('lm_sim: no free port range')


def connect(port, timeout=10.0):
    end = time.time() + timeout
    while True:
        try:
            return socket.create_connection(('127.0.0.1', port))
        except OSError:
            if time.time() > end:
                raise
            time.sleep(0.1)


class Monitor(threading.Thread):
    """A second peripheral: DSKY decode + output channel log (jets)."""

    def __init__(self, port):
        super().__init__(daemon=True)
        self.sock = connect(port)
        self.lock = threading.Lock()
        self.dsky = sim_client.Dsky()
        self.stop = False
        self.jet_reset()

    def jet_reset(self):
        with self.lock:
            self.jet_on = {}            # (ch, bit) -> time switched on
            self.jet_count = {k: 0 for k in JET_NAMES}
            self.jet_time = {k: 0.0 for k in JET_NAMES}
            self.chan_last = {5: 0, 6: 0}
            self.packets = {5: 0, 6: 0}

    def jets(self):
        now = time.time()
        with self.lock:
            out = {}
            for k, name in JET_NAMES.items():
                t = self.jet_time[k] + (now - self.jet_on[k]
                                        if k in self.jet_on else 0.0)
                out[name] = (self.jet_count[k], round(t, 3))
            out['packets'] = (self.packets[5], self.packets[6])
            return out

    def take_jets(self, chan, value):
        now = time.time()
        old = self.chan_last[chan]
        for bit in range(1, 9):
            m = 1 << (bit - 1)
            k = (chan, bit)
            if value & m and not old & m:
                self.jet_on[k] = now
                self.jet_count[k] += 1
            elif old & m and not value & m and k in self.jet_on:
                self.jet_time[k] += now - self.jet_on.pop(k)
        self.chan_last[chan] = value

    def run(self):
        buf = b''
        while not self.stop:
            try:
                data = self.sock.recv(4096)
            except OSError:
                return
            if not data:
                return
            buf += data
            while len(buf) >= 4:
                p = sim_client.parse_packet(buf[:4])
                if p is None:
                    buf = buf[1:]
                    continue
                buf = buf[4:]
                chan, value, ubit = p
                if ubit:
                    continue
                with self.lock:
                    self.dsky.take(chan, value)
                    if chan in (5, 6):
                        self.packets[chan] += 1
                        self.take_jets(chan, value)

    def display(self):
        with self.lock:
            d = self.dsky
            return {
                'PROG': d.field('PROG'),
                'VERB': d.field('VERB'),
                'NOUN': d.field('NOUN'),
                'R1': d.field('R1'),
                'R2': d.field('R2'),
                'R3': d.field('R3'),
                'FLASH': d.lamp('FLASH'),
                'lamps': sorted(
                    k for k in sim_client.LAMPS if k not in (
                        'FLASH',
                        'COMPACTY') and d.lamp(k)),
                'text': d.text()}


class Relay(threading.Thread):
    """--repeat-jets: a protocol relay between the simulator and the AGC
    that re-sends the last channel 5 and 6 words every 50 ms, the cadence
    at which yaAGC broadcasts them (one packet per WRITE of the channel,
    CpuWriteIO), for an AGC that broadcasts on change only.  Diagnostic:
    it separates the protocol difference from the flight code."""

    def __init__(self, listen_port, agc_port):
        super().__init__(daemon=True)
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(('127.0.0.1', listen_port))
        self.srv.listen(1)
        self.agc_port = agc_port
        self.lock = threading.Lock()
        self.last = {}
        self.stop = False

    def pump_up(self, sim, agc):
        while not self.stop:
            data = sim.recv(4096)
            if not data:
                return
            agc.sendall(data)

    def run(self):
        sim, _ = self.srv.accept()
        agc = connect(self.agc_port)
        threading.Thread(
            target=self.pump_up, args=(
                sim, agc), daemon=True).start()
        threading.Thread(target=self.repeat, args=(sim,), daemon=True).start()
        buf = b''
        while not self.stop:
            data = agc.recv(4096)
            if not data:
                return
            buf += data
            out = b''
            while len(buf) >= 4:
                p = sim_client.parse_packet(buf[:4])
                if p is None:
                    out += buf[:1]
                    buf = buf[1:]
                    continue
                if p[0] in (5, 6) and not p[2]:
                    self.last[p[0]] = buf[:4]
                out += buf[:4]
                buf = buf[4:]
            with self.lock:
                sim.sendall(out)

    def repeat(self, sim):
        while not self.stop:
            time.sleep(0.05)
            with self.lock:
                for ch in (5, 6):
                    if ch in self.last:
                        sim.sendall(self.last[ch])


class Harness:
    """The LM Simulator under tools/lm_sim_harness.tcl."""

    def __init__(self, lmsim, ctrl_port, agc_port, gui, log):
        prog = 'wish' if gui else 'tclsh'
        cmd = [
            prog,
            os.path.join(
                HERE,
                'lm_sim_harness.tcl'),
            lmsim,
            str(ctrl_port),
            '--port=%d' %
            agc_port,
            '--cfg=lm_simulator.ini']
        self.proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL)
        self.sock = connect(ctrl_port)
        self.f = self.sock.makefile('rw')

    def tcl(self, script):
        self.f.write(script.replace('\n', ' ') + '\n')
        self.f.flush()
        r = self.f.readline().rstrip('\n')
        if not r.startswith('OK'):
            raise RuntimeError('simulator: %s -> %s' % (script, r))
        return r[3:]

    def state(self):
        return {v: float(self.tcl('set ::' + v)) for v in SIM_VARS}

    def close(self):
        self.proc.terminate()
        try:
            self.proc.wait(5)
        except subprocess.TimeoutExpired:
            self.proc.kill()


class Flight:
    """One AGC + the simulator + the monitor, playing a sequence."""

    def __init__(self, args, agc, outdir):
        self.args, self.agc, self.out = args, agc, outdir
        os.makedirs(outdir, exist_ok=True)
        self.base = free_base(args.port)
        self.log = open(os.path.join(outdir, 'run.log'), 'w')
        rope = ROPE
        if agc == 'ours':
            cmd = [args.sim, '--core', rope,
                   '--port', str(self.base), '--quiet']
        else:
            cmd = [args.yaagc, '--core=' + rope, '--port=%d' % self.base,
                   '--no-resume', '--quiet']
        self.agc_proc = subprocess.Popen(cmd, cwd=outdir, stdout=self.log,
                                         stderr=subprocess.STDOUT,
                                         stdin=subprocess.DEVNULL)
        self.t0 = time.time()
        self.mon = Monitor(self.base + 1)
        self.mon.start()
        sim_port = self.base + 4
        if args.repeat_jets and agc == 'ours':
            self.relay = Relay(self.base + 150, self.base + 4)
            self.relay.start()
            sim_port = self.base + 150
        self.sim = Harness(args.lmsim, self.base + 153, sim_port,
                           args.gui, self.log)
        self.snaps = []
        self.failures = []

    def say(self, msg):
        line = '[%s %6.1f] %s' % (self.agc, time.time() - self.t0, msg)
        print(line, flush=True)
        self.log.write(line + '\n')
        self.log.flush()

    def keys(self, text):
        for ch in text.upper().replace(' ', ''):
            self.sim.tcl('.dsky.%s invoke' % KEY_BUTTON[ch])
            time.sleep(0.4)

    def ok(self, field, want):
        d = self.mon.display()
        f = field.upper()
        if f == 'FLASH':
            return d['FLASH'] == (want.lower() == 'on')
        if f == 'LAMP':
            name, state = want.split()
            return (name.upper() in d['lamps']) == (state.lower() == 'on')
        got = d[f].strip() or 'blank'
        return got == want or got.replace(' ', '') == want

    def until(self, field, want, within):
        end = time.time() + within
        while time.time() < end:
            if self.ok(field, want):
                self.say(
                    'until %s %s: ok (%s)' %
                    (field, want, self.mon.display()['text']))
                return
            time.sleep(0.1)
        msg = 'until %s %s: not within %.0f s (%s)' % (
            field, want, within, self.mon.display()['text'])
        self.failures.append(msg)
        self.say('FAIL ' + msg)

    def snap(self, name, tol):
        s = {'name': name, 't': round(time.time() - self.t0, 1), 'tol': tol,
             'display': self.mon.display(), 'sim': self.sim.state(),
             'jets': self.mon.jets()}
        self.snaps.append(s)
        st = s['sim']
        fired = {k: v for k, v in s['jets'].items() if v[0] and k != 'packets'}
        fired['ch5/6 packets'] = s['jets']['packets']
        self.say(
            'snap %s: %s | gimbals %.2f %.2f %.2f | rates %.3f %.3f %.3f'
            ' | needles %d %d %d | jets %s' %
            (name,
             s['display']['text'],
                st['IMUX_ANGLE'],
                st['IMUY_ANGLE'],
                st['IMUZ_ANGLE'],
                st['Omega_Roll'],
                st['Omega_Pitch'],
                st['Omega_Yaw'],
                st['error_x'],
                st['error_y'],
                st['error_z'],
                fired or '-'))

    def shot(self, name):
        if not self.args.gui:
            return
        import xshot  # noqa: E402  (found through --xshot)
        for w, slug in WINDOWS.items():
            xshot.shot(w, os.path.join(self.out, '%s_%s.png' % (name, slug)))

    def command(self, line):
        words = line.split()
        op, rest = words[0].lower(), words[1:]
        if op == 'keys':
            self.keys(''.join(rest))
        elif op == 'pro':
            self.sim.tcl('.dsky.b10 invoke')
            time.sleep(0.6)
        elif op == 'crew':
            self.sim.tcl('.crewinp.%s invoke' % CREW_BUTTON[rest[0].upper()])
            time.sleep(0.2)
        elif op == 'rhc':
            if rest[0].upper() == 'DETENT':
                self.sim.tcl('.crewinp.neutral invoke')
            else:
                axis, n = rest[0].upper(), int(rest[1])
                self.sim.tcl(
                    'set ::RHC%s %d; write_rhc %s %d' %
                    (axis.capitalize(), n, axis, n))
        elif op == 'wait':
            time.sleep(float(rest[0]))
        elif op == 'until':
            within = 30.0
            if 'within' in rest:
                i = rest.index('within')
                within = float(rest[i + 1])
                rest = rest[:i]
            self.until(rest[0], ' '.join(rest[1:]), within)
        elif op == 'expect':
            if not self.ok(rest[0], ' '.join(rest[1:])):
                msg = 'expect %s: got %s' % (
                    ' '.join(rest), self.mon.display()['text'])
                self.failures.append(msg)
                self.say('FAIL ' + msg)
        elif op == 'snap':
            tol = dict(w.split('=') for w in rest[1:])
            self.snap(rest[0], tol)
        elif op == 'jets' and rest and rest[0] == 'reset':
            self.mon.jet_reset()
        elif op == 'shot':
            self.shot(rest[0])
        elif op == 'tcl':
            self.say(
                'tcl %s -> %s' %
                (' '.join(rest),
                 self.sim.tcl(
                    ' '.join(rest))))
        elif op == 'say':
            self.say(' '.join(rest))
        else:
            raise ValueError('unknown command: ' + line)

    def play(self, lines):
        try:
            for raw in lines:
                line = raw.split('#', 1)[0].strip()
                if line:
                    self.command(line)
        finally:
            self.close()
        with open(os.path.join(self.out, 'snaps.json'), 'w') as f:
            json.dump({'agc': self.agc, 'snaps': self.snaps,
                       'failures': self.failures}, f, indent=1)
        return self.snaps

    def close(self):
        self.mon.stop = True
        self.sim.close()
        self.agc_proc.terminate()
        try:
            self.agc_proc.wait(5)
        except subprocess.TimeoutExpired:
            self.agc_proc.kill()
        self.log.close()


def number(text):
    t = text.replace(' ', '')
    try:
        return int(t)
    except ValueError:
        return None


def compare(a, b):
    """Checkpoint by checkpoint, ours (a) against yaAGC (b): mismatches."""
    bad = []
    print(
        '\n%-14s %-8s %s' %
        ('checkpoint',
         'agc',
         'DSKY | gimbals | rates | needles | jet firings (count, s)'))
    for sa, sb in zip(a, b):
        tol = sa['tol']
        for who, s in (('agc_sim', sa), ('yaAGC', sb)):
            st = s['sim']
            fired = ' '.join(
                '%s %d/%.2f' %
                (k, v[0], v[1]) for k, v in s['jets'].items()
                if v[0] and k != 'packets')
            print(
                '%-14s %-8s %s | %.2f %.2f %.2f | %.2f %.2f %.2f'
                ' | %d %d %d | %s' %
                (s['name'],
                 who,
                 s['display']['text'],
                    st['IMUX_ANGLE'],
                    st['IMUY_ANGLE'],
                    st['IMUZ_ANGLE'],
                    st['Omega_Roll'],
                    st['Omega_Pitch'],
                    st['Omega_Yaw'],
                    st['error_x'],
                    st['error_y'],
                    st['error_z'],
                    fired or '-'))
        da, db = sa['display'], sb['display']
        if tol.get('display') != 'skip':
            for f in ('PROG', 'VERB', 'NOUN', 'FLASH', 'lamps'):
                if da[f] != db[f]:
                    bad.append(
                        '%s: %s %s vs %s' %
                        (sa['name'], f, da[f], db[f]))
            rt = tol.get('reg', '0')
            for f in ('R1', 'R2', 'R3'):
                if rt == 'skip':
                    continue
                na, nb = number(da[f]), number(db[f])
                if (na is None or nb is None) and da[f] != db[f]:
                    bad.append(
                        '%s: %s %r vs %r' %
                        (sa['name'], f, da[f], db[f]))
                elif (na is not None and nb is not None
                      and abs(na - nb) > int(rt)):
                    bad.append(
                        '%s: %s %s vs %s' %
                        (sa['name'], f, da[f], db[f]))
        deg = float(tol.get('deg', 0.5))
        for v in ('IMUX_ANGLE', 'IMUY_ANGLE', 'IMUZ_ANGLE'):
            d = abs(sa['sim'][v] - sb['sim'][v]) % 360.0
            if min(d, 360.0 - d) > deg:
                bad.append('%s: %s %.3f vs %.3f' %
                           (sa['name'], v, sa['sim'][v], sb['sim'][v]))
        rate = float(tol.get('rate', 0.2))
        for v in ('Omega_Roll', 'Omega_Pitch', 'Omega_Yaw'):
            if abs(sa['sim'][v] - sb['sim'][v]) > rate:
                bad.append('%s: %s %.3f vs %.3f' %
                           (sa['name'], v, sa['sim'][v], sb['sim'][v]))
        needle = float(tol.get('needle', 30))
        for v in ('error_x', 'error_y', 'error_z'):
            if abs(sa['sim'][v] - sb['sim'][v]) > needle:
                bad.append('%s: %s %d vs %d' %
                           (sa['name'], v, sa['sim'][v], sb['sim'][v]))
        if tol.get('jets') != 'skip':
            jf = float(tol.get('jets', 0.5))
            ta = sum(v[1] for k, v in sa['jets'].items() if k != 'packets')
            tb = sum(v[1] for k, v in sb['jets'].items() if k != 'packets')
            if abs(ta - tb) > jf * max(ta, tb, 0.2):
                bad.append(
                    '%s: total jet on-time %.2f s vs %.2f s' %
                    (sa['name'], ta, tb))
            used_a = {
                k for k,
                v in sa['jets'].items() if v[0] and k != 'packets'}
            used_b = {
                k for k,
                v in sb['jets'].items() if v[0] and k != 'packets'}
            if (ta > 0.2 or tb > 0.2) and not used_a & used_b:
                bad.append('%s: jets %s vs %s' %
                           (sa['name'], sorted(used_a), sorted(used_b)))
    if len(a) != len(b):
        bad.append('checkpoint count %d vs %d' % (len(a), len(b)))
    return bad


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--lmsim', required=True)
    ap.add_argument('--agc', choices=('ours', 'yaagc', 'both'), default='both')
    ap.add_argument(
        '--sequence',
        default=os.path.join(
            ROOT,
            'sim',
            'lm_simulator',
            'tutorial.txt'))
    ap.add_argument('--gui', action='store_true')
    ap.add_argument('--out', default=os.path.join(ROOT, 'build', 'lm_sim'))
    ap.add_argument('--port', type=int, default=0)
    ap.add_argument(
        '--repeat-jets',
        action='store_true',
        help='relay re-sending ch 5/6 every 50 ms to the simulator '
             '(agc_sim only; diagnostic)')
    ap.add_argument(
        '--xshot',
        default=HERE,
        help='directory of xshot.py (--gui screenshots)')
    ap.add_argument('--sim', default=os.path.join(ROOT, 'build', 'agc_sim'))
    ap.add_argument(
        '--yaagc',
        default=os.path.join(
            ROOT,
            'tools',
            'oracle',
            'bin',
            'yaAGC'))
    args = ap.parse_args(argv)
    for k in ('lmsim', 'sim', 'yaagc', 'out', 'sequence'):
        setattr(args, k, os.path.abspath(getattr(args, k)))
    if args.gui:
        sys.path.insert(0, os.path.abspath(args.xshot))
    with open(args.sequence) as f:
        lines = f.readlines()
    runs = {}
    fails = []
    for agc in (('ours', 'yaagc') if args.agc == 'both' else (args.agc,)):
        fl = Flight(args, agc, os.path.join(args.out, agc))
        runs[agc] = fl.play(lines)
        fails += ['%s: %s' % (agc, m) for m in fl.failures]
    if args.agc == 'both':
        fails += compare(runs['ours'], runs['yaagc'])
    print(
        '\n%d checkpoint(s); %s' %
        (len(
            next(
                iter(
                    runs.values()))),
         'all matched' if not fails else '%d mismatch(es):' %
         len(fails)))
    for m in fails:
        print('  ' + m)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
