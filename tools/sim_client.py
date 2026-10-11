#!/usr/bin/env python3
"""sim_client.py -- scripted peripheral for build/agc_sim (or yaAGC).

Speaks yaAGC's peripheral socket protocol (virtualagc yaAGC/agc_utilities.c
FormIoPacket / ParseIoPacket): sends DSKY key packets (channel 015), the
PRO button (channel 032 bit 14 through a u-bit mask), input discretes and
counter increments, collects the output channels the AGC broadcasts, decodes
the DSKY relay words (channel 010) into PROG / VERB / NOUN / R1-R3 and the
lamps (010 relay row 12, channel 011, fictitious channel 0163), and checks a
scenario's expected displays.

Usage:
  sim_client.py --scenario FILE [--vehicle csm|lm] [--max-cs N] [--sim PATH]
  sim_client.py --scenario FILE --connect HOST:PORT [--realtime]
  sim_client.py --interactive [...]       (type scenario commands)

By default it starts `PATH --lockstep --vehicle V --port P` on a free port
and drives its clock: time advances only when the client steps it (sim-only
channel 0150 = N centiseconds, acknowledged on 0151), so runs are
deterministic. With --connect it attaches to a running AGC (yaAGC, or
agc_sim without --lockstep) and steps by wall-clock time instead.

Scenario language (one command per line, `#` comments; times in cs):
  vehicle csm|lm              must match --vehicle (or sets it)
  pad ITEM ...                erasable pad load before power-on: the pad
                              lines are passed to agc_sim --pad (src/sim/
                              pad.c: refsmmat, flagword); not available
                              with --connect
  run N                       advance N centiseconds
  keys V06N36E                key presses: digits, V N E C(LR) R(SET) K(EY REL)
                              + -; spaces ignored; words PRO, KEYREL allowed
  pro                         press PRO for 0.5 s (T4RUPT samples it)
  keygap N                    centiseconds between key presses (default 20)
  input CH VALUE [MASK]       input channel (octal), through a u-bit mask
  counter ADDR TYPE [COUNT]   counter increments (octal addr; TYPE PINC
                              MINC PCDU MCDU DINC SHINC SHANC or number)
  expect FIELD VALUE [~TOL]   check now; FIELD: PROG VERB NOUN R1 R2 R3
                              (value like 00, +00123, blank), FLASH on|off,
                              LAMP name on|off, CH ch value (octal), CLOCK
                              [now] (R1-R3 = N36 hours, minutes, seconds
                              show TIME at the last ENTER, or now; ~TOL cs)
  clockref H M S.SS           the AGC clock read H:M:S.SS at the last ENTER
                              (after a V25N36 load); CLOCK checks follow it
  until FIELD VALUE [within N] run until the check holds (default 1000 cs)
  xexpect BUG FIELD VALUE ... known flight-code bug (docs/modules/sim.md,
  xuntil BUG FIELD VALUE ...  "Bugs found", id BUG): the AGC-correct
                              expectation, expected to FAIL today. A failure
                              is reported XFAIL and does not fail the run; a
                              pass is reported XPASS (bug fixed: turn it
                              into expect) and fails the run under --strict
  show                        print the display
Exit status 0 if every expectation held, 1 otherwise.
"""
import argparse
import os
import select
import socket
import subprocess
import sys
import tempfile
import time

CH_RELAY, CH_LAMPS11, CH_DSKY163 = 0o10, 0o11, 0o163
CH_STEP, CH_STEP_ACK = 0o150, 0o151

RELAY_DIGIT = {0o25: '0', 0o3: '1', 0o31: '2', 0o33: '3', 0o17: '4',
               0o36: '5', 0o34: '6', 0o23: '7', 0o35: '8', 0o37: '9', 0: ' '}
KEY_CODE = {'0': 0o20, '1': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6,
            '7': 7, '8': 0o10, '9': 0o11, 'V': 0o21, 'N': 0o37, 'E': 0o34,
            'C': 0o36, 'R': 0o22, 'K': 0o31, '+': 0o32, '-': 0o33}
INC_TYPE = {'PINC': 0, 'PCDU': 1, 'MINC': 2, 'MCDU': 3, 'DINC': 4,
            'SHINC': 5, 'SHANC': 6}
# lamp name -> (channel, bit mask); relay row 12 of channel 010 is 'R12'.
LAMPS = {
    'PRIODISP': ('R12', 0o1), 'NODAP': ('R12', 0o2), 'VEL': ('R12', 0o4),
    'NOATT': ('R12', 0o10), 'ALT': ('R12', 0o20), 'GIMBAL': ('R12', 0o40),
    'TRACKER': ('R12', 0o200), 'PROG': ('R12', 0o400),
    'COMPACTY': (CH_LAMPS11, 0o2), 'UPLINK': (CH_LAMPS11, 0o4),
    'TEMP': (CH_DSKY163, 0o10), 'KEYREL': (CH_LAMPS11, 0o20),
    'FLASH': (CH_LAMPS11, 0o40), 'OPRERR': (CH_LAMPS11, 0o100),
    'RESTART': (CH_DSKY163, 0o200), 'STBY': (CH_DSKY163, 0o400),
    'ENGINEON': (CH_LAMPS11, 0o10000), 'ENGINEOFF': (CH_LAMPS11, 0o20000),
}


def form_packet(chan, value, ubit=False):
    """yaAGC FormIoPacket."""
    p0 = (chan >> 3) | (0x20 if ubit else 0)
    return bytes([p0, 0x40 | ((chan << 3) & 0x38) | ((value >> 12) & 7),
                  0x80 | ((value >> 6) & 0x3F), 0xC0 | (value & 0x3F)])


def parse_packet(p):
    """yaAGC ParseIoPacket -> (chan, value, ubit) or None."""
    if (p[0] & 0xC0, p[1] & 0xC0, p[2] & 0xC0, p[3] & 0xC0) != (0, 0x40, 0x80, 0xC0):
        return None
    chan = ((p[0] & 0x1F) << 3) | ((p[1] >> 3) & 7)
    value = ((p[1] << 12) & 0x7000) | ((p[2] << 6) & 0x0FC0) | (p[3] & 0x3F)
    return chan, value, bool(p[0] & 0x20)


class Dsky:
    """The DSKY as yaDSKY2 decodes it (ActOnIncomingIO)."""

    def __init__(self):
        self.d = {k: ' ' for k in ('MD1', 'MD2', 'VD1', 'VD2', 'ND1', 'ND2')}
        for r in (1, 2, 3):
            for i in range(1, 6):
                self.d['R%dD%d' % (r, i)] = ' '
        self.sign = {1: 0, 2: 0, 3: 0}     # bit 2 plus, bit 1 minus
        self.chan = {}                     # last value per channel
        self.row12 = 0

    def digit(self, code):
        return RELAY_DIGIT.get(code, '?')

    def relay(self, v):
        row, b = (v >> 11) & 0o17, (v >> 10) & 1
        left, right = self.digit((v >> 5) & 0o37), self.digit(v & 0o37)
        pairs = {11: ('MD1', 'MD2'), 10: ('VD1', 'VD2'), 9: ('ND1', 'ND2'),
                 8: (None, 'R1D1'), 7: ('R1D2', 'R1D3'), 6: ('R1D4', 'R1D5'),
                 5: ('R2D1', 'R2D2'), 4: ('R2D3', 'R2D4'), 3: ('R2D5', 'R3D1'),
                 2: ('R3D2', 'R3D3'), 1: ('R3D4', 'R3D5')}
        if row == 12:
            self.row12 = v & 0o3777
            return
        if row not in pairs:
            return
        lk, rk = pairs[row]
        if lk:
            self.d[lk] = left
        self.d[rk] = right
        signs = {7: (1, 2), 6: (1, 1), 5: (2, 2), 4: (2, 1), 2: (3, 2), 1: (3, 1)}
        if row in signs:
            reg, bit = signs[row]
            self.sign[reg] = (self.sign[reg] | bit) if b else (self.sign[reg] & ~bit)

    def take(self, chan, value):
        self.chan[chan] = value
        if chan == CH_RELAY:
            self.relay(value)

    def field(self, name):
        n = name.upper()
        if n in ('PROG', 'VERB', 'NOUN'):
            k = {'PROG': 'MD', 'VERB': 'VD', 'NOUN': 'ND'}[n]
            return self.d[k + '1'] + self.d[k + '2']
        if n in ('R1', 'R2', 'R3'):
            r = int(n[1])
            s = self.sign[r]
            sg = '+' if s & 2 else ('-' if s & 1 else ' ')
            return sg + ''.join(self.d['R%dD%d' % (r, i)] for i in range(1, 6))
        raise ValueError('unknown field ' + name)

    def lamp(self, name):
        ch, mask = LAMPS[name.upper()]
        v = self.row12 if ch == 'R12' else self.chan.get(ch, 0)
        return bool(v & mask)

    def text(self):
        lit = [k for k in LAMPS if k not in ('FLASH', 'COMPACTY') and self.lamp(k)]
        fl = ' FLASH' if self.lamp('FLASH') else ''
        return ('PROG %s VERB %s NOUN %s%s | R1 %s R2 %s R3 %s | %s' % (
            self.field('PROG'), self.field('VERB'), self.field('NOUN'), fl,
            self.field('R1'), self.field('R2'), self.field('R3'),
            ' '.join(lit) or '-'))


class Link:
    """Socket to the AGC; lockstep or wall-clock stepping."""

    def __init__(self, host, port, lockstep, trace=False):
        self.s = None
        for _ in range(200):
            try:
                self.s = socket.create_connection((host, port), timeout=5)
                break
            except OSError:
                time.sleep(0.05)
        if self.s is None:
            raise SystemExit('sim_client: cannot connect to %s:%d' % (host, port))
        self.s.setblocking(False)
        self.buf = b''
        self.lockstep = lockstep
        self.dsky = Dsky()
        self.cs = 0
        self.trace = trace
        self.downlink = []

    def send(self, chan, value, ubit=False):
        self.s.sendall(form_packet(chan, value, ubit))

    def _drain(self, timeout):
        """Read what is there (waiting up to timeout s); returns packets."""
        out = []
        r, _, _ = select.select([self.s], [], [], timeout)
        if not r:
            return out
        data = self.s.recv(65536)
        if not data:
            raise SystemExit('sim_client: AGC closed the connection')
        self.buf += data
        while len(self.buf) >= 4:
            if self.buf[0] & 0xC0:
                self.buf = self.buf[1:]      # resync (keep-alive 0xFF bytes)
                continue
            p = parse_packet(self.buf[:4])
            self.buf = self.buf[4:]
            if p:
                out.append(p)
        return out

    def _take(self, packets):
        acked = False
        for chan, value, _ in packets:
            if chan == CH_STEP_ACK:
                acked = True
                continue
            if self.trace:
                print('  %7d < %04o %05o' % (self.cs, chan, value))
            if chan in (0o34, 0o35):
                self.downlink.append((self.cs, chan, value))
            self.dsky.take(chan, value)
        return acked

    def step(self, n):
        while n > 0:
            k = min(n, 0o77777)
            if self.lockstep:
                self.send(CH_STEP, k)
                deadline = time.time() + 60
                while not self._take(self._drain(1.0)):
                    if time.time() > deadline:
                        raise SystemExit('sim_client: no step acknowledgement')
            else:
                end = time.time() + k / 100.0
                while time.time() < end:
                    self._take(self._drain(max(0.0, min(0.05, end - time.time()))))
            self.cs += k
            n -= k


class Scenario:
    def __init__(self, link, vehicle, max_cs, verbose):
        self.l = link
        self.vehicle = vehicle
        self.max_cs = max_cs
        self.verbose = verbose
        self.keygap = 20
        self.fail = 0
        self.checks = 0
        self.xfail = 0
        self.xpass = 0
        self.strict = False
        self.enter_cs = None
        self.clock_base = 0                # AGC TIME minus client cs
        self.last_text = ''

    def say(self, msg):
        print('%8.2fs %s' % (self.l.cs / 100.0, msg))

    def watch(self):
        t = self.l.dsky.text()
        if self.verbose and t != self.last_text:
            self.say(t)
        self.last_text = t

    def run(self, n):
        if self.max_cs and self.l.cs + n > self.max_cs:
            n = max(0, self.max_cs - self.l.cs)
        step = 10 if self.verbose else n
        while n > 0:
            k = min(step, n)
            self.l.step(k)
            n -= k
            self.watch()

    def key(self, code):
        self.l.send(0o15, code)
        if code == KEY_CODE['E']:
            self.enter_cs = self.l.cs
        self.run(self.keygap)

    def keys(self, text):
        t = text.upper().replace('KEYREL', 'K').replace('KEY REL', 'K')
        i = 0
        while i < len(t):
            if t.startswith('PRO', i):
                self.pro()
                i += 3
                continue
            c = t[i]
            i += 1
            if c in ' \t':
                continue
            if c not in KEY_CODE:
                raise ValueError('bad key %r' % c)
            self.key(KEY_CODE[c])

    def pro(self):
        self.l.send(0o32, 0o20000, ubit=True)   # mask: bit 14 only
        self.l.send(0o32, 0)                    # pressed (inverted input)
        self.run(50)
        self.l.send(0o32, 0o20000)              # released
        self.run(self.keygap)

    def value_ok(self, field, want, tol):
        d = self.l.dsky
        f = field.upper()
        if f == 'FLASH':
            return d.lamp('FLASH') == (want.lower() == 'on'), d.lamp('FLASH')
        if f == 'LAMP':
            name, state = want.split()
            return d.lamp(name) == (state.lower() == 'on'), d.lamp(name)
        if f == 'CH':
            ch, v = (int(x, 8) for x in want.split())
            got = d.chan.get(ch, 0)
            return got == v, '%05o' % got
        if f == 'CLOCK':
            return self.clock_ok(tol, want.strip().lower() == 'now')
        got = d.field(f)
        w = want.replace('_', ' ')
        if w.lower() == 'blank':
            w = ' ' * len(got)
        if tol is not None and got.strip() and w.strip():
            try:
                return abs(int(got.replace(' ', '')) - int(w)) <= tol, got
            except ValueError:
                return False, got
        return got.strip() == w.strip() and len(got) >= len(w.strip()), got

    def clock_ok(self, tol, now=False):
        d = self.l.dsky
        try:
            h, m, s = (int(d.field(r).replace(' ', '')) for r in ('R1', 'R2', 'R3'))
        except ValueError:
            return False, d.text()
        shown = (h * 3600 + m * 60) * 100 + s
        at = self.l.cs if now or self.enter_cs is None else self.enter_cs
        ref = at + self.clock_base
        return abs(shown - ref) <= (tol or 50), '%d cs (want %d cs)' % (shown, ref)

    def check(self, args, quiet=False, bug=None):
        tol = None
        if args and args[-1].startswith('~'):
            tol = int(args[-1][1:])
            args = args[:-1]
        field, want = args[0], ' '.join(args[1:])
        ok, got = self.value_ok(field, want, tol)
        if quiet:
            return ok
        self.checks += 1
        if bug is None:
            self.fail += 0 if ok else 1
            tag = 'ok  ' if ok else 'FAIL'
            self.say('%s expect %s %s: %s' % (tag, field, want, got))
            return ok
        if ok:
            self.xpass += 1
            self.fail += 1 if self.strict else 0
        else:
            self.xfail += 1
        self.say('%s xexpect [%s] %s %s: %s' % ('XPASS' if ok else 'XFAIL',
                                                bug, field, want, got))
        return ok

    def until(self, args, bug=None):
        within = 1000
        if 'within' in args:
            i = args.index('within')
            within = int(args[i + 1])
            args = args[:i] + args[i + 2:]
        start = self.l.cs
        while self.l.cs - start < within:
            if self.check(args, quiet=True):
                break
            self.run(5)
        self.check(args, bug=bug)

    def command(self, line):
        w = line.split()
        op = w[0].lower()
        if op == 'vehicle':
            if w[1].lower() != self.vehicle:
                raise SystemExit('scenario is for %s, run with --vehicle %s'
                                 % (w[1], w[1]))
        elif op == 'pad':
            pass                        # applied at start-up (main)
        elif op == 'run':
            self.run(int(w[1]))
        elif op == 'keys':
            self.keys(' '.join(w[1:]))
        elif op == 'pro':
            self.pro()
        elif op == 'keygap':
            self.keygap = int(w[1])
        elif op == 'input':
            ch, v = int(w[1], 8), int(w[2], 8)
            if len(w) > 3:
                self.l.send(ch, int(w[3], 8), ubit=True)
            self.l.send(ch, v)
        elif op == 'counter':
            addr = int(w[1], 8)
            typ = INC_TYPE.get(w[2].upper(), None)
            typ = int(w[2], 8) if typ is None else typ
            for _ in range(int(w[3]) if len(w) > 3 else 1):
                self.l.send(0o200 | addr, typ)
        elif op == 'expect':
            self.check(w[1:])
        elif op == 'until':
            self.until(w[1:])
        elif op == 'xexpect':
            self.check(w[2:], bug=w[1])
        elif op == 'xuntil':
            self.until(w[2:], bug=w[1])
        elif op == 'clockref':
            h, m, sec = int(w[1]), int(w[2]), float(w[3])
            value = (h * 3600 + m * 60) * 100 + int(round(sec * 100))
            self.clock_base = value - (self.enter_cs or 0)
        elif op == 'show':
            self.say(self.l.dsky.text())
        else:
            raise SystemExit('sim_client: unknown command %r' % line)

    def play(self, lines):
        for raw in lines:
            line = raw.split('#', 1)[0].strip()
            if not line:
                continue
            if self.max_cs and self.l.cs >= self.max_cs:
                self.say('max-cs reached')
                break
            self.command(line)


def free_port():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


def scenario_vehicle(path):
    with open(path) as fh:
        for raw in fh:
            w = raw.split('#', 1)[0].split()
            if len(w) == 2 and w[0].lower() == 'vehicle':
                return w[1].lower()
    return None


def scenario_pads(path):
    out = []
    with open(path) as fh:
        for raw in fh:
            w = raw.split('#', 1)[0].split()
            if w and w[0].lower() == 'pad':
                out.append(' '.join(w[1:]))
    return out


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--scenario')
    ap.add_argument('--interactive', action='store_true')
    ap.add_argument('--vehicle', choices=('csm', 'lm'))
    ap.add_argument('--max-cs', type=int, default=0)
    ap.add_argument('--sim', default='build/agc_sim')
    ap.add_argument('--connect')
    ap.add_argument('--realtime', action='store_true')
    ap.add_argument('--verbose', '-v', action='store_true')
    ap.add_argument('--trace', action='store_true')
    ap.add_argument('--strict', action='store_true',
                    help='an XPASS (known bug no longer seen) fails the run')
    a = ap.parse_args(argv[1:])
    vehicle = a.vehicle or (scenario_vehicle(a.scenario) if a.scenario else None) or 'csm'
    proc = None
    pads = scenario_pads(a.scenario) if a.scenario else []
    padfile = None
    if a.connect:
        if pads:
            print('sim_client: pad lines ignored with --connect')
        host, port = a.connect.rsplit(':', 1)
        link = Link(host, int(port), lockstep=not a.realtime, trace=a.trace)
    else:
        port = free_port()
        cmd = [a.sim, '--lockstep', '--vehicle', vehicle, '--port', str(port)]
        if pads:
            fd, padfile = tempfile.mkstemp(prefix='agc_sim_pad_', suffix='.txt')
            with os.fdopen(fd, 'w') as fh:
                fh.write('\n'.join(pads) + '\n')
            cmd += ['--pad', padfile]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        link = Link('127.0.0.1', port, lockstep=True, trace=a.trace)
    sc = Scenario(link, vehicle, a.max_cs, a.verbose)
    sc.strict = a.strict
    try:
        if a.scenario:
            with open(a.scenario) as fh:
                sc.play(fh.readlines())
        if a.interactive:
            for line in sys.stdin:
                if line.strip():
                    sc.play([line])
                    sc.say(link.dsky.text())
    finally:
        link.s.close()
        if proc:
            out, _ = proc.communicate(timeout=30)
            sys.stdout.write(out.decode())
        if padfile:
            os.unlink(padfile)
    print('sim_client: %s %s: %d checks, %d failed, %d xfail, %d xpass, '
          '%.2f s of AGC time' % (
              os.path.basename(a.scenario or 'interactive'), vehicle,
              sc.checks, sc.fail, sc.xfail, sc.xpass, link.cs / 100.0))
    return 1 if sc.fail else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
