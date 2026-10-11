"""Small builder for agc_probe specs (see tools/oracle/README.md)."""
class Spec:
    def __init__(self, rope, comment, maxcycles=5000000):
        self.l = ['# ' + c if c else '#' for c in comment]
        self.l += ['rope ' + rope, 'entry INTPRET', 'maxcycles %d' % maxcycles]
    def comment(self, *c):
        self.l += ['# ' + x for x in c]
    def run(self, tag, q, fbank, stop, sets, gets, ebank=None):
        """sets: list of (name, 'w,w,...'); gets: list of 'NAME[:dp|:tp]'.
        Every run gets FIXLOC = PUSHLOC = VAC1 (the job's work area)."""
        self.l += ['clearstops'] + ['stop ' + s for s in stop.split()]
        self.l += ['cleargets'] + ['get ' + g for g in gets]
        self.l += ['q ' + q, 'fbank %o' % fbank]
        if ebank is not None:
            self.l += ['ebank %o' % ebank]
        self.l += ['set FIXLOC=@VAC1', 'set PUSHLOC=@VAC1']
        for name, words in sets:
            w = words.split(',')
            for k in range(0, len(w), 8):      # agc_probe: <= 8 words per set
                self.l.append('set %s=%s' % (off(name, k), ','.join(w[k:k + 8])))
        self.l += ['run ' + tag]
    def write(self, path):
        open(path, 'w').write('\n'.join(self.l) + '\n')
def off(name, k):
    """NAME+k with NAME possibly already 'BASE+n'."""
    if '+' in name[1:]:
        b, n = name.rsplit('+', 1)
        k += int(n)
    else:
        b = name
    return b if k == 0 else '%s+%d' % (b, k)
def gets_dp(name, n):
    """n DP words from NAME (NAME, NAME+2, ...)."""
    return [off(name, 2 * k) + ':dp' for k in range(n)]
def gets_vec(name):
    return gets_dp(name, 3)

def probe(spec_lines):
    """Run agc_probe on a spec (list of lines); return {tag: {NAME: octal}}."""
    import os, subprocess
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.join(here, '..', '..')
    out = subprocess.run([os.path.join(root, 'bin', 'agc_probe')],
                         input='\n'.join(spec_lines) + '\n', capture_output=True,
                         text=True, cwd=root,
                         env=dict(os.environ, AGC_ORACLE_ROPES='ropes')).stdout
    res, cur = {}, None
    for line in out.splitlines():
        if line.startswith('#') or not line.strip():
            continue
        k, _, v = line.partition(' ')
        if k == 'CASE':
            cur = res.setdefault(v, {})
        elif cur is not None:
            cur[k] = v
    return res
