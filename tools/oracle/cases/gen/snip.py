"""Assemble an interpretive snippet with yaYUL for poking into erasable.

assemble(body, fixed={NAME: (bank, sreg)}, eras={NAME: pseudo}) -> [octal str]
body: YUL source lines (tab separated) WITHOUT the leading TC INTPRET; the
words returned start at the first opcode word (Q / LOC points there).
Fixed externals are assembled as local dummy labels, then the address words
are patched to bank*02000 + sreg - 02000 (15-bit interpretive address).
"""
import os, re, shutil, subprocess, tempfile
YUL = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'bin', 'yaYUL')
def faddr(bank, sreg):
    return (bank * 0o2000 + sreg - 0o2000) & 0o77777
def assemble(body, fixed=None, eras=None):
    fixed = fixed or {}; eras = eras or {}
    alias = {n: 'XF%d' % i for i, n in enumerate(fixed)}
    src = ['\t\tBANK\t33', '\t\tCOUNT*\t$$/S', 'INTPRET\t\tEQUALS\t4001']
    for n, v in eras.items():
        src.append('%s\t\tEQUALS\t%o' % (n, v))
    src.append('SNIPX\t\tTC\tINTPRET')
    for l in body:
        for n, a in alias.items():
            l = re.sub(r'(?<![\w/\-])' + re.escape(n) + r'(?![\w/\-])', a, l)
        src.append(l)
    for n, a in alias.items():
        src.append('%s\t\tRVQ' % a)
    d = tempfile.mkdtemp(dir=os.environ.get('SNIPTMP'))
    p = os.path.join(d, 's.agc')
    open(p, 'w').write('\n'.join(src) + '\n')
    out = subprocess.run([YUL, 's.agc'], capture_output=True, text=True, cwd=d).stdout
    shutil.rmtree(d, ignore_errors=True)
    words = []; labels = {}
    for line in out.splitlines():
        if line[15:18] != '33,' or not line[33:38].strip().isdigit():
            if line[15:18] == '33,' and line[46:65].strip():
                labels[line[46:65].strip()] = int(line[18:22], 8)
            continue
        loc = int(line[18:22], 8); w = line[33:38]; lab = line[46:65].strip()
        words.append((loc, w, lab))
        if lab: labels[lab] = loc
    m = re.search(r'(\d+) [Ff]atal', out)
    if m and int(m.group(1)):
        raise SystemExit(out)
    first = labels['SNIPX']
    res = []
    rev = {(0o2000 * 0o33 + labels[a] - 0o2000) & 0o77777: n for n, a in alias.items()}
    dummies = set(labels[a] for a in alias.values())
    for loc, w, lab in words:
        if loc <= first or loc in dummies: continue
        v = int(w, 8)
        if v in rev:
            b, s = fixed[rev[v]]
            v = faddr(b, s)
        res.append('%05o' % v)
    return res
