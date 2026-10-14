#!/usr/bin/env python3
"""analyze.py ROOT XREF.tsv OUTDIR -- structural findings (a)-(g) as TSVs."""
import collections, os, re, subprocess, sys
root, xref_tsv, outd = sys.argv[1:4]
os.chdir(root)
os.makedirs(outd, exist_ok=True)
def read(p):
    return open(p, encoding='utf-8', errors='replace').read()
COMMENT = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\\n])*"', re.S)
def strip(t):
    return COMMENT.sub(lambda m: ' ' * 0 + ('\n' * m.group(0).count('\n')), t)
TOK = re.compile(r'\b[A-Za-z_]\w*\b')
ls = subprocess.run(['git', 'ls-files'], capture_output=True, text=True).stdout.split()
csrc = [f for f in ls if f.startswith('src/') and f.endswith(('.c', '.h')) and not f.startswith('src/nassp/')]
hdrs = [f for f in ls if f.startswith('include/') and f.endswith('.h')]
tests = [f for f in ls if f.startswith('tests/') and f.endswith(('.c', '.h'))]
extra = [f for f in ['src/nassp/engine.c', 'sim/nassp/replay.c'] if os.path.exists(f)]
allf = csrc + hdrs + tests + extra
raw = {f: read(f) for f in allf}
txt = {f: strip(raw[f]) for f in allf}
toks = {f: set(TOK.findall(txt[f])) for f in allf}
def mod(f):
    return os.path.dirname(f)
def write(name, header, rows):
    with open(os.path.join(outd, name), 'w') as o:
        o.write('\t'.join(header) + '\n')
        for r in rows:
            o.write('\t'.join(str(x) for x in r) + '\n')
    print(name, len(rows))
# ---- call graph
cg = []
with open('history/data/callgraph.tsv') as fh:
    hdr = fh.readline().rstrip('\n').split('\t')
    for line in fh:
        r = dict(zip(hdr, line.rstrip('\n').split('\t')))
        cg.append(r)
byname = collections.defaultdict(list)
for r in cg:
    byname[r['callee']].append(r['file'])
def caller_files(callee, callers):
    out = []
    for c in callers:
        if c.startswith('test:') or c == '-' or not c:
            continue
        fs = byname.get(c, [])
        fs2 = [f for f in fs if callee in toks.get(f, ())]
        out.append((c, (fs2 or fs or ['?'])[0]))
    return out
ext_users = {n for f in extra for n in toks[f]}
reg = [l.rstrip('\n').split('\t') for l in open('tools/registry.tsv') if l.strip() and not l.startswith('#')]
reg_csm = {r[2] for r in reg if len(r) > 3}
reg_lm = {r[3] for r in reg if len(r) > 3}
# function extents (brace matching from the definition line)
def extent(f, line):
    lines = raw[f].split('\n')
    i = int(line) - 1
    depth = 0; started = False
    for j in range(i, len(lines)):
        s = COMMENT.sub('', lines[j])
        for ch in s:
            if ch == '{':
                depth += 1; started = True
            elif ch == '}':
                depth -= 1
        if started and depth == 0:
            return i, j
    return i, i
file_statics = {}
def statics(f):
    if f not in file_statics:
        names = set()
        for m in re.finditer(r'^static\b[^;{(]*?\b([A-Za-z_]\w*)\s*(\(|\[|=|;)', txt[f], re.M):
            names.add(m.group(1))
        file_statics[f] = names
    return file_statics[f]
# ---- (a) wrong owner, (d) dead, (f) vehicle split
rows_a, rows_d, rows_f = [], [], []
for r in cg:
    f = r['file']
    if f.startswith('src/nassp/'):
        continue
    name = r['callee']
    callers = [c for c in r['callers'].split(',') if c and c != '-']
    cf = caller_files(name, callers)
    test_callers = [c for c in callers if c.startswith('test:')]
    nonself = [(c, g) for c, g in cf if g != f]
    dispatch = r['dispatch'] == 'yes'
    if r['static'] == 'extern':
        # (d)
        if not cf and not dispatch and name not in ext_users and name != 'main':
            rows_d.append((name, f, r['line'], r['agc_label'], len(test_callers), r['public']))
        # (a)
        if cf and not dispatch:
            dirs = collections.Counter(mod(g) for c, g in cf)
            own = sum(n for d, n in dirs.items() if d == mod(f))
            others = {d: n for d, n in dirs.items() if d != mod(f)}
            if others and own == 0 and len(others) == 1 and name not in ext_users:
                (d, n), = others.items()
                rel = 'near' if (d.startswith(mod(f) + '/') or mod(f).startswith(d + '/')) else 'far'
                a, b = extent(f, r['line'])
                body = set(TOK.findall(COMMENT.sub('', '\n'.join(raw[f].split('\n')[a:b + 1]))))
                uses = sorted((body & statics(f)) - {name})
                rows_a.append((name, f, r['line'], d, n, rel, 'movable' if not uses else 'uses-statics:' + ','.join(uses[:5]), r['agc_label']))
    # (f) vehicle
    top = f.split('/')[1]
    vdirs = {mod(g).split('/')[1] for c, g in cf if g.startswith('src/')}
    if dispatch:
        if name in reg_csm: vdirs.add('csm*')
        if name in reg_lm: vdirs.add('lm*')
    if top in ('csm', 'lm'):
        other = 'lm' if top == 'csm' else 'csm'
        if other in vdirs or other + '*' in vdirs:
            rows_f.append((name, f, 'vehicle code called by the other vehicle', ','.join(sorted(vdirs))))
    elif top not in ('core', 'exec', 'sim', 'erasable_map') and r['static'] == 'extern':
        veh = {v.rstrip('*') for v in vdirs if v.rstrip('*') in ('csm', 'lm')}
        shared = {v for v in vdirs if v.rstrip('*') not in ('csm', 'lm')}
        if len(veh) == 1 and not shared and (cf or dispatch):
            rows_f.append((name, f, 'shared-folder code called by one vehicle only', ','.join(sorted(vdirs))))
write('a_wrong_owner.tsv', ['function', 'file', 'line', 'callers_dir', 'n_callers', 'relation', 'movable', 'agc_label'], rows_a)
write('d_dead.tsv', ['function', 'file', 'line', 'agc_label', 'test_callers', 'public'], rows_d)
write('f_vehicle.tsv', ['function', 'file', 'finding', 'caller_dirs'], rows_f)
# ---- (b) header internals
def ctags(f):
    out = subprocess.run(['ctags', '-x', '--c-kinds=+p-m', f], capture_output=True, text=True).stdout
    res = []
    for l in out.split('\n'):
        p = l.split(None, 4)
        if len(p) >= 4:
            res.append((p[0], p[1], p[2]))
    return res
defs = {}
for r in cg:
    if r['static'] == 'extern':
        defs.setdefault(r['callee'], r['file'])
rows_b = []
hdr_syms = {}
for h in hdrs:
    syms = ctags(h)
    hdr_syms[h] = {s for s, k, l in syms}
    for s, k, l in syms:
        if k in ('member',) or s.startswith('AGC_') and k == 'macro' and s.endswith('_H'):
            continue
        users = [f for f in allf if f != h and s in toks[f]]
        src_users = [f for f in users if f.startswith('src/') or f in extra]
        test_users = [f for f in users if f.startswith('tests/')]
        inc_users = [f for f in users if f.startswith('include/')]
        if inc_users:
            continue
        d = defs.get(s)
        if k == 'prototype':
            if not src_users or (d and set(src_users) <= {d}):
                rows_b.append((s, k, h, l, d or '-', 'file-static' if not test_users else 'internal (def file + tests)', len(test_users)))
            elif d and len({mod(u) for u in src_users}) == 1 and mod(d) == mod(src_users[0]):
                rows_b.append((s, k, h, l, d, 'internal.h of ' + mod(d), len(test_users)))
        else:
            if len(src_users) == 1:
                rows_b.append((s, k, h, l, src_users[0], 'file-local (one .c)' + ('' if not test_users else ' + tests'), len(test_users)))
            elif not src_users and not test_users:
                rows_b.append((s, k, h, l, '-', 'unused', 0))
write('b_header_internals.tsv', ['symbol', 'kind', 'header', 'line', 'only_user', 'proposal', 'test_users'], rows_b)
# ---- (c) duplicate translations
xr = [l.rstrip('\n').split('\t') for l in open(xref_tsv)][1:]
bylab = collections.defaultdict(list)
for lab, agc, fn, f, line, kind in xr:
    if kind == 'prolog':
        bylab[lab].append((fn, f, line, agc))
rows_c = []
def lane(f):
    p = f.split('/')
    return '/'.join(p[1:3]) if p[1] in ('csm', 'lm', 'programs', 'nav', 'io') else p[1]
for lab, fs in sorted(bylab.items()):
    files = {f for fn, f, line, agc in fs}
    lanes = {lane(f) for f in files}
    if len(lanes) < 2:
        continue
    vehs = {f.split('/')[1] for f in files}
    ropes = fs[0][3]
    kind = 'per-vehicle variant' if vehs == {'csm', 'lm'} else 'cross-lane'
    rows_c.append((lab, ropes, kind, '; '.join('%s %s:%s' % (fn, f, line) for fn, f, line, agc in fs)))
write('c_duplicates.tsv', ['label', 'rope', 'kind', 'functions'], rows_c)
# ---- (e) files
rows_e = []
for f in sorted(csrc):
    if not f.endswith('.c'):
        continue
    n = raw[f].count('\n')
    m = re.search(r'AGC SOURCE:\s*\n((?:\s*\*[^\n]*\n)+?)\s*\*\s*\n', raw[f])
    secs = set(re.findall(r'([A-Z][A-Z0-9_+-]{3,})\.agc', m.group(1))) if m else set()
    secs |= set(re.findall(r'\b([A-Z][A-Z0-9-]*_[A-Z0-9_-]+)\b(?=[^\n]*pp?\.)', m.group(1))) if m else set()
    if n > 800 or len(secs) > 1:
        rows_e.append((f, n, len(secs), ','.join(sorted(secs))))
write('e_files.tsv', ['file', 'lines', 'n_sections', 'sections'], rows_e)
# ---- (g) include graph
inc = collections.defaultdict(set)
INC = re.compile(r'^\s*#\s*include\s+"([^"]+)"', re.M)
for f in allf:
    for m in INC.finditer(raw[f]):
        p = m.group(1)
        cand = [os.path.normpath(os.path.join(os.path.dirname(f), p)), os.path.join('include', p)]
        for c in cand:
            if os.path.exists(c):
                inc[f].add(c); break
# cycles among headers (Tarjan)
idx = {}; low = {}; st = []; on = set(); sccs = []; counter = [0]
sys.setrecursionlimit(10000)
def sc(v):
    idx[v] = low[v] = counter[0]; counter[0] += 1; st.append(v); on.add(v)
    for w in inc.get(v, ()):
        if w not in idx:
            sc(w); low[v] = min(low[v], low[w])
        elif w in on:
            low[v] = min(low[v], idx[w])
    if low[v] == idx[v]:
        comp = []
        while True:
            w = st.pop(); on.discard(w); comp.append(w)
            if w == v: break
        if len(comp) > 1: sccs.append(comp)
for v in list(inc):
    if v not in idx: sc(v)
write('g_cycles.tsv', ['cycle'], [(' -> '.join(c),) for c in sccs])
for h in [f for f in allf if f.endswith('.h') and f not in hdr_syms]:
    hdr_syms[h] = {s for s, k, l in ctags(h)}
rows_g = []
for f in sorted(inc):
    for h in sorted(inc[f]):
        syms = hdr_syms.get(h, set())
        if syms and not (syms & toks[f]):
            rows_g.append((f, h, len(syms)))
write('g_unused_includes.tsv', ['file', 'header', 'header_symbols'], rows_g)
