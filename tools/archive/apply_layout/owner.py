#!/usr/bin/env python3
"""owner.py ROOT ANALYSIS_DIR XREF.tsv -- judge analyze.py's a_wrong_owner.tsv by AGC section (one-off)."""
import collections, os, re, sys
root, an, xref = sys.argv[1:4]
os.chdir(root)
def secs_of(f):
    t = open(f, errors='replace').read()
    m = re.search(r'AGC SOURCE:?\s*\n((?:\s*\*[^\n]*\n)+?)\s*\*\s*\n', t)
    s = m.group(1) if m else t[:3000]
    return set(x.upper().replace('-', '_') for x in re.findall(r'([A-Z][A-Z0-9_+-]{2,})(?:\.agc)?', s) if '_' in x or x.endswith('.AGC'))
fsecs = {}
def secs(f):
    if f not in fsecs: fsecs[f] = secs_of(f)
    return fsecs[f]
fn_agc = collections.defaultdict(set)
for l in list(open(xref))[1:]:
    lab, agc, fn, f, line, kind = l.rstrip('\n').split('\t')
    for part in agc.split(';'):
        m = re.search(r'[CL] (\S+) p', part.strip())
        if m: fn_agc[fn].add(m.group(1).upper().replace('-', '_'))
rows = [l.rstrip('\n').split('\t') for l in open(os.path.join(an, 'a_wrong_owner.tsv'))]
hdr, rows = rows[0], rows[1:]
out = []
for r in rows:
    fn, f, line, d = r[0], r[1], r[2], r[3]
    a = fn_agc.get(fn, set())
    own = secs(f)
    tgt = set()
    for g in os.listdir(d):
        if g.endswith('.c'): tgt |= secs(os.path.join(d, g))
    if not a:
        v = 'no-label (accessor/helper: keep with its storage)'
    elif a & own:
        v = 'section matches own file: keep'
    elif a & tgt:
        v = 'SECTION OF TARGET: move'
    else:
        v = 'section in neither (' + ','.join(sorted(a)) + ')'
    out.append(r + [v])
with open(os.path.join(an, 'a_judged.tsv'), 'w') as o:
    o.write('\t'.join(hdr + ['verdict']) + '\n')
    for r in out: o.write('\t'.join(r) + '\n')
print(collections.Counter(r[-1].split(' (')[0] for r in out))
for r in out:
    if r[-1].startswith('SECTION') or r[-1].startswith('section in'):
        print(r[0], r[1], '->', r[3], r[6][:40], r[-1])
