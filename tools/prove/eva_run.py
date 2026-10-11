#!/usr/bin/env python3
"""eva_run.py -- Frama-C EVA over libagc, one analysis per entry point.

Usage (from the repo root; `make prove` runs it):
    python3 -I tools/prove/eva_run.py [--jobs N] [--only NAME...]
        [--out build/prove] [--precision P]

Every AGC_ASSERT(e) is replaced by `agc_eva_check((e) ? 1 : 0)`, whose ACSL
precondition `c != 0` EVA checks at each call site
(tools/prove/eva_assert.h, force-included; the real assert.h is skipped by
its include guard).  The entry points are the program's boundaries: the
sim harness (src/sim/*.c, main) and every test program (tests/**/*.c,
main), each analysed with the whole library.  Per entry point the Report
plug-in writes <out>/<name>.csv; the merged per-site verdict goes to
<out>/eva_verdicts.tsv:

    file  line  verdict  mains

verdict, over every analysis that reached the site:
    VALID    the condition holds on every path of every analysis
    UNKNOWN  EVA could not decide it on some path (imprecision or a real
             failure path)
    INVALID  EVA found a path on which it fails
    DEAD     no analysis reached the site

The flags of the build that matter for EVA: -DAGC_HOST_DEBUG (the test
library), -c11 (_Static_assert), and the AGC docs' `/*@` comment markers
are not ACSL (annot-error demoted to a warning).
"""
import argparse
import collections
import concurrent.futures
import csv
import glob
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SHIM = 'tools/prove/eva_assert.h'
RANK = {'INVALID': 3, 'UNKNOWN': 2, 'VALID': 1}


def lib_sources():
    out = []
    for dp, _, files in os.walk(os.path.join(ROOT, 'src')):
        rel = os.path.relpath(dp, ROOT)
        if rel == 'src/sim' or rel.startswith('src/sim/'):
            continue
        out += [os.path.join(rel, f) for f in files if f.endswith('.c')]
    return sorted(out)


def entry_points(with_sim=False):
    eps = [('sim', sorted(glob.glob('src/sim/*.c', root_dir=ROOT)))] if with_sim else []
    for t in sorted(glob.glob('tests/**/*.c', root_dir=ROOT, recursive=True)):
        eps.append((t[len('tests/'):-2].replace('/', '_'), [t]))
    return eps


def run_one(name, extra, out, precision, timeout):
    csv_path = os.path.join(out, name + '.csv')
    log_path = os.path.join(out, name + '.log')
    cmd = ['frama-c', '-c11', '-kernel-warn-key', 'annot-error=active',
           '-cpp-extra-args=-Iinclude -include ' + SHIM + ' -DAGC_HOST_DEBUG',
           *lib_sources(), *extra,
           '-main', 'main', '-eva', '-eva-precision', str(precision),
           '-eva-no-alloc-returns-null',
           '-then', '-report-csv', csv_path]
    with open(log_path, 'w') as log:
        try:
            r = subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
        except subprocess.TimeoutExpired:
            return name, 'timeout'
    return name, r.returncode


def merge(out, names):
    site = collections.defaultdict(lambda: {'v': None, 'mains': set()})
    for name in names:
        path = os.path.join(out, name + '.csv')
        if not os.path.exists(path):
            continue
        with open(path, encoding='utf-8', errors='replace') as fh:
            for row in csv.DictReader(fh, delimiter='\t'):
                if row['property kind'] != 'precondition of agc_eva_check':
                    continue
                key = (os.path.join(row['directory'], row['file']), int(row['line']))
                st = row['status'].lower()
                if st.startswith('dead') or st.startswith('unreachable'):
                    v = None
                elif st.startswith('valid'):
                    v = 'VALID'
                elif st.startswith('invalid'):
                    v = 'INVALID'
                else:
                    v = 'UNKNOWN'
                rec = site[key]
                if v is None:
                    continue
                rec['mains'].add(name)
                if rec['v'] is None or RANK[v] > RANK[rec['v']]:
                    rec['v'] = v
    with open(os.path.join(out, 'eva_verdicts.tsv'), 'w') as fh:
        fh.write('file\tline\tverdict\tmains\n')
        for (f, ln) in sorted(site):
            rec = site[(f, ln)]
            fh.write(f"{f}\t{ln}\t{rec['v'] or 'DEAD'}\t{','.join(sorted(rec['mains'])) or '-'}\n")
    counts = collections.Counter(r['v'] or 'DEAD' for r in site.values())
    return counts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--jobs', type=int, default=8)
    ap.add_argument('--only', nargs='*')
    ap.add_argument('--out', default='build/prove')
    ap.add_argument('--precision', type=int, default=0)
    ap.add_argument('--merge-only', action='store_true')
    ap.add_argument('--timeout', type=int, default=120)
    ap.add_argument('--with-sim', action='store_true')
    a = ap.parse_args()
    out = os.path.join(ROOT, a.out)
    os.makedirs(out, exist_ok=True)
    eps = [e for e in entry_points(a.with_sim) if not a.only or e[0] in a.only]
    if not a.merge_only:
        with concurrent.futures.ThreadPoolExecutor(a.jobs) as ex:
            futs = [ex.submit(run_one, n, x, out, a.precision, a.timeout) for n, x in eps]
            for f in concurrent.futures.as_completed(futs):
                n, rc = f.result()
                print(f'eva {n}: exit {rc}', flush=True)
    names = [n for n, _ in entry_points(True)]
    counts = merge(out, names)
    print('assertion sites: ' + ', '.join(f'{k} {v}' for k, v in sorted(counts.items())))
    return 0


if __name__ == '__main__':
    sys.exit(main())
