#!/usr/bin/env python3
"""eva_run.py -- Frama-C EVA over libagc, one analysis per entry point.

Usage (from the repo root; `make prove` runs it detached):
    python3 -I tools/prove/eva_run.py [--jobs 2] [--only NAME...]
        [--out build/prove] [--precision 1] [--timeout 3600] [--merge-only]

Every AGC_ASSERT(e) is replaced by `agc_eva_check((e) ? 1 : 0)`, whose ACSL
precondition `c != 0` EVA checks at each call site
(tools/prove/eva_assert.h, force-included; the real assert.h is skipped by
its include guard).  The entry points are the test programs (tests/**/*.c,
main), each analysed with the whole library, so every assertion is judged
in the calling contexts that really occur.

EVA is a one-time, end-of-project cleanup in this project, not a working
step: one pass on the finished tree; the guards it proves VALID are
deleted (tools/prove/manual.tsv, "EVA VALID 2026-10-12"), and the tree is
not re-proved after that.  The pass that was applied merged 33 completed
entry points (build/prove/eva_verdicts.tsv: VALID 100, UNKNOWN 146,
INVALID 2, DEAD 644 assertion sites).

The agc_sim program's own main (src/sim/*.c) is NOT an entry point.  It
runs unbounded (argv options, a socket server loop), so EVA never
converges on it: on 2026-10-12 it was stopped at 14.4 GB of memory and
78 % coverage on a 15 GB host, after 98 min.  The `sim_harness` entry
(tests/sim/harness.c) stands in for it; note that it is analysed with the
library only (lib_sources() leaves src/sim out), so the assertions in
src/sim itself have no EVA verdict (0 src/sim rows in eva_verdicts.tsv).

The runs analyse a snapshot of src/ include/ tests/ (<out>/tree, commit in
<out>/tree/COMMIT), so the tree may be edited while EVA runs; file:line in
the verdicts refer to the snapshot.

Each run needs ~4 GB: the default is two concurrent frama-c processes and
60 min per entry point.  Progress:
  tools/prove/runs/<entry>.log  the run's own output, line-buffered
                                (`-eva-show-progress`: `tail -f` shows the
                                function being analysed)
  tools/prove/progress.log      one line per finished run (start, end,
                                entry, result, verdict counts)
  tools/prove/summary.txt       rewritten every 30 s: runs done/total,
                                verdicts so far, ETA, the live runs with
                                their current function and elapsed time
Per entry point the Report plug-in writes <out>/<entry>.csv; the merged
per-site verdict goes to <out>/eva_verdicts.tsv (file line verdict mains):

    VALID    the condition holds on every path of every analysis
    UNKNOWN  EVA could not decide it on some path (imprecision or a real
             failure path)
    INVALID  EVA found a path on which it fails
    DEAD     no analysis reached the site

Build flags that matter for EVA: -DAGC_HOST_DEBUG (the test library),
-c11 (_Static_assert); the docs' `/*@` comment markers are not ACSL
(annot-error demoted to a warning).
"""
import argparse
import collections
import concurrent.futures
import csv
import glob
import os
import re
import shutil
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SHIM = 'tools/prove/eva_assert.h'
RANK = {'INVALID': 3, 'UNKNOWN': 2, 'VALID': 1}
RUNS = os.path.join(ROOT, 'tools', 'prove', 'runs')
# the slowest entry points first, so the short ones fill the other slot
SLOW = ('exec_exec', 'io_dsky')


def lib_sources(base):
    out = []
    for dp, _, files in os.walk(os.path.join(base, 'src')):
        rel = os.path.relpath(dp, base)
        if rel == 'src/sim' or rel.startswith('src/sim/'):
            continue
        out += [os.path.join(rel, f) for f in files if f.endswith('.c')]
    return sorted(out)


def entry_points():
    """One entry per test program; agc_sim's main is excluded (docstring)."""
    eps = []
    for t in sorted(glob.glob('tests/**/*.c', root_dir=ROOT, recursive=True)):
        eps.append((t[len('tests/'):-2].replace('/', '_'), [t]))
    eps.sort(key=lambda e: (e[0] not in SLOW, SLOW.index(e[0]) if e[0] in SLOW else 0, e[0]))
    return eps


def status_key(st):
    st = st.lower()
    if st.startswith(('dead', 'unreach')):
        return 'DEAD'
    if st.startswith('valid'):
        return 'VALID'
    if st.startswith('invalid'):
        return 'INVALID'
    return 'UNKNOWN'


def csv_rows(path):
    if not os.path.exists(path):
        return
    with open(path, encoding='utf-8', errors='replace') as fh:
        for row in csv.DictReader(fh, delimiter='\t'):
            if row['property kind'] == 'precondition of agc_eva_check':
                yield row


def csv_counts(path):
    return collections.Counter(status_key(r['status']) for r in csv_rows(path))


def snapshot(out):
    tree = os.path.join(out, 'tree')
    if os.path.exists(tree):
        shutil.rmtree(tree)
    for d in ('src', 'include', 'tests', os.path.join('tools', 'prove')):
        shutil.copytree(os.path.join(ROOT, d), os.path.join(tree, d),
                        ignore=shutil.ignore_patterns('runs', '*.log', 'summary.txt'))
    head = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, capture_output=True, text=True)
    dirty = subprocess.run(['git', 'status', '--porcelain', 'src', 'include', 'tests'], cwd=ROOT,
                           capture_output=True, text=True).stdout.strip()
    with open(os.path.join(tree, 'COMMIT'), 'w') as fh:
        fh.write(head.stdout.strip() + (' (+ uncommitted changes)' if dirty else '') + '\n')
    return tree


COMPUTING = re.compile(r'computing for function (\S+)(?: <- (\S+))?')


def current_function(log):
    """Last `computing for function X` of a live log (tail only)."""
    try:
        with open(log, 'rb') as fh:
            fh.seek(0, 2)
            n = fh.tell()
            fh.seek(max(0, n - 65536))
            tail = fh.read().decode('utf-8', 'replace')
    except OSError:
        return '?'
    m = COMPUTING.findall(tail)
    if m:
        # the assertion stub itself is entered from everywhere: name its caller
        fn, caller = m[-1]
        return (caller if fn == 'agc_eva_check' and caller else fn).rstrip('.')
    return 'parsing' if 'Parsing' in tail[-2000:] else '?'


class Progress:
    """tools/prove/progress.log and tools/prove/summary.txt."""

    def __init__(self, total, jobs):
        self.total = total
        self.jobs = jobs
        self.done = []
        self.live = {}
        self.lock = threading.Lock()
        self.t0 = time.time()
        self.log = os.path.join(ROOT, 'tools', 'prove', 'progress.log')
        self.summary = os.path.join(ROOT, 'tools', 'prove', 'summary.txt')
        with open(self.log, 'a') as fh:
            fh.write(f"# {time.strftime('%Y-%m-%d %H:%M:%S')} make prove: {total} entry points, "
                     f"{jobs} at a time\n")
        self.stop = threading.Event()
        threading.Thread(target=self._tick, daemon=True).start()
        self.write_summary()

    def _tick(self):
        while not self.stop.wait(30):
            self.write_summary()

    def start(self, name, log):
        with self.lock:
            self.live[name] = (time.time(), log)
        self.write_summary()

    def record(self, name, start, end, result, counts):
        with self.lock:
            self.live.pop(name, None)
            self.done.append((name, end - start, result, counts))
            cs = ' '.join(f'{k} {counts.get(k, 0)}' for k in ('VALID', 'UNKNOWN', 'INVALID', 'DEAD'))
            with open(self.log, 'a') as fh:
                fh.write(f"{time.strftime('%H:%M:%S', time.localtime(start))} -> "
                         f"{time.strftime('%H:%M:%S', time.localtime(end))}  {name:26s} "
                         f"{result:8s} {cs}\n")
        self.write_summary()

    def write_summary(self):
        with self.lock:
            done = list(self.done)
            live = dict(self.live)
        n = len(done)
        tot = collections.Counter()
        for d in done:
            tot.update(d[3])
        mean = sum(d[1] for d in done) / n if n else 0
        eta = mean * (self.total - n) / self.jobs if n else 0
        now = time.time()
        lines = [f"make prove (Frama-C EVA), started "
                 f"{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.t0))}; "
                 f"updated {time.strftime('%H:%M:%S')}",
                 f'runs done: {n}/{self.total}  (timeouts: {sum(1 for d in done if d[2] == "TIMEOUT")})',
                 f'elapsed: {(now - self.t0) / 60:.1f} min; mean run {mean / 60:.1f} min; '
                 f'ETA {eta / 60:.0f} min ({self.jobs} runs at a time)'
                 + ('' if n else ' (no run finished yet)'),
                 'assertion verdicts so far (per run, summed): ' + ', '.join(
                     f'{k} {tot.get(k, 0)}' for k in ('VALID', 'UNKNOWN', 'INVALID', 'DEAD')),
                 'running:']
        for name, (t, log) in sorted(live.items()):
            lines.append(f'  {name:26s} {(now - t) / 60:6.1f} min  in {current_function(log)}'
                         f'  (tail -f {os.path.relpath(log, ROOT)})')
        if self.total == n:
            lines.append('  (none: finished)')
        with open(self.summary, 'w') as fh:
            fh.write('\n'.join(lines) + '\n')


def run_one(name, extra, out, a, prog, tree):
    csv_path = os.path.join(out, name + '.csv')
    log_path = os.path.join(RUNS, name + '.log')
    cmd = ['stdbuf', '-oL', '-eL', 'frama-c', '-c11', '-kernel-warn-key', 'annot-error=active',
           '-cpp-extra-args=-Iinclude -include ' + SHIM + ' -DAGC_HOST_DEBUG',
           *lib_sources(tree), *extra,
           '-main', 'main', '-eva', '-eva-precision', str(a.precision), '-eva-show-progress',
           '-eva-no-alloc-returns-null',
           '-then', '-report-csv', csv_path]
    if os.path.exists(csv_path):
        os.remove(csv_path)
    start = time.time()
    prog.start(name, log_path)
    with open(log_path, 'w', buffering=1) as log:
        try:
            r = subprocess.run(cmd, cwd=tree, stdout=log, stderr=subprocess.STDOUT, timeout=a.timeout)
            result = 'OK' if r.returncode == 0 else f'EXIT{r.returncode}'
        except subprocess.TimeoutExpired:
            result = 'TIMEOUT'
    prog.record(name, start, time.time(), result, csv_counts(csv_path))
    return name, result


def merge(out, names):
    site = collections.defaultdict(lambda: {'v': None, 'mains': set()})
    for name in names:
        for row in csv_rows(os.path.join(out, name + '.csv')):
            key = (os.path.join(row['directory'], row['file']), int(row['line']))
            v = status_key(row['status'])
            rec = site[key]
            if v == 'DEAD':
                continue
            rec['mains'].add(name)
            if rec['v'] is None or RANK[v] > RANK[rec['v']]:
                rec['v'] = v
    with open(os.path.join(out, 'eva_verdicts.tsv'), 'w') as fh:
        fh.write('file\tline\tverdict\tmains\n')
        for (f, ln) in sorted(site):
            rec = site[(f, ln)]
            fh.write(f"{f}\t{ln}\t{rec['v'] or 'DEAD'}\t{','.join(sorted(rec['mains'])) or '-'}\n")
    return collections.Counter(r['v'] or 'DEAD' for r in site.values())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--jobs', type=int, default=2)
    ap.add_argument('--only', nargs='*')
    ap.add_argument('--out', default='build/prove')
    ap.add_argument('--precision', type=int, default=1)
    ap.add_argument('--timeout', type=int, default=3600)
    ap.add_argument('--merge-only', action='store_true')
    a = ap.parse_args()
    out = os.path.join(ROOT, a.out)
    os.makedirs(out, exist_ok=True)
    os.makedirs(RUNS, exist_ok=True)
    eps = [e for e in entry_points() if not a.only or e[0] in a.only]
    if not a.merge_only:
        tree = snapshot(out)
        prog = Progress(len(eps), a.jobs)
        with concurrent.futures.ThreadPoolExecutor(a.jobs) as ex:
            futs = [ex.submit(run_one, n, x, out, a, prog, tree) for n, x in eps]
            for f in concurrent.futures.as_completed(futs):
                n, rc = f.result()
                print(f'eva {n}: {rc}', flush=True)
        prog.stop.set()
        prog.write_summary()
    counts = merge(out, [n for n, _ in entry_points()])
    print('assertion sites: ' + ', '.join(f'{k} {v}' for k, v in sorted(counts.items())))
    return 0


if __name__ == '__main__':
    sys.exit(main())
