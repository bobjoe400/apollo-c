"""Progress meter for one running EVA entry (tools/prove/runs/<entry>.log).

A fixpoint analysis has no fixed amount of work, so there is no exact
ETA.  What the log does give:

  coverage   distinct functions EVA has finished at least once, against
             the functions reachable from the entry's main in the static
             call graph (build/overview/callgraph.tsv).  While this is
             below 100 % the run is still discovering code and the rate
             gives a lower bound on the time to first full coverage.
  position   the call stack of the function EVA is computing right now
             (the last "computing for function A <- B <- main" line) and
             how many times each function directly under main has been
             (re)computed: a second computation of the same top-level
             function means the outer loop is iterating.
  loops      how many loop heads have started widening ("starting to
             merge loop iterations") and how often the hottest ones were
             re-entered; widening converges in a bounded number of steps
             once the state stops changing, so a loop head seen N times
             with no new functions discovered is in its last few rounds.
  rate       log growth and elapsed time (from tools/prove/progress.log),
             memory of the frama-c process.

Usage: python3 -I tools/prove/eta.py [entry] [--tail MB]   (default: sim)
"""
import collections
import os
import re
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def sh(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True).stdout.strip()


def reachable_from_main(entry):
    """Closure of the static call graph from the entry's main (callgraph.tsv
    lists callee -> callers); for test entries the test file's own
    functions are not in the graph, so take every src function a test
    may call (public ones) as the upper bound."""
    path = os.path.join(ROOT, 'build/overview/callgraph.tsv')
    if not os.path.exists(path):
        return None
    callers_of = {}
    public = set()
    with open(path) as fh:
        head = fh.readline().rstrip('\n').split('\t')
        for line in fh:
            row = dict(zip(head, line.rstrip('\n').split('\t')))
            callers_of[row['callee']] = [c for c in row['callers'].split(',') if c and c != '-']
            if row.get('public') == 'yes':
                public.add(row['callee'])
    callees_of = collections.defaultdict(set)
    for callee, callers in callers_of.items():
        for c in callers:
            callees_of[c].add(callee)
    roots = {'main'} if entry == 'sim' and 'main' in callers_of else public
    seen, todo = set(roots), list(roots)
    while todo:
        f = todo.pop()
        for g in callees_of.get(f, ()):
            if g not in seen:
                seen.add(g)
                todo.append(g)
    return seen


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    entry = args[0] if args else 'sim'
    tail_mb = 64
    if '--tail' in sys.argv:
        tail_mb = int(sys.argv[sys.argv.index('--tail') + 1])
    log = os.path.join(ROOT, 'tools/prove/runs', entry + '.log')
    if not os.path.exists(log):
        sys.exit(f'no log: {log}')
    size = os.path.getsize(log)

    # elapsed + rate
    prog = os.path.join(ROOT, 'tools/prove/progress.log')
    started = None
    if os.path.exists(prog):
        for line in open(prog):
            m = re.match(r'(\d\d:\d\d:\d\d) start\s+' + re.escape(entry) + r'\b', line)
            if m:
                started = m.group(1)
    mtime = os.path.getmtime(log)
    t0 = time.time()
    time.sleep(5)
    rate = (os.path.getsize(log) - size) / (time.time() - t0)

    # whole-log counts (grep is far faster than Python on a multi-GB log)
    done_total = int(sh(f"grep -c 'Done for function' '{log}'") or 0)
    distinct = set(sh(f"grep 'Done for function' '{log}' | awk '{{print $NF}}' | sort -u").split())
    merges = sh(f"grep -o '^\\[eva\\] [^:]*:[0-9]*: starting to merge loop iterations' '{log}' | sort | uniq -c | sort -rn | head -5")
    top = sh(f"grep -o 'computing for function [A-Za-z0-9_]* <- main\\.' '{log}' | sort | uniq -c | sort -rn | head -8")

    # current position from the tail
    with open(log, 'rb') as fh:
        fh.seek(max(0, size - tail_mb * 1024 * 1024))
        tail = fh.read().decode('utf-8', 'replace')
    stacks = re.findall(r'computing for function ([A-Za-z0-9_]+(?: <- [A-Za-z0-9_]+)*)', tail)
    current = stacks[-1] if stacks else '?'

    reach = reachable_from_main(entry)
    print(f'entry {entry}: log {size / 1e9:.2f} GB, growing {rate / 1e6:.1f} MB/s, last write {time.time() - mtime:.0f} s ago'
          + (f', started {started}' if started else ''))
    rss = sh("ps -o rss= -C frama-c | sort -n | tail -1")
    if rss:
        print(f'frama-c RSS {int(rss) / 1e6:.1f} GB')
    if reach is not None:
        covered = len(distinct & reach)
        print(f'coverage: {covered}/{len(reach)} reachable functions finished at least once ({100 * covered / max(1, len(reach)):.0f} %), {len(distinct)} distinct in all, {done_total} completions')
        if covered < len(reach) and started:
            print('  (still discovering code: a lower bound on the remaining time is the discovery rate so far)')
    else:
        print(f'coverage: {len(distinct)} distinct functions finished, {done_total} completions (no callgraph.tsv for the reachable set)')
    print('position: ' + current.replace(' <- ', ' <- '))
    print('top-level functions under main (times computed):')
    for line in top.splitlines():
        print('  ' + line.strip())
    print('loop heads that started widening (times):')
    for line in merges.splitlines():
        print('  ' + line.strip())
    print('reading: once coverage stops rising and the same loop heads keep re-merging, the run is in its widening rounds; '
          'EVA widens a counter to its full range after a few rounds, so a loop head should not merge many more than ~10 times.')


if __name__ == '__main__':
    main()
