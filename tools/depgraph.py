#!/usr/bin/env python3
"""depgraph.py -- module dependency map of src/, rendered as Mermaid.

Usage (from the repo root):
    python3 -I tools/depgraph.py [--work DIR] [--readme FILE] [--stdout]

Method (no reading or grepping of calls; CODING_STANDARD.md "Tooling"):
  * every src/**/*.c is compiled with GCC -O0 -fno-inline
    -fcallgraph-info=su,da into --work (default build/depgraph); each TU's
    VCG file gives the defined functions (a node with a stack-usage line)
    and one edge per call site;
  * a callee is resolved to the file that DEFINES it (static functions are
    titled "file:name" by GCC, so same-named statics stay apart);
  * #include "agc/<subsystem>/<h>.h" lines give the header dependencies.

A module is a src/ folder two levels deep (src/nav/conic, src/csm/dap; a
file directly under src/<x>/ belongs to module <x>).  src/exec/dispatch.c
(the generated job/task switch) and src/exec/interrupts.c (the interrupt
lead-ins) are the documented layering exceptions and are kept as their own
nodes so their fan-out does not hide the rest.

Output: the Markdown between the markers
    <!-- depgraph:begin -->  ...  <!-- depgraph:end -->
in --readme (default docs/overview/README.md), or stdout with --stdout:
  1. a layer diagram (top-level subsystems, call counts on the edges);
  2. a module diagram grouped by subsystem.  Leaf fan-out is collapsed:
     edges into a hub module that most modules call (src/core, src/exec)
     are not drawn but counted in a table, and edges with fewer than
     MIN_DRAWN call pairs are left to the table;
  3. the full edge table (caller module, callee module, distinct
     caller->callee function pairs, call sites);
  4. upward calls against the ARCHITECTURE.md layering.
"""
import argparse
import collections
import glob
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NODE_RE = re.compile(r'^node: \{ title: "([^"]+)" label: "([^"]*)"(.*)\}$')
EDGE_RE = re.compile(r'^edge: \{ sourcename: "([^"]+)" targetname: "([^"]+)" label: "([^"]*)" \}$')
INCLUDE_RE = re.compile(r'^\s*#\s*include\s+"agc/([a-z_]+)/[^"]+"')
BEGIN = '<!-- depgraph:begin -->'
END = '<!-- depgraph:end -->'
SPECIAL = {'src/exec/dispatch.c': 'exec/dispatch', 'src/exec/interrupts.c': 'exec/interrupts'}
# ARCHITECTURE.md: core < exec < io/imu/nav < guidance < programs < csm|lm < sim
LAYER = {'core': 0, 'exec': 1, 'io': 2, 'imu': 2, 'nav': 2, 'guidance': 3, 'programs': 4,
         'csm': 5, 'lm': 5, 'sim': 6}
TOP_ORDER = ['sim', 'csm', 'lm', 'programs', 'guidance', 'io', 'imu', 'nav', 'exec', 'core']
HUB_SHARE = 0.7   # a callee module called by >= 70 % of the modules is a hub
MIN_DRAWN = 5     # module edges with fewer distinct function pairs are table-only
GCC_FLAGS = ['-std=c11', '-O0', '-fno-inline', '-Iinclude', '-fcallgraph-info=su,da', '-c']


def module_of(path):
    """src/a/b/c.c -> 'a/b'; src/a/c.c -> 'a'; the two exec exceptions apart."""
    if path in SPECIAL:
        return SPECIAL[path]
    parts = path.split('/')
    if len(parts) <= 3:
        return parts[1]
    return parts[1] + '/' + parts[2]


def top_of(module):
    return module.split('/')[0]


def compile_all(srcs, work):
    """One gcc per TU, eight at a time; returns the .ci paths."""
    os.makedirs(work, exist_ok=True)
    out, running = [], []
    for src in srcs:
        obj = os.path.join(work, src.replace('/', '_') + '.o')
        out.append(obj[:-2] + '.ci')
        running.append(subprocess.Popen(['gcc'] + GCC_FLAGS + ['-o', obj, src], cwd=ROOT))
        if len(running) >= 8:
            if running.pop(0).wait() != 0:
                sys.exit('depgraph: compile failed')
    for p in running:
        if p.wait() != 0:
            sys.exit('depgraph: compile failed')
    return out


def parse_ci(paths):
    """(defined: title -> file, edges: [(caller title, callee title, site)])."""
    defined, edges = {}, []
    for path in paths:
        with open(path, encoding='utf-8') as fh:
            for line in fh:
                m = NODE_RE.match(line.rstrip('\n'))
                if m:
                    label = m.group(2).split('\\n')
                    if len(label) > 2 and 'bytes' in label[2]:
                        defined[m.group(1)] = label[1].split(':')[0]
                    continue
                m = EDGE_RE.match(line.rstrip('\n'))
                if m:
                    edges.append(m.groups())
    return defined, edges


def includes(srcs):
    """(module, header subsystem) -> number of files including it."""
    inc = collections.Counter()
    for src in srcs:
        seen = set()
        with open(os.path.join(ROOT, src), encoding='utf-8') as fh:
            for line in fh:
                m = INCLUDE_RE.match(line)
                if m:
                    seen.add(m.group(1))
        for sub in seen:
            inc[(module_of(src), sub)] += 1
    return inc


def module_edges(defined, edges):
    """(caller module, callee module) -> [set of function pairs, call sites]."""
    out = collections.defaultdict(lambda: [set(), 0])
    for src, dst, _site in edges:
        if src not in defined or dst not in defined:
            continue
        a, b = module_of(defined[src]), module_of(defined[dst])
        if a == b:
            continue
        cell = out[(a, b)]
        cell[0].add((src, dst))
        cell[1] += 1
    return out


def node_id(module):
    return module.replace('/', '_')


def short(title):
    return title.split(':')[-1]


def mermaid_layers(medges, modules):
    tops = [t for t in TOP_ORDER if any(top_of(m) == t for m in modules)]
    agg = collections.Counter()
    for (a, b), (pairs, _sites) in medges.items():
        ta, tb = top_of(a), top_of(b)
        if ta != tb and a not in SPECIAL.values():
            agg[(ta, tb)] += len(pairs)
    lines = ['```mermaid', 'flowchart TB']
    for t in tops:
        lines.append(f'    {t}["{t}/ (layer {LAYER[t]})"]')
    for (a, b), n in sorted(agg.items(), key=lambda kv: (TOP_ORDER.index(kv[0][0]), TOP_ORDER.index(kv[0][1]))):
        arrow = '-.->' if LAYER[a] < LAYER[b] else '-->'
        lines.append(f'    {a} {arrow}|{n}| {b}')
    lines.append('```')
    return lines, agg


def hubs_of(medges, modules):
    callers = collections.defaultdict(set)
    for (a, b) in medges:
        callers[b].add(a)
    limit = HUB_SHARE * (len(modules) - 1)
    return sorted(b for b, s in callers.items() if len(s) >= limit), callers


def mermaid_modules(medges, callers, hubs, nfiles, nfuncs):
    """One diagram: the drawn edges whose caller's top folder is in callers;
    only the modules on those edges appear, grouped by top folder."""
    drawn = []
    for (a, b), (pairs, _sites) in sorted(medges.items()):
        if top_of(a) not in callers or b in hubs or a in SPECIAL.values() or len(pairs) < MIN_DRAWN:
            continue
        drawn.append((a, b, len(pairs)))
    used = {a for a, _b, _n in drawn} | {b for _a, b, _n in drawn}
    lines = ['```mermaid', 'flowchart LR']
    for t in TOP_ORDER:
        mods = sorted(m for m in used if top_of(m) == t)
        if not mods:
            continue
        lines.append(f'    subgraph {t}_layer["{t}/"]')
        for m in mods:
            lines.append(f'        {node_id(m)}["{m}<br/>{nfiles[m]} files, {nfuncs[m]} fns"]')
        lines.append('    end')
    for a, b, n in drawn:
        arrow = '-.->' if LAYER[top_of(a)] < LAYER[top_of(b)] else '-->'
        lines.append(f'    {node_id(a)} {arrow}|{n}| {node_id(b)}')
    lines.append('```')
    return lines


MODULE_VIEWS = (('Command module (src/csm) calls', ('csm',)),
                ('Lunar module (src/lm) calls', ('lm',)),
                ('Shared layers (programs, guidance, io, imu, nav, exec, sim) calls',
                 ('programs', 'guidance', 'io', 'imu', 'nav', 'exec', 'sim')))


def render(srcs, defined, edges, inc):
    medges = module_edges(defined, edges)
    modules = sorted({module_of(s) for s in srcs})
    nfiles = collections.Counter(module_of(s) for s in srcs)
    nfuncs = collections.Counter(module_of(f) for f in defined.values())
    hubs, callers = hubs_of(medges, modules)
    out = [BEGIN, '',
           '_Generated by `python3 -I tools/depgraph.py` from GCC `-fcallgraph-info` call',
           'edges and `#include` lines of src/; do not edit by hand._', '',
           f'{len(srcs)} translation units, {len(defined)} defined functions, '
           f'{len(modules)} modules.', '',
           '### Layers', '',
           'One node per top-level folder; edge label = distinct caller->callee',
           'function pairs. Dotted arrows call UP the ARCHITECTURE.md layering',
           '(core < exec < io/imu/nav < guidance < programs < csm|lm < sim);',
           'the reasons are in the "Upward calls" table below. The fan-out of',
           'the job/task switch and the interrupt lead-ins is left out here',
           '(see "Dispatch fan-out").', '']
    layer_lines, _agg = mermaid_layers(medges, modules)
    out += layer_lines
    out += ['', '### Modules', '',
            'One node per folder two levels deep, in three views by caller',
            '(CSM, LM, shared layers); node label = files and defined functions,',
            'edge label = distinct caller->callee function pairs. Not drawn, to',
            'keep the maps',
            'readable (all are in the edge table): edges into the hub modules',
            f'{", ".join("`src/" + h + "`" for h in hubs)} (called by at least',
            f'{int(HUB_SHARE * 100)} % of the modules), the fan-out of `exec/dispatch`',
            '(the generated job/task switch) and `exec/interrupts` (interrupt',
            f'lead-ins), and edges with fewer than {MIN_DRAWN} function pairs.', '']
    for title, tops in MODULE_VIEWS:
        out += ['', f'#### {title}', '']
        out += mermaid_modules(medges, tops, hubs, nfiles, nfuncs)
    out += ['', '#### Hub modules (edges not drawn)', '',
            '| Hub | Called from (modules) | Function pairs |', '|---|---|---|']
    for h in hubs:
        n = sum(len(medges[(a, h)][0]) for a in callers[h])
        out.append(f'| src/{h} | {len(callers[h])} | {n} |')
    out += ['', '#### Dispatch fan-out (edges not drawn)', '',
            '| From | Into module | Job/task/rupt handlers |', '|---|---|---|']
    for (a, b), (pairs, _sites) in sorted(medges.items()):
        if a in SPECIAL.values():
            out.append(f'| {a} | src/{b} | {len(pairs)} |')
    out += ['', '#### All module edges', '',
            '| Caller module | Callee module | Function pairs | Call sites |', '|---|---|---|---|']
    for (a, b), (pairs, sites) in sorted(medges.items()):
        out.append(f'| {a} | {b} | {len(pairs)} | {sites} |')
    out += ['', '#### Upward calls (against the layering)', '',
            'Calls from a lower layer into a higher one, outside the two exec',
            'exceptions. Example pairs are the first three in name order.', '',
            '| Caller module | Callee module | Pairs | Examples |', '|---|---|---|---|']
    for (a, b), (pairs, _sites) in sorted(medges.items()):
        if a in SPECIAL.values() or LAYER[top_of(a)] >= LAYER[top_of(b)]:
            continue
        ex = ', '.join(f'`{short(s)}` -> `{short(d)}`' for s, d in sorted(pairs)[:3])
        out.append(f'| {a} | {b} | {len(pairs)} | {ex} |')
    out += ['', '#### Header dependencies', '',
            'Files of each module that include a public header of another',
            'subsystem (include/agc/<subsystem>/), where the module makes no call',
            'into that subsystem (types, constants and ids only).', '',
            '| Module | Header subsystem | Files |', '|---|---|---|']
    called_tops = {(a, top_of(b)) for (a, b) in medges}
    for (m, sub), n in sorted(inc.items()):
        if sub != top_of(m) and (m, sub) not in called_tops:
            out.append(f'| {m} | {sub} | {n} |')
    out += ['', END]
    return out


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--work', default=os.path.join(ROOT, 'build', 'depgraph'))
    ap.add_argument('--readme', default=os.path.join(ROOT, 'docs', 'overview', 'README.md'))
    ap.add_argument('--stdout', action='store_true')
    args = ap.parse_args(argv[1:])
    srcs = sorted(os.path.relpath(p, ROOT) for p in glob.glob(os.path.join(ROOT, 'src', '**', '*.c'), recursive=True))
    defined, edges = parse_ci(compile_all(srcs, os.path.abspath(args.work)))
    text = '\n'.join(render(srcs, defined, edges, includes(srcs))) + '\n'
    if args.stdout:
        sys.stdout.write(text)
        return 0
    with open(args.readme, encoding='utf-8') as fh:
        doc = fh.read()
    if BEGIN not in doc or END not in doc:
        sys.exit(f'depgraph: {args.readme} has no {BEGIN} ... {END} markers')
    head, rest = doc.split(BEGIN, 1)
    tail = rest.split(END, 1)[1]
    with open(args.readme, 'w', encoding='utf-8') as fh:
        fh.write(head + text.rstrip('\n') + tail)
    print(f'depgraph: {len(srcs)} TUs, {len(defined)} functions -> {args.readme}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
