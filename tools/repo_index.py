#!/usr/bin/env python3
"""repo_index.py -- docs/overview/INDEX.md: one row per file of the repository.

Usage (repo root):  python3 -I tools/repo_index.py > docs/overview/INDEX.md

For every tracked (or new, not ignored) file: its purpose, taken from the
file's own header (prolog PURPOSE, module docstring, first comment, Markdown
title; a few binaries and data files have a fixed description below); the
owning lane (docs/MODULE_STATUS.md, by path); which files mention its path
(counted by kind: Makefile, tools, docs, lane notes, src, include, tests,
sim); the last commit that touched it (a staged rename: its old path); its
size; and a verdict (live, generated, history, archived, or the move that
tools/layout/moves.tsv still proposes).  One table per top-level area.
"""
import collections
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
files = subprocess.run(['git', 'ls-files', '--cached', '--others', '--exclude-standard'], capture_output=True, text=True).stdout.split('\n')
files = [f for f in files if f and os.path.isfile(f)]
# last commit date per file (one git log pass)
last = {}
log = subprocess.run(['git', 'log', '--format=@%cs', '--name-only'], capture_output=True, text=True).stdout
d = None
for line in log.split('\n'):
    if line.startswith('@'):
        d = line[1:]
    elif line and line not in last:
        last[line] = d
TEXT_EXT = {'.c','.h','.py','.md','.txt','.tsv','.sh','.lua','.cocci','.tcl','.json','.spec','.patch','.agc','.dot','.cfg','.ini',''}
texts = {}
for f in files:
    ext = os.path.splitext(f)[1]
    if ext in TEXT_EXT and os.path.getsize(f) < 3_000_000 and not f.endswith('.symtab'):
        try:
            texts[f] = open(f, encoding='utf-8', errors='replace').read()
        except Exception:
            pass
base_count = collections.Counter(os.path.basename(f) for f in files)
GENERIC = {'internal.h','state.c','README.md','common.c','clock.c','dap.c','dap.h','navigation.c','program.c','Makefile','display.c'}
def keys(f):
    k = {f}
    top, _, rest = f.partition('/')
    if top == 'include' and rest.startswith('agc/'):
        k.add(rest)
    if top in ('src', 'tests', 'tools', 'docs', 'sim') and '/' in rest:
        k.add(rest)
    b = os.path.basename(f)
    if base_count[b] == 1 and b not in GENERIC and len(b) >= 7:
        k.add(b)
    stem = os.path.splitext(b)[0]
    return k
def cat(g):
    if g == 'Makefile': return 'Makefile'
    if g.startswith('docs/agents/') or g in ('docs/WORKLOG.md','docs/RENAMES.md','docs/RENAMES_inventory.txt') or g.startswith('docs/modules/'):
        return 'notes'
    if g.startswith('docs/'): return 'docs'
    if g.startswith('tools/'): return 'tools'
    if g.startswith('src/'): return 'src'
    if g.startswith('include/'): return 'include'
    if g.startswith('tests/'): return 'tests'
    if g.startswith('sim/'): return 'sim'
    return 'other'
def purpose(f):
    t = texts.get(f)
    ext = os.path.splitext(f)[1]
    if f.endswith('.png'): return 'screenshot'
    if t is None: return ''
    if ext in ('.c', '.h'):
        m = re.search(r'PURPOSE:\s*\n((?:\s*\*[^\n]*\n){1,4})', t)
        if m:
            s = ' '.join(l.strip().lstrip('*').strip() for l in m.group(1).split('\n'))
            return s.strip()
        m = re.search(r'/\*+\s*\n?\s*\**\s*([^\n*][^\n]*)', t)
        return m.group(1).strip() if m else ''
    if ext == '.py':
        m = re.search(r'"""(.*?)(\n\n|""")', t, re.S)
        if m: return ' '.join(m.group(1).split())
        m = re.search(r'^#(?!!)\s*(.+)$', t, re.M)
        return m.group(1) if m else ''
    if ext == '.md':
        lines = [l for l in t.split('\n') if l.strip()]
        h = lines[0].lstrip('# ').strip() if lines else ''
        return h
    if ext in ('.sh','.cocci','.lua','.tcl','.tsv','.txt','.spec','.patch',''):
        for l in t.split('\n')[:12]:
            s = l.strip()
            if s.startswith('#!'): continue
            for p in ('//', '--', '#'):
                if s.startswith(p):
                    s = s[len(p):].strip()
                    if s: return s
            if s and ext in ('.patch','.txt'): return s
    return ''
rows = []
for f in files:
    ks = keys(f)
    refs = collections.Counter()
    who = []
    for g, t in texts.items():
        if g == f: continue
        if any(k in t for k in ks):
            refs[cat(g)] += 1
            who.append(g)
    size = os.path.getsize(f)
    rows.append((f, str(size), last.get(f, '?'), purpose(f)[:220].replace('\t',' '),
                 ', '.join('%s(%d)' % (c, n) for c, n in sorted(refs.items())),
                 ' '.join(sorted(who)[:6])))

prev = {}      # new path -> old path (renames staged, not yet committed)
for l in subprocess.run(['git', 'diff', '--cached', '-M', '--name-status'], capture_output=True, text=True).stdout.split('\n'):
    p = l.split('\t')
    if len(p) == 3 and p[0].startswith('R'):
        prev[p[2]] = p[1]
proposed = {}  # old -> new (tools/layout/moves.tsv, rows not yet applied)
if os.path.exists('tools/layout/moves.tsv'):
    for l in open('tools/layout/moves.tsv'):
        if l.startswith('#') or not l.strip():
            continue
        o, n = l.rstrip('\n').split('\t')[:2]
        if os.path.exists(o) and not os.path.exists(n):
            proposed[o] = n
date = {r[0]: r[2] for r in rows}
MANUAL = {
 '.gitignore': 'build outputs, oracle binaries, prove run logs, compile_commands.json',
 'uncrustify.cfg': 'Uncrustify 0.72 configuration used by tools/format.sh (docs/STYLE.md)',
 '.vscode/c_cpp_properties.json': 'VS Code C/C++ include path (include/) and C11 mode',
 '.vscode/settings.json': 'VS Code editor settings (formatter off, compile_commands.json)',
 'docs/overview/data/callgraph.tsv': 'generated by `make callgraph`: one row per src/ function (callers, sites, label, dispatch)',
 'docs/overview/data/callgraph.dot': 'generated by `make callgraph`: Graphviz call graph',
 'docs/overview/data/collapse_candidates.tsv': 'generated by `make callgraph`: single-caller helpers (collapse wave input)',
 'docs/overview/data/guards.tsv': 'generated by `make prove-classify`: every AGC_ASSERT guard with its EVA/AST class',
 'docs/overview/data/long_statements.tsv': 'statements wider than 120 columns (format2 review list)',
 'docs/overview/data/one_liners.tsv': 'generated by tools/one_liners.py: one-line functions classified (tree kept at PLAIN 0)',
 'docs/reference/NASA-SEL-94-003-C-Style-Guide.pdf': 'NASA SEL-94-003 C Style Guide (source of docs/STYLE.md)',
 'tools/oracle/bin/yaAGC': 'yaAGC binary built from SRC/virtualagc (tools/oracle/BUILD.md)',
 'tools/oracle/bin/yaYUL': 'yaYUL assembler binary (tools/oracle/BUILD.md)',
 'tools/oracle/ropes/Comanche055.bin': 'assembled CM rope (oracle, agc_sim, lm-sim)',
 'tools/oracle/ropes/Luminary099.bin': 'assembled LM rope (oracle, agc_sim, lm-sim)',
 'tools/oracle/ropes/Comanche055.symtab': 'yaYUL symbol table of the CM rope (oracle `whatis`)',
 'tools/oracle/ropes/Luminary099.symtab': 'yaYUL symbol table of the LM rope (oracle `whatis`)',
 'tools/oracle/yaAGC-src/COPYING': 'GPL licence of the vendored yaAGC engine',
}
MANUAL['sim/nassp/patches/0001-apollo-c-agc-backend.patch'] = "patch to NASSP's CMake: build the CMC/LGC on apollo-c (-DAPOLLO_C_DIR) instead of yaAGC"
MANUAL['sim/nassp/patches/0002-lem-lua-test-interface.patch'] = "patch to NASSP: the LEM gets the Saturn's Lua test methods (stock reference and ours)"
MANUAL['sim/nassp/patches/0003-saturn-console-mode.patch'] = "patch to NASSP: Saturn console-mode (headless) crash fix"
for f in ['insertion', 'liftoff_lm', 'mcc2', 'pdi_lm', 'tli']:
    MANUAL['sim/nassp/images/%s.agc' % f] = 'AGC erasable image of the NASSP Apollo 11 scenario (%s), input of `make nassp-replay`' % f
for f in ['NullAPI', 'agc_engine', 'agc_engine_init', 'agc_symtab', 'agc_utilities', 'rfopen']:
    MANUAL['tools/oracle/build/%s.o' % f] = 'object file of the vendored yaAGC engine'
CORE_NUM = ('word', 'divide', 'shift', 'mpac', 'sp_trig', 'vec_div', 'constants', 'interp')
def lane(p):
    base = os.path.basename(p)
    s = p
    for top in ('src/', 'include/agc/', 'tests/'):
        if p.startswith(top):
            s = p[len(top):]
    if p.startswith('tests/oracle/'): return 'oracle-cases'
    if p.startswith(('src/nassp', 'src/erasable_map', 'include/agc/erasable_map', 'tests/erasable_map', 'sim/nassp', 'tools/nassp')) or 'nassp' in base: return 'nassp-integration'
    if p.startswith('sim/lm_simulator') or p.startswith('docs/modules/lm-simulator'): return 'lm-simulator'
    if p.startswith(('sim/scenarios', 'sim/sim_client')): return 'sim'
    if p.startswith('docs/modules/'):
        return re.sub(r'\.md$', '', p.split('/')[2])
    if p.startswith('docs/history/waves/'): return re.sub(r'\.md$', '', p.split('/')[3])
    if p.startswith(('docs/', '.vscode', 'README', 'Makefile', '.gitignore')): return 'lead'
    if p.startswith('tools/oracle'): return 'oracle (cases: oracle-cases)'
    if p.startswith('tools/prove'): return 'status-handling-2, finish'
    if p.startswith('tools/'):
        T = {'check_rules': 'lead, status-handling', 'check_style': 'style', 'format.sh': 'format2', 'fix_comments': 'format2',
             'gen_registry': 'integrate', 'registry.tsv': 'integrate (rows: every lane)', 'callgraph': 'collapse', 'cgast': 'collapse',
             'inline_one': 'collapse', 'unwrap': 'integrate', 'one_liners': 'status-handling-2', 'returns': 'status-handling-2',
             'always_ok': 'always-ok', 'drop_final_return': 'always-ok', 'prolog_check': 'prolog-consistency', 'unpaste_prologs': 'always-ok',
             'depgraph': 'layering', 'xref': 'document', 'agc_label': 'lead', 'compile_commands': 'lead', 'cocci/always_ok': 'always-ok',
             'archive/': 'naming, status-handling, finish (finished)', 'apply_layout': 'organize', 'layout/': 'organize', 'repo_index': 'organize'}
        for k, v in T.items():
            if k in p: return v
        return 'lead'
    if p == 'uncrustify.cfg': return 'style2, format2'
    if s.startswith('core/'):
        return 'core-numeric' if any(base.startswith(c) for c in CORE_NUM) or '/interp' in s or base in ('numeric.c',) else 'state-flags'
    R = [('exec', 'exec-sched'), ('io/dsky/nouns', 'io-nouns'), ('io/nouns', 'io-nouns'), ('io/dsky', 'io-pinball'), ('io/service', 'sys-misc'),
         ('io/telemetry', 'sys-misc'), ('io/verbs', 'sys-misc'), ('io/sysmisc', 'sys-misc'), ('io/nouns.c', 'io-nouns'), ('io/', 'io-pinball'),
         ('imu', 'imu'), ('csm/prelaunch', 'imu'), ('nav/conic', 'nav-conic'), ('nav/integration', 'nav-integ'), ('nav/auxiliary', 'nav-aux'),
         ('nav/navaux', 'nav-aux'), ('nav/', 'nav-conic, nav-integ, nav-aux'), ('guidance', 'guid-powered'), ('csm/burns/return_to_earth', 'p37'), ('csm/return_to_earth', 'p37'),
         ('csm/burns/earth_orbit', 'guid-entry'), ('csm/burns', 'guid-powered'), ('lm/burns', 'guid-powered'), ('lm/landing', 'guid-landing'),
         ('lm/ascent', 'guid-landing'), ('csm/entry', 'guid-entry'), ('lm/dap', 'dap-lm'), ('csm/dap', 'dap-csm'), ('programs/rendezvous', 'prog-rendz'),
         ('programs/navigation', 'prog-rendz'), ('programs/targeting', 'prog-rendz'), ('programs/displays', 'prog-rendz'), ('csm/navigation', 'prog-rendz'),
         ('lm/navigation', 'prog-rendz'), ('lm/radar', 'prog-rendz'), ('programs/', 'prog-align'), ('csm/alignment', 'prog-align'), ('csm/optics', 'prog-align'),
         ('lm/alignment', 'prog-align'), ('lm/optics', 'prog-align'), ('lm/ags', 'prog-align'), ('lm/rendezvous_final', 'prog-align'), ('lm/systems_test', 'sys-misc'),
         ('csm/entry', 'guid-entry'), ('csm/dap', 'dap-csm'), ('lm/dap', 'dap-lm'), ('lm/landing', 'guid-landing'), ('sim', 'sim'), ('csm/', 'guid-entry'), ('lm/', 'guid-landing')]
    for k, v in R:
        if s.startswith(k): return v
    return 'lead'
HIST_AGENTS = ('docs/history/',)
def verdict(p):
    if p in proposed: return 'move -> `%s` (apply_layout.sh)' % proposed[p]
    if p.startswith('docs/history/'): return 'history'
    if p.startswith('tools/archive/'): return 'archived (finished one-off)'
    if p.startswith('tools/layout/') or p == 'tools/apply_layout.sh': return 'one-off: archives itself after it runs'
    if p.startswith('tools/oracle/build/') or p.endswith('.symtab') or p.startswith('tools/oracle/bin/'): return 'build artefact, tracked although .gitignore lists bin/ and symtab: untrack (proposal)'
    if p.startswith('docs/overview/data/'): return 'generated data'
    if p in ('include/agc/exec/jobs.h', 'src/exec/dispatch.c', 'docs/overview/xref.md'): return 'live, generated'
    if p.startswith('docs/agents/'): return 'live (process)'
    if p.startswith('docs/modules/') and p.endswith('.md'): return 'live (lane note)'
    return 'live'
def refs(r):
    s = r[4] if len(r) > 4 else ''
    return s.replace('notes(', 'lane notes(').replace('other(', 'root(') or '**none**'
def last(p):
    d = date.get(p, '?')
    if d in ('?', None) and p in prev:
        d = date.get(prev[p]) or '?'
        if d == '?':
            for o in rows:
                pass
    return d
area = collections.OrderedDict()
for r in rows:
    p = r[0]
    top = p.split('/')[0] if '/' in p else '(root)'
    area.setdefault(top, []).append(r)
ORDER = ['(root)', '.vscode', 'docs', 'include', 'src', 'tests', 'sim', 'tools']
out = []
out.append('# Repository index\n')
out.append('_Generated by tools/repo_index.py from every tracked file: purpose from the file\'s own\n'
           'header (prolog PURPOSE, docstring, first comment or title), owner lane from docs/MODULE_STATUS.md,\n'
           'referenced-by = how many files mention the path (Makefile, tools, docs, lane notes = docs/modules\n'
           'and docs/history, src, include, tests, sim), last change = last commit touching it (moved files:\n'
           'their previous path; "uncommitted" = new in this session). Verdicts: live, generated, history,\n'
           'archived, or a proposal (docs/overview/ORGANIZATION.md, applied later by tools/apply_layout.sh).\n'
           'Regenerate: `python3 -I tools/repo_index.py > docs/overview/INDEX.md`._\n')
total = sum(len(v) for v in area.values())
out.append('%d files. Areas: %s.\n' % (total, ', '.join('[%s](#%s) (%d)' % (a, a.strip('().').replace('/', ''), len(area[a])) for a in ORDER if a in area)))
LEGEND = {
 '(root)': 'Build entry points and repository-wide configuration.',
 '.vscode': 'Editor support.',
 'docs': 'Operational docs at the top (README is the repo root; STATE, PLAN, ARCHITECTURE, CODING_STANDARD, NAMING, STYLE, TRANSLATION_GUIDE, TIPS, MODULE_STATUS), the system overview (overview/), per-lane notes (modules/), live agent process files (agents/), and the process record (history/: WORKLOG, RENAMES, finished assignments, tree-wide wave notes).',
 'include': 'The public API, one header folder per subsystem (agc/<subsystem>/), mirroring src/.',
 'src': 'Flight software (libagc) by subsystem; src/sim = the yaAGC drop-in harness (agc_sim); src/nassp = the NASSP engine shim; src/erasable_map = erasable access by AGC address.',
 'tests': 'One test binary per file, mirroring src/ subsystems; tests/oracle = differential tests against yaAGC dumps.',
 'sim': 'Harnesses that RUN the built AGC against existing tools: scenarios + sim_client.py (headless DSKY), lm_simulator/ (Virtual AGC LM Simulator), nassp/ (NASSP 9).',
 'tools': 'Tools that work ON the tree: checks, formatter, generators, analysis, the yaAGC oracle, Frama-C prove, archived one-offs.',
}
for a in ORDER:
    if a not in area: continue
    out.append('\n## %s\n\n%s\n' % (a, LEGEND.get(a, '')))
    out.append('| Path | Purpose | Owner | Referenced by | Last change | Size | Verdict |')
    out.append('|---|---|---|---|---|---|---|')
    for r in sorted(area[a], key=lambda r: r[0]):
        p = r[0]
        pur = MANUAL.get(p) or r[3] or ''
        if p.endswith('.png'): pur = 'screenshot for the lane note'
        pur = re.sub(r'\s+', ' ', pur).replace('|', '\\|')
        if len(pur) > 140: pur = pur[:137].rsplit(' ', 1)[0] + ' ...'
        d = r[2]
        if d == '?' and p in prev:
            d = subprocess.run(['git', 'log', '-1', '--format=%cs', '--', prev[p]], capture_output=True, text=True).stdout.strip() or '?'
        if d == '?' : d = 'uncommitted'
        size = int(r[1])
        sz = '%d B' % size if size < 10240 else '%d KB' % (size // 1024)
        out.append('| `%s` | %s | %s | %s | %s | %s | %s |' % (p, pur, lane(p), refs(r), d, sz, verdict(p)))
print('\n'.join(out))
