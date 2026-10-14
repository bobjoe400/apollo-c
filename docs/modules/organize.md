# organize -- repository index, layout review, layout script

Scope: index every file (docs/overview/INDEX.md); judge the layout and the
code's internal structure against the lead's seven criteria and checks
(a)-(g); apply the safe moves (docs, tools, sim outside the NASSP lane);
prepare the rest as tools/apply_layout.sh, verified on a scratch copy, for
the lead to run between NASSP milestones. Rationale, findings and counts:
docs/overview/ORGANIZATION.md.

## Progress
- DONE safe moves: docs/history/ (WORKLOG, RENAMES, 41 finished assignments,
  20 wave notes), docs/agents/brief.md, docs/overview/data/ (6 generated
  tables), tools/archive/ (6 one-offs), harness drivers into sim/; all
  references rewritten; Makefile CG_SRCS excludes src/nassp
- DONE code-structure analysis (a)-(g) on a fresh `make callgraph` + xref
- DONE tools/apply_layout.sh + tools/layout/ (moves, 10 dead functions,
  3 stale prototypes, 102 file-static, 213 unused includes, doc edits)
- DONE docs/overview/INDEX.md, docs/overview/ORGANIZATION.md
- Verification: see "Verification" below

## What moved (applied, uncommitted)
See ORGANIZATION.md section 2. Old -> new maps: the git index (`git status`
shows each rename).

## What is proposed
ORGANIZATION.md sections 3-5: the script (src/include/tests/NASSP moves and
the semantics-neutral code edits) and the findings that would change
behaviour (duplicate translations, cross-vehicle placement, labelled
routines nothing calls, file splits, internal.h prototypes).

## Verification
- Working tree after the safe moves (scratch copy): make OK; make test 0
  failures (35 binaries + scenarios); make check 0 hard, format clean, 0
  style findings; make nassp-replay and make lm-sim: see below.
- apply_layout.sh on a scratch copy (committed baseline): see below.

## How the index and the analysis were made
(the analysis scripts are kept with the layout script: tools/layout/{analyze.py, owner.py, inc_verify.sh}; usage in their headers)
- tools/repo_index.py (kept, live): renders INDEX.md (per file: size, last
  commit, purpose from its header, owner lane, files that mention its path,
  verdict).
- analyze.py over docs/overview/data/callgraph.tsv (fresh `make callgraph`,
  CG_SRCS without src/nassp, WARN without -Werror for free_fall.c), xref
  TSV (`tools/xref.py --tsv`), ctags of every header: a_wrong_owner,
  b_header_internals, c_duplicates, d_dead, e_files, f_vehicle,
  g_cycles, g_unused_includes; owner.py judges (a) by AGC section and
  storage; inc_verify.sh proves each include removal (object code equal).

## Lessons
- xref's "prolog" rows include labels a prolog merely names, so "one label
  -> several functions" overstates duplicates about 5x; read the rows.
- "Callers all in another folder" is mostly by design here (storage
  accessors, restart resets each module supplies to change_program); judge
  by the AGC section and the statics a function touches, not by callers.
- An include whose symbols a file does not use can still be load-bearing
  (transitive declarations): prove removals by compiling and comparing
  object code, one line at a time, then all together.
