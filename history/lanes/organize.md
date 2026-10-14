# organize -- lane history

Process diary of the organize lane, moved verbatim out of [docs/modules/organize.md](../../docs/modules/organize.md), which keeps the reader-facing explanation. Note dates: 2026-10-13 to 2026-10-14 (first and last commit touching the note).

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
- Working tree after the safe moves (scratch copy, 2026-10-13): make OK;
  make test 0 failures; make check 0 hard, format-check clean, 0 style
  findings in 325 files; make nassp-replay 5/5 images agree; make lm-sim
  34/34 checkpoints matched.
- apply_layout.sh on a scratch copy of bd7a7ee + this lane's edits, then:
  make OK; make test (36 binaries + 14 scenarios) 0 failures; make check 0
  hard, 0 style findings; prolog-report 0; always-ok-report 0;
  nassp-replay 5/5; make lm-sim 34/34 checkpoints matched.
- The lead committed the safe moves (bcc1329) after 7b99601 had swept in
  only the staged renames. The old-path WORKLOG entry from bd7a7ee was
  merged into docs/history/WORKLOG.md.

## Lessons
- xref's "prolog" rows include labels a prolog merely names, so "one label
  -> several functions" overstates duplicates about 5x; read the rows.
- "Callers all in another folder" is mostly by design here (storage
  accessors, restart resets each module supplies to change_program); judge
  by the AGC section and the statics a function touches, not by callers.
- An include whose symbols a file does not use can still be load-bearing
  (transitive declarations): prove removals by compiling and comparing
  object code, one line at a time, then all together.

