# organize -- repository index, layout review, layout script

History of this lane: [history/lanes/organize.md](../../history/lanes/organize.md)

Scope: index every file (history/INDEX.md); judge the layout and the
code's internal structure against the lead's seven criteria and checks
(a)-(g); apply the safe moves (docs, tools, sim outside the NASSP lane);
prepare the rest as tools/apply_layout.sh, verified on a scratch copy, for
the lead to run between NASSP milestones. Rationale, findings and counts:
history/ORGANIZATION.md.

## How the index and the analysis were made
(the analysis scripts are kept with the layout script: tools/layout/{analyze.py, owner.py, inc_verify.sh}; usage in their headers)
- tools/repo_index.py (kept, live): renders history/INDEX.md (per file: size, last
  commit, purpose from its header, owner lane, files that mention its path,
  verdict).
- analyze.py over history/data/callgraph.tsv (fresh `make callgraph`,
  CG_SRCS without src/nassp, WARN without -Werror for free_fall.c), xref
  TSV (`tools/xref.py --tsv`), ctags of every header: a_wrong_owner,
  b_header_internals, c_duplicates, d_dead, e_files, f_vehicle,
  g_cycles, g_unused_includes; owner.py judges (a) by AGC section and
  storage; inc_verify.sh proves each include removal (object code equal).

## Program flow and decisions

_Final documentation pass (2026-10-14)._ This lane wrote no flight code:
it indexed the repository (`tools/repo_index.py` -> history/INDEX.md)
and prepared the layout script (applied as 755db91, kept in
`tools/archive/apply_layout/`). The layout it produced is described in
docs/ARCHITECTURE.md; the code-structure findings it raised were settled
by [structure](structure.md#program-flow-and-decisions).
