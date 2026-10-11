# collapse -- single-use helper collapse (phase 1: tooling + candidate list)

## Scope
docs/agents/collapse.md steps 1-2 and the candidate list of step 3. Nothing
applied to the real tree yet: waiting for the lead's review of
docs/overview/collapse_candidates.tsv.

## Tools
| file | what |
|---|---|
| Makefile `callgraph` | compiles src/ + tests/ at -O0 -fno-inline -fcallgraph-info=su,da into build/callgraph/ (one .ci per TU), runs cflow, then tools/callgraph.py |
| tools/callgraph.py | parses the .ci VCG files (nodes, static = "file:name" title, one edge per call site), cross-checks extern->extern edges with cflow (build/callgraph/crosscheck.txt), libclang for extents/lines/comments/guards; writes docs/overview/{callgraph.tsv, callgraph.dot, collapse_candidates.tsv} |
| tools/cgast.py | libclang helpers: structural canonical form of expressions, guard facts, statement effects (writes / writes through pointers / calls), established guard facts at a call site |
| tools/cgast_assert.h | force-included for libclang only: AGC_ASSERT as a function so asserted expressions keep real extents |
| tools/inline_one.py | HELPER CALLER [--file] [--dry-run] [--max-lines 60]: splices a mechanical candidate, de-duplicates guards, re-indents, deletes definition + prototype (same file or src/ private header), compiles, reverts on failure |

## Candidate rule
Defined in src/ (not src/sim/), exactly one caller function, caller not in
tests/, src/sim/ or dispatch.c, not declared in include/agc/, header comment
(definition, else private-header prototype) carries no AGC label: no page /
.agc / "label" reference and does not open with a real AGC label (label list
from both ropes). 1004 single-caller helpers, 828 kept for their label, 176
candidates.

## Progress
* DONE make callgraph, callgraph.tsv/.dot, collapse_candidates.tsv
* DONE inline_one.py tested on a scratch copy: 13 mechanical inlines, make
  lib/lib-debug/test OK, make format + rebuild OK, check_rules 0 new hard;
  revert path and header-prototype deletion tested on synthetic files
* TODO (after review) apply, hand-collapse refused shapes, make check/test,
  final counts and readability wins

## Lessons
* GCC -fcallgraph-info at -O2 attributes inlined callees' calls to the
  caller (with the callee's line numbers); the graph must be built at -O0
  -fno-inline.  "(static)" in a node label is the stack-usage qualifier,
  not linkage: a static function is the node whose title is "file:name".
* cflow 1.7 drops calls after a `for` statement holding nested calls
  (149 extern edges missing, 0 extra); use it only as a cross-check.
* libclang collapses the extents of everything inside a macro argument, so
  guard conditions inside AGC_ASSERT(...) lose operator tokens; parsing
  with AGC_ASSERT declared as a function (tools/cgast_assert.h) fixes it.
  Literals from macros (`true`, NULL) have no tokens; read the source at
  the expansion location.
* A call under a `case` label cannot become declarations directly
  (C11: a label is not followed by a declaration): the block needs braces.
