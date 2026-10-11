# collapse -- single-use helper collapse

## Scope
docs/agents/collapse.md steps 1-4. Phase 1 (tooling + candidate list) was
reviewed by the lead; phase 2 applied it with the lead's decisions: fan_*
verb-range splits kept, multi-site helpers out of scope, protection only for
an AGC label distinct from the caller's.

## Tools
| file | what |
|---|---|
| Makefile `callgraph` | compiles src/ + tests/ at -O0 -fno-inline -fcallgraph-info=su,da into build/callgraph/ (one .ci per TU), runs cflow, then tools/callgraph.py |
| tools/callgraph.py | parses the .ci VCG files (nodes, static = "file:name" title, one edge per call site), cross-checks extern->extern edges with cflow (build/callgraph/crosscheck.txt), libclang for extents/lines/comments/guards; writes docs/overview/{callgraph.tsv, callgraph.dot, collapse_candidates.tsv} |
| tools/cgast.py | libclang helpers: structural canonical form of expressions, guard facts, statement effects (writes / writes through pointers / calls), established guard facts at a call site |
| tools/cgast_assert.h | force-included for libclang only: AGC_ASSERT as a function so asserted expressions keep real extents |
| tools/inline_one.py | HELPER CALLER [--file] [--caller-file] [--dry-run] [--max-lines 60]: splices a helper into its caller on the libclang AST (shapes and rules in its docstring), de-duplicates guards, deletes definition + prototypes, checks rule 4 on the merged caller, compiles every changed TU and every includer of a changed header, reverts on failure |

## Candidate rule
Defined in src/ (not src/sim/), exactly one caller function, caller not in
tests/, src/sim/ or dispatch.c, not declared in include/agc/.  Protected
(lead decision, phase 2) only when the header comment (definition, else the
private-header prototype) names as its own an AGC *code* label (label on an
instruction, not on ERASE/EQUALS/DEC/OCT/ADRES/... data) that the caller
does not claim: the caller's head labels and any "A .. B" range in its first
line count as the caller's.  A page reference, or the parent's label/range,
does not protect.

Counts before applying: 1004 single-caller helpers, 603 protected
(label-distinct), 401 candidates = 176 under the first rule + 225 page-only
(column `protection` in collapse_candidates.tsv; label-distinct rows are
appended with mech = kept).  In scope (one call site, not fan_*): 333.

## Result
* 153 collapsed: 150 by tools/inline_one.py in three passes (every one
  compiled, formatted with uncrustify, rule 4 checked on the merged caller),
  ddv_normalised -> agc_w_ddv (parameter assigned but dead in the caller),
  and two by hand: t4_cage_engine_off -> t4_imucage (early return became
  if/else), rr_remode_done -> rr_axis_advance (early return became ?:).
* Re-split along AGC sections: entry_step_a/entry_step_b (one switch cut in
  two halves for length, opaque suffixes) became entry_initial_roll_step,
  entry_huntest_step, entry_upcontrol_step, entry_ballistic_step,
  entry_final_phase_step, one per "* START" section of REENTRY_CONTROL.agc
  (src/csm/entry/reentry.c).  The other roots with 3+ remaining pieces
  (kepler, lambert, findcduw, GAMDV search, CSI, noun tables, downlink items)
  are already cut at AGC paragraphs with descriptive names, or are tables
  split by owner; left as they are.
* 176 kept, with reason (the refusal text of inline_one.py):
  | n | reason |
  |---|---|
  | 96 | multi-exit helper (returns outside guards) whose value is used: inlining needs a result variable and else-chains; the named predicate/conversion reads better |
  | 29 | cross-file: the helper uses its own file's statics, macros or includes (vehicle-split exec/change_program/*, flags_csm/lm, conic private state): it belongs to its file |
  | 14 | value helper that is a switch with a return per case (noun/downlink tables) |
  | 14 | merged caller would exceed 60 code lines |
  | 9 | call under && / || / ?: (inlining would change what is evaluated) |
  | 6 | helper assigns a parameter the caller still reads |
  | 6 | helper guard but the caller has no recovery value of its type (enum/struct returns) |
  | 2 | another call in the same statement (evaluation order) |
* Out of scope: 64 multi-site rows, fan_* (6 rows).

Semantics: every transformation is exact on the non-failure path.  The one
deliberate change is on the assertion-failure path: a helper guard that is
not already established by the caller now returns the caller's own recovery
value (its first guard's constant return, else `return;`,
`AGC_STATUS_BAD_ARG`, `false`) instead of the helper's.  Assertion density:
check_rules warnings 152 -> 146.

Verification: make (lib, lib-debug, sim) OK; check_rules 0 hard in my scope
(the 2 hard are src/sim, sim lane); uncrustify --check clean on src/ and
include/ outside src/sim; every test binary prints output identical to the
pre-collapse baseline; tests/oracle/entry (new, oracle-cases lane) failed
11 cm_pose checks identically before and after the collapse while that lane
was working, and passes 521/521 on the final tree;
make callgraph regenerated (3990 functions, 248 candidates left, 7
mechanical, all of them fan_* or too long).

## Ten biggest readability wins
1. entry sequencer: two opaque halves -> five dispatchers named after the AGC sections.
2. STARTDAP "first half" (startdap_zero) back into lm_dap_startdap.
3. BACKP "first part" (backp_measure) back into lm_dap_backp.
4. RODCOMP "first part" (ll_rod_pipas) back into ll_rodcomp.
5. ASCENT "first part" (as_vgvect) back into agc_lm_ascent.
6. TARGETNG "part 1" (targ_velocity) back into agc_entry_targeting.
7. SFOUTABR "part 2" (pin_sfout_word) back into pin_dspdcput.
8. mu_range / mu_shaft / mu_trunnion: each measurement kind now reads in its case of mu_compute.
9. ddv_normalised -> agc_w_ddv and genddv_words -> agc_w_genddv: DDV reads as one routine as on p.1167.
10. got_count / got_index -> agc_job_uplink_update_load: the V71 load stages read in one place.

## Progress
* DONE make callgraph, callgraph.tsv/.dot, collapse_candidates.tsv
* DONE inline_one.py tested on a scratch copy: 13 mechanical inlines, make
  lib/lib-debug/test OK, make format + rebuild OK, check_rules 0 new hard;
  revert path and header-prototype deletion tested on synthetic files
* DONE phase 2 applied (see Result)

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
* A guard return copied into a different function must not mention locals
  (`return (st);` from a later guard of the caller was a first, wrong pick).
* Deleting a prototype: the comment "above" it may be the trailing comment
  of the previous prototype; a header comment must open its own line.
* After deleting from a private header, compile every TU that includes it
  (they are found by resolving the #include paths, several are ../../).
* A re-split dispatcher must not claim its steps' labels in its first
  comment line, or the protection rule treats the steps as its fragments.
