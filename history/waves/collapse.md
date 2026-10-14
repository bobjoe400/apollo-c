# collapse -- single-use helper collapse

## Scope
docs/history/assignments/collapse.md steps 1-4. Phase 1 (tooling + candidate list) was
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

## Forwarding wrappers (user rule, phase 3)
Rule, independent of caller count: a function in src/ (not src/sim/) whose
body is only `if (!AGC_ASSERT(c)) { return ...; }` guards plus exactly one
statement calling another function with its own parameters (any order,
renamed or cast; optionally returning the call's value) is a forwarding
wrapper: it is deleted and every caller calls the callee.  A wrapper guard
the callee already asserts is dropped; one it does not assert moves into
the callee once (core/exec public functions may be edited for that).
Tool: tools/unwrap.py (`--list` classifies by body shape on the libclang
AST; `WRAPPER --file F [--move N] [--drop N]` applies, compiles every
changed TU and includer, reverts on failure).  Guard comparison is
structural after value normalisation (WORD15 == 077777u == AGC_CHAN_MASK,
NULL == 0, `((agc_dp_t) -0x0FFFFFFF)` by value) and after expanding a
callee guard that calls a one-line predicate of its own file
(chan_valid, w15_ok).

Found 113.  Removed 26: chan_write / chan_or / chan_and (fresh_start.c),
chan_or (imu/mode.c, t4rupt.c), ch_or (t4rupt_csm.c), rr_chan_or (radar),
raise_alarm, im30_set, im33_set, markstat, phase_word, sp_ts,
agc_csm_servxt, agc_integ_flag, rz_state, rz_clear_ovf, csi_f, csi_set,
agc_conic_mload_ / mstore_ / mtad_ / mshift_, and the three conic exports
agc_conic_cogaovfl_ / time_out_fill_ / time_out_clear_ (the file-private
callee now carries the exported name instead of a wrapper).
Guards moved into the callee: `ctx->id != AGC_JOB_NONE` into
agc_job_servexit; `n > -128 && n < 128` (shift count) into agc_tp_shift.
Guards dropped because the callee asserts the fact in another form:
chan_write's `0 <= chan < 040` and `PYJETS <= chan <= CHAN14` (agc_chan_*
assert chan_valid and chan_writable, i.e. a real output channel; the
narrower range was the fresh-start caller's own), imu/mode.c's
`chan == 12 || chan == 14` (same), sp_ts's `ovf != 0` (agc_w_ts asserts it
before the store), agc_conic_mtad_'s TP range (agc_tp_add asserts
agc_w_ok on the major word), agc_conic_mshift_'s `ip != NULL` (in
agc_tp_shift's combined guard), csi_f/csi_set's CSI-flag range
(rz_vflag_* assert the flag range; every call passes a CSI constant),
time_out_clear's `sizeof (*out) > 0` (cannot fail, does not count).
Kept 87: 81 declared in include/agc (public API other lanes link against;
59 of them are dispatch-registry job/task entry points); csm_dap_ddoubl and
lm_dap_ddoubl (forward one parameter twice: das(x, x) -- not a pure
forward, and DDOUBL names the AGC instruction); agc_csm_vn (converts the
callee's uint16_t to agc_sp_t); exec_up_servidle (exec -> LM up-call:
exec may not include the LM layer, ARCHITECTURE.md); startsub (its
post-check "executive idle" holds for the fresh-start path but not for
ENEMA, STARTSB1's other caller, so it cannot move into STARTSB1);
set_markstat (its guard "MARKSTAT holds a VAC address" is the marking
routine's own invariant; agc_erase_set_markstat(0) is a legal call
elsewhere, so it cannot move into the setter).  No two-level chain was
removable (agc_waitlist_fixdelay -> vardelay -> call are public).
Verification: make OK, make test OK (all tests and oracles pass; the only
output difference is one expected-assert line number in
nav/integration/init.c), check_rules 0 hard, uncrustify clean outside
src/sim, make callgraph regenerated.

## Repeated guards (user rule, phase 3)
Within a function a guard whose AGC_ASSERT condition repeats an earlier one
(whitespace-normalised text, the tools/check_rules.py rule 5 hard check) is
removed, the first kept.  Found 20 outside src/sim:
* 2 truly redundant (nothing between changed the fact): set_lampword,
  ll_rodcomp -- removed.
* 9 were the same text on a *different* value: `st == AGC_STATUS_OK` /
  `st != AGC_STATUS_BAD_ARG` re-checking `st` after a new call (and once in
  an exclusive branch).  Removing them would leave a call's status
  unchecked (rule 7), so each call's status got its own name instead
  (st_optcadr, st_ca_ece, st_tvc, st_cage, st_pole, st_cduw, st_pad x2,
  st_p in PAXIS's PSKIP branch); the facts are now distinct texts.
* 9 post-checks of a modified value or of a module invariant.  The lead
  narrowed the rule afterwards (check_rules flags only back-to-back
  repeats; a re-check after intervening statements is a postcondition), so
  the real ones were restored: agc_rupt_go's inhint depth after GOPROG,
  pin_invariants() at the end of agc_dsky_keys / pin_putcom / pin_dspdcput
  / pin_vbfandir, lm_dap_switch_curves' ACCSW range, agc_lm_dap_rcsmon's
  PVALVEST range.  Left out because they cannot fail: agc_w_tp_shl1's
  agc_w_ok(m.m0) (agc_w_ts keeps the word in range) and
  agc_time_advance_cs's TIME2 check after the mask.
check_rules: 0 hard.  ll_landjunk used to overwrite the status of
agc_latlong_from_r / agc_navaux_set_latlong unchecked; LANDJUNK (Luminary
p. 791) calls LAT-LONG and falls straight into R-TO-RP, with no failure
exit, so the status is now checked and propagated like R-TO-RP's.

## Progress
* DONE make callgraph, callgraph.tsv/.dot, collapse_candidates.tsv
* DONE inline_one.py tested on a scratch copy: 13 mechanical inlines, make
  lib/lib-debug/test OK, make format + rebuild OK, check_rules 0 new hard;
  revert path and header-prototype deletion tested on synthetic files
* DONE phase 2 applied (see Result)
* DONE phase 3 forwarding wrappers and repeated guards

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
* libclang gives a literal that comes from a macro either no tokens or the
  whole span from the #define to the use; read the identifier at the
  expansion location and resolve it with `gcc -dM -E` to compare by value.
* A wrapper's guard placed after its call is a post-condition: it cannot be
  moved to the top of the callee.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ A tree-wide pass (single-use helpers folded into their callers): it changed code owned by the translation lanes without changing what the program decides, so it has no program flow of its own. The flows as the code now stands are in the lane sections: [exec-sched](../../docs/modules/exec-sched.md#program-flow-and-decisions), [core-numeric](../../docs/modules/core-numeric.md#program-flow-and-decisions), [state-flags](../../docs/modules/state-flags.md#program-flow-and-decisions), [nav-conic](../../docs/modules/nav-conic.md#program-flow-and-decisions), [nav-integ](../../docs/modules/nav-integ.md#program-flow-and-decisions), [nav-aux](../../docs/modules/nav-aux.md#program-flow-and-decisions), [io-pinball](../../docs/modules/io-pinball.md#program-flow-and-decisions), [io-nouns](../../docs/modules/io-nouns.md#program-flow-and-decisions), [imu](../../docs/modules/imu.md#program-flow-and-decisions), [guid-powered](../../docs/modules/guid-powered.md#program-flow-and-decisions), [guid-landing](../../docs/modules/guid-landing.md#program-flow-and-decisions), [guid-entry](../../docs/modules/guid-entry.md#program-flow-and-decisions), [p37](../../docs/modules/p37.md#program-flow-and-decisions), [dap-lm](../../docs/modules/dap-lm.md#program-flow-and-decisions), [dap-csm](../../docs/modules/dap-csm.md#program-flow-and-decisions), [prog-rendz](../../docs/modules/prog-rendz.md#program-flow-and-decisions), [prog-align](../../docs/modules/prog-align.md#program-flow-and-decisions), [sys-misc](../../docs/modules/sys-misc.md#program-flow-and-decisions), [sim](../../docs/modules/sim.md#program-flow-and-decisions); the system view is [docs/overview/README.md](../../docs/overview/README.md) and its flowcharts.
