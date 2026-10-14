# verify-collapse -- second pair of eyes on status-handling-2

## Scope
docs/history/assignments/verify-collapse.md: independent check of the status-handling-2
diff (`git diff 1b72892 a02850c -- src include tests`). Items 1-8 in order.
Scratch: SCRATCH/verify-collapse/.

## Progress
- SCOPE (lead, 2026-10-11): only the eleven already-reviewed lanes. Excluded
  (each gets its own review of the diff): src/exec/**, src/programs/alignment/**,
  src/csm/alignment/**, src/lm/alignment/**, src/io/service/**, src/io/verbs/**,
  src/io/telemetry/**, src/guidance/**, src/csm/burns/sps/**, src/lm/burns/**,
  src/nav/integration/** and their tests. Item 8: test_sysmisc is sys-misc
  (excluded) and test_integration is nav/integration (excluded): skipped.
  src/sim/** read-only (sim lane running).
- DONE 1 side effects / mixed conditions. libclang inventory of every
  AGC_ASSERT in 1b72892 vs a02850c (6,041 -> 1,920; 4,144 removed by
  file+condition multiset, 3,007 in scope). In scope: 22 removed asserts
  whose argument passes `&` to a call (vec_ok/mat_ok x20 in core/erasable.c,
  dif_cont_valid in io/dsky, rr_radmodes_test in lm/radar), 18 in an `if`
  with a normal-flow disjunct/conjunct (core shifts `|| n == 0` x5, mpac
  `(o != 0) &&` x2, compensation x2, dsky idleret3/display, sim x8), 1
  non-if `(void)AGC_ASSERT` (csm/optics sxtang1). Every callee in the 537
  in-scope removed conditions that contain a call was read: all are pure
  predicates/accessors (no writes; agc_chan_read is a plain array read).
  Every normal-flow branch is still taken in the head (n == 0 early
  returns, fd/strncmp/clock_gettime/sscanf tests, gcomp_read call,
  agc_dsky_response call, dif_cont_valid inlined with all three tests).
  41/41 agree.
- DONE 2 deletions in untested functions. Reach = `[eva] Done for function`
  in tools/prove/runs/<entry>.log (the CSVs only list functions that still
  have a property). Joined by file+function (3,047 in-scope removals at
  function level): 1,026 removed guards sit in 868 functions no test main
  reached (514 reached by nothing, 512 only by the partial sim run), 101
  helper functions are gone (inlined; see item 5). Of the 1,026: 605
  pointer non-NULL guards or src/sim rows (sim read-only) -- machine check (libclang, head tree): no call
  site passes a literal NULL/0 at a guarded parameter position (1 hit,
  agc_vec_div's `s != 0`, is the scalar, and vsc_shift14 returns before it
  for s == 0); 177 word-range facts (SP/DP/TP/15-bit/LM_NEGMAX..POSMAX,
  representation invariants) accepted by category; 47 index guards: every
  call site read (constants, bounded loops, DETENT 1..6 after 0/7 tests,
  RTSTLOC under the caller's own test); 19 job-id guards: constant
  AGC_JOB_* at every call (or a parameter whose callers pass constants);
  178 other: read, incl. all 51 removed `st == OK` / `st != BAD_ARG`
  guards in scope (each callee now void, or the status is returned).
  Agree on all. Minor: display_if.c:1063 `st = agc_dsky_blanksub()` is now
  a dead store (BUSY unreachable there: NVMONOPT just passed the same
  DSPLOCK/MONSAVE1-bit-14 test, no job switch between; the AGC's TCF NVDSP
  path is dead in flight too); 82 function headers still promise
  "BAD_ARG for a NULL ctx" on functions that lost ctx (doc drift, for the
  document lane).
- DONE 3 manual.tsv. 998 rows, 644 GUARANTEED in scope. (a) Every row
  whose condition names a file-scope static (ctags, base tree): 68 rows
  on 49 statics; every writer grepped in head src+tests (e.g. lm_pad
  TAUROD/LEADTIME written only by tests, zero in flight; rd.samplim
  decrements gated by `> 0`/`== 0`; ln.loc += 7 bounded by nmarks <= 5;
  tmark via prelaunch_set_tmark's own status check; ab_pipctr1 stored
  before the writer's still-present range check). 68/68 agree.
  (b) Stratified random sample (seed 7) of the other 576: 181 rows over
  49 directories/files (core 49, csm/entry 20, lm/dap 25, io/dsky 12,
  imu 20, lm 25, programs/nav 10, sim 13, ...). Divisor guards in
  csm/entry all precede DDV (agc_dp_div models zero); LM DAP divisions
  go through lm_dap_dv; shift-count/index guards traced to constants,
  NORM counts or 8-bit packet fields. 181/181 agree; none shown false.
  Note: the per-row "why" is directory boilerplate (one text per lane),
  so the file is a log of decisions, not of evidence.
- DONE 4 status conversions. Signature diff (ctags, base vs head): 30
  functions gained agc_status_t, 22 became void. In scope: chklinus (2
  callers: toballa returns it, automanv tests `!= OK` and returns it),
  cagetest (5 task callers return the IMUBAD status when caged),
  as_engoff (agc_lm_ascent returns it; both job callers tail-return),
  ll_start_rate_of_descent_landing / inlined STARTP67 (NEWMODE status ->
  agc_lm_guildenstern -> agc_lm_job_lunland `!= OK` return), ll_rodcomp
  (DOIT's waitlist status returned instead of BAD_ARG), gyro_next
  (agc_exec_job_wake_if_asleep `!= OK` return), sim_channels_init (start
  chain `== OK ?`), sim_channels_tick (g_stats.errors). Every new failure
  path reaches the job/task return (the executive's handling, out of
  scope) or the sim's own accounting. No ignored status: dead-store scan
  (libclang, every agc_status_t store with no later read) finds 1 in
  scope (display_if.c:1063, item 2). The 14 in-scope `!AGC_ASSERT(st !=
  BAD_ARG)` sites all `return (st)` right after (propagate). 10/10 agree.
  Residual (pre-existing, not the lane's): 62 in-scope `if
  (!AGC_ASSERT(st == OK)) return (AGC_STATUS_BAD_ARG);` rewrite a callee's
  FULL/BAILOUT into BAD_ARG (66 at 1b72892).
- DONE 5 inlined helpers. 239 functions defined at 1b72892 are gone at
  a02850c (ctags), 178 in scope (inlined, plus the PLAIN one-liners and
  unused deletions). libclang on each base body: none assigns a by-value
  parameter; after discounting guard returns, 48 have more than one
  return. 44 of those were called as `return helper(...)` (tail): the
  merged returns are the caller's own returns; argument expressions are
  bound to locals before the body (zone_between_curves `f_curves =
  lm_dap_sat(g)`, prec_time_step `rf_step`). The non-tail ones read in
  full: getdown (status chain `st_getdown`, negation kept), pp_maxvtest
  (early returns -> nested if/else, every store kept), fc_dcmcl (single
  exit once its guards went), chan_writable (renamed agc_chan_writable,
  not inlined); js_acy/js_bdz by hand into js_roll (`taken` reproduces
  both TAUCHCK returns; `> 0` vs `!= 0` index tests kept). 40 random
  single-exit merges (seed 11) compared statement by statement (renamed
  locals `_xcommon`, `_rect`, `_tripa`, ...; `->` to `.` when the caller
  holds the struct): 40/40 same statements, same order. -Wshadow
  (build flag) rules out capture of a file static by a caller local. No
  stale prototype of a removed function in any header. 52/52 agree.
  Note: inline_one.py pastes the helper's full header block into the
  caller body (77 blocks, 154 bar lines in src/); comments only.
- DONE 6 return rewrites. Ground truth: the lane's pre-step-8 tree
  (SCRATCH/status-handling-2/src_before_returns) and its report ret.txt
  (147 `early` rows with tail <= 12, 15 `trailing`). All 9 in-scope
  trailing deletions diffed (imu_modeexit, agc_imu_setcoars,
  agc_imu_pipa_fail_enable, agc_imu_pipfree, manustat, agc_radar_lralt,
  agc_radar_lr_velocity_beam, agc_radar_lrvel, agc_rendz_reset): no
  statement lost. 40 of 99 in-scope rewritten functions (seed 5) diffed
  pre vs head: every negation correct (`< 0` -> `>= 0`, `!(a || b)`,
  `!= OK` -> `== OK` with the tail nested), conditions with side effects
  (agc_interp_test_ovf clears OVFIND) still evaluated once, later
  inlining inside the new blocks (stroke_next_set) kept. 49/49 agree.
  Finding (minor, not fixed): programs/navigation/common.c:441
  agc_rendz_reset now reads `if (agc_rendz_selectmu(rz, false) != OK) {
  }` -- the trailing-return deletion left an empty branch that drops
  selectmu's status (it was dropped by a bare `return;` before, so no
  behaviour change; fixing needs agc_rendz_reset to return status, and
  its flight caller is src/exec/change_program/idle.c, excluded). The
  same rewrite left imu/maneuver.c:874 `if (!AGC_ASSERT(tm_hi >= 0 &&
  tm_lo >= -ONESEC_CS)) { agc_relint(); }` (fact holds by construction:
  tm_lo = (lo - 1SEC) with lo >= 0; dead branch).
- DONE 7 void conversions and dropped parameters. Void (10 in scope:
  agc_erase_set_failreg, agc_sv_get, agc_imu_ballangs, agc_imu_comp_get,
  agc_imu_clear_cmds, agc_dsky_response, agc_dsky_set_dsptem,
  agc_rr_settrkf, rz_set_dsptem_dp,
  agc_ground_track_determination_flight_path): every non-OK return at
  1b72892 was a guard return (or the status of agc_rtb_v1sto2s /
  agc_dsky_set_dsptem, themselves guard-only and made void), so no caller
  could have depended on an AGC-defined failure; former `st` plumbing in
  callers removed consistently (items 1-2). Dropped parameters: libclang
  over 234 functions whose parameter list shrank (216 ctx, 13 ip, 3 jc,
  1 group, 1 p; 145 in scope): every base use of a dropped parameter
  outside an AGC_ASSERT guard (66 functions) is a pass-through to a
  callee that dropped the same parameter in head (checked against head
  signatures; 0 callees still take it). 155/155 agree.
- SKIPPED 8 (scope): test_sysmisc (sys-misc) and test_integration
  (nav/integration) are excluded. Recorded for the owning reviews: two
  in-scope tests also count reports -- tests/nav/auxiliary.c:502
  `CHECK(g_assert_reports == before + 2)` (the two agc_star_vector
  BAD_ARG statuses are already checked on the lines before; the count
  line is redundant) and tests/lm/dap.c:444 `CHECK(g_assert_reports ==
  reports)` (no report during a zero-erasable 1/ACCS run). Not edited.

## Findings
No change of the lane was shown wrong in scope; no src/ edit, no test
added. Minor items, recorded for the owning reviews / document lane:
1. src/programs/navigation/common.c:441 agc_rendz_reset: `if
   (agc_rendz_selectmu(rz, false) != AGC_STATUS_OK) { }` -- empty branch
   left by the trailing-return deletion; the status was already dropped
   (bare `return;`) at 1b72892. Proper fix (agc_rendz_reset returns the
   status, or selectmu becomes void once `rtmu > 0` is shown) touches the
   flight caller in src/exec/change_program/idle.c (excluded).
2. src/io/dsky/display_interface.c:1063 dif_nvdsp: `st = agc_dsky_blanksub()` is
   a dead store since the BUSY guard went (BUSY is unreachable there; the
   AGC's BLANKCHK `TCF NVDSP` path is dead for the same reason).
3. src/imu/maneuver.c:874 `if (!AGC_ASSERT(tm_hi >= 0 && tm_lo >=
   -ONESEC_CS)) { agc_relint(); }` -- returns rewrite + MANUSTAT merge left
   a guard with no return; fact holds by construction (dead branch).
4. 82 function headers still say "BAD_ARG for a NULL ctx" on functions
   that lost ctx; 16 say "0 on a failed precondition / no valid result"
   for guards that are gone; inline_one.py pasted 77 helper header
   blocks into caller bodies (document lane).
5. Pre-existing, not the lane's: 62 in-scope `if (!AGC_ASSERT(st == OK))
   return (AGC_STATUS_BAD_ARG);` turn a callee's FULL/BAILOUT into
   BAD_ARG; ~24 assert-then-clamp recoveries without a return (e.g.
   core/mpac.c:49, csm/entry/lexicon.c:105) that check_rules does not
   flag as silent.

## Agree / disagree per item
| item | checked | agree | disagree |
|---|---|---|---|
| 1 side effects / mixed | 41 removals (+537 conditions with calls read for purity) | 41 | 0 |
| 2 untested functions | 1,026 removals in 868 functions (604 by machine check, 422 read/categorised) | 1,026 | 0 |
| 3 manual.tsv | 68 static-invariant rows + 181 sampled = 249 | 249 | 0 |
| 4 status conversions | 10 conversions in scope, 14 `!= BAD_ARG` sites | 10 | 0 |
| 5 inlined helpers | 12 multi-exit/hand + 40 single-exit = 52 (178 in scope scanned) | 52 | 0 |
| 6 return rewrites | 9 trailing + 40 if/else = 49 | 49 | 0 |
| 7 void / dropped params | 10 void + 145 dropped-param functions = 155 | 155 | 0 |
| 8 test hygiene | skipped (scope) | - | - |

Gates (2026-10-11, shared working tree): make OK; make test 34/34, 0
failures; check_rules src include 0 hard; make check stops at
format-check on src/exec/interrupts.c (review-exec-sched's file, being
edited; nothing of mine).

## Lessons
* The CSVs list only functions that still own a property; reach per
  entry point is `[eva] Done for function` in tools/prove/runs/*.log.
* A file+condition multiset diff over-reports removals where a helper
  was inlined (the condition moves function); compare at
  file+function+condition, then read.
* EVA-only deletions rest on the test mains' inputs (e.g. ecadr <
  0100000 was "EVA VALID (oracle_dsky)"); re-derive such facts from the
  call sites (15-bit keyed words, uplink page check) before trusting them.
* manual.tsv's reason column is one boilerplate text per directory; for
  a second look, the writers of the static / the call-site arguments are
  the evidence, not the row.
* Deleting a trailing `return;` from a guard turns it into an empty
  branch or a return-less guard: grep for `{\n}` and `if (!AGC_ASSERT`
  blocks without `return` after any returns pass.
* Tail-position merges (`return helper()`) are safe with multi-exit
  helpers; check the non-tail ones and the argument binding to locals.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ A tree-wide pass (a check of status-handling-2): it changed code owned by the translation lanes without changing what the program decides, so it has no program flow of its own. The flows as the code now stands are in the lane sections: [exec-sched](../../docs/modules/exec-sched.md#program-flow-and-decisions), [core-numeric](../../docs/modules/core-numeric.md#program-flow-and-decisions), [state-flags](../../docs/modules/state-flags.md#program-flow-and-decisions), [nav-conic](../../docs/modules/nav-conic.md#program-flow-and-decisions), [nav-integ](../../docs/modules/nav-integ.md#program-flow-and-decisions), [nav-aux](../../docs/modules/nav-aux.md#program-flow-and-decisions), [io-pinball](../../docs/modules/io-pinball.md#program-flow-and-decisions), [io-nouns](../../docs/modules/io-nouns.md#program-flow-and-decisions), [imu](../../docs/modules/imu.md#program-flow-and-decisions), [guid-powered](../../docs/modules/guid-powered.md#program-flow-and-decisions), [guid-landing](../../docs/modules/guid-landing.md#program-flow-and-decisions), [guid-entry](../../docs/modules/guid-entry.md#program-flow-and-decisions), [p37](../../docs/modules/p37.md#program-flow-and-decisions), [dap-lm](../../docs/modules/dap-lm.md#program-flow-and-decisions), [dap-csm](../../docs/modules/dap-csm.md#program-flow-and-decisions), [prog-rendz](../../docs/modules/prog-rendz.md#program-flow-and-decisions), [prog-align](../../docs/modules/prog-align.md#program-flow-and-decisions), [sys-misc](../../docs/modules/sys-misc.md#program-flow-and-decisions), [sim](../../docs/modules/sim.md#program-flow-and-decisions); the system view is [docs/overview/README.md](../../docs/overview/README.md) and its flowcharts.
