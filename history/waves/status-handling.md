# status-handling -- rule 7: no (void)-cast agc_ calls or AGC_ASSERTs in src/

## Scope
1. Remove every `(void) agc_...(...)` cast in src/ (712 sites) by either
   making the callee `void` (it cannot fail when called correctly) or doing
   what the AGC does on the failure path at that site.
2. (Scope widened by the lead.) Remove every `(void) AGC_ASSERT(cond);`
   (3,233 sites at the time of the request) -- the rule-7 dodge that also
   breaks rule 5 -- by giving each assertion an explicit recovery:
   `if (!AGC_ASSERT(cond)) { return <failure>; }`.
3. The same for the 42 `(void)` casts of module-local functions that the
   checker does not see (only `memset`/`fprintf` keep a commented cast).

## Before / after
| measure | before | after |
|---|---|---|
| `(void) agc_...` casts in src/ | 712 | 0 |
| `(void) AGC_ASSERT(...)` in src/ | 3,233 (3,009 when reached) | 0 |
| other `(void) local_fn()` casts | 42 | 0 (memset x5, fprintf x1 kept, commented) |
| check_rules hard violations | 713 (then 3,524 with the widened checker) | 0 |
| check_rules warnings | 147 | 154 (7 rule-5 files, see "Rule 5" below) |
| functions > 60 code lines | 1 (entry_step_a) | 0 |
| src .c compiling (-O2 -Werror, Makefile flags) | 220/220 | 220/220; tests 21/21 compile |
| linked test binaries | 7 (state: 2 fail, navaux: 2 fail) | same 7, same 4 failures, no new ones |

`oracle/core_numeric` (bit-exact against yaAGC) still passes after the core
changes (agc_w_ovf, agc_dp_norm_count, the *_ovf_only helpers). The other
tests still do not link (integration's job, unchanged).

## Phase A -- infallible APIs made void
Bad arguments are programming errors: each keeps its `AGC_ASSERT`s and returns
early, and returns nothing. Callers were rewritten with
`tools/archive/status_void.cocci` (`(void) f(a);` -> `f(a);`, `return f(a);` ->
`f(a); return;` in a void function or `f(a); return AGC_STATUS_OK;`), then the
`st = f(a);` assignments the compiler reported ("void value not ignored") were
rewritten (call kept; dead checks of `st` removed; `st` kept as a constant
only where still read). A last pass folded every status variable that could
only hold AGC_STATUS_OK (unwrapped `if (st == OK)`, removed checks that could
not fail, returned the constant, deleted the variable) -- 0 such artifacts
remain.

Core/exec (lead-owned headers, edited for this task):
* `agc_flag_set/clear/invert`, `agc_flagword_set`, new `agc_flag_put(f, on)`
  (SET or CLEAR by a computed condition; replaced 20 `c ? set : clear`
  ternaries and three local flag_put helpers).
* `agc_chan_write/or/and` (writing an input channel is now an assertion).
* `agc_time_set_cs`, `agc_rupt_set_time`, `agc_rupt_set_t5_task`,
  `agc_rupt_set_t6_task`.
* `agc_phase_change`, `agc_phase_change_var`, `agc_phase_change2`,
  `agc_phase_kill`, `agc_phase_newphase` (a bad code word is an assertion).
* `agc_exec_prio_change`: BUSY was a signal, not a failure. Now void; a caller
  that must give way tests `agc_exec_newjob() > 0` and does
  `agc_exec_job_yield()` (as the existing callers did with BUSY).
* `agc_alarm`, `agc_prioalarm`, `agc_exec_varalarm`, `agc_curtains`: ALARM
  "lights PROG, records, returns"; `return agc_alarm(x)` became
  `agc_alarm(x); return AGC_STATUS_ALARM;` (same status to the caller).
* `agc_exec_job_sleep` (only failure: not called from a job).
* New `void agc_interp_clear_ovf(ip)` for the BOV +1 / BOVB TCDANZIG idiom
  (32 sites); `agc_interp_test_ovf` keeps its bool for real BOV branches.
* New `void agc_exec_job_wake_if_asleep(job)`: JOBWAKE whose caller does not
  care that the job is gone (AGC JOBWAKE2: "EXIT IF SLEEPING JOB NOT FOUND").
* New `int agc_w_ovf(a16)` (DOUBLE; TS L: overflow only), `int
  agc_dp_norm_count(a)` (NORM X1, count only), `void agc_dp_add_ovf_only`,
  `void agc_dp_div_ovf_only` (DAD/DDV whose MPAC is never read, then BOV),
  `int rz_norm_xreg(a)`.
* Pending shims: `agc_waitlist_kill`, `agc_phase_set_tbase`,
  `agc_pfs_pipasr_noscale` declared void (KILLTASK, TBASE store and PIPA read
  cannot fail).

Module functions whose only failure was a bad argument (checked one by one:
every non-OK return sits under an AGC_ASSERT, and no fallible call inside),
made void the same way, in dependency order: see the regex list in
`tools/archive/status_void.cocci` (105 functions with the core ones above, e.g. `agc_guid_smnb/nbsm/axisrot/
reference_to_body`, `agc_imu_setcoars/pipuse1/pipfree`, `agc_lm_dap_stoprate/
zatteror`, `agc_csm_spsoff/tvczap/setmindb/setmaxdb/dap_zerojet/massprop`,
`agc_radar_lralt/lrvel`, `agc_rendz_flip/clradmod`, `agc_erase_set_thetad/
dsptab`, `imu_modeexit`, `align_phase_job`, `serv_phase`, `gnufaze5`).

## Phase B -- genuinely fallible calls (per callee)
| callee | AGC failure | decision at the (void) sites |
|---|---|---|
| agc_waitlist_call / longcall (25) | WAITLIST full: BAILOUT 1203; dt = 0: POODOO 1204 | already raised inside; caller returns the status (`agc_relint()` first when under INHINT); a void caller returns early |
| agc_exec_novac / findvac (19) | no core set / VAC: BAILOUT 1202 / 1201 | same |
| agc_poodoo / agc_bailout (8) | never return | `(void) f(x); return POODOO;` -> `return f(x);` (keeps the BAILOUT that POODOO gives with SERVICER on); integration's INT-ABRT POODOO moved to the driver's INTEG_NEXT_ABORT case |
| agc_exec_gotopooh (1) | NOVAC may BAILOUT | `agc_imu_status_check` returns it, else POODOO |
| agc_imu_status_check (7) | R02: alarm 210/220 + GOTOPOOH | caller returns (program abandoned) |
| agc_lm_dap_setmindb/maxdb/pflitedb/restordb/allcoast (11), agc_lm_dap_jet_accel_update | 1/ACCS NOVAC | propagate (relint first) |
| agc_csm_dap_rcsdapon (4), agc_dap_needler (5) | T5 setup / needle chain | propagate |
| agc_lm_updatevg, agc_lm_cross_product_steering, agc_lm_stclok1, agc_lm_throttle_flatout, agc_phase_newmode, agc_radar_lrpos2, agc_radar_powered_flight_designate | waitlist / findvac / display paths | propagate |
| agc_csm_updatevg, agc_csm_cross_product_steering, steering_after_vg_update, surely9_restart | INCRSVG: ALARM 1407 then "GOTO QTEMP" (returns) | propagate everything except OK and ALARM |
| agc_dsky_display (6) | PRIOBORT POODOO 1502, exec overflow | propagate; see the two GODSPRET and two CLEANDSP fixes below |
| agc_exec_job_wake (2) | not found: no error | `agc_exec_job_wake_if_asleep` |
| agc_dsky_releasedsp / pinbrnch (ab_legal) | ABORTALM ends the job | split into `ab_legal()` (bool) and `ab_abortalm()` (status) |
| agc_dp_sqrt/acos/asin wrappers of *_chk | ABORT 1302 / ALARM 1301 | wrapper tests the status and returns the documented value (0 / the limit); core cannot raise alarms (layering), callers that must use the *_chk forms |
| agc_vec_unit (2, only |v|^2 wanted) | -- | `agc_vec_sq` (the same VSQ that UNIT stores at 34D) |
| agc_lm_mungrav / agc_lm_ycomp (3) | -- | result stored explicitly (GDT1/2, Y) |
| intexit (2) | INTWAKE status | combined with the driver's failure status |

GODSPRET with a NULL continuation (dps_burn.c, two sites) always failed the
display layer's assertion, so V16N85 was never shown. Split at the AGC's
"CADR GODSPRET" into continuation jobs `agc_job_rcs_burn_n85_displayed` and
`agc_job_redo6_7_n85_displayed` (ids in include/agc/guidance/pending.h,
GUID_BASE + 100/101; dispatch.c wiring is integration's).
CLEANDSP/KLEENEX ("TC BANKCALL / CADR CLEANDSP", no return slots) no longer
require the three flash continuations in `agc_dsky_display`.

## Assertions (`(void) AGC_ASSERT`)
Done with a transformer keyed on the enclosing function's return type
(scratch script; `tools/archive/assert_recover.cocci` is the reusable Coccinelle form
of the mechanical part -- it converts 2,212 of the 3,222 sites of the
original tree), then by hand:
* agc_status_t: `return AGC_STATUS_BAD_ARG;` void: `return;`.
* value-returning: the function's own existing guard value when it had one,
  else `0 /* no valid result */`, `false`, `agc_vec_zero()`, `(T){0}`; step
  enums: ST_ERROR, INTEG_NEXT_FAIL, TVCX_FIN, LAMB_SUFFCHEK, KEPLER_KEPRTN,
  CSI_L_ALMXIT, AGC_JOB_NONE, AGC_TASK_NONE, AGC_RESTART_NONE,
  AGC_IMU_STALL_BAD.
* inside INHINT: `agc_relint();` before the return (28 sites).
* constant conditions (26): `_Static_assert` (one on a `static const`
  deleted).
* `AGC_ASSERT(false)` markers (18): the recovery statement after them is the
  explicit failure path; marker removed, reason kept as a comment.
* pointer accessors of static erasables (27 checks): no recovery a caller
  could use -> deleted; three `*_begin(ctx, ip)` helpers lost a duplicate
  ctx check and the ctx parameter.
* side effects inside an assertion (2, `agc_interp_test_ovf`): rewritten.
* agc_vec_zero/agc_vec_make recursed through their failure values: fixed.

## Rule 5 (warning-level)
Removing dead assertions (on the status of now-void calls, constants, accessor
invariants) dropped some files below 2.0. About 45 real pre/postconditions were
added (vehicle checks on LM/CSM-only code, pointer and range checks, set/clear
postconditions). Still below 2.0 vs. the original: csm/dap/state.c and
guidance/guid_state.c (pointer accessors only), exec/fresh_start.c 1.82,
imu/gyro.c 1.88, nav/integration/jobs.c 1.94, core/interp/vector.c 1.95,
nav/conic/common.c 1.97.

## Tools changed
* tools/archive/status_void.cocci (new), tools/archive/assert_recover.cocci (new).
* tools/check_rules.py: a statement `if (...)` inside a function with a
  multi-line signature was taken for a definition named `if` ("direct
  recursion in if", display_if.c); C keywords are no longer function names.
* `make format` (Uncrustify) run tree-wide; `make format-check` clean.

## Requested jobs-tasks
* AGC_JOB_RCS_BURN_N85_DISPLAYED -> agc_job_rcs_burn_n85_displayed,
  AGC_JOB_REDO6_7_N85_DISPLAYED -> agc_job_redo6_7_n85_displayed (dispatch).

## Open questions
* CLEANDSP may still put the calling job to sleep (dif_gosleeps) when a
  priority/mark display is up; with no continuation it would wake at
  AGC_JOB_NONE. The two callers propagate any non-OK status.
* The C executive cannot suspend a job inside PRIOCHNG; callers that ignored
  BUSY (dps_burn, ignition, entry, attitude maneuver) keep running until they
  return (timing deviation, as before).
* No ALARM 1301 / ABORT 1302 is raised by code using agc_dp_acos/asin/sqrt
  (not the *_chk forms) -- same as before, now visible in the wrappers.

## Lessons
* A `(void)` cast hides two different things: a status nobody can act on
  (make the callee void) and a failure the AGC acts on (BAILOUT/POODOO never
  return; return the status). Decide per callee by reading its returns --
  most "fallible" wrappers only ever failed on an assertion.
* Voiding cascades: wrapper -> wrapper. Work leaves first, rerun the
  compiler, then the wrappers; let the compiler list the `st = f()` sites.
* Constant-folding the left-over `st = AGC_STATUS_OK` is only safe up to the
  first reference, never across a loop entered after the store unless the
  body stores nothing but OK.
* An early-return recovery inside INHINT must RELINT; an early return also
  exposes -Wmaybe-uninitialized at -O2 (one struct now zero-initialised):
  check with the Makefile flags, not -fsyntax-only.
* GODSPRET/flash displays in C need a continuation job; a NULL continuation
  silently fails an assertion inside the display layer.

## Progress
- DONE Phase A (core/exec + 106 module functions void, cocci + compiler fixes)
- DONE Phase B (all 712 sites; 42 local casts)
- DONE assertion sweep (0 left), function lengths, format, docs

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ A tree-wide pass ((void)-cast calls and assertions (rule 7)): it changed code owned by the translation lanes without changing what the program decides, so it has no program flow of its own. The flows as the code now stands are in the lane sections: [exec-sched](../../docs/modules/exec-sched.md#program-flow-and-decisions), [core-numeric](../../docs/modules/core-numeric.md#program-flow-and-decisions), [state-flags](../../docs/modules/state-flags.md#program-flow-and-decisions), [nav-conic](../../docs/modules/nav-conic.md#program-flow-and-decisions), [nav-integ](../../docs/modules/nav-integ.md#program-flow-and-decisions), [nav-aux](../../docs/modules/nav-aux.md#program-flow-and-decisions), [io-pinball](../../docs/modules/io-pinball.md#program-flow-and-decisions), [io-nouns](../../docs/modules/io-nouns.md#program-flow-and-decisions), [imu](../../docs/modules/imu.md#program-flow-and-decisions), [guid-powered](../../docs/modules/guid-powered.md#program-flow-and-decisions), [guid-landing](../../docs/modules/guid-landing.md#program-flow-and-decisions), [guid-entry](../../docs/modules/guid-entry.md#program-flow-and-decisions), [p37](../../docs/modules/p37.md#program-flow-and-decisions), [dap-lm](../../docs/modules/dap-lm.md#program-flow-and-decisions), [dap-csm](../../docs/modules/dap-csm.md#program-flow-and-decisions), [prog-rendz](../../docs/modules/prog-rendz.md#program-flow-and-decisions), [prog-align](../../docs/modules/prog-align.md#program-flow-and-decisions), [sys-misc](../../docs/modules/sys-misc.md#program-flow-and-decisions), [sim](../../docs/modules/sim.md#program-flow-and-decisions); the system view is [docs/overview/README.md](../../docs/overview/README.md) and its flowcharts.
