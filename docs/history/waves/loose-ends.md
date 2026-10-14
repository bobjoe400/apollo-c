# loose-ends -- the open residuals in docs/agents/review_items.md

## Scope
Ten items routed to this lane (review_items.md "Open residuals after
follow-up-done-lanes", "From fix-b6 and review-sim", "From the document
wave's first pass", "From prolog-consistency" item 1, and the 6
`make prolog-report` findings in src/exec and src/csm/entry). Constraint: an EVA run is in progress on a frozen
snapshot, so no AGC_ASSERT guard was added or deleted (per-file counts
unchanged: waitlist.c 24, idle.c 9, clock.c 1, pad.c 0, reentry.c 8,
executive.c 17, cm_body_attitude.c 8, dap.c 20, navigation/common.c 5).

## Progress
- DONE 1 entry DOWNCNTL on stored words (F3): src/csm/entry/reentry.c,
  include/agc/csm/entry.h; oracle entry L/D and L/D1 tolerance 9 / 8 -> 0
- DONE 2 waitlist delay bound: DEVIATION documented, test
- DONE 3 pad keyword `sps_impulse`; scenario
  sim/scenarios/external_delta_v_impulsive_burn_csm.txt flies the
  impulsive path (42 checks, 0 failed)
- DONE 4 executive 64-jobs-per-call bound: rationale and sizing in the
  clock_exec prolog; no behaviour change
- DONE 5 review-prog-align.md item 2b row -> DONE (resume 3 (a))
- DONE 6 exec_cp_cleanup_lm: false alarm, body present
- DONE 7 agc_power_on header comment
- DONE 8 agc_waitlist_time3 / agc_waitlist_entry comments
- DONE 9 GET+MGA: second UNIT on the LM (src/programs/navigation/common.c)
- DONE 10 prolog-report 6 -> 0 (executive.c PRIOCHNG, idle.c power-on,
  cm_body_attitude.c CM/POSE and SERVNOUT, dap.c roll phase plane)

## Findings and decisions

| # | where | AGC reference | finding | action |
|---|---|---|---|---|
| 1 | reentry.c targ_geteta, step_downcntl; entry.h v1 | GETETA (REENTRY_CONTROL p.851), HUNTEST V1 DAD/V1LEAD DSU (p.855), DOWNCNTL / CONSTD1 (pp.867-868) | Measured at tolerance 0: D was 1 LSB off in the huntest and predict3 passes (GETETA's DMP D DAD D on values), which made CONSTD1's L/D 8 off; the upcontrol passes' L/D 1 and 9 came from DOWNCNTL on values. F3's guess (D's words) was not the cause: the word that matters is V1 as HUNTEST stored it (DOWNCNTL with V1 canonicalised: 256 / 265 off; with V1 as stored: exact). D, RDOT and A0 words change nothing in these cases. | GETETA on the MPAC words (RDOT and D as DSTORE stored them, passed in); DSTORE's ABVAL DMP SL1 on words; agc_entry_state_t.v1 is agc_dp_words_t; DOWNCNTL on agc_mpac_* (DREF and the pushed K2D term as words). oracle_entry 521 checks, 0 failures with L/D and L/D1 at 0. D stays a value (DEVIATION in entry.h: src/exec/nouns.c points the D noun at it as agc_dp_t); the D tolerance 160 stays (yaAGC DV bug in the initroll / kep2 passes, 84 / 156 LSB). |
| 2 | src/exec/waitlist.c agc_waitlist_call | WAITLIST C p.1221-1226 / L p.1117-1122: WARNINGS 1) 1 <= C(A) <= 16250D; run-time test only BZMF WATLST0- (C) / BZMF WAITPOOH in WAIT2 (L) | The AGC never tests the maximum; above 16250 its WAIT2 overflow test misreads TIME3 (< 134) and places the next task 163.84 s late, and above 16383 the delay is a negative word (POODOO 1204). The C check guards no undefined behaviour. | Kept as a DEVIATION in the prolog (the guard could not be deleted during the EVA run anyway): refusing makes the misschedule visible to the caller, who propagates it. test_waitlist_delay_bound (16250 OK, 16251 BAD_ARG, list/TIME3/FAILREG untouched). |
| 3 | src/sim/pad.c | P40 PAD LOADS (ERASABLE_ASSIGNMENTS C p.87), EMDOT (C p.68), S40.13 (P40-P47 C pp.726-727) | -- | `sps_impulse K1 K2 K3 F MDOT` (N s, N s, N, N, kg/s), CSM only, range-checked to each erasable's scale. EK3VAL is one word followed by FANG and DDV K3VAL reads them as one DP, so the C's DP ek3val is stored as (EK3VAL, FANG). New scenario: TGO = (m v + K2)/K3 = 0.75 s, engine on 480.00 s, off 480.75 s; V16N85 residuals 0. The pad values are representative (K1 = K3 x 1 s - K2), not the flown ones. |
| 4 | src/sim/clock.c SIM_JOBS_PER_CALL | EXECUTIVE C pp.1208-1220; CHECKG1 (IMU_PERFORMANCE_TEST_2 L p.378) | All 1,817 hits in systems_test_lm are one job: CHECKG1 polling PIPAX for the first and the last pulse (cs 15746-16004, 45195-46004), which in the AGC keeps the processor 100% busy too. 64 per call exceeds the AGC's rate: an empty job costs > 50 MCT (NOVAC core-set scan, ENDJOB1/EJSCAN over 7 core sets, CHANJOB) of the 853 MCT in a cs, so < 17 jobs/cs, and the next call is at most one scaler count (1/3200 s) later. | Not too low; documented in the clock_exec prolog. No change. |
| 6 | src/exec/change_program/lm.c:159-166 | R00 after INTSTALL, FRESH_START_AND_RESTART L p.229 | Body present and matches the rope line by line: DOWNFLAG 3AXISFLG, CAF LRBYBIT / TS FLGWRD11, DOWNFLAG R04FLAG, MUNFLAG, XOVINFLG (false alarm in the document pass). | No code change; test_change_program_cleanup_lm added in tests/exec/exec.c (V37 CANV37 -> POOH on the LM clears the four flags and sets FLGWRD11 = LRBYBIT exactly). |
| 7 | src/exec/change_program/idle.c agc_power_on | -- | header named agc_fresh_dofstart() | now names main.c's sequence: rope, pad, channels, agc_rupt_go() (GOJAM -> GOPROG -> DOFSTART or restart) |
| 8 | src/exec/waitlist.c | -- | TIME3 "0 on a failed precondition"; ENTRY "BAD_ARG on a bad index or NULL pointer": neither exists | RETURNS say no failure case; ENTRY documents the caller's contract (host inspection) |
| 9 | src/programs/navigation/common.c agc_rendz_get_mga | GET+MGA, P34-35_P74-75.agc C line 1336 / L line 1406 | Luminary has `VLOAD UNIT` then `UNIT` (the half-unit vector unitized again), Comanche one UNIT; the C did one for both. No oracle case exists for GET+MGA. | second rz_unit when agc_state_vehicle() is the LM; test_get_mga_per_rope in tests/programs/rendezvous.c (64 directions, each vehicle matches its own sequence, and the second UNIT moves +MGA for at least one). |
| 10 | prologs | -- | 6 prolog-report findings in this lane's directories (two of them, idle.c's, made by this lane's own wording: the tool reads `name()` in a prolog as a call) | reworded; RETURNS of agc_entry_cm_pose_triad and agc_entry_dap_roll_phase_plane say always OK; entry_servnout ARGUMENTS none, RETURNS names the yield / SERVEXIT status and the (cannot-fail, kept for EVA) NEWJOB assertion |

## Tests written
- tests/exec/exec.c: test_waitlist_delay_bound, test_change_program_cleanup_lm
- tests/oracle/entry.c: L/D, L/D1 tolerance 0
- tests/programs/rendezvous.c: test_get_mga_per_rope
- sim/scenarios/external_delta_v_impulsive_burn_csm.txt (in `make test`)

## For the lead (docs this lane may not edit)
- docs/modules/oracle-cases.md entry paragraph still says "L/D 9, L/D1 8
  (DOWNCNTL: DREF words depend on D as stored)": now 0 / 0, and the
  cause was V1's words.
- docs/modules/sim.md / fix-b6.md "a pad keyword ... would let" lines:
  done (`sps_impulse`, new scenario).
- sim/sim_client.py docstring lists sps_impulse (one line).
- EVA leads after the run: entry_servnout's `AGC_ASSERT(nj <= 0)` follows
  `if (nj > 0) return` (dead); agc_waitlist_entry and the two always-OK
  entry functions could be void. Not changed (guards frozen).
- docs/MODULE_STATUS.md row not written (outside the docs this lane may edit).

## Lessons
- Measure which stored word matters before converting erasables: the F3
  hypothesis (D's split) was wrong; toggling each input of the chain
  between words and canonical form (env switches in a scratch copy)
  found V1 in three runs.
- A busy-bound counter in a harness is a CPU-saturation meter, not a
  sizing error, when the job that hits it is an AGC polling loop.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ Each item changed or documented one decision; the full flows are in the owning lanes:

| item | decision as the code now takes it | lane section |
|---|---|---|
| 1 | `src/csm/entry/entry_targeting.c:targ_geteta`, `entry_upcontrol.c:step_downcntl` (then reentry.c): GETETA and DOWNCNTL on the MPAC words (V1 as HUNTEST stored it) | [guid-entry](../../modules/guid-entry.md#program-flow-and-decisions) |
| 2 | `src/exec/waitlist.c:agc_waitlist_call`: a delay above 16250 cs is BAD_ARG (DEVIATION; the AGC would misplace it) | [exec-sched](../../modules/exec-sched.md#program-flow-and-decisions) |
| 3 | `src/sim/pad.c`: `sps_impulse` pad keyword, range-checked to each erasable's scale | [sim](../../modules/sim.md#program-flow-and-decisions) |
| 4 | `src/sim/clock.c`: at most SIM_JOBS_PER_CALL jobs per executive call | [sim](../../modules/sim.md#program-flow-and-decisions) |
| 6 | `src/exec/change_program/lm.c`: R00 after INTSTALL clears 3AXISFLG, R04FLAG, MUNFLAG, XOVINFLG and sets FLGWRD11 = LRBYBIT | [exec-sched](../../modules/exec-sched.md#program-flow-and-decisions) |
| 9 | `src/programs/navigation/rendezvous_subroutines.c:agc_rendz_get_mga` (then common.c): a second UNIT on the LM (Luminary GET+MGA) | [prog-rendz](../../modules/prog-rendz.md#program-flow-and-decisions) |
