# review-nav-integ -- review of the nav-integ lane

## Scope

docs/agents/review.md for lane nav-integ (docs/modules/nav-integ.md):
src/nav/integration/{init,jobs,orbital_integration,flags_csm,setitctr_lm,
svdwn}.c, internal.h, pending.h, include/agc/nav/integration.h,
tests/nav/integration.c, tests/oracle/nav_integ.c.
AGC: INTEGRATION_INITIALIZATION.agc, ORBITAL_INTEGRATION.agc (both ropes),
STATEUP/THISVINT (FRESH_START_AND_RESTART).

Build note: HEAD (4f35a43 checkpoint) does not build (other review lanes
mid-edit, e.g. src/imu/mode.c). Lane work is verified in a scratch tree
from 3f9ede8 (lane files unchanged since) with the lane files overlaid.

## Progress

State at checkpoint (2026-10-11, paused by coordinator): HEAD 0917504
(after format2) builds; lane files are committed and unchanged since;
unit 158 checks 0 fail, oracle 439 checks 0 fail in the real tree
(build/tests/nav/integration, build/tests/oracle/nav_integ).  The scratch
tree is no longer needed.  No file is mid-edit.

| item | status |
|---|---|
| baseline (unit 158 checks, oracle 247, 0 known) | DONE |
| constants vs assembled rope (both ropes) | DONE (N1 fixed) |
| review items Q1-Q4 (oracle / TEPHEM / X2 / state machine) | DONE |
| X2 after A-PCHK origin change | DONE (N2: include/agc/nav/integration.h AGC_INTEG_X2_ORIGEX, init.c a_pchk/atop, svdwn.c) |
| new oracle cases (spec+txt regenerated, tests/oracle/nav_integ.c) | DONE: origchng_timestep/_apchk, integ_earth_long, third_earth_1step/_multi, third_moon_1step/_multi, third_sphere_entry, wmatrix_6x6 |
| AGC_INTEG_MAX_PASSES comment | DONE (header) |
| fidelity: orbital_integration.c | DONE (oracle-verified) |
| fidelity: init.c | DONE, no further discrepancy |
| fidelity: jobs.c | DONE (both ropes), no discrepancy |
| fidelity: svdwn.c, setitctr_lm.c, flags_csm.c | DONE |
| N4 KEPPREP 1-LSB (nav-conic code) | reported, not fixed (request) |
| naming: lane-internal digit-suffixed names | DONE: fbr3->advance_conic_half_step, quality2/3->lunar_j22_term/lunar_c31_term, dow1->body_gravity_gradient, difeq0/01/2->nystrom_start/stage/end, entmid1->midtoav_integrate, integrv2->load_permanent_state, agc_integ_integrv1->agc_integ_integrate_permanent, agc_integ_ckmid2->agc_integ_midtoav_check, statint1_grab/tdec1->state_integration_*, stateup_grab_lm->stateup_lm_state |
| naming: public names (agc_integ_midtoav1/2, agc_integ_intwake0, agc_job_statint1*, AGC_JOB_STATINT1*) | TODO: request only (callers / registry in other lanes) |
| DMP vs DMPR audit (review_items "ALL remaining lanes") | TODO: next item. Inventory: 23 agc_dp_mul (DMPR rounded), 9 agc_dp_mul_tp, 6 agc_dp_sq, 18 agc_vec_scale (VXSC), 2 agc_vec_dot, 1 agc_vec_sq; check each against the listing (DMP -> agc_dp_mul_trunc / _tp), re-tighten oracle tolerances, report count changed |
| shift rounding audit (SR/SRR/VSR, overflow wrap vs saturate) | TODO: 11 agc_integ_vshift, 2 agc_integ_dshift, 5 agc_dp_shr_round, 1 agc_dp_shl_round |
| style debt (856 findings after format2: 349 decl-comment, 106 function-separator, 196 prolog fields, 46 ternary-comment, 19 qualifier-base, 18 long-block-comment, 14 one-declaration, 13 define-comment) | TODO: prolog tool SCRATCH/review-nav-integ/pro.py (draft/apply) and draft *.spec exist but predate format2: regenerate drafts from HEAD before editing/applying; run make format on edited files |
| rule-5 density orbital_integration.c 1.91 | TODO |
| nav-integ.md deviation list update (X2 deviation gone) | DONE |
| make / make test / make check in the real tree | TODO at end (HEAD builds now; lane tests pass) |

## Findings

| id | where | AGC | finding | fix |
|---|---|---|---|---|
| N1 | src/nav/integration/init.c RME_CSM | RME 2DEC 7178165 B-29 (FRESH_START_AND_RESTART p. 210), assembled 00333 01733 | C had 3589082 (01732), 1 LSB (2 m) low in the MIDFLAG threshold | 3589083 |
| N2 | init.c a_pchk/atop, svdwn.c | A-PCHK -> ORIGCHNG ends LXA,2 ORIGEX; ANDOUT -> ATOPCSM -> SVDWN1 runs before RECTOUT's RECTIFY reloads X2 | the C passed PBODY (DEVIATION); in the AGC X2 = 27205 (return address) and SVDWN1's VSL* 0-7,2 / 0,2 shift everything out: RN = VN = 0 (R-OTHER/V-OTHER for ATOPLEM), PIPTIME = TET.  TIMESTEP's DOSWITCH path is unaffected (ACCOMP reloads X2) | AGC_INTEG_X2_ORIGEX passed from a_pchk; svdwn stores zeros; oracle cases origchng_apchk / origchng_timestep |
| N3 | init.c dt2comp | DT/2COMP SL 8 overflow -> GETMAXDT SIGNMPAC -> USEMAXDT | not covered by any case (all runs < 2^20 cs) | confirmed: oracle case integ_earth_long (10986 s, 42 steps) reaches TDEC1 exactly |
| N4 | src/nav/conic/kepler.c agc_conic_kepprep (nav-conic code, KEPPREP is ORBITAL_INTEGRATION p. 1334) | KEPPREP series | XKEPNEW 1 LSB low vs the DV-patched yaAGC for 2 of 4 calls of a LEO step (repro: SCRATCH/review-nav-integ/kep/kep.spec, entry INTPRET Q=KEPPREP fbank 11, inputs from a C trace).  KEPLER converges on it: XKEP 1-2 LSB, VCV 1 KEPLER LSB (512), and a rectification fixes it into RRECT/VRECT: 2.5 km / 3 m/s after 8.8 h LEO | not fixed here (nav-conic files under review); request to review-nav-conic |
| N6 | tests | ACCOMP MIDFLAG path (LSPOS, secondary body + sun GAMCOMP), CHKSWTCH / DOSWITCH / ORIGCHNG with LUNVEL | no oracle case had MIDFLAG set | new cases third_earth_1step/_multi, third_moon_1step/_multi, third_sphere_entry (nav_aux ephemeris pad loads): agree to 1 LSB |
| N7 | tests | W matrix: AMOVED/NEXTCOL/DOW../DIFEQ+2 JSWITCH/RELOADSV, INTEGRV from the permanent state | only a finite-difference unit test | new case wmatrix_6x6 (10 min, 6x6, MODREG 20): W, RRECTCSM, VRECTCSM, TETCSM bit-exact |
| N5 | tests | F1 (yaAGC DV bug) | with the patched probe the multi-step nav-integ cases agree to <= 1280 LSB velocity (all of it N4), positions <= 32 | tolerances unchanged; documented |

## Lessons

## Review items (docs/agents/review_items.md)

* Q1 oracle nav-integ findings: none of F1-F15 is in this lane (F1 is the
  emulator).  Re-run against the DV-patched probe (gen/dvcheck.sh): see
  N5; the residual is N4.
* Q2 TEPHEM dual source: the integration (LUNPOS/LUNVEL, CM) reads only
  the core erasable agc_erase_tephem(); nav-aux's pad-load struct now
  writes and reads TEPHEM through the same accessor (planetary.c
  agc_navaux_set/get_padload).  One source; nothing to change here.
* Q3 X2 after origin change: N2 (modelled, no longer a deviation).
* Q4 bounded state machine vs job sleep: the AGC integration is one job
  that the executive may pre-empt between interpretive instructions
  (higher-priority jobs run meanwhile) and that JOBSLEEPs only in
  INTSTALL.  In C a job body runs to completion, so an integration is
  atomic: nothing else can observe the half-integrated erasables, and
  INTSTALL can only be found busy across job boundaries (a caller that
  slept, or STATEINT vs a program job), which agc_integ_install models
  (BUSY + resume_job; INTWAKE wakes the sleepers).  Results are the
  same; only the order in which other jobs see the CPU differs (as for
  every C job).  The pass bound AGC_INTEG_MAX_PASSES = 60000: a step
  takes 2 passes (TESTLOOP, TIMESTEP; 4 with W: WMATRIX, RELOAD); the
  full TDEC range 2^28 cs at the shortest step (24192 cs, R = earth
  radius) is 11096 steps = 44384 passes.  Exceeding it releases INTSTALL
  and returns NO_CONVERGE (header comment corrected; it claimed
  "2^28 cs / 128 cs-steps x 3", which is 6.3 million).
