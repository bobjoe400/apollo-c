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

| item | status |
|---|---|
| baseline (unit 158 checks, oracle 247, 0 known) | DONE |
| review items: oracle findings / TEPHEM / X2-after-ORIGCHNG / state machine vs sleep | DONE (N2-N5, Q1-Q4) |
| constants vs assembled rope (both ropes) | DONE (N1) |
| fidelity: orbital_integration.c | DONE (oracle: one/multi step earth+moon, third bodies, sphere entry, W, RECTIFY, long run; constants) |
| fidelity: init.c | DONE (entries, TESTLOOP/NORFINAL/DT2COMP/POOHCHK, A-PCHK, RECTOUT, INTEXIT, RVCON, INTSTALL/INTWAKE, USEPIOS) |
| fidelity: jobs.c | DONE (STATEINT/STATINT1/NOINT/STATEUP both ropes, AVETOMID CM+LM, MIDTOAV1/2, CKMID2/MID2/TIMEINC/NOTIME, INTWAKEU) -- no discrepancy |
| fidelity: svdwn.c, setitctr_lm.c, flags_csm.c | DONE |
| naming / clarity | TODO |
| style debt (827 findings at start) | TODO |
| rule-5 density (orbital_integration.c 1.91) | TODO |

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
