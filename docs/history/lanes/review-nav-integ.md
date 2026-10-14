# review-nav-integ -- lane history

Process diary of the review-nav-integ lane, moved verbatim out of [docs/modules/review-nav-integ.md](../../modules/review-nav-integ.md), which keeps the reader-facing explanation. Note dates: 2026-10-10 to 2026-10-14 (first and last commit touching the note).

## Progress

State at checkpoint (2026-10-12, resume): lane files edited for the
DMP/DMPR and shift audits (orbital_integration.c, init.c,
tests/oracle/nav_integration.c); unit 158 / oracle 439 checks pass.  Scratch
harnesses (SCRATCH/review-nav-integ/): try.sh (oracle against the
DV-patched probe, dvrun/), oblbuild.sh / moonbuild.sh / gambuild.sh
(OBLATE earth 3000 cases, OBLATE moon 600, GAMCOMP 600 run alone in the
patched probe and compared word for word).

| item | status |
|---|---|
| baseline (unit 158 checks, oracle 247, 0 known) | DONE |
| constants vs assembled rope (both ropes) | DONE (N1 fixed) |
| review items Q1-Q4 (oracle / TEPHEM / X2 / state machine) | DONE |
| X2 after A-PCHK origin change | DONE (N2: include/agc/nav/integration.h AGC_INTEG_X2_ORIGEX, init.c a_pchk/atop, svdwn.c) |
| new oracle cases (spec+txt regenerated, tests/oracle/nav_integration.c) | DONE: origchng_timestep/_apchk, integ_earth_long, third_earth_1step/_multi, third_moon_1step/_multi, third_sphere_entry, wmatrix_6x6 |
| AGC_INTEG_MAX_PASSES comment | DONE (header) |
| fidelity: orbital_integration.c | DONE (oracle-verified) |
| fidelity: init.c | DONE, no further discrepancy |
| fidelity: jobs.c | DONE (both ropes), no discrepancy |
| fidelity: svdwn.c, setitctr_lm.c, flags_csm.c | DONE |
| N4 KEPPREP 1-LSB (nav-conic code) | reported, not fixed (request) |
| naming: lane-internal digit-suffixed names | DONE: fbr3->advance_conic_half_step, quality2/3->lunar_j22_term/lunar_c31_term, dow1->body_gravity_gradient, difeq0/01/2->nystrom_start/stage/end, entmid1->midtoav_integrate, integrv2->load_permanent_state, agc_integ_integrv1->agc_integ_integrate_permanent, agc_integ_ckmid2->agc_integ_midtoav_check, statint1_grab/tdec1->state_integration_*, stateup_grab_lm->stateup_lm_state |
| naming: public names (agc_integ_midtoav1/2, agc_integ_intwake0, agc_job_statint1*, AGC_JOB_STATINT1*) | request only: every one has callers or registry rows in other lanes (burns, navigation, targeting, exec) |
| DMP vs DMPR audit (review_items "ALL remaining lanes") | DONE (N8): 9 DMP sites were rounded (COMTERM DMP* x2, QUALITY2 DMP, QUALITY3 DMP x5, LM TIMESTEP DOT DMP) + NORFINAL DMP SR1R double-rounded; GAMCOMP / COMTERM / QUALITY1-3 / OBLATE tail now word-level (agc_mpac_*); 45 oracle tolerances tightened |
| shift rounding audit (SR/SRR/VSR, overflow wrap vs saturate) | DONE (N8): SR then SR1R/SR2R and DOT SL1R carried MPAC+2 (GAMCOMP rho/4, DOT, POOHCHK 4DT, NORFINAL); VSR on sign-disagreeing words incl. VRIGHT2; all overflow stores wrap; agc_integ_vshift/dshift (clamping guard, same as agc_vec_shl/agc_dp_shl) deleted |
| status-handling-2 diff (1b72892..a02850c) | DONE (N11): deleted guards confirmed (by construction: internal ctx/stage/pbody/area, NULL of &locals); repaired: INTEG_NEXT_FAIL / ACC_FAIL now carry the callee status (ctx->fail) instead of BAD_ARG; 9 `!AGC_ASSERT(st ...)` callee checks -> plain propagation; 8 tautological asserts deleted (MOONFLAG after set, AVEMIDSW after clear, !rect, rc/d >= 0, stage, TDEC1 >= 0 x3); inlined label helpers restored as functions (USEPIOS, STATEUP, SETCOAST, OTHERS, INTGRATE..ENDSTATE); RELOADSV case block; regrab/agc_integ_vshift/dshift wrappers removed; setmoon, set_lunar_coeffs void |
| GRP2PC rerun safety (exec R11) | DONE (N12): restart left REINTFLG set with INTFLBIT clear -> the rerun segment's INTSTALL slept for ever: install now takes the area over (DEVIATION, unit test); INTSTALL never sleeps a NONE resume (EVA JOBWAKE lead), FULL instead of a silently dropped sleeper; W integration interrupted mid-columns is re-integrated from the moved state (documented DEVIATION); INTWAKEU's 04026 reruns the P27 segment (request) |
| style debt (859 findings at resume) | DONE: 0 in src/nav/integration, integration.h, tests/nav/integration.c, tests/oracle/nav_integration.c (prologs via SCRATCH/review-nav-integ/pro.py + *.spec, decl comments annot.py + *.map, tests main-first via mainfirst.py); make format on lane files only |
| nav-integ.md deviation list update (X2 deviation gone) | DONE |
| make / make test / make check / make scenarios / check_rules | DONE: make OK, make test 34/34 (unit 161, oracle 439 checks), scenarios 8/8, check_rules 0 hard; check-rules and style-report clean; format-check clean on the lane files (fails only on other lanes' in-flight files: src/lm/radar/acquisition.c, src/programs/navigation/common.c, internal.h) |

## Findings

| id | where | AGC | finding | fix |
|---|---|---|---|---|
| N1 | src/nav/integration/init.c RME_CSM | RME 2DEC 7178165 B-29 (FRESH_START_AND_RESTART p. 210), assembled 00333 01733 | C had 3589082 (01732), 1 LSB (2 m) low in the MIDFLAG threshold | 3589083 |
| N2 | init.c a_pchk/atop, svdwn.c | A-PCHK -> ORIGCHNG ends LXA,2 ORIGEX; ANDOUT -> ATOPCSM -> SVDWN1 runs before RECTOUT's RECTIFY reloads X2 | the C passed PBODY (DEVIATION); in the AGC X2 = 27205 (return address) and SVDWN1's VSL* 0-7,2 / 0,2 shift everything out: RN = VN = 0 (R-OTHER/V-OTHER for ATOPLEM), PIPTIME = TET.  TIMESTEP's DOSWITCH path is unaffected (ACCOMP reloads X2) | AGC_INTEG_X2_ORIGEX passed from a_pchk; svdwn stores zeros; oracle cases origchng_apchk / origchng_timestep |
| N3 | init.c dt2comp | DT/2COMP SL 8 overflow -> GETMAXDT SIGNMPAC -> USEMAXDT | not covered by any case (all runs < 2^20 cs) | confirmed: oracle case integ_earth_long (10986 s, 42 steps) reaches TDEC1 exactly |
| N4 | src/nav/conic/kepler.c agc_conic_kepprep (nav-conic code, KEPPREP is ORBITAL_INTEGRATION p. 1334) | KEPPREP series | XKEPNEW 1 LSB low vs the DV-patched yaAGC for 2 of 4 calls of a LEO step (repro: SCRATCH/review-nav-integ/kep/kep.spec, entry INTPRET Q=KEPPREP fbank 11, inputs from a C trace).  KEPLER converges on it: XKEP 1-2 LSB, VCV 1 KEPLER LSB (512), and a rectification fixes it into RRECT/VRECT: 2.5 km / 3 m/s after 8.8 h LEO | not fixed here (nav-conic files under review); request to review-nav-conic |
| N6 | tests | ACCOMP MIDFLAG path (LSPOS, secondary body + sun GAMCOMP), CHKSWTCH / DOSWITCH / ORIGCHNG with LUNVEL | no oracle case had MIDFLAG set | new cases third_earth_1step/_multi, third_moon_1step/_multi, third_sphere_entry (nav_aux ephemeris pad loads): agree to 1 LSB |
| N7 | tests | W matrix: AMOVED/NEXTCOL/DOW../DIFEQ+2 JSWITCH/RELOADSV, INTEGRV from the permanent state | only a finite-difference unit test | new case wmatrix_6x6 (10 min, 6x6, MODREG 20): W, RRECTCSM, VRECTCSM, TETCSM bit-exact |
| N8 | orbital_integration.c GAMCOMP, COMTERM, QUALITY1-3, OBLATE tail; init.c NORFINAL, POOHCHK | DMP (DMPSUB, no round) at COMTERM DMP* J4REQ/J3 (x2), QUALITY2 DMP 5/8, QUALITY3 DMP 5/8 / DMP DMP URPV 5/8 / DMP DMP URPV+4 5/8; LM TIMESTEP DOT DMP; SR*/SR4 then SR1R/SR2R and DOT then SL1R round the bits left in MPAC+2; DSU/BDSU/DAD results whose words disagree in sign feed DMPR, VXSC (DVXSC DMPSUB) and VSR (MPACSRND, VRIGHT2) word by word; BDDV* J2REQSQ carries MPAC+2 under the dividend | C used agc_dp_mul (DMPR) at all 9 DMP sites, rounded the DP after an SR, and did the f(q), Legendre and J22/C31 chains on values: OBLATE alone 16/200 TVEC and FV off by 1-8 LSB, GAMCOMP alone 14/600, moon OBLATE 59/600 | word-level chains (agc_mpac_*, local vec_words_t: VXSC, VAD, VSL/VSR incl. VRIGHT2); OBLATE earth 3000/3000, GAMCOMP 600/600 exact against the probe; moon OBLATE FV exact except where RP-TO-R / R-TO-RP (nav-aux, value API) differ (N10) |
| N9 | tests/oracle/nav_integration.c | | against the DV-patched probe the lane now agrees exactly except VATT 1 LSB (integ_moon_1step, third_moon_1step, integ_earth_long) and FV 288 LSB at 3 h: TNUV is stored as the Nystrom VAD leaves its words and RECTIFY / RECTOUT VSR round those words; the C keeps TNUV as a value in agc_integ_sv_t (shared with nav lanes) | documented; tolerances tightened to the committed-oracle (F1) maxima: 45 changed |
| N10 | nav-aux RP-TO-R / R-TO-RP | | moon OBLATE FV 1-2 LSB in ~8% of cases: the rotation takes values, the AGC's VXM/MXV sees the words QUALITY3 leaves | request to nav-aux (word-level input) |
| N5 | tests | F1 (yaAGC DV bug) | against the DV-patched probe (SCRATCH/review-nav-integ/dvrun) every case now agrees except N9 (earth_multi and moon_multi exact; before N8 they were off by up to 1280 LSB, which had been put down to N4) | tolerances set to the committed-oracle (F1) maxima |
| N11 | status-handling-2 diff | | INTEG_NEXT_FAIL / ACC_FAIL mapped every callee failure to BAD_ARG (driver); 9 `!AGC_ASSERT(st ...)` callee checks (some `return BAD_ARG`, some ignoring POODOO/BUSY from INTEGRV in STATEUP/SETCOAST/INT-W); tautological asserts; label helpers (USEPIOS, STATEUP, SETCOAST, OTHERS, INTGRATE) inlined into nested blocks; RELOADSV case block | ctx->fail carries the callee status to agc_integ_run; plain propagation; asserts deleted; helpers restored |
| N12 | init.c agc_integ_install | GOPROG2 clears INTFLBIT, REINTFLG stays; the AGC resumes the integration after its PHASCHNG 04022 | with exec R11 (rerun the segment from its top) the rerun's INTSTALL found REINTFLG set and slept for ever (no INTWAKE comes): STATEINT died after a restart; a NONE resume was put to sleep unwakeable (EVA JOBWAKE lead); a full sleeper list silently dropped the job | take the area over when INTFLBIT is clear (DEVIATION, unit test); NONE: BUSY without sleeping; full: FULL |

## Lessons

* DDV ignores MPAC+2 (the dividend goes through DXCH MPAC into A, L) but
  SRR / SLR / ROUND use it: an SR or DOT followed by a rounding shift
  rounds bits a DP value no longer has.  Check what the NEXT instruction
  reads, not only DMP vs DMPR.
* A 1-LSB-per-call difference that survives the DMP/DMPR fix is usually a
  word split: DVXSC (VXSC with the scalar in MPAC) and VSR multiply /
  shift the words as held, and DSU/BDSU/DAD leave sign-disagreeing words.
  Converting to a value and back hides it.
* Run the routine alone in the probe on thousands of random inputs
  (OBLATE: entry INTPRET, Q = OBLATE, DIFEQCNT -24, DIM0FLAG, stop at
  PHASCHNG; GAMCOMP: QPRET = ABTCONIC, stop at POODOO) instead of
  bisecting a 3-hour integration: the long run's 9-vs-1 LSB agreement
  favoured the wrong (rounded) DMP by accident until the word-level VXSC
  was in place.
* Use the DV-patched probe (dvcheck.sh) as the reference when tightening:
  F1 swamps the integration cases in the committed oracle.
* check_style reads `integ_case_t` as `int` + `eg_case_t`: avoid type
  names starting with a C keyword.

## Requests (other lanes)

* nav-aux: RP-TO-R / R-TO-RP / MOONMX take values; the AGC's VXM / MXV see
  the words QUALITY3 leaves (N10, moon OBLATE FV 1-2 LSB in ~8% of cases):
  a word-level input (agc_tp_words_t components) would close it.
* nav-conic: N4 (KEPPREP XKEPNEW 1 LSB, repro SCRATCH/review-nav-integ/kep).
* core-numeric: a public word-level vector API (VXSC with MPAC scalar,
  VAD, VSL, VSR incl. VRIGHT2) would replace the lane-local vec_words_t
  helpers in orbital_integration.c.
* prog-align (tests/programs/alignment.c checks agc_integ_svdwn's status) and
  prog-rendz (src/programs/navigation/common.c checks agc_integ_rectify's):
  both cannot fail; make them void once the callers drop the check.
* io/uplink (P27): INTWAKEU ends with PHASCHNG 04026 (group 6), which now
  reruns the calling P27 job segment from its top after a restart; that
  segment must be safe to rerun with UPSVFLAG = 0 and INTSTALL released.
* every lane whose job calls the integration (navigation, targeting, burn
  MIDTOAV/AVETOMID): a restart during the integration reruns that job
  segment from its top (exec R11); INTSTALL now lets it in (N12).
* EVA lead orbital_integration.c:430 (XSUM signed overflow): XSUM is
  S1 - n2 + n3 + S2 + n4 + n1, NORM counts <= 42 and ASCALE words >= -12:
  |XSUM| < 200, no host UB (now gamcomp's xsum).
* Public names agc_integ_midtoav1/2, agc_integ_intwake0, agc_job_statint1*,
  AGC_JOB_STATINT1*: renames need the callers / registry (other lanes).

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

