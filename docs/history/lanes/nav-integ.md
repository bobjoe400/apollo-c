# nav-integ -- lane history

Process diary of the nav-integ lane, moved verbatim out of [docs/modules/nav-integ.md](../../modules/nav-integ.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-14 (first and last commit touching the note).

## Progress

* design: DONE (this note)
* agc_integ.h: DONE include/agc/nav/integration.h
* internal.h / pending.h: DONE src/nav/
* init.c: DONE (compiles) -- INTSTALL/ERASTAL1/2/OKTOGRAB agc_integ_install, INTWAKE agc_integ_intwake, INTWAKE0/1 agc_integ_intwake0, ERASWAK1/2 agc_integ_eraswake, WAKE wake, CSMPREC/LEMPREC/CSMCONIC/LEMCONIC special_entry, INTEGRVS agc_integ_integrvs, INTEGRV agc_integ_integrv, INTEGRV1 agc_integ_integrv1, INTEGRV2 integrv2, ALOADED aloaded, ATOPCSM/ATOPLEM atop, PTOACSM/PTOALEM agc_integ_ptoacsm/ptoalem, USEPIOS usepios, TESTLOOP testloop, NORFINAL norfinal, DT/2COMP..USEMAXDT dt2comp, POOHCHK poohchk, A-PCHK/ANDOUT/PHEXIT a_pchk, RECTOUT rectout, INTEXIT intexit, RVCON rvcon, ENDSTATE W part wmatrix, RELOADSV reload, driver agc_integ_run, SETIFLGS, THISVINT, GRP2PC, permanent SV agc_integ_perm_get/set
* jobs.c: DONE (compiles) -- STATEINT agc_task_stateint, STATINT1 agc_job_statint1(+_stall), STATEUP stateup/stateup_lm(+agc_job_stateup_stall), ENDINT endint, NOINT (in statint1), AVETOMID agc_integ_avetomid (INT/W int_w, SETCOAST setcoast_csm, OTHERS others_lm, MOVATHIS move_this), MIDTOAV1/2 agc_integ_midtoav1/2, ENTMID1 entmid1, NOTIME notime, CKMID2/MID2 agc_integ_ckmid2, TIMEINC timeinc, INTWAKEU agc_integ_intwakeu
* orbital_integration.c: DONE (compiles) -- KEPPREP wrapper agc_integ_kepprep, FBR3 fbr3, GOBAQUE gobaque, GAMCOMP gamcomp/gamcomp_rho, ACCOMP accomp/third_body, OBLATE oblate/moon_axes/legendre_terms/comterm_tvec, QUALITY1-3 quality2/quality3, DOW.. dow/dow1, DIFEQ0/+0/+1/+2/DIFEQCOM difeq0/difeq01/difeq2, ENDSTATE state_step, AMOVED/NEXTCOL/WMATEND agc_integ_wcolumns/wmatend, RECTEST rectest, TIMESTEP agc_integ_timestep (+ LM lm_sphere), CHKSWTCH agc_integ_chkswtch, ORIGCHNG agc_integ_origchng, RECTIFY/MINIRECT agc_integ_rectify/minirect, SETMOON/CLRMOON agc_integ_setmoon
* flags_csm.c / setitctr_lm.c: DONE (SETITCTR agc_integ_setitctr)
* assertion density top-up: DONE (check_rules: no warnings on the nav-integ files)
* scratch run of the test: DONE (see Tests written)
* tests/nav/integration.c: DONE (12 tests, 177 checks; all pass against the current core/exec/nav objects with a scratch link shim for dispatch/svdwn/dsky)

## Requested core additions

* `void agc_erase_set_trkmkcnt(agc_sp_t v);` (TRKMKCNT, B(1)PRM, owner
  prog-rendz) -- AVETOMID zeroes it. Meanwhile `trkmkcnt_pending` static in
  jobs.c.
* Layering: ATOPCSM/ATOPLEM call `agc_align_svdwn()` (SVDWN1/SVDWN2,
  agc_align.h, src/programs/alignment/locsam.c), i.e. nav calls programs.
  Request: move SVDWN to nav (e.g. `agc_integ_svdwn`) or core.
* agc_state.h comment fix: RN/VN are always 2^29 m / 2^7 m/cs (AVERAGEG);
  SETCOAST shifts them left 2 for the moon scale (VSR* 0,2 with X2 = 2).
* agc_nav.h: `agc_conic_kepprep()` must take the current conic point
  (RCV, VCV at TC) in in->rrect/in->vrect -- KEPPREP reads RCV/VCV, not
  RRECT/VRECT; agc_integ_kepprep() passes them that way (works with the
  current kepler.c: all integration tests pass).
* E3J22R2M / E32C31RM (SP pad loads at FANG+2/+3, CM P40 pad-load area) are
  module statics here with `agc_integ_set_lunar_coeffs()`; if the uplink /
  pad-load code needs them, promote to shared accessors.

## Requested jobs-tasks

Placeholders in src/nav/integration/pending.h (range 0x0F):

| id | AGC | body | how started |
|---|---|---|---|
| AGC_JOB_STATINT1 | STATINT1 | agc_job_statint1 | FINDVAC PRIO5 from task STATEINT |
| AGC_JOB_STATINT1_STALL | STATINT1 at INTSTALL +1 | agc_job_statint1_stall | JOBWAKE (INTWAKE) |
| AGC_JOB_STATEUP_STALL | STATEUP 2nd INTSTALL +1 | agc_job_stateup_stall | JOBWAKE (INTWAKE) |
| AGC_TASK_STATEINT | STATEINT | agc_task_stateint | LONGCALL 600SECS from ENDINT (and fresh start) |

Callers of the CSMPREC/... entrances, AVETOMID and MIDTOAV1/2 pass their own
continuation job (resume_job) for the INTSTALL sleep.

## Oracle cases

Proposed for tools/oracle (Comanche055 unless noted; inputs poked into the
erasables, interpretive entry through INTPRET):
1. RECTIFY with nonzero TDELTAV/TNUV, PBODY 0 and 2: RRECT VRECT RCV VCV.
2. GAMCOMP primary term (X1 = X2 = 0) for a LEO RCV and a 1000 m ALPHAV:
   FV bit pattern (checks ASCALE and the NORM bookkeeping).
3. GAMCOMP sun term (X1 = 8, X2 = 4) and moon term (X1 = 4, X2 = 2).
4. OBLATE earth (COSPHIE) at 45 deg latitude: FV.
5. OBLATE moon with QUALITY1-3 (needs MOONMX pad loads): FV.
6. One full TIMESTEP (INTGRATE..ENDSTATE) from a fixed state: TDELTAV,
   TNUV, RCV, VCV, TET, TC, XKEP (KEPLER included).
7. TESTLOOP/NORFINAL/DT/2COMP: DT/2 and 12D for r = 6678 km, 60000 km,
   1850 km (moon).
8. DOW.. for one W column: FV.
9. Luminary099: CSMPREC over 10 minutes end to end (RATT VATT TAT).

## Open questions

* RN/VN scale comment in agc_state.h (see above).
* The integration is synchronous in C: a caller that holds INTSTALL across
  a JOBSLEEP elsewhere would block STATEINT exactly as the AGC; no
  pre-emption inside an integration is modelled (CHANG1/CHANG2 are not
  called inside it).
* PHASCHNG codes are passed as the full octal word (04022, 00052, 020032,
  00002, 04026) to agc_phase_change(group, code); type C 04022 restarts
  "at the return address" which the C model cannot resume mid-integration.
* VPROJ argument order assumed agc_vec_proj(ip, mpac, x) = (mpac.x) x
  (DOW..1 passes ALPHAV, unit R); the W test confirms the result.

