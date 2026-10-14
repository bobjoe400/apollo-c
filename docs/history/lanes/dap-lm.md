# dap-lm -- lane history

Process diary of the dap-lm lane, moved verbatim out of [docs/modules/dap-lm.md](../../modules/dap-lm.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-14 (first and last commit touching the note).

## Progress

Session 2 (resumed): only lm_dap_pending.h existed. Files planned (all src/lm/dap/):
lm_dap_internal.h (state struct = DAP erasables, word primitives), arith.c (MP/DV/DAS/MSU/
1STOTWOS/OVERSUB/DVOVSUB), state.c (state, accessors, fresh start/restart), dapif.c,
dapidler.c, needles.c, rate.c, paxis.c, pmanual.c, qraxis.c, qrmanual.c, qrjets.c,
t6rupt.c, tjetlaw.c, gts.c, accs.c, accont.c, rcsmon.c, spsrcs.c.

* DONE lm_dap_internal.h, lm_dap_pending.h (updated: flags now real), arith.c, state.c
  (accessors, agc_lm_dap_fresh_start/restart), dapif.c (SETMAXDB..ALLCOAST, CALLACCS),
  dapidler.c (CHEKBITS, DAPIDLER, STARTDAP, MOREIDLE, SHUTDOWN, t5rupt), needles.c
* DONE rate.c (RATELOOP/ROTORQUE, BACKP, offset estimator), paxis.c (PAXIS, PRETIMCK, CHKVISFZ,
  PJETSLEC, SELECTP, ALTSYST, CALCPERR, PURGENCY, JETSOFF) -- session 3 fixed chkvisfz prototype
* DONE pmanual.c (state logic, min impulse, DETENTCK, RATERROR, PEGI, JUSTOUT, RATEDONE, ZEROENBL)
* DONE tjetlaw.c (TJETLAW all zones + RUFLAW1-3), t6rupt.c (JTLST, T6JOB, T6JOBCHK, DOT6RUPT, WRITEP/U/V,
  TIME6 DINC model)
* Session 3: NAMING rule applied (1/ACCS -> jet_accel_update etc., see docs/history/RENAMES.md)
* DONE qraxis.c (QRAXIS, CALLQERR, TRYGTS, RCS, SENSEGET, QRCONTRL, QERRCALC, STILLRCS), qrmanual.c
  (min impulse, CHEKSTIK, RHCACTIV, ENTERUV), qrjets.c (U,V loop bodies, AFTERTJ, DOSKIP, SELCTSUB, XTRANS,
  ROT-TOUV, CLOSEOUT)
* DONE gts.c (GTS law, GTSQRT, ROOTCYCL, ACDT+C12, TIMEGMBL)
* Session 4 (after naming reset / restructure / header move):
  * DONE accs.c (1/ACCS, 1/ACCJOB, 1/ACCSET, F(MASS), MASSFIX, COMMEQS, JACCUV, LRESC,
    DOCKED, SPSCONT, DOCKTEST, 1/ACCRET, 1/ACCFIX helper)
  * DONE accont.c (1/ACCONT, GETAOSUV, DOPAXIS, BOTHAXES, BIGAOS, NOTMUCH, SKIPDB1/2,
    ACCTHERE, 1/ANET-, 1/NETMIN, DO1/NET+, DOACCFUN, FIXMIN, STMIN-, FAIL-, DBFUN, STORV,
    DRFDB, INVERT)
  * DONE rcs/rcsmon.c (RCSMON), rcs/spsrcs.c (SPSRCS, POSTHRST, SPSSTART, SETCTR, ZAPTJ)
  * DONE tests/lm/dap.c (120 checks)
  * FIXED rcs/qrjets.c ROT-TOUV: U and V were swapped (AGC: U = -COEFFQ q + COEFFR r in A,
    V = COEFFQ q + COEFFR r in L).
  * Rule 7 (no (void) on agc_ calls): lm_dap_alarm() wrapper (dapif.c) for TC ALARM in a DAP
    pass; lm_dap_chekbits(bool *on) and lm_dap_chkvisfz(bool *policy) now return status;
    PAXIS propagates TIME5 / ACDT+C12 / BAILOUT statuses; T6JOBCHK returns agc_ccshole();
    DAPIDLER propagates a failed NOVAC (1/ACCSET).
* All routines in scope are translated. Nothing left TODO.

## Requested core additions

* LM DAP jobs need the exec to call agc_job_jet_accel_update / agc_job_jet_accel_reset for
  AGC_JOB_JET_ACCEL_UPDATE / _RESET (dispatch.c), and T4RUPT (src/io/service/t4rupt_lm.c)
  to call agc_lm_dap_rcsmon() every 480 ms.
* Still unpublished (declared in src/lm/dap/pending.h): agc_erase_m11/m21/m31/m22/m32 are
  defined in t4rupt_lm.c but not declared in a public header; agc_erase_p_rhcctr/q_/r_ and
  set_ have no definition anywhere yet.

* agc_flags_lm.h: the DAPBOOLS flagword is FLGWRD13 (flags 195-209). AGC_FLAGWORDS must be >= 14.
  Flags used: AGC_FLAG_PULSEFLG, USEQRFLG, CSMDKFLG, OURRCFLG, ACC4_2FL, AORBTFLG, XOVINFLG,
  DRIFTDFL, ULLAGFLG, AORBSFLG, DBSELFLG, ACCOKFLG (flagword 13); AGC_FLAG_NEEDLFLG (word 0 bit 4),
  AGC_FLAG_SNUFFER (word 5 bit 13), AGC_FLAG_APSFLAG (word 10 bit 13).
* Erasable accessors read by the DAP (owned elsewhere), exact declarations in src/lm/dap/pending.h:
  agc_erase_cdux/cduy/cduz (uint16_t, 2's complement at pi), agc_erase_cphi/ctheta/cpsi,
  agc_erase_m11/m21/m31/m22/m32 (agc_sp_t at 1, written by T4RUPT), agc_erase_mass/set_mass (agc_dp_t B16 kg),
  agc_erase_lemmass/set_lemmass, agc_erase_csmmass (agc_sp_t B16 kg), agc_erase_abdelv (agc_dp_t B13 cm/s2),
  agc_erase_imodes33, agc_erase_p_rhcctr/q_rhcctr/r_rhcctr + set_ (hand controller counters),
  agc_erase_cduxcmd/cduycmd/cduzcmd + set_ (IMU error counter outputs).
* TIME5/TIME6 hardware counters: the DAP keeps TIME6 privately (lm_dap state) and exposes
  agc_lm_dap_time6_count(n) for the sim harness to decrement it; the sim must call
  agc_lm_dap_t5rupt() every 100 ms and agc_lm_dap_t6rupt() when agc_lm_dap_t6_pending() is true.

## Requested jobs-tasks

* AGC_JOB_JET_ACCEL_UPDATE (PRIO27, NOVAC) -> agc_job_jet_accel_update()
* AGC_JOB_JET_ACCEL_RESET (PRIO27, NOVAC) -> agc_job_jet_accel_reset()
* Interrupts (not waitlist tasks): T5RUPT -> agc_lm_dap_t5rupt(); T6RUPT -> agc_lm_dap_t6rupt();
  T4RUPT must call agc_lm_dap_rcsmon() every 480 ms (RCSMONIT).

## Open questions

* DOCKED inertia loop (AOSTASK_AND_AOSJOB p.1495): CCS COEFCTR with COEFCTR = +0 skips to the
  TORQCONS 2DEC (high word 00000). Either the transcription differs from the flown rope or
  the docked 1/ACCS path executes TC A here. Worth checking against the page images / yaAGC.
* BIGAOS (p.1499): the code stores the ADS sum (2 DB) in DB4(3), the comment says 1 DB.
  Code followed.

## Lessons

* Read ROT-TOUV-style "result in A, second result in L" routines against their comment
  ("U-component in A, V in L"); the sum/difference is easy to assign to the wrong axis, and
  nothing downstream crashes, the jets just fire on the wrong axis.
* Ones'-complement -0 is load-bearing in control flow: a loop index counted down by
  "CS TWO; ADS MPAC" reaches -0, not +0, and a later CCS on it takes the fourth branch.
  INDEX with -0 adds nothing (end-around carry), so the table lookups still use row 0.
* DVOVSUB and DV callers often pass the full A,L pair left by an earlier MP; L also survives
  CCS/XCH/AD, so a DV can silently include an old remainder (COMMEQS divides with the
  remainder of the last F(MASS) DV in L). Model DP pairs explicitly.
* ADS leaves the sum in A as well as in memory; a following TS stores the sum, not the
  addend.
* For TC ALARM inside a DAP pass, wrap agc_alarm (returns AGC_STATUS_ALARM) so the pass
  continues while a genuine failure (bad code) is still propagated (rule 7).
* SPSRCS keeps firing until the rate passes the 0.1 deg/s target on the far side (L = EDOT
  for negative firing), not until zero rate.

