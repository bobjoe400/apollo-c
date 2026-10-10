# dap-lm -- Luminary 099 LM digital autopilot (RCS DAP, trim gimbal, 1/ACCS)

## Scope

Luminary099 files (pages 1403-1506 plus 191-192):
DAPIDLER_PROGRAM, P-AXIS_RCS_AUTOPILOT, Q_R-AXIS_RCS_AUTOPILOT, TJET_LAW,
KALMAN_FILTER (in Luminary 099 this is RATELOOP/ROTORQUE, the jet-torque
feedback of the rate estimator, not a separate filter), TRIM_GIMBAL_CONTROL_SYSTEM,
AOSTASK_AND_AOSJOB (in Luminary 099 this file holds 1/ACCS, 1/ACCONT, DVOVSUB;
AOSTASK itself no longer exists -- the offset estimate lives in the P-axis rate
derivation), DAP_INTERFACE_SUBROUTINES, T6-RUPT_PROGRAMS, RCS_FAILURE_MONITOR,
SPS_BACK-UP_RCS_CONTROL (name is correct in the repo; it holds SPSRCS, the
CSM-docked RCS law).

Code: shared glue in `src/lm/dap/` (arith.c, state.c, dapif.c, accs.c, accont.c,
gts.c, private `internal.h`, shim `pending.h` for not-yet-published shared symbols);
the RCS autopilot in `src/lm/dap/rcs/` (dapidler, needles, paxis, pmanual, rate,
qraxis, qrmanual, qrjets, tjetlaw, t6rupt, rcsmon, spsrcs). Public API
`include/agc/lm/dap.h`. Test: `tests/test_lm_dap.c`.

## Design table

Scales: angles "at pi" = SP fraction x pi rad; rates "at pi/4" rad/s;
jet times in TIME6 counts (0.625 ms, "scaled at 10.24 s") unless noted.

| AGC label (page) | Inputs (scale) | Outputs (scale) | C function (file) |
|---|---|---|---|
| CHEKBITS/CHEKMORE (1410) | CHAN31 b13-14, IMODES33 b6, CHAN30 b10 | DAP on/idle decision | lm_dap_chekbits (dapidler.c) |
| DAPIDLER, STARTDAP (1411-1413) | RCSFLAGS b13, ACCSOKAY | DAP erasables zeroed, T5ADR=PAXIS | agc_lm_dap_t5rupt / lm_dap_idler / lm_dap_startdap (dapidler.c) |
| MOREIDLE, SHUTDOWN (1413) | | errors, jets off, CHAN12 trim off | lm_dap_moreidle / lm_dap_shutdown (dapidler.c) |
| ALTDSPLY, DSPLYTOT (1414-1416) | P/Q/RERROR (pi), CPHI.. CDUX.. M11.. | AK,AK1,AK2 (at pi) | lm_dap_altdsply / lm_dap_dsplytot (needles.c) |
| NEEDLER, NEEDLES, DACLOOP (1417-1419) | AK (at 180 deg), RCSFLAGS b2,b3 | CDUXCMD.., EDRIVEX.., CHAN12 b6, CHAN14 b13-15 | lm_dap_needler (needles.c) |
| OVERSUB2 / OVERSUB (1419,1422) | A with overflow | A limited | lm_dap_add_lim (arith.c) |
| PAXIS (1421-1422) | DAPZRUPT, CDUs, CDUxD, DELCDUx | CDUxD advanced, TCP,TCQR | lm_dap_paxis (paxis.c) |
| 1STOTWOS, SUBDIVDE, DIVIDER (1422) | | | lm_dap_ones_to_twos (arith.c), lm_dap_subdivde (rate.c) |
| RATELOOP, ROTORQUE (KALMAN_FILTER 1470-1471) | TJP/U/V, NO.xJETS, 1JACC.. | JETRATE,Q,R (pi/4), DOWNTORK | lm_dap_rateloop (rate.c) |
| BACKP rate derivation (1423-1427) | CDU deltas, M-matrix, gains | OMEGAP,Q,R (pi/4), TRAPEDx, AOSQ/R, DxERROR | lm_dap_backp / lm_dap_traps (rate.c) |
| Offset estimator WORKTIME (1427) | QACCDOT,RACCDOT (pi/2^7) | AOSQ,R (pi/2), ALPHAQ,R, AOSxTERM (pi/4) | lm_dap_offset_est (rate.c) |
| PRETIMCK, TURNOFFQ/R (1428-1429) | Q/RGIMTIMR (0.1 s) | CHAN12 b9-12, NEGUQ/R | lm_dap_pretimck (paxis.c) |
| CHKVISFZ, TRYUORV, ALTERYZ (1429-1430) | CHAN31 b9-12, CH6MASK | NEXTP | lm_dap_chkvisfz (paxis.c) |
| P state logic, min impulse, DETENTCK, RATERROR, PEGI (1430-1436) | CHAN31, P-RHCCTR, STIKSENS | TJP, PLAST, PBIT | lm_dap_pstate / lm_dap_pminimp / lm_dap_detentck / lm_dap_raterror / lm_dap_pegi (pmanual.c) |
| PJETSLEC, SELECTP, ALTSYST (1436-1440) | TJP, NUMBERT, CH6MASK | CHAN6, NO.PJETS, T6 list | lm_dap_pjetslec / lm_dap_selectp (paxis.c) |
| CALCPERR, PURGENCY (1437-1439) | CDUX/Y, CDUXD/YD, M11, DELPEROR | PERROR, EDOTP, TJP | lm_dap_calcperr / lm_dap_purgency (paxis.c) |
| QRAXIS, CALLQERR, Q,RORGTS (1441-1442) | OMEGAQ/R, OMEGAQD/RD, COTROLER | EDOTQ/R | lm_dap_qraxis (qraxis.c) |
| SENSEGET X-translation (1442-1444) | CHAN31 b7-8, ULLAGER, ACC4OR2X, AORBTRAN | NEXTU, NEXTV, SENSETYP | lm_dap_senseget (qraxis.c) |
| QRCONTRL min impulse (1444-1446) | CHAN31 b1,2,5,6 | TJU,TJV | lm_dap_qrminimp (qrmanual.c) |
| CHEKSTIK, RHCACTIV, ENTERUV, BACKHAND (1446-1449) | SAVEHAND, STIKSENS | QLAST,RLAST, TJU,TJV, QRBIT | lm_dap_chekstik / lm_dap_enteruv / lm_dap_backhand (qrmanual.c) |
| TRYGTS, CHKINGTS (1449-1450) | USEQRJTS, ALLOWGTS, CHAN5, INGTS | control choice | lm_dap_trygts (qraxis.c) |
| QERRCALC, RERRCALC (1451) | CDUY/Z, CDUYD/ZD, M21..M32, DELQ/REROR | QERROR, RERROR (pi) | lm_dap_qerrcalc (qraxis.c) |
| STILLRCS/ATTSTEER, TJLAW (1452) | Q/RERROR, OMEGAU/V | UERROR,VERROR, TJU/TJV | lm_dap_tjlaw_axis (qraxis.c) |
| AFTERTJ, DOROTAT, DOSKIP, FEEDBACK, XTRANS (1453-1455) | TJU/V, NUMBERT, SNUFFER | CHAN5, SKIPU/V, NO.U/VJETS | lm_dap_aftertj / lm_dap_xtrans (qrjets.c) |
| JTLST (1456-1457) | T6FURTHA (counts), index | TIME6, T6NEXT, NXT6ADR, CHAN13 b15 | lm_dap_jtlst (t6rupt.c) |
| ROT-TOUV (1457) | Q,R comps, COEFFQ,COEFFR | U,V comps | agc_lm_dap_rot_touv (qrjets.c) |
| SELCTSUB, FAILOOP (1457-1458) | ROTINDEX, NUMBERT, CH5MASK | POLYTEMP | lm_dap_selctsub (qrjets.c) |
| CLOSEOUT, ENDJASK (1458) | | DAPZRUPT=NEGMAX | lm_dap_closeout (qrjets.c) |
| TJETLAW (1460-1469) all zones, RUFLAWs | E (pi), EDOT (pi/4), BLOCKTOP | TJETU (T6 counts, signed), NUMBERT, FIREFCT (pi/4) | lm_dap_tjetlaw (tjetlaw.c) |
| GTS, ALGORTHM .. NEGUSUM (1472-1476,1481-1484) | AOSQ/R, EDOTQ/R, KQ, QDIFF | NEGUQ/R, CHAN12, RCSFLAGS b5 | lm_dap_gts (gts.c) |
| GTSQRT, ROOTCYCL (1481-1484) | FUNCTION (DP) | sqrt (SP), SHFTFLAG | lm_dap_gtsqrt (gts.c) |
| ACDT+C12 (1476) | NEGUQ/R, ACCDOTQ/R | QACCDOT,RACCDOT, CHAN12 | lm_dap_acdt_c12 (gts.c) |
| TIMEGMBL (1478-1480) | AOSQ/R (pi/2), ACCDOTQ/R (pi/2^7) | Q/RGIMTIMR (0.1 s), NEGUQ/R, ALLOWGTS | agc_lm_dap_timegmbl (gts.c) |
| 1/ACCSET, 1/ACCJOB, 1/ACCS, F(MASS), COMMEQS, LRESC, SPSCONT (1486-1491) | MASS (B16 kg), ABDELV (B13 cm/s2), DB, AOSQ/R | 1JACC.. (pi/4), COEFFQ/R, ACCDOTQ/R, KQ | agc_lm_dap_jet_accel_update / agc_job_jet_accel_update / agc_job_jet_accel_reset; static lem_mass, fit_row, commeqs, jaccuv, lresc, spscont (accs.c) |
| DVOVSUB (1492) | DP dividend, SP divisor | SP quotient, limited | lm_dap_dvovsub (arith.c) |
| DOCKED inertia fit (1494-1496) | LEMMASS, CSMMASS | 1JACC.., ACCDOTR | static docked / docked_fit (accs.c) |
| 1/ACCONT, BOTHAXES, DO1/NET+, 1/ANET-, FIXMIN, DBFUN, STORV, DRFDB, 1/ACCRET (1497-1505) | 1JACC.., AOSU/V, DB, FLAT | BLOCKTOP[48] | lm_dap_switch_curves; static invert, do1net_plus, net_min, anet_minus, p_axis, db_bigaos, db_smallaos, axis_with_aos, axis_against_aos, axis_fail_dist, store_axis, store_deadbands (accont.c) |
| RESTORDB, SETMAXDB, SETMINDB, PFLITEDB (1407-1408) | DBSELECT | DB (at 45 deg), 1/ACCJOB | agc_lm_dap_restordb/setmaxdb/setmindb/pflitedb (dapif.c) |
| ZATTEROR, STOPRATE, ALLCOAST (1408-1409) | CDUs | CDUxD, OMEGAxD, DELCDUx, DELxEROR, AOS | agc_lm_dap_zatteror/stoprate/allcoast (dapif.c) |
| T6JOBCHK, T6JOB, DOT6RUPT, WRITEP/U/V (1403-1405) | TIME6 list | CHAN5, CHAN6, CHAN13 b15 | agc_lm_dap_t6jobchk / agc_lm_dap_t6rupt / lm_dap_write_p/u/v (t6rupt.c) |
| RCSMON, 1/ACCFIX (190-192) | CHAN32 b1-8, PVALVEST | CH5MASK, CH6MASK, 1/ACCJOB | agc_lm_dap_rcsmon (rcs/rcsmon.c), lm_dap_accfix (accs.c) |
| SPSRCS, POSTHRST, SPSSTART (SPS_BACK-UP 1507-1510) | E, EDOT, DKDB, DRIFTBIT | TJU[AXISCTR], UJETCTR | lm_dap_spsrcs (rcs/spsrcs.c) |
| Fresh start / restart DAP parts (FRESH_START 212-213, 219) | | DB, STIKSENS, gains, T5ADR | agc_lm_dap_fresh_start / agc_lm_dap_restart (state.c) |

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
* Session 3: NAMING rule applied (1/ACCS -> jet_accel_update etc., see docs/RENAMES.md)
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
  * DONE tests/test_lm_dap.c (120 checks)
  * FIXED rcs/qrjets.c ROT-TOUV: U and V were swapped (AGC: U = -COEFFQ q + COEFFR r in A,
    V = COEFFQ q + COEFFR r in L).
  * Rule 7 (no (void) on agc_ calls): lm_dap_alarm() wrapper (dapif.c) for TC ALARM in a DAP
    pass; lm_dap_chekbits(bool *on) and lm_dap_chkvisfz(bool *policy) now return status;
    PAXIS propagates TIME5 / ACDT+C12 / BAILOUT statuses; T6JOBCHK returns agc_ccshole();
    DAPIDLER propagates a failed NOVAC (1/ACCSET).
* All routines in scope are translated. Nothing left TODO.

## Deviations

* -0 is not represented (all files): values are ones'-complement magnitudes in agc_sp_t.
* accs.c lm_alone: the AGC tells descent from ascent after the F(MASS) loop by CCS on the
  index MPAC, which counts 6, 4, 2 down to -0 (2 + -2 in ones' complement) in descent and
  stays positive in ascent; translated as an explicit `descent` flag.
* accs.c docked: read literally, after the inertia pass COEFCTR is +0 and CCS COEFCTR lands
  on the first word of the TORQCONS constant (00000). The comments' intent (save inertia in
  MPAC +1, then run the CG pass) is translated. See Open questions.
* accont.c: in powered flight without GTS the AGC copies Z3TEM (a VBUF word left from
  whatever used VBUF last) into ZONE3LIM; here ZONE3LIM keeps its previous value.
* accont.c: the U-axis DXCH of DBB1.. into UDB1.. is a copy (the AGC swap leaves stale
  words in DBB3/DBB4, which only matter for AXISDIST in drifting flight, where AXISDIST is
  replaced by DRFDB anyway).
* rcs/spsrcs.c: CCS UJETCTR < 0 also reaches ZAPTJ (code), commented "not positive" (listing);
  the counter is never negative, so a nonzero counter inhibits.
* tjetlaw.c: ACCSWU -1 index for the P axis (see the file).

## Stubs

(none)

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

## Tests written

tests/test_lm_dap.c, 120 checks: 1/ACCS descent at 15000 kg (jet accelerations, U,V
coefficients, -2JETLIM, ACCSOKAY, BLOCKTOP filled, mass limited to HIDESCNT); TJETLAW zero
error/rate gives TJ = 0 on P, U, V; +10 deg error gives TJ < 0 and -10 deg gives TJ > 0 on U, V;
rate estimator converges on a constant 400 (1.1 deg/s) P rate to +-8 within 30 passes; RCSMON
valve close/open masks; SPSRCS outer rate limit, firing to target, inhibition counter;
ROT-TOUV polarity. The repo tree does not link as a whole yet (other lanes' missing symbols),
so `make test` cannot build it; verified by linking the DAP sources + core + guid_state.o with
a scratch stub file (exec/alarm/inhint, M-matrix, RHC counters, IMU CDU commands, TIME5 model).

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
