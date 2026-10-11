# review-imu -- review of the imu lane

Lane files: src/imu/*.c, src/imu/internal.h, include/agc/imu/imu.h,
src/csm/prelaunch_align.c, src/csm/prelaunch_gyrocompass.c,
src/csm/prelaunch_verify.c, src/csm/prelaunch_internal.h, tests/imu/imu.c,
tests/oracle/imu.c (+ registry rows in tools/registry.tsv).
AGC: IMU_MODE_SWITCHING_ROUTINES, IMU_COMPENSATION_PACKAGE,
INFLIGHT_ALIGNMENT_ROUTINES, GIMBAL_LOCK_AVOIDANCE, KALCMANU_STEERING,
ANGLFIND / ATTITUDE_MANEUVER_ROUTINE, R60_62, IMU_CALIBRATION_AND_ALIGNMENT,
P51-P53 (CALCSMSC).

## Progress
REVIEW DONE (2026-10-11).  `make` and `make test` green; `make check`: the
lane's files have 0 check_rules violations/warnings (rule-5 density >= 2.0
in every lane file), 0 check_style findings (18 files, 984 at start) and
pass uncrustify --check; check-rules still fails on other lanes' files
(src/csm/dap/tvc, src/programs/alignment/word_forms.c, accont.c, ...).
No file is mid-edit.

* DONE translated stubs: P06 / PRESTAND / SCALPREP / POSTAND (mode.c); the 11
  Comanche prelaunch entries: P01 (GTSCPSS .. IMUBACK/NBPOSPL, prelaunch_align.c),
  P02 gyrocompassing incl. ESTIMS (prelaunch_gyrocompass.c), V78 AZMTHCG1, IMUTEST /
  GUESS / LATAZCHK, and P03 optical verification (new prelaunch_verify.c).
  Remaining justified stubs: the IMU performance test's drift part (PIPACHK, ALCGKK ..
  ONCEMORE) -- it ends in TC LAT(SPL), an erasable-memory program not in the rope.
* DONE registry: prelaunch/P03 ids added, gen_registry.py run (the build break of
  2026-10-11 came from the ids missing; fixed first).
* DONE MODECADR -0 kept as -2 (DEVIATION, agc_sp_t has no -0) with
  agc_imu_modecadr_bzf() for the AGC's BZF tests; other-lane call sites under Requests.
* DONE oracle F6 fixed, F5 partly fixed and the residual explained (Findings).
* DONE DMP/DMPR and shift audit (review_items "ALL remaining lanes"): 36 DMPR -> DMP
  sites + 2 DOT SL1 TP shifts; 6 DMPR sites confirmed; SR/VSR as the AGC.
* DONE fidelity pass of every lane file against Comanche055 (and Luminary099 where the
  lane has LM code); fixes in Findings.
* DONE naming: digit-suffixed labels spelled out (docs/RENAMES.md "review-imu").
* DONE style debt 984 -> 0; rule 7 (void)-cast asserts made checked.
* DONE tests: test_imu 65 -> 104 checks; oracle_imu 211 checks with tolerances
  tightened (calcgta 1 -> 0, pipa GCOMP 1 -> 0, GCOMPSW 2+sign -> 0, kalc_small AM
  512 -> 5, kalc_large COF 32 -> 1, delcomp 8 -> 4, dcmtocdu 4 -> 3).

## Findings

| # | where | AGC | finding | resolution |
|---|-------|-----|---------|------------|
| 1 | compensation.c | 1/PIPA, IRIGCOMP, DRFTSUB2, 1/GYRO, GCOMP1 (F6) | GCOMP/DELV normalised; the AGC's DAS leaves sign-disagreeing pairs (49, -14827) and DRFTSUB2 tests the major word: the >2-pulse torque decision could differ near threshold | word_pair_t with word-by-word DAS; GCOMP words shadowed beside the gyro buffer; oracle GCOMP/GCOMPSW exact |
| 2 | maneuver_calc.c | CDUTODCM, DELCOMP (F5) | DMPR (rounded) where ANGLFIND has DMP; TP shifts ignored | DMP + TP product (dmp_dad_shl); kalc_small AM 323 -> 5 LSB |
| 3 | maneuver_calc.c | DELCOMP, DCMTOCDU, CDUTODCM C1 (F5 residual) | yaAGC trace: BDSU leaves 1-cos A as (00001, 40057) = 48, DMPSUB truncates by words; DCMTOCDU shifts the TP COS result; C1's BDSU keeps MPAC+2 | not representable with agc_dp_t values; tolerances set to measured (<= 4 LSB); core request below |
| 4 | alignment.c | CALCGTA, CALCSMSC, CALCGA | DMPR for DMP (10 sites); CALCGA COS(MG) = DOT SL1 lost MPAC+2 | fixed; CALCGTA bit-exact; CALCGA +-1 CDU = 1TO2SUB high word of a split ARCTRIG pair (F5 class) |
| 5 | attitude_maneuver.c | LM BALLANGS, VECPOINT | DMPR for DMP (7 sites); VECPOINT DOT SL1 ARCCOS | fixed |
| 6 | maneuver.c | GIMBAL_LOCK_AVOIDANCE TM | AM DMP ANGLTIME used DMPR | fixed |
| 7 | maneuver.c | MANUSTAT / TIMECHK | TM kept as a value; the AGC's DAS (-0, -1SEC) leaves the minor word negative under a positive major word and TIMECHK branches on the major word: MANUSTOP 1 cs instead of up to 100 cs in the last second | TM words kept (tm_hi, tm_lo) with DAS carry in TIMECHK |
| 8 | mode.c | PIPFREE | alarm path re-INHINTed after ALARM (the AGC needs it because ALARM ends with RELINT; agc_alarm is balanced): interrupts left off | second INHINT dropped; test_pipfree_alarm |
| 9 | mode.c | SETISSW callers | setissw() asserted OK, but SETISSW returns ALARM when it lights the ISS warning | accept ALARM |
| 10 | mode.c | IMUFINED | asserted the gyros idle (not an AGC invariant; false at fine align + torquing) | removed |
| 11 | compensation.c | LASTBIAS (CM p.306) | CM calls PIPUSE (PIPAX..Z = -0), C called PIPUSE1 (LM) for both | by vehicle; test_lastbias_pipuse |
| 12 | compensation.c | NBDONLY/LASTBIAS NBD2 (CM) | equal TIME1 readings give -0 and the CM job ends (GCOMPSW untouched); C ran NBD3 with dt 0 | cm_no_time_elapsed (LM's HALF+HALF goes on with 0, as before) |
| 13 | prelaunch_verify.c | P03 GCOMPVER .. GCOMP5 | was a stub | translated; test_optical_verification (two marks on an aligned IMU: OGC < 5e-5 rev, ERCOMP1 += OGC, back to MM 02) |
| 14 | mode.c | IMUFIN20 | forwarding wrapper of IMUFINE | removed; src/csm/alignment caller uses agc_imu_fine |
| 15 | mode.c | MODABORT, GOODEND | (confirmed) CM POODOO / LM BAILOUT 01210; Comanche would JOBWAKE a negative MODECADR, Luminary resets it | Luminary way kept (DEVIATION, documented) |

Confirmed routine by routine (no change): ZEROICDU, IMUZERO..IMUZERO3, IMUCOARS, COARS,
COARS2/COMPOS/COMNEG/COMZERO/SENDPULS, CHKCORS/COARSERR, CA+ECE, SETCOARS, RNDREFDR,
IMUFINE, IFAILOK, PFAILOK/PFAILOK2, NOATTOFF, PIPUSE/PIPUSE1, ENDIMU, CAGETEST/CAGETSTQ/
CAGETSTJ, STALL/MODEGOOD/MG2/MODESLP, R02BOTH/R02ZERO, IMUPULSE/GYROBUSY/GWAKE,
STRTGYRO/STRTGYR2/GSELECT/MIN+-/MAJ+-/GMERGE/LONGGYRO/LASTSEG/AUG3/8192AUG/AUG2,
GCOMPSUB/DRIFTSUB/FBIASSUB, IRIGX/Y/Z, ARCTRIG, AXISGEN (bit-exact), CALCGA (CPHIFLAG:
BONCLR ... S2 returns without storing THETAD), KALCMAN3/SECAD/CHECKMAX/ALTCALC/COFMAXGO/
METHOD1-3, WCALC, NEWDELHI/NEWANGL/INCRDCDU/CONTMANU/UPDTCALL/MANUSTOP/ENDMANU/STOPRATE/
STOPYZ/ZEROERROR/LOADCDUD, GOMANUR, R60CSM/REDOMANN/TOBALL/TOBALLC/CHKLINUS/RELINUS/
R61TEST/ENDMANU1, VECPOINT/COMPMATX/PICKAXIS/ROT180/FINDGIMB, R62, ALFLT/ALWAYSG/COMPGS
(DMPR, as the AGC).

Lane open questions (docs/modules/imu.md): CALCSMSC has one implementation
(agc_align_calcsmsc wraps agc_imu_calcsmsc); the sim drains GYROCMD/CDUxCMD
(src/sim/channels.c); IMUMON is in sys-misc's T4RUPT -- all resolved.

## Deviations (kept, each commented in the code)
* MODECADR -0 stored as -2 (agc_imu_modecadr_bzf for BZF tests).
* GOODEND on a negative MODECADR resets it (Luminary) for both vehicles.
* gyro.c: augment count in a module variable, not the buffer's major word.
* CONTMANU: NEXTIME == TIME1 gives 1 cs, not a zero WAITLIST delay.
* F5 residuals (<= 4 LSB, CALCGA +-1 CDU): word splits the agc_dp_t value cannot hold.
* prelaunch: STOREDTA keeps only the compass words; ALTIM -0 as +0; GYROBUSY reruns
  RESTAIER (see prelaunch_gyrocompass.c prolog).

## Requests to other lanes
* core-numeric: a public word-level MPAC API (pair/TP words with DMPSUB, DAS on MPAC,
  TSSL, STORE, and SIN/COS returning the TP MPAC) so interpretive code can reproduce
  sign-disagreeing intermediate words (would close F5 and the F9 class).  The private
  model already exists in src/core/mpac.h.
* io (src/io/verbs/extended_verbs.c:194 CKMODCAD, extended_verbs_csm.c:599 OPTCOARK)
  and prog-rendz/dap-lm (src/lm/radar/designate.c:123 DESRETRN): the AGC tests MODECADR
  with BZF, so -0 (complete, failed) counts as free; use agc_imu_modecadr_bzf(d)
  instead of `agc_erase_modecadr(d) == 0`.
* prog-align: src/csm/alignment/imu_orientation_determination.c and imu_realign.c have
  rule-5 density 1.53 / 1.27.
* tools: check_rules.py walks directories only; a file argument checks nothing.

## Lessons
* An agc_dp_t holds a value, the AGC holds two words.  Wherever a DAS/BDSU result is
  read word by word (CCS on the major word, MP of one word, STORE after a shift), the
  split matters: model the words locally in basic code (compensation.c word_pair_t,
  maneuver.c TM) and expect <= few-LSB residuals in interpretive code.
* DMP vs DMPR: DMP keeps the TP product; a following SLn shifts MPAC+2's top bits in
  and the STORE truncates.  Use agc_dp_mul_trunc, or agc_dp_mul_tp + agc_tp_shift +
  agc_tp_to_dp; DAD/DSU on that TP leaves MPAC+2 alone (agc_tp_add with a DP).
* ALARM ends with RELINT in the AGC; agc_alarm() is balanced, so an INHINT the AGC
  repeats after ALARM must not be repeated in C (counted inhibit).
* CCS of +1 + -1: ones' complement 1 + (-1) = -0, so "AD MINUS1, CCS" lands on the
  fourth branch (LONGGYRO N = 1).
* yaAGC trace (tools/oracle agc_probe, `trace 1`, gets on the VAC work area) finds the
  exact word split behind a few-LSB difference in minutes.
* Comanche and Luminary differ in small places inside "identical" routines (LASTBIAS
  PIPUSE vs PIPUSE1, NBD2 -0 handling): diff the two ropes routine by routine.
