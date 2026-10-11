# review-imu -- review of the imu lane

Lane files: src/imu/*.c, src/imu/internal.h, src/csm/prelaunch_align.c,
include/agc/imu/imu.h, tests/imu/imu.c, tests/oracle/imu.c.
AGC: IMU_MODE_SWITCHING_ROUTINES, IMU_COMPENSATION_PACKAGE,
INFLIGHT_ALIGNMENT_ROUTINES, GIMBAL_LOCK_AVOIDANCE, KALCMANU_STEERING,
ANGLFIND / ATTITUDE_MANEUVER_ROUTINE, R60_62, IMU_CALIBRATION_AND_ALIGNMENT.

## Progress
* DONE P06 / PRESTAND / SCALPREP / POSTAND (src/imu/mode.c: agc_job_standby,
  agc_job_standby_request, agc_job_standby_recovery); registry rows AGC_JOB_STANDBY,
  _STANDBY_REQUEST, _STANDBY_RECOVERY added (tools/registry.tsv, regenerated);
  imu.h declarations updated.  mode.c compiles.
* DONE MODECADR -0: keep -2; AGC_MODECADR_FREE/GOOD/BAD + agc_imu_modecadr_bzf()
  (imu.h, mode.c); call sites in other lanes listed under Requests.
* PARTIAL Comanche prelaunch (P01/P02/P03, V65, V78, IMUTEST/LATAZCHK):
  - NEW src/csm/prelaunch_internal.h (AINLA overlay struct, shared helpers) -- written.
  - src/csm/prelaunch_align.c REWRITTEN (P01 GTSCPSS..GEOIMUTT/IMUBACK jobs, NBPOSPL,
    IMUSTLLG, EARTHR/EARTHR*, SOMERR2/ENDTEST1, IMUTEST/GUESS/LATAZCHK; PIPACHK stub).
    agc_csm_finetime REMOVED (duplicate of agc_systems_test_finetime) -- also delete its
    declaration from imu.h.  Does NOT compile yet: needs the declarations/ids below.
  - NEW src/csm/prelaunch_gyrocompass.c written (ESTIMS..RESCHNG, ALLOOP tasks, ALFLT,
    ALWAYSG/COMPGS, CHKCOMED/PRELTERM, V78 AZMTHCG1).  Does NOT compile yet (same reason).
  - DONE src/csm/prelaunch_verify.c (P03: GCOMPVER/SETNBPOS, OPTDATA V05N30/V06N41, TAR/EREF,
    CONTIN33/TARGDRVE/RETARG(1), NEXTBNKS/NEXBNKSS/LITTLSUB(=agc_csm_sxtsm on mark 0), AXISGEN,
    CALCGTA, GCOMP4 V06N93, ERCOMP1 += OGC, GCOMP5, GTSOPTCS 1602).  make, make test green for
    the lane (test_imu, oracle_imu); test_rendezvous orbit-params failure is NOT this lane.
  - DONE imu.h prototypes (P01/P02/P03/V78 jobs+tasks, set_pad, thetan) and registry rows;
    gen_registry.py run; `make lib` GREEN (2026-10-11).  Remaining link gap: P03 functions
    (agc_job_optical_verification*, prelaunch_verify_start) -> prelaunch_verify.c.
* DONE rule 7: (void) AGC_ASSERT in prelaunch_align.c / prelaunch_gyrocompass.c made checked
  asserts with recovery; check_rules on lane files: 0 hard violations (density warns remain:
  attitude_maneuver 1.62, compensation 1.73, gyro 1.88, mode 1.57).
* DONE test: tests/imu/imu.c test_optical_verification (P03 end to end: V33 loads, two
  marks on an aligned IMU via MARKRUPT/MARK2/ENTANSWR/ENDMARKS, OGC < 5e-5 rev, ERCOMP1 += OGC,
  MM back to 02).  TAR/EREF uses align_dmp_shl (DMP SL1 on the TP product).
* DONE F6: compensation.c keeps GCOMP and DELV as AGC word pairs (word_pair_t, DAS with
  low-word carry, no sign normalisation; GCOMP words shadowed beside the gyro buffer and
  resynced when the buffer value changes).  oracle pipa_*: GCOMP and GCOMPSW now tol 0 (were
  1 / 2+sign).  The old code could differ in the >2-pulse torque decision near threshold.
* DONE F5 (partly fixed, residual explained): CDUTODCM and DELCOMP used DMPR (rounded) where
  ANGLFIND has DMP (TP product, MPAC+2 shifted by SL, STORE truncates): now dmp_dad_shl /
  agc_dp_mul_tp.  kalc_small AM 323 -> 5 LSB, kalc_large COF 17 -> 0.  Residual <= 4 LSB is the
  word-split class (yaAGC trace: BDSU leaves 1-cos A = (00001,40057) = 48 and DMPSUB truncates
  by words; DCMTOCDU's COS..SL1 needs the TP COS; C1's BDSU keeps MPAC+2).  Oracle tolerances
  tightened to the measured residuals (tests/oracle/imu.c, oracle-cases.md rows).
* DONE mode.c fidelity (IMUZERO..IMUZERO3, IMUCOARS/COARS/COARS2/COMPOS/COMNEG/COMZERO/CHKCORS,
  SETCOARS/RNDREFDR, IMUFINE/IMUFINED/IFAILOK/PFAILOK/PFAILOK2, NOATTOFF, PIPUSE/PIPUSE1/PIPFREE,
  ENDIMU/IMUBAD/CAGETEST/CAGETSTQ/CAGETSTJ, GOODEND/BADEND/ENDMODE, STALL/MODEGOOD/MG2/MODESLP/
  MODABORT, R02BOTH/R02ZERO) vs Comanche pp.1420-1444.  Fixed: PIPFREE alarm path re-INHINTed
  (interrupts left off; test_pipfree_alarm); setissw() asserted OK but SETISSW returns ALARM when
  it lights the ISS warning; IMUFINED asserted gyros idle (not an AGC invariant).  Renamed
  IMUZERO2/3, COARS/COARS2/CHKCORS tasks, DSP11_*, dsp11_write, pfailok2, agc_imu_pipuse1;
  removed agc_imu_fin20 (IMUFIN20 = IMUFINE alias; caller in csm/alignment uses agc_imu_fine).
* NOTE: src/lm/radar/read.c (other lane) is mid-edit and breaks `make`; lane tests are built with
  SCRATCH/review-imu/tbuild.sh (re-archives the last good objects) until it compiles again.
* DONE DMP/DMPR + shift audit (review_items "ALL remaining lanes"): every DMP in ANGLFIND,
  INFLIGHT_ALIGNMENT, GIMBAL_LOCK_AVOIDANCE, R60_62 (LM BALLANGS), P51-P53 CALCSMSC and P03 TAR/EREF
  now truncating (agc_dp_mul_trunc, or the TP product where the AGC shifts it: DMP SLn, DOT SL1
  before ARCCOS/STORE).  36 sites changed DMPR->DMP (CDUTODCM 14, DELCOMP 4, CALCGTA 2,
  CALCSMSC 8, BALLANGS 7, KALCMANU TM 1) + 2 DOT SL1 TP shifts (CALCGA, VECPOINT); 6 DMPR sites
  confirmed (ALWAYSG/COMPGS).  SR/VSR: all SR truncate, ALTCALC VSR1 rounds -- as the AGC.
  CALCGTA now bit-exact (tol 1 -> 0).  Not fixable with the public API (no TP COS): DCMTOCDU and
  GUESS "COS SL1" lose MPAC+2's top bit.
* DONE gyro.c fidelity (IMUPULSE..8192AUG all match, incl. N=1 -0 CCS); compensation.c: CM LASTBIAS
  calls PIPUSE (zeroes PIPAs) where LM calls PIPUSE1 (was PIPUSE1 for both); CM NBD2 ends the job
  on equal TIME1 readings (-0) instead of running NBD3 with dt 0 (test_lastbias_pipuse).
* DONE fidelity attitude_maneuver.c (R60CSM/REDOMANN/TOBALL/TOBALLC/CHKLINUS/RELINUS/R61TEST/
  ENDMANU1, VECPOINT/COMPMATX/PICKAXIS/ROT180/FINDGIMB, LM BALLANGS) and alignment.c (CALCGTA,
  ARCTRIG, CALCGA, AXISGEN, CALCSMSC): match.  Renamed VECPNT1 -> agc_imu_vecpoint_from_cdus
  (caller in src/lm/navigation/tracking_attitude.c), ENDMANU1, GIMLOCK1.
* DONE fidelity maneuver.c (GOMANUR, KALCMAN3 branches, WCALC/LOCSKIRT, NEWDELHI, NEWANGL,
  INCRDCDU, TIMECHK, MANUSTAT, CONTMANU, UPDTCALL, MANUSTOP, ENDMANU, STOPRATE/STOPYZ, ZEROERROR,
  LOADCDUD).  Fixed: TM kept as the AGC words MANUSTAT's two DAS leave (the minor word can be
  negative under a positive major word; TIMECHK branches on the major word first) -- with a
  normalised TM the last-second MANUSTOP delay could be 1 cs instead of up to 100.
  Renamed KALCMAN3 (job id AGC_JOB_KALCMANU_START), NEWANGL+1, NOGOM2.
* TODO fidelity pass: maneuver (KALCMANU steering), vflags*; CALCGA CPHIFLAG checked: BONCLR CPHIFLAG S2 returns without STCALL THETAD, as the C
* TODO naming (digit-suffixed labels)
* DONE style mode.c, gyro.c, compensation.c, alignment.c, attitude_maneuver.c, maneuver.c, internal.h, prelaunch_*.c/h: 0 check_style findings (prologs via
  SCRATCH/review-imu/prolog.py + spec_*.py; decl comments via declfix.py).
* TODO style debt (984 findings at start in lane files; remaining: gyro, compensation, alignment,
  attitude_maneuver, maneuver, maneuver_calc, vflags*, imu.h)
* Note: other lanes' files were reformatted once by a full `make format` (formatting only);
  from now on format only lane files with uncrustify directly.

## Requests to other lanes
* io-pinball/sys-misc (src/io/verbs/extended_verbs.c agc_extverb_ckmodcad,
  extended_verbs_csm.c OPTCOARK) and dap-lm/prog-rendz (src/lm/radar/designate.c
  DESRETRN): the AGC tests MODECADR with BZF, so -0 (complete, failed) counts
  as free.  Replace `agc_erase_modecadr(d) == 0` / `!= 0` with
  `agc_imu_modecadr_bzf(d)` / `!agc_imu_modecadr_bzf(d)`.
