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
* TODO oracle findings F5 / F6
* TODO fidelity pass per file (note: CALCGA CPHIFLAG branch stores nothing -- check AGC)
* TODO naming (digit-suffixed labels)
* TODO style debt (984 findings at start in lane files)
* Note: other lanes' files were reformatted once by a full `make format` (formatting only);
  from now on format only lane files with uncrustify directly.

## Requests to other lanes
* io-pinball/sys-misc (src/io/verbs/extended_verbs.c agc_extverb_ckmodcad,
  extended_verbs_csm.c OPTCOARK) and dap-lm/prog-rendz (src/lm/radar/designate.c
  DESRETRN): the AGC tests MODECADR with BZF, so -0 (complete, failed) counts
  as free.  Replace `agc_erase_modecadr(d) == 0` / `!= 0` with
  `agc_imu_modecadr_bzf(d)` / `!agc_imu_modecadr_bzf(d)`.
