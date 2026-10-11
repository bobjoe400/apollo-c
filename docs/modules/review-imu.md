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
  - TODO src/csm/prelaunch_verify.c (P03: GCOMPVER/SETNBPOS, prelaunch_verify_start =
    MKRELEAS+OPTDATA, target_show (GODSPRET V05N30), target_flash (V06N41), target_loaded,
    CONTIN33, TAR/EREF, TARGDRVE/RETARG (SXTANG, SXTMARK 1, OPTSTALL), target_marked,
    NEXTBNKS/LITTLSUB (agc_sxtmark_mark(0)+agc_csm_sxtsm), AXISGEN, CALCGTA -> align ogc,
    GCOMP4 V06N93 torque_display, torque_accepted (ERCOMP1 += OGC), GCOMP5 end, GTSOPTCS 1602).
  - TODO imu.h: replace the old stub prototypes (agc_csm_prelaunch_estims/alwaysg/restaier/
    chkcomed/tar_eref/gcompver/optdata/latazchk/azmthcg1/targdrve, agc_task_prelaunch_*,
    agc_csm_finetime, agc_csm_prelaunch_earthr) with: agc_csm_prelaunch_set_pad,
    agc_csm_prelaunch_thetan, agc_job_prelaunch_initialization/_imu_zeroed/_imu_coarse_aligned/
    _imu_fine_aligned, agc_task_gyrocompassing_wake, agc_job_imu_test_coarse_aligned/
    _site_display/_site_flash/_site_loaded, agc_job_gyrocompassing_start/_restart/_wait/_filter/
    _filter_restart/_torque/_torqued/_erection_check/_azimuth_check/_azimuth_slew/_azimuth_slewed,
    agc_task_gyrocompassing_sample/_sample_restart/_filter_start, agc_job_gyrocompass_azimuth_change/
    _flash/_loaded, P03 jobs.
  - TODO tools/registry.tsv: point AGC_JOB_RSTGTS1->gyrocompassing_restart, WTLISTNT->_wait,
    ALFLT1->_filter_restart, RESTAIER->_torque, RESTEST1->_erection_check, RESTEST3->_azimuth_check,
    RESCHNG->_azimuth_slew, COMPVER->optical_verification, GYROCOMPASS_AZIMUTH_CHANGE->
    gyrocompass_azimuth_change, PRELAUNCH_INITIALIZATION->prelaunch_initialization,
    TASK_ALLOOP1->sample_restart, TASK_SPECSTS->filter_start; add new ids (PRELAUNCH_IMU_ZEROED,
    _COARSE_ALIGNED, _FINE_ALIGNED, IMU_TEST_*, GYROCOMPASSING_START/_FILTER/_TORQUED/
    _AZIMUTH_SLEWED, GYROCOMPASS_AZIMUTH_FLASH/_LOADED, TASK_GYROCOMPASSING_SAMPLE/_WAKE,
    P03 ids); run gen_registry.py; compile; test.
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
