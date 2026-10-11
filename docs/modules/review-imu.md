# review-imu -- review of the imu lane

Lane files: src/imu/*.c, src/imu/internal.h, src/csm/prelaunch_align.c,
include/agc/imu/imu.h, tests/imu/imu.c, tests/oracle/imu.c.
AGC: IMU_MODE_SWITCHING_ROUTINES, IMU_COMPENSATION_PACKAGE,
INFLIGHT_ALIGNMENT_ROUTINES, GIMBAL_LOCK_AVOIDANCE, KALCMANU_STEERING,
ANGLFIND / ATTITUDE_MANEUVER_ROUTINE, R60_62, IMU_CALIBRATION_AND_ALIGNMENT.

## Progress
* [x] P06 / PRESTAND / SCALPREP / POSTAND translated (mode.c: agc_job_standby,
  agc_job_standby_request, agc_job_standby_recovery; registry rows added; compiles)
* [x] MODECADR -0 encoding resolved: keep -2 (agc_sp_t has no -0), public
  AGC_MODECADR_FREE/GOOD/BAD + agc_imu_modecadr_bzf() in imu.h; callers in
  other lanes requested to switch (see Requests)
* [ ] prelaunch gyrocompass (P01/P02/P03, V65, V78) translated
* [ ] oracle findings F5 / F6
* [ ] fidelity pass per file
* [ ] naming (digit-suffixed labels)
* [ ] style debt to zero

## Requests to other lanes
* io-pinball/sys-misc (src/io/verbs/extended_verbs.c agc_extverb_ckmodcad,
  extended_verbs_csm.c OPTCOARK) and dap-lm/prog-rendz (src/lm/radar/designate.c
  DESRETRN): the AGC tests MODECADR with BZF, so -0 (complete, failed) counts
  as free.  Replace `agc_erase_modecadr(d) == 0` / `!= 0` with
  `agc_imu_modecadr_bzf(d)` / `!agc_imu_modecadr_bzf(d)`.
