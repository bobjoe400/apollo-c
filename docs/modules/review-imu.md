# review-imu -- review of the imu lane

Lane files: src/imu/*.c, src/imu/internal.h, src/csm/prelaunch_align.c,
include/agc/imu/imu.h, tests/imu/imu.c, tests/oracle/imu.c.
AGC: IMU_MODE_SWITCHING_ROUTINES, IMU_COMPENSATION_PACKAGE,
INFLIGHT_ALIGNMENT_ROUTINES, GIMBAL_LOCK_AVOIDANCE, KALCMANU_STEERING,
ANGLFIND / ATTITUDE_MANEUVER_ROUTINE, R60_62, IMU_CALIBRATION_AND_ALIGNMENT.

## Progress
* [ ] P06 / PRESTAND / SCALPREP / POSTAND translated (mode.c)
* [ ] MODECADR -0 encoding resolved
* [ ] prelaunch gyrocompass (P01/P02/P03, V65, V78) translated
* [ ] oracle findings F5 / F6
* [ ] fidelity pass per file
* [ ] naming (digit-suffixed labels)
* [ ] style debt to zero
