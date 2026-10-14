# review-imu -- review of the imu lane

History of this lane: [docs/history/lanes/review-imu.md](../history/lanes/review-imu.md)

Lane files: src/imu/*.c, src/imu/internal.h, include/agc/imu/imu.h,
src/csm/prelaunch/align.c, src/csm/prelaunch/gyrocompass.c,
src/csm/prelaunch/verify.c, src/csm/prelaunch/internal.h, tests/imu/imu.c,
tests/oracle/imu.c (+ registry rows in tools/registry.tsv).
AGC: IMU_MODE_SWITCHING_ROUTINES, IMU_COMPENSATION_PACKAGE,
INFLIGHT_ALIGNMENT_ROUTINES, GIMBAL_LOCK_AVOIDANCE, KALCMANU_STEERING,
ANGLFIND / ATTITUDE_MANEUVER_ROUTINE, R60_62, IMU_CALIBRATION_AND_ALIGNMENT,
P51-P53 (CALCSMSC).

## Deviations (kept, each commented in the code)
* MODECADR -0 stored as -2 (agc_imu_modecadr_bzf for BZF tests).
* GOODEND on a negative MODECADR resets it (Luminary) for both vehicles.
* gyro.c: augment count in a module variable, not the buffer's major word.
* CONTMANU: NEXTIME == TIME1 gives 1 cs, not a zero WAITLIST delay.
* F5 residuals (<= 4 LSB, CALCGA +-1 CDU): word splits the agc_dp_t value cannot hold.
* prelaunch: STOREDTA keeps only the compass words; ALTIM -0 as +0; GYROBUSY reruns
  RESTAIER (see prelaunch_gyrocompass.c prolog).

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [imu](imu.md#program-flow-and-decisions).
