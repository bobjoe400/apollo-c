# Assignment: imu   (tier: translation)

Scope, into src/imu/ with include/agc/agc_imu.h (you create it):
- IMU_MODE_SWITCHING_ROUTINES.agc (IMUZERO, IMUCOARS, IMUFINE, IMUSTALL,
  the CDU zero / coarse-align pulse logic, gyro torquing (IMUPULSE), the
  IMU fail / CDU fail / PIPA fail monitors, IMUMON) -> mode.c
- IMU_COMPENSATION_PACKAGE.agc (1/PIPA, PIPA bias and scale compensation,
  gyro drift compensation NBDX/ADIAX/ADSRAX, 1/GYRO) -> compensation.c
- INFLIGHT_ALIGNMENT_ROUTINES.agc (CALCGTA, CALCGA, AXISGEN, CALCSMSC) ->
  alignment.c
- GIMBAL_LOCK_AVOIDANCE.agc, KALCMANU_STEERING.agc, and the maneuver
  routine (Comanche ANGLFIND.agc / Luminary ATTITUDE_MANEUVER_ROUTINE.agc;
  diff and translate both where they differ) -> maneuver.c
- R60_62.agc (attitude maneuver R60, crew-defined maneuver R62) -> r60.c
- IMU_CALIBRATION_AND_ALIGNMENT.agc (Comanche; prelaunch P01-P03 pieces) ->
  src/csm/prelaunch_align.c
Diff CM vs LM for shared files. The IMU hardware side (CDU counters,
channel 012/014/030 bits, PIPA counters) is accessed via agc_channels.h and
agc_state.h accessors. Waitlist tasks and jobs via agc_exec.h.
Tests: tests/test_imu.c (compiles): CALCGTA on identity REFSMMAT gives zero
gyro angles; gimbal-lock check on a known attitude; PIPA compensation with
zero bias is identity.
