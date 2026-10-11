# review-nav-aux -- review of the nav-aux lane

Lane files: src/nav/auxiliary/*.c, internal.h, include/agc/nav/auxiliary.h,
tests/nav/navaux.c, tests/oracle/nav_aux.c.
AGC: LATITUDE_LONGITUDE_SUBROUTINES, PLANETARY_INERTIAL_ORIENTATION,
MEASUREMENT_INCORPORATION, TIME_OF_FREE_FALL,
LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES (CM and LM), STAR_TABLES.

## Progress
* [x] NEWANGLE: TP product shifted then rounded once (agc_tp_shift /
      agc_tp_to_dp) -- 40/40 random cases bit-exact vs Comanche055
      (old code: 16/40 off by 1 LSB). Committed in checkpoint 4f35a43.
* [x] MOONMX mul_half: TP product, SL1, round (was DP-rounded then
      shifted): 270 elements, 148 -> 45 off, max 8 -> 4 LSB.
* [x] navaux unit test: 143/0 failures; oracle nav_aux 484/0 (private
      build: HEAD imu/mode.c does not compile, other lane).
* [ ] MOONMX residual (45/270, <= 4 LSB)
* [ ] pad-load accessor unification
* [ ] fidelity pass per file
* [ ] style debt to zero
