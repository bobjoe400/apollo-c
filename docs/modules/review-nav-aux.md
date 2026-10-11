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
* [x] MOONMX residual (45/270, <= 4 LSB) EXPLAINED: NEWANGLE leaves
      NODI (all 30 cases) as a sign-disagreeing word pair (DAD); COS/SIN
      on those words differ by 1 LSB from COS/SIN of the joined value.
      Proven: COS of the AGC's NODI words (core word model) = AGC AVECTR
      in 30/30; with the joined value A.x is off in 21/30. agc_dp_t cannot
      carry the split -> accepted ones'-complement deviation (F2 class),
      core request for word-level SIN/COS.
* [x] latlong.c rounding order: GETERAD DSQ/SL1 on the TP; LAT-LONG and
      ARCTAN sum of squares keeps MPAC+2 of the second DSQ into SQRT
      (agc_tp_sqrt); DMP SL1R GAMRP shifts the TP then rounds.
* [x] latlong earth LAT 1 LSB EXPLAINED: ARCTAN's SR1 leaves the shifted-
      out bit in MPAC+2 and ARCSIN's ACOSST2 (DCS MPAC / AD HALF / SQRTSUB)
      takes that stale, un-complemented MPAC+2 into SQRTSUB. Shown with the
      core word model: m2 = 0 -> 29827835 (C), m2 = 8192 -> 29827834
      (AGC). Needs a TP-input ASIN/ACOS in core (request).
* [ ] pad-load accessor unification
* [ ] fidelity pass per file
* [ ] style debt to zero
