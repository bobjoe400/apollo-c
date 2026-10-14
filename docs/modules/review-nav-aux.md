# review-nav-aux -- review of the nav-aux lane

History of this lane: [history/lanes/review-nav-aux.md](../../history/lanes/review-nav-aux.md)

Lane files: src/nav/auxiliary/*.c, internal.h, include/agc/nav/auxiliary.h,
tests/nav/auxiliary.c, tests/oracle/nav_auxiliary.c.
AGC: LATITUDE_LONGITUDE_SUBROUTINES, PLANETARY_INERTIAL_ORIENTATION,
MEASUREMENT_INCORPORATION, TIME_OF_FREE_FALL,
LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES (CM and LM), STAR_TABLES.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [nav-aux](nav-aux.md#program-flow-and-decisions).
