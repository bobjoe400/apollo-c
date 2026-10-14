# review-nav-integ -- review of the nav-integ lane

History of this lane: [history/lanes/review-nav-integ.md](../../history/lanes/review-nav-integ.md)

## Scope

history/assignments/review.md for lane nav-integ (docs/modules/nav-integ.md):
src/nav/integration/{init,jobs,orbital_integration,flags_csm,setitctr_lm,
svdwn}.c, internal.h, pending.h, include/agc/nav/integration.h,
tests/nav/integration.c, tests/oracle/nav_integration.c.
AGC: INTEGRATION_INITIALIZATION.agc, ORBITAL_INTEGRATION.agc (both ropes),
STATEUP/THISVINT (FRESH_START_AND_RESTART).

Build note: HEAD (4f35a43 checkpoint) does not build (other review lanes
mid-edit, e.g. src/imu/mode.c). Lane work is verified in a scratch tree
from 3f9ede8 (lane files unchanged since) with the lane files overlaid.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [nav-integ](nav-integ.md#program-flow-and-decisions).
