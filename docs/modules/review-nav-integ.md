# review-nav-integ -- review of the nav-integ lane

## Scope

docs/agents/review.md for lane nav-integ (docs/modules/nav-integ.md):
src/nav/integration/{init,jobs,orbital_integration,flags_csm,setitctr_lm,
svdwn}.c, internal.h, pending.h, include/agc/nav/integration.h,
tests/nav/integration.c, tests/oracle/nav_integ.c.
AGC: INTEGRATION_INITIALIZATION.agc, ORBITAL_INTEGRATION.agc (both ropes),
STATEUP/THISVINT (FRESH_START_AND_RESTART).

Build note: HEAD (4f35a43 checkpoint) does not build (other review lanes
mid-edit, e.g. src/imu/mode.c). Lane work is verified in a scratch tree
from 3f9ede8 (lane files unchanged since) with the lane files overlaid.

## Progress

| item | status |
|---|---|
| baseline (unit 158 checks, oracle 247, 0 known) | DONE |
| review items: oracle findings / TEPHEM / X2-after-ORIGCHNG / state machine vs sleep | TODO |
| fidelity: orbital_integration.c | TODO |
| fidelity: init.c | TODO |
| fidelity: jobs.c | TODO |
| fidelity: svdwn.c, setitctr_lm.c, flags_csm.c | TODO |
| naming / clarity | TODO |
| style debt (827 findings at start) | TODO |
| rule-5 density (orbital_integration.c 1.91) | TODO |

## Findings

| id | where | AGC | finding | fix |
|---|---|---|---|---|

## Lessons
