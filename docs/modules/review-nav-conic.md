# review-nav-conic -- review of the nav-conic lane

History of this lane: [history/lanes/review-nav-conic.md](../../history/lanes/review-nav-conic.md)

## Scope

history/assignments/review.md for lane nav-conic (docs/modules/nav-conic.md):
src/nav/conic/*, include/agc/nav/conic.h, tests/nav/conic.c,
tests/oracle/conic.c, tools/oracle/cases/nav_conic{,_lm}.{spec,txt}.

Build note: the live tree has other lanes' edits in flight (and HEAD's
src/imu/mode.c does not compile), so the lane was built and tested in a
scratch snapshot (git archive HEAD + lane files overlaid; SCRATCH/
review-nav-conic/t.sh).

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [nav-conic](nav-conic.md#program-flow-and-decisions).
