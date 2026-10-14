# review-guid-landing -- review of the guid-landing lane

History of this lane: [history/lanes/review-guid-landing.md](../../history/lanes/review-guid-landing.md)

Scope: src/lm/landing/{throttle,guidance,lunar_landing,aborts,displays,findcduw,state}.c,
src/lm/ascent.c, include/agc/lm/landing.h, tests/lm/landing.c,
tests/oracle/landing.c. AGC: Luminary099 pp. 785-856, 898-925.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [guid-landing](guid-landing.md#program-flow-and-decisions).
