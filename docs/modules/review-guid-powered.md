# review-guid-powered -- review of the powered-flight guidance lane

History of this lane: [docs/history/lanes/review-guid-powered.md](../history/lanes/review-guid-powered.md)

STATUS: DONE (resume 2, 2026-10-12). Open: the requests below.

## Scope

Lane files (docs/modules/guid-powered.md, after restructure):
src/guidance/{powered_flight,servicer_common,external_delta_v,burn_steering,guid_state,guid_nav_glue}.c,
src/csm/burns/{servicer,external_delta_v}.c, src/csm/burns/sps/*.c,
src/lm/burns/servicer/*.c, src/lm/burns/dps_burn*.c, src/lm/burns/ignition/*.c,
include/agc/guidance/{guidance,pending}.h, tests/guidance/, tests/oracle/guidance.c.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [guid-powered](guid-powered.md#program-flow-and-decisions).
