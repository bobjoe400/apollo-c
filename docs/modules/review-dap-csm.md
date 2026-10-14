# review-dap-csm -- review of the dap-csm lane

History of this lane: [history/lanes/review-dap-csm.md](../../history/lanes/review-dap-csm.md)

Lane files: src/csm/dap/{mysubs,state}.c, src/csm/dap/internal.h,
src/csm/dap/rcs/*.c, src/csm/dap/tvc/*.c, include/agc/csm/dap.h,
tests/csm/dap.c, tests/oracle/dap_csm.c.
AGC (Comanche055): MYSUBS, RCS-CSM_DIGITAL_AUTOPILOT, AUTOMATIC_MANEUVERS,
RCS-CSM_DAP_EXECUTIVE_PROGRAMS, JET_SELECTION_LOGIC, TVCINITIALIZE,
TVCEXECUTIVE, TVCMASSPROP, TVCRESTARTS, TVCDAPS, TVCSTROKETEST, TVCROLLDAP,
P40-P47 S41.2/S40.14/RCSDAPON.

(Earlier run left no note; restarted 2026-10-10 from scratch.)

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [dap-csm](dap-csm.md#program-flow-and-decisions).
