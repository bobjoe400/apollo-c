# review-guid-entry -- review of the guid-entry and p37 lanes

History of this lane: [history/lanes/review-guid-entry.md](../../history/lanes/review-guid-entry.md)

Lane files: src/csm/entry/{lexicon.c,reentry.c,programs.c,cm_body_attitude.c,dap.c},
src/csm/burns/earth_orbit_insertion.c, src/csm/burns/return_to_earth/*,
include/agc/csm/entry*.h, tests/csm/{entry,return_to_earth}.c,
tests/oracle/{entry,return_to_earth}.c.
AGC: ENTRY_LEXICON, REENTRY_CONTROL, P61-P67, CM_BODY_ATTITUDE,
CM_ENTRY_DIGITAL_AUTOPILOT, P11, P37_P70 (Comanche055).

Tools (scratch, review-guid-entry/): probe.c = agc_probe + `istop LABEL n`
(stop at the n-th interpretive dispatch of LABEL) and `itfrom LABEL n`
(trace MPAC per interpretive instruction); mk.py builds per-pass specs
from a case; rt.sh builds a test against fresh lane objects + a private
snapshot library (mylib.sh) because the shared tree is mid-edit.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [guid-entry](guid-entry.md#program-flow-and-decisions), [p37](p37.md#program-flow-and-decisions).
