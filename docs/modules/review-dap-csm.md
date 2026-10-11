# review-dap-csm -- review of the dap-csm lane

Lane files: src/csm/dap/{mysubs,state}.c, src/csm/dap/{internal,pending}.h,
src/csm/dap/rcs/*.c, src/csm/dap/tvc/*.c, include/agc/csm/dap.h,
tests/csm/dap.c, tests/oracle/dap_csm.c.
AGC (Comanche055): MYSUBS, RCS-CSM_DIGITAL_AUTOPILOT, AUTOMATIC_MANEUVERS,
RCS-CSM_DAP_EXECUTIVE_PROGRAMS, JET_SELECTION_LOGIC, TVCINITIALIZE,
TVCEXECUTIVE, TVCMASSPROP, TVCRESTARTS, TVCDAPS, TVCSTROKETEST, TVCROLLDAP,
P40-P47 S41.2/S40.14/RCSDAPON.

(Earlier run left no note; restarted 2026-10-10 from scratch.)

## Progress
* [ ] F9 PITCHDAP CMDTMP / ERRBTMP (oracle KNOWN -> hard)
* [ ] ROLLSET overflow limiting
* [ ] digit-suffixed names
* [ ] fidelity pass per file
* [ ] style debt to zero, rule-5 density >= 2.0
* [ ] make, make test, make check
