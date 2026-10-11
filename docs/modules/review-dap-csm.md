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
* [x] F9 PITCHDAP: TVC filter DP quantities are now agc_csm_dp_pair_t (AGC word pairs, DAS without sign agreement, mysubs.c csm_dap_pair_*); daps.c, exec.c (CG.CORR), init.c rewritten; oracle pitch cases bit-exact word by word, 8 KNOWN -> hard (282 checks, 0 known)
* [x] ATTINIT bug: P40 DOTVCON writes -ERRORY,-ERRORZ into ERRBTMP but init.c read errbinit[] that nobody wrote; errbinit removed, ATTINIT reads errbtmp.hi/.lo (guid-powered already adapted ignition.c)
* [x] daps.c tvcd_dap returned an uninitialised status on the pitch pass (fixed in the rewrite)
* [x] renames applied (registry regenerated): see Renames
* [ ] RCS DAP F9 class (DRHO, ADOT, MERROR, EDOT/KMPAC, SMALLMP, DPADD) -> pairs
* [x] style + fidelity DONE: tvc/daps.c, tvc/exec.c, tvc/init.c, tests/oracle/dap_csm.c
* [x] ROLLSET: the second DDOUBL wraps the 16-bit A when the product >= 1/2 and the TS limit then takes the opposite sign; modelled exactly (rolldap.c roll_fire_time), new oracle cases roll_wrap / roll_wrap_neg (1/CONACC .75) pass, and fail with the old true-sign limit
* [x] QUARTER bug: filter.c (-(.1)ADOT) and needles.c (CS QUARTER) used 8192; QUARTER = BIT13 = 010000 = 4096 (rope). TO FIX in the RCS pass
* [x] style + fidelity DONE: tvc/rolldap.c
* [ ] digit-suffixed names
* [ ] fidelity pass per file
* [ ] style debt to zero, rule-5 density >= 2.0
* [ ] make, make test, make check
