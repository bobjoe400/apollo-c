# review-guid-entry -- review of the guid-entry and p37 lanes

Lane files: src/csm/entry/{lexicon.c,reentry.c,programs.c,cm_body_attitude.c,dap.c},
src/csm/burns/earth_orbit_insertion.c, src/csm/burns/return_to_earth/*,
include/agc/csm/entry*.h, tests/csm/{entry,dap,return_to_earth}.c,
tests/oracle/{entry,return_to_earth}.c.
AGC: ENTRY_LEXICON, REENTRY_CONTROL, P61-P67, CM_BODY_ATTITUDE,
CM_ENTRY_DIGITAL_AUTOPILOT, P11, P37_P70 (Comanche055).

Tools (scratch, review-guid-entry/): probe.c = agc_probe + `istop LABEL n`
(stop at the n-th interpretive dispatch of LABEL) and `itfrom LABEL n`
(trace MPAC per interpretive instruction); mk.py builds per-pass specs
from a case; rt.sh builds a test against fresh lane objects + a private
snapshot library (mylib.sh) because the shared tree is mid-edit.

## Progress
* [x] F14 cause 1: DSU on a TP MPAC (V2T100 THETA1, XT1LIM BETA10) keeps
  MPAC+2 (rte_tp_dsu, velocity.c). leo X(T2) 38927 -> 221 LSB.
* [x] F14 cause 2: DVCALC SR* 6,2 truncates into MPAC+2 and SQRT reads the TP
  MPAC (agc_tp_sqrt). First DVCALC now bit-exact (DV 12541 36655), GAMDV
  X(T1) path identical to the AGC for rte_leo pass 1. KNOWN 58 -> 45.
* [x] F14 remainder traced, outside the lane (see Requests): INVC115
  sign-disagreeing N (core), GAMDV X(T1) sign-disagreeing after DAD (core),
  TIMERAD V(T2) (nav-conic). With the AGC's UH/ injected, rte_transearth
  pass 1 GAMDV (64 steps) is word-exact. RCON PCON DV R(T2)/ T12 flipped to
  hard checks (tests/oracle/return_to_earth.c).
* [ ] fidelity pass p37 files
* [ ] fidelity pass entry files, five dispatchers vs "* START" sections
* [x] RTARG push-list: probed (scratch p37w.spec). RTENCK3D leaves RATT at
  0D (PDDL), so the first P37W pops R(T2)/ (BMN path) or TIMETHET's R (PL 6);
  a V16N45 recycle pops the 6 words below the VAC area (garbage). C keeps
  RTARG: deviation stands, comment fixed.
* [x] AUGEKUGL: probed 30 (V, gamma) pairs 20-40 kfps: band selection
  identical, TE exact (1 LSB once), PHI within 62 LSB (0.005 NM) from the
  DSU-then-DMP* sign-disagreeing words (core class, as F9).
* [ ] naming / style debt / check
