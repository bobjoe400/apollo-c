# oracle-cases -- yaAGC differential cases for the numeric modules

## Scope

Definition-of-done item 6: per numeric module, a spec
tools/oracle/cases/<module>.spec, its yaAGC dump <module>.txt, and a test
tests/oracle/<module>.c that runs the C translation on the same inputs and
compares within a documented tolerance. Writes only tests/oracle/,
tools/oracle/cases/ and docs; mismatches in src/ are recorded below as
findings for the review wave.

## Progress

| Module | Status | Spec / test |
|---|---|---|
| nav-integ | DONE (11 cases, 247 checks pass) | nav_integ.spec, tests/oracle/nav_integ.c |
| nav-aux | TODO | |
| imu | TODO | |
| guidance | TODO | |
| landing | TODO | |
| entry | TODO | |
| dap-lm | TODO | |
| dap-csm | TODO | |
| targeting | TODO | |
| return_to_earth | TODO | |
| dsky | TODO | |

## Method

* tests/oracle/oracle_txt.h (new, tests only): loads
  tools/oracle/cases/<name>.spec and <name>.txt; `oracle_in*()` read the
  input words of a case from its `set` lines (the spec is the only copy
  of the inputs), `oracle_out*()` the dumped words, `oracle_cmp*()`
  compare with a tolerance in LSB of the stored word and count failures.
  A missing case file prints SKIP and passes (as core_numeric.c does).
* Tolerances are in LSB of the AGC word (DP: 2^-28 of the scale). Every
  tolerance > 0 is listed per case below with its reason.
* Regenerate dumps: `cd tools/oracle && make cases`.

## nav-integ (tools/oracle/cases/nav_integ.spec, tests/oracle/nav_integ.c)

Comanche055. Integration entered at INTEGRVS through INTPRET (Q =
INTEGRVS, FBANK 13); inputs RCV VCV TET, TDEC1 = VAC1+32, MOONFLAG
(FLAGWRD0 04000), E3J22R2M 5232d / E32C31RM 1465d (moon); everything else
0. Stop INTWAKE (the TC at the end of INTEXIT). |R| < RME/RMM so MIDFLAG
stays clear: each ACCOMP stage is GAMCOMP (primary) + OBLATE (earth J2-J4;
moon J2/J3, R-TO-RP, QUALITY1-3, RP-TO-R). TIMESTEP state before RECTOUT:
same entry with STATEFLG+VINTFLAG (FLAGWRD3 00024), stop PHASCHNG in
ANDOUT; C compares the permanent CSM state written by ATOPCSM. RECTIFY:
Q = RECTIFY (bank 11), QPRET = VAC1+42 = 25751 (ABTCONIC), stop POODOO.

| case | what | tolerance (LSB) |
|---|---|---|
| integ_earth_1step, integ_earth2_1step | 3000 cs, one step, 6678 / 6900 km inclined circular | RATT VATT RRECT VRECT 2, TAT MU(P) TET 0, FV 8 (observed <= 6) |
| integ_moon_1step | 3000 cs, lunar 1850 km | as above (VATT.z 1 LSB) |
| integ_earth_multi | 100000 cs, 5 steps | positions 64, velocities 8192, FV 1024 (finding F1) |
| integ_moon_multi | 100000 cs lunar | positions 64, velocities 4096, FV 1024 (F1) |
| step_earth_max | TDEC1 = one maximum step (24064 cs): TDELTAV TNUV RCV VCV TC XKEP TET FV | TDELTAV/TNUV 4 (TNUV 2 observed), RCV VCV TC TET 0, XKEP 2 |
| step_earth_2max | two maximum steps | VCV 1024 (512 observed, F1), XKEP 8 |
| step_moon | lunar one step | TNUV 4, XKEP 2 (1 observed) |
| rectify_earth / _moon / _earth_neg | RECTIFY PBODY 0 / 2, negative TDELTAV | bit-exact |

ACCOMP/GAMCOMP/OBLATE are static in C (orbital_integration.c), so they
are checked through FV (the last stage's disturbing acceleration) and
the step results rather than by separate entries.
