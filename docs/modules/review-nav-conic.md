# review-nav-conic -- review of the nav-conic lane

## Scope

docs/agents/review.md for lane nav-conic (docs/modules/nav-conic.md):
src/nav/conic/*, include/agc/nav/conic.h, tests/nav/conic.c,
tests/oracle/conic.c, tools/oracle/cases/nav_conic{,_lm}.{spec,txt}.

Build note: the live tree has other lanes' edits in flight (and HEAD's
src/imu/mode.c does not compile), so the lane was built and tested in a
scratch snapshot (git archive HEAD + lane files overlaid; SCRATCH/
review-nav-conic/t.sh).

## Progress

| item | status |
|---|---|
| -0 / mixed-sign gap on circular orbits (review item) | DONE: yaAGC DV defect (F1); 5 DVBUG rows in tests/oracle/conic.c re-based on the patched emulator (tools/oracle/cases/nav_conic_dvfix.txt) |
| assembled TWEEKIT / GEOMSGN (review item) | DONE: rope-verified; internal.h constants now octal 027777 / 010000 with rope addresses |
| LEMCONIC SURFFLAG ownership (review item) | DONE: nav-integ owns CSMCONIC/LEMCONIC/USEPIOS; dead agc_conic_rvcon removed (conic.h, kepler.c, tests/nav/conic.c) |
| fidelity pass per routine | DONE (KEPLER, KEPPREP, ITERATOR, LAMBERT CSM/LM, TIMERAD read side by side; rest via oracle) |
| KEPPREP split (kepprep_series + enum kepprep_cell), TIMERAD timerad_eccvec helper, mneg_ removed, ABTCONIC passes BAILOUT | DONE |
| style 0 findings, <=80 cols: apsides, timetheta, timeradius, universal, kepler, lambert, geom, common, internal.h, conic.h | DONE (all compile; t.sh: oracle 299/0, unit 36/0) |
| check_rules src/nav/conic (rule-5 >= 2.0) | DONE (0 warnings) |
| tests/nav/conic.c style (45 findings: prologs, decl comments, main-first, ternary) | TODO |
| tests/oracle/conic.c: tolerance 16384/2^19 -> 1024 (kep_leo_2p25 8192), header comment, style (56) | TODO |
| make / make test / make check in live tree | TODO (live tree has other lanes mid-edit; HEAD imu/mode.c broken) |
| note: Findings table, Lessons, MODULE_STATUS row | TODO |

No file is mid-edit; every lane file compiles (snapshot build, SCRATCH/review-nav-conic/t.sh).
