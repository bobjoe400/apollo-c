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
| -0 / mixed-sign gap on circular orbits (review item) | DONE: yaAGC DV defect (F1), rows re-based on patched emulator |
| assembled TWEEKIT / GEOMSGN (review item) | DONE: rope-verified |
| LEMCONIC SURFFLAG ownership (review item) | DONE: nav-integ owns; agc_conic_rvcon removed |
| fidelity pass per routine | DONE (see Findings) |
| style: apsides, timetheta, timeradius, universal, kepler, lambert, geom | DONE (0 findings) |
| style: common.c, internal.h, conic.h, tests, oracle tolerance | TODO |
