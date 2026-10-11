# review-core-numeric -- review of the core-numeric lane

## Scope

docs/agents/review.md for lane core-numeric (docs/modules/core-numeric.md):
src/core/{mpac.c,mpac.h,divide.c,shift.c,word.c,sp_trig.c,constants.c,
vec_div.c}, src/core/interp/{scalar,trig,vector}.c,
include/agc/core/{interp.h,word.h,constants.h}, tests/core/numeric.c,
tests/oracle/core_numeric.c, tools/oracle/cases/core_numeric.{spec,txt}.
Review items (docs/agents/review_items.md): UNIT precision at B27 for short
vectors (R63); oracle F1-F6; stale-DVSIGN deviation on V/SC and UNIT.

Lane build while other lanes have the tree broken: SCRATCH/review-core-numeric/
build.sh compiles the eleven core-numeric sources with both tests.

## Progress

| item | status |
|---|---|
| stale DVSIGN (V/SC, UNIT early DVOVF) | DONE: modelled (agc_interp_t.dvsign_neg), 20 oracle snippets |
| UNIT short vectors at B27 (R63) | DONE: AGC UNIT is equally coarse; C bit-exact; R63 uses NORMUNIT (prog-align A1) |
| oracle F1-F6 | TODO |
| fidelity pass per AGC routine | TODO |
| forwarding wrappers in lane | TODO |
| style findings to zero (777 at start) | TODO |
| rule-5 density (trig.c 1.88, vector.c 1.95) | TODO |
