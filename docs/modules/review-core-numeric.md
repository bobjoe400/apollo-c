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
| stale DVSIGN (V/SC, UNIT early DVOVF) | DONE: modelled (agc_interp_t.dvsign_neg), 20 oracle snippets; unit test test_stale_dvsign |
| UNIT short vectors at B27 (R63) | DONE: AGC UNIT is equally coarse; C bit-exact (oracle UNIT 2000,0,0 etc.); R63 uses NORMUNIT (prog-align A1); note in interp.h |
| oracle F1-F6 | DONE (analysis in Findings) |
| constants vs assembled ropes | DONE: 265 SP label/rope pairs, DP/vector pool, SN1/ACOS3 coefs, SQRT SLOPE/BIAS: 0 mismatches |
| new oracle cases | DONE: +44 (SLR, TP SL/SR/NORM, SHORTMP, DMPNSUB, POWRSERS) appended to core_numeric.spec, .txt regenerated; 361 cases, 0 mismatches |
| fidelity read per file | DONE: sp_trig mpac shift divide vec_div word interp/scalar interp/trig interp/vector (MXM3 CM/LM text differs, same bits) |
| forwarding wrappers | DONE: one (agc_sp_sin), kept: sp_sine takes the 16-bit SPCOS accumulator |
| style findings, src/include lane files | DONE: 0 findings in all 11 .c, mpac.h, word.h, interp.h, constants.h |
| style findings, tests/core/numeric.c | DONE: 0 findings (main first, prologs, new test_stale_dvsign) |
| style findings, tests/oracle/core_numeric.c | TODO: ~94 findings; file compiles and passes as is. Plan: rewrite as main-first with scalar_op/shift_op/vector_op/read_line helpers (draft content was written in session, not saved) |
| rule-5 density | DONE: check_rules 0 hard, 0 warnings on lane files |
| Findings table, Lessons, MODULE_STATUS row | TODO |
| make / make test / make check | lane build via SCRATCH/review-core-numeric/build.sh green; full make blocked by other lanes (imu/mode.c) |

Files mid-edit: none. Every touched file compiles (build.sh green).

