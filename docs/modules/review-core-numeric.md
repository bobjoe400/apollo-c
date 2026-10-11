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

## Word-level MPAC API (design, 2026-10-11)

Requested by nav-aux, imu, prog-rendz, guid-entry, guid-landing, nav-conic
(review_items core-numeric line). agc_dp_t keeps a value; the AGC keeps
words, and DAS/DCS/BDSU leave pairs whose signs disagree (R1A = (00401,
40000) from PARAM's BDSU, 1-cos A = (00001, 40057), TBUP/TGO after DSU,
GCOMP (49, -14827)). The low bits of a following DMP, DDV, SQRT, SR, ROUND
or NORM depend on that split. The private model (src/core/mpac.h) already
works on words; the public API exposes it, one function per interpretive
instruction, with MPAC as three words.

| item | declaration (include/agc/core/word.h, include/agc/core/interp_words.h) | AGC |
|---|---|---|
| DP pair as stored | `agc_dp_words_t {agc_sp_t hi, lo}` (word.h) | any DP erasable |
| TP / MPAC | `agc_tp_words_t {agc_sp_t hi, mid, lo}` (word.h) | MPAC, MPAC+1, MPAC+2 |
| conversions | `agc_dp_to_words(v)`, `agc_dp_from_words(w)`, `agc_tp_to_words(v)`, `agc_tp_from_words(m)` (word.h) | sign-agreeing split / value |
| load / store | `agc_mpac_dload(x)` (MPAC+2 = 0), `agc_mpac_store(m)` (MPAC, MPAC+1 as they are) | DLOAD, STORE |
| add | `agc_mpac_dad / dsu / bdsu(ip, m, x)`, `agc_mpac_tad(ip, m, t)` | DAS MPAC + OVERFLOW; TAD |
| multiply | `agc_mpac_dmp(m, x)` (TP), `agc_mpac_dmpr(ip, m, x)`, `agc_mpac_dsq(m)` | DMPSUB, +ROUNDSUB, DSQSUB |
| divide | `agc_mpac_ddv(ip, m, x)`, `agc_mpac_bddv(ip, m, x)` | DDV/BDDV word for word (BUFPOS/BUFNEG sign forcing, MPAC+ agreement only when majors are within 1, BUFNORM DAS, DDOUBL prologue, GENDDV on unagreed words, FINALDV); DVSIGN kept in ip |
| root | `agc_mpac_sqrt(ip, m)` | SQRTSUB on the TP; MPAC+2 left as the AGC leaves it (input's after no shift, MPACSHR's low bits, GENSCR's) |
| shifts | `agc_mpac_shift(ip, m, n)` (SL n > 0, SR -n), `agc_mpac_shift_round(ip, m, n)` (SLR/SRR), `agc_mpac_round(ip, m)`, `agc_mpac_norm(m, &n)` | TSSL, GENSCR/MPACSHR, ROUNDSUB, TPAGREE+TSLC |
| sign | `agc_mpac_dcomp(m)`, `agc_mpac_abs(m)`, `agc_mpac_sign(m, x)` | DCOMP/COMP, ABS (BRANCH), SIGN |
| trig | `agc_mpac_sin / cos / asin / acos(ip, m)` returning the TP MPAC | SINE/COSINE (BRANCH on three words), ARCCOS/ARCSIN (MPAC+2 stale into SQRTSUB) |
| vector | `agc_mpac_dot(ip, u, v)`, `agc_mpac_vsq(ip, v)` | DOTSUB, VSQSUB TP words |

Rules: every function takes and returns words exactly as the AGC holds
them (no sign agreement unless the AGC forces it); -0 is not modelled; the
value API (interp.h) stays the common path and its results are unchanged
(it is the same code on sign-agreeing words). Verification: an oracle spec
(tools/oracle/cases/core_words.spec, gen_core_words.py) runs each
instruction on random sign-disagreeing MPAC/operand words in yaAGC, plus
the lane sequences: PARAM BDSU + WLOOP (SR4 TAD SQRT ROUND DAD) and
POLYCOEF DMP R1A (nav-conic), DELCOMP 1-cos A BDSU + DMP (imu), ascent
TBUP/TGO DSU, SR 11D, DDV (guid-landing), entry HUNTEST1 DMP DDV DDV DDV
(guid-entry), COS of NODI words and ARCSIN with stale MPAC+2 (nav-aux).

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
| word-level MPAC API | DONE (code): include/agc/core/interp_words.h + src/core/interp/words.c (+ trig.c SQRT/SIN/COS/ASIN/ACOS on words); word.h agc_dp_words_t/agc_tp_words_t + conversions; private model: word-level DDV/BDDV (agc_w_ddv on words, ddv_unit_major/bufzero/agree/quotient/normalise), GENDDV on words, SQRTSUB returns MPAC+2, TPAGREE in NORM, BRANCH; oracle core_words.spec (460 runs, gen_core_words.py) + tests/oracle/core_words.c: 2021 checks, 0 failures; core_numeric 361/0 unchanged. TODO: full make/test/check, lane migration notes |
| make / make test / make check | lane build via SCRATCH/review-core-numeric/build.sh green; full make blocked by other lanes (imu/mode.c) |

Files mid-edit: none. Every touched file compiles (build.sh green).

