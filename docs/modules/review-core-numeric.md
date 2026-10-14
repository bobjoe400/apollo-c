# review-core-numeric -- review of the core-numeric lane

History of this lane: [docs/history/lanes/review-core-numeric.md](../history/lanes/review-core-numeric.md)

## Scope

docs/history/assignments/review.md for lane core-numeric (docs/modules/core-numeric.md):
src/core/{mpac.c,mpac.h,divide.c,shift.c,word.c,sp_trig.c,constants.c,
vec_div.c}, src/core/interp/{scalar,trig,vector}.c,
include/agc/core/{interp.h,word.h,constants.h}, tests/core/numeric.c,
tests/oracle/core_numeric.c, tools/oracle/cases/core_numeric.{spec,txt};
added by this review: include/agc/core/interp_words.h,
src/core/interp/words.c, tests/oracle/core_words.c,
tools/oracle/cases/core_words.{spec,txt}, tools/oracle/cases/gen/gen_core_words.py.
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

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [core-numeric](core-numeric.md#program-flow-and-decisions).
