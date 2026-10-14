# review-core-numeric -- lane history

Process diary of the review-core-numeric lane, moved verbatim out of [docs/modules/review-core-numeric.md](../../docs/modules/review-core-numeric.md), which keeps the reader-facing explanation. Note dates: 2026-10-10 to 2026-10-13 (first and last commit touching the note).

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
| style findings, tests/oracle/core_numeric.c | DONE: format2 left 11 (scale comments, closing-brace comments): 0 findings |
| rule-5 density | DONE: check_rules 0 hard, 0 warnings on lane files |
| Findings table, Lessons, MODULE_STATUS row | DONE |
| word-level MPAC API | DONE (code): include/agc/core/interp_words.h + src/core/interp/words.c (+ trig.c SQRT/SIN/COS/ASIN/ACOS on words); word.h agc_dp_words_t/agc_tp_words_t + conversions; private model: word-level DDV/BDDV (agc_w_ddv on words, ddv_unit_major/bufzero/agree/quotient/normalise), GENDDV on words, SQRTSUB returns MPAC+2, TPAGREE in NORM, BRANCH; oracle core_words.spec (460 runs, gen_core_words.py) + tests/oracle/core_words.c: 2021 checks, 0 failures; core_numeric 361/0 unchanged. unit test test_mpac_words (R1A words, WLOOP W = 8724671); lane migration requests below |
| make / make test / make check | DONE for the lane: make, make test green (all suites, oracle_core_words 2021/0); make check fails only on other lanes' files (check-rules: csm/dap/tvc, programs/alignment/word_forms.c, csm/optics; format-check: src/lm/dap mid-edit by review-dap-lm); lane files 0 style findings, 0 rule warnings |
| agc_dp_sign SIGN semantics | DONE: confirmed (CCS major, CCSL minor when major is 0; COMP takes MPAC+2 too, agc_mpac_sign); oracle SIGN cases incl. major 0 |

Files mid-edit: none. Every touched file compiles; make and make test green.

## Findings

| # | where | AGC | finding | resolution |
|---|---|---|---|---|
| C1 | src/core/divide.c agc_w_ddv | DDV/BDDV pp.1162-1169 | the model split the dividend/divisor value into sign-agreeing words; the AGC forces agreement only for a +-1 divisor major word (BUFPOS/BUFNEG) or when the major words are within one (MPAC+), and GENDDV divides unagreed words ("SIGN AGREEMENT IS UNNECESSARY") | DDV/BDDV now run on words (BUFZERO with TPAGREE, BUF-/MPAC- DCS, MPAC+ test, BUFNORM DAS, DDOUBL prologue, GENDDV/MAXDV, FINALDV); value API unchanged (361/0); 64 random DDV/BDDV word cases + ATMAG/ASCENT/HUNTEST sequences exact |
| C2 | src/core/divide.c agc_w_sqrtsub, interp/trig.c | SQRT / SQRTSHFT p.1181 | the root's MPAC+2 was dropped (prog-rendz P2): after no shift the AGC leaves the argument's (normalised) MPAC+2, SQRTSHFT zeroes it before MPACSHR, GENSCR shifts the root words into it | agc_w_sqrtsub returns it; agc_mpac_sqrt keeps it; WLOOP "SQRT ROUND" exact |
| C3 | src/core/interp/trig.c acos_words | ACOSST2 p.1189 | ACOS/ASIN took a DP: the AGC's DCS MPAC / AD HALF leaves MPAC+2 uncomplemented and SQRTSUB normalises it in (nav-aux latlong 1 LSB); BRANCH tests three words | TP-word ARCCOS/ARCSIN (agc_mpac_asin/acos); 32 random + 8 SR1-ASIN cases exact |
| C4 | src/core/shift.c agc_w_tslc | TSLC2 p.1157 | NORM skipped TPAGREE (harmless for agc_tp_t, wrong for unagreed words) | TPAGREE (agc_w_tpagree) first |
| C5 | src/core/interp/trig.c | COSINE p.1187 | COS of words: BRANCH on all three words, DCS of two | cosine_words; SIN/COS return the TP MPAC (DMPSUB + two TP doublings) |
| C6 | include/agc/core | -- | four lane-local copies of a {hi, lo} word pair (agc_lm_words_t, agc_csm_dp_pair_t, lm_dp_pair_t, imu word_pair_t) | public agc_dp_words_t / agc_tp_words_t (word.h) and interp_words.h; migration is the lanes' call (Requests) |
| F1 | yaAGC | DV SpToDecent | -0 minor word taken as a borrow | oracle defect; 5 core_words runs dropped (stock and patched engines differ only there) |
| F2 | nav-aux MOONMX | NODI -> COS | sign-disagreeing NODI words into COS | agc_mpac_cos on the stored words (COS cases exact) |
| F4 | nav-aux LSPOS | LOS -> SIN | sign-disagreeing words into SIN | agc_mpac_sin |
| F5 | imu DELCOMP/DCMTOCDU | BDSU DPHALF -> DMP | 1-cos A words (00001, 40057) | DELCOMP prefix (SIN, COS, SR2, BDSU, COF DSQ DMP DAD SL3) exact in 10 cases with the word API |
| F6 | imu GCOMP | DAS | already exact with the lane's word_pair_t | agc_dp_words_das is the shared form |
| F13 | nav-conic GETX/APSIDES | PARAM BDSU, WLOOP, GETECC | R1A = (00401, 40000) | WLOOP passes 329590 -> 8724671 -> 20827920 (the AGC W of prog-rendz P1) and GETECC ECC 0 (circular) exact with the word API |
| E1 | guid-entry HUNTEST1 | DLOAD A0 DMP 2C1HS DDV V1 DDV V1 DDV LEWD | entry reported AGC 13571567 vs C 13571613 | with those values as sign-agreeing words yaAGC gives 13571613 (= C, case HUNTEST1); the entry difference therefore comes from the stored words of A0/V1/LEWD in the full run, not from DDV; trace them and use agc_mpac_* |

## Requests (to other lanes)

* nav-conic (F13, prog-rendz P1/P6): keep R1A (PARAM, LAMBERT), COGA, X, KEPC2 as agc_dp_words_t and run WLOOP, POLYCOEF, XCOMMON, TRUE360X, GETECC, TIMERAD on agc_mpac_*; the private-model plan in review-nav-conic.md maps one to one (agc_w_* -> agc_mpac_*). Do not include src/core/mpac.h.
* imu (F5): DELCOMP/CDUTODCM/DCMTOCDU on agc_mpac_* (SIN/COS TP MPAC, SR2, BDSU DPHALF, DMP); replace word_pair_t by agc_dp_words_t and agc_dp_words_das.
* guid-landing (R1, finding 15): TBUP/TGO as agc_dp_words_t; ATMAG/MAINENG/ASCENT/YOK DDV and SR 11D on agc_mpac_ddv / agc_mpac_shift (sequences exact in core_words); agc_lm_words_t can become agc_dp_words_t.
* guid-entry (E1, AUGEKUGL/PREC155): HUNTEST1 chain and DOT-then-SL1 (agc_mpac_dot, agc_mpac_shift) on the stored words.
* nav-aux (F2, F4, latlong): agc_mpac_cos/sin on NODI and LOS words; ARCTAN SR1 then ASIN via agc_mpac_shift + agc_mpac_asin.
* prog-align: src/programs/alignment/word_forms.c re-derives ARCCOS/ASINEX word forms (and has 16 rule-7 FAILs); agc_mpac_acos/asin give the words directly.
* dap-csm, dap-lm: agc_csm_dp_pair_t / lm_dp_pair_t are agc_dp_words_t; DAS is agc_dp_words_das.
* lead: docs/TRANSLATION_GUIDE.md section 1 should name agc_dp_words_t and interp_words.h ("a DAS/DSU/BDSU result re-read by DMP/DDV/SR/SQRT/SIN: keep the words").

## Lessons

* The interpreter forces sign agreement only in a few places (DDV's MPAC+ when the majors are within one, BUFPOS/BUFNEG for a +-1 divisor major, TPAGREE in NORM and BUFZERO, VECAGREE in V/SC); everything else (DAS, DMPSUB, GENDDV, SQRTSUB, SN1, POLY) works on the words as they are. A value model is exact only where one of those agreement points precedes the use.
* MPAC+2 is live state: DLOAD zeroes it, DAD/DSU/BDSU/DCS of two words and the ARCSIN/ARCCOS epilogues leave it, SQRT/SR/DMP/DOT write it, and ROUND, SR, NORM, SQRTSUB and TAD read it. "X then ROUND" or "X then SQRT" needs the TP words.
* An interpretive op with its address on the push-down list must be followed by an op line in YUL source; "DAD EXIT" at the end of a snippet assembles DAD with an address word (it then takes the TC ENDOFJOB word as its operand and the run hangs).
* Mutation check of the oracle test: forcing DDV agreement fails 36 checks (DDV, BDDV, ATMAG, ASCENT, HUNTEST); zeroing ACOS's stale MPAC+2 fails 13 (ASIN, ACOS). The random word cases are sensitive to exactly the effects the lanes reported.

