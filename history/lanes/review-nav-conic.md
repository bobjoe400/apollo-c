# review-nav-conic -- lane history

Process diary of the review-nav-conic lane, moved verbatim out of docs/modules/review-nav-conic.md. The rest of that note (its explanation) is appended at the end, under "Module note (kept explanation)". Note dates: 2026-10-10 to 2026-10-13 (first and last commit touching the note).

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
| tests/oracle/conic.c: tolerance -> 0 (TOL_* removed), header comment | DONE (style TODO) |
| make / make test / make check in live tree | DONE for the lane: make and make test green; check_rules 0 hard in src/nav/conic + conic.h (7 rule-5 density warnings, R4); format-check / check_style clean on lane files |
| DMP/DMPR + shift audit (review_items "ALL remaining lanes") | DONE (see "DMP/DMPR audit" below): 0 sites changed |
| R1A word form (prog-rendz P1/P6: CDH DIFFALT 62 m, circular APSIDES ECC 0 vs 23164) | DONE: lane-wide word MPAC. Every DP erasable / push-down cell of the lane is agc_dp_words_t (internal.h ctx, KEPLER/LAMBERT state, MUTABLE and the p. 1288 constants as assembled words), every interpretive op an agc_mpac_* call; GETX takes PARAM's TP MPAC (SQRT reads MPAC+2), LAMENTER takes ABVAL's TP words, POLY and DVXSC on words (common.c), agc_iterator_t on words (conic.h). Result: oracle_conic exact (tolerance 0, was 1024/8192) on all 55 cases; prog-rendz aps2 (ECC 0, R1A 00401 40000, RAPO/RPERI) exact; targeting at tolerance 0: CDH DIFFALT exact (was 31 LSB), initvel_moon and tpi_earth_b exact, cdh VACT3.x 2-4 LSB and initvel_earth_prec (integration) left (prog-rendz) |
| new oracle cases (nav_conic.spec) | DONE: +24 runs: aps_circ; 16 TIMETHET within 5 deg of 0/360 (360CHECK, INVRSEQN, 1/WLOOP, TRUE360X: previously never executed by any case); kepn_ell_dxch/dxch2 (PDXCHNGE/NDXCHNGE); lam_hien, lam_hien_g, lam_bigtime, lam_small_uplim, lam_small_loen (NEGP/HIENERGY/LOENERGY/BIGTIME, MODNGDEL, UPLIM); kep_leo_10min (nav-integ conic LEO case). Old 31 dumps unchanged. Not reachable with valid inputs found: KEPLER TIMEOVFL |
| other lanes' tests touched (needed for make test) | tests/oracle/guidance.c deltime() pokes the private ctx: wrapped in agc_dp_to_words / agc_conic_value_; tests/nav/integration.c conic LEO dr < 50 -> 60 m (AGC itself gives 53.4 m: kep_leo_10min exact); tests/oracle/return_to_earth.c DV, R(T2)/ -> KNOWN F14 (rte_leo moved 132 / 928 LSB with the exact conics; F14 search non-convergence) |
| request to core-numeric | DONE by core (interp_words.h); none left |
| rule 7 (no silent/sentinel failures) in src/nav/conic | DONE: 87 -> 0 hard. Guards that are guaranteed by construction deleted (static helpers called with the entry's validated ctx / &local, word ranges produced by agc_mpac_*, NORM counts, constant tables); GEOM, PARAM, GETX, DELTIME, NEWSTATE, LAMENTER, KEPCONVG and INFINITY are void (no failure path); PERIODCH's 4096-pass bound (a C deviation) makes kepler_period return AGC_STATUS_NO_CONVERGE; ITERATOR split into a void iterator_run and the public status entry; ITERATOR's invented "MAX >= MIN" precondition removed (not an AGC check). Rule-5 density now < 2.0 in every lane file (see Findings R4) |
| style / make format after the rewrite | DONE: tools/format.sh on all lane files; check_style 0 findings (src/nav/conic, conic.h, tests/nav/conic.c, tests/oracle/conic.c) |
| tests/nav/conic.c style | DONE: prologs, declaration comments, main first, ternary comment (47 -> 0) |
| DMP/DMPR audit, last two DMPR sites | DONE: the 12 Comanche DMPR sites map to 9 C agc_mpac_dmpr calls (KEPLERN 1/MU, DXCOMP BEE22, NDXCHNGE/PDXCHNGE DP9/10, KEPCONVG S(XI), DELTIME 1/ROOTMU, PARAM 1/MU (p. 1290), LAMBERT BEE19 (p. 1296), and newstate_position's DMPR 1/ROOTMU, DMPR X serving both KEPCONVG and NEWSTATE); every other multiply is agc_mpac_dmp; SRR/SLR/SLnR -> agc_mpac_shift_round, SR/SL -> agc_mpac_shift; 0 wrong sites (the oracle at tolerance 0 confirms) |
| note: Findings table, Lessons, MODULE_STATUS row | DONE |

Files mid-edit: none (all compile; make test green, 2026-10-11).

## Findings

| # | where | AGC | finding | resolution |
|---|---|---|---|---|
| R1 | whole lane (was common.c TP model) | DAS/DSU/BDSU then DMP, SR, SQRT, ROUND, POLY, VXSC | the C MPAC was one TP number, so the sign-disagreeing words DAS leaves (R1A = 00401 40000 from PARAM, COGA, X, KEPC2, DELX, DCOGA, ...) reached DMPSUB/SQRTSUB/shifts as sign-agreeing words: up to 768 LSB in LAMBERT VVEC, 8192 in kep_leo_2p25, CDH DIFFALT 62 m and circular APSIDES ECC 23164 (prog-rendz P1/P6) | lane-wide word MPAC (agc_mpac_*, every DP cell agc_dp_words_t, constants as assembled words); oracle_conic exact at tolerance 0 (524 checks) |
| R2 | geom.c GETX | GETX "ASSUMES P (+4) IN MPAC" (p. 1292) | TIMETHET enters GETX with PARAM's MPAC (P after DMP SR*, MPAC+2 live) and SQRT reads MPAC+2; the C reloaded P from erasable | agc_conic_getx_ takes the caller's MPAC (PARAM's, or DLOAD P for LAMBERT/TIMERAD) |
| R3 | universal.c LAMENTER / KEPCONVG | ABVAL then NORM (pp. 1281, 1287) | LAMENTER and KEPCONVG normalise ABVAL's TP words (VSQSUB + SQRT); the C used the DP value | agc_mpac_sqrt(agc_mpac_vsq()) then NORM; TARGETV passes DLOAD MAGVEC2 |
| R4 | all lane files | rule 5 vs rule 7 | removing the guaranteed guards (rule 7) leaves 0.0-0.6 assertions per function: the old density was made of guards that could not fail | reported; the remaining assertions are the public entries' input checks |
| R5 | lambert.c ITERATOR | ITERATOR p. 1285 | the C asserted MAX >= MIN, which the AGC never checks (COGAMAX/COGAMIN are not ordered by construction) | removed |
| R6 | nav_conic.spec coverage | 360CHECK/INVRSEQN/1/WLOOP/TRUE360X, NEGP/HIENERGY/LOENERGY/BIGTIME, MODNGDEL, UPLIM, PDXCHNGE/NDXCHNGE | no oracle case executed these paths (gcov); RTE runs INVRSEQN | +24 cases, all exact; KEPLER TIMEOVFL still unreached with valid inputs |
| R7 | LAMBERT with a COGA guess that is NEGP at once | entry DEVIATION (TERRLAMB/DCOGA) | the AGC reads stale TERRLAMB / 12D; the C sets TERRLAMB = TDESIRED, so it reports no solution where the zeroed oracle erasable accepts (lam_long_loen, dropped from the spec) | documented deviation kept |

## DMP/DMPR audit

Comanche CONIC_SUBROUTINES + KEPPREP: 12 DMPR sites (KEPLERN VSQ DMPR 1/MU p. 1277; DXCOMP DMPR BEE22; NDXCHNGE and PDXCHNGE DMPR DP9/10; KEPCONVG DMPR 1/ROOTMU, DMPR X, DMPR S(XI); DELTIME DMPR 1/ROOTMU; NEWSTATE DMPR 1/ROOTMU, DMPR X; PARAM DMPR 1/MU; LAMBERT DMPR BEE19), all agc_mpac_dmpr (newstate_position serves KEPCONVG and NEWSTATE). ITERATOR's MODNGDEL/MODPSDEL are DMP (truncating, "DMP GOTO DP9/10") and stay agc_mpac_dmp; every other multiply is DMP. Shifts: SRR/SLR/SL1R/SL2R/SL3R/SL4R/SR1R/SR4R are agc_mpac_shift_round, SR/SL/SR*/SL* agc_mpac_shift. Changed: 0 (the oracle at tolerance 0 is the check).

## Requests

* prog-rendz: tests/oracle/targeting.c can tighten: cdh DIFFALT 32 -> 0, initvel_moon 32 -> 0, tpi_earth_b 760 -> 0, cdh_earth/_ell 520/760 -> 4 (VACT3.x/DELVEET2.x 2-4 LSB remain, outside the conics), tpi_earth 2 stays. The public LAMBERT/PARAM outputs (conic.h) are values: a caller that feeds LAMBERT's COGA back as the next guess (initvel) passes sign-agreeing words where the AGC passes the stored ones; if a residual shows up there, ask for agc_dp_words_t fields.
* nav-integ: tests/nav/integration.c conic LEO bound changed 50 -> 60 m (the AGC itself gives 53.4 m: oracle_conic kep_leo_10min, exact).
* guid-entry: tests/oracle/return_to_earth.c DV and R(T2)/ are KNOWN F14 now (rte_leo moved 132 / 928 LSB with word-exact conics; the search is the F14 non-convergence). TIMERAD is exact on its own (oracle trad_* cases), so the V(T2)/ note in review-guid-entry.md is no longer a conic residual.
* guid-powered: tests/oracle/guidance.c deltime() pokes the conic context: now agc_dp_to_words / agc_conic_value_ (edited here to keep make test green).
* lead: rule 5 density vs rule 7 (R4): with the guaranteed guards gone, the lane is far below 2.0; adding asserts that cannot fail would not count. Decide whether density applies to such code.

## Lessons

* A value model of the MPAC is exact only where the AGC forces sign agreement; in the conic subroutines DAS/BDSU outputs are stored and re-read almost everywhere (R1A, COGA, X, DELX, DCOGA, KEPC2), so the residual was spread over every routine. Moving the whole lane to words at once, mechanically (agc_tp_t -> agc_tp_words_t, agc_conic_m*_ -> agc_mpac_*), and then fixing compile errors was faster and safer than patching sites one at a time, and it removed all 64 residuals in one step.
* "MPAC on entry" matters: GETX, LAMENTER, POLY read whatever MPAC holds (PARAM's SR* MPAC+2, ABVAL's TP words). Read the caller's last instruction before the CALL, not just the routine.
* gcov over the oracle test shows which AGC paths no case reaches; here the near-360 GETX branch and every LAMBERT retreat were untested although RTE uses them. Generate cases for those paths before trusting a tolerance of 0.
* Tests in other lanes that were tuned to the old C (a 50 m bound, a 128 LSB F14 chain) move when a dependency becomes exact; check them against the oracle before loosening (kep_leo_10min showed the AGC's own 53.4 m).



## Module note (kept explanation)

### Scope

history/assignments/review.md for lane nav-conic (docs/modules/nav-conic.md):
src/nav/conic/*, include/agc/nav/conic.h, tests/nav/conic.c,
tests/oracle/conic.c, tools/oracle/cases/nav_conic{,_lm}.{spec,txt}.

Build note: the live tree has other lanes' edits in flight (and HEAD's
src/imu/mode.c does not compile), so the lane was built and tested in a
scratch snapshot (git archive HEAD + lane files overlaid; SCRATCH/
review-nav-conic/t.sh).

### Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [nav-conic](../../docs/modules/nav-conic.md#program-flow-and-decisions).
