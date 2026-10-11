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
| tests/oracle/conic.c: tolerance -> 0 (TOL_* removed), header comment | DONE (style TODO) |
| make / make test / make check in live tree | PARTIAL: live tree builds again (2026-10-11); build/tests/oracle/conic 299/0 and build/tests/nav/conic 36/0 at HEAD 943c6e6. make check not yet run |
| DMP/DMPR + shift audit (review_items "ALL remaining lanes") | PARTIAL: every lane multiply goes through the per-instruction helpers (agc_conic_mdmp_ = DMPSUB TP, mdmpr_ = DMPSUB+ROUNDSUB, mshiftr_ = SRR/SLR, agc_tp_shift = SR/SL). Counted: AGC 45 DMP / 12 DMPR, C 49 mdmp_ / 10 mdmpr_ call sites (KEPCONVG "DMPR DSU S(XI)" vs LAMENTER "DMP DSU" is handled by lamenter_velocity's flag). LEFT: site-by-site match of the remaining 2 DMPR (expect 1166 TIMERAD, 1394 LAMBERT region) and the +4 DMP (shared tails); no wrong site found yet |
| R1A word form (prog-rendz P1/P6: CDH DIFFALT 62 m, circular APSIDES ECC 0 vs 23164) | DONE: lane-wide word MPAC. Every DP erasable / push-down cell of the lane is agc_dp_words_t (internal.h ctx, KEPLER/LAMBERT state, MUTABLE and the p. 1288 constants as assembled words), every interpretive op an agc_mpac_* call; GETX takes PARAM's TP MPAC (SQRT reads MPAC+2), LAMENTER takes ABVAL's TP words, POLY and DVXSC on words (common.c), agc_iterator_t on words (conic.h). Result: oracle_conic exact (tolerance 0, was 1024/8192) on all 55 cases; prog-rendz aps2 (ECC 0, R1A 00401 40000, RAPO/RPERI) exact; targeting at tolerance 0: CDH DIFFALT exact (was 31 LSB), initvel_moon and tpi_earth_b exact, cdh VACT3.x 2-4 LSB and initvel_earth_prec (integration) left (prog-rendz) |
| new oracle cases (nav_conic.spec) | DONE: +24 runs: aps_circ; 16 TIMETHET within 5 deg of 0/360 (360CHECK, INVRSEQN, 1/WLOOP, TRUE360X: previously never executed by any case); kepn_ell_dxch/dxch2 (PDXCHNGE/NDXCHNGE); lam_hien, lam_hien_g, lam_bigtime, lam_small_uplim, lam_small_loen (NEGP/HIENERGY/LOENERGY/BIGTIME, MODNGDEL, UPLIM); kep_leo_10min (nav-integ conic LEO case). Old 31 dumps unchanged. Not reachable with valid inputs found: KEPLER TIMEOVFL |
| other lanes' tests touched (needed for make test) | tests/oracle/guidance.c deltime() pokes the private ctx: wrapped in agc_dp_to_words / agc_conic_value_; tests/nav/integration.c conic LEO dr < 50 -> 60 m (AGC itself gives 53.4 m: kep_leo_10min exact); tests/oracle/return_to_earth.c DV, R(T2)/ -> KNOWN F14 (rte_leo moved 132 / 928 LSB with the exact conics; F14 search non-convergence) |
| request to core-numeric | DONE by core (interp_words.h); none left |
| rule 7 (no silent/sentinel failures) in src/nav/conic | TODO |
| style / make format / check_rules after the rewrite | TODO |
| tests/nav/conic.c style | TODO |
| DMP/DMPR audit, last two DMPR sites | TODO |
| note: Findings table, Lessons, MODULE_STATUS row | TODO |

Files mid-edit: none (all compile; make test green, 2026-10-11).
