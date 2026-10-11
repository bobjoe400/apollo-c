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
| make / make test / make check in live tree | PARTIAL: live tree builds again (2026-10-11); build/tests/oracle/conic 299/0 and build/tests/nav/conic 36/0 at HEAD 943c6e6. make check not yet run |
| DMP/DMPR + shift audit (review_items "ALL remaining lanes") | PARTIAL: every lane multiply goes through the per-instruction helpers (agc_conic_mdmp_ = DMPSUB TP, mdmpr_ = DMPSUB+ROUNDSUB, mshiftr_ = SRR/SLR, agc_tp_shift = SR/SL). Counted: AGC 45 DMP / 12 DMPR, C 49 mdmp_ / 10 mdmpr_ call sites (KEPCONVG "DMPR DSU S(XI)" vs LAMENTER "DMP DSU" is handled by lamenter_velocity's flag). LEFT: site-by-site match of the remaining 2 DMPR (expect 1166 TIMERAD, 1394 LAMBERT region) and the +4 DMP (shared tails); no wrong site found yet |
| R1A word form (prog-rendz P1/P6: CDH DIFFALT 62 m, circular APSIDES ECC 0 vs 23164) | PARTIAL (analysis done, no code yet). Findings: R1A from PARAM is "PUSH BDSU D1/32" = DAS of D1/32 and the complemented MPAC words -> (512-hi, -lo), sign-disagreeing; LAMBERT's R1A is "SR* 0 -8D,1 BDSU D1/32" on a TP with MPAC+2 left; LAMBERT's COGA ("DAD COGA STCALL COGA"), KEPLER's X ("DAD X") and KEPC2 ("BDSU D1/64" of R1A words: (-1, +16383) for the circular case) are also DAS-stored and re-read by DSQ/DMP/SQRT -- the general source of the 768-LSB residual. Core has the word API only privately (src/core/mpac.h: agc_w_dmpsub/dsqsub/das/dad/dcs/sqrtsub/mpacshr/genscr/tssl/tslc/roundsub/poly); core value ops are exact wrappers of it for sign-agreeing words, so a lane word model is a drop-in. PLAN: (1) add word-level helpers in common.c on agc_w3_t (include "../../core/mpac.h", precedent lm/radar -> programs/navigation/internal.h; request core-numeric to publish it), SQRT keeping MPAC+2 per prog-rendz P2 via agc_w_sqrtsub + agc_w_genscr; (2) ctx->r1a as agc_w2_t words, produced by PARAM and LAMBERT as the AGC does; (3) word versions of WLOOP (SR4 TAD SQRT ROUND DAD), 1/WLOOP, POLYCOEF DMP R1A, TRUE360X SQRT R1A, XCOMMON DMP R1A + KEPC2, APSIDES GETECC + NORM R1A, TIMERAD's two R1A uses; (4) check with SCRATCH/review-prog-rendz/aps2.spec (AGC ECC 0, R1A words) and tests/oracle/targeting.c cdh + initvel_earth_far (P1 noted the WLOOP-only fix broke VTPRIME.x by 512 because LAMBERT's R1A word form was not carried); (5) if residuals remain, extend to COGA (LAMBERT), X/DELX (KEPLER), KEPC2, i.e. a lane-wide word MPAC (agc_conic_m*_ on agc_w3_t), then cut TOL_DEFAULT |
| request to core-numeric | TODO (write in Requests): publish the word-level MPAC API (or bless the private include) and a TP SQRT that keeps MPAC+2 (same as prog-rendz P2) |
| note: Findings table, Lessons, MODULE_STATUS row | TODO |

No file is mid-edit; git tree clean for the lane at 943c6e6 (no uncommitted kepler.c edits remain); every lane file compiles in the live tree (make build/tests/oracle/conic build/tests/nav/conic). Paused 2026-10-11 on coordinator request before starting the R1A code change.
