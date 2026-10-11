# review-dap-lm -- review of the LM DAP lane (src/lm/dap)

## Progress

* DONE (earlier run, commit 4f35a43): accs_aos_* oracle cases (13), AOSV -0
  modelled in 1/ACCONT (F10/F11 now bit-exact), INVERT overflow path,
  DV |dividend| = |divisor| case.
* DONE: docked 1/ACCS traced in the rope (R1); comment fixed; three docked
  oracle cases added (accs_docked_thrust/_aps/_fix), 1478 checks pass.
* DONE: BIGAOS 2 DB confirmed in the rope (R2).
* DONE: sim B2 (zero erasable): improper DV modelled at register level
  (port of yaAGC SimulateDV), TS/XCH stores wrap instead of saturate in
  1/ACCS and 1/ACCONT, data-dependent asserts replaced by word-range
  checks; unit test 8 (126 checks).
* DONE fidelity: accs.c, accont.c, dapif.c, dapidler.c, needles.c,
  t6rupt.c, rcsmon.c, spsrcs.c, paxis.c, pmanual.c, qraxis.c, qrmanual.c,
  qrjets.c, gts.c (+ 10 GTS/TIMEGMBL oracle cases, bit-exact: 1592 checks).
* DONE: tjetlaw.c (20-case oracle sweep found R6), rate.c (2 docked
  Kalman cases), state.c (fresh start / restart constants and order).
  oracle_dap_lm 1780 checks, 0 failures, 0 KNOWN, tolerance 0 everywhere.
* Next: style (1222 check_style findings in lane files), naming, wrappers.
* Build/test now in a HEAD worktree (SCRATCH/review-dap-lm/wt, sync.sh):
  the shared tree has other lanes' WIP. oracle_landing guid_brak_b TTF/8
  fails at HEAD without this lane's changes (not dap-lm).

## Scratch build

Other review lanes edit the tree concurrently; SCRATCH/review-dap-lm/build.sh
builds a debug lib (falls back to HEAD for files that do not compile) and
t.sh runs one test against it.

## Findings

| # | Where | AGC | Finding | Action |
|---|---|---|---|---|
| R1 | accs.c docked() | DOCKED / SPSLOOP2 p.1495 | Open question resolved: DIM COEFCTR takes 1 to -0 (ones' complement 1 + -1), so CCS COEFCTR takes the fourth branch over the TORQCONS words to CA MPAC; the CG pass leaves COEFCTR = 6 and exits. Traced in the rope (yaAGC). The C already followed this intent. | DEVIATION comment replaced by the explanation; oracle cases accs_docked_thrust/_aps/_fix added (ACCDOTQ/R, KQ from the CG fit; docked ascent; docked MASSFIX) |
| R2 | accont.c BIGAOS | p.1499 | Open question resolved: DB4(3) = 2 DB (ADS leaves the sum in A), the listing comment's 1 DB is wrong. Rope: accs_aos_big COASTDB = 00310 with DB = 00144. | comment cites the rope |
| R3 | arith.c lm_dap_dvqr | DV | Improper DV (and zero divisor) asserted and returned 0; the AGC hardware gives a deterministic "nonsense" result. | register-level model (port of yaAGC SimulateDV); unit test checks the rope's BIGIQ divide (77470,0 / 77652 -> 37777, 77652) |
| R4 | accs.c, accont.c | TS / XCH / AD before MP | sim B2: 1/ACCS with the zero erasable of power-on ended BAD_ARG on data-dependent asserts (negative 1JACCU, improper INVERT). AGC stores overflow-corrected words and carries on. | TS 1JACC, TS COEFFQ/R, JACCUV AD, XCH ANET now wrap; asserts reduced to word-range checks; job completes (unit test 8). Remaining difference documented as DEVIATION in lm_alone(): the AGC's STCTR1 TS overflow skip reruns the loop with indexes below the tables (meaningless values replaced by the next 1/ACCS) |
| R5 | dapif.c lm_dap_alarm | TC ALARM | Forwarding wrapper left over from the status sweep (agc_alarm is void; wrapper returned OK always) | removed; 5 callers call agc_alarm directly |
| R6 | tjetlaw.c ruflaw12 | RUFLAW12 p.1468 | The three DAS A double the whole A,L product; the C multiplied only the major word by 8, losing minor-word carries (TJ 1 LSB low: tjet_u_large, tjs_v_hugeerr) | DP product used; tjet_u_large tolerance 1 -> 0 |
| R7 | paxis.c lm_dap_pretimck | PRETIMCK | Tautological postcondition (`... || true == ...`) | replaced by jet-counter range check |
| R8 | qraxis.c QERRCALC | p.1451 | TS E leaves E = (CDUY-CDUYD) M31 as a side effect; not modelled. E is always rewritten (TJLAW, CALCPERR, pseudo-auto) before any read | recorded only |
| R9 | qrjets.c BACKHAND | p.1449 | DAS A / AD Q in a 16-bit A can wrap past the overflow test for |1/ANET x rate| > 10922 (~30 deg/s); C uses exact arithmetic | recorded only (unreachable rates) |
| R10 | review item F9 at PITCHDAP | -- | PITCHDAP is the CSM TVC DAP (src/csm/dap/tvc/daps.c), not an LM routine | belongs to dap-csm; nothing in this lane |

Oracle cases added by this review: accs_aos_* (13, earlier run), accs_docked_thrust/_aps/_fix, gts_a..f, tg_a..d, tjs_* (20), kal_docked, kal_docked_pw.
