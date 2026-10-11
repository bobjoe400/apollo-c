# review-dap-lm -- review of the LM DAP lane (src/lm/dap)

## Progress

* DONE fidelity (all lane files): accs.c, accont.c, arith.c, dapif.c,
  gts.c, state.c, rcs/{dapidler,needles,t6rupt,rcsmon,spsrcs,paxis,
  pmanual,qraxis,qrmanual,qrjets,tjetlaw,rate}.c. Findings R1-R10 below.
* DONE open questions: docked 1/ACCS COEFCTR (R1), BIGAOS 2 DB (R2);
  F9 at PITCHDAP is dap-csm (R10); sim B2 fixed (R3, R4).
* DONE oracle: new cases accs_docked_thrust/_aps/_fix, gts_a..f, tg_a..d,
  tjs_* (20), kal_docked/_docked_pw; oracle_dap_lm 1780 checks, 0 fail,
  0 KNOWN, tolerance 0 everywhere (tjet_u_large 1 -> 0 after R6).
  test_lm_dap 125 checks (test 8 added).
* DONE naming: static digit names renamed (zone1..5 -> zone_beyond_switch,
  zone_fire_to_axis, zone_coast_jets_on, zone_between_curves,
  zone_axis_time; ruflaw12/_entry/3 -> rough_law_jet_time/_large_error/
  _large_rate; subdivde3 -> measured_increment; needle11 ->
  needles_first_pass; do1net_plus -> net_accel_function;
  lm_dap_acdt_c12 -> lm_dap_gimbal_drive_update).
* DONE wrappers: lm_dap_alarm removed (R5); public agc_lm_dap_rot_touv
  removed from include/agc/lm/dap.h (test uses lm_dap_rot_touv).
* All touched files compile; make test of HEAD + this lane's files passes
  except oracle_landing guid_brak_b (fails at HEAD without this lane).
* TODO style: 1222 check_style findings in lane files (function prologs
  ~500, decl/scale comments ~455, ternary 113, long-block 39, one-decl 34,
  define 33, file prologs 22, qualifier/sizeof/main-first in tests).
  Prolog converter written (SCRATCH/review-dap-lm/style/prologs.py), NOT
  yet run on any file. No file is mid-edit.
* TODO: make format, make check, MODULE_STATUS row, Lessons section.
* DONE DMP/DMPR audit: lane AGC files (AOSTASK_AND_AOSJOB, DAPIDLER,
  DAP_INTERFACE_SUBROUTINES, KALMAN_FILTER, P-AXIS, Q_R-AXIS, TJET_LAW,
  TRIM_GIMBAL, RCS_FAILURE_MONITOR, SPS_BACK-UP_RCS, T6-RUPT) contain no
  interpretive code: 0 DMP/DMPR/VXSC/DDV/VSR/SRR sites; C uses no
  agc_dp_mul*. Count changed: 0.
* PARTIAL overflow-store audit (lm_dap_sat vs lm_dap_wrap, 49 sat sites;
  grep -rn "lm_dap_sat(" src/lm/dap). No file edited yet. yaAGC MP uses
  the overflow-corrected A (= wrap), TS/ADS/XCH store wrapped.
  - Correct as sat (OVERSUB/OVERSUB2/LIMITS/explicit POSMAX): rate.c
    167,218,349,357,365; qraxis.c 270-271; qrjets.c 214-215 (ROT-TOUV);
    needles.c 88-92 (OVERSUB2; confirm); pmanual.c 300 (OVERSUB after
    DAS A); accont.c m-path POSMAX (~line 465).
  - To change sat -> lm_dap_wrap (AGC stores via TS/ADS or feeds MP;
    all unreachable in range, so tests unaffected): accont.c 263
    (TS FUNTEM), 315, 384 (ADS DBB2), 397 (DOUBLE then MP DB), 426, 428,
    445, 489, 491 (TS AXDSTEM), 569 (TS PDB3), 624, 625, 663, 688
    (TS ANET); rate.c 297-298 (AD then MP -BIT14); t6rupt.c 212
    (DXCH T6FURTHA; 220 LASTCHG cannot overflow, either is fine).
  - Keep sat + comment as unreachable: pmanual.c 353, qrmanual.c 114
    (4|D|+LINRAT, RHC counts <= 42; wrap's |v|<=3 POSMAX assert would
    trip for test inputs near POSMAX); gts.c 181 ROOTCYCL (guess >= root
    so (x+arg/x)/2 <= x).
  - Still to check: rate.c 101-102 (ROTORQUE: AD then MP 1JACCR/Q ->
    likely wrap), accs.c 195, 395-401, tjetlaw.c 117,209,213,239,278,
    294,398,430,574.
  Then rerun make test (oracle_dap_lm tolerance 0) and record finding R11.

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
