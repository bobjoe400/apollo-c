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
* DONE DMP/DMPR audit: lane AGC files (AOSTASK_AND_AOSJOB, DAPIDLER,
  DAP_INTERFACE_SUBROUTINES, KALMAN_FILTER, P-AXIS, Q_R-AXIS, TJET_LAW,
  TRIM_GIMBAL, RCS_FAILURE_MONITOR, SPS_BACK-UP_RCS, T6-RUPT) contain no
  interpretive code: 0 DMP/DMPR/VXSC/DDV/VSR/SRR sites; C uses no
  agc_dp_mul*. Count changed: 0.
* DONE overflow-store audit (R11): 30 lm_dap_sat sites -> lm_dap_wrap
  (accont.c 14, accs.c 4 COMMEQS/docked fit, rate.c 4 ROTORQUE/BACKP AD
  before MP, t6rupt.c 1 DXCH T6FURTHA, tjetlaw.c 7 MP/DV/DXCH operands).
  19 sat sites kept: OVERSUB/LIMITS/explicit POSMAX (rate.c, qraxis.c,
  qrjets.c, needles.c, pmanual.c 299, accont.c POSMAX path) and 5
  unreachable ones commented "sat = wrap" (pmanual/qrmanual RHC sums,
  gts ROOTCYCL, tjetlaw FIREFCT TS and ZONE5 TS L, where the TS overflow
  skip is not modelled). yaAGC MP and DV both take the overflow-corrected
  A. make test green, oracle_dap_lm 1780/0 at tolerance 0.
* DONE accont.c check_rules: no (void) casts remain in the lane (rule 7
  clear); the one hard violation, rule 4 lm_dap_switch_curves 61 lines,
  fixed by moving the NEG(POS)/FIXMIN/STMIN- block to axis_against_aos.
* DONE style debt: check_style 714 -> 0 findings on all 22 lane files
  (template file prologs; lm_dap_t one field per line with AGC scale;
  scale/intent comments on every declaration; 9 nested ternaries
  unrolled; closing-brace comments; tests main-first, long int, sizeof
  ()). Also removed: a tautological assertion in rate.c RATELOOP, unused
  TJ_BIT12. Tools in SCRATCH/review-dap-lm/style (fix.py dump/apply/
  blocks, prolog.py, mainfirst.py).
* DONE rule-5 density >= 2.0 in every lane file (check_rules 0 hard,
  0 warnings on src/lm/dap and the two tests): real pre/postconditions
  (word ranges at the arith.c instruction models, +0 only from +0 + +0
  in the adder, axis/index ranges, LM-rope-only entries, RESTORDB leaves
  5 or 0.3 deg, TIME6/TIME5 read back, NXT6ADR is an axis index, jet
  codes 8 bits, ROTSENSE = +-SENSOR, EDOT reflected >= 0 in the rough
  laws, PULSEFLG on/off at QRMINIMP/CHEKSTIK). state.c: 18 unused
  accessors removed (and from include/agc/lm/dap.h): agc_erase_{ch5mask,
  ch6mask,db,set_db,delcdux/y/z,delperor/q/r,hiascent,set_hiascent,kq,
  neg_ratedb,perror,qerror,rerror}, agc_lm_dap_running; the rest check
  dap_valid() (T5ADR, RETJADR, AXISCTR in range) and the value range.
* DONE make check: no finding in lane files (other lanes' rule-7
  failures remain). All 34 test programs pass on a scratch build (src/core
  is mid-edit by review-core-numeric; its failing files fall back to
  HEAD in SCRATCH/review-dap-lm/build.sh).
* DONE MODULE_STATUS row, Lessons. Lane review complete.

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
| R11 | 30 sites (accont, accs, rate, t6rupt, tjetlaw) | TS/XCH/DXCH/ADS stores, AD before MP/DV | lm_dap_sat (POSMAX limit) used where the AGC stores or multiplies the overflow-corrected word | -> lm_dap_wrap; unreachable TS-skip sites keep sat with a comment; all in-range, oracle unchanged |

| R12 | src/exec/fresh_start.c startsb1 (exec lane) | STARTSB1 p.220 | On an LM software restart (ENEMA) exec sets the T5 task to DAPIDLER itself and never calls agc_lm_dap_restart(): RCSFLAGS bit 13 and ACCSOKAY stay set, T6NEXT/NXT6ADR/NEXTP are not reset, and lm_dap_t.t5adr stays PAXIS, so the next DAPIDLER -> STARTDAP trips AGC_ASSERT(d->t5adr == LM_T5_DAPIDLER) and 1/ACCS is not rerun | REQUEST to exec: in startsb1(), LM branch, replace the WAND/agc_rupt_set_t5_task lines with a checked call `agc_status_t sd = agc_lm_dap_restart();` (it does the WAND, T5ADR, RCSFLAGS, T6NEXT, NXT6ADR, NEXTP, ACCSOKAY) |
| R13 | state.c, dapif.c, rate.c, paxis.c | -- | Tautological assertions (`x = c; ASSERT(x == c)`, `st = OK; ASSERT(st == OK)`) found while raising density | rate.c RATELOOP one removed; others left where they also guard a later call; new assertions are pre/postconditions across calls |

## Requests

* exec (R12): call agc_lm_dap_restart() from fresh_start.c startsb1()
  for the LM.
* sim: agc_lm_dap_time6_count / agc_lm_dap_t6_pending (TIME6 DINC
  harness) are not called anywhere; the sim does not count TIME6, so
  no T6RUPT ends a jet firing in scenarios.

## Lessons

* Overflow stores: TS/XCH/DXCH/ADS store the overflow-corrected word,
  and yaAGC's MP and DV also take the overflow-corrected A, so "AD then
  MP" is a wrap, not a saturate. Saturate only where the code says so
  (OVERSUB, LIMITS, explicit CA POSMAX). Where TS overflows into a skip
  that changes control flow, show the case is unreachable and say so.
* Style debt at this size is a writing task: a dump/apply loop (line
  number + comment text, then the formatter) keeps it to judgement on
  each declaration instead of hand editing; check scales against
  ERASABLE_ASSIGNMENTS rather than guessing.
* `st = (s != OK) ? s : st` keeps the LAST failure, not the first;
  comment such accumulators precisely.
* Removing unused public accessors is cheaper than padding them with
  assertions; grep src, tests and sim first.

Oracle cases added by this review: accs_aos_* (13, earlier run), accs_docked_thrust/_aps/_fix, gts_a..f, tg_a..d, tjs_* (20), kal_docked, kal_docked_pw.
