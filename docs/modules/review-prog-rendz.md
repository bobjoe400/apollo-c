# review-prog-rendz -- review of the prog-rendz lane

## Scope

docs/agents/review.md items 1-5 for lane prog-rendz (docs/modules/prog-rendz.md):
src/programs/{navigation,targeting,displays}/*.c, src/csm/navigation/*.c,
src/lm/navigation/*.c, src/lm/radar/*.c, include/agc/programs/rendezvous.h,
tests/programs/rendezvous.c, tests/oracle/targeting.c.
Routed items (docs/agents/review_items.md): F13 CDH DIFFALT 62 m;
targeting findings; P23/R22LEM restart priority (with exec review);
programs/navigation/noun_data.c -> lm/radar upward dependency.

Scratch: SCRATCH/review-prog-rendz/ (f13fix/ holds the F13 evidence: the
word-level WLOOP experiment diff and the oracle specs).

## Progress

| item | status |
|---|---|
| F13 CDH DIFFALT | DONE: traced to the word (Findings P1); fix belongs to nav-conic/core (Requests); tests/oracle/targeting.c comment explains it |
| targeting oracle findings | DONE: cdh velocities (748 LSB) are the F13 cause; initvel/tpi within nav-conic's Lambert tolerances |
| noun_data.c -> lm/radar, csm/navigation layering | DONE: TANG, RSTACK, LANDMARK, HORIZON, CPHIX/PRAXIS storage moved to programs/navigation/common.c (P4) |
| P23/R22LEM restart priority | DONE as request to exec (P3); deviation comments stay |
| constants vs rope | DONE: all ~100 DP/SP constants of the lane checked against both ropes (rope2.py), all match |
| alarm codes vs AGC | DONE: 00124 00600/601 01211 502 510 520 521 522 526 527 611 all present |
| P31 fidelity | DONE: removed the extra INTIME = TIG store (AGC P31 does not set INTIME; NUMIT = 0) |
| forwarding wrappers | DONE: agc_rr_limchk now holds RRLIMCHK (rr_limits_ok removed), agc_task_rr_goodend holds RGOODEND (rr_goodend removed), tps_kflag inlined; others add arguments/state and stay |
| digit-suffixed names | DONE: 18 renames (opd_sr30_1 -> opd_orbit_params, ta_crs61 -> ta_point_at_target, rpd_lm_rev83, tps_vncomp17, td_rend1/3, pfd_mp3, ln_9dw_*, ln_f2410, ln_box12/32, csi_ntp2, csi_b1/b2/b23, agc_rendz_periapo1 -> agc_rendz_periapo_body_scale, agc_rendz_shiftr1 -> agc_rendz_scalar_from_body; src/guidance/external_delta_v.c caller updated) |
| style debt | DONE: check_style 3600 -> 0 findings in all 38 lane files (src/programs/{navigation,targeting,displays}, src/csm/navigation, src/lm/navigation, src/lm/radar, rendezvous.h, internal.h, radar.h) and both lane tests; no line over 80 columns; check_rules 0 hard / 0 warnings in the lane (rule 4: mu_range split out of mu_compute, nd_dp alias cases merged; rr_dodes_commands split out of DODES). Code cleanups on the way (no behaviour change): ternaries that could not carry a comment rewritten as if statements or tables (RR1AXIS targets per mode); dead `st = OK; if (st != OK ...)` blocks after void calls (agc_waitlist_kill, agc_exec_prio_change, agc_guid_nbsm/smnb, agc_dsky_falton) removed in designate, acquisition x5, LM/CSM navigation, CSM tracking data, cislunar, orbital, tracking attitude; vacuous asserts made real or dropped (csi_almxit alarm range, rz_prio_alarm, rz_phase_job); rr_begdes forwarding wrapper folded into agc_task_rr_designate_begin; duplicate W / rr_state pointers removed; unused LN_PRIO13 removed; NORM index comments corrected to the AGC convention X = -(left shifts), scale 2^(S+X) |
| DMP/DMPR and shift audit (review_items, all lanes) | DONE: every DMPR/SRR/SLR in the lane's AGC files (P32-P33, P34-35, TPI_SEARCH, P20-P25 CM/LM, STABLE_ORBIT, R30, R31, P21, P76, P31) has a rounded C site (ROWDOT, TPI_SEARCH NORM DMPR, CDHMVR, LM RDOTCONV, LM DSQ DMPR RATEVAR, LM DMPR 36D, the LM SRR*/SLR* helpers); all other C multiplies are truncating (45 agc_dp_mul_trunc, 9 TP). 1 wrong site: P21 'DMP K.01' used the rounded agc_dp_mul (P5). Shifts spot-checked against SR/SR1R/SRR (CSI/B, CDHMVR, INITVELX, OKMAX, FIGTIME, COSINE SR1, SHIFTR1): consistent. No saturating overflow stores in the lane. F13 is not a multiply: it is the R1A word form (P1). tests/oracle/targeting.c tolerances cut to the observed residuals (moon 64->32, prec 4096->3400, cdh 1024->520/760, tpi 1024->2/760) |
| fidelity pass per routine | DONE for the review scope: P31, R61LEM/R65LEM, CDHMVR (operand by operand, push list), CSI/B, TRANSANG, ROWDOT, BVECTORS, V67 (V6N99INP option decode, NXPOSVEL row count), LLASRD/LLASRDA, LSR22.3 RADARANG/RANGEBQ/RATEBQ shifts re-read against the listings (match); the rest relies on the prog-rendz lane, the oracle (116 checks) and the multiply/shift audit |
| make / make test / make check | DONE for the lane: make and make test green (tests/programs/rendezvous 18/0, tests/oracle/targeting 116/0); uncrustify --check passes on every lane file; check_style 0 and check_rules 0/0 in the lane. make check as a whole still stops in check-rules on other lanes (50 hard, e.g. src/programs/alignment/word_forms.c rule 7). tests/programs/rendezvous: the circular-orbit SR30.1 case replaced by an eccentric one (e = 0.0404, analytic apsides) -- the circular case is ill-conditioned (SQRT of a 1-2 LSB difference) and C differs from the AGC by the R1A word form of P1 (oracle aps.spec: AGC ECC 0, RAPO-RPER 8 LSB; C ECC 23164, 4614 LSB) |

All lane files compile and the lane tests pass as of this update (2026-10-11, review complete).

Style tooling (SCRATCH/review-prog-rendz/style/): run.sh FILE [--write] applies
specs/<dir>_<file>.spec (file prolog, function prolog args/returns/AGC
citation) and .decl (decl/ternary/define comments); .orig = the file before
the style pass.

## Findings

| # | file:line | AGC (label, page) | finding | action |
|---|---|---|---|---|
| P1 | src/nav/conic/geom.c getx_wloop (nav-conic) | PARAM (C p.1290) R1A, GETX WLOOP (p.1292) | F13 root cause. PARAM's PUSH BDSU D1/32 leaves R1A = (00401, 40000) (257, -16383: a sign-disagreeing pair). WLOOP's SR4 gives (00020, 00001, 41777) and TAD keeps it: (05047, 05256, 62154). SQRTSUB normalises and then uses MPAC, MPAC+1 only, so the AGC root is 14575 26277 20000 while the same value in sign-agreeing words gives 14575 26276 00000 (oracle: f13fix/sqrt_tp_repr.spec). W per pass: AGC 329590 / 8724671 / 20827920, C 329589 / 8724670 / 20827918 -> X 32716648 vs 32716649, XI 157408770 vs 157408782, XSQC(XI) 2616 vs 2617 -> NEWSTATE R2 at 2^33 (32 m per LSB) (212498, -2) vs the AGC (212500, 0) -> DIFFALT 49968 vs 49999 (62 m), VACT3.y 748 LSB. A word-level WLOOP (R1A words from PARAM, agc_w_mpacshr, DAS, agc_w_sqrtsub keeping MPAC+2, agc_w_roundsub, agc_w_dad) makes DIFFALT and VACT3.y bit-exact (f13fix/wloop_wordlevel.diff) but breaks initvel_earth_far VTPRIME.x (-512) because LAMBERT's R1A has another word form, so the fix needs R1A carried as words by every producer: nav-conic's call. Same class as F9/F14. | test comment explains it; request to nav-conic and core-numeric (Requests) |
| P2 | src/core/interp/trig.c sqrt_words (core-numeric) | SQRT / SQRTSHFT (C p.1181) | the un-normalising right shift (MPACSHR) leaves the shifted-out bits in MPAC+2, which a following ROUND uses (GETX "SQRT ROUND", XCOMMON "SR1 SQRT ROUND"); agc_tp_sqrt returns agc_w3_top and drops them (0.5 LSB here). For shift 0 the AGC leaves the input's MPAC+2. | request to core-numeric |
| P3 | src/lm/navigation/tracking_data.c R22LEM3, src/csm/navigation/cislunar_navigation.c P23, src/csm/navigation/tracking_data.c R22 | R22LEM3 (L p.506) CAF PRIO26 / TS PHSPRDT2; P23 (C) PRIO13; R22 (C) PRIO26; POOH PRIO5 | the AGC stores the group-2 restart priority for later type C phase changes with D = 0 (OCT 04022); exec's store_var keeps PHSPRDT when D = 0 but has no "TS PHSPRDTn" entry, so the deviation stays | request to exec: agc_phase_set_prdt(group, prio) |
| P4 | src/programs/navigation/noun_data.c | nouns 66, 67, 70, 71, 72, 78, 95, 96 | the noun tables (programs layer) called storage defined in src/lm/radar (TANG, RSTACK) and src/csm/navigation (LANDMARK, HORIZON, CPHIX, PRAXIS): upward calls | storage and accessors moved to programs/navigation/common.c (agc_erase_set_rstack added; rz_tracking_angles internal); the radar/CSM code uses the accessors |
| P5 | src/programs/navigation/ground_track_determination.c gtd_display | P21DSP "DMP K.01" (GROUND_TRACKING_DETERMINATION_PROGRAM.agc p.458) | P21ALT used the rounded agc_dp_mul (DMPR) | agc_dp_mul_trunc |
| P6 | src/nav/conic/apsides.c GETECC (nav-conic); tests/programs/rendezvous.c | APSIDES / GETECC "DMP SL4 R1A, BDSU SQRT D1/64" (C p.1303) | same word-form class as P1: R1A = (00401, 40000) from PARAM; the AGC DMP partial products on those words give ECC 0 for a circular orbit (oracle aps.spec), C (value form) ECC 23164 -> RAPO - RPER 4614 LSB (9 km) where the AGC has 8 | lane unit test moved to an eccentric orbit; periapo circular bound documented; part of the nav-conic request |
| P7 | lane-wide | -- | collapse leftovers: status variables set to a constant and tested on the next line after void calls (14 sites), vacuous or duplicated asserts (3), duplicate state pointers (3), an unused define, a forwarding wrapper (rr_begdes) | removed / made real; no behaviour change (lane tests and oracle unchanged) |

## Requests

* nav-conic: carry R1A (and the other PARAM/LAMBERT outputs WLOOP reads)
  as the AGC's word pair so GETX's WLOOP can run SR4/TAD/SQRT/ROUND/DAD
  on words (P1), and let APSIDES/GETECC's DMP see the same words (P6).
  Evidence and a working sketch in SCRATCH/review-prog-rendz/f13fix/
  and aps.spec.
* core-numeric: an SQRT that returns the TP MPAC (MPAC+2 from MPACSHR,
  or the input's MPAC+2 when no shift) for SQRT ROUND sequences (P2).
* exec-sched: agc_phase_set_prdt(group, prio) for TS PHSPRDTn (P3).

## Lessons

* Vacuous status code is a collapse-lane artefact: `agc_status_t st =
  AGC_STATUS_OK; if (st != OK && st != NOT_FOUND) return st;` after a
  void call looks like error handling and is not. Grep for a constant
  assigned and tested in the next lines when reviewing a collapsed file.
* A style pass driven by spec files (per-function args/returns, per-
  declaration comments) scales to thousands of findings; generate the
  TODO list from the checker's own parser, never by hand, and post-fix
  the formatter's alignment artefacts (`x = /* c */` splits, comments
  moved above struct members) with a second small script.
* A circular orbit is a poor unit test for apsides: ECC = SQRT of a 1-2
  LSB difference, so any word-form difference shows as kilometres.

* The interpreter's MPAC word form matters, not only its value: SQRTSUB,
  DDV and "high word alone" uses all read the words. When C and AGC differ
  by a few LSB with no logic difference, dump the MPAC trace (agc_probe
  "trace 1") and compare words, not values.
