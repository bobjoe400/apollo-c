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
| style debt | PARTIAL: zero findings in src/lm/radar/leadin.c, src/programs/targeting/lambert_aimpoint_guidance_lm.c, src/programs/navigation/{flags_csm,flags_lm,noun_data}.c; src/lm/navigation/tracking_attitude.c 1 finding left (TA_250DEC define comment) + 1 line > 80 (ternary comment at ~240, shorten) -- compiles. TODO: the other 30 lane files + rendezvous.h, internal.h, radar.h |
| fidelity pass per routine | PARTIAL: P31, R61LEM/R65LEM re-read against the source (match); rest relies on the prog-rendz lane + oracle |
| make / make test / make check | TODO at the end. Full tree builds only with -Wno-error=comment (io-dsky pinball/internal.h mid-edit). tests/programs/rendezvous: 3 failures in test_orbit_parameters_circular caused by the in-flight nav-conic APSIDES change (C rapo/rper differ by 1246 LSB on a circular orbit; AGC APSIDES gives 8 LSB, oracle SCRATCH/review-prog-rendz/aps.spec) -- recheck before finishing, report to nav-conic if it persists |

All lane files compile (-fsyntax-only with the project flags) as of this update.

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

## Requests

* nav-conic: carry R1A (and the other PARAM/LAMBERT outputs WLOOP reads)
  as the AGC's word pair so GETX's WLOOP can run SR4/TAD/SQRT/ROUND/DAD
  on words (P1). Evidence and a working sketch in
  SCRATCH/review-prog-rendz/f13fix/.
* core-numeric: an SQRT that returns the TP MPAC (MPAC+2 from MPACSHR,
  or the input's MPAC+2 when no shift) for SQRT ROUND sequences (P2).
* exec-sched: agc_phase_set_prdt(group, prio) for TS PHSPRDTn (P3).

## Lessons

* The interpreter's MPAC word form matters, not only its value: SQRTSUB,
  DDV and "high word alone" uses all read the words. When C and AGC differ
  by a few LSB with no logic difference, dump the MPAC trace (agc_probe
  "trace 1") and compare words, not values.
