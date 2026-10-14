# review-nav-aux -- lane history

Process diary of the review-nav-aux lane, moved verbatim out of [docs/modules/review-nav-aux.md](../../docs/modules/review-nav-aux.md), which keeps the reader-facing explanation. Note dates: 2026-10-10 to 2026-10-14 (first and last commit touching the note).

## Progress
* [x] NEWANGLE: TP product shifted then rounded once (agc_tp_shift /
      agc_tp_to_dp) -- 40/40 random cases bit-exact vs Comanche055
      (old code: 16/40 off by 1 LSB). Committed in checkpoint 4f35a43.
* [x] MOONMX mul_half: TP product, SL1, round (was DP-rounded then
      shifted): 270 elements, 148 -> 45 off, max 8 -> 4 LSB.
* [x] navaux unit test: 143/0 failures; oracle nav_aux 484/0 (private
      build: HEAD imu/mode.c does not compile, other lane).
* [x] MOONMX residual (45/270, <= 4 LSB) EXPLAINED: NEWANGLE leaves
      NODI (all 30 cases) as a sign-disagreeing word pair (DAD); COS/SIN
      on those words differ by 1 LSB from COS/SIN of the joined value.
      Proven: COS of the AGC's NODI words (core word model) = AGC AVECTR
      in 30/30; with the joined value A.x is off in 21/30. agc_dp_t cannot
      carry the split -> accepted ones'-complement deviation (F2 class),
      core request for word-level SIN/COS.
* [x] latlong.c rounding order: GETERAD DSQ/SL1 on the TP; LAT-LONG and
      ARCTAN sum of squares keeps MPAC+2 of the second DSQ into SQRT
      (agc_tp_sqrt); DMP SL1R GAMRP shifts the TP then rounds.
* [x] latlong earth LAT 1 LSB EXPLAINED: ARCTAN's SR1 leaves the shifted-
      out bit in MPAC+2 and ARCSIN's ACOSST2 (DCS MPAC / AD HALF / SQRTSUB)
      takes that stale, un-complemented MPAC+2 into SQRTSUB. Shown with the
      core word model: m2 = 0 -> 29827835 (C), m2 = 8192 -> 29827834
      (AGC). Needs a TP-input ASIN/ACOS in core (request).
* [x] free_fall.c rounding order (F3): every DMP that feeds SR*/SL*/
      SQRT/BZE/BPL now keeps the TP (agc_dp_mul_tp + agc_tp_shift), DMP
      results used by DP instructions are truncated (agc_dp_mul_trunc, was
      DMPR-rounded), VSQ DCOMP / DOT SL3 / CALCTFF Q2^2 chain keep MPAC+2
      into SQRT, ENDTFF returns the truncated top words. ALFA = truncated
      top of the shifted TP + 2/RMAG (DAD touches only the top words).
      Result: TFFALFA, RPER, RAPO, TFF, TFFQ1 bit-exact (were up to 9 LSB);
      TFFX 144 -> 26, TFFTEM 9 -> 1. Residual TFFRTALF <= 3 LSB (-> TFF1/ALF
      <= 11): SQRT of the sign-disagreeing TP that DAD leaves (top +, MPAC+2
      -); a word-level SQRTSUB (core model) reproduces the AGC in 2/3 cases.
* [x] LM LSPOS (F4): POSITB/POSITD SL on the TP then DAD/DSU on the top
      words (was: TP rounded after the shift), POSITB DMP* truncated;
      lspos_lm_t0/t1 bit-exact, t2 VSUN <= 2 LSB (sign-disagreeing LOS
      words into SIN, MOONMX class).  CM RES: OMEGAES product truncated.
* [x] measurement.c INCORP1: VSQ via agc_vec_sq_tp (removes the DOTSUB
      deviation), INCOR1C DMP TEMPVAR -> TP SQRT, SL* 0,2 on the TP;
      agc_tp_norm / agc_tp_shift replace the util.c raw TP helpers.
* [x] util.c: removed tp_norm, tp_shl2, rev_tp_shl, rev_add, tp_from_cs
      (core TP API now covers them).
* [x] renames: agc_incorp1 -> agc_incorp_state_deviation,
      agc_incorp2_wupdate -> agc_incorp_update_w, agc_incorp2_state ->
      agc_incorp_update_state (callers updated; NAMING.md rows added).
* [x] pad-load accessor unification: TEPHEM and RLS have one store (core
      agc_erase_tephem/rls); agc_navaux_set_padload writes them there and
      agc_navaux_get_padload reads them back, so agc_lsposition and the
      pad-taking functions agree. AZO, AXO, -AYO, 504LM stay module-static
      in planetary.c (no core accessors; request below). Stale comments
      fixed (auxiliary.h pad-load block, agc_lsposition).
* [x] fidelity pass: latlong, planetary, free_fall, measurement (INCORP1;
      INCORP2 oracle bit-exact), ephemerides CM+LM, star_tables (rope-
      verified by nav-aux). Findings table below.
* [x] oracle tolerances tightened in tests/oracle/nav_auxiliary.c: LM LSPOS
      8 -> 2, TFF 16 -> 12, TFFX 256 -> 32 (484/0). No KNOWN entries exist
      for this lane (F2-F4 were tolerances).
* [x] style debt: 588 check_style findings -> 0 in all 11 lane files
      (src/nav/auxiliary/*.c, internal.h, include/agc/nav/auxiliary.h,
      tests/nav/auxiliary.c, tests/oracle/nav_auxiliary.c): template file and
      function prologs (AGC label + pages, argument tables with scales,
      RETURNS), decl/scale comments, ternaries rewritten as if/else,
      closing-brace comments on blocks > 40 lines, main() first in the
      tests, `long int`, `sizeof (x)`, all lines <= 80 columns.
      check_rules: 0 hard, 0 warnings (util.c density 1.33 -> 2.17 with
      real pre/postconditions: input ranges, SR/VSR never grows a
      magnitude, the TP low words carry the sign of v).  Vacuous
      `st = OK; if (!AGC_ASSERT(st == OK))` in MOONMX removed.
      Forwarding wrappers: tools/unwrap.py --list finds none in the lane.
* [x] note: Findings table, Requests, Lessons (below).

State of files: all lane files and every includer of auxiliary.h
compile with -Werror; test_navaux 143/0, oracle_nav_aux 484/0 (private
build: full `make` is blocked by src/csm/prelaunch/align.c, another
lane's in-progress job/task ids).  check_style 0, check_rules 0/0,
uncrustify --check clean on the lane.  No file is mid-edit.  REVIEW DONE.

## Findings

| # | where | AGC | finding | status |
|---|---|---|---|---|
| 1 | planetary.c NEWANGLE | NEWANGLE p.1248 | TP product rounded after SL; AGC truncates (DAD on top words) | fixed, 40/40 bit-exact |
| 2 | planetary.c MOONMX mul_half | DMP SL1R | rounded before the shift | fixed (148 -> 45 of 270 off) |
| 3 | planetary.c MOONMX (F2) | NEWANGLE -> COS | NODI/F leave NEWANGLE as sign-disagreeing words; SIN/COS of them differ 1 LSB from the joined value; <= 4 LSB in M, -> LAT/LONG moon 23/202 LSB | explained, accepted (needs word-level MPAC in core) |
| 4 | latlong.c GETERAD, LAT-LONG, ARCTAN | DSQ SL1, DSQ/DAD/SQRT, DMP SL1R | DSQ TP and SL1R order lost MPAC+2 | fixed (no observed change in the 8 cases) |
| 5 | latlong.c ARCTAN -> ASIN | SR1 ASIN / ACOSST2 | ARCSIN's SQRTSUB takes the stale, un-complemented MPAC+2 left by SR1: earth LAT 1 LSB | explained; needs TP-input ASIN in core |
| 6 | free_fall.c (F3) | TFFCONIC TFFRP/RA CALCTFF TFFEL1 | DMPR used where the AGC DMPs into a TP shift / DP truncation; DP shifts where the AGC shifts the TP | fixed: ALFA RPER RAPO TFF Q1 exact; TFFX 144 -> 26 |
| 7 | free_fall.c TFFCONIC | DAD / ABS SQRT | SQRT of the sign-disagreeing TP DAD leaves: TFFRTALF <= 3, TFF1/ALF <= 11 LSB | explained, accepted |
| 8 | ephemerides.c LM LSPOS (F4) | POSITB/POSITD | TP rounded after SL; DMP* rounded | fixed: t0/t1 exact, t2 <= 2 LSB (class 3) |
| 9 | ephemerides.c CM RES | PUSH DMP OMEGAES | angle rounded, AGC truncates | fixed (oracle was already exact) |
| 10 | measurement.c INCORP1 | VSQ RTB TPMODE, INCOR1C | own exact VSQ (deviation) and DMPR/DP SQRT | fixed with agc_vec_sq_tp, TP SQRT/shift |
| 11 | measurement.c | INCORP1/INCORP2 | digit-suffixed names | renamed (incorp_state_deviation, incorp_update_w, incorp_update_state) |
| 12 | planetary.c MOONMX | -- | dead `st = OK; if (!AGC_ASSERT(st == OK))` check | removed |
| 13 | lane (11 files) | -- | 588 check_style findings, util.c rule-5 density 1.33 | 0 findings, density >= 2.0 everywhere |

Open questions from docs/modules/nav-aux.md: VGAMCALC / TFF/TRIG belong
to guid-entry (P61-P67.agc) -- unchanged; the pad-load duplication is
resolved (one TEPHEM/RLS store).

## Requests (core-numeric)

* TP-input ASIN/ACOS (agc_tp_asin/acos): ARCSIN/ARCCOS feed the stale
  MPAC+2 into SQRTSUB (finding 5).
* A word-level MPAC (sign-disagreeing pairs) through DAD -> SIN/COS/SQRT
  would make findings 3, 7 and the LM t2 residue exact; today agc_dp_t /
  agc_tp_t are values and lose the split.
* Shared accessors for AZO, AXO, -AYO, 504LM (and the CM ephemeris
  uplink TIMEMO/VECOEM/RESO/VESO/OMEGAES) to retire the module statics.

## Lessons

* check_rules rule 7 rejects `(void) AGC_ASSERT(...)`: a pre/post-
  condition must be the guard form `if (!AGC_ASSERT(c)) { return ...; }`.
  A range predicate itself carries no assertion (core/mpac.c agc_w_ok).
* A comment like `SL*/SR*` ends the block comment early; write `SL* and
  SR*` in prologs.
* Before trusting a "<= n LSB" tolerance, probe the intermediates: the
  agc_probe "stop <basic label>" trick (stop DVXSC / ARCSIN inside an
  interpretive routine) dumps the push list mid-routine.
* Interpretive DMP never rounds and DAD/DSU touch only the top two
  words; most residues were DMPR/DP-shift substitutions for TP chains.
* The remaining residues are ones'-complement word splits, not logic.

