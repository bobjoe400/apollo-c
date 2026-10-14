# oracle-cases -- lane history

Process diary of the oracle-cases lane, moved verbatim out of [docs/modules/oracle-cases.md](../../modules/oracle-cases.md), which keeps the reader-facing explanation. Note dates: 2026-10-10 to 2026-10-14 (first and last commit touching the note).

## Progress

| Module | Status | Spec / test |
|---|---|---|
| nav-integ | DONE (11 cases, 247 checks pass) | nav_integ.spec, tests/oracle/nav_integration.c |
| nav-aux | DONE (34 cases, 484 checks pass) | nav_aux.spec, nav_aux_lm.spec (gen/gen_nav_aux.py), tests/oracle/nav_auxiliary.c |
| imu | DONE (23 cases, 211 checks pass) | imu.spec (gen/gen_imu.py), tests/oracle/imu.c |
| guidance | DONE (14 cases, 203 checks pass, 27 KNOWN = finding F7) | guidance.spec (gen/gen_guidance.py), tests/oracle/guidance.c |
| landing | DONE (13 cases, 154 checks pass, 12 KNOWN = finding F9) | landing.spec (gen/gen_landing.py), tests/oracle/landing.c |
| entry | DONE (13 cases, 521 checks pass) | entry.spec (gen/gen_entry.py), tests/oracle/entry.c |
| dap-lm | DONE (19 cases, 473 checks pass) | dap_lm.spec (gen/gen_dap_lm.py), tests/oracle/dap_lm.c |
| dap-csm | DONE (40 cases, 533 checks, 0 KNOWN; F9 resolved by review-dap-csm) | dap_csm.spec (gen/gen_dap_csm.py), tests/oracle/dap_csm.c |
| targeting | DONE (8 cases, 116 checks pass) | targeting.spec (gen/gen_targeting.py), tests/oracle/targeting.c |
| return_to_earth | DONE (4 cases, 144 checks pass, 58 KNOWN = finding F14) | return_to_earth.spec (gen/gen_rte.py), tests/oracle/return_to_earth.c |
| dsky | DONE (92 runs, 1544 checks pass, 7 KNOWN = finding F15) | dsky.spec, dsky_lm.spec (gen/gen_dsky.py), tests/oracle/dsky.c |

## Findings (for the review wave)

Each finding is an unexplained or explained difference of the C from the
AGC. KNOWN lines in the tests carry the id; when a fix lands, the test's
oracle_cmp_known calls for that id turn back into oracle_cmp.

| id | where (C) | AGC label | values | status |
|---|---|---|---|---|
| F1 | yaAGC (tools/oracle) | DV (SpToDecent, -0 minor word) | precision integrations: velocities up to 5889 LSB over 5 steps; with the patched emulator (gen/dvcheck.sh) <= 1280, residual in the final partial step | oracle defect, not C; tolerances in nav_integ / targeting / entry |
| F2 | src/nav (R-TO-RP moon, 2^29 scale) | MOONMX, LAT-LONG | MOONMX <= 2 LSB -> LAT 23, LONG 202 LSB | fixed (follow-up-done-lanes: push-down words), 0 |
| F3 | src/nav/aux TFF | TFFCONIC TFFRP/RA CALCTFF | TFF1/ALF 11, TFFX 144 LSB | fixed (follow-up-done-lanes: MPAC words, TFFALFA kept as words), 0 |
| F4 | src/nav LSPOS (LM) | LSPOS | <= 5 LSB | fixed (follow-up-done-lanes: GTMP/STMP words), 0 |
| F5 | src/imu | CDUTODCM DELCOMP DCMTOCDU (kalc_small) | <= 5 LSB; AM 323 via ARCCOS | partly fixed (review-imu: DMP truncation; AM 5); residual <= 4 LSB is the ones'-complement word-split class (F9): BDSU leaves 1-cos A = (00001, 40057), DMPSUB truncates by words |
| F6 | src/imu/compensation.c (IRIGCOMP) | IRIGCOMP / GCOMP | GCOMP kept as a sign-disagreeing pair (49, -14827) -> GCOMPSW 45 vs 47 (same sign) | fixed (review-imu): GCOMP/DELV kept as word pairs; exact |
| F7 | src/guidance/servicer_common.c:47-49 | CALCGRAV (J2REQSQ) | J20_CM = 4357942 and J2_CM = 435794 are 1/4 of the assembled Comanche words (2DEC* ... B1*: 17431770, 1743177) -> CM GOBL1/2 4x too small, GDT1/2, VN1 up to 208 LSB | bug; 27 KNOWN in guidance.c |
| F8 | src/guidance/burn_steering.c:329-330 | S40.2,3 (PDVL UNIT RTIG / VXV VCOMP) | crosses UT with raw RTIG instead of UNIT(RTIG) before UNIT: YSCREF/ZSCREF up to 66 LSB | bug (small) |
| F9 | src/lm/landing/throttle.c:~168, src/lm/landing/guidance.c:121, src/csm/dap/tvc/daps.c:~124 | THROTTLE (PIF -> PSEUDO55), TDISPSET, PITCHDAP (CMDTMP) | a sign-disagreeing DAS result whose high word the AGC then uses alone: THRUST/PSEUDO55 1 pulse, FWEIGHT up to 23388 LSB, TTFDISP 8, CMDTMP (77765,00356) -> PCMD TVCPITCH AK1 1, DELPBAR 163 | ones'-complement class; landing fixed (review-guid-landing); dap-csm fixed (review-dap-csm: TVC and RCS DP erasables as agc_dp_words_t, 0 KNOWN) |
| F10/F11 | src/lm/dap 1/ACCONT | CCS AOSV | AOSV = -0 swaps the V-axis block pairs | accepted -0 deviation (dap-lm section) |
| F13 | src/programs/targeting/csi.c (CDH, ~770-871: rz_timethet_rv) | CDHMVR / TIMETHET RNEW | DIFFALT 31 LSB (62 m) for a circular passive orbit (AGC exactly 100 km), velocities <= 748 LSB | unexplained, small |
| F14 | src/csm/burns/return_to_earth/conic.c:147 | INVC115 (UR1 VXV UV1) | N kept sign-disagreeing (00232 76635) -> UH/ <= 77 LSB -> the conic search converges elsewhere: T2 up to 17 s, LNG(SPL) up to 51410 LSB (0.07 deg), VPRED 7464, X(T2) 38927 (rte_leo) | 58 KNOWN in return_to_earth.c |
| F15 | src/io/dsky/pinball/scale.c:566 (pin_opdegout) | OPDEGOUT / NEGOPT | CDUT = 61737 (= -20BIAS): the C treats sum == 0 as CCSHOLE and aborts (AGC_STATUS_POODOO); in ones' complement -20BIAS + 20BIAS is -0, so the AGC takes the "-0" branch (COM) and shows +89.997 deg | bug (edge); 7 KNOWN in dsky.c |

F12 was withdrawn (C fresh-start state, not a translation difference).

## Requested core additions

* tools/oracle (oracle lane): (1) the DV defect of the vendored yaAGC
  (F1; gen/dvcheck.sh builds a patched probe and lists affected runs);
  (2) WRITE/ROR/WOR to channel 7 (SUPERBNK) never reaches
  OutputChannel7 (agc_engine.c:3482 copies OutputChannel7 back into
  InputChannel[7] every instruction), so software superbank switching
  (NVSUB, SUPDXCHZ, BANKCALL into 40-47 from below) fails; (3) agc_probe
  parses names starting with a digit as octal addresses (504LM, 1/PIPADT,
  1JACC, RTED1 need explicit addresses) and names containing '-' or ','
  (LAT-LONG, L,PVT-CG) as offsets.
* io-nouns: agc_noun_write for VHFCNT/TRKMKCNT and FUNNYDSP (read-only
  now) and for negative TTOGO/TIG (the time ids refuse values < 0, but
  TTOGO is negative before ignition).

## Open questions

* F13 (CDH DIFFALT 62 m): not traced to a line; needs a TIMETHET-level
  case to isolate.
* Should the -0 display deviation of pinball be closed (DSPDECWD could
  take negzero = (word == 77777) for raw SP components)?

## Lessons

* Return traps must match the bank half: interpretive routines in FBANK
  < 20 return through QPRET to ABTCONIC (25751 CM / 25742 LM), FBANK >= 20
  to V2TERROR 65105 (CM) or P39P79 71456 (LM); a CALL from an erasable
  snippet cannot return (CALL builds QPRET from BANKSET+LOC), so enter the
  routine directly with Q = the trap.
* Runs that reach NOVAC/INTWAKE/JOBWAKE need executive core sets: NEWJOB
  0, PRIORITY 37401, PRIORITY+12k = 77777 for k = 1..6.
* Erasable overlays (LEQ = FX+1, GAMMAL = PREDANG, MFS = MFI = DEL ...)
  make some outputs uncomparable; compare only words the path leaves.
* Two-stage generation (probe output poked into later runs) keeps C and
  AGC on identical inputs where one routine consumes another's push list.
* C module state outlives agc_state_init (TVC temporaries, DAP fresh-start
  block): tests zero it to match the AGC's zeroed erasable.
* oracle_in() folds 77777 to 0; raw inputs go through oracle_in_raw(),
  which keeps -0.

