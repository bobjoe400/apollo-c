# review-guid-entry -- review of the guid-entry and p37 lanes

Lane files: src/csm/entry/{lexicon.c,reentry.c,programs.c,cm_body_attitude.c,dap.c},
src/csm/burns/earth_orbit_insertion.c, src/csm/burns/return_to_earth/*,
include/agc/csm/entry*.h, tests/csm/{entry,return_to_earth}.c,
tests/oracle/{entry,return_to_earth}.c.
AGC: ENTRY_LEXICON, REENTRY_CONTROL, P61-P67, CM_BODY_ATTITUDE,
CM_ENTRY_DIGITAL_AUTOPILOT, P11, P37_P70 (Comanche055).

Tools (scratch, review-guid-entry/): probe.c = agc_probe + `istop LABEL n`
(stop at the n-th interpretive dispatch of LABEL) and `itfrom LABEL n`
(trace MPAC per interpretive instruction); mk.py builds per-pass specs
from a case; rt.sh builds a test against fresh lane objects + a private
snapshot library (mylib.sh) because the shared tree is mid-edit.

## Progress
(resume 2026-10-11: velocity.c F14 fix verified complete -- compiles
-Werror, oracle return_to_earth 144 checks/0 fail/45 KNOWN; stale scratch
stubs.c dropped from the private lib.)
(state at checkpoint: every touched file compiles with -Werror; oracle
return_to_earth 0 failures / 45 KNOWN (was 58), oracle entry 521/0,
tests/csm/entry 23/0, tests/csm/return_to_earth 21/0. Shared `make` is
red from other lanes' mid-edits (imu/mode.c, burn_steering.c) - not mine.)
* DONE F14 cause 1: DSU on a TP MPAC keeps MPAC+2 (rte_tp_dsu; now
  rte_tp_dsu/rte_tp_bdsu exported in return_to_earth/internal.h, velocity.c).
* DONE F14 cause 2: DVCALC SR* 6,2 into MPAC+2, SQRT on the TP MPAC
  (velocity.c rte_dv_calc). First DVCALC word-exact; GAMDV path exact.
* DONE F14 remainder traced to other lanes (Requests): INVC115 VXV on
  disagreeing words (core), GAMDV X(T1) disagreeing words (core), TIMERAD
  V(T2) (nav-conic). RCON PCON DV R(T2)/ T12 flipped to hard checks.
* DONE RTARG push-list (probe): deviation kept, comment fixed (program.c).
* DONE AUGEKUGL (probe, 30 pairs): bands identical, PHI <= 62 LSB (core class).
* DONE PREC150/155 (probe, 40 cases): BDSU keeps MPAC+2, SL2 SQRT DMP BDSU
  NORM on TP (precision.c prec_radius_ratio): mismatches 17 -> 4 (rest =
  SQRT of disagreeing TP words, core class). PREC162/160 exact.
* DONE PREC205-207 (probe, 40 cases): exact except 1 LSB DRCON (DSU words).
* DONE precision constants vs rope; entry lexicon/DAP constants vs rope
  (54 auto + 13 by hand, 0 mismatches).
* DONE five dispatchers vs "* START" sections: GOTOUPSY moved from the
  HUNT TEST dispatcher to UP CONTROL (reentry.c). Others match.
* DONE GETLEWD open question: label typo in the transcription only; the
  assembled rope defines GETLEWD at 25,2677. CH1 = .64 confirmed (24365 30244).
* DONE dap.c bugs fixed: (a) READGYM1 MSU is modular -> entry_angle_wrap
  (was saturating: gimbal through 180 deg); (b) EXDAP2 ROLLHOLD MSU same;
  (c) phase plane SR: CA SR re-edits SR, so SR = -VT/360 (was -VT/180)
  for the DZ2 test; (d) CM/FDAI ADS PAXERR1 "OVFL OK" wraps by 1.0 (new
  entry_sp_store_ovf in dap.c, declared in include/agc/csm/entry.h).
* DONE dap.c fidelity: READGYMB, DOBRATE?, BODYRATE, ATTRATES, EXDAP*,
  BIASEDZ/3DDZ/4D/SDZ/2D/SDZ, CM/RCS..COMPAT, WHICHALF..TIMSCAL, TIMETST,
  JETCALL*, DZCALL, CM/FDAI(R), CM/DUMPR/CMTMFILE/SAVENDX index (-13..-1
  step 3 -> ENDBUF-14..ENDBUF), CM/DAPON (+NOTYET), CM/DAPIC/CM/DAP2C
  (JETEM: POS1/2 = BIT14 = 020000, so 2 POS1/2 = 1.0; Python model of the
  ones'-complement ADS over all TIME1/PIPTIME pairs = C's mod-2^14
  difference; CCS loop incl. -0 -> 1), CM/SWIC1 016017 / SWIC2 012 bits.
  SETJTAG: TUSED sign flipped vs AGC -> now a DEVIATION comment (TIMETST
  consistent). Gimbal-wrap / OVFL-OK test added (tests/csm/entry.c, 29/0).
* DONE DMP fidelity (new finding F-E1): REENTRY_CONTROL, CM_BODY_ATTITUDE,
  P61-P67 and P11 contain no DMPR, yet the C used agc_dp_mul (DMPR,
  rounded) at 92 sites. Now agc_dp_mul_trunc (DMP); the 4 CM/TRIG VXSC
  sites stay rounded. TP MPAC modelled where the AGC shifts or roots it:
  new entry_tp_shl (entry.h/reentry.c) for DOT SL1 / DMP SL1 (RDOT, D,
  L/DCALC, STARTENT THETAH, CM/TRIG BETA/GAMA, FISHCALC, VRCALC);
  agc_tp_sqrt/agc_tp_add for HUNTEST1 VL, CONT3 VREF, RANGER E; TP SL 8D
  in CONSTD1. LOG uses agc_dp_poly (POLY). Oracle entry residuals:
  DTEAROT/RT/THETAH 1 -> exact/<=3 (RT.y was 98: one cs of DTEAROT);
  ROLLC 2159 -> 1 (PREDICT3; was blamed on yaAGC DV); L/D 373 -> <=9
  except entry_pass_next 256 (VL -38 carried); RDOT +-1 (DOT TP low word,
  core class). Remaining D 84/156 (initroll/kep2) unchanged.
* DONE tests/oracle/entry.c tolerances tightened: THETAH 128->8, RT
  128->8, ROLLC 4096->16, L/D(1) 512->320 (521/0).
* DONE S61.1 / S61.1A-C / RETRN1-3, P61..P67 / P61.1-.4 / P62.1-.3 /
  P63.1 / P65.1 / P66END / KILLGRP4 / P67.1-.3 / WAKEP62 read vs C:
  display returns (GOFLASH/GOFLASHR/GOPERF1R L+2..L+5), PHASCHNG words,
  CMDAPMOD test (+1 -> P63.1), CS THREE (GYMDIFSW+CM/DSTBY), OCT41=33DEC
  all match. S61.2 / FISHCALC / VGAMCALC / TFF/TRIG / DISPTARG / VRCALC /
  GMAXCALC / S62.3 order of operations match (GMAX lo word = VPRED hi in
  the AGC overlay; C keeps a separate DP - display/downlink use SP only).
* DONE programs.c S62.3 read (YNB/XNB/ZNB order, CPHIFLAG clear, CALCGA).
* DONE earth_orbit_insertion.c (P11) read vs P11.agc; fixed:
  (a) ATERJOB: switch on (STRSAT) with SATSW > 0 skipped the ATRESET
      NEEDLER init and 60 ms DELAYJOB (went straight to ATRESET +5);
  (b) STEERSAT TAD BIASAK+AK is a TP add (carries between words), and
      RESCALES (MP, DDOUBL x2) + TS AKn: overflow stores modulo 1 and the
      TS skips CA AKn+1 (next word rescaled from +-1); was per-word
      saturation of 4 x (high word) (low-word carries lost);
  (c) NOPOLYM SR2 is a truncating TP shift (was rounded);
  (d) roll = SPOLYARG x SATRLRT SL 14D on the TP (14 bits were lost);
      RLTST tests the high word of MPAC x SATRLRT +1 (>0 = rollout), was
      sign(roll) x sign(SATRLRT);
  (e) ATTDISPR / ATTDISP2 / S11.1 HDOT DMP|DOT SL1 on the TP; POWRSERS
      TP result before SL 5 (pitch);
  (f) REP11 restart with TIME2 not yet zeroed: TLIFTOFF = clock read at
      the DXCH TIME2 (was LIFTTEMP);
  (g) GETDOWN's LALOTORV +5 copy uses DMPR (kept rounded) and GAMMA x
      SIN(LAT) by DMPR (was a truncating SR1);
  (h) NEEDLER calls pass the current AK words (were zeros); SATSTICK AK
      kept for SATOUT.
  P11 pad POLYNUM is taken as the degree (the AGC POLYNUM is N-1 for
  POWRSERS); whoever loads it from erasable must add 1 (Request).
* STYLE (scratch tool style/apply.py + per-file .spec: boxed prologs,
  decl/scale comments, ternary and long-block comments): DONE lexicon.c,
  return_to_earth/{precision,conic,velocity,program}.c + internal.h
  (0 findings, rule-5 ok, no line > 80). conic.c CFPA / GAMMAEI / GAMMAE
  DOT SL1 now on the TP MPAC (oracle unchanged); cm_body_attitude.c
  (CMTR2 BETA store now modulo 1 like XCH TEMPBETA, was saturating);
  earth_orbit_insertion.c; programs.c (VGAMCALC SL* 0-8D,1 with a
  negative count is a truncating TP shift - was rounded; rule 5 now 2.02).
  dap.c (nested ternaries replaced by dap_signed / dap_low_carry; rule 5
  2.00); reentry.c (agc_entry_targeting split into targ_velocity /
  targ_ldcalc, dead "st" code removed; rule 5 2.00); P11 launch_refsmmat
  split out (rule 4). All lane .c files: 0 style findings, 0 rule
  violations/warnings; entry.h (struct members one per line with
  scales), entry_lexicon.h; tests/csm/{entry,return_to_earth}.c and
  tests/oracle/{entry,return_to_earth}.c (main first, prologs). All 17
  lane files: 0 check_style findings, uncrustify --check clean, no line
  > 80 columns.
* DONE naming (item 3): digit-suffixed label identifiers spelled out from
  the AGC comments: INITRL1 -> step_hunt_trigger, CONT1 ->
  step_upcontrol_drag_test, CONT3 -> step_upcontrol_reference, CONTINU2
  -> step_upcontrol_q7min_test, CONSTD1 -> step_constant_drag_ld,
  UPCNTRL3 -> step_upcontrol_ld, L353 -> step_lateral_switch, L355 ->
  step_roll_command, L357 -> step_lateral_min_ld, KEP2 ->
  step_kepler_test (AGC_ENTRY_GOTO_KEPLER), PREDICT3 ->
  step_final_predict (AGC_ENTRY_GOTO_FINAL_PREDICT), HUNTEST1 ->
  hunt_exit_velocity, STARTEN1 -> entry_initial_target, KILLGRP4 ->
  entry_disable_restart_group, REFAZE10 -> agc_entry_reestablish_
  sequencer, EXDAP1/2 -> dap_exdap_yaw_pitch_jets / _beta_roll_hold,
  2D/SDZ -> dap_rate_deadzone. AGC_JOB_REFAZE10 / agc_job_refaze10 kept
  (exec jobs.h, dispatch.c: Request).
* DONE lane open questions answered in guid-entry.md and p37.md.
* DONE verification: make, make test (all pass), make scenarios (7/7);
  make check: 0 rule violations/warnings, uncrustify clean and 0 style
  findings in every lane file (the remaining check-rules FAILs are in
  other lanes: word_forms.c, noun_data.c, imu/mode.c).
* DONE entry residual traced: entry_pass_next VL -38 / L/D 256 come from
  HUNTEST1's DDV chain (ALP = A0 2C1HS / V1 / V1 / LEWD): exact 13571593,
  AGC 13571567, C 13571613 with identical inputs -> core DDV (Request).

## Findings
| # | Where | What | Fix |
|---|---|---|---|
| E1 | reentry.c, cm_body_attitude.c, programs.c, earth_orbit_insertion.c (92 sites) | every DMP translated as agc_dp_mul (DMPR, rounded); the listings have no DMPR | agc_dp_mul_trunc; VXSC sites stay rounded; DTEAROT/RT exact, ROLLC 2159 -> 1 LSB |
| E2 | same files | DMP/DOT followed by SL, SQRT, ACOS or DAD+SQRT used the DP top only | TP MPAC kept (entry_tp_shl, agc_tp_sqrt/add/shift): RDOT, D, L/DCALC, THETAH, BETA/GAMA, FISHCALC, VRCALC, VL, VREF, E, CONSTD1, P11 roll/pitch/needles, conic.c CFPA/GAMMAEI/GAMMAE |
| E3 | programs.c VGAMCALC | SL* 0-8D,1 with a negative count rounded | truncating (GENSCR/MPACSHR) |
| E4 | earth_orbit_insertion.c NOPOLYM | SR2 rounded | truncating TP shift |
| E5 | earth_orbit_insertion.c ATERJOB | switch-on + SATSW > 0 skipped NEEDLER init and the 60 ms wait | ATRESET path shared |
| E6 | earth_orbit_insertion.c STEERSAT/RESCALES | per-word saturating add, 4 x high word | TAD with word carries; MP DDOUBL DDOUBL incl. low-word carry; TS overflow stores mod 1 and skips CA AKn+1 |
| E7 | earth_orbit_insertion.c RLTST | sign(roll) sign(SATRLRT) | MP of MPAC by SATRLRT +1, high word > 0 |
| E8 | earth_orbit_insertion.c REP11 | TLIFTOFF from LIFTTEMP when TIME2 not yet zeroed | clock read at the DXCH TIME2 |
| E9 | cm_body_attitude.c CMTR2 | BETA saturated | XCH TEMPBETA stores modulo 1 |
| E10 | dap.c (from the earlier session) | READGYM1/EXDAP2 MSU saturating; SR re-edit; PAXERR1 OVFL OK | modular / -VT/360 / mod-1 store |
| E11 | reentry.c LOG | Horner by DMPR | agc_dp_poly (POLY) |
| F14 | return_to_earth (earlier session) | DSU/BDSU and SR* on the TP MPAC | rte_tp_dsu/bdsu, DVCALC TP; 58 -> 45 KNOWN, rest other lanes |

Deviations kept (commented in the code): TUSED holds +TIME1 (TIMETST
forms the same difference); RTARG on V32 recycle uses the saved vector
(the AGC pops garbage); RESCALES overflow after TS AK2 is stored without
the skip into CADR NEEDLER; GMAX keeps its own low word (the AGC overlays
VPRED); DUMPP61 retries at most 4 times.

## Requests (to other lanes)
* nav-conic: TIMERAD V(T2)/ differs by 1-3 high-word LSB (R(T2)/ exact).
  Reproducer (rte_leo pass 1): RVEC (3000000, 1600000, 0) B29, VVEC
  (-73597833, 136552611, 10197486) B7, RDESIRED 3247500, SGNRDOT -.5,
  earth, RVSW clear -> AGC V (-160229376, -30080768, 3212032), R (-398528,
  3216800, 198912), T 106847; C V (-160229120, -30081536, 3212032).
* core-numeric: a word-level MPAC (disagreeing signs) through VXV, DMP,
  SQRT is what is left of F14/AUGEKUGL/PREC155 (same class as F9).
* guid-powered: export AUGEKUGL (external_delta_v.c has a static copy);
  then drop the local copy in return_to_earth/conic.c.
* core-numeric: DDV with a small dividend: HUNTEST1 ALP chain, DLOAD A0
  (35781135) DMP 2C1HS (5797757) DDV V1 (165392454) DDV V1 DDV LEWD
  (40265318): exact 13571593, yaAGC 13571567, C 13571613. Also DOT's
  MPAC+2 (sign-disagreeing accumulation) shows as +-1 LSB in RDOT once
  SL1 shifts it in.
* exec / registry: AGC_JOB_REFAZE10 / agc_job_refaze10 keep a digit label
  (jobs.h, dispatch.c); suggested stem reestablish_entry_sequencer (the
  entry-side function is now agc_entry_reestablish_sequencer).
* whoever loads the P11 pad from erasable: POLYNUM in the AGC is degree-1
  (POWRSERS count); agc_earth_orbit_insertion_monitor_set_padload takes
  the degree.
* oracle-cases: the "D 256 (yaAGC DV bug)" entry tolerance stays (KEP2
  156, INITROLL 84); the ROLLC 4096 and RT/THETAH 128 tolerances were not
  yaAGC: they came from E1 (now 16 and 8).

## Lessons
* grep the listing for DMPR before writing agc_dp_mul: the interpreter
  DMP truncates and most guidance code never rounds. One systematic
  rounding error per multiply showed up as hundreds of LSB downstream
  (DTEAROT one cs -> RT.y 98 LSB) and was blamed on the emulator.
* "DMP SL1", "DOT SL1", "DMP DAD SQRT" and "POLY ... SL n" read MPAC+2:
  shift or root the TP, not the stored DP. SR of a scalar (and SL* with a
  negative count) truncates; only SRR, VSR and vector shifts round.
* Basic-code TS of an overflowed A skips the next instruction: in
  RESCALES the skipped CA changes which word the next rescale uses.
* Test assertions should state facts the AGC guarantees (limits, ranges,
  flag postconditions), not restate the store just made; dispatchers can
  assert the pass state they hand on.
* A per-file spec (function args, local comments with scales, ternary
  comments) applied by a script that knows the checker's parser turns
  style debt into a writing task; keep trailing comments short enough
  for 80 columns or split after '='.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [guid-entry](guid-entry.md#program-flow-and-decisions), [p37](p37.md#program-flow-and-decisions).
