# review-guid-entry -- review of the guid-entry and p37 lanes

Lane files: src/csm/entry/{lexicon.c,reentry.c,programs.c,cm_body_attitude.c,dap.c},
src/csm/burns/earth_orbit_insertion.c, src/csm/burns/return_to_earth/*,
include/agc/csm/entry*.h, tests/csm/{entry,dap,return_to_earth}.c,
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
* DONE S61.1 / S61.1A-C / RETRN1-3, P61..P67 / P61.1-.4 / P62.1-.3 /
  P63.1 / P65.1 / P66END / KILLGRP4 / P67.1-.3 / WAKEP62 read vs C:
  display returns (GOFLASH/GOFLASHR/GOPERF1R L+2..L+5), PHASCHNG words,
  CMDAPMOD test (+1 -> P63.1), CS THREE (GYMDIFSW+CM/DSTBY), OCT41=33DEC
  all match. S61.2 / FISHCALC / VGAMCALC / TFF/TRIG / DISPTARG / VRCALC /
  GMAXCALC / S62.3 order of operations match (GMAX lo word = VPRED hi in
  the AGC overlay; C keeps a separate DP - display/downlink use SP only).
* TODO programs.c (rest) and earth_orbit_insertion.c (P11) read.
  NOTE another agent has uncommitted edits in programs.c / entry.h
  (agc_entry_tff_trig, agc_entry_vgamcalc exports) - edit around them.
* TODO naming / style debt (1599 check_style findings in lane files) /
  rule-5 density / make check; findings table, Requests, Lessons below.

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
