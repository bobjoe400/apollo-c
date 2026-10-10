# guid-entry -- Comanche entry (P61-P67), entry DAP, CM body attitude, P37, P11

## Scope
Comanche055 only: ENTRY_LEXICON, REENTRY_CONTROL, P61-P67, CM_BODY_ATTITUDE,
CM_ENTRY_DIGITAL_AUTOPILOT, P37_P70, P11. Output in src/csm/ plus the public
header include/agc/agc_csm_entry.h (created by this lane).
Note: Comanche P37_P70.agc contains only P37 (return to earth) and its
subroutines; "P70" appears only in the constants title (the LM P70 abort
program is a Luminary routine). There is no Saturn-abort code in the file.

Files:
* include/agc/agc_csm_entry.h -- entry state struct (ENTRY_LEXICON), enums, public API
* src/csm/entry_lexicon.h/.c -- constants with scales; the static entry state
* src/csm/entry_internal.h -- prototypes shared between files of this lane only
* src/csm/entry_pending.h -- PENDING core symbols (flags, accessors, DSKY, nav,
  IMU, job/task ids). Delete at integration; see "Requested core additions".
* src/csm/reentry.c, p61_p67.c, cm_body_attitude.c, entry_dap.c, p37.c, p11.c
* tests/test_entry.c

## Design table

Scales: /2VS = /51532.3946 fps; /805 = /25 g; revs = angle/360 = range/21600 NM.

| AGC label (page) | inputs -> outputs (scale) | C function (file) |
|---|---|---|
| STARTENT (844) | LODPAD LADPAD HEADSUP -> LOD LAD L/DCMINR KLAT Q7 FACTOR L/D LATANG K2ROLL Q2 GOTOADDR POSEXIT | agc_entry_startent (reentry.c) |
| STARTEN1 (845) | LAT(SPL) LNG(SPL) PIPTIME -> RTINIT TIME/RTO DTEAROT RT THETAH | entry_starten1 (reentry.c) |
| SCALEPOP/REFAZE10 (847) | -> TARGETNG, sequencer | agc_entry_scalepop (reentry.c) |
| TARGETNG/GETVEL/GETUNITV (848-850) | VN or -VREL, UNITR, DELV -> VEL UNITV VSQUARE LEQ V RDOT D UNI LATANG GONEBY THETAH RTGON67 .05GSW L/DCALC | agc_entry_targeting (reentry.c) |
| GETETA/SUBETA/SETMIND/TINYTHET (851-852) | D predictor, time of flight, small-angle range | entry_geteta, entry_range_angle (reentry.c) |
| EARROT1/EARROT2 (CSM_GEOMETRY 293-294) | RTINIT DTEAROT UNITW -> RTEAST RTNORM RT | entry_earrot (reentry.c, local copy) |
| LOG (CSM_GEOMETRY 291) | x -> -ln(x)/32 | entry_log (reentry.c, local copy) |
| INITROLL/D0EQ/INROLOUT/KATEST/INITRL1 (853-854) | LEQ V RDOT D -> KAT D0 C/D0 L/D | step functions (reentry.c) |
| HUNTEST/A0CALC/V1LEAD/HUNTEST1/GETDHOOK/HUNTEST3 (855-858) | D RDOT V LAD LEWD Q7 -> A0 A1 V1 ALP FACT1 FACT2 VL GAMMAL1 VBARS DVL VS1 DHOOK AHOOKDV GAMMAL | entry_huntest_core (reentry.c) |
| NEGAMA/DHOOKYQ7 (862) | GAMMAL<0 -> VL Q7 VBARS | entry_negama, entry_dhookyq7 |
| RANGER (859-860) | GAMMAL VBARS VL V1 A0 RDOT -> ASKEP ASP1 ASPUP ASPDWN ASP3 DIFF (revs) | entry_ranger_core |
| GETDLEWD/LWDSTORE/SIDETRAK/LEWDOVFL/LEWDPTR (860-861) | secant iteration on LEWD | entry_step_getdlewd |
| PRE-HUNT/FOREHUNT (863) | restart / init of HUNTEST | agc_entry_pre_hunt, entry_forehunt |
| UPCONTRL/VLTEST/CONT1/CONT3/CONTINU2/UPCNTRL3/NEGTESTS (864-866) | -> VREF RDOTREF FACTOR L/D | step functions |
| DCONSTD/BECONSTD/CONSTD/CONSTD1/DOWNCNTL (867-868) | constant drag -> L/D | step functions |
| KEP/KEP2 (869) | ballistic phase | step functions |
| PREFINAL/PREDICT3 (870-872) | VREFER table -> FX PREDANG DNRNGERR L/D | entry_step_predict3, entry_table_lookup |
| GONEGLAD/GOMAXL/D/GLIMITER/GOPOSLAD (872) | g limiter -> L/D | step functions |
| LIMITL/D/L350/L353/L355/L357 (873-875) | lateral logic -> K2ROLL ROLLC | entry_step_limitld, entry_step_l355 |
| ENDEXIT/OVERNOUT/SERVNOUT/STEEROFF (873-875) | display, exit | entry_endexit, entry_step_steeroff |
| VREFER table (876-878) | 13 x 6 SP table | entry_vrefer[] (entry_lexicon.c) |
| CM/POSE, CM/POSE2 (883) | VN UNITR UNITW -> -VREL UXA UYA UZA OLDUYA | agc_entry_cm_pose (cm_body_attitude.c) |
| CM/TRIO (884-886) | AOG/AIG/AMG at PIP, REFSMMAT -> UBX UBY UBZ, roll beta alfa gamma | entry_cm_trio |
| ARCTRIG (INFLIGHT_ALIGNMENT 1357) | sin/4 cos/4 -> angle revs | entry_arctrig (local copy) |
| CDULOGIC (RT8 op codes) | SP CDU -> DP revs | entry_cdulogic (local copy) |
| CM/ATUP/CMTR1/CMTR2/REDOPOSE/CM/POSE3/CORANGOV (887-889) | GAMA GAMDOT ROLL/180 ALFA/180 BETA/180 VMAGI | entry_cm_atup |
| READGYMB/DOBRATE?/DOBRATE/CM/GYMIC (1063-1064) | CDUX/Y/Z -> -DELAOG/AIG/AMG | agc_entry_dap_readgymb (entry_dap.c) |
| ANGOVCOR/FLUSHJET/RATEAVG (1065) | helpers | dap_angovcor, agc_entry_dap_flushjet, dap_rateavg |
| CM/DAPON/NOTYET (1066-1067) | DAP turn-on, attitude hold | agc_entry_dap_on |
| CM/DAPIC (1068) | READGYMB start, flag init | agc_entry_dap_ic |
| BODYRATE (1070-1072) | gimbal deltas -> PREL QREL RREL (rate TCDU/180) | dap_bodyrate |
| ATTRATES + yaw/pitch atm DAP (1073-1075) | -> ALFA BETA ROLL, pitch/yaw jets ch 5 | dap_attrates, dap_atm_dap |
| 3DDZ/BIASEDZ/4D/SDZ/2D/SDZ (1076) | dead zones -> jet index -1/0/+1 | dap_dz3, dap_biasedz, dap_dz2 |
| EXDAP/EXDAP1-4/EXDAP2 (1077-1079) | extra-atmospheric DAP | dap_exdap* |
| SETJTAG (1081) | 2 s roll update flag | agc_entry_dap_setjtag |
| CM/RCS/GETLCX/COMPAT (1081-1082) | ROLLC ROLL PREL -> LCX/360, -VT/180 | dap_cmrcs |
| WHICHALF..TIMSCAL (1083-1086) | phase plane -> TON1 TOFF TON2 JNDX JNDX1 | agc_entry_dap_roll_phase_plane |
| TIMETST/JETCALL/JETACTN/DZCALL (1087-1089) | waitlist jet schedule, ch 6 | dap_timetst, agc_entry_dap_jetcall, dap_dzcall |
| CM/FDAI/CM/FDAIR/CM/DUMPR/CMTMFILE (1090-1091) | FDAI needles, body rate TM | dap_cm_fdai |
| P61, P61.1-P61.4 (789-791) | displays N61 N60 N63 | agc_entry_p61* (p61_p67.c) |
| P62, P62.1-P62.3 (792-794) | separation, DAP on, N22 | agc_entry_p62* |
| P63 .. P67, P67.1-P67.3, KILLGRP4 (795-802) | mode changes, displays | agc_entry_p63.., entry_p64..entry_p67 |
| S61.1/S61.1A-C (803-805) | IMU orientation check, alarms 1426/1427 | agc_entry_s61_1, agc_entry_s61_1a |
| S61.2, FISHCALC, VGAMCALC, TFF/TRIG, DISPTARG, VRCALC (806-816) | N60/N63 predictions | agc_entry_s61_2 (stub) + pure helpers |
| S62.3 (817-818) | ROLLC ALFAPAD UXA UYA UZA -> CPHI | agc_entry_s62_3 |
| P37 + subroutines (890-933) | return to earth | p37.c (mostly stubs, see Stubs) |
| P11, MATRXJOB, VHHDOT, S11.1, ATERJOB.., SATSTKON/SATSTICK (533-550) | insertion monitor | p11.c |

## Progress
(DONE <C function> <file> / PARTIAL / TODO)
- ENTRY_LEXICON: DONE agc_csm_entry.h struct; TODO entry_lexicon.h/.c constants
- REENTRY_CONTROL: TODO
- CM_BODY_ATTITUDE: TODO
- CM_ENTRY_DIGITAL_AUTOPILOT: TODO
- P61-P67: TODO
- P37_P70: TODO
- P11: TODO
- tests/test_entry.c: TODO

## Deviations
(filled as found)
- -0 in CMDAPMOD modelled as AGC_ENTRY_DAPMOD_MINUS0.

## Stubs
(filled as found)

## Requested core additions
All are declared (not defined) in src/csm/entry_pending.h; delete that file
at integration once the owners provide them.
- Flags (agc_flags_csm.h): CPHIFLAG AMOONFLG NJETSFLG ERADFLAG NODOP01 AVEGFLAG
  XDELVFLG FINALFLG NODOFLAG REFSMFLG LUNAFLAG RETROFLG SLOWFLG DAPBIT1 DAPBIT2
  ENTRYDSP CMDAPARM GAMDIFSW GONEPAST RELVELSW EGSW NOSWITCH HIND INRLSW LATSW
  .05GSW(AGC_FLAG__05GSW) CM/DSTBY(AGC_FLAG_CM_DSTBY) GYMDIFSW NORMSW RVSW GONEBY.
- Channel bits: AGC_CH030_SCCONTROL_OF_SATURN (ch30 BIT10), AGC_CH012_SIVB_TAKEOVER (ch12 BIT9).
- agc_state.h accessors: agc_erase_unitw, _unitr, _delv, _delvref, _refsmmat,
  _set_refsmmat, _cdux/_cduy/_cduz, _imodes33, _tephem/_set_tephem,
  _tliftoff/_set_tliftoff, _set_avegexit(agc_job_id_t), _set_dnlstcod.
- agc_dsky.h: agc_dsky_display(verb, noun, flash, on_terminate, on_proceed, on_enter),
  agc_dsky_clean (CLEANDSP), agc_dsky_blank (BLANKET).
- agc_nav.h: agc_nav_lalotorv, agc_nav_latlong, agc_nav_geterad.
- agc_imu.h: agc_imu_calcga, agc_imu_calcgta.
- dap-csm: agc_dap_needler.
- Shared math kept as local static copies until exported: LOG, EARROT1/2
  (CSM_GEOMETRY), ARCTRIG (INFLIGHT_ALIGNMENT_ROUTINES), CDULOGIC, SIGNMPAC
  (RT8_OP_CODES), POLY (INTERPRETER). Suggest agc_interp.h/agc_nav.h exports.

## Requested jobs-tasks
Jobs: GOTOPOOH SERVEXIT CM_POSE P61 P61_4 P61_1 P61_2 P61_3 P62 P62_1 P62_2 P62_2B
P63 P65_1 P65_1B P67_1 P67_1B PRE_HUNT S61_1A CM_DAPON P37 P11 MATRXJOB VHHDOT
ATERJOB NORMLIZE. Tasks: READGYMB SETJTAG JETCALL WAKEP62 S61_1C ATERTASK
READACCS. (Temporary values in entry_pending.h.)

## Tests written
(TODO)

## Open questions
- REENTRY_CONTROL p. 860 references label GETLEWD which is not defined;
  GETDLEWD is assumed (transcription typo).
