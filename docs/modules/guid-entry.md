# guid-entry -- Comanche entry (P61-P67), entry DAP, CM body attitude, P37, P11

History of this lane: [history/lanes/guid-entry.md](../../history/lanes/guid-entry.md)

## Scope
Comanche055 only: ENTRY_LEXICON, REENTRY_CONTROL, P61-P67, CM_BODY_ATTITUDE,
CM_ENTRY_DIGITAL_AUTOPILOT, P37_P70, P11. Output in src/csm/ plus the public
header include/agc/csm/entry.h (created by this lane).
Note: Comanche P37_P70.agc contains only P37 (return to earth) and its
subroutines; "P70" appears only in the constants title (the LM P70 abort
program is a Luminary routine). There is no Saturn-abort code in the file.

Files:
* include/agc/csm/entry.h -- entry state struct (ENTRY_LEXICON), enums, public API
* include/agc/csm/entry_lexicon.h/.c -- constants with scales; the static entry state
* include/agc/csm/entry_internal.h -- prototypes shared between files of this lane only
* include/agc/csm/entry_pending.h -- PENDING core symbols (flags, accessors, DSKY, nav,
  IMU, job/task ids). Delete at integration; see "Requested core additions" in [the lane history](../../history/lanes/guid-entry.md).
* src/csm/entry/reentry.c, programs.c, cm_body_attitude.c, dap.c, src/csm/burns/return_to_earth/program.c, earth_orbit_insertion.c
* tests/csm/entry.c

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
| VREFER table (876-878) | 13 x 6 SP table | entry_vrefer[] (lexicon.c) |
| CM/POSE, CM/POSE2 (883) | VN UNITR UNITW -> -VREL UXA UYA UZA OLDUYA | agc_entry_cm_pose (cm_body_attitude.c) |
| CM/TRIO (884-886) | AOG/AIG/AMG at PIP, REFSMMAT -> UBX UBY UBZ, roll beta alfa gamma | entry_cm_trio |
| ARCTRIG (INFLIGHT_ALIGNMENT 1357) | sin/4 cos/4 -> angle revs | entry_arctrig (local copy) |
| CDULOGIC (RT8 op codes) | SP CDU -> DP revs | entry_cdulogic (local copy) |
| CM/ATUP/CMTR1/CMTR2/REDOPOSE/CM/POSE3/CORANGOV (887-889) | GAMA GAMDOT ROLL/180 ALFA/180 BETA/180 VMAGI | entry_cm_atup |
| READGYMB/DOBRATE?/DOBRATE/CM/GYMIC (1063-1064) | CDUX/Y/Z -> -DELAOG/AIG/AMG | agc_entry_dap_readgymb (dap.c) |
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
| P61, P61.1-P61.4 (789-791) | displays N61 N60 N63 | agc_job_entry_preparation* (programs.c) |
| P62, P62.1-P62.3 (792-794) | separation, DAP on, N22 | agc_job_cm_sm_separation* |
| P63 .. P67, P67.1-P67.3, KILLGRP4 (795-802) | mode changes, displays | agc_job_entry_initialization.., entry_post_0_05g..entry_final_phase |
| S61.1/S61.1A-C (803-805) | IMU orientation check, alarms 1426/1427 | agc_job_entry_imu_check, agc_job_entry_imu_axis_test |
| S61.2, FISHCALC, VGAMCALC, TFF/TRIG, DISPTARG, VRCALC (806-816) | N60/N63 predictions | agc_entry_ems_data (stub) + pure helpers |
| S62.3 (817-818) | ROLLC ALFAPAD UXA UYA UZA -> CPHI | agc_entry_desired_gimbal_angles |
| P37 + subroutines (890-933) | return to earth | src/csm/burns/return_to_earth/program.c (mostly stubs, see Stubs) |
| P11, MATRXJOB, VHHDOT, S11.1, ATERJOB.., SATSTKON/SATSTICK (533-550) | insertion monitor | earth_orbit_insertion.c |

## Deviations
(filled as found)
- -0 in CMDAPMOD modelled as AGC_ENTRY_DAPMOD_MINUS0.

## Stubs
- agc_job_return_to_earth (P37_P70.agc pp. 891-930): returns AGC_STATUS_UNIMPLEMENTED.
  Not translated: P37/RTE299..RTE390 conic iteration, P37E/RTEVN/RTEDISP
  displays, TMRAD100, INVC100, PREC100..PRECX precision trajectory,
  RTENCK1-3, V2T100..V2T1X, GAMDV10..GAMDVX, DVCALC, P37G..P37W targeting
  and the P370 alarms (605, 612, 613). Only XT1LIM (agc_return_to_earth_xt1lim) is done.
  About 1500 lines of interpretive code; suggest a sibling agent (needs
  INTEGRVS, CSMPREC, TIMERAD, PARAM, GETERAD, TIMETHET, LAT-LONG).

## Tests written
tests/csm/entry.c (compiles with -Werror; links only once the whole
library links -- verified locally with abort stubs for other lanes'
missing symbols: 23 checks, 0 failures, 0 assert reports):
- HUNTEST/RANGER for a nominal up-control state (V 32000 fps, RDOT -300
  fps, D 4 g, LAD .3, LEWD .15, THETAH 1300 NM): result RANGER, NEGAMA
  path (GAMMAL = 0), VL and GAMMAL within 1e-5 of a double-precision model
  of the AGC equations, predicted range 1304.0 NM (model 1304.0, < 1 NM).
- Entry DAP roll phase plane: +20 deg error -> JNDX -1 (positive roll
  jets), TON1 > 0; -20 deg -> JNDX +1; 1 deg -> dead zone.
- XT1LIM: MA = 2 RCON = 2 R(T1), BETA5 = 5 -> X(T1)LIM = 2.

## Program flow and decisions

_Documentation wave (2026-10-12); prose and tables brought to the final tree (HEAD 3b97dc8) by the final pass (2026-10-14). The prose was written by reading the code; the two tables were extracted from the function bodies (flag, alarm, channel and scheduling calls; `agc_erase_*` accessors) and list only what the code does. Files: `src/csm/entry/*.c`, `src/csm/burns/earth_orbit_insertion.c`._

The P61-P67 sequence is drawn in
[flowcharts/earth-entry.md](../overview/flowcharts/earth-entry.md).
CSM only. The decision points:

* **P61** (`src/csm/entry/programs.c:agc_job_entry_preparation*`):
  extended verbs locked out; S61.1 (`agc_job_entry_imu_check`): R02BOTH;
  with AVERAGEG on the IMU axis test runs at once, otherwise the state is
  brought forward (MIDTOAV2) and AVERAGEG started
  (`agc_job_entry_imu_check_extrapolated`, `agc_task_entry_start_average_g`).
  `agc_job_entry_imu_axis_test`: the IMU Y axis must be within 30 deg of
  UNIT(VN x RN); -Y within 30 deg is ALARM 01427 (IMU reversed), neither
  is 01426 (unsatisfactory), then V05N09 and a 10 s DELAYJOB. Displays
  V06N61, V06N60 (S61.2 `agc_entry_ems_data`: GMAX, VPRED, GAMMAEI) and
  V06N63 (RTGO, VIO, TTE corrected for time spent at the display).
* **P62/P63** (`programs.c`, `dap.c:agc_entry_dap_ic`): CM/SM separation
  and the entry DAP on (CM/DAPIC: READGYMB in phase with PIPTIME,
  CM/DSTBY, LATSW, restart groups 6 and 4); P63
  (`agc_job_entry_initialization`) sets POSEXIT = STARTENT and waits for
  .05 g in CM/POSE.
* **Every 2 s** (`cm_body_attitude.c`): CM/POSE after SERVICER builds the
  trajectory and body triads (`agc_entry_cm_pose_triad`,
  `agc_entry_cm_trio`: UYA held at OLDUYA once |VREL| <= VQUIT) and goes
  to POSEXIT; SERVNOUT's BAILOUT outranks the guidance status.
* **Guidance** (REENTRY_CONTROL, split at its own phase boundaries:
  `entry_guidance.c` the sequencer and ENDEXIT, `entry_targeting.c`
  STARTENT and TARGETNG, `entry_initial_roll.c`, `entry_huntest.c`,
  `entry_upcontrol.c` with KEP2, `entry_final_phase.c` PREFINAL,
  PREDICT3, the G limiter, the lateral logic and the roll command; one
  call per pass through `entry_sequencer`, each AGC section a step
  function): TARGETNG (`agc_entry_targeting`:
  D, RT, THETAH, GONEBY, .05GSW, L/D from `targ_ldcalc` skipped on
  overflow) then the phase in GOTOADDR: INITIAL ROLL (INRLSW, `step_inrolout`)
  -> HUNTEST (`step_huntest`: RANGER, PREFINAL or constant drag) ->
  UPCONTROL (`step_upcontrl`, `step_upcontrol_reference`,
  `step_upcontrol_ld`, `step_downcntl` on the stored words) -> KEP2
  (`step_kepler_test`: final phase once D > Q7F + KDMIN, otherwise hold
  ROLLC; below .05 g zero it) -> PREDICT3 (`step_prefinal` sets P67 and
  EGSW; `step_final_predict`: stop steering below VQUIT, else the range
  error DNRNGERR, or GONEGLAD past the target). L/D is limited by
  NEGTESTS (above 175 fpss LATSW cleared, negative L/D -> 0), GLIMITER
  (`step_glimiter`: limit downlift above GMAX/2) and the lateral corridor
  (`step_limitld`, switching when NOSWITCH is clear); ROLLC = SIGN(K2ROLL)
  ACOS(L/D1 / LAD) (`step_roll_command`). The P64-P67 mode changes and
  displays (V16N69 P65, V16N67 P67) are made from these steps;
  `agc_job_entry_final_phase_end` turns the DAP off and ends in GOTOPOOH.
* **Entry DAP** (`dap.c`): READGYMB every 0.1 s (`agc_task_readgymb`:
  first pass only sets GYMDIFSW, `dap_gymic`), body rates, then
  extra-atmospheric (`dap_exdap`: rate and attitude dead zones, 3.6 deg
  YAWLIM, 2 deg/s rate dead zone; P63FLAG +0 with |cos alfa| > .707 wakes
  P62 later) or atmospheric roll (`dap_cm_rcs` -> `dap_phase_plane` ->
  `dap_timetst` keeps jet intervals > 2 cs that fit the 2 s period ->
  `dap_jetlist` WAITLISTs JETCALL); a WAITLIST overflow (BAILOUT 1203)
  leaves the pass without jets. `dap_fdai` integrates the roll error for
  the needles on alternate passes.
* **P11** (`src/csm/burns/earth_orbit_insertion.c`): at liftoff (or V75)
  zero the clock, TEPHEM to liftoff, PREREAD1, launch REFSMMAT and RN/VN
  (MATRXJOB, NO_PRELAUNCH_INITIALIZATION set so P01 is refused later),
  V06N62 every 2 s; ATERJOB drives the FDAI needles and the Saturn
  takeover while DAPBIT1/2 select it.

### Decision points by function

52 of the lane's 160 functions test or change a flag, raise an alarm, test a channel bit or start a job/task (AGC = the label(s) the prolog names; "+F" sets flag F, "-F" clears it; "starts" = NOVAC/FINDVAC, WAITLIST, LONGCALL, DELAYJOB targets).

| C function | AGC | flags tested | flags set (+) / cleared (-) | alarms | channel bits | starts |
|---|---|---|---|---|---|---|
| `agc_job_earth_orbit_insertion_monitor_setup` (earth_orbit_insertion.c) | - | NO_PRELAUNCH_INITIALIZATION | +NO_PRELAUNCH_INITIALIZATION | - | - | task atertask |
| `matrx_tail` (earth_orbit_insertion.c) | MATRXJOB | - | - | - | - | task readaccs |
| `agc_job_matrxjob` (earth_orbit_insertion.c) | MATRXJOB | - | -ERADFLAG +REFSMFLG | - | - | - |
| `agc_csm_velocity_altitude_rate` (earth_orbit_insertion.c) | S11.1 | AMOONFLG | - | - | - | - |
| `agc_task_atertask` (earth_orbit_insertion.c) | ATERTASK | - | - | - | - | job aterjob |
| `agc_job_aterjob` (earth_orbit_insertion.c) | ATERJOB | DAPBIT1 DAPBIT2 | - | - | ch30 SCCTLSATURN | job atreset_after_delay, job aterjob |
| `agc_job_atreset_after_delay` (earth_orbit_insertion.c) | ATRESET | - | - | - | ch12 SIVBTAKEOVER | job aterjob |
| `agc_csm_satstkon` (earth_orbit_insertion.c) | SATSTKON | DAPBIT1 | +DAPBIT1 +DAPBIT2 | - | - | - |
| `agc_task_redosat` (earth_orbit_insertion.c) | REDOSAT | - | - | - | ch12 SIVBTAKEOVER | - |
| `agc_job_cm_pose` (cm_body_attitude.c) | CM/POSE | GAMDIFSW | +GAMDIFSW | - | - | - |
| `dap_gymic` (dap.c) | CM/GYMIC | - | +GYMDIFSW | - | - | - |
| `agc_task_readgymb` (dap.c) | READGYMB READGYM1 DOBRATE | CMDAPARM CM_DSTBY GYMDIFSW | -GYMDIFSW | - | - | task readgymb |
| `dap_exdap` (dap.c) | EXDAP EXDAP4 EXDAP3 | - | - | - | - | task wake_cm_sm_separation |
| `dap_jetlist` (dap.c) | JETCALL1 JETACTN | - | - | - | - | task jetcall |
| `agc_task_jetcall` (dap.c) | JETCALL | CM_DSTBY | - | - | - | - |
| `dap_cm_rcs` (dap.c) | CM/RCS COMPAT | LATSW | +LATSW | - | - | - |
| `agc_task_attrates` (dap.c) | ATTRATES | _05GSW | - | - | - | - |
| `agc_job_cm_dapon_wait` (dap.c) | CM/DAPON | GAMDIFSW | +CMDAPARM +NODOFLAG | - | - | job cm_dapon_wait |
| `agc_job_cm_dapon` (dap.c) | CM/DAPON | - | -DAPBIT1 -DAPBIT2 | - | - | - |
| `agc_entry_dap_ic` (dap.c) | CM/DAPIC | CM_DSTBY | +CM_DSTBY +LATSW | - | - | task readgymb |
| `step_prefinal` (entry_final_phase.c) | PREFINAL | - | +EGSW | - | - | - |
| `step_final_predict` (entry_final_phase.c) | PREDICT3 | GONEBY GONEPAST | +GONEPAST | - | - | - |
| `step_limitld` (entry_final_phase.c) | LIMITL/D L350 | GONEPAST | - | - | - | - |
| `step_roll_command` (entry_final_phase.c) | L355 | - | -NOSWITCH | - | - | - |
| `entry_final_phase_step` (entry_final_phase.c) | - | NOSWITCH | -NOSWITCH | - | - | job entry_final_phase_display |
| `entry_endexit` (entry_guidance.c) | ENDEXIT OVERNOUT NODISKY | ENTRYDSP | - | - | - | - |
| `entry_huntest_step` (entry_huntest.c) | - | HIND | +HIND | - | - | - |
| `step_inrolout` (entry_initial_roll.c) | INROLOUT | INRLSW | +INRLSW | - | - | - |
| `agc_job_pre_hunt` (entry_initial_roll.c) | PRE-HUNT | - | -HIND | - | - | - |
| `entry_initial_roll_step` (entry_initial_roll.c) | - | INRLSW _05GSW | -GONEPAST | - | - | - |
| `targ_geteta` (entry_targeting.c) | GETETA SUBETA SUBETA2 | EGSW | +RELVELSW | - | - | - |
| `agc_entry_targeting` (entry_targeting.c) | TARGETNG | GONEBY RELVELSW | -GONEBY +GONEBY -_05GSW +_05GSW | - | - | - |
| `entry_initial_target` (entry_targeting.c) | STARTEN1 | - | -ERADFLAG -LUNAFLAG | - | - | - |
| `agc_entry_startent` (entry_targeting.c) | STARTENT | - | +ENTRYDSP +GONEPAST +LATSW | - | - | - |
| `step_upcontrl` (entry_upcontrol.c) | UPCONTRL VLTEST | - | +NOSWITCH | - | - | - |
| `step_negtests` (entry_upcontrol.c) | NEGTESTS | - | -LATSW | - | - | - |
| `step_kepler_test` (entry_upcontrol.c) | KEP2 | _05GSW | - | - | - | - |
| `agc_job_cm_sm_separation_end_hold` (programs.c) | P62.1 PROCEED | - | +ENTRYDSP | - | - | - |
| `agc_task_wake_cm_sm_separation` (programs.c) | WAKEP62 | - | - | - | - | job entry_initialization |
| `entry_post_0_05g` (programs.c) | P64 | _05GSW | - | - | - | - |
| `entry_upcontrol` (programs.c) | P65 | - | - | - | - | job entry_upcontrol_display |
| `agc_job_entry_upcontrol_display` (programs.c) | P65.1 | ENTRYDSP | -ENTRYDSP | - | - | - |
| `agc_job_entry_upcontrol_proceed` (programs.c) | P65.1 PROCEED | ENTRYDSP | +ENTRYDSP | - | - | - |
| `entry_ballistic` (programs.c) | P66 | - | +ENTRYDSP | - | - | - |
| `entry_final_phase` (programs.c) | P67 | - | +ENTRYDSP | - | - | - |
| `agc_job_entry_final_phase_end` (programs.c) | P67.1 | CM_DSTBY | -CM_DSTBY -GYMDIFSW | - | - | - |
| `agc_entry_final_phase_latlong` (programs.c) | P67.2 | - | -ERADFLAG -LUNAFLAG | - | - | - |
| `agc_job_entry_imu_check_extrapolated` (programs.c) | S61.1 | - | - | - | - | task entry_start_average_g |
| `agc_job_entry_imu_check` (programs.c) | S61.1 | AVEGFLAG | - | - | - | - |
| `agc_task_entry_start_average_g` (programs.c) | S61.1C | - | - | - | - | job entry_imu_axis_test |
| `agc_job_entry_imu_axis_test` (programs.c) | S61.1A S61.1B | - | - | ALARM code) | - | job entry_imu_check_return |
| `agc_entry_desired_gimbal_angles` (programs.c) | S62.3 | - | -CPHIFLAG | - | - | - |

### Shared erasables read and written

Accessors of `include/agc/core/state.h` and the lanes' public headers called by this lane, with the number of its functions that read / write each. Lane-private erasables are `static` in the lane's files or in its state struct; their scales are on the declarations.

| erasable (accessor) | meaning and scale (accessor prolog) | read by | written by |
|---|---|---|---|
| `aig` | none (AIG erasable, CM_ENTRY_DIGITAL_AUTOPILOT.agc p. 1063) -- accessor for other lanes.; returns agc_sp_t -- agc_sp_t -- AIG inner gimbal angle /180, | 1 | 0 |
| `amg` | none (AMG erasable, CM_ENTRY_DIGITAL_AUTOPILOT.agc p. 1063) -- accessor for other lanes.; returns agc_sp_t -- agc_sp_t -- AMG middle gimbal angle /180 | 1 | 0 |
| `aog` | none (AOG erasable, CM_ENTRY_DIGITAL_AUTOPILOT.agc p. 1063) -- accessor for other lanes.; returns agc_sp_t -- agc_sp_t -- AOG outer gimbal angle /180, | 1 | 0 |
| `cdux` | CDU X (outer gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (register CDUX, 032) | 3 | 0 |
| `cduy` | CDU Y (inner gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUY) | 3 | 0 |
| `cduz` | CDU Z (middle gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUZ) | 3 | 0 |
| `cm_gymdt` | P40TMP (CSM, DP cs), SAVET-30 (LM, DP cs), ZOOMTIME (LM pad load, cs), S61DT and CM/GYMDT (CSM entry, cs): read by the restart tables. | 2 | 2 |
| `dapdatr1` | DAP configuration word 1 (DAPDATR1, DSP noun 46 R1) | 1 | 1 |
| `delv` | accumulated PIPA delta-V, stable-member axes (DELV, I(6)): PIPA pulses scale 2^14 (count in the high word); 1 pulse = 5.85 cm/s CSM, 1 cm/s LM. KPIP1  | 1 | 0 |
| `extvbact` | EXTVBACT (ERASABLE_ASSIGNMENTS; TESTXACT p.237 C / p.264 L) -- the extended verb activity bits, read RETURNS: EXTVBACT, 15 bits; returns EXTVBACT, 15  | 0 | 1 |
| `imodes33` | last-sampled channel 33 / DAP-enable bits, T4RUPT C33TEST (IMODES33, B(1)PRM) | 1 | 0 |
| `modreg` | MODREG major mode (program) | 1 | 0 |
| `paxerr1` | PAXERR1 (CSM entry DAP): the integrated roll error, sent on the entry downlist. AGC: PAXERR1 (Comanche055 ERASABLE_ASSIGNMENTS.agc); returns agc_sp_t  | 1 | 0 |
| `piptime` | time of the PIPA reading / this vehicle's state vector, centiseconds, scale 2^28 (PIPTIME, B(2)PRM) | 4 | 0 |
| `prel` | PREL +k (CSM entry DAP): the body rates x TCDU, sent after CMDAPMOD on the entry downlist. AGC: PREL, QREL, RREL (Comanche055 ERASABLE_ASSIGNMENTS.agc | 1 | 0 |
| `refsmmat` | reference-to-stable-member matrix, row-major, DP elements, scale 2^1 (REFSMMAT, I(18D)PRM; 'REFSMMAT SCALE OF B1') | 3 | 1 |
| `rls` | landing site vector, moon-fixed reference, meters, scale 2^27 (RLS; R30 'SCALED AT (-27)M'); used by P20/P22/P52 lat-long, R30, P57, landing | 1 | 0 |
| `rn` | this vehicle's position, meters, scale 2^29 (earth) or 2^27 (moon) (RN, B(6)PRM) | 4 | 0 |
| `s61dt` | P40TMP (CSM, DP cs), SAVET-30 (LM, DP cs), ZOOMTIME (LM pad load, cs), S61DT and CM/GYMDT (CSM entry, cs): read by the restart tables. | 0 | 1 |
| `tephem` | time of the ephemeris epoch (TEPHEM = TIMSUBO, TP, centiseconds, scale 2^42: L "CSEC B-42 (TRIPLE PRECISION)"; P11 sets it at liftoff) | 2 | 1 |
| `tevent` | time of event for downlist, centiseconds, scale 2^28 (TEVENT; TLIFTOFF in P11) | 2 | 2 |
| `time1` | ---- time: TIME1/TIME2 (centiseconds, DP), TIME3..TIME6 are exec-private | 6 | 0 |
| `time2` | ---- time: TIME1/TIME2 (centiseconds, DP), TIME3..TIME6 are exec-private | 1 | 0 |
| `vn` | this vehicle's velocity, m/cs, scale 2^7 (earth) or 2^5 (moon) (VN, B(6)PRM) | 5 | 0 |

### AGC labels -> C functions

The "AGC" column above names the label each listed function translates; the complete label -> function -> file:line index for the lane (and the tree) is [docs/overview/xref.md](../overview/xref.md), generated by `tools/xref.py`.
