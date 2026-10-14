# CSM Earth entry (P61 to P67)

AGC: Comanche055 P61-P67.agc (pp.789-818), REENTRY_CONTROL.agc
(pp.844-875), CM_BODY_ATTITUDE.agc (pp.883-888), CM_ENTRY_DIGITAL_
AUTOPILOT.agc, ENTRY_LEXICON.agc. C: `src/csm/entry/{programs,
cm_body_attitude,dap,lexicon}.c` and REENTRY_CONTROL split by its own
phases: `src/csm/entry/{entry_guidance,entry_targeting,entry_initial_roll,
entry_huntest,entry_upcontrol,entry_final_phase}.c`.

P61 and P62 are crew dialogues before entry; from P63 on the programs are
selected by the entry guidance itself as the flight passes its phase
boundaries. Guidance runs once per 2-second SERVICER pass as the AVGEXIT
job CM/POSE; its sections are step functions run by `entry_sequencer`
(entry_guidance.c), resuming at GOTOADDR, the AGC's own restart-protected
continuation.

## Preparation (P61, P62)

```mermaid
flowchart TD
    A["V37 N61: P61<br/>programs.c:agc_job_entry_preparation<br/>extended verbs locked out; HEADSUP = -1"] --> A1["S61.1 IMU check<br/>programs.c:agc_job_entry_imu_check"]
    A1 --> A2{"R02 ok? AVERAGEG running?"}
    A2 -- "not running" --> A3["MIDTOAV2 then S61.1C task at the state time:<br/>PREREAD starts AVERAGEG (5.41)<br/>agc_job_entry_imu_check_extrapolated, agc_task_entry_start_average_g"]
    A2 -- "running" --> A4
    A3 --> A4["S61.1A: IMU Y axis within 30 deg of UNIT(VN x RN)?<br/>programs.c:agc_job_entry_imu_axis_test"]
    A4 -- "-Y within 30 deg" --> A5["ALARM 1427 (IMU reversed), V05N09"]
    A4 -- "neither" --> A6["ALARM 1426 (unsatisfactory), V05N09"]
    A4 -- "ok" --> B0
    A5 --> B0
    A6 --> B0
    B0["P61: V06N61 LAT(SPL) LNG(SPL) HEADSUP (GOFLASHR)<br/>agc_job_entry_preparation_target_display<br/>4.1 restart at the display (P61.3)"] -- "PROCEED" --> B1["P61.4: ROLLC from HEADSUP; NEWRNVN, STARTEN1<br/>target at PIPTIME (retry if SERVICER moved RN, VN);<br/>S61.2 EMS data<br/>agc_job_entry_preparation_compute, agc_entry_ems_data"]
    B1 --> B2["P61.1: V06N60 GMAX VPRED GAMMAEI"]
    B2 -- "PROCEED" --> B3["P61.2: V06N63 RTGO VIO TTE<br/>(TTE corrected for display time)"]
    B3 -- "PROCEED" --> C0["P62: NEWMODE 62, entry downlist<br/>programs.c:agc_job_cm_sm_separation_newmode"]
    C0 --> C1["S61.1 again; POSEXIT = P62.3; CM/DAPIC<br/>agc_job_cm_sm_separation_after_check"]
    C1 --> C2["P62.2: AVGEXIT = CM/POSE; V50N25 00041<br/>(perform CM/SM separation)"]
    C2 -- "PROCEED" --> C3["CM/DAPON: entry DAP on, NODOFLAG set<br/>agc_job_cm_sm_separation_proceed"]
    C3 --> C4["P62.1: V06N61 again"]
    C4 -- "PROCEED" --> C5["end attitude hold: ROLLC, ALFACOM = ALFAPAD,<br/>BETACOM = 0; P63FLAG = +1; N22 display<br/>agc_job_cm_sm_separation_end_hold"]
    C5 --> C6{"CMDAPMOD = -1 or +0?"}
    C6 -- "yes" --> D0
    C6 -- "no" --> C7["wait for WAKEP62 (entry DAP: ALFA near trim)<br/>agc_task_wake_cm_sm_separation"]
    C7 --> D0["P63: POSEXIT = STARTENT; N64 display;<br/>P63FLAG = -1; group 4 off<br/>programs.c:agc_job_entry_initialization"]
```

## Entry guidance, every 2 s

```mermaid
flowchart TD
    S["SERVICER pass -> AVGEXIT = CM/POSE<br/>cm_body_attitude.c:agc_job_cm_pose<br/>body triad (CM/POSE, CM/TRIO), angles"] --> PX{"POSEXIT<br/>cm_body_attitude.c:entry_posexit"}
    PX -- "P62.3 desired gimbals" --> PG["S62.3<br/>programs.c:agc_entry_desired_gimbal_angles"]
    PX -- "STARTENT (first pass)" --> SE["STARTENT: MM 63, STARTEN1, GOTOADDR = INITROLL<br/>entry_targeting.c:agc_entry_startent"]
    PX -- "SCALEPOP (every later pass)" --> SP["SCALEPOP: TARGETNG (velocity, drag D,<br/>target RT, THETAH, .05GSW, L/D measured)<br/>entry_guidance.c:agc_entry_scalepop,<br/>entry_targeting.c:agc_entry_targeting"]
    SP --> RF["REFAZE10: restart 5.3; GOTO GOTOADDR<br/>entry_guidance.c:agc_entry_reestablish_sequencer -> entry_sequencer"]
    RF --> IR{"GOTOADDR"}
    IR -- "INITROLL" --> I0{"INITROLL<br/>entry_initial_roll.c:entry_initial_roll_step"}
    I0 -- "D < .05 g" --> LL["LIMITL/D, roll command"]
    I0 -- "first .05 g pass" --> I1["P64: MM 64, N74 (ROLLC VI D)<br/>programs.c:entry_post_0_05g<br/>KAT; GONEPAST down"]
    I1 --> I2{"V >= VFINAL1?"}
    I2 -- "no (short entry)" --> KEP["GOTOADDR = KEP2"]
    I2 -- "yes" --> I3["D0EQ: drag level D0; L/D = +-LAD"]
    I0 -- "INRLSW set" --> I4{"RDOT + VRCONT >= 0?"}
    I4 -- "no" --> I5["KATEST: KAT - D >= 0 ? LIMITL/D : CONSTD"]
    I4 -- "yes" --> H0
    IR -- "HUNTEST" --> H0["HUNTEST: predict range at nominal L/D;<br/>RANGER: DIFF vs THETAH<br/>entry_huntest.c:entry_huntest_step"]
    H0 --> H1{"|DIFF| < 25 NM?"}
    H1 -- "yes" --> U0["GOTOUPSY: P65 (MM 65, V16N69 job);<br/>GOTOADDR = UPCONTRL<br/>programs.c:entry_upcontrol"]
    H1 -- "no" --> H2["GETDLEWD secant step on LEWD;<br/>SIDETRAK: 4.47, PRIO16, iterate<br/>(bounded per pass: DEVIATION)"]
    H2 --> H0
    IR -- "UPCONTRL" --> U1{"UPCONTRL<br/>entry_upcontrol.c:step_upcontrl"}
    U0 --> U1
    U1 -- "V >= V1" --> U2["DOWNCNTL"]
    U1 -- "D < Q7" --> K0["KEP: P66 ballistic, MM 66, N22;<br/>GOTOADDR = KEP2<br/>programs.c:entry_ballistic"]
    U1 -- "RDOT < 0 and V - VL < C18" --> F0
    U1 -- "otherwise" --> U3["CONT1..UPCNTRL3: reference trajectory,<br/>L/D command; NEGTESTS"]
    IR -- "KEP2" --> K1{"D > Q7F + KDMIN?<br/>entry_upcontrol.c:step_kepler_test"}
    K0 --> K1
    KEP --> K1
    K1 -- "no" --> K2["hold ROLLC (zero below .05 g);<br/>S62.3 trim gimbal angles"]
    K1 -- "yes" --> F0["PREFINAL: P67 final phase, MM 67, N66;<br/>EGSW; GOTOADDR = PREDICT3<br/>entry_final_phase.c:step_prefinal -> programs.c:entry_final_phase"]
    IR -- "PREDICT3" --> F1{"PREDICT3: V < VQUIT?<br/>entry_final_phase.c:step_final_predict"}
    F0 --> F1
    F1 -- "yes" --> F2["steering stops; P67.1: V16N67 RTOGO LAT LONG;<br/>end: entry DAP off, SERVEXIT, P00<br/>agc_job_entry_final_phase_display, _end"]
    F1 -- "no" --> F3["range error DNRNGERR -> L/D (HAVDNRNG) or<br/>GONEGLAD past the target; G-LIMITER"]
    U2 --> LL
    U3 --> LL
    I3 --> LL
    I5 --> LL
    F3 --> LL
    LL --> RC["LIMITL/D lateral corridor (LATSW, roll-over);<br/>L355: ROLLC = SIGN ACOS(L/D / LAD)<br/>entry_final_phase.c:step_limitld, step_roll_command"]
    RC --> EX["ENDEXIT / OVERNOUT: entry display ENTRYVN<br/>entry_guidance.c:entry_endexit; SERVNOUT"]
    K2 --> EX
    SE --> SN["SERVNOUT (no display on the first pass)"]
    PG --> EX
```

The entry DAP (`src/csm/entry/dap.c`: READGYMB every 0.1 s, ATTRATES by
T5RUPT, the predictive roll system CM/RCS every 2 s with JETCALL tasks on
channel 6) flies the roll command ROLLC with the RCS jets; the guidance
above only computes ROLLC and the displays. V37 is refused while NODOFLAG is set (from CM/DAPON on;
[program-change.md](program-change.md)).
