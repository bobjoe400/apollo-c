# CSM SPS burn (P40) and RCS burn (P41)

AGC: Comanche055 P40-P47.agc (pp.684-731): P40CSM, P41CSM, P40S/F,
P40SXTY, P40TTOG, TRIMONLY / TST,TRIM, P40S/RS, P40S/SV, TIGBLNK,
TIGAVEG, TIG-5, TIG-0, IGNITION, DOTVCON, DOSTRULL, STEERING, S40.8,
ENGINOFF, DOSPSOFF, DOTVCRCS, POSTBURN, CLOKTASK / CLOCKJOB, V99/V97;
SERVICER207.agc (READACCS, SERVICER, AVERAGEG). C:
`src/csm/burns/sps/{burn,ignition,clock,steer}.c`,
`src/csm/burns/servicer.c`, `src/guidance/*.c`.

The burn is a chain of jobs and waitlist tasks timed from TIG; each link
records its restart point in group 4 (pre-ignition sequence), 3, 5
(SERVICER) or 6 (CLOKTASK) so a restart resumes at the same step.

## Set-up to TIG - 30 s

```mermaid
flowchart TD
    A["V37 N40: P40CSM<br/>burn.c:agc_job_sps_burn<br/>ENG2FLAG down; CSTEER = 0 if XDELVFLG (external dV)"] --> B
    A2["V37 N41: P41CSM<br/>burn.c:agc_job_rcs_burn_csm<br/>ENG2FLAG up; F = 2- or 4-jet (NJETSFLG)"] --> B
    B["P40S/F: NOMTIG = TIG<br/>burn.c:burn_setup_with_thrust"] --> B1["R02 IMU status check<br/>imu:agc_imu_status_check"]
    B1 -- "IMU not usable" --> X1["status returned: program stops<br/>(R02 displays its own alarm)"]
    B1 -- "ok" --> B2["S40.1 initial thrust direction, VGTIG<br/>guidance:agc_guid_initial_velocity_to_gain"]
    B2 --> B3["S40.2,3 preferred attitude<br/>guidance:agc_guid_preferred_thrust_attitude_csm"]
    B3 --> B4["narrow deadband (SETMINDB);<br/>R60 attitude maneuver, continue as<br/>AGC_JOB_SPS_BURN_AFTER_MANEUVER<br/>imu:agc_imu_attitude_maneuver"]
    B4 --> C["P40SXTY: NBRCYCLS = -1, TIMRFLAG<br/>burn.c:agc_job_sps_burn_after_maneuver"]
    C --> C1{"P41 (MODREG odd)?<br/>burn.c:agc_csm_is_rcs_burn"}
    C1 -- "yes" --> C2["P41/DSP: V06N85 by CLOCKJOB,<br/>AVGEXIT = CALCN85<br/>burn.c:rcs_burn_display"]
    C1 -- "no" --> C3["P40TTOG: V06N40; AVGEXIT = STEERING;<br/>V50N25 R1 = 00204 (gimbal test?)"]
    C3 -- "PROCEED" --> C4["TST,TRIM: drive test + trim (18 s)<br/>burn.c:agc_job_gimbal_test_and_trim"]
    C3 -- "ENTER" --> C5["TRIMONLY: trim only (5 s)<br/>burn.c:agc_job_gimbal_trim_only"]
    C3 -- "TERMINATE" --> Z["POST41: P00<br/>ignition.c:agc_job_post41"]
    C4 --> C6["S40.6 gimbal drive / trim tasks<br/>clock.c:agc_task_gimbal_test_trim .. _step"]
    C5 --> C6
    C6 --> D["6.2, 4.23; P40S/RS: CLOKTASK in 10 ms<br/>burn.c:agc_job_preburn_start_clock"]
    C2 --> D
    D --> D1["P40S/SV: integrate to TIG - 29.96 s (MIDTOAV1)<br/>burn.c:agc_job_preburn_state_vector"]
    D1 --> D2{"integration finished late?<br/>(MIDTOAV slipped)"}
    D2 -- "yes" --> D3["TIG = integrated time + 29.96 s"]
    D2 -- "no" --> D4
    D3 --> D4["P40SET: P40TMP = time to TIG - 35 s; LONGCALL TIGBLNK; 4.21<br/>(P40TMP <= 0: LONGCALL of 0 = POODOO 1204, as the AGC)<br/>burn.c:agc_csm_preburn_call_tigblnk"]
```

## TIG - 35 s to ignition

```mermaid
flowchart TD
    T0["TIGBLNK task (TIG-35)<br/>burn.c:agc_task_tigblnk<br/>TIGAVEG in 5 s; CLOCKJOB display off;<br/>NOVAC P40BLNKR (blank DSKY); 4.37"] --> T1["TIGAVEG task (TIG-30)<br/>burn.c:agc_task_tigaveg"]
    T1 --> T2{"P41?"}
    T2 -- "yes" --> T3["V16N85; TTG/0 in 29.96 s; 4.4"]
    T2 -- "no" --> T4["V06N40 back on; TIG-5 in 24.96 s; 4.6"]
    T3 --> T5["PRECHECK: start average-G unless group 5 active<br/>burn.c:agc_task_precheck -> servicer.c:agc_task_preread_csm"]
    T4 --> T5
    T5 --> T6["TIG-5 task<br/>burn.c:agc_task_tig_5_csm<br/>TIG-0 in 5 s; NVWORD1 = V99 flash;<br/>FINDVAC S40.13 TIMEBURN (PRIO20)"]
    T6 --> T7["TIG-0 task<br/>burn.c:agc_task_tig_0_csm: IGNFLAG up"]
    T7 --> T8{"ASTNFLAG: crew already answered V99?"}
    T8 -- "no" --> T9["wait: V99 PROCEED<br/>clock.c:agc_job_engine_enable_proceed<br/>(ASTNFLAG up; IGNFLAG up -> IGNITION in 10 ms)"]
    T8 -- "yes" --> I0
    T9 --> I0["IGNITION task<br/>ignition.c:agc_task_ignition_csm<br/>OGAD = CDUX; TEVENT; ENGONFLG; SPS on (ch 11)"]
    V99E["V99 ENTER: SPS bypassed<br/>clock.c:agc_job_engine_enable_bypass"] --> RT["P40RCS: RCS trim<br/>ignition.c:agc_job_sps_burn_rcs_trim"]
    V99T["V99 TERMINATE<br/>clock.c:agc_job_engine_enable_terminate"] --> Z2["POST41: P00"]
    I0 --> I1{"IMPULSW: impulsive burn?<br/>(S40.13 TIMEBURN found it short:<br/>guidance:agc_guid_timeburn_csm)"}
    I1 -- "yes" --> I2["STRULLSW down; TIG = now + TGO;<br/>ENGINOFF in TGO; IMPLCONT<br/>ignition.c:agc_task_implcont"]
    I1 -- "no" --> I3["STRULLSW up: steer after ullage"]
    I2 --> I4["PREPTVC: RCS DAP killed, 4.3,<br/>DOTVCON in 0.4 s<br/>ignition.c:agc_csm_preptvc"]
    I3 --> I4
    I4 --> I5["DOTVCON: TVC DAP takes T5 (T5LOC = TVCDAPON),<br/>6.3, 4.5, DOSTRULL in 1.6 s<br/>ignition.c:agc_task_dotvcon"]
    I5 --> I6["DOSTRULL: STEERSW if STRULLSW; ullage off;<br/>group 4 off<br/>ignition.c:agc_task_dostrull"]
```

## Powered flight: the 2-second cycle

```mermaid
flowchart TD
    S0["READACCS task every 2 s<br/>servicer.c:agc_task_readaccs_csm<br/>PIPAs read and zeroed (agc_guid_pipasr)"] --> S1["CHEKAVEG: recall READACCS while AVEGFLAG;<br/>FINDVAC SERVICER<br/>servicer.c:chekaveg"]
    S1 --> S2["SERVICER job<br/>servicer.c:agc_job_servicer_csm"]
    S2 --> S3{"PIPA saturated?<br/>(> 6399 pulses / 2 s: pipas_saturated)"}
    S3 -- "no" --> S4["compensation; DVTOTAL += |DELV|<br/>servicer.c:agc_job_serv_dvtotup_csm"]
    S3 -- "yes" --> S5
    S4 --> S5["AVERAGEG: CALCRVG (new R, V, gravity)<br/>servicer.c:agc_job_serv_averageg_csm, guidance:agc_guid_calcrvg"]
    S5 --> S6["restart-protected copy, then AVGEXIT<br/>servicer.c:agc_job_serv_copy_csm, agc_csm_avgexit"]
    S6 --> G0["STEERING (AVGEXIT of P40)<br/>steer.c:agc_job_steering_csm"]
    G0 --> G1["UPDATEVG: aimpoint burns accumulate DELVSUM and<br/>start S40.9 (VTOGAIN, Lambert) when NBRCYCLS < 0<br/>steer.c:agc_csm_updatevg"]
    G1 --> G2["S40.8: VG = VGPREV + BDT - DELVREF<br/>steer.c:agc_csm_cross_product_steering"]
    G2 --> G3{"STEERSW?"}
    G3 -- "no" --> G9
    G3 -- "yes" --> G4{"|DELVREF| < DVTHRESH?"}
    G4 -- "yes: thrust failed" --> G5["LOTHRUST: TVC attitude hold, V97 flash<br/>(unless IDLEFAIL)<br/>steer.c:lothrust"]
    G4 -- "no" --> G6["TGOCALC; TIG = PIPTIME + TGO<br/>guidance:agc_guid_steering_tgo_csm<br/>(failure: ALARM 1407)"]
    G6 --> G7{"TGO < 4 s?"}
    G7 -- "yes" --> G8["S40.81: IMPULSW up, OMEGAC = 0,<br/>STEERSW down<br/>steer.c:short_burn_cutoff"]
    G7 -- "no" --> G10["XPRODUCT: OMEGAC rate commands for the TVC DAP<br/>guidance:agc_guid_steering_cross_product"]
    G5 --> G9
    G8 --> G9
    G10 --> G9{"IMPULSW and SPS on?<br/>steer.c:steering_tail"}
    G9 -- "no" --> G11["SERVEXIT<br/>servicer_common.c:agc_job_servexit"]
    G9 -- "yes" --> G12["ENGINOFF at TIG (at least 1 cs);<br/>3.15 / 5.3 restart points"]
```

## Cut-off and after

```mermaid
flowchart TD
    E0["ENGINOFF task<br/>ignition.c:agc_task_enginoff_csm<br/>MASSTMP = CSMMASS; group 3 off; 4.63"] --> E1["DOSPSOFF: SPSOFF (TEVENT, ENGONFLG down,<br/>SPS off, MASSBACK); 4.27; DOTVCRCS in 2.5 s<br/>ignition.c:agc_task_dospsoff, agc_csm_spsoff"]
    E1 --> E2["DOTVCRCS: wide deadband; RCS DAP on (kills TVC);<br/>mass properties; TVCZAP; 4.35; NOVAC POSTBURN<br/>ignition.c:agc_task_dotvcrcs"]
    E2 --> E3["POSTBURN: V16N40 flash<br/>ignition.c:agc_job_postburn_csm"]
    E3 -- "PROCEED" --> E4["P40RCS: AVGEXIT = CALCN85; 2 s;<br/>P40MINDB narrow deadband; TIGNOW: V16N85 flash<br/>ignition.c:agc_job_sps_burn_rcs_trim .. agc_job_tignow_csm"]
    E3 -- "TERMINATE" --> E5["POST41: AVGEXIT = SERVEXIT; GOTOPOOH<br/>ignition.c:agc_job_post41"]
    E4 -- "PROCEED / TERMINATE" --> E5
```

Engine-failure branch (R40, V97 flash raised by LOTHRUST through the
CLOCKJOB, `clock.c:cknvwrd1`): V97 TERMINATE shuts the SPS down and goes
to P00 (`agc_job_engine_fail_terminate` -> `agc_task_engine_fail_shutdown`
-> `agc_task_engine_fail_rcs_dap`); V97 PROCEED keeps burning with one
R40 bypass (`agc_job_engine_fail_proceed` -> `agc_task_engine_fail_resume`,
IDLEFAIL); V97 ENTER re-ignites (`agc_job_engine_fail_reignite` ->
`agc_task_engine_fail_reignite_prep` -> `agc_task_quickign` -> V99
flash).
