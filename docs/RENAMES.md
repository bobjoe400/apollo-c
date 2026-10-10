# RENAMES.md -- naming map (phase 1 of docs/agents/naming.md)

Status: PHASE 1 MAP, NOT APPLIED. Produced by agent `naming` on 2026-10-10 for lead review.

Columns: `old` | `new` | where `old` occurs now (code files; `docs:` = docs/modules notes) | AGC source label + the comment the name was derived from.
Rows marked `[S-section]` rename GSOP section-numbered subroutines (S40.9, S61.1 ...); they go beyond the P/R/V/N regex and follow the dap-csm precedent in NAMING.md -- the lead may drop that group. `[1/-label]` rows follow the 1/ACCS precedent.

Conventions used for every row:
* "first-attempt name" = a name the stopped mechanical rename put in the tree (commit 23aa58d); both it and the original numbered name map to the same new name, because some lanes (src/guidance, src/programs, dap) still use the original.
* Word count (rule 4) counts a canonical NAMING.md stem as one term: `agc_job_` + `entry_upcontrol` + `display`.
* `_restart` = the restart point a job resumes at after a PHASCHNG (replaces the lane-invented `_B`, `TJOB`, `EJOB`, `TTASK2`).
* R40/V97/V99 responses are named by what the response does (terminate / proceed / reignite / bypass), not by the T/P/E letter. Jobs and tasks use the routine stems `engine_fail` (R40, V97) and `engine_enable` (V99); the NV-word constants keep the verb stems `engine_fail_procedure` / `engine_enable_request`.
* "(kept)" rows are numbered old names whose first-attempt replacement is already compliant; they are listed so the whole map is one table.

## guid-powered -- P30/P31, P40/P41/P42/P47, R40 engine-fail (V97/V99), CLOKJOB

| old | new | files | AGC source / comment |
|---|---|---|---|
| `AGC_JOB_P30` | `AGC_JOB_EXTERNAL_DELTA_V` | src/guidance/guid_pending.h src/guidance/p30.c | CM P30-P37.agc P30: "+30(EXTERNAL DELTA-V TARGETTING PROGRAM)" |
| `agc_job_p30` | `agc_job_external_delta_v` | src/guidance/p30.c | CM P30-P37.agc P30: "EXTERNAL DELTA-V TARGETTING PROGRAM" |
| `AGC_JOB_P31` | `AGC_JOB_LAMBERT_AIMPOINT_GUIDANCE` | src/guidance/guid_pending.h src/guidance/p30.c | CM P30-P37.agc P31: "P31(GENERAL LAMBERT AIMPOINT GUIDANCE)" |
| `agc_job_p31` | `agc_job_lambert_aimpoint_guidance` | src/guidance/p30.c | CM P30-P37.agc P31: "GENERAL LAMBERT AIMPOINT GUIDANCE" |
| `AGC_JOB_P30_N81` | `AGC_JOB_EXTERNAL_DELTA_V_DISPLAY_DV` | src/guidance/guid_pending.h src/guidance/p30.c | CM CNTNUP30 / LM P30N33+2: "CNTUP30 - DISPLAYS DELV(LV)" (V06N81) |
| `agc_job_p30_n81` | `agc_job_external_delta_v_display_dv` | src/guidance/p30.c | CM CNTNUP30: "CNTUP30 - DISPLAYS DELV(LV)" |
| `AGC_JOB_P31_N81` | `AGC_JOB_LAMBERT_AIMPOINT_DISPLAY_DV` | src/guidance/guid_pending.h src/guidance/p30.c | CM P31 -> CNTNUP30: "DISPLAYS DELV(LV)" (V06N81 DELVLVC) |
| `agc_job_p31_n81` | `agc_job_lambert_aimpoint_display_dv` | src/guidance/p30.c | CM P31 -> CNTNUP30: "DISPLAYS DELV(LV)" |
| `AGC_JOB_P30_CALC` | `AGC_JOB_EXTERNAL_DELTA_V_COMPUTE` | src/guidance/guid_pending.h src/guidance/p30.c | CM P30 after N81 PRO: CALL S30.1 then PARAM30 |
| `agc_job_p30_calc` | `agc_job_external_delta_v_compute` | src/guidance/p30.c | CM P30 after N81 PRO: CALL S30.1 then PARAM30 |
| `AGC_JOB_P31_CALC` | `AGC_JOB_LAMBERT_AIMPOINT_COMPUTE` | src/guidance/guid_pending.h src/guidance/p30.c | CM P31 after N33 PRO: reset UPDATFLG/NORMSW, CALL S31.1 |
| `agc_job_p31_calc` | `agc_job_lambert_aimpoint_compute` | src/guidance/p30.c | CM P31 after N33 PRO: CALL S31.1 |
| `AGC_JOB_P30_PARAM` | `AGC_JOB_EXTERNAL_DELTA_V_DISPLAY_PARAMETERS` | src/guidance/guid_pending.h src/guidance/p30.c | PARAM30: "DISPLAYS APOGEE,PERIGEE,DELV(MAG),MGA" (V06N42) |
| `agc_job_p30_param` | `agc_job_external_delta_v_display_parameters` | src/guidance/p30.c | PARAM30: "DISPLAYS APOGEE,PERIGEE,DELV(MAG),MGA" |
| `AGC_JOB_P30_ALL_DONE` | `AGC_JOB_EXTERNAL_DELTA_V_ACCEPT` | src/guidance/guid_pending.h src/guidance/p30.c | LM PARAM30 PROCEED: "SETGO XDELVFLG (FOR P40'S: EXTERNAL DELTA-V GUIDANCE) REVN1645" |
| `agc_job_p30_all_done` | `agc_job_external_delta_v_accept` | src/guidance/p30.c | LM PARAM30 PROCEED: sets XDELVFLG, goes to REVN1645 |
| `AGC_JOB_P30_REFTEST` | `AGC_JOB_EXTERNAL_DELTA_V_REFTEST` | src/guidance/guid_pending.h src/guidance/p30.c | REFTEST (word label kept): "REFSMFLAG=0 THEN BRANCH TO NOTSET" else +MGA |
| `AGC_JOB_EXTERNAL_DELTA_V_DISP45` | `AGC_JOB_MARKS_TFI_MGA_DISPLAY` | src/csm/external_delta_v_csm.c | CM DISP45: V16N45 flash (N45 = marks, time from ignition, MGA); "USED IN P30" |
| `AGC_JOB_P30_DISP45` | `AGC_JOB_MARKS_TFI_MGA_DISPLAY` | src/guidance/guid_pending.h | CM DISP45: V16N45 (old guid_pending.h name of the same job) |
| `agc_job_external_delta_v_disp45` | `agc_job_marks_tfi_mga_display` | include/agc/agc_guid.h src/csm/external_delta_v_csm.c | CM DISP45: CAF V16N45, GOFLASHR |
| `AGC_JOB_EXTERNAL_DELTA_V_END45` | `AGC_JOB_MARKS_TFI_MGA_DONE` | src/csm/external_delta_v_csm.c | CM END45: PROCEED on N45: CLEAR TIMRFLAG, exit |
| `AGC_JOB_P30_END45` | `AGC_JOB_MARKS_TFI_MGA_DONE` | src/guidance/guid_pending.h | CM END45 (old guid_pending.h name of the same job) |
| `agc_job_external_delta_v_end45` | `agc_job_marks_tfi_mga_done` | include/agc/agc_guid.h src/csm/external_delta_v_csm.c | CM END45: CLEAR TIMRFLAG |
| `AGC_JOB_REVN1645` | `AGC_JOB_MARKS_TFI_MGA_MONITOR` | src/guidance/guid_pending.h src/guidance/p30.c | LM REVN1645: "TRKMKCNT, TTOGO, +MGA" redisplayed V16N45 loop |
| `p30_p31_entry` | `ignition_time_entry` | src/guidance/p30.c | CM P30/P31: "P30/P31 - DISPLAYS TIG" (V06N33 T OF IGN) |
| `agc_guid_s30_1` | `agc_guid_external_delta_v_vectors` | include/agc/agc_guid.h src/guidance/p30.c docs:guid-powered | S30.1: "COMPUTE PERIGEE, APOGEE AND DELTA-V REQUIRED IN REF. COORDS (DELVSIN)" [S-section] |
| `s30_1_csm` | `external_delta_v_vectors_csm` | src/guidance/p30.c | S30.1 CM body [S-section] |
| `s30_1_lm` | `external_delta_v_vectors_lm` | src/guidance/p30.c | S30.1 LM body [S-section] |
| `agc_guid_s31_1` | `agc_guid_lambert_aimpoint_vectors` | include/agc/agc_guid.h src/guidance/p30.c docs:guid-powered | S31.1: "COMPUTES DELV IN REF AND LV COORDS, MAG OF DELV, INTERCEPT TIME" [S-section] |
| `AGC_JOB_P40` | `AGC_JOB_DPS_BURN` | src/guidance/guid_pending.h | P40CSM/P40LM job id; only LM code references it (CSM registers agc_job_sps_burn directly) |
| `agc_job_sps_burn_csm` | `agc_job_sps_burn` | include/agc/agc_guid.h src/csm/sps_burn.c | CM P40CSM (vehicle suffix redundant with sps) |
| `agc_job_dps_burn_lm` | `agc_job_dps_burn` | include/agc/agc_guid.h src/lm/dps_burn.c | LM P40LM (vehicle suffix redundant with dps) |
| `AGC_JOB_P41` | `AGC_JOB_RCS_BURN` | src/guidance/guid_pending.h | P41CSM/P41LM: RCS thrusting |
| `AGC_JOB_P42` | `AGC_JOB_APS_BURN` | src/guidance/guid_pending.h | LM P42LM: APS thrusting |
| `agc_job_aps_burn_lm` | `agc_job_aps_burn` | include/agc/agc_guid.h src/lm/dps_burn.c | LM P42LM (vehicle suffix redundant with aps) |
| `AGC_JOB_P42STAGE` | `AGC_JOB_APS_BURN_STAGE_CHECKED` | src/lm/ignition.c | LM P42STAGE: REP40ALM PROCEED "FOR P42, ALLOW CREW TO PROCEED" after stage check |
| `AGC_JOB_APS_BURN_STAGE` | `AGC_JOB_APS_BURN_STAGE_CHECKED` | src/lm/dps_burn.c src/lm/guid_pending_lm.h | LM P42STAGE (first-attempt name) |
| `AGC_JOB_P47` | `AGC_JOB_THRUST_MONITOR` | src/guidance/guid_pending.h | P47CSM/P47LM thrust monitor |
| `AGC_JOB_P47BODY` | `AGC_JOB_THRUST_MONITOR_RESET` | src/guidance/guid_pending.h | P47BODY: "CLEAR DISPLAY AND ACCUMULATOR STORAGE UPON INITIATION OR ENTER RESPONSE" |
| `AGC_JOB_THRUST_MONITOR_BODY` | `AGC_JOB_THRUST_MONITOR_RESET` | src/csm/sps_burn_ignition.c src/exec/exec_pending.h src/exec/restart_tables.c src/lm/dps_burn.c | P47BODY (first-attempt name) |
| `agc_job_thrust_monitor_body_csm` | `agc_job_thrust_monitor_reset_csm` | include/agc/agc_guid.h src/csm/sps_burn_ignition.c | CM P47BODY |
| `agc_job_thrust_monitor_body_lm` | `agc_job_thrust_monitor_reset_lm` | include/agc/agc_guid.h src/lm/dps_burn.c | LM P47BODY |
| `AGC_JOB_P47BOD` | `AGC_JOB_THRUST_MONITOR_DISPLAY` | src/guidance/guid_pending.h | P47BOD: "LOWER PRIO THAN CALCN83", P47/DSP flash V16N83 |
| `AGC_JOB_THRUST_MONITOR_BOD` | `AGC_JOB_THRUST_MONITOR_DISPLAY` | src/csm/sps_burn_ignition.c | P47BOD (first-attempt name) |
| `agc_job_thrust_monitor_bod_csm` | `agc_job_thrust_monitor_display_csm` | include/agc/agc_guid.h src/csm/sps_burn_ignition.c | CM P47BOD / P47/DSP |
| `agc_job_thrust_monitor_bod_lm` | `agc_job_thrust_monitor_display_lm` | include/agc/agc_guid.h src/lm/dps_burn.c | LM P47BOD |
| `AGC_JOB_CALCN83` | `AGC_JOB_CALC_DELTA_V_BODY` | src/guidance/guid_pending.h | CALCN83: DELVIMU = S41.1(DELVCTL+DELVREF) for N83 (DELTA V body), S11.1 for N62 |
| `AGC_JOB_CALCN85` | `AGC_JOB_CALC_VG_BODY` | src/guidance/guid_pending.h src/lm/ignition.c | CALCN85: "NEW VG, S40.8; P40CNV85 COMPUTE VGBODY" (N85 = VG body) |
| `AGC_JOB_CALCN83_B` | `AGC_JOB_CALC_DELTA_V_RESTART` | src/csm/guid_pending_csm.h src/csm/sps_burn_steer.c | CM CALCN83 after the 5.3 type-B phase change (GENTRAN DV47TEMP->DELVCTL) |
| `agc_job_calc_delta_v_body_b` | `agc_job_calc_delta_v_restart` | include/agc/agc_guid.h src/csm/sps_burn_steer.c | CM CALCN83 restart point after PHASCHNG OCT 10035 |
| `AGC_JOB_P40_AFTER_R60` | `AGC_JOB_SPS_BURN_AFTER_MANEUVER` | src/guidance/guid_pending.h | CM P40SXTY: return from R60CSM "ATTITUDE MANEUVER" |
| `AGC_JOB_SPS_BURN_AFTER_ATTITUDE_MANEUVER` | `AGC_JOB_SPS_BURN_AFTER_MANEUVER` | src/csm/sps_burn.c | first-attempt name (5 words) |
| `agc_job_sps_burn_after_attitude_maneuver` | `agc_job_sps_burn_after_maneuver` | include/agc/agc_guid.h src/csm/sps_burn.c | first-attempt name (5 words) |
| `AGC_JOB_P41_AFTER_R60` | `AGC_JOB_RCS_BURN_AFTER_MANEUVER` | src/guidance/guid_pending.h | LM P41MANU after R60LEM |
| `AGC_JOB_RCS_BURN_AFTER_ATTITUDE_MANEUVER` | `AGC_JOB_RCS_BURN_AFTER_MANEUVER` | src/lm/dps_burn.c | first-attempt name |
| `agc_job_rcs_burn_after_attitude_maneuver` | `agc_job_rcs_burn_after_maneuver` | include/agc/agc_guid.h src/lm/dps_burn.c | first-attempt name |
| `AGC_JOB_P40_TRIMONLY` | `AGC_JOB_GIMBAL_TRIM_ONLY` | src/guidance/guid_pending.h | CM TRIMONLY: "SET MRKRTEMP FOR GIMBAL TRIM (-1)" |
| `AGC_JOB_SPS_BURN_TRIMONLY` | `AGC_JOB_GIMBAL_TRIM_ONLY` | src/csm/sps_burn.c | first-attempt name |
| `agc_job_sps_burn_trimonly` | `agc_job_gimbal_trim_only` | include/agc/agc_guid.h src/csm/sps_burn.c | first-attempt name |
| `AGC_JOB_P40_TSTTRIM` | `AGC_JOB_GIMBAL_TEST_AND_TRIM` | src/guidance/guid_pending.h | CM TST,TRIM: "SET UP FOR GIMB DRIVE TEST AND TRIM (+1)" |
| `AGC_JOB_SPS_BURN_TSTTRIM` | `AGC_JOB_GIMBAL_TEST_AND_TRIM` | src/csm/sps_burn.c | first-attempt name |
| `agc_job_sps_burn_tsttrim` | `agc_job_gimbal_test_and_trim` | include/agc/agc_guid.h src/csm/sps_burn.c | first-attempt name |
| `AGC_JOB_SPS_BURN_S_RS` | `AGC_JOB_PREBURN_START_CLOCK` | src/csm/guid_pending_csm.h src/csm/sps_burn.c | first-attempt name (opaque S_RS) |
| `agc_job_sps_burn_s_rs` | `agc_job_preburn_start_clock` | include/agc/agc_guid.h src/csm/sps_burn.c | first-attempt name |
| `agc_csm_sps_burn_s_rs` | `agc_csm_preburn_start_clock` | src/csm/sps_burn.c src/csm/sps_burn_ignition.c src/csm/sps_burn_internal.h | first-attempt name |
| `AGC_JOB_P40S_SV` | `AGC_JOB_PREBURN_STATE_VECTOR` | src/guidance/guid_pending.h | CM P40S/SV: "TIG-29.96 -> MIDTOAV1 (integrate state), P40SNEWM if too long; LONGCALL TIGBLNK" |
| `AGC_JOB_SPS_BURN_S_SV` | `AGC_JOB_PREBURN_STATE_VECTOR` | src/exec/exec_pending.h src/exec/restart_tables.c | first-attempt name (opaque S_SV) |
| `agc_job_sps_burn_s_sv` | `agc_job_preburn_state_vector` | include/agc/agc_guid.h src/csm/sps_burn.c | first-attempt name |
| `sps_burn_s_f` | `burn_setup_with_thrust` | src/csm/sps_burn.c | first-attempt name |
| `AGC_JOB_P40BLNKR` | `AGC_JOB_PREBURN_BLANK_DSKY` | src/guidance/guid_pending.h | CM P40BLNKR: "CLEANDSP REMOVE RESIDUE" (from TIGBLNK) |
| `AGC_JOB_SPS_BURN_BLNKR` | `AGC_JOB_PREBURN_BLANK_DSKY` | src/csm/sps_burn.c | first-attempt name |
| `agc_job_sps_burn_blnkr` | `agc_job_preburn_blank_dsky` | include/agc/agc_guid.h src/csm/sps_burn.c | first-attempt name |
| `AGC_JOB_P41BLANK` | `AGC_JOB_RCS_BURN_BLANK_DSKY` | src/guidance/guid_pending.h src/lm/ignition.c | LM P41BLANK: "BLANK DSKY" (CLOKJOB IS NOT RUNNING DURING P41) |
| `agc_job_rcs_burn_blank` | `agc_job_rcs_burn_blank_dsky` | include/agc/agc_guid.h | first-attempt name |
| `AGC_JOB_P40RCS` | `AGC_JOB_SPS_BURN_RCS_TRIM` | src/guidance/guid_pending.h | CM P40RCS: "V99N40 ENTERS HERE ON A P40 BYPASS SPS"; AVEGEXIT=CALCN85, then N85 residual trim |
| `AGC_JOB_SPS_BURN_RCS` | `AGC_JOB_SPS_BURN_RCS_TRIM` | src/csm/sps_burn_clock.c src/csm/sps_burn_ignition.c | first-attempt name |
| `agc_job_sps_burn_rcs` | `agc_job_sps_burn_rcs_trim` | include/agc/agc_guid.h src/csm/sps_burn_ignition.c | first-attempt name |
| `AGC_JOB_P40MINDB` | `AGC_JOB_SPS_BURN_MIN_DEADBAND` | src/guidance/guid_pending.h | CM P40MINDB: "TCR SETMINDB" (minimum DAP deadband) then TIGNOW |
| `AGC_JOB_SPS_BURN_MINDB` | `AGC_JOB_SPS_BURN_MIN_DEADBAND` | src/csm/sps_burn_ignition.c | first-attempt name |
| `agc_job_sps_burn_mindb` | `agc_job_sps_burn_min_deadband` | include/agc/agc_guid.h src/csm/sps_burn_ignition.c | first-attempt name |
| `AGC_JOB_P40_ALARM` | `AGC_JOB_STAGE_ALARM_DISPLAY` | src/guidance/guid_pending.h src/lm/ignition.c | LM REP40ALM: V05N09 alarm 1706 "PROGRAM SELECTION NOT CONSISTENT WITH VEHICLE CONFIGURATION" |
| `agc_job_repeat_dps_burn_alm` | `agc_job_stage_alarm_display` | include/agc/agc_guid.h src/lm/dps_burn.c | first-attempt name |
| `AGC_JOB_P40_ALARM_PRO` | `AGC_JOB_STAGE_ALARM_PROCEED` | src/guidance/guid_pending.h | LM REP40ALM PROCEED: "CHECK FOR P42" |
| `agc_job_repeat_dps_burn_alm_pro` | `agc_job_stage_alarm_proceed` | include/agc/agc_guid.h | first-attempt name |
| `dps_burn_alm` | `stage_alarm` | src/lm/dps_burn.c | first-attempt name |
| `dps_burn_in` | `burn_program_start` | src/lm/dps_burn.c | first-attempt name |
| `AGC_JOB_P40_DSKY_DONE` | `AGC_JOB_BURN_DISPLAY_DONE` | src/guidance/guid_pending.h | alias of ENDOFJOB after burn displays |
| `AGC_JOB_P40AUTO_RET` | `AGC_JOB_AUTO_CONTROL_RECYCLE` | src/guidance/guid_pending.h | LM P40A/P: "CHECK PGNCS CONTROL AND AUTO STABILIZATION MODES"; V50N25 R1=203 recycle |
| `agc_job_dps_burn_auto_ret` | `agc_job_auto_control_recycle` | include/agc/agc_guid.h | first-attempt name |
| `dps_burn_auto_ret` | `auto_control_recycle` | include/agc/agc_guid.h | first-attempt name |
| `agc_lm_p40auto` | `agc_lm_check_auto_control` | src/lm/ignition.c | LM P40AUTO: "SUBROUTINE TO CHECK PGNCS CONTROL AND AUTO STABILIZATION MODES" |
| `agc_lm_dps_burn_auto` | `agc_lm_check_auto_control` | include/agc/agc_guid.h src/lm/landing_aborts.c | first-attempt name |
| `AGC_TASK_P40ZOOM` | `AGC_TASK_DPS_BURN_ZOOM` | src/guidance/guid_pending.h | LM P40ZOOM: THRUST=BIT13, CH14 BIT4 (throttle to full); ZOOM is the AGC word |
| `AGC_TASK_P63ZOOM` | `AGC_TASK_BRAKING_PHASE_ZOOM` | src/guidance/guid_pending.h src/lm/ignition.c | LM P63ZOOM: AVEGEXIT=LUNLAND, FLATOUT |
| `AGC_TASK_TIG_30_1` | `AGC_TASK_RESTORE_BURN_DISPLAY` | src/guidance/guid_pending.h src/lm/ignition.c | LM TIG-30.1: "SET UP JOB TO RESTORE DISPLAY AT TIG-30" (TIG-30A) |
| `agc_task_tig_30_1` | `agc_task_restore_burn_display` | include/agc/agc_guid.h src/lm/ignition.c | LM TIG-30.1 |
| `AGC_LM_WHICH_P40` | `AGC_LM_WHICH_DPS_BURN` | src/guidance/guid_state.c src/lm/ignition.c | WHICH -> P40TABLE |
| `agc_csm_rcs_burn_dsp` | `agc_csm_rcs_burn_display` | src/csm/sps_burn.c src/csm/sps_burn_ignition.c src/csm/sps_burn_internal.h | first-attempt name |
| `agc_csm_sps_burn_convert_vg_body` | `agc_csm_convert_vg_body` | src/csm/sps_burn_clock.c src/csm/sps_burn_ignition.c src/csm/sps_burn_internal.h src/csm/sps_burn_steer.c | first-attempt name (5 words) |
| `SPS_BURN_CKLS2` | `GIMBAL_TEST_CHECKLIST` | src/csm/sps_burn.c | first-attempt name |
| `sps_burn_tmp` | `preread_delay` | include/agc/agc_guid.h src/csm/sps_burn.c src/csm/sps_burn_ignition.c | first-attempt name |
| `RST_E_SPS_BURN_TMP` | `RST_E_PREREAD_DELAY` | src/exec/exec_internal.h src/exec/restart_tables.c | first-attempt name |
| `AGC_JOB_V97T` | `AGC_JOB_ENGINE_FAIL_TERMINATE` | src/guidance/guid_pending.h | CM V97T: TERMINATE response to V97: "KILL GROUP 6 (CLOKTASK)", V97TTASK |
| `AGC_JOB_ENGINE_FAIL_PROCEDURE_T` | `AGC_JOB_ENGINE_FAIL_TERMINATE` | src/csm/sps_burn_clock.c | first-attempt name (opaque _T) |
| `agc_job_engine_fail_procedure_t` | `agc_job_engine_fail_terminate` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | first-attempt name |
| `AGC_JOB_V97P` | `AGC_JOB_ENGINE_FAIL_PROCEED` | src/guidance/guid_pending.h | CM V97P: PROCEED response to V97 -> V97PTASK (resume burn) |
| `AGC_JOB_ENGINE_FAIL_PROCEDURE_P` | `AGC_JOB_ENGINE_FAIL_PROCEED` | src/csm/sps_burn_clock.c | first-attempt name (opaque _P) |
| `agc_job_engine_fail_procedure_p` | `agc_job_engine_fail_proceed` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | first-attempt name |
| `AGC_JOB_V97E` | `AGC_JOB_ENGINE_FAIL_REIGNITE` | src/guidance/guid_pending.h | CM V97E: ENTER response -> V97ETASK: shut down, retrim (PRE40.6), V99 flash for re-ignition |
| `AGC_JOB_ENGINE_FAIL_PROCEDURE_E` | `AGC_JOB_ENGINE_FAIL_REIGNITE` | src/csm/sps_burn_clock.c | first-attempt name (opaque _E) |
| `agc_job_engine_fail_procedure_e` | `agc_job_engine_fail_reignite` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | first-attempt name |
| `AGC_JOB_V99T` | `AGC_JOB_ENGINE_ENABLE_TERMINATE` | src/guidance/guid_pending.h | CM V99T: TERMINATE response to V99: "KILL PRE40.6/CLOKTASK PROTECTION" -> V99TJOB |
| `AGC_JOB_ENGINE_ENABLE_REQUEST_T` | `AGC_JOB_ENGINE_ENABLE_TERMINATE` | src/csm/sps_burn_clock.c | first-attempt name (opaque _T) |
| `agc_job_engine_enable_request_t` | `agc_job_engine_enable_terminate` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | first-attempt name |
| `AGC_JOB_ENGINE_ENABLE_REQUEST_TJOB` | `AGC_JOB_ENGINE_ENABLE_TERMINATE_RESTART` | src/csm/guid_pending_csm.h | first-attempt name (pasted TJOB) |
| `AGC_JOB_V99P` | `AGC_JOB_ENGINE_ENABLE_PROCEED` | src/guidance/guid_pending.h | CM V99P: PROCEED: set ASTNFLAG, IGNITION at TIG |
| `AGC_JOB_ENGINE_ENABLE_REQUEST_P` | `AGC_JOB_ENGINE_ENABLE_PROCEED` | src/csm/sps_burn_clock.c | first-attempt name (opaque _P) |
| `agc_job_engine_enable_request_p` | `agc_job_engine_enable_proceed` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | first-attempt name |
| `AGC_JOB_V99E` | `AGC_JOB_ENGINE_ENABLE_BYPASS` | src/guidance/guid_pending.h | CM V99E: ENTER: "V16N85 POST-BURN OPERATIONS" (P40RCS: "P40 BYPASS SPS") |
| `AGC_JOB_ENGINE_ENABLE_REQUEST_E` | `AGC_JOB_ENGINE_ENABLE_BYPASS` | src/csm/sps_burn_clock.c | first-attempt name (opaque _E) |
| `agc_job_engine_enable_request_e` | `agc_job_engine_enable_bypass` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | first-attempt name |
| `AGC_JOB_ENGINE_ENABLE_REQUEST_EJOB` | `AGC_JOB_ENGINE_ENABLE_BYPASS_RESTART` | src/csm/guid_pending_csm.h | first-attempt name (pasted EJOB) |
| `engine_enable_request_end` | `engine_enable_finish` | src/csm/sps_burn_clock.c | common tail of V99E/V99T (TVCZAP, next job) |
| `AGC_TASK_V97TTASK` | `AGC_TASK_ENGINE_FAIL_SHUTDOWN` | src/guidance/guid_pending.h | CM V97TTASK: "DISABLE CLOCKJOB", 3MDOT mass, SPSOFF "SHUTDOWN SPS ENGINE" |
| `AGC_TASK_ENGINE_FAIL_PROCEDURE_TTASK` | `AGC_TASK_ENGINE_FAIL_SHUTDOWN` | src/csm/sps_burn_clock.c src/exec/exec_pending.h src/exec/restart_tables.c | first-attempt name (pasted TTASK) |
| `agc_task_engine_fail_procedure_ttask` | `agc_task_engine_fail_shutdown` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | first-attempt name |
| `AGC_TASK_ENGINE_FAIL_PROCEDURE_TTASK2` | `AGC_TASK_ENGINE_FAIL_SHUTDOWN_RESTART` | src/csm/guid_pending_csm.h src/csm/sps_burn_clock.c | first-attempt name (pasted TTASK2) |
| `agc_task_engine_fail_procedure_ttask2` | `agc_task_engine_fail_shutdown_restart` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | first-attempt name |
| `AGC_TASK_V97TRCS` | `AGC_TASK_ENGINE_FAIL_RCS_DAP` | src/guidance/guid_pending.h | CM V97TRCS: "RCS DAP IN 0.6SEC ... SET UP V99T FOR TVCZAP AND POST41" |
| `AGC_TASK_ENGINE_FAIL_PROCEDURE_TRCS` | `AGC_TASK_ENGINE_FAIL_RCS_DAP` | src/csm/sps_burn_clock.c src/exec/exec_pending.h src/exec/restart_tables.c | first-attempt name (pasted TRCS) |
| `agc_task_engine_fail_procedure_trcs` | `agc_task_engine_fail_rcs_dap` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | first-attempt name |
| `AGC_TASK_V97PTASK` | `AGC_TASK_ENGINE_FAIL_RESUME` | src/guidance/guid_pending.h | CM V97PTASK: "GET MASS UPDATES (TVCEXEC) GOING AGAIN", SET STEERSW "RE-ENABLE STEERING" |
| `AGC_TASK_ENGINE_FAIL_PROCEDURE_PTASK` | `AGC_TASK_ENGINE_FAIL_RESUME` | src/csm/sps_burn_clock.c src/exec/exec_pending.h src/exec/restart_tables.c | first-attempt name (pasted PTASK) |
| `agc_task_engine_fail_procedure_ptask` | `agc_task_engine_fail_resume` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | first-attempt name |
| `AGC_TASK_R40ENABL` | `AGC_TASK_ENGINE_FAIL_REENABLE` | src/guidance/guid_pending.h | CM R40ENABL: "RE-ENABLE R40 BY CLEARING IDLEFAIL" |
| `AGC_TASK_ENGINE_FAIL_ENABL` | `AGC_TASK_ENGINE_FAIL_REENABLE` | src/csm/sps_burn_clock.c src/exec/exec_pending.h src/exec/restart_tables.c | first-attempt name |
| `agc_task_engine_fail_enabl` | `agc_task_engine_fail_reenable` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | first-attempt name |
| `AGC_TASK_V97ETASK` | `AGC_TASK_ENGINE_FAIL_REIGNITE_PREP` | src/guidance/guid_pending.h | CM V97ETASK: "FORCE R1 OF V06N40 TO READ 59X59", mass, then SPSOFF97 |
| `AGC_TASK_ENGINE_FAIL_PROCEDURE_ETASK` | `AGC_TASK_ENGINE_FAIL_REIGNITE_PREP` | src/csm/sps_burn_clock.c src/exec/exec_pending.h src/exec/restart_tables.c | first-attempt name (pasted ETASK) |
| `agc_task_engine_fail_procedure_etask` | `agc_task_engine_fail_reignite_prep` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | first-attempt name |
| `AGC_TASK_V97E40_6` | `AGC_TASK_ENGINE_FAIL_REIGNITE_TRIM` | src/guidance/guid_pending.h | CM V97E40.6: "USE S40.6 RESTART ENTRY TO TRIM ENGINE", RCS DAP, QUICKIGN |
| `AGC_TASK_ENGINE_FAIL_PROCEDURE_E40_6` | `AGC_TASK_ENGINE_FAIL_REIGNITE_TRIM` | src/csm/sps_burn_clock.c src/exec/exec_pending.h src/exec/restart_tables.c | first-attempt name (pasted E40_6) |
| `agc_task_engine_fail_procedure_e40_6` | `agc_task_engine_fail_reignite_trim` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | first-attempt name |
| `AGC_TASK_V99FLASH` | `AGC_TASK_ENGINE_ENABLE_FLASH` | src/guidance/guid_pending.h | CM V99FLASH: "CAUSE V99 TO FLASH" |
| `AGC_TASK_ENGINE_ENABLE_REQUEST_FLASH` | `AGC_TASK_ENGINE_ENABLE_FLASH` | src/csm/sps_burn_clock.c | first-attempt name |
| `agc_task_engine_enable_request_flash` | `agc_task_engine_enable_flash` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | first-attempt name |
| `AGC_TASK_PRE40_6` | `AGC_TASK_GIMBAL_TRIM_RESTART` | src/csm/sps_burn_clock.c src/exec/exec_pending.h src/exec/restart_tables.c src/guidance/guid_pending.h | CM PRE40.6: "RESTART ENTRY TO RE-DO S40.6 ... TO GET ACTUATORS TRIMMED FOR IGNITION" [S-section] |
| `agc_task_pre40_6` | `agc_task_gimbal_trim_restart` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | CM PRE40.6 [S-section] |
| `v97vcntr` | `engine_fail_vcntr` | include/agc/agc_csm_dap.h src/csm/dap/tvc_exec.c src/csm/dap/tvc_init.c src/csm/sps_burn_clock.c src/csm/sps_burn_ignition.c | CM V97VCNTR: VCNTR saved for V97PTASK "GET MASS UPDATES (TVCEXEC) GOING AGAIN" (struct member) |
| `AGC_JOB_CLOK_V97` | `AGC_JOB_CLOKJOB_ENGINE_FAIL` | src/guidance/guid_pending.h | LM CLOKJOB -35: "INDICATE VERB 97 PASTE"; PROCEED -> COMFAIL1 |
| `agc_job_clok_engine_fail_procedure` | `agc_job_clokjob_engine_fail` | include/agc/agc_guid.h | first-attempt name |
| `AGC_JOB_CLOK_V97E` | `AGC_JOB_CLOKJOB_ENGINE_FAIL_SHUTDOWN` | src/guidance/guid_pending.h | LM COMFAIL2 (V97 ENTER): "KILL ZOOM", "COMMAND ENGINE OFF" (ENGINOF4) |
| `AGC_JOB_CLOK_V99` | `AGC_JOB_CLOKJOB_ENGINE_ENABLE` | src/guidance/guid_pending.h | LM CLOKJOB -13: "PLEASE ENABLE ENGINE" V99; *PROCEED: SET ASTNFLAG, IGNITE |
| `agc_job_clok_engine_enable_request` | `agc_job_clokjob_engine_enable` | include/agc/agc_guid.h | first-attempt name |
| `AGC_JOB_CLOK_V99E` | `AGC_JOB_CLOKJOB_ENGINE_ENABLE_BYPASS` | src/guidance/guid_pending.h | LM *ENTER on V99: INDEX WHICH -> GOPOST (POSTBURN, no ignition) |
| `AGC_JOB_CLOK_N61` | `AGC_JOB_CLOKJOB_EVENT_TIMER_PROCEED` | src/guidance/guid_pending.h | LM CLOKJOB -25 V06N61: "USED BY THE CREW IN P63 TO RESET HIS EVENT TIMER TO AGREE WITH TIG"; PROCEED -> ASTNRETN |
| `agc_job_clok_entry_target_proceed` | `agc_job_clokjob_event_timer_proceed` | include/agc/agc_guid.h | first-attempt name (wrong: LM N61 is not the entry target) |
| `agc_guid_s40_1` | `agc_guid_initial_velocity_to_gain` | include/agc/agc_guid.h src/csm/sps_burn.c src/guidance/p40.c src/lm/dps_burn.c docs:guid-powered | S40.1: "COMPUTE INITIAL THRUST DIRECTION(UT) AND INITIAL VALUE OF VG VECTOR(VGTIG)" [S-section] |
| `s40_1b_csm` | `initial_vg_lambert_csm` | src/guidance/p40.c | S40.1B: Lambert-targeted branch of S40.1 (CM) [S-section] |
| `s40_1b_lm` | `initial_vg_lambert_lm` | src/guidance/p40.c | S40.1B: Lambert-targeted branch of S40.1 (LM) [S-section] |
| `agc_guid_s40_23_csm` | `agc_guid_preferred_thrust_attitude_csm` | include/agc/agc_guid.h src/csm/sps_burn.c src/guidance/p40.c docs:guid-powered | S40.2,3: "COMPUTE PREFERRED ATTITUDE" / "POINTVSM ... DESIRED THRUST VECTOR IN SM COORDINATES" [S-section] |
| `agc_guid_s40_23_lm` | `agc_guid_preferred_thrust_attitude_lm` | include/agc/agc_guid.h src/guidance/p40.c src/lm/dps_burn.c docs:guid-powered | S40.2,3 (LM) [S-section] |
| `agc_guid_s41_1` | `agc_guid_reference_to_body` | include/agc/agc_guid.h src/csm/sps_burn_steer.c src/guidance/p40.c src/lm/dps_burn.c docs:guid-powered | S41.1: "COORDINATE SYSTEM TRANSFORMATION FROM THE REFERENCE FRAME TO THE BODY" [S-section] |
| `agc_csm_s40_8` | `agc_csm_cross_product_steering` | include/agc/agc_guid.h src/csm/sps_burn_steer.c docs:guid-powered | S40.8: "UPDATES VG ... COMPUTES TGO, AND CALLS FINDCDUW" (GSOP cross-product steering) [S-section] |
| `agc_lm_s40_8` | `agc_lm_cross_product_steering` | include/agc/agc_guid.h src/lm/dps_burn.c docs:guid-powered | S40.8 (LM) [S-section] |
| `agc_guid_s40_8_tgo_csm` | `agc_guid_steering_tgo_csm` | include/agc/agc_guid.h src/csm/sps_burn_steer.c src/guidance/p40.c | S40.8 TGO computation (CM) [S-section] |
| `agc_guid_s40_8_tgo_lm` | `agc_guid_steering_tgo_lm` | include/agc/agc_guid.h src/guidance/p40.c src/lm/dps_burn.c | S40.8 TGDCALC (LM) [S-section] |
| `agc_guid_s40_8_xproduct` | `agc_guid_steering_cross_product` | include/agc/agc_guid.h src/csm/sps_burn_steer.c src/guidance/p40.c docs:guid-powered | S40.8 cross-product rate command [S-section] |
| `s40_8_part2` | `steering_after_vg_update` | src/csm/sps_burn_steer.c | S40.8 after VG update (TGO, steering) (CM) [S-section] |
| `s40_8_steer` | `steering_commands` | src/lm/dps_burn.c | S40.8 steering-command tail (LM) [S-section] |
| `s40_81` | `short_burn_cutoff` | src/csm/sps_burn_steer.c | S40.81: "TGO LESS THAN 4 SECONDS ... FOR ENGINE-OFF CALL", RATEZRO "TVC TO ATTITUDE HOLD" [S-section] |
| `AGC_JOB_S40_8B` | `AGC_JOB_CROSS_PRODUCT_RESTART` | src/csm/guid_pending_csm.h src/csm/sps_burn_steer.c | S40.8 restart point after 5.3 type-B phase change (lane-invented _8B) [S-section] |
| `agc_job_s40_8b_csm` | `agc_job_cross_product_restart` | include/agc/agc_guid.h src/csm/sps_burn_steer.c | S40.8/UPDATEVG/SURELY.9 restart entries (CM) [S-section] |
| `AGC_JOB_UPDATEVG_B` | `AGC_JOB_UPDATEVG_RESTART` | src/csm/guid_pending_csm.h src/csm/sps_burn_steer.c | UPDATEVG restart point after 5.3 B (lane-invented _B) |
| `AGC_JOB_SURELY9_B` | `AGC_JOB_SURELY9_RESTART` | src/csm/guid_pending_csm.h src/csm/sps_burn_steer.c | SURELY.9 restart point after 2PHSCHNG (lane-invented _B) |
| `surely9_b` | `surely9_restart` | src/csm/sps_burn_steer.c | SURELY.9 continuation (lane-invented _b) |
| `AGC_JOB_STEERING_B` | `AGC_JOB_STEERING_RESTART` | src/csm/guid_pending_csm.h src/csm/sps_burn_steer.c | STEERING restart point "after ENGINOFF set" (lane-invented _B) |
| `agc_job_steering_b` | `agc_job_steering_restart` | include/agc/agc_guid.h src/csm/sps_burn_steer.c | STEERING restart point (lane-invented _b) |
| `AGC_JOB_S40_9` | `AGC_JOB_VTOGAIN` | src/csm/sps_burn_steer.c src/guidance/guid_pending.h src/lm/dps_burn.c | S40.9: "NAME S40.9 -- VTOGAIN (AIMPOINT MANEUVERS ONLY)" [S-section] |
| `agc_job_s40_9_csm` | `agc_job_vtogain_csm` | include/agc/agc_guid.h src/csm/sps_burn_steer.c docs:guid-powered | S40.9 VTOGAIN (CM) [S-section] |
| `agc_job_s40_9_lm` | `agc_job_vtogain_lm` | include/agc/agc_guid.h src/lm/dps_burn.c docs:guid-powered | S40.9 VTOGAIN (LM) [S-section] |
| `agc_csm_s40_9_body` | `agc_csm_vtogain_body` | src/csm/sps_burn_internal.h | S40.9 to COPY40.9 (body of VTOGAIN; "body" = code body) [S-section] |
| `s40_9_started` | `vtogain_started` | include/agc/agc_guid.h src/csm/sps_burn_steer.c | UPDATEVG out-param: S40.9 job started [S-section] |
| `s409` | `vtogain_started` | src/csm/sps_burn_steer.c | same out-param, local [S-section] |
| `AGC_JOB_S40_13` | `AGC_JOB_TIMEBURN` | src/csm/sps_burn.c src/csm/sps_burn_clock.c src/exec/exec_pending.h src/exec/restart_tables.c src/guidance/guid_pending.h src/lm/ignition.c | S40.13: "NAME: S40.13 -- TIMEBURN" [S-section] |
| `agc_job_s40_13_csm` | `agc_job_timeburn_csm` | include/agc/agc_guid.h src/csm/sps_burn_steer.c | S40.13 TIMEBURN (CM) [S-section] |
| `agc_job_s40_13_lm` | `agc_job_timeburn_lm` | include/agc/agc_guid.h src/lm/dps_burn.c | S40.13 TIMEBURN (LM) [S-section] |
| `agc_guid_s40_13_csm` | `agc_guid_timeburn_csm` | include/agc/agc_guid.h src/csm/sps_burn_steer.c src/guidance/p40.c docs:guid-powered | S40.13 TIMEBURN arithmetic (CM) [S-section] |
| `S40_135` | `TWO_JET_IMPULSE` | src/guidance/p40.c | CM S40.135: "IMPULSE FROM 7.96 SECS OF 2-JET FIRING" [S-section] |
| `S40_136` | `DPS_IMPULSE_THRUST` | src/lm/dps_burn.c | LM S40.136: ".4671 M NEWTONS (DPS)" [S-section] |
| `S40_136S` | `DPS_IMPULSE_THRUST_SHIFTED` | src/lm/dps_burn.c | LM S40.136_: "S40.136 SHIFTED LEFT 10" [S-section] |
| `AGC_TASK_S40_6` | `AGC_TASK_GIMBAL_TEST_TRIM` | src/csm/sps_burn.c src/guidance/guid_pending.h | S40.6: "GIMBAL DRIVE TEST AND/OR GIMBAL TRIM" [S-section] |
| `agc_task_s40_6` | `agc_task_gimbal_test_trim` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | S40.6 [S-section] |
| `AGC_TASK_S40_6_B` | `AGC_TASK_GIMBAL_TEST_RELAYS` | src/csm/sps_burn_clock.c src/guidance/guid_pending.h | S40.6 after 60 ms (relay set-up) (lane-invented _B) [S-section] |
| `agc_task_s40_6_relays` | `agc_task_gimbal_test_relays` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | S40.6 relay latching step [S-section] |
| `AGC_TASK_S40_6_C` | `AGC_TASK_GIMBAL_TEST_RSTRTST` | src/csm/sps_burn_clock.c src/guidance/guid_pending.h | RSTRTST: "CHECK FOR RESTART ENTRY (PRE40.6)" (lane-invented _C) [S-section] |
| `agc_task_s40_6_rstrtst` | `agc_task_gimbal_test_rstrtst` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | RSTRTST [S-section] |
| `AGC_TASK_S40_6_OUT` | `AGC_TASK_GIMBAL_TEST_STEP` | src/csm/sps_burn_clock.c src/guidance/guid_pending.h | S40.6 OUTPUT return / GIMTRIM [S-section] |
| `agc_task_s40_6_step` | `agc_task_gimbal_test_step` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | S40.6 OUTPUT return / GIMTRIM [S-section] |
| `s406step` | `gimbal_test_step` | include/agc/agc_guid.h src/csm/sps_burn_clock.c | S40.6 gimbal-drive-test step counter (struct member) [S-section] |
| `agc_csm_s11_1` | `agc_csm_velocity_altitude_rate` | include/agc/agc_csm_entry.h src/csm/earth_orbit_insertion.c src/csm/guid_pending_csm.h src/csm/sps_burn_steer.c | S11.1: "CALC. VI, H, HDOT FOR NOUN 62" [S-section] |
| `agc_job_external_delta_v_calc` | `agc_job_external_delta_v_compute` | include/agc/agc_guid.h | first-attempt header name of P30 after N81 PRO (S30.1, PARAM30) |
| `agc_job_external_delta_v_param` | `agc_job_external_delta_v_display_parameters` | include/agc/agc_guid.h | first-attempt header name of PARAM30 |
| `agc_job_external_delta_v_all_done` | `agc_job_external_delta_v_accept` | include/agc/agc_guid.h | first-attempt header name of LM PARAM30 PROCEED |
| `agc_job_external_delta_v_delta_v_lv` | `agc_job_external_delta_v_display_dv` | include/agc/agc_guid.h | first-attempt header name of CNTNUP30 (stutter delta_v_delta_v) |
| `agc_job_lambert_aimpoint_guidance_calc` | `agc_job_lambert_aimpoint_compute` | include/agc/agc_guid.h | first-attempt header name of P31 after N33 PRO |
| `agc_job_lambert_aimpoint_guidance_delta_v_lv` | `agc_job_lambert_aimpoint_display_dv` | include/agc/agc_guid.h | first-attempt header name of P31 CNTNUP30 |

## guid-entry -- P61-P67, S61/S62, P11

| old | new | files | AGC source / comment |
|---|---|---|---|
| `AGC_JOB_ENTRY_PREPARATION_DSP` | `AGC_JOB_ENTRY_PREPARATION_TARGET_DISPLAY` | src/csm/entry_pending.h src/csm/entry_programs.c | P61 display V06N61 "LAT(SPL) LNG(SPL) HEADSUP" (splash target) |
| `agc_job_entry_preparation_dsp` | `agc_job_entry_preparation_target_display` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | P61 V06N61 LAT(SPL) LNG(SPL) HEADSUP |
| `AGC_JOB_ENTRY_PREPARATION_3` | `AGC_JOB_ENTRY_DISPLAY_PROTECT` | src/csm/entry_pending.h | P61.3: "EST. GRP 4 FOR DISPLAY AND DO ENDOFJOB" (shared by P61, P62.2, P65.1) |
| `agc_job_entry_preparation_3` | `agc_job_entry_display_protect` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | P61.3 PHASCHNG group 4 at the display, ENDOFJOB |
| `AGC_JOB_ENTRY_PREPARATION_4` | `AGC_JOB_ENTRY_PREPARATION_COMPUTE` | src/csm/entry_pending.h src/csm/entry_programs.c | P61.4: ROLLC from HEADSUP, NEWRNVN target, "S61.2 GET DISPLAY DATA FOR N60 AND N63" |
| `agc_job_entry_preparation_4` | `agc_job_entry_preparation_compute` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | P61.4 |
| `AGC_JOB_ENTRY_PREPARATION_1` | `AGC_JOB_ENTRY_PREPARATION_PREDICTION_DISPLAY` | src/csm/entry_pending.h src/csm/entry_programs.c | P61.1: V06N60 GMAX VPRED GAMMAEI |
| `agc_job_entry_preparation_1` | `agc_job_entry_preparation_prediction_display` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | P61.1 V06N60 |
| `AGC_JOB_ENTRY_PREPARATION_2` | `AGC_JOB_ENTRY_PREPARATION_EMS_DISPLAY` | src/csm/entry_pending.h src/csm/entry_programs.c | P61.2: "CORRECT TTE FOR TIME LAPSE", V06N63 RTGO VIO TTE (EMS initialization) |
| `agc_job_entry_preparation_2` | `agc_job_entry_preparation_ems_display` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | P61.2 V06N63 |
| `AGC_JOB_CM_SM_SEPARATION_S61RET` | `AGC_JOB_CM_SM_SEPARATION_AFTER_CHECK` | src/csm/entry_pending.h src/csm/entry_programs.c | P62 after S61.1 "CHECK STATE VECTOR AND IMU ORIENTATION"; POSEXIT=P62.3, CM/DAPIC |
| `agc_job_cm_sm_separation_s61ret` | `agc_job_cm_sm_separation_after_check` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | P62 after S61.1 |
| `AGC_JOB_CM_SM_SEPARATION_2` | `AGC_JOB_CM_SM_SEPARATION_REQUEST` | src/csm/entry_dap.c src/csm/entry_pending.h src/csm/entry_programs.c | P62.2: AVEGEXIT=CM/POSE, GOPERF1R 41 (request CM/SM separation) |
| `agc_job_cm_sm_separation_2` | `agc_job_cm_sm_separation_request` | include/agc/agc_csm_entry.h src/csm/entry_dap.c src/csm/entry_programs.c | P62.2 |
| `AGC_JOB_CM_SM_SEPARATION_2B` | `AGC_JOB_CM_SM_SEPARATION_PROCEED` | src/csm/entry_pending.h src/csm/entry_programs.c | P62.2 +3 "PROCEED": CM/DAPON |
| `agc_job_cm_sm_separation_2b` | `agc_job_cm_sm_separation_proceed` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | P62.2 PROCEED |
| `AGC_JOB_CM_SM_SEPARATION_1` | `AGC_JOB_CM_SM_SEPARATION_TARGET_DISPLAY` | src/csm/entry_pending.h src/csm/entry_programs.c | P62.1: "CA V06N61 LAT(SPL) LNG(SPL) HEADSUP" |
| `agc_job_cm_sm_separation_1` | `agc_job_cm_sm_separation_target_display` | include/agc/agc_csm_entry.h src/csm/entry_dap.c src/csm/entry_programs.c | P62.1 V06N61 |
| `AGC_JOB_CM_SM_SEPARATION_1B` | `AGC_JOB_CM_SM_SEPARATION_END_HOLD` | src/csm/entry_pending.h src/csm/entry_programs.c | P62.1 PROCEED: "TERMINATE ATTITUDE HOLD. SET UP COMMANDS", N22 display |
| `agc_job_cm_sm_separation_1b` | `agc_job_cm_sm_separation_end_hold` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | P62.1 PROCEED |
| `AGC_ENTRY_POSEXIT_CM_SM_SEPARATION_3` | `AGC_ENTRY_POSEXIT_DESIRED_GIMBALS` | include/agc/agc_csm_entry.h src/csm/cm_body_attitude.c src/csm/entry_lexicon.c src/csm/entry_programs.c | first-attempt name (opaque _3) |
| `entry_initialization_flag` | `wake_separation_interlock` | include/agc/agc_csm_entry.h src/csm/entry_dap.c src/csm/entry_programs.c | first-attempt name (was P63FLAG) |
| `AGC_JOB_ENTRY_UPCONTROL_1` | `AGC_JOB_ENTRY_UPCONTROL_DISPLAY` | src/csm/entry_programs.c src/exec/exec_pending.h src/exec/restart_tables.c | P65.1: entry display off, V16N69 ROLLC DL VL, GOFLASHR |
| `agc_job_entry_upcontrol_1` | `agc_job_entry_upcontrol_display` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | P65.1 V16N69 |
| `AGC_JOB_ENTRY_UPCONTROL_1B` | `AGC_JOB_ENTRY_UPCONTROL_PROCEED` | src/csm/entry_pending.h src/csm/entry_programs.c | P65.1 "IF PROCEED, CONTINUE": "DISABLE GRP 4, START UP ENTRY DISPLAY" |
| `agc_job_entry_upcontrol_1b` | `agc_job_entry_upcontrol_proceed` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | P65.1 PROCEED |
| `AGC_JOB_ENTRY_FINAL_PHASE_1` | `AGC_JOB_ENTRY_FINAL_PHASE_DISPLAY` | src/csm/entry_programs.c src/csm/reentry.c src/exec/exec_pending.h src/exec/restart_tables.c | P67.1: "CA V16N67 RTOGO LAT LONG" |
| `agc_job_entry_final_phase_1` | `agc_job_entry_final_phase_display` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | P67.1 V16N67 |
| `AGC_JOB_ENTRY_FINAL_PHASE_1B` | `AGC_JOB_ENTRY_FINAL_PHASE_END` | src/csm/entry_pending.h src/csm/entry_programs.c | P67.1 end: entry DAP off (CM/DSTBY), AVEGEXIT=SERVEXIT, GOTOPOOH |
| `agc_job_entry_final_phase_1b` | `agc_job_entry_final_phase_end` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | P67.1 end |
| `agc_entry_final_phase_2_latlong` | `agc_entry_final_phase_latlong` | include/agc/agc_csm_entry.h src/csm/entry_programs.c src/csm/reentry.c | P67.2: "CALC PRESENT LAT, LONG, ALT" (NAMING.md example name) |
| `AGC_ENTRY_GOTO_ENTRY_FINAL_PHASE_2` | `AGC_ENTRY_GOTO_FINAL_PHASE_LATLONG` | include/agc/agc_csm_entry.h src/csm/reentry.c | GOTO P67.2 after VQUIT: lat/long only (stutter removed) |
| `ST_ENTRY_FINAL_PHASE_2` | `ST_FINAL_PHASE_LATLONG` | src/csm/reentry.c | reentry step P67.2 (lane-local enum) |
| `step_entry_final_phase_2` | `step_final_phase_latlong` | src/csm/reentry.c | reentry step function for P67.2 |
| `AGC_DATA_RANGE_TO_GO_ENTRY_POST_0_05G` | `AGC_DATA_RANGE_TO_SPLASH` | include/agc/agc_nouns.h src/io/noun_tables_csm.c | first-attempt name (7 words) |
| `AGC_DATA_RANGE_TO_GO_ENTRY_FINAL_PHASE` | `AGC_DATA_RANGE_TO_TARGET` | include/agc/agc_nouns.h src/io/noun_tables_csm.c | first-attempt name |
| `range_to_go_entry_final_phase` | `range_to_target` | include/agc/agc_csm_entry.h src/csm/reentry.c | first-attempt name |
| `agc_job_s61_1` | `agc_job_entry_imu_check` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | S61.1 "CHECK STATE VECTOR AND IMU ORIENTATION" (P62 comment); AVERAGEG on? IMU Y within 30 deg [S-section] |
| `agc_job_s61_1a` | `agc_job_entry_imu_axis_test` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | S61.1A: "SEE IF IMU Y AXIS IS WITHIN 30 DEG OF VAR", alarms 01426/01427 [S-section] |
| `AGC_JOB_S61_1A` | `AGC_JOB_ENTRY_IMU_AXIS_TEST` | src/csm/entry_programs.c src/exec/exec_pending.h src/exec/restart_tables.c | S61.1A restart 4.45 [S-section] |
| `agc_job_s61_1_mid` | `agc_job_entry_imu_check_extrapolated` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | S61.1 after CALRB MIDTOAV2: "UPDATE THE STATE VECTOR TO PRESENT TIME + TOLERANCE" [S-section] |
| `AGC_JOB_S61_1_MID` | `AGC_JOB_ENTRY_IMU_CHECK_EXTRAPOLATED` | src/csm/entry_pending.h src/csm/entry_programs.c | same job id [S-section] |
| `agc_job_s61_1_ret` | `agc_job_entry_imu_check_return` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | RETRN1 after "WAIT 10 SEC AND THEN EXIT SUBROUTINE S61.1" [S-section] |
| `AGC_JOB_S61_1_RET` | `AGC_JOB_ENTRY_IMU_CHECK_RETURN` | src/csm/entry_pending.h src/csm/entry_programs.c | same job id [S-section] |
| `agc_task_s61_1c` | `agc_task_entry_start_average_g` | include/agc/agc_csm_entry.h src/csm/entry_programs.c | S61.1C: "TURN ON AVERAGEG AT THAT TIME" (restart 4.43) [S-section] |
| `AGC_TASK_S61_1C` | `AGC_TASK_ENTRY_START_AVERAGE_G` | src/csm/entry_programs.c src/exec/exec_pending.h src/exec/restart_tables.c | same task id [S-section] |
| `s61_ret_job` | `imu_check_return_job` | src/csm/entry_programs.c | 60GENRET return point of S61.1 [S-section] |
| `s61_return` | `imu_check_return` | src/csm/entry_programs.c | RETRN1 / 60GENRET [S-section] |
| `agc_entry_s61_2` | `agc_entry_ems_data` | include/agc/agc_csm_entry.h src/csm/entry_programs.c docs:guid-entry | S61.2 "GET DISPLAY DATA FOR N60 AND N63" (EMS initialization) [S-section] |
| `s61_2_ems` | `ems_altitude_range` | src/csm/entry_programs.c | S61.2 first part: TTE to EMSALT and RTGO from EMSALT [S-section] |
| `s61_tfftrig` | `tfftrig` | src/csm/entry_programs.c | TFF/TRIG "(USED BY S61.2)" word label [S-section] |
| `s61_fishcalc` | `fishcalc` | src/csm/entry_programs.c | FISHCALC "(USED BY S61.2)" word label [S-section] |
| `s61_vgamcalc` | `vgamcalc` | src/csm/entry_programs.c | VGAMCALC "(USED BY S61.2)" word label [S-section] |
| `s61_disptarg` | `disptarg` | src/csm/entry_programs.c | DISPTARG word label (S61.2) [S-section] |
| `agc_entry_s62_3` | `agc_entry_desired_gimbal_angles` | include/agc/agc_csm_entry.h src/csm/cm_body_attitude.c src/csm/entry_programs.c src/csm/reentry.c docs:guid-entry | S62.3: "COMPUTE DESIRED GIMBAL ANGLES FOR ENTRY ATTITUDE" [S-section] |
| `AGC_JOB_EARTH_ORBIT_INSERTION_MONITOR_TEPH` | `AGC_JOB_EARTH_ORBIT_INSERTION_MONITOR_PREREAD` | src/csm/earth_orbit_insertion.c src/csm/entry_pending.h | P11 after 05023 PHASCHNG: TEPHEM = TEPHEM1, PREREAD1 (opaque TEPH) |
| `agc_job_earth_orbit_insertion_monitor_teph` | `agc_job_earth_orbit_insertion_monitor_preread` | include/agc/agc_csm_entry.h src/csm/earth_orbit_insertion.c | same |
| `AGC_JOB_EARTH_ORBIT_INSERTION_MONITOR_ATER` | `AGC_JOB_EARTH_ORBIT_INSERTION_MONITOR_SETUP` | src/csm/earth_orbit_insertion.c src/csm/entry_pending.h | P11 after PREREAD1: ATERTASK in .5 s, MM 11, NODOP01, downlist, EARTHR -> MATRXJOB |
| `agc_job_earth_orbit_insertion_monitor_ater` | `agc_job_earth_orbit_insertion_monitor_setup` | include/agc/agc_csm_entry.h src/csm/earth_orbit_insertion.c | same |
| `AGC_JOB_RESTART_EARTH_ORBIT_INSERTION_MONITOR` | `AGC_JOB_EARTH_ORBIT_INSERTION_MONITOR_RESTART` | src/exec/exec_pending.h src/exec/restart_tables.c | first-attempt name |
| `agc_job_restart_earth_orbit_insertion_monitor` | `agc_job_earth_orbit_insertion_monitor_restart` | include/agc/agc_csm_entry.h src/csm/earth_orbit_insertion.c | first-attempt name |
| `AGC_JOB_REP11A` | `AGC_JOB_EARTH_ORBIT_INSERTION_MONITOR_EPHEMERIS` | src/exec/exec_pending.h src/exec/restart_tables.c | REP11A: "TEPHEM1 = TEPHEM + TLIFTOFF" (CORRECT FOR OVERFLOW), restart 3.13 |
| `agc_job_restart_earth_orbit_insertion_monitor_a` | `agc_job_earth_orbit_insertion_monitor_ephemeris` | include/agc/agc_csm_entry.h src/csm/earth_orbit_insertion.c | first-attempt name (opaque _a) |
| `AGC_JOB_P11SC` | `AGC_JOB_VHHDOT` | src/guidance/guid_pending.h | P11SCADR "2CADR VHHDOT" (AVGEXIT in P11); duplicate of entry_pending.h AGC_JOB_VHHDOT |
| `AGC_JOB_ATRESET2` | `AGC_JOB_ATRESET_AFTER_DELAY` | src/csm/earth_orbit_insertion.c src/csm/entry_pending.h docs:guid-entry | ATRESET +5 after DELAYJOB 6 (lane-invented "2") |
| `agc_job_atreset2` | `agc_job_atreset_after_delay` | include/agc/agc_csm_entry.h src/csm/earth_orbit_insertion.c | ATRESET +5 after DELAYJOB |
| `agc_earth_orbit_insertion_monitor_s11_1` | `agc_earth_orbit_insertion_monitor_readout` | include/agc/agc_csm_entry.h src/csm/earth_orbit_insertion.c | reader of the S11.1 VI/HDOT/H values for N62 (noun tables) |
| `agc_csm_prelaunch_earth_orbit_insertion_monitor_earthr` | `agc_csm_prelaunch_earthr_at_liftoff` | src/csm/earth_orbit_insertion.c src/csm/entry_pending.h docs:guid-entry | EARTHR called from P11 with TLIFTOFF (pending decl.) |
| `EARTH_ORBIT_INSERTION_MONITOR_RPAD` | `INSERTION_RPAD` | src/csm/earth_orbit_insertion.c | first-attempt name (stem + label) |
| `EARTH_ORBIT_INSERTION_MONITOR_MERTHRAT` | `INSERTION_MERTHRAT` | src/csm/earth_orbit_insertion.c | first-attempt name |
| `EARTH_ORBIT_INSERTION_MONITOR_HALFREV` | `INSERTION_HALFREV` | src/csm/earth_orbit_insertion.c | first-attempt name |
| `EARTH_ORBIT_INSERTION_MONITOR_POWDNCOD` | `INSERTION_POWDNCOD` | src/csm/earth_orbit_insertion.c | first-attempt name |
| `EARTH_ORBIT_INSERTION_MONITOR_2SECS` | `INSERTION_2SECS` | src/csm/earth_orbit_insertion.c | first-attempt name |
| `EARTH_ORBIT_INSERTION_MONITOR_ATER_DT` | `INSERTION_ATERTASK_DELAY` | src/csm/earth_orbit_insertion.c | first-attempt name |
| `EARTH_ORBIT_INSERTION_MONITOR_100MST5` | `INSERTION_100MST5` | src/csm/earth_orbit_insertion.c | first-attempt name |
| `EARTH_ORBIT_INSERTION_MONITOR_STIKBITS` | `INSERTION_STIKBITS` | src/csm/earth_orbit_insertion.c | first-attempt name |
| `earth_orbit_insertion_monitor_pad` | `insertion_pad` | src/csm/earth_orbit_insertion.c | first-attempt name |
| `earth_orbit_insertion_monitor_desired` | `desired_attitude` | src/csm/earth_orbit_insertion.c | first-attempt name |
| `earth_orbit_insertion_monitor_getdown` | `getdown` | src/csm/earth_orbit_insertion.c | first-attempt name |
| `earth_orbit_insertion_monitor_launch_refsmmat` | `launch_refsmmat` | src/csm/earth_orbit_insertion.c | first-attempt name |
| `earth_orbit_insertion_monitor_matrx_tail` | `matrx_tail` | src/csm/earth_orbit_insertion.c | first-attempt name |
| `vmagi_earth_orbit_insertion_monitor` | `insertion_vmagi` | src/csm/earth_orbit_insertion.c | first-attempt name |
| `hdot_earth_orbit_insertion_monitor` | `insertion_hdot` | src/csm/earth_orbit_insertion.c | first-attempt name |
| `alti_earth_orbit_insertion_monitor` | `insertion_alti` | src/csm/earth_orbit_insertion.c | first-attempt name |

## p37 -- P37

| old | new | files | AGC source / comment |
|---|---|---|---|
| `agc_job_p37` | `agc_job_return_to_earth` | src/csm/p37.c | P37: "RETURN TO EARTH" (canonical stem) |
| `agc_p37_xt1lim` | `agc_return_to_earth_xt1lim` | src/csm/p37.c | XT1LIM word label (P37 conic limit); first-attempt name kept |
| `P37_ONE_B11` | `RETURN_TO_EARTH_ONE_B11` | src/csm/p37.c | 1RTEB25 +1 = 1.0 B-11 constant (file-local) |

## landing -- P12, P63-P68, P70/P71, R10/R11, R12, R29

> Phase-2 note: this guid-landing section was generated in phase 1 but dropped from the reviewed
> copy by a lane-key typo in the generator (42 rows). It was restored and applied in phase 2.

| old | new | files | AGC source / comment |
|---|---|---|---|
| `AGC_JOB_BRAKING_PHASE_LM` | `AGC_JOB_BRAKING_PHASE` | src/lm/landing_pending.h src/lm/lunar_landing.c | first-attempt name (LM suffix redundant) |
| `agc_lm_job_braking_phase_lm` | `agc_lm_job_braking_phase` | include/agc/agc_lm_landing.h src/lm/lunar_landing.c docs:guid-landing | first-attempt name |
| `AGC_JOB_BRAKING_PHASE_SPOT2` | `AGC_JOB_BRAKING_PHASE_BURN_ATTITUDE` | src/lm/landing_pending.h src/lm/lunar_landing.c | first-attempt name (pasted SPOT2) |
| `agc_lm_job_braking_phase_spot2` | `agc_lm_job_braking_phase_burn_attitude` | include/agc/agc_lm_landing.h src/lm/lunar_landing.c docs:guid-landing | first-attempt name |
| `AGC_JOB_BRAKING_PHASE_SPOT3` | `AGC_JOB_BRAKING_PHASE_ANTENNA_CHECK` | src/lm/landing_pending.h src/lm/lunar_landing.c | first-attempt name |
| `agc_lm_job_braking_phase_spot3` | `agc_lm_job_braking_phase_antenna_check` | include/agc/agc_lm_landing.h src/lm/lunar_landing.c docs:guid-landing | first-attempt name |
| `AGC_JOB_BRAKING_PHASE_SPOT4` | `AGC_JOB_BRAKING_PHASE_RADAR_INIT` | src/lm/landing_pending.h src/lm/lunar_landing.c | first-attempt name |
| `agc_lm_job_braking_phase_spot4` | `agc_lm_job_braking_phase_radar_init` | include/agc/agc_lm_landing.h src/lm/lunar_landing.c | first-attempt name |
| `AGC_JOB_P63DISPS` | `AGC_JOB_BRAKING_PHASE_DISPLAY` | src/guidance/guid_pending.h | P63DISPS -2: "CAF V06N63 ... REGODSPR" |
| `agc_lm_job_braking_phase_disps_m2` | `agc_lm_job_braking_phase_display` | include/agc/agc_lm_landing.h src/lm/landing_guidance.c docs:guid-landing | first-attempt name (opaque m2) |
| `AGC_JOB_APPROACH_PHASE_DISPS` | `AGC_JOB_APPROACH_PHASE_DISPLAY` | src/lm/landing_guidance.c src/lm/landing_pending.h | first-attempt name |
| `agc_lm_job_approach_phase_disps` | `agc_lm_job_approach_phase_display` | include/agc/agc_lm_landing.h src/lm/landing_guidance.c docs:guid-landing | first-attempt name |
| `AGC_JOB_APPROACH_PHASE_CEED` | `AGC_JOB_APPROACH_PHASE_PROCEED` | src/lm/landing_guidance.c src/lm/landing_pending.h | first-attempt name (pasted CEED) |
| `agc_lm_job_approach_phase_ceed` | `agc_lm_job_approach_phase_proceed` | include/agc/agc_lm_landing.h src/lm/landing_guidance.c docs:guid-landing | first-attempt name |
| `agc_lm_velocity_nulling_start` | `agc_lm_start_velocity_nulling` | include/agc/agc_lm_landing.h src/lm/landing_guidance.c docs:guid-landing | first-attempt name |
| `ll_start_rate_of_descent_landing_a` | `ll_rate_of_descent_setup` | src/lm/lunar_landing.c | first-attempt name (opaque _a) |
| `agc_lm_rate_of_descent_landing_verta` | `agc_lm_rate_of_descent_vertical` | include/agc/agc_lm_landing.h src/lm/landing_guidance.c src/lm/lunar_landing.c docs:guid-landing | first-attempt name (pasted VERTA) |
| `agc_lm_manual_landing_vert` | `agc_lm_manual_landing_vertical` | include/agc/agc_lm_landing.h src/lm/landing_guidance.c src/lm/lunar_landing.c docs:guid-landing | first-attempt name |
| `agc_lm_job_landing_confirmation_b` | `agc_lm_job_landing_confirmation_proceed` | include/agc/agc_lm_landing.h src/lm/lunar_landing.c docs:guid-landing | first-attempt name (opaque _b) |
| `agc_lm_job_dps_abort_a` | `agc_lm_job_dps_abort_commanded` | include/agc/agc_lm_landing.h src/lm/landing_aborts.c | first-attempt name (opaque _a) |
| `agc_lm_job_aps_abort_a` | `agc_lm_job_aps_abort_commanded` | include/agc/agc_lm_landing.h src/lm/landing_aborts.c | first-attempt name (opaque _a) |
| `AGC_JOB_APS_ABORT_OLD` | `AGC_JOB_APS_ABORT_OLDTIME` | src/lm/landing_aborts.c src/lm/landing_pending.h | first-attempt name |
| `agc_lm_job_aps_abort_old` | `agc_lm_job_aps_abort_oldtime` | include/agc/agc_lm_landing.h src/lm/landing_aborts.c docs:guid-landing | first-attempt name |
| `AGC_JOB_ABTP40A` | `AGC_JOB_ABORT_AUTO_CONTROL_CHECK` | src/lm/landing_aborts.c src/lm/landing_pending.h | UPTHROT1 -3: "P40AUTO" (check PGNCS/auto modes) during abort |
| `agc_lm_job_abort_dps_burn_a` | `agc_lm_job_abort_auto_control_check` | include/agc/agc_lm_landing.h src/lm/landing_aborts.c docs:guid-landing | first-attempt name (wrong: not a DPS burn) |
| `ab_aps_abort_ret` | `ab_aps_abort_return` | src/lm/landing_aborts.c docs:guid-landing | first-attempt name |
| `AGC_TASK_R10R11` | `AGC_TASK_ABORT_MONITOR` | src/guidance/guid_pending.h | R10,R11: R11 "IS AN ABORT (STAGE) COMMANDED?" then R10 LANDISP |
| `AGC_TASK_LANDING_RADAR_MONITOR` | `AGC_TASK_ABORT_MONITOR` | src/lm/servicer_lm.c | first-attempt name (R10/R11 is not a radar monitor) |
| `agc_lm_task_landing_radar_monitor` | `agc_lm_task_abort_monitor` | include/agc/agc_lm_landing.h src/lm/landing_aborts.c docs:guid-landing | first-attempt name |
| `agc_lm_landing_radar_monitor_a` | `agc_lm_abort_monitor_resume` | include/agc/agc_lm_landing.h src/lm/guid_pending_lm.h src/lm/landing_aborts.c src/lm/servicer_lm_lr.c docs:guid-landing | first-attempt name (opaque _a) |
| `LANDING_RADAR_MONITOR_FIRST_CS` | `ABORT_MONITOR_FIRST_CS` | src/lm/servicer_lm.c | first-attempt name |
| `landing_radar_monitor_setup` | `abort_monitor_setup` | src/lm/servicer_lm.c | first-attempt name |
| `AGC_JOB_POWERED_ASCENT_LM` | `AGC_JOB_POWERED_ASCENT` | src/lm/ascent.c src/lm/landing_pending.h | first-attempt name (LM suffix redundant) |
| `agc_lm_job_powered_ascent_lm` | `agc_lm_job_powered_ascent` | include/agc/agc_lm_landing.h src/lm/ascent.c docs:guid-landing | first-attempt name |
| `AGC_JOB_POWERED_ASCENT_LM2` | `AGC_JOB_POWERED_ASCENT_PROCEED` | src/lm/ascent.c src/lm/landing_pending.h | first-attempt name (pasted LM2) |
| `agc_lm_job_powered_ascent_lm2` | `agc_lm_job_powered_ascent_proceed` | include/agc/agc_lm_landing.h src/lm/ascent.c | first-attempt name |
| `AGC_JOB_POWERED_ASCENT_LMB` | `AGC_JOB_POWERED_ASCENT_AT_IGNITION` | src/lm/ascent.c src/lm/landing_pending.h | first-attempt name (pasted LMB) |
| `agc_lm_job_powered_ascent_lmb` | `agc_lm_job_powered_ascent_at_ignition` | include/agc/agc_lm_landing.h src/lm/ascent.c | first-attempt name |
| `agc_lm_powered_ascent_ret` | `agc_lm_powered_ascent_return` | include/agc/agc_lm_landing.h src/lm/ascent.c docs:guid-landing | first-attempt name |
| `agc_radar_landing_downlink_lites` | `agc_radar_landing_radar_lights` | src/lm/guid_pending_lm.h src/lm/servicer_lm_lr.c | first-attempt name (not downlink) |
| `agc_radar_powered_flight_radar_designate` | `agc_radar_powered_flight_designate` | src/lm/guid_pending_lm.h src/lm/servicer_lm_lr.c | first-attempt name (stutter radar_radar) |
| `no_powered_flight_radar_designate_now` | `no_powered_designate_now` | src/lm/servicer_lm_lr.c | first-attempt name |

## exec -- exec-sched: V37/P00, restart-table targets

| old | new | files | AGC source / comment |
|---|---|---|---|
| `AGC_JOB_GOP00FIX` | `AGC_JOB_GO_IDLE_FIX` | include/agc/agc_jobs.h src/exec/dispatch.c src/exec/fresh_start.c src/exec/restart_tables.c tests/test_exec.c docs:exec-sched | GOP00FIX: after a restart into P00 "CAF V37N99 GOFLASH" (flash V37, request a program); P00 -> idle |
| `AGC_JOB_GOP00FIX_REFLASH` | `AGC_JOB_GO_IDLE_FIX_REFLASH` | include/agc/agc_jobs.h src/exec/change_program_idle.c src/exec/dispatch.c docs:exec-sched | GOP00FIX "TCF -3": re-flash V37 after any response |
| `agc_job_change_program_check` | `agc_job_change_program_continue` | include/agc/agc_exec.h src/exec/change_program_idle.c docs:exec-sched | first-attempt name (CANV37 is not a check) |
| `agc_exec_canv37` | `agc_exec_change_program_continue` | src/guidance/guid_pending.h | CANV37 (guid_pending.h decl) |
| `agc_exec_change_program_check` | `agc_exec_change_program_continue` | src/csm/servicer_csm.c | first-attempt name |
| `VP_NOUN_ENABLE_REQUEST` | `VP_GO_IDLE_NOUN` | src/exec/change_program_idle.c | first-attempt name (wrong meaning) |
| `flash_change_program_enable_request` | `flash_program_request` | src/exec/change_program_idle.c | first-attempt name (wrong meaning) |
| `AGC_JOB_PIKUP20` | `AGC_JOB_RENDEZVOUS_NAVIGATION_PICKUP` | src/exec/exec_pending.h src/exec/restart_tables.c | CM PIKUP20 (restart 1.11): "ALLOW HIGHER PRIO THAN LAMBERT; IS TRACK FLAG SET" |
| `AGC_JOB_RENDEZVOUS_TRACKING_DATA_RSTRT` | `AGC_JOB_RENDEZVOUS_TRACKING_DATA_RESTART` | src/exec/exec_pending.h src/exec/restart_tables.c | first-attempt name (RSTRT spelled out) |
| `AGC_JOB_ALTERNATE_LOS_SIGHTING_MARK_ENTER` | `AGC_JOB_ALTERNATE_LOS_ENTER` | src/exec/exec_pending.h src/exec/restart_tables.c | first-attempt name (5 words) |
| `AGC_JOB_PREFERRED_TRACKING_ATTITUDE_LEM1` | `AGC_JOB_PREFERRED_TRACKING_ATTITUDE_LOOP` | src/exec/exec_pending.h src/exec/restart_tables.c | first-attempt name (pasted LEM1) |
| `AGC_TASK_RENDEZVOUS_NAVIGATION_LEMC1` | `AGC_TASK_RENDEZVOUS_NAVIGATION_CYCLE` | src/exec/exec_pending.h src/exec/restart_tables.c | first-attempt name (pasted LEMC1) |
| `AGC_JOB_DPS_ABORT_A` | `AGC_JOB_DPS_ABORT_COMMANDED` | src/exec/exec_pending.h src/exec/restart_tables.c | first-attempt name (opaque _A) |
| `AGC_JOB_APS_ABORT_A` | `AGC_JOB_APS_ABORT_COMMANDED` | src/exec/exec_pending.h src/exec/restart_tables.c | first-attempt name (opaque _A) |
| `AGC_TASK_LANDING_RADAR_MONITOR_RESTART` | `AGC_TASK_ABORT_MONITOR_RESTART` | src/exec/exec_pending.h src/exec/restart_tables.c src/lm/landing_aborts.c | first-attempt name (R10/R11 are not a radar monitor) |

## imu -- imu: R02, R60/R62, 1/GYRO

| old | new | files | AGC source / comment |
|---|---|---|---|
| `agc_imu_r02both` | `agc_imu_status_check` | src/csm/s_band_antenna_csm.c src/lm/s_band_antenna_lm.c src/programs/imu_realign.c | R02BOTH "IMU STATUS CHECK" (BOTH = both vehicles; dropped) |
| `agc_imu_status_check_both` | `agc_imu_status_check` | include/agc/agc_imu.h src/csm/entry_programs.c src/csm/sps_burn.c src/csm/sps_burn_ignition.c src/imu/imu_mode.c src/lm/ascent.c src/lm/dps_burn.c src/lm/lunar_landing.c docs:guid-landing,imu | first-attempt name |
| `agc_imu_status_check_zero` | `agc_imu_set_in_use` | include/agc/agc_imu.h src/imu/imu_mode.c | first-attempt name (opaque ZERO) |
| `AGC_JOB_ATTITUDE_MANEUVER_ENDMANUV` | `AGC_JOB_ENDMANUV` | src/imu/attitude_maneuver.c src/imu/imu_pending.h docs:imu | first-attempt name (stutter) |
| `agc_job_attitude_maneuver_endmanuv` | `agc_job_endmanuv` | include/agc/agc_imu.h src/imu/attitude_maneuver.c docs:imu | first-attempt name |
| `AGC_JOB_ATTITUDE_MANEUVER_REDOMANC` | `AGC_JOB_REDOMANC` | src/imu/attitude_maneuver.c src/imu/imu_pending.h docs:imu | first-attempt name (stutter) |
| `agc_job_attitude_maneuver_redomanc` | `agc_job_redomanc` | include/agc/agc_imu.h src/imu/attitude_maneuver.c docs:imu | first-attempt name |
| `AGC_JOB_ATTITUDE_MANEUVER_TERM` | `AGC_JOB_ATTITUDE_MANEUVER_TERMINATE` | src/imu/attitude_maneuver.c src/imu/imu_pending.h docs:imu | first-attempt name |
| `agc_job_attitude_maneuver_term` | `agc_job_attitude_maneuver_terminate` | include/agc/agc_imu.h src/imu/attitude_maneuver.c docs:imu | first-attempt name |
| `AGC_JOB_CREW_DEFINED_MANEUVER_DISP` | `AGC_JOB_CREW_DEFINED_MANEUVER_DISPLAY` | src/imu/attitude_maneuver.c src/imu/imu_pending.h docs:imu | first-attempt name |
| `agc_job_crew_defined_maneuver_disp` | `agc_job_crew_defined_maneuver_display` | include/agc/agc_imu.h src/imu/attitude_maneuver.c docs:imu | first-attempt name |
| `AGC_JOB_CREW_DEFINED_MANEUVER_TERM` | `AGC_JOB_CREW_DEFINED_MANEUVER_TERMINATE` | src/imu/attitude_maneuver.c src/imu/imu_pending.h docs:imu | first-attempt name |
| `agc_job_crew_defined_maneuver_term` | `agc_job_crew_defined_maneuver_terminate` | include/agc/agc_imu.h src/imu/attitude_maneuver.c docs:imu | first-attempt name |
| `temp_attitude_maneuver` | `attitude_maneuver_return` | src/imu/attitude_maneuver.c | first-attempt name |
| `IMU_VF_ATTITUDE_MANEUVER_FLAG` | `IMU_VF_TRACKING_USE_MANEUVER` | src/imu/imu_internal.h | first-attempt name |
| `attitude_maneuver_vsave` | `pointvsm_save` | include/agc/agc_lm_landing.h src/lm/lunar_landing.c | first-attempt name |
| `AGC_JOB_1_GYRO` | `AGC_JOB_GYRO_COMMAND_PULSE` | src/imu/imu_comp.c src/imu/imu_pending.h docs:imu | 1/GYRO: "SCALE GYRO COMMANDS FOR IMUPULSE" ("1/" label; NAMING precedent 1/ACCS) [1/-label] |
| `AGC_JOB_1_GYRO_PULSE` | `AGC_JOB_GYRO_COMMAND_RETRY` | src/imu/imu_comp.c src/imu/imu_pending.h docs:imu | 1/GYRO after GYROBUSY [1/-label] |
| `agc_job_1_gyro` | `agc_job_gyro_command_pulse` | include/agc/agc_imu.h src/imu/imu_comp.c docs:imu | 1/GYRO [1/-label] |
| `agc_job_1_gyro_pulse` | `agc_job_gyro_command_retry` | include/agc/agc_imu.h src/imu/imu_comp.c docs:imu | 1/GYRO retry [1/-label] |
| `agc_imu_1_pipadt` | `agc_imu_pipa_interval` | include/agc/agc_imu.h src/imu/imu_comp.c | 1/PIPADT B(1)PRM: PIPA interval cs x 2^8 used by drift compensation [1/-label] |
| `agc_imu_set_1_pipadt` | `agc_imu_set_pipa_interval` | include/agc/agc_imu.h src/csm/earth_orbit_insertion.c src/csm/servicer_csm.c src/imu/imu_comp.c src/io/t4rupt.c src/lm/servicer_lm.c src/programs/align_common.c tests/test_imu.c | 1/PIPADT setter [1/-label] |

## align -- prog-align / prog-rendz

| old | new | files | AGC source / comment |
|---|---|---|---|
| `agc_csm_auto_optics_positioning_sub_1` | `agc_csm_target_optics_angles` | include/agc/agc_align.h | first-attempt name (opaque sub_1) |
| `agc_align_r51_solve` | `agc_align_fine_solve` | docs:prog-align | R51 star-pair solution (AXISGEN) (stutter align_fine_align collapsed) |
| `agc_align_fine_align_solve` | `agc_align_fine_solve` | include/agc/agc_align.h src/programs/align_locsam.c tests/test_align.c | first-attempt name (stutter) |
| `agc_align_cal53a_need_coarse` | `agc_align_coarse_needed` | docs:prog-align | CAL53A coarse-align test |
| `agc_align_coarse_align_needed` | `agc_align_coarse_needed` | include/agc/agc_align.h src/programs/align_orientation.c src/programs/imu_realign.c | lane name (stutter) |
| `agc_r63_los` | `agc_rendezvous_final_attitude_los` | docs:prog-align | R63 LOS (kept) |
| `savqr52` | `auto_optics_return` | src/programs/align_common.c | SAVQR52 I(2)TMP: saved return of R52 |
| `save_q_auto_optics_positioning` | `auto_optics_return` | include/agc/agc_align.h | first-attempt name |
| `r53exit` | `sighting_mark_return` | src/programs/align_common.c | R53EXIT I(1)TMP: R53 return |
| `sighting_mark_exit` | `sighting_mark_return` | include/agc/agc_align.h | first-attempt name |
| `agc_job_lm_fine_align_k` | `agc_job_lm_fine_align_recheck` | src/lm/lunar_landing.c src/programs/imu_realign.c docs:guid-landing | first-attempt name (opaque _k) |
| `agc_job_csm_fine_align_k` | `agc_job_csm_fine_align_recheck` | src/programs/imu_realign.c | lane name (opaque _k) |
| `agc_rendz_r61csm` | `agc_rendz_tracking_attitude_csm` | src/programs/align_pending.h | R61CSM tracking attitude (align_pending.h decl) |
| `p52_s52_2` | `realign_gimbal_angles` | src/programs/imu_realign.c | P52 call of S52.2 "COMPUTE GIMBAL ANGLES" [S-section] |
| `r55_flash` | `gyro_angles_flash` | src/programs/sighting_data_and_gyro_torquing.c | R55.1: "CAF V06N93 GOFLASH" (N93 gyro torquing angles) |
| `r55_torqued` | `gyro_torquing_done` | src/programs/sighting_data_and_gyro_torquing.c | R55.2 after IMUPULSE (R55RET) |
| `AGC_JOB_FINE_ALIGN_1` | `AGC_JOB_FINE_ALIGN_PERFORM` | src/exec/exec_pending.h src/exec/restart_tables.c | first-attempt name (opaque _1) |
| `rz_p20flgon` | `rz_tracking_flags_on` | src/programs/rendz_common.c src/programs/rendz_csi_cdh_targeting.c src/programs/rendz_internal.h docs:prog-rendz | P20FLGON: "SET UPDATFLG, TRACKFLG" |
| `agc_p76_lv_to_ref` | `agc_target_delta_v_lv_to_ref` | docs:prog-rendz | P76 (kept) |
| `agc_job_auto_optics_positioning_term` | `agc_job_auto_optics_positioning_terminate` | src/csm/sxtmark.c | R52 terminate response (TERM spelled out, as R60/R62) |

## flags -- state-flags

| old | new | files | AGC source / comment |
|---|---|---|---|
| `AGC_FLAG_V37FLAG` | `AGC_FLAG_SERVICER_RUNNING` | src/programs/rendz_flags_csm.c src/programs/rendz_flags_lm.c | V37FLAG: "AVERAGEG (SERVICER) RUNNING / OFF" |
| `AGC_FLAG_CHANGE_PROGRAM_FLAG` | `AGC_FLAG_SERVICER_RUNNING` | include/agc/agc_flags_common.h src/csm/servicer_csm.c src/exec/alarm.c src/exec/change_program_idle.c src/lm/servicer_lm.c tests/test_state.c | first-attempt name (stutter, wrong meaning) |
| `AGC_FLAG_STABLE_ORBIT_MIDCOURSE_SW` | `AGC_FLAG_STABLE_ORBIT_MIDCOURSE_ACTIVE` | include/agc/agc_flags_common.h tests/test_state.c docs:state-flags | first-attempt name (pasted SW) |
| `AGC_FLAG_SIGHTING_MARK_FLAG` | `AGC_FLAG_SIGHTING_MARK_REQUESTED` | include/agc/agc_flags_csm.h | first-attempt name (stutter) |
| `AGC_FLAG_BACKUP_SIGHTING_MARK_FLG` | `AGC_FLAG_BACKUP_MARKING` | include/agc/agc_flags_csm.h tests/test_state.c | first-attempt name (pasted FLG) |
| `AGC_FLAG_P21FLAG` | `AGC_FLAG_GROUND_TRACK_BASE_VECTORS` | src/programs/rendz_flags_csm.c src/programs/rendz_flags_lm.c | P21FLAG: CM "SUCCEEDING PASS THRU P21, USE BASE VECTOR"; LM "USE BASE VECTORS ALREADY CALCULATED" |
| `AGC_FLAG_GROUND_TRACK_DETERMINATION_FLAG` | `AGC_FLAG_GROUND_TRACK_BASE_VECTORS` | include/agc/agc_flags_csm.h include/agc/agc_flags_lm.h tests/test_state_lm.c | first-attempt name (stutter) |
| `AGC_FLAG_R21MARK` | `AGC_FLAG_MARKRUPT_OPTION_ONE` | src/csm/sxtmark.c | CM R21MARK: "OPTION ONE FOR MARKRUPT / OPTION TWO FOR MARKRUPT" |
| `AGC_FLAG_RENDEZVOUS_SIGHTING_MARK` | `AGC_FLAG_MARKRUPT_OPTION_ONE` | include/agc/agc_flags_csm.h | first-attempt name (names the routine, not the state) |
| `AGC_FLAG_ORBITAL_NAVIGATION_MKFLG` | `AGC_FLAG_ORBITAL_NAVIGATION_MARK_TAKEN` | include/agc/agc_flags_csm.h | first-attempt name (pasted MKFLG) |
| `AGC_FLAG_ATTITUDE_MANEUVER_FLAG` | `AGC_FLAG_TRACKING_USE_MANEUVER` | include/agc/agc_flags_csm.h src/imu/imu_vflags_csm.c | first-attempt name (stutter) |
| `AGC_FLAG_V59FLAG` | `AGC_FLAG_CALIBRATION_MARKING` | src/csm/sxtmark.c | CM V59FLAG: "CALIBRATING FOR P23 / NORMAL MARKING FOR P23" |
| `AGC_FLAG_CALIBRATION_FLAG` | `AGC_FLAG_CALIBRATION_MARKING` | include/agc/agc_flags_csm.h | first-attempt name (stutter) |
| `AGC_FLAG_OPTICS_CALIBRATION_FLAG` | `AGC_FLAG_TRUNNION_BIAS_KNOWN` | include/agc/agc_flags_csm.h tests/test_state.c | first-attempt name (stutter, inverted sense) |
| `AGC_FLAG_STOP_INTEGRATION_ONFLG` | `AGC_FLAG_IDLE_INTEGRATION_INHIBITED` | include/agc/agc_flags_csm.h src/nav/integration_csm.c | first-attempt name (pasted ONFLG) |
| `agc_integ_csm_clear_stop_integration_onflg` | `agc_integ_csm_resume_idle_integration` | src/nav/integ_internal.h src/nav/integration_csm.c src/nav/integration_jobs.c | first-attempt name |
| `AGC_FLAG_R31FLAG` | `AGC_FLAG_RENDEZVOUS_DISPLAY_THETA` | src/programs/rendz_flags_csm.c | CM R31FLAG: "CHK R31FLAG. ON=R31 THETA, OFF=R34 PHI" (R31.agc) |
| `AGC_FLAG_RENDEZVOUS_PARAMETER_DISPLAY_FLAG` | `AGC_FLAG_RENDEZVOUS_DISPLAY_THETA` | include/agc/agc_flags_csm.h | first-attempt name (stutter) |
| `AGC_FLAG_N22ORN17` | `AGC_FLAG_ATTITUDE_ERROR_DESIRED_ANGLES` | src/csm/dap/rcs_filter.c | CM N22ORN17: "COMPUTE TOTAL ATTITUDE ERRORS W.R.T. N22 (V62) / W.R.T. N17 (V63)" |
| `AGC_FLAG_TOTAL_ATTITUDE_ERROR_REFERENCE` | `AGC_FLAG_ATTITUDE_ERROR_DESIRED_ANGLES` | include/agc/agc_flags_csm.h | first-attempt name (sense not stated) |
| `n22` | `use_desired_angles` | src/csm/dap/rcs_filter.c | local copy of N22ORN17 (rcs_filter.c) |
| `AGC_FLAG_RENDEZVOUS_TRACKING_DATA_CAFLG` | `AGC_FLAG_RENDEZVOUS_TRACKING_BUSY` | include/agc/agc_flags_csm.h | first-attempt name (pasted CAFLG) |
| `AGC_FLAG_ALTERNATE_LOS_SIGHTING_MARK_FLAG` | `AGC_FLAG_ALTERNATE_LOS_ALLOWED` | include/agc/agc_flags_csm.h | first-attempt name (stutter) |
| `AGC_FLAG_V82EMFLG` | `AGC_FLAG_ORBIT_DISPLAY_MOON` | src/programs/rendz_flags_csm.c src/programs/rendz_flags_lm.c | V82EMFLG: "MOON VICINITY / EARTH VICINITY" (V82 orbit parameter display) |
| `AGC_FLAG_ORBIT_PARAMETER_DISPLAY_EMFLG` | `AGC_FLAG_ORBIT_DISPLAY_MOON` | include/agc/agc_flags_csm.h include/agc/agc_flags_lm.h tests/test_state_lm.c | first-attempt name (pasted EMFLG) |
| `AGC_FLAG_NODOP01` | `AGC_FLAG_NO_PRELAUNCH_INITIALIZATION` | include/agc/agc_flags_csm.h src/csm/earth_orbit_insertion.c | CM NODOP01: "P01 NOT ALLOWED / P01 ALLOWED" |
| `AGC_FLAG_LANDING_ANALOG_DISPLAYS_FLAG` | `AGC_FLAG_ALTITUDE_METERS_ONLY` | include/agc/agc_flags_lm.h src/lm/ascent.c src/lm/landing_aborts.c src/lm/landing_displays.c | first-attempt name (stutter) |
| `AGC_FLAG_PREFERRED_TRACKING_ATTITUDE_FLAG` | `AGC_FLAG_PREFERRED_TRACKING_ATTITUDE_ACTIVE` | include/agc/agc_flags_lm.h src/lm/lunar_landing.c | first-attempt name (stutter) |
| `AGC_FLAG_TRACKING_ATTITUDE_FLAG` | `AGC_FLAG_USE_TRACKING_ATTITUDE` | include/agc/agc_flags_lm.h | first-attempt name (stutter) |
| `AGC_FLAG_RADAR_TEST_FLAG` | `AGC_FLAG_ALARM_521_SUPPRESSED` | include/agc/agc_flags_lm.h tests/test_state_lm.c | first-attempt name (stutter) |
| `AGC_FLAG_RADAR_TEST_NOALARM_FLAG` | `AGC_FLAG_RADAR_ALARMS_SUPPRESSED` | include/agc/agc_flags_lm.h | first-attempt name (stutter) |
| `AGC_FLAG_W_MATRIX_OVERWRITE_FLAG` | `AGC_FLAG_W_MATRIX_OVERWRITE` | include/agc/agc_flags_lm.h | first-attempt name (stutter) |
| `AGC_FLAG_NOR29FLG` | `AGC_FLAG_NO_POWERED_DESIGNATE` | include/agc/agc_flags_lm.h src/lm/ascent.c src/lm/servicer_lm.c src/lm/servicer_lm_lr.c | LM NOR29FLG: "R29 NOT ALLOWED / R29 ALLOWED (RR DESIGNATED POWERED FLT)" |

## io -- io-nouns / io-pinball / sys-misc / tests

| old | new | files | AGC source / comment |
|---|---|---|---|
| `AGC_DATA_RADAR_RESIDUAL_DISP` | `AGC_DATA_MARK_DELTA_R` | include/agc/agc_nouns.h src/io/noun_tables_csm.c | first-attempt name |
| `AGC_DATA_RADAR_RESIDUAL_DISP_P2` | `AGC_DATA_MARK_DELTA_V` | include/agc/agc_nouns.h src/io/noun_tables_csm.c | first-attempt name (opaque _P2) |
| `AGC_DATA_RADAR_RESIDUAL_DISP_P4` | `AGC_DATA_MARK_SOURCE_CODE` | include/agc/agc_nouns.h src/io/noun_tables_csm.c | first-attempt name (opaque _P4) |
| `AGC_DATA_RENDEZVOUS_TRACKING_DATA_DISP` | `AGC_DATA_RADAR_DELTA_R` | include/agc/agc_nouns.h src/io/noun_tables_lm.c | first-attempt name |
| `AGC_DATA_RENDEZVOUS_TRACKING_DATA_DISP_P2` | `AGC_DATA_RADAR_DELTA_V` | include/agc/agc_nouns.h src/io/noun_tables_lm.c | first-attempt name (opaque _P2) |
| `agc_p27_start` | `agc_uplink_update_start` | src/io/sysmisc.h | P27 UPDATE PROGRAM: V70..V73 entry |
| `agc_p27_response` | `agc_uplink_update_response` | src/io/sysmisc.h | P27 crew response |
| `agc_p27_pending` | `agc_uplink_update_pending` | src/io/sysmisc.h | P27 pending display |
| `agc_p27_dnword` | `agc_uplink_update_dnword` | src/io/sysmisc.h | P27 downlink words COMPNUMB, UPOLDMOD, UPVERB, UPCOUNT, UPBUFF |
| `agc_p27_restart` | `agc_uplink_update_restart` | src/io/sysmisc.h | P27 group 6 restart |
| `agc_p07_zeroing` | `agc_systems_test_zeroing` | src/io/sysmisc.h | LM P07 IMU performance tests: ZEROING |
| `agc_p07_finetime` | `agc_systems_test_finetime` | src/io/sysmisc.h | P07 FINETIME |
| `agc_p07_overffix` | `agc_systems_test_overffix` | src/io/sysmisc.h | P07 OVERFFIX |
| `agc_p07_erthrvse` | `agc_systems_test_erthrvse` | src/io/sysmisc.h | P07 ERTHRVSE (CSM twin: agc_csm_prelaunch_erthrvse) |
| `agc_p07_earthr` | `agc_systems_test_earthr` | src/io/sysmisc.h | P07 EARTHR (CSM twin: agc_csm_prelaunch_earthr) |
| `agc_p07_redo` | `agc_systems_test_redo` | src/io/sysmisc.h | P07 REDO (V92) |
| `agc_p07_estims` | `agc_systems_test_estims` | src/io/sysmisc.h | P07 ESTIMS |
| `verb06` | `saw_decimal_display` | tests/test_dsky.c | test_dsky.c local: V06 "display decimal" seen |
| `CMPG2205` | `CM_ORBITAL_NAV_LIST05` | src/csm/downlink_lists_csm.c | CM DOWNLINK_LISTS CMPG2205: sublist 05 of the CM P22 list (CMPG22) "SNAPSHOT DATA" |
| `cmpg2205` | `cm_orbital_nav_list05` | src/csm/downlink_lists_csm.c | same, data array |

## files -- file renames (git mv) and file names in notes

| old | new | files | AGC source / comment |
|---|---|---|---|
| `p30.c` | `external_delta_v.c` | src/guidance/p30.c docs:guid-powered | src/guidance/p30.c: P30 external delta-V + P31 Lambert aimpoint + S30.1/S31.1 (git mv) |
| `p40.c` | `burn_steering.c` | src/guidance/p40.c docs:guid-powered | src/guidance/p40.c: "the steering mathematics of P40-P47.agc": S40.1, S40.2,3, S41.1, S40.8, S40.13 (git mv) |
| `p37.c` | `return_to_earth.c` | src/csm/p37.c docs:MODULE_STATUS,guid-entry | src/csm/p37.c: P37 RETURN TO EARTH (git mv; NAMING.md example) |
| `p11.c` | `earth_orbit_insertion.c` | docs:MODULE_STATUS | docs only (already moved by first attempt) |
| `p30_csm.c` | `external_delta_v_csm.c` | docs:guid-powered | docs only (already moved) |
| `p40_csm.c` | `sps_burn.c` | docs:dap-csm,guid-powered | docs only (already moved) |
| `p40_csm_burn.c` | `sps_burn_ignition.c` | docs:guid-powered | docs only (already moved) |
| `p40_csm_clock.c` | `sps_burn_clock.c` | docs:guid-powered | docs only (already moved) |
| `p40_csm_internal.h` | `sps_burn_internal.h` | docs:guid-powered | docs only (already moved) |
| `p40_csm_steer.c` | `sps_burn_steer.c` | docs:guid-powered | docs only (already moved) |
| `p40_lm.c` | `dps_burn.c` | docs:guid-powered | docs only (already moved) |
| `p40_lm_rasteer.c` | `dps_burn_rasteer.c` | docs:guid-powered | docs only (already moved) |
| `p61_p67.c` | `entry_programs.c` | docs:MODULE_STATUS | docs only (already moved) |
| `p20_csm.c` | `rendezvous_navigation_csm.c` | docs:prog-rendz | docs only, planned file (prog-rendz) |
| `p20_lm.c` | `rendezvous_navigation_lm.c` | docs:prog-rendz | docs only, planned file (prog-rendz) |
| `p21.c` | `ground_track_determination.c` | docs:prog-rendz | docs only, planned file |
| `p27.c` | `uplink_update.c` | docs:sys-misc | docs only, planned file (sys-misc) |
| `p51_p53_csm.c` | `align_programs_csm.c` | docs:prog-align | docs only, planned file (prog-align: P51/P52/P53 CSM) |
| `p51_p53_lm.c` | `align_programs_lm.c` | docs:prog-align | docs only, planned file |
| `p57.c` | `lunar_surface_align.c` | docs:prog-align | docs only, planned file |
| `p76.c` | `target_delta_v.c` | docs:prog-rendz | docs only, planned file |
| `r30.c` | `orbit_parameter_display.c` | docs:prog-rendz | docs only, planned file |
| `r31.c` | `rendezvous_parameter_display.c` | docs:prog-rendz | docs only, planned file |
| `r31_csm.c` | `rendezvous_parameter_display_csm.c` | docs:prog-rendz | docs only, planned file |
| `r31_lm.c` | `rendezvous_parameter_display_lm.c` | docs:prog-rendz | docs only, planned file |
| `r52_r53_csm.c` | `optics_sighting_csm.c` | docs:prog-align | docs only, planned file (R52 auto optics + R53 sighting mark, CSM) |
| `r59_r52_lm.c` | `star_acquisition_lm.c` | docs:prog-align | docs only, planned file (R59 lunar-surface star acquisition + R52, LM) |
| `r63.c` | `rendezvous_final_attitude.c` | docs:prog-align | docs only, planned file |
| `rendz_p32_p33.c` | `rendz_csi_cdh_targeting.c` | docs:prog-rendz | docs only (file now exists under this name) |
| `rendz_p34_p35.c` | `rendz_tpi_tpm_targeting.c` | docs:prog-rendz | docs only, planned file |
| `rendz_r36.c` | `rendz_out_of_plane.c` | docs:prog-rendz | docs only, planned file |

## docs -- identifiers that occur only in docs/modules notes

| old | new | files | AGC source / comment |
|---|---|---|---|
| `P40S_SV` | `PREBURN_STATE_VECTOR` | docs:exec-sched | bare suffix of AGC_JOB_P40S_SV in exec-sched.md |
| `P61_1` | `ENTRY_PREPARATION_PREDICTION_DISPLAY` | docs:guid-entry | bare suffix (P61.1) |
| `P61_2` | `ENTRY_PREPARATION_EMS_DISPLAY` | docs:guid-entry | bare suffix (P61.2) |
| `P61_3` | `ENTRY_DISPLAY_PROTECT` | docs:guid-entry | bare suffix (P61.3) |
| `P61_4` | `ENTRY_PREPARATION_COMPUTE` | docs:guid-entry | bare suffix (P61.4) |
| `P62_1` | `CM_SM_SEPARATION_TARGET_DISPLAY` | docs:guid-entry | bare suffix (P62.1) |
| `P62_2` | `CM_SM_SEPARATION_REQUEST` | docs:guid-entry | bare suffix (P62.2) |
| `P62_2B` | `CM_SM_SEPARATION_PROCEED` | docs:guid-entry | bare suffix (P62.2 +3) |
| `P65_1` | `ENTRY_UPCONTROL_DISPLAY` | docs:exec-sched,guid-entry | bare suffix (P65.1) |
| `P65_1B` | `ENTRY_UPCONTROL_PROCEED` | docs:guid-entry | bare suffix (P65.1 PROCEED) |
| `P67_1` | `ENTRY_FINAL_PHASE_DISPLAY` | docs:exec-sched,guid-entry | bare suffix (P67.1) |
| `P67_1B` | `ENTRY_FINAL_PHASE_END` | docs:guid-entry | bare suffix (P67.1 end) |
| `R10_R11` | `ABORT_MONITOR` | docs:exec-sched,guid-landing | bare suffix of AGC_TASK_R10_R11 |
| `N49DISP_P2` | `MARK_DELTA_V` | docs:io-nouns | bare suffix of AGC_DATA_N49DISP_P2 |
| `N49DISP_P4` | `MARK_SOURCE_CODE` | docs:io-nouns | bare suffix of AGC_DATA_N49DISP_P4 |
| `R22DISP_P2` | `RADAR_DELTA_V` | docs:io-nouns | bare suffix of AGC_DATA_R22DISP_P2 |
| `R51_1` | `FINE_ALIGN_PERFORM` | docs:exec-sched | bare suffix of AGC_JOB_R51_1 |
| `S40_13` | `TIMEBURN` | docs:exec-sched | bare suffix [S-section] |
| `S61_1A` | `ENTRY_IMU_AXIS_TEST` | docs:exec-sched,guid-entry | bare suffix [S-section] |
| `S61_1C` | `ENTRY_START_AVERAGE_G` | docs:exec-sched,guid-entry | bare suffix [S-section] |
| `V97E40_6` | `ENGINE_FAIL_REIGNITE_TRIM` | docs:exec-sched | bare suffix of AGC_TASK_V97E40_6 |
| `_p00_klean` | `_idle_klean` | docs:exec-sched | doc fragment of agc_phase_p00_klean |
| `_v37_klean` | `_change_program_klean` | docs:exec-sched | doc fragment of agc_phase_v37_klean |
| `_p12flash` | `_powered_ascent_flash` | docs:guid-landing | doc fragment |
| `_p12lm2` | `_powered_ascent_proceed` | docs:guid-landing | doc fragment |
| `_p12lmb` | `_powered_ascent_at_ignition` | docs:guid-landing | doc fragment |
| `_p68_flash` | `_landing_confirmation_flash` | docs:guid-landing | doc fragment |
| `_p68b` | `_landing_confirmation_proceed` | docs:guid-landing | doc fragment |
| `agc_p07_` | `agc_systems_test_` | docs:sys-misc | doc prefix |
| `agc_p27_` | `agc_uplink_update_` | docs:sys-misc | doc prefix |
| `agc_align_p52ls_csm` | `agc_align_landing_site_orientation_csm` | docs:prog-align | P52LS (CSM) landing-site orientation (doc) |
| `agc_align_r51p63` | `agc_align_fine_align_recheck` | docs:guid-landing | R51P63 entry of R51K (doc) |
| `agc_imu_r51p63` | `agc_imu_fine_align_recheck` | docs:guid-landing,imu | R51P63 entry of R51K (doc) |
| `agc_align_s52_2` | `agc_align_gimbal_angles` | docs:prog-align | S52.2 "COMPUTE GIMBAL ANGLES" (name already in agc_align.h) [S-section] |
| `agc_align_s52_3` | `agc_align_nominal_orientation` | docs:prog-align | S52.3 (name already in agc_align.h) [S-section] |
| `agc_entry_p61` | `agc_job_entry_preparation` | docs:guid-entry | doc |
| `agc_entry_p62` | `agc_job_cm_sm_separation` | docs:guid-entry | doc |
| `agc_entry_p63` | `agc_job_entry_initialization` | docs:guid-entry | doc |
| `agc_entry_s61_1` | `agc_job_entry_imu_check` | docs:guid-entry | doc [S-section] |
| `agc_entry_s61_1a` | `agc_job_entry_imu_axis_test` | docs:guid-entry | doc [S-section] |
| `agc_erase_p40tmp` | `agc_erase_preread_delay` | docs:exec-sched | doc: P40TMP "DELTA TIME TO PREREAD" |
| `agc_guid_s40_13_lm` | `agc_guid_timeburn_lm` | docs:guid-powered | doc [S-section] |
| `agc_guid_s40_8_tgo` | `agc_guid_steering_tgo` | docs:guid-powered | doc [S-section] |
| `agc_imu_r60lem` | `agc_imu_attitude_maneuver` | docs:imu | doc (R60LEM) |
| `agc_job_r60` | `agc_job_attitude_maneuver` | docs:imu | doc |
| `agc_job_p30_csm` | `agc_job_external_delta_v_csm` | docs:guid-powered | doc |
| `agc_job_p30_lm` | `agc_job_external_delta_v_lm` | docs:guid-powered | doc |
| `agc_job_p31_csm` | `agc_job_lambert_aimpoint_guidance_csm` | docs:guid-powered | doc |
| `agc_lm_job_p12newload` | `agc_lm_job_powered_ascent_newload` | docs:guid-landing | doc (NEWLOAD word label) |
| `agc_lm_p64disps` | `agc_lm_approach_phase_display` | docs:guid-landing | doc |
| `agc_lm_p70` | `agc_lm_dps_abort` | docs:guid-landing | doc |
| `agc_lm_p71` | `agc_lm_aps_abort` | docs:guid-landing | doc |
| `agc_p21_flight_path` | `agc_ground_track_determination_flight_path` | docs:prog-rendz | doc (first-attempt map name) |
| `agc_prog_p21` | `agc_prog_ground_track_determination` | docs:prog-rendz | doc, planned (prog-rendz) |
| `agc_prog_p31` | `agc_prog_lambert_aimpoint_guidance` | docs:prog-rendz | doc, planned |
| `agc_prog_p32` | `agc_prog_csi_targeting` | docs:prog-rendz | doc, planned |
| `agc_prog_p33` | `agc_prog_cdh_targeting` | docs:prog-rendz | doc, planned |
| `agc_prog_p34` | `agc_prog_tpi_targeting` | docs:prog-rendz | doc, planned |
| `agc_prog_p38` | `agc_prog_stable_orbit_rendezvous` | docs:prog-rendz | doc, planned |
| `agc_prog_p76` | `agc_prog_target_delta_v` | docs:prog-rendz | doc, planned |
| `agc_r30_maxchk` | `agc_orbit_parameter_display_maxchk` | docs:prog-rendz | doc, planned (R30 MAXCHK) |
| `agc_r30_sr30_1` | `agc_orbit_parameter_display_compute` | docs:prog-rendz | doc, planned (SR30.1) [S-section] |
| `agc_r31_compdisp` | `agc_rendezvous_parameter_display_compdisp` | docs:prog-rendz | doc, planned (R31 COMPDISP) |
| `agc_rendz_p38_deltt` | `agc_rendz_stable_orbit_rendezvous_deltt` | docs:prog-rendz | doc, planned |
| `agc_rendz_r36_compute` | `agc_rendz_out_of_plane_compute` | docs:prog-rendz | doc, planned (R36) |
| `agc_verb82_r30` | `agc_verb_orbit_parameter_display` | docs:prog-rendz | doc, planned V82 handler (rule 6: verb handler by function) |
| `agc_verb83_r31` | `agc_verb_rendezvous_parameter_display` | docs:prog-rendz | doc, planned V83 handler |
| `agc_verb90_r36` | `agc_verb_out_of_plane_display` | docs:prog-rendz | doc, planned V90 handler |
| `ldg_p65vert` | `ldg_velocity_nulling_vertical` | docs:guid-landing | doc (LM P65VERT) |
| `lnd_startp66` | `lnd_start_rate_of_descent_landing` | docs:guid-landing | doc |
| `lnd_startp67` | `lnd_start_manual_landing` | docs:guid-landing | doc |
| `lnd_strtp66a` | `lnd_rate_of_descent_setup` | docs:guid-landing | doc |
| `p70_p71` | `landing_aborts` | docs:MODULE_STATUS | doc file stem |

## keep -- regex hits left unchanged

| identifier | files | reason |
|---|---|---|
| `AGC_CH013_RSTTRAP31A` | include/agc/agc_channels.h src/lm/landing_guidance.c src/lm/lunar_landing.c | channel 13 bit name (RUPT trap 31A, hardware) |
| `AGC_CH013_RSTTRAP31B` | include/agc/agc_channels.h | channel 13 bit name |
| `AGC_CH013_RSTTRAP32` | include/agc/agc_channels.h | channel 13 bit name |
| `AGC_CH032_THR1011DIS` | include/agc/agc_channels.h | channel 32 thruster-pair disable bits (thruster numbers) |
| `AGC_CH032_THR1315DIS` | include/agc/agc_channels.h | thruster numbers |
| `AGC_CH032_THR13DIS` | include/agc/agc_channels.h | thruster numbers |
| `AGC_CH032_THR1416DIS` | include/agc/agc_channels.h | thruster numbers |
| `AGC_CH032_THR24DIS` | include/agc/agc_channels.h | thruster numbers |
| `AGC_CH032_THR58DIS` | include/agc/agc_channels.h | thruster numbers |
| `AGC_CH032_THR67DIS` | include/agc/agc_channels.h | thruster numbers |
| `AGC_CH032_THR912DIS` | include/agc/agc_channels.h | thruster numbers |
| `AGC_CHAN_CHAN12` | include/agc/agc_channels.h src/core/agc_channels.c src/csm/dap/needles.c src/csm/dap/tvc_restarts.c src/csm/t4rupt_csm.c src/exec/fresh_start.c src/imu/imu_gyro.c src/imu/imu_mode.c src/io/pinball_monitor.c src/io/t4rupt.c src/lm/landing_displays.c src/lm/lunar_landing.c src/lm/t4rupt_lm.c tests/test_state.c | I/O channel number (hardware address) |
| `AGC_CHAN_CHAN13` | include/agc/agc_channels.h src/core/agc_channels.c src/csm/dap/jet_select.c src/csm/dap/rcs_dap.c src/csm/dap/tvc_rolldap.c src/exec/fresh_start.c src/io/pinball_keys.c src/io/pinball_monitor.c src/lm/landing_guidance.c src/lm/lunar_landing.c | I/O channel number |
| `AGC_CHAN_CHAN14` | include/agc/agc_channels.h src/core/agc_channels.c src/csm/dap/needles.c src/csm/dap/tvc_daps.c src/csm/dap/tvc_restarts.c src/csm/dap/tvc_stroke.c src/csm/t4rupt_csm.c src/exec/fresh_start.c src/imu/imu_gyro.c src/imu/imu_mode.c src/io/t4rupt.c src/lm/landing_aborts.c src/lm/landing_displays.c src/lm/throttle.c | I/O channel number |
| `AGC_CHAN_CHAN30` | include/agc/agc_channels.h src/core/agc_channels.c src/csm/t4rupt_csm.c src/imu/attitude_maneuver.c src/io/t4rupt.c src/lm/findcduw.c src/lm/landing_aborts.c src/lm/landing_displays.c src/lm/landing_state.c src/lm/lunar_landing.c src/lm/t4rupt_lm.c tests/test_state.c tests/test_state_lm.c | I/O channel number |
| `AGC_CHAN_CHAN31` | include/agc/agc_channels.h src/core/agc_channels.c src/csm/dap/jet_select.c src/csm/dap/rcs_attitude.c src/csm/dap/rcs_dap.c src/csm/dap/rcs_manual.c src/imu/attitude_maneuver.c src/lm/findcduw.c src/lm/landing_guidance.c src/lm/lunar_landing.c tests/test_state.c | I/O channel number |
| `AGC_CHAN_CHAN32` | include/agc/agc_channels.h src/core/agc_channels.c src/csm/dap/rcs_attitude.c src/io/t4rupt.c tests/test_state.c | I/O channel number |
| `AGC_CHAN_CHAN33` | include/agc/agc_channels.h src/core/agc_channels.c src/csm/t4rupt_csm.c src/exec/fresh_start.c src/io/noun_data.c src/io/pinball_scale.c src/io/t4rupt.c src/lm/landing_state.c src/lm/lunar_landing.c src/lm/t4rupt_lm.c tests/test_exec.c tests/test_state.c | I/O channel number |
| `AGC_CSM_N10_WORDS` | include/agc/agc_csm_dap.h src/csm/dap/tvc_init.c | TVC filter coefficient block N10.. (CSMN10 "N10 CSM ONLY FILTER COEFFICIENTS"), not a noun |
| `AGC_DSKY_KEY_1` | include/agc/agc_dsky.h | DSKY key "1" code |
| `AGC_ENTRY_KTRCS` | src/csm/entry_dap.c src/csm/entry_lexicon.h | KTRCS gain constant (matched TRCS) |
| `AGC_FLAG_S32_1F1` | include/agc/agc_flags_csm.h include/agc/agc_flags_lm.h src/programs/rendz_flags_csm.c src/programs/rendz_flags_lm.c tests/test_state.c docs:state-flags | AGC flag-word name S32.1F1 (S-section flag; lead may extend S-section scope to flags) |
| `AGC_FLAG_S32_1F2` | include/agc/agc_flags_csm.h include/agc/agc_flags_lm.h src/programs/rendz_flags_csm.c src/programs/rendz_flags_lm.c | AGC flag-word name S32.1F2 |
| `AGC_FLAG_S32_1F3A` | include/agc/agc_flags_csm.h include/agc/agc_flags_lm.h src/programs/rendz_flags_csm.c src/programs/rendz_flags_lm.c | AGC flag-word name S32.1F3A |
| `AGC_FLAG_S32_1F3B` | include/agc/agc_flags_csm.h include/agc/agc_flags_lm.h src/programs/rendz_flags_csm.c src/programs/rendz_flags_lm.c tests/test_state.c | AGC flag-word name S32.1F3B |
| `AGC_JOB_HIGATJOB` | src/guidance/guid_pending.h src/lm/servicer_lm_lr.c | HIGATJOB word label (matched TJOB) |
| `AGC_JOB_HIGATJOB_DONE` | src/guidance/guid_pending.h src/lm/servicer_lm_lr.c | HIGATJOB after RADSTALL |
| `AGC_K_SUPER011` | include/agc/agc_constants.h | SUPERBNK bit pattern 011 |
| `AGC_K_SUPER101` | include/agc/agc_constants.h | SUPERBNK bit pattern 101 |
| `AGC_K_SUPER110` | include/agc/agc_constants.h | SUPERBNK bit pattern 110 |
| `ARITH_SIGN15` | src/lm/dap/arith.c | bit 15 sign |
| `AS_ATRCS` | src/lm/ascent.c | AT/RCS constant (matched TRCS) |
| `CMENTR05` | src/csm/downlink_lists_csm.c | CMENTR05 downlink sublist label (entry list 05); no program number |
| `DSP11_CHANGED` | src/imu/imu_internal.h src/imu/imu_mode.c | DSPTAB +11D lamp word index |
| `DSP11_GLOCK` | src/imu/imu_internal.h src/imu/imu_mode.c | DSPTAB +11D lamp word index |
| `DSP11_NOATT` | src/imu/imu_internal.h src/imu/imu_mode.c | DSPTAB +11D lamp word index |
| `FS_PHASE_4_1` | src/exec/fresh_start.c | restart group 4 phase 1 (data) |
| `MIN14MS` | src/csm/dap/jet_select.c | 14 ms minimum jet time |
| `MUDT_E` | src/guidance/servicer_common.c | Earth MUDT |
| `MU_E` | tests/test_integration.c | Earth mu (test) |
| `OPT_CODE_1` | src/programs/imu_realign.c | option code value 1 (data) |
| `PIN_13_11_1` | src/io/pinball_monitor.c | IMODES33 bit mask "13-11,1" |
| `QTSN45` | src/imu/alignment.c | QTSN45 constant sin(45)/4 |
| `RST_E_S61DT` | src/exec/exec_internal.h src/exec/restart_tables.c | restart erasable selector for S61DT |
| `RST_E_TGO_1` | src/exec/exec_internal.h src/exec/restart_tables.c | TGO +1 word (low word) |
| `RST_T` | src/exec/restart_tables.c | restart type task |
| `RZ_SN359` | src/programs/rendz_csi.c src/programs/rendz_internal.h | sin(359 deg) constant |
| `SIN33` | src/programs/align_locsam.c | SIN33 constant |
| `T6_IDX_P` | src/lm/dap/t6rupt.c | P-axis index |
| `agc_csm_prelaunch_posn17c` | include/agc/agc_imu.h src/csm/prelaunch_align.c | POSN17C AGC label "COMPASS POSITION Z DOWN" (not noun 17) |
| `agc_erase_imodes30` | include/agc/agc_state.h src/core/agc_erasable.c src/exec/change_program_idle.c src/exec/fresh_start.c src/imu/imu_mode.c src/io/pinball_keys.c src/io/pinball_monitor.c src/io/t4rupt.c docs:state-flags | IMODES30 (channel 30 image) |
| `agc_erase_imodes33` | include/agc/agc_state.h src/core/agc_erasable.c src/csm/dap/rcs_dap.c src/csm/entry_dap.c src/csm/t4rupt_csm.c src/exec/fresh_start.c src/imu/imu_mode.c src/io/pinball_keys.c src/io/pinball_monitor.c src/io/t4rupt.c src/lm/dap/dapidler.c src/lm/landing_aborts.c src/lm/landing_displays.c docs:dap-lm,state-flags | IMODES33 (channel 33 image) |
| `agc_erase_s61dt` | include/agc/agc_csm_entry.h src/csm/entry_programs.c docs:exec-sched | erasable accessor for S61DT (agc_erase_<agcname> convention) |
| `agc_erase_set_imodes30` | include/agc/agc_state.h src/core/agc_erasable.c src/exec/fresh_start.c src/imu/imu_mode.c src/io/pinball_keys.c src/io/pinball_monitor.c src/io/t4rupt.c docs:state-flags | IMODES30 setter |
| `agc_erase_set_imodes33` | include/agc/agc_state.h src/core/agc_erasable.c src/exec/fresh_start.c src/imu/imu_mode.c src/io/pinball_keys.c src/io/pinball_monitor.c src/io/t4rupt.c src/lm/landing_displays.c docs:state-flags | IMODES33 setter |
| `agc_imu_fin20` | include/agc/agc_imu.h src/imu/imu_mode.c | FIN20 imu_mode helper (no P/R/V/N number; "FIN" + 20) |
| `chan14` | src/imu/imu_gyro.c | channel 14 image (local) |
| `chan31` | include/agc/agc_csm_dap.h src/csm/dap/jet_select.c tests/test_csm_dap.c | channel 31 image (member) |
| `cmentr05` | src/csm/downlink_lists_csm.c | same, data array |
| `csmn10` | src/csm/dap/tvc_init.c | CSMN10 TVC filter coefficients (TVCINITIALIZE.agc) |
| `dsp11_write` | src/imu/imu_mode.c | DSPTAB +11D writer |
| `err20` | tests/test_entry.c | 20-degree error (test local) |
| `fc_wrap15` | src/lm/findcduw.c | 15-bit wrap helper |
| `hbn10` | include/agc/agc_csm_dap.h src/csm/dap/tvc_init.c | HBN10 TVC filter coefficients |
| `lbn10` | src/csm/dap/tvc_init.c | LBN10 TVC filter coefficients |
| `n10` | include/agc/agc_csm_dap.h src/csm/dap/tvc_daps.c src/csm/dap/tvc_init.c tests/test_csm_dap.c | N10.. TVC filter coefficient array |
| `noun_access_chan33` | src/io/noun_data.c | channel 33 accessor |
| `pin_noun7_bits` | src/io/pinball_load.c | NOUN 7 (bit set/reset noun) handler is a table-indexed data noun; kept (see report) |
| `temp333` | src/csm/dap/tvc_massprop.c | TEMP333 erasable (TVCMASSPROP counting register) |

## keep -- compliant names already in the tree (first-attempt or lane names reviewed and accepted)

`AGC_CSM_NV_ENGINE_ENABLE_REQUEST`, `AGC_CSM_NV_ENGINE_FAIL_PROCEDURE`, `AGC_DATA_GROUND_TRACK_DETERMINATION_ALT`, `AGC_DATA_GROUND_TRACK_DETERMINATION_GAM`, `AGC_DATA_GROUND_TRACK_DETERMINATION_VEL`, `AGC_EARTH_ORBIT_INSERTION_MONITOR_POLY_MAX`, `AGC_JOB_APS_ABORT`, `AGC_JOB_ATTITUDE_MANEUVER_ENTER`, `AGC_JOB_CDH_TARGETING`, `AGC_JOB_CDH_TARGETING_OTHER_VEHICLE`, `AGC_JOB_CM_SM_SEPARATION`, `AGC_JOB_CM_SM_SEPARATION_NEWMODE`, `AGC_JOB_COARSE_ALIGN`, `AGC_JOB_COARSE_ALIGN_COARSED`, `AGC_JOB_CREW_DEFINED_MANEUVER_GOMOVE`, `AGC_JOB_CSI_TARGETING`, `AGC_JOB_CSI_TARGETING_OTHER_VEHICLE`, `AGC_JOB_DPS_ABORT`, `AGC_JOB_EARTH_ORBIT_INSERTION_MONITOR`, `AGC_JOB_ENTRY_INITIALIZATION`, `AGC_JOB_ENTRY_PREPARATION`, `AGC_JOB_FINE_ALIGN`, `AGC_JOB_GYRO_TORQUING_PULSE`, `AGC_JOB_GYRO_TORQUING_PULSED`, `AGC_JOB_GYRO_TORQUING_RETURN`, `AGC_JOB_IMU_REALIGN_OPTION`, `AGC_JOB_IMU_REALIGN_OPTION_DISPLAY`, `AGC_JOB_JET_ACCEL_RESET`, `AGC_JOB_JET_ACCEL_UPDATE`, `AGC_JOB_POWERED_ASCENT_BURN`, `AGC_JOB_POWERED_ASCENT_FLASH`, `AGC_JOB_REDO_RENDEZVOUS_TRACKING_DATA`, `AGC_JOB_RENDEZVOUS_TRACKING_DATA`, `AGC_JOB_RETURN_TO_EARTH`, `AGC_JOB_SYSTEMS_TEST_REDO`, `AGC_JOB_TPI_TARGETING`, `AGC_JOB_TPI_TARGETING_OTHER_VEHICLE`, `AGC_JOB_TPM_TARGETING`, `AGC_JOB_TPM_TARGETING_OTHER_VEHICLE`, `AGC_JOB_UPLINK_UPDATE_RESUME`, `AGC_JOB_UPLINK_UPDATE_TERM`, `AGC_K_ALARM_CODES_DISPLAY`, `AGC_K_NEW_GIMBAL_ANGLES_DISPLAY`, `AGC_LM_WHICH_APS_BURN`, `AGC_LM_WHICH_BRAKING_PHASE`, `AGC_LM_WHICH_POWERED_ASCENT`, `AGC_LM_WHICH_RCS_BURN`, `AGC_TASK_WAKE_CM_SM_SEPARATION`, `CALLER_CALC_VG_BODY`, `ENGINE_FAIL_DELAY`, `RZ_VF_CHANGE_PROGRAM`, `SPS_BURN_INTERNAL_H`, `THR_RATE_OF_DESCENT_LANDING`, `VN_ENTRY_PREDICTION_DISPLAY`, `VN_ENTRY_TARGET_DISPLAY`, `VP_APS_ABORT`, `VP_DPS_ABORT`, `VP_VERB_CHANGE_PROGRAM`, `ab_dps_abort_init`, `agc_align_gyro_torquing`, `agc_align_ones_to_twos`, `agc_align_sighting_data_display`, `agc_change_program`, `agc_csm_dap_decode_dap_data`, `agc_csm_dap_rcs_gains`, `agc_csm_is_rcs_burn`, `agc_earth_orbit_insertion_monitor_attitude_errors`, `agc_earth_orbit_insertion_monitor_padload`, `agc_earth_orbit_insertion_monitor_padload_t`, `agc_earth_orbit_insertion_monitor_set_padload`, `agc_imu_standby`, `agc_job_attitude_maneuver_enter`, `agc_job_calc_delta_v_body_csm`, `agc_job_calc_delta_v_body_lm`, `agc_job_calc_vg_body_csm`, `agc_job_calc_vg_body_lm`, `agc_job_cdh_targeting`, `agc_job_cdh_targeting_other_vehicle`, `agc_job_cm_sm_separation_newmode`, `agc_job_coarse_align`, `agc_job_coarse_align_coarsed`, `agc_job_coarse_align_fineonly`, `agc_job_crew_defined_maneuver_gomove`, `agc_job_csi_targeting`, `agc_job_csi_targeting_other_vehicle`, `agc_job_csm_imu_realign_landing_site`, `agc_job_earth_orbit_insertion_monitor`, `agc_job_external_delta_v_reftest`, `agc_job_go_idle_fix`, `agc_job_go_idle_fix_reflash`, `agc_job_gyro_torquing_display`, `agc_job_gyro_torquing_pulse`, `agc_job_gyro_torquing_pulsed`, `agc_job_gyro_torquing_return`, `agc_job_imu_realign`, `agc_job_imu_realign_option`, `agc_job_imu_realign_option_display`, `agc_job_jet_accel_reset`, `agc_job_jet_accel_update`, `agc_job_lm_fine_align`, `agc_job_lm_imu_realign_landing_site`, `agc_job_rcs_burn_csm`, `agc_job_rcs_burn_lm`, `agc_job_thrust_monitor_csm`, `agc_job_thrust_monitor_lm`, `agc_lm_dap_jet_accel_update`, `agc_lm_job_aps_abort`, `agc_lm_job_dps_abort`, `agc_lm_job_landing_confirmation`, `agc_lm_job_landing_confirmation_flash`, `agc_lm_job_powered_ascent_burn`, `agc_lm_job_powered_ascent_flash`, `agc_lm_powered_ascent_init`, `agc_lm_start_approach_phase`, `agc_phase_change_program_klean`, `agc_phase_idle_klean`, `agc_rendz_goto_terminate_tracking`, `agc_task_braking_phase_zoom`, `agc_task_dps_burn_zoom`, `agc_task_start_thrust_monitor`, `agc_task_wake_cm_sm_separation`, `change_program_refuse`, `csm_tvc_gains`, `entry_ballistic`, `entry_final_phase`, `entry_post_0_05g`, `entry_preparation_mm`, `entry_preparation_newrnvn`, `entry_preparation_tff`, `entry_upcontrol`, `is_cislunar_midcourse_navigation`, `landing_site_time`, `ll_start_manual_landing`, `ll_start_rate_of_descent_landing`, `lm_dap_ones_to_twos`, `lm_dap_switch_curves`, `noun_clock_time`, `orbit_parameter_display_hpermin`, `rcs_burn`, `test_display_clock_time`, `test_load_component_one`, `test_load_three_components`

## Duplicate definitions that become one name (phase 2 must keep exactly one)

The paused lanes declared the same job/task twice under different names
(old pending block in src/guidance/guid_pending.h, first-attempt name in
src/exec/exec_pending.h or src/csm/entry_pending.h, usually with different
numeric ids). After the rename they share a name, so phase 2 must delete the
guid_pending.h block (its header says to do so at integration) and keep the
exec/entry definition:

| new name | definitions now |
|---|---|
| `AGC_JOB_PREBURN_STATE_VECTOR` | guid_pending.h `AGC_JOB_P40S_SV`; exec_pending.h `AGC_JOB_SPS_BURN_S_SV` |
| `AGC_JOB_THRUST_MONITOR_RESET` | guid_pending.h `AGC_JOB_P47BODY`; exec_pending.h `AGC_JOB_THRUST_MONITOR_BODY` |
| `AGC_JOB_TIMEBURN` | `AGC_JOB_S40_13` in both guid_pending.h and exec_pending.h (pre-existing) |
| `AGC_JOB_VHHDOT` | guid_pending.h `AGC_JOB_P11SC`; entry_pending.h `AGC_JOB_VHHDOT` |
| `AGC_TASK_ENGINE_FAIL_RCS_DAP` | guid_pending.h `AGC_TASK_V97TRCS`; exec_pending.h `..._PROCEDURE_TRCS` |
| `AGC_TASK_ENGINE_FAIL_REENABLE` | guid_pending.h `AGC_TASK_R40ENABL`; exec_pending.h `AGC_TASK_ENGINE_FAIL_ENABL` |
| `AGC_TASK_ENGINE_FAIL_REIGNITE_PREP` | guid_pending.h `AGC_TASK_V97ETASK`; exec_pending.h `..._PROCEDURE_ETASK` |
| `AGC_TASK_ENGINE_FAIL_REIGNITE_TRIM` | guid_pending.h `AGC_TASK_V97E40_6`; exec_pending.h `..._PROCEDURE_E40_6` |
| `AGC_TASK_ENGINE_FAIL_RESUME` | guid_pending.h `AGC_TASK_V97PTASK`; exec_pending.h `..._PROCEDURE_PTASK` |
| `AGC_TASK_ENGINE_FAIL_SHUTDOWN` | guid_pending.h `AGC_TASK_V97TTASK`; exec_pending.h `..._PROCEDURE_TTASK` |
| `AGC_TASK_GIMBAL_TRIM_RESTART` | `AGC_TASK_PRE40_6` in both headers (pre-existing) |
| `AGC_FLAG_GROUND_TRACK_BASE_VECTORS`, `AGC_FLAG_ORBIT_DISPLAY_MOON` | one per vehicle flag header (as before; never both in one TU) |

`AGC_JOB_P40` (shared CSM/LM id) maps to `AGC_JOB_DPS_BURN` because only LM
code references it; CSM P40 registers `agc_job_sps_burn` directly.

## Not resolved from the source (lead decision)

* `agc_rendz_s17_1`, `s17_2`, `agc_rendz_s32_33_1`, `agc_rendz_s32_33_x`,
  `agc_rendz_s33_34_1`, `agc_rendz_s34_35_1`, `agc_rendz_s34_35_2`: planned
  names in docs/modules/prog-rendz.md only (no code yet). prog-rendz must name
  them per NAMING.md when it resumes; not mapped here.
* `AGC_FLAG_S32_1F1..F3B`, `agc_erase_s61dt`, `RST_E_S61DT`: AGC flag-word /
  erasable labels that contain an S-section number; kept under the
  `AGC_FLAG_<AGCNAME>` / `agc_erase_<agcname>` conventions of _common.md.
* `CMPG2205` (DOWNLINK_LISTS sublist label) is renamed because the AGC label
  itself embeds P22; its siblings `CMENTR05`, `CMPOWE07` are kept.
* Dangling references in paused lanes (no declaration anywhere, 42 in the
  inventory, e.g. `agc_job_lm_fine_align_k`, `agc_job_csm_imu_realign_landing_site`,
  `AGC_TASK_LANDING_RADAR_MONITOR`) are mapped to final names but still need a
  definition from their lane.

## Baseline compile (phase 1, before any rename)

`gcc -std=c11 ... -Werror -Iinclude -c` over every .c in src/ and tests/:
156 compile, 21 fail. The failing files (most fail because the stopped rename
changed headers but not the paused lanes) are:
src/csm/dap/rcs_filter.c, src/csm/external_delta_v_csm.c, src/csm/p37.c,
src/csm/s_band_antenna_csm.c, src/csm/servicer_csm.c, src/csm/sps_burn.c,
src/csm/sps_burn_clock.c, src/csm/sps_burn_ignition.c, src/csm/sxtmark.c,
src/guidance/guid_state.c, src/guidance/p30.c, src/io/service.c,
src/lm/dps_burn.c, src/lm/ignition.c, src/lm/lunar_landing.c,
src/lm/s_band_antenna_lm.c, src/lm/servicer_lm.c, src/programs/align_common.c,
src/programs/imu_realign.c, src/programs/rendz_flags_csm.c,
src/programs/rendz_flags_lm.c.

## Earlier lane renames (dap-lm, already applied by dap-lm itself)

| Old identifier | New identifier | Cross-lane callers to update |
|---|---|---|
| agc_lm_dap_1_accs | agc_lm_dap_jet_accel_update | src/lm/servicer_lm.c:370 |
| agc_job_1_accjob / AGC_JOB_1_ACCJOB | agc_job_jet_accel_update / AGC_JOB_JET_ACCEL_UPDATE | dispatch.c (when registered) |
| agc_job_1_accset / AGC_JOB_1_ACCSET | agc_job_jet_accel_reset / AGC_JOB_JET_ACCEL_RESET | dispatch.c (when registered) |
| lm_dap_1_accont, lm_dap_1sto2s, lm_dap_pjetslec6, lm_dap_purgency4 (lane-private) | lm_dap_switch_curves, lm_dap_ones_to_twos, lm_dap_pjetslec_pair, lm_dap_purgency_law | none |
