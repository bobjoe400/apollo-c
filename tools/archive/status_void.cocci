// status_void.cocci -- rule 7 (docs/CODING_STANDARD.md): callers of the
// functions that the status-handling lane made void (they cannot fail when
// called with valid arguments; bad arguments are caught by AGC_ASSERT
// inside). The lists below are the full record of those functions.
//
//   spatch --sp-file tools/archive/status_void.cocci --in-place --dir src \
//          --include-headers -I include
//
// 1. `(void) f(args);`  ->  `f(args);`
// 2. `return f(args);`  ->  `f(args); return;` in a void function, else
//    `f(args); return AGC_STATUS_OK;`
//    (ALARM family: `return AGC_STATUS_ALARM;`, the status the caller's
//    caller saw before)
// Assignments such as `st = f(args);` were fixed with the compiler's help
// ("void value not ignored"): the call stays, dead checks of st go.

@voidcast@
identifier f =~ "^(agc_flag_set|agc_flag_clear|agc_flag_invert|agc_flagword_set|agc_time_set_cs|agc_chan_write|agc_chan_or|agc_chan_and|agc_phase_change|agc_phase_change_var|agc_phase_change2|agc_phase_kill|agc_phase_newphase|agc_exec_prio_change|agc_rupt_set_t5_task|agc_rupt_set_t6_task|agc_rupt_set_time|agc_flag_put|agc_conic_mirror_flag_|align_phase_job|agc_lm_serv_quikfaz5|agc_erase_set_thetad|agc_integ_setiflgs|agc_lm_noullage|agc_lm_dap_stoprate|agc_guid_quictrig|agc_lm_nullclok|agc_rendz_clradmod|agc_lm_tgocomp|agc_lm_altchk|agc_dsky_set_nvword1|agc_csm_tvczap|agc_csm_spsoff|agc_csm_setmindb|agc_imu_noattoff|agc_imu_zeroicdu|agc_imu_set_pointvsm|agc_lm_phaschng|agc_dsky_falton|agc_radar_landing_radar_lights|agc_lm_tmptospt|agc_guid_fleshpot|agc_guid_copycycle|agc_lm_enginof3|agc_lm_initcduw|agc_lm_onullage|agc_imu_gyro_buf_set|agc_svc_set_extvbact|agc_dsky_clearmrk|agc_telem_select_list|agc_earth_orbit_insertion_monitor_attitude_errors|agc_csm_velocity_altitude_rate|agc_csm_setmaxdb|agc_integ_csm_resume_idle_integration|agc_integ_thisvint|agc_integ_csm_set_amoonflg|agc_integ_minirect|agc_imu_hw_set_gyrocmd|agc_exec_job_sleep|agc_erase_set_dsptab|agc_phase_set_tbase|agc_waitlist_kill|agc_integ_grp2pc|agc_lm_dap_zatteror|agc_kalc_stoprate|agc_rendz_flip|agc_lm_enginof4|agc_lm_fastchng|agc_entry_dap_flushjet|agc_lm_throtup|agc_guid_axisrot|agc_guid_cdtrigs|agc_guid_cdutrig|agc_guid_smnb|agc_guid_nbsm|agc_guid_cdu_smnb|agc_guid_trg_smnb|agc_guid_reference_to_body|agc_csm_convert_vg_body|imu_modeexit|agc_imu_rndrefdr|agc_imu_pipfree|agc_imu_pipuse1|agc_imu_setcoars|rd_initread|agc_radar_lr_velocity_beam|agc_radar_lrvel|agc_radar_lralt|agc_lm_comminit|agc_lm_powered_ascent_init|agc_csm_dap_fixcw|agc_csm_dap_massprop|agc_imu_calcgta|agc_csm_dap_zerojet|entry_earrot|gnufaze5|serv_phase|gcomp_write|imu_vflag_set|imu_vflag_clear|ld_imodes33_bits|agc_alarm|agc_prioalarm|agc_exec_varalarm|agc_curtains)$";
expression list es;
@@
- (void) f(es);
+ f(es);

@retvoid@
identifier fn;
identifier f =~ "^(agc_flag_set|agc_flag_clear|agc_flag_invert|agc_flagword_set|agc_time_set_cs|agc_chan_write|agc_chan_or|agc_chan_and|agc_phase_change|agc_phase_change_var|agc_phase_change2|agc_phase_kill|agc_phase_newphase|agc_exec_prio_change|agc_rupt_set_t5_task|agc_rupt_set_t6_task|agc_rupt_set_time|agc_flag_put|agc_conic_mirror_flag_|align_phase_job|agc_lm_serv_quikfaz5|agc_erase_set_thetad|agc_integ_setiflgs|agc_lm_noullage|agc_lm_dap_stoprate|agc_guid_quictrig|agc_lm_nullclok|agc_rendz_clradmod|agc_lm_tgocomp|agc_lm_altchk|agc_dsky_set_nvword1|agc_csm_tvczap|agc_csm_spsoff|agc_csm_setmindb|agc_imu_noattoff|agc_imu_zeroicdu|agc_imu_set_pointvsm|agc_lm_phaschng|agc_dsky_falton|agc_radar_landing_radar_lights|agc_lm_tmptospt|agc_guid_fleshpot|agc_guid_copycycle|agc_lm_enginof3|agc_lm_initcduw|agc_lm_onullage|agc_imu_gyro_buf_set|agc_svc_set_extvbact|agc_dsky_clearmrk|agc_telem_select_list|agc_earth_orbit_insertion_monitor_attitude_errors|agc_csm_velocity_altitude_rate|agc_csm_setmaxdb|agc_integ_csm_resume_idle_integration|agc_integ_thisvint|agc_integ_csm_set_amoonflg|agc_integ_minirect|agc_imu_hw_set_gyrocmd|agc_exec_job_sleep|agc_erase_set_dsptab|agc_phase_set_tbase|agc_waitlist_kill|agc_integ_grp2pc|agc_lm_dap_zatteror|agc_kalc_stoprate|agc_rendz_flip|agc_lm_enginof4|agc_lm_fastchng|agc_entry_dap_flushjet|agc_lm_throtup|agc_guid_axisrot|agc_guid_cdtrigs|agc_guid_cdutrig|agc_guid_smnb|agc_guid_nbsm|agc_guid_cdu_smnb|agc_guid_trg_smnb|agc_guid_reference_to_body|agc_csm_convert_vg_body|imu_modeexit|agc_imu_rndrefdr|agc_imu_pipfree|agc_imu_pipuse1|agc_imu_setcoars|rd_initread|agc_radar_lr_velocity_beam|agc_radar_lrvel|agc_radar_lralt|agc_lm_comminit|agc_lm_powered_ascent_init|agc_csm_dap_fixcw|agc_csm_dap_massprop|agc_imu_calcgta|agc_csm_dap_zerojet|entry_earrot|gnufaze5|serv_phase|gcomp_write|imu_vflag_set|imu_vflag_clear|ld_imodes33_bits|agc_alarm|agc_prioalarm|agc_exec_varalarm|agc_curtains)$";
expression list es;
@@
void fn(...)
{
<...
- return f(es);
+ f(es);
+ return;
...>
}

@retok depends on !retvoid || retvoid@
identifier f =~ "^(agc_flag_set|agc_flag_clear|agc_flag_invert|agc_flagword_set|agc_time_set_cs|agc_chan_write|agc_chan_or|agc_chan_and|agc_phase_change|agc_phase_change_var|agc_phase_change2|agc_phase_kill|agc_phase_newphase|agc_exec_prio_change|agc_rupt_set_t5_task|agc_rupt_set_t6_task|agc_rupt_set_time|agc_flag_put|agc_conic_mirror_flag_|align_phase_job|agc_lm_serv_quikfaz5|agc_erase_set_thetad|agc_integ_setiflgs|agc_lm_noullage|agc_lm_dap_stoprate|agc_guid_quictrig|agc_lm_nullclok|agc_rendz_clradmod|agc_lm_tgocomp|agc_lm_altchk|agc_dsky_set_nvword1|agc_csm_tvczap|agc_csm_spsoff|agc_csm_setmindb|agc_imu_noattoff|agc_imu_zeroicdu|agc_imu_set_pointvsm|agc_lm_phaschng|agc_dsky_falton|agc_radar_landing_radar_lights|agc_lm_tmptospt|agc_guid_fleshpot|agc_guid_copycycle|agc_lm_enginof3|agc_lm_initcduw|agc_lm_onullage|agc_imu_gyro_buf_set|agc_svc_set_extvbact|agc_dsky_clearmrk|agc_telem_select_list|agc_earth_orbit_insertion_monitor_attitude_errors|agc_csm_velocity_altitude_rate|agc_csm_setmaxdb|agc_integ_csm_resume_idle_integration|agc_integ_thisvint|agc_integ_csm_set_amoonflg|agc_integ_minirect|agc_imu_hw_set_gyrocmd|agc_exec_job_sleep|agc_erase_set_dsptab|agc_phase_set_tbase|agc_waitlist_kill|agc_integ_grp2pc|agc_lm_dap_zatteror|agc_kalc_stoprate|agc_rendz_flip|agc_lm_enginof4|agc_lm_fastchng|agc_entry_dap_flushjet|agc_lm_throtup|agc_guid_axisrot|agc_guid_cdtrigs|agc_guid_cdutrig|agc_guid_smnb|agc_guid_nbsm|agc_guid_cdu_smnb|agc_guid_trg_smnb|agc_guid_reference_to_body|agc_csm_convert_vg_body|imu_modeexit|agc_imu_rndrefdr|agc_imu_pipfree|agc_imu_pipuse1|agc_imu_setcoars|rd_initread|agc_radar_lr_velocity_beam|agc_radar_lrvel|agc_radar_lralt|agc_lm_comminit|agc_lm_powered_ascent_init|agc_csm_dap_fixcw|agc_csm_dap_massprop|agc_imu_calcgta|agc_csm_dap_zerojet|entry_earrot|gnufaze5|serv_phase|gcomp_write|imu_vflag_set|imu_vflag_clear|ld_imodes33_bits)$";
expression list es;
@@
- return f(es);
+ f(es);
+ return AGC_STATUS_OK;

@retalarm@
identifier f =~ "^(agc_alarm|agc_prioalarm|agc_exec_varalarm|agc_curtains)$";
expression list es;
@@
- return f(es);
+ f(es);
+ return AGC_STATUS_ALARM;
