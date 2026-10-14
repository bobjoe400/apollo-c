# guid-landing -- lane history

Process diary of the guid-landing lane, moved verbatim out of [docs/modules/guid-landing.md](../../docs/modules/guid-landing.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-14 (first and last commit touching the note).

## Progress

Infrastructure:
- DONE include/agc/agc_landing_pending.h (all not-yet-existing shared symbols, one place)
- DONE include/agc/lm/landing.h (lane API + state structs; may grow)
- DONE src/lm/landing/state.c (state, PHASCHNG/FASTCHNG/DV/MP helpers)
- DONE landing_pending.h reconciled 2026-10-09 with agc_guid.h, agc_lm_dap.h, agc_integ.h, agc_exec.h, exec_pending.h (job ids moved to 0x0F00/0x0F80)

Per file (TODO unless marked):
- DONE throttle.c: agc_lm_throttle, agc_lm_throttle_measured (AFDUMP..DOPIF), thr_region (WHERETO..FLATOUT2), thr_doit, thr_fweight, agc_lm_throttle_flatout, agc_lm_throttle_whatout, agc_lm_massmult
- DONE findcduw.c: agc_lm_initcduw, agc_lm_findcduw (+ _m2 = FINDCDUW -2), fc_fltrsub, fc_unwctest, fc_dcmcl, fc_nb2cdusp, fc_dvbycosm, agc_lm_arctrgsp, agc_lm_sparcsin, agc_lm_limitsub, agc_lm_onesto2s, fc_noattcnt, ALARMMGA in fc_gimbal_changes
- DONE guidance.c: agc_lm_job_lunland, agc_lm_guidpass (+ldg_front = GUILDRET+2..BRSPOT2, ldg_quadratic), agc_lm_ttfincr, agc_lm_calcrgvg, agc_lm_rgvgcalc, agc_lm_ttf8cl, agc_lm_rootpsrs, agc_lm_quadguid (+ldg_quadcoef), agc_lm_afccalc (+ldg_afclimit), agc_lm_cgcalc, ldg_extlogic, ldg_exgsub, EXBRAK, ldg_exnorm, agc_lm_steer_exit (STEER?/EXVERT/RATESTOP/GDUMP1), agc_lm_dispexit, agc_lm_job_braking_phase_display, agc_lm_job_approach_phase_display, agc_lm_job_approach_phase_proceed, agc_lm_vertguid (P65VERT), agc_lm_start_approach_phase, agc_lm_start_velocity_nulling, agc_lm_tdispset
- DONE lunar_landing.c: agc_lm_job_braking_phase (+FLAGORGY), ll_ignalg_start/ll_ignalg_prec (IGNALG, LEMPREC wait job IGNALGP), ll_ignaloop/ll_ign_guidance (IGNALOOP, INTSTALL wait job IGNALOOP), ll_ddum (DDUMCALC), ll_ddumgood (DDUMGOOD/R60INIT), agc_lm_job_ignalgrt (IGNALGRT/ASTNCLOK), agc_lm_job_astnret, agc_lm_job_braking_phase_burn_attitude/3/4, agc_lm_guildenstern (R13 incl. STARTP66/STRTP66A/VRTSTART/STARTP67/STABL?/P66NOW?/RESTART?), agc_lm_redesig (+ll_redes1), agc_lm_rate_of_descent_vertical, agc_lm_task_rodtask, agc_lm_job_rodcomp (ll_rod_pipas/ll_rod_state/ll_rod_afc), ll_vhorcomp, agc_lm_manual_landing_vertical, agc_lm_pitfall, agc_lm_task_redesmon (+COUNT'EM, RESETRPT), agc_lm_descbits, agc_lm_job_landing_confirmation / _landing_confirmation_flash / _landing_confirmation_proceed (LANDJUNK)
- DONE aborts.c: agc_lm_task_abort_monitor, agc_lm_abort_monitor_resume (FLASHH?/FLASHV?/10,11/P71NOW?/P70NOW?), ab_abrtjask (CONTABRT/ABRTJASK), ab_legal (LEGAL?/ABORTALM), agc_lm_job_dps_abort/p71/p70a/p71a, agc_lm_job_goabort, ab_dps_abort_init, ab_bothpoly (BOTHPOLY..YOK), ab_upthrot, agc_lm_job_abort_auto_control_check (UPTHROT1 -3), agc_lm_job_upthrot1 (+GRP4OFF), ab_aps_abort_return, agc_lm_job_aps_abort_oldtime (OLDTIME), agc_lm_tgocomp, agc_lm_throtup
- DONE ascent.c: agc_lm_guidinit, agc_lm_comminit, agc_lm_powered_ascent_init, agc_lm_ycomp, agc_lm_logsub, as_vgvect (ASCENT state/GEFF/VGVECT/VGBODY), as_maineng, as_t2test (T2TEST/LOGSUB/NORATES/CONST), as_rates (RATES/CHKBMAG/PROK), as_cmponent (CMPONENT/NO-ATP/AIMER), as_mainline (MAINLINE/ANG1CHEK/OFFROT/CLRXFLAG), as_exitvr, as_checkalt (CHECKALT/CHECKYAW/KEEPVR/EXITVR1), as_keepvr1, as_ascterm/as_ascterm1 (ASCTERM..4), as_engoff, agc_lm_ascent, agc_lm_job_atmag, agc_lm_task_engoff1, agc_lm_job_cutoff/_cutoff5/_cutoff1/_cutoff2/_termasc, agc_lm_job_powered_ascent/_powered_ascent_flash/_powered_ascent_proceed/_powered_ascent_at_ignition/_newload/_newload2, agc_lm_powered_ascent_return, agc_lm_job_powered_ascent_burn
- DONE displays.c: agc_lm_landisp (LANDISP/ALTROUT/ALTOUT dispatch), ld_disindat (DISINDAT), agc_lm_task_intlze (INTLZE), ld_speedrun (SPEEDRUN to VARDELAY), agc_lm_task_speedrun2 (SPEEDRUN after the pause, GET22/32, LATFWDV), agc_lm_vmonitor + ld_vmon_delta (VMONITOR..ZEROLSTY, checked branch by branch against pp. 904-907), ld_altrout (ARCOMP), ld_altout/ld_extrapolate (OLDDATA/NEWDATA/ZERODATA), ld_dataout (DATAOUT), ld_disprset (DISPRSET/ABORTON)
- DONE tests/lm/landing.c (compiles; see Tests written)
- DONE 2026-10-10 reconciliation after wave 1: ASTNRET now sets agc_align_ram()->qmaj and calls agc_job_lm_fine_align_recheck (agc_align.h R51K/R51P63) instead of a private agc_align_fine_align_recheck / agc_imu_fine_align_recheck; 1JACCQ/1JACCR taken from agc_lm_dap.h, TRKMKCNT from agc_programs.h; agc_imu.h signatures used as declared (agc_imu_status_check, agc_imu_attitude_maneuver, agc_imu_set_pointvsm/_scaxis, agc_imu_comp_get); all displays go through agc_dsky_display()

## Open questions

Resolved by review-guid-landing (2026-10-10):
- THRUST / ALTM / CDUTCMD / CDUSCMD: accessors exist in include/agc/core/state.h (ALTM as a raw uint16_t word, as asked).
- TBASE2 for LANDISP: no setter needed; agc_phase_newphase(2, current phase, true) sets TBASE2 to the present time, which is what CS TIME1 / DXCH TBASE2 encodes in the exec's TBASE model (DEVIATION note kept in displays.c).

## Requested core additions

All satisfied (checked 2026-10-10 by review-guid-landing): agc_erase_set_thrust / _altm / _cdutcmd and agc_erase_tem (core/state.h), agc_align_refmf (programs/alignment.h), agc_navaux_set_latlong (nav/auxiliary.h), agc_rendz_flip / agc_rendz_clradmod (programs/rendezvous.h), agc_erase_m22 / _m32 (io/service.h); alarms 01406/01410/01412 are lane constants in include/agc/lm/landing.h. include/agc/agc_landing_pending.h no longer exists.

## Requested jobs-tasks

Provisional ids in landing_pending.h (jobs 0x0F01-0x0F25, tasks 0x0F81-0x0F86; GOABORT/P70A/P71A/R10,R11/ZOOM from exec_pending.h). Labels:
jobs P63LM IGNALGRT ASTNRET P63SPOT3 P63SPOT4 P64CEED P64DISPS RODCOMP
LANDJUNK LANDJUNK2 GOABORT P12LM P12LM2 NEWLOAD NEWLOAD2 CUTOFF CUTOFF1
CUTOFF2 TERMASC LUNLAND ATMAG; tasks RODTASK REDESMON ENGOFF1 ABORT_MONITOR INTLZE
SPEEDRUN2.

