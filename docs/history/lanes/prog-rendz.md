# prog-rendz -- lane history

Process diary of the prog-rendz lane, moved verbatim out of [docs/modules/prog-rendz.md](../../modules/prog-rendz.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-14 (first and last commit touching the note).

## Progress

* DONE infrastructure: programs/navigation/pending.h (pending externals + provisional ids
  0x1100), internal.h, glue.c (adapted to the real agc_nav.h,
  agc_integ.h, agc_dsky.h, agc_exec.h), flags_csm.c / _lm.c
* DONE common.c: SELECTMU agc_rendz_selectmu, VECSHIFT
  agc_rendz_vec_to_body, SHIFTR1 agc_rendz_shiftr1, PERIAPO1/PERIAPO/SETRAD
  agc_rendz_periapo1/agc_rendz_periapo, PRECSET/LEMSTORE/CSMSTORE/ACTIVE/
  PASSIVE rz_precset/rz_store_*, INTINT rz_intint, S34/35.1, LOMAT,
  GET.LVC, GET+MGA, AVFLAGA/P rz_avflag, P20FLGON, TRKMKCNT accessors
* DONE csi.c: ROTATE agc_rendz_rotate, ADVANCE agc_rendz_advance,
  CSI/A..SCNDSOL agc_rendz_csi_a, CDHMVR agc_rendz_cdhmvr, S32/33.X
  agc_rendz_s32_33_x, S32/33.1 agc_rendz_s32_33_1
* DONE tpi.c: S33/34.1 agc_rendz_tpi_time_search(+_cont), S34/35.2
  agc_rendz_tpi_required_velocity, S34/35.3 agc_rendz_tpi_crew_overwrite,
  S34/35.4 agc_rendz_tpi_los_delta_v, P34/P74E agc_rendz_tpi_compute
* DONE initvel.c: INITVEL agc_rendz_initvel
* DONE display.c: VNPOOH rz_vnpooh, DISPLAYE, VN1645 rz_ttogo_display
  (+ KILCLOCK/N45PROC/CLUPDATE jobs), S34/35.5 rz_tpi_final_displays,
  DISDVLVC, P3XORP7X, ALMXIT/611 rz_alarm_flash
* NAMING (docs/NAMING.md): all identifiers renamed to descriptive stems
  (map in "Renames" below); job ids in programs/navigation/pending.h are descriptive.
* DONE csi_cdh_targeting.c: P32/P72, P33/P73 drivers (agc_job_csi_*, agc_job_cdh_*)
* DONE tpi_tpm_targeting.c: P34/P74, P35/P75 drivers (agc_job_tpi_*, agc_job_tpm_*); ATIGINC/PTIGINC added to agc_rendz_t
* DONE orbit_parameter_display.c: R30 V82 (SR30.1 agc_orbit_params_compute, MAXCHK, body select, V82 jobs, TICKTEST task, B5OFF/ENDEXT jobs)
* DONE initvel.c HAVEGUES agc_rendz_havegues (requested by guid lane), COGA kept between calls
* DONE tests/programs/rendezvous.c (17 checks pass when linked with a scratch build of the tree)
* DONE target_delta_v.c: P76 (agc_job_target_delta_v*, COMPMAT agc_target_delta_v_lv_to_ref); VHFCNT, MRKBUF2 accessors in common.c
* DONE ground_track_determination.c: P21 (agc_job_ground_track_*, P21VSAVE agc_ground_track_determination_flight_path); rz_dsptem1_dp in glue.c
* DONE stable_orbit.c: P38/P78, P39/P79 (agc_job_stable_orbit_*, DELTTIME agc_rendz_stable_orbit_deltt); S3435.25 split out of tpi.c as agc_rendz_intercept_velocity
* DONE rule-7 pass (no (void) casts of agc_ calls; rz_flag_put/rz_vflag_set/rz_avflag/rz_tracking_flags_on now void)
* DONE out_of_plane_display.c: R36 V90 (agc_rendz_out_of_plane_compute, agc_job_out_of_plane_*)
* DONE rendezvous_parameter_display.c: R31 V83 both vehicles + CM R34 (COMPDISP agc_rendezvous_parameter_display_compdisp, agc_job_rendezvous_params_*)
* DONE src/csm/navigation/tpi_search.c (P17/P77: S17.1, S17.2, TRANSANG, jobs) -- verified complete on resume
* DONE lambert_aimpoint_guidance_lm.c (P31) -- verified complete on resume
* DONE src/lm/radar/radar.h (private LM radar interface; RADMODES bit names)
* DONE src/lm/radar/moding.c: MAGSUB, RRLIMCHK agc_rr_limchk, SETTRKF agc_rr_settrkf,
  FLIP agc_rendz_flip, CLRADMOD agc_rendz_clradmod, RRTURNON/RRZEROSB tasks,
  RRZERO agc_rr_zero, DORREPOS/REPOSRPT, REMODE, RR1AXIS (bounded loop),
  RROUT, ENDRADAR/RGOODEND/RDBADEND, RADNOOP
* DONE src/lm/radar/designate.c: RRDESSM agc_rr_designate_sm, RRDESNB
  agc_rr_designate_nb, STARTDES, BEGDES/STDESIG/DESLOOP, DODES
  agc_job_rr_designate_command, GOTOV56 agc_rendz_goto_terminate_tracking
* DONE src/lm/radar/read.c: INITREAD + lead-ins (agc_radar_rrrange/rrrdot/lralt/lrvel),
  RADAREAD agc_rendz_radar_read, LRPOS2 + scan tasks, R12LITES/RADLITES
* DONE src/lm/radar/powered_flight_designate.c (R29: agc_radar_powered_flight_designate,
  R29REM0J, PREPOS29, BEGDES29/R29DLOOP, R29DODES, R29READ/R29RDJOB)
* DONE src/lm/radar/leadin.c (RADSAMP agc_task_radsamp, DORSAMP, VARADAR, RSTACK)
* DONE src/lm/radar/designate.c additions: agc_rr_designate_gimbal (V41 request), TANG accessors
* DONE src/lm/navigation/navigation.c: PROG20/PROG22 (LM), ORBCHGO..ORBCHG3, PROG20A,
  P20LEM1, 526ALARM, P20LEMA/B/B3-B6/C/C3/D + tasks P20LEMC1/D1, P20LEMWT
  (lmnav_wait), UPPSV, LPS20.1 lmnav_los, LPS20.2, P25 (PROG25, P25LEM1/2)
* DONE src/lm/navigation/tracking_attitude.c: R61LEM/R65LEM (lmnav_tracking_attitude, jobs)
* DONE src/lm/radar/acquisition.c: R21LEM (all), R23LEM, R24LEM + LRS24.1
* DONE src/lm/navigation/tracking_data.c (R22LEM, LRS22.1/2, N49DSP + N49FLAG wait, R22RSTRT)
* DONE src/lm/navigation/measurement_update.c (LSR22.3/.4, GETULC, RADARANG, MXMYMZ,
  LGCUPDTE (INCORP1 + RMAX/VMAX check), INCORP2 glue, WLINIT, INTGRCAL)
* DONE common.c: agc_rendz_nav_pad (pad loads), X789 accessors
* DONE w_matrix_monitor.c (V67 both vehicles: V67CALL, V67WW, V6N99PRO/INP)
* DONE rz_clear_ovf, rz_phase, rz_phase_job, rz_prio_alarm glue helpers
* DONE src/lm/navigation/navigation.c renamed from rendezvous_navigation_lm.c (file-name rule:
  the folder name is not repeated); LM entry is agc_job_rendezvous_navigation_lm
* DONE common.c: MRKBUF1/MRKBUF2 storage (agc_erase_mrkbuf / set; owned here),
  rz_incorp2 (INCORP2 glue, both vehicles), rz_word_value
* DONE src/csm/navigation/tracking_attitude.c: R61CSM agc_rendz_tracking_attitude_csm, CRS61.1,
  CRS61.2(A), the R63 subroutine (desired CDUs), R61CNTR accessors
* DONE src/csm/navigation/navigation.c: P20/P22 vehicle dispatch (agc_job_rendezvous_navigation,
  agc_job_orbital_navigation), PROG20 CSM, PIKUP20, P20R52JB, SETINTG rz_setintg,
  WAITONE/CONTCHK/REDOR22 rz_waitone, R21CSM (V57), R23CSM (V54), LANDMARK
* DONE src/csm/navigation/tracking_data.c: R22 CSM (REND1..REND18, RENDISP), BVECTORS, GETUM,
  SHIFTNDX, CMPOS/LMPOS, INITIALW, RANGERD, VHFREAD agc_rendz_vhf_read
* DONE V89 CSM (R63: agc_job_rendezvous_final_attitude_csm..., src/csm/navigation/tracking_attitude.c)
* DONE display.c agc_rendz_ttogo_display_start; noun_data.c agc_rendz_noun_read/write
* DONE src/csm/navigation/navigation.c LLASRD/LLASRDA (agc_rendz_llasrd/llasrda); P21 now stores
  LAT LONG ALT (agc_navaux_set_latlong)
* DONE src/csm/navigation/orbital_navigation.c: PROG22 (CSM), V06N45, PROG22A/B/C, S22N7071 code
  check (S22DSPPA/S22DSPP/S22ABDE), S22LSITE, OBTAINLL/P22SUBRB/N89ERRX, CALLR52,
  DE-GR-50, DOV5N71, P22OVER
* DONE src/csm/navigation/landmark_navigation.c: S22.1 (marks to SVMRKDAT, FIG2EXIT/S2INTS1,
  DMPINTEG, unknown landmark U/E/W72-W78, known landmark S22BOX22, S22D=9,
  S22BOX12/32/42, N49, S22F244, N89, RLS), S22F2410, S2231X13, 9DWTO6DW, ROWDOT
* DONE common.c rz_bvectors (BVECTORS, shared CSM R22/S22.1)
* DONE src/csm/navigation/cislunar_navigation.c: R57 (rz_optics_calibration, R57A/MARKDISP/R57B/
  ENDR57), P23 (P23.00 .. P23.85, LDPLANET, V94ENTER, POINTAXS, RECT.1, HORIZ,
  R23.55 .. R23.7), TRUNBIAS and HORIZON storage (noun 70/71 R3 in noun_data.c),
  N49DISP shared with R22 CSM (rz_set_csm_n49disp)
* DONE docs/NAMING.md rows for the new stems (radar_search, manual_acquisition,
  optics_calibration, w_matrix_monitor, ...)
* MODULE COMPLETE: all files compile with the pedantic flag set; check_rules 0 hard
  violations (rule-5 assertion-density warnings only)

## Requested core additions

All are declared in src/programs/navigation/pending.h with the owning lane's
spelling, to be moved to include/agc when the owner publishes them:
* sys-misc: agc_svc_delayjob, agc_erase_extvbact / set (EXTVBACT);
  R04/R77 storage accessors (RSAMPDT, RTSTDEX, RTSTBASE, RTSTMAX, RTSTLOC,
  RFAILCNT).
* exec-sched: agc_waitlist_kill (KILLTASK).
* state-flags: agc_erase_rnrad (RNRAD counter), agc_erase_set_cdutcmd
  (CDUTCMD/CDUSCMD); RADMODES must be FLGWRD12 (one storage word).
* nav-aux: agc_navaux_get_latlong / agc_navaux_set_latlong (LAT LONG ALT,
  LANDLAT EQUALS LAT on the CM).
* Published by this module for other lanes (include/agc/programs/rendezvous.h):
  agc_rr_limchk, agc_rr_settrkf, agc_rr_zero, agc_rr_designate_gimbal,
  agc_rendz_radar_read, agc_rendz_vhf_read, agc_rendz_flip,
  agc_rendz_clradmod, agc_radar_lralt/lrvel/lrpos2/samplsum/
  landing_radar_lights/powered_flight_designate, agc_rendz_goto_terminate_tracking,
  agc_rendz_tracking_attitude_csm, agc_rendz_llasrd/llasrda,
  agc_erase_landmark/set, agc_erase_horizon/set, agc_erase_trunbias,
  agc_erase_mrkbuf/set, agc_erase_tang/set, agc_erase_rstack,
  agc_rendz_ttogo_display_start, agc_rendz_noun_read/write, agc_rendz_nav_pad.

* agc_state.h: `agc_vec_t agc_erase_rls(void);` landing site vector RLS
  (moon-fixed, m 2^27); used by SETRAD/PERIAPO, R30 MOONPAD, P22.
* Interfaces assumed from not-yet-existing headers: see programs/navigation/pending.h
  (conic TIMETHET/TIMERAD/LAMBERT/APSIDES, CSM/LEM PREC/CONIC, INTEGRVS,
  INTSTALL, TFF routines, LAT-LONG, DSKY flash displays, GOTOPOOH, ENDEXT,
  DELAYJOB, CDUTRIG/*NBSM*/SXTNB).

## Requested jobs-tasks

Provisional ids in src/programs/navigation/pending.h (prototypes in
include/agc/programs/rendezvous.h before the "end of job entry points" comment): jobs
0x1100-0x1217 (plus 0x036B W_MATRIX_MONITOR, 0x036C/0x036D V57/V54 with
the values of src/io/verbs/pending.h); tasks 0x1101-0x111C plus RRTURNON
0x312 and DORREPOS 0x313 (the values of sysmisc_ext.h). The lead maps them
to agc_jobs.h members and dispatch.c cases. Verb entries: V94 ->
AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_POINT; V67 -> W_MATRIX_MONITOR;
V57/V54/V89 CSM -> the _CSM jobs (LM V89 is prog-align's R63).

## Open questions

* CM 1DPB2 is transcribed "2DEC 1.08-2"; LM has "1.0 B-2". Used 1.0 B-2.
* RADMODES EQUALS FLGWRD12 in the AGC; sys-misc keeps a separate static
  RADMODES. One storage word is needed (state-flags).
* LUNDESCH -> NODESSM performs a mode invert that looks spurious; kept
  faithful.
* V57 / V54 / V89 dispatch by vehicle (CSM jobs here, LM R63 in
  prog-align).
* tpi.c push cell 0D (see Deviations).

## Lessons

* Interpretive relative offsets (+11) are octal; PDDL/PDVL with no operand
  exchange with the push-down top; NORM stores -shifts and indexed shift
  counts are addr - C(X).
* Keep TRUNION/MRKBUF words raw and convert (MSU) at the point of use.
* Split a job where the AGC re-reads the clock, so a BUSY resume does not
  change MARKTIME (V94ENTER / POINTAXS).

