# prog-rendz -- rendezvous programs (P17, P20-P25, P31-P39, P72-P79, P76, P21, R30, R31, R36)

## Scope

AGC sources (Comanche055 = CSM, Luminary099 = LM):
P20-P25.agc (both), P32-P33_P72-P73.agc (CSM) / P32-P35_P72-P75.agc (LM),
P34-35_P74-75.agc (both), R30.agc, R31.agc, STABLE_ORBIT.agc, P76.agc,
GROUND_TRACKING_DETERMINATION_PROGRAM.agc (both), TPI_SEARCH.agc (CSM),
GENERAL_LAMBERT_AIMPOINT_GUIDANCE.agc, RADAR_LEADIN_ROUTINES.agc (LM).

C target: src/programs/ (all files, vehicle-only files suffixed _csm/_lm
because the assignment names src/programs as the target directory), public
header include/agc/programs/rendezvous.h section "rendezvous".

Vehicle diff summary (code lines, comments stripped):
* CSI/CDH: identical algorithm. LM adds automatic TCSI (TCSI<0 -> TIMERAD
  to apocenter), uses TMIN (=60000 cs) where CM uses 600SEC (=60000 cs),
  TWOPI vs 2PISC SL2 (same value), CM 1DPB2 transcribed "1.08-2" (read as
  1.0 B-2, as in LM).
* P34/P35: LM sets CENTANG = 130 deg at P34 entry; LM S34/35.5 checks crew
  overwrite of DELVLVC word by word (NTARGFLG, GDT/2); LM INITVEL computes
  MU/A and MUASTEER at exit, stores RTMAG; LM MIDGIM is only GET+MGA. CM
  MAX250 has B-28 (same value 25000 cs).
* R30: LM has no DELRSPL (P11 splash) branch; moon flag AMOONFLG (CM) vs
  MOONTHIS (LM).
* R31: different. CM: STATEXTP/HAVEBASE, R34 (V65 phi) shares it, LM base
  vector code differs (MUNFLAG, SURFFLAG, GETRVN with REFSMMAT conversion).
* STABLE_ORBIT: identical except NN=2 setup placement and DSPTEM1 copy.
* P76: LM saves DELVLVC to DELVOV first; CM clears VHFCNT/MRKBUF2;
  P76SUB1 moon-index differs.
* P21: flag polarity of VINTFLAG test; LM recycles base vector only when
  not SURFFLAG.
* P20-P25: different programs (CM optics/VHF, LM rendezvous radar).

## Design table

Scales: r m 2^29 (2^27 moon body scale), v m/cs 2^7 (2^5), t cs 2^28,
angles revolutions 2^0, unit vectors 2^1, mu m^3/cs^2 2^36.

| AGC label (file, page) | inputs -> outputs | C function | file |
|---|---|---|---|
| SELECTMU (P34-35 p.494) | CMOONFLG -> RTX1 RTX2 RTMU RTSR1/MU | agc_rendz_selectmu | common.c |
| VECSHIFT / SHIFTR1 (p.499-500) | earth-scale <-> body-scale | agc_rendz_vec_to_body / agc_rendz_shiftr1 | common.c |
| PERIAPO / PERIAPO1 / SETRAD (p.495-497) | r,v -> rapo hapo rper hper ecc | agc_rendz_periapo1 | common.c |
| PRECSET / LEMSTORE / CSMSTORE / ACTIVE / PASSIVE (p.498) | TDEC1 -> RACT3 VACT3 RPASS3 VPASS3 | rz_precset | common.c |
| INTINT / INTINT2C / INTINT3P / GOINT (p.479, 677) | r v t1 t2 conic? -> RATT VATT | rz_intint | common.c |
| S34/35.1 (p.474) | RACT3 RPASS3 VACT3 -> ULOS UNRM | agc_rendz_s34_35_1 | common.c |
| LOMAT (p.479) | UNRM RACT3 -> LV matrix | agc_rendz_lomat | common.c |
| MIDGIM / GET+MGA / GET.LVC (p.492-493) | dV -> +MGA or DELVLVC | agc_rendz_get_mga / agc_rendz_get_lvc | common.c |
| AVFLAGA/P, P20FLGON, VNPOOH, VN1645, COMPTGO, P3XORP7X | flags / displays | rz_avflag, rz_tracking_flags_on, job chain | common.c, display.c |
| ADVANCE / ROTATE (P32 p.675-676) | TIG -> UP1 R/V ACT1,2 PASS1,2 | agc_rendz_advance / agc_rendz_rotate | csi.c |
| CSI/A..CSINEXT4, SCNDSOL, ALMXIT (p.663-674) | -> DELVCSI DELVEET1 TCDH T1TOT2 T2TOT3 POSTCSI POSTCDH | agc_rendz_csi_a | csi.c |
| CDHMVR (p.680-682) | RACT2 VACT2 RPASS2 VPASS2 -> VACT3 DELVEET2 DIFFALT | agc_rendz_cdhmvr | csi.c |
| S32/33.1, S32/33.X, DISDVLVC (p.662,678,679) | DELVEET -> DELVSIN DELVSAB DELVLVC | agc_rendz_s32_33_1 / _x | csi.c |
| P32/P72, P33/P73 drivers (p.652-660) | DSKY chains | agc_prog_csi_targeting / agc_prog_cdh_targeting + jobs | csi_cdh_targeting.c |
| S33/34.1 ELCALC..TIMEX (p.469-473) | iterate TTPI / ELEV | agc_rendz_s33_34_1 | tpi.c |
| S34/35.2/.3/.4/.5 (p.475-480) | Lambert / LV / overwrite | agc_rendz_s34_35_2.. | tpi.c |
| INITVEL (p.486-491) | RINIT VINIT RTARG DELLT4 -> VIPRIME VTPRIME DELVEET3 RTARG | agc_rendz_initvel | initvel.c |
| P34/P74, P35/P75 drivers | DSKY chains | agc_prog_tpi_targeting / p35 | tpi.c |
| R36 (p.501-504) | -> RANGE(Y) RRATE(YDOT) RTHETA(PSI) | agc_rendz_out_of_plane_compute, agc_verb_out_of_plane_display | rendz_out_of_plane.c |
| V82CALL, SR30.1, MAXCHK (R30 p.514-523) | RONE VONE -> HAPOX HPERX TFF -TPER | agc_orbit_parameter_display_compute, agc_orbit_parameter_display_maxchk, agc_verb_orbit_parameter_display | orbit_parameter_display.c |
| R31 V83 COMPDISP / STATEXTP (R31) | -> RANGE RRATE RTHETA | agc_rendezvous_parameter_display_compdisp, agc_verb_rendezvous_parameter_display | rendezvous_parameter_display.c |
| P38/P39 (STABLE_ORBIT) | | agc_prog_stable_orbit_rendezvous / p39, agc_rendz_stable_orbit_rendezvous_deltt | stable_orbit.c |
| S17.1 S17.2 TRANSANG P17 (TPI_SEARCH, CSM) | | agc_rendz_s17_1 / s17_2 / transang | src/csm/navigation/tpi_search.c |
| P31 (GLAG, LM) | TIG RTARG DELLT4 -> DELVSIN HAPO HPER | agc_prog_lambert_aimpoint_guidance | glag_lm.c |
| P76 | DELVLVC TIG -> other SV updated | agc_prog_target_delta_v, agc_target_delta_v_lv_to_ref | target_delta_v.c |
| P21 PROG21 (GTDP) | time -> lat long alt, vel, gamma | agc_prog_ground_track_determination, agc_ground_track_determination_flight_path | ground_track_determination.c |
| PROG20 CSM, PIKUP20, SETINTG, WAITONE/CONTCHK, R21CSM, R23CSM, LLASRD | P20 flow | agc_job_rendezvous_navigation_csm*, rz_setintg, rz_waitone | src/csm/navigation/navigation.c |
| R22 CSM REND1..REND18, BVECTORS, GETUM, SHIFTNDX, INITIALW, RANGERD, VHFREAD | optics/VHF marks -> state update | agc_job_rendezvous_tracking_data_csm*, rz_bvectors, agc_rendz_vhf_read | src/csm/navigation/tracking_data.c, common.c |
| R61CSM CRS61.1/.2, R63 (V89 CSM) | desired CDUs, DAP rates | agc_rendz_tracking_attitude_csm, agc_job_rendezvous_final_attitude_csm* | src/csm/navigation/tracking_attitude.c |
| PROG22 CSM, S22N7071, P22SUBRB, CALLR52, DOV5N71 | P22 flow | agc_job_orbital_navigation_csm* | src/csm/navigation/orbital_navigation.c |
| S22.1, S22F2410, 9DWTO6DW, ROWDOT | landmark marks -> state | agc_job_orbital_navigation_marks* | src/csm/navigation/landmark_navigation.c |
| P23, V94ENTER, POINTAXS, R23.55-R23.7, HORIZ, RECT.1, LDPLANET | star-landmark/horizon angle -> CSM state | agc_job_cislunar_midcourse_navigation* | src/csm/navigation/cislunar_navigation.c |
| R57 R57A/B, ENDR57 | trunnion bias TRUNBIAS | rz_optics_calibration, agc_job_optics_calibration* | src/csm/navigation/cislunar_navigation.c |
| PROG20/22 LM, ORBCHG*, P20LEM*, LPS20.1/.2, UPPSV, P25 | LM P20/P22/P25 flow | agc_job_rendezvous_navigation_lm*, agc_job_preferred_tracking_attitude* | src/lm/navigation/navigation.c |
| R61LEM/R65LEM | LM tracking attitude | lmnav_tracking_attitude, agc_job_tracking_attitude_* | src/lm/navigation/tracking_attitude.c |
| R21LEM, R23LEM, R24LEM/LRS24.1 | RR acquisition, manual, search | agc_job_rendezvous_sighting_mark*, agc_job_manual_acquisition*, agc_job_radar_search* | src/lm/radar/acquisition.c |
| R22LEM, LRS22.1/.2, N49DSP | RR marks | agc_job_rendezvous_tracking_data* | src/lm/navigation/tracking_data.c |
| LSR22.3/.4, GETULC, RADARANG, WLINIT, LGCUPDTE | RR data -> state update | agc_job_rendezvous_tracking_data_incorporate/_measure | src/lm/navigation/measurement_update.c |
| MAGSUB, RRLIMCHK, SETTRKF, FLIP, CLRADMOD, RRTURNON, RRZERO, REMODE, RROUT, ENDRADAR | RR moding | agc_rr_*, agc_rendz_flip/clradmod, agc_task_rr_* | src/lm/radar/moding.c |
| RRDESSM, RRDESNB, BEGDES, DODES, GOTOV56 | RR designate | agc_rr_designate_*, agc_job_rr_designate_command | src/lm/radar/designate.c |
| RADAREAD, INITREAD, LRPOS2, R12LITES/RADLITES | radar reads | agc_rendz_radar_read, agc_radar_* | src/lm/radar/read.c |
| R29 | powered-flight designate | agc_radar_powered_flight_designate, agc_job/task_powered_flight_* | src/lm/radar/powered_flight_designate.c |
| RADSAMP / DORSAMP / VARADAR (LM) | radar sampling test | agc_task_radsamp, agc_job_radar_sample* | src/lm/radar/leadin.c |
| V67 (both) | W-matrix monitor | agc_job_w_matrix_monitor* | w_matrix_monitor.c |

External calls are funnelled through src/programs/navigation/glue.c (one place
to adapt when agc_nav.h / agc_integ.h / agc_navaux.h / agc_dsky.h / agc_imu.h
land); assumed signatures are in src/programs/navigation/pending.h.

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

## Deviations

Each is marked DEVIATION in the source:
* No recursion / unbounded AGC loops: CSI/A (csi.c) and the P17 search
  (src/csm/navigation/tpi_search.c) have a pass bound (AGC_STATUS_NO_CONVERGE); RR1AXIS,
  the R22 incorporation chains and S22.1 are bounded loops.
* INTSTALL BUSY: the job returns BUSY and resumes from its start entry; P20
  CSM re-reads MARKTIME after the sleep; P35 redoes the PRECSET step.
* Restart priorities PHSPRDT2 (R22LEM PRIO26, P23 PRIO13) and TEMPFLSH
  (R60ADRS) are not kept (exec lane owns the restart tables).
* GOTOV56 starts verb 56 as its own job (agc_extverb) instead of continuing
  the calling job at TRMTRACK.
* COMPTGO: own copy (rz_comptgo) with the return job as parameter.
* RSTACK has its own storage (AGC: EQUALS RANGE); RADMODES bit names in
  src/lm/radar/radar.h but storage is FLGWRD12 (see Open questions).
* Ones'-complement artefacts not modelled: NTARGCHK sums exactly; MSU -180
  deg exactly taken as -16383; RROUT's -0 zero change; ENDRADAR "-2" entries
  same as plain entries.
* R31 LM GETRVN reads RN/VN without the PRIO22 raise; R30 CANDEL does the
  INTWAKE itself when DELRSPL fails.
* tpi.c S34/35.2 uses UNIT RP for push cell 0D (open question).

## Stubs

None. (P24 rate-aided tracking named in NAMING.md is not in Comanche055 or
Luminary099; nothing to translate.)

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

## Tests written

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

## Program flow and decisions

_Documentation wave (2026-10-12), from the final tree (HEAD 99b2c6e). The prose was written by reading the code; the two tables were extracted from the function bodies (flag, alarm, channel and scheduling calls; `agc_erase_*` accessors) and list only what the code does. Files: `src/programs/navigation/*.c`, `src/programs/targeting/*.c`, `src/programs/displays/*.c`, `src/csm/navigation/*.c`, `src/lm/navigation/*.c`, `src/lm/radar/*.c`._

Rendezvous navigation, targeting and the orbit / rendezvous displays of
both vehicles, plus the LM radars. Every AGC program is one job that
sleeps on its flashing displays; here every display return and every
INTSTALL resumption is its own job (`agc_job_*`). Common subroutines and
erasables are in `src/programs/navigation/common.c`; vehicle-specific
flag ids come from `flags_csm.c` / `flags_lm.c`.

* **P20 / P22 navigation.** CSM (`src/csm/navigation/navigation.c`):
  `agc_job_rendezvous_navigation_csm_check` needs REFSMFLG; TRACKFLG
  enables tracking and UPDATFLG the state update; marks (R21 V57, R23
  V54 backup with BACKUP_MARKING) and VHF ranges are processed by R22
  (`src/csm/navigation/tracking_data.c`): VEHUPFLG picks which vehicle is
  updated (`td_shiftndx`), SOURCFLG optics or VHF variance (`td_variance`),
  INCORFLG the incorporation pass (`td_incorp1`, `td_passes`, through
  nav-aux INCORP1/2), RENDWFLG says W is valid (otherwise INITIALW);
  a refused VHF read lights TRACKER FAIL (`agc_rendz_vhf_read`, SKIPVHF).
  P22 landmark navigation (`src/csm/navigation/orbital_navigation.c`,
  `landmark_navigation.c`: KNOWNFLG known/unknown landmark, CMOONFLG
  body, DMENFLG 9-dim W reduced to 6 by `ln_dmpinteg`, an overflow skips
  a mark). P23 cislunar navigation and R57 optics calibration
  (`src/csm/navigation/cislunar_navigation.c`: star-horizon or
  star-landmark by NORFHOR / LUNAFLAG; R57 with the marking system busy is
  BAILOUT 01211). LM (`src/lm/navigation/navigation.c`, PROG22 = PROG20):
  SURFFLAG selects surface or flight; `lmnav_los` uses LOSCMFLG / RNDVZFLG;
  beyond 400 NM is alarm 00526; RR out of auto outside P20/P22 alarm 00514.
  R22LEM (`src/lm/navigation/tracking_data.c`): a radar LOS more than 3 deg
  from the state-vector LOS is alarm 00525 and goes back to the designate;
  `src/lm/navigation/measurement_update.c` incorporates range, range rate,
  shaft and trunnion (LSR22.3/4: too large a change stops for V06N49).
* **LM radar** (`src/lm/radar/`): designate (`designate.c`: RRDESSM within
  either mode's limits, else maneuver required / no coverage; bad angles
  alarm 00502), acquisition (`acquisition.c`: no lock-on in 60 tries
  00503, out of mode 2 coverage on the surface for 600 s 00530, manual
  acquisition outside limits 00501, search point out of coverage 00527),
  moding (`moding.c`: RRZERO out of auto 00510; a failed RR CDU ends in
  BADEND), reading (`read.c:agc_rendz_radar_read`: RADAR RUPT not expected
  00520, N failed samples 00521 unless R12/R04 runs, LR position wrong
  00522), R29 powered-flight designate (`powered_flight_designate.c`), the
  R04/R77 sampling loop (`leadin.c:agc_task_radsamp`).
* **Targeting.** P32/P72 CSI and P33/P73 CDH
  (`src/programs/targeting/csi_cdh_targeting.c`, iteration in
  `csi.c:agc_rendz_csi_a`: no solution is alarm 00600 + CSIALRM - 1 =
  00600..00606, bounded by CSI_MAX_PASSES as a DEVIATION); P34/P74 TPI and
  P35/P75 TPM (`tpi_tpm_targeting.c`, `tpi.c:agc_rendz_tpi_time_search`:
  no TPI time is alarm 00611); P38/P78 and P39/P79 stable orbit
  (`stable_orbit.c`); LM P31 Lambert aimpoint
  (`lambert_aimpoint_guidance_lm.c`); P76 target dV (`target_delta_v.c`);
  CSM P17/P77 TPI search (`src/csm/navigation/tpi_search.c`: no safe
  pericenter in the sector is alarm 00124). INITVEL
  (`initvel.c:agc_rendz_initvel`) takes LAMBERT's NO_CONVERGE as good
  enough (SOLNSW not tested). Each targeting program offers this vehicle
  or the other (`*_other_vehicle` jobs), ends at the V16N45 / V06N55
  displays (`src/programs/displays/display.c`: VN1645, alarms via
  `rz_alarm_flash` with V05N09), and leaves RTARG / TPASS4 / DELVSIN for
  P40-P42.
* **Displays**: R30 V82 (`orbit_parameter_display.c`: with average G off
  the crew picks the vehicle on V04N12, then V16N44), R31 V83 / CSM R34
  (`rendezvous_parameter_display.c`: RANGE, RRATE, RTHETA in a PRIO3 loop),
  R36 V90 (`out_of_plane_display.c`), P21 ground track
  (`ground_track_determination.c`), V67 W-matrix monitor
  (`w_matrix_monitor.c`: overflow shows DPPOSMAX), tracking attitudes R61
  (`src/csm/navigation/tracking_attitude.c`,
  `src/lm/navigation/tracking_attitude.c`).
* **Nouns**: `noun_data.c:agc_rendz_noun_read` / `agc_rendz_noun_write`
  serve this lane's erasables to pinball (NOT_FOUND for other ids).

### Decision points by function

170 of the lane's 657 functions test or change a flag, raise an alarm, test a channel bit or start a job/task (AGC = the label(s) the prolog names; "+F" sets flag F, "-F" clears it; "starts" = NOVAC/FINDVAC, WAITLIST, LONGCALL, DELAYJOB targets).

| C function | AGC | flags tested | flags set (+) / cleared (-) | alarms | channel bits | starts |
|---|---|---|---|---|---|---|
| `rz_optics_calibration` (cislunar_navigation.c) | R57 P23ABRT | - | - | BAILOUT 1211 | - | - |
| `agc_job_optics_calibration_end` (cislunar_navigation.c) | ENDR57 | - | - | - | - | job optics_calibration_end_mark |
| `cn_horiz` (cislunar_navigation.c) | HORIZ HORIZ.3 HORIZ.6 | LUNAFLAG NORFHOR | - | - | - | - |
| `cn_landmark` (cislunar_navigation.c) | R23.3 LALOTORV | LUNAFLAG | - | - | - | - |
| `cn_pointaxs` (cislunar_navigation.c) | POINTAXS R23.05 R23.5 | CMOONFLG LUNAFLAG ORBWFLAG ZMEASURE | - | - | - | - |
| `agc_job_cislunar_midcourse_navigation` (cislunar_navigation.c) | P23 P23.05 | REFSMFLG | - | - | - | - |
| `agc_job_cislunar_midcourse_navigation_checklist` (cislunar_navigation.c) | P23.31 | SAVECFLG | - | - | - | - |
| `agc_job_cislunar_midcourse_navigation_maneuvered` (cislunar_navigation.c) | P23.56 | TRUNNION_BIAS_KNOWN | - | - | - | - |
| `agc_job_cislunar_midcourse_navigation_incorporate` (cislunar_navigation.c) | P23.85 R23.6 | CMOONFLG | - | - | - | - |
| `agc_job_cislunar_midcourse_navigation_update` (cislunar_navigation.c) | R23.7 | CMOONFLG | - | - | - | - |
| `ln_unknown_landmark` (landmark_navigation.c) | DMPINTEG | CMOONFLG | - | - | - | - |
| `agc_job_orbital_navigation_marks` (landmark_navigation.c) | S22.111 | CMOONFLG | - | - | - | - |
| `ln_w_setup` (landmark_navigation.c) | FIG2EXIT SETWW5D S2INTS1 | DMENFLG ORBWFLAG | - | - | - | - |
| `ln_variance_and_incorporate` (landmark_navigation.c) | S22BOX12 | 22DSPFLG | - | - | - | - |
| `ln_incorporate` (landmark_navigation.c) | S22BOX12 S22BOX42 | CMOONFLG INCORFLG | - | - | - | - |
| `ln_dmpinteg` (landmark_navigation.c) | DMPINTEG S22D | CMOONFLG DMENFLG ERADFLAG KNOWNFLG | - | - | - | - |
| `agc_job_orbital_navigation_site` (landmark_navigation.c) | S22F244 S22BOX44 S22BX44A | CMOONFLG ERADFLAG | - | - | - | - |
| `rz_waitone` (navigation.c) | WAITONE | - | - | - | - | job rendezvous_navigation_csm_check |
| `agc_job_rendezvous_navigation_csm_check` (navigation.c) | CONTCHK REDOR22 | REFSMFLG TRACKFLG UPDATFLG | - | - | - | job rendezvous_navigation_csm_check |
| `agc_job_rendezvous_navigation_csm_integrate` (navigation.c) | P20.1 P20.3 | RENDWFLG VEHUPFLG | - | - | - | job rendezvous_tracking_data_csm |
| `agc_job_rendezvous_navigation_csm_track` (navigation.c) | PIKUP20 | REFSMFLG TRACKFLG | - | - | - | - |
| `agc_job_backup_sighting_mark` (navigation.c) | R23CSM | BACKUP_MARKING | - | - | - | - |
| `agc_job_orbital_navigation_landmark` (orbital_navigation.c) | PROG22A PROG22B | CMOONFLG | - | - | - | - |
| `on_site_from_rls` (orbital_navigation.c) | S22LSITE | ERADFLAG LUNAFLAG | - | - | - | - |
| `on_abde` (orbital_navigation.c) | S22ABDE OBTAINLL | KNOWNFLG | - | - | - | - |
| `tps_store` (tpi_search.c) | LEMSTORE CSMSTORE | AVFLAG | - | - | - | - |
| `tps_limits` (tpi_search.c) | S17.2 | KFLAG | - | - | - | - |
| `tps_trial` (tpi_search.c) | CONCAUL JUNCT3 PERIAPO | AVFLAG | - | - | - | - |
| `tps_step` (tpi_search.c) | JUNCT1 JUNCT2 | KFLAG | - | - | - | - |
| `tps_decide` (tpi_search.c) | HALFSAFE LRGRDVO | ITSWICH | - | - | - | - |
| `tps_enden` (tpi_search.c) | ENDEN USEKAY NEXUS | KFLAG | - | - | - | - |
| `agc_job_tpi_search_start` (tpi_search.c) | P17.1 | KFLAG | -UPDATFLG +UPDATFLG | - | - | - |
| `agc_job_tpi_search_compute` (tpi_search.c) | P17.2 ALARUMS | - | -UPDATFLG +UPDATFLG | ALARM 0124 | - | - |
| `ta_desired` (tracking_attitude.c) | R63 CRS61.1 | PRFTRKAT | - | - | - | - |
| `ta_point_at_target` (tracking_attitude.c) | CRS61.1 STKTEST AUTOCK | STIKFLAG | - | - | - | - |
| `td_shiftndx` (tracking_data.c) | SHIFTNDX | VEHUPFLG | - | - | - | - |
| `td_variance` (tracking_data.c) | REND9A REND10 | SOURCFLG | - | - | - | - |
| `td_optics_mark` (tracking_data.c) | REND7 REND9A | BACKUP_MARKING INCORFLG VEHUPFLG | - | - | - | - |
| `td_incorp1` (tracking_data.c) | REND10A | INCORFLG | - | - | - | - |
| `agc_job_rendezvous_tracking_data_csm_vhf` (tracking_data.c) | REND3 RANGERD | VHFRFLAG | - | - | - | - |
| `agc_rendz_vhf_read` (tracking_data.c) | VHFREAD VHFGOOD BYPASS | PDSPFLAG SKIPVHF UPDATFLG | - | - | - | - |
| `td_integrate` (tracking_data.c) | REND4 CSMUPP REND5 | RENDWFLG VEHUPFLG | - | - | - | - |
| `td_passes` (tracking_data.c) | REND7 REND18 | CMOONFLG INCORFLG SOURCFLG VEHUPFLG | - | - | - | - |
| `agc_job_rendezvous_tracking_data_csm_incorporate` (tracking_data.c) | REND4 REND6 | RENDWFLG | - | - | - | - |
| `agc_job_rendezvous_tracking_data_csm_display` (tracking_data.c) | RENDISP | SOURCFLG | - | - | - | job rendezvous_tracking_data_csm_residuals |
| `agc_job_rendezvous_tracking_data_csm_residuals_wait` (tracking_data.c) | RENDISP7 RENDISP3 RENDISP4 | SOURCFLG | - | - | - | - |
| `mu_sign` (measurement_update.c) | VEHUPFLG | VEHUPFLG | - | - | - | - |
| `mu_wlinit` (measurement_update.c) | WLINIT | SURFFLAG | - | - | - | - |
| `mu_integrate` (measurement_update.c) | LSR22.3 | RENDWFLG VEHUPFLG | - | - | - | - |
| `mu_incorp1` (measurement_update.c) | LGCUPDTE | DMENFLG | - | - | - | - |
| `mu_incorp2` (measurement_update.c) | ASTOK INCORP2 | LMOONFLG | - | - | - | - |
| `mu_next` (measurement_update.c) | RENDEND | SURFFLAG | - | - | - | - |
| `agc_job_rendezvous_tracking_data_incorporate` (measurement_update.c) | LSR22.3 LSR22.4 MARKTEST | RENDWFLG SURFFLAG | - | - | - | - |
| `agc_job_rendezvous_tracking_data_measure` (measurement_update.c) | RANGEBQ1 | LMOONFLG | - | - | - | - |
| `lmnav_los` (navigation.c) | LPS20.1 | LOSCMFLG RNDVZFLG SURFFLAG | - | - | - | - |
| `nav_uppsv_one` (navigation.c) | UPPSV | SURFFLAG | - | - | - | - |
| `nav_uppsv` (navigation.c) | UPPSV UPPSV1 UPPSV4 | RENDWFLG SURFFLAG VEHUPFLG | - | - | - | - |
| `agc_job_rendezvous_navigation_lm` (navigation.c) | PROG20 PROG22 LUNSFCHK | SURFFLAG | - | - | - | - |
| `nav_orbit_states` (navigation.c) | ORBCHG3 | RENDWFLG | - | - | - | - |
| `agc_job_rendezvous_navigation_attitude` (navigation.c) | P20LEMA | SURFFLAG | - | - | - | - |
| `agc_job_rendezvous_navigation_acquire` (navigation.c) | P20LEMB P20LEMB5 P20LEMB6 | TRACKFLG | - | - | - | - |
| `agc_job_rendezvous_navigation_manual_request` (navigation.c) | P20LEMB2 | SURFFLAG | - | - | - | - |
| `agc_job_rendezvous_navigation_cdu_zero_wait` (navigation.c) | P20LEMB3 P20LEMB4 | ACMODFLG SRCHOPTN | - | - | - | job rendezvous_navigation_cdu_zero_wait |
| `agc_job_rendezvous_navigation_designate` (navigation.c) | P20LEMC P20LEMD P20LEMF | RNDVZFLG TRACKFLG | - | - | - | task rendezvous_navigation_track_wait |
| `lmnav_wait` (navigation.c) | P20LEMWT P20LMWT1 | TRACKFLG | - | - | - | task rendezvous_navigation_track_check |
| `agc_task_rendezvous_navigation_track_check` (navigation.c) | P20LEMC1 | RNDVZFLG TRACKFLG | - | - | - | job rendezvous_tracking_data_next, task rendezvous_navigation_track_check |
| `agc_task_rendezvous_navigation_track_wait` (navigation.c) | P20LEMD1 P20LEMD2 | TRACKFLG | - | - | - | job rendezvous_navigation_update_states, task rendezvous_navigation_track_wait |
| `agc_job_preferred_tracking_attitude_check` (navigation.c) | P25LEM1 P25LMWT1 | PREFERRED_TRACKING_ATTITUDE_ACTIVE TRACKFLG | - | - | - | task preferred_tracking_attitude_wait |
| `agc_task_preferred_tracking_attitude_wait` (navigation.c) | P25LEM2 | - | - | - | - | job preferred_tracking_attitude_check |
| `ta_next` (tracking_attitude.c) | R61C+L06 | USE_TRACKING_ATTITUDE | - | - | - | task tracking_attitude_wait |
| `agc_job_tracking_attitude_compute` (tracking_attitude.c) | R61C+L01 R61LEM1 | TRACKFLG | - | - | - | - |
| `agc_task_tracking_attitude_wait` (tracking_attitude.c) | R61C+L2 | - | - | - | - | job tracking_attitude_compute |
| `agc_job_rendezvous_tracking_data` (tracking_data.c) | R22LEM LRS22.1 | RNDVZFLG TRACKFLG | - | - | - | - |
| `agc_job_rendezvous_tracking_data_mark` (tracking_data.c) | LRS22.1 CHEXERR | RNGSCFLG | - | - | - | - |
| `agc_job_rendezvous_tracking_data_accept` (tracking_data.c) | R22LEM2 LRS22.2 | SURFFLAG TRACKFLG | - | - | - | - |
| `agc_job_rendezvous_tracking_data_update` (tracking_data.c) | R22LEM3 | NOUPFLAG UPDATFLG | - | - | - | - |
| `agc_job_rendezvous_tracking_data_next` (tracking_data.c) | R22LEM42 R22LEM46 | SURFFLAG | - | - | - | job rendezvous_tracking_data |
| `agc_job_rendezvous_tracking_data_done` (tracking_data.c) | R22LEM93 R22LEM44 | SURFFLAG | - | - | - | job rendezvous_tracking_data |
| `agc_job_rendezvous_tracking_data_residuals_wait` (tracking_data.c) | R22LEM96 N49FLAG | SURFFLAG | - | - | - | - |
| `agc_job_rendezvous_sighting_mark` (acquisition.c) | R21LEM R21LEM5 R21LEM6 | SURFFLAG | - | - | - | - |
| `agc_job_rendezvous_sighting_mark_reposition` (acquisition.c) | R21LEM4 R21LEM12 R21LEM7 | FSPASFLG | - | - | - | - |
| `agc_task_rendezvous_sighting_mark_redesignate` (acquisition.c) | R21LEM9 | - | - | - | - | job rendezvous_sighting_mark_designate_los |
| `agc_job_radar_search_maneuver` (acquisition.c) | R24LEM3 | - | - | - | - | job radar_search_maneuver_done |
| `agc_job_radar_search_maneuver_done` (acquisition.c) | R24LEM3 | SURFFLAG | - | - | - | - |
| `agc_job_radar_search_point` (acquisition.c) | CHKSRCH | SRCHOPTN | - | - | - | task radar_search_next_point |
| `agc_job_radar_search_los` (acquisition.c) | LRS24.11 CONTDES2 OMEGCALC | - | - | ALARM 0527 | - | - |
| `agc_task_radar_search_next_point` (acquisition.c) | CALLDGCH | RNDVZFLG | - | - | - | job radar_search_data_check |
| `rr_startdes` (designate.c) | STARTDES DESRETRN | - | - | - | - | task rr_designate_begin |
| `agc_rr_designate_sm` (designate.c) | RRDESSM TRYSWS | RNDVZFLG SURFFLAG | - | - | - | - |
| `agc_rr_designate_nb` (designate.c) | RRDESNB | - | - | ALARM 0502 | - | - |
| `rr_stdesig` (designate.c) | STDESIG MOREDES | - | - | - | - | job rr_designate_command, task rr_designate_loop |
| `rr_dodes_los` (designate.c) | DODES DONBRD | RRNBSW | - | - | - | - |
| `rr_dorrout` (designate.c) | DORROUT NOTP20 | LOSCMFLG SRCHOPTN | - | - | - | - |
| `rr_dodes_commands` (designate.c) | DODES | RRNBSW | - | - | - | - |
| `agc_job_rr_designate_command` (designate.c) | DODES | LOKONSW | - | - | - | - |
| `agc_task_radsamp` (leadin.c) | RADSAMP | - | - | - | - | task radsamp, job radar_sample |
| `agc_job_radar_sample_done` (leadin.c) | DORSAMP | RADAR_ALARMS_SUPPRESSED | - | - | - | - |
| `rr_radnoop` (moding.c) | RADNOOP | - | - | - | - | task rr_goodend |
| `rr_zero_begin` (moding.c) | RRZEROSB | - | - | - | - | task rr_zero_counters |
| `agc_task_rr_zero_counters` (moding.c) | RRZEROSB | - | - | - | - | task rr_zero_settled |
| `agc_task_rr_zero_settled` (moding.c) | RRZEROSB | - | - | - | - | task rr_turnon_done |
| `agc_rr_zero` (moding.c) | RRZERO | - | - | ALARM 0510 | - | task rr_zero |
| `rr_axis_pass` (moding.c) | RR1AX2 | - | - | - | - | task rr_one_axis |
| `agc_task_dorrepos` (moding.c) | DORREPOS | - | - | - | - | task rr_reposition_drive |
| `agc_radar_powered_flight_designate` (powered_flight_designate.c) | R29 | LOSCMFLG NO_POWERED_DESIGNATE OLDESFLG | - | - | - | task powered_flight_preposition, job powered_flight_remode, task powered_flight_designate |
| `agc_job_powered_flight_remode` (powered_flight_designate.c) | R29REM0J | - | - | - | - | task rr_remode |
| `agc_task_powered_flight_designate` (powered_flight_designate.c) | BEGDES29 | - | - | - | - | job powered_flight_designate, task powered_flight_designate_loop |
| `agc_task_powered_flight_designate_loop` (powered_flight_designate.c) | R29DLOOP | LOSCMFLG | - | - | - | task powered_flight_designate_loop |
| `agc_job_powered_flight_designate` (powered_flight_designate.c) | R29DODES | - | - | - | - | task powered_flight_read |
| `agc_task_powered_flight_read` (powered_flight_designate.c) | R29READ | - | - | - | - | job powered_flight_read, task powered_flight_read_loop |
| `agc_task_powered_flight_read_loop` (powered_flight_designate.c) | R29READ | READRFLG | - | - | - | - |
| `agc_job_powered_flight_read` (powered_flight_designate.c) | R29RDJOB | NO_POWERED_DESIGNATE | - | - | - | - |
| `agc_job_powered_flight_read_done` (powered_flight_designate.c) | - | RNGSCFLG | - | - | - | - |
| `rd_lr_lamp` (read.c) | HLIGHT VLIGHT | SCALBAD | - | - | - | - |
| `rd_nomore` (read.c) | NOMORE | ALARM_521_SUPPRESSED LRBYPASS | - | ALARM 0521 | - | - |
| `agc_rendz_radar_read` (read.c) | RADAREAD | RADAR_ALARMS_SUPPRESSED | - | ALARM 0520, ALARM 0522 | - | - |
| `agc_radar_lrpos2` (read.c) | LRPOS2 | - | - | - | - | task lr_position_scan |
| `agc_task_lr_position_scan` (read.c) | LRPOSCAN | - | - | - | - | task lr_position_check |
| `agc_task_lr_position_check` (read.c) | LRPOSNXT | - | - | - | - | task lr_position_settled, task lr_position_check |
| `rz_alarm_flash` (display.c) | ALMXIT | - | - | ALARM code) | - | - |
| `rz_ttogo_display` (display.c) | VN1645 | FINALFLG REFSMFLG | - | - | - | job rendz_ttogo_display |
| `agc_job_rendz_clupdate` (display.c) | CLUPDATE | - | -UPDATFLG | - | - | - |
| `agc_job_rendz_ttogo_proceed` (display.c) | N45PROC | FINALFLG | +FINALFLG | - | - | - |
| `rz_tpi_final_displays` (display.c) | S34/35.5 | FINALFLG | +UPDATFLG | - | - | - |
| `agc_job_rendz_final_dv_checked` (display.c) | S34/35.5 | - | -XDELVFLG | - | - | - |
| `rz_prio_alarm` (display.c) | PRIOLARM | - | - | PRIOLARM code) | - | - |
| `rz_comptgo` (display.c) | COMPTGO | - | - | - | - | task cloktask |
| `agc_job_orbit_parameter_display` (orbit_parameter_display.c) | V82CALL | - | - | - | - | job orbit_display_average_g |
| `agc_job_orbit_display_vehicle_selected` (orbit_parameter_display.c) | PROCEED | - | - | - | - | task ticktest |
| `agc_job_orbit_display_recycle` (orbit_parameter_display.c) | V82GOFLP | - | - | - | - | job orbit_display_update |
| `agc_job_orbit_display_stall` (orbit_parameter_display.c) | V82STALL FLAGGON | - | - | - | - | job orbit_display_stall |
| `agc_job_orbit_display_repeat` (orbit_parameter_display.c) | V82GON3 | - | - | - | - | job orbit_display_average_g |
| `agc_task_ticktest` (orbit_parameter_display.c) | TICKTEST | - | - | - | - | job rendz_endext, task ticktest |
| `rpd_csm_havebase` (rendezvous_parameter_display.c) | HAVEBASE | SURFFLAG | - | - | - | - |
| `rpd_lm_update_states` (rendezvous_parameter_display.c) | REV83 | SURFFLAG | - | - | - | - |
| `rpd_lm_compute` (rendezvous_parameter_display.c) | V83CALL | SURFFLAG | - | - | - | - |
| `agc_job_rendezvous_parameter_display` (rendezvous_parameter_display.c) | R31CALL | - | - | - | - | job rendezvous_params_start, job rendezvous_params_wait, task rendezvous_params_wait |
| `agc_task_rendezvous_parameter_display` (rendezvous_parameter_display.c) | R31CALL | - | - | - | - | job rendezvous_params_start, task rendezvous_params_wait |
| `agc_job_rendezvous_params_wait` (rendezvous_parameter_display.c) | DSPDELAY | - | - | - | - | job rendezvous_params_wait |
| `agc_task_rendezvous_params_wait` (rendezvous_parameter_display.c) | DSPDELAY | - | - | - | - | task rendezvous_params_wait, job rendezvous_params_monitor |
| `rz_tracking_flags_on` (common.c) | P20FLGON | - | +TRACKFLG +UPDATFLG | - | - | - |
| `rz_precset` (common.c) | PRECSET LEMSTORE CSMSTORE | AVFLAG | - | - | - | - |
| `rz_intint` (common.c) | INTINT | CMOONFLG | - | - | - | - |
| `agc_rendz_get_lvc` (common.c) | GET.LVC | - | +MGLVFLAG | - | - | - |
| `agc_rendz_get_mga` (common.c) | GET+MGA | - | -MGLVFLAG | - | - | - |
| `rz_incorp2` (common.c) | INCORP2 | VEHUPFLG | - | - | - | - |
| `agc_job_ground_track_compute` (ground_track_determination.c) | P21PROG1 P21DSP | SURFFLAG | - | - | - | - |
| `agc_job_w_matrix_monitor_proceed` (w_matrix_monitor.c) | V06N9933 V6N99PRO | SURFFLAG W_MATRIX_OVERWRITE | - | - | - | - |
| `agc_rendz_advance` (csi.c) | ADVANCE | - | +XDELVFLG | - | - | - |
| `csi_apsis` (csi.c) | CSI/B3 | CMOONFLG | - | - | - | - |
| `csi_selectmu` (csi_cdh_targeting.c) | SELECTMU | CMOONFLG | -FINALFLG | - | - | - |
| `csi_almxit` (csi_cdh_targeting.c) | ALMXIT | - | - | ALARM (agc_alarm_code_t) c | - | - |
| `agc_job_csi_compute` (csi_cdh_targeting.c) | P32/P72B | FINALFLG | +UPDATFLG | - | - | - |
| `cdh_results` (csi_cdh_targeting.c) | P33/P73C P33/P73F | FINALFLG | +UPDATFLG | - | - | - |
| `agc_job_lambert_aimpoint_compute_lm` (lambert_aimpoint_guidance_lm.c) | V06N33 | - | -UPDATFLG -XDELVFLG | - | - | - |
| `agc_job_marks_tfi_mga_monitor` (lambert_aimpoint_guidance_lm.c) | REVN1645 | - | +FINALFLG | - | - | - |
| `so_intrpvp` (stable_orbit.c) | INTRPVP | AVFLAG | - | - | - | - |
| `agc_job_stable_orbit_start` (stable_orbit.c) | JUNCTN1 | CMOONFLG | - | - | - | - |
| `agc_job_stable_orbit_recycle` (stable_orbit.c) | RECYCLE JUNCTN2 | OPTNSW | - | - | - | - |
| `agc_job_stable_orbit_target` (stable_orbit.c) | MAINRTNE | FINALFLG STABLE_ORBIT_MIDCOURSE_ACTIVE | - | - | - | - |
| `agc_job_stable_orbit_dv_accepted` (stable_orbit.c) | DSPLY81 | STABLE_ORBIT_MIDCOURSE_ACTIVE | - | - | - | - |
| `som_start` (stable_orbit.c) | P39/P79A | CMOONFLG | - | - | - | - |
| `tdv_moonflag` (target_delta_v.c) | P76SUB1 | CMOONFLG | - | - | - | - |
| `agc_job_target_delta_v` (target_delta_v.c) | P76 | - | +TRACKFLG | - | - | - |
| `agc_job_target_delta_v_store` (target_delta_v.c) | INTSTALL | - | +REINTFLG | - | - | - |
| `tpi_search_pass` (tpi.c) | ELCALC ADTIME | ETPIFLAG | - | - | - | - |
| `tpi_selectmu` (tpi_tpm_targeting.c) | SELECTMU | CMOONFLG | -FINALFLG | - | - | - |
| `agc_job_tpi_after_elev` (tpi_tpm_targeting.c) | P34/P74A P34/P74B | - | -ETPIFLAG +ETPIFLAG | - | - | - |
| `agc_job_tpi_cycle` (tpi_tpm_targeting.c) | P34/P74C | ETPIFLAG | - | - | - | - |
| `tpi_after_search` (tpi_tpm_targeting.c) | SWCHCLR P34/P74D | ETPIFLAG | - | - | - | - |

### Shared erasables read and written

Accessors of `include/agc/core/state.h` and the lanes' public headers called by this lane, with the number of its functions that read / write each. Lane-private erasables are `static` in the lane's files or in its state struct; their scales are on the declarations.

| erasable (accessor) | meaning and scale (accessor prolog) | read by | written by |
|---|---|---|---|
| `cdus` | CSM optics shaft CDU raw word (CDUS / OPTX); LM: rendezvous radar shaft | 8 | 1 |
| `cdut` | CSM optics trunnion CDU raw word (CDUT / OPTY); LM: rendezvous radar trunnion | 9 | 1 |
| `cdutcmd` | CDUTCMD (053) / CDUSCMD (054): optics (CSM) or RR (LM) CDU error counter drive, SP pulse counts; which 0 = CDUTCMD, 1 = CDUSCMD. In TVC mode the same  | 0 | 1 |
| `cdux` | CDU X (outer gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (register CDUX, 032) | 5 | 0 |
| `cduxd` | none -- accessor of the DAP erasable CDUXD (ERASABLE_ASSIGNMENTS.agc).; returns uint16_t -- the erasable, at the scale given in agc/lm/dap.h | 1 | 1 |
| `cduy` | CDU Y (inner gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUY) | 5 | 0 |
| `cduyd` | none -- accessor of the DAP erasable CDUYD (ERASABLE_ASSIGNMENTS.agc).; returns uint16_t -- the erasable, at the scale given in agc/lm/dap.h | 1 | 1 |
| `cduz` | CDU Z (middle gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUZ) | 5 | 0 |
| `cduzd` | none -- accessor of the DAP erasable CDUZD (ERASABLE_ASSIGNMENTS.agc).; returns uint16_t -- the erasable, at the scale given in agc/lm/dap.h | 1 | 1 |
| `dnrrange` | DNRRANGE .. (downlink): index 0..6 = select code 1..7. AGC: DNRRANGE (ERASABLE_ASSIGNMENTS.agc); returns uint16_t -- the last raw RNRAD of that code ( | 3 | 0 |
| `dsptab` | display panel relay buffer DSPTAB +0..+10D, +11D is the C/S lights word (15-bit relay words) Index 0..11. | 2 | 1 |
| `extvbact` | EXTVBACT (ERASABLE_ASSIGNMENTS; TESTXACT p.237 C / p.264 L) -- the extended verb activity bits, read RETURNS: EXTVBACT, 15 bits; returns EXTVBACT, 15  | 5 | 2 |
| `horizon` | HORIZON (CM noun 70/71 R3), raw word. AGC: HORIZON (ERASABLE_ASSIGNMENTS.agc); returns uint16_t -- the raw 15-bit word | 3 | 2 |
| `imodes33` | last-sampled channel 33 / DAP-enable bits, T4RUPT C33TEST (IMODES33, B(1)PRM) | 3 | 0 |
| `landmark` | LANDMARK (CM P22 code ABCDE, V52) as a signed SP number. AGC: LANDMARK (ERASABLE_ASSIGNMENTS.agc); returns agc_sp_t -- the code (0 when the word is ou | 6 | 3 |
| `markdown` | MARKDOWN +0..+6 / MARK2DWN +0..+6: mark data for the downlink (time high/low, CDUY, CDUS(OPTX), CDUZ, CDUT(OPTY), CDUX), raw words. | 0 | 3 |
| `modecadr` | C(MODECADR +dev) (ERASABLE_ASSIGNMENTS.agc "MODE SWITCHING ERASABLE").; returns agc_sp_t -- the word (+0 none, positive a waiting job) | 1 | 2 |
| `modreg` | MODREG major mode (program) | 4 | 0 |
| `mrkbuf` | One raw word of MRKBUF1 / MRKBUF2 (layout above). AGC: MRKBUF1, MRKBUF2 (ERASABLE_ASSIGNMENTS.agc); returns uint16_t -- the raw 15-bit word (0 for a b | 7 | 6 |
| `mrkbuf2` | MRKBUF2 +index as an SP number (P76 ENDP76, R22: -1 = invalid). AGC: MRKBUF2 (ERASABLE_ASSIGNMENTS.agc); returns agc_sp_t -- MRKBUF2 +index as a signe | 2 | 4 |
| `piptime` | time of the PIPA reading / this vehicle's state vector, centiseconds, scale 2^28 (PIPTIME, B(2)PRM) | 2 | 0 |
| `r61cntr` | R61CNTR: R61 passes to skip (positive) or KALCMANU running (-1). AGC: R61CNTR (ERASABLE_ASSIGNMENTS.agc); returns agc_sp_t -- R61CNTR, a count | 2 | 2 |
| `radmodes` | RADMODES (ERASABLE_ASSIGNMENTS; EQUALS FLGWRD12, FLAGWORD_ASSIGNMENTS p.85) -- the radar mode bit word, read RETURNS: uint16_t -- RADMODES (15 bits);  | 11 | 6 |
| `refsmmat` | reference-to-stable-member matrix, row-major, DP elements, scale 2^1 (REFSMMAT, I(18D)PRM; 'REFSMMAT SCALE OF B1') | 13 | 0 |
| `rfailcnt` | none -- RFAILCNT (R04Z, RADSAMP) -- bad samples counted RETURNS: RFAILCNT; returns RFAILCNT | 1 | 1 |
| `rls` | landing site vector, moon-fixed reference, meters, scale 2^27 (RLS; R30 'SCALED AT (-27)M'); used by P20/P22/P52 lat-long, R30, P57, landing | 2 | 0 |
| `rn` | this vehicle's position, meters, scale 2^29 (earth) or 2^27 (moon) (RN, B(6)PRM) | 2 | 0 |
| `rnrad` | RNRAD (046): the radar reading (RR, LR or VHF range), raw 15-bit word. | 3 | 0 |
| `rsampdt` | none -- RSAMPDT (R04 / R77, read by RADSAMP) -- the sampling interval RETURNS: RSAMPDT: interval + 1, cs (0 stops the sampling); returns RSAMPDT: inte | 1 | 0 |
| `rstack` | RSTACK (LM, R04/R77 nouns 66, 67, 78): four DP radar samples, m or m/cs at the radar scale. AGC: RSTACK (ERASABLE_ASSIGNMENTS.agc); returns agc_dp_t - | 2 | 2 |
| `rtstbase` | none -- RTSTBASE (R04X / R04L) -- base of the sample storage RETURNS: 0 for the RR, 2 for the LR; returns 0 for the RR, 2 for the LR | 1 | 0 |
| `rtstdex` | none -- RTSTDEX (R04X) -- which radar is sampled RETURNS: RTSTDEX: 1 RR, 2 LR (test index); returns RTSTDEX: 1 RR, 2 LR (test index) | 1 | 1 |
| `rtstloc` | none -- RTSTLOC (R04Z, RADSAMP) -- the next sample slot RETURNS: RTSTLOC; returns RTSTLOC | 2 | 1 |
| `rtstmax` | none -- RTSTMAX (R04C) -- the last sample slot RETURNS: 2 for the RR, 6 for the LR; returns 2 for the RR, 6 for the LR | 1 | 0 |
| `tang` | TANG, TANG +1 (LM, noun 72): RR gimbal command or, for RRDESNB, the desired trunnion and shaft CDU words, as ones' complement SP numbers. AGC: TANG (E | 5 | 7 |
| `tetcsm` | time of the permanent CSM state vector, centiseconds, scale 2^28 (TETCSM) | 2 | 0 |
| `tetlem` | time of the permanent LM state vector, centiseconds, scale 2^28 (TETLEM) | 3 | 0 |
| `thetad` | desired gimbal angles THETAD +0..+2 (CPHI/CTHETA/CPSI), raw 15-bit CDU-style words, 2^15 = 1 revolution Index 0..2. | 0 | 2 |
| `tig` | time of ignition, centiseconds, scale 2^28 (TIG; DSP noun 33) | 10 | 6 |
| `trkmkcnt` | TRKMKCNT: rendezvous tracking mark count (noun 45 R1), whole number. | 5 | 9 |
| `trunbias` | TRUNBIAS: the R57 trunnion bias. AGC: TRUNBIAS (ERASABLE_ASSIGNMENTS.agc); returns agc_sp_t -- TRUNBIAS, CDU units | 1 | 0 |
| `ttpi` | TPI time, centiseconds, scale 2^28 (TTPI; DSP noun 37) | 13 | 3 |
| `vhfcnt` | VHFCNT (CM): VHF range mark count. AGC: VHFCNT (ERASABLE_ASSIGNMENTS.agc); returns agc_sp_t -- VHFCNT, a count | 3 | 5 |
| `vn` | this vehicle's velocity, m/cs, scale 2^7 (earth) or 2^5 (moon) (VN, B(6)PRM) | 2 | 0 |
| `x789` | X789: the radar / landmark bias states of the 9-dimensional W. AGC: X789 (ERASABLE_ASSIGNMENTS.agc); returns agc_vec_t -- X789 at the bias scales | 8 | 4 |

### AGC labels -> C functions

The "AGC" column above names the label each listed function translates; the complete label -> function -> file:line index for the lane (and the tree) is [docs/overview/xref.md](../overview/xref.md), generated by `tools/xref.py`.
