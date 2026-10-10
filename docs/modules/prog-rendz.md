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
| S17.1 S17.2 TRANSANG P17 (TPI_SEARCH, CSM) | | agc_rendz_s17_1 / s17_2 / transang | tpi_search_csm.c |
| P31 (GLAG, LM) | TIG RTARG DELLT4 -> DELVSIN HAPO HPER | agc_prog_lambert_aimpoint_guidance | glag_lm.c |
| P76 | DELVLVC TIG -> other SV updated | agc_prog_target_delta_v, agc_target_delta_v_lv_to_ref | target_delta_v.c |
| P21 PROG21 (GTDP) | time -> lat long alt, vel, gamma | agc_prog_ground_track_determination, agc_ground_track_determination_flight_path | ground_track_determination.c |
| PROG20 CSM, PIKUP20, SETINTG, WAITONE/CONTCHK, R21CSM, R23CSM, LLASRD | P20 flow | agc_job_rendezvous_navigation_csm*, rz_setintg, rz_waitone | navigation_csm.c |
| R22 CSM REND1..REND18, BVECTORS, GETUM, SHIFTNDX, INITIALW, RANGERD, VHFREAD | optics/VHF marks -> state update | agc_job_rendezvous_tracking_data_csm*, rz_bvectors, agc_rendz_vhf_read | tracking_data_csm.c, common.c |
| R61CSM CRS61.1/.2, R63 (V89 CSM) | desired CDUs, DAP rates | agc_rendz_tracking_attitude_csm, agc_job_rendezvous_final_attitude_csm* | tracking_attitude_csm.c |
| PROG22 CSM, S22N7071, P22SUBRB, CALLR52, DOV5N71 | P22 flow | agc_job_orbital_navigation_csm* | orbital_navigation_csm.c |
| S22.1, S22F2410, 9DWTO6DW, ROWDOT | landmark marks -> state | agc_job_orbital_navigation_marks* | landmark_navigation_csm.c |
| P23, V94ENTER, POINTAXS, R23.55-R23.7, HORIZ, RECT.1, LDPLANET | star-landmark/horizon angle -> CSM state | agc_job_cislunar_midcourse_navigation* | cislunar_navigation_csm.c |
| R57 R57A/B, ENDR57 | trunnion bias TRUNBIAS | rz_optics_calibration, agc_job_optics_calibration* | cislunar_navigation_csm.c |
| PROG20/22 LM, ORBCHG*, P20LEM*, LPS20.1/.2, UPPSV, P25 | LM P20/P22/P25 flow | agc_job_rendezvous_navigation_lm*, agc_job_preferred_tracking_attitude* | navigation_lm.c |
| R61LEM/R65LEM | LM tracking attitude | lmnav_tracking_attitude, agc_job_tracking_attitude_* | tracking_attitude_lm.c |
| R21LEM, R23LEM, R24LEM/LRS24.1 | RR acquisition, manual, search | agc_job_rendezvous_sighting_mark*, agc_job_manual_acquisition*, agc_job_radar_search* | acquisition_lm.c |
| R22LEM, LRS22.1/.2, N49DSP | RR marks | agc_job_rendezvous_tracking_data* | tracking_data_lm.c |
| LSR22.3/.4, GETULC, RADARANG, WLINIT, LGCUPDTE | RR data -> state update | agc_job_rendezvous_tracking_data_incorporate/_measure | measurement_update_lm.c |
| MAGSUB, RRLIMCHK, SETTRKF, FLIP, CLRADMOD, RRTURNON, RRZERO, REMODE, RROUT, ENDRADAR | RR moding | agc_rr_*, agc_rendz_flip/clradmod, agc_task_rr_* | rr_moding_lm.c |
| RRDESSM, RRDESNB, BEGDES, DODES, GOTOV56 | RR designate | agc_rr_designate_*, agc_job_rr_designate_command | rr_designate_lm.c |
| RADAREAD, INITREAD, LRPOS2, R12LITES/RADLITES | radar reads | agc_rendz_radar_read, agc_radar_* | radar_read_lm.c |
| R29 | powered-flight designate | agc_radar_powered_flight_designate, agc_job/task_powered_flight_* | powered_flight_designate_lm.c |
| RADSAMP / DORSAMP / VARADAR (LM) | radar sampling test | agc_task_radsamp, agc_job_radar_sample* | radar_leadin_lm.c |
| V67 (both) | W-matrix monitor | agc_job_w_matrix_monitor* | w_matrix_monitor.c |

External calls are funnelled through src/programs/rendezvous/glue.c (one place
to adapt when agc_nav.h / agc_integ.h / agc_navaux.h / agc_dsky.h / agc_imu.h
land); assumed signatures are in src/programs/rendezvous/ext.h.

## Progress

* DONE infrastructure: ext.h (pending externals + provisional ids
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
  (map in "Renames" below); job ids in ext.h are descriptive.
* DONE csi_cdh_targeting.c: P32/P72, P33/P73 drivers (agc_job_csi_*, agc_job_cdh_*)
* DONE tpi_tpm_targeting.c: P34/P74, P35/P75 drivers (agc_job_tpi_*, agc_job_tpm_*); ATIGINC/PTIGINC added to agc_rendz_t
* DONE orbit_parameter_display.c: R30 V82 (SR30.1 agc_orbit_params_compute, MAXCHK, body select, V82 jobs, TICKTEST task, B5OFF/ENDEXT jobs)
* DONE initvel.c HAVEGUES agc_rendz_havegues (requested by guid lane), COGA kept between calls
* DONE tests/test_rendezvous.c (17 checks pass when linked with a scratch build of the tree)
* DONE target_delta_v.c: P76 (agc_job_target_delta_v*, COMPMAT agc_target_delta_v_lv_to_ref); VHFCNT, MRKBUF2 accessors in common.c
* DONE ground_track_determination.c: P21 (agc_job_ground_track_*, P21VSAVE agc_ground_track_determination_flight_path); rz_dsptem1_dp in glue.c
* DONE stable_orbit.c: P38/P78, P39/P79 (agc_job_stable_orbit_*, DELTTIME agc_rendz_stable_orbit_deltt); S3435.25 split out of tpi.c as agc_rendz_intercept_velocity
* DONE rule-7 pass (no (void) casts of agc_ calls; rz_flag_put/rz_vflag_set/rz_avflag/rz_tracking_flags_on now void)
* DONE out_of_plane_display.c: R36 V90 (agc_rendz_out_of_plane_compute, agc_job_out_of_plane_*)
* DONE rendezvous_parameter_display.c: R31 V83 both vehicles + CM R34 (COMPDISP agc_rendezvous_parameter_display_compdisp, agc_job_rendezvous_params_*)
* DONE tpi_search_csm.c (P17/P77: S17.1, S17.2, TRANSANG, jobs) -- verified complete on resume
* DONE lambert_aimpoint_guidance_lm.c (P31) -- verified complete on resume
* DONE radar_lm.h (private LM radar interface; RADMODES bit names)
* DONE rr_moding_lm.c: MAGSUB, RRLIMCHK agc_rr_limchk, SETTRKF agc_rr_settrkf,
  FLIP agc_rendz_flip, CLRADMOD agc_rendz_clradmod, RRTURNON/RRZEROSB tasks,
  RRZERO agc_rr_zero, DORREPOS/REPOSRPT, REMODE, RR1AXIS (bounded loop),
  RROUT, ENDRADAR/RGOODEND/RDBADEND, RADNOOP
* DONE rr_designate_lm.c: RRDESSM agc_rr_designate_sm, RRDESNB
  agc_rr_designate_nb, STARTDES, BEGDES/STDESIG/DESLOOP, DODES
  agc_job_rr_designate_command, GOTOV56 agc_rendz_goto_terminate_tracking
* DONE radar_read_lm.c: INITREAD + lead-ins (agc_radar_rrrange/rrrdot/lralt/lrvel),
  RADAREAD agc_rendz_radar_read, LRPOS2 + scan tasks, R12LITES/RADLITES
* DONE powered_flight_designate_lm.c (R29: agc_radar_powered_flight_designate,
  R29REM0J, PREPOS29, BEGDES29/R29DLOOP, R29DODES, R29READ/R29RDJOB)
* DONE radar_leadin_lm.c (RADSAMP agc_task_radsamp, DORSAMP, VARADAR, RSTACK)
* DONE rr_designate_lm.c additions: agc_rr_designate_gimbal (V41 request), TANG accessors
* DONE navigation_lm.c: PROG20/PROG22 (LM), ORBCHGO..ORBCHG3, PROG20A,
  P20LEM1, 526ALARM, P20LEMA/B/B3-B6/C/C3/D + tasks P20LEMC1/D1, P20LEMWT
  (lmnav_wait), UPPSV, LPS20.1 lmnav_los, LPS20.2, P25 (PROG25, P25LEM1/2)
* DONE tracking_attitude_lm.c: R61LEM/R65LEM (lmnav_tracking_attitude, jobs)
* DONE acquisition_lm.c: R21LEM (all), R23LEM, R24LEM + LRS24.1
* DONE tracking_data_lm.c (R22LEM, LRS22.1/2, N49DSP + N49FLAG wait, R22RSTRT)
* DONE measurement_update_lm.c (LSR22.3/.4, GETULC, RADARANG, MXMYMZ,
  LGCUPDTE (INCORP1 + RMAX/VMAX check), INCORP2 glue, WLINIT, INTGRCAL)
* DONE common.c: agc_rendz_nav_pad (pad loads), X789 accessors
* DONE w_matrix_monitor.c (V67 both vehicles: V67CALL, V67WW, V6N99PRO/INP)
* DONE rz_clear_ovf, rz_phase, rz_phase_job, rz_prio_alarm glue helpers
* DONE navigation_lm.c renamed from rendezvous_navigation_lm.c (file-name rule:
  the folder name is not repeated); LM entry is agc_job_rendezvous_navigation_lm
* DONE common.c: MRKBUF1/MRKBUF2 storage (agc_erase_mrkbuf / set; owned here),
  rz_incorp2 (INCORP2 glue, both vehicles), rz_word_value
* DONE tracking_attitude_csm.c: R61CSM agc_rendz_tracking_attitude_csm, CRS61.1,
  CRS61.2(A), the R63 subroutine (desired CDUs), R61CNTR accessors
* DONE navigation_csm.c: P20/P22 vehicle dispatch (agc_job_rendezvous_navigation,
  agc_job_orbital_navigation), PROG20 CSM, PIKUP20, P20R52JB, SETINTG rz_setintg,
  WAITONE/CONTCHK/REDOR22 rz_waitone, R21CSM (V57), R23CSM (V54), LANDMARK
* DONE tracking_data_csm.c: R22 CSM (REND1..REND18, RENDISP), BVECTORS, GETUM,
  SHIFTNDX, CMPOS/LMPOS, INITIALW, RANGERD, VHFREAD agc_rendz_vhf_read
* DONE V89 CSM (R63: agc_job_rendezvous_final_attitude_csm..., tracking_attitude_csm.c)
* DONE display.c agc_rendz_ttogo_display_start; noun_data.c agc_rendz_noun_read/write
* DONE navigation_csm.c LLASRD/LLASRDA (agc_rendz_llasrd/llasrda); P21 now stores
  LAT LONG ALT (agc_navaux_set_latlong)
* DONE orbital_navigation_csm.c: PROG22 (CSM), V06N45, PROG22A/B/C, S22N7071 code
  check (S22DSPPA/S22DSPP/S22ABDE), S22LSITE, OBTAINLL/P22SUBRB/N89ERRX, CALLR52,
  DE-GR-50, DOV5N71, P22OVER
* DONE landmark_navigation_csm.c: S22.1 (marks to SVMRKDAT, FIG2EXIT/S2INTS1,
  DMPINTEG, unknown landmark U/E/W72-W78, known landmark S22BOX22, S22D=9,
  S22BOX12/32/42, N49, S22F244, N89, RLS), S22F2410, S2231X13, 9DWTO6DW, ROWDOT
* DONE common.c rz_bvectors (BVECTORS, shared CSM R22/S22.1)
* DONE cislunar_navigation_csm.c: R57 (rz_optics_calibration, R57A/MARKDISP/R57B/
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
  (tpi_search_csm.c) have a pass bound (AGC_STATUS_NO_CONVERGE); RR1AXIS,
  the R22 incorporation chains and S22.1 are bounded loops.
* INTSTALL BUSY: the job returns BUSY and resumes from its start entry; P20
  CSM re-reads MARKTIME after the sleep; P35 redoes the PRECSET step.
* Restart priorities PHSPRDT2 (R22LEM PRIO26, P23 PRIO13) and TEMPFLSH
  (R60ADRS) are not kept (exec lane owns the restart tables).
* GOTOV56 starts verb 56 as its own job (agc_extverb) instead of continuing
  the calling job at TRMTRACK.
* COMPTGO: own copy (rz_comptgo) with the return job as parameter.
* RSTACK has its own storage (AGC: EQUALS RANGE); RADMODES bit names in
  radar_lm.h but storage is FLGWRD12 (see Open questions).
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

All are declared in src/programs/rendezvous/ext.h with the owning lane's
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
* Interfaces assumed from not-yet-existing headers: see ext.h
  (conic TIMETHET/TIMERAD/LAMBERT/APSIDES, CSM/LEM PREC/CONIC, INTEGRVS,
  INTSTALL, TFF routines, LAT-LONG, DSKY flash displays, GOTOPOOH, ENDEXT,
  DELAYJOB, CDUTRIG/*NBSM*/SXTNB).

## Requested jobs-tasks

Provisional ids in src/programs/rendezvous/ext.h (prototypes in
include/agc/programs/rendezvous.h before /*@JOBS-END@*/): jobs
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
