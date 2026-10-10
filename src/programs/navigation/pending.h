/*
 * ext.h -- interfaces the rendezvous programs (prog-rendz) use that
 * other lanes own but had not published in include/agc when this module was
 * written, plus the provisional job/task ids of this module.
 *
 * INTEGRATION: every external declaration below is spelled exactly as the
 * owning lane spells it in its own pending header (named per block), so a
 * single implementation satisfies all of them. When the declaration moves
 * into include/agc, delete it here. Listed in docs/modules/prog-rendz.md
 * under "Requested core additions".
 *
 * Job/task ids: object-like constants AGC_JOB_<LABEL> / AGC_TASK_<LABEL>
 * with provisional values in the 0x1100 range; the lead replaces them with
 * agc_jobs.h members and dispatch.c cases (docs/modules/prog-rendz.md,
 * "Requested jobs-tasks").
 */
#ifndef RENDZ_EXT_H
#define RENDZ_EXT_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/core/interp.h"
#include "agc/exec/exec.h"
#include "agc/core/state.h"
#include "agc/nav/auxiliary.h"

/* ==== sys-misc (include/agc/io/service_internal.h, src/io/service/sysmisc_ext.h) ================== */
/* DELAYJOB (SERVICE_ROUTINES): sleep the calling job delay_cs, then resume
 * it as wake_as.                                                          */
agc_status_t agc_svc_delayjob(uint16_t delay_cs, agc_job_id_t wake_as);
/* EXTVBACT: extended-verb / marking-system activity word (R30/R31/R36).  */
uint16_t     agc_erase_extvbact(void);
void         agc_erase_set_extvbact(uint16_t v);

/* ==== exec-sched: KILLTASK (same declaration as include/agc/guidance/pending.h) */
agc_status_t agc_waitlist_kill(agc_task_id_t task);

/* ==== state-flags: radar counters (same declarations as src/lm/landing/pending.h)
 * RNRAD (counter 046): the radar reading, raw 15-bit word.                 */
uint16_t     agc_erase_rnrad(void);
void         agc_erase_set_cdutcmd(int which, agc_sp_t v); /* 0 CDUTCMD 053, 1 CDUSCMD 054 */

/* ==== sys-misc: R04 / R77 radar test storage (src/io/verbs/extended_verbs_lm_radar.c) */
agc_sp_t agc_erase_rsampdt(void);
agc_sp_t agc_erase_rtstdex(void);
void     agc_erase_set_rtstdex(agc_sp_t v);
agc_sp_t agc_erase_rtstbase(void);
agc_sp_t agc_erase_rtstmax(void);
agc_sp_t agc_erase_rtstloc(void);
void     agc_erase_set_rtstloc(agc_sp_t v);
agc_sp_t agc_erase_rfailcnt(void);
void     agc_erase_set_rfailcnt(agc_sp_t v);

/* ==== nav-aux: LAT, LONG, ALT erasables (noun 43; same declaration as
 * src/lm/landing/pending.h for the setter). LANDLAT EQUALS LAT (CM).     */
agc_status_t agc_navaux_set_latlong(const agc_latlong_t *ll);
agc_status_t agc_navaux_get_latlong(agc_latlong_t *ll);

/* ==== provisional job ids (prog-rendz), range 0x1100 ===================== */
/* common (display.c) */
#define AGC_JOB_RENDZ_VNPOOH                 ((agc_job_id_t)0x1101) /* VNPOOH TC -5: same display again */
#define AGC_JOB_RENDZ_TTOGO_DISPLAY          ((agc_job_id_t)0x1102) /* VN1645 after DELAYJOB: V16N45 */
#define AGC_JOB_RENDZ_KILCLOCK               ((agc_job_id_t)0x1103) /* VN1645 TERMINATE: KILCLOCK */
#define AGC_JOB_RENDZ_TTOGO_PROCEED          ((agc_job_id_t)0x1104) /* VN1645 PROCEED: N45PROC */
#define AGC_JOB_RENDZ_CLUPDATE               ((agc_job_id_t)0x1105) /* VN1645 RECYCLE: CLUPDATE */
#define AGC_JOB_RENDZ_FINAL_DV_CHECKED       ((agc_job_id_t)0x1106) /* S34/35.5 after V06N81 PROCEED */
/* P32/P72 CSI targeting (csi_cdh_targeting.c) */
#define AGC_JOB_CSI_TARGETING                ((agc_job_id_t)0x1107) /* P32 entry */
#define AGC_JOB_CSI_TARGETING_OTHER_VEHICLE  ((agc_job_id_t)0x1108) /* P72 entry */
#define AGC_JOB_CSI_INPUT                    ((agc_job_id_t)0x1109) /* P32/P72A (ALMXIT recycle) */
#define AGC_JOB_CSI_AFTER_TCSI               ((agc_job_id_t)0x110A) /* after V06N11 TCSI */
#define AGC_JOB_CSI_AFTER_NN_ELEV            ((agc_job_id_t)0x110B) /* after V06N55 NN ELEV */
#define AGC_JOB_CSI_AFTER_TTPI               ((agc_job_id_t)0x110C) /* after V06N37 TTPI */
#define AGC_JOB_CSI_COMPUTE                  ((agc_job_id_t)0x110D) /* P32/P72B */
#define AGC_JOB_CSI_ALARM_DISPLAY            ((agc_job_id_t)0x110E) /* ALMXIT V05N09 (PROCEED: show again) */
#define AGC_JOB_CSI_AFTER_RESULTS            ((agc_job_id_t)0x110F) /* after V06N75: S32/33.1 / DISDVLVC N81 */
#define AGC_JOB_CSI_AFTER_DV_LV              ((agc_job_id_t)0x1110) /* after V06N81: S32/33.1 tail, DISDVLVC N82 */
#define AGC_JOB_CSI_AFTER_CDH_DV             ((agc_job_id_t)0x1111) /* after V06N82: TTPIO, VN1645 */
/* P33/P73 CDH targeting */
#define AGC_JOB_CDH_TARGETING                ((agc_job_id_t)0x1112) /* P33 entry */
#define AGC_JOB_CDH_TARGETING_OTHER_VEHICLE  ((agc_job_id_t)0x1113) /* P73 entry */
#define AGC_JOB_CDH_INPUT                    ((agc_job_id_t)0x1114) /* P33/P73A (611 recycle) */
#define AGC_JOB_CDH_AFTER_TCDH               ((agc_job_id_t)0x1115) /* after V06N13 TCDH */
#define AGC_JOB_CDH_COMPUTE                  ((agc_job_id_t)0x1116) /* P33/P73B */
#define AGC_JOB_CDH_TIME_SEARCH              ((agc_job_id_t)0x1117) /* S33/34.1 after an INTSTALL sleep */
#define AGC_JOB_CDH_ALARM_PROCEED            ((agc_job_id_t)0x1118) /* 611 V05N09 PROCEED */
#define AGC_JOB_CDH_AFTER_RESULTS            ((agc_job_id_t)0x1119) /* after V06N75 */
#define AGC_JOB_CDH_AFTER_DV_LV              ((agc_job_id_t)0x111A) /* after V06N81 */
/* P34/P74 TPI and P35/P75 TPM targeting (rendz_tpi_tpm_targeting.c) */
#define AGC_JOB_TPI_TARGETING                ((agc_job_id_t)0x111B) /* P34 entry */
#define AGC_JOB_TPI_TARGETING_OTHER_VEHICLE  ((agc_job_id_t)0x111C) /* P74 entry */
#define AGC_JOB_TPI_INPUT                    ((agc_job_id_t)0x111D) /* P34/P74A (611 PROCEED) */
#define AGC_JOB_TPI_AFTER_TTPI               ((agc_job_id_t)0x111E) /* after V06N37 */
#define AGC_JOB_TPI_AFTER_ELEV               ((agc_job_id_t)0x111F) /* after DISPLAYE V06N55 */
#define AGC_JOB_TPI_CYCLE                    ((agc_job_id_t)0x1120) /* P34/P74C */
#define AGC_JOB_TPI_INTLOOP                  ((agc_job_id_t)0x1121) /* INTLOOP */
#define AGC_JOB_TPI_TIME_SEARCH              ((agc_job_id_t)0x1122) /* S33/34.1 after an INTSTALL sleep */
#define AGC_JOB_TPI_ALARM                    ((agc_job_id_t)0x1123) /* 611 alarm and V05N09 (V32: again) */
#define AGC_JOB_TPI_COMPUTE                  ((agc_job_id_t)0x1124) /* P34/P74E */
#define AGC_JOB_TPI_AFTER_RESULTS            ((agc_job_id_t)0x1125) /* after V06N58: S34/35.5 */
#define AGC_JOB_TPI_AFTER_FINAL              ((agc_job_id_t)0x1126) /* after S34/35.5: VN1645 */
#define AGC_JOB_TPM_TARGETING                ((agc_job_id_t)0x1127) /* P35 entry */
#define AGC_JOB_TPM_TARGETING_OTHER_VEHICLE  ((agc_job_id_t)0x1128) /* P75 entry */
#define AGC_JOB_TPM_COMPUTE                  ((agc_job_id_t)0x1129) /* P35/P75B */
#define AGC_JOB_TPM_AFTER_FINAL              ((agc_job_id_t)0x112A) /* after S34/35.5: VN1645 */
#define AGC_JOB_TPM_ADVANCE                  ((agc_job_id_t)0x112B) /* P35/P75B from PRECSET (INTSTALL resume) */
/* R30 orbit parameter display, verb 82 (orbit_parameter_display.c) */
#define AGC_JOB_ORBIT_PARAMETER_DISPLAY          ((agc_job_id_t)0x1130) /* V82CALL */
#define AGC_JOB_ORBIT_DISPLAY_VEHICLE_SELECT     ((agc_job_id_t)0x1131) /* V82GOFF V04N12 */
#define AGC_JOB_ORBIT_DISPLAY_VEHICLE_SELECTED   ((agc_job_id_t)0x1132) /* V04N12 PROCEED */
#define AGC_JOB_ORBIT_DISPLAY_RECYCLE            ((agc_job_id_t)0x1133) /* V82GOFLP */
#define AGC_JOB_ORBIT_DISPLAY_STALL              ((agc_job_id_t)0x1134) /* V82STALL */
#define AGC_JOB_ORBIT_DISPLAY_UPDATE             ((agc_job_id_t)0x1135) /* V82GOFF1 */
#define AGC_JOB_ORBIT_DISPLAY_AVERAGE_G          ((agc_job_id_t)0x1136) /* V82GON1 */
#define AGC_JOB_ORBIT_DISPLAY_REPEAT             ((agc_job_id_t)0x1137) /* V82GON3 */
#define AGC_JOB_ORBIT_DISPLAY_SPLASH             ((agc_job_id_t)0x1138) /* CANDEL (CM) */
#define AGC_JOB_ORBIT_DISPLAY_MONITOR            ((agc_job_id_t)0x1139) /* V82REDSP */
#define AGC_JOB_RENDZ_DISPLAY_TERMINATE          ((agc_job_id_t)0x113A) /* B5OFF */
#define AGC_JOB_RENDZ_ENDEXT                     ((agc_job_id_t)0x113B) /* ENDEXT as a job */
/* P76 target delta V (target_delta_v.c) */
#define AGC_JOB_TARGET_DELTA_V                   ((agc_job_id_t)0x1140) /* P76 */
#define AGC_JOB_TARGET_DELTA_V_DISPLAY           ((agc_job_id_t)0x1141) /* V06N84 */
#define AGC_JOB_TARGET_DELTA_V_TIG               ((agc_job_id_t)0x1142) /* V06N33 */
#define AGC_JOB_TARGET_DELTA_V_UPDATE            ((agc_job_id_t)0x1143) /* OTHPREC, COMPMAT */
#define AGC_JOB_TARGET_DELTA_V_INTEGRATE         ((agc_job_id_t)0x1144) /* INTSTALL, INTEGRVS */
#define AGC_JOB_TARGET_DELTA_V_STORE             ((agc_job_id_t)0x1145) /* INTSTALL, MINIRECT, ATOPOTH */
#define AGC_JOB_TARGET_DELTA_V_END               ((agc_job_id_t)0x1146) /* ENDP76 */
/* P21 ground track determination (ground_track_determination.c) */
#define AGC_JOB_GROUND_TRACK_DETERMINATION       ((agc_job_id_t)0x1148) /* PROG21 */
#define AGC_JOB_GROUND_TRACK_VEHICLE             ((agc_job_id_t)0x1149) /* GOPERF4 option 2 */
#define AGC_JOB_GROUND_TRACK_TIME                ((agc_job_id_t)0x114A) /* P21PROG1 V06N34 */
#define AGC_JOB_GROUND_TRACK_COMPUTE             ((agc_job_id_t)0x114B) /* INTSTALL .. V06N43 */
#define AGC_JOB_GROUND_TRACK_RECYCLE             ((agc_job_id_t)0x114C) /* V32 on V06N43 */
/* P38/P78 stable orbit rendezvous, P39/P79 midcourse (stable_orbit.c) */
#define AGC_JOB_STABLE_ORBIT_RENDEZVOUS          ((agc_job_id_t)0x1150) /* P38 */
#define AGC_JOB_STABLE_ORBIT_RENDEZVOUS_OTHER_VEHICLE ((agc_job_id_t)0x1151) /* P78 */
#define AGC_JOB_STABLE_ORBIT_TIG                 ((agc_job_id_t)0x1152) /* after V06N33 */
#define AGC_JOB_STABLE_ORBIT_CENTANG             ((agc_job_id_t)0x1153) /* V06N55 GOFLASHR */
#define AGC_JOB_STABLE_ORBIT_OPTION              ((agc_job_id_t)0x1154) /* OPTION1 = 5, OPTION2 = 1, V04N06 */
#define AGC_JOB_STABLE_ORBIT_OPTION_DISPLAY      ((agc_job_id_t)0x1155) /* V04N06 recycle */
#define AGC_JOB_STABLE_ORBIT_OPTION_CHOSEN       ((agc_job_id_t)0x1156) /* SOI (OPTN1) / SOR */
#define AGC_JOB_STABLE_ORBIT_DELTAR              ((agc_job_id_t)0x1157) /* V06N57 GOFLASHR */
#define AGC_JOB_STABLE_ORBIT_INTERCEPT_TIME      ((agc_job_id_t)0x1158) /* V06N34 */
#define AGC_JOB_STABLE_ORBIT_START               ((agc_job_id_t)0x1159) /* JUNCTN1 */
#define AGC_JOB_STABLE_ORBIT_RECYCLE             ((agc_job_id_t)0x115A) /* RECYCLE */
#define AGC_JOB_STABLE_ORBIT_TARGET              ((agc_job_id_t)0x115B) /* MAINRTNE */
#define AGC_JOB_STABLE_ORBIT_DV_LV               ((agc_job_id_t)0x115C) /* DSPLY81 */
#define AGC_JOB_STABLE_ORBIT_DV_ACCEPTED         ((agc_job_id_t)0x115D) /* after V06N81 */
#define AGC_JOB_STABLE_ORBIT_MIDCOURSE           ((agc_job_id_t)0x115E) /* P39 */
#define AGC_JOB_STABLE_ORBIT_MIDCOURSE_OTHER_VEHICLE ((agc_job_id_t)0x115F) /* P79 */
#define AGC_JOB_STABLE_ORBIT_MIDCOURSE_COMPUTE   ((agc_job_id_t)0x1160) /* P39/P79B */
#define AGC_JOB_STABLE_ORBIT_MIDCOURSE_ADVANCE   ((agc_job_id_t)0x1161) /* P39/P79B PRECSET */
/* R36 out-of-plane display, verb 90 (out_of_plane_display.c) */
#define AGC_JOB_OUT_OF_PLANE_DISPLAY             ((agc_job_id_t)0x1168) /* R36 */
#define AGC_JOB_OUT_OF_PLANE_VEHICLE             ((agc_job_id_t)0x1169) /* CM V04N12 */
#define AGC_JOB_OUT_OF_PLANE_START               ((agc_job_id_t)0x116A) /* R36A */
#define AGC_JOB_OUT_OF_PLANE_TIME                ((agc_job_id_t)0x116B) /* R36P3 V06N16 */
#define AGC_JOB_OUT_OF_PLANE_COMPUTE             ((agc_job_id_t)0x116C) /* R36INT .. V06N90 */
/* R31 rendezvous parameter display, verb 83 (rendezvous_parameter_display.c) */
#define AGC_JOB_RENDEZVOUS_PARAMETER_DISPLAY     ((agc_job_id_t)0x1170) /* R31CALL */
#define AGC_JOB_RENDEZVOUS_PARAMS_WAIT           ((agc_job_id_t)0x1171) /* CM DSPDELAY */
#define AGC_JOB_RENDEZVOUS_PARAMS_MONITOR        ((agc_job_id_t)0x1172) /* DISPN5X */
#define AGC_JOB_RENDEZVOUS_PARAMS_START          ((agc_job_id_t)0x1173) /* V83CALL */
#define AGC_JOB_RENDEZVOUS_PARAMS_UPDATE         ((agc_job_id_t)0x1174) /* V83 / REV83 */
/* P17/P77 TPI search, CSM (tpi_search_csm.c) */
#define AGC_JOB_TPI_SEARCH                       ((agc_job_id_t)0x1178) /* P17 */
#define AGC_JOB_TPI_SEARCH_OTHER_VEHICLE         ((agc_job_id_t)0x1179) /* P77 */
#define AGC_JOB_TPI_SEARCH_INPUT                 ((agc_job_id_t)0x117A) /* P17.1 V06N37 */
#define AGC_JOB_TPI_SEARCH_START                 ((agc_job_id_t)0x117B) /* S17.1, V06N72 */
#define AGC_JOB_TPI_SEARCH_COMPUTE               ((agc_job_id_t)0x117C) /* S17.2, V06N58 */
#define AGC_JOB_TPI_SEARCH_RESULTS               ((agc_job_id_t)0x117D) /* V06N55 */
#define AGC_JOB_TPI_SEARCH_DISPLAY               ((agc_job_id_t)0x117E) /* VNCOMP17 again */
#define AGC_JOB_TPI_SEARCH_RESPONSE              ((agc_job_id_t)0x117F) /* VNCOMP17 ENTER */
/* P31 general Lambert aimpoint guidance, LM (lambert_aimpoint_guidance_lm.c) */
#define AGC_JOB_LAMBERT_AIMPOINT_GUIDANCE        ((agc_job_id_t)0x1180) /* P31 */
#define AGC_JOB_LAMBERT_AIMPOINT_COMPUTE         ((agc_job_id_t)0x1181) /* after V06N33 */
#define AGC_JOB_LAMBERT_AIMPOINT_APSIDES         ((agc_job_id_t)0x1182) /* V06N42 */
#define AGC_JOB_LAMBERT_AIMPOINT_TTOGO           ((agc_job_id_t)0x1183) /* REVN1645 */
/* LM radar (rr_designate_lm.c, powered_flight_designate_lm.c, radar_leadin_lm.c) */
#define AGC_JOB_RR_DESIGNATE_COMMAND             ((agc_job_id_t)0x1188) /* DODES */
#define AGC_JOB_RENDEZVOUS_SIGHTING_MARK_NEW_LOS ((agc_job_id_t)0x1189) /* R21LEM2 (LM) */
#define AGC_JOB_POWERED_FLIGHT_REMODE            ((agc_job_id_t)0x118A) /* R29REM0J */
#define AGC_JOB_POWERED_FLIGHT_DESIGNATE         ((agc_job_id_t)0x118B) /* R29DODES */
#define AGC_JOB_POWERED_FLIGHT_READ              ((agc_job_id_t)0x118C) /* R29RDJOB */
#define AGC_JOB_POWERED_FLIGHT_READ_RANGE        ((agc_job_id_t)0x118D) /* R29RDJOB after RRRDOT RADSTALL */
#define AGC_JOB_POWERED_FLIGHT_READ_DONE         ((agc_job_id_t)0x118E) /* R29RDJOB after RRRANGE RADSTALL */
#define AGC_JOB_RADAR_SAMPLE                     ((agc_job_id_t)0x118F) /* DORSAMP */
#define AGC_JOB_RADAR_SAMPLE_DONE                ((agc_job_id_t)0x1190) /* DORSAMP after RADSTALL */
/* LM P20/P22/P25 (navigation_lm.c, tracking_attitude_lm.c) */
#define AGC_JOB_RENDEZVOUS_NAVIGATION            ((agc_job_id_t)0x1191) /* PROG20 = PROG22 (LM) */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_ORBIT_OPTION ((agc_job_id_t)0x1192) /* ORBCHGO GOPERF4 */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_ORBIT_CHOSEN ((agc_job_id_t)0x1193) /* ORBCHG1 */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_LAUNCH_TIME  ((agc_job_id_t)0x1194) /* V06N33* */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_ORBIT_CHANGE ((agc_job_id_t)0x1195) /* ORBCHG2/3 */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_START      ((agc_job_id_t)0x1196) /* PROG20A */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_RANGE_CHECK ((agc_job_id_t)0x1197) /* P20LEM1 */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_RANGE_ALARM ((agc_job_id_t)0x1198) /* 526ALARM */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_ATTITUDE   ((agc_job_id_t)0x1199) /* P20LEMA */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_ACQUIRE    ((agc_job_id_t)0x119A) /* P20LEMB */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_MANUAL_REQUEST ((agc_job_id_t)0x119B) /* P20LEMB6 ENTER */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_CDU_ZERO_WAIT ((agc_job_id_t)0x119C) /* P20LEMB3 */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_UPDATE_STATES ((agc_job_id_t)0x119D) /* P20LEMC3 */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_DESIGNATE  ((agc_job_id_t)0x119E) /* P20LEMC */
#define AGC_JOB_PREFERRED_TRACKING_ATTITUDE      ((agc_job_id_t)0x119F) /* PROG25 */
#define AGC_JOB_PREFERRED_TRACKING_ATTITUDE_CHECK ((agc_job_id_t)0x11A0) /* P25LEM1 */
#define AGC_JOB_TRACKING_ATTITUDE_COMPUTE        ((agc_job_id_t)0x11A1) /* R61C+L01 */
#define AGC_JOB_TRACKING_ATTITUDE_MANEUVERED     ((agc_job_id_t)0x11A2) /* after R60LEM */
/* LM R21 / R23 / R24 (acquisition_lm.c) */
#define AGC_JOB_RENDEZVOUS_SIGHTING_MARK         ((agc_job_id_t)0x11A3) /* R21LEM */
#define AGC_JOB_RENDEZVOUS_SIGHTING_MARK_CENTERED ((agc_job_id_t)0x11A4) /* R21LEM6 RADSTALL */
#define AGC_JOB_RENDEZVOUS_SIGHTING_MARK_LOCKON  ((agc_job_id_t)0x11A5) /* R21LEM3 RADSTALL */
#define AGC_JOB_RENDEZVOUS_SIGHTING_MARK_ALARM   ((agc_job_id_t)0x11A6) /* R21-503 */
#define AGC_JOB_RENDEZVOUS_SIGHTING_MARK_REPOSITION ((agc_job_id_t)0x11A7) /* R21LEM12 */
#define AGC_JOB_RENDEZVOUS_SIGHTING_MARK_COVERAGE_ALARM ((agc_job_id_t)0x11A8) /* R21LEM11 */
#define AGC_JOB_RENDEZVOUS_SIGHTING_MARK_VERIFY  ((agc_job_id_t)0x11A9) /* R21DISP */
#define AGC_JOB_RENDEZVOUS_SIGHTING_MARK_DESIGNATE_LOS ((agc_job_id_t)0x11AA) /* R21LEM10 */
#define AGC_JOB_MANUAL_ACQUISITION               ((agc_job_id_t)0x11AB) /* R23LEM */
#define AGC_JOB_MANUAL_ACQUISITION_DISPLAY       ((agc_job_id_t)0x11AC) /* R23LEM1 */
#define AGC_JOB_MANUAL_ACQUISITION_CHECK         ((agc_job_id_t)0x11AD) /* R23LEM11 */
#define AGC_JOB_MANUAL_ACQUISITION_MANEUVER      ((agc_job_id_t)0x11AE) /* R23LEM3 */
#define AGC_JOB_MANUAL_ACQUISITION_TERMINATE     ((agc_job_id_t)0x11AF) /* R23LEM2 */
#define AGC_JOB_MANUAL_ACQUISITION_LIMIT_ALARM   ((agc_job_id_t)0x11B0) /* OUTOFLIM */
#define AGC_JOB_RADAR_SEARCH                     ((agc_job_id_t)0x11B1) /* R24LEM */
#define AGC_JOB_RADAR_SEARCH_DISPLAY             ((agc_job_id_t)0x11B2) /* R24LEM2 */
#define AGC_JOB_RADAR_SEARCH_ACCEPT              ((agc_job_id_t)0x11B3) /* R24END */
#define AGC_JOB_RADAR_SEARCH_MANEUVER            ((agc_job_id_t)0x11B4) /* R24LEM3 */
#define AGC_JOB_RADAR_SEARCH_MANEUVER_DONE       ((agc_job_id_t)0x11B5) /* R24LEM4 */
#define AGC_JOB_RADAR_SEARCH_POINT               ((agc_job_id_t)0x11B6) /* LRS24.1 CHKSRCH */
#define AGC_JOB_RADAR_SEARCH_DATA_CHECK          ((agc_job_id_t)0x11B7) /* DATGDCHK */
/* LM R22 (tracking_data_lm.c, measurement_update_lm.c) */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA         ((agc_job_id_t)0x11B8) /* R22LEM */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_RANGE   ((agc_job_id_t)0x11B9) /* LRS22.1 after RRRDOT */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_MARK    ((agc_job_id_t)0x11BA) /* LRS22.1 after RRRANGE */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_LOS_ALARM ((agc_job_id_t)0x11BB) /* alarm 525 */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_DTHETA  ((agc_job_id_t)0x11BC) /* R22LEM1 V06N05 */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_ACCEPT  ((agc_job_id_t)0x11BD) /* R22LEM2 */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_NEXT    ((agc_job_id_t)0x11BE) /* R22LEM42 */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_UPDATE  ((agc_job_id_t)0x11BF) /* R22LEM3 */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_N49     ((agc_job_id_t)0x11C0) /* N49DSP */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_N49_WAIT ((agc_job_id_t)0x11C1) /* N49FLAG poll */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_N49_PROCEED ((agc_job_id_t)0x11C2) /* N49 PROCEED */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_N49_RECYCLE ((agc_job_id_t)0x11C3) /* N49 RECYCLE */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_RESTART ((agc_job_id_t)0x11C4) /* R22RSTRT */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_RESTART_READ ((agc_job_id_t)0x11C5) /* R22RSTRT RADSTALL */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_INCORPORATE ((agc_job_id_t)0x11C6) /* LSR22.3 */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_MEASURE ((agc_job_id_t)0x11C7) /* LSR22.3 RANGEBQ.. after R65 */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_DONE    ((agc_job_id_t)0x11C8) /* R22LEM93/R22LEM44 */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_ATTITUDE ((agc_job_id_t)0x11C9) /* R22LEM45 R65 */
#define AGC_JOB_W_MATRIX_MONITOR                 ((agc_job_id_t)0x036B) /* V67CALL (value as in src/io/verbs/pending.h) */
#define AGC_JOB_W_MATRIX_MONITOR_PROCEED         ((agc_job_id_t)0x11CA) /* V6N99PRO */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE  ((agc_job_id_t)0x11CB) /* GOTOV56 as a job */
#define AGC_JOB_RADAR_SEARCH_RESUME              ((agc_job_id_t)0x11CC) /* R24LEM4 */
#define AGC_JOB_RADAR_SEARCH_LOS                 ((agc_job_id_t)0x11CD) /* LRS24.11 */
#define AGC_JOB_RENDEZVOUS_SIGHTING_MARK_WAIT    ((agc_job_id_t)0x11CE) /* R21DISP PROCEED */
#define AGC_JOB_RENDEZVOUS_SIGHTING_MARK_AHEAD   ((agc_job_id_t)0x11CF) /* R21LEMB */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_WAIT    ((agc_job_id_t)0x11D0) /* R22WAIT after R61 */
#define AGC_JOB_W_MATRIX_MONITOR_DISPLAY         ((agc_job_id_t)0x11D1) /* V06N99DS */
/* CSM P20-P25 (navigation_csm.c, tracking_attitude_csm.c, tracking_data_csm.c) */
#define AGC_JOB_TRACKING_ATTITUDE_CSM            ((agc_job_id_t)0x11D2) /* R61CSM CRS61.1 */
#define AGC_JOB_TRACKING_ATTITUDE_CSM_MANEUVERED ((agc_job_id_t)0x11D3) /* after R60CSM */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_CHECK  ((agc_job_id_t)0x11D4) /* CONTCHK after 4 s */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_INTEGRATE ((agc_job_id_t)0x11D5) /* PROG20 integration */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_TRACK  ((agc_job_id_t)0x11D6) /* PIKUP20 */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_OPTICS ((agc_job_id_t)0x11D7) /* P20R52JB */
#define AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_OPTICS_DONE ((agc_job_id_t)0x11D8) /* after R52 */
#define AGC_JOB_RENDEZVOUS_SIGHTING_MARK_CSM     ((agc_job_id_t)0x036C) /* R21CSM, V57 (= AGC_JOB_SIGHTING_MARK in src/io/verbs/pending.h) */
#define AGC_JOB_BACKUP_SIGHTING_MARK             ((agc_job_id_t)0x036D) /* R23CSM, V54 (value as in src/io/verbs/pending.h) */
#define AGC_JOB_RENDEZVOUS_SIGHTING_MARK_REQUEST ((agc_job_id_t)0x11D9) /* R21C1 V51 */
#define AGC_JOB_BACKUP_SIGHTING_MARK_REQUEST     ((agc_job_id_t)0x11DA) /* R23CSM1 V53 */
#define AGC_JOB_BACKUP_SIGHTING_MARK_TAKEN       ((agc_job_id_t)0x11DB) /* R23CSM1 ENTER */
#define AGC_JOB_RENDEZVOUS_SIGHTING_MARK_DONE    ((agc_job_id_t)0x11DC) /* R21CSMA */
#define AGC_JOB_RENDEZVOUS_SIGHTING_MARK_END     ((agc_job_id_t)0x11DD) /* R21END */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM     ((agc_job_id_t)0x11DE) /* R22 (CSM) */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_MARK ((agc_job_id_t)0x11DF) /* REND1A */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_VHF ((agc_job_id_t)0x11E0) /* REND3 after WAITONE */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_RANGE ((agc_job_id_t)0x11E1) /* RANGERD after RADSTALL */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_INCORPORATE ((agc_job_id_t)0x11E2) /* REND4 */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_DISPLAY ((agc_job_id_t)0x11E3) /* RENDISP */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_N49 ((agc_job_id_t)0x11E4) /* RENDISP2 */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_N49_WAIT ((agc_job_id_t)0x11E5) /* RENDISP7 */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_N49_PROCEED ((agc_job_id_t)0x11E6)
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_N49_RECYCLE ((agc_job_id_t)0x11E7)
#define AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_CSM    ((agc_job_id_t)0x11E8) /* V89CALL (CSM) */
#define AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_CSM_OPTION ((agc_job_id_t)0x11E9) /* V04N06 */
#define AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_CSM_CHOSEN ((agc_job_id_t)0x11EA) /* SETPAF/RSTPAF */
#define AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_CSM_COMPUTE ((agc_job_id_t)0x11EB) /* V89RECL */
#define AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_CSM_MANEUVER ((agc_job_id_t)0x11EC) /* R60CSM */
/* CSM P22 (orbital_navigation_csm.c, landmark_navigation_csm.c) */
#define AGC_JOB_ORBITAL_NAVIGATION_MARKS         ((agc_job_id_t)0x11ED) /* S22.1 */
#define AGC_JOB_ORBITAL_NAVIGATION_MARK          ((agc_job_id_t)0x11EE) /* S22NXTIN */
#define AGC_JOB_ORBITAL_NAVIGATION_DELTA_ACCEPT  ((agc_job_id_t)0x11EF) /* S22BOX42 */
#define AGC_JOB_ORBITAL_NAVIGATION_MARKS_EXIT    ((agc_job_id_t)0x11F0) /* S22EXEX */
#define AGC_JOB_ORBITAL_NAVIGATION_SITE          ((agc_job_id_t)0x11F1) /* S22F244 */
#define AGC_JOB_ORBITAL_NAVIGATION_SITE_ACCEPT   ((agc_job_id_t)0x11F2) /* RLS */
#define AGC_JOB_ORBITAL_NAVIGATION_SITE_DONE     ((agc_job_id_t)0x11F3) /* S22.981X */
#define AGC_JOB_ORBITAL_NAVIGATION_SITE_TERMINATE ((agc_job_id_t)0x11F4) /* S22GTP */
#define AGC_JOB_ORBITAL_NAVIGATION               ((agc_job_id_t)0x11F5) /* P22 (by vehicle) */
#define AGC_JOB_ORBITAL_NAVIGATION_ORBIT_DISPLAY ((agc_job_id_t)0x11F6) /* V06N45 */
#define AGC_JOB_ORBITAL_NAVIGATION_LANDMARK      ((agc_job_id_t)0x11F7) /* PROG22A */
#define AGC_JOB_ORBITAL_NAVIGATION_CODE          ((agc_job_id_t)0x11F8) /* S22DSPPA */
#define AGC_JOB_ORBITAL_NAVIGATION_CODE_CHECK    ((agc_job_id_t)0x11F9) /* S22DSPPA PROCEED */
#define AGC_JOB_ORBITAL_NAVIGATION_LANDING_SITE  ((agc_job_id_t)0x11FA) /* P22SUBRB */
#define AGC_JOB_ORBITAL_NAVIGATION_SITE_CHECK    ((agc_job_id_t)0x11FB) /* P22SUBRB PROCEED */
#define AGC_JOB_ORBITAL_NAVIGATION_OPTICS        ((agc_job_id_t)0x11FC) /* CALLR52 */
#define AGC_JOB_ORBITAL_NAVIGATION_ADVANCED_OPTICS ((agc_job_id_t)0x11FD) /* DE-GR-50 */
#define AGC_JOB_ORBITAL_NAVIGATION_MARKED        ((agc_job_id_t)0x11FE) /* DOV5N71 */
#define AGC_JOB_ORBITAL_NAVIGATION_PROCESS       ((agc_job_id_t)0x11FF) /* PROG22C */
#define AGC_JOB_ORBITAL_NAVIGATION_OVER          ((agc_job_id_t)0x1200) /* P22OVER */
#define AGC_JOB_OPTICS_CALIBRATION               ((agc_job_id_t)0x1201) /* R57A */
#define AGC_JOB_OPTICS_CALIBRATION_MARK_DISPLAY  ((agc_job_id_t)0x1202) /* MARKDISP */
#define AGC_JOB_OPTICS_CALIBRATION_BIAS          ((agc_job_id_t)0x1203) /* R57B */
#define AGC_JOB_OPTICS_CALIBRATION_END           ((agc_job_id_t)0x1204) /* ENDR57 */
#define AGC_JOB_OPTICS_CALIBRATION_END_MARK      ((agc_job_id_t)0x1205) /* ENDMARK PRIO14 */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION    ((agc_job_id_t)0x1206) /* P23 */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_SIGHT ((agc_job_id_t)0x1207) /* P23.00 R53 */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_IDENT ((agc_job_id_t)0x1208) /* P23.06 */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_IDENT_CHECK ((agc_job_id_t)0x1209) /* P23.15 */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_STAR ((agc_job_id_t)0x120A) /* P23.30 */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_PLANET ((agc_job_id_t)0x120B) /* LDPLANET */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_CHECKLIST ((agc_job_id_t)0x120C) /* P23.31 */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_CALIBRATE ((agc_job_id_t)0x120D) /* P23.55/56 */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_POINT ((agc_job_id_t)0x120E) /* V94ENTER */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_POINT_AXES ((agc_job_id_t)0x120F) /* V94ENTER POINTAXS */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_MANEUVERED ((agc_job_id_t)0x1210) /* after R60CSM */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_OPTICS ((agc_job_id_t)0x1211) /* P23.57 */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_OPTICS_DONE ((agc_job_id_t)0x1212) /* after R52 */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_MARKED ((agc_job_id_t)0x1213) /* P23.60 */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_MARK_DISPLAY ((agc_job_id_t)0x1214) /* V05N71 */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_SAVE ((agc_job_id_t)0x1215) /* P23.65 */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_INCORPORATE ((agc_job_id_t)0x1216) /* P23.85 */
#define AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_UPDATE ((agc_job_id_t)0x1217) /* R23.7 */
/*@MOREJOBS@*/

/* ==== provisional task ids ================================================ */
#define AGC_TASK_TICKTEST     ((agc_task_id_t)0x1101) /* R30 V82 1 s tick      */
#define AGC_TASK_RENDEZVOUS_PARAMS_WAIT ((agc_task_id_t)0x1102) /* R31 DSPDELAY (LM) */
#define AGC_TASK_RADSAMP      ((agc_task_id_t)0x1103) /* LM radar sampling     */
/* CLOKTASK (LM BURN_BABY_BURN, owner guid-powered); same name as in
 * include/agc/guidance/pending.h.                                           */
#define AGC_TASK_CLOKTASK_RZ  ((agc_task_id_t)0x1104)
/* LM radar moding (rr_moding_lm.c); RRTURNON and DORREPOS keep the values
 * sys-misc gave them in src/io/service/sysmisc_ext.h.                       */
#define AGC_TASK_RRTURNON  ((agc_task_id_t)0x312)  /* prog-rendz: RRTURNON   */
#define AGC_TASK_DORREPOS  ((agc_task_id_t)0x313)  /* prog-rendz: DORREPOS   */
#define AGC_TASK_RR_ZERO_COUNTERS      ((agc_task_id_t)0x1105) /* RRZEROSB +20 ms */
#define AGC_TASK_RR_ZERO_SETTLED       ((agc_task_id_t)0x1106) /* RRZEROSB +10 s  */
#define AGC_TASK_RR_TURNON_DONE        ((agc_task_id_t)0x1107) /* RRTURNON +1 s   */
#define AGC_TASK_RR_ZERO               ((agc_task_id_t)0x1108) /* RRZ2            */
#define AGC_TASK_RR_GOODEND            ((agc_task_id_t)0x1109) /* RADNOOP: RGOODEND */
#define AGC_TASK_RR_ONE_AXIS           ((agc_task_id_t)0x110A) /* NXTRR1AX        */
#define AGC_TASK_RR_REPOSITION_DRIVE   ((agc_task_id_t)0x110B) /* DORREPOS +20 ms */
#define AGC_TASK_RR_REMODE             ((agc_task_id_t)0x110C) /* REMODE as a task */
#define AGC_TASK_RR_DESIGNATE_BEGIN    ((agc_task_id_t)0x110D) /* BEGDES          */
#define AGC_TASK_RR_DESIGNATE_LOOP     ((agc_task_id_t)0x110E) /* DESLOOP/STDESIG */
#define AGC_TASK_LR_POSITION_SCAN      ((agc_task_id_t)0x110F) /* LRPOSCAN        */
#define AGC_TASK_LR_POSITION_CHECK     ((agc_task_id_t)0x1110) /* LRPOSNXT        */
#define AGC_TASK_LR_POSITION_SETTLED   ((agc_task_id_t)0x1111) /* LASTLRDT        */
#define AGC_TASK_POWERED_FLIGHT_PREPOSITION ((agc_task_id_t)0x1112) /* PREPOS29 */
#define AGC_TASK_POWERED_FLIGHT_DESIGNATE   ((agc_task_id_t)0x1113) /* BEGDES29 */
#define AGC_TASK_POWERED_FLIGHT_DESIGNATE_LOOP ((agc_task_id_t)0x1114) /* R29DLOOP after VARDELAY */
#define AGC_TASK_POWERED_FLIGHT_READ   ((agc_task_id_t)0x1115) /* R29READ         */
#define AGC_TASK_POWERED_FLIGHT_READ_LOOP ((agc_task_id_t)0x1116) /* R29READ after 2 s */
#define AGC_TASK_RENDEZVOUS_NAVIGATION_TRACK_CHECK ((agc_task_id_t)0x1117) /* P20LEMC1 */
#define AGC_TASK_RENDEZVOUS_NAVIGATION_TRACK_WAIT  ((agc_task_id_t)0x1118) /* P20LEMD1 */
#define AGC_TASK_PREFERRED_TRACKING_ATTITUDE_WAIT  ((agc_task_id_t)0x1119) /* P25LEM2 */
#define AGC_TASK_TRACKING_ATTITUDE_WAIT            ((agc_task_id_t)0x111A) /* R61C+L2 */
#define AGC_TASK_RENDEZVOUS_SIGHTING_MARK_REDESIGNATE ((agc_task_id_t)0x111B) /* R21LEM9 */
#define AGC_TASK_RADAR_SEARCH_NEXT_POINT           ((agc_task_id_t)0x111C) /* CALLDGCH */
/*@MORETASKS@*/

#endif /* RENDZ_EXT_H */
