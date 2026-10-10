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

/* ==== sys-misc (include/agc/io/service_internal.h, src/io/service/sysmisc_ext.h) ================== */
/* DELAYJOB (SERVICE_ROUTINES): sleep the calling job delay_cs, then resume
 * it as wake_as.                                                          */
agc_status_t agc_svc_delayjob(uint16_t delay_cs, agc_job_id_t wake_as);
/* EXTVBACT: extended-verb / marking-system activity word (R30/R31/R36).  */
uint16_t     agc_erase_extvbact(void);
void         agc_erase_set_extvbact(uint16_t v);

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
/*@MOREJOBS@*/

/* ==== provisional task ids ================================================ */
#define AGC_TASK_TICKTEST     ((agc_task_id_t)0x1101) /* R30 V82 1 s tick      */
#define AGC_TASK_RENDEZVOUS_PARAMS_WAIT ((agc_task_id_t)0x1102) /* R31 DSPDELAY (LM) */
#define AGC_TASK_RADSAMP      ((agc_task_id_t)0x1103) /* LM radar sampling     */
/* CLOKTASK (LM BURN_BABY_BURN, owner guid-powered); same name as in
 * include/agc/guidance/pending.h.                                           */
#define AGC_TASK_CLOKTASK_RZ  ((agc_task_id_t)0x1104)

#endif /* RENDZ_EXT_H */
