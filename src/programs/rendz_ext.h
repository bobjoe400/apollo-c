/*
 * rendz_ext.h -- interfaces the rendezvous programs (prog-rendz) use that
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
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_exec.h"
#include "agc/agc_state.h"

/* ==== sys-misc (src/io/sysmisc.h, src/io/sysmisc_ext.h) ================== */
/* DELAYJOB (SERVICE_ROUTINES): sleep the calling job delay_cs, then resume
 * it as wake_as.                                                          */
agc_status_t agc_svc_delayjob(uint16_t delay_cs, agc_job_id_t wake_as);
/* EXTVBACT: extended-verb / marking-system activity word (R30/R31/R36).  */
uint16_t     agc_erase_extvbact(void);
void         agc_erase_set_extvbact(uint16_t v);

/* ==== provisional job ids (prog-rendz), range 0x1100 ===================== */
/* common (rendz_common.c, rendz_display.c) */
#define AGC_JOB_RENDZ_TTOGO_DISPLAY    ((agc_job_id_t)0x1101) /* VN1645 after DELAYJOB  */
#define AGC_JOB_RENDZ_KILCLOCK   ((agc_job_id_t)0x1102) /* KILCLOCK               */
#define AGC_JOB_RENDZ_TTOGO_PROCEED   ((agc_job_id_t)0x1103) /* N45PROC                */
#define AGC_JOB_RENDZ_CLUPDATE  ((agc_job_id_t)0x1104) /* CLUPDATE               */
#define AGC_JOB_S3435_N81     ((agc_job_id_t)0x1105) /* S34/35.5 V06N81 shown  */
#define AGC_JOB_RENDZ_FINAL_DV_CHECKED ((agc_job_id_t)0x1106) /* S34/35.5 after N81 PRO */
#define AGC_JOB_S3435_N59     ((agc_job_id_t)0x1107) /* S34/35.5 V06N59 shown  */
#define AGC_JOB_DISDVLVC_DONE ((agc_job_id_t)0x1108) /* DISDVLVC after V06N8x  */
/* P32/P72 (rendz_p32_p33.c) */
#define AGC_JOB_P32_N11       ((agc_job_id_t)0x1110) /* P32/P72A V06N11 TCSI   */
#define AGC_JOB_P32_N55       ((agc_job_id_t)0x1111) /* V06N55 NN ELEV         */
#define AGC_JOB_P32_N37       ((agc_job_id_t)0x1112) /* V06N37 TTPI            */
#define AGC_JOB_P32_SELMU     ((agc_job_id_t)0x1113) /* TIG = TCSI, SELECTMU   */
#define AGC_JOB_P32_CALC      ((agc_job_id_t)0x1114) /* P32/P72B compute       */
#define AGC_JOB_P32_N75       ((agc_job_id_t)0x1115) /* P32/P72F V06N75 shown  */
#define AGC_JOB_P32_DV        ((agc_job_id_t)0x1116) /* after N75: S32/33.1    */
#define AGC_JOB_P32_DV2       ((agc_job_id_t)0x1117) /* after V06N81: DISDVLVC */
#define AGC_JOB_P32_DV3       ((agc_job_id_t)0x1118) /* after V06N82: VN1645   */
#define AGC_JOB_P32_ALARM     ((agc_job_id_t)0x1119) /* ALMXIT V05N09          */
#define AGC_JOB_P32_N11_LM    ((agc_job_id_t)0x111A) /* LM VN0611 (after N11)  */
/* P33/P73 */
#define AGC_JOB_P33_N13       ((agc_job_id_t)0x1120) /* P33/P73A V06N13 TCDH   */
#define AGC_JOB_P33_SELMU     ((agc_job_id_t)0x1121) /* SELECTMU               */
#define AGC_JOB_P33_CALC      ((agc_job_id_t)0x1122) /* P33/P73B compute       */
#define AGC_JOB_P33_ALARM     ((agc_job_id_t)0x1123) /* 611 V05N09             */
#define AGC_JOB_P33_ALARM_PRO ((agc_job_id_t)0x1124) /* 611 V05N09 PROCEED     */
#define AGC_JOB_P33_N75       ((agc_job_id_t)0x1125) /* V06N75                 */
#define AGC_JOB_P33_DV        ((agc_job_id_t)0x1126) /* after N75              */
#define AGC_JOB_P33_DV2       ((agc_job_id_t)0x1127) /* after V06N81           */
/* P34/P74, P35/P75 (rendz_tpi.c) */
#define AGC_JOB_P34_N37       ((agc_job_id_t)0x1130) /* P34/P74A V06N37        */
#define AGC_JOB_P34_N55       ((agc_job_id_t)0x1131) /* DISPLAYE V06N55        */
#define AGC_JOB_P34_SELMU     ((agc_job_id_t)0x1132) /* P34/P74B SELECTMU      */
#define AGC_JOB_P34_CALC      ((agc_job_id_t)0x1133) /* P34/P74C compute       */
#define AGC_JOB_P34_ALARM     ((agc_job_id_t)0x1134) /* 611 V05N09             */
#define AGC_JOB_P34_N55B      ((agc_job_id_t)0x1135) /* SWCHCLR DISPLAYE       */
#define AGC_JOB_P34_N37B      ((agc_job_id_t)0x1136) /* P34/P74D V06N37        */
#define AGC_JOB_P34_E         ((agc_job_id_t)0x1137) /* P34/P74E compute       */
#define AGC_JOB_P34_N58       ((agc_job_id_t)0x1138) /* V06N58 shown           */
#define AGC_JOB_P34_FINAL     ((agc_job_id_t)0x1139) /* after N58: S34/35.5    */
#define AGC_JOB_P34_LOOP      ((agc_job_id_t)0x113A) /* after S34/35.5         */
#define AGC_JOB_P35_SELMU     ((agc_job_id_t)0x113B) /* P35/P75A SELECTMU      */
#define AGC_JOB_P35_CALC      ((agc_job_id_t)0x113C) /* P35/P75B compute       */
#define AGC_JOB_P35_LOOP      ((agc_job_id_t)0x113D) /* after S34/35.5         */
/* R36 (rendz_r36.c) */
#define AGC_JOB_R36_OPTION    ((agc_job_id_t)0x1140) /* CM V04N12 answer       */
#define AGC_JOB_R36_N16       ((agc_job_id_t)0x1141) /* R36P3 V06N16 shown     */
#define AGC_JOB_R36_INT       ((agc_job_id_t)0x1142) /* after V06N16           */
#define AGC_JOB_R36_N90       ((agc_job_id_t)0x1143) /* R36TAG2 V06N90         */
/* R30 (r30.c) */
#define AGC_JOB_V82_START     ((agc_job_id_t)0x1150) /* V82CALL                */
#define AGC_JOB_V82_OPTION    ((agc_job_id_t)0x1151) /* V82GOFF V04N06 answer  */
#define AGC_JOB_V82GOFF1      ((agc_job_id_t)0x1152) /* FINDVAC V82GOFF1       */
#define AGC_JOB_V82_N44       ((agc_job_id_t)0x1153) /* V16N44 answer          */
#define AGC_JOB_V82GON1       ((agc_job_id_t)0x1154) /* FINDVAC V82GON1 / loop */
#define AGC_JOB_V82_OPT2      ((agc_job_id_t)0x1155) /* V82GON V04N06 answer   */
#define AGC_JOB_V82_SHOW      ((agc_job_id_t)0x1156) /* V82GON2 display        */
/* R31 (r31.c) */
#define AGC_JOB_V83CALL       ((agc_job_id_t)0x1160) /* FINDVAC V83CALL        */
#define AGC_JOB_V83_DISP      ((agc_job_id_t)0x1161) /* DISPN5X                */
#define AGC_JOB_V83_LOOP      ((agc_job_id_t)0x1162) /* V83 recompute          */
/* P38/P39 (stable_orbit.c) */
#define AGC_JOB_P38_N33       ((agc_job_id_t)0x1170)
#define AGC_JOB_P38_N55       ((agc_job_id_t)0x1171)
#define AGC_JOB_P38_CALC      ((agc_job_id_t)0x1172)
#define AGC_JOB_P38_N58       ((agc_job_id_t)0x1173)
#define AGC_JOB_P38_N81       ((agc_job_id_t)0x1174)
#define AGC_JOB_P38_DONE      ((agc_job_id_t)0x1175)
#define AGC_JOB_P39_CALC      ((agc_job_id_t)0x1176)
#define AGC_JOB_P39_N81       ((agc_job_id_t)0x1177)
#define AGC_JOB_P39_DONE      ((agc_job_id_t)0x1178)
/* P17 (tpi_search_csm.c) */
#define AGC_JOB_P17_N37       ((agc_job_id_t)0x1180)
#define AGC_JOB_P17_N55       ((agc_job_id_t)0x1181)
#define AGC_JOB_P17_CALC      ((agc_job_id_t)0x1182)
#define AGC_JOB_P17_N58       ((agc_job_id_t)0x1183)
#define AGC_JOB_P17_DONE      ((agc_job_id_t)0x1184)
/* P31 (glag_lm.c) */
#define AGC_JOB_P31_N33       ((agc_job_id_t)0x1190)
#define AGC_JOB_P31_CALC      ((agc_job_id_t)0x1191)
#define AGC_JOB_P31_N81       ((agc_job_id_t)0x1192)
#define AGC_JOB_P31_N42       ((agc_job_id_t)0x1193)
#define AGC_JOB_P31_DONE      ((agc_job_id_t)0x1194)
/* P76 (p76.c) */
#define AGC_JOB_P76_N84       ((agc_job_id_t)0x11A0)
#define AGC_JOB_P76_N33       ((agc_job_id_t)0x11A1)
#define AGC_JOB_P76_CALC      ((agc_job_id_t)0x11A2)
/* P21 (p21.c) */
#define AGC_JOB_P21_N34       ((agc_job_id_t)0x11B0)
#define AGC_JOB_P21_CALC      ((agc_job_id_t)0x11B1)
#define AGC_JOB_P21_N43       ((agc_job_id_t)0x11B2)
#define AGC_JOB_P21_NEXT      ((agc_job_id_t)0x11B3)
/* radar lead-in (radar_leadin_lm.c) */
#define AGC_JOB_DORSAMP       ((agc_job_id_t)0x11C0) /* LM radar sample job    */

/* ==== provisional task ids ================================================ */
#define AGC_TASK_RZ_TICKTEST  ((agc_task_id_t)0x1101) /* R30 V82 1 s tick      */
#define AGC_TASK_RZ_DSPDELAY  ((agc_task_id_t)0x1102) /* R31 display delay     */
#define AGC_TASK_RADSAMP      ((agc_task_id_t)0x1103) /* LM radar sampling     */
/* CLOKTASK (LM BURN_BABY_BURN, owner guid-powered); same name as in
 * src/guidance/guid_pending.h.                                           */
#define AGC_TASK_CLOKTASK_RZ  ((agc_task_id_t)0x1104)

#endif /* RENDZ_EXT_H */
