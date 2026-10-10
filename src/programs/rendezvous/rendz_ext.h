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

/* ==== sys-misc (include/agc/agc_sysmisc.h, src/io/service/sysmisc_ext.h) ================== */
/* DELAYJOB (SERVICE_ROUTINES): sleep the calling job delay_cs, then resume
 * it as wake_as.                                                          */
agc_status_t agc_svc_delayjob(uint16_t delay_cs, agc_job_id_t wake_as);
/* EXTVBACT: extended-verb / marking-system activity word (R30/R31/R36).  */
uint16_t     agc_erase_extvbact(void);
void         agc_erase_set_extvbact(uint16_t v);

/* ==== provisional job ids (prog-rendz), range 0x1100 ===================== */
/* common (rendz_display.c) */
#define AGC_JOB_RENDZ_VNPOOH                 ((agc_job_id_t)0x1101) /* VNPOOH TC -5: same display again */
#define AGC_JOB_RENDZ_TTOGO_DISPLAY          ((agc_job_id_t)0x1102) /* VN1645 after DELAYJOB: V16N45 */
#define AGC_JOB_RENDZ_KILCLOCK               ((agc_job_id_t)0x1103) /* VN1645 TERMINATE: KILCLOCK */
#define AGC_JOB_RENDZ_TTOGO_PROCEED          ((agc_job_id_t)0x1104) /* VN1645 PROCEED: N45PROC */
#define AGC_JOB_RENDZ_CLUPDATE               ((agc_job_id_t)0x1105) /* VN1645 RECYCLE: CLUPDATE */
#define AGC_JOB_RENDZ_FINAL_DV_CHECKED       ((agc_job_id_t)0x1106) /* S34/35.5 after V06N81 PROCEED */
/* P32/P72 CSI targeting (rendz_csi_cdh_targeting.c) */
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
/*@MOREJOBS@*/

/* ==== provisional task ids ================================================ */
#define AGC_TASK_RZ_TICKTEST  ((agc_task_id_t)0x1101) /* R30 V82 1 s tick      */
#define AGC_TASK_RZ_DSPDELAY  ((agc_task_id_t)0x1102) /* R31 display delay     */
#define AGC_TASK_RADSAMP      ((agc_task_id_t)0x1103) /* LM radar sampling     */
/* CLOKTASK (LM BURN_BABY_BURN, owner guid-powered); same name as in
 * include/agc/agc_guid_pending.h.                                           */
#define AGC_TASK_CLOKTASK_RZ  ((agc_task_id_t)0x1104)

#endif /* RENDZ_EXT_H */
