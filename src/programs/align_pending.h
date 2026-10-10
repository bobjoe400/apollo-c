/*
 * align_pending.h -- interfaces the alignment code (prog-align) uses that
 * other lanes own and have not published in include/agc yet.
 *
 * INTEGRATION: every declaration below is an ASSUMED interface. When the
 * owning header appears, delete the matching block here and include that
 * header instead. Each block names its owner. Listed again in
 * docs/modules/prog-align.md under "Requested core additions".  Where
 * another lane already uses a name for the same thing (sys-misc
 * sysmisc.h / sysmisc_ext.h), the same name and signature are used here.
 *
 * Job/task identifiers: object-like constants AGC_JOB_<LABEL> /
 * AGC_TASK_<LABEL> with provisional values in the 0x0A00 range; the lead
 * replaces them with real agc_jobs.h members and dispatch.c cases.
 */
#ifndef ALIGN_PENDING_H
#define ALIGN_PENDING_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_exec.h"
#include "agc/agc_state.h"

/* ==== state-flags: shared erasables not yet in agc_state.h =============== */
agc_sp_t  agc_erase_option1(int index);        /* OPTION1 +index (N06), 0..2 */
void      agc_erase_set_option1(int index, agc_sp_t v);
uint16_t  agc_erase_extvbact(void);            /* EXTVBACT (as sys-misc)     */
void      agc_erase_set_extvbact(uint16_t v);
uint16_t  agc_erase_optmodes(void);            /* OPTMODES (CM, as sys-misc) */
void      agc_erase_set_optmodes(uint16_t v);
void      agc_erase_set_optind(agc_sp_t v);    /* OPTIND (CM, as sys-misc)   */
uint16_t  agc_erase_radmodes(void);            /* RADMODES (LM, as sys-misc) */
agc_dp_t  agc_erase_samptime(void);            /* SAMPTIME cs B28            */
void      agc_erase_set_samptime(agc_dp_t v);
agc_dp_t  agc_erase_tland(void);               /* TLAND cs B28 (guid-landing)*/
void      agc_erase_set_tland(agc_dp_t v);
uint16_t  agc_erase_aotaz(int index);          /* AOTAZ -1+index, cdu (pad)  */
uint16_t  agc_erase_aotel(int index);          /* AOTEL -1+index, cdu (pad)  */
agc_sp_t  agc_erase_landmark(void);            /* LANDMARK (CM, prog-rendz)  */
agc_sp_t  agc_erase_cduchkwd(void);            /* CDUCHKWD cs (CM, pad)      */
agc_sp_t  agc_erase_swsample(void);            /* SWSAMPLE (CM, T4RUPT)      */
uint16_t  agc_erase_agsword(void);             /* AGSWORD old downlist code  */
void      agc_erase_set_mrkbuf(int buf, int index, uint16_t v); /* MRKBUF1/2 */
uint16_t  agc_erase_mrkbuf(int buf, int index);
void      agc_erase_set_markdown(int index, uint16_t v); /* MARKDOWN +0..6   */
void      agc_erase_set_mark2dwn(int index, uint16_t v); /* MARK2DWN +0..6   */

/* ==== sys-misc (src/io/sysmisc.h, not public yet) ======================== */
agc_status_t agc_svc_delayjob(uint16_t delay_cs, agc_job_id_t wake_as); /* DELAYJOB */
agc_status_t agc_svc_b5off(void);                                       /* B5OFF    */
agc_status_t agc_telem_select_list(int dnlstcod);                       /* TS DNLSTCOD */

/* ==== exec-sched: VAC areas claimed by the mark system ==================== */
/* MARKOK/MKVAC: first free VACnUSE, marked occupied (CCS VACnUSE loop);
 * AGC_STATUS_FULL when none (the caller BAILOUTs 01207).                   */
agc_status_t agc_exec_vac_claim(int *vac_index);
agc_status_t agc_exec_vac_release(int vac_index);   /* TS VACnUSE          */

/* ==== prog-rendz (P20-P25, Comanche) ===================================== */
agc_status_t agc_rendz_llasrd(void);     /* LLASRD: LAT,LONG,ALT -> N89 regs */
agc_status_t agc_rendz_llasrda(void);    /* LLASRDA: N89 regs -> LAT,LONG,ALT*/
/* R61CSM: CSM tracking attitude; returns after the maneuver/update.        */
agc_status_t agc_rendz_r61csm(const agc_job_ctx_t *ctx, agc_job_id_t ret_job);

/* ==== provisional job and task identifiers ================================ */
#define AGC_JOB_ALIGN_COARSE_DONE           ((agc_job_id_t)0x0A01)
#define AGC_JOB_ALIGN_FINE_DONE             ((agc_job_id_t)0x0A02)
#define AGC_JOB_CHKSDATA_ENTER              ((agc_job_id_t)0x0A03)
#define AGC_JOB_CHKSDATA_FLASH              ((agc_job_id_t)0x0A04)
#define AGC_JOB_CHKSDATA_PROCEED            ((agc_job_id_t)0x0A05)
#define AGC_JOB_COARSE_ALIGN                ((agc_job_id_t)0x0A06)
#define AGC_JOB_COARSE_ALIGN_COARSED        ((agc_job_id_t)0x0A07)
#define AGC_JOB_COARSTYP                    ((agc_job_id_t)0x0A08)
#define AGC_JOB_ENTANSWR                    ((agc_job_id_t)0x0A09)
#define AGC_JOB_GYCRS                       ((agc_job_id_t)0x0A0A)
#define AGC_JOB_GYCRS_PULSE                 ((agc_job_id_t)0x0A0B)
#define AGC_JOB_GYCRS_PULSED                ((agc_job_id_t)0x0A0C)
#define AGC_JOB_GYCRS_TORQUED               ((agc_job_id_t)0x0A0D)
#define AGC_JOB_GYRO_TORQUING_PULSE         ((agc_job_id_t)0x0A0E)
#define AGC_JOB_GYRO_TORQUING_PULSED        ((agc_job_id_t)0x0A0F)
#define AGC_JOB_GYRO_TORQUING_RETURN        ((agc_job_id_t)0x0A10)
#define AGC_JOB_IMU_REALIGN_OPTION          ((agc_job_id_t)0x0A11)
#define AGC_JOB_IMU_REALIGN_OPTION_DISPLAY  ((agc_job_id_t)0x0A12)
#define AGC_JOB_MARKDISP                    ((agc_job_id_t)0x0A13)
#define AGC_JOB_MARK_COMPLETE_DISPLAY       ((agc_job_id_t)0x0A14)
#define AGC_JOB_MARK_REDISPLAY              ((agc_job_id_t)0x0A15)
#define AGC_JOB_MARK_REQUEST_DISPLAY        ((agc_job_id_t)0x0A16)
#define AGC_JOB_MARK_REQUEST_FLASH          ((agc_job_id_t)0x0A17)
#define AGC_JOB_PLANET_FLASH                ((agc_job_id_t)0x0A18)
#define AGC_JOB_PLANET_LOADED               ((agc_job_id_t)0x0A19)
#define AGC_JOB_PLANET_RESUME               ((agc_job_id_t)0x0A1A)
#define AGC_JOB_REALIGN_GIMBAL_DISPLAY      ((agc_job_id_t)0x0A1B)
#define AGC_JOB_REALIGN_NOMINAL_ORIENTATION ((agc_job_id_t)0x0A1C)
#define AGC_JOB_REALIGN_TIME_FLASH          ((agc_job_id_t)0x0A1D)
#define AGC_JOB_REALIGN_TIME_PROCEED        ((agc_job_id_t)0x0A1E)
#define AGC_JOB_SBANDANT_CM_RECYCLE         ((agc_job_id_t)0x0A1F)
#define AGC_JOB_SBANDANT_LM_RECYCLE         ((agc_job_id_t)0x0A20)
#define AGC_JOB_SBAND_B5OFF                 ((agc_job_id_t)0x0A21)
#define AGC_JOB_TERMSXT                     ((agc_job_id_t)0x0A22)

#define AGC_TASK_ENDMARKS                   ((agc_task_id_t)0x0A01)
#define AGC_TASK_MARKDIF                    ((agc_task_id_t)0x0A02)

#endif /* ALIGN_PENDING_H */
