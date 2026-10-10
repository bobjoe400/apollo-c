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
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/core/interp.h"
#include "agc/exec/exec.h"
#include "agc/core/state.h"
#include "agc/io/service_internal.h"

/* ==== state-flags: shared erasables not yet in agc_state.h =============== */
agc_sp_t  agc_erase_option1(int index);        /* OPTION1 +index (N06), 0..2 */
void      agc_erase_set_option1(int index, agc_sp_t v);
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

/* ==== exec-sched: VAC areas claimed by the mark system ==================== */
/* MARKOK/MKVAC: first free VACnUSE, marked occupied (CCS VACnUSE loop);
 * AGC_STATUS_FULL when none (the caller BAILOUTs 01207).                   */
agc_status_t agc_exec_vac_claim(int *vac_index);
agc_status_t agc_exec_vac_release(int vac_index);   /* TS VACnUSE          */

/* ==== prog-rendz (P20-P25, Comanche) ===================================== */
agc_status_t agc_rendz_llasrd(void);     /* LLASRD: LAT,LONG,ALT -> N89 regs */
agc_status_t agc_rendz_llasrda(void);    /* LLASRDA: N89 regs -> LAT,LONG,ALT*/
/* R61CSM: CSM tracking attitude; returns after the maneuver/update.        */
agc_status_t agc_rendz_tracking_attitude_csm(const agc_job_ctx_t *ctx, agc_job_id_t ret_job);

/* ==== provisional job and task identifiers ================================ */
#define AGC_JOB_ALIGN_COARSE_DONE                ((agc_job_id_t)0x0A01)
#define AGC_JOB_ALIGN_FINE_DONE                  ((agc_job_id_t)0x0A02)
#define AGC_JOB_ALTERNATE_LOS_ANGLES_DISPLAY     ((agc_job_id_t)0x0A03)
#define AGC_JOB_ALTERNATE_LOS_MARKED             ((agc_job_id_t)0x0A04)
#define AGC_JOB_ALTERNATE_LOS_MARK_REQUEST       ((agc_job_id_t)0x0A05)
#define AGC_JOB_ALTERNATE_LOS_REMARK             ((agc_job_id_t)0x0A06)
#define AGC_JOB_ALTERNATE_LOS_RESERVE            ((agc_job_id_t)0x0A07)
#define AGC_JOB_AUTO_OPTICS_POSITIONING_DELAYED  ((agc_job_id_t)0x0A08)
#define AGC_JOB_AUTO_OPTICS_POSITIONING_RESUME   ((agc_job_id_t)0x0A09)
#define AGC_JOB_AUTO_OPTICS_POSITIONING_TERMINATE ((agc_job_id_t)0x0A0A)
#define AGC_JOB_AUTO_OPTICS_POSITIONING_TRACKED  ((agc_job_id_t)0x0A0B)
#define AGC_JOB_AUTO_OPTICS_POSITIONING_WAIT     ((agc_job_id_t)0x0A0C)
#define AGC_JOB_AUTO_OPTICS_SIGHTING             ((agc_job_id_t)0x0A0D)
#define AGC_JOB_CHKSDATA_ENTER                   ((agc_job_id_t)0x0A0E)
#define AGC_JOB_CHKSDATA_FLASH                   ((agc_job_id_t)0x0A0F)
#define AGC_JOB_CHKSDATA_PROCEED                 ((agc_job_id_t)0x0A10)
#define AGC_JOB_COARSE_ALIGN                     ((agc_job_id_t)0x0A11)
#define AGC_JOB_COARSE_ALIGN_COARSED             ((agc_job_id_t)0x0A12)
#define AGC_JOB_COARSTYP                         ((agc_job_id_t)0x0A13)
#define AGC_JOB_CSM_FINE_ALIGN                   ((agc_job_id_t)0x0A14)
#define AGC_JOB_CSM_FINE_ALIGN_DATA_CHECKED      ((agc_job_id_t)0x0A15)
#define AGC_JOB_CSM_FINE_ALIGN_MARKED            ((agc_job_id_t)0x0A16)
#define AGC_JOB_CSM_FINE_ALIGN_STAR_ACCEPTED     ((agc_job_id_t)0x0A17)
#define AGC_JOB_CSM_FINE_ALIGN_STAR_AT_MARK      ((agc_job_id_t)0x0A18)
#define AGC_JOB_CSM_FINE_ALIGN_STAR_CODE         ((agc_job_id_t)0x0A19)
#define AGC_JOB_CSM_FINE_ALIGN_STAR_DISPLAY      ((agc_job_id_t)0x0A1A)
#define AGC_JOB_CSM_FINE_ALIGN_STAR_LOCATED      ((agc_job_id_t)0x0A1B)
#define AGC_JOB_CSM_FINE_ALIGN_TORQUED           ((agc_job_id_t)0x0A1C)
#define AGC_JOB_CSM_LANDING_SITE_ACCEPTED        ((agc_job_id_t)0x0A1D)
#define AGC_JOB_CSM_LANDING_SITE_AXES            ((agc_job_id_t)0x0A1E)
#define AGC_JOB_CSM_LANDING_SITE_DISPLAY         ((agc_job_id_t)0x0A1F)
#define AGC_JOB_CSM_ORIENTATION_ACQUIRE_DISPLAY  ((agc_job_id_t)0x0A20)
#define AGC_JOB_CSM_ORIENTATION_COARSE           ((agc_job_id_t)0x0A21)
#define AGC_JOB_CSM_ORIENTATION_COARSE_DONE      ((agc_job_id_t)0x0A22)
#define AGC_JOB_CSM_ORIENTATION_COARSE_START     ((agc_job_id_t)0x0A23)
#define AGC_JOB_CSM_ORIENTATION_COARSE_VERB      ((agc_job_id_t)0x0A24)
#define AGC_JOB_CSM_ORIENTATION_DATA_CHECKED     ((agc_job_id_t)0x0A25)
#define AGC_JOB_CSM_ORIENTATION_FINE_DONE        ((agc_job_id_t)0x0A26)
#define AGC_JOB_CSM_ORIENTATION_FIRST_STAR       ((agc_job_id_t)0x0A27)
#define AGC_JOB_CSM_ORIENTATION_MARKED           ((agc_job_id_t)0x0A28)
#define AGC_JOB_CSM_ORIENTATION_MARK_SETUP       ((agc_job_id_t)0x0A29)
#define AGC_JOB_CSM_ORIENTATION_MARK_STAR        ((agc_job_id_t)0x0A2A)
#define AGC_JOB_CSM_ORIENTATION_SECOND_STAR      ((agc_job_id_t)0x0A2B)
#define AGC_JOB_CSM_REALIGN_PICK_STARS           ((agc_job_id_t)0x0A2C)
#define AGC_JOB_CSM_REALIGN_STARS_LOCATED        ((agc_job_id_t)0x0A2D)
#define AGC_JOB_CSM_REALIGN_STAR_SELECTION       ((agc_job_id_t)0x0A2E)
#define AGC_JOB_ENTANSWR                         ((agc_job_id_t)0x0A2F)
#define AGC_JOB_GYCRS                            ((agc_job_id_t)0x0A30)
#define AGC_JOB_GYCRS_PULSE                      ((agc_job_id_t)0x0A31)
#define AGC_JOB_GYCRS_PULSED                     ((agc_job_id_t)0x0A32)
#define AGC_JOB_GYCRS_TORQUED                    ((agc_job_id_t)0x0A33)
#define AGC_JOB_GYRO_TORQUING_PULSE              ((agc_job_id_t)0x0A34)
#define AGC_JOB_GYRO_TORQUING_PULSED             ((agc_job_id_t)0x0A35)
#define AGC_JOB_GYRO_TORQUING_RETURN             ((agc_job_id_t)0x0A36)
#define AGC_JOB_IMU_REALIGN_OPTION               ((agc_job_id_t)0x0A37)
#define AGC_JOB_IMU_REALIGN_OPTION_DISPLAY       ((agc_job_id_t)0x0A38)
#define AGC_JOB_MARKDISP                         ((agc_job_id_t)0x0A39)
#define AGC_JOB_MARK_COMPLETE_DISPLAY            ((agc_job_id_t)0x0A3A)
#define AGC_JOB_MARK_REDISPLAY                   ((agc_job_id_t)0x0A3B)
#define AGC_JOB_MARK_REQUEST_DISPLAY             ((agc_job_id_t)0x0A3C)
#define AGC_JOB_MARK_REQUEST_FLASH               ((agc_job_id_t)0x0A3D)
#define AGC_JOB_PLANET_FLASH                     ((agc_job_id_t)0x0A3E)
#define AGC_JOB_PLANET_LOADED                    ((agc_job_id_t)0x0A3F)
#define AGC_JOB_PLANET_RESUME                    ((agc_job_id_t)0x0A40)
#define AGC_JOB_REALIGN_GIMBAL_DISPLAY           ((agc_job_id_t)0x0A41)
#define AGC_JOB_REALIGN_NOMINAL_ORIENTATION      ((agc_job_id_t)0x0A42)
#define AGC_JOB_REALIGN_TIME_FLASH               ((agc_job_id_t)0x0A43)
#define AGC_JOB_REALIGN_TIME_PROCEED             ((agc_job_id_t)0x0A44)
#define AGC_JOB_SBANDANT_CM_RECYCLE              ((agc_job_id_t)0x0A45)
#define AGC_JOB_SBANDANT_LM_RECYCLE              ((agc_job_id_t)0x0A46)
#define AGC_JOB_SBAND_B5OFF                      ((agc_job_id_t)0x0A47)
#define AGC_JOB_SIGHTING_MARK_CODE_CLEAN         ((agc_job_id_t)0x0A48)
#define AGC_JOB_SIGHTING_MARK_CODE_DISPLAY       ((agc_job_id_t)0x0A49)
#define AGC_JOB_SIGHTING_MARK_CODE_ENTERED       ((agc_job_id_t)0x0A4A)
#define AGC_JOB_SIGHTING_MARK_MARKED             ((agc_job_id_t)0x0A4B)
#define AGC_JOB_TERMSXT                          ((agc_job_id_t)0x0A4C)

#define AGC_TASK_ENDMARKS                        ((agc_task_id_t)0x0A01)
#define AGC_TASK_MARKDIF                         ((agc_task_id_t)0x0A02)

#endif /* ALIGN_PENDING_H */
