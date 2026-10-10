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

#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/io/service_internal.h"

/* ==== state-flags: shared erasables not yet in agc_state.h =============== */
agc_sp_t agc_erase_option1(int index); /* OPTION1 +index (N06), 0..2 */
void     agc_erase_set_option1(int index, agc_sp_t v);
agc_dp_t agc_erase_samptime(void); /* SAMPTIME cs B28            */
void     agc_erase_set_samptime(agc_dp_t v);
agc_dp_t agc_erase_tland(void); /* TLAND cs B28 (guid-landing)*/
void     agc_erase_set_tland(agc_dp_t v);
uint16_t agc_erase_aotaz(int index);   /* AOTAZ -1+index, cdu (pad)  */
uint16_t agc_erase_aotel(int index);   /* AOTEL -1+index, cdu (pad)  */
agc_sp_t agc_erase_landmark(void);     /* LANDMARK (CM, prog-rendz)  */
agc_sp_t agc_erase_cduchkwd(void);     /* CDUCHKWD cs (CM, pad)      */
agc_sp_t agc_erase_swsample(void);     /* SWSAMPLE (CM, T4RUPT)      */
uint16_t agc_erase_agsword(void);      /* AGSWORD old downlist code  */
agc_sp_t agc_erase_optionx(int index); /* OPTIONX +index (N12), LM   */
void     agc_erase_set_optionx(int index, agc_sp_t v);
agc_dp_t agc_erase_dsptemx(void); /* DSPTEMX (N16 time), cs B28 */
void     agc_erase_set_dsptemx(agc_dp_t v);
void     agc_erase_set_mrkbuf(int buf, int index, uint16_t v); /* MRKBUF1/2 */
uint16_t agc_erase_mrkbuf(int buf, int index);
void     agc_erase_set_markdown(int index, uint16_t v); /* MARKDOWN +0..6   */
void     agc_erase_set_mark2dwn(int index, uint16_t v); /* MARK2DWN +0..6   */

/* ==== exec-sched: VAC areas claimed by the mark system ==================== */
/* MARKOK/MKVAC: first free VACnUSE, marked occupied (CCS VACnUSE loop);
 * AGC_STATUS_FULL when none (the caller BAILOUTs 01207).                   */
agc_status_t agc_exec_vac_claim(int *vac_index);
agc_status_t agc_exec_vac_release(int vac_index); /* TS VACnUSE          */

/* ==== prog-rendz (P20-P25, Comanche) ===================================== */
agc_status_t agc_rendz_llasrd(void);  /* LLASRD: LAT,LONG,ALT -> N89 regs */
agc_status_t agc_rendz_llasrda(void); /* LLASRDA: N89 regs -> LAT,LONG,ALT*/
/* R61CSM: CSM tracking attitude; returns after the maneuver/update.        */
agc_status_t agc_rendz_tracking_attitude_csm(const agc_job_ctx_t *ctx,
                                             agc_job_id_t         ret_job);

/* ==== provisional job and task identifiers ================================ */
#define AGC_JOB_AGS_INITIALIZATION_COMPUTE          ((agc_job_id_t) 0x0A01)
#define AGC_JOB_AGS_INITIALIZATION_CSM_STATE        ((agc_job_id_t) 0x0A02)
#define AGC_JOB_AGS_INITIALIZATION_END              ((agc_job_id_t) 0x0A03)
#define AGC_JOB_AGS_INITIALIZATION_IMU_ZEROED       ((agc_job_id_t) 0x0A04)
#define AGC_JOB_AGS_INITIALIZATION_LM_STATE         ((agc_job_id_t) 0x0A05)
#define AGC_JOB_AGS_INITIALIZATION_NEW_ZERO         ((agc_job_id_t) 0x0A06)
#define AGC_JOB_AGS_INITIALIZATION_SENT             ((agc_job_id_t) 0x0A07)
#define AGC_JOB_AGS_INITIALIZATION_ZERO_IMU         ((agc_job_id_t) 0x0A08)
#define AGC_JOB_ALIGN_COARSE_DONE                   ((agc_job_id_t) 0x0A09)
#define AGC_JOB_ALIGN_FINE_DONE                     ((agc_job_id_t) 0x0A0A)
#define AGC_JOB_ALTERNATE_LOS_ANGLES_DISPLAY        ((agc_job_id_t) 0x0A0B)
#define AGC_JOB_ALTERNATE_LOS_MARKED                ((agc_job_id_t) 0x0A0C)
#define AGC_JOB_ALTERNATE_LOS_MARK_REQUEST          ((agc_job_id_t) 0x0A0D)
#define AGC_JOB_ALTERNATE_LOS_REMARK                ((agc_job_id_t) 0x0A0E)
#define AGC_JOB_ALTERNATE_LOS_RESERVE               ((agc_job_id_t) 0x0A0F)
#define AGC_JOB_AOT_MARK_CHECK                      ((agc_job_id_t) 0x0A10)
#define AGC_JOB_AOT_MARK_COAS_DISPLAY               ((agc_job_id_t) 0x0A11)
#define AGC_JOB_AOT_MARK_COAS_LOADED                ((agc_job_id_t) 0x0A12)
#define AGC_JOB_AOT_MARK_DETENT                     ((agc_job_id_t) 0x0A13)
#define AGC_JOB_AOT_MARK_KILL                       ((agc_job_id_t) 0x0A14)
#define AGC_JOB_AOT_MARK_NEXT_ACTION                ((agc_job_id_t) 0x0A15)
#define AGC_JOB_AOT_MARK_STAR_DISPLAY               ((agc_job_id_t) 0x0A16)
#define AGC_JOB_AOT_MARK_SURFACE_ANSWER             ((agc_job_id_t) 0x0A17)
#define AGC_JOB_AOT_MARK_SURFACE_END                ((agc_job_id_t) 0x0A18)
#define AGC_JOB_AUTO_OPTICS_POSITIONING_DELAYED     ((agc_job_id_t) 0x0A19)
#define AGC_JOB_AUTO_OPTICS_POSITIONING_RESUME      ((agc_job_id_t) 0x0A1A)
#define AGC_JOB_AUTO_OPTICS_POSITIONING_TERMINATE   ((agc_job_id_t) 0x0A1B)
#define AGC_JOB_AUTO_OPTICS_POSITIONING_TRACKED     ((agc_job_id_t) 0x0A1C)
#define AGC_JOB_AUTO_OPTICS_POSITIONING_WAIT        ((agc_job_id_t) 0x0A1D)
#define AGC_JOB_AUTO_OPTICS_SIGHTING                ((agc_job_id_t) 0x0A1E)
#define AGC_JOB_CHKSDATA_ENTER                      ((agc_job_id_t) 0x0A1F)
#define AGC_JOB_CHKSDATA_FLASH                      ((agc_job_id_t) 0x0A20)
#define AGC_JOB_CHKSDATA_PROCEED                    ((agc_job_id_t) 0x0A21)
#define AGC_JOB_COARSE_ALIGN                        ((agc_job_id_t) 0x0A22)
#define AGC_JOB_COARSE_ALIGN_COARSED                ((agc_job_id_t) 0x0A23)
#define AGC_JOB_COARSTYP                            ((agc_job_id_t) 0x0A24)
#define AGC_JOB_CSM_FINE_ALIGN                      ((agc_job_id_t) 0x0A25)
#define AGC_JOB_CSM_FINE_ALIGN_DATA_CHECKED         ((agc_job_id_t) 0x0A26)
#define AGC_JOB_CSM_FINE_ALIGN_MARKED               ((agc_job_id_t) 0x0A27)
#define AGC_JOB_CSM_FINE_ALIGN_STAR_ACCEPTED        ((agc_job_id_t) 0x0A28)
#define AGC_JOB_CSM_FINE_ALIGN_STAR_AT_MARK         ((agc_job_id_t) 0x0A29)
#define AGC_JOB_CSM_FINE_ALIGN_STAR_CODE            ((agc_job_id_t) 0x0A2A)
#define AGC_JOB_CSM_FINE_ALIGN_STAR_DISPLAY         ((agc_job_id_t) 0x0A2B)
#define AGC_JOB_CSM_FINE_ALIGN_STAR_LOCATED         ((agc_job_id_t) 0x0A2C)
#define AGC_JOB_CSM_FINE_ALIGN_TORQUED              ((agc_job_id_t) 0x0A2D)
#define AGC_JOB_CSM_LANDING_SITE_ACCEPTED           ((agc_job_id_t) 0x0A2E)
#define AGC_JOB_CSM_LANDING_SITE_AXES               ((agc_job_id_t) 0x0A2F)
#define AGC_JOB_CSM_LANDING_SITE_DISPLAY            ((agc_job_id_t) 0x0A30)
#define AGC_JOB_CSM_ORIENTATION_ACQUIRE_DISPLAY     ((agc_job_id_t) 0x0A31)
#define AGC_JOB_CSM_ORIENTATION_COARSE              ((agc_job_id_t) 0x0A32)
#define AGC_JOB_CSM_ORIENTATION_COARSE_DONE         ((agc_job_id_t) 0x0A33)
#define AGC_JOB_CSM_ORIENTATION_COARSE_START        ((agc_job_id_t) 0x0A34)
#define AGC_JOB_CSM_ORIENTATION_COARSE_VERB         ((agc_job_id_t) 0x0A35)
#define AGC_JOB_CSM_ORIENTATION_DATA_CHECKED        ((agc_job_id_t) 0x0A36)
#define AGC_JOB_CSM_ORIENTATION_FINE_DONE           ((agc_job_id_t) 0x0A37)
#define AGC_JOB_CSM_ORIENTATION_FIRST_STAR          ((agc_job_id_t) 0x0A38)
#define AGC_JOB_CSM_ORIENTATION_MARKED              ((agc_job_id_t) 0x0A39)
#define AGC_JOB_CSM_ORIENTATION_MARK_SETUP          ((agc_job_id_t) 0x0A3A)
#define AGC_JOB_CSM_ORIENTATION_MARK_STAR           ((agc_job_id_t) 0x0A3B)
#define AGC_JOB_CSM_ORIENTATION_SECOND_STAR         ((agc_job_id_t) 0x0A3C)
#define AGC_JOB_CSM_REALIGN_PICK_STARS              ((agc_job_id_t) 0x0A3D)
#define AGC_JOB_CSM_REALIGN_STARS_LOCATED           ((agc_job_id_t) 0x0A3E)
#define AGC_JOB_CSM_REALIGN_STAR_SELECTION          ((agc_job_id_t) 0x0A3F)
#define AGC_JOB_ENTANSWR                            ((agc_job_id_t) 0x0A40)
#define AGC_JOB_GYCRS                               ((agc_job_id_t) 0x0A41)
#define AGC_JOB_GYCRS_PULSE                         ((agc_job_id_t) 0x0A42)
#define AGC_JOB_GYCRS_PULSED                        ((agc_job_id_t) 0x0A43)
#define AGC_JOB_GYCRS_TORQUED                       ((agc_job_id_t) 0x0A44)
#define AGC_JOB_GYRO_TORQUING_PULSE                 ((agc_job_id_t) 0x0A45)
#define AGC_JOB_GYRO_TORQUING_PULSED                ((agc_job_id_t) 0x0A46)
#define AGC_JOB_GYRO_TORQUING_RETURN                ((agc_job_id_t) 0x0A47)
#define AGC_JOB_IMU_REALIGN_OPTION                  ((agc_job_id_t) 0x0A48)
#define AGC_JOB_IMU_REALIGN_OPTION_DISPLAY          ((agc_job_id_t) 0x0A49)
#define AGC_JOB_LM_AUTO_OPTICS_AZ_EL_DISPLAY        ((agc_job_id_t) 0x0A4A)
#define AGC_JOB_LM_AUTO_OPTICS_AZ_EL_LOADED         ((agc_job_id_t) 0x0A4B)
#define AGC_JOB_LM_AUTO_OPTICS_MANEUVERED           ((agc_job_id_t) 0x0A4C)
#define AGC_JOB_LM_AUTO_OPTICS_ORIENT               ((agc_job_id_t) 0x0A4D)
#define AGC_JOB_LM_AUTO_OPTICS_STAR_DISPLAY         ((agc_job_id_t) 0x0A4E)
#define AGC_JOB_LM_AUTO_OPTICS_STAR_LOCATED         ((agc_job_id_t) 0x0A4F)
#define AGC_JOB_LM_FINE_ALIGN_DATA_CHECKED          ((agc_job_id_t) 0x0A50)
#define AGC_JOB_LM_FINE_ALIGN_FIRST_LOCATED         ((agc_job_id_t) 0x0A51)
#define AGC_JOB_LM_FINE_ALIGN_FIRST_STAR            ((agc_job_id_t) 0x0A52)
#define AGC_JOB_LM_FINE_ALIGN_MARK                  ((agc_job_id_t) 0x0A53)
#define AGC_JOB_LM_FINE_ALIGN_MARKED                ((agc_job_id_t) 0x0A54)
#define AGC_JOB_LM_FINE_ALIGN_PICK_STARS            ((agc_job_id_t) 0x0A55)
#define AGC_JOB_LM_FINE_ALIGN_SECOND_LOCATED        ((agc_job_id_t) 0x0A56)
#define AGC_JOB_LM_FINE_ALIGN_STARS_LOCATED         ((agc_job_id_t) 0x0A57)
#define AGC_JOB_LM_FINE_ALIGN_STAR_ACQUIRE          ((agc_job_id_t) 0x0A58)
#define AGC_JOB_LM_FINE_ALIGN_TORQUED               ((agc_job_id_t) 0x0A59)
#define AGC_JOB_LM_LANDING_SITE_ACCEPTED            ((agc_job_id_t) 0x0A5A)
#define AGC_JOB_LM_LANDING_SITE_DISPLAY             ((agc_job_id_t) 0x0A5B)
#define AGC_JOB_LM_ORIENTATION_COARSE               ((agc_job_id_t) 0x0A5C)
#define AGC_JOB_LM_ORIENTATION_COARSE_DONE          ((agc_job_id_t) 0x0A5D)
#define AGC_JOB_LM_ORIENTATION_COARSE_START         ((agc_job_id_t) 0x0A5E)
#define AGC_JOB_LM_ORIENTATION_COARSE_VERB          ((agc_job_id_t) 0x0A5F)
#define AGC_JOB_LM_ORIENTATION_DATA_CHECKED         ((agc_job_id_t) 0x0A60)
#define AGC_JOB_LM_ORIENTATION_FIRST_STAR           ((agc_job_id_t) 0x0A61)
#define AGC_JOB_LM_ORIENTATION_MARKED               ((agc_job_id_t) 0x0A62)
#define AGC_JOB_LM_ORIENTATION_MARK_SETUP           ((agc_job_id_t) 0x0A63)
#define AGC_JOB_LM_ORIENTATION_SECOND_STAR          ((agc_job_id_t) 0x0A64)
#define AGC_JOB_LM_REALIGN_SITE_ORIENTATION         ((agc_job_id_t) 0x0A65)
#define AGC_JOB_LUNAR_GRAVITY_ACCEPTED              ((agc_job_id_t) 0x0A66)
#define AGC_JOB_LUNAR_GRAVITY_ACCUMULATE            ((agc_job_id_t) 0x0A67)
#define AGC_JOB_LUNAR_GRAVITY_COARSED               ((agc_job_id_t) 0x0A68)
#define AGC_JOB_LUNAR_GRAVITY_MEASURED              ((agc_job_id_t) 0x0A69)
#define AGC_JOB_LUNAR_GRAVITY_RECYCLE               ((agc_job_id_t) 0x0A6A)
#define AGC_JOB_LUNAR_GRAVITY_ROTATE                ((agc_job_id_t) 0x0A6B)
#define AGC_JOB_LUNAR_SURFACE_ALIGNED               ((agc_job_id_t) 0x0A6C)
#define AGC_JOB_LUNAR_SURFACE_COARSE_TRIMMED        ((agc_job_id_t) 0x0A6D)
#define AGC_JOB_LUNAR_SURFACE_COARSE_TRIM_DISPLAY   ((agc_job_id_t) 0x0A6E)
#define AGC_JOB_LUNAR_SURFACE_COARSE_TRIM_GO        ((agc_job_id_t) 0x0A6F)
#define AGC_JOB_LUNAR_SURFACE_DATA_CHECKED          ((agc_job_id_t) 0x0A70)
#define AGC_JOB_LUNAR_SURFACE_FIRST_STAR            ((agc_job_id_t) 0x0A71)
#define AGC_JOB_LUNAR_SURFACE_OPTION                ((agc_job_id_t) 0x0A72)
#define AGC_JOB_LUNAR_SURFACE_OPTIONS_CHOSEN        ((agc_job_id_t) 0x0A73)
#define AGC_JOB_LUNAR_SURFACE_OPTIONS_DISPLAY       ((agc_job_id_t) 0x0A74)
#define AGC_JOB_LUNAR_SURFACE_OPTION_DISPLAY        ((agc_job_id_t) 0x0A75)
#define AGC_JOB_LUNAR_SURFACE_POST                  ((agc_job_id_t) 0x0A76)
#define AGC_JOB_LUNAR_SURFACE_POST_ENTER            ((agc_job_id_t) 0x0A77)
#define AGC_JOB_LUNAR_SURFACE_REALIGN               ((agc_job_id_t) 0x0A78)
#define AGC_JOB_LUNAR_SURFACE_SECOND_STAR           ((agc_job_id_t) 0x0A79)
#define AGC_JOB_LUNAR_SURFACE_SITE_ORIENTATION      ((agc_job_id_t) 0x0A7A)
#define AGC_JOB_LUNAR_SURFACE_SITE_UPDATED          ((agc_job_id_t) 0x0A7B)
#define AGC_JOB_LUNAR_SURFACE_STAR_ACQUISITION      ((agc_job_id_t) 0x0A7C)
#define AGC_JOB_LUNAR_SURFACE_STAR_CHOSEN           ((agc_job_id_t) 0x0A7D)
#define AGC_JOB_LUNAR_SURFACE_STAR_MARK             ((agc_job_id_t) 0x0A7E)
#define AGC_JOB_LUNAR_SURFACE_STAR_MARKED           ((agc_job_id_t) 0x0A7F)
#define AGC_JOB_LUNAR_SURFACE_STAR_POSITIONED       ((agc_job_id_t) 0x0A80)
#define AGC_JOB_LUNAR_SURFACE_TIME_DISPLAY          ((agc_job_id_t) 0x0A81)
#define AGC_JOB_LUNAR_SURFACE_TIME_ENTERED          ((agc_job_id_t) 0x0A82)
#define AGC_JOB_LUNAR_SURFACE_TORQUE_TEST           ((agc_job_id_t) 0x0A83)
#define AGC_JOB_LUNAR_SURFACE_TRIMMED               ((agc_job_id_t) 0x0A84)
#define AGC_JOB_LUNAR_SURFACE_TRIM_PULSE            ((agc_job_id_t) 0x0A85)
#define AGC_JOB_LUNAR_SURFACE_TRIM_RETRY            ((agc_job_id_t) 0x0A86)
#define AGC_JOB_MARKDISP                            ((agc_job_id_t) 0x0A87)
#define AGC_JOB_MARK_COMPLETE_DISPLAY               ((agc_job_id_t) 0x0A88)
#define AGC_JOB_MARK_REDISPLAY                      ((agc_job_id_t) 0x0A89)
#define AGC_JOB_MARK_REQUEST_DISPLAY                ((agc_job_id_t) 0x0A8A)
#define AGC_JOB_MARK_REQUEST_FLASH                  ((agc_job_id_t) 0x0A8B)
#define AGC_JOB_PLANET_FLASH                        ((agc_job_id_t) 0x0A8C)
#define AGC_JOB_PLANET_LOADED                       ((agc_job_id_t) 0x0A8D)
#define AGC_JOB_PLANET_RESUME                       ((agc_job_id_t) 0x0A8E)
#define AGC_JOB_REALIGN_GIMBAL_DISPLAY              ((agc_job_id_t) 0x0A8F)
#define AGC_JOB_REALIGN_NOMINAL_ORIENTATION         ((agc_job_id_t) 0x0A90)
#define AGC_JOB_REALIGN_TIME_FLASH                  ((agc_job_id_t) 0x0A91)
#define AGC_JOB_REALIGN_TIME_PROCEED                ((agc_job_id_t) 0x0A92)
#define AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_COMPUTE   ((agc_job_id_t) 0x0A93)
#define AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_CSM_STATE ((agc_job_id_t) 0x0A94)
#define AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_END       ((agc_job_id_t) 0x0A95)
#define AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_LM_STATE  ((agc_job_id_t) 0x0A96)
#define AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_MANEUVER  ((agc_job_id_t) 0x0A97)
#define AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_OPTION    ((agc_job_id_t) 0x0A98)
#define AGC_JOB_SBANDANT_CM_RECYCLE                 ((agc_job_id_t) 0x0A99)
#define AGC_JOB_SBANDANT_LM_RECYCLE                 ((agc_job_id_t) 0x0A9A)
#define AGC_JOB_SBAND_B5OFF                         ((agc_job_id_t) 0x0A9B)
#define AGC_JOB_SIGHTING_MARK_CODE_CLEAN            ((agc_job_id_t) 0x0A9C)
#define AGC_JOB_SIGHTING_MARK_CODE_DISPLAY          ((agc_job_id_t) 0x0A9D)
#define AGC_JOB_SIGHTING_MARK_CODE_ENTERED          ((agc_job_id_t) 0x0A9E)
#define AGC_JOB_SIGHTING_MARK_MARKED                ((agc_job_id_t) 0x0A9F)
#define AGC_JOB_TERMSXT                             ((agc_job_id_t) 0x0AA0)

#define AGC_TASK_AOT_MARK_RELEASE     ((agc_task_id_t) 0x0A01)
#define AGC_TASK_ENDMARKS             ((agc_task_id_t) 0x0A02)
#define AGC_TASK_LUNAR_GRAVITY_SAMPLE ((agc_task_id_t) 0x0A03)
#define AGC_TASK_MARKDIF              ((agc_task_id_t) 0x0A04)

#endif /* ALIGN_PENDING_H */
