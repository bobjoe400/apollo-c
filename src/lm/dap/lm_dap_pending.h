/*
 * lm_dap_pending.h -- shared symbols the LM DAP needs that other lanes have
 * not published yet (see docs/agents/_common.md naming convention).
 *
 * INTEGRATION: when agc_flags_lm.h / agc_flags_common.h, the erasable
 * accessors and the job ids exist, replace the body of this file with
 *     #include "agc/agc_flags_common.h"
 *     #include "agc/agc_flags_lm.h"
 * and delete the definitions below. Every name here follows the convention,
 * so no DAP source file needs to change.
 *
 * Flag ids are encoded (flagword << 4) | (AGC bit number - 1) as a
 * placeholder; the authoritative encoding belongs to the state-flags agent.
 */
#ifndef LM_DAP_PENDING_H
#define LM_DAP_PENDING_H

#include <stdint.h>
#include "agc/agc_word.h"
#include "agc/agc_state.h"
#include "agc/agc_jobs.h"

/* ---- flags (FLAGWORD_ASSIGNMENTS.agc) ----------------------------------- */
#define AGC_FLAG_NEEDLFLG ((agc_flag_id_t)((0u << 4) | 3u))   /* 011 FW0 B4  */
#define AGC_FLAG_SNUFFER  ((agc_flag_id_t)((5u << 4) | 12u))  /* 077 FW5 B13 */
#define AGC_FLAG_APSFLAG  ((agc_flag_id_t)((10u << 4) | 12u)) /* 152 FW10 B13 */
/* DAPBOOLS = FLGWRD13 (flags 195-209)                                      */
#define AGC_FLAG_PULSEFLG ((agc_flag_id_t)((13u << 4) | 14u)) /* PULSES   B15 */
#define AGC_FLAG_USEQRFLG ((agc_flag_id_t)((13u << 4) | 13u)) /* USEQRJTS B14 */
#define AGC_FLAG_CSMDKFLG ((agc_flag_id_t)((13u << 4) | 12u)) /* CSMDOCKD B13 */
#define AGC_FLAG_OURRCFLG ((agc_flag_id_t)((13u << 4) | 11u)) /* OURRCBIT B12 */
#define AGC_FLAG_ACC4_2FL ((agc_flag_id_t)((13u << 4) | 10u)) /* ACC4OR2X B11 */
#define AGC_FLAG_AORBTFLG ((agc_flag_id_t)((13u << 4) | 9u))  /* AORBTRAN B10 */
#define AGC_FLAG_XOVINFLG ((agc_flag_id_t)((13u << 4) | 8u))  /* XOVINHIB B9  */
#define AGC_FLAG_DRIFTDFL ((agc_flag_id_t)((13u << 4) | 7u))  /* DRIFTBIT B8  */
#define AGC_FLAG_ULLAGFLG ((agc_flag_id_t)((13u << 4) | 5u))  /* ULLAGER  B6  */
#define AGC_FLAG_AORBSFLG ((agc_flag_id_t)((13u << 4) | 4u))  /* AORBSYST B5  */
#define AGC_FLAG_DBSELFLG ((agc_flag_id_t)((13u << 4) | 3u))  /* DBSELECT B4  */
#define AGC_FLAG_ACCOKFLG ((agc_flag_id_t)((13u << 4) | 2u))  /* ACCSOKAY B3  */

/* ---- job ids (agc_jobs.h, integrated by the lead) ----------------------- */
#define AGC_JOB_1_ACCJOB ((agc_job_id_t)101)                  /* 1/ACCJOB    */
#define AGC_JOB_1_ACCSET ((agc_job_id_t)102)                  /* 1/ACCSET    */

/* ---- erasables owned by other lanes ------------------------------------ */
uint16_t agc_erase_cdux(void);        /* CDUX, 15-bit 2's complement at pi  */
uint16_t agc_erase_cduy(void);
uint16_t agc_erase_cduz(void);
uint16_t agc_erase_cphi(void);        /* THETAD desired gimbal angles, 2's c */
uint16_t agc_erase_ctheta(void);
uint16_t agc_erase_cpsi(void);
agc_sp_t agc_erase_m11(void);         /* gimbal-to-pilot matrix, at 1 (T4RUPT) */
agc_sp_t agc_erase_m21(void);
agc_sp_t agc_erase_m31(void);
agc_sp_t agc_erase_m22(void);
agc_sp_t agc_erase_m32(void);
agc_dp_t agc_erase_mass(void);        /* MASS, DP at 2^16 kg                */
void     agc_erase_set_mass(agc_dp_t v);
agc_sp_t agc_erase_lemmass(void);     /* LEMMASS, at 2^16 kg                */
void     agc_erase_set_lemmass(agc_sp_t v);
agc_sp_t agc_erase_csmmass(void);     /* CSMMASS, at 2^16 kg                */
agc_dp_t agc_erase_abdelv(void);      /* ABDELV, DP at 2^13 cm/s^2          */
uint16_t agc_erase_imodes33(void);    /* IMODES33 (bit 6: IMU not usable)   */
agc_sp_t agc_erase_p_rhcctr(void);    /* RHC counters (counter cells 042-044) */
agc_sp_t agc_erase_q_rhcctr(void);
agc_sp_t agc_erase_r_rhcctr(void);
void     agc_erase_set_p_rhcctr(agc_sp_t v);
void     agc_erase_set_q_rhcctr(agc_sp_t v);
void     agc_erase_set_r_rhcctr(agc_sp_t v);
agc_sp_t agc_erase_cduxcmd(void);     /* IMU error counter drive outputs    */
agc_sp_t agc_erase_cduycmd(void);
agc_sp_t agc_erase_cduzcmd(void);
void     agc_erase_set_cduxcmd(agc_sp_t v);
void     agc_erase_set_cduycmd(agc_sp_t v);
void     agc_erase_set_cduzcmd(agc_sp_t v);

#endif /* LM_DAP_PENDING_H */
