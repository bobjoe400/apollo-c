/*
 * pending.h -- TEMPORARY ids and prototypes for the extended verbs and the
 * update program (src/io/verbs/).  Everything here is listed in
 * docs/modules/sys-misc.md under "Requested jobs-tasks" / "Requested core
 * additions"; at integration the ids move to agc/exec/jobs.h and the
 * dispatch switch, and this file goes away.  Where another lane's shim
 * already names an entry, the same name and value are used here, so the
 * definitions are token-identical.
 */
#ifndef AGC_IO_VERBS_PENDING_H
#define AGC_IO_VERBS_PENDING_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"

/* ---- jobs owned by sys-misc (0x0340..0x037F) --------------------------- */
#define AGC_JOB_EXTENDED_VERB             ((agc_job_id_t)0x0340) /* GOEXTVB / LST2FAN       */
#define AGC_JOB_EXTENDED_VERB_END         ((agc_job_id_t)0x0341) /* ENDEXT (TERMEXTV)       */
#define AGC_JOB_EXTENDED_VERB_GOPIN       ((agc_job_id_t)0x0342) /* GOPIN (PINBRNCH)        */
#define AGC_JOB_KEYBOARD_COARSE_ALIGN     ((agc_job_id_t)0x0343) /* V41 ICORK2              */
#define AGC_JOB_KEYBOARD_COARSE_DRIVE     ((agc_job_id_t)0x0344) /* V41 IMUCOARS + IMUSTALL */
#define AGC_JOB_KEYBOARD_FINE_ALIGN       ((agc_job_id_t)0x0345) /* V42 after V25N93        */
#define AGC_JOB_KEYBOARD_FINE_MODE        ((agc_job_id_t)0x0346) /* V42 IMUFINE + IMUSTALL  */
#define AGC_JOB_KEYBOARD_GYRO_TORQUE      ((agc_job_id_t)0x0347) /* V42 FINEK2 (IMUPULSE)   */
#define AGC_JOB_CLOCK_ALIGN_UPDATE        ((agc_job_id_t)0x0348) /* V55 UPDATIME            */
#define AGC_JOB_SHOW_BANKSUM_DISPLAY      ((agc_job_id_t)0x0349) /* V91 NOKILL              */
#define AGC_JOB_SHOW_BANKSUM_NEXT         ((agc_job_id_t)0x034A) /* V91 NXTBNK              */
#define AGC_JOB_SHOW_BANKSUM_END          ((agc_job_id_t)0x034B) /* V91 terminate           */
#define AGC_JOB_ATTITUDE_ERROR_LOAD       ((agc_job_id_t)0x034C) /* V43 after V25N22        */
#define AGC_JOB_ATTITUDE_ERROR_TERMINATE  ((agc_job_id_t)0x034D) /* V43 CSM TRMATTCK        */
#define AGC_JOB_DAP_DATA_LOAD_NOUN46      ((agc_job_id_t)0x034E) /* R03 DONOUN46 / DAPDATA1 */
#define AGC_JOB_DAP_DATA_LOAD_CONFIG      ((agc_job_id_t)0x034F) /* R03 after N46 (MAXOUT / DPDAT1) */
#define AGC_JOB_DAP_DATA_LOAD_NOUN47      ((agc_job_id_t)0x0350) /* R03 DONOUN47 / DAPDATA2 */
#define AGC_JOB_DAP_DATA_LOAD_MASS        ((agc_job_id_t)0x0351) /* R03 after N47 (MASSPROP / DAPDAT2) */
#define AGC_JOB_DAP_DATA_LOAD_NOUN48      ((agc_job_id_t)0x0352) /* R03 DONOUN48 / DAPDATA3 */
#define AGC_JOB_DAP_DATA_LOAD_TRIM        ((agc_job_id_t)0x0353) /* R03 LM DPDAT3           */
#define AGC_JOB_DAP_DATA_LOAD_END         ((agc_job_id_t)0x0354) /* R03 LM ENDR03 (V34)     */
#define AGC_JOB_STATE_VECTOR_COPY         ((agc_job_id_t)0x0355) /* V66 ATTACHIT / V47 LMTOCM */
#define AGC_JOB_TRACKING_TERMINATE        ((agc_job_id_t)0x0356) /* V56 CSM after INTSTALL  */
#define AGC_JOB_ATTITUDE_ERROR_ENABLE     ((agc_job_id_t)0x0357) /* V43 LM after EXDSPRET   */
#define AGC_JOB_UPLINK_UPDATE             ((agc_job_id_t)0x0360) /* V70-V73 entry           */
#define AGC_JOB_UPLINK_UPDATE_LOAD        ((agc_job_id_t)0x0361) /* after a V21N01 load     */
#define AGC_JOB_UPLINK_UPDATE_TERMINATE   ((agc_job_id_t)0x0362) /* V34: UPOUT +1           */
#define AGC_JOB_UPLINK_UPDATE_PROCEED     ((agc_job_id_t)0x0363) /* V33 on a V21N01/N02     */
#define AGC_JOB_UPLINK_UPDATE_EXECUTE     ((agc_job_id_t)0x0364) /* V33 after the V21N01 index */
/* reused names from other lanes' shims (same value) */
#define AGC_JOB_CREW_DEFINED_MANEUVER_DISPLAY ((agc_job_id_t)0x331)  /* R62DISP (V49)  */
#define AGC_JOB_STRKTSTI                  ((agc_job_id_t)0x0D21) /* V68 stroke test init  */
#define AGC_JOB_COMPVER                   ((agc_job_id_t)0x0E29) /* V65 GCOMPVER (4.15)   */
/* entries of other lanes that have no id yet (names from NAMING.md)       */
#define AGC_JOB_RENDEZVOUS_PARAMETER_DISPLAY ((agc_job_id_t)0x0368) /* R31CALL (CSM V83/V85)  */
#define AGC_JOB_OUT_OF_PLANE_DISPLAY      ((agc_job_id_t)0x0369) /* R36 (V90)               */
#define AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE ((agc_job_id_t)0x036A) /* V89CALL (R63)           */
#define AGC_JOB_W_MATRIX_MONITOR          ((agc_job_id_t)0x036B) /* V67CALL                 */
#define AGC_JOB_SIGHTING_MARK             ((agc_job_id_t)0x036C) /* R21CSM (V57)            */
#define AGC_JOB_BACKUP_SIGHTING_MARK      ((agc_job_id_t)0x036D) /* R23CSM (V54)            */
#define AGC_JOB_GYROCOMPASS_AZIMUTH_CHANGE ((agc_job_id_t)0x036E) /* AZMTHCG1 (V78)         */
#define AGC_JOB_SYSTEMS_TEST              ((agc_job_id_t)0x036F) /* REDO (V92, P07)         */
#define AGC_JOB_AGS_INITIALIZATION        ((agc_job_id_t)0x0370) /* AGSINIT (LM V47, R47)   */
#define AGC_JOB_RR_LOS_DISPLAY            ((agc_job_id_t)0x0371) /* RRLOSDSP (LM V85)       */
#define AGC_JOB_SBANDANT_CM               ((agc_job_id_t)0x0372) /* SBANDANT (CM V64), agc_job_sbandant_cm */
#define AGC_JOB_SBANDANT_LM               ((agc_job_id_t)0x0373) /* SBANDANT (LM V64), agc_job_sbandant_lm */
#define AGC_TASK_RENDEZVOUS_PARAMETER_DISPLAY ((agc_task_id_t)0x0368) /* R31CALL (LM V83)   */
/* tasks owned by sys-misc */
#define AGC_TASK_ATTITUDE_ERROR_OUTPUT    ((agc_task_id_t)0x0307) /* V43 ATTCK1 / ATTCK2    */

/* ---- shared parts of the extended verbs (extended_verbs.c) -------------- */
/* The verb and noun GOEXTVB was entered with (VERBREG, NOUNREG).          */
int          agc_extverb_verb(void);
int          agc_extverb_noun(void);
agc_status_t agc_extverb_testxact(bool *ok);   /* TESTXACT: false = ALM/END done */
agc_status_t agc_extverb_alm_end(void);        /* ALM/END: FALTON, PINBRNCH     */
agc_status_t agc_extverb_gopin(void);          /* GOPIN: PINBRNCH               */
bool         agc_extverb_chkpooh(void);        /* CHKPOOH: P00 (ALM/END if not) */
bool         agc_extverb_ckmodcad(void);       /* CKMODCAD: no IMU mode user     */
agc_status_t agc_extverb_goloadlv(void);       /* GOLOADLV: please-perform done */
agc_status_t agc_extverb_upflag(agc_flag_id_t f);   /* TC UPFLAG, TC GOPIN       */
agc_status_t agc_extverb_downflag(agc_flag_id_t f); /* TC DOWNFLAG, TC GOPIN     */
agc_status_t agc_extverb_schedule(agc_prio_t prio, agc_job_id_t job, bool vac);
/* Request a flashing display for the running verb (GOXDSPF = GOMARKF).    */
agc_status_t agc_extverb_flash(const agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun,
                               agc_job_id_t on_proceed, agc_job_id_t on_enter);
/* EXDSPRET: show our verb again and continue at next.                     */
agc_status_t agc_extverb_redisplay(const agc_job_ctx_t *ctx, uint8_t verb,
                                   agc_job_id_t next);

/* ---- requested from state-flags (same declaration as io-pinball's) ----- */
agc_status_t agc_erase_write_ecadr(uint16_t ecadr, uint16_t word);  /* V71/V72 store */
/* ---- requested from prog-rendz (LANDMARK owner): V52 sets the offset --- */
void         agc_erase_set_landmark(agc_sp_t v);

agc_status_t agc_extverb_state_vector_copy(bool from_other); /* V66 / CSM V47 */
agc_status_t agc_extverb_lm_attck2(void);      /* LM V43 ATTCK2 (task body)  */

/* vehicle halves of the jobs whose AGC code differs (wrappers in
 * extended_verbs.c pick one by vehicle)                                   */
agc_status_t agc_job_attitude_error_load_csm(agc_job_ctx_t *ctx);
agc_status_t agc_job_attitude_error_load_lm(agc_job_ctx_t *ctx);
agc_status_t agc_job_tracking_terminate_csm(agc_job_ctx_t *ctx);
agc_status_t agc_job_tracking_terminate_lm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_noun46_csm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_noun46_lm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_config_csm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_config_lm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_noun47_csm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_noun47_lm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_mass_csm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_mass_lm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_noun48_csm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_noun48_lm(agc_job_ctx_t *ctx);

/* vehicle verb fans (extended_verbs_csm.c / extended_verbs_lm.c)          */
agc_status_t agc_extverb_csm(agc_job_ctx_t *ctx, int verb);
agc_status_t agc_extverb_lm(agc_job_ctx_t *ctx, int verb);

#endif /* AGC_IO_VERBS_PENDING_H */
