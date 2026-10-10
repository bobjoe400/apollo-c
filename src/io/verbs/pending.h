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

#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"

/* ---- jobs owned by sys-misc (0x0340..0x037F) --------------------------- */
#define AGC_JOB_EXTENDED_VERB \
    ((agc_job_id_t) 0x0340) /* GOEXTVB / LST2FAN       */
#define AGC_JOB_EXTENDED_VERB_END \
    ((agc_job_id_t) 0x0341) /* ENDEXT (TERMEXTV)       */
#define AGC_JOB_EXTENDED_VERB_GOPIN \
    ((agc_job_id_t) 0x0342) /* GOPIN (PINBRNCH)        */
#define AGC_JOB_KEYBOARD_COARSE_ALIGN \
    ((agc_job_id_t) 0x0343) /* V41 ICORK2              */
#define AGC_JOB_KEYBOARD_COARSE_DRIVE \
    ((agc_job_id_t) 0x0344) /* V41 IMUCOARS + IMUSTALL */
#define AGC_JOB_KEYBOARD_FINE_ALIGN \
    ((agc_job_id_t) 0x0345) /* V42 after V25N93        */
#define AGC_JOB_KEYBOARD_FINE_MODE \
    ((agc_job_id_t) 0x0346) /* V42 IMUFINE + IMUSTALL  */
#define AGC_JOB_KEYBOARD_GYRO_TORQUE \
    ((agc_job_id_t) 0x0347) /* V42 FINEK2 (IMUPULSE)   */
#define AGC_JOB_CLOCK_ALIGN_UPDATE \
    ((agc_job_id_t) 0x0348) /* V55 UPDATIME            */
#define AGC_JOB_SHOW_BANKSUM_DISPLAY \
    ((agc_job_id_t) 0x0349) /* V91 NOKILL              */
#define AGC_JOB_SHOW_BANKSUM_NEXT \
    ((agc_job_id_t) 0x034A) /* V91 NXTBNK              */
#define AGC_JOB_SHOW_BANKSUM_END \
    ((agc_job_id_t) 0x034B) /* V91 terminate           */
#define AGC_JOB_ATTITUDE_ERROR_LOAD \
    ((agc_job_id_t) 0x034C) /* V43 after V25N22        */
#define AGC_JOB_ATTITUDE_ERROR_TERMINATE \
    ((agc_job_id_t) 0x034D) /* V43 CSM TRMATTCK        */
#define AGC_JOB_DAP_DATA_LOAD_NOUN46 \
    ((agc_job_id_t) 0x034E) /* R03 DONOUN46 / DAPDATA1 */
#define AGC_JOB_DAP_DATA_LOAD_CONFIG \
    ((agc_job_id_t) 0x034F) /* R03 after N46 (MAXOUT / DPDAT1) */
#define AGC_JOB_DAP_DATA_LOAD_NOUN47 \
    ((agc_job_id_t) 0x0350) /* R03 DONOUN47 / DAPDATA2 */
#define AGC_JOB_DAP_DATA_LOAD_MASS \
    ((agc_job_id_t) 0x0351) /* R03 after N47 (MASSPROP / DAPDAT2) */
#define AGC_JOB_DAP_DATA_LOAD_NOUN48 \
    ((agc_job_id_t) 0x0352) /* R03 DONOUN48 / DAPDATA3 */
#define AGC_JOB_DAP_DATA_LOAD_TRIM \
    ((agc_job_id_t) 0x0353) /* R03 LM DPDAT3           */
#define AGC_JOB_DAP_DATA_LOAD_END \
    ((agc_job_id_t) 0x0354) /* R03 LM ENDR03 (V34)     */
#define AGC_JOB_STATE_VECTOR_COPY \
    ((agc_job_id_t) 0x0355) /* V66 ATTACHIT / V47 LMTOCM */
#define AGC_JOB_TRACKING_TERMINATE \
    ((agc_job_id_t) 0x0356) /* V56 CSM after INTSTALL  */
#define AGC_JOB_ATTITUDE_ERROR_ENABLE \
    ((agc_job_id_t) 0x0357) /* V43 LM after EXDSPRET   */
#define AGC_JOB_UPLINK_UPDATE \
    ((agc_job_id_t) 0x0360) /* V70-V73 entry           */
#define AGC_JOB_UPLINK_UPDATE_LOAD \
    ((agc_job_id_t) 0x0361) /* after a V21N01 load     */
#define AGC_JOB_UPLINK_UPDATE_TERMINATE \
    ((agc_job_id_t) 0x0362) /* V34: UPOUT +1           */
#define AGC_JOB_UPLINK_UPDATE_PROCEED \
    ((agc_job_id_t) 0x0363) /* V33 on a V21N01/N02     */
#define AGC_JOB_UPLINK_UPDATE_EXECUTE \
    ((agc_job_id_t) 0x0364) /* V33 after the V21N01 index */
/* reused names from other lanes' shims (same value) */
#define AGC_JOB_CREW_DEFINED_MANEUVER_DISPLAY \
    ((agc_job_id_t) 0x331)                       /* R62DISP (V49)  */
#define AGC_JOB_STRKTSTI ((agc_job_id_t) 0x0D21) /* V68 stroke test init  */
#define AGC_JOB_COMPVER  ((agc_job_id_t) 0x0E29) /* V65 GCOMPVER (4.15)   */
/* entries of other lanes that have no id yet (names from NAMING.md)       */
#define AGC_JOB_RENDEZVOUS_PARAMETER_DISPLAY \
    ((agc_job_id_t) 0x0368) /* R31CALL (CSM V83/V85)  */
#define AGC_JOB_OUT_OF_PLANE_DISPLAY \
    ((agc_job_id_t) 0x0369) /* R36 (V90)               */
#define AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE \
    ((agc_job_id_t) 0x036A) /* V89CALL (R63)           */
#define AGC_JOB_W_MATRIX_MONITOR \
    ((agc_job_id_t) 0x036B) /* V67CALL                 */
#define AGC_JOB_SIGHTING_MARK \
    ((agc_job_id_t) 0x036C) /* R21CSM (V57)            */
#define AGC_JOB_BACKUP_SIGHTING_MARK \
    ((agc_job_id_t) 0x036D) /* R23CSM (V54)            */
#define AGC_JOB_GYROCOMPASS_AZIMUTH_CHANGE \
    ((agc_job_id_t) 0x036E) /* AZMTHCG1 (V78)         */
#define AGC_JOB_SYSTEMS_TEST \
    ((agc_job_id_t) 0x036F) /* REDO (V92, P07)         */
#define AGC_JOB_AGS_INITIALIZATION \
    ((agc_job_id_t) 0x0370) /* AGSINIT (LM V47, R47)   */
#define AGC_JOB_RR_LOS_DISPLAY \
    ((agc_job_id_t) 0x0371) /* RRLOSDSP (LM V85)       */
#define AGC_JOB_SBANDANT_CM \
    ((agc_job_id_t) 0x0372) /* SBANDANT (CM V64), agc_job_sbandant_cm */
#define AGC_JOB_SBANDANT_LM \
    ((agc_job_id_t) 0x0373) /* SBANDANT (LM V64), agc_job_sbandant_lm */
#define AGC_TASK_RENDEZVOUS_PARAMETER_DISPLAY \
    ((agc_task_id_t) 0x0368) /* R31CALL (LM V83)   */
/* LM radar verbs (extended_verbs_lm_radar.c)                              */
#define AGC_JOB_LR_POSITION_TWO_CHECK \
    ((agc_job_id_t) 0x0380) /* V60 after RADSTALL      */
#define AGC_JOB_RR_DESIGNATE_LOAD \
    ((agc_job_id_t) 0x0381) /* V41N72 V24N73 (V33 again) */
#define AGC_JOB_RR_LOCKON_QUERY \
    ((agc_job_id_t) 0x0382) /* AURLOKON (ENTER on N73) */
#define AGC_JOB_RR_LOCKON_DISPLAY \
    ((agc_job_id_t) 0x0383) /* AURLOKON -5 V04N12      */
#define AGC_JOB_RR_LOCKON_ANSWER \
    ((agc_job_id_t) 0x0384) /* AURLOKON +5 (V33)       */
#define AGC_JOB_RR_DESIGNATE_SCHEDULE \
    ((agc_job_id_t) 0x0385) /* RRDESNBK after EXDSPRET */
#define AGC_JOB_RR_DESIGNATE_DRIVE \
    ((agc_job_id_t) 0x0386) /* RRDESK2                 */
#define AGC_JOB_RR_DESIGNATE_RESULT \
    ((agc_job_id_t) 0x0387) /* RRDESK2 after RADSTALL  */
#define AGC_JOB_RR_DESIGNATE_END_MONITOR \
    ((agc_job_id_t) 0x0388) /* V44 after DELAYJOB      */
#define AGC_JOB_RR_LOS_DISPLAY_NEXT \
    ((agc_job_id_t) 0x0389) /* V85 RRLOSDSP after 1 s  */
#define AGC_JOB_RR_LOS_DISPLAY_STOP \
    ((agc_job_id_t) 0x038A) /* V85 responses: B5OFF    */
#define AGC_JOB_RADAR_TEST_OPTION_DISPLAY \
    ((agc_job_id_t) 0x038B) /* R04A +2 V04N12          */
#define AGC_JOB_RADAR_TEST_SELECT \
    ((agc_job_id_t) 0x038C) /* R04 V33 on V04N12       */
#define AGC_JOB_RADAR_TEST_AUTO_CHECK \
    ((agc_job_id_t) 0x038D) /* R04B                    */
#define AGC_JOB_RADAR_TEST_AUTO_REQUEST \
    ((agc_job_id_t) 0x038E) /* R04B +5 V50N25          */
#define AGC_JOB_RADAR_TEST_FIRST_DISPLAY \
    ((agc_job_id_t) 0x038F) /* R04RR / R04LR           */
#define AGC_JOB_RADAR_TEST_SECOND_DISPLAY \
    ((agc_job_id_t) 0x0390) /* V16N78 / V16N67         */
#define AGC_JOB_RADAR_TEST_SWITCH \
    ((agc_job_id_t) 0x0391) /* R04Y                    */
#define AGC_JOB_RADAR_TEST_SWITCH_RESUME \
    ((agc_job_id_t) 0x0392) /* R04Y after 2 s          */
#define AGC_JOB_RADAR_TEST_END \
    ((agc_job_id_t) 0x0393) /* R04END                  */
#define AGC_JOB_RADAR_TEST_END_RESUME \
    ((agc_job_id_t) 0x0394) /* R04END after 1.28 s     */
#define AGC_JOB_LR_SPURIOUS_TEST_END_RESUME \
    ((agc_job_id_t) 0x0395) /* R77END after 320 ms   */
#define AGC_JOB_OPTICS_COARSE_DRIVE \
    ((agc_job_id_t) 0x0396) /* CSM V41N91 OPTC1        */
#define AGC_JOB_OPTICS_COARSE_START \
    ((agc_job_id_t) 0x0397) /* CSM V41N91 after EXDSPRET */
/* prog-rendz's RADAR_LEADIN_ROUTINES task (value as in programs/navigation/pending.h)          */
#define AGC_TASK_RADSAMP ((agc_task_id_t) 0x1103) /* LM radar sampling     */
/* tasks owned by sys-misc */
#define AGC_TASK_ATTITUDE_ERROR_OUTPUT \
    ((agc_task_id_t) 0x0307) /* V43 ATTCK1 / ATTCK2    */

/* ---- shared parts of the extended verbs (extended_verbs.c) -------------- */
/* The verb and noun GOEXTVB was entered with (VERBREG, NOUNREG).          */
int agc_extverb_verb(void);
int agc_extverb_noun(void);
agc_status_t
agc_extverb_testxact(bool *ok);          /* TESTXACT: false = ALM/END done */
agc_status_t agc_extverb_alm_end(void);  /* ALM/END: FALTON, PINBRNCH     */
agc_status_t agc_extverb_gopin(void);    /* GOPIN: PINBRNCH               */
bool         agc_extverb_chkpooh(void);  /* CHKPOOH: P00 (ALM/END if not) */
bool         agc_extverb_ckmodcad(void); /* CKMODCAD: no IMU mode user     */
agc_status_t agc_extverb_goloadlv(void); /* GOLOADLV: please-perform done */
agc_status_t
agc_extverb_upflag(agc_flag_id_t f); /* TC UPFLAG, TC GOPIN       */
agc_status_t
agc_extverb_downflag(agc_flag_id_t f); /* TC DOWNFLAG, TC GOPIN     */
agc_status_t agc_extverb_schedule(agc_prio_t prio, agc_job_id_t job, bool vac);
/* Request a flashing display for the running verb (GOXDSPF = GOMARKF).    */
agc_status_t agc_extverb_flash(const agc_job_ctx_t *ctx, uint8_t verb,
                               uint8_t noun, agc_job_id_t on_proceed,
                               agc_job_id_t on_enter);
/* EXDSPRET: show our verb again and continue at next.                     */
agc_status_t agc_extverb_redisplay(const agc_job_ctx_t *ctx, uint8_t verb,
                                   agc_job_id_t next);

/* ---- requested from state-flags (same declaration as io-pinball's) ----- */
agc_status_t agc_erase_write_ecadr(uint16_t ecadr,
                                   uint16_t word); /* V71/V72 store */
/* ---- requested from prog-rendz (LANDMARK owner): V52 sets the offset --- */
void agc_erase_set_landmark(agc_sp_t v);

/* ---- requested from prog-rendz (P20-P25.agc; same declarations as the
 * landing and powered-flight shims where those exist) ------------------- */
agc_status_t agc_rr_zero(void); /* RRZERO: zero the RR CDUs      */
/* RRDESNB: designate to TANG, TANG +1.  *started = false when RRDESNB
 * ended the job instead (angles rejected, alarm 502; or a job already
 * waits in RADSTALL); then the caller ends without RADSTALL.             */
agc_status_t agc_rr_designate_gimbal(bool *started);
agc_status_t agc_rendz_clradmod(void); /* CLRADMOD (RADMODES)           */
agc_status_t agc_radar_lrpos2(void);   /* LRPOS2: LR to position 2      */
/* ---- requested from prog-align / io-pinball / io-nouns (display buffers) */
agc_sp_t agc_erase_optionx(int index); /* OPTIONX +index (N12)          */
void     agc_erase_set_optionx(int index, agc_sp_t v);
void     agc_erase_set_dsptem1(int index, agc_sp_t v); /* DSPTEM1 +index   */
void     agc_erase_set_rr_az(agc_dp_t v);   /* RR-AZ (N56), revs           */
void     agc_erase_set_rr_elev(agc_dp_t v); /* RR-ELEV (N56), revs         */

/* LM radar verb entries (extended_verbs_lm_radar.c)                       */
agc_status_t agc_extverb_rr_zero(agc_job_ctx_t *ctx);              /* V40 N72 */
agc_status_t agc_extverb_rr_designate(agc_job_ctx_t *ctx);         /* V41 N72 */
agc_status_t agc_extverb_rr_designate_end(agc_job_ctx_t *ctx);     /* V44     */
agc_status_t agc_extverb_lr_position_two(agc_job_ctx_t *ctx);      /* V60     */
agc_status_t agc_extverb_radar_test(agc_job_ctx_t *ctx);           /* V63 R04 */
agc_status_t agc_extverb_lr_spurious_test(agc_job_ctx_t *ctx);     /* V78 R77 */
agc_status_t agc_extverb_lr_spurious_test_end(agc_job_ctx_t *ctx); /* V79    */
agc_status_t agc_extverb_rr_los_display(agc_job_ctx_t *ctx);       /* V85     */

agc_status_t agc_extverb_state_vector_copy(bool from_other); /* V66 / CSM V47 */
agc_status_t agc_extverb_lm_attck2(void); /* LM V43 ATTCK2 (task body)  */

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
