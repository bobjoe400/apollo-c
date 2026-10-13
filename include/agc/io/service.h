/**********************************************************************
 * FILE NAME: service.h
 *
 * PURPOSE:
 *   Public interface of the sys-misc lane: RTB op codes, T4RUPT (both
 *   vehicles), the downlink lists and the down-telemetry program, the
 *   extended verbs and the P27 update program, the service routines, the
 *   self-check / show-banksum and the LM P07 IMU performance tests.
 *   Called by the executive (dispatch, restarts, interrupts), pinball and
 *   the programs that use the service routines.
 *
 * AGC SOURCE:
 *   RTB_OP_CODES.agc, T4RUPT_PROGRAM.agc, DOWNLINK_LISTS.agc,
 *   DOWN-TELEMETRY_PROGRAM.agc, EXTENDED_VERBS.agc, UPDATE_PROGRAM.agc,
 *   SERVICE_ROUTINES.agc, AGC_BLOCK_TWO_SELF-CHECK.agc, INTER-BANK_
 *   COMMUNICATION.agc (both ropes); LM SYSTEM_TEST_STANDARD_LEAD_INS,
 *   IMU_PERFORMANCE_TEST_2, IMU_PERFORMANCE_TESTS_4.  Pages per section.
 *
 * REQUIRES:
 *   none (self-contained; includes what it uses)
 *
 * GLOBAL VARIABLES:
 *   none (the lane's erasables are reached through accessors; struct
 *   members below carry their scale in a short comment)
 **********************************************************************/
#ifndef AGC_SYSMISC_H
#define AGC_SYSMISC_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"

/* ======================================================================
 * RTB op codes (rtb_ops.c; RTB_OP_CODES.agc LM pp.1397-1402, RT8_OP_CODES.agc CM)
 * ==================================================================== */

/* Word-level image of MPAC, MPAC+1, MPAC+2: three signed SP words (sign +
 * 14 bits each).  Used only where sign disagreement must be expressible.  */
typedef struct agc_mpac_tp
{
    agc_sp_t w[3]; /* MPAC, MPAC +1, MPAC +2: 2^0, 2^-14, 2^-28 of the scale */
} agc_mpac_tp_t;

agc_dp_t     agc_rtb_loadtime(void);                                                                      /* LOADTIME, cs 2^28 */
agc_dp_t     agc_rtb_cdulogic(uint16_t cdu_pattern);                                                      /* CDULOGIC, revs    */
int          agc_rtb_sgnagree(agc_mpac_tp_t *mpac);                                                       /* SGNAGREE/TPAGREE  */
agc_dp_t     agc_rtb_sgnagree_dp(agc_sp_t hi, agc_sp_t lo);
uint16_t     agc_rtb_1sto2s(agc_dp_t angle_rev);                                                          /* 1STO2S            */
void         agc_rtb_v1sto2s(agc_vec_t angles_rev, uint16_t out[3]);                                      /* V1STO2S  */
void         agc_rtb_2v1sto2s(agc_dp_t a_rev, agc_dp_t b_rev, uint16_t out[2]);
uint16_t     agc_rtb_cduinc(uint16_t angle_pattern, agc_sp_t inc);                                        /* CDUINC   */
void         agc_rtb_incrcdus(uint16_t thetad[3], agc_vec_t incr);                                        /* INCRCDUS */
agc_vec_t    agc_rtb_readpips(void);                                                                      /* READPIPS */
agc_status_t agc_rtb_pulseimu(agc_vec_t gyro_cmds);                                                       /* PULSEIMU */
void         agc_rtb_transp(agc_mat_t *m);                                                                /* TRANSP1/TRANSP2  */
agc_dp_t     agc_rtb_signmpac(agc_dp_t mpac);                                                             /* SIGNMPAC         */
agc_vec_t    agc_rtb_normunit(agc_interp_t *ip, agc_vec_t v, int *shift, agc_dp_t *abval, agc_dp_t *vsq); /* NORMUNIT/NORMUNX1 */
agc_vec_t    agc_rtb_vecsgnag(agc_vec_t v);                                                               /* VECSGNAG         */
agc_vec_t    agc_rtb_quality2(agc_interp_t *ip, agc_vec_t urpv, agc_sp_t e3j22r2m);
agc_status_t agc_rtb_quality1(agc_interp_t *ip, agc_vec_t urpv, bool moonflag, agc_sp_t e3j22r2m, agc_vec_t *out);

/* ======================================================================
 * Downlink lists (src/io/downlink_lists_csm.c, src/io/downlink_lists_lm.c)
 * ==================================================================== */

/* Downlinked erasable items.  An entry names an item plus a word offset;
 * telemetry fetches two consecutive SP words per DP downlink word.        */
typedef enum agc_dn_item
{
    AGC_DN_SPARE = 0, /* SPARE / UNKNOWN = ERASZERO (register 7, zero) */
    AGC_DN_DNTMBUFF,  /* the snapshot buffer itself                    */
    AGC_DN_TIME2,     /* TIME2,TIME1                                   */
    AGC_DN_STATE,     /* FLAGWRD0..FLAGWRD11                           */
    AGC_DN_DSPTAB,    /* DSPTAB +0..+11D                               */
    AGC_DN_RN,
    AGC_DN_VN,
    AGC_DN_PIPTIME,
    AGC_DN_R_OTHER,
    AGC_DN_V_OTHER,
    AGC_DN_T_OTHER,
    AGC_DN_TIG,
    AGC_DN_REFSMMAT,
    AGC_DN_CDUX,      /* CDUX,CDUY,CDUZ,CDUT (LM order)                */
    AGC_DN_CDUZ,      /* CDUZ,CDUT (CM order)                          */
    AGC_DN_CDUS,      /* CDUS,PIPAX,PIPAY,PIPAZ                        */
    AGC_DN_IMODES30,  /* IMODES30,IMODES33                             */
    AGC_DN_OPTMODES,  /* OPTMODES,HOLDFLAG (CM)                        */
    AGC_DN_RADMODES,  /* RADMODES,DAPBOOLS (LM)                        */
    AGC_DN_COMPNUMB,  /* COMPNUMB,UPOLDMOD,UPVERB,UPCOUNT,UPBUFF...    */
    AGC_DN_UPBUFF,    /* UPBUFF +0..+19D                               */
    AGC_DN_CADRFLSH,  /* CADRFLSH,+1,+2,FAILREG,+1,+2                  */
    AGC_DN_REDOCTR,
    AGC_DN_RSBBQ,
    AGC_DN_LEMMASS,
    AGC_DN_DAPDATR1,
    AGC_DN_ADOT,
    AGC_DN_AK,
    AGC_DN_THETADX,
    AGC_DN_ERRORX,
    AGC_DN_WBODY,
    AGC_DN_DELLT4,
    AGC_DN_RTARG,
    AGC_DN_TGO,
    AGC_DN_PIPTIME1,
    AGC_DN_DELV,
    AGC_DN_PACTOFF,
    AGC_DN_PCMD,
    AGC_DN_CSTEER,
    AGC_DN_DELVEET1,
    AGC_DN_DELVEET2,
    AGC_DN_DELVEET3,
    AGC_DN_ELEV,
    AGC_DN_CENTANG,
    AGC_DN_DELTAR,
    AGC_DN_TEVENT,
    AGC_DN_VGTIG,
    AGC_DN_BESTI,
    AGC_DN_MARKDOWN,
    AGC_DN_MARK2DWN,
    AGC_DN_HAPOX,
    AGC_DN_OGC,
    AGC_DN_LAUNCHAZ,
    AGC_DN_VHFTIME,
    AGC_DN_VHFCNT,
    AGC_DN_TTPI,
    AGC_DN_ECSTEER,
    AGC_DN_DELVTPF,
    AGC_DN_TCDH,
    AGC_DN_TPASS4,
    AGC_DN_DELVSLV,
    AGC_DN_RANGE,
    AGC_DN_DIFFALT,
    AGC_DN_RTHETA,
    AGC_DN_LAT_SPL,
    AGC_DN_VPRED,
    AGC_DN_CMDAPMOD,
    AGC_DN_L_D1,
    AGC_DN_PAXERR1,
    AGC_DN_LATANG,
    AGC_DN_ALFA_180,
    AGC_DN_ROLLTM,
    AGC_DN_GAMMAEI,
    AGC_DN_TTE,
    AGC_DN_VIO,
    AGC_DN_SVMRKDAT,
    AGC_DN_LANDMARK,
    AGC_DN_LONG,
    AGC_DN_ALT,
    AGC_DN_LAT,
    AGC_DN_8NN,
    AGC_DN_RLS,
    /* Luminary-only items */
    AGC_DN_TCSI,
    AGC_DN_DNLRVELZ,
    AGC_DN_DNLRVELX,
    AGC_DN_OMEGAP,
    AGC_DN_CDUXD,
    AGC_DN_OMEGAPD,
    AGC_DN_POSTORKU,
    AGC_DN_DNRRANGE,
    AGC_DN_ALPHAQ,
    AGC_DN_POSTORKP,
    AGC_DN_AGSK,
    AGC_DN_TALIGN,
    AGC_DN_AOTCODE,
    AGC_DN_X789,
    AGC_DN_LASTYCMD,
    AGC_DN_STARSAV1,
    AGC_DN_STARSAV2,
    AGC_DN_AIG,
    AGC_DN_AOG,
    AGC_DN_TANGNB,
    AGC_DN_MKTIME,
    AGC_DN_RANGRDOT,
    AGC_DN_NN,
    AGC_DN_LRZCDUDL,
    AGC_DN_VSELECT,
    AGC_DN_LRVTIMDL,
    AGC_DN_VMEAS,
    AGC_DN_HMEAS,
    AGC_DN_RM,
    AGC_DN_LRXCDUDL,
    AGC_DN_UNFC_2,
    AGC_DN_VGVECT,
    AGC_DN_TTF_8,
    AGC_DN_DELTAH,
    AGC_DN_RGU,
    AGC_DN_VGU,
    AGC_DN_LAND,
    AGC_DN_AT,
    AGC_DN_TLAND,
    AGC_DN_FC,
    AGC_DN_PSEUDO55,
    AGC_DN_TTOGO,
    AGC_DN_YNBSAV,
    AGC_DN_GSAV,
    AGC_DN_AGSBUFF,
    AGC_DN_ITEM_COUNT
} agc_dn_item_t;

/* Entry op codes: 1..6 = 1DNADR..6DNADR (n DP words), DNCHAN, DNPTR.      */
#define AGC_DN_OP_CHAN 7 /* DNCHAN: channels n, n+1          */
#define AGC_DN_OP_PTR  8 /* DNPTR: a sublist                   */

typedef struct agc_dn_entry
{
    uint8_t  op;     /* 1..6 = nDNADR, AGC_DN_OP_CHAN, AGC_DN_OP_PTR       */
    bool     neg;    /* written negated in the AGC list (-xDNADR etc.)    */
    uint16_t ref;    /* agc_dn_item_t, channel number, or sublist id       */
    uint8_t  offset; /* SP-word offset from the item (e.g. RN +2 -> 2)     */
} agc_dn_entry_t;

typedef struct agc_dn_list
{
    const agc_dn_entry_t *entry;    /* the list words in AGC order     */
    uint8_t               count;    /* number of entries               */
    bool                  snapshot; /* sublist starting with -1DNADR      */
} agc_dn_list_t;

#define AGC_DNLIST_CSM_COUNT \
    5                            /* DNTABLE: COAST, ENTRY/UPDATE, REND, POWERED, P22 */
#define AGC_DNLIST_LM_COUNT \
    6                            /* DNTABLE: COAST, AGS, REND, ORBMAN, DESC/ASC, SURF */
#define AGC_DNLIST_WORDS 100     /* DP words per downlist (2 s at 50 words/s) */
#define AGC_DN_LOWIDCOD  077340u /* LOWIDCOD, the sync word             */

void         agc_dnlist_csm_control(int code, agc_dn_list_t *out);
agc_status_t agc_dnlist_csm_sublist(int sub, agc_dn_list_t *out);
agc_status_t agc_dnlist_lm_control(int code, agc_dn_list_t *out);
agc_status_t agc_dnlist_lm_sublist(int sub, agc_dn_list_t *out);

/* Number of DP downlink words a control list produces, ID word included.  */
agc_status_t agc_dnlist_word_count(int code, int *words);

/* ======================================================================
 * Down-telemetry program (telemetry.c; DOWN_TELEMETRY_PROGRAM.agc pp.988-997)
 * ==================================================================== */
void         agc_telem_fresh_start(void);         /* DOFSTART: DNLSTCOD = 0  */
void         agc_telem_slap1(void);               /* LM SLAP1: DUMPCNT       */
void         agc_telem_restart(void);             /* STARTSUB: restart list  */
void         agc_telem_select_list(int dnlstcod); /* TS DNLSTCOD             */
agc_status_t agc_telem_downrupt(void);            /* DODOWNTM, every 20 ms   */
void         agc_telem_start_dump(void);          /* V74: DNTMGOTO = DNDUMPI */
uint16_t     agc_erase_agsword(void);             /* AGSWORD (LM): saved list */
void         agc_erase_set_agsword(uint16_t v);

/* ======================================================================
 * Service routines (service.c; SERVICE_ROUTINES.agc C pp.1485-1492)
 * ==================================================================== */
uint16_t     agc_svc_upflag(agc_flag_id_t f);                            /* UPFLAG: returns flagword */
uint16_t     agc_svc_downflag(agc_flag_id_t f);                          /* DOWNFLAG                 */
void         agc_svc_upent2(uint16_t arg);                               /* UPENT2 (FLAGWRD4 on LM)  */
void         agc_svc_downent2(uint16_t arg);                             /* DOWNENT2                 */
agc_status_t agc_svc_delayjob(uint16_t delay_cs, agc_job_id_t wake_as);  /* DELAYJOB */
agc_status_t agc_svc_delay_wake(int slot);                               /* WAKER task body (-1)     */
void         agc_svc_startsub_reset(void);                               /* DELAYLOC = 0             */
void         agc_svc_gentran(const uint16_t *from, uint16_t *to, int n); /* GENTRAN */
void         agc_svc_b5off(void);                                        /* B5OFF                    */
void         agc_svc_trfailof(void);                                     /* TRFAILOF (CM)            */
void         agc_svc_trfailon(void);                                     /* TRFAILON (CM)            */

/* ======================================================================
 * T4RUPT (t4rupt.c + src/io/service/t4rupt_csm.c + src/io/service/t4rupt_lm.c)
 * ==================================================================== */
/* Run one T4RUPT; *next_cs receives the TIME4 reload (centiseconds until
 * the next T4RUPT: 2 for the 20 ms DSKY relay passes, 12 otherwise).     */
agc_status_t agc_t4rupt(uint16_t *next_cs);
agc_status_t agc_t4rupt_rupt(void);           /* rupt entry: + TIME4      */
void         agc_t4_startsub_reset(void);     /* STARTSUB: DSRUPTSW = +0  */
void         agc_t4_fresh_start_flush(void);  /* SLAP1: DSRUPTSW = -PRIO12 */
agc_status_t agc_t4_task(agc_task_id_t task); /* ENDTNON UNZ2 ISSUP ENDZOPT */
void         agc_t4_setissw(void);            /* SETISSW (called by imu)  */
void         agc_t4_cagesub(void);            /* CAGESUB                  */
void         agc_t4_cagesub2(void);           /* CAGESUB2                 */
bool         agc_t4_lamptest(void);           /* LAMPTEST: true if on     */
agc_status_t agc_t4_imumon(void);             /* IMUMON .. GLOCKMON       */
agc_status_t agc_t4_tnontest(void);           /* TNONTEST                 */
agc_status_t agc_t4_c33test(void);            /* C33TEST                  */
agc_status_t agc_t4_glockmon(void);           /* GLOCKMON                 */

/* CSM optics (src/io/service/t4rupt_csm.c). OPTIND -0 is AGC_OPTIND_NEG0.      */
#define AGC_OPTIND_NEG0 ((agc_sp_t) -2)              /* OPTIND -0: optics reserved (TVC) */
agc_status_t agc_t4_optics_monitor(void);            /* OPTMON                   */
void         agc_t4_optics_drive(void);              /* OPTDRIVE                 */
agc_status_t agc_t4_optics_task(agc_task_id_t task); /* ENDZOPT              */
void         agc_t4_optics_reset(bool fresh);        /* OPTIND/OPTMODES init     */
uint16_t     agc_erase_optmodes(void);               /* OPTMODES (CM)            */
void         agc_erase_set_optmodes(uint16_t v);
agc_sp_t     agc_erase_optind(void);                 /* OPTIND (CM)              */
void         agc_erase_set_optind(agc_sp_t v);
agc_sp_t     agc_erase_swsample(void);               /* SWSAMPLE (CM) +1 CSC, 0 manual, -1 zero optics */
uint16_t     agc_erase_desoptt(void);                /* DESOPTT (CM): OPTDRIVE trunnion, CDU word */
void         agc_erase_set_desoptt(uint16_t v);
uint16_t     agc_erase_desopts(void);                /* DESOPTS (CM): OPTDRIVE shaft, CDU word */
void         agc_erase_set_desopts(uint16_t v);

/* LM radar monitor and DAP matrix (src/io/service/t4rupt_lm.c).                 */
agc_status_t agc_t4_rr_monitor(void);                 /* RRAUTCHK..RRGIMON+GPMATRIX */
void         agc_t4_gimbal_matrix(void);              /* GPMATRIX / DAPT4S        */
uint16_t     agc_erase_radmodes(void);                /* RADMODES (LM)            */
void         agc_erase_set_radmodes(uint16_t v);
void         agc_t4_radmodes_init(void);              /* STARTSUB: RADMODES = RMODINIT */
bool         agc_rr_magsub(int a, int k);             /* MAGSUB: |a| + k <= 0 (L+3)     */
bool         agc_rr_limchk(uint16_t t, uint16_t s);   /* RRLIMCHK: in the mode's limits */
void         agc_rr_settrkf(void);                    /* SETTRKF: tracker fail lamp     */
void         agc_rr_lamp_put(uint16_t lamp, bool on); /* TRKFLON +1: DSPTAB +11D lamp  */
void         agc_rendz_flip(uint16_t lamp_bit);       /* FLIP: invert a DSPTAB +11D lamp */
void         agc_rendz_clradmod(void);                /* CLRADMOD: designate bits, RR error counters off */
agc_status_t agc_rr_zero(void);                       /* RRZERO: zero the RR CDUs (RADSTALL next) */
agc_status_t agc_radar_lrpos2(void);                  /* LRPOS2: landing radar to position 2      */
agc_sp_t     agc_erase_m11(void);                     /* SIN(MG), scaled at 1     */
agc_sp_t     agc_erase_m21(void);                     /* COS(OG)COS(MG)           */
agc_sp_t     agc_erase_m22(void);                     /* SIN(OG)                  */
agc_sp_t     agc_erase_m31(void);                     /* -SIN(OG)COS(MG)          */
agc_sp_t     agc_erase_m32(void);                     /* COS(OG)                  */
agc_sp_t     agc_erase_cosmg(void);                   /* COS(MG)                  */

/* ======================================================================
 * Extended verbs (src/io/verbs/extended_verbs{,_csm,_lm}.c) and the update
 * program P27 (src/io/verbs/uplink_update.c)
 * ==================================================================== */
agc_status_t agc_extverb(int verb, int noun);           /* GOEXTVB (pinball) */
agc_status_t agc_extverb_goto_tracking_terminate(void); /* GOTOV56: V56 TRACKTRM */

/* ---- shared parts of the extended verbs (extended_verbs.c) -------------- */
/* The verb and noun GOEXTVB was entered with (VERBREG, NOUNREG).          */
int          agc_extverb_verb(void);
int          agc_extverb_noun(void);
agc_status_t agc_extverb_testxact(bool *ok);        /* TESTXACT: false = ALM/END done */
agc_status_t agc_extverb_alm_end(void);             /* ALM/END: FALTON, PINBRNCH     */
agc_status_t agc_extverb_gopin(void);               /* GOPIN: PINBRNCH               */
agc_status_t agc_extverb_chkpooh(bool *p00);        /* CHKPOOH: P00 (ALM/END if not) */
agc_status_t agc_extverb_ckmodcad(bool *free);      /* CKMODCAD: no IMU mode user     */
agc_status_t agc_extverb_goloadlv(void);            /* GOLOADLV: please-perform done */
agc_status_t agc_extverb_upflag(agc_flag_id_t f);   /* TC UPFLAG, TC GOPIN       */
agc_status_t agc_extverb_downflag(agc_flag_id_t f); /* TC DOWNFLAG, TC GOPIN     */
agc_status_t agc_extverb_schedule(agc_prio_t prio, agc_job_id_t job, bool vac);

/* Request a flashing display for the running verb (GOXDSPF = GOMARKF).    */
agc_status_t agc_extverb_flash(const agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun, agc_job_id_t on_proceed, agc_job_id_t on_enter);

/* EXDSPRET: show our verb again and continue at next.                     */
agc_status_t agc_extverb_redisplay(const agc_job_ctx_t *ctx, uint8_t verb, agc_job_id_t next);

/* LM radar verb entries (extended_verbs_lm_radar.c)                       */
agc_status_t agc_extverb_rr_zero(void);                        /* V40 N72 */
agc_status_t agc_extverb_rr_designate(agc_job_ctx_t *ctx);     /* V41 N72 */
agc_status_t agc_extverb_rr_designate_end(void);               /* V44     */
agc_status_t agc_extverb_lr_position_two(void);                /* V60     */
agc_status_t agc_extverb_radar_test(agc_job_ctx_t *ctx);       /* V63 R04 */
agc_status_t agc_extverb_lr_spurious_test(agc_job_ctx_t *ctx); /* V78 R77 */
agc_status_t agc_extverb_lr_spurious_test_end(void);           /* V79    */
agc_status_t agc_extverb_rr_los_display(agc_job_ctx_t *ctx);   /* V85     */

agc_status_t agc_extverb_state_vector_copy(bool from_other); /* V66 / CSM V47 */
agc_status_t agc_extverb_lm_attck2(void);                    /* LM V43 ATTCK2 (task body)  */

/* vehicle halves of the jobs whose AGC code differs (wrappers in
 * extended_verbs.c pick one by vehicle)                                   */
agc_status_t agc_job_attitude_error_load_csm(void);
agc_status_t agc_job_attitude_error_load_lm(agc_job_ctx_t *ctx);
agc_status_t agc_job_tracking_terminate_csm(void);
agc_status_t agc_job_tracking_terminate_lm(void);
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

bool agc_extverb_busy(void); /* EXTVBACT != 0   */

/* EXTVBACT: extended-verb / marking-system activity word (bit 2 marking
 * system in use, bit 5 an extended verb is active, bit 14 P61 lock-out). */
uint16_t agc_erase_extvbact(void);
void     agc_erase_set_extvbact(uint16_t v);

/* jobs and tasks (ids in src/io/verbs/pending.h until integration)       */
agc_status_t agc_job_extended_verb(agc_job_ctx_t *ctx);         /* LST2FAN   */
agc_status_t agc_job_extended_verb_end(void);                   /* ENDEXT    */
agc_status_t agc_job_extended_verb_gopin(void);                 /* GOPIN     */
agc_status_t agc_job_keyboard_coarse_align(agc_job_ctx_t *ctx); /* V41 ICORK2 */
agc_status_t agc_job_keyboard_coarse_drive(void);               /* V41 IMUCOARS */
agc_status_t agc_job_keyboard_fine_align(agc_job_ctx_t *ctx);   /* V42       */
agc_status_t agc_job_keyboard_fine_mode(void);                  /* V42 IMUFINE */
agc_status_t agc_job_keyboard_gyro_torque(void);                /* V42 FINEK2 */
agc_status_t agc_job_clock_align_update(void);                  /* V55 UPDATIME */
agc_status_t agc_job_show_banksum_display(agc_job_ctx_t *ctx);  /* V91 NOKILL */
agc_status_t agc_job_show_banksum_next(agc_job_ctx_t *ctx);     /* V91 NXTBNK */
agc_status_t agc_job_show_banksum_end(void);                    /* V91 V34   */
agc_status_t agc_job_attitude_error_load(agc_job_ctx_t *ctx);   /* V43       */
agc_status_t agc_job_attitude_error_terminate(void);            /* V43 CSM TRMATTCK */
agc_status_t agc_task_attitude_error_output(void);              /* V43 ATTCK1/2 */
agc_status_t agc_job_dap_data_load_noun46(agc_job_ctx_t *ctx);  /* R03 N46   */
agc_status_t agc_job_dap_data_load_config(agc_job_ctx_t *ctx);  /* R03 N46 done */
agc_status_t agc_job_dap_data_load_noun47(agc_job_ctx_t *ctx);  /* R03 N47   */
agc_status_t agc_job_dap_data_load_mass(agc_job_ctx_t *ctx);    /* R03 N47 done */
agc_status_t agc_job_dap_data_load_noun48(agc_job_ctx_t *ctx);  /* R03 N48   */
agc_status_t agc_job_dap_data_load_trim(void);                  /* LM DPDAT3 */
agc_status_t agc_job_dap_data_load_end(void);                   /* LM ENDR03 */
agc_status_t agc_job_trimdone(agc_job_ctx_t *ctx);              /* LM TRIMDONE */
agc_status_t agc_job_state_vector_copy(void);                   /* V66 / CSM V47 */
agc_status_t agc_job_tracking_terminate(void);                  /* CSM V56 */
agc_status_t agc_job_attitude_error_enable(void);               /* LM V43 */
agc_status_t agc_job_optics_coarse_start(void);                 /* CSM V41 N91 OPTIND */

/* LM radar verbs (src/io/verbs/extended_verbs_lm_radar.c; their RRDESK2 and
 * RRLOSDSP jobs are in src/lm/radar and src/lm/optics, rendezvous.h and
 * alignment.h).                                                          */
agc_status_t agc_job_lr_position_two_check(void);                   /* V60 LRP2ALM */
agc_status_t agc_job_rr_designate_load(agc_job_ctx_t *ctx);         /* V41N72 V24N73 */
agc_status_t agc_job_rr_lockon_query(agc_job_ctx_t *ctx);           /* AURLOKON  */
agc_status_t agc_job_rr_lockon_display(agc_job_ctx_t *ctx);         /* AURLOKON V04N12 */
agc_status_t agc_job_rr_lockon_answer(agc_job_ctx_t *ctx);          /* AURLOKON +5 */
agc_status_t agc_job_rr_designate_schedule(void);                   /* RRDESNBK end */
void         agc_job_rr_designate_result(void);                     /* RRDESK2 alarm 503 */
agc_status_t agc_job_rr_designate_end_monitor(void);                /* V44 NORRMON */
void         agc_job_rr_los_display_stop(void);                     /* V85 B5OFF */
agc_status_t agc_job_radar_test_option_display(agc_job_ctx_t *ctx); /* R04A V04N12 */
agc_status_t agc_job_radar_test_select(agc_job_ctx_t *ctx);         /* R04 RTSTDEX */
agc_status_t agc_job_radar_test_auto_check(agc_job_ctx_t *ctx);     /* R04B      */
agc_status_t agc_job_radar_test_auto_request(agc_job_ctx_t *ctx);   /* R04B V50N25 */
agc_status_t agc_job_radar_test_first_display(agc_job_ctx_t *ctx);  /* R04RR/R04LR */
agc_status_t agc_job_radar_test_second_display(agc_job_ctx_t *ctx); /* N78/N67 */
agc_status_t agc_job_radar_test_switch(void);                       /* R04Y      */
agc_status_t agc_job_radar_test_switch_resume(agc_job_ctx_t *ctx);  /* R04Y +2 s */
agc_status_t agc_job_radar_test_end(void);                          /* R04END    */
agc_status_t agc_job_radar_test_end_resume(void);                   /* R04END +1.28 s */
agc_status_t agc_job_lr_spurious_test_end_resume(void);             /* R77END */

/* R04/R77 radar test storage (owned here; RADSAMP/DORSAMP read and write
 * them).  SP integers.                                                    */
agc_sp_t agc_erase_rsampdt(void);  /* RSAMPDT: interval + 1, cs; 0 stops */
void     agc_erase_set_rsampdt(agc_sp_t v);
agc_sp_t agc_erase_rtstdex(void);  /* RTSTDEX: sample index           */
void     agc_erase_set_rtstdex(agc_sp_t v);
agc_sp_t agc_erase_rtstbase(void); /* RTSTBASE: 0 RR, 2 LR            */
agc_sp_t agc_erase_rtstmax(void);  /* RTSTMAX: 2 RR, 6 LR             */
agc_sp_t agc_erase_rtstloc(void);  /* RTSTLOC: storage index          */
void     agc_erase_set_rtstloc(agc_sp_t v);
agc_sp_t agc_erase_rfailcnt(void); /* RFAILCNT: bad samples           */
void     agc_erase_set_rfailcnt(agc_sp_t v);

/* P27 (V70-V73).  The uplinked words are UPBUFF; COMPNUMB, UPOLDMOD,
 * UPVERB, UPCOUNT, UPBUFF +0..+19D are downlinked (index 0..23).          */
agc_status_t agc_uplink_update_start(agc_job_ctx_t *ctx, int verb); /* V70..V73 */
uint16_t     agc_uplink_update_dnword(int idx);
void         agc_uplink_update_fresh_start(void);                   /* UPSVFLAG etc.   */

/* The word an ECADR load/write puts into COMPNUMB..UPBUFF +19D (0..23) or
 * UPTEMP (24); agc_uplink_update_target() = the index the V21N01/N02 on
 * the DSKY loads (the AGC's MPAC+2).                                      */
agc_status_t agc_uplink_update_write(int idx, uint16_t word);
int          agc_uplink_update_target(void);
agc_sp_t     agc_erase_upsvflag(void);                          /* UPSVFLAG: 0, +-1 earth, +-2 moon (+CSM -LM) */
agc_status_t agc_erase_set_upsvflag(agc_sp_t v);                /* BAD_ARG outside -2..2 */
agc_status_t agc_job_uplink_update_load(agc_job_ctx_t *ctx);    /* after V21N01 */
agc_status_t agc_job_uplink_update_execute(void);               /* UPJOB  */
agc_status_t agc_job_uplink_update_terminate(void);             /* UPOUT +1 */
agc_status_t agc_job_uplink_update_proceed(agc_job_ctx_t *ctx); /* V33 on a load */
agc_status_t agc_job_timedidr(void);                            /* restart 6.5 */

/* ======================================================================
 * Self-check and show-banksum (src/io/service/self_check.c;
 * AGC_BLOCK_TWO_SELF_CHECK.agc)
 * ==================================================================== */
#define AGC_ROPE_BANK_WORDS 02000 /* 1024 words per fixed bank          */
#define AGC_ROPE_BANKS      044   /* banks 0..43 octal                  */

/* The sim's fixed-memory image: nbanks banks of 1024 words, bank b at
 * rope[b * 1024], 15-bit words.  NULL (nbanks 0) = no rope: every bank
 * passes ROPECHK.                                                         */
void agc_selfcheck_set_rope(const uint16_t *rope, int nbanks);

/* Sum one bank as ROPECHK / SHOWSUM do: sum = the ones'-complement sum
 * with end-around carry up to the two TC SELF words and the bugger word. */
agc_status_t agc_selfcheck_banksum(int bank, uint16_t *sum, uint16_t *bugger);
void         agc_selfcheck_set_smode(uint16_t word);     /* N27 load (SMODE, raw; 077777 = -0) */
uint16_t     agc_selfcheck_smode(void);                  /* SMODE                              */
void         agc_selfcheck_idle_pass(void);              /* ADVAN -> SELFRET: one idle pass    */
void         agc_selfcheck_fresh_start(bool slap1);      /* SMODE = 0; SLAP1: ERCOUNT = 0      */
void         agc_selfcheck_restart(void);                /* STARTSB2: SELFRET = SELFCHK        */
void         agc_selfcheck_showsum_start(void);          /* V91 STSHOSUM                       */
void         agc_selfcheck_showsum_next(void);           /* V33 NXTBNK                         */
void         agc_selfcheck_showsum_words(uint16_t w[3]); /* SKEEP1..3 (V05 N01)               */
uint16_t     agc_selfcheck_error_count(void);            /* ERCOUNT (ALMCADR +2)               */

/* ======================================================================
 * P07 IMU performance tests (src/lm/systems_test.c; Luminary099
 * SYSTEM_TEST_STANDARD_LEAD_INS, IMU_PERFORMANCE_TEST_2, _TESTS_4)
 * ==================================================================== */
agc_status_t agc_systems_test_zeroing(uint16_t *block, int n);          /* ZEROING  */
agc_dp_t     agc_systems_test_overffix(agc_interp_t *ip, agc_dp_t a);   /* OVERFFIX */
agc_status_t agc_job_systems_test(agc_job_ctx_t *ctx);                  /* REDO (V92) */
agc_status_t agc_job_systems_test_imu_zeroed(agc_job_ctx_t *ctx);       /* IMUBACK          */
agc_status_t agc_job_systems_test_site_display(agc_job_ctx_t *ctx);     /* LATAZCHK V06N41  */
agc_status_t agc_job_systems_test_site_loaded(agc_job_ctx_t *ctx);      /* POSGMBL          */
agc_status_t agc_job_systems_test_end(agc_job_ctx_t *ctx);              /* ENDTEST1         */
agc_status_t agc_job_systems_test_coarse_aligned(agc_job_ctx_t *ctx);   /* POSGMBL stall    */
agc_status_t agc_job_systems_test_fine_aligned(agc_job_ctx_t *ctx);     /* FINIMUDD stall   */
agc_status_t agc_job_systems_test_drift_start(agc_job_ctx_t *ctx);      /* ESTIMS           */
agc_status_t agc_job_systems_test_resume(agc_job_ctx_t *ctx);           /* TC QPLACE        */
agc_status_t agc_job_systems_test_earth_rate_retry(agc_job_ctx_t *ctx); /* PULSEIMU again   */
agc_status_t agc_job_systems_test_earth_rate_done(agc_job_ctx_t *ctx);  /* TC QPLACES       */
agc_status_t agc_job_systems_test_pip_catch(agc_job_ctx_t *ctx);        /* CHECKG1          */
agc_status_t agc_job_systems_test_pip_result(agc_job_ctx_t *ctx);       /* PIPJOBB          */
agc_status_t agc_job_systems_test_filter(agc_job_ctx_t *ctx);           /* ALFLT            */
agc_status_t agc_job_systems_test_torque_retry(agc_job_ctx_t *ctx);     /* IMUPULSE again   */
agc_status_t agc_job_systems_test_torqued(agc_job_ctx_t *ctx);          /* GEOSTRT4         */
agc_status_t agc_job_systems_test_gimbals_zeroed(agc_job_ctx_t *ctx);   /* COAALIGN stall   */
agc_status_t agc_job_systems_test_show(agc_job_ctx_t *ctx);             /* SHOW1 V06N98     */
agc_status_t agc_task_systems_test_drift_wake(void);                    /* GOESTIMS         */
agc_status_t agc_task_systems_test_pip_timer(void);                     /* PIPATASK         */
agc_status_t agc_task_systems_test_sample(void);                        /* ALLOOP           */

#endif /* AGC_SYSMISC_H */
