/*
 * sysmisc.h -- public interface of the sys-misc lane (T4RUPT, extended verbs,
 * downlink lists and down-telemetry, P27 update program, service routines,
 * self-check / show-banksum, RTB op codes, LM P07 IMU tests).
 *
 * Promoted from src/io/sysmisc.h by the restructure; owned by the sys-misc
 * lane (docs/modules/sys-misc.md).
 */
#ifndef AGC_SYSMISC_H
#define AGC_SYSMISC_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/core/interp.h"
#include "agc/exec/exec.h"
#include "agc/core/state.h"

/* ======================================================================
 * RTB op codes (rtb_ops.c; RTB_OP_CODES.agc LM pp.1397-1402, RT8_OP_CODES.agc CM)
 * ==================================================================== */

/* Word-level image of MPAC, MPAC+1, MPAC+2: three signed SP words (sign +
 * 14 bits each).  Used only where sign disagreement must be expressible.  */
typedef struct agc_mpac_tp {
    agc_sp_t w[3];
} agc_mpac_tp_t;

agc_dp_t     agc_rtb_loadtime(void);                       /* LOADTIME, cs 2^28 */
agc_dp_t     agc_rtb_cdulogic(uint16_t cdu_pattern);       /* CDULOGIC, revs    */
int          agc_rtb_sgnagree(agc_mpac_tp_t *mpac);        /* SGNAGREE/TPAGREE  */
agc_dp_t     agc_rtb_sgnagree_dp(agc_sp_t hi, agc_sp_t lo);
uint16_t     agc_rtb_1sto2s(agc_dp_t angle_rev);           /* 1STO2S            */
agc_status_t agc_rtb_v1sto2s(agc_vec_t angles_rev, uint16_t out[3]);   /* V1STO2S  */
agc_status_t agc_rtb_2v1sto2s(agc_dp_t a_rev, agc_dp_t b_rev, uint16_t out[2]);
uint16_t     agc_rtb_cduinc(uint16_t angle_pattern, agc_sp_t inc);     /* CDUINC   */
agc_status_t agc_rtb_incrcdus(uint16_t thetad[3], agc_vec_t incr);     /* INCRCDUS */
agc_vec_t    agc_rtb_readpips(void);                                   /* READPIPS */
agc_status_t agc_rtb_pulseimu(agc_vec_t gyro_cmds);                    /* PULSEIMU */
agc_status_t agc_rtb_transp(agc_mat_t *m);                     /* TRANSP1/TRANSP2  */
agc_dp_t     agc_rtb_signmpac(agc_dp_t mpac);                  /* SIGNMPAC         */
agc_vec_t    agc_rtb_normunit(agc_interp_t *ip, agc_vec_t v, int *shift,
                              agc_dp_t *abval, agc_dp_t *vsq); /* NORMUNIT/NORMUNX1 */
agc_vec_t    agc_rtb_vecsgnag(agc_vec_t v);                    /* VECSGNAG         */
agc_vec_t    agc_rtb_quality2(agc_interp_t *ip, agc_vec_t urpv, agc_sp_t e3j22r2m);
agc_status_t agc_rtb_quality1(agc_interp_t *ip, agc_vec_t urpv, bool moonflag,
                              agc_sp_t e3j22r2m, agc_vec_t *out);

/* ======================================================================
 * Downlink lists (src/io/downlink_lists_csm.c, src/io/downlink_lists_lm.c)
 * ==================================================================== */

/* Downlinked erasable items.  An entry names an item plus a word offset;
 * telemetry fetches two consecutive SP words per DP downlink word.        */
typedef enum agc_dn_item {
    AGC_DN_SPARE = 0,      /* SPARE / UNKNOWN = ERASZERO (register 7, zero) */
    AGC_DN_DNTMBUFF,       /* the snapshot buffer itself                    */
    AGC_DN_TIME2,          /* TIME2,TIME1                                   */
    AGC_DN_STATE,          /* FLAGWRD0..FLAGWRD11                           */
    AGC_DN_DSPTAB,         /* DSPTAB +0..+11D                               */
    AGC_DN_RN, AGC_DN_VN, AGC_DN_PIPTIME,
    AGC_DN_R_OTHER, AGC_DN_V_OTHER, AGC_DN_T_OTHER,
    AGC_DN_TIG, AGC_DN_REFSMMAT,
    AGC_DN_CDUX,           /* CDUX,CDUY,CDUZ,CDUT (LM order)                */
    AGC_DN_CDUZ,           /* CDUZ,CDUT (CM order)                          */
    AGC_DN_CDUS,           /* CDUS,PIPAX,PIPAY,PIPAZ                        */
    AGC_DN_IMODES30,       /* IMODES30,IMODES33                             */
    AGC_DN_OPTMODES,       /* OPTMODES,HOLDFLAG (CM)                        */
    AGC_DN_RADMODES,       /* RADMODES,DAPBOOLS (LM)                        */
    AGC_DN_COMPNUMB,       /* COMPNUMB,UPOLDMOD,UPVERB,UPCOUNT,UPBUFF...    */
    AGC_DN_UPBUFF,         /* UPBUFF +0..+19D                               */
    AGC_DN_CADRFLSH,       /* CADRFLSH,+1,+2,FAILREG,+1,+2                  */
    AGC_DN_REDOCTR, AGC_DN_RSBBQ, AGC_DN_LEMMASS, AGC_DN_DAPDATR1,
    AGC_DN_ADOT, AGC_DN_AK, AGC_DN_THETADX, AGC_DN_ERRORX, AGC_DN_WBODY,
    AGC_DN_DELLT4, AGC_DN_RTARG, AGC_DN_TGO, AGC_DN_PIPTIME1, AGC_DN_DELV,
    AGC_DN_PACTOFF, AGC_DN_PCMD, AGC_DN_CSTEER, AGC_DN_DELVEET1,
    AGC_DN_DELVEET2, AGC_DN_DELVEET3, AGC_DN_ELEV, AGC_DN_CENTANG,
    AGC_DN_DELTAR, AGC_DN_TEVENT, AGC_DN_VGTIG, AGC_DN_BESTI,
    AGC_DN_MARKDOWN, AGC_DN_MARK2DWN, AGC_DN_HAPOX, AGC_DN_OGC,
    AGC_DN_LAUNCHAZ, AGC_DN_VHFTIME, AGC_DN_VHFCNT, AGC_DN_TTPI,
    AGC_DN_ECSTEER, AGC_DN_DELVTPF, AGC_DN_TCDH, AGC_DN_TPASS4,
    AGC_DN_DELVSLV, AGC_DN_RANGE, AGC_DN_DIFFALT, AGC_DN_RTHETA,
    AGC_DN_LAT_SPL, AGC_DN_VPRED, AGC_DN_CMDAPMOD, AGC_DN_L_D1,
    AGC_DN_PAXERR1, AGC_DN_LATANG, AGC_DN_ALFA_180, AGC_DN_ROLLTM,
    AGC_DN_GAMMAEI, AGC_DN_TTE, AGC_DN_VIO, AGC_DN_SVMRKDAT,
    AGC_DN_LANDMARK, AGC_DN_LONG, AGC_DN_ALT, AGC_DN_LAT, AGC_DN_8NN,
    AGC_DN_RLS,
    /* Luminary-only items */
    AGC_DN_TCSI, AGC_DN_DNLRVELZ, AGC_DN_DNLRVELX, AGC_DN_OMEGAP,
    AGC_DN_CDUXD, AGC_DN_OMEGAPD, AGC_DN_POSTORKU, AGC_DN_DNRRANGE,
    AGC_DN_ALPHAQ, AGC_DN_POSTORKP, AGC_DN_AGSK, AGC_DN_TALIGN,
    AGC_DN_AOTCODE, AGC_DN_X789, AGC_DN_LASTYCMD, AGC_DN_STARSAV1,
    AGC_DN_STARSAV2, AGC_DN_AIG, AGC_DN_AOG, AGC_DN_TANGNB, AGC_DN_MKTIME,
    AGC_DN_RANGRDOT, AGC_DN_NN, AGC_DN_LRZCDUDL, AGC_DN_VSELECT,
    AGC_DN_LRVTIMDL, AGC_DN_VMEAS, AGC_DN_HMEAS, AGC_DN_RM,
    AGC_DN_LRXCDUDL, AGC_DN_UNFC_2, AGC_DN_VGVECT, AGC_DN_TTF_8,
    AGC_DN_DELTAH, AGC_DN_RGU, AGC_DN_VGU, AGC_DN_LAND, AGC_DN_AT,
    AGC_DN_TLAND, AGC_DN_FC, AGC_DN_PSEUDO55, AGC_DN_TTOGO, AGC_DN_YNBSAV,
    AGC_DN_GSAV, AGC_DN_AGSBUFF,
    AGC_DN_ITEM_COUNT
} agc_dn_item_t;

/* Entry op codes: 1..6 = 1DNADR..6DNADR (n DP words), DNCHAN, DNPTR.      */
#define AGC_DN_OP_CHAN 7
#define AGC_DN_OP_PTR  8

typedef struct agc_dn_entry {
    uint8_t  op;      /* 1..6 = nDNADR, AGC_DN_OP_CHAN, AGC_DN_OP_PTR       */
    bool     neg;     /* written negated in the AGC list (-xDNADR etc.)    */
    uint16_t ref;     /* agc_dn_item_t, channel number, or sublist id       */
    uint8_t  offset;  /* SP-word offset from the item (e.g. RN +2 -> 2)     */
} agc_dn_entry_t;

typedef struct agc_dn_list {
    const agc_dn_entry_t *entry;
    uint8_t               count;
    bool                  snapshot;  /* sublist starting with -1DNADR      */
} agc_dn_list_t;

#define AGC_DNLIST_CSM_COUNT 5   /* DNTABLE: COAST, ENTRY/UPDATE, REND, POWERED, P22 */
#define AGC_DNLIST_LM_COUNT  6   /* DNTABLE: COAST, AGS, REND, ORBMAN, DESC/ASC, SURF */
#define AGC_DNLIST_WORDS     100 /* DP words per downlist (2 s at 50 words/s) */
#define AGC_DN_LOWIDCOD      077340u  /* LOWIDCOD, the sync word             */

agc_status_t agc_dnlist_csm_control(int code, agc_dn_list_t *out);
agc_status_t agc_dnlist_csm_sublist(int sub, agc_dn_list_t *out);
agc_status_t agc_dnlist_lm_control(int code, agc_dn_list_t *out);
agc_status_t agc_dnlist_lm_sublist(int sub, agc_dn_list_t *out);
/* Number of DP downlink words a control list produces, ID word included.  */
agc_status_t agc_dnlist_word_count(agc_vehicle_t v, int code, int *words);

/* ======================================================================
 * Down-telemetry program (telemetry.c; DOWN_TELEMETRY_PROGRAM.agc pp.988-997)
 * ==================================================================== */
agc_status_t agc_telem_fresh_start(int dnlstcod);   /* DNLSTCOD, DNTMGOTO init */
agc_status_t agc_telem_restart(void);               /* STARTSUB: restart list  */
agc_status_t agc_telem_select_list(int dnlstcod);   /* TS DNLSTCOD             */
int          agc_telem_list_code(void);             /* C(DNLSTCOD)             */
agc_status_t agc_telem_downrupt(void);              /* DODOWNTM, every 20 ms   */
agc_status_t agc_telem_start_dump(void);            /* V74: DNTMGOTO = DNDUMPI */
agc_status_t agc_telem_set_dumpcnt(uint16_t dumpcnt); /* DUMPCNT (V21N01)      */
uint16_t     agc_telem_dumpcnt(void);
uint16_t     agc_erase_agsword(void);               /* AGSWORD (LM): saved list */
void         agc_erase_set_agsword(uint16_t v);

/* ======================================================================
 * Service routines (service.c; SERVICE_ROUTINES.agc C pp.1485-1492)
 * ==================================================================== */
uint16_t     agc_svc_upflag(agc_flag_id_t f);    /* UPFLAG: returns flagword */
uint16_t     agc_svc_downflag(agc_flag_id_t f);  /* DOWNFLAG                 */
agc_status_t agc_svc_upent2(uint16_t arg);       /* UPENT2 (FLAGWRD4 on LM)  */
agc_status_t agc_svc_downent2(uint16_t arg);     /* DOWNENT2                 */
agc_status_t agc_svc_delayjob(uint16_t delay_cs, agc_job_id_t wake_as); /* DELAYJOB */
agc_status_t agc_svc_delay_wake(int slot);       /* WAKER task body (-1)     */
agc_status_t agc_svc_startsub_reset(void);       /* DELAYLOC = 0             */
agc_status_t agc_svc_gentran(const uint16_t *from, uint16_t *to, int n); /* GENTRAN */
agc_status_t agc_svc_b5off(void);                /* B5OFF                    */
agc_status_t agc_svc_trfailof(void);             /* TRFAILOF (CM)            */
agc_status_t agc_svc_trfailon(void);             /* TRFAILON (CM)            */

/* ======================================================================
 * T4RUPT (t4rupt.c + src/io/service/t4rupt_csm.c + src/io/service/t4rupt_lm.c)
 * ==================================================================== */
/* Run one T4RUPT; *next_cs receives the TIME4 reload (centiseconds until
 * the next T4RUPT: 2 for the 20 ms DSKY relay passes, 12 otherwise).     */
agc_status_t agc_t4rupt(uint16_t *next_cs);
agc_status_t agc_t4rupt_rupt(void);              /* rupt entry: + TIME4      */
agc_status_t agc_t4_startsub_reset(void);        /* STARTSUB: DSRUPTSW = +0  */
agc_status_t agc_t4_fresh_start_flush(void);     /* SLAP1: DSRUPTSW = -PRIO12 */
agc_status_t agc_t4_task(agc_task_id_t task);    /* ENDTNON UNZ2 ISSUP ENDZOPT */
agc_status_t agc_t4_setissw(void);               /* SETISSW (called by imu)  */
agc_status_t agc_t4_cagesub(void);               /* CAGESUB                  */
agc_status_t agc_t4_cagesub2(void);              /* CAGESUB2                 */
bool         agc_t4_lamptest(void);              /* LAMPTEST: true if on     */
agc_status_t agc_t4_imumon(void);                /* IMUMON .. GLOCKMON       */
agc_status_t agc_t4_tnontest(void);              /* TNONTEST                 */
agc_status_t agc_t4_c33test(void);               /* C33TEST                  */
agc_status_t agc_t4_glockmon(void);              /* GLOCKMON                 */
/* CSM optics (src/io/service/t4rupt_csm.c). OPTIND -0 is AGC_OPTIND_NEG0.      */
#define AGC_OPTIND_NEG0 ((agc_sp_t)-2)
agc_status_t agc_t4_optics_monitor(void);        /* OPTMON                   */
agc_status_t agc_t4_optics_drive(void);          /* OPTDRIVE                 */
agc_status_t agc_t4_optics_task(agc_task_id_t task); /* ENDZOPT              */
agc_status_t agc_t4_optics_reset(bool fresh);    /* OPTIND/OPTMODES init     */
agc_sp_t     agc_t4_optics_cmd(int which);       /* CDUTCMD(0)/CDUSCMD(1)    */
uint16_t     agc_erase_optmodes(void);           /* OPTMODES (CM)            */
void         agc_erase_set_optmodes(uint16_t v);
agc_sp_t     agc_erase_optind(void);             /* OPTIND (CM)              */
void         agc_erase_set_optind(agc_sp_t v);
agc_sp_t     agc_erase_swsample(void);           /* SWSAMPLE (CM) +1 CSC, 0 manual, -1 zero optics */
/* LM radar monitor and DAP matrix (src/io/service/t4rupt_lm.c).                 */
agc_status_t agc_t4_rr_monitor(void);            /* RRAUTCHK..RRGIMON+GPMATRIX */
agc_status_t agc_t4_gimbal_matrix(void);         /* GPMATRIX / DAPT4S        */
uint16_t     agc_erase_radmodes(void);           /* RADMODES (LM)            */
void         agc_erase_set_radmodes(uint16_t v);
agc_sp_t     agc_erase_m11(void);                /* SIN(MG), scaled at 1     */
agc_sp_t     agc_erase_m21(void);                /* COS(OG)COS(MG)           */
agc_sp_t     agc_erase_m22(void);                /* SIN(OG)                  */
agc_sp_t     agc_erase_m31(void);                /* -SIN(OG)COS(MG)          */
agc_sp_t     agc_erase_m32(void);                /* COS(OG)                  */
agc_sp_t     agc_erase_cosmg(void);              /* COS(MG)                  */

/* ======================================================================
 * Extended verbs (src/io/verbs/extended_verbs{,_csm,_lm}.c) and the update
 * program P27 (src/io/verbs/uplink_update.c)
 * ==================================================================== */
agc_status_t agc_extverb(int verb, int noun);            /* GOEXTVB (pinball) */
bool         agc_extverb_busy(void);                     /* EXTVBACT != 0   */
/* EXTVBACT: extended-verb / marking-system activity word (bit 2 marking
 * system in use, bit 5 an extended verb is active, bit 14 P61 lock-out). */
uint16_t     agc_erase_extvbact(void);
void         agc_erase_set_extvbact(uint16_t v);
agc_status_t agc_svc_set_extvbact(uint16_t v);           /* TS EXTVBACT     */
/* jobs and tasks (ids in src/io/verbs/pending.h until integration)       */
agc_status_t agc_job_extended_verb(agc_job_ctx_t *ctx);           /* LST2FAN   */
agc_status_t agc_job_extended_verb_end(agc_job_ctx_t *ctx);       /* ENDEXT    */
agc_status_t agc_job_extended_verb_gopin(agc_job_ctx_t *ctx);     /* GOPIN     */
agc_status_t agc_job_keyboard_coarse_align(agc_job_ctx_t *ctx);   /* V41 ICORK2 */
agc_status_t agc_job_keyboard_coarse_drive(agc_job_ctx_t *ctx);   /* V41 IMUCOARS */
agc_status_t agc_job_keyboard_fine_align(agc_job_ctx_t *ctx);     /* V42       */
agc_status_t agc_job_keyboard_fine_mode(agc_job_ctx_t *ctx);      /* V42 IMUFINE */
agc_status_t agc_job_keyboard_gyro_torque(agc_job_ctx_t *ctx);    /* V42 FINEK2 */
agc_status_t agc_job_clock_align_update(agc_job_ctx_t *ctx);      /* V55 UPDATIME */
agc_status_t agc_job_show_banksum_display(agc_job_ctx_t *ctx);    /* V91 NOKILL */
agc_status_t agc_job_show_banksum_next(agc_job_ctx_t *ctx);       /* V91 NXTBNK */
agc_status_t agc_job_show_banksum_end(agc_job_ctx_t *ctx);        /* V91 V34   */
agc_status_t agc_job_attitude_error_load(agc_job_ctx_t *ctx);     /* V43       */
agc_status_t agc_job_attitude_error_terminate(agc_job_ctx_t *ctx); /* V43 CSM TRMATTCK */
agc_status_t agc_task_attitude_error_output(void);                /* V43 ATTCK1/2 */
agc_status_t agc_job_dap_data_load_noun46(agc_job_ctx_t *ctx);    /* R03 N46   */
agc_status_t agc_job_dap_data_load_config(agc_job_ctx_t *ctx);    /* R03 N46 done */
agc_status_t agc_job_dap_data_load_noun47(agc_job_ctx_t *ctx);    /* R03 N47   */
agc_status_t agc_job_dap_data_load_mass(agc_job_ctx_t *ctx);      /* R03 N47 done */
agc_status_t agc_job_dap_data_load_noun48(agc_job_ctx_t *ctx);    /* R03 N48   */
agc_status_t agc_job_dap_data_load_trim(agc_job_ctx_t *ctx);      /* LM DPDAT3 */
agc_status_t agc_job_dap_data_load_end(agc_job_ctx_t *ctx);       /* LM ENDR03 */
agc_status_t agc_job_trimdone(agc_job_ctx_t *ctx);                /* LM TRIMDONE */
agc_status_t agc_job_state_vector_copy(agc_job_ctx_t *ctx);       /* V66 / CSM V47 */
agc_status_t agc_job_tracking_terminate(agc_job_ctx_t *ctx);      /* CSM V56 */
agc_status_t agc_job_attitude_error_enable(agc_job_ctx_t *ctx);   /* LM V43 */

/* P27 (V70-V73).  The uplinked words are UPBUFF; COMPNUMB, UPOLDMOD,
 * UPVERB, UPCOUNT, UPBUFF +0..+19D are downlinked (index 0..23).          */
agc_status_t agc_uplink_update_start(agc_job_ctx_t *ctx, int verb); /* V70..V73 */
uint16_t     agc_uplink_update_dnword(int idx);
agc_status_t agc_uplink_update_fresh_start(void);         /* UPSVFLAG etc.   */
/* The word an ECADR load/write puts into COMPNUMB..UPBUFF +19D (0..23) or
 * UPTEMP (24); agc_uplink_update_target() = the index the V21N01/N02 on
 * the DSKY loads (the AGC's MPAC+2).                                      */
agc_status_t agc_uplink_update_write(int idx, uint16_t word);
int          agc_uplink_update_target(void);
agc_sp_t     agc_erase_upsvflag(void);   /* UPSVFLAG: 0, +-1 earth, +-2 moon (+CSM -LM) */
void         agc_erase_set_upsvflag(agc_sp_t v);
agc_status_t agc_job_uplink_update_load(agc_job_ctx_t *ctx);      /* after V21N01 */
agc_status_t agc_job_uplink_update_execute(agc_job_ctx_t *ctx);   /* UPJOB  */
agc_status_t agc_job_uplink_update_terminate(agc_job_ctx_t *ctx); /* UPOUT +1 */
agc_status_t agc_job_uplink_update_proceed(agc_job_ctx_t *ctx);   /* V33 on a load */
agc_status_t agc_job_timedidr(agc_job_ctx_t *ctx);                /* restart 6.5 */

/* ======================================================================
 * Self-check and show-banksum (src/io/service/self_check.c;
 * AGC_BLOCK_TWO_SELF_CHECK.agc)
 * ==================================================================== */
#define AGC_ROPE_BANK_WORDS 02000   /* 1024 words per fixed bank          */
#define AGC_ROPE_BANKS      044     /* banks 0..43 octal                  */
/* The sim's fixed-memory image: nbanks banks of 1024 words, bank b at
 * rope[b * 1024], 15-bit words.  NULL = no rope (all words zero).         */
agc_status_t agc_selfcheck_set_rope(const uint16_t *rope, int nbanks);
/* Sum one bank as ROPECHK / SHOWSUM do: sum = the ones'-complement sum
 * with end-around carry up to the two TC SELF words and the bugger word. */
agc_status_t agc_selfcheck_banksum(int bank, uint16_t *sum, uint16_t *bugger);
bool         agc_selfcheck_bank_ok(int bank);             /* |sum| = bank    */
agc_status_t agc_selfcheck_set_smode(uint16_t word);      /* N27 load (SMODE, raw; 077777 = -0) */
uint16_t     agc_selfcheck_smode(void);
agc_status_t agc_selfcheck_idle_pass(void);               /* SMODECHK, one pass */
agc_status_t agc_selfcheck_fresh_start(bool slap1);       /* SMODE = 0; SLAP1: ERCOUNT = 0 */
agc_status_t agc_selfcheck_showsum_start(void);           /* V91 STSHOSUM    */
agc_status_t agc_selfcheck_showsum_next(void);            /* V33 NXTBNK      */
agc_status_t agc_selfcheck_showsum_end(void);             /* V34             */
uint16_t     agc_selfcheck_skeep(int i);                  /* SKEEP1..3 (N01) */
uint16_t     agc_selfcheck_error_count(void);             /* ERCOUNT         */

/* ======================================================================
 * P07 IMU performance tests (src/lm/systems_test.c; Luminary099
 * SYSTEM_TEST_STANDARD_LEAD_INS, IMU_PERFORMANCE_TEST_2, _TESTS_4)
 * ==================================================================== */
agc_status_t agc_systems_test_zeroing(uint16_t *block, int n);     /* ZEROING  */
agc_status_t agc_systems_test_finetime(agc_dp_t *t);               /* FINETIME */
agc_dp_t     agc_systems_test_overffix(agc_interp_t *ip, agc_dp_t a); /* OVERFFIX */
agc_status_t agc_job_systems_test(agc_job_ctx_t *ctx);             /* REDO (V92) */

#endif /* AGC_SYSMISC_H */
