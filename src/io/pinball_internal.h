/*
 * pinball_internal.h -- private state and helpers shared by the pinball
 * translation units (src/io/pinball_*.c, src/io/display_if.c).
 *
 * The AGC keeps pinball's erasables in fixed locations (PINBALL p. 310-312).
 * Here they live in ONE static struct in pinball_state.c, reached through
 * pin_st() so that the module's several .c files share it without extern
 * globals (rule 6).
 */
#ifndef PINBALL_INTERNAL_H
#define PINBALL_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/agc_assert.h"
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_exec.h"
#include "agc/agc_dsky.h"
#include "agc/agc_nouns.h"
#include "agc/agc_channels.h"
#include "agc/agc_flags_common.h"
#include "agc/agc_state.h"

/* ---- DSPCOUNT numbers (PINBALL p. 313, octal) -------------------------- */
#define PIN_MD1   025
#define PIN_VD1   023
#define PIN_ND1   021
#define PIN_ND2   020
#define PIN_R1D1  016
#define PIN_R2D1  011
#define PIN_R3D1  004
#define PIN_DSPTAB_LEN 12          /* DSPTAB +0..+10 panel, +11 C relays (core) */
#define PIN_DSPTAB_PANEL 11        /* TABLNTH + 1                            */

/* ---- INREL values (GETINREL, p. 318) ----------------------------------- */
#define PIN_INREL_VERB 0
#define PIN_INREL_NOUN 1
#define PIN_INREL_X    2
#define PIN_INREL_Y    3
#define PIN_INREL_Z    4

/* ---- AGC constants used across files ----------------------------------- */
#define PIN_BIT15  040000u
#define PIN_BIT14  020000u
#define PIN_BIT12  004000u
#define PIN_BIT11  002000u
#define PIN_LOW5   000037u
#define PIN_LOW7   000177u
#define PIN_LOW11  003777u
#define PIN_POSMAX 037777u
#define PIN_WORD   077777u          /* 15-bit word mask                      */
#define PIN_NEGZERO 077777u         /* ones' complement -0                   */
#define PIN_CHRPRIO ((agc_prio_t)030) /* CHRPRIO OCT 30000 (>> 9)            */
#define PIN_PRIO32  ((agc_prio_t)032)
#define PIN_PRIO33  ((agc_prio_t)033)
#define PIN_PRIO15  ((agc_prio_t)015)
#define PIN_MONDEL_CS ((uint16_t)0144) /* MONDEL OCT 144 = 100 cs = 1 s      */
#define PIN_SHOLTS_CS ((uint16_t)0764) /* SHOLTS OCT 764 = 500 cs = 5 s      */
#define PIN_ALARM_DSPALARM ((agc_alarm_code_t)01501) /* bad NVSUB V/N       */
#define PIN_ALARM_DSPABORT ((agc_alarm_code_t)01206) /* 2nd sleeper         */
#define PIN_ALARM_PRIOBORT ((agc_alarm_code_t)01502) /* display conflict    */
#define PIN_MIXCON 40               /* MIXCON: first mixed noun = 40 decimal */
#define PIN_LOWVERB 28              /* LOWVERB: verbs >= 28 skip noun test   */
#define PIN_LST2CON 40              /* LST2CON: first extended verb          */

/* ---- channel bits (agc_channels.h names) ------------------------------- */
#define PIN_CH11_UPACT   ((uint16_t)AGC_CH011_UPACT)      /* bit 3            */
#define PIN_CH11_TEMP    ((uint16_t)AGC_CH011_TEMPCAUT)   /* bit 4            */
#define PIN_CH11_KEYREL  ((uint16_t)AGC_CH011_KEYREL)     /* bit 5            */
#define PIN_CH11_FLASH   ((uint16_t)AGC_CH011_VNFLASH)    /* bit 6            */
#define PIN_CH11_OPRERR  ((uint16_t)AGC_CH011_OPERERR)    /* bit 7            */
#define PIN_CH11_CAUTRST ((uint16_t)AGC_CH011_CAUTRESET)  /* bit 10           */
#define PIN_CH13_TESTALM ((uint16_t)AGC_CH013_TESTALM)    /* bit 10           */
#define PIN_CH12_COARSE  ((uint16_t)AGC_CH012_COARSE)     /* bit 4            */

/* ---- job and task ids pinball needs (Requested jobs/tasks in the note).
 * INTEGRATION: replace each with AGC_JOB_<LABEL>/AGC_TASK_<LABEL> once
 * agc_jobs.h carries them; until then they are placeholders past the end
 * of the lead's enum so that this module compiles.                     */
#define PIN_JOB_CHARIN   AGC_JOB_CHARIN
#define PIN_JOB_PROCKEY  ((agc_job_id_t)(AGC_JOB_COUNT_PLACEHOLDER + 1))
#define PIN_JOB_MONDO    ((agc_job_id_t)(AGC_JOB_COUNT_PLACEHOLDER + 2))
#define PIN_JOB_DSPMMJB  ((agc_job_id_t)(AGC_JOB_COUNT_PLACEHOLDER + 3))
#define PIN_JOB_TSTLTS3  ((agc_job_id_t)(AGC_JOB_COUNT_PLACEHOLDER + 4))
#define PIN_JOB_MAKEPLAY ((agc_job_id_t)(AGC_JOB_COUNT_PLACEHOLDER + 5))
#define PIN_JOB_PLAYJUM1 ((agc_job_id_t)(AGC_JOB_COUNT_PLACEHOLDER + 6))
#define PIN_JOB_MARKPLAY ((agc_job_id_t)(AGC_JOB_COUNT_PLACEHOLDER + 7))
#define PIN_JOB_REDOPRIO ((agc_job_id_t)(AGC_JOB_COUNT_PLACEHOLDER + 8))
#define PIN_JOB_TERMATE  ((agc_job_id_t)(AGC_JOB_COUNT_PLACEHOLDER + 9))
#define PIN_JOB_PROCEED  ((agc_job_id_t)(AGC_JOB_COUNT_PLACEHOLDER + 10))
#define PIN_JOB_IDLERET3 ((agc_job_id_t)(AGC_JOB_COUNT_PLACEHOLDER + 11))
#define PIN_JOB_JAMTERM  ((agc_job_id_t)(AGC_JOB_COUNT_PLACEHOLDER + 12))
#define PIN_TASK_MONREQ  ((agc_task_id_t)(AGC_TASK_COUNT_PLACEHOLDER + 1))
#define PIN_TASK_TSTLTS2 ((agc_task_id_t)(AGC_TASK_COUNT_PLACEHOLDER + 2))

/* ---- hand-offs to other lanes and requested accessors -----------------
 * Each is listed under "Requested core additions" in
 * docs/modules/io-pinball.md with its owner. Declared here (not in a
 * core header) so the module compiles before the owners deliver.       */
agc_status_t agc_extverb_fan(uint8_t verb_minus_40);   /* GOEXTVB, sys-misc */
agc_status_t agc_v37_change_program(uint8_t new_mm);   /* V37, exec-sched   */
agc_status_t agc_fresh_start_slap1(void);              /* SLAP1, exec-sched */
agc_status_t agc_erase_read_ecadr(uint16_t ecadr, uint16_t *word);  /* state */
agc_status_t agc_erase_write_ecadr(uint16_t ecadr, uint16_t word);  /* state */
uint16_t     agc_erase_optmodes(void);                 /* CSM only, state  */
void         agc_erase_set_optmodes(uint16_t v);
uint16_t     agc_erase_radmodes(void);                 /* LM only, state   */
void         agc_erase_set_radmodes(uint16_t v);
void         agc_erase_set_samptime(uint32_t t_cs);    /* SAMPTIME (N65)   */
void         agc_erase_set_option1(agc_sp_t v);        /* OPTION1 (GOPERF4) */
void         agc_erase_set_sfail(uint16_t v);          /* SFAIL (ERROR)    */

/* ---- AGC SF routine numbers (SFOUTABR / SFINTABR, p. 334 / 349) -------- */
enum pin_sfrout {
    PIN_SF_OCTAL = 0,   /* octal only                                       */
    PIN_SF_FRACT,       /* straight fractional                              */
    PIN_SF_DEG,         /* CDU degrees                                      */
    PIN_SF_ARITH,       /* arithmetic (point at left of SF constant)        */
    PIN_SF_DP1,         /* DP1OUT                                           */
    PIN_SF_DP2,         /* DP2OUT                                           */
    PIN_SF_OPDEG,       /* CM: optics degrees.  LM: landing radar position  */
    PIN_SF_DP3,         /* DP3OUT                                           */
    PIN_SF_HMS,         /* hours minutes seconds                            */
    PIN_SF_MS,          /* minutes/seconds MMbSS (display only)             */
    PIN_SF_DP4,         /* DP4OUT                                           */
    PIN_SF_ARITH1,      /* arithmetic, point between hi and lo of SF const  */
    PIN_SF_2INT,        /* two integers (display only)                      */
    PIN_SF_DPFRAC,      /* CM: DP fraction.  LM: 360 - CDU degrees          */
    PIN_SF_COUNT
};

/* ---- REQRET: where ENTER continues a load (replaces the AGC address) --- */
typedef enum pin_reqret {
    PIN_REQ_PASS0 = 0,       /* +0 / positive: ENTER executes V/N (ENTPAS0) */
    PIN_REQ_ABC_X,           /* ABCLOAD after REQDATX                       */
    PIN_REQ_ABC_Y,           /* ABCLOAD after REQDATY                       */
    PIN_REQ_ABC_Z,           /* ABCLOAD after REQDATZ -> PUTXYZ             */
    PIN_REQ_AB_X,            /* ABLOAD after REQDATX                        */
    PIN_REQ_AB_Y,            /* ABLOAD after REQDATY -> PUTXY               */
    PIN_REQ_A_X,             /* ALOAD after REQDATX                         */
    PIN_REQ_B_Y,             /* BLOAD after REQDATY                         */
    PIN_REQ_C_Z,             /* CLOAD after REQDATZ                         */
    PIN_REQ_MCTBS,           /* REQADD after REQDATZ (machine address)      */
    PIN_REQ_MM,              /* MMCHANG after REQMM                         */
    PIN_REQ_COUNT
} pin_reqret_t;

/* ---- ENTRET: exit taken by ENTEXIT ------------------------------------- */
typedef enum pin_entret {
    PIN_ENT_ENDOFJOB = 0,    /* TC ENDOFJOB (keyboard)                      */
    PIN_ENT_NVSUBEND,        /* TC NVSUBEND (internal NVSUB caller)         */
    PIN_ENT_PASTEVB          /* ADRES PASTEVB (monitor display)             */
} pin_entret_t;

/* ---- LOADSTAT (p. 372) ------------------------------------------------- */
typedef enum pin_loadstat {
    PIN_LS_WAITING = 0,      /* +0 inactive, waiting for data               */
    PIN_LS_PROCEED,          /* +1 proceed without data (V33)               */
    PIN_LS_TERMINATE,        /* -1 terminate (V34)                          */
    PIN_LS_DATAIN            /* -0 data in or resequence (V32)              */
} pin_loadstat_t;

/* A job put to sleep inside pinball (CADRSTOR or DSPLIST). The AGC woke
 * the same job; we start the continuation at the same priority.        */
typedef struct pin_sleeper {
    bool         used;
    agc_job_id_t cont[3];    /* terminate, proceed, data-in (DSPLIST: [0])  */
    agc_prio_t   prio;
    bool         vac;
} pin_sleeper_t;

typedef struct pin_state {
    int16_t  nout;                   /* NOUT  activity counter for DSPTAB    */
    int16_t  dspcnt;                 /* DSPCNT counter for DSPOUT            */
    int16_t  dspcount;               /* DSPCOUNT magnitude (may carry BIT14) */
    bool     dspcount_neg;           /* DSPCOUNT sign: true = blocked        */
    uint16_t decbrnch;               /* DECBRNCH                             */
    uint8_t  verbreg;                /* VERBREG                              */
    uint8_t  nounreg;                /* NOUNREG                              */
    int32_t  vnacc[2];               /* verb/noun digit accumulators         */
    uint16_t xyz_oct[3];             /* XREG YREG ZREG (octal word)          */
    int32_t  xyz_dec[3];             /* decimal digits entered (integer N)   */
    agc_dp_t xyz_frac[3];            /* XREG,XREGLP.. after DECEND +-N/10^5  */
    bool     xyz_negzero[3];         /* -00000 typed (ones' complement -0)   */
    pin_reqret_t reqret;             /* REQRET continuation                  */
    bool     reqret_neg;             /* REQRET negative: waiting for data    */
    pin_entret_t entret;             /* ENTRET                               */
    uint8_t  verbsave;               /* VERBSAVE (AGC keeps -verb)           */
    int16_t  clpass;                 /* CLPASS: 0 pass0, + hipass, - no back */
    pin_loadstat_t loadstat;         /* LOADSTAT                             */
    bool     dsplock;                /* DSPLOCK  (1 = keyboard busy)         */
    bool     old_dsplock;            /* 21/22REG: DSPLOCK before CHARIN      */
    pin_sleeper_t cadrstor;          /* CADRSTOR (ENDIDLE)                   */
    pin_sleeper_t dsplist;           /* DSPLIST  (NVSUBUSY)                  */
    uint16_t monsave;                /* MONSAVE  verb<<7 | noun, +0 = off    */
    uint16_t monsave1;               /* MONSAVE1 bit15 kill, bit14 external  */
    uint16_t monsave2;               /* MONSAVE2 NVMONOPT options            */
    uint16_t nouncadr;               /* NOUNCADR machine address (MCTBS)     */
    agc_noun_t noun;                 /* LODNNTAB result for NOUNREG          */
    bool     noun_valid;             /* NNADTEM not +0                       */
    uint8_t  mixbr;                  /* MIXBR 1 normal, 2 mixed              */
    uint8_t  nntype;                 /* 0 normal, 1 MCTBS, 2 channel, 3 augment */
    uint16_t dsptem[7];              /* DSPTEM1 +0..+2, DSPTEM2 +0..+2, +3   */
    uint8_t  resp_verb;              /* MPAC at ENDIDLE wake (verb)          */
    uint8_t  resp_noun;              /* MPAC +1 at ENDIDLE wake (noun)       */
    uint16_t mctbs_cadr;             /* MPAC +2 for internal MCTBS           */
    uint8_t  keyq[8];                /* key codes waiting for CHARIN jobs    */
    uint8_t  keyq_head;
    uint8_t  keyq_count;
} pin_state_t;

#define PIN_KEYQ_LEN 8
#define PIN_NNTYPE_NORMAL 0
#define PIN_NNTYPE_MCTBS  1
#define PIN_NNTYPE_CHAN   2
#define PIN_NNTYPE_AUGMENT 3

pin_state_t *pin_st(void);

/* ---- word helpers (pinball_state.c) ------------------------------------ */
bool     pin_ones_negative(uint16_t w);
uint16_t pin_ones_mag(uint16_t w);
agc_sp_t pin_sp_from_ones(uint16_t w);
uint16_t pin_ones_from_sp(agc_sp_t v);
uint16_t pin_ones_neg(uint16_t mag);

/* ---- lamps (pinball_state.c) ------------------------------------------- */
void pin_falton(void);             /* FALTON   */
void pin_flashon(void);            /* FLASHON  */
void pin_flashoff(void);           /* FLASHOFF */
void pin_reldspon(void);           /* RELDSPON */
void pin_lodsampt(void);           /* LODSAMPT */

/* ---- DSPCOUNT and DSPTAB (pinball_state.c) ----------------------------- */
void pin_set_dspcount(int16_t n);           /* TS DSPCOUNT (positive)       */
void pin_block_dspcount(int16_t n);         /* CS n; TS DSPCOUNT            */
int  pin_getinrel(void);                    /* GETINREL                     */
int  pin_inrel_of(int16_t dspcount);        /* INRELTAB lookup              */
agc_status_t pin_dspin(int16_t count, uint16_t code);   /* DSPIN            */
agc_status_t pin_11dspin(int rel, bool on);             /* 11DSPIN          */
agc_status_t pin_2blank(void);                          /* 2BLANK           */
agc_status_t pin_5blank(void);                          /* 5BLANK           */
agc_status_t pin_dsptab_store(int rel, uint16_t mag);   /* -mag, NOUT bump  */

/* ---- word display (pinball_display.c) ---------------------------------- */
agc_status_t pin_plus_on(void);                         /* +ON              */
agc_status_t pin_minus_on(void);                        /* -ON              */
agc_status_t pin_dspdecwd(agc_dp_t v, bool negzero);    /* DSPDECWD         */
agc_status_t pin_dspdecnr(agc_dp_t v, bool negzero);    /* DSPDECNR         */
agc_status_t pin_dspdc2nr(agc_dp_t v, bool negzero);    /* DSPDC2NR         */
agc_status_t pin_dsp2dec(agc_dp_t v);                   /* DSP2DEC          */
agc_status_t pin_dspdecvn(int n);                       /* DSPDECVN         */
agc_status_t pin_dspoctwo(uint16_t w);                  /* DSPOCTWO         */
agc_status_t pin_dsp2bit(uint16_t w);                   /* DSP2BIT          */
agc_status_t pin_updatvb(uint8_t verb);                 /* UPDATVB -1       */
agc_status_t pin_updatnn(uint8_t noun);                 /* UPDATNN -1       */

/* ---- keyboard (pinball_keys.c) ----------------------------------------- */
agc_status_t pin_charin(uint8_t code);                  /* CHARIN           */
agc_status_t pin_enter(void);                           /* ENTER            */
agc_status_t pin_vbreldsp(void);                        /* VBRELDSP         */

/* ---- verb execution (pinball_verbs.c) ---------------------------------- */
agc_status_t pin_entpas0(void);                         /* ENTPAS0          */
agc_status_t pin_testnn(void);                          /* TESTNN           */
agc_status_t pin_entexit(void);                         /* TC ENTEXIT       */
agc_status_t pin_dspalarm(void);                        /* DSPALARM         */
agc_status_t pin_predspal(void);                        /* PREDSPAL         */
agc_status_t pin_charalrm(void);                        /* CHARALRM         */
agc_status_t pin_almcycle(void);                        /* ALMCYCLE         */
agc_status_t pin_pastevb(void);                         /* PASTEVB          */
agc_status_t pin_noun_load(void);                       /* LODNNTAB         */
agc_status_t pin_vbproc_common(pin_loadstat_t ls);      /* VBPROC +1        */
agc_status_t pin_monitor(void);                         /* MONITOR          */
agc_status_t pin_vbtstlts(void);                        /* VBTSTLTS         */
agc_status_t pin_mmchang_resume(void);                  /* MMCHANG +1       */
agc_status_t pin_reqmm(void);                           /* REQMM            */

/* ---- loads (pinball_load.c) -------------------------------------------- */
agc_status_t pin_load_verb(uint8_t verb);               /* ALOAD..ABCLOAD   */
agc_status_t pin_load_resume(pin_reqret_t step);        /* TC REQRET        */
agc_status_t pin_reqdat(int inrel, pin_reqret_t step);  /* REQDATX/Y/Z      */
agc_status_t pin_loadlv(void);                          /* LOADLV           */
pin_reqret_t pin_reqret_backup(pin_reqret_t step);      /* REQRET +3        */

/* ---- scaling (pinball_scale.c) ----------------------------------------- */
agc_status_t pin_comp_sf(int k, uint8_t *rout, agc_dp_t *sfout, agc_dp_t *sfin);
bool         pin_sf_is_dp(uint8_t rout);                /* DPTEST           */
agc_status_t pin_decdsp_comp(int k);                    /* DSPDCPUT for k   */
agc_status_t pin_sfin_comp(int k, agc_dp_t *out, bool *recycle);  /* PUTDECSF */
agc_status_t pin_comp_read(int k, agc_dp_t *v);         /* noun component   */
agc_status_t pin_comp_write(int k, agc_dp_t v);

/* ---- NVSUB family (pinball_nvsub.c) ------------------------------------ */
agc_status_t pin_recaltst(void);                        /* RECALTST         */
agc_status_t pin_reldsp(void);                          /* RELDSP           */
agc_status_t pin_reldsp1(void);                         /* RELDSP1          */
void         pin_kilmonon(void);                        /* KILMONON         */
agc_status_t pin_start_sleeper(const pin_sleeper_t *s, int which);
agc_status_t pin_nvmonopt(int16_t nvtemp, uint16_t options, uint16_t mpac2);
bool         pin_dsp_blocked(void);                     /* MONSAVE1 EMB + DSPLOCK */
agc_status_t pin_blanksub1(uint16_t code);              /* BLNKSUB1         */

#endif /* PINBALL_INTERNAL_H */
