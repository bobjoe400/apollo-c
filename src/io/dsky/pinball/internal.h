/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private state (pin_state_t) and helpers shared by the pinball
 *   translation units (src/io/dsky/pinball/*.c, src/io/dsky/display_if.c).
 *
 * AGC SOURCE:
 *   PINBALL_GAME_BUTTONS_AND_LIGHTS.agc CM pp. 310-312 (the pinball
 *   erasables), LM pp. 393-395
 *
 * REQUIRES:
 *   none (self-contained; includes what it uses)
 *
 * GLOBAL VARIABLES:
 *   none: the AGC keeps pinball's erasables in fixed locations; here
 *   they live in ONE static struct in state.c, reached through pin_st(),
 *   so the module's .c files share it without extern globals (rule 6).
 *   Struct members carry their AGC name and scale in a short comment.
 *
 * NOTES:
 *   Control flow model. PINBALL jumps freely between labels; here every
 *   routine is a function returning agc_status_t:
 *     AGC_STATUS_OK       the AGC path ended in ENDOFJOB, TC ENTEXIT or a
 *                         plain return -- the caller goes on to its own
 *                         end (job end, NVSUBEND, PASTEVB);
 *     PIN_STATUS_RECYCLE  ALMCYCLE ran: the AGC re-enters ENTER with the
 *                         original verb; pin_enter() loops on it (no
 *                         recursion, rule 1);
 *     AGC_STATUS_POODOO   DSPALARM from an internal NVSUB (abort 01501) or
 *                         DSPABORT (01206);
 *     other               a parameter/assertion failure.
 **********************************************************************/
#ifndef PINBALL_INTERNAL_H
#define PINBALL_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/io/dsky.h"
#include "agc/io/nouns.h"
#include "agc/io/service.h"

#define PIN_STATUS_RECYCLE AGC_STATUS_ALARM /* ALMCYCLE: re-enter ENTER  */
#define PIN_RECYCLE_MAX    3 /* ENTER passes per key (ALMCYCLE bound)  */
#define PIN_CLPASS_NOBACK \
    ((int16_t) -16383) /* CAF BIT15 TS CLPASS (pass 0 only) */

/* ---- DSPCOUNT numbers (PINBALL p. 313, octal) -------------------------- */
#define PIN_MD1          025
#define PIN_VD1          023
#define PIN_VD2          022
#define PIN_ND1          021
#define PIN_ND2          020
#define PIN_R1D1         016
#define PIN_R2D1         011
#define PIN_R3D1         004
#define PIN_DSPTAB_LEN   12 /* DSPTAB +0..+10 panel, +11 C relays (core) */
#define PIN_DSPTAB_PANEL 11 /* TABLNTH + 1                            */

/* ---- INREL values (GETINREL, p. 318) ----------------------------------- */
#define PIN_INREL_VERB 0
#define PIN_INREL_NOUN 1
#define PIN_INREL_X    2
#define PIN_INREL_Y    3
#define PIN_INREL_Z    4

/* ---- AGC constants used across files ----------------------------------- */
#define PIN_BIT1           000001u
#define PIN_BIT3           000004u
#define PIN_BIT4           000010u
#define PIN_BIT5           000020u
#define PIN_BIT6           000040u
#define PIN_BIT7           000100u
#define PIN_BIT10          001000u
#define PIN_BIT11          002000u
#define PIN_BIT12          004000u
#define PIN_BIT13          010000u
#define PIN_BIT14          020000u
#define PIN_BIT15          040000u
#define PIN_LOW5           000037u
#define PIN_LOW7           000177u
#define PIN_LOW9           000777u
#define PIN_LOW11          003777u
#define PIN_MID7           037600u /* MID7: bits 8-14 (a verb in a VN word) */
#define PIN_POSMAX         037777u
#define PIN_WORD           077777u /* 15-bit word mask                      */
#define PIN_NEGZERO        077777u /* ones' complement -0                   */
#define PIN_CHRPRIO        ((agc_prio_t) 030) /* CHRPRIO OCT 30000 (>> 9) */
#define PIN_PRIO15         ((agc_prio_t) 015)
#define PIN_PRIO32         ((agc_prio_t) 032)
#define PIN_PRIO33         ((agc_prio_t) 033)
#define PIN_MONDEL_CS      ((uint16_t) 0144) /* MONDEL OCT 144 = 100 cs = 1 s */
#define PIN_SHOLTS_CS      ((uint16_t) 0764) /* SHOLTS OCT 764 = 500 cs = 5 s */
#define PIN_ALARM_DSPALARM ((agc_alarm_code_t) 01501) /* bad NVSUB V/N       */
#define PIN_ALARM_DSPABORT ((agc_alarm_code_t) 01206) /* 2nd sleeper         */
#define PIN_ALARM_PRIOBORT ((agc_alarm_code_t) 01502) /* display conflict    */
#define PIN_MIXCON         40 /* MIXCON: first mixed noun = 40 decimal */
#define PIN_LOWVERB        28 /* LOWVERB: verbs >= 28 skip noun test   */
#define PIN_LST2CON        40 /* LST2CON: first extended verb          */
#define PIN_LOWLOAD        22 /* LOWLOAD DEC 22 (V21..V23 test)        */

/* ---- channel bits (agc_channels.h names) ------------------------------- */
#define PIN_CH11_UPACT   ((uint16_t) AGC_CH011_UPACT)     /* bit 3            */
#define PIN_CH11_TEMP    ((uint16_t) AGC_CH011_TEMPCAUT)  /* bit 4            */
#define PIN_CH11_KEYREL  ((uint16_t) AGC_CH011_KEYREL)    /* bit 5            */
#define PIN_CH11_FLASH   ((uint16_t) AGC_CH011_VNFLASH)   /* bit 6            */
#define PIN_CH11_OPRERR  ((uint16_t) AGC_CH011_OPERERR)   /* bit 7            */
#define PIN_CH11_CAUTRST ((uint16_t) AGC_CH011_CAUTRESET) /* bit 10           */
#define PIN_CH13_TESTALM ((uint16_t) AGC_CH013_TESTALM)   /* bit 10           */
#define PIN_CH12_COARSE  ((uint16_t) AGC_CH012_COARSE)    /* bit 4            */

/* ---- job and task ids pinball needs (Requested jobs/tasks in the note).
 * INTEGRATION: replace each with AGC_JOB_<LABEL>/AGC_TASK_<LABEL> once
 * agc_jobs.h carries them; until then they are placeholders past the end
 * of the lead's enum so that this module compiles.                     */

/* ---- AGC SF routine numbers (SFOUTABR / SFINTABR, p. 334 / 349) -------- */
enum pin_sfrout
{
    PIN_SF_OCTAL = 0, /* octal only                                       */
    PIN_SF_FRACT,     /* straight fractional                              */
    PIN_SF_DEG,       /* CDU degrees                                      */
    PIN_SF_ARITH,     /* arithmetic (point at left of SF constant)        */
    PIN_SF_DP1,       /* DP1OUT                                           */
    PIN_SF_DP2,       /* DP2OUT                                           */
    PIN_SF_OPDEG,     /* CM: optics degrees.  LM: landing radar position  */
    PIN_SF_DP3,       /* DP3OUT                                           */
    PIN_SF_HMS,       /* hours minutes seconds                            */
    PIN_SF_MS,        /* minutes/seconds MMbSS (display only)             */
    PIN_SF_DP4,       /* DP4OUT                                           */
    PIN_SF_ARITH1,    /* arithmetic, point between hi and lo of SF const  */
    PIN_SF_2INT,      /* two integers (display only)                      */
    PIN_SF_DPFRAC,    /* CM: DP fraction.  LM: 360 - CDU degrees          */
    PIN_SF_COUNT
};

/* ---- REQRET: where ENTER continues a load (replaces the AGC address) --- */
typedef enum pin_reqret
{
    PIN_REQ_PASS0 = 0, /* +0 / positive: ENTER executes V/N (ENTPAS0) */
    PIN_REQ_ABC_X,     /* ABCLOAD after REQDATX                       */
    PIN_REQ_ABC_Y,     /* ABCLOAD after REQDATY                       */
    PIN_REQ_ABC_Z,     /* ABCLOAD after REQDATZ -> PUTXYZ             */
    PIN_REQ_AB_X,      /* ABLOAD after REQDATX                        */
    PIN_REQ_AB_Y,      /* ABLOAD after REQDATY -> PUTXY               */
    PIN_REQ_A_X,       /* ALOAD after REQDATX                         */
    PIN_REQ_B_Y,       /* BLOAD after REQDATY                         */
    PIN_REQ_C_Z,       /* CLOAD after REQDATZ                         */
    PIN_REQ_MCTBS,     /* REQADD after REQDATZ (machine address)      */
    PIN_REQ_MM,        /* MMCHANG after REQMM                         */
    PIN_REQ_COUNT
} pin_reqret_t;

/* ---- ENTRET: exit taken by ENTEXIT ------------------------------------- */
typedef enum pin_entret
{
    PIN_ENT_ENDOFJOB = 0, /* TC ENDOFJOB (keyboard)                      */
    PIN_ENT_NVSUBEND,     /* TC NVSUBEND (internal NVSUB caller)         */
    PIN_ENT_PASTEVB       /* ADRES PASTEVB (monitor display)             */
} pin_entret_t;

/* ---- LOADSTAT (p. 372) ------------------------------------------------- */
typedef enum pin_loadstat
{
    PIN_LS_WAITING = 0, /* +0 inactive, waiting for data               */
    PIN_LS_PROCEED,     /* +1 proceed without data (V33)               */
    PIN_LS_TERMINATE,   /* -1 terminate (V34)                          */
    PIN_LS_DATAIN       /* -0 data in or resequence (V32)              */
} pin_loadstat_t;

/* A job asleep inside pinball: CADRSTOR (ENDIDLE) or DSPLIST (NVSUBUSY).
 * cont[0] is the LOC the job sleeps at (the AGC's stored CADR); for
 * CADRSTOR cont[1], cont[2] are the proceed and data-in returns that
 * RECAL2 reaches by adding 1 or 2 to LOC.                              */
typedef struct pin_sleeper
{
    bool         used; /* a job waits here */
    agc_job_id_t cont[3]; /* terminate, proceed, data in */
} pin_sleeper_t;

typedef struct pin_state
{
    int16_t        nout;           /* NOUT  activity counter for DSPTAB    */
    int16_t        dspcnt;         /* DSPCNT counter for DSPOUT            */
    int16_t        dspcount;       /* DSPCOUNT magnitude (may carry BIT14) */
    bool           dspcount_neg;   /* DSPCOUNT sign: true = blocked        */
    uint16_t       decbrnch;       /* DECBRNCH                             */
    uint8_t        verbreg;        /* VERBREG                              */
    uint8_t        nounreg;        /* NOUNREG                              */
    uint8_t        chr;            /* CHAR: digit of the key being handled */
    uint16_t       xyz_oct[3];     /* XREG YREG ZREG (octal word)          */
    int32_t        xyz_dec[3];     /* decimal digits entered (integer N)   */
    agc_dp_t       xyz_frac[3];    /* XREG,XREGLP.. after DECEND +-N/10^5  */
    bool           xyz_negzero[3]; /* -00000 typed (ones' complement -0)   */
    pin_reqret_t   reqret;         /* REQRET continuation                  */
    bool           reqret_neg;     /* REQRET negative: waiting for data    */
    pin_entret_t   entret;         /* ENTRET                               */
    uint8_t        verbsave;       /* VERBSAVE (AGC keeps -verb)           */
    int16_t        clpass;         /* CLPASS: 0 pass0, + hipass, - no back */
    pin_loadstat_t loadstat;       /* LOADSTAT                             */
    bool           dsplock;        /* DSPLOCK  (1 = keyboard busy)         */
    bool           old_dsplock;    /* 21/22REG: DSPLOCK before CHARIN      */
    pin_sleeper_t  cadrstor;       /* CADRSTOR (ENDIDLE)                   */
    pin_sleeper_t  dsplist;        /* DSPLIST  (NVSUBUSY)                  */
    uint16_t       monsave;        /* MONSAVE  verb<<7 | noun, +0 = off    */
    uint16_t       monsave1;       /* MONSAVE1 bit15 kill, bit14 external  */
    uint16_t       monsave2;       /* MONSAVE2 NVMONOPT options            */
    uint16_t       nouncadr;       /* NOUNCADR machine address (MCTBS)     */
    agc_noun_t     noun;           /* LODNNTAB result for NOUNREG          */
    uint8_t        nntype;         /* 0 normal, 1 MCTBS, 2 channel, 3 augment */
    uint16_t       dsptem[7];      /* DSPTEM1 +0..+2, DSPTEM2 +0..+2, +3   */
    uint8_t        resp_verb;      /* MPAC at ENDIDLE wake (verb)          */
    uint8_t        resp_noun;      /* MPAC +1 at ENDIDLE wake (noun)       */
    uint16_t       mpac2;          /* MPAC +2: ECADR for internal MCTBS    */
} pin_state_t;

#define PIN_NNTYPE_NORMAL  0
#define PIN_NNTYPE_MCTBS   1 /* NNADTEM -  : specify machine address */
#define PIN_NNTYPE_CHAN    2 /* NNADTEM -1 : channel to be specified */
#define PIN_NNTYPE_AUGMENT 3 /* NNADTEM -0 : increment machine addr  */
#define PIN_NNTYPE_SPARE   4 /* NNADTEM +0 : not in use              */

pin_state_t *pin_st(void);
bool         pin_invariants(void); /* AGC_ASSERT(pin_invariants())   */

/* ---- word helpers (state.c) ------------------------------------ */
bool     pin_ones_negative(uint16_t w);
uint16_t pin_ones_mag(uint16_t w);
agc_sp_t pin_sp_from_ones(uint16_t w);
uint16_t pin_ones_from_sp(agc_sp_t v);
uint16_t pin_ones_neg(uint16_t mag);

/* ---- lamps (state.c) ------------------------------------------- */
void pin_flashon(void);  /* FLASHON  */
void pin_flashoff(void); /* FLASHOFF */
void pin_reldspon(void); /* RELDSPON */
void pin_lodsampt(void); /* LODSAMPT */

/* ---- DSPCOUNT and DSPTAB (state.c) ----------------------------- */
void         pin_set_dspcount(int16_t n);    /* TS DSPCOUNT (positive)       */
void         pin_block_dspcount(int16_t n);  /* CS n; TS DSPCOUNT            */
int          pin_getinrel(void);             /* GETINREL                     */
int          pin_inrel_of(int16_t dspcount); /* INRELTAB lookup              */
agc_status_t pin_dspin(int16_t count, uint16_t code); /* DSPIN            */
agc_status_t pin_11dspin(int rel, bool on);           /* 11DSPIN          */
agc_status_t pin_2blank(void);                        /* 2BLANK           */
agc_status_t pin_5blank(void);                        /* 5BLANK           */
agc_status_t pin_5blank_rel(int inrel);               /* 5BLANK +2        */
agc_status_t pin_dsptab_store(int rel, uint16_t mag); /* -mag, NOUT bump  */

/* ---- word display (display.c) ---------------------------------- */
agc_status_t pin_plus_on(void);                      /* +ON              */
agc_status_t pin_minus_on(void);                     /* -ON              */
agc_status_t pin_dspdecwd(agc_dp_t v, bool negzero); /* DSPDECWD         */
agc_status_t pin_dspdecnr(agc_dp_t v, bool negzero); /* DSPDECNR         */
agc_status_t pin_dspdc2nr(agc_dp_t v, bool negzero); /* DSPDC2NR         */
agc_status_t pin_dsp2dec(agc_dp_t v);                /* DSP2DEC          */
agc_status_t pin_dspdecvn(int n);                    /* DSPDECVN         */
agc_status_t pin_dspoctwo(uint16_t w);               /* DSPOCTWO         */
agc_status_t pin_dsp2bit(uint16_t w);                /* DSP2BIT          */
agc_status_t pin_updatvb(uint8_t verb);              /* UPDATVB -1       */
agc_status_t pin_updatnn(uint8_t noun);              /* UPDATNN -1       */

/* ---- keyboard (keys.c) ----------------------------------------- */
agc_status_t pin_charin(uint8_t code); /* CHARIN           */
agc_status_t pin_enter(void);          /* ENTER            */
agc_status_t pin_vbreldsp(void);       /* VBRELDSP         */
agc_status_t pin_tstlts4(void);        /* TSTLTS4          */
agc_status_t pin_error_key(void);      /* ERROR            */

/* ---- verb execution (verbs.c) ---------------------------------- */
agc_status_t pin_entpas0(void);         /* ENTPAS0          */
agc_status_t pin_testnn(void);          /* TESTNN           */
agc_status_t pin_verbfan(void);         /* VERBFAN          */
agc_status_t pin_usead(uint16_t ecadr); /* USEADD           */
agc_status_t pin_entexit(void);         /* TC ENTEXIT       */
agc_status_t pin_dspalarm(void);        /* DSPALARM         */
agc_status_t pin_predspal(void);        /* PREDSPAL         */
agc_status_t pin_charalrm(void);        /* CHARALRM         */
agc_status_t pin_reqadd_resume(void);   /* REQADD after Z   */
agc_status_t pin_almcycle(void);        /* ALMCYCLE         */
agc_status_t pin_noun_load(void);       /* LODNNTAB         */
/* The component tests return true when the AGC routine would return to
 * its caller; on false the caller takes the alarm exit itself
 * (COMPTEST, DECTEST, NOUNTEST: DSPALARM; DCTSTCYC: ALMCYCLE).          */
bool pin_comptest(int verb_comp); /* COMPTEST         */
bool pin_dectest(void);           /* DECTEST/DCTSTCYC */
bool pin_nountest(void);          /* NOUNTEST         */

/* ---- monitor, test lights, requests, proceed (monitor.c) ------- */
agc_status_t pin_monitor(void);                    /* MONITOR          */
agc_status_t pin_pastevb(void);                    /* PASTEVB          */
agc_status_t pin_vbproc_common(pin_loadstat_t ls); /* VBPROC +1        */
agc_status_t pin_vbtstlts(void);                   /* VBTSTLTS         */
agc_status_t pin_mmchang_resume(void);             /* MMCHANG +1       */
agc_status_t pin_reqmm(void);                      /* REQMM            */
agc_status_t pin_vbrqexec(void);                   /* VBRQEXEC (V30)   */
agc_status_t pin_vbrqwait(void);                   /* VBRQWAIT (V31)   */
agc_status_t pin_dspfmem(void);                    /* DSPFMEM  (V27)   */

/* ---- loads (load.c) -------------------------------------------- */
agc_status_t pin_load_verb(uint8_t verb);              /* ALOAD..ABCLOAD   */
agc_status_t pin_load_resume(pin_reqret_t step);       /* TC REQRET        */
agc_status_t pin_reqdat(int inrel, pin_reqret_t step); /* REQDATX/Y/Z      */
pin_reqret_t pin_reqret_backup(pin_reqret_t step);     /* REQRET +3        */

/* ---- scaling (scale.c) ----------------------------------------- */
agc_status_t pin_comp_scale(int k, agc_scale_info_t *info);
uint8_t      pin_comp_routine(int k);           /* SFRUTNOR/SFRUTMIX */
bool         pin_sf_is_dp(uint8_t rout);        /* DPTEST           */
agc_status_t pin_comp_read(int k, agc_dp_t *v); /* noun component   */
agc_status_t pin_comp_write(int k, agc_dp_t v);
agc_status_t pin_comp_octal(int k, uint16_t *word); /* word for V01-V05 */
agc_status_t pin_decdsp(void);                      /* DECDSP (V06)     */
agc_status_t pin_dspdpdec(void);                    /* DSPDPDEC (V07)   */
/* PUTDECSF: the value to store for component k from the decimal fraction
 * entered; *exited when an alarm path ended the load instead.           */
agc_status_t pin_putdecsf(int k, agc_dp_t frac, bool negzero, agc_dp_t *out,
                          bool *exited);
uint16_t     pin_360_cdu(uint16_t w); /* 360-CDU (LM)     */
agc_dp_t     pin_value_of_word(const agc_scale_info_t *info, uint16_t w);
agc_status_t pin_hmsin(void); /* HMSIN            */

/* ---- NVSUB family (nvsub.c) ------------------------------------ */
agc_status_t pin_recaltst(void); /* RECALTST         */
agc_status_t pin_reldsp1(void);  /* RELDSP1          */
void         pin_kilmonon(void); /* KILMONON         */
agc_status_t pin_nvmonopt(uint16_t nvtemp, uint16_t options); /* NVMONOPT   */
bool         pin_dsp_blocked(void);       /* MONSAVE1 EMB + DSPLOCK */
bool         pin_cadrstor_full(void);     /* CCS CADRSTOR     */
bool         pin_dsplist_full(void);      /* CCS DSPLIST      */
/* WITCHONE's XCH CADRSTOR/DSPLIST: empty the slot, return its sleeper.   */
pin_sleeper_t pin_take_sleeper(bool dsplist);

/* ---- display interface (display_if.c) ---------------------------------- */
agc_status_t pin_pinbrnch(void); /* PINBRNCH         */

#endif /* PINBALL_INTERNAL_H */
