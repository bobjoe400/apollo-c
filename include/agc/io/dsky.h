/**********************************************************************
 * FILE NAME: dsky.h
 *
 * PURPOSE:
 *   DSKY / PINBALL interface: keyboard and uplink entries, DSPOUT, the
 *   internal program interface (NVSUB, ENDIDLE, ...), the display
 *   interface (agc_dsky_display), pinball's jobs and tasks, and the sim
 *   readout of the DSKY.
 *
 * AGC SOURCE:
 *   PINBALL_GAME_BUTTONS_AND_LIGHTS.agc (CM pp. 307-389, LM pp. 390-471),
 *   DISPLAY_INTERFACE_ROUTINES.agc (CM pp. 1452-1484, LM pp. 1341-1373),
 *   KEYRUPT_UPRUPT.agc (CM pp. 1449-1451, LM pp. 1338-1340). The noun
 *   tables are in nouns.h.
 *
 * REQUIRES:
 *   none (self-contained; includes what it uses)
 *
 * GLOBAL VARIABLES:
 *   none (pinball erasables are private to src/io/dsky)
 *
 * NOTES:
 *   "THE LANGUAGE OF COMMUNICATION WITH THE PROGRAM IS A PAIR OF WORDS
 *    KNOWN AS VERB AND NOUN."
 *
 *   Panel layout and DSPCOUNT numbers (all octal, from the AGC listing):
 *       MD1 25  MD2 24                       (major mode = PROG)
 *       VD1 23  VD2 22      ND1 21  ND2 20   (VERB, NOUN)
 *       R1D1 16 R1D2 15 R1D3 14 R1D4 13 R1D5 12
 *       R2D1 11 R2D2 10 R2D3  7 R2D4  6 R2D5  5
 *       R3D1  4 R3D2  3 R3D3  2 R3D4  1 R3D5  0
 *
 *   DSPTAB (11 words) feeds output channel 010 as AAAABCCCCCDDDDD: AAAA
 *   relay word = DSPTAB index + 1, B sign relay, CCCCC left char, DDDDD
 *   right char. A DSPTAB word that is negative (bit 15 set, ones'
 *   complement) still has to be sent by DSPOUT.
 **********************************************************************/
#ifndef AGC_DSKY_H
#define AGC_DSKY_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"

/* ---- key codes, channel 15 / 16 (5 bits), PINBALL p. 313 ------------- */
enum agc_dsky_key
{
    AGC_DSKY_KEY_1      = 001,
    AGC_DSKY_KEY_2      = 002,
    AGC_DSKY_KEY_3      = 003,
    AGC_DSKY_KEY_4      = 004,
    AGC_DSKY_KEY_5      = 005,
    AGC_DSKY_KEY_6      = 006,
    AGC_DSKY_KEY_7      = 007,
    AGC_DSKY_KEY_8      = 010,
    AGC_DSKY_KEY_9      = 011,
    AGC_DSKY_KEY_0      = 020,
    AGC_DSKY_KEY_VERB   = 021,
    AGC_DSKY_KEY_RSET   = 022, /* ERROR LIGHT RESET                    */
    AGC_DSKY_KEY_KEYREL = 031, /* KEY RELEASE                          */
    AGC_DSKY_KEY_PLUS   = 032,
    AGC_DSKY_KEY_MINUS  = 033,
    AGC_DSKY_KEY_ENTR   = 034,
    AGC_DSKY_KEY_CLR    = 036,
    AGC_DSKY_KEY_NOUN   = 037
};

/* PRO is not a key code: it is channel 32 bit 14, sampled by T4RUPT,
 * which calls agc_dsky_prockey_request() (PROCKEY).                     */

/* ---- 5-bit relay codes (PINBALL p. 314) ------------------------------ */
#define AGC_DSKY_RELAY_BLANK 000 /* relay code of a blank char */

/* digit d -> agc_dsky_relay_code(d): 0 25, 1 03, 2 31, 3 33, 4 17,
 *                                    5 36, 6 34, 7 23, 8 35, 9 37      */
uint16_t agc_dsky_relay_code(int digit);
int      agc_dsky_relay_digit(uint16_t code); /* -1 for blank/bad  */

/* ---- channel 011 lamp bits used by PINBALL ---------------------------- */
#define AGC_DSKY_LAMP_UPACT    0004  /* bit 3 uplink activity            */
#define AGC_DSKY_LAMP_TEMP     0010  /* bit 4 temp caution               */
#define AGC_DSKY_LAMP_KEYREL   0020  /* bit 5 key release                */
#define AGC_DSKY_LAMP_FLASH    0040  /* bit 6 verb/noun flash            */
#define AGC_DSKY_LAMP_OPRERR   0100  /* bit 7 operator error             */
#define AGC_DSKY_CAUTION_RESET 01000 /* bit 10 caution reset             */

/* ---- display state for the sim harness ------------------------------- */
#define AGC_DSKY_BLANK   (-1) /* digit value of a blank position   */
#define AGC_DSKY_RDIGITS 5    /* digits per register */

typedef struct agc_dsky_state
{
    int8_t   prog[2];                /* MD1 MD2 (or AGC_DSKY_BLANK)       */
    int8_t   verb[2];                /* VD1 VD2                           */
    int8_t   noun[2];                /* ND1 ND2                           */
    int8_t   r[3][AGC_DSKY_RDIGITS]; /* R1..R3, D1..D5                    */
    int8_t   r_sign[3];              /* +1 plus lit, -1 minus lit, 0 none */
    uint16_t lamps;                  /* channel 011 image                 */
    bool     flash;                  /* V/N flash (bit 6)                 */
    bool     key_rel;                /* KEY REL lamp (bit 5)              */
    bool     opr_err;                /* OPR ERR lamp (bit 7)              */
    bool     uplink_act;             /* UPLINK ACTY lamp (bit 3)          */
    uint8_t  verbreg;                /* VERBREG (two-digit verb code)     */
    uint8_t  nounreg;                /* NOUNREG                           */
} agc_dsky_state_t;

/* ---- initialisation (part of STARTSUB, FRESH_START_AND_RESTART) ------ */
agc_status_t agc_dsky_init(void);

/* ---- keyboard / uplink ------------------------------------------------ */
/* Sim convenience: KEYRUPT1 + immediate CHARIN for one key code.       */
agc_status_t agc_dsky_key(uint8_t code);

/* Same, for a string such as "V06N36E" (V N E C R K + - and digits;
 * 'P' = PRO via PROCKEY). Stops at the first error.                    */
agc_status_t agc_dsky_keys(const char *seq);
agc_status_t agc_dsky_keyrupt1(void); /* KEYRUPT1, main DSKY ch 15  */
agc_status_t agc_dsky_keyrupt2(void); /* KEYCOM for the nav DSKY ch 16 */

/* UPRUPT, triple-redundant word */
agc_status_t agc_dsky_uprupt(uint16_t inlink);
agc_status_t agc_dsky_prockey_request(void); /* NOVAC PROCKEY (T4RUPT) */
void         agc_dsky_falton(void);          /* FALTON: OPR ERR lamp on */

/* ---- display output (DSPOUT/DSPOUTSB, T4RUPT_PROGRAM p. 133) ---------- */
/* Sends the next pending DSPTAB relay word to channel 010 and returns it
 * in *relay_word. AGC_STATUS_NOT_FOUND when nothing is pending (the
 * caller then turns the relays off as NODSPOUT does).                  */
agc_status_t agc_dsky_dspout(uint16_t *relay_word);

/* CCS NOUT / TC DSPOUTSB without the DSKYFLAG test (QUIKDSP, p. 137).  */
agc_status_t agc_dsky_dspoutsb(uint16_t *relay_word);

/* DSPTAB itself is the shared erasable agc_erase_dsptab(0..11) in
 * agc_state.h (+11D = C/S lamp relays, written by T4RUPT/IMU/alarms).  */
int16_t      agc_dsky_nout(void); /* NOUT                        */
agc_status_t agc_dsky_get_state(agc_dsky_state_t *out);

/* ---- pinball erasables other modules / noun data read ----------------- */
/* DSPTEM1 +0..+2 = index 0..2, DSPTEM2 +0..+2 = index 3..5, index 6 =
 * the word after DSPTEM2 +2 (DSPTEMX +2). Raw 15-bit AGC words.
 * DSPTEMX = OPTIONX = DSPTEM2 +1, NORMTEM1 = DSPTEM1.                   */
uint16_t     agc_dsky_dsptem(int index);
agc_status_t agc_dsky_set_dsptem(int index, uint16_t word);

/* The same buffer under the erasable names the programs use: DSPTEM1 +i
 * (i 0..2), OPTIONX +i (i 0..2, = DSPTEM2 +1 +i), DSPTEMX (DP, cs 2^28
 * for N16 or two whole numbers for LM N14). SP values as agc_sp_t.        */
void     agc_erase_set_dsptem1(int index, agc_sp_t v);
agc_sp_t agc_erase_optionx(int index);
void     agc_erase_set_optionx(int index, agc_sp_t v);
agc_dp_t agc_erase_dsptemx(void);
void     agc_erase_set_dsptemx(agc_dp_t v);

/* XREG YREG ZREG as the AGC keeps them for noun 07 (octal words).      */
uint16_t agc_dsky_xyzreg(int index);

/* Pinball resolves the noun data ids that name its own erasables
 * (DSPTEM1.., DSPTEM2.., DSPTEMX.., OPTIONX.., XREG/YREG/ZREG).
 * NOT_FOUND when the id is not one of them.                            */
agc_status_t agc_dsky_noun_read(uint16_t data_id, agc_dp_t *value);
agc_status_t agc_dsky_noun_write(uint16_t data_id, agc_dp_t value);

/* Verb and noun at the time the operator answered ENDIDLE (the AGC left
 * them in the woken job's MPAC, MPAC+1).                               */
agc_status_t agc_dsky_response(uint8_t *verb, uint8_t *noun);

/* ---- internal program interface (NVSUB etc., PINBALL pp. 369-383) ---- */
/* Return AGC_STATUS_OK = "L+2, performed", AGC_STATUS_BUSY = "L+1,
 * display system blocked", AGC_STATUS_POODOO after abort 01501.        */
agc_status_t agc_dsky_nvsub(uint8_t verb, uint8_t noun);
agc_status_t agc_dsky_nvsub_mctbs(uint8_t verb, uint8_t noun, uint16_t ecadr);
agc_status_t agc_dsky_nvmonopt(uint8_t verb, uint8_t noun, uint16_t options);

/* NVSUB with -4 full blank, -3 leave mode, -2 leave mode and verb,
 * -1 blank R's only.                                                    */
agc_status_t agc_dsky_nvsub_blank(int code);

/* NVSBWAIT: executes now (OK) or puts the job to sleep in DSPLIST and
 * resumes it as wake_as when the operator releases the DSKY (BUSY; the
 * job must return, and wake_as re-issues the call).                   */
agc_status_t agc_dsky_nvsbwait(const agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun, agc_job_id_t wake_as);

/* NVSUBUSY: sleep in DSPLIST (KEY REL on) until RELDSP wakes the job as
 * wake_as; BUSY. POODOO (abort 01206) if CADRSTOR or DSPLIST is taken. */
agc_status_t agc_dsky_nvsubusy(agc_job_id_t wake_as);
agc_status_t agc_dsky_blanksub(uint16_t code); /* bit1 R1 bit2 R2 bit3 R3 */

/* ENDIDLE: puts the calling job to sleep (it must return right after an
 * OK); the operator's answer wakes it at one continuation: V34
 * terminate, V33 proceed, data load / V32 recycle. agc_dsky_response()
 * then gives the verb and noun of the answer. POODOO (abort 01206) if a
 * job already waits in CADRSTOR or DSPLIST.                           */
agc_status_t agc_dsky_endidle(agc_job_id_t on_terminate, agc_job_id_t on_proceed, agc_job_id_t on_enter);
agc_status_t agc_dsky_jamterm(void);    /* JAMTERM (V34 by program)   */
agc_status_t agc_dsky_jamproc(void);    /* JAMPROC (V33 by program)   */
agc_status_t agc_dsky_dspmm(void);      /* DSPMM: show MODREG         */
agc_status_t agc_dsky_releasedsp(void); /* RELDSP                     */

/* LOADLV (via LOADLV1 / GOLOADLV of extended verbs 50, 51): end of a
 * load or please-perform, then RECALTST.                               */
agc_status_t agc_dsky_loadlv(void);

/* NVWORD1 (set by P30-P47): -0 pastes V97, -0400 pastes V99 on a
 * CLOCPLAY display (raw 15-bit word).                                  */
void agc_dsky_set_nvword1(uint16_t word);

/* ---- display interface routines (DISPLAY_INTERFACE_ROUTINES) --------- */
/* Astronaut returns of a flashing display (AGC "call CADR +1/+2/+3"):
 * terminate = V34, proceed = V33, enter = ENTER or V32 recycle.
 * GODSPRET / EXDSPRET continue at .terminate (call CADR +1).           */
typedef struct agc_dsky_cont
{
    agc_job_id_t terminate; /* V34: call CADR +1 */
    agc_job_id_t proceed;   /* V33: call CADR +2 */
    agc_job_id_t enter;     /* data in / V32: CADR +3 */
} agc_dsky_cont_t;

typedef enum agc_dsky_disp
{
    AGC_DSKY_GODSP = 0,
    AGC_DSKY_GODSPR,
    AGC_DSKY_GODSPRET,
    AGC_DSKY_GOFLASH,
    AGC_DSKY_GOFLASHR,
    AGC_DSKY_REGODSP,
    AGC_DSKY_REGODSPR,
    AGC_DSKY_REFLASH,
    AGC_DSKY_REFLASHR,
    AGC_DSKY_CLEANDSP,
    AGC_DSKY_GOPERF1,
    AGC_DSKY_GOPERF1R,
    AGC_DSKY_GOPERF2,
    AGC_DSKY_GOPERF2R,
    AGC_DSKY_GOPERF4,
    AGC_DSKY_GOPERF4R,
    AGC_DSKY_PRIODSP,
    AGC_DSKY_PRIODSPR,
    AGC_DSKY_GOMARK,
    AGC_DSKY_GOMARKR,
    AGC_DSKY_GOMARKF,
    AGC_DSKY_GOMARKFR,
    AGC_DSKY_GOMARK2,
    AGC_DSKY_GOMARK2R,
    AGC_DSKY_GOMARK3,
    AGC_DSKY_GOMARK3R,
    AGC_DSKY_GOMARK4,
    AGC_DSKY_EXDSPRET,
    AGC_DSKY_KLEENEX,
    AGC_DSKY_CLOCPLAY,
    AGC_DSKY_DISP_COUNT
} agc_dsky_disp_t;

/* Packs verb and noun as the AGC VN pseudo-op does: 0VVVVVVVNNNNNNN.   */
uint16_t agc_dsky_vn(uint8_t verb, uint8_t noun);

/* Requests a display. vn_or_value is the VN code, or the checklist
 * value (GOPERF1), or the option code (GOPERF4). Non-R kinds run as part
 * of the calling job, which must return right after an OK: the job has
 * then ended (GODSP), been put to sleep (flashing displays: it wakes at
 * cont->terminate/proceed/enter) or been set to continue at
 * cont->terminate (GODSPRET, EXDSPRET). R kinds return OK at once (the
 * immediate return, call CADR +4) and the display runs as job MAKEPLAY
 * at the caller's priority. cont may be NULL for non-flashing kinds.   */
agc_status_t agc_dsky_display(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind, uint16_t vn_or_value, const agc_dsky_cont_t *cont);
agc_status_t agc_dsky_blanket(uint16_t bits); /* BLANKET after an R call  */
agc_status_t agc_dsky_endmark(void);          /* ENDMARK (ENDEXT EQUALS it) */
void         agc_dsky_clearmrk(void);         /* CLEARMRK                  */
agc_status_t agc_dsky_pinbrnch(void);         /* PINBRNCH                  */
agc_status_t agc_dsky_initdsp(void);          /* INITDSP (.1 restarts)     */

/* ---- executive entry points owned by pinball (dispatch.c calls these) */
agc_status_t agc_job_charin(agc_job_ctx_t *ctx);   /* CHARIN              */
agc_status_t agc_job_prockey(void);  /* PROCKEY             */
agc_status_t agc_job_mondo(void);    /* MONDO               */
agc_status_t agc_job_dspmmjb(agc_job_ctx_t *ctx);  /* DSPMMJB             */
agc_status_t agc_job_tstlts3(void);  /* TSTLTS3             */
agc_status_t agc_job_makeplay(agc_job_ctx_t *ctx); /* MAKEPLAY (R kinds)  */
agc_status_t agc_job_playjum1(void); /* PLAYJUM1            */
agc_status_t agc_job_markplay(void); /* MARKPLAY            */
agc_status_t agc_job_redoprio(void); /* REDOPRIO            */
agc_status_t agc_job_termate(void);  /* TERMATE  (ENDIDLE+1) */
agc_status_t agc_job_proceed(void);  /* PROCEED  (ENDIDLE+2) */
agc_status_t agc_job_idleret3(agc_job_ctx_t *ctx); /* IDLERET1+2 (ENDIDLE+3) */
agc_status_t agc_job_jamterm(void);  /* JAMTERM via FINDVAC */

/* XCHSLEEP, FACEREG 0 (mark) */
agc_status_t agc_job_xchsleepm(void);

/* XCHSLEEP, FACEREG 1 (normal) */
agc_status_t agc_job_xchsleepn(void);
agc_status_t agc_task_monreq(void);  /* MONREQ  (waitlist)  */
agc_status_t agc_task_tstlts2(void); /* TSTLTS2 (waitlist)  */

#endif /* AGC_DSKY_H */
