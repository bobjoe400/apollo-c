/*
 * radar_lm.h -- private interface between the LM radar files of the
 * rendezvous programs (P20-P25.agc, Luminary099 pp.520-563, 594-613):
 *   rr_moding_lm.c          RR limits, lamps, turn-on, CDU zero, reposition,
 *                           remode, single-axis drive, RROUT
 *   rr_designate_lm.c       RRDESSM, RRDESNB, STARTDES, BEGDES, DODES
 *   radar_read_lm.c         radar read lead-ins, RADAREAD, LRPOS2, R12 lamps
 *   powered_flight_designate_lm.c   R29 (RR designate in powered flight)
 *   radar_leadin_lm.c       RADSAMP / DORSAMP (radar sampling test)
 *
 * RADMODES is the radar flagword FLGWRD12 (owned by sys-misc,
 * agc_erase_radmodes); its bits are named below as in
 * FLAGWORD_ASSIGNMENTS.agc (p.85-86).
 *
 * Angles: CDU words are 15-bit counter words (two's complement, one bit =
 * 360/2^15 degrees). The AGC reads them with CA and treats them as ones'
 * complement SP numbers scaled in half revolutions (1.0 = 180 degrees);
 * rr_ones() does the same. Gyro commands TANG, TANG +1 are SP pulses.
 */
#ifndef RENDZ_RADAR_LM_H
#define RENDZ_RADAR_LM_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/core/interp.h"
#include "agc/exec/exec.h"

/* ---- RADMODES (FLGWRD12) bits ---------------------------------------- */
#define RR_CDESBIT   040000u   /* BIT15 CDESFLAG: continuous designate      */
#define RR_REMODBIT  020000u   /* BIT14 REMODFLG: remode requested/running  */
#define RR_RCDU0BIT  010000u   /* BIT13 RCDU0FLG: RR CDUs being zeroed      */
#define RR_ANTENBIT  004000u   /* BIT12 ANTENFLG: RR antenna in mode 2      */
#define RR_REPOSBIT  002000u   /* BIT11 REPOSMON: reposition in progress    */
#define RR_DESIGBIT  001000u   /* BIT10 DESIGFLG: designate requested       */
#define RR_ALTSCBIT  000400u   /* BIT9  ALTSCALE: LR altitude high scale    */
#define RR_LRVELBIT  000200u   /* BIT8  LRVELFLG: LR velocity data fail     */
#define RR_RCDUFBIT  000100u   /* BIT7  RCDUFAIL: set = RR CDU fail NOT seen */
#define RR_LRPOSBIT  000040u   /* BIT6  LRPOSFLG: LR position 2 desired     */
#define RR_LRALTBIT  000020u   /* BIT5  LRALTFLG: LR altitude data fail     */
#define RR_RRDATABT  000010u   /* BIT4  RRDATAFL: RR data fail              */
#define RR_RRRSBIT   000004u   /* BIT3  RRRSFLAG: RR range high scale       */
#define RR_AUTOMBIT  000002u   /* BIT2  AUTOMODE: RR NOT in auto (LGC) mode */
#define RR_TURNONBT  000001u   /* BIT1  TURNONFL: RR turn-on in progress    */

/* ---- channel bits used by the radar routines -------------------------- */
#define RR_CH12_ZERO_CDU   000001u   /* chan 12 bit 1: zero RR CDUs         */
#define RR_CH12_ENB_ECTR   000002u   /* chan 12 bit 2: enable RR error ctrs */
#define RR_CH12_LRPOS2     010000u   /* chan 12 bit 13: LR to position 2    */
#define RR_CH12_TRACK      020000u   /* chan 12 bit 14: RR auto track enable */
#define RR_CH14_DRIVE_TS   006000u   /* chan 14 bits 11,12 (PRIO6): drive S, T */
#define RR_CH33_RR_DGOOD   000010u   /* chan 33 bit 4: RR data good (0=good) */
#define RR_CH33_LR_POS2    000100u   /* chan 33 bit 7: LR in position 2      */

#define RR_WORD_MASK       077777u

/* RR1AXIS clients: the routine that called RRTONLY/RRSONLY, whose next
 * step runs when the single-axis drive ends (the AGC kept the return
 * address in RRRET).                                                    */
typedef enum rr_axis_client {
    RR_AXIS_REPOSITION = 0,   /* DORREPOS                                  */
    RR_AXIS_REMODE,           /* REMODE                                    */
    RR_AXIS_PREPOSITION_R29,  /* PREPOS29                                  */
    RR_AXIS_CLIENT_COUNT
} rr_axis_client_t;

/* Results of RRDESSM / RRDESNB (the AGC's return points).               */
typedef enum agc_rr_designate_result {
    AGC_RR_DESIGNATE_NO_COVERAGE = 0, /* RRDESSM L+2: LOS outside mode 2
                                       * coverage on the lunar surface     */
    AGC_RR_DESIGNATE_MANEUVER,        /* RRDESSM L+3: vehicle maneuver
                                       * required                          */
    AGC_RR_DESIGNATE_STARTED,         /* RRDESSM L+4 / RRDESNB L+2: designate
                                       * begun; the caller RADSTALLs       */
    AGC_RR_DESIGNATE_END_JOB,         /* ENDOFJOB: a job already waits in
                                       * RADSTALL, or RRDESNB rejected the
                                       * angles (alarm 502)                */
    AGC_RR_DESIGNATE_RESULT_COUNT
} agc_rr_designate_result_t;

/* Designate erasables (P20-P25, EBANK= LOSCOUNT).                        */
typedef struct rr_state {
    agc_vec_t rrtarget;   /* RRTARGET: LOS half unit, SM (NB when RRNBSW),
                           * scale 2^1 (R29: LOSSM, the non-unit LOS in SM
                           * axes, m 2^24)                                 */
    agc_dp_t  mlosv;      /* MLOSV: |LOS|, m 2^29                           */
    agc_vec_t losvel;     /* LOSVEL: LOS rate, m/cs 2^16 so that LOSVEL x
                           * MCTOMS is m 2^29 (R29: LOSVDT/4, m 2^24)     */
    agc_sp_t  loscount;   /* LOSCOUNT: DODES passes before a new LOS       */
    agc_sp_t  descount;   /* DESCOUNT: designate passes left (.5 s each)   */
    agc_sp_t  tang[2];    /* TANG, TANG +1: trunnion, shaft gyro command,
                           * pulses (RRDESNB input: desired CDU words)     */
    uint16_t  tangnb[2];  /* TANGNB: RRDESNB trunnion, shaft CDU words     */
    uint16_t  modea[2];   /* MODEA: (T, S) CDU words for the present mode  */
    uint16_t  modeb[2];   /* MODEB: (T, S) for the other mode              */
    agc_sp_t  lastcmd[2]; /* LASTYCMD (trunnion), LASTXCMD (shaft)         */
} rr_state_t;

rr_state_t *rr_state(void);

/* ---- word helpers (rr_moding_lm.c) ----------------------------------- */
int      rr_ones(uint16_t w);          /* 15-bit word as ones' complement  */
uint16_t rr_word(int v);               /* ones' complement word of v       */
/* MAGSUB: true ("L+3") when |a| <= -k, k a negative constant.           */
bool     rr_magsub(int a, int k);
void     rr_chan_or(int chan, uint16_t bits);
void     rr_chan_clear(int chan, uint16_t bits);
void     rr_radmodes_set(uint16_t bits);
void     rr_radmodes_clear(uint16_t bits);
bool     rr_radmodes_test(uint16_t bits);   /* any of bits set           */
bool     rr_mode2(void);                    /* ANTENBIT                  */
void     rr_mode_invert(void);              /* RMODINV                   */

/* ---- moding routines (rr_moding_lm.c) -------------------------------- */
/* RRLIMCHK on (T, S) words, the present mode's limits.                  */
bool         rr_limits_ok(uint16_t t, uint16_t s);
void         rr_setrrecr(void);                         /* SETRRECR      */
void         rr_rrout(void);                            /* RROUT (TANG)  */
/* RRTONLY / RRSONLY for a client and its step; runs RR1AX2 now.         */
agc_status_t rr_axis_start(rr_axis_client_t client, int step, int rrindex,
                           uint16_t rdes);
agc_status_t rr_remode_start(void);                     /* REMODE        */
agc_status_t rr_endradar(void);                         /* ENDRADAR      */
agc_status_t rr_goodend(void);                          /* RGOODEND      */
agc_status_t rr_badend(void);                           /* RDBADEND      */
agc_status_t rr_radnoop(void);                          /* RADNOOP       */
/* Turn the lamp `lamp` (DSPTAB +11D bit) on or off (TRKFLON +1).       */
void         rr_lamp_put(uint16_t lamp, bool on);

/* ---- designate (rr_designate_lm.c) ----------------------------------- */
agc_status_t rr_begdes(void);           /* BEGDES (task body)            */
agc_status_t rr_stdesig(void);          /* STDESIG (task body)           */

/* ---- radar read (radar_read_lm.c) ------------------------------------ */
agc_dp_t     rr_timehold(void);         /* TIMEHOLD: reading mid time    */
/* RADLITES: lamps after a read; ITEMP1 = the data-good bit (BIT4 RR,
 * BIT5 LR altitude, BIT8 LR velocity).                                  */
void         rr_radlites(uint16_t itemp1);

#endif /* RENDZ_RADAR_LM_H */
