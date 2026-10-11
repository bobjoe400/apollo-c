/**********************************************************************
 * FILE NAME: radar.h
 *
 * PURPOSE:
 *   Private interface between the LM radar files of the rendezvous
 *   programs and the LM navigation files that drive them:
 *     lm/radar/moding.c      RR limits, lamps, turn-on, CDU zero,
 *                            reposition, remode, single-axis drive, RROUT
 *     lm/radar/designate.c   RRDESSM, RRDESNB, STARTDES, BEGDES, DODES
 *     lm/radar/read.c        radar read lead-ins, RADAREAD, LRPOS2, R12
 *                            lamps
 *     lm/radar/powered_flight_designate.c   R29 (powered flight)
 *     lm/radar/leadin.c      RADSAMP / DORSAMP (radar sampling test)
 *     lm/navigation          the LM rendezvous navigation erasables
 *   RADMODES is the radar flagword FLGWRD12 (owned by sys-misc,
 *   agc_erase_radmodes); its bits are named below as in
 *   FLAGWORD_ASSIGNMENTS.agc (pp.85-86).
 *
 * AGC SOURCE:
 *   Luminary099 P20-P25.agc pp. 520-563, 594-613; LM only
 *
 * REQUIRES:
 *   agc/programs/rendezvous.h (job and status types)
 *
 * GLOBAL VARIABLES:
 *   Name          Type      Scale          Description
 *   ------------  --------  -------------  -------------------------
 *   none (rr_state() and lmnav_state() return the file-static radar and
 *   navigation erasables; struct members carry their scale)
 *
 * Angles: CDU words are 15-bit counter words (two's complement, one bit
 * = 360/2^15 degrees). The AGC reads them with CA and treats them as
 * ones' complement SP numbers scaled in half revolutions (1.0 = 180
 * degrees); rr_ones() does the same. Gyro commands TANG, TANG +1 are SP
 * pulses.
 **********************************************************************/
#ifndef RENDZ_RADAR_LM_H
#define RENDZ_RADAR_LM_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/interp.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/programs/rendezvous.h"

/* ---- RADMODES (FLGWRD12) bits ---------------------------------------- */
#define RR_CDESBIT  040000u /* BIT15 CDESFLAG: continuous designate      */
#define RR_REMODBIT 020000u /* BIT14 REMODFLG: remode requested/running  */
#define RR_RCDU0BIT 010000u /* BIT13 RCDU0FLG: RR CDUs being zeroed      */
#define RR_ANTENBIT 004000u /* BIT12 ANTENFLG: RR antenna in mode 2      */
#define RR_REPOSBIT 002000u /* BIT11 REPOSMON: reposition in progress    */
#define RR_DESIGBIT 001000u /* BIT10 DESIGFLG: designate requested       */
#define RR_ALTSCBIT 000400u /* BIT9  ALTSCALE: LR altitude high scale    */
#define RR_LRVELBIT 000200u /* BIT8  LRVELFLG: LR velocity data fail     */
#define RR_RCDUFBIT 000100u /* BIT7  RCDUFAIL: set = RR CDU fail NOT seen */
#define RR_LRPOSBIT 000040u /* BIT6  LRPOSFLG: LR position 2 desired     */
#define RR_LRALTBIT 000020u /* BIT5  LRALTFLG: LR altitude data fail     */
#define RR_RRDATABT 000010u /* BIT4  RRDATAFL: RR data fail              */
#define RR_RRRSBIT  000004u /* BIT3  RRRSFLAG: RR range high scale       */
#define RR_AUTOMBIT 000002u /* BIT2  AUTOMODE: RR NOT in auto (LGC) mode */
#define RR_TURNONBT 000001u /* BIT1  TURNONFL: RR turn-on in progress    */

/* ---- channel bits used by the radar routines -------------------------- */
#define RR_CH12_ZERO_CDU 000001u /* chan 12 bit 1: zero RR CDUs         */
#define RR_CH12_ENB_ECTR 000002u /* chan 12 bit 2: enable RR error ctrs */
#define RR_CH12_LRPOS2   010000u /* chan 12 bit 13: LR to position 2    */
#define RR_CH12_TRACK    020000u /* chan 12 bit 14: RR auto track enable */
#define RR_CH14_DRIVE_TS 006000u /* chan 14 bits 11,12 (PRIO6): drive S, T */
#define RR_CH33_RR_DGOOD 000010u /* chan 33 bit 4: RR data good (0=good) */
#define RR_CH33_LR_POS2  000100u /* chan 33 bit 7: LR in position 2      */

#define RR_WORD_MASK 077777u /* 15-bit word */

/* RR1AXIS clients: the routine that called RRTONLY/RRSONLY, whose next
 * step runs when the single-axis drive ends (the AGC kept the return
 * address in RRRET).                                                    */
typedef enum rr_axis_client
{
    RR_AXIS_REPOSITION = 0,      /* DORREPOS                                  */
    RR_AXIS_REMODE,              /* REMODE                                    */
    RR_AXIS_PREPOSITION_POWERED, /* PREPOS29                                  */
    RR_AXIS_CLIENT_COUNT
} rr_axis_client_t;

/* Designate erasables (P20-P25, EBANK= LOSCOUNT).                        */
typedef struct rr_state
{
    agc_vec_t rrtarget;   /* RRTARGET: LOS half unit, SM (NB when RRNBSW), scale 2^1 (R29: LOSSM, the non-unit LOS in SM axes, m 2^24) */
    agc_dp_t  mlosv;      /* MLOSV: |LOS|, m 2^20 in P20 (LPS20.1 shifts the LOS 9 places) */
    agc_vec_t losvel;     /* LOSVEL: relative velocity V(CSM) - V(LM) in SM axes, m/cs 2^7 (R29: LOSVDT/4, m 2^24) */
    agc_sp_t  loscount;   /* LOSCOUNT: passes to a new LOS, unscaled */
    agc_sp_t  descount;   /* DESCOUNT: designate passes left, unscaled    */
    uint16_t  tangnb[2];  /* TANGNB: RRDESNB trunnion, shaft CDU words     */
    uint16_t  modea[2];   /* MODEA: (T, S) CDU words for the present mode  */
    uint16_t  modeb[2];   /* MODEB: (T, S) for the other mode              */
    agc_sp_t  lastcmd[2]; /* LASTYCMD (T), LASTXCMD (S), pulses unscaled  */
} rr_state_t;

rr_state_t *rr_state(void);

/* ---- word helpers (lm/radar/moding.c) ----------------------------------- */
int      rr_ones(uint16_t w); /* 15-bit word as ones' complement  */
uint16_t rr_word(int v);      /* ones' complement word of v       */

/* MAGSUB: true ("L+3") when |a| <= -k, k a negative constant.           */
bool rr_magsub(int a, int k);
void rr_chan_clear(int chan, uint16_t bits);
void rr_radmodes_set(uint16_t bits);
void rr_radmodes_clear(uint16_t bits);
bool rr_radmodes_test(uint16_t bits); /* any of bits set           */
bool rr_mode2(void);                  /* ANTENBIT                  */
void rr_mode_invert(void);            /* RMODINV                   */

/* ---- moding routines (lm/radar/moding.c) -------------------------------- */
/* RRLIMCHK on (T, S) words, the present mode's limits.                  */
void rr_setrrecr(void); /* SETRRECR      */
void rr_rrout(void);    /* RROUT (TANG)  */

/* RRTONLY / RRSONLY for a client and its step; runs RR1AX2 now.         */
agc_status_t rr_axis_start(rr_axis_client_t client, int step, int rrindex, uint16_t rdes);
agc_status_t rr_remode_start(void); /* REMODE        */
agc_status_t rr_endradar(void);     /* ENDRADAR      */
agc_status_t rr_badend(void);       /* RDBADEND      */
agc_status_t rr_radnoop(void);      /* RADNOOP       */

/* Turn the lamp `lamp` (DSPTAB +11D bit) on or off (TRKFLON +1).       */
void rr_lamp_put(uint16_t lamp, bool on);

/* ---- designate (lm/radar/designate.c) ----------------------------------- */
agc_status_t rr_stdesig(void); /* STDESIG (task body)           */

/* ---- radar read (lm/radar/read.c) ------------------------------------ */
agc_dp_t rr_timehold(void); /* TIMEHOLD: reading mid time    */

/* RADLITES: lamps after a read; ITEMP1 = the data-good bit (BIT4 RR,
 * BIT5 LR altitude, BIT8 LR velocity).                                  */
void rr_radlites(uint16_t itemp1);

/* ---- LM rendezvous navigation erasables (P20-P25) ---------------------- */
typedef struct lmnav_state
{
    /* LPS20.1 */
    agc_vec_t lmpos; /* LMPOS: LM position at TDEC1, m 2^29           */
    agc_vec_t lmvel; /* LMVEL: LM velocity, m/cs 2^7                  */

    /* LRS22.1 mark data */
    agc_dp_t  rm;          /* RM: RR range, m 2^29                          */
    agc_dp_t  rdotm;       /* RDOTM: RR range rate, m/cs 2^7                */
    agc_dp_t  rrtrun;      /* RRTRUN: RR trunnion, revolutions 2^0           */
    agc_dp_t  rrshaft;     /* RRSHAFT: RR shaft, revolutions 2^0             */
    agc_dp_t  mktime;      /* MKTIME: mark time (range-rate read), cs 2^28   */
    agc_dp_t  cdutime;     /* 20D: time of the CDU readings, cs 2^28        */
    uint16_t  aig;         /* AIG: IMU inner gimbal CDU word at the mark     */
    uint16_t  amg;         /* AMG: IMU middle gimbal CDU word                */
    uint16_t  aog;         /* AOG: IMU outer gimbal CDU word                 */
    uint16_t  tang[2];     /* TANG: RR CDU words at the mark                */
    agc_dp_t  rdotmsav;    /* RDOTMSAV: raw range-rate count, unscaled      */
    agc_vec_t rrborsit;    /* RRBORSIT: RR boresight, NB, half unit 2^1     */
    agc_dp_t  dtheta;      /* DSPTEM1: radar vs SV LOS, rev 2^0 (N05)       */
    uint16_t  rangrdot[2]; /* RANGRDOT: raw range, range rate (downlink)    */

    /* R22 */
    agc_dp_t r22disp[2]; /* R22DISP: |dR| m 2^29, |dV| m/cs 2^7 (noun 49) */
    agc_sp_t n49flag;    /* N49FLAG: 0 open, -1 PRO, + V32, unscaled  */
    agc_sp_t whchread;   /* WHCHREAD: 1 R, 2 RDOT, 3 S, 4 T, unscaled     */
    agc_sp_t r65cntr;    /* R65CNTR: R65 passes left, unscaled            */

    /* R21 */
    agc_sp_t reposcnt; /* REPOSCNT: 10 s retries left, unscaled         */
    agc_dp_t repostm;  /* REPOSTM: time of the LOS, cs 2^28             */

    /* R24 */
    agc_sp_t  nsrchpnt; /* NSRCHPNT: search pattern point 0..6, unscaled */
    agc_sp_t  datagood; /* DATAGOOD: 11111 lock-on (N80), unscaled  */
    agc_dp_t  omegad;   /* OMEGDISP: boresight to +Z, rev 2^0 (N80)      */
    agc_vec_t losdesrd; /* LOSDESRD: desired LOS, reference, 2^1         */
    agc_vec_t vxrcm;    /* VXRCM: unit(V(CSM) x R(CSM)), 2^1              */
    agc_vec_t uxvect;   /* UXVECT: search pattern X axis, 2^1            */
    agc_vec_t uyvect;   /* UYVECT: search pattern Y axis, 2^1            */
    agc_vec_t savlemv;  /* SAVLEMV: V(CSM) - V(LM), SM, m/cs 2^7         */

    /* R61 / R65 */
    agc_job_id_t genret; /* GENRET: where R61LEM / R65LEM return          */

    /* P22 surface: assumed CSM orbit change (ORBCHG3)                     */
    agc_dp_t lnchtm; /* LNCHTM: estimated launch time, cs 2^28        */
} lmnav_state_t;

lmnav_state_t *lmnav_state(void);

/* LPS20.1: LM and CSM to tdec1 (conic), RRTARGET (SM half unit), MLOSV,
 * LOSVEL; *too_far when the range exceeds the 9-place shift (526ALARM).
 * BUSY: INTSTALL put the job to sleep; it resumes as `resume`.          */
agc_status_t lmnav_los(agc_interp_t *ip, agc_dp_t tdec1, agc_job_id_t resume, bool *too_far);
bool         lmnav_range_ok(void); /* LPS20.2        */

/* R61LEM (r61 = true) / R65LEM: preferred tracking attitude; the job
 * continues as ret_job when done.                                       */
agc_status_t lmnav_tracking_attitude(bool r61, agc_job_id_t ret_job);

/* P20LEMWT (+1 with the delay): wait, then R22 or the designate again.  */
agc_status_t lmnav_wait(uint16_t delay_cs);

/* R22LEM96: the noun 49 display job and the N49FLAG wait.             */
agc_status_t lmnav_residuals_display(void);

/* ASTOK after a noun 49 PROCEED (lm/navigation/measurement_update.c). */
agc_status_t lmnav_incorporate_accepted(agc_job_ctx_t *ctx);

#endif /* RENDZ_RADAR_LM_H */
