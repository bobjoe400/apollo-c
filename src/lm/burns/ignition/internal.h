/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   The BURN, BABY, BURN ignition tables shared by burnbaby.c and
 *   clock.c.
 *
 * AGC SOURCE:
 *   Luminary099 BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc pp. 731-751
 *
 * REQUIRES:
 *   the project headers it includes (self-contained)
 *
 * GLOBAL VARIABLES:
 *   none (state is reached through accessors)
 *
 * NOTES:
 *   The AGC indexes a table of constants and TCF instructions by WHICH
 *   (P12TABLE, P40TABLE, P41TABLE -5, P42TABLE, P63TABLE, ABRTABLE). Here
 *   each table is a struct whose fields carry the AGC entry number (octal)
 *   and whose TCF targets are enumerations (no function pointers).
 **********************************************************************/
#ifndef IGNITION_INTERNAL_H
#define IGNITION_INTERNAL_H

#include "agc/core/channels.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/nav/conic.h"
#include "agc/programs/rendezvous.h"

/* (3) the *ENTER response of the V99 engine-enable request.               */
typedef enum ign_enter
{
    IGN_ENTER_GOPOST = 0, /* GOPOST: POSTBURN, coasting DAP                */
    IGN_ENTER_GOCUTOFF,   /* GOCUTOFF: CUTOFF, coasting DAP                */
    IGN_ENTER_RECYCLE     /* V99RECYC: show the V99 request again          */
} ign_enter_t;

/* (11) what TIG-5 does after IGNFLAG and ASTNFLAG are reset.              */
typedef enum ign_tig5
{
    IGN_TIG5_TIMEBURN = 0, /* P40SJUNK: start S40.13 unless it is running   */
    IGN_TIG5_DISPCHNG,     /* DISPCHNG: the V99 request from TIG-5          */
    IGN_TIG5_COMMON        /* COMMON: only the TIG-0 restart point (P41)    */
} ign_tig5_t;

/* (13) the program-specific part of IGNITION.                             */
typedef enum ign_kind
{
    IGN_KIND_NONE = 0, /* P41: no ignition (RCS)                        */
    IGN_KIND_ASCENT,   /* P12IGN: DAP bias accelerations, then ABRTIGN  */
    IGN_KIND_DPS,      /* P40IGN: throttle-up (ZOOM) unless NOTHROTL    */
    IGN_KIND_APS,      /* P42IGN                                        */
    IGN_KIND_BRAKING,  /* P63IGN: landing displays and P70/P71 set-up   */
    IGN_KIND_ABORT     /* ABRTIGN: ascent guidance on SERVICER          */
} ign_kind_t;

typedef struct ign_table
{
    uint8_t      noun;          /* (0)  VN 06xx noun (decimal, as keyed)     */
    bool         want_aps;      /* (1)  WANTAPS before ULLGNOT               */
    bool         comfail_kill;  /* (2)  COMFAIL3 (kill CLOKTASK) / COMFAIL4  */
    ign_enter_t  enter;         /* (3)                                       */
    bool         start_clock;   /* (5)  P40SPOT (STCLOK2) / P41SPOT          */
    agc_sp_t     ullage_cs;     /* (6)  ullage before TIG-0, cs; < 0 P41; unscaled */
    agc_job_id_t avgexit;       /* (7)  AVEGEXIT from TIG-30                 */
    ign_tig5_t   tig5;          /* (11)                                      */
    bool         tigtask;       /* (12) TIGTASK (P41) / WAITABIT             */
    ign_kind_t   kind;          /* (13)                                      */
    agc_job_id_t alarm_proceed; /* (14) REP40ALM PROCEED: recycle / P42STAGE */
} ign_table_t;

/* The table WHICH selects (WHICH out of range is an assertion failure and
 * gives the P40 table).                                                   */
const ign_table_t *ign_table(void);

/* DISPDEX indices (the negative of these selects a CLOKJOB display).       */
#define IGN_CNTDNDEX 017 /* LOW4: countdown V06Nxx                        */
#define IGN_VB99DEX  013 /* ELEVEN: V99 engine-enable request             */
#define IGN_VB97DEX  035 /* OCT35: V97 engine-fail (COMFAIL)              */
#define IGN_ASTNDEX  025 /* V06N61 (landing ASTNCLOK)                     */
#define IGN_BLANKDEX 2   /* TWO: DSKY blank at TIG-35                     */

/* "CA Z / TS DISPDEX": any positive DISPDEX kills CLOKTASK. The AGC stores
 * the (positive) address in Z; 1 is used here, as POSTBURN does.         */
#define IGN_DISPDEX_KILL 1 /* positive DISPDEX: kill CLOKTASK */

/* STCLOK3 (p.743-744): start CLOKTASK at the next whole second before TIG
 * (6.3) and protect the caller with a 4.2 type C restart at restart_job.
 * DISPDEX must already be set (STCLOK1/STCLOK2).                          */
agc_status_t ign_start_clock(agc_job_id_t restart_job);

#endif /* IGNITION_INTERNAL_H */
