/**********************************************************************
 * FILE NAME: state.c
 *
 * PURPOSE:
 *   LM DAP erasables, their shared accessors, and the DAP parts of fresh
 *   start and restart.
 *
 * AGC SOURCE:
 *   Luminary099 ERASABLE_ASSIGNMENTS.agc pp. 105-136 (layout, scales),
 *     FRESH_START_AND_RESTART.agc pp. 211-213 (DOFSTRT1, DAP pad loads),
 *     pp. 220-221 (STARTSB1)
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   none
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ------------------------------------
 *   agc_chan_and                WAND channel
 *   agc_chan_write              WRITE channel
 *   agc_flag_clear              clear a flag bit (flags.h)
 *   agc_flagword_set            store a whole flagword (flags.h)
 *   agc_guid_state              guidance erasables (MASS, ABDELV ...)
 *   lm_dap_dpmake               sign-agreeing DP pair (arith.c)
 *   lm_dap_dpval                DP pair value (arith.c)
 *   lm_dap_set_t5adr            DXCH T5ADR (dapif.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   None.
 *
 * NOTES:
 *   Sources: ERASABLE_ASSIGNMENTS.agc pp.105-136 (layout and scales),
 *   FRESH_START_AND_RESTART.agc pp.211-213 (DOFSTRT1, DAP pad-load defaults)
 *   and pp.220-221 (STARTSB1).
 **********************************************************************/
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/guidance/guidance.h"
#include "agc/lm/dap.h"
#include "internal.h"

/* Fresh-start constants (FRESH_START_AND_RESTART.agc p.218, CONTROLLED_
 * CONSTANTS FULLAPS; octal values checked in the yaYUL listing).         */
#define FS_RCSFLAGS   ((uint16_t) 4)      /* FOUR: needles initialisation  */
#define FS_MAXDB      ((agc_sp_t) 1820)   /* MAXDB 03434: 5 deg at 45      */
#define FS_BOOLSTRT   ((uint16_t) 021312) /* BOOLSTRT: DAPBOOLS            */
#define FS_STIKSTRT   ((agc_sp_t) 13521)  /* STIKSTRT 32321: .825268       */
#define FS_RATESTRT   ((agc_sp_t) -218)   /* RATESTRT 77445                */
#define FS_FULLAPS    ((agc_sp_t) 1263)   /* FULLAPS 02357: 5050 kg B16    */
#define FS_TRAP       ((agc_sp_t) -510)   /* 77001OCT: .14 deg at 4.5 deg  */
#define FS_KAOS       ((agc_sp_t) 60)     /* 60DEC: 6 s alpha gain         */
#define FS_DKOMEGAN   ((agc_sp_t) 10)     /* TEN: 1 s omega gain (docked)  */
#define FS_DKDB       ((agc_sp_t) 128)    /* BIT8: docked DB 1.4 deg       */
#define DAPBOOLS_WORD 13                  /* DAPBOOLS = FLGWRD13           */

static lm_dap_t g_dap; /* the DAP erasables (EBANK 6), one instance */

/**********************************************************************
 * AGC: none -- the LM DAP erasables (ERASABLE_ASSIGNMENTS.agc
 * pp.105-136), one static instance.
 *
 * FUNCTION NAME: lm_dap_state
 *
 * ARGUMENTS: none
 *
 * RETURNS: lm_dap_t * -- the DAP erasables (never NULL)
 **********************************************************************/
lm_dap_t *lm_dap_state(void)
{
    return (&g_dap);
}

/**********************************************************************
 * STARTSB1 DAP part (p.220-221): done on every fresh start and restart.
 *
 * FUNCTION NAME: agc_lm_dap_restart
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_lm_dap_restart(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->rcsflags     = (uint16_t) (d->rcsflags & ~LM_RCS_ACCSET); /* DAPIDLER calls 1/ACCS */
    d->t6next[0]    = (agc_sp_t) LM_T6_POSMAX;                   /* a pending T6RUPT is made     */
    agc_chan_and(013, LM_T6_POSMAX);                             /* ineffectual        */
    d->nxt6adr      = 0;
    d->nextp        = 0;
    agc_flag_clear(AGC_FLAG_ACCOKFLG);                           /* ACCSOKAY = 0    */
    agc_status_t ts = lm_dap_set_t5adr(LM_T5_DAPIDLER);          /* T5ADR = DAPIDLER */

    return (ts);
}

/**********************************************************************
 * DOFSTRT1 DAP part (pp.211-213), then STARTSB1.
 *
 * FUNCTION NAME: agc_lm_dap_fresh_start
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_lm_dap_fresh_start(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->rcsflags                 = FS_RCSFLAGS;
    agc_guid_state()->lm.abdelv = 0; /* ABDELV: DAP initialisation     */
    d->ch5mask                  = 0;
    d->ch6mask                  = 0;
    d->pvalvest                 = 0; /* RCS failure monitor           */
    agc_chan_write(005, 0);          /* turn off RCS jets       */
    agc_chan_write(006, 0);
    d->db                       = FS_MAXDB;
    agc_flagword_set(DAPBOOLS_WORD, FS_BOOLSTRT);
    d->stiksens                 = FS_STIKSTRT;
    d->neg_ratedb               = FS_RATESTRT;
    d->hiascent                 = FS_FULLAPS;
    d->dktrap                   = FS_TRAP;
    d->lmtrap                   = FS_TRAP;
    d->dkkaosn                  = FS_KAOS;
    d->lmkaosn                  = FS_KAOS;
    d->lmomegan                 = 0; /* unity gain                     */
    d->dkomegan                 = FS_DKOMEGAN;
    d->dkdb                     = FS_DKDB;

    return (agc_lm_dap_restart());
}

/* ---- accessors of DAP erasables shared with other modules -------------- */
/**********************************************************************
 * AGC: none -- accessor of the DAP erasable CDUXD (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_cduxd
 *
 * ARGUMENTS: none
 *
 * RETURNS: uint16_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
uint16_t agc_erase_cduxd(void)
{
    return (lm_dap_state()->cduxd);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable CDUYD (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_cduyd
 *
 * ARGUMENTS: none
 *
 * RETURNS: uint16_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
uint16_t agc_erase_cduyd(void)
{
    return (lm_dap_state()->cduyd);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable CDUZD (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_cduzd
 *
 * ARGUMENTS: none
 *
 * RETURNS: uint16_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
uint16_t agc_erase_cduzd(void)
{
    return (lm_dap_state()->cduzd);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable CDUXD (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_cduxd
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         uint16_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_cduxd(uint16_t v)
{
    if (AGC_ASSERT(v <= 077777u))
    {
        lm_dap_state()->cduxd = v;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable CDUYD (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_cduyd
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         uint16_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_cduyd(uint16_t v)
{
    if (AGC_ASSERT(v <= 077777u))
    {
        lm_dap_state()->cduyd = v;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable CDUZD (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_cduzd
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         uint16_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_cduzd(uint16_t v)
{
    if (AGC_ASSERT(v <= 077777u))
    {
        lm_dap_state()->cduzd = v;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable DELCDUX (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_delcdux
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_delcdux(void)
{
    return (lm_dap_state()->delcdux);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable DELCDUY (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_delcduy
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_delcduy(void)
{
    return (lm_dap_state()->delcduy);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable DELCDUZ (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_delcduz
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_delcduz(void)
{
    return (lm_dap_state()->delcduz);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable DELCDUX (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_delcdux
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_sp_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_delcdux(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX))
    {
        lm_dap_state()->delcdux = v;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable DELCDUY (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_delcduy
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_sp_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_delcduy(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX))
    {
        lm_dap_state()->delcduy = v;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable DELCDUZ (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_delcduz
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_sp_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_delcduz(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX))
    {
        lm_dap_state()->delcduz = v;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable OMEGAPD (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_omegapd
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_omegapd(void)
{
    return (lm_dap_state()->omegapd);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable OMEGAQD (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_omegaqd
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_omegaqd(void)
{
    return (lm_dap_state()->omegaqd);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable OMEGARD (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_omegard
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_omegard(void)
{
    return (lm_dap_state()->omegard);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable OMEGAPD (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_omegapd
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_sp_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_omegapd(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX))
    {
        lm_dap_state()->omegapd = v;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable OMEGAQD (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_omegaqd
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_sp_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_omegaqd(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX))
    {
        lm_dap_state()->omegaqd = v;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable OMEGARD (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_omegard
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_sp_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_omegard(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX))
    {
        lm_dap_state()->omegard = v;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable DELPEROR (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_delperor
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_delperor(void)
{
    return (lm_dap_state()->delperor);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable DELQEROR (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_delqeror
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_delqeror(void)
{
    return (lm_dap_state()->delqeror);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable DELREROR (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_delreror
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_delreror(void)
{
    return (lm_dap_state()->delreror);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable DELPEROR (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_delperor
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_sp_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_delperor(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX))
    {
        lm_dap_state()->delperor = v;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable DELQEROR (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_delqeror
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_sp_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_delqeror(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX))
    {
        lm_dap_state()->delqeror = v;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable DELREROR (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_delreror
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_sp_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_delreror(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX))
    {
        lm_dap_state()->delreror = v;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable OMEGAP (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_omegap
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_omegap(void)
{
    return (lm_dap_state()->omegap);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable OMEGAQ (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_omegaq
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_omegaq(void)
{
    return (lm_dap_state()->omegaq);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable OMEGAR (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_omegar
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_omegar(void)
{
    return (lm_dap_state()->omegar);
}

/**********************************************************************
 * AOSQ/AOSR are DP word pairs at pi/2 (the minor word may disagree in
 * sign after DAS, as in the AGC); the accessor returns their value.
 * AGC: none (helper)
 *
 * FUNCTION NAME: agc_erase_aosq
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_dp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_dp_t agc_erase_aosq(void)
{
    const lm_dap_t *d = lm_dap_state(); /* DAP erasables */
    lm_dp_pair_t    p = {d->aosq, d->aosq_lo};

    return ((agc_dp_t) lm_dap_dpval(p));
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable AOSR (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_aosr
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_dp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_dp_t agc_erase_aosr(void)
{
    const lm_dap_t *d = lm_dap_state(); /* DAP erasables */
    lm_dp_pair_t    p = {d->aosr, d->aosr_lo};

    return ((agc_dp_t) lm_dap_dpval(p));
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable AOSQ (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_aosq
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_dp_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_aosq(agc_dp_t v)
{
    if (AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX))
    {
        lm_dp_pair_t p = lm_dap_dpmake((int32_t) v); /* AOSQ words, scaled pi/2 rad/s^2 */

        lm_dap_state()->aosq    = p.hi;
        lm_dap_state()->aosq_lo = p.lo;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable AOSR (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_aosr
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_dp_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_aosr(agc_dp_t v)
{
    if (AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX))
    {
        lm_dp_pair_t p = lm_dap_dpmake((int32_t) v); /* AOSR words, scaled pi/2 rad/s^2 */

        lm_dap_state()->aosr    = p.hi;
        lm_dap_state()->aosr_lo = p.lo;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable ALPHAQ (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_alphaq
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_alphaq(void)
{
    return (lm_dap_state()->alphaq);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable ALPHAR (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_alphar
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_alphar(void)
{
    return (lm_dap_state()->alphar);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable PERROR (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_perror
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_perror(void)
{
    return (lm_dap_state()->perror);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable QERROR (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_qerror
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_qerror(void)
{
    return (lm_dap_state()->qerror);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable RERROR (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_rerror
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_rerror(void)
{
    return (lm_dap_state()->rerror);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable 1JACC (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_1jacc
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_1jacc(void)
{
    return (lm_dap_state()->jacc[0]);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable 1JACCQ (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_1jaccq
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_1jaccq(void)
{
    return (lm_dap_state()->jacc[1]);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable 1JACCR (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_1jaccr
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_1jaccr(void)
{
    return (lm_dap_state()->jacc[2]);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable KQ (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_kq
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_kq(void)
{
    return (lm_dap_state()->kq);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable DB (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_db
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_db(void)
{
    return (lm_dap_state()->db);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable DB (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_db
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_sp_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_db(agc_sp_t v)
{
    if (AGC_ASSERT(v >= 0))
    {
        lm_dap_state()->db = v;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable STIKSENS (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_stiksens
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_stiksens(void)
{
    return (lm_dap_state()->stiksens);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable STIKSENS (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_stiksens
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_sp_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_stiksens(agc_sp_t v)
{
    if (AGC_ASSERT(v >= 0))
    {
        lm_dap_state()->stiksens = v;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable NEG_RATEDB (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_neg_ratedb
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_neg_ratedb(void)
{
    return (lm_dap_state()->neg_ratedb);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable NEG_RATEDB (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_neg_ratedb
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_sp_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_neg_ratedb(agc_sp_t v)
{
    if (AGC_ASSERT(v <= 0))
    {
        lm_dap_state()->neg_ratedb = v;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable HIASCENT (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_hiascent
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
agc_sp_t agc_erase_hiascent(void)
{
    return (lm_dap_state()->hiascent);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable HIASCENT (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_set_hiascent
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_sp_t  I    new value, at the scale of the erasable
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_hiascent(agc_sp_t v)
{
    if (AGC_ASSERT(v >= 0))
    {
        lm_dap_state()->hiascent = v;
    }
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable CH5MASK (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_ch5mask
 *
 * ARGUMENTS: none
 *
 * RETURNS: uint16_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
uint16_t agc_erase_ch5mask(void)
{
    return (lm_dap_state()->ch5mask);
}

/**********************************************************************
 * AGC: none -- accessor of the DAP erasable CH6MASK (ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_erase_ch6mask
 *
 * ARGUMENTS: none
 *
 * RETURNS: uint16_t -- the erasable, at the scale given in agc/lm/dap.h
 **********************************************************************/
uint16_t agc_erase_ch6mask(void)
{
    return (lm_dap_state()->ch6mask);
}

/**********************************************************************
 * AGC: none -- DAP running (T5ADR = PAXIS), for the sim.
 *
 * FUNCTION NAME: agc_lm_dap_running
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when T5ADR is PAXIS
 **********************************************************************/
bool agc_lm_dap_running(void)
{
    const lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    if (!AGC_ASSERT(d->t5adr <= LM_T5_PAXIS))
    {
        return (false);
    }

    return (d->t5adr == LM_T5_PAXIS);
}
