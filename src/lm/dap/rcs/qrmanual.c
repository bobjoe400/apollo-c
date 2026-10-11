/*
 * qrmanual.c -- Q,R-axis manual modes (Q_R-AXIS_RCS_AUTOPILOT.agc,
 * Luminary 099 pp.1444-1449): minimum impulse (QRCONTRL, FIREQR, MINQR)
 * and rate command (CHEKSTIK, RHCACTIV, ENTERQR, ENTERUV, QRTIME).
 *
 * Q,R manual control law (p.1446): WC = A (B + |D|) D, with WC the
 * commanded rate (at 45 deg/s), A the quadratic sensitivity STIKSENS,
 * B = LINRAT the linear/quadratic sensitivity and D the hand-controller
 * deflection in counts. Q and R are coupled through U,V control and must
 * use the same law (direct rate or pseudo-auto).
 */
#include "../internal.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/lm/dap.h"

#define QM_14MS   ((agc_sp_t) 22) /* 14MS = +TJMINT6                    */
#define QM_OCT63  0x0033u         /* OCT63: channel 31 bits 1,2,5,6     */
#define QM_DETENT 0x4000u         /* BIT15 of CH31TEMP = 0: out of detent */
#define QM_LINRAT 46              /* LINRAT DEC 46                      */
#define QM_40CYCL ((agc_sp_t) 40) /* 40CYCL OCT 50: 4 s                 */

/**********************************************************************
 * Minimum impulse (pp.1444-1446): one 14 ms U,V pulse pair per RHC
 * deflection (channel 31 bits 1,2 = +-pitch Q, 5,6 = +-roll R).
 *
 * FUNCTION NAME: lm_dap_qrminimp
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_qrminimp(void)
{
    lm_dap_t *d = lm_dap_state();

    agc_inhint();
    agc_lm_dap_zatteror();
    d->qerror = 0; /* displays       */
    d->rerror = 0;
    agc_relint();
    uint16_t t31 = agc_chan_read(031); /* TEMP31         */

    if (d->oldqrmin > 0)
    { /* CHECKIN        */
        d->oldqrmin = (agc_sp_t) ((uint16_t) ~t31 & QM_OCT63);

        return (lm_dap_xtrans());
    }

    agc_sp_t u;
    agc_sp_t v;

    if ((t31 & 0x0001u) == 0u)
    { /* +QMIN          */
        u = QM_14MS;
        v = (agc_sp_t) -QM_14MS;
    }
    else if ((t31 & 0x0002u) == 0u)
    { /* -QMIN          */
        u = (agc_sp_t) -QM_14MS;
        v = QM_14MS;
    }
    else if ((t31 & 0x0010u) == 0u)
    { /* +RMIN          */
        u = QM_14MS;
        v = QM_14MS;
    }
    else if ((t31 & 0x0020u) == 0u)
    { /* -RMIN          */
        u = (agc_sp_t) -QM_14MS;
        v = (agc_sp_t) -QM_14MS;
    }
    else
    {
        return (lm_dap_xtrans());
    }

    d->tj[1]    = u; /* MINQR          */
    d->tj[2]    = v;
    d->oldqrmin = 1;

    if (!AGC_ASSERT(d->oldqrmin == 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (lm_dap_uvloop(LM_UV_MINRTN, 1));
}

/**********************************************************************
 * Stick scaling (RHCACTIV, p.1446): commanded rate at 45 deg/s from
 * counts c: STIKSENS x low word of ((4 (|c| - 1) + LINRAT) c).
 *
 * FUNCTION NAME: stick_rate
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   c         agc_sp_t  I    hand-controller counts
 *   stiksens  agc_sp_t  I    STIKSENS, quadratic sensitivity, scaled 1
 *
 * RETURNS: agc_sp_t -- commanded rate, scaled 45 deg/s
 **********************************************************************/
static agc_sp_t stick_rate(agc_sp_t c, agc_sp_t stiksens)
{
    if (!AGC_ASSERT(c >= LM_NEGMAX && c <= LM_POSMAX))
    {
        return (0); /* no valid result */
    }

    agc_sp_t k   = lm_dap_sat(4 * (int32_t) lm_dap_ccs_dim(c) + QM_LINRAT);
    agc_sp_t lin = lm_dap_mp(k, c).lo; /* CA L           */
    agc_sp_t r   = lm_dap_mphi(lin, stiksens);

    if (!AGC_ASSERT(r >= LM_NEGMAX && r <= LM_POSMAX))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/**********************************************************************
 * Breakout test (ENTERQR): |change| - 1 + (-RATEDB) > 0.
 * AGC: none (helper)
 *
 * FUNCTION NAME: beyond
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   x         agc_sp_t  I    command or rate change, scaled 45 deg/s
 *   negdb     agc_sp_t  I    -RATEDB (TARGETDB), scaled 45 deg/s, <= 0
 *
 * RETURNS: bool -- true beyond the breakout level
 **********************************************************************/
static bool beyond(agc_sp_t x, agc_sp_t negdb)
{
    if (!AGC_ASSERT(negdb <= 0))
    {
        return (false);
    }

    if (!AGC_ASSERT(x >= LM_NEGMAX && x <= LM_POSMAX))
    {
        return (false);
    }

    return ((int32_t) lm_dap_ccs_dim(x) + negdb > 0);
}

/**********************************************************************
 * ENTERUV / VDB / QRTIME / TOPSEUDO (pp.1448-1449): direct rate. Zero the
 * attitude errors; an axis already inside the target deadband gets no
 * rate command; stay in direct rate (QRBIT) while TCQR runs, unless both
 * are inside (fire once, then pseudo-auto). Then the BACKHAND loop.
 *
 * FUNCTION NAME: enteruv
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
static agc_status_t enteruv(lm_dap_t *d)
{
    agc_inhint();
    agc_lm_dap_zatteror();
    agc_relint();
    d->qerror     = 0; /* DYERROR        */
    d->dyerror_lo = 0;
    d->rerror     = 0; /* DZERROR        */
    d->dzerror_lo = 0;
    bool uout = beyond(d->omegau, d->neg_ratedb); /* TARGETDB       */
    bool vout = beyond(d->omegav, d->neg_ratedb);
    bool direct;

    if (uout)
    {
        if (!vout)
        {
            d->omegav = 0; /* VRATEDIF = 0   */
        }

        direct = d->tcqr > 0; /* QRTIME         */
    }
    else if (vout)
    {
        d->omegau = 0; /* URATEDIF = 0   */
        direct    = d->tcqr > 0;
    }
    else
    {
        direct = false; /* TOPSEUDO       */
    }

    if (direct)
    {
        d->rcsflags = (uint16_t) (d->rcsflags | LM_RCS_QRBIT);
    }
    else
    {
        d->rcsflags = (uint16_t) (d->rcsflags & ~LM_RCS_QRBIT);
    }

    return (lm_dap_uvloop(LM_UV_BACKHAND, 1));
}

/**********************************************************************
 * RHCACTIV / ENTERQR (pp.1446-1447): commanded Q,R rates from SAVEHAND,
 * rate differences into U,V; a command change beyond the breakout level
 * starts direct rate (TCQR = 4 s); otherwise direct rate continues
 * (QRBIT) or pseudo-auto (STILLRCS) runs.
 *
 * FUNCTION NAME: rhcactiv
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
static agc_status_t rhcactiv(lm_dap_t *d)
{
    agc_sp_t q  = stick_rate(d->savehand[0], d->stiksens);
    agc_sp_t dq = lm_dap_wrap((int32_t) q - d->qlast); /* DAPTEMP3       */

    d->qlast = q;
    agc_sp_t r  = stick_rate(d->savehand[1], d->stiksens);
    agc_sp_t dr = lm_dap_wrap((int32_t) r - d->rlast); /* DAPTEMP4       */

    d->rlast = r;
    /* TS QRATEDIF / RRATEDIF: an overflow (> 45 deg/s difference) would
     * make the AGC skip an instruction; the stored wrapped value is used. */
    d->edotq = lm_dap_wrap((int32_t) d->omegaq - d->qlast);
    d->edotr = lm_dap_wrap((int32_t) d->omegar - d->rlast);
    lm_dap_rot_touv(d->edotq, d->edotr, &d->omegau, &d->omegav); /* URATEDIF */

    if (beyond(dq, d->neg_ratedb) || beyond(dr, d->neg_ratedb))
    {
        d->tcqr = QM_40CYCL;

        return (enteruv(d));
    }

    if ((d->rcsflags & LM_RCS_QRBIT) != 0u)
    {
        return (enteruv(d));
    }

    return (lm_dap_stillrcs());
}

/**********************************************************************
 * CHEKSTIK / DAMPING (p.1446): rate command. Not in GTS while in
 * attitude hold (COTROLER = -1: RCS next pass; 1/ACCS does the nulling
 * drives). In detent after manual control: damp to zero command rates.
 *
 * FUNCTION NAME: lm_dap_chekstik
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_chekstik(void)
{
    lm_dap_t *d = lm_dap_state();

    d->ingts    = 0;
    d->cotroler = -1;

    if ((d->ch31temp & QM_DETENT) == 0u)
    { /* out of detent  */
        return (rhcactiv(d));
    }

    if (!agc_flag_test(AGC_FLAG_OURRCFLG))
    {
        return (lm_dap_stillrcs());
    }

    d->rcsflags    = (uint16_t) (d->rcsflags & ~LM_RCS_JUSTIN);
    d->savehand[0] = 0; /* DAMPING        */
    d->savehand[1] = 0;

    if (!AGC_ASSERT(d->cotroler < 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rhcactiv(d));
}
