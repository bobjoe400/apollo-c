/**********************************************************************
 * FILE NAME: qrjets.c
 *
 * PURPOSE:
 *   The U,V jet loop of the Q,R-axis RCS autopilot (Q_R-
 *   AXIS_RCS_AUTOPILOT.agc, Luminary 099 pp.1445-1458): the three
 *   "RETJADR" bodies (TJLAW +3 attitude control, BACKHAND direct rate,
 *   MINRTN minimum impulse), the jet selection AFTERTJ / DOROTAT /
 *   DOSKIP / FEEDBACK, SELCTSUB, XTRANS, ROT-TOUV and CLOSEOUT.
 *
 * AGC SOURCE:
 *   Luminary099 Q_R-AXIS_RCS_AUTOPILOT.agc pp. 1445-1458
 *     (TJLAW, BACKHAND, MINRTN, AFTERTJ, DOROTAT, DOSKIP, FEEDBACK,
 *     SELCTSUB, ROT-TOUV, CLOSEOUT)
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   DAP block     lm_dap_t  I/O  per field      EBANK 6 DAP erasables (lm_dap_state())
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ------------------------------------
 *   agc_alarm                   ALARM: record an alarm code
 *   agc_chan_read               READ channel
 *   agc_flag_test               test a flag bit (flags.h)
 *   agc_inhint                  INHINT
 *   agc_relint                  RELINT
 *   lm_dap_dpval                DP pair value (arith.c)
 *   lm_dap_jtlst                JTLST (t6rupt.c)
 *   lm_dap_mp                   MP: A,L product (arith.c)
 *   lm_dap_mphi                 A after MP (arith.c)
 *   lm_dap_ovf                  TS overflow-skip test (arith.c)
 *   lm_dap_sat                  OVERSUB / LIMITS (arith.c)
 *   lm_dap_spsrcs               SPSRCS (spsrcs.c)
 *   lm_dap_state                DAP erasables (state.c)
 *   lm_dap_tjetlaw              TJETLAW (tjetlaw.c)
 *   lm_dap_write                WRITEP/U/V (t6rupt.c)
 *   lm_dap_write_next           WRITEx -1 (t6rupt.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Alarm 02004 when no U/V rotation jets are available (the pass
 *   goes on). AGC_STATUS_BAD_ARG when an internal assertion fails.
 *
 * NOTES:
 *   The AGC runs a body for the V axis (AXISCTR = 1) and then for U
 *   (AXISCTR = 0); LOOPER returns through RETJADR. Here that is a bounded
 *   loop over the axis in lm_dap_uvloop().
 **********************************************************************/
#include <stddef.h>

#include "../internal.h"
#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/lm/dap.h"

#define QJ_150MS    240  /* -150MS = -240: 150 ms in T6 counts      */
#define QJ_TJMIN    22   /* +TJMINT6 = 14MS: 14 ms minimum firing   */
#define QJ_60MS     96   /* 60MS: docked minimum impulse            */
#define QJ_SKIPOFF  4    /* SKIPU/V = 4: not skipping               */
#define QJ_USEQRBIT 8191 /* USEQRJTS (BIT14) - 1 after CCS          */
#define QJ_NALL     9    /* ALLJETS: number of U,V jet codes */

/* INDEXES (p.1455): T6 axis index of U and V (WRITEP +4, WRITEP +13).    */
static const agc_sp_t indexes[2] = {4, 13};

/* AXISDIFF (TJET_LAW p.1469) for U, V.                                   */
static const agc_sp_t uvdiff[2] = {0, 16};

/* ALLJETS (-U -V +U +V) followed by TYPEPOLY (-X +X A B ALL), contiguous
 * as in the AGC so that X translation indexes ALLJETS with 4 and 5.     */
static const uint16_t alljets[QJ_NALL] = {00110, 00022, 00204, 00041, 00125, 00252, 00146, 00231, 00377};

/**********************************************************************
 * SELCTSUB / FAILOOP (pp.1457-1458): POLYTEMP = ALLJETS(ROTINDEX) &
 * TYPEPOLY(NUMBERT); on a failed jet retry NUMBERT = 3, 2, 1, 0. False if
 * even NUMBERT = 0 has a failed jet.
 *
 * FUNCTION NAME: policy_ok
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *
 * RETURNS: bool -- true when no jet of POLYTEMP is failed
 **********************************************************************/
static bool policy_ok(lm_dap_t *d)
{
    d->polytemp = (agc_sp_t) (alljets[d->rotindex] & alljets[4 + d->numbert]);

    return (((uint16_t) d->polytemp & d->ch5mask) == 0u);
}

/**********************************************************************
 * SELCTSUB / FAILOOP (pp.1457-1458): the U,V or X-translation
 * policy for ROTINDEX, NUMBERT with failed jets avoided.
 *
 * FUNCTION NAME: lm_dap_selctsub
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- false when every policy has a failed jet
 **********************************************************************/
bool lm_dap_selctsub(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    if (policy_ok(d))
    {
        return (true);
    }

    for (int n = 3; n >= 0; n--)
    { /* FAILOOP        */
        d->numbert = (agc_sp_t) n;

        if (policy_ok(d))
        {
            return (true);
        }
    }

    return (false);
}

/**********************************************************************
 * CLOSEOUT / ENDJASK (p.1458): the jask is over (DAPZRUPT negative).
 *
 * FUNCTION NAME: lm_dap_closeout
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_closeout(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->dapzrupt = LM_NEGMAX;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * XTRANS (p.1455): no rotation (or control inhibited): TJU = TJV = 0;
 * write the translation jets unless a short firing is still timing out
 * (SKIPx = 0); then CLOSEOUT.
 *
 * FUNCTION NAME: lm_dap_xtrans
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_xtrans(void)
{
    lm_dap_t    *d  = lm_dap_state(); /* DAP erasables */
    agc_status_t st = AGC_STATUS_OK;  /* a failing write replaces st */

    d->tj[1] = 0;
    d->tj[2] = 0;

    for (int axis = 0; axis < 2; axis++)
    {
        agc_sp_t old = d->skip[axis]; /* SKIPU/V before the reset, unscaled */

        d->skip[axis] = QJ_SKIPOFF;

        if (old != 0)
        {
            agc_status_t s = lm_dap_write_next(indexes[axis]); /* WRITEU/V -1: jets off */

            st = (s != AGC_STATUS_OK) ? s : st; /* a failure replaces st */
        }
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!AGC_ASSERT(d->skip[0] == QJ_SKIPOFF))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t sc = lm_dap_closeout(); /* CLOSEOUT status */

    return ((st != AGC_STATUS_OK) ? st : sc); /* first failure wins */
}

/**********************************************************************
 * ROT-TOUV (p.1457): (q, r) -> non-orthogonal (u, v) in which RCS jets
 * produce no cross-coupling. The AGC leaves U in A = -COEFFQ q + COEFFR r
 * and V in L = COEFFQ q + COEFFR r (COEFFQ < 0, so U ~ (Q+R)/sqrt2 and
 * V ~ (R-Q)/sqrt2), each limited to POSMAX/NEGMAX.
 *
 * FUNCTION NAME: lm_dap_rot_touv
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   q         agc_sp_t    I    Q component
 *   r         agc_sp_t    I    R component
 *   u         agc_sp_t *  O    U component, same scale
 *   v         agc_sp_t *  O    V component, same scale
 *
 * RETURNS: none
 **********************************************************************/
void lm_dap_rot_touv(agc_sp_t q, agc_sp_t r, agc_sp_t *u, agc_sp_t *v)
{
    const lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    agc_sp_t cq = lm_dap_mphi(q, d->coeffq); /* COEFFQ Q, scale of q */
    agc_sp_t cr = lm_dap_mphi(r, d->coeffr); /* COEFFR R, scale of r */

    *u = lm_dap_sat((int32_t) cr - cq);
    *v = lm_dap_sat((int32_t) cq + cr);
}

/**********************************************************************
 * NOROTAT (p.1454): no rotation on this axis; write its queued
 * translation jets.
 *
 * FUNCTION NAME: norotat
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  --------------------
 *   axis      int   I    U,V axis: 0 = U, 1 = V (AXISCTR)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
static agc_status_t norotat(int axis)
{
    agc_inhint();
    agc_status_t st = lm_dap_write_next(indexes[axis]); /* WRITEU/V -1: jets off */

    agc_relint();

    if (!AGC_ASSERT(axis == 0 || axis == 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (st);
}

/**********************************************************************
 * SELCTSUB failure exit (p.1458): TJx = 0, ALARM 02004, NOROTAT.
 *
 * FUNCTION NAME: rotation_failed
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   axis      int         I    U,V axis: 0 = U, 1 = V (AXISCTR)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
static agc_status_t rotation_failed(lm_dap_t *d, int axis)
{
    if (!AGC_ASSERT(axis == 0 || axis == 1))
    {
        return (AGC_STATUS_BAD_ARG); /* U or V only */
    }

    d->tj[axis + 1] = 0;
    agc_alarm(AGC_ALARM_02004); /* the pass goes on */

    if (!AGC_ASSERT(d->tj[axis + 1] == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t sn = norotat(axis); /* NOROTAT status */

    return (sn);
}

/**********************************************************************
 * FEEDBACK (p.1455): jets fired, for the rate estimator (1 or 2).
 *
 * FUNCTION NAME: feedback
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   axis      int         I    U,V axis: 0 = U, 1 = V (AXISCTR)
 *
 * RETURNS: none
 **********************************************************************/
static void feedback(lm_dap_t *d, int axis)
{
    d->nojets[axis + 1] = (d->numbert - 3 > 0) ? 2 : 1; /* NUMBERT 4: two jets, else one */
}

/**********************************************************************
 * AFTERTJ / DOROTAT (pp.1453-1454): select and fire the rotation jets for
 * TJU or TJV. Returns true in *end if XTRANS ended the pass (SNUFFBIT
 * set in a DPS burn: control inhibited).
 *
 * FUNCTION NAME: aftertj
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   axis      int         I    U,V axis: 0 = U, 1 = V (AXISCTR)
 *   end       bool *      O    true when XTRANS has already ended the pass
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
static agc_status_t aftertj(lm_dap_t *d, int axis, bool *end)
{
    *end = false;

    if (agc_flag_test(AGC_FLAG_SNUFFER) && !agc_flag_test(AGC_FLAG_APSFLAG) && !agc_flag_test(AGC_FLAG_DRIFTDFL))
    {
        *end = true;

        return (lm_dap_xtrans());
    }

    agc_sp_t tj = d->tj[axis + 1]; /* TJU or TJV, scaled 0.625 ms per count */

    if (tj == 0)
    {
        return (norotat(axis));
    }

    d->abstj    = (agc_sp_t) ((tj > 0) ? tj : -tj);       /* ABSTJ */
    d->rotindex = (agc_sp_t) (axis + ((tj > 0) ? 2 : 0)); /* -U -V +U +V   */

    if (d->abstj - QJ_150MS <= 0)
    {
        /**********************************************************************
         * DOSKIP / NOTMIN (pp.1453-1455): a firing of 150 ms or less is timed by
         * JTLST and the next pass of this axis is skipped. Under 14 ms becomes a
         * one-jet 14 ms minimum impulse, on the side translation prefers (or
         * chosen by the low bit of LOSCALAR).
         *
         * FUNCTION NAME: doskip
         *
         * ARGUMENTS:
         *   Argument  Type        I/O  Description (scale)
         *   --------  ----------  ---  --------------------
         *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
         *   axis      int         I    U,V axis: 0 = U, 1 = V (AXISCTR)
         *
         * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
         *          failed assertion, else the status of the routine it
         *          ends in
         **********************************************************************/
        if (!AGC_ASSERT(axis == 0 || axis == 1))
        {
            return (AGC_STATUS_BAD_ARG); /* U or V only */
        }

        if (QJ_TJMIN - d->abstj > 0)
        {
            d->abstj        = QJ_TJMIN;
            d->tj[axis + 1] = (agc_sp_t) ((d->tj[axis + 1] > 0) ? QJ_TJMIN : -QJ_TJMIN); /* MINTIMES with the sign of TJ */

            if (d->sensetyp > 0)
            {
                d->numbert = (agc_sp_t) (d->sensetyp - 1); /* NOTMIN -1      */
            }
            else
            {
                d->numbert = (agc_sp_t) (agc_chan_read(AGC_CHAN_LOSCALAR) & 1u);
            }
        }

        if (!lm_dap_selctsub())
        { /* NOTMIN         */
            return (rotation_failed(d, axis));
        }

        agc_inhint();
        d->t6furtha[1]  = indexes[axis];
        agc_status_t sw = lm_dap_write(indexes[axis], (uint16_t) d->polytemp); /* WRITEU/V: jets on now */

        d->t6furtha[0]  = d->abstj;
        agc_status_t sj = lm_dap_jtlst(); /* JTLST: schedule the turn-off */

        agc_relint();
        d->skip[axis] = 0;
        feedback(d, axis);

        if (sw != AGC_STATUS_OK)
        {
            return (sw);
        }

        return (sj);
    }

    if (!lm_dap_selctsub())
    {
        return (rotation_failed(d, axis));
    }

    agc_inhint();
    agc_status_t st = lm_dap_write(indexes[axis], (uint16_t) d->polytemp); /* WRITEU/V: jets on now */

    agc_relint();
    feedback(d, axis);

    if (!AGC_ASSERT(d->abstj > QJ_150MS))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
} /* aftertj */

/**********************************************************************
 * Skip logic (TJLAW, BACKHAND): SKIPx = 0 means a short firing of the
 * last pass is still being timed; skip this axis once.
 * AGC: none (helper)
 *
 * FUNCTION NAME: skip_axis
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   axis      int         I    U,V axis: 0 = U, 1 = V (AXISCTR)
 *
 * RETURNS: bool -- true when this axis is skipped this pass
 **********************************************************************/
static bool skip_axis(lm_dap_t *d, int axis)
{
    if (d->skip[axis] == 0)
    {
        d->skip[axis] = QJ_SKIPOFF;

        return (true);
    }

    return (false);
}

/**********************************************************************
 * TJLAW +3 (pp.1452-1453): attitude control of one U,V axis: TJETLAW,
 * or SPSRCS when docked (then GTS is tried next pass if allowed).
 *
 * FUNCTION NAME: body_tjlaw
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   axis      int         I    U,V axis: 0 = U, 1 = V (AXISCTR)
 *   end       bool *      O    true when XTRANS has already ended the pass
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
static agc_status_t body_tjlaw(lm_dap_t *d, int axis, bool *end)
{
    if (!AGC_ASSERT(axis == 0 || axis == 1))
    {
        return (AGC_STATUS_BAD_ARG); /* U or V only */
    }

    if (skip_axis(d, axis))
    {
        return (AGC_STATUS_OK);
    }

    d->e    = (axis == 0) ? d->uerror : d->verror; /* U or V attitude error */
    d->edot = (axis == 0) ? d->omegau : d->omegav; /* U or V rate */
    agc_status_t st;                               /* status of this axis' body */

    if (!agc_flag_test(AGC_FLAG_CSMDKFLG))
    {
        st = lm_dap_tjetlaw();
    }
    else
    {
        if (!agc_flag_test(AGC_FLAG_USEQRFLG))
        {
            d->cotroler = QJ_USEQRBIT; /* GTS next pass  */
        }

        agc_inhint();
        st         = lm_dap_spsrcs();
        agc_relint();
        d->numbert = 4; /* 2 jets U,V     */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (aftertj(d, axis, end));
}

/**********************************************************************
 * BACKHAND (pp.1449): direct rate. TJx = -(1/ANET) URATEDIF, at 32 s,
 * times 3 (scaled at 10.67 s, close to T6's 10.24); on overflow the
 * single product (right sign, more than 150 ms).
 *
 * FUNCTION NAME: body_backhand
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   axis      int         I    U,V axis: 0 = U, 1 = V (AXISCTR)
 *   end       bool *      O    true when XTRANS has already ended the pass
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
static agc_status_t body_backhand(lm_dap_t *d, int axis, bool *end)
{
    if (!AGC_ASSERT(axis == 0 || axis == 1))
    {
        return (AGC_STATUS_BAD_ARG); /* U or V only */
    }

    d->numbert = 4;

    if (skip_axis(d, axis))
    {
        return (AGC_STATUS_OK);
    }

    agc_sp_t rate = (axis == 0) ? d->omegau : d->omegav;                               /* URATEDIF or VRATEDIF, scaled pi/4 rad/s */
    int      i    = LM_BLK_U + uvdiff[axis] + ((rate < 0) ? 1 : 0) + LM_BK_1ANET2 + 1; /* 1/ANET2 word for the rate's sign (index) */

    if (!AGC_ASSERT(i >= 0 && i < LM_BLOCK_N))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_words_t p      = lm_dap_mp((agc_sp_t) -d->block[i], rate); /* -1/ANET2 x rate: jet time, DP */
    int32_t        triple = (2 * lm_dap_dpval(p)) / 16384 + p.hi;     /* DAS A, AD Q */

    d->tj[axis + 1] = lm_dap_ovf(triple) ? p.hi : (agc_sp_t) triple; /* DAS A, AD Q overflow: keep the major word */

    return (aftertj(d, axis, end));
}

/**********************************************************************
 * LOOPER (p.1454): run the RETJADR body for V (AXISCTR 1) then U, then
 * CLOSEOUT -- unless XTRANS already closed the pass.
 *
 * FUNCTION NAME: lm_dap_uvloop
 *
 * ARGUMENTS:
 *   Argument    Type             I/O  Description (scale)
 *   ----------  ---------------  ---  --------------------
 *   body        lm_dap_uvbody_t  I    RETJADR body: TJLAW, BACKHAND or MINRTN
 *   first_axis  agc_sp_t         I    AXISCTR of the first body pass (1 = V)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_uvloop(lm_dap_uvbody_t body, agc_sp_t first_axis)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->retjadr = body;

    for (int axis = first_axis; axis >= 0; axis--)
    {
        d->axisctr       = (agc_sp_t) axis;
        bool         end = false; /* XTRANS already ended the pass */
        agc_status_t st;          /* status of the body */

        switch (body)
        {
            case LM_UV_TJLAW:
                st = body_tjlaw(d, axis, &end);
                break;
            case LM_UV_BACKHAND:
                st = body_backhand(d, axis, &end);
                break;
            case LM_UV_MINRTN:
                /**********************************************************************
                 * MINRTN / MIMRET (pp.1445-1446): minimum impulse; 60 ms when docked.
                 *
                 * FUNCTION NAME: body_minrtn
                 *
                 * ARGUMENTS:
                 *   Argument  Type        I/O  Description (scale)
                 *   --------  ----------  ---  --------------------
                 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
                 *   axis      int         I    U,V axis: 0 = U, 1 = V (AXISCTR)
                 *   end       bool *      O    true when XTRANS has already ended the pass
                 *
                 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
                 *          failed assertion, else the status of the routine it
                 *          ends in
                 **********************************************************************/
            {
                if (!AGC_ASSERT(axis == 0 || axis == 1))
                {
                    return (AGC_STATUS_BAD_ARG);
                }

                if (agc_flag_test(AGC_FLAG_CSMDKFLG))
                {
                    agc_sp_t tj = d->tj[axis + 1]; /* TJU or TJV, scaled 0.625 ms per count */

                    if (tj > 0)
                    {
                        d->tj[axis + 1] = QJ_60MS;
                    }
                    else if (tj < 0)
                    {
                        d->tj[axis + 1] = -QJ_60MS;
                    }
                }

                d->numbert = agc_flag_test(AGC_FLAG_AORBTFLG) ? 3 : 2; /* B : A     */
                st         = aftertj(d, axis, &end);
            }
            break;
            default:
                st = AGC_STATUS_BAD_ARG;
                break;
        }

        if (end || st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    return (lm_dap_closeout());
}
