/*
 * qrjets.c -- the U,V jet loop of the Q,R-axis RCS autopilot
 * (Q_R-AXIS_RCS_AUTOPILOT.agc, Luminary 099 pp.1445-1458): the three
 * "RETJADR" bodies (TJLAW +3 attitude control, BACKHAND direct rate,
 * MINRTN minimum impulse), the jet selection AFTERTJ / DOROTAT / DOSKIP /
 * FEEDBACK, SELCTSUB, XTRANS, ROT-TOUV and CLOSEOUT.
 *
 * The AGC runs a body for the V axis (AXISCTR = 1) and then for U
 * (AXISCTR = 0); LOOPER returns through RETJADR. Here that is a bounded
 * loop over the axis in lm_dap_uvloop().
 */
#include <stddef.h>
#include "agc/agc_assert.h"
#include "agc/agc_channels.h"
#include "agc/agc_lm_dap.h"
#include "agc/agc_lm_dap_internal.h"

#define QJ_150MS    240         /* -150MS = -240: 150 ms in T6 counts      */
#define QJ_TJMIN    22          /* +TJMINT6 = 14MS: 14 ms minimum firing   */
#define QJ_60MS     96          /* 60MS: docked minimum impulse            */
#define QJ_SKIPOFF  4           /* SKIPU/V = 4: not skipping               */
#define QJ_USEQRBIT 8191        /* USEQRJTS (BIT14) - 1 after CCS          */
#define QJ_NALL     9

/* INDEXES (p.1455): T6 axis index of U and V (WRITEP +4, WRITEP +13).    */
static const agc_sp_t indexes[2] = { 4, 13 };
/* AXISDIFF (TJET_LAW p.1469) for U, V.                                   */
static const agc_sp_t uvdiff[2] = { 0, 16 };
/* ALLJETS (-U -V +U +V) followed by TYPEPOLY (-X +X A B ALL), contiguous
 * as in the AGC so that X translation indexes ALLJETS with 4 and 5.     */
static const uint16_t alljets[QJ_NALL] = {
    00110, 00022, 00204, 00041, 00125, 00252, 00146, 00231, 00377
};

/* SELCTSUB / FAILOOP (pp.1457-1458): POLYTEMP = ALLJETS(ROTINDEX) &
 * TYPEPOLY(NUMBERT); on a failed jet retry NUMBERT = 3, 2, 1, 0. False if
 * even NUMBERT = 0 has a failed jet.                                    */
static bool policy_ok(lm_dap_t *d)
{
    d->polytemp = (agc_sp_t)(alljets[d->rotindex] & alljets[4 + d->numbert]);
    (void)AGC_ASSERT(d->polytemp >= 0 && d->polytemp <= 0377);
    (void)AGC_ASSERT(d->ch5mask <= 0377u);
    return ((uint16_t)d->polytemp & d->ch5mask) == 0u;
}

bool lm_dap_selctsub(void)
{
    lm_dap_t *d = lm_dap_state();
    if (!AGC_ASSERT(d->rotindex >= 0 && d->rotindex <= 5) ||
        !AGC_ASSERT(d->numbert >= 0 && d->numbert <= 4)) {
        return false;
    }
    if (policy_ok(d)) {
        return true;
    }
    for (int n = 3; n >= 0; n--) {                        /* FAILOOP        */
        d->numbert = (agc_sp_t)n;
        if (policy_ok(d)) {
            return true;
        }
    }
    return false;
}

/* CLOSEOUT / ENDJASK (p.1458): the jask is over (DAPZRUPT negative).     */
agc_status_t lm_dap_closeout(void)
{
    lm_dap_t *d = lm_dap_state();
    d->dapzrupt = LM_NEGMAX;
    (void)AGC_ASSERT(d->dapzrupt < 0);
    (void)AGC_ASSERT(d->t5adr <= LM_T5_PAXIS);
    return AGC_STATUS_OK;
}

/* XTRANS (p.1455): no rotation (or control inhibited): TJU = TJV = 0;
 * write the translation jets unless a short firing is still timing out
 * (SKIPx = 0); then CLOSEOUT.                                            */
agc_status_t lm_dap_xtrans(void)
{
    lm_dap_t *d = lm_dap_state();
    agc_status_t st = AGC_STATUS_OK;
    d->tj[1] = 0;
    d->tj[2] = 0;
    for (int axis = 0; axis < 2; axis++) {
        agc_sp_t old = d->skip[axis];
        d->skip[axis] = QJ_SKIPOFF;
        if (old != 0) {
            agc_status_t s = lm_dap_write_next(indexes[axis]);
            st = (s != AGC_STATUS_OK) ? s : st;
        }
    }
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    (void)AGC_ASSERT(d->skip[0] == QJ_SKIPOFF);
    agc_status_t sc = lm_dap_closeout();
    return (st != AGC_STATUS_OK) ? st : sc;
}

/* ROT-TOUV (p.1457): (q, r) -> non-orthogonal (u, v) in which RCS jets
 * produce no cross-coupling: u = COEFFQ q + COEFFR r, v = -COEFFQ q +
 * COEFFR r, each limited to POSMAX/NEGMAX.                              */
void lm_dap_rot_touv(agc_sp_t q, agc_sp_t r, agc_sp_t *u, agc_sp_t *v)
{
    const lm_dap_t *d = lm_dap_state();
    if (!AGC_ASSERT(u != NULL && v != NULL)) {
        return;
    }
    agc_sp_t cq = lm_dap_mphi(q, d->coeffq);
    agc_sp_t cr = lm_dap_mphi(r, d->coeffr);
    *u = lm_dap_sat((int32_t)cq + cr);
    *v = lm_dap_sat((int32_t)cr - cq);
    (void)AGC_ASSERT(*u >= LM_NEGMAX && *v >= LM_NEGMAX);
}

agc_status_t agc_lm_dap_rot_touv(agc_sp_t q, agc_sp_t r, agc_sp_t *u, agc_sp_t *v)
{
    if (!AGC_ASSERT(u != NULL) || !AGC_ASSERT(v != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    lm_dap_rot_touv(q, r, u, v);
    return AGC_STATUS_OK;
}

/* NOROTAT (p.1454): no rotation on this axis; write its queued
 * translation jets.                                                      */
static agc_status_t norotat(int axis)
{
    agc_inhint();
    agc_status_t st = lm_dap_write_next(indexes[axis]);
    agc_relint();
    (void)AGC_ASSERT(axis == 0 || axis == 1);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return st;
}

/* SELCTSUB failure exit (p.1458): TJx = 0, ALARM 02004, NOROTAT.        */
static agc_status_t rotation_failed(lm_dap_t *d, int axis)
{
    d->tj[axis + 1] = 0;
    (void)agc_alarm(AGC_ALARM_02004);
    (void)AGC_ASSERT(d->tj[axis + 1] == 0);
    return norotat(axis);
}

/* FEEDBACK (p.1455): jets fired, for the rate estimator (1 or 2).       */
static void feedback(lm_dap_t *d, int axis)
{
    d->nojets[axis + 1] = (d->numbert - 3 > 0) ? 2 : 1;
    (void)AGC_ASSERT(axis == 0 || axis == 1);
    (void)AGC_ASSERT(d->numbert >= 0 && d->numbert <= 4);
}

/* DOSKIP / NOTMIN (pp.1453-1455): a firing of 150 ms or less is timed by
 * JTLST and the next pass of this axis is skipped. Under 14 ms becomes a
 * one-jet 14 ms minimum impulse, on the side translation prefers (or
 * chosen by the low bit of LOSCALAR).                                    */
static agc_status_t doskip(lm_dap_t *d, int axis)
{
    if (QJ_TJMIN - d->abstj > 0) {
        d->abstj = QJ_TJMIN;
        d->tj[axis + 1] = (agc_sp_t)((d->tj[axis + 1] > 0) ? QJ_TJMIN : -QJ_TJMIN);
        if (d->sensetyp > 0) {
            d->numbert = (agc_sp_t)(d->sensetyp - 1);    /* NOTMIN -1      */
        } else {
            d->numbert = (agc_sp_t)(agc_chan_read(AGC_CHAN_LOSCALAR) & 1u);
        }
    }
    if (!lm_dap_selctsub()) {                            /* NOTMIN         */
        return rotation_failed(d, axis);
    }
    agc_inhint();
    d->t6furtha[1] = indexes[axis];
    agc_status_t sw = lm_dap_write(indexes[axis], (uint16_t)d->polytemp);
    d->t6furtha[0] = d->abstj;
    agc_status_t sj = lm_dap_jtlst();
    agc_relint();
    d->skip[axis] = 0;
    feedback(d, axis);
    (void)AGC_ASSERT(sw == AGC_STATUS_OK && sj == AGC_STATUS_OK);
    return sw;
}

/* AFTERTJ / DOROTAT (pp.1453-1454): select and fire the rotation jets for
 * TJU or TJV. Returns true in *end if XTRANS ended the pass (SNUFFBIT
 * set in a DPS burn: control inhibited).                                 */
static agc_status_t aftertj(lm_dap_t *d, int axis, bool *end)
{
    *end = false;
    if (agc_flag_test(AGC_FLAG_SNUFFER) && !agc_flag_test(AGC_FLAG_APSFLAG) &&
        !agc_flag_test(AGC_FLAG_DRIFTDFL)) {
        *end = true;
        return lm_dap_xtrans();
    }
    agc_sp_t tj = d->tj[axis + 1];
    if (tj == 0) {
        return norotat(axis);
    }
    d->abstj = (agc_sp_t)((tj > 0) ? tj : -tj);
    d->rotindex = (agc_sp_t)(axis + ((tj > 0) ? 2 : 0)); /* -U -V +U +V   */
    if (d->abstj - QJ_150MS <= 0) {
        return doskip(d, axis);
    }
    if (!lm_dap_selctsub()) {
        return rotation_failed(d, axis);
    }
    agc_inhint();
    agc_status_t st = lm_dap_write(indexes[axis], (uint16_t)d->polytemp);
    agc_relint();
    feedback(d, axis);
    (void)AGC_ASSERT(d->abstj > QJ_150MS);
    return st;
}

/* Skip logic (TJLAW, BACKHAND): SKIPx = 0 means a short firing of the
 * last pass is still being timed; skip this axis once.                  */
static bool skip_axis(lm_dap_t *d, int axis)
{
    (void)AGC_ASSERT(d->skip[axis] == 0 || d->skip[axis] == QJ_SKIPOFF);
    if (d->skip[axis] == 0) {
        d->skip[axis] = QJ_SKIPOFF;
        return true;
    }
    (void)AGC_ASSERT(axis == 0 || axis == 1);
    return false;
}

/* TJLAW +3 (pp.1452-1453): attitude control of one U,V axis: TJETLAW,
 * or SPSRCS when docked (then GTS is tried next pass if allowed).       */
static agc_status_t body_tjlaw(lm_dap_t *d, int axis, bool *end)
{
    if (skip_axis(d, axis)) {
        return AGC_STATUS_OK;
    }
    d->e = (axis == 0) ? d->uerror : d->verror;
    d->edot = (axis == 0) ? d->omegau : d->omegav;
    agc_status_t st;
    if (!agc_flag_test(AGC_FLAG_CSMDKFLG)) {
        st = lm_dap_tjetlaw();
    } else {
        if (!agc_flag_test(AGC_FLAG_USEQRFLG)) {
            d->cotroler = QJ_USEQRBIT;                    /* GTS next pass  */
        }
        agc_inhint();
        st = lm_dap_spsrcs();
        agc_relint();
        d->numbert = 4;                                   /* 2 jets U,V     */
    }
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return aftertj(d, axis, end);
}

/* BACKHAND (pp.1449): direct rate. TJx = -(1/ANET) URATEDIF, at 32 s,
 * times 3 (scaled at 10.67 s, close to T6's 10.24); on overflow the
 * single product (right sign, more than 150 ms).                        */
static agc_status_t body_backhand(lm_dap_t *d, int axis, bool *end)
{
    d->numbert = 4;
    if (skip_axis(d, axis)) {
        return AGC_STATUS_OK;
    }
    agc_sp_t rate = (axis == 0) ? d->omegau : d->omegav;  /* URATEDIF       */
    int i = LM_BLK_U + uvdiff[axis] + ((rate < 0) ? 1 : 0) + LM_BK_1ANET2 + 1;
    if (!AGC_ASSERT(i >= 0 && i < LM_BLOCK_N)) {
        return AGC_STATUS_BAD_ARG;
    }
    lm_dp_pair_t p = lm_dap_mp((agc_sp_t)-d->block[i], rate);
    int32_t triple = (2 * lm_dap_dpval(p)) / 16384 + p.hi;   /* DAS A, AD Q */
    d->tj[axis + 1] = lm_dap_ovf(triple) ? p.hi : (agc_sp_t)triple;
    return aftertj(d, axis, end);
}

/* MINRTN / MIMRET (pp.1445-1446): minimum impulse; 60 ms when docked.   */
static agc_status_t body_minrtn(lm_dap_t *d, int axis, bool *end)
{
    if (agc_flag_test(AGC_FLAG_CSMDKFLG)) {
        agc_sp_t tj = d->tj[axis + 1];
        d->tj[axis + 1] = (agc_sp_t)((tj > 0) ? QJ_60MS : ((tj < 0) ? -QJ_60MS : 0));
    }
    d->numbert = agc_flag_test(AGC_FLAG_AORBTFLG) ? 3 : 2;   /* B : A     */
    (void)AGC_ASSERT(axis == 0 || axis == 1);
    return aftertj(d, axis, end);
}

/* LOOPER (p.1454): run the RETJADR body for V (AXISCTR 1) then U, then
 * CLOSEOUT -- unless XTRANS already closed the pass.                     */
agc_status_t lm_dap_uvloop(lm_dap_uvbody_t body, agc_sp_t first_axis)
{
    lm_dap_t *d = lm_dap_state();
    if (!AGC_ASSERT(first_axis == 0 || first_axis == 1) ||
        !AGC_ASSERT(body <= LM_UV_MINRTN)) {
        return AGC_STATUS_BAD_ARG;
    }
    d->retjadr = body;
    for (int axis = first_axis; axis >= 0; axis--) {
        d->axisctr = (agc_sp_t)axis;
        bool end = false;
        agc_status_t st;
        switch (body) {
        case LM_UV_TJLAW:    st = body_tjlaw(d, axis, &end);    break;
        case LM_UV_BACKHAND: st = body_backhand(d, axis, &end); break;
        case LM_UV_MINRTN:   st = body_minrtn(d, axis, &end);   break;
        default:             st = AGC_STATUS_BAD_ARG;           break;
        }
        if (end || st != AGC_STATUS_OK) {
            return st;
        }
    }
    return lm_dap_closeout();
}
