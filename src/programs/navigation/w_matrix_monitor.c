/*
 * w_matrix_monitor.c -- V67 W-MATRIX MONITOR, both vehicles (P20-P25.agc,
 * Comanche055 pp.632-634, Luminary099 pp.608-611).
 *
 * The extended verb shows on noun 99 the RMS position and velocity
 * errors of the W matrix (WWPOS = SQRT(sum of the squares of the position
 * rows), WWVEL; LM also WWBIAS for the radar bias rows), and lets the
 * crew load new initial values:
 *   CM: R3 = WWOPT selects which pad pair gets WWPOS, WWVEL (1 rendezvous
 *       WRENDPOS/WRENDVEL, 2 orbital WORBPOS/WORBVEL, 3 midcourse
 *       WMIDPOS/WMIDVEL); 0 changes nothing.
 *   LM: a change of any displayed value (V67FLAG) loads WRENDPOS/WRENDVEL
 *       (WSURFPOS/WSURFVEL on the surface) and WSHAFT = WTRUN = WWBIAS.
 * The W matrix is then invalid (RENDWFLG, CM also ORBWFLAG, cleared).
 *
 * WWPOS EQUALS RANGE, WWVEL EQUALS RRATE, WWOPT/WWBIAS EQUALS RTHETA.
 * Scales: WWPOS m 2^19 (limited to 99999 ft), WWVEL m/cs, WWBIAS noun
 * scale B-5 (internal B+5); the W elements as integrated.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "internal.h"

#define WM_FT99999 ((agc_dp_t) 15605248) /* FT99999 2DEC 30479 B-19      */
#define WM_DEC2    ((agc_sp_t) 2)        /* V67DEC2 2DEC 2 B-14 (option 2) */

/* LM: the noun 99 values before the display (WWBIAS +2 .. +7).          */
static agc_dp_t wm_saved[3];

/* Sum of |row|^2 over rows 0..rows-1 of a W column block.               */
static agc_dp_t wm_sum(agc_interp_t *ip, const agc_incorp_w_t *w, int b,
                       int rows)
{
    agc_dp_t s = 0;

    for (int m = 0; m < rows && m < AGC_INCORP_ROWS; m++)
    {
        s = agc_dp_add(ip, agc_vec_sq(ip, w->blk[b][m]), s); /* VSQ, DAD */
    }

    if (!AGC_ASSERT(b >= 0 && b < AGC_INCORP_BLKS))
    {
        return (0); /* no valid result */
    }

    return (s);
}

/* V67WW: under INTSTALL, the RMS values; an overflow anywhere makes them
 * all DPPOSMAX; WWPOS limited to 99999 ft; INTWAKE.                     */
static agc_status_t wm_compute(agc_interp_t *ip, agc_rendz_t *rz)
{
    agc_status_t st = agc_integ_install(0, AGC_JOB_W_MATRIX_MONITOR);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_clear_ovf(ip); /* BOV +1   */
    const agc_incorp_w_t *w    = &agc_integ_shared_ctx()->w;
    bool                  csm  = rz_is_csm();
    int                   rows = csm ? 6 : 9; /* X1 36/54 */
    agc_dp_t              pos  = wm_sum(ip, w, 0, rows);
    agc_dp_t              vel  = wm_sum(ip, w, 1, rows);

    rz->rtheta =
        csm ? 0 : agc_dp_shr(agc_dp_sqrt(ip, wm_sum(ip, w, 2, rows)), 10);
    rz->rrate = agc_dp_sqrt(ip, vel);
    rz->range = agc_dp_sqrt(ip, pos);

    if (agc_interp_test_ovf(ip))
    {
        rz->range  = RZ_DPPOSMAX;
        rz->rrate  = RZ_DPPOSMAX;
        rz->rtheta = csm ? rz->rtheta : RZ_DPPOSMAX;
    }

    if (agc_dp_sub(ip, rz->range, WM_FT99999) >= 0)
    { /* V67XXX   */
        rz->range = WM_FT99999;
    }

    return (agc_integ_intwake());
}

/* V67CALL: compute, (LM) keep the values, V06N99 (GOXDSPF): TERMINATE
 * ends the verb, PROCEED takes the loads, ENTER shows it again.       */
agc_status_t agc_job_w_matrix_monitor(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_rendz_t *rz = rz_state();
    agc_interp_t ip;

    agc_interp_init(&ip);

    if (!rz_is_csm())
    {
        rz_flag_put(AGC_FLAG_W_MATRIX_OVERWRITE, false); /* V67FLAG  */
    }

    agc_status_t st = wm_compute(&ip, rz);

    if (st != AGC_STATUS_OK)
    {
        return (st); /* BUSY: asleep in INTSTALL  */
    }

    wm_saved[0] = rz->range;
    wm_saved[1] = rz->rrate;
    wm_saved[2] = rz->rtheta;

    return (agc_job_w_matrix_monitor_display(ctx));
}

agc_status_t agc_job_w_matrix_monitor_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_flash(
                ctx, AGC_DSKY_GOMARKF, 6U, 99U, AGC_JOB_RENDZ_ENDEXT, /* GOXDSPF */
                AGC_JOB_W_MATRIX_MONITOR_PROCEED,
                AGC_JOB_W_MATRIX_MONITOR_DISPLAY));
}

/* CM V06N9933 / V6N99INP: option 0 changes nothing; else the pad pair
 * of the option gets the high words of WWPOS, WWVEL.                  */
static void wm_load_csm(const agc_rendz_t *rz, agc_rendz_nav_pad_t *p)
{
    agc_sp_t opt = agc_dp_high(rz->rtheta); /* WWOPT    */
    agc_sp_t x1  = agc_dp_high(rz->range);  /* LXA,1    */
    agc_sp_t x2  = agc_dp_high(rz->rrate);

    if (opt == WM_DEC2)
    { /* V67WORB  */
        p->worbpos = x1;
        p->worbvel = x2;
    }
    else if (opt > WM_DEC2)
    { /* V67WMID  */
        p->wmidpos = x1;
        p->wmidvel = x2;
    }
    else
    {
        p->wrendpos = x1;
        p->wrendvel = x2;
    }

    rz_flag_put(AGC_FLAG_ORBWFLAG, false); /* V67EXITX */
}

/* LM V06N9933: WWPOS SL5 and WWVEL to WRENDPOS/WRENDVEL (WSURF.. on the
 * surface), WWBIAS SL10 to WTRUN and WSHAFT.                            */
static void wm_load_lm(agc_interp_t *ip, const agc_rendz_t *rz,
                       agc_rendz_nav_pad_t *p)
{
    agc_sp_t pos =
        agc_dp_high(agc_dp_shl(ip, agc_dp_shl(ip, rz->range, 4), 1));
    agc_sp_t vel  = agc_dp_high(rz->rrate);
    agc_sp_t bias = agc_dp_high(agc_dp_shl(ip, rz->rtheta, 10));

    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    { /* V67SURF  */
        p->wsurfpos = pos;
        p->wsurfvel = vel;
    }
    else
    {
        p->wrendpos = pos;
        p->wrendvel = vel;
    }

    p->wtrun  = bias; /* V67CLRF  */
    p->wshaft = bias;
}

/* PROCEED on V06N99: (LM V6N99PRO) any displayed value changed sets
 * V67FLAG; load the pads; W invalid (RENDWFLG off); ENDEXT.            */
agc_status_t agc_job_w_matrix_monitor_proceed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_rendz_t         *rz = rz_state();
    agc_rendz_nav_pad_t *p  = agc_rendz_nav_pad();
    agc_interp_t         ip;

    agc_interp_init(&ip);

    if (rz_is_csm())
    {
        if (agc_dp_high(rz->rtheta) == 0)
        { /* BHIZ     */
            return (rz_endext());
        }

        wm_load_csm(rz, p);
    }
    else
    {
        if (rz->range != wm_saved[0] || rz->rrate != wm_saved[1] ||
            rz->rtheta != wm_saved[2])
        {
            rz_flag_put(AGC_FLAG_W_MATRIX_OVERWRITE, true);
        }

        if (!agc_flag_test(AGC_FLAG_W_MATRIX_OVERWRITE))
        {
            return (rz_endext());
        }

        wm_load_lm(&ip, rz, p);
    }

    rz_flag_put(AGC_FLAG_RENDWFLG, false);

    return (rz_endext());
}
