/*
 * external_delta_v.c -- the Comanche-only end of P30/P31 (P30-P37.agc, Comanche055
 * pp. 636-638): REFTEST (+MGA), DISPMGA/COMPTGO (start the TTOGO clock),
 * DISP45 (V16N45) and END45. They use the Comanche flag TIMRFLAG, so they
 * are kept out of the shared src/guidance/external_delta_v.c. Also DELRSPL
 * and AUGEKUGL (pp. 643-647), the V82 splash-error continuation in P11.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/csm/dap.h"
#include "agc/csm/entry.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"
#include "agc/programs/rendezvous.h"

#define GRP4 4

/* MARSDP OCT 00000, OCT 35100: 0.01 degree in the low word (rev 2^0).     */
static const agc_dp_t MARSDP = (agc_dp_t) 035100;

/* COMPTGO (CM p.636): start CLOKTASK (TTOGO), group 6.3 and group 4 type C
 * at PRIO13 continuing at DISP45.                                         */
agc_status_t agc_csm_comptgo(void)
{
    agc_guid_state()->csm.nvword1 = 0; /* NVWORD1 = 0           */
    agc_flag_set(AGC_FLAG_TIMRFLAG);
    agc_status_t st = agc_waitlist_call(1, AGC_TASK_CLOKTASK);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_MARKS_TFI_MGA_DISPLAY,
                         AGC_TASK_NONE, (int16_t) AGC_PRIO13,
                         false, 0U};

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_TIMRFLAG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change2((uint16_t) 040036, (uint16_t) 005024, &v);

    return (AGC_STATUS_OK); /* 6.3, 4 C */
}

/* REFTEST (CM p.638): +MGA from DELVSIN if REFSMMAT is valid, else
 * -00001 (MARSDP: shows -.01 deg); DISPMGA: COMPTGO, then DISP45.        */
agc_status_t agc_job_external_delta_v_reftest(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_rendz_t *rz = agc_rendz_erasable();

    if (agc_flag_test(AGC_FLAG_REFSMFLG))
    {
        agc_interp_t ip;

        agc_interp_init(&ip);
        agc_mat_t    refsmmat = agc_erase_refsmmat();
        agc_status_t st       =
            agc_rendz_get_mga(&ip, rz->delvsin, &refsmmat, &rz->plus_mga);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }
    else
    {
        rz->plus_mga = -MARSDP; /* NOTSET: DCS MARSDP    */
    }

    agc_status_t st = agc_csm_comptgo();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_marks_tfi_mga_display(ctx));
}

/* DISP45 (CM p.636): flash V16N45 (marks, TFI, +MGA) -- PROCEED: END45,
 * ENTER: redisplay; then P30PHSI: 4.1 and end of job.                    */
agc_status_t agc_job_marks_tfi_mga_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* A flashing display whose terminate goes to P00 (GOTOPOOH).             */
    agc_dsky_cont_t cont =
    {AGC_JOB_GOTOPOOH, AGC_JOB_MARKS_TFI_MGA_DONE,
     AGC_JOB_MARKS_TFI_MGA_DISPLAY};

    if (!AGC_ASSERT(AGC_JOB_MARKS_TFI_MGA_DONE != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st =
        agc_dsky_display(ctx, AGC_DSKY_GOFLASHR, agc_dsky_vn(16, 45), &cont);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_change(GRP4, (uint16_t) 000014);

    return (AGC_STATUS_OK); /* P30PHSI               */
}

/* END45 (CM p.636) and the tails of P30 and P31 (p.637): TIMRFLAG off,
 * XDELVFLG set after P30 (external delta-V) and reset after P31 (aimpoint),
 * then P00.                                                               */
agc_status_t agc_job_marks_tfi_mga_done(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_clear(AGC_FLAG_TIMRFLAG);

    if (agc_erase_modreg() == 31)
    {
        agc_flag_clear(AGC_FLAG_XDELVFLG);
    }
    else
    {
        agc_flag_set(AGC_FLAG_XDELVFLG);
    }

    return (agc_exec_gotopooh());
}

/* ---------------------------------------------------------------------- */
/* DELRSPL (CM pp.643-647): in-plane splash error for V82 in P11 / P00     */
/* ---------------------------------------------------------------------- */

/* AUGEKUGL's indexed tables (p.646-647). The AGC addresses them as
 * CK1K2 +1,1, YK1K2 +1,1 and V(32K) +1,1, i.e. base + 1 - X1 with X1 an
 * odd number 1..11; row (X1 - 1)/2 here. K = YK1K2 (V - V) + CK1K2.
 * Words read from the assembled rope (bank 32, 2201-2243).               */
typedef struct delrspl_k
{
    agc_dp_t vbase; /* velocity breakpoint, m/cs B-7                      */
    agc_dp_t slope; /* YK1K2 entry, B7 scaling of its 2DEC* value         */
    agc_dp_t bias;  /* CK1K2 entry                                        */
} delrspl_k_t;

static const delrspl_k_t AUG_K[6] = {
    {204547817, 8924363, 1789570},   /* X1 1:  V(32K), .000285, 2.4  */
    {178979340, 32879233, 745654},   /* X1 3:  28K,    .00105,  1    */
    {153410863, 38359106, -2386093}, /* X1 5:  V(24K), .001225, -3.2 */
    {19176358, 0, 82850},            /* X1 7:  V(3K),  0,       2400 */
    {178979340, -642218, 82850},     /* X1 9:  V(28K), -.443,   2400 */
    {2556848, 0, 189866},            /* X1 11: V(400), 0,       5500 */
};

/* Velocity bands of AUGEKUGL, m/cs B-7: V(21K) 64.000, V(3K) 9.144,
 * V(4K) 12.192, V(400) 1.2192 (subtracted in turn), V(26K) 79.248.      */
static const agc_dp_t AUG_V21K = 134217728;
static const agc_dp_t AUG_V3K  = 19176358;
static const agc_dp_t AUG_V4K  = 25568477;
static const agc_dp_t AUG_V400 = 2556848;
static const agc_dp_t AUG_V26K = 166195102;
/* MAXPHIC 2DEC .09259298: 2000 NM, the largest entry range, rev.         */
static const agc_dp_t AUG_MAXPHIC = 24855239;
/* TLESS26 2DEC* 5.70146688 E7 B-35*: 8660 PHI / V below 26000 ft/s;
 * TGR26CON 2DEC 7.2 E5 B-28: PHI/3 above, entry time cs 2^28.            */
static const agc_dp_t AUG_TLESS26  = 445427;
static const agc_dp_t AUG_TGR26CON = 720000;
/* 1BITDP: one low-order bit; TFF - 1BITDP overflows only for -POSMAX.    */
static const agc_dp_t DELRSPL_1BITDP = 1;

/* AUGEKUGL's band selection (p.644): X1 = 10, X2 = 8, S1 = 6 from
 * X1CON, then index-register exchanges per velocity band, INCR,1 1.
 * Returns the row of K1; *k2 the row of K2 (TIX,1: X1 - S1).            */
static int augekugl_rows(agc_interp_t *ip, agc_dp_t v, int *k2)
{
    agc_dp_t d    = agc_dp_sub(ip, v, AUG_V21K);
    int      x1   = 11;
    int      s1   = 6;
    int      band = 0;

    if (d >= 0)
    {
        d    = agc_dp_sub(ip, d, AUG_V3K);
        band = 1;
        x1   = 9;
        s1   = 4;
    }

    if (band == 1 && d >= 0)
    {
        d  = agc_dp_sub(ip, d, AUG_V4K);
        s1 = 6;
        band++;
    }

    if (band == 2 && d >= 0)
    {
        d  = agc_dp_sub(ip, d, AUG_V400);
        x1 = 7;
        s1 = (d >= 0) ? 6 : 4; /* SXA,1 S1 only above 28400 ft/s       */
    }

    *k2 = (x1 - s1 - 1) / 2;

    if (!AGC_ASSERT(*k2 >= 0 && *k2 < 3) || !AGC_ASSERT(x1 >= 7))
    {
        *k2 = 0;
    }

    return ((x1 - 1) / 2);
}

/* AUGEKUGL (pp.644-645): the estimated entry range PHI (rev) from the
 * speed V (m/cs B-7) and flight path angle GAMMA (rev) at 300 kft,
 *   PHI = K1 / (GAMMA - K2), limited to (0, MAXPHIC],
 * and the entry time T = PHI 8660/V below 26000 ft/s, PHI/3 above.      */
static agc_dp_t augekugl(agc_interp_t *ip, agc_dp_t v, agc_dp_t gamma,
                         agc_dp_t *phi)
{
    int               r2 = 0;
    const delrspl_k_t *a = &AUG_K[augekugl_rows(ip, v, &r2)];
    const delrspl_k_t *b = &AUG_K[r2];
    agc_dp_t           k1 = agc_dp_add(
        ip, agc_dp_mul_trunc(agc_dp_sub(ip, v, a->vbase), a->slope), a->bias);
    agc_dp_t k2 = agc_dp_add(
        ip, agc_dp_mul_trunc(agc_dp_sub(ip, v, b->vbase), b->slope), b->bias);
    agc_dp_t p = agc_dp_div(ip, k1, agc_dp_sub(ip, gamma, k2)); /* BDDV */

    if (agc_interp_test_ovf(ip) || p < 0 ||
        agc_dp_sub(ip, p, AUG_MAXPHIC) >= 0)
    {
        p = AUG_MAXPHIC; /* MAXPHI                                      */
    }

    *phi = p;

    if (!AGC_ASSERT(p >= 0 && p <= AUG_MAXPHIC))
    {
        return (0); /* no valid result */
    }

    if (agc_dp_sub(ip, v, AUG_V26K) >= 0)
    {
        return (agc_dp_mul_trunc(AUG_TGR26CON, p)); /* TGR26, TENT     */
    }

    return (agc_dp_mul_trunc(agc_dp_div(ip, AUG_TLESS26, v), p)); /* TENT */
}

/* DELRSPL up to GETARG (p.643): the free-fall transfer angle to 300 kft
 * (TFF/TRIG's .5 COS in CDELF/2) plus the entry range of AUGEKUGL, and
 * AUGEKUGL's entry time. The TFF debris is copied: VGAMCALC rewrites it. */
static agc_dp_t delrspl_theta(agc_interp_t *ip, const agc_tff_t *tff,
                              agc_dp_t *tent)
{
    agc_tff_t t      = *tff;
    agc_dp_t  vgam   = 0;
    agc_dp_t  cdelf2 = 0;
    agc_dp_t  phi    = 0;
    agc_dp_t  gamma  = agc_entry_vgamcalc(ip, &t, t.rterm, &vgam);
    agc_dp_t  sdelf2 = agc_entry_tff_trig(ip, &t, &cdelf2);

    if (!AGC_ASSERT(sdelf2 >= -AGC_DP_HALF && sdelf2 <= AGC_DP_HALF))
    {
        return (0); /* no valid result */
    }

    *tent = augekugl(ip, vgam, gamma, &phi);

    if (!AGC_ASSERT(*tent >= 0))
    {
        return (0); /* no valid result */
    }

    return (agc_dp_add(ip, agc_dp_acos(ip, cdelf2), phi)); /* THETA(1)  */
}

/* DELRSPL (pp.643-644). If the time of free fall to 300 kft TFF is
 * negative (still to come) and not -POSMAX (no solution):
 *   RSP-RREC = THETA(1) - angle(RONE, target at TIME - TFF + T ENTRY),
 * THETA(1) = free-fall angle + entry range; else (CANTDO) THETA(1) = 0
 * and the present target is used. TIME is PIPTIME with average G on
 * (V37FLAG), else TSTART82. Positive: overshoot. Ends with INTWAKE0.     */
agc_status_t agc_guid_delrspl(agc_interp_t *ip, const agc_tff_t *tff,
                              agc_dp_t tff_cs)
{
    if (!AGC_ASSERT(ip != NULL && tff != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_rendz_t *rz     = agc_rendz_erasable();
    agc_dp_t     theta1 = 0;
    agc_dp_t     tent   = 0;
    agc_dp_t     tffuse = 0; /* 8D: TFF, or 0 for CANTDO              */

    if (tff_cs < 0)
    { /* BPL CANTDO: past 300 kft     */
        agc_dp_t below = agc_dp_sub(ip, tff_cs, DELRSPL_1BITDP);

        if (!agc_interp_test_ovf(ip))
        { /* BOV CANTDO: TFF = -POSMAX  */
            theta1 = delrspl_theta(ip, tff, &tent);
            tffuse = tff_cs;
        }

        if (!AGC_ASSERT(below < tff_cs || tffuse == 0))
        {
            return (AGC_STATUS_BAD_ARG);
        }
    }

    agc_latlong_t ll = {agc_entry_state()->lat_spl,
                        agc_entry_state()->lng_spl, 0, 0}; /* GETARG */
    agc_dp_t t       = agc_flag_test(AGC_FLAG_SERVICER_RUNNING) ?
                       agc_erase_piptime() : rz->tstart_orbit;
    agc_navaux_padload_t pad;
    agc_vec_t            r  = agc_vec_zero();
    agc_status_t         st = agc_navaux_set_latlong(&ll);

    t = agc_dp_add(ip, agc_dp_sub(ip, t, tffuse), tent);
    agc_flag_clear(AGC_FLAG_ERADFLAG);
    agc_latlong_opt_t opt = {agc_flag_test(AGC_FLAG_LUNAFLAG), false};

    if (st == AGC_STATUS_OK)
    {
        st = agc_navaux_get_padload(&pad);
    }

    if (st == AGC_STATUS_OK)
    {
        st = agc_latlong_to_r(ip, &pad, opt, &ll, t, &r, NULL); /* LALOTORV */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_dp_t c = agc_vec_dot(ip, agc_vec_unit(ip, rz->rone, NULL, NULL),
                             agc_vec_unit(ip, r, NULL, NULL));

    agc_guid_state()->csm.rsp_rrec =
        agc_dp_sub(ip, theta1, agc_dp_acos(ip, agc_dp_shl(ip, c, 1)));

    return (agc_integ_intwake0()); /* DELRDONE: STCALL RSP-RREC INTWAKE0 */
}
