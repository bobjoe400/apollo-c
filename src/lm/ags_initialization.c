/*
 * ags_initialization.c -- R47 AGS initialization (LM), AGS_INITIALIZATION.agc,
 * Luminary099 pp. 206-210 (labels AGSINIT, NEWAGS, REDSPTEM, AGSDISPK,
 * AGSVCALC, CKSTALL, AGSEND, SCALEVEC).  Extended verb V47: the crew
 * sets the AGS clock zero AGSK (V06N16; V32 takes the time of the ENTER
 * keystroke), then the LM and CSM states in SM axes, feet and ft/s,
 * rounded for the AGS's 2's complement words, and the time since AGSK are
 * put in AGSBUFF and sent on the AGS downlist for 20 s; finally the ICDUs
 * are zeroed (IMUZERO) unless the IMU is in use.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

#define ALM_NO_REFSMMAT ((agc_alarm_code_t) 0220)
#define VN_AGS_TIME     ALIGN_VN(6, 16)   /* V06N16 VN 0616                */
#define VN_AGS_DONE     ALIGN_VN(50, 16)  /* V50N16 VN 5016                */
#define LAGSLIST        1                 /* LAGSLIST = ONE (AGS downlist) */
#define DT_DOWNLIST     ((uint16_t) 2000) /* 20SEC DEC 2000                */
#define DT_RETRY        ((uint16_t) 10)   /* TEN cs                        */
/* RSCALE 2DEC 3.280839 B-3 (m -> ft) and VSCALE 2DEC 3.280839 E2 B-9
 * (m/cs -> ft/s): the words assembled in the Luminary 099 rope (15077 05041,
 * 24402 25724), 30 and 15 LSB from the exact decimal conversion.          */
#define RSCALE ((agc_dp_t) 110086689) /* 3.280839 B-3, ft/m              */
#define VSCALE ((agc_dp_t) 172010452) /* 328.0839 B-9, (ft/s)/(m/cs)     */
#define TSCALE ((agc_dp_t) 26214400) /* TSCALE 2DEC 100 B-10          */
/* AGSRND1 (2OCT 0000060000) + AGSRND2 (2OCT 0000037777): half a high-word
 * LSB, added in two steps; here as one DP addend.                        */
#define AGS_ROUND    ((agc_dp_t) 8192)
#define AGSBUFF_TIME 12 /* AGSBUFF +12D: the DP time     */

static uint16_t agsbuff[AGC_ALIGN_AGSBUFF_WORDS];
static agc_dp_t agsk; /* AGSK: AGS clock zero, cs B28  */

const uint16_t *agc_ags_buffer(void)
{
    _Static_assert(AGC_ALIGN_AGSBUFF_WORDS == 14, "AGC constant");

    return (agsbuff);
}

agc_dp_t agc_ags_agsk(void)
{
    if (!AGC_ASSERT(agsk >= AGC_DP_MIN))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(agsk <= AGC_DP_MAX))
    {
        return (0); /* no valid result */
    }

    return (agsk);
}

agc_status_t agc_ags_set_agsk(agc_dp_t t)
{
    if (!AGC_ASSERT((t >= AGC_DP_MIN) && (t <= AGC_DP_MAX)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agsk = t;

    return (AGC_STATUS_OK);
}

/* VECSGNAG and the high words: v + half an LSB, the high word of each
 * component as the raw 15-bit word the AGS reads (2's complement).      */
static void ags_words(agc_interp_t *ip, agc_vec_t v, uint16_t w[3])
{
    for (int i = 0; i < 3; i++)
    {
        const agc_dp_t r = agc_dp_add(ip, v.c[i], AGS_ROUND);

        w[i] = align_sp_to_word(agc_dp_high(r));
    }

    if (!AGC_ASSERT(w[0] <= 077777u))
    {
        return;
    }
}

/*
 * SCALEVEC (p. 209): V = REFSMMAT VATT1 x VSCALE, VSL2; R = REFSMMAT RATT1
 * x RSCALE, VSL8; each rounded.  With the half-unit REFSMMAT (MXV keeps
 * its B1) and moon-sphere states (RATT1 m B27, VATT1 m/cs B5) the high
 * words are R in 512 ft (2^23 ft full scale) and V in 0.5 ft/s (2^13 ft/s).
 * out = RX, VX, RY, VY, RZ, VZ (the high words; VATT1's go into R's
 * low-word places).
 */
agc_status_t agc_ags_scalevec(agc_interp_t *ip, agc_vec_t ratt1,
                              agc_vec_t vatt1, const agc_mat_t *refsmmat,
                              uint16_t out[6])
{
    if (!AGC_ASSERT((ip != NULL) && (refsmmat != NULL)) ||
        !AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_vec_t v = agc_vec_shl(
        ip, agc_vec_scale(ip, agc_mat_vec(ip, refsmmat, vatt1), VSCALE), 2);
    const agc_vec_t r = agc_vec_shl(
        ip, agc_vec_scale(ip, agc_mat_vec(ip, refsmmat, ratt1), RSCALE), 8);
    uint16_t rw[3];
    uint16_t vw[3];

    ags_words(ip, v, vw);
    ags_words(ip, r, rw);

    for (int i = 0; i < 3; i++)
    {
        out[2 * i]       = rw[i];
        out[(2 * i) + 1] = vw[i];
    }

    return (AGC_STATUS_OK);
}

/* AGSBUFF +12D: (TAT - AGSK) / TSCALE, seconds B18. */
agc_status_t agc_ags_time(agc_interp_t *ip, agc_dp_t tat, agc_dp_t k,
                          agc_dp_t *out)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *out = agc_dp_div(ip, agc_dp_sub(ip, tat, k), TSCALE);

    return (AGC_STATUS_OK);
}

/* REDSPTEM / AGSDISPK: DSPTEMX = AGSK; GOMARKF V06N16. */
static agc_status_t show_agsk(agc_job_ctx_t *ctx)
{
    agc_erase_set_dsptemx(agsk);

    return (align_flash(ctx, AGC_DSKY_GOMARKF, VN_AGS_TIME,
                        AGC_JOB_AGS_INITIALIZATION_END,
                        AGC_JOB_AGS_INITIALIZATION_COMPUTE,
                        AGC_JOB_AGS_INITIALIZATION_NEW_ZERO));
}

/* AGSINIT: alarm 220 and ENDEXT without a REFSMMAT; else REDSPTEM. */
agc_status_t agc_job_ags_initialization(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) ||
        !AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!agc_flag_test(AGC_FLAG_REFSMFLG))
    {
        agc_alarm(ALM_NO_REFSMMAT);

        return (agc_dsky_endext()); /* TC ALARM returns; TC ENDEXT */
    }

    return (show_agsk(ctx));
}

/*
 * V06N16 ENTER: V32 (NEWAGS) makes the time of the ENTER keystroke
 * (SAMPTIME) the new AGSK; otherwise the AGSK keyed in (V25) is taken
 * from DSPTEMX; shown again.
 */
agc_status_t agc_job_ags_initialization_new_zero(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agsk = align_resp_recycle() ? agc_erase_samptime() : agc_erase_dsptemx();

    if (!AGC_ASSERT(agsk >= AGC_DP_MIN))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (show_agsk(ctx));
}

/* AGSVCALC: SET NODOFLAG (no V37), XDSPFLAG; EXDSPRET V06N16. */
agc_status_t agc_job_ags_initialization_compute(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_set(AGC_FLAG_NODOFLAG);
    agc_flag_set(AGC_FLAG_XDSPFLAG);

    return (align_flash(ctx, AGC_DSKY_EXDSPRET, VN_AGS_TIME,
                        AGC_JOB_AGS_INITIALIZATION_LM_STATE,
                        AGC_JOB_AGS_INITIALIZATION_LM_STATE,
                        AGC_JOB_AGS_INITIALIZATION_LM_STATE));
}

/* LEMPREC to now; SCALEVEC -> AGSBUFF; TAT kept for the CSM. */
static agc_dp_t ags_tat;

agc_status_t agc_job_ags_initialization_lm_state(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_integ_out_t o;
    agc_status_t    st = align_integ(&ip, ALIGN_LEMPREC, align_loadtime(),
                                     AGC_JOB_AGS_INITIALIZATION_LM_STATE, &o);

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* INTSTALL */
    }

    const agc_mat_t refsmmat = agc_erase_refsmmat();

    if (st == AGC_STATUS_OK)
    {
        st = agc_ags_scalevec(&ip, o.ratt1, o.vatt1, &refsmmat, &agsbuff[0]);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ags_tat = o.tat;

    return (agc_job_ags_initialization_csm_state(ctx));
}

/*
 * CSMPREC to TAT; SCALEVEC -> AGSBUFF +6; AGSBUFF +12D = time since AGSK;
 * the AGS downlist (LAGSLIST) for 20 s.
 */
agc_status_t agc_job_ags_initialization_csm_state(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_integ_out_t o;
    agc_status_t    st = align_integ(&ip, ALIGN_CSMPREC, ags_tat,
                                     AGC_JOB_AGS_INITIALIZATION_CSM_STATE, &o);

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK);
    }

    const agc_mat_t refsmmat = agc_erase_refsmmat();
    agc_dp_t        t        = 0;

    if (st == AGC_STATUS_OK)
    {
        st = agc_ags_scalevec(&ip, o.ratt1, o.vatt1, &refsmmat, &agsbuff[6]);
    }

    if (st == AGC_STATUS_OK)
    {
        st = agc_ags_time(&ip, ags_tat, agsk, &t);
    }

    if (st == AGC_STATUS_OK)
    {
        agsbuff[AGSBUFF_TIME]     = align_sp_to_word(agc_dp_high(t));
        agsbuff[AGSBUFF_TIME + 1] = align_sp_to_word(agc_dp_low(t));
        agc_telem_select_list(LAGSLIST); /* DNLSTCOD */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_svc_delayjob(DT_DOWNLIST, AGC_JOB_AGS_INITIALIZATION_SENT));
}

/* AGSEND: DOWNFLAG NODOFLAG; GOMARK3 V50N16; every answer ENDEXT. */
static agc_status_t ags_end(const agc_job_ctx_t *ctx)
{
    agc_flag_clear(AGC_FLAG_NODOFLAG);

    return (align_flash(
                ctx, AGC_DSKY_GOMARK3, VN_AGS_DONE,
                AGC_JOB_AGS_INITIALIZATION_END,
                AGC_JOB_AGS_INITIALIZATION_END,
                AGC_JOB_AGS_INITIALIZATION_END));
}

/*
 * CKSTALL: wait (0.1 s at a time) while the IMU is otherwise busy
 * (IMUCADR non-zero), then IMUZERO and IMUSTALL; either end goes to
 * AGSEND.
 */
agc_status_t agc_job_ags_initialization_zero_imu(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_erase_imucadr() != 0)
    {
        return (agc_svc_delayjob(DT_RETRY,
                                 AGC_JOB_AGS_INITIALIZATION_ZERO_IMU));
    }

    agc_status_t st = agc_imu_zero(); /* IMUZERO  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_imu_stall_result_t r = AGC_IMU_STALL_BAD;

    st = agc_imu_stall_dev(AGC_IMU_DEV_IMU,
                           AGC_JOB_AGS_INITIALIZATION_IMU_ZEROED, &r);

    if ((st != AGC_STATUS_OK) || (r == AGC_IMU_STALL_SLEEPING))
    {
        return (st);
    }

    return (ags_end(ctx));
}

/* After the 20 s: the old downlist (AGSWORD); IMUSE set -> AGSEND. */
agc_status_t agc_job_ags_initialization_sent(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_telem_select_list((int) agc_erase_agsword());

    if (agc_flag_test(AGC_FLAG_IMUSE))
    {
        return (ags_end(ctx));
    }

    return (agc_job_ags_initialization_zero_imu(ctx));
}

/* IMUSTALL woken (good or bad end): AGSEND. */
agc_status_t agc_job_ags_initialization_imu_zeroed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_imu_stall_outcome(AGC_IMU_DEV_IMU) !=
                    AGC_IMU_STALL_SLEEPING))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (ags_end(ctx));
}

/* ENDEXT (= ENDMARK): the extended verb is over.  NODOFLAG is left as it
 * is: AGSEND has already cleared it, and V34 on the first V06N16 comes
 * before AGSVCALC sets it.                                               */
agc_status_t agc_job_ags_initialization_end(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_endext());
}
