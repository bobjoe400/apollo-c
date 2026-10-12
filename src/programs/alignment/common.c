/*
 * common.c -- storage for the alignment erasables and the small
 * helpers shared by every alignment and marking routine: word formats,
 * CDUTRIG/CALCSMSC glue, integration, displays, stalls and phase changes.
 *
 * Sources: P51-P53.agc (Comanche055 pp. 737-784, Luminary099 pp. 926-983),
 * RTB_OP_CODES.agc (CDULOGIC, 1STO2S) and POWERED_FLIGHT_SUBROUTINES.agc
 * (CDUTRIG) for the helpers this module needs locally.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"
#include "agc/programs/alignment.h"

#define CDU_WORD_MASK     077777u /* 15-bit CDU register                  */
#define CDU_MODULUS       0100000 /* one revolution in CDU counts         */
#define CDU_HALF_REV      16384   /* counts in half a revolution          */
#define CDU_TO_REVS_SHIFT 8192    /* 2^13: cdu count -> DP revs           */
#define WORD_SIGN         040000u /* sign bit of a 15-bit word            */
#define WORD_ONES_NEG     077777  /* 1's complement bias                  */

static agc_align_ram_t align_ram;

/* Alignment erasable block (ERASABLE_ASSIGNMENTS "ALIGNMENT STORAGE"). */
agc_align_ram_t *agc_align_ram(void)
{
    return (&align_ram);
}

/* Fresh start: clear the alignment erasables. */
void agc_align_ram_reset(void)
{
    const agc_align_ram_t zero = {.mkvac_index = -1};

    align_ram                      = zero;
    align_ram.qmin                 = AGC_JOB_NONE;
    align_ram.qmaj                 = AGC_JOB_NONE;
    align_ram.auto_optics_return   = AGC_JOB_NONE;
    align_ram.sighting_mark_return = AGC_JOB_NONE;
    align_ram.planet_ret           = AGC_JOB_NONE;
    align_ram.coarse_ret           = AGC_JOB_NONE;
}

/*
 * CDULOGIC (RTB_OP_CODES.agc, Luminary p. 1397) -- the SP 2's complement
 * CDU angle (half-revolutions) as a DP angle in revolutions.  The routine
 * itself is guid-powered's agc_guid_cdulogic(); this keeps the alignment
 * name used throughout the lane.
 */
agc_dp_t agc_align_cdulogic(uint16_t cdu)
{
    const agc_dp_t r = agc_guid_cdulogic((uint16_t) (cdu & CDU_WORD_MASK));

    return (r);
}

/*
 * 1STO2S / 1TO2SUB (RTB_OP_CODES.agc, Luminary p. 1397-1398) -- convert a
 * DP angle in revolutions to a SP 2's complement angle in half revolutions.
 * The low word is dropped (truncation toward zero, as the 1's complement
 * high word is), and a doubling overflow wraps by one revolution.
 * DEVIATION: local copy of the RTB op (owner sys-misc, not public yet).
 */
uint16_t agc_align_ones_to_twos(agc_dp_t revs)
{
    int32_t hv = revs / CDU_TO_REVS_SHIFT;

    if (!AGC_ASSERT((revs >= AGC_DP_MIN) && (revs <= AGC_DP_MAX)))
    {
        hv = 0;
    }

    if (hv >= CDU_HALF_REV)
    {
        hv -= CDU_MODULUS;
    }
    else if (hv < -CDU_HALF_REV)
    {
        hv += CDU_MODULUS;
    }
    else
    {
        /* no doubling overflow */
    }

    if (hv < 0)
    {
        hv += CDU_MODULUS;
    }

    return ((uint16_t) hv);
}

/*
 * CDUTRIG with explicit angles (POWERED_FLIGHT_SUBROUTINES CD*TR*GS):
 * the CDUSPOT order is Y, Z, X.
 */
agc_status_t agc_align_cdutrig(const uint16_t cdu_xyz[3], agc_cdutrig_t *t)
{
    uint16_t yzx[3];

    yzx[AGC_TRIG_X] = cdu_xyz[0];
    yzx[AGC_TRIG_Y] = cdu_xyz[1];
    yzx[AGC_TRIG_Z] = cdu_xyz[2];
    agc_guid_cdtrigs(t, yzx);

    return (AGC_STATUS_OK);
}

/*
 * CALCSMSC (P51-P53.agc, Comanche p. 763) on the SINCDU/COSCDU left by
 * CDUTRIG: the routine is the imu lane's agc_imu_calcsmsc (X, Y, Z order).
 */
agc_status_t agc_align_calcsmsc(agc_interp_t *ip, const agc_cdutrig_t *t, agc_mat_t *nb)
{
    const agc_vec_t s = agc_vec_make(t->sincdu[AGC_TRIG_X], t->sincdu[AGC_TRIG_Y], t->sincdu[AGC_TRIG_Z]);
    const agc_vec_t c = agc_vec_make(t->coscdu[AGC_TRIG_X], t->coscdu[AGC_TRIG_Y], t->coscdu[AGC_TRIG_Z]);

    return (agc_imu_calcsmsc(ip, s, c, nb));
}

/* The present CDUX, CDUY, CDUZ (RDCDUS reads them under INHINT). */
void align_read_cdus(uint16_t cdu[3])
{
    agc_inhint();
    cdu[0] = agc_erase_cdux();
    cdu[1] = agc_erase_cduy();
    cdu[2] = agc_erase_cduz();
    agc_relint();
}

/* CALL CDUTRIG; CALL CALCSMSC on the present CDUs. */
agc_status_t align_nb_present(agc_interp_t *ip, agc_cdutrig_t *t, agc_mat_t *nb)
{
    agc_guid_cdutrig(t);
    agc_status_t st = AGC_STATUS_OK;

    st = agc_align_calcsmsc(ip, t, nb);

    return (st);
}

/* CSMPREC / LEMPREC / CSMCONIC / LEMCONIC (INTEGRATION_INITIALIZATION). */
agc_status_t align_integ(agc_interp_t *ip, align_integ_kind_t kind, agc_dp_t tdec1, agc_job_id_t resume_job, agc_integ_out_t *out)
{
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();
    agc_status_t     st  = AGC_STATUS_BAD_ARG;

    switch (kind)
    {
        case ALIGN_CSMPREC:
            st = agc_integ_csmprec(ip, ctx, tdec1, resume_job);
            break;
        case ALIGN_LEMPREC:
            st = agc_integ_lemprec(ip, ctx, tdec1, resume_job);
            break;
        case ALIGN_CSMCONIC:
            st = agc_integ_csmconic(ip, ctx, tdec1, resume_job);
            break;
        case ALIGN_LEMCONIC:
            st = agc_integ_lemconic(ip, ctx, tdec1, resume_job);
            break;
        default:
            break;
    }

    if (st == AGC_STATUS_OK)
    {
        *out = ctx->out;
    }

    return (st);
}

/* GOFLASH / GOFLASHR / GOMARK.. / GOPERF1 / GOPERF4R with the returns. */
agc_status_t align_flash(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind, uint16_t vn, agc_job_id_t term, agc_job_id_t proceed, agc_job_id_t enter)
{
    agc_dsky_cont_t cont;

    cont.terminate = term;
    cont.proceed   = proceed;
    cont.enter     = enter;

    return (agc_dsky_display(ctx, kind, vn, &cont));
}

/* GODSP / GODSPR / GODSPRET / CLEANDSP / KLEENEX (no astronaut return). */
agc_status_t align_show(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind, uint16_t vn)
{
    return (agc_dsky_display(ctx, kind, vn, NULL));
}

/* The verb of the answer was V32 (MPAC = V32 at the third return). */
bool align_resp_recycle(void)
{
    uint8_t verb = 0;
    uint8_t noun = 0;

    agc_dsky_response(&verb, &noun);

    return (verb == 32u);
}

/* TC / GOTO another resumption point of the running job. */
agc_status_t align_goto(agc_job_id_t next)
{
    if (!AGC_ASSERT(next != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_status_t st = agc_exec_job_yield(next);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* IMUSTALL / OPTSTALL: TC CURTAINS on the bad return. */
agc_status_t align_stall(agc_imu_device_t dev, agc_job_id_t wake_as, bool *done)
{
    agc_imu_stall_result_t r  = AGC_IMU_STALL_BAD;
    agc_status_t           st = agc_imu_stall_dev(dev, wake_as, &r);

    *done = false;

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (r == AGC_IMU_STALL_BAD)
    {
        agc_curtains(); /* TC CURTAINS (alarm 217)  */
        const agc_status_t c = AGC_STATUS_ALARM;

        return ((c == AGC_STATUS_OK) ? AGC_STATUS_ALARM : c);
    }

    *done = (r == AGC_IMU_STALL_GOOD);

    return (AGC_STATUS_OK);
}

/* The outcome seen by the job woken from a stall. */
agc_status_t align_stall_outcome(agc_imu_device_t dev)
{
    const agc_imu_stall_result_t r = agc_imu_stall_outcome(dev);

    if (!AGC_ASSERT(r != AGC_IMU_STALL_SLEEPING))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (r != AGC_IMU_STALL_GOOD)
    {
        agc_curtains();
        const agc_status_t c = AGC_STATUS_ALARM;

        return ((c == AGC_STATUS_OK) ? AGC_STATUS_ALARM : c);
    }

    return (AGC_STATUS_OK);
}

/* TC PHASCHNG; OCT code (type C, job at the next instruction). */
void align_phase_job(uint16_t code, agc_job_id_t job, agc_prio_t prio)
{
    agc_phase_var_t v = {0};

    v.kind       = AGC_RESTART_JOB;
    v.job        = job;
    v.prio_or_dt = (int16_t) prio;
    agc_phase_change_var(code, &v);
}

/* LOADTIME (RTB): TIME2, TIME1, cs B28. */
agc_dp_t align_loadtime(void)
{
    const uint32_t t = agc_time_now_cs();

    const agc_dp_t d = (agc_dp_t) (t & (AGC_TIME_MODULUS_CS - 1u));

    return (d);
}

/* A raw 15-bit 1's complement word as a signed SP (-0 reads as 0). */
agc_sp_t align_word_to_sp(uint16_t w)
{
    const uint16_t m = (uint16_t) (w & CDU_WORD_MASK);

    if ((m & WORD_SIGN) != 0u)
    {
        return ((agc_sp_t) ((int32_t) m - WORD_ONES_NEG));
    }

    return ((agc_sp_t) m);
}

/* A signed SP as the raw 15-bit 1's complement word. */
uint16_t align_sp_to_word(agc_sp_t v)
{
    if (v < 0)
    {
        return ((uint16_t) ((int32_t) v + WORD_ONES_NEG));
    }

    return ((uint16_t) v);
}

/* DSPTEM1, DSPTEM1 +1 (pinball buffer) as a DP. */
agc_dp_t align_dsptem1(void)
{
    const agc_sp_t hi = align_word_to_sp(agc_dsky_dsptem(0));
    const agc_sp_t lo = align_word_to_sp(agc_dsky_dsptem(1));

    return (agc_dp_pack(hi, lo));
}

void align_set_dsptem1(agc_dp_t v)
{
    agc_dsky_set_dsptem(0, align_sp_to_word(agc_dp_high(v)));
    agc_dsky_set_dsptem(1, align_sp_to_word(agc_dp_low(v)));
}

/*
 * SET1/PDT (Comanche p. 769) with the GCOMP clear and DRIFTFLG set that
 * follow it at each call; NCOARSE (Luminary p. 951) is the same plus PIPA
 * zeroing, done by its caller.  1/PIPADT = TIME1.
 */
agc_status_t align_set1_pdt(void)
{
    const agc_dp_t zero3[3] = {0, 0, 0};

    agc_imu_set_pipa_interval(agc_erase_time1());
    agc_imu_gyro_buf_set(AGC_IMU_GBUF_GCOMP, zero3);
    agc_flag_set(AGC_FLAG_DRIFTFLG);

    return (AGC_STATUS_OK);
}

/* TC PHASCHNG; OCT code (type A, group 4). */
agc_status_t align_phase4(uint16_t code)
{
    agc_phase_change(code);

    return (AGC_STATUS_OK);
}

/*
 * NCOARSE (P51-P53.agc, Luminary p. 951) -- 1/PIPADT = TIME1, PIPAX, Y, Z
 * = -0 (0 here), GCOMP = 0, SET DRIFTFLG.
 */
agc_status_t align_ncoarse(void)
{
    agc_erase_set_pipax(0);
    agc_erase_set_pipay(0);
    agc_erase_set_pipaz(0);

    if (!AGC_ASSERT(agc_erase_pipax() == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_set1_pdt());
}

/* COARSE, second half: IMUFINE and its IMUSTALL. */
static agc_status_t coarse_fine(void)
{
    agc_status_t st = agc_imu_fine(); /* IMUFINE   */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    bool done = false;

    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_ALIGN_FINE_DONE, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st);
    }

    return (align_goto(agc_align_ram()->coarse_ret));
}

/* COARSE (Luminary p. 951): IMUCOARS to THETAD and its IMUSTALL. */
agc_status_t align_coarse(agc_job_id_t ret)
{
    agc_align_ram()->coarse_ret = ret;
    uint16_t thetad[3];

    for (int i = 0; i < 3; i++)
    {
        thetad[i] = agc_erase_thetad(i);
    }

    agc_status_t st = agc_imu_coarse(thetad); /* IMUCOARS  */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    bool done = false;

    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_ALIGN_COARSE_DONE, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st);
    }

    return (coarse_fine());
}

/* COARSE: woken after the IMUCOARS stall. */
agc_status_t agc_job_align_coarse_done(void)
{
    if (align_stall_outcome(AGC_IMU_DEV_IMU) != AGC_STATUS_OK)
    {
        return (AGC_STATUS_OK); /* CURTAINS   */
    }

    return (coarse_fine());
}

/* COARSE: woken after the IMUFINE stall; continue as the caller asked. */
agc_status_t agc_job_align_fine_done(void)
{
    if (align_stall_outcome(AGC_IMU_DEV_IMU) != AGC_STATUS_OK)
    {
        return (AGC_STATUS_OK); /* CURTAINS   */
    }

    return (align_goto(agc_align_ram()->coarse_ret));
}

/*
 * BOV +1 / BOVB to the next instruction: the overflow indicator is tested
 * (and so cleared) but either way the code goes on.
 */
void align_clear_ovf(agc_interp_t *ip)
{
    const bool was_set = agc_interp_test_ovf(ip);

    (void) was_set; /* the overflow is discarded    */
}
