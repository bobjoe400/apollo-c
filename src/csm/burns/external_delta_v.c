/*
 * external_delta_v.c -- the Comanche-only end of P30/P31 (P30-P37.agc, Comanche055
 * pp. 636-638): REFTEST (+MGA), DISPMGA/COMPTGO (start the TTOGO clock),
 * DISP45 (V16N45) and END45. They use the Comanche flag TIMRFLAG, so they
 * are kept out of the shared src/guidance/external_delta_v.c.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/guidance/guidance.h"
#include "guid_pending.h"

#define GRP4 4

/* MARSDP OCT 00000, OCT 35100: 0.01 degree in the low word (rev 2^0).     */
static const agc_dp_t MARSDP = (agc_dp_t) 035100;

/* A flashing display whose terminate goes to P00 (GOTOPOOH).             */
static agc_status_t flash(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind,
                          uint8_t verb, uint8_t noun, agc_job_id_t proceed,
                          agc_job_id_t enter)
{
    agc_dsky_cont_t cont = {AGC_JOB_GOTOPOOH, proceed, enter};

    if (!AGC_ASSERT(proceed != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return agc_dsky_display(ctx, kind, agc_dsky_vn(verb, noun), &cont);
}

/* COMPTGO (CM p.636): start CLOKTASK (TTOGO), group 6.3 and group 4 type C
 * at PRIO13 continuing at DISP45.                                         */
agc_status_t agc_csm_comptgo(void)
{
    agc_guid_state()->csm.nvword1 = 0; /* NVWORD1 = 0           */
    agc_flag_set(AGC_FLAG_TIMRFLAG);
    agc_status_t st = agc_waitlist_call(1, AGC_TASK_CLOKTASK);

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_MARKS_TFI_MGA_DISPLAY,
                         AGC_TASK_NONE, (int16_t) AGC_PRIO13,
                         false, 0U};

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_TIMRFLAG)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_phase_change2((uint16_t) 040036, (uint16_t) 005024, &v);

    return AGC_STATUS_OK; /* 6.3, 4 C */
}

/* REFTEST (CM p.638): +MGA from DELVSIN if REFSMMAT is valid, else
 * -00001 (MARSDP: shows -.01 deg); DISPMGA: COMPTGO, then DISP45.        */
agc_status_t agc_job_external_delta_v_reftest(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
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
            return st;
        }
    }
    else
    {
        rz->plus_mga = -MARSDP; /* NOTSET: DCS MARSDP    */
    }

    agc_status_t st = agc_csm_comptgo();

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    return agc_job_marks_tfi_mga_display(ctx);
}

/* DISP45 (CM p.636): flash V16N45 (marks, TFI, +MGA) -- PROCEED: END45,
 * ENTER: redisplay; then P30PHSI: 4.1 and end of job.                    */
agc_status_t agc_job_marks_tfi_mga_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_status_t st =
        flash(ctx, AGC_DSKY_GOFLASHR, 16, 45, AGC_JOB_MARKS_TFI_MGA_DONE,
              AGC_JOB_MARKS_TFI_MGA_DISPLAY);

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    agc_phase_change(GRP4, (uint16_t) 000014);

    return AGC_STATUS_OK; /* P30PHSI               */
}

/* END45 (CM p.636) and the tails of P30 and P31 (p.637): TIMRFLAG off,
 * XDELVFLG set after P30 (external delta-V) and reset after P31 (aimpoint),
 * then P00.                                                               */
agc_status_t agc_job_marks_tfi_mga_done(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
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

    return agc_exec_gotopooh();
}
