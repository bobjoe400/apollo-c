/*
 * display.c -- the display subroutines shared by the rendezvous
 * targeting programs (P32-P35, P72-P75), both vehicles:
 *   VNPOOH     (P34-35_P74-75.agc p.484)  flash, TERM -> P00
 *   DISPLAYE   (p.483)                    V06N55 flash
 *   P3XORP7X   (p.484)
 *   VN1645     (p.481-482)  +MGA, COMPTGO, V16N45, KILCLOCK, N45PROC,
 *                           CLUPDATE
 *   S34/35.5   (p.480; LM NTARGCHK)       final-pass N81 / N59 displays
 *   DISDVLVC   (P32-P33_P72-P73.agc p.662)
 *   ALMXIT / 611 alarm flash (P32-P33 p.652, P34-35 p.464)
 *
 * The AGC subroutines above put the calling job to sleep on a flashing
 * display and return into the job when the crew answers. In C the job
 * ends at the display and is resumed as a continuation job (agc_jobs.h
 * convention); the continuation of a subroutine that "returns" is stored
 * in the rendezvous erasables (QSAVED / NORMEX / SUBEXIT), and a
 * continuation that falls back into its caller does so through
 * agc_exec_job_yield (the job keeps its core set and priority).
 */
#include "../navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"

/* PHASCHNG OCT 04024 (VN1645 p.482): group 4, phase 2, type A.          */
#define RZ_PHASE_04024 ((uint16_t) 04024)
/* HIGH9 (P3XORP7X): MODREG >= 64 decimal is a P7x program.             */
#define RZ_HIGH9_MODES 64

/* The display a VNPOOH redisplay (ENTER) repeats: VERBNOUN, and where
 * PROCEED continues (the return address of the VNPOOH call).          */
typedef struct rz_vnpooh_slot
{
    uint8_t      verb;
    uint8_t      noun;
    agc_job_id_t next;
} rz_vnpooh_slot_t;

static rz_vnpooh_slot_t vnpooh_slot = {6U, 55U, AGC_JOB_GOTOPOOH};

/* ---- P3XORP7X (p.484): MODREG & HIGH9 = 0 -> P3x ---------------------- */
bool rz_active_vehicle_program(void)
{
    agc_sp_t mm = agc_erase_modreg();

    if (!AGC_ASSERT(mm >= 0))
    {
        return (false);
    }

    return (mm < RZ_HIGH9_MODES);
}

/* Continue the running job at another entry (the AGC falls back into the
 * caller's code at the stored return address).                         */
agc_status_t rz_continue(agc_job_id_t next)
{
    if (!AGC_ASSERT(next != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_exec_job_yield(next);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* ---- VNPOOH (p.484): CAF VNxxxx, TC VNPOOH ----------------------------
 * GOFLASH; TERMINATE -> GOTOPOOH, PROCEED -> return (next), ENTER ->
 * "TC -5", the same display again.                                    */
agc_status_t rz_vnpooh(const agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun,
                       agc_job_id_t next)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(next != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    vnpooh_slot.verb = verb; /* TS VERBNOUN  */
    vnpooh_slot.noun = noun;
    vnpooh_slot.next = next;

    return (rz_flash(ctx, AGC_DSKY_GOFLASH, verb, noun, AGC_JOB_GOTOPOOH, next,
                     AGC_JOB_RENDZ_VNPOOH));
}

/* VNPOOH "TC -5": ENTER on a VNPOOH display shows it again. */
agc_status_t agc_job_rendz_vnpooh(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(vnpooh_slot.next != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_vnpooh(ctx, vnpooh_slot.verb, vnpooh_slot.noun,
                      vnpooh_slot.next));
}

/* ---- DISPLAYE (p.483): V06N55 (NN/ELEV/CENTANG), return = next -------- */
agc_status_t rz_displaye(const agc_job_ctx_t *ctx, agc_job_id_t next)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(next != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_vnpooh(ctx, 6U, 55U, next));
}

/* ---- TC ALARM / OCT code, CAF V05N09, GOFLASH (P32 ALMXIT, P33 and P34
 * 611): TERM -> P00, PROCEED -> proceed, V32 -> recycle (the alarm and
 * display again, or the program step the AGC returns to).              */
agc_status_t rz_alarm_flash(const agc_job_ctx_t *ctx, agc_alarm_code_t code,
                            agc_job_id_t proceed, agc_job_id_t recycle)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(code != 0U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = rz_alarm(code);

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_ALARM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_flash(ctx, AGC_DSKY_GOFLASH, 5U, 9U, AGC_JOB_GOTOPOOH, proceed,
                     recycle));
}

/* ---- VN1645 (p.481) -----------------------------------------------------
 * +MGA = -.01 deg (not computed); -.02 deg on the final pass; when
 * REFSMFLG is set and this is a P3x program, +MGA from DELVSIN (GET+MGA).
 * Then COMPTGO (TTOGO clock), QSAVED = return, DELAYJOB 1 s, V16N45.   */
agc_status_t rz_ttogo_display(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                              agc_rendz_t *rz, agc_job_id_t resume_as)
{
    if (!AGC_ASSERT(ctx != NULL && ip != NULL) || !AGC_ASSERT(rz != NULL) ||
        !AGC_ASSERT(resume_as != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz->plus_mga = RZ_DP_MINUS_01; /* MGA = -.01 */

    if (agc_flag_test(AGC_FLAG_FINALFLG))
    {
        rz->plus_mga = agc_dp_add(ip, RZ_DP_MINUS_01, RZ_DP_MINUS_01);

        if (agc_flag_test(AGC_FLAG_REFSMFLG) && rz_active_vehicle_program())
        {
            agc_mat_t    refsmmat = agc_erase_refsmmat();
            agc_status_t st       =
                agc_rendz_get_mga(ip, rz->delvsin, &refsmmat, &rz->plus_mga);

            if (!AGC_ASSERT(st == AGC_STATUS_OK))
            {
                return (AGC_STATUS_BAD_ARG);
            }
        }
    }

    agc_status_t st =
        rz_comptgo(AGC_JOB_RENDZ_TTOGO_DISPLAY); /* GET45: COMPTGO */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz->qsaved = resume_as; /* TS QSAVED      */

    return (rz_delayjob(RZ_1SEC_CS,
                        AGC_JOB_RENDZ_TTOGO_DISPLAY)); /* DELAYJOB 1SEC  */
}

/* VN1645 after DELAYJOB: V16N45 (TRKMKCNT, TTOGO, +MGA) flashing. */
agc_status_t agc_job_rendz_ttogo_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(rz_state()->qsaved != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_flash(ctx, AGC_DSKY_GOFLASH, 16U, 45U, AGC_JOB_RENDZ_KILCLOCK,
                     AGC_JOB_RENDZ_TTOGO_PROCEED, AGC_JOB_RENDZ_CLUPDATE));
}

/* KILCLOCK (p.481): DISPDEX = 0 (stop the clock display), GOTOPOOH. */
agc_status_t agc_job_rendz_kilclock(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_dispdex_zero();
    agc_status_t st = rz_gotopooh();

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_POODOO))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* CLUPDATE (p.482): DISPDEX = 0, PHASCHNG 04024, CLEAR UPDATFLG, return
 * to QSAVED for the next computation cycle.                            */
agc_status_t agc_job_rendz_clupdate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_dispdex_zero();
    agc_phase_change(4, RZ_PHASE_04024);
    agc_flag_clear(AGC_FLAG_UPDATFLG);

    return (rz_continue(rz_state()->qsaved));
}

/* N45PROC (p.482): FINALFLG already set -> KILCLOCK (await a new
 * program); otherwise PHASCHNG 04024, set FINALFLG, CLUPDATE.          */
agc_status_t agc_job_rendz_ttogo_proceed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_flag_test(AGC_FLAG_FINALFLG))
    {
        return (agc_job_rendz_kilclock(ctx));
    }

    agc_phase_change(4, RZ_PHASE_04024);
    agc_flag_set(AGC_FLAG_FINALFLG); /* UPFLAG     */

    return (agc_job_rendz_clupdate(ctx));
}

/* ---- DISDVLVC (P32-P33 p.662) -------------------------------------------
 * DELVLVC = dv; M = S32/33.X; DELVLVC = M DELVLVC (VSL1); VNPOOH with the
 * verb/noun the caller left in X1 (V06N81 or V06N82); return = next.   */
agc_status_t rz_disdvlvc(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                         agc_rendz_t *rz, agc_vec_t dv, uint8_t noun,
                         agc_job_id_t next)
{
    if (!AGC_ASSERT(ctx != NULL && ip != NULL) || !AGC_ASSERT(rz != NULL) ||
        !AGC_ASSERT(noun == 81U || noun == 82U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz->delvlvc = dv; /* STORE      */
    agc_mat_t    m;
    agc_status_t st = agc_rendz_csi_cdh_lv_matrix(ip, rz, &m);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->delvlvc = agc_vec_shl(ip, agc_mat_vec(ip, &m, rz->delvlvc), 1);

    return (rz_vnpooh(ctx, 6U, noun, next));
}

/* ---- S34/35.5 (p.480) ---------------------------------------------------
 * Not the final pass: set UPDATFLG, go to FLAGOFF. Final pass: save the
 * computed DELVLVC (CM in DVLOS, LM in GDT/2 after clearing NTARGFLG)
 * and show V06N81 for a crew overwrite.                                 */
agc_status_t rz_tpi_final_displays(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                                   agc_rendz_t *rz, agc_job_id_t resume_as)
{
    if (!AGC_ASSERT(ctx != NULL && ip != NULL) || !AGC_ASSERT(rz != NULL) ||
        !AGC_ASSERT(resume_as != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz->normex = resume_as; /* STQ SUBEXIT  */

    if (!agc_flag_test(AGC_FLAG_FINALFLG))
    {
        agc_flag_set(AGC_FLAG_UPDATFLG);

        return (rz_tpi_flagoff(ctx, ip, rz));
    }

    if (rz_is_csm())
    {
        rz->dvlos = rz->delvlvc; /* FLAGON (CM)  */
    }
    else
    {
        rz_vflag_set(RZ_VF_NTARGFLG, false); /* FLAGON (LM)  */
        rz->gdt_2 = rz->delvlvc;
    }

    return (rz_vnpooh(ctx, 6U, 81U, AGC_JOB_RENDZ_FINAL_DV_CHECKED));
}

/* LM NTARGCHK: add the differences of the six words of DELVLVC and its
 * saved copy GDT/2. DEVIATION: the AGC sums ones' complement SP words
 * with ADS (overflow wraps); here the sum is exact, which only differs
 * when the word differences overflow 14 bits and cancel.              */
static bool rz_ntargchk(const agc_rendz_t *rz)
{
    int32_t sum = 0;

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        sum += (int32_t) agc_dp_high(rz->gdt_2.c[i]) -
               (int32_t) agc_dp_high(rz->delvlvc.c[i]);
        sum += (int32_t) agc_dp_low(rz->gdt_2.c[i]) -
               (int32_t) agc_dp_low(rz->delvlvc.c[i]);
    }

    if (!AGC_ASSERT(sum >= -(6 * 2 * 0x4000) && sum <= 6 * 2 * 0x4000))
    {
        return (false);
    }

    return (sum != 0);
}

/* S34/35.5 after PROCEED on V06N81: test for a crew overwrite of the
 * computed delta V (CM: |DELVLVC - DVLOS| non-zero; LM: NTARGCHK sets
 * NTARGFLG); an overwrite is processed by S34/35.3. NOCHG: CLEAR
 * XDELVFLG, DELVSIN = DELVEET3. FLAGOFF follows.                       */
agc_status_t agc_job_rendz_final_dv_checked(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_rendz_t *rz = rz_state();
    bool         overwrite;

    if (rz_is_csm())
    {
        overwrite =
            rz_abval(&ip, agc_vec_sub(&ip, rz->delvlvc, rz->dvlos)) != 0;
    }
    else
    {
        overwrite = rz_ntargchk(rz);

        if (overwrite)
        {
            rz_vflag_set(RZ_VF_NTARGFLG, true); /* UPFLAG       */
        }
    }

    if (overwrite)
    {
        agc_status_t st = agc_rendz_tpi_crew_overwrite(
            &ip, rz, AGC_JOB_RENDZ_FINAL_DV_CHECKED);

        if (st != AGC_STATUS_OK)
        {
            return (st); /* INTSTALL sleep: redo step */
        }
    }

    agc_flag_clear(AGC_FLAG_XDELVFLG); /* NOCHG     */
    rz->delvsin = rz->delveet3;

    return (rz_tpi_flagoff(ctx, &ip, rz));
}

/* FLAGOFF: S34/35.4 (DVLOS from DELVEET3), V06N59, return to SUBEXIT. */
agc_status_t rz_tpi_flagoff(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                            agc_rendz_t *rz)
{
    if (!AGC_ASSERT(ctx != NULL && ip != NULL) || !AGC_ASSERT(rz != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_rendz_tpi_los_delta_v(ip, rz);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_vnpooh(ctx, 6U, 59U, rz->normex));
}

/* VN1645 as a public entry (return to earth, guid-entry): the same as
 * rz_ttogo_display with the rendezvous erasables.                        */
agc_status_t agc_rendz_ttogo_display_start(const agc_job_ctx_t *ctx,
                                           agc_interp_t        *ip,
                                           agc_job_id_t         resume_as)
{
    if (!AGC_ASSERT(ctx != NULL && ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_ttogo_display(ctx, ip, rz_state(), resume_as));
}
