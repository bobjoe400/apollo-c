/*
 * sighting_mark.c -- the CSM sighting routines of P51-P53.agc:
 *   R53  sighting mark (Comanche pp. 771-772): SXTMARK + OPTSTALL, then
 *        the star code (V01N71) is checked (CHKSCODE) and kept in BESTI.
 *   R56  alternate LOS sighting mark (Comanche pp. 781-782): a backup
 *        mark from keyed-in optics angles (V06N94, V53) when the optics
 *        cannot be used (P53, P54).
 *   SXTSM (Comanche p. 770): the line of sight of the first mark in
 *        stable-member axes.
 * Both R53 and R56 are interpretive subroutines entered with STQ R53EXIT;
 * here the caller names the job to continue as, kept in
 * agc_align_ram()->sighting_mark_return, and the routine ends with
 * TERMIFLG set (R53OUT) so that R52 stops driving the optics.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

#define VN_STAR_CODE    ALIGN_VN(1, 71)  /* V01N71 VN 0171: star code      */
#define VN_ALT_ANGLES   ALIGN_VN(6, 94)  /* V06N94B VN 00694: optics angles */
#define VN_ALT_MARK     ALIGN_VN(53, 0)  /* VB53 VN 05300: alternate mark  */
#define CHK_MARK_DONE   ((uint16_t) 016) /* OCT16: checklist "mark done"   */
#define STAR_BITS       ((agc_sp_t) 077) /* CS HIGH9: star bits 1-6        */
#define SIGHTSIX        ((agc_sp_t) 6)   /* SIGHTSIX = SIX                 */
#define MARKS_MASK      ((agc_sp_t) 07)  /* LOW3: number of marks          */
#define MARK_WORDS      7                /* MKT2T1 .. MKCDUX               */
#define MRKBUF_CDUS     3                /* MRKBUF1 +3 (N94 shaft)         */
#define MRKBUF_CDUT     5                /* MRKBUF1 +5 (N94 trunnion)      */
#define MM_ORBITAL_NAV  ((agc_sp_t) 22)  /* P22                            */
#define MM_CISLUNAR_NAV ((agc_sp_t) 23)  /* P23                            */

/* R53A: SXTMARK for MARKINDX marks, then OPTSTALL (waking at R53A +5). */
static agc_status_t sighting_mark_request(void)
{
    const agc_sp_t n = (agc_sp_t) (agc_align_ram()->markindx & MARKS_MASK);

    if (!AGC_ASSERT((n >= 0) && (n <= MARKS_MASK)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_sxtmark(n);

    if (st != AGC_STATUS_OK)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* BAILOUT */
    }

    bool done = false;

    st = align_stall(AGC_IMU_DEV_OPTICS, AGC_JOB_SIGHTING_MARK_MARKED, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* CURTAINS */
    }

    return (align_goto(AGC_JOB_SIGHTING_MARK_MARKED));
}

/*
 * R53 (Comanche p. 771): STQ R53EXIT, SET R53FLAG, then R53A.
 */
agc_status_t agc_csm_sighting_mark(const agc_job_ctx_t *ctx, agc_job_id_t ret)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(ret != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram()->sighting_mark_return = ret; /* R53EXIT  */
    agc_flag_set(AGC_FLAG_SIGHTING_MARK_REQUESTED);

    return (sighting_mark_request());
}

/* R53D / R53OUT: SET TERMIFLG (R52 stops), GOTO R53EXIT. */
static agc_status_t sighting_mark_done(void)
{
    agc_flag_set(AGC_FLAG_TERMIFLG);

    return (align_goto(agc_align_ram()->sighting_mark_return));
}

/*
 * R53A after OPTSTALL: QPRET (marks made) > 0 goes on to R53B; with no
 * mark the VAC area and MARKSTAT are released and SXTMARK is asked again.
 * R53B: P22 and P23 need no star code (R53D); otherwise R53C1.
 */
agc_status_t agc_job_sighting_mark_marked(agc_job_ctx_t *ctx)
{
    if (align_stall_outcome(AGC_IMU_DEV_OPTICS) != AGC_STATUS_OK)
    {
        return (AGC_STATUS_OK); /* CURTAINS */
    }

    if (agc_sxtmark_marks_made() <= 0)
    {
        const agc_status_t st = agc_sxtmark_free_vac(); /* XCH MARKSTAT */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        return (sighting_mark_request()); /* TCF R53A */
    }

    if (agc_phase_checkmm(MM_ORBITAL_NAV) || agc_phase_checkmm(MM_CISLUNAR_NAV))
    {
        return (sighting_mark_done()); /* R53D     */
    }

    return (agc_job_sighting_mark_code_clean(ctx));
}

/* R53C1: CAF ZERO, CLEANDSP; the job continues at R53C. */
agc_status_t agc_job_sighting_mark_code_clean(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_align_ram()->sighting_mark_return != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_CLEANDSP, 0, AGC_JOB_SIGHTING_MARK_CODE_DISPLAY, AGC_JOB_SIGHTING_MARK_CODE_DISPLAY, AGC_JOB_SIGHTING_MARK_CODE_DISPLAY));
}

/* R53C: flash V01N71; V34 terminates, V33 R53Z, ENTER re-flashes. */
agc_status_t agc_job_sighting_mark_code_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_align_ram()->sighting_mark_return != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_STAR_CODE, AGC_JOB_GOTOPOOH, AGC_JOB_SIGHTING_MARK_CODE_ENTERED, AGC_JOB_SIGHTING_MARK_CODE_DISPLAY));
}

/*
 * R53Z: CHKSCODE; a bad code lights OPR ERR and shows V01N71 again.  A
 * good one: BESTI +STARIND = 6 x (STARCODE bits 1-6), then R53D.
 */
agc_status_t agc_job_sighting_mark_code_entered(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram();

    if (!agc_align_chkscode(ram->starcode))
    {
        agc_dsky_falton(); /* FALTON   */

        return (agc_job_sighting_mark_code_display(ctx));
    }

    const agc_sp_t star6 = (agc_sp_t) ((ram->starcode & STAR_BITS) * SIGHTSIX);

    if (!AGC_ASSERT((ram->starind == 0) || (ram->starind == 1)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (ram->starind == 0)
    {
        ram->besti = star6; /* INDEX STARIND */
    }
    else
    {
        ram->bestj = star6;
    }

    return (sighting_mark_done());
}

/*
 * R56 (Comanche p. 781): STQ R53EXIT, then flash V06N94 (the optics
 * angles of the alternate line of sight).
 */
agc_status_t agc_csm_alternate_los_sighting_mark(const agc_job_ctx_t *ctx, agc_job_id_t ret)
{
    agc_align_ram()->sighting_mark_return = ret; /* R53EXIT  */

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_ALT_ANGLES, AGC_JOB_GOTOPOOH, AGC_JOB_ALTERNATE_LOS_RESERVE, AGC_JOB_ALTERNATE_LOS_ANGLES_DISPLAY));
}

/* R56 ENTER on V06N94: new angles, flash again. */
agc_status_t agc_job_alternate_los_angles_display(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_ALT_ANGLES, AGC_JOB_GOTOPOOH, AGC_JOB_ALTERNATE_LOS_RESERVE, AGC_JOB_ALTERNATE_LOS_ANGLES_DISPLAY));
}

/* R56A +2: CLEANDSP, continuing at R56A1. */
static agc_status_t alternate_los_clean(const agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_align_ram()->sighting_mark_return != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_CLEANDSP, 0, AGC_JOB_ALTERNATE_LOS_MARK_REQUEST, AGC_JOB_ALTERNATE_LOS_MARK_REQUEST, AGC_JOB_ALTERNATE_LOS_MARK_REQUEST));
}

/* R56A (V33 on V06N94): SXTMARK +2 (marks system and a VAC area only). */
agc_status_t agc_job_alternate_los_reserve(agc_job_ctx_t *ctx)
{
    const agc_status_t st = agc_sxtmark_reserve();

    if (st != AGC_STATUS_OK)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* BAILOUT */
    }

    return (alternate_los_clean(ctx));
}

/* R56A1: GOMARK2 VB53 (please make the alternate mark); V33 shows it
 * again, ENTER is the mark.                                              */
agc_status_t agc_job_alternate_los_mark_request(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_align_ram()->sighting_mark_return != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_GOMARK2, VN_ALT_MARK, AGC_JOB_GOTOPOOH, AGC_JOB_ALTERNATE_LOS_MARK_REQUEST, AGC_JOB_ALTERNATE_LOS_MARKED));
}

/*
 * R56A1 ENTER (a backup system mark): SAC = MRKBUF1 +3, PAC = MRKBUF1 +5
 * (the N94 angles); under INHINT the mark slot gets TIME2/1, CDUY, SAC,
 * CDUZ, PAC, CDUX.  CLEARMRK, then V50N25 R1 00016: PROCEED goes to R53C1
 * (star code), ENTER marks again (R56A +2).
 */
agc_status_t agc_job_alternate_los_marked(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram();

    ram->sac = agc_erase_mrkbuf(1, MRKBUF_CDUS);
    ram->pac = agc_erase_mrkbuf(1, MRKBUF_CDUT);
    agc_inhint();
    agc_align_sxt_mark_t m;

    m.t             = align_loadtime();
    m.cduy          = agc_erase_cduy();
    m.cdus          = ram->sac;
    m.cduz          = agc_erase_cduz();
    m.cdut          = ram->pac;
    m.cdux          = agc_erase_cdux();
    ram->sxt[0]     = m; /* INDEX MARKSTAT TS */
    agc_relint();
    agc_dsky_clearmrk(); /* CLEARMRK */
    agc_status_t st = align_flash(ctx, AGC_DSKY_GOPERF1, CHK_MARK_DONE, AGC_JOB_GOTOPOOH, AGC_JOB_SIGHTING_MARK_CODE_CLEAN, AGC_JOB_ALTERNATE_LOS_REMARK);

    return (st);
}

/* R56A +2 (V32 on V50N25: another mark, like a reject). */
agc_status_t agc_job_alternate_los_remark(agc_job_ctx_t *ctx)
{
    return (alternate_los_clean(ctx));
}

/*
 * SXTSM line of sight (Comanche p. 770): SXTNB of the mark's trunnion and
 * shaft CDUs (half-unit, nav base), then NBSM with the mark's IMU CDUs
 * (CDUY, CDUZ, CDUX at words 2, 4, 6 of the mark): half-unit, SM axes.
 */
agc_status_t agc_csm_sxtsm(agc_interp_t *ip, const agc_align_sxt_mark_t *m, agc_vec_t *los_sm)
{
    agc_vec_t    v;
    agc_status_t st = agc_csm_sxtnb(ip, m->cdut, m->cdus, &v); /* SXTNB */

    if (st == AGC_STATUS_OK)
    {
        const uint16_t yzx[3] = {m->cduy, m->cduz, m->cdux};
        agc_cdutrig_t  t;

        st = agc_guid_trg_nbsm(&t, yzx, &v); /* NBSM  */

        if (st == AGC_STATUS_OK)
        {
            *los_sm = v;
        }
    }

    return (st);
}

/* The mark as the 7 downlink words (MKT2T1 .. MKCDUX). */
static void mark_words(const agc_align_sxt_mark_t *m, uint16_t w[MARK_WORDS])
{
    w[0] = align_sp_to_word(agc_dp_high(m->t));
    w[1] = align_sp_to_word(agc_dp_low(m->t));
    w[2] = m->cduy;
    w[3] = m->cdus;
    w[4] = m->cduz;
    w[5] = m->cdut;
    w[6] = m->cdux;
}

/*
 * SXTSM (Comanche p. 770) as called: TSIGHT = time of the first mark; the
 * 7 mark words go to MARKDOWN (STARIND 0) or MARK2DWN (STARIND 1) for the
 * downlink; the line of sight in SM axes.
 */
agc_status_t agc_csm_sxtsm_marks(agc_interp_t *ip, agc_vec_t *los_sm)
{
    agc_align_ram_t           *ram = agc_align_ram();
    const agc_align_sxt_mark_t m   = ram->sxt[0];

    ram->tsight = m.t; /* TSIGHT   */
    uint16_t w[MARK_WORDS];

    mark_words(&m, w);

    for (int i = 0; i < MARK_WORDS; i++)
    {
        if (ram->starind == 0)
        {
            agc_erase_set_markdown(i, w[i]);
        }
        else
        {
            agc_erase_set_mark2dwn(i, w[i]);
        }
    }

    return (agc_csm_sxtsm(ip, &m, los_sm));
}
