/**********************************************************************
 * FILE NAME: lunar_landing.c
 *
 * PURPOSE:
 *   The landing program flow.
 *
 *     THE_LUNAR_LANDING.agc (Luminary 099, pp. 785-792): P63 braking phase
 *       start, the ignition algorithm IGNALG, R51/R60 before ignition, and
 *       P68 landing confirmation.
 *     LUNAR_LANDING_GUIDANCE_EQUATIONS.agc (pp. 800-822): GUILDENSTERN (R13,
 *       the auto-modes monitor), the P66/P67 starts, the landing site
 *       redesignation equations, P66 rate-of-descent guidance (RODCOMP), P67,
 *       the redesignator trap (PITFALL/REDESMON) and the R.O.D. trap.
 *
 *   Jobs that wait for the crew, the integration lock or another routine
 *   are split at those points; each piece is a job function and the AGC's
 *   return address becomes the continuation job id (TRANSLATION_GUIDE s.4).
 *
 * AGC SOURCE:
 *   Luminary099 THE_LUNAR_LANDING.agc, pp. 785-792;
 *   LUNAR_LANDING_GUIDANCE_EQUATIONS.agc, pp. 800-822
 *
 * EXTERNAL VARIABLES:
 *   agc_lm_ldg(), agc_lm_pad(), agc_lm_thr() (landing, pad-load and
 *   throttle erasables); agc_guid_state() (R, V, GDT/2, MASS,
 *   HDOTDISP); agc_integ_shared_ctx() (IGNALG integration)
 *
 * EXTERNAL REFERENCES:
 *   agc_lm_guidpass, agc_lm_throttle_measured, agc_lm_dispexit,
 *   agc_integ_lemprec, agc_integ_integrvs, agc_integ_install,
 *   agc_waitlist_call, agc_phase_newmode, agc_dsky_display,
 *   agc_latlong_from_r, agc_planet_r_to_rp, agc_align_refmf
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   alarm 01412 (IGNALG did not converge in 40 passes: NIGNLOOP
 *   exhausted); IGNALOOP is bounded at 64 passes (AGC: unbounded),
 *   then NO_CONVERGE
 **********************************************************************/
#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

/* ---- constants (pp. 789, 820-822) -------------------------------------- */
#define LL_DPSTHRSH      36                    /* DPSTHRSH: THRESH1 + THRESH3 for P63     */
#define LL_ASTNDEX       025                   /* ASTNDEX = MD1: CLOKTASK display index   */
#define LL_CODE500       00500                 /* CODE500: crank the LR antenna           */
#define LL_99999CON      ((agc_dp_t) 487675)   /* 2DEC 30479.7 B-24: 99999 ft */
#define LL_GUIDDURN      ((agc_dp_t) 66440)    /* 2DEC +66440: 664.4 s, cs 2^28 */
#define LL_DDUMCRIT      ((agc_dp_t) 8)        /* 2DEC +8 B-28: IGNALG converged  */
#define LL_NIGNLOOP      40                    /* SSP NIGNLOOP 40D                        */
#define LL_BIASFACT      ((agc_dp_t) 655)      /* 2DEC 655.36 B-28                */
#define LL_KPIP1         ((agc_dp_t) 3435974)  /* 2DEC .0128: DELV to 2^7 m/cs */
#define LL_4SEC28        ((agc_dp_t) 400)      /* 2DEC 400 B-28                   */
#define LL_GSCALE        ((agc_dp_t) 13107200) /* 2DEC 100 B-11 */
#define LL_SHFTFACT      ((agc_dp_t) 2048)     /* 2DEC 1 B-17                     */
#define LL_SCALEFAC      ((agc_dp_t) 3268444)  /* +7.97959872 E+2 B-16 */
#define LL_DEPRCRIT      ((agc_dp_t) -2684355) /* 2DEC -.02 B-1 */
#define LL_ALL4BITS      00063                 /* ALL4BITS: channel 31 bits 1, 2, 5, 6    */
#define LL_AZEACH        572                   /* DEC .03491: 2 degrees                   */
#define LL_ELEACH        143                   /* DEC .00873: 1/2 degree                  */
#define LL_PLUSELBIT     00002                 /* +ELBIT = BIT2 (-pitch)                  */
#define LL_MINUSELBIT    00001                 /* -ELBIT = BIT1 (+pitch)                  */
#define LL_PLUSAZBIT     00020                 /* +AZBIT = BIT5                           */
#define LL_MINUSAZBIT    00040                 /* -AZBIT = BIT6                           */
#define LL_1SEC          100                   /* 1SEC: 100 cs                            */
#define LL_PRIO22        022                   /* PRIO22: RODCOMP job priority            */
#define LL_IGN_MAX_LOOPS 64                    /* bound on IGNALOOP (AGC: none)           */

/**********************************************************************
 * PHASCHNG OCT 04024 (THE_LUNAR_LANDING.agc pp. 785-792): group 4
 * restarts at this point of this job
 *
 * ARGUMENTS:
 *   job  agc_job_id_t  I    job to restart at
 **********************************************************************/
static void ll_restart_here(agc_job_id_t job)
{
    const agc_phase_var_t v = {AGC_RESTART_JOB, job, AGC_TASK_NONE, 0, false, 0U};

    agc_phase_change_var(04024, &v);
}

/**********************************************************************
 * GOFLASH/GOPERF1 requests of THE_LUNAR_LANDING.agc (pp. 788-792) --
 * the crew display continuations shared by this file
 *
 * ARGUMENTS:
 *   ctx      const agc_job_ctx_t *  I    executive job context
 *   kind     agc_dsky_disp_t        I    GOFLASH, GOPERF1 or GODSPR request
 *   vn       uint16_t               I    verb-noun (or option code) to show
 *   proceed  agc_job_id_t           I    job on PROCEED
 *   enter    agc_job_id_t           I    job on ENTER (recycle)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t ll_display(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind, uint16_t vn, agc_job_id_t proceed, agc_job_id_t enter)
{
    const agc_dsky_cont_t cont = {AGC_JOB_GOTOPOOH, proceed, enter};

    return (agc_dsky_display(ctx, kind, vn, &cont));
}

/* ======================================================================
 * P63 and the ignition algorithm (THE_LUNAR_LANDING, pp. 785-789)
 * ====================================================================== */

/* IGNALOOP, defined below (IGNALG returns to it). */
static agc_status_t ll_ignaloop(agc_job_ctx_t *ctx, bool from_install);

/**********************************************************************
 * IGNALG, first part (pp. 785-786): the landing site in stable member
 * coordinates at TLAND, WM and /LAND/, and the LM state integrated to
 * TLAND - GUIDDURN (LEMPREC; may wait for the integration lock).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *   ip   agc_interp_t *   I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t ll_ignalg_prec(agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_ldg_t *l     = agc_lm_ldg();                                                              /* landing erasables */
    agc_dp_t      tdec1 = agc_dp_sub(ip, agc_lm_pad()->tland, LL_GUIDDURN);                          /* TLAND - GUIDDURN, cs 2^28 */
    agc_status_t  st    = agc_integ_lemprec(ip, agc_integ_shared_ctx(), tdec1, AGC_JOB_IGNALG_PREC); /* step status */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* asleep; resumes as IGNALGP    */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    l->nignloop = LL_NIGNLOOP;
    l->cg       = agc_mat_identity_half(); /* UNITX, UNITY, UNITZ           */
    l->deltah   = LL_99999CON;             /* for the V16N68 display        */
    l->unfc_2   = agc_vec_zero();          /* trim velocity correction term */
    l->ttf_8    = 0;
    l->ttf_8_hi = 0;

    return (ll_ignaloop(ctx, false));
}

/**********************************************************************
 * IGNALOOP .. EXGSUB (pp. 786-787, 811-812): one IGNALOOP iteration up
 * to DDUMCALC: state from the integration
 * output, gravity, three passes of quadratic guidance (?GUIDSUB, then
 * GUIDSUB while NGUIDSUB counts down), NIGNLOOP.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *   ip   agc_interp_t *   I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t ll_ign_guidance(agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_ldg_t          *l = agc_lm_ldg();                 /* landing erasables */
    agc_guid_t            *g = agc_guid_state();             /* guidance state */
    const agc_integ_out_t *o = &agc_integ_shared_ctx()->out; /* integration output (RATT1, VATT1, TAT) */

    g->avg.piptime1 = o->tat;
    agc_mat_t m     = agc_erase_refsmmat(); /* REFSMMAT, half unit */

    g->lm.r         = agc_vec_shl(ip, agc_mat_vec(ip, &m, o->ratt1), 4);
    g->avg.gdt_2    = agc_lm_mungrav(ip, g->lm.r);
    l->nguidsub     = 2;                                           /* N = 3        */
    agc_status_t st = agc_lm_guidpass(ctx, AGC_LM_ENTRY_QGUIDSUB); /* step status */

    for (int n = 0; n < 2 && st == AGC_STATUS_OK && l->nguidsub > 0; n++)
    {
        l->nguidsub = (agc_sp_t) (l->nguidsub - 1); /* CCS NGUIDSUB */
        st          = agc_lm_guidpass(ctx, AGC_LM_ENTRY_GUIDSUB);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (l->nignloop > 0)
    {
        l->nignloop = (agc_sp_t) (l->nignloop - 1); /* CCS NIGNLOOP */
    }
    else
    {
        agc_alarm(AGC_ALARM_01412); /* DEVIATION: NIGNLOOP stays 0 */
    }

    if (!AGC_ASSERT(l->nignloop >= 0 && l->nignloop < LL_NIGNLOOP))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
} /* ll_ign_guidance */

/**********************************************************************
 * IGNALOOP (pp. 786-788) with DDUMCALC; on convergence DDUMGOOD/R60INIT
 * and IGNALGRT.
 *
 * ARGUMENTS:
 *   ctx           agc_job_ctx_t *  I    executive job context
 *   from_install  bool             I    resuming after INTSTALL was busy
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t ll_ignaloop(agc_job_ctx_t *ctx, bool from_install)
{
    agc_interp_t ip; /* interpreter state (OVFIND) */

    agc_interp_init(&ip);
    agc_integ_ctx_t *ic     = agc_integ_shared_ctx(); /* shared integration context */
    bool             resume = from_install;           /* skip guidance: resume after INTSTALL */

    for (int n = 0; n < LL_IGN_MAX_LOOPS; n++)
    {
        if (!resume)
        {
            agc_status_t st = ll_ign_guidance(ctx, &ip); /* step status */

            if (st != AGC_STATUS_OK)
            {
                return (st);
            }

            /* DDUMCALC (pp. 786-787): the ignition time correction
             *          (RIGNZ - RGU2)/16 + 16 RGU1^2 KIGNY/B8 + (RGU0 - RIGNX) KIGNX/B4
             *          + (|VGU| - VIGN) KIGNV/B4
             *   DDUM = ----------------------------------------------------------
             *                     2^10 (VGU2 - 16 VGU0 KIGNX/B4)
             * numerator 2^28 m, denominator 2^10 m/cs, quotient rescaled to 2^28 cs. */
            const agc_lm_ldg_t *l_ddum = agc_lm_ldg();                                                             /* landing erasables */
            const agc_lm_pad_t *p      = agc_lm_pad();                                                             /* pad loads */
            agc_dp_t            den    = agc_dp_shl_round(&ip, agc_dp_mul(&ip, l_ddum->vgu.c[0], p->kignx_b4), 4); /* VGU2 - 16 VGU0 KIGNX/B4, 2^10 m/cs */

            den           = agc_dp_sub(&ip, l_ddum->vgu.c[2], den);
            agc_dp_t a    = agc_dp_shr_round(agc_dp_sub(&ip, p->rignz, l_ddum->rgu.c[2]), 4);                        /* (RIGNZ - RGU2)/16, 2^28 m */
            agc_dp_t b    = agc_dp_shl_round(&ip, agc_dp_mul(&ip, agc_dp_sq(l_ddum->rgu.c[1]), p->kigny_b8), 4);     /* 16 RGU1^2 KIGNY/B8, 2^28 m */
            agc_dp_t c    = agc_dp_mul(&ip, agc_dp_sub(&ip, l_ddum->rgu.c[0], p->rignx), p->kignx_b4);               /* (RGU0 - RIGNX) KIGNX/B4, 2^28 m */
            agc_dp_t d    = agc_dp_mul(&ip, agc_dp_sub(&ip, agc_vec_abval(&ip, l_ddum->vgu), p->vign), p->kignv_b4); /* (|VGU| - VIGN) KIGNV/B4, 2^28 m */
            agc_dp_t num  = agc_dp_add(&ip, agc_dp_add(&ip, agc_dp_add(&ip, d, c), b), a);                           /* numerator, 2^28 m */
            agc_dp_t ddum = agc_dp_shr_round(agc_dp_div(&ip, num, den), 10);                                         /* DDUM, cs 2^28 */

            /* DDUM, cs 2^28 */

            ic->tdec1 = agc_dp_add(&ip, ddum, agc_guid_state()->avg.piptime1);

            if (agc_dp_abs(ddum) < LL_DDUMCRIT)
            {
                /* DDUMGOOD and R60INIT (p. 788): TIG = TDEC1 - ZOOMTIME; the landing
                 * site's distance out of the orbital plane (+ right/north); UNFC/2 kept
                 * in R60VSAVE for the attitude maneuver. */
                agc_dp_t tdec1 = ic->tdec1;

                agc_lm_ldg_t        *l  = agc_lm_ldg();                    /* landing erasables */
                const agc_guid_lm_t *g  = &agc_guid_state()->lm;           /* LM guidance state */
                agc_dp_t             zt = (agc_dp_t) agc_erase_zoomtime(); /* SLOAD ZOOMTIME, SR 14D: cs 2^28 */

                agc_erase_set_tig(agc_dp_sub(&ip, tdec1, zt));
                agc_vec_t n_ddumgood = agc_vec_unit(&ip, agc_vec_cross(&ip, g->v, g->r), NULL, NULL); /* UNIT(V x R), half unit 2^1 */

                /* DOT SL1: the shift takes in MPAC+2 of the DOT's TP product.     */
                l->outofpln      = agc_tp_to_dp(agc_tp_shift(&ip, agc_vec_dot_tp(&ip, n_ddumgood, l->land), 1));
                l->pointvsm_save = l->unfc_2;

                return (agc_lm_job_ignalgrt());
            }
        }

        resume          = false;
        agc_status_t st = agc_integ_install(0, AGC_JOB_IGNALOOP); /* step status */

        if (st == AGC_STATUS_BUSY)
        {
            return (AGC_STATUS_OK); /* asleep; resumes IGNALOOP  */
        }

        if (st != AGC_STATUS_OK)
        {
            return (st); /* INTSTALL could not sleep the job */
        }

        agc_flag_set(AGC_FLAG_INTYPFLG);
        agc_flag_set(AGC_FLAG_MOONFLAG);
        ic->sv.tet = agc_guid_state()->avg.piptime1;
        ic->sv.rcv = ic->out.ratt1; /* hopefully ?GUIDSUB did not */
        ic->sv.vcv = ic->out.vatt1; /* clobber RATT1 and VATT1    */
        st         = agc_integ_integrvs(&ip, ic, ic->tdec1);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    } /* end IGNALOOP passes */

    return (AGC_STATUS_NO_CONVERGE); /* DEVIATION: loop bound      */
} /* ll_ignaloop */

/**********************************************************************
 * P63LM and FLAGORGY (p. 785): "Dionysian flag waving", then IGNALG.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_braking_phase(agc_job_ctx_t *ctx)
{
    ll_restart_here(AGC_JOB_BRAKING_PHASE);

    if (agc_imu_status_check() != AGC_STATUS_OK)
    {
        return (AGC_STATUS_OK); /* R02 alarmed and went to P00 */
    }

    agc_guid_lm_t *g = &agc_guid_state()->lm; /* LM guidance state */

    g->which               = AGC_LM_WHICH_BRAKING_PHASE;         /* for BURNBABY */
    g->dvthrush            = LL_DPSTHRSH;                        /* initialize DVMON           */
    g->dvcntr              = 4;
    agc_lm_ldg()->wchphase = AGC_LM_PHASE_IGNALG;
    agc_lm_ldg()->flpass0  = 0;
    agc_chan_and(AGC_CHAN_CHAN12, (uint16_t) (077777u & ~(unsigned int) AGC_CH012_RRLOCKON));
    agc_flag_clear(AGC_FLAG_NOTHROTL);
    agc_flag_clear(AGC_FLAG_REDFLAG);
    agc_flag_clear(AGC_FLAG_LRBYPASS);
    agc_flag_set(AGC_FLAG_MUNFLAG);
    agc_flag_clear(AGC_FLAG_PREFERRED_TRACKING_ATTITUDE_ACTIVE); /* P25 off */
    agc_flag_clear(AGC_FLAG_RNDVZFLG);                           /* terminate P20 if running   */

    agc_interp_t ip; /* interpreter state (OVFIND) */

    agc_interp_init(&ip);
    agc_lm_ldg_t        *l      = agc_lm_ldg();                 /* landing erasables */
    const agc_lm_pad_t  *p      = agc_lm_pad();                 /* pad loads */
    agc_navaux_padload_t nav;                                   /* pad loads of the auxiliary nav lane */
    agc_status_t         st_pad = agc_navaux_get_padload(&nav); /* step status */

    if (st_pad != AGC_STATUS_OK)
    {
        return (st_pad);
    }

    agc_vec_t    r  = agc_vec_zero();                                                     /* landing site in SM, 2^24 m */
    agc_status_t st = agc_planet_rp_to_r(&ip, &nav, true, agc_erase_rls(), p->tland, &r); /* step status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    l->tpip     = p->tland;             /* TPIP for first guidance pass  */
    agc_mat_t m = agc_erase_refsmmat(); /* REFSMMAT, half unit */

    l->land = agc_vec_shl(&ip, agc_mat_vec(&ip, &m, r), 4); /* 2^24 m */
    st      = agc_lm_guidinit(&ip);                         /* WM and /LAND/                 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (ll_ignalg_prec(ctx, &ip));
} /* agc_lm_job_braking_phase */

/**********************************************************************
 * IGNALG (p. 786): LEMPREC was busy: try again (same job, resumed by
 * INTWAKE).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_ignalg_prec(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state (OVFIND) */

    agc_interp_init(&ip);

    if (!AGC_ASSERT(agc_lm_ldg()->wchphase == AGC_LM_PHASE_IGNALG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (ll_ignalg_prec(ctx, &ip));
}

/**********************************************************************
 * IGNALOOP (p. 787): INTSTALL was busy: continue with the integration.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_ignaloop(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_lm_ldg()->wchphase == AGC_LM_PHASE_IGNALG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (ll_ignaloop(ctx, true));
}

/**********************************************************************
 * IGNALGRT / ASTNCLOK (p. 788): prevent repeating IGNALG; start the clock
 * display (STCLOK2), returning in a new job at ASTNRET.
 *
 * ARGUMENTS:
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_ignalgrt(void)
{
    ll_restart_here(AGC_JOB_IGNALGRT);

    return (agc_lm_stclok2((agc_sp_t) -LL_ASTNDEX, AGC_JOB_ASTNRET)); /* STCLOK2 continues at ASTNRET */
}

/**********************************************************************
 * ASTNRET (p. 788): pick up the display at the end of R51; PROCEED does a
 * fine alignment, ENTER returns to P63SPOT2 (QMAJ).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_astnret(agc_job_ctx_t *ctx)
{
    agc_align_ram()->qmaj = AGC_JOB_BRAKING_PHASE_BURN_ATTITUDE; /* SSP QMAJ    */

    if (!AGC_ASSERT(agc_align_ram()->qmaj == AGC_JOB_BRAKING_PHASE_BURN_ATTITUDE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_lm_fine_align_recheck(ctx)); /* RTB R51P63  */
}

/**********************************************************************
 * P63SPOT2 (pp. 788-789): KALCMANU set for the burn attitude (+X along
 * UNIT(R60VSAVE)), 1 degree deadband, R60LEM; returns at P63SPOT3.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_braking_phase_burn_attitude(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state (OVFIND) */

    agc_interp_init(&ip);
    agc_imu_set_pointvsm(agc_vec_unit(&ip, agc_lm_ldg()->pointvsm_save, NULL, NULL));
    agc_status_t st = agc_imu_set_scaxis(agc_vec_make(AGC_DP_HALF, 0, 0)); /* step status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_inhint();
    agc_status_t sw = agc_lm_dap_pflitedb(); /* step status */

    if (sw != AGC_STATUS_OK)
    {
        agc_relint();

        return (sw);
    }

    agc_relint();

    return (agc_imu_attitude_maneuver(ctx, AGC_JOB_BRAKING_PHASE_ANTENNA_CHECK));
}

/**********************************************************************
 * P63SPOT3 (p. 789): is the LR antenna in position 1 yet? If not, ask the
 * astronaut to crank the silly thing around (V50N25 R1 = 00500).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_braking_phase_antenna_check(agc_job_ctx_t *ctx)
{
    ll_restart_here(AGC_JOB_BRAKING_PHASE_ANTENNA_CHECK); /* prevent recalling R60     */

    if (!AGC_ASSERT(agc_erase_modreg() == 63)) /* P63SPOT3 is in P63  */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_lm_chan_bit_present(AGC_CHAN_CHAN33, AGC_CH033_LRPOS1))
    {
        return (agc_lm_job_braking_phase_radar_init(ctx));
    }

    return (ll_display(ctx, AGC_DSKY_GOPERF1, LL_CODE500, AGC_JOB_BRAKING_PHASE_ANTENNA_CHECK, AGC_JOB_BRAKING_PHASE_RADAR_INIT)); /* PROCEED: see if he's lying; ENTER */
}

/**********************************************************************
 * P63SPOT4 (p. 789): initialize the landing radar, off to see the wizard.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_braking_phase_radar_init(agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_lm_setpos1(); /* step status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_lm_burnbaby(ctx));
}

/* ======================================================================
 * GUILDENSTERN: auto-modes monitor R13 (pp. 800-801)
 * ====================================================================== */

/**********************************************************************
 * STRTP66A (p. 800): ROD initialisation (also after a restart in P66,
 * leaving VDGVERT as is).
 **********************************************************************/
static void ll_rate_of_descent_setup(void)
{
    agc_lm_ldg_t         *l = agc_lm_ldg(); /* landing erasables */
    agc_imu_comp_params_t c;                /* IMU compensation pad loads (PBIAS) */

    agc_imu_comp_get(&c);

    agc_interp_t ip; /* interpreter state (OVFIND) */

    agc_interp_init(&ip);
    agc_vec_t b = agc_vec_make(agc_dp_from_sp(c.pipabias[0]), /* PBIAS, 2^14 */
                               agc_dp_from_sp(c.pipabias[1]),
                               agc_dp_from_sp(c.pipabias[2]));

    l->vbias = agc_vec_scale(&ip, b, LL_BIASFACT);
    agc_flag_set(AGC_FLAG_RODFLAG);

    for (int i = 0; i < 3; i++)
    { /* VCOMP of TEMX..TEMZ  */
        l->oldpipa[i] = (agc_sp_t) -agc_erase_tem(i);
    }

    l->delvrod            = agc_vec_zero();
    l->rodscal1           = agc_lm_pad()->rodscale;
    l->lasttpip           = agc_erase_piptime();
    agc_lm_thr()->fcold   = 0;
    agc_lm_thr()->fweight = agc_dp_to_words(0);
}

/**********************************************************************
 * VRTSTART (pp. 800-801): WCHPHASE = 2 (P65, P66, P67), STOPRATE ("tem-
 * porary, I hope hope hope"), X-axis override and redesignation off.
 *
 * ARGUMENTS:
 *   wchvert  agc_sp_t  I    WCHVERT: 0 for P66, 10 for P67
 **********************************************************************/
static void ll_vrtstart(agc_sp_t wchvert)
{
    agc_lm_ldg_t *l = agc_lm_ldg(); /* landing erasables */

    l->wchvert  = wchvert;
    l->wchphold = AGC_LM_PHASE_VERTICAL;
    l->wchphase = AGC_LM_PHASE_VERTICAL;
    agc_lm_dap_stoprate();
    agc_flag_clear(AGC_FLAG_XOVINFLG);
    agc_flag_clear(AGC_FLAG_REDFLAG);
}

/**********************************************************************
 * STARTP66 (p. 800): P66 with desired altitude rate = current rate.
 *
 * RETURNS: agc_status_t -- OK, or the NEWMODE (MMDSPLAY NOVAC) status
 **********************************************************************/
static agc_status_t ll_start_rate_of_descent_landing(void)
{
    agc_lm_fastchng();
    agc_status_t st = agc_phase_newmode(66); /* NEWMODE: MMDSPLAY (NOVAC) */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_lm_ldg()->vdgvert = agc_guid_state()->lm.hdotdisp;
    ll_rate_of_descent_setup();
    ll_vrtstart(0);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * GUILDEN(STERN) (pp. 800-801). On every appearance or disappearance of
 * the manual throttle discrete select P67 or P66; on every appearance of
 * attitude hold select P66, unless in P67. *entry: GUILDRET (continue
 * the automatic landing) or VERTGUID.
 *
 * ARGUMENTS:
 *   entry  int *                  O    GUILDRET (continue the automatic
 *                                      landing) or VERTGUID
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_guildenstern(int *entry)
{
    agc_lm_ldg_t *l = agc_lm_ldg(); /* landing erasables */

    *entry = AGC_LM_ENTRY_VERTGUID;

    if (!agc_lm_chan_bit_present(AGC_CHAN_CHAN30, AGC_CH030_AUTOTHROT))
    {
        /* un-auto-throttle present  */
        /* STARTP67 (p. 801): no harm in "starting" P67 over and over. */
        agc_status_t st = agc_phase_newmode(67); /* NEWMODE: MMDSPLAY (NOVAC) */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        agc_lm_ldg()->rodcount = 0;
        ll_vrtstart(10);

        return (AGC_STATUS_OK);
    }

    if (agc_phase_checkmm(67))
    { /* P67NOW?                   */
        return (ll_start_rate_of_descent_landing());
    }

    if (!agc_lm_chan_bit_present(AGC_CHAN_CHAN31, AGC_CH031_ATTHOLD))
    {
        *entry = AGC_LM_ENTRY_GUILDRET; /* STABL?: all's well        */

        return (AGC_STATUS_OK);
    }

    if (agc_erase_modreg() == 66)
    { /* P66NOW? -> RESTART?       */
        if (!agc_flag_test(AGC_FLAG_RODFLAG))
        {
            ll_rate_of_descent_setup(); /* restart: reinitialize     */
            ll_vrtstart(0);
        }

        return (AGC_STATUS_OK);
    }

    if (l->rodcount == 0)
    {
        *entry = AGC_LM_ENTRY_GUILDRET; /* ROD switch not clicked    */

        return (AGC_STATUS_OK);
    }

    return (ll_start_rate_of_descent_landing()); /* switch into the ROD mode  */
} /* agc_lm_guildenstern */

/* ======================================================================
 * Landing site perturbation equations (pp. 804-805)
 * ====================================================================== */

/**********************************************************************
 * REDESIG (pp. 804-805): with REDFLAG set and TREDES not zero, move the
 * landing site by the accumulated LPD increments ELINCR (about the Y
 * body axis) and AZINCR, keeping the line of sight below DEPRCRIT.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_redesig(agc_interp_t *ip)
{
    agc_lm_ldg_t *l = agc_lm_ldg(); /* landing erasables */

    if (!agc_flag_test(AGC_FLAG_REDFLAG) || l->tredes == 0)
    {
        return (agc_lm_rgvgcalc(ip)); /* skip redesignation logic  */
    }

    agc_inhint();
    l->elincr                = agc_dp_from_sp(l->elincr1);                                                  /* low words zeroed          */
    l->azincr                = agc_dp_from_sp(l->azincr1);
    agc_lm_fastchng();
    l->elincr1               = 0;
    l->azincr1               = 0;
    agc_relint();
    const agc_guid_lm_t *g   = &agc_guid_state()->lm;                                                       /* LM guidance state */
    int                  sh  = 0;                                                                           /* NORMUNIT shift count */
    agc_vec_t            u   = agc_rtb_normunit(ip, agc_vec_sub(ip, l->land, g->r), &sh, NULL, NULL);       /* UNIT(LAND - R), half unit 2^1 */
    agc_vec_t            ynb = g->xnbpip.row[1];                                                            /* YNBPIP, half unit 2^1 */
    agc_vec_t            e   = agc_vec_scale(ip, agc_vec_shl(ip, agc_vec_cross(ip, u, ynb), 1), l->elincr); /* ELINCR (YNB x UNIT(LAND - R)), 2^1 */
    agc_vec_t            w   = agc_vec_add(ip, agc_vec_sub(ip, agc_vec_scale(ip, ynb, l->azincr), e), u);   /* W: perturbed line of sight, half size 2^1 */

    if (agc_dp_sub(ip, w.c[0], LL_DEPRCRIT) >= 0)
    {
        w.c[0] = LL_DEPRCRIT; /* not too close to horizon  */
    }

    /* REDES1 (p. 805): the new site where the perturbed line of sight W
     * (half size) meets the plane of the old site's X coordinate. */
    agc_lm_ldg_t   *l_site = agc_lm_ldg();                                                      /* landing erasables */
    const agc_vec_t r      = agc_guid_state()->lm.r;                                            /* R, SM, 2^24 m */
    agc_dp_t        q      = agc_dp_div(ip, agc_dp_sub(ip, l_site->land.c[0], r.c[0]), w.c[0]); /* 2^24 m */
    agc_vec_t       n      = agc_vec_add(ip, agc_vec_scale(ip, w, q), r);                       /* R + q W, 2^24 m */

    n           = agc_vec_unit(ip, n, NULL, NULL);
    l->landtemp = agc_vec_shl(ip, agc_vec_scale(ip, n, l_site->land_mag), 1);
    agc_lm_fastchng();
    l->land     = l->landtemp;

    if (!AGC_ASSERT(sh >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_rgvgcalc(ip));
} /* agc_lm_redesig */

/* ======================================================================
 * P66 (rate of descent) and P67 (pp. 816-819)
 * ====================================================================== */

/**********************************************************************
 * VHORCOMP (p. 819): VHORIZ from the given velocity (2^7 m/cs) corrected
 * for lunar rotation, minus its vertical part; then DISPEXIT +3.
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    executive job context
 *   ip   agc_interp_t *         I/O  interpreter state (OVFIND)
 *   vel  agc_vec_t              I    velocity, SM, 2^7 m/cs
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t ll_vhorcomp(const agc_job_ctx_t *ctx, agc_interp_t *ip, agc_vec_t vel)
{
    const agc_guid_lm_t *g  = &agc_guid_state()->lm;                                              /* LM guidance state */
    agc_vec_t            v  = agc_vec_shr(agc_vec_add(ip, agc_vec_shl(ip, vel, 2), g->delvs), 2); /* V + DELVS without its vertical part, 2^7 m/cs */
    agc_vec_t            ur = agc_vec_unit(ip, g->r, NULL, NULL);                                 /* UNIT(R), half unit 2^1 */
    agc_vec_t            vv = agc_vec_shl(ip, agc_vec_scale(ip, ur, g->hdotdisp), 1);             /* HDOTDISP UNIT(R), 2^7 m/cs */

    agc_lm_ldg()->vhoriz = agc_vec_abval(ip, agc_vec_sub(ip, v, vv));

    return (agc_lm_dispexit(ctx, false)); /* avoid the PHASCHNG        */
}

/**********************************************************************
 * RODCOMP acceleration command (pp. 818-819):
 *   /AFC/ = ((VDGVERT - HDOTDISP)/TAUROD + |G|) / (X.UR), then corrected
 *   by LAG/TAU for the measured acceleration and limited to MINFORCE/MASS
 *   .. MAXFORCE/MASS. Returns the measured acceleration (2^-4 m/cs^2).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 *   ur  agc_vec_t       I    UNIT(R), half unit 2^1
 *
 * RETURNS: agc_dp_t -- measured acceleration, 2^-4 m/cs^2
 **********************************************************************/
static agc_dp_t ll_rod_afc(agc_interp_t *ip, agc_vec_t ur)
{
    agc_lm_ldg_t       *l    = agc_lm_ldg();                                                              /* landing erasables */
    agc_guid_t         *g    = agc_guid_state();                                                          /* guidance state */
    const agc_lm_pad_t *p    = agc_lm_pad();                                                              /* pad loads */
    agc_dp_t            arod = agc_dp_div(ip, agc_dp_sub(ip, l->vdgvert, g->lm.hdotdisp), p->taurod);     /* (VDGVERT - HDOTDISP)/TAUROD, 2^-4 m/cs^2 */
    agc_dp_t            gm   = agc_dp_shr(agc_dp_div(ip, agc_vec_abval(ip, g->avg.gdt_2), LL_GSCALE), 2); /* |G| (20D), 2^-4 m/cs^2 */
    agc_vec_t           x    = agc_vec_make(AGC_DP_HALF, 0, 0);                                           /* UNITX, transformed to SM by CDU*NBSM, 2^1 */

    agc_guid_cdu_nbsm(&g->trig, &x);

    agc_dp_t xur = agc_vec_dot(ip, x, ur); /* X.UR (22D), scale 2^2 */

    l->afc        = agc_dp_div(ip, agc_dp_add(ip, gm, arod), xur);
    agc_vec_t dv  = agc_vec_add(ip, agc_vec_scale(ip, l->delvrod, LL_KPIP1), l->vbias); /* DELVROD KPIP1 + VBIAS, 2^7 m/cs */
    agc_dp_t  dt2 = agc_dp_sub(ip, l->thistpip, l->lasttpip);                           /* THISTPIP - LASTTPIP, cs 2^28 */

    l->lasttpip   = l->thistpip;
    agc_dp_t meas = agc_dp_div(ip, agc_vec_abval(ip, dv), agc_dp_div(ip, dt2, LL_SHFTFACT)); /* measured acceleration, 2^-4 m/cs^2 */

    /* DLOAD FWEIGHT, DMP BIT1H: (hi, lo) x 2^-14 leaves (0, hi, lo) in
     * MPAC; the DP read is (0, hi) even when THROTTLE's DAS left the
     * pair sign-disagreeing (lo never borrows from hi).                */
    agc_dp_t fw = agc_dp_pack(0, agc_lm_thr()->fweight.hi); /* FWEIGHT DMP BIT1H = (0, FWEIGHT hi), 2^28 */

    fw            = agc_dp_div(ip, agc_dp_div(ip, fw, g->lm.mass), LL_SCALEFAC);
    agc_dp_t e    = agc_dp_sub(ip, agc_dp_div(ip, gm, xur), agc_dp_add(ip, fw, meas)); /* G/(X.UR) - (FWEIGHT + measured), 2^-4 */
    agc_dp_t afc  = agc_dp_add(ip, agc_dp_mul_trunc(e, p->lag_tau), l->afc);           /* corrected /AFC/, 2^-4 m/cs^2 */
    agc_dp_t amax = agc_dp_div(ip, p->maxforce, g->lm.mass);                           /* MAXFORCE/MASS, 2^-4 m/cs^2 */
    agc_dp_t amin = agc_dp_div(ip, p->minforce, g->lm.mass);                           /* MINFORCE/MASS, 2^-4 m/cs^2 */

    l->afc = afc; /* AFCSPOT: limited to MINFORCE/MASS .. MAXFORCE/MASS */

    if (afc < amin)
    {
        l->afc = amin;
    }
    else if (afc >= amax)
    {
        l->afc = amax;
    }

    return (meas);
} /* ll_rod_afc */

/**********************************************************************
 * RODCOMP first part (pp. 816-817): ROD clicks into VDGVERT, PIPA
 * snapshot at THISTPIP, DELVROD = counts since the last snapshot
 * (under INHINT).
 *
 * RETURNS: agc_vec_t -- PIPA + PIPATMP, the counts since PIPTIME, in the
 *          high words (2^14 counts)
 **********************************************************************/
static agc_vec_t ll_rod_pipas(void)
{
    agc_lm_ldg_t *l_pipas = agc_lm_ldg(); /* landing erasables */

    agc_inhint();
    agc_sp_t clicks = l_pipas->rodcount; /* ROD clicks since last pass, unscaled */

    l_pipas->rodcount      = 0;
    l_pipas->vdgvert       = (agc_dp_t) (l_pipas->vdgvert + agc_sp_mul(clicks, l_pipas->rodscal1));
    const agc_sp_t pipa[3] = {agc_erase_pipax(), agc_erase_pipay(), agc_erase_pipaz()};
    agc_sp_t       rupt[3]; /* RUPTREG1..3: old OLDPIPA, counts 2^14 */

    for (int i = 0; i < 3; i++)
    {
        rupt[i]             = l_pipas->oldpipa[i]; /* RUPTREG1..3 */
        l_pipas->oldpipa[i] = pipa[i];
    }

    l_pipas->thistpip   = (agc_dp_t) agc_time_now_cs();
    const agc_sp_t *tmp = agc_guid_state()->avg.pipatmp; /* PIPATMPX..Z, counts 2^14 */
    agc_vec_t       pip;                                 /* PIPA + PIPATMP, counts, high words 2^14 */

    for (int i = 0; i < 3; i++)
    {
        /* CS OLDPIPAX / AD TEMX / AD RUPTREG1 / TS DELVROD (high word). */
        agc_sp_t d = agc_lm_ad(agc_lm_ad((agc_sp_t) -pipa[i], agc_erase_tem(i)), rupt[i]); /* counts since the last pass, 2^14 */

        pip.c[i]              = agc_dp_from_sp(agc_lm_ad(pipa[i], tmp[i]));
        l_pipas->delvrod.c[i] = agc_dp_from_sp(d);
        agc_erase_set_tem(i, 0); /* know when READACCS changes */
    }

    agc_relint();

    return (pip);
} /* ll_rod_pipas */

/**********************************************************************
 * RODCOMP (pp. 816-819): P66 rate-of-descent guidance, every second
 * (from P66VERTA in the guidance job, and 1 s later from RODTASK).
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t ll_rodcomp(const agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state (OVFIND) */

    agc_interp_init(&ip);
    agc_vec_t pip = ll_rod_pipas(); /* PIPA + PIPATMP, counts 2^14 */

    agc_dp_t dt = 0; /* THISTPIP - PIPTIME (30D), cs 2^28 */

    /* RODCOMP velocity and altitude (pp. 817-818): V propagated to THISTPIP,
     * HDOTDISP and HCALC1 for noun 63. Returns the updated velocity.     */
    agc_lm_ldg_t *l   = agc_lm_ldg();                      /* landing erasables */
    agc_guid_t   *g   = agc_guid_state();                  /* guidance state */
    agc_vec_t     dvp = agc_vec_scale(&ip, pip, LL_KPIP1); /* 2^7 m/cs  */

    dt              = agc_dp_sub(&ip, l->thistpip, agc_erase_piptime());                   /* 30D       */
    agc_dp_t  ratio = agc_dp_div(&ip, dt, LL_4SEC28);                                      /* dt / 4SEC(28), scale 2^0 */
    agc_vec_t v     = agc_vec_scale(&ip, agc_vec_sub(&ip, g->avg.gdt_2, l->vbias), ratio); /* V at THISTPIP, 2^7 m/cs */

    v                  = agc_vec_add(&ip, agc_vec_add(&ip, agc_vec_shl(&ip, v, 2), g->lm.v), dvp);
    agc_dp_t  rmag     = 0;                                       /* |R| (36D), 2^24 m */
    agc_vec_t ur_state = agc_vec_unit(&ip, g->lm.r, &rmag, NULL); /* UNIT(R) (14D), half unit 2^1 */

    /* DOT SL1 shifts the TP product (MPAC+2 bit enters the low word). */
    g->lm.hdotdisp = agc_tp_to_dp(agc_tp_shift(&ip, agc_vec_dot_tp(&ip, ur_state, v), 1));
    agc_dp_t h     = agc_dp_mul_trunc(agc_dp_shl(&ip, dt, 11), g->lm.hdotdisp); /* dt HDOTDISP, 2^24 m */

    l->hcalc1 = agc_dp_sub(&ip, agc_dp_add(&ip, h, rmag), l->land_mag);

    if (!AGC_ASSERT(dt >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t    ur   = agc_vec_unit(&ip, agc_guid_state()->lm.r, NULL, NULL); /* UNIT(R), half unit 2^1 */
    agc_dp_t     meas = ll_rod_afc(&ip, ur);                                   /* measured acceleration, 2^-4 m/cs^2 */
    agc_status_t st   = agc_lm_throttle_measured(meas);                        /* THROTTLE +3   */

    if (st != AGC_STATUS_OK)
    {
        return (st); /* DOIT's waitlist failure (BAILOUT raised there) */
    }

    return (ll_vhorcomp(ctx, &ip, v));
} /* ll_rodcomp */

/**********************************************************************
 * P66VERTA (p. 816): terminate group 3, RODTASK in one second, RODCOMP.
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_rate_of_descent_vertical(const agc_job_ctx_t *ctx)
{
    agc_phase_change(00003);
    agc_status_t st = agc_waitlist_call(LL_1SEC, AGC_TASK_RODTASK); /* TWIDDLE */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (ll_rodcomp(ctx));
}

/**********************************************************************
 * RODTASK (p. 816): FINDVAC RODCOMP at priority 22.
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_task_rodtask(void)
{
    agc_status_t st = agc_exec_findvac(LL_PRIO22, AGC_JOB_RODCOMP); /* step status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!AGC_ASSERT(agc_lm_ldg()->wchphase == AGC_LM_PHASE_VERTICAL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK); /* TASKOVER     */
}

/**********************************************************************
 * RODCOMP (p. 816) as a job (from RODTASK).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_rodcomp(agc_job_ctx_t *ctx)
{
    return (ll_rodcomp(ctx));
}

/**********************************************************************
 * P67VERT (p. 816): terminate group 3; VHORIZ from SERVICER's V.
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    executive job context
 *   ip   agc_interp_t *         I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_manual_landing_vertical(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    if (!AGC_ASSERT(agc_lm_ldg()->wchvert > 0)) /* P67VERT: WCHVERT > 0 */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change(00003);

    return (ll_vhorcomp(ctx, ip, agc_guid_state()->lm.v));
}

/* ======================================================================
 * Redesignator trap and R.O.D. trap (pp. 820-822)
 * ====================================================================== */

/**********************************************************************
 * AGC: none (READ CHAN31 / COM / MASK ALL4BITS, PITFALL and REDESMON
 * p. 820) -- channel 31 LPD bits that are present (inverted inputs)
 *
 * RETURNS: uint16_t -- LPD bits present (ALL4BITS mask)
 **********************************************************************/
static uint16_t ll_lpd_bits(void)
{
    uint16_t c = (uint16_t) (~agc_chan_read(AGC_CHAN_CHAN31) & LL_ALL4BITS); /* present LPD bits */

    _Static_assert(LL_ALL4BITS == 063, "AGC constant");

    return (c);
}

/**********************************************************************
 * PITFALL (p. 820): RUPT10 from the hand controller. Only in P64: note
 * the bits and start REDESMON in 50 ms.
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_pitfall(void)
{
    if (agc_phase_checkmm(64)) /* RESUME       */
    {
        agc_lm_ldg_t *l = agc_lm_ldg(); /* landing erasables */

        l->elvira       = (agc_sp_t) ll_lpd_bits();
        l->zerlina      = 2;
        agc_status_t st = agc_waitlist_call(5, AGC_TASK_REDESMON); /* step status */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        if (!AGC_ASSERT(l->elvira >= 0))
        {
            return (AGC_STATUS_BAD_ARG);
        }
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * RESETRPT (p. 821): re-enable RUPT10 and end the monitor.
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t ll_resetrpt(void)
{
    agc_chan_or(AGC_CHAN_CHAN13, AGC_CH013_RSTTRAP31A);
    _Static_assert(AGC_CH013_RSTTRAP31A == 04000, "AGC constant");

    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN13) & AGC_CH013_RSTTRAP31A) != 0u)) /* RUPT10 re-enabled */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK); /* TASKOVER     */
}

/**********************************************************************
 * REDESMON (pp. 820-821): every 70 ms while bits are present; count them
 * when released; give up when ZERLINA runs out.
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_task_redesmon(void)
{
    agc_lm_ldg_t *l    = agc_lm_ldg();         /* landing erasables */
    uint16_t      last = (uint16_t) l->elvira; /* ELVIRA of the last pass */

    l->elvira = (agc_sp_t) ll_lpd_bits();

    if (l->elvira != 0)
    {
        return (agc_waitlist_call(7, AGC_TASK_REDESMON)); /* PREMON2       */
    }

    if (last != 0u)
    {
        /* COUNT'EM (p. 821): each released LPD click moves AZINCR1 by 2 degrees
         * or ELINCR1 by 1/2 degree, unless in attitude hold.                */
        agc_lm_ldg_t *l_countem = agc_lm_ldg(); /* landing erasables */

        if (agc_lm_chan_bit_present(AGC_CHAN_CHAN31, AGC_CH031_ATTHOLD))
        {
            return (ll_resetrpt());
        }

        if ((last & LL_MINUSAZBIT) != 0u)
        {
            l_countem->azincr1 = agc_lm_ad(l_countem->azincr1, (agc_sp_t) -LL_AZEACH);
        }

        if ((last & LL_PLUSAZBIT) != 0u)
        {
            l_countem->azincr1 = agc_lm_ad(l_countem->azincr1, LL_AZEACH);
        }

        if ((last & LL_MINUSELBIT) != 0u)
        {
            l_countem->elincr1 = agc_lm_ad(l_countem->elincr1, (agc_sp_t) -LL_ELEACH);
        }

        if ((last & LL_PLUSELBIT) != 0u)
        {
            l_countem->elincr1 = agc_lm_ad(l_countem->elincr1, LL_ELEACH);
        }

        if (!AGC_ASSERT(last <= LL_ALL4BITS))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        return (ll_resetrpt());
    }

    if (l->zerlina > 0)
    {
        l->zerlina = (agc_sp_t) (l->zerlina - 1); /* PREMON1      */

        return (agc_waitlist_call(7, AGC_TASK_REDESMON));
    }

    if (!AGC_ASSERT(l->zerlina == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (ll_resetrpt());
} /* agc_lm_task_redesmon */

/**********************************************************************
 * DESCBITS (p. 822): from the MARKRUPT coding with channel 16 bit 7
 * (- rate increment) or bit 6 (+ increment). The trap is reset when the
 * switch is released.
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_descbits(void)
{
    uint16_t      c16 = agc_chan_read(AGC_CHAN_NAVKEYIN); /* channel 16 (DESCENT bits 6, 7) */
    agc_sp_t      inc = 1;                                /* BIT6: + rate increment, unscaled */
    agc_lm_ldg_t *l   = agc_lm_ldg();                     /* landing erasables */

    if ((c16 & AGC_CH016_DESCENTDN) != 0u)
    {
        inc = -1; /* BIT7: - rate increment */
    }

    l->rodcount = agc_lm_ad(l->rodcount, inc);

    if (!AGC_ASSERT((c16 & (AGC_CH016_DESCENTDN | AGC_CH016_DESCENTUP)) != 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK); /* RESUME       */
}

/* ======================================================================
 * P68: landing confirmation (pp. 791-792)
 * ====================================================================== */

/**********************************************************************
 * LANDJUNK (p. 791): zero attitude error, 5 degree deadband, surface
 * flags, LAT/LONG/ALT of the landing point and RLS = R-TO-RP(RN).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t ll_landjunk(agc_interp_t *ip)
{
    agc_inhint();
    agc_lm_dap_zatteror();
    agc_status_t sw = agc_lm_dap_setmaxdb(); /* step status */

    if (sw != AGC_STATUS_OK)
    {
        agc_relint();

        return (sw);
    }

    agc_relint();
    agc_flag_set(AGC_FLAG_SURFFLAG);
    agc_flag_clear(AGC_FLAG_LETABORT);
    agc_flag_set(AGC_FLAG_APSFLAG);
    agc_flag_set(AGC_FLAG_LUNAFLAG);
    agc_navaux_padload_t nav;                                   /* pad loads of the auxiliary nav lane */
    agc_status_t         st_pad = agc_navaux_get_padload(&nav); /* step status */

    if (st_pad != AGC_STATUS_OK)
    {
        return (st_pad);
    }

    agc_vec_t         rn  = agc_erase_rn();      /* RN at PIPTIME (ALPHAV), 2^29 m */
    agc_dp_t          t   = agc_erase_piptime(); /* PIPTIME, cs 2^28 */
    agc_latlong_opt_t opt = {true, agc_flag_test(AGC_FLAG_ERADFLAG)};
    agc_latlong_t     ll;                        /* latitude, longitude, altitude */

    agc_status_t st = agc_latlong_from_r(ip, &nav, opt, rn, t, &ll); /* step status */

    if (st == AGC_STATUS_OK)
    {
        st = agc_navaux_set_latlong(&ll); /* N43 data     */
    }

    /* LAT-LONG has no failure exit in the AGC; it falls straight into
     * R-TO-RP, so a failure here is propagated like R-TO-RP's.         */
    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_vec_t rls = agc_vec_zero(); /* RLS, moon-fixed, 2^27 m */

    st = agc_planet_r_to_rp(ip, &nav, true, agc_vec_shl(ip, rn, 2), t, &rls);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_erase_set_rls(rls);

    return (AGC_STATUS_OK);
} /* ll_landjunk */

/**********************************************************************
 * LANDJUNK (p. 791): the P68 job.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_landing_confirmation(agc_job_ctx_t *ctx)
{
    ll_restart_here(AGC_JOB_LANDING_CONFIRMATION);
    agc_interp_t ip; /* interpreter state (OVFIND) */

    agc_interp_init(&ip);
    agc_status_t st = ll_landjunk(&ip); /* step status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_lm_job_landing_confirmation_flash(ctx));
}

/**********************************************************************
 * LANDJUNK (p. 791): V06N43 flash, "astronaut: now look where you ended
 * up".
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_landing_confirmation_flash(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_SURFFLAG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (ll_display(ctx, AGC_DSKY_GOFLASH, agc_dsky_vn(6, 43), AGC_JOB_LANDING_CONFIRMATION_PROCEED, AGC_JOB_LANDING_CONFIRMATION_FLASH));
}

/**********************************************************************
 * After PROCEED (p. 792): GSAV and (using REFMF) YNBSAV, ZNBSAV and
 * ATTFLAG for P57; then "astronaut: please select P57".
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_landing_confirmation_proceed(void)
{
    agc_interp_t ip; /* interpreter state (OVFIND) */

    agc_interp_init(&ip);
    agc_align_ram()->gsav = agc_vec_make(AGC_DP_HALF, 0, 0); /* UNITX  */
    agc_status_t st       = agc_align_refmf(&ip);            /* step status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_exec_gotopooh());
}
