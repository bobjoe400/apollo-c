/**********************************************************************
 * FILE NAME: aborts.c
 *
 * PURPOSE:
 *   P70-P71.agc (Luminary 099, pp. 829-837): R10,R11 (the
 *   quarter-second landing radar lamp / landing analog display / abort
 *   monitor task), the DPS (P70) and APS (P71) abort programs, and their
 *   targeting: GOABORT, P70INIT, BOTHPOLY (abort injection velocity
 *   polynomials), the throttle-up and the hand-over to ascent guidance.
 *
 *   Abort entry. In the AGC an abort (from the ABORT/ABORT STAGE switches
 *   seen by R10,R11, or V37 P70/P71) goes INHINT, replaces BRUPT and
 *   RESUMEs into ABRTJASK, which sets restart group 4 at 4.3 (GOABORT) and
 *   requests a software restart (ENEMA). Here ABRTJASK runs as a function
 *   and the restart is the executive's deferred ENEMA (exec-sched note).
 *
 * AGC SOURCE:
 *   Luminary099 P70-P71.agc, pp. 829-837
 *
 * EXTERNAL VARIABLES:
 *   lane erasables through agc_lm_asc(), agc_lm_pad(); guidance state
 *   through agc_guid_state(); ab_pipctr1 (PIPCTR1, file static)
 *
 * EXTERNAL REFERENCES:
 *   agc_lm_landisp, agc_lm_lrhtask, agc_lm_comminit,
 *   agc_lm_powered_ascent_init, agc_lm_ycomp,
 *   agc_lm_check_auto_control, agc_exec_enema, agc_phase_newphase,
 *   agc_rendz_flip, agc_rendz_clradmod
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ABORTALM: OPR ERR (FALTON) and back to pinball when a V37 P70/P71
 *   request is illegal; no program alarms
 **********************************************************************/
#include "agc/core/assert.h"
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

#define AB_OCT31   25                     /* OCT31: R10,R11 period, cs                */
#define AB_OCTAL27 027                    /* 4.27 (P70A) / 4.31 (P71A) restart phase  */
#define AB_THRESH2 308                    /* THRESH2: DVMON threshold, ascent         */
#define AB_HLITE   020                    /* HLITE = BIT5                             */
#define AB_VLITE   004                    /* VLITE = BIT3                             */
#define AB_PRIO30  030000                 /* PRIO30: channel 11 bits 13 and 14        */
#define AB_BIT13   010000                 /* BIT13 of channel 11: engine on */
#define AB_LRBYBIT 040000                 /* LRBYBIT = BIT15 of FLGWRD11              */
#define AB_T5_40MS 037774                 /* OCT37774: T5RUPT in 40 ms (DAPIDLER)     */
#define AB_MDOTDPS ((agc_dp_t) 4966056)   /* 2DEC 0.1480 B-3 kg/cs       */
#define AB_K1DV    ((agc_dp_t) 228956570) /* K(1/DV) 2DEC 436.70 B-9     */
#define AB_KAT     ((agc_dp_t) 5368709)   /* K(AT) 2DEC .02              */
#define AB_DTDECAY ((agc_dp_t) -38)       /* DTDECAY 2DEC -38 (cs)       */
#define AB_DPSVEX  (-15134)               /* DPSVEX DEC* -2.95588868E+1 B-5 m/cs     */

/* PIPCTR1 (= LADQSAVE): the R10,R11 pass count handed to LANDISP. */
static agc_sp_t ab_pipctr1; /* PIPCTR1 (= LADQSAVE), passes, unscaled */

/**********************************************************************
 * AGC: none (accessor) -- PIPCTR1 (= LADQSAVE, P70-P71.agc p. 829):
 * the R10,R11 pass count LANDISP copies back into PIPCTR
 *
 * RETURNS: agc_sp_t -- PIPCTR1, passes left in the cycle, 0..7 (unscaled)
 **********************************************************************/
agc_sp_t agc_lm_pipctr1(void)
{
    if (!AGC_ASSERT(ab_pipctr1 >= 0))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(ab_pipctr1 <= 7))
    {
        return (0); /* no valid result */
    }

    return (ab_pipctr1);
}

/**********************************************************************
 * R10,R11 (P70-P71.agc p. 829): the quarter-second task SERVICER starts.
 * Each pass flashes the landing radar lamps, watches the ABORT and ABORT
 * STAGE discretes (R11, "is an abort (stage) commanded?") and goes on to
 * the landing analog displays (R10, LANDISP). On the last pass of the
 * two-second cycle (PIPCTR = 0) it calls LRHTASK, which continues at
 * R10,R11A (agc_lm_abort_monitor_resume).
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_task_abort_monitor(void)
{
    if (!agc_flag_test(AGC_FLAG_AVEGFLAG))
    {
        return (AGC_STATUS_OK); /* let AVGEND take care of group 2 */
    }

    agc_sp_t pipctr = agc_guid_state()->avg.pipctr; /* PHSPRDT2, unscaled */

    if (!AGC_ASSERT(pipctr >= 0))
    { /* CCS would store |PIPCTR| - 1; SERVICER never sets it < 0   */
        return (AGC_STATUS_BAD_ARG);
    }

    if (pipctr <= 0)
    {
        return (agc_lm_lrhtask()); /* last pass      */
    }

    ab_pipctr1 = (agc_sp_t) (pipctr - 1);

    if (!AGC_ASSERT(ab_pipctr1 >= 0 && ab_pipctr1 <= 7)) /* PIPCTR1 0..7 */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_waitlist_call(AB_OCT31, AGC_TASK_ABORT_MONITOR); /* step status */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_abort_monitor_resume());
}

/**********************************************************************
 * ABRTJASK (pp. 831-832): commit to the abort. aps_abort selects P71 (APS).
 *
 * ARGUMENTS:
 *   aps_abort  bool  I    true for P71 (APS), false for P70 (DPS)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t ab_abrtjask(bool aps_abort)
{
    agc_sp_t mm    = 70;         /* MODE70 (Q = -0), unscaled      */
    uint16_t phase = AB_OCTAL27; /* 4.27, OCTAL27 + Q              */

    if (aps_abort)
    { /* P71A: Q = 2                                                */
        mm    = 71;
        phase = (uint16_t) (phase + 2u);
    }

    agc_phase_newphase(4, phase, false);
    agc_erase_set_modreg(mm);
    agc_guid_state()->lm.dispdex = mm; /* insure DISPDEX is positive     */

    if (!AGC_ASSERT(agc_erase_modreg() == mm)) /* P70 or P71 now     */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (aps_abort)
    {
        agc_flag_set(AGC_FLAG_APSFLAG); /* prior to the ENEMA      */
    }

    agc_flag_clear(AGC_FLAG_ULLAGFLG);               /* DAPBITS: ullage, drift, */
    agc_flag_clear(AGC_FLAG_DRIFTDFL);               /* and XOVINHIB off        */
    agc_flag_clear(AGC_FLAG_XOVINFLG);
    agc_flag_set(AGC_FLAG_ENGONFLG);
    uint16_t c11 = agc_chan_read(AGC_CHAN_DSALMOUT); /* engine on if armed */

    agc_chan_write(AGC_CHAN_DSALMOUT, (uint16_t) ((c11 & ~(unsigned int) AB_PRIO30) | AB_BIT13));
    agc_flagword_set(11, AB_LRBYBIT);            /* terminate R12           */
    agc_flag_set(AGC_FLAG_ALTITUDE_METERS_ONLY); /* no cross-pointer outputs */
    agc_rendz_clradmod();                        /* RADMODES for R29        */
    agc_erase_set_tevent((agc_dp_t) agc_time_now_cs());
    agc_guid_state()->avg.avgexit = AGC_JOB_SERVEXIT;
    agc_phase_kill(1);
    agc_phase_kill(3);
    agc_phase_kill(6);
    agc_phase_newphase(4, 3, false);             /* 4.3SPOT for GOABORT     */
    agc_rupt_set_time(5, AB_T5_40MS);            /* DAPIDLER in 40 ms       */

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_ENGONFLG))) /* engine armed on */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_enema());
} /* ab_abrtjask */

/**********************************************************************
 * R10,R11A .. P70NOW? (P70-P71.agc pp. 829-830): flash the LR lamps
 * (not during a lamp test), then, with aborts enabled (LETABORT), look
 * for ABORT STAGE (P71A) unless in P71, or ABORT (P70A) unless in P70;
 * otherwise on to R10 (LANDISP). Also entered from LRHTASK.
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_abort_monitor_resume(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM)) /* LM only   */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if ((agc_erase_imodes33() & 1u) == 0u)
    { /* not a lamp test         */
        if (agc_flag_test(AGC_FLAG_HFLSHFLG))
        {
            agc_rendz_flip(AB_HLITE); /* flip H lite             */
        }

        if (agc_flag_test(AGC_FLAG_VFLSHFLG))
        {
            agc_rendz_flip(AB_VLITE); /* flip V lite             */
        }
    }

    agc_sp_t mm = agc_erase_modreg(); /* MODREG, program number, unscaled */

    if (!agc_flag_test(AGC_FLAG_LETABORT) || mm == 71)
    {
        return (agc_lm_landisp()); /* proceed to R10          */
    }

    if (agc_lm_chan_bit_present(AGC_CHAN_CHAN30, AGC_CH030_ABORTAPS))
    {
        return (ab_abrtjask(true)); /* P71A                    */
    }

    if (mm == 70)
    {
        return (agc_lm_landisp());
    }

    if (agc_lm_chan_bit_present(AGC_CHAN_CHAN30, AGC_CH030_ABORTDPS))
    {
        return (ab_abrtjask(false)); /* P70A                    */
    }

    if (!AGC_ASSERT(mm != 70 && mm != 71))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_landisp());
} /* agc_lm_abort_monitor_resume */

/**********************************************************************
 * LEGAL? (pp. 835-836): the V37 entry is legal only when the program is
 * not already running, aborts are enabled and SERVICER is on the air.
 *
 * ARGUMENTS:
 *   mm  agc_sp_t  I    program asked for, 70 or 71 (MMNUMBER)
 *
 * RETURNS: bool -- true when the V37 request may proceed
 **********************************************************************/
static bool ab_legal(agc_sp_t mm)
{
    if (!AGC_ASSERT(mm == 70 || mm == 71))
    {
        return (false);
    }

    return (agc_erase_modreg() != mm && agc_flag_test(AGC_FLAG_LETABORT) && agc_flag_test(AGC_FLAG_AVEGFLAG));
}

/**********************************************************************
 * ABORTALM (p. 836): TC FALTON, TC RELDSP, POSTJUMP PINBRNCH -- the OPR
 * ERR lamp and back to pinball; the job ends there.
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t ab_abortalm(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM)) /* LM only   */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dsky_falton();
    agc_dsky_state_t dsky; /* DSKY lamps after FALTON */

    if (!AGC_ASSERT(agc_dsky_get_state(&dsky) == AGC_STATUS_OK && dsky.opr_err)) /* OPR ERR lit         */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_dsky_releasedsp(); /* step status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_pinbrnch());
}

/**********************************************************************
 * P70 / P71 (p. 830) selected by V37.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_dps_abort(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!ab_legal(70))
    {
        return (ab_abortalm());
    }

    agc_inhint();
    agc_status_t st = ab_abrtjask(false); /* step status */

    agc_relint();

    if (!AGC_ASSERT(st != AGC_STATUS_OK || agc_erase_modreg() == 70))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * P71 (P70-P71.agc p. 830) selected by V37: LEGAL? then ABRTJASK for
 * the APS abort (ABORTALM when not legal)
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_aps_abort(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!ab_legal(71))
    {
        return (ab_abortalm());
    }

    agc_inhint();
    agc_status_t st = ab_abrtjask(true); /* step status */

    agc_relint();

    if (!AGC_ASSERT(st != AGC_STATUS_OK || agc_erase_modreg() == 71))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * P70A (P70-P71.agc p. 830): restart point 4.27, ABRTJASK again for
 * the DPS abort
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_dps_abort_commanded(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    agc_status_t st = ab_abrtjask(false); /* step status */

    agc_relint();

    if (!AGC_ASSERT(st != AGC_STATUS_OK || agc_erase_modreg() == 70))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * P71A (P70-P71.agc p. 830): restart point 4.31, ABRTJASK again for
 * the APS abort
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_aps_abort_commanded(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    agc_status_t st = ab_abrtjask(true); /* step status */

    agc_relint();

    if (!AGC_ASSERT(st != AGC_STATUS_OK || agc_erase_modreg() == 71))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * TGOCOMP (p. 836): TGO = (TIME - TIG) at 2^17 cs.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 **********************************************************************/
void agc_lm_tgocomp(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return;
    }

    agc_dp_t t = agc_dp_sub(ip, agc_rtb_loadtime(), agc_erase_tig()); /* TIME - TIG, 2^28 cs */

    agc_lm_asc()->tgo = agc_dp_shl(ip, t, 11);

    if (!AGC_ASSERT(agc_rtb_loadtime() >= 0))
    {
        return;
    }
}

/**********************************************************************
 * THROTUP (p. 836): full throttle (THRUST = BIT13, drive enabled).
 **********************************************************************/
void agc_lm_throtup(void)
{
    agc_erase_set_thrust(AGC_LM_FEXTRA);
    agc_chan_or(AGC_CHAN_CHAN14, AGC_CH014_THRUSTACT);

    if (!AGC_ASSERT(agc_erase_thrust() == AGC_LM_FEXTRA))
    {
        return;
    }

    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN14) & AGC_CH014_THRUSTACT) != 0u))
    {
        return;
    }
}

/**********************************************************************
 * BOTHPOLY .. YOK (pp. 833-834): ZDOTD = A + TGO(B + TGO(C + TGO D)),
 * not below VMIN; RDOTD = ABTRDOT; the cross-range target YCO is the
 * part of Y beyond YLIM; XRANGE for N76. aps: the APS coefficients.
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *  I/O  interpreter state (OVFIND)
 *   aps  bool            I    true: APS coefficients (X1 = 8D), false: DPS
 **********************************************************************/
static void ab_bothpoly(agc_interp_t *ip, bool aps)
{
    agc_lm_asc_t       *a = agc_lm_asc();                           /* ascent erasables */
    const agc_lm_pad_t *p = agc_lm_pad();                           /* pad loads */
    int                 b = aps ? 4 : 0;                            /* AXC,1 8D / 0D */
    agc_dp_t            z = agc_dp_mul_trunc(p->abtcof[b], a->tgo); /* ZDOTD partial sum, 2^7 m/cs */

    z        = agc_dp_mul_trunc(agc_dp_add(ip, z, p->abtcof[b + 1]), a->tgo);
    z        = agc_dp_mul_trunc(agc_dp_add(ip, z, p->abtcof[b + 2]), a->tgo);
    a->zdotd = agc_dp_add(ip, z, p->abtcof[b + 3]);

    if (agc_dp_sub(ip, a->zdotd, p->vmin) < 0)
    {
        a->zdotd = p->vmin; /* too small    */
    }

    if (!AGC_ASSERT(a->zdotd >= p->vmin)) /* at least VMIN        */
    {
        return;
    }

    a->rdotd    = p->abtrdot;                                /* UPRATE       */
    a->y        = agc_lm_ycomp(ip);
    agc_dp_t dy = agc_dp_sub(ip, agc_dp_abs(a->y), p->ylim); /* |Y| - YLIM, 2^24 m */

    if (dy >= 0)
    {
        a->yco = agc_dp_sign(dy, a->y); /* deficit      */
    }

    a->xrange = agc_dp_shr(agc_dp_sub(ip, a->yco, a->y), 5); /* YOK  */

    if (!AGC_ASSERT(b == 0 || b == 4))
    {
        return;
    }
}

/**********************************************************************
 * UPTHROT (p. 834): vertical rise, throttle up, restart protect, then
 * P40AUTO checks the panel switches and returns at UPTHROT1.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t ab_upthrot(agc_job_ctx_t *ctx)
{
    agc_flag_set(AGC_FLAG_FLVR);
    agc_flag_set(AGC_FLAG_ROTFLAG);
    agc_lm_throtup();
    const agc_phase_var_t v = {
        AGC_RESTART_JOB, AGC_JOB_ABORT_AUTO_CONTROL_CHECK,
        AGC_TASK_NONE, 0,
        false, 0U
    };

    agc_phase_change_var(04024, &v);

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_FLVR))) /* vertical rise */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_ROTFLAG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_job_abort_auto_control_check(ctx));
}

/**********************************************************************
 * UPTHROT1 -3 (p. 834): P40AUTO, returning to UPTHROT1.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_abort_auto_control_check(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_FLVR) || agc_flag_test(AGC_FLAG_FLAP)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_check_auto_control(ctx, AGC_JOB_ABORT_TO_ASCENT_GUIDANCE));
}

/**********************************************************************
 * THROTUP / UPTHROT1 / GRP4OFF (pp. 834-835): SERVICER now calls the
 * ascent guidance; group 4 is no longer used.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_abort_to_ascent_guidance(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_throtup();
    agc_guid_state()->avg.avgexit = AGC_JOB_ATMAG;
    agc_phase_change(4, 00004);

    if (!AGC_ASSERT(agc_erase_thrust() == AGC_LM_FEXTRA)) /* full thrust */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK); /* ENDOFJOB     */
}

/**********************************************************************
 * P70INIT (p. 833): DPS engine data and targets for P70.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *   ip   agc_interp_t *   I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t ab_dps_abort_init(agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_asc_t *a    = agc_lm_asc();              /* ascent erasables */
    agc_dp_t      mass = agc_guid_state()->lm.mass; /* MASS, 2^16 kg */

    if (!AGC_ASSERT(mass > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_tgocomp(ip);
    a->tbup     = agc_dp_div(ip, mass, agc_dp_shl(ip, AB_MDOTDPS, 4));
    agc_dp_t dv = agc_dp_shr(agc_dp_div(ip, mass, AB_K1DV), 1); /* 1/DV1 = MASS / K(1/DV), SR1, 2^8 */

    a->dv[1] = dv;
    a->dv[2] = dv;
    a->dv[3] = dv;
    a->at    = agc_dp_div(ip, AB_KAT, dv);
    a->tto   = agc_dp_shl(ip, agc_dp_neg(AB_DTDECAY), 11);
    a->ve    = agc_dp_shr(agc_dp_from_sp((agc_sp_t) -AB_DPSVEX), 2);

    if (!AGC_ASSERT(dv > 0 && a->tbup > 0)) /* positive mass terms */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_set(AGC_FLAG_FLAP);
    agc_lm_comminit(ip);
    ab_bothpoly(ip, false);

    return (ab_upthrot(ctx));
}

/**********************************************************************
 * OLDTIME restart point (p. 835): TGO = TGO1, then UPTHROT1 -3.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_aps_abort_oldtime(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_asc()->tgo = agc_lm_asc()->tgo1;

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_FLAP)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_job_abort_auto_control_check(ctx));
}

/**********************************************************************
 * P71RET (p. 835): ascent threshold; after P70 (FLAP) the remaining TGO
 * is doubled (OLDTIME) and the targets kept; otherwise TGO from TIG and
 * the APS injection targets (INJTARG).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *   ip   agc_interp_t *   I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t ab_aps_abort_return(agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_asc_t *a = agc_lm_asc(); /* ascent erasables */

    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_clear(AGC_FLAG_LETABORT);
    agc_guid_state()->lm.dvthrush = AB_THRESH2;

    if (!AGC_ASSERT(agc_erase_modreg() == 71)) /* P71RET: in P71   */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_flag_test(AGC_FLAG_FLAP))
    {
        a->tgo1                 = agc_dp_shl(ip, a->tgo, 1); /* OLDTIME      */
        agc_lm_powered_ascent_init(ip);
        const agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_APS_ABORT_OLDTIME,
                                   AGC_TASK_NONE, 0,
                                   false, 0U};

        agc_phase_change_var(04024, &v);

        return (agc_lm_job_aps_abort_oldtime(ctx));
    }

    agc_lm_tgocomp(ip);
    agc_lm_powered_ascent_init(ip); /* to INJTARG   */
    ab_bothpoly(ip, true);

    return (ab_upthrot(ctx));
}

/**********************************************************************
 * GOABORT (pp. 832-833): the 4.3 restart after the abort ENEMA.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_goabort(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_sp_t mm = agc_erase_modreg(); /* MODREG set by ABRTJASK, unscaled */

    if (!AGC_ASSERT(mm == 70 || mm == 71))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip; /* interpreter state (OVFIND) */

    agc_interp_init(&ip);
    agc_lm_initcduw();
    agc_guid_lm_t *g = &agc_guid_state()->lm; /* LM guidance state */

    g->dvcntr = 4;
    g->which  = AGC_LM_WHICH_ABORT;
    agc_flag_clear(AGC_FLAG_FLRCS);
    agc_flag_clear(AGC_FLAG_FLUNDISP);
    agc_flag_clear(AGC_FLAG_IDLEFLAG);
    agc_flag_set(AGC_FLAG_ACC4_2FL); /* 4-jet translation       */

    if (!agc_phase_checkmm(70))
    {
        return (ab_aps_abort_return(ctx, &ip));
    }

    return (ab_dps_abort_init(ctx, &ip));
}
