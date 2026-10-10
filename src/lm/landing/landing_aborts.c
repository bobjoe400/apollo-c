/*
 * landing_aborts.c -- P70-P71.agc (Luminary 099, pp. 829-837): R10,R11 (the
 * quarter-second landing radar lamp / landing analog display / abort
 * monitor task), the DPS (P70) and APS (P71) abort programs, and their
 * targeting: GOABORT, P70INIT, BOTHPOLY (abort injection velocity
 * polynomials), the throttle-up and the hand-over to ascent guidance.
 *
 * Abort entry. In the AGC an abort (from the ABORT/ABORT STAGE switches
 * seen by R10,R11, or V37 P70/P71) goes INHINT, replaces BRUPT and
 * RESUMEs into ABRTJASK, which sets restart group 4 at 4.3 (GOABORT) and
 * requests a software restart (ENEMA). Here ABRTJASK runs as a function
 * and the restart is the executive's deferred ENEMA (exec-sched note).
 */
#include "agc/agc_assert.h"
#include "agc/agc_channels.h"
#include "agc/agc_lm_landing.h"
#include "agc/agc_landing_pending.h"
#include "agc/agc_sysmisc.h"

#define AB_OCT31      25        /* OCT31: R10,R11 period, cs                */
#define AB_OCTAL27    027       /* 4.27 (P70A) / 4.31 (P71A) restart phase  */
#define AB_THRESH2    308       /* THRESH2: DVMON threshold, ascent         */
#define AB_HLITE      020       /* HLITE = BIT5                             */
#define AB_VLITE      004       /* VLITE = BIT3                             */
#define AB_PRIO30     030000    /* PRIO30: channel 11 bits 13 and 14        */
#define AB_BIT13      010000
#define AB_LRBYBIT    040000    /* LRBYBIT = BIT15 of FLGWRD11              */
#define AB_T5_40MS    037774    /* OCT37774: T5RUPT in 40 ms (DAPIDLER)     */
#define AB_MDOTDPS    ((agc_dp_t)4966056)   /* 2DEC 0.1480 B-3 kg/cs       */
#define AB_K1DV       ((agc_dp_t)228956570) /* K(1/DV) 2DEC 436.70 B-9     */
#define AB_KAT        ((agc_dp_t)5368709)   /* K(AT) 2DEC .02              */
#define AB_DTDECAY    ((agc_dp_t)-38)       /* DTDECAY 2DEC -38 (cs)       */
#define AB_DPSVEX     (-15134)  /* DPSVEX DEC* -2.95588868E+1 B-5 m/cs     */

/* PIPCTR1 (= LADQSAVE): the R10,R11 pass count handed to LANDISP. */
static agc_sp_t ab_pipctr1;

agc_sp_t agc_lm_pipctr1(void)
{
    (void)AGC_ASSERT(ab_pipctr1 >= 0);
    (void)AGC_ASSERT(ab_pipctr1 <= 7);
    return ab_pipctr1;
}

/* R10,R11 (p. 829), every 0.25 s while SERVICER runs; on the last pass of
 * the cycle LRHTASK (which continues at R10,R11A).                   */
agc_status_t agc_lm_task_abort_monitor(void)
{
    if (!agc_flag_test(AGC_FLAG_AVEGFLAG)) {
        return AGC_STATUS_OK;            /* let AVGEND take care of group 2 */
    }
    agc_sp_t pipctr = agc_guid_state()->avg.pipctr;     /* = PHSPRDT2     */
    if (pipctr <= 0) {
        return agc_lm_lrhtask();                         /* last pass      */
    }
    ab_pipctr1 = (agc_sp_t)(pipctr - 1);
    agc_status_t st = agc_waitlist_call(AB_OCT31, AGC_TASK_ABORT_MONITOR_RESTART);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return agc_lm_abort_monitor_resume();
}

/* ABRTJASK (pp. 831-832): commit to the abort. aps_abort selects P71 (APS). */
static agc_status_t ab_abrtjask(bool aps_abort)
{
    agc_sp_t mm = aps_abort ? 71 : 70;
    agc_status_t st = agc_phase_newphase(4, (uint16_t)(AB_OCTAL27 + (aps_abort ? 2 : 0)), false);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    agc_erase_set_modreg(mm);
    agc_guid_state()->lm.dispdex = mm;   /* insure DISPDEX is positive     */
    if (aps_abort) {
        (void)agc_flag_set(AGC_FLAG_APSFLAG);   /* prior to the ENEMA      */
    }
    (void)agc_flag_clear(AGC_FLAG_ULLAGFLG);    /* DAPBITS: ullage, drift, */
    (void)agc_flag_clear(AGC_FLAG_DRIFTDFL);    /* and XOVINHIB off        */
    (void)agc_flag_clear(AGC_FLAG_XOVINFLG);
    (void)agc_flag_set(AGC_FLAG_ENGONFLG);
    uint16_t c11 = agc_chan_read(AGC_CHAN_DSALMOUT);    /* engine on if armed */
    st = agc_chan_write(AGC_CHAN_DSALMOUT,
                        (uint16_t)((c11 & ~(unsigned)AB_PRIO30) | AB_BIT13));
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    (void)agc_flagword_set(11, AB_LRBYBIT);     /* terminate R12           */
    (void)agc_flag_set(AGC_FLAG_ALTITUDE_METERS_ONLY);       /* no cross-pointer outputs */
    (void)agc_rendz_clradmod();                 /* RADMODES for R29        */
    agc_erase_set_tevent((agc_dp_t)agc_time_now_cs());
    agc_guid_state()->avg.avgexit = AGC_JOB_SERVEXIT;
    (void)agc_phase_kill(1);
    (void)agc_phase_kill(3);
    (void)agc_phase_kill(6);
    (void)agc_phase_newphase(4, 3, false);      /* 4.3SPOT for GOABORT     */
    (void)agc_rupt_set_time(5, AB_T5_40MS);     /* DAPIDLER in 40 ms       */
    return agc_exec_enema();
}

/* R10,R11A .. P70NOW? (pp. 829-830): flash the LR lamps, then (with
 * aborts enabled) look for the ABORT STAGE or ABORT discretes. */
agc_status_t agc_lm_abort_monitor_resume(void)
{
    if ((agc_erase_imodes33() & 1u) == 0u) {    /* not a lamp test         */
        if (agc_flag_test(AGC_FLAG_HFLSHFLG)) {
            (void)agc_rendz_flip(AB_HLITE);     /* flip H lite             */
        }
        if (agc_flag_test(AGC_FLAG_VFLSHFLG)) {
            (void)agc_rendz_flip(AB_VLITE);     /* flip V lite             */
        }
    }
    agc_sp_t mm = agc_erase_modreg();
    if (!agc_flag_test(AGC_FLAG_LETABORT) || mm == 71) {
        return agc_lm_landisp();                /* proceed to R10          */
    }
    if (agc_lm_chan_bit_present(AGC_CHAN_CHAN30, AGC_CH030_ABORTAPS)) {
        return ab_abrtjask(true);               /* P71A                    */
    }
    if (mm == 70) {
        return agc_lm_landisp();
    }
    if (agc_lm_chan_bit_present(AGC_CHAN_CHAN30, AGC_CH030_ABORTDPS)) {
        return ab_abrtjask(false);              /* P70A                    */
    }
    (void)AGC_ASSERT(mm != 70 && mm != 71);
    return agc_lm_landisp();
}

/* LEGAL? / ABORTALM (pp. 835-836): the V37 entry is legal only when the
 * program is not already running, aborts are enabled and SERVICER is on
 * the air; otherwise the OPR ERR lamp and back to pinball.          */
static bool ab_legal(agc_sp_t mm)
{
    if (agc_erase_modreg() != mm && agc_flag_test(AGC_FLAG_LETABORT) &&
        agc_flag_test(AGC_FLAG_AVEGFLAG)) {
        return true;
    }
    (void)agc_dsky_falton();
    (void)agc_dsky_releasedsp();
    (void)agc_dsky_pinbrnch();
    (void)AGC_ASSERT(mm == 70 || mm == 71);
    return false;
}

/* P70 / P71 (p. 830) selected by V37. */
agc_status_t agc_lm_job_dps_abort(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!ab_legal(70)) {
        return AGC_STATUS_OK;
    }
    agc_inhint();
    agc_status_t st = ab_abrtjask(false);
    agc_relint();
    return st;
}

agc_status_t agc_lm_job_aps_abort(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!ab_legal(71)) {
        return AGC_STATUS_OK;
    }
    agc_inhint();
    agc_status_t st = ab_abrtjask(true);
    agc_relint();
    return st;
}

/* P70A / P71A: restart points 4.27 and 4.31 (ABRTJASK again). */
agc_status_t agc_lm_job_dps_abort_commanded(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    agc_status_t st = ab_abrtjask(false);
    agc_relint();
    return st;
}

agc_status_t agc_lm_job_aps_abort_commanded(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    agc_status_t st = ab_abrtjask(true);
    agc_relint();
    return st;
}

/* TGOCOMP (p. 836): TGO = (TIME - TIG) at 2^17 cs. */
agc_status_t agc_lm_tgocomp(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t t = agc_dp_sub(ip, agc_rtb_loadtime(), agc_erase_tig());
    agc_lm_asc()->tgo = agc_dp_shl(ip, t, 11);
    (void)AGC_ASSERT(agc_rtb_loadtime() >= 0);
    return AGC_STATUS_OK;
}

/* THROTUP (p. 836): full throttle (THRUST = BIT13, drive enabled). */
agc_status_t agc_lm_throtup(void)
{
    agc_erase_set_thrust(AGC_LM_FEXTRA);
    agc_status_t st = agc_chan_or(AGC_CHAN_CHAN14, AGC_CH014_THRUSTACT);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    (void)AGC_ASSERT(AGC_LM_FEXTRA == 4096);
    return st;
}

/* BOTHPOLY .. YOK (pp. 833-834): ZDOTD = A + TGO(B + TGO(C + TGO D)),
 * not below VMIN; RDOTD = ABTRDOT; the cross-range target YCO is the
 * part of Y beyond YLIM; XRANGE for N76. aps: the APS coefficients.  */
static void ab_bothpoly(agc_interp_t *ip, bool aps)
{
    agc_lm_asc_t *a = agc_lm_asc();
    const agc_lm_pad_t *p = agc_lm_pad();
    int b = aps ? 4 : 0;                                  /* AXC,1 8D / 0D */
    agc_dp_t z = agc_dp_mul_trunc(p->abtcof[b], a->tgo);
    z = agc_dp_mul_trunc(agc_dp_add(ip, z, p->abtcof[b + 1]), a->tgo);
    z = agc_dp_mul_trunc(agc_dp_add(ip, z, p->abtcof[b + 2]), a->tgo);
    a->zdotd = agc_dp_add(ip, z, p->abtcof[b + 3]);
    if (agc_dp_sub(ip, a->zdotd, p->vmin) < 0) {
        a->zdotd = p->vmin;                               /* too small    */
    }
    a->rdotd = p->abtrdot;                                /* UPRATE       */
    a->y = agc_lm_ycomp(ip);
    agc_dp_t dy = agc_dp_sub(ip, agc_dp_abs(a->y), p->ylim);
    if (dy >= 0) {
        a->yco = agc_dp_sign(dy, a->y);                   /* deficit      */
    }
    a->xrange = agc_dp_shr(agc_dp_sub(ip, a->yco, a->y), 5);    /* YOK  */
    (void)AGC_ASSERT(b == 0 || b == 4);
}

/* UPTHROT (p. 834): vertical rise, throttle up, restart protect, then
 * P40AUTO checks the panel switches and returns at UPTHROT1.         */
static agc_status_t ab_upthrot(agc_job_ctx_t *ctx)
{
    (void)agc_flag_set(AGC_FLAG_FLVR);
    (void)agc_flag_set(AGC_FLAG_ROTFLAG);
    (void)agc_lm_throtup();
    const agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_ABORT_AUTO_CONTROL_CHECK, AGC_TASK_NONE,
                                0, false, 0U };
    agc_status_t st = agc_phase_change_var(04024, &v);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return agc_lm_job_abort_auto_control_check(ctx);
}

/* UPTHROT1 -3 (p. 834): P40AUTO, returning to UPTHROT1. */
agc_status_t agc_lm_job_abort_auto_control_check(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(agc_flag_test(AGC_FLAG_FLVR) || agc_flag_test(AGC_FLAG_FLAP));
    return agc_lm_check_auto_control(ctx, AGC_JOB_UPTHROT1);
}

/* THROTUP / UPTHROT1 / GRP4OFF (pp. 834-835): SERVICER now calls the
 * ascent guidance; group 4 is no longer used.                       */
agc_status_t agc_lm_job_upthrot1(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)agc_lm_throtup();
    agc_guid_state()->avg.avgexit = AGC_JOB_ATMAG;
    agc_status_t st = agc_phase_change(4, 00004);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return AGC_STATUS_OK;                                 /* ENDOFJOB     */
}

/* P70INIT (p. 833): DPS engine data and targets for P70. */
static agc_status_t ab_dps_abort_init(agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_asc_t *a = agc_lm_asc();
    agc_dp_t mass = agc_guid_state()->lm.mass;
    if (!AGC_ASSERT(mass > 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)agc_lm_tgocomp(ip);
    a->tbup = agc_dp_div(ip, mass, agc_dp_shl(ip, AB_MDOTDPS, 4));
    agc_dp_t dv = agc_dp_shr(agc_dp_div(ip, mass, AB_K1DV), 1);
    a->dv[1] = dv;
    a->dv[2] = dv;
    a->dv[3] = dv;
    a->at = agc_dp_div(ip, AB_KAT, dv);
    a->tto = agc_dp_shl(ip, agc_dp_neg(AB_DTDECAY), 11);
    a->ve = agc_dp_shr(agc_dp_from_sp((agc_sp_t)-AB_DPSVEX), 2);
    (void)agc_flag_set(AGC_FLAG_FLAP);
    agc_status_t st = agc_lm_comminit(ip);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    ab_bothpoly(ip, false);
    return ab_upthrot(ctx);
}

/* OLDTIME restart point (p. 835): TGO = TGO1, then UPTHROT1 -3. */
agc_status_t agc_lm_job_aps_abort_oldtime(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_lm_asc()->tgo = agc_lm_asc()->tgo1;
    (void)AGC_ASSERT(agc_flag_test(AGC_FLAG_FLAP));
    return agc_lm_job_abort_auto_control_check(ctx);
}

/* P71RET (p. 835): ascent threshold; after P70 (FLAP) the remaining TGO
 * is doubled (OLDTIME) and the targets kept; otherwise TGO from TIG and
 * the APS injection targets (INJTARG).                              */
static agc_status_t ab_aps_abort_return(agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_asc_t *a = agc_lm_asc();
    (void)agc_flag_clear(AGC_FLAG_LETABORT);
    agc_guid_state()->lm.dvthrush = AB_THRESH2;
    if (agc_flag_test(AGC_FLAG_FLAP)) {
        a->tgo1 = agc_dp_shl(ip, a->tgo, 1);             /* OLDTIME      */
        agc_status_t st = agc_lm_powered_ascent_init(ip);
        (void)AGC_ASSERT(st == AGC_STATUS_OK);
        const agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_APS_ABORT_OLDTIME,
                                    AGC_TASK_NONE, 0, false, 0U };
        st = agc_phase_change_var(04024, &v);
        (void)AGC_ASSERT(st == AGC_STATUS_OK);
        return agc_lm_job_aps_abort_oldtime(ctx);
    }
    (void)agc_lm_tgocomp(ip);
    agc_status_t st = agc_lm_powered_ascent_init(ip);                 /* to INJTARG   */
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    ab_bothpoly(ip, true);
    return ab_upthrot(ctx);
}

/* GOABORT (pp. 832-833): the 4.3 restart after the abort ENEMA. */
agc_status_t agc_lm_job_goabort(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_status_t st = agc_lm_initcduw();
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    agc_guid_lm_t *g = &agc_guid_state()->lm;
    g->dvcntr = 4;
    g->which = AGC_LM_WHICH_ABORT;
    (void)agc_flag_clear(AGC_FLAG_FLRCS);
    (void)agc_flag_clear(AGC_FLAG_FLUNDISP);
    (void)agc_flag_clear(AGC_FLAG_IDLEFLAG);
    (void)agc_flag_set(AGC_FLAG_ACC4_2FL);   /* 4-jet translation       */
    if (!agc_phase_checkmm(70)) {
        return ab_aps_abort_return(ctx, &ip);
    }
    return ab_dps_abort_init(ctx, &ip);
}
