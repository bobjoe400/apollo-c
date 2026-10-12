/**********************************************************************
 * FILE NAME: steer.c
 *
 * PURPOSE:
 *   The CSM 2-second steering cycle run from AVGEXIT: UPDATEVG (with
 *   SETUP.9 / SURELY.9), S40.8 (VG update, low-thrust test, TGOCALC,
 *   XPRODUCT, S40.81, LOTHRUST), STEERING (engine-off set-up), CALCN85,
 *   P40CNV85, CALCN83, SERVXT, the S40.9 Lambert job (ENDLAMB, FIRSTTME,
 *   COPY40.9, REDO40.9) and the S40.13 TIMEBURN job.
 *
 * AGC SOURCE:
 *   Comanche055 P40-P47.agc pp. 689-728
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale        Description
 *   ------------  ---------  ---  -----------  -----------------------
 *   VGTIG         agc_vec_t  I/O  m/cs 2^7     VGPREV (same erasable)
 *   TIG, TGO      agc_dp_t   I/O  cs 2^28      engine-off time, time to go
 *   RN, VN        agc_vec_t  I    2^29, 2^7    state (S40.9)
 *   PIPTIME       agc_dp_t   I    cs 2^28      time of RN, VN
 *   REFSMMAT      agc_mat_t  I    2^1          SM to reference
 *   DAPDATR1      uint16_t   I    -            LM attached (REPFRAC)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_guid_steering_tgo_csm        TGOCALC (burn_steering.c)
 *   agc_guid_steering_cross_product  XPRODUCT (burn_steering.c)
 *   agc_guid_timeburn_csm            S40.13 (burn_steering.c)
 *   agc_rendz_initvel                INITVEL for S40.9 (rendezvous.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ALARM 01407 (VG increasing, INCRSVG). Lambert, executive and
 *   waitlist failures come back as their status.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only; runs in the SERVICER job (AVGEXIT) every 2 s and as the
 *   S40.9 / S40.13 jobs. Restart groups 1, 3 and 5: the type B phase
 *   changes resume in AGC_JOB_CROSS_PRODUCT_RESTART, UPDATEVG_RESTART,
 *   SURELY9_RESTART, STEERING_RESTART and CALC_DELTA_V_RESTART.
 *
 * NOTES:
 *   csm.updcaller records which AVGEXIT routine called UPDATEVG (the
 *   AGC keeps that in QTEMP1).
 *
 * ALGORITHM:
 *   As in the listing; scales: VG, BDT, DELVREF m/cs 2^7, TGO cs.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/guidance/guidance.h"
#include "internal.h"

#define CALLER_STEERING     1                   /* UPDATEVG called from STEERING */
#define CALLER_CALC_VG_BODY 2                   /* UPDATEVG called from CALCN85 */
#define PH_5_3_B            ((uint16_t) 010035) /* B: 5.3 REREADAC + job */

/* 2VEXHUST 2DEC 63.020792 B-7: 2 (10 338.0564 ft/s), m/cs 2^7.            */
static const agc_dp_t TWOVEXHUST = 132164180; /* 2VEXHUST, m/cs 2^7 */

/* FOURSEC 2DEC 400.0 B-28; DPB-9 2DEC 1 B-9.                              */
static const agc_dp_t FOURSEC = 400;    /* FOURSEC, cs 2^28 */
static const agc_dp_t DPB_9   = 524288; /* DPB-9: 1 at 2^9 */

/* 200CSHI 2DEC 200 B-12; EARTHMU 2DEC* -3.986032 E10 B-36*.               */
static const agc_dp_t CS200HI = 13107200;   /* 200CSHI, cs 2^12 */
static const agc_dp_t EARTHMU = -155704375; /* EARTHMU, m^3/cs^2 2^36 */

/* EP4(45)L .125, EP4(10)L .027777777: EPSILON4 for HAVEGUES.              */
static const agc_dp_t EP4_45L = 33554432; /* EPSILON4 45 deg, rev 2^0 */
static const agc_dp_t EP4_10L = 7456540;  /* EPSILON4 10 deg, rev 2^0 */

/**********************************************************************
 * PHASCHNG type B for group 5 (5.3 REREADAC + this job at PRIO20).
 * AGC: none (helper for the type B PHASCHNG 5.3 calls of S40.8,
 * UPDATEVG and STEERING, pp.699-704, 719).
 *
 * FUNCTION NAME: phase_b
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   job       agc_job_id_t  I    restart job of the type B group 5 phase
 **********************************************************************/
static void phase_b(agc_job_id_t job)
{
    agc_phase_var_t v = {AGC_RESTART_JOB, job, AGC_TASK_NONE, (int16_t) AGC_PRIO20, false, 0U};

    agc_phase_change_var(PH_5_3_B, &v);
}

/**********************************************************************
 * S40.81 (p.721): TGO < 4 s: IMPULSW set (engine-off call), OMEGAC = 0
 * (TVC attitude hold), CNTR = POSMAX (no switch-over/CG tracking),
 * STEERSW reset.
 *
 * FUNCTION NAME: short_burn_cutoff
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   tvc       agc_csm_tvc_t *  I/O  CSM TVC DAP erasables (agc_csm_tvc_t)
 **********************************************************************/
static void short_burn_cutoff(agc_csm_tvc_t *tvc)
{
    agc_flag_set(AGC_FLAG_IMPULSW);
    tvc->omegac[0] = 0; /* RATEZRO         */
    tvc->omegac[1] = 0;
    tvc->omegac[2] = 0;
    tvc->cntr      = AGC_SP_MAX;
    agc_flag_clear(AGC_FLAG_STEERSW);
}

/**********************************************************************
 * LOTHRUST (p.722): thrust failure, unless IDLEFAIL (one bypass after V97
 * PROCEED): TVC to attitude hold, mass updating and CG tracking killed,
 * V97 flash by the CLOCKJOB, steering off.
 *
 * FUNCTION NAME: lothrust
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   tvc       agc_csm_tvc_t *  I/O  CSM TVC DAP erasables (agc_csm_tvc_t)
 **********************************************************************/
static void lothrust(agc_csm_tvc_t *tvc)
{
    if (!agc_flag_test(AGC_FLAG_IDLEFAIL))
    {
        tvc->omegac[0]                = 0;
        tvc->omegac[1]                = 0;
        tvc->omegac[2]                = 0;
        tvc->vcntr                    = 0;                                /* CS ZERO: KILL MASS UPD. */
        tvc->vcntrtmp                 = 0;
        tvc->repfrac                  = 0;
        agc_guid_state()->csm.nvword1 = AGC_CSM_NV_ENGINE_FAIL_PROCEDURE; /* ENGINE-FAIL V97 FLASH  */
        agc_flag_clear(AGC_FLAG_STEERSW);
    }
}

/**********************************************************************
 * S40.8 after its restart point (p.719-720): VGPREV = VG; with STEERSW:
 * low-thrust test against DVTHRESH, CG tracking fraction, TGOCALC (TIG =
 * PIPTIME + TGO), S40.81 when TGO < 4 s, else XPRODUCT.
 *
 * FUNCTION NAME: steering_after_vg_update
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_ALARM (an alarm was
 *          raised); otherwise the status of the routine it continues in
 **********************************************************************/
static agc_status_t steering_after_vg_update(agc_interp_t *ip)
{
    agc_guid_t    *g   = agc_guid_state();  /* guidance erasables */
    agc_csm_tvc_t *tvc = agc_csm_dap_tvc(); /* TVC DAP erasables */

    agc_erase_set_vgtig(g->burn.vg); /* VGPREV = VG            */

    if (!agc_flag_test(AGC_FLAG_STEERSW))
    {
        return (AGC_STATUS_OK); /* SKIP TGO AND XPRODUCT  */
    }

    agc_dp_t thr = agc_dp_mul_trunc(agc_dp_from_sp(g->csm.dvthresh), DPB_9); /* DVTHRESH, m/cs 2^7 */

    if (agc_dp_sub(ip, agc_vec_abval(ip, g->avg.delvref), thr) < 0)
    {
        lothrust(tvc);

        return (AGC_STATUS_OK);
    }

    int lmon = ((agc_erase_dapdatr1() & 020000U) != 0U) ? 1 : 0; /* BIT14   */

    tvc->repfrac     = agc_csm_dap_com()->erepfrac[lmon];
    agc_tgo_in_t in  = {g->burn.vg, agc_vec_sub(ip, g->burn.bdt, g->avg.delvref), TWOVEXHUST, agc_dp_shr(agc_dp_from_sp(g->csm.etdecay), 14)};
    agc_dp_t     tgo = 0;                                        /* TGO, cs 2^28 */
    agc_status_t st  = agc_guid_steering_tgo_csm(ip, &in, &tgo); /* status of agc_guid_steering_tgo_csm */

    if (st == AGC_STATUS_ALARM)
    {
        agc_alarm(AGC_ALARM_01407);

        return (AGC_STATUS_ALARM); /* INCRSVG                */
    }

    agc_erase_set_tgo(tgo);
    agc_erase_set_tig(agc_dp_add(ip, tgo, agc_erase_piptime()));

    if (agc_dp_sub(ip, tgo, FOURSEC) < 0)
    {
        short_burn_cutoff(tvc);

        return (AGC_STATUS_OK);
    }

    agc_xprod_in_t xin      = {g->burn.vg, g->avg.delvref, g->burn.bdt, g->burn.csteer, tvc->kprimedt};
    agc_mat_t      refsmmat = agc_erase_refsmmat(); /* REFSMMAT, half-unit rows (2^1) */
    agc_vec_t      om;                              /* OMEGAC commanded rates (XPRODUCT); scale of OMEGAC (csm/dap.h) */

    st = agc_guid_steering_cross_product(ip, &xin, &refsmmat, &g->trig, &om);

    for (int i = 0; i < 3; i++)
    {
        tvc->omegac[i] = om.c[i]; /* OMEGACLC               */
    }

    return (st);
} /* steering_after_vg_update */

/**********************************************************************
 * S40.8 Comanche (p.719): VG = VGPREV + BDT - DELVREF, VGDISP = |VG|,
 * type B restart (5.3 and S40.8B), then the rest.
 *
 * FUNCTION NAME: agc_csm_cross_product_steering
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_csm_cross_product_steering(agc_interp_t *ip)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    g->burn.vg     = agc_vec_add(ip, agc_vec_sub(ip, g->burn.bdt, g->avg.delvref), agc_erase_vgtig()); /* BVSU VAD VGPREV        */
    g->burn.vgdisp = agc_vec_abval(ip, g->burn.vg);
    phase_b(AGC_JOB_CROSS_PRODUCT_RESTART);

    return (steering_after_vg_update(ip));
}

/**********************************************************************
 * SURELY.9 continuation (p.700): the S40.9 initial conditions RINIT VINIT
 * TNIT DELLT4 from the present state, DELVSUM and NBRCYCLS/P cleared.
 *
 * FUNCTION NAME: surely9_restart
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t surely9_restart(agc_interp_t *ip)
{
    agc_guid_t  *g  = agc_guid_state();     /* guidance erasables */
    agc_rendz_t *rz = agc_rendz_erasable(); /* targeting erasables */

    rz->rinit        = agc_erase_rn(); /* RADIUS VECTOR AT T1    */
    rz->vinit        = agc_erase_vn();
    g->burn.tnit     = agc_erase_piptime();
    rz->dellt4       = agc_dp_sub(ip, rz->tpass4, g->burn.tnit);
    g->burn.delvsum  = agc_vec_zero();
    g->burn.nbrcycls = 0;
    g->burn.nbrcyclp = 0;

    return (agc_csm_cross_product_steering(ip));
}

/**********************************************************************
 * SETUP.9 / SURELY.9 (p.700-701): on later passes VGPREV = VGTEMP +
 * NBRCYCLP BDT - DELVSUM (BDT times the cycle count through NORM X1 and
 * VSR* 0-14D,1); start the S40.9 job; 1.5 (REDO40.9) and 5.3.
 *
 * FUNCTION NAME: setup_9
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t setup_9(agc_interp_t *ip)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    if (!agc_flag_test(AGC_FLAG_FIRSTFLG))
    {
        int       k = 0;                                                          /* NORM X1 shift count */
        agc_dp_t  n = agc_dp_norm(agc_dp_from_sp(g->burn.nbrcyclp), &k);          /* NBRCYCLP normalised; scale 2^-k */
        agc_vec_t b = agc_vec_shl(ip, agc_vec_scale(ip, g->burn.bdt, n), 14 - k); /* NBRCYCLP BDT, m/cs 2^7 */

        b = agc_vec_add(ip, b, agc_rendz_erasable()->delveet3);   /* VGTEMP      */
        agc_erase_set_vgtig(agc_vec_sub(ip, b, g->burn.delvsum)); /* VGPREV     */
    }

    agc_status_t st = agc_exec_findvac(AGC_PRIO10, AGC_JOB_VTOGAIN); /* S40.9 job */

    if (st == AGC_STATUS_OK)
    {
        agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_SURELY9_RESTART, AGC_TASK_NONE, (int16_t) AGC_PRIO20, false, 0U};

        agc_phase_change2((uint16_t) 000051, PH_5_3_B, &v);
        st = surely9_restart(ip);
    }

    return (st);
}

/**********************************************************************
 * UPDATEVG (p.699): external delta-V: S40.8 only; aimpoint: every cycle
 * DELVSUM += DELVREF and NBRCYCLS += 1 (restart protected), S40.9 set up
 * when NBRCYCLS < 0.
 *
 * FUNCTION NAME: agc_csm_updatevg
 *
 * ARGUMENTS:
 *   Argument         Type            I/O  Description (scale)
 *   ---------------  --------------  ---  ----------------------------
 *   ip               agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                         registers)
 *   vtogain_started  bool *          O    set when S40.9 (VTOGAIN) was
 *                                         started
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_csm_updatevg(agc_interp_t *ip, bool *vtogain_started)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    agc_status_t st; /* status of the steering branch */

    *vtogain_started = (!agc_flag_test(AGC_FLAG_XDELVFLG)) && (g->burn.nbrcycls < 0);

    if (*vtogain_started)
    {
        st = setup_9(ip);
    }
    else
    {
        if (!agc_flag_test(AGC_FLAG_XDELVFLG))
        { /* aimpoint: accumulate DELVSUM, restart protected */
            g->burn.delvsump = agc_vec_add(ip, g->burn.delvsum, g->avg.delvref);
            g->burn.nbrcyclp = (agc_sp_t) (g->burn.nbrcycls + 1);
            phase_b(AGC_JOB_UPDATEVG_RESTART);
            g->burn.nbrcycls = g->burn.nbrcyclp;
            g->burn.delvsum  = g->burn.delvsump;
        }

        st = agc_csm_cross_product_steering(ip); /* CALL40.8 */
    }

    return (st);
}

/**********************************************************************
 * The rest of STEERING after UPDATEVG (p.701): with IMPULSW and the engine
 * on, set up ENGINOFF at TIG (at least 1 cs ahead), 3.15 and 5.3.
 *
 * FUNCTION NAME: steering_tail
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t steering_tail(void)
{
    if (!agc_flag_test(AGC_FLAG_IMPULSW) || (agc_chan_read(011) & AGC_CSM_SPSON) == 0U)
    {
        return (agc_job_servexit()); /* PRE-IGNITE / POST-OFF  */
    }

    agc_inhint();
    agc_dp_t t  = agc_erase_tig() - (agc_dp_t) agc_time_now_cs(); /* TPAGREE; TIG - TIME, cs 2^28 */
    agc_sp_t lo = agc_dp_low(t);                                  /* TIG - TIME low word, cs; scale 2^14 */
    agc_sp_t dt = (agc_dp_high(t) == 0 && lo > 0) ? lo : 1;       /* CCS: >= 1; engine-off delay, cs (unscaled), at least 1 */

    agc_erase_set_tgo((agc_dp_t) dt);
    agc_status_t st = agc_waitlist_call((uint16_t) dt, AGC_TASK_ENGINOFF); /* status of agc_waitlist_call */

    if (st != AGC_STATUS_OK)
    {
        agc_relint();

        return (st);
    }

    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_STEERING_RESTART, AGC_TASK_NONE, (int16_t) AGC_PRIO20, false, 0U};

    agc_phase_change2((uint16_t) 040153, PH_5_3_B, &v);
    agc_relint();

    return (agc_job_steering_restart());
}

/**********************************************************************
 * STEERING (p.701): the AVGEXIT of P40.
 *
 * FUNCTION NAME: agc_job_steering_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_steering_csm(void)
{
    agc_guid_state()->csm.updcaller = CALLER_STEERING;
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    bool         vtogain_started = false;                                   /* S40.9 job started by UPDATEVG */
    agc_status_t st              = agc_csm_updatevg(&ip, &vtogain_started); /* status of agc_csm_updatevg */

    if (st != AGC_STATUS_OK && st != AGC_STATUS_ALARM)
    {
        return (st);
    }

    return (steering_tail());
}

/**********************************************************************
 * STEERING after its restart point: IMPULSW reset (ENGINOFF is set up).
 * AGC: STEERING restart point (P40-P47.agc p.701-704).
 *
 * FUNCTION NAME: agc_job_steering_restart
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_steering_restart(void)
{
    agc_flag_clear(AGC_FLAG_IMPULSW);

    return (agc_job_servexit());
}

/**********************************************************************
 * P40CNV85 (p.689): VGBODY = S41.1(VGPREV), VG in control axes for N85.
 *
 * FUNCTION NAME: agc_csm_convert_vg_body
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 **********************************************************************/
void agc_csm_convert_vg_body(agc_interp_t *ip)
{
    agc_burn_t *b = &agc_guid_state()->burn; /* burn erasables */

    agc_guid_reference_to_body(ip, AGC_VEHICLE_CSM, agc_erase_vgtig(), &b->vgbody);
}

/**********************************************************************
 * CALCN85 (p.689): AVEGEXIT of P41 / post-burn: UPDATEVG, N85, SERVEXIT.
 *
 * FUNCTION NAME: agc_job_calc_vg_body_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_calc_vg_body_csm(void)
{
    agc_guid_state()->csm.updcaller = CALLER_CALC_VG_BODY;
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    bool         vtogain_started = false;                                   /* S40.9 job started by UPDATEVG */
    agc_status_t st              = agc_csm_updatevg(&ip, &vtogain_started); /* status of agc_csm_updatevg */

    if (st != AGC_STATUS_OK && st != AGC_STATUS_ALARM)
    {
        return (st);
    }

    agc_csm_convert_vg_body(&ip);

    return (agc_job_servexit());
}

/**********************************************************************
 * Restart entries inside the cycle: S40.8B, UPDATEVG_B, SURELY9_B.
 * AGC: S40.8 / UPDATEVG / SURELY.9 restart points (P40-P47.agc
 * pp.699-700, 719).
 *
 * FUNCTION NAME: agc_job_cross_product_restart
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_cross_product_restart(agc_job_ctx_t *ctx)
{
    agc_guid_t  *g = agc_guid_state(); /* guidance erasables */
    agc_interp_t ip;                   /* interpreter state */

    agc_interp_init(&ip);

    if (ctx->id == AGC_JOB_UPDATEVG_RESTART)
    {
        g->burn.nbrcycls = g->burn.nbrcyclp;
        g->burn.delvsum  = g->burn.delvsump;
        agc_status_t st  = agc_csm_cross_product_steering(&ip); /* status of agc_csm_cross_product_steering */

        if (st != AGC_STATUS_OK && st != AGC_STATUS_ALARM)
        {
            return (st);
        }
    }
    else if (ctx->id == AGC_JOB_SURELY9_RESTART)
    {
        agc_status_t sj = surely9_restart(&ip); /* status of surely9_restart */

        if (sj != AGC_STATUS_OK && sj != AGC_STATUS_ALARM)
        {
            return (sj);
        }
    }
    else
    {
        agc_status_t sw = steering_after_vg_update(&ip); /* AGC_JOB_S40_8B         */

        if (sw != AGC_STATUS_OK && sw != AGC_STATUS_ALARM)
        {
            return (sw);
        }
    }

    /* The caller's tail after a restart inside UPDATEVG / S40.8.               */
    if (agc_guid_state()->csm.updcaller == CALLER_CALC_VG_BODY)
    {
        agc_csm_convert_vg_body(&ip);

        return (agc_job_servexit());
    }

    return (steering_tail());
} /* agc_job_cross_product_restart */

/**********************************************************************
 * CALCN83 (p.691): P47 thrust monitor: DV47TEMP (= VG) = DELVCTL +
 * DELVREF, DELVIMU = S41.1(DV47TEMP), S11.1 (V I, H, HDOT for N62), then
 * DELVCTL = DV47TEMP after a 5.3 type B restart point.
 *
 * FUNCTION NAME: agc_job_calc_delta_v_body_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_calc_delta_v_body_csm(void)
{
    agc_guid_t  *g = agc_guid_state(); /* guidance erasables */
    agc_interp_t ip;                   /* interpreter state */

    agc_interp_init(&ip);
    g->burn.vg = agc_vec_add(&ip, g->burn.delvctl, g->avg.delvref);
    agc_guid_reference_to_body(&ip, AGC_VEHICLE_CSM, g->burn.vg, &g->burn.delvimu);
    agc_csm_velocity_altitude_rate(&ip);
    phase_b(AGC_JOB_CALC_DELTA_V_RESTART);

    return (agc_job_calc_delta_v_restart());
}

/**********************************************************************
 * CALCN83 restart point (p.691): DELVCTL (DV47TEMP) = VG under INHINT,
 * then SERVEXIT.
 *
 * FUNCTION NAME: agc_job_calc_delta_v_restart
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_calc_delta_v_restart(void)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    agc_inhint();
    g->burn.delvctl = g->burn.vg; /* GENTRAN DV47TEMP       */
    agc_relint();

    return (agc_job_servexit());
}

/**********************************************************************
 * S40.9 (p.723-724): HAVEGUES (GUESSW reset: use the last COGA) with
 * NUMIT 0, EPSILON4 45 (NORMSW) or 10 degrees; 1 type C at ENDLAMB.
 *
 * FUNCTION NAME: agc_job_vtogain_csm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_vtogain_csm(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_dp_t e4 = agc_flag_test(AGC_FLAG_NORMSW) ? EP4_45L : EP4_10L; /* EPSILON4, rev 2^0 */

    agc_flag_clear(AGC_FLAG_GUESSW);
    agc_status_t st = agc_rendz_havegues(&ip, agc_rendz_erasable(), 0, e4, ctx->id); /* status of agc_rendz_havegues */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_ENDLAMB, AGC_TASK_NONE, (int16_t) AGC_PRIO10, false, 0U};

    agc_phase_change_var((uint16_t) 005021, &v);

    return (agc_job_endlamb());
}

/**********************************************************************
 * ENDLAMB / FIRSTTME (p.724): after the first pass BDT = (VIPRIME -
 * VRPREV) 2 s / (TNIT - TNITPREV) - GDT/2 (VSL1); about the earth VGTEMP +=
 * GOBL/2 (PIPTIME - NOMTIG) EARTHMU / RN^2. Then 1 type C at COPY40.9.
 *
 * FUNCTION NAME: agc_job_endlamb
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_endlamb(void)
{
    agc_guid_t  *g  = agc_guid_state();     /* guidance erasables */
    agc_rendz_t *rz = agc_rendz_erasable(); /* targeting erasables */
    agc_interp_t ip;                        /* interpreter state */

    agc_interp_init(&ip);

    if (!agc_flag_test(AGC_FLAG_FIRSTFLG))
    {
        agc_dp_t  dtn = agc_dp_shl(&ip, agc_dp_sub(&ip, g->burn.tnit, g->burn.tnitprev), 17); /* TNIT - TNITPREV, cs 2^11 */
        agc_dp_t  k   = agc_dp_div(&ip, CS200HI, dtn);                                        /* BDDV 200CSHI; 200 cs / (TNIT - TNITPREV), scale 2^1 */
        agc_vec_t dv  = agc_vec_scale(&ip, agc_vec_sub(&ip, rz->viprime, g->burn.vrprev), k); /* VIPRIME - VRPREV over the interval, m/cs 2^7 */

        g->burn.bdt = agc_vec_shl(&ip, agc_vec_sub(&ip, dv, g->avg.gdt_2), 1);
    }

    if (rz->body.rtx2 == 0)
    { /* NOT MOONCASE */
        /* UNIT RN leaves |RN|^2 at 34D (LVSQUARE, the VSQ of RN).      */
        agc_dp_t rsq = agc_vec_sq(&ip, agc_erase_rn());                                                 /* RN^2, m^2 2^58 */
        agc_dp_t s   = agc_dp_mul_trunc(agc_dp_sub(&ip, agc_erase_piptime(), g->burn.nomtig), EARTHMU); /* (PIPTIME - NOMTIG) EARTHMU, scale per p.724 */

        s            = agc_dp_div(&ip, s, rsq);
        rz->delveet3 = agc_vec_add(&ip, agc_vec_scale(&ip, g->avg.gobl_2, s), rz->delveet3);
    }

    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_COPY40_9, AGC_TASK_NONE, (int16_t) AGC_PRIO10, false, 0U};

    agc_phase_change_var((uint16_t) 004021, &v);

    return (agc_job_copy40_9());
}

/**********************************************************************
 * COPY40.9 / ENDS40.9 (p.724-725): TNITPREV = TNIT, VRPREV = VIPRIME,
 * FIRSTFLG reset, NBRCYCLS = -1 (next UPDATEVG sets up S40.9), group 1 off.
 *
 * FUNCTION NAME: agc_job_copy40_9
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK
 **********************************************************************/
agc_status_t agc_job_copy40_9(void)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    g->burn.tnitprev = g->burn.tnit;
    g->burn.vrprev   = agc_rendz_erasable()->viprime;
    agc_flag_clear(AGC_FLAG_FIRSTFLG);
    g->burn.nbrcycls = -1;

    agc_phase_kill(1);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * REDO40.9 (p.725): restart of S40.9: give up this pass, keep VGPREV.
 *
 * FUNCTION NAME: agc_job_redo40_9
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK
 **********************************************************************/
agc_status_t agc_job_redo40_9(void)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    g->burn.delvsum                = agc_vec_zero();
    g->burn.nbrcyclp               = 0;
    agc_rendz_erasable()->delveet3 = agc_erase_vgtig(); /* VGTEMP      */
    g->burn.nbrcycls               = -1;                /* ENDS40.9 -2 */

    agc_phase_kill(1);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * S40.13 job (p.726-728): TIMEBURN with the present mass, then group 3 off.
 * +X translation: channel 31 bit 7 reads 0 (inverted input).
 *
 * FUNCTION NAME: agc_job_timeburn_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_timeburn_csm(void)
{
    agc_guid_t             *g   = agc_guid_state();  /* guidance erasables */
    const agc_csm_dapcom_t *com = agc_csm_dap_com(); /* CSM DAP common erasables */

    g->burn.weight_g    = agc_dp_from_sp(com->weight_g);
    bool         ullage = (agc_chan_read(031) & (uint16_t) AGC_CH031_PLUSX) == 0U; /* +X translation (channel 31 bit 7 inverted) */
    agc_interp_t ip;                                                               /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st = agc_guid_timeburn_csm(&ip, ullage, !agc_flag_test(AGC_FLAG_NJETSFLG), com->emdot, g->csm.fang); /* status of agc_guid_timeburn_csm */

    agc_phase_change((uint16_t) 000003); /* KILL GROUP 3           */

    return (st);
}
