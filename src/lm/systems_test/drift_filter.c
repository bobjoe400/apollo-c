/**********************************************************************
 * FILE NAME: drift_filter.c
 *
 * PURPOSE:
 *   IMU_PERFORMANCE_TESTS_4: the gyro drift estimation -- ESTIMS (filter
 *   start), ALLOOP / ALFLT once per 1SECXT (NORMLOP, ALKCG, DELMLP, ALKLP,
 *   LOOSE / BOOP), PERFERAS, SETUPER1 (torque the gyros by the drift
 *   angles), GEOSTRT4, SLEEPIE and SOMERR2.
 *
 * AGC SOURCE:
 *   Luminary099 IMU_PERFORMANCE_TESTS_4.agc pp.382-389
 *
 * EXTERNAL VARIABLES:
 *   (AGC erasables reached through accessors or by ECADR)
 *   Name          Type          I/O  Scale     Description
 *   ------------  ------------  ---  --------  ----------------------------
 *   W .. AINLA+99 uint16_t[]    I/O  words     the test erasables (st_words())
 *   TRANSM1,ALFDK uint16_t[]    I    words     filter matrix and gains (E4)
 *   OGC           agc_dp_t[3]   O    rev 2^0   gyro torquing angles
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_mpac_*            the interpreter on AGC words (interp_words.h)
 *   agc_imu_pulse_buf     IMUPULSE (OGC, ERCOMP buffers)
 *   agc_exec_*            VARALARM, FINDVAC, JOBSLEEP
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   VARALARM 01601 (bad IMU moding, SOMERR2).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   LM only; the jobs and tasks of P07 (V92).
 *
 * NOTES:
 *   DEVIATION (ATIGINC): PERFERAS's erasable program is not run (see
 *   job_entries.c); the filter's gains and transition matrix are read
 *   by address so a V21 N01 load reaches them. **********************************************************************/
#include <stddef.h>

#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/interp_words.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/programs/alignment.h"
#include "internal.h"

/* INTVAL +2 .. +7: ALX1S, CMPX1, ALK, ALK +2 (SOUPLY) as words.           */
static const uint16_t intval_words[6] = {000220u, 077776u, 035730u, 000035u, 010317u, 017550u};

static agc_dp_t perferas_vac[6]; /* VAC 10D..20D: sin (2^-3) and cos (2^0) of ANGX..ANGZ */

/**********************************************************************
 * SOMERR2 (p.388): VARALARM 1601 (bad IMU moding), IMUSE down, ENDOFJOB.
 *
 * RETURNS: none
 **********************************************************************/
void somerr2(void)
{
    agc_exec_varalarm(ALARM_BAD_IMU);
    agc_flag_clear(AGC_FLAG_IMUSE);
}

/**********************************************************************
 * SLEEPIE +1 (p.388): CCS TORQNDX -- vertical drift: EARTHR*; ENDOFJOB.
 *
 * RETURNS: AGC_STATUS_OK (ENDOFJOB) or the EARTHR* status
 **********************************************************************/
static agc_status_t sleepie_tail(void)
{
    if (w_sp(W_TORQNDX) > 0)
    {
        return (earthr_star(STEP_ENDOFJOB));
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * ESTIMS (p.383): ALLOOP every 1SECXT, PIPAs and the filter zeroed, the
 * initial gains, GCOMPSW, DELV, GCOMP zero; ERTHRVSE unless vertical;
 * SLEEPIE +1.
 *
 * RETURNS: the TWIDDLE (1SECXT out of range) or ZEROING status, or that
 *          of SLEEPIE
 **********************************************************************/
agc_status_t estims(void)
{
    uint16_t      *w       = st_words(); /* W .. AINLA +99D, the test erasables */
    const agc_dp_t zero[3] = {0, 0, 0};
    agc_interp_t   ip;                   /* interpreter state */

    agc_inhint();
    agc_status_t st = agc_waitlist_call(w[W_1SECXT], AGC_TASK_SYSTEMS_TEST_SAMPLE); /* TWIDDLE ALLOOP */

    agc_erase_set_pipax(0);
    agc_erase_set_pipay(0);
    agc_erase_set_pipaz(0);
    agc_relint();

    if (st == AGC_STATUS_OK)
    {
        st = agc_systems_test_zeroing(&w[W_ALX1S - 1], ESTIMS_ZEROED); /* ALXXXZ, 77DECML */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    for (int i = 0; i < 6; i++)
    {
        w[W_ALX1S + i] = intval_words[i]; /* STOVL ALX1S: INTVAL +2 */
    }

    st = agc_imu_set_gcompsw(0); /* STOVL GCOMPSW -1      */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_erase_set_delv(agc_vec_zero());
    agc_imu_gyro_buf_set(AGC_IMU_GBUF_GCOMP, zero);

    if (w_sp(W_TORQNDX) <= 0)
    {
        agc_interp_init(&ip);
        erthrvse(&ip); /* DCOMP BMN VERTSKIP */
    }

    return (sleepie_tail());
} /* estims */

/**********************************************************************
 * GEOSTRT4 (p.388): vertical drift -> VALMIS; else ERTHRVSE, TORQUE.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the display status (TORQUE or VALMIS)
 **********************************************************************/
agc_status_t geostrt4(agc_job_ctx_t *ctx)
{
    uint16_t    *w = st_words(); /* W .. AINLA +99D, the test erasables */
    agc_interp_t ip;             /* interpreter state */

    if (w_sp(W_TORQNDX) > 0)
    {
        /* VALMIS (p.377): DSPTEM2 = 0, DRIFTO; SHOW, then ENDTEST1.         */
        agc_dsky_set_dsptem(4, w[W_DRIFTO]);
        agc_dsky_set_dsptem(3, 0);

        return (show(STEP_ENDTEST1, ctx));
    }

    agc_interp_init(&ip);
    erthrvse(&ip);

    return (torque(ctx));
}

/**********************************************************************
 * SETUPER1 (p.388) after IMUPULSE: IMUSLLLG, GEOSTRT4.
 *
 * ARGUMENTS:
 *   pulse  agc_status_t     I   the IMUPULSE status
 *   ctx    agc_job_ctx_t *  I   the running job
 *
 * RETURNS: AGC_STATUS_OK (asleep), the IMUPULSE or stall status, or that
 *          of GEOSTRT4
 **********************************************************************/
agc_status_t setuper1_stall(agc_status_t pulse, agc_job_ctx_t *ctx)
{
    bool good = false; /* torquing over and good */

    if (pulse == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* GYROBUSY: asleep          */
    }

    if ((pulse != AGC_STATUS_OK) && (pulse != AGC_STATUS_ALARM))
    {
        return (pulse);
    }

    agc_status_t st = imu_stall(AGC_JOB_SYSTEMS_TEST_TORQUED, &good); /* IMUSLLLG status */

    if (!good)
    {
        return (st);
    }

    return (geostrt4(ctx));
}

/**********************************************************************
 * SETUPER1 (p.388): OGC = XSM (-(ANGX, ANGY, ANGZ) GEORGEJ) / 2, IMUPULSE.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the IMUPULSE / IMUSLLLG / GEOSTRT4 status
 **********************************************************************/
static agc_status_t setuper1(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                                                                   /* interpreter state */
    agc_vec_t    ang = agc_vec_make(w_dp(W_ANGX), w_dp(W_ANGX - 2), w_dp(W_ANGX - 6)); /* VDEF ANGX, ANGY, ANGZ: rev 2^0 */

    agc_interp_init(&ip);
    ang = agc_vec_scale(&ip, agc_vec_neg(ang), GEORGEJ);                /* VCOMP, VXSC */
    ang = agc_vec_shr(agc_mat_vec(&ip, &agc_align_ram()->xsm, ang), 1); /* MXV, VSR1   */

    for (int i = 0; i < 3; i++)
    {
        agc_imu_ogc()[i] = ang.c[i]; /* STORE OGC */
    }

    return (setuper1_stall(agc_imu_pulse_buf(AGC_IMU_GBUF_OGC, AGC_JOB_SYSTEMS_TEST_TORQUE_RETRY), ctx));
}

/**********************************************************************
 * PERFERAS (p.387): TC ATIGINC (not run, see DEVIATION); CCS LENGTHOT --
 * passes left: SLEEPIE (TS LENGTHOT = LENGTHOT - 1); done: LOSVEC +1 =
 * CDUX when vertical (or LENGTHOT < 0), SETUPER1.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the SLEEPIE or SETUPER1 status
 **********************************************************************/
static agc_status_t perferas(agc_job_ctx_t *ctx)
{
    uint16_t *w   = st_words();         /* W .. AINLA +99D, the test erasables */
    st_ccs_t  len = ccs(w[W_LENGTHOT]); /* CCS LENGTHOT */

    if (len.branch == 0)
    {
        w[W_LENGTHOT] = len.a; /* SLEEPIE: TS LENGTHOT */

        return (sleepie_tail());
    }

    st_ccs_t torq = ccs(w[W_TORQNDX]); /* CCS TORQNDX (from LENGTHOT +0) */

    if (len.branch == 2 || (len.branch == 1 && torq.branch != 1))
    {
        w[W_LOSVEC + 1] = (len.branch == 1 && torq.branch == 3) ? 0u : agc_erase_cdux(); /* FOR TROUBLESHOOTING */
    }

    return (setuper1(ctx));
}

/**********************************************************************
 * ALFLT / NORMLOP (p.385): 1/PIPA unless GEOCOMPS is +0; DPIPAY = -(DELV
 * XSM 2).y, DPIPAZ = (DELV XSM 2).z.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I   interpreter state (OVFIND)
 *
 * RETURNS: AGC_STATUS_OK or the 1/PIPA status
 **********************************************************************/
static agc_status_t normlop(agc_interp_t *ip)
{
    uint16_t *w = st_words();       /* W .. AINLA +99D, the test erasables */
    agc_vec_t d = agc_erase_delv(); /* DELV, PIPA pulses x 2^14 */

    if (ccs(w[W_GEOCOMPS]).branch != 1)
    {
        agc_status_t st = agc_imu_one_over_pipa(&d); /* CCS GEOCOMPS: 1/PIPA */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        agc_erase_set_delv(d);
    }

    /* VXM XSM: component j = sum over i of DELV_i XSM_ij (DMP products as
     * DOT sums them), then VSL1; DPIPAY = DCOMP of Y, DPIPAZ = Z.         */
    const agc_mat_t *xsm = &agc_align_ram()->xsm; /* XSM, half-unit */

    for (int j = 1; j < 3; j++)
    {
        agc_tp_words_t m = agc_mpac_dmp(agc_mpac_dload(agc_dp_to_words(d.c[0])), agc_dp_to_words(xsm->row[0].c[j])); /* component j of DELV XSM, pulses x 2^14 */

        for (int i = 1; i < 3; i++)
        {
            m = agc_mpac_tad(ip, m, agc_mpac_dmp(agc_mpac_dload(agc_dp_to_words(d.c[i])), agc_dp_to_words(xsm->row[i].c[j])));
        }

        m = agc_mpac_shift(ip, agc_mpac_dload(agc_mpac_store(m)), 1);

        if (j == 1)
        {
            w_store(W_DPIPAY, agc_mpac_dcomp(m)); /* DCOMP, STORE DPIPAY */
        }
        else
        {
            w_store(W_DPIPAY + 4, m); /* STORE DPIPAZ */
        }
    }

    return (AGC_STATUS_OK);
} /* normlop */

/**********************************************************************
 * ALKCG / ALKCG2 (p.385): the next 12 gain words from ALFDK (ALTIM, ALTIMS
 * and ALDK .. ALDK +9), ALX1S -= 12.
 *
 * RETURNS: AGC_STATUS_OK, or the image's status (an ALX1S that points out
 *          of erasable memory)
 **********************************************************************/
static agc_status_t alkcg(void)
{
    int          x1 = (int) w_sp(W_ALX1S); /* X1 = ALX1S, gain word index */
    agc_status_t st = AGC_STATUS_OK;       /* image status */

    for (int k = 0; k < 6 && st == AGC_STATUS_OK; k++)
    {
        agc_dp_words_t g = {0, 0};

        st = image_words((int) EC_ALFDK + ALFDK_WORDS - x1 + 2 * k, &g); /* DLOAD* ALFDK +144D,1 */
        w_set_sp(W_ALDK - 2 + 2 * k, g.hi);                              /* STORE ALDK +10D,2    */
        w_set_sp(W_ALDK - 1 + 2 * k, g.lo);
    }

    w_set_sp(W_ALX1S, (agc_sp_t) (x1 - 12)); /* SXA,1 ALX1S */

    return (st);
}

/**********************************************************************
 * DELMLP / ALILP (pp.385-386): level PIPA increments into INTY, INTZ and
 * the measurements DELM; ALK, ALK +2 scaled by ALDK.  Word for word: the
 * DAD / DSU results keep the sign-disagreeing words the AGC stores.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I   interpreter state (OVFIND)
 **********************************************************************/
static void delmlp(agc_interp_t *ip)
{
    static const agc_dp_words_t pipasc = {2139, 1205};   /* 2DEC .13055869  */
    static const agc_dp_words_t velsc  = {-8556, -4822}; /* 2DEC -.52223476 */

    for (int k = 0; k <= 4; k += 4)
    {
        agc_tp_words_t m = agc_mpac_shift_round(ip, agc_mpac_dmp(agc_mpac_dload(w_words(W_DPIPAY + k)), pipasc), 9); /* DMP PIPASC, SLR 9D */

        m                     = agc_mpac_bdsu(ip, m, w_words(W_INTY + k));                                              /* BDSU* INTY */
        w_store(W_INTY + k, m);
        agc_dp_words_t pushed = agc_mpac_store(m);                                                                      /* PDDL           */
        agc_tp_words_t v      = agc_mpac_shift_round(ip, agc_mpac_dmp(agc_mpac_dload(velsc), w_words(W_VLAUN + k)), 2); /* DMP* VLAUN, SL2R */

        v = agc_mpac_dsu(ip, v, pushed); /* DSU (push-down), STADR */
        w_store(W_DELM + k, v);
        w_store(W_DELM + k + 2, v);
    }

    for (int k = 0; k <= 2; k += 2)
    {
        w_store(W_ALK + k, agc_mpac_dmpr(ip, agc_mpac_dload(w_words(W_ALK + k)), w_words(W_ALDK + k))); /* ALILP: DMPR* ALDK */
    }
}

/**********************************************************************
 * ALKLP (p.386): the four filter updates (CMPX1 alternates ALK / ALK +2).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I   interpreter state (OVFIND)
 **********************************************************************/
static void alklp(agc_interp_t *ip)
{
    static const agc_dp_words_t alsk[2] = {{2839, 5503}, {-136, -14205}}; /* ALSK, ALSK +2 */

    for (int o = 0; o <= 6; o += 2)
    {
        w_set_sp(W_CMPX1, (agc_sp_t) -w_sp(W_CMPX1));                                /* LXC,1 SXA,1 CMPX1 */
        int            a = (w_sp(W_CMPX1) > 0) ? 0 : 2;                              /* ALK +1 - X1        */
        agc_dp_words_t d = w_words(W_DELM + o);                                      /* DELM measurement */
        agc_tp_words_t m = agc_mpac_dmpr(ip, agc_mpac_dload(w_words(W_ALK + a)), d); /* MPAC */

        w_store(W_INTY + o, agc_mpac_dad(ip, m, w_words(W_INTY + o)));                 /* DAD* INTY */
        m = agc_mpac_dad(ip, agc_mpac_dload(w_words(W_ALK + 4 + o)), w_words(W_ALDK + 4 + o));
        w_store(W_ALK + 4 + o, m);
        m = agc_mpac_dad(ip, agc_mpac_dmpr(ip, m, d), w_words(W_INTY + 8 + o));        /* DMPR* DELM, DAD* INTY +16D */
        w_store(W_INTY + 8 + o, m);
        m = agc_mpac_shift_round(ip, agc_mpac_dmp(agc_mpac_dload(alsk[a / 2]), d), 1); /* DMP* DELM, SL1R */
        w_store(W_VLAUN + o, agc_mpac_dad(ip, m, w_words(W_VLAUN + o)));
    }
}

/**********************************************************************
 * LOOSE / BOOP (pp.386-387): (POSNV, VLAUN, ACCWD) through TRANSM1 for Y
 * and Z; sines and cosines of the drift angles for the erasable program.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I   interpreter state (OVFIND)
 *
 * RETURNS: AGC_STATUS_OK or the image's read status of TRANSM1
 **********************************************************************/
static agc_status_t loose_boop(agc_interp_t *ip)
{
    agc_dp_words_t t[TRANSM1_WORDS / 2]; /* TRANSM1 row by row, as stored */
    agc_status_t   st = AGC_STATUS_OK;   /* image status */

    for (int i = 0; i < TRANSM1_WORDS / 2 && st == AGC_STATUS_OK; i++)
    {
        st = image_words((int) EC_TRANSM1 + 2 * i, &t[i]);
    }

    for (int k = 0; k <= 4 && st == AGC_STATUS_OK; k += 4)
    {
        const int      out[3] = {W_POSNV + k, W_VLAUN + k, W_ACCWD + k};
        agc_dp_words_t v[3]   = {w_words(out[0]), w_words(out[1]), w_words(out[2])}; /* VDEF */
        agc_tp_words_t r[3];                                                         /* MXV components (TP MPAC) */

        for (int i = 0; i < 3; i++)
        {
            /* MXV: row i . v, the three DMP products summed (DOT), VSL1.    */
            r[i] = agc_mpac_dmp(agc_mpac_dload(v[0]), t[3 * i]); /* DMPSUB: MPAC x operand */

            for (int j = 1; j < 3; j++)
            {
                r[i] = agc_mpac_tad(ip, r[i], agc_mpac_dmp(agc_mpac_dload(v[j]), t[3 * i + j]));
            }
        }

        for (int i = 0; i < 3; i++)
        {
            w_store(out[i], agc_mpac_shift(ip, agc_mpac_dload(agc_mpac_store(r[i])), 1));
        }
    }

    for (int j = 0; j < 3; j++)
    {
        static const int ang[3] = {W_ANGX, W_ANGX - 2, W_ANGX - 6};                           /* ANGX, ANGY, ANGZ */
        agc_dp_t         m      = agc_dp_shr_round(agc_dp_mul(ip, w_dp(ang[j]), GEORGEJ), 2); /* DMPR GEORGEJ, SR2R: rev x 2/pi 2^2 */

        perferas_vac[j]     = agc_dp_shl_round(ip, agc_dp_sin(m), 3); /* SIN SL3R: 10D, 12D, 14D */
        perferas_vac[j + 3] = agc_dp_cos(m);                          /* COS: 16D, 18D, 20D       */
    }

    return (st);
} /* loose_boop */

/**********************************************************************
 * ALFLT (pp.385-387): the drift filter, one pass per ALLOOP sample.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the NORMLOP, ALKCG or LOOSE status, or that of PERFERAS
 **********************************************************************/
agc_status_t alflt(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                 /* interpreter state */
    agc_status_t st = AGC_STATUS_OK; /* status of the filter steps */

    agc_interp_init(&ip);
    st = normlop(&ip);

    if (st == AGC_STATUS_OK && w_sp(W_GEOCOMPS) <= 0)
    {
        /* ALCGKK: new gains unless ALTIMS < 0; ALFLT3.                      */
        if (w_sp(W_ALTIMS) >= 0)
        {
            st = alkcg();
        }

        if (st == AGC_STATUS_OK)
        {
            delmlp(&ip);
            alklp(&ip);
            st = loose_boop(&ip);
        }
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (perferas(ctx));
}

/**********************************************************************
 * ALLOOP (p.384): stop on OVFLOWCK; step ALTIM; next sample in 1SECXT
 * while passes are left; DELV = PIPAs (zeroed); SPECSTS: FINDVAC ALFLT.
 *
 * RETURNS: AGC_STATUS_OK (TASKOVER), or the TWIDDLE or FINDVAC status
 **********************************************************************/
agc_status_t alloop(void)
{
    uint16_t *w = st_words(); /* W .. AINLA +99D, the test erasables */

    if (w[W_OVFLOWCK] != 0u)
    {
        return (AGC_STATUS_OK); /* TASKOVER */
    }

    st_ccs_t c = ccs(w[W_ALTIM]); /* CCS ALTIM */

    if (c.branch <= 1)
    {
        w[W_ALTIMS] = c.a;                       /* CA A, TS ALTIMS (+0: 0) */
        w[W_ALTIM]  = (uint16_t) (WORD15 - c.a); /* CS A, TS ALTIM          */
    }
    else
    {
        w[W_ALTIM] = (c.branch == 2) ? (uint16_t) (WORD15 - c.a) : 0u; /* -0: TS ALTIM with A = 0 */
    }

    agc_status_t st  = AGC_STATUS_OK;           /* TWIDDLE status */
    bool         geo = (w_sp(W_GEOCOMPS) == 1); /* GEOCOMPS = 1: sample on */

    if (geo || (((w[W_LENGTHOT] & 040000u) == 0u) && w[W_LENGTHOT] != 0u))
    {
        st = agc_waitlist_call((uint16_t) w[W_1SECXT], AGC_TASK_SYSTEMS_TEST_SAMPLE); /* TWIDDLE ALLOOP */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_vec_t      d       = agc_erase_delv(); /* DELV, PIPA pulses x 2^14 */
    const agc_sp_t pipa[3] = {agc_erase_pipax(), agc_erase_pipay(), agc_erase_pipaz()};

    for (int i = 0; i < 3; i++)
    {
        pipa_set(i, 0);                                    /* CAF ZERO, XCH PIPAX */
        d.c[i] = agc_dp_pack(pipa[i], agc_dp_low(d.c[i])); /* TS DELVX            */
    }

    agc_erase_set_delv(d);

    return (agc_exec_findvac(PRIO20, AGC_JOB_SYSTEMS_TEST_FILTER)); /* SPECSTS */
} /* alloop */
