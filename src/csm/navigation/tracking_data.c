/**********************************************************************
 * FILE NAME: tracking_data.c
 *
 * PURPOSE:
 *   R22 RENDEZVOUS TRACKING DATA PROCESSING ROUTINE, CSM: R22 / REND1 ..
 *   REND18, BVECTORS, GETUM, CMPOS / LMPOS, SHIFTNDX, INITIALW, RENDISP,
 *   RANGERD and the VHF radar RUPT reader VHFREAD. "To process
 *   rendezvous sighting mark data to update the state vector of either
 *   the CSM or LM as defined by the rendezvous navigation program
 *   (P20)." R22 runs as a job started by P20: every 4 s it looks for an
 *   optics mark in MRKBUF2 (left there by R21 / R23) and, once a minute
 *   without one, reads the VHF ranging. A mark is incorporated twice
 *   (the star direction of the optics LOS, first and second pass); the
 *   VHF range once.
 *
 * AGC SOURCE:
 *   Comanche055 P20-P25.agc pp. 576-589 (R22, REND1 .. REND18, RENDISP ..
 *   RENDISP7, RANGERD, RANGERD3, LIGHTON, VHFREAD, VHFGOOD, BYPASS,
 *   GETUM, SHIFTNDX, CMPOS, LMPOS, INITIALW); CSM only
 *
 * EXTERNAL VARIABLES:
 *   Name        Type       I/O  Scale      Description
 *   ----------  ---------  ---  ---------  ---------------------------
 *   MRKBUF2     words      I/O  raw        the optics mark
 *   MARKTIME    agc_dp_t   I/O  cs 2^28    time of the mark
 *   RNRAD       word       I    .01 NM     VHF range
 *   CHAN13 CHAN33  channels I/O -          VHF read, data good
 *   W           agc_incorp_w_t I/O  as integrated
 *   N49DISP     agc_dp_t   O    m 2^29, m/cs 2^7  noun 49
 *   VHFCNT TRKMKCNT agc_sp_t I/O -         mark counters
 *   MARKDOWN    words      O    raw        downlink copy of the mark
 *   RVAR RVARMIN ALTVAR INTVAR WRENDPOS WRENDVEL RMAX VMAX  pad loads
 *   VEHUPFLG CMOONFLG LMOONFLG SOURCFLG INCORFLG RENDWFLG ORBWFLAG
 *   VHFRFLAG SKIPVHF UPDATFLG PDSPFLAG R23FLG DMENFLG  flags  I/O
 *   VHFTIME VHFRANGE RM CSMPOS LEMPOS RCLP UM USTAR (file static)  I/O
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ----------------------------------------
 *   agc_integ_integrv           INTEGRV (both vehicles to MARKTIME)
 *   agc_csm_sxtsm               SXTNB / NBSM of the mark
 *   rz_bvectors                 BVECTORS
 *   agc_incorp_state_deviation  INCORP1
 *   rz_incorp2                  INCORP2
 *   agc_imu_stall_dev           RADSTALL for the VHF read
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms; the tracker fail lamp (TRFAILON) for a refused VHF read.
 *   AGC_STATUS_NO_CONVERGE after TD_PASSES incorporation passes (cannot
 *   happen: at most two). Executive, display and integration statuses
 *   are passed back.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Jobs of P20 (CSM); VHFREAD runs in the radar RUPT.
 *
 * NOTES:
 *   Scales: positions body scale (earth 2^29, moon 2^27; X2 = 0 earth,
 *   -2 moon), unit vectors 2^1, DELTAQ as noted, VARIANCE TP B-40,
 *   VHFRANGE m 2^27, times cs 2^28.
 **********************************************************************/
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/nouns.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

#define TD_SXTVAR ((agc_dp_t) 703687) /* SXTVAR 2DEC 0.04 E-6 B+16  */
#define TD_IMUVAR ((agc_dp_t) 703687) /* IMUVAR 2DEC 0.04 E-6 B+16  */

/* CONVRNGE 2DEC 18.52 B-13: .01 NM to m 2^27                          */
#define TD_CONVRNGE      ((agc_dp_t) 606863) /* CONVRNGE, B-13            */
#define TD_RANGEB14      ((agc_dp_t) 606863) /* RANGEB14 2DEC 303431.7 B-27 */
#define TD_60SECDP       ((agc_dp_t) 6000)   /* 60SECDP                    */
#define TD_PRIO26        ((agc_prio_t) 026)  /* PRIO26 */
#define TD_PRIO27        ((agc_prio_t) 027)  /* PRIO27 */
#define TD_NEG3          ((agc_sp_t) -3)     /* MRKBUF2 = -3: data used    */
#define TD_EMPTY         077776u             /* -1                         */
#define TD_CH13_READ     000011u             /* OCT11: VHF range read bits */
#define TD_CH33_VHF_GOOD 000002u             /* chan 33 bit 2 (0 = good)   */
#define TD_POSMAX        037777u             /* 14 bits */
#define TD_PASSES        4                   /* REND7 passes in one job    */

/* R22 erasables (EBANK= MRKBUF1).                                       */
typedef struct td_csm
{
    agc_dp_t     vhftime;    /* VHFTIME: last VHF reading, cs 2^28        */
    agc_dp_t     vhfrange;   /* VHFRANGE: m 2^27                          */
    uint16_t     rm;         /* RM: raw VHF range (.01 NM), 15 bits       */
    uint16_t     mark[7];    /* MARKTIME, VTEMP: the mark being used      */
    agc_vec_t    csmpos;     /* CSMPOS: body scale                        */
    agc_vec_t    lempos;     /* LEMPOS: body scale                        */
    agc_vec_t    rclp;       /* RCLP: LM - CSM, body scale                */
    agc_vec_t    ucl;        /* UCL: unit RCLP, 2^1                       */
    agc_vec_t    um;         /* UM: optics LOS, reference, 2^1            */
    agc_vec_t    ustar;      /* USTAR: star direction, 2^1                */
    agc_dp_t     n49disp[3]; /* N49DISP: |dR| 2^29, |dV| 2^7, source 1/2 */
    agc_sp_t     tempor1;    /* TEMPOR1: 0 open, -1 PRO, + V32, unscaled  */
    int          x2;         /* SHIFTNDX: 0 earth, -2 moon                */
    agc_incorp_t inc;        /* BVECTOR DELTAQ VARIANCE .. DELTAX         */
} td_csm_t;

static td_csm_t td = {0, 0, 0U, {0U}, {{0, 0, 0}}, {{0, 0, 0}}, {{0, 0, 0}}, {{0, 0, 0}}, {{0, 0, 0}}, {{0, 0, 0}}, {0, 0, 0}, 0, 0, {0}};

/* ---- SHIFTNDX / CMPOS / LMPOS (pp.584, 588) ------------------------------ */
/**********************************************************************
 * X2 = -2 when the vehicle being updated is in the moon's sphere.
 * AGC: SHIFTNDX (P20-P25.agc p.584)
 *
 * FUNCTION NAME: td_shiftndx
 *
 * ARGUMENTS: none
 *
 * RETURNS: int -- X2: -2 when the updated vehicle is in the moon's sphere,
 *          else 0
 **********************************************************************/
static int td_shiftndx(void)
{
    agc_flag_id_t f = AGC_FLAG_LMOONFLG; /* the LM is updated */

    if (agc_flag_test(AGC_FLAG_VEHUPFLG))
    {
        f = AGC_FLAG_CMOONFLG; /* the CSM is updated */
    }

    if (agc_flag_test(f))
    {
        return (-2); /* moon */
    }

    return (0);
}

/**********************************************************************
 * Position = conic + deviation (VSR* 7,2), body scale.
 * AGC: CMPOS, LMPOS (P20-P25.agc p.588)
 *
 * FUNCTION NAME: td_pos
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   v         agc_vehicle_t   I    the vehicle
 *   pos       agc_vec_t *     O    its position (conic + deviation), body
 *                                  scale
 *
 * RETURNS: agc_status_t -- the permanent state status
 **********************************************************************/
static agc_status_t td_pos(agc_interp_t *ip, agc_vehicle_t v, agc_vec_t *pos)
{
    agc_integ_sv_t sv;                              /* permanent state */
    agc_status_t   st = agc_integ_perm_get(v, &sv); /* state */

    *pos = agc_vec_add(ip, rz_vshift(ip, sv.deltav, -(7 - td.x2)), sv.rcv);

    return (st);
}

/**********************************************************************
 * INITIALW (p.588): W = 0 in the 6x6 part, diagonal position and
 * velocity values.
 * AGC: INITIALW (P20-P25.agc p.588)
 *
 * FUNCTION NAME: td_initialw
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   pos       agc_sp_t  I    WRENDPOS, pad scale
 *   vel       agc_sp_t  I    WRENDVEL, pad scale
 **********************************************************************/
static void td_initialw(agc_sp_t pos, agc_sp_t vel)
{
    agc_incorp_w_t *w = &agc_integ_shared_ctx()->w; /* W matrix */

    for (int b = 0; b < 2; b++)
    {
        for (int m = 0; m < 6; m++)
        {
            w->blk[b][m] = agc_vec_zero();
        }
    }

    for (int k = 0; k < 3; k++)
    {
        w->blk[0][k].c[k]     = agc_dp_from_sp(pos);
        w->blk[1][3 + k].c[k] = agc_dp_from_sp(vel);
    }
}

/**********************************************************************
 * GETUM (p.578): the optics LOS of the mark in reference axes (SXTNB,
 * NBSM with the mark's CDUs, x REFSMMAT, VSL1); MARKDOWN for downlink.
 * AGC: GETUM (P20-P25.agc p.578)
 *
 * FUNCTION NAME: td_getum
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   um        agc_vec_t *     O    UM: optics LOS, reference axes, 2^1
 *
 * RETURNS: agc_status_t -- the SXTNB / NBSM status
 **********************************************************************/
static agc_status_t td_getum(agc_interp_t *ip, agc_vec_t *um)
{
    agc_align_sxt_mark_t m; /* the mark */

    m.t    = agc_dp_pack((agc_sp_t) td.mark[0], (agc_sp_t) td.mark[1]);
    m.cduy = td.mark[2];
    m.cdus = td.mark[3];
    m.cduz = td.mark[4];
    m.cdut = td.mark[5];
    m.cdux = td.mark[6];

    for (int i = 0; i < 7; i++)
    {
        agc_erase_set_markdown(i, td.mark[i]);
    }

    agc_vec_t    los      = agc_vec_zero();              /* SM LOS, 2^1 */
    agc_status_t st       = agc_csm_sxtsm(ip, &m, &los); /* SXTNB, NBSM */
    agc_mat_t    refsmmat = agc_erase_refsmmat();        /* REFSMMAT, 2^1 */

    *um = agc_vec_shl(ip, agc_vec_mat(ip, los, &refsmmat), 1);

    return (st);
}

/**********************************************************************
 * BVECTORS (p.576): rz_bvectors on the R22 state.
 * AGC: BVECTORS (P20-P25.agc p.576)
 *
 * FUNCTION NAME: td_bvectors
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 **********************************************************************/
static void td_bvectors(agc_interp_t *ip)
{
    rz_bvectors(ip, td.rclp, td.um, &td.ustar, &td.inc);
}

/* ---- variances (REND9A .. REND10 pp.581-582) --------------------------- */
/**********************************************************************
 * VARIANCE = base |RCLP|^2 (normalized, X1 doubled, plus 2 X2), SR*
 * 0-2,1 to B-40, + INTVAR (SR 25); VHF: at least -RVARMIN.
 * AGC: REND9A .. REND10 (P20-P25.agc pp.581-582)
 *
 * FUNCTION NAME: td_variance
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   base      agc_dp_t        I    the measurement variance (SXTVAR +
 *                                  IMUVAR, ALTVAR + IMUVAR or RVAR), B+16
 **********************************************************************/
static void td_variance(agc_interp_t *ip, agc_dp_t base)
{
    const agc_rendz_nav_pad_t *p  = agc_rendz_nav_pad(); /* pad loads */
    int                        x1 = 0;                   /* NORM index, -(left shifts) */
    agc_dp_t                   n;                        /* 2^(S+X1) */

    n = rz_norm(agc_vec_abval(ip, td.rclp), &x1);
    agc_tp_t v; /* TP, B-40 after the SR* */

    v = agc_dp_mul_tp(agc_dp_sq(n), base);

    x1              = 2 * x1 + 2 * td_shiftndx();      /* XAD,1    */
    v               = agc_tp_shift(ip, v, -(-2 - x1)); /* SR* 0-2,1 */
    agc_tp_t intvar = (agc_tp_t) p->intvar * 8;        /* SR 25, B-40 */

    v = agc_tp_add(ip, intvar, v);

    if (agc_flag_test(AGC_FLAG_SOURCFLG))
    {
        agc_tp_t vmin = (agc_tp_t) p->rvarmin_cm[0] * 268435456 + (agc_tp_t) p->rvarmin_cm[1] * 16384 + p->rvarmin_cm[2]; /* TP, B-40 */

        if (v + vmin < 0)
        { /* BPL      */
            v = (vmin < 0) ? -vmin : vmin; /* ABS of RVARMIN */
        }
    }

    td.inc.variance = v;
}

/**********************************************************************
 * REND7 .. REND9A for an optics mark: UCL; on the first pass UM and the
 * star direction USTAR = UNIT(UCL x UM) (overflow: ignore the mark);
 * BVECTORS, complemented if the LM is corrected. False: ignore.
 * AGC: REND7 .. REND9A (P20-P25.agc pp.580-581)
 *
 * FUNCTION NAME: td_optics_mark
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   st        agc_status_t *  O    the GETUM status
 *
 * RETURNS: bool -- true to incorporate; false: ignore the mark (UCL x UM
 *          overflow)
 **********************************************************************/
static bool td_optics_mark(agc_interp_t *ip, agc_status_t *st)
{
    *st    = AGC_STATUS_OK;
    td.ucl = agc_vec_unit(ip, td.rclp, NULL, NULL);

    if (agc_flag_test(AGC_FLAG_INCORFLG))
    {
        *st         = td_getum(ip, &td.um);
        agc_interp_clear_ovf(ip);
        agc_vec_t c = agc_vec_cross(ip, td.ucl, td.um); /* REND8, 2^2 */

        td.ustar = agc_vec_unit(ip, c, NULL, NULL);

        if (agc_interp_test_ovf(ip))
        {
            return (false); /* REND3    */
        }
    }

    td_bvectors(ip); /* REND9    */

    if (!agc_flag_test(AGC_FLAG_VEHUPFLG))
    {
        td.inc.bvector[0] = agc_vec_neg(td.inc.bvector[0]);
    }

    agc_phase_change(04022);                            /* REND9A   */
    const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad(); /* pad loads */

    if (agc_flag_test(AGC_FLAG_BACKUP_MARKING))
    { /* REND15: COAS */
        td_variance(ip, agc_dp_add(ip, agc_dp_from_sp(p->altvar), TD_IMUVAR));
    }
    else
    {
        td_variance(ip, agc_dp_add(ip, TD_SXTVAR, TD_IMUVAR)); /* sextant */
    }

    return (true);
}

/**********************************************************************
 * REND10A: INCORP1 (6x6); on the first pass the noun 49 values and the
 * RMAX / VMAX check. True: too large (RENDISP).
 * AGC: REND10A (P20-P25.agc p.582)
 *
 * FUNCTION NAME: td_incorp1
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   st        agc_status_t *  O    the INCORP1 status
 *
 * RETURNS: bool -- true when |dR| > RMAX or |dV| > VMAX (RENDISP)
 **********************************************************************/
static bool td_incorp1(agc_interp_t *ip, agc_status_t *st)
{
    rz_flag_put(AGC_FLAG_DMENFLG, false);
    td.inc.dim9 = false;
    *st         = agc_incorp_state_deviation(ip,
                                             &agc_integ_shared_ctx()->w,
                                             &td.inc);
    agc_phase_change(04022);

    if (*st != AGC_STATUS_OK || !agc_flag_test(AGC_FLAG_INCORFLG))
    {
        return (false);
    }

    const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad(); /* pad loads */
    int                        k = 0 - td_shiftndx();   /* SR* 0,2  */

    td.n49disp[1] = rz_shift(ip, agc_vec_abval(ip, td.inc.deltax[1]), -k);
    td.n49disp[0] = rz_shift(ip, agc_vec_abval(ip, td.inc.deltax[0]), -k);

    if (agc_dp_sub(ip, agc_dp_shr(agc_dp_from_sp(p->rmax), 10), td.n49disp[0]) < 0)
    {
        return (true);
    }

    return (agc_dp_sub(ip, agc_dp_from_sp(p->vmax), td.n49disp[1]) < 0);
}

/* ---- the R22 job --------------------------------------------------------- */
/**********************************************************************
 * R22: priority 26; MRKBUF2 = -3 (used); VHFTIME = now; REND1.
 * AGC: R22, REND1 (P20-P25.agc p.576)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_csm(void)
{
    agc_exec_prio_change(TD_PRIO26); /* PRIOCHNG */

    agc_erase_set_mrkbuf2(0, TD_NEG3);
    td.vhftime = agc_rtb_loadtime();
    agc_phase_change(04022); /* REND1    */

    return (rz_waitone(AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_MARK));
}

/**********************************************************************
 * REND1 / REND3 as a job: GRP2PC, WAITONE, then look for a mark (REND1)
 * or the VHF (REND3).
 * AGC: REND1 (P20-P25.agc p.576)
 *
 * FUNCTION NAME: td_wait_for_mark
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the WAITONE status
 **********************************************************************/
static agc_status_t td_wait_for_mark(void)
{
    agc_phase_change(04022);

    return (rz_waitone(AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_MARK));
}

/**********************************************************************
 * REND3 as a job: GRP2PC, WAITONE, then the VHF.
 * AGC: REND3 (P20-P25.agc p.577)
 *
 * FUNCTION NAME: td_wait_for_vhf
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the WAITONE status
 **********************************************************************/
static agc_status_t td_wait_for_vhf(void)
{
    agc_phase_change(04022);

    return (rz_waitone(AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_VHF));
}

/**********************************************************************
 * REND1A / REND2: a mark in MRKBUF2 (first word not negative) is taken
 * (MARKTIME, VTEMP), MRKBUF2 = -3, optics source; else REND3.
 * AGC: REND1A, REND2 (P20-P25.agc p.577)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_csm_mark
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_csm_mark(void)
{
    if (agc_erase_mrkbuf2(0) < 0)
    {
        return (td_wait_for_vhf()); /* REND3A   */
    }

    agc_inhint();

    for (int i = 0; i < 7; i++)
    { /* GENTRAN  */
        td.mark[i] = agc_erase_mrkbuf(2, i);
    }

    agc_erase_set_mrkbuf2(0, TD_NEG3);
    agc_relint();
    rz_set_marktime(agc_dp_pack((agc_sp_t) td.mark[0], (agc_sp_t) td.mark[1]));
    rz_flag_put(AGC_FLAG_SOURCFLG, false); /* optics   */
    agc_phase_change(04022);

    return (agc_job_rendezvous_tracking_data_csm_incorporate()); /* REND4 */
}

/**********************************************************************
 * REND3 after WAITONE: with VHF ranging on (VHFRFLAG) and a minute since
 * the last reading, RANGERD: start the read, MARKTIME = now, SKIPVHF
 * off, RADSTALL.
 * AGC: REND3, RANGERD (P20-P25.agc pp.577, 587)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_csm_vhf
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_csm_vhf(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_dp_t d = agc_dp_sub(&ip, agc_dp_sub(&ip, agc_rtb_loadtime(), TD_60SECDP), td.vhftime); /* cs 2^28 */

    if (!agc_flag_test(AGC_FLAG_VHFRFLAG) || d < 0)
    {
        return (agc_job_rendezvous_tracking_data_csm_mark()); /* REND1A */
    }

    agc_inhint();
    agc_chan_and(AGC_CHAN_CHAN13, (uint16_t) (077777u & ~017u));
    agc_chan_or(AGC_CHAN_CHAN13, TD_CH13_READ);
    agc_relint();
    rz_set_marktime(agc_rtb_loadtime());
    rz_flag_put(AGC_FLAG_SKIPVHF, false);
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING; /* RADSTALL result */
    agc_job_id_t           next;                         /* RANGERD */

    next            = AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_RANGE;
    agc_status_t st = agc_imu_stall_dev(AGC_IMU_DEV_RADAR, next, &res); /* RADSTALL */

    if (st != AGC_STATUS_OK || res == AGC_IMU_STALL_SLEEPING)
    {
        return (st);
    }

    return (rz_continue(AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_RANGE));
}

/**********************************************************************
 * RANGERD after RADSTALL: bad: LIGHTON (tracker lamp when the reading was
 * refused for bad data) and REND1; good: lamp off; RM (.01 NM, 15-bit
 * unsigned; +0 = no range) to VHFRANGE (m 2^27); VHF source; REND4.
 * AGC: RANGERD, RANGERD3, LIGHTON (P20-P25.agc p.587)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_csm_range
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_csm_range(void)
{
    agc_status_t st; /* lamp status */

    if (agc_imu_stall_outcome(AGC_IMU_DEV_RADAR) != AGC_IMU_STALL_GOOD)
    {
        st         = AGC_STATUS_OK;
        td.vhftime = rz_marktime(); /* LIGHTON  */

        if (td.vhfrange != 0)
        {
            st = agc_svc_trfailon(); /* refused for bad data: lamp on */
        }

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        return (td_wait_for_mark());
    }

    st = agc_svc_trfailof();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (td.rm == 0U)
    {
        td.vhftime = rz_marktime(); /* LIGHTON +4 */

        return (td_wait_for_mark());
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_dp_t r = agc_dp_mul_trunc(agc_dp_from_sp((agc_sp_t) (td.rm & TD_POSMAX)), TD_CONVRNGE); /* m 2^27 */

    if ((td.rm & 040000u) != 0U)
    {
        r = agc_dp_add(&ip, r, TD_RANGEB14); /* RANGERD3 */
    }

    td.vhfrange = r;
    rz_flag_put(AGC_FLAG_SOURCFLG, true);
    td.vhftime  = rz_marktime();

    return (agc_job_rendezvous_tracking_data_csm_incorporate()); /* REND4 */
} /* agc_job_rendezvous_tracking_data_csm_range */

/**********************************************************************
 * VHFREAD (p.587): the radar RUPT of the CSM. Nothing when SKIPVHF is set
 * (VHF restart); with updating off or priority displays on the reading is
 * refused (VHFRANGE = 0, bad end); RM = RNRAD; data good (chan 33 bit 2
 * zero): good end, else VHFRANGE nonzero and bad end.
 * AGC: VHFREAD, VHFGOOD, BYPASS (P20-P25.agc p.587)
 *
 * FUNCTION NAME: agc_rendz_vhf_read
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the GOODEND / BADEND status, AGC_STATUS_OK when
 *          skipped
 **********************************************************************/
agc_status_t agc_rendz_vhf_read(void)
{
    if (agc_flag_test(AGC_FLAG_SKIPVHF))
    {
        return (AGC_STATUS_OK); /* TASKOVER */
    }

    agc_imu_device_t dev = AGC_IMU_DEV_RADAR; /* RADSTALL device */

    if (!agc_flag_test(AGC_FLAG_UPDATFLG) || agc_flag_test(AGC_FLAG_PDSPFLAG))
    {
        td.vhfrange = 0; /* BYPASS1  */

        return (agc_imu_badend(dev));
    }

    td.rm = (uint16_t) (agc_erase_rnrad() & 077777u);
    uint16_t dg; /* data good */

    dg = (uint16_t) (agc_chan_read(AGC_CHAN_CHAN33) & TD_CH33_VHF_GOOD);

    if (dg == 0U)
    {
        return (agc_imu_goodend(dev)); /* VHFGOOD  */
    }

    td.vhfrange = (agc_dp_t) dg; /* BYPASS   */

    return (agc_imu_badend(dev));
}

/**********************************************************************
 * REND4 / CSMUPP / REND5: both vehicles to MARKTIME, the vehicle being
 * corrected last with its W matrix (when valid); then SHIFTNDX, CSMPOS,
 * LEMPOS, first pass, W initialized (and the mark counts zero) if it was
 * invalid, RCLP.
 * AGC: REND4, CSMUPP, REND5 (P20-P25.agc pp.578-579)
 *
 * FUNCTION NAME: td_integrate
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the integration status
 *          (INTSTALL sleep)
 **********************************************************************/
static agc_status_t td_integrate(agc_interp_t *ip)
{
    agc_job_id_t resume; /* INTSTALL */

    resume           = AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_INCORPORATE;
    bool         csm = agc_flag_test(AGC_FLAG_VEHUPFLG); /* CSM corrected */
    agc_status_t st  = rz_setintg(resume);               /* SETINTG */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_flag_put(AGC_FLAG_VINTFLAG, !csm); /* the other vehicle   */
    st = agc_integ_integrv(ip, agc_integ_shared_ctx(), rz_marktime());
    agc_phase_change(04022);

    if (st == AGC_STATUS_OK)
    {
        st = rz_setintg(resume);
    }

    if (st == AGC_STATUS_OK)
    {
        rz_flag_put(AGC_FLAG_VINTFLAG, csm);
        rz_flag_put(AGC_FLAG_DIM0FLAG, agc_flag_test(AGC_FLAG_RENDWFLG)); /* REND5 */
        st    = agc_integ_integrv(ip, agc_integ_shared_ctx(), rz_marktime());
        td.x2 = td_shiftndx();

        if (st == AGC_STATUS_OK)
        {
            st = td_pos(ip, AGC_VEHICLE_CSM, &td.csmpos); /* CMPOS    */
        }

        rz_flag_put(AGC_FLAG_INCORFLG, true);

        if (st == AGC_STATUS_OK)
        {
            st = td_pos(ip, AGC_VEHICLE_LM, &td.lempos); /* LMPOS    */
        }
    }

    return (st);
} /* td_integrate */

/**********************************************************************
 * The incorporation passes from REND7 (or, after a noun 49 PROCEED, from
 * REND12): optics marks twice, VHF once. Ends in REND1 / REND3 (the next
 * look), or in RENDISP.
 * AGC: REND7 .. REND18 (P20-P25.agc pp.580-584)
 *
 * FUNCTION NAME: td_passes
 *
 * ARGUMENTS:
 *   Argument    Type             I/O  Description (scale)
 *   ----------  ---------------  ---  -----------------------------
 *   ip          agc_interp_t *   I/O  interpreter state (OVFIND)
 *   at_incorp2  bool             I    true: start at REND12 (after a noun
 *                                     49 PROCEED)
 *
 * RETURNS: agc_status_t -- the status of the next look (WAITONE), the
 *          display, or the step that failed
 **********************************************************************/
static agc_status_t td_passes(agc_interp_t *ip, bool at_incorp2)
{
    for (int i = 0; i < TD_PASSES; i++)
    {
        agc_status_t st = AGC_STATUS_OK; /* status of the last step */

        if (!at_incorp2)
        {
            if (agc_flag_test(AGC_FLAG_SOURCFLG))
            {
                /* REND14   */
                /**********************************************************************
                 * REND14: the VHF range: B0 = UNIT(RCLP) (complemented if the CSM is
                 * corrected), DELTAQ = VHFRANGE (SR2 in earth orbit) - |RCLP|.
                 * AGC: REND14, REND14A (P20-P25.agc p.583)
                 *
                 * FUNCTION NAME: td_vhf_mark
                 *
                 * ARGUMENTS:
                 *   Argument  Type            I/O  Description (scale)
                 *   --------  --------------  ---  --------------------------------
                 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
                 **********************************************************************/
                agc_dp_t  absr = 0;                                      /* |RCLP|, body scale */
                agc_vec_t u    = agc_vec_unit(ip, td.rclp, &absr, NULL); /* 2^1 */

                td.inc.bvector[0] = u;
                td.inc.bvector[1] = agc_vec_zero();
                td.inc.bvector[2] = agc_vec_zero();

                if (agc_flag_test(AGC_FLAG_VEHUPFLG))
                {
                    td.inc.bvector[0] = agc_vec_neg(u); /* the CSM is corrected */
                }

                agc_dp_t r = td.vhfrange; /* moon: m 2^27 */

                if (!agc_flag_test(AGC_FLAG_CMOONFLG))
                {
                    r = agc_dp_shr(td.vhfrange, 2); /* earth: SR2, m 2^29 */
                }

                td.inc.deltaq = agc_dp_sub(ip, r, absr);
                rz_flag_put(AGC_FLAG_INCORFLG, true);
                agc_phase_change(04022); /* REND14A  */
                td_variance(ip, agc_rendz_nav_pad()->rvar);
            }
            else if (!td_optics_mark(ip, &st))
            {
                if (st != AGC_STATUS_OK)
                {
                    return (st);
                }

                return (td_wait_for_vhf()); /* REND3: ignore the mark */
            }

            bool big = false; /* noun 49 wanted */

            if (st == AGC_STATUS_OK)
            {
                big = td_incorp1(ip, &st);
            }

            if (st != AGC_STATUS_OK)
            {
                return (st);
            }

            if (big)
            {
                return (agc_job_rendezvous_tracking_data_csm_display());
            }
        }

        at_incorp2 = false;
        st         = rz_incorp2(ip, &td.inc, agc_flag_test(AGC_FLAG_CMOONFLG)); /* REND12 */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        if (agc_flag_test(AGC_FLAG_SOURCFLG))
        { /* REND16   */
            agc_erase_set_vhfcnt((agc_sp_t) (agc_erase_vhfcnt() + 1));

            return (td_wait_for_mark());
        }

        if (!agc_flag_test(AGC_FLAG_INCORFLG))
        { /* REND17   */
            agc_erase_set_trkmkcnt((agc_sp_t) (agc_erase_trkmkcnt() + 1));

            return (td_wait_for_vhf());
        }

        td.x2 = td_shiftndx();

        if (agc_flag_test(AGC_FLAG_VEHUPFLG))
        {
            st = td_pos(ip, AGC_VEHICLE_CSM, &td.csmpos); /* REND18   */
        }
        else
        {
            st = td_pos(ip, AGC_VEHICLE_LM, &td.lempos);
        }

        agc_phase_change(04022); /* REND13   */
        td.rclp = agc_vec_sub(ip, td.lempos, td.csmpos);
        rz_flag_put(AGC_FLAG_INCORFLG, false);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    } /* for (int i = 0; i < TD_PASSES; i++) */

    return (AGC_STATUS_NO_CONVERGE); /* step bound exhausted */
} /* td_passes */

/**********************************************************************
 * REND4 .. REND6, then the passes.
 * AGC: REND4 .. REND6 (P20-P25.agc pp.578-580)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_csm_incorporate
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_csm_incorporate(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st = td_integrate(&ip); /* REND4 */

    if (st != AGC_STATUS_OK)
    {
        return (st); /* BUSY: asleep in INTSTALL  */
    }

    rz_flag_put(AGC_FLAG_ORBWFLAG, false);

    if (!agc_flag_test(AGC_FLAG_RENDWFLG))
    {
        const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad(); /* pad loads */

        td_initialw(p->wrendpos, p->wrendvel); /* INITIALW */
        agc_erase_set_vhfcnt(0);
        agc_erase_set_trkmkcnt(0);
    }

    rz_flag_put(AGC_FLAG_RENDWFLG, true); /* REND6    */
    td.rclp = agc_vec_sub(&ip, td.lempos, td.csmpos);

    return (td_passes(&ip, false));
}

/**********************************************************************
 * RENDISP (pp.584-585): noun 49 (|dR|, |dV|, source 1 optics / 2
 * VHF) in a PRIO27 job; this job polls TEMPOR1.
 * AGC: RENDISP (P20-P25.agc p.584)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_csm_display
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_csm_display(void)
{
    td.n49disp[2] = 1; /* optics */

    if (agc_flag_test(AGC_FLAG_SOURCFLG))
    {
        td.n49disp[2] = 2; /* VHF */
    }

    td.tempor1      = 0;
    agc_phase_change(04022);
    agc_status_t st = agc_exec_novac(TD_PRIO27, AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_RESIDUALS); /* RENDISP2 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (rz_continue(AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_RESIDUALS_WAIT));
}

/**********************************************************************
 * RENDISP2: V06N49 (PRIODSP): TERMINATE exits P20 (V56), PROCEED
 * TEMPOR1 = -1, RECYCLE TEMPOR1 positive.
 * AGC: RENDISP2 (P20-P25.agc p.585)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_csm_residuals
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_csm_residuals(agc_job_ctx_t *ctx)
{
    return (rz_flash(ctx, AGC_DSKY_PRIODSP, 6U, 49U, AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE, AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_RESIDUALS_PROCEED, AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_RESIDUALS_RECYCLE));
}

/**********************************************************************
 * RENDISP2 PROCEED: TEMPOR1 = -1 (incorporate).
 * AGC: RENDISP2 (P20-P25.agc p.585)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_csm_residuals_proceed
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_csm_residuals_proceed(void)
{
    td.tempor1 = -1;

    return (AGC_STATUS_OK); /* ENDOFJOB */
}

/**********************************************************************
 * RENDISP2 RECYCLE: TEMPOR1 positive (drop the mark).
 * AGC: RENDISP2 (P20-P25.agc p.585)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_csm_residuals_recycle
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_csm_residuals_recycle(void)
{
    td.tempor1 = 1;

    return (AGC_STATUS_OK); /* ENDOFJOB */
}

/**********************************************************************
 * RENDISP7: wait for the answer; PROCEED: REND12; RECYCLE (RENDISP3): a
 * VHF reading is dropped (REND1); an optics mark empties both buffers
 * (REND3).
 * AGC: RENDISP7, RENDISP3, RENDISP4 (P20-P25.agc p.585)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_csm_residuals_wait
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_csm_residuals_wait(void)
{
    if (td.tempor1 == 0)
    {
        agc_job_id_t again; /* look again */

        again = AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_RESIDUALS_WAIT;

        return (rz_continue(again));
    }

    if (td.tempor1 < 0)
    {
        agc_interp_t ip; /* interpreter state */

        agc_interp_init(&ip);

        return (td_passes(&ip, true));
    }

    if (agc_flag_test(AGC_FLAG_SOURCFLG))
    {
        return (td_wait_for_mark());
    }

    agc_inhint();
    agc_erase_set_mrkbuf(1, 0, TD_EMPTY);
    agc_erase_set_mrkbuf(2, 0, TD_EMPTY);
    agc_relint();

    return (td_wait_for_vhf()); /* RENDISP4 */
}

/**********************************************************************
 * N49DISP +index (index 0 |dR|, 1 |dV|, 2 source code) for noun 49.
 * AGC: N49DISP (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: rz_csm_residuals
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   index     int   I    0 |dR|, 1 |dV|, 2 source code
 *
 * RETURNS: agc_dp_t -- the value (0 for a bad index)
 **********************************************************************/
agc_dp_t rz_csm_residuals(int index)
{
    return (td.n49disp[index]);
}

/**********************************************************************
 * Store N49DISP +index (P23 uses it too).
 * AGC: N49DISP (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: rz_set_csm_residuals
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   index     int       I    0 |dR|, 1 |dV|, 2 source code
 *   v         agc_dp_t  I    the value (m 2^29, m/cs 2^7 or a code)
 **********************************************************************/
void rz_set_csm_residuals(int index, agc_dp_t v)
{
    if (AGC_ASSERT(index >= 0 && index < 3))
    {
        td.n49disp[index] = v;
    }
}

/**********************************************************************
 * Noun data of the CSM rendezvous navigation: the N49 mark residuals
 * (R22 / P23). NOT_FOUND: not one of them.
 * AGC: none (noun table access; noun 49)
 *
 * FUNCTION NAME: agc_csm_rendz_noun_read
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   data_id   uint16_t    I    noun data id
 *   v         agc_dp_t *  O    the value at its noun scale
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_NOT_FOUND when not a
 *          noun 49 item, AGC_STATUS_BAD_ARG on a bad argument
 **********************************************************************/
agc_status_t agc_csm_rendz_noun_read(uint16_t data_id, agc_dp_t *v)
{
    switch (data_id)
    {
        case AGC_DATA_MARK_DELTA_R:
            *v = rz_csm_residuals(0);

            return (AGC_STATUS_OK);

        case AGC_DATA_MARK_DELTA_V:
            *v = rz_csm_residuals(1);

            return (AGC_STATUS_OK);

        case AGC_DATA_MARK_SOURCE_CODE:
            *v = agc_dp_from_sp((agc_sp_t) rz_csm_residuals(2));

            return (AGC_STATUS_OK);

        default:
            break;
    }

    return (AGC_STATUS_NOT_FOUND);
}
