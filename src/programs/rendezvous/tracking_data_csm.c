/*
 * tracking_data_csm.c -- R22 RENDEZVOUS TRACKING DATA PROCESSING ROUTINE,
 * CSM (P20-P25.agc, Comanche055 pp.576-589): R22 / REND1 .. REND18,
 * BVECTORS, GETUM, CMPOS / LMPOS, SHIFTNDX, INITIALW, RENDISP, RANGERD
 * and the VHF radar RUPT reader VHFREAD.
 *
 * "To process rendezvous sighting mark data to update the state vector of
 * either the CSM or LM as defined by the rendezvous navigation program
 * (P20)." R22 runs as a job started by P20: every 4 s it looks for an
 * optics mark in MRKBUF2 (left there by R21 / R23) and, once a minute
 * without one, reads the VHF ranging. A mark is incorporated twice (the
 * star direction of the optics LOS, first and second pass); the VHF range
 * once.
 *
 * Scales: positions body scale (earth 2^29, moon 2^27; X2 = 0 earth, -2
 * moon), unit vectors 2^1, DELTAQ as noted, VARIANCE TP B-40, VHFRANGE m
 * 2^27, times cs 2^28.
 */
#include <stddef.h>
#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/imu/imu.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"
#include "agc/programs/alignment_pending.h"
#include "agc/io/service_internal.h"
#include "internal.h"

#define TD_SXTVAR    ((agc_dp_t)703687)     /* SXTVAR 2DEC 0.04 E-6 B+16  */
#define TD_IMUVAR    ((agc_dp_t)703687)     /* IMUVAR 2DEC 0.04 E-6 B+16  */
#define TD_CONVRNGE  ((agc_dp_t)606863)     /* CONVRNGE 2DEC 18.52 B-13:
                                             * .01 NM to m 2^27           */
#define TD_RANGEB14  ((agc_dp_t)606863)     /* RANGEB14 2DEC 303431.7 B-27 */
#define TD_60SECDP   ((agc_dp_t)6000)       /* 60SECDP                    */
#define TD_PRIO26    ((agc_prio_t)026)
#define TD_PRIO27    ((agc_prio_t)027)
#define TD_NEG3      ((agc_sp_t)-3)         /* MRKBUF2 = -3: data used    */
#define TD_EMPTY     077776u                /* -1                         */
#define TD_CH13_READ 000011u                /* OCT11: VHF range read bits */
#define TD_CH33_VHF_GOOD 000002u            /* chan 33 bit 2 (0 = good)   */
#define TD_POSMAX    037777u
#define TD_PASSES    4                      /* REND7 passes in one job    */

/* R22 erasables (EBANK= MRKBUF1).                                       */
typedef struct td_csm {
    agc_dp_t  vhftime;       /* VHFTIME: last VHF reading, cs 2^28        */
    agc_dp_t  vhfrange;      /* VHFRANGE: m 2^27                          */
    uint16_t  rm;            /* RM: raw VHF range (.01 NM), 15 bits       */
    uint16_t  mark[7];       /* MARKTIME, VTEMP: the mark being used      */
    agc_vec_t csmpos;        /* CSMPOS, LEMPOS: body scale                */
    agc_vec_t lempos;
    agc_vec_t rclp;          /* RCLP: LM - CSM, body scale                */
    agc_vec_t ucl;           /* UCL: unit RCLP                            */
    agc_vec_t um;            /* UM: optics LOS, reference, half unit      */
    agc_vec_t ustar;         /* USTAR: fictitious star direction          */
    agc_dp_t  n49disp[3];    /* N49DISP: |dR| 2^29, |dV| 2^7, source 1/2 */
    agc_sp_t  tempor1;       /* TEMPOR1: 0 open, -1 proceed, + recycle    */
    int       x2;            /* SHIFTNDX: 0 earth, -2 moon                */
    agc_incorp_t inc;        /* BVECTOR DELTAQ VARIANCE .. DELTAX         */
} td_csm_t;

static td_csm_t td = { 0, 0, 0U, { 0U }, { { 0, 0, 0 } }, { { 0, 0, 0 } },
                       { { 0, 0, 0 } }, { { 0, 0, 0 } }, { { 0, 0, 0 } },
                       { { 0, 0, 0 } }, { 0, 0, 0 }, 0, 0, { 0 } };

/* ---- SHIFTNDX / CMPOS / LMPOS (pp.584, 588) ------------------------------ */
/* X2 = -2 when the vehicle being updated is in the moon's sphere.       */
static int td_shiftndx(void)
{
    bool moon = agc_flag_test(agc_flag_test(AGC_FLAG_VEHUPFLG)
                              ? AGC_FLAG_CMOONFLG : AGC_FLAG_LMOONFLG);
    return moon ? -2 : 0;
}

/* Position = conic + deviation (VSR* 7,2), body scale.                  */
static agc_status_t td_pos(agc_interp_t *ip, agc_vehicle_t v, agc_vec_t *pos)
{
    agc_integ_sv_t sv;
    agc_status_t st = agc_integ_perm_get(v, &sv);
    *pos = agc_vec_add(ip, rz_vshift(ip, sv.deltav, -(7 - td.x2)), sv.rcv);
    return st;
}

/* INITIALW (p.588): W = 0 in the 6x6 part, diagonal position and
 * velocity values.                                                       */
static void td_initialw(agc_sp_t pos, agc_sp_t vel)
{
    agc_incorp_w_t *w = &agc_integ_shared_ctx()->w;
    for (int b = 0; b < 2; b++) {
        for (int m = 0; m < 6; m++) {
            w->blk[b][m] = agc_vec_zero();
        }
    }
    for (int k = 0; k < 3; k++) {
        w->blk[0][k].c[k] = agc_dp_from_sp(pos);
        w->blk[1][3 + k].c[k] = agc_dp_from_sp(vel);
    }
    (void)AGC_ASSERT(w->blk[0][0].c[0] == agc_dp_from_sp(pos));
}

/* ---- GETUM (p.578): the optics LOS of the mark in reference axes (SXTNB,
 * NBSM with the mark's CDUs, x REFSMMAT, VSL1); MARKDOWN for downlink. */
static agc_status_t td_getum(agc_interp_t *ip, agc_vec_t *um)
{
    agc_align_sxt_mark_t m;
    m.t = agc_dp_pack((agc_sp_t)td.mark[0], (agc_sp_t)td.mark[1]);
    m.cduy = td.mark[2];
    m.cdus = td.mark[3];
    m.cduz = td.mark[4];
    m.cdut = td.mark[5];
    m.cdux = td.mark[6];
    for (int i = 0; i < 7; i++) {
        agc_erase_set_markdown(i, td.mark[i]);
    }
    agc_vec_t los = agc_vec_zero();
    agc_status_t st = agc_csm_sxtsm(ip, &m, &los);
    agc_mat_t refsmmat = agc_erase_refsmmat();
    *um = agc_vec_shl(ip, agc_vec_mat(ip, los, &refsmmat), 1);
    return st;
}

/* ---- BVECTORS (p.576): rz_bvectors on the R22 state.                  */
static void td_bvectors(agc_interp_t *ip)
{
    rz_bvectors(ip, td.rclp, td.um, &td.ustar, &td.inc);
}

/* ---- variances (REND9A .. REND10 pp.581-582) --------------------------- */
/* VARIANCE = base |RCLP|^2 (normalized, X1 doubled, plus 2 X2), SR*
 * 0-2,1 to B-40, + INTVAR (SR 25); VHF: at least -RVARMIN.             */
static void td_variance(agc_interp_t *ip, agc_dp_t base)
{
    const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad();
    int x1 = 0;
    agc_dp_t n = rz_norm(agc_vec_abval(ip, td.rclp), &x1);
    agc_tp_t v = agc_dp_mul_tp(agc_dp_sq(ip, n), base);
    x1 = 2 * x1 + 2 * td_shiftndx();                          /* XAD,1    */
    v = agc_tp_shift(ip, v, -(-2 - x1));                      /* SR* 0-2,1 */
    agc_tp_t intvar = (agc_tp_t)p->intvar * 8;                /* SR 25    */
    v = agc_tp_add(ip, intvar, v);
    if (agc_flag_test(AGC_FLAG_SOURCFLG)) {
        agc_tp_t vmin = (agc_tp_t)p->rvarmin_cm[0] * 268435456
                      + (agc_tp_t)p->rvarmin_cm[1] * 16384 + p->rvarmin_cm[2];
        if (v + vmin < 0) {                                   /* BPL      */
            v = (vmin < 0) ? -vmin : vmin;                    /* ABS      */
        }
    }
    td.inc.variance = v;
}

/* REND7 .. REND9A for an optics mark: UCL; on the first pass UM and the
 * star direction USTAR = UNIT(UCL x UM) (overflow: ignore the mark);
 * BVECTORS, complemented if the LM is corrected. False: ignore.       */
static bool td_optics_mark(agc_interp_t *ip, agc_status_t *st)
{
    *st = AGC_STATUS_OK;
    td.ucl = agc_vec_unit(ip, td.rclp, NULL, NULL);
    if (agc_flag_test(AGC_FLAG_INCORFLG)) {
        *st = td_getum(ip, &td.um);
        rz_clear_ovf(ip);
        agc_vec_t c = agc_vec_cross(ip, td.ucl, td.um);       /* REND8    */
        td.ustar = agc_vec_unit(ip, c, NULL, NULL);
        if (agc_interp_test_ovf(ip)) {
            return false;                                     /* REND3    */
        }
    }
    td_bvectors(ip);                                          /* REND9    */
    if (!agc_flag_test(AGC_FLAG_VEHUPFLG)) {
        td.inc.bvector[0] = agc_vec_neg(td.inc.bvector[0]);
    }
    rz_phase(2, 04022);                                       /* REND9A   */
    const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad();
    agc_dp_t base = agc_flag_test(AGC_FLAG_BACKUP_MARKING)    /* REND15   */
                  ? agc_dp_add(ip, agc_dp_from_sp(p->altvar), TD_IMUVAR)
                  : agc_dp_add(ip, TD_SXTVAR, TD_IMUVAR);
    td_variance(ip, base);
    return true;
}

/* REND14: the VHF range: B0 = UNIT(RCLP) (complemented if the CSM is
 * corrected), DELTAQ = VHFRANGE (SR2 in earth orbit) - |RCLP|.         */
static void td_vhf_mark(agc_interp_t *ip)
{
    agc_dp_t absr = 0;
    agc_vec_t u = agc_vec_unit(ip, td.rclp, &absr, NULL);
    td.inc.bvector[0] = agc_flag_test(AGC_FLAG_VEHUPFLG) ? agc_vec_neg(u) : u;
    td.inc.bvector[1] = agc_vec_zero();
    td.inc.bvector[2] = agc_vec_zero();
    agc_dp_t r = agc_flag_test(AGC_FLAG_CMOONFLG) ? td.vhfrange
                                                  : agc_dp_shr(td.vhfrange, 2);
    td.inc.deltaq = agc_dp_sub(ip, r, absr);
    rz_flag_put(AGC_FLAG_INCORFLG, true);
    rz_phase(2, 04022);                                       /* REND14A  */
    td_variance(ip, agc_rendz_nav_pad()->rvar);
}

/* REND10A: INCORP1 (6x6); on the first pass the noun 49 values and the
 * RMAX / VMAX check. True: too large (RENDISP).                         */
static bool td_incorp1(agc_interp_t *ip, agc_status_t *st)
{
    rz_flag_put(AGC_FLAG_DMENFLG, false);
    td.inc.dim9 = false;
    *st = agc_incorp1(ip, &agc_integ_shared_ctx()->w, &td.inc);
    rz_phase(2, 04022);
    if (*st != AGC_STATUS_OK || !agc_flag_test(AGC_FLAG_INCORFLG)) {
        return false;
    }
    const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad();
    int k = 0 - td_shiftndx();                                /* SR* 0,2  */
    td.n49disp[1] = rz_shift(ip, agc_vec_abval(ip, td.inc.deltax[1]), -k);
    td.n49disp[0] = rz_shift(ip, agc_vec_abval(ip, td.inc.deltax[0]), -k);
    if (agc_dp_sub(ip, agc_dp_shr(agc_dp_from_sp(p->rmax), 10), td.n49disp[0]) < 0) {
        return true;
    }
    return agc_dp_sub(ip, agc_dp_from_sp(p->vmax), td.n49disp[1]) < 0;
}

/* ---- the R22 job --------------------------------------------------------- */
/* R22: priority 26; MRKBUF2 = -3 (used); VHFTIME = now; REND1.           */
agc_status_t agc_job_rendezvous_tracking_data_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_exec_prio_change(TD_PRIO26);
    if (st != AGC_STATUS_OK && st != AGC_STATUS_BUSY) {
        return st;
    }
    agc_erase_set_mrkbuf2(0, TD_NEG3);
    td.vhftime = rz_loadtime();
    rz_phase(2, 04022);                                       /* REND1    */
    return rz_waitone(AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_MARK);
}

/* REND1 / REND3 as a job: GRP2PC, WAITONE, then look for a mark (REND1)
 * or the VHF (REND3).                                                    */
static agc_status_t td_rend1(void)
{
    rz_phase(2, 04022);
    return rz_waitone(AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_MARK);
}

static agc_status_t td_rend3(void)
{
    rz_phase(2, 04022);
    return rz_waitone(AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_VHF);
}

/* REND1A / REND2: a mark in MRKBUF2 (first word not negative) is taken
 * (MARKTIME, VTEMP), MRKBUF2 = -3, optics source; else REND3.           */
agc_status_t agc_job_rendezvous_tracking_data_csm_mark(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_erase_mrkbuf2(0) < 0) {
        return td_rend3();                                    /* REND3A   */
    }
    agc_inhint();
    for (int i = 0; i < 7; i++) {                             /* GENTRAN  */
        td.mark[i] = agc_erase_mrkbuf(2, i);
    }
    agc_erase_set_mrkbuf2(0, TD_NEG3);
    agc_relint();
    rz_set_marktime(agc_dp_pack((agc_sp_t)td.mark[0], (agc_sp_t)td.mark[1]));
    rz_flag_put(AGC_FLAG_SOURCFLG, false);                    /* optics   */
    rz_phase(2, 04022);
    return agc_job_rendezvous_tracking_data_csm_incorporate(ctx);  /* REND4 */
}

/* REND3 after WAITONE: with VHF ranging on (VHFRFLAG) and a minute since
 * the last reading, RANGERD: start the read, MARKTIME = now, SKIPVHF
 * off, RADSTALL.                                                         */
agc_status_t agc_job_rendezvous_tracking_data_csm_vhf(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_dp_t d = agc_dp_sub(&ip, agc_dp_sub(&ip, rz_loadtime(), TD_60SECDP), td.vhftime);
    if (!agc_flag_test(AGC_FLAG_VHFRFLAG) || d < 0) {
        return agc_job_rendezvous_tracking_data_csm_mark(ctx);    /* REND1A */
    }
    agc_inhint();
    agc_status_t st = agc_chan_and(AGC_CHAN_CHAN13, (uint16_t)(077777u & ~017u));
    if (st == AGC_STATUS_OK) {
        st = agc_chan_or(AGC_CHAN_CHAN13, TD_CH13_READ);
    }
    agc_relint();
    rz_set_marktime(rz_loadtime());
    rz_flag_put(AGC_FLAG_SKIPVHF, false);
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING;
    if (st == AGC_STATUS_OK) {
        st = agc_imu_stall_dev(AGC_IMU_DEV_RADAR, AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_RANGE, &res);
    }
    if (st != AGC_STATUS_OK || res == AGC_IMU_STALL_SLEEPING) {
        return st;
    }
    return rz_continue(AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_RANGE);
}

/* RANGERD after RADSTALL: bad: LIGHTON (tracker lamp when the reading was
 * refused for bad data) and REND1; good: lamp off; RM (.01 NM, 15-bit
 * unsigned; +0 = no range) to VHFRANGE (m 2^27); VHF source; REND4.   */
agc_status_t agc_job_rendezvous_tracking_data_csm_range(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st;
    if (agc_imu_stall_outcome(AGC_IMU_DEV_RADAR) != AGC_IMU_STALL_GOOD) {
        st = (td.vhfrange != 0) ? agc_svc_trfailon() : AGC_STATUS_OK;
        td.vhftime = rz_marktime();                           /* LIGHTON  */
        return (st != AGC_STATUS_OK) ? st : td_rend1();
    }
    st = agc_svc_trfailof();
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if (td.rm == 0U) {
        td.vhftime = rz_marktime();                           /* LIGHTON +4 */
        return td_rend1();
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_dp_t r = agc_dp_mul_trunc(agc_dp_from_sp((agc_sp_t)(td.rm & TD_POSMAX)),
                                  TD_CONVRNGE);
    if ((td.rm & 040000u) != 0U) {
        r = agc_dp_add(&ip, r, TD_RANGEB14);                  /* RANGERD3 */
    }
    td.vhfrange = r;
    rz_flag_put(AGC_FLAG_SOURCFLG, true);
    td.vhftime = rz_marktime();
    return agc_job_rendezvous_tracking_data_csm_incorporate(ctx);  /* REND4 */
}

/* VHFREAD (p.587): the radar RUPT of the CSM. Nothing when SKIPVHF is set
 * (VHF restart); with updating off or priority displays on the reading is
 * refused (VHFRANGE = 0, bad end); RM = RNRAD; data good (chan 33 bit 2
 * zero): good end, else VHFRANGE nonzero and bad end.                  */
agc_status_t agc_rendz_vhf_read(void)
{
    if (agc_flag_test(AGC_FLAG_SKIPVHF)) {
        return AGC_STATUS_OK;                                 /* TASKOVER */
    }
    agc_imu_device_t dev = AGC_IMU_DEV_RADAR;
    if (!agc_flag_test(AGC_FLAG_UPDATFLG) || agc_flag_test(AGC_FLAG_PDSPFLAG)) {
        td.vhfrange = 0;                                      /* BYPASS1  */
        return agc_imu_badend(dev);
    }
    td.rm = (uint16_t)(agc_erase_rnrad() & 077777u);
    uint16_t dg = (uint16_t)(agc_chan_read(AGC_CHAN_CHAN33) & TD_CH33_VHF_GOOD);
    if (dg == 0U) {
        return agc_imu_goodend(dev);                          /* VHFGOOD  */
    }
    td.vhfrange = (agc_dp_t)dg;                               /* BYPASS   */
    return agc_imu_badend(dev);
}

/* REND4 / CSMUPP / REND5: both vehicles to MARKTIME, the vehicle being
 * corrected last with its W matrix (when valid); then SHIFTNDX, CSMPOS,
 * LEMPOS, first pass, W initialized (and the mark counts zero) if it was
 * invalid, RCLP.                                                        */
static agc_status_t td_integrate(agc_interp_t *ip)
{
    agc_job_id_t resume = AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_INCORPORATE;
    bool csm = agc_flag_test(AGC_FLAG_VEHUPFLG);
    agc_status_t st = rz_setintg(resume);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz_flag_put(AGC_FLAG_VINTFLAG, !csm);       /* the other vehicle   */
    st = agc_integ_integrv(ip, agc_integ_shared_ctx(), rz_marktime());
    rz_phase(2, 04022);
    if (st == AGC_STATUS_OK) {
        st = rz_setintg(resume);
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz_flag_put(AGC_FLAG_VINTFLAG, csm);
    rz_flag_put(AGC_FLAG_DIM0FLAG, agc_flag_test(AGC_FLAG_RENDWFLG));   /* REND5 */
    st = agc_integ_integrv(ip, agc_integ_shared_ctx(), rz_marktime());
    td.x2 = td_shiftndx();
    if (st == AGC_STATUS_OK) {
        st = td_pos(ip, AGC_VEHICLE_CSM, &td.csmpos);         /* CMPOS    */
    }
    rz_flag_put(AGC_FLAG_INCORFLG, true);
    if (st == AGC_STATUS_OK) {
        st = td_pos(ip, AGC_VEHICLE_LM, &td.lempos);          /* LMPOS    */
    }
    return st;
}

/* The incorporation passes from REND7 (or, after a noun 49 PROCEED, from
 * REND12): optics marks twice, VHF once. Ends in REND1 / REND3 (the next
 * look), or in RENDISP.                                                  */
static agc_status_t td_passes(agc_job_ctx_t *ctx, agc_interp_t *ip, bool at_incorp2)
{
    for (int i = 0; i < TD_PASSES; i++) {
        agc_status_t st = AGC_STATUS_OK;
        if (!at_incorp2) {
            if (agc_flag_test(AGC_FLAG_SOURCFLG)) {
                td_vhf_mark(ip);                              /* REND14   */
            } else if (!td_optics_mark(ip, &st)) {
                return (st != AGC_STATUS_OK) ? st : td_rend3();
            }
            bool big = (st == AGC_STATUS_OK) && td_incorp1(ip, &st);
            if (st != AGC_STATUS_OK) {
                return st;
            }
            if (big) {
                return agc_job_rendezvous_tracking_data_csm_display(ctx);
            }
        }
        at_incorp2 = false;
        st = rz_incorp2(ip, &td.inc, agc_flag_test(AGC_FLAG_CMOONFLG));  /* REND12 */
        if (st != AGC_STATUS_OK) {
            return st;
        }
        if (agc_flag_test(AGC_FLAG_SOURCFLG)) {               /* REND16   */
            agc_erase_set_vhfcnt((agc_sp_t)(agc_erase_vhfcnt() + 1));
            return td_rend1();
        }
        if (!agc_flag_test(AGC_FLAG_INCORFLG)) {              /* REND17   */
            agc_erase_set_trkmkcnt((agc_sp_t)(agc_erase_trkmkcnt() + 1));
            return td_rend3();
        }
        td.x2 = td_shiftndx();
        bool csm = agc_flag_test(AGC_FLAG_VEHUPFLG);
        st = td_pos(ip, csm ? AGC_VEHICLE_CSM : AGC_VEHICLE_LM,
                    csm ? &td.csmpos : &td.lempos);           /* REND18   */
        rz_phase(2, 04022);                                   /* REND13   */
        td.rclp = agc_vec_sub(ip, td.lempos, td.csmpos);
        rz_flag_put(AGC_FLAG_INCORFLG, false);
        if (st != AGC_STATUS_OK) {
            return st;
        }
    }
    (void)AGC_ASSERT(false);
    return AGC_STATUS_NO_CONVERGE;
}

/* REND4 .. REND6, then the passes.                                       */
agc_status_t agc_job_rendezvous_tracking_data_csm_incorporate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_status_t st = td_integrate(&ip);
    if (st != AGC_STATUS_OK) {
        return st;                        /* BUSY: asleep in INTSTALL  */
    }
    rz_flag_put(AGC_FLAG_ORBWFLAG, false);
    if (!agc_flag_test(AGC_FLAG_RENDWFLG)) {
        const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad();
        td_initialw(p->wrendpos, p->wrendvel);                /* INITIALW */
        agc_erase_set_vhfcnt(0);
        agc_erase_set_trkmkcnt(0);
    }
    rz_flag_put(AGC_FLAG_RENDWFLG, true);                     /* REND6    */
    td.rclp = agc_vec_sub(&ip, td.lempos, td.csmpos);
    return td_passes(ctx, &ip, false);
}

/* ---- RENDISP (pp.584-585): noun 49 (|dR|, |dV|, source 1 optics / 2
 * VHF) in a PRIO27 job; this job polls TEMPOR1.                         */
agc_status_t agc_job_rendezvous_tracking_data_csm_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    td.n49disp[2] = agc_flag_test(AGC_FLAG_SOURCFLG) ? 2 : 1;
    td.tempor1 = 0;
    rz_phase(2, 04022);
    agc_status_t st = agc_exec_novac(TD_PRIO27, AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_N49);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return rz_continue(AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_N49_WAIT);
}

/* RENDISP2: V06N49 (PRIODSP): TERMINATE exits P20 (V56), PROCEED
 * TEMPOR1 = -1, RECYCLE TEMPOR1 positive.                               */
agc_status_t agc_job_rendezvous_tracking_data_csm_n49(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_flash(ctx, AGC_DSKY_PRIODSP, 6U, 49U,
                    AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE,
                    AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_N49_PROCEED,
                    AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_N49_RECYCLE);
}

agc_status_t agc_job_rendezvous_tracking_data_csm_n49_proceed(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    td.tempor1 = -1;
    return AGC_STATUS_OK;                                     /* ENDOFJOB */
}

agc_status_t agc_job_rendezvous_tracking_data_csm_n49_recycle(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    td.tempor1 = 1;
    return AGC_STATUS_OK;                                     /* ENDOFJOB */
}

/* RENDISP7: wait for the answer; PROCEED: REND12; RECYCLE (RENDISP3): a
 * VHF reading is dropped (REND1); an optics mark empties both buffers
 * (REND3).                                                               */
agc_status_t agc_job_rendezvous_tracking_data_csm_n49_wait(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (td.tempor1 == 0) {
        return rz_continue(AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM_N49_WAIT);
    }
    if (td.tempor1 < 0) {
        agc_interp_t ip;
        agc_interp_init(&ip);
        return td_passes(ctx, &ip, true);
    }
    if (agc_flag_test(AGC_FLAG_SOURCFLG)) {
        return td_rend1();
    }
    agc_inhint();
    agc_erase_set_mrkbuf(1, 0, TD_EMPTY);
    agc_erase_set_mrkbuf(2, 0, TD_EMPTY);
    agc_relint();
    return td_rend3();                                        /* RENDISP4 */
}

/* N49DISP +index (index 0 |dR|, 1 |dV|, 2 source code) for noun 49.     */
agc_dp_t rz_csm_n49disp(int index)
{
    if (!AGC_ASSERT(index >= 0 && index < 3)) {
        return 0;
    }
    return td.n49disp[index];
}
