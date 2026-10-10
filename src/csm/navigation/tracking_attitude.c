/*
 * tracking_attitude_csm.c -- R61CSM PREFERRED TRACKING ATTITUDE ROUTINE
 * and CRS61.1 / CRS61.2, CSM only (P20-P25.agc, Comanche055 pp.574-575,
 * 590-597).
 *
 * "To compute the preferred tracking attitude of the CSM to enable optics
 * tracking of the LM and to perform the maneuver to the preferred or
 * X-axis tracking attitude." CRS61.1 points the track axis (55 degrees
 * from +Z toward +X: preferred; or +X for COAS tracking) at the LM with
 * VECPOINT; more than 10 degrees from the present CDUs it asks R60 for a
 * maneuver, otherwise (in auto, nobody on the stick) it feeds the DAP
 * directly: desired CDUs, their 0.1 s increments and the LOS rate in
 * control axes (CRS61.2).
 *
 * Scales: LOS unit half unit, |LOS| m 2^29, |DV| m/cs 2^7, CDU words
 * two's complement half revolutions, DELCDU 180 deg, WBODY 450 deg/s.
 */
#include <stddef.h>
#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/imu/imu.h"
#include "agc/guidance/guidance.h"
#include "agc/csm/dap.h"
#include "agc/programs/alignment_pending.h"
#include "../../programs/navigation/internal.h"

#define TA_DEGREE10   910                     /* DEGREE10 DEC .05556       */
#define TA_RVCS_RDS   ((agc_dp_t)267017681)   /* RVCS/RDS 2DEC 15.915494 B-4 */
#define TA_TENTH      ((agc_dp_t)214748365)   /* TENTH 2DEC .1 B+3         */
#define TA_POINT8     ((agc_dp_t)214748365)   /* POINT8 2DEC .8            */
#define TA_PRF_X      ((agc_dp_t)109944726)   /* PRFUNIT .40957602 (sin 55)/2 */
#define TA_PRF_Z      ((agc_dp_t)76984127)    /* PRFUNIT .28678822 (cos 55)/2 */
#define TA_COS725     ((agc_dp_t)133144651)   /* .99200495 B-1             */
#define TA_SIN725     ((agc_dp_t)16938139)    /* .12619897 B-1             */
#define TA_CH31_AUTO  030000u                 /* PRIO30: bits 15-13 = 011  */
#define TA_FURST3     070000u                 /* FURST3 = 13,14,15         */
#define TA_UPACTY     000004u                 /* DSALMOUT BIT3: UPLINK ACTY */

/* CRS61.1 erasables (EBANK7): SAVEPOS, SAVEVEL, SAVEDCDU, P21TIME (here
 * |LOS|), CPHIX (N96), PRAXIS (N95), R61CNTR, GENRET.                  */
typedef struct ta_csm {
    agc_vec_t savepos;       /* SAVEPOS: CSM position, then unit LOS 2^1 */
    agc_vec_t savevel;       /* SAVEVEL: CSM velocity, m/cs 2^7          */
    agc_vec_t vlem;          /* DCDU (temp): LM velocity, m/cs 2^7       */
    agc_dp_t  abslos;        /* P21TIME (temp): |LOS|, m 2^29            */
    uint16_t  savedcdu[3];   /* SAVEDCDU: desired CDUs (THETAD)          */
    uint16_t  cphix[3];      /* CPHIX: X-axis tracking attitude (N96)    */
    uint16_t  praxis[3];     /* PRAXIS: preferred tracking attitude (N95) */
    agc_sp_t  r61cntr;       /* R61CNTR: + passes to skip, - KALCMANU    */
    agc_job_id_t genret;     /* GENRET                                   */
} ta_csm_t;

static ta_csm_t ta;

typedef enum ta_result {
    TA_FED_DAP = 0,     /* MPAC = 0: DAP fed, or not in auto, or stick   */
    TA_MANEUVER = 1     /* MPAC = 1: CPHI.. set for KALCMANU             */
} ta_result_t;

agc_sp_t agc_erase_r61cntr(void)
{
    return ta.r61cntr;
}

void agc_erase_set_r61cntr(agc_sp_t v)
{
    (void)AGC_ASSERT(v >= -1 && v <= 3);
    ta.r61cntr = v;
}

/* R63 (the CRS61.1 subroutine): CSM and LM to TIME (conic); LOS unit in
 * SAVEPOS, its length, POINTVSM (SM); VECPOINT for +X (CPHIX) and for the
 * 55 degree axis (PRAXIS); THETAD = PRAXIS or (PRFTRKAT off) CPHIX.    */
static agc_status_t ta_desired(agc_interp_t *ip, agc_dp_t t, agc_job_id_t resume)
{
    agc_integ_out_t o;
    agc_status_t st = rz_integ(ip, RZ_CSMCONIC, t, resume, &o);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    ta.savepos = o.ratt;                                      /* HOLDATT  */
    ta.savevel = o.vatt;
    st = rz_integ(ip, RZ_LEMCONIC, t, resume, &o);            /* CALCLEM  */
    if (st != AGC_STATUS_OK) {
        return st;
    }
    ta.vlem = o.vatt;
    ta.savepos = agc_vec_unit(ip, agc_vec_sub(ip, o.ratt, ta.savepos), &ta.abslos, NULL);
    agc_mat_t refsmmat = agc_erase_refsmmat();
    agc_vec_t pointvsm = agc_vec_shl(ip, agc_mat_vec(ip, &refsmmat, ta.savepos), 1);
    agc_vec_t unitx = agc_vec_make(AGC_DP_HALF, 0, 0);
    agc_vec_t prf = agc_vec_make(TA_PRF_X, 0, TA_PRF_Z);
    st = agc_imu_vecpoint(ip, pointvsm, unitx, ta.cphix);
    if (st == AGC_STATUS_OK) {
        st = agc_imu_vecpoint(ip, pointvsm, prf, ta.praxis);
    }
    bool prftrk = agc_flag_test(AGC_FLAG_PRFTRKAT);
    if (st == AGC_STATUS_OK) {
        st = agc_imu_set_scaxis(prftrk ? prf : unitx);        /* CRSTOR1  */
    }
    for (int i = 0; i < 3 && st == AGC_STATUS_OK; i++) {
        ta.savedcdu[i] = prftrk ? ta.praxis[i] : ta.cphix[i];
        st = agc_erase_set_thetad(i, ta.savedcdu[i]);         /* CRSTOR   */
    }
    return st;
}

/* STEP2CK: any CDU more than 10 degrees from its desired value. (The AGC
 * forms ACDU - DCDU with MSU and checks the SP part as a DP.)        */
static bool ta_gross(void)
{
    uint16_t cdu[3] = { agc_erase_cdux(), agc_erase_cduy(), agc_erase_cduz() };
    for (int i = 2; i >= 0; i--) {                            /* CDULOOP  */
        int d = ((int)cdu[i] - (int)ta.savedcdu[i]) & 077777;
        if ((d & 040000) != 0) {
            d -= 0100000;
        }
        int m = (d < 0) ? -d : d;
        if (m - TA_DEGREE10 >= 0) {
            return true;
        }
    }
    (void)AGC_ASSERT(TA_DEGREE10 > 0);
    return false;
}

/* CRS61.2: LOS rate OMEGA = ((-UNIT DV) x UNIT LOS) |DV| / |LOS|
 * (100/2PI), normalized (X1); in SM axes, 0.1 s of it to CDU increments
 * (SMCDURES); in NB then control axes (MBDYTCTL) x .8 for WBODY.       */
static agc_status_t ta_rates(agc_interp_t *ip, agc_vec_t *delcdu, agc_vec_t *wbody)
{
    agc_dp_t absdv = 0;
    agc_vec_t u = agc_vec_neg(agc_vec_unit(ip, agc_vec_sub(ip, ta.vlem, ta.savevel),
                                           &absdv, NULL));
    agc_vec_t cross = agc_vec_scale(ip, agc_vec_cross(ip, u, ta.savepos), TA_RVCS_RDS);
    int x1 = 0;
    int s1 = 0;
    agc_dp_t den = rz_norm(ta.abslos, &x1);
    agc_dp_t num = rz_norm(absdv, &s1);
    x1 -= s1;                                                 /* XSU,1 S1 */
    agc_dp_t quot = agc_dp_div(ip, agc_dp_shr(num, 1), den);
    agc_mat_t refsmmat = agc_erase_refsmmat();
    agc_vec_t om = agc_vec_shl(ip, agc_mat_vec(ip, &refsmmat,
                                               agc_vec_scale(ip, cross, quot)), 1);
    agc_cdutrig_t trig;
    agc_status_t st = agc_guid_cdutrig(&trig);
    agc_vec_t dcdu = agc_vec_zero();
    if (st == AGC_STATUS_OK) {
        st = agc_guid_smcdures(ip, &trig, agc_vec_scale(ip, om, TA_TENTH), &dcdu);
    }
    *delcdu = rz_vshift(ip, dcdu, -17 - x1);                  /* VSL* 0-17D,1 */
    if (st == AGC_STATUS_OK) {
        st = agc_guid_smnb(&trig, &om);                       /* *SMNB*   */
    }
    agc_mat_t m;
    m.row[0] = agc_vec_make(AGC_DP_HALF, 0, 0);               /* MBDYTCTL */
    m.row[1] = agc_vec_make(0, TA_COS725, agc_dp_neg(TA_SIN725));
    m.row[2] = agc_vec_make(0, TA_SIN725, TA_COS725);
    agc_vec_t w = agc_vec_scale(ip, agc_mat_vec(ip, &m, om), TA_POINT8);
    *wbody = rz_vshift(ip, w, -14 - x1);                      /* VSL* 0-14D,1 */
    return st;
}

/* CRS61.2A: the DAP inputs under INHINT; HOLDFLAG = -1.                 */
static void ta_feed_dap(agc_vec_t delcdu, agc_vec_t wbody)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    agc_inhint();
    for (int i = 0; i < 3; i++) {
        r->cdud[i] = agc_dp_from_sp((agc_sp_t)rz_word_value(ta.savedcdu[i]));
        r->wbody[i] = wbody.c[i];
        r->delcdu[i] = delcdu.c[i];
    }
    r->holdflag = -1;
    agc_relint();
    (void)AGC_ASSERT(r->holdflag < 0);
}

/* CRS61.1 (pp.592-596): desired CDUs now; a gross maneuver (MANUEXIT,
 * CPHI.. = SAVEDCDU) unless someone is on the stick (then the UPLINK
 * ACTY lamp); in auto with the stick free, CRS61.2 and the DAP inputs. */
static agc_status_t ta_crs61(agc_interp_t *ip, agc_job_id_t resume, ta_result_t *res)
{
    *res = TA_FED_DAP;
    agc_status_t st = ta_desired(ip, rz_loadtime(), resume); /* STORT    */
    if (st != AGC_STATUS_OK) {
        return st;
    }
    st = agc_chan_and(AGC_CHAN_DSALMOUT, (uint16_t)(077777u & ~TA_UPACTY)); /* UPACTOFF */
    if (st != AGC_STATUS_OK) {
        return st;
    }
    bool stick = agc_flag_test(AGC_FLAG_STIKFLAG);
    if (ta_gross()) {                                         /* STKTEST  */
        if (!stick) {
            *res = TA_MANEUVER;                               /* MANUEXIT */
            return AGC_STATUS_OK;
        }
        return agc_chan_or(AGC_CHAN_DSALMOUT, TA_UPACTY);     /* ASET     */
    }
    uint16_t c31 = agc_chan_read(AGC_CHAN_CHAN31);
    if (((c31 ^ TA_CH31_AUTO) & TA_FURST3) != 0U || stick) {  /* AUTOCK   */
        return AGC_STATUS_OK;                                 /* ASET     */
    }
    agc_vec_t delcdu;
    agc_vec_t wbody;
    st = ta_rates(ip, &delcdu, &wbody);                       /* CRS61.2  */
    if (st == AGC_STATUS_OK) {
        ta_feed_dap(delcdu, wbody);                           /* CRS61.2A */
    }
    return st;
}

/* R61CSM (pp.574-575): R61CNTR > 0: skip this pass (count down); < 0:
 * KALCMANU running; 0: CRS61.1, and on a gross error the R60 maneuver
 * (3-axis flag off, priority displays) with R61CNTR = -1. The caller
 * continues as ret_job.                                                 */
agc_status_t agc_rendz_tracking_attitude_csm(const agc_job_ctx_t *ctx,
                                             agc_job_id_t ret_job)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(ret_job != AGC_JOB_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    ta.genret = ret_job;
    if (ta.r61cntr > 0) {
        ta.r61cntr--;                                         /* DECRM61  */
        return rz_continue(ret_job);
    }
    if (ta.r61cntr < 0) {
        return rz_continue(ret_job);                          /* DECRM61+1 */
    }
    return rz_continue(AGC_JOB_TRACKING_ATTITUDE_CSM);
}

/* R61CSM after the counter test (also the INTSTALL resume point).       */
agc_status_t agc_job_tracking_attitude_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    ta_result_t res = TA_FED_DAP;
    agc_status_t st = ta_crs61(&ip, AGC_JOB_TRACKING_ATTITUDE_CSM, &res);
    if (st != AGC_STATUS_OK) {
        return st;                        /* BUSY: asleep in INTSTALL  */
    }
    if (res == TA_FED_DAP) {
        ta.r61cntr = 3;                                       /* R61END   */
        return rz_continue(ta.genret);
    }
    rz_flag_put(AGC_FLAG_3AXISFLG, false);                    /* R61C1    */
    ta.r61cntr = -1;
    rz_flag_put(AGC_FLAG_PDSPFLAG, true);
    return agc_imu_attitude_maneuver(ctx, AGC_JOB_TRACKING_ATTITUDE_CSM_MANEUVERED);
}

/* After R60CSM: priority displays off, PHASCHNG 00111, R61CNTR = 0.     */
agc_status_t agc_job_tracking_attitude_csm_maneuvered(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rz_flag_put(AGC_FLAG_PDSPFLAG, false);
    rz_phase(1, 00111);
    ta.r61cntr = 0;
    return rz_continue(ta.genret);
}

/* CPHIX (noun 96) and PRAXIS (noun 95) for the displays.                */
uint16_t agc_rendz_tracking_angles(bool preferred, int axis)
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3)) {
        return 0U;
    }
    return preferred ? ta.praxis[axis] : ta.cphix[axis];
}

/* ---- V89CALL (R63, CSM, pp.616-618) -------------------------------------
 * "Calculates and displays final gimbal angles to point CSM +X axis or
 * preferred axis (UNIT(Z)COS55 DEG + UNIT(X)SIN55 DEG) at LM." V04N06
 * chooses the axis (R2 = 1 preferred, 2 +X); V06N18 shows the angles
 * one minute ahead; PROCEED maneuvers (R60CSM). DP1MIN 2DEC 6000.     */
#define TA_DP1MIN ((agc_dp_t)6000)

agc_status_t agc_job_rendezvous_final_attitude_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_imu_status_check();                 /* R02BOTH  */
    if (st == AGC_STATUS_OK) {
        st = rz_set_option(1, 3U);
    }
    if (st == AGC_STATUS_OK) {
        st = rz_set_option(2, 1U);
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_job_rendezvous_final_attitude_csm_option(ctx);
}

/* V04N06 flashing: TERMINATE ends the verb, PROCEED takes the axis.     */
agc_status_t agc_job_rendezvous_final_attitude_csm_option(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_flash(ctx, AGC_DSKY_GOFLASH, 4U, 6U, AGC_JOB_RENDZ_ENDEXT,
                    AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_CSM_CHOSEN,
                    AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_CSM_OPTION);
}

/* SETPAF / RSTPAF: the preferred attitude flag from R2; V89RECL.        */
agc_status_t agc_job_rendezvous_final_attitude_csm_chosen(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rz_flag_put(AGC_FLAG_PRFTRKAT, rz_option(2) == 1U);
    return agc_job_rendezvous_final_attitude_csm_compute(ctx);
}

/* V89RECL: R63COMP one minute from now; V06N18 (desired gimbal angles):
 * PROCEED maneuvers, RECYCLE computes again.                            */
agc_status_t agc_job_rendezvous_final_attitude_csm_compute(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_status_t st = ta_desired(&ip, agc_dp_add(&ip, rz_loadtime(), TA_DP1MIN),
                                 AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_CSM_COMPUTE);
    if (st != AGC_STATUS_OK) {
        return st;                        /* BUSY: asleep in INTSTALL  */
    }
    return rz_flash(ctx, AGC_DSKY_GOFLASH, 6U, 18U, AGC_JOB_RENDZ_ENDEXT,
                    AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_CSM_MANEUVER,
                    AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_CSM_COMPUTE);
}

/* PROCEED on V06N18: 3-axis flag off, R60CSM, then ENDEXT.              */
agc_status_t agc_job_rendezvous_final_attitude_csm_maneuver(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rz_flag_put(AGC_FLAG_3AXISFLG, false);
    return agc_imu_attitude_maneuver(ctx, AGC_JOB_RENDZ_ENDEXT);
}
