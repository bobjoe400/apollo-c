/*
 * auto_optics_positioning.c -- R52 automatic optics positioning (CSM),
 * P51-P53.agc, Comanche055 pp. 764-768 (R52, R53JOB, R52L, TERM52,
 * ADVORB), pp. 775-779 (SR52.1, ADVTRACK, ROTA).
 *
 * R52 runs in the job of its caller (R51, P22, P23): every half second it
 * computes the sextant shaft and trunnion angles (SAC, PAC) of the target
 * line of sight 1.3 s ahead (SR52.1) and, with the optics in CMC mode,
 * hands them to the T4RUPT optics drive (DESOPTT, DESOPTS).  When the
 * crew switches the optics to manual, R53 is started in its own job
 * (R53JOB); R53 ends with TERMIFLG set, and R52 then returns to its
 * caller (SAVQR52, here agc_align_ram()->auto_optics_return).  With the
 * LM as target (TARG1FLG) the cycle goes through R61CSM (tracking
 * attitude) instead.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

#define DT_HALF_SEC      ((uint16_t) 50)  /* .5SEC  DELAYJOB cs            */
#define DT_SYNC          ((uint16_t) 180) /* 1.8SEC DEC 180 cs             */
#define AOP_LEAD_CS      ((agc_dp_t) 130) /* 1.3SECDP 2DEC 130, cs B28     */
#define VN_OPTICS_ANGLES ALIGN_VN(6, 92)  /* V06N92 VN 00692               */
#define VN_ALARM         ALIGN_VN(5, 9)   /* V05N09 (PRIOLARM display)     */
#define ALM_TRUNNION_90 \
    ((agc_alarm_code_t) 0404)             /* OCT404: trunnion > 90 deg */
#define ALM_TRUNNION_50 \
    ((agc_alarm_code_t) 0407)             /* 407: trunnion > 50 deg    */

/* 38TRDEG 2DEC .66666667 (50 degrees of trunnion on the PAC scale).      */
#define TRUN_50DEG ((agc_dp_t) 178956972)

/* 20DEGSMN DEC -07199, DEC -0 (the low limit of the PAC scale).          */
#define TRUN_LOW         ((agc_dp_t) (-7199 * 16384))
#define MPERIOD          ((agc_dp_t) 12782628) /* MPERIOD 2DEC .047619 revs     */
#define DP_ONE_SIXTH     ((agc_dp_t) 44739241) /* DP1/6 2DEC .16666666 revs     */
#define LANDMARK_PERIODS ((agc_sp_t) 07)       /* MASK SEVEN                   */
#define N_OVER_16_SHIFT  24                    /* N x BIT11 as a DP fraction /16 */

enum
{
    SR52_OVER_90  = 0, /* SR52E1: QMIN +0 -> R52L             */
    SR52_OVER_50  = 1, /* SR52E2: QMIN +1 -> R52J             */
    SR52_UNDER_50 = 2  /* SR52E3: QMIN +2                     */
};

static bool cmc_mode(void)
{
    return (agc_erase_swsample() > 0); /* CA SWSAMPLE  BZMF     */
}

/*
 * ROTA (Comanche p. 779) -- rotate the half-unit LOS by angle (revs)
 * about the half-unit axis UR (STAR):
 *   UNIT(sin A (UR x LOS) + (UR.LOS) UR + cos A (LOS - (UR.LOS) UR))
 * with the AGC's shifts (VSL2, VSL2, VSL1) and order of the adds.
 */
agc_vec_t agc_csm_rota(agc_interp_t *ip, agc_vec_t axis, agc_vec_t los, agc_dp_t angle)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (los);
    }

    if (!AGC_ASSERT((angle >= AGC_DP_MIN) && (angle <= AGC_DP_MAX)))
    {
        return (los);
    }

    const agc_dp_t  s  = agc_dp_sin(ip, angle); /* 1/2 SIN */
    const agc_vec_t t1 = agc_vec_shl(ip, agc_vec_scale(ip, agc_vec_cross(ip, axis, los), s), 2);
    const agc_dp_t  d  = agc_vec_dot(ip, axis, los);
    const agc_vec_t t2 = agc_vec_shl(ip, agc_vec_scale(ip, axis, d), 2);
    const agc_dp_t  c  = agc_dp_cos(ip, angle); /* 1/2 COS */
    const agc_vec_t t3 = agc_vec_shl(ip, agc_vec_scale(ip, agc_vec_sub(ip, los, t2), c), 1);

    return (agc_vec_unit(ip, agc_vec_add(ip, agc_vec_add(ip, t3, t2), t1), NULL, NULL));
}

/*
 * ADVTRACK (Comanche p. 778) -- advanced orbit tracking of a landmark
 * N orbits ahead (LANDMARK bits 1-3): the CSM's down vector at AOPTIME
 * is rotated by N/16 x MPERIOD about the moon's spin axis (UZ from
 * RP-TO-R), then by 1/6 rev - that angle about -UNIT(V x R).
 */
agc_status_t agc_csm_advtrack(agc_interp_t *ip, agc_job_id_t resume_job, agc_vec_t *los)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(los != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t     *ram = agc_align_ram();
    agc_navaux_padload_t pad;
    agc_status_t         st  = agc_navaux_get_padload(&pad);

    ram->aoptime       = align_loadtime();
    const agc_vec_t uz = agc_vec_make(0, 0, AGC_DP_HALF); /* UNITZ   */

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_rp_to_r(ip, &pad, agc_flag_test(AGC_FLAG_LUNAFLAG), uz, ram->aoptime, &ram->star);
    }

    agc_integ_out_t o;

    if (st == AGC_STATUS_OK)
    {
        st = align_integ(ip, ALIGN_CSMCONIC, ram->aoptime, resume_job, &o);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    const agc_vec_t axis2 = agc_vec_unit(ip, agc_vec_cross(ip, o.vatt, o.ratt), NULL, NULL);
    agc_vec_t       v     = agc_vec_neg(agc_vec_unit(ip, o.ratt, NULL, NULL));
    const agc_sp_t  n     = (agc_sp_t) (agc_erase_landmark() & LANDMARK_PERIODS);
    const agc_dp_t  n16   = (agc_dp_t) n * (1 << N_OVER_16_SHIFT); /* N/16      */

    ram->aoptime = agc_dp_mul(ip, n16, MPERIOD);
    v            = agc_csm_rota(ip, ram->star, v, ram->aoptime);
    ram->star    = axis2;
    ram->aoptime = agc_dp_sub(ip, DP_ONE_SIXTH, ram->aoptime);
    *los         = agc_csm_rota(ip, ram->star, v, ram->aoptime);

    return (AGC_STATUS_OK);
}

/* LEM52 / LMK52 / LMKLMCOM: the target (LM or landmark) position at
 * AOPTIME, then LOS = UNIT(target - RATT(CSM)).                          */
static agc_status_t target_los(agc_interp_t *ip, agc_job_id_t resume_job, agc_vec_t *los)
{
    agc_align_ram_t *ram = agc_align_ram();
    agc_status_t     st  = AGC_STATUS_OK;
    agc_integ_out_t  o;

    if (agc_flag_test(AGC_FLAG_TARG1FLG))
    { /* LEM52   */
        st        = align_integ(ip, ALIGN_LEMCONIC, ram->aoptime, resume_job, &o);
        ram->star = o.ratt;
    }
    else
    { /* LMK52   */
        agc_navaux_padload_t pad;

        st = agc_navaux_get_padload(&pad);
        agc_latlong_t ll;

        ll.lat                      = ram->landlat; /* LLASRDA */
        ll.lon                      = agc_dp_shl(ip, ram->landlong, 1);
        ll.alt                      = ram->landalt;
        ll.eradm                    = 0;
        const agc_latlong_opt_t opt = {
            .lunar = agc_flag_test(AGC_FLAG_LUNAFLAG),
            .erad = agc_flag_test(AGC_FLAG_ERADFLAG)
        };

        if (st == AGC_STATUS_OK)
        {
            st = agc_latlong_to_r(ip, &pad, opt, &ll, ram->aoptime, &ram->star, NULL);
        }
    }

    if (st == AGC_STATUS_OK)
    {
        st = align_integ(ip, ALIGN_CSMCONIC, ram->aoptime, resume_job, &o);
    }

    if (st == AGC_STATUS_OK)
    {
        *los = agc_vec_unit(ip, agc_vec_sub(ip, ram->star, o.ratt), NULL, NULL);
    }

    return (st);
}

/* TRUN38: the PAC limits.  50 degrees or more -> SR52E2, else SR52E3.   */
static int trunnion_class(uint16_t pac)
{
    const agc_dp_t p = agc_dp_pack(align_word_to_sp(pac), 0); /* DLOAD PAC */

    if ((p - TRUN_50DEG) >= 0)
    {
        return (SR52_OVER_50); /* SR52E22  */
    }

    if (!AGC_ASSERT(p < TRUN_50DEG))
    {
        return (0); /* no valid result */
    }

    return (((p - TRUN_LOW) >= 0) ? SR52_UNDER_50 : SR52_OVER_50);
}

/*
 * SR52.1 (Comanche p. 775) -- AOPTIME = now + 1.3 s (unless resumed after
 * an integration wait); the target LOS (LM, landmark, advanced landmark
 * or the STARSAV1/2 star); STAR = UNIT(REFSMMAT LOS); CDUTRIG; CALCSXA
 * -> SAC, PAC.  *result: 0 trunnion over 90 deg (CULTFLAG), 1 over 50 deg,
 * 2 under 50 deg.
 */
agc_status_t agc_csm_target_optics_angles(agc_interp_t *ip, bool resumed, agc_job_id_t resume_job, int *result)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(result != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();

    if (!resumed)
    {
        ram->aoptime = agc_dp_add(ip, align_loadtime(), AOP_LEAD_CS);
    }

    agc_vec_t    los;
    agc_status_t st = AGC_STATUS_OK;

    if (agc_flag_test(AGC_FLAG_TARG1FLG) || (agc_flag_test(AGC_FLAG_TARG2FLG) && !agc_flag_test(AGC_FLAG_ADVTRK)))
    {
        st = target_los(ip, resume_job, &los);
    }
    else if (agc_flag_test(AGC_FLAG_TARG2FLG))
    {
        st = agc_csm_advtrack(ip, resume_job, &los); /* ADVTRACK */
    }
    else
    {
        los = (ram->starind != 0) ? ram->starsav1 : ram->starsav2; /* STAR52 */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    const agc_mat_t refsmmat = agc_erase_refsmmat();

    ram->star          = agc_vec_unit(ip, agc_mat_vec(ip, &refsmmat, los), NULL, NULL); /* COM52 */
    agc_cdutrig_t t;
    bool          cult = false;

    agc_guid_cdutrig(&t); /* CDUTRIG  */
    st      = AGC_STATUS_OK;
    st      = agc_csm_calcsxa(ip, ram->star, &t, &ram->sac, &ram->pac, &cult);
    *result = cult ? SR52_OVER_90 : trunnion_class(ram->pac);

    return (st);
}

/* R52F: wait 1/2 s (or 1.8 s from R52SYNC), then R52 continues. */
static agc_status_t auto_optics_wait(uint16_t dt)
{
    if (!AGC_ASSERT((dt == DT_HALF_SEC) || (dt == DT_SYNC)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_status_t st = agc_svc_delayjob(dt, AGC_JOB_AUTO_OPTICS_POSITIONING_DELAYED);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/*
 * R52JA / R52E: show V06N92 (GODSPR) unless the target is the LM or R53 is
 * running; in CMC mode with TRUNFLAG set, DESOPTT = PAC, DESOPTS = SAC.
 */
static agc_status_t auto_optics_drive(const agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram();

    if (!agc_flag_test(AGC_FLAG_TARG1FLG) && !agc_flag_test(AGC_FLAG_SIGHTING_MARK_REQUESTED))
    {
        const agc_status_t st = align_show(ctx, AGC_DSKY_GODSPR, VN_OPTICS_ANGLES);

        if (!AGC_ASSERT(st == AGC_STATUS_OK))
        {
            return (AGC_STATUS_BAD_ARG);
        }
    }

    if (cmc_mode() && agc_flag_test(AGC_FLAG_TRUNFLAG))
    { /* R52E     */
        ram->desoptt = ram->pac;
        ram->desopts = ram->sac;
    }

    return (auto_optics_wait(DT_HALF_SEC));
}

/*
 * R52D: SR52.1 and its three returns.  Over 90 deg: R52L (the LM target
 * goes to R52J); over 50 deg: R52J (CLEAR TRUNFLAG, alarm 407);
 * under 50 deg: UPFLAG TRUNFLAG.  Then R52JA.
 */
static agc_status_t auto_optics_target(const agc_job_ctx_t *ctx, bool resumed)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    int          r  = SR52_OVER_90;
    agc_status_t st = agc_csm_target_optics_angles(&ip, resumed, AGC_JOB_AUTO_OPTICS_POSITIONING_RESUME, &r);

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* INTSTALL */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if ((r == SR52_OVER_90) && !agc_flag_test(AGC_FLAG_TARG1FLG))
    { /* R52L */
        agc_prioalarm(ALM_TRUNNION_90);

        return (align_flash(ctx, AGC_DSKY_PRIODSPR, VN_ALARM, AGC_JOB_AUTO_OPTICS_POSITIONING_TERMINATE, AGC_JOB_AUTO_OPTICS_POSITIONING_WAIT, AGC_JOB_AUTO_OPTICS_POSITIONING_WAIT)); /* ENDOFJOB */
    }

    if (r == SR52_UNDER_50)
    {
        agc_svc_upflag(AGC_FLAG_TRUNFLAG); /* UPFLAG   */
    }
    else
    {
        agc_svc_downflag(AGC_FLAG_TRUNFLAG); /* R52J     */
        agc_alarm(ALM_TRUNNION_50);
    }

    return (auto_optics_drive(ctx));
}

/*
 * R52C: optics in CMC mode -> R52D; manual -> R52M: unless R53 already
 * runs, start R53JOB (FINDVAC PRIO24); then R52F.
 */
static agc_status_t auto_optics_cycle(const agc_job_ctx_t *ctx)
{
    if (cmc_mode())
    {
        return (auto_optics_target(ctx, false));
    }

    if (!agc_flag_test(AGC_FLAG_SIGHTING_MARK_REQUESTED))
    { /* R52M     */
        agc_inhint();
        const agc_status_t st = agc_exec_findvac(ALIGN_PRIO24, AGC_JOB_AUTO_OPTICS_SIGHTING);

        agc_relint();

        if (!AGC_ASSERT(st == AGC_STATUS_OK))
        {
            return (AGC_STATUS_BAD_ARG);
        }
    }

    return (auto_optics_wait(DT_HALF_SEC));
}

/*
 * R52VRB / R52A: DESOPTT, DESOPTS = CDUT, CDUS; OPTIND = 0; CLEAR
 * R53FLAG; SET TRUNFLAG; with the LM as target go to R61CSM (R52H),
 * otherwise CLEAR TERMIFLG and start the cycle (R52C).
 */
static agc_status_t auto_optics_start(const agc_job_ctx_t *ctx, agc_job_id_t ret)
{
    agc_align_ram_t *ram = agc_align_ram();

    ram->auto_optics_return = ret; /* SAVQR52  */
    ram->desoptt            = agc_erase_cdut();
    ram->desopts            = agc_erase_cdus();
    agc_erase_set_optind(0);
    agc_flag_clear(AGC_FLAG_SIGHTING_MARK_REQUESTED);
    agc_flag_set(AGC_FLAG_TRUNFLAG);

    if (agc_flag_test(AGC_FLAG_TARG1FLG))
    { /* R52H     */
        return (agc_rendz_tracking_attitude_csm(ctx, AGC_JOB_AUTO_OPTICS_POSITIONING_TRACKED));
    }

    agc_flag_clear(AGC_FLAG_TERMIFLG);

    return (auto_optics_cycle(ctx));
}

/* R52 (Comanche p. 764): STQ SAVQR52, CLEAR ADVTRK. */
agc_status_t agc_csm_auto_optics_positioning(const agc_job_ctx_t *ctx, agc_job_id_t ret)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(ret != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_clear(AGC_FLAG_ADVTRK);

    return (auto_optics_start(ctx, ret));
}

/* ADVORB (Comanche p. 768): R52 with ADVTRK, LUNAFLAG and ERADFLAG set. */
agc_status_t agc_csm_auto_optics_advanced_orbit(const agc_job_ctx_t *ctx, agc_job_id_t ret)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(ret != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_set(AGC_FLAG_ADVTRK);
    agc_flag_set(AGC_FLAG_LUNAFLAG);
    agc_flag_set(AGC_FLAG_ERADFLAG);

    return (auto_optics_start(ctx, ret));
}

/* R52 woken from SR52.1's integration wait. */
agc_status_t agc_job_auto_optics_positioning_resume(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_align_ram()->auto_optics_return != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (auto_optics_target(ctx, true));
}

/* R52F (PROCEED / ENTER on the 404 alarm): wait 1/2 s. */
agc_status_t agc_job_auto_optics_positioning_wait(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_align_ram()->auto_optics_return != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (auto_optics_wait(DT_HALF_SEC));
}

/*
 * R52F +3 (after DELAYJOB): LM target -> R52HA (R61CSM); otherwise
 * TERMIFLG clear -> R52C, set -> R52Q (GOTO SAVQR52).
 */
agc_status_t agc_job_auto_optics_positioning_delayed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_flag_test(AGC_FLAG_TARG1FLG))
    {
        return (agc_rendz_tracking_attitude_csm(ctx, AGC_JOB_AUTO_OPTICS_POSITIONING_TRACKED));
    }

    if (!agc_flag_test(AGC_FLAG_TERMIFLG))
    {
        return (auto_optics_cycle(ctx));
    }

    return (align_goto(agc_align_ram()->auto_optics_return)); /* R52Q     */
}

/*
 * R52HA after R61CSM: TRACKFLG clear -> R52Q; UPDATFLG clear -> R52SYNC
 * (wait 1.8 s); PRFTRKAT set -> R52D; otherwise R52SYNC.
 */
agc_status_t agc_job_auto_optics_positioning_tracked(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!agc_flag_test(AGC_FLAG_TRACKFLG))
    {
        return (align_goto(agc_align_ram()->auto_optics_return));
    }

    if (agc_flag_test(AGC_FLAG_UPDATFLG) && agc_flag_test(AGC_FLAG_PRFTRKAT))
    {
        return (auto_optics_target(ctx, false)); /* R52I     */
    }

    return (auto_optics_wait(DT_SYNC)); /* R52SYNC  */
}

/* R53JOB (FINDVAC PRIO24): CALL R53, then ENDPLAC (ENDOFJOB). */
agc_status_t agc_job_auto_optics_sighting(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_csm_sighting_mark(ctx, AGC_JOB_ENDOFJOB));
}

/*
 * TERM52 (Comanche p. 768): CLEARMRK, MKRELEAS (kill the mark system),
 * OPTCADR = 0, KLEENEX (clear extended verbs), GOTOPOOH.
 */
agc_status_t agc_job_auto_optics_positioning_terminate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dsky_clearmrk();
    agc_status_t st = agc_sxtmark_mkreleas();

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_status_t st_optcadr = agc_erase_set_modecadr(AGC_IMU_DEV_OPTICS, 0);

    if (!AGC_ASSERT(st_optcadr == AGC_STATUS_OK)) /* OPTCADR  */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_KLEENEX, 0, AGC_JOB_GOTOPOOH, AGC_JOB_GOTOPOOH, AGC_JOB_GOTOPOOH));
}
