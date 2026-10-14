/**********************************************************************
 * FILE NAME: auto_optics_positioning.c
 *
 * PURPOSE:
 *   R52 automatic optics positioning (CSM).  R52 runs in the job of its
 *   caller (R51, P22, P23): every half second it computes the sextant
 *   shaft and trunnion angles (SAC, PAC) of the target line of sight 1.3
 *   s ahead (SR52.1) and, with the optics in CMC mode, hands them to the
 *   T4RUPT optics drive (DESOPTT, DESOPTS).  When the crew switches the
 *   optics to manual, R53 is started in its own job (R53JOB); R53 ends
 *   with TERMIFLG set, and R52 then returns to its caller (SAVQR52, here
 *   agc_align_ram()->auto_optics_return).  With the LM as target
 *   (TARG1FLG) the cycle goes through R61CSM (tracking attitude) instead.
 *
 * AGC SOURCE:
 *   Comanche055 P51-P53.agc pp. 764-768 (R52, R53JOB, R52L, TERM52, ADVORB)
 *   Comanche055 P51-P53.agc pp. 775-779 (SR52.1, ADVTRACK, ROTA)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   SWSAMPLE      agc_sp_t  I    unscaled       CMC mode test
 *   LANDMARK      agc_sp_t  I    unscaled       orbits ahead (ADVTRACK)
 *   AOPTIME, STAR, SAC, PAC, DESOPTT, DESOPTS, LANDLAT, LANDLONG, LANDALT, STARSAV1/2: agc_align_ram()
 *   REFSMMAT      agc_mat_t I    half-unit      agc_erase_refsmmat()
 *   TARG1FLG, TARG2FLG, ADVTRK, TRUNFLAG, R53FLAG, TERMIFLG, LUNAFLAG, ERADFLAG  flags (I/O)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dsky_clearmrk     DSKY displays, astronaut returns (dsky.h)
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_ram         alignment routines of this lane (alignment.h)
 *   agc_exec_findvac      executive: jobs, VAC areas, sleep / wake (exec.h)
 *   agc_svc_*             service routines: DELAYJOB, flags, B5OFF (service.h)
 *   agc_imu_cdutrig      CDUTRIG, *NBSM* / *SMNB*, PIPASR (imu.h)
 *   agc_planet_rp_to_r    RP-TO-R / R-TO-RP (auxiliary.h)
 *   agc_latlong_to_r      LAT-LONG / LALOTORV (auxiliary.h)
 *   agc_navaux_get_padload  pad loads (auxiliary.h)
 *   agc_vec_*             vector arithmetic of the interpreter (interp.h)
 *   agc_mat_vec           matrix arithmetic of the interpreter (interp.h)
 *   agc_dp_*              DP arithmetic of the interpreter (interp.h, word.h)
 *   agc_interp_init       interpreter state: OVFIND (interp.h)
 *   agc_flag_*            flag words (flags_common.h)
 *   agc_erase_*           erasable accessors (state.h)
 *   agc_alarm             ALARM (exec.h)
 *   agc_prioalarm         PRIOLARM (exec.h)
 *   agc_inhint            INHINT / RELINT (exec.h)
 *   agc_relint            INHINT / RELINT (exec.h)
 *   agc_csm_*             CSM optics and sighting routines (alignment.h)
 *   agc_sxtmark_mkreleas  SXTMARK routines (alignment.h)
 *   agc_rendz_tracking_attitude_csm  see its header
 *   agc_extverb_redisplay  EXDSPRET of V41 N91 (service.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 *
 * NOTES:
 *   ADVTRACK chooses the moon in RP-TO-R by BHIZ on the time (finding
 *   A14).
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

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
#define TRUN_50DEG ((agc_dp_t) 178956972) /* 50 deg of trunnion, PAC scale */

/* 20DEGSMN DEC -07199, DEC -0 (the low limit of the PAC scale).          */
#define TRUN_LOW         ((agc_dp_t) (-7199 * 16384)) /* PAC scale low limit, DP */
#define MPERIOD          ((agc_dp_t) 12782628)        /* MPERIOD 2DEC .047619 revs     */
#define DP_ONE_SIXTH     ((agc_dp_t) 44739241)        /* DP1/6 2DEC .16666666 revs     */
#define LANDMARK_PERIODS ((agc_sp_t) 07)              /* MASK SEVEN                   */
#define N_OVER_16_SHIFT  24                           /* N x BIT11 as a DP fraction /16 */

enum
{
    SR52_OVER_90  = 0, /* SR52E1: QMIN +0 -> R52L             */
    SR52_OVER_50  = 1, /* SR52E2: QMIN +1 -> R52J             */
    SR52_UNDER_50 = 2  /* SR52E3: QMIN +2                     */
};

/**********************************************************************
 * AGC: none (CA SWSAMPLE, BZMF in R52)
 *
 * The optics are in CMC mode (SWSAMPLE > 0).
 *
 * FUNCTION NAME: cmc_mode
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true in CMC mode
 **********************************************************************/
static bool cmc_mode(void)
{
    return (agc_erase_swsample() > 0); /* CA SWSAMPLE  BZMF     */
}

/**********************************************************************
 * ROTA (Comanche p. 779) -- rotate the half-unit LOS by angle (revs) about
 * the half-unit axis UR (STAR): UNIT(sin A (UR x LOS) + (UR.LOS) UR + cos A
 * (LOS - (UR.LOS) UR)) with the AGC's shifts (VSL2, VSL2, VSL1) and order
 * of the adds.
 *
 * FUNCTION NAME: agc_csm_rota
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   axis      agc_vec_t       I    UR, the rotation axis, half-unit (2^1)
 *   los       agc_vec_t       O    line of sight, half-unit (2^1)
 *   angle     agc_dp_t        I    rotation angle, revs 2^0
 *
 * RETURNS: agc_vec_t -- the rotated LOS, half-unit (2^1)
 **********************************************************************/
static agc_vec_t agc_csm_rota(agc_interp_t *ip, agc_vec_t axis, agc_vec_t los, agc_dp_t angle)
{
    const agc_dp_t  s  = agc_dp_sin(angle);                                                      /* 1/2 SIN of the angle, scale 2^1 */
    const agc_vec_t t1 = agc_vec_shl(ip, agc_vec_scale(ip, agc_vec_cross(ip, axis, los), s), 2); /* (UR x LOS) SIN VSL2, half-unit (2^1) */
    const agc_dp_t  d  = agc_vec_dot(ip, axis, los);                                             /* UR . LOS, scale 2^2 */
    const agc_vec_t t2 = agc_vec_shl(ip, agc_vec_scale(ip, axis, d), 2);                         /* (UR . LOS) UR VSL2, half-unit (2^1) */
    const agc_dp_t  c  = agc_dp_cos(angle);                                                      /* 1/2 COS of the angle, scale 2^1 */
    const agc_vec_t t3 = agc_vec_shl(ip, agc_vec_scale(ip, agc_vec_sub(ip, los, t2), c), 1);     /* (LOS - t2) COS VSL1, half-unit (2^1) */

    return (agc_vec_unit(ip, agc_vec_add(ip, agc_vec_add(ip, t3, t2), t1), NULL, NULL));
}

/**********************************************************************
 * ADVTRACK (Comanche p. 778) -- advanced orbit tracking of a landmark N
 * orbits ahead (LANDMARK bits 1-3): the CSM's down vector at AOPTIME is
 * rotated by N/16 x MPERIOD about the moon's spin axis (UZ from RP-TO-R),
 * then by 1/6 rev - that angle about -UNIT(V x R).
 *
 * FUNCTION NAME: agc_csm_advtrack
 *
 * ARGUMENTS:
 *   Argument    Type            I/O  Description (scale)
 *   ----------  --------------  ---  ------------------------------------
 *   ip          agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   resume_job  agc_job_id_t    I    job resumed after an integration wait
 *                                    (INTSTALL)
 *   los         agc_vec_t *     O    line of sight, half-unit (2^1)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t agc_csm_advtrack(agc_interp_t *ip, agc_job_id_t resume_job, agc_vec_t *los)
{
    agc_align_ram_t     *ram = agc_align_ram(); /* alignment erasables */
    agc_navaux_padload_t pad;                   /* pad loads (ephemeris, RLS) */

    agc_navaux_get_padload(&pad);
    ram->aoptime       = align_loadtime();
    const agc_vec_t uz = agc_vec_make(0, 0, AGC_DP_HALF);                                                            /* UNITZ, half-unit (2^1) */
    agc_status_t    st = agc_planet_rp_to_r(ip, &pad, agc_dp_high(ram->aoptime) != 0, uz, ram->aoptime, &ram->star); /* BHIZ on MPAC = the time: moon */

    agc_integ_out_t o; /* integration output (RATT, VATT) */

    if (st == AGC_STATUS_OK)
    {
        st = align_integ(ip, ALIGN_CSMCONIC, ram->aoptime, resume_job, &o);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    const agc_vec_t axis2 = agc_vec_unit(ip, agc_vec_cross(ip, o.vatt, o.ratt), NULL, NULL); /* UNIT(V x R): second rotation axis, half-unit (2^1) */
    agc_vec_t       v     = agc_vec_neg(agc_vec_unit(ip, o.ratt, NULL, NULL));               /* -UNIT(RATT), the down vector, half-unit (2^1) */
    const agc_sp_t  n     = (agc_sp_t) (agc_erase_landmark() & LANDMARK_PERIODS);            /* LANDMARK bits 1-3: orbits ahead (unscaled) */
    const agc_dp_t  n16   = (agc_dp_t) n * (1 << N_OVER_16_SHIFT);                           /* N/16, scale 2^0 (a fraction) */

    ram->aoptime = agc_dp_mul_trunc(n16, MPERIOD); /* SLOAD DMP */
    v            = agc_csm_rota(ip, ram->star, v, ram->aoptime);
    ram->star    = axis2;
    ram->aoptime = agc_dp_sub(ip, DP_ONE_SIXTH, ram->aoptime);
    *los         = agc_csm_rota(ip, ram->star, v, ram->aoptime);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: TRUN38 (Comanche055 P51-P53.agc p. 777)
 *
 * TRUN38: the PAC limits.  50 degrees or more -> SR52E2, else SR52E3.
 *
 * FUNCTION NAME: trunnion_class
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   pac       uint16_t  O    trunnion angle command, CDU word
 *
 * RETURNS: int -- SR52_OVER_50 (PAC >= 50 deg or below the low limit) or
 *          SR52_UNDER_50
 **********************************************************************/
static int trunnion_class(uint16_t pac)
{
    const agc_dp_t p = agc_dp_pack(align_word_to_sp(pac), 0); /* PAC as a DP (DLOAD PAC), scale 2^0 */

    if ((p - TRUN_50DEG) >= 0)
    {
        return (SR52_OVER_50); /* SR52E22  */
    }

    return (((p - TRUN_LOW) >= 0) ? SR52_UNDER_50 : SR52_OVER_50); /* within the PAC scale */
}

/**********************************************************************
 * SR52.1 (Comanche p. 775) -- AOPTIME = now + 1.3 s (unless resumed after
 * an integration wait); the target LOS (LM, landmark, advanced landmark or
 * the STARSAV1/2 star); STAR = UNIT(REFSMMAT LOS); CDUTRIG; CALCSXA -> SAC,
 * PAC.  *result: 0 trunnion over 90 deg (CULTFLAG), 1 over 50 deg, 2 under
 * 50 deg.
 *
 * FUNCTION NAME: agc_csm_target_optics_angles
 *
 * ARGUMENTS:
 *   Argument    Type            I/O  Description (scale)
 *   ----------  --------------  ---  ------------------------------------
 *   ip          agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   resumed     bool            I    true when re-entered after an
 *                                    integration wait
 *   resume_job  agc_job_id_t    I    job resumed after an integration wait
 *                                    (INTSTALL)
 *   result      int *           O    SR52_OVER_90 (CULTFLAG), SR52_OVER_50
 *                                    or SR52_UNDER_50
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t agc_csm_target_optics_angles(agc_interp_t *ip, bool resumed, agc_job_id_t resume_job, int *result)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    if (!resumed)
    {
        ram->aoptime = agc_dp_add(ip, align_loadtime(), AOP_LEAD_CS);
    }

    agc_vec_t    los;                /* target line of sight (reference), half-unit (2^1) */
    agc_status_t st = AGC_STATUS_OK; /* status of the call */

    if (agc_flag_test(AGC_FLAG_TARG1FLG) || (agc_flag_test(AGC_FLAG_TARG2FLG) && !agc_flag_test(AGC_FLAG_ADVTRK)))
    {
        /* LEM52 / LMK52 / LMKLMCOM: the target (LM or landmark) position at
         * AOPTIME in STAR, then LOS = UNIT(STAR - RATT(CSM)).                     */
        agc_integ_out_t o; /* integration output (RATT, VATT) */

        if (agc_flag_test(AGC_FLAG_TARG1FLG))
        { /* LEM52   */
            st = align_integ(ip, ALIGN_LEMCONIC, ram->aoptime, resume_job, &o);

            if (st == AGC_STATUS_OK)
            {
                ram->star = o.ratt;
            }
        }
        else
        { /* LMK52   */
            agc_navaux_padload_t    pad; /* pad loads (ephemeris, RLS) */
            agc_latlong_t           ll;  /* latitude, longitude, altitude */
            const agc_latlong_opt_t opt = {.lunar = agc_flag_test(AGC_FLAG_LUNAFLAG), .erad = agc_flag_test(AGC_FLAG_ERADFLAG)};

            ll.lat   = ram->landlat; /* LLASRDA */
            ll.lon   = agc_dp_shl(ip, ram->landlong, 1);
            ll.alt   = ram->landalt;
            ll.eradm = 0;
            agc_navaux_get_padload(&pad);
            st       = agc_latlong_to_r(ip, &pad, opt, &ll, ram->aoptime, &ram->star, NULL);
        }

        if (st == AGC_STATUS_OK)
        {
            st = align_integ(ip, ALIGN_CSMCONIC, ram->aoptime, resume_job, &o);
        }

        if (st == AGC_STATUS_OK)
        {
            los = agc_vec_unit(ip, agc_vec_sub(ip, ram->star, o.ratt), NULL, NULL);
        }
    } /* end if */
    else if (agc_flag_test(AGC_FLAG_TARG2FLG))
    {
        st = agc_csm_advtrack(ip, resume_job, &los); /* ADVTRACK */
    }
    else
    {
        los = (ram->starind != 0) ? ram->starsav1 : ram->starsav2; /* STAR52 */
    }

    if (st == AGC_STATUS_OK)
    {
        const agc_mat_t refsmmat = agc_erase_refsmmat(); /* REFSMMAT, half-unit rows */

        ram->star          = agc_vec_unit(ip, agc_mat_vec(ip, &refsmmat, los), NULL, NULL); /* COM52 */
        agc_cdutrig_t t;                                                                    /* CDUTRIG sines and cosines */
        bool          cult = false;                                                         /* CULTFLAG from CALCSXA */

        agc_imu_cdutrig(&t);                                      /* CDUTRIG  */
        agc_csm_calcsxa(ip, ram->star, &t, &ram->sac, &ram->pac, &cult);
        *result = cult ? SR52_OVER_90 : trunnion_class(ram->pac); /* CULTFLAG: trunnion over 90 deg */
    }

    return (st);
} /* agc_csm_target_optics_angles */

/**********************************************************************
 * AGC: R52F (Comanche055 P51-P53.agc p. 745)
 *
 * R52F: wait 1/2 s (or 1.8 s from R52SYNC), then R52 continues.
 *
 * FUNCTION NAME: auto_optics_wait
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   dt        uint16_t  I    delay, cs
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t auto_optics_wait(uint16_t dt)
{
    return (agc_svc_delayjob(dt, AGC_JOB_AUTO_OPTICS_POSITIONING_DELAYED));
}

/**********************************************************************
 * AGC: R52D (Comanche055 P51-P53.agc p. 744)
 *
 * R52D: SR52.1 and its three returns.  Over 90 deg: R52L (the LM target
 * goes to R52J); over 50 deg: R52J (CLEAR TRUNFLAG, alarm 407); under 50
 * deg: UPFLAG TRUNFLAG.  Then R52JA.
 *
 * FUNCTION NAME: auto_optics_target
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *   resumed   bool                   I    true when re-entered after an
 *                                         integration wait
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t auto_optics_target(const agc_job_ctx_t *ctx, bool resumed)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    int          r  = SR52_OVER_90;                                                                           /* SR52.1 return: over 90, over 50, under 50 */
    agc_status_t st = agc_csm_target_optics_angles(&ip, resumed, AGC_JOB_AUTO_OPTICS_POSITIONING_RESUME, &r); /* status of the call */

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

    /*
     * R52JA / R52E: show V06N92 (GODSPR) unless the target is the LM or R53 is
     * running; in CMC mode with TRUNFLAG set, DESOPTT = PAC, DESOPTS = SAC.
     */
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    if (!agc_flag_test(AGC_FLAG_TARG1FLG) && !agc_flag_test(AGC_FLAG_SIGHTING_MARK_REQUESTED))
    {
        st = align_show(ctx, AGC_DSKY_GODSPR, VN_OPTICS_ANGLES);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    if (cmc_mode() && agc_flag_test(AGC_FLAG_TRUNFLAG))
    { /* R52E     */
        agc_erase_set_desoptt(ram->pac);
        agc_erase_set_desopts(ram->sac);
    }

    return (auto_optics_wait(DT_HALF_SEC));
} /* auto_optics_target */

/**********************************************************************
 * AGC: R52C (Comanche055 P51-P53.agc p. 744)
 *
 * R52C: optics in CMC mode -> R52D; manual -> R52M: unless R53 already
 * runs, start R53JOB (FINDVAC PRIO24); then R52F.
 *
 * FUNCTION NAME: auto_optics_cycle
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t auto_optics_cycle(const agc_job_ctx_t *ctx)
{
    if (cmc_mode())
    {
        return (auto_optics_target(ctx, false));
    }

    if (!agc_flag_test(AGC_FLAG_SIGHTING_MARK_REQUESTED))
    { /* R52M     */
        agc_inhint();
        const agc_status_t st = agc_exec_findvac(ALIGN_PRIO24, AGC_JOB_AUTO_OPTICS_SIGHTING); /* status of the call */

        agc_relint();

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    return (auto_optics_wait(DT_HALF_SEC));
}

/**********************************************************************
 * AGC: R52VRB (Comanche055 P51-P53.agc p. 744)
 *
 * R52VRB / R52A: DESOPTT, DESOPTS = CDUT, CDUS; OPTIND = 0; CLEAR R53FLAG;
 * SET TRUNFLAG; with the LM as target go to R61CSM (R52H), otherwise CLEAR
 * TERMIFLG and start the cycle (R52C).
 *
 * FUNCTION NAME: auto_optics_start
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *   ret       agc_job_id_t           I    job the routine continues as when
 *                                         it ends (the AGC return address)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t auto_optics_start(const agc_job_ctx_t *ctx, agc_job_id_t ret)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    ram->auto_optics_return = ret; /* SAVQR52  */
    agc_erase_set_desoptt(agc_erase_cdut());
    agc_erase_set_desopts(agc_erase_cdus());
    agc_erase_set_optind(0);
    agc_flag_clear(AGC_FLAG_SIGHTING_MARK_REQUESTED);
    agc_flag_set(AGC_FLAG_TRUNFLAG);

    if (agc_flag_test(AGC_FLAG_TARG1FLG))
    { /* R52H     */
        return (agc_rendz_tracking_attitude_csm(AGC_JOB_AUTO_OPTICS_POSITIONING_TRACKED));
    }

    agc_flag_clear(AGC_FLAG_TERMIFLG);

    return (auto_optics_cycle(ctx));
}

/**********************************************************************
 * R52 (Comanche p. 764): STQ SAVQR52, CLEAR ADVTRK.
 *
 * FUNCTION NAME: agc_csm_auto_optics_positioning
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *   ret       agc_job_id_t           I    job the routine continues as when
 *                                         it ends (the AGC return address)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_csm_auto_optics_positioning(const agc_job_ctx_t *ctx, agc_job_id_t ret)
{
    agc_flag_clear(AGC_FLAG_ADVTRK);

    return (auto_optics_start(ctx, ret));
}

/**********************************************************************
 * ADVORB (Comanche p. 768): R52 with ADVTRK, LUNAFLAG and ERADFLAG set.
 *
 * FUNCTION NAME: agc_csm_auto_optics_advanced_orbit
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *   ret       agc_job_id_t           I    job the routine continues as when
 *                                         it ends (the AGC return address)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_csm_auto_optics_advanced_orbit(const agc_job_ctx_t *ctx, agc_job_id_t ret)
{
    agc_flag_set(AGC_FLAG_ADVTRK);
    agc_flag_set(AGC_FLAG_LUNAFLAG);
    agc_flag_set(AGC_FLAG_ERADFLAG);

    return (auto_optics_start(ctx, ret));
}

/**********************************************************************
 * AGC: R52 (Comanche055 P51-P53.agc p. 744)
 *
 * R52 woken from SR52.1's integration wait.
 *
 * FUNCTION NAME: agc_job_auto_optics_positioning_resume
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_auto_optics_positioning_resume(agc_job_ctx_t *ctx)
{
    return (auto_optics_target(ctx, true));
}

/**********************************************************************
 * AGC: R52F (Comanche055 P51-P53.agc p. 745)
 *
 * R52F (PROCEED / ENTER on the 404 alarm): wait 1/2 s.
 *
 * FUNCTION NAME: agc_job_auto_optics_positioning_wait
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_auto_optics_positioning_wait(void)
{
    return (auto_optics_wait(DT_HALF_SEC));
}

/**********************************************************************
 * AGC: R52F (Comanche055 P51-P53.agc p. 745)
 *
 * R52F +3 (after DELAYJOB): LM target -> R52HA (R61CSM); otherwise TERMIFLG
 * clear -> R52C, set -> R52Q (GOTO SAVQR52).
 *
 * FUNCTION NAME: agc_job_auto_optics_positioning_delayed
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_auto_optics_positioning_delayed(agc_job_ctx_t *ctx)
{
    if (agc_flag_test(AGC_FLAG_TARG1FLG))
    {
        return (agc_rendz_tracking_attitude_csm(AGC_JOB_AUTO_OPTICS_POSITIONING_TRACKED));
    }

    if (!agc_flag_test(AGC_FLAG_TERMIFLG))
    {
        return (auto_optics_cycle(ctx));
    }

    return (align_goto(agc_align_ram()->auto_optics_return)); /* R52Q     */
}

/**********************************************************************
 * AGC: R52HA (Comanche055 P51-P53.agc p. 745)
 *
 * R52HA after R61CSM: TRACKFLG clear -> R52Q; UPDATFLG clear -> R52SYNC
 * (wait 1.8 s); PRFTRKAT set -> R52D; otherwise R52SYNC.
 *
 * FUNCTION NAME: agc_job_auto_optics_positioning_tracked
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_auto_optics_positioning_tracked(agc_job_ctx_t *ctx)
{
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

/**********************************************************************
 * AGC: R53JOB (Comanche055 P51-P53.agc p. 746)
 *
 * R53JOB (FINDVAC PRIO24): CALL R53, then ENDPLAC (ENDOFJOB).
 *
 * FUNCTION NAME: agc_job_auto_optics_sighting
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_auto_optics_sighting(void)
{
    return (agc_csm_sighting_mark(AGC_JOB_ENDOFJOB));
}

/**********************************************************************
 * TERM52 (Comanche p. 768): CLEARMRK, MKRELEAS (kill the mark system),
 * OPTCADR = 0, KLEENEX (clear extended verbs), GOTOPOOH.
 *
 * FUNCTION NAME: agc_job_auto_optics_positioning_terminate
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_auto_optics_positioning_terminate(agc_job_ctx_t *ctx)
{
    agc_dsky_clearmrk();
    agc_status_t st = agc_sxtmark_mkreleas(); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_erase_set_modecadr(AGC_IMU_DEV_OPTICS, 0); /* OPTCADR  */

    return (align_flash(ctx, AGC_DSKY_KLEENEX, 0, AGC_JOB_GOTOPOOH, AGC_JOB_GOTOPOOH, AGC_JOB_GOTOPOOH));
}

/**********************************************************************
 * AGC: OPTC1 +4 (Comanche055 EXTENDED_VERBS.agc p.242) -- V41 N91
 * after the V24N92 load (V33 or ENTER): DESOPTS = SAC, DESOPTT = PAC,
 * the hand-over to the T4RUPT optics drive that R52 also makes; then V41
 * is shown again (EXDSPRET). The verb itself is OPTCOARK in
 * src/io/verbs/extended_verbs_csm.c; this continuation is filed with
 * the optics because SAC and PAC are the alignment erasables.
 *
 * FUNCTION NAME: agc_job_optics_coarse_drive
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: agc_status_t -- the display request status
 **********************************************************************/
agc_status_t agc_job_optics_coarse_drive(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ar = agc_align_ram(); /* SAC, PAC */

    agc_erase_set_desopts(ar->sac);
    agc_erase_set_desoptt(ar->pac);

    return (agc_extverb_redisplay(ctx, 41, AGC_JOB_OPTICS_COARSE_START));
}
