/**********************************************************************
 * FILE NAME: prelaunch_verify.c
 *
 * PURPOSE:
 *   Comanche prelaunch alignment, part 3: P03 optical verification of
 *   the gyrocompass (V65).  The crew loads the azimuth and elevation of
 *   two ground targets (OPTDATA, V06N41), marks each with the sextant
 *   (TARGDRVE, RETARG), and the program compares the marked lines of
 *   sight with the expected ones (AXISGEN, CALCGTA).  The resulting gyro
 *   angles are shown (V06N93) and, on PROCEED, added to the earth-rate
 *   compensation ERCOMP1 that P02 torques in.
 *
 * AGC SOURCE:
 *   Comanche055 IMU_CALIBRATION_AND_ALIGNMENT.agc pp. 449-454 (CM only)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   XSM..ZSM      agc_mat_t  I    half-unit      agc_align_ram()->xsm
 *   XNB..ZNB      agc_mat_t  I/O  half-unit      agc_align_ram()->xnb
 *   STARAD        agc_vec_t  O    half-unit      agc_align_ram()->starad
 *   STAR          agc_vec_t  O    half-unit      agc_align_ram()->star
 *   OGC,IGC,MGC   agc_dp_t   O    rev 2^0        agc_align_ram()->ogc
 *   DESOPTS/T     uint16_t   O    CDU            agc_align_ram()->desopt*
 *   ERCOMP1       agc_dp_t   I/O  rev 2^0        gyro buffer ERCOMP
 *   TAZEL1, RUN,  struct     I/O  see header     prelaunch_ram()
 *   TARG1/2, LOSVEC
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_csm_sxtang        SXTANG (optics angles for the target)
 *   agc_sxtmark           SXTMARK, MKRELEAS, QPRET, the mark VAC
 *   agc_imu_stall_dev     OPTSTALL
 *   agc_csm_sxtsm         SXTNB + TRG*NBSM (LITTLSUB)
 *   agc_imu_axisgen       AXISGEN
 *   agc_imu_calcgta       CALCGTA
 *   agc_dsky_display      GODSPRET V05N30, GOFLASH V06N41, V06N93
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ALARM 01602 (GTSOPTCS) when the optics stall ends bad; V34 on any
 *   flash or TRM03FLG ends P03 (GCOMP5) back in P02.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only.  P03 runs while P02's waitlist loop keeps gyrocompassing;
 *   restart phase 4.15 restarts at GCOMPVER, GCOMP5 sets group 4 off.
 *
 * NOTES:
 *   TARGDRVE's return address QPLAC is TARG1/2 here: 0 continues at
 *   NEXTBNKS (target 1 marked), 6 at the end of NEXBNKSS (target 2).
 *
 * ALGORITHM:
 *   Target i (0 or 1) at azimuth AZ (from north, clockwise) and elevation
 *   EL in the earth-reference frame of NBPOSPL (X up, Y and Z in the
 *   horizontal plane): TAR/EREF = (sin EL, -cos AZ cos EL, sin AZ cos EL)
 *   / 2.  Expected LOS in SM = XSM TAR/EREF; marked LOS = SXTSM(mark).
 *   AXISGEN(expected, marked) gives XDC, CALCGTA the three gyro angles.
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"
#include "prelaunch_internal.h"

#define VN_TARGET_NUMBER ((uint16_t) ((5 << 7) | 30)) /* V05N30E: VN 0530 */
#define VN_TARGET_AZ_EL  ((uint16_t) ((6 << 7) | 41)) /* VN0641: V06N41  */
#define VN_GYRO_ANGLES   ((uint16_t) ((6 << 7) | 93)) /* V06N93S: VN 0693 */
#define PHASE_VERIFY     ((uint16_t) 00154) /* PHASCHNG 4.15 (GCOMPVER)    */
#define PHASE_GROUP4_OFF ((uint16_t) 00004) /* PHASCHNG group 4 inactive   */
#define RUN_TARGET2      ((agc_sp_t) 2)     /* RUN: TAZEL1 +2 (CAF TWO)    */
#define TARG_TARGET2     ((agc_sp_t) 6)     /* TARG1/2: PD12 (CAF SIX)     */
#define TARGETS          2                  /* TAR/EREF builds two vectors */

/**********************************************************************
 * TAR/EREF / TAR1 (IMU_CALIBRATION_AND_ALIGNMENT.agc pp.449-450) -- the
 * two target vectors in the earth-reference frame from TAZEL1
 *
 * EL is stored with 90 deg = 1 (SLOAD SR2 gives revolutions), AZ as a
 * CDU word (RTB CDULOGIC).
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *  I/O  interpreter state
 *   tar  agc_vec_t[2]    O    PD06 (target 1), PD12 (target 2),
 *                             half-unit
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG on a NULL argument
 **********************************************************************/
static agc_status_t target_vectors(agc_interp_t *ip, agc_vec_t tar[TARGETS])
{
    const prelaunch_ram_t *r = prelaunch_ram();

    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(tar != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int i = 0; i < TARGETS; i++) /* TIX,2 TAR1: two passes        */
    {
        agc_sp_t el_sp  = align_word_to_sp(r->tazel[(2 * i) + 1]);
        agc_dp_t el     = agc_dp_shr(agc_dp_pack(el_sp, 0), 2); /* SR2   */
        agc_dp_t az     = agc_rtb_cdulogic(r->tazel[2 * i]);  /* rev  */
        agc_dp_t cos_el = agc_dp_cos(ip, el);                 /* PUSH */
        /* DMP SL1 on the TP product (MPAC+2's top bit reaches the LSB). */
        agc_dp_t s_az = align_dmp_shl(ip, agc_dp_sin(ip, az), cos_el, 1);
        agc_dp_t c_az = align_dmp_shl(ip, agc_dp_cos(ip, az), cos_el, 1);

        tar[i] = agc_vec_make(agc_dp_sin(ip, el), /* SIN(EL)            */
                              agc_dp_neg(c_az),   /* DCOMP              */
                              s_az);
    }

    if (!AGC_ASSERT(tar[0].c[0] >= -AGC_DP_HALF))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * OPTDATA +2 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.449) -- RUN, and the
 * display words: DSPTEM1, +1 = TAZEL1 +RUN (AZ, EL), DSPTEM1 +2 = the
 * target number (1 or 2)
 *
 * ARGUMENTS:
 *   run  agc_sp_t  I  RUN: 0 target 1, 2 target 2
 *
 * RETURNS: agc_status_t of the DSPTEM1 stores
 **********************************************************************/
static agc_status_t optdata_load(agc_sp_t run)
{
    prelaunch_ram_t *r = prelaunch_ram();

    if (!AGC_ASSERT((run == 0) || (run == RUN_TARGET2)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    r->run = run; /* LXCH RUN                                         */
    const uint16_t w[3] = {r->tazel[run], r->tazel[run + 1],
                           (run == 0) ? 1u : 2u}; /* BIT1 / TWO       */

    for (int i = 0; i < 3; i++)
    {
        agc_status_t st = agc_dsky_set_dsptem(i, w[i]); /* DSPTEM1 +i */

        if (!AGC_ASSERT(st == AGC_STATUS_OK))
        {
            return (st);
        }
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * GCOMPVER / SETNBPOS (IMU_CALIBRATION_AND_ALIGNMENT.agc p.449, V65) --
 * restart 4.15, MM 03, NBPOSPL; NBPOSPL sees MM 03 and goes on at
 * SETNBPOS +1
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the V65 job
 *
 * RETURNS: agc_status_t of SETNBPOS +1
 **********************************************************************/
agc_status_t agc_job_optical_verification(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state for NBPOSPL            */

    if (!AGC_ASSERT(ctx != NULL) ||
        !AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change(4, PHASE_VERIFY); /* PHASCHNG OCT 00154          */
    agc_status_t st = agc_phase_newmode(PRELAUNCH_MM_VERIFY); /* MM 03 */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    agc_interp_init(&ip);
    st = prelaunch_nbpospl(&ip); /* SETNBPOS: TC NBPOSPL              */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (prelaunch_verify_start(ctx)); /* CHECKMM 03: SETNBPOS +1 */
}

/**********************************************************************
 * SETNBPOS +1 / OPTDATA (IMU_CALIBRATION_AND_ALIGNMENT.agc p.449) --
 * MKRELEAS, RUN = 0, show target 1
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the P03 job
 *
 * RETURNS: agc_status_t of the V05N30 display
 **********************************************************************/
agc_status_t prelaunch_verify_start(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_sxtmark_mkreleas(); /* BANKCALL MKRELEAS     */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    st = optdata_load(0); /* OPTDATA: CAF BIT1, ZL, LXCH RUN          */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_optical_verification_target_show(ctx));
}

/**********************************************************************
 * OPTDATA +9 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.449) -- GODSPRET
 * V05N30 (target number); the job goes on at the V06N41 flash.  ENTER
 * on that flash comes back here (TC -8D) without reloading DSPTEM1.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the P03 job
 *
 * RETURNS: agc_status_t of GODSPRET
 **********************************************************************/
agc_status_t agc_job_optical_verification_target_show(agc_job_ctx_t *ctx)
{
    const agc_dsky_cont_t next = {AGC_JOB_OPTICAL_VERIFICATION_TARGET_FLASH,
                                  AGC_JOB_OPTICAL_VERIFICATION_TARGET_FLASH,
                                  AGC_JOB_OPTICAL_VERIFICATION_TARGET_FLASH};

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_display(ctx, AGC_DSKY_GODSPRET, VN_TARGET_NUMBER,
                             &next));
}

/**********************************************************************
 * OPTDATA +11 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.449) -- flash V06N41
 * (AZ, EL): V34 GCOMP5, V33 store, ENTER shows V05N30 again
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the P03 job
 *
 * RETURNS: agc_status_t of GOFLASH
 **********************************************************************/
agc_status_t agc_job_optical_verification_target_flash(agc_job_ctx_t *ctx)
{
    const agc_dsky_cont_t cont = {
        AGC_JOB_OPTICAL_VERIFICATION_END,         /* TC GCOMP5 */
        AGC_JOB_OPTICAL_VERIFICATION_TARGET_LOADED, /* TC +3     */
        AGC_JOB_OPTICAL_VERIFICATION_TARGET_SHOW
    };                                              /* TC -8D    */

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT((prelaunch_ram()->run == 0) ||
                    (prelaunch_ram()->run == RUN_TARGET2)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_display(ctx, AGC_DSKY_GOFLASH, VN_TARGET_AZ_EL, &cont));
}

/**********************************************************************
 * RETARG / RETARG1 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.453) -- OPTIND
 * = 0, SXTMARK for one mark, OPTSTALL (wakes the marked job)
 *
 * RETURNS: agc_status_t of SXTMARK or of the stall
 **********************************************************************/
static agc_status_t retarg(void)
{
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING; /* OPTSTALL  */

    agc_erase_set_optind(0); /* RETARG: CAF ZERO, TS OPTIND            */
    agc_status_t st = agc_sxtmark(1); /* CAF ONE, SXTMARK             */

    if (st != AGC_STATUS_OK)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* BAILOUT */
    }

    st = agc_imu_stall_dev(AGC_IMU_DEV_OPTICS,
                           AGC_JOB_OPTICAL_VERIFICATION_MARKED, &res);

    if ((st != AGC_STATUS_OK) || (res == AGC_IMU_STALL_SLEEPING))
    {
        return (st);
    }

    if (!AGC_ASSERT((res == AGC_IMU_STALL_GOOD) || (res == AGC_IMU_STALL_BAD)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_job_yield(AGC_JOB_OPTICAL_VERIFICATION_MARKED));
}

/**********************************************************************
 * TARGDRVE (IMU_CALIBRATION_AND_ALIGNMENT.agc pp.452-453) -- STARCODE,
 * TARG1/2, STAR = target, SXTANG, drive the optics there (DESOPTS,
 * DESOPTT), then RETARG
 *
 * ARGUMENTS:
 *   starcode  agc_sp_t  I  STARCODE: 1 or 2
 *   targ      agc_sp_t  I  TARG1/2: 0 target 1, 6 target 2
 *
 * RETURNS: agc_status_t of SXTANG or of RETARG
 **********************************************************************/
static agc_status_t targdrve(agc_sp_t starcode, agc_sp_t targ)
{
    agc_align_ram_t *ar = agc_align_ram();
    agc_interp_t     ip;               /* interpreter state           */
    agc_vec_t        tar[TARGETS];     /* PD06, PD12: half-unit       */
    bool             cult = false;     /* CULTFLAG (not tested here)  */

    if (!AGC_ASSERT((targ == 0) || (targ == TARG_TARGET2)) ||
        !AGC_ASSERT((starcode == 1) || (starcode == 2)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    ar->starcode          = starcode; /* TS STARCODE                 */
    prelaunch_ram()->targ = targ;     /* QXCH QPLAC, TS TARG1/2      */
    agc_interp_init(&ip);
    agc_status_t st = target_vectors(&ip, tar); /* CALL TAR/EREF       */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ar->star = tar[(targ == 0) ? 0 : 1]; /* VLOAD* 6D,1; STCALL STAR    */
    st       =
        agc_csm_sxtang(&ip, ar->star, &ar->xnb, &ar->sac, &ar->pac, &cult);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ar->desopts = ar->sac; /* CA SAC, TS DESOPTS                      */
    ar->desoptt = ar->pac; /* CA PAC, TS DESOPTT                      */

    return (retarg());
}

/**********************************************************************
 * OPTDATA V33 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.449) -- TAZEL1 +RUN
 * = DSPTEM1, +1; after target 1 show target 2 (RUN = 2), after target
 * 2 CONTIN33: drive to and mark target 1
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the P03 job
 *
 * RETURNS: agc_status_t of the next display or of TARGDRVE
 **********************************************************************/
agc_status_t agc_job_optical_verification_target_loaded(agc_job_ctx_t *ctx)
{
    prelaunch_ram_t *r = prelaunch_ram();

    if (!AGC_ASSERT(ctx != NULL) ||
        !AGC_ASSERT((r->run == 0) || (r->run == RUN_TARGET2)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    r->tazel[r->run]     = agc_dsky_dsptem(0); /* DXCH DSPTEM1        */
    r->tazel[r->run + 1] = agc_dsky_dsptem(1); /* INDEX RUN, DXCH     */

    if (r->run == 0) /* CCS RUN: +0                                   */
    {
        agc_status_t st = optdata_load(RUN_TARGET2); /* TCF OPTDATA +2 */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        return (agc_job_optical_verification_target_show(ctx));
    }

    return (targdrve(1, 0)); /* CONTIN33: STARCODE 1, TARGDRVE 0      */
}

/**********************************************************************
 * NEXTBNKS / NEXBNKSS (IMU_CALIBRATION_AND_ALIGNMENT.agc pp.450-451) --
 * target 1 marked: STARAD = XSM PD06 VSL1, STARAD +6 = XSM PD12 VSL1,
 * LOSVEC = LITTLSUB; MKRELEAS; then drive to target 2 (STARCODE 2,
 * TARGDRVE 6)
 *
 * RETURNS: agc_status_t of LITTLSUB, MKRELEAS or TARGDRVE
 **********************************************************************/
static agc_status_t target1_marked(void)
{
    agc_align_ram_t     *ar = agc_align_ram();
    agc_interp_t         ip;           /* interpreter state           */
    agc_vec_t            tar[TARGETS]; /* PD06, PD12: half-unit       */
    agc_align_sxt_mark_t m;            /* the mark (VAC +2 .. +6)     */

    agc_interp_init(&ip);
    agc_status_t st = target_vectors(&ip, tar); /* CALL TAR/EREF       */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    for (int i = 0; i < TARGETS; i++) /* VLOAD MXV, VSL1, STORE STARAD */
    {
        ar->starad[i] = agc_vec_shl(&ip, agc_mat_vec(&ip, &ar->xsm, tar[i]), 1);
    }

    st = agc_sxtmark_mark(0, &m); /* LITTLSUB: LXC,1 MARKSTAT        */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    st = agc_csm_sxtsm(&ip, &m, &prelaunch_ram()->losvec); /* STORE LOSVEC */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_sxtmark_mkreleas(); /* BANKCALL MKRELEAS                  */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    return (targdrve(2, TARG_TARGET2)); /* NEXBNKSS                    */
}

/**********************************************************************
 * NEXBNKSS +5 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.451) -- target 2
 * marked: 12D = LITTLSUB, 6D = LOSVEC, AXISGEN (STARAD expected, 6D/12D
 * marked), CALCGTA into OGC, IGC, MGC; then GCOMP4
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the P03 job
 *
 * RETURNS: agc_status_t of LITTLSUB, AXISGEN or the V06N93 flash
 **********************************************************************/
static agc_status_t target2_marked(agc_job_ctx_t *ctx)
{
    agc_align_ram_t     *ar = agc_align_ram();
    agc_interp_t         ip;         /* interpreter state             */
    agc_vec_t            marked[2];  /* 6D, 12D: marked LOS, half-unit */
    agc_align_sxt_mark_t m;          /* the mark (VAC +2 .. +6)       */
    agc_mat_t            xdc;        /* XDC..ZDC, half-unit           */

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_init(&ip);
    agc_status_t st = agc_sxtmark_mark(0, &m); /* CALL LITTLSUB        */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    st = agc_csm_sxtsm(&ip, &m, &marked[1]); /* STOVL 12D             */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    marked[0] = prelaunch_ram()->losvec; /* LOSVEC, STCALL 06D        */
    st        = agc_imu_axisgen(&ip, ar->starad, marked, &xdc); /* AXISGEN */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ar->xdc = xdc;
    agc_imu_calcgta(&ip, &xdc, ar->ogc); /* CALL CALCGTA               */

    return (agc_job_optical_verification_torque_display(ctx)); /* GCOMP4 */
}

/**********************************************************************
 * RETARG +4 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.453) -- after OPTSTALL:
 * bad -> GTSOPTCS (ALARM 1602, GCOMP5); TRM03FLG -> GCOMP5; no mark ->
 * RETARG1 (free the VAC, mark again); else back to QPLAC
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the P03 job woken from OPTSTALL
 *
 * RETURNS: agc_status_t of the next step
 **********************************************************************/
agc_status_t agc_job_optical_verification_marked(agc_job_ctx_t *ctx)
{
    const agc_sp_t targ = prelaunch_ram()->targ; /* QPLAC (TARG1/2)   */

    if (!AGC_ASSERT(ctx != NULL) ||
        !AGC_ASSERT((targ == 0) || (targ == TARG_TARGET2)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_imu_stall_outcome(AGC_IMU_DEV_OPTICS) != AGC_IMU_STALL_GOOD)
    {
        agc_alarm(PRELAUNCH_ALARM_OPTICS); /* GTSOPTCS: ALARM 01602   */

        return (agc_job_optical_verification_end(ctx));
    }

    if (agc_flag_test(AGC_FLAG_TRM03FLG))
    {
        return (agc_job_optical_verification_end(ctx)); /* TC GCOMP5 */
    }

    if (agc_sxtmark_marks_made() <= 0) /* INDEX MARKSTAT, CA QPRET    */
    {
        agc_status_t st = agc_sxtmark_free_vac(); /* RETARG1          */

        if (!AGC_ASSERT(st == AGC_STATUS_OK))
        {
            return (st);
        }

        return (retarg()); /* TCF RETARG                             */
    }

    if (targ == 0)
    {
        return (target1_marked()); /* TC QPLAC: NEXTBNKS              */
    }

    return (target2_marked(ctx)); /* TC QPLAC: NEXBNKSS +5            */
}

/**********************************************************************
 * GCOMP4 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.451) -- flash V06N93
 * (OGC, IGC, MGC): V34 GCOMP5, V33 torque, ENTER flash again
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the P03 job
 *
 * RETURNS: agc_status_t of GOFLASH
 **********************************************************************/
agc_status_t agc_job_optical_verification_torque_display(agc_job_ctx_t *ctx)
{
    const agc_dsky_cont_t cont = {
        AGC_JOB_OPTICAL_VERIFICATION_END,             /* TC GCOMP5  */
        AGC_JOB_OPTICAL_VERIFICATION_TORQUE_ACCEPTED, /* TCF +2     */
        AGC_JOB_OPTICAL_VERIFICATION_TORQUE_DISPLAY
    };                                                /* TCF GCOMP4 */

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_display(ctx, AGC_DSKY_GOFLASH, VN_GYRO_ANGLES, &cont));
}

/**********************************************************************
 * GCOMP4 +5 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.451) -- ERCOMP1 = OGC
 * VAD ERCOMP1 (P02 torques it in), then GCOMP5
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the P03 job
 *
 * RETURNS: agc_status_t of GCOMP5
 **********************************************************************/
agc_status_t agc_job_optical_verification_torque_accepted(agc_job_ctx_t *ctx)
{
    const agc_align_ram_t *ar = agc_align_ram();
    agc_interp_t           ip;     /* interpreter state (VAD)         */
    agc_dp_t               buf[3]; /* ERCOMP1, rev 2^0                */

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_imu_gyro_buf_get(AGC_IMU_GBUF_ERCOMP, buf);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    agc_interp_init(&ip);
    const agc_vec_t sum =
        agc_vec_add(&ip, agc_vec_make(ar->ogc[0], ar->ogc[1], ar->ogc[2]),
                    agc_vec_make(buf[0], buf[1], buf[2])); /* VLOAD VAD */

    for (int i = 0; i < 3; i++)
    {
        buf[i] = sum.c[i]; /* STORE ERCOMP1                            */
    }

    agc_imu_gyro_buf_set(AGC_IMU_GBUF_ERCOMP, buf);

    return (agc_job_optical_verification_end(ctx)); /* GCOMP5          */
}

/**********************************************************************
 * GCOMP5 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.451) -- end of P03:
 * MKRELEAS, TRM03FLG down, MM 02, group 4 off, ENDOFJOB
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the P03 job
 *
 * RETURNS: agc_status_t -- OK (ENDOFJOB)
 **********************************************************************/
agc_status_t agc_job_optical_verification_end(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_sxtmark_mkreleas(); /* BANKCALL MKRELEAS     */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    agc_flag_clear(AGC_FLAG_TRM03FLG); /* DOWNFLAG TRM03FLG            */
    st = agc_phase_newmode(PRELAUNCH_MM_COMPASS); /* NEWMODEX MM 02     */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    agc_phase_change(4, PHASE_GROUP4_OFF); /* PHASCHNG OCT 00004       */

    return (AGC_STATUS_OK); /* ENDOFJOB                             */
}
