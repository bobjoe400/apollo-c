/*
 * align_p52.c -- the part of P52 (IMU realign) common to both vehicles:
 * PROG52, the option display P52B, the time display P52T, the nominal
 * orientation S52.3, the gimbal-angle display P52D, the coarse-align
 * choice COARSTYP, CAL53A (coarse/fine align to XSMD) and the gyro-torque
 * coarse align GYCRS / GYCOARS.
 * Sources: P51-P53.agc, Comanche055 pp. 739-744, 761, Luminary099
 * pp. 927-933, 944, 947.  The differences (CM phase and flag clears in
 * PROG52, the option-to-routine map, TALIGN in the LM, the ends of CAL53A
 * and GYCRS) are behind agc_state_vehicle() tests.
 *
 * OPTION2 (OPTION1 +1) holds the alignment option: 1 preferred,
 * 2 nominal, 3 REFSMMAT, 4 landing site (option 4 & 3 = 0).
 */
#include <stdbool.h>
#include <stdint.h>
#include "agc/agc_assert.h"
#include "agc/agc_flags_common.h"
#include "align_internal.h"

#define PH_REALIGN   ((uint16_t)00254)   /* PROG52 (CM): group 4 phase 25    */
#define PH_OPTION_RETURN     ((uint16_t)00014)   /* P52B display return: phase 1     */
#define VN_ALIGN_TIME      ALIGN_VN(6, 34)     /* VN 0634: TALIGN / time           */
#define VN_GIMBAL_ANGLES      ALIGN_VN(6, 22)     /* VN 00622: gimbal angles THETAD   */
#define VN_MONITOR_GIMBALS      ALIGN_VN(16, 20)    /* VN 1620: monitor the gimbals     */
#define OPT_CODE_1  ((uint16_t)1)       /* GOPERF4R with BIT1: V04N06       */
#define CHK_OCT13   ((uint16_t)013)     /* OCT13: coarse align checklist    */
#define OPTION2_PREF ((agc_sp_t)1)      /* BIT1: preferred                  */
#define OPTION2_NOM  ((agc_sp_t)2)      /* BIT2: nominal                    */
#define OPTION_MASK3 3                  /* MASK THREE                       */

static bool is_csm(void)
{
    return agc_state_vehicle() == AGC_VEHICLE_CSM;
}

/* OPTION2 (OPTION1 +1). */
static agc_sp_t option2(void)
{
    const agc_sp_t v = agc_erase_option1(1);
    (void)AGC_ASSERT((v >= 0) && (v <= AGC_SP_MAX));
    return v;
}

/*
 * PROG52 alias P54 (CM) / PROG52 (LM) (Comanche p. 739, Luminary p. 927):
 * CM: restart phase 4.25, clear UPDATFLG and TRACKFLG.  Both: R02BOTH;
 * OPTION2 = preferred if PFRATFLG is set, else nominal; then P52B.
 */
agc_status_t agc_job_imu_realign(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = AGC_STATUS_OK;
    if (is_csm()) {
        st = align_phase4(PH_REALIGN);
        (void)agc_flag_clear(AGC_FLAG_UPDATFLG);  /* status needs no check */
        (void)agc_flag_clear(AGC_FLAG_TRACKFLG);
    }
    st = agc_imu_status_check();                                   /* R02BOTH   */
    if (st != AGC_STATUS_OK) {
        return AGC_STATUS_OK;               /* alarm 210/220, GOTOPOOH done */
    }
    agc_erase_set_option1(1, agc_flag_test(AGC_FLAG_PFRATFLG) ? OPTION2_PREF
                                                              : OPTION2_NOM);
    (void)AGC_ASSERT(option2() != 0);
    return agc_job_imu_realign_option_display(ctx);
}

/*
 * P52B: GOPERF4R (flash V04N06, option code 00001, OPTION2); V34
 * terminates, V33 goes on with the option, ENTER (a new option) re-flashes.
 * The immediate return sets restart phase 4.1 and ends the job.
 */
agc_status_t agc_job_imu_realign_option_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = align_flash(ctx, AGC_DSKY_GOPERF4R, OPT_CODE_1,
                                  AGC_JOB_GOTOPOOH, AGC_JOB_IMU_REALIGN_OPTION,
                                  AGC_JOB_IMU_REALIGN_OPTION_DISPLAY);
    if (st == AGC_STATUS_OK) {
        st = align_phase4(PH_OPTION_RETURN);
    }
    return st;                                               /* ENDOFJOB  */
}

/*
 * P52B +9 (V33 on V04N06): branch on OPTION2 & 3.
 *   CM:  0 (landing site) P52T, 1 P52J -> P52D, 2 P52T, 3 P52C.
 *   LM:  0 OPT4 (TLAND shown in VN_ALIGN_TIME), 1 P52H -> P52D, 2 P52T,
 *        3 P52E -> P52F (R51, then P52OUT = GOTOPOOH).
 */
agc_status_t agc_job_imu_realign_option(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    const int opt = (int)(option2() & OPTION_MASK3);
    (void)AGC_ASSERT((opt >= 0) && (opt <= OPTION_MASK3));
    if (opt == 1) {
        return agc_job_realign_gimbal_display(ctx);                       /* P52J / P52H    */
    }
    if (opt == 2) {
        return agc_job_realign_time(ctx);
    }
    if (is_csm()) {
        return (opt == 0) ? agc_job_realign_time(ctx) : agc_job_csm_realign_star_selection(ctx);
    }
    if (opt == 0) {                                     /* OPT4           */
        const agc_status_t st = align_set_dsptem1(agc_erase_tland());
        return (st == AGC_STATUS_OK) ? agc_job_realign_time_flash(ctx) : st;
    }
    agc_align_ram()->qmaj = AGC_JOB_GOTOPOOH;           /* P52OUT         */
    return agc_job_lm_fine_align(ctx);                         /* P52E, P52F     */
}

/* P52T: DSPTEM1 = -0 (0 here: both tests treat +-0 alike), then VN_ALIGN_TIME. */
agc_status_t agc_job_realign_time(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_status_t st = align_set_dsptem1(0);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return agc_job_realign_time_flash(ctx);
}

/* P52T +3: flash VN_ALIGN_TIME (time of alignment); ENTER re-flashes. */
agc_status_t agc_job_realign_time_flash(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(option2() >= 0);
    return align_flash(ctx, AGC_DSKY_GOFLASH, VN_ALIGN_TIME, AGC_JOB_GOTOPOOH,
                       AGC_JOB_REALIGN_TIME_PROCEED, AGC_JOB_REALIGN_TIME_FLASH);
}

/*
 * P52T +10 (V33): a zero time means now.  CM keeps the time in DSPTEM1;
 * LM: TALIGN = DSPTEM1, or TIME2/1 if it is zero or negative (P52V).
 * OPTION2 bit 2 (nominal) -> S52.3, else the landing site (P52LS).
 */
agc_status_t agc_job_realign_time_proceed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_dp_t t = align_dsptem1();
    const bool nominal = (option2() & OPTION2_NOM) != 0;
    if (is_csm()) {
        if (t == 0) {                                       /* BZF       */
            const agc_status_t st = align_set_dsptem1(align_loadtime());
            (void)AGC_ASSERT(st == AGC_STATUS_OK);
        }
        return nominal ? agc_job_realign_nominal_orientation(ctx) : agc_job_csm_imu_realign_landing_site(ctx);
    }
    agc_align_ram()->talign = (t <= 0) ? align_loadtime() : t; /* BZMF   */
    (void)AGC_ASSERT(agc_align_ram()->talign >= 0);
    return nominal ? agc_job_realign_nominal_orientation(ctx) : agc_job_lm_imu_realign_landing_site(ctx);
}

/*
 * CALL S52.3 (nominal orientation at DSPTEM1 (CM) / TALIGN (LM)), then
 * P52D.  The integration may put the job to sleep in INTSTALL; it then
 * resumes here.
 */
agc_status_t agc_job_realign_nominal_orientation(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_align_ram_t *ram = agc_align_ram();
    const agc_dp_t t = is_csm() ? align_dsptem1() : ram->talign;
    agc_interp_t ip;
    agc_interp_init(&ip);
    const agc_status_t st = agc_align_nominal_orientation(&ip, t, AGC_JOB_REALIGN_NOMINAL_ORIENTATION, &ram->xsmd);
    if (st == AGC_STATUS_BUSY) {
        return AGC_STATUS_OK;                         /* INTSTALL sleep   */
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_job_realign_gimbal_display(ctx);
}

/* CALL S52.2 on the present CDUs: XSM = XSMD, THETAD, XNB (reference). */
static agc_status_t realign_gimbal_angles(void)
{
    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t ip;
    agc_interp_init(&ip);
    uint16_t cdu[3];
    align_read_cdus(cdu);
    const agc_mat_t refsmmat = agc_erase_refsmmat();
    uint16_t thetad[3];
    const agc_status_t st = agc_align_gimbal_angles(&ip, &ram->xsmd, &refsmmat, cdu,
                                            &ram->xsm, &ram->xnb, thetad);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return st;
}

/* P52D: S52.2, flash VN_GIMBAL_ANGLES; V33 COARSTYP, ENTER (vehicle moved) P52D. */
agc_status_t agc_job_realign_gimbal_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_status_t st = realign_gimbal_angles();
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return align_flash(ctx, AGC_DSKY_GOFLASH, VN_GIMBAL_ANGLES, AGC_JOB_GOTOPOOH,
                       AGC_JOB_COARSTYP, AGC_JOB_REALIGN_GIMBAL_DISPLAY);
}

/* COARSTYP: V50N25 R1 00013; V33 normal coarse (CAL53A), V32 gyro torque. */
agc_status_t agc_job_coarstyp(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(option2() >= 0);
    return align_flash(ctx, AGC_DSKY_GOPERF1, CHK_OCT13, AGC_JOB_GOTOPOOH,
                       AGC_JOB_COARSE_ALIGN, AGC_JOB_GYCRS);
}

/*
 * CAL53A (Comanche p. 761, Luminary p. 947): a last S52.2, read the CDUs;
 * if any gimbal is more than 1 degree from THETAD (CALOOP), restart phase
 * 4.2 and COARSE, then (CM) SET1/PDT, GCOMP = 0, DRIFTFLG / (LM) NCOARSE;
 * in either case FINEONLY.
 */
agc_status_t agc_job_coarse_align(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = realign_gimbal_angles();
    if (st != AGC_STATUS_OK) {
        return st;
    }
    uint16_t cdu[3];
    uint16_t thetad[3];
    align_read_cdus(cdu);                                     /* RDCDUS    */
    for (int i = 0; i < 3; i++) {
        thetad[i] = agc_erase_thetad(i);
    }
    if (!agc_align_coarse_needed(thetad, cdu)) {
        return agc_job_coarse_align_fineonly(ctx);                  /* CALOOP1   */
    }
    st = align_phase_job(ALIGN_PH_04024, AGC_JOB_COARSE_ALIGN, ctx->prio); /* COARFINE */
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return align_coarse(AGC_JOB_COARSE_ALIGN_COARSED);
}

/* After COARSE in CAL53A: drift compensation restarted, then FINEONLY. */
agc_status_t agc_job_coarse_align_coarsed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_status_t st = is_csm() ? align_set1_pdt() : align_ncoarse();
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return agc_job_coarse_align_fineonly(ctx);
}

/*
 * FINEONLY: REFSMMAT = XSM (MATMOVE).  CM CAL53RET: SET REFSMFLG, P52C.
 * LM COARSRET: SET REFSMFLG, CLEAR PFRATFLG, P52F (R51, then P52OUT).
 */
agc_status_t agc_job_coarse_align_fineonly(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_align_ram_t *ram = agc_align_ram();
    agc_erase_set_refsmmat(ram->xsm);                         /* MATMOVE   */
    agc_status_t st = agc_flag_set(AGC_FLAG_REFSMFLG);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    if (is_csm()) {
        return agc_job_csm_realign_star_selection(ctx);
    }
    st = agc_flag_clear(AGC_FLAG_PFRATFLG);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    ram->qmaj = AGC_JOB_GOTOPOOH;                             /* P52OUT    */
    return agc_job_lm_fine_align(ctx);
}

/*
 * GYCRS (Comanche p. 741) / COARSTYP V32 + GYCOARS (Luminary pp. 931,
 * 944): XDC.. = UNIT(REFSMMAT XSMD..), the desired SM wrt the present SM;
 * CALCGTA; CLEAR DRIFTFLG, REFSMFLG; monitor VN_MONITOR_GIMBALS (GODSPR); torque.
 */
agc_status_t agc_job_gycrs(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t ip;
    agc_interp_init(&ip);
    const agc_mat_t refsmmat = agc_erase_refsmmat();
    agc_status_t st = agc_align_gyro_coarse_dc(&ip, &ram->xsmd, &refsmmat, &ram->xdc);
    if (st == AGC_STATUS_OK) {
        st = agc_imu_calcgta(&ip, &ram->xdc, ram->ogc);        /* CALCGTA   */
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    (void)agc_flag_clear(AGC_FLAG_DRIFTFLG);        /* flag ids are valid  */
    (void)agc_flag_clear(AGC_FLAG_REFSMFLG);
    st = align_show(ctx, AGC_DSKY_GODSPR, VN_MONITOR_GIMBALS);             /* GODSPR    */
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return agc_job_gycrs_pulse(ctx);
}

/* GYCRS: IMUPULSE (R55CDR = OGC) and IMUSTALL; resumed if gyros busy. */
agc_status_t agc_job_gycrs_pulse(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_imu_pulse_buf(AGC_IMU_GBUF_OGC, AGC_JOB_GYCRS_PULSE);
    if (st == AGC_STATUS_BUSY) {
        return AGC_STATUS_OK;
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    bool done = false;
    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_GYCRS_PULSED, &done);
    if ((st != AGC_STATUS_OK) || !done) {
        return (st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st;
    }
    return agc_job_gycrs_torqued(ctx);
}

/* GYCRS: woken from the IMUSTALL. */
agc_status_t agc_job_gycrs_pulsed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (align_stall_outcome(AGC_IMU_DEV_IMU) != AGC_STATUS_OK) {
        return AGC_STATUS_OK;                                 /* CURTAINS  */
    }
    return agc_job_gycrs_torqued(ctx);
}

/*
 * GYCRS after torquing: phase 4.2; REFSMMAT = XSMD; CLEAR PFRATFLG, SET
 * REFSMFLG; CM SET1/PDT, GCOMP = 0, DRIFTFLG / LM NCOARSE; GOTO R51K
 * (the LM returns from R51K's ENTER to QMAJ = P52OUT).
 */
agc_status_t agc_job_gycrs_torqued(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_align_ram_t *ram = agc_align_ram();
    agc_status_t st = align_phase_job(ALIGN_PH_04024, AGC_JOB_GYCRS_TORQUED, ctx->prio);
    agc_erase_set_refsmmat(ram->xsmd);                        /* MATMOVE   */
    (void)agc_flag_clear(AGC_FLAG_PFRATFLG);        /* flag ids are valid  */
    st = agc_flag_set(AGC_FLAG_REFSMFLG);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    st = is_csm() ? align_set1_pdt() : align_ncoarse();
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    if (is_csm()) {
        return agc_job_csm_fine_align_recheck(ctx);
    }
    ram->qmaj = AGC_JOB_GOTOPOOH;                             /* P52OUT    */
    return agc_job_lm_fine_align_recheck(ctx);
}
