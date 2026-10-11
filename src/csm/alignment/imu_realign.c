/*
 * imu_realign.c -- the CSM-only parts of P52 IMU realign (and P54, its
 * backup, PROG52's alias): the star selection P52C/P52E/P52I and the
 * landing-site orientation P52LS / LSDISP.
 * Source: P51-P53.agc, Comanche055 pp. 740-743.  The front end common to
 * both vehicles (PROG52 .. P52D, COARSTYP, CAL53A, GYCRS) is in
 * src/programs/alignment/imu_realign.c.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"

#define PH_STAR_SELECT  ((uint16_t) 04024)        /* P52C PHASCHNG: group 4.2      */
#define CHK_ACQUIRE     ((uint16_t) 015)          /* ALRM15 = OCT15: V50N25 00015  */
#define TSIGHT1_CS      ((agc_dp_t) 24000)        /* TSIGHT1 2DEC 24000: 4 min     */
#define ALM_NO_STARS    ((agc_alarm_code_t) 0405) /* OCT 405: no star pair     */
#define VN_ALARM        ALIGN_VN(5, 9)            /* V05N09                        */
#define VN_LANDING_SITE ALIGN_VN(6, 89)           /* V06N89* VN 0689               */

static bool is_csm(void)
{
    return (agc_state_vehicle() == AGC_VEHICLE_CSM);
}

/*
 * P52C: PHASCHNG 04024; V50N25 R1 00015 (perform star acquisition):
 * V34 terminates, V33 selects the stars (P52E), ENTER goes straight to
 * fine align with the stars the crew chooses (P52F = R51).
 */
agc_status_t agc_job_csm_realign_star_selection(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(is_csm()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_status_t st = align_phase4(PH_STAR_SELECT);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_GOPERF1, CHK_ACQUIRE, AGC_JOB_GOTOPOOH, AGC_JOB_CSM_REALIGN_PICK_STARS, AGC_JOB_CSM_FINE_ALIGN));
}

/*
 * LOCSAM at TSIGHT, then PICAPAR (P52E): a pair found goes to R51 (P52F);
 * none: alarm 405 and V05N09 (PROCEED fine align anyway, ENTER = the
 * vehicle was maneuvered: P52C again).
 */
static agc_status_t realign_pick_stars(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);
    agc_status_t st = agc_align_locsam(&ip, ram->tsight, AGC_JOB_CSM_REALIGN_STARS_LOCATED, &ram->locsam);

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* INTSTALL */
    }

    uint16_t cdu[3];

    align_read_cdus(cdu);
    const agc_mat_t refsmmat = agc_erase_refsmmat();
    bool            found    = false;

    if (st == AGC_STATUS_OK)
    {
        st = agc_align_picapar(&ip, &ram->locsam, &refsmmat, cdu, &ram->besti, &ram->bestj, &found);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (found)
    {
        return (agc_job_csm_fine_align(ctx)); /* P52F     */
    }

    agc_alarm(ALM_NO_STARS); /* P52I     */

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_ALARM, AGC_JOB_GOTOPOOH, AGC_JOB_CSM_FINE_ALIGN, AGC_JOB_CSM_REALIGN_STAR_SELECTION));
}

/* P52C V33: TSIGHT = now + TSIGHT1 (4 min), CALL LOCSAM. */
agc_status_t agc_job_csm_realign_pick_stars(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_align_ram()->tsight = agc_dp_add(&ip, align_loadtime(), TSIGHT1_CS);

    if (!AGC_ASSERT(agc_align_ram()->tsight >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (realign_pick_stars(ctx));
}

/* LOCSAM resumed after its integration wait. */
agc_status_t agc_job_csm_realign_stars_located(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_align_ram()->tsight >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (realign_pick_stars(ctx));
}

/* LLASRD (P20-P25.agc p. 611): N89 registers from LAT, LONG, ALT. */
static void landmark_data_from_latlong(const agc_latlong_t *ll)
{
    agc_align_ram_t *ram = agc_align_ram();

    if (!AGC_ASSERT(ll != NULL))
    {
        return;
    }

    ram->landalt  = ll->alt;                /* LANDALT  */
    ram->landlong = agc_dp_shr(ll->lon, 1); /* SR1      */
    ram->landlat  = ll->lat;

    if (!AGC_ASSERT((ram->landlat >= -AGC_DP_QUARTER) && (ram->landlat <= AGC_DP_QUARTER)))
    {
        return;
    }
}

/*
 * P52LS (Comanche p. 742): STQ QMAJ (= P52D), SET LUNAFLAG; TSIGHT =
 * DSPTEM1; SET ERADFLAG; ALPHAV = RP-TO-R(RLS) VSR2 (m B29); LAT-LONG;
 * LLASRD; then LSDISP.
 */
agc_status_t agc_job_csm_imu_realign_landing_site(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(is_csm()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();

    ram->qmaj   = AGC_JOB_REALIGN_GIMBAL_DISPLAY; /* QMAJ     */
    agc_flag_set(AGC_FLAG_LUNAFLAG);
    agc_flag_set(AGC_FLAG_ERADFLAG);
    ram->tsight = align_dsptem1();
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_navaux_padload_t pad;
    agc_status_t         st = agc_navaux_get_padload(&pad);
    agc_vec_t            r  = agc_vec_zero();

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_rp_to_r(&ip, &pad, true, agc_erase_rls(), ram->tsight, &r);
    }

    const agc_vec_t         alphav = agc_vec_shr(r, 2); /* VSR2     */
    const agc_latlong_opt_t opt    = {.lunar = true, .erad = true};
    agc_latlong_t           ll;

    if (st == AGC_STATUS_OK)
    {
        st = agc_latlong_from_r(&ip, &pad, opt, alphav, ram->tsight, &ll);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    landmark_data_from_latlong(&ll); /* LLASRD   */

    return (agc_job_csm_landing_site_display(ctx));
}

/* LSDISP: flash V06N89 (LAT, LONG/2, ALT); ENTER shows it again. */
agc_status_t agc_job_csm_landing_site_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_align_ram()->qmaj != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_LANDING_SITE, AGC_JOB_GOTOPOOH, AGC_JOB_CSM_LANDING_SITE_ACCEPTED, AGC_JOB_CSM_LANDING_SITE_DISPLAY));
}

/*
 * CSMPREC to TSIGHT; ZSMD = UNIT((RATT x VATT) x XSMD), YSMD = UNIT(ZSMD x
 * XSMD); GOTO QMAJ.
 */
static agc_status_t landing_site_axes(void)
{
    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);
    agc_integ_out_t o;
    agc_status_t    st = align_integ(&ip, ALIGN_CSMPREC, ram->tsight, AGC_JOB_CSM_LANDING_SITE_AXES, &o);

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* INTSTALL */
    }

    if (st == AGC_STATUS_OK)
    {
        st = agc_align_lsorient(&ip, ram->xsmd.row[0], o.ratt, o.vatt, &ram->xsmd);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (align_goto(ram->qmaj)); /* GOTO QMAJ */
}

/*
 * LSDISP V33: LLASRDA (LAT = LANDLAT, LONG = 2 LANDLONG, ALT = LANDALT);
 * LALOTORV at TSIGHT; XSMD = UNIT(ALPHAV); then the other two axes.
 */
agc_status_t agc_job_csm_landing_site_accepted(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);
    agc_latlong_t ll;

    ll.lat                         = ram->landlat;
    ll.lon                         = agc_dp_shl(&ip, ram->landlong, 1); /* SL1      */
    ll.alt                         = ram->landalt;
    ll.eradm                       = 0;
    const agc_latlong_opt_t opt    = {.lunar = agc_flag_test(AGC_FLAG_LUNAFLAG),.erad = agc_flag_test(AGC_FLAG_ERADFLAG)};
    agc_navaux_padload_t    pad;
    agc_status_t            st     = agc_navaux_get_padload(&pad);
    agc_vec_t               alphav = agc_vec_zero();

    if (st == AGC_STATUS_OK)
    {
        st = agc_latlong_to_r(&ip, &pad, opt, &ll, ram->tsight, &alphav, NULL);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ram->xsmd.row[0] = agc_vec_unit(&ip, alphav, NULL, NULL); /* XSMD     */

    return (landing_site_axes());
}

/* CSMPREC resumed after its integration wait. */
agc_status_t agc_job_csm_landing_site_axes(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_align_ram()->qmaj != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (landing_site_axes());
}
