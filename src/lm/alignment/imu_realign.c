/*
 * imu_realign.c -- the LM-only part of P52 IMU realign: the landing-site
 * orientation P52LS (option 4) and N89DISP, the landing-site latitude /
 * longitude / altitude display also used by P57.
 * Source: P51-P53.agc, Luminary099 pp. 932-933 (P52LS, N89DISP, LSDISP).
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment_internal.h"

#define VN_LANDING_SITE ALIGN_VN(6, 89) /* V06N89* VN 689                */

static agc_latlong_opt_t latlong_options(void)
{
    const agc_latlong_opt_t opt = {.lunar = agc_flag_test(AGC_FLAG_LUNAFLAG),
                                   .erad  = agc_flag_test(AGC_FLAG_ERADFLAG)};

    return (opt);
}

/*
 * N89DISP (Luminary p. 933): STQ QMAJ; GDT/2 +4 = time; LAT-LONG of
 * ALPHAV (m B29); LANDLONG = LONG/2, LANDALT = ALT, LANDLAT = LAT; then
 * LSDISP.  ret is the job continued after V33 (QMAJ), with the new site
 * (LALOTORV, m B29) in agc_align_ram()->alphav.
 */
agc_status_t agc_lm_landing_site_latlong(agc_job_ctx_t *ctx, agc_dp_t t,
                                         agc_job_id_t ret)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(ret != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();

    ram->qmaj              = ret; /* STQ QMAJ */
    ram->landing_site_time = t;   /* GDT/2 +4 */
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_navaux_padload_t pad;
    agc_latlong_t        ll;
    agc_status_t         st = agc_navaux_get_padload(&pad);

    if (st == AGC_STATUS_OK)
    {
        st = agc_latlong_from_r(&ip, &pad, latlong_options(), ram->alphav, t,
                                &ll);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ram->landlong = agc_dp_shr(ll.lon, 1); /* SR1      */
    ram->landalt  = ll.alt;
    ram->landlat  = ll.lat;

    return (agc_job_lm_landing_site_display(ctx));
}

/* LSDISP: flash V06N89; V34 terminates (exits P57), V33 accepts, V32 or
 * ENTER looks again (or loads a new site).                               */
agc_status_t agc_job_lm_landing_site_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_align_ram()->qmaj != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_LANDING_SITE,
                        AGC_JOB_GOTOPOOH,
                        AGC_JOB_LM_LANDING_SITE_ACCEPTED,
                        AGC_JOB_LM_LANDING_SITE_DISPLAY));
}

/*
 * LSDISP V33: LONG = 2 LANDLONG, ALT = LANDALT, LAT = LANDLAT; LALOTORV at
 * GDT/2 +4 -> ALPHAV (RLS, m B29); GOTO QMAJ.
 */
agc_status_t agc_job_lm_landing_site_accepted(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);
    agc_latlong_t ll;

    ll.lon   = agc_dp_shl(&ip, ram->landlong, 1); /* SL1      */
    ll.alt   = ram->landalt;
    ll.lat   = ram->landlat;
    ll.eradm = 0;
    agc_navaux_padload_t pad;
    agc_status_t         st = agc_navaux_get_padload(&pad);

    if (st == AGC_STATUS_OK)
    {
        st = agc_latlong_to_r(&ip, &pad, latlong_options(), &ll,
                              ram->landing_site_time, &ram->alphav, NULL);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (align_goto(ram->qmaj)); /* GOTO QMAJ */
}

/*
 * P52LS (Luminary p. 932): SET LUNAFLAG, CLEAR ERADFLAG (|RLS| is the
 * radius); TLAND = TALIGN (option 4 time); ALPHAV = RP-TO-R(RLS) VSR2
 * (m B29); N89DISP at TALIGN.
 */
agc_status_t agc_job_lm_imu_realign_landing_site(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) ||
        !AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();

    agc_flag_set(AGC_FLAG_LUNAFLAG);
    agc_flag_clear(AGC_FLAG_ERADFLAG);
    agc_erase_set_tland(ram->talign); /* TLAND    */
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_navaux_padload_t pad;
    agc_vec_t            r  = agc_vec_zero();
    agc_status_t         st = agc_navaux_get_padload(&pad);

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_rp_to_r(&ip, &pad, true, agc_erase_rls(), ram->talign,
                                &r);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ram->alphav = agc_vec_shr(r, 2); /* VSR2     */

    return (agc_lm_landing_site_latlong(ctx, ram->talign,
                                        AGC_JOB_LM_REALIGN_SITE_ORIENTATION));
}

/*
 * P52LS after N89DISP: XSMD = UNIT(ALPHAV); LSORIENT with the CSM's
 * RRECTCSM, VRECTCSM; then P52D (gimbal angles).
 */
agc_status_t agc_job_lm_realign_site_orientation(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);
    const agc_vec_t    xsmd = agc_vec_unit(&ip, ram->alphav, NULL, NULL);
    const agc_status_t st   = agc_align_lsorient(
        &ip, xsmd, agc_erase_rrectcsm(), agc_erase_vrectcsm(), &ram->xsmd);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_realign_gimbal_display(ctx)); /* P52D     */
}
