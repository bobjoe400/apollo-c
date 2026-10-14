/**********************************************************************
 * FILE NAME: imu_realign.c
 *
 * PURPOSE:
 *   The LM-only part of P52 IMU realign: the landing-site orientation
 *   P52LS (option 4) and N89DISP, the landing-site latitude / longitude /
 *   altitude display also used by P57.
 *
 * AGC SOURCE:
 *   Luminary099 P51-P53.agc pp. 932-933 (P52LS, N89DISP, LSDISP)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   TALIGN, ALPHAV, LANDLAT, LANDLONG, LANDALT, XSMD, QMAJ: agc_align_ram()
 *   RLS           agc_vec_t I    m 2^27         landing site (moon fixed)
 *   TLAND         agc_dp_t  O    cs 2^28        option 4 time
 *   LUNAFLAG, ERADFLAG  flags (I/O)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_*           alignment routines of this lane (alignment.h)
 *   agc_planet_rp_to_r    RP-TO-R / R-TO-RP (auxiliary.h)
 *   agc_latlong_*         LAT-LONG / LALOTORV (auxiliary.h)
 *   agc_navaux_get_padload  pad loads (auxiliary.h)
 *   agc_vec_*             vector arithmetic of the interpreter (interp.h)
 *   agc_dp_*              DP arithmetic of the interpreter (interp.h, word.h)
 *   agc_interp_init       interpreter state: OVFIND (interp.h)
 *   agc_flag_*            flag words (flags_common.h)
 *   agc_erase_*           erasable accessors (state.h)
 *   agc_job_realign_gimbal_display  see its header
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/flags_common.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"
#include "internal.h"

#define VN_LANDING_SITE ALIGN_VN(6, 89) /* V06N89* VN 689                */

/**********************************************************************
 * AGC: none (flag test for LAT-LONG, LALOTORV)
 *
 * LAT-LONG / LALOTORV options from LUNAFLAG and ERADFLAG.
 *
 * FUNCTION NAME: latlong_options
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_latlong_opt_t -- the options (lunar, Fischer radius)
 **********************************************************************/
static agc_latlong_opt_t latlong_options(void)
{
    const agc_latlong_opt_t opt = {.lunar = agc_flag_test(AGC_FLAG_LUNAFLAG), .erad = agc_flag_test(AGC_FLAG_ERADFLAG)};

    return (opt);
}

/**********************************************************************
 * N89DISP (Luminary p. 933): STQ QMAJ; GDT/2 +4 = time; LAT-LONG of ALPHAV
 * (m B29); LANDLONG = LONG/2, LANDALT = ALT, LANDLAT = LAT; then LSDISP.
 * ret is the job continued after V33 (QMAJ), with the new site (LALOTORV, m
 * B29) in agc_align_ram()->alphav.
 *
 * FUNCTION NAME: agc_lm_landing_site_latlong
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *   t         agc_dp_t         I    time, cs 2^28
 *   ret       agc_job_id_t     I    job the routine continues as when it
 *                                   ends (the AGC return address)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_lm_landing_site_latlong(agc_job_ctx_t *ctx, agc_dp_t t, agc_job_id_t ret)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    ram->qmaj              = ret; /* STQ QMAJ */
    ram->landing_site_time = t;   /* GDT/2 +4 */
    agc_interp_t ip;              /* interpreter state */

    agc_interp_init(&ip);
    agc_navaux_padload_t pad; /* pad loads (ephemeris, RLS) */
    agc_latlong_t        ll;  /* latitude, longitude, altitude */

    agc_navaux_get_padload(&pad);
    agc_status_t st = agc_latlong_from_r(&ip, &pad, latlong_options(), ram->alphav, t, &ll); /* status of LAT-LONG from R */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ram->landlong = agc_dp_shr(ll.lon, 1); /* SR1      */
    ram->landalt  = ll.alt;
    ram->landlat  = ll.lat;

    return (agc_job_lm_landing_site_display(ctx));
}

/**********************************************************************
 * AGC: LSDISP (Luminary099 P51-P53.agc p. 933)
 *
 * LSDISP: flash V06N89; V34 terminates (exits P57), V33 accepts, V32 or
 * ENTER looks again (or loads a new site).
 *
 * FUNCTION NAME: agc_job_lm_landing_site_display
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
agc_status_t agc_job_lm_landing_site_display(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_LANDING_SITE, AGC_JOB_GOTOPOOH, AGC_JOB_LM_LANDING_SITE_ACCEPTED, AGC_JOB_LM_LANDING_SITE_DISPLAY));
}

/**********************************************************************
 * AGC: LSDISP (Luminary099 P51-P53.agc p. 933)
 *
 * LSDISP V33: LONG = 2 LANDLONG, ALT = LANDALT, LAT = LANDLAT; LALOTORV at
 * GDT/2 +4 -> ALPHAV (RLS, m B29); GOTO QMAJ.
 *
 * FUNCTION NAME: agc_job_lm_landing_site_accepted
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_lm_landing_site_accepted(void)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    agc_latlong_t ll; /* latitude, longitude, altitude */

    ll.lon   = agc_dp_shl(&ip, ram->landlong, 1); /* SL1      */
    ll.alt   = ram->landalt;
    ll.lat   = ram->landlat;
    ll.eradm = 0;
    agc_navaux_padload_t pad;                     /* pad loads (ephemeris, RLS) */

    agc_navaux_get_padload(&pad);
    agc_status_t st = agc_latlong_to_r(&ip, &pad, latlong_options(), &ll, ram->landing_site_time, &ram->alphav, NULL); /* status of R from LAT-LONG */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (align_goto(ram->qmaj)); /* GOTO QMAJ */
}

/**********************************************************************
 * P52LS (Luminary p. 932): SET LUNAFLAG, CLEAR ERADFLAG (|RLS| is the
 * radius); TLAND = TALIGN (option 4 time); ALPHAV = RP-TO-R(RLS) VSR2 (m
 * B29); N89DISP at TALIGN.
 *
 * FUNCTION NAME: agc_job_lm_imu_realign_landing_site
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
agc_status_t agc_job_lm_imu_realign_landing_site(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    agc_flag_set(AGC_FLAG_LUNAFLAG);
    agc_flag_clear(AGC_FLAG_ERADFLAG);
    agc_erase_set_tland(ram->talign); /* TLAND    */
    agc_interp_t ip;                  /* interpreter state */

    agc_interp_init(&ip);
    agc_navaux_padload_t pad;                /* pad loads (ephemeris, RLS) */
    agc_vec_t            r = agc_vec_zero(); /* RP-TO-R of RLS, m 2^27 */

    agc_navaux_get_padload(&pad);
    agc_status_t st = agc_planet_rp_to_r(&ip, &pad, true, agc_erase_rls(), ram->talign, &r); /* status of RP-TO-R */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ram->alphav = agc_vec_shr(r, 2); /* VSR2     */

    return (agc_lm_landing_site_latlong(ctx, ram->talign, AGC_JOB_LM_REALIGN_SITE_ORIENTATION));
}

/**********************************************************************
 * AGC: P52LS (Luminary099 P51-P53.agc p. 932)
 *
 * P52LS after N89DISP: XSMD = UNIT(ALPHAV); LSORIENT with the CSM's
 * RRECTCSM, VRECTCSM; then P52D (gimbal angles).
 *
 * FUNCTION NAME: agc_job_lm_realign_site_orientation
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
agc_status_t agc_job_lm_realign_site_orientation(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    const agc_vec_t xsmd = agc_vec_unit(&ip, ram->alphav, NULL, NULL); /* XSMD = UNIT(ALPHAV), half-unit (2^1) */
    agc_align_lsorient(&ip, xsmd, agc_erase_rrectcsm(), agc_erase_vrectcsm(), &ram->xsmd);

    return (agc_job_realign_gimbal_display(ctx)); /* P52D     */
}
