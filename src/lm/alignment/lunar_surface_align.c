/*
 * lunar_surface_align.c -- P57 lunar surface align: the option logic,
 * the star / attitude / gravity line-of-sight pairing and the end of the
 * program.  P51-P53.agc, Luminary099 pp. 966-983 (labels P57, P57OPT,
 * ALIGNOPT, TDISP, P57AA, P57C, P57D, PACKOPTN, DSPOPTN, GETLMATT,
 * BYLMATT, ATTCHK, P57JUMP, P57OPT0-3, SAMETYP, SURFLINE, WRTDESIR,
 * DOALIGN, INITBY, 5DEGTEST, SURFDISP, B2F8, P57POST, MFREF, REFMF).
 * The gravity determination and the IMU torquing are in
 * lunar_surface_gravity.c, the star sightings (R59) in
 * lunar_surface_sighting.c.
 *
 * OPTION2 (OPTION1 +1): 0 = use the stored attitude (YNBSAV, ZNBSAV,
 * moon fixed), 1 = landing site and gravity, 2 = two stars, 3 = one star
 * and gravity.  Bit 1 (options 1, 3) asks for a gravity determination.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"

#define PH_SURFACE_ALIGN ((uint16_t) 04024)   /* OCT 04024                      */
#define PH_OPTION        ((uint16_t) 00014)   /* OCT 00014                      */
#define OPT_CODE_1       ((uint16_t) 1)       /* CAF BIT1 for GOPERF4R          */
#define OPTION_MASK      ((agc_sp_t) 3)       /* MASK THREE                     */
#define OPTION_GRAVITY   ((agc_sp_t) 1)       /* BIT1: options 1 and 3          */
#define OPTION_STARS     ((agc_sp_t) 2)       /* BIT2: options 2 and 3          */
#define OPTION_SITE      ((agc_sp_t) 2)       /* option 2: compute the site     */
#define OPTION1_CHECK    ((agc_sp_t) 010)     /* BIT4 in OPTION1: checklist     */
#define OPTION3_REFSM    ((agc_sp_t) 0100)    /* BIT7: REFSMFLG set             */
#define OPTION3_ATT      ((agc_sp_t) 010)     /* BIT4: ATTFLAG set              */
#define CHK_POST         ((uint16_t) 014)     /* OCT14                          */
#define ALM_OPTION       ((agc_alarm_code_t) 0701)
#define VN_ALIGN_TIME    ALIGN_VN(6, 34)      /* V06N34*                        */
#define VN_OPTIONS       ALIGN_VN(5, 6)       /* VB05N06                        */
#define VN_ALARM         ALIGN_VN(5, 9)       /* VB05N09                        */
#define VN_TORQUE        ALIGN_VN(6, 93)      /* DISPGYRO VN 0693               */
#define FIVE_DEGREES     ((agc_dp_t) 3728270) /* 5DEGREES 2DEC .013888889 revs */

static agc_sp_t option2(void)
{
    const agc_sp_t v = agc_erase_option1(1);

    if (!AGC_ASSERT((v >= 0) && (v <= AGC_SP_MAX)))
    {
        return (0); /* no valid result */
    }

    return (v);
}

static agc_status_t surface_align_phase(void)
{
    const agc_status_t st = align_phase4(PH_SURFACE_ALIGN);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/*
 * MFREF (p. 980): TSIGHT = now; VEC1, VEC2 from moon-fixed to reference
 * axes (RP-TO-R).
 */
agc_status_t agc_align_mfref(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();

    ram->tsight               = align_loadtime();
    agc_navaux_padload_t pad;
    agc_status_t         st   = agc_navaux_get_padload(&pad);
    const bool           moon = agc_flag_test(AGC_FLAG_LUNAFLAG);

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_rp_to_r(ip, &pad, moon, ram->vec1, ram->tsight, &ram->vec1);
    }

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_rp_to_r(ip, &pad, moon, ram->vec2, ram->tsight, &ram->vec2);
    }

    return (st);
}

/*
 * REFMF (p. 980): CDUTRIG, TSIGHT = now, CALCSMSC; YNBSAV, ZNBSAV =
 * R-TO-RP(UNIT(YNB / ZNB VXM REFSMMAT)) (moon fixed); SET ATTFLAG.
 */
agc_status_t agc_align_refmf(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();
    agc_cdutrig_t    t;
    agc_status_t     st  = align_nb_present(ip, &t, &ram->xnb);

    ram->tsight                   = align_loadtime();
    const agc_mat_t      refsmmat = agc_erase_refsmmat();
    agc_navaux_padload_t pad;

    if (st == AGC_STATUS_OK)
    {
        st = agc_navaux_get_padload(&pad);
    }

    const bool      moon = agc_flag_test(AGC_FLAG_LUNAFLAG);
    const agc_vec_t y    = agc_vec_unit(ip, agc_vec_mat(ip, ram->xnb.row[1], &refsmmat), NULL, NULL);
    const agc_vec_t z    = agc_vec_unit(ip, agc_vec_mat(ip, ram->xnb.row[2], &refsmmat), NULL, NULL);

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_r_to_rp(ip, &pad, moon, y, ram->tsight, &ram->ynbsav);
    }

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_r_to_rp(ip, &pad, moon, z, ram->tsight, &ram->znbsav);
    }

    if (st == AGC_STATUS_OK)
    {
        agc_flag_set(AGC_FLAG_ATTFLAG);
    }

    return (st);
}

/* P57: IMUCHK; OPTION2 = 3 (REFSMMAT) for the first display; P57OPT. */
agc_status_t agc_job_lunar_surface_align(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_status_t st = agc_lm_imu_check();

    if (st != AGC_STATUS_OK)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st);
    }

    agc_erase_set_option1(1, OPTION_MASK);

    return (agc_job_lunar_surface_option_display(ctx));
}

/* P57OPT: GOPERF4R V04N06 (alignment option); immediate return:
 * PHASCHNG 00014, ENDOFJOB.                                              */
agc_status_t agc_job_lunar_surface_option_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = align_flash(ctx, AGC_DSKY_GOPERF4R, OPT_CODE_1, AGC_JOB_GOTOPOOH, AGC_JOB_LUNAR_SURFACE_OPTION, AGC_JOB_LUNAR_SURFACE_OPTION_DISPLAY);

    if (st == AGC_STATUS_OK)
    {
        st = align_phase4(PH_OPTION);
    }

    return (st);
}

/* PACKOPTN: OPTION2 = 0; OPTION3 = REFSMFLG (BIT7) + ATTFLAG (BIT4);
 * OPTION1 = 00010; then DSPOPTN.                                         */
static agc_status_t pack_options(agc_job_ctx_t *ctx)
{
    agc_sp_t bits = 0;

    if (agc_flag_test(AGC_FLAG_REFSMFLG))
    {
        bits = (agc_sp_t) (bits + OPTION3_REFSM);
    }

    if (agc_flag_test(AGC_FLAG_ATTFLAG))
    {
        bits = (agc_sp_t) (bits + OPTION3_ATT);
    }

    agc_erase_set_option1(1, 0);
    agc_erase_set_option1(2, bits);
    agc_erase_set_option1(0, OPTION1_CHECK);

    if (!AGC_ASSERT(bits <= (OPTION3_REFSM + OPTION3_ATT)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_lunar_surface_options_display(ctx));
}

/*
 * ALIGNOPT: option 0 (landing site orientation at TALIGN) TDISP;
 * 1 PACKOPTN; 2 is not valid here (P57OPT again); 3 XSMD = REFSMMAT.
 */
agc_status_t agc_job_lunar_surface_option(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_sp_t opt = (agc_sp_t) (option2() & OPTION_MASK);

    if (opt == 0)
    { /* TDISP    */
        const agc_status_t st = align_set_dsptem1(agc_erase_tig());

        return ((st == AGC_STATUS_OK) ? agc_job_lunar_surface_time_display(ctx) : st);
    }

    if (opt == 2)
    {
        return (agc_job_lunar_surface_option_display(ctx));
    }

    if (opt == OPTION_MASK)
    {
        agc_align_ram()->xsmd = agc_erase_refsmmat(); /* MATMOVE  */
    }

    return (pack_options(ctx));
}

/* P57AA: flash V06N34 (TALIGN); ENTER shows it again. */
agc_status_t agc_job_lunar_surface_time_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(option2() >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_ALIGN_TIME, AGC_JOB_GOTOPOOH, AGC_JOB_LUNAR_SURFACE_TIME_ENTERED, AGC_JOB_LUNAR_SURFACE_TIME_DISPLAY));
}

/* P57D: LEMPREC to TALIGN; XSMD = UNIT(RATT); LSORIENT; PACKOPTN. */
static agc_status_t site_orientation(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);
    agc_integ_out_t o;
    agc_status_t    st = align_integ(&ip, ALIGN_LEMPREC, ram->talign, AGC_JOB_LUNAR_SURFACE_SITE_ORIENTATION, &o);

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* INTSTALL */
    }

    if (st == AGC_STATUS_OK)
    {
        st = agc_align_lsorient(&ip, agc_vec_unit(&ip, o.ratt, NULL, NULL), agc_erase_rrectcsm(), agc_erase_vrectcsm(), &ram->xsmd);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (pack_options(ctx));
}

/*
 * P57AA V33: a zero time means now (P57C); a time already past is used
 * as is; a later one also becomes TIG.  TALIGN = that time; P57D.
 */
agc_status_t agc_job_lunar_surface_time_entered(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();
    const agc_dp_t   now = align_loadtime();
    const agc_dp_t   t   = align_dsptem1();

    if (t == 0)
    {
        ram->talign = now; /* BZE P57C */
    }
    else if ((now - t) >= 0)
    {
        ram->talign = t; /* BPL P57C */
    }
    else
    {
        agc_erase_set_tig(t); /* STORE TIG */
        ram->talign = t;
    }

    if (!AGC_ASSERT(ram->talign >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (site_orientation(ctx));
}

/* LEMPREC resumed after its integration wait. */
agc_status_t agc_job_lunar_surface_site_orientation(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_align_ram()->talign >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (site_orientation(ctx));
}

/* DSPOPTN: flash V05N06 (option code and flag bits). */
agc_status_t agc_job_lunar_surface_options_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_erase_option1(0) == OPTION1_CHECK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_OPTIONS, AGC_JOB_GOTOPOOH, AGC_JOB_LUNAR_SURFACE_OPTIONS_CHOSEN, AGC_JOB_LUNAR_SURFACE_OPTIONS_DISPLAY));
}

/*
 * BYLMATT: SET INITALGN; options 1 and 3 determine gravity (GVDETER),
 * the others go to ATTCHK.
 */
static agc_status_t by_lm_attitude(agc_job_ctx_t *ctx)
{
    agc_flag_set(AGC_FLAG_INITALGN);

    if ((option2() & OPTION_GRAVITY) != 0)
    {
        return (agc_lm_lunar_surface_gravity(ctx));
    }

    return (agc_job_lunar_surface_attitude_check(ctx));
}

/*
 * DSPOPTN V33: REFSMFLG set -> GETLMATT (REFMF first); ATTFLAG set or
 * option 2/3 -> BYLMATT; otherwise alarm 701 and V05N09 (back to DSPOPTN).
 */
agc_status_t agc_job_lunar_surface_options_chosen(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_flag_test(AGC_FLAG_REFSMFLG))
    { /* GETLMATT */
        agc_interp_t ip;

        agc_interp_init(&ip);
        const agc_status_t st = agc_align_refmf(&ip);

        return ((st == AGC_STATUS_OK) ? by_lm_attitude(ctx) : st);
    }

    if (agc_flag_test(AGC_FLAG_ATTFLAG) || ((option2() & OPTION_STARS) != 0))
    {
        return (by_lm_attitude(ctx));
    }

    agc_alarm(ALM_OPTION);

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_ALARM, AGC_JOB_GOTOPOOH, AGC_JOB_LUNAR_SURFACE_OPTIONS_DISPLAY, AGC_JOB_LUNAR_SURFACE_OPTIONS_DISPLAY));
}

/* ATTCHK: PHASCHNG; ATTFLAG set -> P57OPT0, else P57JUMP. */
agc_status_t agc_job_lunar_surface_attitude_check(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_status_t st = surface_align_phase();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (agc_flag_test(AGC_FLAG_ATTFLAG))
    {
        return (agc_job_lunar_surface_stored_attitude(ctx));
    }

    return (agc_job_lunar_surface_realign(ctx));
}

/* SAMETYP: STARSAV1, STARSAV2 (present SM); MFREF; SURFLINE. */
static agc_status_t same_type(agc_job_ctx_t *ctx, agc_vec_t first, agc_vec_t second)
{
    agc_align_ram_t *ram = agc_align_ram();

    ram->starsav1 = first;
    ram->starsav2 = second;
    agc_interp_t ip;

    agc_interp_init(&ip);
    const agc_status_t st = agc_align_mfref(&ip);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_lm_lunar_surface_surfline(ctx));
}

/*
 * P57OPT0: VEC1, VEC2 = YNBSAV, ZNBSAV (moon fixed); STARSAV1, STARSAV2
 * = YNB, ZNB wrt the present SM.
 */
agc_status_t agc_job_lunar_surface_stored_attitude(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();

    ram->vec1 = ram->ynbsav;
    ram->vec2 = ram->znbsav;
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_cdutrig_t      t;
    const agc_status_t st = align_nb_present(&ip, &t, &ram->xnb);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (same_type(ctx, ram->xnb.row[1], ram->xnb.row[2]));
}

/* GSAV (nav base) in present-SM axes (CDU*NBSM). */
static agc_status_t gravity_sm(agc_vec_t *g)
{
    agc_cdutrig_t t;

    *g                    = agc_align_ram()->gsav;
    const agc_status_t st = agc_guid_cdu_nbsm(&t, g);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/*
 * P57OPT1: VEC1 = UNIT(RLS), VEC2 = ZNBSAV; STARSAV1 = GSAV in SM,
 * STARSAV2 = ZNB (present SM).  P57OPT3: VEC1 = VEC2 = UNIT(RLS) (VEC2 is
 * a dummy until the star is marked); STARSAV1 = GSAV in SM; MFREF; 1STAR.
 */
static agc_status_t site_and_gravity(agc_job_ctx_t *ctx, bool one_star)
{
    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);
    ram->vec1        = agc_vec_unit(&ip, agc_erase_rls(), NULL, NULL);
    ram->vec2        = one_star ? ram->vec1 : ram->znbsav;
    agc_cdutrig_t t;
    agc_vec_t     g;
    agc_status_t  st = align_nb_present(&ip, &t, &ram->xnb);

    if (st == AGC_STATUS_OK)
    {
        st = gravity_sm(&g);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!one_star)
    {
        return (same_type(ctx, g, ram->xnb.row[2]));
    }

    ram->starsav1 = g;
    st            = agc_align_mfref(&ip);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_lm_lunar_surface_one_star(ctx));
}

/* P57JUMP: PHASCHNG; CLEAR INITALGN; branch on the option. */
agc_status_t agc_job_lunar_surface_realign(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_status_t st = surface_align_phase();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_clear(AGC_FLAG_INITALGN);
    const agc_sp_t opt = (agc_sp_t) (option2() & OPTION_MASK);

    if (opt == 0)
    {
        return (agc_job_lunar_surface_stored_attitude(ctx)); /* P57OPT0  */
    }

    if (opt == 2)
    {
        return (agc_lm_lunar_surface_two_stars(ctx)); /* P57OPT2  */
    }

    return (site_and_gravity(ctx, opt == OPTION_MASK)); /* 1 or 3   */
}

/*
 * SURFLINE / WRTDESIR: STARAD, STARAD +6 = UNIT(XSMD VEC1 / VEC2)
 * (desired SM); 6D, 12D = STARSAV1, STARSAV2 (present SM).  The initial
 * pass (INITALGN) skips R54 (INITBY); otherwise DOALIGN: R54.
 */
agc_status_t agc_lm_lunar_surface_surfline(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);
    ram->starad[0] = agc_vec_unit(&ip, agc_mat_vec(&ip, &ram->xsmd, ram->vec1), NULL, NULL);
    ram->starad[1] = agc_vec_unit(&ip, agc_mat_vec(&ip, &ram->xsmd, ram->vec2), NULL, NULL);
    ram->vac6      = ram->starsav1;
    ram->vac12     = ram->starsav2;

    if (agc_flag_test(AGC_FLAG_INITALGN))
    {
        return (agc_job_lunar_surface_solve(ctx)); /* INITBY   */
    }

    return (agc_align_sighting_data_display(ctx, AGC_JOB_LUNAR_SURFACE_DATA_CHECKED));
}

/* DOALIGN after R54: data refused -> P57POST, accepted -> INITBY. */
agc_status_t agc_job_lunar_surface_data_checked(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!agc_flag_test(AGC_FLAG_FREEFLAG))
    {
        return (agc_job_lunar_surface_post(ctx));
    }

    return (agc_job_lunar_surface_solve(ctx));
}

/*
 * INITBY: AXISGEN (desired SM wrt present: XDC), CALCGTA (OGC); the
 * initial pass goes straight to 5DEGTEST, otherwise V06N93 is flashed
 * (PROCEED 5DEGTEST, V32 P57POST +1).
 */
agc_status_t agc_job_lunar_surface_solve(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);
    const agc_vec_t a[2] = {ram->starad[0], ram->starad[1]};
    const agc_vec_t b[2] = {ram->vac6, ram->vac12};
    agc_status_t    st   = agc_align_fine_solve(&ip, a, b, &ram->xdc);

    if (st == AGC_STATUS_OK)
    {
        agc_imu_calcgta(&ip, &ram->xdc, ram->ogc);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (agc_flag_test(AGC_FLAG_INITALGN))
    {
        return (agc_job_lunar_surface_torque_test(ctx));
    }

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_TORQUE, AGC_JOB_GOTOPOOH, AGC_JOB_LUNAR_SURFACE_TORQUE_TEST, AGC_JOB_LUNAR_SURFACE_POST));
}

/*
 * 5DEGTEST: OGCT = OGC; any angle of 5 degrees or more (V/SC 5DEGREES
 * overflows) -> COATRIM; otherwise QMIN = SURFDISP and JUSTTRIM.
 */
agc_status_t agc_job_lunar_surface_torque_test(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram   = agc_align_ram();
    bool             large = false;

    for (int i = 0; i < 3; i++)
    {
        ram->ogct[i] = ram->ogc[i]; /* SURFSUP  */
        large        = large || (agc_dp_abs(ram->ogc[i]) >= FIVE_DEGREES);
    }

    if (large)
    {
        return (agc_lm_lunar_surface_coarse_trim(ctx)); /* COATRIM  */
    }

    ram->lunar_gravity_return = AGC_JOB_LUNAR_SURFACE_ALIGNED; /* SSP QMIN */

    return (agc_lm_lunar_surface_just_trim(ctx));
}

/*
 * SURFDISP: PHASCHNG; REFSMMAT = XSMD, SET REFSMFLG.  Option 0 ends
 * (P57POST +1); B2F8: the initial pass realigns (P57JUMP), otherwise REFMF
 * keeps the new attitude, then P57POST.
 */
agc_status_t agc_job_lunar_surface_aligned(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = surface_align_phase();

    agc_erase_set_refsmmat(agc_align_ram()->xsmd); /* MATMOVE  */
    agc_flag_set(AGC_FLAG_REFSMFLG);

    if ((st != AGC_STATUS_OK) || (option2() == 0))
    {
        return ((st == AGC_STATUS_OK) ? agc_job_lunar_surface_post(ctx) : st);
    }

    if (agc_flag_test(AGC_FLAG_INITALGN))
    {
        return (agc_job_lunar_surface_realign(ctx)); /* B2F8     */
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    st = agc_align_refmf(&ip);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_lunar_surface_post(ctx));
}

/* P57POST: V50N25 R1 00014; PROCEED realigns (P57JUMP), ENTER ends. */
agc_status_t agc_job_lunar_surface_post(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(option2() >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_GOPERF1, CHK_POST, AGC_JOB_GOTOPOOH, AGC_JOB_LUNAR_SURFACE_REALIGN, AGC_JOB_LUNAR_SURFACE_POST_ENTER));
}

/*
 * P57POST ENTER: aligned by option 2 -> compute the landing site:
 * G(ref) = (CDU*NBSM GSAV) VXM REFSMMAT, SET LUNAFLAG, ALPHAV = |RLS| G
 * (m B29), CLEAR ERADFLAG, N89DISP at the present time.  Otherwise P00.
 */
agc_status_t agc_job_lunar_surface_post_enter(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (option2() != OPTION_SITE)
    {
        return (agc_exec_gotopooh());
    }

    agc_status_t st = surface_align_phase();
    agc_vec_t    g;

    if (st == AGC_STATUS_OK)
    {
        st = gravity_sm(&g);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    const agc_mat_t refsmmat = agc_erase_refsmmat();

    g                       = agc_vec_mat(&ip, g, &refsmmat);
    agc_flag_set(AGC_FLAG_LUNAFLAG);
    agc_align_ram()->alphav = agc_vec_scale(&ip, g, agc_vec_abval(&ip, agc_erase_rls()));
    agc_flag_clear(AGC_FLAG_ERADFLAG);

    return (agc_lm_landing_site_latlong(ctx, align_loadtime(), AGC_JOB_LUNAR_SURFACE_SITE_UPDATED));
}

/*
 * After N89DISP: RN = RLS (m B29, the LM position); PIPTIME = the time;
 * RLS = R-TO-RP(ALPHAV VSL2) (moon fixed, m B27); P00.
 */
agc_status_t agc_job_lunar_surface_site_updated(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);
    agc_erase_set_rn(ram->alphav);                 /* RN       */
    agc_erase_set_piptime(ram->landing_site_time); /* PIPTIME  */
    agc_navaux_padload_t pad;
    agc_vec_t            rls = agc_vec_zero();
    agc_status_t         st  = agc_navaux_get_padload(&pad);

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_r_to_rp(&ip, &pad, true, agc_vec_shl(&ip, ram->alphav, 2), ram->landing_site_time, &rls);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_erase_set_rls(rls);

    return (agc_exec_gotopooh());
}
