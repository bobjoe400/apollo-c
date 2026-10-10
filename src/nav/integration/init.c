/*
 * init.c -- INTEGRATION INITIALIZATION
 * (Comanche055 INTEGRATION_INITIALIZATION.agc pp. 1309-1333; Luminary099
 * pp. 1203-1226).  Entrances CSMPREC LEMPREC CSMCONIC LEMCONIC INTEGRVS
 * INTEGRV, the INTSTALL lock, the state vector moves, TESTLOOP (time step
 * selection), A-PCHK/RECTOUT/INTEXIT (termination and output), RVCON
 * (conic), and the driver that strings the AGC labels together.
 *
 * CM/LM differences (agc_state_vehicle()): CM has three stall areas, the
 * origin check at A-PCHK, CLEAR MIDAVFLG at INTEXIT (LM: CLEAR STATEFLG),
 * and RME/RMM = 800 km altitudes; in the LM RME = RMM = DPPOSMAX so
 * MIDFLAG (third-body terms) is never set.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/nav/integration.h"
#include "agc/programs/alignment.h"
#include "internal.h"

/* RME 2DEC 7178165 B-29 (oct 00333 01732), 800 km above the equatorial
 * radius; RMM 2DEC 2538.09 E3 B-27 (oct 00465 32324), 800 km above the
 * lunar surface (CM FRESH_START_AND_RESTART p. 210). LM: both = LODPMAX,
 * 2OCT 3777737777 (DPPOSMAX). Indexed RME - PBODY: [0] earth, [1] moon. */
static const agc_dp_t RME_CSM[2] = {3589082, 5076180};
static const agc_dp_t RME_LM[2]  = {AGC_DP_MAX, AGC_DP_MAX};
/* .3D 2DEC .3 B-2 (oct 02314 31463).                                     */
static const agc_dp_t K_P3D = 20132659;
/* DT/2MIN 2DEC 3 B-20 (oct 00000 01400): 3 cs in DT/2 format.            */
static const agc_dp_t K_DT2MIN = 768;
/* DT/2MAX 2DEC 4000 E2 B-20 (oct 14152 00000): 4000 s in DT/2 format.    */
static const agc_dp_t K_DT2MAX = 102400000;
/* OMEGMOON 2DEC* 2.66169947 E-8 B+23* (oct 07112 06620): lunar rotation
 * rate, rad/cs, scale 2^-23.                                             */
static const agc_dp_t K_OMEGMOON = 59936144;

/* INTBITAB: RASFLAG bits of each stall area (INTFLAG + REINTFLG for area
 * 0: OCT 20100, 10040, 04020); INTFLBIT + area: BIT14, BIT13, BIT12.      */
static const uint16_t INTBITAB[AGC_INTEG_AREAS_MAX] = {020100, 010040, 004020};
static const uint16_t INTFLBIT[AGC_INTEG_AREAS_MAX] = {020000, 010000, 004000};

/* Jobs asleep on a stall area (the AGC finds them by their LOC =
 * WAKESTAL; here each sleeper names its own continuation job).          */
#define INTEG_MAX_SLEEPERS AGC_EXEC_CORE_SETS_LM
static agc_job_id_t sleepers[AGC_INTEG_AREAS_MAX][INTEG_MAX_SLEEPERS];
static int          n_sleepers[AGC_INTEG_AREAS_MAX];

/* The integration erasables (one set, guarded by INTSTALL).              */
static agc_integ_ctx_t shared_ctx;

/* Permanent state vectors beyond RRECT/VRECT/TET, which live in the
 * shared erasables: DELTAV NUV RCV VCV TC XKEP of CSM [0] and LM [1].    */
typedef struct perm_ext
{
    agc_vec_t deltav, nuv, rcv, vcv;
    agc_dp_t  tc, xkep;
} perm_ext_t;

static perm_ext_t perm_ext[2];

/* ---- small services --------------------------------------------------- */

void agc_integ_flag(agc_flag_id_t f, bool on)
{
    agc_flag_put(f, on);

    if (!AGC_ASSERT(agc_flag_test(f) == on))
    {
        return;
    }
}

/* LOADTIME: TIME2,TIME1 as a DP, cs scale 2^28.                          */
agc_dp_t agc_integ_loadtime(void)
{
    const uint32_t t = agc_time_now_cs();

    if (!AGC_ASSERT(t < AGC_TIME_MODULUS_CS))
    {
        return 0; /* no valid result */
    }

    if (!AGC_ASSERT((agc_dp_t) (t & 0x0FFFFFFFu) <= AGC_DP_MAX))
    {
        return 0; /* no valid result */
    }

    return (agc_dp_t) (t & 0x0FFFFFFFu);
}

bool agc_integ_valid_ctx(const agc_integ_ctx_t *ctx)
{
    return (ctx != NULL) &&
           ((ctx->pbody == INTEG_PBODY_EARTH) ||
            (ctx->pbody == INTEG_PBODY_MOON)) &&
           (ctx->stage >= 0) && (ctx->stage <= 2) && (ctx->colreg >= 0) &&
           (ctx->colreg < AGC_INTEG_W_COLUMNS);
}

agc_integ_ctx_t *agc_integ_shared_ctx(void)
{
    return &shared_ctx;
}

agc_status_t agc_integ_ctx_init(agc_integ_ctx_t *ctx)
{
    static const agc_integ_ctx_t zero_ctx;

    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    *ctx = zero_ctx;

    if (!AGC_ASSERT(agc_integ_valid_ctx(ctx)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

/* GRP2PC (p. 1333): PHASCHNG OCT 04022.                                  */
void agc_integ_grp2pc(void)
{
    agc_phase_change(2, 04022);

    if (!AGC_ASSERT(agc_phase_get(2) == 2u))
    {
        return;
    }
}

/* SETIFLGS (p. 1314).                                                    */
void agc_integ_setiflgs(void)
{
    agc_integ_flag(AGC_FLAG_STATEFLG, true);
    agc_integ_flag(AGC_FLAG_INTYPFLG, false);
    agc_integ_flag(AGC_FLAG_DIM0FLAG, false);
    agc_integ_flag(AGC_FLAG_D6OR9FLG, false);

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_STATEFLG)))
    {
        return;
    }

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_INTYPFLG)))
    {
        return;
    }
}

/* THISVINT (FRESH_START_AND_RESTART CM p. 210 / LM p. 237).              */
void agc_integ_thisvint(void)
{
    const bool csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);

    agc_integ_flag(AGC_FLAG_VINTFLAG, csm);

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_VINTFLAG) == csm))
    {
        return;
    }
}

/* ---- permanent state vectors ----------------------------------------- */

agc_status_t agc_integ_perm_get(agc_vehicle_t veh, agc_integ_sv_t *out)
{
    if (!AGC_ASSERT(out != NULL) ||
        !AGC_ASSERT((veh == AGC_VEHICLE_CSM) || (veh == AGC_VEHICLE_LM)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    const bool        csm = (veh == AGC_VEHICLE_CSM);
    const perm_ext_t *e   = &perm_ext[csm ? 0 : 1];

    out->rrect  = csm ? agc_erase_rrectcsm() : agc_erase_rrectlem();
    out->vrect  = csm ? agc_erase_vrectcsm() : agc_erase_vrectlem();
    out->tet    = csm ? agc_erase_tetcsm() : agc_erase_tetlem();
    out->deltav = e->deltav;
    out->nuv    = e->nuv;
    out->rcv    = e->rcv;
    out->vcv    = e->vcv;
    out->tc     = e->tc;
    out->xkep   = e->xkep;

    return AGC_STATUS_OK;
}

agc_status_t agc_integ_perm_set(agc_vehicle_t veh, const agc_integ_sv_t *in)
{
    if (!AGC_ASSERT(in != NULL) ||
        !AGC_ASSERT((veh == AGC_VEHICLE_CSM) || (veh == AGC_VEHICLE_LM)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    perm_ext_t *e = &perm_ext[(veh == AGC_VEHICLE_CSM) ? 0 : 1];

    if (veh == AGC_VEHICLE_CSM)
    {
        agc_erase_set_rrectcsm(in->rrect);
        agc_erase_set_vrectcsm(in->vrect);
        agc_erase_set_tetcsm(in->tet);
    }
    else
    {
        agc_erase_set_rrectlem(in->rrect);
        agc_erase_set_vrectlem(in->vrect);
        agc_erase_set_tetlem(in->tet);
    }

    e->deltav = in->deltav;
    e->nuv    = in->nuv;
    e->rcv    = in->rcv;
    e->vcv    = in->vcv;
    e->tc     = in->tc;
    e->xkep   = in->xkep;

    return AGC_STATUS_OK;
}

/* ---- INTSTALL lock (p. 1324-1326) ------------------------------------ */

static int area_count(void)
{
    const agc_vehicle_t v = agc_state_vehicle();

    if (!AGC_ASSERT((v == AGC_VEHICLE_CSM) || (v == AGC_VEHICLE_LM)))
    {
        return 0; /* no valid result */
    }

    return (v == AGC_VEHICLE_CSM) ? AGC_INTEG_AREAS_CSM : AGC_INTEG_AREAS_LM;
}

bool agc_integ_area_busy(int area)
{
    if (!AGC_ASSERT((area >= 0) && (area < area_count())))
    {
        return true;
    }

    return (agc_flagword_get(INTEG_RASFLAG_WORD) & INTBITAB[area]) != 0;
}

/*
 * INTSTALL / ERASTAL1 / ERASTAL2 / ALLSTALL / OKTOGRAB (p. 1324-1326).
 * If the stall area is in use the job sleeps (JOBSLEEP on WAKESTAL) and
 * is resumed as resume_job; otherwise INTFLBIT is added to RASFLAG.
 * DEVIATION: the LM OKTOGRAB INHINTs and never RELINTs; here the inhibit
 * is released after the flag update (agc_inhint is counted).            */
agc_status_t agc_integ_install(int area, agc_job_id_t resume_job)
{
    if (!AGC_ASSERT((area >= 0) && (area < area_count())))
    {
        return AGC_STATUS_BAD_ARG;
    }

    const uint16_t ras = agc_flagword_get(INTEG_RASFLAG_WORD);

    if ((ras & INTBITAB[area]) != 0)
    {
        agc_exec_job_sleep(resume_job); /* JOBSLEEP */

        if (AGC_ASSERT(n_sleepers[area] < INTEG_MAX_SLEEPERS))
        {
            sleepers[area][n_sleepers[area]] = resume_job;
            n_sleepers[area]                += 1;
        }

        return AGC_STATUS_BUSY;
    }

    agc_inhint();
    agc_flagword_set(INTEG_RASFLAG_WORD, (uint16_t) (ras | INTFLBIT[area]));
    agc_relint();

    return AGC_STATUS_OK;
}

/* WAKE / WAKE1 (p. 1325): wake every job asleep on the area, release it. */
static agc_status_t wake(int area)
{
    if (!AGC_ASSERT((area >= 0) && (area < AGC_INTEG_AREAS_MAX)) ||
        !AGC_ASSERT(n_sleepers[area] <= INTEG_MAX_SLEEPERS))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_inhint(); /* INHINT (the AGC repeats it per JOBWAKE) */

    for (int i = 0; i < n_sleepers[area]; i++)
    {
        agc_exec_job_wake_if_asleep(sleepers[area][i]); /* JOBWAKE */
    }

    n_sleepers[area] = 0;
    const uint16_t ras = agc_flagword_get(INTEG_RASFLAG_WORD);

    agc_flagword_set(
        INTEG_RASFLAG_WORD, (uint16_t) (ras & (uint16_t) ~INTBITAB[area]));
    agc_relint();

    return AGC_STATUS_OK;
}

/* INTWAKE (p. 1325): if REINTFLG is set (a phase change happened between
 * INTSTALL and INTWAKE), PHASCHNG 04022 keeping our QPRET; if a restart
 * has since cleared REINTFLG do not wake.                               */
agc_status_t agc_integ_intwake(void)
{
    if ((agc_flagword_get(INTEG_RASFLAG_WORD) & INTEG_REINTBIT) != 0)
    {
        agc_integ_grp2pc();

        if ((agc_flagword_get(INTEG_RASFLAG_WORD) & INTEG_REINTBIT) == 0)
        {
            return AGC_STATUS_OK; /* GOBAC   */
        }
    }

    return wake(0);
}

agc_status_t agc_integ_intwake0(void)
{
    if (!AGC_ASSERT(area_count() >= 1))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(n_sleepers[0] <= INTEG_MAX_SLEEPERS))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return wake(0);
}

agc_status_t agc_integ_eraswake(int area)
{
    if (!AGC_ASSERT((area == 1) || (area == 2)) ||
        !AGC_ASSERT(area < area_count()))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return wake(area);
}

/* ---- state vector moves (p. 1314-1317) -------------------------------- */

/* ATOPCSM / ATOPLEM (p. 1314-1316): MOVEACSM/MOVEALEM (42 words), set the
 * permanent moon flag from MOONFLAG, SVDWN1/SVDWN2 downlink copy.  SVDWN1
 * is the CSM in both programs (this vehicle in the CM, the other in the
 * LM).  DEVIATION: after ORIGCHNG the AGC reaches ANDOUT with X2 = ORIGEX
 * (a return address); x2 here is always 0 or 2.                          */
static agc_status_t atop(agc_interp_t *ip, agc_integ_ctx_t *ctx, int x2,
                         agc_vehicle_t veh)
{
    if (!AGC_ASSERT((veh == AGC_VEHICLE_CSM) || (veh == AGC_VEHICLE_LM)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT((x2 == 0) || (x2 == 2)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    const agc_flag_id_t mflag =
        (veh == AGC_VEHICLE_CSM) ? AGC_FLAG_CMOONFLG : AGC_FLAG_LMOONFLG;
    agc_status_t st = agc_integ_perm_set(veh, &ctx->sv);

    agc_integ_flag(mflag, true);

    if (st == AGC_STATUS_OK)
    {
        st = agc_align_svdwn(ip, agc_state_vehicle() == veh, x2, ctx->sv.rcv,
                             ctx->sv.vcv, ctx->sv.deltav, ctx->sv.nuv,
                             ctx->sv.tet);
    }

    if (!agc_flag_test(AGC_FLAG_MOONFLAG))
    {
        agc_integ_flag(mflag, false);
    }

    return st;
}

agc_status_t agc_integ_atopcsm(agc_interp_t *ip, agc_integ_ctx_t *ctx, int x2)
{
    if (!AGC_ASSERT(agc_integ_valid_ctx(ctx)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return atop(ip, ctx, x2, AGC_VEHICLE_CSM);
}

agc_status_t agc_integ_atoplem(agc_interp_t *ip, agc_integ_ctx_t *ctx, int x2)
{
    if (!AGC_ASSERT(agc_integ_valid_ctx(ctx)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return atop(ip, ctx, x2, AGC_VEHICLE_LM);
}

/* PTOACSM (p. 1315): MOVEPCSM, then SETMOON / CLRMOON from CMOONFLG.    */
agc_status_t agc_integ_ptoacsm(agc_integ_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_integ_valid_ctx(ctx)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    const agc_status_t st = agc_integ_perm_get(AGC_VEHICLE_CSM, &ctx->sv);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return st;
    }

    return agc_integ_setmoon(ctx, agc_flag_test(AGC_FLAG_CMOONFLG));
}

/* USEPIOS (p. 1316-1317): the LM on the lunar surface. RCV = RP-TO-R(RLS)
 * at TDEC1, VCV = OMEGMOON (lunar pole x RCV), deviations zero, PBODY 2. */
static agc_status_t usepios(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    agc_navaux_padload_t pad;
    agc_vec_t            pole  = agc_vec_zero();
    const agc_vec_t      zunit = agc_vec_make(0, 0, AGC_DP_HALF);

    ctx->sv.tet = ctx->tdec1;
    agc_status_t st = agc_navaux_get_padload(&pad);

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_rp_to_r(ip, &pad, true, agc_erase_rls(), ctx->sv.tet,
                                &ctx->sv.rcv); /* m 2^27     */
    }

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_rp_to_r(ip, &pad, true, zunit, ctx->sv.tet, &pole);
    }

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return st;
    }

    agc_integ_flag(AGC_FLAG_MOONFLAG, true);
    ctx->sv.vcv = agc_vec_scale(ip, agc_vec_cross(ip, pole, ctx->sv.rcv),
                                K_OMEGMOON);    /* m/cs 2^5   */
    ctx->sv.deltav = agc_vec_zero();
    ctx->sv.nuv    = agc_vec_zero();
    ctx->pbody     = INTEG_PBODY_MOON;

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_MOONFLAG)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

/* PTOALEM (p. 1316): USEPIOS when SURFFLAG, else MOVEPLEM and
 * SETMOON / CLRMOON from LMOONFLG.                                       */
agc_status_t agc_integ_ptoalem(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                               bool *surface)
{
    if (!AGC_ASSERT(surface != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(agc_integ_valid_ctx(ctx)) || !AGC_ASSERT(ip != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    const bool surf = agc_flag_test(AGC_FLAG_SURFFLAG);

    if (surface != NULL)
    {
        *surface = surf;
    }

    if (surf)
    {
        return usepios(ip, ctx);
    }

    const agc_status_t st = agc_integ_perm_get(AGC_VEHICLE_LM, &ctx->sv);

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    return agc_integ_setmoon(ctx, agc_flag_test(AGC_FLAG_LMOONFLG));
}

/* ---- TESTLOOP: time step selection (p. 1323-1324) --------------------- */

/* NORFINAL (p. 1323): DT = .3 sqrt(R^3 / MU) truncated to a multiple of
 * 128 cs, limited to DT/2MAX (4000 s); result in DT/2 format (12D).     */
static agc_dp_t norfinal(agc_interp_t *ip, const agc_integ_ctx_t *ctx,
                         agc_dp_t rsq)
{
    if (!AGC_ASSERT(rsq >= 0))
    {
        return 0; /* no valid result */
    }

    if (!AGC_ASSERT(ctx->rc >= 0))
    {
        return 0; /* no valid result */
    }

    agc_dp_t x =
        agc_dp_shr_round(agc_tp_round_dp(agc_dp_mul_tp(ctx->rc, rsq)), 1);

    x = agc_dp_sqrt(ip, agc_dp_div(ip, x, agc_integ_mu(ctx->pbody)));
    x = agc_dp_shr(agc_tp_to_dp(agc_dp_mul_tp(x, K_P3D)),
                   7);         /* SR3 SR4, DLOAD MPAC */
    x = agc_dp_shl(ip, x, 15); /* SL 15D      */

    if (agc_interp_test_ovf(ip))
    {
        return K_DT2MAX; /* BOV MAXDT   */
    }

    if (agc_dp_sub(ip, K_DT2MAX, x) < 0)
    {
        return K_DT2MAX; /* BDSU BMN    */
    }

    return x;
}

/* DT/2COMP / GETMAXDT / USEMAXDT (p. 1323-1324): DT/2 = (TDEC - TET) SL 8,
 * limited in magnitude to 12D with its sign kept.                       */
static void dt2comp(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx->dtmax >= 0))
    {
        return;
    }

    if (!AGC_ASSERT(ip != NULL))
    {
        return;
    }

    const agc_dp_t dt =
        agc_dp_shl(ip, agc_dp_sub(ip, ctx->tdec, ctx->sv.tet), 8);

    ctx->dt2 = dt; /* DT/2 B-19   */

    if (agc_interp_test_ovf(ip))
    {
        ctx->dt2 = (dt < 0) ? AGC_DP_MIN : AGC_DP_MAX; /* SIGNMPAC    */
        ctx->dt2 = agc_dp_sign(ctx->dtmax, ctx->dt2);  /* USEMAXDT    */
    }
    else if (agc_dp_sub(ip, agc_dp_abs(dt), ctx->dtmax) >= 0)
    {
        ctx->dt2 = agc_dp_sign(ctx->dtmax, ctx->dt2); /* USEMAXDT    */
    }
    else
    {
        /* |DT/2| below the limit: POOHCHK */
    }
}

/* POOHCHK (p. 1324): done when |DT/2| < 3 cs.  In P00 (MODREG = 0) and
 * not via CSM/LEMPREC, stop before the last partial step, and on a new
 * entry integrate only if the state is at least 4 steps old.            */
static agc_integ_next_t poohchk(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx->dtmax >= 0))
    {
        return INTEG_NEXT_FAIL;
    }

    if (!AGC_ASSERT(ip != NULL))
    {
        return INTEG_NEXT_FAIL;
    }

    if (agc_dp_sub(ip, agc_dp_abs(ctx->dt2), K_DT2MIN) < 0)
    {
        return INTEG_NEXT_APCHK;
    }

    if ((agc_erase_modreg() != 0) || agc_flag_test(AGC_FLAG_PRECIFLG))
    {
        return INTEG_NEXT_TIMESTEP;
    }

    if (agc_dp_sub(ip, ctx->dt2, ctx->dtmax) < 0)
    {
        return INTEG_NEXT_APCHK;
    }

    if (!agc_flag_test(AGC_FLAG_NEWIFLG))
    {
        return INTEG_NEXT_TIMESTEP; /* BOFCLR      */
    }

    agc_integ_flag(AGC_FLAG_NEWIFLG, false);
    const agc_dp_t d = agc_dp_sub(ip, ctx->tdec, ctx->sv.tet);

    if (d < 0)
    {
        return INTEG_NEXT_INTEXIT; /* no backward */
    }

    const agc_dp_t dt4 =
        agc_dp_shr_round(agc_dp_shr(ctx->dt2, 4), 2); /* 4 DT */

    if (agc_dp_sub(ip, d, dt4) < 0)
    {
        return INTEG_NEXT_INTEXIT;
    }

    return INTEG_NEXT_TIMESTEP;
}

/* TESTLOOP (p. 1323): QUITFLAG, MIDFLAG (|RCV| beyond RME/RMM), step size. */
static agc_integ_next_t testloop(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (agc_flag_test(AGC_FLAG_QUITFLAG))
    {
        agc_integ_flag(AGC_FLAG_STATEFLG, false);

        return INTEG_NEXT_INTEXIT; /* CLRGO       */
    }

    if (!AGC_ASSERT(agc_integ_valid_ctx(ctx)))
    {
        return INTEG_NEXT_FAIL;
    }

    const int       b   = ctx->pbody / 2;
    const agc_dp_t *rme =
        (agc_state_vehicle() == AGC_VEHICLE_CSM) ? RME_CSM : RME_LM;

    ctx->rc = agc_vec_abval(ip, ctx->sv.rcv); /* 10D         */
    const agc_dp_t rsq = agc_vec_sq(ip, ctx->sv.rcv);    /* 34D         */

    agc_integ_flag(AGC_FLAG_MIDFLAG, false);

    if (agc_dp_sub(ip, ctx->rc, rme[b]) >= 0)
    {
        agc_integ_flag(AGC_FLAG_MIDFLAG, true);
    }

    ctx->dtmax = norfinal(ip, ctx, rsq); /* 12D         */
    dt2comp(ip, ctx);

    return poohchk(ip, ctx);
}

/* ---- termination (p. 1320-1321) --------------------------------------- */

/* A-PCHK / ANDOUT / PHEXIT: CM origin check, permanent update when
 * STATEFLG (phase change, REINTFLG, ATOPCSM/ATOPLEM, GRP2PC).            */
static agc_integ_next_t a_pchk(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return INTEG_NEXT_FAIL;
    }

    if (!AGC_ASSERT(agc_integ_valid_ctx(ctx)))
    {
        return INTEG_NEXT_FAIL;
    }

    const bool   csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);
    agc_status_t st  = AGC_STATUS_OK;

    if (csm && agc_flag_test(AGC_FLAG_MIDFLAG))
    {
        agc_dp_t test = 0;

        st = agc_integ_chkswtch(ip, ctx, &test);

        if ((st == AGC_STATUS_OK) && (test < 0))
        {
            st = agc_integ_origchng(ip, ctx); /* BPL ANDOUT  */
        }
    }

    if ((st == AGC_STATUS_OK) && agc_flag_test(AGC_FLAG_STATEFLG))
    {
        if (csm)
        {
            agc_integ_flag(AGC_FLAG_STATEFLG, false); /* BOFCLR      */
        }

        agc_integ_grp2pc(); /* OCT 04022   */
        agc_integ_flag(AGC_FLAG_REINTFLG, true);
        st = agc_flag_test(AGC_FLAG_VINTFLAG) ?
             agc_integ_atopcsm(ip, ctx, ctx->pbody) :
             agc_integ_atoplem(ip, ctx, ctx->pbody);
        agc_integ_grp2pc(); /* PHEXIT      */
    }

    return (st == AGC_STATUS_OK) ? INTEG_NEXT_RECTOUT : INTEG_NEXT_FAIL;
}

/* RECTOUT (p. 1321): rectify and leave RATT VATT TAT RATT1 VATT1 MU(P),
 * X1, X2 in the push list (ctx->out).                                    */
static agc_integ_next_t rectout(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (agc_integ_rectify(ip, ctx) != AGC_STATUS_OK)
    {
        return INTEG_NEXT_FAIL;
    }

    const int p = ctx->pbody;

    if (!AGC_ASSERT((p == INTEG_PBODY_EARTH) || (p == INTEG_PBODY_MOON)))
    {
        return INTEG_NEXT_FAIL;
    }

    ctx->out.ratt  = agc_vec_shr(ctx->sv.rrect, p); /* VSL* 0,2    */
    ctx->out.vatt  = agc_vec_shr(ctx->sv.vrect, p);
    ctx->out.tat   = ctx->sv.tet;
    ctx->out.ratt1 = ctx->sv.rrect;
    ctx->out.vatt1 = ctx->sv.vrect;
    ctx->out.mu_p  = agc_integ_mu(p);
    ctx->out.x1    = agc_flag_test(AGC_FLAG_MOONFLAG) ? -10 : -2;
    ctx->out.x2    = p;
    ctx->out.valid = true;

    if (!AGC_ASSERT(ctx->out.valid))
    {
        return INTEG_NEXT_FAIL;
    }

    return INTEG_NEXT_INTEXIT;
}

/* INTEXIT (p. 1321): reset OVFIND, clear MIDAVFLG (CM) / STATEFLG (LM),
 * AVEMIDSW, PRECIFLG, and INTWAKE.                                      */
static agc_status_t intexit(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_interp_clear_ovf(ip); /* BOV +1      */

    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        agc_integ_flag(AGC_FLAG_MIDAVFLG, false);
    }
    else
    {
        agc_integ_flag(AGC_FLAG_STATEFLG, false);
    }

    agc_integ_flag(AGC_FLAG_AVEMIDSW, false);
    agc_integ_flag(AGC_FLAG_PRECIFLG, false);

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_AVEMIDSW)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return agc_integ_intwake();
}

/* RVCON (p. 1321-1322): conic solution over TDEC - TET.                 */
static agc_integ_next_t rvcon(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return INTEG_NEXT_FAIL;
    }

    if (!AGC_ASSERT(agc_integ_valid_ctx(ctx)))
    {
        return INTEG_NEXT_FAIL;
    }

    ctx->tau = agc_dp_sub(ip, ctx->tdec, ctx->sv.tet);

    if ((agc_integ_rectify(ip,
                           ctx) != AGC_STATUS_OK) ||
        (agc_integ_kepprep(ip, ctx) != AGC_STATUS_OK))
    {
        return INTEG_NEXT_FAIL;
    }

    ctx->sv.tet = agc_dp_add(ip, ctx->sv.tc, ctx->sv.tet);

    return INTEG_NEXT_RECTOUT;
}

/* ALOADED (p. 1320): TDEC = TDEC1; precision (TESTLOOP) or conic (RVCON). */
static agc_integ_next_t aloaded(agc_integ_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return INTEG_NEXT_FAIL;
    }

    ctx->tdec = ctx->tdec1;

    if (!AGC_ASSERT(ctx->tdec == ctx->tdec1))
    {
        return INTEG_NEXT_FAIL;
    }

    return agc_flag_test(AGC_FLAG_INTYPFLG) ? INTEG_NEXT_RVCON :
           INTEG_NEXT_TESTLOOP;
}

/* INTEGRV2 (p. 1320): load A-memory from the permanent state vector of the
 * vehicle chosen by VINTFLAG, then ALOADED (or A-PCHK after USEPIOS).   */
static agc_integ_next_t integrv2(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    bool surface = false;

    if (!AGC_ASSERT(ip != NULL))
    {
        return INTEG_NEXT_FAIL;
    }

    if (!AGC_ASSERT(agc_integ_valid_ctx(ctx)))
    {
        return INTEG_NEXT_FAIL;
    }

    const agc_status_t st = agc_flag_test(AGC_FLAG_VINTFLAG) ?
                            agc_integ_ptoacsm(ctx) :
                            agc_integ_ptoalem(ip, ctx, &surface);

    if (st != AGC_STATUS_OK)
    {
        return INTEG_NEXT_FAIL;
    }

    return surface ? INTEG_NEXT_APCHK : aloaded(ctx);
}

/* ENDSTATE with DIM0FLAG (p. 1349): PHASCHNG 04022, REINTFLG, move the
 * state to permanent (ATOPCSM/ATOPLEM), then AMOVED: the W columns.     */
static agc_integ_next_t wmatrix(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    agc_integ_grp2pc();
    agc_integ_flag(AGC_FLAG_REINTFLG, true);
    const agc_status_t st = agc_flag_test(AGC_FLAG_VINTFLAG) ?
                            agc_integ_atopcsm(ip, ctx, ctx->pbody) :
                            agc_integ_atoplem(ip, ctx, ctx->pbody);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return INTEG_NEXT_FAIL;
    }

    return agc_integ_wcolumns(ip, ctx);
}

/* RELOADSV (p. 1350): reload the temporary state from the permanent one
 * by TDEC1 = TDEC and INTEGRV2.                                         */
static agc_integ_next_t reload(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_integ_valid_ctx(ctx)))
    {
        return INTEG_NEXT_FAIL;
    }

    if (!AGC_ASSERT(ip != NULL))
    {
        return INTEG_NEXT_FAIL;
    }

    ctx->tdec1 = ctx->tdec;

    return integrv2(ip, ctx);
}

/*
 * The integration proper, from `first` to INTEXIT: one bounded loop over
 * the AGC labels (rule 1/2: no goto, no recursion, fixed bound).  A
 * failure of a called routine (C only) releases the lock (DEVIATION);
 * INT-ABRT (POODOO 00430) leaves it held as the AGC does.
 */
agc_status_t agc_integ_run(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                           agc_integ_next_t first)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(agc_integ_valid_ctx(ctx)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_integ_next_t next = first;

    for (int32_t pass = 0; pass < AGC_INTEG_MAX_PASSES; pass++)
    {
        switch (next)
        {
            case INTEG_NEXT_TESTLOOP:
                next = testloop(ip, ctx);
                break;
            case INTEG_NEXT_TIMESTEP:
                next = agc_integ_timestep(ip, ctx);
                break;
            case INTEG_NEXT_APCHK:
                next = a_pchk(ip, ctx);
                break;
            case INTEG_NEXT_RECTOUT:
                next = rectout(ip, ctx);
                break;
            case INTEG_NEXT_CKMID2:
                next = agc_integ_ckmid2(ip, ctx);
                break;
            case INTEG_NEXT_WMATRIX:
                next = wmatrix(ip, ctx);
                break;
            case INTEG_NEXT_RELOAD:
                next = reload(ip, ctx);
                break;
            case INTEG_NEXT_RVCON:
                next = rvcon(ip, ctx);
                break;
            case INTEG_NEXT_INTEXIT:
                return intexit(ip);
            case INTEG_NEXT_ABORT: /* TC POODOO, OCT 00430 (INT-ABRT) */
                return agc_poodoo(AGC_INTEG_ALARM_INT_ABRT);
            case INTEG_NEXT_FAIL:
            {
                const agc_status_t sx = intexit(ip); /* release (DEVIATION) */

                return (sx != AGC_STATUS_OK) ? sx : AGC_STATUS_BAD_ARG;
            }
            default:
                return AGC_STATUS_BAD_ARG;
        }
    }

    const agc_status_t sx = intexit(ip);

    return (sx != AGC_STATUS_OK) ? sx : AGC_STATUS_NO_CONVERGE;
}

/* ---- entrances (p. 1318-1320) ----------------------------------------- */

/* INTEGRV1 (p. 1320): SET RPQFLAG NEWIFLG, then INTEGRV2 and on.        */
agc_status_t agc_integ_integrv1(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(agc_integ_valid_ctx(ctx)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_integ_flag(AGC_FLAG_RPQFLAG, true);
    agc_integ_flag(AGC_FLAG_NEWIFLG, true);
    ctx->out.valid = false;

    return agc_integ_run(ip, ctx, integrv2(ip, ctx));
}

/* CSMPREC LEMPREC (IFLAGP) / CSMCONIC LEMCONIC (IFLAGC), p. 1318-1319.   */
static agc_status_t special_entry(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                                  agc_dp_t tdec1, agc_job_id_t resume_job,
                                  bool csm, bool conic)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(agc_integ_valid_ctx(ctx)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    const agc_status_t st = agc_integ_install(0, resume_job); /* INTSTALL */

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    ctx->tdec1 = tdec1;
    agc_integ_flag(AGC_FLAG_VINTFLAG, csm);

    if (!conic)
    {
        agc_integ_flag(AGC_FLAG_PRECIFLG, true);
    }

    agc_integ_flag(AGC_FLAG_DIM0FLAG, false);
    agc_integ_flag(AGC_FLAG_INTYPFLG, conic);

    return agc_integ_integrv1(ip, ctx);
}

agc_status_t agc_integ_csmprec(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                               agc_dp_t tdec1, agc_job_id_t resume_job)
{
    if (!AGC_ASSERT((tdec1 >= AGC_DP_MIN) && (tdec1 <= AGC_DP_MAX)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return special_entry(ip, ctx, tdec1, resume_job, true, false);
}

agc_status_t agc_integ_lemprec(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                               agc_dp_t tdec1, agc_job_id_t resume_job)
{
    if (!AGC_ASSERT((tdec1 >= AGC_DP_MIN) && (tdec1 <= AGC_DP_MAX)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return special_entry(ip, ctx, tdec1, resume_job, false, false);
}

agc_status_t agc_integ_csmconic(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                                agc_dp_t tdec1, agc_job_id_t resume_job)
{
    if (!AGC_ASSERT((tdec1 >= AGC_DP_MIN) && (tdec1 <= AGC_DP_MAX)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return special_entry(ip, ctx, tdec1, resume_job, true, true);
}

agc_status_t agc_integ_lemconic(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                                agc_dp_t tdec1, agc_job_id_t resume_job)
{
    if (!AGC_ASSERT((tdec1 >= AGC_DP_MIN) && (tdec1 <= AGC_DP_MAX)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return special_entry(ip, ctx, tdec1, resume_job, false, true);
}

/* INTEGRVS (p. 1319): integrate the caller's RCV, VCV, TET.              */
agc_status_t agc_integ_integrvs(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                                agc_dp_t tdec1)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(agc_integ_valid_ctx(ctx)) ||
        !AGC_ASSERT(agc_integ_area_busy(0)))
    { /* INTSTALL held  */
        return AGC_STATUS_BAD_ARG;
    }

    ctx->tdec1 = tdec1;
    agc_integ_flag(AGC_FLAG_PRECIFLG, true);
    ctx->pbody =
        agc_flag_test(AGC_FLAG_MOONFLAG) ? INTEG_PBODY_MOON : INTEG_PBODY_EARTH;
    ctx->sv.deltav = agc_vec_zero();
    ctx->sv.nuv    = agc_vec_zero();
    const agc_status_t st = agc_integ_rectify(ip, ctx);

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    agc_integ_flag(AGC_FLAG_DIM0FLAG, false);
    agc_integ_flag(AGC_FLAG_NEWIFLG, true);
    agc_integ_flag(AGC_FLAG_RPQFLAG, true);
    ctx->out.valid = false;

    return agc_integ_run(ip, ctx, aloaded(ctx));
}

/* INTEGRV (p. 1320): caller-set flags, permanent state of VINTFLAG.     */
agc_status_t agc_integ_integrv(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                               agc_dp_t tdec1)
{
    if (!AGC_ASSERT(agc_integ_valid_ctx(ctx)) ||
        !AGC_ASSERT(agc_integ_area_busy(0)))
    { /* INTSTALL held  */
        return AGC_STATUS_BAD_ARG;
    }

    ctx->tdec1 = tdec1;

    return agc_integ_integrv1(ip, ctx);
}
