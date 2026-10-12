/**********************************************************************
 * FILE NAME: init.c
 *
 * PURPOSE:
 *   INTEGRATION INITIALIZATION: the entrances (CSMPREC, LEMPREC,
 *   CSMCONIC, LEMCONIC, INTEGRVS, INTEGRV), the INTSTALL lock (INTSTALL,
 *   ERASTAL1/2, INTWAKE, WAKE), the state vector moves (ATOPCSM/LEM,
 *   PTOACSM/LEM, USEPIOS), TESTLOOP (step selection: NORFINAL,
 *   DT/2COMP, POOHCHK), A-PCHK / RECTOUT / INTEXIT (termination and
 *   output), RVCON (conic) and the driver that strings the labels
 *   together.
 *
 * AGC SOURCE:
 *   Comanche055 INTEGRATION_INITIALIZATION.agc pp. 1309-1333;
 *   FRESH_START_AND_RESTART.agc p. 210 (RME, THISVINT)
 *   Luminary099 INTEGRATION_INITIALIZATION.agc pp. 1203-1226 (differs:
 *   one stall area, no A-PCHK origin check, CLEAR STATEFLG at INTEXIT,
 *   RME = RMM = DPPOSMAX so MIDFLAG is never set)
 *
 * EXTERNAL VARIABLES:
 *   Name        Type       I/O  Scale     Description
 *   ----------  ---------  ---  --------  -----------------------------
 *   RRECTCSM..  agc_vec_t  I/O  2^29 m    permanent CSM / LM states
 *   TETCSM/LEM  agc_dp_t   I/O  2^28 cs   their times
 *   RASFLAG     flagword   I/O  -         INTSTALL bits (FLAGWRD10)
 *   STATEFLG..  flags      I/O  -         integration control flags
 *   RLS         agc_vec_t  I    2^27 m    landing site (USEPIOS)
 *   MODREG      agc_sp_t   I    -         major mode (POOHCHK)
 *
 * EXTERNAL REFERENCES:
 *   Name                    Description
 *   ----------------------  -----------------------------------------
 *   agc_exec_job_sleep      JOBSLEEP (exec.h)
 *   agc_exec_job_wake_...   JOBWAKE (exec.h)
 *   agc_phase_change        PHASCHNG (exec.h)
 *   agc_planet_rp_to_r      RP-TO-R (nav/auxiliary.h)
 *   agc_integ_svdwn         SVDWN1 / SVDWN2 (svdwn.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   POODOO 00430 (INT-ABRT) from GOBAQUE; AGC_STATUS_NO_CONVERGE after
 *   AGC_INTEG_MAX_PASSES (C only); a called routine's failure is
 *   returned after INTSTALL is released (DEVIATION).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs in the caller's job, which holds INTSTALL; both vehicles
 *   (agc_state_vehicle()).  DEVIATION (restart): a rerun job segment
 *   takes over a stall area a restart left with REINTFLG set.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp_words.h"
#include "agc/nav/integration.h"
#include "internal.h"

/* RME 2DEC 7178165 B-29 (oct 00333 01733), 800 km above the equatorial
 * radius; RMM 2DEC 2538.09 E3 B-27 (oct 00465 32324), 800 km above the
 * lunar surface (CM FRESH_START_AND_RESTART p. 210). LM: both = LODPMAX,
 * 2OCT 3777737777 (DPPOSMAX). Indexed RME - PBODY: [0] earth, [1] moon. */
static const agc_dp_t RME_CSM[2] = {3589083, 5076180};
static const agc_dp_t RME_LM[2]  = {AGC_DP_MAX, AGC_DP_MAX};

/* .3D 2DEC .3 B-2 (oct 02314 31463).                                     */
static const agc_dp_t K_P3D = 20132659; /* .3 at 2^0 */

/* DT/2MIN 2DEC 3 B-20 (oct 00000 01400): 3 cs in DT/2 format.            */
static const agc_dp_t K_DT2MIN = 768; /* 3 cs, cs 2^19 (DT/2 format) */

/* DT/2MAX 2DEC 4000 E2 B-20 (oct 14152 00000): 4000 s in DT/2 format.    */
static const agc_dp_t K_DT2MAX = 102400000; /* 4000 s, cs 2^19 (DT/2 format) */

/* OMEGMOON 2DEC* 2.66169947 E-8 B+23* (oct 07112 06620): lunar rotation
 * rate, rad/cs, scale 2^-23.                                             */
static const agc_dp_t K_OMEGMOON = 59936144; /* rad/cs 2^-23 */

/* INTBITAB: RASFLAG bits of each stall area (INTFLAG + REINTFLG for area
 * 0: OCT 20100, 10040, 04020); INTFLBIT + area: BIT14, BIT13, BIT12.      */
static const uint16_t INTBITAB[AGC_INTEG_AREAS_MAX] = {020100, 010040, 004020};
static const uint16_t INTFLBIT[AGC_INTEG_AREAS_MAX] = {020000, 010000, 004000};

/* Jobs asleep on a stall area (the AGC finds them by their LOC =
 * WAKESTAL; here each sleeper names its own continuation job).          */
#define INTEG_MAX_SLEEPERS AGC_EXEC_CORE_SETS_LM                       /* one per core set at most */
static agc_job_id_t sleepers[AGC_INTEG_AREAS_MAX][INTEG_MAX_SLEEPERS]; /* continuation of each sleeper */
static int          n_sleepers[AGC_INTEG_AREAS_MAX];                   /* sleepers per area            */

/* The integration erasables (one set, guarded by INTSTALL).              */
static agc_integ_ctx_t shared_ctx; /* the integration erasables */

/* Permanent state vectors beyond RRECT/VRECT/TET, which live in the
 * shared erasables: DELTAV NUV RCV VCV TC XKEP of CSM [0] and LM [1].    */
typedef struct perm_ext
{
    agc_vec_t deltav; /* TDELTAV, m 2^22 (moon 2^18)            */
    agc_vec_t nuv;    /* TNUV, m/cs 2^3 (moon 2^-1)             */
    agc_vec_t rcv;    /* RCV, m 2^29 (moon 2^27)                */
    agc_vec_t vcv;    /* VCV, m/cs 2^7 (moon 2^5)               */
    agc_dp_t  tc;     /* TC, cs 2^28                            */
    agc_dp_t  xkep;   /* XKEP, sqrt(m) 2^17 (moon 2^16)         */
} perm_ext_t;

static perm_ext_t perm_ext[2]; /* [0] CSM, [1] LM */

/* ---- small services --------------------------------------------------- */

/**********************************************************************
 * LOADTIME (RT8_OP_CODES.agc C p. 1508, RTB_OP_CODES.agc L) -- TIME2, TIME1 as
 * a DP
 *
 * FUNCTION NAME: agc_integ_loadtime
 *
 * RETURNS: agc_dp_t -- agc_dp_t the time now, cs 2^28
 *********************************************************************/
agc_dp_t agc_integ_loadtime(void)
{
    const uint32_t t = agc_time_now_cs(); /* TIME2, TIME1, cs */

    return ((agc_dp_t) (t & 0x0FFFFFFFu));
}

/**********************************************************************
 * AGC: none (the integration erasables) -- the one set of integration
 * erasables, which INTSTALL guards
 *
 * FUNCTION NAME: agc_integ_shared_ctx
 *
 * RETURNS: agc_integ_ctx_t * -- the shared context
 *********************************************************************/
agc_integ_ctx_t *agc_integ_shared_ctx(void)
{
    return (&shared_ctx);
}

/**********************************************************************
 * AGC: none (C only) -- clear a context (all erasables zero)
 *
 * FUNCTION NAME: agc_integ_ctx_init
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ctx       agc_integ_ctx_t *  O    the context to clear
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a NULL
 *                          ctx
 *********************************************************************/
agc_status_t agc_integ_ctx_init(agc_integ_ctx_t *ctx)
{
    static const agc_integ_ctx_t zero_ctx; /* all fields zero */

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *ctx = zero_ctx;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (C only) -- a routine the integration called failed: keep its
 * status for agc_integ_run, which releases INTSTALL and returns it
 *
 * FUNCTION NAME: agc_integ_fail
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (fail)
 *   st        agc_status_t       I    the status of the routine that failed
 *
 * RETURNS: agc_integ_next_t -- INTEG_NEXT_FAIL
 *********************************************************************/
agc_integ_next_t agc_integ_fail(agc_integ_ctx_t *ctx, agc_status_t st)
{
    ctx->fail = st;

    return (INTEG_NEXT_FAIL);
}

/**********************************************************************
 * GRP2PC (INTEGRATION_INITIALIZATION.agc p. 1333) -- PHASCHNG OCT 04022 (group
 * 2, restart the running job segment)
 *
 * FUNCTION NAME: agc_integ_grp2pc
 *********************************************************************/
void agc_integ_grp2pc(void)
{
    agc_phase_change(04022);
}

/**********************************************************************
 * SETIFLGS (INTEGRATION_INITIALIZATION.agc p. 1314) -- STATEFLG set, INTYPFLG,
 * DIM0FLAG, D6OR9FLG clear
 *
 * FUNCTION NAME: agc_integ_setiflgs
 *********************************************************************/
void agc_integ_setiflgs(void)
{
    agc_flag_put(AGC_FLAG_STATEFLG, true);
    agc_flag_put(AGC_FLAG_INTYPFLG, false);
    agc_flag_put(AGC_FLAG_DIM0FLAG, false);
    agc_flag_put(AGC_FLAG_D6OR9FLG, false);
}

/**********************************************************************
 * THISVINT (FRESH_START_AND_RESTART.agc C p. 210 / L p. 237) -- VINTFLAG for
 * this vehicle
 *
 * FUNCTION NAME: agc_integ_thisvint
 *********************************************************************/
void agc_integ_thisvint(void)
{
    const bool csm = (agc_state_vehicle() == AGC_VEHICLE_CSM); /* CSM: VINTFLAG set */

    agc_flag_put(AGC_FLAG_VINTFLAG, csm);
}

/* ---- permanent state vectors ----------------------------------------- */

/**********************************************************************
 * MOVEPCSM / MOVEPLEM (INTEGRATION_INITIALIZATION.agc p. 1315) -- read a
 * permanent state vector (RRECT .. XKEP)
 *
 * FUNCTION NAME: agc_integ_perm_get
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   veh       agc_vehicle_t     I    vehicle of the permanent state (CSM or
 *                                    LM)
 *   out       agc_integ_sv_t *  O    the state vector
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a NULL
 *                          out or an unknown vehicle
 *********************************************************************/
agc_status_t agc_integ_perm_get(agc_vehicle_t veh, agc_integ_sv_t *out)
{
    if (!AGC_ASSERT(out != NULL) || !AGC_ASSERT((veh == AGC_VEHICLE_CSM) || (veh == AGC_VEHICLE_LM)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const bool        csm = (veh == AGC_VEHICLE_CSM); /* CSM (else LM) */
    const perm_ext_t *e   = &perm_ext[csm ? 0 : 1];   /* this vehicle's extension */

    out->rrect  = csm ? agc_erase_rrectcsm() : agc_erase_rrectlem(); /* RRECTCSM / RRECTLEM */
    out->vrect  = csm ? agc_erase_vrectcsm() : agc_erase_vrectlem(); /* VRECTCSM / VRECTLEM */
    out->tet    = csm ? agc_erase_tetcsm() : agc_erase_tetlem();     /* TETCSM / TETLEM */
    out->deltav = e->deltav;
    out->nuv    = e->nuv;
    out->rcv    = e->rcv;
    out->vcv    = e->vcv;
    out->tc     = e->tc;
    out->xkep   = e->xkep;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * MOVEACSM / MOVEALEM (INTEGRATION_INITIALIZATION.agc p. 1314) -- write a
 * permanent state vector (RRECT .. XKEP)
 *
 * FUNCTION NAME: agc_integ_perm_set
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ------------------------------
 *   veh       agc_vehicle_t           I    vehicle of the permanent state
 *                                          (CSM or LM)
 *   in        const agc_integ_sv_t *  I    the state vector
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a NULL in
 *                          or an unknown vehicle
 *********************************************************************/
agc_status_t agc_integ_perm_set(agc_vehicle_t veh, const agc_integ_sv_t *in)
{
    if (!AGC_ASSERT(in != NULL) || !AGC_ASSERT((veh == AGC_VEHICLE_CSM) || (veh == AGC_VEHICLE_LM)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    perm_ext_t *e = &perm_ext[(veh == AGC_VEHICLE_CSM) ? 0 : 1]; /* CSM [0] or LM [1] extension */

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

    return (AGC_STATUS_OK);
}

/* ---- INTSTALL lock (p. 1324-1326) ------------------------------------ */

/**********************************************************************
 * AGC: none (INTBITAB size, INTEGRATION_INITIALIZATION.agc p. 1325) -- stall
 * areas of this vehicle
 *
 * FUNCTION NAME: area_count
 *
 * RETURNS: int -- 3 (CM: INTSTALL, ERASTAL1, ERASTAL2) or 1 (LM)
 *********************************************************************/
static int area_count(void)
{
    const agc_vehicle_t v = agc_state_vehicle(); /* this vehicle */

    return ((v == AGC_VEHICLE_CSM) ? AGC_INTEG_AREAS_CSM : AGC_INTEG_AREAS_LM); /* CM 3, LM 1 */
}

/**********************************************************************
 * INTSTALL test (INTEGRATION_INITIALIZATION.agc p. 1324) -- RASFLAG INTBITAB
 * bits of the area
 *
 * FUNCTION NAME: agc_integ_area_busy
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   area      int         I    stall area: 0 INTSTALL, 1 ERASTAL1, 2 ERASTAL2
 *                              (CM)
 *
 * RETURNS: bool -- true when the area is in use (INTFLBIT or REINTFLG)
 *********************************************************************/
bool agc_integ_area_busy(int area)
{
    return ((agc_flagword_get(INTEG_RASFLAG_WORD) & INTBITAB[area]) != 0);
}

/**********************************************************************
 * INTSTALL / ERASTAL1 / ERASTAL2 / ALLSTALL / OKTOGRAB
 * (INTEGRATION_INITIALIZATION.agc pp. 1324-1326) -- grab a stall area.
 * If the stall area is in use the job sleeps (JOBSLEEP on WAKESTAL) and
 * is resumed as resume_job; otherwise INTFLBIT is added to RASFLAG.
 * DEVIATION: the LM OKTOGRAB INHINTs and never RELINTs; here the inhibit
 * is released after the flag update (agc_inhint is counted).
 *
 * FUNCTION NAME: agc_integ_install
 *
 * ARGUMENTS:
 *   Argument    Type          I/O  Description (scale)
 *   ----------  ------------  ---  --------------------------------------
 *   area        int           I    stall area: 0 INTSTALL, 1 ERASTAL1, 2
 *                                  ERASTAL2 (CM)
 *   resume_job  agc_job_id_t  I    job to continue as when INTSTALL is busy
 *                                  (JOBSLEEP); AGC_JOB_NONE: do not sleep
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (grabbed), AGC_STATUS_BUSY (asleep,
 *                          or busy for a NONE resume), AGC_STATUS_FULL
 *                          (sleeper list full), AGC_STATUS_BAD_ARG for an
 *                          unknown area
 *********************************************************************/
agc_status_t agc_integ_install(int area, agc_job_id_t resume_job)
{
    if (!AGC_ASSERT((area >= 0) && (area < area_count())))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint16_t ras = agc_flagword_get(INTEG_RASFLAG_WORD); /* RASFLAG */

    /* DEVIATION (restart): a restart clears INTFLBIT (GOPROG2) but leaves
     * REINTFLG set by the integration it interrupted.  The AGC resumes that
     * integration after its PHASCHNG 04022 and its INTWAKE releases the
     * area; in C the restart reruns the interrupted job segment from its
     * top (exec R11), so that segment takes the area over here instead of
     * sleeping on it for ever.                                            */
    if ((area == 0) && ((ras & INTFLBIT[0]) == 0))
    {
        ras = (uint16_t) (ras & (uint16_t) ~INTBITAB[0]);
    }

    if ((ras & INTBITAB[area]) != 0)
    {
        /* Each sleeper holds a core set, so the list cannot fill in flight;
         * a caller with no continuation (AGC_JOB_NONE) is not put to sleep:
         * nothing could wake it (JOBWAKE of NONE finds no job).           */
        if (n_sleepers[area] >= INTEG_MAX_SLEEPERS)
        {
            return (AGC_STATUS_FULL);
        }

        if (resume_job != AGC_JOB_NONE)
        {
            agc_exec_job_sleep(resume_job); /* JOBSLEEP */
            sleepers[area][n_sleepers[area]] = resume_job;
            n_sleepers[area]                 += 1;
        }

        return (AGC_STATUS_BUSY);
    }

    agc_inhint();
    agc_flagword_set(INTEG_RASFLAG_WORD, (uint16_t) (ras | INTFLBIT[area]));
    agc_relint();

    return (AGC_STATUS_OK);
} /* agc_integ_install */

/**********************************************************************
 * WAKE / WAKE1 (INTEGRATION_INITIALIZATION.agc p. 1325) -- wake every job
 * asleep on the area, release it.
 *
 * FUNCTION NAME: wake
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   area      int         I    stall area: 0 INTSTALL, 1 ERASTAL1, 2 ERASTAL2
 *                              (CM)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the CCSHOLE status of JOBWAKE
 *********************************************************************/
static agc_status_t wake(int area)
{
    agc_inhint(); /* INHINT (the AGC repeats it per JOBWAKE) */

    for (int i = 0; i < n_sleepers[area]; i++)
    {
        agc_status_t st = agc_exec_job_wake_if_asleep(sleepers[area][i]); /* JOBWAKE */

        if (st != AGC_STATUS_OK)
        {
            agc_relint();

            return (st); /* CCSHOLE: POODOO */
        }
    }

    n_sleepers[area]   = 0;
    const uint16_t ras = agc_flagword_get(INTEG_RASFLAG_WORD); /* RASFLAG */

    agc_flagword_set(INTEG_RASFLAG_WORD, (uint16_t) (ras & (uint16_t) ~INTBITAB[area]));
    agc_relint();

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * INTWAKE (INTEGRATION_INITIALIZATION.agc p. 1325) -- if REINTFLG is set (a
 * phase change happened between
 * INTSTALL and INTWAKE), PHASCHNG 04022 keeping our QPRET; if a restart
 * has since cleared REINTFLG do not wake.
 *
 * FUNCTION NAME: agc_integ_intwake
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of WAKE
 *********************************************************************/
agc_status_t agc_integ_intwake(void)
{
    if ((agc_flagword_get(INTEG_RASFLAG_WORD) & INTEG_REINTBIT) != 0)
    {
        agc_integ_grp2pc();

        if ((agc_flagword_get(INTEG_RASFLAG_WORD) & INTEG_REINTBIT) == 0)
        {
            return (AGC_STATUS_OK); /* GOBAC   */
        }
    }

    return (wake(0));
}

/**********************************************************************
 * INTWAKE0 (INTEGRATION_INITIALIZATION.agc p. 1325) -- WAKE for stall area 0
 * without the REINTFLG test.
 *
 * FUNCTION NAME: agc_integ_intwake_release
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of WAKE
 *********************************************************************/
agc_status_t agc_integ_intwake_release(void)
{
    return (wake(0));
}

/**********************************************************************
 * ERASWAKE (INTEGRATION_INITIALIZATION.agc C p. 1325) -- WAKE for ERASTAL1 /
 * ERASTAL2
 *
 * FUNCTION NAME: agc_integ_eraswake
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   area      int         I    stall area: 1 ERASTAL1, 2 ERASTAL2 (CM)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG for another
 *                          area, or the status of WAKE
 *********************************************************************/
agc_status_t agc_integ_eraswake(int area)
{
    if (!AGC_ASSERT((area == 1) || (area == 2)) || !AGC_ASSERT(area < area_count()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (wake(area));
}

/* ---- state vector moves (p. 1314-1317) -------------------------------- */

/**********************************************************************
 * ATOPCSM / ATOPLEM (INTEGRATION_INITIALIZATION.agc p. 1314-1316) --
 * MOVEACSM/MOVEALEM (42 words), set the
 * permanent moon flag from MOONFLAG, SVDWN1/SVDWN2 downlink copy.  SVDWN1
 * is the CSM in both programs (this vehicle in the CM, the other in the
 * LM).  x2: PBODY, or AGC_INTEG_X2_ORIGEX after an A-PCHK origin change
 * (ORIGCHNG leaves X2 = its return address; SVDWN then stores zeros).
 *
 * FUNCTION NAME: atop
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *   x2        int                I    X2 for SVDWN: PBODY (0 earth, 2 moon)
 *                                     or AGC_INTEG_X2_ORIGEX
 *   veh       agc_vehicle_t      I    vehicle of the permanent state (CSM or
 *                                     LM)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG for a NULL ip or
 *                          another x2, or the MOVEA / SVDWN status
 *********************************************************************/
static agc_status_t atop(agc_interp_t *ip, agc_integ_ctx_t *ctx, int x2, agc_vehicle_t veh)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT((x2 == 0) || (x2 == 2) || (x2 == AGC_INTEG_X2_ORIGEX)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_flag_id_t mflag = (veh == AGC_VEHICLE_CSM) ? AGC_FLAG_CMOONFLG : AGC_FLAG_LMOONFLG; /* CMOONFLG or LMOONFLG */
    agc_status_t        st    = agc_integ_perm_set(veh, &ctx->sv);                                /* status of MOVEA */

    agc_flag_put(mflag, true);

    if (st == AGC_STATUS_OK)
    {
        agc_integ_svdwn(ip, agc_state_vehicle() == veh, x2, ctx->sv.rcv, ctx->sv.vcv, ctx->sv.deltav, ctx->sv.nuv, ctx->sv.tet); /* SVDWN */
    }

    if (!agc_flag_test(AGC_FLAG_MOONFLAG))
    {
        agc_flag_put(mflag, false);
    }

    return (st);
}

/**********************************************************************
 * ATOPCSM (INTEGRATION_INITIALIZATION.agc p. 1314) -- the temporary state to
 * the CSM permanent state
 *
 * FUNCTION NAME: agc_integ_atopcsm
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *   x2        int                I    X2 for SVDWN: PBODY (0 earth, 2 moon)
 *                                     or AGC_INTEG_X2_ORIGEX
 *
 * RETURNS: agc_status_t -- the status of atop
 *********************************************************************/
agc_status_t agc_integ_atopcsm(agc_interp_t *ip, agc_integ_ctx_t *ctx, int x2)
{
    return (atop(ip, ctx, x2, AGC_VEHICLE_CSM));
}

/**********************************************************************
 * ATOPLEM (INTEGRATION_INITIALIZATION.agc p. 1315) -- the temporary state to
 * the LM permanent state
 *
 * FUNCTION NAME: agc_integ_atoplem
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *   x2        int                I    X2 for SVDWN: PBODY (0 earth, 2 moon)
 *                                     or AGC_INTEG_X2_ORIGEX
 *
 * RETURNS: agc_status_t -- the status of atop
 *********************************************************************/
agc_status_t agc_integ_atoplem(agc_interp_t *ip, agc_integ_ctx_t *ctx, int x2)
{
    return (atop(ip, ctx, x2, AGC_VEHICLE_LM));
}

/**********************************************************************
 * PTOACSM (INTEGRATION_INITIALIZATION.agc p. 1315) -- MOVEPCSM, then SETMOON /
 * CLRMOON from CMOONFLG.
 *
 * FUNCTION NAME: agc_integ_ptoacsm
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the MOVEPCSM status
 *********************************************************************/
agc_status_t agc_integ_ptoacsm(agc_integ_ctx_t *ctx)
{
    const agc_status_t st = agc_integ_perm_get(AGC_VEHICLE_CSM, &ctx->sv); /* status of MOVEPCSM */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_integ_setmoon(ctx, agc_flag_test(AGC_FLAG_CMOONFLG));

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * USEPIOS (INTEGRATION_INITIALIZATION.agc p. 1316-1317) -- the LM on the lunar
 * surface. RCV = RP-TO-R(RLS)
 * at TDEC1, VCV = OMEGMOON (lunar pole x RCV), deviations zero, PBODY 2.
 *
 * FUNCTION NAME: usepios
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the pad-load / RP-TO-R status
 *********************************************************************/
static agc_status_t usepios(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    agc_navaux_padload_t pad;                                     /* lunar pad loads */
    agc_vec_t            pole  = agc_vec_zero();                  /* lunar pole in reference axes, 2^1 */
    const agc_vec_t      zunit = agc_vec_make(0, 0, AGC_DP_HALF); /* ZUNIT, half unit 2^1 */

    ctx->sv.tet     = ctx->tdec1;
    agc_status_t st = agc_navaux_get_padload(&pad); /* status of the pad-load read, RP-TO-R */

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_rp_to_r(ip, &pad, true, agc_erase_rls(), ctx->sv.tet, &ctx->sv.rcv); /* m 2^27     */
    }

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_rp_to_r(ip, &pad, true, zunit, ctx->sv.tet, &pole);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_put(AGC_FLAG_MOONFLAG, true);
    ctx->sv.vcv    = agc_vec_scale(ip, agc_vec_cross(ip, pole, ctx->sv.rcv), K_OMEGMOON); /* m/cs 2^5   */
    ctx->sv.deltav = agc_vec_zero();
    ctx->sv.nuv    = agc_vec_zero();
    ctx->pbody     = INTEG_PBODY_MOON;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * PTOALEM (INTEGRATION_INITIALIZATION.agc p. 1316) -- USEPIOS when SURFFLAG,
 * else MOVEPLEM and
 * SETMOON / CLRMOON from LMOONFLG.
 *
 * FUNCTION NAME: agc_integ_ptoalem
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *   surface   bool *             O    SURFFLAG: USEPIOS was used (A-PCHK
 *                                     next); may be NULL
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG for a NULL ip,
 *                          or the USEPIOS / MOVEPLEM status
 *********************************************************************/
agc_status_t agc_integ_ptoalem(agc_interp_t *ip, agc_integ_ctx_t *ctx, bool *surface)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const bool surf = agc_flag_test(AGC_FLAG_SURFFLAG); /* SURFFLAG: LM on the surface */

    if (surface != NULL)
    {
        *surface = surf;
    }

    if (surf)
    {
        return (usepios(ip, ctx));
    }

    const agc_status_t st = agc_integ_perm_get(AGC_VEHICLE_LM, &ctx->sv); /* status of MOVEPLEM */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_integ_setmoon(ctx, agc_flag_test(AGC_FLAG_LMOONFLG));

    return (AGC_STATUS_OK);
}

/* ---- TESTLOOP: time step selection (p. 1323-1324) --------------------- */

/**********************************************************************
 * NORFINAL (INTEGRATION_INITIALIZATION.agc p. 1323) -- DT = .3 sqrt(R^3 / MU)
 * truncated to a multiple of
 * 128 cs, limited to DT/2MAX (4000 s); result in DT/2 format (12D).
 *
 * FUNCTION NAME: norfinal
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  -----------------------------
 *   ip        agc_interp_t *           I/O  interpretive state: MPAC work and
 *                                           the overflow indicator
 *   ctx       const agc_integ_ctx_t *  I    integration erasables (the set
 *                                           INTSTALL guards)
 *   rsq       agc_dp_t                 I    |RCV|^2 (34D), m^2 2^58 (moon
 *                                           2^54)
 *
 * RETURNS: agc_dp_t -- agc_dp_t DT/2 maximum (12D), cs 2^19
 *********************************************************************/
static agc_dp_t norfinal(agc_interp_t *ip, const agc_integ_ctx_t *ctx, agc_dp_t rsq)
{
    /* DLOAD DMP 10D 34D, SR1R: the TP product rounds once, after the shift. */
    const agc_tp_words_t m = agc_mpac_shift_round(ip, agc_mpac_dmp(agc_mpac_dload(agc_dp_to_words(ctx->rc)), agc_dp_to_words(rsq)), -1); /* R |R|^2 SR1R, m^3 2^88 (moon 2^82) */
    agc_dp_t             x = agc_dp_from_words(agc_mpac_store(m));                                                                       /* R^3 / MU, then DT, cs 2^19 */

    x = agc_dp_sqrt(agc_dp_div(ip, x, agc_integ_mu(ctx->pbody)));
    x = agc_dp_shr(agc_tp_to_dp(agc_dp_mul_tp(x, K_P3D)), 7); /* SR3 SR4, DLOAD MPAC */
    x = agc_dp_shl(ip, x, 15);                                /* SL 15D      */

    if (agc_interp_test_ovf(ip))
    {
        return (K_DT2MAX); /* BOV MAXDT   */
    }

    if (agc_dp_sub(ip, K_DT2MAX, x) < 0)
    {
        return (K_DT2MAX); /* BDSU BMN    */
    }

    return (x);
}

/**********************************************************************
 * POOHCHK (INTEGRATION_INITIALIZATION.agc p. 1324) -- done when |DT/2| < 3 cs.
 * In P00 (MODREG = 0) and
 * not via CSM/LEMPREC, stop before the last partial step, and on a new
 * entry integrate only if the state is at least 4 steps old.
 *
 * FUNCTION NAME: poohchk
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *
 * RETURNS: agc_integ_next_t -- INTEG_NEXT_TIMESTEP, INTEG_NEXT_APCHK or
 *                              INTEG_NEXT_INTEXIT
 *********************************************************************/
static agc_integ_next_t poohchk(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (agc_dp_sub(ip, agc_dp_abs(ctx->dt2), K_DT2MIN) < 0)
    {
        return (INTEG_NEXT_APCHK);
    }

    if (!((agc_erase_modreg() != 0) || agc_flag_test(AGC_FLAG_PRECIFLG)))
    {
        if (agc_dp_sub(ip, ctx->dt2, ctx->dtmax) < 0)
        {
            return (INTEG_NEXT_APCHK);
        }

        if (agc_flag_test(AGC_FLAG_NEWIFLG)) /* BOFCLR      */
        {
            agc_flag_put(AGC_FLAG_NEWIFLG, false);
            const agc_dp_t d = agc_dp_sub(ip, ctx->tdec, ctx->sv.tet); /* TDEC - TET, cs 2^28 */

            if (d < 0)
            {
                return (INTEG_NEXT_INTEXIT); /* no backward */
            }

            /* PDDL SR4 DT/2, SR2R: the SR2R rounds the bits SR4 left in MPAC+2.  */
            const agc_tp_words_t m   = agc_mpac_shift_round(ip, agc_mpac_shift(ip, agc_mpac_dload(agc_dp_to_words(ctx->dt2)), -4), -2); /* DT/2 SR4 SR2R (TP) */
            const agc_dp_t       dt4 = agc_dp_from_words(agc_mpac_store(m));                                                            /* 4 DT; 4 DT, cs 2^28 */

            if (agc_dp_sub(ip, d, dt4) < 0)
            {
                return (INTEG_NEXT_INTEXIT);
            }
        }
    }

    return (INTEG_NEXT_TIMESTEP);
}

/**********************************************************************
 * TESTLOOP (INTEGRATION_INITIALIZATION.agc p. 1323) -- QUITFLAG, MIDFLAG
 * (|RCV| beyond RME/RMM), step size.
 *
 * FUNCTION NAME: testloop
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *
 * RETURNS: agc_integ_next_t -- INTEG_NEXT_INTEXIT (QUITFLAG) or the POOHCHK
 *                              outcome
 *********************************************************************/
static agc_integ_next_t testloop(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (agc_flag_test(AGC_FLAG_QUITFLAG))
    {
        agc_flag_put(AGC_FLAG_STATEFLG, false);

        return (INTEG_NEXT_INTEXIT); /* CLRGO       */
    }

    const int       b   = ctx->pbody / 2;                                              /* PBODY / 2: 0 earth, 1 moon */
    const agc_dp_t *rme = (agc_state_vehicle() == AGC_VEHICLE_CSM) ? RME_CSM : RME_LM; /* RME, RMM of this vehicle */

    ctx->rc            = agc_vec_abval(ip, ctx->sv.rcv); /* 10D         */
    const agc_dp_t rsq = agc_vec_sq(ip, ctx->sv.rcv);    /* 34D; |RCV|^2, m^2 2^58 (moon 2^54) */

    agc_flag_put(AGC_FLAG_MIDFLAG, false);

    if (agc_dp_sub(ip, ctx->rc, rme[b]) >= 0)
    {
        agc_flag_put(AGC_FLAG_MIDFLAG, true);
    }

    ctx->dtmax = norfinal(ip, ctx, rsq); /* 12D         */

    /* DT/2COMP / GETMAXDT / USEMAXDT (p. 1323-1324): DT/2 = (TDEC - TET) SL 8,
     * limited in magnitude to 12D with its sign kept.                       */
    const agc_dp_t dt = agc_dp_shl(ip, agc_dp_sub(ip, ctx->tdec, ctx->sv.tet), 8); /* (TDEC - TET) SL 8, cs 2^20 = DT/2 2^19 */

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

    return (poohchk(ip, ctx));
} /* testloop */

/* ---- termination (p. 1320-1321) --------------------------------------- */

/**********************************************************************
 * A-PCHK / ANDOUT / PHEXIT (INTEGRATION_INITIALIZATION.agc p. 1321) -- CM
 * origin check, permanent update when STATEFLG (phase change, REINTFLG,
 * ATOPCSM/ATOPLEM, GRP2PC).
 *
 * FUNCTION NAME: a_pchk
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *
 * RETURNS: agc_integ_next_t -- INTEG_NEXT_RECTOUT, or INTEG_NEXT_FAIL with
 *                              ctx->fail
 *********************************************************************/
static agc_integ_next_t a_pchk(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const bool   csm = (agc_state_vehicle() == AGC_VEHICLE_CSM); /* CM: origin check, STATEFLG cleared */
    agc_status_t st  = AGC_STATUS_OK;                            /* status of the origin change / state moves */
    int          x2  = ctx->pbody;                               /* X2 seen by ATOPCSM/ATOPLEM     */

    if (csm && agc_flag_test(AGC_FLAG_MIDFLAG))
    {
        agc_dp_t test = 0; /* CHKSWTCH test value, m 2^29 */

        st = agc_integ_chkswtch(ip, ctx, &test);

        if ((st == AGC_STATUS_OK) && (test < 0))
        {
            st = agc_integ_origchng(ip, ctx); /* BPL ANDOUT  */
            x2 = AGC_INTEG_X2_ORIGEX;         /* LXA,2 ORIGEX */
        }
    }

    if ((st == AGC_STATUS_OK) && agc_flag_test(AGC_FLAG_STATEFLG))
    {
        if (csm)
        {
            agc_flag_put(AGC_FLAG_STATEFLG, false); /* BOFCLR      */
        }

        agc_integ_grp2pc();                                                                                      /* OCT 04022   */
        agc_flag_put(AGC_FLAG_REINTFLG, true);
        st = agc_flag_test(AGC_FLAG_VINTFLAG) ? agc_integ_atopcsm(ip, ctx, x2) : agc_integ_atoplem(ip, ctx, x2); /* ATOPCSM or ATOPLEM */
        agc_integ_grp2pc();                                                                                      /* PHEXIT      */
    }

    if (st != AGC_STATUS_OK)
    {
        return (agc_integ_fail(ctx, st));
    }

    return (INTEG_NEXT_RECTOUT);
}

/**********************************************************************
 * RECTOUT (INTEGRATION_INITIALIZATION.agc p. 1321) -- rectify and leave RATT
 * VATT TAT RATT1 VATT1 MU(P),
 * X1, X2 in the push list (ctx->out).
 *
 * FUNCTION NAME: rectout
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *
 * RETURNS: agc_integ_next_t -- INTEG_NEXT_INTEXIT
 *********************************************************************/
static agc_integ_next_t rectout(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    agc_integ_rectify(ip, ctx); /* RECTIFY */
    const int p = ctx->pbody;   /* PBODY: 0 earth, 2 moon */

    ctx->out.ratt  = agc_vec_shr(ctx->sv.rrect, p);               /* VSL* 0,2    */
    ctx->out.vatt  = agc_vec_shr(ctx->sv.vrect, p);
    ctx->out.tat   = ctx->sv.tet;
    ctx->out.ratt1 = ctx->sv.rrect;
    ctx->out.vatt1 = ctx->sv.vrect;
    ctx->out.mu_p  = agc_integ_mu(p);
    ctx->out.x1    = agc_flag_test(AGC_FLAG_MOONFLAG) ? -10 : -2; /* X1: -10 moon, -2 earth */
    ctx->out.x2    = p;
    ctx->out.valid = true;

    return (INTEG_NEXT_INTEXIT);
}

/**********************************************************************
 * INTEXIT (INTEGRATION_INITIALIZATION.agc p. 1321) -- reset OVFIND, clear
 * MIDAVFLG (CM) / STATEFLG (LM),
 * AVEMIDSW, PRECIFLG, and INTWAKE.
 *
 * FUNCTION NAME: intexit
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  I/O  interpretive state: MPAC work and the
 *                                  overflow indicator
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of INTWAKE
 *********************************************************************/
static agc_status_t intexit(agc_interp_t *ip)
{
    agc_interp_clear_ovf(ip); /* BOV +1      */

    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        agc_flag_put(AGC_FLAG_MIDAVFLG, false);
    }
    else
    {
        agc_flag_put(AGC_FLAG_STATEFLG, false);
    }

    agc_flag_put(AGC_FLAG_AVEMIDSW, false);
    agc_flag_put(AGC_FLAG_PRECIFLG, false);

    return (agc_integ_intwake());
}

/**********************************************************************
 * RVCON (INTEGRATION_INITIALIZATION.agc p. 1321-1322) -- conic solution over
 * TDEC - TET.
 *
 * FUNCTION NAME: rvcon
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *
 * RETURNS: agc_integ_next_t -- INTEG_NEXT_RECTOUT, or INTEG_NEXT_FAIL with
 *                              ctx->fail
 *********************************************************************/
static agc_integ_next_t rvcon(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    ctx->tau = agc_dp_sub(ip, ctx->tdec, ctx->sv.tet);

    agc_integ_rectify(ip, ctx);                   /* RECTIFY */
    agc_status_t st = agc_integ_kepprep(ip, ctx); /* status of KEPPREP */

    if (st != AGC_STATUS_OK)
    {
        return (agc_integ_fail(ctx, st));
    }

    ctx->sv.tet = agc_dp_add(ip, ctx->sv.tc, ctx->sv.tet);

    return (INTEG_NEXT_RECTOUT);
}

/**********************************************************************
 * ALOADED (INTEGRATION_INITIALIZATION.agc p. 1320) -- TDEC = TDEC1; precision
 * (TESTLOOP) or conic (RVCON).
 *
 * FUNCTION NAME: aloaded
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *
 * RETURNS: agc_integ_next_t -- INTEG_NEXT_RVCON (INTYPFLG) or
 *                              INTEG_NEXT_TESTLOOP
 *********************************************************************/
static agc_integ_next_t aloaded(agc_integ_ctx_t *ctx)
{
    ctx->tdec = ctx->tdec1;

    return (agc_flag_test(AGC_FLAG_INTYPFLG) ? INTEG_NEXT_RVCON : INTEG_NEXT_TESTLOOP); /* conic (RVCON) or precision (TESTLOOP) */
}

/**********************************************************************
 * INTEGRV2 (INTEGRATION_INITIALIZATION.agc p. 1320) -- load A-memory from the
 * permanent state vector of the
 * vehicle chosen by VINTFLAG, then ALOADED (or A-PCHK after USEPIOS).
 *
 * FUNCTION NAME: load_permanent_state
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *
 * RETURNS: agc_integ_next_t -- INTEG_NEXT_APCHK (surface), the ALOADED
 *                              outcome, or INTEG_NEXT_FAIL with ctx->fail
 *********************************************************************/
static agc_integ_next_t load_permanent_state(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    bool surface = false; /* USEPIOS used */

    const agc_status_t st = agc_flag_test(AGC_FLAG_VINTFLAG) ? agc_integ_ptoacsm(ctx) : agc_integ_ptoalem(ip, ctx, &surface); /* status of PTOACSM or PTOALEM */

    if (st != AGC_STATUS_OK)
    {
        return (agc_integ_fail(ctx, st));
    }

    return (surface ? INTEG_NEXT_APCHK : aloaded(ctx)); /* A-PCHK after USEPIOS, else ALOADED */
}

/**********************************************************************
 * ENDSTATE with DIM0FLAG (ORBITAL_INTEGRATION.agc p. 1349) -- PHASCHNG 04022,
 * REINTFLG, move the
 * state to permanent (ATOPCSM/ATOPLEM), then AMOVED: the W columns.
 *
 * FUNCTION NAME: wmatrix
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *
 * RETURNS: agc_integ_next_t -- the AMOVED outcome, or INTEG_NEXT_FAIL with
 *                              ctx->fail
 *********************************************************************/
static agc_integ_next_t wmatrix(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    agc_integ_grp2pc();
    agc_flag_put(AGC_FLAG_REINTFLG, true);
    const agc_status_t st = agc_flag_test(AGC_FLAG_VINTFLAG) ? agc_integ_atopcsm(ip, ctx, ctx->pbody) : agc_integ_atoplem(ip, ctx, ctx->pbody); /* status of ATOPCSM or ATOPLEM */

    if (st != AGC_STATUS_OK)
    {
        return (agc_integ_fail(ctx, st));
    }

    return (agc_integ_wcolumns(ip, ctx));
}

/**********************************************************************
 * TESTLOOP .. INTEXIT (INTEGRATION_INITIALIZATION.agc pp. 1320-1324) -- the
 * integration proper, from `first` to INTEXIT: one bounded loop over
 * the AGC labels (rule 1/2: no goto, no recursion, fixed bound).  A
 * failure of a called routine (C only) releases the lock (DEVIATION);
 * INT-ABRT (POODOO 00430) leaves it held as the AGC does.
 *
 * FUNCTION NAME: agc_integ_run
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *   first     agc_integ_next_t   I    the label to start at
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (INTEXIT), POODOO 00430 (INT-ABRT),
 *                          the failed routine's status,
 *                          AGC_STATUS_NO_CONVERGE after AGC_INTEG_MAX_PASSES,
 *                          or AGC_STATUS_BAD_ARG for a NULL ip
 *********************************************************************/
agc_status_t agc_integ_run(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_integ_next_t first)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_integ_next_t next = first; /* label to run next */

    for (int32_t pass = 0; pass < AGC_INTEG_MAX_PASSES; pass++)
    {
        switch (next)
        {
            case INTEG_NEXT_TESTLOOP:
                next       = testloop(ip, ctx);
                break;
            case INTEG_NEXT_TIMESTEP:
                next       = agc_integ_timestep(ip, ctx);
                break;
            case INTEG_NEXT_APCHK:
                next       = a_pchk(ip, ctx);
                break;
            case INTEG_NEXT_RECTOUT:
                next       = rectout(ip, ctx);
                break;
            case INTEG_NEXT_MIDTOAV_CHECK:
                next       = agc_integ_midtoav_check(ip, ctx);
                break;
            case INTEG_NEXT_WMATRIX:
                next       = wmatrix(ip, ctx);
                break;
            case INTEG_NEXT_RELOAD: /* RELOADSV (p. 1350): TDEC1 = TDEC, INTEGRV2 */
                ctx->tdec1 = ctx->tdec;
                next       = load_permanent_state(ip, ctx);
                break;
            case INTEG_NEXT_RVCON:
                next       = rvcon(ip, ctx);
                break;
            case INTEG_NEXT_INTEXIT:
                return (intexit(ip));
            case INTEG_NEXT_ABORT:  /* TC POODOO, OCT 00430 (INT-ABRT) */
                return (agc_poodoo(AGC_INTEG_ALARM_INT_ABRT));
            case INTEG_NEXT_FAIL:
            {
                const agc_status_t sx = intexit(ip); /* release (DEVIATION) */

                return ((ctx->fail != AGC_STATUS_OK) ? ctx->fail : sx); /* the failed routine's status first */
            }
            default:
                return (AGC_STATUS_BAD_ARG);
        }
    } /* driver passes */

    const agc_status_t sx = intexit(ip); /* status of INTEXIT */

    return ((sx != AGC_STATUS_OK) ? sx : AGC_STATUS_NO_CONVERGE); /* INTEXIT's status, else the bound ran out */
} /* agc_integ_run */

/* ---- entrances (p. 1318-1320) ----------------------------------------- */

/**********************************************************************
 * INTEGRV1 (INTEGRATION_INITIALIZATION.agc p. 1320) -- SET RPQFLAG NEWIFLG,
 * then INTEGRV2 and on.
 *
 * FUNCTION NAME: agc_integ_integrate_permanent
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *
 * RETURNS: agc_status_t -- the status of agc_integ_run, or AGC_STATUS_BAD_ARG
 *                          for a NULL ip
 *********************************************************************/
agc_status_t agc_integ_integrate_permanent(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_put(AGC_FLAG_RPQFLAG, true);
    agc_flag_put(AGC_FLAG_NEWIFLG, true);
    ctx->out.valid = false;

    return (agc_integ_run(ip, ctx, load_permanent_state(ip, ctx)));
}

/**********************************************************************
 * CSMPREC LEMPREC (IFLAGP) / CSMCONIC LEMCONIC (IFLAGC)
 * (INTEGRATION_INITIALIZATION.agc pp. 1318-1319) -- INTSTALL, then integrate a
 * permanent state to TDEC1.
 *
 * FUNCTION NAME: special_entry
 *
 * ARGUMENTS:
 *   Argument    Type               I/O  Description (scale)
 *   ----------  -----------------  ---  ---------------------------------
 *   ip          agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                       overflow indicator
 *   ctx         agc_integ_ctx_t *  I/O  integration erasables (the set
 *                                       INTSTALL guards)
 *   tdec1       agc_dp_t           I    TDEC1, the time to integrate to, cs
 *                                       2^28
 *   resume_job  agc_job_id_t       I    job to continue as when INTSTALL is
 *                                       busy (JOBSLEEP)
 *   csm         bool               I    VINTFLAG: the CSM state (else the LM)
 *   conic       bool               I    INTYPFLG: conic only (else precision)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, the INTSTALL status (BUSY: asleep),
 *                          or the integration status
 *********************************************************************/
static agc_status_t special_entry(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1, agc_job_id_t resume_job, bool csm, bool conic)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_status_t st = agc_integ_install(0, resume_job); /* INTSTALL */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ctx->tdec1 = tdec1;
    agc_flag_put(AGC_FLAG_VINTFLAG, csm);

    if (!conic)
    {
        agc_flag_put(AGC_FLAG_PRECIFLG, true);
    }

    agc_flag_put(AGC_FLAG_DIM0FLAG, false);
    agc_flag_put(AGC_FLAG_INTYPFLG, conic);

    return (agc_integ_integrate_permanent(ip, ctx));
}

/**********************************************************************
 * CSMPREC (INTEGRATION_INITIALIZATION.agc p. 1318) -- precision integration of
 * the CSM permanent state to TDEC1
 *
 * FUNCTION NAME: agc_integ_csmprec
 *
 * ARGUMENTS:
 *   Argument    Type               I/O  Description (scale)
 *   ----------  -----------------  ---  ---------------------------------
 *   ip          agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                       overflow indicator
 *   ctx         agc_integ_ctx_t *  I/O  integration erasables (the set
 *                                       INTSTALL guards)
 *   tdec1       agc_dp_t           I    TDEC1, the time to integrate to, cs
 *                                       2^28
 *   resume_job  agc_job_id_t       I    job to continue as if INTSTALL is
 *                                       busy (JOBSLEEP)
 *
 * RETURNS: agc_status_t -- the status of special_entry
 *********************************************************************/
agc_status_t agc_integ_csmprec(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1, agc_job_id_t resume_job)
{
    return (special_entry(ip, ctx, tdec1, resume_job, true, false));
}

/**********************************************************************
 * THISPREC (INTEGRATION_INITIALIZATION.agc p. 1318; EQUALS CSMPREC in
 * Comanche, LEMPREC in Luminary) -- precision integration of this vehicle's
 * permanent state.
 *
 * FUNCTION NAME: agc_integ_thisprec
 *
 * ARGUMENTS:
 *   Argument    Type               I/O  Description (scale)
 *   ----------  -----------------  ---  ---------------------------------
 *   ip          agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                       overflow indicator
 *   ctx         agc_integ_ctx_t *  I/O  integration erasables (the set
 *                                       INTSTALL guards)
 *   tdec1       agc_dp_t           I    TDEC1, the time to integrate to, cs
 *                                       2^28
 *   resume_job  agc_job_id_t       I    job to continue as if INTSTALL is
 *                                       busy (JOBSLEEP)
 *
 * RETURNS: agc_status_t -- the status of special_entry
 *********************************************************************/
agc_status_t agc_integ_thisprec(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1, agc_job_id_t resume_job)
{
    bool csm = agc_state_vehicle() == AGC_VEHICLE_CSM; /* this vehicle is the CSM */

    return (special_entry(ip, ctx, tdec1, resume_job, csm, false));
}

/**********************************************************************
 * OTHPREC (INTEGRATION_INITIALIZATION.agc p. 1318) -- precision integration of
 * the other vehicle's permanent state
 *
 * FUNCTION NAME: agc_integ_othprec
 *
 * ARGUMENTS:
 *   Argument    Type               I/O  Description (scale)
 *   ----------  -----------------  ---  ---------------------------------
 *   ip          agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                       overflow indicator
 *   ctx         agc_integ_ctx_t *  I/O  integration erasables (the set
 *                                       INTSTALL guards)
 *   tdec1       agc_dp_t           I    TDEC1, the time to integrate to, cs
 *                                       2^28
 *   resume_job  agc_job_id_t       I    job to continue as if INTSTALL is
 *                                       busy (JOBSLEEP)
 *
 * RETURNS: agc_status_t -- the status of special_entry
 *********************************************************************/
agc_status_t agc_integ_othprec(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1, agc_job_id_t resume_job)
{
    bool csm = agc_state_vehicle() == AGC_VEHICLE_CSM; /* this vehicle is the CSM */

    return (special_entry(ip, ctx, tdec1, resume_job, !csm, false));
}

/**********************************************************************
 * LEMPREC (INTEGRATION_INITIALIZATION.agc p. 1318) -- precision integration of
 * the LM permanent state to TDEC1
 *
 * FUNCTION NAME: agc_integ_lemprec
 *
 * ARGUMENTS:
 *   Argument    Type               I/O  Description (scale)
 *   ----------  -----------------  ---  ---------------------------------
 *   ip          agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                       overflow indicator
 *   ctx         agc_integ_ctx_t *  I/O  integration erasables (the set
 *                                       INTSTALL guards)
 *   tdec1       agc_dp_t           I    TDEC1, the time to integrate to, cs
 *                                       2^28
 *   resume_job  agc_job_id_t       I    job to continue as if INTSTALL is
 *                                       busy (JOBSLEEP)
 *
 * RETURNS: agc_status_t -- the status of special_entry
 *********************************************************************/
agc_status_t agc_integ_lemprec(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1, agc_job_id_t resume_job)
{
    return (special_entry(ip, ctx, tdec1, resume_job, false, false));
}

/**********************************************************************
 * CSMCONIC (INTEGRATION_INITIALIZATION.agc p. 1319) -- conic extrapolation of
 * the CSM permanent state to TDEC1
 *
 * FUNCTION NAME: agc_integ_csmconic
 *
 * ARGUMENTS:
 *   Argument    Type               I/O  Description (scale)
 *   ----------  -----------------  ---  ---------------------------------
 *   ip          agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                       overflow indicator
 *   ctx         agc_integ_ctx_t *  I/O  integration erasables (the set
 *                                       INTSTALL guards)
 *   tdec1       agc_dp_t           I    TDEC1, the time to integrate to, cs
 *                                       2^28
 *   resume_job  agc_job_id_t       I    job to continue as if INTSTALL is
 *                                       busy (JOBSLEEP)
 *
 * RETURNS: agc_status_t -- the status of special_entry
 *********************************************************************/
agc_status_t agc_integ_csmconic(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1, agc_job_id_t resume_job)
{
    return (special_entry(ip, ctx, tdec1, resume_job, true, true));
}

/**********************************************************************
 * LEMCONIC (INTEGRATION_INITIALIZATION.agc p. 1319) -- conic extrapolation of
 * the LM permanent state to TDEC1
 *
 * FUNCTION NAME: agc_integ_lemconic
 *
 * ARGUMENTS:
 *   Argument    Type               I/O  Description (scale)
 *   ----------  -----------------  ---  ---------------------------------
 *   ip          agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                       overflow indicator
 *   ctx         agc_integ_ctx_t *  I/O  integration erasables (the set
 *                                       INTSTALL guards)
 *   tdec1       agc_dp_t           I    TDEC1, the time to integrate to, cs
 *                                       2^28
 *   resume_job  agc_job_id_t       I    job to continue as if INTSTALL is
 *                                       busy (JOBSLEEP)
 *
 * RETURNS: agc_status_t -- the status of special_entry
 *********************************************************************/
agc_status_t agc_integ_lemconic(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1, agc_job_id_t resume_job)
{
    return (special_entry(ip, ctx, tdec1, resume_job, false, true));
}

/**********************************************************************
 * INTEGRVS (INTEGRATION_INITIALIZATION.agc p. 1319) -- integrate the caller's
 * RCV, VCV, TET.
 *
 * FUNCTION NAME: agc_integ_integrvs
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *   tdec1     agc_dp_t           I    TDEC1, the time to integrate to, cs
 *                                     2^28
 *
 * RETURNS: agc_status_t -- the status of agc_integ_run, RECTIFY's, or
 *                          AGC_STATUS_BAD_ARG for a NULL ip or INTSTALL not
 *                          held
 *********************************************************************/
agc_status_t agc_integ_integrvs(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(agc_integ_area_busy(0)))
    { /* INTSTALL held  */
        return (AGC_STATUS_BAD_ARG);
    }

    ctx->tdec1     = tdec1;
    agc_flag_put(AGC_FLAG_PRECIFLG, true);
    ctx->pbody     = agc_flag_test(AGC_FLAG_MOONFLAG) ? INTEG_PBODY_MOON : INTEG_PBODY_EARTH; /* PBODY from MOONFLAG */
    ctx->sv.deltav = agc_vec_zero();
    ctx->sv.nuv    = agc_vec_zero();
    agc_integ_rectify(ip, ctx);

    agc_flag_put(AGC_FLAG_DIM0FLAG, false);
    agc_flag_put(AGC_FLAG_NEWIFLG, true);
    agc_flag_put(AGC_FLAG_RPQFLAG, true);
    ctx->out.valid = false;

    return (agc_integ_run(ip, ctx, aloaded(ctx)));
}

/**********************************************************************
 * INTEGRV (INTEGRATION_INITIALIZATION.agc p. 1320) -- caller-set flags,
 * permanent state of VINTFLAG.
 *
 * FUNCTION NAME: agc_integ_integrv
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *   tdec1     agc_dp_t           I    TDEC1, the time to integrate to, cs
 *                                     2^28
 *
 * RETURNS: agc_status_t -- the status of agc_integ_run, or AGC_STATUS_BAD_ARG
 *                          when INTSTALL is not held
 *********************************************************************/
agc_status_t agc_integ_integrv(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1)
{
    if (!AGC_ASSERT(agc_integ_area_busy(0)))
    { /* INTSTALL held  */
        return (AGC_STATUS_BAD_ARG);
    }

    ctx->tdec1 = tdec1;

    return (agc_integ_integrate_permanent(ip, ctx));
}
