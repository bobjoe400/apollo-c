/**********************************************************************
 * FILE NAME: prelaunch_gyrocompass.c
 *
 * PURPOSE:
 *   Comanche prelaunch alignment, part 2: P02 gyrocompassing.  The
 *   half-second loop (SLEEPIE, SETGWLST, ALLOOP, ALFLT, NORMLOP), the
 *   erection and compass filter (ALWAYSG, COMPGS), the five-second
 *   torquing (ENDGTSAL, LASTGTS, RESTAIER), the azimuth change (RESTEST1,
 *   RESTEST3, RESCHNG, SPITGYRO), the liftoff test (CHKCOMED, PRELTERM)
 *   and the V78 azimuth load (AZMTHCG1).
 *
 * AGC SOURCE:
 *   Comanche055 IMU_CALIBRATION_AND_ALIGNMENT.agc pp. 434-447, 454 (CM)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   DELV          agc_vec_t  I/O  PIPA pulses B14 agc_erase_delv()
 *   PIPAX..Z      agc_sp_t   I/O  PIPA pulses    agc_erase_pipax() ..
 *   XSM..ZSM      agc_mat_t  I    half-unit      agc_align_ram()->xsm
 *   ERCOMP1       agc_dp_t   I/O  rev 2^0        gyro buffer ERCOMP
 *   AINLA overlay struct     I/O  see header     prelaunch_ram()
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  ------------------------------------------
 *   agc_imu_one_over_pipa  1/PIPA (compensation of DELV)
 *   agc_imu_pulse_buf      IMUPULSE on ERCOMP (SPITGYRO)
 *   agc_job_earth_orbit_   P11 (PRELTERM: POSTJUMP P11)
 *   insertion_monitor
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ALARM 01601 via SOMERR2 when the torquing ends bad.  The IMU
 *   performance test's drift filter (ALCGKK .., GEOCOMP1 = 0) returns
 *   AGC_STATUS_UNIMPLEMENTED (erasable-memory program, see
 *   prelaunch_align.c).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only.  Restart group 5 (phases 5.7 .. 5.33) and 4.17 (V78);
 *   tasks run with interrupts inhibited.
 *
 * NOTES:
 *   DEVIATION: STOREDTA/LOADSTDT copy 26 words (THETAX1 .. AINLA +37D);
 *   here only the compass words (THETAX1 .. INTVEC1 +5) are kept, the
 *   other six belong to the drift test.  ALTIM -0 is kept as +0.
 *   DEVIATION: GYROBUSY in EARTHR* runs RESTAIER again (the AGC resumes
 *   inside IMUPULSE); RESTAIER recomputes ERCOMP1 and TMARK from THETAX1
 *   and ALK, so the commands are the same.
 *
 * ALGORITHM:
 *   Every 0.5 s ALLOOP samples the PIPAs into DELV; ALFLT compensates
 *   them (1/PIPA), resolves them to the local level (DELV XSM), filters
 *   the two horizontal components (FILDELV1, INTVEC1) and builds THETAN1
 *   (erection while ERECTIM1 > 0, gyrocompass after).  Every 10 samples
 *   (LENGTHOT) ERCOMP1 = XSM THETAN1 + THETAX1 plus the earth rate is
 *   torqued in (RESTAIER, EARTHR*).  A new LAUNCHAZ (V78) slews the Z
 *   gyro and re-erects for 320 s.  Liftoff (channel 30 bit 5 or the
 *   backup GRR flag) goes to P11.
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/csm/entry.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"
#include "prelaunch_internal.h"

#define GEOCONS1     ((agc_dp_t) 26843546)        /* 2DEC .1    (03146 14632)    */
#define GEOCONS2     ((agc_dp_t) 1342177)         /* 2DEC .005  (00121 35341)    */
#define GEOCONS3     ((agc_dp_t) 16642998)        /* 2DEC .062  (01767 31666)    */
#define GEOCONS4     ((agc_dp_t) 80531)           /* 2DEC .0003 (00004 35223)    */
#define GEOCONS5     ((agc_dp_t) 134217728)       /* HIDPHALF (20000 00000), .5  */
#define SOUPLY       ((agc_dp_t) 251002909)       /* 2DEC .93505870, PIP gain    */
#define SOUPLY_ERECT ((agc_dp_t) 70508392)        /* 2DEC .26266423, gain/4      */
#define INTVAL_ALX1S ((agc_sp_t) 0220)            /* INTVAL +2: DEC 144          */
#define INTVAL_CMPX1 ((agc_sp_t) -1)              /* INTVAL +3: DEC -1           */
#define TIME1_MASK   037777u                      /* TIME1: 14-bit counter, cs   */
#define TIME1_NEGMAX ((int32_t) 037777)           /* AD NEGMAX: -16383 cs        */
#define SAMPLE_SOON  ((int32_t) 4)                /* RIGHTGTS: CAF FOUR, 4 cs    */
#define CYCLE_PASSES ((agc_sp_t) 9)               /* ANNNNNN: CAF NINE           */
#define SLEW_PASSES  ((agc_sp_t) 0100)            /* RESCHNG: BIT7, 320 s erect  */
#define FILTER_PHASE ((uint16_t) 023)             /* 19DECML: NEWPHASE 5.23      */
#define VN_LAUNCH_AZ ((uint16_t) ((6 << 7) | 29)) /* VN0629: V06N29        */

/**********************************************************************
 * AGC: none (helper) -- TIME1, the low 14 bits of the clock
 *
 * RETURNS: agc_sp_t -- C(TIME1), cs B0 (0 .. 16383)
 **********************************************************************/
static agc_sp_t time1(void)
{
    uint32_t t = agc_time_now_cs(); /* TIME2,TIME1, cs 2^28            */

    if (!AGC_ASSERT(t < AGC_TIME_MODULUS_CS))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) (t & TIME1_MASK));
}

/**********************************************************************
 * SETGWLST (IMU_CALIBRATION_AND_ALIGNMENT.agc p.436) -- WAITLIST ALLOOP
 * 1SECXT1 after the last sample (GTSWTLT1), or in 4 cs if that is past
 *
 * GTSWTLT1 - TIME1 > 0 means TIME1 wrapped: AD NEGMAX (-16383, "10 MS
 * ERROR OK").
 *
 * RETURNS: agc_status_t of WAITLIST (TWIDDLE)
 **********************************************************************/
static agc_status_t setgwlst(void)
{
    prelaunch_ram_t *r = prelaunch_ram(); /* AINLA overlay */

    if (!AGC_ASSERT(r->onesecxt1 > 0) || !AGC_ASSERT(r->gtswtlt1 >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    int32_t dt = (int32_t) r->gtswtlt1 - (int32_t) time1(); /* CS, AD  */

    if (dt > 0)
    {
        dt = dt - TIME1_NEGMAX; /* AD NEGMAX                         */
    }

    dt = dt + (int32_t) r->onesecxt1; /* AD 1SECXT1                  */

    if (dt <= 0)
    {
        dt = SAMPLE_SOON; /* RIGHTGTS                                */
    }

    if (!AGC_ASSERT(dt > 0 && dt <= 2 * (int32_t) r->onesecxt1))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_waitlist_call((uint16_t) dt, AGC_TASK_GYROCOMPASSING_SAMPLE); /* status of the call */

    agc_relint();

    return (st);
}

/**********************************************************************
 * CHKCOMED / PRELTERM (IMU_CALIBRATION_AND_ALIGNMENT.agc pp.446-447) --
 * liftoff check; on liftoff PRIOCHNG 22 and POSTJUMP P11
 *
 * Liftoff: channel 30 bit 5 reads 0 (inverted input) or GRRBKFLG (backup
 * GRR) is set.  In MM 07 (IMU test) it returns at once.
 * DEVIATION: the AGC returns from the MM 07 branch with interrupts still
 * inhibited; the counted C INHINT is released on every path.
 *
 * ARGUMENTS:
 *   ctx      agc_job_ctx_t *  I  the calling job
 *   liftoff  bool *           O  true: P11 has taken over this job
 *
 * RETURNS: agc_status_t -- OK, or the status of P11
 **********************************************************************/
static agc_status_t chkcomed(agc_job_ctx_t *ctx, bool *liftoff)
{
    *liftoff = false;
    agc_inhint();

    if (agc_phase_checkmm(PRELAUNCH_MM_IMU_TEST))
    {
        agc_relint();

        return (AGC_STATUS_OK); /* GOBKCALB                       */
    }

    bool lift = ((agc_chan_read(AGC_CHAN_CHAN30) & AGC_CH030_LIFTOFF) == 0u) || agc_flag_test(AGC_FLAG_GRRBKFLG); /* RXOR, GRRBKBIT */

    agc_relint();

    if (!lift)
    {
        return (AGC_STATUS_OK); /* RELINT, GOBKCALB                */
    }

    *liftoff = true; /* PRELTERM                                     */

    if (!AGC_ASSERT(!agc_phase_checkmm(PRELAUNCH_MM_IMU_TEST)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_exec_prio_change(PRELAUNCH_PRIO_FILTER); /* PRIOCHNG PRIO22   */

    return (agc_job_earth_orbit_insertion_monitor(ctx)); /* POSTJUMP P11 */
}

/**********************************************************************
 * SLEEPIE +1 / WTLISTNT (IMU_CALIBRATION_AND_ALIGNMENT.agc p.436) --
 * phase 5.13; CHKCOMED; SETGWLST; ENDOFJOB
 *
 * CCS TORQNDX (vertical drift test) is not modelled: TORQNDX is zero
 * except in the untranslated drift test.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the gyrocompass job
 *
 * RETURNS: agc_status_t -- OK (ENDOFJOB) or the status of P11
 **********************************************************************/
static agc_status_t sleepie_next(agc_job_ctx_t *ctx)
{
    agc_phase_change(5, 00135); /* restart at WTLISTNT              */

    if (!AGC_ASSERT(prelaunch_ram()->onesecxt1 > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(prelaunch_ram()->lengthot >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_gyrocompassing_wait(ctx));
}

/**********************************************************************
 * WTLISTNT (IMU_CALIBRATION_AND_ALIGNMENT.agc p.436, restart 5.13)
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the gyrocompass job
 *
 * RETURNS: agc_status_t -- OK (ENDOFJOB) or the status of P11
 **********************************************************************/
agc_status_t agc_job_gyrocompassing_wait(agc_job_ctx_t *ctx)
{
    bool liftoff = false; /* CHKCOMED went to P11                 */

    agc_status_t st = chkcomed(ctx, &liftoff); /* status of the call */

    if (liftoff || (st != AGC_STATUS_OK))
    {
        return (st);
    }

    st = setgwlst();

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    return (AGC_STATUS_OK); /* ENDOFJOB                             */
}

/**********************************************************************
 * ANNNNNN (IMU_CALIBRATION_AND_ALIGNMENT.agc p.446) -- LENGTHOT = 9,
 * SLEEPIE +1 (ten passes per torquing cycle)
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the gyrocompass job
 *
 * RETURNS: agc_status_t of WTLISTNT
 **********************************************************************/
static agc_status_t torquing_cycle_start(agc_job_ctx_t *ctx)
{
    prelaunch_ram()->lengthot = CYCLE_PASSES; /* CAF NINE           */

    if (!AGC_ASSERT(prelaunch_ram()->lengthot == 9))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (sleepie_next(ctx));
}

/**********************************************************************
 * ESTIMS (IMU_CALIBRATION_AND_ALIGNMENT.agc p.434) -- restart group 5 to
 * RSTGTS1 (phase 5.7), group 4 off, then RSTGTS1
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the gyrocompass job
 *
 * RETURNS: agc_status_t of RSTGTS1
 **********************************************************************/
agc_status_t prelaunch_estims(agc_job_ctx_t *ctx)
{
    agc_phase_change2(00075, 00004, NULL); /* 2PHSCHNG 5.7, 4 off      */

    if (!AGC_ASSERT(prelaunch_ram()->geocomp1 >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_gyrocompassing_restart(ctx));
}

/**********************************************************************
 * ESTICADR (IMU_CALIBRATION_AND_ALIGNMENT.agc p.424) -- the job woken by
 * GOESTIMS after PERFDLAY: ESTIMS
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the woken job
 *
 * RETURNS: agc_status_t of ESTIMS
 **********************************************************************/
agc_status_t agc_job_gyrocompassing_start(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (prelaunch_estims(ctx));
}

/**********************************************************************
 * RSTGTS1 (IMU_CALIBRATION_AND_ALIGNMENT.agc pp.434-435, restart 5.7) --
 * zero the PIPAs and the filter, load the initial gains; for the
 * compass: ERTHRVSE, ERECTIME = LENGTHOT, P02, PIPUSE, ANNNNNN
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the gyrocompass job
 *
 * RETURNS: agc_status_t of WTLISTNT
 **********************************************************************/
agc_status_t agc_job_gyrocompassing_restart(agc_job_ctx_t *ctx)
{
    prelaunch_ram_t *r    = prelaunch_ram(); /* AINLA overlay */
    const agc_dp_t   z[3] = {0, 0, 0};       /* SCHZEROS, rev 2^0           */
    agc_interp_t     ip;                     /* interpreter state           */

    agc_inhint();
    r->gtswtlt1     = time1();        /* CA TIME1, TS GTSWTLT1                  */
    agc_erase_set_pipax(0);
    agc_erase_set_pipay(0);
    agc_erase_set_pipaz(0);
    agc_relint();
    r->filt.thetax1 = agc_vec_zero(); /* ZEROING ALX1S -1 .. +77D     */
    r->filt.thetan1 = agc_vec_zero();

    for (int k = 0; k < 2; k++)
    {
        r->filt.fildelv1[k] = 0;
        r->filt.intvec1[k]  = 0;
        r->dpipa[k]         = 0;
    }

    r->altim             = 0;
    r->altims            = 0;
    agc_imu_set_gcompsw(0);                      /* STOVL GCOMPSW -1                      */
    r->alx1s             = INTVAL_ALX1S;         /* INTVAL +2: initial drift gains        */
    r->cmpx1             = INTVAL_CMPX1;
    r->alk               = SOUPLY;
    r->filt.thetax1.c[0] = SOUPLY_ERECT;
    agc_imu_gyro_buf_set(AGC_IMU_GBUF_GCOMP, z); /* STORE GCOMP        */
    agc_erase_set_delv(agc_vec_zero());          /* STORE DELVX        */

    if (r->geocomp1 == 0)
    {
        return (sleepie_next(ctx)); /* CCS GEOCOMP1: IMU test        */
    }

    agc_interp_init(&ip);
    agc_status_t st = agc_csm_prelaunch_erthrvse(&ip, r->latitude); /* status of the call */

    if (st == AGC_STATUS_OK)
    {
        r->erectim1 = r->lengthot;                             /* CA LENGTHOT, TS ERECTIME        */
        st          = agc_phase_newmode(PRELAUNCH_MM_COMPASS); /* MM 02 */
    }

    if (st == AGC_STATUS_OK)
    {
        st = agc_imu_pipuse(); /* PIPA fail -> ISS alarm             */
    }

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    return (torquing_cycle_start(ctx));
} /* agc_job_gyrocompassing_restart */

/**********************************************************************
 * ALLOOP / ALLOOP3 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.437) -- the
 * sample task: GTSWTLT1 = TIME1, GEOSAVE1 = ALTIM, phase 5.11, ALLOOP1
 *
 * RETURNS: agc_status_t of ALLOOP1
 **********************************************************************/
agc_status_t agc_task_gyrocompassing_sample(void)
{
    prelaunch_ram_t *r = prelaunch_ram(); /* AINLA overlay */

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    r->gtswtlt1 = time1();  /* CA TIME1, TS GTSWTLT1                  */
    r->geosave1 = r->altim; /* ALLOOP3                               */
    agc_phase_change(5, 00115);

    if (!AGC_ASSERT(r->gtswtlt1 >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_task_gyrocompassing_sample_restart());
}

/**********************************************************************
 * ALLOOP1 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.437, restart 5.11) --
 * count ALTIM toward zero, DELV = PIPAs (PIPAs zeroed), phase 5.23,
 * SPECSTS
 *
 * ALTIM > 0: ALTIMS = ALTIM - 1, ALTIM = -(ALTIM - 1); ALTIM = 0:
 * ALTIMS = 0; ALTIM < 0: ALTIM = -(|ALTIM| - 1) (CCS A fall-through).
 *
 * RETURNS: agc_status_t of SPECSTS
 **********************************************************************/
agc_status_t agc_task_gyrocompassing_sample_restart(void)
{
    prelaunch_ram_t *r = prelaunch_ram();  /* AINLA overlay */
    agc_vec_t        d = agc_erase_delv(); /* DELV, PIPA pulses B14 */

    r->altim = r->geosave1; /* CAE GEOSAVE1, TS ALTIM               */

    if (r->altim > 0)
    {
        r->altims = (agc_sp_t) (r->altim - 1); /* CA A, TS ALTIMS     */
        r->altim  = (agc_sp_t) (1 - r->altim); /* CS A, TS ALTIM      */
    }
    else if (r->altim == 0)
    {
        r->altims = 0; /* TS ALTIMS; ALTIM = -0, kept as +0          */
    }
    else
    {
        r->altim = (agc_sp_t) (r->altim + 1); /* CS A, TS ALTIM       */
    }

    const agc_sp_t pipa[3] = {agc_erase_pipax(), agc_erase_pipay(), agc_erase_pipaz()}; /* PIPA pulses B0  */

    agc_erase_set_pipax(0); /* CAF ZERO, XCH PIPAX                  */
    agc_erase_set_pipay(0);
    agc_erase_set_pipaz(0);

    for (int i = 0; i < 3; i++)
    {
        d.c[i] = agc_dp_pack(pipa[i], agc_dp_low(d.c[i])); /* TS DELVX */
    }

    agc_erase_set_delv(d);
    agc_phase_newphase(5, FILTER_PHASE, false); /* NEWPHASE 5.23      */

    if (!AGC_ASSERT(agc_erase_pipax() == 0 && agc_erase_pipaz() == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(r->altims >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_task_gyrocompassing_filter_start());
} /* agc_task_gyrocompassing_sample_restart */

/**********************************************************************
 * SPECSTS (IMU_CALIBRATION_AND_ALIGNMENT.agc p.437, restart 5.23) --
 * FINDVAC PRIO22 ALFLT; TASKOVER
 *
 * RETURNS: agc_status_t of FINDVAC
 **********************************************************************/
agc_status_t agc_task_gyrocompassing_filter_start(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_exec_findvac(PRELAUNCH_PRIO_FILTER, AGC_JOB_GYROCOMPASSING_FILTER); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return (st);
    }

    return (st); /* TASKOVER                                         */
}

/**********************************************************************
 * ALWAYSG (IMU_CALIBRATION_AND_ALIGNMENT.agc p.444) -- level filter on
 * DPIPAY and DPIPAZ (X1 = 8, 4)
 *
 *   FILDELV1 = FILDELV1 + .1 (DPIPA - FILDELV1)
 *   INTVEC1  = INTVEC1 + FILDELV1
 *   term     = (FILDELV1 + .005 INTVEC1) HIDPHALF   (pushed)
 *
 * ARGUMENTS:
 *   ip    agc_interp_t *  I/O  interpreter state
 *   term  agc_dp_t[2]     O    PD 0 (Y) and PD 2 (Z) terms, B14
 **********************************************************************/
static void alwaysg(agc_interp_t *ip, agc_dp_t term[2])
{
    prelaunch_filter_t *f = &prelaunch_ram()->filt; /* compass filter words */

    for (int k = 0; k < 2; k++)
    {
        const agc_dp_t dp = prelaunch_ram()->dpipa[k];          /* PIPA B14     */
        agc_dp_t       x  = agc_dp_sub(ip, dp, f->fildelv1[k]); /* DSU*: B14 */

        x              = agc_dp_add(ip, agc_dp_mul(ip, x, GEOCONS1), f->fildelv1[k]);
        f->fildelv1[k] = x;                                 /* STORE    */
        f->intvec1[k]  = agc_dp_add(ip, x, f->intvec1[k]);  /* DAD*     */
        x              = agc_dp_mul(ip, f->intvec1[k], GEOCONS2);
        x              = agc_dp_add(ip, x, f->fildelv1[k]); /* DAD*     */
        term[k]        = agc_dp_mul(ip, x, GEOCONS5);       /* DMPR, PUSH     */
    }

    if (!AGC_ASSERT(prelaunch_ram()->erectim1 >= 0))
    {
        prelaunch_ram()->erectim1 = 0; /* recovery: erection over     */
    }
}

/**********************************************************************
 * ALWAYSG +7 / COMPGS / ADDINDRF (IMU_CALIBRATION_AND_ALIGNMENT.agc
 * pp.444-445) -- THETAN1 from the filter
 *
 * Erection (ERECTIM1 > 0): THETAN1 +2 -= term Z, THETAN1 +4 -= term Y.
 * Compass: THETAN1 += FILDELV1; THETAN1 +4 -= .062 FILDELV1;
 * THETAN1 +2 -= .062 FILDELV1 +4 + .0003 INTVEC1 +4.
 *
 * ARGUMENTS:
 *   ip    agc_interp_t *    I/O  interpreter state
 *   term  const agc_dp_t *  I    ALWAYSG terms (Y, Z), B14
 **********************************************************************/
static void compgs(agc_interp_t *ip, const agc_dp_t term[2])
{
    prelaunch_filter_t *f  = &prelaunch_ram()->filt; /* compass filter words */
    agc_vec_t          *th = &f->thetan1;            /* THETAN1, rev 2^0        */

    if (prelaunch_ram()->erectim1 != 0)
    { /* ERECTION ONLY                                                */
        th->c[1] = agc_dp_sub(ip, th->c[1], term[1]); /* DSU, STODL   */
        th->c[2] = agc_dp_sub(ip, th->c[2], term[0]); /* BDSU         */

        return;
    }

    th->c[0]   = agc_dp_add(ip, th->c[0], f->fildelv1[0]);                           /* COMPGS    */
    th->c[2]   = agc_dp_sub(ip, th->c[2], agc_dp_mul(ip, f->fildelv1[0], GEOCONS3));
    agc_dp_t t = agc_dp_sub(ip, th->c[1], agc_dp_mul(ip, f->fildelv1[1], GEOCONS3)); /* Y torque term, rev 2^0 */

    th->c[1] = agc_dp_sub(ip, t, agc_dp_mul(ip, f->intvec1[1], GEOCONS4));

    if (!AGC_ASSERT(prelaunch_ram()->erectim1 == 0))
    {
        prelaunch_ram()->erectim1 = 0; /* recovery: erection over     */
    }
}

/**********************************************************************
 * NORMLOP (IMU_CALIBRATION_AND_ALIGNMENT.agc p.438) -- level PIPA
 * increments: (DELV VXM XSM) VSL1, DPIPAY = -Y, DPIPAZ = Z
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state
 **********************************************************************/
static void normlop(agc_interp_t *ip)
{
    prelaunch_ram_t *r = prelaunch_ram(); /* AINLA overlay */

    agc_vec_t v = agc_vec_shl(ip, agc_vec_mat(ip, agc_erase_delv(), &agc_align_ram()->xsm), 1); /* level PIPA increments, B14 */

    r->dpipa[0] = agc_dp_neg(v.c[1]); /* DLOAD DCOMP MPAC +3       */
    r->dpipa[1] = v.c[2];             /* MPAC +5                   */

    if (!AGC_ASSERT(r->geocomp1 == 0 || r->geocomp1 == 1))
    {
        r->geocomp1 = 1; /* recovery: compass                          */
    }
}

/**********************************************************************
 * ALCGKK (IMU_CALIBRATION_AND_ALIGNMENT.agc pp.439-441) -- the IMU
 * performance test drift filter.  Stub: it ends in the erasable-memory
 * program LAT(SPL) (see prelaunch_align.c).
 *
 * ARGUMENTS: none
 *
 * RETURNS: AGC_STATUS_UNIMPLEMENTED
 **********************************************************************/
static agc_status_t alcgkk(void)
{
    if (!AGC_ASSERT(prelaunch_ram()->geocomp1 == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_UNIMPLEMENTED);
}

/**********************************************************************
 * ENDGTSAL / LASTGTS (IMU_CALIBRATION_AND_ALIGNMENT.agc p.445) -- after
 * ten passes torque the platform, unless the gyros are still busy
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the filter job
 *
 * RETURNS: agc_status_t of SLEEPIE, CHKCOMED/P11 or RESTAIER
 **********************************************************************/
static agc_status_t endgtsal(agc_job_ctx_t *ctx)
{
    prelaunch_ram_t *r       = prelaunch_ram(); /* AINLA overlay */
    bool             liftoff = false;           /* CHKCOMED went to P11        */
    agc_dp_t         buf[3];                    /* ERCOMP1, rev 2^0            */

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(r->lengthot >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (r->lengthot > 0)
    {
        r->lengthot = (agc_sp_t) (r->lengthot - 1); /* SLEEPIE: TS     */

        return (sleepie_next(ctx));
    }

    agc_status_t st = chkcomed(ctx, &liftoff); /* status of the call */

    if (liftoff || (st != AGC_STATUS_OK))
    {
        return (st);
    }

    if (agc_imu_gyros_busy())
    {
        return (sleepie_next(ctx)); /* CCS LGYRO: .5 s from now     */
    }

    st              = agc_imu_gyro_buf_get(AGC_IMU_GBUF_ERCOMP, buf); /* LASTGTS   */
    r->filt.thetax1 = agc_vec_make(buf[0], buf[1], buf[2]);
    r->alk          = prelaunch_tmark();                              /* DLOAD TMARK, STORE ALK    */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    return (agc_job_gyrocompassing_torque(ctx));
}

/**********************************************************************
 * ALFLT1 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.438, restart 5.21) --
 * LOADSTDT, then the filter pass
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the filter job
 *
 * RETURNS: agc_status_t of the pass (see ALFLT)
 **********************************************************************/
agc_status_t agc_job_gyrocompassing_filter_restart(agc_job_ctx_t *ctx)
{
    prelaunch_ram_t *r       = prelaunch_ram(); /* AINLA overlay */
    bool             liftoff = false;           /* CHKCOMED went to P11        */
    agc_dp_t         term[2];                   /* ALWAYSG push list, B14      */
    agc_interp_t     ip;                        /* interpreter state           */

    r->filt = r->restarpt; /* LOADSTDT                              */

    if (r->geocomp1 > 0)
    {
        agc_status_t st = chkcomed(ctx, &liftoff); /* status of the call */

        if (liftoff || (st != AGC_STATUS_OK))
        {
            return (st);
        }

        agc_vec_t d = agc_erase_delv(); /* DELV, PIPA pulses B14     */

        st = agc_imu_one_over_pipa(&d); /* COMPENSATION IF IN COMPASS */
        agc_erase_set_delv(d);

        if (!AGC_ASSERT(st == AGC_STATUS_OK))
        {
            return (st);
        }
    }

    agc_interp_init(&ip);
    normlop(&ip);

    if (r->geocomp1 <= 0)
    {
        return (alcgkk()); /* BMN ALWAYSG not taken            */
    }

    alwaysg(&ip, term);
    compgs(&ip, term);

    return (endgtsal(ctx)); /* ADDINDRF EXIT                         */
} /* agc_job_gyrocompassing_filter_restart */

/**********************************************************************
 * ALFLT (IMU_CALIBRATION_AND_ALIGNMENT.agc p.438) -- STOREDTA, phase
 * 5.21, then ALFLT1's pass
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the filter job (FINDVAC PRIO22)
 *
 * RETURNS: agc_status_t of the pass
 **********************************************************************/
agc_status_t agc_job_gyrocompassing_filter(agc_job_ctx_t *ctx)
{
    prelaunch_ram_t *r = prelaunch_ram(); /* AINLA overlay */

    r->restarpt = r->filt; /* STOREDTA                               */
    agc_phase_change(5, 00215);

    if (!AGC_ASSERT(r->restarpt.thetan1.c[0] == r->filt.thetan1.c[0]))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_gyrocompassing_filter_restart(ctx));
}

/**********************************************************************
 * RESTAIER (IMU_CALIBRATION_AND_ALIGNMENT.agc p.445, restart 5.27) --
 * ERCOMP1 = (THETAN1 MXV XSM) VSL1 + THETAX1, TMARK = ALK, EARTHR*
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the gyrocompass job
 *
 * RETURNS: agc_status_t of EARTHR* or of the next step
 **********************************************************************/
agc_status_t agc_job_gyrocompassing_torque(agc_job_ctx_t *ctx)
{
    prelaunch_ram_t *r    = prelaunch_ram(); /* AINLA overlay */
    bool             done = false;           /* torquing already over         */
    agc_dp_t         buf[3];                 /* ERCOMP1, rev 2^0              */
    agc_interp_t     ip;                     /* interpreter state             */

    agc_phase_change(5, 00275);
    agc_interp_init(&ip);
    agc_vec_t e = agc_vec_add(&ip, agc_vec_shl(&ip, agc_mat_vec(&ip, &agc_align_ram()->xsm, r->filt.thetan1), 1), r->filt.thetax1); /* ERCOMP1 vector, rev 2^0 */

    for (int i = 0; i < 3; i++)
    {
        buf[i] = e.c[i]; /* STODL ERCOMP1                           */
    }

    agc_imu_gyro_buf_set(AGC_IMU_GBUF_ERCOMP, buf);
    agc_status_t st = prelaunch_set_tmark(r->alk); /* STORE TMARK     */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    st = prelaunch_earthr_star(&ip, AGC_JOB_RESTAIER, AGC_JOB_GYROCOMPASSING_TORQUED, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return (st);
    }

    return (agc_job_gyrocompassing_torqued(ctx));
} /* agc_job_gyrocompassing_torque */

/**********************************************************************
 * RESTAIER after EARTHR* (IMU_CALIBRATION_AND_ALIGNMENT.agc p.445) --
 * GEOSAVE1 = ERECTIM1, phase 5.15, RESTEST1
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the woken job
 *
 * RETURNS: agc_status_t of RESTEST1, or SOMERR2's
 **********************************************************************/
agc_status_t agc_job_gyrocompassing_torqued(agc_job_ctx_t *ctx)
{
    bool ok = false; /* torquing ended good                       */

    agc_status_t st = prelaunch_stall_outcome(&ok); /* status of the call */

    if (!ok)
    {
        return (st);
    }

    prelaunch_ram()->geosave1 = prelaunch_ram()->erectim1; /* CAE, TS */
    agc_phase_change(5, 00155);

    if (!AGC_ASSERT(prelaunch_ram()->geosave1 >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_gyrocompassing_erection_check(ctx));
}

/**********************************************************************
 * NOCHORLD / ANNNNNN (IMU_CALIBRATION_AND_ALIGNMENT.agc p.446) --
 * ERECTIM1 = GEOSAVE1 - 1 while positive, then ten more passes
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the gyrocompass job
 *
 * RETURNS: agc_status_t of ANNNNNN
 **********************************************************************/
static agc_status_t nochorld(agc_job_ctx_t *ctx)
{
    prelaunch_ram_t *r = prelaunch_ram(); /* AINLA overlay */

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(r->geosave1 >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (r->geosave1 > 0)
    {
        r->erectim1 = (agc_sp_t) (r->geosave1 - 1); /* CCS, TS ERECTIM1 */
    }

    return (torquing_cycle_start(ctx));
}

/**********************************************************************
 * RESTEST1 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.445, restart 5.15) --
 * THETAN1 = 0; PREMTRXC > 0: NOCHORLD, else phase 5.25, RESTEST3
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the gyrocompass job
 *
 * RETURNS: agc_status_t of NOCHORLD or RESTEST3
 **********************************************************************/
agc_status_t agc_job_gyrocompassing_erection_check(agc_job_ctx_t *ctx)
{
    prelaunch_ram_t *r = prelaunch_ram(); /* AINLA overlay */

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(r->premtrxc >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    r->filt.thetan1 = agc_vec_zero(); /* VLOAD SCHZEROS, STORE      */

    if (r->premtrxc > 0)
    {
        return (nochorld(ctx)); /* CCS PREMTRXC                      */
    }

    agc_phase_change(5, 00255);

    return (agc_job_gyrocompassing_azimuth_check(ctx));
}

/**********************************************************************
 * RESTEST3 / ADERCOMP / NOAZCHGE (IMU_CALIBRATION_AND_ALIGNMENT.agc
 * pp.445-446, restart 5.25) -- a new LAUNCHAZ (V78): PREMTRXC + 1,
 * NEWAZMTH = LAUNCHAZ, ERCOMP +4 = LAUNCHAZ - OLDAZMTH, POSN17C, phase
 * 5.33, RESCHNG; none: PREMTRXC = 1, NOCHORLD
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the gyrocompass job
 *
 * RETURNS: agc_status_t of RESCHNG or NOCHORLD
 **********************************************************************/
agc_status_t agc_job_gyrocompassing_azimuth_check(agc_job_ctx_t *ctx)
{
    prelaunch_ram_t *r = prelaunch_ram(); /* AINLA overlay */
    agc_dp_t         buf[3];              /* ERCOMP, rev 2^0                      */
    agc_interp_t     ip;                  /* interpreter state                    */

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(r->premtrxc >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_init(&ip);
    agc_dp_t d = agc_dp_sub(&ip, r->launchaz, r->oldazmth); /* rev 2^0 */

    if (d == 0)
    {
        r->premtrxc = 1; /* NOAZCHGE                                 */

        return (nochorld(ctx));
    }

    r->premtrxc     = (agc_sp_t) (r->premtrxc + 1);                   /* SLOAD DAD, STODL  */
    r->newazmth     = r->launchaz;
    agc_status_t st = agc_imu_gyro_buf_get(AGC_IMU_GBUF_ERCOMP, buf); /* status of the call */

    buf[2] = d; /* ADERCOMP: STORE ERCOMP +4                         */
    agc_imu_gyro_buf_set(AGC_IMU_GBUF_ERCOMP, buf);
    st     = prelaunch_posn17c(&ip);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    agc_phase_change(5, 00335);

    return (agc_job_gyrocompassing_azimuth_slew(ctx));
}

/**********************************************************************
 * RESCHNG / SPITGYRO (IMU_CALIBRATION_AND_ALIGNMENT.agc p.446, restart
 * 5.33) -- OLDAZMTH = NEWAZMTH, LENGTHOT = 64 (320 s erecting), phase
 * 5.7, IMUPULSE ERCOMP, IMUSTALL
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the gyrocompass job
 *
 * RETURNS: agc_status_t of the stall or of ESTIMS
 **********************************************************************/
agc_status_t agc_job_gyrocompassing_azimuth_slew(agc_job_ctx_t *ctx)
{
    prelaunch_ram_t *r    = prelaunch_ram(); /* AINLA overlay */
    bool             done = false;           /* torquing already over         */

    r->oldazmth     = r->newazmth;                                             /* DCA NEWAZMTH, DXCH OLDAZMTH         */
    r->lengthot     = SLEW_PASSES;                                             /* CA BIT7                             */
    agc_phase_change(5, 00075);
    agc_status_t st = agc_imu_pulse_buf(AGC_IMU_GBUF_ERCOMP, AGC_JOB_RESCHNG); /* status of the call */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* GYROBUSY: RESCHNG runs again     */
    }

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_ALARM))
    {
        return (st);
    }

    st = prelaunch_imustall(AGC_JOB_GYROCOMPASSING_AZIMUTH_SLEWED, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return (st);
    }

    return (agc_job_gyrocompassing_azimuth_slewed(ctx));
}

/**********************************************************************
 * SPITGYRO after IMUSTALL (IMU_CALIBRATION_AND_ALIGNMENT.agc p.446) --
 * TC ESTIMS (re-initialize)
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the woken job
 *
 * RETURNS: agc_status_t of ESTIMS, or SOMERR2's
 **********************************************************************/
agc_status_t agc_job_gyrocompassing_azimuth_slewed(agc_job_ctx_t *ctx)
{
    bool ok = false; /* torquing ended good                       */

    agc_status_t st = prelaunch_stall_outcome(&ok); /* status of the call */

    if (!ok)
    {
        return (st);
    }

    if (!AGC_ASSERT(prelaunch_ram()->lengthot == SLEW_PASSES))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (prelaunch_estims(ctx));
}

/**********************************************************************
 * AZMTHCG1 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.454, V78, restart
 * 4.17) -- show NEWAZMTH (1STO2S into DSPTEM1), CLEANDSP
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the V78 job
 *
 * RETURNS: agc_status_t of CLEANDSP (the job goes on at the flash)
 **********************************************************************/
agc_status_t agc_job_gyrocompass_azimuth_change(agc_job_ctx_t *ctx)
{
    const agc_dsky_cont_t next = {AGC_JOB_GYROCOMPASS_AZIMUTH_FLASH,
                                  AGC_JOB_GYROCOMPASS_AZIMUTH_FLASH,
                                  AGC_JOB_GYROCOMPASS_AZIMUTH_FLASH};

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_dsky_set_dsptem(0, agc_rtb_1sto2s(prelaunch_ram()->newazmth)); /* RTB 1STO2S */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    return (agc_dsky_display(ctx, AGC_DSKY_CLEANDSP, 0, &next));
}

/**********************************************************************
 * AZMTHCG1 +7 (p.454) -- flash V06N29; V34 and ENTER flash again, V33
 * loads
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the V78 job
 *
 * RETURNS: agc_status_t of GOFLASH
 **********************************************************************/
agc_status_t agc_job_gyrocompass_azimuth_flash(agc_job_ctx_t *ctx)
{
    const agc_dsky_cont_t cont = {AGC_JOB_GYROCOMPASS_AZIMUTH_FLASH,
                                  AGC_JOB_GYROCOMPASS_AZIMUTH_LOADED,
                                  AGC_JOB_GYROCOMPASS_AZIMUTH_FLASH};

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_display(ctx, AGC_DSKY_GOFLASH, VN_LAUNCH_AZ, &cont));
}

/**********************************************************************
 * AZMTHCG1 +11 (p.454) -- LAUNCHAZ = CDULOGIC(DSPTEM1), PREMTRXC = 0
 * (RESTEST3 slews at the next torquing), group 4 off, PINBRNCH
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t of PINBRNCH
 **********************************************************************/
agc_status_t agc_job_gyrocompass_azimuth_loaded(void)
{
    prelaunch_ram_t *r = prelaunch_ram(); /* AINLA overlay */

    r->launchaz = agc_rtb_cdulogic(agc_dsky_dsptem(0)); /* STORE LAUNCHAZ */
    r->premtrxc = 0;
    agc_phase_change(4, 00004);

    return (agc_dsky_pinbrnch()); /* POSTJUMP PINBRNCH               */
}
