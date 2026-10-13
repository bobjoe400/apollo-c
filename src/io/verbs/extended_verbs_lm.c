/**********************************************************************
 * FILE NAME: extended_verbs_lm.c
 *
 * PURPOSE:
 *   The LGC's own extended verbs (LST2FAN entries that exist only in
 *   Luminary or differ from Comanche) except the radar verbs, which are in
 *   extended_verbs_lm_radar.c.
 *
 * AGC SOURCE:
 *   Luminary099 EXTENDED_VERBS.agc pp.261-296 (R03: pp.292-296)
 *
 * EXTERNAL VARIABLES:
 *   (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale     Description
 *   ------------  --------  ---  --------  -------------------------------
 *   DAPBOOLS      uint16_t  I/O  bits      FLGWRD13, DAP options (R03)
 *   DAPDATR1      uint16_t  I/O  bits      R03 display word
 *   LEMMASS       agc_sp_t  I    kg 2^16   LM mass (N47)
 *   CSMMASS       agc_sp_t  I    kg 2^16   CSM mass (N47)
 *   MASS          agc_dp_t  O    kg 2^16   DAP mass
 *   STIKSENS      agc_sp_t  O    2^0       stick sensitivity
 *   THETAD        uint16_t  I    CDU       V43 attitude error angles
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  ------------------------------------------
 *   agc_extverb_*          TESTXACT, GOPIN, ALM/END, the radar verbs
 *   agc_lm_dap_*           RESTORDB, STOPRATE, ZATTEROR (LM DAP)
 *   agc_kalc_set_rateindx  RATEINDX (KALCMANU)
 *   agc_imu_hw_set_cducmd  CDUXCMD.. (V43 ATTCK2)
 *   agc_exec_enema         GOPROG2 (V56)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ALM/END for spares and verbs not allowed now; R03 asks again for
 *   masses below an empty LM / CSM.
 *
 * NOTES:
 *   The verbs that exist only in the LGC or differ from the CMC's:
 *     V43 IMU attitude error meters (ATTCK2), V47 AGS initialization, V48
 *     load DAP data (R03: DAPDATA1..DAPDATA3, STIKLOAD, TRIMDONE), V52-V54
 *     please-mark verbs, V56 terminate tracking (TRMTRACK), V57/V58 landing
 *     radar updates, V61/V62 DAP / total attitude error, V65/V75 U,V jets
 *     during DPS burns (SNUFFER), V76/V77 minimum impulse / rate command,
 *     V80/V81 vector to update, V83 R31, V92 IMU performance test, V93
 *     RENDWFLG reset, V95 no state vector update, V96 quit integration.
 *   The rendezvous / landing radar verbs (V40 N72, V41 N72, V44, V60, V63,
 *   V78, V79, V85) are in extended_verbs_lm_radar.c.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/nav/integration.h"
#include "agc/programs/rendezvous.h"

#define WORD15                      077777u            /* one AGC word */
#define BIT3                        000004u            /* BIT3 */
#define BIT6                        000040u            /* channel 12 bit 6: error counters */
#define BIT13                       010000u            /* DAPDATR1 bit 13 */
#define BIT14                       020000u            /* DAPDATR1 bit 14 / channel 12 bit 14 */
#define OCT24                       000024u            /* BLANKET bits 3, 5                          */
#define OCT30                       000030u            /* CHAN12 bits 4,5: coarse align, zero ICDU   */
#define BIT13_14                    030000u            /* BIT13-14 */
#define OCT70000                    070000u            /* CHAN14 bits 13,14,15: drive the CDUs        */
#define BOOLSMSK                    013113u            /* DAPBOOLS bits shown in DAPDATR1            */
#define DSPLYMSK                    033113u            /* legal DAPDATR1 bits                         */
#define REVCNT                      000006u            /* BLANKET bits 2, 3                           */
#define DAPBOOLS_WORD               13                 /* DAPBOOLS = FLGWRD13                         */
#define CSMDOCKD                    010000u            /* DAPBOOLS bit 13 (CSMDKFLG)                  */
#define ACC4OR2X                    002000u            /* DAPBOOLS bit 11 (ACC4-2FL): 4-jet X         */
#define RHCSCALE                    000100u            /* DAPBOOLS bit 7 (RHCSCFLG): normal stick     */
#define PULSES                      040000u            /* DAPBOOLS bit 15 (PULSEFLG): minimum impulse */
#define RATE_BITS                   000003u            /* DAPBOOLS bits 1,2: KALCMANU rate           */
#define NORMAL_SENS                 ((agc_sp_t) 10817) /* NORMAL DEC .660214              */
#define FINE_SENS                   ((agc_sp_t) 2704)  /* FINE DEC .165054 (4 d/s)         */
#define ONE_TENTH                   ((agc_sp_t) 1638)  /* 1/10 DEC .1                      */
#define ATTSCALE                    ((agc_sp_t) 1638)  /* ATTSCALE DEC 0.1                 */
#define RATEDB_LM                   ((agc_sp_t) -218)  /* -0.6D/S, scaled at 45 d/s        */
#define RATEDB_CSM                  ((agc_sp_t) -109)  /* -0.3D/S                          */
#define MINLMD                      ((int32_t) -713)   /* DEC -2850 B-16: -2850 kg at 2^16 (rope word 76466) */
#define MINMINLM                    ((int32_t) -550)   /* DEC -2200 B-16: -2200 kg at 2^16 (rope word 76731) */
#define MINCSM                      ((int32_t) 02000)  /* BIT11: min CSM mass (9050 lb)    */
#define PRIO4                       ((agc_prio_t) 004) /* PRIO4 */
#define PRIO7                       ((agc_prio_t) 007) /* PRIO7 */
#define PRIO22                      ((agc_prio_t) 022) /* PRIO22 */
#define ATTCK_DELAY_CS              2u                 /* CAF TWO: .32 s comment, 20 ms count */
#define RENDEZVOUS_DISPLAY_DELAY_CS 2u                 /* CAF BIT2, WAITLIST                  */
#define TRIM_DELAY_CS               1u                 /* CAF BIT1, WAITLIST TRIMGIMB         */

/* ---- V43 ------------------------------------------------------------------ */

/**********************************************************************
 * AGC: none -- V43 after the load: show V43 again (EXDSPRET).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display request status
 **********************************************************************/
agc_status_t agc_job_attitude_error_load_lm(agc_job_ctx_t *ctx)
{
    return (agc_extverb_redisplay(ctx, 43, AGC_JOB_ATTITUDE_ERROR_ENABLE));
}

/**********************************************************************
 * AGC: none -- Enable the error counters, ATTCK2 in 20 ms, ENDEXT.
 *
 * RETURNS: the WAITLIST or ENDEXT status
 **********************************************************************/
agc_status_t agc_job_attitude_error_enable(void)
{
    agc_chan_or((int) AGC_CHAN_CHAN12, (uint16_t) BIT6);
    agc_status_t st = agc_waitlist_call(ATTCK_DELAY_CS, AGC_TASK_ATTITUDE_ERROR_OUTPUT); /* WAITLIST status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_endmark());
}

/**********************************************************************
 * AGC: none -- ATTCK2 (task): CDUXCMD..CDUZCMD = THETAD x 0.1 (the CDU limits excess
 * data), then drive (channel 14 bits 13-15); error counters stay enabled.
 *
 * RETURNS: AGC_STATUS_OK (TASKOVER), or the CDU command status
 **********************************************************************/
agc_status_t agc_extverb_lm_attck2(void)
{
    for (int i = 2; i >= 0; i--)
    {
        /* MP takes the CDU-format THETAD word as a ones'-complement SP.       */
        uint16_t     w  = (uint16_t) (agc_erase_thetad(i) & WORD15);                             /* THETAD word, CDU format */
        agc_sp_t     th = (agc_sp_t) (((w & 040000u) != 0u) ? (int) w - (int) WORD15 : (int) w); /* as MP reads it, half-revolutions 2^0 */
        agc_status_t st = agc_imu_hw_set_cducmd(i, agc_dp_high(agc_sp_mul(th, ATTSCALE)));       /* CDU command status */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    agc_chan_or((int) AGC_CHAN_CHAN14, (uint16_t) OCT70000);

    return (AGC_STATUS_OK);
}

/* ---- V48 R03 ---------------------------------------------------------------- */

/**********************************************************************
 * AGC: none -- GOXDSPFR display: proceed -> next, enter -> again, then BLANKET.
 *
 * ARGUMENTS:
 *   ctx    agc_job_ctx_t *  I   the running job
 *   verb   uint8_t          I   VERB (01 or 06)
 *   noun   uint8_t          I   NOUN (46, 47, 48)
 *   term   agc_job_id_t     I   job for V34
 *   next   agc_job_id_t     I   job for V33 (proceed)
 *   again  agc_job_id_t     I   job for ENTER (new data)
 *   blank  uint16_t         I   BLANKET bits after the display
 *
 * RETURNS: the display or BLANKET status
 **********************************************************************/
static agc_status_t dap_data_display(agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun, agc_job_id_t term, agc_job_id_t next, agc_job_id_t again, uint16_t blank)
{
    agc_dsky_cont_t cont = {term, next, again};
    agc_status_t    st   = agc_dsky_display(ctx, AGC_DSKY_GOMARKFR, agc_dsky_vn(verb, noun), &cont); /* display status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_blanket(blank)); /* then ENDOFJOB           */
}

/**********************************************************************
 * AGC: none -- CHKDATA1: bits 13 and 14 both zero -> force bit 13; mask out illegal
 * bits; V01N46.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display status
 **********************************************************************/
agc_status_t agc_job_dap_data_load_noun46_lm(agc_job_ctx_t *ctx)
{
    uint16_t d = agc_erase_dapdatr1(); /* DAPDATR1 */

    if ((d & BIT13_14) == 0u)
    {
        d = (uint16_t) (d + BIT13); /* FORCEONE                */
    }

    agc_erase_set_dapdatr1((uint16_t) (d & DSPLYMSK));

    return (dap_data_display(ctx, 1, 46, AGC_JOB_EXTENDED_VERB_END, AGC_JOB_DAP_DATA_LOAD_CONFIG, AGC_JOB_DAP_DATA_LOAD_NOUN46, REVCNT));
}

/**********************************************************************
 * AGC: none -- DPDAT1 (N46 proceed): APSFLAG = not bit 14; DAPBOOLS from DAPDATR1
 * (CSMDOCKD only with both bits 13, 14); MASS high word; NJETSFLG; RATEINDX;
 * STIKLOAD; then V06N47.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the RATEINDX status, or the display status
 **********************************************************************/
agc_status_t agc_job_dap_data_load_config_lm(agc_job_ctx_t *ctx)
{
    uint16_t d = agc_erase_dapdatr1(); /* DAPDATR1 as displayed */

    agc_inhint();
    agc_flag_id_t aps = (agc_flag_id_t) AGC_FLAG_APSFLAG; /* APSFLAG */

    agc_flag_put(aps, (d & BIT14) == 0u);
    uint16_t mask = ((d & BIT13_14) == BIT13_14) ? BOOLSMSK : (uint16_t) (BOOLSMSK - CSMDOCKD);         /* CSMDOCKD only with bits 13 and 14 */
    uint16_t b    = (uint16_t) ((agc_flagword_get(DAPBOOLS_WORD) & ~(uint32_t) BOOLSMSK) | (d & mask)); /* new DAPBOOLS */

    agc_flagword_set(DAPBOOLS_WORD, b);
    int32_t hi = (int32_t) agc_erase_lemmass() + (((b & CSMDOCKD) != 0u) ? (int32_t) agc_erase_csmmass() : 0); /* MASS high word, kg 2^16 */

    agc_erase_set_mass(agc_dp_pack((agc_sp_t) hi, agc_dp_low(agc_erase_mass()))); /* TS MASS */
    agc_flag_id_t nj = (agc_flag_id_t) AGC_FLAG_NJETSFLG;                         /* NJETSFLG */

    agc_flag_put(nj, (b & ACC4OR2X) == 0u);
    agc_status_t sr = agc_kalc_set_rateindx((agc_sp_t) ((b & RATE_BITS) * 2u)); /* RATEINDX status */

    /* STIKLOAD: stick sensitivity and rate deadband (divided by 10 / halved
     * when the CSM is docked).                                                */
    agc_sp_t sens   = ((b & RHCSCALE) != 0u) ? (agc_sp_t) (NORMAL_SENS + FINE_SENS) : FINE_SENS; /* STIKSENS, scaled at 1 (NORMAL / FINE) */
    agc_sp_t ratedb = RATEDB_LM;                                                                 /* -RATEDB, 45 d/s at 2^0 (-0.6 d/s; docked -0.3) */

    if ((b & CSMDOCKD) != 0u)
    {
        sens   = agc_dp_high(agc_sp_mul(sens, ONE_TENTH));
        ratedb = RATEDB_CSM;
    }

    agc_status_t ss = agc_erase_set_stiksens(sens); /* STIKSENS, -RATEDB store status */

    ss = (ss == AGC_STATUS_OK) ? agc_erase_set_neg_ratedb(ratedb) : ss; /* if OK */
    agc_relint();

    if (sr != AGC_STATUS_OK)
    {
        return (sr);
    }

    if (ss != AGC_STATUS_OK)
    {
        return (ss);
    }

    return (agc_job_dap_data_load_noun47_lm(ctx));
} /* agc_job_dap_data_load_config_lm */

/**********************************************************************
 * AGC: none -- DAPDATA2: V06N47 LM and CSM mass; V34 ends R03 via ENDR03.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display status
 **********************************************************************/
agc_status_t agc_job_dap_data_load_noun47_lm(agc_job_ctx_t *ctx)
{
    return (dap_data_display(ctx, 6, 47, AGC_JOB_DAP_DATA_LOAD_END, AGC_JOB_DAP_DATA_LOAD_MASS, AGC_JOB_DAP_DATA_LOAD_NOUN47, BIT3));
}

/**********************************************************************
 * AGC: none -- ENDR03: RESTORDB, ENDEXT.
 *
 * RETURNS: the RESTORDB or ENDEXT status
 **********************************************************************/
agc_status_t agc_job_dap_data_load_end(void)
{
    agc_inhint();
    agc_status_t st = agc_lm_dap_restordb(); /* RESTORDB status */

    agc_relint();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_endmark());
}

/**********************************************************************
 * AGC: none -- DAPDAT2 (N47 proceed): the masses must exceed an empty LM (descent stage
 * unless staged) and, docked, an empty CSM; MASS = sum; RESTORDB; V06N48
 * unless the LM has staged.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display, RESTORDB or ENDEXT status; BAD_ARG for a mass
 *          over 2^16 kg
 **********************************************************************/
agc_status_t agc_job_dap_data_load_mass_lm(agc_job_ctx_t *ctx)
{
    bool    staged = agc_flag_test((agc_flag_id_t) AGC_FLAG_APSFLAG);    /* APSFLAG */
    bool    docked = (agc_flagword_get(DAPBOOLS_WORD) & CSMDOCKD) != 0u; /* CSMDOCKD */
    int32_t lem    = (int32_t) agc_erase_lemmass();                      /* LEMMASS, kg 2^16 */
    int32_t csm    = (int32_t) agc_erase_csmmass();                      /* CSMMASS, kg 2^16 */

    if ((staged ? MINMINLM : MINLMD + MINMINLM) + lem <= 0 || (docked && csm - MINCSM <= 0)) /* staged: ascent stage only */
    {
        return (agc_job_dap_data_load_noun47_lm(ctx)); /* ask for new masses   */
    }

    int32_t m = lem + (docked ? csm : 0); /* MASS, kg 2^16 */

    if (!AGC_ASSERT(m <= AGC_SP_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_erase_set_mass(agc_dp_from_sp((agc_sp_t) m)); /* ZL, DXCH */
    agc_inhint();
    agc_status_t st = agc_lm_dap_restordb();          /* deadband, inertias      */

    agc_relint();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (staged)
    {
        return (agc_dsky_endmark());
    }

    return (agc_job_dap_data_load_noun48_lm(ctx));
}

/**********************************************************************
 * AGC: none -- DAPDATA3: V06N48 trim angles; proceed trims the gimbal.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display status
 **********************************************************************/
agc_status_t agc_job_dap_data_load_noun48_lm(agc_job_ctx_t *ctx)
{
    return (dap_data_display(ctx, 6, 48, AGC_JOB_EXTENDED_VERB_END, AGC_JOB_DAP_DATA_LOAD_TRIM, AGC_JOB_DAP_DATA_LOAD_NOUN48, BIT3));
}

/**********************************************************************
 * AGC: none -- DPDAT3: TRIMGIMB by WAITLIST (it uses FIXDELAY / VARDELAY), ENDOFJOB.
 *
 * RETURNS: the WAITLIST status
 **********************************************************************/
agc_status_t agc_job_dap_data_load_trim(void)
{
    agc_inhint();
    agc_status_t st = agc_waitlist_call(TRIM_DELAY_CS, AGC_TASK_TRIMGIMB); /* WAITLIST status */

    agc_relint();

    return (st);
}

/**********************************************************************
 * AGC: none -- TRIMDONE: "trim is finished; please terminate R03" (V50N48).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display or BLANKET status
 **********************************************************************/
agc_status_t agc_job_trimdone(agc_job_ctx_t *ctx)
{
    agc_dsky_cont_t cont = {AGC_JOB_EXTENDED_VERB_END,
                            AGC_JOB_EXTENDED_VERB_END,
                            AGC_JOB_EXTENDED_VERB_END};
    agc_status_t    st   = agc_dsky_display(ctx, AGC_DSKY_GOMARK3R, agc_dsky_vn(50, 48), &cont); /* display status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_blanket(OCT24)); /* BIT5 perform, BIT3 blank */
}

/* ---- tracking, DAP modes, misc ------------------------------------------ */

/**********************************************************************
 * TRMTRACK (V56, p.278): REND or P25 flag on -> knock down REND, P25 and
 * the search option; if TRACK was on as well, stop P20 (TRMTRAK1).
 *
 * RETURNS: the GOPIN status, or that of TRMTRAK1
 **********************************************************************/
static agc_status_t verb_terminate_tracking(void)
{
    if (!agc_flag_test((agc_flag_id_t) AGC_FLAG_RNDVZFLG) && !agc_flag_test((agc_flag_id_t) AGC_FLAG_PREFERRED_TRACKING_ATTITUDE_ACTIVE))
    {
        return (agc_extverb_gopin());
    }

    agc_flag_clear((agc_flag_id_t) AGC_FLAG_RNDVZFLG);
    agc_flag_clear((agc_flag_id_t) AGC_FLAG_PREFERRED_TRACKING_ATTITUDE_ACTIVE);
    agc_flag_clear((agc_flag_id_t) AGC_FLAG_SRCHOPTN);

    if (!agc_flag_test((agc_flag_id_t) AGC_FLAG_TRACKFLG))
    {
        return (agc_extverb_gopin());
    }

    agc_flag_clear((agc_flag_id_t) AGC_FLAG_UPDATFLG);
    agc_flag_clear((agc_flag_id_t) AGC_FLAG_TRACKFLG);
    agc_flag_clear((agc_flag_id_t) AGC_FLAG_IMUSE);

    return (agc_job_tracking_terminate_lm());
}

/**********************************************************************
 * AGC: none -- TRMTRAK1 after INTSTALL: kill group 2, STOPRATE, RESTORDB, CLRADMOD,
 * lock-on off, GOPROG2 (software restart).
 *
 * RETURNS: AGC_STATUS_OK (asleep in INTSTALL), the INTSTALL or RESTORDB
 *          status, or the ENEMA request status
 **********************************************************************/
agc_status_t agc_job_tracking_terminate_lm(void)
{
    agc_status_t st = agc_integ_install(0, AGC_JOB_TRACKING_TERMINATE); /* INTSTALL / RESTORDB status */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* sleeps in INTSTALL      */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_change(02);
    agc_inhint();
    agc_lm_dap_stoprate();
    st = agc_lm_dap_restordb();
    agc_rendz_clradmod(); /* bits 10, 15 of RADMODES */
    agc_chan_and((int) AGC_CHAN_CHAN12, (uint16_t) (WORD15 & ~BIT14));

    if (st != AGC_STATUS_OK)
    {
        agc_relint();

        return (st);
    }

    agc_exec_enema();

    return (AGC_STATUS_OK); /* GOPROG2                 */
}

/**********************************************************************
 * AGC: none -- MINIMP (V76) / NOMINIMP (V77, also ZATTEROR): DAPBOOLS PULSES bit.
 *
 * ARGUMENTS:
 *   on  bool  I   true V76 (MINIMP), false V77 (NOMINIMP)
 *
 * RETURNS: the GOPIN status
 **********************************************************************/
static agc_status_t verb_minimum_impulse(bool on)
{
    agc_inhint();
    uint16_t b = agc_flagword_get(DAPBOOLS_WORD); /* DAPBOOLS */

    b = on ? (uint16_t) (b | PULSES) : (uint16_t) (b & ~(uint32_t) PULSES); /* MINIMP / NOMINIMP */
    agc_flagword_set(DAPBOOLS_WORD, b);

    if (!on)
    {
        agc_lm_dap_zatteror(); /* IBNKCALL ZATTEROR       */
    }

    agc_relint();

    return (agc_extverb_gopin());
}

/**********************************************************************
 * AGC: none -- LEMVEC / CSMVEC (V80/V81): VEHUPFLG, and allow updates (NOUPFLAG off).
 *
 * ARGUMENTS:
 *   csm  bool  I   true V81 (CSMVEC), false V80 (LEMVEC)
 *
 * RETURNS: the GOPIN status
 **********************************************************************/
static agc_status_t verb_vector_to_update(bool csm)
{
    agc_flag_id_t f = (agc_flag_id_t) AGC_FLAG_VEHUPFLG; /* VEHUPFLG */

    agc_flag_put(f, csm);

    return (agc_extverb_downflag((agc_flag_id_t) AGC_FLAG_NOUPFLAG));
}

/**********************************************************************
 * AGC: none -- V83PERF (LM): R31CALL as a waitlist task in 20 ms.
 *
 * RETURNS: the TESTXACT or WAITLIST status
 **********************************************************************/
static agc_status_t verb_rendezvous_display(void)
{
    bool         ok = false;                     /* TESTXACT got the display */
    agc_status_t st = agc_extverb_testxact(&ok); /* TESTXACT / WAITLIST status */

    if (ok)
    {
        agc_inhint();
        st = agc_waitlist_call(RENDEZVOUS_DISPLAY_DELAY_CS, AGC_TASK_RENDEZVOUS_PARAMETER_DISPLAY);
        agc_relint();
    }

    return (st);
}

/**********************************************************************
 * AGC: none -- TESTXACT (and CHKPOOH), FINDVAC job at prio.
 *
 * ARGUMENTS:
 *   pooh  bool          I   true: CHKPOOH first
 *   prio  agc_prio_t    I   the job's priority
 *   job   agc_job_id_t  I   the job FINDVAC starts
 *
 * RETURNS: the CHKPOOH, TESTXACT or FINDVAC status
 **********************************************************************/
static agc_status_t xact_findvac(bool pooh, agc_prio_t prio, agc_job_id_t job)
{
    bool ok = false; /* TESTXACT got the display */

    bool         xv_ok = true;          /* CHKPOOH passed (or not asked) */
    agc_status_t xv_st = AGC_STATUS_OK; /* ALM/END outcome otherwise */

    if (pooh)
    {
        xv_st = agc_extverb_chkpooh(&xv_ok);
    }

    if (!xv_ok)
    {
        return (xv_st);
    }

    agc_status_t st = agc_extverb_testxact(&ok); /* TESTXACT status */

    if (!ok)
    {
        return (st);
    }

    return (agc_extverb_schedule(prio, job, true));
}

/**********************************************************************
 * AGC: none -- Verbs 40-69.
 *
 * ARGUMENTS:
 *   ctx   agc_job_ctx_t *  I   the extended-verb job
 *   verb  int              I   40..69
 *
 * RETURNS: the verb's status (ALM/END for spares)
 **********************************************************************/
static agc_status_t fan_40s_60s(agc_job_ctx_t *ctx, int verb)
{
    switch (verb)
    {
        case 40:
            return ((agc_extverb_noun() == 72) ? agc_extverb_rr_zero() : agc_extverb_alm_end());         /* N72 only */
        case 41:
            return ((agc_extverb_noun() == 72) ? agc_extverb_rr_designate(ctx) : agc_extverb_alm_end()); /* N72 only */
        case 43:
            /* IMUATTCK (V43, p.272): P00; not with coarse align or zero ICDU; the
             * attitude-indicator mode switch (channel 31 bits 13,14) must be off.     */
        {
            bool         xv_ok = false;                       /* CHKPOOH passed */
            agc_status_t xv_st = agc_extverb_chkpooh(&xv_ok); /* ALM/END outcome otherwise */

            if (!xv_ok)
            {
                return (xv_st);
            }

            if ((agc_chan_read((int) AGC_CHAN_CHAN12) & OCT30) != 0u || ((agc_chan_read((int) AGC_CHAN_CHAN31) ^ BIT13_14) & BIT13_14) != 0u)
            {
                return (agc_extverb_alm_end());
            }

            bool         ok = false;                     /* TESTXACT got the display */
            agc_status_t st = agc_extverb_testxact(&ok); /* TESTXACT status */

            if (!ok)
            {
                return (st);
            }

            agc_dsky_cont_t cont = {AGC_JOB_EXTENDED_VERB_END,
                                    AGC_JOB_ATTITUDE_ERROR_LOAD,
                                    AGC_JOB_ATTITUDE_ERROR_LOAD};

            return (agc_dsky_display(ctx, AGC_DSKY_GOMARKF, agc_dsky_vn(25, 22), &cont));
        }
        case 44:
            return (agc_extverb_rr_designate_end());
        case 60:
            return (agc_extverb_lr_position_two());
        case 63:
            return (agc_extverb_radar_test(ctx));
        case 47:
            return (xact_findvac(false, PRIO4, AGC_JOB_AGS_INITIALIZATION));
        case 48:
            /* DAPDISP (V48): TESTXACT, priority 7, DAPDATA1: DAPDATR1 from DAPBOOLS
             * with bit 14 = not APSFLAG.                                              */
        {
            bool         ok = false;                     /* TESTXACT got the display */
            agc_status_t st = agc_extverb_testxact(&ok); /* TESTXACT status */

            if (!ok)
            {
                return (st);
            }

            agc_exec_prio_change(PRIO7);
            uint16_t d = (uint16_t) (agc_flagword_get(DAPBOOLS_WORD) & BOOLSMSK); /* DAPDATR1 from DAPBOOLS */

            if (!agc_flag_test((agc_flag_id_t) AGC_FLAG_APSFLAG))
            {
                d = (uint16_t) (d + BIT14);
            }

            agc_erase_set_dapdatr1(d);

            return (agc_job_dap_data_load_noun46_lm(ctx));
        }
        case 52:
        case 53:
        case 54:
            return (agc_extverb_goloadlv());
        case 56:
            return (verb_terminate_tracking());
        case 57:
            return (agc_extverb_upflag((agc_flag_id_t) AGC_FLAG_LRINH));
        case 58:
            return (agc_extverb_downflag((agc_flag_id_t) AGC_FLAG_LRINH));
        case 61:
            return (agc_extverb_downflag((agc_flag_id_t) AGC_FLAG_NEEDLFLG));
        case 62:
            return (agc_extverb_upflag((agc_flag_id_t) AGC_FLAG_NEEDLFLG));
        case 65:
            return (agc_extverb_upflag((agc_flag_id_t) AGC_FLAG_SNUFFER));
        default:
            break;
    } /* switch (verb) */

    return (agc_extverb_alm_end()); /* V45, V46, V51, V59, V68 */
} /* fan_40s_60s */

/**********************************************************************
 * AGC: none -- Verbs 75-98.
 *
 * ARGUMENTS:
 *   ctx   agc_job_ctx_t *  I   the extended-verb job
 *   verb  int              I   70..99
 *
 * RETURNS: the verb's status (ALM/END for spares)
 **********************************************************************/
static agc_status_t fan_70s_90s(agc_job_ctx_t *ctx, int verb)
{
    switch (verb)
    {
        case 75:
            return (agc_extverb_downflag((agc_flag_id_t) AGC_FLAG_SNUFFER));
        case 76:
            return (verb_minimum_impulse(true));
        case 77:
            return (verb_minimum_impulse(false));
        case 78:
            return (agc_extverb_lr_spurious_test(ctx));
        case 79:
            return (agc_extverb_lr_spurious_test_end());
        case 85:
            return (agc_extverb_rr_los_display(ctx));
        case 80:
            return (verb_vector_to_update(false));
        case 81:
            return (verb_vector_to_update(true));
        case 83:
            return (verb_rendezvous_display());
        case 92:
            return (xact_findvac(true, PRIO22, AGC_JOB_SYSTEMS_TEST));
        case 93:
            /* WMATRXNG (V93): INHINT, RENDWFLG off.                                    */
        {
            agc_inhint();
            agc_flag_clear((agc_flag_id_t) AGC_FLAG_RENDWFLG);
            agc_relint();

            return (agc_extverb_gopin());
        }
        case 95:
            return (agc_extverb_upflag((agc_flag_id_t) AGC_FLAG_NOUPFLAG));
        case 96:
            /* VERB96: QUITFLAG, V37 to P00.                                           */
        {
            agc_flag_set((agc_flag_id_t) AGC_FLAG_QUITFLAG);

            return (agc_change_program(0));
        }
        default:
            break;
    } /* switch (verb) */

    return (agc_extverb_alm_end()); /* V84, V86-88, V94, V98  */
} /* fan_70s_90s */

/**********************************************************************
 * AGC: none -- LST2FAN, LGC entries not shared with the CMC.
 *
 * ARGUMENTS:
 *   ctx   agc_job_ctx_t *  I   the extended-verb job
 *   verb  int              I   40..99
 *
 * RETURNS: the verb's status, BAD_ARG for a verb out of range
 **********************************************************************/
agc_status_t agc_extverb_lm(agc_job_ctx_t *ctx, int verb)
{
    if (!AGC_ASSERT(verb >= 40 && verb <= 99))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (verb < 70)
    {
        return (fan_40s_60s(ctx, verb));
    }

    return (fan_70s_90s(ctx, verb));
}
