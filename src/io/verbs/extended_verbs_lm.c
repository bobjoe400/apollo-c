/*
 * extended_verbs_lm.c -- Luminary099 EXTENDED_VERBS.agc (pp. 261-296),
 * the verbs that exist only in the LGC or differ from the CMC's:
 *   V43 IMU attitude error meters (ATTCK2), V47 AGS initialization, V48
 *   load DAP data (R03: DAPDATA1..DAPDATA3, STIKLOAD, TRIMDONE), V52-V54
 *   please-mark verbs, V56 terminate tracking (TRMTRACK), V57/V58 landing
 *   radar updates, V61/V62 DAP / total attitude error, V65/V75 U,V jets
 *   during DPS burns (SNUFFER), V76/V77 minimum impulse / rate command,
 *   V80/V81 vector to update, V83 R31, V92 IMU performance test, V93
 *   RENDWFLG reset, V95 no state vector update, V96 quit integration.
 * The rendezvous / landing radar verbs (V40 N72, V41 N72, V44, V60, V63,
 * V78, V79, V85) are in extended_verbs_lm_radar.c.
 */
#include <stddef.h>

#include "agc/core/assert.h"
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

#define WORD15                      077777u
#define BIT3                        000004u
#define BIT6                        000040u
#define BIT13                       010000u
#define BIT14                       020000u
#define OCT24                       000024u            /* BLANKET bits 3, 5                          */
#define OCT30                       000030u            /* CHAN12 bits 4,5: coarse align, zero ICDU   */
#define BIT13_14                    030000u
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
#define MINLMD                      ((int32_t) -2850)  /* min descent stage mass, kg B16   */
#define MINMINLM                    ((int32_t) -2200)  /* min ascent stage mass, kg B16    */
#define MINCSM                      ((int32_t) 02000)  /* BIT11: min CSM mass (9050 lb)    */
#define PRIO4                       ((agc_prio_t) 004)
#define PRIO7                       ((agc_prio_t) 007)
#define PRIO22                      ((agc_prio_t) 022)
#define ATTCK_DELAY_CS              2u                 /* CAF TWO: .32 s comment, 20 ms count */
#define RENDEZVOUS_DISPLAY_DELAY_CS 2u                 /* CAF BIT2, WAITLIST                  */
#define TRIM_DELAY_CS               1u                 /* CAF BIT1, WAITLIST TRIMGIMB         */

/* ---- V43 ------------------------------------------------------------------ */

/* V43 after the load: show V43 again (EXDSPRET).                          */
agc_status_t agc_job_attitude_error_load_lm(agc_job_ctx_t *ctx)
{
    return (agc_extverb_redisplay(ctx, 43, AGC_JOB_ATTITUDE_ERROR_ENABLE));
}

/* Enable the error counters, ATTCK2 in 20 ms, ENDEXT.                     */
agc_status_t agc_job_attitude_error_enable(agc_job_ctx_t *ctx)
{
    agc_chan_or((int) AGC_CHAN_CHAN12, (uint16_t) BIT6);
    agc_status_t st = agc_waitlist_call(ATTCK_DELAY_CS, AGC_TASK_ATTITUDE_ERROR_OUTPUT);

    if (!AGC_ASSERT(ctx != NULL) || st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_endmark());
}

/* ATTCK2 (task): CDUXCMD..CDUZCMD = THETAD x 0.1 (the CDU limits excess
 * data), then drive (channel 14 bits 13-15); error counters stay enabled. */
agc_status_t agc_extverb_lm_attck2(void)
{
    for (int i = 2; i >= 0; i--)
    {
        uint16_t w = agc_erase_thetad(i);
        int      v = ((w & 040000u) != 0u) ? (int) w - 0100000 : (int) w;

        if (!AGC_ASSERT(v >= -040000 && v < 040000))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        agc_sp_t     th = (agc_sp_t) ((v < AGC_SP_MIN) ? AGC_SP_MIN : v);
        agc_status_t st = agc_imu_hw_set_cducmd(i, agc_dp_high(agc_sp_mul(th, ATTSCALE)));

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    agc_chan_or((int) AGC_CHAN_CHAN14, (uint16_t) OCT70000);

    return (AGC_STATUS_OK);
}

/* ---- V48 R03 ---------------------------------------------------------------- */

/* GOXDSPFR display: proceed -> next, enter -> again, then BLANKET.       */
static agc_status_t dap_data_display(agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun, agc_job_id_t term, agc_job_id_t next, agc_job_id_t again, uint16_t blank)
{
    agc_dsky_cont_t cont = {term, next, again};
    agc_status_t    st   = agc_dsky_display(ctx, AGC_DSKY_GOMARKFR, agc_dsky_vn(verb, noun), &cont);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_blanket(blank)); /* then ENDOFJOB           */
}

/* CHKDATA1: bits 13 and 14 both zero -> force bit 13; mask out illegal
 * bits; V01N46.                                                            */
agc_status_t agc_job_dap_data_load_noun46_lm(agc_job_ctx_t *ctx)
{
    uint16_t d = agc_erase_dapdatr1();

    if ((d & BIT13_14) == 0u)
    {
        d = (uint16_t) (d + BIT13); /* FORCEONE                */
    }

    agc_erase_set_dapdatr1((uint16_t) (d & DSPLYMSK));

    return (dap_data_display(ctx, 1, 46, AGC_JOB_EXTENDED_VERB_END, AGC_JOB_DAP_DATA_LOAD_CONFIG, AGC_JOB_DAP_DATA_LOAD_NOUN46, REVCNT));
}

/* DPDAT1 (N46 proceed): APSFLAG = not bit 14; DAPBOOLS from DAPDATR1
 * (CSMDOCKD only with both bits 13, 14); MASS high word; NJETSFLG; RATEINDX;
 * STIKLOAD; then V06N47.                                                   */
agc_status_t agc_job_dap_data_load_config_lm(agc_job_ctx_t *ctx)
{
    uint16_t d = agc_erase_dapdatr1();

    agc_inhint();
    agc_flag_id_t aps = (agc_flag_id_t) AGC_FLAG_APSFLAG;

    agc_flag_put(aps, (d & BIT14) == 0u);
    uint16_t mask = ((d & BIT13_14) == BIT13_14) ? BOOLSMSK : (uint16_t) (BOOLSMSK - CSMDOCKD);
    uint16_t b    = (uint16_t) ((agc_flagword_get(DAPBOOLS_WORD) & ~(uint32_t) BOOLSMSK) | (d & mask));

    agc_flagword_set(DAPBOOLS_WORD, b);
    agc_guid_lm_t *g  = &agc_guid_state()->lm;
    int32_t        hi = (int32_t) agc_erase_lemmass() + (((b & CSMDOCKD) != 0u) ? (int32_t) agc_erase_csmmass() : 0);

    g->mass          = agc_dp_pack((agc_sp_t) hi, agc_dp_low(g->mass)); /* TS MASS */
    agc_flag_id_t nj = (agc_flag_id_t) AGC_FLAG_NJETSFLG;

    agc_flag_put(nj, (b & ACC4OR2X) == 0u);
    agc_status_t sr = agc_kalc_set_rateindx((agc_sp_t) ((b & RATE_BITS) * 2u));

    /* STIKLOAD: stick sensitivity and rate deadband (divided by 10 / halved
     * when the CSM is docked).                                                */
    agc_sp_t sens   = ((b & RHCSCALE) != 0u) ? (agc_sp_t) (NORMAL_SENS + FINE_SENS) : FINE_SENS;
    agc_sp_t ratedb = RATEDB_LM;

    if ((b & CSMDOCKD) != 0u)
    {
        sens   = agc_dp_high(agc_sp_mul(sens, ONE_TENTH));
        ratedb = RATEDB_CSM;
    }

    agc_erase_set_stiksens(sens);
    agc_erase_set_neg_ratedb(ratedb);
    agc_relint();

    if (!AGC_ASSERT(ctx != NULL) || sr != AGC_STATUS_OK)
    {
        return ((sr != AGC_STATUS_OK) ? sr : AGC_STATUS_BAD_ARG);
    }

    return (agc_job_dap_data_load_noun47_lm(ctx));
}

/* DAPDATA2: V06N47 LM and CSM mass; V34 ends R03 via ENDR03.             */
agc_status_t agc_job_dap_data_load_noun47_lm(agc_job_ctx_t *ctx)
{
    return (dap_data_display(ctx, 6, 47, AGC_JOB_DAP_DATA_LOAD_END, AGC_JOB_DAP_DATA_LOAD_MASS, AGC_JOB_DAP_DATA_LOAD_NOUN47, BIT3));
}

/* ENDR03: RESTORDB, ENDEXT.                                                */
agc_status_t agc_job_dap_data_load_end(void)
{
    agc_inhint();
    agc_status_t st = agc_lm_dap_restordb();

    agc_relint();

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_endmark());
}

/* DAPDAT2 (N47 proceed): the masses must exceed an empty LM (descent stage
 * unless staged) and, docked, an empty CSM; MASS = sum; RESTORDB; V06N48
 * unless the LM has staged.                                               */
agc_status_t agc_job_dap_data_load_mass_lm(agc_job_ctx_t *ctx)
{
    bool    staged = agc_flag_test((agc_flag_id_t) AGC_FLAG_APSFLAG);
    bool    docked = (agc_flagword_get(DAPBOOLS_WORD) & CSMDOCKD) != 0u;
    int32_t lem    = (int32_t) agc_erase_lemmass();
    int32_t csm    = (int32_t) agc_erase_csmmass();

    if ((staged ? MINMINLM : MINLMD + MINMINLM) + lem <= 0 || (docked && csm - MINCSM <= 0))
    {
        return (agc_job_dap_data_load_noun47_lm(ctx)); /* ask for new masses   */
    }

    int32_t m = lem + (docked ? csm : 0);

    if (!AGC_ASSERT(m <= AGC_SP_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_state()->lm.mass = agc_dp_from_sp((agc_sp_t) m); /* ZL, DXCH */
    agc_inhint();
    agc_status_t st           = agc_lm_dap_restordb();        /* deadband, inertias      */

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

/* DAPDATA3: V06N48 trim angles; proceed trims the gimbal.                  */
agc_status_t agc_job_dap_data_load_noun48_lm(agc_job_ctx_t *ctx)
{
    return (dap_data_display(ctx, 6, 48, AGC_JOB_EXTENDED_VERB_END, AGC_JOB_DAP_DATA_LOAD_TRIM, AGC_JOB_DAP_DATA_LOAD_NOUN48, BIT3));
}

/* DPDAT3: TRIMGIMB by WAITLIST (it uses FIXDELAY / VARDELAY), ENDOFJOB.   */
agc_status_t agc_job_dap_data_load_trim(void)
{
    agc_inhint();
    agc_status_t st = agc_waitlist_call(TRIM_DELAY_CS, AGC_TASK_TRIMGIMB);

    agc_relint();

    return (st);
}

/* TRIMDONE: "trim is finished; please terminate R03" (V50N48).           */
agc_status_t agc_job_trimdone(agc_job_ctx_t *ctx)
{
    agc_dsky_cont_t cont = {AGC_JOB_EXTENDED_VERB_END,
                            AGC_JOB_EXTENDED_VERB_END,
                            AGC_JOB_EXTENDED_VERB_END};
    agc_status_t    st   = agc_dsky_display(ctx, AGC_DSKY_GOMARK3R, agc_dsky_vn(50, 48), &cont);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_blanket(OCT24)); /* BIT5 perform, BIT3 blank */
}

/* ---- tracking, DAP modes, misc ------------------------------------------ */

/* TRMTRACK (V56, p.278): REND or P25 flag on -> knock down REND, P25 and
 * the search option; if TRACK was on as well, stop P20 (TRMTRAK1).       */
static agc_status_t verb_terminate_tracking(agc_job_ctx_t *ctx)
{
    if (!agc_flag_test((agc_flag_id_t) AGC_FLAG_RNDVZFLG) && !agc_flag_test((agc_flag_id_t) AGC_FLAG_PREFERRED_TRACKING_ATTITUDE_ACTIVE))
    {
        return (agc_extverb_gopin());
    }

    agc_flag_clear((agc_flag_id_t) AGC_FLAG_RNDVZFLG);
    agc_flag_clear((agc_flag_id_t) AGC_FLAG_PREFERRED_TRACKING_ATTITUDE_ACTIVE);
    agc_flag_clear((agc_flag_id_t) AGC_FLAG_SRCHOPTN);

    if (!AGC_ASSERT(ctx != NULL) || !agc_flag_test((agc_flag_id_t) AGC_FLAG_TRACKFLG))
    {
        return (agc_extverb_gopin());
    }

    agc_flag_clear((agc_flag_id_t) AGC_FLAG_UPDATFLG);
    agc_flag_clear((agc_flag_id_t) AGC_FLAG_TRACKFLG);
    agc_flag_clear((agc_flag_id_t) AGC_FLAG_IMUSE);

    return (agc_job_tracking_terminate_lm(ctx));
}

/* TRMTRAK1 after INTSTALL: kill group 2, STOPRATE, RESTORDB, CLRADMOD,
 * lock-on off, GOPROG2 (software restart).                                */
agc_status_t agc_job_tracking_terminate_lm(agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_integ_install(0, AGC_JOB_TRACKING_TERMINATE);

    if (!AGC_ASSERT(ctx != NULL) || st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK);
    }

    agc_phase_change(02);
    agc_inhint();
    agc_lm_dap_stoprate();
    agc_status_t s2 = agc_lm_dap_restordb();

    agc_rendz_clradmod(); /* bits 10, 15 of RADMODES */
    agc_chan_and((int) AGC_CHAN_CHAN12, (uint16_t) (WORD15 & ~BIT14));
    st = AGC_STATUS_OK;

    if (!AGC_ASSERT(s2 == AGC_STATUS_OK))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_enema()); /* GOPROG2                 */
}

/* MINIMP (V76) / NOMINIMP (V77, also ZATTEROR): DAPBOOLS PULSES bit.      */
static agc_status_t verb_minimum_impulse(bool on)
{
    agc_inhint();
    uint16_t b = agc_flagword_get(DAPBOOLS_WORD);

    b               = on ? (uint16_t) (b | PULSES) : (uint16_t) (b & ~(uint32_t) PULSES);
    agc_flagword_set(DAPBOOLS_WORD, b);
    agc_status_t st = AGC_STATUS_OK;

    if (st == AGC_STATUS_OK && !on)
    {
        agc_lm_dap_zatteror();
        st = AGC_STATUS_OK;
    }

    agc_relint();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_extverb_gopin());
}

/* LEMVEC / CSMVEC (V80/V81): VEHUPFLG, and allow updates (NOUPFLAG off). */
static agc_status_t verb_vector_to_update(bool csm)
{
    agc_flag_id_t f = (agc_flag_id_t) AGC_FLAG_VEHUPFLG;

    agc_flag_put(f, csm);

    return (agc_extverb_downflag((agc_flag_id_t) AGC_FLAG_NOUPFLAG));
}

/* V83PERF (LM): R31CALL as a waitlist task in 20 ms.                      */
static agc_status_t verb_rendezvous_display(void)
{
    bool         ok = false;
    agc_status_t st = agc_extverb_testxact(&ok);

    if (ok)
    {
        agc_inhint();
        st = agc_waitlist_call(RENDEZVOUS_DISPLAY_DELAY_CS, AGC_TASK_RENDEZVOUS_PARAMETER_DISPLAY);
        agc_relint();
    }

    return (st);
}

/* TESTXACT (and CHKPOOH), FINDVAC job at prio.                            */
static agc_status_t xact_findvac(bool pooh, agc_prio_t prio, agc_job_id_t job)
{
    bool ok = false;

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

    agc_status_t st = agc_extverb_testxact(&ok);

    if (!ok)
    {
        return (st);
    }

    return (agc_extverb_schedule(prio, job, true));
}

/* Verbs 40-69.                                                             */
static agc_status_t fan_40s_60s(agc_job_ctx_t *ctx, int verb)
{
    switch (verb)
    {
        case 40:
            return ((agc_extverb_noun() == 72) ? agc_extverb_rr_zero(ctx) : agc_extverb_alm_end());
        case 41:
            return ((agc_extverb_noun() == 72) ? agc_extverb_rr_designate(ctx) : agc_extverb_alm_end());
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

                bool         ok = false;
                agc_status_t st = agc_extverb_testxact(&ok);

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
            return (agc_extverb_lr_position_two(ctx));
        case 63:
            return (agc_extverb_radar_test(ctx));
        case 47:
            return (xact_findvac(false, PRIO4, AGC_JOB_AGS_INITIALIZATION));
        case 48:
            /* DAPDISP (V48): TESTXACT, priority 7, DAPDATA1: DAPDATR1 from DAPBOOLS
             * with bit 14 = not APSFLAG.                                              */
            {
                bool         ok = false;
                agc_status_t st = agc_extverb_testxact(&ok);

                if (!AGC_ASSERT(ctx != NULL) || !ok)
                {
                    return (st);
                }

                agc_exec_prio_change(PRIO7);
                uint16_t d = (uint16_t) (agc_flagword_get(DAPBOOLS_WORD) & BOOLSMSK);

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
            return (verb_terminate_tracking(ctx));
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
    }

    return (agc_extverb_alm_end()); /* V45, V46, V51, V59, V68 */
}

/* Verbs 75-98.                                                             */
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
    }

    if (!AGC_ASSERT(verb >= 70 && verb <= 99))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_extverb_alm_end()); /* V84, V86-88, V94, V98  */
}

/* LST2FAN, LGC entries not shared with the CMC.                            */
agc_status_t agc_extverb_lm(agc_job_ctx_t *ctx, int verb)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(verb >= 40 && verb <= 99))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (verb < 70)
    {
        return (fan_40s_60s(ctx, verb));
    }

    return (fan_70s_90s(ctx, verb));
}
