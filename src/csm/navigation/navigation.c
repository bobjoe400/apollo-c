/*
 * navigation.c -- P20 RENDEZVOUS NAVIGATION, CSM (P20-P25.agc,
 * Comanche055 pp.562-564, 572-573, 586), with the R21CSM / R23CSM marking
 * verbs (V57 / V54), SETINTG, the CNTCHK / WAITONE / REDOR22 wait of R22,
 * and the LANDMARK erasable. Also the vehicle dispatch of the P20 / P22
 * entries shared with the LM (PROG22 = PROG20 in the LM).
 *
 * "To control the CSM attitude and optics to acquire the LEM in the S+T
 * field and to point the CSM transponder at the LEM. To update either
 * the LEM or CSM state vector (as specified by the astronaut by the DSKY
 * entry) on the basis of optical tracking data."
 *
 * Scales: t cs 2^28; mark buffers raw words.
 */
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/core/state.h"
#include "agc/csm/dap.h"
#include "agc/imu/imu.h"
#include "agc/programs/alignment.h"

#define NC_PRIO26       ((agc_prio_t) 026)
#define NC_PRIO14       ((agc_prio_t) 014)
#define NC_4SECS        400U    /* 4SECS DEC 400              */
#define NC_MRKBUF_EMPTY 077776u /* -1: no mark in the buffer  */
#define NC_MRK_WORDS    7

static uint16_t     nc_landmark; /* LANDMARK (P22, V52)        */
static agc_dp_t     nc_marktime; /* MARKTIME: cs 2^28          */
static agc_job_id_t nc_pointex;  /* POINTEX: WAITONE return    */

/* ---- vehicle dispatch --------------------------------------------------- */
/* P20 (V37E20E): the CSM or the LM program.                              */
agc_status_t agc_job_rendezvous_navigation(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_is_csm() ? agc_job_rendezvous_navigation_csm(ctx) :
                         agc_job_rendezvous_navigation_lm(ctx);
}

/* P22 (V37E22E): CSM orbital navigation; LM lunar surface navigation is
 * the P20 code (PROG22 = PROG20).                                       */
agc_status_t agc_job_orbital_navigation(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_is_csm() ? agc_job_orbital_navigation_csm(ctx) :
                         agc_job_rendezvous_navigation_lm(ctx);
}

/* ---- LANDMARK (P22 landmark code ABCDE, V52 offset) -------------------- */
agc_sp_t agc_erase_landmark(void)
{
    (void) AGC_ASSERT(nc_landmark <= 077777u);
    return (agc_sp_t) rz_word_value(nc_landmark);
}

void agc_erase_set_landmark(agc_sp_t v)
{
    if (!AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX))
    {
        return;
    }
    nc_landmark = (v >= 0) ? (uint16_t) v : (uint16_t) (077777 + v);
}

agc_dp_t rz_marktime(void)
{
    return nc_marktime;
}

void rz_set_marktime(agc_dp_t t)
{
    (void) AGC_ASSERT(t >= 0);
    nc_marktime = t;
}

/* ---- SETINTG (p.586): INTSTALL, TDEC1 = MARKTIME, STATEFLG, precision,
 * no W, the CSM, W 6x6. BUSY: asleep in INTSTALL.                      */
agc_status_t rz_setintg(agc_job_id_t resume)
{
    agc_status_t st = agc_integ_install(0, resume);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    rz_flag_put(AGC_FLAG_STATEFLG, true);
    rz_flag_put(AGC_FLAG_INTYPFLG, false);
    rz_flag_put(AGC_FLAG_DIM0FLAG, false);
    rz_flag_put(AGC_FLAG_VINTFLAG, true);
    rz_flag_put(AGC_FLAG_D6OR9FLG, false);
    return AGC_STATUS_OK;
}

/* ---- WAITONE / CONTCHK / REDOR22 (p.586) --------------------------------
 * Wait 4 s; then end the job if REFSMMAT is bad, wait again while
 * KALCMANU runs (R61CNTR < 0), return (POINTEX) when updating is on,
 * end when tracking is off, else (tracking without update) wait again
 * at priority 26 (REDOR22).                                             */
agc_status_t rz_waitone(agc_job_id_t pointex)
{
    if (!AGC_ASSERT(pointex != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    nc_pointex = pointex;
    return rz_delayjob(NC_4SECS, AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_CHECK);
}

agc_status_t agc_job_rendezvous_navigation_csm_check(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!agc_flag_test(AGC_FLAG_REFSMFLG))
    {
        return AGC_STATUS_OK; /* ENDPLAC  */
    }
    if (agc_erase_r61cntr() < 0)
    {
        return rz_delayjob(NC_4SECS, AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_CHECK);
    }
    if (agc_flag_test(AGC_FLAG_UPDATFLG))
    {
        return rz_continue(nc_pointex);
    }
    if (!agc_flag_test(AGC_FLAG_TRACKFLG))
    {
        return AGC_STATUS_OK; /* ENDPLAC  */
    }
    rz_phase(2, 00132); /* REDOR22  */
    agc_status_t st = agc_exec_prio_change(NC_PRIO26);
    if (st != AGC_STATUS_OK && st != AGC_STATUS_BUSY)
    {
        return st;
    }
    return rz_delayjob(NC_4SECS, AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_CHECK);
}

/* ---- PROG20 (pp.562-564) ----------------------------------------------- */
/* PROG20: IMU check; mark counters zero; preferred tracking attitude; the
 * LM state to be updated; TRACK, UPDATE, RENDEZVOUS flags on; restart at
 * priority 26 at the integration.                                       */
agc_status_t agc_job_rendezvous_navigation_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_imu_status_check(); /* R02BOTH  */
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_erase_set_trkmkcnt(0);
    agc_erase_set_vhfcnt(0);
    rz_flag_put(AGC_FLAG_PRFTRKAT, true);
    rz_flag_put(AGC_FLAG_VEHUPFLG, false);
    rz_flag_put(AGC_FLAG_TRACKFLG, true);
    rz_flag_put(AGC_FLAG_UPDATFLG, true);
    rz_flag_put(AGC_FLAG_RNDVZFLG, true);
    rz_phase_job(04U, 05022U, AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_INTEGRATE, 026);
    return agc_job_rendezvous_navigation_csm_integrate(ctx);
}

/* Both state vectors to now: the vehicle to be updated with its W matrix
 * when valid, then the other one; the R22 job; PIKUP20. DEVIATION: after
 * an INTSTALL sleep MARKTIME is taken again (the AGC keeps TDEC1).     */
agc_status_t agc_job_rendezvous_navigation_csm_integrate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    nc_marktime      = rz_loadtime();
    bool         csm = agc_flag_test(AGC_FLAG_VEHUPFLG);
    agc_status_t st  = rz_setintg(AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_INTEGRATE);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    rz_flag_put(AGC_FLAG_DIM0FLAG, agc_flag_test(AGC_FLAG_RENDWFLG));
    rz_flag_put(AGC_FLAG_VINTFLAG, csm); /* P20.1    */
    st = agc_integ_integrv(&ip, agc_integ_shared_ctx(), nc_marktime);
    rz_phase(2, 04022); /* GRP2PC   */
    if (st == AGC_STATUS_OK)
    {
        st = rz_setintg(AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_INTEGRATE);
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    rz_flag_put(AGC_FLAG_VINTFLAG, !csm); /* P20.3    */
    st = agc_integ_integrv(&ip, agc_integ_shared_ctx(), nc_marktime);
    if (st == AGC_STATUS_OK)
    {
        st = agc_exec_findvac(NC_PRIO26, AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM);
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    rz_phase(2, 00072);
    rz_phase(1, 00111);
    return agc_job_rendezvous_navigation_csm_track(ctx);
}

/* PIKUP20: priority 14 (above LAMBERT); with tracking on and REFSMMAT
 * good, R61CSM (counter zero), then R52.                                */
agc_status_t agc_job_rendezvous_navigation_csm_track(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_exec_prio_change(NC_PRIO14);
    if (st == AGC_STATUS_BUSY)
    {
        return rz_continue(AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_TRACK);
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    if (!agc_flag_test(AGC_FLAG_TRACKFLG) || !agc_flag_test(AGC_FLAG_REFSMFLG))
    {
        return AGC_STATUS_OK; /* ENDOFJOB */
    }
    agc_erase_set_r61cntr(0);
    return agc_rendz_tracking_attitude_csm(
        ctx, AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_OPTICS);
}

/* P20R52JB: the LM as target; R52 (auto optics positioning).           */
agc_status_t agc_job_rendezvous_navigation_csm_optics(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_flag_put(AGC_FLAG_TARG1FLG, true);
    return agc_csm_auto_optics_positioning(
        ctx, AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_OPTICS_DONE);
}

/* After R52: MKRLEES; HOLDFLAG = 1 (hold the present attitude).         */
agc_status_t agc_job_rendezvous_navigation_csm_optics_done(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_sxtmark_mkreleas();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_csm_dap_rcs()->holdflag = 1;
    return AGC_STATUS_OK; /* ENDOFJOB */
}

/* ---- R21CSM / R23CSM (pp.572-573) ---------------------------------------
 * "To perform sighting marks in conjunction with the rendezvous
 * navigation program. Called by astronaut via extended verb." MARKRUPT
 * (R21MARK) keeps each mark in MRKBUF1; PROCEED moves the last one to
 * MRKBUF2 for R22. R23 (R23FLG): backup marks with the COAS, the optics
 * angles keyed in on noun 94, the CDUs read at ENTER.                  */
static void nc_mrkbuf_copy(void)
{
    for (int i = 0; i < NC_MRK_WORDS; i++)
    { /* GENTRAN  */
        agc_erase_set_mrkbuf(2, i, agc_erase_mrkbuf(1, i));
    }
    (void) AGC_ASSERT(agc_erase_mrkbuf(2, 0) == agc_erase_mrkbuf(1, 0));
}

/* V57: R21MARK on, then R23CSM.                                          */
agc_status_t agc_job_rendezvous_sighting_mark_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_flag_put(AGC_FLAG_MARKRUPT_OPTION_ONE, true); /* R21MARK  */
    return agc_job_backup_sighting_mark(ctx);
}

/* V54 (R23CSM): both buffers empty; R23 marking: V06N94 (optics angles)
 * first; else V51 "please mark".                                        */
agc_status_t agc_job_backup_sighting_mark(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_erase_set_mrkbuf(1, 0, NC_MRKBUF_EMPTY);
    agc_erase_set_mrkbuf(2, 0, NC_MRKBUF_EMPTY);
    if (!agc_flag_test(AGC_FLAG_BACKUP_MARKING))
    {
        return agc_job_rendezvous_sighting_mark_request(ctx); /* R21C1    */
    }
    return rz_flash(
        ctx, AGC_DSKY_GOMARKF, 6U, 94U, AGC_JOB_RENDEZVOUS_SIGHTING_MARK_END,
        AGC_JOB_BACKUP_SIGHTING_MARK_REQUEST, AGC_JOB_BACKUP_SIGHTING_MARK);
}

/* R21C1: V51 (GOMARK2): PROCEED ends the marking, ENTER again.          */
agc_status_t agc_job_rendezvous_sighting_mark_request(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_flash(ctx, AGC_DSKY_GOMARK2, 51U, 0U,
                    AGC_JOB_RENDEZVOUS_SIGHTING_MARK_END,
                    AGC_JOB_RENDEZVOUS_SIGHTING_MARK_DONE,
                    AGC_JOB_RENDEZVOUS_SIGHTING_MARK_REQUEST);
}

/* R23CSM1: V53 (alternate LOS mark, GOMARK2); ENTER is the mark.        */
agc_status_t agc_job_backup_sighting_mark_request(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_flash(ctx, AGC_DSKY_GOMARK2, 53U, 0U,
                    AGC_JOB_RENDEZVOUS_SIGHTING_MARK_END,
                    AGC_JOB_RENDEZVOUS_SIGHTING_MARK_DONE,
                    AGC_JOB_BACKUP_SIGHTING_MARK_TAKEN);
}

/* R23 mark: MRKBUF1 to MRKBUF2, then time and the IMU CDUs into MRKBUF1
 * (the optics words stay as keyed in).                                  */
agc_status_t agc_job_backup_sighting_mark_taken(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    nc_mrkbuf_copy();
    uint32_t t = (uint32_t) rz_loadtime();
    agc_erase_set_mrkbuf(1, 0, (uint16_t) ((t >> 14) & 037777u)); /* TIME2 */
    agc_erase_set_mrkbuf(1, 1, (uint16_t) (t & 037777u));         /* TIME1 */
    agc_erase_set_mrkbuf(1, 2, agc_erase_cduy());
    agc_erase_set_mrkbuf(1, 4, agc_erase_cduz());
    agc_erase_set_mrkbuf(1, 6, agc_erase_cdux());
    agc_relint();
    return agc_job_backup_sighting_mark_request(ctx);
}

/* R21CSMA: a mark in MRKBUF1 goes to MRKBUF2; then R21END.              */
agc_status_t agc_job_rendezvous_sighting_mark_done(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_erase_mrkbuf(1, 0) != NC_MRKBUF_EMPTY)
    {
        agc_inhint();
        nc_mrkbuf_copy();
        agc_relint();
    }
    return agc_job_rendezvous_sighting_mark_end(ctx);
}

/* R21END: KLEENEX, R21MARK off, ENDMARK.                                 */
agc_status_t agc_job_rendezvous_sighting_mark_end(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_dsky_display(ctx, AGC_DSKY_KLEENEX, 0U, NULL);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    rz_flag_put(AGC_FLAG_MARKRUPT_OPTION_ONE, false);
    return agc_dsky_endmark();
}

/* ---- LLASRD / LLASRDA (p.610) -------------------------------------------
 * LLASRD: "modify ALT and store LAT to LAT+5 in LANDLAT to LANDLAT+5
 * prior to display": LANDALT = ALT, LANDLONG = LONG SR1 (LANDLAT EQUALS
 * LAT). LLASRDA, after the crew loaded noun 89: ALT = LANDALT, LONG =
 * LANDLONG SL1.                                                          */
agc_status_t agc_rendz_llasrd(void)
{
    agc_latlong_t ll;
    agc_status_t  st = agc_navaux_get_latlong(&ll);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_align_ram_t *a = agc_align_ram();
    a->landlat         = ll.lat;
    a->landalt         = ll.alt;
    a->landlong        = agc_dp_shr(ll.lon, 1);
    return AGC_STATUS_OK;
}

agc_status_t agc_rendz_llasrda(void)
{
    agc_latlong_t ll;
    agc_status_t  st = agc_navaux_get_latlong(&ll);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_align_ram_t *a = agc_align_ram();
    agc_interp_t     ip;
    agc_interp_init(&ip);
    ll.lat = a->landlat;
    ll.alt = a->landalt;
    ll.lon = agc_dp_shl(&ip, a->landlong, 1);
    return agc_navaux_set_latlong(&ll);
}
