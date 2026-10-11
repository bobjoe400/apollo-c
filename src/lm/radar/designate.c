/*
 * designate.c -- rendezvous radar designate, LM only (P20-P25.agc,
 * Luminary099 pp.538-552, 593): RRDESSM, STARTDES, TRYSWS, RRDESNB,
 * RRLIMNB, BEGDES / STDESIG / DESLOOP, DODES, GOTOV56.
 *
 * "RRDESSM will designate, if desired angles are within the limits of
 * either mode, to a line-of-sight (LOS) vector (half-unit) known with
 * respect to the stable member present orientation." BEGDES then calls
 * the DODES job every .5 s, which turns the angle between the LOS and the
 * RR boresight into gyro commands, until lock-on (RR data good) or until
 * DESCOUNT runs out.
 *
 * Scales: LOS (RRTARGET) half unit 2^1, MLOSV m 2^29, LOSVEL m/cs 2^16,
 * angles revolutions (CDULOGIC), commands SP pulses.
 */
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"
#include "radar.h"

#define RR_MAXTRYS 60 /* MAXTRYS DEC 60             */
#define RR_RDESGAIN \
    8786                                          /* RDESGAIN DEC .53624: null
                                                   * .5 of the error in .5 s   */
#define RR_COS_HALFDEG      ((agc_dp_t) 67106309) /* COS1/2DG 2DEC .999961923 B-2 */
#define RR_MCTOMS           ((agc_dp_t) 3276800) /* MCTOMS 2DEC 100 B-13       */
#define RR_LIM_T1           (-5006) /* 55 degrees (RRLIMCHK)      */
#define RR_LIM_S1           (-5871) /* 64.5 degrees               */
#define RR_5_5DEGS          501
#define RR_20_5DEGS         1866 /* 20.5DEGS DEC .11389        */
#define RR_DT_20MS          2U
#define RR_DT_HALFSEC       50U
#define RR_PRIO26           ((agc_prio_t) 026)
#define RR_ALARM_BAD_ANGLES ((agc_alarm_code_t) 00502)

/* ---- STARTDES / DESRETRN (pp.540-541) ----------------------------------
 * Designate flag on; unless a reposition is running (it will start the
 * designate when done), enable the error counters and start BEGDES in
 * 20 ms. Returns to the caller (STARTED) when nobody waits in RADSTALL,
 * else the job ends. Called inhibited; releases.                       */
static agc_status_t rr_startdes(agc_rr_designate_result_t *res)
{
    agc_status_t st = AGC_STATUS_OK;

    rr_radmodes_set(RR_DESIGBIT);

    if (!rr_radmodes_test(RR_REPOSBIT))
    {
        rr_setrrecr();
        st = agc_waitlist_call(RR_DT_20MS, AGC_TASK_RR_DESIGNATE_BEGIN);
    }

    *res = (agc_erase_modecadr(AGC_IMU_DEV_RADAR) == 0) ?
           AGC_RR_DESIGNATE_STARTED :
           AGC_RR_DESIGNATE_END_JOB;
    agc_relint();

    if (!AGC_ASSERT(rr_radmodes_test(RR_DESIGBIT)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* NORDSTAL (p.541): zero RADCADR, wiping out anyone waiting in RADSTALL,
 * since the caller returns to P20 and may do a new RADSTALL.           */
static void rr_nordstal(void)
{
    agc_status_t st = agc_erase_set_modecadr(AGC_IMU_DEV_RADAR, 0);

    agc_relint(); /* DESRTRN */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return;
    }
}

/* RRANGLES failed (LOS along the navigation-base Y axis): LUNDESCH. On
 * the surface: no coverage. In rendezvous: NODESSM, maneuver required;
 * else (R29) the job ends. Note: NODESSM re-inverts the mode bit
 * although this path never inverted it (as in the AGC).                */
static agc_status_t rr_lundesch(agc_rr_designate_result_t *res)
{
    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        rr_nordstal();
        *res = AGC_RR_DESIGNATE_NO_COVERAGE;

        return (AGC_STATUS_OK);
    }

    if (agc_flag_test(AGC_FLAG_RNDVZFLG))
    {
        rr_mode_invert(); /* NODESSM */
        rr_nordstal();
        *res = AGC_RR_DESIGNATE_MANEUVER;

        return (AGC_STATUS_OK);
    }

    *res = AGC_RR_DESIGNATE_END_JOB; /* ENDOFJOB */

    return (AGC_STATUS_OK);
}

/* ---- RRDESSM (pp.538-539, 542) ----------------------------------------
 * RRTARGET (half unit, SM) to NB axes (CDUTRIG, *SMNB*), RR angles for
 * both modes (RRANGLES); designate in the present mode, or after a remode
 * (TRYSWS), or report that a vehicle maneuver (or, on the surface, no
 * mode 2 coverage) prevents it.                                        */
agc_status_t agc_rr_designate_sm(agc_interp_t              *ip,
                                 agc_rr_designate_result_t *res)
{
    if (!AGC_ASSERT(ip != NULL && res != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rr_state_t *s = rr_state();

    rz_flag_put(AGC_FLAG_RRNBSW, false);
    agc_cdutrig_t trig;
    agc_vec_t     los = s->rrtarget;

    agc_guid_cdutrig(&trig);
    agc_status_t st = AGC_STATUS_OK;

    agc_guid_smnb(&trig, &los);
    bool along_y = false;

    st = agc_lm_rrangles(ip, los, rr_mode2(), s->modea, s->modeb, &along_y);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (along_y)
    {
        return (rr_lundesch(res));
    }

    s->tang[0] = (agc_sp_t) rr_ones(s->modea[0]); /* MODEA EQUALS TANG */
    s->tang[1] = (agc_sp_t) rr_ones(s->modea[1]);
    agc_inhint();

    if (rr_limits_ok(s->modea[0], s->modea[1]))
    {
        return (rr_startdes(res)); /* OKDESSM */
    }

    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    { /* no mode 2 on the moon */
        rr_nordstal();
        *res = AGC_RR_DESIGNATE_NO_COVERAGE;

        return (AGC_STATUS_OK);
    }

    rr_mode_invert(); /* TRYSWS  */
    bool other = rr_limits_ok(s->modeb[0], s->modeb[1]);

    rr_mode_invert();

    if (other)
    {
        rr_radmodes_set(RR_REMODBIT);

        return (rr_startdes(res));
    }

    rr_nordstal(); /* NODESSM */
    *res = AGC_RR_DESIGNATE_MANEUVER;

    return (AGC_STATUS_OK);
}

/* RRLIMNB (pp.543-544): RRLIMCHK with the mode 1 shaft lower limit at -85
 * degrees instead of -70 (verb 41).                                     */
static bool rr_limnb(uint16_t t, uint16_t s)
{
    if (rr_mode2())
    {
        return (rr_limits_ok(t, s)); /* MODE2CHK      */
    }

    int sv = rr_ones(s);

    if (!rr_magsub(rr_ones(t), RR_LIM_T1))
    {
        return (false);
    }

    sv += (sv <= 0) ? RR_20_5DEGS : RR_5_5DEGS; /* NEGSHAFT      */

    return (rr_magsub(sv, RR_LIM_S1));
}

/* ---- RRDESNB (p.543): designate to given RR gimbal angles (CDU words),
 * independent of vehicle motion. Bad angles: alarm 502, CLRADMOD, the
 * job ends (END_JOB).                                                  */
agc_status_t agc_rr_designate_nb(agc_interp_t *ip, uint16_t trunnion,
                                 uint16_t shaft, agc_rr_designate_result_t *res)
{
    if (!AGC_ASSERT(ip != NULL && res != NULL) ||
        !AGC_ASSERT(trunnion <= RR_WORD_MASK && shaft <= RR_WORD_MASK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rr_state_t *s = rr_state();

    rz_flag_put(AGC_FLAG_LOSCMFLG, false); /* keep DODES out of R21     */
    s->descount = RR_MAXTRYS;
    agc_inhint();
    bool ok = rr_limnb(trunnion, shaft);

    if (!ok)
    {
        rr_mode_invert(); /* TRYSWN  */
        ok = rr_limnb(trunnion, shaft);
        rr_mode_invert();

        if (ok)
        {
            rr_radmodes_set(RR_REMODBIT);
        }
    }

    agc_relint();

    if (!ok)
    { /* NODESNB */
        *res = AGC_RR_DESIGNATE_END_JOB;
        agc_alarm(RR_ALARM_BAD_ANGLES);

        agc_rendz_clradmod();

        return (AGC_STATUS_OK); /* avoid the 503 alarm      */
    }

    s->tangnb[0] = trunnion; /* OKDESNB */
    s->tangnb[1] = shaft;
    agc_status_t st = agc_lm_rrnb(ip, trunnion, shaft, &s->rrtarget);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_flag_put(AGC_FLAG_RRNBSW, true);
    agc_inhint();

    return (rr_startdes(res)); /* STARTDES +1 */
}

/* ---- BEGDES / STDESIG / DESLOOP (pp.545-546) -------------------------- */
/* STDESIG: a reposition stops the designate (bad end); continuous
 * designate commands without limit; otherwise end when the designate
 * flag is off, or count DESCOUNT down and, when it runs out, disable the
 * tracker and error counters (bad end). Each pass starts DODES and comes
 * back in .5 s.                                                         */
agc_status_t rr_stdesig(void)
{
    rr_state_t *s = rr_state();

    if (rr_radmodes_test(RR_REPOSBIT))
    {
        rr_radmodes_clear(RR_DESIGBIT); /* BADDES  */

        return (rr_badend());
    }

    if (!rr_radmodes_test(RR_CDESBIT))
    {
        if (!rr_radmodes_test(RR_DESIGBIT))
        {
            return (rr_endradar());
        }

        if (s->descount <= 0)
        { /* STDESIG1 */
            rr_chan_clear(AGC_CHAN_CHAN12,
                          (uint16_t) (RR_CH12_TRACK | RR_CH12_ENB_ECTR));
            rr_radmodes_clear(RR_DESIGBIT);

            return (rr_badend());
        }

        s->descount--; /* MOREDES */
    }

    agc_status_t st = agc_exec_findvac(RR_PRIO26, AGC_JOB_RR_DESIGNATE_COMMAND);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_waitlist_fixdelay(RR_DT_HALFSEC, AGC_TASK_RR_DESIGNATE_LOOP));
}

/* BEGDES: remode first if one was requested (REMODE ends in STDESIG).   */
agc_status_t rr_begdes(void)
{
    if (!AGC_ASSERT(agc_erase_radmodes() <= RR_WORD_MASK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (rr_radmodes_test(RR_REMODBIT))
    {
        return (rr_remode_start());
    }

    return (rr_stdesig());
}

agc_status_t agc_task_rr_designate_begin(void)
{
    if (!AGC_ASSERT(agc_erase_radmodes() <= RR_WORD_MASK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rr_begdes());
}

/* DESLOOP: the .5 s designate pass.                                     */
agc_status_t agc_task_rr_designate_loop(void)
{
    if (!AGC_ASSERT(agc_erase_radmodes() <= RR_WORD_MASK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rr_stdesig());
}

/* ---- DODES (pp.547-552) ----------------------------------------------- */
/* The LOS in NB axes: RRTARGET as is (RRNBSW), or UNIT(RRTARGET MLOSV +
 * one second of LOSVEL) transformed with the present CDUs (*SMNB*).    */
static agc_status_t rr_dodes_los(agc_interp_t *ip, const rr_state_t *s,
                                 agc_cdutrig_t *trig, agc_vec_t *los)
{
    if (agc_flag_test(AGC_FLAG_RRNBSW))
    {
        *los = s->rrtarget; /* DONBRD  */

        return (AGC_STATUS_OK);
    }

    agc_vec_t r  = agc_vec_shl(ip, agc_vec_scale(ip, s->rrtarget, s->mlosv), 1);
    agc_vec_t dv = agc_vec_scale(ip, s->losvel, RR_MCTOMS);

    *los = agc_vec_unit(ip, agc_vec_add(ip, r, dv), NULL, NULL);
    agc_guid_cdutrig(trig);
    agc_guid_smnb(trig, los);

    return (AGC_STATUS_OK);
}

/* RRDESDUN (p.551): designate over; the next DESLOOP pass ends it.      */
static agc_status_t rr_desdun(void)
{
    agc_inhint();
    rr_radmodes_clear(RR_DESIGBIT);
    rz_flag_put(AGC_FLAG_LOSCMFLG, false);
    rr_chan_clear(AGC_CHAN_CHAN12, RR_CH12_ENB_ECTR);
    agc_relint();

    if (!AGC_ASSERT(!rr_radmodes_test(RR_DESIGBIT)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK); /* ENDOFJOB */
}

/* DORROUT / NOTP20 (pp.551-552): with LOSCMFLG or SRCHOPTN, advance the
 * LOS by .5 s of LOSVEL; send the commands unless a monitor reposition
 * took over; every LOSCOUNT passes (LOSCMFLG) kill the designate task and
 * continue as R21 to compute a new LOS.                                */
static agc_status_t rr_dorrout(agc_interp_t *ip, rr_state_t *s)
{
    bool loscm = agc_flag_test(AGC_FLAG_LOSCMFLG);

    if (loscm || agc_flag_test(AGC_FLAG_SRCHOPTN))
    {
        agc_vec_t r =
            agc_vec_shl(ip, agc_vec_scale(ip, s->rrtarget, s->mlosv), 1);
        agc_vec_t dv = agc_vec_shr(agc_vec_scale(ip, s->losvel, RR_MCTOMS), 1);

        s->rrtarget = agc_vec_unit(ip, agc_vec_add(ip, r, dv), &s->mlosv, NULL);
    }

    agc_inhint(); /* NOTP20  */

    if (!rr_radmodes_test(RR_REPOSBIT))
    {
        rr_rrout();
    }

    if (!loscm || s->loscount > 0)
    {
        if (loscm)
        {
            s->loscount--; /* DODESEND */
        }

        agc_relint();

        return (AGC_STATUS_OK); /* ENDOFJOB */
    }

    agc_waitlist_kill(AGC_TASK_RR_DESIGNATE_LOOP);
    agc_status_t st = AGC_STATUS_OK;

    agc_relint();

    if (st != AGC_STATUS_OK && st != AGC_STATUS_NOT_FOUND)
    {
        return (st);
    }

    return (rz_continue(AGC_JOB_RENDEZVOUS_SIGHTING_MARK_NEW_LOS)); /* R21LEM2 */
}

/* DODES: one designate pass (FINDVAC job every .5 s from STDESIG).      */
agc_status_t agc_job_rr_designate_command(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rr_state_t  *s    = rr_state();
    uint16_t     cdut = agc_erase_cdut(); /* TANG    */
    uint16_t     cdus = agc_erase_cdus();
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_cdutrig_t trig;
    agc_vec_t     los     = s->rrtarget;
    agc_dp_t      cos_err = 0;
    agc_status_t  st      = rr_dodes_los(&ip, s, &trig, &los);

    if (st == AGC_STATUS_OK)
    {
        /* Shaft and trunnion commands for the LOS (32D), and the cosine of the
         * angle between the RR boresight and RRTARGET minus cos(.5 deg) (B-2).
         *   shaft    = LOS . (cos S, 0, -sin S)
         *   -trunnion = LOS . (sin T sin S, cos T, sin T cos S)
         *   URR      = (sin S cos T, -sin T, cos S cos T)  (NB, then *NBSM*)  */
        agc_dp_t sa    = agc_guid_cdulogic(cdus);
        agc_dp_t sn_s  = agc_dp_sin(&ip, sa);  /* 0D      */
        agc_dp_t cs_s  = agc_dp_cos(&ip, sa);  /* 2D      */
        agc_dp_t shaft = agc_dp_sub(&ip, agc_dp_mul_trunc(cs_s, los.c[0]),
                                    agc_dp_mul_trunc(los.c[2], sn_s));
        agc_dp_t  ta   = agc_guid_cdulogic(cdut);
        agc_dp_t  cs_t = agc_dp_cos(&ip, ta);  /* 4D      */
        agc_dp_t  sn_t = agc_dp_sin(&ip, ta);  /* 6D      */
        agc_vec_t u    =
            agc_vec_make(agc_dp_shl(&ip, agc_dp_mul_trunc(sn_t, sn_s), 1), cs_t,
                         agc_dp_shl(&ip, agc_dp_mul_trunc(sn_t, cs_s), 1));
        agc_dp_t trun = agc_vec_dot(&ip, u, los);

        s->tang[0] = (agc_sp_t) agc_dp_high(
            agc_sp_mul((agc_sp_t) -agc_dp_high(trun), RR_RDESGAIN));
        agc_sp_t sh = agc_dp_high(shaft);

        sh         = rr_mode2() ? (agc_sp_t) -sh : sh; /* relay reverses in mode 2 */
        s->tang[1] = agc_dp_high(agc_sp_mul(sh, RR_RDESGAIN));
        agc_vec_t urr = agc_vec_make(
            agc_dp_shl(&ip, agc_dp_mul_trunc(sn_s, cs_t), 1), agc_dp_neg(sn_t),
            agc_dp_shl(&ip, agc_dp_mul_trunc(cs_s, cs_t), 1));

        if (!agc_flag_test(AGC_FLAG_RRNBSW))
        {
            agc_guid_nbsm(&trig, &urr);
        }

        cos_err =
            agc_dp_sub(&ip, agc_vec_dot(&ip, urr, s->rrtarget), RR_COS_HALFDEG);
        st = AGC_STATUS_OK;
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool off = agc_dp_high(cos_err) < 0; /* MPAC +1 = 1: not pointed */

    if (rr_radmodes_test(RR_CDESBIT))
    {
        return (rr_dorrout(&ip, s)); /* continuous: no checks    */
    }

    if (!off)
    { /* SIGNLCHK */
        if (!agc_flag_test(AGC_FLAG_LOKONSW))
        {
            return (rr_desdun());
        }

        agc_chan_or(AGC_CHAN_CHAN12, RR_CH12_TRACK); /* tracker */
    }

    if ((agc_chan_read(AGC_CHAN_CHAN33) & RR_CH33_RR_DGOOD) == 0U)
    {
        return (rr_desdun()); /* DGOODCHK: data good    */
    }

    return (rr_dorrout(&ip, s));
}

/* ---- GOTOV56 (p.593): P20 terminates through verb 56 (TRMTRACK)
 * instead of GOTOPOOH. DEVIATION: the AGC continues the calling job at
 * TRMTRACK; here the extended verb is started as its own job and the
 * calling job ends.                                                    */
agc_status_t agc_rendz_goto_terminate_tracking(void)
{
    agc_status_t st = agc_extverb(56, 0);

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* ---- TANG (noun 72) and RRDESNB for verb 41 ----------------------------
 * TANG, TANG +1 hold the gimbal command or, for RRDESNB, the desired
 * trunnion and shaft CDU words (kept as ones' complement SP numbers).  */
agc_sp_t agc_erase_tang(int index)
{
    if (!AGC_ASSERT(index == 0 || index == 1))
    {
        return (0);
    }

    return (rr_state()->tang[index]);
}

void agc_erase_set_tang(int index, agc_sp_t v)
{
    if (AGC_ASSERT(index == 0 || index == 1))
    {
        rr_state()->tang[index] = v;
    }
}

agc_status_t agc_rr_designate_gimbal(bool *started)
{
    if (!AGC_ASSERT(started != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rr_state_t  *s = rr_state();
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_rr_designate_result_t res = AGC_RR_DESIGNATE_END_JOB;
    agc_status_t              st  = agc_rr_designate_nb(&ip, rr_word(
                                                            s->tang[0]),
                                                        rr_word(
                                                            s->tang[1]), &res);

    *started = (res == AGC_RR_DESIGNATE_STARTED);

    return (st);
}
