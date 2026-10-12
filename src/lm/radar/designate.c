/**********************************************************************
 * FILE NAME: designate.c
 *
 * PURPOSE:
 *   Rendezvous radar designate, LM only: RRDESSM, STARTDES, TRYSWS,
 *   RRDESNB, RRLIMNB, BEGDES / STDESIG / DESLOOP, DODES, GOTOV56.
 *   "RRDESSM will designate, if desired angles are within the limits of
 *   either mode, to a line-of-sight (LOS) vector (half-unit) known with
 *   respect to the stable member present orientation." BEGDES then
 *   calls the DODES job every .5 s, which turns the angle between the
 *   LOS and the RR boresight into gyro commands, until lock-on (RR data
 *   good) or until DESCOUNT runs out.
 *
 * AGC SOURCE:
 *   Luminary099 P20-P25.agc pp. 538-552, 593 (RRDESSM, STARTDES,
 *   DESRETRN, NORDSTAL, LUNDESCH, TRYSWS, NODESSM, RRDESNB, RRLIMNB,
 *   BEGDES, STDESIG, DESLOOP, DODES, DONBRD, SIGNLCHK, DGOODCHK,
 *   RRDESDUN, DORROUT, NOTP20, GOTOV56); LM only
 *
 * EXTERNAL VARIABLES:
 *   Name        Type       I/O  Scale      Description
 *   ----------  ---------  ---  ---------  ---------------------------
 *   RRTARGET    agc_vec_t  I/O  2^1        LOS, half unit (SM / NB)
 *   MLOSV       agc_dp_t   I/O  m 2^29     LOS magnitude
 *   LOSVEL      agc_vec_t  I    m/cs 2^16  LOS rate
 *   MODEA MODEB TANGNB words I/O  CDU      RR angles per mode
 *   TANG        agc_sp_t   O    pulses     gyro commands (noun 72)
 *   DESCOUNT LOSCOUNT  SP  I/O  counts     pass counters
 *   RADMODES    word       I/O  -          DESIGBIT REMODBIT CDESBIT ..
 *   RADCADR     word       I/O  -          RADSTALL waiter
 *   CDUT CDUS   words      I    CDU        RR trunnion and shaft
 *   CHAN12 CHAN33  channels I/O -          tracker, error counters,
 *                                          RR data good
 *   RRNBSW LOSCMFLG SRCHOPTN LOKONSW SURFFLAG RNDVZFLG  flags  I/O
 *
 * EXTERNAL REFERENCES:
 *   Name                Description
 *   ------------------  ------------------------------------------------
 *   agc_lm_rrangles     RRANGLES: RR angles for both modes
 *   agc_lm_rrnb         RRNB: gimbal angles to an NB LOS
 *   agc_rr_limchk       RRLIMCHK
 *   agc_guid_smnb       *SMNB* / *NBSM* with CDUTRIG
 *   rr_rrout            RROUT: send the gyro commands
 *   rr_remode_start     REMODE
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   00502 (bad RR angles in RRDESNB). Designate failures are returned
 *   as agc_rr_designate_result_t (maneuver required, no coverage, end
 *   job). AGC_STATUS_BAD_ARG for NULL pointers; waitlist and executive
 *   statuses are passed back.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   STARTDES is called inhibited and releases. DESLOOP is a .5 s
 *   waitlist task, DODES a PRIO26 job; at most MAXTRYS (60) passes.
 *
 * NOTES:
 *   Scales: LOS (RRTARGET) half unit 2^1, MLOSV m 2^29, LOSVEL m/cs
 *   2^16, angles revolutions (CDULOGIC), commands SP pulses.
 *   DEVIATION (GOTOV56): the extended verb 56 is started as its own job
 *   and the calling job ends.
 **********************************************************************/
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

/* RDESGAIN DEC .53624: null .5 of the error in .5 s                    */
#define RR_RDESGAIN         8786                       /* RDESGAIN, 2^0         */
#define RR_COS_HALFDEG      ((agc_dp_t) 67106309)      /* COS1/2DG, B-2        */
#define RR_MCTOMS           ((agc_dp_t) 3276800)       /* MCTOMS 100, B-13      */
#define RR_LIM_T1           (-5006)                    /* 55 degrees (RRLIMCHK)      */
#define RR_LIM_S1           (-5871)                    /* 64.5 degrees               */
#define RR_5_5DEGS          501                        /* 5.5DEGS, CDU units */
#define RR_20_5DEGS         1866                       /* 20.5DEGS DEC .11389        */
#define RR_DT_20MS          2U                         /* 20 ms, cs */
#define RR_DT_HALFSEC       50U                        /* .5 s, cs */
#define RR_PRIO26           ((agc_prio_t) 026)         /* PRIO26 */
#define RR_ALARM_BAD_ANGLES ((agc_alarm_code_t) 00502) /* bad RR angles */

/**********************************************************************
 * STARTDES / DESRETRN (pp.540-541)
 * Designate flag on; unless a reposition is running (it will start the
 * designate when done), enable the error counters and start BEGDES in
 * 20 ms. Returns to the caller (STARTED) when nobody waits in RADSTALL,
 * else the job ends. Called inhibited; releases.
 * AGC: STARTDES, DESRETRN (P20-P25.agc pp.540-541)
 *
 * FUNCTION NAME: rr_startdes
 *
 * ARGUMENTS:
 *   Argument  Type                         I/O  Description (scale)
 *   --------  ---------------------------  ---  -------------------
 *   res       agc_rr_designate_result_t *  O    STARTED (return to the
 *                                               caller) or END_JOB
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG for a failed
 *          invariant, else the waitlist status
 **********************************************************************/
static agc_status_t rr_startdes(agc_rr_designate_result_t *res)
{
    agc_status_t st = AGC_STATUS_OK; /* status of the last step */

    rr_radmodes_set(RR_DESIGBIT);

    if (!rr_radmodes_test(RR_REPOSBIT))
    {
        rr_setrrecr();
        st = agc_waitlist_call(RR_DT_20MS, AGC_TASK_RR_DESIGNATE_BEGIN);
    }

    *res = AGC_RR_DESIGNATE_END_JOB; /* someone waits in RADSTALL */

    if (agc_erase_modecadr(AGC_IMU_DEV_RADAR) == 0)
    {
        *res = AGC_RR_DESIGNATE_STARTED; /* return to the caller */
    }

    agc_relint();

    if (!AGC_ASSERT(rr_radmodes_test(RR_DESIGBIT)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * NORDSTAL (p.541): zero RADCADR, wiping out anyone waiting in RADSTALL,
 * since the caller returns to P20 and may do a new RADSTALL.
 * AGC: NORDSTAL (P20-P25.agc p.541)
 *
 * FUNCTION NAME: rr_nordstal
 *
 * ARGUMENTS: none
 **********************************************************************/
static void rr_nordstal(void)
{
    agc_erase_set_modecadr(AGC_IMU_DEV_RADAR, 0); /* RADCADR */
    agc_relint();                                 /* DESRTRN */
}

/**********************************************************************
 * RRDESSM (pp.538-539, 542)
 * RRTARGET (half unit, SM) to NB axes (CDUTRIG, *SMNB*), RR angles for
 * both modes (RRANGLES); designate in the present mode, or after a remode
 * (TRYSWS), or report that a vehicle maneuver (or, on the surface, no
 * mode 2 coverage) prevents it.
 * AGC: RRDESSM, TRYSWS (P20-P25.agc pp.538-539, 542)
 *
 * FUNCTION NAME: agc_rr_designate_sm
 *
 * ARGUMENTS:
 *   Argument  Type                         I/O  Description (scale)
 *   --------  ---------------------------  ---  -------------------
 *   ip        agc_interp_t *               I/O  interpreter state (OVFIND)
 *   res       agc_rr_designate_result_t *  O    designate result
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the RRANGLES or waitlist status
 **********************************************************************/
agc_status_t agc_rr_designate_sm(agc_interp_t *ip, agc_rr_designate_result_t *res)
{
    rr_state_t *s = rr_state(); /* radar erasables */

    rz_flag_put(AGC_FLAG_RRNBSW, false);
    agc_cdutrig_t trig;              /* CDUTRIG sines / cosines */
    agc_vec_t     los = s->rrtarget; /* LOS, 2^1 */

    agc_guid_cdutrig(&trig);
    agc_status_t st = AGC_STATUS_OK; /* status of the last step */

    agc_guid_smnb(&trig, &los);
    bool along_y = false; /* LOS along NB Y */

    st = agc_lm_rrangles(ip, los, rr_mode2(), s->modea, s->modeb, &along_y);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (along_y)
    {
        /**********************************************************************
         * RRANGLES failed (LOS along the navigation-base Y axis): LUNDESCH. On
         * the surface: no coverage. In rendezvous: NODESSM, maneuver required;
         * else (R29) the job ends. Note: NODESSM re-inverts the mode bit
         * although this path never inverted it (as in the AGC).
         * AGC: LUNDESCH, NODESSM (P20-P25.agc p.542)
         *
         * FUNCTION NAME: rr_lundesch
         *
         * ARGUMENTS:
         *   Argument  Type                         I/O  Description (scale)
         *   --------  ---------------------------  ---  -------------------
         *   res       agc_rr_designate_result_t *  O    NO_COVERAGE, MANEUVER or
         *                                               END_JOB
         *
         * RETURNS: agc_status_t -- AGC_STATUS_OK
         **********************************************************************/
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

    agc_erase_set_tang(0, (agc_sp_t) rr_ones(s->modea[0])); /* MODEA = TANG */
    agc_erase_set_tang(1, (agc_sp_t) rr_ones(s->modea[1]));
    agc_inhint();

    if (agc_rr_limchk(s->modea[0], s->modea[1]))
    {
        return (rr_startdes(res)); /* OKDESSM */
    }

    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    { /* no mode 2 on the moon */
        rr_nordstal();
        *res = AGC_RR_DESIGNATE_NO_COVERAGE;
    }
    else
    {
        rr_mode_invert();                                     /* TRYSWS  */
        bool other = agc_rr_limchk(s->modeb[0], s->modeb[1]); /* other mode OK */

        rr_mode_invert();

        if (other)
        {
            rr_radmodes_set(RR_REMODBIT);

            return (rr_startdes(res));
        }

        rr_nordstal(); /* NODESSM */
        *res = AGC_RR_DESIGNATE_MANEUVER;
    }

    return (AGC_STATUS_OK);
} /* agc_rr_designate_sm */

/**********************************************************************
 * RRLIMNB (pp.543-544): RRLIMCHK with the mode 1 shaft lower limit at -85
 * degrees instead of -70 (verb 41).
 * AGC: RRLIMNB (P20-P25.agc pp.543-544)
 *
 * FUNCTION NAME: rr_limnb
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   t         uint16_t  I    trunnion CDU word
 *   s         uint16_t  I    shaft CDU word
 *
 * RETURNS: bool -- true when the angles are within the limits of the
 *          present mode
 **********************************************************************/
static bool rr_limnb(uint16_t t, uint16_t s)
{
    if (rr_mode2())
    {
        return (agc_rr_limchk(t, s)); /* MODE2CHK      */
    }

    int sv = rr_ones(s); /* shaft, CDU units */

    if (!rr_magsub(rr_ones(t), RR_LIM_T1))
    {
        return (false);
    }

    sv += (sv <= 0) ? RR_20_5DEGS : RR_5_5DEGS; /* NEGSHAFT      */

    return (rr_magsub(sv, RR_LIM_S1));
}

/**********************************************************************
 * RRDESNB (p.543): designate to given RR gimbal angles (CDU words),
 * independent of vehicle motion. Bad angles: alarm 502, CLRADMOD, the
 * job ends (END_JOB).
 * AGC: RRDESNB (P20-P25.agc p.543)
 *
 * FUNCTION NAME: agc_rr_designate_nb
 *
 * ARGUMENTS:
 *   Argument  Type                         I/O  Description (scale)
 *   --------  ---------------------------  ---  -------------------
 *   ip        agc_interp_t *               I/O  interpreter state (OVFIND)
 *   trunnion  uint16_t                     I    trunnion CDU word
 *   shaft     uint16_t                     I    shaft CDU word
 *   res       agc_rr_designate_result_t *  O    designate result
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the RRNB or waitlist status
 **********************************************************************/
agc_status_t agc_rr_designate_nb(agc_interp_t *ip, uint16_t trunnion, uint16_t shaft, agc_rr_designate_result_t *res)
{
    if (!AGC_ASSERT(ip != NULL && res != NULL) || !AGC_ASSERT(trunnion <= RR_WORD_MASK && shaft <= RR_WORD_MASK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rr_state_t *s = rr_state(); /* radar erasables */

    rz_flag_put(AGC_FLAG_LOSCMFLG, false);   /* keep DODES out of R21     */
    s->descount = RR_MAXTRYS;
    agc_inhint();
    bool ok     = rr_limnb(trunnion, shaft); /* within limits */

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

    s->tangnb[0]    = trunnion;                                       /* OKDESNB */
    s->tangnb[1]    = shaft;
    agc_status_t st = agc_lm_rrnb(ip, trunnion, shaft, &s->rrtarget); /* RRNB */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_flag_put(AGC_FLAG_RRNBSW, true);
    agc_inhint();

    return (rr_startdes(res)); /* STARTDES +1 */
} /* agc_rr_designate_nb */

/* ---- BEGDES / STDESIG / DESLOOP (pp.545-546) -------------------------- */
/**********************************************************************
 * STDESIG: a reposition stops the designate (bad end); continuous
 * designate commands without limit; otherwise end when the designate
 * flag is off, or count DESCOUNT down and, when it runs out, disable the
 * tracker and error counters (bad end). Each pass starts DODES and comes
 * back in .5 s.
 * AGC: STDESIG, MOREDES (P20-P25.agc pp.545-546)
 *
 * FUNCTION NAME: rr_stdesig
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the executive or waitlist
 *          status
 **********************************************************************/
agc_status_t rr_stdesig(void)
{
    rr_state_t *s = rr_state(); /* radar erasables */

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
            rr_chan_clear(AGC_CHAN_CHAN12, (uint16_t) (RR_CH12_TRACK | RR_CH12_ENB_ECTR));
            rr_radmodes_clear(RR_DESIGBIT);

            return (rr_badend());
        }

        s->descount--; /* MOREDES */
    }

    agc_status_t st; /* DODES */

    st = agc_exec_findvac(RR_PRIO26, AGC_JOB_RR_DESIGNATE_COMMAND);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_waitlist_fixdelay(RR_DT_HALFSEC, AGC_TASK_RR_DESIGNATE_LOOP));
}

/**********************************************************************
 * BEGDES (waitlist task, 20 ms after STARTDES): remode first if one was
 * requested (REMODE ends in STDESIG), else STDESIG.
 * AGC: BEGDES (P20-P25.agc p.545)
 *
 * FUNCTION NAME: agc_task_rr_designate_begin
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
agc_status_t agc_task_rr_designate_begin(void)
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

/**********************************************************************
 * DESLOOP: the .5 s designate pass.
 * AGC: DESLOOP (P20-P25.agc p.546)
 *
 * FUNCTION NAME: agc_task_rr_designate_loop
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
agc_status_t agc_task_rr_designate_loop(void)
{
    if (!AGC_ASSERT(agc_erase_radmodes() <= RR_WORD_MASK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rr_stdesig());
}

/* ---- DODES (pp.547-552) ----------------------------------------------- */
/**********************************************************************
 * The LOS in NB axes: RRTARGET as is (RRNBSW), or UNIT(RRTARGET MLOSV +
 * one second of LOSVEL) transformed with the present CDUs (*SMNB*).
 * AGC: DODES, DONBRD (P20-P25.agc p.547)
 *
 * FUNCTION NAME: rr_dodes_los
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  ----------------------------
 *   ip        agc_interp_t *      I/O  interpreter state (OVFIND)
 *   s         const rr_state_t *  I    RRTARGET MLOSV LOSVEL
 *   trig      agc_cdutrig_t *     O    CDUTRIG of the present CDUs
 *   los       agc_vec_t *         O    the LOS in NB axes, half unit
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK
 **********************************************************************/
static agc_status_t rr_dodes_los(agc_interp_t *ip, const rr_state_t *s, agc_cdutrig_t *trig, agc_vec_t *los)
{
    if (agc_flag_test(AGC_FLAG_RRNBSW))
    {
        *los = s->rrtarget; /* DONBRD  */
    }
    else
    {
        agc_vec_t r; /* m 2^29 */

        r = agc_vec_shl(ip, agc_vec_scale(ip, s->rrtarget, s->mlosv), 1);
        agc_vec_t dv; /* 1 s of LOSVEL, 2^29 */

        dv = agc_vec_scale(ip, s->losvel, RR_MCTOMS);

        *los = agc_vec_unit(ip, agc_vec_add(ip, r, dv), NULL, NULL);
        agc_guid_cdutrig(trig);
        agc_guid_smnb(trig, los);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * RRDESDUN (p.551): designate over; the next DESLOOP pass ends it.
 * AGC: RRDESDUN (P20-P25.agc p.551)
 *
 * FUNCTION NAME: rr_desdun
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a
 *          failed invariant
 **********************************************************************/
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

/**********************************************************************
 * DORROUT / NOTP20 (pp.551-552): with LOSCMFLG or SRCHOPTN, advance the
 * LOS by .5 s of LOSVEL; send the commands unless a monitor reposition
 * took over; every LOSCOUNT passes (LOSCMFLG) kill the designate task and
 * continue as R21 to compute a new LOS.
 * AGC: DORROUT, NOTP20 (P20-P25.agc pp.551-552)
 *
 * FUNCTION NAME: rr_dorrout
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   s         rr_state_t *    I/O  RRTARGET MLOSV advanced, LOSCOUNT
 *                                  counted
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the status of the R21
 *          continuation
 **********************************************************************/
static agc_status_t rr_dorrout(agc_interp_t *ip, rr_state_t *s)
{
    bool loscm = agc_flag_test(AGC_FLAG_LOSCMFLG); /* LOSCMFLG */

    if (loscm || agc_flag_test(AGC_FLAG_SRCHOPTN))
    {
        agc_vec_t r; /* m 2^29 */

        r = agc_vec_shl(ip, agc_vec_scale(ip, s->rrtarget, s->mlosv), 1);
        agc_vec_t dv; /* .5 s, 2^29 */

        dv = agc_vec_shr(agc_vec_scale(ip, s->losvel, RR_MCTOMS), 1);

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

    agc_waitlist_kill(AGC_TASK_RR_DESIGNATE_LOOP); /* KILLTASK DESLOOP */
    agc_relint();

    /* continue as R21 (R21LEM2) for a new LOS */
    return (rz_continue(AGC_JOB_RENDEZVOUS_SIGHTING_MARK_NEW_LOS));
}

/**********************************************************************
 * DODES commands (pp.548-550): shaft and trunnion commands for the LOS
 * (32D) into TANG, and the cosine of the angle between the RR boresight
 * and RRTARGET minus cos(.5 deg) (B-2).
 *   shaft     = LOS . (cos S, 0, -sin S)
 *   -trunnion = LOS . (sin T sin S, cos T, sin T cos S)
 *   URR       = (sin S cos T, -sin T, cos S cos T)  (NB, then *NBSM*)
 * AGC: DODES (P20-P25.agc pp.548-550)
 *
 * FUNCTION NAME: rr_dodes_commands
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description (scale)
 *   --------  --------------------  ---  ------------------------------
 *   ip        agc_interp_t *        I/O  interpreter state (OVFIND)
 *   s         const rr_state_t *    I    RRTARGET (SM), half unit
 *   trig      const agc_cdutrig_t * I    CDUTRIG of the present CDUs
 *   los       agc_vec_t             I    the LOS in NB axes, half unit
 *
 * RETURNS: agc_dp_t -- cos(boresight, RRTARGET) - cos(.5 deg), B-2
 **********************************************************************/
static agc_dp_t rr_dodes_commands(agc_interp_t *ip, const rr_state_t *s, const agc_cdutrig_t *trig, agc_vec_t los)
{
    agc_dp_t sa    = agc_guid_cdulogic(agc_erase_cdus());                                                /* rev 2^0 */
    agc_dp_t sn_s  = agc_dp_sin(sa);                                                                     /* 0D, 2^1 */
    agc_dp_t cs_s  = agc_dp_cos(sa);                                                                     /* 2D, 2^1 */
    agc_dp_t shaft = agc_dp_sub(ip, agc_dp_mul_trunc(cs_s, los.c[0]), agc_dp_mul_trunc(los.c[2], sn_s)); /* 2^2 */
    agc_dp_t ta    = agc_guid_cdulogic(agc_erase_cdut());                                                /* rev 2^0 */
    agc_dp_t cs_t  = agc_dp_cos(ta);                                                                     /* 4D, 2^1 */
    agc_dp_t sn_t  = agc_dp_sin(ta);                                                                     /* 6D, 2^1 */
    agc_dp_t ux    = agc_dp_shl(ip, agc_dp_mul_trunc(sn_t, sn_s), 1);                                    /* 2^1 */
    agc_dp_t uz    = agc_dp_shl(ip, agc_dp_mul_trunc(sn_t, cs_s), 1);                                    /* 2^1 */
    agc_dp_t trun  = agc_vec_dot(ip, agc_vec_make(ux, cs_t, uz), los);                                   /* 2^2 */
    agc_sp_t cmd_t = (agc_sp_t) -agc_dp_high(trun);                                                      /* trunnion, 2^2 */
    agc_sp_t sh    = agc_dp_high(shaft);                                                                 /* shaft, 2^2 */

    agc_erase_set_tang(0, agc_dp_high(agc_sp_mul(cmd_t, RR_RDESGAIN)));

    if (rr_mode2())
    {
        sh = (agc_sp_t) -sh; /* the relay reverses the shaft in mode 2 */
    }

    agc_erase_set_tang(1, agc_dp_high(agc_sp_mul(sh, RR_RDESGAIN)));
    agc_dp_t  rx  = agc_dp_shl(ip, agc_dp_mul_trunc(sn_s, cs_t), 1); /* 2^1 */
    agc_dp_t  rz  = agc_dp_shl(ip, agc_dp_mul_trunc(cs_s, cs_t), 1); /* 2^1 */
    agc_vec_t urr = agc_vec_make(rx, agc_dp_neg(sn_t), rz);          /* URR, 2^1 */

    if (!agc_flag_test(AGC_FLAG_RRNBSW))
    {
        agc_guid_nbsm(trig, &urr);
    }

    return (agc_dp_sub(ip, agc_vec_dot(ip, urr, s->rrtarget), RR_COS_HALFDEG));
}

/**********************************************************************
 * DODES: one designate pass (FINDVAC job every .5 s from STDESIG).
 * AGC: DODES (P20-P25.agc pp.547-552)
 *
 * FUNCTION NAME: agc_job_rr_designate_command
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rr_designate_command(void)
{
    rr_state_t  *s = rr_state(); /* radar erasables */
    agc_interp_t ip;             /* interpreter state */

    agc_interp_init(&ip);
    agc_cdutrig_t trig;                                    /* CDUTRIG */
    agc_vec_t     los = s->rrtarget;                       /* LOS, 2^1 */
    agc_status_t  st  = rr_dodes_los(&ip, s, &trig, &los); /* NB LOS */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_dp_t cos_err = rr_dodes_commands(&ip, s, &trig, los); /* B-2 */
    bool     off     = agc_dp_high(cos_err) < 0;              /* MPAC +1 = 1: not pointed */

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
} /* agc_job_rr_designate_command */

/**********************************************************************
 * GOTOV56 (p.593): P20 terminates through verb 56 (TRMTRACK)
 * instead of GOTOPOOH. DEVIATION: the AGC continues the calling job at
 * TRMTRACK; here the extended verb is started as its own job and the
 * calling job ends.
 * AGC: GOTOV56 (P20-P25.agc p.593)
 *
 * FUNCTION NAME: agc_rendz_goto_terminate_tracking
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the extended verb request
 **********************************************************************/
agc_status_t agc_rendz_goto_terminate_tracking(void)
{
    agc_status_t st = agc_extverb(56, 0); /* V56 request */

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * RRDESNB for verb 41 (TANG, noun 72)
 * AGC: RRDESNB for verb 41 (P20-P25.agc p.543)
 *
 * FUNCTION NAME: agc_rr_designate_gimbal
 *
 * ARGUMENTS:
 *   Argument  Type    I/O  Description (scale)
 *   --------  ------  ---  ----------------------------------------
 *   started   bool *  O    true when the designate started
 *
 * RETURNS: agc_status_t -- as agc_rr_designate_nb
 **********************************************************************/
agc_status_t agc_rr_designate_gimbal(bool *started)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rr_designate_result_t res = AGC_RR_DESIGNATE_END_JOB;   /* result */
    uint16_t                  t   = rr_word(agc_erase_tang(0)); /* TANG    */
    uint16_t                  sh  = rr_word(agc_erase_tang(1)); /* TANG +1 */
    agc_status_t              st;                               /* RRDESNB */

    st = agc_rr_designate_nb(&ip, t, sh, &res);

    *started = (res == AGC_RR_DESIGNATE_STARTED);

    return (st);
}
