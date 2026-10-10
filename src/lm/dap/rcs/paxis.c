/*
 * paxis.c -- P-AXIS_RCS_AUTOPILOT.agc (Luminary 099 pp.1421-1441): the
 * PAXIS T5RUPT program (every 100 ms while the DAP is on), the gimbal-drive
 * timers, Y,Z translation (CHKVISFZ), the P-axis jet selection (PJETSLEC,
 * SELECTP), CALCPERR and PURGENCY. The manual-mode state logic is in
 * pmanual.c; every P-axis path ends at QRAXIS (qraxis.c).
 *
 * The AGC turns the rest of the pass into a "jask" (SUPERJOB at interrupt
 * level with interrupts enabled; DAPZRUPT > 0 while it runs). In C the
 * pass runs to completion in this call; DAPZRUPT is kept so that a pass
 * that did not reach CLOSEOUT is detected (BAILOUT 02000) as on the AGC.
 */
#include <stddef.h>

#include "../internal.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/lm/dap.h"

#define PX_150MST6   240                /* +150MST6: 150 ms in T6 counts      */
#define PX_MINT6     22                 /* MINTIMES: 14 ms minimum firing     */
#define PX_FOURDEG   1456               /* -FOURDEG = DEC -.08888 at 45 deg   */
#define PX_160MST6   256                /* -160MST6                           */
#define PX_GIMQ      001400u            /* QGIMBITS: chan 12 bits 9,10        */
#define PX_GIMR      006000u            /* RGIMBITS: chan 12 bits 11,12       */
#define PX_TRANSBITS 007400u            /* 07400OCT: chan 31 bits 9-12        */
#define PX_BADINDEX  ((agc_sp_t) 07776) /* INDXYZ: no translation policy    */
#define PX_NTAB      14
#define PX_TYPEP     7 /* TYPEP = JETSALL +7                 */

/* JETSALL (indexed by ROTINDEX) followed by TYPEP (indexed by NUMBERT),
 * contiguous as in the AGC (p.1440): channel 6 jet codes.               */
static const uint16_t jettab[PX_NTAB] = {
    00252, 00125, 00140, 00006,
    00220, 00011, 00151, /* -P +P -Y -Z +Y +Z */
    00146, 00226, 00231, 00151,
    00132, 00245, 00377 /* -U -V +U +V 1-3 2-4 ALL */
};
/* INDXYZ (pp.1440-1441): ROTINDEX for channel 31 bits 9-12 (>> 8).        */
static const agc_sp_t indxyz[16] = {
    -218, 4, 2, PX_BADINDEX, 5, 9,
    10, PX_BADINDEX, 3, 8, 7, PX_BADINDEX,
    PX_BADINDEX, PX_BADINDEX, PX_BADINDEX, PX_BADINDEX
};

/* SELECTP (p.1440): POLYTEMP = TYPEP(NUMBERT) & JETSALL(ROTINDEX); on a
 * failed jet try NUMBERT = 5, 4, ... 0. False if no policy is left.      */
static bool selectp(lm_dap_t *d)
{
    for (int tempnum = 6; tempnum >= -1; tempnum--)
    { /* <= 8 tries     */
        if (!AGC_ASSERT(d->numbert >= 0 && d->numbert <= 6) ||
            !AGC_ASSERT(d->rotindex >= 0 && d->rotindex <= 10))
        {
            return false;
        }

        d->polytemp =
            (agc_sp_t) (jettab[PX_TYPEP + d->numbert] & jettab[d->rotindex]);

        if (((uint16_t) d->polytemp & d->ch6mask) == 0u)
        {
            return true;
        }

        if (tempnum <= 0)
        {
            return false;
        }

        d->numbert = (agc_sp_t) (tempnum - 1); /* SELECTYZ        */
    }

    return false;
}

/* JETSOFF (p.1437): write NEXTP to channel 6, TJP = 0.                    */
agc_status_t lm_dap_jetsoff(void)
{
    lm_dap_t    *d  = lm_dap_state();
    agc_status_t st = lm_dap_write_next(0);

    d->tj[0] = 0;

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(d->tj[0] == 0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return st;
}

/* ALTSYST (p.1437): alternate the P-axis jet system (AORBSYST).          */
static agc_status_t altsyst(void)
{
    agc_flag_invert(AGC_FLAG_AORBSFLG);

    if (!AGC_ASSERT(agc_flag_valid(AGC_FLAG_AORBSFLG)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

/* PJETSLEC (pp.1436-1437), entered at PJETSLEC -1 with NUMBERT = 4, 5
 * (two jets, system A or B) or 6 (four jets): select and fire the P jets;
 * firings under 150 ms are ended by a T6RUPT (JTLST) and the next P pass
 * is skipped (RCSFLAGS bit 12).                                          */
agc_status_t lm_dap_pjetslec(agc_sp_t numbert)
{
    if (!AGC_ASSERT((numbert >= 4) && (numbert <= 6)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    lm_dap_t *d = lm_dap_state();

    d->numbert = numbert;

    if (d->tj[0] == 0)
    {
        return lm_dap_jetsoff();
    }

    d->abstj    = (agc_sp_t) ((d->tj[0] > 0) ? d->tj[0] : -d->tj[0]);
    d->rotindex = (d->tj[0] > 0) ? 1 : 0; /* +P : -P        */

    if (!selectp(d))
    {
        agc_status_t sa = lm_dap_alarm(AGC_ALARM_02003); /* ALARMJET       */
        agc_status_t so = lm_dap_jetsoff();

        return (sa != AGC_STATUS_OK) ? sa : so;
    }

    d->nojets[0] = (d->numbert == 6) ? 4 : 2;
    agc_status_t st = lm_dap_write(0, (uint16_t) d->polytemp);

    if (PX_150MST6 - d->abstj <= 0)
    {
        return st; /* long firing    */
    }

    if (PX_MINT6 - d->abstj > 0)
    { /* MINTIMES       */
        d->abstj = PX_MINT6;
        d->tj[0] = (agc_sp_t) ((d->rotindex != 0) ? PX_MINT6 : -PX_MINT6);
    }

    d->t6furtha[0] = d->abstj;
    d->t6furtha[1] = 0; /* P-axis index   */
    agc_status_t sj = lm_dap_jtlst();

    d->rcsflags = (uint16_t) (d->rcsflags & ~LM_RCS_PSKIP); /* skip next P   */
    agc_status_t sa = altsyst();

    if (!AGC_ASSERT(sj == AGC_STATUS_OK && sa == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(d->abstj >= PX_MINT6))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return st;
}

/* PJETSLEC -6 (p.1436): two jets of the system named by AORBSYST.        */
agc_status_t lm_dap_pjetslec_pair(void)
{
    agc_sp_t n = agc_flag_test(AGC_FLAG_AORBSFLG) ? 5 : 4;

    if (!AGC_ASSERT(n == 4 || n == 5))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(agc_flag_valid(AGC_FLAG_AORBSFLG)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return lm_dap_pjetslec(n);
}

/* CALCPERR (pp.1437-1438): P attitude error at pi, into E and PERROR.    */
agc_status_t lm_dap_calcperr(void)
{
    lm_dap_t *d = lm_dap_state();
    agc_sp_t  e =
        lm_dap_mphi(lm_dap_msu(agc_erase_cduy(), d->cduyd), agc_erase_m11());
    int32_t x = (int32_t) lm_dap_msu(agc_erase_cdux(), d->cduxd) + d->delperor;

    d->e      = lm_dap_wrap((int32_t) e + x); /* ADS E: may wrap */
    d->perror = d->e;                         /* eight-ball     */

    if (!AGC_ASSERT(d->perror >= LM_NEGMAX && d->perror <= LM_POSMAX))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(d->cduxd <= 077777u))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

/* PURGENCY +4 (p.1438): P control with E, EDOT set; docked: SPSRCS,
 * otherwise TJETLAW and the 2-jet / 4-jet choice (HEADTJET).             */
agc_status_t lm_dap_purgency_law(void)
{
    lm_dap_t *d = lm_dap_state();

    d->axisctr = -1;

    if (agc_flag_test(AGC_FLAG_CSMDKFLG))
    {
        agc_status_t ss = lm_dap_spsrcs();

        if (!AGC_ASSERT(ss == AGC_STATUS_OK))
        {
            return AGC_STATUS_BAD_ARG;
        }

        if (d->tj[0] == 0)
        { /* DKALT          */
            agc_status_t sa = altsyst();
            agc_status_t so = lm_dap_jetsoff();

            return (sa != AGC_STATUS_OK) ? sa : so;
        }

        return lm_dap_pjetslec_pair();
    }

    d->sensetyp = 0; /* HEADTJET       */
    agc_status_t st = lm_dap_tjetlaw();

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if ((int32_t) -d->firefct - PX_FOURDEG <= 0)
    { /* not 4 deg past */
        return lm_dap_pjetslec_pair();
    }

    if (d->tj[0] == 0)
    {
        return lm_dap_jetsoff();
    }

    if ((int32_t) lm_dap_ccs_dim(d->tj[0]) - PX_160MST6 <= 0)
    {
        return lm_dap_pjetslec_pair();
    }

    return lm_dap_pjetslec(6); /* four jets      */
}

/* PURGENCY (p.1438): P error and rate error, then the control law.        */
agc_status_t lm_dap_purgency(void)
{
    lm_dap_t    *d  = lm_dap_state();
    agc_status_t st = lm_dap_calcperr();

    d->edot = lm_dap_wrap((int32_t) d->omegap - d->omegapd); /* EDOTP     */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(d->edot >= LM_NEGMAX && d->edot <= LM_POSMAX))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return lm_dap_purgency_law();
}

/* ALTERYZ (p.1430): flip RCSFLAGS bit 1 and move to a diagonal policy.    */
static void alteryz(lm_dap_t *d)
{
    d->rcsflags = (uint16_t) (d->rcsflags ^ LM_RCS_ZEROOR1);
    d->rotindex =
        (agc_sp_t) (d->rotindex + 4 + (int) (d->rcsflags & LM_RCS_ZEROOR1));

    if (!AGC_ASSERT(d->rotindex >= 6 && d->rotindex <= 10))
    {
        return;
    }

    if (!AGC_ASSERT(d->rcsflags <= 077777u))
    {
        return;
    }
}

/* CHKVISFZ / TRYUORV / ABORTYZ (pp.1429-1430): Y,Z translation from the
 * THC into NEXTP. *policy = false if SELECTP found no policy (ALARM 02003,
 * JETSOFF done): the pass then goes straight to QRAXIS.                  */
agc_status_t lm_dap_chkvisfz(bool *policy)
{
    lm_dap_t *d = lm_dap_state();

    if (!AGC_ASSERT(policy != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    *policy = true;
    uint16_t bits = (uint16_t) (~agc_chan_read(031) & PX_TRANSBITS);

    if (bits == 0u)
    {
        d->nextp = 0;

        return AGC_STATUS_OK;
    }

    d->rotindex = indxyz[bits >> 8]; /* MP BIT7        */

    for (int pass = 0; pass < 2; pass++)
    { /* TRYUORV        */
        if (d->rotindex == PX_BADINDEX)
        { /* TC JETSALL -1  */
            d->rcsflags =
                (uint16_t) (d->rcsflags ^ LM_RCS_ZEROOR1); /* ABORTYZ +2 */
            d->nextp = 0;

            return AGC_STATUS_OK;
        }

        d->numbert = 6;

        if (!selectp(d))
        {
            *policy = false;
            agc_status_t sa = lm_dap_alarm(AGC_ALARM_02003);
            agc_status_t so = lm_dap_jetsoff();

            return (sa != AGC_STATUS_OK) ? sa : so;
        }

        if (d->numbert == 6 || (d->rotindex > 5 && d->numbert >= 4))
        {
            d->nextp = (uint16_t) d->polytemp;

            return AGC_STATUS_OK;
        }

        if (d->rotindex > 5)
        {
            break; /* ABORTYZ        */
        }

        alteryz(d);
    }

    agc_status_t sa = lm_dap_alarm(AGC_ALARM_02001);

    d->rcsflags = (uint16_t) (d->rcsflags ^ LM_RCS_ZEROOR1);
    d->nextp    = 0;

    if (!AGC_ASSERT(d->nextp == 0u))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return sa;
}

/* PRETIMCK .. CHKRTIMR (pp.1428-1429): count the trim-gimbal drive timers
 * down (0.1 s per pass) and end a drive at zero; decrement the docked
 * jet-inhibition counters.                                               */
agc_status_t lm_dap_pretimck(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return AGC_STATUS_BAD_ARG;
    }

    lm_dap_t    *d  = lm_dap_state();
    agc_status_t st = AGC_STATUS_OK;

    if (d->qgimtimr > 0)
    {
        d->qgimtimr--; /* DECQTIMR       */
    }
    else if (d->qgimtimr == 0)
    { /* TURNOFFQ       */
        d->neguq   = 0;
        d->qaccdot = 0;
        agc_chan_and(012, (uint16_t) ~PX_GIMQ & 077777u);
        d->qgimtimr = LM_NEGMAX;
    }

    if (d->rgimtimr > 0)
    {
        d->rgimtimr--; /* DECRTIMR       */
    }
    else if (d->rgimtimr == 0)
    { /* TURNOFFR       */
        d->negur   = 0;
        d->raccdot = 0;
        agc_chan_and(012, (uint16_t) ~PX_GIMR & 077777u);
        d->rgimtimr = LM_NEGMAX;
    }

    for (int i = 0; i < 3; i++)
    {
        d->jetctr[i] = lm_dap_dim(d->jetctr[i]); /* DIM PJETCTR..  */
    }

    if (!AGC_ASSERT(d->qgimtimr != 0 || d->rgimtimr != 0 ||
                    true == (st == AGC_STATUS_OK)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return st;
}

/* KALCMANU interface (pp.1421-1422): CDUxD = CDUxD - DELCDUx (modular, 2's
 * complement); then count down the direct-rate timers TCP, TCQR.         */
static void kalcmanu_step(lm_dap_t *d)
{
    d->cduxd = lm_dap_ones_to_twos(
        lm_dap_msu(d->cduxd, (uint16_t) ((uint16_t) d->delcdux & 077777u)));
    d->cduyd = lm_dap_ones_to_twos(
        lm_dap_msu(d->cduyd, (uint16_t) ((uint16_t) d->delcduy & 077777u)));
    d->cduzd = lm_dap_ones_to_twos(
        lm_dap_msu(d->cduzd, (uint16_t) ((uint16_t) d->delcduz & 077777u)));
    d->tcp  = lm_dap_dim(d->tcp);
    d->tcqr = lm_dap_dim(d->tcqr);

    if (!AGC_ASSERT(d->cduxd <= 077777u && d->cduyd <= 077777u))
    {
        return;
    }

    if (!AGC_ASSERT(d->cduzd <= 077777u))
    {
        return;
    }
}

/* PAXIS (pp.1421-1428): the T5RUPT program of the running DAP.            */
agc_status_t agc_task_paxis(void)
{
    lm_dap_t *d  = lm_dap_state();
    uint16_t  t5 = agc_rupt_time(5);

    agc_rupt_set_time(
        5, (uint16_t) ((t5 + LM_MS100) & 077777u)); /* ADS TIME5 */

    if (d->dapzrupt > 0)
    { /* previous jask unfinished */
        return agc_bailout(AGC_ALARM_02000);
    }

    bool         on = false;
    agc_status_t sc = lm_dap_chekbits(&on);

    if (sc != AGC_STATUS_OK || !on)
    {
        return sc;
    }

    d->dapcdu[0] = agc_erase_cdux(); /* DAPTREG4..6              */
    d->dapcdu[1] = agc_erase_cduy();
    d->dapcdu[2] = agc_erase_cduz();
    kalcmanu_step(d);
    agc_status_t st = AGC_STATUS_OK;

    if ((d->rcsflags & LM_RCS_CALLGMBL) != 0u)
    { /* PAXFILT       */
        st = lm_dap_acdt_c12();
    }

    d->dapzrupt = 1; /* SUPERJOB: jask running   */
    st          = (st == AGC_STATUS_OK) ? lm_dap_rateloop() : st;
    st          = (st == AGC_STATUS_OK) ? lm_dap_backp() : st;
    st          = (st == AGC_STATUS_OK) ? lm_dap_offset_est() : st;
    st          = (st == AGC_STATUS_OK) ? lm_dap_pretimck() : st;

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return st; /* DAPZRUPT stays positive   */
    }

    if ((d->rcsflags & LM_RCS_PSKIP) == 0u)
    {
        d->rcsflags = (uint16_t) (d->rcsflags | LM_RCS_PSKIP); /* SKIPPAXS */
    }
    else
    {
        bool policy = true;

        st = lm_dap_chkvisfz(&policy);

        if (st == AGC_STATUS_OK && policy)
        {
            st = lm_dap_pstate();
        }
    }

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return lm_dap_qraxis();
}
