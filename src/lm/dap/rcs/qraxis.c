/*
 * qraxis.c -- Q,R-axis control choice and attitude errors
 * (P-AXIS_RCS_AUTOPILOT.agc p.1441 QRAXIS; Q_R-AXIS_RCS_AUTOPILOT.agc,
 * Luminary 099 pp.1442-1452): CALLQERR, Q,RORGTS, TRYGTS / CHKINGTS,
 * RCS, SENSEGET (X translation), QRCONTRL, QERRCALC/RERRCALC, STILLRCS.
 *
 * Each 100 ms pass the Q,R axes are controlled either by the descent
 * engine trim gimbal (GTS, gts.c) or by the RCS jets about the
 * non-orthogonal U,V axes (qrjets.c).
 */
#include "agc/agc_assert.h"
#include "agc/agc_channels.h"
#include "agc/agc_lm_dap.h"
#include "agc/agc_lm_dap_internal.h"

#define QR_CH31_AUTO   0x1000u     /* BIT13 of 31 = 1: not attitude hold   */
#define QR_CH31_PLUSX  0x0040u     /* BIT7 of 31 = 0: +X translation       */
#define QR_CH31_MINUSX 0x0080u     /* BIT8 of 31 = 0: -X translation       */
#define QR_UBITS       000314u     /* 00314OCT: U-axis jets of channel 5   */

/* QERRCALC / RERRCALC (p.1451): Q,R attitude errors at pi from the Y,Z
 * gimbal-angle errors through M21..M32, plus the KALCMANU lags. Used by
 * the RCS and GTS laws and the eight-ball.                              */
agc_status_t lm_dap_qerrcalc(void)
{
    lm_dap_t *d = lm_dap_state();
    agc_sp_t ydif = lm_dap_msu(agc_erase_cduy(), d->cduyd);    /* DAPTEMP1 */
    agc_sp_t zdif = lm_dap_msu(agc_erase_cduz(), d->cduzd);    /* DAPTEMP2 */
    int32_t q = (int32_t)lm_dap_mphi(ydif, agc_erase_m21()) +
                lm_dap_mphi(zdif, agc_erase_m22()) + d->delqeror;
    int32_t r = (int32_t)lm_dap_mphi(ydif, agc_erase_m31()) +
                lm_dap_mphi(zdif, agc_erase_m32()) + d->delreror;
    d->qerror = lm_dap_wrap(q);                          /* XCH QERROR     */
    d->rerror = lm_dap_wrap(r);
    (void)AGC_ASSERT(d->qerror >= LM_NEGMAX && d->qerror <= LM_POSMAX);
    (void)AGC_ASSERT(d->rerror >= LM_NEGMAX && d->rerror <= LM_POSMAX);
    return AGC_STATUS_OK;
}

/* STILLRCS = ATTSTEER (p.1452): U,V attitude errors, then the TJLAW body
 * of the U,V loop.                                                       */
agc_status_t lm_dap_stillrcs(void)
{
    lm_dap_t *d = lm_dap_state();
    lm_dap_rot_touv(d->qerror, d->rerror, &d->uerror, &d->verror);
    (void)AGC_ASSERT(d->uerror >= LM_NEGMAX);
    (void)AGC_ASSERT(d->verror >= LM_NEGMAX);
    return lm_dap_uvloop(LM_UV_TJLAW, 1);
}

/* SENSEGET (pp.1442-1444): X translation from the THC (bits 7,8 of
 * channel 31) or the ullage request, into NEXTU/NEXTV; SENSETYP tells the
 * rotation logic which X sense to favour (0 none, 1 -X, 2 +X).          */
static void senseget(lm_dap_t *d)
{
    uint16_t ch31 = agc_chan_read(031);
    bool plus = ((ch31 & QR_CH31_PLUSX) == 0u) || (((ch31 & QR_CH31_MINUSX) != 0u) &&
                                                  agc_flag_test(AGC_FLAG_ULLAGFLG));
    bool minus = !plus && ((ch31 & QR_CH31_MINUSX) == 0u);
    if (!plus && !minus) {
        d->nextu = 0;
        d->nextv = 0;
        bool burning = !agc_flag_test(AGC_FLAG_DRIFTDFL);
        d->sensetyp = (burning && agc_flag_test(AGC_FLAG_APSFLAG)) ? 2 : 0; /* TSENSE */
        return;
    }
    d->rotindex = plus ? 5 : 4;                          /* +X0RULGE/-XTRANS */
    d->sensetyp = (agc_sp_t)(d->rotindex - 3);
    if (agc_flag_test(AGC_FLAG_ACC4_2FL)) {
        d->numbert = 4;                                  /* TRANS4         */
    } else {
        d->numbert = agc_flag_test(AGC_FLAG_AORBTFLG) ? 3 : 2;
    }
    bool found = lm_dap_selctsub();
    (void)AGC_ASSERT(found);          /* FAILOOP returns 0 before running out */
    if (!found || d->polytemp == 0) {
        d->polytemp = 0;
        (void)agc_alarm(AGC_ALARM_02002);
    }
    d->nextu = (uint16_t)((uint16_t)d->polytemp & QR_UBITS);    /* TSNEXTS */
    d->nextv = (uint16_t)((uint16_t)d->polytemp & (uint16_t)~QR_UBITS & 0377u);
}

/* RCS + SENSEGET + QRCONTRL (pp.1442-1444): Q,R by jets this pass. U,V
 * rate errors, X translation, then the mode: auto -> ATTSTEER; minimum
 * impulse (PULSES); otherwise rate command / attitude hold (CHEKSTIK).  */
agc_status_t lm_dap_rcs(void)
{
    lm_dap_t *d = lm_dap_state();
    d->cotroler = 0;
    lm_dap_rot_touv(d->edotq, d->edotr, &d->omegau, &d->omegav);
    senseget(d);
    (void)AGC_ASSERT(d->sensetyp >= 0 && d->sensetyp <= 2);
    if ((agc_chan_read(031) & QR_CH31_AUTO) != 0u) {     /* QRCONTRL       */
        return lm_dap_stillrcs();
    }
    if (agc_flag_test(AGC_FLAG_PULSEFLG)) {
        return lm_dap_qrminimp();
    }
    return lm_dap_chekstik();
}

/* TRYGTS / CHKINGTS (pp.1449-1450): use the trim gimbal unless jets are
 * mandatory (USEQRJTS), 1/ACCS has not allowed it (ALLOWGTS) or Q,R jets
 * are firing (channel 5). Leaving GTS sets up a damped nulling drive.  */
static agc_status_t trygts(lm_dap_t *d)
{
    if (agc_flag_test(AGC_FLAG_USEQRFLG) || d->allowgts == 0) {
        return lm_dap_rcs();
    }
    if (agc_chan_read(005) == 0u) {
        return lm_dap_gts();                             /* GOTOGTS        */
    }
    if (d->ingts != 0) {                                 /* CHKINGTS       */
        agc_inhint();
        agc_status_t st = agc_lm_dap_timegmbl();
        agc_relint();
        d->ingts = 0;
        (void)AGC_ASSERT(st == AGC_STATUS_OK);
    }
    (void)AGC_ASSERT(d->ingts == 0);
    return lm_dap_rcs();
}

/* QRAXIS / CALLQERR / Q,RORGTS (pp.1441-1442): Q,R rate errors; attitude
 * errors unless in manual rate command; then GTS, TRYGTS or RCS by
 * COTROLER (> 0 GTS, 0 try GTS, < 0 RCS).                               */
agc_status_t lm_dap_qraxis(void)
{
    lm_dap_t *d = lm_dap_state();
    d->edotr = lm_dap_sat((int32_t)d->omegar - d->omegard);   /* OVERSUB   */
    d->edotq = lm_dap_sat((int32_t)d->omegaq - d->omegaqd);
    bool automatic = (agc_chan_read(031) & QR_CH31_AUTO) != 0u;
    if (automatic || !agc_flag_test(AGC_FLAG_OURRCFLG)) {
        agc_status_t sq = lm_dap_qerrcalc();
        (void)AGC_ASSERT(sq == AGC_STATUS_OK);
    }
    (void)AGC_ASSERT(d->edotq >= LM_NEGMAX && d->edotr >= LM_NEGMAX);
    if (d->cotroler > 0) {
        return lm_dap_gts();
    }
    return (d->cotroler == 0) ? trygts(d) : lm_dap_rcs();
}
