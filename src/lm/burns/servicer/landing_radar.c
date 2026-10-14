/**********************************************************************
 * FILE NAME: landing_radar.c
 *
 * PURPOSE:
 *   The landing radar (LR) part of the lunar-landing SERVICER: MUNRETRN
 *   (the return point of MUNRVG) with the altitude tests ALTCHK, XORCHK and
 *   35KCHK, the high-gate antenna repositioning test (HIGATASK), CONTSERV,
 *   COPYCYC1/R29/NOR29NOW/COPYCYC2 (R10 data and the SM-to-reference copy),
 *   the LR position and velocity updates (UPDATCHK .. VALTCHK, GNURVST,
 *   HFAIL, VFAIL), and the LR jobs and tasks LRHTASK, LRHJOB, LRVJOB,
 *   RDGIMS, HIGATJOB, SETPOS1/2.
 *
 * AGC SOURCE:
 *   Luminary099 SERVICER.agc pp. 872-897
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale        Description
 *   ------------  ---------  ---  -----------  -----------------------
 *   CDUX/Y/Z      uint16_t   I    CDU counts   gimbal angles (RDGIMS)
 *   PIPAX/Y/Z     agc_sp_t   I    pulses       PIPAs at the V reading
 *   PIPTIME       agc_dp_t   I    cs 2^28      time of R, V
 *   REFSMMAT      agc_mat_t  I    2^1          SM to reference
 *   MODREG        agc_sp_t   I    -            major mode (P65-P67 weights)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_radar_*           LR reading (RADSTALL, SAMPLSUM; lm/radar.h)
 *   agc_radar_powered_flight_designate   R29 (lm/radar.h)
 *   agc_imu_nbsm         *NBSM* (powered_flight.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ALARM 00511 (LR antenna not in position, LRPOSALM) and ALARM 00523
 *   (LR antenna did not reach position 2). Radar, executive and waitlist
 *   failures come back as their status.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   LM only. MUNRETRN .. COPYCYC2 run inside the SERVICER job; the LR
 *   jobs and tasks on their own. Restart group 2 (R12).
 *
 * NOTES:
 *   See docs/modules/guid-powered.md (LR weights and limits are pad loads).
 *
 * ALGORITHM:
 *   Scales: R1S m 2^24 (SM), V1S m/cs 2^7, DELVS m/cs 2^5, HCALC m
 *   2^24, HBEAMNB/VBEAMNB half unit (NB), XNBPIP half unit; HMEAS 1.079
 *   ft/bit, VMEAS LR velocity counts (VZSCAL..: ft/s/bit to m/cs 2^18).
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/guidance/guidance.h"
#include "agc/io/service.h"
#include "internal.h"

#define RDGIMS_DELAY 17 /* 170MS EQUALS ND1 (OCT 21 = 17 cs)          */
#define LRV_WAIT_CS  2  /* LRVJOB, HIGATJOB: recheck RADCADR every 20 ms (DEVIATION) */

/* ALTCRIT table (p.878), m 2^24: 25KFT 7620 m, 50KFT 15240 m, 50FT 15.24 m,
 * 30KFT 9144 m (indices 0, 2, 4, 6 as in the listing).                     */
static const agc_dp_t ALTCRIT[4] = {121920, 243840, 244, 146304};

/* 2KFT/SEC DEC 6.096 B-7: 2000 ft/s at m/cs 2^7 (SP).                      */
static const agc_sp_t TWOKFT_SEC = 780; /* 2000 ft/s, m/cs 2^7 */

/* 7.5 2DEC .02286 B-6: 7.5 ft/s at m/cs 2^6.                               */
static const agc_dp_t SEVEN_HALF = 95882; /* 7.5 ft/s, m/cs 2^6 */

/* ALTCONV 2DEC 1.399078846 B-4: m 2^24 -> R10 bit units 2^28.              */
static const agc_dp_t ALTCONV = 23472648; /* m 2^24 to R10 bits 2^28 */

/* ARCONV1 2DEC 656.167979 B-10: altitude-rate compensation to bit units.   */
static const agc_dp_t ARCONV1 = 172010499; /* altitude-rate compensation, 2^10 */

/* 2SEC(28) 2OCT 0000000310: 200 cs at 2^28.                                */
static const agc_dp_t TWOSEC_28 = 200; /* 2 s, cs 2^28 */

/* KPIP1 2DEC .0128: pulses (high word) to m/cs 2^7.                       */
static const agc_dp_t KPIP1_LM = 3435974; /* KPIP1: pulses to m/cs 2^7 */

/* HBEAMANT (CONTROLLED_CONSTANTS p.~): range beam in LR antenna axes.     */
static const agc_dp_t HBEAMANT[3] = {-125816183, 0, -46740633};

/* HSCAL 2DEC -.3288792: 1.079 ft/bit to m 2^22.                            */
static const agc_dp_t HSCAL = -88282838; /* 1.079 ft/bit to m 2^22 */

/* VZSCAL VYSCAL VXSCAL: .8668, 1.212, -.644 ft/s/bit to m/cs 2^18.         */
static const agc_dp_t VSCAL[3] = {145245838, 203089473, -107912228};

/**********************************************************************
 * ALTCHK (p.878): is HCALC at or below the criterion ALTCRIT + index?
 *
 * FUNCTION NAME: agc_lm_altchk
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   index     agc_sp_t    I    ALTCRIT table index (0, 2, 4, 6)
 *   below     bool *      O    HCALC at or below the criterion
 **********************************************************************/
static void agc_lm_altchk(agc_sp_t index, bool *below)
{
    agc_ovf_t ovf = 0;                                                                    /* overflow of the DAS (not tested, as in the AGC) */
    agc_dp_t  d   = agc_dp_sub_raw(ALTCRIT[index / 2], agc_guid_state()->lm.hcalc, &ovf); /* ALTCRIT - HCALC, m 2^24 */

    *below = (d >= 0); /* BRANCH +4: L+4 if <=   */
}

/**********************************************************************
 * GNURVST (p.889-890): store GNUR (= GNUV) in R1S (A = 0) or V1S (A = 6).
 *
 * FUNCTION NAME: gnurvst
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   velocity  bool        I    velocity update (else altitude)
 **********************************************************************/
static void gnurvst(bool velocity)
{
    agc_guid_lm_t *lm = &agc_guid_state()->lm; /* LM guidance erasables */

    if (velocity)
    {
        lm->v1s = lm->gnur;
    }
    else
    {
        lm->r1s = lm->gnur;
    }
}

/**********************************************************************
 * CONTSERV (p.875): clear the "measurement made" discretes RNGEDATA and
 * VELDATA (BITS4-7, OCT 110), set the LR lamps, go to COPYCYC1.
 *
 * FUNCTION NAME: agc_lm_serv_contserv
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_lm_serv_contserv(void)
{
    agc_inhint();
    agc_flag_clear(AGC_FLAG_RNGEDATA);
    agc_flag_clear(AGC_FLAG_VELDATA);
    agc_radar_landing_radar_lights(); /* SET LR LITES PROPERLY  */
    agc_relint();

    return (agc_lm_serv_copycyc1());
}

/**********************************************************************
 * HIGATASK (p.873): about 6 s before high gate set PSTHIGAT and NOLRREAD
 * (PRIO3 = bits 10, 11 of FLGWRD11) and start HIGATJOB (POS1 -> POS2).
 *
 * FUNCTION NAME: higatask
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t higatask(void)
{
    agc_inhint();
    agc_flag_set(AGC_FLAG_PSTHIGAT);
    agc_flag_set(AGC_FLAG_NOLRREAD);
    agc_status_t st = agc_exec_findvac(AGC_PRIO32, AGC_JOB_HIGATJOB); /* HIGATJOB */

    agc_relint();

    if (st == AGC_STATUS_OK)
    {
        st = agc_lm_serv_contserv();
    }

    return (st);
}

/**********************************************************************
 * MUNRETRN (p.874) with XORCHK and 35KCHK (p.878-879): LR reading enabled
 * below 50 000 ft (the code's 35KCHK uses ALTCRIT +2 = 50KFT), X-axis
 * override inhibited below 30 000 ft.
 *
 * FUNCTION NAME: agc_lm_serv_munretrn
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_lm_serv_munretrn(void)
{
    if (agc_flag_test(AGC_FLAG_LRBYPASS))
    {
        return (agc_lm_serv_copycyc1()); /* BYPASS LR LOGIC        */
    }

    bool below = false; /* below the ALTCHK altitude */

    if (!agc_flag_test(AGC_FLAG_READLR))
    { /* 35KCHK                 */
        agc_lm_altchk(2, &below);

        if (below)
        {
            agc_flag_set(AGC_FLAG_READLR);
        }

        return (agc_lm_serv_contserv());
    }

    if (!agc_flag_test(AGC_FLAG_XORFLG))
    { /* XORCHK                 */
        agc_lm_altchk(6, &below);

        if (below)
        {
            agc_flag_set(AGC_FLAG_XOVINFLG);
            agc_flag_set(AGC_FLAG_XORFLG);
        }
    }

    /* HITEST                 */
    /* The antenna-position checks of MUNRETRN (p.874-875): HITEST, POS2CHK,
     * HIGATCHK, POS1CHK, LRPOSALM. Channel 33 is inverted (0 = in position). */
    agc_guid_lm_t *lm   = &agc_guid_state()->lm; /* LM guidance erasables */
    uint16_t       ch33 = agc_chan_read(033);    /* channel 33 (inverted inputs) */

    if (agc_flag_test(AGC_FLAG_PSTHIGAT))
    { /* POS2CHK                */
        if ((ch33 & (uint16_t) AGC_CH033_LRPOS2) == 0U)
        {
            return (agc_lm_serv_updatchk());
        }

        if ((agc_chan_read(012) & (uint16_t) AGC_CH012_LRPOS2CMD) != 0U)
        {
            return (agc_lm_serv_contserv()); /* BEING REPOSITIONED     */
        }
    }
    else
    { /* HIGATCHK               */
        agc_sp_t ttf8 = agc_lm_ttf8_high(); /* CA TTF/8; TTF/8 high word, cs 2^17 */

        if ((int) ttf8 + lm->rpcrtime > 0 && (int) lm->rpcrtqsw - (int) agc_dp_high(lm->xnbpip.row[0].c[0]) <= 0)
        {
            return (higatask()); /* UXBXP > QSW: REPOSITION */
        }

        if ((ch33 & (uint16_t) AGC_CH033_LRPOS1) == 0U)
        { /* POS1CHK         */
            return (agc_lm_serv_updatchk());
        }
    }

    agc_alarm(AGC_ALARM_00511); /* LRPOSALM               */

    return (agc_lm_serv_contserv());
} /* agc_lm_serv_munretrn */

/**********************************************************************
 * NOR29NOW .. COPYCYC2 (p.876-877): HCALC, ALTBITS and UHZP for R10, the
 * reference-frame RN1/VN1, the altitude-rate compensation, then R and V
 * replaced by R1S and V1S and the common COPYCYCL.
 *
 * FUNCTION NAME: no_powered_designate_now
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t no_powered_designate_now(void)
{
    agc_guid_t   *g   = agc_guid_state(); /* guidance erasables */
    agc_lm_ldg_t *ldg = agc_lm_ldg();     /* landing guidance erasables */
    agc_interp_t  ip;                     /* interpreter state */

    agc_interp_init(&ip);
    agc_dp_t r1mag = agc_vec_abval(&ip, g->lm.r1s); /* PUSH; |R1S|, m 2^24 */

    g->lm.hcalc        = agc_dp_sub(&ip, r1mag, ldg->land_mag);
    ldg->hcalc1        = g->lm.hcalc;
    g->lm.altbits      = agc_dp_mul(&ip, g->lm.hcalc, ALTCONV); /* DMPR, SGNAGREE */
    g->lm.uhzp         = agc_vec_unit(&ip, agc_vec_cross(&ip, g->avg.unitr, g->lm.uhyp), NULL, NULL);
    agc_mat_t refsmmat = agc_erase_refsmmat();                  /* REFSMMAT, half-unit rows (2^1) */

    g->avg.rn1    = agc_vec_shr(agc_vec_mat(&ip, g->lm.r1s, &refsmmat), 4);                              /* 2^29 */
    g->avg.vn1    = agc_vec_shl(&ip, agc_vec_mat(&ip, g->lm.v1s, &refsmmat), 1);
    agc_dp_t vh   = agc_dp_shl(&ip, agc_vec_abval(&ip, agc_vec_cross(&ip, g->avg.unitr, g->lm.v1s)), 1); /* horizontal velocity |UNITR x V1S|, m/cs 2^6 */
    agc_dp_t comp = agc_dp_mul(&ip, agc_dp_div(&ip, agc_dp_sq(vh), r1mag), ARCONV1);                     /* altitude-rate compensation (DALTRATE), bits; scale 2^28 */

    agc_inhint(); /* COPYCYC2     */

    for (int i = 0; i < 3; i++)
    {
        g->lm.runit[i] = agc_dp_high(g->avg.unitr.c[i]); /* RUNIT FOR R10 */
    }

    g->lm.daltrate = agc_dp_high(comp); /* DALTRATE     */
    g->lm.r        = g->lm.r1s;
    g->lm.v        = g->lm.v1s;
    agc_relint();

    return (agc_lm_serv_copycycl());
}

/**********************************************************************
 * COPYCYC1 / R29? (p.876): R29 designates the rendezvous radar unless
 * NOR29FLG or READRFLG is set, the RR CDUs are being zeroed or it is not
 * in auto (RADMODES bits 2, 13), or it is repositioning or remoding (bits
 * 11, 14 -> try again in 2 s).
 *
 * FUNCTION NAME: agc_lm_serv_copycyc1
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_lm_serv_copycyc1(void)
{
    agc_lm_serv_quikfaz5(AGC_JOB_SERV_COPYCYC1);
    uint16_t     radmodes = agc_flagword_get(12); /* FLGWRD12 = RADMODES    */
    agc_status_t st       = AGC_STATUS_OK;        /* status of R29         */

    if (agc_flag_test(AGC_FLAG_NO_POWERED_DESIGNATE) || agc_flag_test(AGC_FLAG_READRFLG) || (radmodes & (uint16_t) 010002) != 0U)
    {
        agc_inhint();                      /* R29NODES               */
        agc_flag_clear(AGC_FLAG_DESIGFLG); /* DESIGNATION IS OFF     */
        agc_relint();
    }
    else if ((radmodes & (uint16_t) 022000) == 0U)
    {
        st = agc_radar_powered_flight_designate(); /* TCF R29 (-> NOR29NOW) */
    }
    else
    {
        /* repositioning or remoding: NOR29NOW, R29 tried again in 2 s */
    }

    if (st == AGC_STATUS_OK)
    {
        st = no_powered_designate_now();
    }

    return (st);
}

/**********************************************************************
 * HFAIL (p.890): a bad altitude. Tracker-fail lamp if the previous bad
 * reading was fewer than four readings ago (L - R < 4), then R = L.
 *
 * FUNCTION NAME: hfail
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t hfail(void)
{
    agc_guid_lm_t *lm = &agc_guid_state()->lm; /* LM guidance erasables */

    if (lm->lrrctr != 0 && ((lm->lrlctr - lm->lrrctr) & ~3) == 0)
    {
        agc_flag_set(AGC_FLAG_HFLSHFLG); /* TRACKER FAIL LITE      */
    }

    lm->lrrctr = lm->lrlctr; /* NORLITE: R = L         */

    return (agc_lm_serv_vmeaschk());
}

/**********************************************************************
 * RUPDATED (p.886): R1S = GNUR, then VMEASCHK.
 *
 * FUNCTION NAME: agc_lm_serv_rupdated
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_lm_serv_rupdated(void)
{
    gnurvst(false);

    return (agc_lm_serv_vmeaschk());
}

/**********************************************************************
 * UPDATCHK / POSUPDAT / NOREASON (p.884-886).
 *
 * FUNCTION NAME: agc_lm_serv_updatchk
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_lm_serv_updatchk(void)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    if (agc_flag_test(AGC_FLAG_NOLRREAD))
    {
        return (agc_lm_serv_contserv()); /* LR UPDATE INHIBITED    */
    }

    if (!agc_flag_test(AGC_FLAG_RNGEDATA))
    {
        return (agc_lm_serv_vmeaschk()); /* NO ALT MEAS THIS CYCLE */
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);

    /* The altitude measurement of POSUPDAT (p.884): DELTAH = (corrected slant
     * range along the beam) . UNIT/R/ - HCALC, m 2^24. The doppler correction
     * (V relative to the surface along the beam times RADSKAL, times SKALSKAL
     * on the low scale) is added to HMEAS before scaling by HSCAL.            */
    agc_vec_t hbsm = agc_vec_mat(&ip, g->lm.hbeamnb, &g->lm.xnbpip);                /* 2^2   */
    agc_vec_t vrel = agc_vec_add(&ip, agc_vec_shl(&ip, g->lm.v1s, 2), g->lm.delvs); /* V relative to the surface, m/cs 2^5 */
    agc_dp_t  dop  = agc_dp_mul_trunc(agc_vec_dot(&ip, vrel, hbsm), g->lm.radskal); /* doppler correction, 1.079 ft/bit; scale 2^17 */

    if (!agc_flag_test(AGC_FLAG_ALTSCALE))
    { /* LOW SCALE: RESCALE     */
        dop = agc_tp_to_dp(agc_dp_mul_tp(dop, agc_dp_from_sp(g->lm.skalskal)));
    }

    agc_dp_t  slant = agc_dp_shl(&ip, agc_dp_add(&ip, dop, g->lm.hmeas), 7);    /* slant range, 1.079 ft/bit 2^-7 */
    agc_vec_t sv    = agc_vec_scale(&ip, hbsm, agc_dp_mul_trunc(slant, HSCAL)); /* 2^23 */

    if (!AGC_ASSERT(g->lm.hmeas >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t dh = agc_dp_sub(&ip, agc_vec_dot(&ip, sv, g->avg.unitr), g->lm.hcalc); /* DELTAH, m 2^24 */

    agc_lm_ldg()->deltah = dh; /* STORE DELTAH           */

    if (agc_flag_test(AGC_FLAG_PSTHIGAT))
    { /* REASONABLENESS TEST    */
        agc_dp_t t = agc_dp_sub(&ip, agc_dp_shl(&ip, agc_dp_sub(&ip, agc_dp_abs(dh), g->lm.delqfix), 3), g->lm.hcalc); /* |DELTAH| test, m 2^24 */

        g->lm.lrlctr++;

        if (t >= 0)
        {
            return (hfail()); /* DELTA H TOO LARGE      */
        }

        agc_flag_clear(AGC_FLAG_HFLSHFLG); /* ALT FAIL LAMP OFF      */
    }

    if (!agc_flag_test(AGC_FLAG_LRINH))
    {
        return (agc_lm_serv_vmeaschk()); /* UPDATE INHIBITED       */
    }

    /* NOREASON position update (p.885): below HMAX,
     *   GNUR = R1S + DELTAH * WH * (1 - H/HMAX) * UNIT/R/ (SL1), then MUNGRAV. */
    agc_dp_t h28 = agc_dp_shr(g->lm.hcalc, 4);     /* RESCALE H TO 2(28)M    */
    agc_sp_t hhi = agc_dp_high(h28);               /* H high word; scale 2^28 */
    agc_sp_t hlo = agc_dp_low(h28);                /* H AT 2(14) M; scale 2^14 */
    int      d   = (int) g->lm.lrhmax - (int) hlo; /* LRHMAX - H, m 2^14 */

    if (hhi != 0 || d <= 0 || !AGC_ASSERT(g->lm.lrhmax > 0))
    {
        return (agc_lm_serv_vmeaschk()); /* H > HMAX: BYPASS       */
    }

    agc_ovf_t ovf        = 0;                                                                                        /* overflow of the DAS (not tested, as in the AGC) */
    agc_dp_t  wq         = agc_dp_div_raw(agc_sp_mul((agc_sp_t) d, g->lm.lrwh), agc_dp_from_sp(g->lm.lrhmax), &ovf); /* MP DV; altitude weight, scale 2^0 */
    agc_dp_t  dh_rupdate = agc_tp_to_dp(agc_dp_mul_tp(agc_lm_ldg()->deltah, agc_dp_from_sp(agc_dp_high(wq))));       /* SHORTMP2; weighted DELTAH, m 2^24 */
    agc_vec_t dr         = agc_vec_scale(&ip, g->avg.unitr, agc_dp_shl(&ip, dh_rupdate, 1));                         /* position correction, m 2^24 */

    g->lm.gnur    = agc_vec_add(&ip, dr, g->lm.r1s);
    g->avg.gdt1_2 = agc_lm_mungrav(&ip, g->lm.gnur); /* GDT1/2 */
    agc_lm_serv_quikfaz5(AGC_JOB_SERV_RUPDATED);

    return (agc_lm_serv_rupdated());
} /* agc_lm_serv_updatchk */

/**********************************************************************
 * VFAIL / NOLITE (p.890-891): a bad velocity reading.
 *
 * FUNCTION NAME: vfail
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t vfail(void)
{
    agc_guid_lm_t *lm = &agc_guid_state()->lm; /* LM guidance erasables */

    if (lm->lrsctr != 0 && ((lm->lrmctr - lm->lrsctr) & ~3) == 0)
    {
        agc_flag_set(AGC_FLAG_VFLSHFLG); /* TRACKER FAIL LITE      */
    }

    lm->lrsctr = lm->lrmctr; /* NOLITE: S = M          */

    if (lm->vselect == 0)
    { /* Z COMPONENT            */
        agc_flag_set(AGC_FLAG_VXINH); /* SKIP X (CROSS LOBE)    */
    }

    return (agc_lm_serv_valtchk()); /* ENDVDAT                */
}

/**********************************************************************
 * VELUPDAT (p.886-887): the measured velocity along the beam (VMEAS SL 12
 * times VZSCAL.., m/cs 2^6) against the estimate V(est) = (V + DELVU +
 * G(N-1) (TU - T(N-1)) VSL2 + DELVS) . VBEAMSM. Returns DELTAV (2^6),
 * the beam in SM (half unit) and the limit 7.5 + |VM|/8 (2^6).
 *
 * FUNCTION NAME: lr_deltav
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   g         agc_guid_t *    I/O  guidance erasables
 *   vbsm      agc_vec_t *     O    LR beam vector in SM
 *   lim       agc_dp_t *      O    reasonability limit
 *
 * RETURNS: agc_dp_t -- DELTAV, measured less estimated beam velocity,
 *          m/cs 2^6
 **********************************************************************/
static agc_dp_t lr_deltav(agc_interp_t *ip, agc_guid_t *g, agc_vec_t *vbsm, agc_dp_t *lim)
{
    int            s        = g->lm.vselect; /* VSELECT, beam index */
    const uint16_t lrcdu[3] = {g->lm.lrcdu[1], g->lm.lrcdu[2], g->lm.lrcdu[0]};

    agc_imu_quictrig(agc_imu_trig(), lrcdu);                                                                                                                        /* CDUSPOT = LRY LRZ LRX  */
    *vbsm           = g->lm.vbeamnb[s];
    agc_imu_nbsm(agc_imu_trig(), vbsm);                                                                                                                             /* *NBSM*                 */
    agc_dp_t  vm    = agc_dp_mul_trunc(agc_dp_shl(ip, g->lm.vmeas, 12), VSCAL[s]);                                                                                  /* VM measured beam velocity, m/cs 2^6 */
    agc_vec_t delvu = agc_vec_scale(ip, agc_vec_make(agc_dp_pack(g->lm.piptem[0], 0), agc_dp_pack(g->lm.piptem[1], 0), agc_dp_pack(g->lm.piptem[2], 0)), KPIP1_LM); /* DELVU since the PIPA reading, m/cs 2^7 */
    agc_dp_t  frac  = agc_dp_div(ip, agc_dp_sub(ip, g->lm.lrvtime, agc_erase_piptime()), TWOSEC_28);                                                                /* (LRVTIME - PIPTIME) / 2 s, 2^0 */
    agc_vec_t gt    = agc_vec_shl(ip, agc_vec_scale(ip, g->avg.gdt_2, frac), 1);                                                                                    /* gravity term to LRVTIME, m/cs 2^7 */
    agc_vec_t vu    = agc_vec_add(ip, agc_vec_add(ip, gt, g->lm.v), delvu);                                                                                         /* V(est) before VSL2, m/cs 2^7 */

    vu   = agc_vec_add(ip, agc_vec_shl(ip, vu, 2), g->lm.delvs); /* 2^5         */
    *lim = agc_dp_add(ip, agc_dp_shr(agc_vec_abval(ip, vu), 4), SEVEN_HALF);

    return (agc_dp_sub(ip, vm, agc_vec_dot(ip, vu, *vbsm))); /* BDSU        */
}

/**********************************************************************
 * VMEASCHK / VELUPDAT / VUPDAT (p.886-889).
 *
 * FUNCTION NAME: agc_lm_serv_vmeaschk
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_lm_serv_vmeaschk(void)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    agc_lm_serv_quikfaz5(AGC_JOB_SERV_VMEASCHK);

    if (!agc_flag_test(AGC_FLAG_VELDATA))
    {
        return (agc_lm_serv_valtchk()); /* NO V READING           */
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_vec_t vbsm;                                 /* beam direction in SM, half unit; (2^1) */
    agc_dp_t  lim = 0;                              /* DELTAV limit, m/cs 2^6 */
    agc_dp_t  dv  = lr_deltav(&ip, g, &vbsm, &lim); /* DELTAV, m/cs 2^6 */

    g->lm.lrmctr++;

    if (agc_dp_sub(&ip, agc_dp_abs(dv), lim) >= 0)
    {
        return (vfail()); /* DELTA V TOO LARGE      */
    }

    agc_flag_clear(AGC_FLAG_VFLSHFLG); /* VEL FAIL LAMP OFF      */

    if (agc_flag_test(AGC_FLAG_VXINH))
    {
        agc_flag_clear(AGC_FLAG_VXINH);

        if (g->lm.vselect == 2)
        {
            return (agc_lm_serv_valtchk()); /* BYPASS X UPDATE        */
        }
    }

    if (!agc_flag_test(AGC_FLAG_LRINH))
    {
        return (agc_lm_serv_valtchk()); /* UPDATE INHIBITED       */
    }

    /* The velocity weight of VUPDAT (p.888-889): WVF below VF, 0 above VMAX,
     * else WV (1 - V/VMAX); the P65-P67 constant LRWVFF in modes above 64.     */
    const agc_guid_lm_t *lm = &g->lm; /* LM guidance erasables */

    agc_sp_t v = agc_dp_high(lm->abvel); /* ABVEL*; |V| high word, m/cs 2^7 */
    int      s = lm->vselect;            /* VSELECT, beam index */
    agc_sp_t w;                          /* weight of the update, 2^0 */

    if (v - lm->lrvf <= 0)
    {
        w = lm->lrwvf[s]; /* USEVF                  */
    }
    else if (lm->lrvmax - v <= 0)
    {
        w = 0; /* V > VMAX: W = 0        */
    }
    else
    {
        agc_ovf_t ovf = 0;                                                                                                      /* overflow of the DAS (not tested, as in the AGC) */
        agc_dp_t  q   = agc_dp_div_raw(agc_sp_mul((agc_sp_t) (lm->lrvmax - v), lm->lrwv[s]), agc_dp_from_sp(lm->lrvmax), &ovf); /* weight reduction above LRVMAX, 2^0 */

        w = agc_dp_high(q); /* WV (1 - V/VMAX)        */
    }

    if (agc_erase_modreg() - 64 > 0)
    {
        w = lm->lrwvff; /* P65, P66, P67          */
    }

    if (!AGC_ASSERT(s >= 0 && s <= 2))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t wdv = agc_dp_mul_trunc(agc_dp_from_sp(w), dv); /* weighted DELTAV, m/cs 2^6 */

    g->lm.gnur = agc_vec_add(&ip, agc_vec_scale(&ip, vbsm, wdv), g->lm.v1s); /* GNUV */
    agc_lm_serv_quikfaz5(AGC_JOB_SERV_VUPDATED);

    return (agc_lm_serv_vupdated());
} /* agc_lm_serv_vmeaschk */

/**********************************************************************
 * VUPDATED (p.889): V1S = GNUV, then VALTCHK.
 *
 * FUNCTION NAME: agc_lm_serv_vupdated
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_lm_serv_vupdated(void)
{
    gnurvst(true);

    return (agc_lm_serv_valtchk());
}

/**********************************************************************
 * VALTCHK / READV (p.889): start a velocity reading (LRVJOB) once below
 * 2000 ft/s (READVEL), then CONTSERV.
 *
 * FUNCTION NAME: agc_lm_serv_valtchk
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_lm_serv_valtchk(void)
{
    agc_lm_serv_quikfaz5(AGC_JOB_SERV_VALTCHK);

    if (!agc_flag_test(AGC_FLAG_READVEL))
    {
        if (TWOKFT_SEC - agc_dp_high(agc_guid_state()->lm.abvel) <= 0)
        {
            return (agc_lm_serv_contserv()); /* V > 2000 FT/SEC        */
        }

        agc_flag_set(AGC_FLAG_READVEL);
    }

    agc_status_t st = agc_exec_novac(AGC_PRIO32, AGC_JOB_LRVJOB); /* READV                  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_lm_serv_contserv());
}

/**********************************************************************
 * LRHTASK (p.872): waitlist task (below 25 000 ft) 50 ms before READACCS:
 * start LRHJOB unless LR is bypassed, not to be read or repositioning;
 * group 2 off, then R10,R11A.
 *
 * FUNCTION NAME: agc_lm_lrhtask
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_lm_lrhtask(void)
{
    if (!agc_flag_test(AGC_FLAG_LRBYPASS) && agc_flag_test(AGC_FLAG_READLR) && !agc_flag_test(AGC_FLAG_NOLRREAD))
    {
        agc_status_t st = agc_exec_novac(AGC_PRIO32, AGC_JOB_LRHJOB); /* status of agc_exec_novac */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    agc_phase_kill(2); /* GRP2OFF                */

    return (agc_lm_abort_monitor_resume());
}

/**********************************************************************
 * RDGIMS (p.894): task about midway through the 5 velocity samples: time,
 * CDUs and PIPA counts for the velocity update.
 *
 * FUNCTION NAME: agc_task_rdgims
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
void agc_task_rdgims(void)
{
    agc_guid_lm_t *lm = &agc_guid_state()->lm; /* LM guidance erasables */

    lm->lrvtime   = (agc_dp_t) agc_time_now_cs(); /* DCA TIME2              */
    lm->lrcdu[0]  = agc_erase_cdux();
    lm->lrcdu[1]  = agc_erase_cduy();
    lm->lrcdu[2]  = agc_erase_cduz();
    lm->piptem[0] = agc_erase_pipax();
    lm->piptem[1] = agc_erase_pipay();
    lm->piptem[2] = agc_erase_pipaz();
}

/**********************************************************************
 * LRVJOB (p.892): read the next velocity beam (VSELECT cycles 2, 1, 0),
 * sleep in RADSTALL for about 500 ms.
 * DEVIATION: LRVJOB first waits (DELAYJOB, LRV_WAIT_CS) while another job
 * sleeps in RADSTALL (RADCADR holds a job). In the AGC the two LR reads
 * never meet: LRHTASK starts LRHJOB (one altitude sample, about 90 ms in
 * RADSTALL) 50 ms before READACCS, and SERVICER spends its own AGC time
 * on the navigation update before VALTCHK starts LRVJOB. The C SERVICER
 * reaches VALTCHK in no AGC time, 5 cs after LRHJOB stalled, and the
 * second RADSTALL was BAILOUT 01210 (two programs using one device) on
 * every 2 s pass below 2000 ft/s: no LR update was ever taken (NASSP PDI,
 * 2026-10-13). The wait stands in for that SERVICER time.
 *
 * FUNCTION NAME: agc_job_lrvjob
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_lrvjob(void)
{
    if (agc_erase_modecadr(AGC_IMU_DEV_RADAR) > 0)
    { /* LRHJOB still in RADSTALL: wait for it (DEVIATION above) */
        return (agc_svc_delayjob(LRV_WAIT_CS, AGC_JOB_LRVJOB));
    }

    agc_guid_lm_t *lm = &agc_guid_state()->lm;                            /* LM guidance erasables */
    agc_status_t   sw = agc_waitlist_call(RDGIMS_DELAY, AGC_TASK_RDGIMS); /* status of agc_waitlist_call */

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    agc_sp_t beam = (lm->vselect > 0) ? (agc_sp_t) (lm->vselect - 1) : 2; /* next beam; unscaled */

    agc_radar_lrvel((agc_sp_t) (2 * beam));                                                       /* 2 X VSELECT INDEX      */
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING;                                          /* RADSTALL outcome */
    agc_status_t           st  = agc_imu_stall_dev(AGC_IMU_DEV_RADAR, AGC_JOB_LRVJOB_DONE, &res); /* status of agc_imu_stall_dev */

    if (st != AGC_STATUS_OK || res == AGC_IMU_STALL_SLEEPING)
    {
        return (st);
    }

    agc_job_lrvjob_done();

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * LRVJOB after RADSTALL / ENDLRV / VBAD / VSTILBAD (p.892-893); a good
 * reading also keeps LRVTIME, LRXCDU, LRYCDU, LRZCDU for the downlink
 * (LRVTIMDL, LRXCDUDL, LRYCDUDL, LRZCDUDL).
 *
 * FUNCTION NAME: agc_job_lrvjob_done
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
void agc_job_lrvjob_done(void)
{
    agc_guid_lm_t *lm = &agc_guid_state()->lm; /* LM guidance erasables */

    if (agc_imu_stall_outcome(AGC_IMU_DEV_RADAR) != AGC_IMU_STALL_GOOD)
    {
        lm->stilbadv = 2; /* VBAD: WAIT 4 SECONDS   */
    }
    else if (lm->stilbadv > 0)
    {
        lm->stilbadv--; /* JUST GOOD -- WAIT      */
    }
    else
    {
        agc_inhint();
        lm->vmeas      = agc_radar_samplsum(); /* STOW AWAY VMEAS        */
        lm->lrvtimdl   = lm->lrvtime;          /* DCA LRVTIME, DXCH LRVTIMDL (FOR DOWNLINK) */
        lm->lrcdudl[0] = lm->lrcdu[0];         /* DCA LRXCDU, DXCH LRXCDUDL */
        lm->lrcdudl[1] = lm->lrcdu[1];
        lm->lrcdudl[2] = lm->lrcdu[2];         /* CA LRZCDU, TS LRZCDUDL  */
        agc_flag_set(AGC_FLAG_VELDATA);
        agc_relint();
    }

    lm->vselect = (lm->vselect > 0) ? (agc_sp_t) (lm->vselect - 1) : 2; /* ENDLRV */
}

/**********************************************************************
 * LRHJOB (p.893): one altitude sample, sleep about 95 ms.
 *
 * FUNCTION NAME: agc_job_lrhjob
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_lrhjob(void)
{
    agc_radar_lralt();
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING;                                          /* RADSTALL outcome */
    agc_status_t           st  = agc_imu_stall_dev(AGC_IMU_DEV_RADAR, AGC_JOB_LRHJOB_DONE, &res); /* status of agc_imu_stall_dev */

    if (st != AGC_STATUS_OK || res == AGC_IMU_STALL_SLEEPING)
    {
        return (st);
    }

    return (agc_job_lrhjob_done());
}

/**********************************************************************
 * LRHJOB after RADSTALL / HBAD / HSTILBAD (p.893-894): a bad return caused
 * by a range-scale change is ignored (RNGSCFLG reset).
 *
 * FUNCTION NAME: agc_job_lrhjob_done
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed)
 **********************************************************************/
agc_status_t agc_job_lrhjob_done(void)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    if (agc_imu_stall_outcome(AGC_IMU_DEV_RADAR) != AGC_IMU_STALL_GOOD)
    {
        if (agc_flag_test(AGC_FLAG_RNGSCFLG))
        {
            agc_flag_clear(AGC_FLAG_RNGSCFLG);

            return (AGC_STATUS_OK); /* SCALE CHANGE: IGNORE   */
        }

        g->lm.stilbadh = 2; /* WAIT 4 SECONDS         */

        return (AGC_STATUS_OK);
    }

    if (g->lm.stilbadh > 0)
    {
        g->lm.stilbadh--; /* HSTILBAD               */
    }
    else
    {
        agc_inhint();
        g->lm.hmeas  = agc_radar_samplsum();         /* 1.079 FT/BIT           */
        g->lm.mktime = g->avg.piptime1;
        g->lm.aig    = (agc_sp_t) g->avg.cdutemp[1]; /* CDUY,Z = AIG,AMG       */
        g->lm.amg    = (agc_sp_t) g->avg.cdutemp[2];
        g->lm.aog    = (agc_sp_t) g->avg.cdutemp[0];
        agc_flag_set(AGC_FLAG_RNGEDATA);
        agc_relint();

        if (!AGC_ASSERT(g->lm.hmeas >= 0))
        {
            return (AGC_STATUS_BAD_ARG);
        }
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * SETPOS (p.896): antenna rotations LRALPHA (about X) and LRBETA (about Y)
 * for position pos into CDUSPOT; the beam vectors to NB axes:
 * VYBEAMNB = TRG*SMNB(UNITY), VXBEAMNB = *SMNB*(UNITX),
 * VZBEAMNB = VXBEAMNB x VYBEAMNB (VSL1), HBEAMNB = *SMNB*(HBEAMANT).
 *
 * FUNCTION NAME: setpos
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   pos       int         I    LR antenna position (1 or 2)
 *
 * RETURNS: none
 **********************************************************************/
static void setpos(int pos)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    const uint16_t cdu[3] = { (uint16_t) ((uint16_t) g->lm.lrbeta[pos] & 077777U), 0U, (uint16_t) ((uint16_t) g->lm.lralpha[pos] & 077777U) };
    agc_vec_t      y      = agc_vec_make(0, AGC_DP_HALF, 0); /* UNITY, half unit; (2^1) */

    agc_imu_trg_smnb(agc_imu_trig(), cdu, &y);
    g->lm.vbeamnb[1] = y;                               /* VYBEAMNB               */
    agc_vec_t x      = agc_vec_make(AGC_DP_HALF, 0, 0); /* UNITX, half unit; (2^1) */

    agc_imu_smnb(agc_imu_trig(), &x);
    g->lm.vbeamnb[2] = x; /* VXBEAMNB               */
    agc_interp_t ip;      /* interpreter state */

    agc_interp_init(&ip);
    g->lm.vbeamnb[0] = agc_vec_shl(&ip, agc_vec_cross(&ip, x, y), 1);       /* Z = X * Y */
    agc_vec_t h      = agc_vec_make(HBEAMANT[0], HBEAMANT[1], HBEAMANT[2]); /* HBEAMANT, half unit (antenna axes); (2^1) */

    agc_imu_smnb(agc_imu_trig(), &h);
    g->lm.hbeamnb = h;
}

/**********************************************************************
 * SETPOS1 (p.895-896): initialise the LR counters, VSELECT and STILBADs,
 * position-1 beam vectors.
 *
 * FUNCTION NAME: agc_lm_setpos1
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
void agc_lm_setpos1(void)
{
    agc_guid_lm_t *lm = &agc_guid_state()->lm; /* LM guidance erasables */

    lm->stilbadh = 2;
    lm->stilbadv = 2;
    lm->lrlctr   = 0;
    lm->lrmctr   = 0;
    lm->lrrctr   = 0;
    lm->lrsctr   = 0;
    lm->vselect  = 0;

    setpos(0);
}

/**********************************************************************
 * SETPOS2 (p.896): position-2 beam vectors.
 *
 * FUNCTION NAME: agc_lm_setpos2
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
static agc_status_t agc_lm_setpos2(void)
{
    if (!AGC_ASSERT(agc_guid_state()->lm.vselect <= 2))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    setpos(1);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * HIGATJOB (p.895): reposition the antenna to POS2 and sleep until done.
 * DEVIATION: first waits while another job sleeps in RADSTALL, as LRVJOB
 * does (HIGATASK runs inside the C SERVICER in no AGC time; at P64 entry
 * the HIGATJOB RADSTALL met LRHJOB's: BAILOUT 01210, the reposition was
 * lost and every later LR read alarmed 00511, NASSP PDI 2026-10-13).
 *
 * FUNCTION NAME: agc_job_higatjob
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_higatjob(agc_job_ctx_t *ctx)
{
    if (agc_erase_modecadr(AGC_IMU_DEV_RADAR) > 0)
    { /* LRHJOB still in RADSTALL: wait for it (DEVIATION at LRVJOB) */
        return (agc_svc_delayjob(LRV_WAIT_CS, AGC_JOB_HIGATJOB));
    }

    agc_status_t sw = agc_radar_lrpos2(); /* status of agc_radar_lrpos2 */

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING;                                            /* RADSTALL outcome */
    agc_status_t           st  = agc_imu_stall_dev(AGC_IMU_DEV_RADAR, AGC_JOB_HIGATJOB_DONE, &res); /* status of agc_imu_stall_dev */

    if (st != AGC_STATUS_OK || res == AGC_IMU_STALL_SLEEPING)
    {
        return (st);
    }

    return (agc_job_higatjob_done(ctx));
}

/**********************************************************************
 * POSGOOD (p.895): lower priority, POS2 beam vectors, LR reading enabled.
 * DEVIATION: PRIOCHNG's possible change-job is not taken here (the rest of
 * the job is short and runs at once).
 *
 * FUNCTION NAME: posgood
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t posgood(void)
{
    agc_exec_prio_change(AGC_PRIO23);
    agc_status_t st = agc_lm_setpos2(); /* status of agc_lm_setpos2 */

    agc_flag_clear(AGC_FLAG_NOLRREAD); /* ENABLE LR READING      */

    return (st);
}

/**********************************************************************
 * POSALARM (p.895): PRIOLARM 523, V05N09: terminate -> P00, proceed ->
 * check again (POSALARM_PRO), V32 -> end R12.
 *
 * FUNCTION NAME: posalarm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t posalarm(agc_job_ctx_t *ctx)
{
    agc_prioalarm(AGC_ALARM_00523);
    agc_dsky_cont_t cont = {AGC_JOB_GOTOPOOH, AGC_JOB_POSALARM_PRO, AGC_JOB_ENDOFJOB};

    return (agc_dsky_display(ctx, AGC_DSKY_PRIODSP, agc_dsky_vn(5, 9), &cont));
}

/**********************************************************************
 * HIGATJOB after RADSTALL (p.895).
 *
 * FUNCTION NAME: agc_job_higatjob_done
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_higatjob_done(agc_job_ctx_t *ctx)
{
    if (agc_imu_stall_outcome(AGC_IMU_DEV_RADAR) != AGC_IMU_STALL_GOOD)
    {
        return (posalarm(ctx)); /* BAD END: ALARM         */
    }

    return (posgood());
}

/**********************************************************************
 * POSALARM +3 (p.895): PROCEED -- in POS2 yet?
 *
 * FUNCTION NAME: agc_job_posalarm_pro
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_posalarm_pro(agc_job_ctx_t *ctx)
{
    if ((agc_chan_read(033) & (uint16_t) AGC_CH033_LRPOS2) == 0U)
    {
        return (posgood()); /* POS2 ACHIEVED          */
    }

    return (posalarm(ctx)); /* STILL DIDN'T MAKE IT   */
}
