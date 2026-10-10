/*
 * needles.c -- FDAI attitude error display subroutine.
 * RCS-CSM_DIGITAL_AUTOPILOT.agc (Comanche055 pp. 1010-1013): NEEDLER,
 * NEEDLER1, NEEDLE11, NEEDLER2, NEEDLES3, NEEDLES, DACLOOP.
 * Program description: D. Keene 5/24/67.
 *
 * "This subroutine is used to display attitude errors on the FDAI via the
 * digital to analog converters (DACs) in the CDUs. Care is taken to meter
 * out the appropriate number of pulses to the IMU error counters and
 * prevent overflow, to control the relay sequencing, and to avoid
 * interference with the coarse align loop which also uses the DACs."
 * Full scale deflection = 16 7/8 degrees of attitude error = 384 bits in
 * the IMU error counter. At least 3 passes, 50 ms apart, are needed before
 * anything is displayed. Sign convention: AK = THETAC - THETA.
 * Users set RCSFLAGS BIT3 in their initialisation, then store AK, AK1,
 * AK2 (scaled 180 deg) and call NEEDLER under INHINT.
 */
#include "../internal.h"
#include "agc/imu/imu.h"

#define FLAGS_23 ((uint16_t) 000006) /* SIX: RCSFLAGS bits 3, 2              */
#define DACLIMIT ((agc_sp_t) 384)    /* DACLIMIT +-1: DEC 384 (00600)        */
#define MQUARTER ((agc_sp_t) -8192)  /* CS QUARTER                           */
#define DAC_BITS ((uint16_t) 070000) /* 13,14,15: DAC activity, channel 14   */

static agc_status_t needle11(void);
static agc_status_t needles3(void);
static agc_status_t needles(void);

/* NEEDLER (p. 1011).                                                     */
agc_status_t csm_dap_needler(void)
{
    agc_csm_dapcom_t *c    = agc_csm_dap_com();
    uint16_t          ch12 = agc_chan_read(AGC_CHAN_CHAN12);
    if ((ch12 & (uint16_t) AGC_CH012_COARSE) != 0u)
    {
        /* in coarse align do not use the IMU error counters             */
        c->rcsflags = (uint16_t) (c->rcsflags | AGC_CSM_RCSF_NEEDLE1);
        return AGC_STATUS_OK;
    }
    uint16_t f = (uint16_t) (c->rcsflags & FLAGS_23); /* NEEDLER1    */
    if (f == 0u)
    {
        return needles3();
    }
    if ((f & AGC_CSM_RCSF_NEEDLE1) != 0u)
    {
        /* first pass: disable IMU error counter to zero the DACs; must
         * wait at least 60 ms before enabling counters.                  */
        agc_chan_and(
            AGC_CHAN_CHAN12,
            (uint16_t) (077777u & ~(unsigned) AGC_CH012_ENBIMUERR));
        return needle11();
    }
    agc_chan_or(AGC_CHAN_CHAN12, (uint16_t) AGC_CH012_ENBIMUERR);
    c->rcsflags =
        (uint16_t) (c->rcsflags & ~(unsigned) FLAGS_23); /* NEEDLER2 */
    if (!AGC_ASSERT((c->rcsflags & FLAGS_23) == 0u))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK; /* wait at least 4 ms for relay closure               */
}

/* NEEDLE11: zero the inputs, the display registers and the out counters
 * on the first pass; reset RCSFLAGS for pass 2.                          */
static agc_status_t needle11(void)
{
    agc_csm_dapcom_t *c  = agc_csm_dap_com();
    agc_status_t      st = AGC_STATUS_OK;
    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        c->ak[i]       = 0; /* CS ZERO: -0            */
        c->edrive[i]   = 0;
        agc_status_t s = agc_imu_hw_set_cducmd(i, 0); /* CDUXCMD..CDUZCMD */
        st             = (st == AGC_STATUS_OK) ? s : st;
    }
    c->rcsflags = (uint16_t) ((c->rcsflags & ~(unsigned) FLAGS_23) |
                              AGC_CSM_RCSF_NEEDLE2);
    if (!AGC_ASSERT((c->rcsflags & AGC_CSM_RCSF_NEEDLE2) != 0u))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st; /* end pass 1             */
}

/* NEEDLES3: if the IMU error counter is not enabled, recycle (NEEDLER +5). */
static agc_status_t needles3(void)
{
    agc_csm_dapcom_t *c    = agc_csm_dap_com();
    uint16_t          ch12 = agc_chan_read(AGC_CHAN_CHAN12);
    if (!AGC_ASSERT(ch12 <= CSM_DAP_NEG0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if ((ch12 & (uint16_t) AGC_CH012_ENBIMUERR) == 0u)
    {
        c->rcsflags = (uint16_t) (c->rcsflags | AGC_CSM_RCSF_NEEDLE1);
        return AGC_STATUS_OK;
    }
    return needles();
}

/* NEEDLES, DACLOOP (p. 1012): EDRIVE = -AK/4 limited to +-384 pulses;
 * the change is added to the CDU error counter; DAC activity bits set.  */
static agc_status_t needles(void)
{
    agc_csm_dapcom_t *c  = agc_csm_dap_com();
    agc_status_t      st = AGC_STATUS_OK;
    for (int i = AGC_CSM_AXES - 1; i >= 0; i--)
    {
        agc_sp_t l = csm_dap_mp_hi(MQUARTER, c->ak[i]); /* TS L        */
        if (l > DACLIMIT)
        { /* overflow of +-16000 + L: limit    */
            l = DACLIMIT;
        }
        else if (l < -DACLIMIT)
        {
            l = (agc_sp_t) -DACLIMIT;
        }
        else
        {
            /* within the DAC range */
        }
        agc_sp_t inc   = (agc_sp_t) (l - c->edrive[i]); /* CS EDRIVEX; AD L */
        agc_sp_t cmd   = csm_dap_ads(agc_imu_cducmd(i), inc); /* ADS CDUXCMD */
        agc_status_t s = agc_imu_hw_set_cducmd(i, cmd);
        st             = (st == AGC_STATUS_OK) ? s : st;
        c->edrive[i]   = l; /* LXCH EDRIVEX */
    }
    agc_chan_or(AGC_CHAN_CHAN14, DAC_BITS);
    agc_status_t s2 = AGC_STATUS_OK;
    if (!AGC_ASSERT(c->edrive[0] <= DACLIMIT))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return (st == AGC_STATUS_OK) ? s2 : st;
}

/* NEEDLER for other lanes (entry DAP, R60): AK..AK2 stored first; init
 * requests the turn-on sequence (RCSFLAGS BIT3) as the user's
 * initialisation section does. Called like "INHINT; TC IBNKCALL; CADR
 * NEEDLER; RELINT".                                                      */
agc_status_t agc_dap_needler(agc_sp_t ak, agc_sp_t ak1, agc_sp_t ak2, bool init)
{
    agc_csm_dapcom_t *c = agc_csm_dap_com();
    if (!AGC_ASSERT((ak >= AGC_SP_MIN) && (ak1 >= AGC_SP_MIN) && (ak2 >= AGC_SP_MIN)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (init)
    {
        c->rcsflags = (uint16_t) (c->rcsflags | AGC_CSM_RCSF_NEEDLE1);
    }
    c->ak[0] = ak;
    c->ak[1] = ak1;
    c->ak[2] = ak2;
    agc_inhint();
    agc_status_t st = csm_dap_needler();
    agc_relint();
    return st;
}
