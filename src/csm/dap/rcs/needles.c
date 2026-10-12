/**********************************************************************
 * FILE NAME: needles.c
 *
 * PURPOSE:
 *   The FDAI attitude error display subroutine (D. Keene 5/24/67): "used
 *   to display attitude errors on the FDAI via the digital to analog
 *   converters (DACs) in the CDUs. Care is taken to meter out the
 *   appropriate number of pulses to the IMU error counters and prevent
 *   overflow, to control the relay sequencing, and to avoid interference
 *   with the coarse align loop which also uses the DACs." Labels:
 *   NEEDLER, NEEDLER1, NEEDLE11, NEEDLER2, NEEDLES3, NEEDLES, DACLOOP.
 *
 * AGC SOURCE:
 *   Comanche055 RCS-CSM_DIGITAL_AUTOPILOT.agc pp. 1010-1013
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   AK..AK2       agc_sp_t  I/O  180 deg        attitude errors
 *   EDRIVEX..Z    agc_sp_t  I/O  pulses         DAC contents
 *   CDUXCMD..Z    agc_sp_t  I/O  pulses         IMU error counters
 *   RCSFLAGS      uint16_t  I/O  bits           bits 3, 2: pass
 *   CHAN12        uint16_t  I/O  bits           4 coarse align (read),
 *                                               6 error counter enable
 *   CHAN14        uint16_t  O    bits           13-15 DAC activity
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_imu_cducmd,       CDUXCMD..CDUZCMD (imu)
 *   agc_imu_hw_set_cducmd
 *   csm_dap_mp_hi, _ads   MP, ADS (mysubs.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none in the AGC. The status of the CDUXCMD write is passed on.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Called under INHINT. Users set RCSFLAGS BIT3 in their
 *   initialisation, then store AK, AK1, AK2 (scaled 180 deg) and call
 *   NEEDLER; at least 3 passes, 50 ms apart, are needed before anything
 *   is displayed. Sign convention: AK = THETAC - THETA.
 *
 * NOTES:
 *   Full scale deflection = 16 7/8 degrees of attitude error = 384 bits
 *   in the IMU error counter: EDRIVE = -AK/4 (CS QUARTER; MP AK) limited
 *   to +-384. The imu lane's agc_imu_hw_set_cducmd refuses counts beyond
 *   +-192 (the coarse-align limit), which a full needle swing exceeds:
 *   request in docs/modules/review-dap-csm.md.
 **********************************************************************/
#include "../internal.h"

#include "agc/imu/imu.h"

#define FLAGS_23 ((uint16_t) 000006) /* SIX: RCSFLAGS bits 3, 2              */
#define DACLIMIT ((agc_sp_t) 384)    /* DACLIMIT +-1: DEC 384 (00600) pulses */
#define MQUARTER ((agc_sp_t) -4096)  /* CS QUARTER (BIT13 = 1/4), scale 1    */
#define DAC_BITS ((uint16_t) 070000) /* 13,14,15: DAC activity, channel 14   */

/**********************************************************************
 * NEEDLER, NEEDLER1, NEEDLER2 (RCS-CSM_DIGITAL_AUTOPILOT.agc p. 1011)
 * -- in coarse align request the initialisation pass and do nothing;
 * otherwise by RCSFLAGS bits 3, 2: first pass (NEEDLE11), second pass
 * (enable the IMU error counters) or a display pass (NEEDLES3).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK or the CDUXCMD write status
 **********************************************************************/
agc_status_t csm_dap_needler(void)
{
    agc_csm_dapcom_t *c    = agc_csm_dap_com();              /* RCSFLAGS          */
    uint16_t          ch12 = agc_chan_read(AGC_CHAN_CHAN12); /* CAF BIT4; RAND    */

    if ((ch12 & (uint16_t) AGC_CH012_COARSE) != 0u)
    {
        /* in coarse align do not use the IMU error counters: set BIT3   */
        c->rcsflags = (uint16_t) (c->rcsflags | AGC_CSM_RCSF_NEEDLE1);

        return (AGC_STATUS_OK);
    }

    uint16_t f = (uint16_t) (c->rcsflags & FLAGS_23); /* NEEDLER1: MASK SIX */

    if (f == 0u)
    {
        /**********************************************************************
         * NEEDLES3 (RCS-CSM_DIGITAL_AUTOPILOT.agc p. 1012) -- if the IMU error
         * counter is not enabled, recycle the needles (NEEDLER +5: set BIT3);
         * otherwise NEEDLES.
         *
         * ARGUMENTS: none
         *
         * RETURNS: agc_status_t -- AGC_STATUS_OK or the NEEDLES status
         **********************************************************************/
        agc_csm_dapcom_t *c_needles3    = agc_csm_dap_com();              /* RCSFLAGS          */
        uint16_t          ch12_needles3 = agc_chan_read(AGC_CHAN_CHAN12); /* CAF BIT6; RAND    */

        if ((ch12_needles3 & (uint16_t) AGC_CH012_ENBIMUERR) == 0u)
        {
            c_needles3->rcsflags = (uint16_t) (c_needles3->rcsflags | AGC_CSM_RCSF_NEEDLE1);

            return (AGC_STATUS_OK);
        }

        /**********************************************************************
         * NEEDLES, DACLOOP (RCS-CSM_DIGITAL_AUTOPILOT.agc p. 1012) -- for Z, Y,
         * X: L = -AK/4; DACLIMIT (16000) +- L overflows when |L| >= 384, and the
         * output is then limited to +-384; the change over EDRIVE is added to
         * CDUXCMD and L becomes the new EDRIVE. Then the DAC activity bits.
         *
         * ARGUMENTS: none
         *
         * RETURNS: agc_status_t -- the first CDUXCMD write failure, else OK
         **********************************************************************/
        agc_csm_dapcom_t *c_needles  = agc_csm_dap_com(); /* AK, EDRIVE             */
        agc_status_t      st_needles = AGC_STATUS_OK;     /* first write failure    */

        for (int i_needles = AGC_CSM_AXES - 1; i_needles >= 0; i_needles--)
        {
            agc_sp_t l = csm_dap_mp_hi(MQUARTER, c_needles->ak[i_needles]); /* CS QUARTER; MP AK; TS L, unscaled pulses */

            if (l > DACLIMIT)
            { /* CA DACLIMIT; AD L; TS T5TEMP overflows: limit */
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

            agc_sp_t     inc       = (agc_sp_t) (l - c_needles->edrive[i_needles]); /* CS EDRIVEX; AD L, unscaled pulses */
            agc_sp_t     cmd       = csm_dap_ads(agc_imu_cducmd(i_needles), inc);   /* ADS CDUXCMD, unscaled pulses     */
            agc_status_t s_needles = agc_imu_hw_set_cducmd(i_needles, cmd);         /* counter write status */

            st_needles                   = (st_needles == AGC_STATUS_OK) ? s_needles : st_needles; /* ?: keep the first failure */
            c_needles->edrive[i_needles] = l;                                                      /* LXCH EDRIVEX             */
        }

        agc_chan_or(AGC_CHAN_CHAN14, DAC_BITS); /* CAF 13,14,15; WOR CHAN14   */

        return (st_needles);
    }

    if ((f & AGC_CSM_RCSF_NEEDLE1) != 0u)
    {
        /* first pass: disable the IMU error counter to zero the DACs;
         * wait at least 60 ms before enabling the counters               */
        agc_chan_and(AGC_CHAN_CHAN12, (uint16_t) (077777u & ~(unsigned int) AGC_CH012_ENBIMUERR));

        /**********************************************************************
         * NEEDLE11 (RCS-CSM_DIGITAL_AUTOPILOT.agc p. 1011) -- first pass: zero
         * the inputs, the display registers and the out counters; reset RCSFLAGS
         * for pass 2 (bit 2).
         *
         * ARGUMENTS: none
         *
         * RETURNS: agc_status_t -- the first CDUXCMD write failure, else OK
         **********************************************************************/
        agc_csm_dapcom_t *c_needle11 = agc_csm_dap_com(); /* AK, EDRIVE, RCSFLAGS   */
        agc_status_t      st         = AGC_STATUS_OK;     /* first write failure    */

        for (int i = 0; i < AGC_CSM_AXES; i++)
        {
            c_needle11->ak[i]     = 0;                           /* CS ZERO: -0            */
            c_needle11->edrive[i] = 0;                           /* TS EDRIVEX             */
            agc_status_t s        = agc_imu_hw_set_cducmd(i, 0); /* TS CDUXCMD..CDUZCMD    */

            st = (st == AGC_STATUS_OK) ? s : st; /* ?: keep the first failure */
        }

        /* CS SIX; MASK RCSFLAGS; AD BIT2; TS RCSFLAGS                         */
        c_needle11->rcsflags = (uint16_t) ((c_needle11->rcsflags & ~(unsigned int) FLAGS_23) | AGC_CSM_RCSF_NEEDLE2);

        return (st); /* end pass 1                                          */
    }

    /* NEEDLER2: enable the IMU error counters; wait at least 4 ms for
     * relay closure                                                      */
    agc_chan_or(AGC_CHAN_CHAN12, (uint16_t) AGC_CH012_ENBIMUERR);
    c->rcsflags = (uint16_t) (c->rcsflags & ~(unsigned int) FLAGS_23);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * NEEDLER for other lanes (entry DAP, R60) (RCS-CSM_DIGITAL_AUTOPILOT.agc
 * p. 1010, calling sequence) -- AK..AK2 stored first; init requests the
 * turn-on sequence (RCSFLAGS BIT3) as the user's initialisation section
 * does; "INHINT; TC IBNKCALL; CADR NEEDLER; RELINT".
 *
 * ARGUMENTS:
 *   ak    agc_sp_t  I  AK, scale 180 deg
 *   ak1   agc_sp_t  I  AK1, scale 180 deg
 *   ak2   agc_sp_t  I  AK2, scale 180 deg
 *   init  bool      I  set RCSFLAGS BIT3 first
 *
 * RETURNS: agc_status_t -- the NEEDLER status; AGC_STATUS_BAD_ARG for an
 *          error that is not a word value
 **********************************************************************/
agc_status_t agc_dap_needler(agc_sp_t ak, agc_sp_t ak1, agc_sp_t ak2, bool init)
{
    agc_csm_dapcom_t *c = agc_csm_dap_com(); /* AK, RCSFLAGS            */

    if (init)
    {
        c->rcsflags = (uint16_t) (c->rcsflags | AGC_CSM_RCSF_NEEDLE1);
    }

    c->ak[0]        = ak;
    c->ak[1]        = ak1;
    c->ak[2]        = ak2;
    agc_inhint();
    agc_status_t st = csm_dap_needler(); /* TC IBNKCALL; CADR NEEDLER   */

    agc_relint();

    return (st);
}
