/**********************************************************************
 * FILE NAME: needles.c
 *
 * PURPOSE:
 *   The FDAI attitude-error display of the LM DAP (DAPIDLER_PROGRAM.agc
 *   pp.1414-1420): ALTDSPLY, DSPLYTOT, NEEDLER.
 *
 * AGC SOURCE:
 *   Luminary099 DAPIDLER_PROGRAM.agc pp. 1414-1420
 *     (ALTDSPLY, DSPLYTOT, NEEDLER)
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   DAP block     lm_dap_t  I/O  per field      EBANK 6 DAP erasables (lm_dap_state())
 *   CDUX          uint16_t  I    2^0 rev        CDU X angle (agc_erase_cdux())
 *   CDUY          uint16_t  I    2^0 rev        CDU Y angle (agc_erase_cduy())
 *   CDUZ          uint16_t  I    2^0 rev        CDU Z angle (agc_erase_cduz())
 *   M11           agc_sp_t  I    2^0            gimbal matrix (agc_erase_m11())
 *   M21           agc_sp_t  I    2^0            gimbal matrix (agc_erase_m21())
 *   M22           agc_sp_t  I    2^0            gimbal matrix (agc_erase_m22())
 *   M31           agc_sp_t  I    2^0            gimbal matrix (agc_erase_m31())
 *   M32           agc_sp_t  I    2^0            gimbal matrix (agc_erase_m32())
 *   THETAD        uint16_t  I    2^0 rev        desired CDU angles (agc_erase_thetad())
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ------------------------------------
 *   agc_chan_and                WAND channel
 *   agc_chan_or                 WOR channel
 *   agc_chan_read               READ channel
 *   agc_flag_test               test a flag bit (flags.h)
 *   agc_imu_cducmd              CDUXCMD.. command counters (imu)
 *   agc_imu_hw_set_cducmd       CDU error-counter drive (imu)
 *   agc_state_vehicle           CSM or LM rope
 *   lm_dap_mphi                 A after MP (arith.c)
 *   lm_dap_msu                  MSU (arith.c)
 *   lm_dap_sat                  OVERSUB / LIMITS (arith.c)
 *   lm_dap_state                DAP erasables (state.c)
 *   lm_dap_wrap                 overflow-corrected store, TS/ADS/XCH (arith.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. AGC_STATUS_BAD_ARG when an internal assertion fails.
 *
 * NOTES:
 *   ALTDSPLY alternates every 100 ms between computing the errors (DAP
 *   following errors, or total errors when NEEDLFLG is set) and driving the
 *   needles through the IMU error counters (CDUXCMD..CDUZCMD) via NEEDLER.
 **********************************************************************/
#include "../internal.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/imu/imu.h"
#include "agc/lm/dap.h"

#define NDL_ONETENTH ((agc_sp_t) 03146) /* ONETENTH: 0.1 at 1             */
#define NDL_DACMAX   384                /* 384 bits = 42 3/16 deg (DAC)   */
#define NDL_IMUERR   000040u            /* CHAN12 bit 6: IMU error cntr   */
#define NDL_DACBITS  070000u            /* 13,14,15: CHAN14 DAC activity  */
#define NDL_PASSBITS 6u                 /* SIX: RCSFLAGS bits 2,3         */

/**********************************************************************
 * ALTDSPLY (pp.1414-1415): invert DSPLYALT; display (NEEDLER) or compute
 * the errors for the next pass.
 *
 * FUNCTION NAME: lm_dap_altdsply
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_altdsply(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->rcsflags = (uint16_t) (d->rcsflags ^ LM_RCS_DSPLYALT);

    if ((d->rcsflags & LM_RCS_DSPLYALT) != 0u)
    {
        return (lm_dap_needler());
    }

    if (agc_flag_test(AGC_FLAG_NEEDLFLG))
    { /* total attitude error   */
        return (lm_dap_dsplytot());
    }

    d->ak[1] = (agc_sp_t) -d->qerror; /* error complements      */
    d->ak[2] = (agc_sp_t) -d->rerror;
    d->ak[0] = (agc_sp_t) -d->perror;

    if (!AGC_ASSERT(d->ak[0] >= LM_NEGMAX && d->ak[0] <= LM_POSMAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(d->ak[1] >= LM_NEGMAX && d->ak[2] >= LM_NEGMAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * DSPLYTOT (pp.1415-1416): desired minus present gimbal angles resolved
 * into pilot axes (M13 = 1), each limited by OVERSUB2. At pi rad.
 *
 * FUNCTION NAME: lm_dap_dsplytot
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_dsplytot(void)
{
    lm_dap_t *d    = lm_dap_state(); /* DAP erasables */
    agc_sp_t  ydif = lm_dap_msu(agc_erase_thetad(1), agc_erase_cduy()); /* CTHETA-CDUY */
    agc_sp_t  zdif = lm_dap_msu(agc_erase_thetad(2), agc_erase_cduz()); /* CPSI-CDUZ   */
    agc_sp_t  xdif = lm_dap_msu(agc_erase_thetad(0), agc_erase_cdux()); /* CPHI-CDUX   */

    d->ak[1] = lm_dap_sat((int32_t) lm_dap_mphi(ydif, agc_erase_m21()) + (int32_t) lm_dap_mphi(zdif, agc_erase_m22()));
    d->ak[2] = lm_dap_sat((int32_t) lm_dap_mphi(ydif, agc_erase_m31()) + (int32_t) lm_dap_mphi(zdif, agc_erase_m32()));
    d->ak[0] = lm_dap_sat((int32_t) lm_dap_mphi(ydif, agc_erase_m11()) + (int32_t) xdif);

    if (!AGC_ASSERT(d->ak[0] >= LM_NEGMAX && d->ak[0] <= LM_POSMAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(d->ak[1] >= LM_NEGMAX && d->ak[2] >= LM_NEGMAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * NEEDLE11 (p.1418): first pass -- zero inputs, display registers and
 * the error counters; next pass enables the counters.
 *
 * FUNCTION NAME: needles_first_pass
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
static agc_status_t needles_first_pass(lm_dap_t *d)
{
    agc_status_t st = AGC_STATUS_OK;

    for (int i = 0; i < 3; i++)
    {
        d->ak[i]       = 0;
        d->edrive[i]   = 0;
        agc_status_t s = agc_imu_hw_set_cducmd(i, 0); /* CS ZERO: -0       */

        st = (s != AGC_STATUS_OK) ? s : st;
    }

    d->rcsflags = (uint16_t) ((d->rcsflags & ~NDL_PASSBITS) | LM_RCS_NEEDLE2);

    if (!AGC_ASSERT((d->rcsflags & LM_RCS_NEEDLE3) == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * NEEDLES / DACLOOP (p.1419): meter the change of each display into the
 * IMU error counter, limited to +-384 bits.
 *
 * FUNCTION NAME: needles
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
static agc_status_t needles(lm_dap_t *d)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(d != 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = AGC_STATUS_OK;

    for (int i = 2; i >= 0; i--)
    { /* DINDX = 2, 1, 0       */
        int32_t l = lm_dap_mphi((agc_sp_t) -NDL_ONETENTH, d->ak[i]); /* at 1800 deg */

        if (l > NDL_DACMAX)
        {
            l = NDL_DACMAX; /* overflow: DACLIMIT +1 */
        }
        else if (l < -NDL_DACMAX)
        {
            l = -NDL_DACMAX;
        }

        int32_t      cmd = (int32_t) agc_imu_cducmd(i) + l - (int32_t) d->edrive[i];
        agc_status_t s   = agc_imu_hw_set_cducmd(i, lm_dap_wrap(cmd)); /* ADS CDUXCMD */

        st           = (s != AGC_STATUS_OK) ? s : st;
        d->edrive[i] = (agc_sp_t) l;
    }

    agc_chan_or(014, NDL_DACBITS); /* DAC activity bits */

    if (!AGC_ASSERT(d->edrive[0] >= -NDL_DACMAX && d->edrive[0] <= NDL_DACMAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
} /* needles */

/**********************************************************************
 * NEEDLER (pp.1418-1419): relay sequencing, then the DAC loop.
 *
 * FUNCTION NAME: lm_dap_needler
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_needler(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    lm_dap_t *d    = lm_dap_state(); /* DAP erasables */
    uint16_t  pass = (uint16_t) (d->rcsflags & NDL_PASSBITS);

    if (pass != 0u)
    {
        if ((pass & LM_RCS_NEEDLE3) != 0u)
        { /* first pass         */
            agc_chan_and(012, (uint16_t) ~NDL_IMUERR & 077777u);

            return (needles_first_pass(d));
        }

        agc_chan_or(012, NDL_IMUERR); /* NEEDLER2: enable */
        d->rcsflags = (uint16_t) (d->rcsflags & ~NDL_PASSBITS);

        return (AGC_STATUS_OK);
    }

    if ((agc_chan_read(012) & NDL_IMUERR) == 0u)
    { /* NEEDLES3           */
        d->rcsflags = (uint16_t) (d->rcsflags | LM_RCS_NEEDLE3); /* re-init  */

        return (AGC_STATUS_OK);
    }

    return (needles(d));
}
