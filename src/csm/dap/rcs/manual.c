/**********************************************************************
 * FILE NAME: manual.c
 *
 * PURPOSE:
 *   Phase 2 of the RCS-CSM autopilot: manual rotation commands from the
 *   rotational hand controller (RHC). A change of channel 31 bits 1-6
 *   (RHC rotation, 0 = commanded) sets the forced firing bits (RCSFLAGS
 *   bits 10-5) and the rate damping flags (bits 13-11). With the RHC out
 *   of detent the DAP flies the commanded rate (RATEINDX: 0.05, 0.2, 0.5
 *   or 2.0 deg/s) and accumulates the attitude error MERROR = MERROR +
 *   measured change - commanded change (the measured change is added in
 *   the rate filter). Labels: MANUAL ROTATION COMMANDS, NOCHANGE,
 *   SETWBODY, ZEROER, NORATE, MERUPDAT, SPNDXCHK, FREEFUNC, STICKCHK;
 *   tables MANTABLE, FREETAU.
 *
 * AGC SOURCE:
 *   Comanche055 RCS-CSM_DIGITAL_AUTOPILOT.agc pp. 1016-1021
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type            I/O  Scale        Description
 *   ------------  --------------  ---  -----------  ------------------
 *   CH31TEMP      uint16_t        I/O  bits         last channel 31
 *   RCSFLAGS      uint16_t        I/O  bits         forced, damping bits
 *   HOLDFLAG      agc_sp_t        O    -            + while commanded
 *   RMANNDX..     agc_sp_t        I/O  index        STICKCHK results
 *   RATEINDX      agc_sp_t        I    index        0, 2, 4, 6
 *   WBODY..       agc_dp_t        O    450 deg/s    commanded rate
 *   MERRORX..     agc_dp_words_t  I/O  180 deg      manual att. error
 *   ERRORX..      agc_sp_t        O    180 deg      upper word of MERROR
 *   TAU..TAU2     agc_sp_t        O    1/1600 s     free-mode commands
 *   ATTKALMN      agc_sp_t        O    index        -1 at high rate
 *   STIKFLAG      flag            O    -            astronaut control
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   csm_rcs_ahfnorot      AHFNOROT (attitude.c)
 *   csm_rcs_jets          JETS (attitude.c)
 *   csm_rcs_t6prog_manual T6PROGM (attitude.c)
 *   csm_rcs_reinit        REINIT (t5rupt.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none in the AGC. AGC_STATUS_BAD_ARG for command bits outside MANROT
 *   or a MANTABLE index outside the table.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs in T5PHASE2 after the forced-firing bits have been cleared.
 *   After the RHC returns to detent the program branches to AHFNOROT and
 *   avoids STICKCHK, so the MANNDX keep their last out-of-detent values.
 **********************************************************************/
#include "../internal.h"

#define MANROT     ((uint16_t) 000077) /* MANROT OCT 77                    */
#define FORCEDBITS ((uint16_t) 001760) /* OCT01760 forced firing mask      */
#define DAMPBITS   ((uint16_t) 016000) /* OCT16000 rate damping mask       */
#define BIT9       ((agc_sp_t) 0400)   /* MP BIT9: MANTABLE x 2^-6 = WBODY */
#define BIT7       ((agc_sp_t) 0100)   /* MP BIT7: x 2^-8 = change in .1 s */
#define MANTAB_N   9                   /* MANTABLE words, with =+14MS      */

/* MANTABLE (p. 1019), indexed by MANNDX + RATEINDX - 1, rate scaled
 * (450 deg/s) x 2^6. The ninth word is the constant that follows in
 * fixed memory (=+14MS, DEC 23), reached when STICKCHK yields index 3
 * with RATEINDX = 6.                                                     */
static const agc_sp_t mantable[MANTAB_N] = {
    117,   /* 00165  .0071111  +0.05 deg/s */
    -117,  /* 77612 -.0071111  -0.05 deg/s */
    466,   /* 00722  .028444   +0.2  deg/s */
    -466,  /* 77055 -.028444   -0.2  deg/s */
    1165,  /* 02215  .071111   +0.5  deg/s */
    -1165, /* 75562 -.071111   -0.5  deg/s */
    4660,  /* 11064  .284444   +2.0  deg/s */
    -4660, /* 66713 -.284444   -2.0  deg/s */
    23     /* 00027 =+14MS (following word)  */
};

/* FREETAU (p. 1020): free-mode acceleration commands, 1/1600 s (DEC 480
 * is 0.30 s; the AGC comment reads 0.10).                                */
static const agc_sp_t freetau[4] = {
    0,    /* 0 sec        */
    480,  /* +0.30 sec    */
    -480, /* -0.30 sec    */
    0     /* (+3) 0 sec   */
};

/**********************************************************************
 * MANUAL ROTATION COMMANDS .. NOCHANGE (RCS-CSM_DIGITAL_AUTOPILOT.agc
 * pp. 1016-1018) -- on a change of the RHC bits (CH31TEMP XOR channel
 * 31) save channel 31, set the forced firing bits of the changed axes
 * (bits 6-1 into bits 10-5: 10,9 roll, 8,7 yaw, 6,5 pitch) and the rate
 * damping flags. With no rotation commanded go to AHFNOROT; otherwise
 * HOLDFLAG +, STICKCHK, STIKFLAG, then free mode (FREEFUNC), filter
 * not initialised (REINIT), or the commanded rates (SETWBODY), with
 * ATTKALMN -1 at RATEINDX 6.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the branch taken
 **********************************************************************/
agc_status_t csm_rcs_manual(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs(); /* RCS DAP erasables       */
    agc_csm_dapcom_t *c = agc_csm_dap_com(); /* RCSFLAGS                */

    c->rcsflags = (uint16_t) (c->rcsflags & ~(unsigned int) FORCEDBITS); /* CS OCT01760; MASK */
    uint16_t ch31 = agc_chan_read(AGC_CHAN_CHAN31);                        /* READ CHAN31; TS L */
    uint16_t chg = (uint16_t) ((r->ch31temp ^ ch31) & MANROT);            /* RXOR LCHAN; MASK  */

    if (chg != 0u)
    {
        r->ch31temp = ch31; /* LXCH A; TS CH31TEMP                      */

        /* CA L; MP BIT5; CA L; ADS RCSFLAGS: bits 6-1 into bits 10-5     */
        c->rcsflags = (uint16_t) (c->rcsflags + (unsigned int) (chg << 4));
        c->rcsflags = (uint16_t) (c->rcsflags | DAMPBITS); /* CS; MASK OCT16000; ADS */
    }

    uint16_t cmd = (uint16_t) (~(unsigned int) r->ch31temp & MANROT); /* NOCHANGE: CS CH31TEMP; MASK */

    if (cmd == 0u)
    {
        return (csm_rcs_ahfnorot()); /* BZMF AHFNOROT: no manual commands */
    }

    r->holdflag = (agc_sp_t) cmd;            /* TS HOLDFLAG: +          */
    agc_status_t st = agc_csm_dap_stickchk(cmd); /* TC STICKCHK             */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_set(AGC_FLAG_STIKFLAG); /* inform P20: astronaut control   */

    if ((agc_chan_read(AGC_CHAN_CHAN31) & (uint16_t) AGC_CH031_FREE) == 0u)
    {
        /* CAF BIT14; RAND CHAN31; BZMF        */
        /**********************************************************************
         * FREEFUNC (RCS-CSM_DIGITAL_AUTOPILOT.agc pp. 1019-1020) --
         * acceleration commands in free mode: TAU = FREETAU by the STICKCHK
         * indices; T6PROGM.
         *
         * ARGUMENTS: none
         *
         * RETURNS: agc_status_t -- the T6PROGM status
         **********************************************************************/
        agc_csm_rcs_t *r_freefunc = agc_csm_dap_rcs(); /* MANNDX, TAU                */

        for (int i = 0; i < AGC_CSM_AXES; i++)
        {
            r_freefunc->tau[i] = freetau[r_freefunc->manndx[i] & 3]; /* INDEX RMANNDX; CA FREETAU; TS TAU */
        }

        return (csm_rcs_t6prog_manual());
    }

    if (csm_rcs_rate_init_pending())
    {
        return (csm_rcs_reinit()); /* ..... TILT, recycle the filter    */
    }

    if (r->ratindx - 5 > 0)
    {
        r->attkalmn = -1; /* CS FIVE; AD RATEINDX: high-rate maneuver    */
    }

    /**********************************************************************
     * SETWBODY, ZEROER, NORATE, MERUPDAT, SPNDXCHK
     * (RCS-CSM_DIGITAL_AUTOPILOT.agc pp. 1018-1019) -- auto-hold manual
     * rotation, axes Z, Y, X: WBODY = MANTABLE x 2^-6 (0 with no rotation);
     * once rate damping is completed MERRORX -= MANTABLE x 2^-8 (the
     * commanded change in .1 s), else MERRORX = 0 for a commanded axis or
     * while damping; ERRORX = upper word of MERRORX; then JETS.
     *
     * ARGUMENTS: none
     *
     * RETURNS: agc_status_t -- the JETS status; AGC_STATUS_BAD_ARG for a
     *          MANTABLE index outside the table
     **********************************************************************/
    agc_csm_rcs_t *r_setwbody = agc_csm_dap_rcs();                       /* RCS DAP erasables      */
    const agc_csm_dapcom_t *c_setwbody = agc_csm_dap_com();                       /* RCSFLAGS               */
    bool damped = (c_setwbody->rcsflags & DAMPBITS) == 0u; /* rate damping completed */

    for (int i_setwbody = AGC_CSM_AXES - 1; i_setwbody >= 0; i_setwbody--)
    {
        int m = (int) r_setwbody->manndx[i_setwbody]; /* 0 none, 1 +, 2 - rotation, 3 both */

        if (m == 0)
        {
            r_setwbody->wbody[i_setwbody] = 0; /* NORATE: ZL; DXCH WBODY                  */
        }
        else
        {
            int q = m + (int) r_setwbody->ratindx - 1; /* INDEX Q; MANTABLE -1  */

            if (!AGC_ASSERT((q >= 0) && (q < MANTAB_N)))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            r_setwbody->wbody[i_setwbody] = agc_sp_mul(mantable[q], BIT9); /* MP BIT9; DXCH WBODY, scale 450 deg/s */

            if (damped)
            { /* MERUPDAT: CS MANTABLE -1; MP BIT7; DAS MERRORX */
                csm_dap_words_das(&r_setwbody->merror[i_setwbody], csm_dap_words_mp((agc_sp_t) -mantable[q], BIT7));
            }
        }

        if (!damped)
        {
            r_setwbody->merror[i_setwbody] = (agc_dp_words_t){0, 0}; /* ZEROER: CA ZERO; ZL; DXCH */
        }

        r_setwbody->error[i_setwbody] = r_setwbody->merror[i_setwbody].hi; /* SPNDXCHK: high order word of MERRORX */
    }

    return (csm_rcs_jets());
} /* csm_rcs_manual */

/**********************************************************************
 * STICKCHK (RCS-CSM_DIGITAL_AUTOPILOT.agc p. 1021; also SATSTICK in
 * guid-entry) -- RMANNDX,
 * PMANNDX, YMANNDX = 0, 1, 2 (or 3) for no, +, or - rotation (both)
 * from the six command bits (1,2 pitch, 3,4 yaw, 5,6 roll): MASK THREE,
 * MP QUARTER (shift right 2), MASK THREE, MP QUARTER.
 *
 * ARGUMENTS:
 *   bits  uint16_t  I  RHC command bits 1-6, active 1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG for bits
 *          outside MANROT
 **********************************************************************/
agc_status_t agc_csm_dap_stickchk(uint16_t bits)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs(); /* MANNDX                     */

    if (!AGC_ASSERT(bits <= MANROT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    r->manndx[1] = (agc_sp_t) (bits & 3u);   /* TS T5TEMP; MASK THREE; TS PMANNDX */
    unsigned int t = (unsigned int) bits >> 2; /* CA T5TEMP; MP QUARTER          */

    r->manndx[2] = (agc_sp_t) (t & 3u); /* MASK THREE; TS YMANNDX            */
    r->manndx[0] = (agc_sp_t) (t >> 2); /* MP QUARTER; TS RMANNDX            */

    return (AGC_STATUS_OK);
}
