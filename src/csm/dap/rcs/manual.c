/*
 * manual.c -- phase 2 of the RCS-CSM autopilot: manual rotation
 * commands from the rotational hand controller (RHC).
 * RCS-CSM_DIGITAL_AUTOPILOT.agc (Comanche055 pp. 1017-1021): MANUAL
 * ROTATION COMMANDS, NOCHANGE, SETWBODY, ZEROER, NORATE, MERUPDAT,
 * SPNDXCHK, FREEFUNC, STICKCHK, and the tables MANTABLE and FREETAU.
 *
 * Channel 31 bits 1-6 (RHC rotation, 0 = commanded) change -> forced
 * firing bits (RCSFLAGS bits 10-5) and rate damping flags (bits 13-11).
 * With the RHC out of detent the DAP flies the commanded rate (RATEINDX:
 * 0.05, 0.2, 0.5 or 2.0 deg/s) and accumulates the attitude error MERROR
 * = MERROR + measured change - commanded change (the measured change is
 * added in the rate filter).
 */
#include "../internal.h"

#define MANROT     ((uint16_t) 000077) /* MANROT OCT 77                    */
#define FORCEDBITS ((uint16_t) 001760) /* OCT01760 forced firing mask      */
#define DAMPBITS   ((uint16_t) 016000) /* OCT16000 rate damping mask       */
#define BIT9       ((agc_sp_t) 0400)   /* MP BIT9: MANTABLE x 2^-6 = WBODY */
#define BIT7       ((agc_sp_t) 0100)   /* MP BIT7: x 2^-8 = change in .1 s */
#define MANTAB_N   9

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

/* FREETAU (p. 1020): free-mode acceleration commands, 1/1600 s.          */
static const agc_sp_t freetau[4] = {
    0,    /* 0 sec       */
    480,  /* +0.30 sec (00740); the AGC comment reads +0.10 */
    -480, /* -0.30 sec (77037) */
    0     /* (+3) 0 sec  */
};

static agc_status_t rcs_setwbody(void);
static agc_status_t rcs_freefunc(void);

/* MANUAL ROTATION COMMANDS .. NOCHANGE (pp. 1017-1018).                 */
agc_status_t csm_rcs_manual(void)
{
    agc_csm_rcs_t    *r = agc_csm_dap_rcs();
    agc_csm_dapcom_t *c = agc_csm_dap_com();

    c->rcsflags = (uint16_t) (c->rcsflags & ~(unsigned) FORCEDBITS);
    uint16_t ch31 = agc_chan_read(AGC_CHAN_CHAN31);
    uint16_t chg  = (uint16_t) ((r->ch31temp ^ ch31) & MANROT); /* RXOR LCHAN */

    if (chg != 0u)
    {
        r->ch31temp = ch31; /* save channel 31 in CH31TEMP       */
        /* MP BIT5: bits 6-1 into bits 10-5: 10,9 roll, 8,7 yaw, 6,5 pitch */
        c->rcsflags = (uint16_t) (c->rcsflags + (unsigned) (chg << 4));
        c->rcsflags = (uint16_t) (c->rcsflags | DAMPBITS); /* rate damping */
    }

    uint16_t cmd = (uint16_t) (~(unsigned) r->ch31temp & MANROT); /* NOCHANGE */

    if (cmd == 0u)
    {
        return (csm_rcs_ahfnorot()); /* no manual commands                */
    }

    r->holdflag = (agc_sp_t) cmd; /* set HOLDFLAG +                    */
    agc_status_t st = csm_rcs_stickchk(cmd);

    if (st == AGC_STATUS_OK)
    {
        agc_flag_set(AGC_FLAG_STIKFLAG); /* inform P20: astronaut control */
        st = AGC_STATUS_OK;
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if ((agc_chan_read(AGC_CHAN_CHAN31) & (uint16_t) AGC_CH031_FREE) == 0u)
    {
        return (rcs_freefunc());
    }

    if (csm_rcs_rate_init_pending())
    {
        return (csm_rcs_reinit()); /* ..... TILT, recycle the filter    */
    }

    if (r->ratindx - 5 > 0)
    {
        r->attkalmn = -1; /* manual maneuver at high rate      */
    }

    return (rcs_setwbody());
}

/* SETWBODY .. SPNDXCHK (pp. 1018-1019), auto-hold manual rotation.       */
static agc_status_t rcs_setwbody(void)
{
    agc_csm_rcs_t          *r      = agc_csm_dap_rcs();
    const agc_csm_dapcom_t *c      = agc_csm_dap_com();
    bool                    damped = (c->rcsflags & DAMPBITS) == 0u; /* rate damping completed */

    for (int i = AGC_CSM_AXES - 1; i >= 0; i--)
    {
        int m = (int) r->manndx[i]; /* 0 none, 1 +, 2 - rotation         */

        if (m == 0)
        {
            r->wbody[i] = 0; /* NORATE                            */
        }
        else
        {
            int q = m + (int) r->ratindx - 1; /* MANTABLE -1 + Q         */

            if (!AGC_ASSERT((q >= 0) && (q < MANTAB_N)))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            r->wbody[i] = agc_sp_mul(mantable[q], BIT9); /* WBODY       */

            if (damped)
            { /* MERUPDAT    */
                agc_sp_t neg = (agc_sp_t) -mantable[q];

                r->merror[i] = csm_dap_das(r->merror[i], agc_sp_mul(neg, BIT7));
            }
        }

        if (!damped)
        {
            r->merror[i] = 0; /* ZEROER                            */
        }

        r->error[i] = agc_dp_high(r->merror[i]); /* SPNDXCHK    */
    }

    if (!AGC_ASSERT(r->ratindx >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (csm_rcs_jets());
}

/* FREEFUNC (pp. 1019-1020): acceleration commands in free mode.          */
static agc_status_t rcs_freefunc(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();

    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        int m = (int) r->manndx[i];

        if (!AGC_ASSERT((m >= 0) && (m <= 3)))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        r->tau[i] = freetau[m]; /* TAU, TAU1, TAU2                   */
    }

    if (!AGC_ASSERT(r->tau[0] >= -480))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (csm_rcs_t6progm());
}

/* STICKCHK (p. 1021): RMANNDX, PMANNDX, YMANNDX = 0, 1, 2 (or 3) for no,
 * +, or - rotation (both) from the six command bits. After the RHC is
 * returned to detent the indices keep their last out-of-detent values.  */
agc_status_t csm_rcs_stickchk(uint16_t bits)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();

    if (!AGC_ASSERT(bits <= MANROT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    r->manndx[1] = (agc_sp_t) (bits & 3u); /* PMANNDX: bits 1,2   */
    unsigned t = (unsigned) bits >> 2;   /* MP QUARTER          */

    r->manndx[2] = (agc_sp_t) (t & 3u);    /* YMANNDX: bits 3,4   */
    r->manndx[0] = (agc_sp_t) (t >> 2);    /* RMANNDX: bits 5,6   */

    if (!AGC_ASSERT(r->manndx[0] <= 3))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* STICKCHK (p. 1021) for other lanes (SATSTICK): see csm_rcs_stickchk. */
agc_status_t agc_csm_dap_stickchk(uint16_t ch31_bits)
{
    if (!AGC_ASSERT(ch31_bits <= 077u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = csm_rcs_stickchk(ch31_bits);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}
