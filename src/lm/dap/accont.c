/**********************************************************************
 * FILE NAME: accont.c
 *
 * PURPOSE:
 *   1/ACCONT (AOSTASK_AND_AOSJOB.agc, Luminary 099 pp.1497-1506): the
 *   TJETLAW switch-curve parameters in the 48-word BLOCKTOP table.
 *
 * AGC SOURCE:
 *   Luminary099 AOSTASK_AND_AOSJOB.agc pp. 1497-1506
 *     (1/ACCONT, DOPAXIS, BOTHAXES, BIGAOS, NOTMUCH, SKIPDB2, 1/ANET-,
 *     DO1/NET+, FIXMIN, STMIN-, DBFUN, STORV, DRFDB, INVERT)
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   DAP block     lm_dap_t  I/O  per field      EBANK 6 DAP erasables (lm_dap_state())
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ------------------------------------
 *   agc_flag_test               test a flag bit (flags.h)
 *   agc_inhint                  INHINT
 *   agc_relint                  RELINT
 *   lm_dap_dv                   DV quotient (arith.c)
 *   lm_dap_mp                   MP: A,L product (arith.c)
 *   lm_dap_mphi                 A after MP (arith.c)
 *   lm_dap_ovf                  TS overflow-skip test (arith.c)
 *   lm_dap_rot_touv             ROT-TOUV: Q,R to U,V (qrjets.c)
 *   lm_dap_state                DAP erasables (state.c)
 *   lm_dap_wrap                 overflow-corrected store, TS/ADS/XCH (arith.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. AGC_STATUS_BAD_ARG when an internal assertion fails.
 *
 * NOTES:
 *   For the P axis (2 jets, no offset) and for each of U and V (1 and 2
 *   jets, each rotation sense, with the offset acceleration AOSU/AOSV from
 *   ROT-TOUV of AOSQ/AOSR) it stores the reciprocal net accelerations 1/ANET,
 *   the reciprocal coast accelerations 1/ACOAST, the acceleration functions
 *   ACCFUN = -1/(ANET + ANET^2/ACOAST), the fire and coast deadbands and
 *   AXISDIST, FLAT and ZONE3LIM (width and height of the minimum-impulse
 *   zone), and ACCSWU/ACCSWV (max jets needed because one jet cannot beat
 *   the offset). Jet-pair failures (CH5MASK) substitute one-jet values.
 *
 *   Scales: AOS, ANET, 1JACCU at pi/2 rad/s^2; 1/ANET, 1/ACOAST, ACCFUN at
 *   2^7/pi; deadbands at pi/4 rad; ZONE3LIM at 4 s. AMIN = .03 rad/s^2 is
 *   -.03R/S2 = OCT 77377 (-pi/2^7 at pi/2).
 *
 *   The executive temporaries 1/ATEM1 .. Z3TEM (VBUF) are modelled by the
 *   24-word array t[], with the AGC offsets (T_* below), so that the
 *   BLOCKTOP stores are the AGC's GENTRAN / DXCH copies.
 **********************************************************************/
#include "agc/core/assert.h"
#include "agc/lm/dap.h"
#include "internal.h"

#define T_ATEM1  0  /* 1/ATEM1 (+2): 1/ANET for one sense              */
#define T_ATEM2  1  /* 1/ATEM2 (+2): 1/ANET for the other              */
#define T_ACOSTT 4  /* 1/ACOSTT, +1                                    */
#define T_Z1     6  /* Z1TEM (+2): ACCFUN (ACCFCTZ1)                   */
#define T_Z5     7  /* Z5TEM (+2): ACCFUN (ACCFCTZ5)                   */
#define T_UDB    10 /* UDB1, UDB2, UDB4, UDB3, UAXDIST (+1)            */
#define T_DBB1   16 /* DBB1, DBB2, DBB4, DBB3, AXDSTEM (+1)            */
#define T_DBB2   17 /* DBB2 (DB2) */
#define T_DBB4   18 /* DBB4 (DB4) */
#define T_DBB3   19 /* DBB3 (DB3) */
#define T_AXD    20 /* AXDSTEM, +1: AXISDIST for the U (V) axis */
#define T_FLAT   22 /* FLATEMP                                         */
#define T_Z3     23 /* Z3TEM (must follow FLATEMP)                     */
#define T_N      24 /* number of temporaries */
#define T_XFER   10 /* GENTRAN: ten words to 1/ANET1 (+16D)            */

#define AA_BIT8     ((agc_sp_t) 128)   /* .0125RS: AMIN/2; 1 at 2^7        */
#define AA_BIT9     ((agc_sp_t) 256)   /* 1 at 2^6; -(-.03R/S2) = AMIN     */
#define AA_BIT12    ((agc_sp_t) 2048)  /* 1 at 2^3; 9/8 = 1 + BIT12 (9/8 |AOS|) */
#define AA_BIT13    ((agc_sp_t) 4096)  /* .25 at 2^0 (.25 DB) */
#define AA_BIT14    ((agc_sp_t) 8192)  /* .5: MP BIT14 halves (rescales pi/2 -> pi) */
#define AA_M1875    ((agc_sp_t) -3072) /* -.1875 pi/2 rad/s^2 at pi/2      */
#define AA_FLATVAL  ((agc_sp_t) 291)   /* DEC .01778: .8 deg at pi/4       */
#define AA_ZONE3MAX ((agc_sp_t) 72)    /* DEC .004375: 17.5 ms at 4 s      */
#define AA_BLK_FLAT 6                  /* FLAT = BLOCKTOP +6               */
#define AA_BLK_Z3   7                  /* ZONE3LIM = BLOCKTOP +7           */

/* -UMASK, +UMASK (p.1506): channel 5 jets for -U, -V, +U, +V.             */
static const uint16_t uvmask[4] = {000110u, 000022u, 000204u, 000041u};

typedef struct acc_work
{
    agc_sp_t t[T_N];     /* VBUF temporaries at the AGC offsets, scales per T_* above (1/ANET 2^7/pi, deadbands pi/4 rad) */
    agc_sp_t dbval2;     /* DBVAL2 = -.75 DB, scaled pi/4 rad */
    agc_sp_t dbval3;     /* DBVAL3 = .5 DB, scaled pi/4 rad */
    agc_sp_t drifter;    /* 1 in drifting flight, else 0 (unscaled) */
    int      uv;         /* 0 U axis, 1 V axis                         */
    int      s;          /* SIGNAOS: 1 when AOS < 0, else 0 (index) */
    int      ns;         /* -SIGNAOS (index) */
    agc_sp_t absaos;     /* ABSAOS, scaled pi/2 rad/s^2 */
    agc_sp_t anet;       /* ANET scaled pi/2 rad/s^2, then FUNTEM scaled 2^6 */
    agc_sp_t inv;        /* 1/ANET at 2^7/pi                           */
    agc_sp_t accsw;      /* ACCSW: -1, 0, +1 max-jet request (unscaled) */
    bool     negzero[2]; /* AOSU, AOSV are -0 in the AGC (value 0 here) */
} acc_work_t;

/**********************************************************************
 * The sign of a zero MP major word (yaAGC MP): +0 when the memory operand
 * k is 0, else the product sign (a = +0 counts as positive). a is AOSQ or
 * AOSR, taken as +0 when zero (1/ACCSET and fresh start store +0).
 * AGC: none (helper)
 *
 * FUNCTION NAME: mp_negzero
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   a         agc_sp_t  I    AOSQ or AOSR major word, scaled pi/2
 *   k         agc_sp_t  I    COEFFQ or COEFFR, scaled 1
 *
 * RETURNS: bool -- true when the MP major word is -0
 **********************************************************************/
static bool mp_negzero(agc_sp_t a, agc_sp_t k)
{
    int32_t p = (int32_t) a * k; /* product a k, scaled 2^-14 (sign of the MP major word) */

    return ((k != 0) && ((a == 0) ? (k < 0) : ((p < 0) && (p > -16384)))); /* A = +0: sign of k */
}

/**********************************************************************
 * INVERT (p.1504): 1/x, x at pi/2 -> 2^7/pi (BIT9, 0 DV x). x = AMIN
 * (BIT9, from NETNEG or a failed jet pair in FIXMIN) gives POSMAX. x is
 * the unlimited sum in A: when it has overflowed (9/8 |AOS| near pi/2),
 * TS HOLD stores the overflow-corrected word and skips CA BIT9, so the
 * AGC divides +-1 (the TS overflow indicator), not BIT9, by that word.
 *
 * FUNCTION NAME: invert
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  --------------------
 *   x         int32_t  I    unlimited sum in A, scaled pi/2 rad/s^2
 *
 * RETURNS: agc_sp_t -- 1/x, scaled 2^7/pi
 **********************************************************************/
static agc_sp_t invert(int32_t x)
{
    lm_dp_pair_t one = {AA_BIT9, 0};
    agc_sp_t     hold; /* HOLD: the divisor word, scaled pi/2 rad/s^2 */

    if (lm_dap_ovf(x))
    {
        hold   = lm_dap_wrap(x);
        one.hi = (agc_sp_t) ((x > 0) ? 1 : -1); /* the TS overflow indicator +-1 */
    }
    else
    {
        hold = (agc_sp_t) x;
    }

    agc_sp_t r = lm_dap_dv(one, hold); /* 1/x, scaled 2^7/pi */

    return (r);
}

/**********************************************************************
 * DO1/NET+ / DOACCFUN (p.1504): with ANET in w->anet and the ratio term
 * ANET/ACOAST (at 2^6) in a: FUNTEM = 1 + ratio, 1/ANET, and the return
 * value ACCFUN = -1/ANET / FUNTEM at 2^7/pi.
 *
 * FUNCTION NAME: net_accel_function
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   w         acc_work_t *  I/O  working cells of this computation
 *   a         agc_sp_t      I    ratio ANET/ACOAST, scaled 2^6
 *
 * RETURNS: agc_sp_t -- ACCFUN, scaled 2^7/pi
 **********************************************************************/
static agc_sp_t net_accel_function(acc_work_t *w, agc_sp_t a)
{
    agc_sp_t funtem = lm_dap_wrap((int32_t) a + AA_BIT9); /* FUNTEM = 1 + ANET/ACOAST, scaled 2^6 (XCH ANET) */

    w->inv  = invert(w->anet);
    w->anet = funtem;

    return (lm_dap_dv(lm_dap_mp((agc_sp_t) -AA_BIT9, w->inv), funtem));
}

/**********************************************************************
 * 1/NETMIN (p.1504): ratio ANET / ACOAST of the opposite sense.
 *
 * FUNCTION NAME: net_min
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   w         acc_work_t *  I/O  working cells of this computation
 *
 * RETURNS: agc_sp_t -- ACCFUN for the minimum-jet case, scaled
 *          2^7/pi
 **********************************************************************/
static agc_sp_t net_min(acc_work_t *w)
{
    agc_sp_t a = lm_dap_mphi(w->anet, w->t[T_ACOSTT + 1 + w->ns]); /* ANET/ACOAST, scaled 2^6 */

    return (net_accel_function(w, a));
}

/**********************************************************************
 * NOTMUCH (p.1500): |AOS| <= AMIN: ACOAST = AMIN both ways; powered
 * flight deadbands SOMEAOS / NOAOS.
 *
 * FUNCTION NAME: db_smallaos
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  --------------------
 *   d         const lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   w         acc_work_t *      I/O  working cells of this computation
 *
 * RETURNS: none
 **********************************************************************/
static void db_smallaos(const lm_dap_t *d, acc_work_t *w)
{
    w->t[T_ACOSTT]     = LM_POSMAX;
    w->t[T_ACOSTT + 1] = LM_POSMAX;

    if (w->t[T_FLAT] <= 0) /* drift or GTS   */
    {
        if ((int32_t) w->absaos - AA_BIT8 > 0)
        { /* SOMEAOS        */
            w->t[T_DBB3 + w->ns] = w->dbval3; /* DB/2           */
            w->t[T_DBB4 + w->s]  = lm_dap_wrap(2 * (int32_t) w->dbval3);
        }
        else
        { /* NOAOS          */
            w->t[T_DBB3] = d->db;
            w->t[T_DBB4] = d->db;
        }
    }
}

/**********************************************************************
 * BOTHAXES .. ACCTHERE (pp.1498-1501): sign and size of the offset,
 * deadbands, 1/ACOAST, then the maximum- and minimum-jet functions for
 * the sense the offset aids (POS(NEG)).
 *
 * FUNCTION NAME: axis_with_aos
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  --------------------
 *   d         const lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   w         acc_work_t *      I/O  working cells of this computation
 *
 * RETURNS: none
 **********************************************************************/
static void axis_with_aos(const lm_dap_t *d, acc_work_t *w)
{
    agc_sp_t aos = (w->uv == 0) ? d->aosu : d->aosv; /* AOSU or AOSV, scaled pi/2 rad/s^2 */

    w->s           = ((aos < 0) || w->negzero[w->uv]) ? 1 : 0; /* SIGNAOS: 1 for AOS < 0 or -0 */
    w->ns          = -w->s;
    w->absaos      = (agc_sp_t) ((aos < 0) ? -aos : aos);      /* ABS(AOS) */
    w->t[T_DBB1]   = d->db;
    w->t[T_DBB2]   = d->db;
    agc_sp_t jaccu = d->jacc[3];                               /* 1JACCU, scaled pi/2 rad/s^2 */
    agc_sp_t acc;                                              /* ACCFUN, scaled 2^7/pi */

    if ((int32_t) w->absaos - AA_BIT9 <= 0)
    {
        db_smallaos(d, w);
    }
    else
    {
        if (w->t[T_FLAT] == 0)
        {
            /* BIGAOS (pp.1498-1499): |AOS| > AMIN, powered: deadbands */
            agc_sp_t db = d->db; /* DBVAL1 = DB, scaled pi/4 rad */

            /* ADS leaves the sum in A, so DB4(3) gets 2 DB, not the 1 DB
             * of the listing's comment (rope: accs_aos_big COASTDB).    */
            w->t[T_DBB2 + w->ns] = lm_dap_wrap((int32_t) w->t[T_DBB2 + w->ns] + db);
            w->t[T_DBB4 + w->s]  = w->t[T_DBB2 + w->ns];
            agc_sp_t db1; /* DB1(2), scaled pi/4 rad */

            if ((int32_t) AA_M1875 + w->absaos > 0)
            {
                db1 = (agc_sp_t) -w->dbval3; /* -.5 DB         */
            }
            else
            {
                int32_t f = 2 * ((int32_t) -4 * w->absaos + AA_BIT14); /* 1 - 8|AOS| */

                db1 = lm_dap_mphi(lm_dap_wrap(f), db);
            }

            w->t[T_DBB1 + w->s]  = db1;       /* DBONE          */
            w->t[T_DBB3 + w->ns] = w->dbval2; /* -.75 DB        */
        }

        int32_t nine8 = (int32_t) lm_dap_mphi(w->absaos, AA_BIT12) + w->absaos; /* 9/8 |AOS| in A, scaled pi/2 rad/s^2 */

        w->t[T_ACOSTT + 1 + w->ns] = invert(nine8); /* 1/(9/8 |AOS|)  */
        w->t[T_ACOSTT + w->s]      = LM_POSMAX;     /* 1/AMIN         */
    } /* end BIGAOS: |AOS| > AMIN */

    int32_t top = (int32_t) w->absaos + 2 * (int32_t) jaccu + AA_BIT9; /* |AOS| + 2 JACC + 1 at 2^6: overflow test, scale pi/2 */

    if (w->absaos > AA_BIT9 && lm_dap_ovf(top))
    {
        /* SKIPDB1 overflow branch (p.1500): ANET rescaled to pi.                 */
        agc_sp_t anet_pi = lm_dap_wrap((int32_t) lm_dap_mphi(w->absaos, AA_BIT14) + jaccu); /* ANET at pi: |AOS|/2 + JACC, scaled pi rad/s^2 */

        w->anet = lm_dap_wrap((int32_t) anet_pi + AA_BIT8); /* 1 + .. at 2^7  */
        w->inv  = lm_dap_mphi(invert(anet_pi), AA_BIT14);   /* at 2^7/pi      */

        acc = lm_dap_dv(lm_dap_mp((agc_sp_t) -AA_BIT8, w->inv), w->anet);
    }
    else
    { /* SKIPDB2        */
        w->anet = lm_dap_wrap((int32_t) w->absaos + 2 * (int32_t) jaccu);
        acc     = net_accel_function(w, w->anet);
    }

    w->t[T_Z5 + 2 + w->ns]    = acc;                                   /* ACCTHERE       */
    w->t[T_ATEM2 + 2 + w->ns] = w->inv;
    int32_t m                 = (int32_t) w->absaos + jaccu + AA_BIT9; /* |AOS| + JACC + 1 at 2^6, scaled pi/2 (overflow test) */

    w->anet               = (agc_sp_t) ((lm_dap_ovf(m) ? LM_POSMAX : m) - AA_BIT9); /* overflow: truncate to pi/2 */
    acc                   = net_accel_function(w, w->anet);
    w->t[T_Z5 + w->ns]    = acc;
    w->t[T_ATEM2 + w->ns] = w->inv;
} /* axis_with_aos */

/**********************************************************************
 * STORV (pp.1502-1503) and the U-axis store: ACCSWx and the ten words
 * (GENTRAN) to 1/ANET1 or 1/ANET1 +16D; the U deadbands are kept in
 * UDB1.. while V is computed.
 *
 * FUNCTION NAME: store_axis
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   d         lm_dap_t *    I/O  DAP erasables (lm_dap_state())
 *   w         acc_work_t *  I/O  working cells of this computation
 *
 * RETURNS: none
 **********************************************************************/
static void store_axis(lm_dap_t *d, acc_work_t *w)
{
    agc_inhint();
    d->block[w->uv] = w->accsw;              /* ACCSWU / ACCSWV */
    int base        = LM_BLK_U + 16 * w->uv; /* first word of the U or V BLOCKTOP group (index) */

    for (int i = 0; i < T_XFER; i++)
    {
        d->block[base + i] = w->t[i];
    }

    agc_relint();

    if (w->uv == 0)
    {
        for (int i = 0; i < 6; i++)
        { /* DXCH to UDB1.. */
            w->t[T_UDB + i] = w->t[T_DBB1 + i];
        }
    }
}

/**********************************************************************
 * STORV end, DRFDB (pp.1503-1504): FLAT, ZONE3LIM, P deadbands, and the
 * U,V FIREDB, COASTDB, AXISDIST (drift deadbands when FLAT > 0).
 *
 * FUNCTION NAME: store_deadbands
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  --------------------
 *   d         lm_dap_t *          I/O  DAP erasables (lm_dap_state())
 *   w         const acc_work_t *  I/O  working cells of this computation
 *
 * RETURNS: none
 **********************************************************************/
static void store_deadbands(lm_dap_t *d, const acc_work_t *w)
{
    d->block[AA_BLK_FLAT] = w->t[T_FLAT];
    d->block[AA_BLK_Z3]   = w->t[T_Z3];
    agc_sp_t coast        = lm_dap_wrap((int32_t) d->db + w->t[T_FLAT]); /* PDB3/4 = DB + FLAT, scaled pi/4 rad */

    d->block[10] = d->db; /* PDB1, PDB2     */
    d->block[11] = d->db;
    d->block[12] = coast; /* PDB4, PDB3     */
    d->block[13] = coast;
    d->block[14] = 0;     /* PAXDIST        */
    d->block[15] = 0;

    for (int a = 0; a < 2; a++)
    {
        int b = LM_BLK_U + 16 * a; /* first word of the U or V group (index) */

        for (int i = 0; i < 6; i++)
        {
            agc_sp_t v = w->t[((a == 0) ? T_UDB : T_DBB1) + i]; /* U words come from UDB1.., V from DBB1.., scaled pi/4 rad */

            if (w->t[T_FLAT] > 0)
            { /* DRFDB          */
                v = 0; /* AXISDIST: 0 */

                if (i < 2)
                {
                    v = d->db; /* FIREDB = DB */
                }
                else if (i < 4)
                {
                    v = coast; /* COASTDB = DB + FLAT */
                }
            }

            d->block[b + LM_BK_FIREDB + i] = v;
        }
    }
} /* store_deadbands */

/**********************************************************************
 * NEG(POS) cases, 1/ANET-, NETNEG, FIXMIN, STMIN- (pp.1501-1502,
 * 1504-1505): the U or V axis values for the sense the offset
 * opposes. If one jet cannot beat the offset (ANET < AMIN) the two-jet
 * values are used and ACCSW demands max jets, unless that pair has
 * failed, when AMIN is assumed.
 *
 * FUNCTION NAME: axis_against_aos
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  --------------------
 *   d         const lm_dap_t *  I    DAP erasables (lm_dap_state())
 *   w         acc_work_t *      I/O  working cells of this computation
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion
 **********************************************************************/
static agc_status_t axis_against_aos(const lm_dap_t *d, acc_work_t *w)
{
    agc_sp_t jaccu = d->jacc[3]; /* 1JACCU, scaled pi/2 rad/s^2 */

    /* 1/ANET- (p.1504): ACCSW = 0; an ANET below AMIN is raised to AMIN
     * (NETNEG); then 1/NETMIN.                                                */
    agc_sp_t a = lm_dap_wrap(2 * (int32_t) jaccu - w->absaos); /* ANETNEG(POS) MAX = 2 JACC - |AOS|, scaled pi/2 rad/s^2 */

    w->accsw = 0;
    w->anet  = a;

    if ((int32_t) a - AA_BIT9 <= 0)
    {
        w->anet = AA_BIT9; /* NETNEG         */
    }

    if (!AGC_ASSERT(w->anet >= AA_BIT9))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_sp_t acc   = net_min(w); /* ACCFUN, scaled 2^7/pi */
    agc_sp_t lsave = acc;        /* TS L: ACCFUN saved, scaled 2^7/pi */

    if (!AGC_ASSERT(w->s == -w->ns))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    w->t[T_Z1 + 2 + w->s]    = acc;
    w->t[T_ATEM1 + 2 + w->s] = w->inv;
    w->anet                  = lm_dap_wrap((int32_t) jaccu - w->absaos);

    if ((int32_t) w->anet - AA_BIT9 > 0)
    {
        acc = net_min(w);
    }
    else
    { /* FIXMIN         */
        w->accsw = (agc_sp_t) ((w->s != 0) ? 1 : -1); /* AOS neg: +1, pos: -1 */

        if ((uvmask[w->accsw + 1 + w->uv] & d->ch5mask) != 0u)
        {
            w->anet = AA_BIT9; /* failed: AMIN   */
            acc     = net_min(w);
        }
        else
        {
            acc = lsave; /* max-jet values */
        }
    }

    w->t[T_Z1 + w->s]    = acc; /* STMIN-         */
    w->t[T_ATEM1 + w->s] = w->inv;

    return (AGC_STATUS_OK);
} /* axis_against_aos */

/**********************************************************************
 * 1/ACCONT (p.1497): deadband values, AOSU/V, FLAT and ZONE3LIM, then
 * the P, U and V axes.
 *
 * FUNCTION NAME: lm_dap_switch_curves
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_switch_curves(void)
{
    lm_dap_t  *d       = lm_dap_state();               /* DAP erasables */
    acc_work_t w       = {{0}, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, {false, false}};
    agc_sp_t   quarter = lm_dap_mphi(d->db, AA_BIT13); /* .25 DB, scaled pi/4 rad */

    w.dbval3 = lm_dap_wrap(2 * (int32_t) quarter);     /* .50 DB         */
    w.dbval2 = lm_dap_wrap((int32_t) quarter - d->db); /* -.75 DB        */
    agc_inhint();                                      /* GETAOSUV       */
    lm_dap_rot_touv(d->aosq, d->aosr, &d->aosu, &d->aosv);
    agc_relint();

    /**********************************************************************
     * GETAOSUV (p.1497): which of the ROT-TOUV results AOSU (A), AOSV (L) the
     * AGC leaves as -0. A ones'-complement sum of zero is -0 unless both terms
     * are +0: V = COEFFQ AOSQ + COEFFR AOSR, U = -COEFFQ AOSQ + COEFFR AOSR.
     * With AOSQ = AOSR = +0 (COEFFQ < 0) this makes AOSV = -0, which BOTHAXES
     * takes as a negative offset (CCS fourth branch: SIGNAOS = 1).
     *
     * FUNCTION NAME: aos_negzero
     *
     * ARGUMENTS:
     *   Argument  Type              I/O  Description (scale)
     *   --------  ----------------  ---  --------------------
     *   d         const lm_dap_t *  I/O  DAP erasables (lm_dap_state())
     *   w         acc_work_t *      I/O  working cells of this computation
     *
     * RETURNS: none
     **********************************************************************/
    agc_sp_t tq = lm_dap_mphi(d->aosq, d->coeffq);             /* COEFFQ AOSQ, scaled pi/2 rad/s^2 */
    agc_sp_t tr = lm_dap_mphi(d->aosr, d->coeffr);             /* COEFFR AOSR, scaled pi/2 rad/s^2 */
    bool     nq = (tq == 0) && mp_negzero(d->aosq, d->coeffq); /* that product is -0 */
    bool     nr = (tr == 0) && mp_negzero(d->aosr, d->coeffr); /* that product is -0 */
    bool     pq = (tq == 0) && !nq;                            /* COEFFQ AOSQ = +0  */
    bool     pr = (tr == 0) && !nr;                            /* COEFFR AOSR = +0  */

    w.negzero[0] = (d->aosu == 0) && !(nq && pr);                            /* -(-0) + +0 = +0   */
    w.negzero[1] = (d->aosv == 0) && !(pq && pr);
    w.drifter    = agc_flag_test(AGC_FLAG_DRIFTDFL) ? 1 : 0;                 /* DRIFTER */
    w.t[T_FLAT]  = ((int32_t) w.drifter + d->allowgts > 0) ? AA_FLATVAL : 0; /* FLATEMP: .8 deg in drift or GTS */

    /* DEVIATION: in powered flight without GTS the AGC leaves Z3TEM (a
     * VBUF word) as it was and copies it to ZONE3LIM; here ZONE3LIM
     * keeps its previous value.                                          */
    w.t[T_Z3] = d->block[AA_BLK_Z3];

    if (w.t[T_FLAT] != 0)
    {
        w.t[T_Z3] = (w.drifter != 0) ? AA_ZONE3MAX : 0; /* ZONE3LIM: 17.5 ms drifting, 0 in GTS */
    }

    /**********************************************************************
     * DOPAXIS (pp.1497-1498): P axis, 2 jets, no AOS: ACOAST = AMIN.
     *
     * FUNCTION NAME: p_axis
     *
     * ARGUMENTS:
     *   Argument  Type        I/O  Description (scale)
     *   --------  ----------  ---  --------------------
     *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
     *
     * RETURNS: none
     **********************************************************************/
    agc_sp_t funtem = lm_dap_wrap((int32_t) d->jacc[0] + AA_BIT9); /* FUNTEM = 1 + ANET/ACOAST, scaled 2^6 */
    agc_sp_t inv    = invert(d->jacc[0]);                          /* 1/ANETP, scaled 2^7/pi */

    agc_inhint();                                                         /* consistent     */
    d->block[2] = inv;                                                    /* 1/ANETP        */
    d->block[3] = inv;
    agc_sp_t f  = lm_dap_dv(lm_dap_mp((agc_sp_t) -AA_BIT9, inv), funtem); /* PACCFUN, scaled 2^7/pi */

    d->block[8] = f;         /* PACCFUN        */
    d->block[9] = f;
    d->block[4] = LM_POSMAX; /* 1/ACOSTP = 1/.03 */
    d->block[5] = LM_POSMAX;
    agc_relint();

    if (w.drifter != 0)
    {
        d->aosu      = 0; /* just to be sure (+0) */
        d->aosv      = 0;
        w.negzero[0] = false;
        w.negzero[1] = false;
    }

    for (int uv = 0; uv < 2; uv++)
    { /* UAXIS, V axis  */
        w.uv = uv;
        axis_with_aos(d, &w);

        agc_status_t st = axis_against_aos(d, &w); /* status of the NEG(POS) cases */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        /**********************************************************************
         * FAIL- .. DBFUN (p.1502): one-jet values replace those of a failed jet
         * pair; AXISDIST from the deadbands and FLAT.
         *
         * FUNCTION NAME: axis_fail_dist
         *
         * ARGUMENTS:
         *   Argument  Type              I/O  Description (scale)
         *   --------  ----------------  ---  --------------------
         *   d         const lm_dap_t *  I/O  DAP erasables (lm_dap_state())
         *   w         acc_work_t *      I/O  working cells of this computation
         *
         * RETURNS: none
         **********************************************************************/
        if ((uvmask[2 + w.uv] & d->ch5mask) != 0u)
        { /* +U (+V) failed */
            w.t[T_ATEM2 + 2] = w.t[T_ATEM2];
            w.t[T_Z5 + 2]    = w.t[T_Z5];
        }

        if ((uvmask[w.uv] & d->ch5mask) != 0u)
        { /* -U (-V) failed */
            w.t[T_ATEM1 + 2] = w.t[T_ATEM1];
            w.t[T_Z1 + 2]    = w.t[T_Z1];
        }

        w.t[T_AXD]     = lm_dap_wrap((int32_t) w.t[T_DBB1] - w.t[T_DBB3] + w.t[T_FLAT]);
        w.t[T_AXD + 1] = lm_dap_wrap((int32_t) w.t[T_DBB2] - w.t[T_DBB4] + w.t[T_FLAT]);
        store_axis(d, &w);
    }

    store_deadbands(d, &w);

    if (!AGC_ASSERT(d->db >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
} /* lm_dap_switch_curves */
