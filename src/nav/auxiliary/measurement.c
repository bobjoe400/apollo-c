/**********************************************************************
 * FILE NAME: measurement.c
 *
 * PURPOSE:
 *   MEASUREMENT INCORPORATION: the recursive (Kalman-type) navigation
 *   update with the error transition matrix W (W W^T = covariance):
 *     INCORP1:  Z     = W^T B                  (ZI, 9 or 6 components)
 *               A     = Z.Z + VARIANCE
 *               GAMMA = 1 / (A + sqrt(A VARIANCE))
 *               OMEGA = W Z
 *               DELTAX = OMEGA DELTAQ / A      (state correction)
 *     INCORP2:  W'    = W - GAMMA OMEGA Z^T    (row by row)
 *               state += DELTAX                (TDELTAV/TNUV, or RCV/VCV
 *                                               and RECTIFY on overflow)
 *
 * AGC SOURCE:
 *   Comanche055 MEASUREMENT_INCORPORATION.agc pp. 1252-1261 (INCORP1,
 *     Z123, INCOR1A-C, INCOR2, INCOR3, INCORP2, FAZA-FAZAB5, NEWZCOMP)
 *   Luminary099 MEASUREMENT_INCORPORATION.agc (identical apart from the
 *     bank placement of NEWZCOMP)
 *
 * EXTERNAL VARIABLES:
 *   none (W, the measurement data and the integration temporaries are
 *   passed in agc_incorp_w_t, agc_incorp_t and agc_incorp_sv_t)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_vec_sq_tp         VSQ RTB TPMODE (interp.h)
 *   agc_tp_norm/sqrt      NORM, SQRT on the TP MPAC (interp.h)
 *   agc_navaux_vec_shift  VSL* / VSR* by an index value (internal.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms.  AGC_STATUS_BAD_ARG on NULL, out-of-range inputs, or
 *   A = 0 (no information in the measurement).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Called from the measurement job of P20/P22/P23 (CM) and the LM
 *   rendezvous/landing navigation.  Normalization bookkeeping uses AGC
 *   index values: NORMGAM is the AGC's (negative) X-register value; a
 *   shift "VSL* 0,i" shifts left by -C(Xi).  The scale of W, ZI and
 *   OMEGA is whatever the W-matrix integration keeps; DELTAX comes out at
 *   the state scale (position E 2^29 / M 2^27 m, velocity E 2^7 / M 2^5
 *   m/cs, bias states as X789).
 *
 * NOTES:
 *   DEVIATION (restart protection): INCORP2's PHASCHNG OCT 04022 / GRP2PC
 *   calls, UPFLAG REINTFLG, INTSTALL, the MOVEP* / MOVEA* state-vector
 *   copies, SVDWN1/2, RECTIFY and the POSTJUMP to INTWAKE belong to the
 *   caller (nav-integ / measurement job).  agc_incorp_update_w updates W
 *   in place row by row in the AGC's order; agc_incorp_update_state
 *   works on the integration temporaries supplied in agc_incorp_sv_t.
 *
 * ALGORITHM:
 *   See PURPOSE; each function prolog gives its AGC step.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/nav/auxiliary.h"
#include "internal.h"

/* DP1/4TH 2DEC .25 (oct 10000 00000): 1/4, the GAMMA numerator.           */
#define INCORP_DP_QUARTER AGC_DP_QUARTER /* 1/4, scale 2^0 */

/* INCOR1B shifts VARIANCE two places at a time while it fits: at most
 * 42/2 passes of the TP.                                                  */
#define INCORP_MAX_SL2 21 /* SL2 passes   */

/* Rows of W updated by INCORP2: 9 (DMENFLG set) or 6.                     */
#define INCORP_ROWS6 6 /* 6-dim rows   */

/**********************************************************************
 * AGC: none (addressing of W, MEASUREMENT_INCORPORATION.agc p. 1253)
 *   -- the 3x3 partition of W at AGC address W + 54b + 18a
 *
 * FUNCTION NAME: incorp_part
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ----------------------------
 *   w         const agc_incorp_w_t *  I    W, scale of the integration
 *   b         int                     I    column block, 0..2
 *   a         int                     I    row block, 0..2
 *
 * RETURNS: agc_mat_t -- rows blk[b][3a+r], r = 0..2 (identity/2 on bad
 *          arguments)
 **********************************************************************/
static agc_mat_t incorp_part(const agc_incorp_w_t *w, int b, int a)
{
    agc_mat_t m = agc_mat_identity_half(); /* partition, scale of W */

    for (int r = 0; r < 3; r++)
    {
        m.row[r] = w->blk[b][(3 * a) + r];
    }

    return (m);
}

/**********************************************************************
 * Z123 / INCOR1 (MEASUREMENT_INCORPORATION.agc p. 1253, labels Z123,
 *   INCOR1) -- ZI[a] = sum_b MXV(W part (b,a), BVECTOR[b])
 *
 * FUNCTION NAME: incorp_z
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ----------------------------
 *   ip        agc_interp_t *          I/O  interpreter state
 *   w         const agc_incorp_w_t *  I    W, scale of the integration
 *   inc       agc_incorp_t *          I/O  in: bvector, dim9; out: zi
 *                                          (ZI[2] zeroed when 6-dim)
 *
 * RETURNS: void
 **********************************************************************/
static void incorp_z(agc_interp_t *ip, const agc_incorp_w_t *w, agc_incorp_t *inc)
{
    for (int a = 0; a < 3; a++)
    { /* TIX,2 Z123      */
        agc_vec_t z = agc_vec_zero(); /* ZI[a], scale of W B */

        for (int b = 0; b < AGC_INCORP_BLKS; b++)
        {
            agc_mat_t m = incorp_part(w, b, a);                 /* scale of W */
            agc_vec_t t = agc_mat_vec(ip, &m, inc->bvector[b]); /* scale W B */

            z = agc_vec_add(ip, t, z); /* VAD* (first pass: 0 + t) */
        }

        inc->zi[a] = z;
    }

    if (!inc->dim9)
    { /* BON DMENFLG     */
        inc->zi[2] = agc_vec_zero();
    }
}

/**********************************************************************
 * INCOR1A (MEASUREMENT_INCORPORATION.agc p. 1253, label INCOR1A)
 *   -- TRIPA = Z1^2 + Z2^2 + Z3^2 + VARIANCE
 *
 * FUNCTION NAME: incorp_tripa
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description (scale)
 *   --------  --------------------  ---  ------------------------------
 *   ip        agc_interp_t *        I/O  interpreter state
 *   inc       const agc_incorp_t *  I    zi, variance (scale of ZI^2)
 *
 * RETURNS: agc_tp_t -- TRIPA (A), TP at the scale of VARIANCE; 0 on bad
 *          input
 **********************************************************************/
static agc_tp_t incorp_tripa(agc_interp_t *ip, const agc_incorp_t *inc)
{
    if (!AGC_ASSERT(agc_navaux_tp_ok(inc->variance) && (inc->variance >= 0)))
    {
        return (0);
    }

    /* VSQ RTB TPMODE keeps the full DOTSUB triple of each square.         */
    agc_tp_t a = agc_vec_sq_tp(ip, inc->zi[0]); /* sum, scale of ZI^2 */

    a = agc_tp_add(ip, a, agc_vec_sq_tp(ip, inc->zi[1]));
    a = agc_tp_add(ip, a, agc_vec_sq_tp(ip, inc->zi[2]));

    return (agc_tp_add(ip, a, inc->variance)); /* TAD VARIANCE */
}

/**********************************************************************
 * INCOR1B / INCOR1C (MEASUREMENT_INCORPORATION.agc pp. 1253-1254,
 *   labels INCOR1B, INCOR1C) -- GAMMA and NORMGAM
 *   TEMPVAR = VARIANCE 4^k (largest k without overflow), then
 *   GAMMA = (1/4) / NORM(TRIPA + sqrt(TRIPA TEMPVAR) 2^-k),
 *   NORMGAM = -(normalization count) - 2  (AGC index value),
 *   so that the true gamma is GAMMA 2^-NORMGAM
 *
 * FUNCTION NAME: incorp_gamma
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   inc       agc_incorp_t *  I/O  in: variance; out: gamma (2^0),
 *                                  normgam
 *   tripa     agc_tp_t        I    TRIPA, scale of VARIANCE
 *
 * RETURNS: void
 **********************************************************************/
static void incorp_gamma(agc_interp_t *ip, agc_incorp_t *inc, agc_tp_t tripa)
{
    agc_interp_clear_ovf(ip);          /* TLOAD BOV: clear  */
    agc_tp_t tempvar = inc->variance;  /* scale of VARIANCE 4^k */
    int      k       = 0;              /* X2 = 0            */
    bool     ovf     = (tempvar == 0); /* BZE INCOR1C       */

    for (int i = 0; (i < INCORP_MAX_SL2) && !ovf; i++)
    {
        agc_tp_t t = agc_tp_shift(ip, tempvar, 2); /* SL2: scale 4 TEMPVAR */

        ovf = agc_interp_test_ovf(ip); /* BOV INCOR1C      */

        if (!ovf)
        {
            tempvar = t; /* STORE TEMPVAR    */
            k++;         /* INCR,2 1          */
        }
    }

    /* TLOAD ROUND TRIPA, DMP TEMPVAR (its top two words): the TP product
     * goes to SQRT whole; SL* 0,2 shifts the root right k places into
     * MPAC+2 and TAD adds the TP TRIPA.                                    */
    agc_tp_t prod = agc_dp_mul_tp(agc_tp_round(ip, tripa), agc_tp_to_dp(tempvar)); /* TRIPA TEMPVAR, scale of A^2 4^k */
    agc_tp_t root = agc_tp_from_dp(agc_tp_sqrt(prod));                         /* scale A 2^k */
    agc_tp_t den  = agc_tp_shift(ip, root, -k);                                    /* SL* 0,2: scale of A */
    int      n    = 0;                                                             /* NORM count        */

    den = agc_tp_add(ip, den, tripa); /* TAD TRIPA        */

    den          = agc_tp_norm(den, &n);                                 /* NORM X2           */
    inc->normgam = -n - 2;                                               /* INCR,2 -2; SXA,2  */
    inc->gamma   = agc_dp_div(ip, INCORP_DP_QUARTER, agc_tp_to_dp(den)); /* BDDV: GAMMA, scale 2^0 */
} /* incorp_gamma */

/**********************************************************************
 * INCOR1C tail (MEASUREMENT_INCORPORATION.agc p. 1254, label INCOR1C)
 *   -- DELTAQ/A, both normalized
 *
 * FUNCTION NAME: incorp_dq_over_a
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description (scale)
 *   --------  --------------------  ---  ------------------------------
 *   ip        agc_interp_t *        I/O  interpreter state
 *   inc       const agc_incorp_t *  I    deltaq (DELTAQ)
 *   tripa     agc_tp_t              I    TRIPA (A), > 0
 *   x1        int *                 O    -m + q, the AGC X1 value
 *
 * RETURNS: agc_dp_t -- DELTAQ/A scaled 2^(q-m-1) relative to the true
 *          quotient; 0 on bad input
 **********************************************************************/
static agc_dp_t incorp_dq_over_a(agc_interp_t *ip, const agc_incorp_t *inc, agc_tp_t tripa, int *x1)
{
    int      m  = 0;                                    /* NORM X1 count */
    agc_dp_t a  = agc_tp_to_dp(agc_tp_norm(tripa, &m)); /* A 2^m, DLOAD  */
    int      q  = 0;                                    /* NORM S1 count */
    agc_dp_t dq = agc_dp_norm(inc->deltaq, &q);         /* DELTAQ 2^q    */

    *x1 = -m + q; /* XSU,1 S1 */

    return (agc_dp_div(ip, agc_dp_shr(dq, 1), a)); /* SR1 DDV  */
}

/**********************************************************************
 * NEWZCOMP / NEWZCMP1 (MEASUREMENT_INCORPORATION.agc pp. 1260-1261,
 *   labels NEWZCOMP, NEWZCMP1) -- normalize ZI by its largest magnitude
 *
 * FUNCTION NAME: incorp_newzcomp
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state
 *   inc       agc_incorp_t *  I/O  zi shifted left p-2 (p = NORM count
 *                                  of max |ZI[a]|), normgam += 2(p-2)
 *   x1_dq     int             I    X1 from INCOR1C (-m + q)
 *
 * RETURNS: int -- the X1 for INCOR3, (p-2) + x1_dq
 **********************************************************************/
static int incorp_newzcomp(agc_interp_t *ip, agc_incorp_t *inc, int x1_dq)
{
    if (!AGC_ASSERT((x1_dq <= NAVAUX_MAX_SHIFT) && (x1_dq >= -NAVAUX_MAX_SHIFT)))
    {
        return (0);
    }

    agc_dp_t normzi = agc_vec_abval(ip, inc->zi[0]); /* scale of ZI */

    for (int a = 1; a < 3; a++)
    {
        agc_dp_t mag = agc_vec_abval(ip, inc->zi[a]); /* scale of ZI */

        if (agc_dp_sub(ip, mag, normzi) >= 0)
        { /* DSU BMN         */
            normzi = mag;
        }
    }

    int p     = agc_dp_norm_count(normzi); /* NORM X1         */
    int shift = p - 2;                     /* INCR,1 2        */

    for (int a = 0; a < 3; a++)
    {
        inc->zi[a] = agc_navaux_vec_shift(ip, inc->zi[a], shift); /* VSL* 0,1 */
    }

    inc->normgam = inc->normgam + (2 * shift); /* XSU,1 twice     */

    return (shift + x1_dq); /* LXC,1 XAD,1     */
}

/**********************************************************************
 * INCOR2 (MEASUREMENT_INCORPORATION.agc p. 1254, label INCOR2)
 *   -- OMEGA[b] = sum_a VXM(ZI[a], W part (b,a))
 *
 * FUNCTION NAME: incorp_omega
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ----------------------------
 *   ip        agc_interp_t *          I/O  interpreter state
 *   w         const agc_incorp_w_t *  I    W, scale of the integration
 *   inc       agc_incorp_t *          I/O  in: zi, dim9; out: omega
 *                                          (OMEGA[2] zeroed when 6-dim)
 *
 * RETURNS: void
 **********************************************************************/
static void incorp_omega(agc_interp_t *ip, const agc_incorp_w_t *w, agc_incorp_t *inc)
{
    for (int b = 0; b < AGC_INCORP_BLKS; b++)
    { /* TIX,2 INCOR2    */
        agc_vec_t om = agc_vec_zero(); /* OMEGA[b], scale of W Z */

        for (int a = 0; a < 3; a++)
        {
            agc_mat_t m = incorp_part(w, b, a);            /* scale of W */
            agc_vec_t t = agc_vec_mat(ip, inc->zi[a], &m); /* VXM*, scale Z W */

            om = agc_vec_add(ip, t, om); /* VAD (first pass: 0 + t) */
        }

        inc->omega[b] = om;
    }

    if (!inc->dim9)
    { /* BON DMENFLG     */
        inc->omega[2] = agc_vec_zero();
    }
}

/**********************************************************************
 * INCOR3 (MEASUREMENT_INCORPORATION.agc p. 1255, label INCOR3)
 *   -- DELTAX[b] = OMEGA[b] (DELTAQ/A) shifted left -X1
 *
 * FUNCTION NAME: incorp_deltax
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state
 *   inc       agc_incorp_t *  I/O  in: omega; out: deltax (state scale;
 *                                  the velocity triple a further VSL3)
 *   dqa       agc_dp_t        I    DELTAQ/A from INCOR1C (normalized)
 *   x1        int             I    X1 from NEWZCOMP
 *
 * RETURNS: void
 **********************************************************************/
static void incorp_deltax(agc_interp_t *ip, agc_incorp_t *inc, agc_dp_t dqa, int x1)
{
    if (!AGC_ASSERT((x1 <= NAVAUX_MAX_SHIFT) && (x1 >= -NAVAUX_MAX_SHIFT)))
    {
        return;
    }

    for (int b = 0; b < AGC_INCORP_BLKS; b++)
    { /* TIX,2 INCOR3    */
        agc_vec_t d = agc_vec_scale(ip, inc->omega[b], dqa); /* scale 2^X1 */

        inc->deltax[b] = agc_navaux_vec_shift(ip, d, -x1); /* VSL* 0,1 */
    }

    inc->deltax[1] = agc_vec_shl(ip, inc->deltax[1], 3); /* VSL3   */
}

/**********************************************************************
 * INCORP1 (MEASUREMENT_INCORPORATION.agc pp. 1252-1255, label INCORP1,
 *   with NEWZCOMP pp. 1260-1261) -- state deviation from a measurement
 *
 * FUNCTION NAME: agc_incorp_state_deviation
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ----------------------------
 *   ip        agc_interp_t *          I/O  interpreter state
 *   w         const agc_incorp_w_t *  I    W, scale of the integration
 *   inc       agc_incorp_t *          I/O  in: dim9 (DMENFLG), bvector,
 *                                          deltaq, variance; out: zi
 *                                          (normalized), omega, deltax,
 *                                          gamma, normgam
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on NULL,
 *          out-of-range input or A = 0
 **********************************************************************/
agc_status_t agc_incorp_state_deviation(agc_interp_t         *ip,
                                        const agc_incorp_w_t *w,
                                        agc_incorp_t         *inc)
{
    if (!AGC_ASSERT(agc_navaux_tp_ok(inc->variance) && (inc->variance >= 0) && agc_navaux_dp_ok(inc->deltaq)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    incorp_z(ip, w, inc);
    agc_tp_t tripa = incorp_tripa(ip, inc); /* A, scale of VARIANCE */

    if (!AGC_ASSERT(tripa > 0))
    { /* A = 0: no information at all */
        return (AGC_STATUS_BAD_ARG);
    }

    incorp_gamma(ip, inc, tripa);
    int      x1  = 0;                                     /* AGC X1     */
    agc_dp_t dqa = incorp_dq_over_a(ip, inc, tripa, &x1); /* 2^(q-m-1) */

    x1 = incorp_newzcomp(ip, inc, x1);
    incorp_omega(ip, w, inc);
    incorp_deltax(ip, inc, dqa, x1);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * INCORP2 phases 1-2, FAZA..FAZB5 (MEASUREMENT_INCORPORATION.agc
 *   pp. 1256-1258, labels INCORP2, FAZA, FAZB5) -- W' = W - GAMMA OMEGA Z^T
 *
 * FUNCTION NAME: agc_incorp_update_w
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description (scale)
 *   --------  --------------------  ---  ------------------------------
 *   ip        agc_interp_t *        I/O  interpreter state
 *   inc       const agc_incorp_t *  I    zi, omega, gamma, normgam, dim9
 *   w         agc_incorp_w_t *      I/O  W, scale of the integration
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on NULL or
 *          out-of-range NORMGAM/ZI
 *
 * One row m (ZI component m) at a time, m = 0..8 (DMENFLG set) or 0..5,
 * blocks 0..2 or 0..1.  The AGC stages each row in HOLDW under restart
 * protection; the C update of a row is atomic so HOLDW is not needed.
 **********************************************************************/
agc_status_t agc_incorp_update_w(agc_interp_t *ip, const agc_incorp_t *inc, agc_incorp_w_t *w)
{
    if (!AGC_ASSERT((inc->normgam <= NAVAUX_MAX_SHIFT) && (inc->normgam >= -NAVAUX_MAX_SHIFT)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t omegam[AGC_INCORP_BLKS]; /* GAMMA OMEGA, scale of W */

    for (int b = 0; b < AGC_INCORP_BLKS; b++)
    { /* OMEGAMb        */
        omegam[b] = agc_vec_scale(ip, inc->omega[b], inc->gamma);
    }

    int rows = INCORP_ROWS6; /* FAZB5: rows of W updated */
    int blks = 2;            /* BOFF DMENFLG: column blocks */

    if (inc->dim9)
    {
        rows = AGC_INCORP_ROWS;
        blks = AGC_INCORP_BLKS;
    }

    for (int m = 0; m < rows; m++)
    { /* TIX,1 / BHIZ   */
        agc_dp_t zim = inc->zi[m / 3].c[m % 3]; /* DLOAD* ZI,2, ZI scale */

        for (int b = 0; b < blks; b++)
        {
            /* One element of FAZA1 (p. 1256): blk[b][m] - ZI_m OMEGAM_b,
             * with the scale restored: VXSC by the normalized -ZI_m (NORM
             * S2), VSL* by -(s + NORMGAM), VAD* the old W row.             */
            agc_vec_t wrow        = w->blk[b][m]; /* old row, scale of W */
            agc_vec_t omegam_wrow = omegam[b];    /* scale of W          */
            int       normgam     = inc->normgam; /* AGC index value     */

            if (!AGC_ASSERT(agc_navaux_dp_ok(zim) && (normgam <= NAVAUX_MAX_SHIFT) && (normgam >= -NAVAUX_MAX_SHIFT)))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            int       s  = 0;                                  /* NORM S2   */
            agc_dp_t  nz = agc_dp_norm(agc_dp_neg(zim), &s);   /* -ZI 2^s   */
            agc_vec_t d  = agc_vec_scale(ip, omegam_wrow, nz); /* W 2^-sh   */
            int       sh = -(s + normgam);                     /* LXC XAD   */

            if (sh > NAVAUX_MAX_SHIFT)
            { /* zero ZI_m only */
                sh = NAVAUX_MAX_SHIFT;
            }
            else if (sh < -NAVAUX_MAX_SHIFT)
            {
                sh = -NAVAUX_MAX_SHIFT;
            }

            d            = agc_navaux_vec_shift(ip, d, sh); /* VSL* 0,2       */
            w->blk[b][m] = agc_vec_add(ip, d, wrow);
        }
    }

    return (AGC_STATUS_OK);
} /* agc_incorp_update_w */

/**********************************************************************
 * INCORP2 phase 3, FAZC..FAZAB5 (MEASUREMENT_INCORPORATION.agc
 *   pp. 1258-1259, labels FAZC, FAZAB1, FAZAB2, FAZAB5) -- add DELTAX to
 *   the state deviations
 *     TDELTAV += DELTAX[0] SL 7 (earth) / SL 9 (moon)
 *     TNUV    += DELTAX[1] SL 4 (earth) / SL 6 (moon)
 *
 * FUNCTION NAME: agc_incorp_update_state
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description (scale)
 *   --------  --------------------  ---  ------------------------------
 *   ip        agc_interp_t *        I/O  interpreter state (OVFIND)
 *   inc       const agc_incorp_t *  I    deltax, dim9
 *   sv        agc_incorp_sv_t *     I/O  integration temporaries
 *                                        (scales in the type)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on NULL or
 *          out-of-range TDELTAV/TNUV.  On overflow of either sum (FAZAB1
 *          / FAZAB2) the correction goes into the conic RCV / VCV instead
 *          and sv->rectify asks the caller to CALL RECTIFY with PBODY = 0
 *          (earth) or 2 (moon).  X789 += DELTAX[2] if DMENFLG.
 **********************************************************************/
agc_status_t agc_incorp_update_state(agc_interp_t *ip, const agc_incorp_t *inc, agc_incorp_sv_t *sv)
{
    if (!AGC_ASSERT(agc_navaux_vec_ok(sv->tdeltav) && agc_navaux_vec_ok(sv->tnuv)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t tx789 = sv->x789; /* TX789, scale of X789 */

    tx789 = agc_vec_add(ip, tx789, inc->deltax[2]);

    agc_interp_clear_ovf(ip); /* BOVB TCDANZIG  */
    int x2 = 0;               /* MOONTHIS: 0 earth, 2 moon */

    if (sv->moon)
    {
        x2 = 2;
    }

    sv->rectify  = false;
    agc_vec_t dv = agc_navaux_vec_shift(ip, inc->deltax[0], 7 + x2); /* VSR* 0-7,2: TDELTAV scale, 2^22 / 2^18 m */

    dv = agc_vec_add(ip, dv, sv->tdeltav);

    if (agc_interp_test_ovf(ip))
    { /* BOV FAZAB1     */
        sv->rcv     = agc_vec_add(ip, sv->rcv, inc->deltax[0]);
        sv->vcv     = agc_vec_add(ip, sv->vcv, inc->deltax[1]);
        sv->rectify = true;
    }
    else
    {
        sv->tdeltav  = dv;                                               /* STOVL TDELTAV  */
        agc_vec_t nu = agc_navaux_vec_shift(ip, inc->deltax[1], 4 + x2); /* VSR* 0-4,2: TNUV scale, 2^3 / 2^-1 m/cs */

        nu = agc_vec_add(ip, nu, sv->tnuv);

        if (agc_interp_test_ovf(ip))
        { /* BOV FAZAB2     */
            sv->vcv     = agc_vec_add(ip, sv->vcv, inc->deltax[1]);
            sv->rectify = true;
        }
        else
        {
            sv->tnuv = nu; /* STCALL TNUV    */
        }
    }

    if (inc->dim9)
    { /* BOFF DMENFLG   */
        sv->x789 = tx789;
    }

    return (AGC_STATUS_OK);
} /* agc_incorp_update_state */
