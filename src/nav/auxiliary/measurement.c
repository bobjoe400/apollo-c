/*
 * measurement.c -- MEASUREMENT INCORPORATION
 * (Comanche055 MEASUREMENT_INCORPORATION.agc pp. 1252-1261; Luminary099
 * code identical apart from the bank placement of NEWZCOMP).
 *
 * The recursive (Kalman-type) navigation update with the error transition
 * matrix W (W W^T = covariance):
 *
 *   INCORP1:  Z     = W^T B                  (ZI, 9 or 6 components)
 *             A     = Z.Z + VARIANCE
 *             GAMMA = 1 / (A + sqrt(A VARIANCE))
 *             OMEGA = W Z
 *             DELTAX = OMEGA DELTAQ / A      (state correction)
 *   INCORP2:  W'    = W - GAMMA OMEGA Z^T    (row by row, restart-safe)
 *             state += DELTAX                (TDELTAV/TNUV, or RCV/VCV and
 *                                             RECTIFY on overflow), X789.
 *
 * Normalization bookkeeping uses AGC index values: NORMGAM is the AGC's
 * (negative) X-register value; a shift "VSL* 0,i" shifts left by -C(Xi).
 * The scale of W, ZI and OMEGA is whatever the W-matrix integration keeps;
 * DELTAX comes out at the state scale (position E 2^29 / M 2^27 m,
 * velocity E 2^7 / M 2^5 m/cs, bias states as X789).
 *
 * DEVIATION (restart protection): INCORP2's PHASCHNG OCT 04022 / GRP2PC
 * calls, UPFLAG REINTFLG, INTSTALL, the MOVEP* / MOVEA* state-vector copies,
 * SVDWN1/2, RECTIFY and the POSTJUMP to INTWAKE belong to the caller
 * (nav-integ / P20 measurement job).  agc_incorp2_wupdate updates W in
 * place row by row in the AGC's order; agc_incorp2_state works on the
 * integration temporaries supplied in agc_incorp_sv_t.
 */
#include <stddef.h>
#include "agc/agc_assert.h"
#include "agc/agc_navaux.h"
#include "internal.h"

/* DP1/4TH 2DEC .25 (oct 10000 00000): 1/4, the GAMMA numerator.           */
#define INCORP_DP_QUARTER AGC_DP_QUARTER
/* INCOR1B shifts VARIANCE two places at a time while it fits: at most
 * 42/2 passes of the TP.                                                  */
#define INCORP_MAX_SL2 21
/* Rows of W updated by INCORP2: 9 (DMENFLG set) or 6.                     */
#define INCORP_ROWS6 6

/* The 3x3 partition of W at AGC address W + 54b + 18a: rows blk[b][3a+r]. */
static agc_mat_t incorp_part(const agc_incorp_w_t *w, int b, int a)
{
    agc_mat_t m = agc_mat_identity_half();
    if (!AGC_ASSERT(w != NULL)) {
        return m;
    }
    if (!AGC_ASSERT((b >= 0) && (b < AGC_INCORP_BLKS) && (a >= 0) && (a < 3))) {
        return m;
    }
    for (int r = 0; r < 3; r++) {
        m.row[r] = w->blk[b][(3 * a) + r];
    }
    return m;
}

/* Z123 / INCOR1 (p. 1253): ZI[a] = sum_b MXV(W part (b,a), BVECTOR[b]);
 * ZI[2] is zeroed for a 6-dimensional incorporation.                     */
static void incorp_z(agc_interp_t *ip, const agc_incorp_w_t *w, agc_incorp_t *inc)
{
    if (!AGC_ASSERT((ip != NULL) && (w != NULL) && (inc != NULL))) {
        return;
    }
    for (int a = 0; a < 3; a++) {                       /* TIX,2 Z123      */
        agc_vec_t z = agc_vec_zero();
        for (int b = 0; b < AGC_INCORP_BLKS; b++) {
            agc_mat_t m = incorp_part(w, b, a);
            agc_vec_t t = agc_mat_vec(ip, &m, inc->bvector[b]);  /* MXV*  */
            z = (b == 0) ? t : agc_vec_add(ip, t, z);            /* VAD*  */
        }
        inc->zi[a] = z;
    }
    if (!inc->dim9) {                                   /* BON DMENFLG     */
        inc->zi[2] = agc_vec_zero();
    }
    (void)AGC_ASSERT(agc_navaux_vec_ok(inc->zi[0]));
}

/* VSQ in TP: sum of the exact component squares.  DEVIATION: the AGC's
 * DOTSUB drops the lowest partial product of each DMP; this keeps it.   */
static agc_tp_t incorp_vsq_tp(agc_interp_t *ip, agc_vec_t v)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return 0;
    }
    if (!AGC_ASSERT(agc_navaux_vec_ok(v))) {
        return 0;
    }
    agc_tp_t s = 0;
    for (int i = 0; i < AGC_VEC_N; i++) {
        s = agc_tp_add(ip, s, agc_dp_mul_tp(v.c[i], v.c[i]));
    }
    return s;
}

/* INCOR1A (p. 1253): TRIPA = Z1^2 + Z2^2 + Z3^2 + VARIANCE (TP).         */
static agc_tp_t incorp_tripa(agc_interp_t *ip, const agc_incorp_t *inc)
{
    if (!AGC_ASSERT((ip != NULL) && (inc != NULL))) {
        return 0;
    }
    if (!AGC_ASSERT(agc_navaux_tp_ok(inc->variance) && (inc->variance >= 0))) {
        return 0;
    }
    agc_tp_t a = incorp_vsq_tp(ip, inc->zi[0]);
    a = agc_tp_add(ip, a, incorp_vsq_tp(ip, inc->zi[1]));
    a = agc_tp_add(ip, a, incorp_vsq_tp(ip, inc->zi[2]));
    return agc_tp_add(ip, a, inc->variance);                /* TAD VARIANCE */
}

/*
 * INCOR1B / INCOR1C (pp. 1253-1254):
 *   TEMPVAR = VARIANCE 4^k (largest k without overflow), then
 *   GAMMA = (1/4) / NORM(TRIPA + sqrt(TRIPA TEMPVAR) 2^-k),
 *   NORMGAM = -(normalization count) - 2  (AGC index value),
 * so that the true gamma is GAMMA 2^-NORMGAM.
 */
static void incorp_gamma(agc_interp_t *ip, agc_incorp_t *inc, agc_tp_t tripa)
{
    if (!AGC_ASSERT((ip != NULL) && (inc != NULL))) {
        return;
    }
    if (!AGC_ASSERT(agc_navaux_tp_ok(tripa))) {
        return;
    }
    (void)agc_interp_test_ovf(ip);                  /* TLOAD BOV: clear  */
    agc_tp_t tempvar = inc->variance;
    int k = 0;                                      /* X2 = 0            */
    bool ovf = (tempvar == 0);                      /* BZE INCOR1C       */
    for (int i = 0; (i < INCORP_MAX_SL2) && !ovf; i++) {
        agc_tp_t t = agc_navaux_tp_shl2(tempvar, &ovf);   /* SL2 BOV     */
        if (!ovf) {
            tempvar = t;
            k++;                                    /* INCR,2 1          */
        }
    }
    agc_dp_t prod = agc_dp_mul(ip, agc_tp_round_dp(tripa),
                               agc_tp_to_dp(tempvar));  /* ROUND DMP     */
    agc_dp_t root = agc_navaux_dp_shift(ip, agc_dp_sqrt(ip, prod), -k); /* SL* 0,2 */
    agc_tp_t den = agc_tp_add(ip, agc_tp_from_dp(root), tripa);         /* TAD */
    int n = 0;
    den = agc_navaux_tp_norm(den, &n);              /* NORM X2           */
    inc->normgam = -n - 2;                          /* INCR,2 -2; SXA,2  */
    inc->gamma = agc_dp_div(ip, INCORP_DP_QUARTER, agc_tp_to_dp(den)); /* BDDV */
}

/* INCOR1C tail (p. 1254): DELTAQ/A, both normalized.  Returns the
 * quotient (scale DELTAQ/A 2^(q-m-1)); *x1 = -m + q (AGC X1 value).      */
static agc_dp_t incorp_dq_over_a(agc_interp_t *ip, const agc_incorp_t *inc,
                                 agc_tp_t tripa, int *x1)
{
    if (!AGC_ASSERT((ip != NULL) && (inc != NULL) && (x1 != NULL))) {
        return 0;
    }
    if (!AGC_ASSERT(agc_navaux_dp_ok(inc->deltaq) && (tripa > 0))) {
        *x1 = 0;
        return 0;
    }
    int m = 0;
    agc_dp_t a = agc_tp_to_dp(agc_navaux_tp_norm(tripa, &m));  /* NORM X1; DLOAD MPAC */
    int q = 0;
    agc_dp_t dq = agc_dp_norm(inc->deltaq, &q);                /* NORM S1  */
    *x1 = -m + q;                                              /* XSU,1 S1 */
    return agc_dp_div(ip, agc_dp_shr(dq, 1), a);               /* SR1 DDV  */
}

/*
 * NEWZCOMP / NEWZCMP1 (pp. 1260-1261): normalize ZI by its largest
 * component magnitude.  With p the NORM count of max |ZI[a]|, every ZI is
 * shifted left p-2, NORMGAM += 2(p-2), and the X1 for INCOR3 becomes
 * (p-2) + x1_dq.  Returns that X1.
 */
static int incorp_newzcomp(agc_interp_t *ip, agc_incorp_t *inc, int x1_dq)
{
    if (!AGC_ASSERT((ip != NULL) && (inc != NULL))) {
        return 0;
    }
    if (!AGC_ASSERT((x1_dq <= NAVAUX_MAX_SHIFT) && (x1_dq >= -NAVAUX_MAX_SHIFT))) {
        return 0;
    }
    agc_dp_t normzi = agc_vec_abval(ip, inc->zi[0]);
    for (int a = 1; a < 3; a++) {
        agc_dp_t mag = agc_vec_abval(ip, inc->zi[a]);
        if (agc_dp_sub(ip, mag, normzi) >= 0) {        /* DSU BMN         */
            normzi = mag;
        }
    }
    int p = 0;
    (void)agc_dp_norm(normzi, &p);                     /* NORM X1         */
    int shift = p - 2;                                 /* INCR,1 2        */
    for (int a = 0; a < 3; a++) {
        inc->zi[a] = agc_navaux_vec_shift(ip, inc->zi[a], shift);  /* VSL* 0,1 */
    }
    inc->normgam = inc->normgam + (2 * shift);         /* XSU,1 twice     */
    return shift + x1_dq;                              /* LXC,1 XAD,1     */
}

/* INCOR2 (p. 1254): OMEGA[b] = sum_a VXM(ZI[a], W part (b,a));
 * OMEGA[2] zeroed for a 6-dimensional incorporation.                     */
static void incorp_omega(agc_interp_t *ip, const agc_incorp_w_t *w, agc_incorp_t *inc)
{
    if (!AGC_ASSERT((ip != NULL) && (w != NULL) && (inc != NULL))) {
        return;
    }
    for (int b = 0; b < AGC_INCORP_BLKS; b++) {         /* TIX,2 INCOR2    */
        agc_vec_t om = agc_vec_zero();
        for (int a = 0; a < 3; a++) {
            agc_mat_t m = incorp_part(w, b, a);
            agc_vec_t t = agc_vec_mat(ip, inc->zi[a], &m);       /* VXM*  */
            om = (a == 0) ? t : agc_vec_add(ip, t, om);          /* VAD   */
        }
        inc->omega[b] = om;
    }
    if (!inc->dim9) {                                   /* BON DMENFLG     */
        inc->omega[2] = agc_vec_zero();
    }
    (void)AGC_ASSERT(agc_navaux_vec_ok(inc->omega[0]));
}

/* INCOR3 (p. 1255): DELTAX[b] = OMEGA[b] (DELTAQ/A) shifted left -X1;
 * the velocity triple is shifted a further VSL3.                         */
static void incorp_deltax(agc_interp_t *ip, agc_incorp_t *inc, agc_dp_t dqa, int x1)
{
    if (!AGC_ASSERT((ip != NULL) && (inc != NULL))) {
        return;
    }
    if (!AGC_ASSERT((x1 <= NAVAUX_MAX_SHIFT) && (x1 >= -NAVAUX_MAX_SHIFT))) {
        return;
    }
    for (int b = 0; b < AGC_INCORP_BLKS; b++) {         /* TIX,2 INCOR3    */
        agc_vec_t d = agc_vec_scale(ip, inc->omega[b], dqa);     /* VXSC 0 */
        inc->deltax[b] = agc_navaux_vec_shift(ip, d, -x1);       /* VSL* 0,1 */
    }
    inc->deltax[1] = agc_vec_shl(ip, inc->deltax[1], 3);         /* VSL3   */
}

/*
 * INCORP1 (MEASUREMENT_INCORPORATION.agc pp. 1252-1255, label INCORP1,
 * with NEWZCOMP pp. 1260-1261).
 * In:  w, inc->dim9 (DMENFLG), inc->bvector, inc->deltaq, inc->variance.
 * Out: inc->zi (normalized), omega, deltax, gamma, normgam.
 */
agc_status_t agc_incorp1(agc_interp_t *ip, const agc_incorp_w_t *w,
                         agc_incorp_t *inc)
{
    if (!AGC_ASSERT((ip != NULL) && (w != NULL) && (inc != NULL))) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(agc_navaux_tp_ok(inc->variance) && (inc->variance >= 0) &&
                    agc_navaux_dp_ok(inc->deltaq))) {
        return AGC_STATUS_BAD_ARG;
    }
    incorp_z(ip, w, inc);
    agc_tp_t tripa = incorp_tripa(ip, inc);
    if (!AGC_ASSERT(tripa > 0)) {          /* A = 0: no information at all */
        return AGC_STATUS_BAD_ARG;
    }
    incorp_gamma(ip, inc, tripa);
    int x1 = 0;
    agc_dp_t dqa = incorp_dq_over_a(ip, inc, tripa, &x1);
    x1 = incorp_newzcomp(ip, inc, x1);
    incorp_omega(ip, w, inc);
    incorp_deltax(ip, inc, dqa, x1);
    return AGC_STATUS_OK;
}

/* One element of FAZA1 (p. 1256): blk[b][m] - ZI_m OMEGAM_b, with the
 * scale restored: VXSC by the normalized -ZI_m (NORM S2), VSL* by
 * -(s + NORMGAM), VAD* the old W row.                                     */
static agc_vec_t incorp_wrow(agc_interp_t *ip, agc_vec_t wrow, agc_dp_t zim,
                             agc_vec_t omegam, int normgam)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return wrow;
    }
    if (!AGC_ASSERT(agc_navaux_dp_ok(zim) && (normgam <= NAVAUX_MAX_SHIFT) &&
                    (normgam >= -NAVAUX_MAX_SHIFT))) {
        return wrow;
    }
    int s = 0;
    agc_dp_t nz = agc_dp_norm(agc_dp_neg(zim), &s);      /* DCOMP NORM S2  */
    agc_vec_t d = agc_vec_scale(ip, omegam, nz);         /* VXSC OMEGAMb   */
    int sh = -(s + normgam);                             /* LXC,2 XAD,2    */
    if (sh > NAVAUX_MAX_SHIFT) {                         /* zero ZI_m only */
        sh = NAVAUX_MAX_SHIFT;
    } else if (sh < -NAVAUX_MAX_SHIFT) {
        sh = -NAVAUX_MAX_SHIFT;
    }
    d = agc_navaux_vec_shift(ip, d, sh);                 /* VSL* 0,2       */
    return agc_vec_add(ip, d, wrow);                     /* VAD* W         */
}

/*
 * INCORP2 phases 1-2, FAZA..FAZB5 (MEASUREMENT_INCORPORATION.agc
 * pp. 1256-1258): W' = W - GAMMA OMEGA Z^T, one row m (ZI component m)
 * at a time, m = 0..8 (DMENFLG set) or 0..5, blocks 0..2 or 0..1.
 * The AGC stages each row in HOLDW under restart protection; the C
 * update of a row is atomic so HOLDW is not needed.
 */
agc_status_t agc_incorp2_wupdate(agc_interp_t *ip, const agc_incorp_t *inc,
                                 agc_incorp_w_t *w)
{
    if (!AGC_ASSERT((ip != NULL) && (inc != NULL) && (w != NULL))) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT((inc->normgam <= NAVAUX_MAX_SHIFT) &&
                    (inc->normgam >= -NAVAUX_MAX_SHIFT))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_vec_t omegam[AGC_INCORP_BLKS];
    for (int b = 0; b < AGC_INCORP_BLKS; b++) {          /* OMEGAMb        */
        omegam[b] = agc_vec_scale(ip, inc->omega[b], inc->gamma);
    }
    int rows = inc->dim9 ? AGC_INCORP_ROWS : INCORP_ROWS6;   /* FAZB5     */
    int blks = inc->dim9 ? AGC_INCORP_BLKS : 2;          /* BOFF DMENFLG   */
    for (int m = 0; m < rows; m++) {                     /* TIX,1 / BHIZ   */
        agc_dp_t zim = inc->zi[m / 3].c[m % 3];          /* DLOAD* ZI,2    */
        for (int b = 0; b < blks; b++) {
            w->blk[b][m] = incorp_wrow(ip, w->blk[b][m], zim, omegam[b],
                                       inc->normgam);
        }
    }
    return AGC_STATUS_OK;
}

/*
 * INCORP2 phase 3, FAZC..FAZAB5 (MEASUREMENT_INCORPORATION.agc
 * pp. 1258-1259): add DELTAX to the state deviations.
 *   TDELTAV += DELTAX[0] SL 7 (earth) / SL 9 (moon)
 *   TNUV    += DELTAX[1] SL 4 (earth) / SL 6 (moon)
 * On overflow of either sum (FAZAB1 / FAZAB2) the correction goes into the
 * conic RCV / VCV instead and sv->rectify asks the caller to CALL RECTIFY
 * with PBODY = 0 (earth) or 2 (moon).  X789 += DELTAX[2] if DMENFLG.
 */
agc_status_t agc_incorp2_state(agc_interp_t *ip, const agc_incorp_t *inc,
                               agc_incorp_sv_t *sv)
{
    if (!AGC_ASSERT((ip != NULL) && (inc != NULL) && (sv != NULL))) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(agc_navaux_vec_ok(sv->tdeltav) && agc_navaux_vec_ok(sv->tnuv))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_vec_t tx789 = agc_vec_add(ip, sv->x789, inc->deltax[2]);   /* TX789 */
    (void)agc_interp_test_ovf(ip);                       /* BOVB TCDANZIG  */
    int x2 = sv->moon ? 2 : 0;                           /* MOONTHIS       */
    sv->rectify = false;
    agc_vec_t dv = agc_navaux_vec_shift(ip, inc->deltax[0], 7 + x2); /* VSR* 0-7,2 */
    dv = agc_vec_add(ip, dv, sv->tdeltav);
    if (agc_interp_test_ovf(ip)) {                       /* BOV FAZAB1     */
        sv->rcv = agc_vec_add(ip, sv->rcv, inc->deltax[0]);
        sv->vcv = agc_vec_add(ip, sv->vcv, inc->deltax[1]);
        sv->rectify = true;
    } else {
        sv->tdeltav = dv;                                /* STOVL TDELTAV  */
        agc_vec_t nu = agc_navaux_vec_shift(ip, inc->deltax[1], 4 + x2); /* VSR* 0-4,2 */
        nu = agc_vec_add(ip, nu, sv->tnuv);
        if (agc_interp_test_ovf(ip)) {                   /* BOV FAZAB2     */
            sv->vcv = agc_vec_add(ip, sv->vcv, inc->deltax[1]);
            sv->rectify = true;
        } else {
            sv->tnuv = nu;                               /* STCALL TNUV    */
        }
    }
    if (inc->dim9) {                                     /* BOFF DMENFLG   */
        sv->x789 = tx789;
    }
    return AGC_STATUS_OK;
}
