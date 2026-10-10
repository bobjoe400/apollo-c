/*
 * tjetlaw.c -- TJETLAW (TJET_LAW.agc, Luminary 099 pp.1460-1469), designed
 * by R. D. Goss and P. S. Weissman: the jet-firing time for one axis
 * (AXISCTR -1 = P, 0 = U, 1 = V) from the attitude error E and error rate
 * EDOT, using the phase-plane switch curves that 1/ACCONT stores in the
 * 48-word BLOCKTOP table.
 *
 * Negative EDOT is reflected into the upper half plane (E, EDOT negated,
 * ROTSENSE negative, indexers moved by one). Small errors use FINELAW
 * (zones 1-5); errors beyond pi/16 or rates beyond pi/32 rad/s use the
 * rough laws RUFLAW1-3. The result, scaled at 4 s inside the law, is
 * multiplied by ROTSENSE (+-.390625 = 4/10.24) to give TJP/TJU/TJV in
 * TIME6 counts with the rotation sign.
 *
 * Inputs : E (at pi), EDOT (at pi/4 rad/s), AXISCTR, SENSETYP, TJx.
 * Outputs: TJx (T6 counts), NUMBERT (U,V jet policy), FIREFCT (at pi/4).
 */
#include "agc/agc_assert.h"
#include "lm_dap_internal.h"

#define TJ_SENSOR    6400    /* SENSOR 14400: 4 s / 10.24 s               */
#define TJ_M3DEG     (-1092) /* -3DEG  DEC -.06667 at 45 deg              */
#define TJ_M0112A8   (-23)   /* -.0112A8 DEC -.00141: -.01125 s^2 at 8    */
#define TJ_1AT4      410     /* .1AT4  DEC .025: 0.1 s at 4               */
#define TJ_1AT2      819     /* .1AT2  DEC .05: 0.1 s at 2                */
#define TJ_0375AT4   154     /* .0375AT4 DEC .00938                       */
#define TJ_025AT2    205     /* -(-.025AT2) DEC .0125: .025 s at 2        */
#define TJ_025AT4    102     /* -(-.025AT4) DEC .00625                    */
#define TJ_M05AT2    (-410)  /* -.05AT2 DEC -.025                         */
#define TJ_M15AT2    (-1229) /* -.15AT2 DEC -.075                         */
#define TJ_00375HI   7       /* .00375A8 2DEC .00375 B-3, major word      */
#define TJ_00375LO   11141   /*                          minor word       */
#define TJ_MTJMAX    (-614)  /* -TJMAX DEC -.0375: 150 ms at 4 s          */
#define TJ_TJMIN     82      /* TJMIN DEC .005: 20 ms at 4 s              */
#define TJ_RUFRATE   2366    /* RUFRATE DEC .1444: 6.5 deg/s at pi/4      */
#define TJ_FULLTIME  1024    /* BIT11: 250 ms at 4 s                      */
#define TJ_MINIMP    32      /* BIT6: one-jet minimum impulse (zone 3)    */
#define TJ_M1_8      (-2048) /* -1/8                                      */
#define TJ_BIT12     2048
#define TJ_BIT13     ((agc_sp_t)4096)
#define TJ_BIT14     ((agc_sp_t)8192)
#define TJ_BIT11     ((agc_sp_t)1024)
#define TJ_FLAT      6       /* FLAT = BLOCKTOP +6                        */
#define TJ_ZONE3LIM  7       /* ZONE3LIM = BLOCKTOP +7                    */

/* AXISDIFF (p.1469): BLOCKTOP offset of the axis block relative to U.    */
static const agc_sp_t axisdiff[3] = { -16, 0, 16 };

/* TJETLAW working registers (DAPTEMP1-6, DAPTREG1-4).                    */
typedef struct tjl {
    agc_sp_t adr1;            /* ADRSDIF1: axis + sign-of-EDOT indexer     */
    agc_sp_t adr2;            /* ADRSDIF2: ADRSDIF1 + max-jet / sense       */
    agc_sp_t rotsense;        /* ROTSENSE +-SENSOR                          */
    agc_sp_t edotsq;          /* EDOTSQ at pi^2/2^8                         */
    agc_sp_t ttoaxis;         /* TTOAXIS at 4 s                             */
    lm_dp_pair_t hh;          /* HH, DP at 8 s^2                            */
} tjl_t;

/* A BLOCKTOP parameter: base label offset (from 1/ANET1) + indexer.      */
static agc_sp_t blk(const lm_dap_t *d, int label, agc_sp_t idx)
{
    int i = LM_BLK_U + label + (int)idx;
    if (!AGC_ASSERT(i >= 0 && i < LM_BLOCK_N)) {
        return 0;
    }
    (void)AGC_ASSERT(d->block[i] >= LM_NEGMAX);
    return d->block[i];
}

/* CHKMINTJ (p.1467): computed times under 20 ms become zero.             */
static agc_sp_t chkmintj(int32_t a)
{
    (void)AGC_ASSERT(a >= -4 * LM_POSMAX && a <= 4 * LM_POSMAX);
    if (a - TJ_TJMIN <= 0) {
        return 0;                                         /* COASTTJ        */
    }
    agc_sp_t r = lm_dap_sat(a);
    (void)AGC_ASSERT(r > 0);
    return r;
}

/* Z123COMP (p.1465): reverse ROTSENSE, time to axis at 4 s. True if it
 * exceeds 150 ms (go to FULLTIME).                                       */
static bool z123comp(const lm_dap_t *d, tjl_t *w)
{
    w->rotsense = (agc_sp_t)-w->rotsense;
    w->ttoaxis = lm_dap_mphi(d->edot, blk(d, LM_BK_1ANET1, w->adr2));
    (void)AGC_ASSERT(w->rotsense != 0);
    (void)AGC_ASSERT(w->ttoaxis >= LM_NEGMAX);
    return (int32_t)w->ttoaxis + TJ_MTJMAX > 0;
}

/* ZONE2,3 (p.1466): fire to the axis, or a minimum impulse when close.   */
static agc_sp_t zone23(const lm_dap_t *d, const tjl_t *w)
{
    (void)AGC_ASSERT(d->block[TJ_ZONE3LIM] >= 0);
    (void)AGC_ASSERT(d->edot >= 0);
    if ((int32_t)w->ttoaxis - d->block[TJ_ZONE3LIM] > 0) {
        return w->ttoaxis;                                /* ZONE2          */
    }
    return (d->edot > 0) ? (agc_sp_t)TJ_MINIMP : 0;       /* ZONE3          */
}

/* ZONE1 (pp.1466-1467): beyond the switch curve. H = 4 (FIREFCT -
 * AXISDIST) ACCFCTZ1; TJET = H/.1 + TTOAXIS + .0375 or H/.025 + TTOAXIS. */
static agc_sp_t zone1(const lm_dap_t *d, tjl_t *w)
{
    agc_sp_t x = lm_dap_sat((int32_t)d->firefct - blk(d, LM_BK_AXISDIST, w->adr1));
    lm_dp_pair_t p = lm_dap_mp(x, blk(d, LM_BK_ACCFCTZ1, w->adr2));
    w->hh = lm_dap_ddoubl(lm_dap_ddoubl(p));
    agc_sp_t t = lm_dap_sat((int32_t)w->ttoaxis + TJ_MTJMAX);
    if ((int32_t)lm_dap_mphi(t, t) - w->hh.hi <= 0) {
        return TJ_FULLTIME;
    }
    (void)AGC_ASSERT(w->hh.hi >= LM_NEGMAX);
    if ((int32_t)w->hh.hi - 2 <= 0) {                     /* FORMULA3       */
        agc_sp_t q3 = lm_dap_dv(w->hh, TJ_025AT2);
        return chkmintj((int32_t)q3 + w->ttoaxis);
    }
    agc_sp_t q = lm_dap_dv(w->hh, TJ_1AT2);
    (void)AGC_ASSERT(q >= LM_NEGMAX);
    return lm_dap_sat((int32_t)q + w->ttoaxis + TJ_0375AT4);
}

/* ZONE5 (pp.1463-1465): between the coast and fire curves; TJET from
 * H = 4 f ACCFCTZ5 and TTOAXIS = 1/ANET2 EDOT by FORMULA1/2 or FULLTIME. */
static agc_sp_t zone5(const lm_dap_t *d, tjl_t *w, agc_sp_t f)
{
    if (!AGC_ASSERT(w->rotsense != 0)) {                  /* CCSHOLE        */
        return 0;
    }
    if (w->rotsense < 0) {
        w->adr2 = (agc_sp_t)(w->adr2 - 2);
    }
    w->hh = lm_dap_ddoubl(lm_dap_ddoubl(lm_dap_mp(f, blk(d, LM_BK_ACCFCTZ5, w->adr2))));
    w->ttoaxis = lm_dap_mphi(blk(d, LM_BK_1ANET2, w->adr2), d->edot);
    int32_t t1 = (int32_t)lm_dap_mphi(w->ttoaxis, TJ_M05AT2) + w->hh.hi - 2;
    if (t1 <= 0) {                                        /* FORMULA1       */
        agc_sp_t q1 = lm_dap_dv(w->hh, lm_dap_sat((int32_t)w->ttoaxis + TJ_025AT4));
        return chkmintj(lm_dap_mphi(q1, TJ_BIT14));
    }
    int32_t t2 = (int32_t)lm_dap_mphi(w->ttoaxis, TJ_M15AT2) + w->hh.hi + TJ_M0112A8;
    if (t2 > 0) {
        return TJ_FULLTIME;
    }
    lm_dp_pair_t k = { TJ_00375HI, TJ_00375LO };          /* FORMULA2       */
    w->hh = lm_dap_das(w->hh, k);
    agc_sp_t q2 = lm_dap_dv(w->hh, lm_dap_sat((int32_t)w->ttoaxis + TJ_1AT4));
    (void)AGC_ASSERT(q2 >= LM_NEGMAX);
    return lm_dap_mphi(q2, TJ_BIT14);
}
