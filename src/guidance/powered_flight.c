/*
 * powered_flight.c -- POWERED_FLIGHT_SUBROUTINES.agc
 *   Comanche055 pp. 1365-1372, Luminary099 pp. 1259-1267.
 *
 * CDUTRIG / CD*TR*GS / QUICTRIG compute the sines and cosines of three CDU
 * angles; AX*SR*T rotates a vector between stable-member (SM) and
 * navigation-base (NB) axes using them; SMCDURES (Comanche) converts SM
 * increments to CDU increments; FLESHPOT (Luminary) builds the NB->SM
 * matrix XNB.
 *
 * The AGC keeps the triples in the order Y Z X (CDUSPOT, SINCDU, COSCDU)
 * because AX*SR*T rotates about the inner, middle and outer gimbal axes
 * in that order. The C arrays keep that order (AGC_TRIG_Y/Z/X).
 *
 * The basic-language routines (CD*TR*GS's loop, QUICTRIG, AX*SR*T,
 * FLESHPOT) use the raw DP operations of agc_word.h with the AGC's own
 * truncation (DMP/DMPSUB keep the high two words of the TP product) and
 * DDOUBL rescaling; SMCDURES is interpretive and uses agc_interp.h.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/guidance/guidance.h"
#include "agc/guidance/pending.h"

#define CDU_SIGN_BIT   ((uint16_t) 040000) /* bit 15 of the 15-bit CDU word  */
#define CDU_MASK       ((uint16_t) 077777)
#define CDU_FULL_SCALE 0100000L /* 2^15 counts = one revolution    */

/* ---------------------------------------------------------------------- */
/* Helpers                                                                 */
/* ---------------------------------------------------------------------- */

/* The CDU word as a signed SP fraction scaled at a half revolution (pi),
 * which is how SPSIN/SPCOS and the AGC's 1's-complement arithmetic see it.
 * DEVIATION: the AGC applies 1's-complement logic to a 2's-complement
 * reading, an error of one count (40 arc-seconds) for negative angles;
 * QUICTRIG accepts that error and so do we (CDU 040000 maps to -037777). */
static agc_sp_t cdu_as_sp(uint16_t cdu)
{
    uint16_t w = (uint16_t) (cdu & CDU_MASK);
    if ((w & CDU_SIGN_BIT) != 0U)
    {
        return (agc_sp_t) ((int) w - 077777);
    }
    return (agc_sp_t) w;
}

/* DMP / DMPSUB in basic code: TP product, high two words kept.          */
static agc_dp_t dmpsub(agc_dp_t a, agc_dp_t b)
{
    if (!AGC_ASSERT(a >= AGC_DP_MIN && a <= AGC_DP_MAX))
    {
        return 0; /* no valid result */
    }
    if (!AGC_ASSERT(b >= AGC_DP_MIN && b <= AGC_DP_MAX))
    {
        return 0; /* no valid result */
    }
    return agc_tp_to_dp(agc_dp_mul_tp(a, b));
}

/* DDOUBL: double a DP value (overflow saturates; the AGC's callers keep
 * operands below one half so that it never overflows).                    */
static agc_dp_t ddoubl(agc_dp_t a)
{
    agc_ovf_t ovf = 0;
    agc_dp_t  r   = agc_dp_shl_raw(a, 1, &ovf);
    if (!AGC_ASSERT(ovf == 0))
    {
        return 0; /* no valid result */
    }
    return r;
}

static agc_dp_t das(agc_dp_t a, agc_dp_t b)
{
    agc_ovf_t ovf = 0;
    agc_dp_t  r   = agc_dp_add_raw(a, b, &ovf);
    if (!AGC_ASSERT(ovf == 0))
    {
        return 0; /* no valid result */
    }
    return r;
}

/* ---------------------------------------------------------------------- */

/* CDULOGIC (RTB_OP_CODES.agc, Comanche p.1488): convert the 15-bit
 * 2's-complement CDU angle (scaled in half revolutions) to a DP angle in
 * revolutions, range [-1/2, +1/2). 2^15 counts are one revolution, so the
 * DP value is the signed count times 2^13.                                */
agc_dp_t agc_guid_cdulogic(uint16_t cdu)
{
    long w = (long) (cdu & CDU_MASK);
    if (w >= (long) CDU_SIGN_BIT)
    {
        w -= CDU_FULL_SCALE; /* 2's complement negative angle */
    }
    return (agc_dp_t) (w * 8192L); /* w / 2^15 rev at scale 2^0    */
}

/* CD*TR*GS (POWERED_FLIGHT_SUBROUTINES, CM p.1366, LM p.1260): for the
 * three 2's-complement angles at CDUSPOT (order Y Z X) store the 1's
 * complement DP angles back in CDUSPOT and their half-sines and
 * half-cosines in SINCDU and COSCDU. The AGC loops X, Z, Y (index 4, 2, 0)
 * and leaves MPAC untouched; C locals make the latter automatic.          */
void agc_guid_cdtrigs(agc_cdutrig_t *t, const uint16_t cdu_yzx[3])
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(cdu_yzx != NULL))
    {
        return;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    for (int k = 2; k >= 0; k--)
    { /* TRIGLOOP (TR*GL*P): index 4, 2, 0 */
        t->cduraw[k]  = cdu_as_sp(cdu_yzx[k]);
        t->cduspot[k] = agc_guid_cdulogic(cdu_yzx[k]);  /* USPRCADR CDULOGIC */
        t->coscdu[k]  = agc_dp_cos(&ip, t->cduspot[k]); /* COSINE, 2^1     */
        t->sincdu[k]  = agc_dp_sin(&ip, t->cduspot[k]); /* SINE +1, 2^1    */
    }
}

/* CDUTRIG / CDUTRIGS (CM p.1366, LM p.1260): as CD*TR*GS but from the
 * present CDU counters, loaded CDUY, CDUZ, CDUX into CDUSPOT. CDUTEMP is
 * not touched so it stays consistent with the last PIPA reading.          */
void agc_guid_cdutrig(agc_cdutrig_t *t)
{
    if (!AGC_ASSERT(t != NULL))
    {
        return;
    }
    uint16_t cdu[3];
    cdu[AGC_TRIG_X] = agc_erase_cdux(); /* CA CDUX  TS CDUSPOT +4       */
    cdu[AGC_TRIG_Y] = agc_erase_cduy(); /* CA CDUY  TS CDUSPOT          */
    cdu[AGC_TRIG_Z] = agc_erase_cduz(); /* CA CDUZ  TS CDUSPOT +2       */
    agc_guid_cdtrigs(t, cdu);
}

/* QUICTRIG (CM p.1367, LM p.1261): fast single-precision version of
 * CD*TR*GS for the guidance cycle (4.1 ms vs 41 ms). SPSIN/SPCOS take the
 * 2's-complement angle scaled at pi and return sin, cos scaled at 1; the
 * MP BIT14 halves them to match the interpreter's sin/2 outputs. CDUSPOT
 * keeps the 2's-complement angles (no CDULOGIC). Runs under INHINT in the
 * AGC because the DAP shares the temporaries; C locals need no inhibit,
 * but the INHINT/RELINT pair is kept for the shared SINCDU/COSCDU.        */
void agc_guid_quictrig(agc_cdutrig_t *t, const uint16_t cdu_yzx[3])
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(cdu_yzx != NULL))
    {
        return;
    }
    agc_inhint();
    for (int k = 2; k >= 0; k--)
    { /* index 4, 2, 0 (X Z Y)       */
        agc_sp_t a   = cdu_as_sp(cdu_yzx[k]);
        t->cduraw[k] = a;
        t->sincdu[k] = agc_sp_mul(agc_sp_sin(a), AGC_SP_HALF); /* MP BIT14 */
        t->coscdu[k] = agc_sp_mul(agc_sp_cos(a), AGC_SP_HALF);
    }
    agc_relint();
}

/* One elementary rotation of AX*SR*T (CM pp.1370-1371, LM pp.1263-1264)
 * about the gimbal axis whose half-sine/half-cosine sit at index d1,
 * mixing vector components d2 and d1 (component index = word offset / 2):
 *   NBSM: v[d2]' = 2(v[d2] c - v[d1] s),  v[d1]' = 2(v[d1] c + v[d2] s)
 *   SMNB: v[d2]' = 2(v[d2] c + v[d1] s),  v[d1]' = 2(v[d1] c - v[d2] s)
 * The AGC forms the sine term first (TERM1TMP), then DAS adds the cosine
 * term and DDOUBL restores the scale lost by the half sine/cosine.        */
static void axis_step(const agc_cdutrig_t *t, bool nbsm, int d1, int d2,
                      agc_vec_t *v)
{
    agc_dp_t s    = t->sincdu[d1];
    agc_dp_t c    = t->coscdu[d1];
    agc_dp_t old2 = v->c[d2];
    /* first pass, BUF = 1: DCS VBUF(D1) times sine                       */
    agc_dp_t term = dmpsub(agc_dp_neg(v->c[d1]), s);
    if (!nbsm)
    {
        term = agc_dp_neg(term); /* SMNB case: DCS MPAC          */
    }
    v->c[d2] = ddoubl(das(term, dmpsub(v->c[d2], c)));
    /* second pass, BUF = 0: old VBUF(D2) times sine                      */
    term = dmpsub(old2, s);
    if (!nbsm)
    {
        term = agc_dp_neg(term);
    }
    v->c[d1] = ddoubl(das(term, dmpsub(v->c[d1], c)));
}

/* AX*SR*T (CM pp.1369-1371, LM pp.1262-1264): A = +3 transforms NB to SM,
 * A = -3 transforms SM to NB. The INDEXI table (4, 2, 0, 4) picks the
 * (DEX1, DEX2) pairs; DEXDEX counts 3, 2, 1 (or -3, -2, -1). Guaranteed
 * safe only for vectors of magnitude less than unity.                    */
void agc_guid_axisrot(const agc_cdutrig_t *t, agc_axisrot_dir_t dir,
                              agc_vec_t *v)
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(v != NULL))
    {
        return;
    }
    if (!AGC_ASSERT(dir == AGC_AXISROT_NBSM || dir == AGC_AXISROT_SMNB))
    {
        return;
    }
    /* INDEXI pairs in DP word offsets 4,2 / 2,0 / 0,4, halved to indices. */
    static const int indexi[4] = {2, 1, 0, 2};
    const bool       nbsm      = (dir == AGC_AXISROT_NBSM);
    for (int dexdex = 3; dexdex >= 1; dexdex--)
    {                                               /* ROTLOOP, DIM DEXDEX */
        int k = nbsm ? (3 - dexdex) : (dexdex - 1); /* +3->0 ... -3->2      */
        if (!AGC_ASSERT(k >= 0 && k <= 2))
        {
            return;
        }
        axis_step(t, nbsm, indexi[k], indexi[k + 1], v);
    }
}

/* *SMNB* (CM p.1369, LM p.1263): SM -> NB with the present SINCDU/COSCDU. */
void agc_guid_smnb(const agc_cdutrig_t *t, agc_vec_t *v)
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(v != NULL))
    {
        return;
    }
    agc_guid_axisrot(t, AGC_AXISROT_SMNB, v);
    return; /* C*MM*N1: CS THREE */
}

/* *NBSM* (CM p.1369, LM p.1263): NB -> SM with the present SINCDU/COSCDU. */
void agc_guid_nbsm(const agc_cdutrig_t *t, agc_vec_t *v)
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(v != NULL))
    {
        return;
    }
    agc_guid_axisrot(t, AGC_AXISROT_NBSM, v);
    return; /* C*MM*N3: CA THREE */
}

/* CDU*SMNB (CM p.1368, LM p.1262): CDUTRIGS then SM -> NB.                */
void agc_guid_cdu_smnb(agc_cdutrig_t *t, agc_vec_t *v)
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(v != NULL))
    {
        return;
    }
    agc_guid_cdutrig(t);
    agc_guid_smnb(t, v);
    return;
}

/* CDU*NBSM (CM p.1368, LM p.1262): CDUTRIGS then NB -> SM.                */
agc_status_t agc_guid_cdu_nbsm(agc_cdutrig_t *t, agc_vec_t *v)
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(v != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_cdutrig(t);
    agc_guid_nbsm(t, v);
    return AGC_STATUS_OK;
}

/* TRG*SMNB (CM p.1368, LM p.1262): CD*TR*GS on caller's angles (Y Z X),
 * then SM -> NB.                                                          */
void agc_guid_trg_smnb(agc_cdutrig_t *t, const uint16_t cdu_yzx[3],
                               agc_vec_t *v)
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(v != NULL))
    {
        return;
    }
    agc_guid_cdtrigs(t, cdu_yzx);
    agc_guid_smnb(t, v);
    return;
}

/* TRG*NBSM (CM p.1368, LM p.1262): CD*TR*GS then NB -> SM. (Comanche's
 * NBSM interface, "the first to go", is not translated: no callers.)     */
agc_status_t agc_guid_trg_nbsm(agc_cdutrig_t *t, const uint16_t cdu_yzx[3],
                               agc_vec_t *v)
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(v != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_cdtrigs(t, cdu_yzx);
    agc_guid_nbsm(t, v);
    return AGC_STATUS_OK;
}

/* SMCDURES (Comanche p.1372): CDU increments from SM increments.
 *   DCDU(X) =  (dx cos IGA - dz sin IGA) / cos MGA
 *   DCDU(Y) =  dy - DCDU(X) sin MGA
 *   DCDU(Z) =  dx sin IGA + dz cos IGA
 * IGA = CDUY, MGA = CDUZ. DTHETASM and DCDU are in revolutions (2^0);
 * the sines and cosines are halves (2^1), hence the SL1s.                */
agc_status_t agc_guid_smcdures(agc_interp_t *ip, const agc_cdutrig_t *t,
                               agc_vec_t dthetasm, agc_vec_t *dcdu)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(t != NULL) ||
        !AGC_ASSERT(dcdu != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_dp_t sy = t->sincdu[AGC_TRIG_Y], cy = t->coscdu[AGC_TRIG_Y];
    const agc_dp_t sz = t->sincdu[AGC_TRIG_Z], cz = t->coscdu[AGC_TRIG_Z];
    /* DLOAD DMP DTHETASM COSCDUY ; PDDL DMP DTHETASM+4 SINCDUY ; BDSU ; DDV */
    agc_dp_t pd0 = agc_dp_mul(ip, dthetasm.c[0], cy);
    agc_dp_t a   = agc_dp_sub(ip, pd0, agc_dp_mul(ip, dthetasm.c[2], sy));
    a            = agc_dp_div(ip, a, cz);
    dcdu->c[0]   = a;                                      /* STORE DCDU     */
    a          = agc_dp_shl(ip, agc_dp_mul(ip, a, sz), 1); /* DMP SL1 SINCDUZ */
    dcdu->c[1] = agc_dp_sub(ip, dthetasm.c[1], a);         /* BDSU DTHETASM+2 */
    /* DLOAD DTHETASM ; DMP PDDL SINCDUY DTHETASM+4 ; DMP DAD COSCDUY ; SL1 */
    pd0        = agc_dp_mul(ip, dthetasm.c[0], sy);
    a          = agc_dp_add(ip, agc_dp_mul(ip, dthetasm.c[2], cy), pd0);
    dcdu->c[2] = agc_dp_shl(ip, a, 1); /* STORE DCDU +4  */
    return AGC_STATUS_OK;
}

/* Row 0 and row 2 elements of FLESHPOT (see agc_guid_fleshpot).          */
static void fleshpot_rows02(const agc_cdutrig_t *t, agc_mat_t *m)
{
    const agc_dp_t sx = t->sincdu[AGC_TRIG_X], cx = t->coscdu[AGC_TRIG_X];
    const agc_dp_t sy = t->sincdu[AGC_TRIG_Y], cy = t->coscdu[AGC_TRIG_Y];
    const agc_dp_t sz = t->sincdu[AGC_TRIG_Z], cz = t->coscdu[AGC_TRIG_Z];
    m->row[0].c[0] = ddoubl(dmpsub(cy, cz));                    /* COSY COSZ  */
    m->row[0].c[1] = sz;                                        /* SINZ       */
    m->row[0].c[2] = ddoubl(dmpsub(agc_dp_neg(sy), cz));        /* -SINY COSZ */
    m->row[2].c[1] = ddoubl(dmpsub(agc_dp_neg(sx), cz));        /* -SINX COSZ */
    agc_dp_t mxz   = dmpsub(agc_dp_neg(sx), sz);                /* -SX SZ /4  */
    agc_dp_t t5    = ddoubl(ddoubl(dmpsub(mxz, sy)));           /* -SX SY SZ  */
    agc_dp_t buf = ddoubl(ddoubl(dmpsub(agc_dp_neg(mxz), cy))); /* SX SZ CY  */
    m->row[2].c[2] = das(t5, ddoubl(dmpsub(cy, cx))); /* CX CY - SX SY SZ */
    /* The listing's comment reads "- SINY COSX + SINX SINZ COSY" but the
     * code loads +SINCDUY (DCA, not DCS): the element is +SY CX + SX SZ CY,
     * the correct Z_NB component. The code is translated, not the comment. */
    m->row[2].c[0] = das(buf, ddoubl(dmpsub(sy, cx)));
}

/* FLESHPOT / CALCSMSC (Luminary pp.1265-1267): the NB->SM matrix (XNB or
 * XNBPIP, half-unit rows scaled 2^1) from SINCDU/COSCDU. Rows are the NB
 * axes in SM coordinates; row 1 = row 2 x row 0 (TCF VXV), doubled.       */
void agc_guid_fleshpot(const agc_cdutrig_t *t, agc_mat_t *xnb)
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(xnb != NULL))
    {
        return;
    }
    fleshpot_rows02(t, xnb);
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_vec_t y = agc_vec_cross(&ip, xnb->row[2], xnb->row[0]); /* VXV   */
    for (int i = 0; i < AGC_VEC_N; i++)
    {
        xnb->row[1].c[i] = ddoubl(y.c[i]); /* DDOUBL */
    }
    const bool ovf = agc_interp_test_ovf(&ip); /* VXV must not overflow */
    if (!AGC_ASSERT(!ovf))
    {
        return;
    }
}
