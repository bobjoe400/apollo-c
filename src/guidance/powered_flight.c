/**********************************************************************
 * FILE NAME: powered_flight.c
 *
 * PURPOSE:
 *   POWERED_FLIGHT_SUBROUTINES: CDUTRIG / CD*TR*GS / QUICTRIG compute the
 *   sines and cosines of three CDU angles; AX*SR*T rotates a vector
 *   between stable-member (SM) and navigation-base (NB) axes using them;
 *   SMCDURES (Comanche) converts SM increments to CDU increments;
 *   FLESHPOT (Luminary) builds the NB->SM matrix XNB.
 *
 * AGC SOURCE:
 *   Comanche055 POWERED_FLIGHT_SUBROUTINES.agc pp. 1365-1372 (CDUTRIG,
 *     CD*TR*GS, QUICTRIG, AX*SR*T, *SMNB*, *NBSM*, SMCDURES)
 *   Luminary099 POWERED_FLIGHT_SUBROUTINES.agc pp. 1259-1267 (differs:
 *     FLESHPOT / CALCSMSC instead of SMCDURES)
 *   CDULOGIC: RTB_OP_CODES.agc, Comanche p.1488
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale        Description
 *   ------------  --------  ---  -----------  ------------------------
 *   CDUX/Y/Z      uint16_t  I    2^15/rev     CDU counters (CDUTRIG)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dp_sin, _cos      SINE / COSINE (interp.h)
 *   agc_sp_sin, _cos      SPSIN / SPCOS (QUICTRIG)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   None; AGC_STATUS_BAD_ARG on a failed precondition.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   AX*SR*T is safe only for vectors of magnitude below one (as in the
 *   AGC). QUICTRIG runs under INHINT.
 *
 * NOTES:
 *   The AGC keeps the triples in the order Y Z X (CDUSPOT, SINCDU,
 *   COSCDU) because AX*SR*T rotates about the inner, middle and outer
 *   gimbal axes in that order; the C arrays keep that order
 *   (AGC_TRIG_Y/Z/X). DEVIATION: CDULOGIC's 1's-complement logic on a
 *   2's-complement count (one count for negative angles), see cdu_as_sp.
 *
 * ALGORITHM:
 *   The basic-language routines (CD*TR*GS's loop, QUICTRIG, AX*SR*T,
 *   FLESHPOT) use the raw DP operations with the AGC's own truncation
 *   (DMPSUB keeps the high two words of the TP product) and DDOUBL
 *   rescaling; SMCDURES is interpretive (DMP truncates, SL1 and DAD act
 *   on the TP MPAC).
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"

#define CDU_SIGN_BIT   ((uint16_t) 040000) /* bit 15 of the 15-bit CDU word  */
#define CDU_MASK       ((uint16_t) 077777) /* the 15 bits of a CDU word */
#define CDU_FULL_SCALE 0100000L /* 2^15 counts = one revolution    */

/* ---------------------------------------------------------------------- */
/* Helpers                                                                 */
/* ---------------------------------------------------------------------- */

/**********************************************************************
 * The CDU word as a signed SP fraction scaled at a half revolution (pi),
 * which is how SPSIN/SPCOS and the AGC's 1's-complement arithmetic see it.
 * DEVIATION: the AGC applies 1's-complement logic to a 2's-complement
 * reading, an error of one count (40 arc-seconds) for negative angles;
 * QUICTRIG accepts that error and so do we (CDU 040000 maps to -037777).
 * AGC: none (helper; the AGC uses the CDU word directly, p.1367)
 *
 * FUNCTION NAME: cdu_as_sp
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   cdu       uint16_t    I    CDU angle, 15-bit 2s-complement counts
 *
 * RETURNS: agc_sp_t -- the angle, half revolutions (scale 2^0 = pi)
 **********************************************************************/
static agc_sp_t cdu_as_sp(uint16_t cdu)
{
    uint16_t w = (uint16_t) (cdu & CDU_MASK); /* 15-bit CDU word */

    if ((w & CDU_SIGN_BIT) != 0U)
    {
        return ((agc_sp_t) ((int) w - 077777));
    }

    return ((agc_sp_t) w);
}

/**********************************************************************
 * DMP / DMPSUB in basic code: TP product, high two words kept.
 * AGC: none (helper for DMPSUB, INTERPRETER.agc p.1136, as AX*SR*T and
 * FLESHPOT call it)
 *
 * FUNCTION NAME: dmpsub
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   a         agc_dp_t    I    multiplicand, DP fraction
 *   b         agc_dp_t    I    multiplier, DP fraction
 *
 * RETURNS: agc_dp_t -- a b truncated to DP, scale of a times b
 **********************************************************************/
static agc_dp_t dmpsub(agc_dp_t a, agc_dp_t b)
{
    if (!AGC_ASSERT(a >= AGC_DP_MIN && a <= AGC_DP_MAX))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(b >= AGC_DP_MIN && b <= AGC_DP_MAX))
    {
        return (0); /* no valid result */
    }

    return (agc_tp_to_dp(agc_dp_mul_tp(a, b)));
}

/**********************************************************************
 * DDOUBL: double a DP value (overflow saturates; the AGC's callers keep
 * operands below one half so that it never overflows).
 * AGC: none (helper for the DDOUBL instruction, p.1370)
 *
 * FUNCTION NAME: ddoubl
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   a         agc_dp_t    I    DP fraction to double
 *
 * RETURNS: agc_dp_t -- 2 a (0 on overflow)
 **********************************************************************/
static agc_dp_t ddoubl(agc_dp_t a)
{
    agc_ovf_t ovf = 0;                         /* overflow of the shift */
    agc_dp_t  r   = agc_dp_shl_raw(a, 1, &ovf); /* 2 a, scale of a / 2   */

    if (!AGC_ASSERT(ovf == 0))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/**********************************************************************
 * DAS: DP add as the basic instruction does it (the callers keep the
 * sum below one). AGC: none (helper for DAS TERM1TMP, p.1370)
 *
 * FUNCTION NAME: das
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   a         agc_dp_t    I    addend, DP
 *   b         agc_dp_t    I    addend, DP
 *
 * RETURNS: agc_dp_t -- a + b (0 on overflow)
 **********************************************************************/
static agc_dp_t das(agc_dp_t a, agc_dp_t b)
{
    agc_ovf_t ovf = 0;                         /* overflow of the sum */
    agc_dp_t  r   = agc_dp_add_raw(a, b, &ovf); /* a + b, scale of a   */

    if (!AGC_ASSERT(ovf == 0))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/* ---------------------------------------------------------------------- */

/**********************************************************************
 * CDULOGIC (RTB_OP_CODES.agc, Comanche p.1488): convert the 15-bit
 * 2's-complement CDU angle (scaled in half revolutions) to a DP angle in
 * revolutions, range [-1/2, +1/2). 2^15 counts are one revolution, so the
 * DP value is the signed count times 2^13.
 *
 * FUNCTION NAME: agc_guid_cdulogic
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   cdu       uint16_t    I    CDU angle, 15-bit 2s-complement counts
 *
 * RETURNS: agc_dp_t -- the angle, rev 2^0, in [-1/2, +1/2)
 **********************************************************************/
agc_dp_t agc_guid_cdulogic(uint16_t cdu)
{
    long int w = (long int) (cdu & CDU_MASK); /* signed counts, 2^15/rev */

    if (w >= (long int) CDU_SIGN_BIT)
    {
        w -= CDU_FULL_SCALE; /* 2's complement negative angle */
    }

    return ((agc_dp_t) (w * 8192L)); /* w / 2^15 rev at scale 2^0    */
}

/**********************************************************************
 * CD*TR*GS (POWERED_FLIGHT_SUBROUTINES, CM p.1366, LM p.1260): for the
 * three 2's-complement angles at CDUSPOT (order Y Z X) store the 1's
 * complement DP angles back in CDUSPOT and their half-sines and
 * half-cosines in SINCDU and COSCDU. The AGC loops X, Z, Y (index 4, 2, 0)
 * and leaves MPAC untouched; C locals make the latter automatic.
 *
 * FUNCTION NAME: agc_guid_cdtrigs
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   t         agc_cdutrig_t *  I/O  CDU trig block (CDUSPOT, SINCDU, COSCDU
 *                                   2^1)
 *   cdu_yzx   const uint16_t   I    CDU angles in Y Z X order, counts
 **********************************************************************/
void agc_guid_cdtrigs(agc_cdutrig_t *t, const uint16_t cdu_yzx[3])
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(cdu_yzx != NULL))
    {
        return;
    }

    agc_interp_t ip; /* for SINE and COSINE */

    agc_interp_init(&ip);

    for (int k = 2; k >= 0; k--)
    { /* TRIGLOOP (TR*GL*P): index 4, 2, 0 */
        t->cduraw[k]  = cdu_as_sp(cdu_yzx[k]);
        t->cduspot[k] = agc_guid_cdulogic(cdu_yzx[k]);  /* USPRCADR CDULOGIC */
        t->coscdu[k]  = agc_dp_cos(&ip, t->cduspot[k]); /* COSINE, 2^1     */
        t->sincdu[k]  = agc_dp_sin(&ip, t->cduspot[k]); /* SINE +1, 2^1    */
    }
}

/**********************************************************************
 * CDUTRIG / CDUTRIGS (CM p.1366, LM p.1260): as CD*TR*GS but from the
 * present CDU counters, loaded CDUY, CDUZ, CDUX into CDUSPOT. CDUTEMP is
 * not touched so it stays consistent with the last PIPA reading.
 *
 * FUNCTION NAME: agc_guid_cdutrig
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   t         agc_cdutrig_t *  I/O  CDU trig block (CDUSPOT, SINCDU, COSCDU
 *                                   2^1)
 **********************************************************************/
void agc_guid_cdutrig(agc_cdutrig_t *t)
{
    if (!AGC_ASSERT(t != NULL))
    {
        return;
    }

    uint16_t cdu[3]; /* CDUY, CDUZ, CDUX counts */

    cdu[AGC_TRIG_X] = agc_erase_cdux(); /* CA CDUX  TS CDUSPOT +4       */
    cdu[AGC_TRIG_Y] = agc_erase_cduy(); /* CA CDUY  TS CDUSPOT          */
    cdu[AGC_TRIG_Z] = agc_erase_cduz(); /* CA CDUZ  TS CDUSPOT +2       */
    agc_guid_cdtrigs(t, cdu);
}

/**********************************************************************
 * QUICTRIG (CM p.1367, LM p.1261): fast single-precision version of
 * CD*TR*GS for the guidance cycle (4.1 ms vs 41 ms). SPSIN/SPCOS take the
 * 2's-complement angle scaled at pi and return sin, cos scaled at 1; the
 * MP BIT14 halves them to match the interpreter's sin/2 outputs. CDUSPOT
 * keeps the 2's-complement angles (no CDULOGIC). Runs under INHINT in the
 * AGC because the DAP shares the temporaries; C locals need no inhibit,
 * but the INHINT/RELINT pair is kept for the shared SINCDU/COSCDU.
 *
 * FUNCTION NAME: agc_guid_quictrig
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   t         agc_cdutrig_t *  I/O  CDU trig block (CDUSPOT, SINCDU, COSCDU
 *                                   2^1)
 *   cdu_yzx   const uint16_t   I    CDU angles in Y Z X order, counts
 **********************************************************************/
void agc_guid_quictrig(agc_cdutrig_t *t, const uint16_t cdu_yzx[3])
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(cdu_yzx != NULL))
    {
        return;
    }

    agc_inhint();

    for (int k = 2; k >= 0; k--)
    { /* index 4, 2, 0 (X Z Y)       */
        agc_sp_t a = cdu_as_sp(cdu_yzx[k]); /* angle, scale 2^0 = pi */

        t->cduraw[k] = a;
        t->sincdu[k] = agc_sp_mul(agc_sp_sin(a), AGC_SP_HALF); /* MP BIT14 */
        t->coscdu[k] = agc_sp_mul(agc_sp_cos(a), AGC_SP_HALF);
    }

    agc_relint();
}

/**********************************************************************
 * AX*SR*T (CM pp.1369-1371, LM pp.1262-1264): A = +3 transforms NB to SM,
 * A = -3 transforms SM to NB. The INDEXI table (4, 2, 0, 4) picks the
 * (DEX1, DEX2) pairs; DEXDEX counts 3, 2, 1 (or -3, -2, -1). Guaranteed
 * safe only for vectors of magnitude less than unity.
 *
 * FUNCTION NAME: agc_guid_axisrot
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  ----------------------------
 *   t         const agc_cdutrig_t *  I    CDU trig block (SINCDU, COSCDU
 *                                         2^1)
 *   dir       agc_axisrot_dir_t      I    rotation direction (SM to NB or
 *                                         NB to SM)
 *   v         agc_vec_t *            I/O  vector rotated in place, any
 *                                         scale |v| < 1
 **********************************************************************/
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
    const bool       nbsm      = (dir == AGC_AXISROT_NBSM); /* A = +3 */

    for (int dexdex = 3; dexdex >= 1; dexdex--)
    {                                               /* ROTLOOP, DIM DEXDEX */
        int k = nbsm ? (3 - dexdex) : (dexdex - 1); /* +3->0 ... -3->2      */

        if (!AGC_ASSERT(k >= 0 && k <= 2))
        {
            return;
        }

        /* One elementary rotation of AX*SR*T (CM pp.1370-1371, LM pp.1263-1264)
         * about the gimbal axis whose half-sine/half-cosine sit at index d1,
         * mixing vector components d2 and d1 (component index = word offset / 2):
         *   NBSM: v[d2]' = 2(v[d2] c - v[d1] s),  v[d1]' = 2(v[d1] c + v[d2] s)
         *   SMNB: v[d2]' = 2(v[d2] c + v[d1] s),  v[d1]' = 2(v[d1] c - v[d2] s)
         * The AGC forms the sine term first (TERM1TMP), then DAS adds the cosine
         * term and DDOUBL restores the scale lost by the half sine/cosine.        */
        int d1 = indexi[k];     /* DEX1: gimbal axis, component */
        int d2 = indexi[k + 1]; /* DEX2: second component      */

        agc_dp_t s    = t->sincdu[d1]; /* half sine, scale 2^1      */
        agc_dp_t c    = t->coscdu[d1]; /* half cosine, scale 2^1    */
        agc_dp_t old2 = v->c[d2];      /* VBUF(D2) before, scale of v */
        /* first pass, BUF = 1: DCS VBUF(D1) times sine                       */
        agc_dp_t term = dmpsub(agc_dp_neg(v->c[d1]), s); /* TERM1TMP, 2^-1 v */

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
} /* agc_guid_axisrot */

/**********************************************************************
 * *SMNB* (CM p.1369, LM p.1263): SM -> NB with the present SINCDU/COSCDU.
 *
 * FUNCTION NAME: agc_guid_smnb
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  ----------------------------
 *   t         const agc_cdutrig_t *  I    CDU trig block (SINCDU, COSCDU
 *                                         2^1)
 *   v         agc_vec_t *            I/O  vector rotated in place, any
 *                                         scale |v| < 1
 **********************************************************************/
void agc_guid_smnb(const agc_cdutrig_t *t, agc_vec_t *v)
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(v != NULL))
    {
        return;
    }

    agc_guid_axisrot(t, AGC_AXISROT_SMNB, v);

    return; /* C*MM*N1: CS THREE */
}

/**********************************************************************
 * *NBSM* (CM p.1369, LM p.1263): NB -> SM with the present SINCDU/COSCDU.
 *
 * FUNCTION NAME: agc_guid_nbsm
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  ----------------------------
 *   t         const agc_cdutrig_t *  I    CDU trig block (SINCDU, COSCDU
 *                                         2^1)
 *   v         agc_vec_t *            I/O  vector rotated in place, any
 *                                         scale |v| < 1
 **********************************************************************/
void agc_guid_nbsm(const agc_cdutrig_t *t, agc_vec_t *v)
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(v != NULL))
    {
        return;
    }

    agc_guid_axisrot(t, AGC_AXISROT_NBSM, v);

    return; /* C*MM*N3: CA THREE */
}

/**********************************************************************
 * CDU*SMNB (CM p.1368, LM p.1262): CDUTRIGS then SM -> NB.
 *
 * FUNCTION NAME: agc_guid_cdu_smnb
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   t         agc_cdutrig_t *  I/O  CDU trig block (CDUSPOT, SINCDU, COSCDU
 *                                   2^1)
 *   v         agc_vec_t *      I/O  vector rotated in place, any scale |v|
 *                                   < 1
 **********************************************************************/
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

/**********************************************************************
 * CDU*NBSM (CM p.1368, LM p.1262): CDUTRIGS then NB -> SM.
 *
 * FUNCTION NAME: agc_guid_cdu_nbsm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   t         agc_cdutrig_t *  I/O  CDU trig block (CDUSPOT, SINCDU, COSCDU
 *                                   2^1)
 *   v         agc_vec_t *      I/O  vector rotated in place, any scale |v|
 *                                   < 1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed)
 **********************************************************************/
agc_status_t agc_guid_cdu_nbsm(agc_cdutrig_t *t, agc_vec_t *v)
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(v != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_cdutrig(t);
    agc_guid_nbsm(t, v);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * TRG*SMNB (CM p.1368, LM p.1262): CD*TR*GS on caller's angles (Y Z X),
 * then SM -> NB.
 *
 * FUNCTION NAME: agc_guid_trg_smnb
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   t         agc_cdutrig_t *  I/O  CDU trig block (CDUSPOT, SINCDU, COSCDU
 *                                   2^1)
 *   cdu_yzx   const uint16_t   I    CDU angles in Y Z X order, counts
 *   v         agc_vec_t *      I/O  vector rotated in place, any scale |v|
 *                                   < 1
 **********************************************************************/
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

/**********************************************************************
 * TRG*NBSM (CM p.1368, LM p.1262): CD*TR*GS then NB -> SM. (Comanche's
 * NBSM interface, "the first to go", is not translated: no callers.)
 *
 * FUNCTION NAME: agc_guid_trg_nbsm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   t         agc_cdutrig_t *  I/O  CDU trig block (CDUSPOT, SINCDU, COSCDU
 *                                   2^1)
 *   cdu_yzx   const uint16_t   I    CDU angles in Y Z X order, counts
 *   v         agc_vec_t *      I/O  vector rotated in place, any scale |v|
 *                                   < 1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed)
 **********************************************************************/
agc_status_t agc_guid_trg_nbsm(agc_cdutrig_t *t, const uint16_t cdu_yzx[3],
                               agc_vec_t *v)
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(v != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_cdtrigs(t, cdu_yzx);
    agc_guid_nbsm(t, v);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * SMCDURES (Comanche p.1372): CDU increments from SM increments.
 *   DCDU(X) =  (dx cos IGA - dz sin IGA) / cos MGA
 *   DCDU(Y) =  dy - DCDU(X) sin MGA
 *   DCDU(Z) =  dx sin IGA + dz cos IGA
 * IGA = CDUY, MGA = CDUZ. DTHETASM and DCDU are in revolutions (2^0);
 * the sines and cosines are halves (2^1), hence the SL1s.
 *
 * FUNCTION NAME: agc_guid_smcdures
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  ----------------------------
 *   ip        agc_interp_t *         I/O  interpreter state (OVFIND, index
 *                                         registers)
 *   t         const agc_cdutrig_t *  I    CDU trig block (SINCDU, COSCDU
 *                                         2^1)
 *   dthetasm  agc_vec_t              I    SM attitude increments, rev 2^0
 *   dcdu      agc_vec_t *            O    CDU increments, rev 2^0
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed)
 **********************************************************************/
agc_status_t agc_guid_smcdures(agc_interp_t *ip, const agc_cdutrig_t *t,
                               agc_vec_t dthetasm, agc_vec_t *dcdu)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(t != NULL) ||
        !AGC_ASSERT(dcdu != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_dp_t sy = t->sincdu[AGC_TRIG_Y]; /* sin IGA / 2, scale 2^1 */
    const agc_dp_t cy = t->coscdu[AGC_TRIG_Y]; /* cos IGA / 2, scale 2^1 */
    const agc_dp_t sz = t->sincdu[AGC_TRIG_Z]; /* sin MGA / 2, scale 2^1 */
    const agc_dp_t cz = t->coscdu[AGC_TRIG_Z]; /* cos MGA / 2, scale 2^1 */
    /* DLOAD DMP DTHETASM COSCDUY ; PDDL DMP DTHETASM+4 SINCDUY ; BDSU ; DDV.
     * DMP keeps the third product word in MPAC+2: PDDL and DDV use the top
     * two (truncation), SL1 shifts all three, and DAD leaves MPAC+2 alone. */
    agc_dp_t pd0 = agc_dp_mul_trunc(dthetasm.c[0], cy); /* PDL 0, rev 2^1 */
    agc_dp_t a   = /* DCDU(X) numerator, rev 2^1 */
        agc_dp_sub(ip, pd0, agc_dp_mul_trunc(dthetasm.c[2], sy));

    a          = agc_dp_div(ip, a, cz);
    dcdu->c[0] = a; /* STORE DCDU     */
    a = agc_tp_to_dp(agc_tp_shift(ip, agc_dp_mul_tp(a, sz), 1)); /* DMP SL1 */
    dcdu->c[1] = agc_dp_sub(ip, dthetasm.c[1], a); /* BDSU DTHETASM+2 */
    /* DLOAD DTHETASM ; DMP PDDL SINCDUY DTHETASM+4 ; DMP DAD COSCDUY ; SL1 */
    pd0 = agc_dp_mul_trunc(dthetasm.c[0], sy);
    agc_tp_t z = /* DCDU(Z) / 2, rev 2^1 (TP) */
        agc_tp_add(ip, agc_dp_mul_tp(dthetasm.c[2], cy), agc_tp_from_dp(pd0));

    dcdu->c[2] = agc_tp_to_dp(agc_tp_shift(ip, z, 1)); /* STORE DCDU +4  */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * FLESHPOT / CALCSMSC (Luminary pp.1265-1267): the NB->SM matrix (XNB or
 * XNBPIP, half-unit rows scaled 2^1) from SINCDU/COSCDU. Rows are the NB
 * axes in SM coordinates; row 1 = row 2 x row 0 (TCF VXV), doubled.
 *
 * FUNCTION NAME: agc_guid_fleshpot
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  ----------------------------
 *   t         const agc_cdutrig_t *  I    CDU trig block (SINCDU, COSCDU
 *                                         2^1)
 *   xnb       agc_mat_t *            O    NB-to-SM matrix, half-unit rows
 **********************************************************************/
void agc_guid_fleshpot(const agc_cdutrig_t *t, agc_mat_t *xnb)
{
    if (!AGC_ASSERT(t != NULL) || !AGC_ASSERT(xnb != NULL))
    {
        return;
    }

    /* Row 0 and row 2 elements of FLESHPOT (see agc_guid_fleshpot).          */
    const agc_dp_t sx = t->sincdu[AGC_TRIG_X]; /* sin OGA / 2, scale 2^1 */
    const agc_dp_t cx = t->coscdu[AGC_TRIG_X]; /* cos OGA / 2, scale 2^1 */
    const agc_dp_t sy = t->sincdu[AGC_TRIG_Y]; /* sin IGA / 2, scale 2^1 */
    const agc_dp_t cy = t->coscdu[AGC_TRIG_Y]; /* cos IGA / 2, scale 2^1 */
    const agc_dp_t sz = t->sincdu[AGC_TRIG_Z]; /* sin MGA / 2, scale 2^1 */
    const agc_dp_t cz = t->coscdu[AGC_TRIG_Z]; /* cos MGA / 2, scale 2^1 */

    xnb->row[0].c[0] = ddoubl(dmpsub(cy, cz));                /* COSY COSZ  */
    xnb->row[0].c[1] = sz;                                    /* SINZ       */
    xnb->row[0].c[2] = ddoubl(dmpsub(agc_dp_neg(sy), cz));    /* -SINY COSZ */
    xnb->row[2].c[1] = ddoubl(dmpsub(agc_dp_neg(sx), cz));    /* -SINX COSZ */
    agc_dp_t mxz = dmpsub(agc_dp_neg(sx), sz);                /* -SX SZ, 2^2  */
    agc_dp_t t5  = ddoubl(ddoubl(dmpsub(mxz, sy)));           /* -SX SY SZ, 2^1 */
    agc_dp_t buf = ddoubl(ddoubl(dmpsub(agc_dp_neg(mxz), cy))); /* SX SZ CY, 2^1 */

    xnb->row[2].c[2] = das(t5, ddoubl(dmpsub(cy, cx))); /* CX CY - SX SY SZ */
    /* The listing's comment reads "- SINY COSX + SINX SINZ COSY" but the
     * code loads +SINCDUY (DCA, not DCS): the element is +SY CX + SX SZ CY,
     * the correct Z_NB component. The code is translated, not the comment. */
    xnb->row[2].c[0] = das(buf, ddoubl(dmpsub(sy, cx)));
    agc_interp_t ip; /* for VXV */

    agc_interp_init(&ip);
    agc_vec_t y = agc_vec_cross(&ip, xnb->row[2], xnb->row[0]); /* VXV, 2^2 */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        xnb->row[1].c[i] = ddoubl(y.c[i]); /* DDOUBL */
    }

    const bool ovf = agc_interp_test_ovf(&ip); /* VXV must not overflow */

    if (!AGC_ASSERT(!ovf))
    {
        return;
    }
} /* agc_guid_fleshpot */
