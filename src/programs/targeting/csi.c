/**********************************************************************
 * FILE NAME: csi.c
 *
 * PURPOSE:
 *   Concentric flight plan targeting, both vehicles: CSI/A .. CSINEXT4,
 *   SCNDSOL, ALMXIT (the CSI iteration), ADVANCE, ROTATE, INTINT2C,
 *   INTINT3P, CDHMVR, S32/33.X and the matrix part of S32/33.1. The
 *   push-down list of the AGC is kept in named locals (the AGC cell is
 *   given in the comment). Address-less push-down operations follow the
 *   AGC interpreter: a load with no address pops; PDDL/PDVL with no
 *   address exchanges MPAC with the top of the list.
 *
 * AGC SOURCE:
 *   Comanche055 P32-P33_P72-P73.agc pp. 663-682 (CSI/A, CSI/B .. CSI/B3,
 *   NTP/2, CSINEXT1 .. CSINEXT4, K2., FRSTPAS, FIFTYFPS, NEWTN, CSISTEP,
 *   CSI/SOL, SCNDSOL, ALMXIT, ADVANCE, ROTATE, S32/33.X, CDHMVR)
 *   Luminary099 P32-P35_P72-P75.agc (same text; equal constants under
 *   other names: TMIN = 600SEC, TWOPI = 2PISC SL2, 1DPB2 = 1RTEB2,
 *   INITST = INITST1, P30ZERO = ZEROVEC)
 *
 * EXTERNAL VARIABLES:
 *   Name        Type       I/O  Scale      Description
 *   ----------  ---------  ---  ---------  ---------------------------
 *   TIG TTPI    agc_dp_t   I    cs 2^28    CSI and TPI times
 *   XDELVFLG    flag       O    -          set by ADVANCE
 *   CMOONFLG    flag       I    -          moon sphere (scale shifts)
 *   ELEV NN TCSI (agc_rendz_t) I           CSI inputs (N55, N11)
 *   RACT1..3 VACT1..4 RPASS1..3 VPASS1..3 UP1 (agc_rendz_t)
 *               I/O  m 2^29, m/cs 2^7      the trial states
 *   DELVCSI DELDV DVPREV GAMPREV LOOPCT CSIALRM HAFPA1 TCDH (agc_rendz_t)
 *               I/O  m/cs 2^7, cs 2^28     iteration state
 *   DELVEET1 DELVEET2 DIFFALT POSTCSI POSTCDH T1TOT2 T2TOT3 DELVSIN
 *               O    m/cs 2^7, m 2^29, cs 2^28  results
 *
 * EXTERNAL REFERENCES:
 *   Name                          Description
 *   ----------------------------  ------------------------------------
 *   rz_precset rz_intint          PRECSET / INTINT state updates
 *   rz_timethet_rv                TIMETHET: transfer time for an angle
 *   agc_rendz_periapo_body_scale  PERIAPO1: apsides
 *   agc_rendz_scalar_from_body    SHIFTR1: body scale to earth scale
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   agc_rendz_csi_a returns AGC_STATUS_ALARM with *alarm = 00600 +
 *   CSIALRM - 1 (00600 .. 00606, ALMXIT); the caller displays it.
 *   AGC_STATUS_NO_CONVERGE after CSI_MAX_PASSES label steps (DEVIATION:
 *   the AGC has no bound; LOOPMX limits it in practice).
 *   AGC_STATUS_BAD_ARG for NULL pointers or a failed invariant;
 *   integration statuses (INTSTALL sleep) are passed back.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs inside the P32/P72 job. The AGC branches inside CSI/A are kept
 *   as a label state machine (csi_label_t).
 *
 * NOTES:
 *   Scales: r m 2^29, v m/cs 2^7, t cs 2^28, angles revolutions, unit
 *   vectors 2^1, RTMU 2^36. "Body scale" = 2^27 / 2^5 about the moon.
 *   F13 (CDH DIFFALT 62 m vs the AGC) is a GETX word-form effect in
 *   nav-conic: history/lanes/review-prog-rendz.md P1.
 **********************************************************************/
#include "../navigation/internal.h"
#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"
#include "internal.h"

/* ---- constants (P32-P33 p.663, values as 2^28 fractions) --------------- */
#define CSI_INITST     ((agc_dp_t) 63921)    /* .03048 B-7: 10 fps            */
#define CSI_DVMAX1     ((agc_dp_t) 6392119)  /* 3.0480 B-7: 1000 fps          */
#define CSI_DVMAX2     ((agc_dp_t) 6321806)  /* 3.014472 B-7: 989 fps         */
#define CSI_1DPB2      ((agc_dp_t) 67108864) /* 1.0 B-2 (CM 1RTEB2 / 1DPB2)   */
#define CSI_1DPB28     ((agc_dp_t) 1)        /* 2DEC 1: one count at B28      */
#define CSI_LOOPMX     ((agc_dp_t) 16)       /* LOOPMX 2DEC 16                */
#define CSI_EPSILN1    ((agc_dp_t) 639)      /* .0003048 B-7: .1 fps          */
#define CSI_FIFPSDP    ((agc_dp_t) -319606)  /* -.1524 B-7: 50 fps            */
#define CSI_DELMAX1    ((agc_dp_t) 1278424)  /* .6096 B-7: 200 fps            */
#define CSI_PMINE      ((agc_dp_t) 78710)    /* 157420 B-29: 84 NM            */
#define CSI_PMINM      ((agc_dp_t) 5334)     /* 10668 B-29: 35000 ft          */
#define CSI_NICKELDP   ((agc_dp_t) 44745)    /* .021336 B-7: 7 fps            */
#define CSI_ONETHTH    ((agc_dp_t) 3355)     /* .0001 B-3                     */
#define CSI_DP1_4      AGC_DP_QUARTER        /* DP1/4 (LM DP1/4TH)            */
#define CSI_MAX_PASSES 200                   /* bound on CSI/B..CSI/B1 passes */

/* CSI labels: the targets of the AGC branches inside CSI/A. */
typedef enum csi_label
{
    CSI_L_B = 0,   /* CSI/B: new cycle, first DELVCSI estimate          */
    CSI_L_B1,      /* CSI/B1: count the iteration                       */
    CSI_L_B2,      /* CSI/B2: limit DELVCSI, then CSI/B23 trial         */
    CSI_L_SCNDSOL, /* SCNDSOL: second cycle or alarm                    */
    CSI_L_ALMXITA, /* ALMXITA: alarm with X2                            */
    CSI_L_ALMXIT,  /* ALMXIT: alarm with the stored CSIALRM             */
    CSI_L_DONE,    /* CSINEXT4 BPL P32/P72C: solution found             */
    CSI_L_ERROR    /* a called routine failed / INTSTALL sleep          */
} csi_label_t;

/* Working cells of one CSI/A call (push list, index registers). */
typedef struct csi_work
{
    agc_dp_t     gamma;  /* 0D GAMMA, revolutions 2^1                     */
    agc_dp_t     slope;  /* 4D (GAM-GAMPREV)/(DV-DVPREV), 2^(-6+N1)       */
    int          x2;     /* X2 = alarm index for SCNDSOL / ALMXITA         */
    agc_job_id_t resume; /* INTSTALL continuation                         */
    agc_status_t st;     /* status when CSI_L_ERROR                       */
} csi_work_t;

/**********************************************************************
 * CSI/A first lines and SCNDSOL: F1 = 0, F2 = 1, F3A F3B = 0 1 (first
 * cycle) or 0 0 (second cycle).
 * AGC: CSI/A first lines, SCNDSOL (P32-P33_P72-P73.agc pp.663, 674)
 *
 * FUNCTION NAME: csi_set_cycle
 *
 * ARGUMENTS:
 *   Argument     Type  I/O  Description (scale)
 *   -----------  ----  ---  ---------------------------------------
 *   first_cycle  bool  I    true: first cycle (F3B = 1), false: second
 **********************************************************************/
static void csi_set_cycle(bool first_cycle)
{
    rz_vflag_set(RZ_VF_CSI_F1, false);
    rz_vflag_set(RZ_VF_CSI_F2, true);
    rz_vflag_set(RZ_VF_CSI_F3A, false);
    rz_vflag_set(RZ_VF_CSI_F3B, first_cycle);
}

/**********************************************************************
 * ROTATE (p.676): project v into the passive orbit plane, keep |v|
 * AGC: ROTATE (P32-P33_P72-P73.agc p.676)
 *
 * FUNCTION NAME: agc_rendz_rotate
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   up1       agc_vec_t       I    UP1, passive orbit normal, unit 2^1
 *   v         agc_vec_t       I    position (m 2^29) or velocity (m/cs 2^7)
 *                                  to rotate
 *   out       agc_vec_t *     O    V in the passive plane, |V| kept, V's
 *                                  scale
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a NULL
 *          pointer
 **********************************************************************/
static agc_status_t agc_rendz_rotate(agc_interp_t *ip, agc_vec_t up1, agc_vec_t v, agc_vec_t *out)
{
    /* PUSH PUSH; DOT VXSC UP1 UP1; VSL2 BVSU; UNIT PDVL; ABVAL VXSC; VSL1 */
    agc_vec_t proj; /* V's scale x 2^2 */

    proj = agc_vec_scale(ip, up1, agc_vec_dot(ip, v, up1));
    agc_vec_t inplane; /* V's scale */

    inplane       = agc_vec_sub(ip, v, agc_vec_shl(ip, proj, 2));
    agc_vec_t u   = rz_unit(ip, inplane); /* unit, 2^1 */
    agc_dp_t  mag = rz_abval(ip, v);      /* |V|, V's scale */

    *out = agc_vec_shl(ip, agc_vec_scale(ip, u, mag), 1);

    if (!AGC_ASSERT(mag >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * ADVANCE (p.675)
 * TDEC1 = TIG; PRECSET; XDELVFLG set; passive state at TIG into the
 * 1, 2 and 3 cells; UP1 = unit(unit(RP) x VP); active state at TIG
 * (RTIG, VTIG) rotated into the passive plane as RACT1/2, VACT1/2.
 * AGC: ADVANCE (P32-P33_P72-P73.agc p.675)
 *
 * FUNCTION NAME: agc_rendz_advance
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the PRECSET status (INTSTALL sleep)
 **********************************************************************/
agc_status_t agc_rendz_advance(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = rz_precset(ip, rz, agc_erase_tig(), resume); /* PRECSET */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_set(AGC_FLAG_XDELVFLG);
    rz->vpass2 = rz->vpass3;
    rz->vpass1 = rz->vpass3;
    rz->rpass2 = rz->rpass3;
    rz->rpass1 = rz->rpass3;
    rz->up1    = rz_unit(ip, agc_vec_cross(ip, rz_unit(ip, rz->rpass1), rz->vpass1));
    rz->rtig   = rz->ract3;
    st         = agc_rendz_rotate(ip, rz->up1, rz->ract3, &rz->ract2);
    rz->ract1  = rz->ract2;
    rz->vtig   = rz->vact3;

    if (st == AGC_STATUS_OK)
    {
        st = agc_rendz_rotate(ip, rz->up1, rz->vact3, &rz->vact2);
    }

    rz->vact1 = rz->vact2;

    return (st);
}

/**********************************************************************
 * CSI/B1 (p.666): LOOPCT + 1; 16 passes -> SCNDSOL with X2 = 6
 * AGC: CSI/B1 (P32-P33_P72-P73.agc p.666)
 *
 * FUNCTION NAME: csi_count_pass
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   rz        agc_rendz_t *  I/O  rendezvous erasables
 *   w         csi_work_t *   O    X2 = 6 when LOOPMX is reached
 *
 * RETURNS: csi_label_t -- CSI_L_B2, or CSI_L_SCNDSOL after 16 passes
 **********************************************************************/
static csi_label_t csi_count_pass(agc_rendz_t *rz, csi_work_t *w)
{
    rz->loopct = (agc_sp_t) (rz->loopct + 1); /* + 1DPB28   */

    if ((agc_dp_t) rz->loopct - CSI_LOOPMX >= 0)
    {
        w->x2 = 6;

        return (CSI_L_SCNDSOL);
    }

    return (CSI_L_B2);
}

/**********************************************************************
 * CSI/B2, CSI/B22 (p.666): limit |DELVCSI| to 989 fps once
 * Returns false (SCNDSOL, X2 = 7) when the limit was already applied or
 * in the 50 fps stage of the second cycle.
 * AGC: CSI/B2, CSI/B22 (P32-P33_P72-P73.agc p.666)
 *
 * FUNCTION NAME: csi_limit_delta_v
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   w         csi_work_t *    O    X2 = 7 for the SCNDSOL exit
 *
 * RETURNS: bool -- true: go on to CSI/B23; false: SCNDSOL
 **********************************************************************/
static bool csi_limit_delta_v(agc_interp_t *ip, agc_rendz_t *rz, csi_work_t *w)
{
    if (agc_dp_sub(ip, agc_dp_abs(rz->delvcsi), CSI_DVMAX1) >= 0) /* CSI/B23    */
    {
        w->x2 = 7;

        if (rz_vflag_test(RZ_VF_CSI_F1))
        {
            return (false);
        }

        if (rz_vflag_test(RZ_VF_CSI_F3A) && rz_vflag_test(RZ_VF_CSI_F3B))
        {
            return (false);
        }

        rz_vflag_set(RZ_VF_CSI_F1, true); /* CSI/B22    */
        rz->delvcsi = agc_dp_sign(CSI_DVMAX2, rz->delvcsi);
    }

    return (true);
}

/**********************************************************************
 * VECSHIFT + TIMETHET (time only) of the post-CSI state (RACT1, v).
 * AGC: VECSHIFT, TIMETHET (CONIC_SUBROUTINES.agc p.1284)
 *
 * FUNCTION NAME: csi_timethet
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   v         agc_vec_t       I    velocity at RACT1, m/cs 2^7
 *   snth      agc_dp_t        I    SNTH, sine of the transfer angle, 2^1
 *   csth      agc_dp_t        I    CSTH, cosine of the transfer angle, 2^1
 *   t         agc_dp_t *      O    transfer time, cs 2^28
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the TIMETHET status
 **********************************************************************/
static agc_status_t csi_timethet(agc_interp_t *ip, agc_rendz_t *rz, agc_vec_t v, agc_dp_t snth, agc_dp_t csth, agc_dp_t *t)
{
    agc_conic_time_out_t o;  /* TIMETHET result */
    agc_status_t         st; /* TIMETHET */

    st = rz_timethet_rv(ip, rz, rz->ract1, v, snth, csth, true, &o);

    *t = o.t;

    return (st);
}

/**********************************************************************
 * CSI/B23 .. CSI/B23D (p.667): DELVEET1 = DELVCSI UH1, VACT4, HAFPA1 =
 * half period (TIMETHET through 359.99 deg), POSTCSI; *ellipse is true
 * when the post-CSI orbit is not to be treated as circular.
 * AGC: CSI/B23 .. CSI/B23D (P32-P33_P72-P73.agc p.667)
 *
 * FUNCTION NAME: csi_post_csi_orbit
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ip        agc_interp_t *         I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *          I/O  rendezvous erasables
 *   pa        agc_rendz_periapo_t *  O    PERIAPO1 results of the post-CSI
 *                                         orbit
 *   ellipse   bool *                 O    true: the post-CSI orbit is not
 *                                         taken as circular
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the TIMETHET or PERIAPO1
 *          status
 **********************************************************************/
static agc_status_t csi_post_csi_orbit(agc_interp_t *ip, agc_rendz_t *rz, agc_rendz_periapo_t *pa, bool *ellipse)
{
    agc_vec_t uh1; /* UH1, 2^1 */

    uh1 = rz_unit(ip, agc_vec_cross(ip, rz->up1, rz_unit(ip, rz->ract1)));

    rz->delveet1 = agc_vec_shl(ip, agc_vec_scale(ip, uh1, rz->delvcsi), 1);
    rz->vact4    = agc_vec_add(ip, rz->delveet1, rz->vact1);

    if (agc_interp_test_ovf(ip))
    {
        /* BOV CSI/B23D: the branch target is the next line; the test
         * only resets the overflow indicator.                          */
    }

    agc_dp_t     t = 0; /* time, cs 2^28 */
    agc_status_t st;    /* 359.99 deg */

    st = csi_timethet(ip, rz, rz->vact4, RZ_SN359, RZ_CS359, &t);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->hafpa1 = agc_dp_shr(t, 1); /* SR1        */
    agc_rendz_periapo_body_scale(ip, rz, rz->ract1, rz->vact4, pa);

    rz->postcsi = agc_rendz_scalar_from_body(rz, pa->hperi);
    *ellipse    = (rz->centang == 0) && (agc_dp_sub(ip, pa->ecc, CSI_ONETHTH) >= 0);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * Non-circular post-CSI orbit (p.668-669): RDOTV test (7 fps), then the
 * time T from RACT1 to the apsis by TIMETHET of the reversed state;
 * *pop = the value DSU'd at NTP/2 (T or HAFPA1 - T).
 * AGC: CSI/B3 (P32-P33_P72-P73.agc pp.668-669)
 *
 * FUNCTION NAME: csi_apsis
 *
 * ARGUMENTS:
 *   Argument  Type                         I/O  Description (scale)
 *   --------  ---------------------------  ---  -------------------
 *   ip        agc_interp_t *               I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *                I/O  rendezvous erasables
 *   pa        const agc_rendz_periapo_t *  I    PERIAPO1 results (R1, P)
 *   circl     bool *                       O    true: RDOTV under 7 fps,
 *                                               treated as circular (CIRCL)
 *   pop       agc_dp_t *                   O    value subtracted at NTP/2
 *                                               (T or HAFPA1 - T), cs 2^28
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG for a failed
 *          invariant, else the TIMETHET status
 **********************************************************************/
static agc_status_t csi_apsis(agc_interp_t *ip, agc_rendz_t *rz, const agc_rendz_periapo_t *pa, bool *circl, agc_dp_t *pop)
{
    int      x1 = 0; /* NORM index, -(left shifts) */
    agc_dp_t r1n;    /* 2D, 2^(29+X1) */

    r1n          = rz_norm(agc_rendz_scalar_from_body(rz, pa->r1), &x1);
    agc_dp_t rdv = agc_vec_dot(ip, rz->ract1, rz->vact4); /* R.V, 2^36 */
    agc_dp_t q   = agc_dp_div(ip, agc_dp_abs(rdv), r1n);  /* B7         */

    q      = agc_dp_sub(ip, rz_sl_star(ip, q, 0, x1), CSI_NICKELDP);
    *circl = (q < 0);

    if (*circl)
    {
        return (AGC_STATUS_OK);
    }

    agc_dp_t d14; /* 14D, 2^2 */

    d14 = agc_dp_sub(ip, agc_dp_shl(ip, pa->p, 2), CSI_1DPB2);

    /* (P/MU)^.5 / R1 part of the apsis-angle computation (p.668), B-31 (moon
     * SL3 more).                                                             */
    agc_dp_t a; /* 2D, B-31 */

    a = agc_dp_div(ip, agc_dp_shr(rz->body.rtsr1mu, 1), r1n);
    agc_dp_t s; /* P R1, earth scale */

    s = agc_rendz_scalar_from_body(rz, agc_dp_mul_trunc(pa->p, pa->r1));

    s = agc_dp_shl(ip, agc_dp_shl(ip, s, 4), 1); /* SL4 SL1    */
    s = agc_dp_mul_trunc(agc_dp_sqrt(s), a);     /* SQRT DMP   */

    if (agc_flag_test(AGC_FLAG_CMOONFLG))
    {
        s = agc_dp_shl(ip, s, 3); /* SL3        */
    }

    rz->rdotv   = agc_vec_dot(ip, rz->ract1, rz->vact4); /* CSI/B3     */
    int      x2 = 0;                                     /* NORM index, -(left shifts) */
    agc_dp_t d12;                                        /* 12D, scale by SL* */

    d12 = agc_dp_mul_trunc(rz_norm(agc_dp_abs(rz->rdotv), &x2), s);

    x1           = x1 - x2;                                /* XSU,1 X2   */
    d12          = rz_sl_star(ip, d12, 3, x1);
    agc_vec_t sc = rz_unit(ip, agc_vec_make(d12, d14, 0)); /* SNTH CSTH, 2^1 */

    /* SIGN on a vector: complement v when s is negative. */
    agc_vec_t v_vsign = rz->vact4; /* VACT4, m/cs 2^7 */
    agc_dp_t  s_vsign = rz->rdotv; /* RDOTV, 2^36 */

    agc_vec_t v = agc_vec_neg(v_vsign); /* SIGN VCOMP, m/cs 2^7 */

    if (s_vsign < 0)
    {
        v = v_vsign; /* SIGN complements, VCOMP restores */
    }

    agc_dp_t     t = 0; /* time, cs 2^28 */
    agc_status_t st;    /* TIMETHET */

    st = csi_timethet(ip, rz, v, sc.c[0], sc.c[1], &t);

    if (st == AGC_STATUS_OK)
    {
        *pop = t; /* RDOTV >= 0: T */

        if (rz->rdotv < 0)
        {
            *pop = agc_dp_sub(ip, rz->hafpa1, t); /* HAFPA1 - T */
        }
    }

    return (st);
} /* csi_apsis */

/**********************************************************************
 * NTP/2 (p.669): TCDH = NN HAFPA1 - pop + TCSI; TTPI - TCDH < 0 ->
 * SCNDSOL with X2 = 5 (returns false).
 * AGC: NTP/2 (P32-P33_P72-P73.agc p.669)
 *
 * FUNCTION NAME: csi_cdh_time
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   pop       agc_dp_t        I    apsis time term, cs 2^28
 *   w         csi_work_t *    O    X2 = 5 for the SCNDSOL exit
 *
 * RETURNS: bool -- true when TTPI - TCDH >= 0; false: SCNDSOL
 **********************************************************************/
static bool csi_cdh_time(agc_interp_t *ip, agc_rendz_t *rz, agc_dp_t pop, csi_work_t *w)
{
    agc_dp_t nn = agc_dp_from_sp(rz->nn); /* NN, B14 */
    agc_dp_t t;                           /* cs 2^28 */

    t = agc_dp_shl(ip, agc_dp_mul_trunc(nn, rz->hafpa1), 14);

    t        = agc_dp_add(ip, agc_dp_sub(ip, t, pop), rz->tcsi);
    rz->tcdh = t;
    w->x2    = 5;

    return (agc_dp_sub(ip, agc_erase_ttpi(), rz->tcdh) >= 0);
}

/**********************************************************************
 * CSINEXT1 .. CSINEXT2 head (p.669-670): states at CDH (conic), CDHMVR,
 * POSTCDH, active state at TTPI (conic) into RACT3/VACT3.
 * AGC: CSINEXT1 .. CSINEXT2 (P32-P33_P72-P73.agc pp.669-670)
 *
 * FUNCTION NAME: csi_cdh_tpi
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the integration, CDHMVR or
 *          PERIAPO1 status
 **********************************************************************/
static agc_status_t csi_cdh_tpi(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume)
{
    agc_vec_t    r;                                                                                      /* position, m 2^29 */
    agc_vec_t    v;                                                                                      /* velocity, m/cs 2^7 */
    agc_status_t st = rz_intint(ip, rz, rz->ract1, rz->vact4, rz->tcsi, rz->tcdh, true, resume, &r, &v); /* to CDH */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->ract2 = r;
    rz->vact2 = v;
    st        = rz_intint(ip, rz, rz->rpass1, rz->vpass1, rz->tcsi, rz->tcdh, true, resume, &r, &v);

    if (st == AGC_STATUS_OK)
    {
        rz->rpass2 = r;
        rz->vpass2 = v;
        st         = agc_rendz_cdhmvr(ip, rz);
        agc_rendz_periapo_t pa; /* PERIAPO1 results */

        if (st == AGC_STATUS_OK)
        {
            agc_rendz_periapo_body_scale(ip, rz, rz->ract2, rz->vact3, &pa);
            rz->postcdh = agc_rendz_scalar_from_body(rz, pa.hperi);
            st          = rz_intint(ip, rz, rz->ract2, rz->vact3, rz->tcdh, agc_erase_ttpi(), true, resume, &r, &v);

            if (st == AGC_STATUS_OK)
            {
                rz_store_active(rz, r, v); /* ACTIVE     */
            }
        }
    }

    return (st);
} /* csi_cdh_tpi */

/**********************************************************************
 * CSINEXT2 (p.670-671): TEMP1 = U.RA3, TEMP2 = TEMP1^2 - RA3^2 + RP3^2
 * (TP). Negative TEMP2 (no intersection of the LOS with the passive
 * radius): LOOPCT = 1 -> ALMXITA (X2 = 1), else halve DELDV and retry.
 * Otherwise *k = the root of smaller magnitude, K1 or K2.
 * AGC: CSINEXT2 (P32-P33_P72-P73.agc pp.670-671)
 *
 * FUNCTION NAME: csi_k
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   u         agc_vec_t       I    desired LOS at TPI, unit 2^1
 *   k         agc_dp_t *      O    K, the root of smaller magnitude, m 2^29
 *   w         csi_work_t *    O    X2 = 1 for the ALMXITA exit
 *
 * RETURNS: csi_label_t -- CSI_L_DONE with *k set, CSI_L_ALMXITA, or
 *          CSI_L_B1 (DELDV halved)
 **********************************************************************/
static csi_label_t csi_k(agc_interp_t *ip, agc_rendz_t *rz, agc_vec_t u, agc_dp_t *k, csi_work_t *w)
{
    agc_dp_t temp1; /* 6D, m 2^29 */

    temp1       = agc_dp_shl(ip, agc_vec_dot(ip, u, rz->ract3), 1);
    agc_tp_t t2 = agc_dp_sq_tp(temp1);                       /* 8D, 2^58 */
    agc_tp_t ra = -agc_vec_dot_tp(ip, rz->ract3, rz->ract3); /* 11D, 2^58 */
    agc_tp_t rp;                                             /* RP3^2, 2^58 */

    rp = agc_vec_dot_tp(ip, rz->rpass3, rz->rpass3);
    agc_tp_t temp2; /* TEMP2, 2^58 */

    temp2 = agc_tp_add(ip, agc_tp_add(ip, rp, ra), t2);

    if (temp2 < 0)
    {
        w->x2 = 1;

        if ((agc_dp_t) rz->loopct - CSI_1DPB28 == 0)
        {
            return (CSI_L_ALMXITA);
        }

        rz->deldv   = agc_dp_shr(rz->deldv, 1);
        rz->delvcsi = agc_dp_sub(ip, rz->dvprev, rz->deldv);

        return (CSI_L_B1);
    }

    agc_dp_t temp3 = agc_tp_sqrt(temp2);                       /* K10RK2 B29 */
    agc_dp_t k2    = agc_dp_sub(ip, agc_dp_neg(temp3), temp1); /* 10D, m 2^29 */
    agc_dp_t k1    = agc_dp_sub(ip, temp3, temp1);             /* 12D, m 2^29 */
    agc_dp_t d;                                                /* m 2^29 */

    d = agc_dp_sub(ip, agc_dp_abs(k2), agc_dp_abs(k1));

    *k = (d < 0) ? k2 : k1; /* smaller root */

    return (CSI_L_DONE);
}

/**********************************************************************
 * K2. (p.671): GAMMA = SIGN(TEMP) ARCCOS(UNIT(RA3 + K U) . URP3), B1.
 * AGC: K2. (P32-P33_P72-P73.agc p.671)
 *
 * FUNCTION NAME: csi_gamma
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   ip        agc_interp_t *       I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *  I    rendezvous erasables
 *   u         agc_vec_t            I    desired LOS at TPI, unit 2^1
 *   k         agc_dp_t             I    K, m 2^29
 *
 * RETURNS: agc_dp_t -- GAMMA, revolutions 2^1 (0 when the invariant fails)
 **********************************************************************/
static agc_dp_t csi_gamma(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t u, agc_dp_t k)
{
    agc_vec_t ku; /* K U, m 2^29 */

    ku             = agc_vec_shl(ip, agc_vec_scale(ip, u, k), 1);
    agc_vec_t uv   = rz_unit(ip, agc_vec_add(ip, ku, rz->ract3)); /* 0D, 2^1 */
    agc_vec_t urp3 = rz_unit(ip, rz->rpass3);                     /* 6D, 2^1 */
    agc_vec_t uvp3 = rz_unit(ip, rz->vpass3);                     /* UNIT VP3, 2^1 */
    agc_vec_t a    = agc_vec_cross(ip, uvp3, urp3);               /* 12D, 2^2 */
    agc_dp_t  temp;                                               /* sign, 2^4 */

    temp = agc_vec_dot(ip, agc_vec_cross(ip, urp3, uv), a);
    agc_dp_t c; /* cos, 2^1 */

    c          = agc_dp_shl(ip, agc_vec_dot(ip, urp3, uv), 1);
    agc_dp_t g = agc_dp_sign(agc_dp_acos(ip, c), temp); /* rev 2^0 */

    return (agc_dp_shr(g, 1)); /* SR1 */
}

/**********************************************************************
 * FRSTPAS (p.672): GAMPREV = GAMMA, DVPREV = DELVCSI, step by DELDV.
 * AGC: FRSTPAS, CSINEXT3 (P32-P33_P72-P73.agc p.672)
 *
 * FUNCTION NAME: csi_frstpas
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  ----------------------------
 *   ip        agc_interp_t *      I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *       I/O  rendezvous erasables
 *   w         const csi_work_t *  I    GAMMA
 *
 * RETURNS: csi_label_t -- CSI_L_B1, or CSI_L_ALMXIT when the F2 invariant
 *          fails
 **********************************************************************/
static csi_label_t csi_frstpas(agc_interp_t *ip, agc_rendz_t *rz, const csi_work_t *w)
{
    rz->gamprev = w->gamma;
    rz->dvprev  = rz->delvcsi;
    rz->delvcsi = agc_dp_sub(ip, rz->delvcsi, rz->deldv); /* CSINEXT3   */
    rz_vflag_set(RZ_VF_CSI_F2, false);

    return (CSI_L_B1);
}

/**********************************************************************
 * FIFTYFPS (p.673): DELDV = 50 fps signed by slope and GAMPREV; next
 * stage "11"; back to CSI/B2 without counting.
 * AGC: FIFTYFPS (P32-P33_P72-P73.agc p.673)
 *
 * FUNCTION NAME: csi_fiftyfps
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  ----------------------------
 *   ip        agc_interp_t *      I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *       I/O  rendezvous erasables
 *   w         const csi_work_t *  I    GAMMA and the slope
 *
 * RETURNS: csi_label_t -- CSI_L_B2
 **********************************************************************/
static csi_label_t csi_fiftyfps(agc_interp_t *ip, agc_rendz_t *rz, const csi_work_t *w)
{
    agc_dp_t d; /* m/cs 2^7 */

    d = agc_dp_sign(agc_dp_sign(CSI_FIFPSDP, w->slope), rz->gamprev);

    rz->deldv   = d;
    rz->delvcsi = agc_dp_add(ip, agc_dp_neg(d), rz->delvcsi);
    rz_vflag_set(RZ_VF_CSI_F3B, true);
    rz_vflag_set(RZ_VF_CSI_F3A, true);
    rz->gamprev = w->gamma;

    return (CSI_L_B2);
}

/**********************************************************************
 * NEWTN, CSISTEP (p.673): Newton step DELDV = GAMMA / slope, limited to
 * 200 fps; converged (< .1 fps) -> CSI/SOL (returned as CSI_L_DONE).
 * AGC: NEWTN, CSISTEP (P32-P33_P72-P73.agc p.673)
 *
 * FUNCTION NAME: csi_newton
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  ----------------------------
 *   ip        agc_interp_t *      I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *       I/O  rendezvous erasables
 *   w         const csi_work_t *  I    GAMMA and the slope
 *   x1        int                 I    NORM shift count of DELVCSI - DVPREV
 *
 * RETURNS: csi_label_t -- CSI_L_DONE (converged: CSI/SOL) or CSI_L_B1
 **********************************************************************/
static csi_label_t csi_newton(agc_interp_t *ip, agc_rendz_t *rz, const csi_work_t *w, int x1)
{
    int      x2 = 0;                            /* NORM index, -(left shifts) */
    agc_dp_t sn = rz_norm(w->slope, &x2);       /* slope, 2^(-6+N1+X2) */
    agc_dp_t d  = agc_dp_div(ip, w->gamma, sn); /* BDDV 00D, 2^7 after SR* */

    x1          = x1 - x2;               /* XSU,1 X2   */
    rz->deldv   = rz_sr_star(ip, d, 0, x1);
    rz->gamprev = w->gamma;
    agc_dp_t ad = agc_dp_abs(rz->deldv); /* m/cs 2^7 */

    if (agc_dp_sub(ip, ad, CSI_EPSILN1) < 0)
    {
        return (CSI_L_DONE); /* CSI/SOL    */
    }

    if (agc_dp_sub(ip, ad, CSI_DELMAX1) >= 0)
    {
        rz->deldv = agc_dp_sign(CSI_DELMAX1, rz->deldv);
    }

    rz->delvcsi = agc_dp_sub(ip, rz->delvcsi, rz->deldv); /* CSISTEP    */

    return (CSI_L_B1);
}

/**********************************************************************
 * Gamma bookkeeping after K2. (p.671-673): first pass of a cycle, the
 * slope, the cycle stage logic (F3A F3B), Newton iteration.
 * AGC: K2. .. THRDCHK (P32-P33_P72-P73.agc pp.671-673)
 *
 * FUNCTION NAME: csi_iterate
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   w         csi_work_t *    I/O  GAMMA in; the slope out
 *
 * RETURNS: csi_label_t -- the next label (CSI_L_B1, CSI_L_B2, CSI_L_DONE or
 *          CSI_L_ALMXIT)
 **********************************************************************/
static csi_label_t csi_iterate(agc_interp_t *ip, agc_rendz_t *rz, csi_work_t *w)
{
    if (rz_vflag_test(RZ_VF_CSI_F2))
    {
        return (csi_frstpas(ip, rz, w));
    }

    agc_dp_t dg = agc_dp_sub(ip, w->gamma, rz->gamprev); /* 2D, rev 2^1 */
    int      x1 = 0;                                     /* NORM index, -(left shifts) */
    agc_dp_t ddv;                                        /* 2^(7+X1) */

    ddv = rz_norm(agc_dp_sub(ip, rz->delvcsi, rz->dvprev), &x1);

    w->slope   = agc_dp_div(ip, dg, ddv);      /* 4D         */
    rz->dvprev = rz->delvcsi;
    bool f3a   = rz_vflag_test(RZ_VF_CSI_F3A); /* stage flag F3A */
    bool f3b   = rz_vflag_test(RZ_VF_CSI_F3B); /* stage flag F3B */

    if (f3a && f3b)
    { /* "11": end of the 50 fps stage */
        if (agc_dp_mul_trunc(dg, rz->gamprev) >= 0)
        {
            return (csi_fiftyfps(ip, rz, w));
        }

        rz->deldv = agc_dp_sign(CSI_INITST, rz->deldv); /* INITST1   */
        rz_vflag_set(RZ_VF_CSI_F3A, true);
        rz_vflag_set(RZ_VF_CSI_F3B, false);

        return (csi_frstpas(ip, rz, w));
    }

    if (!f3a && !f3b)
    { /* THRDCHK "00" -> FIFTYFPS    */
        return (csi_fiftyfps(ip, rz, w));
    }

    return (csi_newton(ip, rz, w, x1)); /* THRDCHK "10" or "01"        */
}

/**********************************************************************
 * CSI/SOL, CSINEXT4 (p.674): perigee and spacing tests of the solution.
 * Returns CSI_L_DONE (go to P32/P72C) or CSI_L_SCNDSOL with X2 set.
 * AGC: CSI/SOL, CSINEXT4 (P32-P33_P72-P73.agc p.674)
 *
 * FUNCTION NAME: csi_sol
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   w         csi_work_t *    O    X2 = 2 .. 5 for the SCNDSOL exit
 *
 * RETURNS: csi_label_t -- CSI_L_DONE (P32/P72C) or CSI_L_SCNDSOL
 **********************************************************************/
static csi_label_t csi_sol(agc_interp_t *ip, agc_rendz_t *rz, csi_work_t *w)
{
    agc_dp_t pmin = (rz->body.rtx1 == -10) ? CSI_PMINM : CSI_PMINE; /* moon? m 2^29 */

    w->x2 = 2;

    if (agc_dp_sub(ip, rz->postcsi, pmin) >= 0)
    {
        w->x2 = 3;

        if (agc_dp_sub(ip, rz->postcdh, pmin) >= 0)
        {
            rz->t1tot2 = agc_dp_sub(ip, rz->tcdh, rz->tcsi);
            w->x2      = 4;

            if (agc_dp_sub(ip, rz->t1tot2, RZ_600SEC) >= 0)
            {
                w->x2      = 5;
                rz->t2tot3 = agc_dp_sub(ip, agc_erase_ttpi(), rz->tcdh);

                if (agc_dp_sub(ip, rz->t2tot3, RZ_600SEC) >= 0)
                {
                    return (CSI_L_DONE);
                }
            }
        }
    }

    return (CSI_L_SCNDSOL);
}

/**********************************************************************
 * SCNDSOL (p.674): in the first cycle ("01") start the second cycle at
 * CSI/B with CSIALRM = X2; otherwise ALMXIT.
 * AGC: SCNDSOL (P32-P33_P72-P73.agc p.674)
 *
 * FUNCTION NAME: csi_scndsol
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  ----------------------------
 *   rz        agc_rendz_t *       I/O  rendezvous erasables
 *   w         const csi_work_t *  I    X2, the alarm index of the failed
 *                                      test
 *
 * RETURNS: csi_label_t -- CSI_L_B (second cycle) or CSI_L_ALMXIT
 **********************************************************************/
static csi_label_t csi_scndsol(agc_rendz_t *rz, const csi_work_t *w)
{
    if (rz_vflag_test(RZ_VF_CSI_F3A) || !rz_vflag_test(RZ_VF_CSI_F3B))
    {
        return (CSI_L_ALMXIT);
    }

    rz->csialrm = (agc_sp_t) w->x2;
    csi_set_cycle(false);
    rz->loopct  = 0;

    return (CSI_L_B);
}

/**********************************************************************
 * One trial at the current DELVCSI: CSI/B23 .. K2. and the gamma logic.
 * AGC: CSI/B23 .. K2. (P32-P33_P72-P73.agc pp.667-673)
 *
 * FUNCTION NAME: csi_trial
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   w         csi_work_t *    I/O  iteration cells; st set for CSI_L_ERROR
 *
 * RETURNS: csi_label_t -- the next label of the CSI/A state machine
 **********************************************************************/
static csi_label_t csi_trial(agc_interp_t *ip, agc_rendz_t *rz, csi_work_t *w)
{
    agc_rendz_periapo_t pa;              /* PERIAPO1 results */
    bool                ellipse = false; /* not circular */

    w->st          = csi_post_csi_orbit(ip, rz, &pa, &ellipse);
    bool     circl = !ellipse; /* CIRCL path */
    agc_dp_t pop   = 0;        /* CIRCL: 0D, cs 2^28 */

    if (w->st == AGC_STATUS_OK && ellipse)
    {
        w->st = csi_apsis(ip, rz, &pa, &circl, &pop);

        if (circl)
        {
            pop = 0;
        }
    }

    if (w->st != AGC_STATUS_OK)
    {
        return (CSI_L_ERROR);
    }

    if (!csi_cdh_time(ip, rz, pop, w))
    {
        return (CSI_L_SCNDSOL);
    }

    w->st = csi_cdh_tpi(ip, rz, w->resume);

    if (w->st != AGC_STATUS_OK)
    {
        return (CSI_L_ERROR);
    }

    /* U = sin(E) URA3 + cos(E) UH3 (B1): the desired line of sight at TPI. */
    agc_dp_t  sine = agc_dp_sin(rz->elev);   /* 6D, 2^1 */
    agc_vec_t ura3 = rz_unit(ip, rz->ract3); /* 0D, 2^1 */
    agc_vec_t uh3;                           /* UH3, 2^1 */

    uh3 = rz_unit(ip, agc_vec_cross(ip, rz->up1, ura3));
    agc_vec_t c; /* 18D, 2^2 */

    c = agc_vec_scale(ip, uh3, agc_dp_cos(rz->elev));
    agc_vec_t u_unit; /* 2^2 */

    u_unit        = agc_vec_add(ip, agc_vec_scale(ip, ura3, sine), c);
    agc_vec_t   u = agc_vec_shl(ip, u_unit, 1); /* U, 2^1 */
    agc_dp_t    k = 0;                          /* K, m 2^29 */
    csi_label_t l = csi_k(ip, rz, u, &k, w);    /* next label */

    if (l != CSI_L_DONE)
    {
        return (l);
    }

    w->gamma = csi_gamma(ip, rz, u, k);
    l        = csi_iterate(ip, rz, w);

    return ((l == CSI_L_DONE) ? csi_sol(ip, rz, w) : l); /* solution: tests */
} /* csi_trial */

/**********************************************************************
 * CSI/A (p.663-674)
 * Iterates DELVCSI so that the CDH and TPI conditions are met. OK: the
 * solution (DELVEET1, DELVCSI, TCDH, T1TOT2, T2TOT3, POSTCSI, POSTCDH)
 * is set and the program continues at P32/P72C. ALARM: *alarm is the
 * octal code 00600 + CSIALRM - 1 for ALMXIT.
 * AGC: CSI/A (P32-P33_P72-P73.agc pp.663-674)
 *
 * FUNCTION NAME: agc_rendz_csi_a
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   alarm     agc_sp_t *      O    alarm code 00600 .. 00606 for ALMXIT,
 *                                  else 0
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (solution), AGC_STATUS_ALARM
 *          (ALMXIT), AGC_STATUS_NO_CONVERGE (pass bound),
 *          AGC_STATUS_BAD_ARG, else the integration status
 **********************************************************************/
agc_status_t agc_rendz_csi_a(agc_interp_t *ip, agc_rendz_t *rz, agc_sp_t *alarm, agc_job_id_t resume)
{
    csi_work_t w = {0, 0, 0, resume, AGC_STATUS_OK};

    csi_set_cycle(true);
    rz->loopct    = 0;
    rz->csialrm   = 0;
    *alarm        = 0;
    csi_label_t l = CSI_L_B; /* next label */

    for (int pass = 0; pass < CSI_MAX_PASSES; pass++)
    {
        switch (l)
        {
            case CSI_L_B:
                /* CSI/B (p.665): first estimate of the CSI horizontal delta V
                 * DELVCSI = sqrt(2 MU / ((1 + RA1/RP3) RA1)) - VA1 . UH1, DELDV = 10 fps.
                 * (Velocity at RA1 of an ellipse with apocenter RP3, minus the present
                 * horizontal velocity.)
                 * AGC: CSI/B (P32-P33_P72-P73.agc p.665) */
            {
                agc_dp_t ra1  = rz_abval(ip, rz->ract1);                /* 0D B29     */
                int      x2   = 0;                                      /* NORM index, -(left shifts) */
                agc_dp_t d2   = agc_dp_shr(rz_norm(ra1, &x2), 1);       /* 2D, 2^(30+X2) */
                int      x1   = 0;                                      /* NORM index, -(left shifts) */
                agc_dp_t rp3n = rz_norm(rz_abval(ip, rz->rpass3), &x1); /* 2^(29+X1) */
                agc_dp_t q    = agc_dp_div(ip, d2, rp3n);               /* BDDV, B2 after SR* */

                x2 = x2 - x1;                                             /* XSU,2 X1   */
                q  = rz_sr_star(ip, q, 1, x2);                            /* RA1/RP3 B2 */
                q  = agc_dp_mul_trunc(agc_dp_add(ip, q, CSI_1DPB2), ra1); /* B31        */
                q  = rz_norm(q, &x1);
                agc_dp_t vr;                                              /* 2MU/, B7 below */

                vr = agc_dp_div(ip, agc_dp_shr(rz->body.rtmu, 1), q);

                vr = agc_dp_sqrt(rz_sl_star(ip, vr, -7, x1)); /* B7  0D     */
                agc_vec_t uh1;                                /* UH1, 2^1 */

                uh1 = rz_unit(ip, agc_vec_cross(ip, rz->up1, rz_unit(ip, rz->ract1)));
                agc_dp_t vh; /* m/cs 2^7 */

                vh = agc_dp_shl(ip, agc_vec_dot(ip, uh1, rz->vact1), 1);

                rz->delvcsi = agc_dp_sub(ip, vr, vh); /* BDSU       */
                rz->deldv   = CSI_INITST;
            }
                l = CSI_L_B1;
                break;
            case CSI_L_B1:
                l = csi_count_pass(rz, &w);
                break;
            case CSI_L_B2:
                l = CSI_L_SCNDSOL; /* limit already applied: X2 = 7 */

                if (csi_limit_delta_v(ip, rz, &w))
                {
                    l = csi_trial(ip, rz, &w);
                }

                break;
            case CSI_L_SCNDSOL:
                l           = csi_scndsol(rz, &w);
                break;
            case CSI_L_ALMXITA:
                rz->csialrm = (agc_sp_t) w.x2;
                l           = CSI_L_ALMXIT;
                break;
            case CSI_L_ALMXIT:
                *alarm      = (agc_sp_t) (AGC_ALARM_CSI_00600 + rz->csialrm - 1);

                return (AGC_STATUS_ALARM);
            case CSI_L_DONE:
                return (AGC_STATUS_OK);
            case CSI_L_ERROR:
            default:

                if (w.st == AGC_STATUS_OK)
                {
                    w.st = AGC_STATUS_BAD_ARG; /* an error label, never OK */
                }

                return (w.st);
        } /* end switch (l) */
    } /* for (int pass = 0; pass < CSI_MAX_PASSES; pass++) */

    return (AGC_STATUS_NO_CONVERGE); /* DEVIATION: the AGC has no pass bound */
} /* agc_rendz_csi_a */

/**********************************************************************
 * CDHMVR head: the passive state advanced to the radial line of RACT2 by
 * TIMETHET; UNVEC = UNIT(RACT2).
 * AGC: CDHMVR (P32-P33_P72-P73.agc p.680)
 *
 * FUNCTION NAME: cdh_passive
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ------------------------
 *   ip        agc_interp_t *          I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *           I/O  rendezvous erasables
 *   o         agc_conic_time_out_t *  O    TIMETHET result (RNEW, VNEW, R1,
 *                                          R1A)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG for a failed
 *          invariant, else the TIMETHET status
 **********************************************************************/
static agc_status_t cdh_passive(agc_interp_t *ip, agc_rendz_t *rz, agc_conic_time_out_t *o)
{
    rz->unvec = rz_unit(ip, rz->ract2); /* UNVEC      */
    agc_dp_t c;                         /* 6D, 2^2 */

    c             = agc_vec_dot(ip, rz_unit(ip, rz->rpass2), rz->unvec);
    agc_dp_t csth = agc_dp_shl(ip, c, 1); /* CSTH, 2^1 */
    agc_dp_t s2;                          /* 2^4 */

    s2           = agc_dp_sub(ip, agc_dp_shr(CSI_DP1_4, 2), agc_dp_sq(c));
    agc_dp_t  sn = agc_dp_shl(ip, agc_dp_sqrt(s2), 1); /* 0D, 2^1 */
    agc_vec_t n;                                       /* 2^58 */

    n = agc_vec_cross(ip, agc_vec_neg(rz->ract2), rz->rpass2);
    agc_dp_t snth; /* SNTH, 2^1 */

    snth = agc_dp_sign(sn, agc_vec_dot(ip, n, rz->up1));
    agc_status_t st; /* TIMETHET */

    st = rz_timethet_rv(ip, rz, rz->rpass2, rz->vpass2, snth, csth, false, o);

    return (st);
}

/**********************************************************************
 * Constant delta height maneuver at CDH: passive advanced to the radial
 * line of RACT2 (TIMETHET), DIFFALT, the passive semimajor axis, then the
 * active velocity VACT3 giving the same apsidal geometry; DELVEET2.
 * AGC: CDHMVR (P32-P33_P72-P73.agc pp.680-682)
 *
 * FUNCTION NAME: agc_rendz_cdhmvr
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the TIMETHET status
 **********************************************************************/
agc_status_t agc_rendz_cdhmvr(agc_interp_t *ip, agc_rendz_t *rz)
{
    agc_conic_time_out_t o;                            /* TIMETHET result */
    agc_status_t         st = cdh_passive(ip, rz, &o); /* passive at CDH */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    int       x2 = rz->body.rtx2;              /* X2 = RTX2 */
    agc_vec_t vp = rz_vshift(ip, o.vnew, -x2); /* m/cs 2^7 */
    agc_dp_t  vpv;                             /* m/cs 2^7 */

    vpv         = agc_dp_shl_round(ip, agc_vec_dot(ip, vp, rz->unvec), 1);
    agc_dp_t rp = rz_shift(ip, rz_abval(ip, o.rnew), -x2); /* m 2^29 */
    agc_dp_t ra = rz_abval(ip, rz->ract2);                 /* 2D, m 2^29 */

    rz->diffalt   = agc_dp_sub(ip, rp, ra);
    int      x1   = 0;                   /* NORM index, -(left shifts) */
    agc_dp_t r1an = rz_norm(o.r1a, &x1); /* 4D, 2^(6+X1) */
    agc_dp_t r1;                         /* m 2^30 */

    r1 = agc_dp_shr_round(agc_rendz_scalar_from_body(rz, o.r1), 1);
    agc_dp_t a_p; /* m 2^29 */

    a_p          = rz_sl_star(ip, agc_dp_div(ip, r1, r1an), -5, x1);
    agc_dp_t a_a = agc_dp_sub(ip, a_p, rz->diffalt); /* 4D A SUB A, m 2^29 */
    agc_dp_t vav = 0;                                /* vertical, m/cs 2^7 */
    agc_dp_t vah = 0;                                /* horizontal, m/cs 2^7 */

    /* Vertical (B7) and horizontal (B7) velocity of the active vehicle. */
    agc_dp_t q = agc_dp_div(ip, agc_dp_shr(a_p, 2), a_a); /* 6D  B2     */
    agc_dp_t p = agc_dp_mul(ip, agc_dp_sqrt(q), q);       /* DMPR, 2^3 */

    vav                = agc_dp_shl_round(ip, agc_dp_mul_trunc(p, vpv), 3); /* 8D B7     */
    int      x1_speeds = 0;                                                 /* NORM index of RA, -(shifts) */
    agc_dp_t rn        = rz_norm(ra, &x1_speeds);                           /* 2^(29+X1) */
    agc_dp_t t2mu;                                                          /* B14 after SL* */

    t2mu = agc_dp_div(ip, agc_dp_shr(rz->body.rtmu, 1), rn);

    t2mu               = rz_sl_star(ip, t2mu, -5, x1_speeds); /* 10D B14    */
    int      x2_speeds = 0;                                   /* NORM index of A, -(shifts) */
    agc_dp_t an        = rz_norm(a_a, &x2_speeds);            /* 2^(29+X2) */
    agc_dp_t mua;                                             /* B14 after SL* */

    mua = agc_dp_div(ip, agc_dp_shr(rz->body.rtmu, 1), an);

    mua        = rz_sl_star(ip, mua, -6, x2_speeds);
    agc_dp_t e = agc_dp_sub(ip, t2mu, mua); /* BDSU, B14 */

    vah         = agc_dp_sqrt(agc_dp_sub(ip, e, agc_dp_sq(vav)));
    agc_vec_t h = agc_vec_scale(ip, rz_unit(ip, agc_vec_cross(ip, rz->up1, rz->unvec)), vah); /* 2^8 */
    agc_vec_t v;                                                                              /* 2^8 */

    v = agc_vec_add(ip, agc_vec_scale(ip, rz->unvec, vav), h);

    rz->vact3    = agc_vec_shl(ip, v, 1);
    rz->delveet2 = agc_vec_sub(ip, rz->vact3, rz->vact2);

    return (AGC_STATUS_OK);
} /* agc_rendz_cdhmvr */

/**********************************************************************
 * S32/33.X (p.679): rows -URA1 x UP1 (VSL1), -UP1, -URA1
 * AGC: S32/33.X (P32-P33_P72-P73.agc p.679)
 *
 * FUNCTION NAME: agc_rendz_csi_cdh_lv_matrix
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   ip        agc_interp_t *       I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *  I    rendezvous erasables
 *   m         agc_mat_t *          O    rows -URA1 x UP1, -UP1, -URA1, unit
 *                                       2^1
 *
 * RETURNS: none
 **********************************************************************/
void agc_rendz_csi_cdh_lv_matrix(agc_interp_t *ip, const agc_rendz_t *rz, agc_mat_t *m)
{
    m->row[1] = agc_vec_neg(rz->up1);                /* 6D         */
    m->row[2] = agc_vec_neg(rz_unit(ip, rz->ract1)); /* 12D        */
    m->row[0] = agc_vec_shl(ip, agc_vec_cross(ip, m->row[2], rz->up1), 1);
}

/**********************************************************************
 * S32/33.1 tail (p.678): DELVSIN = M^T DELVLVC (VXM, VSL1), DELVSAB.
 * The AGC returns DELVSIN in MPAC; the program stores it as DELVEET1/2.
 * AGC: S32/33.1 (P32-P33_P72-P73.agc p.678)
 *
 * FUNCTION NAME: agc_rendz_csi_cdh_dv_to_reference
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   delvsin   agc_vec_t *     O    DELVSIN, delta V in reference axes, m/cs
 *                                  2^7
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a NULL
 *          pointer
 **********************************************************************/
agc_status_t agc_rendz_csi_cdh_dv_to_reference(agc_interp_t *ip,
                                               agc_rendz_t  *rz,
                                               agc_vec_t    *delvsin)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(delvsin != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_mat_t m; /* S32/33.X rows, 2^1 */

    agc_rendz_csi_cdh_lv_matrix(ip, rz, &m); /* rows */
    rz->delvsin = agc_vec_shl(ip, agc_vec_mat(ip, rz->delvlvc, &m), 1);
    rz->delvsab = rz_abval(ip, rz->delvsin);
    *delvsin    = rz->delvsin;

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_guid_state()->burn.vgdisp = rz->delvsab; /* Luminary VGDISP = DELVSAB */
    }

    return (AGC_STATUS_OK);
}
