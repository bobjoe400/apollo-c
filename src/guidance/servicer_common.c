/**********************************************************************
 * FILE NAME: servicer_common.c
 *
 * PURPOSE:
 *   The average-G package shared by SERVICER207.agc and SERVICER.agc:
 *   the PIPA reader PIPASR, the REREADAC restart decision, the gravity
 *   routine CALCGRAV and the state-vector integrator CALCRVG, and the
 *   copy cycle (GENTRAN / COPYCYC).
 *
 * AGC SOURCE:
 *   Comanche055 SERVICER207.agc pp. 819-836 (PIPASR, REREADAC, CALCGRAV,
 *     CALCRVG, GENTRAN)
 *   Luminary099 SERVICER.agc pp. 857-897 (same routines; differs: UNITZ
 *     for UNITW, KPIP1 after the VXM, PGUIDE for the 2-s period)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale        Description
 *   ------------  ---------  ---  -----------  -----------------------
 *   RN, VN        agc_vec_t  O    2^29, 2^7    state, copy cycle
 *   PIPTIME       agc_dp_t   O    cs 2^28      time of RN, VN
 *   RTX1, RTX2    agc_sp_t   O    -            primary body (MIDTOAV)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_pipasr_noscale    PIPASR +3: reads and zeroes the PIPAs (core)
 *   agc_vec_unit          UNIT: unit vector, |R| and |R|^2 (interp.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   None raised here; AGC_STATUS_BAD_ARG on a failed precondition.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Called from the SERVICER jobs (both vehicles) every 2 s.
 *
 * NOTES:
 *   DEVIATION: PIPASR reads the three PIPAs in one C step (see below).
 *   CALCRVG integrates the equations of motion by averaging the thrust
 *   and gravitational accelerations over the guidance period:
 *     RN1 = RN + (VN + DELVREF/2 + GDT/2) dt
 *     VN1 = VN + DELVREF + GDT/2 + GDT1/2
 *   where GDT1/2 is half the velocity gained from gravity over dt at RN1,
 *   including the first (J2) oblateness term about the earth.
 *
 * ALGORITHM:
 *   As in the listing; scales: RN, RN1 m 2^29; VN, VN1, DELVREF, GDT/2,
 *   GDT1/2 m/cs 2^7; DELV PIPA pulses in the high words (DP scale 2^14
 *   pulses); UNITR half unit (2^1).
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
#include "agc/programs/rendezvous.h"

/* ---- constants (SERVICER207 p.836 / SERVICER p.881, CONTROLLED_CONSTANTS) */
/* -MUDT(E) 2DEC* -7.9720645 E+12 B-44*: -mu(earth) * 200 cs, m^3/cs.      */
static const agc_dp_t MUDT_E =
    -(agc_dp_t) 121644051; /* round(7.9720645e12 / 2^44 * 2^28) */
/* -MUDT(M) / -MUDT1 2DEC* -9.805556 E+10 B-44*: -mu(moon) * 200 cs.       */
static const agc_dp_t MUDT_M =
    -(agc_dp_t) 1496209; /* round(9.805556e10 / 2^44 * 2^28) */
/* DP1/20 2DEC 0.05                                                        */
static const agc_dp_t DP1_20 = (agc_dp_t) 13421773; /* 0.05 * 2^28 */
/* RESQ: earth radius squared 40.6809913 E12 m^2; CM B-59, LM B-58.        */
static const agc_dp_t RESQ_CM =
    (agc_dp_t) 18944; /* 4.06809913e13 / 2^59 * 2^28 */
static const agc_dp_t RESQ_LM =
    (agc_dp_t) 37887; /* 4.06809913e13 / 2^58 * 2^28 */
/* 20J, 2J (J = 1.5 J2 = 1.62346005e-3): CM 2DEC* ... B1* (stored as twice
 * the value: rope 37,3464 = 02047 36332, 37,3466 = 00152 14511), LM B0.   */
static const agc_dp_t J20_CM =
    (agc_dp_t) 17431770;                           /* 3.24692010e-2*2 * 2^28 */
static const agc_dp_t J2_CM  = (agc_dp_t) 1743177; /* 3.24692010e-3*2 * 2^28 */
static const agc_dp_t J20_LM = (agc_dp_t) 8715885; /* 3.24692010e-2 * 2^28 */
static const agc_dp_t J2_LM  = (agc_dp_t) 871588;  /* 3.24692010e-3 * 2^28 */
/* KPIP1: PIPA pulse to m/cs 2^7. CM 2DEC 0.074880 (1 pulse = 5.85 cm/s);
 * LM 2DEC .0128 (1 pulse = 1 cm/s, applied after the half-unit VXM).      */
static const agc_dp_t KPIP1_CM = (agc_dp_t) 20100447; /* 0.074880 * 2^28 */
static const agc_dp_t KPIP1_LM = (agc_dp_t) 3435974;  /* 0.0128 * 2^28 */
/* 2SEC(22) 2DEC 200 B-22: the CM guidance period, cs 2^22.               */
static const agc_dp_t TWOSEC_22 = (agc_dp_t) (200 * 64); /* cs 2^22 */

/* CDU register count of 15 bits; PIPA readings use the SP model.          */

/* ---------------------------------------------------------------------- */

/**********************************************************************
 * PIPASR (CM p.832-833, LM p.869-870): read the PIPA counters and zero
 * them. The high words of DELV get the counts, the low words are zeroed
 * (the AGC zeroes them "for TM indication": non-zero low words on the
 * downlink mean the values have been compensated).
 * DEVIATION: the AGC reads X and Y with one DXCH and Z with an XCH, using
 * TEMX..TEMZ and PIPAGE so that REREADAC can finish a reading interrupted
 * by a restart. In C the reading is done under INHINT and cannot be
 * interrupted; PIPAGE still marks the reading as started (0) for
 * agc_guid_rereadac_resume. The read itself is PIPASR +3 (core).
 *
 * FUNCTION NAME: agc_guid_pipasr
 *
 * ARGUMENTS:
 *   Argument      Type           I/O  Description (scale)
 *   ------------  -------------  ---  --------------------------------
 *   a             agc_avg_g_t *  I/O  average-G block (DELV, RN1, VN1,
 *                                     GDT1/2, PIPAGE...)
 *   set_piptime1  bool           I    also latch PIPTIME1 = TIME2,1
 *                                     (PIPASR, not PIPASR +3)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed)
 **********************************************************************/
agc_status_t agc_guid_pipasr(agc_avg_g_t *a, bool set_piptime1)
{
    if (!AGC_ASSERT(a != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();

    if (set_piptime1)
    {
        a->piptime1 =
            (agc_dp_t) agc_time_now_cs(); /* DCA TIME2 DXCH PIPTIME1 */
    }

    a->pipage = 0;         /* reading in progress     */
    agc_pipasr_noscale(); /* PIPASR +3 (core)        */
    agc_relint();

    if (!AGC_ASSERT(a->piptime1 >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * REREADAC decision (CM p.833, LM p.871): CCS PIPAGE. Positive: the PIPAs
 * were last read in an earlier cycle and marked done at REDO5.5, so this
 * restart came before the new reading -- go back to READACCS. +0: the
 * reading was started (and, in C, completed) -- continue at PIPSDONE.
 *
 * FUNCTION NAME: agc_guid_rereadac_resume
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ------------------------------------
 *   a         agc_avg_g_t *  I/O  average-G block (DELV, RN1, VN1, GDT1/2,
 *                                 PIPAGE...)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed), AGC_STATUS_NOT_FOUND (not found / not
 *          applicable (see above))
 **********************************************************************/
agc_status_t agc_guid_rereadac_resume(agc_avg_g_t *a)
{
    if (!AGC_ASSERT(a != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(a->pipage >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((a->pipage > 0) ? AGC_STATUS_NOT_FOUND : /* positive: READACCS */
            AGC_STATUS_OK);                        /* +0: PIPSDONE      */
}

/**********************************************************************
 * The oblateness term of CALCGRAV (CM p.835, LM p.880), entered with
 * UNITR / UNIT/R/ (half unit) and |R|^2 (34D, 2^58):
 *   s    = UNITR . UNITW (CM)  or  (UNITZ . UNIT/R/) SL1 (LM),  = sin L/2
 *   GOBL = UNITR ((RE/R)^2 20J (0.05 - s^2)) + UNITW ((s 2J) (RE/R)^2)
 *          [LM: UNITZ, then VSL1]
 * In Comanche RESQ is B-59 (so (RE/R)^2 is held at half the Luminary
 * value) and the J terms are B1 (twice the Luminary words): the products,
 * and so GOBL1/2 and UNITGOBL, carry the same scale. The DMPs truncate.
 *
 * FUNCTION NAME: calcgrav_oblateness
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   cm        bool            I    Comanche (UNITW) rather than
 *                                  Luminary (UNITZ)
 *   a         agc_avg_g_t *   I/O  in: UNITR, UNITW, |R|^2 (RMAGSQ);
 *                                  out: (RE/R)^2 (32D), GOBL1/2
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (|R| = 0)
 **********************************************************************/
static agc_status_t calcgrav_oblateness(agc_interp_t *ip, bool cm,
                                        agc_avg_g_t *a)
{
    const agc_vec_t unitz = agc_vec_make(0, 0, AGC_DP_HALF); /* UNITZ, 2^1 */
    agc_dp_t        s;   /* sin(latitude), scale 2^1                */
    agc_vec_t       t2;  /* polar term of GOBL, scale 2^1           */

    if (!AGC_ASSERT(ip != NULL && a != NULL) || !AGC_ASSERT(a->rmagsq > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (cm)
    {
        s = agc_vec_dot(ip, a->unitr, a->unitw); /* DOT UNITW      */
    }
    else
    {
        s = agc_dp_shl(ip, agc_vec_dot(ip, unitz, a->unitr), 1); /* SL1 */
    }

    agc_dp_t fact = /* 0.05 - s^2, scale 2^0 */
                    agc_dp_sub(ip, DP1_20, agc_dp_sq(ip, s)); /* DSQ BDSU DP1/20 */

    a->resq_rsq = agc_dp_div(ip, cm ? RESQ_CM : RESQ_LM, /* CM B-59, LM B-58 */
                             a->rmagsq);                 /* DDV 34D -> 32D  */
    agc_dp_t m = /* equatorial coefficient, scale of GOBL */
                 agc_dp_mul_trunc(agc_dp_mul_trunc(a->resq_rsq,
                                                   cm ? J20_CM : J20_LM), /* 20J */
                                  fact);                 /* DMP DMP        */
    agc_vec_t t1 = agc_vec_scale(ip, a->unitr, m); /* VXSC UNITR, 2^1 */
    agc_dp_t  n  = /* polar coefficient, scale of GOBL */
                   agc_dp_mul_trunc(agc_dp_mul_trunc(s, cm ? J2_CM : J2_LM), /* 2J */
                                    a->resq_rsq);         /* DMP 32D */

    if (cm)
    {
        t2 = agc_vec_scale(ip, a->unitw, n); /* VXSC UNITW     */
    }
    else
    {
        t2 = agc_vec_shl(ip, agc_vec_scale(ip, unitz, n), 1); /* VXSC VSL1 */
    }

    a->gobl1_2 = agc_vec_add(ip, t2, t1); /* VAD, STORE GOBL1/2 */

    return (AGC_STATUS_OK);
} /* calcgrav_oblateness */

/**********************************************************************
 * CALCGRAV (CM pp.835-836, LM p.880): unit vector, magnitude and gravity.
 * UNIT leaves |R| (36D, 2^29) and |R|^2 (34D, 2^58). About the earth the
 * oblateness vector is stored in GOBL1/2 (CM) / UNITGOBL (LM) and added to
 * UNITR. Then (ITISMOON / CALCGRV1) the normalised |R|^2 divides -MU*DT and
 * SLR* 0-21D,2 rescales: with NORM's n left shifts the AGC's X2 holds -n,
 * so the shift is SR (21 - n), rounded. GDT1/2 = that scalar * vector,
 * scaled 2^7 m/cs.
 *
 * FUNCTION NAME: agc_guid_calcgrav
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   veh       agc_vehicle_t   I    vehicle: AGC_VEHICLE_CSM or
 *                                  AGC_VEHICLE_LM
 *   moon      bool            I    moon-centred state (RTX2 = 2): no
 *                                  oblateness term
 *   r         agc_vec_t       I    position RN1, m 2^29 (2^27 moon)
 *   a         agc_avg_g_t *   I/O  in: UNITW; out: UNITR, |R|, |R|^2,
 *                                  GOBL1/2, GDT1/2 (m/cs 2^7)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed)
 **********************************************************************/
agc_status_t agc_guid_calcgrav(agc_interp_t *ip, agc_vehicle_t veh, bool moon,
                               agc_vec_t r, agc_avg_g_t *a)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(a != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(veh == AGC_VEHICLE_CSM || veh == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    a->unitr = agc_vec_unit(ip, r, &a->rmag, &a->rmagsq); /* UNIT PUSH */
    agc_vec_t dir = /* gravity direction, scale 2^1 */
                    a->unitr; /* gravity direction, half unit */

    if (!moon)
    { /* BMN ITISMOON not taken */
        agc_status_t st = /* status of the oblateness term */
                          calcgrav_oblateness(ip, veh == AGC_VEHICLE_CSM, a);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        dir = agc_vec_add(ip, a->gobl1_2, a->unitr); /* VAD PUSH UNITR */
    }

    int      n = 0;                           /* NORM shift count (X2 = -n) */
    agc_dp_t x = agc_dp_norm(a->rmagsq, &n); /* NORM 34D X2, 2^(58-n) m^2 */

    if (!AGC_ASSERT(x != 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t k = /* -mu dt / r^2, rescaled to 2^7 m/cs (half unit vector) */
                 agc_dp_div(ip, moon ? MUDT_M : MUDT_E, x); /* BDDV* -MUDT,1 */

    k         = agc_dp_shl_round(ip, k, n - 21); /* SLR* 0-21D,2   */
    a->gdt1_2 = agc_vec_scale(ip, dir, k);       /* VXSC STORE     */

    return (AGC_STATUS_OK);
} /* agc_guid_calcgrav */

/**********************************************************************
 * CALCRVG (CM p.836, LM pp.880-881). Comanche scales DELV by KPIP1 before
 * the VXM; Luminary does the VXM first (and its KPIP1 assumes the extra
 * factor of 2 the half-unit matrix introduces). The dt is the fixed
 * 2SEC(22) in Comanche and PGUIDE (SL 6 to 2^22) in Luminary.
 *
 * FUNCTION NAME: agc_guid_calcrvg
 *
 * ARGUMENTS:
 *   Argument  Type                        I/O  Description (scale)
 *   --------  --------------------------  ---  -----------------------
 *   ip        agc_interp_t *              I/O  interpreter state (OVFIND,
 *                                              index registers)
 *   veh       agc_vehicle_t               I    vehicle: AGC_VEHICLE_CSM or
 *                                              AGC_VEHICLE_LM
 *   moon      bool                        I    moon-centred state (RTX2 =
 *                                              2): no oblateness term
 *   refsmmat  const agc_mat_t *           I    REFSMMAT, half-unit rows
 *   delv      agc_vec_t                   I    DELV, PIPA counts this cycle
 *                                              (SM), pulses
 *   sv        const agc_state_vector_t *  I    RN m 2^29, VN m/cs 2^7 at
 *                                              PIPTIME
 *   a         agc_avg_g_t *               I/O  average-G block (DELV, RN1,
 *                                              VN1, GDT1/2, PIPAGE...)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed); otherwise the status of the routine it
 *          continues in
 **********************************************************************/
agc_status_t agc_guid_calcrvg(agc_interp_t *ip, agc_vehicle_t veh, bool moon,
                              const agc_mat_t *refsmmat, agc_vec_t delv,
                              const agc_state_vector_t *sv, agc_avg_g_t *a)
{
    if (!AGC_ASSERT(ip != NULL && refsmmat != NULL) ||
        !AGC_ASSERT(sv != NULL && a != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t dt22; /* guidance period, cs 2^22 */

    if (veh == AGC_VEHICLE_CSM)
    {
        agc_vec_t dv = /* PIPA delta-V, SM, m/cs 2^7 */
                       agc_vec_scale(ip, delv, KPIP1_CM); /* VXSC KPIP1  */

        a->delvref =
            agc_vec_shl(ip, agc_vec_mat(ip, dv, refsmmat), 1); /* VXM VSL1 */
        dt22 = TWOSEC_22;
    }
    else
    {
        agc_vec_t dv = /* PIPA counts, reference frame, 2^14 pulses */
                       agc_vec_mat(ip, delv, refsmmat); /* VXM REFSMMAT */

        a->delvref =
            agc_vec_shl(ip, agc_vec_scale(ip, dv, KPIP1_LM), 1); /* VXSC VSL1 */
        dt22 = agc_dp_shl(ip, a->pguide, 6); /* PDDL PGUIDE SL 6 */
    }

    agc_vec_t half_dv = /* DELVREF/2, m/cs 2^7 */
                        agc_vec_shr(a->delvref, 1); /* VSR1 PUSH   */
    agc_vec_t avg_acc = /* DELVREF/2 + GDT/2, m/cs 2^7 */
                        agc_vec_add(ip, half_dv, a->gdt_2); /* VAD PUSH GDT/2 */
    agc_vec_t vdt = /* position change, m 2^29 */
                    agc_vec_scale(ip, agc_vec_add(ip, avg_acc, sv->v), dt22); /* VAD VN, VXSC */

    a->rn1 = agc_vec_add(ip, vdt, sv->r); /* VAD RN; STORE RN1 */
    agc_status_t st = /* status of CALCGRAV */
                      agc_guid_calcgrav(ip, veh, moon, a->rn1, a);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_vec_t v1 = /* VN + GDT1/2, m/cs 2^7 */
                   agc_vec_add(ip, a->gdt1_2, sv->v); /* VAD VN      */

    v1     = agc_vec_add(ip, v1, avg_acc);      /* VAD (pop)   */
    a->vn1 = agc_vec_add(ip, v1, half_dv);      /* VAD (pop)   */

    return (AGC_STATUS_OK);
} /* agc_guid_calcrvg */

/**********************************************************************
 * GENTRAN (CM AVERAGEG p.829, NORMLIZE p.831) / COPYCYC (LM p.867): copy
 * the newly computed values into the working ones under INHINT, for
 * restart protection. Comanche's 26-word copy includes GOBL1/2 -> GOBL/2;
 * Luminary's 22-word COPYCYC includes MASS1 -> MASS (NORMLIZ2 copies 20,
 * without MASS).
 *
 * FUNCTION NAME: agc_guid_copycycle
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   veh       agc_vehicle_t    I    vehicle: AGC_VEHICLE_CSM or
 *                                   AGC_VEHICLE_LM
 *   kind      agc_copy_kind_t  I    which block to copy (R V T, + G, +
 *                                   mass)
 **********************************************************************/
void agc_guid_copycycle(agc_vehicle_t veh, agc_copy_kind_t kind)
{
    if (!AGC_ASSERT(veh == AGC_VEHICLE_CSM || veh == AGC_VEHICLE_LM))
    {
        return;
    }

    if (!AGC_ASSERT(kind == AGC_COPY_RVT || kind == AGC_COPY_RVTG ||
                    kind == AGC_COPY_ALL))
    {
        return;
    }

    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    agc_inhint();
    agc_erase_set_rn(g->avg.rn1);
    agc_erase_set_vn(g->avg.vn1);
    agc_erase_set_piptime(g->avg.piptime1);

    if (kind != AGC_COPY_RVT)
    {
        g->avg.gdt_2 = g->avg.gdt1_2;

        if (veh == AGC_VEHICLE_CSM)
        {
            g->avg.gobl_2 = g->avg.gobl1_2;
        }
    }

    if (kind == AGC_COPY_ALL && veh == AGC_VEHICLE_LM)
    {
        g->lm.mass = g->lm.mass1;
    }

    agc_relint();
}

/**********************************************************************
 * MIDTOAV1 / MIDTOAV2 leave RN1, VN1, PIPTIME1 (average-G block) and RTX1,
 * RTX2 (SXA,1 / SXA,2) in erasable (INTEGRATION_INITIALIZATION p.1330-1333);
 * nav-integ returns them in *o, the caller stores them here.
 *
 * FUNCTION NAME: agc_guid_store_midtoav
 *
 * ARGUMENTS:
 *   Argument  Type                             I/O  Description (scale)
 *   --------  -------------------------------  ---  --------------------
 *   o         const agc_integ_midtoav_out_t *  I
 *        MIDTOAV1/2 results (RN1 VN1 PIPTIME1 RTX1 RTX2)
 **********************************************************************/
void agc_guid_store_midtoav(const agc_integ_midtoav_out_t *o)
{
    if (!AGC_ASSERT(o != NULL))
    {
        return;
    }

    if (!AGC_ASSERT(o->rtx2 == 0 || o->rtx2 == 2))
    {
        return;
    }

    agc_avg_g_t *a  = &agc_guid_state()->avg; /* average-G block */
    agc_rendz_t *rz = /* rendezvous erasables (RTX1, RTX2) */
                      agc_rendz_erasable();

    a->rn1        = o->rn1;
    a->vn1        = o->vn1;
    a->piptime1   = o->piptime1;
    rz->body.rtx1 = (agc_sp_t) o->rtx1;
    rz->body.rtx2 = (agc_sp_t) o->rtx2;
    rz->body.moon = (o->rtx2 != 0);
}
