/*
 * common.c -- rendezvous erasables and the shared rendezvous
 * subroutines of P34-35_P74-75.agc (Comanche pp.469-500, Luminary
 * equivalent): SELECTMU, VECSHIFT, SHIFTR1, PERIAPO/PERIAPO1/SETRAD,
 * PRECSET/ACTIVE/PASSIVE, INTINT, S34/35.1, LOMAT, MIDGIM, AVFLAGA/P,
 * P20FLGON, plus the numeric helpers used by every rendezvous file.
 *
 * Scales (from the AGC text): r m 2^29, v m/cs 2^7, t cs 2^28, angles
 * revolutions 2^0, unit vectors 2^1, mu 2^36. "Body scale" is 2^27/2^5
 * when RTX2 = 2 (moon).
 */
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"
#include "internal.h"

/* The rendezvous erasables (E-bank storage of ERASABLE_ASSIGNMENTS). */
static agc_rendz_t rendz_erasable;

/* MUTABLE (CONIC_SUBROUTINES p.1307, LM CONTROLLED_CONSTANTS):
 * MUE   2DEC* 3.986032 E10 B-36*    -> 155704375
 * 1/SQRT(MUE) .50087529 E-5 B+17*   -> 176229826
 * MUM   2DEC 4.902778 E8 B-30       -> 122569450
 * 1/SQRT(MUM) .45162595 E-4 B+14*   -> 198627193 */
static const agc_dp_t mutable_mue       = 155704375;
static const agc_dp_t mutable_rtmue_inv = 176229826;
static const agc_dp_t mutable_mum       = 122569450;
static const agc_dp_t mutable_rtmum_inv = 198627193;

/* MUTABLE entry for the primary body: MUE 2^36 (earth) or MUM 2^30 (moon),
 * i.e. DLOAD* MUTABLE -2,1 with X1 = RTX1.                              */
agc_dp_t rz_mutable_mu(bool moon)
{
    agc_dp_t mu = moon ? mutable_mum : mutable_mue;

    if (!AGC_ASSERT(mu > 0))
    {
        return (0); /* no valid result */
    }

    return (mu);
}

agc_rendz_t *agc_rendz_erasable(void)
{
    return (&rendz_erasable);
}

/* VHFCNT (CM): VHF range mark count. */
agc_sp_t agc_erase_vhfcnt(void)
{
    if (!AGC_ASSERT(rendz_erasable.vhfcnt >= AGC_SP_MIN))
    {
        return (0); /* no valid result */
    }

    return (rendz_erasable.vhfcnt);
}

void agc_erase_set_vhfcnt(agc_sp_t v)
{
    if (!AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX))
    {
        return;
    }

    rendz_erasable.vhfcnt = v;
}

/* MRKBUF1, MRKBUF2 (CM, EBANK= MRKBUF1): the two seven-word mark buffers
 * (time DP, CDUY, CDUS, CDUZ, CDUT, CDUX), raw words; word 0 = -1 (77776)
 * marks a buffer empty. SXTMARK fills MRKBUF1 (and moves it to MRKBUF2
 * for R21); R22, P76 and the verbs read and clear them.                */
#define RZ_MRKBUF_WORDS 7
static uint16_t mrkbuf[2][RZ_MRKBUF_WORDS];

uint16_t agc_erase_mrkbuf(int buf, int index)
{
    if (!AGC_ASSERT(buf == 1 || buf == 2) ||
        !AGC_ASSERT(index >= 0 && index < RZ_MRKBUF_WORDS))
    {
        return (0U);
    }

    return (mrkbuf[buf - 1][index]);
}

void agc_erase_set_mrkbuf(int buf, int index, uint16_t v)
{
    if (!AGC_ASSERT(buf == 1 || buf == 2) ||
        !AGC_ASSERT(index >= 0 && index < RZ_MRKBUF_WORDS))
    {
        return;
    }

    mrkbuf[buf - 1][index] = (uint16_t) (v & 077777u);
}

/* MRKBUF2 +index as an SP number (P76 ENDP76, R22: -1 = invalid).      */
agc_sp_t agc_erase_mrkbuf2(int index)
{
    return ((agc_sp_t) rz_word_value(agc_erase_mrkbuf(2, index)));
}

void agc_erase_set_mrkbuf2(int index, agc_sp_t v)
{
    if (!AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX))
    {
        return;
    }

    uint16_t w = (v >= 0) ? (uint16_t) v : (uint16_t) (077777 + v);

    agc_erase_set_mrkbuf(2, index, w);
}

/* TANG, TANG +1 (LM, noun 72): RR gimbal command or, for RRDESNB, the
 * desired trunnion and shaft CDU words, as ones' complement SP numbers.
 * RSTACK (LM, R04/R77 nouns 66, 67, 78): four DP radar samples, m or
 * m/cs at the radar scale. Owned here so the noun tables (this layer)
 * need not reach up into src/lm/radar; the radar code uses the accessors. */
#define RZ_TANG_WORDS   2
#define RZ_RSTACK_WORDS 4
static agc_sp_t tang[RZ_TANG_WORDS];
static agc_dp_t rstack[RZ_RSTACK_WORDS];

agc_sp_t agc_erase_tang(int index)
{
    if (!AGC_ASSERT(index >= 0 && index < RZ_TANG_WORDS))
    {
        return (0);
    }

    return (tang[index]);
}

void agc_erase_set_tang(int index, agc_sp_t v)
{
    if (!AGC_ASSERT(index >= 0 && index < RZ_TANG_WORDS) ||
        !AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX))
    {
        return;
    }

    tang[index] = v;
}

agc_dp_t agc_erase_rstack(int index)
{
    if (!AGC_ASSERT(index >= 0 && index < RZ_RSTACK_WORDS))
    {
        return (0);
    }

    return (rstack[index]);
}

void agc_erase_set_rstack(int index, agc_dp_t v)
{
    if (!AGC_ASSERT(index >= 0 && index < RZ_RSTACK_WORDS) ||
        !AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX))
    {
        return;
    }

    rstack[index] = v;
}

/* LANDMARK (CM P22 code ABCDE, V52), HORIZON (CM noun 70/71 R3) and
 * the CM R63 results CPHIX (noun 96, X axis) and PRAXIS (noun 95,
 * preferred axis), CDU words. Owned here, with the noun tables, so this
 * layer does not reach up into src/csm/navigation.                     */
static uint16_t landmark;
static uint16_t horizon;
static uint16_t trk_angles[2][3]; /* [0] CPHIX, [1] PRAXIS               */

agc_sp_t agc_erase_landmark(void)
{
    if (!AGC_ASSERT(landmark <= 077777u))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) rz_word_value(landmark));
}

void agc_erase_set_landmark(agc_sp_t v)
{
    if (!AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX))
    {
        return;
    }

    landmark = (v >= 0) ? (uint16_t) v : (uint16_t) (077777 + v);
}

uint16_t agc_erase_horizon(void)
{
    if (!AGC_ASSERT(horizon <= 077777u))
    {
        return (0U); /* no valid result */
    }

    return (horizon);
}

void agc_erase_set_horizon(uint16_t v)
{
    if (!AGC_ASSERT(v <= 077777u))
    {
        return;
    }

    horizon = (uint16_t) (v & 077777u);
}

uint16_t agc_rendz_tracking_angles(bool preferred, int axis)
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3))
    {
        return (0U);
    }

    return (trk_angles[preferred ? 1 : 0][axis]);
}

uint16_t *rz_tracking_angles(bool preferred)
{
    uint16_t *p = trk_angles[preferred ? 1 : 0];

    if (!AGC_ASSERT(p != NULL))
    {
        return (trk_angles[0]);
    }

    return (p);
}

/* Fresh start: all rendezvous erasables zero, earth selected. */
void agc_rendz_reset(agc_rendz_t *rz)
{
    if (!AGC_ASSERT(rz != NULL))
    {
        return;
    }

    static const agc_rendz_t zero;

    *rz = zero;

    if (agc_rendz_selectmu(rz, false) != AGC_STATUS_OK)
    {
        return;
    }

    if (!AGC_ASSERT(rz->body.rtx1 == -2))
    {
        return;
    }
}

/* ---- vehicle-dependent flags ------------------------------------------- */
bool rz_is_csm(void)
{
    agc_vehicle_t v = agc_state_vehicle();

    if (!AGC_ASSERT(v == AGC_VEHICLE_CSM || v == AGC_VEHICLE_LM))
    {
        return (false);
    }

    return (v == AGC_VEHICLE_CSM);
}

agc_flag_id_t rz_vflag(rz_vflag_t f)
{
    if (!AGC_ASSERT((int) f >= 0 && f < RZ_VF_COUNT))
    {
        return (rz_vflag_csm(RZ_VF_ITSWICH));
    }

    return (rz_is_csm() ? rz_vflag_csm(f) : rz_vflag_lm(f));
}

/* A flag that does not exist in this vehicle reads as clear. */
bool rz_vflag_test(rz_vflag_t f)
{
    if (!AGC_ASSERT((int) f >= 0 && f < RZ_VF_COUNT))
    {
        return (false);
    }

    agc_flag_id_t id = rz_vflag(f);

    return ((id != 0U) && agc_flag_test(id));
}

/* SET / CLEAR by a run-time choice (cannot fail for a valid flag id). */
void rz_flag_put(agc_flag_id_t f, bool on)
{
    if (!AGC_ASSERT(f != 0U))
    {
        return;
    }

    if (on)
    {
        agc_flag_set(f);
    }
    else
    {
        agc_flag_clear(f);
    }
}

void rz_vflag_set(rz_vflag_t f, bool on)
{
    if (!AGC_ASSERT((int) f >= 0 && f < RZ_VF_COUNT))
    {
        return;
    }

    agc_flag_id_t id = rz_vflag(f);

    if (!AGC_ASSERT(id != 0U))
    {
        return; /* not a flag of this vehicle */
    }

    rz_flag_put(id, on);
}

/* MOONTHIS EQUALS CMOONFLG (CM) / LMOONFLG (LM), TAGS_FOR_RELATIVE_SETLOC */
bool rz_moonthis(void)
{
    agc_flag_id_t f = rz_is_csm() ? AGC_FLAG_CMOONFLG : AGC_FLAG_LMOONFLG;

    if (!AGC_ASSERT(f != 0U))
    {
        return (false);
    }

    return (agc_flag_test(f));
}

/* ---- numeric helpers ---------------------------------------------------- */
/* UNIT: half-unit vector (scale 2^1) */
agc_vec_t rz_unit(agc_interp_t *ip, agc_vec_t v)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    if (!AGC_ASSERT(v.c[0] != AGC_DP_MIN))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    return (agc_vec_unit(ip, v, NULL, NULL));
}

/* ABVAL */
agc_dp_t rz_abval(agc_interp_t *ip, agc_vec_t v)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (0); /* no valid result */
    }

    agc_dp_t a = agc_vec_abval(ip, v);

    if (!AGC_ASSERT(a >= 0))
    {
        return (0); /* no valid result */
    }

    return (a);
}

/* NORM X: left-justify; *xreg = the AGC index register, -(left shifts). */
/* NORM when only the index register is wanted: -(left shifts), 0 for 0. */
int rz_norm_xreg(agc_dp_t a)
{
    if (a == 0)
    {
        return (0);
    }

    const int n = agc_dp_norm_count(a);

    if (!AGC_ASSERT(n >= 0 && n <= AGC_DP_BITS))
    {
        return (0); /* no shift */
    }

    return (-n);
}

agc_dp_t rz_norm(agc_dp_t a, int *xreg)
{
    if (!AGC_ASSERT(xreg != NULL))
    {
        return (a);
    }

    if (a == 0)
    {
        *xreg = 0;

        return (0);
    }

    int      n = 0;
    agc_dp_t r = agc_dp_norm(a, &n);

    if (!AGC_ASSERT(n >= 0 && n <= AGC_DP_BITS))
    {
        return (a);
    }

    *xreg = -n;

    return (r);
}

/* SL k (k > 0) or SR -k (k < 0): the AGC SL* and SR* with an index.     */
agc_dp_t rz_shift(agc_interp_t *ip, agc_dp_t a, int k)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(k >= -60 && k <= 60))
    {
        return (a);
    }

    if (k > 0)
    {
        return (agc_dp_shl(ip, a, k));
    }

    if (k < 0)
    {
        return (agc_dp_shr(a, -k));
    }

    return (a);
}

/* SL* addr,X: shift count = addr - C(X); a negative count shifts right. */
agc_dp_t rz_sl_star(agc_interp_t *ip, agc_dp_t a, int addr, int xreg)
{
    if (!AGC_ASSERT(addr >= -30 && addr <= 30))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(xreg >= -60 && xreg <= 60))
    {
        return (0); /* no valid result */
    }

    return (rz_shift(ip, a, addr - xreg));
}

/* SR* addr,X: shift right by addr - C(X); a negative count shifts left. */
agc_dp_t rz_sr_star(agc_interp_t *ip, agc_dp_t a, int addr, int xreg)
{
    if (!AGC_ASSERT(addr >= -30 && addr <= 30))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(xreg >= -60 && xreg <= 60))
    {
        return (0); /* no valid result */
    }

    return (rz_shift(ip, a, xreg - addr));
}

agc_vec_t rz_vshift(agc_interp_t *ip, agc_vec_t v, int k)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(k >= -60 && k <= 60))
    {
        return (v);
    }

    if (k > 0)
    {
        return (agc_vec_shl(ip, v, k));
    }

    if (k < 0)
    {
        return (agc_vec_shr(v, -k));
    }

    return (v);
}

/* VECSHIFT / VSR* 0,2 (P34-35 p.499): earth scale -> body scale. With
 * X2 = RTX2 = 2 (moon) the count 0 - X2 is negative: a left shift by 2. */
agc_vec_t agc_rendz_vec_to_body(agc_interp_t *ip, const agc_rendz_t *rz,
                                agc_vec_t v)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL))
    {
        return (v);
    }

    if (!AGC_ASSERT(rz->body.rtx2 == 0 || rz->body.rtx2 == 2))
    {
        return (v);
    }

    return (rz_vshift(ip, v, rz->body.rtx2));
}

/* VSL* 0,2: body scale -> earth scale (moon: shift right 2). */
agc_vec_t rz_vec_from_body(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t v)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL))
    {
        return (v);
    }

    if (!AGC_ASSERT(rz->body.rtx2 == 0 || rz->body.rtx2 == 2))
    {
        return (v);
    }

    return (rz_vshift(ip, v, -(int) rz->body.rtx2));
}

/* SHIFTR1 (P34-35 p.500): SL* 0,2 -- a body-scale scalar to 2^29. */
agc_dp_t agc_rendz_scalar_from_body(const agc_rendz_t *rz, agc_dp_t a)
{
    if (!AGC_ASSERT(rz != NULL))
    {
        return (a);
    }

    if (!AGC_ASSERT(rz->body.rtx2 == 0 || rz->body.rtx2 == 2))
    {
        return (a);
    }

    return (agc_dp_shr(a, rz->body.rtx2));
}

/* ---- SELECTMU (P34-35 p.494) --------------------------------------------
 * Choose mu and the scale indices for the primary body of the CSM state
 * (CMOONFLG). RTMU is always at 2^36 (the lunar MUM, 2^30, is shifted
 * right 6). The AGC then clears FINALFLG and goes to VN1645; the program
 * drivers do the display (rz_ttogo_display) themselves.                         */
agc_status_t agc_rendz_selectmu(agc_rendz_t *rz, bool cmoonflg)
{
    if (!AGC_ASSERT(rz != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz->body.moon = cmoonflg;

    if (cmoonflg)
    {
        rz->body.rtx1    = -10;
        rz->body.rtx2    = 2;
        rz->body.rtsr1mu = mutable_rtmum_inv;          /* 2^14 */
        rz->body.rtmu    = agc_dp_shr(mutable_mum, 6); /* 2^30 -> 2^36 */
    }
    else
    {
        rz->body.rtx1    = -2;
        rz->body.rtx2    = 0;
        rz->body.rtsr1mu = mutable_rtmue_inv; /* 2^17 */
        rz->body.rtmu    = mutable_mue;       /* 2^36 */
    }

    if (!AGC_ASSERT(rz->body.rtmu > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* ---- AVFLAGA / AVFLAGP / P20FLGON (CM P30-P37 p.~, LM P32-P35) ---------
 * AVFLAG reset = CSM active, set = LM active. "This vehicle active" is
 * AVFLAGA. The CM AVFLAGA also sets ECSTEER = 1 (BIT13).                  */
void rz_avflag(bool this_active)
{
    bool csm       = rz_is_csm();
    bool lm_active = csm ? !this_active : this_active;

    rz_flag_put(AGC_FLAG_AVFLAG, lm_active);

    if (csm && this_active)
    {
        /* CAF BIT13, TS ECSTEER: "SET ECSTEER = 1" (high word only) */
        agc_burn_t *b = &agc_guid_state()->burn;

        b->ecsteer = agc_dp_pack((agc_sp_t) 010000, agc_dp_low(b->ecsteer));
    }

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_AVFLAG) == lm_active))
    {
        return;
    }
}

void rz_tracking_flags_on(void)
{
    agc_flag_set(AGC_FLAG_TRACKFLG);
    agc_flag_set(AGC_FLAG_UPDATFLG);

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_UPDATFLG)))
    {
        return;
    }

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_TRACKFLG)))
    {
        return;
    }
}

/* ---- PRECSET / LEMSTORE / CSMSTORE / ACTIVE / PASSIVE (p.498) ---------- */
void rz_store_active(agc_rendz_t *rz, agc_vec_t r, agc_vec_t v)
{
    if (!AGC_ASSERT(rz != NULL))
    {
        return;
    }

    rz->ract3 = r; /* ACTIVE: STOVL RACT3        */
    rz->vact3 = v; /*         STORE VACT3        */

    if (!AGC_ASSERT(rz->body.rtx2 >= 0))
    {
        return;
    }
}

void rz_store_passive(agc_rendz_t *rz, agc_vec_t r, agc_vec_t v)
{
    if (!AGC_ASSERT(rz != NULL))
    {
        return;
    }

    rz->rpass3 = r; /* PASSIVE: STOVL RPASS3      */
    rz->vpass3 = v; /*          STORE VPASS3      */

    if (!AGC_ASSERT(rz->body.rtx2 >= 0))
    {
        return;
    }
}

/* PRECSET: TDEC2 = TDEC1; LEMPREC, LEMSTORE (AVFLAG set = LM active);
 * TDEC1 = TDEC2; CSMPREC, CSMSTORE. RATT/VATT are at earth scale.     */
agc_status_t rz_precset(agc_interp_t *ip, agc_rendz_t *rz, agc_dp_t tdec1,
                        agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(tdec1 >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    bool            lm_active = agc_flag_test(AGC_FLAG_AVFLAG);
    agc_integ_out_t o;

    rz->tdec2 = tdec1; /* TDEC2   */
    agc_status_t st = rz_integ(ip, RZ_LEMPREC, tdec1, resume, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (lm_active)
    { /* LEMSTORE */
        rz_store_active(rz, o.ratt, o.vatt);
    }
    else
    {
        rz_store_passive(rz, o.ratt, o.vatt);
    }

    st = rz_integ(ip, RZ_CSMPREC, rz->tdec2, resume, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (lm_active)
    { /* CSMSTORE */
        rz_store_passive(rz, o.ratt, o.vatt);
    }
    else
    {
        rz_store_active(rz, o.ratt, o.vatt);
    }

    return (AGC_STATUS_OK);
}

/* ---- INTINT (p.479) ------------------------------------------------------
 * Push list on entry: V, R, T1, T2, conic indicator (non-zero = conic).
 * INTSTALL; INTYPFLG per indicator; TDEC1 = T2; MOONFLAG = CMOONFLG;
 * TET = T1; RCV, VCV = VSR* 0,2 (body scale); INTEGRVS; RATT, VATT out.
 * GOINT (p.479) is INTINT with T2 = TTPI + NOMTPI... called by S33/34.1. */
agc_status_t rz_intint(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t r,
                       agc_vec_t v, agc_dp_t t1, agc_dp_t t2, bool conic,
                       agc_job_id_t resume, agc_vec_t *ratt, agc_vec_t *vatt)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) ||
        !AGC_ASSERT(ratt != NULL && vatt != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_integrvs_in_t in;

    in.conic = conic;
    in.tdec1 = t2;
    in.moon  = agc_flag_test(AGC_FLAG_CMOONFLG);
    in.tet   = t1;
    in.rcv   = agc_rendz_vec_to_body(ip, rz, r);
    in.vcv   = agc_rendz_vec_to_body(ip, rz, v);
    agc_integ_out_t o;
    agc_status_t    st = rz_integrvs(ip, &in, resume, &o);

    if (st == AGC_STATUS_OK)
    {
        *ratt = o.ratt;
        *vatt = o.vatt;
    }

    return (st);
}

/* TIMETHET call set-up used throughout P32-P39 (VECSHIFT, X1 = RTX1). */
agc_status_t rz_timethet_rv(agc_interp_t *ip, const agc_rendz_t *rz,
                            agc_vec_t r, agc_vec_t v, agc_dp_t snth,
                            agc_dp_t csth, bool rvsw, agc_conic_time_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_timetheta_in_t in;

    in.rvec = agc_rendz_vec_to_body(ip, rz, r);
    in.vvec = agc_rendz_vec_to_body(ip, rz, v);
    in.snth = snth;
    in.csth = csth;
    in.rvsw = rvsw;
    in.body = agc_conic_body_of_x1(rz->body.rtx1);
    rz_flag_put(AGC_FLAG_RVSW, rvsw);

    return (agc_conic_timethet(ip, &in, out));
}

/* ---- PERIAPO1 / PERIAPO / SETRAD (p.495-497) ----------------------------
 * PERIAPO1 shifts the earth-scale state in the push list to body scale;
 * SETRAD gives XXXALT = RPAD (2^29) about the earth, |RLS| (2^27) about
 * the moon; APSIDES gives the apsides. Altitudes = radii - XXXALT, all at
 * body scale (callers apply SHIFTR1).                                    */
agc_status_t agc_rendz_periapo_body_scale(agc_interp_t *ip,
                                          const agc_rendz_t *rz,
                                          agc_vec_t r, agc_vec_t v,
                                          agc_rendz_periapo_t *out)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t vvec = agc_rendz_vec_to_body(ip, rz, v);
    agc_vec_t rvec = agc_rendz_vec_to_body(ip, rz, r);

    return (agc_rendz_periapo(ip, rz, rvec, vvec, out));
}

/* PERIAPO (p.496): RVEC, VVEC already at body scale. */
agc_status_t agc_rendz_periapo(agc_interp_t *ip, const agc_rendz_t *rz,
                               agc_vec_t rvec, agc_vec_t vvec,
                               agc_rendz_periapo_t *out)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_conic_body_t body = agc_conic_body_of_x1(rz->body.rtx1);

    /* SETRAD: RPAD (2^29) about the earth, |RLS| (2^27) about the moon */
    out->xxxalt =
        (body == AGC_CONIC_MOON) ? rz_abval(ip, agc_erase_rls()) : RZ_RPAD_B29;
    agc_apsides_out_t aps;
    agc_status_t      st = agc_conic_apsides(ip, rvec, vvec, body, &aps);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    out->rapo  = aps.rapo;  /* 2D */
    out->rperi = aps.rperi; /* 6D */
    out->ecc   = aps.ecc;
    out->r1    = aps.r1;
    out->p     = aps.p;
    out->hapo  = agc_dp_sub(ip, out->rapo, out->xxxalt);  /* 4D */
    out->hperi = agc_dp_sub(ip, out->rperi, out->xxxalt); /* 8D */

    return (AGC_STATUS_OK);
}

/* ---- S34/35.1 (p.474): unit normal and line of sight at T3 ------------- */
agc_status_t agc_rendz_los_and_normal(agc_interp_t *ip, agc_rendz_t *rz)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz->ulos = rz_unit(ip, agc_vec_sub(ip, rz->rpass3, rz->ract3));
    rz->unrm = rz_unit(ip, agc_vec_cross(ip, rz->ract3, rz->vact3));

    if (!AGC_ASSERT(rz->unrm.c[0] != AGC_DP_MIN))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* ---- LOMAT (p.479): LV matrix rows X = Z x (-Y)... from UNRM, RACT3 ----
 * Y = -UNRM, Z = -UNIT(RACT3), X = (Z x UNRM) VSL1. Rows at 2^1.         */
agc_status_t agc_rendz_lomat(agc_interp_t *ip, const agc_rendz_t *rz,
                             agc_mat_t *m)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(m != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    m->row[1] = agc_vec_neg(rz->unrm);
    m->row[2] = agc_vec_neg(rz_unit(ip, rz->ract3));
    m->row[0] = agc_vec_shl(ip, agc_vec_cross(ip, m->row[2], rz->unrm), 1);

    return (AGC_STATUS_OK);
}

/* ---- GET.LVC (MIDGIM p.493): delta V to local vertical coordinates -----
 * Z = U(-R), Y = U(U(-R) x V), X = U(Y x Z); DELVLVC = M dV, VSL1.      */
agc_status_t agc_rendz_get_lvc(agc_interp_t *ip, agc_vec_t rinit,
                               agc_vec_t vinit, agc_vec_t dv,
                               agc_vec_t *delvlvc)
{
    if (!AGC_ASSERT(ip != NULL && delvlvc != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_mat_t m;

    m.row[2] = agc_vec_neg(rz_unit(ip, rinit));                    /* 18D */
    m.row[1] = rz_unit(ip, agc_vec_cross(ip, m.row[2], vinit));    /* 12D */
    m.row[0] = rz_unit(ip, agc_vec_cross(ip, m.row[1], m.row[2])); /*  6D */
    *delvlvc = agc_vec_shl(ip, agc_mat_vec(ip, &m, dv), 1);

    if (!AGC_ASSERT(delvlvc->c[0] != AGC_DP_MIN))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_set(AGC_FLAG_MGLVFLAG);

    return (AGC_STATUS_OK);
}

/* ---- GET+MGA (MIDGIM p.493): +MGA = ARCSIN(UNIT(dV) . Ysm) in 0..1 rev */
agc_status_t agc_rendz_get_mga(agc_interp_t *ip, agc_vec_t dv,
                               const agc_mat_t *refsmmat, agc_dp_t *mga)
{
    if (!AGC_ASSERT(ip != NULL && refsmmat != NULL) || !AGC_ASSERT(mga != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* HALFREV 2DEC 1 B-1 */
    static const agc_dp_t halfrev = AGC_DP_HALF;
    agc_dp_t              s       =
        agc_dp_shl(ip, agc_vec_dot(ip, rz_unit(ip, dv), refsmmat->row[1]), 1);
    agc_dp_t a = agc_dp_asin(ip, s);

    if (a < 0)
    { /* convert -MGA to +MGA: add one rev */
        a = agc_dp_add(ip, agc_dp_add(ip, a, halfrev), halfrev);
    }

    *mga = a;
    agc_flag_clear(AGC_FLAG_MGLVFLAG);

    return (AGC_STATUS_OK);
}

/* ---- navigation pad loads and X789 (P20-P25 erasables, owned here) ---- */
static agc_rendz_nav_pad_t nav_pad;
static agc_vec_t           x789; /* X789: radar / landmark bias states     */

agc_rendz_nav_pad_t *agc_rendz_nav_pad(void)
{
    return (&nav_pad);
}

agc_vec_t agc_erase_x789(void)
{
    if (!AGC_ASSERT(x789.c[0] >= AGC_DP_MIN))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    return (x789);
}

void agc_erase_set_x789(agc_vec_t v)
{
    if (!AGC_ASSERT(v.c[0] >= AGC_DP_MIN && v.c[0] <= AGC_DP_MAX))
    {
        return;
    }

    x789 = v;
}

int rz_word_value(uint16_t w)
{
    uint16_t v = (uint16_t) (w & 077777u);
    int      r = ((v & 040000u) != 0u) ? -(int) (077777u - v) : (int) v;

    if (!AGC_ASSERT(r >= AGC_SP_MIN && r <= AGC_SP_MAX))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/* INCORP2: W update, then the deviation into the corrected vehicle's
 * state (MOVEP.., FAZAB, RECTIFY on overflow, MOVEA.. with SVDWN), X789,
 * INTWAKE. moonthis: CMOONFLG (CM) or LMOONFLG (LM).                    */
agc_status_t rz_incorp2(agc_interp_t *ip, const agc_incorp_t *inc,
                        bool moonthis)
{
    if (!AGC_ASSERT(ip != NULL && inc != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();
    bool             csm = agc_flag_test(AGC_FLAG_VEHUPFLG);
    agc_vehicle_t    veh = csm ? AGC_VEHICLE_CSM : AGC_VEHICLE_LM;
    agc_status_t     st  = agc_incorp_update_w(ip, inc, &ctx->w);

    agc_phase_change(2, 04022); /* FAZC     */

    if (st == AGC_STATUS_OK)
    {
        st = agc_integ_perm_get(veh, &ctx->sv); /* MOVEP..  */
    }

    agc_incorp_sv_t sv;

    sv.moon    = moonthis; /* MOONTHIS */
    sv.rcv     = ctx->sv.rcv;
    sv.vcv     = ctx->sv.vcv;
    sv.tdeltav = ctx->sv.deltav;
    sv.tnuv    = ctx->sv.nuv;
    sv.x789    = agc_erase_x789();

    if (st == AGC_STATUS_OK)
    {
        st = agc_incorp_update_state(ip, inc, &sv);
    }

    ctx->sv.rcv    = sv.rcv;
    ctx->sv.vcv    = sv.vcv;
    ctx->sv.deltav = sv.tdeltav;
    ctx->sv.nuv    = sv.tnuv;
    ctx->pbody     = sv.moon ? 2 : 0;
    rz_flag_put(AGC_FLAG_MOONFLAG, sv.moon);

    if (st == AGC_STATUS_OK && sv.rectify)
    {
        st = agc_integ_rectify(ip, ctx); /* FAZAB2   */
    }

    agc_phase_change(2, 04022); /* FAZAB3   */

    if (st == AGC_STATUS_OK)
    {
        st = csm ? agc_integ_atopcsm(ip, ctx, ctx->pbody) :
             agc_integ_atoplem(ip, ctx, ctx->pbody);
    }

    if (inc->dim9)
    {
        agc_erase_set_x789(sv.x789); /* FAZAB4   */
    }

    agc_status_t sw =
        agc_integ_area_busy(0) ? agc_integ_intwake() : AGC_STATUS_OK;

    return ((st == AGC_STATUS_OK) ? sw : st);
}

/* BVECTORS (P20-P25 CM p.576): USTAR = UNIT(USTAR x UNIT(RCLP)) = B0,
 * DELTAQ = (ACOS(USTAR . UM) - 1/4) PI/4 |RCLP| normalized (SRR* 0-3,1),
 * B1 = B2 = 0. rclp body scale, um and ustar half unit.                 */
#define RZ_PI_4 ((agc_dp_t) 210828714) /* PI/4.0 2DEC .785398164     */

void rz_bvectors(agc_interp_t *ip, agc_vec_t rclp, agc_vec_t um,
                 agc_vec_t *ustar, agc_incorp_t *inc)
{
    if (!AGC_ASSERT(ip != NULL && ustar != NULL) || !AGC_ASSERT(inc != NULL))
    {
        return;
    }

    agc_dp_t  absr = 0;
    agc_vec_t u    = agc_vec_unit(ip, rclp, &absr, NULL); /* 26D      */

    inc->bvector[0] =
        agc_vec_unit(ip, agc_vec_cross(ip, *ustar, u), NULL, NULL);
    agc_phase_change(2, 04022); /* GRP2PC   */
    *ustar = inc->bvector[0];
    agc_dp_t a =
        agc_dp_acos(ip, agc_dp_shl(ip, agc_vec_dot(ip, *ustar, um), 1));
    int      x1 = 0;
    agc_dp_t n  = rz_norm(agc_dp_sub(ip, a, AGC_DP_QUARTER), &x1);
    agc_tp_t t  = agc_dp_mul_tp(agc_dp_mul_trunc(n, RZ_PI_4), absr);

    inc->deltaq =
        agc_tp_round(ip, agc_tp_shift(ip, t, -(-3 - x1))); /* SRR* 0-3,1 */
    inc->bvector[1] = agc_vec_zero();
    inc->bvector[2] = agc_vec_zero();
}

uint16_t rz_word_of(int v)
{
    if (!AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX))
    {
        return (0U);
    }

    return ((v >= 0) ? (uint16_t) v : (uint16_t) (077777 + v));
}

/* THISPREC / OTHPREC: CSMPREC or LEMPREC by vehicle. */
static rz_integ_kind_t rz_resolve_kind(rz_integ_kind_t kind)
{
    bool csm = rz_is_csm();

    if (!AGC_ASSERT((int) kind >= 0 && kind < RZ_INTEG_KIND_COUNT))
    {
        return (csm ? RZ_CSMPREC : RZ_LEMPREC); /* THISPREC */
    }

    if (kind == RZ_THISPREC)
    {
        return (csm ? RZ_CSMPREC : RZ_LEMPREC);
    }

    if (kind == RZ_OTHPREC)
    {
        return (csm ? RZ_LEMPREC : RZ_CSMPREC);
    }

    return (kind);
}

/* CSMPREC LEMPREC CSMCONIC LEMCONIC THISPREC OTHPREC
 * (INTEGRATION_INITIALIZATION p.1318): permanent state to TDEC1. */
agc_status_t rz_integ(agc_interp_t *ip, rz_integ_kind_t kind, agc_dp_t tdec1,
                      agc_job_id_t resume, agc_integ_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && out != NULL) ||
        !AGC_ASSERT((int) kind >= 0 && kind < RZ_INTEG_KIND_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();
    agc_status_t     st;

    switch (rz_resolve_kind(kind))
    {
        case RZ_CSMPREC:
            st = agc_integ_csmprec(ip, ctx, tdec1, resume);
            break;
        case RZ_LEMPREC:
            st = agc_integ_lemprec(ip, ctx, tdec1, resume);
            break;
        case RZ_CSMCONIC:
            st = agc_integ_csmconic(ip, ctx, tdec1, resume);
            break;
        case RZ_LEMCONIC:
            st = agc_integ_lemconic(ip, ctx, tdec1, resume);
            break;
        case RZ_THISPREC:
        case RZ_OTHPREC:
        case RZ_INTEG_KIND_COUNT:
        default:
            st = AGC_STATUS_BAD_ARG;
            break;
    }

    if (st == AGC_STATUS_OK)
    {
        *out = ctx->out;
    }

    return (st);
}

/* INTSTALL, then the INTEGRVS set-up common to INTINT, INITVEL, R31. */
agc_status_t rz_integrvs(agc_interp_t *ip, const rz_integrvs_in_t *in,
                         agc_job_id_t resume, agc_integ_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL) || !AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_integ_install(0, resume); /* INTSTALL */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();

    rz_flag_put(AGC_FLAG_INTYPFLG, in->conic);
    rz_flag_put(AGC_FLAG_MOONFLAG, in->moon);
    ctx->sv.rcv = in->rcv;
    ctx->sv.vcv = in->vcv;
    ctx->sv.tet = in->tet;
    st          = agc_integ_integrvs(ip, ctx, in->tdec1);

    if (st == AGC_STATUS_OK)
    {
        *out = ctx->out;
    }

    return (st);
}

/* PHASCHNG / 2PHSCHNG with a type C code whose restart point is job
 * `job` at priority `prio` (the AGC's OCT 05022 / OCT 26000 pair: the
 * restart runs the job by FINDVAC). code_a = 0: a single PHASCHNG.     */
void rz_phase_job(uint16_t code_a, uint16_t code_c, agc_job_id_t job,
                  int16_t prio)
{
    if (!AGC_ASSERT(job != AGC_JOB_NONE))
    {
        return;
    }

    if (!AGC_ASSERT(job != AGC_JOB_NONE && prio > 0))
    {
        return;
    }

    agc_phase_var_t v;

    v.kind       = AGC_RESTART_JOB;
    v.job        = job;
    v.task       = AGC_TASK_NONE;
    v.prio_or_dt = prio;
    v.immediate  = false;
    v.long_dt_cs = 0U;

    if (code_a == 0U)
    {
        agc_phase_change_var(code_c, &v);
    }
    else
    {
        agc_phase_change2(code_a, code_c, &v);
    }
}
