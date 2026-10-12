/**********************************************************************
 * FILE NAME: common.c
 *
 * PURPOSE:
 *   The rendezvous erasables and the shared rendezvous subroutines of
 *   P34-35_P74-75.agc: SELECTMU, VECSHIFT, SHIFTR1, PERIAPO/PERIAPO1/
 *   SETRAD, PRECSET/ACTIVE/PASSIVE, INTINT, S34/35.1, LOMAT, MIDGIM
 *   (GET.LVC, GET+MGA), AVFLAGA/P, P20FLGON; the navigation erasables
 *   the noun tables show (MRKBUF1/2, VHFCNT, TANG, RSTACK, LANDMARK,
 *   HORIZON, CPHIX, PRAXIS, X789, the W pad loads); INCORP2 and BVECTORS
 *   for the navigation routines; plus the numeric helpers (UNIT, ABVAL,
 *   NORM, SL*, SR*) used by every rendezvous file. Owning the noun
 *   storage here keeps the programs layer from reaching up into
 *   src/lm/radar and src/csm/navigation.
 *
 * AGC SOURCE:
 *   Comanche055 P34-35_P74-75.agc pp. 469-500 (SELECTMU p.494, PERIAPO
 *   p.496, SETRAD p.497, PRECSET p.498, VECSHIFT p.499, SHIFTR1 p.500,
 *   INTINT LOMAT p.479, MIDGIM p.493, S34/35.1 p.474), P30-P37.agc p.648
 *   (AVFLAGA, AVFLAGP, P20FLGON), P20-P25.agc p.576 (BVECTORS),
 *   MEASUREMENT_INCORPORATION.agc p.1256 (INCORP2),
 *   INTEGRATION_INITIALIZATION.agc pp.1318-1324 (CSMPREC .. INTSTALL)
 *   Luminary099 P34-35_P74-75.agc pp. 672-699 (same subroutines),
 *   P32-P35_P72-P75.agc p.629, MEASUREMENT_INCORPORATION.agc p.1153
 *
 * EXTERNAL VARIABLES:
 *   Name       Type        I/O  Scale      Description
 *   ---------  ----------  ---  ---------  ----------------------------
 *   (rendezvous erasables) agc_rendz_t I/O  see rendezvous.h
 *   MRKBUF1 MRKBUF2  words  I/O  -         CM mark buffers
 *   VHFCNT     agc_sp_t    I/O  -          CM VHF range mark count
 *   TANG RSTACK  sp / dp   I/O  CDU, radar LM radar nouns 66 67 72 78
 *   LANDMARK HORIZON CPHIX PRAXIS words I/O CDU  CM nouns 70 71 95 96
 *   X789       agc_vec_t   I/O  bias scales  radar / landmark biases
 *   AVFLAG MOONFLAG INTYPFLG TRACKFLG UPDATFLG MGLVFLAG RVSW  flags  O
 *   ECSTEER    agc_dp_t    O    -          CM AVFLAGA: BIT13
 *   RLS        agc_vec_t   I    m 2^27     SETRAD about the moon
 *
 * EXTERNAL REFERENCES:
 *   Name                     Description
 *   -----------------------  -------------------------------------------
 *   agc_integ_csmprec ..     CSMPREC LEMPREC CSMCONIC LEMCONIC
 *   agc_integ_install        INTSTALL
 *   agc_integ_integrvs       INTEGRVS
 *   agc_conic_timethet       TIMETHET
 *   agc_conic_apsides        APSIDES
 *   agc_incorp_update_w      INCORP2 W update
 *   agc_incorp_update_state  INCORP2 state update
 *   agc_phase_change2        PHASCHNG / 2PHSCHNG
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. Functions return AGC_STATUS_BAD_ARG (or a zero value)
 *   for NULL pointers and out-of-range indices; integration statuses
 *   (INTSTALL sleep) are passed back.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Called from rendezvous and navigation jobs. The "this vehicle" /
 *   "other vehicle" choices follow agc_state_vehicle().
 *
 * NOTES:
 *   Scales (from the AGC text): r m 2^29, v m/cs 2^7, t cs 2^28, angles
 *   revolutions 2^0, unit vectors 2^1, mu 2^36. "Body scale" is 2^27 /
 *   2^5 when RTX2 = 2 (moon). NORM leaves -(left shifts) in the index
 *   register, so a normalized value of scale 2^S is at 2^(S + X).
 **********************************************************************/
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"
#include "internal.h"

/* The rendezvous erasables (E-bank storage of ERASABLE_ASSIGNMENTS). */
static agc_rendz_t rendz_erasable; /* rendezvous erasables */

/* MUTABLE (CONIC_SUBROUTINES p.1307, LM CONTROLLED_CONSTANTS):
 * MUE   2DEC* 3.986032 E10 B-36*    -> 155704375
 * 1/SQRT(MUE) .50087529 E-5 B+17*   -> 176229826
 * MUM   2DEC 4.902778 E8 B-30       -> 122569450
 * 1/SQRT(MUM) .45162595 E-4 B+14*   -> 198627193 */
static const agc_dp_t mutable_mue       = 155704375; /* MUE, 2^36 */
static const agc_dp_t mutable_rtmue_inv = 176229826; /* 1/SQRT(MUE), 2^17 */
static const agc_dp_t mutable_mum       = 122569450; /* MUM, 2^30 */
static const agc_dp_t mutable_rtmum_inv = 198627193; /* 1/SQRT(MUM), 2^14 */

/**********************************************************************
 * MUTABLE entry for the primary body: MUE 2^36 (earth) or MUM 2^30 (moon),
 * i.e. DLOAD* MUTABLE -2,1 with X1 = RTX1.
 * AGC: MUTABLE (CONIC_SUBROUTINES.agc p.1307)
 *
 * FUNCTION NAME: rz_mutable_mu
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   moon      bool  I    true: MUM, false: MUE
 *
 * RETURNS: agc_dp_t -- MUE (m^3/cs^2 2^36) or MUM (2^30)
 **********************************************************************/
agc_dp_t rz_mutable_mu(bool moon)
{
    agc_dp_t mu = moon ? mutable_mum : mutable_mue; /* 2^30 / 2^36 */

    return (mu);
}

/**********************************************************************
 * The rendezvous erasables (E-bank storage of ERASABLE_ASSIGNMENTS).
 * AGC: ERASABLE_ASSIGNMENTS.agc
 *
 * FUNCTION NAME: agc_rendz_erasable
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_rendz_t * -- the rendezvous erasables (never NULL)
 **********************************************************************/
agc_rendz_t *agc_rendz_erasable(void)
{
    return (&rendz_erasable);
}

/**********************************************************************
 * VHFCNT (CM): VHF range mark count.
 * AGC: VHFCNT (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_vhfcnt
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- VHFCNT, a count
 **********************************************************************/
agc_sp_t agc_erase_vhfcnt(void)
{
    return (rendz_erasable.vhfcnt);
}

/**********************************************************************
 * Store VHFCNT (CM): VHF range mark count.
 * AGC: VHFCNT (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_set_vhfcnt
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   v         agc_sp_t  I    the count
 **********************************************************************/
void agc_erase_set_vhfcnt(agc_sp_t v)
{
    rendz_erasable.vhfcnt = v;
}

/* MRKBUF1, MRKBUF2 (CM, EBANK= MRKBUF1): the two seven-word mark buffers
 * (time DP, CDUY, CDUS, CDUZ, CDUT, CDUX), raw words; word 0 = -1 (77776)
 * marks a buffer empty. SXTMARK fills MRKBUF1 (and moves it to MRKBUF2
 * for R21); R22, P76 and the verbs read and clear them.                */
#define RZ_MRKBUF_WORDS 7                   /* words per buffer */
static uint16_t mrkbuf[2][RZ_MRKBUF_WORDS]; /* MRKBUF1, MRKBUF2 */

/**********************************************************************
 * One raw word of MRKBUF1 / MRKBUF2 (layout above).
 * AGC: MRKBUF1, MRKBUF2 (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_mrkbuf
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   buf       int   I    buffer, 1 or 2
 *   index     int   I    word, 0 .. 6
 *
 * RETURNS: uint16_t -- the raw 15-bit word (0 for a bad index)
 **********************************************************************/
uint16_t agc_erase_mrkbuf(int buf, int index)
{
    return (mrkbuf[buf - 1][index]);
}

/**********************************************************************
 * Store one word of MRKBUF1 / MRKBUF2 (raw, 15 bits kept).
 * AGC: MRKBUF1, MRKBUF2 (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_set_mrkbuf
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   buf       int       I    buffer, 1 or 2
 *   index     int       I    word, 0 .. 6
 *   v         uint16_t  I    the raw word
 **********************************************************************/
void agc_erase_set_mrkbuf(int buf, int index, uint16_t v)
{
    mrkbuf[buf - 1][index] = (uint16_t) (v & 077777u);
}

/**********************************************************************
 * MRKBUF2 +index as an SP number (P76 ENDP76, R22: -1 = invalid).
 * AGC: MRKBUF2 (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_mrkbuf2
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   index     int   I    word index
 *
 * RETURNS: agc_sp_t -- MRKBUF2 +index as a signed SP number
 **********************************************************************/
agc_sp_t agc_erase_mrkbuf2(int index)
{
    return ((agc_sp_t) rz_word_value(agc_erase_mrkbuf(2, index)));
}

/**********************************************************************
 * Store MRKBUF2 +index from a signed SP number (ones' complement word).
 * AGC: MRKBUF2 (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_set_mrkbuf2
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   index     int       I    word index
 *   v         agc_sp_t  I    the SP number
 **********************************************************************/
void agc_erase_set_mrkbuf2(int index, agc_sp_t v)
{
    uint16_t w = (v >= 0) ? (uint16_t) v : (uint16_t) (077777 + v); /* 1s compl. */

    agc_erase_set_mrkbuf(2, index, w);
}

/* TANG, TANG +1 (LM, noun 72): RR gimbal command or, for RRDESNB, the
 * desired trunnion and shaft CDU words, as ones' complement SP numbers.
 * RSTACK (LM, R04/R77 nouns 66, 67, 78): four DP radar samples, m or
 * m/cs at the radar scale. Owned here so the noun tables (this layer)
 * need not reach up into src/lm/radar; the radar code uses the accessors. */
#define RZ_TANG_WORDS   2                /* TANG, TANG +1 */
#define RZ_RSTACK_WORDS 4                /* four samples */
static agc_sp_t tang[RZ_TANG_WORDS];     /* TANG, CDU words, unscaled */
static agc_dp_t rstack[RZ_RSTACK_WORDS]; /* RSTACK, radar scale */

/**********************************************************************
 * TANG, TANG +1 (LM, noun 72): RR gimbal command or, for RRDESNB, the
 * desired trunnion and shaft CDU words, as ones' complement SP numbers.
 * AGC: TANG (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_tang
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   index     int   I    0 (trunnion) or 1 (shaft)
 *
 * RETURNS: agc_sp_t -- the CDU word as an SP number (0 for a bad index)
 **********************************************************************/
agc_sp_t agc_erase_tang(int index)
{
    return (tang[index]);
}

/**********************************************************************
 * Store TANG +index (LM).
 * AGC: TANG (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_set_tang
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   index     int       I    0 (trunnion) or 1 (shaft)
 *   v         agc_sp_t  I    the CDU word as an SP number
 **********************************************************************/
void agc_erase_set_tang(int index, agc_sp_t v)
{
    tang[index] = v;
}

/**********************************************************************
 * RSTACK (LM, R04/R77 nouns 66, 67, 78): four DP radar samples, m or m/cs
 * at the radar scale.
 * AGC: RSTACK (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_rstack
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   index     int   I    sample, 0 .. 3
 *
 * RETURNS: agc_dp_t -- the sample at the radar scale (0 for a bad index)
 **********************************************************************/
agc_dp_t agc_erase_rstack(int index)
{
    return (rstack[index]);
}

/**********************************************************************
 * Store RSTACK +index (LM radar sample).
 * AGC: RSTACK (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_set_rstack
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   index     int       I    sample, 0 .. 3
 *   v         agc_dp_t  I    the sample at the radar scale
 **********************************************************************/
void agc_erase_set_rstack(int index, agc_dp_t v)
{
    if (!AGC_ASSERT(index >= 0 && index < RZ_RSTACK_WORDS) || !AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX))
    {
        return;
    }

    rstack[index] = v;
}

/* LANDMARK (CM P22 code ABCDE, V52), HORIZON (CM noun 70/71 R3) and
 * the CM R63 results CPHIX (noun 96, X axis) and PRAXIS (noun 95,
 * preferred axis), CDU words. Owned here, with the noun tables, so this
 * layer does not reach up into src/csm/navigation.                     */
static uint16_t landmark;         /* LANDMARK code */
static uint16_t horizon;          /* HORIZON word */
static uint16_t trk_angles[2][3]; /* [0] CPHIX, [1] PRAXIS               */

/**********************************************************************
 * LANDMARK (CM P22 code ABCDE, V52) as a signed SP number.
 * AGC: LANDMARK (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_landmark
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the code (0 when the word is out of range)
 **********************************************************************/
agc_sp_t agc_erase_landmark(void)
{
    return ((agc_sp_t) rz_word_value(landmark));
}

/**********************************************************************
 * Store LANDMARK (CM) from a signed SP number.
 * AGC: LANDMARK (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_set_landmark
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   v         agc_sp_t  I    the code
 **********************************************************************/
void agc_erase_set_landmark(agc_sp_t v)
{
    landmark = (v >= 0) ? (uint16_t) v : (uint16_t) (077777 + v); /* 1s compl. */
}

/**********************************************************************
 * HORIZON (CM noun 70/71 R3), raw word.
 * AGC: HORIZON (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_horizon
 *
 * ARGUMENTS: none
 *
 * RETURNS: uint16_t -- the raw 15-bit word
 **********************************************************************/
uint16_t agc_erase_horizon(void)
{
    return (horizon);
}

/**********************************************************************
 * Store HORIZON (CM), raw word.
 * AGC: HORIZON (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_set_horizon
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   v         uint16_t  I    the raw word
 **********************************************************************/
void agc_erase_set_horizon(uint16_t v)
{
    horizon = (uint16_t) (v & 077777u);
}

/**********************************************************************
 * The CM R63 results CPHIX (noun 96, X axis) and PRAXIS (noun 95, preferred
 * axis), CDU words.
 * AGC: CPHIX, PRAXIS (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_rendz_tracking_angles
 *
 * ARGUMENTS:
 *   Argument   Type  I/O  Description (scale)
 *   ---------  ----  ---  -----------------------------------------
 *   preferred  bool  I    true: PRAXIS, false: CPHIX
 *   axis       int   I    0 .. 2 (outer, inner, middle)
 *
 * RETURNS: uint16_t -- the CDU word (0 for a bad axis)
 **********************************************************************/
uint16_t agc_rendz_tracking_angles(bool preferred, int axis)
{
    return (trk_angles[preferred ? 1 : 0][axis]); /* PRAXIS / CPHIX */
}

/**********************************************************************
 * The CPHIX or PRAXIS words for R63 to fill.
 * AGC: CPHIX, PRAXIS (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: rz_tracking_angles
 *
 * ARGUMENTS:
 *   Argument   Type  I/O  Description (scale)
 *   ---------  ----  ---  -----------------------------------------
 *   preferred  bool  I    true: PRAXIS, false: CPHIX
 *
 * RETURNS: uint16_t * -- the three words (never NULL)
 **********************************************************************/
uint16_t *rz_tracking_angles(bool preferred)
{
    uint16_t *p = trk_angles[preferred ? 1 : 0]; /* PRAXIS / CPHIX */

    if (!AGC_ASSERT(p != NULL))
    {
        return (trk_angles[0]);
    }

    return (p);
}

/**********************************************************************
 * Fresh start: all rendezvous erasables zero, earth selected.
 * AGC: none (fresh start; SELECTMU p.494 for the earth)
 *
 * FUNCTION NAME: agc_rendz_reset
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   rz        agc_rendz_t *  I/O  rendezvous erasables
 **********************************************************************/
void agc_rendz_reset(agc_rendz_t *rz)
{
    static const agc_rendz_t zero; /* all zero */

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

/**********************************************************************
 * vehicle-dependent flags
 * AGC: none (the two ropes; agc_state_vehicle)
 *
 * FUNCTION NAME: rz_is_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true in the CM build, false in the LM build
 **********************************************************************/
bool rz_is_csm(void)
{
    agc_vehicle_t v = agc_state_vehicle(); /* this rope */

    return (v == AGC_VEHICLE_CSM);
}

/**********************************************************************
 * The flag of this vehicle for a vehicle-dependent rendezvous flag (0 when
 * the vehicle has none).
 * AGC: FLAGWRD assignments (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: rz_vflag
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   f         rz_vflag_t  I    vehicle flag
 *
 * RETURNS: agc_flag_id_t -- the flag id, or 0 when this vehicle has no such
 *          flag
 **********************************************************************/
agc_flag_id_t rz_vflag(rz_vflag_t f)
{
    return (rz_is_csm() ? rz_vflag_csm(f) : rz_vflag_lm(f)); /* by rope */
}

/**********************************************************************
 * A flag that does not exist in this vehicle reads as clear.
 * AGC: BON / BOFF on the vehicle's flag (INTERPRETER.agc)
 *
 * FUNCTION NAME: rz_vflag_test
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   f         rz_vflag_t  I    vehicle flag
 *
 * RETURNS: bool -- true when the flag exists in this vehicle and is set
 **********************************************************************/
bool rz_vflag_test(rz_vflag_t f)
{
    agc_flag_id_t id = rz_vflag(f); /* vehicle flag id */

    return ((id != 0U) && agc_flag_test(id));
}

/**********************************************************************
 * SET / CLEAR by a run-time choice (cannot fail for a valid flag id).
 * AGC: SET / CLEAR (INTERPRETER.agc)
 *
 * FUNCTION NAME: rz_flag_put
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   f         agc_flag_id_t  I    flag id (not 0)
 *   on        bool           I    true: SET, false: CLEAR
 **********************************************************************/
void rz_flag_put(agc_flag_id_t f, bool on)
{
    if (on)
    {
        agc_flag_set(f);
    }
    else
    {
        agc_flag_clear(f);
    }
}

/**********************************************************************
 * SET / CLEAR a vehicle-dependent rendezvous flag (it must exist in this
 * vehicle).
 * AGC: SET / CLEAR on the vehicle's flag (INTERPRETER.agc)
 *
 * FUNCTION NAME: rz_vflag_set
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   f         rz_vflag_t  I    vehicle flag
 *   on        bool        I    true: SET, false: CLEAR
 **********************************************************************/
void rz_vflag_set(rz_vflag_t f, bool on)
{
    agc_flag_id_t id = rz_vflag(f); /* vehicle flag id */

    if (!AGC_ASSERT(id != 0U))
    {
        return; /* not a flag of this vehicle */
    }

    rz_flag_put(id, on);
}

/**********************************************************************
 * MOONTHIS EQUALS CMOONFLG (CM) / LMOONFLG (LM), TAGS_FOR_RELATIVE_SETLOC
 * AGC: MOONTHIS (TAGS_FOR_RELATIVE_SETLOC.agc)
 *
 * FUNCTION NAME: rz_moonthis
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- CMOONFLG (CM) or LMOONFLG (LM)
 **********************************************************************/
bool rz_moonthis(void)
{
    agc_flag_id_t f = rz_is_csm() ? AGC_FLAG_CMOONFLG : AGC_FLAG_LMOONFLG; /* MOONTHIS */

    if (!AGC_ASSERT(f != 0U))
    {
        return (false);
    }

    return (agc_flag_test(f));
}

/* ---- numeric helpers ---------------------------------------------------- */
/**********************************************************************
 * UNIT: half-unit vector (scale 2^1)
 * AGC: UNIT (INTERPRETER.agc p.1175)
 *
 * FUNCTION NAME: rz_unit
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   v         agc_vec_t       I    vector at any scale
 *
 * RETURNS: agc_vec_t -- the half-unit vector, 2^1 (zero for a bad argument)
 **********************************************************************/
agc_vec_t rz_unit(agc_interp_t *ip, agc_vec_t v)
{
    if (!AGC_ASSERT(v.c[0] != AGC_DP_MIN))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    return (agc_vec_unit(ip, v, NULL, NULL));
}

/**********************************************************************
 * ABVAL
 * AGC: ABVAL (INTERPRETER.agc p.1180)
 *
 * FUNCTION NAME: rz_abval
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   v         agc_vec_t       I    vector
 *
 * RETURNS: agc_dp_t -- |V| at the scale of V
 **********************************************************************/
agc_dp_t rz_abval(agc_interp_t *ip, agc_vec_t v)
{
    agc_dp_t a = agc_vec_abval(ip, v); /* |V|, V's scale */

    return (a);
}

/**********************************************************************
 * NORM when only the index register is wanted: -(left shifts), 0 for 0.
 * AGC: NORM (INTERPRETER.agc), index register only
 *
 * FUNCTION NAME: rz_norm_xreg
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   a         agc_dp_t  I    the value
 *
 * RETURNS: int -- -(left shifts), 0 for 0
 **********************************************************************/
int rz_norm_xreg(agc_dp_t a)
{
    if (a == 0)
    {
        return (0);
    }

    const int n = agc_dp_norm_count(a); /* left shifts */

    if (!AGC_ASSERT(n >= 0 && n <= AGC_DP_BITS))
    {
        return (0); /* no shift */
    }

    return (-n);
}

/**********************************************************************
 * NORM X: left-justify A; *xreg = the AGC index register, -(left shifts).
 * AGC: NORM (INTERPRETER.agc)
 *
 * FUNCTION NAME: rz_norm
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   a         agc_dp_t  I    the value, scale 2^S
 *   xreg      int *     O    -(left shifts)
 *
 * RETURNS: agc_dp_t -- the normalized value, scale 2^(S + X)
 **********************************************************************/
agc_dp_t rz_norm(agc_dp_t a, int *xreg)
{
    if (a == 0)
    {
        *xreg = 0;

        return (0);
    }

    int      n = 0;                  /* left shifts */
    agc_dp_t r = agc_dp_norm(a, &n); /* normalized, 2^(S-N) */

    if (!AGC_ASSERT(n >= 0 && n <= AGC_DP_BITS))
    {
        return (a);
    }

    *xreg = -n;

    return (r);
}

/**********************************************************************
 * SL k (k > 0) or SR -k (k < 0): the AGC SL* and SR* with an index.
 * AGC: SL, SR (INTERPRETER.agc)
 *
 * FUNCTION NAME: rz_shift
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   a         agc_dp_t        I    the value
 *   k         int             I    left shift (k > 0) or right shift (k <
 *                                  0)
 *
 * RETURNS: agc_dp_t -- the shifted value (OVFIND set on overflow)
 **********************************************************************/
agc_dp_t rz_shift(agc_interp_t *ip, agc_dp_t a, int k)
{
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

/**********************************************************************
 * SL* addr,X: shift count = addr - C(X); a negative count shifts right.
 * AGC: SL* (INTERPRETER.agc)
 *
 * FUNCTION NAME: rz_sl_star
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   a         agc_dp_t        I    the value
 *   addr      int             I    the address field
 *   xreg      int             I    the index register
 *
 * RETURNS: agc_dp_t -- A shifted left by addr - C(X)
 **********************************************************************/
agc_dp_t rz_sl_star(agc_interp_t *ip, agc_dp_t a, int addr, int xreg)
{
    return (rz_shift(ip, a, addr - xreg));
}

/**********************************************************************
 * SR* addr,X: shift right by addr - C(X); a negative count shifts left.
 * AGC: SR* (INTERPRETER.agc)
 *
 * FUNCTION NAME: rz_sr_star
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   a         agc_dp_t        I    the value
 *   addr      int             I    the address field
 *   xreg      int             I    the index register
 *
 * RETURNS: agc_dp_t -- A shifted right by addr - C(X)
 **********************************************************************/
agc_dp_t rz_sr_star(agc_interp_t *ip, agc_dp_t a, int addr, int xreg)
{
    return (rz_shift(ip, a, xreg - addr));
}

/**********************************************************************
 * VSL k (k > 0) or VSR -k (k < 0).
 * AGC: VSL, VSR (INTERPRETER.agc)
 *
 * FUNCTION NAME: rz_vshift
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   v         agc_vec_t       I    the vector
 *   k         int             I    left shift (k > 0) or right shift (k <
 *                                  0)
 *
 * RETURNS: agc_vec_t -- the shifted vector
 **********************************************************************/
agc_vec_t rz_vshift(agc_interp_t *ip, agc_vec_t v, int k)
{
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

/**********************************************************************
 * VECSHIFT / VSR* 0,2 (P34-35 p.499): earth scale -> body scale. With
 * X2 = RTX2 = 2 (moon) the count 0 - X2 is negative: a left shift by 2.
 * AGC: VECSHIFT (P34-35_P74-75.agc p.499)
 *
 * FUNCTION NAME: agc_rendz_vec_to_body
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   ip        agc_interp_t *       I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *  I    rendezvous erasables
 *   v         agc_vec_t            I    vector at the earth scale
 *
 * RETURNS: agc_vec_t -- V at the body scale
 **********************************************************************/
agc_vec_t agc_rendz_vec_to_body(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t v)
{
    return (rz_vshift(ip, v, rz->body.rtx2));
}

/**********************************************************************
 * VSL* 0,2: body scale -> earth scale (moon: shift right 2).
 * AGC: "VSL* 0,2" (P34-35_P74-75.agc)
 *
 * FUNCTION NAME: rz_vec_from_body
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   ip        agc_interp_t *       I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *  I    rendezvous erasables
 *   v         agc_vec_t            I    vector at the body scale
 *
 * RETURNS: agc_vec_t -- V at the earth scale
 **********************************************************************/
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

/**********************************************************************
 * SHIFTR1 (P34-35 p.500): SL* 0,2 -- a body-scale scalar to 2^29.
 * AGC: SHIFTR1 (P34-35_P74-75.agc p.500)
 *
 * FUNCTION NAME: agc_rendz_scalar_from_body
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   rz        const agc_rendz_t *  I    rendezvous erasables
 *   a         agc_dp_t             I    scalar at the body scale
 *
 * RETURNS: agc_dp_t -- A at the earth scale (m 2^29)
 **********************************************************************/
agc_dp_t agc_rendz_scalar_from_body(const agc_rendz_t *rz, agc_dp_t a)
{
    return (agc_dp_shr(a, rz->body.rtx2));
}

/**********************************************************************
 * SELECTMU (P34-35 p.494)
 * Choose mu and the scale indices for the primary body of the CSM state
 * (CMOONFLG). RTMU is always at 2^36 (the lunar MUM, 2^30, is shifted
 * right 6). The AGC then clears FINALFLG and goes to VN1645; the program
 * drivers do the display (rz_ttogo_display) themselves.
 * AGC: SELECTMU (P34-35_P74-75.agc p.494)
 *
 * FUNCTION NAME: agc_rendz_selectmu
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   rz        agc_rendz_t *  I/O  rendezvous erasables
 *   cmoonflg  bool           I    CMOONFLG: the CSM state is moon-centred
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a NULL
 *          pointer
 **********************************************************************/
agc_status_t agc_rendz_selectmu(agc_rendz_t *rz, bool cmoonflg)
{
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

/**********************************************************************
 * AVFLAGA / AVFLAGP / P20FLGON (CM P30-P37 p.~, LM P32-P35)
 * AVFLAG reset = CSM active, set = LM active. "This vehicle active" is
 * AVFLAGA. The CM AVFLAGA also sets ECSTEER = 1 (BIT13).
 * AGC: AVFLAGA, AVFLAGP (P30-P37.agc p.648)
 *
 * FUNCTION NAME: rz_avflag
 *
 * ARGUMENTS:
 *   Argument     Type  I/O  Description (scale)
 *   -----------  ----  ---  ---------------------------------------
 *   this_active  bool  I    true: this vehicle is the active one
 **********************************************************************/
void rz_avflag(bool this_active)
{
    bool csm       = rz_is_csm();                      /* CM build */
    bool lm_active = csm ? !this_active : this_active; /* AVFLAG */

    rz_flag_put(AGC_FLAG_AVFLAG, lm_active);

    if (csm && this_active)
    {
        /* CAF BIT13, TS ECSTEER: "SET ECSTEER = 1" (high word only) */
        agc_burn_t *b = &agc_guid_state()->burn; /* burn erasables */

        b->ecsteer = agc_dp_pack((agc_sp_t) 010000, agc_dp_low(b->ecsteer));
    }

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_AVFLAG) == lm_active))
    {
        return;
    }
}

/**********************************************************************
 * P20FLGON: SET TRACKFLG, SET UPDATFLG.
 * AGC: P20FLGON (P30-P37.agc p.648)
 *
 * FUNCTION NAME: rz_tracking_flags_on
 *
 * ARGUMENTS: none
 **********************************************************************/
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

/**********************************************************************
 * PRECSET / LEMSTORE / CSMSTORE / ACTIVE / PASSIVE (p.498)
 * AGC: ACTIVE (P34-35_P74-75.agc p.498)
 *
 * FUNCTION NAME: rz_store_active
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   rz        agc_rendz_t *  I/O  rendezvous erasables
 *   r         agc_vec_t      I    position, m 2^29
 *   v         agc_vec_t      I    velocity, m/cs 2^7
 **********************************************************************/
void rz_store_active(agc_rendz_t *rz, agc_vec_t r, agc_vec_t v)
{
    rz->ract3 = r; /* ACTIVE: STOVL RACT3        */
    rz->vact3 = v; /*         STORE VACT3        */
}

/**********************************************************************
 * PASSIVE: RPASS3 = R, VPASS3 = V.
 * AGC: PASSIVE (P34-35_P74-75.agc p.498)
 *
 * FUNCTION NAME: rz_store_passive
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   rz        agc_rendz_t *  I/O  rendezvous erasables
 *   r         agc_vec_t      I    position, m 2^29
 *   v         agc_vec_t      I    velocity, m/cs 2^7
 **********************************************************************/
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

/**********************************************************************
 * PRECSET: TDEC2 = TDEC1; LEMPREC, LEMSTORE (AVFLAG set = LM active);
 * TDEC1 = TDEC2; CSMPREC, CSMSTORE. RATT/VATT are at earth scale.
 * AGC: PRECSET, LEMSTORE, CSMSTORE (P34-35_P74-75.agc p.498)
 *
 * FUNCTION NAME: rz_precset
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   tdec1     agc_dp_t        I    TDEC1, cs 2^28
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the integration status (INTSTALL sleep)
 **********************************************************************/
agc_status_t rz_precset(agc_interp_t *ip, agc_rendz_t *rz, agc_dp_t tdec1, agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(tdec1 >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    bool            lm_active = agc_flag_test(AGC_FLAG_AVFLAG); /* AVFLAG */
    agc_integ_out_t o;                                          /* integration result */

    rz->tdec2       = tdec1;                                       /* TDEC2   */
    agc_status_t st = rz_integ(ip, RZ_LEMPREC, tdec1, resume, &o); /* LEMPREC */

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
} /* rz_precset */

/**********************************************************************
 * INTINT (p.479)
 * Push list on entry: V, R, T1, T2, conic indicator (non-zero = conic).
 * INTSTALL; INTYPFLG per indicator; TDEC1 = T2; MOONFLAG = CMOONFLG;
 * TET = T1; RCV, VCV = VSR* 0,2 (body scale); INTEGRVS; RATT, VATT out.
 * GOINT (p.479) is INTINT with T2 = TTPI + NOMTPI... called by S33/34.1.
 * AGC: INTINT (P34-35_P74-75.agc p.479)
 *
 * FUNCTION NAME: rz_intint
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   ip        agc_interp_t *       I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *  I    rendezvous erasables
 *   r         agc_vec_t            I    position at T1, m 2^29
 *   v         agc_vec_t            I    velocity at T1, m/cs 2^7
 *   t1        agc_dp_t             I    time of the state, cs 2^28
 *   t2        agc_dp_t             I    time to integrate to, cs 2^28
 *   conic     bool                 I    true: conic, false: precision
 *   resume    agc_job_id_t         I    job that continues the caller
 *   ratt      agc_vec_t *          O    position at T2, m 2^29
 *   vatt      agc_vec_t *          O    velocity at T2, m/cs 2^7
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the integration status
 **********************************************************************/
agc_status_t rz_intint(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t r, agc_vec_t v, agc_dp_t t1, agc_dp_t t2, bool conic, agc_job_id_t resume, agc_vec_t *ratt, agc_vec_t *vatt)
{
    rz_integrvs_in_t in; /* INTEGRVS inputs */

    in.conic           = conic;
    in.tdec1           = t2;
    in.moon            = agc_flag_test(AGC_FLAG_CMOONFLG);
    in.tet             = t1;
    in.rcv             = agc_rendz_vec_to_body(ip, rz, r);
    in.vcv             = agc_rendz_vec_to_body(ip, rz, v);
    agc_integ_out_t o;                                     /* integration result */
    agc_status_t    st = rz_integrvs(ip, &in, resume, &o); /* INTEGRVS */

    if (st == AGC_STATUS_OK)
    {
        *ratt = o.ratt;
        *vatt = o.vatt;
    }

    return (st);
}

/**********************************************************************
 * TIMETHET call set-up used throughout P32-P39 (VECSHIFT, X1 = RTX1).
 * AGC: VECSHIFT, TIMETHET (CONIC_SUBROUTINES.agc p.1284)
 *
 * FUNCTION NAME: rz_timethet_rv
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ------------------------
 *   ip        agc_interp_t *          I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *     I    rendezvous erasables
 *   r         agc_vec_t               I    position, m 2^29
 *   v         agc_vec_t               I    velocity, m/cs 2^7
 *   snth      agc_dp_t                I    SNTH, 2^1
 *   csth      agc_dp_t                I    CSTH, 2^1
 *   rvsw      bool                    I    RVSW: time only
 *   out       agc_conic_time_out_t *  O    TIMETHET result
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the TIMETHET status
 **********************************************************************/
agc_status_t rz_timethet_rv(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t r, agc_vec_t v, agc_dp_t snth, agc_dp_t csth, bool rvsw, agc_conic_time_out_t *out)
{
    agc_timetheta_in_t in; /* TIMETHET inputs */

    in.rvec = agc_rendz_vec_to_body(ip, rz, r);
    in.vvec = agc_rendz_vec_to_body(ip, rz, v);
    in.snth = snth;
    in.csth = csth;
    in.rvsw = rvsw;
    in.body = agc_conic_body_of_x1(rz->body.rtx1);
    rz_flag_put(AGC_FLAG_RVSW, rvsw);

    return (agc_conic_timethet(ip, &in, out));
}

/**********************************************************************
 * PERIAPO1 / PERIAPO / SETRAD (p.495-497)
 * PERIAPO1 shifts the earth-scale state in the push list to body scale;
 * SETRAD gives XXXALT = RPAD (2^29) about the earth, |RLS| (2^27) about
 * the moon; APSIDES gives the apsides. Altitudes = radii - XXXALT, all at
 * body scale (callers apply SHIFTR1).
 * AGC: PERIAPO1 (P34-35_P74-75.agc p.496)
 *
 * FUNCTION NAME: agc_rendz_periapo_body_scale
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ip        agc_interp_t *         I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *    I    rendezvous erasables
 *   r         agc_vec_t              I    position, m 2^29
 *   v         agc_vec_t              I    velocity, m/cs 2^7
 *   out       agc_rendz_periapo_t *  O    apsides and altitudes, body scale
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the APSIDES status
 **********************************************************************/
agc_status_t agc_rendz_periapo_body_scale(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t r, agc_vec_t v, agc_rendz_periapo_t *out)
{
    agc_vec_t vvec = agc_rendz_vec_to_body(ip, rz, v); /* body scale */
    agc_vec_t rvec = agc_rendz_vec_to_body(ip, rz, r); /* body scale */

    return (agc_rendz_periapo(ip, rz, rvec, vvec, out));
}

/**********************************************************************
 * PERIAPO (p.496): RVEC, VVEC already at body scale.
 * AGC: PERIAPO, SETRAD (P34-35_P74-75.agc pp.496-497)
 *
 * FUNCTION NAME: agc_rendz_periapo
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ip        agc_interp_t *         I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *    I    rendezvous erasables
 *   rvec      agc_vec_t              I    position, body scale
 *   vvec      agc_vec_t              I    velocity, body scale
 *   out       agc_rendz_periapo_t *  O    apsides and altitudes, body scale
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the APSIDES status
 **********************************************************************/
agc_status_t agc_rendz_periapo(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t rvec, agc_vec_t vvec, agc_rendz_periapo_t *out)
{
    agc_conic_body_t body = agc_conic_body_of_x1(rz->body.rtx1); /* X1 */

    /* SETRAD: RPAD (2^29) about the earth, |RLS| (2^27) about the moon */
    out->xxxalt = RZ_RPAD_B29; /* SETRAD: earth RPAD */

    if (body == AGC_CONIC_MOON)
    {
        out->xxxalt = rz_abval(ip, agc_erase_rls()); /* moon |RLS| */
    }

    agc_apsides_out_t aps; /* APSIDES results */
    agc_status_t      st;  /* APSIDES */

    st = agc_conic_apsides(ip, rvec, vvec, body, &aps);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    out->rapo  = aps.rapo;                                /* 2D */
    out->rperi = aps.rperi;                               /* 6D */
    out->ecc   = aps.ecc;
    out->r1    = aps.r1;
    out->p     = aps.p;
    out->hapo  = agc_dp_sub(ip, out->rapo, out->xxxalt);  /* 4D */
    out->hperi = agc_dp_sub(ip, out->rperi, out->xxxalt); /* 8D */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * S34/35.1 (p.474): unit normal and line of sight at T3
 * AGC: S34/35.1 (P34-35_P74-75.agc p.474)
 *
 * FUNCTION NAME: agc_rendz_los_and_normal
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
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

/**********************************************************************
 * LOMAT (p.479): LV matrix rows X = Z x (-Y)... from UNRM, RACT3
 * Y = -UNRM, Z = -UNIT(RACT3), X = (Z x UNRM) VSL1. Rows at 2^1.
 * AGC: LOMAT (P34-35_P74-75.agc p.479)
 *
 * FUNCTION NAME: agc_rendz_lomat
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   ip        agc_interp_t *       I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *  I    rendezvous erasables
 *   m         agc_mat_t *          O    LV matrix rows, 2^1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a NULL
 *          pointer
 **********************************************************************/
agc_status_t agc_rendz_lomat(agc_interp_t *ip, const agc_rendz_t *rz, agc_mat_t *m)
{
    m->row[1] = agc_vec_neg(rz->unrm);
    m->row[2] = agc_vec_neg(rz_unit(ip, rz->ract3));
    m->row[0] = agc_vec_shl(ip, agc_vec_cross(ip, m->row[2], rz->unrm), 1);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * GET.LVC (MIDGIM p.493): delta V to local vertical coordinates
 * Z = U(-R), Y = U(U(-R) x V), X = U(Y x Z); DELVLVC = M dV, VSL1.
 * AGC: GET.LVC (P34-35_P74-75.agc p.493)
 *
 * FUNCTION NAME: agc_rendz_get_lvc
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rinit     agc_vec_t       I    position, m 2^29
 *   vinit     agc_vec_t       I    velocity, m/cs 2^7
 *   dv        agc_vec_t       I    delta V, reference axes, m/cs 2^7
 *   delvlvc   agc_vec_t *     O    delta V, local vertical axes, m/cs 2^7
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on a bad
 *          argument
 **********************************************************************/
agc_status_t agc_rendz_get_lvc(agc_interp_t *ip, agc_vec_t rinit, agc_vec_t vinit, agc_vec_t dv, agc_vec_t *delvlvc)
{
    agc_mat_t m; /* matrix rows, unit 2^1 */

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

/**********************************************************************
 * GET+MGA (MIDGIM p.493): +MGA = ARCSIN(UNIT(dV) . Ysm) in 0..1 rev
 * AGC: GET+MGA (P34-35_P74-75.agc p.493)
 *
 * FUNCTION NAME: agc_rendz_get_mga
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND)
 *   dv        agc_vec_t          I    delta V, reference axes, m/cs 2^7
 *   refsmmat  const agc_mat_t *  I    REFSMMAT, 2^1
 *   mga       agc_dp_t *         O    +MGA, revolutions 0 .. 1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on a bad
 *          argument
 **********************************************************************/
agc_status_t agc_rendz_get_mga(agc_interp_t *ip, agc_vec_t dv, const agc_mat_t *refsmmat, agc_dp_t *mga)
{
    /* HALFREV 2DEC 1 B-1 */
    static const agc_dp_t halfrev = AGC_DP_HALF; /* HALFREV, rev 2^0 */
    agc_dp_t              s;                     /* sin, 2^1 */

    s          = agc_dp_shl(ip, agc_vec_dot(ip, rz_unit(ip, dv), refsmmat->row[1]), 1);
    agc_dp_t a = agc_dp_asin(ip, s); /* MGA, rev 2^0 */

    if (a < 0)
    { /* convert -MGA to +MGA: add one rev */
        a = agc_dp_add(ip, agc_dp_add(ip, a, halfrev), halfrev);
    }

    *mga = a;
    agc_flag_clear(AGC_FLAG_MGLVFLAG);

    return (AGC_STATUS_OK);
}

/* ---- navigation pad loads and X789 (P20-P25 erasables, owned here) ---- */
static agc_rendz_nav_pad_t nav_pad; /* pad loads */
static agc_vec_t           x789;    /* X789: biases, bias scales */

/**********************************************************************
 * The navigation pad loads (W initial values, P20-P25 erasables).
 * AGC: WRENDPOS .. WSHAFT (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_rendz_nav_pad
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_rendz_nav_pad_t * -- the pad loads (never NULL)
 **********************************************************************/
agc_rendz_nav_pad_t *agc_rendz_nav_pad(void)
{
    return (&nav_pad);
}

/**********************************************************************
 * X789: the radar / landmark bias states of the 9-dimensional W.
 * AGC: X789 (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_x789
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_vec_t -- X789 at the bias scales
 **********************************************************************/
agc_vec_t agc_erase_x789(void)
{
    return (x789);
}

/**********************************************************************
 * Store X789.
 * AGC: X789 (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_set_x789
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   v         agc_vec_t  I    X789 at the bias scales
 **********************************************************************/
void agc_erase_set_x789(agc_vec_t v)
{
    x789 = v;
}

/**********************************************************************
 * A raw 15-bit ones' complement word as a signed number.
 * AGC: none (word conversion)
 *
 * FUNCTION NAME: rz_word_value
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   w         uint16_t  I    raw word
 *
 * RETURNS: int -- the signed value (0 when out of range)
 **********************************************************************/
int rz_word_value(uint16_t w)
{
    uint16_t v = (uint16_t) (w & 077777u);                               /* 15 bits */
    int      r = ((v & 040000u) != 0u) ? -(int) (077777u - v) : (int) v; /* signed */

    return (r);
}

/**********************************************************************
 * INCORP2: W update, then the deviation into the corrected vehicle's
 * state (MOVEP.., FAZAB, RECTIFY on overflow, MOVEA.. with SVDWN), X789,
 * INTWAKE. moonthis: CMOONFLG (CM) or LMOONFLG (LM).
 * AGC: INCORP2 (MEASUREMENT_INCORPORATION.agc p.1256)
 *
 * FUNCTION NAME: rz_incorp2
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description (scale)
 *   --------  --------------------  ---  --------------------------
 *   ip        agc_interp_t *        I/O  interpreter state (OVFIND)
 *   inc       const agc_incorp_t *  I    INCORP1 results (deviations, W
 *                                        update terms)
 *   moonthis  bool                  I    CMOONFLG (CM) or LMOONFLG (LM)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the W, state, RECTIFY or INTWAKE
 *          step
 **********************************************************************/
agc_status_t rz_incorp2(agc_interp_t *ip, const agc_incorp_t *inc, bool moonthis)
{
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();                 /* integration erasables */
    bool             csm = agc_flag_test(AGC_FLAG_VEHUPFLG);       /* VEHUPFLG */
    agc_vehicle_t    veh = csm ? AGC_VEHICLE_CSM : AGC_VEHICLE_LM; /* corrected */
    agc_status_t     st  = agc_incorp_update_w(ip, inc, &ctx->w);  /* W update */

    agc_phase_change(2, 04022); /* FAZC     */

    if (st == AGC_STATUS_OK)
    {
        st = agc_integ_perm_get(veh, &ctx->sv); /* MOVEP..  */
    }

    agc_incorp_sv_t sv; /* state for the update */

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
    ctx->pbody     = sv.moon ? 2 : 0; /* X2 of the body */
    rz_flag_put(AGC_FLAG_MOONFLAG, sv.moon);

    if (st == AGC_STATUS_OK && sv.rectify)
    {
        st = agc_integ_rectify(ip, ctx); /* FAZAB2   */
    }

    agc_phase_change(2, 04022); /* FAZAB3   */

    if (st == AGC_STATUS_OK)
    {
        st = csm ? agc_integ_atopcsm(ip, ctx, ctx->pbody) : agc_integ_atoplem(ip, ctx, ctx->pbody); /* MOVEA.. */
    }

    if (inc->dim9)
    {
        agc_erase_set_x789(sv.x789); /* FAZAB4   */
    }

    agc_status_t sw = AGC_STATUS_OK; /* INTWAKE status */

    if (agc_integ_area_busy(0))
    {
        sw = agc_integ_intwake(); /* INTWAKE */
    }

    return ((st == AGC_STATUS_OK) ? sw : st); /* first failure */
} /* rz_incorp2 */

#define RZ_PI_4 ((agc_dp_t) 210828714) /* PI/4.0 2DEC .785398164     */

/**********************************************************************
 * BVECTORS (P20-P25 CM p.576): USTAR = UNIT(USTAR x UNIT(RCLP)) = B0,
 * DELTAQ = (ACOS(USTAR . UM) - 1/4) PI/4 |RCLP| normalized (SRR* 0-3,1),
 * B1 = B2 = 0. rclp body scale, um and ustar half unit.
 * AGC: BVECTORS (P20-P25.agc p.576)
 *
 * FUNCTION NAME: rz_bvectors
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rclp      agc_vec_t       I    RCLP, body scale
 *   um        agc_vec_t       I    UM, half unit
 *   ustar     agc_vec_t *     I/O  USTAR, half unit; B0 out
 *   inc       agc_incorp_t *  O    BVECTOR, DELTAQ
 **********************************************************************/
void rz_bvectors(agc_interp_t *ip, agc_vec_t rclp, agc_vec_t um, agc_vec_t *ustar, agc_incorp_t *inc)
{
    agc_dp_t  absr = 0;                                   /* |RCLP|, body scale */
    agc_vec_t u    = agc_vec_unit(ip, rclp, &absr, NULL); /* 26D, 2^1 */

    inc->bvector[0] = agc_vec_unit(ip, agc_vec_cross(ip, *ustar, u), NULL, NULL);
    agc_phase_change(2, 04022); /* GRP2PC   */
    *ustar          = inc->bvector[0];
    agc_dp_t a;                 /* rev 2^0 */

    a           = agc_dp_acos(ip, agc_dp_shl(ip, agc_vec_dot(ip, *ustar, um), 1));
    int      x1 = 0;                                               /* NORM index, -(left shifts) */
    agc_dp_t n  = rz_norm(agc_dp_sub(ip, a, AGC_DP_QUARTER), &x1); /* 2^(X1) */
    agc_tp_t t;                                                    /* TP, scale set by SRR* */

    t = agc_dp_mul_tp(agc_dp_mul_trunc(n, RZ_PI_4), absr);

    inc->deltaq     = agc_tp_round(ip, agc_tp_shift(ip, t, -(-3 - x1))); /* SRR* 0-3,1 */
    inc->bvector[1] = agc_vec_zero();
    inc->bvector[2] = agc_vec_zero();
}

/**********************************************************************
 * A signed number as a raw 15-bit ones' complement word.
 * AGC: none (word conversion)
 *
 * FUNCTION NAME: rz_word_of
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   v         int   I    the number
 *
 * RETURNS: uint16_t -- the raw word (0 when out of range)
 **********************************************************************/
uint16_t rz_word_of(int v)
{
    return ((v >= 0) ? (uint16_t) v : (uint16_t) (077777 + v)); /* 1s compl. */
}

/**********************************************************************
 * THISPREC / OTHPREC: CSMPREC or LEMPREC by vehicle.
 * AGC: THISPREC / OTHPREC (P34-35_P74-75.agc), CSMPREC / LEMPREC by
 * vehicle
 *
 * FUNCTION NAME: rz_resolve_kind
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   kind      rz_integ_kind_t  I    integration kind
 *
 * RETURNS: rz_integ_kind_t -- CSMPREC, LEMPREC, CSMCONIC or LEMCONIC
 **********************************************************************/
static rz_integ_kind_t rz_resolve_kind(rz_integ_kind_t kind)
{
    bool csm = rz_is_csm(); /* CM build */

    if (kind == RZ_THISPREC)
    {
        return (csm ? RZ_CSMPREC : RZ_LEMPREC); /* THISPREC */
    }

    if (kind == RZ_OTHPREC)
    {
        return (csm ? RZ_LEMPREC : RZ_CSMPREC); /* OTHPREC */
    }

    return (kind);
}

/**********************************************************************
 * CSMPREC LEMPREC CSMCONIC LEMCONIC THISPREC OTHPREC
 * (INTEGRATION_INITIALIZATION p.1318): permanent state to TDEC1.
 * AGC: CSMPREC .. LEMCONIC (INTEGRATION_INITIALIZATION.agc p.1318)
 *
 * FUNCTION NAME: rz_integ
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND)
 *   kind      rz_integ_kind_t    I    integration kind (THISPREC / OTHPREC
 *                                     resolved here)
 *   tdec1     agc_dp_t           I    TDEC1, cs 2^28
 *   resume    agc_job_id_t       I    job that continues the caller
 *   out       agc_integ_out_t *  O    integration result (RATT VATT TAT
 *                                     RATT1 VATT1 X2)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the integration status (INTSTALL sleep)
 **********************************************************************/
agc_status_t rz_integ(agc_interp_t *ip, rz_integ_kind_t kind, agc_dp_t tdec1, agc_job_id_t resume, agc_integ_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && out != NULL) || !AGC_ASSERT((int) kind >= 0 && kind < RZ_INTEG_KIND_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_integ_ctx_t *ctx = agc_integ_shared_ctx(); /* integration erasables */
    agc_status_t     st;                           /* integration status */

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

/**********************************************************************
 * INTSTALL, then the INTEGRVS set-up common to INTINT, INITVEL, R31.
 * AGC: INTSTALL, INTEGRVS (INTEGRATION_INITIALIZATION.agc pp.1319-1324)
 *
 * FUNCTION NAME: rz_integrvs
 *
 * ARGUMENTS:
 *   Argument  Type                      I/O  Description (scale)
 *   --------  ------------------------  ---  ----------------------
 *   ip        agc_interp_t *            I/O  interpreter state (OVFIND)
 *   in        const rz_integrvs_in_t *  I    RCV VCV TET TDEC1, conic, moon
 *   resume    agc_job_id_t              I    job that continues the caller
 *   out       agc_integ_out_t *         O    integration result
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the integration status (INTSTALL sleep)
 **********************************************************************/
agc_status_t rz_integrvs(agc_interp_t *ip, const rz_integrvs_in_t *in, agc_job_id_t resume, agc_integ_out_t *out)
{
    agc_status_t st = agc_integ_install(0, resume); /* INTSTALL */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_integ_ctx_t *ctx = agc_integ_shared_ctx(); /* integration erasables */

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

/**********************************************************************
 * PHASCHNG / 2PHSCHNG with a type C code whose restart point is job
 * `job` at priority `prio` (the AGC's OCT 05022 / OCT 26000 pair: the
 * restart runs the job by FINDVAC). code_a = 0: a single PHASCHNG.
 * AGC: PHASCHNG, 2PHSCHNG (PHASE_TABLE_MAINTENANCE.agc p.1409)
 *
 * FUNCTION NAME: rz_phase_job
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------
 *   code_a    uint16_t      I    first code of 2PHSCHNG, 0 for a single
 *                                PHASCHNG
 *   code_c    uint16_t      I    the type C code
 *   job       agc_job_id_t  I    restart job
 *   prio      int16_t       I    its priority
 **********************************************************************/
void rz_phase_job(uint16_t code_a, uint16_t code_c, agc_job_id_t job, int16_t prio)
{
    if (!AGC_ASSERT(job != AGC_JOB_NONE) || !AGC_ASSERT(prio > 0))
    {
        return;
    }

    agc_phase_var_t v; /* restart job */

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
