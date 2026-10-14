/**********************************************************************
 * FILE NAME: rendezvous_erasables.c
 *
 * PURPOSE:
 *   The rendezvous erasables (agc_rendz_t) and the navigation erasables
 *   the noun tables show (VHFCNT, TANG, RSTACK, HORIZON, CPHIX, PRAXIS,
 *   X789, the W pad loads), the vehicle-dependent flag names (AVFLAG,
 *   MOONTHIS ...) and the word conversions the noun code uses. Owning
 *   the noun storage here keeps the programs layer from reaching up into
 *   src/lm/radar and src/csm/navigation.
 *
 * AGC SOURCE:
 *   Comanche055 / Luminary099 ERASABLE_ASSIGNMENTS.agc (the rendezvous
 *   and navigation erasables), TAGS_FOR_RELATIVE_SETLOC.agc (MOONTHIS)
 *
 * EXTERNAL VARIABLES:
 *   Name       Type        I/O  Scale      Description
 *   ---------  ----------  ---  ---------  ----------------------------
 *   (rendezvous erasables) agc_rendz_t I/O  see rendezvous.h
 *   VHFCNT     agc_sp_t    I/O  -          CM VHF range mark count
 *   TANG RSTACK  sp / dp   I/O  CDU, radar LM radar nouns 66 67 72 78
 *   HORIZON CPHIX PRAXIS words I/O CDU     CM nouns 70 71 95 96
 *   X789       agc_vec_t   I/O  bias scales  radar / landmark biases
 *   CMOONFLG LMOONFLG ... flags I/O  -     by vehicle (flags_csm.c / flags_lm.c)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_state_vehicle     the rope flown (CSM or LM)
 *   agc_flag_test / put   flag words (flags_common.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   None.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Both vehicles. The "this vehicle" / "other vehicle" choices follow
 *   agc_state_vehicle().
 **********************************************************************/
#include "agc/core/flags_common.h"
#include "internal.h"

/* The rendezvous erasables (E-bank storage of ERASABLE_ASSIGNMENTS). */
static agc_rendz_t rendz_erasable; /* rendezvous erasables */

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
    rstack[index] = v;
}

/* HORIZON (CM noun 70/71 R3) and the CM R63 results CPHIX (noun 96, X
 * axis) and PRAXIS (noun 95, preferred axis), CDU words. Owned here, with
 * the noun tables, so this layer does not reach up into
 * src/csm/navigation. (LANDMARK is a core erasable: agc/core/state.h.)  */
static uint16_t horizon;          /* HORIZON word */
static uint16_t trk_angles[2][3]; /* [0] CPHIX, [1] PRAXIS               */

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
    agc_rendz_selectmu(rz, false);
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

    return (agc_flag_test(f));
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
