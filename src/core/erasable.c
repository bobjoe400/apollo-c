/**********************************************************************
 * FILE NAME: erasable.c
 *
 * PURPOSE:
 *   Storage and accessors for the erasables shared by several modules
 *   (times of state and burns, state vectors, REFSMMAT, masses, CDU and
 *   PIPA registers, mode words, DSPTAB, FAILREG, THETAD). One static
 *   object per AGC erasable (rule 6: no extern globals); each accessor
 *   is a plain get/set with a range check.
 *
 * AGC SOURCE:
 *   Comanche055 ERASABLE_ASSIGNMENTS.agc pp. 39-124
 *   Luminary099 ERASABLE_ASSIGNMENTS.agc pp. 92-147 (same names; each
 *   function prolog gives both pages)
 *
 * EXTERNAL VARIABLES:  (the erasables below, all module-static)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   see the storage block: every object carries its AGC name and scale
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_state_vehicle     vehicle selected by agc_state_init (state.c)
 *   agc_erase_reset_*     fresh-start zeroing of the other erasable
 *                         files (counters, display, pad loads, ecadr,
 *                         restart delays)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No AGC alarms. A failed AGC_ASSERT (value out of range, vehicle not
 *   selected) makes a getter return 0 and a setter leave the erasable
 *   unchanged.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Values are held exactly as the AGC holds them. DP/TP values are the
 *   signed agc_dp_t/agc_tp_t model; CDU and bit words are raw 15-bit
 *   words (agc_sp_t cannot hold 0100000..077777 as a positive count).
 *
 * NOTES:
 *   DELVSLV is not here: DELVSLV = DELVLVC in both ropes, so it is the
 *   one vector agc_rendz_erasable()->delvlvc. The AGC E-bank overlay
 *   layout (TIG EQUALS DELVSLV +6 etc.) is not reproduced.
 *   docs/modules/state-flags.md has the full erasable -> owner table.
 *
 * ALGORITHM:
 *   none (storage)
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/state.h"

#define AGC_WORD15_MAX 077777u /* largest raw 15-bit AGC word */
#define DSPTAB_WORDS   12      /* DSPTAB +0..+11D             */
#define FAILREG_WORDS  3       /* FAILREG +0..+2              */
#define THETAD_WORDS   3       /* THETAD +0..+2               */

/* ---- storage ------------------------------------------------------- */
static agc_sp_t  g_modreg;                 /* MODREG, unscaled integer */
static agc_dp_t  g_piptime;                /* PIPTIME, cs, scale 2^28 */
static agc_dp_t  g_tig;                    /* TIG, cs, scale 2^28 */
static agc_dp_t  g_ttogo;                  /* TTOGO, cs, scale 2^28 */
static agc_dp_t  g_tgo;                    /* TGO, cs, scale 2^28 */
static agc_dp_t  g_ttpi;                   /* TTPI, cs, scale 2^28 */
static agc_dp_t  g_tevent;                 /* TEVENT, cs, scale 2^28 */
static agc_dp_t  g_tetcsm;                 /* TETCSM, cs, scale 2^28 */
static agc_dp_t  g_tetlem;                 /* TETLEM, cs, scale 2^28 */
static agc_vec_t g_rn;                     /* RN, m, scale 2^29 earth / 2^27 moon */
static agc_vec_t g_vn;                     /* VN, m/cs, scale 2^7 earth / 2^5 moon */
static agc_vec_t g_rrectcsm;               /* RRECTCSM, m, scale 2^29 earth / 2^27 moon */
static agc_vec_t g_vrectcsm;               /* VRECTCSM, m/cs, scale 2^7 earth / 2^5 moon */
static agc_vec_t g_rrectlem;               /* RRECTLEM, m, scale 2^29 earth / 2^27 moon */
static agc_vec_t g_vrectlem;               /* VRECTLEM, m/cs, scale 2^7 earth / 2^5 moon */
static agc_vec_t g_delv;                   /* DELV, PIPA pulses, scale 2^14 */
static agc_vec_t g_vgtig;                  /* VGTIG, m/cs, scale 2^7 */
static agc_vec_t g_rls;                    /* RLS, m, scale 2^27 */
static agc_mat_t g_refsmmat;               /* REFSMMAT, scale 2^1 */
static agc_tp_t  g_tephem;                 /* TEPHEM, cs, TP scale 2^42 */
static agc_sp_t  g_csmmass;                /* CSMMASS, kg, scale 2^16 */
static agc_sp_t  g_lemmass;                /* LEMMASS, kg, scale 2^16 */
static agc_sp_t  g_pipax;                  /* PIPAX, unscaled pulses */
static agc_sp_t  g_pipay;                  /* PIPAY, unscaled pulses */
static agc_sp_t  g_pipaz;                  /* PIPAZ, unscaled pulses */
static uint16_t  g_cdux;                   /* CDUX, raw word, 2^15 = 1 rev */
static uint16_t  g_cduy;                   /* CDUY, raw word, 2^15 = 1 rev */
static uint16_t  g_cduz;                   /* CDUZ, raw word, 2^15 = 1 rev */
static uint16_t  g_cdut;                   /* CDUT, raw word, 2^15 = 1 rev */
static uint16_t  g_cdus;                   /* CDUS, raw word, 2^15 = 1 rev */
static uint16_t  g_markstat;               /* MARKSTAT, bit word */
static uint16_t  g_imodes30;               /* IMODES30, bit word */
static uint16_t  g_imodes33;               /* IMODES33, bit word */
static uint16_t  g_dapdatr1;               /* DAPDATR1, bit word */
static uint16_t  g_dapdatr2;               /* DAPDATR2, bit word */
static uint16_t  g_dsptab[DSPTAB_WORDS];   /* DSPTAB, relay bit words */
static uint16_t  g_failreg[FAILREG_WORDS]; /* FAILREG, octal alarm codes */
static uint16_t  g_thetad[THETAD_WORDS];   /* THETAD, raw words, 2^15 = 1 rev */

/**********************************************************************
 * AGC: MODREG (ERASABLE_ASSIGNMENTS.agc C p. 76, L p. 101) -- read
 *   MODREG: major mode (program number) shown in PROG
 *
 * FUNCTION NAME: agc_erase_modreg
 *
 * RETURNS: agc_sp_t -- MODREG, unscaled integer (0 after an assertion
 *          report)
 **********************************************************************/
agc_sp_t agc_erase_modreg(void)
{
    return (g_modreg);
}

/**********************************************************************
 * AGC: MODREG (ERASABLE_ASSIGNMENTS.agc C p. 76, L p. 101) -- store
 *   MODREG: major mode (program number) shown in PROG
 *
 * FUNCTION NAME: agc_erase_set_modreg
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_sp_t    I    new value, unscaled integer
 **********************************************************************/
void agc_erase_set_modreg(agc_sp_t v)
{
    g_modreg = v;
}

/**********************************************************************
 * AGC: PIPTIME (ERASABLE_ASSIGNMENTS.agc C p. 79, L p. 105) -- read
 *   PIPTIME: time of the PIPA reading and of RN, VN
 *
 * FUNCTION NAME: agc_erase_piptime
 *
 * RETURNS: agc_dp_t -- PIPTIME, cs, scale 2^28 (0 after an assertion
 *          report)
 **********************************************************************/
agc_dp_t agc_erase_piptime(void)
{
    return (g_piptime);
}

/**********************************************************************
 * AGC: PIPTIME (ERASABLE_ASSIGNMENTS.agc C p. 79, L p. 105) -- store
 *   PIPTIME: time of the PIPA reading and of RN, VN
 *
 * FUNCTION NAME: agc_erase_set_piptime
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_dp_t    I    new value, cs, scale 2^28
 **********************************************************************/
void agc_erase_set_piptime(agc_dp_t v)
{
    g_piptime = v;
}

/**********************************************************************
 * AGC: TIG (ERASABLE_ASSIGNMENTS.agc C p. 117, L p. 139) -- read TIG:
 *   time of ignition (noun 33)
 *
 * FUNCTION NAME: agc_erase_tig
 *
 * RETURNS: agc_dp_t -- TIG, cs, scale 2^28 (0 after an assertion report)
 **********************************************************************/
agc_dp_t agc_erase_tig(void)
{
    return (g_tig);
}

/**********************************************************************
 * AGC: TIG (ERASABLE_ASSIGNMENTS.agc C p. 117, L p. 139) -- store TIG:
 *   time of ignition (noun 33)
 *
 * FUNCTION NAME: agc_erase_set_tig
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_dp_t    I    new value, cs, scale 2^28
 **********************************************************************/
void agc_erase_set_tig(agc_dp_t v)
{
    g_tig = v;
}

/**********************************************************************
 * AGC: TTOGO (ERASABLE_ASSIGNMENTS.agc C p. 123, L p. 139) -- read TTOGO:
 *   time to go to ignition / cutoff (nouns 35, 40)
 *
 * FUNCTION NAME: agc_erase_ttogo
 *
 * RETURNS: agc_dp_t -- TTOGO, cs, scale 2^28 (0 after an assertion
 *          report)
 **********************************************************************/
agc_dp_t agc_erase_ttogo(void)
{
    return (g_ttogo);
}

/**********************************************************************
 * AGC: TTOGO (ERASABLE_ASSIGNMENTS.agc C p. 123, L p. 139) -- store
 *   TTOGO: time to go to ignition / cutoff (nouns 35, 40)
 *
 * FUNCTION NAME: agc_erase_set_ttogo
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_dp_t    I    new value, cs, scale 2^28
 **********************************************************************/
void agc_erase_set_ttogo(agc_dp_t v)
{
    g_ttogo = v;
}

/**********************************************************************
 * AGC: TGO (ERASABLE_ASSIGNMENTS.agc C p. 117, L p. 147) -- read TGO:
 *   time to go in the burn
 *
 * FUNCTION NAME: agc_erase_tgo
 *
 * RETURNS: agc_dp_t -- TGO, cs, scale 2^28 (0 after an assertion report)
 **********************************************************************/
agc_dp_t agc_erase_tgo(void)
{
    return (g_tgo);
}

/**********************************************************************
 * AGC: TGO (ERASABLE_ASSIGNMENTS.agc C p. 117, L p. 147) -- store TGO:
 *   time to go in the burn
 *
 * FUNCTION NAME: agc_erase_set_tgo
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_dp_t    I    new value, cs, scale 2^28
 **********************************************************************/
void agc_erase_set_tgo(agc_dp_t v)
{
    g_tgo = v;
}

/**********************************************************************
 * AGC: TTPI (ERASABLE_ASSIGNMENTS.agc C p. 123, L p. 142) -- read TTPI:
 *   TPI time (noun 37)
 *
 * FUNCTION NAME: agc_erase_ttpi
 *
 * RETURNS: agc_dp_t -- TTPI, cs, scale 2^28 (0 after an assertion report)
 **********************************************************************/
agc_dp_t agc_erase_ttpi(void)
{
    return (g_ttpi);
}

/**********************************************************************
 * AGC: TTPI (ERASABLE_ASSIGNMENTS.agc C p. 123, L p. 142) -- store TTPI:
 *   TPI time (noun 37)
 *
 * FUNCTION NAME: agc_erase_set_ttpi
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_dp_t    I    new value, cs, scale 2^28
 **********************************************************************/
void agc_erase_set_ttpi(agc_dp_t v)
{
    g_ttpi = v;
}

/**********************************************************************
 * AGC: TEVENT (ERASABLE_ASSIGNMENTS.agc C p. 81, L p. 107) -- read
 *   TEVENT: time of event for the downlist (TLIFTOFF in P11)
 *
 * FUNCTION NAME: agc_erase_tevent
 *
 * RETURNS: agc_dp_t -- TEVENT, cs, scale 2^28 (0 after an assertion
 *          report)
 **********************************************************************/
agc_dp_t agc_erase_tevent(void)
{
    return (g_tevent);
}

/**********************************************************************
 * AGC: TEVENT (ERASABLE_ASSIGNMENTS.agc C p. 81, L p. 107) -- store
 *   TEVENT: time of event for the downlist (TLIFTOFF in P11)
 *
 * FUNCTION NAME: agc_erase_set_tevent
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_dp_t    I    new value, cs, scale 2^28
 **********************************************************************/
void agc_erase_set_tevent(agc_dp_t v)
{
    g_tevent = v;
}

/**********************************************************************
 * AGC: TETCSM (ERASABLE_ASSIGNMENTS.agc C p. 86, L p. 111) -- read
 *   TETCSM: time of the permanent CSM state vector
 *
 * FUNCTION NAME: agc_erase_tetcsm
 *
 * RETURNS: agc_dp_t -- TETCSM, cs, scale 2^28 (0 after an assertion
 *          report)
 **********************************************************************/
agc_dp_t agc_erase_tetcsm(void)
{
    return (g_tetcsm);
}

/**********************************************************************
 * AGC: TETCSM (ERASABLE_ASSIGNMENTS.agc C p. 86, L p. 111) -- store
 *   TETCSM: time of the permanent CSM state vector
 *
 * FUNCTION NAME: agc_erase_set_tetcsm
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_dp_t    I    new value, cs, scale 2^28
 **********************************************************************/
void agc_erase_set_tetcsm(agc_dp_t v)
{
    g_tetcsm = v;
}

/**********************************************************************
 * AGC: TETLEM (ERASABLE_ASSIGNMENTS.agc C p. 86, L p. 111) -- read
 *   TETLEM: time of the permanent LM state vector
 *
 * FUNCTION NAME: agc_erase_tetlem
 *
 * RETURNS: agc_dp_t -- TETLEM, cs, scale 2^28 (0 after an assertion
 *          report)
 **********************************************************************/
agc_dp_t agc_erase_tetlem(void)
{
    return (g_tetlem);
}

/**********************************************************************
 * AGC: TETLEM (ERASABLE_ASSIGNMENTS.agc C p. 86, L p. 111) -- store
 *   TETLEM: time of the permanent LM state vector
 *
 * FUNCTION NAME: agc_erase_set_tetlem
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_dp_t    I    new value, cs, scale 2^28
 **********************************************************************/
void agc_erase_set_tetlem(agc_dp_t v)
{
    g_tetlem = v;
}

/**********************************************************************
 * AGC: RN (ERASABLE_ASSIGNMENTS.agc C p. 79, L p. 105) -- read RN: this
 *   vehicle position, reference axes
 *
 * FUNCTION NAME: agc_erase_rn
 *
 * RETURNS: agc_vec_t -- RN, m, scale 2^29 earth / 2^27 moon (zero after
 *          an assertion report)
 **********************************************************************/
agc_vec_t agc_erase_rn(void)
{
    return (g_rn);
}

/**********************************************************************
 * AGC: RN (ERASABLE_ASSIGNMENTS.agc C p. 79, L p. 105) -- store RN: this
 *   vehicle position, reference axes
 *
 * FUNCTION NAME: agc_erase_set_rn
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_vec_t   I    new value, m, scale 2^29 earth / 2^27 moon
 **********************************************************************/
void agc_erase_set_rn(agc_vec_t v)
{
    g_rn = v;
}

/**********************************************************************
 * AGC: VN (ERASABLE_ASSIGNMENTS.agc C p. 79, L p. 105) -- read VN: this
 *   vehicle velocity, reference axes
 *
 * FUNCTION NAME: agc_erase_vn
 *
 * RETURNS: agc_vec_t -- VN, m/cs, scale 2^7 earth / 2^5 moon (zero after
 *          an assertion report)
 **********************************************************************/
agc_vec_t agc_erase_vn(void)
{
    return (g_vn);
}

/**********************************************************************
 * AGC: VN (ERASABLE_ASSIGNMENTS.agc C p. 79, L p. 105) -- store VN: this
 *   vehicle velocity, reference axes
 *
 * FUNCTION NAME: agc_erase_set_vn
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_vec_t   I    new value, m/cs, scale 2^7 earth / 2^5
 *                              moon
 **********************************************************************/
void agc_erase_set_vn(agc_vec_t v)
{
    g_vn = v;
}

/**********************************************************************
 * AGC: RRECTCSM (ERASABLE_ASSIGNMENTS.agc C p. 86, L p. 110) -- read
 *   RRECTCSM: permanent CSM position (rectification base)
 *
 * FUNCTION NAME: agc_erase_rrectcsm
 *
 * RETURNS: agc_vec_t -- RRECTCSM, m, scale 2^29 earth / 2^27 moon (zero
 *          after an assertion report)
 **********************************************************************/
agc_vec_t agc_erase_rrectcsm(void)
{
    return (g_rrectcsm);
}

/**********************************************************************
 * AGC: RRECTCSM (ERASABLE_ASSIGNMENTS.agc C p. 86, L p. 110) -- store
 *   RRECTCSM: permanent CSM position (rectification base)
 *
 * FUNCTION NAME: agc_erase_set_rrectcsm
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_vec_t   I    new value, m, scale 2^29 earth / 2^27 moon
 **********************************************************************/
void agc_erase_set_rrectcsm(agc_vec_t v)
{
    g_rrectcsm = v;
}

/**********************************************************************
 * AGC: VRECTCSM (ERASABLE_ASSIGNMENTS.agc C p. 86, L p. 110) -- read
 *   VRECTCSM: permanent CSM velocity
 *
 * FUNCTION NAME: agc_erase_vrectcsm
 *
 * RETURNS: agc_vec_t -- VRECTCSM, m/cs, scale 2^7 earth / 2^5 moon (zero
 *          after an assertion report)
 **********************************************************************/
agc_vec_t agc_erase_vrectcsm(void)
{
    return (g_vrectcsm);
}

/**********************************************************************
 * AGC: VRECTCSM (ERASABLE_ASSIGNMENTS.agc C p. 86, L p. 110) -- store
 *   VRECTCSM: permanent CSM velocity
 *
 * FUNCTION NAME: agc_erase_set_vrectcsm
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_vec_t   I    new value, m/cs, scale 2^7 earth / 2^5
 *                              moon
 **********************************************************************/
void agc_erase_set_vrectcsm(agc_vec_t v)
{
    g_vrectcsm = v;
}

/**********************************************************************
 * AGC: RRECTLEM (ERASABLE_ASSIGNMENTS.agc C p. 86, L p. 111) -- read
 *   RRECTLEM: permanent LM position (rectification base)
 *
 * FUNCTION NAME: agc_erase_rrectlem
 *
 * RETURNS: agc_vec_t -- RRECTLEM, m, scale 2^29 earth / 2^27 moon (zero
 *          after an assertion report)
 **********************************************************************/
agc_vec_t agc_erase_rrectlem(void)
{
    return (g_rrectlem);
}

/**********************************************************************
 * AGC: RRECTLEM (ERASABLE_ASSIGNMENTS.agc C p. 86, L p. 111) -- store
 *   RRECTLEM: permanent LM position (rectification base)
 *
 * FUNCTION NAME: agc_erase_set_rrectlem
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_vec_t   I    new value, m, scale 2^29 earth / 2^27 moon
 **********************************************************************/
void agc_erase_set_rrectlem(agc_vec_t v)
{
    g_rrectlem = v;
}

/**********************************************************************
 * AGC: VRECTLEM (ERASABLE_ASSIGNMENTS.agc C p. 86, L p. 111) -- read
 *   VRECTLEM: permanent LM velocity
 *
 * FUNCTION NAME: agc_erase_vrectlem
 *
 * RETURNS: agc_vec_t -- VRECTLEM, m/cs, scale 2^7 earth / 2^5 moon (zero
 *          after an assertion report)
 **********************************************************************/
agc_vec_t agc_erase_vrectlem(void)
{
    return (g_vrectlem);
}

/**********************************************************************
 * AGC: VRECTLEM (ERASABLE_ASSIGNMENTS.agc C p. 86, L p. 111) -- store
 *   VRECTLEM: permanent LM velocity
 *
 * FUNCTION NAME: agc_erase_set_vrectlem
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_vec_t   I    new value, m/cs, scale 2^7 earth / 2^5
 *                              moon
 **********************************************************************/
void agc_erase_set_vrectlem(agc_vec_t v)
{
    g_vrectlem = v;
}

/**********************************************************************
 * AGC: DELV (ERASABLE_ASSIGNMENTS.agc C p. 79, L p. 99) -- read DELV:
 *   accumulated PIPA counts, stable-member axes
 *
 * FUNCTION NAME: agc_erase_delv
 *
 * RETURNS: agc_vec_t -- DELV, PIPA pulses, scale 2^14 (zero after an
 *          assertion report)
 **********************************************************************/
agc_vec_t agc_erase_delv(void)
{
    return (g_delv);
}

/**********************************************************************
 * AGC: DELV (ERASABLE_ASSIGNMENTS.agc C p. 79, L p. 99) -- store DELV:
 *   accumulated PIPA counts, stable-member axes
 *
 * FUNCTION NAME: agc_erase_set_delv
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_vec_t   I    new value, PIPA pulses, scale 2^14
 **********************************************************************/
void agc_erase_set_delv(agc_vec_t v)
{
    g_delv = v;
}

/**********************************************************************
 * AGC: VGTIG (ERASABLE_ASSIGNMENTS.agc C p. 124, L p. 143) -- read VGTIG:
 *   velocity to be gained at TIG
 *
 * FUNCTION NAME: agc_erase_vgtig
 *
 * RETURNS: agc_vec_t -- VGTIG, m/cs, scale 2^7 (zero after an assertion
 *          report)
 **********************************************************************/
agc_vec_t agc_erase_vgtig(void)
{
    return (g_vgtig);
}

/**********************************************************************
 * AGC: VGTIG (ERASABLE_ASSIGNMENTS.agc C p. 124, L p. 143) -- store
 *   VGTIG: velocity to be gained at TIG
 *
 * FUNCTION NAME: agc_erase_set_vgtig
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_vec_t   I    new value, m/cs, scale 2^7
 **********************************************************************/
void agc_erase_set_vgtig(agc_vec_t v)
{
    g_vgtig = v;
}

/**********************************************************************
 * AGC: RLS (ERASABLE_ASSIGNMENTS.agc C p. 88, L p. 113) -- read RLS:
 *   landing site, moon-fixed axes
 *
 * FUNCTION NAME: agc_erase_rls
 *
 * RETURNS: agc_vec_t -- RLS, m, scale 2^27 (zero after an assertion
 *          report)
 **********************************************************************/
agc_vec_t agc_erase_rls(void)
{
    return (g_rls);
}

/**********************************************************************
 * AGC: RLS (ERASABLE_ASSIGNMENTS.agc C p. 88, L p. 113) -- store RLS:
 *   landing site, moon-fixed axes
 *
 * FUNCTION NAME: agc_erase_set_rls
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_vec_t   I    new value, m, scale 2^27
 **********************************************************************/
void agc_erase_set_rls(agc_vec_t v)
{
    g_rls = v;
}

/**********************************************************************
 * AGC: REFSMMAT (ERASABLE_ASSIGNMENTS.agc C p. 86, L p. 111) -- read
 *   REFSMMAT: reference-to-stable-member matrix, row-major
 *
 * FUNCTION NAME: agc_erase_refsmmat
 *
 * RETURNS: agc_mat_t -- REFSMMAT, scale 2^1 (zero after an assertion
 *          report)
 **********************************************************************/
agc_mat_t agc_erase_refsmmat(void)
{
    return (g_refsmmat);
}

/**********************************************************************
 * AGC: REFSMMAT (ERASABLE_ASSIGNMENTS.agc C p. 86, L p. 111) -- store
 *   REFSMMAT: reference-to-stable-member matrix, row-major
 *
 * FUNCTION NAME: agc_erase_set_refsmmat
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_mat_t   I    new value, scale 2^1
 **********************************************************************/
void agc_erase_set_refsmmat(agc_mat_t v)
{
    g_refsmmat = v;
}

/**********************************************************************
 * AGC: TEPHEM (ERASABLE_ASSIGNMENTS.agc C p. 86, L p. 111) -- read
 *   TEPHEM: ephemeris epoch time (TIMSUBO)
 *
 * FUNCTION NAME: agc_erase_tephem
 *
 * RETURNS: agc_tp_t -- TEPHEM, cs, TP scale 2^42 (0 after an assertion
 *          report)
 **********************************************************************/
agc_tp_t agc_erase_tephem(void)
{
    return (g_tephem);
}

/**********************************************************************
 * AGC: TEPHEM (ERASABLE_ASSIGNMENTS.agc C p. 86, L p. 111) -- store
 *   TEPHEM: ephemeris epoch time (TIMSUBO)
 *
 * FUNCTION NAME: agc_erase_set_tephem
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_tp_t    I    new value, cs, TP scale 2^42
 **********************************************************************/
void agc_erase_set_tephem(agc_tp_t v)
{
    g_tephem = v;
}

/**********************************************************************
 * AGC: CSMMASS (ERASABLE_ASSIGNMENTS.agc C p. 103, L p. 107) -- read
 *   CSMMASS: CSM mass (noun 47 R1)
 *
 * FUNCTION NAME: agc_erase_csmmass
 *
 * RETURNS: agc_sp_t -- CSMMASS, kg, scale 2^16 (0 after an assertion
 *          report)
 **********************************************************************/
agc_sp_t agc_erase_csmmass(void)
{
    return (g_csmmass);
}

/**********************************************************************
 * AGC: CSMMASS (ERASABLE_ASSIGNMENTS.agc C p. 103, L p. 107) -- store
 *   CSMMASS: CSM mass (noun 47 R1)
 *
 * FUNCTION NAME: agc_erase_set_csmmass
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_sp_t    I    new value, kg, scale 2^16
 **********************************************************************/
void agc_erase_set_csmmass(agc_sp_t v)
{
    g_csmmass = v;
}

/**********************************************************************
 * AGC: LEMMASS (ERASABLE_ASSIGNMENTS.agc C p. 103, L p. 107) -- read
 *   LEMMASS: LM mass (noun 47 R2)
 *
 * FUNCTION NAME: agc_erase_lemmass
 *
 * RETURNS: agc_sp_t -- LEMMASS, kg, scale 2^16 (0 after an assertion
 *          report)
 **********************************************************************/
agc_sp_t agc_erase_lemmass(void)
{
    return (g_lemmass);
}

/**********************************************************************
 * AGC: LEMMASS (ERASABLE_ASSIGNMENTS.agc C p. 103, L p. 107) -- store
 *   LEMMASS: LM mass (noun 47 R2)
 *
 * FUNCTION NAME: agc_erase_set_lemmass
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_sp_t    I    new value, kg, scale 2^16
 **********************************************************************/
void agc_erase_set_lemmass(agc_sp_t v)
{
    g_lemmass = v;
}

/**********************************************************************
 * AGC: PIPAX (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- read PIPAX:
 *   PIPA X counter
 *
 * FUNCTION NAME: agc_erase_pipax
 *
 * RETURNS: agc_sp_t -- PIPAX, unscaled pulses (0 after an assertion
 *          report)
 **********************************************************************/
agc_sp_t agc_erase_pipax(void)
{
    return (g_pipax);
}

/**********************************************************************
 * AGC: PIPAX (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- store PIPAX:
 *   PIPA X counter
 *
 * FUNCTION NAME: agc_erase_set_pipax
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_sp_t    I    new value, unscaled pulses
 **********************************************************************/
void agc_erase_set_pipax(agc_sp_t v)
{
    g_pipax = v;
}

/**********************************************************************
 * AGC: PIPAY (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- read PIPAY:
 *   PIPA Y counter
 *
 * FUNCTION NAME: agc_erase_pipay
 *
 * RETURNS: agc_sp_t -- PIPAY, unscaled pulses (0 after an assertion
 *          report)
 **********************************************************************/
agc_sp_t agc_erase_pipay(void)
{
    return (g_pipay);
}

/**********************************************************************
 * AGC: PIPAY (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- store PIPAY:
 *   PIPA Y counter
 *
 * FUNCTION NAME: agc_erase_set_pipay
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_sp_t    I    new value, unscaled pulses
 **********************************************************************/
void agc_erase_set_pipay(agc_sp_t v)
{
    g_pipay = v;
}

/**********************************************************************
 * AGC: PIPAZ (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- read PIPAZ:
 *   PIPA Z counter
 *
 * FUNCTION NAME: agc_erase_pipaz
 *
 * RETURNS: agc_sp_t -- PIPAZ, unscaled pulses (0 after an assertion
 *          report)
 **********************************************************************/
agc_sp_t agc_erase_pipaz(void)
{
    return (g_pipaz);
}

/**********************************************************************
 * AGC: PIPAZ (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- store PIPAZ:
 *   PIPA Z counter
 *
 * FUNCTION NAME: agc_erase_set_pipaz
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         agc_sp_t    I    new value, unscaled pulses
 **********************************************************************/
void agc_erase_set_pipaz(agc_sp_t v)
{
    g_pipaz = v;
}

/**********************************************************************
 * AGC: CDUX (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- read CDUX: CDU
 *   X (outer gimbal) counter
 *
 * FUNCTION NAME: agc_erase_cdux
 *
 * RETURNS: uint16_t -- CDUX, raw word, 2^15 = 1 rev (0 after an assertion
 *          report)
 **********************************************************************/
uint16_t agc_erase_cdux(void)
{
    return (g_cdux);
}

/**********************************************************************
 * AGC: CDUX (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- store CDUX:
 *   CDU X (outer gimbal) counter
 *
 * FUNCTION NAME: agc_erase_set_cdux
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         uint16_t    I    new value, raw word, 2^15 = 1 rev
 **********************************************************************/
void agc_erase_set_cdux(uint16_t v)
{
    g_cdux = v;
}

/**********************************************************************
 * AGC: CDUY (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- read CDUY: CDU
 *   Y (inner gimbal) counter
 *
 * FUNCTION NAME: agc_erase_cduy
 *
 * RETURNS: uint16_t -- CDUY, raw word, 2^15 = 1 rev (0 after an assertion
 *          report)
 **********************************************************************/
uint16_t agc_erase_cduy(void)
{
    return (g_cduy);
}

/**********************************************************************
 * AGC: CDUY (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- store CDUY:
 *   CDU Y (inner gimbal) counter
 *
 * FUNCTION NAME: agc_erase_set_cduy
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         uint16_t    I    new value, raw word, 2^15 = 1 rev
 **********************************************************************/
void agc_erase_set_cduy(uint16_t v)
{
    g_cduy = v;
}

/**********************************************************************
 * AGC: CDUZ (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- read CDUZ: CDU
 *   Z (middle gimbal) counter
 *
 * FUNCTION NAME: agc_erase_cduz
 *
 * RETURNS: uint16_t -- CDUZ, raw word, 2^15 = 1 rev (0 after an assertion
 *          report)
 **********************************************************************/
uint16_t agc_erase_cduz(void)
{
    return (g_cduz);
}

/**********************************************************************
 * AGC: CDUZ (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- store CDUZ:
 *   CDU Z (middle gimbal) counter
 *
 * FUNCTION NAME: agc_erase_set_cduz
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         uint16_t    I    new value, raw word, 2^15 = 1 rev
 **********************************************************************/
void agc_erase_set_cduz(uint16_t v)
{
    g_cduz = v;
}

/**********************************************************************
 * AGC: CDUT (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- read CDUT:
 *   optics trunnion (CSM) / RR trunnion (LM) CDU
 *
 * FUNCTION NAME: agc_erase_cdut
 *
 * RETURNS: uint16_t -- CDUT, raw word, 2^15 = 1 rev (0 after an assertion
 *          report)
 **********************************************************************/
uint16_t agc_erase_cdut(void)
{
    return (g_cdut);
}

/**********************************************************************
 * AGC: CDUT (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- store CDUT:
 *   optics trunnion (CSM) / RR trunnion (LM) CDU
 *
 * FUNCTION NAME: agc_erase_set_cdut
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         uint16_t    I    new value, raw word, 2^15 = 1 rev
 **********************************************************************/
void agc_erase_set_cdut(uint16_t v)
{
    g_cdut = v;
}

/**********************************************************************
 * AGC: CDUS (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- read CDUS:
 *   optics shaft (CSM) / RR shaft (LM) CDU
 *
 * FUNCTION NAME: agc_erase_cdus
 *
 * RETURNS: uint16_t -- CDUS, raw word, 2^15 = 1 rev (0 after an assertion
 *          report)
 **********************************************************************/
uint16_t agc_erase_cdus(void)
{
    return (g_cdus);
}

/**********************************************************************
 * AGC: CDUS (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- store CDUS:
 *   optics shaft (CSM) / RR shaft (LM) CDU
 *
 * FUNCTION NAME: agc_erase_set_cdus
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         uint16_t    I    new value, raw word, 2^15 = 1 rev
 **********************************************************************/
void agc_erase_set_cdus(uint16_t v)
{
    g_cdus = v;
}

/**********************************************************************
 * AGC: MARKSTAT (ERASABLE_ASSIGNMENTS.agc C p. 81, L p. 106) -- read
 *   MARKSTAT: mark status word
 *
 * FUNCTION NAME: agc_erase_markstat
 *
 * RETURNS: uint16_t -- MARKSTAT, bit word (0 after an assertion report)
 **********************************************************************/
uint16_t agc_erase_markstat(void)
{
    return (g_markstat);
}

/**********************************************************************
 * AGC: MARKSTAT (ERASABLE_ASSIGNMENTS.agc C p. 81, L p. 106) -- store
 *   MARKSTAT: mark status word
 *
 * FUNCTION NAME: agc_erase_set_markstat
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         uint16_t    I    new value, bit word
 **********************************************************************/
void agc_erase_set_markstat(uint16_t v)
{
    g_markstat = v;
}

/**********************************************************************
 * AGC: IMODES30 (ERASABLE_ASSIGNMENTS.agc C p. 81, L p. 106) -- read
 *   IMODES30: last sampled channel 30 IMU bits (IMUMON)
 *
 * FUNCTION NAME: agc_erase_imodes30
 *
 * RETURNS: uint16_t -- IMODES30, bit word (0 after an assertion report)
 **********************************************************************/
uint16_t agc_erase_imodes30(void)
{
    return (g_imodes30);
}

/**********************************************************************
 * AGC: IMODES30 (ERASABLE_ASSIGNMENTS.agc C p. 81, L p. 106) -- store
 *   IMODES30: last sampled channel 30 IMU bits (IMUMON)
 *
 * FUNCTION NAME: agc_erase_set_imodes30
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         uint16_t    I    new value, bit word
 **********************************************************************/
void agc_erase_set_imodes30(uint16_t v)
{
    g_imodes30 = v;
}

/**********************************************************************
 * AGC: IMODES33 (ERASABLE_ASSIGNMENTS.agc C p. 81, L p. 106) -- read
 *   IMODES33: last sampled channel 33 bits (C33TEST)
 *
 * FUNCTION NAME: agc_erase_imodes33
 *
 * RETURNS: uint16_t -- IMODES33, bit word (0 after an assertion report)
 **********************************************************************/
uint16_t agc_erase_imodes33(void)
{
    return (g_imodes33);
}

/**********************************************************************
 * AGC: IMODES33 (ERASABLE_ASSIGNMENTS.agc C p. 81, L p. 106) -- store
 *   IMODES33: last sampled channel 33 bits (C33TEST)
 *
 * FUNCTION NAME: agc_erase_set_imodes33
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         uint16_t    I    new value, bit word
 **********************************************************************/
void agc_erase_set_imodes33(uint16_t v)
{
    g_imodes33 = v;
}

/**********************************************************************
 * AGC: DAPDATR1 (ERASABLE_ASSIGNMENTS.agc C p. 103, L p. 107) -- read
 *   DAPDATR1: DAP configuration word 1 (noun 46 R1)
 *
 * FUNCTION NAME: agc_erase_dapdatr1
 *
 * RETURNS: uint16_t -- DAPDATR1, bit word (0 after an assertion report)
 **********************************************************************/
uint16_t agc_erase_dapdatr1(void)
{
    return (g_dapdatr1);
}

/**********************************************************************
 * AGC: DAPDATR1 (ERASABLE_ASSIGNMENTS.agc C p. 103, L p. 107) -- store
 *   DAPDATR1: DAP configuration word 1 (noun 46 R1)
 *
 * FUNCTION NAME: agc_erase_set_dapdatr1
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         uint16_t    I    new value, bit word
 **********************************************************************/
void agc_erase_set_dapdatr1(uint16_t v)
{
    g_dapdatr1 = v;
}

/**********************************************************************
 * AGC: DAPDATR2 (ERASABLE_ASSIGNMENTS.agc C p. 103) -- read DAPDATR2: DAP
 *   configuration word 2 (noun 46 R2, CSM)
 *
 * FUNCTION NAME: agc_erase_dapdatr2
 *
 * RETURNS: uint16_t -- DAPDATR2, bit word (0 after an assertion report)
 **********************************************************************/
uint16_t agc_erase_dapdatr2(void)
{
    return (g_dapdatr2);
}

/**********************************************************************
 * AGC: DAPDATR2 (ERASABLE_ASSIGNMENTS.agc C p. 103) -- store DAPDATR2:
 *   DAP configuration word 2 (noun 46 R2, CSM)
 *
 * FUNCTION NAME: agc_erase_set_dapdatr2
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   v         uint16_t    I    new value, bit word
 **********************************************************************/
void agc_erase_set_dapdatr2(uint16_t v)
{
    g_dapdatr2 = v;
}

/**********************************************************************
 * AGC: DSPTAB (ERASABLE_ASSIGNMENTS.agc C p. 76, L p. 102) -- read one
 *   word of DSPTAB: display relay buffer +0..+10D, +11D C/S lamps
 *
 * FUNCTION NAME: agc_erase_dsptab
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   index     int         I    word offset 0..11
 *
 * RETURNS: uint16_t -- DSPTAB +index, relay bit words (0 after an
 *          assertion report)
 **********************************************************************/
uint16_t agc_erase_dsptab(int index)
{
    return (g_dsptab[index]);
}

/**********************************************************************
 * AGC: DSPTAB (ERASABLE_ASSIGNMENTS.agc C p. 76, L p. 102) -- store one
 *   word of DSPTAB: display relay buffer +0..+10D, +11D C/S lamps
 *
 * FUNCTION NAME: agc_erase_set_dsptab
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   index     int         I    word offset 0..11
 *   v         uint16_t    I    new word, relay bit words
 **********************************************************************/
void agc_erase_set_dsptab(int index, uint16_t v)
{
    g_dsptab[index] = v;
}

/**********************************************************************
 * AGC: FAILREG (ERASABLE_ASSIGNMENTS.agc C p. 74, L p. 100) -- read one
 *   word of FAILREG: alarm code registers +0..+2
 *
 * FUNCTION NAME: agc_erase_failreg
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   index     int         I    word offset 0..2
 *
 * RETURNS: uint16_t -- FAILREG +index, octal alarm codes (0 after an
 *          assertion report)
 **********************************************************************/
uint16_t agc_erase_failreg(int index)
{
    return (g_failreg[index]);
}

/**********************************************************************
 * AGC: FAILREG (ERASABLE_ASSIGNMENTS.agc C p. 74, L p. 100) -- store one
 *   word of FAILREG: alarm code registers +0..+2
 *
 * FUNCTION NAME: agc_erase_set_failreg
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   index     int         I    word offset 0..2
 *   v         uint16_t    I    new word, octal alarm codes
 *
 * Every caller passes a constant index 0..2 or one from a loop to 3.
 **********************************************************************/
void agc_erase_set_failreg(int index, uint16_t v)
{
    g_failreg[index] = v;
}

/**********************************************************************
 * AGC: THETAD (ERASABLE_ASSIGNMENTS.agc C p. 78, L p. 99) -- read one
 *   word of THETAD: desired gimbal angles CPHI, CTHETA, CPSI
 *
 * FUNCTION NAME: agc_erase_thetad
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   index     int         I    word offset 0..2
 *
 * RETURNS: uint16_t -- THETAD +index, raw words, 2^15 = 1 rev (0 after an
 *          assertion report)
 **********************************************************************/
uint16_t agc_erase_thetad(int index)
{
    return (g_thetad[index]);
}

/**********************************************************************
 * AGC: THETAD (ERASABLE_ASSIGNMENTS.agc C p. 78, L p. 99) -- store one
 *   word of THETAD: desired gimbal angles CPHI, CTHETA, CPSI
 *
 * FUNCTION NAME: agc_erase_set_thetad
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   index     int         I    word offset 0..2
 *   v         uint16_t    I    new word, raw words, 2^15 = 1 rev
 **********************************************************************/
void agc_erase_set_thetad(int index, uint16_t v)
{
    g_thetad[index] = v;
}

/**********************************************************************
 * AGC: none (fresh-start zeroing of the erasables of this file and of
 *      the other shared-erasable files; the AGC fresh start itself is
 *      FRESH_START_AND_RESTART, exec-sched)
 *
 * FUNCTION NAME: agc_erase_reset
 **********************************************************************/
void agc_erase_reset(void)
{
    const agc_mat_t zero_mat = {0}; /* REFSMMAT after fresh start */

    agc_erase_reset_counters();
    agc_erase_reset_display();
    agc_erase_reset_pad_loads();
    agc_erase_reset_ecadr();
    agc_erase_reset_restart_delays();

    g_modreg   = 0;
    g_piptime  = 0;
    g_tig      = 0;
    g_ttogo    = 0;
    g_tgo      = 0;
    g_ttpi     = 0;
    g_tevent   = 0;
    g_tetcsm   = 0;
    g_tetlem   = 0;
    g_rn       = (agc_vec_t){{0, 0, 0}};
    g_vn       = (agc_vec_t){{0, 0, 0}};
    g_rrectcsm = (agc_vec_t){{0, 0, 0}};
    g_vrectcsm = (agc_vec_t){{0, 0, 0}};
    g_rrectlem = (agc_vec_t){{0, 0, 0}};
    g_vrectlem = (agc_vec_t){{0, 0, 0}};
    g_delv     = (agc_vec_t){{0, 0, 0}};
    g_vgtig    = (agc_vec_t){{0, 0, 0}};
    g_rls      = (agc_vec_t){{0, 0, 0}};
    g_refsmmat = zero_mat;
    g_tephem   = 0;
    g_csmmass  = 0;
    g_lemmass  = 0;
    g_pipax    = 0;
    g_pipay    = 0;
    g_pipaz    = 0;
    g_cdux     = 0u;
    g_cduy     = 0u;
    g_cduz     = 0u;
    g_cdut     = 0u;
    g_cdus     = 0u;
    g_markstat = 0u;
    g_imodes30 = 0u;
    g_imodes33 = 0u;
    g_dapdatr1 = 0u;
    g_dapdatr2 = 0u;

    for (int i = 0; i < DSPTAB_WORDS; i++)
    {
        g_dsptab[i] = 0u;
    }

    for (int i = 0; i < FAILREG_WORDS; i++)
    {
        g_failreg[i] = 0u;
    }

    for (int i = 0; i < THETAD_WORDS; i++)
    {
        g_thetad[i] = 0u;
    }
} /* agc_erase_reset */
