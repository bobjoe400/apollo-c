/**********************************************************************
 * FILE NAME: display_erasables.c
 *
 * PURPOSE:
 *   Display and downlink registers shared by pinball and the programs:
 *   OPTION1..3, SAMPTIME, SFAIL, MARKDOWN, MARK2DWN, TRKMKCNT, T3TOT4,
 *   RR-AZ and RR-ELEV. Accessors in the style of erasable.c: one static
 *   object per AGC erasable, scale factors in include/agc/core/state.h.
 *
 * AGC SOURCE:
 *   Comanche055 ERASABLE_ASSIGNMENTS.agc pp. 39, 78, 82, 94, 117, 125
 *   Luminary099 ERASABLE_ASSIGNMENTS.agc pp. 92, 104, 108, 118, 139
 *
 * EXTERNAL VARIABLES:  (module-static)
 *   Name         Type      I/O  Scale             Description
 *   -----------  --------  ---  ----------------  ----------------------
 *   g_option     agc_sp_t  I/O  octal word        OPTION1..OPTION3
 *   g_samptime   agc_dp_t  I/O  cs, 2^28          SAMPTIME (TIME2, TIME1)
 *   g_sfail      uint16_t  I/O  octal word        SFAIL (= SELFERAS)
 *   g_markdown   uint16_t  I/O  raw word          MARKDOWN +0..+6
 *   g_mark2dwn   uint16_t  I/O  raw word          MARK2DWN +0..+6
 *   g_trkmkcnt   agc_sp_t  I/O  count, unscaled   TRKMKCNT
 *   g_t3tot4     agc_dp_t  I/O  cs, 2^28          T3TOT4
 *   g_rr_az      agc_dp_t  I/O  revs, 2^0         RR-AZ
 *   g_rr_elev    agc_dp_t  I/O  revs, 2^0         RR-ELEV
 *
 * EXTERNAL REFERENCES:
 *   none
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No AGC alarms and no status returns. The accessors hold no checks:
 *   the indexed ones are called with the AGC's own indices (constants or
 *   bounded loops) and an erasable holds whatever word the AGC stores.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   The vehicle-specific cells (MARKDOWN CSM; RR-AZ, RR-ELEV LM) are
 *   present for both vehicles and used by one.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/state.h"

#define EW15_MAX 077777u /* a raw 15-bit AGC word */

static agc_sp_t g_option[3];   /* OPTION1..OPTION3 (noun 06), octal words, unscaled */
static agc_dp_t g_samptime;    /* SAMPTIME, SAMPTIME +1: sampled TIME2, TIME1 (noun 65), cs 2^28 */
static uint16_t g_sfail;       /* SFAIL (= SELFERAS): self-check failure address, octal word */
static uint16_t g_markdown[7]; /* MARKDOWN +0..+6: CSM P22/P23/R21 mark data for the downlink, raw words */
static uint16_t g_mark2dwn[7]; /* MARK2DWN +0..+6: second mark of the alignment P50s, raw words */
static agc_sp_t g_trkmkcnt;    /* TRKMKCNT: rendezvous tracking mark counter (noun 45 R1), whole count, unscaled */
static agc_dp_t g_t3tot4;      /* T3TOT4 (= RELDELV +2): TPF time for N39 (P34/P35/P37), cs 2^28 */
static agc_dp_t g_rr_az;       /* RR-AZ (LM, noun 56): RR LOS azimuth, revs 2^0 */
static agc_dp_t g_rr_elev;     /* RR-ELEV (LM, noun 56): RR LOS elevation, revs 2^0 */

/**********************************************************************
 * AGC: none (fresh start) -- zero the display and downlink registers
 *   (called by agc_erase_reset)
 *
 * FUNCTION NAME: agc_erase_reset_display
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_reset_display(void)
{
    for (int i = 0; i < 7; i++)
    {
        g_markdown[i] = 0;
        g_mark2dwn[i] = 0;
    }

    for (int i = 0; i < 3; i++)
    {
        g_option[i] = 0;
    }

    g_samptime = 0;
    g_sfail    = 0;
    g_trkmkcnt = 0;
    g_t3tot4   = 0;
    g_rr_az    = 0;
    g_rr_elev  = 0;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 78, L p. 104 -- read
 *   OPTION1..OPTION3 (noun 06 option words)
 *
 * FUNCTION NAME: agc_erase_option1
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------
 *   index     int   I    which word: 0 = OPTION1, 1 = OPTION2, 2 =
 *                        OPTION3
 *
 * RETURNS: agc_sp_t -- OPTION1+index, octal word, unscaled
 **********************************************************************/
agc_sp_t agc_erase_option1(int index)
{
    return (g_option[index - 0]);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 78, L p. 104 -- store
 *   OPTION1..OPTION3 (noun 06 option words)
 *
 * FUNCTION NAME: agc_erase_set_option1
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   index     int       I    which word: 0 = OPTION1, 1 = OPTION2, 2 =
 *                            OPTION3
 *   v         agc_sp_t  I    OPTION1+index, octal word, unscaled
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_option1(int index, agc_sp_t v)
{
    g_option[index - 0] = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92 -- read SAMPTIME,
 *   SAMPTIME +1: sampled TIME2, TIME1 (noun 65)
 *
 * FUNCTION NAME: agc_erase_samptime
 *
 * RETURNS: agc_dp_t -- SAMPTIME, cs, scale 2^28
 **********************************************************************/
agc_dp_t agc_erase_samptime(void)
{
    return (g_samptime);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92 -- store SAMPTIME,
 *   SAMPTIME +1: sampled TIME2, TIME1 (noun 65)
 *
 * FUNCTION NAME: agc_erase_set_samptime
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_dp_t  I    SAMPTIME, cs, scale 2^28
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_samptime(agc_dp_t v)
{
    g_samptime = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 82, L p. 108 -- read SFAIL (=
 *   SELFERAS): self-check failure address
 *
 * FUNCTION NAME: agc_erase_sfail
 *
 * RETURNS: uint16_t -- SFAIL, octal word, unscaled
 **********************************************************************/
uint16_t agc_erase_sfail(void)
{
    return (g_sfail);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 82, L p. 108 -- store SFAIL (=
 *   SELFERAS): self-check failure address
 *
 * FUNCTION NAME: agc_erase_set_sfail
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         uint16_t  I    SFAIL, octal word, unscaled
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_sfail(uint16_t v)
{
    g_sfail = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 125 -- read MARKDOWN +0..+6, the
 *   CSM P22/P23/R21 mark data for the downlink
 *
 * FUNCTION NAME: agc_erase_markdown
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------
 *   index     int   I    word of the mark data, 0..6
 *
 * RETURNS: uint16_t -- MARKDOWN+index, raw word, unscaled
 **********************************************************************/
uint16_t agc_erase_markdown(int index)
{
    return (g_markdown[index - 0]);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 125 -- store MARKDOWN +0..+6, the
 *   CSM P22/P23/R21 mark data for the downlink
 *
 * FUNCTION NAME: agc_erase_set_markdown
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   index     int       I    word of the mark data, 0..6
 *   v         uint16_t  I    MARKDOWN+index, raw word, unscaled
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_markdown(int index, uint16_t v)
{
    g_markdown[index - 0] = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 117 -- read MARK2DWN +0..+6, the
 *   second mark of the alignment P50s
 *
 * FUNCTION NAME: agc_erase_mark2dwn
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------
 *   index     int   I    word of the mark data, 0..6
 *
 * RETURNS: uint16_t -- MARK2DWN+index, raw word, unscaled
 **********************************************************************/
uint16_t agc_erase_mark2dwn(int index)
{
    return (g_mark2dwn[index - 0]);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 117 -- store MARK2DWN +0..+6, the
 *   second mark of the alignment P50s
 *
 * FUNCTION NAME: agc_erase_set_mark2dwn
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   index     int       I    word of the mark data, 0..6
 *   v         uint16_t  I    MARK2DWN+index, raw word, unscaled
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_mark2dwn(int index, uint16_t v)
{
    g_mark2dwn[index - 0] = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 78, L p. 139 -- read TRKMKCNT,
 *   the rendezvous tracking mark counter (noun 45 R1)
 *
 * FUNCTION NAME: agc_erase_trkmkcnt
 *
 * RETURNS: agc_sp_t -- TRKMKCNT, count of marks, unscaled (int)
 **********************************************************************/
agc_sp_t agc_erase_trkmkcnt(void)
{
    return (g_trkmkcnt);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 78, L p. 139 -- store TRKMKCNT,
 *   the rendezvous tracking mark counter (noun 45 R1)
 *
 * FUNCTION NAME: agc_erase_set_trkmkcnt
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_sp_t  I    TRKMKCNT, count of marks, unscaled (int)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_trkmkcnt(agc_sp_t v)
{
    g_trkmkcnt = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 94 -- read T3TOT4 (= RELDELV +2):
 *   TPF time for noun 39 (P34/P35/P37)
 *
 * FUNCTION NAME: agc_erase_t3tot4
 *
 * RETURNS: agc_dp_t -- T3TOT4, cs, scale 2^28
 **********************************************************************/
agc_dp_t agc_erase_t3tot4(void)
{
    return (g_t3tot4);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 94 -- store T3TOT4 (= RELDELV
 *   +2): TPF time for noun 39 (P34/P35/P37)
 *
 * FUNCTION NAME: agc_erase_set_t3tot4
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_dp_t  I    T3TOT4, cs, scale 2^28
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_t3tot4(agc_dp_t v)
{
    g_t3tot4 = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc L p. 118 -- read RR-AZ, the RR
 *   line-of-sight azimuth (LM, noun 56)
 *
 * FUNCTION NAME: agc_erase_rr_az
 *
 * RETURNS: agc_dp_t -- RR-AZ, revolutions, scale 2^0
 **********************************************************************/
agc_dp_t agc_erase_rr_az(void)
{
    return (g_rr_az);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc L p. 118 -- store RR-AZ, the RR
 *   line-of-sight azimuth (LM, noun 56)
 *
 * FUNCTION NAME: agc_erase_set_rr_az
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_dp_t  I    RR-AZ, revolutions, scale 2^0
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_rr_az(agc_dp_t v)
{
    g_rr_az = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc L p. 118 -- read RR-ELEV, the RR
 *   line-of-sight elevation (LM, noun 56)
 *
 * FUNCTION NAME: agc_erase_rr_elev
 *
 * RETURNS: agc_dp_t -- RR-ELEV, revolutions, scale 2^0
 **********************************************************************/
agc_dp_t agc_erase_rr_elev(void)
{
    return (g_rr_elev);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc L p. 118 -- store RR-ELEV, the RR
 *   line-of-sight elevation (LM, noun 56)
 *
 * FUNCTION NAME: agc_erase_set_rr_elev
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_dp_t  I    RR-ELEV, revolutions, scale 2^0
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_rr_elev(agc_dp_t v)
{
    g_rr_elev = v;
}
