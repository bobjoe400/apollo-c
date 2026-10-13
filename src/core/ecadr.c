/**********************************************************************
 * FILE NAME: ecadr.c
 *
 * PURPOSE:
 *   The raw image of the AGC's 15-bit address space (0000-77777:
 *   erasable E0..E7 at 0000-3777, then the fixed-memory pseudo
 *   addresses, which hold no rope here). Erasable access by address
 *   (ECADR) is src/erasable_map/address_map.c, which maps the flagwords
 *   and the named erasables onto their typed storage and keeps every
 *   word as last stored here; this file is only the image, so core
 *   depends on nothing above it.
 *
 * AGC SOURCE:
 *   none (the address space itself; ERASABLE_ASSIGNMENTS.agc gives the
 *   names the erasable map attaches to it)
 *
 * EXTERNAL VARIABLES:  (module-static)
 *   Name      Type      I/O  Scale          Description
 *   --------  --------  ---  -------------  ----------------------
 *   g_image   uint16_t  I/O  raw 15-bit     one word per ECADR, 00000..77777
 *
 * EXTERNAL REFERENCES:
 *   none
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none (an address is masked to 15 bits; every value is in range)
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Bit 16 of an ECADR is ignored. A fresh start sets every word +0.
 **********************************************************************/
#include <stdint.h>

#include "agc/core/state.h"

#define EC_WORDS    0100000u /* address 00000 .. 77777 */
#define EC_ADDR_MAX 077777u  /* a 15-bit address       */

static uint16_t g_image[EC_WORDS]; /* erasable image, one 15-bit word per ECADR */

/**********************************************************************
 * AGC: none (fresh start) -- set every word of the 15-bit address
 *   space image to +0
 *
 * FUNCTION NAME: agc_erase_reset_ecadr
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_reset_ecadr(void)
{
    for (uint32_t i = 0; i < EC_WORDS; i++)
    {
        g_image[i] = 0;
    }
}

/**********************************************************************
 * AGC: none (erasable image access by ECADR) -- read the word stored
 *   at a 15-bit address
 *
 * FUNCTION NAME: agc_erase_raw_read
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   ecadr     uint16_t    I    ECADR; bit 16 is ignored
 *   word      uint16_t *  O    the stored word, raw 15-bit
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_raw_read(uint16_t ecadr, uint16_t *word)
{
    *word = g_image[ecadr & EC_ADDR_MAX];
}

/**********************************************************************
 * AGC: none (erasable image access by ECADR) -- store a word at a
 *   15-bit address
 *
 * FUNCTION NAME: agc_erase_raw_write
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   ecadr     uint16_t  I    ECADR; bit 16 is ignored
 *   word      uint16_t  I    the word to store, raw 15-bit
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_raw_write(uint16_t ecadr, uint16_t word)
{
    g_image[ecadr & EC_ADDR_MAX] = word;
}
