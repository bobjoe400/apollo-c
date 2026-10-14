/**********************************************************************
 * FILE NAME: rope.c
 *
 * PURPOSE:
 *   The rope image as the harness's fixed memory (--core, --exec,
 *   --rope, or the bare rope path), and the vehicle the rope names.
 *
 * AGC SOURCE:
 *   Comanche055 / Luminary099 binaries (tools/oracle/ropes, as yaYUL
 *   assembles them). Read as data by ROPECHK and SHOW-BANKSUM
 *   (AGC_BLOCK_TWO_SELF_CHECK.agc C pp.1394-1403 / L pp.1291-1300).
 *
 * EXTERNAL VARIABLES:
 *   none (the image is file-static and handed to the self-check)
 *
 * EXTERNAL REFERENCES:
 *   Name                    Description
 *   ----------------------  ----------------------------------------
 *   agc_selfcheck_set_rope  register the image (src/io/service)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   sim_rope_load: AGC_STATUS_NOT_FOUND when the file cannot be opened,
 *   AGC_STATUS_BAD_ARG (reported) when it is not a whole 36-bank image.
 *   sim_rope_vehicle: AGC_STATUS_NOT_FOUND for a rope other than
 *   Comanche055 / Luminary099.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Called once at start-up (main.c).
 *
 * NOTES:
 *   The translated program needs no fixed memory to run: its code and
 *   constants are C. ROPECHK and SHOW-BANKSUM (verb 91) sum each bank;
 *   with the real binary registered, those sums are the real bank sums
 *   (data only; no instruction is ever fetched from it).
 *   File format (yaYUL / yaAGC "exec-ropes" file): whole banks of 1024
 *   big-endian 16-bit words, each word shifted left by one (bit 0 =
 *   parity, dropped), in the order banks 2, 3, 0, 1, 4, 5, ... 43 octal.
 *   The rope also names the vehicle when the Virtual AGC launcher starts
 *   us as yaAGC (--core=source/Comanche055/Comanche055.bin): Comanche055
 *   is the CSM program, Luminary099 the LM's. Any other rope is refused:
 *   this program is those two translated and cannot run another rope.
 *
 * ALGORITHM:
 *   none (file read)
 **********************************************************************/

#include <stdio.h>
#include <string.h>

#include "agc/io/service.h"
#include "internal.h"

#define ROPE_WORDS       (AGC_ROPE_BANKS * AGC_ROPE_BANK_WORDS) /* words in the image          */
#define ROPE_BANK_BYTES  (2 * AGC_ROPE_BANK_WORDS)              /* bytes per bank in the file  */
#define ROPE_NAME        64                                     /* longest rope name           */
#define ROPE_FIXED_FIXED 4                                      /* banks 0-3, stored 2, 3, 0, 1 */

static uint16_t g_rope[ROPE_WORDS]; /* fixed memory, bank b at b * 1024, 15-bit words */

/**********************************************************************
 * AGC: none -- the rope name in a path: the last component up to its
 * first dot.
 *
 * ARGUMENTS:
 *   path  const char *  I  rope path
 *   name  char *        O  the name, NUL-terminated
 *   size  size_t        I  bytes at name, >= 1
 **********************************************************************/
static void rope_name(const char *path, char *name, size_t size)
{
    const char *slash = strrchr(path, '/');                 /* last separator */
    const char *base  = (slash != NULL) ? &slash[1] : path; /* last component */
    size_t      n     = strcspn(base, ".");                 /* name length */

    n       = (n < size - 1u) ? n : size - 1u; /* truncate to fit */
    (void) memcpy(name, base, n);              /* stdlib: returns name */
    name[n] = '\0';
}

/**********************************************************************
 * AGC: none -- the vehicle a rope belongs to: Comanche055 -> CSM,
 * Luminary099 -> LM.
 *
 * ARGUMENTS:
 *   path     const char *     I  rope path
 *   vehicle  agc_vehicle_t *  O  the vehicle
 *
 * RETURNS: agc_status_t -- OK, or NOT_FOUND for any other rope
 **********************************************************************/
agc_status_t sim_rope_vehicle(const char *path, agc_vehicle_t *vehicle)
{
    char name[ROPE_NAME]; /* rope name */

    rope_name(path, name, sizeof (name));

    if (strcmp(name, "Comanche055") == 0)
    {
        *vehicle = AGC_VEHICLE_CSM;
    }
    else if (strcmp(name, "Luminary099") == 0)
    {
        *vehicle = AGC_VEHICLE_LM;
    }
    else
    {
        return (AGC_STATUS_NOT_FOUND);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none -- read the banks of an open rope file into g_rope.
 *
 * ARGUMENTS:
 *   f  FILE *  I  the open file
 *
 * RETURNS: int -- whole banks read (a partial bank is not counted)
 **********************************************************************/
static int rope_read(FILE *f)
{
    int banks = 0; /* whole banks read */

    for (int fb = 0; fb < AGC_ROPE_BANKS; fb++)
    {
        unsigned char b[ROPE_BANK_BYTES]; /* one bank as stored */

        if (fread(b, 1, sizeof (b), f) != sizeof (b))
        {
            break; /* end of file (a partial bank is not counted) */
        }

        int bank = (fb < ROPE_FIXED_FIXED) ? (fb ^ 2) : fb; /* file order 2, 3, 0, 1, 4.. */

        for (int r = 0; r < AGC_ROPE_BANK_WORDS; r++)
        {
            g_rope[bank * AGC_ROPE_BANK_WORDS + r] = (uint16_t) ((((unsigned int) b[2 * r] << 8) | b[2 * r + 1]) >> 1);
        }

        banks++;
    }

    return (banks);
}

/**********************************************************************
 * AGC: none -- load the rope at `path` as fixed memory.
 *
 * ARGUMENTS:
 *   path  const char *  I  rope file
 *
 * RETURNS: agc_status_t -- OK; NOT_FOUND when the file cannot be opened;
 *          BAD_ARG when it is not a whole 36-bank image (reported)
 **********************************************************************/
agc_status_t sim_rope_load(const char *path)
{
    FILE *f = fopen(path, "rb"); /* the rope file */

    if (f == NULL)
    {
        return (AGC_STATUS_NOT_FOUND);
    }

    int  banks = rope_read(f);    /* whole banks read */
    bool tail  = fgetc(f) != EOF; /* more than 36 banks, or a partial bank */

    (void) fclose(f); /* stdlib: read-only file, nothing to lose */

    if (banks != AGC_ROPE_BANKS || tail)
    {
        (void) fprintf(stderr, "agc_sim: %s is not a 36-bank rope image\n", path); /* stdlib diagnostic */

        return (AGC_STATUS_BAD_ARG);
    }

    agc_selfcheck_set_rope(g_rope, AGC_ROPE_BANKS);

    return (AGC_STATUS_OK);
}
