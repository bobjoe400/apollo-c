/*
 * rope.c -- the rope image as the harness's fixed memory (--core, --rope).
 *
 * The translated program needs no fixed memory to run: its code and
 * constants are C. Two AGC routines read the rope as data, the idle-loop
 * self-check ROPECHK and SHOW-BANKSUM (verb 91), which sum each bank
 * (AGC_BLOCK_TWO_SELF_CHECK.agc, C pp.1394-1403 / L pp.1291-1300). The
 * harness loads the real Comanche055 / Luminary099 binary and registers it
 * with agc_selfcheck_set_rope(), so those sums are the real bank sums
 * (data only; no instruction is ever fetched from it).
 *
 * File format (yaYUL / yaAGC "exec-ropes" file, as tools/oracle/ropes and
 * the Virtual AGC source tree hold it): whole banks of 1024 big-endian
 * 16-bit words, each word shifted left by one (bit 0 = parity, dropped),
 * in the order banks 2, 3, 0, 1, 4, 5, ... 43 octal.
 *
 * The rope also names the vehicle when the Virtual AGC launcher starts us
 * as yaAGC (--core=source/Comanche055/Comanche055.bin): Comanche055 is the
 * CSM program, Luminary099 the LM's. Any other rope is refused: this
 * program is those two translated, it cannot run another rope's software.
 */
#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "agc/core/assertion.h"
#include "agc/io/service.h"
#include "internal.h"

#define ROPE_WORDS      (AGC_ROPE_BANKS * AGC_ROPE_BANK_WORDS)
#define ROPE_BANK_BYTES (2 * AGC_ROPE_BANK_WORDS)
#define ROPE_PATH       512

static uint16_t g_rope[ROPE_WORDS]; /* fixed memory, bank b at b * 1024, 15-bit words */

/* The rope name in a path: the last component without ".bin".          */
static void rope_name(const char *path, char *name, size_t size)
{
    const char *slash = strrchr(path, '/');
    const char *base  = (slash != NULL) ? &slash[1] : path;
    size_t      n     = strcspn(base, ".");

    n       = (n < size - 1u) ? n : size - 1u;
    (void) memcpy(name, base, n); /* returns name: nothing to check */
    name[n] = '\0';
}

/* The vehicle a rope belongs to: Comanche055 -> CSM, Luminary099 -> LM.
 * NOT_FOUND for any other rope.                                         */
agc_status_t sim_rope_vehicle(const char *path, agc_vehicle_t *vehicle)
{
    char name[64];

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

/* Read the banks of an open rope file into g_rope; the bank count.     */
static int rope_read(FILE *f)
{
    int banks = 0;

    for (int fb = 0; fb < AGC_ROPE_BANKS; fb++)
    {
        unsigned char b[ROPE_BANK_BYTES]; /* one bank as stored */

        if (fread(b, 1, sizeof (b), f) != sizeof (b))
        {
            break; /* end of file (a partial bank is not counted) */
        }

        int bank = (fb < 4) ? (fb ^ 2) : fb; /* file order 2, 3, 0, 1, 4.. */

        for (int r = 0; r < AGC_ROPE_BANK_WORDS; r++)
        {
            g_rope[bank * AGC_ROPE_BANK_WORDS + r] = (uint16_t) ((((unsigned int) b[2 * r] << 8) | b[2 * r + 1]) >> 1);
        }

        banks++;
    }

    return (banks);
}

/* Load the rope at `path` as fixed memory. NOT_FOUND when the file cannot
 * be opened, BAD_ARG when it is not a whole 36-bank image.             */
agc_status_t sim_rope_load(const char *path)
{
    FILE *f = fopen(path, "rb");

    if (f == NULL)
    {
        return (AGC_STATUS_NOT_FOUND);
    }

    int  banks = rope_read(f);
    bool tail  = fgetc(f) != EOF; /* more than 36 banks, or a partial bank */

    (void) fclose(f); /* read-only file: nothing to lose */

    if (banks != AGC_ROPE_BANKS || tail)
    {
        (void) fprintf(stderr, "agc_sim: %s is not a 36-bank rope image\n", path); /* diagnostic */

        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_selfcheck_set_rope(g_rope, AGC_ROPE_BANKS));
}

/* Without --core: the vehicle's rope from tools/oracle/ropes, looked up
 * from the working directory, then next to the executable (build/..).
 * Not finding it is not an error: ROPECHK then passes every bank
 * (self_check.c DEVIATION); the caller logs it.                        */
agc_status_t sim_rope_load_default(agc_vehicle_t vehicle, const char *argv0)
{
    const char *name = (vehicle == AGC_VEHICLE_LM) ? "Luminary099" : "Comanche055";
    char        path[ROPE_PATH + 64];
    char        exe[ROPE_PATH];
    ssize_t     n    = readlink("/proc/self/exe", exe, sizeof (exe) - 1u);

    (void) snprintf(path, sizeof (path), "tools/oracle/ropes/%s.bin", name); /* fits */
    agc_status_t st = sim_rope_load(path);

    if (st == AGC_STATUS_NOT_FOUND)
    {
        if (n > 0)
        {
            exe[n] = '\0';
        }
        else
        {
            (void) snprintf(exe, sizeof (exe), "%s", argv0); /* no /proc: argv[0] */
        }

        char *slash = strrchr(exe, '/');

        if (slash != NULL)
        {
            *slash = '\0';
            (void) snprintf(path, sizeof (path), "%s/../tools/oracle/ropes/%s.bin", exe, name); /* truncation only fails the open */
            st     = sim_rope_load(path);
        }
    }

    return (st);
}
