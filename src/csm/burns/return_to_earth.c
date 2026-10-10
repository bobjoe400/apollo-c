/*
 * p37.c -- P37_P70.agc (Comanche055, pp. 890-933): P37, return to earth.
 *
 * In Comanche the file holds only P37 (the "P70" of the title is the LM
 * abort program of Luminary; there is no Saturn-abort code here). P37
 * computes a conic, then optionally a precision, transearth trajectory
 * from the CSM state at the crew-entered TIG (SPRTETIG), the desired
 * reentry flight path angle GAMMAEI (or a computed one) and the desired
 * delta-V (RTEDVD, or minimum fuel), and stores the targets RTARG, TPASS4,
 * DELVSIN for P40/P41:
 *
 *   P37 .. RTE299   (pp. 891-892) displays V06N33 (TIG), V06N60 (GAMMAEI,
 *                   VPRED); INVC100 state at TIG (CSMPREC); SLOWFLG
 *   RTE317 .. RTE390 (pp. 892-895) the conic iteration on RCON / X(T2)
 *                   (max 9 passes, alarm 605): V2T100, TMRAD100, GETERAD
 *   P37E, RTEVN, RTEDISP (pp. 895-900) displays N61 N39 N60 N81
 *   PREC100 .. PRECX (pp. 903-910) precision trajectory (INTEGRVS),
 *                   RTENCK1-3 (pp. 911-913) state checks, alarms 612/613
 *   V2T100 .. V2T1X (pp. 914-922) the post-impulse velocity for a given
 *                   X(T1), with GAMDV10.. (pp. 923-927) the minimum-DV
 *                   search, DVCALC (pp. 928-929), XT1LIM (p. 930)
 *   P37G .. P37W     target storage, TIG for SPS/RCS (OPTION2), +MGA,
 *                   V06N33/N39/N81 final displays, then P40/P41.
 *
 * Only XT1LIM is translated; the program itself is a stub (see
 * docs/modules/guid-entry.md, Stubs).
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "agc/agc_assert.h"
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_exec.h"
#include "agc/agc_csm_entry.h"
#include "agc/agc_entry_internal.h"

#define RETURN_TO_EARTH_ONE_B11   ((agc_dp_t)(8 * 16384))   /* 1RTEB25 +1 = 1.0 B-11  */

/* P37 (p. 891): return-to-earth targeting. STUB: not translated.         */
agc_status_t agc_job_return_to_earth(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return AGC_STATUS_UNIMPLEMENTED;
}

/* XT1LIM / XT1LIM2 / XT1LIM3 / XT1LIM5 (p. 930): bound on the independent
 * variable X(T1) (cotangent of the post-impulse flight path angle, B5):
 *   BETA10 = BETA5 (MA - R(T1)) / (MA - RCON) - 1        (B11, then B10)
 *   X(T1)LIM = SQRT(BETA10), POSMAX if BETA10 overflows, 0 if negative.
 * MA B30, RCON and R(T1) B29, BETA5 B9.
 * DEVIATION: the AGC shifts the quotient with SL* 0-3,2 (X2 = -NORM count
 * of MA - RCON) and the product with SL* 0-6,1, where X1 is left by the
 * caller V2T100; here the shifts are those that give the documented
 * scales (ratio B3, product B12 -> B11), i.e. X1 = -7.                    */
agc_status_t agc_return_to_earth_xt1lim(agc_dp_t ma, agc_dp_t rcon, agc_dp_t r_t1,
                            agc_dp_t beta5, agc_dp_t *xt1lim)
{
    agc_interp_t *ip = entry_ip();
    agc_dp_t      den;
    agc_dp_t      q;
    int           n2 = 0;

    if (!AGC_ASSERT(xt1lim != NULL) || !AGC_ASSERT(ma > 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    den = agc_dp_sub(ip, ma, agc_dp_shr(rcon, 1));         /* MA - RCON B30 */
    if (!AGC_ASSERT(den != 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    den = agc_dp_norm(den, &n2);                           /* NORM X2       */
    q = agc_dp_div(ip, agc_dp_sub(ip, ma, agc_dp_shr(r_t1, 1)), den);
    q = (n2 >= 3) ? agc_dp_shl(ip, q, n2 - 3) : agc_dp_shr(q, 3 - n2); /* B3 */
    q = agc_dp_mul(ip, q, beta5);                          /* B12           */
    q = agc_dp_sub(ip, agc_dp_shl(ip, q, 1), RETURN_TO_EARTH_ONE_B11); /* BETA10 B11    */
    q = agc_dp_shl(ip, q, 1);                              /* SL1 -> B10    */
    if (agc_interp_test_ovf(ip)) {
        q = AGC_DP_MAX;                                    /* XT1LIM2       */
    } else if (q < 0) {
        *xt1lim = 0;                                       /* XT1LIM5       */
        return AGC_STATUS_OK;
    }
    *xt1lim = agc_dp_sqrt(ip, q);                          /* XT1LIM3, B5   */
    (void)AGC_ASSERT(*xt1lim >= 0);
    return AGC_STATUS_OK;
}
