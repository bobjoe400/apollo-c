/*
 * internal.h -- helpers private to the nav-aux translation units
 * (latlong.c, planetary.c, measurement.c, free_fall.c, ephemerides.c).
 *
 * They model interpretive behaviour that agc_interp.h does not expose:
 * shifts by a signed (indexed) count, TP shifts and normalization, and
 * the overflow wrap-around that angle arithmetic in revolutions relies on.
 */
#ifndef NAVAUX_INTERNAL_H
#define NAVAUX_INTERNAL_H

#include <stdbool.h>

#include "agc/core/interp.h"
#include "agc/core/word.h"

/* Largest shift count the AGC encodes in a shift address (7 bits).       */
#define NAVAUX_MAX_SHIFT 60

bool agc_navaux_dp_ok(agc_dp_t a);   /* |a| <= DPPOSMAX              */
bool agc_navaux_vec_ok(agc_vec_t v); /* every component in range      */
bool agc_navaux_tp_ok(agc_tp_t a);   /* |a| <= TP max                 */

/* SL* / SR* with an indexed count: n > 0 shifts left (SL, overflow sets
 * OVFIND), n < 0 shifts right by -n (SR, truncating).                     */
agc_dp_t  agc_navaux_dp_shift(agc_interp_t *ip, agc_dp_t a, int n);
agc_vec_t agc_navaux_vec_shift(agc_interp_t *ip, agc_vec_t v, int n);

/* Low two words (MPAC+1, MPAC+2) of a TP as a DP fraction.               */
agc_dp_t agc_navaux_tp_low_dp(agc_tp_t v);

#endif /* NAVAUX_INTERNAL_H */
