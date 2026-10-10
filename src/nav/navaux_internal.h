/*
 * navaux_internal.h -- helpers private to the nav-aux translation units
 * (latlong.c, planetary.c, measurement.c, free_fall.c, ephemerides.c).
 *
 * They model interpretive behaviour that agc_interp.h does not expose:
 * shifts by a signed (indexed) count, TP shifts and normalization, and
 * the overflow wrap-around that angle arithmetic in revolutions relies on.
 */
#ifndef NAVAUX_INTERNAL_H
#define NAVAUX_INTERNAL_H

#include <stdbool.h>
#include "agc/agc_word.h"
#include "agc/agc_interp.h"

/* Largest shift count the AGC encodes in a shift address (7 bits).       */
#define NAVAUX_MAX_SHIFT 60

bool agc_navaux_dp_ok(agc_dp_t a);      /* |a| <= DPPOSMAX              */
bool agc_navaux_vec_ok(agc_vec_t v);    /* every component in range      */
bool agc_navaux_tp_ok(agc_tp_t a);      /* |a| <= TP max                 */

/* SL* / SR* with an indexed count: n > 0 shifts left (SL, overflow sets
 * OVFIND), n < 0 shifts right by -n (SR, truncating).                     */
agc_dp_t  agc_navaux_dp_shift(agc_interp_t *ip, agc_dp_t a, int n);
agc_vec_t agc_navaux_vec_shift(agc_interp_t *ip, agc_vec_t v, int n);

/* DP time in cs at scale 2^28 shifted SR 14 into a TP at scale 2^42.     */
agc_tp_t agc_navaux_tp_from_cs(agc_dp_t t_cs);
/* TP at scale 2^42 cs back to DP at scale 2^28 cs (|t| < 2^28 cs).      */
agc_dp_t agc_navaux_cs_from_tp(agc_tp_t t_tp);
/* Low two words (MPAC+1, MPAC+2) of a TP as a DP fraction.               */
agc_dp_t agc_navaux_tp_low_dp(agc_tp_t v);
/* TSLC on a TP: normalize to |v| >= 1/2, *shift = number of left shifts. */
agc_tp_t agc_navaux_tp_norm(agc_tp_t v, int *shift);
/* Two-place left shift of a TP (SL2 in TP mode); *ovf set on overflow.   */
agc_tp_t agc_navaux_tp_shl2(agc_tp_t v, bool *ovf);

/* Angle arithmetic in revolutions (scale 2^0) with the AGC's overflow
 * correction, i.e. modulo one revolution with the sign kept.              */
agc_dp_t agc_navaux_rev_tp_shl(agc_tp_t v, int n);  /* SL n of a TP, round */
agc_dp_t agc_navaux_rev_add(agc_dp_t a, agc_dp_t b);/* DAD, wrap           */

#endif /* NAVAUX_INTERNAL_H */
