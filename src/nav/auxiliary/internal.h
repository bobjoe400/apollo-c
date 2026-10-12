/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Helpers private to the nav-aux translation units (latlong.c,
 *   planetary.c, measurement.c, free_fall.c, ephemerides.c), defined in
 *   util.c.  They model interpretive behaviour that interp.h does not
 *   expose: range predicates, shifts by a signed (indexed) count, and
 *   the low words of a TP.
 *
 * AGC SOURCE:
 *   INTERPRETER.agc (SL*, SR*, VSL*, VSR*; DLOAD X+1)
 *
 * REQUIRES:
 *   none (self-contained; includes what it uses)
 *
 * GLOBAL VARIABLES:
 *   none
 **********************************************************************/
#ifndef NAVAUX_INTERNAL_H
#define NAVAUX_INTERNAL_H

#include <stdbool.h>

#include "agc/core/interp.h"
#include "agc/core/word.h"

/* Largest shift count the AGC encodes in a shift address (7 bits).       */
#define NAVAUX_MAX_SHIFT 60 /* shift count limit */

/* SL* / SR* with an indexed count: n > 0 shifts left (SL, overflow sets
 * OVFIND), n < 0 shifts right by -n (SR, truncating).                     */
agc_dp_t  agc_navaux_dp_shift(agc_interp_t *ip, agc_dp_t a, int n);
agc_vec_t agc_navaux_vec_shift(agc_interp_t *ip, agc_vec_t v, int n);

/* Low two words (MPAC+1, MPAC+2) of a TP as a DP fraction.               */
agc_dp_t agc_navaux_tp_low_dp(agc_tp_t v);

#endif /* NAVAUX_INTERNAL_H */
