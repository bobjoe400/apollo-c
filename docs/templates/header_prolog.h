/**********************************************************************
 * FILE NAME: <name>.h
 *
 * PURPOSE:
 *   <What this header declares and which modules use it.>
 *
 * AGC SOURCE:
 *   <FILE>.agc pp. <n-m> (<LABEL>)   -- or "none" for infrastructure
 *
 * REQUIRES:
 *   none (self-contained; includes what it uses)
 *
 * GLOBAL VARIABLES:
 *   Name          Type      Scale          Description
 *   ------------  --------  -------------  -------------------------
 *   none (shared erasables are reached through accessors; struct
 *   members below carry their scale in a short comment)
 *
 * (Abbreviated prolog of guide Figure 6. DEVELOPMENT HISTORY is git log.)
 **********************************************************************/
#ifndef AGC_TEMPLATE_HEADER_PROLOG_H
#define AGC_TEMPLATE_HEADER_PROLOG_H

#include <stdint.h>

#include "agc/core/word.h"

/* 4.4 order: enums, typedefs, constant macros, function macros.       */
#define AGC_TEMPLATE_LIMIT  ((agc_dp_t)0x0400000)  /* example, 2^28 cs */

#endif /* AGC_TEMPLATE_HEADER_PROLOG_H */
