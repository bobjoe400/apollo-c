/**********************************************************************
 * FILE NAME: <file>.c
 *
 * PURPOSE:
 *   <One or two sentences: what this file translates and why it is a
 *   unit. Name the AGC routines it holds by their labels.>
 *
 * AGC SOURCE:
 *   Comanche055 <FILE>.agc pp. <n-m>   (<LABEL>, <LABEL>)
 *   Luminary099 <FILE>.agc pp. <n-m>   (identical | differs: <how>)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   RN            agc_vec_t I    2^29 m         CSM position (accessor
 *                                               agc_erase_rn())
 *   <none>
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_vec_unit          UNIT: unit vector and length (interp.h)
 *   <none>
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   <AGC alarm codes raised (e.g. AGC_ALARM_00600), BAILOUT/POODOO
 *   paths, agc_status_t values returned and what triggers each.>
 *   <none>
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   <Iteration bounds, timing (runs as a job / task / interrupt),
 *   restart protection, vehicle (CSM, LM, both).>
 *
 * NOTES:
 *   <DEVIATION: summaries, readings of the source that are not
 *   obvious, cross-references to docs/modules/<agent>.md.>
 *
 * ALGORITHM:
 *   <Short prose or numbered steps of the AGC logic. PDL is not used;
 *   the AGC listing cited above is the design document.>
 *
 * (FILE REFERENCES and DEVELOPMENT HISTORY are omitted: the library does
 *  no file I/O, and git log is the history. See docs/CODING_STANDARD.md.)
 **********************************************************************/

/* ---- include order (docs/CODING_STANDARD.md): <stdio.h>/<stdlib.h>, other system
 *      headers, then project headers, alphabetical within each group --- */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/status.h"
#include "agc/core/word.h"

/* ---- 4.4: enums, typedefs, constant macros, function macros ---------- */

/* ---- 4.5: extern declarations (none: rule 6), non-static definitions,
 *      static definitions; each with a short comment giving its scale -- */

/* ---- 4.6: functions, each preceded by its boxed prolog
 *      (docs/templates/function_prolog.c) ------------------------------ */

/* Header files (.h) use the abbreviated prolog in header_prolog.h. */
