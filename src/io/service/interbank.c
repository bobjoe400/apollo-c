/*
 * interbank.c -- INTER-BANK_COMMUNICATION.agc (Comanche055 pp. 1103-1106,
 * Luminary099 pp. 998-1001).  Nothing in this file translates to code; it
 * records where each routine went.
 *
 * The AGC addresses 1024 words of fixed memory at a time (the FB bank
 * register, extended by the super-bank bit in channel 7) and 256 words of
 * switchable erasable (EB).  These routines let code in one bank call or
 * jump to code in another and get back, carrying the bank registers in the
 * 2CADR/BBCON words that follow the call:
 *
 *   BANKCALL  (TC BANKCALL, CADR X)  call X in another bank, return to the
 *             caller's bank (SWRETURN).
 *   SWCALL    BANKCALL with the CADR in A (used by the executive/waitlist).
 *   SWRETURN  the return half of BANKCALL / SWCALL.
 *   POSTJUMP  (TC POSTJUMP, CADR X) jump to X in another bank, no return.
 *   BANKJUMP  POSTJUMP with the CADR in A.
 *   MAKECADR  form the CADR of the caller's return address (the AGC's way
 *             of storing "where to continue", e.g. DESRET in V41 N72).
 *   SUPDACAL  fetch a word from another (super) bank (SUPER DATA CALL; used
 *             by ROPECHK / SHOW-BANKSUM).
 *   IBNKCALL, ISWCALL, ISWRETRN  the same calls with interrupts inhibited,
 *             for use from interrupt and waitlist level.
 *   USPRCADR, SUPERSW  calls and switches that also set the super bank.
 *   Q+10000, PRIO12  constants (the second is shared with the executive).
 *
 * In C there is one flat address space (docs/TRANSLATION_GUIDE.md
 * section 3), so:
 *   - TC BANKCALL / IBNKCALL / USPRCADR + CADR X  become a C call of X;
 *   - TC POSTJUMP / BANKJUMP + CADR X             become `return X(...);`
 *     (a tail call in the same job);
 *   - MAKECADR + a later BANKJUMP through the saved CADR become a
 *     continuation job id passed to the routine that will resume the
 *     caller (docs/TRANSLATION_GUIDE.md section 4);
 *   - SUPDACAL becomes an array read of the simulator's rope image
 *     (src/io/service/self_check.c, rope_word);
 *   - the EBANK=, SBANK=, BBCON and 2CADR bookkeeping disappears.
 * No function pointers are needed for any of it (Power of Ten rule 9).
 */
#include "agc/io/service_internal.h"

/* C11 forbids an empty translation unit; this declaration costs nothing. */
_Static_assert(AGC_ROPE_BANK_WORDS == 02000, "a fixed bank is 1024 words");
