# core-numeric -- lane history

Process diary of the core-numeric lane, moved verbatim out of [docs/modules/core-numeric.md](../../modules/core-numeric.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-14 (first and last commit touching the note).

## Progress

* DONE everything in the table above; all files compile with the project
  flags; check_rules: 0 hard, density >= 2 in every core-numeric file.
* DONE tests/core/numeric.c (passes), tests/oracle/core_numeric.c +
  tools/oracle/cases/core_numeric.{spec,txt} (287 cases, 0 mismatches).
* `make lib` currently stops on other lanes' files (src/csm/entry/reentry.c,
  include/agc/io/service_internal.h, src/programs/*); core-numeric objects and both tests
  were built and run by linking src/core/agc_{assert,mpac,divide,shift,word,
  sp_trig,constants,interp_scalar,interp_trig,interp_vector,vec_div}.c.

## Requested core additions

none (this lane owns the numeric headers). Added to agc_interp.h (allowed):
agc_dp_mul_trunc, agc_dp_sq_tp, agc_dp_sqrt_chk, agc_dp_asin_chk,
agc_dp_acos_chk, agc_tp_sqrt, agc_tp_round, agc_tp_shift (nav-conic,
nav-aux request), agc_tp_norm (nav-conic, nav-aux request),
agc_tp_mul_sp (SHORTMP), agc_dp_mul_int (DMPNSUB), agc_dp_poly (POLY,
guid-entry request), agc_vec_dot_tp, agc_vec_sq_tp; plus #include
"agc_status.h". New header include/agc/core/constants.h.

## Requested jobs-tasks

none

## Open questions

* yaAGC bug (tools/oracle owner): the fast path of DV in
  yaAGC-src/agc_engine.c reads L sign-extended to 16 bits, so a -0 minor
  word (0177777) is not recognised by SpToDecent (compares with 077777) and
  is treated as a borrow: the quotient comes out one major LSB low. The
  same engine's hardware-level SimulateDV gives the unborrowed result.
  It shows up when DCS makes a -0 minor word (negative dividends with a
  zero minor word in DDV/V/SC/UNIT, ACOS/ABVAL/SQRT arguments with a zero
  minor word). Patching SpToDecent to mask Msb/Lsb with 077777 removes
  every mismatch; affected cases were left out of core_numeric.spec.
* docs/TRANSLATION_GUIDE.md section 2 says agc_dp_* "saturate the way the
  AGC does": the AGC does not saturate DAD/DSU/SL/VAD (it keeps the
  overflow-corrected value and sets OVFIND); this library follows the AGC.
  Only DDV/V/SC/UNIT/SQRT-near-1 produce DPPOSMAX. Lanes that assumed
  saturation (nav-aux notes it) should rely on BOV, as the AGC code does.
* The guide maps DMP to agc_dp_mul (= DMPR, rounded); bit-exact DMP is
  agc_dp_mul_trunc (or agc_dp_mul_tp when the TP MPAC is used next, e.g.
  DMP then SQRT/TAD/ROUND).

