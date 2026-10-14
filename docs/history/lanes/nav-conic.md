# nav-conic -- lane history

Process diary of the nav-conic lane, moved verbatim out of [docs/modules/nav-conic.md](../../modules/nav-conic.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-14 (first and last commit touching the note).

## Progress

| Routine / table | Status |
|---|---|
| agc_nav.h (conic section) | DONE include/agc/nav/conic.h |
| internal.h, helpers (TP MPAC model agc_conic_m*_, POLY) | DONE src/nav/conic/internal.h, common.c |
| MUTABLE table | DONE mutable_tab common.c |
| DELTIME | DONE agc_conic_deltime_ universal.c |
| GEOM / PARAM | DONE agc_conic_geom_, agc_conic_param_, agc_conic_param geom.c |
| GETX (WLOOP/INVRSEQN/1/WLOOP/POLYCOEF/XCOMMON/TRUE360X/INFINITY) | DONE agc_conic_getx_ + static helpers geom.c |
| NEWSTATE / LAMENTER / KEPCONVG | DONE agc_conic_newstate_, _lamenter_, _kepconvg_ universal.c |
| ITERATOR | DONE agc_conic_iterator lambert.c |
| KEPLERN (KEPLER) | DONE agc_conic_kepler kepler.c |
| KEPPREP, RVCON, CSMCONIC/LEMCONIC conic path | DONE agc_conic_kepprep/_rvcon/_csmconic/_lemconic kepler.c (LEMCONIC+SURFFLAG -> UNIMPLEMENTED) |
| LAMBERT | DONE agc_conic_lambert lambert.c |
| TIMETHET / COMMNOUT / COGAOVFL | DONE agc_conic_timethet, agc_conic_commnout_ timetheta.c |
| TIMERAD | DONE agc_conic_timerad timeradius.c |
| APSIDES | DONE agc_conic_apsides apsides.c |
| tests/nav/conic.c, tests/oracle/conic.c, tools/oracle/cases/nav_conic*.spec/.txt | DONE |

## Requested core additions

None outstanding. The TP operations requested earlier (TP shift with
overflow-corrected wrap, TP NORM, POLY) have landed in agc_interp.h and
the agc_conic_m*_ helpers now wrap agc_tp_shift, agc_tp_norm, agc_tp_round,
agc_tp_sqrt, agc_dp_sq_tp, agc_dp_mul_tp, agc_dp_poly and agc_vec_dot_tp.
* For core-numeric (observation): yaAGC gives DDV of (00200,77777) by
  (00313,31474) = 0.627988409 vs 0.628065 for the sign-agreeing
  (00200,00000)+1 -- a -0 minor word changes DDV by ~2^-14.
* Flags used (all present in agc_flags_common.h): AGC_FLAG_SOLNSW,
  AGC_FLAG_INFINFLG, AGC_FLAG_COGAFLAG, AGC_FLAG_APSESW, AGC_FLAG_ORDERSW,
  AGC_FLAG_CMOONFLG, AGC_FLAG_LMOONFLG, AGC_FLAG_SURFFLAG. Accessors used
  (present): agc_erase_rrectcsm/vrectcsm/tetcsm/rrectlem/vrectlem/tetlem.

## Requested jobs-tasks

None (conic subroutines run inside the caller's job).

## Open questions

* src/programs/navigation/pending.h (prog-rendz) declares its own agc_timethet_in_t,
  agc_lambert_in_t and an agc_conic_apsides with a different signature;
  those must be replaced by include/agc/nav/conic.h (adapt glue.c).
* nav-integ's agc_integ_kepprep (orbital_integration.c) already calls
  agc_conic_kepprep + agc_conic_kepler with RCV/VCV vs RRECT/VRECT split
  correctly; agc_conic_rvcon/csmconic/lemconic are the stand-alone
  (rectified-state) versions for callers outside the integration package.
* LM LAMBERT callers must supply in->iterctr (SETITCTR: 20, or 5 when
  AVEGFLAG); the CSM ignores it.

