# nav-conic -- CONIC_SUBROUTINES.agc (+ KEPPREP, CSMCONIC/LEMCONIC conic path)

## Scope

* Comanche055 CONIC_SUBROUTINES.agc pp. 1262-1308 (Luminary099 pp. 1159-1204).
* KEPPREP (ORBITAL_INTEGRATION.agc p. 1334-1335, both programs, identical).
* The conic branch of CSMCONIC/LEMCONIC (INTEGRATION_INITIALIZATION.agc
  p. 1318-1322: INTSTALL -> INTEGRV1 -> ALOADED -> RVCON -> RECTIFY ->
  KEPPREP -> KEPLERN -> TET update). The integration lock, A-memory and
  TESTLOOP/INTEXIT bookkeeping belong to nav-integ.

C target: include/agc/agc_nav.h (section "conic"), src/nav/conic_internal.h,
src/nav/conic_common.c, conic_kepler.c, conic_lambert.c, conic_timetheta.c,
conic_timeradius.c, conic_apsides.c; tests/test_conic.c.

### Comanche vs Luminary (code-only diff, comments stripped)

| Place | Comanche 055 | Luminary 099 | C treatment |
|-------|--------------|--------------|-------------|
| everywhere | `STCALL x / label` | `STORE x / GOTO label` | identical semantics; one translation |
| LAMBERT entry | `SSP ITERCTR 20D` | `CLEAR SOLNSW` (ITERCTR set by caller, SETITCTR in ORBITAL_INTEGRATION: 20, or 5 if AVEGFLAG) | `agc_state_vehicle()` test; LM takes `in->iterctr` |
| SUFFCHEK tolerance | `TDESIRED * D1/4` (25 %) | `TDESIRED * BEE17` (2^-17) | vehicle test |
| SUFFCHEK failure | `SETGO SOLNSW RTNLAMB` (no velocity) | `SETGO SOLNSW INITV` (velocity computed anyway) | vehicle test |
| INITV | `CLEAR SOLNSW` | (cleared at entry instead) | vehicle test |
| MUTABLE | in CONIC_SUBROUTINES | in CONTROLLED_CONSTANTS, `2DEC*` for MUM too | assembled octal identical (checked in both listings) |

## Design table

Scales: E/M = earth/moon central body. "rev" not used here.

| AGC label (page) | Inputs (scale) | Outputs (scale) | C function |
|---|---|---|---|
| KEPLERN p.1277-1282 | RRECT 2^29/2^27 m, VRECT 2^7/2^5 m/cs, TAU. 2^28 cs, XKEPNEW 2^17/2^16 sqrt m, TC 2^28, XPREV 2^17/2^16, X1 body | RCV, VCV, TC, XPREV; debris URRECT 2^1, R1 2^29/2^27, ALPHA 2^-22/2^-20 1/m, TAU. reduced | `agc_conic_kepler()` (conic_kepler.c) |
| KEPPREP p.1334-1335 (ORBITAL_INTEGRATION) | RCV(=RRECT), VCV, TAU., TC, XKEP | XKEPNEW 2^17/2^16 | `agc_conic_kepprep()` |
| CSMCONIC/LEMCONIC conic path (RVCON) p.1318-1322 | state vector, TDEC 2^28 | RATT/VATT/TAT equivalent | `agc_conic_rvcon()`, `agc_conic_csmconic()`, `agc_conic_lemconic()` |
| DELTIME p.1283-1284 | XI 2^6, 0D=XSQ 2^34/2^32 -N1, X, KEPC1 2^17/2^16, KEPC2 2^6, R1 | T 2^28, S(XI) 2^1, XSQC(XI) 2^33/2^31 | `agc_conic_deltime_()` (internal) |
| ITERATOR p.1285-1286 | INDEP=COGA, DEP=T, DEPREV, DELDEP, MAX, MIN, TWEEKIT, SLOPESW, ORDERSW | DELINDEP | `agc_conic_iterator()` (public) |
| CHECKCTR p.1286 | ITERCTR | ITERCTR-1 | inline decrement (bounded loop counter) |
| NEWSTATE / LAMENTER p.1287 | UR1, VVEC, R1, X, XI, S, XSQC, T | R2VEC 2^29/2^27 (0D), V2VEC 2^7/2^5 (MPAC) | `agc_conic_newstate_()`, `agc_conic_lamenter_()` |
| TIMETHET p.1289 | RVEC, VVEC, SNTH 2^1, CSTH 2^1, RVSW | T 2^28, (R2,V2), INFINFLG, COGAFLAG | `agc_conic_timethet()` |
| COMMNOUT p.1289 | XI, flags | T, state | `agc_conic_commnout_()` |
| PARAM p.1289-1290 | RVEC (0D), VVEC (MPAC) | P 2^4, R1A 2^6, COGA 2^5, R1, UR1, U2, UN, MAGVEC2 | `agc_conic_param()` (public: P37 calls it) |
| GEOM p.1291 | R1VEC (0D), V2VEC (MPAC), GEOMSGN, NORMSW, UN | U2, MAGVEC2, UR1, R1, CSTH 2^1 (0D), UN, SNTH 2^1 | `agc_conic_geom_()` |
| GETX p.1292-1295 (WLOOP, POLYCOEF, XCOMMON, 360CHECK, INVRSEQN, 1/WLOOP, TRUE360X, INFINITY) | P 2^4, CSTH, SNTH, COGA, R1A, R1 | X, XI, 0D=XSQ, KEPC1, KEPC2, INFINFLG, 360SW | `agc_conic_getx_()` + helpers |
| LAMBERT p.1296-1300 (LAMBLOOP, NEGP, HIENERGY, COMMONLM, BIGTIME, LOENERGY, SUFFCHEK, 360LAMB, NOGUESS, LOLIM, INITV, TARGETV) | R1VEC, R2VEC, TDESIRED 2^28, GEOMSGN, GUESSW, COGA, NORMSW, UN, VTARGTAG | VVEC, VTARGET, SOLNSW, debris | `agc_conic_lambert()` (conic_lambert.c) |
| TIMERAD p.1301-1302 (TERMNVEC, CIRCULAR, BADR2) | RVEC, VVEC, RDESIRED 2^29/2^27, SGNRDOT, RVSW | T, (R2,V2), APSESW, SOLNSW, INFINFLG, COGAFLAG, CSTH, SNTH | `agc_conic_timerad()` |
| APSIDES p.1303 (GETECC, INFINAPO) | RVEC, VVEC | RA 2^29/2^27 (MPAC), RP (0D), ECC 2^3 | `agc_conic_apsides()` |
| ABTCONIC p.1304 | - | POODOO 00607 | `agc_poodoo(0607)` + `AGC_STATUS_POODOO` |
| MUTABLE p.1305 | - | MU, 1/MU, SQRT MU, 1/SQRT MU, E and M | `static const` table, octal in comments |

Interpreter-level conventions used throughout (verified against
INTERPRETER.agc and by re-deriving KEPPREP's scale chain):
* indexed addresses subtract the index register (`MUTABLE,1` with X1=-2 is MUTABLE+2);
* `NORM Xn` stores the NEGATIVE of the left-shift count, so `SL* 0 k,1` is a
  left shift by k+N1 and `SR* 0 k,1` a right shift by k+N1 (negative = other way);
* `TIX,2` with X2=3, S2=1 makes WLOOP and 1/WLOOP run exactly 3 passes;
* `STADR` is a no-op that lets a store follow a push-down operand.

## Progress

| Routine / table | Status |
|---|---|
| agc_nav.h (conic section) | DONE include/agc/agc_nav.h |
| conic_internal.h, helpers (TP MPAC model agc_conic_m*_, POLY) | DONE src/nav/conic_internal.h, conic_common.c |
| MUTABLE table | DONE mutable_tab conic_common.c |
| DELTIME | DONE agc_conic_deltime_ conic_universal.c |
| GEOM / PARAM | DONE agc_conic_geom_, agc_conic_param_, agc_conic_param conic_geom.c |
| GETX (WLOOP/INVRSEQN/1/WLOOP/POLYCOEF/XCOMMON/TRUE360X/INFINITY) | DONE agc_conic_getx_ + static helpers conic_geom.c |
| NEWSTATE / LAMENTER / KEPCONVG | DONE agc_conic_newstate_, _lamenter_, _kepconvg_ conic_universal.c |
| ITERATOR | DONE agc_conic_iterator conic_lambert.c |
| KEPLERN (KEPLER) | DONE agc_conic_kepler conic_kepler.c |
| KEPPREP, RVCON, CSMCONIC/LEMCONIC conic path | DONE agc_conic_kepprep/_rvcon/_csmconic/_lemconic conic_kepler.c (LEMCONIC+SURFFLAG -> UNIMPLEMENTED) |
| LAMBERT | DONE agc_conic_lambert conic_lambert.c |
| TIMETHET / COMMNOUT / COGAOVFL | DONE agc_conic_timethet, agc_conic_commnout_ conic_timetheta.c |
| TIMERAD | DONE agc_conic_timerad conic_timeradius.c |
| APSIDES | DONE agc_conic_apsides conic_apsides.c |
| tests/test_conic.c | TODO |

## Requested core additions

* `agc_tp_t agc_tp_shift(agc_interp_t *ip, agc_tp_t v, int n);` -- SL/SR on
  the TP MPAC (n>0 left with OVFIND, n<0 truncating right). Interim local
  version: `agc_conic_tp_shift_()` in src/nav/conic_common.c.
* `agc_tp_t agc_tp_norm(agc_tp_t v, int *nshift);` -- NORM on the TP MPAC
  (left-justify until |v| >= 1/2, *nshift = left shifts; the AGC stores
  -*nshift in the index register). Interim: `agc_conic_tp_norm_()`.
* Flags (naming convention, header agc_flags_common.h not yet present):
  AGC_FLAG_SOLNSW, AGC_FLAG_INFINFLG, AGC_FLAG_COGAFLAG, AGC_FLAG_APSESW.

## Requested jobs-tasks

None (conic subroutines run inside the caller's job).
