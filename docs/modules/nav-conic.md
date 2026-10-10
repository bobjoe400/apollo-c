# nav-conic -- CONIC_SUBROUTINES.agc (+ KEPPREP, CSMCONIC/LEMCONIC conic path)

## Scope

* Comanche055 CONIC_SUBROUTINES.agc pp. 1262-1308 (Luminary099 pp. 1159-1204).
* KEPPREP (ORBITAL_INTEGRATION.agc p. 1334-1335, both programs, identical).
* The conic branch of CSMCONIC/LEMCONIC (INTEGRATION_INITIALIZATION.agc
  p. 1318-1322: INTSTALL -> INTEGRV1 -> ALOADED -> RVCON -> RECTIFY ->
  KEPPREP -> KEPLERN -> TET update). The integration lock, A-memory and
  TESTLOOP/INTEXIT bookkeeping belong to nav-integ.

C target: include/agc/nav/conic.h (section "conic"), src/nav/conic/internal.h,
src/nav/conic/common.c (MUTABLE, MPAC model, flags, ABTCONIC),
universal.c (DELTIME, NEWSTATE, LAMENTER, KEPCONVG), geom.c
(GEOM, PARAM, GETX), kepler.c (KEPLERN, KEPPREP, RVCON, CSMCONIC,
LEMCONIC), lambert.c (LAMBERT, ITERATOR), timetheta.c
(TIMETHET, COMMNOUT, COGAOVFL), timeradius.c (TIMERAD),
apsides.c (APSIDES); tests/test_conic.c, tests/oracle_conic.c.

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
| KEPLERN p.1277-1282 | RRECT 2^29/2^27 m, VRECT 2^7/2^5 m/cs, TAU. 2^28 cs, XKEPNEW 2^17/2^16 sqrt m, TC 2^28, XPREV 2^17/2^16, X1 body | RCV, VCV, TC, XPREV; debris URRECT 2^1, R1 2^29/2^27, ALPHA 2^-22/2^-20 1/m, TAU. reduced | `agc_conic_kepler()` (kepler.c) |
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
| LAMBERT p.1296-1300 (LAMBLOOP, NEGP, HIENERGY, COMMONLM, BIGTIME, LOENERGY, SUFFCHEK, 360LAMB, NOGUESS, LOLIM, INITV, TARGETV) | R1VEC, R2VEC, TDESIRED 2^28, GEOMSGN, GUESSW, COGA, NORMSW, UN, VTARGTAG | VVEC, VTARGET, SOLNSW, debris | `agc_conic_lambert()` (lambert.c) |
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
* MPAC is modelled as a TP accumulator (agc_conic_m*_ helpers, one per
  interpretive op): DMP/DSQ leave TP, DAD/DSU/BDSU touch MPAC,+1 only and
  keep MPAC+2 (BDSU does NOT complement MPAC+2), shifts/NORM/TAD/ROUND use
  all three words, DP STORE/PUSH drop MPAC+2. Scalar SL that overflows sets
  OVFIND and keeps the "overflow-corrected" (wrapped, true-sign) value
  (TSSL p. 1154) -- the oracle confirmed this (KEPPREP hyperbola case).
* Interpretive GOTO/CALL through an erasable is indirect and switches EBANK
  from the pseudo-address (GOTOGE), which is why KEPLERN can mix E3/E4/E5.

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
| tests/test_conic.c, tests/oracle_conic.c, tools/oracle/cases/nav_conic*.spec/.txt | DONE |

## Deviations

* PERIODCH (KEPLER modulo-period loop) has no bound in the AGC; bounded at
  AGC_CONIC_PERIOD_MAX_PASSES = 4096 periods (asserts if reached).
* LAMBLOOP passes through NEGP/HIENERGY/LOENERGY do not count ITERCTR; the
  loop is bounded at AGC_CONIC_LAMBERT_MAX_PASSES = 630 and then goes to
  SUFFCHEK (cannot happen on an AGC path: DCOGA halves to 0 first).
* LAMBERT reads DCOGA (push-down 12D) and TERRLAMB (erasable) before
  setting them if the first pass exits through NEGP; the AGC uses leftovers.
  Here DCOGA = 0 and TERRLAMB = TDESIRED, so that exit reports SOLNSW.
* CSM SUFFCHEK failure / 360LAMB: the AGC leaves VVEC/VTARGET unchanged;
  here they are zero.
* KEPLER TIMEOVFL exit through KEPRTN (DELX halved to 0): RCV/VCV are not
  computed by the AGC; out->rcv/vcv = in->rrect/vrect (what RECTIFY left in
  RCV/VCV), status AGC_STATUS_OVERFLOW.
* MPAC sign-mixed words are not modelled (an agc_tp_t is one number): a
  DMP of a sign-mixed MPAC or BDSU's uncomplemented MPAC+2 can differ from
  the AGC in the last bits before a SL/SQRT (APSIDES ECC differs by ~40
  LSB in the oracle for this reason).
* SQRT of a negative argument (SQRTABRT, POODOO 01302) is only asserted:
  every conic SQRT argument is tested or built non-negative first.
* ITERATOR reads TWEEKIT as the DP (40D, 41D) = (TWEEKIT, S2) with S2 = 1.
* SSP constants are the ASSEMBLED words: "SSP TWEEKIT 20000" is 10000
  (.25) and "SSP GEOMSGN 37777" is 27777 in both assembled ropes
  (Comanche055.va.lst / Luminary099.va.lst; the ropes are checksum-
  verified against the flight listings). Using .5 made LM LAMBERT with a
  small ITERCTR take a different branch than the AGC.
* RVCON/CSMCONIC/LEMCONIC: the state vector has no TDELTAV/TNUV, so it is
  taken as rectified (RCV = RRECT); TET += TC; output stays in the body's
  scale (RECTOUT's RATT/VATT rescale to 2^29/2^7 and INTSTALL/INTEXIT
  bookkeeping are nav-integ's). LEMCONIC with SURFFLAG (USEPIOS) returns
  AGC_STATUS_UNIMPLEMENTED.
* ABTCONIC (POODOO 00607) returns AGC_STATUS_POODOO after agc_poodoo();
  the AGC never returns. Output flags and debris are filled first.
* Ones' complement -0 is not representable: the AGC's DDV of a dividend
  with a -0 minor word (KEPLERN ALPHA for an exactly circular orbit, MPAC
  = 00200 77777) comes out ~2^-14 low; the C gives the exact quotient. In
  the oracle this moves circular-orbit KEPLER results by up to ~12000 LSB
  (VCV) and, through the period, the 2.25-period case by 200 cs.

## Stubs

* LEMCONIC when SURFFLAG is set (USEPIOS path, owned by nav-integ):
  AGC_STATUS_UNIMPLEMENTED. Nothing else is stubbed.

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

## Tests written

* tests/test_conic.c -- analytic two-body checks (earth, CSM build):
  KEPPREP+KEPLER quarter period forward/backward and 2.25 periods
  (PERIODCH), LAMBERT 90 degrees in a quarter period (VVEC, VTARGET,
  COGA ~ 0) and 360LAMB, APSIDES ellipse + hyperbola (RA = DPPOSMAX),
  TIMETHET 90 degrees, TIMERAD to r = a vs Kepler's equation, RVCON.
* tests/oracle_conic.c -- 31 Comanche055 + 4 Luminary099 cases against the
  yaAGC dumps (tolerance one SP LSB, see Deviations for the -0 case).
* Both run against the real core (src/core, built in a scratch dir):
  test_conic 40/40, oracle_conic 299/299. With the real core KEPPREP,
  TIMETHET, TIMERAD and most KEPLER outputs are bit-exact; the rest are
  within a few hundred LSB except the -0 / sign-mixed-word effects noted
  under Deviations.

## Oracle cases

Specs: tools/oracle/cases/nav_conic.spec (Comanche055) and
nav_conic_lm.spec (Luminary099); dumps nav_conic.txt / nav_conic_lm.txt.
Method: `entry INTPRET`, `q <label>`, `fbank <its bank>`; the return
address is the interpretive address of ABTCONIC (CM 12,3751 -> 25751; LM
12,3742 -> 25742) put in QPRET = work-area word 42D (`set VAC1+42=...`),
for KEPLERN directly in KEPRTN; `stop POODOO` (ABTCONIC = EXIT, TC POODOO).
X1 = work-area 38D: 77775 (-2, earth) or 77765 (-10D, moon).

* KEPLER, case kep_leo_q (via KEPPREP, bank 11, ebank 3):
  inputs RCV = RRECT = (00313,31474, 0,0, 0,0) [6678136 m, 2^29],
  VCV = VRECT = (0,0, 23241,00170, 0,0) [77.26 m/cs, 2^7],
  TAU. = 00010,11143 [135779 cs, 2^28], TC = XKEP = 0, PBODY = 0,
  FLAGWRD0 = 0 (MOONFLAG clear). Outputs: XKEPNEW 00773 15045 [2^17],
  RCV (77777 77637, 00313 31440, 00000 00000) [2^29],
  VCV (54536 50777, 77777 64777, 00000 77777) [2^7], TC = TAU. =
  00010 11143, XPREV = XKEPNEW. Direct KEPLERN entry (kepn_* cases): inputs
  RRECT, VRECT, TAU., XKEPNEW, TC, XPREV; KEPRTN = 25751.
* LAMBERT, case lam_3d (bank 12, ebank 5): R1VEC (7000 km, 0, 0),
  R2VEC (-2000, 8000, 1000 km) [2^29], TDESIRED 250000 cs [2^28],
  GEOMSGN 20000, VTARGTAG 0, FLAGWRD1 = 00002 (GUESSW), FLAGWRD7 = 0
  (NORMSW clear; 01000 with UN for the lam_un_* cases). Outputs: VVEC,
  VTARGET [2^7], COGA [2^5], P [2^4], R1A [2^6], FLAGWRD5 (SOLNSW = 00004),
  FLAGWRD8. LM: ITERCTR = work-area 22D set by the caller.
* Also TIMETHET (RVEC, VVEC, SNTH, CSTH, FLAGWRD7 RVSW 00400; out T = 30D,
  R2 = 0D-5D, V2 = MPAC, INFINFLG/COGAFLAG in FLAGWRD8), TIMERAD
  (RDESIRED, SGNRDOT; also CSTH, SNTH, APSESW, SOLNSW) and APSIDES (out
  MPAC = RA, 0D = RP, ECC).
* Agreement (real core): XKEPNEW/TC/T exact, positions within ~100 LSB,
  velocities within ~800 LSB (VSL8 amplifies) except the circular -0 cases,
  flags and abort paths (INFINFLG, COGAFLAG/COLINEAR, CIRCULAR, APSESW, LM
  SUFFCHEK with VVEC) match. Scratch harness: session scratchpad
  nav-conic/run.sh, realrun.sh.

## Open questions

* src/programs/rendezvous/ext.h (prog-rendz) declares its own agc_timethet_in_t,
  agc_lambert_in_t and an agc_conic_apsides with a different signature;
  those must be replaced by include/agc/nav/conic.h (adapt glue.c).
* nav-integ's agc_integ_kepprep (orbital_integration.c) already calls
  agc_conic_kepprep + agc_conic_kepler with RCV/VCV vs RRECT/VRECT split
  correctly; agc_conic_rvcon/csmconic/lemconic are the stand-alone
  (rectified-state) versions for callers outside the integration package.
* LM LAMBERT callers must supply in->iterctr (SETITCTR: 20, or 5 when
  AVEGFLAG); the CSM ignores it.
