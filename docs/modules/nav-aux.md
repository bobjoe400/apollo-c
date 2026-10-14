# nav-aux -- lat/long, planetary orientation, measurement incorporation, TFF, ephemerides, star tables

History of this lane: [docs/history/lanes/nav-aux.md](../history/lanes/nav-aux.md)

## Scope

| AGC file (Comanche055 pages) | CM vs LM | C file |
|---|---|---|
| LATITUDE_LONGITUDE_SUBROUTINES.agc (pp. 1236-1242) | identical code (LM keeps ERAD/504RM in CONTROLLED_CONSTANTS) | src/nav/auxiliary/latlong.c |
| PLANETARY_INERTIAL_ORIENTATION.agc (pp. 1243-1251) | identical code (LM: LO6ZEROS for HI6ZEROS, bank split only) | src/nav/auxiliary/planetary.c |
| MEASUREMENT_INCORPORATION.agc (pp. 1252-1261) | identical code (NEWZCOMP bank split only) | src/nav/auxiliary/measurement.c |
| TIME_OF_FREE_FALL.agc (pp. 1373-1388) | identical code (LM keeps 1/RTMU in CONTROLLED_CONSTANTS) | src/nav/auxiliary/free_fall.c |
| LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc (CM pp. 785-788, LM pp. 984-987) | DIFFERENT: CM 9th-degree lunar polynomial + rotating sun vector; LM analytic mean-element unit vectors | src/nav/auxiliary/ephemerides.c |
| STAR_TABLES.agc (CM pp. 1389-1393) / LM CONTROLLED_CONSTANTS.agc pp. 49-51 | identical 37-star table, CATLOG 6970 | src/nav/auxiliary/star_tables.c |

Public header: include/agc/nav/auxiliary.h. Private helpers: src/nav/auxiliary/internal.h + src/nav/auxiliary/util.c.
All constants were verified word-for-word against the assembled ropes
(tools/oracle/ropes/*.bin read at the symtab address).

Scale notation: "2^N u" = fraction * 2^N in unit u (AGC "B-N" comment means
the same 2^N). cs = centiseconds. revs = revolutions.

## Design table

| AGC label | C function | inputs (scale) | outputs (scale) |
|---|---|---|---|
| LAT-LONG p.1236 | agc_latlong_from_r | R (2^29 m), T (2^28 cs), LUNAFLAG, ERADFLAG, pad load | LAT, LONG (revs 2^0), ALT (2^29 m), ERADM (2^29 m) |
| LALOTORV p.1238 | agc_latlong_to_r | LAT, LONG (revs), ALT (2^29 m), T | R (2^29 m) |
| GETERAD p.1239 | agc_latlong_geterad | 1/2 sin(lat) (2^1) | ERADM (2^29 m) Fischer ellipsoid |
| ARCTAN p.1240 | agc_latlong_arctan | SINTH, COSTH (2^1) | THETA revs 2^0 in [-1/2,1/2] |
| SETGAMMA p.1241 | agc_latlong_setgamma | LUNAFLAG | GAMRP (2^1) |
| SETRE p.1242 | agc_latlong_setre | flags, 1/2 sin lat, RLS (2^27 m) | ERADM (2^29 m) |
| RP-TO-R p.1243 | agc_planet_rp_to_r | RP (any scale), T, moon? | R = MT(RP + LP x RP), same scale |
| R-TO-RP p.1245 | agc_planet_r_to_rp | R, T, moon? | RP = M(R - L x R), same scale |
| MOONMX p.1246 | agc_planet_moonmx | T (2^28 cs), TIMSUBO (TEPHEM, 2^42 cs TP) | M (2^1) rows M0 M1 M2 |
| NEWANGLE p.1248 | agc_planet_newangle | X0 revs 2^0, XDOT rev/cs 2^-23/-28/-27, X1 = 0/5/4 | X revs 2^0 (mod 1 rev) |
| EARTHMX p.1249 | agc_planet_earthmx | AZO revs, WEARTH | M (2^1) |
| EARTHL p.1250 | agc_planet_earthl | AXO, -AYO (rad 2^0) | L = (-AX,-AY,0) rad 2^0 |
| INCORP1 p.1252 (+NEWZCOMP p.1260) | agc_incorp_state_deviation | W (agc_incorp_w_t), BVECTOR[3], DELTAQ (DP), VARIANCE (TP), DMENFLG | ZI[3], OMEGA[3], DELTAX[3], GAMMA, NORMGAM |
| INCORP2 FAZA..FAZB p.1256 | agc_incorp_update_w | W, ZI, OMEGA, GAMMA, NORMGAM | W updated in place |
| INCORP2 FAZC..FAZAB5 p.1259 | agc_incorp_update_state | DELTAX, X789, TDELTAV/TNUV/RCV/VCV, MOONTHIS | updated integration temps, rectify request, X789 |
| TFFCONIC/TFFCONMU p.1376 | agc_tff_conic | RONE (2^29/2^27 m), VONE (2^7/2^5 m/cs), 1/RTMU (2^-17/2^-14) | agc_tff_t (RMAG1, NRMAG, NR, TFFNP, TFFVSQ, TFFALFA, TFFRTALF, NA, TFF1/ALF, VONE') |
| TFFRP/RA p.1379 | agc_tff_rp_ra | agc_tff_t | RPER, RAPO (2^29/2^27 m; RAPO = NEARONE if invalid) |
| CALCTFF / CALCTPER p.1382 | agc_tff_calctff / agc_tff_calctper | RTERM or RPER (2^29/2^27 m) | TFF cs 2^28 (NEARONE = no free fall) |
| T(X) p.1387 | agc_tff_poly | X (2^0) | T(X) (2^0) |
| LSPOS/LUNPOS/LUNVEL/SOLPOS CM p.786 | agc_ephem_csm_lspos / _lunpos / _lunvel / _solpos | T (2^28 cs), TEPHEM, TIMEMO (2^42 cs TP), VECOEM, RESO (2^38 m), VESO (2^9 m/cs), OMEGAES (2^-26 rev/cs) | moon 2^29 m, moon vel 2^7 m/cs, sun 2^38 m |
| LSPOS LM p.985 | agc_ephem_lm_lspos | T (2^28 cs), TEPHEM | VMOON, VSUN unit (2^1) |
| (contract) | agc_lsposition | dispatches on agc_state_vehicle() | CM: moon 2^29 m, sun 2^38 m; LM: unit 2^1 both |
| STAR TABLE / CATLOG | agc_star_vector | star 1..37 | half-unit vector (2^1) |

W layout (agc_incorp_w_t): blk[b][m] is the 3-vector at AGC address
W + 54b + 6m, b = 0..2 (column triple), m = 0..8 (row). ZI = sum_b
MXV(rows blk[b][3a..3a+2]) BVECTOR[b]; OMEGA[b] = sum_a VXM(ZI[a], same).

## Key AGC semantics relied on (from INTERPRETER.agc)

* Interpretive indexing subtracts: `ADDR,1` = ADDR - C(X1).
* NORM (TSLC) stores the COMPLEMENT of the shift count; so `SL* 0,1` after
  `NORM X1` shifts by +n.  C code keeps positive counts n and writes the
  AGC index arithmetic out explicitly.
* PDDL / STODL / STOVL with no operand pop the push list BEFORE pushing.
* NEWANGLE relies on overflow wrap (mod 1 revolution) of SL and DAD;
  agc_interp saturates, so wrap is modelled with explicit helpers (DEVIATION).
* DDV uses only the DP part (MPAC, MPAC+1) of a TP dividend (LM LSPOS).

## Deviations

* util.c: TP shift / TP normalize / low-DP-of-TP and angle wrap
  helpers work on the integer representation, because agc_interp.h has no
  TP shift or normalize and its DP ops saturate where the AGC wraps.
* NEWANGLE (planetary.c) and LM LSPOS POSITB/POSITD (ephemerides.c): angle
  products are carried as exact TP (agc_dp_mul_tp) through the SL and
  rounded once; SL/DAD/DSU overflow is modelled as wrap mod 1 rev.
* INCORP1 (measurement.c): VSQ of ZI is formed from exact TP component
  products (the AGC's DOTSUB drops the lowest partial product).  ROUND /
  DMP / SQRT on TRIPA use the DP API (SQRT does not see MPAC+2).
* INCORP2 (measurement.c): restart protection and state-vector plumbing
  (PHASCHNG 04022, GRP2PC, UPFLAG REINTFLG, INTSTALL, MOVEP*/MOVEA*,
  SVDWN1/2, RECTIFY call, POSTJUMP INTWAKE) are the caller's job.  W rows
  are updated in place (no HOLDW staging); agc_incorp_update_state works on the
  integration temporaries in agc_incorp_sv_t and sets sv->rectify.
* TFFEL1 (free_fall.c): BOVB SIGNMPAC is the saturation agc_dp_shl already
  gives.  ENDTFF's BOV sees OVFIND accumulated since the last test, as in
  the AGC (the interpreter state is shared through ip).
* agc_dp_sign is taken as the AGC SIGN instruction (complement MPAC if the
  operand is negative), not copysign; GONEPAST in CALCTFF relies on it.
* agc_lsposition reads TEPHEM from agc_erase_tephem() (shared erasable);
  the other functions take TEPHEM from agc_navaux_padload_t.  Callers must
  keep the two in step until the pad load moves into core (below).
* Bank switching, EBANK, the X1/X2/S1/S2 table walks (TIX loops) and
  FREEFLAG / RCB-13 loop control in LM LSPOS are written as ordinary
  bounded loops or straight-line calls.

## Stubs

None.

## Tests written

tests/nav/auxiliary.c (CSM build, agc_state_init(AGC_VEHICLE_CSM)):
* LAT-LONG of an equatorial point (inertial +Y, 6 h after epoch, ERAD):
  lat 0, long = 1/4 rev - WEARTH t, alt 100 km.
* LALOTORV -> LAT-LONG round trip on the Fischer ellipsoid (Cape Kennedy).
* R-TO-RP -> RP-TO-R round trip, earth (AXO, -AYO) and moon (504LM).
* TFF circular orbit: RPER = RAPO = r; CALCTFF to 400k ft -> NEARONE
  (MAXTFF1, no conic to RTERM).
* TFF ellipse from apogee: CALCTFF vs Kepler (0.2%), CALCTPER = half
  period (Z indeterminate, TFFEL1 with D/N = 0).
* T(X): T(0) = 1/3 and the series at X = 0.5.
* Star table: 37 half-unit vectors, Polaris near +Z, bad indices rejected.
* INCORP1/INCORP2 scalar Kalman case (W = 0.25 I, B = 0.5 e1): DELTAX,
  W(0,0) update vs the closed form, TDELTAV update.
* CM LSPOS/LUNVEL with a constant + linear polynomial and fixed sun; LM
  LSPOS sun direction on 16 July 1969 within 0.02.

## Program flow and decisions

_Documentation wave (2026-10-12); prose and tables brought to the final tree (HEAD 3b97dc8) by the final pass (2026-10-14). The prose was written by reading the code; the two tables were extracted from the function bodies (flag, alarm, channel and scheduling calls; `agc_erase_*` accessors) and list only what the code does. Files: `src/nav/auxiliary/*.c`._

Utility subroutines for both vehicles, called inside the caller's job;
none raises an alarm. Their decisions are range and branch choices:

* **Ephemerides** (`src/nav/auxiliary/ephemerides.c`): the two ropes
  differ under the same labels. CSM (`agc_ephem_csm_lunpos`,
  `agc_ephem_csm_lunvel`, `agc_ephem_csm_solpos`): the moon from the
  ninth-degree polynomial VECOEM about TIMEMO, the sun by rotating RESO
  about RESO x VESO at OMEGAES; a time outside the fit span (|t| >= 2^26
  cs) is OVERFLOW. LM (`agc_ephem_lm_lspos`): LSPOS from mean elements
  (GTMP, LOM, LOS, LON kept as the words DAD / DSU leave). Outputs: moon
  2^29 m, sun 2^38 m, lunar velocity 2^7 m/cs. `agc_lsposition` picks the
  vehicle's routine.
* **Planetary** (`planetary.c`): `agc_planet_newangle` (angle at time t
  from TEPHEM; times before the epoch are BAD_ARG), `agc_planet_earthmx`,
  `agc_planet_moonmx`, `agc_planet_earthl`; `agc_planet_r_to_rp` /
  `agc_planet_rp_to_r` choose earth or moon frame on their flag argument
  (the `_words` forms keep the AGC's sign-disagreeing pairs).
* **Lat/long** (`latlong.c:agc_latlong_from_r`, `agc_latlong_to_r`):
  earth uses the pad radius or the Fischer ellipsoid (`agc_latlong_geterad`,
  `agc_latlong_setre`), moon 504RM or |RLS| (`agc_latlong_setgamma`); LAT,
  LONG, ALT are stored by `agc_navaux_set_latlong`.
* **Free fall** (`free_fall.c`: `agc_tff_conic`, `agc_tff_rp_ra`,
  `agc_tff_calctff`, `agc_tff_calctper`): no conic or an overflow returns
  NEARONE (AGC_TFF_NEARONE, "no free fall"), which the display routines
  show as such; |delta E| >= 90 deg takes the TFFEL1 ellipse form.
* **Measurement incorporation** (`measurement.c`):
  `agc_incorp_state_deviation` (INCORP1: Z = W^T B, A = Z.Z + VARIANCE,
  GAMMA, OMEGA, DELTAX; A = 0 is BAD_ARG, "no information"),
  `agc_incorp_update_w` (INCORP2 row by row, 9 rows with DMENFLG else 6),
  `agc_incorp_update_state` (state += DELTAX). The restart protection,
  REINTFLG and RECTIFY around INCORP2 belong to the caller (DEVIATION;
  nav-integ and the measurement jobs of prog-rendz).
* **Star tables** (`star_tables.c:agc_star_vector`): star 1..37 (BAD_ARG
  otherwise), the assembled words of the catalogue.

### Decision points by function

None of the lane's 59 functions tests a flag, raises an alarm or starts a job or task; its decisions are the numeric branches described above.

### Shared erasables read and written

Accessors of `include/agc/core/state.h` and the lanes' public headers called by this lane, with the number of its functions that read / write each. Lane-private erasables are `static` in the lane's files or in its state struct; their scales are on the declarations.

| erasable (accessor) | meaning and scale (accessor prolog) | read by | written by |
|---|---|---|---|
| `rls` | landing site vector, moon-fixed reference, meters, scale 2^27 (RLS; R30 'SCALED AT (-27)M'); used by P20/P22/P52 lat-long, R30, P57, landing | 1 | 1 |
| `tephem` | time of the ephemeris epoch (TEPHEM = TIMSUBO, TP, centiseconds, scale 2^42: L "CSEC B-42 (TRIPLE PRECISION)"; P11 sets it at liftoff) | 2 | 1 |

### AGC labels -> C functions

The "AGC" column above names the label each listed function translates; the complete label -> function -> file:line index for the lane (and the tree) is [docs/overview/xref.md](../overview/xref.md), generated by `tools/xref.py`.
