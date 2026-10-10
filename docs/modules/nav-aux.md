# nav-aux -- lat/long, planetary orientation, measurement incorporation, TFF, ephemerides, star tables

## Scope

| AGC file (Comanche055 pages) | CM vs LM | C file |
|---|---|---|
| LATITUDE_LONGITUDE_SUBROUTINES.agc (pp. 1236-1242) | identical code (LM keeps ERAD/504RM in CONTROLLED_CONSTANTS) | src/nav/latlong.c |
| PLANETARY_INERTIAL_ORIENTATION.agc (pp. 1243-1251) | identical code (LM: LO6ZEROS for HI6ZEROS, bank split only) | src/nav/planetary.c |
| MEASUREMENT_INCORPORATION.agc (pp. 1252-1261) | identical code (NEWZCOMP bank split only) | src/nav/measurement.c |
| TIME_OF_FREE_FALL.agc (pp. 1373-1388) | identical code (LM keeps 1/RTMU in CONTROLLED_CONSTANTS) | src/nav/free_fall.c |
| LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc (CM pp. 785-788, LM pp. 984-987) | DIFFERENT: CM 9th-degree lunar polynomial + rotating sun vector; LM analytic mean-element unit vectors | src/nav/ephemerides.c |
| STAR_TABLES.agc (CM pp. 1389-1393) / LM CONTROLLED_CONSTANTS.agc pp. 49-51 | identical 37-star table, CATLOG 6970 | src/nav/star_tables.c |

Public header: include/agc/agc_navaux.h. Private helpers: src/nav/navaux_internal.h + src/nav/navaux_util.c.
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
| INCORP1 p.1252 (+NEWZCOMP p.1260) | agc_incorp1 | W (agc_incorp_w_t), BVECTOR[3], DELTAQ (DP), VARIANCE (TP), DMENFLG | ZI[3], OMEGA[3], DELTAX[3], GAMMA, NORMGAM |
| INCORP2 FAZA..FAZB p.1256 | agc_incorp2_wupdate | W, ZI, OMEGA, GAMMA, NORMGAM | W updated in place |
| INCORP2 FAZC..FAZAB5 p.1259 | agc_incorp2_state | DELTAX, X789, TDELTAV/TNUV/RCV/VCV, MOONTHIS | updated integration temps, rectify request, X789 |
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

## Progress

| Routine / table | Status |
|---|---|
| header agc_navaux.h | DONE include/agc/agc_navaux.h |
| navaux_util.c helpers | DONE src/nav/navaux_util.c (compiles) |
| planetary.c (NEWANGLE EARTHMX EARTHL MOONMX R-TO-RP RP-TO-R, pad load) | DONE agc_planet_* src/nav/planetary.c (compiles, not yet run) |
| latlong.c (LAT-LONG LALOTORV GETERAD ARCTAN SETGAMMA SETRE) | DONE agc_latlong_* src/nav/latlong.c (compiles, not yet run) |
| free_fall.c (TFFCONIC TFFRP/RA CALCTFF CALCTPER T(X)) | TODO |
| measurement.c (INCORP1 NEWZCOMP INCORP2) | TODO |
| ephemerides.c (CM LSPOS LUNPOS LUNVEL SOLPOS, LM LSPOS, agc_lsposition) | TODO |
| star_tables.c | TODO |
| tests/test_navaux.c | TODO |

## Deviations

(filled in as code is written)

## Stubs

(none yet)

## Requested core additions

* Shared erasable accessors (pad load, currently module-static in
  planetary.c / ephemerides.c behind agc_navaux_set_padload /
  agc_ephem_set_csm_data): agc_erase_tephem(), agc_erase_azo(),
  agc_erase_axo(), agc_erase__ayo() (-AYO), agc_erase_504lm(), agc_erase_rls(),
  agc_erase_timemo(), agc_erase_vecoem(), agc_erase_reso(), agc_erase_veso(),
  agc_erase_omegaes().
* A TP shift / TP normalize in agc_interp.h (agc_tp_shl, agc_tp_norm) would
  remove two raw-representation helpers in navaux_util.c.

## Requested jobs-tasks

None (INCORP2's PHASCHNG/GRP2PC restart protection and POSTJUMP INTWAKE
belong to the caller / nav-integ; see Deviations).

## Tests written

TODO

## Oracle cases

TODO

## Open questions

* VGAMCALC and TFF/TRIG are in P61-P67.agc (CM entry), not in
  TIME_OF_FREE_FALL.agc; left to guid-entry.
