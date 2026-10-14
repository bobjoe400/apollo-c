# Assignment: nav-aux   (tier: translation)

Scope, into src/nav/ with a header you create, include/agc/nav/auxiliary.h:
- LATITUDE_LONGITUDE_SUBROUTINES.agc (LAT-LONG, LALOTORV, Fischer
  ellipsoid / lunar radius logic) -> latlong.c
- PLANETARY_INERTIAL_ORIENTATION.agc (RP-TO-R, R-TO-RP, earth/moon
  rotation matrices, 504LM libration terms, EARTHROT/MOONROT) -> planetary.c
- MEASUREMENT_INCORPORATION.agc (INCORP1/INCORP2, W-matrix measurement
  update, DELTAQ/BVECTOR) -> measurement.c
- TIME_OF_FREE_FALL.agc (CALCTFF, CALCTPER, TFF/TPER/TFFCONIC/VGAMCALC and
  the special cases) -> free_fall.c
- LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc (LSPOS; the CM and LM
  versions differ: CM uses stored ephemeris coefficients, LM an analytic
  form -- translate both, select by agc_state_vehicle()) -> ephemerides.c,
  exposing exactly:
  agc_status_t agc_lsposition(agc_interp_t *ip, agc_dp_t time_cs,
                              agc_vec_t *moon_pos, agc_vec_t *sun_pos);
  (nav-integ codes against this name; document scales.)
- STAR_TABLES.agc (check both vehicles) -> star_tables.c, static const unit
  vectors with catalogue numbers and names in comments.
Diff CM vs LM, translate once where identical. Structs/params, not globals;
scale factors in comments; constants with octal and engineering values.

Tests: tests/nav/auxiliary.c (compiles now): lat/long of an equatorial point
at a known time; RP-TO-R round trip; TFF for a circular orbit (no-perigee
special case); star table unit length. List oracle cases in your note.
