# Assignment: nav-integ   (tier: translation)

Scope: INTEGRATION_INITIALIZATION.agc and ORBITAL_INTEGRATION.agc (Comanche
pp. 1309-1354; diff against Luminary; translate once, note differences)
into src/nav/integration/integration_init.c and src/nav/integration/orbital_integration.c with a
header you create, include/agc/agc_integ.h.

Routines: INTEGRV/INTEGRVS/INTSTALL/INTWAKE (the integration lock and
sleep/wake protocol, via include/agc/agc_exec.h); the Encke method
(RECTIFY, ACCOMP, disturbing acceleration with earth J2-J4 and the
moon/earth/sun third-body terms, the Nystrom step, TIMESTEP selection,
ORBCHGO); the W-matrix integration path (DIM0FLAG/D6OR9FLG); state-vector
selection (CSM vs LM, moon vs earth sphere of influence); termination and
alarm paths. Explicit context struct (agc_integ_ctx_t: RCV VCV DELTAV NUV
TC TDELTAV TNUV XKEP ..., W matrix as a fixed array) by pointer; scale
factors in comments.

Ephemeris: agent nav-aux implements
  agc_status_t agc_lsposition(agc_interp_t *ip, agc_dp_t time_cs,
                              agc_vec_t *moon_pos, agc_vec_t *sun_pos);
declare it in agc_integ.h with that exact signature and documented scales;
put any changes you need under "Requested core additions".

Tests: tests/test_integration.c (compiles now): two-body-only LEO
propagation over 10 minutes vs analytic circular motion. List oracle cases
in your note.
