# Assignment: nav-conic   (tier: translation)

Scope: CONIC_SUBROUTINES.agc (Comanche pp. 1262-1308; diff against
Luminary's copy, note differences) into src/nav/conic/conic*.c (common.c,
kepler.c, lambert.c, timetheta.c, timeradius.c,
apsides.c as needed) with a header you create, include/agc/nav/conic.h
(section "conic").

Routines: CSMCONIC/LEMCONIC wrappers, KEPPREP, KEPLER, LAMBERT, TIMETHET,
TIMERAD, APSIDES, and the lower-rank shared subroutines (DELTIME, GEOM,
PARAM, the universal-variable S and C evaluations, COMMNOUT, ...).

Inputs/outputs through explicit structs (agc_kepler_in_t: rrect vrect tau
xkepnew tc xprev body; agc_kepler_out_t: rcv vcv tc xprev plus the useful
"debris" urrect r1 alpha tmodulo), not global erasables. Preserve the
documented scales (2^29/2^27 m, 2^7/2^5 m/cs, 2^28 cs, 2^17/2^16 sqrt m) in
struct docs. MUTABLE (earth/moon mu and associated scaling) as static const
with AGC octal values. Keep iteration logic exactly (XKEPNEW guess,
EPSILONT test, ITERCTR bound as the loop bound, modulo-period handling);
no convergence -> AGC_STATUS_NO_CONVERGE plus the flag/alarm the AGC sets.

Tests: tests/nav/conic.c (compiles now, runs once core exists): circular
LEO Kepler quarter-period vs analytic; Lambert between two points on that
orbit; apsides of an ellipse. Use agc_dp_from_double for inputs.

Note section "Oracle cases": entry label, input erasables and output
erasables for one Kepler and one Lambert case (see tools/oracle/README.md).
