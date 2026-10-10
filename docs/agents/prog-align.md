# Assignment: prog-align   (tier: translation)

Scope, into src/programs/ (shared) and src/csm/, src/lm/ (vehicle
specific), with include/agc/programs/rendezvous.h section "alignment" (the
prog-rendz agent creates that header in parallel: create
include/agc/programs/alignment.h instead to avoid conflicts):
P51-P53.agc (both; diff): IMU orientation determination P51, P52 realign
with the four options (preferred, nominal, REFSMMAT, landing site), P53/P54
backup; the star selection routine R56? (check), the sighting mark
processing S52.2/S52.3; SXTMARK.agc (Comanche sextant/scanning telescope
marks, the optics CDU handling) -> src/csm/optics/sxtmark.c; AOTMARK.agc
(Luminary alignment optical telescope marks, the cursor/spiral logic) ->
src/lm/aotmark.c; R63.agc (Luminary rendezvous final attitude);
CSM_GEOMETRY.agc / LEM_GEOMETRY.agc (NB-to-SM matrices, optics/AOT
geometry constants); S-BAND_ANTENNA_FOR_CM.agc and S-BAND_ANTENNA_FOR_LM.agc
(antenna pointing angles); LUNAR_LANDMARK_SELECTION_FOR_CM.agc; AGS_INITIALIZATION.agc
(Luminary, R47 AGS initialisation downlink). Star vectors come from the
nav-aux star table (include/agc/nav/auxiliary.h; code against
`agc_star_vector(index, &vec)` and list it if the name differs).
Tests: tests/test_align.c (compiles): P52 option 3 with identity REFSMMAT
and two orthogonal star sightings gives identity; S-band antenna angles for
earth directly along the +X body axis.
