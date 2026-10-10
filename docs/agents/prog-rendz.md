# Assignment: prog-rendz   (tier: translation)

Scope (both vehicles; diff and share), into src/programs/ with
include/agc/programs/rendezvous.h (you create it; section "rendezvous"):
P20-P25.agc (rendezvous navigation P20, the R22 incorporation, the R21/
R23 marking, the P21 ground track, P22/P23 (CM) landmark and cislunar nav,
P24, P25; the LM radar versions), P32-P33_P72-P73.agc and
P32-P35_P72-P75.agc (CSI/CDH targeting), P34-35_P74-75.agc (TPI/TPM), R30.agc
(orbit parameter display), R31.agc (rendezvous parameter display),
STABLE_ORBIT.agc (P38/P39), TPI_SEARCH.agc (Comanche), P76.agc (target
delta-V), GROUND_TRACKING_DETERMINATION_PROGRAM.agc (P21),
GENERAL_LAMBERT_AIMPOINT_GUIDANCE.agc (Luminary), RADAR_LEADIN_ROUTINES.agc
(Luminary). These call the conic routines (include/agc/nav/conic.h, written
in parallel by nav-conic; if not present, code against the struct names in
docs/agents/nav-conic.md) and integration (agc_integ.h). Display through
the agc_dsky.h interface; scheduling through agc_exec.h.
Tests: tests/programs/rendezvous.c (compiles): CSI targeting on a coplanar
circular case gives a delta-V of the expected sign; R30 apogee/perigee of
a circular orbit are equal within precision.
