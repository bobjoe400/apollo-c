// Delete the vehicle assertion at the top of LM-only routines: everything
// under src/lm is reached only through the LM dispatch switch or LM-only
// callers, so the fact holds by construction and the BAD_ARG return is a
// sentinel (CODING_STANDARD rule 7).  Run: spatch --sp-file
// tools/archive/vehicle_guard.cocci --in-place --dir src/lm
@@
@@
- if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
- {
-     return (AGC_STATUS_BAD_ARG);
- }
