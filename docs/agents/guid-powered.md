# Assignment: guid-powered   (tier: translation)

Scope, into src/guidance/ with include/agc/guidance/guidance.h (you create it):
- POWERED_FLIGHT_SUBROUTINES.agc (CDUTRIG, FINDCDU/CALCGTA interplay,
  READACCS/PIPASR accelerometer reading, AVETOMID/MIDTOAVE, the average-G
  routine CALCRVG, the "flashing" and thrust direction helpers) ->
  powered_flight.c
- P30-P37 (Comanche P30-P37.agc, Luminary P30_P37.agc): external delta-V
  P30, Lambert aim-point P31 if present, and the common S30.1/S31.1
  targeting -> p30.c
- P40-P47 (both vehicles, diff): P40 SPS/DPS, P41 RCS, P42 APS, P47
  thrust monitor; S40.1/S40.2/S40.8/S40.9 steering (cross-product
  steering), the TIG-30/TIG-5 countdown, ullage, engine on/off, the
  pre-ignition display sequence -> p40.c (shared) with vehicle-specific
  parts in src/csm/burns/sps_burn.c and src/lm/burns/dps_burn.c
- BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc (Luminary) -> src/lm/burns/ignition.c
- SERVICER (Luminary SERVICER.agc: the 2-second average-G loop, PIPA
  readings, state update, the landing/ascent branches) and SERVICER207
  (Comanche) -> servicer_lm.c / servicer_csm.c, with the shared
  average-G integration in servicer_common.c
Engine/thrust commands go through agc_channels.h; the executive/waitlist
through agc_exec.h; display requests through the agc_dsky.h interface
(agc_dsky_display(verb, noun, continuation ids); if the header is not yet
present, declare locally what you need and list it).
Tests: tests/guidance/guidance.c (compiles): average-G with zero delta-V over
one 2-s step moves position by v*dt within precision; cross-product steering
with velocity-to-be-gained along thrust gives zero commanded rate.
