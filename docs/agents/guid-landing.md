# Assignment: guid-landing   (tier: translation)

Scope (Luminary only), into src/lm/ with include/agc/agc_lm_landing.h:
- LUNAR_LANDING_GUIDANCE_EQUATIONS.agc (P63/P64/P66 guidance: the
  quadratic/explicit guidance law, the target tables, LPD angle, the
  throttle command interface, the 1202/1201 context (where tasks are
  scheduled), the landing radar updates) -> landing_guidance.c
- THE_LUNAR_LANDING.agc (P63 ignition algorithm, P63/P64 entry, P66/P67
  manual phases, the landing-site redesignation) -> lunar_landing.c
- THROTTLE_CONTROL_ROUTINES.agc (THROTTLE, the DPS throttle law with
  the 10%/92.5% regions, the "throttle-up" logic) -> throttle.c
- P70-P71.agc (landing aborts to orbit) -> p70_p71.c
- P12.agc and ASCENT_GUIDANCE.agc (lunar ascent, the ascent guidance
  equations, the tail-off, the insertion targeting) -> ascent.c
- LANDING_ANALOG_DISPLAYS.agc (altitude/altitude-rate meters) -> landing_displays.c
- FINDCDUW--GUIDAP_INTERFACE.agc (commanded attitude -> CDU angles for
  the DAP, with the gimbal-lock and the window-pointing constraints) ->
  findcduw.c
Use agc_channels.h for radar/throttle channels, agc_exec.h for jobs/tasks,
the agc_dsky.h display interface as in docs/agents/guid-powered.md.
Tests: tests/test_landing.c (compiles): throttle law at a commanded
thrust in the 10%-60% band returns the expected pulse count; guidance with
the vehicle exactly on a target trajectory commands zero correction.
