# Assignment: dap-lm   (tier: translation)

Scope (Luminary only), into src/lm/dap/ with include/agc/lm/dap.h:
DAPIDLER_PROGRAM.agc (DAP startup, the 100 ms T5RUPT cycle, mode
selection), P-AXIS_RCS_AUTOPILOT.agc, Q_R-AXIS_RCS_AUTOPILOT.agc (the
P/Q/R axis phase-plane logic, jet firing times), TJET_LAW.agc (the
jet-firing-time law), KALMAN_FILTER.agc (rate estimator), TRIM_GIMBAL_CONTROL_SYSTEM.agc
(DPS gimbal trim), AOSTASK_AND_AOSJOB.agc (acceleration-of-stick
offset estimation), DAP_INTERFACE_SUBROUTINES.agc, T6-RUPT_PROGRAMS.agc
(the jet on/off timing interrupt), RCS_FAILURE_MONITOR.agc,
SPS_BACK-UP_RCS_CONTROL.agc (check: may be mis-named in the repo; translate
whatever is there). Jet commands go to channels 005/006 via agc_channels.h;
timing via the waitlist/T6 model in agc_exec.h (if T6 needs a new primitive,
declare it locally and list it under requested core additions).
Tests: tests/lm/dap.c (compiles): TJET law for zero error/rate is no
firing; phase-plane for a large positive error commands the correct jet
polarity; Kalman estimator converges on a constant rate within N cycles.
