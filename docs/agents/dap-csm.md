# Assignment: dap-csm   (tier: translation)

Scope (Comanche only), into src/csm/dap/ with include/agc/csm/dap.h:
RCS-CSM_DIGITAL_AUTOPILOT.agc (the CSM RCS DAP: rate/attitude error,
phase plane, the DAP data load R03), JET_SELECTION_LOGIC.agc (quad
selection tables, the jet-polarity logic with the failed-quad flags),
AUTOMATIC_MANEUVERS.agc (KALCMANU driver for the CSM), RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc,
TVCINITIALIZE.agc, TVCEXECUTIVE.agc, TVCMASSPROP.agc, TVCRESTARTS.agc,
TVCDAPS.agc (pitch/yaw thrust-vector-control filters), TVCSTROKETEST.agc,
TVCROLLDAP.agc, MYSUBS.agc. Keep the filter coefficients with octal and
engineering values and the sample periods. Jet commands on channels
005/006, gimbal commands on the OPTICS/gimbal CDU channels via
agc_channels.h; timing via agc_exec.h.
Tests: tests/test_csm_dap.c (compiles): jet selection for +pitch with all
quads good yields the documented jet pair; TVC filter step response sign.
