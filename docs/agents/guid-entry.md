# Assignment: guid-entry   (tier: translation)

Scope (Comanche only), into src/csm/ with include/agc/agc_csm_entry.h:
- ENTRY_LEXICON.agc (erasable and constant definitions for entry; produce
  the entry state struct and constants with scales) -> entry_lexicon.c/.h
- REENTRY_CONTROL.agc (P61-P67 entry guidance: initial roll, HUNTEST,
  UPCONTROL, KEP2 (ballistic), the final phase (PREDICT3), the
  drag/lift logic, the range prediction, the lateral logic) -> reentry.c
- P61-P67.agc (the program sequencing, displays, CM/SM separation
  checks, the entry DAP handoff) -> p61_p67.c
- CM_BODY_ATTITUDE.agc (body-attitude computation for entry) -> cm_body_attitude.c
- CM_ENTRY_DIGITAL_AUTOPILOT.agc (the entry DAP: roll command, the
  pitch/yaw rate damping, jet selection for the CM) -> entry_dap.c
- P37_P70.agc (return-to-earth targeting P37 and P70 saturn abort?
  -- read and describe) -> p37.c
- P11.agc (earth-orbit insertion monitor) -> p11.c
Use agc_channels.h, agc_exec.h, the agc_dsky.h display interface as in
docs/agents/guid-powered.md. Tests: tests/test_entry.c (compiles):
HUNTEST / range prediction for a nominal entry state produces a finite
range; entry DAP roll command sign for a positive roll error.
