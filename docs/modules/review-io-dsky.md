# review-io-dsky -- review of io-pinball + io-nouns

## Scope
src/io/dsky/display_if.c, src/io/dsky/pinball/*.c, src/io/dsky/nouns/*.c,
include/agc/io/dsky.h, include/agc/io/nouns.h, tests/io/dsky.c,
tests/io/nouns.c, tests/oracle/dsky.c. AGC: PINBALL_GAME_BUTTONS_AND_LIGHTS,
PINBALL_NOUN_TABLES, DISPLAY_INTERFACE_ROUTINES, KEYRUPT_UPRUPT (CM + LM).

## Progress
- DONE (session 1, in checkpoint 4f35a43) F15 OPDEGOUT -0; machine-address
  -0 through DSPSIGN/PRSHRTMP; negative TIG/TTOGO writes; all 64 KNOWN
  oracle mismatches fixed; tests/oracle/dsky.c bit-exact (1627 checks,
  0 known); CLEANDSP/KLEENEX without continuation -> own MAKEPLAY job;
  pin_dspdcput split into SFOUTABR fan + pin_2intout + pin_sfout_dspdcend.
- DONE (session 2) build check of src/io/dsky; tests/io/dsky 179/0,
  tests/io/nouns 3401/0 (private build: tree is mid-edit by other lanes,
  src/imu/mode.c at HEAD does not compile).
- TODO fidelity inventory, V27 justification, style debt, density, note.
- DONE (session 2) CLEANDSP-without-continuation unit test
  (test_cleandsp_no_continuation: OK, caller not asleep, one MAKEPLAY job).
- DONE (session 2) forwarding wrappers: agc_dsky_falton/loadlv/releasedsp/
  blanksub/endidle/nvsubusy now ARE the implementations (pin_* removed);
  agc_dsky_endext removed, 12 callers call agc_dsky_endmark (ENDEXT EQUALS
  ENDMARK). Kept agc_job_markplay (job entry with dispatch signature).
