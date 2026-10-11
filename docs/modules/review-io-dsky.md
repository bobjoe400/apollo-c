# review-io-dsky -- review of io-pinball + io-nouns

## Scope
src/io/dsky/display_if.c, src/io/dsky/pinball/*.c, src/io/dsky/nouns/*.c,
include/agc/io/dsky.h, include/agc/io/nouns.h, tests/io/dsky.c,
tests/io/nouns.c, tests/oracle/dsky.c. AGC: PINBALL_GAME_BUTTONS_AND_LIGHTS,
PINBALL_NOUN_TABLES, DISPLAY_INTERFACE_ROUTINES, KEYRUPT_UPRUPT (CM + LM).

## Progress
- DONE (session 1, checkpoint 4f35a43) F15 OPDEGOUT -0; machine-address -0
  through DSPSIGN/PRSHRTMP; negative TIG/TTOGO writes; all 64 KNOWN oracle
  mismatches fixed, tests/oracle/dsky.c bit-exact (1627 checks, 0 known);
  CLEANDSP/KLEENEX without continuation -> own MAKEPLAY job; pin_dspdcput
  split into SFOUTABR fan + pin_2intout + pin_sfout_dspdcend.
- DONE (session 2) src/io/dsky compiles; internal.h nested-comment ("/*"
  in prolog) FIXED. Tests (private HEAD-snapshot build, since other lanes'
  files at HEAD do not compile): io/dsky 200/0, io/nouns 3401/0,
  oracle/dsky 1627/0.
- DONE CLEANDSP-without-continuation unit test (test_cleandsp_no_continuation).
- DONE forwarding wrappers: agc_dsky_falton/loadlv/releasedsp/blanksub/
  endidle/nvsubusy now ARE the implementations (pin_* removed; endidle and
  nvsubusy lost their unused ctx param, no external callers);
  agc_dsky_endext removed, 12 callers in other lanes now call
  agc_dsky_endmark (ENDEXT EQUALS ENDMARK). Kept agc_job_markplay (job entry).
- DONE style: check_style 0 findings in the 17 lane files (function prologs,
  file prologs, decl/define/ternary/scale/closing-brace comments, nested
  ternaries rewritten as if/else); no line > 80; uncrustify clean.
- TODO rule-5 density >= 2.0 (display_if 1.77, display 1.12, keys 1.85,
  monitor 1.92, nvsub 1.41, scale 1.70, state 0.83 before this session).
- TODO note sections: findings table, V27 justification, noun write gaps,
  Lessons; MODULE_STATUS row; io-pinball.md deviation for CLEANDSP.
- No file mid-edit.
