# review-io-dsky -- review of io-pinball + io-nouns

## Scope
src/io/dsky/display_interface.c, src/io/dsky/pinball/*.c, src/io/dsky/nouns/*.c,
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
- DONE (session 3) make lib: lane compiles; the one leftover
  agc_dsky_endext caller (src/csm/prelaunch/align.c:544) -> endmark.
  Remaining lib breakage is review-imu's mid-rewrite of prelaunch_align.c /
  prelaunch_gyrocompass.c / exec/dispatch.c (missing AGC_JOB_* ids and
  imu.h prototypes), not this lane. Tests via SCRATCH/review-io-dsky/tb.sh.
- DONE rule-5 density >= 2.0 in every lane file (check_rules 0 warnings):
  real pre/postconditions -- lamp/flag bits read back after WOR/WAND and
  FLAGWRD4 up/down, ones' complement round trip, NOUT bounds, DSPCOUNT
  blocked at VD1 after DSPLV/BLANKDSP, DSPTAB word positive after DSPOUT,
  sign relay lit after SGNON, CADRSTOR/DSPLIST full after ENDIDLE/NVSUBUSY,
  RELINT done before PINBRNCH, EXTVBACT 0 after CLEARMRK, null/range
  preconditions. Fixes found on the way: pin_own_id returned 0 (a valid
  index) on a failed precondition, now -1; pin_dspdcput read the scale of
  k before checking k; pin_recaltst checked `which` after using it;
  blanksub/init tautology asserts replaced. Lane src 0 style findings,
  uncrustify clean, tests 200/3401/1627 pass, no dsky assertion report in
  any test binary.
- DONE test style: tests/io/nouns.c, tests/oracle/dsky.c, tests/io/dsky.c
  0 findings (file prologs, function prologs, main first, decl comments);
  tests/io/dsky.c's local agc_erase_read_ecadr declaration dropped (now in
  state.h). 200/3401/1627 pass.
- DONE fidelity inventory, findings table, open questions, V27, noun write
  gaps, requests, Lessons; MODULE_STATUS row; io-pinball.md deviation.
- DONE sim B1 (QUIKDSP tested DSKYFLAG): new agc_dsky_dspoutsb (CCS NOUT /
  DSPOUTSB, no flag test) in dsky.h/state.c; agc_dsky_dspout = DSKYFLAG
  test + it; t4rupt.c t4_quikdsp calls it (DEVIATION comment removed);
  fresh_start_{csm,lm} xexpect B1 -> expect; sim.md B1 marked fixed;
  unit test test_quikdsp_without_dskyflag (dsky 220/0).
- DONE all 7 sim scenarios pass on a debug sim (no dsky assertion
  reports). The p52 run caught my too-strict pin_ones_neg precondition
  (MARKPLAY's NVWORD reset CSes a negative word): fixed, see Findings 17.
- No file mid-edit. Lane complete.

## Method
* Oracle: tests/oracle/dsky.c runs 92 internal NVSUBs per vehicle through
  yaAGC (tools/oracle/cases/dsky*.spec) and compares the eleven DSPTAB
  words, NOUT, VERBREG and NOUNREG bit-exact: 1627 checks, 0 KNOWN. This
  covers every SFOUTABR output routine, DSPDECWD/DSPDC2NR/DSP2DEC,
  HMSOUT, M/SOUT, 2INTOUT, OPDEGOUT, the machine-address nouns and the
  verb paste, so the rounding and order of operations of the display
  conversion are checked against the assembled ropes, not the text.
* Label inventory (script in SCRATCH/review-io-dsky): 619 labels in the
  CM + LM PINBALL_GAME_BUTTONS_AND_LIGHTS, DISPLAY_INTERFACE_ROUTINES and
  KEYRUPT_UPRUPT files; 462 are named in the C. The other 157 are branch
  points inside routines the C has (e.g. IMMEDRET, SKIPADD, WAKEPLAY,
  PRIOPLAY, DSPOCTIN, the END* bank-switch returns), constants (HI5, MID5,
  33DEC, 34DEC, the SECON/MINCON/HRCON tables, folded into the scale
  code) and bank layout (PINSUPBT, ENDPINS1/2, *BBNK). Of those only
  LEFT5, 33DEC/34DEC, PINSUPBT and ENDPINS1 are used by other AGC files;
  LEFT5 is a 5-bit cycle (`<< 5` in C), the rest are not code.
* Noun data ids: every component of every noun 00-99, both vehicles, read
  and (if loadable) written through agc_dsky_noun_* / agc_noun_* (gaps.c in
  the scratch dir); results under "Noun write-path gaps".
* Rule 5 / style: tools/check_rules.py and tools/check_style.py on the 17
  lane source files and 3 lane tests, uncrustify --check.

## Findings
| # | Where | Finding | Resolution |
|---|---|---|---|
| 1 | pinball/scale.c OPDEGOUT (oracle F15) | CDUT = -20BIAS: sum -0 was treated as CCSHOLE (POODOO); the AGC takes the -0 branch and shows +89.997 deg | fixed (session 1); 7 KNOWN flipped to hard |
| 2 | scale.c DSPSIGN/PRSHRTMP | machine-address nouns 01-03 lost -0 (shown +) | fixed (session 1) |
| 3 | nouns/data.c, scale.c | negative TIG/TTOGO (HMS, M/S) could not be written (BAD_ARG) | fixed (session 1); all 64 KNOWN dsky oracle mismatches now hard checks |
| 4 | tests/io/dsky.c | 4 unit failures | fixed (session 1); 200/0 |
| 5 | display_if.c CLEANDSP/KLEENEX | without a continuation the caller's job slept forever in C | DEVIATION: own MAKEPLAY job (io-pinball.md); test_cleandsp_no_continuation |
| 6 | scale.c pin_dspdcput | one routine for the whole SFOUTABR fan, > 60 lines | split: SFOUTABR fan, pin_2intout, pin_sfout_dspdcend |
| 7 | 7 forwarding wrappers | agc_dsky_falton/loadlv/releasedsp/blanksub/endidle/nvsubusy/endext added nothing | wrappers are now the implementations; endext removed, 13 callers -> agc_dsky_endmark (ENDEXT EQUALS ENDMARK), last one src/csm/prelaunch/align.c this session |
| 8 | pinball/internal.h | "/*" inside a comment (nested comment) | fixed (session 2) |
| 9 | scale.c pin_own_id | returned 0 (a valid DSPTEM index) on a failed precondition, kind unset | returns -1 (callers test < 0) |
| 10 | scale.c pin_dspdcput | read the scale of component k before checking k | check first |
| 11 | nvsub.c pin_recaltst | asserted `which` in range after using it as an index | check before use, RELINT on the error path |
| 12 | nvsub.c blanksub, state.c init | tautological assertions (nvtemp <= 7 after & 7; nout == 11 just stored) | replaced by pin_invariants() postconditions |
| 13 | rule 5 | density 0.83-1.92 in 7 files | >= 2.0 everywhere with real checks (Progress) |
| 14 | tests/io/dsky.c | own prototype of agc_erase_read_ecadr (now in state.h) | removed |
| 15 | noun data layer | loadable noun components with no write path | listed below; request to owners |
| 16 | V27 DSPFMEM | stub | justified below |
| 17 | state.c pin_ones_neg | new precondition `mag <= POSMAX` was wrong: dif_first_vn / MARKPLAY CS a negative NVWORD back to positive; P52 then hit DSPALARM 01501 | precondition is any 15-bit word; postcondition: CS flips bit 15 |
| 18 | T4RUPT QUIKDSP (sim B1) | called agc_dsky_dspout, which tests DSKYFLAG; AGC QUIKDSP is CCS NOUT / TC DSPOUTSB | agc_dsky_dspoutsb; scenarios flipped to expect |

## Open questions (lane notes) -- answers
* io-pinball "DSPOUT lives here": yes; src/io/service/t4rupt.c calls
  agc_dsky_dspout() (DSPOUT and the T4RUPT DSPOUTSB path), no copy.
* io-pinball "V37 UNIMPLEMENTED": closed; src/exec/change_program/sequence.c
  implements agc_change_program, which MMCHANG calls.
* io-pinball "unwired data ids": reads are complete except N08 ALMCADR
  +0..+2 (the AGC alarm return address; no owner in C, NOT_FOUND by
  design, docs/history/waves/integrate.md). SPECIFY_ADDR (N01-03),
  SPECIFY_CHANNEL (N10) and INCREMENT_ADDR (N15) are NOT_FOUND in the data
  layer because pinball handles them itself (MCTBS, CHANDSP, V15).
  Writes: see below.
* io-nouns "raw 15-bit word for OCTAL/CDU": confirmed; pinball takes
  is_raw words as the 15-bit word (pin_value_of_word) and the oracle CDU
  and octal runs are bit-exact.
* io-nouns "HMS rule (only data_id[0])": correct; HMSOUT shows one DP time
  in R1-R3 (noun 65 lists SAMPTIME three times only because the table
  has three slots).
* io-nouns "2DEC* SF constants": the rope values are used; LM LR_VELX etc.
  read back negative (SFIN 77766 60557) in tests/io/nouns.c.

## V27 (DSPFMEM) stub -- justification
V27 shows a word of fixed memory addressed by an FCADR plus superbank bits
(noun 26 / 02). The C translation has no fixed-memory image: constants
are compiled C values and code is C functions, so there is no word at an
FCADR to show. The only rope image in the project is the test oracle's
(tools/oracle), which the flight library must not depend on. pin_dspfmem
checks its preconditions and returns AGC_STATUS_UNIMPLEMENTED; listed in
io-pinball.md "Stubs". Not used by any flight procedure (ground checkout
verb).

## Noun write-path gaps
Loadable (no NNTYPTAB no-load bit) components whose read works but whose
write returns NOT_FOUND from agc_dispatch_noun (src/exec/nouns.c, not this
lane): the values are computed on read (noun_derived) or owned by a
module that exposes a reader only.
* CSM: N17/N96 CPHIX..+2, N38 TET, N49 DELTAR/DELTAV/source code, N51
  RHOSB/GAMMASB, N55 NN1, N62 HDOT/ALTI (P11 readout), N87/N94 MRKBUF1+3/+5,
  N95 PRAXIS..+2.
* LM: N18 FDAIX..Z, N38 TET, N49 DELTAR/DELTAV/WHCHREAD, N51 ALPHASB/BETASB,
  N55 NN, N67/N78 RSTACK words, N80 DATAGOOD/OMEGAD.
In the AGC a V21/V25 on these stores into the erasable; in C the load
stops with NOT_FOUND. All are displays the programs overwrite (FDAI
angles, mark buffers, P11 readout, TET) except N55 NN/NN1 (P34/P35
iteration count, which the crew may load) -- see Requests.

## Requests (outside this lane)
* review-imu (src/csm/prelaunch/align.c, prelaunch_gyrocompass.c,
  src/exec/dispatch.c): `make lib` fails on the unfinished prelaunch
  rewrite (AGC_JOB_PRELAUNCH_*, GYROCOMPASSING_*, IMU_TEST_* ids and the
  imu.h prototypes in review-imu.md items 29-44). The one dsky-related
  error there (agc_dsky_endext) is fixed.
* prog-align / burns lanes: `agc_chan_and(chan, (uint16_t) ~BIT)` passes a
  16-bit mask, so AGC_ASSERT(mask <= AGC_CHAN_MASK) fails and the WAND is
  skipped (bit never cleared): src/csm/optics/sxtmark.c:218 (seen in the
  P52 scenario), src/csm/burns/servicer.c:905,
  src/csm/burns/sps/clock.c:522,655, src/csm/burns/earth_orbit_insertion.c
  :808. Mask with 077777u as the LM files do.
* rendezvous / src/exec/nouns.c owner: write paths for N55 NN (LM) and NN1
  (CSM) so V25N55 works; optionally a plain store for the other ids above.

## Lessons
* check_rules counts assertion *lines*: `if (!AGC_ASSERT(a) ||
  !AGC_ASSERT(b))` on one line counts once. Put each assertion on its own
  line.
* Cheap real postconditions exist for most void helpers: read back the
  channel or flag word just written (WOR/WAND, FLAGWRD4), the DSPCOUNT a
  routine promises to leave, the ones' complement round trip.
* Adding postconditions finds bugs in the preconditions next to them
  (pin_own_id's bad-path return, pin_dspdcput's order, recaltst's index).
* When removing a public function, grep the whole tree again at the end:
  a lane mid-rewrite (prelaunch_align.c) brought a removed call back.
* The oracle was what found every display bug in this lane (-0 cases);
  the unit tests passed on all of them.
* Run the sim scenarios on a debug build (AGC_HOST_DEBUG) after adding
  preconditions: a helper's name ("ones_neg of a magnitude") is not its
  contract; a P52 scenario was the only test reaching the caller that
  used it as a general CS.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [io-pinball](io-pinball.md#program-flow-and-decisions), [io-nouns](io-nouns.md#program-flow-and-decisions).
