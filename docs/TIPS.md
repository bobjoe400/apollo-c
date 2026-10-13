# TIPS.md -- tips and tricks for translating AGC assembly into C

Consolidated 2026-10-12 (documentation wave, PLAN.md item 9e) from the
"Lessons" section of every note in docs/modules/, deduplicated and
organised by topic. Each tip keeps the first-hand example that taught it;
the lane in parentheses is where to read the full story
(docs/modules/<lane>.md).

Contents: 1 Numbers are words, not values -- 2 Basic-language idioms --
3 Interpretive idioms -- 4 Jobs, displays and scheduling -- 5 Erasables
and shared state -- 6 Reading the AGC text -- 7 Status, assertions and
failure handling -- 8 The yaAGC oracle and testing -- 9 Tools: formatting,
rewriting, analysis -- 10 Closed loops: GUI, LM Simulator, NASSP --
11 Running many agents.

## 1. Numbers are words, not values

* **The AGC holds two or three words; an `agc_dp_t` holds a value.** The
  interpreter forces sign agreement only in a few places (DDV's MPAC+ when
  the majors are within one, BUFPOS/BUFNEG, TPAGREE in NORM and BUFZERO,
  VECAGREE in V/SC). DAS, DAD, DSU, BDSU, DMPSUB, GENDDV, SQRTSUB, SN1 and
  POLY work on the words as they are, so their results can be pairs whose
  signs disagree, and the next DMP, DDV, SR, SQRT or ASIN reads those
  words. A value model is exact only where an agreement point precedes
  the use (review-core-numeric). A residual of a few LSB with no logic
  difference is nearly always such a pair (follow-up-done-lanes,
  review-imu, review-nav-integ, review-prog-rendz).
* **Fix word residuals by moving the whole chain to words, at once.** The
  conic lane went from 64 residual cases to 0 in one mechanical step
  (agc_tp_t -> agc_tp_words_t, every op -> agc_mpac_*), faster than
  patching sites (review-nav-conic). Converting a chain that never touches
  such words changes nothing (UPCNTRL3 was reverted).
* **Keep words where they are stored, not only inside a routine.** TFFALFA
  (TFFCONIC -> CALCTFF), TBUP/TGO (ATMAG -> MAINENG -> T2TEST) and
  PCONS/YCONS (CONST -> next pass) lose exactly the needed bits when a
  local conversion canonicalises at the store. Half a conversion can be
  worse than none: DREF as words with D still a value put L/D 256 off
  (follow-up-done-lanes). Measure which stored word matters before
  converting: the guess "D's split" was wrong; toggling each input
  between words and canonical form found V1 in three runs (loose-ends).
* **MPAC+2 is live state.** DLOAD, DDV and DMPR zero it; DOT, DSQ, DMP,
  SQRT and SR write it; DAD/DSU/BDSU/DCS keep it; ROUND, SL, SR, NORM,
  SQRTSUB and TAD read it. "DMP SL1", "DOT SL1", "DMP DAD SQRT" and
  "POLY ... SL n" shift or root the TP, not the stored DP; SQRT leaves its
  own unnormalising bits there, so SQRT SL1 is not shl(sqrt)
  (review-core-numeric, review-guid-landing, review-guid-entry). DDV
  ignores MPAC+2 but SRR/SLR/ROUND use it: check what the NEXT
  instruction reads (review-nav-integ).
* **DMP truncates, DMPR rounds; most guidance never rounds.** grep the
  listing for DMPR before writing a rounded multiply: one rounding per
  multiply became hundreds of LSB over an entry trajectory (DTEAROT one cs
  -> RT.y 98 LSB) and was first blamed on the emulator (review-guid-entry;
  also review-imu, review-nav-aux, review-guid-powered).
* **Add and shift wrap; only divides saturate.** DAD, DSU, BDSU, TAD, SL,
  SLR, VAD, VSU, VSL wrap modulo the word and set OVFIND; only DDV/BDDV,
  V/SC and UNIT give DPPOSMAX (core-numeric, 20k yaAGC runs). TS, XCH,
  DXCH and ADS store the overflow-corrected word (16383 + 1 -> +0), and
  yaAGC's MP and DV take the corrected A, so "AD then MP" wraps. Saturate
  only where the code says so (OVERSUB, LIMITS, CA POSMAX)
  (review-dap-lm, review-guid-landing). DAS/DDOUBL filter states near 1.0
  wrap too: test a limiter through a stage with headroom (dap-csm).
* **SR truncates; SRR, VSR and vector shifts round.** SL* with a negative
  count is a truncating SR (review-guid-entry). VSR on a stored vector
  rounds per component on the words (MPACSRND with MPAC+2 = 0)
  (review-guid-powered).
* **Ones' complement -0 is a value with meaning.** T6JOB relies on TIME6 =
  -0 (rupt at the next DINC) and +0 there is the CCS hole; MIDORLST's
  `SU TIME6` of equal counts gives -0, and a C list of plain integers gave
  +0 and POODOO 1103 in every RCS firing (closed-loop-fixes L3). A loop
  index counted down by "CS TWO; ADS MPAC" reaches -0 and a later CCS
  takes the fourth branch; 1 + (-1) = -0 does the same (LONGGYRO N = 1)
  (dap-lm, review-imu). Model the observable behaviour, write the
  DEVIATION, and give oracle tests the one-LSB tolerance only there.
* **SIGN is "negate if negative", not copysign** (CALCTFF's gone-past
  test). **BZF/BZMF after DCA test only the major word**: a C DP compare
  with zero is a different test (review-prog-align).
* **Stale registers are real behaviour.** V/SC and UNIT overflow exits use
  the DVSIGN of the previous divide (core-numeric, kept in agc_interp_t).
* **Scale factors never change inside the stored state vector** (RN/VN
  2^29 m / 2^7 m/cs; the 2^27 / 2^5 lunar scales belong to conic-local
  copies). Write the scale on every field. Two constants in one listing
  can still have different units: MINCSM was already a word while
  MINLMD/MINMINLM were decimal kg (review-sys-misc).
* **Read constants from the rope, not the source text.** `SSP TWEEKIT
  20000` is stored as 10000 and `GEOMSGN 37777` as 27777 (LM LAMBERT took
  a different branch); "B1" in a 2DEC* stores TWICE the number (the F7
  constants were read as B-1); BIT13 is 1/4, not 1 (QUARTER,
  review-dap-csm); 2OCT words with bit 15 in the low word are negative
  low words (AGSRND1, prog-align). A rope reader is 20 lines of Python
  (review-guid-powered).

## 2. Basic-language idioms

* **TS of an overflowed A skips the next instruction.** Look for "TS x;
  TCF +3; INDEX A; CS LIMITS" (ROLLSET) and the TS L skip in ACTLIM; in
  RESCALES the skipped CA changes which word the next rescale uses
  (dap-csm, review-guid-entry).
* **After DAS, A is the overflow indicator and L is +0**, so "DXCH TMPn"
  stores (overflow, 0): that is how the TVC filter clears a node (dap-csm).
  DAS never re-signs a DP: a later CA of the major word can read hi +-1 of
  the canonical split (TTF/8, F9) (review-guid-powered). Routines ending
  in DAS/DDOUBL (AX*SR*T, FLESHPOT) leave such words for the next pass:
  run them on agc_dp_words_t and collapse to a value at the API.
* **ADS leaves the sum in A as well as in memory**; a following TS stores
  the sum (dap-lm).
* **DV can divide an old remainder.** L survives CCS/XCH/AD, so a DV can
  include the remainder of an earlier MP/DV (COMMEQS divides with the last
  F(MASS) remainder in L); model DP pairs explicitly (dap-lm).
* **"Result in A, second result in L" routines** (ROT-TOUV: U in A, V in
  L) are easy to swap; nothing crashes, the jets just fire on the wrong
  axis (dap-lm).
* **INDEX I; 0 X with I = a CA or CS word is a run-time sign selector**:
  an int +1 / -1 (0 for "same jets") translates it (dap-csm).
* **Registers are reused as arguments.** TC WAITLIST takes the delay from
  A, and in R04 the A left by "TS RTSTMAX" (2 or 6) is the first RADSAMP
  delay (sys-misc). `CA Z / TS DISPDEX` stores the program counter, i.e.
  "some positive number": name it and say why any positive value works
  (guid-powered).
* **A table of TCs is not a fall-through.** "TC RCSMONIT" in the LM T4JUMP
  table exits through RCSMONEX = RESUME, so each slot is one activity
  (sys-misc). `INDEX WHICH / TCF n` tables become a struct of enums per
  program, with the AGC entry number in the field comment (guid-powered).
* **KILLTASK compares addresses**, so an alias (`ZOOM = P40ZOOMA`) only
  kills tasks scheduled under that label: read the `=` lines first
  (guid-powered). STCLOK3's "mod 1 s" is (dt mod 2^19) mod 100: keep the
  AGC arithmetic, not a plain `%` (guid-powered).
* **ALARM ends with RELINT**; agc_alarm() is balanced, so an INHINT the
  AGC repeats after ALARM must not be repeated in C (review-imu).

## 3. Interpretive idioms

* **The interpreter is a library, not an emulator.** Each op is one C call,
  the push-down list becomes locals, OVFIND / index registers / switches
  stay; bank bookkeeping (BANKCALL, SWCALL, POSTJUMP, 2CADR, BBCON)
  vanishes and INTER-BANK_COMMUNICATION.agc is a comment. RTB op codes are
  real routines (LOADTIME, SGNAGREE, CDULOGIC are called from flight code).
* **MXV/VXM are three plain DOTs, no shift**, so a half-unit matrix
  (REFSMMAT, XNB) halves the vector; SCALEVEC's VSL2/VSL8 only land on the
  right scale because of that factor (prog-align). DMPSUB multiplies MPAC
  by the operand (vector component x matrix element) (review-sys-misc).
* **Addresses on the push-down list.** PDDL then DDV with no address pops
  the pushed word as the divisor (DV/VC in P37T, not VC/DV); VLOAD with no
  address pops it, and on a V32 re-entry the cell is stale, so keep the
  value in module storage (p37). PDDL/PDVL with no operand exchange with
  the top; relative offsets (+11) are octal; NORM stores -shifts, and
  indexed shift counts are addr - C(X) (prog-rendz).
* **"MPAC on entry" matters.** GETX, LAMENTER, POLY read whatever MPAC
  holds (PARAM's SR* MPAC+2, ABVAL's TP words): read the caller's last
  instruction before the CALL (review-nav-conic). RP-TO-R / R-TO-RP pick
  the body by BHIZ on MPAC, which at most call sites is the time just
  pushed: the moon, whatever LUNAFLAG says (review-prog-align).
* **POLY coefficients**: the 2DEC "B-3/B-1/B1" annotations apply to the
  stored words; "DEC 1" before them means three coefficients (p37).

## 4. Jobs, displays and scheduling

* **Defer software restarts.** BAILOUT/POODOO jump into ENEMA at once;
  doing that inside a C job makes the call graph recursive. Record the
  request and serve it when the job, T3RUPT or interrupt returns (exec).
* **Split a job at every wait.** A job that sleeps for a DSKY answer, an
  IMU pulse train or INTSTALL becomes one function per resumption point,
  and the waiting primitive gets the continuation job id. Name pieces by
  what they do after waking (`_after_display`, `_resume`). INTSTALL: when
  the integration says BUSY, return OK and name the job to resume; keep
  iteration state in module erasables (p37). Split where the AGC re-reads
  the clock so a BUSY resume does not change MARKTIME (prog-rendz).
* **Display returns are different jobs.** GOXDSPF returns +1 V34, +2 V33,
  +3 ENTER; RRDESNBK's "TCF -4" re-flashes on V33 (sys-misc). GOMARKF's +3
  covers ENTER and V32: test the verb for BIT6 (prog-align). EXDSPRET
  continues at the terminate id, so a non-flashing display mid-job needs
  all three ids set (prog-align). An R-kind display (GOMARKFR) returns at
  once; a MAKECADR/DESRET return becomes a continuation job (sys-misc). A
  GODSPRET or flash display with a NULL continuation fails silently
  (status-handling).
* **A "paste" display is two NVSUB calls**: the first VN is the one in A;
  the flashing verb comes from 1STOR2ND. Passing the pasted verb first
  makes pinball run it, and V97/V99 are extended verbs with code of their
  own (fix-b6).
* **A status value is not a control-flow token.** Pinball took every
  ALARM from a load-resume routine for ALMCYCLE and ran ENTER again, so a
  refused V37 also lit OPR ERR. Mark the event at its source (ALMCYCLE
  ran) instead of overloading a status (closed-loop-fixes L1).
* **Restart code runs inside a core set in the AGC.** After a software
  restart, MMDSPLAY's DSPMMJOB lands in core set 0 and SPECTEST makes it
  current; GOTOPOOH's GOP00FIX then replaces it there, so PROG stays blank.
  The C ran restart code outside any job and showed PROG 00 until
  exec_core0_continue_as reproduced the takeover (closed-loop-fixes D1).
* **Shared end jobs keep the routine's own flags.** ENDEXT is ENDMARK in
  Luminary and does not clear NODOFLAG; only the routine's own DOWNFLAG
  does (prog-align). A restart reruns CANV37 / V37XEQ with whatever
  MINDEX holds, so their checks stay (review-exec-sched).
* **Interrupt timing needs the scaler, not the centisecond.** Taking
  interrupts once per cs lost the rest of a T6RUPT chain and ran T5 before
  T3; stepping the 3200 Hz scaler 32 counts per cs gives yaAGC's phases
  (review-sim). A busy-bound counter hit by an AGC polling loop (CHECKG1)
  measures CPU saturation, not a sizing error (loose-ends).

## 5. Erasables and shared state

* **One AGC erasable must be one C store.** OGC/IGC/MGC lived in the noun
  table and in the gyro driver; the keyboard load wrote one and V42
  torqued the other, so fine align did nothing in the LM Simulator
  (closed-loop-fixes L2). Check EQUALS / `=` aliases of every shared
  erasable against the C storage: the DELVSLV/DELVLVC split compiled,
  passed every test and zeroed P30's input (review-state-flags).
* **Erasable by address needs a map, element by element.** Pinball's
  machine-address nouns, V71/V72 uplink and hosts that restore scenarios
  (NASSP) address erasable by ECADR; typed C storage needs an explicit
  address map (src/core/ecadr.c). Map whole blocks: restoring only RN/VN
  left RCV/DELTAV at 0 and P00 integration exited at once; the 42-word
  state vector blocks, LAT/LONG/ALT and LAT(SPL)/LNG(SPL) (HA/HP were 5
  nmi high without them) all had to be in it (WORKLOG, NASSP milestone 1).
* **Mixed-sign words survive a load.** V71 stores a DP or TP one word at a
  time; keep the raw words as loaded and rebuild the typed value from all
  of them; refresh the raw words from the typed value only for elements
  whose value changed, or the AGC's sign-disagreeing words are lost
  (ecadr.c refresh, WORKLOG milestone 1). Ground-loaded test erasables
  (P07) likewise are reachable only at their ECADR (review-sys-misc).
* **Lane "reset" functions are power-on helpers, not DOFSTART**: the AGC
  never clears program erasable on a fresh start (integrate). C module
  state outlives agc_state_init, so tests zero it to match the AGC's zeroed
  erasable (oracle-cases); a second agc_power_on in one process sees
  display-interface state survive (review-sim).
* **Before adding a "missing" routine, grep for a second copy**: several
  lanes had translated CSMCONIC/LEMCONIC, SETMINDB, CDUTCMD storage,
  RADMODES; keep the owner's (integrate).

## 6. Reading the AGC text

* **Diff Comanche against Luminary routine by routine.** Most code is
  identical; LAMBERT, LSPOS, SERVICER, the flagwords, LASTBIAS (PIPUSE vs
  PIPUSE1), GET+MGA (UNIT twice on the LM) and WOTEST (Luminary writes the
  channel 13 word-order bit only when it is 0; 50 extra packets a second
  otherwise) are not (review-imu, loose-ends, closed-loop-fixes).
* **The sorted flag tables have typos and names never assembled**; use the
  per-flag definitions and the assembled symbol table (review-state-flags).
  Comanche's erasable file has no per-bit channel comments: derive CM
  channel bits from the code that uses them.
* **Keep the page number and label in every prolog, and check them.** Page
  numbers drift (the CM P40-P47 file was 4-9 pages ahead; MKVB51 is
  p. 232, not 227); an index built from the listings' "# Page" markers
  gives exact cites (review-guid-powered, review-prog-align).
* **Read the label's comment before naming anything.** V97E/P/T are crew
  responses, not engine states; derive flag names from the comment for the
  set state (naming, docs/NAMING.md). VN pseudo-op digits are decimal:
  `VN 0674` is noun 74 (guid-powered).
* **AGC text breaks C comments.** Vector names end in `/` and multiply is
  `*` (R(T1)/, UR1/*X), so "UR1/*X" opens a nested comment and "X*/" closes
  one; `SL*/SR*` too. Write "UR1/ x X", "SL* and SR*" (p37, review-nav-aux).
* **Some code cannot be translated**: LM P07 and CSM P07 jump into
  erasable-memory programs not in the rope; document them as stubs
  (sys-misc).

## 7. Status, assertions and failure handling

* **Recovery is not chosen by return type.** A failed precondition either
  cannot happen (delete the guard) or can (return a status the caller acts
  on). "void returns, value functions return 0" produced 2,400 silent
  failures in one sweep. `(void) AGC_ASSERT(x)` is a no-op, and 3,233
  slipped through a case-sensitive check.
* **A `(void)` cast hides two different things**: a status nobody can act
  on (make the callee void) and a failure the AGC acts on (WAITLIST full ->
  1203, FINDVAC -> 1201/1202, CCSHOLE -> 1103: return it). Lint the
  pattern `(void) identifier(`, not the `agc_` prefix: a lane-local helper
  reintroduced 31 casts (status-handling).
* **Do not filter statuses.** `if (!AGC_ASSERT(st == OK || st == BAILOUT))
  return BAD_ARG;` turns every other real status into BAD_ARG
  (review-exec-sched); tolerating "anything but BAD_ARG" ran on after a
  BAILOUT the AGC never returns from (status-handling-2).
  `st = (s != OK) ? s : st` keeps the LAST failure (review-dap-lm).
* **Never-fail functions should not return a status.** An always-OK status
  spreads `if (st != OK)` into every caller and hides the real failures
  next to it (review-prog-align); one that is OK only in its last switch
  case truncates a caller's loop (review-dap-csm). After a guard-deletion
  pass, stale "RETURNS" that reduce to "always OK" are the signal for a
  void conversion (prolog-consistency, always-ok: 136 functions).
* **Guards: what is guaranteed by construction.** Most guards the
  classifier could not prove were module invariants (a static only the
  module writes), postconditions of the previous statement, 15-bit words,
  constants, or AGC arithmetic facts; reading the writers of the static
  settles them faster than call-site analysis (status-handling-2). A guard
  that "checks the previous job did its work" can hide a missing step: the
  LM P51 never zeroed THETAD, and the guard made every P51 after a P52
  BAD_ARG (review-prog-align). A `!= 0` guard before DDV or the DAP's DV
  protects nothing; a C `/` does need one (status-handling-2).
* **Deleting guards safely.** Never delete a condition that mixes an
  assertion with normal flow (`!AGC_ASSERT(x) || n == 0`): split it. A
  condition can carry the side effect (`!AGC_ASSERT(decode(code, &c))`).
  A trailing-return deletion can leave an empty guard; grep for `{\n}`
  (status-handling-2, verify-collapse). A re-check after intervening
  statements is a postcondition, not a duplicate.
* **Rule 5 is a diagnostic, not a quota.** "Two assertions per function"
  produced no-op casts, redundant guards and silent-return dodges. State
  where assertions belong (boundary inputs, loop bounds, scale ranges,
  overflow consistency, each with recovery) and report the density; the
  no-silent-failure rule wins over density (review-dap-csm, review-sys-misc).
  Test assertions should state facts the AGC guarantees, not restate the
  store just made (review-guid-entry); cheap real postconditions exist for
  most void helpers (read back the channel or flag word just written),
  and adding them finds bugs in the preconditions next to them
  (review-io-dsky).
* **An early return inside INHINT must RELINT**, and can expose
  -Wmaybe-uninitialized at -O2: check with the Makefile flags
  (status-handling). AGC_ASSERT's debug and release forms must have the
  same value or flow analysis differs between the libraries (integrate).

## 8. The yaAGC oracle and testing

* **Assemble virtualagc's own Comanche055/Luminary099**; the Apollo-11
  GitHub sources do not assemble with current yaYUL. Verify against the
  official .binsource (0 mismatches). Keep the ~12 MB symbol tables out of
  git.
* **Run an interpretive routine without a caller**: poke op, EXIT, TC
  ENDOFJOB into an unused VAC area and point Q at it (INTPRET fetches
  through INDEX LOC, so erasable works as program memory). Return traps
  must match the bank half (FBANK < 20 -> QPRET to ABTCONIC; >= 20 ->
  V2TERROR / P39P79); a CALL from an erasable snippet cannot return, enter
  the routine with Q = the trap. Runs reaching NOVAC need executive core
  sets set up (NEWJOB 0, PRIORITY 37401, ...) (oracle-cases). An op with
  its address on the push-down list must be followed by an op line, or
  yaYUL takes the next word as its address and the run hangs
  (review-core-numeric).
* **Probe tricks**: interpretive labels never appear in Z, so stop on the
  next basic-language call and enter at INTPRET with Q = the label
  (review-guid-landing); "stop <basic label>" inside a routine dumps the
  push list mid-routine (review-nav-aux); `trace 1` finds the word split
  behind a few-LSB difference in minutes (review-imu); erasable names
  starting with a digit (1/DV1) need the E-bank address. Run a routine
  alone on thousands of random inputs instead of bisecting a 3-hour
  integration (review-nav-integ).
* **Split a residual between translation and representation** by rerunning
  the rope from the middle with canonical words: if the rope then equals
  the C, the C is right (review-guid-landing). Compare only words the path
  leaves (erasable overlays: LEQ = FX+1, GAMMAL = PREDANG); use two-stage
  generation when one routine consumes another's push list; raw inputs
  keep -0 (oracle-cases).
* **yaAGC is a reference, not scripture.** Its DV reads a -0 minor word as
  a borrow (mask with 077777, or use the DV-patched probe); its optics drive
  reads 035/036 where the program writes 053/054; its CDU drive empties
  CDUxCMD where the hardware counts down (review-sim, review-nav-integ).
  Follow the hardware where yaAGC is evidently wrong and write the
  DEVIATION.
* **Coverage and sensitivity.** gcov over the oracle test shows untested
  paths (near-360 GETX, every LAMBERT retreat) (review-nav-conic); a
  mutation check (force DDV agreement: 36 failures) proves the cases see
  the effects (review-core-numeric). Word-level erasables need cases with
  sign-disagreeing inputs (review-dap-csm). The oracle found every display
  bug in io-dsky; the unit tests passed on all of them (-0 cases).
* **Other lanes' tests move when a dependency becomes exact**: check them
  against the oracle before loosening (kep_leo_10min showed the AGC's own
  53.4 m) (review-nav-conic). A circular orbit is a poor apsides test
  (ECC = SQRT of a 1-2 LSB difference) (review-prog-rendz).
* **Link tests against the real library.** A test-local stand-in that
  shadows a library symbol hides link-order bugs; `ar r` replaces members
  by base name, so rebuild archives from scratch (integrate). AGC_ASSERTs
  inside libagc.a never reach a test's report hook (no AGC_HOST_DEBUG):
  tests count only their own (sys-misc). Run the sim scenarios on a debug
  build after adding preconditions (review-io-dsky).

## 9. Tools: formatting, rewriting, analysis

* **Rename and rewrite with Coccinelle, inventory with ctags or libclang**:
  sed cannot tell a function from a comment, and comments are where the AGC
  labels live. Coccinelle's pretty-printer is not layout-preserving: use it
  as the oracle of which tokens change and keep the layout by token-stream
  comparison. Check argument semantics on a sample (`--dir` is not
  repeatable in spatch 1.1.1; a bare `- OLD` rule does not parse) (naming).
  A `... return; }` rule is path-based and deletes early returns too:
  delete only the statement-level last return (always-ok).
* **libclang**: lexing needs no headers (`-nostdinc`,
  PARSE_SKIP_FUNCTION_BODIES; comments come back as tokens, 4 ms a file)
  (format2); macro arguments lose their tokens, so parse with AGC_ASSERT
  declared as a function, and read macro literals at the expansion
  location (collapse).
* **Call graphs**: build at -O0 -fno-inline (-fcallgraph-info at -O2
  attributes inlined calls to the caller); cflow 1.7 drops calls after a
  `for` with nested calls (collapse).
* **Uncrustify is the only formatter** (clang-format cannot insert blank
  lines for statement paragraphing). It parses .h as C++ unless `file_ext C
  .c .h`; `cmt_multi_check_last` strips ` * ` indents; it never joins lines
  and cannot scope alignment to a paragraph, so a token-level post-pass does
  that and `format-check` must run the whole pipeline; alignment is not
  idempotent in one pass, format twice (style2, format2). Run `make format`
  before `make check` (integrate).
* **Comments go above the statement, never inside it**; never align
  continuation lines to the open paren in chains of nested calls; split
  statements with more than two continuation lines into named temporaries.
  Style debt at this size is a writing task: per-file spec files applied by
  a script that knows the checker's parser (review-guid-entry,
  review-prog-rendz, review-dap-lm).
* **Regex style checkers misparse identifiers**: `longtime`, `integ_case_t`
  (int + eg_case_t), `g_tig` (`\w+_t`); rename or anchor with word
  boundaries (review-exec-sched, review-nav-integ, review-state-flags).
  check_rules counts assertion lines: one assertion per line
  (review-io-dsky).
* **Mechanical passes compound.** Guard conversion then inlining produced
  functions with the same guard twice; inlining left prolog boxes inside
  callers (grep indented `/****`); collapsed files kept vacuous `st`
  plumbing; a removed call came back from a lane mid-rewrite. After any
  automated rewrite, run structural checks (duplicate guard, forwarding
  wrapper, `{\n}`) and grep the whole tree again (collapse,
  review-exec-sched, review-prog-rendz, review-io-dsky). A here-doc driven
  replace must strip the here-doc's final newline (finish).
* **Static analysis must be observable and non-blocking.** Run Frama-C EVA
  per translation unit as a detached job with a progress log, cap
  concurrency by RAM, and fold in proven verdicts at the end. EVA's
  "invalid" is relative to its precision (invariants between two statics);
  EVA-only deletions rest on the test mains' inputs, so re-derive them from
  the call sites (status-handling-2, verify-collapse).

## 10. Closed loops: VirtualAGC GUI, LM Simulator, NASSP

* **The launcher has no AGC-binary setting**: it runs `../bin/yaAGC`, so
  the drop-in is a link there; rope -> vehicle, --port, --cfg, --resume
  all come from that command line, and the --resume slot can carry our
  pad file. Build the launcher with `-DYADSKY2 -DYADEDA2`; preselect a
  mission by writing Resources/VirtualAGC.cfg (Apollo 11 CM = 16, LM = 21)
  (sim). XTest clicks on wx/GTK buttons need a pointer motion 0.2 s before
  the press; stop peripherals with `pkill -x NAME`, since `pkill -f`
  kills the shell that runs it (sim).
* **Drop-in includes when packets are sent.** The LM Simulator applies the
  jets of the previous channel 5/6 packet; invisible at yaAGC's 20 writes a
  second, fatal with change-only output. Send one packet per WRITE / WOR /
  WAND, and count packets per second per channel against yaAGC before
  comparing physics (lm-simulator, closed-loop-fixes S-L1, WOTEST). Pace
  CDU counts as yaAGC does (FIFO at 400 / 6400 counts a second), or the
  coarse-align alarm 00211 differs (S-L2).
* **Compare against the reference in the same harness, including the
  reference against itself.** Several oddities (1204 POODOO, V50N48 never
  coming) were yaAGC + simulator behaviour; after the first RHC detent two
  yaAGC runs differed by more than the tutorial's tolerances, so those
  checkpoints compare displays, jets and a hold check, with tolerances
  from yaAGC's own spread (lm-simulator, closed-loop-fixes).
* **Drive a Tcl/Tk peripheral through a wrapper** that sources it unchanged
  and opens a control socket (Tk `send` is refused without xauth); inert Tk
  stand-ins under tclsh make it headless. gdb on agc_sim through a --sim
  wrapper catches flight-code alarms live (`break agc_ccshole` gave L3's
  chain at once) (lm-simulator).
* **Know the scenario's assumptions.** The LM Simulator tutorial was
  written for Luminary 131 (V60, R03's V50N48, V49 refused during R03, V37
  refused for 90 s after IMU operate) (lm-simulator).
* **NASSP restores scenarios through the erasable address map**: every
  quantity a mission scenario or MCC uplink writes must reach its typed C
  store (section 5); find gaps by replaying a scenario's AGC image through
  the shim on Linux next to NASSP's own yaAGC engine built offline
  (nassp-integration).
* **Step cost per frame is the integration's budget.** NASSP steps the AGC
  by warp x real dt each Orbiter frame; a full 2048-word mirror export
  every centisecond (~15 ms CPU per AGC second) made frames grow until
  Orbiter stopped responding at 50x. Counters per scaler count, import per
  cs, full export per second brought it to ~6 ms, like yaAGC. Do not run
  other CPU load on the host during a run: the stock AGC itself went to
  garbage when frames stretched (nassp-integration).
* **One Orbiter session at a time.** NASSP's PCM telemetry binds TCP 14242
  and MCC uplinks ride it: one session, kill, then the next.
  Orbiter_ng.exe only launches Modules\Server\Orbiter.exe and exits (wait
  on Orbiter.exe); `-s SCN -T SECONDS` runs a scenario unattended and saves
  (Current state).scn (nassp-integration).
* **Use the mission's own checklist workbook as the executor.** NASSP's
  "Apollo 11 Checklists.xls" compiled to Lua tables and executed
  identically on the stock and our builds (switch rows, DSKY rows, display
  waits, TIG-relative waits, engine cutoff on channel 11) makes the two
  runs comparable without a human. Lua key presses are set 1 then set 0
  (callbacks fire on change); uplink needs CMUplinkSwitch and UPTLMSwitch
  at ACCEPT plus the CAPCOM answer; NASSP's set_agcerasable is unusable
  for pokes (stores the address as the value) (nassp-integration).

## 11. Running many agents

* **Pin conventions before lanes run in parallel**: names for symbols that
  do not exist yet, comment placement, continuation indent, the void-cast
  rule from day one. A shared scratch directory needs per-agent
  subdirectories.
* **Parallel lanes grow adapter layers** (glue.c, ext.h, pending.h
  wrappers, second copies of THISPREC/MIDTOAV); budget a pass to replace
  every wrapper with a direct call to the owner and keep one copy. When
  folders move mid-run, re-grep the pending headers (two lanes had the same
  task id with different values) (guid-powered).
* **Checkpoint the module note per routine**; after a cutoff, list live
  agents before relaunching (a blind relaunch put two agents on the same
  files). A `--check` FAIL on a file you did not touch is usually a
  concurrent edit: rerun before investigating (format3).
* **Build review documents from the same data the apply step uses** and
  count rows in both (a section-key typo dropped 42 rows); compare
  before/after against a fixed commit, not a moving HEAD (naming).
* **Generate what can be generated** (job prototypes and ids, label ->
  page indexes, TODO lists from the checker's parser): hand-edited copies
  drift (prog-align, review-prog-rendz). Look labels up exactly
  (`tools/agc_label.py LABEL`), not by substring grep.
* **Numeric rules become quotas.** State the intent and report the number
  (assertion density, "no silent failure"); a quota produces dodges.
