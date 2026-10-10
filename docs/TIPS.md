# TIPS.md -- tips and tricks for translating AGC assembly into C

Living document. Lanes append to the "Lessons" section of their module note;
the lead folds them in here. Final consolidation is wave 6 (PLAN.md item 9).

## Numerics

* **Add/shift wrap, divide saturates.** DAD, DSU, BDSU, TAD, SL, SLR, VAD,
  VSU, VSL wrap modulo the word and set OVFIND. Only DDV/BDDV, V/SC and UNIT
  produce DPPOSMAX. Translating "overflow" as saturation everywhere breaks
  bit-exactness (core-numeric, verified on 20k yaAGC runs).
* **DMP vs DMPR.** DMP leaves a triple-precision product (truncated when
  read as DP); DMPR rounds. Pick the one the AGC used at that spot.
* **SIGN is "negate if negative", not copysign.** CALCTFF's gone-past test
  depends on it.
* **Negative zero exists in the AGC and not in C.** It changes: CCS branch
  selection, DDV quotients by ~2^-14 when a low word is -0, KEPLER on
  exactly circular orbits. Model the observable behaviour, comment it as a
  DEVIATION, and give oracle tests a one-LSB tolerance there.
* **Scale factors never change inside the state vector.** RN/VN are always
  2^29 m / 2^7 m/cs; the 2^27 / 2^5 lunar scales apply to conic-local
  copies. Write the scale on every struct field.
* **Use the assembled constants, not the source text.** The rope stores
  `SSP TWEEKIT 20000` as 10000 (0.25) and `GEOMSGN 37777` as 27777. The
  source-text value made LM LAMBERT take a different branch from the real
  AGC. When a constant matters, read it out of the .bin with the oracle.
* **Stale registers are real behaviour.** V/SC and UNIT overflow use the
  DVSIGN left by the previous divide. Decide explicitly whether to model it.

## Control flow and scheduling

* **Defer software restarts.** BAILOUT/POODOO/ABORT in the AGC jump into
  ENEMA immediately. Doing that from inside a job makes the C call graph
  recursive (rule 1). Record the request; run ENEMA at the end of the job,
  the T3RUPT, or the interrupt.
* **Split jobs at every wait.** An AGC job that sleeps for a DSKY answer or
  an IMU pulse train stores a return address. In C, split it into one
  function per resumption point and pass the continuation job id to the
  waiting primitive. Name the pieces by what they do after waking
  (`_after_display`, `_resume`), not `_1`/`_1b`.
* **Interpreter is a library, not an emulator.** Each interpretive op maps
  to one C call; the push-down list becomes locals. Keep OVFIND, the index
  registers when used as data, and the switches; drop everything else.
* **Bank bookkeeping vanishes.** BANKCALL/SWCALL/POSTJUMP/2CADR/BBCON carry
  no meaning in a flat address space. INTER-BANK_COMMUNICATION.agc
  translates to a comment.
* **RTB op codes are real routines.** They look like interpreter plumbing
  but LOADTIME, SGNAGREE, FORMDEG, CDULOGIC are called from flight code.

## Working from the AGC text

* **Diff Comanche vs Luminary before translating a shared file.** Most of
  CHIEFTAN is identical; LAMBERT, LSPOS (ephemerides), SERVICER and the
  flagwords are not. Translate once, gate the differences on the vehicle.
* **The sorted flag tables have typos.** Use the per-flag definitions, not
  the summary tables, and list the discrepancies.
* **Comanche's erasable file has no per-bit channel comments.** Derive CM
  channel bits from the code that uses them (T4RUPT, P11, P40s, the DAP).
* **Keep the page number and label in every function header.** It is the
  only reliable way back to the source during review.
* **Read the label's comment before naming anything.** V97E/V97P/V97T are
  not "E", "P", "T"; the AGC says what each variant is. Pasting a label
  fragment onto a prefix (`AGC_TASK_..._TTASK`) means the source was not
  read. See docs/NAMING.md.

## Oracle (Virtual AGC)

* **The Apollo-11 GitHub sources do not assemble with current yaYUL.** Use
  virtualagc's own Comanche055/Luminary099 and verify against the official
  .binsource (0 mismatches).
* **Run an interpretive routine without a caller** by poking a tiny
  snippet (op, EXIT, TC ENDOFJOB) into an unused VAC area and pointing Q at
  it: INTPRET does QXCH LOC and fetches through INDEX LOC, so erasable
  works as "program" memory. This is the flown EMP technique.
* **yaAGC has a divide bug.** SpToDecent reads a -0 minor word as a borrow
  when L is sign-extended; mask with 077777. Exclude affected cases or
  patch the vendored engine.
* **Big symtabs.** yaYUL symbol tables are ~12 MB each; keep them out of
  git and rebuild per tools/oracle/BUILD.md.

## Managing the work

* **Pin the naming convention for not-yet-existing symbols** before lanes
  run in parallel, or every lane invents its own flag/accessor names.
* **Each lane keeps a pending header** (`<lane>_pending.h`) for symbols it
  needs from other lanes, with the exact declaration, and lists them in
  its note. Integration removes the headers; nothing is lost.
* **Checkpoint the module note per routine.** Sessions get cut off;
  a Progress table lets a fresh agent resume without re-reading everything.
* **Rename with Coccinelle, inventory with ctags.** Sed cannot tell a
  function from a comment, and comments are where the AGC labels must stay.
* **A shared scratch directory needs per-agent subdirectories.** Two lanes
  overwrote each other's files before that rule existed.
* **Look labels up exactly.** yaYUL labels start in column 0;
  `tools/agc_label.py LABEL` finds the definition in both ropes with its
  comment block, where a substring grep finds every mention.
