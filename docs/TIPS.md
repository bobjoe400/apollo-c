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
* **Do not let `(void)` become the rule-7 escape hatch.** 777 casts had
  accumulated. Most were on operations that cannot fail (flag set, channel
  mask, phase change): those should return void. The rest hid real AGC
  failure paths (WAITLIST overflow -> 1203, FINDVAC -> 1201/1202) that the
  C must raise. Make the checker reject `(void)agc_` from day one.
* **MXV/VXM do not shift.** The interpreter's matrix ops multiply at face
  value, so a half-unit matrix (rows UNITX/Y/Z = 0.5) halves the vector.
  Code that follows with VSL2/VSL8 (SCALEVEC) only lands on the right scale
  because of that factor; keep it.
* **Parallel lanes grow adapter layers; budget a pass to dissolve them.**
  Each lane insulated itself from the others with `glue.c`, `ext.h`,
  `pending.h` wrappers (rz_lambert around agc_conic_lambert, a second copy
  of THISPREC/MIDTOAV). Fine while headers churn; wrong in the result.
  Integration must replace every wrapper with a direct call through the
  owning module's public header and keep one copy of each AGC entry.
* **An assertion with `(void)` in front is not an assertion.** Rule 5 wants
  an explicit recovery; `(void) AGC_ASSERT(x);` is a no-op. Write
  `if (!AGC_ASSERT(x)) { return <failure>; }` and make the checker's
  pattern case-insensitive from the start (3,233 of these slipped through).
* **clang-format cannot insert blank lines.** For statement paragraphing
  (NASA guide 5.5) use Uncrustify (nl_before_if, nl_func_var_def_blk,
  nl_before_return, nl_after_brace_close) and let it be the only formatter.
* **Mechanical passes compound into duplicates.** Converting `(void)`
  assertions to guards and then inlining helpers produced functions with
  the same guard twice. After any automated rewrite, run a structural
  duplicate-guard check (now in check_rules as a hard violation) and a
  forwarding-wrapper check, not just the compiler.
* **No forwarding wrappers.** A function that is only AGC_ASSERT guards plus
  one call passing its own parameters on (`chan_write` -> `agc_chan_write`)
  is deleted and its callers call the callee; a guard the callee does not
  already assert moves into the callee once.  `python3 -I tools/unwrap.py
  --list` finds them by body shape (docs/modules/collapse.md).
* **"Duplicate" assertions are only duplicates when adjacent.** A re-check
  of the same condition after intervening statements is a postcondition.
  A blunt dedup rule deleted four real checks before it was narrowed.
* **After a cutoff, list live agents before relaunching.** One reviewer
  survived a usage-limit kill; a blind relaunch put two agents on the same
  files. The surviving agent caught it; the lead should have.
* **DMP is not DMPR.** A guide that maps "multiply" to the rounded product
  silently changes results by hundreds of LSB over an entry trajectory.
  Audit every multiply against the listing before blaming the emulator for
  oracle mismatches.
* **Scale comments belong above the statement, never inside it.** Lanes
  annotated at the point of use (`th = /* theta/2 */ agc_dp_div(...)`),
  which no formatter can repair. State the placement rule in the brief and
  enforce it with a token-level script from the first commit.
* **Never align continuation lines to the open paren** when the code is
  chains of nested calls; use a fixed continuation indent, and split
  statements with more than two continuation lines into named temporaries.
* **Lint the pattern, not the prefix.** The void-cast check matched `agc_`
  names and `AGC_ASSERT`; the next lane-local helper (`csm_dap_words_das`)
  reintroduced 31 casts. Match `(void) identifier(` and allowlist the few
  stdlib calls instead.
* **Recovery is not chosen by return type.** A failed precondition either
  cannot happen (then delete the guard) or can (then the function returns a
  status and the caller acts). "void functions just return, value
  functions return 0" produced 2,400 silent failures in one sweep.
* **Rule 5 is a diagnostic, not a quota.** Telling agents "two assertions
  per function" produced void-cast no-ops, redundant guards and then
  silent-return dodges. State where assertions belong (boundary inputs,
  loop bounds, scale ranges, overflow consistency, each with recovery) and
  only report the density.
* **Static analysis must be observable and non-blocking.** Run Frama-C EVA per
  translation unit as a detached job that logs every run (progress.log +
  summary with ETA), cap concurrency by RAM, and keep working on the
  dataflow-classified cases meanwhile; fold in the proven verdicts at the end.
