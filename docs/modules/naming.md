# naming -- program/routine/verb/noun numbers out of identifiers

## Scope
Apply docs/NAMING.md to every identifier and file name in include/, src/,
tests/ and the docs/modules notes, replacing both the original numbered
names and the names left by the stopped mechanical rename (commit 23aa58d).
Map: docs/RENAMES.md (phase 1, reviewed). Raw inventory:
docs/RENAMES_inventory.txt.

## Method (phase 2)
1. `tools/gen_rename_cocci.py` reads the lane tables of docs/RENAMES.md
   (446 C identifiers) and writes ONE semantic patch, `tools/rename.cocci`:
   a Python table plus 21 rules, one per identifier kind (function
   definitions, prototypes, parameters, variables and arrays, struct fields
   and `.`/`->` access, enumerators in plain/typedef/anonymous enums,
   `#define` names, and every remaining expression use). Each rule builds
   the new name with a `fresh identifier`, so the rule count is fixed.
2. `spatch --sp-file tools/rename.cocci --in-place --include-headers --dir D`
   once per directory (spatch 1.1.1 keeps only the LAST `--dir`, so
   `--dir src --dir include --dir tests` silently processes tests/ only).
3. spatch re-pretty-prints every line it touches (breaks long initializer
   lines, drops `#define` column alignment, writes `case X :`). To keep the
   original layout, the same map was re-applied to the pre-rename text
   outside comments and string/char literals, and that text was accepted
   for a file only if its token stream (comments excluded) is identical to
   spatch's output: 86 of 87 files matched; guid_pending.h differed only by
   the intentional VHHDOT edit and was re-applied the same way.
4. `git mv` src/guidance/external_delta_v.c -> external_delta_v.c, src/guidance/burn_steering.c ->
   burn_steering.c, src/csm/burns/return_to_earth/program.c -> src/csm/burns/return_to_earth/program.c (no #include or
   Makefile change needed; the Makefile globs src/).
5. docs/modules/*.md and docs/MODULE_STATUS.md: word-boundary text
   substitution with every map row incl. the files/docs sections, longest
   first (210 replacements in 12 notes). `p71` was left out of the notes
   (too short to substitute in prose safely).

## Verification
* No old identifier from the map remains in code outside comments/strings
  (token scan of include/ src/ tests/).
* Comments are byte-identical in all 87 changed files except one added
  pointer comment in guid_pending.h; AGC labels such as V97TTASK, P63SPOT2,
  R51K, CANV37, GOP00FIX, S40.9, REP11A have the same counts as before.
* Compile (brief flags, -c) of every .c under src/ and tests/: baseline 21
  failures -> 5, no new failure. Remaining: src/csm/optics/sxtmark.c,
  src/io/service/service.c, src/lm/burns/ignition.c, src/lm/landing/lunar_landing.c,
  src/programs/alignment/imu_realign.c -- all are references to functions their
  paused lanes have not written yet (e.g. agc_job_lm_fine_align_recheck,
  agc_erase_extvbact, AGC_JOB_IGN_INTEGRATE).
* tools/check_rules.py src include tests: identical before/after
  (1 hard violation in tests/oracle/core_numeric.c, 95 warnings).
* Wave-1 tests (linked against the library with auto-stubs for missing
  symbols, -w): before the rename test_align, test_csm_dap and test_landing
  crashed and test_entry failed 1 check (stubs stood in for functions whose
  names disagreed between lanes); after, all link and run, test_entry,
  test_align and test_landing pass. Unchanged pre-existing failures:
  test_csm_dap 1 (line 144 pcmd, TVC erasables are stubs), test_dsky 4,
  test_navaux 1.

## Deviations from the assignment
* Duplicate definitions: the lead asked to delete the guid_pending.h copies
  of the 10 ids that now share a name with exec_pending.h. They turned out
  to be part of a pre-existing parallel registry of about 40 ids
  (TIG_0, TIG_5, TTG_0, CLOKTASK, ENGINOFF, REDO40_9 ...) and the two
  headers are never in one translation unit; deleting only the 10 would
  have broken src/csm/burns/sps_burn*.c and src/lm/burns/dps_burn.c. Only the unused
  AGC_JOB_VHHDOT (ex P11SC) line was removed. Merging the registries is an
  integration task (see RENAMES.md).
* The guid-landing section (42 rows) was missing from the reviewed
  RENAMES.md because of a lane-key typo in my phase-1 generator; it was
  restored (marked in RENAMES.md) and applied.

## Progress
* DONE map (phase 1), DONE apply, DONE verification, DONE notes.

## Lessons
* Build the review document from the same data the apply step uses, and
  count rows per section in both. A typo in a section key silently dropped
  42 reviewed-looking rows; a "rows in map == rows in document" check would
  have caught it in phase 1.
* Coccinelle is the right engine for *which* tokens change (it never
  touches comments and understands declarations vs uses), but its
  pretty-printer is not layout-preserving. Use it as the oracle and keep
  the original layout by checking token-stream equality against a
  comment-aware substitution.
* Check the tool's argument semantics on a sample before a tree-wide run:
  `--dir` is not repeatable in spatch 1.1.1, and a bare `- OLD` rule does
  not parse (wrap it in a disjunction), and typedef enums need their own
  rule (`typedef enum EN {...} TN;` with `type TN`).
* Compare before/after against a fixed commit, not HEAD: the lead committed
  mid-phase, and a first comparison against HEAD measured the renamed tree
  against itself.
* Read the code a "duplicate" lives in before deleting it: the duplicates
  were a symptom of two pending registries, not of the rename.
* Name responses by what they do (V97 T/P/E are crew responses, not engine
  states), and derive flag names from the AGC comment for the set state;
  the mechanical pass got several of these backwards.
