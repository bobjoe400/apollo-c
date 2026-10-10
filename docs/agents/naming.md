# Assignment: naming   (tier: translation -- judgment required)

All other agents are PAUSED. You own the whole tree for this task.

Goal: apply docs/NAMING.md (read all of it, especially "Rules for deriving a
name") to every identifier and file name in include/, src/, tests/ and the
docs/modules notes. A previous mechanical attempt produced bad names and was
stopped; some of its renames are already in the tree (git log -1 shows the
checkpoint; `git show --stat HEAD~1` shows what it touched). Treat those as
candidates to re-examine, not as done.

Phase 1 -- inventory and map (STOP after this phase and report):
1. Inventory every identifier and file name containing a program/routine/
   verb/noun/page number: regex over identifiers for \b[pPrRvVnN][0-9]{2}
   (as a token or token part: p65_1b, V97E, r51k, n61, P63SPOT2, pg1234,
   _p40_, p30, canv37, v37_, noun36), plus the bad names from the first
   attempt (grep for `_entry_entry`, `_E\b`, `_P\b`, `_T\b`, `_1B`, `_1\b`,
   `TASK2`, `EJOB`, `TJOB` in identifiers). Write the raw inventory to
   docs/RENAMES_inventory.txt (one per line with file).
2. For EACH, read the AGC label's comment/code (SRC/Apollo-11/...) and
   derive the name per the rules. Write docs/RENAMES.md as a table:
   `old | new | files | AGC source + comment used (one line)`. Group by
   lane. Where you keep a name (word label), list it too with "keep".
   Add any new stems to the canonical table in docs/NAMING.md.
3. Self-check the map against the eight rules: no stutter, no opaque
   suffix, <= 4 words, one stem per concept, consistent job id / function /
   file names. Fix before reporting.
4. Report (<= 15 lines): counts, the 10 hardest decisions with your
   reasoning, and anything you could not resolve from the source.
   Do NOT apply anything in phase 1.

Phase 2 -- apply (only when the lead says go):
5. Apply the map tree-wide with word-boundary replacement (longest old
   names first so prefixes don't clobber), `git mv` for files, and update
   #include lines, the Makefile if needed, docs/modules notes, and
   docs/MODULE_STATUS.md.
6. Compile every .c under src/ and tests/ with the brief's flags (-c).
   Files that failed to compile BEFORE your change (record the list in
   phase 1 by compiling first) may still fail, but no NEW failures.
7. Run tools/check_rules.py; run the wave-1 tests that linked before (see
   each lane's note for the link command). Write docs/modules/naming.md.
