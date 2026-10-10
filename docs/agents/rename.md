# Assignment: rename   (tier: mechanical)

Apply docs/NAMING.md to already-committed code. Other agents are still
writing in some directories, so your scope is strictly:
  include/agc/, src/core/, src/exec/, src/nav/, src/imu/,
  src/io/dsky/pinball_*.c src/io/dsky/display_if.c src/io/dsky/noun_*.c,
  src/csm/ EXCEPT src/csm/burns/return_to_earth/program.c and src/csm/dap/,
  src/lm/ EXCEPT src/lm/dap/,
  tests/ files belonging to those lanes, docs/modules notes of those lanes.
Do NOT touch src/guidance, src/programs, src/lm/dap, src/csm/dap,
src/csm/burns/return_to_earth/program.c, src/io/service/t4rupt*.c, src/io/extended_verbs*, src/io/telemetry*,
src/io/service/service*, src/io/service/rtb_ops*, src/io/self_check* (in progress elsewhere).

Steps:
1. grep for identifiers containing program/routine/verb/noun/page numbers:
   patterns like \b[pP][0-9]{2}\b inside identifiers (p40_, _p63, P37,
   AGC_JOB_P52, r60_, v37, canv37, noun36 handler names, pg1234). Build a
   rename map BEFORE editing: docs/RENAMES.md with columns
   old identifier | new identifier | file(s). Use the canonical stems in
   docs/NAMING.md; add rows to NAMING.md for anything missing (e.g. labels
   like P63SPOT2 -> braking_phase_spot2, R51K -> fine_align_k, CANV37 ->
   change_program_check). Keep the AGC label in the header comment.
2. Rename file names too (git mv), e.g. src/csm/entry/programs.c ->
   src/csm/entry/programs.c, src/lm/landing/aborts.c -> src/lm/landing/aborts.c,
   src/exec/v37_p00.c -> src/exec/change_program_idle.c, src/imu/attitude_maneuver.c ->
   src/imu/attitude_maneuver.c, src/csm/burns/earth_orbit_insertion.c -> earth_orbit_insertion.c.
3. Apply the renames across your scope (sed with word boundaries), then
   grep the EXCLUDED directories for any old identifier you changed and
   append those occurrences to docs/RENAMES.md under "Callers outside
   scope to update at integration" (do not edit them).
4. Compile every file in scope with the brief's flags (-c). Run
   tools/check_rules.py on scope. Run the wave-1 tests that linked before
   (tests/test_core_numeric, test_exec, test_state, test_nouns, test_imu,
   test_conic, test_integration, test_navaux, oracle_*): see each lane's
   note for the link command.
5. Write docs/modules/rename.md (Progress, RENAMES summary, anything you
   could not rename and why). Do not use git except `git mv`.
