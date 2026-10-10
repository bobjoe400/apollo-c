# Assignment: restructure   (tier: mechanical)

All other agents are PAUSED. Move .c files (and lane-private .h files that
live beside them) into subsystem subfolders. include/agc/ stays flat; no
`#include "agc/..."` line changes. The Makefile globs src/ recursively.

Target layout (place every existing file; if a file does not fit, choose
the closest folder and list it in your note):

src/core, src/exec                         unchanged
src/io/dsky        pinball_*.c display_if.c noun_*.c keyrupt/uprupt
src/io/telemetry   downlink lists, down telemetry, uplink update (P27)
src/io/verbs       extended verbs (shared, csm, lm)
src/io/service     t4rupt*.c service routines, rtb ops, self-check
src/imu            unchanged
src/nav/conic      conic_*.c
src/nav/integration integration_*.c orbital_integration.c svdwn.c
src/nav/aux        latlong planetary measurement free_fall ephemerides star_tables
src/guidance       powered-flight subroutines, shared burn programs, servicer_common
src/programs/rendezvous   rendz_*.c and the P20s/P3x/R30/R31/stable-orbit files
src/programs/alignment    align_*.c sighting marks, R63
src/csm/entry      entry lexicon, reentry, entry programs, cm body attitude, entry dap
src/csm/dap        unchanged (rcs + tvc)
src/csm/burns      sps burn, earth-orbit-insertion monitor, return to earth, csm servicer
src/csm/prelaunch  prelaunch alignment / gyrocompassing
src/csm/optics     sxtmark, s-band antenna csm, landmark selection
src/lm/landing     landing guidance, throttle, landing displays, aborts, findcduw
src/lm/ascent      powered ascent, ascent guidance
src/lm/burns       dps burn (+rasteer), ignition, lm servicer
src/lm/dap         unchanged
src/lm/optics      aotmark, s-band antenna lm, radar lead-ins, ags init
src/lm/tests       imu performance tests (LM P07) if present

Steps:
1. Baseline: compile every .c under src/ and tests/ with the brief's flags
   (-c), record pass/fail list.
2. `git mv` files into place. Fix every relative `#include "x.h"` that now
   crosses folders (prefer moving the private header with its users; if a
   private header is shared by two folders, move it to the parent folder
   and include it as "../name.h" -- no, instead promote it to
   include/agc/ with an `agc_` prefix and note it).
3. Re-run the compile; no new failures versus baseline. Run
   tools/check_rules.py src include.
4. Update docs/ARCHITECTURE.md to the new tree (replace the layout block),
   every docs/modules/*.md and docs/agents/*.md path mention (plain text
   substitution), and docs/MODULE_STATUS.md "Files produced".
5. Write docs/modules/restructure.md: the move table, includes you fixed,
   files you had to place by judgment. Report <= 15 lines. Do not use git
   except `git mv`.
