# Assignment: restructure   (tier: mechanical)

All other agents are PAUSED. Move .c files (and lane-private .h files that
live beside them) into subsystem subfolders. include/agc/ stays flat; no
`#include "agc/..."` line changes. The Makefile globs src/ recursively.

Rule: a subfolder needs >= 3 files; otherwise keep the file in the parent.
Rule: three or more files in one folder sharing a name prefix become a
subfolder named by that prefix and lose it (sps_burn_*.c -> sps/burn.c,
clock.c, ...; noun_*.c -> nouns/; agc_interp_*.c -> interp/). Project
prefix `agc_` is dropped from .c file names (it stays on public headers).
Rule: never create a `common/` folder; shared glue lives at the parent level
and only specialised subsystems (rcs, tvc, gimbal) get subfolders.

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
src/csm/dap/rcs    RCS digital autopilot, jet selection logic, automatic
                   maneuvers, RCS DAP executive programs, DAP data load (R03)
src/csm/dap/tvc    TVCINITIALIZE, TVCEXECUTIVE, TVCMASSPROP, TVCRESTARTS,
                   TVCDAPS (pitch/yaw), TVCSTROKETEST, TVCROLLDAP
src/csm/dap        (parent) MYSUBS, DAP state -- shared glue stays at the parent
src/csm/burns      sps burn, earth-orbit-insertion monitor, return to earth, csm servicer
src/csm/prelaunch  prelaunch alignment / gyrocompassing
src/csm/optics     sxtmark, s-band antenna csm, landmark selection
src/lm/landing     landing guidance, throttle, landing displays, aborts, findcduw
src/lm/ascent      powered ascent, ascent guidance
src/lm/burns       dps burn (+rasteer), ignition, lm servicer
src/lm/dap/rcs     DAP idler, P-axis, Q/R-axis, TJET law, Kalman filter,
                   T6RUPT jet timing, RCS failure monitor
src/lm/dap/gimbal  trim gimbal control system, AOS task and job
src/lm/dap         (parent) DAP interface subroutines, arithmetic, state,
                   SPS back-up RCS control -- shared glue stays at the parent
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
5. Write docs/history/waves/restructure.md: the move table, includes you fixed,
   files you had to place by judgment. Report <= 15 lines. Do not use git
   except `git mv`.

## File names after the move (added mid-task)

Once a file sits in its subsystem folder, drop any filename prefix that
only repeats an enclosing folder name, for .c and private .h alike:

    src/nav/conic/conic_kepler.c          -> kepler.c
    src/nav/integration/integration_init.c-> init.c
    src/csm/entry/entry_programs.c        -> programs.c
    src/csm/dap/tvc/tvc_daps.c            -> daps.c
    src/lm/dap/rcs/lm_dap_tjet.c          -> tjet.c
    src/io/dsky/pinball_state.c           -> state.c   (folder already says dsky)
    src/lm/landing/landing_guidance.c     -> guidance.c
    src/csm/optics/sxtmark.c              -> sxtmark.c (keep: AGC name, not a folder)

Keep words that carry meaning rather than location (AGC names like
sxtmark, aotmark, servicer, kalcmanu). Vehicle suffixes `_csm`/`_lm` stay
in shared folders (src/io/verbs/extended_verbs_csm.c) and drop inside a
vehicle folder (src/csm/optics/s_band_antenna.c). If two files would
collapse to the same name in one folder, keep one distinguishing word on
each. Update every `#include "..."` of a renamed private header, every
path in docs/modules, docs/agents, MODULE_STATUS.md, and the link
commands in the lane notes. Record old -> new in your move table.
Public headers in include/agc/ keep their names.
