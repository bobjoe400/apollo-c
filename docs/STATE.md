# STATE.md -- current state in one page (lead keeps this current)

Updated: 2026-10-12

## Running agents (max 2)
- lm-simulator (Opus): PLAN 10a, Virtual AGC LM Simulator closed loop vs
  yaAGC. Note: docs/modules/lm-simulator.md.
- nassp-integration (Opus): PLAN 10b+10c on NASSP 9 via WSL interop
  (docs/agents/nassp-integration.md: study, stock build, shim, integrated
  build, mission). Note: docs/modules/nassp-integration.md.
- document: PAUSED (a, b, d done; c partial; e + README todo); resumes
  from docs/modules/document.md Progress when a slot frees.
- Queued after lm-simulator: closed-loop-fixes (LM Simulator bugs L1-L3 +
  the document wave's code findings in review_items.md). Host: C:\NASSP9 (Orbiter 2024 x64 +
  NASSP-V9.0-Orbiter2024-437) loads Apollo 11 at the pad = the stock
  baseline for 10c. C:\Orbiter2016, C:\Orbiter2024, C:\Orbiter-nightly
  are dead ends (NASSP 8 needs Orbiter Beta R90) and can be deleted.
- CODE IS FINAL: finish lane applied the EVA pass (d53337b); all gates
  green; nothing under src/ include/ tests/ tools/prove changes any more.

## Queue after it (two at a time)
1. (review queue empty once guid-powered and nav-integ finish) -- items in docs/agents/review_items.md
   (now including the EVA bug leads).
2. follow-up-done-lanes (DMP audit + word-pair API adoption for the eight
   early-reviewed lanes).
3. Lead: rename include/agc/core/assert.h -> agc_assert.h (237 includes;
   commands in docs/modules/review-state-flags.md).
3. ORDER (user, 2026-10-12): (a) loose ends closed, `make prolog-report`
   and `make always-ok-report` at 0, gates green, tree frozen; (b) ONE EVA run at the very end (after reviews and the GUI-verified sim):
   cleanup of provably-dead guards, not a working-code step (user,
   2026-10-11). No EVA runs before that. The sim entry runs unbounded,
   alone, with no timeout (it converges; memory is the limit).
5. document (docs/agents/document.md), last.
Stretch: NASSP integration (docs/PLAN.md item 10). No own world model.

## Tree health (last verified)
make OK; make test 32 binaries 0 failures; format-check clean outside the
running lane; check_rules hard count falling (1,015 at relaunch, all
silent/sentinel guards being removed); oracle: conic, DAP-LM, DAP-CSM, DSKY
exact at tolerance 0; others within documented tolerances.

## Rules in force (see docs/CODING_STANDARD.md, docs/NAMING.md)
Power of Ten by spirit; no (void)-cast calls; no silent or sentinel
failures; assertions only at real boundaries, density report-only; no
P/R/V/N numbers in identifiers or file names; subsystem folders, no
common/glue/pending files; Uncrustify + tools/format.sh (one statement per
line, no column limit, outer-call wraps only, comments never inside
expressions, paragraph alignment); one-line functions only if NAMED or an
ACCESSOR; single normal-flow return; dispatch = 4 flat generated switches.

## Process rules
<= 2 agents; checkpoint Progress sections; on cutoff: commit, ListAgents,
relaunch only dead agents with resume; lead commits; agents never spawn
agents; assignments in docs/agents/<name>.md, one-line launch prompts.
