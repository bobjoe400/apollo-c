# STATE.md -- current state in one page (lead keeps this current)

Updated: 2026-10-13

## Running agents (max 2)
- nassp-integration (Opus, resumed after the restart): our PDI to
  touchdown vs the stock record, then liftoff, rendezvous, entry. Runs
  from docs/modules/nassp-integration.md Progress (CHECKPOINT bullet) with docs/agents/nassp-integration.md
  (hard rules: unattended runs, held warp, one Orbiter at a time, short
  polls, answer via SendMessage to main; LM runs at 1x: warp tumbled the
  stock LM). Host: C:\NASSP9 stock baseline (untouched), C:\NASSP9-ref
  (stock + LEM test hook 0002), C:\NASSP9-dev (ours); sources under
  C:\Users\cnama\Downloads\nassp-src and nassp-work; the lane's scratch
  is backed up at C:\Users\cnama\nassp-work\scratch-backup.
  Next: rerun OUR PDI from the scenario (stock LANDED unattended:
  P64 at TIG+496.7 s, auto P65 at +660.8 s, touchdown +706.9 s at
  -0.85 m/s, lng 23.7265 lat 0.6695), compare, then liftoff, rendezvous,
  entry.
- Mission status on our AGC in NASSP 9: insertion+uplink, TLI, MCC-2, LOI,
  TEI, LM DOI PASS; PDI pending. Flight bugs found by the mission: 4, all
  fixed with tests (FANG/EK3VAL map, V82 on a hyperbola, VGDISP = DELVSAB,
  one TLAND).
- Definition of done: all nine items met (PLAN.md status block); `make
  style` 0 tree-wide; layering restored; docs generated at HEAD.

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
