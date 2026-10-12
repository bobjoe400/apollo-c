# STATE.md -- current state in one page (lead keeps this current)

Updated: 2026-10-11

## Running agents (max 2)
- review-exec-sched (Opus, resume): docs/agents/review_items.md exec-sched
  line + EVA bug leads; note docs/modules/review-exec-sched.md.
- verify-collapse (Opus): independent check of the status-handling-2 diff
  in the eleven already-reviewed lanes only; the remaining reviews check
  it in their own files (review.md item). Note: docs/modules/verify-collapse.md.

## Queue after it (two at a time)
1. review-prog-align, review-sys-misc, review-guid-powered,
   review-nav-integ (resume) -- items in docs/agents/review_items.md
   (now including the EVA bug leads).
2. follow-up-done-lanes (DMP audit + word-pair API adoption for the eight
   early-reviewed lanes).
3. Lead: rename include/agc/core/assert.h -> agc_assert.h (237 includes;
   commands in docs/modules/review-state-flags.md).
4. sim (resume; docs/agents/sim.md END GOAL): acceptance work -- yaAGC
   command-line drop-in, scenario e, GUI runs -- AFTER all reviews (user,
   2026-10-11), so it is done once on settled flight code.
5. ONE EVA run at the very end (after reviews and the GUI-verified sim):
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
