# STATE.md -- current state in one page (lead keeps this current)

Updated: 2026-10-11

## Running agents (max 2)
- status-handling-2 (Opus, runs ALONE): guards by dataflow/EVA, fallible ->
  status returns, one-liner inlining, label-rule collapse, dead returns, then
  ONE EVA run on the finished tree (`make prove`, deferred verdicts in the
  five unreviewed lanes). Note: docs/modules/status-handling-2.md.

## Queue after it (two at a time; sim first)
1. sim (docs/agents/sim.md): agc_sim as drop-in for yaAGC under the
   VirtualAGC GUI; no device models; headless scenarios via tools/sim_client.py.
2. review-exec-sched, review-prog-align, review-sys-misc,
   review-guid-powered, review-nav-integ (resume) -- items in
   docs/agents/review_items.md; each ends with `make prove-classify`.
3. follow-up-done-lanes (DMP audit + word-pair API adoption for the eight
   early-reviewed lanes).
4. Lead: rename include/agc/core/assert.h -> agc_assert.h (237 includes;
   commands in docs/modules/review-state-flags.md).
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
