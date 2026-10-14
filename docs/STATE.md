# STATE.md -- current state in one page (lead keeps this current)

Updated: 2026-10-13

## Running agents (max 2)
- guide (Opus): docs/guide/, a reader's guide to the AGC through the C
  (12 chapters + glossary) -- the document the project was for.
- docs-split (Sonnet): docs/modules/*.md keep only explanation; lane
  diaries move verbatim to docs/history/lanes/.
- Code CLOSED 2026-10-14: documentation at the final code (8ab9a3d),
  close-out fixes (0b4217a). Code complete as of a5bcdb1: definition of done
  items 1-9 met; stretch item 10 done (LM Simulator loop; the full Apollo
  11 mission in NASSP 9 on our AGC, all ten milestones matching stock on
  decisions and end state). 17 flight bugs from the mission fixed with
  tests. All gates green: make, make test (38 binaries + 14 scenarios),
  make check (rules 0, style 0), prolog 0, always-ok 0, lm-sim 34/34,
  nassp-replay 5/5, no upward edge.
- Known limitations (documented): zero-time executive (timing-born
  behaviours are expected deviations, PLAN.md principle); scenario
  restore cannot resume a variable restart phase whose PHSNAME is a rope
  address; DOWNLIST_SKIP words (docs/modules/telemetry.md).
- Risks noted by the last lane: milestones 1-9 not reflown after bug 15
  (SIGNMPAC major-word sign; matters only when a term overflows to 1);
  stock's V77 OPR ERR not root-caused; the 42 s CDH and RR mark-count
  differences attributed to execution time, not proven.
- Optional follow-ups: NASSP upstream items (review_items.md), the -O0
  conversion warning (free_fall.c:443), hi-res Orbiter textures.

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
5. document (docs/history/assignments/document.md), last.
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
