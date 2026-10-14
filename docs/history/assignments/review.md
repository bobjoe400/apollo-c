# Assignment: review-<lane>   (tier: translation -- one agent per lane)

Definition-of-done item 8. Runs after sim and oracle-cases. One review
agent per original translation lane (see docs/PLAN.md table); each reads
its lane's AGC source files side by side with the C and checks:
1. **Fidelity.** Every AGC routine in the lane's files appears in the C
   (or is a listed, justified stub). For each routine: same order of
   operations where it affects rounding, same limits, same flags
   set/cleared, same alarms, same iteration bounds, same constants
   (verify assembled values with the oracle rope, not the source text).
   Record each confirmed routine and each discrepancy in
   docs/modules/review-<lane>.md.
2. **Open questions.** Resolve the lane note's "Open questions" and the
   REVIEW items in docs/WORKLOG.md for this lane (e.g. ZOOM alias scope,
   docked 1/ACCS COEFCTR reading, BIGAOS 2*DB, UNIT precision at B27 for
   short vectors, R63 ranges) by reading the page images via the oracle
   rope where the text is ambiguous. State the answer and fix the code.
3. **Clarity.** Function and variable names per docs/NAMING.md; each
   function header has AGC label + pages; scale factors on every DP/SP
   quantity; no leftover digit-suffixed label names (e.g. agc_task_noroll1,
   csm_tvc_init4 -> spell out); comments explain intent, not restate code.
4. **Style debt.** Work the check_style findings for the lane to zero:
   shorten >80-column trailing comments, add missing declaration comments,
   closing-brace comments on long blocks, prologs per docs/templates.
   Rule 5 is spirit not count: no assertions added to reach a number.
5. Fix what you find; run `make`, `make test`, `make check` clean. Report
   <= 15 lines; findings table in the note; Lessons section.

## Scope and inputs (added at launch, 2026-10-10)
* Edit only your lane's files (docs/MODULE_STATUS.md "Files produced" and
  docs/modules/restructure.md move tables tell you where they are now) and
  your lane's tests. Shared/public headers: edit only the declarations your
  lane owns; anything else goes in your note as a request.
* Oracle findings for your lane: docs/modules/oracle-cases.md "Findings"
  table (F1-F15) and tests/oracle/<module>.c KNOWN entries. Fix the cause,
  flip the KNOWN entry to a hard check, rerun.
* Sim findings: docs/modules/sim.md "Bugs found" (sim lane may still be
  running; re-read it before you finish).
* The collapse lane kept 81 public forwarding wrappers; if one is in your
  lane and adds nothing, remove it (update include/ and callers).
* Keep `make`, `make test`, `make check` green; `make format` on edits.
* Your note: docs/modules/review-<lane>.md; status row "review-<lane>".
* Check the status-handling-2 diff in your lane (`git diff 1b72892
  a02850c -- <your files>`): every deleted guard (reason in
  tools/prove/manual.tsv), status conversion, inlined helper and
  rewritten return there is yours to confirm or repair. The
  verify-collapse lane covered only the lanes reviewed before that pass.
* Do NOT run Frama-C EVA (`make prove`, `make prove-classify`). One EVA
  run happens at the end of the project on the frozen tree (docs/PLAN.md).
  Guards marked `eva-valid-deferred` stay until then.
