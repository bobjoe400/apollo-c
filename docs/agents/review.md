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
   Rule-5 density >= 2.0 per file with real pre/postconditions.
5. Fix what you find; run `make`, `make test`, `make check` clean. Report
   <= 15 lines; findings table in the note; Lessons section.
