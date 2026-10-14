# Assignment: follow-up-done-lanes   (tier: review; Opus)

The sixteen reviews are done or finishing. Every item routed to a lane
whose review is already closed is now yours. Read docs/agents/review_items.md
IN FULL: the per-lane lines at the top, "ALL remaining lanes" (DMP/DMPR
audit), "Word-level API adoption", "EVA bug leads", "verify-collapse
findings", and the "From review-*" sections. Work every item whose owner
is one of: nav-aux, io-dsky, state-flags, guid-entry (+ return_to_earth),
imu, guid-landing, prog-rendz (+ targeting, displays, programs/navigation),
dap-lm, dap-csm, nav-conic, core-numeric, exec-sched, sys-misc,
prog-align, guid-powered, follow-up-done-lanes. Skip items owned by
nav-integ (src/nav/integration/**: its review is still running; do not
touch those files), sim, and the document lane; leave those lines in
place.

Order: (1) fidelity bugs and status propagation first (the
`AGC_ASSERT(st == OK) -> return BAD_ARG` sites, dropped statuses, the
EVA leads: each is a real-defect-or-false-alarm decision with the AGC
reference; PHSPRDT presets; OPTIND -0; selectmu status); (2) the API
requests (void conversions of agc_csm_sxtnb, agc_lm_rrnb, agc_lm_rrangles,
agc_svc_b5off, trfailof, trfailon; fold agc_svc_set_extvbact; DOT/VXM on
stored words and agc_vec_words_t into word.h; widen agc_imu_hw_set_cducmd;
uplink write path for P27; V91 MPAC+2 display address) with every caller
updated; (3) DMP/DMPR audit and word-level adoption in the eight
early-reviewed lanes where the review predates the word-level API, with
oracle tolerances re-tightened (report counts changed); (4) test hygiene
(no test counts assertion reports: tests/nav/auxiliary.c:502,
tests/lm/dap.c:444); dead leftovers (display_if.c:1063, maneuver.c:874,
entry_earrot duplicate); prog-rendz targeting tolerances tightened.

Each item: fix with the AGC page reference, a test where behaviour
changes, and strike the line in review_items.md (replace it with a
one-line DONE + commit-less pointer to your note). docs/CODING_STANDARD.md
and docs/NAMING.md apply; `make format` on files you edit only;
no Frama-C EVA. Note: docs/history/waves/follow-up-done-lanes.md with a
Progress checklist (one line per item, checkpoint after each), Findings,
Lessons; row in docs/MODULE_STATUS.md. Gates before the report: `make`,
`make test`, `make scenarios`, `make check`, `python3 tools/check_rules.py`
0 hard. Final report <= 15 lines.
