# prolog-consistency -- boxed prologs versus bodies after the rewrite passes

## Scope
The `make prolog-report` findings (tools/prolog_check.py) in every
directory except src/exec/**, src/csm/entry/**, src/sim/** (loose-ends lane;
the lead hands those findings over afterwards). Prologs edited in place: no
renames, no restructuring, no AGC_ASSERT added or deleted.

Method: for each finding the prolog and the body were read side by side;
where the prolog's description named an action the body did not show, the
AGC label it cites was read (tools/agc_label.py) to decide which side was
wrong. RETURNS now names exactly the statuses the function can return,
with propagated ones attributed to their source call; "always OK" is
written out where no failure path is left. ARGUMENTS rows for a dropped
`ctx` were removed (`ARGUMENTS: none` when nothing is left).

## Progress by directory
Start: 118 findings tree-wide (call-missing 45, ctx-stale 12,
returns-missing 60, returns-void 1); 112 in scope, 6 in the excluded
directories.

| Directory | State |
|-----------|-------|
| src/csm (burns, dap, navigation, prelaunch_*) | DONE (prologs) |
| src/guidance | DONE (prologs) |
| src/io (dsky nouns, pinball, t4rupt) | DONE (prologs; call-missing by tool rule) |
| src/lm (dap, alignment, landing, navigation, radar) | DONE (prologs; call-missing by tool rule) |
| src/nav (auxiliary, conic) | DONE (prologs) |
| src/programs (displays, navigation, targeting) | DONE (prologs) |
| second round after the returns-missing tightening (17) | DONE (prologs) |
| src/exec, src/csm/entry, src/sim (6 findings) | NOT IN SCOPE (loose-ends) |

`make prolog-report` outside the excluded directories: 0.

## Findings
* In scope, first round: 112 = call-missing 42 (all false positives,
  closed by tool rule 1), ctx-stale 11, returns-missing 58, returns-void 1.
* Prolog fixed: 70 first-round findings (ctx-stale 11, returns-missing
  58, returns-void 1) plus 17 second-round returns-missing, and 3
  file-prolog ABNORMAL TERMINATION paragraphs (nav/conic geom.c,
  apsides.c, kepler.c) that still promised BAD_ARG for a NULL pointer or
  an unknown body.
* Body fixed: 0. Every body whose prolog named an action it lacked was
  checked against the cited AGC text (MATRXJOB, NOATTCNT, NEGUSUM ..
  LOUPE, P64CEED, DORSAMP, FAZC .. FAZAB5, GET+MGA, B5OFF, AMBGUPDT's
  FLAGWRD6 test); in each the body does what the AGC does and the prolog
  was stale.
* returns-extra found while reading: agc_lm_rootpsrs returns
  AGC_STATUS_NO_CONVERGE (BADROOT) that its prolog did not name; fixed.

### Tool changes (tools/prolog_check.py)
1. call-missing, systematic false positive (42 of 45): a `name()` in the
   prolog was treated as a call claim wherever it stood. Rule added
   (`call_claims`): a mention is provenance, not a claim, when it is in an
   ARGUMENTS row (`d  lm_dap_t *  I/O  DAP erasables (lm_dap_state())`: the
   caller fetched the pointer), when it is followed by `->`
   (`agc_align_ram()->qmaj`: an object the function uses), or when it is
   the agent of a passive (`selected by agc_state_init()`).
2. returns-void: `RETURNS says none` now needs `RETURNS: none`, not the
   word "none" anywhere in the text (t4_jump's "OK when none" was flagged;
   its prolog was rewritten anyway).
3. returns-missing tightened (false negatives): the old rule skipped every
   function with a non-literal return ("propagates"), so stale BAD_ARG text
   above `return (st)` was never seen. It now computes, over the whole
   tree, what each agc_status_t function can return: the AGC_STATUS_*
   literals it returns or assigns, plus everything its status-returning
   callees can return when it returns a variable or call (fixpoint over the
   call graph from libclang; static functions keyed by file). A function
   with an agc_status_t parameter or one that passes a status by address
   (out-parameter) can return anything and is not checked. NO_CONVERGE
   added to the status words. This surfaced 17 more stale prologs in scope
   (fixed) and none in the excluded directories.

### Observations for the lead (not changed: outside this lane's remit)
* Always-OK status functions. 50-odd functions in scope now document
  "AGC_STATUS_OK (always)" (their guards were deleted, nothing left can
  fail), and some callers still test their status (e.g. csm_dap_set_t5,
  agc_conic_ctx_init_, lm_dap_set_t5adr, agc_guid_pipasr,
  agc_conic_apsides). Rule 7 says an operation that cannot fail returns
  void; converting them is a void-conversion pass, not a prolog fix. The
  list is every RETURNS containing "(always" in the diff of this lane.
* returns-extra is listed in the tool docstring but not implemented. A
  scratch count: 172 functions outside the excluded directories return a
  literal status their RETURNS does not name (BAD_ARG 114, ALARM 36,
  NO_CONVERGE 7, NOT_FOUND 5, BUSY 4, ...); many describe it in words
  ("an input check", "the alarm"), so a rule needs a vocabulary before it
  is a gate. Not added.
* src/lm/radar/leadin.c agc_job_radar_sample_done: `if (!R77 &&
  AGC_ASSERT(loc in RSTACK))` skips the RSTACK store silently when the
  assertion fails (rule 7 silent guard that check_rules does not see,
  because the assertion is a conjunct). Prolog now says so.
* src/csm/dap/tvc/init.c csm_tvc_mrclean ends `if (st != OK) return st;
  return st;` (dead early return).
* src/programs/navigation/common.c agc_rendz_get_mga: Luminary's GET+MGA
  (P34-35_P74-75.agc L line 1406) does `VLOAD UNIT / UNIT` (UNIT twice),
  Comanche once; the C unitizes once for both vehicles. Possibly an LSB
  difference on the LM; not checked against the oracle.
* Prologs that still say "BAD_ARG for a NULL pointer" where BAD_ARG is
  reachable for another reason (nav/conic lambert.c, timeradius.c,
  timetheta.c) pass the status-level check; their wording is stale but
  outside what the tool can see.

### Review items 7 and 8 (document wave, comment items)
agc_power_on's header and the waitlist time3/entry comments are in
src/exec, which loose-ends is editing (its working-tree diff already
rewrites both); left to loose-ends.

## Lessons
* A "propagates, so skip" escape in a consistency check hides exactly the
  stale text it exists to find; compute what the callee can return instead.
* Most stale RETURNS after a guard-deletion pass reduce to "always OK";
  that is the signal for a void conversion, which belongs to its own pass.

## Gates (2026-10-12, end of run)
`make` OK; `make test` rc 0 (every binary 0 failures, scenarios 0 failed);
`make check` rc 0 (check_rules 0 hard, 0 warnings; format-check clean);
`python3 -I tools/check_rules.py src` 0 hard; `make prolog-report` 6
findings tree-wide, all in src/exec (3) and src/csm/entry (3), 0 elsewhere.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ A tree-wide pass (prologs made to match the bodies): it changed code owned by the translation lanes without changing what the program decides, so it has no program flow of its own. The flows as the code now stands are in the lane sections: [exec-sched](../../docs/modules/exec-sched.md#program-flow-and-decisions), [core-numeric](../../docs/modules/core-numeric.md#program-flow-and-decisions), [state-flags](../../docs/modules/state-flags.md#program-flow-and-decisions), [nav-conic](../../docs/modules/nav-conic.md#program-flow-and-decisions), [nav-integ](../../docs/modules/nav-integ.md#program-flow-and-decisions), [nav-aux](../../docs/modules/nav-aux.md#program-flow-and-decisions), [io-pinball](../../docs/modules/io-pinball.md#program-flow-and-decisions), [io-nouns](../../docs/modules/io-nouns.md#program-flow-and-decisions), [imu](../../docs/modules/imu.md#program-flow-and-decisions), [guid-powered](../../docs/modules/guid-powered.md#program-flow-and-decisions), [guid-landing](../../docs/modules/guid-landing.md#program-flow-and-decisions), [guid-entry](../../docs/modules/guid-entry.md#program-flow-and-decisions), [p37](../../docs/modules/p37.md#program-flow-and-decisions), [dap-lm](../../docs/modules/dap-lm.md#program-flow-and-decisions), [dap-csm](../../docs/modules/dap-csm.md#program-flow-and-decisions), [prog-rendz](../../docs/modules/prog-rendz.md#program-flow-and-decisions), [prog-align](../../docs/modules/prog-align.md#program-flow-and-decisions), [sys-misc](../../docs/modules/sys-misc.md#program-flow-and-decisions), [sim](../../docs/modules/sim.md#program-flow-and-decisions); the system view is [docs/overview/README.md](../../docs/overview/README.md) and its flowcharts.
