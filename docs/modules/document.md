# document -- documentation wave (PLAN.md definition of done, item 9)

## Scope

docs/agents/document.md deliverables a-e, generated from the frozen tree
(HEAD c025a30, EVA run in progress; no edits under src/ include/ tests/
sim/ tools/prove/). Writes only docs/ and tools/{depgraph,xref}.py.

## Progress

PAUSED 2026-10-12 by lead (NASSP lane needs the slot). Resume here.
Tree: FINAL (HEAD d53337b). README.md (repo root) and docs/TIPS.md untouched.

* d. DONE tools/xref.py + docs/overview/xref.md (3067 labels -> 2976
  functions, 4158 rows). Regenerate: `python3 -I tools/xref.py`.
* a. DONE docs/overview/README.md (sections 1-7 prose; section 8 module
  map written by `python3 -I tools/depgraph.py --work <scratch>/ci`).
* b. DONE docs/overview/flowcharts/: README.md (index), fresh-start,
  software-restart, job-preemption, waitlist-tick, program-change,
  sps-burn, imu-realign, lunar-landing, earth-entry, powered-ascent.
* c. PARTIAL, nothing written into docs/modules/ yet (all lane notes
  unchanged). Scratch helpers in SCRATCH/document/: decisions.py (per-lane
  generated tables: decision points by function + shared erasables with
  scale; lane -> file globs in its LANES dict), assemble.py (appends or
  replaces "## Program flow and decisions" = narr/<lane>.md + generated
  tables), narr/exec-sched.md (the only narrative written). If scratch is
  gone, rewrite both helpers from this description. NEXT: narratives for
  core-numeric, state-flags, nav-conic, nav-integ, nav-aux, io-pinball,
  io-nouns, imu, guid-powered, guid-landing, guid-entry, p37, dap-lm,
  dap-csm, prog-rendz, prog-align, sys-misc, sim (flowcharts already
  cover guid-*, prog-align, exec); then `python3 decisions.py <lane> >
  lanes/<lane>.md` and `python3 assemble.py <lanes...>` from repo root.
* e. docs/TIPS.md consolidation -- NOT STARTED (read every note's Lessons).
* README.md "Documentation" section + Status line -- NOT STARTED.

## Findings (code undocumented or inconsistent; not fixed here)

Earlier three findings (exec_cp_cleanup_lm empty, agc_power_on caller,
waitlist time3/entry prologs) are RESOLVED in the final tree. New:

* src/core/erasable.c file prolog and 12 getter prologs (+3 in state.c,
  include/agc/core/state.h:109) say a failed AGC_ASSERT makes a getter
  return 0 / a setter ignore the value; the accessors contain no
  assertion at all (stale after guard removal).
* Assertions that restate the line above (cannot fail; CODING_STANDARD
  rule 5 says such are not written): lm/ascent.c agc_lm_ascent (FLVR),
  as_exitvr, agc_lm_job_cutoff (FLRCS); lm/landing/lunar_landing.c
  agc_lm_job_astnret (QMAJ); lm/landing/guidance.c
  agc_lm_start_velocity_nulling (WCHVERT); change_program/idle.c
  agc_job_go_idle_fix (EXTVBACT).
* programs/alignment/imu_realign.c agc_job_realign_nominal_orientation and
  realign_gimbal_angles prologs name "AGC: CALL (INTERPRETER.agc)" instead
  of S52.3 / S52.2 (so xref lists them under CALL).
* csm/burns/sps/burn.c agc_job_preburn_state_vector: a failed
  AGC_ASSERT(P40TMP >= 0) silently sets P40TMP = 1 (silent recovery).
* exec/executive.c agc_exec_vac_claim prolog calls vac_index an input
  "index, 0..4"; it is an output pointer (-1 when none).

## Lessons
