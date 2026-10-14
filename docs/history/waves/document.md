# document -- documentation wave (PLAN.md definition of done, item 9)

## Scope

docs/agents/document.md deliverables a-e plus the root README.md
"Status" and "Documentation" sections, generated from the final tree
(HEAD 99b2c6e; no edits under src/ include/ tests/ tools/prove src/nassp).
Writes only docs/, README.md and tools/{depgraph,xref}.py.

## Progress

RESUMED 2026-10-12 on HEAD 99b2c6e (closed-loop-fixes; code final).
Scratch: SCRATCH/document/ (decisions.py, assemble.py, digest.py,
compact.py, addsec.py, narr/<lane>.md, extra/<note>.md, lanes/, dig/).

* d. DONE tools/xref.py + docs/overview/xref.md. Regenerate:
  `python3 -I tools/xref.py`. (Re-run at HEAD: see below.)
* a. DONE docs/overview/README.md (module map: `python3 -I
  tools/depgraph.py --work <scratch>/ci`).
* b. DONE docs/overview/flowcharts/ (index + 10 flowcharts).
* c. DONE 2026-10-12: every docs/modules/<lane>.md has "## Program flow
  and decisions" except nassp-integration.md (being edited by its lane:
  mtime within 10 min at each check). 19 code lanes: prose from the code
  (narr/) + generated tables (decisions.py: decision points by function,
  shared erasables with scale) via assemble.py. lm-simulator and
  closed-loop-fixes: hand-written from tools/lm_sim.py and the fixed
  functions; review-*, tree-wide passes, oracle*, fix-b6, loose-ends,
  document: short sections pointing at the owning lanes (extra/, addsec.py).
  nassp-integration.md SKIPPED: its lane is running (note written 17:46,
  src/sim/socket.c 17:44; lead: read, do not edit). Its section is ready
  in SCRATCH/document/extra/nassp-integration.md; apply with
  `python3 SCRATCH/document/addsec.py nassp-integration` once the lane ends.
* e. DONE 2026-10-12: docs/TIPS.md rewritten by topic (11 sections) from
  all 45 Lessons sections plus the NASSP milestones (WORKLOG), deduplicated,
  each tip citing its lane.
* README.md "Status" and "Documentation" sections DONE 2026-10-12.
* Re-run at HEAD 99b2c6e DONE: xref 3071 labels -> 2984 functions, 4170
  rows; depgraph 240 TUs, 3986 functions (tools/depgraph.py now skips the
  host-only src/nassp/, as the Makefile's LIB_SRCS does).
  flowcharts/software-restart.md GOTOPOOH node updated for D1.

## Findings (code undocumented or inconsistent; not fixed here)

The five findings of the first pass (stale erasable.c / state.c getter
prologs, repeated assertions, imu_realign.c CALL citations, P40TMP silent
recovery, agc_exec_vac_claim argument) are FIXED (closed-loop-fixes).
New, from the HEAD 99b2c6e pass (comments only, except the layering one):

* File prologs that promise an assertion the file no longer has (0
  AGC_ASSERT in the file): src/lm/dap/arith.c ("fail an assertion and
  return 0"), src/lm/dap/gts.c, src/lm/dap/rcs/rcsmon.c,
  src/lm/dap/rcs/spsrcs.c ("BAD_ARG when an internal assertion fails"),
  src/nav/auxiliary/util.c ("a failed assertion returns 0 / the zero
  vector"), src/io/service/rtb_ops.c ("fail an assertion and give 0 /
  BAD_ARG"). Same family as the erasable.c finding; prolog_check.py reads
  function prologs, not the file box.
* src/core/erasable.c file PURPOSE still says each accessor "is a plain
  get/set with a range check" (line 9); its ABNORMAL paragraph (fixed)
  says they hold no checks.
* Layering (definition of done item 2): the module map now has upward
  edges from core: src/core/ecadr.c includes csm/dap.h, csm/entry.h,
  guidance, imu, io/service, nav, programs headers and calls them (core ->
  csm 4, programs 4, guidance 2 call edges; nassp-integration's address
  map), and src/imu/gyro.c:gyro_words calls agc_align_ram() in
  src/programs/alignment (imu -> programs 1; closed-loop-fixes L2).
  Candidates: move the address map above core (e.g. src/io or its own
  module) and give OGC/IGC/MGC an imu-owned home that alignment reads.
* Vacuous status plumbing left by collapse: src/io/dsky/pinball/keys.c
  agc_dsky_uprupt, ERROR RESET branch: `agc_status_t st = AGC_STATUS_OK;
  return ((st == AGC_STATUS_OK) ? pin_keycom(low) : st);`.
* src/core/ecadr.c has a free-form file comment, not the boxed prolog
  (FILE NAME / PURPOSE / ... ) the other files use.

## Lessons

* Prolog boxes are a good digest of a lane (PURPOSE, ABNORMAL, function
  heads) but not proof: six file boxes still promised assertions the
  files no longer contain. Count the construct in the body before
  repeating a prolog's claim.
* Extract what can be extracted (flag tests, alarm calls, channel bits,
  job/task starts, accessor reads and writes) from the bodies by script,
  and spend the reading on the branches and their order; the tables keep
  the prose honest and are cheap to regenerate at a new HEAD.
* A tool that globs src/** must follow the build's own exclusions: the
  module map failed on the host-only src/nassp shim until depgraph.py
  skipped what the Makefile's LIB_SRCS skips.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This lane writes documentation only. Its two generators: `tools/depgraph.py` (ctags + the #include and call graph -> the Mermaid module map in docs/overview/README.md section 8) and `tools/xref.py` (the AGC labels named in each function prolog, checked against the rope listings -> docs/overview/xref.md). Every lane section above was assembled from prose written by reading the code plus tables extracted from the function bodies.
