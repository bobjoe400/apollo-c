# document -- documentation wave (PLAN.md definition of done, item 9)

## Scope

docs/agents/document.md deliverables a-e, generated from the frozen tree
(HEAD c025a30, EVA run in progress; no edits under src/ include/ tests/
sim/ tools/prove/). Writes only docs/ and tools/{depgraph,xref}.py.

## Progress

STOPPED 2026-10-12 by lead (code not final: loose ends + EVA pending).
Resume here after the EVA pass; re-check everything below against the
frozen tree.

* a. PARTIAL. tools/depgraph.py WRITTEN and run (GCC -fcallgraph-info per
  TU into --work, module = src folder two deep, three Mermaid views
  CSM/LM/shared, hubs core,core/interp,exec,exec/interrupts collapsed,
  edge/upward/header tables; writes between `<!-- depgraph:begin -->` /
  `<!-- depgraph:end -->` in docs/overview/README.md, or --stdout).
  Last run: 240 TUs, 3921 functions, 35 modules (output only in scratch).
  NEXT: write docs/overview/README.md prose (two vehicles, exec/waitlist,
  interrupts+clock, erasable/flags, DSKY loop, program lifecycle, restart
  groups) with the markers, then run depgraph.py into it. Already read
  for this: src/exec/{executive,waitlist,interrupts,fresh_start,alarm,
  restarts,phase_table,restart_tables}.c, change_program/{sequence,csm,
  lm,idle}.c, src/sim/{main,clock}.c, display_if.c prolog, keys.c KEYRUPT.
* b. docs/overview/flowcharts/*.md -- TODO
* c. "Program flow and decisions" in each lane note -- TODO
* d. docs/overview/xref.md + tools/xref.py -- TODO. Design notes: prolog
  heads come in several forms ("LABEL (FILE.agc pp.)", "A / B (C p.x /
  L p.y):", "AGC: LABEL (...)", "AGC: none --", prose then labels);
  take caps tokens before the first citation paren, keep only rope code
  labels (column-0, non-data ops, as tools/callgraph.py agc_labels);
  page of a label from the last "## Page N" line above it in the .agc;
  also scan in-body block comments "/* LABEL (... p.N) */" -> enclosing
  function.
* e. docs/TIPS.md consolidation -- TODO
* README.md "Documentation" section -- TODO

## Findings (code undocumented or inconsistent; not fixed here)

* src/exec/change_program/lm.c exec_cp_cleanup_lm: empty body, but its
  prolog says "3AXISFLG down, FLGWRD11 = LRBYBIT (R12), R04FLAG, MUNFLAG
  and XOVINFLG down" (R00 after INTSTALL, L p.229). Code or comment wrong.
* src/exec/change_program/idle.c agc_power_on prolog: "The caller follows
  with agc_fresh_dofstart()"; src/sim/main.c start() calls agc_rupt_go()
  (GOJAM -> GOPROG) instead.
* src/exec/waitlist.c: agc_waitlist_time3 prolog "0 on a failed
  precondition" and agc_waitlist_entry "BAD_ARG on a bad index or NULL
  pointer" -- neither function checks anything (stale after guard removal).

## Lessons
