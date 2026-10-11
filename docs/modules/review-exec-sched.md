# review-exec-sched -- review of the exec-sched lane

## Scope
docs/agents/review.md items 1-5 for src/exec (executive, waitlist, phase
table, restarts, restart tables, alarm/abort, interrupts, fresh start,
change_program/, nouns.c, dispatch.c via tools/gen_registry.py),
include/agc/exec/{exec,jobs}.h (exec-owned declarations), tests/exec/exec.c.
Routed items (docs/agents/review_items.md): deferred-restart vs immediate
ENEMA; V37 handoff completeness; activity-light mask fix; P23/R22LEM
restart priority; ZOOM/P40ZOOMA group-3 question.

## Progress
- DONE fidelity: executive.c (R1 fixed)
- DONE fidelity: waitlist.c (incl. LM KILLTASK)
- DONE fidelity: phase_table.c (R2 comment)
- DONE fidelity: restarts.c, restart_tables.c (all entries of both ropes vs source; 600SECS and LM 1.2SPOT words read from the rope; R3 comments)
- DONE fidelity: alarm.c
- DONE fidelity: interrupts.c, erasable.c (T5RUPT .5SEC test, T6LOC/T6ADR, lead-ins match; nouns.c and dispatch.c are glue, no AGC routine)
- DONE fidelity: fresh_start.c (R4; SWINIT, IM30INIF/IM30INIR/IM33INIT/IFAILINH/9,6,4 read from both ropes)
- DONE fidelity: change_program/{v37,csm,lm,idle}.c (R5-R9; PREMM1/FCADRMM1/DNLADMM1 both ropes, 3.1SEC and OCT44571 from the rope)
- DONE routed items (5), see "Routed items"
- PARTIAL style debt (1917 findings at start). DONE (0 findings): dispatch.c
  + jobs.h (tools/gen_registry.py rewritten: per-vehicle switch trees, no
  ternaries, boxed prologs; mapping verified identical to the old file),
  executive.c, waitlist.c (LONGTIME/LONGCADR statics renamed
  pending_longtime/pending_longcadr: check_style's `long` type regex
  matched `longtime = ...` as a declaration), alarm.c.
  PARTIAL phase_table.c: prologs done; left: file prolog, scale comment
  on `mm` (line ~840), ternary comment (line ~334).
  TODO: restarts.c, restart_tables.c, fresh_start.c, interrupts.c
  (nested ternary in agc_rupt_time), erasable.c, nouns.c, internal.h,
  change_program/{v37,csm,lm,idle}.c (rule-5 density < 2 in all four,
  nouns.c, interrupts.c, phase_table.c, restarts.c), exec.h,
  tests/exec/exec.c.
  Tools (scratch, review-exec-sched/): prolog.py FILE OV.py (boxed
  prologs from existing comments + overrides), annot.py FILE MAP (append
  decl comments by line), fp.py FILE SPEC.py (file prolog template).
  All src/exec files compile (-Werror) as of this checkpoint.
- TODO make / make test / make check (run uncrustify only on exec files,
  not `make format`: other lanes are mid-edit)

## Findings
| # | file:line | AGC (label, page) | finding | action |
|---|---|---|---|---|
| R1 | src/exec/executive.c activity_light | DUMMYJOB CS TWO / WAND DSALMOUT; NUDIRECT WOR (C p.1220, L p.1116) | WAND mask fix (077775) confirmed; but `st` was read uninitialised on the "on" path (UB; assertion on garbage) | removed `st`; static-asserted the mask; postcondition reads the channel 11 latch |
| R2 | src/exec/phase_table.c NEWPHASE | NEWPHASE (C RESTART_TABLES p.220) | header cited "L p.243": NEWPHASE is Comanche-only (Luminary uses GNUFAZE5 in SERVICER) | comment fixed |
| R3 | src/exec/restart_tables.c rst_read_sp | ITSINDIR NDX A / CA 1 (C p.1415) | comments said SAVET-30 / P40TMP "+1 low word"; the AGC reads the first word, written by TS (TIG-30 L p.735, P47CSM C p.691); the C writers store the SP as the DP, so the value matches | comments corrected; 5.17 marked GEOSTRT4 (= ENDOFJOB, C p.448) |
| R4 | src/exec/fresh_start.c phase_tables_ok | PTBAD -> DOFSTRT1 (L p.218) / DOFSTART (C p.188) | the 1107 fresh start skipped the other modules' DOFSTART resets (exec_up_dofstart: SMODE, UPSVFLAG, DNLSTCOD, TRKMKCNT, LM DAP words, CSM OPTIND/OPTMODES); LM THRUST = -0 was done in the up-call although only DOFSTART (not DOFSTRT1) writes it | dofstart(slap1, engine_off) is the one entry; PTBAD calls it with engine_off = false; THRUST moved into the LM engine-off branch |
| R5 | src/exec/change_program/idle.c exec_initsub_csm | INITSUB (C p.203) | OPTIND = -1 was not done ("optics owner's") | CA NEGONE / TS OPTIND added (agc_erase_set_optind) |
| R6 | src/exec/change_program/idle.c GOP00FIX | GOP00FIX TC CLEARMRK +2 (C p.194, L p.223) | C called full CLEARMRK, which also zeroes EXTVBACT | EXTVBACT kept around agc_dsky_clearmrk(); request to io-dsky for a CLEARMRK +2 entry |
| R7 | src/exec/change_program/sequence.c CANV37 | CANV37 TEMPFLSH = R00AD, PHASCHNG OCT 14 (C p.199, L p.229) | phase 4.1 restarts the C INITDSP, which re-issues the last display: a restart inside CANV37 lost the V37 (AGC resumes at ROC/R00) | group 4 type C job phase restarting CANV37 (AGC_JOB_CHANGE_PROGRAM_CONTINUE) at PRIO30; V37XEQ restart priority PRIO30 (INITDSP's PRIOCHNG to RESTREG) instead of the program's |
| R8 | src/exec/change_program/sequence.c POOH, V37XEQ | CAF PRIO5 / TS PHSPRDT2 (C p.200, L p.229); TS PHSPRDT4 (C p.202, L p.232) | not modelled (no exec entry) | new agc_phase_set_priority(group, prio) (exec.h) used in both |
| R9 | src/exec/change_program/sequence.c NOUVEAU (LM) | SEUDOP00 TS DNLSTCOD / TS AGSWORD (L p.229) | AGSWORD was set only for P00 | AGSWORD = DNLADMM1 for every new LM program |

## Routed items
* **Deferred restart vs immediate ENEMA.** AGC WHIMPER fakes a RESUME into
  ENEMA, so nothing after the BAILOUT/POODOO call runs. C requests ENEMA and
  runs it at the next service point (end of the job in agc_exec_run_one, end
  of T3RUPT incl. the RUPTAGN chain, end of an interrupt lead-in). Observable
  differences: (1) code a caller runs after an ignored BAILOUT status (the
  AGC never returns) -- erasable writes there survive ENEMA, table writes
  are wiped by STARTSB2; callers must return the status (rule 7); (2) a
  BAILOUT inside ENEMA's own NXTRST loop (e.g. 1201/1202 while restarting
  groups): the AGC abandons the loop and re-enters ENEMA at once, C finishes
  the loop and runs a second ENEMA; the jobs/tasks of the first pass are
  wiped by the second STARTSB2, only LONGTIME/LONGCADR of a later group's
  LONGCALL could differ; (3) callers see AGC_STATUS_BAILOUT/POODOO returned
  instead of no return; (4) no timing difference: nothing else runs
  between the request and the service point (all synchronous). Kept.
* **V37 handoff completeness.** Traced V37 -> CANTR00/CHECKTAB/ISSERVON ->
  CANV37 -> POOH/NOUVEAU -> RENDV00/STATQUO -> GOPROG2 -> 4.1 -> V37XEQ for
  both ropes. Gaps fixed: R5, R6, R7, R8, R9. Remaining (documented):
  RESTREG has two copies (exec fresh_start.c, io-dsky display_if.c); C
  V37XEQ does not reproduce the PHSPRDT3/TBASE4 debris of MMTEMP/BASETEMP;
  LM POOH's EBANKTEM = -OCT120 side effect not modelled (no EBANK).
* **Activity-light mask fix verified** (R1): WAND mask is 077775 (static
  assert); the uninitialised `st` read on the "on" path is gone.
* **P23/R22LEM restart priority.** The AGC presets PHSPRDT2 (R22LEM3
  PRIO26, P23 PRIO13, CSM R22 PRIO26, POOH PRIO5) for a later type C
  PHASCHNG with D = 0 (e.g. OCT 04022), whose restart then uses it;
  store_var keeps PHSPRDTn when D = 0, so only the "TS PHSPRDTn" entry was
  missing. Added `agc_phase_set_priority(group, prio)` (exec.h), used by
  POOH; the R22LEM3/P23/R22 call sites (prog-rendz files) should call it
  (request to prog-rendz review; without it a group-2 restart there has
  PHSPRDT2 = 0 and start_job asserts). Same entry serves CSM reentry.c
  PHSPRDT5 = PRIO16 (guid-entry).
* **ZOOM / P40ZOOMA group 3.** Confirmed from the rope: ZOOM and P40ZOOMA
  are both 17,2022; P40ZOOM 17,2015, P63ZOOM 17,2007. The waitlist holds
  P40ZOOM/P63ZOOM, so COMFAIL2's KILLTASK ZOOM only removes a ZOOM put on
  the list by the 3.3 restart; the 3.3 restart (ZOOMTIME from TBASE3,
  read at restart time) only clears group 3 and does not repeat the
  throttle-up. src/lm/burns translates this as flown; exec's KILLTASK
  (first match only, TASKOVER in its place) and 3.3 table entry agree.

## Lessons
