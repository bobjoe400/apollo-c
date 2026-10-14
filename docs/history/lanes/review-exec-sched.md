# review-exec-sched -- lane history

Process diary of the review-exec-sched lane, moved verbatim out of [docs/modules/review-exec-sched.md](../../modules/review-exec-sched.md), which keeps the reader-facing explanation. Note dates: 2026-10-10 to 2026-10-13 (first and last commit touching the note).

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
- DONE style debt (1917 findings at start, 0 now in src/exec,
  include/agc/exec, tests/exec). Scratch tools (review-exec-sched/):
  prolog.py, annot.py, fp.py, defc.py.
- RESUME 2 (2026-10-11, after status-handling-2), in order:
  - DONE B4 WAIT2 overflow branch (waitlist.c wait2, R10) + test
    test_waitlist_ruptagn_insert (fails on the old code: 3 checks)
  - DONE test_phase_codes: status-handling-2 removed the group argument
    and the mismatched-code negative test; no test in tests/exec uses
    the assertion-report count (printed only)
  - DONE R12 (review-dap-lm): LM STARTSB1 calls agc_lm_dap_restart()
    (fresh_start.c startsb1); test_poodoo_lm checks ACCSOKAY cleared
  - DONE RDOT noun (review-guid-landing R2): LM AGC_DATA_RDOT reads
    agc_guid_state()->lm.hdotdisp (RDOT = HDOTDISP, L ERASABLE p.1300 line)
  - DONE EVA lead SPECTEST (executive.c): invariant confirmed (core set 0
    free => NEWJOB = -0; argument in the spectest header); the guard is
    the AGC's own CCSHOLE and stays. Residual: a job JOBWAKEing its own
    sleep LOC before returning would hit it (no job does).
  - DONE EVA lead JOBWAKE(NONE): precondition removed, NONE is NOT_FOUND
    as in the rope (test). Tautological postconditions removed
    (job_wake, job_wake_as, agc_exec_init -> static assert), prio_word
    sentinel guard removed.
  - DONE EVA lead restarts.c start_job/start_long (R11): audit of every
    B/C PHASCHNG (scratch phcodes.py): 45 plain agc_phase_change calls
    with type C job codes, A and D clear (GRP2PC 04022 family, 04026,
    04024 symbols) never stored PHSNAMEn: the AGC takes the return
    address. phase_table.c store_return_point: such a call from a job
    stores the running job segment (DEVIATION: reruns the segment from
    its top). PHSPRDTn presets missing in other lanes -> Requests. The
    start_job/start_long guards stay (a real unset point -> BAD_ARG).
  - DONE renames: AGC_JOB_REFAZE10 / agc_job_refaze10 ->
    AGC_JOB_REESTABLISH_ENTRY_SEQUENCER / agc_job_reestablish_entry_sequencer
    (registry.tsv + gen_registry, spatch on csm/entry, NAMING.md row);
    agc_phase_set_prdt was already added as agc_phase_set_priority (R8)
  - DONE (2) status-handling-2 diff check. Confirmed: waitlist.c,
    restart_tables.c, nouns.c, internal.h, erasable.c, interrupts.c,
    executive.c (deletions are by-construction facts). Repaired:
    alarm.c (PROGLARM prolog left inside store_alarm; tautological
    enema/restarts_run asserts, restarts_run removed (write-only);
    service_restart no longer filters ENEMA statuses into BAD_ARG;
    gopoodoo returned BAD_ARG with INHINT held and turned a SERVIDLE
    status into an assertion; poodoo single return); KLEANs (MR.KLEAN,
    P00KLEAN, V37KLEAN) cannot fail -> void (exec.h, callers in exec).
    phase_table.c confirmed (BELOW1-4 prolog left in phaschng_common
    and a silent-skip assert removed; phase_var_get -> void). restarts.c
    rewritten: ITSAVAR helper back (restart_variable), single returns,
    boxed prologs (style 0); restart_all kept its semantics.
    fresh_start.c repaired: ENEMA's TVCEXEC WAITLIST status was turned
    into BAD_ARG with INHINT held (now propagated); GOPROG3 dropped a
    MMDSPLAY BAILOUT (now returned); REDOCTR < 65535 guards and
    BAD_ARG status filters removed; single returns (lightset,
    phase_tables_ok, fresh_enema merged LM/CSM tails); style 0.
    dispatch.c: marker moved into the prolog (gen_registry.py,
    CODING_STANDARD rule 4 wording), style 0.
    interrupts.c: duplicate inhibited/time3/depth tautologies removed
    (rupt_exit keeps the one real check), INHINT cap / RELINT assert
    were silent skips (RELINT when enabled is a no-op as in the AGC),
    T5IDLOC guards by construction; boxed prologs, style 0.
    change_program/* and exec.h diff confirmed (ctx-less jobs via the
    registry, CHECKTAB inlined, up-calls void).
  - (4) style: DONE 0 findings: restarts.c, fresh_start.c, interrupts.c,
    erasable.c, restart_tables.c, dispatch.c, alarm.c, waitlist.c,
    executive.c, phase_table.c, nouns.c (agc_dispatch_noun split:
    noun_by_function, noun_p11_readout, noun_latlong; same lookup order).
    change_program/{csm,lm,idle,sequence}.c (cleanup_lm -> void,
    rendezvous_lm -> bool like CSM; RELDSP status no longer filtered;
    by-construction vehicle guards removed), internal.h, exec.h,
    tests/exec/exec.c (file prolog, main first with prototypes, test
    names without alarm numbers). The whole lane is at 0 findings.
  - DONE (3) DMP/DMPR, shift rounding, word-level API: exec has no
    flight arithmetic (integer cs/priority words only): 0 sites.
  - DONE gates: make, make test (all binaries 0 failures), make
    scenarios (7/7, 0 errors/restarts), make check (check-rules 0 hard,
    format-check clean), one_liners PLAIN 0.

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
| R10 | src/exec/waitlist.c wait2 | WAIT2 CS TIME3 / AD BIT8 / CCS A (C p.1226, L p.1122) | sim B4: DEVIATION "TIME3 overflow already happened cannot arise" was wrong; inside a RUPTAGN chain TIME3 = T - T2 (0 for two tasks due together) and the C code pushed the next chained task back 163.84 s (P30 CLOKTASK starved MONREQ) | both CCS paths translated (C(TIME3) < 200: TD - T1 + 1 = DT + C(TIME3) + 1; = 200: -0 -> AD Q); test added |
| R11 | src/exec/phase_table.c agc_phase_change | GETNEWNM, A clear: restart point = return address (C p.1411, L p.1300) | plain PHASCHNG of a type B/C job code (GRP2PC OCT 04022 etc., 45 sites) kept a PHSNAMEn nobody stored, so a restart of that group started AGC_JOB_NONE (start_job BAD_ARG; EVA lead restarts.c:54/72) | store_return_point: from a running job, PHSNAMEn = the running segment (DEVIATION: reruns it from its top); test_restart_return_point |
| R13 | src/exec/alarm.c, fresh_start.c, interrupts.c (status-handling-2 diff) | ENEMA / GOPROG2 (C p.188), GOPROG3 MMDSPLAY, GOPOODOO (L p.1383) | ENEMA turned the TVCEXEC WAITLIST status into BAD_ARG and returned with INHINT held; GOPOODOO the same after the KLEANs; GOPROG3 dropped a MMDSPLAY BAILOUT; service_restart / goprog / rupt filters mapped real statuses to BAD_ARG; leftover helper prologs inside function bodies | statuses propagated, single returns, KLEANs void (cannot fail), filters removed |
| R12 | src/exec/executive.c agc_exec_job_wake | JOBWAKE2 CS ONE / TS LOCCTR (C p.1216, L p.1111) | JOBWAKE(AGC_JOB_NONE) was BAD_ARG; in the rope a LOC nobody sleeps at is "not found" (EVA lead executive.c:598) | precondition removed; test |

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

## Requests
* guid-entry (REENTRY_CONTROL p.761, src/csm/entry/reentry.c ~2026): the
  comment says "no exec API for PHSPRDT5 alone"; there is one now:
  `agc_phase_set_priority(5, AGC_ENTRY_PRIO16)` (CA PRIO16 / TS PHSPRDT5),
  before the PHASCHNG OCT 00474; the PRIOCHNG after it (CA PRIO16 / TC
  PRIOCHNG) is the AGC's and stays. Without the preset, a group 5
  restart of the type B OCT 10035 phase uses whatever PHSPRDT5 the last
  D-set phase stored.
* prog-rendz (P3 in review-prog-rendz.md): the R22LEM3 / P23 / R22
  `TS PHSPRDT2` presets: call `agc_phase_set_priority(2, prio)` (the
  entry the request called agc_phase_set_prdt).
* nav-integ / prog-rendz: the 45 plain `agc_phase_change(04022)`-family
  calls (GRP2PC etc.) now restart the job segment that made the change
  (R11). Where that segment is not safe to rerun from its top, name the
  continuation job with `agc_phase_change_var`.
* sim: B4 fixed (R10); scenario e can `expect` V16N45 (MONREQ) instead
  of `xexpect`.

## Lessons
* A guard deleted as "by construction" must be checked against every
  caller, including restart jobs: CANV37 and V37XEQ run again after a
  restart with whatever MINDEX/MMNUMBER hold, so their mindex checks stay.
* Inlining a helper during a mechanical pass leaves its prolog comment
  inside the caller's body (PROGLARM, BELOW1-4): grep for `/****` lines
  indented inside functions after such a pass.
* "Return the status" also means "do not filter it": an
  `if (!AGC_ASSERT(st == OK || st == BAILOUT)) return BAD_ARG;` after a
  call turns every other real status into BAD_ARG and drops the cause.
* tools/check_style.py's declaration regex reads `long...`/`int...`
  identifiers (`longtime`, `interp_result`) as types; rename the variable
  rather than fight the tool.

