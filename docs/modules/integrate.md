# integrate -- wave 3: one registry, no scaffolding, make/test/check clean

## Scope
docs/agents/integrate.md items 1-6 (definition of done items 2, 4, 5), plus
the two GODSPRET continuation jobs of status-handling and its open items
(CLEANDSP continuation; ALARM 1301 / ABORT 1302 raised by the callers of
sqrt/asin/acos).

## Progress
- DONE 1a registry: tools/registry.tsv (869 ids: 709 jobs, 160 tasks) ->
  tools/gen_registry.py -> include/agc/exec/jobs.h + src/exec/dispatch.c
  (generated; chunked switch by enum range, <= 60 lines each). All 957
  provisional #defines deleted; 40 duplicate/placeholder ids merged via
  docs/RENAMES.md "integrate" section + tools/rename.cocci; 12 dead ids
  dropped. agc_dispatch_rupt moved to src/exec/interrupts.c. New
  handlers: prelaunch ALLOOP1/SPECSTS task stubs, CSM REDO5.5/CHEKAVEG,
  LM REDO5.5, LM REDO2.17, LM R31CALL task; P31 split _csm/_lm (was a
  duplicate definition).
- DONE 1b all 15 pending/ext shim headers deleted; declarations moved to
  their owners' public headers (exec.h: KILLTASK, TBASE setter, JOBWAKE-as,
  VAC claim, PRIOnn; io/service.h (was service_internal.h): extended verbs;
  guidance.h / lm/landing.h / csm/entry.h: alarm codes, priorities).
- DONE 2a guid_nav_glue.c deleted: callers use agc_integ_thisprec/othprec
  (new, THISPREC/OTHPREC), csmprec/lemprec, midtoav1/2, avetomid directly;
  agc_guid_store_midtoav (servicer_common.c) stores RN1 VN1 PIPTIME1 RTX1/2;
  INTSTALL resume default moved to guid_state.c.
- DONE 2b alignment_internal.h -> programs/alignment.h, entry_internal.h ->
  csm/entry.h (folded, deleted).
- DONE 2c programs/navigation/glue.c deleted: pass-throughs replaced by the
  owners' calls (spatch: agc_conic_timethet/timerad/apsides,
  agc_exec_gotopooh, agc_dsky_endext, agc_svc_delayjob, agc_rtb_loadtime,
  agc_phase_change, agc_erase_rls; rz_alarm -> agc_alarm); the RTX1 body
  selector is agc_conic_body_of_x1 (nav/conic); AGC-routine helpers moved
  into the lane (COMPTGO, flash/priority displays, DSPTEM views ->
  programs/displays/display.c; THISPREC/OTHPREC kinds, INTEGRVS set-up,
  type C PHASCHNG -> programs/navigation/common.c; LAMBERT/SETITCTR choice
  -> targeting/initvel.c, static).
- DONE 3a core accessors: src/core/counters.c (RHCCTR, RNRAD, CDUTCMD/
  CDUSCMD = TVCYAW/TVCPITCH, THRUST, ALTM, TEMX..Z, PIPASR +3),
  display_erasables.c (OPTION1-3, SAMPTIME, SFAIL, MARKDOWN, MARK2DWN,
  TRKMKCNT (moved from programs), T3TOT4, RR-AZ/ELEV), pad_loads.c (AOTAZ,
  AOTEL, CDUCHKWD, RTED1, P37RANGE, TLAND), ecadr.c (read/write by ECADR;
  flagwords mapped). LAT/LONG/ALT in nav/auxiliary; UN/GEOMSGN in
  nav/conic (LAMBERT); DSPTEM1/OPTIONX/DSPTEMX views in pinball; TEPHEM and
  RLS one storage (core). PREREAD1 public; P11 EARTHR-at-liftoff added.
  Library links with no undefined or duplicate symbol.
- DONE 3b noun data: every noun component of both tables now resolves
  (scratch scan) except the specials pinball handles (N01-03, 10, 15) and
  N08 ALMCADR (+1 +2), LM N70/N71 AOTCODE +1/+2 (no owner in C). Core
  erasables in io/dsky/nouns/data.c; everything above io through the new
  agc_dispatch_noun (src/exec/nouns.c, the dispatch.c layering exception)
  into the owners' public state blocks and noun functions (rendezvous,
  new agc_lm_rendz_noun_read / agc_csm_rendz_noun_read (the vehicle parts
  moved out of programs/navigation/noun_data.c, which no longer includes
  lm/radar/radar.h), agc_return_to_earth_noun, nav LAT/LONG/ALT, P11).
- DONE 4a V37 complete (src/exec/change_program/{v37,csm,lm,idle}.c):
  FCADRMM1/PREMM1/DNLADMM1 tables of both ropes, CANV37 (INTSTALL, outbits,
  flags), POOH, NOUVEAU/RENDV00 (rendezvous groups, STATQUO), V37XEQ as
  the group 4 type C restart job after ENEMA (deviation for TEMPFLSH),
  CSM R00TOP00 (+DELAYJOB, jets off), LM SETUP70/71, V37RET (OUTROUTE).
  Scratch run: V37E30E -> PROG 30 V06N33 flashing (both vehicles); P52
  without IMU -> alarm 210 + V37 flash; V37E99E -> OPR ERR.
- DONE 4b interrupts: T4RUPT, UPRUPT (INLINK counter), DOWNRUPT, RADAR
  (CSM VHFREAD / LM RADAREAD) registered in agc_dispatch_rupt; T4JUMP LM
  slot 0 now RCSMONIT then RRAUTCHK (480 ms), as T4RUPT_PROGRAM p.159.
- DONE 4c V89 by vehicle (registry), V54/V57 per vehicle verb files.
- DONE 4d RADMODES = FLGWRD12 (sys-misc static removed); CDUTCMD/CDUSCMD
  one storage (sys-misc optics copy removed); TEPHEM/RLS one storage.
- PARTIAL 4 cross-lane wiring: exec_up_servidle -> agc_lm_servidle
  (STRTIDLE; no GOTOPOOH after it, the 4.33 restart does it),
  exec_up_setcoars -> agc_imu_setcoars, INITSUB restores the deadband
  (one SETMINDB/SETMAXDB, csm/dap; the burns copy removed).
- DONE 5a make test green through the real library: 21/21 binaries link
  build/libagc_debug.a (new lib-debug target, -DAGC_HOST_DEBUG; AGC_ASSERT
  debug form is now `(report, false)` so flow analysis matches release).
  Archive rebuilt from scratch each time (duplicate member base names).
  Stand-ins removed (landing THRUST, exec rupt/up-call doubles); exec test
  keeps dispatch_job/task doubles (documented). Stale test expectations
  fixed (navaux a1 overflowed 2^5; integration JOBSLEEP is void now; dsky
  V21N01 at 12345 -> ECADR image covers 0-77777). nav GRP2PC assertion
  fixed (type C phase is 0402, not 2).
- DONE 4e fresh-start / power-on: DOFSTART/STARTSB2 already reset every
  erasable the AGC resets through exec_up_dofstart/exec_up_startsb2. The
  lanes' "reset for tests" blocks (alignment RAM, rendezvous, LM landing/
  ascent, entry lexicon) are cleared by the new host agc_power_on(v)
  (idle.c, exec.h), not by DOFSTART: the AGC keeps erasable through a
  fresh start. CALCSMSC exists once (imu); agc_align_calcsmsc is only the
  CDUTRIG struct -> vectors call. AGC_JOB_GOPROG gone. Restart-table
  delays (P40TMP, SAVET-30, ZOOMTIME, S61DT, CM/GYMDT) read the owners.
- DONE 5b src/sim/main.c: agc_sim [csm|lm] [cs] -> agc_power_on,
  DOFSTART, TIME1/TIME3 tick + executive (P00, no alarms, both vehicles).
  `make`, `make test` (21 binaries, libagc_debug.a), `make check`
  (0 hard, format-check clean, style report only) all exit 0; oracle
  tests (conic 299, core_numeric) pass.
- DONE status-handling items: N85 GODSPRET continuations are
  AGC_JOB_RCS_BURN_VG_DISPLAYED / AGC_JOB_REDO6_7_VG_DISPLAYED in the
  registry; CLEANDSP without continuation ends the job when woken
  (documented deviation in display_if.c); ALARM 1301 / ABORT 1302: core
  latches (agc_interp_take_alarm, now also TP SQRT), exec raises it when
  the job/task returns (exec_service_interp_alarm). Test in tests/exec.
- DONE removed agc_conic_csmconic/lemconic (unused second copy of the
  integration's CONIC path; its SURFFLAG branch was a stub).
- DONE 6 note, MODULE_STATUS.

## Decisions
* One registry: tools/registry.tsv is the source; tools/gen_registry.py
  writes include/agc/exec/jobs.h and src/exec/dispatch.c. Add a job by
  adding a row and re-running the generator, never by hand.
* Layering exceptions in exec are exactly three files: dispatch.c,
  nouns.c (agc_dispatch_noun) and change_program/idle.c (exec_up_* and
  agc_power_on). Everything else in src/exec calls only core.
* Interpreter alarms raised at job/task return, not at the instruction
  (DEVIATION in alarm.c): core cannot call exec.
* agc_power_on is a host function (no AGC counterpart); DOFSTART keeps
  the AGC's semantics.

## Deferred (lane stubs, not integration items)
* DELRSPL (P11 V82 splash error, AUGEKUGL tables, CM pp.643-647): the
  V82 job does INTWAKE itself when it returns UNIMPLEMENTED.
* P06 standby/POSTAND (imu/mode.c), CSM prelaunch IMUTEST etc., LM
  systems test, V27 fixed-memory display (rope not modelled).
* Noun components with no C owner: N08 ALMCADR +1/+2, LM N70/N71 AOTCODE
  +1/+2.
* Timing deviations kept: PRIOCHNG cannot suspend a running C job.
* Sim stub has no T4RUPT/T5RUPT/T6RUPT clock or DSKY; the sim lane owns it.

## Lessons
* Run `make format` before every `make check`: Uncrustify reflows lines
  edited by scripts (spatch output especially).
* `ar r` replaces archive members by base name; with several clock.o /
  state.o the archive silently lost objects. Rebuild the archive from
  scratch (rm then ar rcs).
* A test-local stand-in that shadows a library symbol hides link-order
  bugs; link tests against the real library and keep only documented
  doubles (exec's dispatch_job/task).
* AGC_ASSERT's debug form must have the same value as the release form
  (`(report, false)`), or flow analysis (-Wmaybe-uninitialized) differs
  between the library and the debug library.
* Before adding a "missing" routine, grep for a second copy: several
  lanes had translated the same AGC routine (CSMCONIC/LEMCONIC, SETMINDB,
  CDUTCMD storage, RADMODES); keep the owner's and delete the rest.
* Lane "reset" functions are power-on helpers, not DOFSTART: the AGC
  never clears program erasable on a fresh start.
