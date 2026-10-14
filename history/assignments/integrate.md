# Assignment: integrate   (tier: translation -- judgment across lanes)

Runs after status-handling. You own the whole tree. Goal: definition-of-done
items 2, 4, 5 in docs/PLAN.md: `make`, `make test`, `make check` all clean,
one flat job/task registry, no scaffolding files, layering respected.
Read docs/history/WORKLOG.md ("Integration:" lines) and every docs/modules/*.md
"Requested core additions" / "Requested jobs-tasks" / "Open questions"
section first; they are the worklist. Keep a Progress section in
docs/history/waves/integrate.md from the start.

1. **Job/task registry.** Collect every provisional id from all
   `pending.h`, `ext.h`, `*_pending.h`, `sysmisc_ext.h` into
   include/agc/exec/jobs.h as one enum each for jobs and tasks (names per
   docs/NAMING.md; where two lanes defined the same concept with different
   values, keep one). Fill the two switches in src/exec/dispatch.c with
   every job/task function. Then delete every pending/ext/shim header and
   fix includers.
2. **Dissolve adapters.** Delete src/programs/navigation/glue.c and
   src/guidance/guid_nav_glue.c: callers call the owning module's public
   API directly; the earth/moon body-selection helper moves into
   nav/conic as a named function; THISPREC/CSMPREC/LEMPREC/MIDTOAV1/2/
   AVETOMID exist once, in nav/integration. Fold `*_internal.h` under
   include/agc/ into the real public headers (programs/alignment.h,
   csm/entry.h, io/service.h -- create service.h) and delete them.
3. **Missing accessors.** Implement every "Requested core additions"
   accessor in src/core (state/erasable) or the owning module: TVCPITCH/
   TVCYAW, TEPHEM pad loads (AZO AXO -AYO 504LM TIMEMO VECOEM RESO VESO
   OMEGAES), TRKMKCNT, THRUST/ALTM/CDUTCMD/CDUSCMD counters, hand-controller
   counters, read/write_ecadr, optmodes/radmodes (RADMODES is ONE storage
   word = FLGWRD12; remove sys-misc's copy), samptime/option1/sfail,
   t3tot4/rted1/return_to_earth_range, LAT/LONG/ALT, RNRAD, KILLTASK,
   M11..M32 public declarations, and the 182 noun data ids that io-nouns
   left UNIMPLEMENTED (wire each to its accessor; list any that truly have
   no owner). Add `agc_exec_job_wake_as(loc,new_loc)` and a TBASE2 setter
   to exec. Unify the two TEPHEM sources (nav-aux note).
4. **Cross-lane wiring.** V37 handoff (exec change_program_idle: CSM
   R00TOP00, LM SETUP70/71 via the burns lanes); SERVIDLE, SETCOARS,
   SETMAXDB/SETMINDB up-calls; interrupt handlers T4RUPT/UPRUPT/DOWNRUPT/
   radar registered in src/exec/interrupts.c; V57/V54/V89 dispatched by
   vehicle; T4RUPT calls agc_lm_dap_rcsmon every 480 ms (verify); each
   lane's fresh-start reset entry point called from fresh_start.c;
   restart-table delay values P40TMP SAVET-30 ZOOMTIME S61DT CM/GYMDT.
   Fix the upward dependency programs/navigation/noun_data.c ->
   lm/radar/radar.h (move the LM noun resolution into lm/, or expose the
   data through the public header). Remove the duplicate CALCSMSC (keep
   imu's). Remove AGC_JOB_GOPROG placeholder if unused.
5. **Build.** `make` (lib + sim stub main if src/sim is empty: create
   src/sim/main.c that fresh-starts a vehicle chosen by argv and runs the
   executive N ticks -- the real harness is a later lane), `make test`
   through the real library (remove every LANDING_TEST_FAKE_THRUST-style
   test-local stand-in), `make check` (check_rules 0 hard; format-check
   clean; `make style` is allowed to report for now), oracle tests
   (tests/oracle/*) pass. Add a debug library target so tests see
   AGC_HOST_DEBUG asserts (sys-misc note).
6. **Report** <= 15 lines; docs/history/waves/integrate.md has the decisions,
   anything deferred, and Lessons. Update docs/MODULE_STATUS.md.
