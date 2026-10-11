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
- TODO 5b sim stub main, make check
- TODO status-handling open items (N85 jobs, CLEANDSP, 1301/1302)
- TODO 6 note, MODULE_STATUS
