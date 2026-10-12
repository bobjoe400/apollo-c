# Lane-specific review items (routed from oracle findings, open questions, worklog)

- core-numeric: UNIT precision at B27 for short vectors (R63 ranges); oracle F1-F6; stale-DVSIGN deviation on V/SC and UNIT overflow (model or justify). **Word-level MPAC API** (requested by nav-aux, imu, prog-rendz, guid-entry): a public type/ops carrying DP/TP values as the AGC's sign-disagreeing word pairs, with DAS/DMP/DDV/SQRT/ASIN/ACOS on it, so R1A in GETX, GCOMP, 1-cos A and the entry L/D residuals can be bit-exact; keep agc_dp_t for the common path. Also: SQRT keeping MPAC+2; TP-input ASIN/ACOS; DDV and SR on stored word pairs (review-guid-landing R1); `agc_dp_sign` AGC SIGN semantics confirmed.
- exec-sched: B4 from the sim lane (docs/modules/sim.md "Bugs found"): src/exec/waitlist.c wait2() -- the DEVIATION claiming the TIME3-overflow-already-happened branch (C(TIME3) < 200 octal) cannot arise is wrong; it arises inside a RUPTAGN chain (two tasks due in the same cs), WAIT2 (WAITLIST C p.1225 / L p.1121) forms TD - T1 + 1 from 200 - C(TIME3); the C code pushes the second task 163.84 s away (P30: CLOKTASK starves MONREQ, V16N45 stays blank). Fix per the rope + a test in tests/exec/exec.c; then flip the scenario's xexpect. Also: tests/exec/exec.c test_phase_codes passes a mismatched phase code (040064 to group 3) as a negative test and counts the silent assertion report; after status-handling-2 makes agc_phase_change return agc_status_t, the test must CHECK the returned status instead (no assertion-report counting as a pass criterion anywhere in tests). R12 from review-dap-lm: on LM software restart startsb1() in src/exec/fresh_start.c must call agc_lm_dap_restart() (clears RCSFLAGS bit 13 / ACCSOKAY, reruns 1/ACCS, resets t5adr) -- otherwise STARTDAP asserts. src/exec/nouns.c:155 RDOT noun must read HDOTDISP (agc_guid_state()->lm.hdotdisp), not the unwritten agc_lm_asc()->rdot (review-guid-landing R2); rename AGC_JOB_REFAZE10 (review-guid-entry); agc_phase_set_prdt (review-prog-rendz). Deferred-restart vs immediate ENEMA (observable differences); V37 handoff completeness; activity-light mask fix verified; P23/R22LEM restart-priority deviation; ZOOM/P40ZOOMA group-3 question.
- state-flags: unconfirmed CSM channel bits; flag-table typo list; RADMODES one word; noun write-path gaps (VHFCNT, FUNNYDSP, negative TTOGO/TIG); include/agc/core/assert.h name shadows <assert.h> (guide 3.4): rename if clean.
- nav-conic: -0 / mixed-sign gap on circular orbits (model now that oracle inputs keep -0?); assembled TWEEKIT/GEOMSGN; LEMCONIC SURFFLAG ownership.
- nav-integ: oracle nav-integ findings; TEPHEM dual source; X2-after-origin-change deviation; bounded state machine vs job sleep.
- nav-aux: oracle nav-aux KNOWNs; navaux unit test 1-2 failing checks; pad-load accessor unification.
- io-dsky (io-pinball + io-nouns): F15 OPDEGOUT -0 treated as CCSHOLE (scale.c:566; AGC shows +89.997); the 64 KNOWN dsky oracle mismatches -> fix causes, flip to hard; dsky unit test 4 failures; noun write-path gaps; CLEANDSP-without-continuation; V27 stub justification.
- imu: oracle imu findings; translate the translatable stubs (P06, POSTAND, 11 Comanche prelaunch entries incl. ESTIMS); MODECADR -0 as -2 deviation.
- guid-powered: F7 CM J2 constants 1/4 of assembled (servicer_common.c:47-49); F8 burn_steering.c:330 raw RTIG vs UNIT (S40.2,3); F9 at TDISPSET; DELRSPL stub (translate); ZOOM/P40ZOOMA with exec review; grep for other octal-written table nouns.
- guid-landing: oracle landing findings; F9 at THROTTLE (sign-disagreeing DAS high word used alone: match AGC); LANDING_TEST_FAKE_THRUST gone; R10/R11 abort-monitor naming.
- guid-entry (+p37): oracle entry findings; F14 RTE INVC115 (return_to_earth/conic.c:147) convergence; five re-split entry dispatchers map to "* START" sections; RTARG push-list and AUGEKUGL deviations.
- dap-lm: oracle dap-lm findings; docked 1/ACCS CCS COEFCTR -> TORQCONS (resolve by running the rope in the oracle); BIGAOS 2*DB vs 1*DB (same); F9 at PITCHDAP.
- dap-csm: oracle dap-csm findings; digit-suffixed names (agc_task_noroll1, tvcinit1, csm_tvc_init4...) spelled out per NAMING rule 2; ROLLSET overflow-limiting deviation.
- prog-rendz: F13 CDH DIFFALT 62 m (trace); targeting findings; P23/R22LEM restart priority with exec review; programs/navigation/noun_data.c -> lm/radar upward dependency (fix layering).
- prog-align: UNIT-at-B27 for R63 with core review (call-site scaling per AGC); alignment tests at realistic ranges; N70/N71 AOTCODE orphan components.
- sys-misc: LM P07 REDO stub (document what the rope lacks, translate the fixed-memory part fully); N08 ALMCADR orphans; RADMODES one word; sysmisc test free of scratch stand-ins.

## ALL remaining lanes (added 2026-10-11 after review-guid-entry finding E1)
- **DMP vs DMPR audit.** The original translation guide mapped DMP to the
  rounded multiply; core-numeric corrected it later (DMP = agc_dp_mul_trunc
  / TP product, DMPR = agc_dp_mul). Entry had 92 wrong sites and the error
  had been blamed on yaAGC. For every multiply in your lane, check the AGC
  listing: DMP -> truncating (keep the TP accumulator where the AGC shifts,
  square-roots or arc-cosines the product), DMPR -> rounded. Fix and
  re-tighten your oracle tolerances. Report the count changed.
- **Shift rounding.** Same check for SR/SRR and VSR (truncate vs round) and
  for overflow stores: wrap modulo 1 where the AGC does, do not saturate.

- sim: nothing calls agc_lm_dap_time6_count / agc_lm_dap_t6_pending, so TIME6 never counts and no T6RUPT ends a jet firing in scenarios (review-dap-lm request); wire TIME6 into the clock driver.

## Word-level API adoption (review-core-numeric, 2026-10-11)
Lanes with documented word-split residuals move those sites onto
agc_dp_words_t / agc_mpac_* (docs/TRANSLATION_GUIDE.md section 1) and
tighten the oracle tolerance to 0: nav-conic (R1A in GETX/PARAM/LAMBERT,
COGA, X, KEPC2), imu (DELCOMP 1-cos A; replace word_pair_t), guid-landing
(TBUP/TGO; replace agc_lm_words_t), guid-entry (A0/V1/LEWD words -> L/D),
nav-aux (NODI COS, SR1 ASIN), prog-align (word_forms.c), dap-csm / dap-lm
(their pair types). Lanes already reviewed get this in the follow-up pass.

- imu follow-up (from review-dap-csm): agc_imu_hw_set_cducmd rejects counts beyond +/-192 but a full needle swing writes up to +/-768; widen to the full SP range.

## EVA bug leads (status-handling-2, make prove 2026-10-11; run artefacts in build/prove/*.csv, build/prove/eva_verdicts.tsv, build/prove/tree (snapshot), tools/prove/{summary.txt,progress.log,runs/*.log}; positions are the snapshot's = the tree of that date)
EVA is the project-end cleanup step; these are leads for the owning reviews, not fixes.
- exec-sched: INVALID executive.c:325 `!AGC_ASSERT(newjob > 0)` in SPECTEST (exec_exec, imu, io_dsky, lm_dap, oracle_entry, programs_rendezvous): EVA reaches NOVAC2 -> SPECTEST with core set 0 free and NEWJOB = +0. The AGC's CCS NEWJOB sends +0 to CCSHOLE; the C relies on "core set 0 free => NEWJOB = -0", which EVA does not track (likely imprecision); confirm the invariant holds after the deferred JOBSLEEP EJSCAN (C runs EJSCAN when the body returns, the AGC at once).
- exec-sched / nav-integ: INVALID executive.c:598 `!AGC_ASSERT(job != AGC_JOB_NONE)` in JOBWAKE (oracle_targeting, programs_rendezvous, nav_integration, exec, imu, io_dsky), reached from WAKE <- INTWAKE <- agc_integ_run in the CSI/TPI integrations: a sleeper slot holding AGC_JOB_NONE. The AGC's JOBWAKE of a LOC nobody sleeps at is NOT_FOUND (LOCCTR -1), not an error; check INTSTALL never records a NONE resume, or make JOBWAKE(NONE) NOT_FOUND.
- exec-sched (sim, partial run): restarts.c:54/72 start_job/start_long guards (PRDT 0 / task NONE) reached from a restart of a group whose variable restart point was never stored (type B/C code via the fixed agc_phase_change): check every B/C PHASCHNG stores its point first.
- guid-landing: INVALID lm/ascent.c:509 (oracle_landing): WAITLIST for ENGOFFDT fails on every EVA path. Fixed the C side (as_engoff now returns the WAITLIST status into the ascent job, BAILOUT 1203); still check ENGOFFDT <= 16250 cs (LONGCALL range) against ASCENT's TGO window.
- sys-misc / io-dsky (sim, partial run): status guards EVA judged false in some sim context: display_if.c:329 (CONT NONE or RESTREG prio), scale.c:793/829 (-0 with nonzero SP), scale.c:861 (SFOUT < 0), monitor.c:309/344 (MONREFRS prio/job, delay 0): likely imprecision of the unbounded sim analysis; recheck under the bounded sim entry at project end.
- dap-csm: dapdata.c:167 IXX/IAVG < 0 in EVA's sim context: the CSM mass/inertia pad words (V48 DAP data) can be loaded negative by the crew; decide the AGC's behaviour (it does not check) and the entry check.
- imu: compensation.c:337 GCOMP word pair read-back; guid-landing: lunar_landing.c:319 (st after a display/waitlist call), throttle.c:173 PIFPSET range: sim-context alarms, recheck at project end.
- prog-rendz: targeting status guards `!AGC_ASSERT(st == AGC_STATUS_OK)` (tpi.c:224, tpi_tpm_targeting.c:103/619, target_delta_v.c:228, stable_orbit.c:100/512/845, csi_cdh_targeting.c:543/918) return BAD_ARG for any callee failure: propagate the callee's status instead (BAILOUT from FINDVAC/WAITLIST must reach the job). tpi.c:714 / target_delta_v.c:96 (-DPPOSMAX component), lambert_aimpoint_guidance_lm.c:104 (HPER > MAXNM), csi_cdh_targeting.c:386 (alarm code range): recheck. EVA's "uninitialised" reads: initvel.c:235-237 and 359-360 were real (INITVEL copied LAMBERT's outputs and the CSTH/SNTH debris before testing the status, so a LAMBERT entry failure read an unwritten struct): fixed by status-handling-2 (copies only when LAMBERT ran); 276/278 are in iv_finish, reached only on OK. o.x2 (target_delta_v.c:225,276; lambert_aimpoint_guidance_lm.c:154) and o.t (csi_cdh_targeting.c:310) are the non-relational domain's: rz_integ/rz_integrvs copy the whole ctx->out on OK, agc_conic_timerad clears its out first, and the callers return on non-OK: no defect.
- nav-integ / core-numeric: "signed overflow" at orbital_integration.c:430 (XSUM = XS + ASCALE + N4 + N1, all NORM counts and table shifts < 100) and divide.c:97 (DV remainder + MP high word, both < 2^14 with the normalised divisor GENDDV requires): EVA does not carry those bounds through the out-parameters; no host UB.
- sim (sim, partial run): clock.c:207/245/251/330, socket.c:138: harness guards reached in EVA's unbounded sim context; the sim entry needs a bounded proof main at project end (agc_sim's own main timed out at 60 min).

## verify-collapse findings (2026-10-11; nothing shown wrong in the 11 reviewed lanes; note docs/modules/verify-collapse.md)
- ALL remaining lanes + follow-up-done-lanes: `AGC_ASSERT(st == OK) -> return BAD_ARG` sites replace a callee's FULL/BAILOUT/POODOO status with BAD_ARG (62 in the reviewed lanes, predating the pass). Propagate the callee's status; an assertion there is not a guard.
- follow-up-done-lanes: tests must not compare assertion-report counts: tests/nav/navaux.c:502, tests/lm/dap.c:444 (check the returned status instead).
- exec-sched (change_program/idle.c) + prog-rendz: src/programs/navigation/common.c:441 agc_rendz_reset drops agc_rendz_selectmu's status in an empty if-branch (pre-existing); propagate.
- io-dsky: src/io/dsky/display_if.c:1063 agc_dsky_blanksub result stored, never read (its BUSY case cannot happen there): make the call's result meaningful or the callee void.
- imu: src/imu/maneuver.c:874 leftover assertion guard with no return; branch can never run: delete or make fallible.
- document lane: 82 function headers still say "BAD_ARG for a NULL ctx" on functions without ctx; inline_one.py pasted 77 helper header blocks into caller bodies: rewrite as section comments.
- tools/prove/manual.tsv: the "why" column is one boilerplate text per directory; it records the decision, not the evidence (lesson for the end-of-project EVA pass: per-row evidence or none).

## From review-exec-sched (2026-10-11; note docs/modules/review-exec-sched.md)
- follow-up-done-lanes (guid-entry): preset PHSPRDT5 with agc_phase_set_priority(5, PRIO16) in reentry.c where the AGC does `TS PHSPRDT5`.
- follow-up-done-lanes (prog-rendz): add the PHSPRDT2 presets (R22LEM3 PRIO26, P23 PRIO13, CSM R22 PRIO26) via agc_phase_set_priority.
- nav-integ + prog-rendz: plain type-C PHASCHNG calls (GRP2PC 04022 family) made from a running job now store the current job segment as the restart point and a group-2 restart reruns that segment from its top (DEVIATION in exec): check each such segment in your files is safe to rerun.

## From review-sys-misc checkpoint (2026-10-11; note docs/modules/review-sys-misc.md)
- io-dsky: V91 SHOW-BANKSUM needs the display to carry the MPAC+2 address the AGC passes; the noun/display path cannot yet, so V91 cannot show bank sums.
- sim: load the real rope image (Comanche055 / Luminary099 binary from tools/oracle) as fixed memory so the idle self-check (ROPECHK) and SHOW-BANKSUM run on real bank sums; data only, no model.

## From review-prog-align checkpoint (2026-10-11; note docs/modules/review-prog-align.md)
- follow-up-done-lanes (radar, navigation, displays, io): agc_csm_sxtnb, agc_lm_rrnb, agc_lm_rrangles cannot fail any more; make them void and drop the callers' status plumbing.
- guid-powered: give TRG*NBSM / *NBSM* a word-level form (agc_dp_words_t out) so a component can stay as sign-disagreeing words; clears oracle KNOWN PA-W4 (AVEIT VSR3 rounding of an exact half).

## From review-prog-align final (2026-10-11; note docs/modules/review-prog-align.md, findings A1-A15, C1-C10)
- guid-entry (follow-up-done-lanes): entry_earrot duplicates EARROT1/2 (now agc_csm_earrot2 in the alignment lane); use one.
- prog-align residual: AOTMARK's job and button-handling flow was not re-read line by line (arithmetic is oracle-verified); a later pass or the document wave should read it against the listing.

## From review-sys-misc final (2026-10-12; note docs/modules/review-sys-misc.md, findings S10-S18)
- guid-powered: src/csm/burns/sps/clock.c:464 stores OPTIND = -1 where the AGC stores -0.
- follow-up-done-lanes: make agc_svc_b5off, trfailof, trfailon void (callers in other lanes); fold agc_svc_set_extvbact into agc_erase_set_extvbact.
- io-dsky: P27 cannot receive data until something calls agc_uplink_update_write (wire the uplink path from the UPRUPT handler / V71-V73 flow).
- sim: drive the scaler channels 3 and 4 (the AGC's SCALER1/SCALER2 read-back) as yaAGC does.
