# Lane-specific review items (routed from oracle findings, open questions, worklog)

- core-numeric: UNIT precision at B27 for short vectors (R63 ranges); oracle F1-F6; stale-DVSIGN deviation on V/SC and UNIT overflow (model or justify). **Word-level MPAC API** (requested by nav-aux, imu, prog-rendz, guid-entry): a public type/ops carrying DP/TP values as the AGC's sign-disagreeing word pairs, with DAS/DMP/DDV/SQRT/ASIN/ACOS on it, so R1A in GETX, GCOMP, 1-cos A and the entry L/D residuals can be bit-exact; keep agc_dp_t for the common path. Also: SQRT keeping MPAC+2; TP-input ASIN/ACOS; DDV and SR on stored word pairs (review-guid-landing R1); `agc_dp_sign` AGC SIGN semantics confirmed.
- exec-sched: B4 from the sim lane (history/lanes/sim.md "Bugs found"): src/exec/waitlist.c wait2() -- the DEVIATION claiming the TIME3-overflow-already-happened branch (C(TIME3) < 200 octal) cannot arise is wrong; it arises inside a RUPTAGN chain (two tasks due in the same cs), WAIT2 (WAITLIST C p.1225 / L p.1121) forms TD - T1 + 1 from 200 - C(TIME3); the C code pushes the second task 163.84 s away (P30: CLOKTASK starves MONREQ, V16N45 stays blank). Fix per the rope + a test in tests/exec/exec.c; then flip the scenario's xexpect. Also: tests/exec/exec.c test_phase_codes passes a mismatched phase code (040064 to group 3) as a negative test and counts the silent assertion report; after status-handling-2 makes agc_phase_change return agc_status_t, the test must CHECK the returned status instead (no assertion-report counting as a pass criterion anywhere in tests). R12 from review-dap-lm: on LM software restart startsb1() in src/exec/fresh_start.c must call agc_lm_dap_restart() (clears RCSFLAGS bit 13 / ACCSOKAY, reruns 1/ACCS, resets t5adr) -- otherwise STARTDAP asserts. src/exec/nouns.c:155 RDOT noun must read HDOTDISP (agc_guid_state()->lm.hdotdisp), not the unwritten agc_lm_asc()->rdot (review-guid-landing R2); rename AGC_JOB_REFAZE10 (review-guid-entry); agc_phase_set_prdt (review-prog-rendz). Deferred-restart vs immediate ENEMA (observable differences); V37 handoff completeness; activity-light mask fix verified; P23/R22LEM restart-priority deviation; ZOOM/P40ZOOMA group-3 question.
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
- DONE DMP vs DMPR and shift-rounding audit: the closed lanes in history/waves/follow-up-done-lanes.md item 3.1 (and their reviews); nav-integ in review-nav-integ.

- sim: nothing calls agc_lm_dap_time6_count / agc_lm_dap_t6_pending, so TIME6 never counts and no T6RUPT ends a jet firing in scenarios (review-dap-lm request); wire TIME6 into the clock driver.

## Word-level API adoption (review-core-numeric, 2026-10-11)
- DONE imu, guid-landing (TBUP/TGO), guid-entry (RDOT, HUNTEST1, CONSTD), nav-aux (ARCTAN ASIN, GETERAD, TFF, LSPOS, MOONMX/NEWANGLE), pair types unified: history/waves/follow-up-done-lanes.md items 3.2, 5.1; nav-conic, prog-align, dap-csm, dap-lm closed in their reviews.

- DONE agc_imu_hw_set_cducmd widened to the SP range + test; history/waves/follow-up-done-lanes.md item 2.4.

## EVA bug leads (status-handling-2, make prove 2026-10-11; run artefacts in build/prove/*.csv, build/prove/eva_verdicts.tsv, (the build/prove/tree snapshot was deleted 2026-10-12: stale, and editors picked its headers up), tools/prove/{summary.txt,progress.log,runs/*.log}; positions are the snapshot's = the tree of that date)
EVA is the project-end cleanup step; these are leads for the owning reviews, not fixes.
- DONE exec-sched SPECTEST lead: false alarm in flight (every PRIOCHNG caller is a job body); history/waves/follow-up-done-lanes.md "EVA lead decisions".
- DONE exec-sched JOBWAKE(NONE) lead: closed by review-exec-sched R12 (nav-integ INTSTALL side stays with nav-integ); history/waves/follow-up-done-lanes.md.
- DONE exec-sched restarts.c:54/72 lead: closed by review-exec-sched R11; history/waves/follow-up-done-lanes.md.
- DONE guid-landing ENGOFFDT: ENGOFF only after TGO < 4 s (BMN ENGOFF), ENGOFFDT <= ~600 cs; history/waves/follow-up-done-lanes.md.
- DONE sys-misc / io-dsky guards: VBRQWAIT hid POODOO 1204 (fixed, test); the others take SFOUTAB constants / pinball-internal values (false alarm); history/waves/follow-up-done-lanes.md.
- DONE dap-csm S40.14: the AGC forms signed gains unchecked; guard removed, agc_csm_dap_rcs_gains void, test; history/waves/follow-up-done-lanes.md.
- DONE imu GCOMP read-back (status filter, now propagated); guid-landing IGNALOOP dropped an INTSTALL failure (fixed); THROTTLE DOIT refused a PIFPSET the AGC sends (guard removed, TS PSEUDO55 overflow skip modelled, FLATOUT void, test); history/waves/follow-up-done-lanes.md.
- DONE prog-rendz targeting: `AGC_ASSERT(st == OK) -> BAD_ARG` propagated (item 1.1); DVLOS / LV-row / HPER postconditions were not the AGC's (deleted, two functions void); ALMXIT code range false alarm; history/waves/follow-up-done-lanes.md.
- nav-integ: "signed overflow" at orbital_integration.c:430 (XSUM = XS + ASCALE + N4 + N1, all NORM counts and table shifts < 100): EVA does not carry those bounds through the out-parameters; no host UB. (core-numeric divide.c:97 part DONE: false alarm, history/waves/follow-up-done-lanes.md.)
- sim (sim, partial run): clock.c:207/245/251/330, socket.c:138: harness guards reached in EVA's unbounded sim context; the sim entry needs a bounded proof main at project end (agc_sim's own main timed out at 60 min).

## verify-collapse findings (2026-10-11; nothing shown wrong in the 11 reviewed lanes; note history/waves/verify-collapse.md)
- DONE status propagation in all closed lanes (128 sites, filters and remaps); history/waves/follow-up-done-lanes.md item 1.1.
- DONE navaux.c / lm dap.c tests check statuses, not assertion-report counts; history/waves/follow-up-done-lanes.md item 4.1.
- DONE SELECTMU cannot fail: agc_rendz_selectmu void, empty branch gone; history/waves/follow-up-done-lanes.md item 1.2.
- DONE NVDSP checks the BLANKSUB status; history/waves/follow-up-done-lanes.md item 4.2.
- DONE maneuver.c leftover and dead INCRDCDU guards deleted; history/waves/follow-up-done-lanes.md item 4.2.
- document lane: 82 function headers still say "BAD_ARG for a NULL ctx" on functions without ctx; inline_one.py pasted 77 helper header blocks into caller bodies: rewrite as section comments.
- tools/prove/manual.tsv: the "why" column is one boilerplate text per directory; it records the decision, not the evidence (lesson for the end-of-project EVA pass: per-row evidence or none).

## From review-exec-sched (2026-10-11; note docs/modules/review-exec-sched.md)
- DONE PHSPRDT5 preset at SIDETRAK + test; history/waves/follow-up-done-lanes.md item 1.3.
- DONE PHSPRDT2 presets (R22LEM3, P23, CSM R22) + test; history/waves/follow-up-done-lanes.md item 1.4.
- nav-integ: plain type-C PHASCHNG calls (GRP2PC 04022 family) made from a running job now store the current job segment as the restart point and a group-2 restart reruns that segment from its top (DEVIATION in exec): check each such segment in your files is safe to rerun. (prog-rendz part DONE: rz_grp2pc names the restart job at 30 sites; history/waves/follow-up-done-lanes.md item 1.7.)

## From review-sys-misc checkpoint (2026-10-11; note history/lanes/review-sys-misc.md)
- DONE V91 shows the bank sum through MPAC+2 (agc_dsky_display_ecadr); history/waves/follow-up-done-lanes.md item 2.6.
- sim: load the real rope image (Comanche055 / Luminary099 binary from tools/oracle) as fixed memory so the idle self-check (ROPECHK) and SHOW-BANKSUM run on real bank sums; data only, no model.

## From review-prog-align checkpoint (2026-10-11; note history/lanes/review-prog-align.md)
- DONE agc_csm_sxtnb / agc_lm_rrnb / agc_lm_rrangles void (and SXTSM, SXTSM marks, GETUM after them); history/waves/follow-up-done-lanes.md item 2.1.
- DONE *NBSM* word-level form: review-guid-powered R11 (docs/modules/review-guid-powered.md); KNOWN PA-W4 closed.

## From review-prog-align final (2026-10-11; note history/lanes/review-prog-align.md, findings A1-A15, C1-C10)
- DONE entry uses agc_csm_earrot1/2 (one EARROT); history/waves/follow-up-done-lanes.md item 4.2.
- DONE prog-align residual: AOTMARK job and button flow read line by line, no discrepancy; history/waves/follow-up-done-lanes.md item 6.1.

## From review-sys-misc final (2026-10-12; note history/lanes/review-sys-misc.md, findings S10-S18)
- DONE S40.6 OPTIND = -0 (AGC_OPTIND_NEG0) + test; history/waves/follow-up-done-lanes.md item 1.5.
- DONE agc_svc_b5off / trfailof / trfailon void; agc_svc_set_extvbact folded into agc_erase_set_extvbact (3 callers, Coccinelle); history/waves/follow-up-done-lanes.md item 2.2.
- DONE P27's loads arrive through the MPAC+2 ECADR of the V21N01/N02 display; history/waves/follow-up-done-lanes.md item 2.5.
- sim: drive the scaler channels 3 and 4 (the AGC's SCALER1/SCALER2 read-back) as yaAGC does.

## From review-guid-powered final (2026-10-12; note docs/modules/review-guid-powered.md, R10-R15)
- DONE agc_vec_words_t in word.h, DOT/VXM/MXV/VXV/VXSC/VAD/VSL/VSR on stored words; s40_2 sps 0 LSB; radial 10 LSB remain (word-level UNIT not built); history/waves/follow-up-done-lanes.md item 2.3.
- DONE prog-align KNOWN PA-W4 closed (oracle_align 570/0 at tolerance 0; review-guid-powered R11).

## From review-nav-integ final (2026-10-12; note docs/modules/review-nav-integ.md, N8-N12)
- DONE RP-TO-R / R-TO-RP / MOONMX / NEWANGLE word-level, word-input entry points; latlong_moon exact; history/waves/follow-up-done-lanes.md item 5.1.
- DONE KEPPREP: 24 repro runs exact against the DV-patched probe at HEAD (not reproduced); history/waves/follow-up-done-lanes.md item 5.2.
- DONE one word vector type and API in core (nav-integ's local copy replaced); history/waves/follow-up-done-lanes.md items 2.3, 5.3.
- DONE agc_integ_svdwn / agc_integ_rectify void; nav-integ public renames with callers and registry; history/waves/follow-up-done-lanes.md item 5.4.
- DONE INTWAKEU 04026 restarts at UPOUT +1 (named), not the P27 segment top; history/waves/follow-up-done-lanes.md item 5.5.

## Open residuals after follow-up-done-lanes (2026-10-12; all routed items closed)
- DONE guid-entry DOWNCNTL / GETETA on the MPAC words, V1 kept as stored words (the cause, not D); oracle entry L/D, L/D1 at 0; history/waves/loose-ends.md item 1.
- DONE prog-align item 2b row marked DONE (resume 3 (a)); history/waves/loose-ends.md item 5.

## From fix-b6 and review-sim (2026-10-12)
- DONE exec-sched waitlist 16250 cs bound: the AGC only documents it; kept as a DEVIATION with the reason + test; history/waves/loose-ends.md item 2.
- DONE sim pad keyword sps_impulse + sim/scenarios/external_delta_v_impulsive_burn_csm.txt (impulsive path); history/waves/loose-ends.md item 3.
- DONE exec-sched 64-jobs-per-call bound: all hits are CHECKG1 polling; 64 exceeds the AGC's job rate; rationale in the clock_exec prolog; history/waves/loose-ends.md item 4.

## From the document wave's first pass (2026-10-12; routed to loose-ends)
- DONE exec-sched exec_cp_cleanup_lm: false alarm, body matches L p.229 (test added); history/waves/loose-ends.md item 6.
- DONE exec-sched agc_power_on header names agc_rupt_go(); history/waves/loose-ends.md item 7.
- DONE exec-sched agc_waitlist_time3 / agc_waitlist_entry comments; history/waves/loose-ends.md item 8.

## From prolog-consistency (2026-10-12)
- DONE loose-ends GET+MGA: second UNIT on the LM as in Luminary (no oracle case exists) + test; prolog-report 6 -> 0 in src/exec and src/csm/entry; history/waves/loose-ends.md items 9, 10.
- ~~always-ok: src/lm/radar/leadin.c agc_job_radar_sample_done skips a store silently when its assertion fails (check_rules misses the pattern: an assertion whose false branch skips a statement without a status); fix per rule 7 and teach check_rules the pattern.~~ DONE (always-ok: 22 other silent-skip sites open, see history/waves/always-ok.md)
- ~~always-ok: returns-extra (body returns a status the prolog does not name) is described in tools/prolog_check.py but not implemented (~172 functions); implement it and fix the prologs.~~ DONE (always-ok)

## After the EVA pass (2026-10-12)
- ~~post-eva: 21 positive-assertion setter guards (`if (AGC_ASSERT(c)) { store; }`, 16 in src/lm/dap/state.c, imu/compensation.c x2, lm/dap/rcs/t6rupt.c, csm/navigation/tracking_data.c, csm/entry/dap.c) left for EVA by user decision (classifier blocked the lane's deletion; per-site reasoning in history/waves/always-ok.md): delete those EVA proves VALID in the apply step, convert the rest to status returns.~~ DONE (finish, 2026-10-12): 1 deleted (EVA VALID: executive.c end_of_job; the list named lm/dap/rcs/t6rupt.c, the site was executive.c), 20 return BAD_ARG with every caller handling it; history/waves/finish.md.
- ~~post-eva: `make check` fails on exactly those 21 (check-rules "silent skip") until they are resolved; note `python3 tools/check_rules.py` run bare reports 0 and "density 0/1" (it does not scan the same set as the Makefile's check-rules target): make the bare invocation match the target.~~ DONE (finish, 2026-10-12): `make check` passes; a bare check_rules.py scans src include (DEFAULT_ROOTS), the Makefile target uses the bare form.

## From the document wave (2026-10-12, history/waves/document.md Findings; for a closed-loop-fixes lane with the LM Simulator bugs)
- ~~guid-powered: src/csm/burns/sps/burn.c: a failed assertion on P40TMP silently sets it to 1 (rule 7: delete if guaranteed, else return a status the caller handles).~~ DONE (closed-loop-fixes, P40TMP; docs/modules/closed-loop-fixes.md)
- ~~core: src/core/erasable.c and include/agc/core/state.h comments describe assertion checks the accessors no longer contain: fix the comments.~~ DONE (closed-loop-fixes, stale comments; docs/modules/closed-loop-fixes.md)
- ~~all: about six assertions repeat the statement just above them and cannot fail: delete (list in document.md Findings).~~ DONE (closed-loop-fixes, repeated assertions; docs/modules/closed-loop-fixes.md)
- ~~prog-align: two P52 function prologs name the label "CALL" instead of S52.3 / S52.2.~~ DONE (closed-loop-fixes, stale comments; docs/modules/closed-loop-fixes.md)
- ~~exec: agc_exec_vac_claim's prolog describes an output pointer as an input index.~~ DONE (closed-loop-fixes, stale comments; docs/modules/closed-loop-fixes.md)

## From lm-simulator (2026-10-12; closed loop vs yaAGC; history/lanes/lm-simulator.md "Bugs found" and Findings) -- closed-loop-fixes lane
- ~~L1 (io-dsky/exec): a V37 refused with alarm 1520 also lights OPR ERR: cp_refuse returns AGC_STATUS_ALARM, which PIN_STATUS_RECYCLE aliases (src/io/dsky/pinball/internal.h:52), so ENTER runs again (change_program/sequence.c:117, pinball/keys.c:448). AGC CANTROD/V37BAD, Luminary p.227.~~ DONE (closed-loop-fixes, L1; docs/modules/closed-loop-fixes.md)
- ~~L2 (imu/verbs): extended_verbs.c:828 FINEK2 pulses gyro.c's private OGC buffer but the V25N93 load writes agc_align_ram()->ogc (nouns.c:235): V42 fine align never torques. EXTENDED_VERBS Luminary p.273.~~ DONE (closed-loop-fixes, L2; docs/modules/closed-loop-fixes.md)
- ~~L3 (dap-lm): t6rupt.c:252 MIDORLST computes +0 where the AGC's SU TIME6 gives -0; T6JOBCHK then hits CCSHOLE and POODOO 01103 during jet firings. Q_R-AXIS Luminary p.1457.~~ DONE (closed-loop-fixes, L3; docs/modules/closed-loop-fixes.md)
- ~~PROG display after a POODOO restart: yaAGC blank, ours 00 (not root-caused).~~ DONE (closed-loop-fixes, D1: GOP00FIX runs on in core set 0 over DSPMMJOB; docs/modules/closed-loop-fixes.md)
- ~~S-L1 (sim): channels.c:200 broadcasts ch5/6 only on change; yaAGC sends a packet on every write and the LM Simulator applies jets one packet late: match yaAGC.~~ DONE (closed-loop-fixes, S-L1; docs/modules/closed-loop-fixes.md)
- ~~S-L2 (sim): CDU counts applied at once where yaAGC paces them at 400/6400 per second (review-sim paced the drive; check the LM Simulator path).~~ DONE (closed-loop-fixes, S-L2; docs/modules/closed-loop-fixes.md)
- ~~S-L3 (alignment): coarse-align alarm 00211 raised by yaAGC, not by ours (finding in the note).~~ DONE (closed-loop-fixes, S-L2 / S-L3; docs/modules/closed-loop-fixes.md)

## From the document wave, final (2026-10-12; history/waves/document.md Findings) -- layering + comments lane after NASSP milestones
- ~~LAYERING (definition of done item 2): src/core/ecadr.c (NASSP scenario-restore address map) includes and calls csm, guidance, programs and other higher-level headers; src/imu/gyro.c calls agc_align_ram() in programs (closed-loop fix L2). core must not call upward. Fix: move the address map out of core into a top-of-graph module (e.g. src/erasable_map/ or alongside src/sim, which already depends on everything) with core keeping only the generic ECADR mechanics; move the OGC/IGC/MGC gyro-command words into imu (their owner) so alignment calls imu, not the reverse. Regenerate docs/overview module map after (tools/depgraph.py) and show no upward edge from core.~~ DONE (nassp-integration: src/erasable_map; layering: OGC in imu, every other upward edge removed, depgraph shows none; history/waves/layering.md "Decisions per edge")
- ~~comments: six file prologs promise assertions the files no longer contain (lm/dap/arith.c, gts.c, rcs/rcsmon.c, rcs/spsrcs.c, nav/auxiliary/util.c, io/service/rtb_ops.c); erasable.c line 9 claims per-accessor range checks; keys.c agc_dsky_uprupt has dead status code in its ERROR RESET branch; ecadr.c lacks the boxed file prolog.~~ DONE (layering: six prologs, erasable.c, keys.c CLUPLOCK -> ACCEPTUP; ecadr.c left to nassp-integration, whose file it is)
- document: the nassp-integration.md "Program flow and decisions" section is prepared in the scratchpad document/extra/ (apply with addsec.py when that lane ends).

## Later (2026-10-13)
- core-numeric: src/nav/auxiliary/free_fall.c:443 trips -Werror=conversion when built at -O0 (not the project's flags); make the expression explicit so every optimisation level builds.

## Upstream to NASSP after the mission (2026-10-13)
- File an issue/PR on rcflyinghokie/NASSP (or orbiternassp) for the Saturn console-mode crash: Saturn::clbkPostStep calls oapiCameraInternal()/oapiCockpitMode() without the focus guard the LEM has; Orbiter 2024 faults with no graphics client. Patch: sim/nassp/patches/0003-saturn-console-mode.patch.
- Note the deprecated Orbiter API use (get_horizonairspeedvector logged every frame in console mode during entry) as a 2024-compatibility item; NASSP's code, not ours.
- Offer patch 0002 (LEM Lua test interface) as an optional test hook.
