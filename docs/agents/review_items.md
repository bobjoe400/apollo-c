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
- DONE exec-sched SPECTEST lead: false alarm in flight (every PRIOCHNG caller is a job body); docs/modules/follow-up-done-lanes.md "EVA lead decisions".
- DONE exec-sched JOBWAKE(NONE) lead: closed by review-exec-sched R12 (nav-integ INTSTALL side stays with nav-integ); docs/modules/follow-up-done-lanes.md.
- DONE exec-sched restarts.c:54/72 lead: closed by review-exec-sched R11; docs/modules/follow-up-done-lanes.md.
- DONE guid-landing ENGOFFDT: ENGOFF only after TGO < 4 s (BMN ENGOFF), ENGOFFDT <= ~600 cs; docs/modules/follow-up-done-lanes.md.
- DONE sys-misc / io-dsky guards: VBRQWAIT hid POODOO 1204 (fixed, test); the others take SFOUTAB constants / pinball-internal values (false alarm); docs/modules/follow-up-done-lanes.md.
- DONE dap-csm S40.14: the AGC forms signed gains unchecked; guard removed, agc_csm_dap_rcs_gains void, test; docs/modules/follow-up-done-lanes.md.
- DONE imu GCOMP read-back (status filter, now propagated); guid-landing IGNALOOP dropped an INTSTALL failure (fixed); THROTTLE DOIT refused a PIFPSET the AGC sends (guard removed, TS PSEUDO55 overflow skip modelled, FLATOUT void, test); docs/modules/follow-up-done-lanes.md.
- DONE prog-rendz targeting: `AGC_ASSERT(st == OK) -> BAD_ARG` propagated (item 1.1); DVLOS / LV-row / HPER postconditions were not the AGC's (deleted, two functions void); ALMXIT code range false alarm; docs/modules/follow-up-done-lanes.md.
- nav-integ: "signed overflow" at orbital_integration.c:430 (XSUM = XS + ASCALE + N4 + N1, all NORM counts and table shifts < 100): EVA does not carry those bounds through the out-parameters; no host UB. (core-numeric divide.c:97 part DONE: false alarm, docs/modules/follow-up-done-lanes.md.)
- sim (sim, partial run): clock.c:207/245/251/330, socket.c:138: harness guards reached in EVA's unbounded sim context; the sim entry needs a bounded proof main at project end (agc_sim's own main timed out at 60 min).

## verify-collapse findings (2026-10-11; nothing shown wrong in the 11 reviewed lanes; note docs/modules/verify-collapse.md)
- DONE status propagation in all closed lanes (128 sites, filters and remaps); docs/modules/follow-up-done-lanes.md item 1.1.
- follow-up-done-lanes: tests must not compare assertion-report counts: tests/nav/navaux.c:502, tests/lm/dap.c:444 (check the returned status instead).
- DONE SELECTMU cannot fail: agc_rendz_selectmu void, empty branch gone; docs/modules/follow-up-done-lanes.md item 1.2.
- io-dsky: src/io/dsky/display_if.c:1063 agc_dsky_blanksub result stored, never read (its BUSY case cannot happen there): make the call's result meaningful or the callee void.
- imu: src/imu/maneuver.c:874 leftover assertion guard with no return; branch can never run: delete or make fallible.
- document lane: 82 function headers still say "BAD_ARG for a NULL ctx" on functions without ctx; inline_one.py pasted 77 helper header blocks into caller bodies: rewrite as section comments.
- tools/prove/manual.tsv: the "why" column is one boilerplate text per directory; it records the decision, not the evidence (lesson for the end-of-project EVA pass: per-row evidence or none).

## From review-exec-sched (2026-10-11; note docs/modules/review-exec-sched.md)
- DONE PHSPRDT5 preset at SIDETRAK + test; docs/modules/follow-up-done-lanes.md item 1.3.
- DONE PHSPRDT2 presets (R22LEM3, P23, CSM R22) + test; docs/modules/follow-up-done-lanes.md item 1.4.
- nav-integ: plain type-C PHASCHNG calls (GRP2PC 04022 family) made from a running job now store the current job segment as the restart point and a group-2 restart reruns that segment from its top (DEVIATION in exec): check each such segment in your files is safe to rerun. (prog-rendz part DONE: rz_grp2pc names the restart job at 30 sites; docs/modules/follow-up-done-lanes.md item 1.7.)

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
- DONE S40.6 OPTIND = -0 (AGC_OPTIND_NEG0) + test; docs/modules/follow-up-done-lanes.md item 1.5.
- follow-up-done-lanes: make agc_svc_b5off, trfailof, trfailon void (callers in other lanes); fold agc_svc_set_extvbact into agc_erase_set_extvbact.
- io-dsky: P27 cannot receive data until something calls agc_uplink_update_write (wire the uplink path from the UPRUPT handler / V71-V73 flow).
- sim: drive the scaler channels 3 and 4 (the AGC's SCALER1/SCALER2 read-back) as yaAGC does.

## From review-guid-powered final (2026-10-12; note docs/modules/review-guid-powered.md, R10-R15)
- core-numeric (follow-up-done-lanes): a DOT / VXM that takes stored words (last 2 LSB of oracle s40_2); move agc_vec_words_t into include/agc/core/word.h; UNIT-at-B27 behind the radial case's 10 LSB.
- prog-align: KNOWN PA-W4 closed (oracle_align 570/0 hard at tolerance 0).

## From review-nav-integ final (2026-10-12; note docs/modules/review-nav-integ.md, N8-N12)
- follow-up-done-lanes (nav-aux): give RP-TO-R a word-level input (N10; moon OBLATE 1-2 LSB).
- follow-up-done-lanes (nav-conic): KEPPREP 1 LSB (N4).
- follow-up-done-lanes (core-numeric): public word-level vector API (nav-integ has a local word vector type; guid-powered asked for agc_vec_words_t in word.h: one type, one place).
- follow-up-done-lanes: make agc_integ_svdwn / agc_integ_rectify void once their callers drop the status check; public renames listed in the nav-integ note (agc_integ_midtoav1/2, agc_integ_intwake0, agc_job_statint1*, AGC_JOB_STATINT1*) per NAMING.md, callers and registry included.
- follow-up-done-lanes (uplink/P27): the job segment rerun from INTWAKEU's PHASCHNG 04026 after a restart must be safe (nav-integ DEVIATION N12: INTSTALL takes the area over).
