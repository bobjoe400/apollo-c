# Lane-specific review items (routed from oracle findings, open questions, worklog)

- core-numeric: UNIT precision at B27 for short vectors (R63 ranges); oracle F1-F6; stale-DVSIGN deviation on V/SC and UNIT overflow (model or justify). **Word-level MPAC API** (requested by nav-aux, imu, prog-rendz, guid-entry): a public type/ops carrying DP/TP values as the AGC's sign-disagreeing word pairs, with DAS/DMP/DDV/SQRT/ASIN/ACOS on it, so R1A in GETX, GCOMP, 1-cos A and the entry L/D residuals can be bit-exact; keep agc_dp_t for the common path. Also: SQRT keeping MPAC+2; TP-input ASIN/ACOS; `agc_dp_sign` AGC SIGN semantics confirmed.
- exec-sched: deferred-restart vs immediate ENEMA (observable differences); V37 handoff completeness; activity-light mask fix verified; P23/R22LEM restart-priority deviation; ZOOM/P40ZOOMA group-3 question.
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
