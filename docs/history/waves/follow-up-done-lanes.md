# follow-up-done-lanes -- routed items for the closed review lanes

Assignment: docs/history/assignments/follow-up-done-lanes.md. Work list:
docs/agents/review_items.md, items owned by nav-aux, io-dsky, state-flags,
guid-entry, imu, guid-landing, prog-rendz, dap-lm, dap-csm, nav-conic,
core-numeric, exec-sched, sys-misc, prog-align, guid-powered and this lane.
Not touched: src/nav/integration/** (review-nav-integ running), sim, document.

Build used while the other agent runs: `make BUILD=<scratch>/build`.

## Progress

Order (1) fidelity / status, (2) API, (3) DMP + word-level, (4) hygiene.

- [x] 1.1 DONE status propagation: 83 `if (!AGC_ASSERT(st == OK)) return BAD_ARG/st` -> `if (st != OK) return st` (script); 19 stale `OK || FULL/UNIMPLEMENTED/BUSY/NOT_FOUND` filters before `return st` deleted; 26 compound/filtered sites by hand; 6 entry steps and 1 entry NOVAC stored BAD_ARG/FULL for a callee status; 4 GCOMP reads + 2 SCAXIS stores; stale FULL in 10 RETURNS comments
- [x] 1.2 DONE SELECTMU (P34-35 p.494) cannot fail (MUE/MUM rope constants): agc_rendz_selectmu void, empty if in agc_rendz_reset gone, 9 callers (Coccinelle + 4 dead status locals)
- [x] 1.3 DONE SIDETRAK (REENTRY_CONTROL p.861) CA PRIO16 / TS PHSPRDT5 -> agc_phase_set_priority(5, PRIO16) before PHASCHNG 00474; test_sidetrak_restart_priority (tests/csm/entry.c; fails without the preset)
- [x] 1.4 DONE PHSPRDT2 presets via agc_phase_set_priority: LM R22LEM3 PRIO26 (L P20-P25 p.506), CSM P23 PRIO13 (C p.619), CSM R22 PRIO26 (C p.576); DEVIATION notes removed; test_group2_restart_priority (tests/programs/rendezvous.c)
- [x] 1.5 DONE S40.6 (P40-P47 p.730) CS ZERO / TS OPTIND stored -1 (coarse optics not working) instead of -0 (reserved for TVC, AGC_OPTIND_NEG0): 33OPTMON did not bypass, V41 did not alarm; test_tvc_optics_reserved (tests/guidance/guidance.c)
- [x] 1.6 DONE EVA leads, one decision each (details under "EVA lead decisions"): 4 real (VBRQWAIT hid POODOO 1204; S40.14 and DOIT refused values the AGC uses; IGNALOOP dropped an INTSTALL failure), 3 non-AGC postconditions deleted (S34/35.4 DVLOS, LV-to-reference row, HPER after MAXCHK), 2 closed earlier by review-exec-sched (R11, R12), rest false alarms with the reason
- [x] 1.7 DONE GRP2PC rerun check: 53 plain 04022 calls in the prog-rendz files (libclang list); new rz_grp2pc(restart_at) names the AGC's return address as a job; 30 sites named (15 at the top of a job, waits -> CONTCHK, REND2 -> REND4, the LM measurement sequence -> new AGC_JOB_RENDEZVOUS_TRACKING_DATA_MEASURE_CONTINUE, S22I=N); the other 25 are inside one segment before a named one, or rerun only integrations/INCORP1 (verdict table); test in tests/programs/rendezvous.c
- [x] 2.1 DONE agc_csm_sxtnb, agc_lm_rrnb, agc_lm_rrangles void (RRANGLES single return); cascade: agc_csm_sxtsm, agc_csm_sxtsm_marks void; callers by Coccinelle + 9 by hand (designate, acquisition OMEGCALC, LM R22 mark, fine align, R22/P22 GETUM, R34 display, align tests). GETUM (R22, P22) and the R22 optics-mark status went with them
- [x] 2.2 DONE agc_svc_b5off, agc_svc_trfailof, agc_svc_trfailon void (11 + 4 callers by Coccinelle, R22 range job by hand); agc_svc_set_extvbact removed, its 3 callers renamed to agc_erase_set_extvbact (Coccinelle)
- [x] 2.3 DONE agc_vec_words_t in core/word.h (guidance.h copy removed); interp_words.h vector API on stored words: agc_vec_to_words/from_words, agc_mpac_vdot, _vxm, _mxv (rows as words), _vxv, _vxsc, _vad, _vshift (src/core/interp/vector_words.c, DOTSUB on word pairs agc_w_dotsub_words); S40.2,3 SPS uses VXM on the VDEF words and VXV VSL1 rows: s40_2_sps / _sps_b tol 2 -> 0; radial stays 10 (UNIT of VXV words: word-level UNIT not built); test_vector_words
- [x] 2.4 DONE agc_imu_hw_set_cducmd takes the SP range (was +-COMMAX 192; NEEDLES writes +-768) and now checks the axis; test_cducmd_needle_range
- [x] 2.5 DONE display interface carries the user's MPAC+2 (dif_req_t / class copy, set before NVSUB; new agc_dsky_display_ecadr); P27's V21N01/N02 passes the ECADR of the word (Comanche COMPNUMB 0300 / UPTEMP 0330, Luminary 1170 / 1167, from the yaYUL listings); the load job takes the ground's word from that address; tests test_display_machine_address (dsky), test_uplink_clock_increment through the ECADR (sysmisc)
- [x] 2.6 DONE V91 SDISPLAY: MPAC+2 = ADRS1 (SKEEP1, 1371 both ropes), SKEEP1-3 copied to the image (DEVIATION: typed storage), V05N01 through agc_dsky_display_ecadr; test_showsum_display
- [x] 3.1 DONE DMP/DMPR audit for the early lanes: prog-rendz, imu, guid-landing, guid-entry (E1), dap-lm closed it in their reviews; this pass: nav-aux 4 DMPR sites (LALOTORV GAMMA SIN LAT, SIN/COS LONG, LAT-LONG DMPR BDSU) all agc_dp_mul, every other multiply truncating; entry VGAMCALC/TFF/TRIG already DMP (agc_dp_mul_trunc); CM/POSE VXSC sites rounded (correct); io-dsky and state-flags have no interpretive multiply. Changed: 0
- [x] 3.2 DONE word-level adoption: pair types unified on agc_dp_words_t (lm_dp_pair_t, agc_lm_words_t, word_pair_t removed; agc_lm_words/_value/_das_ovf and imu pair_of replaced by core; lm_dap_das, agc_lm_das, imu das on agc_dp_words_das); imu DELCOMP / CDUTODCM / DCMTOCDU on agc_mpac_* (oracle_imu delcomp 4->0, dcmtocdu 3->0, kalc 4/1/4/2 -> 0/0/1/1). nav-aux DONE: ARCTAN SR1 ASIN and GETERAD on the MPAC words, TFFCONIC / CALCTFF / TFFXTEST on words (agc_tff_t.tffalfa now agc_dp_words_t), LM LSPOS GTMP/STMP words into SIN/COS/MXV; oracle_nav_aux latlong 2 -> 0, lalotorv 4 -> 0, tff 12 -> 0 (TFF 4 -> 0, TFFX 32 -> 0), lspos_lm 2 -> 0: every nav_aux case at 0. landing DONE: agc_lm_asc_t tgo, tgo1, tbup, geff, dzdot, dydot, drdot, pcons, ycons as agc_dp_words_t; ATMAG, ASCENT (VG, GEFF), MAINENG, T2TEST/RATES/CONST, CMPONENT/AIMER, TGOCOMP, OLDTIME, BOTHPOLY on agc_mpac_*; oracle_landing ascent AT 4, TTOGO 1, PRATE/YRATE 32, PCONS/YCONS 4, ATR..PITCH 8, UNFC/2 8 -> all 0, plus high-word checks of TBUP TGO PCONS YCONS (278 checks, 0 failures). entry DONE: RDOT (DOT SL1 on agc_mpac_dot), HUNTEST A0CALC/V1LEAD/HUNTEST1 (V1 DAD words into the DMP/DDV chain, ALP, FACT1, FACT2, VL, GAMMAL1), CONSTD/CONSTD1 (K2D term and DREF as words in the pass; DOWNCNTL stays on values: its DREF words need D as stored, see Findings F3); oracle_entry VL 40 -> 0, L/D 256 -> 9, L/D1 256 -> 8, THETAH 4 -> 3, ROLLC 8 -> 6, pass/startent general 1/4 -> 0 (RDOT, A1 exact); return_to_earth KNOWN 52 -> 47 (GETERAD words move the F14 conic search)
- [x] 4.1 DONE tests/nav/navaux.c and tests/lm/dap.c no longer compare assertion-report counts (the returned statuses are checked)
- [x] 4.2 DONE display_if NVDSP: BLANKSUB status checked (BUSY is the AGC's other return, other failures propagate); maneuver.c MANUSTAT leftover guard and INCRDCDU's loop-index / just-stored / NULL guards deleted (INHINT is released in INCRDCDU, as the AGC); entry_earrot now passes the entry erasables to agc_csm_earrot1/2 (one EARROT; earrot1 void); oracle_entry 521/0 unchanged
- [x] 4.3 DONE tests/oracle/targeting.c: initvel_moon 32 -> 0, initvel_earth_prec 3400 -> 1, cdh_earth 520 -> 2, cdh_earth_ell 760 -> 4, tpi_earth 2 -> 2, tpi_earth_b 760 -> 0, DIFFALT 32 -> 0 (116 checks, 0 failures)
- [x] 5.1 DONE NEWANGLE, EARTHMX, MOONMX, R-TO-RP and RP-TO-R run on the push-down words (agc_mpac_*: NEWANGLE's TP MPAC into COS, DMP SL1R, VXSC/VAD/VSU/VSL1, VXV/BVSU/MXV/VXM); new agc_planet_r_to_rp_words / agc_planet_rp_to_r_words take 504RPR as stored; oracle_nav_aux latlong_moon_rm/_rls 256 -> 0 (were 23 / 202 LSB), rptor/rtorp_moon 4 -> 0; nav-integ OBLATE passes its values (QUALITY3's V/SC output is a quotient, sign-agreeing)
- [x] 5.2 DONE (not reproduced) KEPPREP: harness on the 24 repro runs of SCRATCH/review-nav-integ/kep against a DV-patched probe (dvcheck.sh build): XKEPNEW exact in all 24 at HEAD; every run is a DVBUG run for the committed probe, so no case was committed
- [x] 5.3 DONE nav-integ's local vec_words_t, vxsc_mpac, vec_to/from_words, vshift_words (with its VRIGHT2 copy vsr_words), vad_words replaced by the core API (Coccinelle); oracle_nav_integ 439/0 unchanged (agc_w_vright = the local VRIGHT2)
- [x] 5.4 DONE agc_integ_svdwn / agc_integ_rectify void (12 callers: Coccinelle + 8 by hand; RECTOUT cannot fail any more); renames by Coccinelle + registry: agc_integ_midtoav_at_time / _now, agc_integ_intwake_release, AGC_JOB_STATEINT_UPDATE(_STALL) / agc_job_stateint_update(_stall); docs/history/RENAMES.md rows; dispatch.c / jobs.h regenerated
- [x] 5.5 DONE INTWAKEU's PHASCHNG 04026 names the caller's continuation (new agc_integ_intwakeu `resume` argument; P27: AGC_JOB_UPLINK_UPDATE_TERMINATE = UPOUT +1, the AGC's return address) instead of rerunning the P27 segment; UPWAKE's 04026 names the execute job (rerun of the idempotent store, N12 lets INTSTALL in); test in tests/nav/integration.c
- [x] 6.1 DONE AOTMARK job and button flow (Luminary AOTMARK.agc pp. 244-260) read line by line against src/lm/optics/aotmark.c (review-prog-align residual): AOTMARK/EXTVBCHK/MKVACFND, GETDAT/DODAT/CODE7/CODE1TO6, GETMKS/PASTIT, MARKCHEX/CNTCHK/AVESTAR/AVEIT/ENDMARKS, MKRELEAS/KILLAOT, MARKRUPT/FINDKEY/SOMEKEY, X/YMKRUPT/VERIFYMK/5MKALARM, MKREJ/REJECT/REJECT2/SURFREJ, SURFSTOR/VACSTOR/REMARK/SURFJOB/CHANGEVB, DSPV6N79/SURFEND/SURFAGAN: every branch, MARKSTAT bit and alarm (105 107 111-115, 1207 1211) matches; no change
- [x] 7.1 DONE rule 4: agc_lm_ascent split at its natural label (as_vgvect, ASCENT first part); check_rules 0 hard

Gates (2026-10-12, after 7.1): make OK; make test 34 binaries 0 failures; make scenarios 13/13; make check rc 0 (format clean); check_rules 0 hard.

## Findings

| # | where | AGC reference | finding | action |
|---|---|---|---|---|
| F2 | src/csm/burns/sps/clock.c S40.6 | S40.6 CS ZERO / TS OPTIND (P40-P47 p.730); 33OPTMON CCS OPTIND (T4RUPT p.160) | OPTIND = -1 where the AGC stores -0: T4RUPT optics monitor kept running during the burn, V41 saw optics free | AGC_OPTIND_NEG0; test |
| F3 | src/csm/entry/reentry.c DOWNCNTL / CONSTD1 | DSTORE ABVAL DMP SL1 (D), DOWNCNTL BDDV DSU (DREF), CONSTD1 BDSU D (p. 867) | with DREF as the DSU words the C L/D is 256 LSB off; with DREF canonical, 1-9 LSB: the AGC's D is itself stored as words whose split the C does not keep, so D - DREF only agrees when both stay values | DOWNCNTL left on values (L/D 9, L/D1 8); closing it needs D (and every entry erasable a DSU/DMP leaves) as agc_dp_words_t |
| F4 | src/nav/auxiliary/free_fall.c TFFCONIC | TFFCONIC DAD STADR (p. 1377) | TFFALFA stored as (4615, -5359): CALCTFF's DMP TFFALFA and TFFXTEST's DDV read those words (TFFX 26, TFFTEM 1 LSB with the value) | agc_tff_t.tffalfa is agc_dp_words_t |
| F1 | 128 sites in 31 lane files | WAITLIST/NOVAC/FINDVAC BAILOUT 1201-1203 (C p.1216ff, p.1225) | callee BAILOUT/POODOO turned into BAD_ARG; the `|| FULL` filters accepted a status these calls never return (they BAILOUT) and turned the real BAILOUT into BAD_ARG | propagate the callee status everywhere; restates after FALTON / FLATOUT / SATSW deleted |

## EVA lead decisions (review_items.md "EVA bug leads")

| lead | decision | reason / action |
|---|---|---|
| executive.c SPECTEST `newjob > 0` | false alarm in flight | NEWJOB = +0 with core set 0 free needs PRIOCHNG/CHANJOB outside a job; all 22 agc_exec_prio_change callers are job bodies (libclang caller list). Reproduced only by a host test calling a job body directly (seen while writing test_sidetrak_restart_priority: the test now runs STARTSB2 first). |
| executive.c JOBWAKE(NONE) | closed | review-exec-sched R12 (NOT_FOUND, LOCCTR -1) |
| restarts.c:54/72 start_job guards | closed | review-exec-sched R11 (plain type B/C PHASCHNG stores the running segment) |
| lm/ascent.c ENGOFFDT range | false alarm | ENGOFF is reached only through `DSU 4SEC(17) / BMN ENGOFF` (ASCENT_GUIDANCE p.846): TGO < 4 s, so ENGOFFDT <= ~600 cs, far inside WAITLIST's 16250 |
| display_if.c ENDIT cont/prio, scale.c ARTOUT/DPOUT -0 and SFOUT | false alarm | arguments are SFOUTAB constants (>= 0) and pinball-internal; keep for the project-end EVA |
| monitor.c VBRQEXEC/VBRQWAIT | REAL | the `delay >= 1` guard returned BAD_ARG where WAITLIST gives POODOO 1204 (C p.1225); guards duplicated NOVAC/WAITLIST's own. Now only a crew-data range check (job/task id < COUNT, DEVIATION commented); test_request_waitlist_zero_delay |
| csm/dap/rcs/dapdata.c IXX/IAVG < 0 | REAL | S40.14 (P40-P47 p.735) never checks; MP/DV are signed and DV limits. Guard removed, agc_csm_dap_rcs_gains void; test_rcs_gains_signed |
| imu/compensation.c GCOMP read-back | resolved | the site was the `AGC_ASSERT(st == OK) || g != NULL` filter; now propagates the buffer status |
| lm/landing/throttle.c PIFPSET range | REAL | DOIT (p.795) adds PIFPSET unchecked; FCOMPSET's FP - FMAXODD can exceed FEXTRA. Guard removed; TS PSEUDO55 overflow skip of TS THRUST modelled; agc_lm_throttle_flatout void; test in test_throttle_band |
| lm/landing/lunar_landing.c IGNALOOP st | REAL | an INTSTALL status other than OK/BUSY fell through into INTEGRVS; now returned |
| targeting `AGC_ASSERT(st == OK)` -> BAD_ARG | done | item 1.1 |
| tpi.c S34/35.4 DVLOS != -DPPOSMAX, target_delta_v.c row != DPMIN, lambert_aimpoint HPER <= MAXNM | non-AGC postconditions | the AGC stores VSL1 results as they come; -UNIT(R) is within +-1/2; MAXCHK caps at MAXNM. Guards deleted; agc_rendz_tpi_los_delta_v and agc_target_delta_v_lv_to_ref void |
| csi_cdh_targeting.c ALMXIT code range | false alarm | code = 00600 + CSIALRM - 1, CSIALRM from the CSI X2 labels 1..7 |
| divide.c:97 signed overflow (core-numeric part) | false alarm | DV remainder and MP high word < 2^14 with the normalised divisor GENDDV requires; no host UB |

## GRP2PC restart points (review-exec-sched R11 request)

A C segment always runs to its end (or an error return) before a restart
is served, so what matters is the LAST group 2 phase change a segment (or
a flow of chained job bodies) makes. Plain agc_phase_change(04022) stores
the dispatched segment; a body reached by a direct call (`return
(agc_job_x())`) or a wait helper then restarts the caller from its top.

| site class | count | verdict |
|---|---|---|
| GRP2PC first statement of a job body (cislunar maneuvered, orbital over, LM tracking data / DTHETA / accept / done / residuals wait, LM navigation range check / attitude / designate / preferred attitude, LM incorporate, measure, sighting verify, radar search / display / maneuver, tracking attitude compute / maneuvered, CSM residual display) | 20 | named = the job itself: exact even when reached by chaining (R24LEM -> R24LEM2 was restarting R24LEM) |
| CSM R22 waits (R22/REND1, REND1, REND3) | 3 | named CONTCHK (AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_CHECK; DEVIATION: no 4 s wait first). Was: rerun of the incorporating segment (W and VHFCNT/TRKMKCNT twice) |
| CSM REND2 after the mark is taken | 1 | named REND4 (CSM_INCORPORATE). Was: rerun saw MRKBUF2 = -3 and dropped the mark |
| LM RANGEBQ..TRUNBQ step, R22LEM7/ASTOK, N49 display | 4 | named MEASURE_CONTINUE (new job: mu_run at WHCHREAD). Was: RANGEBQ1 rerun from RANGE re-incorporated the earlier measurements |
| P22 S22SAVET / S22I=N | 2 | named SITE / MARK |
| INCORP2 FAZC/FAZAB3 (rz_incorp2), INCORP1 / BVECTORS helpers, in-pass REND9A/REND10A/REND13/REND14A, S22BOX42 loop, DMPINTEG, 9DWTO6DW | 19 | followed in the same segment by a named GRP2PC on every normal path; a restart between them needs a BAILOUT inside INCORP2 (DEVIATION comment in rz_incorp2) |
| after INTEGRV/INTSTALL only (P23 POINTAXS, CSM P20 integrate, REND4 integrate, LM UPPSV / orbit change / INTGRCAL) | 6 | rerun repeats an integration of unchanged permanent states: benign |

## API changes

| change | callers |
|---|---|
| agc_rendz_selectmu, agc_csm_sxtnb, agc_lm_rrnb, agc_lm_rrangles, agc_csm_sxtsm(_marks), agc_svc_b5off/trfailof/trfailon, agc_csm_dap_rcs_gains, agc_lm_throttle_flatout, agc_rendz_tpi_los_delta_v, agc_target_delta_v_lv_to_ref, agc_integ_svdwn, agc_integ_rectify: void | all updated (Coccinelle + by hand) |
| agc_svc_set_extvbact removed (agc_erase_set_extvbact) | 3 |
| agc_vec_words_t in core/word.h; agc_vec_to/from_words, agc_mpac_vdot/vxm/mxv/vxv/vxsc/vad/vshift | guidance S40.2,3, nav-integ, planetary |
| agc_imu_hw_set_cducmd: SP range | NEEDLES |
| agc_dsky_display_ecadr (MPAC+2 of the display request) | P27, V91 |
| agc_planet_r_to_rp_words / agc_planet_rp_to_r_words | nav-integ OBLATE |
| agc_integ_intwakeu(resume); nav-integ renames (docs/history/RENAMES.md) | P27, registry |
| agc_tff_t.tffalfa: agc_dp_words_t | free_fall.c, tests/oracle/nav_aux.c |
| agc_lm_asc_t tgo, tgo1, tbup, geff, dzdot, dydot, drdot, pcons, ycons: agc_dp_words_t | ascent.c, aborts.c, tests/oracle/landing.c |
| pair types lm_dp_pair_t, agc_lm_words_t, word_pair_t removed (agc_dp_words_t, agc_dp_words_das) | lm dap, landing, imu |

## Oracle tolerance changes

| oracle | case / field | before | after |
|---|---|---|---|
| guidance | s40_2_sps, s40_2_sps_b | 2 | 0 |
| imu | delcomp_a/b/c | 4 | 0 |
| imu | dcmtocdu_a/b/c | 3 | 0 |
| imu | kalc normal/large/neg/small | 4/1/4/2 | 0/0/1/1 |
| nav_aux | latlong_moon_rm / _rls | 256 | 0 |
| nav_aux | rptor_moon / rtorp_moon | 4 | 0 |
| entry | pass tol / D / THETAH / ROLLC / L/D / VL | 4 / 256 / 8 / 16 / 320 / 64 | 1 / 160 / 4 / 8 / 256 / 40 |
| targeting | initvel_moon | 32 | 0 |
| targeting | initvel_earth_prec | 3400 | 1 |
| targeting | cdh_earth / cdh_earth_ell | 520 / 760 | 2 / 4 |
| targeting | tpi_earth_b | 760 | 0 |
| targeting | cdh DIFFALT | 32 | 0 |
| nav_aux | latlong_earth_* | 2 | 0 |
| nav_aux | lalotorv_* | 4 | 0 |
| nav_aux | tff_* (TFF, TFFX) | 12 (4, 32) | 0 (0, 0) |
| nav_aux | lspos_lm_* | 2 | 0 |
| landing | ascent AT / TTOGO / PRATE, YRATE / PCONS, YCONS / ATR..PITCH / UNFC/2 | 4 / 1 / 32 / 4 / 8 / 8 | 0 (+ high-word checks of TBUP TGO PCONS YCONS) |
| entry | general (startent / passes) | 4 / 1 | 0 |
| entry | VL / L/D / L/D1 / THETAH / ROLLC | 40 / 256 / 256 / 4 / 8 | 0 / 9 / 8 / 3 / 6 |
| return_to_earth | KNOWN F14 | 52 | 47 |

## Lessons

- A residual of a few LSB in a chain of interpretive DP ops is nearly always a stored word pair whose signs disagree (DAD/DSU/BDSU, DOT or DMP top words) read by a later DMP, DDV, SR, SQRT or ASIN. Converting the whole chain to agc_mpac_* (inputs to_words, outputs from_words) and then testing at tolerance 0 found every one in nav-aux, ascent and HUNTEST1; converting a chain that does not touch such words changed nothing (UPCNTRL3: reverted).
- Words must be kept where they are STORED, not only inside one routine: TFFALFA (TFFCONIC -> CALCTFF), TBUP/TGO (ATMAG -> MAINENG -> T2TEST), PCONS/YCONS (CONST -> CMPONENT next pass). A local conversion that canonicalises at a store loses exactly the bits the next routine needs.
- Half a conversion can be worse than none: DOWNCNTL's DREF as words gave L/D 256 off because D (stored by DSTORE from a DMP) was still a value; keep both operands of a DSU at the same representation.
- Debug a step sequencer by printing the step ids to stdout (same stream as the oracle's FAIL lines), not stderr: the interleaving then shows which path a case takes.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ A tree-wide pass (routed review items (DMP audit, word-pair API)): it changed code owned by the translation lanes without changing what the program decides, so it has no program flow of its own. The flows as the code now stands are in the lane sections: [exec-sched](../../modules/exec-sched.md#program-flow-and-decisions), [core-numeric](../../modules/core-numeric.md#program-flow-and-decisions), [state-flags](../../modules/state-flags.md#program-flow-and-decisions), [nav-conic](../../modules/nav-conic.md#program-flow-and-decisions), [nav-integ](../../modules/nav-integ.md#program-flow-and-decisions), [nav-aux](../../modules/nav-aux.md#program-flow-and-decisions), [io-pinball](../../modules/io-pinball.md#program-flow-and-decisions), [io-nouns](../../modules/io-nouns.md#program-flow-and-decisions), [imu](../../modules/imu.md#program-flow-and-decisions), [guid-powered](../../modules/guid-powered.md#program-flow-and-decisions), [guid-landing](../../modules/guid-landing.md#program-flow-and-decisions), [guid-entry](../../modules/guid-entry.md#program-flow-and-decisions), [p37](../../modules/p37.md#program-flow-and-decisions), [dap-lm](../../modules/dap-lm.md#program-flow-and-decisions), [dap-csm](../../modules/dap-csm.md#program-flow-and-decisions), [prog-rendz](../../modules/prog-rendz.md#program-flow-and-decisions), [prog-align](../../modules/prog-align.md#program-flow-and-decisions), [sys-misc](../../modules/sys-misc.md#program-flow-and-decisions), [sim](../../modules/sim.md#program-flow-and-decisions); the system view is [docs/overview/README.md](../../overview/README.md) and its flowcharts.
