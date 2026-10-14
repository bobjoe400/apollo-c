# review-sys-misc -- lane history

Process diary of the review-sys-misc lane, moved verbatim out of [docs/modules/review-sys-misc.md](../../modules/review-sys-misc.md), which keeps the reader-facing explanation. Note dates: 2026-10-10 to 2026-10-14 (first and last commit touching the note).

## Progress
(Per file "DONE" = AGC fidelity checked side by side, check_style clean,
check_rules clean, compiles, sysmisc test passes. Resumed 2026-10-11: the
density criterion is obsolete (rule 5 by spirit); the earlier DONE files
still need the status-handling-2 diff check and a sweep for assertions
that cannot fail (assert-after-assign, vehicle asserts on vehicle-only
paths) -- tracked as "S-H2" below.)
ALL ITEMS DONE 2026-10-12: make, make test, make scenarios (8/8), check_rules
0 hard, format-check clean for every lane file (other lanes' in-progress
files excepted).
* DONE t4rupt_lm.c
* DONE t4rupt.c
* DONE t4rupt_csm.c (agc_t4_optics_cmd wrapper removed from it and service.h)
* DONE rtb_ops.c (1TO2SUB/1STO2S, CDUINC, CDULOGIC checked against the rope
  in yaAGC: 33 cases bit-exact; spec in SCRATCH/review-sys-misc/rtb*.spec)
* DONE service.c
* DONE self_check.c (fidelity pass 2: one bank per idle pass, SCOUNT
  counting, -0/- option error paths, set_rope validates, void resets;
  real Comanche055/Luminary099 ropes pass ROPECHK in test_sysmisc;
  idle hook in src/sim/clock.c; STARTSB2 SELFRET via exec_up_startsb2)
* DONE N08 ALMCADR orphans (exec/nouns.c: R1 agc_alarm_almcadr job,
  R2 +0 DEVIATION, R3 ERCOUNT; duplicate ERCOUNT in fresh_start.c removed)
* DONE STARTSUB delegations wired: exec_up_startsub (DNTMGOTO, LM
  RADMODES = ch33 bit6 + RMODINIT), exec_up_goprog_downlist (LM
  DNLSTCOD = AGSWORD); LM DUMPCNT moved to SLAP1 (agc_telem_slap1)
* DONE interbank.c (file prolog; style clean)
* DONE down_telemetry.c fidelity (2026-10-11 resume): DOFSTART part
  (agc_telem_fresh_start(void): DNLSTCOD=0, LM AGSWORD=0), agc_telem_slap1
  (LM DUMPCNT=BIT12, was wrongly on every DOFSTART), agc_telem_restart
  void. Verified vs rope: DODNCHAN DNTM1 = ch n, DNTM2 = ch n+1 (oracle
  run, SCRATCH/review-sys-misc/dnchan.spec); NEWLIST ID, SENDID, snapshot
  (last entry sent now, rest to DNTMBUFF), FETCH2WD, TIME2 word-order test
  only for control entries, ERASID CSM 1777 / LM 1776: all match.
  RADMODES,DAPBOOLS = FLGWRD12,13 -> RADMODES one word DONE. DNDUMP2 now
  sends the register pair read by address (agc_erase_read_ecadr(DUMPLOC &
  3777), +1) instead of 0,0 (test_erasable_dump). Cannot-fail asserts
  deleted (fetch2wd dn_left, dnphase1 !active, dndump_id LOW8, start_dump,
  &sub, downrupt dn_left range); list-table constraints (no DNCHAN 0, no
  nested DNPTR, snapshot 1DNADRs fit DNTMBUFF) moved from flight asserts to
  test_downlist_tables; DODNPTR/DOSUBLST re-joined to the common SETWO +2
  coding (no duplicate send code, inline_one debris gone); list_lookup
  status propagated plainly. Dead API removed: agc_telem_list_code,
  agc_telem_set_dumpcnt, agc_telem_dumpcnt (no callers); start_dump void
  (V74 caller updated); set_agsword plain store (callers pass 0 or the
  SEUDOP00 table code). Style done (see below).
* DONE downlink_lists_csm.c, downlink_lists_lm.c: every list and sublist
  compared entry by entry with the AGC source (SCRATCH/review-sys-misc/
  dlcmp.py: op, sign, item, offset, EQUALS aliases resolved): 0
  mismatches in both; DNTABLE order checked. CMPG2205 sublist renamed
  cm_landmark_snapshot (was cm_orbital_nav_list05, digit suffix);
  snapshot flag == first entry negated now checked in test_downlist_tables;
  every row commented (formatter misaligned uncommented rows), one-line
  table rows, prologs; style clean.
* DONE extended_verbs*.c, uplink_update.c (2026-10-11): rule pass done
  (ctx asserts that hid a NULL behind OK removed and the now-unused ctx
  parameters dropped from 20 jobs/helpers, registry dispatch regenerated;
  assert-wrapped callee statuses propagated; INTSTALL non-BUSY failures
  no longer lost in ATTACHIT/LMTOCM, TRACKTRM (both), UPJOB; cannot-fail
  asserts deleted; junk `st = OK` plumbing removed). Fidelity fixes:
  F-V1 LM DAPDAT2 MINLMD/MINMINLM were raw kg (-2850/-2200) against
  LEMMASS words at 2^16 kg: rope words -713/-550 (an 8000 kg LM was
  refused); F-V2 V73 never copied UPBUFF to UPBUFF +8D (UPEND73 DCA/DXCH),
  so the octal clock increment added 0; F-V3 V71/V72 stores now reach
  ECADR & 3777 (TS EBANK + MASK LOW8) instead of the raw word; F-V4
  ATTCK1/ATTCK2 read THETAD as ones'-complement SP (MP / AK) instead of
  two's complement. Verified vs source: LST2FAN both ropes, TESTXACT,
  CHKPOOH, CKMODCAD, VBZERO/IMUZEROK, ICORK2, IMUFINEK/FINEK2, ALINTIME/
  UPDATIME, ATTACHIT/LMTOCM, TRACKTRM/TRMTRACK/TRMTRAK1, OPTCOARK, DAPDISP
  (both), STABLISH, SYSTEST (both), CKOPTVB/CHAZFOGC, VERB94/96 (both),
  V52, V60-V63, GOTOR21/23, V83/V85/V86, IMUATTCK (both), DPDAT1/DAPDAT2,
  MINIMP/NOMINIMP, LEMVEC/CSMVEC, WMATRXNG, RDRUSECK, R04/R77 (R04Z..
  R77END), AURLOKON, RRDESEND, RRLOSDSP, P27 (V70-V73 all paths).
  Tests: test_uplink_clock_increment, test_dap_data_mass. Style done.
* DONE never-fail sweep (S-H2) in the earlier DONE files: t4rupt_lm.c
  (GPMATRIX vehicle assert + matrix_consistent quota check gone,
  agc_t4_gimbal_matrix void; RRAUTCHK vehicle/post-XOR asserts gone),
  t4rupt_csm.c (optics_ok quota check gone; SETDESMD/MANTOCSC/CANCOARS
  void; ZOPTDES rewritten flat: SETZOEND's WAITLIST FULL/BAILOUT was
  dropped by a `!= BAD_ARG` assert, now propagated; OPTDRIVE void; set_optind
  / set_optmodes plain stores; a wrong "differs" comment on the stop
  avoidance fixed), t4rupt.c (phase/vehicle/sequence/TIME4 asserts that
  cannot fail gone; three inline_one prolog blocks rewritten as section
  comments), service.c (UPENT2/DOWNENT2/GENTRAN void, DELAYJOB returns
  BAILOUT plainly, WAKER/B5OFF/TRFAILOF/TRFAILON post- and vehicle-asserts
  gone), rtb_ops.c (2V1STO2S/INCRCDUS/TRANSP void, no callers; PULSEIMU
  returns IMUPULSE's status), self_check.c (banksum NULL check). S-H2 diff
  (1b72892..a02850c) and manual.tsv GUARANTEED rows of the lane: checked,
  all hold; no BAD_ARG-replaces-callee-status sites remain in the lane.
* Cross-lane edits made (callers of changed lane API): src/exec/
  {alarm.c (agc_alarm_almcadr), nouns.c (N08), fresh_start.c (STARTSUB
  hook, dup ERCOUNT removed), change_program/idle.c (exec_up_startsub,
  exec_up_goprog_downlist, void resets), internal.h}, include/agc/exec/
  exec.h, src/io/dsky/nouns/data.c (set_smode void), src/sim/clock.c
  (idle self-check hook), tests/io/nouns.c (N08 now resolves).
  NOTE: one `tools/format.sh` call also ran over review-prog-align's
  modified files (whitespace-only formatter, fixed point).
* Requests (2026-10-11 resume):
  - guid-powered: src/csm/burns/sps/clock.c:464 stores OPTIND = -1 where
    the AGC does CS ZERO (OPTIND = -0, "optics reserved" for TVC);
    use AGC_OPTIND_NEG0, else V41 N91 never gives alarm 117.
  - follow-up (callers in other lanes): agc_svc_b5off, agc_svc_trfailof,
    agc_svc_trfailon can no longer fail (always OK); make them void and
    drop the status plumbing in csm/optics/s_band_antenna.c,
    programs/displays/orbit_parameter_display.c, csm/navigation/
    tracking_data.c. Also fold agc_svc_set_extvbact (csm/entry/programs.c,
    programs/displays/rendezvous_parameter_display.c) into
    agc_erase_set_extvbact (same store).
  - io-dsky / state-flags: P27 cannot receive data end to end: nothing
    calls agc_uplink_update_write (the V21N01/N02 load into the UPBUFF
    index agc_uplink_update_target() names, the AGC's MPAC+2). Likewise a
    V21N01 load of DUMPCNT or UPSVFLAG reaches only the ecadr.c image.
  - sim: drive the scaler channels 3/4 (HISCALAR/LOSCALAR) from the
    clock; FINETIME (P07 CHECKG) reads them, so the PIPA rate is the DDV
    overflow value in the sim today.
  - dispatch.c regenerated by this lane (20 jobs lost their unused ctx);
    the generator also picked up two void jobs of review-prog-align's
    in-progress headers (cislunar_midcourse_navigation_sight,
    auto_optics_sighting).
* Requests: io-dsky: display requests carry no MPAC+2 (MPAC2SAV/MARK2PAC),
  so SHOW-BANKSUM's V05N01 at SKEEP1 cannot show SKEEP1-3
  (agc_selfcheck_showsum_words holds them). sim: register the rope image
  (agc_selfcheck_set_rope; yaYUL .bin order 2,3,0,1,4.., word>>1) so
  ROPECHK/SHOW-BANKSUM run on real fixed memory.
* DONE systems_test.c: LM P07 translated (was a stub): REDO, GEOIMUTT/
  IMUZERR, IMUBACK, NBPOSPL, GUESS/LATAZCHK (V06N41), POSGMBL (CALCGA,
  IMUCOARS, gimbal lock count), FINIMUDD, GOESTIMS, ESTIMS, ALLOOP,
  SPECSTS, ALFLT (NORMLOP, ALKCG, DELMLP, ALILP, ALKLP, LOOSE, BOOP),
  PERFERAS, SETUPER1, GEOSTRT4, TORQUE, VALMIS, SHOW, PIPACHK, CHECKG,
  PIPATASK, PIPJOBB/AINGOTN, COAALIGN, VERTDRFT/PON2/PON4/PON, GUESS1,
  ERTHRVSE, EARTHR, EARTHR*, SLEEPIE, ENDTEST1, SOMERR2. What the rope
  lacks (documented in the file prolog): the erasable program at ATIGINC
  (E7,1400) PERFERAS calls, and the ground-loaded words (TRANSM1/ALFDK
  gains at E4,1400, PERFDLAY, 1SECXT, LENGTHOT, POSITON, PIPINDEX).
  Test erasables (W .. AINLA +99D) kept as AGC words at their ECADRs (the
  code depends on the overlays) so V21 N01 reaches them; QPLACE/QPLACES
  hold step numbers. ALFLT and AINGOTN are word for word against yaAGC
  (tools/oracle/cases/systems_test_lm.{spec,txt}, generator gen/
  gen_systems_test_lm.py; 416 words, replayed by test_systems_test_oracle;
  only -0 words differ, the word API has no -0). End-to-end: new scenario
  sim/scenarios/p07_systems_test_lm.txt (V92, V06N41, drift test, PIPA
  test, V34). 17 jobs + 3 tasks added to tools/registry.tsv (group lm).
* DONE DMP/DMPR audit: one site changed (rtb_ops.c QUALITY2 DMP 5/8 was
  DMPR, now agc_dp_mul_trunc); every other multiply is MP or DMPR as in
  the listing; P07's filter uses the word-level API (interp_words.h).
* DONE style: every lane source file and service.h at 0 check_style
  findings (prologs, declaration / ternary / define comments, closing-
  brace comments; DNTMEXIT, SENDID, DNDUMPI+2, FETCH2WD void; nested ?:
  replaced; one-liners cdu_ones / iabs cite OZONE/CMDSETUP, word_sp
  inlined: PLAIN 0). TODO: tests/io/service_verbs_telemetry.c style.
* DONE tests/io/service_verbs_telemetry.c: links the real library (no stand-ins), no
  assertion-report counting (negative tests judged by status); style TODO
* No file is mid-edit; every touched file compiles (-Werror) and is clean.
* Tools (scratch): prolog.py / declcmt.py / tercmt.py / agcnone.py /
  voidctx.py / dlcmp.py / linecmt.py / fix80.py / braces.py /
  voidassert.py / fprolog.py / density.py / chk.sh / t.sh in
  SCRATCH/review-sys-misc/. Note check_rules counts assertion *lines*
  and treats "(void) AGC_ASSERT" as a rule-7 hard violation.

## Findings
| # | file / routine | finding | fix |
|---|---|---|---|
| S1 | t4rupt.c t4_jump (LM T4JUMP p.159) | integration had made phase 0/4 run RCSMONIT *then* RRAUTCHK+GPMATRIX; RCSMON exits through RCSMONEX = RESUME on every path (RCS_FAILURE_MONITOR p.190), the TC never returns | slot 0 runs agc_lm_dap_rcsmon() only |
| S2 | t4rupt.c CDRVE p.134 | DSPTAB+11D = -0: CCS skips the XCH and sends/stores +0 lamps; C sent LOW11 of 077777 (all 11 lamps) | -0 sends 0 |
| S3 | t4rupt.c QUIKDSP p.137 | AGC tests NOUT only; agc_dsky_dspout also tests DSKYFLAG | DEVIATION comment (DSPOUTSB is pinball's) |
| S4 | t4rupt.c C33TEST | WAND CHAN33 was written as a no-op AND with 077777 | agc_chan_and(CHAN33, 33RDMSK) as in the AGC (the channel model resets the flip-flops on any write) |
| S5 | t4rupt_lm.c | RRAUTCHK/RRCDUCHK/RRGIMON/MONREPOS/GPMATRIX checked line by line (OCT05776, OCT10001, OCT32002, OCT20002, RCDUFBIT, REPOSBIT, NORRMBIT verified in the rope) | confirmed; M11..COSMG accessors now assert the matrix invariants |
| S6 | t4rupt.c | CDRVE, DSPOUT, HANG20, QUIKDSP/QUIKOFF/NODSPY/SYNCT4, PROCEEDE, IMUMON/NXTIFBIT, TLIM, ITURNON(2), IMUCAGE, IMUOP(2), PIPFAIL, DNTMFAST, UPTMFAST, SETISSW, CAGESUB(1,2), TNONTEST, PROCTNON, OPONLY, ISSZERO, ENDTNON(2), RETNON, UNZ2, ISSUP, C33TEST, GLOCKMON/SETGLOCK/GLINVERT, LAMPTEST: both ropes diffed, constants -70DEGS -15DEGS 4SECS 90SECS 30RDMSK checked | confirmed |
| S7 | t4rupt_csm.c (C pp.160-169) | OPTMON/33OPTMON/SETSAMP/PROCESSW, ZOPTDES/ZTOMAN/ZOPALARM/ZTOCSC, ZOPFINI, CANZOPT, MANUDES/MANTOMAN/OPTZERO, MANTOCSC/COARSLOK, CSCDES/CSCTOZOP/CSCTOMAN, CANCOARS, SETZOEND, ENDZOPT/ZEROPCDU, INITZOPT, OCDUFTST/OPFAILON/OPFAILOF/SETOFF, OPTDRIVE/OZONE/CONTDRVE, OPT2/GETOPCMD/STORCMD/TRUNCMD, shaft stop avoidance, CMDSETUP/POS/NEGOPCMD/DELOPCMD/NEXTOPT/SENDOCMD, SETBIT; MAXPLS -165, MAXPLS1 -164, 13,14,15 = -4095, NEG1/2, 20DEGS 7199, OPTINITF/R checked in the rope | confirmed. SWSAMPLE is BIT5-1 = 17 octal for CSC in the AGC (only its sign is ever tested); C keeps +1, documented |
| S9 | service.c (C pp.1485-1492, L pp.1374-1380) | UPENT2/DOWNENT2 (both vehicles), UPFLAG/DOWNFLAG/DEBIT, DELAYJOB/DELLOOP/WAKER, GENTRAN, B5OFF, TRFAILOF/TRFAILON diffed and checked | confirmed (WAKER slot by earliest due instead of BBANK: existing deviation) |
| S8 | service.h agc_t4_optics_cmd | forwarding wrapper of agc_erase_cdutcmd, no callers | removed |
| S10 | down_telemetry.c DNDUMP2 | erasable dump sent 0,0 for every register pair | reads the pair by address (ECADR image) |
| S11 | extended_verbs_lm.c DAPDAT2 | MINLMD / MINMINLM were raw kg (-2850 / -2200) against LEMMASS words at 2^16 kg; rope words -713 / -550 (an 8000 kg LM was refused) | rope values; test_dap_data_mass |
| S12 | uplink_update.c UPEND73 | V73 never copied UPBUFF to UPBUFF +8D: the octal clock increment added 0 | DCA/DXCH done; test_uplink_clock_increment |
| S13 | uplink_update.c UPEND71/72 | stores went to the raw ECADR word, not ECADR & 3777 (TS EBANK, MASK LOW8) | masked |
| S14 | ATTCK1 / ATTCK2 | THETAD read as two's complement; MP / AK take it as ones' complement | ones' complement |
| S15 | t4rupt_csm.c SETZOEND | WAITLIST FULL / BAILOUT dropped by a `!= BAD_ARG` assertion | propagated |
| S16 | INTSTALL callers (ATTACHIT, TRACKTRM both, UPJOB) | a non-BUSY INTSTALL failure was lost | propagated |
| S17 | rtb_ops.c QUALITY2 | DMP 5/8 translated as DMPR | agc_dp_mul_trunc |
| S18 | lm/systems_test.c P07 | stub (UNIMPLEMENTED) | translated; ALFLT / AINGOTN word for word vs yaAGC; scenario |

## Lessons
* A guard that "re-checks" a value the same function just stored, or a
  quota-style consistency check of library math (sin^2 + cos^2), hides
  nothing and costs a status nobody can act on; delete it and let the
  function go void when nothing else can fail.
* `if (!AGC_ASSERT(ctx != NULL) || cond) return st;` returns OK on a NULL
  ctx: a silent failure. ctx comes from the dispatcher; drop the check
  (and the parameter when that leaves it unused; gen_registry.py picks up
  (void) jobs from the header).
* Ground-loaded test erasables (P07: PERFDLAY, 1SECXT, TRANSM1/ALFDK)
  are only reachable through V21 N01 if the C keeps them at their ECADR
  (src/core/ecadr.c image); keeping them as AGC words also makes the
  overlays (DATAPL = ANGZ, ALKCG storing ALTIM with ALDK) automatic.
* For word-exact interpretive code: DMPSUB multiplies MPAC by the operand
  (MXV/VXM: vector component x matrix element, not the other way round);
  DOT sums the products with TAD semantics; DLOAD/STORE keep the words as
  held. With that, ALFLT matched yaAGC on all 416 words (only -0 differs).
* Two scaled constants in one listing can have different units: MINCSM
  was already a word (BIT11) while MINLMD/MINMINLM were copied as the
  source's decimal kg; read DEC x B-16 as x/4 words, or take the rope word.

