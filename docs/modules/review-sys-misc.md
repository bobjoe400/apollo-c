# review-sys-misc -- review of the sys-misc lane

Lane files: src/io/service/{rtb_ops,service,t4rupt,t4rupt_csm,t4rupt_lm,
self_check,interbank}.c; src/io/telemetry/{downlink_lists_csm,
downlink_lists_lm,down_telemetry}.c; src/io/verbs/{extended_verbs,
extended_verbs_csm,extended_verbs_lm,extended_verbs_lm_radar,
uplink_update}.c; src/lm/systems_test.c; include/agc/io/service.h (lane
declarations); tests/io/sysmisc.c.
Review items (review_items.md): LM P07 REDO stub; N08 ALMCADR orphans;
RADMODES one word; sysmisc test free of scratch stand-ins.

(Restarted 2026-10-10 after a cutoff: the earlier run left no note.)

## Progress
(Per file "DONE" = AGC fidelity checked side by side, check_style clean,
check_rules clean, compiles, sysmisc test passes. Resumed 2026-10-11: the
density criterion is obsolete (rule 5 by spirit); the earlier DONE files
still need the status-handling-2 diff check and a sweep for assertions
that cannot fail (assert-after-assign, vehicle asserts on vehicle-only
paths) -- tracked as "S-H2" below.)
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
* PAUSED 2026-10-11 (lead: usage limit) IN down_telemetry.c. Done there:
  DOFSTART part (agc_telem_fresh_start(void): DNLSTCOD=0, LM AGSWORD=0),
  agc_telem_slap1 (LM DUMPCNT=BIT12, was wrongly on every DOFSTART),
  agc_telem_restart void. Verified vs rope: DODNCHAN sends DNTM1 = ch n,
  DNTM2 = ch n+1 (oracle run, SCRATCH/review-sys-misc/dnchan.spec); NEWLIST
  ID, SENDID, snapshot (last entry sent now, rest to DNTMBUFF), FETCH2WD,
  TIME2 word-order test only for control entries, ERASID CSM 1777 / LM
  1776: all match. RADMODES,DAPBOOLS k==1 already = FLGWRD13 (DAPBOOLS
  EQUALS FLGWRD13 confirmed) -> RADMODES one word DONE.
  NEXT in down_telemetry.c: (a) dumped words: read agc_erase_read_ecadr(
  DUMPLOC & 03777) and +1 instead of 0,0 (DNDUMP1/DNDUMP2), (b) delete
  cannot-fail asserts (dndump_id LOW8, dnphase1 !active, start_dump,
  `&sub != NULL`), (c) agc_erase_set_agsword silent assert, (d) prologs.
  Then downlink_lists_{csm,lm}.c, extended_verbs*.c, uplink_update.c,
  systems_test.c.
* Rule-5 sweep still owed in the earlier DONE files: t4rupt_lm.c
  (gimbal_matrix LM assert + matrix_consistent quota check, rr_monitor
  post-XOR assert), t4rupt_csm.c optics_ok asserts, set_optind /
  set_upsvflag silent `if (AGC_ASSERT) store` setters.
* Cross-lane edits made (callers of changed lane API): src/exec/
  {alarm.c (agc_alarm_almcadr), nouns.c (N08), fresh_start.c (STARTSUB
  hook, dup ERCOUNT removed), change_program/idle.c (exec_up_startsub,
  exec_up_goprog_downlist, void resets), internal.h}, include/agc/exec/
  exec.h, src/io/dsky/nouns/data.c (set_smode void), src/sim/clock.c
  (idle self-check hook), tests/io/nouns.c (N08 now resolves).
  NOTE: one `tools/format.sh` call also ran over review-prog-align's
  modified files (whitespace-only formatter, fixed point).
* Requests: io-dsky: display requests carry no MPAC+2 (MPAC2SAV/MARK2PAC),
  so SHOW-BANKSUM's V05N01 at SKEEP1 cannot show SKEEP1-3
  (agc_selfcheck_showsum_words holds them). sim: register the rope image
  (agc_selfcheck_set_rope; yaYUL .bin order 2,3,0,1,4.., word>>1) so
  ROPECHK/SHOW-BANKSUM run on real fixed memory.
* TODO down_telemetry.c (check RADMODES,DAPBOOLS entry: k==1 returns 0,
  should be FLGWRD13/DAPBOOLS), downlink_lists_csm.c, downlink_lists_lm.c
* TODO extended_verbs.c, extended_verbs_csm.c, extended_verbs_lm.c,
  extended_verbs_lm_radar.c, uplink_update.c
* TODO systems_test.c: P07 fixed-memory part translated
* TODO tests/io/sysmisc.c free of scratch stand-ins; style for it
* TODO include/agc/io/service.h style (8 findings)
* No file is mid-edit; every touched file compiles (-Werror) and is clean.
* Tools (scratch): prolog.py / linecmt.py / fix80.py / braces.py /
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

## Lessons
