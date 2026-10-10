# sys-misc -- module note

## Scope

T4RUPT_PROGRAM, EXTENDED_VERBS, DOWNLINK_LISTS, DOWN_TELEMETRY_PROGRAM,
UPDATE_PROGRAM (P27), SERVICE_ROUTINES, AGC_BLOCK_TWO_SELF_CHECK,
SYSTEM_TEST_STANDARD_LEAD_INS, IMU_PERFORMANCE_TEST_2 / _TESTS_4 (LM P07),
INTER-BANK_COMMUNICATION, RTB_OP_CODES (LM) / RT8_OP_CODES (CM).

Public API of this lane: `include/agc/io/service_internal.h`.  Symbols owned
by other lanes / not yet in include/agc are declared in
`src/io/service/sysmisc_ext.h` (service/T4RUPT side) and
`src/io/verbs/pending.h` (verbs side, with the provisional job ids); see
"Requested core additions" / "Requested jobs-tasks".

## Design table

| AGC routine (file, pages) | Inputs / outputs, scale | C function | file |
|---|---|---|---|
| LOADTIME (RTB p.1397) | -> TIME2,TIME1 cs, DP 2^28 cs | agc_rtb_loadtime | src/io/service/rtb_ops.c |
| CDULOGIC (p.1397) | SP 2s-comp CDU half-revs -> DP revs | agc_rtb_cdulogic | rtb_ops.c |
| SGNAGREE/TPAGREE (p.1397) | MPAC word triple -> sign-agreed triple, signum | agc_rtb_sgnagree, _dp | rtb_ops.c |
| 1STO2S, V1STO2S, 2V1STO2S, 1TO2SUB (p.1397-98) | DP revs -> SP 2s-comp half-revs | agc_rtb_1sto2s / v1sto2s / 2v1sto2s | rtb_ops.c |
| CDUINC (p.1398-99), INCRCDUS (CM) | 2s-comp angle + 1s-comp SP inc | agc_rtb_cduinc / incrcdus | rtb_ops.c |
| READPIPS (CM) | PIPAX/Y/Z -> vector hi words | agc_rtb_readpips | rtb_ops.c |
| PULSEIMU (p.1400) | gyro cmds -> IMUPULSE | agc_rtb_pulseimu | rtb_ops.c |
| TRANSP1/TRANSP2 (CM) | XNB/XNB1 in-place transpose | agc_rtb_transp | rtb_ops.c |
| SIGNMPAC (p.1401) | DP -> +-DPOSMAX | agc_rtb_signmpac | rtb_ops.c |
| NORMUNIT/NORMUNX1 (p.1401-02) | vector -> unit, shift 13/0 | agc_rtb_normunit | rtb_ops.c |
| VECSGNAG (p.1402) | identity in this number model | agc_rtb_vecsgnag | rtb_ops.c |
| QUALITY1/QUALITY2 (RT8, CM) | URPV B1, E3J22R2M B58 -> J22 term B61 | agc_rtb_quality1/2 | rtb_ops.c |
| CM downlists (pp.170-180) | static const entry tables, 5 lists | agc_dnlist_csm_control/sublist | src/io/telemetry/downlink_lists_csm.c |
| LM downlists | 6 lists | agc_dnlist_lm_control/sublist | src/io/telemetry/downlink_lists_lm.c |
| word count | list -> 100 DP words | agc_dnlist_word_count | src/io/telemetry/down_telemetry.c |
| DODOWNTM, DNPHASE1/2, NEWLIST, SENDID, FETCH2WD, DODNPTR, SNAPLOOP, DODNCHAN, WOZERO, WOTEST (pp.988-997) | per DOWNRUPT write DNTM1/DNTM2 (ch 34/35), CHAN13 bit 7 word order | agc_telem_downrupt + statics | src/io/telemetry/down_telemetry.c |
| DNDUMPI/DNDUMP/DNDUMP1/DNDUMP2 (p.995-996) | erasable dump via V74 | statics | down_telemetry.c |
| UPFLAG/DOWNFLAG/DEBIT, UPENT2/DOWNENT2 (pp.1485-87) | flag id -> flagword | agc_svc_upflag/downflag/upent2/downent2 | src/io/service/service.c |
| DELAYJOB/WAKER (p.1488) | delay cs, job sleep/wake, BAILOUT 1104 | agc_svc_delayjob/delay_wake | service.c |
| GENTRAN (p.1490) | block copy | agc_svc_gentran | service.c |
| B5OFF, TRFAILOF/ON (p.1491-92) | EXTVBACT bit 5; DSPTAB+11D bits | agc_svc_b5off/trfailof/trfailon | service.c |
| T4RUPT, CDRVE, DSPOUT, DSPOUTSB, QUIKDSP, HANG20, PROCEEDE, T4JUMP | DSPTAB -> ch 10 relay words; TIME4 reload | agc_t4rupt + statics | src/io/service/t4rupt.c |
| IMUMON, TNONTEST, PROCTNON, ENDTNON, UNZ2, ISSUP, OPONLY, ISSZERO, C33TEST, GLOCKMON, TLIM, ITURNON, IMUCAGE, IMUOP, PIPFAIL, DNTMFAST, UPTMFAST, SETISSW, CAGESUB(1,2), LAMPTEST | IMODES30/33, ch 11/12/14/30/33, CDUZ | statics + agc_t4_* | t4rupt.c |
| OPTMON, OPTDRIVE, ZOPT logic, ENDZOPT, OCDUFTST (CM) | OPTMODES, OPTIND, CDUS/T, ch 12/14 | statics | src/io/service/t4rupt_csm.c |
| RRAUTCHK, RRCDUCHK, RRGIMON, GPMATRIX/DAPT4S (LM) | RADMODES, M11..M32 SP scaled 1 | statics | src/io/service/t4rupt_lm.c |
| GOEXTVB/LST2FAN, TESTXACT, XACT0, ALM/END, GOPIN, CHKPOOH, CKMODCAD (shared) | verb 40-99 dispatch | agc_extverb + statics | src/io/verbs/extended_verbs.c |
| vehicle verbs | see Progress | statics + agc_job_* | src/io/verbs/extended_verbs_csm.c / _lm.c |
| LM radar verbs RRZEROK, RRDESNBK, AURLOKON, RRDESK2, RRDESEND, LRPOS2K, R04, R77, R77END, VERB85/DSPRRLOS/RRLOSDSP, RDRUSECK (L pp.265-280, 298-300) | RADMODES, OPTIONX, RSAMPDT.., CDUT/CDUS -> RR-AZ/RR-ELEV revs | agc_extverb_rr_* / lr_* / radar_test, agc_job_rr_* / radar_test_* | src/io/verbs/extended_verbs_lm_radar.c |
| OPTCOARK / OPTC1 (CM V41 N91, C p.242) | SAC,PAC -> DESOPTS,DESOPTT; OPTIND | verb_optics_coarse, agc_job_optics_coarse_drive/start | extended_verbs_csm.c |
| V70-V73 UPDATE PROGRAM P27 (pp. Comanche UPDATE_PROGRAM) | UPBUFF etc. | agc_uplink_update_* | src/io/verbs/uplink_update.c |
| SELF-CHECK SMODECHK/ROPECHK/ADSUM/ADRSCHK/NXTBNK/SOPTION, SHOWSUM | rope image -> bank sums | agc_selfcheck_* | src/io/service/self_check.c |
| P07 REDO/ZEROING/FINETIME/OVERFFIX/ERTHRVSE/EARTHR/ESTIMS (LM) | | agc_systems_test_* | src/lm/systems_test.c |
| BANKCALL/SWCALL/POSTJUMP..., E/BKCALL, E/CALL, E/JOBWAK | vanish | comment file | src/io/service/interbank.c |

## Progress

- DONE  RTB op codes (all LM + CM extras incl. QUALITY1/2)  src/io/service/rtb_ops.c
- DONE  CM downlink lists  agc_dnlist_csm_control/sublist  src/io/telemetry/downlink_lists_csm.c
- DONE  LM downlink lists  agc_dnlist_lm_control/sublist  src/io/telemetry/downlink_lists_lm.c
- DONE  down-telemetry DODOWNTM..DNDUMP, word count  src/io/telemetry/down_telemetry.c
- DONE  service.c (UPFLAG..TRFAILON, DELAYJOB/WAKER, GENTRAN, B5OFF)  src/io/service/service.c
- DONE  t4rupt.c / t4rupt_csm.c / t4rupt_lm.c; T4JUMP calls agc_lm_dap_rcsmon() at phases 0 and 4
        (RCSMONIT every 480 ms; it exits via RESUME, so nothing else runs in that slot);
        agc_erase_m11/m21/m22/m31/m32/cosmg defined in t4rupt_lm.c, declared in
        include/agc/io/service_internal.h (dap-lm can drop them from src/lm/dap/pending.h)
- DONE  extended verbs, shared fan + V40/41 N20, V42, V49, V50/97/99, V55, V64, V67, V69, V74, V82, V89, V90, V91  src/io/verbs/extended_verbs.c
- DONE  extended verbs CSM, incl. V41 N91 OPTCOARK  src/io/verbs/extended_verbs_csm.c
- DONE  extended verbs LM  src/io/verbs/extended_verbs_lm.c
- DONE  LM radar verbs V40/41 N72, V44, V60, V63 R04, V78/V79 R77, V85  src/io/verbs/extended_verbs_lm_radar.c
- DONE  P27 V70-V73 (UPPART2..UPOUT, TIMEDIDL/R, UPEND70-73)  src/io/verbs/uplink_update.c
- DONE  self-check / show-banksum  src/io/service/self_check.c
- PARTIAL LM P07 systems test: ZEROING, FINETIME, OVERFFIX done; REDO stub  src/lm/systems_test.c
- DONE  interbank.c (comment file)  src/io/service/interbank.c
- DONE  tests/test_sysmisc.c (271 checks pass against a scratch build of the tree with link
        stubs for other lanes' missing symbols; the tree does not link yet)

## Deviations

- SGNAGREE: AGC lets the high word overflow when |V|>=1; C saturates at POSMAX.
- NORMUNIT keeps the AGC's "DAS gains a little accuracy" factor (2^13+1).
- CDU angles are uint16_t 15-bit two's-complement patterns throughout.
- AURLOKON's MAKECADR/DESRET return and RRDESK2's hand release of its VAC area become a
  continuation job (agc_job_rr_lockon_answer runs the rest of RRDESNBK); VAC bookkeeping
  vanishes.
- RRDESEND holds INHINT into DELAYJOB in the AGC; the C releases interrupts first
  (agc_svc_delayjob inhibits them itself).
- RRLOSDSP's DAD DPPOSMAX for negative angles keeps the AGC's 2^-28 rev error.

## Stubs

- `agc_job_systems_test` (LM REDO / P07 IMU performance tests, IMU_PERFORMANCE_TEST_2 and
  _TESTS_4): sets MM 07, returns AGC_STATUS_UNIMPLEMENTED.  Not translatable whole: the
  drift filter's calculation (PERFERAS) jumps into an erasable-memory program
  ("TC ATIGINC -- GOTO ERASABLE TO CALCULATE") that must be loaded before the test and is
  not in the rope.  ERTHRVSE/EARTHR are the CM prelaunch ones (agc_csm_prelaunch_*).

## Requested core additions

Declared (extern, not defined) in src/io/service/sysmisc_ext.h or src/io/verbs/pending.h;
names follow docs/agents/_common.md.  Array erasables take an index argument.
- prog-rendz (P20-P25.agc): `bool agc_rr_limchk(uint16_t cdut, uint16_t cdus)` (RRLIMCHK),
  `agc_status_t agc_rr_settrkf(void)` (SETTRKF), `agc_status_t agc_rr_zero(void)`
  (RRZERO; prog-rendz has it in rr_moding_lm.c), `agc_status_t agc_rr_designate_gimbal(bool *started)`
  (RRDESNB, angles in TANG; *started false when RRDESNB ended the job, e.g. alarm 502),
  `agc_status_t agc_rendz_clradmod(void)` (CLRADMOD; same declaration as the landing shim),
  `agc_status_t agc_radar_lrpos2(void)` (LRPOS2; same as the powered-flight shim),
  task `AGC_TASK_RADSAMP` (0x1103, value from src/programs/rendezvous/ext.h).
  RADSAMP/DORSAMP should use this lane's R04/R77 storage accessors
  agc_erase_rsampdt/rtstdex/rtstbase/rtstmax/rtstloc/rfailcnt (service_internal.h).
- prog-align / io-pinball: `agc_sp_t agc_erase_optionx(int index)`,
  `void agc_erase_set_optionx(int index, agc_sp_t v)` (as alignment_pending.h),
  `void agc_erase_set_dsptem1(int index, agc_sp_t v)` (R04 checklist code 201).
- io-nouns: `void agc_erase_set_rr_az(agc_dp_t v)`, `void agc_erase_set_rr_elev(agc_dp_t v)`
  (N56, revolutions; RR-AZ shares storage with PITCHANG).
- state-flags: `agc_status_t agc_erase_write_ecadr(uint16_t ecadr, uint16_t word)` (V71/V72).
- prog-rendz: `void agc_erase_set_landmark(agc_sp_t v)` (CM V52).
- imu: `agc_status_t agc_imu_imupulse(agc_vec_t gyro_cmds)` was requested earlier; RTB
  PULSEIMU now uses agc_imu_pulse_buf.

## Tests written

tests/test_sysmisc.c: every downlist (CM 5, LM 6) is 100 DP words; DOWNRUPT sequence of
the CM coast-and-align list (ID/sync, TIME2/TIME1, next list, UPDNLIST); LOADTIME;
SGNAGREE / TPAGREE sign fix-up; ROPECHK/SHOWSUM bank sums; UPFLAG/DOWNFLAG, GENTRAN,
B5OFF/EXTVBACT; LM T4RUPT fills M11..M32 from CDUZ/CDUX; V44 RRDESEND outside continuous
designate.  271 checks pass (linked against a scratch build plus link stubs).

## Open questions

- FORMDEG named in the assignment does not exist in Comanche055 or
  Luminary099.
- R04/R77 storage (RSAMPDT, RTSTDEX ...) is defined here; prog-rendz's RADSAMP must use
  the accessors rather than define its own (RTSTDEX shares a word with LMPOS).

## Lessons

- AGC_ASSERTs inside libagc.a never reach a test's agc_assert_report: the Makefile builds
  the library without AGC_HOST_DEBUG.  Tests can count only their own assert reports.
- "TC RCSMONIT" in the LM T4JUMP table looks like a call that falls into the next entry,
  but RCSMON exits through RCSMONEX = RESUME, so each table slot is one activity.
- TC WAITLIST takes the delay from A: in R04 the A left over from "TS RTSTMAX" (2 or 6)
  is the first RADSAMP delay.  Watch for registers reused as arguments like this.
- GOXDSPF's returns are +1 V34, +2 V33, +3 ENTER; RRDESNBK's "TCF -4" on V33 re-flashes
  the load, so V33 and ENTER do different things there.
- An R-kind display (GOMARKFR) returns at once and the job then blankets and ends; the
  answers run as new jobs, so a MAKECADR/DESRET return (AURLOKON) becomes a continuation
  job that runs the rest of the caller.
- Some ground-test code (LM P07) jumps into erasable-memory programs that are not in the
  rope; those parts cannot be translated and must be documented as stubs.
- In a unit test, the first LM T4RUPT passes see the ISS turn on and zero the ICDUs;
  settle T4RUPT before setting CDUs you want GPMATRIX to see.

