# sys-misc -- module note

## Scope

T4RUPT_PROGRAM, EXTENDED_VERBS, DOWNLINK_LISTS, DOWN_TELEMETRY_PROGRAM,
UPDATE_PROGRAM (P27), SERVICE_ROUTINES, AGC_BLOCK_TWO_SELF_CHECK,
SYSTEM_TEST_STANDARD_LEAD_INS, IMU_PERFORMANCE_TEST_2 / _TESTS_4 (LM P07),
INTER-BANK_COMMUNICATION, RTB_OP_CODES (LM) / RT8_OP_CODES (CM).

Public API of this lane: `src/io/sysmisc.h` (request: promote to
`include/agc/agc_sysmisc.h`).  Symbols owned by other lanes / not yet in
include/agc are declared in `src/io/sysmisc_ext.h` (temporary shim, see
"Requested core additions"); flag-bit shims in `src/io/sysmisc_flags_*.h`.

## Design table

| AGC routine (file, pages) | Inputs / outputs, scale | C function | file |
|---|---|---|---|
| LOADTIME (RTB p.1397) | -> TIME2,TIME1 cs, DP 2^28 cs | agc_rtb_loadtime | src/io/rtb_ops.c |
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
| CM downlists (pp.170-180) | static const entry tables, 5 lists | agc_dnlist_csm_control/sublist | src/csm/downlink_lists_csm.c |
| LM downlists | 6 lists | agc_dnlist_lm_control/sublist | src/lm/downlink_lists_lm.c |
| word count | list -> 100 DP words | agc_dnlist_word_count | src/io/telemetry.c |
| DODOWNTM, DNPHASE1/2, NEWLIST, SENDID, FETCH2WD, DODNPTR, SNAPLOOP, DODNCHAN, WOZERO, WOTEST (pp.988-997) | per DOWNRUPT write DNTM1/DNTM2 (ch 34/35), CHAN13 bit 7 word order | agc_telem_downrupt + statics | src/io/telemetry.c |
| DNDUMPI/DNDUMP/DNDUMP1/DNDUMP2 (p.995-996) | erasable dump via V74 | statics in telemetry.c | telemetry.c |
| UPFLAG/DOWNFLAG/DEBIT, UPENT2/DOWNENT2 (pp.1485-87) | flag id -> flagword | agc_svc_upflag/downflag/upent2/downent2 | src/io/service.c |
| DELAYJOB/WAKER (p.1488) | delay cs, job sleep/wake, BAILOUT 1104 | agc_svc_delayjob/delay_wake | service.c |
| GENTRAN (p.1490) | block copy | agc_svc_gentran | service.c |
| B5OFF, TRFAILOF/ON (p.1491-92) | EXTVBACT bit 5; DSPTAB+11D bits | agc_svc_b5off/trfailof/trfailon | service.c |
| T4RUPT, CDRVE, DSPOUT, DSPOUTSB, QUIKDSP, HANG20, PROCEEDE, T4JUMP | DSPTAB -> ch 10 relay words; TIME4 reload | agc_t4rupt + statics | src/io/t4rupt.c |
| IMUMON, TNONTEST, PROCTNON, ENDTNON, UNZ2, ISSUP, OPONLY, ISSZERO, C33TEST, GLOCKMON, TLIM, ITURNON, IMUCAGE, IMUOP, PIPFAIL, DNTMFAST, UPTMFAST, SETISSW, CAGESUB(1,2), LAMPTEST | IMODES30/33, ch 11/12/14/30/33, CDUZ | statics + agc_t4_* | t4rupt.c |
| OPTMON, OPTDRIVE, ZOPT logic, ENDZOPT, OCDUFTST (CM) | OPTMODES, OPTIND, CDUS/T, ch 12/14 | statics | src/csm/t4rupt_csm.c |
| RRAUTCHK, RRCDUCHK, RRGIMON, GPMATRIX/DAPT4S (LM) | RADMODES, M11..M32 SP scaled 1 | statics | src/lm/t4rupt_lm.c |
| GOEXTVB/LST2FAN, TESTXACT, XACT0, ALM/END, GOPIN, CHKPOOH, CKMODCAD (shared) | verb 40-99 dispatch | agc_extverb + statics | src/io/extended_verbs.c |
| vehicle verbs | see Progress | statics | extended_verbs_csm.c / _lm.c |
| V70-V73 UPDATE PROGRAM P27 (pp. Comanche UPDATE_PROGRAM) | UPBUFF etc. | agc_p27_* | src/io/p27.c |
| SELF-CHECK SMODECHK/ROPECHK/ADSUM/ADRSCHK/NXTBNK/SOPTION, SHOWSUM | rope image -> bank sums | agc_selfcheck_* | src/io/self_check.c |
| P07 REDO/ZEROING/FINETIME/OVERFFIX/ERTHRVSE/EARTHR/ESTIMS (LM) | | agc_p07_* | src/lm/imu_tests.c |
| BANKCALL/SWCALL/POSTJUMP..., E/BKCALL, E/CALL, E/JOBWAK | vanish | comment file | src/io/interbank.c |

## Progress

- DONE  RTB op codes (all LM + CM extras incl. QUALITY1/2)  src/io/rtb_ops.c  (compile pending)
- TODO  downlink lists CM / LM
- TODO  telemetry.c
- TODO  service.c
- TODO  t4rupt.c / t4rupt_csm.c / t4rupt_lm.c
- TODO  p27.c
- TODO  extended verbs
- TODO  self_check.c
- TODO  imu_tests.c
- TODO  interbank.c (comment file)
- TODO  tests/test_sysmisc.c

## Deviations

- SGNAGREE: AGC lets the high word overflow when |V|>=1; C saturates at POSMAX.
- NORMUNIT keeps the AGC's "DAS gains a little accuracy" factor (2^13+1).
- CDU angles are uint16_t 15-bit two's-complement patterns throughout.

## Stubs

(none yet)

## Requested core additions

Declared (extern, not defined) in src/io/sysmisc_ext.h; names follow
docs/agents/_common.md.  Array erasables take an index argument.
- `agc_sp_t agc_erase_pipax(void)`, `agc_erase_pipay`, `agc_erase_pipaz` (PIPA counters)
- `agc_status_t agc_imu_imupulse(agc_vec_t gyro_cmds)` (imu lane, IMUPULSE)

## Requested jobs-tasks

(filled as code is written)

## Tests written

tests/test_sysmisc.c (TODO)

## Open questions

- FORMDEG named in the assignment does not exist in Comanche055 or
  Luminary099.
