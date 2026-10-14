# sys-misc -- module note

History of this lane: [history/lanes/sys-misc.md](../../history/lanes/sys-misc.md)

## Scope

T4RUPT_PROGRAM, EXTENDED_VERBS, DOWNLINK_LISTS, DOWN_TELEMETRY_PROGRAM,
UPDATE_PROGRAM (P27), SERVICE_ROUTINES, AGC_BLOCK_TWO_SELF_CHECK,
SYSTEM_TEST_STANDARD_LEAD_INS, IMU_PERFORMANCE_TEST_2 / _TESTS_4 (LM P07),
INTER-BANK_COMMUNICATION, RTB_OP_CODES (LM) / RT8_OP_CODES (CM).

Public API of this lane: `include/agc/io/service_internal.h`.  Symbols owned
by other lanes / not yet in include/agc are declared in
`src/io/service/sysmisc_ext.h` (service/T4RUPT side) and
`src/io/verbs/pending.h` (verbs side, with the provisional job ids); see
"Requested core additions" / "Requested jobs-tasks" ([the lane history](../../history/lanes/sys-misc.md)).

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

## Tests written

tests/io/service_verbs_telemetry.c: every downlist (CM 5, LM 6) is 100 DP words; DOWNRUPT sequence of
the CM coast-and-align list (ID/sync, TIME2/TIME1, next list, UPDNLIST); LOADTIME;
SGNAGREE / TPAGREE sign fix-up; ROPECHK/SHOWSUM bank sums; UPFLAG/DOWNFLAG, GENTRAN,
B5OFF/EXTVBACT; LM T4RUPT fills M11..M32 from CDUZ/CDUX; V44 RRDESEND outside continuous
designate.  271 checks pass (linked against a scratch build plus link stubs).

## Program flow and decisions

_Documentation wave (2026-10-12); prose and tables brought to the final tree (HEAD 3b97dc8) by the final pass (2026-10-14). The prose was written by reading the code; the two tables were extracted from the function bodies (flag, alarm, channel and scheduling calls; `agc_erase_*` accessors) and list only what the code does. Files: `src/io/service/*.c`, `src/io/telemetry/*.c`, `src/io/verbs/*.c`, `src/lm/systems_test/*.c`._

T4RUPT, DOWNRUPT, the extended verbs, P27 uplink, the service routines,
self-check and the LM systems test. The decision points:

* **T4RUPT** (`src/io/service/t4rupt.c:agc_t4rupt_rupt` -> `agc_t4rupt`,
  every 120 ms): CDRVE sends the lamp word or one DSKY relay word (after
  a relay word the next five T4RUPTs come 20 ms apart, QUIKDSP);
  PROCEEDE edge-detects channel 32 bit 14 (PRO); T4JUMP (`t4_jump`) runs
  one activity per phase, each once per 0.96 s: IMUMON (turn-on request
  dropped 00207, turn-on without operate 00213, ISS off while in use
  00214, PIPA fail 00212; ISS warning by VARALARM), C33TEST (down/uplink
  too fast 01105/01106), GLOCKMON (gimbal lock lamp and coarse align
  stop), the CSM optics (`t4rupt_csm.c`: OPTMON zeroing sequence, switch
  moved while zeroing 00116, not zeroed since fresh start 00120; OPTDRIVE)
  and the LM radar monitors (`t4rupt_lm.c`: RR CDU fail with RNDVZFLG in
  P20/P22 is 00515; GPMATRIX for the DAP). RCSMON (dap-lm) also runs here.
* **DOWNRUPT** (`src/io/telemetry/down_telemetry.c:agc_telem_downrupt`,
  every 20 ms): word order bit to 1 in channel 13 (LM WOTEST: only when it
  is 0, Comanche every time; closed-loop-fixes), then one DP word of the
  current downlist (control list and sublists of
  `downlink_lists_csm.c` / `downlink_lists_lm.c`, each the rope's table
  of DNADR words: count and ECADR, read through the address map by
  `word_at` / `fetch2wd`, SNAPLOOP's snapshot into DNTMBUFF by address)
  to channels 34/35, 100 words a list; a new DNLSTCOD outside DNTABLE is BAD_ARG at NEWLIST; V74
  replaces the list by the erasable dump.
* **Extended verbs** (`src/io/verbs/extended_verbs.c`,
  `extended_verbs_csm.c`, `extended_verbs_lm.c`,
  `extended_verbs_lm_radar.c`): GOEXTVB fans verbs 40-99 (LST2FAN);
  TESTXACT refuses with OPR ERR (ALM/END) when the extended-verb display
  is busy, a priority display holds the DSKY, MODECADR is in use, or the
  verb needs P00 (CHKPOOH). Per-verb refusals: CSM V54/V57 without
  rendezvous tracking 00406, V41N91 00115/00117, V92 after P11 POODOO
  01521; LM radar verbs refused while a radar is in use (RDRUSECK) or P20
  runs, designate fail 00503, LR not in position 2 00523; LM R03 asks
  again for masses below an empty vehicle. V40-V42 (IMU zero, coarse and
  fine align from the keyboard; V42 torques by OGC/IGC/MGC).
* **P27 uplink** (`src/io/verbs/uplink_update.c`): V70 liftoff time, V71
  block (3..20 components, OPR ERR if it crosses an EBANK), V72 scattered
  pairs (even count OPR ERR), V73 clock; refused in a mode that may not
  be interrupted. Words arrive by V21N01; V21N02 / V33 verify
  (`agc_job_uplink_update_proceed`); execute (`agc_job_uplink_update_execute`)
  holds INTSTALL, restart-protects, sets REINTFLG and stores through
  `agc_erase_write_ecadr` (state-flags); V34 or an error restores the old
  mode and kills group 6 (`agc_job_uplink_update_terminate`).
* **Service** (`src/io/service/service.c`): DELAYJOB
  (`agc_svc_delayjob`) sleeps the job and wakes it as the given
  continuation; all four DELAYLOCs in use is BAILOUT 01104. UPFLAG /
  DOWNFLAG / UPENT2 / DOWNENT2 (CSM: flagword in bits 1-3; LM: FLAGWRD4
  always), GENTRAN, B5OFF, CSM tracker lamps. `rtb_ops.c`: the RTB op
  codes as plain functions on MPAC values. `interbank.c`: no code (bank
  switching does not exist in C).
* **Self-check** (`self_check.c`): a bank whose sum is not +-bank number
  is alarm 01102 (ERCOUNT); SMODE + drops to idle, - restarts SELFCHK, -0
  goes on; memory tests pass by construction (DEVIATION).
* **LM P07 systems test** (`src/lm/systems_test/`: `test_erasables.c`
  the test erasables, `imu_performance_test.c`, `drift_filter.c`,
  `job_entries.c` the registry entries and step dispatch): IMU performance
  tests (latitude/azimuth V06N41, coarse and fine align, gyro drift, PIPA
  scale, vertical drift); bad IMU moding is VARALARM 01601 (SOMERR2); a
  first PIPA count of +0 is the CCS hole; the erasable program (ATIGINC)
  is not run (DEVIATION).

### Decision points by function

61 of the lane's 417 functions test or change a flag, raise an alarm, test a channel bit or start a job/task (AGC = the label(s) the prolog names; "+F" sets flag F, "-F" clears it; "starts" = NOVAC/FINDVAC, WAITLIST, LONGCALL, DELAYJOB targets).

| C function | AGC | flags tested | flags set (+) / cleared (-) | alarms | channel bits | starts |
|---|---|---|---|---|---|---|
| `ropechk_bank` (self_check.c) | ROPECHK | - | - | ALARM 1102 | - | - |
| `agc_svc_delayjob` (service.c) | DELAYJOB | - | - | BAILOUT 1104 | - | task waker |
| `agc_t4_setissw` (t4rupt.c) | SETISSW | - | - | ALARM (agc_alarm_code_t) ( | - | - |
| `t4_imucage` (t4rupt.c) | IMUCAGE | - | -ENGONFLG | - | - | - |
| `t4_iturnon` (t4rupt.c) | ITURNON | - | - | ALARM 0207 | - | - |
| `t4_imuop` (t4rupt.c) | IMUOP | IMUSE | -RNDVZFLG -IMUSE | ALARM 0214 | - | - |
| `t4_proctnon` (t4rupt.c) | PROCTNON | IMUSE | - | ALARM 0213 | - | - |
| `agc_t4_c33test` (t4rupt.c) | C33TEST | - | - | ALARM 1105, ALARM 0212 | - | - |
| `t4_unz2` (t4rupt.c) | UNZ2 | - | - | - | - | task issup |
| `agc_t4_task` (t4rupt.c) | - | DRIFTFLG IMUSE | +DRIFTFLG | - | - | task endtnon |
| `t4_zoptdes` (t4rupt_csm.c) | ZOPTDES ZTOMAN ZOPALARM | - | - | ALARM 0116 | - | task endzopt |
| `agc_t4_optics_task` (t4rupt_csm.c) | ENDZOPT ZEROPCDU | - | - | - | - | task endzopt2 |
| `agc_t4_optics_drive` (t4rupt_csm.c) | OPTDRIVE OZONE CONTDRVE | - | - | ALARM 0120 | - | - |
| `radnoop` (t4rupt_lm.c) | RADNOOP | - | - | - | - | task rr_goodend |
| `agc_rr_zero` (t4rupt_lm.c) | RRZERO | - | - | ALARM 0510 | - | task rr_zero |
| `agc_radar_lrpos2` (t4rupt_lm.c) | LRPOS2 | - | - | - | - | task lr_position_scan |
| `t4_rrgimon` (t4rupt_lm.c) | RRGIMON MONREPOS | AVEGFLAG MUNFLAG NORRMON | - | - | - | task dorrepos |
| `t4_rrcduchk` (t4rupt_lm.c) | RRCDUCHK | RNDVZFLG | - | ALARM 0515 | - | - |
| `agc_t4_rr_monitor` (t4rupt_lm.c) | RRAUTCHK | - | - | - | - | task rrturnon |
| `after_liftoff` (extended_verbs_csm.c) | - | GRRBKFLG | - | - | - | - |
| `agc_job_attitude_error_load_csm` (extended_verbs_csm.c) | - | - | - | - | - | task attitude_error_output |
| `verb_marking` (extended_verbs_csm.c) | GOTOR21 GOTOR23 | RNDVZFLG TRACKFLG | - | ALARM 0406 | - | - |
| `verb_terminate_tracking` (extended_verbs_csm.c) | TRACKTRM | RNDVZFLG TRACKFLG | -RNDVZFLG -TRACKFLG -UPDATFLG -IMUSE | - | - | - |
| `verb_total_error` (extended_verbs_csm.c) | - | - | +NEEDLFLG | - | - | - |
| `verb_systems_test` (extended_verbs_csm.c) | SYSTEST | NO_PRELAUNCH_INITIALIZATION | - | POODOO 1521 | - | - |
| `verb_optics_coarse` (extended_verbs_csm.c) | OPTCOARK | - | - | ALARM 0115, ALARM 0117 | - | - |
| `fan_70s_90s` (extended_verbs_csm.c) | - | - | -VHFRFLAG -RENDWFLG +QUITFLAG +IDLE_INTEGRATION_INHIBITED | - | - | - |
| `agc_job_attitude_error_enable` (extended_verbs_lm.c) | - | - | - | - | - | task attitude_error_output |
| `agc_job_dap_data_load_mass_lm` (extended_verbs_lm.c) | - | APSFLAG | - | - | - | - |
| `agc_job_dap_data_load_trim` (extended_verbs_lm.c) | - | - | - | - | - | task trimgimb |
| `verb_terminate_tracking` (extended_verbs_lm.c) | TRMTRACK | PREFERRED_TRACKING_ATTITUDE_ACTIVE RNDVZFLG TRACKFLG | -RNDVZFLG -PREFERRED_TRACKING_ATTITUDE_ACTIVE -SRCHOPTN -UPDATFLG -TRACKFLG -IMUSE | - | - | - |
| `verb_rendezvous_display` (extended_verbs_lm.c) | - | - | - | - | - | task rendezvous_parameter_display |
| `fan_40s_60s` (extended_verbs_lm.c) | - | APSFLAG | - | - | - | - |
| `fan_70s_90s` (extended_verbs_lm.c) | - | - | -RENDWFLG +QUITFLAG | - | - | - |
| `radar_in_use_check` (extended_verbs_lm_radar.c) | RDRUSECK | LRBYPASS NO_POWERED_DESIGNATE RADAR_ALARMS_SUPPRESSED RNDVZFLG SERVICER_RUNNING TRACKFLG | - | - | - | - |
| `agc_job_lr_position_two_check` (extended_verbs_lm_radar.c) | - | - | - | ALARM 0523 | - | - |
| `agc_extverb_rr_designate` (extended_verbs_lm_radar.c) | RRDESNBK | RNDVZFLG | - | - | - | - |
| `agc_job_rr_lockon_answer` (extended_verbs_lm_radar.c) | - | - | =LOKONSW +CDESFLAG +NORRMON | - | - | - |
| `agc_job_rr_designate_result` (extended_verbs_lm_radar.c) | - | - | - | ALARM 0503 | - | - |
| `agc_extverb_rr_designate_end` (extended_verbs_lm_radar.c) | RRDESEND | - | - | - | - | job rr_designate_end_monitor |
| `agc_job_rr_designate_end_monitor` (extended_verbs_lm_radar.c) | - | - | -NORRMON | - | - | - |
| `radar_test_is_self_test` (extended_verbs_lm_radar.c) | FLAGWRD3 R04FLBIT | ALARM_521_SUPPRESSED | - | - | - | - |
| `radar_test_start` (extended_verbs_lm_radar.c) | - | - | - | - | - | task radsamp |
| `agc_extverb_radar_test` (extended_verbs_lm_radar.c) | R04 | - | =ALARM_521_SUPPRESSED | - | - | - |
| `agc_extverb_lr_spurious_test` (extended_verbs_lm_radar.c) | R77 | - | =RADAR_ALARMS_SUPPRESSED | - | - | - |
| `agc_job_radar_test_switch` (extended_verbs_lm_radar.c) | R04Y | - | - | - | - | job radar_test_switch_resume |
| `agc_job_radar_test_end` (extended_verbs_lm_radar.c) | R04END | - | - | - | - | job radar_test_end_resume |
| `agc_job_radar_test_end_resume` (extended_verbs_lm_radar.c) | - | - | -ALARM_521_SUPPRESSED | - | - | - |
| `agc_extverb_lr_spurious_test_end` (extended_verbs_lm_radar.c) | R77END | - | - | - | - | job lr_spurious_test_end_resume |
| `agc_job_lr_spurious_test_end_resume` (extended_verbs_lm_radar.c) | - | - | -RADAR_ALARMS_SUPPRESSED | - | - | - |
| `agc_job_uplink_update_proceed` (uplink_update.c) | - | - | ~VERIFLAG | - | - | - |
| `agc_job_uplink_update_execute` (uplink_update.c) | - | - | +REINTFLG | - | - | - |
| `somerr2` (drift_filter.c) | SOMERR2 | - | -IMUSE | ALARM 1601 | - | - |
| `estims` (drift_filter.c) | ESTIMS | - | - | - | - | task systems_test_sample |
| `alloop` (drift_filter.c) | ALLOOP | - | - | - | - | task systems_test_sample, job systems_test_filter |
| `endtest1` (imu_performance_test.c) | ENDTEST1 | - | -IMUSE | - | - | - |
| `fine_aligned` (imu_performance_test.c) | FINIMUDD | - | - | - | - | task systems_test_drift_wake |
| `posgmbl` (imu_performance_test.c) | LATAZCHK POSGMBL | GLOKFAIL | -GLOKFAIL | - | - | - |
| `pipjobb_result` (imu_performance_test.c) | AINGOTN | - | - | POODOO 1103 | - | - |
| `pip_timer` (imu_performance_test.c) | PIPATASK | - | - | - | - | task systems_test_pip_timer, job systems_test_pip_result |
| `run_step` (job_entries.c) | - | - | - | - | - | task systems_test_pip_timer |

### Shared erasables read and written

Accessors of `include/agc/core/state.h` and the lanes' public headers called by this lane, with the number of its functions that read / write each. Lane-private erasables are `static` in the lane's files or in its state struct; their scales are on the declarations.

| erasable (accessor) | meaning and scale (accessor prolog) | read by | written by |
|---|---|---|---|
| `agsword` | none -- AGSWORD (LM): the downlist to return to after AGS initialization. RETURNS: AGSWORD; returns AGSWORD | 1 | 2 |
| `cdus` | CSM optics shaft CDU raw word (CDUS / OPTX); LM: rendezvous radar shaft | 2 | 1 |
| `cdut` | CSM optics trunnion CDU raw word (CDUT / OPTY); LM: rendezvous radar trunnion | 2 | 1 |
| `cdutcmd` | CDUTCMD (053) / CDUSCMD (054): optics (CSM) or RR (LM) CDU error counter drive, SP pulse counts; which 0 = CDUTCMD, 1 = CDUSCMD. In TVC mode the same  | 0 | 1 |
| `cdux` | CDU X (outer gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (register CDUX, 032) | 4 | 0 |
| `cduy` | CDU Y (inner gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUY) | 1 | 0 |
| `cduz` | CDU Z (middle gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUZ) | 3 | 0 |
| `cosmg` | COSMG (GPMATRIX, T4RUPT_PROGRAM.agc p.188) -- COS(MG), a factor only RETURNS: COSMG, scaled at 1; returns COSMG, scaled at 1 | 1 | 0 |
| `csmmass` | CSM mass, kg, scale 2^16 (CSMMASS; TVCMASSPROP 'B+16 KILOGRAMS'; noun 47 R1) | 2 | 0 |
| `dapdatr1` | DAP configuration word 1 (DAPDATR1, DSP noun 46 R1) | 5 | 2 |
| `delv` | accumulated PIPA delta-V, stable-member axes (DELV, I(6)): PIPA pulses scale 2^14 (count in the high word); 1 pulse = 5.85 cm/s CSM, 1 cm/s LM. KPIP1  | 2 | 3 |
| `desopts` | DESOPTS (ERASABLE_ASSIGNMENTS.agc, CM) -- the shaft angle OPTDRIVE drives to, read RETURNS: DESOPTS, CDU word (2^15 = 1 rev); returns DESOPTS, CDU wor | 2 | 1 |
| `desoptt` | DESOPTT (ERASABLE_ASSIGNMENTS.agc, CM) -- the trunnion angle OPTDRIVE drives to, read RETURNS: DESOPTT, CDU word (2^15 = 1 rev); returns DESOPTT, CDU  | 2 | 1 |
| `dnlstcod` | none -- DNLSTCOD, read: the code of the list sent from the next NEWLIST on (the erasable map's address 0332, both ropes) RETURNS: DNLSTCOD; returns DN | 1 | 0 |
| `dsptab` | display panel relay buffer DSPTAB +0..+10D, +11D is the C/S lights word (15-bit relay words) Index 0..11. | 8 | 5 |
| `dsptem1` | - | 0 | 3 |
| `dumpcnt` | none -- DUMPCNT, read (the erasable map's 0333, both ropes) RETURNS: DUMPCNT (BIT12 one dump, BIT13 two, BIT14 four; the low          bits count the d | 1 | 1 |
| `extvbact` | EXTVBACT (ERASABLE_ASSIGNMENTS; TESTXACT p.237 C / p.264 L) -- the extended verb activity bits, read RETURNS: EXTVBACT, 15 bits; returns EXTVBACT, 15  | 2 | 2 |
| `imodes30` | last-sampled channel 30 bits, T4RUPT IMUMON (IMODES30, B(1)PRM) | 12 | 8 |
| `imodes33` | last-sampled channel 33 / DAP-enable bits, T4RUPT C33TEST (IMODES33, B(1)PRM) | 8 | 5 |
| `landmark` | LANDMARK (CM P22 code ABCDE, V52), a signed SP number | 1 | 1 |
| `lemmass` | LM mass, kg, scale 2^16 (LEMMASS; DSP noun 47 R2) | 2 | 0 |
| `m11` | M11 (GPMATRIX, T4RUPT_PROGRAM.agc p.188) -- SIN(MG), read by the DAP RETURNS: M11, scaled at 1; returns M11, scaled at 1 | 1 | 0 |
| `m21` | M21 (GPMATRIX, T4RUPT_PROGRAM.agc p.189) -- COS(OG)COS(MG) RETURNS: M21, scaled at 1; returns M21, scaled at 1 | 1 | 0 |
| `m22` | M22 (GPMATRIX, T4RUPT_PROGRAM.agc p.188) -- SIN(OG), also MR22 RETURNS: M22, scaled at 1; returns M22, scaled at 1 | 1 | 0 |
| `m31` | M31 (GPMATRIX, T4RUPT_PROGRAM.agc p.189) -- -SIN(OG)COS(MG) RETURNS: M31, scaled at 1; returns M31, scaled at 1 | 1 | 0 |
| `m32` | M32 (GPMATRIX, T4RUPT_PROGRAM.agc p.189) -- COS(OG), also MR23 RETURNS: M32, scaled at 1; returns M32, scaled at 1 | 1 | 0 |
| `markstat` | mark status bit word (MARKSTAT, B(1)PRM; SXTMARK/R21/R22/P20/P23) | 1 | 0 |
| `mass` | LM vehicle mass, kg, scale 2^16, DP (MASS: SERVICER MASSMON, 1/ACCS) | 1 | 2 |
| `modecadr` | C(MODECADR +dev) (ERASABLE_ASSIGNMENTS.agc "MODE SWITCHING ERASABLE").; returns agc_sp_t -- the word (+0 none, positive a waiting job) | 2 | 0 |
| `modreg` | MODREG major mode (program) | 4 | 0 |
| `mrkbuf` | MRKBUF1 / MRKBUF2 (CM): two seven-word mark buffers (time DP, CDUY, word 0 = -1 (77776) marks a buffer empty. mrkbuf2 / set_mrkbuf2 read and store MRK | 0 | 1 |
| `neg_ratedb` | - | 0 | 1 |
| `optind` | OPTIND (ERASABLE_ASSIGNMENTS.agc, CM) -- coarse optics state, read RETURNS: OPTIND: -1, 0, 1, or AGC_OPTIND_NEG0 for -0; returns OPTIND: -1, 0, 1, or  | 2 | 3 |
| `optionx` | none (erasable read) OPTIONX +index (= DSPTEM2 +1 +index).; returns agc_sp_t -- OPTIONX +index as SP | 2 | 2 |
| `optmodes` | OPTMODES (ERASABLE_ASSIGNMENTS.agc, CM) -- the optics mode bit word, read RETURNS: OPTMODES, 15 bits (0 in an LM build); returns OPTMODES, 15 bits (0  | 2 | 2 |
| `optmon` | SWSAMPLE, DESOPMOD, WTOPTION, ZOPTCNT, ZONE (ERASABLE_ASSIGNMENTS, CM; the optics monitor's words, OPTMON pp.160-164) -- read by address; returns agc_ | 1 | 1 |
| `pipax` | PIPA X pulse count since last read (register PIPAX, 037), unscaled pulses: 1 pulse = 5.85 cm/s (CSM SERVICER207), 1 cm/s (LM) | 3 | 2 |
| `pipay` | PIPA Y pulse count (register PIPAY) | 3 | 2 |
| `pipaz` | PIPA Z pulse count (register PIPAZ) | 3 | 2 |
| `radmodes` | RADMODES (ERASABLE_ASSIGNMENTS; EQUALS FLGWRD12, FLAGWORD_ASSIGNMENTS p.85) -- the radar mode bit word, read RETURNS: uint16_t -- RADMODES (15 bits);  | 12 | 11 |
| `read_ecadr` | Read the erasable word at ECADR (raw 15-bit word): a flagword, a mapped erasable (its holders synced from their typed values), or the raw image. AGC:  | 5 | 0 |
| `rfailcnt` | none -- RFAILCNT (R04Z, RADSAMP) -- bad samples counted RETURNS: RFAILCNT; returns RFAILCNT | 1 | 1 |
| `rsampdt` | none -- RSAMPDT (R04 / R77, read by RADSAMP) -- the sampling interval RETURNS: RSAMPDT: interval + 1, cs (0 stops the sampling); returns RSAMPDT: inte | 1 | 1 |
| `rtstbase` | none -- RTSTBASE (R04X / R04L) -- base of the sample storage RETURNS: 0 for the RR, 2 for the LR; returns 0 for the RR, 2 for the LR | 1 | 0 |
| `rtstdex` | none -- RTSTDEX (R04X) -- which radar is sampled RETURNS: RTSTDEX: 1 RR, 2 LR (test index); returns RTSTDEX: 1 RR, 2 LR (test index) | 1 | 1 |
| `rtstloc` | none -- RTSTLOC (R04Z, RADSAMP) -- the next sample slot RETURNS: RTSTLOC; returns RTSTLOC | 1 | 1 |
| `rtstmax` | none -- RTSTMAX (R04C) -- the last sample slot RETURNS: 2 for the RR, 6 for the LR; returns 2 for the RR, 6 for the LR | 1 | 0 |
| `stiksens` | none -- accessor of the DAP erasable STIKSENS (ERASABLE_ASSIGNMENTS.agc).; returns agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h | 0 | 1 |
| `swsample` | SWSAMPLE (ERASABLE_ASSIGNMENTS, CM; set in OPTMON p.160) -- the optics mode switch as last sampled (read by V41 N91 and R52) RETURNS: +1 computer (CSC | 2 | 0 |
| `tephem` | time of the ephemeris epoch (TEPHEM = TIMSUBO, TP, centiseconds, scale 2^42: L "CSEC B-42 (TRIPLE PRECISION)"; P11 sets it at liftoff) | 1 | 1 |
| `tetcsm` | time of the permanent CSM state vector, centiseconds, scale 2^28 (TETCSM) | 1 | 1 |
| `tetlem` | time of the permanent LM state vector, centiseconds, scale 2^28 (TETLEM) | 1 | 1 |
| `thetad` | desired gimbal angles THETAD +0..+2 (CPHI/CTHETA/CPSI), raw 15-bit CDU-style words, 2^15 = 1 revolution Index 0..2. | 3 | 1 |
| `time1` | ---- time: TIME1/TIME2 (centiseconds, DP), TIME3..TIME6 are exec-private | 1 | 0 |
| `upsvflag` | UPSVFLAG (UPDATE_PROGRAM.agc, INTWAKEU) -- the state vector a P27 update replaced RETURNS: UPSVFLAG: 0, +-1 earth, +-2 moon (+ CSM, - LM); returns UPS | 1 | 1 |
| `write_ecadr` | Store a raw 15-bit word at ECADR: a flagword, a mapped erasable (the first item holding the address rebuilt from all its words), or the raw image. AGC | 5 | 0 |

### AGC labels -> C functions

The "AGC" column above names the label each listed function translates; the complete label -> function -> file:line index for the lane (and the tree) is [docs/overview/xref.md](../overview/xref.md), generated by `tools/xref.py`.
