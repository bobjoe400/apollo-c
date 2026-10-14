# sys-misc -- lane history

Process diary of the sys-misc lane, moved verbatim out of [docs/modules/sys-misc.md](../../docs/modules/sys-misc.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-14 (first and last commit touching the note).

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
- DONE  tests/io/service_verbs_telemetry.c (271 checks pass against a scratch build of the tree with link
        stubs for other lanes' missing symbols; the tree does not link yet)

## Requested core additions

Declared (extern, not defined) in src/io/service/sysmisc_ext.h or src/io/verbs/pending.h;
names follow docs/agents/_common.md.  Array erasables take an index argument.
- prog-rendz (P20-P25.agc): `bool agc_rr_limchk(uint16_t cdut, uint16_t cdus)` (RRLIMCHK),
  `agc_status_t agc_rr_settrkf(void)` (SETTRKF), `agc_status_t agc_rr_zero(void)`
  (RRZERO; prog-rendz has it in src/lm/radar/moding.c), `agc_status_t agc_rr_designate_gimbal(bool *started)`
  (RRDESNB, angles in TANG; *started false when RRDESNB ended the job, e.g. alarm 502),
  `agc_status_t agc_rendz_clradmod(void)` (CLRADMOD; same declaration as the landing shim),
  `agc_status_t agc_radar_lrpos2(void)` (LRPOS2; same as the powered-flight shim),
  task `AGC_TASK_RADSAMP` (0x1103, value from src/programs/navigation/pending.h).
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

