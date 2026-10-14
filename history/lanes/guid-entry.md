# guid-entry -- lane history

Process diary of the guid-entry lane, moved verbatim out of [docs/modules/guid-entry.md](../../docs/modules/guid-entry.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-14 (first and last commit touching the note).

## Progress
(DONE <C function> <file> / PARTIAL / TODO)
- ENTRY_LEXICON: DONE agc_csm_entry.h struct, entry_lexicon.h/.c (constants, VREFER)
- REENTRY_CONTROL: DONE reentry.c (targeting, STARTENT, all steering steps, sequencer, REFAZE10, PRE-HUNT, HUNTEST/RANGER test API)
- CM_BODY_ATTITUDE: DONE cm_body_attitude.c (CM/POSE triad, CM/TRIO, CM/ATUP, REDOPOSE, POSEXIT, SERVNOUT, REFAZE10 job)
- CM_ENTRY_DIGITAL_AUTOPILOT: DONE dap.c (READGYMB, BODYRATE, ATTRATES, atm/ext-atm DAP, CM/RCS phase plane, TIMETST, JETCALL, DZCALL, CM/FDAI+TM, SETJTAG, CM/DAPON, CM/DAPIC)
- P61-P67: DONE programs.c (P61..P67 jobs, P64-P67 mode changes, KILLGRP4, WAKEP62, S61.1/S61.1A/S61.1C, S61.2 with TFF/TRIG FISHCALC VGAMCALC GMAX DISPTARG VRCALC, S62.3, P67.2)
- P37_P70: PARTIAL src/csm/burns/return_to_earth/program.c (XT1LIM done; P37 program and its conic/precision
  subroutines are a documented stub -- assignment asked "read and describe")
- P11: DONE earth_orbit_insertion.c (P11, REP11, REP11A, MATRXJOB+GETDOWN, VHHDOT, S11.1,
  ATERTASK, ATERJOB/ATTDISP/attitude errors, ATRESET, SATSTKON, REDOSAT, SATSTICK)
- tests/csm/entry.c: DONE (23 checks; see Tests written)
- entry_pending.h: DONE rewritten against the now-available core headers (resume 2)

## Requested core additions
All are declared (not defined) in include/agc/csm/entry_pending.h; delete that file
at integration once the owners provide them. Flags, channel bits, DSKY
(agc_dsky_display/clearmrk/vn), lat/long, CALCGA/CALCGTA, NEEDLER and the
AVERAGEG state now come from the published headers.
- sys-misc: `agc_status_t agc_svc_set_extvbact(uint16_t v);` (TS EXTVBACT, P61).
- guid-powered: `agc_status_t agc_guid_preread1_csm(void);` (PREREAD1, P11).
- imu/prelaunch: `agc_status_t agc_csm_prelaunch_earthr_at_liftoff(agc_interp_t *ip,
  agc_dp_t tliftoff);` (P11 tail of the prelaunch ERCOMP/EARTHR).
- dap-csm: `agc_status_t agc_csm_dap_stickchk(uint16_t ch31_bits);` -- public
  wrapper for csm_rcs_stickchk (manual.c), used by SATSTICK.
- io-pinball: `agc_status_t agc_dsky_gopin(const agc_job_ctx_t *ctx);` (GOPIN,
  also requested by guid-powered).
- Provisional job/task ids AGC_JOB_SERVEXIT..AGC_JOB_ATRESET_AFTER_DELAY (0x0C00..0x0C1D),
  AGC_TASK_JETCALL ATTRATES READACCS SATSTICK (0x0C80..0x0C83), entry
  priorities, DNLSTCOD codes: move to agc_jobs.h / sysmisc.
- Shared math kept as local static copies until exported: LOG, EARROT1/2
  (CSM_GEOMETRY), ARCTRIG (INFLIGHT_ALIGNMENT_ROUTINES), CDULOGIC, SIGNMPAC
  (RT8_OP_CODES), POLY (INTERPRETER). Suggest agc_interp.h/agc_nav.h exports.
- agc_csm_entry.h: agc_earth_orbit_insertion_monitor_padload_t gained `satrate[4]` (SATRATE pad load).

## Requested jobs-tasks
Jobs: GOTOPOOH SERVEXIT CM_POSE P61 ENTRY_PREPARATION_COMPUTE ENTRY_PREPARATION_PREDICTION_DISPLAY ENTRY_PREPARATION_EMS_DISPLAY ENTRY_DISPLAY_PROTECT P62 CM_SM_SEPARATION_TARGET_DISPLAY CM_SM_SEPARATION_REQUEST CM_SM_SEPARATION_PROCEED
P63 ENTRY_UPCONTROL_DISPLAY ENTRY_UPCONTROL_PROCEED ENTRY_FINAL_PHASE_DISPLAY ENTRY_FINAL_PHASE_END PRE_HUNT ENTRY_IMU_AXIS_TEST CM_DAPON P37 P11 MATRXJOB VHHDOT
ATERJOB NORMLIZE. Tasks: READGYMB SETJTAG JETCALL WAKEP62 ENTRY_START_AVERAGE_G ATERTASK
READACCS. (Temporary values in entry_pending.h.)

## Open questions
- REENTRY_CONTROL p. 860 references label GETLEWD which is not defined;
  GETDLEWD is assumed (transcription typo).
- CH1 is 2DEC .32 B1, i.e. machine value .64 (= 16 CH1/25); the lexicon
  constant uses .64, confirmed by the test model.
- RESOLVED (review-guid-entry): GETLEWD is a typo of the transcription
  only; the assembled rope defines GETLEWD at 25,2677 (the code jumps
  there). CH1 = .64 confirmed against the rope words (24365 30244).

