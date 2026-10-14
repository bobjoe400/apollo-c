# exec-sched -- lane history

Process diary of the exec-sched lane, moved verbatim out of [docs/modules/exec-sched.md](../../modules/exec-sched.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-14 (first and last commit touching the note).

## Progress

- DONE executive.c (NOVAC FINDVAC SPECTEST CHANJOB EJSCAN ENDOFJOB JOBSLEEP JOBWAKE PRIOCHNG CHANG1 DUMMYJOB)
- DONE waitlist.c (compiles; T3RUPT services a pending ENEMA; TASKOVER task added)
- DONE exec_pending.h (provisional foreign job/task ids, 0x0E00 range)
- DONE agc_exec.h second-pass prototypes; agc_jobs.h AGC_JOB_ENDOFJOB, AGC_TASK_TASKOVER
- DONE phase_table.c (PHASCHNG A/B/C, 2PHSCHNG, NEWPHASE, KLEANs, NEWMODE, CHECKMM, SETUPDSP)
- DONE restart_tables.c (both vehicles, static const; 5 indirect erasables UNIMPLEMENTED pending accessors)
- DONE restarts.c (RESTARTS, FINDTIME, LONGCALL restart, NXTRST loop = agc_phase_restart_all)
- DONE alarm.c (ALARM VARALARM PRIOLARM(store only) CURTAINS BAILOUT POODOO GOPOODOO CCSHOLE ABORT VAC5STOR, ENEMA request/service)
- DONE interrupts.c (INHINT/RELINT counter, 11 lead-ins, TIME4-6, T5LOC/T6LOC, T5IDLOC)
- DONE erasable.c (MODECADR: IMUCADR/OPTCADR/RADCADR)
- DONE fresh_start.c (SLAP1 DOFSTART DOFSTRT1 GOPROG ENEMA GOPROG3 LIGHTSET STARTSUB STARTSB1 STARTSB2 GOTOPOOH; foreign erasables delegated)
- PARTIAL change_program_idle.c (GOP00FIX + reflash, INITSUB/RAKE, V37 front-end checks + PREMM1 tables, upcalls; CANV37, R00TOP00, SETUP70 stubbed)
- DONE dispatch.c (job, task, rupt switches; provisional ids -> default NOT_FOUND)
- DONE check_rules clean (0 hard, density >= 2 all files)
- DONE tests/exec/exec.c (197 checks pass; links src/exec + src/core objects only)
- DONE module note sections, MODULE_STATUS row

## Requested core additions

Added by me to include/agc/exec/exec.h (allowed by the assignment): agc_exec_restart_pending, agc_exec_service_restart, agc_exec_enema, agc_exec_varalarm, agc_phase_kill, agc_job_gotopooh, agc_job_go_idle_fix, agc_job_go_idle_fix_reflash, agc_job_gopoodoo, agc_job_endofjob, agc_job_change_program_continue, agc_task_endtask, agc_task_svct3, agc_task_longcycl, agc_task_getcadr, agc_task_t5idloc, agc_task_taskover, AGC_MODECADR_N, agc_erase_modecadr, agc_erase_set_modecadr, agc_erase_imucadr, agc_rupt_time, agc_rupt_set_time (earlier pass: everything under "Additions by exec-sched").

Requested from other owners (exact declarations):
- state-flags / guid-powered: `agc_dp_t agc_erase_preread_delay(void);` `agc_dp_t agc_erase_savet_30(void);` `agc_sp_t agc_erase_zoomtime(void);` (restart tables; exec must not call agc_guid_state()).
- state-flags / guid-entry: `agc_sp_t agc_erase_s61dt(void);` `agc_sp_t agc_erase_cm_gymdt(void);`
- guid-powered: `agc_status_t agc_guid_servidle(void);` (SERVIDLE: SERVICER to ground state only; exec then does GOTOPOOH).
- imu: `agc_status_t agc_imu_setcoars(void);` (CSM GOPROG NO ATT).
- dap-csm: `agc_status_t agc_csm_dap_setmaxdb(void);` `agc_status_t agc_csm_dap_setmindb(void);` (CSM INITSUB).
- sys-misc: public `agc_status_t agc_t4rupt_rupt(void);` and `agc_status_t agc_telem_downrupt(void);` for dispatch_rupt; io-pinball: an UPRUPT entry that reads INLINK itself; prog-rendz/sys-misc: the radar RUPT handler (VHFREAD / RADAREAD).
- Each owner of a "delegated" erasable (list under Stubs): a `<lane>_startsb2_reset()` / `_dofstart_reset()` entry that fresh_start.c can call through change_program_idle.c.

## Requested jobs-tasks

Mine, already added to agc_jobs.h: AGC_JOB_GOTOPOOH, AGC_JOB_GO_IDLE_FIX, AGC_JOB_GO_IDLE_FIX_REFLASH, AGC_JOB_GOPOODOO, AGC_JOB_ENDOFJOB; AGC_TASK_SVCT3, AGC_TASK_LONGCYCL, AGC_TASK_GETCADR, AGC_TASK_T5IDLOC, AGC_TASK_TASKOVER (dispatch.c cases present).

Foreign, provisional in include/agc/agc_exec_pending.h (0x0E00 range; replace with the owner's id and add the dispatch case):
- jobs: INITDSP, DSPMMJB, NBDONLY, STATINT1, RELINUS, TIMEBURN, POSTBURN, NORMLIZE, SERVICER, TIMEDIDR, REDO40_9, PIKUP20, R22, V94ENTER, REDOR22, MATRXJOB, REP11, REP11A, P47BODY, COMPVER, AZMTHCG1, PREBURN_STATE_VECTOR, PROG52, R51, ENTRY_FINAL_PHASE_DISPLAY, ENTRY_IMU_AXIS_TEST, PRE_HUNT, ENTRY_UPCONTROL_DISPLAY, RSTGTS1, WTLISTNT, RESTEST1, GEOSTRT4, ALFLT1, RESTEST3, RESTAIER, RESCHNG, P25LEM1, R22RSTRT, GOABORT, FINE_ALIGN_PERFORM, P70A, P71A, COMFAIL, REDO6_7.
- tasks: STATEINT, REREADAC, REDO5_5, CLOKTASK, TIG_5, TIG_0, IGNITION, SETJTAG, ENGINOFF, PRECHECK, TTG_0, DOTVCON, DOSTRULL, ENGINE_FAIL_REIGNITE_TRIM, R40ENABL, TIGBLNK, DOTVCRCS, WAKEP62, TIGAVEG, ENTRY_START_AVERAGE_G, ATERTASK, V97ETASK, TIGON, DOSPSOFF, V97TTASK, V97TRCS, V97PTASK, SPSOFF97, ALLOOP1, SPECSTS, CHEKAVEG, PREREAD, PRE40_6, READGYMB, REDORCS, REDOTVC, REDOSAT, TVCEXEC, CA_ECE, ULLGTASK, P20LEMC1, REDO2_17, ABORT_MONITOR, ZOOM, REDO4_2, ULLAGOFF, ENGOFTSK, TIG_30, TIG_35, BIBIBIAS, DAPIDLER.

## Open questions

- AGC_JOB_GOTOPOOH is an agc_jobs.h member but include/agc/csm/entry_pending.h and src/programs/navigation/pending.h #define it again: those lanes must drop their macros.
- AGC_JOB_GOPROG in agc_jobs.h does not correspond to an AGC job; dispatch returns UNIMPLEMENTED. Remove or repurpose?
- STATEUP / THISVINT (end of FRESH_START_AND_RESTART) are P00 integration code; assumed to belong to nav-integ.
- Layering: change_program_idle.c and dispatch.c call io/imu/dap; everything else in src/exec calls only core and exec.

