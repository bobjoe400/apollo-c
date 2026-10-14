# exec-sched -- executive, waitlist, phase tables, restarts, alarms

## Scope

EXECUTIVE, WAITLIST, PHASE_TABLE_MAINTENANCE, RESTARTS_ROUTINE,
RESTART_TABLES, ALARM_AND_ABORT, INTERRUPT_LEAD_INS, FRESH_START_AND_RESTART
(Comanche055 = CSM, Luminary099 = LM). Translated once into src/exec/,
vehicle differences behind `agc_state_vehicle()`.

## Design table

| AGC label(s) (file, pages C/L) | C function | file | notes |
|---|---|---|---|
| NOVAC (EXEC C1208/L1103) | agc_exec_novac | executive.c | core set search NOVAC2/NOVAC3, 1202 |
| FINDVAC, FINDVAC2, VACFOUND (C1208,1211/L1103,1106) | agc_exec_findvac | executive.c | VAC1USE..VAC5USE, 1201 |
| SPECTEST, SETLOC (C1212/L1107) | exec_set_newjob (static) | executive.c | NEWJOB comparison on priority word |
| CHANG1, CHANG2, CHANJOB (C1208,1213/L1103,1108) | agc_exec_job_yield + chanjob (static) | executive.c | swap core set 0 <-> NEWJOB |
| JOBSLEEP, JOBSLP1 (C1210,1215/L1105,1110) | agc_exec_job_sleep | executive.c | priority negated, LOC = wake id |
| JOBWAKE, JOBWAKE2-4, WAKETEST (C1210,1216/L1105,1111) | agc_exec_job_wake | executive.c | |
| PRIOCHNG, PRIOCH2 (C1210,1217/L1105,1112) | agc_exec_prio_change | executive.c | BUSY => caller should yield |
| ENDOFJOB, ENDJOB1, EJSCAN, EJ1, EJ2 (C1210,1218/L1105,1113) | exec_end_of_job / exec_ejscan (static) | executive.c | |
| DUMMYJOB, ADVAN, NUDIRECT, NUCHANG2 (C1220/L1116) | agc_exec_run_one / agc_exec_idle | executive.c | activity light ch 11 bit 2 |
| WAITLIST, TWIDDLE (WAITLIST C1223/L1119) | agc_waitlist_call | waitlist.c | dt 1..16250 cs, 1204 |
| VARDELAY, FIXDELAY (C1223/L1119) | agc_waitlist_vardelay / _fixdelay | waitlist.c | |
| WAIT2, WTLST4, WTLST5, WTLST2 (C1226-1230/L1122-1127) | waitlist_insert (static) | waitlist.c | LST1 delta model, 1203 |
| T3RUPT, T3RUPT2, TASKOVER (C1231/L1128) | agc_waitlist_t3rupt / agc_waitlist_tick | waitlist.c | TIME3 = 16384-(T1-T) |
| ENDTASK, SVCT3, SVCT3X (C1225/L1121) | AGC_TASK_ENDTASK / AGC_TASK_SVCT3 | waitlist.c | NBDONLY every 81.93 s |
| LONGCALL, LNGCALL2, LONGCYCL, LASTTIME, MUCHTIME, GETCADR (C1233/L1130) | agc_waitlist_longcall + tasks | waitlist.c | 8192 cs cycles |
| PHASCHNG, 2PHSCHNG, PHSCHNGA, PHSCHNG2, CON2, BELOW1-4 (PTM C1409/L1298) | agc_phase_change, agc_phase_change_var, agc_phase_change2 | phase_table.c | types A/B/C |
| NEWPHASE (RESTART_TABLES C221) | agc_phase_newphase | phase_table.c | |
| NEWMODEX/NEWMODEA/MMDSPLAY/SETUPDSP/CHECKMM (PTM C1404/L1294) | agc_phase_newmode / agc_phase_checkmm / fresh_mmdsplay | phase_table.c | |
| MR.KLEAN, P00KLEAN, V37KLEAN (FRESH C185/L213) | agc_phase_mr_klean / _idle_klean / _change_program_klean | phase_table.c | group sets differ CSM/LM |
| RESTARTS ... PHSPART2 (RESTARTS_ROUTINE C1414/L1303) | agc_restart_group | restarts.c | |
| SIZETAB, n.mSPOT tables (RESTART_TABLES C212/L239) | rst_table_lookup | restart_tables.c | static const, both vehicles |
| ALARM, ALARM1/2, VARALARM, PRIOLARM, CURTAINS (A&A C1493/L1381) | agc_alarm, agc_prioalarm, agc_curtains | alarm.c | FAILREG[3], PROG light |
| BAILOUT, BAILOUT1, WHIMPER, ABORT (C1495/L1382) | agc_bailout, agc_abort_restart | alarm.c | -> ENEMA |
| POODOO, POODOO1, GOPOODOO, STRTIDLE, FLAGS, CCSHOLE (C1495/L1383) | agc_poodoo, agc_ccshole, AGC_JOB_GOPOODOO | alarm.c | |
| interrupt lead-ins at 4000 (INTERRUPT_LEAD_INS C132/L153) | agc_rupt_* | interrupts.c | |
| SLAP1, DOFSTART, DOFSTRT1, STARTSUB, STARTSB1, STARTSB2, ENEMA, GOPROG, GOPROG3, LIGHTSET, ENDRSTRT, GOTOPOOH, GOP00FIX, VAC5STOR, T5IDLOC (FRESH C183-205/L211-225) | agc_fresh_start, agc_restart_goprog, fresh_enema, ... | fresh_start.c | foreign erasables delegated |
| V37 / CANV37 / POOH / V37XEQ (FRESH C197-208/L227-236) | agc_change_program, agc_job_change_program_continue | change_program_idle.c | PARTIAL: checks only |
| job/task/rupt dispatch | agc_dispatch_job / _task / _rupt | dispatch.c | one switch each |

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

## Deviations

- Pre-emption: a job runs to completion or to agc_exec_job_yield/sleep (explained at the top of executive.c); MPAC/VAC contents are not saved.
- Software restart is deferred: BAILOUT/POODOO/ABORT record the alarm and request ENEMA; it runs at the next service point (end of job in agc_exec_run_one, end of T3RUPT, end of an interrupt lead-in, or agc_exec_service_restart). This keeps the call graph acyclic (ENEMA -> RESTARTS -> NOVAC -> BAILOUT would recurse). SLAP1/DOFSTART/GOPROG run immediately; run_one skips ENDOFJOB when the tables were re-initialised under it.
- WTLST4 checks for overflow before shifting (equivalent; the BAILOUT re-initialises the lists).
- WAIT2 "TIME3 already overflowed" branch cannot arise (T3RUPT is synchronous).
- VARDELAY/FIXDELAY with dt = 0 POODOO 1204 on both vehicles (Luminary behaviour).
- TBASEn stores TIME1, not -TIME1 (same elapsed time).
- PHASCHNG type B/C: the restart point is always named by the caller in agc_phase_var_t (C has no return address); an indirect delta time is resolved by the caller at phase-change time; agc_phase_change() with a B/C code keeps the stored point and priority.
- Restart-table "DEC 0" placeholders restart nothing (AGC would start a job at address 0). Even-table aliases kept (CSM 1.2=2.2=3.2=4.2, LM 2.2=3.2=6.2=1.2).
- ALMCADR/LOCALARM hold the running job's id, not an address.
- PRIOLARM records the alarm only; the caller requests the V05N09 PRIODSPR display (needs job ctx + continuations).
- Comanche ABORT (= BAILOUT with an inline code) does a plain WHIMPER.
- GOTOPOOH: from inside a job the job continues as GOP00FIX (yield); from restart code GOP00FIX is a NOVAC job at RESTREG (PRIO30).
- MODREG = -0 is represented as -1 (EXEC_MODREG_NEG0); any negative MODREG means -0 (agreed with io-pinball DSPMMJB).
- GOPROG: ERASCHK restore (ERESTORE/SKEEP4-7) not modelled (ERESTORE = +0); the WAND that resets channel 33 flip-flops is left to the sim harness.
- INHINT/RELINT are counted (nesting) rather than a single flag.

## Stubs

- agc_job_change_program_continue (CANV37 .. POOH / NOUVEAU / V37XEQ): UNIMPLEMENTED.
- agc_change_program: PARTIAL (IMU-init / NODOFLAG -> 1520, PREMM1 validity -> NOT_FOUND, SERVICER running -> BUSY); returns UNIMPLEMENTED for the change itself, CSM R00TOP00 and LM SETUP70/71.
- exec_up_servidle (LM STRTIDLE SERVIDLE) and exec_up_setcoars (CSM GOPROG SETCOARS): UNIMPLEMENTED until the owners publish them.
- rst_read_sp/rst_read_dp for SAVET-30, ZOOMTIME, P40TMP, S61DT, CM/GYMDT: UNIMPLEMENTED (restart of LM 1.3, 3.3, 4.25; CSM 4.21, 4.43, 4.57, 6.11 returns UNIMPLEMENTED).
- CSM INITSUB deadband restore (SETMAXDB/SETMINDB) skipped.
- agc_dispatch_rupt: T4RUPT, UPRUPT, DOWNRUPT, RADAR RUPT return UNIMPLEMENTED (handlers not in public headers).
- Delegated resets (named in comments where the AGC writes them): pinball DELAYLOC..DSPCOUNT, NOUT, MONSAVE*, VERBREG/NOUNREG/DSPLIST; sys-misc DNTMGOTO, DSRUPTSW, EXTVBACT, SELFRET, DNLSTCOD, AGSWORD, DUMPCNT, ERESTORE/SMODE/UPSVFLAG; imu ATTCADR, LGYRO, OPTIND, OPTMODES; prog-rendz RADMODES, SAMPLIM; dap-lm RCSFLAGS, T6NEXT, NXT6ADR, NEXTP, DAPBOOLS, DB, RATEINDX, STIKSENS, -RATEDB, HIASCENT, DKTRAP etc.; guid-landing THRUST.

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

## Tests written

tests/exec/exec.c (197 checks, all pass): priority ordering and FINDVAC-over-NOVAC tie, VAC exhaustion -> 1201 (+ PROG lamp, FAILREG chain and MULTFAIL bit), core-set exhaustion -> 1202 (7 CSM / 8 LM), sleep/wake, PRIOCHNG BUSY + yield, BAILOUT inside a job, waitlist ordering and LST1 deltas incl. RUPTAGN, overflow -> 1203, dt = 0 -> 1204 -> P00, LONGCALL 200 s, PHASCHNG A/B/C/2PHSCHNG/NEWPHASE codes, table task timing from TBASE (due / overdue), variable job (FINDVAC/NOVAC)/task/type B/X.1 restarts, LM tables (4.33 NOVAC, 1.2 aliases, 2.3 LONGCALL, unpublished erasable), KLEANs per vehicle, GOPROG (1107 + fresh start, P00 with no groups, group restart, T5LOC), LM POODOO -> GOTOPOOH 4.33.
Build (the full libagc does not build yet because of other lanes):
`gcc <flags> -DAGC_HOST_DEBUG -Iinclude tests/exec/exec.c src/exec/{executive,waitlist,phase_table,restart_tables,restarts,alarm,fresh_start,interrupts,exec_erasable}.c src/core/{agc_state,agc_channels,agc_erasable}.c`
(the test provides the dispatch and exec_up_* doubles; do not link dispatch.c or change_program_idle.c with it.)

## Open questions

- AGC_JOB_GOTOPOOH is an agc_jobs.h member but include/agc/csm/entry_pending.h and src/programs/navigation/pending.h #define it again: those lanes must drop their macros.
- AGC_JOB_GOPROG in agc_jobs.h does not correspond to an AGC job; dispatch returns UNIMPLEMENTED. Remove or repurpose?
- STATEUP / THISVINT (end of FRESH_START_AND_RESTART) are P00 integration code; assumed to belong to nav-integ.
- Layering: change_program_idle.c and dispatch.c call io/imu/dap; everything else in src/exec calls only core and exec.

## Program flow and decisions

_Documentation wave (2026-10-12); prose and tables brought to the final tree (HEAD 3b97dc8) by the final pass (2026-10-14). The prose was written by reading the code; the two tables were extracted from the function bodies (flag, alarm, channel and scheduling calls; `agc_erase_*` accessors) and list only what the code does. Files: `src/exec/executive.c`, `src/exec/waitlist.c`, `src/exec/interrupts.c`, `src/exec/fresh_start.c`, `src/exec/alarm.c`, `src/exec/restarts.c`, `src/exec/phase_table.c`, `src/exec/restart_tables.c`, `src/exec/modecadr.c`, `src/exec/dispatch.c`, `src/exec/change_program/*.c`._

The scheduler paths are drawn in [docs/overview/flowcharts/](../overview/flowcharts)
(job-preemption, waitlist-tick, software-restart, fresh-start,
program-change); this is the summary with the decision points.

* **Executive** (`executive.c`). `agc_exec_novac` / `agc_exec_findvac`
  -> `novac2`: no free VAC area is BAILOUT 1201, no free core set BAILOUT
  1202; the new job becomes NEWJOB when its priority word
  (`prio_word`: priority << 9 + VAC rank, negative while asleep) beats
  the running one (`setloc_compare`), or runs at once from idle
  (`spectest`; NEWJOB = +0 there is the AGC's CCS hole -> POODOO 1103).
  `agc_exec_run_one` swaps core set NEWJOB in (`chanjob`), dispatches
  core set 0 and then applies the job's exit: sleeping -> EJSCAN,
  yielded -> CHANJOB if NEWJOB > 0, returned -> `end_of_job`. A restart
  requested in the job is served first (`agc_exec_service_restart`); a
  change of the STARTSB2 generation means the job did a fresh start.
* **Waitlist** (`waitlist.c`). `agc_waitlist_call`: dt = 0 POODOO 1204,
  dt > 16250 cs BAD_ARG (documented DTMAX, DEVIATION), list full BAILOUT
  1203 (`wtlst4`); `wait2` places the task by walking LST1 with the
  AGC's TD - T(n) + 1 arithmetic, including the post-overflow TIME3 < 200
  case. `agc_waitlist_t3rupt` runs the RUPTAGN chain (at most 9 tasks)
  and stops it on a restart. ENDTASK -> `svct3`: DRIFTFLG set and IMU
  free (IMUCADR = 0) -> NOVAC NBDONLY, IMU busy -> retry in 5 s.
* **Alarms** (`alarm.c`). `store_alarm` fills FAILREG +0/+1/+2 (third
  and later alarms set bit 15 = MULTFAIL) and lights PROG on the first.
  `agc_bailout` = record + ENEMA request; `agc_poodoo`: CSM with
  SERVICER running -> ENEMA and BAILOUT status (the program goes on),
  otherwise STATEFLG/REINTFLG/NODOFLAG down, MR.KLEAN, ENEMA; LM ->
  4.35 and `gopoodoo` (SERVIDLE when SERVICER runs). Interpreter alarms
  (1301 ARCSIN/ARCCOS, 1302 SQRT) are latched by core and raised when the
  job or task returns (`exec_service_interp_alarm`, DEVIATION).
* **Restarts** (`restarts.c`, `phase_table.c`, `restart_tables.c`).
  `agc_phase_restart_all` walks groups 6..1; `agc_restart_group` branches
  on the phase word: 0 inactive; fixed phase 1 -> FINDVAC PRIO14 INITDSP;
  odd -> one table entry, even -> two; type C (bit 9) -> the stored
  job/task/LONGCALL by CJW; type B (bit 10) -> the stored job plus the
  first entry of phase & 177. A task restarts at max(1, dt - (TIME1 -
  TBASEn)) cs (`findtime`); a LONGCALL from LONGBASE (`start_long`).
  `phase_tables_ok`: a -PHASEn that is not the complement of PHASEn is
  ALARM 1107 and a fresh start.
* **Fresh start / hardware restart** (`fresh_start.c`): GOPROG tests the
  CSM channel 33 restart-loop condition (OSC FAIL absent, AGC WARNING on)
  and LIGHTSET (MARK REJECT + ERROR RESET held: channel 16 bit 7 CSM /
  bit 5 LM, key code 22 on channel 15/16) -> fresh start; otherwise
  STARTSUB and the vehicle tail (CSM T5LOC from FLAGWRD6 bits 15-14,
  NO ATT lamp -> coarse align + CA+ECE task; ENGONFLG -> engine on), then
  GOPROG3; with no group active MODREG >= 0 goes to P00
  (`agc_exec_gotopooh`: phase 4.1 CSM / 4.33 LM; from a job it yields to
  GOP00FIX, from restart code it takes over core set 0 when SPECTEST made
  MMDSPLAY's DSPMMJOB current (`executive.c:exec_core0_continue_as`), so
  PROG stays blank as in the AGC; otherwise NOVAC GOP00FIX at RESTREG).
* **Dispatch** (`dispatch.c`, generated by tools/gen_registry.py from
  tools/registry.tsv): four flat switches (CSM/LM x job/task) map an
  `agc_job_id_t` / `agc_task_id_t` to its function, the C form of the
  2CADR that NUDIRECT / T3RUPT2 DTCB to; an id with no function on the
  flown vehicle is NOT_FOUND.
* **V37** (`change_program/`): refusals 1520 (IMODES30 bit 6, NODOFLAG)
  and OPR ERR (not in FCADRMM1); SERVICER hand-off (AVEGFLAG); INTSTALL
  wait; POOH or NOUVEAU; STATQUO keeps a running P20/P25 (RNDVZFLG /
  P25 flag); the new program starts from group 4 after ENEMA (V37XEQ).

Data (all module-static, exec-private unless noted): core sets
(`core[]`: state, priority number 1..037, VAC area, LOC id, MPAC word),
`vac_used[5]`, NEWJOB, LOCCTR; TIME3 (cs, 14-bit counter, overflow 16384
cs), LST1 (cs deltas, `int16_t`), LST2 (task ids), LONGTIME (cs, DP),
LONGCADR; TIME4/5/6 (cs, TIME6 1/1600 s), T5LOC/T6LOC; PHASEn/-PHASEn,
PHSNAMEn/PHSPRDTn, TBASEn (TIME1 cs), LONGBASE; ALMCADR, FAILREG (shared,
erasable.c), REDOCTR, RESTREG. MODREG (shared) holds the major mode;
-0 is `EXEC_MODREG_NEG0`.

### Decision points by function

43 of the lane's 222 functions test or change a flag, raise an alarm, test a channel bit or start a job/task (AGC = the label(s) the prolog names; "+F" sets flag F, "-F" clears it; "starts" = NOVAC/FINDVAC, WAITLIST, LONGCALL, DELAYJOB targets).

| C function | AGC | flags tested | flags set (+) / cleared (-) | alarms | channel bits | starts |
|---|---|---|---|---|---|---|
| `agc_curtains` (alarm.c) | CURTAINS | - | - | ALARM 0217, CURTAINS 0217 | - | - |
| `poodoo_flags` (alarm.c) | FLAGS | - | -STATEFLG -REINTFLG -NODOFLAG | - | - | - |
| `gopoodoo` (alarm.c) | GOPOODOO STRTIDLE | SERVICER_RUNNING | - | - | - | - |
| `exec_service_interp_alarm` (alarm.c) | - | - | - | ALARM 1301, POODOO 1302 | - | - |
| `agc_poodoo` (alarm.c) | POODOO POODOO1 ABORT2 | SERVICER_RUNNING | - | - | - | - |
| `agc_ccshole` (alarm.c) | CCSHOLE | - | - | POODOO 1103, POODOO 1103 | - | - |
| `exec_cp_engine_off_to_idle` (csm.c) | R00TOP00 | - | - | - | - | job change_program_jets_off |
| `exec_cp_stop_servicer_csm` (csm.c) | ISSERVON | - | -AVEGFLAG | - | - | - |
| `exec_cp_cleanup_csm` (csm.c) | CANV37 | - | -STIKFLAG -VHFRFLAG -MARKRUPT_OPTION_ONE | - | ch11 UPACT | - |
| `cp_track_flags_off` (csm.c) | SEUDOP00 | - | -TRACKFLG -TARG1FLG -UPDATFLG | - | - | - |
| `exec_cp_pooh_flags_csm` (csm.c) | POOH | - | -IMUSE -RNDVZFLG | - | - | - |
| `exec_cp_nouveau_flags_csm` (csm.c) | NOUVEAU | RNDVZFLG | -IMUSE | - | - | - |
| `exec_cp_rendezvous_csm` (csm.c) | RENDV00 REV37 | RNDVZFLG | +TRACKFLG +UPDATFLG | - | - | - |
| `exec_initsub_csm` (idle.c) | INITSUB RAKE | MAXDBFLG | +IMPULSW | - | - | - |
| `exec_cp_stop_servicer_lm` (lm.c) | ISSERVON | - | -AVEGFLAG | - | - | - |
| `agc_job_change_program_return` (lm.c) | V37RET | PREFERRED_TRACKING_ATTITUDE_ACTIVE RNDVZFLG | - | - | - | - |
| `exec_cp_cleanup_lm` (lm.c) | R00 | - | -3AXISFLG -ALARM_521_SUPPRESSED -MUNFLAG -XOVINFLG | - | - | - |
| `cp_rendezvous_flags_off` (lm.c) | POOH RESET22 | - | -RNDVZFLG -IMUSE -PREFERRED_TRACKING_ATTITUDE_ACTIVE | - | - | - |
| `cp_seudop00` (lm.c) | SEUDOP00 | - | -TRACKFLG -UPDATFLG | - | - | - |
| `exec_cp_nouveau_flags_lm` (lm.c) | NOUVEAU | PREFERRED_TRACKING_ATTITUDE_ACTIVE RNDVZFLG | -IMUSE | - | - | - |
| `exec_cp_rendezvous_lm` (lm.c) | RENDV00 RENDN00 STATQUO | PREFERRED_TRACKING_ATTITUDE_ACTIVE RNDVZFLG | +TRACKFLG +UPDATFLG | - | - | - |
| `cp_refuse` (sequence.c) | CANTR00 V37BAD | - | - | ALARM 1520 | - | - |
| `cp_select` (sequence.c) | ISITP00 CHECKTAB ISSERVON | NODOFLAG SERVICER_RUNNING | - | - | - | - |
| `cp_pooh` (sequence.c) | POOH | - | -NODOFLAG | - | - | - |
| `activity_light` (executive.c) | - | - | - | - | ch11 COMPACT | - |
| `spectest` (executive.c) | SPECTEST | - | - | POODOO 1103 | - | - |
| `novac2` (executive.c) | NOVAC2 NOVAC3 CORFOUND | - | - | BAILOUT 1202 | - | - |
| `agc_exec_novac` (executive.c) | NOVAC | - | - | - | - | job none |
| `agc_exec_findvac` (executive.c) | FINDVAC FINDVAC2 VACFOUND | - | - | BAILOUT 1201 | - | job none |
| `agc_exec_run_one` (executive.c) | DUMMYJOB ADVAN NUCHANG2 | - | - | POODOO 1103 | - | - |
| `phase_tables_ok` (fresh_start.c) | PCLOOP PTBAD | - | - | ALARM 1107 | - | - |
| `goprog_csm_tail` (fresh_start.c) | ELRSKIP NOCOARSE | ENGONFLG | -INTFLAG | - | - | task ca_ece |
| `agc_restart_goprog` (fresh_start.c) | GOPROG | ENGONFLG | -INTFLAG | - | - | - |
| `fresh_enema` (fresh_start.c) | ENEMA GOPROG2 | - | - | - | - | task tvcexec |
| `agc_exec_gotopooh` (fresh_start.c) | GOTOPOOH | - | - | - | - | job go_idle_fix |
| `fresh_mmdsplay` (phase_table.c) | MMDSPLAY SETUPDSP | - | - | - | - | job dspmmjb |
| `restart_table` (restarts.c) | GETPART2 ITSATBL ITSEVEN | - | - | - | - | job initdsp |
| `wtlst4` (waitlist.c) | WTLST4 WTLST2 | - | - | BAILOUT 1203 | - | - |
| `wait2` (waitlist.c) | WAIT2 WTLST5 WTLST2 | - | - | BAILOUT 1203 | - | - |
| `agc_waitlist_call` (waitlist.c) | WAITLIST TWIDDLE | - | - | POODOO 1204 | - | task none |
| `longcycl` (waitlist.c) | LONGCYCL MUCHTIME LASTTIME | - | - | - | - | task longcycl, task getcadr |
| `agc_waitlist_longcall` (waitlist.c) | LONGCALL LNGCALL2 | - | - | POODOO 1204 | - | - |
| `svct3` (waitlist.c) | ENDTASK SVCT3 CKIMUSE | DRIFTFLG | - | - | - | task svct3, job nbdonly |

### Shared erasables read and written

Accessors of `include/agc/core/state.h` and the lanes' public headers called by this lane, with the number of its functions that read / write each. Lane-private erasables are `static` in the lane's files or in its state struct; their scales are on the declarations.

| erasable (accessor) | meaning and scale (accessor prolog) | read by | written by |
|---|---|---|---|
| `agsword` | none -- AGSWORD (LM): the downlist to return to after AGS initialization. RETURNS: AGSWORD; returns AGSWORD | 1 | 2 |
| `cm_gymdt` | P40TMP (CSM, DP cs), SAVET-30 (LM, DP cs), ZOOMTIME (LM pad load, cs), S61DT and CM/GYMDT (CSM entry, cs): read by the restart tables. | 1 | 0 |
| `dsptab` | display panel relay buffer DSPTAB +0..+10D, +11D is the C/S lights word (15-bit relay words) Index 0..11. | 5 | 3 |
| `extvbact` | EXTVBACT (ERASABLE_ASSIGNMENTS; TESTXACT p.237 C / p.264 L) -- the extended verb activity bits, read RETURNS: EXTVBACT, 15 bits; returns EXTVBACT, 15  | 1 | 2 |
| `failreg` | three alarm code registers FAILREG +0..+2 (octal alarm codes) Index 0..2. | 1 | 2 |
| `imodes30` | last-sampled channel 30 bits, T4RUPT IMUMON (IMODES30, B(1)PRM) | 3 | 3 |
| `imodes33` | last-sampled channel 33 / DAP-enable bits, T4RUPT C33TEST (IMODES33, B(1)PRM) | 2 | 3 |
| `imucadr` | C(IMUCADR) = MODECADR +0 (ERASABLE_ASSIGNMENTS.agc; read by SVCT3).; returns agc_sp_t -- the word (+0: nobody uses the IMU) | 2 | 0 |
| `inlink` | INLINK (045): uplink word for UPRUPT, raw 15-bit word. | 1 | 0 |
| `markstat` | mark status bit word (MARKSTAT, B(1)PRM; SXTMARK/R21/R22/P20/P23) | 0 | 1 |
| `modecadr` | C(MODECADR +dev) (ERASABLE_ASSIGNMENTS.agc "MODE SWITCHING ERASABLE").; returns agc_sp_t -- the word (+0 none, positive a waiting job) | 2 | 1 |
| `modreg` | MODREG major mode (program) | 4 | 4 |
| `optind` | OPTIND (ERASABLE_ASSIGNMENTS.agc, CM) -- coarse optics state, read RETURNS: OPTIND: -1, 0, 1, or AGC_OPTIND_NEG0 for -0; returns OPTIND: -1, 0, 1, or  | 0 | 1 |
| `p40tmp` | P40TMP (CSM, DP cs), SAVET-30 (LM, DP cs), ZOOMTIME (LM pad load, cs), S61DT and CM/GYMDT (CSM entry, cs): read by the restart tables. | 2 | 0 |
| `s61dt` | P40TMP (CSM, DP cs), SAVET-30 (LM, DP cs), ZOOMTIME (LM pad load, cs), S61DT and CM/GYMDT (CSM entry, cs): read by the restart tables. | 1 | 0 |
| `savet_30` | P40TMP (CSM, DP cs), SAVET-30 (LM, DP cs), ZOOMTIME (LM pad load, cs), S61DT and CM/GYMDT (CSM entry, cs): read by the restart tables. | 2 | 0 |
| `tgo` | time to go in the burn / guidance, centiseconds, scale 2^28 (TGO) | 1 | 0 |
| `thrust` | THRUST (LM 055): descent engine throttle command pulses, SP. | 0 | 1 |
| `trkmkcnt` | TRKMKCNT: rendezvous tracking mark count (noun 45 R1), whole number. | 0 | 1 |
| `zoomtime` | P40TMP (CSM, DP cs), SAVET-30 (LM, DP cs), ZOOMTIME (LM pad load, cs), S61DT and CM/GYMDT (CSM entry, cs): read by the restart tables. | 1 | 0 |

### AGC labels -> C functions

The "AGC" column above names the label each listed function translates; the complete label -> function -> file:line index for the lane (and the tree) is [docs/overview/xref.md](../overview/xref.md), generated by `tools/xref.py`.
