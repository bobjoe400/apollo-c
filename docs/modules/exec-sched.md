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
| MR.KLEAN, P00KLEAN, V37KLEAN (FRESH C185/L213) | agc_phase_mr_klean / _p00_klean / _v37_klean | phase_table.c | group sets differ CSM/LM |
| RESTARTS ... PHSPART2 (RESTARTS_ROUTINE C1414/L1303) | agc_restart_group | restarts.c | |
| SIZETAB, n.mSPOT tables (RESTART_TABLES C212/L239) | rst_table_lookup | restart_tables.c | static const, both vehicles |
| ALARM, ALARM1/2, VARALARM, PRIOLARM, CURTAINS (A&A C1493/L1381) | agc_alarm, agc_prioalarm, agc_curtains | alarm.c | FAILREG[3], PROG light |
| BAILOUT, BAILOUT1, WHIMPER, ABORT (C1495/L1382) | agc_bailout, agc_abort_restart | alarm.c | -> ENEMA |
| POODOO, POODOO1, GOPOODOO, STRTIDLE, FLAGS, CCSHOLE (C1495/L1383) | agc_poodoo, agc_ccshole, AGC_JOB_GOPOODOO | alarm.c | |
| interrupt lead-ins at 4000 (INTERRUPT_LEAD_INS C132/L153) | agc_rupt_* | interrupts.c | |
| SLAP1, DOFSTART, DOFSTRT1, STARTSUB, STARTSB1, STARTSB2, ENEMA, GOPROG, GOPROG3, LIGHTSET, ENDRSTRT, GOTOPOOH, GOP00FIX, VAC5STOR, T5IDLOC (FRESH C183-205/L211-225) | agc_fresh_start, agc_restart_goprog, fresh_enema, ... | fresh_start.c | foreign erasables delegated |
| V37 / V37XEQ / POOH etc. (FRESH C197-208/L227-236) | agc_v37 | fresh_start.c | STUB |
| job/task dispatch | agc_dispatch_job / agc_dispatch_task | dispatch.c | one switch each |

## Progress

- DONE executive.c (NOVAC FINDVAC SPECTEST CHANJOB EJSCAN ENDOFJOB JOBSLEEP JOBWAKE PRIOCHNG CHANG1 DUMMYJOB)
- DONE waitlist.c (compiles; T3RUPT services a pending ENEMA; TASKOVER task added)
- DONE exec_pending.h (provisional foreign job/task ids, 0x0E00 range)
- DONE agc_exec.h second-pass prototypes; agc_jobs.h AGC_JOB_ENDOFJOB, AGC_TASK_TASKOVER
- DONE phase_table.c (PHASCHNG A/B/C, 2PHSCHNG, NEWPHASE, KLEANs, NEWMODE, CHECKMM, SETUPDSP)
- DONE restart_tables.c (both vehicles, static const; 5 indirect erasables UNIMPLEMENTED pending accessors)
- TODO restarts.c, alarm.c, fresh_start.c, interrupts.c, exec_erasable.c, v37_p00.c, dispatch.c
- TODO tests/test_exec.c

## Requested core additions

(filled in as found)

## Requested jobs-tasks

(filled in as found)
