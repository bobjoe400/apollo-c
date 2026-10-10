# Assignment: exec-sched   (tier: translation)

Scope: the AGC cooperative scheduler and recovery services. Translate (diff
Comanche vs Luminary; translate once, note differences): EXECUTIVE.agc,
WAITLIST.agc, PHASE_TABLE_MAINTENANCE.agc, RESTARTS_ROUTINE.agc,
RESTART_TABLES.agc, ALARM_AND_ABORT.agc, INTERRUPT_LEAD_INS.agc,
FRESH_START_AND_RESTART.agc.

Targets in src/exec/:
- executive.c: job table (7 core sets + dummy), 5 VAC areas, priorities,
  NOVAC FINDVAC CHANG1/CHANG2 JOBSLEEP JOBWAKE PRIOCHNG ENDOFJOB DUMMYJOB.
- waitlist.c: LST1/LST2 delta-time model, TWIDDLE WAITLIST VARDELAY
  FIXDELAY LONGCALL, T3RUPT processing, TASKOVER.
- phase_table.c: PHASCHNG, NEWPHASE, the six restart groups.
- restarts.c: RESTART routine, restart tables as static const data, GOPROG.
- alarm.c: ALARM PRIOLARM BAILOUT POODOO and FAILREG bookkeeping (the
  AGC's error-reporting and software-restart services).
- fresh_start.c: SLAP1 DOFSTART ENEMA STARTSB etc.
- interrupts.c: INHINT/RELINT counter; one plain function per interrupt
  lead-in, callable by the sim harness.
- dispatch.c: ONE switch over agc_job_id_t and ONE over agc_task_id_t.

Contract: include/agc/exec/exec.h and include/agc/exec/jobs.h. You may add
prototypes to agc_exec.h (list each in your note) and job/task ids to
agc_jobs.h only for jobs/tasks defined in your own files.

Keep AGC semantics (7 core sets, 5 VACs, 9 waitlist entries, TIME3 =
16384-(T1-T), LST1 deltas, real octal alarm codes 1201 1202 1203 31202 ...
as named constants) in clear C. At the top of executive.c explain how
pre-emption (CHANG1/CHANG2) is modelled: a C job body runs to completion or
to an explicit sleep/yield; agc_exec_run_one() picks the highest-priority
ready job; state what is preserved and what is lost.

Tests: tests/exec/exec.c: priority ordering, VAC exhaustion -> 1201,
waitlist ordering and delta arithmetic, overflow -> 1203, phase change /
restart group behaviour. Link your own objects if other lanes break the
build.
