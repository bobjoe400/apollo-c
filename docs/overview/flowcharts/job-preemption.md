# Job creation and pre-emption (executive)

AGC: EXECUTIVE.agc (C pp.1208-1220, L pp.1103-1116): NOVAC, FINDVAC,
NOVAC2, SPECTEST, SETLOC, CHANJOB, ENDOFJOB, EJSCAN, JOBSLEEP, JOBWAKE,
CHANG1, PRIOCHNG, DUMMYJOB. C: `src/exec/executive.c`.

A job never loses the processor inside its C body. It is pre-empted at
the next `agc_exec_run_one` (the swap with core set NEWJOB happens before
dispatch) or, if it offers the processor with `agc_exec_job_yield`
(CHANG1), right after it returns. The decision of *who* runs next is the
AGC's: the priority word (priority << 9 + VAC rank, negative while
asleep) compared exactly as SETLOC and EJSCAN compare it (`prio_word`).

## Creating a job

```mermaid
flowchart TD
    A["caller: NOVAC / FINDVAC<br/>executive.c:agc_exec_novac / agc_exec_findvac"] --> B{"FINDVAC: free VAC area?<br/>VAC1USE..VAC5USE in order"}
    B -- "none" --> B1["BAILOUT 1201<br/>alarm.c:agc_bailout -> software restart"]
    B -- "VAC k claimed / NOVAC" --> C["NOVAC2: scan core sets 0..n-1<br/>executive.c:novac2<br/>(n = 7 CSM, 8 LM: agc_exec_core_sets)"]
    C --> D{"free core set?<br/>(PRIORITY = -0)"}
    D -- "none" --> D1["BAILOUT 1202<br/>alarm.c:agc_bailout"]
    D -- "core set i" --> E["load PRIORITY, LOC; LOCCTR = i<br/>(caller may then set the job's MPAC word:<br/>agc_exec_set_new_job_arg)"]
    E --> F{"i = 0 ?"}
    F -- "yes: nothing was running" --> G["SPECTEST<br/>executive.c:spectest<br/>NEWJOB -0 -> +0 (this job runs next)"]
    F -- "no" --> H["SETLOC<br/>executive.c:setloc_compare"]
    H --> I{"prio_word(i) > prio_word(NEWJOB or 0) ?"}
    I -- "yes" --> J["NEWJOB = i<br/>(pre-emption pending)"]
    I -- "no" --> K["NEWJOB unchanged<br/>(job waits)"]
```

## Running jobs

```mermaid
flowchart TD
    S["agc_exec_run_one<br/>(ADVAN / NUDIRECT; called by src/sim/clock.c:clock_exec)"] --> T{"NEWJOB"}
    T -- "-0" --> T0["DUMMYJOB: return NOT_FOUND (idle)<br/>clock.c:clock_exec runs agc_selfcheck_idle_pass"]
    T -- "+k" --> U["CHANJOB: swap core set 0 and k, NEWJOB = +0<br/>executive.c:chanjob  (PRE-EMPTION)"]
    T -- "+0" --> T1{"core set 0 a self-rescheduled loop<br/>that already ran this cs?"}
    U --> T1
    T1 -- "yes" --> T2["return BUSY: next pass in the next cs<br/>(DEVIATION: job pacing)"]
    T1 -- "no" --> V["activity light on; dispatch core set 0 at LOC<br/>dispatch.c:agc_dispatch_job -> agc_job_*(ctx)"]
    V --> W["interpreter alarms of the job<br/>alarm.c:exec_service_interp_alarm<br/>(1301 ALARM, 1302 POODOO)"]
    W --> X{"software restart requested?<br/>alarm.c:agc_exec_restart_pending"}
    X -- "yes" --> X1["ENEMA now<br/>alarm.c:agc_exec_service_restart<br/>(see software-restart.md)"]
    X -- "no" --> Y{"how did the job leave?"}
    Y -- "JOBSLEEP<br/>(agc_exec_job_sleep)" --> Y1["EJSCAN: best waiting job<br/>executive.c:ejscan_change"]
    Y -- "CHANG1<br/>(agc_exec_job_yield)" --> Y2{"NEWJOB > 0 ?"}
    Y2 -- "yes" --> Y3["CHANJOB: yielding job swapped out<br/>executive.c:chanjob"]
    Y2 -- "no" --> Y4["job continues next pass<br/>at its resume LOC"]
    Y -- "returned (ENDOFJOB)" --> Y5["free core set 0 and its VAC area<br/>executive.c:end_of_job"]
    Y5 --> Y1
    Y1 --> Z{"any active job above -0?<br/>executive.c:ejscan"}
    Z -- "no" --> Z1["NEWJOB = -0, activity light off"]
    Z -- "core set b" --> Z2["CHANJOB b"]
```

Other entries that move NEWJOB:

* `agc_exec_job_wake` (JOBWAKE): finds the sleeping core set whose LOC
  is the given id, makes it active and treats it as a new job (SPECTEST
  or SETLOC); not found is NOT_FOUND (`agc_exec_job_wake_if_asleep` turns
  that into OK where the AGC does not care).
* `agc_exec_prio_change` (PRIOCHNG): new priority for the running job;
  if a waiting job is now strictly higher NEWJOB names it, and the caller
  yields with `agc_exec_job_yield` and returns (the AGC changes job
  inside PRIOCHNG).

Deviation (executive.c NOTES): a long C job body delays a newly created
higher-priority job until it returns or yields; MPAC, OVFIND and the VAC
contents are not saved by the executive (a job keeps its data in locals
or module erasables). A job whose last pass ended in CHANG1 to its own
LOC (a polling or recomputing loop: R31/R34 V83 COMPDISP, CHECKG1's PIPA
catch) gets its next pass in the next centisecond (`agc_exec_run_one`
returns BUSY until then): a C pass takes no AGC time, so without it such
a loop ran up to the host's job bound where the AGC runs it a few times a
second. With the landing radar read wait (`landing_radar.c:agc_job_lrvjob`)
it is one of the two ordering deviations that stand in for AGC execution
time (docs/ARCHITECTURE.md, "Known limitations").
