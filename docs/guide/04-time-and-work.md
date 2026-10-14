# 4. Time and work

The AGC flies a spacecraft with one processor. While the guidance
recomputes the steering every two seconds, the autopilot must fire jets
every tenth of a second, the DSKY must be refreshed, a key press must be
answered, and the engine must be shut off at the right centisecond. The
software meets all of this with three mechanisms: hardware **timers and
interrupts**, a time-ordered list of short **tasks** (the waitlist), and
a priority-ordered set of longer **jobs** (the executive). This chapter
explains each, then what the C changes about them.

## 4.1 Clocks

The hardware has a 3200 Hz *scaler* and six counter-timers, erasable
words that the hardware increments. The C has no clock of its own: the
host harness plays the hardware, and one call of `sim_clock_tick` is one
centisecond, 32 scaler counts (`sim/clock.c`, NOTES):

| Counter | Rate | Overflow does | C |
|---|---|---|---|
| TIME2, TIME1 | 1 cs | (none: a 28-bit mission clock, wraps after about 745.7 h) | `agc_time_now_cs()`, `core/state.c` |
| TIME3 | 1 cs | T3RUPT: the next waitlist task is due | `exec/waitlist.c` |
| TIME4 | 1 cs | T4RUPT: displays, PRO key, IMU and optics monitors | `io/service/t4rupt.c` |
| TIME5 | 1 cs | T5RUPT: the digital autopilot | T5LOC / T5ADR, `exec/interrupts.c` |
| TIME6 | 1/1600 s, while enabled | T6RUPT: end of an LM jet firing | `lm/dap/rcs/t6rupt.c` |

The program reads TIME2/TIME1 as "now" (the AGC's DP time, centiseconds
at 2^28, chapter 2) and loads the other counters with the complement of
a delay, so that they overflow when the delay has passed. The scaler
phases at which each counter steps are those of the yaAGC emulator, so
interrupts arrive in the order they do there ("Timing follows yaAGC's
scaler", `sim/clock.c`).

## 4.2 Interrupts

An interrupt stops whatever is running, saves the A and L registers,
and jumps to a fixed vector at address 4000 onward, one per source
(INTERRUPT_LEAD_INS.agc, C p.131 / L p.153). The handler runs with
further interrupts inhibited and ends with RESUME. Interrupts are
taken in vector order when several are pending:

| Interrupt | Raised by | Handler (C) |
|---|---|---|
| GOJAM | power-on, hardware restart | `agc_rupt_go` -> `agc_restart_goprog` (chapter 7) |
| T6RUPT | TIME6 | LM: `agc_lm_dap_t6rupt` (jet timing); CSM: through T6LOC |
| T5RUPT | TIME5 | the task in T5LOC/T5ADR: the CSM RCS or TVC DAP, the LM DAP |
| T3RUPT | TIME3 | `agc_waitlist_t3rupt` (4.3) |
| T4RUPT | TIME4 | `agc_t4rupt_rupt`: "the 120 ms interrupt" (`io/service/t4rupt.c`) |
| KEYRUPT1 | a main-DSKY key | `agc_dsky_keyrupt1` (chapter 6) |
| KEYRUPT2 / MARKRUPT | nav-DSKY key, optics mark button | CSM `agc_sxtmark_markrupt`, LM `agc_aotmark_markrupt` |
| UPRUPT | an uplinked word | `agc_dsky_uprupt` |
| DOWNRUPT | the telemetry wants the next word pair | `agc_telem_downrupt` |
| RADARUPT | a radar reading is ready | CSM `agc_rendz_vhf_read`, LM `agc_rendz_radar_read` |
| HANDRUPT (RUPT10) | hand controller / traps | LM `agc_lm_pitfall` |

In C each vector is a lead-in function in `exec/interrupts.c`
(`agc_rupt_t6` .. `agc_rupt_rupt10`) that inhibits interrupts, calls the
handler, and leaves through `rupt_exit`, the C form of RESUME. Handlers
that belong to higher layers are reached through one switch,
`agc_dispatch_rupt` -- the same device as the job switch, so the
low-level exec never names a function above it except there:

```c
    switch (rupt)
    {
        case AGC_RUPT_T6:                                              /* LM DOT6RUPT; CSM goes through T6LOC     */
            st = lm ? agc_lm_dap_t6rupt() : AGC_STATUS_NOT_FOUND;      /* LM : CSM */
            break;
        case AGC_RUPT_T5:                                              /* T5LOC / T5ADR                           */
            st = agc_dispatch_task(agc_rupt_t5_task());
            break;
        case AGC_RUPT_T3:
            st = agc_waitlist_t3rupt();
            break;
        ...
```

INHINT and RELINT (disable and enable interrupts) appear throughout the
listing around code that must not be interrupted. In C they are
`agc_inhint()` / `agc_relint()`, and they are *counted*: "nested C calls
(a task calling WAITLIST, which itself inhibits) keep interrupts
inhibited until the outermost RELINT" (`exec/interrupts.c`, NOTES).

## 4.3 The waitlist: tasks at a time

A **task** is a short piece of code that must run at a given time:
"TIG-5 s: flash V99", "in 2 s: read the accelerometers", "now + TGO:
engine off". A task runs inside the T3RUPT interrupt, so it must be brief
and must not wait for anything; anything long it hands to a job.

WAITLIST takes a delay in centiseconds (in A) and the task's address
(the 2CADR after the call) and inserts it into a list sorted by time.
The list is the AGC's own delta-time structure, kept exactly
(`exec/waitlist.c`, NOTES):

```c
 *       C(TIME3)    = 16384 - (T1 - T)     centiseconds, T = now
 *       C(LST1 +k)  = -(T(k+2) - T(k+1)) + 1          k = 0..7
 *       C(LST2 +k)  = task k+1                        k = 0..8
```

TIME3 counts up to the first task; LST1 holds the gaps between the next
eight; LST2 holds their addresses (in C, task ids). So there are at
most nine tasks, and the tenth is BAILOUT 1203; a zero delay is
POODOO 1204; the longest delay is 162.5 s, and longer waits use LONGCALL,
which re-arms itself every 81.92 s until the remainder fits. An empty
slot holds ENDTASK, so with nothing scheduled ENDTASK runs every 81.93 s
and uses the occasion for the IMU drift compensation (`svct3`).

When TIME3 overflows, T3RUPT (`agc_waitlist_t3rupt`, T3RUPT / T3RUPT2 /
TASKOVER, C pp.1231-1232) takes task 1, shifts both lists up, reloads
TIME3 from the next gap, and -- if that gap is zero, the next task being
due in the same centisecond (RUPTAGN) -- repeats:

```c
        int32_t sum         = (int32_t) time3 + a + WL_POSMAX; /* ADS TIME3      */

        ruptagn            = sum >= WL_TIME3_OVF;                             /* RUPTAGN = +1     */
        time3              = (uint16_t) (ruptagn ? sum - WL_TIME3_OVF : sum); /* overflow leaves the excess in TIME3 */
        agc_task_id_t task = lst2[0];                                         /* LST2 +0: the task to run */
        ...
        agc_status_t       st  = agc_dispatch_task(task);     /* the task's own status */
```

A task is typically a link in a chain. The SPS burn's task 35 s before
ignition blanks the DSKY and schedules the next link; listing and C side
by side (P40-P47.agc C p.690; `csm/burns/sps/burn.c:agc_task_tigblnk`):

```
TIGBLNK    CAF     5SEC        # CALL TIGAVEG IN FIVE SEC AT TIG-30
           TC      WAITLIST
           2CADR   TIGAVEG
           CAF     ZERO        # DISABLE HERE, NOT IN P40BLNKR
           TS      NVWORD1
           CAF     PRIO14
           TC      NOVAC
           2CADR   P40BLNKR    # DON'T PROTECT -- RESTARTS BLANK DSKY
           CS      OCT37       # 4.37 = TIGAVEG (500CS)
```

```c
agc_status_t agc_task_tigblnk(void)
{
    agc_status_t st = agc_waitlist_call(SEC5, AGC_TASK_TIGAVEG); /* status of agc_waitlist_call */
    if (st != AGC_STATUS_OK)
    {
        return (st);
    }
    agc_guid_state()->csm.nvword1 = 0;                                                      /* DISABLE HERE           */
    agc_status_t sw               = agc_exec_novac(AGC_PRIO14, AGC_JOB_PREBURN_BLANK_DSKY); /* status of agc_exec_novac */
    ...
    agc_phase_newphase(4, 037, true);
    return (AGC_STATUS_OK); /* 4.37 = TIGAVEG         */
}
```

The 2CADR (two-word address of the task or job) becomes an enum id;
the label P40BLNKR becomes a descriptive name (`AGC_JOB_PREBURN_BLANK_DSKY`,
`docs/NAMING.md`); the last line records restart point 4.37 (chapter
7). The C checks every status, as Power of Ten requires: a full
waitlist returns BAILOUT, which the caller passes up.

## 4.4 The executive: jobs by priority

A **job** is anything longer: a guidance pass, a display, a Lambert
solution. Jobs run outside interrupts and are scheduled by priority.
The executive's state, described in the NOTES of `exec/executive.c`:

* **Core sets.** A job's context is a *core set* of 12 words: MPAC, LOC
  (where it is), BANKSET, PUSHLOC, PRIORITY. Comanche has 7, Luminary 8
  (`AGC_EXEC_CORE_SETS_CSM`, `_LM`). "Core set 0 always belongs to the job
  that is running": switching jobs means swapping a core set into slot 0.
* **VAC areas.** A job that uses the interpreter needs a 43-word VAC
  area for its push-down list; there are five.
* **Starting a job.** NOVAC starts a job without a VAC area, FINDVAC
  with one. "No VAC area: BAILOUT 1201"; no free core set: BAILOUT 1202
  -- the executive-overflow alarms that `docs/ARCHITECTURE.md` lists
  among the behaviours that come from execution time (4.6).
* **Priority.** PRIORITY holds the priority number (PRIO14, PRIO20, ...)
  in bits 10-14 and in the low bits an address that breaks ties; it is
  -0 for a free core set and negative for a sleeping job. In C,
  `prio_word(i)` rebuilds exactly that word so that ties are broken as
  the AGC breaks them.
* **NEWJOB** says what should run: +0 keep running core set 0, +k
  switch to core set k (a higher-priority job is waiting), -0 nothing
  to do (DUMMYJOB, the idle loop, which runs the self-check and keeps
  the COMP ACTY light off).
* **Leaving.** A job ends with ENDOFJOB (its core set is freed and the
  best waiting job runs), suspends itself with JOBSLEEP (until another
  job or task names it in JOBWAKE), or offers the processor with
  CHANG1, continuing later from where it stopped.

`agc_exec_novac` (NOVAC, C p.1208 / L p.1103) finds the first free core
set and decides whether the new job pre-empts (`exec/executive.c:novac2`):

```c
    for (int i = 0; i < n && found < 0; i++)
    {
        if (core[i].state == AGC_CORE_FREE)
        {
            found = i;
        }
    }
    if (found < 0)
    {
        return (agc_bailout(AGC_ALARM_01202)); /* NO CORE SETS */
    }
    ...
    if (found == 0)
    {
        return (spectest());
    }
    setloc_compare(found);
```

## 4.5 How a C job is written

In the AGC a job is a stretch of code that may stop anywhere -- to wait
for the crew, for the IMU, for the integration -- and later continue on
the next line. C cannot stop in the middle of a function and resume
there, and the project forbids the tricks that would allow it. So:

* A job is a function `agc_status_t agc_job_<name>(agc_job_ctx_t *ctx)`
  that runs to completion or to a sleep/yield point; returning
  `AGC_STATUS_OK` is ENDOFJOB (`docs/TRANSLATION_GUIDE.md`, section 4).
* A job that waits is **split into one function per resumption point**.
  The waiting primitive is told which job id to continue as, and the
  function returns. JOBSLEEP is `agc_exec_job_sleep(wake_as)`; CHANG1 is
  `agc_exec_job_yield(resume_as)`; both just record the continuation
  (`exec/executive.c`):

  ```c
  void agc_exec_job_sleep(agc_job_id_t wake_as)
  {
      core[0].loc   = wake_as;           /* TS LOC              */
      core[0].state = AGC_CORE_SLEEPING; /* CS PRIORITY         */
  }
  ```

  The AGC stores a return address in LOC; the C stores the id of the
  function that continues.

A typical wait is INTSTALL, which reserves the orbital integrator. If
another job is integrating, the caller sleeps until the integration
wakes it. The landing program's ignition algorithm
(`lm/landing/lunar_landing.c:ll_ignalg_prec`, IGNALG, L pp.785-786)
reads:

```c
    agc_status_t  st    = agc_integ_lemprec(ip, agc_integ_shared_ctx(), tdec1, AGC_JOB_IGNALG_PREC); /* step status */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* asleep; resumes as IGNALGP    */
    }
```

`AGC_STATUS_BUSY` means "I put you to sleep"; the job returns at once,
and when the integration finishes, JOBWAKE starts `AGC_JOB_IGNALG_PREC`,
which runs the same step again with the integrator free. Data that must
survive the gap goes into module erasables, never locals.

Displays work the same way: a job that flashes a question to the crew
passes three continuation ids (terminate, proceed, enter) and returns
(chapter 6).

`agc_exec_run_one` (DUMMYJOB / ADVAN / NUCHANG2 / NUDIRECT, C p.1220)
is the scheduler loop body: if NEWJOB names another core set, swap it
in (CHANJOB, the pre-emption point); dispatch the job in core set 0; then
do what the job's exit did in the AGC:

```c
        if (core[0].state == AGC_CORE_SLEEPING)
        {
            ejscan_change(); /* JOBSLP1 -> EJSCAN   */
        }
        else if (yielded)
        {
            ...
            if (newjob > 0)
            {
                chanjob(newjob); /* CHANG1 -> CHANJOB   */
            }
        }
        else
        {
            end_of_job(); /* ENDOFJOB            */
        }
```

The dispatch itself, from an id to a function, is the generated switch
in `exec/dispatch.c` ("the C form of the AGC's 2CADR words (rule 9: no
function pointers)"), produced from one table, `tools/registry.tsv`,
whose row for P40 reads `job AGC_JOB_SPS_BURN agc_job_sps_burn - csm/burns
P40CSM (V37)`: id, CSM function, LM function, folder, AGC label.

## 4.6 Zero-time jobs

The single largest difference between the C and the AGC is **time**.
In the AGC, a guidance pass takes a sizeable fraction of a second of
processor time, during which interrupts come and go and the interpreter
checks NEWJOB between every pair of instructions (chapter 3) so that a
more urgent job can take over. In the C a job is a function call: it
runs to its end in no AGC time, and interrupts are taken only between
jobs and between scaler counts (`sim/clock.c`, "DEVIATION (timing)").
The NOTES of `exec/executive.c` state what is kept and what is lost:

> What is preserved: priorities and the priority word ordering, NEWJOB,
> core-set positions and CHANJOB swaps, NOVAC/FINDVAC semantics, VAC
> claiming order, 1201/1202 overflow, sleep/wake by LOC, PRIOCHNG, the
> activity light (channel 11 bit 2).
> What is lost: pre-emption in the middle of a job body.

Because the decision of *what* runs next is the AGC's, the decisions a
program makes come out the same; what changes is *when* things happen
within a centisecond, and anything that exists only because computing
took time: an executive overload (1201, 1202) cannot arise from slow
computation, and a computation can never miss a deadline. Such
differences are treated as expected deviations; instruction timing is
not modelled ("yaAGC remains the tool for timing-exact runs",
`docs/ARCHITECTURE.md`, "Known limitations").

In two places the order of events itself depended on execution time,
and the C needed a stand-in for it. Both are documented in the code:

1. **The landing-radar reads.** During powered descent SERVICER reads
   the radar's altitude (LRHJOB) and then a velocity beam (LRVJOB); both
   wait for the radar in RADSTALL, and the radar serves one job at a
   time. In the AGC the two never met, because SERVICER spent tens of
   milliseconds on navigation between them. The C SERVICER reached
   LRVJOB at once, the second RADSTALL was BAILOUT 01210 ("two programs
   using one device") on every pass, and no radar update was ever
   accepted. The fix makes LRVJOB wait while the radar is held
   (`lm/burns/servicer/landing_radar.c:agc_job_lrvjob`, LRVJOB L p.892):

   ```c
       if (agc_erase_modecadr(AGC_IMU_DEV_RADAR) > 0)
       { /* LRHJOB still in RADSTALL: wait for it (DEVIATION above) */
           return (agc_svc_delayjob(LRV_WAIT_CS, AGC_JOB_LRVJOB));
       }
   ```

2. **Jobs that reschedule themselves.** Some jobs loop by yielding to
   their own entry point (CHANG1): the rendezvous display V83's
   recompute loop, CHECKG1's accelerometer check. An AGC pass of such a
   loop takes tens of milliseconds, so it runs a few times a second; a
   C pass takes nothing, so it would run as often as the host allowed.
   `agc_exec_run_one` gives such a job at most one pass per centisecond:

   ```c
       if (core[0].again && core[0].again_cs == agc_time_now_cs())
       {
           return (AGC_STATUS_BUSY); /* DEVIATION: next pass in the next cs */
       }
   ```

`docs/ARCHITECTURE.md` ("Known limitations") lists these two as the
only ordering stand-ins; everything else that differs only in time is
an expected deviation.

## 4.7 One cycle, end to end

Putting the pieces together, one powered-flight cycle of the SPS burn
(chapter 9) goes:

1. TIME3 overflows; T3RUPT runs the READACCS task
   (`csm/burns/servicer.c:agc_task_readaccs_csm`, READACCS C p.824),
   which reads and zeroes the accelerometer counters, records restart
   point 5.5, schedules itself again in 2 s and FINDVACs the SERVICER
   job (CHEKAVEG, `docs/overview/flowcharts/sps-burn.md`).
2. When the running job returns (or at once, if the computer was idle),
   `agc_exec_run_one` sees NEWJOB and dispatches SERVICER, which updates
   position and velocity and continues into the burn's steering job
   through AVGEXIT.
3. Meanwhile TIME5 keeps firing and the TVC autopilot runs in the
   T5RUPT, its pitch and yaw axes alternating every 20 ms
   (`csm/dap/tvc/daps.c`, file prolog); TIME4 fires every 120 ms and the
   DSKY relays and IMU monitors run in the T4RUPT; and a waitlist task
   set for the computed cut-off time shuts the engine off.

Next: [5. Memory and flags](05-memory-and-flags.md).
