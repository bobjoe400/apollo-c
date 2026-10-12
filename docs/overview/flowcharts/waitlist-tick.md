# Waitlist tick (TIME3, T3RUPT)

AGC: WAITLIST.agc (C pp.1221-1235, L pp.1117-1132): WAITLIST, WAIT2,
WTLST2/4/5, T3RUPT, T3RUPT2, TASKOVER, ENDTASK/SVCT3, LONGCALL. C:
`src/exec/waitlist.c`; the counter is driven by `src/sim/clock.c`.

The lists are the AGC's: TIME3 = 16384 - (T1 - T) counts up to the first
task; LST1 +0..+7 hold -(T(k+2) - T(k+1)) + 1, the deltas between the
next tasks; LST2 +0..+8 the task ids. An empty slot holds ENDTASK and
NEG1/2 (-8192), so with nothing scheduled ENDTASK runs every 81.93 s.

## Scheduling a task

```mermaid
flowchart TD
    A["WAITLIST / TWIDDLE<br/>waitlist.c:agc_waitlist_call(dt, task)"] --> B{"dt = 0 ?"}
    B -- "yes" --> B1["POODOO 1204<br/>alarm.c:agc_poodoo"]
    B -- "no" --> C{"dt <= 16250 cs (DTMAX)?"}
    C -- "no" --> C1["BAD_ARG (DEVIATION: AGC restriction,<br/>not tested by the AGC)"]
    C -- "yes" --> D["INHINT; WAIT2<br/>waitlist.c:wait2"]
    D --> E["a = TD - T1 + 1<br/>(post-overflow TIME3 < 200 octal handled as the AGC does)"]
    E --> F{"a <= 0 ?<br/>(before task 1)"}
    F -- "yes" --> F1["TIME3 = 16384 - dt;<br/>insert as entry 0 (WTLST4)"]
    F -- "no" --> G["walk LST1: a = a - 1 + LST1[n]<br/>until a <= 0 (WTLST2)"]
    G --> H["insert at n, adjust LST1[n-1]<br/>waitlist.c:wtlst4"]
    F1 --> I{"last LST2 slot still ENDTASK?"}
    H --> I
    I -- "no: list full" --> I1["BAILOUT 1203<br/>alarm.c:agc_bailout"]
    I -- "yes" --> J["shift entries down, store task; RELINT"]
```

`agc_waitlist_vardelay` / `agc_waitlist_fixdelay` are the same call made
from inside a task (they assert interrupts are inhibited);
`agc_waitlist_longcall` keeps one long delay in LONGTIME/LONGCADR and
re-schedules LONGCYCL every 81.92 s (`longcycl`) until the remainder fits,
then GETCADR runs the task. `agc_waitlist_kill` (LM KILLTASK) replaces
the first matching LST2 entry with TASKOVER.

## The tick and T3RUPT

```mermaid
flowchart TD
    K["sim/clock.c:time1_phase, every 10 ms"] --> L{"TIME3 at its last count?"}
    L -- "no" --> L1["PINC TIME3<br/>waitlist.c:agc_waitlist_tick(1)"]
    L -- "yes" --> L2["request T3RUPT; when taken:<br/>clock.c:rupt_t3 -> agc_waitlist_tick(1)<br/>(TIME3 -> 0: overflow)"]
    L2 --> M["T3RUPT<br/>waitlist.c:agc_waitlist_t3rupt (INHINT)"]
    M --> N["a = LST1[0]; shift LST1 up, LST1[7] = NEG1/2<br/>TIME3 = TIME3 + a + POSMAX (ADS TIME3)"]
    N --> O["RUPTAGN = the sum overflowed<br/>(next task due in this same cs)"]
    O --> P["task = LST2[0]; shift LST2 up, LST2[8] = ENDTASK<br/>GETCADR -> the LONGCALL task"]
    P --> Q["dispatch the task<br/>dispatch.c:agc_dispatch_task"]
    Q --> R["interpreter alarms of the task<br/>alarm.c:exec_service_interp_alarm"]
    R --> S{"restart requested or<br/>STARTSB2 ran inside the task?"}
    S -- "yes" --> S1["stop the RUPTAGN chain"]
    S -- "no" --> T{"RUPTAGN and fewer than 9 passes?"}
    T -- "yes" --> N
    T -- "no" --> U["RELINT"]
    S1 --> U
    U --> V{"restart pending?"}
    V -- "yes" --> V1["ENEMA<br/>alarm.c:agc_exec_service_restart"]
    V -- "no" --> W["return first non-OK task status"]
```

ENDTASK (`agc_task_endtask` -> `svct3`): with DRIFTFLG set, start the IMU
drift-compensation job NBDONLY (NOVAC PRIO35) unless the IMU is in use
(IMUCADR non-zero), in which case retry in 5 s (`agc_task_svct3`).
