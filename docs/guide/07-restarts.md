# 7. Restarts and the phase tables

The AGC was designed to survive its own failures. A power transient, a
parity error, a full job table or an impossible branch makes the
computer *restart*, and the software arranges that a restart in the
middle of a burn or a landing does not end the burn or the landing: the
programs that were running pick up again from the last point they
declared safe. This chapter explains alarms and aborts, the two kinds of
restart, the phase tables that make them work, and how V37 uses the
same machinery to change program.

## 7.1 Alarms, BAILOUT and POODOO

ALARM_AND_ABORT.agc (C pp.1493-1496, L pp.1381-1385) offers four ways
to report trouble, all in `exec/alarm.c`:

| AGC | C | What happens |
|---|---|---|
| ALARM | `agc_alarm(code)` | record the code, light PROG, carry on |
| BAILOUT | `agc_bailout(code)` | record, then a software restart: the programs continue from their restart points |
| POODOO | `agc_poodoo(code)` | record, abandon the program: software restart into P00 |
| CURTAINS | `agc_curtains()` | ALARM 217 |

Alarm codes are octal and kept as such (`AGC_ALARM_01202`). Recording
(`store_alarm`, LARMENT .. MULTFAIL, C p.1493) puts the code in the first
free of three registers FAILREG, FAILREG+1, FAILREG+2 -- which the crew
reads with V05 N09 -- and on the first alarm turns on the PROG lamp:

```c
    if (agc_erase_failreg(0) == 0U)
    {
        agc_erase_set_failreg(0, code); /* first alarm       */

        /* PROGLARM (C p.1494 / L p.1382): CS DSPTAB +11D / MASK OCT40400 /
         * ADS -- PROG lamp on, lamp word marked for output.              */
```

The codes in this guide: 1201/1202 executive overflow, 1203 waitlist
overflow, 1204 zero waitlist delay, 1103 an impossible CCS branch
(CCSHOLE), 1107 phase table failure, 1301/1302 ARCSIN/SQRT out of range,
1520 V37 not allowed now, 01501/01502/01206 display-system misuse,
01406 and 01410 landing guidance (chapter 10), 00401 middle-gimbal limit.

## 7.2 Two kinds of restart

* A **hardware restart** -- GOJAM -- is what the hardware does at
  power-on, after a power transient, or after a hardware alarm: it
  starts the program at address 4000. In C it is `agc_rupt_go` ->
  `agc_restart_goprog` (GOPROG, `exec/fresh_start.c`): count it in
  REDOCTR, check for a restart loop (CSM channel 33) or the crew's
  "force a fresh start" key combination (MARK REJECT + ERROR RESET held,
  `lightset`), reset the timers, lists and displays (STARTSUB), and go
  to GOPROG3.
* A **software restart** -- ENEMA, entered through BAILOUT, POODOO,
  ABORT or V37 -- is the same without the hardware reset
  (`fresh_enema`, ENEMA / GOPROG2, C p.188 / L p.217): clear the
  executive and the waitlist (STARTSB2), then GOPROG3.

GOPROG3 (`exec/fresh_start.c:goprog3`, C pp.188-189 / L pp.218-219) is the
heart of both:

```c
    phase_tables_ok(&ok); /* PCLOOP; PTBAD's fresh start */
    ...
    agc_status_t st = fresh_mmdsplay(); /* TC MMDSPLAY          */
    ...
    st = agc_phase_restart_all(); /* NXTRST loop          */

    if (st == AGC_STATUS_NOT_FOUND)
    { /* no group active      */
        st = AGC_STATUS_OK;

        if (agc_erase_modreg() >= 0)
        { /* MODREG not -0        */
            st = agc_exec_gotopooh();
        }
    }
```

Check the phase tables; redisplay the major mode in PROG; restart every
active group; if none was active, go to P00. Everything that survives a
restart survives because it is in erasable (chapter 5, 5.8) and because
the phase tables say how to resume it.

## 7.3 Restart groups and phases

A program declares its restart points as it goes, by calling PHASCHNG
("phase change") with a code word. There are six **restart groups**,
independent threads of restartable activity. In these ropes group 4
carries the major mode's main sequence (the burn countdown, P52's
dialogue), group 5 SERVICER, group 2 rendezvous tracking, groups 3 and 6
burn and clock tasks (`docs/overview/flowcharts/sps-burn.md`,
`docs/overview/flowcharts/program-change.md`). Each group has a pair of erasables, PHASEn and
-PHASEn, that must always be complements -- a cheap integrity check --
and for some phases a stored job or task to restart (`exec/phase_table.c`,
NOTES):

```c
 *   The code word after TC PHASCHNG (15 bits, octal):
 *     type A  TL0 00P PPP PPP GGG   fixed phase P of group G
 *     type B  TL1 DAP PPP PPP GGG   variable job + the fixed phase P
 *     type C  TL0 1AD XXX CJW GGG   variable job (J), task (W) or LONGCALL (C)
 *   T: set TBASEG, L: set LONGBASE, D: a new priority / delta time is given,
 *   A: a restart 2CADR is given.
```

A phase is written "group.phase", both in octal: 4.37 is group 4, phase
037. In C the call keeps the code word: `agc_phase_change(code)`
(PHASCHNG), `agc_phase_newphase(group, phase, set_tbase)` (NEWPHASE, the
short form), `agc_phase_change_var(code, &v)` (a variable phase, with the
job or task named explicitly), and `agc_phase_kill(group)` /
the KLEAN routines to deactivate groups. The update runs with interrupts
inhibited, writing -PHASEn and PHASEn together (`phaschng_common`).

## 7.4 The restart tables

A *fixed* phase (type A) points into the rope's RESTART_TABLES: odd
phases have one entry, even phases two, and each entry is a job (with
priority; FINDVAC or NOVAC by sign), a task (with its delay measured
from the time the phase was set, TBASEn), or a LONGCALL. The C keeps
the tables as `static const` data (`exec/restart_tables.c`). The SPS
burn's TIGBLNK task (chapter 4) ends by setting phase 4.37; the table
row it names, side by side (RESTART_TABLES.agc C p.215):

```
4.37SPOT   DEC     500
           EBANK=  TIG
           -2CADR  TIGAVEG
```

```c
    {RST_T, 500, RST_D, RST_E0, RST_JN, AGC_TASK_TIGAVEG},            /* 4.37 */
```

"Task TIGAVEG, 500 cs after TBASE4." (The minus sign on the 2CADR is
how the rope says "task"; the C says `RST_T`.) If the computer restarts
anywhere in the five seconds between TIGBLNK and TIGAVEG, GOPROG3 finds
PHASE4 = 37, looks up 4.37 and schedules TIGAVEG at what is left of the
500 cs (`exec/restarts.c:findtime`: "at least 1 cs"). The countdown continues
as if nothing had happened.

`agc_restart_group` (RESTARTS, C p.1414 / L p.1303) reads the phase and
branches on its type:

```c
    if (tempphs == 0U)
    {
        /* inactive group */
    }
    else if ((tempphs & RS_VARBITS) == 0U)
    {
        st = restart_table(group, (int) tempphs, false);
    }
    else
    {
        ...
        if ((tempphs & RS_TYPEB) == 0U)
        {
            st = restart_variable(group, tempphs, &s);
        }
        else
        {
            st               = start_job(s.prdt, s.job);                              /* ITSLIKEB          */
            agc_status_t st2 = restart_table(group, (int) (tempphs & RS_LOW7), true); /* fixed part outcome */
```

Phase 1 of any group is special: it restarts INITDSP, which re-issues
the last display, so a program waiting on the crew comes back with its
question flashing.

## 7.5 Two things C cannot do, and what it does instead

**Return addresses.** A variable phase often says "restart at the
return address of PHASCHNG": the instruction after the call. C cannot
name a point inside a function. The nearest restart point it can name
is the job segment that made the phase change, which it re-enters from
its top (`exec/phase_table.c:store_return_point`):

```c
 * DEVIATION: the AGC resumes after the TC PHASCHNG; C reruns the segment
 * from its start.
```

Because jobs are already split at every wait (chapter 4), segments are
short and re-running one from the top repeats a few lines of setup.

**Jumping into the restart.** BAILOUT and POODOO jump straight into
ENEMA from wherever they were called, abandoning the caller's stack. A
C function that called ENEMA from inside a job would make the call graph
recursive -- ENEMA restarts jobs -- which Power of Ten forbids. So the C
**records** the request and serves it at the next *service point*: the
end of the job, the end of T3RUPT, the exit of any interrupt
(`docs/overview/flowcharts/software-restart.md`). `agc_bailout`
(`exec/alarm.c`, BAILOUT C p.1495 / L p.1383):

```c
agc_status_t agc_bailout(agc_alarm_code_t code)
{
    agc_inhint();
    exec_vac5stor();
    store_alarm(code);
    exec_request_enema(); /* OCT40400+1: WHIMPER */
    agc_relint();

    return (AGC_STATUS_BAILOUT);
}
```

and the function that called it returns `AGC_STATUS_BAILOUT` up to the
executive, which, after the job returns, sees the request
(`agc_exec_run_one`, chapter 4):

```c
    if (agc_exec_restart_pending())
    {
        /* BAILOUT / POODOO / ABORT in the job: WHIMPER -> ENEMA. The
         * job's core set is wiped by STARTSB2, so no ENDOFJOB.        */
        agc_status_t rs = agc_exec_service_restart(); /* ENEMA outcome */
```

This is why almost every C function in the tree returns `agc_status_t`
and every caller checks it: a status is how "the AGC never comes back
from here" travels up a C call stack.

## 7.6 Fresh start

A **fresh start** throws the running programs away. It happens at
power-on with consistent erasable, on V36 (`agc_fresh_start_slap1`,
SLAP1), when the phase tables fail their complement check (alarm 1107,
PTBAD), or when the crew holds MARK REJECT and ERROR RESET through a
restart. DOFSTART (`exec/fresh_start.c:dofstart`, `dofstart_common`, C
pp.183-185 / L pp.211-213) turns the engines and jets off, kills every
restart group (MR.KLEAN), sets MODREG = -0 (PROG blank) and
reinitialises the flagwords from SWINIT, keeping the few facts that
outlive any program (chapter 5, 5.8). The computer is then idle.

## 7.7 V37: changing program through a restart

The crew changes program with V37 *nn* ENTER. The AGC does it -- this
is the elegant part -- by a software restart: the new program's first
job becomes the restart point of group 4, so that a power transient
during the change still ends in the new program (`exec/change_program/
sequence.c`, NOTES; `docs/overview/flowcharts/program-change.md`). In
outline, `agc_change_program` (V37, C pp.197-202 / L pp.227-232):

1. **Refusals.** ALARM 1520 while the IMU is being initialised or
   NODOFLAG ("V37 not allowed now") is set, e.g. during entry
   (`cp_refuse`); OPR ERR if *nn* is not in the vehicle's mode table
   FCADRMM1 (chapter 1, `cp_select`). On the LM, V37 N70 or N71 starts
   the abort programs at once.
2. **Wind down.** If SERVICER (average-G) is running, it is asked to
   stop and its last pass comes back to the V37 job (AVEGFLAG); on the
   CSM a running SPS engine or TVC autopilot is shut down first
   (R00TOP00). The job waits for the orbital integration to finish
   (INTSTALL) and cleans up flags and output bits.
3. **P00 or a new program.** For P00 (POOH) it sets MODREG = 0 and
   restarts with no group 4. For anything else (NOUVEAU) it points
   group 4 at the V37XEQ job and requests the restart
   (`exec/change_program/sequence.c:cp_gogoprog`):

   ```c
       agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_CHANGE_PROGRAM_START, AGC_TASK_NONE, (int16_t) EXEC_PRIO30, false, 0U};

       agc_phase_change_var(CP_GROUP4_JOB, &v); /* TEMPFLSH = V37QCAD */
       ...
       agc_exec_enema(); /* TC POSTJUMP CADR GOPROG2 */
   ```

4. **Start.** The restart clears the executive and the waitlist, runs
   group 4, and V37XEQ (`agc_job_change_program_start`, C p.202 / L
   p.232) FINDVACs the program's first job from the mode table at its
   priority, sets MODREG with NEWMODEA (PROG shows the new number), and
   releases the DSKY.

The C differs in one detail, documented in the file: the AGC routes
group 4 through phase 4.1 (INITDSP, which jumps on to V37XEQ); the C
INITDSP "re-issues the last display and cannot be pointed at TEMPFLSH,
so group 4 gets type C job phases instead" (sequence.c NOTES).

A program **ends** the same way in reverse: `agc_exec_gotopooh`
(GOTOPOOH) protects the request with group 4 and continues as GOP00FIX
(`exec/change_program/idle.c:agc_job_go_idle_fix`, C p.194), which
flashes V37 N99 -- `CAF V37N99 / TC BANKCALL / CADR GOFLASH` -- until the
crew selects a program.

Next: [8. Navigation](08-navigation.md).
