# Software restart (BAILOUT, POODOO, ENEMA) and the restart groups

AGC: ALARM_AND_ABORT.agc (C pp.1493-1496, L pp.1381-1385), FRESH_START_
AND_RESTART.agc (ENEMA, GOPROG2, GOPROG3, NXTRST, C pp.188-189 /
L pp.217-219), RESTARTS_ROUTINE.agc (C pp.1414-1419, L pp.1303-1308),
PHASE_TABLE_MAINTENANCE.agc. C: `src/exec/alarm.c`,
`src/exec/fresh_start.c`, `src/exec/restarts.c`,
`src/exec/phase_table.c`, `src/exec/restart_tables.c`.

In the AGC, BAILOUT and POODOO jump into ENEMA at once. In C that jump
would make the call graph recursive (rule 1), so the request is
recorded and served at the next **service point**: the end of the job
(`executive.c:agc_exec_run_one`), the end of T3RUPT
(`waitlist.c:agc_waitlist_t3rupt`), the exit of any interrupt
(`interrupts.c:rupt_exit`) or the harness loop (`sim/clock.c:clock_exec`,
`rupt_t3`). The function that raised it returns `AGC_STATUS_BAILOUT` /
`AGC_STATUS_POODOO` and every caller passes that up.

## Raising the restart

```mermaid
flowchart TD
    A1["ALARM code<br/>alarm.c:agc_alarm"] --> S["LARMENT: ALMCADR = running job;<br/>FAILREG +0/+1/+2 (MULTFAIL);<br/>first alarm lights PROG<br/>alarm.c:store_alarm"]
    S --> A2["return (no restart)"]
    B1["BAILOUT code<br/>alarm.c:agc_bailout<br/>(1201, 1202, 1203, ...)"] --> B2["VAC5STOR (CSM); store_alarm"]
    B2 --> W["WHIMPER: request ENEMA<br/>alarm.c:exec_request_enema"]
    W --> B3["return BAILOUT"]
    P1["POODOO code<br/>alarm.c:agc_poodoo<br/>(1204, 1103 CCSHOLE, 1302, ...)"] --> P2["VAC5STOR (CSM); store_alarm"]
    P2 --> P3{"vehicle"}
    P3 -- "CSM" --> P4{"SERVICER running?<br/>(AVEGFLAG)"}
    P4 -- "yes" --> P5["WHIMPER -1: ENEMA; return BAILOUT<br/>(the program keeps running)"]
    P4 -- "no" --> P6["STATEFLG, REINTFLG, NODOFLAG down;<br/>MR.KLEAN (all groups killed); WHIMPER"]
    P3 -- "LM" --> P7["-PHASE4 = 4.35 (GOPOODOO restart job)<br/>alarm.c:gopoodoo"]
    P7 --> P8{"SERVICER running?"}
    P8 -- "yes" --> P9["STRTIDLE: SERVIDLE puts SERVICER in its<br/>ground state, WHIMPER -> 4.33 GOP00FIX<br/>change_program/idle.c:exec_up_servidle"]
    P8 -- "no" --> P10["flags down; V37KLEAN + MR.KLEAN; WHIMPER"]
    AB["ABORT<br/>alarm.c:agc_abort_restart"] --> W
    V37["V37 GOPROG2<br/>change_program/sequence.c:cp_gogoprog"] --> EN["alarm.c:agc_exec_enema"]
    EN --> W
```

## Serving the restart

```mermaid
flowchart TD
    SP["service point:<br/>agc_exec_restart_pending()"] --> SV["alarm.c:agc_exec_service_restart"]
    SV --> E0["ENEMA / GOPROG2<br/>fresh_start.c:fresh_enema (INHINT)"]
    E0 --> E1{"vehicle"}
    E1 -- "LM" --> E2["STARTSB1 (timers, DAP restart, STARTSB2)<br/>then LIGHTSET"]
    E1 -- "CSM" --> E3["LIGHTSET, then STARTSB2"]
    E2 --> E4{"LIGHTSET forced a fresh start?"}
    E3 --> E4
    E4 -- "yes" --> X["done (fresh-start.md)"]
    E4 -- "no" --> E5["FLAGWRD10 restart bits cleared;<br/>CSM with TVC DAP on: TVCEXEC in 0.5 s"]
    E5 --> G0["GOPROG3<br/>fresh_start.c:goprog3"]
    G0 --> G1["PCLOOP: every -PHASEn = complement of PHASEn?<br/>fresh_start.c:phase_tables_ok"]
    G1 -- "no" --> G2["ALARM 1107 (PTBAD); DOFSTART / DOFSTRT1"]
    G1 -- "yes" --> G3["MMDSPLAY: NOVAC DSPMMJOB (PROG = MODREG)<br/>phase_table.c:fresh_mmdsplay"]
    G3 --> G4["LM: DIDFLAG, RODFLAG, P21FLAG down;<br/>CSM with RCS DAP: STOPRATE"]
    G4 --> N0["NXTRST: groups 6, 5, ..., 1<br/>restarts.c:agc_phase_restart_all"]
    N0 --> N1{"PHASEn = 0 ?"}
    N1 -- "yes" --> N2["inactive: next group"]
    N1 -- "no" --> R0["RESTARTS<br/>restarts.c:agc_restart_group"]
    R0 --> R1{"phase type"}
    R1 -- "fixed, phase 1" --> R2["FINDVAC PRIO14 INITDSP<br/>(redisplay the last display)"]
    R1 -- "fixed, odd >= 3" --> R3["one table entry<br/>restart_tables.c:rst_table_lookup"]
    R1 -- "fixed, even" --> R4["both table entries"]
    R1 -- "type C (variable)" --> R5["PHSNAMEn by CJW: LONGCALL / job / task<br/>restarts.c:restart_variable"]
    R1 -- "type B" --> R6["the stored job, then the first table entry<br/>of phase & 177"]
    R3 --> T0["entry: job (FINDVAC if prio > 0, NOVAC if < 0),<br/>task due dt after TBASEn (at least 1 cs),<br/>or LONGCALL from LONGBASE<br/>restarts.c:restart_entry, start_job, findtime, start_long"]
    R4 --> T0
    R6 --> T0
    N2 --> N3{"more groups?"}
    T0 --> N3
    R2 --> N3
    R5 --> N3
    N3 -- "yes" --> N1
    N3 -- "no" --> Z{"any group was active?"}
    Z -- "yes" --> Z1["ENDRSTRT: the restarted jobs and tasks run"]
    Z -- "no, MODREG >= 0" --> Z2["GOTOPOOH -> P00 (V37 flash): GOP00FIX takes over core set 0,<br/>where SPECTEST put MMDSPLAY's DSPMMJOB, so PROG stays blank<br/>fresh_start.c:agc_exec_gotopooh, executive.c:exec_core0_continue_as"]
    Z -- "no, MODREG = -0" --> Z3["idle"]
```

## Keeping the phase tables

* `agc_phase_change(code)` (PHASCHNG, phase_table.c) decodes the code
  word (`decode`: group, phase, type A/B/C, TBASE, priority) and stores
  -PHASEn and PHASEn together with interrupts inhibited
  (`phaschng_common`); a type B/C phase also stores the job or task to
  restart (`store_var`, `agc_phase_change_var`).
* `agc_phase_newphase` (NEWPHASE, CSM) is the quick fixed form,
  `agc_phase_change2` (2PHSCHNG) changes two groups at once.
* The KLEAN routines kill groups: `agc_phase_change_program_klean`
  (V37KLEAN: 3, 5, 6 and LM 1), `agc_phase_idle_klean` (P00KLEAN: 4, CSM
  also 1), `agc_phase_mr_klean` (MR.KLEAN: 2, then all).
* Deviation (phase_table.c `store_return_point`): a type A/B phase whose
  restart point is "the return address of PHASCHNG" restarts the job
  segment that made the phase change, the nearest point C can name.
