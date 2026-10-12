# Fresh start and hardware restart (GOJAM)

AGC: FRESH_START_AND_RESTART.agc (C pp.183-194, L pp.211-223): SLAP1,
DOFSTART / DOFSTRT1, STARTSUB / STARTSB1 / STARTSB2, GOPROG, LIGHTSET,
GOPROG3, GOTOPOOH, GOP00FIX. C: `src/exec/fresh_start.c`,
`src/exec/change_program/idle.c`; the power-on driver is
`src/sim/main.c`.

Three ways in: power-on (GOJAM from the harness), a hardware restart
(GOJAM), and V36 (SLAP1, a crew-requested fresh start). A fresh start
throws the running programs away (every restart group killed); a
hardware restart keeps them and re-runs them from the phase tables.

## Power-on and GOJAM

```mermaid
flowchart TD
    P0["sim/main.c:start"] --> P1["agc_power_on(vehicle)<br/>change_program/idle.c<br/>state, lane erasables, executive, waitlist"]
    P1 --> P2["rope, pad / --resume pad<br/>main.c:load_rope, sim/pad.c:sim_pad_load"]
    P2 --> P3["clock and power-on channels<br/>sim/clock.c:sim_clock_init, sim/channels.c:sim_channels_init"]
    P3 --> G0["GOJAM: interrupts.c:agc_rupt_go"]
    G0 --> G1["GOPROG<br/>fresh_start.c:agc_restart_goprog<br/>INCR REDOCTR, INHINT"]
    G1 --> G2{"vehicle"}
    G2 -- "CSM" --> G3["VAC5STOR (core sets, phases, NEWJOB)<br/>alarm.c:exec_vac5stor"]
    G3 --> G4{"channel 33: OSC FAIL absent<br/>and AGC WARNING on?"}
    G4 -- "yes: restart loop" --> F1["STARTSUB + DOFSTART<br/>(fresh start, below)"]
    G4 -- "no" --> L0
    G2 -- "LM" --> G5{"NO ATT lamp on?"}
    G5 -- "yes" --> G6["channel 12 bits 4, 6: coarse align"]
    G5 -- "no" --> L0
    G6 --> L0["LIGHTSET<br/>fresh_start.c:lightset"]
    L0 --> L1{"MARK REJECT and<br/>ERROR LIGHT RESET held?"}
    L1 -- "yes" --> F1
    L1 -- "no" --> S0["STARTSUB: TIME3/4/5, waitlist, executive, DSKY<br/>fresh_start.c:startsub -> startsb1 -> startsb2"]
    S0 --> S1{"vehicle"}
    S1 -- "CSM" --> S2["ELRSKIP..NOCOARSE<br/>fresh_start.c:goprog_csm_tail<br/>T5LOC from FLAGWRD6 DAP bits, optics, lamps,<br/>NO ATT: coarse align + CA+ECE task, engine on if ENGONFLG"]
    S1 -- "LM" --> S3["SETINFL..: INTFLAG down, lamps, IMODES30,<br/>downlist, throttle counter, engine on/off by ENGONFLG"]
    S2 --> R0["GOPROG3<br/>fresh_start.c:goprog3"]
    S3 --> R0
    R0 --> R1["see software-restart.md:<br/>phase table check, MMDSPLAY, restart every active group"]
    R1 --> R2{"no group active?"}
    R2 -- "MODREG = -0" --> R3["stay idle (PROG blank)"]
    R2 -- "MODREG >= 0" --> R4["GOTOPOOH -> GOP00FIX: V37 N99 flash<br/>fresh_start.c:agc_exec_gotopooh,<br/>idle.c:agc_job_go_idle_fix"]
```

At power-on the harness has zeroed every erasable (`agc_state_init`) and
the phase tables start with every group killed (phase_table.c), so
GOPROG3 finds no active group and, with MODREG = +0, goes to P00 through
GOTOPOOH, which is what yaAGC shows with a blank core image.

## Fresh start (V36 SLAP1, DOFSTART)

```mermaid
flowchart TD
    V["V36 ENTER<br/>pinball/verbs.c:pin_vbfandir case 36"] --> A["SLAP1<br/>fresh_start.c:agc_fresh_start_slap1 (INHINT)"]
    A --> B["STARTSUB<br/>fresh_start.c:startsub"]
    B --> B1["exec_up_startsub: DNTMGOTO, LM RADMODES<br/>change_program/idle.c"]
    B1 --> B2["STARTSB1: TIME3 = 37777, TIME4 = 37775, TIME5 = 37774<br/>LM: DAP restart agc_lm_dap_restart"]
    B2 --> B3["STARTSB2: outbits, LST1/LST2 cleared,<br/>core sets freed, DSPTAB blanked, pinball state,<br/>MARKSTAT, IMU/optics/radar CADRs, FLAGWRD4 = 0<br/>fresh_start.c:startsb2"]
    B3 --> C["lamp word keeps NO ATT and GIMBAL LOCK;<br/>FAILREG = 0; REDOCTR = 0"]
    C --> D["DOFSTART<br/>fresh_start.c:dofstart(slap1 = true, engine_off = true)"]
    D --> D1{"vehicle"}
    D1 -- "CSM" --> D2["dofstart_csm: jets off, channels 11-14 = 0"]
    D1 -- "LM" --> D3["engine off (channel 11 bit 14), THRUST = 0;<br/>dofstrt1_lm: jets off, channels 12-14 = 0"]
    D2 --> E["dofstart_common:<br/>coarse align kept if it was on in gimbal lock;<br/>MR.KLEAN (all groups killed); MODREG = -0;<br/>RESTREG = PRIO30; IMODES30/33; T5 idle (CSM);<br/>flagwords from SWINIT keeping REFSMFLG,<br/>CMOONFLG, LMOONFLG, SURFFLAG (+ NODOP01, APSFLAG)"]
    D3 --> E
    E --> F["exec_up_dofstart: other modules' fresh-start resets<br/>change_program/idle.c"]
    F --> G["RELINT; no job, no task: idle, PROG blank<br/>(T4RUPT flushes the blanked DSKY, agc_t4_fresh_start_flush)"]
```

Other ways into DOFSTART: `agc_fresh_dofstart` (machine-initiated),
LIGHTSET (MARK REJECT + ERROR RESET during a restart) and PTBAD (alarm
1107 when a -PHASEn is not the complement of PHASEn, `phase_tables_ok`;
on the LM that path is DOFSTRT1, which leaves the engine alone).
