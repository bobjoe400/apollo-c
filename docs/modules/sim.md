# sim -- host harness: clock/interrupt driver + yaAGC peripheral protocol

## Scope
docs/agents/sim.md, REVISED SCOPE (2026-10-10): build/agc_sim drives the
translated flight software with our own clock and interrupt driver and talks
to Virtual AGC peripherals (yaDSKY2, yaTelemetry, ...) with yaAGC's socket
protocol, so they connect to it as to yaAGC. No DSKY console, no device
physics. Scripted scenarios run through tools/sim_client.py, which speaks
the same protocol.

## Design

| Piece | AGC / yaAGC reference | C (src/sim/) |
|---|---|---|
| clock: TIME1/2 every cs, TIME3 (T3RUPT), TIME5 at +5 ms (T5RUPT), TIME4 at +7.5 ms (T4RUPT), TIME6 16 DINC/cs when ch13 b15 (T6RUPT, clears b15), DOWNRUPT every 20 ms, radar cycle (ch13 b4, 9 x TIME5 phases -> RADARUPT), HANDRUPT traps (ch13 b12-14) | yaAGC agc_engine.c (counter timing); INTERRUPT_LEAD_INS (CM p.131, LM p.153) | clock.c |
| interrupt priority T6 T5 T3 T4 KEY1 KEY2 UP DOWN RADAR HAND, held while INHINT | INTERRUPT_LEAD_INS vector order | clock.c |
| executive: agc_exec_run_one until idle, bounded per cs | EXECUTIVE (C p.1208) | clock.c |
| output channels broadcast on change (sampled after every interrupt and job); 010 after every T4RUPT, 034/035 after every DOWNRUPT | yaAGC ChannelOutput | channels.c |
| fictitious 0163 DSKY lamps (KEY REL, OPR ERR, V/N flash 1.28 s, RESTART, STBY, TEMP) | yaAGC UpdateDSKY | channels.c |
| output counters: CDUxCMD -> 0174-0176, GYROCMD -> 0177, CDUT/SCMD -> 0171/0172 | yaAGC BurstOutput, gyro | channels.c |
| inputs: u-bit masks, counters (0200|addr: PINC MINC PCDU MCDU DINC SHINC SHANC), 015 -> KEYRUPT1, 016 -> KEYRUPT2, 0173 -> INLINK + UPRUPT, 0166-0170 RHC latched on ch13 b8+9 | yaAGC SocketAPI ChannelInput, UnprogrammedIncrement | channels.c, counters.c |
| TCP server ports 19697..19706, one client each, connect update | yaAGC ChannelRoutineGeneric | socket.c |
| lockstep: in 0150 = run N cs, out 0151 = ack (sim-only fictitious channels) | -- | socket.c, main.c |
| --script timed inputs (same semantics as packets), --headless | -- | script.c, main.c |

## Progress
- DONE design (this note)
- DONE src/sim/internal.h, clock.c (counters, priority interrupts, exec), channels.c (outputs, 0163, drives, inputs), counters.c (PINC..SHANC map), socket.c (server, packets, lockstep), script.c (--script), main.c (modes; power-on = GOJAM -> GOPROG)
- DONE tools/sim_client.py (lockstep or --connect/--realtime; relay decoding as yaDSKY2; scenario language)
- DONE cross-check: same client script against tools/oracle/bin/yaAGC + Comanche055 rope: P00, V06N36, V35 sequences match (relay order differs slightly during V35)
- DONE yaDSKY2 (virtualagc, built against Debian wx 3.2 unpacked in the scratchpad) connected to agc_sim in real time, keys clicked on yaDSKY2 via XTest: P00, V06N36 clock, V35 lights, V16N36 monitor, both vehicles: docs/modules/sim/yadsky2_{csm,lm}.png
- DONE Makefile: sim, sim-debug (debug lib, assertions reported), scenarios
- DONE yaTelemetry (built the same way) on LM port 19800 decodes agc_sim's downlink: Coast and Align list (ID 77777, SYNC 77340): docs/modules/sim/yatelemetry_lm.png; default ports by vehicle (CSM 19697, LM 19797)
- DONE check_rules hard violations: internal.h comment "..." (rule 8 regex), main.c opt_value double pointer -> returns the value (0 hard)
- DONE tools/sim_client.py `xexpect BUG ...` / `xuntil BUG ...` (known flight-code bug: XFAIL does not fail the run, XPASS reported; --strict fails on XPASS)
- DONE sim/scenarios/fresh_start_{csm,lm}.txt (scenario a); both also run against yaAGC (XPASS on B1 there)
- DONE sim/scenarios/lights_test_{csm,lm}.txt (b), clock_{csm,lm}.txt (c; client `clockref`, `CLOCK now`); all pass, also against yaAGC
- DONE src/sim/pad.c (`--pad`: refsmmat, flagword, sv csm|lm), client `pad` lines
- DONE sim/scenarios/p52_realign_csm.txt (d): option 3, PICAPAR picks stars 8 and 35, sextant CDUs driven to the catalogue LOS, N05 = 0.00 deg, N93 within 0.01 deg, R51K ENTER -> V37 flash (GOP00FIX, same as yaAGC)
- NEXT scenario e (P30 + P40), then f (P63, decide), then GUI re-check

## Bugs found (flight code; not fixed here -- the collapse lane owns src/)
Scenario lines that check the AGC-correct behaviour use `xexpect <id>`.
- B1 src/io/service/t4rupt.c t4_quikdsp() calls agc_dsky_dspout(), which
  tests DSKYFLAG (src/io/dsky/pinball/state.c agc_dsky_dspout). The AGC's
  QUIKDSP (T4RUPT_PROGRAM C p.137 / L p.160) is `CCS NOUT / TC DSPOUTSB`
  with no FLAGWRD5 test; only the slow path DSPOUT (C p.135) tests it.
  After V36 SWINIT leaves DSKYFLAG off, so the DSPOFF blanks of STARTSB2
  are not sent by the 20 quick passes: VERB 36 / PROG 00 stay lit until the
  next key press. yaAGC blanks them within 0.3 s. Fix: QUIKDSP should call
  the DSPOUTSB part without the DSKYFLAG test.
- B2 LM power-on: job AGC_JOB_JET_ACCEL_RESET (1/ACCS, src/lm/dap/accs.c
  :191,:391, accont.c:74,:99, arith.c:116 assertions) returns BAD_ARG at
  3 cs with the zero erasable of agc_power_on (mass, inertia 0). The AGC
  would compute with overflowed values and carry on; the job just ends
  here. Harmless for the DSKY scenarios; matters for any LM DAP scenario.
- B3 src/io/service/t4rupt.c:127 chan_clear(): `AGC_ASSERT(v <= 0xFFFFFFu)`
  with v = ~(uint32_t)bits is always false (debug builds print it every
  IMU-monitor pass). Intended `bits <= WORD15`.
