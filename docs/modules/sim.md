# sim -- host harness: clock/interrupt driver + yaAGC peripheral protocol

History of this lane: [docs/history/lanes/sim.md](../history/lanes/sim.md)

## Scope
docs/history/assignments/sim.md, REVISED SCOPE (2026-10-10): build/agc_sim drives the
translated flight software with our own clock and interrupt driver and talks
to Virtual AGC peripherals (yaDSKY2, yaTelemetry, ...) with yaAGC's socket
protocol, so they connect to it as to yaAGC. No DSKY console, no device
physics. Scripted scenarios run through sim/sim_client.py, which speaks
the same protocol.

## Design

| Piece | AGC / yaAGC reference | C (src/sim/) |
|---|---|---|
| clock: 32 scaler counts per cs (yaAGC phases): TIME1/2/3 at LOSCALAR&037 = 020 (TIME3 overflow -> T3RUPT), TIME5 at 000 (T5RUPT) + radar gate (ch13 b4; 9 gates -> RADARUPT at 036), TIME4 at 010 (T4RUPT), TIME6 DINC on odd counts when ch13 b15 (ZOUT -> T6RUPT, clears b15), DOWNRUPT every 20 ms, HANDRUPT traps (ch13 b12-14) every count; scaler on ch 3/4 | yaAGC agc_engine.c (counter timing); INTERRUPT_LEAD_INS (CM p.131, LM p.153) | clock.c |
| interrupt priority T6 T5 T3 T4 KEY1 KEY2 UP DOWN RADAR HAND, taken after each scaler count, held while INHINT | INTERRUPT_LEAD_INS vector order | clock.c |
| executive: agc_exec_run_one until idle after each count that took an interrupt and at the end of the cs, bounded per call | EXECUTIVE (C p.1208) | clock.c |
| output channels broadcast on change (sampled after every interrupt and job); 010 after every T4RUPT, 034/035 after every DOWNRUPT | yaAGC ChannelOutput | channels.c |
| fictitious 0163 DSKY lamps (KEY REL, OPR ERR, V/N flash 1.28 s, RESTART, STBY, TEMP) | yaAGC UpdateDSKY | channels.c |
| output counters: CDUxCMD -> 0174-0176 (<= 24 per axis per 75 ms), GYROCMD -> 0177 (bursts <= 1024 per 0.25 s), CDUT/SCMD -> 0171/0172; DINC by packet on 053-055 answers POUT/MOUT/ZOUT | yaAGC BurstOutput, gyro, CounterDINC | channels.c, counters.c |
| inputs: u-bit masks, counters (0200|addr: PINC MINC PCDU MCDU DINC SHINC SHANC), 015 -> KEYRUPT1, 016 -> KEYRUPT2, 0173 -> INLINK + UPRUPT, 0166-0170 RHC latched on ch13 b8+9 | yaAGC SocketAPI ChannelInput, UnprogrammedIncrement | channels.c, counters.c |
| TCP server ports 19697..19706, one client each, connect update | yaAGC ChannelRoutineGeneric | socket.c |
| lockstep: in 0150 = run N cs, out 0151 = ack (sim-only fictitious channels) | -- | socket.c, main.c |
| --script timed inputs (same semantics as packets), --headless | -- | script.c, main.c |
| yaAGC command line (--core rope -> vehicle, --port, --cfg, --resume = pad file or logged) | VirtualAGC.cpp FormCommands | main.c, pad.c |

## How to run it
`ln -sf $PWD/build/agc_sim ~/VirtualAGC/bin/yaAGC && ~/VirtualAGC/bin/VirtualAGC`
(stock Virtual AGC install; pick Apollo 11 Command Module or Lunar Module,
DSKY + Telemetry Downlink Monitor, Run!). For a P52 / P40 start, put a pad
file (pad.c format) in the launcher's AGC Startup "Custom" core-file box:
it arrives as `--resume=FILE`. A scripted second peripheral (keys,
discretes, CDU / PIPA counts) can join on any free port of the ten:
`sim/sim_client.py --connect localhost:19701 --realtime --scenario F`.

## Program flow and decisions

_Documentation wave (2026-10-12); prose and tables brought to the final tree (HEAD 3b97dc8) by the final pass (2026-10-14). The prose was written by reading the code; the two tables were extracted from the function bodies (flag, alarm, channel and scheduling calls; `agc_erase_*` accessors) and list only what the code does. Files: `src/sim/*.c`._

The harness that makes build/agc_sim a drop-in for yaAGC: the clock and
interrupt driver and the peripheral protocol, nothing else (no device
models). It raises no AGC alarms; its decisions are timing and protocol.

* **Start** (`src/sim/main.c`): yaAGC's command line (`--core` rope picks
  the vehicle via `rope.c:sim_rope_vehicle`, `--port`, `--cfg`, `--resume`
  as a pad file) plus `--headless` (no sockets, `--max-cs`), `--lockstep`
  (the clock advances only when the client asks) and `--fast` (no
  real-time pacing). Bad arguments, an unknown rope or no port to listen
  on exit 2; power on = rope, pad (`pad.c:sim_pad_load`: keywords in
  engineering units through the public setters, BAD_ARG with the line
  number refuses to start), channels, then GOJAM (`agc_rupt_go`). A run
  that counted handler errors exits 1.
* **Clock** (`clock.c:sim_clock_tick`, one centisecond = 32 scaler counts
  at 3200 Hz, `sim_clock_count`): at scaler phase 020 TIME1/TIME2 and
  TIME3 (overflow -> T3RUPT); at 000 TIME5 (-> T5RUPT) and the radar gate;
  TIME4 -> T4RUPT, TIME6 at 1600/s while enabled (-> T6RUPT); DOWNRUPT
  every 20 ms (DEVIATION: yaAGC times it from the program's 034/035
  writes); the SBY/PRO standby circuit. Taken interrupts run their
  handler, then the executive runs jobs (bounded per call,
  SIM_JOBS_PER_CALL). A handler or job returning BAD_ARG is counted and
  reported, the run goes on.
* **Outputs** (`channels.c`): one packet per program WRITE / WOR / WAND,
  in order, from core's write log (`send_writes` over
  `agc_chan_next_write`; closed-loop-fixes S-L1), as yaAGC's
  ChannelOutput; the fictitious channels yaAGC computes (DSKY lamps 0163,
  CDU drive 0174-0176, gyro 0177, optics/TVC 0171-0172); a new client
  gets every non-zero output first. LM RHC counters are loaded once per
  centisecond while channel 13 bits 8 and 9 are set (DEVIATION).
* **Inputs** (`socket.c`, `counters.c`): ports base..base+9, one client
  each (CSM 19697, LM 19797 by default); input packets update input
  channels with masks; channel 0200 | addr is an unprogrammed counter
  increment (`sim_counter_increment`: PINC, MINC, PCDU, MCDU, DINC; an
  unknown type is BAD_ARG, an unmapped counter NOT_FOUND). CDUX-CDUZ
  counts go through a per-CDU FIFO served at 400 counts/s (slow) or 6400
  (fast) (`sim_counters_tick`; closed-loop-fixes S-L2), so CDUs lag a
  coarse-align drive as under yaAGC. A broken client is dropped.
* **Scripts** (`script.c`): `--script` events `<cs> <channel> <value>
  [<repeat>]` applied as packets when due; a malformed line ends the
  script.

### Decision points by function

None of the lane's 102 functions tests a flag, raises an alarm or starts a job or task; its decisions are the numeric branches described above.

### Shared erasables read and written

Accessors of `include/agc/core/state.h` and the lanes' public headers called by this lane, with the number of its functions that read / write each. Lane-private erasables are `static` in the lane's files or in its state struct; their scales are on the declarations.

| erasable (accessor) | meaning and scale (accessor prolog) | read by | written by |
|---|---|---|---|
| `cdus` | CSM optics shaft CDU raw word (CDUS / OPTX); LM: rendezvous radar shaft | 1 | 1 |
| `cdut` | CSM optics trunnion CDU raw word (CDUT / OPTY); LM: rendezvous radar trunnion | 1 | 1 |
| `cdutcmd` | CDUTCMD (053) / CDUSCMD (054): optics (CSM) or RR (LM) CDU error counter drive, SP pulse counts; which 0 = CDUTCMD, 1 = CDUSCMD. In TVC mode the same  | 2 | 2 |
| `cdux` | CDU X (outer gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (register CDUX, 032) | 1 | 1 |
| `cduy` | CDU Y (inner gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUY) | 1 | 1 |
| `cduz` | CDU Z (middle gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUZ) | 1 | 1 |
| `failreg` | three alarm code registers FAILREG +0..+2 (octal alarm codes) Index 0..2. | 1 | 0 |
| `inlink` | INLINK (045): uplink word for UPRUPT, raw 15-bit word. | 1 | 2 |
| `modreg` | MODREG major mode (program) | 1 | 0 |
| `p_rhcctr` | P-, Q-, R-RHCCTR (LM 042-044): rotational hand controller counts, SP. | 1 | 2 |
| `pipax` | PIPA X pulse count since last read (register PIPAX, 037), unscaled pulses: 1 pulse = 5.85 cm/s (CSM SERVICER207), 1 cm/s (LM) | 1 | 1 |
| `pipay` | PIPA Y pulse count (register PIPAY) | 1 | 1 |
| `pipaz` | PIPA Z pulse count (register PIPAZ) | 1 | 1 |
| `q_rhcctr` | P-, Q-, R-RHCCTR (LM 042-044): rotational hand controller counts, SP. | 1 | 2 |
| `r_rhcctr` | P-, Q-, R-RHCCTR (LM 042-044): rotational hand controller counts, SP. | 1 | 2 |
| `refsmmat` | reference-to-stable-member matrix, row-major, DP elements, scale 2^1 (REFSMMAT, I(18D)PRM; 'REFSMMAT SCALE OF B1') | 0 | 1 |
| `rnrad` | RNRAD (046): the radar reading (RR, LR or VHF range), raw 15-bit word. | 1 | 1 |
| `thrust` | THRUST (LM 055): descent engine throttle command pulses, SP. | 1 | 1 |

### AGC labels -> C functions

The "AGC" column above names the label each listed function translates; the complete label -> function -> file:line index for the lane (and the tree) is [docs/overview/xref.md](../overview/xref.md), generated by `tools/xref.py`.
