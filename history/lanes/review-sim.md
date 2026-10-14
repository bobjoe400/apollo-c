# review-sim -- lane history

Process diary of the review-sim lane, moved verbatim out of [docs/modules/review-sim.md](../../docs/modules/review-sim.md), which keeps the reader-facing explanation. Note dates: 2026-10-12 to 2026-10-13 (first and last commit touching the note).

## Progress
- STARTED 2026-10-12. Baseline: make OK; scenarios 12/12 pass (the burn
  scenario fails 3 checks in the fix-b6 work in progress, not ours);
  check_rules src/sim 0 hard; check_style src/sim 524 findings.
- DONE (1) fidelity (2026-10-12): clock.c rewritten around yaAGC's
  scaler (32 counts per cs; counters, traps, radar and interrupts per
  count; executive after each count that took an interrupt and at the end
  of the cs); channels.c (connect update, RSET on 016, ch33 latches, RHC
  echo, CDU / gyro drive pacing); counters.c (word-level Counter*, SHINC
  14 bits, DINC POUT/MOUT/ZOUT, output counters 053-055). New unit test
  tests/sim/harness.c (Makefile rule links the sim objects): 52 checks.
  Scenarios 12/12 unchanged.
- DONE (2) scope check: nothing in src/sim models device behaviour. The
  drives emit pulse counts only (yaAGC's own outputs), pad.c is a core
  image substitute, rope.c is data for ROPECHK; the only feeds (CDU /
  PIPA counts) are in the scenarios via sim_client `cdu` / `counter`.
- DONE (3) rules: no (void) casts of our functions (close() now checked
  and reported); never-fail guards removed (clock init, scaler stores,
  pad re-reads); the one unbounded loop is run() in main.c, documented in
  the prolog and at the loop; check_rules 0 hard / 0 warnings, every
  function <= 60 lines. Rule 5 by spirit: two real-boundary assertions
  (a job leaving INHINT in force; a peripheral's counter address / type),
  each counted and reported; peripheral and file input otherwise is
  parsed with explicit checks, not assertions.
- DONE (4) style: check_style src/sim + tests/sim 524 -> 0 (prologs per
  template, decl/define comments, main first, no side effects in tests,
  `long int`, upper-case L); tools/format.sh on every lane file;
  format-check clean.
- DONE (5) sim/sim_client.py: pycodestyle and pyflakes clean (scratch
  venv); dead downlink capture removed; `self.l` -> `self.link`; usage
  and options documented in the docstring next to the scenario language.
- DONE gates 2026-10-12: make; make test (all unit/oracle tests, 13/13
  scenarios incl. the burn scenario, 0 xfail); make check (0 hard);
  fresh_start_{csm,lm} gained `expect LAMP RESTART on` (power-on lamp).

## Findings
| id | where | yaAGC reference | finding | resolution |
|---|---|---|---|---|
| S1 | clock.c | agc_engine counter-timers | all counters of a cs at once, then every pending interrupt in priority order: T5 ran before T3 in a shared cs; T6 counted 16 DINCs in a burst and stopped at the ZOUT, so a T6RUPT that reloads TIME6 lost the rest of the cs (chained jet timing stretched up to 10 ms) | 32 scaler counts per cs; TIME1/3 at 020, TIME5 at 000, TIME4 at 010, TIME6 on odd counts; interrupts taken after each count (T3 at +0, T5 at +5 ms, T4 at +7.5 ms, T6 at its DINC); test_time6 |
| S2 | clock.c | RadarGateCounter | gate counter reset whenever ch13 bit 4 was clear; RADARUPT at the 9th TIME5 phase | count held while bit 4 is clear, completion at phase 036 (9.4 ms later), as yaAGC; test_radar |
| S3 | clock.c, channels.c | CpuWriteIO(013) | trap bits 12-14 stayed in ch13 until the next tick: readable by the program and broadcast to peripherals | sim_clock_arm_traps before every output sample; test_handrupt |
| S4 | clock.c | CpuWriteIO 034/035, TriggeredAlarm | DOWNRUPT timed from the writes of 034+035 in yaAGC; Night Watchman / Rupt Lock / TC Trap / parity GOJAMs and the warning filter | kept the hardware's free-running 20 ms DOWNRUPT; alarms not modelled; both DEVIATIONs now in the clock.c prolog |
| S5 | counters.c | CounterSHINC/SHANC | shift kept 15 bits | 14 bits (037777) |
| S6 | counters.c | UnprogrammedIncrement | PINC/MINC on CDU counters treated as PCDU/MCDU; DINC emitted no POUT/MOUT/ZOUT packet; output counters (053-055, THRUST) refused | word-level Counter* for every type; DINC sends 0200|addr = 015/016/017; DINC on CDUTCMD/CDUSCMD/THRUST; test_counters |
| S7 | channels.c | UpdateAgcPeripheralConnect | relay row 0 skipped; stored RHC (0166-0170) and uplink (0173) inputs not sent | rows 0-15; InputChannel[] equivalents sent; 0163 on connect kept as a documented DEVIATION |
| S8 | channels.c | WriteIO | RSET on the nav DSKY (016) did not reset RESTART; a peripheral could write ch33 bits 11-15 | both as yaAGC; test_channel33 |
| S9 | channels.c | ChannelInput | RHC echo sent the unmasked value; 0166-0170/0173 ignored the old value under the mask | masked merge, echo the stored value |
| S10 | channels.c | BurstOutput | CDU drive sent up to 192 pulses every 10 ms (60x the hardware / yaAGC rate) | 24 pulses per axis per 75 ms, first burst at once; remainder stays in CDUxCMD (DEVIATION documented); test_cdu_drive |
| S11 | channels.c | gyro (GYRO_TIMING_SIMULATED off) | whole GYROCMD sent at once in 03777 chunks | flush pending train, take GYROCMD, bursts of <= 1024 every 0.25 s |
| S12 | channels.c | optics drive | yaAGC sends/clears 035/036 (the optics CDU angle counters), apparently meaning 053/054 | kept CDUTCMD/CDUSCMD; DEVIATION documented |
| S14 | main.c | agc_engine_init | channel 30 = 037777 was set after GOJAM -> GOPROG | power-on channels before GOJAM, as yaAGC initialises InputChannel[] before the first instruction |
| S15 | main.c | agc_cli.c | --port / --max-cs / --vehicle values not validated (`--port abc` became the default port) | whole-number check, refused with usage; option table instead of assignments in conditions |
| S16 | script.c, socket.c | -- | a refused counter increment or unknown type was silently dropped | reported on stderr |
| S13 | channels.c | agc_engine standby return | drives and RHC latch ran in standby | skipped in standby |

Flight-code bugs found: none (nothing added to docs/modules/sim.md "Bugs
found"; B6 now passes in the burn scenario after fix-b6).
Observation: systems_test_lm reports 1817 "busy" calls (64-job bound hit)
during P07's job-heavy loops; harmless (jobs continue next call).

## Lessons
- Taking interrupts only once per centisecond hid two timing errors:
  T6RUPT chains lost the rest of the centisecond, and T5 ran before T3
  when both fell due. Stepping the 3200 Hz scaler count by count (32 per
  cs, cheap) gives yaAGC's phases for free and kept every scenario green.
- yaAGC is a reference, not scripture: its optics drive reads 035/036 (the
  CDU angle counters) where the program writes 053/054, and its CDU drive
  empties CDUxCMD where the hardware counts it down. Follow the hardware
  where yaAGC is evidently wrong, and write the DEVIATION down.
- The host power-on (agc_power_on) is not repeatable within one process:
  display-interface state survives (as erasable memory would), so a unit
  test powers on once per vehicle.
- A sampled channel image cannot see write events (yaAGC's RHC latch on
  each write of ch13); per-cs sampling while the bits are set is the
  closest equivalent and is documented.

