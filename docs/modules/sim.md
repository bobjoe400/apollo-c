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
| yaAGC command line (--core rope -> vehicle, --port, --cfg, --resume = pad file or logged) | VirtualAGC.cpp FormCommands | main.c, pad.c |

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
- DONE scenario e: sim/scenarios/external_delta_v_burn_csm.txt (P30 + P40). P30: N33/N81 loads, N42 HAPO 109.6 / HPER 101.0 nmi / DELVSIN 10.0 ft/s (the old DELVSIN 0 is gone), V16N45 live (B4 fixed), V37. P40: R60 V50N18 (ENTER: attitude accepted), V50N25 00204 (ENTER: no gimbal test), V06N40 countdown, TIGBLNK blank at TIG-35 and back at TIG-30. From TIG-5: flight bug B6 (V99 answered as ENTER at once: SPS bypassed); the V99 flash, ENGINE ON and N85-near-zero lines are xexpect B6; the PIPA feed (44 x / 28 y PINC after ENGINE ON, no physics) is in place for when B6 is fixed. 35 checks, 0 failed, 5 xfail.
- DONE scenario f (LM P63) decision: SKIPPED. A scripted counter feed cannot make it meaningful: P63 is a closed loop (throttle commands -> DPS thrust -> PIPA counts; trajectory -> landing-radar returns via RADARUPT); an open-loop feed diverges from guidance within seconds, so "throttle plausible / no 1201-1202" would test the feed, not the AGC. Needs a vehicle model, which is out of scope (no device models): NASSP / LM_Simulator (PLAN item 10 stretch).
- DONE step 4 (yaAGC drop-in): the launcher (VirtualAGC.cpp FormCommands/FormScript) writes Resources/simulate and runs `../bin/yaAGC --core="source/<rope>/<rope>.bin" --port=<19697|19797> --cfg=<CM|LM>.ini [--resume=CM.core] [--debug --directory=...]` from the Resources directory, then yaDSKY2 0.2 s later, then SimStop (kills all when one exits). agc_sim accepts that line (see main.c), loads the rope from --core, logs --resume. Initial state on connect = yaAGC UpdateAgcPeripheralConnect (relay rows + every non-zero channel 011-0177: 030 = 037777, 031-033 = 077777, 0163 RESTART). RESTART lamp: power-on GOJAM, reset by RSET / ch11 bit 10; STBY lamp + EL-off in standby. Verified: from a scratch install tree (sim/vagc_tree: bin/yaAGC -> build/agc_sim) the exact command line + sim_client --connect --realtime fresh_start_csm passes.
- DONE step 5 prep (2026-10-12): `--resume=F` loads F as a pad file when it is one (pad.c sim_pad_is_pad), else logs "fresh start": the launcher's "custom core file" start-up option carries the P52/P40 pads (REFSMMAT, state vector) into the GUI run. B5 now XPASS (fixed by a review lane): show_banksum_* flipped to expect.
- DONE GUI CSM DSKY sequence (2026-10-12): stock VirtualAGC launcher (scratch build sim/build/vagc with the stock -DYADSKY2 -DYADEDA2, install tree sim/vagc_tree, bin/yaAGC -> build/agc_sim), Apollo 11 Command Module, DSKY + Telemetry Downlink Monitor, Run!: the launcher's simulate script started agc_sim, yaDSKY2 and yaTelemetry (port 19700). Clicked on yaDSKY2: power-on RESTART lamp; RSET + V37E00E -> PROG 00; V05N09E + V36E -> all blank; V35 -> all 88 / lamps; V16N36 -> clock 47.57 s; yaTelemetry Coast and Align list live. docs/modules/sim/gui_csm_dsky_sequence.png, gui_csm_telemetry.png, gui_launcher_csm.png, gui_csm_simstatus.png. Windows closed.
- DONE GUI LM DSKY sequence (2026-10-12): same launcher, Apollo 11 Lunar Module (agc_sim --core Luminary099 --port 19797, yaDSKY2 LM.ini, yaTelemetry 19800): power-on RESTART, P00, V36 blank, V35 (ALT/VEL lamps too), V16N36 clock 40.00 s; yaTelemetry LM Coast and Align. gui_lm_dsky_sequence.png, gui_lm_telemetry.png. Windows closed.
- DONE GUI CSM P52 (2026-10-12): launcher with AGC Startup "Custom" core file = the scenario's pad (--resume=imu_realign_csm.pad: yaTelemetry shows RN 6563000, VN+2 77.933, REFSMMAT diagonal 1); keys, IMU-operate discrete, sextant CDU drives and the two marks from tools/sim_client.py --connect localhost:19701 --realtime (a second peripheral, like the LEB DSKY) running sim/scenarios/imu_realign_csm.txt with three screenshot holds: 40 checks, 0 failed, in real time. yaDSKY2: V22N06 option 3, V01N70 star 10, V06N05 +00000, V06N93 -0.008/-0.005/+0.005 deg, V50N25 00014. gui_csm_p52.png, gui_csm_p52_telemetry.png. Windows closed.
- DONE GUI CSM P30 + P40 (2026-10-12): same set-up with the scenario-e pad as the Custom core file; sim_client --connect runs sim/scenarios/external_delta_v_burn_csm.txt (+ four 4 s screenshot holds): 35 checks, 0 failed, 5 xfail (B6), the same as headless. yaDSKY2: V06N42 HAPO 109.6 / HPER 101.0 / DELVSIN 10.0, V16N45 (TFI counting, MGA), V50N18 burn attitude, V06N40 countdown, TIGBLNK at TIG-35, and at TIG-5 B6 in the GUI too: no V99 flash, V16N85 with the full 8.4 / 5.4 / 0.7 ft/s. gui_csm_p30_p40.png. Windows closed.
- DONE step 5: every GUI sequence (fresh start, V35, V16N36 both vehicles; CSM P52; CSM P30/P40) also runs headless under `make scenarios` (fresh_start_*, lights_test_*, clock_*, imu_realign_csm, external_delta_v_burn_csm): 13/13 pass, 5 xfail (B6). LM P63 not run (f decision: skipped). The paced GUI copies of the scenarios (holds only) and the launcher/click helpers live in the scratchpad sim/gui/, not in the repo.
- DONE re-run 2026-10-11 on the status-handling-2 tree (a02850c/826f7e5): `make` and `make sim` up to date, `make scenarios` green: all 7 (fresh_start, lights_test, clock x csm/lm; p52_realign_csm) pass, 0 failed, 0 xfail, 0 xpass, 0 errors/restarts. agc_sim_debug on fresh_start csm/lm and LM headless 2000 cs: no assertion reports.
- STOPPED here (coordinator, 2026-10-11: sim lane moved to the end of the project, after all reviews). No file edited in this resume; tree as found. Next on resume: (1) B4 fix in src/exec/waitlist.c wait2() (plan below, not started), (2) scenario e file, (3) f decision, (4) yaAGC-launcher drop-in (END GOAL), (5) GUI checks.
- DONE acceptance pass step 1 (2026-10-12, tree at c9fdc11 + follow-up edits): `make`, `make sim`, `make scenarios` green: 8/8 (incl. sys-misc's p07_systems_test_lm), 0 failed, 0 xfail.
- DONE review item "rope image as fixed memory": src/sim/rope.c loads the yaYUL rope (banks 2,3,0,1,4..43, word >> 1) and registers it with agc_selfcheck_set_rope(); --core/--exec/--rope/positional path, else tools/oracle/ropes/<rope>.bin from cwd or <exe>/../; bank sums 0-2 checked = +-bank (CSM 77777/00001/00002, buggers 50114/50017/51322). V91 display blocked by flight bug B5 (scenario show_banksum_*, xexpect B5).
- DONE review item "scaler channels 3/4": clock.c counts the 3200 Hz scaler (32/cs) into LOSCALAR/HISCALAR in yaAGC's phase (TIME1 at low5 = 020, TIME5 00, TIME4 10; jobs see the TIME4 phase). LM P07 PIPA rate now real: R1 +00928 (was the DDV overflow value).
- DONE SBY/PRO circuit (yaAGC): PRO held over the 1.28 s sample + 0.64 s decision with ch13 b11 -> standby (GOJAM clears 5,6,010-014,034,035; 0163 STBY + EL-off; only the scaler runs); again -> GOPROG. Scenarios standby_{csm,lm}.txt: P06 V50N25 00062, STBY/ELOFF, POSTAND restores the clock from the scaler, V37 flash. Client: `pro N` hold, lamp ELOFF.
- DONE renamed scenarios to NAMING.md stems: p52_realign_csm -> imu_realign_csm, p07_systems_test_lm -> systems_test_lm.
- DONE yaAGC command line in main.c (step 4, part 1): one or two dashes, --core/--exec/positional rope -> vehicle (Comanche055 CSM, Luminary099 LM, others refused), --cfg CMSIM/LMSIM, --port, --cd, --quiet; --resume/--debug/--symtab/--directory/... logged and ignored.
- State (older): src/sim compiles with -Werror (all files, incl. pad.c with its prototype in internal.h). `make` currently fails in flight code edited by review agents (src/guidance/burn_steering.c:295 earlier, src/csm/prelaunch_align.c now), not in src/sim. No file mid-edit.

## How to run it
`ln -sf $PWD/build/agc_sim ~/VirtualAGC/bin/yaAGC && ~/VirtualAGC/bin/VirtualAGC`
(stock Virtual AGC install; pick Apollo 11 Command Module or Lunar Module,
DSKY + Telemetry Downlink Monitor, Run!). For a P52 / P40 start, put a pad
file (pad.c format) in the launcher's AGC Startup "Custom" core-file box:
it arrives as `--resume=FILE`. A scripted second peripheral (keys,
discretes, CDU / PIPA counts) can join on any free port of the ten:
`tools/sim_client.py --connect localhost:19701 --realtime --scenario F`.

## Lessons
- The launcher has no "AGC binary" setting: it always writes
  `../bin/yaAGC ...` into Resources/simulate, so the drop-in is a link at
  bin/yaAGC. Everything else (rope path -> vehicle, --port, --cfg, the
  --resume core file) comes from that one command line.
- Build the launcher the way the top-level virtualagc Makefile does
  (`-DYADSKY2 -DYADEDA2`, from YADSKY_SUFFIX=2); without it the launcher
  runs the retired GTK `yaDSKY`, which a modern install does not have.
  Do not add AUTO_MISSION_ENABLE on a partial install: the dropdown then
  asserts on an invalid index.
- The launcher's mission choice lives in Resources/VirtualAGC.cfg as 50
  anonymous `missionRadioButtons[...]` lines in table order (Apollo 11 CM
  = 16, LM = 21); writing that file is a reliable way to preselect a run.
- XTest clicks on wx/GTK buttons need a pointer motion into the button
  (enter-notify) some 0.2 s before the press; a bare press on a freshly
  mapped window is dropped.
- yaAGC's core file cannot map onto the typed C erasable, but its
  command-line slot can carry our pad file; that keeps the GUI start-up
  path (Custom core file) identical to yaAGC's.
- Real-time scenario runs compress: `until` returns as soon as a display
  appears, so a 2 s screenshot loop misses most of a P52; add holds (run N)
  where the AGC waits for the crew anyway (flashing V06/V50 displays).
- `pkill -f PATTERN` from a shell whose own command line contains the
  pattern kills that shell; stop the peripherals with `pkill -x NAME`.

## Bugs found (flight code; not fixed here -- the collapse lane owns src/)
Scenario lines that check the AGC-correct behaviour use `xexpect <id>`.
- B1 FIXED (review-io-dsky: QUIKDSP calls agc_dsky_dspoutsb; scenarios
  flipped to expect). src/io/service/t4rupt.c t4_quikdsp() called agc_dsky_dspout(), which
  tests DSKYFLAG (src/io/dsky/pinball/state.c agc_dsky_dspout). The AGC's
  QUIKDSP (T4RUPT_PROGRAM C p.137 / L p.160) is `CCS NOUT / TC DSPOUTSB`
  with no FLAGWRD5 test; only the slow path DSPOUT (C p.135) tests it.
  After V36 SWINIT leaves DSKYFLAG off, so the DSPOFF blanks of STARTSB2
  are not sent by the 20 quick passes: VERB 36 / PROG 00 stay lit until the
  next key press. yaAGC blanks them within 0.3 s. Fix: QUIKDSP should call
  the DSPOUTSB part without the DSKYFLAG test.
- B2 NO LONGER REPRODUCES (2026-10-11): after the guard pass, AGC_JOB_JET_ACCEL_RESET
  (agc_job_jet_accel_reset, src/lm/dap/accs.c:565) returns AGC_STATUS_OK at LM power-on (checked
  with gdb on agc_sim_debug); no assertion reports. Original entry:
  LM power-on: job AGC_JOB_JET_ACCEL_RESET (1/ACCS, src/lm/dap/accs.c
  :191,:391, accont.c:74,:99, arith.c:116 assertions) returns BAD_ARG at
  3 cs with the zero erasable of agc_power_on (mass, inertia 0). The AGC
  would compute with overflowed values and carry on; the job just ends
  here. Harmless for the DSKY scenarios; matters for any LM DAP scenario.
- B3 FIXED (guard pass): chan_clear() (src/io/service/t4rupt.c:163) now masks
  `~bits & WORD15`, no assertion. Original entry: src/io/service/t4rupt.c:127 chan_clear(): `AGC_ASSERT(v <= 0xFFFFFFu)`
  with v = ~(uint32_t)bits is always false (debug builds print it every
  IMU-monitor pass). Intended `bits <= WORD15`.
- B4 src/exec/waitlist.c:152-163 wait2(): the DEVIATION "the TIME3
  overflow already happened branch (C(TIME3) < 200 octal) cannot arise"
  is wrong. It arises inside a RUPTAGN chain (agc_waitlist_t3rupt: two
  tasks due in the same centisecond): while task 1 runs, TIME3 = T - T2
  (0 here), not 1.0 - (T2 - T). WAIT2 (WAITLIST.agc C p.1225 / L p.1121)
  then forms TD - T1 + 1 from `200 - C(TIME3)` > 0; the C code treats
  task 2 as 163.84 s away, makes the new task task 1 and pushes task 2
  back. Seen in P30: COMPTGO queues CLOKTASK (1 cs) and DISP45's V16N45
  queues MONREQ (1 cs) in the same cs; CLOKTASK re-queues itself every
  second ahead of MONREQ, so MONREQ never runs and V16N45 stays blank
  (yaAGC: marks +00 00, TFI counting, MGA). Scenario e xexpects it.
- B5 FIXED (2026-10-12, a review lane; V91 R1-R3 show SKEEP1-3, scenarios show_banksum_* flipped to expect). Original entry: src/io/verbs/extended_verbs.c:541-546 banksum_display(): SHOW-BANKSUM
  (V91) flashes V05N01 without the noun address. SDISPLAY/NOKILL
  (EXTENDED_VERBS.agc C p.263 / L p.290) puts ADRS1 = ECADR SKEEP1 in
  MPAC+2 before GOXDSPF, so N01 shows SKEEP1-3 (sum, bank, bugger word);
  the C call passes no address (agc_dsky_nvsub_mctbs exists for this) and
  SKEEP1-3 are not readable through the ECADR image (src/core/ecadr.c), so
  R1-R3 show 00000 for every bank. Scenarios show_banksum_* xexpect B5.
- B6 STILL STANDS (2026-10-12: now clock.c:156,163, unchanged). src/csm/burns/sps/clock.c:156,163 cknvwrd1(): the CLOCKJOB shows the
  engine-on enable / engine-fail flashes as CLOCPLAY of V99N40 / V97N40.
  The AGC (ENGREQST / FAILDSP, P40-P47.agc C p.705) does CAF V06N40,
  CLOCPLAY: the first part is V06N40 and the flashing V99/V97 is pasted
  as the second part from NVWORD1 (-BIT9 / -0; 1STOR2ND, DISPLAY_INTERFACE
  _ROUTINES p.1477). The C passes 99 as the verb to execute, so pinball
  runs extended verb 99 = GOLOADLV -> LOADLV -> RECALTST, which wakes the
  ENDIDLE at once with the ENTER return (gdb: pin_recaltst <- agc_dsky_loadlv
  <- agc_extverb_goloadlv <- fan_shared(verb 99)): V99E, SPS bypassed
  (P40RCS), V16N85 with the full DV at TIG-2.4 s, no V99 flash, no ignition.
  Fix: pass agc_dsky_vn(6, 40) and give the display NVWORD1
  (agc_dsky_set_nvword1(csm.nvword1), as src/lm/burns/ignition/clock.c:198
  does; display_if.c dif_second_vn pastes 97 - NVWORD1). Scenario e
  xexpects B6.
- B4 FIXED (review-exec-sched R10, 2026-10-11): wait2() takes both WAIT2
  CCS paths (C(TIME3) < 200: TD - T1 + 1 = DT + C(TIME3) + 1); test
  test_waitlist_ruptagn_insert. Scenario e can `expect` V16N45. Old note:
- B4 (was) STILL STANDS (2026-10-11): src/exec/waitlist.c:195 wait2() still has the
  DEVIATION and computes a = dt - (16384 - time3) + 1 unconditionally. Fix plan
  (not started): when time3 < 128 (octal 200) TIME3 = T - T1 (post-overflow,
  RUPTAGN chain) and a = dt + time3 + 1 (the OCT40001/CS/OCT40201 path of
  WAIT2, C p.1225 / L p.1121); unambiguous because DTMAX 16250 keeps a normal
  TIME3 >= 134. Test in tests/exec/exec.c: two tasks due in the same cs, the
  first queues a 1 cs task from inside the chain; the queued task must run
  1 cs later and the second chained task must not be pushed back 163.84 s.
