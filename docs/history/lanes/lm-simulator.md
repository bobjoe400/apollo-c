# lm-simulator -- lane history

Process diary of the lm-simulator lane, moved verbatim out of [docs/modules/lm-simulator.md](../../modules/lm-simulator.md), which keeps the reader-facing explanation. Note dates: 2026-10-12 to 2026-10-13 (first and last commit touching the note).

## Progress
- DONE step 1 (connect): agc_sim LM peripheral server + stock
  lm_simulator.ini; simulator connects on 19801, all six windows up, DSKY
  Lite shows PROG 00 / VERB 37 flashing after power-on.
- DONE harness (2026-10-12): sim/lm_simulator/harness.tcl sources the
  simulator unmodified (wish = its windows; tclsh = inert Tk stand-ins that
  keep each widget's -command/-variable so `invoke` runs its own button
  script) and adds a control port; sim/lm_simulator/lm_sim.py starts an AGC (agc_sim or
  yaAGC) on a free LM port range, the simulator on base+4, a monitor
  peripheral on base+1 (DSKY decode as yaDSKY2, jet log from ch 5/6), plays
  a sequence of crew actions and compares checkpoints.
- DONE step 2/3 exploration, phase by phase, ours vs yaAGC (details in
  Findings): power-on, V36, V37, V35, ISS turn-on (NO ATT 89.5 s in both),
  V41N20 coarse align (closed loop, both 10.02/20.00/30.02), V42 fine align
  (yaAGC torques, ours does not: bug L2), V40N20 zero, V48 R03 (same
  displays), RCS attitude hold / rate command (ours open-loop in the
  simulator: finding S-L1; alarm 1103: bug L3), V37 refused (bug L1).
- DONE step 3 (side by side): sim/lm_simulator/tutorial.txt (32 checkpoints)
  flown on both AGCs headless, with --repeat-jets (ours only; the relay
  run's yaAGC half was refused a port, the yaAGC reference is the plain
  run's) and with --gui on the real display (no Xvfb on this host; every
  window closed by the harness at the end, `pgrep wish` empty).
  Screenshots, agc_sim (top) vs yaAGC (bottom), DSKY Lite | FDAI/IMU | AGC
  Outputs: docs/modules/lm-simulator/{lights_test,coarse_n20,fine_n20,
  rhc_pitch,man_n20}.png. Results in Findings.
- DONE step 4: headless equivalent = the same harness under tclsh (Tk
  stand-ins); `make lm-sim LMSIM_DIR=...` runs the comparison. Verified end
  to end 2026-10-12: 32 checkpoints, exit 1 with the mismatches explained
  by L2 (fine_n20), L3 + S-L1 (every DAP checkpoint), S-L2 (00211); the
  tutorial now runs V35 before V36 so V37E00E is accepted (L1 is reproduced
  by the lockstep recipe in Bugs found). No process or window left behind.

- 2026-10-12 closed-loop-fixes: L1-L3, S-L1, S-L2/S-L3 and D1 fixed;
  `make lm-sim` exits 0 (34 checkpoints). Tolerances after the first
  detent now come from yaAGC's own run-to-run spread
  (docs/modules/closed-loop-fixes.md Findings).

## Bugs found (flight code; recorded, not fixed -- the finish lane owns src/)
Line numbers as of 2026-10-12 (tree c77e3e5 + the finish lane's edits).
- L1 V37 refused (alarm 1520) also lights OPR ERR. src/exec/change_program/
  sequence.c:117 cp_refuse() returns AGC_STATUS_ALARM, and
  src/io/dsky/pinball/internal.h:52 defines PIN_STATUS_RECYCLE as
  AGC_STATUS_ALARM, so pin_enter() (keys.c:448 loop) takes it for ALMCYCLE
  and runs ENTER again: pin_enter_once -> pin_dspalarm (keys.c:418) ->
  FALTON. AGC: CANTROD / V37BAD (FRESH_START_AND_RESTART.agc L p.227):
  ALARM 1520, RELDSP, PINBRNCH -> ERASER -> ENDOFJOB, no OPR ERR; ALMCYCLE
  (PINBALL L p.446) is only reached through explicit TCs. yaAGC: PROG lamp
  only. Reproduce: sim_client lockstep with the simulator's connect inputs
  (ch30 = 36331: IMU operate) + V36E, V37E00E within 90 s. Any other
  load-resume routine returning ALARM would recycle the same way.
- L2 V42 fine align torques nothing. src/io/verbs/extended_verbs.c:828
  agc_job_keyboard_gyro_torque() (FINEK2, EXTENDED_VERBS.agc L p.273:
  `CAF LGYROBIN` = ECADR OGC, IMUPULSE) pulses gyro.c's private buffer
  gbuf[AGC_IMU_GBUF_OGC] (src/imu/gyro.c:88), but the V25N93 load writes
  agc_align_ram()->ogc (src/exec/nouns.c:235-240): one erasable (OGC, IGC,
  MGC) became two stores, and only alignment.c:164 / systems_test.c:1394
  copy one into the other. Result: GYRO ENABLE (ch14 bit 6) on, no gyro
  select / activity bits, no GYROCMD, NO ATT off; the simulator's gimbals do
  not move (yaAGC: +1.000 / +2.000 / -1.500 deg, Y-GYRO 1024 x 11 + 387 ...).
- L3 POODOO 1103 (CCSHOLE) during RCS jet firings. src/lm/dap/rcs/
  t6rupt.c:252 lm_dap_jtlst() MIDORLST: `t6next[0] = t - time6` is +0 when
  the new firing ends in the T6 count TIME6 now holds; the AGC's
  `EXTEND / SU TIME6` (Q_R-AXIS_RCS_AUTOPILOT.agc L p.1457) gives -0 in
  ones' complement. T6JOB (t6rupt.c:327 set_time6(head)) then loads TIME6 =
  +0 with the clock enabled; at the next T6RUPT T6JOBCHK (T6-RUPT_PROGRAMS
  L p.1404) finds +0 -> CCSHOLE -> POODOO 01103 -> V37 flashing, DAP reset.
  Caught with gdb (agc_ccshole <- agc_lm_dap_t6jobchk <- agc_lm_dap_t6rupt).
  Seen in every RHC run that fired jets (FAILREG 01103 01103 01103).

## Findings (tutorial phases, build/agc_sim vs yaAGC, same simulator and crew sequence)
Runs: `sim/lm_simulator/lm_sim.py --agc both` with sim/lm_simulator/tutorial.txt,
headless and --gui (2026-10-12; agc_sim built from c77e3e5 + working tree).

| Phase (tutorial section) | Loop closed? | yaAGC | build/agc_sim | Verdict |
|---|---|---|---|---|
| connect (start order AGC then wish) | -- | connects on 19801; sends ch5/6 on every write (20/s, 10/s while idle) | connects; ch5/6 only on change (0 packets idle) | S-L1 below |
| power-on, no core file | -- | PROG lamp, alarm 1107 (phase-table check on zero erasable), PROG/VERB blank | RESTART lamp, PROG 00, V37 flashing | expected: different power-on images (yaAGC zero erasable, ours agc_power_on); not compared |
| V35 lights test (0.0 / 4.0) | -- | all 88, all lamps | same | match |
| V36 fresh start (0.1) | -- | DSKY blank, ENGINE OFF | same | match |
| V37E00E within 90 s of IMU operate | -- | alarm 1520 (IMODES30 bit 6), PROG lamp | alarm 1520, PROG **and OPR ERR** | bug L1 |
| ISS turn-on (1.1) | IMU turn-on sequence | NO ATT on, off after 89.5 s | NO ATT on, off after 89.5 s | match |
| V41N20 coarse align 10/20/30 deg (1.3) | yes: CDU drive 0174-0176 -> gimbals -> PCDU back | gimbals 10.02/20.00/30.02, N20 +01002 +02000 +03001, NO ATT on; alarm 00211 | same gimbals and N20, NO ATT on; no 00211 | gimbal loop matches; 00211 is the CDU-count pacing (S-L2) |
| V42 fine align +1/+2/-1.5 deg (1.4) | yes in yaAGC: gyro 0177 -> gimbals | 11.02/22.00/28.52, N20 follows; NO ATT off | gimbals unmoved, N20 unchanged; NO ATT off | bug L2 |
| V40N20 zero IMU (1.5) | yes: ch12 bit 5 -> zero | 0/0/0, N20 zero | same | match |
| V48 R03 DAP data (2.0) | -- | V01N46 21112, V06N47 +15000 +00000, V06N48 +00000 +00000; after PRO trim, V50N48 never comes (no trim feedback in the simulator) | identical | match |
| ATT HOLD + V77, RHC pitch +30 (2.1) | yes in yaAGC | jets Q4D Q3U Q2U Q1D, pitch rate 9.7 deg/s, held after detent | same four jets commanded (1 packet), simulator applies none: rate 0, then tumbles (jets applied one change late) | S-L1; with the relay: 8.7 deg/s, held after detent |
| RHC yaw +30 | yes in yaAGC | Q3A Q1F Q2L Q4R, 10.7 deg/s | relay: same jets, 10.3 deg/s | match with relay |
| DAP alarms after the RHC runs | -- | 01204 once (POODOO, after yaw detent) | 01103 (one to three per run) | bug L3 (ours); yaAGC's 1204 noted, not ours to chase |
| V49 manoeuvre to 30/30/0 in AUTO (2.2) | yes: KALCMANU through the RCS DAP | V06N22, V50N18 +00002 +03000 +33000, gimbals 25.1/32.6/2.7 after 45 s | plain: tumbling (S-L1); relay: V50N18 +00000 +03000 +33000, 26.1/34.2/3.1 | match with relay (both stop short of 30/30/0 in this model) |
| V76 minimum impulse (2.1) | yes | Q2U+Q1D one pulse, 10-20 ms | same pulse (relay) | match |
| P63/P66 descent | not modelled | -- | -- | the simulator has no gravity, trajectory, radar or AGC throttle |

- S-L1 (src/sim, protocol): src/sim/channels.c send_if_changed() (:200)
  broadcasts an output channel only when its value changes; yaAGC sends a
  packet on every WRITE/WOR/WAND (agc_engine.c CpuWriteIO -> ChannelOutput,
  :659). The LM Simulator's process_data (lm_simulator.tcl:813-821) sets its
  jet flags from bo(5/6,..) before it updates bo from the packet, i.e. it
  applies the previous packet's jets; with yaAGC's 20 writes/s the lag is
  one DAP pass, with agc_sim it is a whole firing (jets act when they have
  stopped). That breaks every RCS loop. A drop-in agc_sim must send a packet
  per channel write (at least 5, 6). Shown by sim/lm_simulator/lm_sim.py --repeat-jets
  (a relay re-sending the last ch5/6 words every 50 ms): the RCS loops then
  close and match yaAGC within tolerance.
- S-L2 (src/sim, counters): src/sim/counters.c applies PCDU/MCDU (and the
  fast 021/023) at once; yaAGC queues CDU counts in a FIFO at 400 cps (slow)
  / 6400 cps (fast) (agc_engine.c PushCduFifo). The simulator picks slow mode
  from the body rate, not the gimbal rate, so under yaAGC the CDUs lag the
  coarse-align drive and COARS raises 00211; ours does not. Both end on the
  same gimbal angles. Hardware-faithful pacing is a sim-lane decision.
- D1 (open): after a POODOO restart (1103 / 1204) yaAGC shows PROG blank,
  ours PROG 00 (GOPROG3 MMDSPLAY, FRESH_START_AND_RESTART L p.217; MODREG
  after the restart). Not root-caused.

## Lessons
- A peripheral written against yaAGC can depend on yaAGC's packet cadence,
  not only on its packet format: the LM Simulator's one-packet lag is
  invisible at 20 writes/s and fatal with change-only output. "Drop-in"
  includes when packets are sent.
- Tk's `send` is refused on this X server (no xauth-style authorization),
  so drive a Tcl/Tk peripheral through a wrapper that sources it unchanged
  and opens a control socket; under tclsh, inert Tk stand-ins that keep
  -command/-variable make the same wrapper headless.
- The tutorial was written for Luminary 131: in Luminary099 V60 is "LR to
  position 2", not rate needles; R03 ends with V50N48 only after a gimbal
  trim the simulator cannot report; V49 is refused (OPR ERR) while R03's
  extended verb is still active; V37 is refused (1520) for 90 s after IMU
  operate, which the simulator asserts at connect.
- Compare against yaAGC in the same harness, not against expectations:
  several oddities (1204 POODOO, V50N48 never coming, manoeuvres stopping
  short) are yaAGC + simulator behaviour, not ours.
- gdb on build/agc_sim (-O2 -g) through a wrapper passed as --sim (or as
  sim_client --sim) catches flight-code alarms in live real-time runs:
  `break agc_ccshole` gave bug L3's call chain at once.

