# lm-simulator -- Virtual AGC LM Simulator closed loop on build/agc_sim vs yaAGC

## Scope
PLAN item 10, phase 1 (docs/agents/lm-simulator.md). Stephan Hotto's LM
Simulator (SRC/virtualagc/Contributed/LM_Simulator, Tcl/Tk, v1.0 with Ron
Burkey's 2009/2025 fixes) attached to build/agc_sim (Luminary099) exactly
as to yaAGC, then the same sequence against tools/oracle/bin/yaAGC. Nothing
of the simulator is modified; no device model on our side.

## What the simulator is (read from readme.html, lm_simulator.tcl, modules/)
- Client of yaAGC's peripheral socket protocol; connects to ServerSocket
  19801 (= LM base 19797 + 4), `--port=` / `--cfg=` on its command line
  override the .ini. Start order: AGC first, then `wish lm_simulator.tcl`
  from the LM_Simulator directory (it sources ./modules/, ./doc/).
- On connect it writes channels 016, 030-033 with masks (crew switches:
  ISS turn-on not requested, AGC has control, DAP off, ...).
- Its own "DSKY Lite" (dsky on) decodes relays 010 / lamps 011 and sends
  keys on 015 (mask 037), PRO on 032 bit 14 (released 200 ms later).
- Model: IMU gimbals (coarse align from CDU drive 0174-0176 when ch12 bit 4,
  else FDAI error needles; fine align from gyro 0177; zero on ch12 bit 5)
  sending PCDU/MCDU (021/023 fast above 4 deg/s) on CDUX/Y/Z 032-034; rigid
  body rotation from the 16 RCS jets (ch 5/6) with an inertia hyperbola of
  mass; PIPA PINC/MINC from RCS translation and from crew-set DPS/APS thrust;
  crew switches (ch 30-33), RHC (0166-0170 + ch31), THC.
- Not modelled: gravity, trajectory, landing / rendezvous radar, throttle
  from the AGC (the DPS thrust is a manual slider, engine on/off a crew
  checkbox). So P63/P66 cannot close a loop here; the loops it does close
  are IMU (zero, coarse align, gyro fine align) and the RCS DAP (attitude
  hold, rate command, minimum impulse, V49 automatic manoeuvre, needles).

## Progress
- DONE step 1 (connect): agc_sim LM peripheral server + stock
  lm_simulator.ini; simulator connects on 19801, all six windows up, DSKY
  Lite shows PROG 00 / VERB 37 flashing after power-on.
- DONE harness (2026-10-12): tools/lm_sim_harness.tcl sources the
  simulator unmodified (wish = its windows; tclsh = inert Tk stand-ins that
  keep each widget's -command/-variable so `invoke` runs its own button
  script) and adds a control port; tools/lm_sim.py starts an AGC (agc_sim or
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

## How to run it
LMSIM = virtualagc/Contributed/LM_Simulator (here SRC/virtualagc/...,
virtualagc commit c239e9c, tools/oracle/BUILD.md). Tcl/Tk 8.6 (`wish`).

By hand, exactly as against yaAGC (two terminals, AGC first):

    build/agc_sim --core tools/oracle/ropes/Luminary099.bin --port 19797
    cd $LMSIM && wish lm_simulator.tcl            # stock lm_simulator.ini: port 19801

(yaAGC: `tools/oracle/bin/yaAGC --core=tools/oracle/ropes/Luminary099.bin
--port=19797 --no-resume` from a scratch directory, it writes core dumps.)
The stock .ini works unchanged (ServerSocket 19801 = 19797 + 4, DSKY Lite
on); `--port=N` / `--cfg=F` on the wish line override it. A yaDSKY2 can
join on any other port of the ten (yaDSKY2 --port=19797 --cfg=LM.ini).

Scripted comparison (both AGCs, same crew sequence, real time ~11 min):

    make lm-sim LMSIM_DIR=$LMSIM                     # headless (tclsh)
    make lm-sim LMSIM_DIR=$LMSIM LMSIM_ARGS="--gui --xshot DIR"   # windows + screenshots
    make lm-sim LMSIM_DIR=$LMSIM LMSIM_ARGS=--repeat-jets         # diagnostic relay

= `python3 -I tools/lm_sim.py --lmsim $LMSIM [--agc ours|yaagc|both]
[--sequence sim/lm_simulator/tutorial.txt] [--out build/lm_sim]`. Per AGC
it writes build/lm_sim/<agc>/run.log (sequence log + the simulator's own
log lines: CDU drive, gyro pulses) and snaps.json; then a side-by-side
table and the mismatches (exit 1 on any). Not part of `make test` (real
time, needs the virtualagc checkout).

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
Runs: `tools/lm_sim.py --agc both` with sim/lm_simulator/tutorial.txt,
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
  per channel write (at least 5, 6). Shown by tools/lm_sim.py --repeat-jets
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
