# lm-simulator -- Virtual AGC LM Simulator closed loop on build/agc_sim vs yaAGC

## Scope
Stephan Hotto's LM
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

    make lm-sim                                      # headless (tclsh); fetches LMSIM
                                                     # (virtualagc c239e9c, sparse) into
                                                     # build/virtualagc on first use
    make lm-sim LMSIM_DIR=$LMSIM                     # the same on your own checkout
    make lm-sim LMSIM_DIR=$LMSIM LMSIM_ARGS="--gui --xshot DIR"   # windows + screenshots
    make lm-sim LMSIM_DIR=$LMSIM LMSIM_ARGS=--repeat-jets         # diagnostic relay

= `python3 -I sim/lm_simulator/lm_sim.py --lmsim $LMSIM [--agc ours|yaagc|both]
[--sequence sim/lm_simulator/tutorial.txt] [--out build/lm_sim]`. Per AGC
it writes build/lm_sim/<agc>/run.log (sequence log + the simulator's own
log lines: CDU drive, gyro pulses) and snaps.json; then a side-by-side
table and the mismatches (exit 1 on any). Not part of `make test` (real
time, needs the virtualagc checkout).

## Program flow and decisions

_Documentation wave (2026-10-12), from sim/lm_simulator/lm_sim.py,
sim/lm_simulator/harness.tcl and sim/lm_simulator/tutorial.txt at HEAD
99b2c6e. This lane owns no flight code; the AGC paths the simulator
exercises are in the lane notes named below._

* **Harness** (`sim/lm_simulator/lm_sim.py:main`): for `--agc both` it flies the
  same sequence twice, ours then yaAGC, and compares (`compare`); exit 1
  on any mismatch. Per AGC (`Flight`): a free LM port range
  (`free_base`), the AGC process (build/agc_sim `--core` or
  tools/oracle/bin/yaAGC `--no-resume`), a monitor peripheral on base+1
  (`Monitor`: decodes channel 010/011 like yaDSKY2, logs channel 5/6 jet
  words), the simulator on base+4 (`Harness`: `tclsh` headless or `wish`
  with `--gui`, through `sim/lm_simulator/harness.tcl`, which sources the
  unmodified simulator and opens a control socket; `tcl` refuses any
  reply that is not `OK`). `--repeat-jets` puts a relay (`Relay`) between
  ours and the simulator that re-sends the last ch5/6 words every 50 ms
  (diagnostic for S-L1).
* **Sequence** (`Flight` playing sim/lm_simulator/tutorial.txt): `keys`,
  `pro`, `crew`, `rhc`, `wait`; `until` waits for a DSKY field (default
  30 s); `expect` checks it now; `snap` records a checkpoint with
  tolerances (deg, rate, needle, jets, reg; `=skip` per field;
  `hold=R` checks each AGC alone: every body rate within R deg/s).
* **Comparison** (`compare`): checkpoint by checkpoint, DSKY text,
  gimbal angles, body rates, needles and jets used, within the snap's
  tolerances. After the first RHC detent the tutorial compares displays,
  alarms, jets used and rate during a deflection, and `hold=1.5` instead
  of attitude, because yaAGC against itself is not repeatable there
  (closed-loop-fixes Findings).
* **What it exercises**: IMU zero / coarse / fine align and gyro torquing
  ([imu.md](imu.md#program-flow-and-decisions)), the RCS DAP and T6 jet
  timing ([dap-lm.md](dap-lm.md#program-flow-and-decisions)), pinball and
  V37 ([io-pinball.md](io-pinball.md#program-flow-and-decisions)), the
  restart path after a POODOO
  ([exec-sched.md](exec-sched.md#program-flow-and-decisions)) and the
  sim's packet and CDU pacing ([sim.md](sim.md#program-flow-and-decisions)).
