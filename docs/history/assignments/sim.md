# Assignment: sim   (tier: translation)

Definition-of-done item 7 (docs/PLAN.md): a host harness in src/sim/ that
runs the translated flight software end to end. Runs after `integrate`.

1. **Clock and interrupts.** A software clock in centiseconds. Each tick
   advances TIME1/TIME2, drives T3RUPT (waitlist), T4RUPT (120 ms), T5RUPT
   (DAP, 100 ms when enabled), T6RUPT (jet timing) and the executive loop
   (agc_exec_run_one until idle), in the AGC's priority order, with
   INHINT/RELINT respected. The one unbounded loop allowed by rule 2 lives
   here and is documented as such.
2. **Hardware model** (minimal, deterministic): IMU CDU counters driven by
   a rigid-body attitude integrated from jet/gimbal commands (channels
   005/006/014), PIPA counters from a commanded acceleration profile,
   optics/AOT mark buttons, rendezvous radar and landing radar returning
   values from a scripted truth, uplink word injection, downlink capture.
   Keep it in src/sim/hw_*.c with a struct per device; no dynamic memory.
3. **DSKY console.** Text mode: prints VERB/NOUN/PROG/R1-R3 and lamps
   whenever the display changes (reads the agc_dsky state), accepts key
   presses from a script file or stdin (`V 0 6 N 3 6 E`, `PRO`, `KEY REL`).
4. **Scenarios** in sim/scenarios/*.txt (key presses + hardware script +
   expected displays), run by `build/agc_sim --vehicle csm|lm --scenario
   FILE --max-cs N`, exit non-zero on mismatch:
   a. fresh start (V36), both vehicles: PROG 00, expected lamp state.
   b. V35 lights test.
   c. V06N36 clock display matches the sim clock.
   d. CSM P52 option 3 realign: inject two simulated star marks consistent
      with the loaded REFSMMAT; expect N05 error ~0 and the gyro-torque
      angles N93 near zero; PRO through.
   e. CSM P30 + P40: load a small external delta-V, run through TIG-30,
      TIG-5, ignition (ullage/engine-on channel bits appear), burn until
      VG < threshold, engine-off; expect N85 residuals near zero.
   f. LM P63 braking phase from a scripted PDI state for 60 s of guidance:
      throttle commands plausible, no 1201/1202 unless radar data is
      flooded (document the executive-overload reproduction as a bonus).
5. `make sim` builds it; `make scenarios` runs all and is added to
   `make test`. Each scenario documents the AGC behaviour it checks with
   page references. docs/modules/sim.md with Progress and Lessons.

Scope boundary (parallel run): write only src/sim/, sim/scenarios/, the
Makefile sim/scenarios targets, and docs. If a flight-code bug blocks a
scenario, record it under "Bugs found" in your note with file:line and the
AGC reference and stop that scenario; the collapse lane is editing src/.

## REVISED SCOPE (user direction, 2026-10-10): reuse Virtual AGC peripherals

Do not write a DSKY console or device models. Make src/sim a drop-in for
yaAGC from the peripherals' point of view:
1. Implement yaAGC's peripheral socket protocol (TCP server, default port
   19697 and the per-peripheral conventions; packet format in
   SRC/virtualagc/yaAGC/agc_utilities.c FormIoPacket/ParseIoPacket and the
   Virtual AGC docs). Outbound: every write to an output channel
   (010 relays, 011 lamps, 012-014, 005/006 jets, 034/035 downlink) is
   broadcast as a packet; inbound packets set input channels (015/016
   keys, 030-033 discretes, counters) and raise the matching interrupt
   (KEYRUPT1/2, UPRUPT, etc.) through src/exec/interrupts.c.
2. Verify with the real yaDSKY2 (SRC/virtualagc/yaDSKY2, build if needed)
   connected to build/agc_sim: V35 lights test lights the lamps; V06N36
   shows the clock; keys arrive as KEYRUPT. Also try yaTelemetry on the
   downlink.
3. Scripted scenarios: a small Python client (tools/sim_client.py) that
   speaks the same protocol: sends key packets, collects channel 010/011
   packets, decodes relay words into VERB/NOUN/PROG/R1-R3 and lamps, and
   asserts expected displays. Scenarios a-e of the original list run
   through it; scenario f (P63) only if the LM_Simulator or a scripted
   radar/PIPA feed makes it meaningful -- otherwise document and skip.
4. Internal clock/interrupt driver as originally specified (that part is
   ours). Counters (CDU, PIPA, RHC) advance from the peripheral packets
   or from a `--script` of timed counter increments; no physics.
Keep the minimal `main.c` that integrate wrote working for `--headless`.

## RESUMED (user, 2026-10-10)
GUI verification with yaDSKY2/yaTelemetry is allowed again. Order of work:
protocol server + headless scripted scenarios via tools/sim_client.py
first; GUI verification last. Close GUI windows when a check is done.
The collapse lane is editing src/ (not src/sim) concurrently; do not edit
flight code, record bugs in your note.

## END GOAL (user, 2026-10-11): drop-in for yaAGC under the VirtualAGC GUI
build/agc_sim must be launchable by the stock VirtualAGC launcher
(SRC/virtualagc; see how it spawns yaAGC: command line, working directory,
port, the --core rope path, --cfg, --resume/--debug flags) in place of
yaAGC. Accept yaAGC's command line; map --core=<path> to the vehicle (a
Comanche055 rope path -> CSM, Luminary099 -> LM); honour --port; ignore
flags that have no meaning for us with a logged note. Speak the peripheral
protocol exactly (packet format, channel mapping, the initial channel
state yaDSKY2 expects on connect, the DSKY "standby"/RESTART lamp
behaviour). Acceptance: the GUI sequences listed in docs/PLAN.md item 7,
each also as a headless scenario. Document the one-line way to run it:
`VirtualAGC` launcher with the AGC binary path pointed at build/agc_sim,
or the equivalent manual command.

## No device models (user, 2026-10-11)
Do not model hardware behaviour in src/sim beyond holding channel/counter
state and raising interrupts as yaAGC does. Scripted scenarios may feed
counter increments the way a peripheral would over the protocol, but any
behavioural model (attitude, trajectory, radar truth) is out of scope;
that comes from Virtual AGC's peripherals or NASSP.
