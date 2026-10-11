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
