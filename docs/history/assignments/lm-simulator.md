# Assignment: lm-simulator   (tier: integration; Opus)

PLAN item 10, phase 1 (user, 2026-10-12): the one closed-loop flight test
available on Linux with an existing proven tool. The Virtual AGC
contributed LM Simulator (SRC/virtualagc/Contributed/LM_Simulator,
Stephan Hotto, Tcl/Tk) models the LM (IMU, RCS quads, FDAI, crew switches,
discrete inputs/outputs, a rigid-body model, descent as far as it goes)
and talks to the AGC over yaAGC's peripheral socket protocol. Run it
against build/agc_sim (Luminary099 / LM) exactly as it would run against
yaAGC. Nothing of the simulator is modified beyond its own .ini (ports,
paths); no device model is written on our side.

1. Read Documentation/readme.html and lm_simulator.ini; find how it
   expects to connect (ports, the DSKY it uses, the startup order). Start
   agc_sim with the LM rope on the LM ports and attach the simulator
   (`wish lm_simulator.tcl` with the right .ini). Record the exact
   commands in docs/modules/lm-simulator.md "How to run it".
2. Walk its tutorial (section 4): power-up, V37E00E, P52-type alignment
   if supported, attitude hold and rate commands through the DAP (P63/P66
   if the model supports descent, else the attitude/RCS loop it does
   support). For each step note what the real AGC would do and what ours
   did; every mismatch goes to "Bugs found" with file:line and the AGC
   page, unfixed (the finish lane may be editing src/; record only).
3. Repeat the same sequence against tools/oracle/bin/yaAGC with the
   Luminary099 rope (the simulator's intended partner) and compare the
   DSKY displays, FDAI/needle behaviour and jet firings side by side; that
   comparison is the test. Screenshots under docs/modules/lm-simulator/.
4. Headless equivalent where possible: if the simulator can be driven
   without its GUI (ini `nodsky` variants, Tcl without wish) make a
   `make lm-sim` target that runs the comparison; otherwise document the
   manual procedure.
GUI windows: use the real display only if no Xvfb is available; close
every window when a check is done. Do not edit src/, include/, tests/
or tools/prove. Note with Progress (checkpoint per step), Findings,
Bugs found, Lessons; MODULE_STATUS row. Final report <= 12 lines: which
phases closed the loop, where ours differed from yaAGC, bugs filed.
