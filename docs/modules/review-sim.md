# review-sim -- review of the sim harness lane

Lane: src/sim/** (clock/interrupt driver, yaAGC peripheral protocol, pad,
rope, script, main), tools/sim_client.py, sim/scenarios/** (except
external_delta_v_burn_csm.txt: fix-b6 owns it). Reference: virtualagc
yaAGC agc_engine.c (scaler, counters, interrupts, standby), SocketAPI.c
(ChannelInput/ChannelOutput/UpdateAgcPeripheralConnect), agc_utilities.c
(FormIoPacket/ParseIoPacket), agc_engine_init.c (power-on channels).
Build-only principle: src/sim holds channel/counter state, the clock, the
interrupt driver and the protocol; nothing else.

## Progress
- STARTED 2026-10-12. Baseline: make OK; scenarios 12/12 pass (the burn
  scenario fails 3 checks in the fix-b6 work in progress, not ours);
  check_rules src/sim 0 hard; check_style src/sim 524 findings.
- TODO (1) fidelity: clock/interrupt driver, protocol
- TODO (2) scope check
- TODO (3) rules 7/2/4/5
- TODO (4) style 524 -> 0, make format
- TODO (5) tools/sim_client.py

## Findings
(id, file, yaAGC reference, what, resolution)

## Lessons
