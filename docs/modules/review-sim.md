# review-sim -- review of the sim harness lane

History of this lane: [docs/history/lanes/review-sim.md](../history/lanes/review-sim.md)

Lane: src/sim/** (clock/interrupt driver, yaAGC peripheral protocol, pad,
rope, script, main), sim/sim_client.py, sim/scenarios/** (except
external_delta_v_burn_csm.txt: fix-b6 owns it). Reference: virtualagc
yaAGC agc_engine.c (scaler, counters, interrupts, standby), SocketAPI.c
(ChannelInput/ChannelOutput/UpdateAgcPeripheralConnect), agc_utilities.c
(FormIoPacket/ParseIoPacket), agc_engine_init.c (power-on channels).
Build-only principle: src/sim holds channel/counter state, the clock, the
interrupt driver and the protocol; nothing else.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [sim](sim.md#program-flow-and-decisions).
