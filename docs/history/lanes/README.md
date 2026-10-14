# docs/history/lanes -- lane diaries

One file per lane in [docs/modules/](../../modules/). Each holds what the lane note used to carry besides the explanation: progress logs, findings, bugs found, requests to other lanes, open questions, lessons, test changes, routed items and checkpoints, moved verbatim (2026-10-14). The explanation of the module (scope, design, deviations, stubs, tests, program flow and decisions) stays in the lane note; the lessons are consolidated by topic in [TIPS.md](../../TIPS.md).

* [closed-loop-fixes](closed-loop-fixes.md) -- diary of [closed-loop-fixes](../../modules/closed-loop-fixes.md): LM Simulator bugs (L1-L3, S-L1..S-L3, PROG after POODOO) and the document wave's code findings
* [core-numeric](core-numeric.md) -- diary of [core-numeric](../../modules/core-numeric.md): fixed-point word model and interpretive arithmetic
* [dap-csm](dap-csm.md) -- diary of [dap-csm](../../modules/dap-csm.md): CSM RCS and TVC digital autopilots (Comanche055)
* [dap-lm](dap-lm.md) -- diary of [dap-lm](../../modules/dap-lm.md): Luminary 099 LM digital autopilot (RCS DAP, trim gimbal, 1/ACCS)
* [exec-sched](exec-sched.md) -- diary of [exec-sched](../../modules/exec-sched.md): executive, waitlist, phase tables, restarts, alarms
* [guid-entry](guid-entry.md) -- diary of [guid-entry](../../modules/guid-entry.md): Comanche entry (P61-P67), entry DAP, CM body attitude, P37, P11
* [guid-landing](guid-landing.md) -- diary of [guid-landing](../../modules/guid-landing.md): Luminary landing, aborts, ascent, throttle, FINDCDUW
* [guid-powered](guid-powered.md) -- diary of [guid-powered](../../modules/guid-powered.md): powered-flight guidance (P30s, P40s, SERVICER, BURNBABY)
* [imu](imu.md) -- diary of [imu](../../modules/imu.md): IMU moding, compensation, alignment, attitude maneuvers
* [io-nouns](io-nouns.md) -- diary of [io-nouns](../../modules/io-nouns.md): PINBALL noun tables (PINBALL_NOUN_TABLES.agc, CSM + LM)
* [io-pinball](io-pinball.md) -- diary of [io-pinball](../../modules/io-pinball.md): DSKY / PINBALL / display interface / KEYRUPT
* [lm-simulator](lm-simulator.md) -- diary of [lm-simulator](../../modules/lm-simulator.md): Virtual AGC LM Simulator closed loop on build/agc_sim vs yaAGC
* [nassp-integration](nassp-integration.md) -- diary of [nassp-integration](../../modules/nassp-integration.md): Apollo 11 in NASSP 9 on our AGC (PLAN item 10b+10c)
* [nav-aux](nav-aux.md) -- diary of [nav-aux](../../modules/nav-aux.md): lat/long, planetary orientation, measurement incorporation, TFF, ephemerides, star tables
* [nav-conic](nav-conic.md) -- diary of [nav-conic](../../modules/nav-conic.md): CONIC_SUBROUTINES.agc (+ KEPPREP, CSMCONIC/LEMCONIC conic path)
* [nav-integ](nav-integ.md) -- diary of [nav-integ](../../modules/nav-integ.md): INTEGRATION_INITIALIZATION + ORBITAL_INTEGRATION
* [oracle-cases](oracle-cases.md) -- diary of [oracle-cases](../../modules/oracle-cases.md): yaAGC differential cases for the numeric modules
* [oracle](oracle.md) -- diary of [oracle](../../modules/oracle.md): Virtual AGC differential-test rig
* [organize](organize.md) -- diary of [organize](../../modules/organize.md): repository index, layout review, layout script
* [p37](p37.md) -- diary of [p37](../../modules/p37.md): P37 return to earth (Comanche055 P37_P70.agc, pp. 890-933)
* [prog-align](prog-align.md) -- diary of [prog-align](../../modules/prog-align.md): IMU alignment programs, optics marking, geometry, S-band, R63, R47
* [prog-rendz](prog-rendz.md) -- diary of [prog-rendz](../../modules/prog-rendz.md): rendezvous programs (P17, P20-P25, P31-P39, P72-P79, P76, P21, R30, R31, R36)
* [review-core-numeric](review-core-numeric.md) -- diary of [review-core-numeric](../../modules/review-core-numeric.md): review of the core-numeric lane
* [review-dap-csm](review-dap-csm.md) -- diary of [review-dap-csm](../../modules/review-dap-csm.md): review of the dap-csm lane
* [review-dap-lm](review-dap-lm.md) -- diary of [review-dap-lm](../../modules/review-dap-lm.md): review of the LM DAP lane (src/lm/dap)
* [review-exec-sched](review-exec-sched.md) -- diary of [review-exec-sched](../../modules/review-exec-sched.md): review of the exec-sched lane
* [review-guid-entry](review-guid-entry.md) -- diary of [review-guid-entry](../../modules/review-guid-entry.md): review of the guid-entry and p37 lanes
* [review-guid-landing](review-guid-landing.md) -- diary of [review-guid-landing](../../modules/review-guid-landing.md): review of the guid-landing lane
* [review-guid-powered](review-guid-powered.md) -- diary of [review-guid-powered](../../modules/review-guid-powered.md): review of the powered-flight guidance lane
* [review-imu](review-imu.md) -- diary of [review-imu](../../modules/review-imu.md): review of the imu lane
* [review-io-dsky](review-io-dsky.md) -- diary of [review-io-dsky](../../modules/review-io-dsky.md): review of io-pinball + io-nouns
* [review-nav-aux](review-nav-aux.md) -- diary of [review-nav-aux](../../modules/review-nav-aux.md): review of the nav-aux lane
* [review-nav-conic](review-nav-conic.md) -- diary of [review-nav-conic](../../modules/review-nav-conic.md): review of the nav-conic lane
* [review-nav-integ](review-nav-integ.md) -- diary of [review-nav-integ](../../modules/review-nav-integ.md): review of the nav-integ lane
* [review-prog-align](review-prog-align.md) -- diary of [review-prog-align](../../modules/review-prog-align.md): review of the prog-align lane
* [review-prog-rendz](review-prog-rendz.md) -- diary of [review-prog-rendz](../../modules/review-prog-rendz.md): review of the prog-rendz lane
* [review-sim](review-sim.md) -- diary of [review-sim](../../modules/review-sim.md): review of the sim harness lane
* [review-state-flags](review-state-flags.md) -- diary of [review-state-flags](../../modules/review-state-flags.md): review note
* [review-sys-misc](review-sys-misc.md) -- diary of [review-sys-misc](../../modules/review-sys-misc.md): review of the sys-misc lane
* [sim](sim.md) -- diary of [sim](../../modules/sim.md): host harness: clock/interrupt driver + yaAGC peripheral protocol
* [state-flags](state-flags.md) -- diary of [state-flags](../../modules/state-flags.md): module note
* [structure](structure.md) -- diary of [structure](../../modules/structure.md): duplicates, cross-vehicle placement, uncalled labels, splits, internal.h
* [sys-misc](sys-misc.md) -- diary of [sys-misc](../../modules/sys-misc.md): module note
* [telemetry](telemetry.md) -- diary of [telemetry](../../modules/telemetry.md): every downlist sends what the AGC sends
