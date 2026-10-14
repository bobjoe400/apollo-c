# docs/history/lanes -- lane diaries

One file per lane in [docs/modules/](../../docs/modules/). Each holds what the lane note used to carry besides the explanation: progress logs, findings, bugs found, requests to other lanes, open questions, lessons, test changes, routed items and checkpoints, moved verbatim (2026-10-14). The explanation of the module (scope, design, deviations, stubs, tests, program flow and decisions) stays in the lane note; the lessons are consolidated by topic in [TIPS.md](../../docs/TIPS.md).

* [closed-loop-fixes](closed-loop-fixes.md) -- diary of [closed-loop-fixes](../../docs/modules/closed-loop-fixes.md): LM Simulator bugs (L1-L3, S-L1..S-L3, PROG after POODOO) and the document wave's code findings
* [core-numeric](core-numeric.md) -- diary of [core-numeric](../../docs/modules/core-numeric.md): fixed-point word model and interpretive arithmetic
* [dap-csm](dap-csm.md) -- diary of [dap-csm](../../docs/modules/dap-csm.md): CSM RCS and TVC digital autopilots (Comanche055)
* [dap-lm](dap-lm.md) -- diary of [dap-lm](../../docs/modules/dap-lm.md): Luminary 099 LM digital autopilot (RCS DAP, trim gimbal, 1/ACCS)
* [exec-sched](exec-sched.md) -- diary of [exec-sched](../../docs/modules/exec-sched.md): executive, waitlist, phase tables, restarts, alarms
* [guid-entry](guid-entry.md) -- diary of [guid-entry](../../docs/modules/guid-entry.md): Comanche entry (P61-P67), entry DAP, CM body attitude, P37, P11
* [guid-landing](guid-landing.md) -- diary of [guid-landing](../../docs/modules/guid-landing.md): Luminary landing, aborts, ascent, throttle, FINDCDUW
* [guid-powered](guid-powered.md) -- diary of [guid-powered](../../docs/modules/guid-powered.md): powered-flight guidance (P30s, P40s, SERVICER, BURNBABY)
* [imu](imu.md) -- diary of [imu](../../docs/modules/imu.md): IMU moding, compensation, alignment, attitude maneuvers
* [io-nouns](io-nouns.md) -- diary of [io-nouns](../../docs/modules/io-nouns.md): PINBALL noun tables (PINBALL_NOUN_TABLES.agc, CSM + LM)
* [io-pinball](io-pinball.md) -- diary of [io-pinball](../../docs/modules/io-pinball.md): DSKY / PINBALL / display interface / KEYRUPT
* [lm-simulator](lm-simulator.md) -- diary of [lm-simulator](../../docs/modules/lm-simulator.md): Virtual AGC LM Simulator closed loop on build/agc_sim vs yaAGC
* [nassp-integration](nassp-integration.md) -- diary of [nassp-integration](../../docs/modules/nassp-integration.md): Apollo 11 in NASSP 9 on our AGC (PLAN item 10b+10c)
* [nav-aux](nav-aux.md) -- diary of [nav-aux](../../docs/modules/nav-aux.md): lat/long, planetary orientation, measurement incorporation, TFF, ephemerides, star tables
* [nav-conic](nav-conic.md) -- diary of [nav-conic](../../docs/modules/nav-conic.md): CONIC_SUBROUTINES.agc (+ KEPPREP, CSMCONIC/LEMCONIC conic path)
* [nav-integ](nav-integ.md) -- diary of [nav-integ](../../docs/modules/nav-integ.md): INTEGRATION_INITIALIZATION + ORBITAL_INTEGRATION
* [oracle-cases](oracle-cases.md) -- diary of [oracle-cases](../../docs/modules/oracle-cases.md): yaAGC differential cases for the numeric modules
* [oracle](oracle.md) -- diary of [oracle](../../docs/modules/oracle.md): Virtual AGC differential-test rig
* [organize](organize.md) -- diary of [organize](../../docs/modules/organize.md): repository index, layout review, layout script
* [p37](p37.md) -- diary of [p37](../../docs/modules/p37.md): P37 return to earth (Comanche055 P37_P70.agc, pp. 890-933)
* [prog-align](prog-align.md) -- diary of [prog-align](../../docs/modules/prog-align.md): IMU alignment programs, optics marking, geometry, S-band, R63, R47
* [prog-rendz](prog-rendz.md) -- diary of [prog-rendz](../../docs/modules/prog-rendz.md): rendezvous programs (P17, P20-P25, P31-P39, P72-P79, P76, P21, R30, R31, R36)
* [review-core-numeric](review-core-numeric.md) -- diary of [review-core-numeric](../../docs/modules/review-core-numeric.md): review of the core-numeric lane
* [review-dap-csm](review-dap-csm.md) -- diary of [review-dap-csm](../../docs/modules/review-dap-csm.md): review of the dap-csm lane
* [review-dap-lm](review-dap-lm.md) -- diary of [review-dap-lm](../../docs/modules/review-dap-lm.md): review of the LM DAP lane (src/lm/dap)
* [review-exec-sched](review-exec-sched.md) -- diary of [review-exec-sched](../../docs/modules/review-exec-sched.md): review of the exec-sched lane
* [review-guid-entry](review-guid-entry.md) -- diary of [review-guid-entry](../../docs/modules/review-guid-entry.md): review of the guid-entry and p37 lanes
* [review-guid-landing](review-guid-landing.md) -- diary of [review-guid-landing](../../docs/modules/review-guid-landing.md): review of the guid-landing lane
* [review-guid-powered](review-guid-powered.md) -- diary of [review-guid-powered](../../docs/modules/review-guid-powered.md): review of the powered-flight guidance lane
* [review-imu](review-imu.md) -- diary of [review-imu](../../docs/modules/review-imu.md): review of the imu lane
* [review-io-dsky](review-io-dsky.md) -- diary of [review-io-dsky](../../docs/modules/review-io-dsky.md): review of io-pinball + io-nouns
* [review-nav-aux](review-nav-aux.md) -- diary of [review-nav-aux](../../docs/modules/review-nav-aux.md): review of the nav-aux lane
* [review-nav-conic](review-nav-conic.md) -- diary of [review-nav-conic](../../docs/modules/review-nav-conic.md): review of the nav-conic lane
* [review-nav-integ](review-nav-integ.md) -- diary of [review-nav-integ](../../docs/modules/review-nav-integ.md): review of the nav-integ lane
* [review-prog-align](review-prog-align.md) -- diary of [review-prog-align](../../docs/modules/review-prog-align.md): review of the prog-align lane
* [review-prog-rendz](review-prog-rendz.md) -- diary of [review-prog-rendz](../../docs/modules/review-prog-rendz.md): review of the prog-rendz lane
* [review-sim](review-sim.md) -- diary of [review-sim](../../docs/modules/review-sim.md): review of the sim harness lane
* [review-state-flags](review-state-flags.md) -- diary of [review-state-flags](../../docs/modules/review-state-flags.md): review note
* [review-sys-misc](review-sys-misc.md) -- diary of [review-sys-misc](../../docs/modules/review-sys-misc.md): review of the sys-misc lane
* [sim](sim.md) -- diary of [sim](../../docs/modules/sim.md): host harness: clock/interrupt driver + yaAGC peripheral protocol
* [state-flags](state-flags.md) -- diary of [state-flags](../../docs/modules/state-flags.md): module note
* [structure](structure.md) -- diary of [structure](../../docs/modules/structure.md): duplicates, cross-vehicle placement, uncalled labels, splits, internal.h
* [sys-misc](sys-misc.md) -- diary of [sys-misc](../../docs/modules/sys-misc.md): module note
* [telemetry](telemetry.md) -- diary of [telemetry](../../docs/modules/telemetry.md): every downlist sends what the AGC sends
