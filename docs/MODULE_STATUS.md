# MODULE_STATUS.md -- one line per agent, updated by that agent

Legend: TODO / IN-PROGRESS / DONE (n stubs) / BLOCKED (reason)

| Agent id     | Status | Files produced | Notes |
|--------------|--------|----------------|-------|
| core-numeric | IN-PROGRESS | src/core/agc_*.c (numeric), include/agc/agc_constants.h | see docs/modules/core-numeric.md |
| exec-sched   | TODO | | |
| state-flags  | DONE (0 stubs) | include/agc/agc_flags_{common,csm,lm}.h, agc_channels.h, agc_state.h; src/core/agc_state.c, agc_channels.c, agc_erasable.c; tests/test_state.c, test_state_lm.c | CSM channel bits derived from code comments (no CM bit table in source); see docs/modules/state-flags.md |
| nav-conic    | TODO | | |
| nav-integ    | IN-PROGRESS | include/agc/agc_integ.h, src/nav/integration_init.c, src/nav/orbital_integration.c | design written |
| nav-aux      | IN-PROGRESS | src/nav/{latlong,planetary,measurement,free_fall,ephemerides,star_tables}.c | see docs/modules/nav-aux.md |
| io-pinball   | IN-PROGRESS | include/agc/agc_dsky.h, src/io/pinball_*.c, src/io/display_if.c | see docs/modules/io-pinball.md |
| io-nouns | DONE (0 stubs; 182 data ids UNIMPLEMENTED pending core accessors) | include/agc/agc_nouns.h, src/io/noun_tables_{csm,lm}.c, noun_scales.c, noun_data.c, tests/test_nouns.c | see docs/modules/io-nouns.md |
| imu          | IN-PROGRESS | src/imu, src/csm/prelaunch_align.c | |
| guid-powered | IN-PROGRESS | | design note written |
| guid-landing | IN-PROGRESS | src/lm/ | translating |
| guid-entry   | IN-PROGRESS | src/csm/*entry*, reentry.c, p61_p67.c, cm_body_attitude.c, entry_dap.c, p37.c, p11.c | see docs/modules/guid-entry.md |
| dap-lm       | IN-PROGRESS | src/lm/dap/ | see docs/modules/dap-lm.md |
| dap-csm      | IN-PROGRESS | src/csm/dap/, include/agc/agc_csm_dap.h | see docs/modules/dap-csm.md |
| prog-rendz   | IN-PROGRESS | include/agc/agc_programs.h, src/programs/ | see docs/modules/prog-rendz.md |
| prog-align   | TODO | | |
| sys-misc     | IN-PROGRESS | src/io/rtb_ops.c | see docs/modules/sys-misc.md |
| oracle       | DONE (0 stubs) | tools/oracle/{agc_probe.c,Makefile,README.md,BUILD.md,bin,ropes,cases,yaAGC-src}, docs/modules/oracle.md | yaYUL+yaAGC built; ropes from virtualagc copies (Apollo-11 repo sources do not assemble); SPSIN and interpretive SQRT/SIN/COS cases proven |
| integrate    | TODO | | |
