# MODULE_STATUS.md -- one line per agent, updated by that agent

Legend: TODO / IN-PROGRESS / DONE (n stubs) / BLOCKED (reason)

| Agent id     | Status | Files produced | Notes |
|--------------|--------|----------------|-------|
| core-numeric | DONE (0 stubs) | src/core/agc_{mpac,divide,shift,word,sp_trig,constants,interp_scalar,interp_trig,interp_vector,vec_div}.c, agc_mpac.h; include/agc/agc_constants.h; agc_interp.h prototypes/scalings; tests/test_core_numeric.c, tests/oracle_core_numeric.c, tools/oracle/cases/core_numeric.{spec,txt} | bit-exact vs yaAGC (20.7k interp cases, exhaustive SPSIN/SPCOS); DAD/SL wrap like AGC (no saturation); yaAGC DV -0 bug found; see docs/modules/core-numeric.md |
| exec-sched   | DONE (V37/CANV37 partial; 5 restart erasables + 4 rupt handlers pending owners) | src/exec/*.c, exec_internal.h, exec_pending.h; agc_exec.h/agc_jobs.h additions; tests/test_exec.c (197 checks pass) | see docs/modules/exec-sched.md |
| state-flags  | DONE (0 stubs) | include/agc/agc_flags_{common,csm,lm}.h, agc_channels.h, agc_state.h; src/core/agc_state.c, agc_channels.c, agc_erasable.c; tests/test_state.c, test_state_lm.c | CSM channel bits derived from code comments (no CM bit table in source); see docs/modules/state-flags.md |
| nav-conic    | DONE (1 stub) | include/agc/agc_nav.h (conic), src/nav/conic_*.c, tests/test_conic.c, tests/oracle_conic.c, tools/oracle/cases/nav_conic{,_lm}.{spec,txt} | stub: LEMCONIC+SURFFLAG (USEPIOS, nav-integ); 35 oracle cases match yaAGC within 1 SP LSB (scratch core) |
| nav-integ    | DONE (0 stubs) | include/agc/agc_integ.h; src/nav/{integration_init,integration_jobs,orbital_integration,integration_csm,integration_lm}.c, integ_internal.h, integ_pending.h; tests/test_integration.c | Encke+W+lunar harmonics+third bodies validated vs double reference (<50 m); needs agc_align_svdwn moved to nav (layering), TRKMKCNT accessor, 4 job/task ids; see docs/modules/nav-integ.md |
| nav-aux      | DONE (0 stubs) | include/agc/agc_navaux.h; src/nav/{latlong,planetary,measurement,free_fall,ephemerides,star_tables,navaux_util}.c, navaux_internal.h; tests/test_navaux.c | agc_lsposition ready for nav-integ; pad-load accessors requested; see docs/modules/nav-aux.md |
| io-pinball   | IN-PROGRESS | include/agc/agc_dsky.h, src/io/pinball_*.c, src/io/display_if.c | see docs/modules/io-pinball.md |
| io-nouns | DONE (0 stubs; 182 data ids UNIMPLEMENTED pending core accessors) | include/agc/agc_nouns.h, src/io/noun_tables_{csm,lm}.c, noun_scales.c, noun_data.c, tests/test_nouns.c | see docs/modules/io-nouns.md |
| imu          | DONE (13 stubs: P06, POSTAND, 11 CM prelaunch) | src/imu, src/csm/prelaunch_align.c, include/agc/agc_imu.h | tests/test_imu.c 65 checks pass |
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
