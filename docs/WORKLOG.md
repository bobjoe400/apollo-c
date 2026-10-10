# WORKLOG.md -- lead's running log (newest at bottom)

2026-10-09  Cloned Apollo-11 (Comanche055 86 files / 65k lines, Luminary099
            91 files / 65k lines), NASA Power-of-Ten PDF, virtualagc.
            Wrote docs (PLAN, CODING_STANDARD, TRANSLATION_GUIDE, AGENT_BRIEF,
            ARCHITECTURE, MODULE_STATUS), include/agc contract headers,
            Makefile, tools/check_rules.py. Committed skeleton.
2026-10-09  Wave 1 launched: core-numeric, exec-sched, state-flags, oracle,
            nav-conic, nav-integ, nav-aux, io-pinball (8 agents, parallel).
            Wave 2 waits for state-flags + exec-sched (flag/channel/job ids).
            Wave 3 (integrate) after wave 2.

## Decisions
- Numeric model: fixed-point int16/int32/int64 mirroring SP/DP/TP; no doubles
  in flight code. Interpreter is a library, not an emulator.
- No function pointers: job/task dispatch by enum + switch (rule 9).
- Oracle: Virtual AGC yaAGC runs the real ropes; differential tests compare.
2026-10-09  exec-sched agent hit an API safeguard false positive on launch;
            relaunched with reworded prompt. User directed model tiering
            (Opus for translation, Sonnet for mechanical work); recorded in
            PLAN.md; applies from wave 2.
2026-10-09  Stopped the 7 Fable translation agents (~5 min in, still reading;
            negligible loss). Moved assignments to docs/agents/*.md; relaunched
            on tiers: core-numeric, exec-sched, nav-conic, nav-integ, nav-aux,
            io-pinball on Opus; state-flags and new io-nouns (noun tables split
            out of pinball) on Sonnet. oracle (Fable) left running: ropes
            already assembled.
2026-10-09  Wave 2 launched early (user: no reason to hold at 8). Pinned the
            naming convention for not-yet-existing flags/channels/accessors in
            docs/agents/_common.md so lanes can code ahead of state-flags.
            Running now: 17 agents (8 wave 1 + 9 wave 2: imu, guid-powered,
            guid-landing, guid-entry, dap-lm, dap-csm, prog-rendz, prog-align,
            sys-misc; all Opus). Wave 3 (integrate) after these report.
2026-10-09  oracle DONE. yaYUL/yaAGC built (tools/oracle/BUILD.md). The
            Apollo-11 repo sources do NOT assemble with current yaYUL; ropes
            built from virtualagc's Comanche055/Luminary099 copies, verified
            0 mismatches vs official binsource. agc_probe proven on SPSIN and
            interpretive SQRT/SIN via the VAC-snippet technique. Binaries and
            11.8 MB symtabs are gitignored (rebuild per BUILD.md).
2026-10-09  Usage limit killed 14 Opus lanes mid-work; io-nouns and
            state-flags finished clean first. Committed checkpoint (16.6k
            lines). All lanes have notes with Progress sections. Relaunching
            with resume semantics in two batches: A = core-numeric, exec-sched,
            nav-conic, nav-integ, nav-aux, io-pinball, imu; B = the rest after A.
2026-10-09  nav-aux DONE (0 stubs). Integration items: TEPHEM has two sources
            (shared accessor vs module pad-load struct) until core adds the
            pad-load accessors listed in its note; nav-integ owns INCORP2's
            restart/RECTIFY/INTWAKE; agc_dp_sign must be AGC SIGN semantics.
            Launching batch B lane guid-powered now (one at a time as A frees).
2026-10-09  exec-sched DONE (V37 partial). 197 checks pass. Integration items:
            deferred software restart (ENEMA runs at job/interrupt end, not
            inline, to avoid recursion); 44 provisional job ids + 51 task ids in
            src/exec/exec_pending.h (0x0E00 range) must move into agc_jobs.h and
            dispatch.c; duplicate AGC_JOB_GOTOPOOH in src/csm/entry_pending.h
            and src/programs/rendz_ext.h; AGC_JOB_GOPROG placeholder to remove;
            restart-table delays need core accessors; up-calls SERVIDLE,
            SETCOARS, SETMAXDB/SETMINDB; interrupt handlers T4RUPT/UPRUPT/
            DOWNRUPT/RADAR pending from sys-misc/io lanes.
            Launching batch B lane guid-landing.
2026-10-09  imu DONE (13 stubs: P06/POSTAND + 11 Comanche prelaunch entries).
            65 checks pass. Integration items: provisional ids in
            src/imu/imu_pending.h; MODECADR moved to exec storage (-0 stored
            as -2); prog-align's align_pending.h and guid-landing's
            landing_pending.h declare agc_imu_* with mismatched signatures ->
            adopt agc_imu.h; CALCSMSC duplicated in imu and prog-align (drop
            one); BUG exec-sched executive.c activity_light() passes 0177775 to
            agc_chan_and (mask > 077777 assertion); sim must drain GYROCMD /
            CDU?CMD via agc_imu_hw_set_*. Launching batch B lane guid-entry.
2026-10-09  core-numeric DONE (0 stubs), bit-exact vs yaAGC on 20,720
            interpreter runs and all 65,534 SPSIN/SPCOS inputs; 287 oracle
            cases committed with tests/oracle_core_numeric.c. Translation guide
            corrected (wrap-not-saturate; DMP/DMPR mapping). Found a yaAGC DV
            bug (SpToDecent sign-extension) -> patch note for oracle owner.
            Known deviation: stale DVSIGN on V/SC & UNIT overflow not modelled.
            Launching batch B lane dap-lm.
2026-10-09  nav-integ DONE (0 stubs). 177 checks; LEO J2-J4 10 min within 46 m,
            lunar J2/J3/J22/C31 1 h within 19 m, third-body 6 h within 42 m.
            Integration items: ids STATINT1/STATINT1_STALL/STATEUP_STALL/
            TASK_STATEINT (integ_pending.h); layering violation ATOPCSM/ATOPLEM
            call agc_align_svdwn() in src/programs -> move SVDWN to nav;
            TRKMKCNT accessor needed; agc_state.h comment wrong: RN/VN always
            2^29 m / 2^7 m/cs, not moon-scaled. Launching batch B lane dap-csm.
