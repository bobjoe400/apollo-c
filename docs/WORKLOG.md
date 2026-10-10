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
