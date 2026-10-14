# PLAN.md -- management plan for the Apollo 11 -> C transposition

Owner: lead session. Subagents: one per module group, see table.
This file is the single place where the plan lives. Keep context out of chat.

## Strategy

The AGC code is ~130k lines across two programs that share ~45 files
(CHIEFTAN section: interpreter, executive, waitlist, conic, integration,
alignment, pinball, etc.). We translate:

1. **Foundation first** (wave 1): numeric model, scheduler model, shared
   erasable/flag state, constants. Everything else codes against the headers
   in `include/agc/`, which the lead wrote up front and subagents may NOT edit
   (they request additions in their module note instead).
2. **Math and navigation** (wave 1, parallel): conic subroutines, orbital
   integration, nav auxiliaries. These need only the core header.
3. **Vehicle systems** (wave 2): IMU, DSKY/pinball, DAPs, guidance, programs.
4. **Integration** (wave 3): build everything, fix warnings, write a host
   simulation harness (`src/sim/`) that drives executive/waitlist from a
   software clock, run tests, run rule checker, review.

## Waves and assignments

| Wave | Agent id      | Scope (AGC files)                                             | C target                 |
|------|---------------|---------------------------------------------------------------|--------------------------|
| 1    | core-numeric  | INTERPRETER (arith parts), SINGLE_PRECISION_SUBROUTINES, FIXED_FIXED_CONSTANT_POOL, INTERPRETIVE_CONSTANTS | src/core/ |
| 1    | exec-sched    | EXECUTIVE, WAITLIST, PHASE_TABLE_MAINTENANCE, RESTARTS_ROUTINE, RESTART_TABLES, ALARM_AND_ABORT, INTERRUPT_LEAD_INS, FRESH_START_AND_RESTART | src/exec/ |
| 1    | state-flags   | ERASABLE_ASSIGNMENTS (both), FLAGWORD_ASSIGNMENTS (LM), CONTROLLED_CONSTANTS, INPUT_OUTPUT_CHANNEL_BIT_DESCRIPTIONS | include/agc/agc_flags_*.h, src/core/state.c, include/agc/core/channels.h |
| 1    | nav-conic     | CONIC_SUBROUTINES                                             | src/nav/conic.c          |
| 1    | nav-integ     | INTEGRATION_INITIALIZATION, ORBITAL_INTEGRATION               | src/nav/integration.c    |
| 1    | nav-aux       | LATITUDE_LONGITUDE_SUBROUTINES, PLANETARY_INERTIAL_ORIENTATION, MEASUREMENT_INCORPORATION, TIME_OF_FREE_FALL, LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES, STAR_TABLES | src/nav/ |
| 1    | io-pinball    | PINBALL_GAME_BUTTONS_AND_LIGHTS, PINBALL_NOUN_TABLES, DISPLAY_INTERFACE_ROUTINES, KEYRUPT_UPRUPT | src/io/ |
| 2    | imu           | IMU_MODE_SWITCHING_ROUTINES, IMU_COMPENSATION_PACKAGE, INFLIGHT_ALIGNMENT_ROUTINES, GIMBAL_LOCK_AVOIDANCE, KALCMANU_STEERING, ANGLFIND / ATTITUDE_MANEUVER_ROUTINE, R60_62, IMU_CALIBRATION_AND_ALIGNMENT | src/imu/ |
| 2    | guid-powered  | POWERED_FLIGHT_SUBROUTINES, P40-P47 (both), BURN_BABY_BURN, SERVICER (LM), SERVICER207 (CM), P30-P37 | src/guidance/ |
| 2    | guid-landing  | LUNAR_LANDING_GUIDANCE_EQUATIONS, THE_LUNAR_LANDING, THROTTLE_CONTROL_ROUTINES, P70-P71, ASCENT_GUIDANCE, P12, LANDING_ANALOG_DISPLAYS, FINDCDUW | src/lm/ |
| 2    | guid-entry    | ENTRY_LEXICON, REENTRY_CONTROL, P61-P67, CM_BODY_ATTITUDE, CM_ENTRY_DIGITAL_AUTOPILOT, P37_P70, P11 | src/csm/ |
| 2    | dap-lm        | DAPIDLER_PROGRAM, P-AXIS_RCS_AUTOPILOT, Q_R-AXIS_RCS_AUTOPILOT, TJET_LAW, KALMAN_FILTER, TRIM_GIMBAL_CONTROL_SYSTEM, AOSTASK_AND_AOSJOB, DAP_INTERFACE_SUBROUTINES, T6-RUPT_PROGRAMS, RCS_FAILURE_MONITOR, SPS_BACK-UP_RCS_CONTROL | src/lm/dap/ |
| 2    | dap-csm       | RCS-CSM_DIGITAL_AUTOPILOT, JET_SELECTION_LOGIC, AUTOMATIC_MANEUVERS, RCS-CSM_DAP_EXECUTIVE_PROGRAMS, TVC* (7 files), MYSUBS | src/csm/dap/ |
| 2    | prog-rendz    | P20-P25, P32-P33/P72-P73, P34-35/P74-75, R30, R31, STABLE_ORBIT, TPI_SEARCH, P76, GROUND_TRACKING_DETERMINATION_PROGRAM, GENERAL_LAMBERT_AIMPOINT_GUIDANCE, RADAR_LEADIN_ROUTINES | src/programs/ |
| 2    | prog-align    | P51-P53 (both), SXTMARK, AOTMARK, R63, CSM_GEOMETRY, LEM_GEOMETRY, S-BAND_ANTENNA (both), LUNAR_LANDMARK_SELECTION, AGS_INITIALIZATION | src/programs/ |
| 2    | sys-misc      | T4RUPT_PROGRAM, EXTENDED_VERBS, DOWNLINK_LISTS, DOWN_TELEMETRY_PROGRAM, UPDATE_PROGRAM, SERVICE_ROUTINES, AGC_BLOCK_TWO_SELF_CHECK, SYSTEM_TEST_STANDARD_LEAD_INS, IMU_PERFORMANCE_TESTS, INTER-BANK_COMMUNICATION, RTB_OP_CODES | src/io/, src/csm/, src/lm/ |
| 3    | integrate     | build all, fix -Werror, sim harness, tests, rule checker, review | src/sim/, tests/ |

## Rules for subagents (summary; full text in docs/agents/brief.md)

* Read docs/agents/brief.md, docs/TRANSLATION_GUIDE.md, docs/CODING_STANDARD.md
  and the headers in include/agc/ before translating.
* Write code only in your assigned target directory, plus your module note
  `docs/modules/<agent-id>.md` and your row in docs/MODULE_STATUS.md.
* Do not edit include/agc/*.h. Request additions in your note under
  "Requested core additions".
* Final report back to lead: <= 15 lines. Everything else goes in the note.

## Test oracle: Virtual AGC (added by user request)

https://github.com/virtualagc/virtualagc provides yaYUL (assembler) and yaAGC
(cycle-accurate emulator). Agent `oracle` (wave 1) builds both from the clone
at SRC/virtualagc, assembles Comanche055 and Luminary099 from the Apollo-11
repo, and produces `tools/oracle/`: a driver that loads a rope, pokes
erasable inputs, runs the emulator to a label, and dumps erasable outputs
as JSON. `tests/oracle_*.c` compare the C translation's outputs with those
dumps (differential tests). Each module agent lists in its note the entry
labels, input erasables and output erasables an oracle test needs.

## Model tiering (user direction, 2026-10-09)

Subagents must not default to the lead's model. Assign per task:

| Tier        | Model  | Use for                                                        |
|-------------|--------|----------------------------------------------------------------|
| Lead        | Fable  | plan, API contract, integration decisions, numeric review       |
| Translation | Opus   | wave 2 module translation (reads large assembly, keeps scaling) |
| Mechanical  | Sonnet | table transcription, flag enums, rule-checker fixes, test scaffolds, doc tidy |
| Trivial     | Haiku  | status-board edits, formatting                                  |

Wave 1 ran on Fable (launched before this rule). Not restarted: sunk cost.
Where a wave-2 scope is mostly tables (noun tables, restart tables, channel
bits), split that part off to a Sonnet agent.

## Definition of done (2026-10-10)

1. Coverage: every AGC file in both ropes maps to C in MODULE_STATUS; every
   routine translated or a named, justified stub. No unlisted omissions.
2. Builds clean: `make` builds libagc.a + sim with the full -Werror set; one
   flat job/task registry; no *_pending.h / ext.h / glue / shim / adapter
   files anywhere (callers use the owning subsystem's public header; where
   two lanes translated the same AGC entry from both sides, one copy
   remains, in the owning module); no duplicates; layering kept.
3. Rules pass: `make check` zero hard violations AND assertion-density
   warnings cleared; `make format-check` and `make style` clean (NASA
   SEL-94-003 C Style Guide via .clang-format + tools/check_style.py).
4. Naming: docs/NAMING.md applied everywhere; docs/history/RENAMES.md fully done.
5. Tests: `make test` green through the real library (no scratch stand-ins).
6. Oracle: every numeric module has committed yaAGC cases passing within a
   documented tolerance; bit-exact where -0 quirks allow.
7. Runs end to end UNDER THE VIRTUAL AGC GUI (user's stated end goal,
   2026-10-11): build/agc_sim is a drop-in replacement for yaAGC. The stock
   VirtualAGC launcher (SRC/virtualagc, its Apollo 11 CM and LM
   configurations) starts our binary in place of yaAGC with yaAGC's command
   line (`--core=<rope>` selects the vehicle, `--port`, `--cfg`, `--resume`
   accepted or ignored as documented) and connects the unmodified yaDSKY2,
   yaTelemetry and other peripherals over the socket protocol. Acceptance:
   from the real DSKY window, fresh start, V35 lamp test, V16N36 clock,
   CSM P52 realign with simulated marks, CSM P40 burn, LM P63 braking start,
   each producing the AGC's expected displays; headless scripted scenarios
   (sim/sim_client.py) cover the same sequences for `make test`. Fresh-
   starts either vehicle and completes at least one real program sequence
   per vehicle
   (e.g. P52 realign with simulated marks; P40 with a simulated burn) with
   the AGC's expected displays.
8. Reviewed: final fidelity/clarity review of every lane vs its AGC source,
   findings fixed.
9. Documented (ONLY after 7 and 8 -- documenting unfinished code describes
   intentions, not the program): generated from the final code, not written
   ahead of it.
   a. docs/overview/ -- system overview: the two vehicles, the executive/
      waitlist model, interrupt and clock model, erasable/flag layout,
      the DSKY loop, how a program (Pxx) is started, runs, displays, and
      ends; a layered module map with dependencies (from the real
      #include/call graph, e.g. via clangd/ctags, not hand-drawn).
   b. Flowcharts: one per major program sequence and per scheduler path
      (fresh start, restart, job pre-emption, waitlist tick, V37 change,
      P40 burn, P52 realign, P63-P66 landing, P61-P67 entry), as Mermaid
      in Markdown, derived from the C control flow.
   c. docs/modules/<lane>.md gains a "Program flow and decisions" section:
      the decision points (flags tested, alarms raised, branches taken),
      the data each routine reads/writes with scale, and the AGC labels
      cross-referenced to the C functions.
   d. A cross-reference index AGC label -> C function -> file, generated.
   e. docs/TIPS.md consolidated from every lane's "Lessons" section: the
      tips and tricks for translating AGC into C, organised by topic.

Waves: 1-2 translation (done/in progress) -> 2a style (NASA SEL-94-003:
STYLE.md, .clang-format, check_style.py; then `make format` tree-wide)
-> 2b status-handling (rule 7, whole tree) -> 3 integrate + sim (items 2-5,7)
-> 4 oracle cases for guidance/DAP/entry/landing (item 6) -> 5 review (8)
-> 6 documentation (9).

## Session limits (user direction, 2026-10-10)

Usage limits are expected and irrelevant to planning. Keep full parallelism,
relaunch every interrupted lane on reset with resume semantics, never reduce
lane count to avoid limits. The run continues until the definition of done.

## Wave assignments (docs/agents/)

| Wave | Assignment file          | Status (2026-10-10)                 |
|------|--------------------------|-------------------------------------|
| 1-2  | <lane>.md (19 lanes)     | DONE                                |
| 2a   | style.md, style2.md      | DONE (Uncrustify is the formatter)  |
| 2b   | status-handling.md       | DONE (0 hard violations)            |
| 3    | integrate.md             | DONE                                |
| 3a   | collapse.md              | DONE (3 passes)                     |
| 3b   | sim.md                   | RUNNING (Virtual AGC peripherals)   |
| 4    | oracle-cases.md          | DONE (15 findings -> review)        |
| 5    | review.md (one per lane) | RUNNING (16 agents)                 |
| 6    | document.md              | queued last                         |

## Concurrency (user direction, 2026-10-11)

Run at most 2 agents at a time (reduced from 3 on 2026-10-11). Wide parallel waves kept hitting usage
limits mid-work and the resume overhead was slowing progress. Queue order
for the remaining review lanes (fidelity-bearing first, style-debt last):

0. format2 (comment relocation + fixed continuation indent) -- BEFORE any
   further lane work (user, 2026-10-11)
0b. dispatch DONE
0c. format3 (join partially-wrapped brace initializers) -- DONE (core pending make format)
1. review-io-dsky      DONE
2. review-guid-entry   (F14 fix in progress)
3. review-nav-aux      (INCORP1 fixes in progress)
4. review-prog-rendz   (F13 CDH 62 m)
5. review-guid-landing (F9 THROTTLE)
6. review-dap-lm       DONE
7. review-dap-csm      DONE
8. review-nav-conic    DONE (exact at tolerance 0)
9. review-nav-integ    (PAUSED: fidelity done, 439 oracle checks; DMP audit + style left)
10. review-core-numeric DONE (word-level API landed)
11. review-exec-sched   (prologs in progress)
12. review-imu          (gyrocompass file in progress)
13. review-prog-align   (oracle test additions in progress)
14. review-sys-misc     (t4rupt_csm.c rewrite in progress)
15. review-guid-powered (ZOOM; style debt; prologs in progress)
16. sim  -- runs AFTER all reviews and follow-ups (user, 2026-10-11): what
    remains is acceptance work (yaAGC command-line drop-in, GUI runs) and
    is done once, on settled flight code. It is still the end-goal
    deliverable (VirtualAGC GUI on our C backend).
17. follow-up-done-lanes (nav-aux, io-dsky, state-flags, guid-entry, imu,
    guid-landing, prog-rendz, dap-lm: DMP/DMPR audit where missed, and
    word-level API adoption per review_items.md)
Then: lead assert.h rename; ONE Frama-C EVA run (`make prove`, `make
    prove-classify`) on the frozen tree -- cleanup, not a working-code step
    (user, 2026-10-11). The sim entry is the real unbounded agc_sim main,
    run ALONE with NO timeout (EVA converges by widening; memory, not time,
    is the only limit: ~7 GB at 60 min on a 15 GB host). Apply proven guard
    deletions everywhere, triage any alarms; then the document wave.

## Status against the definition of done (lead, 2026-10-13)

1. Coverage: DONE. 19 lanes, every AGC file of both ropes in MODULE_STATUS;
   the remaining stubs are named and justified there (LM P07 fixed-memory
   part translated; the erasable program at ATIGINC absent from the rope).
2. Builds clean: DONE. -Werror set, one flat registry (4 generated
   switches), no pending/glue/shim files; layering restored 2026-10-13
   (depgraph upward-call table empty; address map in src/erasable_map).
3. Rules pass: DONE except two files. check-rules 0 hard, format-check
   clean, rule 5 by its spirit (no density quota; checkers reject the
   dodges). `make style`: 0 findings outside src/erasable_map/
   address_map.c and src/nassp/engine.c (236 there, the NASSP
   lane's in-progress files; on its closing list).
4. Naming: DONE (NAMING.md applied, RENAMES.md ledger complete, no
   program/verb/noun numbers in identifiers or file names).
5. Tests: DONE. make test = 35 binaries + 14 scenarios through the real
   library, no stand-ins.
6. Oracle: DONE. ~10k checks vs yaAGC; conic, DAP CSM/LM, DSKY, alignment,
   nav-aux, landing ascent, entry, targeting exact at tolerance 0 where -0
   allows; remaining tolerances documented in docs/oracle-cases.md.
7. Runs under the Virtual AGC GUI: DONE for the CSM and the LM DSKY
   sequences (fresh start, V35, V16N36, CSM P52, CSM P30/P40 burn) with
   the stock launcher, yaDSKY2 and yaTelemetry; headless twins in make
   scenarios. LM P63 braking was NOT run there: it is a closed loop that
   needs a vehicle (no device models by principle); it is covered instead
   by the LM Simulator loop (make lm-sim, 34/34 vs yaAGC: ISS, align, DAP)
   and will be flown in NASSP's PDI milestone (item 10).
8. Reviewed: DONE. 16 lane reviews + verify-collapse, follow-up,
   loose-ends, prolog-consistency, always-ok, closed-loop-fixes; every
   routed finding closed (docs/agents/review_items.md).
9. Documented: DONE (47ae168): overview + generated module map, 10
   flowcharts, per-lane program-flow sections, xref (3,071 labels), TIPS.
10. Stretch, DONE (2026-10-14): LM Simulator closed loop (make lm-sim,
    34/34 vs yaAGC); the Apollo 11 mission in NASSP 9 on our AGC, all ten
    milestones (insertion + uplink, TLI, MCC-2, LOI, DOI, PDI/landing,
    liftoff, rendezvous, TEI, entry/splashdown) matching stock on
    decisions and end state, headless and unattended, timing-born
    differences listed as expected deviations; 17 flight bugs found and
    fixed with failing-first tests; `make nassp-replay` (offline images)
    and `make nassp-mission` (generated tables); mission report in
    docs/modules/nassp-integration.md.

## Stretch (not yet in the definition of done; user to decide)

10. Full-mission closed loop (user, 2026-10-12: "a good final test would
    be to run the entire Apollo 11 mission in the CSM and LM using our
    backend"; everything runs from this Linux/WSL host). Phases:
    a. LM Simulator (Virtual AGC Contributed/LM_Simulator, Tcl/Tk): the
       one closed-loop flight test available with an existing proven tool
       on Linux; LM only; compared side by side with yaAGC on the same
       sequence (docs/history/assignments/lm-simulator.md).
    b. NASSP integration study (docs/history/assignments/nassp-study.md): NASSP embeds
       yaAGC's engine and reads/writes erasable directly; inventory of
       its AGC interface, the erasable map onto our typed storage, the
       shim design, the build/run plan. NASSP/Orbiter are Windows
       programs: the build and run go through WSL interop to the Windows
       host (powershell.exe, winget, MSBuild) if interop is enabled, else
       the study stands as the documented next step.
    c. If (b) is feasible from here: build NASSP with libagc behind the
       engine shim on the host, run Apollo 11 with NASSP's MCC/checklist
       automation, script what can be scripted, verify milestones against
       the stock NASSP run. Expected to be multi-day; the erasable map and
       mission scripting are the risks.
    This is the ONLY closed-loop route: we do not build a world model.

## Principle: functional fidelity, not timing emulation (user, 2026-10-14)

The aim is a readable AGC in C that does what the original did, so a
reader can see what a guidance computer looks like. Instruction timing
is a property of the 1960s machine, not of the guidance logic: no timing
model. The zero-time C executive stays; the two ordering deviations
(landing-radar read wait, self-rerunning job pacing) stand in for AGC
execution time and are documented. Behaviours that exist only because
of execution time (1201/1202 overload alarms, 01703 slip timing, how long
a manoeuvre computation takes) are expected deviations, listed with
their traces, not chased. yaAGC remains the tool for timing-exact runs.

## Principle: build only the AGC software (user, 2026-10-11)

This project rewrites the AGC flight software in C. Everything around it is
an existing, proven tool we plug into: yaYUL/yaAGC (assembly and the
numeric oracle), the VirtualAGC GUI and peripherals (yaDSKY2, yaTelemetry,
yaACA, LM_Simulator, piPeripheral) for interactive runs, NASSP for flight.
No lane writes a simulator, device model, DSKY, or physics of its own; the
sim harness is limited to the clock/interrupt driver and the yaAGC
peripheral protocol, i.e. what makes agc_sim a drop-in for yaAGC.
