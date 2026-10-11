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

## Rules for subagents (summary; full text in docs/AGENT_BRIEF.md)

* Read docs/AGENT_BRIEF.md, docs/TRANSLATION_GUIDE.md, docs/CODING_STANDARD.md
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
4. Naming: docs/NAMING.md applied everywhere; docs/RENAMES.md fully done.
5. Tests: `make test` green through the real library (no scratch stand-ins).
6. Oracle: every numeric module has committed yaAGC cases passing within a
   documented tolerance; bit-exact where -0 quirks allow.
7. Runs end to end: build/agc_sim (our clock/interrupt driver + yaAGC's
   peripheral socket protocol, so Virtual AGC's yaDSKY2 and other
   peripherals are the I/O) fresh-starts either vehicle and completes at
   least one real program sequence per vehicle
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

Run at most 3 agents at a time. Wide parallel waves kept hitting usage
limits mid-work and the resume overhead was slowing progress. Queue order
for the remaining review lanes (fidelity-bearing first, style-debt last):

0. format2 (comment relocation + fixed continuation indent) -- BEFORE any
   further lane work (user, 2026-10-11)
0b. dispatch (4 flat generated switches, rule-4 exempt) -- right after format2
1. review-io-dsky      DONE
2. review-guid-entry   (F14 fix in progress)
3. review-nav-aux      (INCORP1 fixes in progress)
4. review-prog-rendz   (F13 CDH 62 m)
5. review-guid-landing (F9 THROTTLE)
6. review-dap-lm       (PAUSED mid-audit: ~16 sat->wrap stores, style; resume from note)
7. review-dap-csm      (ROLLSET; digit-suffixed names)
8. review-nav-conic    (PAUSED: R1A word-pair plan in note; needs core word API first -> run review-core-numeric BEFORE resuming)
9. review-nav-integ    (spec generator in progress)
10. review-core-numeric (UNIT B27; stale DVSIGN; test prologs in progress)
11. review-exec-sched   (prologs in progress)
12. review-imu          (gyrocompass file in progress)
13. review-prog-align   (oracle test additions in progress)
14. review-sys-misc     (t4rupt_csm.c rewrite in progress)
15. review-guid-powered (ZOOM; style debt; prologs in progress)
16. sim                 (pad.c; scenarios)
17. dmp-audit-done-lanes (nav-aux, io-dsky, state-flags: the DMP/DMPR +
    shift-rounding audit from review_items.md, since they finished before it)
Then: lead assert.h rename; document wave (1 agent).
