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
| 1    | state-flags   | ERASABLE_ASSIGNMENTS (both), FLAGWORD_ASSIGNMENTS (LM), CONTROLLED_CONSTANTS, INPUT_OUTPUT_CHANNEL_BIT_DESCRIPTIONS | include/agc/agc_flags_*.h, src/core/agc_state.c, include/agc/agc_channels.h |
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
