# NAMING.md -- no program/routine/page numbers in identifiers

Rule (user direction, 2026-10-10): AGC program numbers (P30, P63...),
routine numbers (R30, R60...), verb/noun numbers (V37, N36) and page
numbers appear ONLY in comments. Every identifier -- file name, function,
type, enum member, variable, test name -- uses a descriptive name taken
from the AGC's own description of the program/routine. The number is kept
in the function header comment, e.g.

    /* P37 RETURN TO EARTH (P37_P70.agc pp. 890-933, label P37) */
    agc_status_t agc_job_return_to_earth(agc_job_ctx_t *ctx);

Noun/verb TABLES stay indexed by number (that is data, not a name), but
handler functions are named by what they do (`agc_verb_change_program`,
not `agc_verb37`).


## Rules for deriving a name (added after the first rename attempt failed)

The first pass did mechanical substitution and produced names like
`entry_entry_upcontrol`, `agc_entry_entry_final_phase_2_latlong` and
`AGC_JOB_ENGINE_FAIL_PROCEDURE_E`. Those are not names. Every rename must
satisfy ALL of the following:

1. **Read the source first.** For every identifier that carries a label
   (P65_1B, V97E, CLOKJOB, N61, R51K, P63SPOT2), open the AGC file, find
   the label, and read its comment and the code around it. The new name
   states what that code DOES, in the AGC's own words where possible.
   Record the AGC comment you used in the map so a reviewer can check it.
2. **No opaque suffixes.** Letters or digits inherited from the label
   (`_E`, `_P`, `_T`, `_1B`, `_2`, `TASK2`, `EJOB`) are forbidden unless the
   source gives them a meaning you then spell out: V97E/V97P/V97T are the
   engine-fail variants for different engine states -> name those states.
   Sequential job segments become `_start`, `_after_display`, `_resume`,
   `_second_pass`, chosen from what the segment does, never `_1`/`_1b`.
   A label fragment that repeats the prefix (`AGC_TASK_..._TTASK`,
   `AGC_JOB_..._EJOB`) is a sure sign the source was not read.
3. **No stutter.** If the prefix already says `entry`, the stem does not
   repeat it: `agc_entry_final_phase_latlong`, not
   `agc_entry_entry_final_phase_2_latlong`. Collapse repeated words.
4. **English phrase, 2-4 words after the prefix.** `agc_job_` + phrase,
   `AGC_TASK_` + PHRASE. If you need more than 4 words the name is
   describing two things; split or pick the dominant one.
5. **Keep word labels.** Labels that are already words (KEPLER, LAMBERT,
   SERVICER, CALCGTA, LANDISP, CLOKJOB, BURNBABY) stay, lower-cased.
   Do not "improve" them.
6. **Verbs by function, nouns by content.** V97 -> `engine_fail_procedure`,
   V99 -> `engine_enable_request`, N61 -> `landing_site_latlong` (read the
   noun table to see what N61 holds). Tables indexed by verb/noun number
   are data and keep numeric indices.
7. **One name per concept, tree-wide.** Before inventing a name, grep the
   tree and docs/NAMING.md; reuse an existing stem. The job id, the job
   function, the file and the test share the stem:
   `AGC_JOB_ENTRY_UPCONTROL`, `agc_job_entry_upcontrol`, `entry_programs.c`.
8. **Map first, apply second.** Write docs/RENAMES.md completely (old |
   new | file | AGC comment used), have it reviewed, then apply with
   word-boundary replacement in one pass over the whole tree.

## Canonical names (use these exactly; extend the table, never invent a
## second name for the same program)

| AGC   | Vehicle | Identifier stem                    | AGC description                       |
|-------|---------|------------------------------------|---------------------------------------|
| P00   | both    | idle                               | CMC/LGC idling                        |
| P01   | CSM     | prelaunch_initialization           |                                       |
| P02   | CSM     | gyrocompassing                     |                                       |
| P03   | CSM     | optical_verification               | optical verification of gyrocompass   |
| P06   | both    | standby                            | CMC/LGC power down                    |
| P07   | both    | systems_test                       |                                       |
| P11   | CSM     | earth_orbit_insertion_monitor      |                                       |
| P12   | LM      | powered_ascent                     |                                       |
| P20   | both    | rendezvous_navigation              |                                       |
| P21   | both    | ground_track_determination         |                                       |
| P22   | both    | orbital_navigation                 | (LM: lunar surface navigation)        |
| P23   | CSM     | cislunar_midcourse_navigation      |                                       |
| P24   | CSM     | rate_aided_tracking                |                                       |
| P25   | both    | preferred_tracking_attitude        |                                       |
| P27   | both    | uplink_update                      |                                       |
| P30   | both    | external_delta_v                   |                                       |
| P31   | LM      | lambert_aimpoint_guidance          | general Lambert aimpoint              |
| P32   | both    | csi_targeting                      | coelliptic sequence initiation        |
| P33   | both    | cdh_targeting                      | constant delta height                 |
| P34   | both    | tpi_targeting                      | transfer phase initiation             |
| P35   | both    | tpm_targeting                      | transfer phase midcourse              |
| P37   | CSM     | return_to_earth                    |                                       |
| P38   | both    | stable_orbit_rendezvous            | SOR                                   |
| P39   | both    | stable_orbit_midcourse             | SOM                                   |
| P40   | CSM     | sps_burn                           | SPS thrusting                         |
| P40   | LM      | dps_burn                           | DPS thrusting                         |
| P41   | both    | rcs_burn                           | RCS thrusting                         |
| P42   | LM      | aps_burn                           | APS thrusting                         |
| P47   | both    | thrust_monitor                     |                                       |
| P51   | both    | imu_orientation_determination      |                                       |
| P52   | both    | imu_realign                        |                                       |
| P53   | both    | backup_imu_orientation             |                                       |
| P54   | both    | backup_imu_realign                 |                                       |
| P57   | LM      | lunar_surface_align                |                                       |
| P61   | CSM     | entry_preparation                  |                                       |
| P62   | CSM     | cm_sm_separation                   |                                       |
| P63   | CSM     | entry_initialization               |                                       |
| P63   | LM      | braking_phase                      | landing braking phase                 |
| P64   | CSM     | entry_post_0_05g                   | post 0.05 g                           |
| P64   | LM      | approach_phase                     | landing approach phase                |
| P65   | CSM     | entry_upcontrol                    |                                       |
| P66   | CSM     | entry_ballistic                    |                                       |
| P66   | LM      | rate_of_descent_landing            |                                       |
| P67   | CSM     | entry_final_phase                  |                                       |
| P68   | LM      | landing_confirmation               |                                       |
| P70   | LM      | dps_abort                          |                                       |
| P71   | LM      | aps_abort                          |                                       |
| P72   | LM      | csi_targeting_other_vehicle        | CSM CSI targeting from the LM         |
| P73   | LM      | cdh_targeting_other_vehicle        |                                       |
| P74   | both    | tpi_targeting_other_vehicle        |                                       |
| P75   | both    | tpm_targeting_other_vehicle        |                                       |
| P76   | both    | target_delta_v                     |                                       |
| R02   | both    | imu_status_check                   |                                       |
| R03   | both    | dap_data_load                      |                                       |
| R05   | both    | s_band_antenna                     |                                       |
| R21   | both    | rendezvous_sighting_mark           |                                       |
| R22   | both    | rendezvous_tracking_data           |                                       |
| R23   | CSM     | backup_sighting_mark               |                                       |
| R30   | both    | orbit_parameter_display            |                                       |
| R31   | both    | rendezvous_parameter_display       |                                       |
| R32   | CSM     | target_delta_v_display             | (check AGC verb list)                 |
| R34   | CSM     | rendezvous_parameter_display_2     |                                       |
| R35   | CSM     | lunar_landmark_selection           |                                       |
| R36   | both    | out_of_plane_display               |                                       |
| R40   | both    | engine_fail                        |                                       |
| R41   | both    | state_vector_integration           |                                       |
| R47   | LM      | ags_initialization                 |                                       |
| R50   | both    | coarse_align                       |                                       |
| R51   | both    | fine_align                         |                                       |
| R52   | both    | auto_optics_positioning            |                                       |
| R53   | both    | sighting_mark                      |                                       |
| R54   | both    | sighting_data_display              |                                       |
| R55   | both    | gyro_torquing                      |                                       |
| R56   | both    | alternate_los_sighting_mark        |                                       |
| R60   | both    | attitude_maneuver                  |                                       |
| R61   | both    | tracking_attitude                  |                                       |
| R62   | both    | crew_defined_maneuver              |                                       |
| R63   | LM      | rendezvous_final_attitude          |                                       |
| V37   | both    | change_program                     | (verb handler name)                   |
| 1/ACCS   | LM   | jet_accel_update                   | DAP jet accelerations / TJETLAW params (agc_lm_dap_jet_accel_update) |
| 1/ACCJOB | LM   | jet_accel_update (job)             | AGC_JOB_JET_ACCEL_UPDATE              |
| 1/ACCSET | LM   | jet_accel_reset (job)              | null offsets, then 1/ACCS: AGC_JOB_JET_ACCEL_RESET |
| 1/ACCONT | LM   | switch_curves                      | TJETLAW switch-curve parameters (lm_dap_switch_curves) |
| 1STOTWOS | LM DAP | ones_to_twos                     | ones- to twos-complement (lm_dap_ones_to_twos) |

File names follow the stem: `return_to_earth.c`, `entry_programs.c` (for
a file holding P61-P67 together), `sps_burn.c`, `landing_programs.c`.
Job ids: `AGC_JOB_RETURN_TO_EARTH`, not `AGC_JOB_P37`. Where the AGC label
itself is a number-free word (KEPLER, LAMBERT, SERVICER, CALCGTA) keep it.

### Added by dap-csm (GSOP section-numbered subroutines)

| AGC    | Vehicle | Identifier stem   | AGC description                                   |
|--------|---------|-------------------|---------------------------------------------------|
| S41.2  | CSM     | decode_dap_data   | decode DAPDATR1/2 (N46) into RCS DAP parameters   |
| S40.14 | CSM     | rcs_gains         | RCS DAP gains J/M, KMJ from IXX, IAVG             |
| S40.15 | CSM     | tvc_gains         | TVC DAP gains 1/CONACC, VARK                      |
| V68    | CSM     | stroke_test       | TVC stroke test (STRKTSTI job keeps its label)    |

### Added by naming (phase 1 map, docs/RENAMES.md)

| AGC     | Vehicle | Identifier stem              | AGC description                                         |
|---------|---------|------------------------------|---------------------------------------------------------|
| P65     | LM      | velocity_nulling             | landing phase P65 (P65START)                            |
| P67     | LM      | manual_landing               | landing phase P67 (STARTP67, P67VERT)                   |
| R10     | LM      | landing_analog_displays      | landing analog displays (LANDISP keeps its label)       |
| R11     | LM      | abort_monitor                | R10,R11 task: "IS AN ABORT (STAGE) COMMANDED?"          |
| R12     | LM      | landing_radar_lights         | R12LITES: H/V landing-radar lamps                       |
| R29     | LM      | powered_flight_designate     | RR designate during powered flight (NOR29FLG)           |
| V97     | both    | engine_fail_procedure        | NV word only; jobs/tasks use the R40 stem engine_fail   |
| V99     | both    | engine_enable_request        | NV word only; jobs/tasks use engine_enable              |
| S11.1   | CSM     | velocity_altitude_rate       | "CALC. VI, H, HDOT FOR NOUN 62"                         |
| S30.1   | both    | external_delta_v_vectors     | apogee, perigee, DELVSIN for P30                        |
| S31.1   | CSM     | lambert_aimpoint_vectors     | DELV in ref and LV coords, intercept time for P31       |
| S40.1   | both    | initial_velocity_to_gain     | "COMPUTE INITIAL THRUST DIRECTION(UT) AND ... VGTIG"    |
| S40.2,3 | both    | preferred_thrust_attitude    | POINTVSM / preferred attitude                           |
| S40.6   | CSM     | gimbal_test_trim             | "GIMBAL DRIVE TEST AND/OR GIMBAL TRIM" (PRE40.6 = gimbal_trim_restart) |
| S40.8   | both    | cross_product_steering       | VG update, TGO, FINDCDUW steering                       |
| S40.9   | both    | vtogain                      | AGC's own name "S40.9 -- VTOGAIN"                       |
| S40.13  | both    | timeburn                     | AGC's own name "S40.13 -- TIMEBURN"                     |
| S41.1   | both    | reference_to_body            | reference-to-body velocity transformation               |
| S52.2   | both    | gimbal_angles                | "COMPUTE GIMBAL ANGLES"                                 |
| S52.3   | both    | nominal_orientation          | nominal REFSMMAT orientation                            |
| SR52.1  | CSM     | target_optics_angles         | SAC/PAC for the target                                  |
| S61.1   | CSM     | entry_imu_check              | "CHECK STATE VECTOR AND IMU ORIENTATION"                |
| S61.2   | CSM     | ems_data                     | N60 / N63 EMS initialization data                       |
| S62.3   | CSM     | desired_gimbal_angles        | "COMPUTE DESIRED GIMBAL ANGLES FOR ENTRY ATTITUDE"      |
| 1/GYRO  | both    | gyro_command_pulse           | "SCALE GYRO COMMANDS FOR IMUPULSE"                      |
| 1/PIPADT| both    | pipa_interval                | PIPA interval used by drift compensation                |

Rule-4 word count counts one canonical stem as one term. A job segment that
resumes after a PHASCHNG restart point ends in `_restart`; crew responses end
in what they do (`_proceed`, `_terminate`, `_display`), never in T/P/E/B/1/1B.

### Added by sys-misc (extended verbs, LM radar)

| AGC      | Vehicle | Identifier stem          | AGC description                                    |
|----------|---------|--------------------------|----------------------------------------------------|
| R04      | LM      | radar_test               | V63 "SAMPLE RADAR ONCE PER SECOND" (R04FLAG)       |
| R77      | LM      | lr_spurious_test         | V78/V79 "LR SPURIOUS RETURN TEST"                  |
| V40 N72  | LM      | rr_zero                  | RRZEROK: zero the RR CDUs (RRZERO = agc_rr_zero)   |
| V41 N72  | LM      | rr_designate             | RRDESNBK: designate to desired gimbal angles       |
| V44      | LM      | rr_designate_end         | RRDESEND: terminate continuous designate           |
| V60      | LM      | lr_position_two          | LRPOS2K: command landing radar to position 2       |
| V85      | LM      | rr_los_display           | RR LOS azimuth and elevation (RRLOSDSP)            |
| V41 N91  | CSM     | optics_coarse            | OPTCOARK: run the optics CDUs from the keyboard    |

### Added by prog-rendz (P20-P25 radar / optics routines)

| AGC      | Vehicle | Identifier stem          | AGC description                                    |
|----------|---------|--------------------------|----------------------------------------------------|
| R23      | LM      | manual_acquisition       | R23LEM: manual acquisition of the CSM by the RR    |
| R24      | LM      | radar_search             | R24LEM / LRS24.1: RR search pattern                |
| R57      | CSM     | optics_calibration       | "OPTICS CALIBRATION ROUTINE" (trunnion bias)       |
| R65      | LM      | tracking_attitude        | R65LEM fine preferred tracking attitude (= R61 stem) |
| V67      | both    | w_matrix_monitor         | V67CALL: W-matrix RMS error display / load         |
| V94      | CSM     | cislunar_midcourse_navigation_point | V94ENTER: CMC points the vehicle in P23 |
| LSR22.3  | LM      | rendezvous_tracking_data_incorporate | LSR22.3/.4 measurement incorporation    |
| S22.1    | CSM     | orbital_navigation_marks | S22.1 landmark mark processing                     |
