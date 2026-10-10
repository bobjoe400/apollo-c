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
