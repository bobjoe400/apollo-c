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

File names follow the stem: `return_to_earth.c`, `entry_programs.c` (for
a file holding P61-P67 together), `sps_burn.c`, `landing_programs.c`.
Job ids: `AGC_JOB_RETURN_TO_EARTH`, not `AGC_JOB_P37`. Where the AGC label
itself is a number-free word (KEPLER, LAMBERT, SERVICER, CALCGTA) keep it.
