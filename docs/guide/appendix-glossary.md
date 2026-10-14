# Appendix: glossary

AGC terms, labels and erasables used in this guide, with the chapter
that explains each and, where there is one, the C name. Scales are the
AGC's (chapter 2). For any label not here, see `docs/overview/xref.md`.

## Machine and numbers

| Term | Meaning | Ch. |
|---|---|---|
| word | 15 bits, ones' complement: sign + 14 magnitude bits; `agc_sp_t` | 2 |
| SP / DP / TP | single, double, triple precision: 1, 2, 3 words; `agc_sp_t`, `agc_dp_t`, `agc_tp_t` | 2 |
| -0 | negative zero, octal 77777; a value the hardware produces and the program tests | 2 |
| scale, B*n*, 2(*n*) | the power of two a stored fraction is multiplied by | 2 |
| POSMAX, DPPOSMAX | largest SP / DP fraction, 1 - 2^-14 / 1 - 2^-28 | 2 |
| HALF, BIT14, BIT13 | constants 1/2, 1/2, 1/4 (constant-pool names are bit numbers) | 2 |
| overflow-corrected | the stored word after an overflow: the sum wrapped by 1 | 2 |
| OVFIND | the interpreter's overflow indicator; `agc_interp_t.ovfind` | 2, 3 |
| word pair | a DP as its two words, signs free to disagree; `agc_dp_words_t` | 2 |
| erasable | read-write memory, 2048 words (E0-E7) | 1, 5 |
| fixed memory, rope | read-only memory, 36 banks of 1024 words | 1 |
| ECADR | erasable address, as pinball, uplink and downlink use it | 5 |
| channel | 15-bit I/O port; `agc_chan_read` / `agc_chan_write` | 1, 5 |
| counter | erasable cell the hardware increments (PIPAs, CDUs, TIME1-6, THRUST) | 1, 4 |
| CDU | the unit that digitises a gimbal or optics angle into a counter (CDUX, CDUY, CDUZ; CDUS, CDUT) | 1 |
| PIPA | the IMU accelerometers: velocity increments as counts (PIPAX..Z) | 9 |
| IMU, stable member | the inertial platform; the frame it holds | 8 |
| REFSMMAT | reference-to-stable-member matrix, half-unit rows | 3, 8 |
| DSKY | display and keyboard | 1, 6 |

## Interpreter

| Term | Meaning | Ch. |
|---|---|---|
| basic code | the machine's own instructions | 1, 3 |
| interpretive code | the vector/DP language run by INTERPRETER.agc; `include/agc/core/interp.h` | 3 |
| MPAC | multi-purpose accumulator, 3 words (6 in vector mode) | 3 |
| push-down list | the job's stack in its VAC area; C locals | 3 |
| X1, X2, S1, S2 | index and step registers | 3 |
| DMP / DMPR | DP multiply, truncated / rounded; `agc_dp_mul_trunc` / `agc_dp_mul` | 3 |
| UNIT | half-unit vector; `agc_vec_unit` | 3 |
| MXV, VXM | matrix times vector, vector times matrix (three DOTs, no shift) | 3 |
| BOV, BOVB | branch on overflow (tests and clears OVFIND); `agc_interp_test_ovf` | 2, 3 |
| SIGNMPAC | load +-DPPOSMAX with the sign of MPAC's major word | 2 |
| RTB | "return to basic": call a basic routine from interpretive code | 3 |
| STODL, STOVL, STCALL | store MPAC, then load / call; with no address they pop | 3 |

## Executive, waitlist, restarts

| Term | Meaning | Ch. |
|---|---|---|
| job | priority-scheduled work; `agc_job_<name>(ctx)`, id `AGC_JOB_*` | 4 |
| task | time-scheduled short work run in T3RUPT; `agc_task_<name>()`, id `AGC_TASK_*` | 4 |
| NOVAC / FINDVAC | start a job without / with a VAC area; `agc_exec_novac` / `agc_exec_findvac` | 4 |
| core set | a job's 12-word context; 7 (CSM) or 8 (LM) | 4 |
| VAC area | 43-word work area for an interpretive job; five | 4 |
| NEWJOB | the executive's "switch to core set k" word | 4 |
| JOBSLEEP / JOBWAKE | suspend / resume a job; `agc_exec_job_sleep` / `agc_exec_job_wake` | 4 |
| CHANG1 | offer the processor; `agc_exec_job_yield` | 4 |
| ENDOFJOB | end the job: return `AGC_STATUS_OK` | 4 |
| DUMMYJOB | the idle loop | 4 |
| WAITLIST, LONGCALL | schedule a task (up to 162.5 s / longer); `agc_waitlist_call`, `agc_waitlist_longcall` | 4 |
| TIME1-TIME6 | counter-timers; TIME3 waitlist, TIME4 T4RUPT, TIME5 DAP, TIME6 LM jet timing | 4 |
| T3RUPT, T4RUPT, T5RUPT, T6RUPT | timer interrupts | 4 |
| KEYRUPT, UPRUPT, DOWNRUPT, RADARUPT, MARKRUPT | keyboard, uplink, downlink, radar, mark interrupts | 4, 6 |
| INHINT / RELINT | inhibit / release interrupts; `agc_inhint` / `agc_relint` | 4 |
| 2CADR | two-word address of a job or task; in C an enum id | 4 |
| ALARM, BAILOUT, POODOO | record a code / software restart / restart into P00 | 7 |
| FAILREG | the three alarm-code registers (V05 N09) | 7 |
| GOJAM | hardware restart | 7 |
| ENEMA, GOPROG2, GOPROG3 | software restart and its common tail | 7 |
| PHASCHNG | record a restart point; `agc_phase_change` | 7 |
| restart group, PHASEn | one of six restartable threads and its phase | 7 |
| fresh start, DOFSTART, SLAP1 | discard running programs (V36) | 7 |
| GOTOPOOH, GOP00FIX | end a program: P00 and the V37 N99 flash | 7 |

## Crew interface

| Term | Meaning | Ch. |
|---|---|---|
| program, P*nn*, major mode | the running activity; MODREG, shown in PROG | 1, 7 |
| routine, R*nn* | a subprogram called by programs or verbs | 1 |
| verb, V*nn* | crew command; 40 and up are extended verbs | 1, 6 |
| noun, N*nn* | data to show or load; noun table row with scales | 6 |
| pinball | PINBALL_GAME_BUTTONS_AND_LIGHTS: the DSKY software | 6 |
| CHARIN | the job that handles one key | 6 |
| DSPTAB | display relay table, sent to channel 10 by T4RUPT | 6 |
| flashing display | a question; answers V33 PRO, V34 terminate, ENTER/V32 | 6 |
| GODSP, GOFLASH, GOPERF*n*, ...R | display interface entries; `agc_dsky_display` kinds | 6 |
| `agc_dsky_cont_t` | the three continuation jobs of a flashing display | 6 |
| paste | V99 or V97 flashed over another noun's data (NVWORD1) | 6 |
| KEY REL, OPR ERR, PROG | DSKY lamps: release display, operator error, program alarm | 6, 7 |
| V37 | change program | 7 |

## Navigation and guidance

| Term | Meaning | Ch. |
|---|---|---|
| RN, VN, PIPTIME | this vehicle's state during powered flight, m 2^29, m/cs 2^7, cs 2^28 | 2, 8 |
| RRECTCSM, RRECTLEM | the permanent state vectors of each vehicle | 8 |
| MOONFLAG | moon is the sphere of influence (and the scale switch) | 5, 8 |
| KEPLER, LAMBERT, TIMETHET, TIMERAD | conic routines | 8 |
| Encke integration, rectification | integrate the deviation from a conic; restart the conic | 8 |
| W matrix | error transition matrix, W W^T = covariance | 8 |
| INCORP1, INCORP2 | measurement incorporation | 8 |
| mark | an optics or radar measurement | 8 |
| SERVICER, average-G, READACCS | powered-flight navigation every 2 s | 9 |
| AVGEXIT | the guidance job SERVICER continues into | 9 |
| VG, VGTIG | velocity to be gained | 9 |
| TIG, TGO, TTOGO | time of ignition, time to go | 9 |
| S40.8, cross-product steering | steer thrust toward VG | 9 |
| DAP, T5 | digital autopilot, run in T5RUPT | 9 |
| TVC | thrust vector control: steering the SPS engine bell | 9 |
| KALCMANU, R60 | automatic attitude manoeuvre | 9 |
| V99, V97 | engine enable request; engine failed | 6, 9 |
| BURNBABY | the LM's shared ignition countdown | 9, 10 |
| WCHPHASE, TTF/8 | landing phase index; time to go / 8 | 10 |
| LPD | landing point designator | 10 |
| ROD | rate of descent (P66) | 10 |
| DELTAH | landing radar altitude minus computed altitude | 10 |
| L/D, LAD, ROLLC | entry lift-to-drag (desired, maximum), roll command | 11 |
| LAT(SPL), LNG(SPL) | splash target | 11 |

## Programs mentioned

| Number | Vehicle | Name (C stem) | Ch. |
|---|---|---|---|
| P00 | both | idle | 7 |
| P12 | LM | powered_ascent | 10 |
| P20 | both | rendezvous_navigation | 8 |
| P30 | both | external_delta_v | 9 |
| P40 | CSM / LM | sps_burn / dps_burn | 9 |
| P41 | both | rcs_burn | 9 |
| P52 | both | imu_realign | 3, 6, 8 |
| P61-P67 | CSM | entry_preparation .. entry_final_phase | 11 |
| P63, P64, P66 | LM | braking_phase, approach_phase, rate_of_descent_landing | 10 |
| P68 | LM | landing_confirmation | 10 |
| P70, P71 | LM | dps_abort, aps_abort | 10 |
| R22 | both | rendezvous_tracking_data | 8 |
| R30 | both | orbit_parameter_display | 1 |
| R60 | both | attitude_maneuver | 9 |

The full table of stems is `docs/NAMING.md`.
