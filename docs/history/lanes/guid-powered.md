# guid-powered -- lane history

Process diary of the guid-powered lane, moved verbatim out of [docs/modules/guid-powered.md](../../modules/guid-powered.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-14 (first and last commit touching the note).

## Progress

(update after every routine/file; DONE <C function> <file> / PARTIAL <left> / TODO)

| AGC routine | status |
|---|---|
| include/agc/guidance/guidance.h | DONE (rewritten on resume: targeting erasables shared with prog-rendz moved to agc_rendz_t; CSM TVC values in agc_csm_dap.h) |
| pending headers (guid_pending*.h) | DONE guid_pending.h, src/csm/burns/guid_pending.h, src/lm/burns/guid_pending.h (extend as needed) |
| CDUTRIG/CD*TR*GS/QUICTRIG/AX*SR*T/*SMNB*/*NBSM*/SMCDURES/FLESHPOT | DONE powered_flight.c (all agc_guid_cdutrig..fleshpot) |
| guid_state.c (agc_guid_state/agc_guid_init) | DONE guid_state.c |
| PIPASR/REREADAC/CALCGRAV/CALCRVG/COPYCYC | DONE agc_guid_pipasr/rereadac_resume/calcgrav/calcrvg/copycycle servicer_common.c |
| CM PREREAD/READACCS/SERVICER/AVERAGEG/AVGEND/ONMNITOR/NORMLIZE/REREADAC/SERVEXIT | DONE src/csm/burns/servicer.c (guid_pending_csm.h written) |
| LM PREREAD/BIBIBIAS/READACCS/SERVICER/MASSMON/DVMON/AVGEND/SERVIDLE/NORMLIZE/REREADAC | DONE src/lm/burns/servicer/average_g.c (src/lm/burns/guid_pending.h, servicer/internal.h) |
| LM RVBOTH/MUNRVG/MUNGRAV | DONE servicer/average_g.c; MUNRETRN/ALTCHK/LR update/LR jobs/SETPOS DONE src/lm/burns/servicer/landing_radar.c |
| P30/P31/S30.1/S31.1/MIDGIM/AGAIN | DONE src/guidance/external_delta_v.c (+ CM REFTEST/COMPTGO/DISP45/END45 in src/csm/burns/external_delta_v.c); DELRSPL stub |
| S40.1/CALCUT/S40.2,3/S41.1/S40.8 TGO+XPRODUCT/CM S40.13 | DONE src/guidance/burn_steering.c (CM S40.8/S40.9 drivers in src/csm/burns/sps/steer.c; LM S40.8/S40.9/S40.13 DONE in dps_burn.c) |
| CM P40/P41/P47 sequencing, S40.6, DAPFIG (S41.2/S40.14/RCSDAPON are dap-csm's) | DONE src/csm/burns/sps/{burn,ignition,steer,clock}.c, sps/internal.h |
| LM P40/P41/P42/P47 sequencing, TRIMGIMB | DONE src/lm/burns/dps_burn.c |
| RASTEER1 | DONE agc_lm_rasteer1 src/lm/burns/dps_burn_rasteer.c (formula verified vs Kepler orbit in python) |
| naming rule (docs/NAMING.md) rename pass | DONE for the ignition files (B*RNB*B* -> burnbaby_countdown, IGNYET? -> ignition_if_enabled, TIG-30A -> restore_burn_display, P41SPOT -> preburn_state_vector_lm); rest was done tree-wide by the naming agent |
| BURNBABY (tables, B*RNB*B*, P41SPOT, TIG-35..TIG-0, IGNYET?, IGNITION, P12/P40/P42/P63/ABRT IGN, P42IGN/DVMONCON, ULLAGOFF, TIGTASK, P40ZOOM/P63ZOOM/ZOOM, P41BLANK, TIG-30A) | DONE src/lm/burns/ignition/burnbaby.c |
| STCLOK1/2/3, CLOKTASK, CLOKJOB, STOPCLOK, NULLCLOK, ASTNRETN, *PROCEED/IGNITE, *ENTER/GOPOST/GOCUTOFF, COMFAIL/1-4, REP40ALM, P40AUTO | DONE src/lm/burns/ignition/clock.c |
| tests/guidance/guidance.c | TODO |

## File moves (layout rule: 3+ files sharing a prefix become a subfolder)

| old | new |
|---|---|
| src/csm/burns/sps_burn.c | src/csm/burns/sps/burn.c |
| src/csm/burns/sps_burn_clock.c | src/csm/burns/sps/clock.c |
| src/csm/burns/sps_burn_ignition.c | src/csm/burns/sps/ignition.c |
| src/csm/burns/sps_burn_steer.c | src/csm/burns/sps/steer.c |
| src/csm/burns/sps_burn_internal.h | src/csm/burns/sps/internal.h |
| src/lm/burns/servicer.c | src/lm/burns/servicer/average_g.c |
| src/lm/burns/servicer_lr.c | src/lm/burns/servicer/landing_radar.c |
| src/lm/burns/servicer_internal.h | src/lm/burns/servicer/internal.h |
| src/lm/burns/ignition.c | src/lm/burns/ignition/burnbaby.c |
| src/lm/burns/ignition_internal.h | src/lm/burns/ignition/internal.h |
| (new) | src/lm/burns/ignition/clock.c |

Private includes now `"internal.h"` and `"../guid_pending.h"` (done with git mv).

## Requested core additions

Each is declared in one pending header until its owner provides it:

* include/agc/guidance/pending.h: `agc_svc_delayjob` (sys-misc), `agc_waitlist_kill`
  (KILLTASK, exec), `agc_exec_change_program_continue` (CANV37), `agc_phase_set_tbase`
  (CS TIME1 / TS TBASEn), the nav glue (`agc_guid_prec_update`, `agc_guid_midtoav1/2`,
  `agc_guid_avetomid`, implemented in src/guidance/guid_nav_glue.c), `agc_rendz_havegues`.
* src/csm/burns/guid_pending.h: `agc_erase_aog/aig/amg`, `agc_erase_set_cm_gymdt`,
  `agc_erase_set_vhfcnt`, `agc_erase_set_mrkbuf2`, `agc_erase_set_optind`,
  `agc_erase_set_tvcpitch/tvcyaw`, `agc_erase_time5/set_time5`,
  `agc_csm_velocity_altitude_rate`, `agc_csm_satstkon`, `agc_dsky_gopin`.
* src/lm/burns/guid_pending.h: `agc_erase_time5/set_time5`, `agc_erase_set_thrust`,
  LR radar routines (`agc_radar_lralt/lrvel/lrpos2/samplsum/landing_radar_lights/
  powered_flight_designate`), `agc_lm_abort_monitor_resume`, `agc_erase_geomsgn`, `agc_erase_un`.

## Requested jobs-tasks

All guid-powered ids are in include/agc/guidance/pending.h (AGC_JOB_GUID_BASE /
AGC_TASK_GUID_BASE) plus the vehicle pending headers. Several duplicate ids
in src/exec/pending.h with different values (CLOKTASK, TIG_5, TIG_0, TIG_30,
TIG_35, IGNITION, ULLGTASK, ULLAGOFF, PREREAD, REREADAC, PREBURN_STATE_VECTOR,
COMFAIL, ...): integration must merge to one value each. New this session:

| id | function | AGC |
|---|---|---|
| AGC_JOB_BURNBABY_COUNTDOWN (was BRNBB) | agc_job_burnbaby_countdown | B*RNB*B* |
| AGC_JOB_PREBURN_STATE_VECTOR (LM use) | agc_job_preburn_state_vector_lm | P41SPOT (same concept as CM P40S/SV; dispatch by vehicle) |
| AGC_JOB_RESTORE_BURN_DISPLAY (was TIG_30A) | agc_job_restore_burn_display | TIG-30A |
| AGC_TASK_IGNITION_IF_ENABLED | agc_task_ignition_if_enabled | IGNYET? (TIG-0 4.1 type C restart) |
| AGC_TASK_IGNITION_FINISH | agc_task_ignition_finish | P42IGN (P40IGN 4.1 type C restart) |
| AGC_TASK_ZOOM (exec's value, copied) | agc_task_zoom | ZOOM = P40ZOOMA (3.3) |
| AGC_JOB_CLOCKJOB (LM use) | agc_job_clokjob | CLOKJOB |
| AGC_TASK_CLOKTASK (LM use) | agc_task_cloktask_lm | CLOKTASK |
| AGC_JOB_CLOKJOB_ENGINE_FAIL / _SHUTDOWN | agc_job_clokjob_engine_fail / _shutdown | COMFAIL1 / COMFAIL2 |
| AGC_JOB_CLOKJOB_ENGINE_ENABLE / _BYPASS | agc_job_clokjob_engine_enable / _bypass | *PROCEED / *ENTER |

Removed (were unused duplicates): AGC_JOB_IGNITE, AGC_JOB_STAR_ENTER, AGC_JOB_COMFAIL1,
AGC_JOB_COMFAIL2, AGC_JOB_ASTNRETN, AGC_JOB_GOPOST.

## Open questions

* The 3.3 restart table entry and COMFAIL2's KILLTASK both name ZOOM (= P40ZOOMA, only
  "PHASCHNG 3"), so a restart between ignition and ZOOMTIME loses the throttle-up, and the
  V97 ENTER cannot remove a pending P40ZOOM/P63ZOOM. Translated as flown; worth a reviewer's look.
* ABRTABLE entries (5)-(7) are NOOPs: P70/P71 never enter BURNBABY. Confirm with guid-landing.

## Lessons

* VN pseudo-op digits are decimal (V99 exists): table nouns like `VN 0674` are noun 74, not
  octal 074. An earlier draft stored 074 (= 60) -- check every VN constant you carry as data.
* `CA Z / TS DISPDEX` stores the program counter, i.e. "some positive number"; give it a named
  constant (IGN_DISPDEX_KILL) and say why any positive value is equivalent.
* `INDEX WHICH / TCF n` tables become a struct of enums per program; keep the AGC entry number
  in the field comment so reviewers can map (11), (13), (14) back to the listing.
* KILLTASK compares task addresses, so an alias (`ZOOM = P40ZOOMA`) only kills tasks scheduled
  under that exact label. Read the `=` lines before translating a kill.
* STCLOK3's "mod 1 s" divides only (low 5 bits of the high word, low word) by 100: it is
  (dt mod 2^19) mod 100. Keep the AGC arithmetic rather than a plain `%`.
* When the lead restructures folders mid-run, re-grep the pending headers: two lanes had defined
  the same task id with different values; copying the other definition token-for-token keeps both
  headers includable together.

