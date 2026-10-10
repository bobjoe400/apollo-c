# guid-powered -- powered-flight guidance (P30s, P40s, SERVICER, BURNBABY)

## Scope

| AGC file (Comanche055 / Luminary099)                       | C file |
|------------------------------------------------------------|--------|
| POWERED_FLIGHT_SUBROUTINES.agc (CM pp.1365-1372, LM pp.1259-1267) | src/guidance/powered_flight.c |
| SERVICER207.agc (CM pp.819-836) / SERVICER.agc (LM pp.857-897), shared part | src/guidance/servicer_common.c |
| SERVICER207.agc, CM-only part                              | src/csm/burns/servicer.c |
| SERVICER.agc, LM-only part (MUNRVG, mass monitor, DVMON, LR update) | src/lm/burns/servicer.c |
| P30-P37.agc (CM pp.635-648) / P30_P37.agc (LM pp.614-617)  | src/guidance/external_delta_v.c |
| P40-P47.agc shared math (S40.1, S40.2,3, S40.8 TGO/XPRODUCT, S40.9, S40.13, S41.1) | src/guidance/burn_steering.c |
| P40-P47.agc CM-only (P40CSM/P41CSM/P47CSM, TIG-30/-5/-0, IGNITION, CLOKTASK, V97/V99, S40.6, S41.2, S40.14, DAPFIG) | src/csm/burns/sps_burn.c |
| P40-P47.agc LM-only (P40LM/P41LM/P42LM/P47LM, POSTBURN, ENGINOFF, STEERING, RASTEER1, TRIMGIMB) | src/lm/burns/dps_burn.c |
| BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc (LM pp.731-751) | src/lm/burns/ignition.c |
| public interface                                           | include/agc/guidance/guidance.h |
| pending (not-yet-existing) shared symbols, one place each  | include/agc/guidance/pending.h, src/csm/burns/guid_pending.h, src/lm/burns/guid_pending.h |

## Design table (written before coding)

Scale notation: "2^n unit" = the stored fraction times 2^n gives the value in that unit.

| AGC label (page) | inputs (scale) | outputs (scale) | C function |
|---|---|---|---|
| CDUTRIG / CDUTRIGS / CD*TR*G(S) (CM 1365-66, LM 1259-60) | CDU counts 2's comp (rev 2^0) | CDUSPOT 1's comp DP rev 2^0; SINCDU/COSCDU half sin/cos 2^1 | agc_guid_cdutrig(), agc_guid_cdtrigs() |
| QUICTRIG (CM 1367, LM 1261) | CDUSPOT SP counts | SINCDU/COSCDU 2^1 (SPSIN/SPCOS precision) | agc_guid_quictrig() |
| AX*SR*T, *SMNB*, *NBSM*, TRG*SMNB, TRG*NBSM, CDU*SMNB, CDU*NBSM (CM 1368-71, LM 1262-64) | vector (any scale, |v|<1), SINCDU/COSCDU order Y Z X | rotated vector, same scale | agc_guid_axisrot(), agc_guid_smnb(), agc_guid_nbsm(), agc_guid_cdu_smnb(), agc_guid_cdu_nbsm() |
| SMCDURES (CM 1372) | DTHETASM rev 2^0, trig | DCDU rev 2^0 | agc_guid_smcdures() |
| CALCSMSC / FLESHPOT (LM 1265-67) | trig | XNB / XNBPIP half-unit matrix 2^1 | agc_guid_fleshpot() |
| PIPASR / REREADAC (CM 832-834, LM 869-871) | PIPAX/Y/Z counts, TIME2/1 | DELV 2^14 pulses, PIPTIME1 cs 2^28, CDUTEMP | agc_guid_pipasr(), agc_guid_rereadac_resume() |
| CALCGRAV (CM 835-836, LM 880) | R m 2^29, UNITW/UNITZ, RTX2 | UNITR 2^1, GOBL1/2 or UNITGOBL, GDT1/2 m/cs 2^7, 34D/36D | agc_guid_calcgrav() |
| CALCRVG (CM 836, LM 880-881) | DELV pulses, RN 2^29, VN 2^7, GDT/2 2^7, REFSMMAT 2^1, dt | RN1 2^29, VN1 2^7, GDT1/2 2^7, DELVREF 2^7 | agc_guid_calcrvg() |
| COPYCYC (LM 867) / GENTRAN copy (CM 829) | *1 values | RN VN PIPTIME GDT/2 GOBL/2 MASS | agc_guid_copycycle() |
| NORMLIZE (CM 831, LM 867) | RN1 VN1 PIPTIME1 | GDT/2, GOBL/2 | agc_job_normlize_csm(), agc_job_normlize_lm() |
| PREREAD/READACCS/SERVICER/AVERAGEG/AVGEND/SERVEXIT (CM) | | | agc_task_preread_csm() ... (servicer_csm.c) |
| ONMNITOR / QUIKREAD (CM 827) | PIPAX/Y/Z | X/Y/ZPIPBUF, X/Y/ZOLDBUF | agc_task_onmnitor(), agc_task_quikread() |
| PREREAD/READACCS/SERVICER/MASSMON/DVMON/AVGEND/SERVIDLE (LM 857-866) | | | servicer_lm.c |
| RVBOTH / MUNRVG / MUNGRAV (LM 882-883) | DELV, R 2^24, V 2^7 (SM), PGUIDE | R1S 2^24, V1S 2^7, GDT1/2, ABVEL, HDOTDISP, DELVS 2^5, HCALC 2^24 | agc_lm_munrvg(), agc_lm_mungrav(), agc_lm_rvboth() |
| MUNRETRN..COPYCYC2, ALTCHK, LR update (LM 874-891) | LR data, flags | R1S/V1S updated, lamps | agc_lm_munretrn() and helpers |
| LRVJOB/LRHJOB/RDGIMS/HIGATJOB/SETPOS1/SETPOS2 (LM 892-897) | LR read routines | VMEAS, HMEAS, beam vectors | servicer_lm.c |
| P30 / P31 / P30/P31 / CNTNUP30 / PARAM30 / REFTEST (CM 636-638), P30 (LM 614-615) | | | agc_job_external_delta_v_csm(), agc_job_lambert_aimpoint_guidance_csm(), agc_job_external_delta_v_lm() + continuations |
| S30.1 (CM 639-640, LM 616-617) | TIG cs 2^28, DELVSLV 2^7 | RTIG 2^29, VTIG 2^7, DELVSIN 2^7, VGDISP/DELVSAB, HAPO/HPER 2^29 | agc_guid_external_delta_v_vectors() |
| S31.1 (CM 641-642) | TIG, DELLT4, RTARG | HAPO HPER DELVLVC VGDISP TPASS4 | agc_guid_lambert_aimpoint_vectors() |
| DELRSPL (CM 643-647) | | RSP-RREC | agc_guid_delrspl() (stub) |
| S40.1 / S40.1B / AGAIN / CALCUT (CM 709-713, LM 764-767) | see header | UT 2^1, VGTIG 2^7, DELVLVC, BDT | agc_guid_initial_velocity_to_gain() |
| S40.2,3 (CM 714-717, LM 768-769) | UT, REFSMMAT, RTIG, trims | POINTVSM, SCAXIS, X/Y/ZSCREF | agc_guid_preferred_thrust_attitude_csm(), agc_guid_preferred_thrust_attitude_lm() |
| S41.1 (CM 717-718, LM 783-784) | VG ref 2^7, trig | VG control/body 2^7 | agc_guid_reference_to_body() |
| S40.8 (CM 719-722, LM 770-772) | VGPREV DELVREF BDT 2^7, CSTEER | VG, VGDISP, TGO 2^28 cs, TIG, OMEGAC | agc_guid_steering_tgo(), agc_guid_steering_cross_product(), agc_csm_cross_product_steering(), agc_lm_cross_product_steering() |
| S40.9 (CM 723-725, LM 775-776) | RINIT VINIT TNIT VRPREV | VGTEMP, BDT, VRPREV | agc_job_vtogain_csm(), agc_job_vtogain_lm() |
| S40.13 TIMEBURN (CM 726-728, LM 773-775) | VGTIG, WEIGHT/G, F, MDOT | IMPULSW, NOTHROTL, TGO | agc_guid_timeburn_csm(), agc_guid_timeburn_lm() |
| UPDATEVG / STEERING / CALCN85 / CALCN83 (CM 699-702, LM 761-763) | | | sps_burn.c / dps_burn.c |
| TIG-30 / TIG-5 / TIG-0 / IGNITION / SPSOFF / ENGINOFF / CLOKTASK / CLOCKJOB / V97x / V99x (CM 693-709) | | | sps_burn.c |
| S40.6 / S41.2 / DAPFIG / S40.14 / RCSDAPON (CM 729-736) | | | sps_burn.c |
| RASTEER1 (LM 776-780) | RTARG RN TPASS4 | VG via VGAIN* | agc_lm_rasteer1() |
| TRIMGIMB (LM 781-782) | PITTIME ROLLTIME | channel 12 | agc_task_trimgimb() ... |
| BURNBABY tables, TIG-35/30/5/0, IGNITION, COMFAIL, CLOKTASK/CLOKJOB, STCLOK, NULLCLOK, P40AUTO, KILLTASK (LM 731-751) | | | ignition.c |

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
| tests/test_guidance.c | TODO |

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

## Deviations

* powered_flight.c CDULOGIC: 1's-complement logic applied to 2's-complement CDU counts (see code).
* servicer_common.c PIPASR: the three PIPAs are read in one C step (the AGC's DXCH/XCH order is moot).
* csm/burns/servicer.c: group 6 TBASE from PIPTIME (see code comment).
* csm/burns/sps/internal.h: -0 markers stored as -1.
* lm/burns/servicer/average_g.c: TBASE2 ".05 s in the past" computed directly.
* lm/burns/servicer/landing_radar.c: RADSTALL return paths and PRIOCHNG change-job (see code).
* lm/burns/ignition/clock.c: STCLOK1 restart point is COMFAIL's start (AGC: COMFAIL +3);
  V97 display uses the table noun instead of NVWORD +2; the V06N61 ENTER recycles through
  CLOKJOB (recomputes TTOGO). The V97/V99 paste is passed as the CLOCPLAY verb (as in the CSM clock).

## Stubs

* DELRSPL (src/guidance/external_delta_v.c): AGC_STATUS_UNIMPLEMENTED.

## Tests written

tests/test_guidance.c (13 checks, all pass when linked with core + src/guidance objects;
the full libagc link currently fails on other lanes' undefined symbols):
1. CALCRVG with zero delta-V and GDT/2 = 0 over a 2-s step (LM, moon): RN1 - RN = VN * 200 cs
   within 2 LSB (4 m at 2^29), VN1 = VN + GDT1/2 within 0.2 % of mu dt / 2 r^2.
2. S40.8 cross-product steering: BDT = 0, DELVREF along VG -> OMEGAC = 0 (within 64 LSB);
   VG perpendicular to thrust -> large OMEGAC.

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
