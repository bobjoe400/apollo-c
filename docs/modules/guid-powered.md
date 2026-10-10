# guid-powered -- powered-flight guidance (P30s, P40s, SERVICER, BURNBABY)

## Scope

| AGC file (Comanche055 / Luminary099)                       | C file |
|------------------------------------------------------------|--------|
| POWERED_FLIGHT_SUBROUTINES.agc (CM pp.1365-1372, LM pp.1259-1267) | src/guidance/powered_flight.c |
| SERVICER207.agc (CM pp.819-836) / SERVICER.agc (LM pp.857-897), shared part | src/guidance/servicer_common.c |
| SERVICER207.agc, CM-only part                              | src/csm/servicer_csm.c |
| SERVICER.agc, LM-only part (MUNRVG, mass monitor, DVMON, LR update) | src/lm/servicer_lm.c |
| P30-P37.agc (CM pp.635-648) / P30_P37.agc (LM pp.614-617)  | src/guidance/external_delta_v.c |
| P40-P47.agc shared math (S40.1, S40.2,3, S40.8 TGO/XPRODUCT, S40.9, S40.13, S41.1) | src/guidance/burn_steering.c |
| P40-P47.agc CM-only (P40CSM/P41CSM/P47CSM, TIG-30/-5/-0, IGNITION, CLOKTASK, V97/V99, S40.6, S41.2, S40.14, DAPFIG) | src/csm/sps_burn.c |
| P40-P47.agc LM-only (P40LM/P41LM/P42LM/P47LM, POSTBURN, ENGINOFF, STEERING, RASTEER1, TRIMGIMB) | src/lm/dps_burn.c |
| BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc (LM pp.731-751) | src/lm/ignition.c |
| public interface                                           | include/agc/agc_guid.h |
| pending (not-yet-existing) shared symbols, one place each  | src/guidance/guid_pending.h, src/csm/guid_pending_csm.h, src/lm/guid_pending_lm.h |

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
| include/agc/agc_guid.h | DONE (rewritten on resume: targeting erasables shared with prog-rendz moved to agc_rendz_t; CSM TVC values in agc_csm_dap.h) |
| pending headers (guid_pending*.h) | DONE guid_pending.h, src/csm/guid_pending_csm.h, src/lm/guid_pending_lm.h (extend as needed) |
| CDUTRIG/CD*TR*GS/QUICTRIG/AX*SR*T/*SMNB*/*NBSM*/SMCDURES/FLESHPOT | DONE powered_flight.c (all agc_guid_cdutrig..fleshpot) |
| guid_state.c (agc_guid_state/agc_guid_init) | DONE guid_state.c |
| PIPASR/REREADAC/CALCGRAV/CALCRVG/COPYCYC | DONE agc_guid_pipasr/rereadac_resume/calcgrav/calcrvg/copycycle servicer_common.c |
| CM PREREAD/READACCS/SERVICER/AVERAGEG/AVGEND/ONMNITOR/NORMLIZE/REREADAC/SERVEXIT | DONE src/csm/servicer_csm.c (guid_pending_csm.h written) |
| LM PREREAD/BIBIBIAS/READACCS/SERVICER/MASSMON/DVMON/AVGEND/SERVIDLE/NORMLIZE/REREADAC | DONE src/lm/servicer_lm.c (guid_pending_lm.h, servicer_lm_internal.h) |
| LM RVBOTH/MUNRVG/MUNGRAV | DONE servicer_lm.c; MUNRETRN/ALTCHK/LR update/LR jobs/SETPOS DONE src/lm/servicer_lm_lr.c |
| P30/P31/S30.1/S31.1/MIDGIM/AGAIN | DONE src/guidance/external_delta_v.c (+ CM REFTEST/COMPTGO/DISP45/END45 in src/csm/external_delta_v_csm.c); DELRSPL stub |
| S40.1/CALCUT/S40.2,3/S41.1/S40.8 TGO+XPRODUCT/CM S40.13 | DONE src/guidance/burn_steering.c (CM S40.8/S40.9 drivers in sps_burn_steer.c; LM S40.8/S40.9/S40.13 DONE in dps_burn.c) |
| CM P40/P41/P47 sequencing, S40.6, DAPFIG (S41.2/S40.14/RCSDAPON are dap-csm's) | DONE src/csm/sps_burn.c, sps_burn_ignition.c, sps_burn_steer.c, sps_burn_clock.c, sps_burn_internal.h |
| LM P40/P41/P42/P47 sequencing, TRIMGIMB | DONE src/lm/dps_burn.c |
| RASTEER1 | DONE agc_lm_rasteer1 src/lm/dps_burn_rasteer.c (formula verified vs Kepler orbit in python) |
| naming rule (docs/NAMING.md) rename pass | TODO (after ignition.c + tests) |
| BURNBABY (ignition.c) | TODO |
| tests/test_guidance.c | TODO |

## Requested core additions

(kept current; each item also declared in a guid_pending*.h until the core has it)

## Requested jobs-tasks

## Deviations

## Stubs

## Tests written

## Open questions
