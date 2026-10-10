# guid-powered -- powered-flight guidance (P30s, P40s, SERVICER, BURNBABY)

## Scope

| AGC file (Comanche055 / Luminary099)                       | C file |
|------------------------------------------------------------|--------|
| POWERED_FLIGHT_SUBROUTINES.agc (CM pp.1365-1372, LM pp.1259-1267) | src/guidance/powered_flight.c |
| SERVICER207.agc (CM pp.819-836) / SERVICER.agc (LM pp.857-897), shared part | src/guidance/servicer_common.c |
| SERVICER207.agc, CM-only part                              | src/csm/servicer_csm.c |
| SERVICER.agc, LM-only part (MUNRVG, mass monitor, DVMON, LR update) | src/lm/servicer_lm.c |
| P30-P37.agc (CM pp.635-648) / P30_P37.agc (LM pp.614-617)  | src/guidance/p30.c |
| P40-P47.agc shared math (S40.1, S40.2,3, S40.8 TGO/XPRODUCT, S40.9, S40.13, S41.1) | src/guidance/p40.c |
| P40-P47.agc CM-only (P40CSM/P41CSM/P47CSM, TIG-30/-5/-0, IGNITION, CLOKTASK, V97/V99, S40.6, S41.2, S40.14, DAPFIG) | src/csm/p40_csm.c |
| P40-P47.agc LM-only (P40LM/P41LM/P42LM/P47LM, POSTBURN, ENGINOFF, STEERING, RASTEER1, TRIMGIMB) | src/lm/p40_lm.c |
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
| P30 / P31 / P30/P31 / CNTNUP30 / PARAM30 / REFTEST (CM 636-638), P30 (LM 614-615) | | | agc_job_p30_csm(), agc_job_p31_csm(), agc_job_p30_lm() + continuations |
| S30.1 (CM 639-640, LM 616-617) | TIG cs 2^28, DELVSLV 2^7 | RTIG 2^29, VTIG 2^7, DELVSIN 2^7, VGDISP/DELVSAB, HAPO/HPER 2^29 | agc_guid_s30_1() |
| S31.1 (CM 641-642) | TIG, DELLT4, RTARG | HAPO HPER DELVLVC VGDISP TPASS4 | agc_guid_s31_1() |
| DELRSPL (CM 643-647) | | RSP-RREC | agc_guid_delrspl() (stub) |
| S40.1 / S40.1B / AGAIN / CALCUT (CM 709-713, LM 764-767) | see header | UT 2^1, VGTIG 2^7, DELVLVC, BDT | agc_guid_s40_1() |
| S40.2,3 (CM 714-717, LM 768-769) | UT, REFSMMAT, RTIG, trims | POINTVSM, SCAXIS, X/Y/ZSCREF | agc_guid_s40_23_csm(), agc_guid_s40_23_lm() |
| S41.1 (CM 717-718, LM 783-784) | VG ref 2^7, trig | VG control/body 2^7 | agc_guid_s41_1() |
| S40.8 (CM 719-722, LM 770-772) | VGPREV DELVREF BDT 2^7, CSTEER | VG, VGDISP, TGO 2^28 cs, TIG, OMEGAC | agc_guid_s40_8_tgo(), agc_guid_s40_8_xproduct(), agc_csm_s40_8(), agc_lm_s40_8() |
| S40.9 (CM 723-725, LM 775-776) | RINIT VINIT TNIT VRPREV | VGTEMP, BDT, VRPREV | agc_job_s40_9_csm(), agc_job_s40_9_lm() |
| S40.13 TIMEBURN (CM 726-728, LM 773-775) | VGTIG, WEIGHT/G, F, MDOT | IMPULSW, NOTHROTL, TGO | agc_guid_s40_13_csm(), agc_guid_s40_13_lm() |
| UPDATEVG / STEERING / CALCN85 / CALCN83 (CM 699-702, LM 761-763) | | | p40_csm.c / p40_lm.c |
| TIG-30 / TIG-5 / TIG-0 / IGNITION / SPSOFF / ENGINOFF / CLOKTASK / CLOCKJOB / V97x / V99x (CM 693-709) | | | p40_csm.c |
| S40.6 / S41.2 / DAPFIG / S40.14 / RCSDAPON (CM 729-736) | | | p40_csm.c |
| RASTEER1 (LM 776-780) | RTARG RN TPASS4 | VG via VGAIN* | agc_lm_rasteer1() |
| TRIMGIMB (LM 781-782) | PITTIME ROLLTIME | channel 12 | agc_task_trimgimb() ... |
| BURNBABY tables, TIG-35/30/5/0, IGNITION, COMFAIL, CLOKTASK/CLOKJOB, STCLOK, NULLCLOK, P40AUTO, KILLTASK (LM 731-751) | | | ignition.c |

## Progress

(update after every routine/file; DONE <C function> <file> / PARTIAL <left> / TODO)

| AGC routine | status |
|---|---|
| include/agc/agc_guid.h | DONE (rewritten on resume: targeting erasables shared with prog-rendz moved to agc_rendz_t; CSM TVC values in agc_csm_dap.h) |
| pending headers (guid_pending*.h) | PARTIAL guid_pending.h DONE (reconciled with state-flags/exec/dsky/imu/programs); csm/lm ones TODO |
| CDUTRIG/CD*TR*GS/QUICTRIG/AX*SR*T/*SMNB*/*NBSM*/SMCDURES/FLESHPOT | DONE powered_flight.c (all agc_guid_cdutrig..fleshpot) |
| guid_state.c (agc_guid_state/agc_guid_init) | TODO |
| PIPASR/REREADAC/CALCGRAV/CALCRVG/COPYCYC/NORMLIZE | TODO |
| CM PREREAD/READACCS/SERVICER/AVERAGEG/AVGEND/ONMNITOR | TODO |
| LM PREREAD/READACCS/SERVICER/MASSMON/DVMON/AVGEND/SERVIDLE | TODO |
| LM RVBOTH/MUNRVG/MUNGRAV/MUNRETRN/LR update/LR jobs | TODO |
| P30/P31/S30.1/S31.1/DELRSPL | TODO |
| S40.1/S40.2,3/S40.8/S40.9/S40.13/S41.1 | TODO |
| CM P40/P41/P47 sequencing, S40.6, DAPFIG (S41.2/S40.14/RCSDAPON are dap-csm's) | TODO |
| LM P40/P41/P42/P47 sequencing, RASTEER1, TRIMGIMB | TODO |
| BURNBABY (ignition.c) | TODO |
| tests/test_guidance.c | TODO |

## Requested core additions

(kept current; each item also declared in a guid_pending*.h until the core has it)

## Requested jobs-tasks

## Deviations

## Stubs

## Tests written

## Open questions
