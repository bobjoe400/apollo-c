# guid-powered -- powered-flight guidance (P30s, P40s, SERVICER, BURNBABY)

History of this lane: [history/lanes/guid-powered.md](../../history/lanes/guid-powered.md)

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

tests/guidance/guidance.c (13 checks, all pass when linked with core + src/guidance objects;
the full libagc link currently fails on other lanes' undefined symbols):
1. CALCRVG with zero delta-V and GDT/2 = 0 over a 2-s step (LM, moon): RN1 - RN = VN * 200 cs
   within 2 LSB (4 m at 2^29), VN1 = VN + GDT1/2 within 0.2 % of mu dt / 2 r^2.
2. S40.8 cross-product steering: BDT = 0, DELVREF along VG -> OMEGAC = 0 (within 64 LSB);
   VG perpendicular to thrust -> large OMEGAC.

## Program flow and decisions

_Documentation wave (2026-10-12); prose and tables brought to the final tree (HEAD 3b97dc8) by the final pass (2026-10-14). The prose was written by reading the code; the two tables were extracted from the function bodies (flag, alarm, channel and scheduling calls; `agc_erase_*` accessors) and list only what the code does. Files: `src/guidance/*.c`, `src/csm/burns/servicer.c`, `src/csm/burns/external_delta_v.c`, `src/csm/burns/sps/*.c`, `src/lm/burns/*.c`, `src/lm/burns/*/*.c`._

The CSM P40 sequence is drawn in
[flowcharts/sps-burn.md](../overview/flowcharts/sps-burn.md). This lane
holds P30/P31 targeting, the P40-P47 burn programs of both vehicles,
the shared steering mathematics, BURN BABY BURN and both SERVICERs.

* **Targeting** (`src/guidance/external_delta_v.c`,
  `src/csm/burns/external_delta_v.c`): P30 takes the crew's external dV
  (XDELVFLG set), P31 (CSM) a Lambert aimpoint (XDELVFLG clear); S30.1 /
  S31.1 integrate to TIG (`NOT_FOUND` when an integration ends without
  RECTOUT) and leave RTIG, VTIG, DELVSIN, +MGA, HAPO/HPER. CSM REFTEST,
  DISP45 and TIMRFLAG drive the TTOGO clock; DELRSPL feeds V82 in P11/P00.
* **Pre-ignition** (CSM `src/csm/burns/sps/burn.c`, LM
  `src/lm/burns/dps_burn.c`, `src/lm/burns/ignition/burnbaby.c`): S40.1
  (`burn_steering.c:agc_guid_initial_velocity_to_gain`, XDELVFLG selects
  CALCTHET or the Lambert form S40.1B), S40.2,3 preferred attitude, R60
  maneuver. The state is integrated to TIG - 30 s
  (`burn.c:agc_job_preburn_state_vector`,
  `burnbaby.c:agc_job_preburn_state_vector_lm`: MIDTOAV1; a slipped TIG is
  moved to 29.96 s / 29.9 s after the integrated time). The next task is
  set up from what time is left: P40SET (`agc_csm_preburn_call_tigblnk`,
  P40TMP) and CALLT-35 (`burnbaby.c:ign_call_tig_35`, SAVET-30) pass a
  delay <= 0 to LONGCALL as 0, i.e. POODOO 1204 exactly as the ropes do
  (closed-loop-fixes P40TMP). The TIG chain: TIG-30 (LM `agc_task_tig_30`:
  ullage, APSFLAG for P42), TIG-5 (`agc_task_tig_5_csm` / `_lm`: V99
  flash, S40.13 TIMEBURN), TIG-0 (`agc_task_tig_0_csm` / `_lm`: IGNFLAG;
  ignite at once if the crew already answered V99 (ASTNFLAG), else wait;
  LM P63 throttles up ZOOMTIME later). The LM stage check is ALARM 01706
  (`dps_burn.c:stage_alarm`, REP40ALM V05N09 loop).
* **Ignition** (`src/csm/burns/sps/ignition.c:agc_task_ignition_csm`,
  `burnbaby.c:agc_task_ignition_finish`): ENGONFLG, engine on, TEVENT;
  IMPULSW (impulsive burn, TGO short) goes to IMPLBURN with an engine-off
  task, otherwise steering (CSM STRULLSW, LM DV monitor connected).
* **Steering every 2 s** (from AVGEXIT): S40.8 (`steer.c:
  steering_after_vg_update` CSM, `dps_burn.c:agc_lm_cross_product_steering`
  LM): VG updated from DELVREF (external dV) or the Lambert / RASTEER1
  aimpoint (`dps_burn_rasteer.c`); with STEERSW the low-thrust test against
  DVTHRESH, TGOCALC (thrust more than 90 deg from VG returns ALARM to the
  caller), and when TGO is short the engine-off task is set (IMPULSW);
  VG growing is ALARM 01407 (INCRSVG). CSM S40.9 re-solves Lambert in a
  job; S40.13 sizes the impulsive burn. LM STEERING
  (`agc_job_steering_lm`) stops the rates and schedules ENGOFTSK at the
  computed cutoff.
* **SERVICER** (CSM `src/csm/burns/servicer.c`, LM
  `src/lm/burns/servicer/average_g.c`, shared
  `src/guidance/servicer_common.c`): PREREAD 2 s before the first update,
  READACCS every 2 s while AVEGFLAG is set (`chekaveg`: last pass makes
  AVGEND the exit), SERVICER compensates the PIPAs (CSM: a saturated PIPA
  is ALARM 00205 and skips compensation), integrates RN/VN by CALCRVG (LM
  on the surface: MUNRVG), copies the cycle (`agc_lm_serv_copycycl`:
  STEERSW cleared before DVMON; IDLEFLAG / AUXFLAG bypass it) and goes to
  AVGEXIT. LM DVMON (`dvmon`): thrust when ABDELV > DVTHRUSH sets STEERSW;
  no thrust after ignition is COMFAIL (`clock.c:agc_job_comfail`: V97
  flash, IDLEFLAG). Landing-radar updates in SERVICER are
  `landing_radar.c` (`agc_lm_serv_munretrn`: LR reads below 50 000 ft,
  X-axis override, ALARM 00511 antenna not in position, 00523 position 2
  not reached).
* **Clock and crew** (`src/csm/burns/sps/clock.c`,
  `src/lm/burns/ignition/clock.c`): CLOKTASK/CLOCKJOB once a second (TTOGO,
  V06N40, NVWORD1 selects V99 / V97), V99 answers (proceed = ignite,
  terminate = POSTBURN without burn), V97 engine fail answers; POSTBURN
  (`agc_job_postburn_csm` / `_lm`) flashes V16N40, PROCEED -> RCS trim
  (CSM P40RCS) or TIGNOW (LM), V34 -> TERM40.

### Decision points by function

161 of the lane's 329 functions test or change a flag, raise an alarm, test a channel bit or start a job/task (AGC = the label(s) the prolog names; "+F" sets flag F, "-F" clears it; "starts" = NOVAC/FINDVAC, WAITLIST, LONGCALL, DELAYJOB targets).

| C function | AGC | flags tested | flags set (+) / cleared (-) | alarms | channel bits | starts |
|---|---|---|---|---|---|---|
| `agc_csm_comptgo` (external_delta_v.c) | COMPTGO | - | +TIMRFLAG | - | - | task cloktask |
| `agc_job_external_delta_v_reftest` (external_delta_v.c) | REFTEST | REFSMFLG | - | - | - | - |
| `agc_job_marks_tfi_mga_done` (external_delta_v.c) | END45 | - | -TIMRFLAG -XDELVFLG +XDELVFLG | - | - | - |
| `agc_guid_delrspl` (external_delta_v.c) | DELRSPL | LUNAFLAG SERVICER_RUNNING | -ERADFLAG | - | - | - |
| `preread1` (servicer.c) | PREREAD1 | - | +AVEGFLAG -DRIFTFLG +SERVICER_RUNNING | - | - | - |
| `agc_task_preread_csm` (servicer.c) | PREREAD REDO5.31 | - | - | - | - | job lastbias, job normlize, task readaccs |
| `readaccs_entry` (servicer.c) | - | CMDAPARM | - | - | - | task quikread, task setjtag |
| `chekaveg` (servicer.c) | CHEKAVEG MAKESERV AVEGOUT | AVEGFLAG | - | - | ch11 TESTCONN | task readaccs, job servicer |
| `redo5_5` (servicer.c) | REDO5.5 | CM_DSTBY | - | - | - | - |
| `agc_task_quikread` (servicer.c) | QUIKREAD CHKCTR | - | - | - | - | task onmnitor |
| `agc_job_servicer_csm` (servicer.c) | SERVICER | - | - | ALARM 0205 | - | - |
| `agc_job_avgend_csm` (servicer.c) | AVGEND | - | +DRIFTFLG | - | - | - |
| `agc_job_avetomid_csm` (servicer.c) | AVGEND | RNDVZFLG | -CM_DSTBY -SERVICER_RUNNING | - | ch11 TESTCONN | - |
| `burn_setup_with_thrust` (burn.c) | P40S/F P40PVA P40SXTY | ENG2FLAG | +FIRSTFLG +PFRATFLG | - | - | - |
| `agc_job_sps_burn` (burn.c) | P40CSM | XDELVFLG | -ENG2FLAG | - | - | - |
| `agc_job_rcs_burn_csm` (burn.c) | P41CSM | NJETSFLG | +ENG2FLAG | - | - | - |
| `agc_job_sps_burn_after_maneuver` (burn.c) | P40SXTY | - | +TIMRFLAG | - | - | - |
| `trim_common` (burn.c) | TRIMONLY | - | - | - | - | task gimbal_test_trim, job preburn_start_clock |
| `agc_csm_preburn_start_clock` (burn.c) | P40S/RS | - | - | - | - | task cloktask |
| `agc_csm_preburn_call_tigblnk` (burn.c) | P40SET | - | - | - | - | task tigblnk |
| `agc_task_tigblnk` (burn.c) | TIGBLNK | - | - | - | - | task tigaveg, job preburn_blank_dsky |
| `agc_task_tigaveg` (burn.c) | TIGAVEG TIG-30 | - | - | - | - | task ttg_0, task tig_5 |
| `agc_task_ttg_0` (burn.c) | TTG/0 | - | -TIMRFLAG | - | - | job tignow |
| `agc_task_tig_5_csm` (burn.c) | TIG-5 | - | - | - | - | task tig_0, job timeburn |
| `agc_task_tig_0_csm` (burn.c) | TIG-0 | ASTNFLAG | +IGNFLAG | - | - | - |
| `agc_task_cloktask_csm` (clock.c) | CLOKTASK | TIMRFLAG | - | - | - | task cloktask, job dyndisp, job clockjob |
| `agc_job_engine_enable_proceed` (clock.c) | V99P | ASTNFLAG IGNFLAG | +ASTNFLAG | - | - | task ignition |
| `agc_job_engine_fail_terminate` (clock.c) | V97T | - | - | - | - | task engine_fail_shutdown |
| `agc_task_engine_fail_shutdown_restart` (clock.c) | V97TRCS | - | - | - | - | task engine_fail_rcs_dap |
| `agc_task_engine_fail_rcs_dap` (clock.c) | V97TRCS | - | - | - | - | job engine_enable_terminate |
| `agc_job_engine_fail_proceed` (clock.c) | V97P | - | - | - | - | task engine_fail_resume |
| `agc_task_engine_fail_resume` (clock.c) | V97PTASK | - | +IDLEFAIL +STEERSW | - | - | task engine_fail_reenable |
| `agc_task_engine_fail_reenable` (clock.c) | R40ENABL | - | -IDLEFAIL | - | - | - |
| `agc_job_engine_fail_reignite` (clock.c) | V97E | - | - | - | - | task engine_fail_reignite_prep |
| `agc_task_spsoff97` (clock.c) | SPSOFF97 | - | - | - | - | task engine_fail_reignite_trim |
| `agc_task_engine_fail_reignite_trim` (clock.c) | V97E40.6 QUICKIGN | - | - | - | - | task gimbal_trim_restart |
| `agc_task_quickign` (clock.c) | QUICKIGN | - | -ASTNFLAG +IGNFLAG | - | - | task engine_enable_flash |
| `agc_task_engine_enable_flash` (clock.c) | V99FLASH | - | - | - | - | job timeburn |
| `agc_task_gimbal_test_trim` (clock.c) | S40.6 | - | - | - | - | task gimbal_test_relays |
| `agc_task_gimbal_test_relays` (clock.c) | S40.6 | - | - | - | - | task gimbal_test_rstrtst |
| `gimbal_output` (clock.c) | OUTPUT | - | - | - | - | task gimbal_test_step |
| `agc_task_gimbal_test_rstrtst` (clock.c) | RSTRTST | - | - | - | - | task gimbal_test_step |
| `agc_task_gimbal_test_step` (clock.c) | TEMPR60 | - | - | - | - | task gimbal_test_step |
| `agc_job_dapfig` (clock.c) | DAPFIG | - | -DAPBIT1 -DAPBIT2 | - | ch12 SIVBTAKEOVER | - |
| `agc_csm_preptvc` (ignition.c) | PREPTVC | - | -DAPBIT1 -DAPBIT2 | - | - | task dotvcon |
| `agc_task_ignition_csm` (ignition.c) | IGNITION | IMPULSW | +ENGONFLG -STRULLSW +STRULLSW | - | - | - |
| `agc_task_implcont` (ignition.c) | IMPLCONT | - | -IMPULSW | - | - | - |
| `agc_task_dotvcon` (ignition.c) | DOTVCON | - | +DAPBIT1 -DAPBIT2 | - | - | task dostrull |
| `agc_task_dostrull` (ignition.c) | DOSTRULL STEERULL ULAGEOFF | STRULLSW | +STEERSW | - | - | - |
| `agc_task_dospsoff` (ignition.c) | DOSPSOFF | - | - | - | - | task dotvcrcs |
| `agc_task_dotvcrcs` (ignition.c) | DOTVCRCS | - | - | - | - | job postburn |
| `agc_csm_spsoff` (ignition.c) | SPSOFF | - | -ENGONFLG | - | - | - |
| `agc_csm_tvczap` (ignition.c) | TVCZAP | - | -TIMRFLAG | - | - | - |
| `agc_job_sps_burn_rcs_trim` (ignition.c) | P40RCS | - | - | - | - | job sps_burn_min_deadband |
| `agc_job_thrust_monitor_csm` (ignition.c) | P47CSM | - | - | - | - | task tigon |
| `agc_task_tigon` (ignition.c) | TIGON | - | - | - | - | job thrust_monitor_reset |
| `short_burn_cutoff` (steer.c) | S40.81 | - | +IMPULSW -STEERSW | - | - | - |
| `lothrust` (steer.c) | LOTHRUST | IDLEFAIL | -STEERSW | - | - | - |
| `steering_after_vg_update` (steer.c) | S40.8 | STEERSW | - | ALARM 1407 | - | - |
| `setup_9` (steer.c) | SETUP.9 SURELY.9 | FIRSTFLG | - | - | - | job vtogain |
| `agc_csm_updatevg` (steer.c) | UPDATEVG | XDELVFLG | - | - | - | - |
| `steering_tail` (steer.c) | - | IMPULSW | - | - | - | task enginoff |
| `agc_job_steering_restart` (steer.c) | STEERING | - | -IMPULSW | - | - | - |
| `agc_job_vtogain_csm` (steer.c) | S40.9 | NORMSW | -GUESSW | - | - | - |
| `agc_job_endlamb` (steer.c) | ENDLAMB FIRSTTME | FIRSTFLG | - | - | - | - |
| `agc_job_copy40_9` (steer.c) | COPY40.9 ENDS40.9 | - | -FIRSTFLG | - | - | - |
| `agc_job_timeburn_csm` (steer.c) | S40.13 | NJETSFLG | - | - | ch31 PLUSX | - |
| `vg_to_lvc` (burn_steering.c) | AVFLAG MIDGIM | - | +AVFLAG | - | - | - |
| `initial_vg_lambert_lm` (burn_steering.c) | S40.1B | NORMSW | - | - | - | - |
| `agc_guid_initial_velocity_to_gain` (burn_steering.c) | S40.1 | XDELVFLG | - | - | - | - |
| `agc_guid_preferred_thrust_attitude_lm` (burn_steering.c) | - | - | +PFRATFLG | - | - | - |
| `agc_guid_timeburn_csm` (burn_steering.c) | S40.13 | - | +IMPULSW -IMPULSW | - | - | - |
| `agc_guid_midgim` (external_delta_v.c) | MIDGIM | AVFLAG | - | - | - | - |
| `agc_guid_again` (external_delta_v.c) | AGAIN | NORMSW | - | - | - | - |
| `agc_guid_lambert_aimpoint_vectors` (external_delta_v.c) | S31.1 | - | +AVFLAG | - | - | - |
| `ignition_time_entry` (external_delta_v.c) | P30 | - | +UPDATFLG +TRACKFLG | - | - | - |
| `agc_job_external_delta_v_compute` (external_delta_v.c) | P30 | - | -UPDATFLG +UPDATFLG | - | - | - |
| `agc_job_external_delta_v_accept` (external_delta_v.c) | PARAM30 PROCEED | - | +XDELVFLG | - | - | - |
| `agc_job_lambert_aimpoint_compute_csm` (external_delta_v.c) | P31 | - | -UPDATFLG -NORMSW | - | - | - |
| `burn_program_start` (dps_burn.c) | P40IN P40SXT4 | - | -3AXISFLG | - | - | - |
| `stage_alarm` (dps_burn.c) | P40ALM | - | - | ALARM 1706 | - | - |
| `agc_job_dps_burn` (dps_burn.c) | P40LM | APSFLAG CSMDKFLG | -NOTHROTL | - | - | - |
| `agc_job_aps_burn` (dps_burn.c) | P42LM P42STAGE | APSFLAG | +AVFLAG | - | - | - |
| `agc_job_rcs_burn_lm` (dps_burn.c) | P41LM | NJETSFLG | - | - | - | - |
| `agc_job_rcs_burn_vg_displayed` (dps_burn.c) | - | - | - | - | - | job dyndisp |
| `agc_job_redo6_7` (dps_burn.c) | REDO6.7 BLNKWAIT | - | - | - | - | job redo6_7 |
| `agc_job_dynmdisp` (dps_burn.c) | DYNMDISP | - | - | - | - | job dyndisp |
| `agc_job_thrust_monitor_lm` (dps_burn.c) | P47LM | - | - | - | - | task tigon |
| `agc_task_start_thrust_monitor` (dps_burn.c) | STARTP47 | - | - | - | - | job thrust_monitor_reset |
| `agc_task_implburn_lm` (dps_burn.c) | IMPLBURN | - | -IGNFLAG -ASTNFLAG -IMPULSW | - | - | task implburn2 |
| `agc_lm_enginof3` (dps_burn.c) | ENGINOF3 | - | -ENGONFLG +USEQRFLG | - | ch11 ENGINEOFF, ch11 ENGINEON, ch14 THRUSTACT | - |
| `agc_lm_enginof1` (dps_burn.c) | ENGINOF1 | - | +IDLEFLAG | - | - | - |
| `agc_lm_enginof2` (dps_burn.c) | ENGINOF2 | - | - | - | - | task coastset |
| `agc_lm_enginoff` (dps_burn.c) | ENGINOFF | - | - | - | - | job postburn |
| `agc_lm_cross_product_steering` (dps_burn.c) | S40.8 | STEERSW XDELVFLG | +IMPULSW -STEERSW | ALARM 1407 | - | - |
| `agc_lm_updatevg` (dps_burn.c) | UPDATEVG | NORMSW XDELVFLG | - | - | - | job vtogain |
| `agc_job_steering_lm` (dps_burn.c) | STEERING NSTEER | IDLEFLAG IMPULSW | -IMPULSW +IDLEFLAG | - | - | task enginoff |
| `agc_job_vtogain_lm` (dps_burn.c) | S40.9 | NORMSW | +AVFLAG | - | - | - |
| `timeburn_long` (dps_burn.c) | S40.13D | APSFLAG | -NOTHROTL +NOTHROTL | - | - | - |
| `agc_job_timeburn_lm` (dps_burn.c) | S40.13 | APSFLAG | -IMPULSW +NOTHROTL +IMPULSW | - | - | - |
| `agc_task_trimgimb` (dps_burn.c) | TRIMGIMB | - | -GMBDRVSW | - | - | task trimgimb2 |
| `agc_task_trimgimb2` (dps_burn.c) | TRIMGIMB | - | - | - | - | task pitchoff, task rolloff |
| `rollover` (dps_burn.c) | ROLLOVER | GMBDRVSW | +GMBDRVSW | - | - | job trimdone |
| `rs_vr` (dps_burn_rasteer.c) | - | NORMSW | - | - | - | - |
| `agc_lm_noullage` (burnbaby.c) | NOULLAGE ONULLAGE | - | -ULLAGFLG | - | - | - |
| `agc_lm_onullage` (burnbaby.c) | ONULLAGE | - | +ULLAGFLG | - | - | - |
| `ign_call_tig_35` (burnbaby.c) | CALLT-35 | - | - | - | - | task tig_35 |
| `agc_job_preburn_state_vector_lm` (burnbaby.c) | P41SPOT CALLT-35 | MUNFLAG | - | - | - | - |
| `agc_task_tig_35` (burnbaby.c) | TIG-35 | - | - | - | - | task tig_30, task restore_burn_display, job rcs_burn_blank_dsky |
| `agc_task_restore_burn_display` (burnbaby.c) | TIG-30.1 | - | - | - | - | job restore_burn_display |
| `agc_task_tig_30` (burnbaby.c) | TIG-30 | - | +APSFLAG | - | - | task tig_5, task ullgtask |
| `agc_task_tig_5_lm` (burnbaby.c) | TIG-5 | - | -IGNFLAG -ASTNFLAG | - | - | task tig_0, job timeburn |
| `agc_task_ignition_if_enabled` (burnbaby.c) | - | ASTNFLAG | - | - | - | - |
| `agc_task_tig_0_lm` (burnbaby.c) | TIG-0 | - | +IGNFLAG | - | - | task braking_phase_zoom |
| `ascent_ignition` (burnbaby.c) | ABRTIGN | - | +SWANDISP | - | - | - |
| `braking_phase_ignition` (burnbaby.c) | P63IGN | - | +LETABORT +SWANDISP -PULSEFLG | - | - | - |
| `dps_ignition` (burnbaby.c) | P40IGN | NOTHROTL | - | - | - | task dps_burn_zoom |
| `agc_task_ignition_lm` (burnbaby.c) | IGNITION | - | +ENGONFLG -FLUNDISP | - | ch11 ENGINEOFF, ch11 ENGINEON | - |
| `agc_task_ignition_finish` (burnbaby.c) | P42IGN | IMPULSW | -DRIFTDFL -IGNFLAG -ASTNFLAG -IDLEFLAG | - | - | task ullagoff |
| `agc_task_tigtask` (burnbaby.c) | TIGTASK TIGTASK1 | - | - | - | - | job tignow |
| `agc_task_dps_burn_zoom` (burnbaby.c) | P40ZOOM | - | - | - | ch14 THRUSTACT | - |
| `ign_start_clock` (clock.c) | STCLOK3 | - | - | - | - | task cloktask |
| `agc_task_cloktask_lm` (clock.c) | CLOKTASK | - | - | - | - | job clockjob, task cloktask |
| `agc_job_clokjob_event_timer_proceed` (clock.c) | ASTNRETN | - | - | - | - | job astnret |
| `agc_job_clokjob_engine_enable` (clock.c) | PROCEED IGNITE | IGNFLAG | +ASTNFLAG | - | - | task ignition |
| `go_postburn` (clock.c) | GOPOST | - | - | - | - | job postburn |
| `go_cutoff` (clock.c) | GOCUTOFF | - | -FLUNDISP | - | - | job cutoff |
| `agc_job_comfail` (clock.c) | COMFAIL | - | +IDLEFLAG +FLUNDISP | - | - | - |
| `agc_job_clokjob_engine_fail` (clock.c) | COMFAIL1 | - | -IDLEFLAG -FLUNDISP | - | - | - |
| `agc_job_clokjob_engine_fail_shutdown` (clock.c) | COMFAIL2 | AORBTFLG | +DRIFTDFL -AORBTFLG +AORBTFLG +ULLAGFLG | - | - | task tig_5 |
| `auto_control_check` (clock.c) | P40A/P | APSFLAG | - | - | ch30 AUTOTHROT | - |
| `agc_task_preread_lm` (average_g.c) | PREREAD | - | - | - | - | job lastbias |
| `agc_task_bibibias` (average_g.c) | BIBIBIAS | - | +SERVICER_RUNNING +AVEGFLAG -DRIFTFLG | - | - | job normlize, task readaccs |
| `redo5_5` (average_g.c) | REDO5.5 MAKEACCS AVEGOUT | AVEGFLAG MUNFLAG | - | - | ch11 TESTCONN | job servicer, task abort_monitor, task readaccs |
| `massmon` (average_g.c) | MASSMON | APSFLAG SURFFLAG | - | - | - | - |
| `getabval` (average_g.c) | GETABVAL AVERAGEG | MUNFLAG | - | - | - | - |
| `lothrust` (average_g.c) | LOTHRUST DECCNTR | - | +USEQRFLG | - | - | job comfail |
| `dvmon` (average_g.c) | DVMON | APSFLAG | +STEERSW +USEQRFLG -USEQRFLG | - | ch32 DPSGIMBDIS | - |
| `agc_lm_serv_copycycl` (average_g.c) | COPYCYCL | AUXFLAG IDLEFLAG | -STEERSW -AUXFLAG +USEQRFLG +AUXFLAG | - | - | - |
| `agc_job_serv_resume_lm` (average_g.c) | QUIKFAZ5 | - | +USEQRFLG | - | - | - |
| `agc_job_avgend_lm` (average_g.c) | AVGEND | - | +DRIFTFLG | - | ch11 TESTCONN | - |
| `agc_job_avetomid_lm` (average_g.c) | AVGEND | - | +NO_POWERED_DESIGNATE -SWANDISP -MUNFLAG -SERVICER_RUNNING | - | - | - |
| `agc_lm_servidle` (average_g.c) | SERVIDLE | MUNFLAG | +IDLEFLAG | - | - | - |
| `agc_job_normlize_lm` (average_g.c) | NORMLIZE | MUNFLAG | - | - | - | - |
| `agc_lm_serv_contserv` (landing_radar.c) | CONTSERV | - | -RNGEDATA -VELDATA | - | - | - |
| `higatask` (landing_radar.c) | HIGATASK | - | +PSTHIGAT +NOLRREAD | - | - | job higatjob |
| `agc_lm_serv_munretrn` (landing_radar.c) | MUNRETRN | LRBYPASS PSTHIGAT READLR XORFLG | +READLR +XOVINFLG +XORFLG | ALARM 0511 | ch12 LRPOS2CMD, ch33 LRPOS1, ch33 LRPOS2 | - |
| `agc_lm_serv_copycyc1` (landing_radar.c) | COPYCYC1 R29 | NO_POWERED_DESIGNATE READRFLG | -DESIGFLG | - | - | - |
| `hfail` (landing_radar.c) | HFAIL | - | +HFLSHFLG | - | - | - |
| `agc_lm_serv_updatchk` (landing_radar.c) | UPDATCHK POSUPDAT NOREASON | ALTSCALE LRINH NOLRREAD PSTHIGAT RNGEDATA | -HFLSHFLG | - | - | - |
| `vfail` (landing_radar.c) | VFAIL NOLITE | - | +VFLSHFLG +VXINH | - | - | - |
| `agc_lm_serv_vmeaschk` (landing_radar.c) | VMEASCHK VELUPDAT VUPDAT | LRINH VELDATA VXINH | -VFLSHFLG -VXINH | - | - | - |
| `agc_lm_serv_valtchk` (landing_radar.c) | VALTCHK READV | READVEL | +READVEL | - | - | job lrvjob |
| `agc_lm_lrhtask` (landing_radar.c) | LRHTASK | LRBYPASS NOLRREAD READLR | - | - | - | job lrhjob |
| `agc_job_lrvjob` (landing_radar.c) | LRVJOB | - | - | - | - | job lrvjob, task rdgims |
| `agc_job_lrvjob_done` (landing_radar.c) | LRVJOB | - | +VELDATA | - | - | - |
| `agc_job_lrhjob_done` (landing_radar.c) | LRHJOB | RNGSCFLG | -RNGSCFLG +RNGEDATA | - | - | - |
| `agc_job_higatjob` (landing_radar.c) | HIGATJOB | - | - | - | - | job higatjob |
| `posgood` (landing_radar.c) | POSGOOD | - | -NOLRREAD | - | - | - |
| `posalarm` (landing_radar.c) | POSALARM | - | - | PRIOLARM 0523 | - | - |
| `agc_job_posalarm_pro` (landing_radar.c) | POSALARM | - | - | - | ch33 LRPOS2 | - |

### Shared erasables read and written

Accessors of `include/agc/core/state.h` and the lanes' public headers called by this lane, with the number of its functions that read / write each. Lane-private erasables are `static` in the lane's files or in its state struct; their scales are on the declarations.

| erasable (accessor) | meaning and scale (accessor prolog) | read by | written by |
|---|---|---|---|
| `abdelv` | /DELV/ of the last PIPA reading, cm/s, scale 2^14 (ABDELV, LM) | 2 | 2 |
| `aig` | none (AIG erasable, CM_ENTRY_DIGITAL_AUTOPILOT.agc p. 1063) -- accessor for other lanes.; returns agc_sp_t -- agc_sp_t -- AIG inner gimbal angle /180, | 1 | 0 |
| `amg` | none (AMG erasable, CM_ENTRY_DIGITAL_AUTOPILOT.agc p. 1063) -- accessor for other lanes.; returns agc_sp_t -- agc_sp_t -- AMG middle gimbal angle /180 | 1 | 0 |
| `aog` | none (AOG erasable, CM_ENTRY_DIGITAL_AUTOPILOT.agc p. 1063) -- accessor for other lanes.; returns agc_sp_t -- agc_sp_t -- AOG outer gimbal angle /180, | 1 | 0 |
| `aosq` | AOSQ/AOSR are DP word pairs at pi/2 (the minor word may disagree in sign after DAS, as in the AGC); the accessor returns their value. AGC: none (helpe | 1 | 1 |
| `aosr` | none -- accessor of the DAP erasable AOSR (ERASABLE_ASSIGNMENTS.agc).; returns agc_dp_t -- the erasable, at the scale given in agc/lm/dap.h | 1 | 1 |
| `cdux` | CDU X (outer gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (register CDUX, 032) | 4 | 0 |
| `cduy` | CDU Y (inner gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUY) | 3 | 0 |
| `cduz` | CDU Z (middle gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUZ) | 3 | 0 |
| `cm_gymdt` | P40TMP (CSM, DP cs), SAVET-30 (LM, DP cs), ZOOMTIME (LM pad load, cs), S61DT and CM/GYMDT (CSM entry, cs): read by the restart tables. | 0 | 1 |
| `csmmass` | CSM mass, kg, scale 2^16 (CSMMASS; TVCMASSPROP 'B+16 KILOGRAMS'; noun 47 R1) | 3 | 1 |
| `dapdatr1` | DAP configuration word 1 (DAPDATR1, DSP noun 46 R1) | 2 | 0 |
| `delv` | accumulated PIPA delta-V, stable-member axes (DELV, I(6)): PIPA pulses scale 2^14 (count in the high word); 1 pulse = 5.85 cm/s CSM, 1 cm/s LM. KPIP1  | 8 | 2 |
| `geomsgn` | none (accessor of erasable GEOMSGN, CONIC_SUBROUTINES.agc p. 1306) -- GEOMSGN as LAMBERT left it; returns agc_sp_t -- GEOMSGN, +.5 short way / -.5 lon | 1 | 0 |
| `mass` | LM vehicle mass, kg, scale 2^16, DP (MASS: SERVICER MASSMON, 1/ACCS) | 3 | 2 |
| `modecadr` | C(MODECADR +dev) (ERASABLE_ASSIGNMENTS.agc "MODE SWITCHING ERASABLE").; returns agc_sp_t -- the word (+0 none, positive a waiting job) | 2 | 0 |
| `modreg` | MODREG major mode (program) | 4 | 0 |
| `mrkbuf2` | MRKBUF1 / MRKBUF2 (CM): two seven-word mark buffers (time DP, CDUY, word 0 = -1 (77776) marks a buffer empty. mrkbuf2 / set_mrkbuf2 read and store MRK | 0 | 1 |
| `optind` | OPTIND (ERASABLE_ASSIGNMENTS.agc, CM) -- coarse optics state, read RETURNS: OPTIND: -1, 0, 1, or AGC_OPTIND_NEG0 for -0; returns OPTIND: -1, 0, 1, or  | 0 | 2 |
| `p40tmp` | P40TMP (CSM, DP cs), SAVET-30 (LM, DP cs), ZOOMTIME (LM pad load, cs), S61DT and CM/GYMDT (CSM entry, cs): read by the restart tables. | 1 | 2 |
| `pipax` | PIPA X pulse count since last read (register PIPAX, 037), unscaled pulses: 1 pulse = 5.85 cm/s (CSM SERVICER207), 1 cm/s (LM) | 2 | 0 |
| `pipay` | PIPA Y pulse count (register PIPAY) | 2 | 0 |
| `pipaz` | PIPA Z pulse count (register PIPAZ) | 2 | 0 |
| `piptime` | time of the PIPA reading / this vehicle's state vector, centiseconds, scale 2^28 (PIPTIME, B(2)PRM) | 12 | 1 |
| `refsmmat` | reference-to-stable-member matrix, row-major, DP elements, scale 2^1 (REFSMMAT, I(18D)PRM; 'REFSMMAT SCALE OF B1') | 12 | 0 |
| `rn` | this vehicle's position, meters, scale 2^29 (earth) or 2^27 (moon) (RN, B(6)PRM) | 6 | 1 |
| `savet_30` | P40TMP (CSM, DP cs), SAVET-30 (LM, DP cs), ZOOMTIME (LM pad load, cs), S61DT and CM/GYMDT (CSM entry, cs): read by the restart tables. | 0 | 2 |
| `tevent` | time of event for downlist, centiseconds, scale 2^28 (TEVENT; TLIFTOFF in P11) | 0 | 4 |
| `tgo` | time to go in the burn / guidance, centiseconds, scale 2^28 (TGO) | 3 | 7 |
| `thrust` | THRUST (LM 055): descent engine throttle command pulses, SP. | 0 | 2 |
| `tig` | time of ignition, centiseconds, scale 2^28 (TIG; DSP noun 33) | 15 | 8 |
| `trkmkcnt` | TRKMKCNT: rendezvous tracking mark count (noun 45 R1), whole number. | 0 | 2 |
| `ttogo` | time to go, centiseconds, scale 2^28 (TTOGO; DSP nouns 35,40,45,59,99) | 0 | 2 |
| `tvcpitch` | CDUTCMD (053) / CDUSCMD (054): optics (CSM) or RR (LM) CDU error counter drive, SP pulse counts; which 0 = CDUTCMD, 1 = CDUSCMD. In TVC mode the same  | 0 | 2 |
| `tvcyaw` | CDUTCMD (053) / CDUSCMD (054): optics (CSM) or RR (LM) CDU error counter drive, SP pulse counts; which 0 = CDUTCMD, 1 = CDUSCMD. In TVC mode the same  | 0 | 2 |
| `un` | none (accessor of erasable UN, CONIC_SUBROUTINES.agc p. 1306) -- UN as LAMBERT left it (LM RASTEER reads it after INITVEL/LAMBERT); returns agc_vec_t  | 1 | 0 |
| `vgtig` | velocity to be gained at TIG, m/cs, scale 2^7 (VGTIG, P40-P47: 'VG IGNITION SCALED AT 2(+7) M/CS') | 11 | 6 |
| `vhfcnt` | VHFCNT (CM): VHF range mark count. AGC: VHFCNT (ERASABLE_ASSIGNMENTS.agc); returns agc_sp_t -- VHFCNT, a count | 0 | 1 |
| `vn` | this vehicle's velocity, m/cs, scale 2^7 (earth) or 2^5 (moon) (VN, B(6)PRM) | 2 | 1 |
| `zoomtime` | P40TMP (CSM, DP cs), SAVET-30 (LM, DP cs), ZOOMTIME (LM pad load, cs), S61DT and CM/GYMDT (CSM entry, cs): read by the restart tables. | 2 | 0 |

### AGC labels -> C functions

The "AGC" column above names the label each listed function translates; the complete label -> function -> file:line index for the lane (and the tree) is [docs/overview/xref.md](../overview/xref.md), generated by `tools/xref.py`.
