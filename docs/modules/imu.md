# imu -- IMU moding, compensation, alignment, attitude maneuvers

## Scope
AGC sources (CM = Comanche055, LM = Luminary099):
* IMU_MODE_SWITCHING_ROUTINES.agc (CM pp.1420-1448, LM pp.1309-1337) -> src/imu/mode.c
* IMU_COMPENSATION_PACKAGE.agc (CM pp.297-306, LM pp.326-337) -> src/imu/compensation.c
* INFLIGHT_ALIGNMENT_ROUTINES.agc (CM pp.1355-1363, LM same text) + CALCSMSC
  (CM P51-P53.agc p.763, LM POWERED_FLIGHT_SUBROUTINES.agc p.1265 FLESHPOT) -> src/imu/alignment.c
* GIMBAL_LOCK_AVOIDANCE.agc, KALCMANU_STEERING.agc, ANGLFIND.agc (CM) /
  ATTITUDE_MANEUVER_ROUTINE.agc (LM) -> src/imu/maneuver.c
* R60_62.agc (both) -> src/imu/attitude_maneuver.c
* IMU_CALIBRATION_AND_ALIGNMENT.agc (CM only, pp.423-455) -> src/csm/prelaunch_align.c
* Public header include/agc/imu/imu.h; pending externals in src/imu/pending.h.

Note: IMUMON / SETISSW / PIPFAIL / GLOCKMON live in T4RUPT_PROGRAM.agc, which is
sys-misc's file. Not duplicated here (see Open questions).

## Design table
| AGC label (file, page) | Inputs (scale) | Outputs (scale) | C function |
|---|---|---|---|
| ZEROICDU (IMS p.1420) | - | CDUX/Y/Z = 0 | static zero_icdu |
| IMUZERO/IMUZERO2/IMUZERO3 (p.1421-1422) | IMODES30, DSPTAB+11 | chan 12, waitlist | agc_imu_zero, agc_task_imuzero2, agc_task_imuzero3 |
| IMUCOARS/COARS/COARS1/COARS2/CHKCORS (p.1423-1425) | THETAD (2s-comp CDU, half-rev) CDUX/Y/Z | CDUXCMD.., chan 14 | agc_imu_coarse, agc_task_coars, agc_task_coars2, agc_task_chkcors |
| SETCOARS/RNDREFDR (p.1426) | chan 12 | flags, chan 12/14 | static set_coars, agc_imu_rndrefdr |
| IMUFINE/IMUFINED/IFAILOK/PFAILOK (p.1427-1428) | | | agc_imu_fine, agc_task_imufined, agc_task_ifailok, agc_task_pfailok |
| NOATTOFF (p.1428) | DSPTAB+11 | | static noatt_off |
| PIPUSE/PIPUSE1/PIPFREE (p.1429) | IMODES30 | | agc_imu_pipuse, agc_imu_pipuse1, agc_imu_pipfree |
| IMUPULSE/GYROAGRE/GYROBUSY/GWAKE (p.1430-1431) | gyro buffer DP rev (2^0, 2^21 pulses/rev) | LGYRO | agc_imu_pulse |
| STRTGYRO/STRTGYR2/GSELECT/MAJ+/MIN+/MAJ-/MIN-/GMERGE/LASTSEG/LONGGYRO/8192AUG (p.1432-1437) | buffer | GYROCMD (pulses), chan 14 | agc_task_strtgyro, agc_task_8192aug |
| ENDIMU/GOODEND/BADEND/CAGETEST/CAGETSTQ/CAGETSTJ (p.1438-1439) | MODECADR | job wake | agc_imu_endimu, agc_imu_goodend, agc_imu_badend |
| STALL/IMUSTALL/OPTSTALL/RADSTALL (p.1440-1441) | MODECADR | | agc_imu_stall |
| GOMANUR (p.1442) | ATTCADR | KALCMAN3 job | agc_imu_gomanur |
| R02BOTH (p.1443-1444) | REFSMFLG, IMODES30 | alarm 210/220 | agc_imu_status_check |
| P06/PRESTAND/SCALPREP/POSTAND (p.1445-1448) | TIME1/2, scaler | | agc_imu_standby (stub), agc_imu_postand (stub) |
| 1/PIPA (ICP p.326) | DELV (pulses 2^14), PIPABIAS (pulses/cs 2^-5 LM, 2^-8 CM), PIPASCF (ppm 2^-9), 1/PIPADT (cs 2^8) | DELV compensated | agc_imu_comp_pipa |
| IRIGCOMP/IRIGX/Y/Z/GCOMPSUB/DRIFTSUB/DRFTSUB2 (p.328-332) | DELV, ADIA/ADSRA (gyro pulses/pipa pulse 2^-6 LM, 2^-3 CM), NBD (pulses/cs 2^-5) | GCOMP (gyro pulses 2^14), GCOMPSW | agc_imu_comp_irig |
| 1/GYRO/GCOMP1 (p.333) | GCOMP | IMUPULSE | agc_job_gyro_command_pulse, agc_job_gcomp1 |
| NBDONLY/FBIASSUB/LASTBIAS/GCOMPZER (p.334-337) | TIME1 | GCOMP | agc_job_nbdonly, agc_job_lastbias, agc_imu_comp_zero |
| CALCGTA (IAR p.1355-1356) | XDC/YDC/ZDC half-unit | IGC/MGC/OGC DP rev | agc_imu_calcgta |
| ARCTRIG (p.1357) | sin/4, cos/4 | angle rev [-.5,.5) | agc_imu_arctrig |
| CALCGA/GIMLOCK1 (p.1359-1360) | XNB.. XSM.. half-unit | THETAD SP 2s-comp, alarm 401 | agc_imu_calcga |
| AXISGEN (p.1361-1362) | star vectors half-unit | XDC/YDC/ZDC half-unit | agc_imu_axisgen |
| CALCSMSC (P51-P53 p.763 / FLESHPOT) | SINCDU/COSCDU (scale 2) | XNB/YNB/ZNB half-unit | agc_imu_calcsmsc |
| CDULOGIC / 1STO2S helpers (RT8 p.1509) | | | agc_imu_cdu_to_rev / agc_imu_rev_to_cdu |
| KALCMAN3/SECAD/CHECKMAX/ALTCALC/COFMAXGO/METHOD1-3 (ANGLFIND p.399-403; AMR p.351-355) | CPHI.. SP 2s-comp, CDU | COF half-unit, AM rev | agc_kalc_compute (+helpers) |
| MXM3/TRANSPOS/TRNSPSPD (p.404-405) | | | agc_mat_mul / agc_mat_transpose (interp) |
| READCDUK/CDUTODCM (p.405-407) | CDU | matrix half-scale | agc_kalc_cdutodcm |
| DELCOMP (p.407-409) | COF half-unit, A rev | DEL scale 1 | agc_kalc_delcomp |
| DCMTOCDU (p.409-410) | matrix half | angles DP rev | agc_kalc_dcmtocdu |
| WCALC/NOGIMLOC (GLA p.412 / p.364) | RATEINDX, ARATE | BRATE, TM, BIASTEMP | static wcalc |
| NEWDELHI/NEWANGL/INCRDCDU/TMANUCHK/TIMECHK/MANUSTAT/CONTMANU/UPDTCALL (KS p.414-417 / p.365-368) | | CDUXD, DELCDU | agc_job_newdelhi, agc_task_updtcall, ... |
| MANUSTOP/ENDROLL/ENDMANU/GOODMANU/STOPRATE/STOPYZ/ZEROERROR/LOADCDUD/NOGO/TOOBADF/TOOBADI | | | agc_task_manustop, agc_task_endmanu, ... |
| R60CSM/R60LEM/R61TEST/CHKLINUS/RELINUS (R60 p.390-392) | | displays | agc_job_attitude_maneuver ... |
| VECPOINT/VECPNT1/PICKAXIS/ROT180 (p.393-396) | SCAXIS, POINTVSM half-unit | CPHI.. SP | agc_imu_vecpoint |
| BALLANGS (LM) | CPHI.. | FDAIX.. | agc_imu_ballangs |
| R62DISP/GOMOVE (p.398) | | | agc_job_crew_defined_maneuver_display |
| prelaunch: POSN17C, ERTHRVSE, EARTHR, ALWAYSG/COMPGS, RESTAIER, TAR/EREF, CHKCOMED (pp.430-450) | LATITUDE, AZIMUTH rev | ERCOMP1 rev | agc_csm_prelaunch_* |

## Progress
(updated as files complete)
* compensation.c: DONE (1/PIPA, IRIGCOMP, GCOMPZER, 1/GYRO, GCOMP1, NBDONLY, LASTBIAS) src/imu/compensation.c; compiles
* alignment.c: DONE (ARCTRIG, CALCGTA, CALCGA, AXISGEN, CALCSMSC) src/imu/alignment.c
* mode.c: DONE (ZEROICDU, IMUZERO/2/3, IMUCOARS/COARS/COARS2/CHKCORS, CA+ECE, SETCOARS, RNDREFDR,
  IMUFINE/IMUFIN20/IMUFINED/IFAILOK/PFAILOK, NOATTOFF, PIPUSE/PIPUSE1/PIPFREE, GOODEND/BADEND/ENDIMU/IMUBAD,
  STALL, R02BOTH/R02ZERO; P06/POSTAND stubbed) src/imu/mode.c
* gyro.c: DONE (IMUPULSE/GYROBUSY/GWAKE, STRTGYRO/STRTGYR2/GSELECT/MAJ/MIN/GMERGE/LASTSEG/LONGGYRO/8192AUG)
* imu_vflags*.c: DONE (vehicle-specific flag ids: CPHIFLAG, AUTR1/2FLG, NEEDLFLG, R60FLAG)
* maneuver_calc.c: DONE (CDUTODCM, DCMTOCDU, DELCOMP, KALCMAN3/SECAD/CHECKMAX/ALTCALC/COFMAXGO/METHOD1-3, TOOBADF test)
* maneuver.c: DONE (GOMANUR, KALCMAN3 job, WCALC/NOGIMLOC, NEWANGL, MANUSTAT/RATEBIAS, INCRDCDU, TIMECHK/MANUSTAL,
  CONTMANU, UPDTCALL, NEWDELHI, NOGO/NOGOM2, MANUSTOP, ENDMANU/GOODMANU, STOPRATE/STOPYZ, ZEROERROR, LOADCDUD/LOADYZ)
* attitude_maneuver.c: DONE (VECPOINT/VECPNT1/COMPMATX/PICKAXIS/ROT180, BALLANGS, G+N,AUTO, ISITAUTO, R60CSM/R60LEM,
  REDOMANN/REDOMANC/TOBALL(A)/AUTOMANV/ENDMANUV/ENDMANU1, R61TEST, CHKLINUS, RELINUS, R62DISP/GOMOVE)
* prelaunch_align.c: PARTIAL -- DONE FINETIME, POSN17C, ERTHRVSE, EARTHR/OVERFFIX; stubs: IMUTEST, ESTIMS, ALWAYSG, RESTAIER, CHKCOMED, TAR/EREF, GCOMPVER, OPTDATA, LATAZCHK, AZMTHCG1, TARGDRVE
* tests/imu/imu.c: DONE (65 checks pass)

## Files
src/imu/mode.c (moding, STALL/GOODEND/BADEND, R02, P06 stub), src/imu/gyro.c (IMUPULSE and
torquing tasks), src/imu/compensation.c, src/imu/alignment.c, src/imu/maneuver_calc.c (KALCMANU mathematics),
src/imu/maneuver.c (GOMANUR/KALCMAN3/steering), src/imu/attitude_maneuver.c (VECPOINT, BALLANGS, R60, R62),
src/imu/imu_vflags*.c (vehicle-specific flag ids, rendz pattern), src/imu/internal.h (lane-private),
src/imu/pending.h (assumed external interfaces), src/csm/prelaunch_align.c, include/agc/imu/imu.h,
tests/imu/imu.c.

## Deviations
* MODECADR -0 (complete, failed) is stored as -2 in exec's MODECADR word (agc_sp_t has no -0).
  Encoding: +0 free, -1 good, -2 failed, positive = id of the stalled job. GOODEND's "advance LOC if
  good" becomes agc_imu_stall_outcome(dev), read by the woken job.
* GOODEND with MODECADR already negative: Luminary resets it to +0; Comanche would JOBWAKE the negative
  word. Both vehicles use the Luminary behaviour.
* IMUPULSE: the AGC keeps the 8192-pulse augment count in the major word of the command buffer while
  torquing; here it is a module variable (aug_left) and the buffer holds only the final fraction.
  GYROBUSY/GWAKE: the sleeping job is woken as wake_as and must call agc_imu_pulse_buf again.
* Coarse align COMMAND/CDUIND overlay GCOMP in the AGC (COMMAND EQUALS GCOMP); here they are separate,
  so coarse align no longer clobbers GCOMP.
* MODEEXIT releases interrupts only if inhibited (IMUPULSE reaches it without INHINT; C inhibit is counted).
* CONTMANU: NEXTIME == TIME1 would give WAITLIST a 0 delay; 1 cs is used.
* FINETIME (CM prelaunch): the re-read loop is bounded (4 tries).
* CALCGA: the AGC also overwrites the OGC/IGC/MGC words (the IMUPULSE buffer) with the CDU angles;
  agc_imu_calcga does not touch the gyro buffer. CALCGTA does load it (as the header says).
* Comanche MINANG/MAXANG are single-word DEC constants read by DSU as DP, so their low words are the
  next constants (MAXANG, SD); modelled exactly (maneuver_calc.c). LM LOCKANGL likewise reads the next
  instruction word (CA CDUZ, 030034) as its low half.
* KALCMANU, CM: RATEINDX is read from / written to the CSM DAP struct (agc_csm_dap_rcs()->ratindx),
  since dap-csm models it there; LM keeps it module-private.
* CSM CDUXD ("TS CDUXD") writes the high word of the DAP's DP cdud[] with the sign-extended
  two's-complement CDU pattern, keeping the low word.
* ENDMANU1 runs the return continuation (TEMPR60) in the same job via agc_dispatch_job (BANKJUMP).

## Stubs
* agc_imu_standby, agc_imu_postand (P06 standby / POSTAND time recovery: needs scaler snapshot, GOPERF1
  loop, restart to POSTAND).
* src/csm/prelaunch_align.c: IMUTEST, ESTIMS (gyrocompass/drift-test filter loop), ALWAYSG/COMPGS,
  RESTAIER, CHKCOMED, TAR/EREF, GCOMPVER, OPTDATA, LATAZCHK, AZMTHCG1, TARGDRVE (agc_csm_prelaunch_*).
  Translated there: FINETIME, POSN17C, ERTHRVSE, EARTHR/OVERFFIX.

## Requested core additions
Everything below is declared in src/imu/pending.h until the owner publishes it:
* sys-misc (include/agc/io/service_internal.h, not public): agc_rtb_cdulogic, agc_rtb_1sto2s, agc_rtb_v1sto2s,
  agc_rtb_signmpac, agc_rtb_loadtime, agc_t4_setissw -- same signatures as sysmisc.h; please make public.
* guid-powered: `agc_status_t agc_pfs_pipasr_noscale(void);` (PIPASR +3, used by LM NBDONLY on surface).
* prog-rendz: `agc_status_t agc_rendz_goto_terminate_tracking(void);` (R61TEST).
* dap-lm: `agc_sp_t agc_erase_1jaccq(void); agc_sp_t agc_erase_1jaccr(void);` (MANUSTAT RATEBIAS).
* sys-misc/prog-rendz (CM): `void agc_erase_set_optind(agc_sp_t v);` (RELINUS).
* exec: restart-table entries CSM 1.7 / LM 2.13 -> AGC_JOB_RELINUS (exec_pending.h already has it);
  CSM 1.11 (PIKUP20), LM 2.7 / 2.11 are prog-rendz's.
* exec-sched bug seen while testing: executive.c activity_light() passes (uint16_t)~BIT2 = 0177775 to
  agc_chan_and, which asserts mask <= 077777.
Other lanes asked the imu lane for: ZEROICDU, NOATTOFF, SETCOARS, IMUBAD, clear_cmds, MODECADR
(sys-misc) -> provided (agc_imu_*; MODECADR now exec's); R02ZERO, IMUFIN20 (prog-align) -> provided.
Signature mismatches in other lanes' shims (they should adopt agc_imu.h): align_pending.h
agc_imu_stall(cont) / agc_imu_optstall / agc_imu_goodend(void) / agc_imu_curtains / agc_imu_ballangs(gimbal)
/ agc_imu_attitude_maneuver(cont); landing_pending.h agc_imu_fine_align_recheck (R51 is not in this lane's files).

## Requested jobs-tasks
Jobs (provisional ids in pending.h): AGC_JOB_GYRO_COMMAND_PULSE -> agc_job_gyro_command_pulse; AGC_JOB_GYRO_COMMAND_RETRY ->
agc_job_gyro_command_retry; AGC_JOB_GCOMP1 -> agc_job_gcomp1; AGC_JOB_KALCMAN3 -> agc_job_kalcman3;
AGC_JOB_NEWDELHI -> agc_job_newdelhi (FINDVAC PRIO26); AGC_JOB_REDOMANC -> agc_job_redomanc;
AGC_JOB_ATTITUDE_MANEUVER_ENTER -> agc_job_attitude_maneuver_enter; AGC_JOB_ATTITUDE_MANEUVER_TERMINATE -> agc_job_attitude_maneuver_terminate; AGC_JOB_ENDMANUV ->
agc_job_endmanuv; AGC_JOB_RELINUS -> agc_job_relinus; AGC_JOB_CREW_DEFINED_MANEUVER_DISPLAY -> agc_job_crew_defined_maneuver_display;
AGC_JOB_CREW_DEFINED_MANEUVER_GOMOVE -> agc_job_crew_defined_maneuver_gomove; AGC_JOB_CREW_DEFINED_MANEUVER_TERMINATE -> agc_job_crew_defined_maneuver_terminate.
NBDONLY / LASTBIAS jobs: agc_job_nbdonly, agc_job_lastbias (started by exec SVCT3 / programs).
Tasks: AGC_TASK_IMUZERO2/3, COARS, COARS2, CHKCORS, IMUFINED, IFAILOK, PFAILOK, STRTGYRO, 8192AUG,
UPDTCALL, MANUSTOP, ENDMANU (= GOODMANU) -> agc_task_<name>; sys-misc's AGC_TASK_CA_ECE ->
agc_task_ca_ece.

## Tests written
tests/imu/imu.c (CSM): ARCTRIG quadrants; CALCGTA identity -> zero gyro angles and OGC buffer;
CALCGA 30 deg MGA (no lock, THETAD stored) and 75 deg (alarm 401, GLOKFAIL); 1/PIPA with zero
bias/scale is identity and leaves GCOMP zero; CDUTODCM/DCMTOCDU round trip; DELCOMP(0) = I;
IMUPULSE/STRTGYRO pulse count, unsent fraction and release; KALCMANU set-up (normal and small);
VECPOINT 30 deg pitch.  65 checks pass when linked against core + exec + imu (other lanes' missing
symbols stubbed in a scratch file, not committed).

## Open questions
* IMUMON and the ISS fail monitors are in T4RUPT_PROGRAM.agc (sys-misc lane), not in
  IMU_MODE_SWITCHING_ROUTINES.agc as the assignment states.
* CALCSMSC is also implemented by prog-align (agc_align_calcsmsc); one should go.
* GYROCMD and CDUXCMD..CDUZCMD are hardware counters: the sim must drain them
  (agc_imu_hw_set_gyrocmd / agc_imu_hw_set_cducmd) or 8192AUG will see them grow.
* The LM DAP header names the KALCMANU increments DELCDUX..Z (LM erasable DELDCDU = DELCDUX); written
  as the negative 0.1 s increment, as the comment there says.
