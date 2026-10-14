# imu -- IMU moding, compensation, alignment, attitude maneuvers

History of this lane: [docs/history/lanes/imu.md](../history/lanes/imu.md)

## Scope
AGC sources (CM = Comanche055, LM = Luminary099):
* IMU_MODE_SWITCHING_ROUTINES.agc (CM pp.1420-1448, LM pp.1309-1337) -> src/imu/mode.c
* IMU_COMPENSATION_PACKAGE.agc (CM pp.297-306, LM pp.326-337) -> src/imu/compensation.c
* INFLIGHT_ALIGNMENT_ROUTINES.agc (CM pp.1355-1363, LM same text) + CALCSMSC
  (CM P51-P53.agc p.763, LM POWERED_FLIGHT_SUBROUTINES.agc p.1265 FLESHPOT) -> src/imu/alignment.c
* GIMBAL_LOCK_AVOIDANCE.agc, KALCMANU_STEERING.agc, ANGLFIND.agc (CM) /
  ATTITUDE_MANEUVER_ROUTINE.agc (LM) -> src/imu/maneuver.c
* R60_62.agc (both) -> src/imu/attitude_maneuver.c
* IMU_CALIBRATION_AND_ALIGNMENT.agc (CM only, pp.423-455) -> src/csm/prelaunch/align.c
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

## Files
src/imu/mode.c (moding, STALL/GOODEND/BADEND, R02, P06 stub), src/imu/gyro.c (IMUPULSE and
torquing tasks), src/imu/compensation.c, src/imu/alignment.c, src/imu/maneuver_calc.c (KALCMANU mathematics),
src/imu/maneuver.c (GOMANUR/KALCMAN3/steering), src/imu/attitude_maneuver.c (VECPOINT, BALLANGS, R60, R62),
src/imu/imu_vflags*.c (vehicle-specific flag ids, rendz pattern), src/imu/internal.h (lane-private),
src/imu/pending.h (assumed external interfaces), src/csm/prelaunch/align.c, include/agc/imu/imu.h,
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
* src/csm/prelaunch/align.c: IMUTEST, ESTIMS (gyrocompass/drift-test filter loop), ALWAYSG/COMPGS,
  RESTAIER, CHKCOMED, TAR/EREF, GCOMPVER, OPTDATA, LATAZCHK, AZMTHCG1, TARGDRVE (agc_csm_prelaunch_*).
  Translated there: FINETIME, POSN17C, ERTHRVSE, EARTHR/OVERFFIX.

## Tests written
tests/imu/imu.c (CSM): ARCTRIG quadrants; CALCGTA identity -> zero gyro angles and OGC buffer;
CALCGA 30 deg MGA (no lock, THETAD stored) and 75 deg (alarm 401, GLOKFAIL); 1/PIPA with zero
bias/scale is identity and leaves GCOMP zero; CDUTODCM/DCMTOCDU round trip; DELCOMP(0) = I;
IMUPULSE/STRTGYRO pulse count, unsent fraction and release; KALCMANU set-up (normal and small);
VECPOINT 30 deg pitch.  65 checks pass when linked against core + exec + imu (other lanes' missing
symbols stubbed in a scratch file, not committed).

## Program flow and decisions

_Documentation wave (2026-10-12); prose and tables brought to the final tree (HEAD 3b97dc8) by the final pass (2026-10-14). The prose was written by reading the code; the two tables were extracted from the function bodies (flag, alarm, channel and scheduling calls; `agc_erase_*` accessors) and list only what the code does. Files: `src/imu/*.c`, `src/csm/prelaunch/*.c`._

IMU mode switching, gyro torquing, compensation, KALCMANU and the CSM
prelaunch programs P01-P03. The realign that uses them is drawn in
[flowcharts/imu-realign.md](../overview/flowcharts/imu-realign.md).

* **Mode switches** (`src/imu/mode.c`): each switch sets the channel 12/14
  discretes, schedules its tasks and returns; the last task ends the
  operation with GOODEND or BADEND (`goodend_common`), which wakes the job
  stalled on the device (`agc_imu_stall`, `agc_imu_stall_dev`: MODECADR
  +0 = still running, the job sleeps; -1 good; -0 bad; a second user of
  one device is MODABORT: POODOO 01210 CSM / BAILOUT 01210 LM, `modabort`).
  - Zero (`agc_imu_zero`): refused with ALARM 00206 in coarse align with
    gimbal lock, 00210 when the IMU is not operating; ZEROICDU on channel 12
    then the release and done tasks (`agc_task_imu_zero_release`,
    `agc_task_imu_zero_done`).
  - Coarse align (`agc_imu_coarse` to THETAD): CDU drive pulses per task
    (`agc_task_coarse_align_pulses`, channel 14 DRIVECDUX/Y/Z), then
    `agc_task_coarse_align_check` (CHKCORS): a gimbal more than COARSTOL
    from its target is ALARM 00211 and IMUBAD.
  - Fine align (`agc_imu_fine`): zero and coarse off, NO ATT off; IFAILOK
    5.12 s later (`agc_task_ifailok`).
  - PIPAs: `agc_imu_pipuse` sets PIPAX..Z = -0; `agc_imu_pipfree` raises
    ALARM 00212 if PIPA fail is on.
  - R02 (`agc_imu_status_check`): REFSMFLG set -> fine; else ALARM 00220
    (REFSMMAT unknown) or 00210 (IMU not operating).
  - P06 (`agc_job_standby`, `agc_job_standby_request`,
    `agc_job_standby_recovery`): NODOFLAG set (no V37) while the GNCS goes
    to standby; channel 13 STANDBYENB; TIME2/TIME1 recovered afterwards.
* **Gyro torquing** (`src/imu/gyro.c:agc_imu_pulse_buf`, `agc_imu_pulse`):
  IMUPULSE reserves the gyros (GYROBUSY: a busy torquer makes the caller
  sleep until GWAKE) and starts STRTGYRO 40 ms later; Y, Z, X in turn at
  3200 pulses/s (channel 14 GYROACT / GYRONEG); commands of 8192 pulses or
  more get 8192AUG augments (`augment_schedule`); less than one pulse is
  kept as the residue. Caged or coarse align while torquing is IMUBAD
  (BADEND) and IMUPULSE returns ALARM. OGC/IGC/MGC is the one alignment
  erasable (`gyro_words`; closed-loop-fixes L2).
* **Compensation** (`compensation.c`): 1/PIPA corrects PIPA scale factor
  and bias; IRIGCOMP accumulates acceleration-sensitive drift into GCOMP;
  NBDONLY (`agc_job_nbdonly`, started by the waitlist's ENDTASK when
  DRIFTFLG is set and the IMU is free) adds bias drift; any gyro command
  over two pulses is torqued (1/GYRO, `comp_irig_words` -> gyro command
  job). SURFFLAG selects the lunar-surface form (`agc_job_lastbias`).
* **Alignment maths** (`alignment.c`): CALCGTA, CALCGA (`agc_imu_calcga`:
  middle gimbal beyond the lock limit sets GLOKFAIL and ALARM 00401 but
  still completes), AXISGEN, CALCSMSC; half-unit vectors throughout.
* **Maneuvers** (`maneuver.c`, `maneuver_calc.c`, `attitude_maneuver.c`):
  GOMANUR (`agc_imu_gomanur`) requires KALCMANU free (else POODOO 01210);
  KALCMAN3 (`kalcmanu_start`) finds axis and angle to CPHI/CTHETA/CPSI
  (LM: |CPSI| >= 70 deg is TOOBADF, ALARM 00401), sets CALCMAN2;
  UPDTCALL (`agc_task_updtcall`) each second FINDVACs NEWDELHI, which
  advances MIS by DEL and hands the DAP new desired CDUs
  (`steering_update`). R60 (`attitude_maneuver.c`) flashes V50N18 and
  V06N18; 3AXISFLG chooses a three-axis maneuver or VECPOINT; V34 ends
  in GOTOPOOH (or GOTOV56 in R61); RELINUS restarts a priority display
  only with TRACKFLG / RNDVZFLG. The auto test reads channel 31 bits
  13-15 (CM) or channel 30 bit 10 then channel 31 bit 14 (LM ISITAUTO).
* **CSM prelaunch** (`src/csm/prelaunch/align.c`,
  `gyrocompass.c`, `verify.c`): P01 after P11
  (NOP01FLG) is POODOO 01521; a mode switch or torquing ending bad is
  ALARM 01601 and ENDTEST1 (`prelaunch_somerr2`). P02 loops every 0.5 s
  (`setgwlst`, ALLOOP/ALFLT: erection while ERECTIM1 > 0, then
  gyrocompass), torques every 10 samples, slews on a V78 azimuth load, and
  ends at liftoff (`chkcomed`: channel 30 LIFTOFF, GRRBKFLG). P03 compares
  two sextant marks with the expected lines of sight (AXISGEN, CALCGTA);
  an optics stall ending bad is ALARM 01602; V34 or TRM03FLG returns to
  P02. The IMU performance test drift part (erasable-memory program, not
  in the rope) is UNIMPLEMENTED.

### Decision points by function

49 of the lane's 280 functions test or change a flag, raise an alarm, test a channel bit or start a job/task (AGC = the label(s) the prolog names; "+F" sets flag F, "-F" clears it; "starts" = NOVAC/FINDVAC, WAITLIST, LONGCALL, DELAYJOB targets).

| C function | AGC | flags tested | flags set (+) / cleared (-) | alarms | channel bits | starts |
|---|---|---|---|---|---|---|
| `prelaunch_endtest1` (align.c) | ENDTEST1 | - | -IMUSE | - | - | - |
| `prelaunch_somerr2` (align.c) | SOMERR2 | - | - | ALARM 1601 | - | - |
| `agc_job_prelaunch_initialization` (align.c) | GTSCPSS GTSCPSSA NXXTENN | NO_PRELAUNCH_INITIALIZATION | - | POODOO 1521 | - | - |
| `agc_job_prelaunch_imu_zeroed` (align.c) | IMUBACK NBPOSPL | GLOKFAIL | -GLOKFAIL | - | - | - |
| `agc_job_prelaunch_imu_fine_aligned` (align.c) | IMUBACK | - | - | - | - | task gyrocompassing_wake |
| `setgwlst` (gyrocompass.c) | SETGWLST | - | - | - | - | task gyrocompassing_sample |
| `chkcomed` (gyrocompass.c) | CHKCOMED PRELTERM | GRRBKFLG | - | - | ch30 LIFTOFF | - |
| `agc_task_gyrocompassing_filter_start` (gyrocompass.c) | SPECSTS | - | - | - | - | job gyrocompassing_filter |
| `agc_job_optical_verification_marked` (verify.c) | RETARG | TRM03FLG | - | ALARM 1602 | - | - |
| `agc_job_optical_verification_end` (verify.c) | GCOMP5 | - | -TRM03FLG | - | - | - |
| `agc_imu_calcga` (alignment.c) | CALCGA CALCGA1 | - | +GLOKFAIL | ALARM 0401 | - | - |
| `final_angles` (attitude_maneuver.c) | REDOMANN REDOMANC | 3AXISFLG | - | - | - | - |
| `chklinus` (attitude_maneuver.c) | CHKLINUS | PDSPFLAG | - | - | - | - |
| `maneuver_return` (attitude_maneuver.c) | ENDMANU1 | 3AXISFLG | -3AXISFLG | - | - | - |
| `agc_job_attitude_maneuver_terminate` (attitude_maneuver.c) | R61TEST | PDSPFLAG | - | - | - | - |
| `agc_job_relinus` (attitude_maneuver.c) | RELINUS GOREDO20 | RNDVZFLG TRACKFLG | +PDSPFLAG | - | - | - |
| `agc_job_crew_defined_maneuver_gomove` (attitude_maneuver.c) | GOMOVE | - | +3AXISFLG | - | - | - |
| `comp_irig_words` (compensation.c) | IRIGCOMP | - | - | - | - | job gyro_command_pulse |
| `agc_job_nbdonly` (compensation.c) | NBDONLY | DRIFTFLG SURFFLAG | - | - | - | - |
| `agc_job_lastbias` (compensation.c) | LASTBIAS | SURFFLAG | - | - | - | - |
| `agc_imu_pulse_buf` (gyro.c) | IMUPULSE GYROBUSY GWAKE | - | - | - | ch14 GYROPWR | - |
| `lastseg` (gyro.c) | LASTSEG GYROEXIT | - | - | - | ch14 GYROACT | - |
| `augment_schedule` (gyro.c) | AUG3 GYROEXIT | - | - | - | ch14 GYROACT | - |
| `torque` (gyro.c) | MAJ+ MAJ- GMERGE | - | - | - | ch14 GYRONEG | - |
| `agc_task_gyro_augment` (gyro.c) | 8192AUG AUG2 | - | - | - | ch12 COARSE | - |
| `steering_update` (maneuver.c) | NEWANGL | CALCMAN2 | -CALCMAN2 | - | - | - |
| `kalcmanu_start` (maneuver.c) | KALCMAN3 | - | +CALCMAN2 | ALARM 0401 | - | - |
| `agc_imu_gomanur` (maneuver.c) | GOMANUR | - | - | POODOO 1210 | - | - |
| `agc_task_updtcall` (maneuver.c) | UPDTCALL | - | - | - | - | job newdelhi |
| `agc_imu_zero` (mode.c) | IMUZERO IMUZEROA | - | - | ALARM 0206, ALARM 0210 | ch12 COARSE, ch12 ENBIMUERR, ch12 ZEROICDU | - |
| `agc_task_imu_zero_release` (mode.c) | IMUZERO2 | - | - | - | ch12 ZEROICDU | task imu_zero_done |
| `agc_task_imu_zero_done` (mode.c) | IMUZERO3 | - | - | - | ch12 ZEROICDU | - |
| `agc_task_coarse_align_commands` (mode.c) | COARS | - | - | - | ch12 ENBIMUERR | task coarse_align_pulses |
| `agc_task_coarse_align_pulses` (mode.c) | COARS2 NEXTCDU SENDPULS | - | - | - | ch14 DRIVECDUX, ch14 DRIVECDUY, ch14 DRIVECDUZ | task coarse_align_pulses, task coarse_align_check |
| `agc_task_coarse_align_check` (mode.c) | CHKCORS CORSCHK2 COARSERR | - | - | ALARM 0211 | - | - |
| `agc_task_ca_ece` (mode.c) | CA+ECE | - | - | - | ch12 ENBIMUERR | - |
| `agc_imu_setcoars` (mode.c) | SETCOARS | - | - | - | ch12 COARSE, ch12 ENBIMUERR, ch14 GYROACT | - |
| `agc_imu_rndrefdr` (mode.c) | RNDREFDR | - | -TRACKFLG -DRIFTFLG -REFSMFLG | - | - | - |
| `agc_imu_fine` (mode.c) | IMUFINE IMUFIN20 | - | - | - | ch12 COARSE, ch12 ZEROICDU | - |
| `agc_task_imufined` (mode.c) | IMUFINED | - | - | - | ch12 ZEROICDU | - |
| `agc_task_ifailok` (mode.c) | IFAILOK | - | - | - | ch12 COARSE | - |
| `agc_imu_pipfree` (mode.c) | PIPFREE | - | - | ALARM 0212 | - | - |
| `agc_imu_endimu` (mode.c) | ENDIMU IMUGOOD | - | - | - | ch11 ISSWARN | - |
| `modabort` (mode.c) | MODABORT | - | - | BAILOUT 1210, POODOO 1210 | - | - |
| `agc_imu_status_check` (mode.c) | R02BOTH | REFSMFLG | - | ALARM code) | - | - |
| `agc_imu_set_in_use` (mode.c) | R02ZERO | IMUSE | +IMUSE | - | - | - |
| `agc_job_standby` (mode.c) | P06 PRESTAND | - | +NODOFLAG -IMUSE -RNDVZFLG | - | ch13 STANDBYENB | - |
| `agc_job_standby_request` (mode.c) | P06 | NODOFLAG | - | - | - | - |
| `agc_job_standby_recovery` (mode.c) | POSTAND POSTCOM | - | -NODOFLAG | - | ch13 STANDBYENB | - |

### Shared erasables read and written

Accessors of `include/agc/core/state.h` and the lanes' public headers called by this lane, with the number of its functions that read / write each. Lane-private erasables are `static` in the lane's files or in its state struct; their scales are on the declarations.

| erasable (accessor) | meaning and scale (accessor prolog) | read by | written by |
|---|---|---|---|
| `1jacc` | none -- accessor of the DAP erasable 1JACC (ERASABLE_ASSIGNMENTS.agc).; returns agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h | 1 | 0 |
| `1jaccq` | none -- accessor of the DAP erasable 1JACCQ (ERASABLE_ASSIGNMENTS.agc).; returns agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h | 1 | 0 |
| `1jaccr` | none -- accessor of the DAP erasable 1JACCR (ERASABLE_ASSIGNMENTS.agc).; returns agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h | 1 | 0 |
| `cdux` | CDU X (outer gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (register CDUX, 032) | 6 | 1 |
| `cduxd` | none -- accessor of the DAP erasable CDUXD (ERASABLE_ASSIGNMENTS.agc).; returns uint16_t -- the erasable, at the scale given in agc/lm/dap.h | 0 | 1 |
| `cduy` | CDU Y (inner gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUY) | 5 | 1 |
| `cduyd` | none -- accessor of the DAP erasable CDUYD (ERASABLE_ASSIGNMENTS.agc).; returns uint16_t -- the erasable, at the scale given in agc/lm/dap.h | 0 | 1 |
| `cduz` | CDU Z (middle gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUZ) | 5 | 1 |
| `cduzd` | none -- accessor of the DAP erasable CDUZD (ERASABLE_ASSIGNMENTS.agc).; returns uint16_t -- the erasable, at the scale given in agc/lm/dap.h | 0 | 1 |
| `delcdux` | - | 0 | 1 |
| `delcduy` | - | 0 | 1 |
| `delcduz` | - | 0 | 1 |
| `delperor` | - | 0 | 1 |
| `delqeror` | - | 0 | 1 |
| `delreror` | - | 0 | 1 |
| `delv` | accumulated PIPA delta-V, stable-member axes (DELV, I(6)): PIPA pulses scale 2^14 (count in the high word); 1 pulse = 5.85 cm/s CSM, 1 cm/s LM. KPIP1  | 3 | 3 |
| `desopts` | DESOPTS (ERASABLE_ASSIGNMENTS.agc, CM) -- the shaft angle OPTDRIVE drives to, read RETURNS: DESOPTS, CDU word (2^15 = 1 rev); returns DESOPTS, CDU wor | 0 | 1 |
| `desoptt` | DESOPTT (ERASABLE_ASSIGNMENTS.agc, CM) -- the trunnion angle OPTDRIVE drives to, read RETURNS: DESOPTT, CDU word (2^15 = 1 rev); returns DESOPTT, CDU  | 0 | 1 |
| `dsptab` | display panel relay buffer DSPTAB +0..+10D, +11D is the C/S lights word (15-bit relay words) Index 0..11. | 4 | 1 |
| `fdai` | none (accessor) -- FDAIX, FDAIY, FDAIZ (LM, noun 18); returns uint16_t -- FDAI angle, CDU pattern | 1 | 0 |
| `imodes30` | last-sampled channel 30 bits, T4RUPT IMUMON (IMODES30, B(1)PRM) | 8 | 2 |
| `imodes33` | last-sampled channel 33 / DAP-enable bits, T4RUPT C33TEST (IMODES33, B(1)PRM) | 1 | 1 |
| `launchaz` | LAUNCHAZ (CSM prelaunch): the launch azimuth, sent on the coast and align downlist. AGC: LAUNCHAZ (Comanche055 ERASABLE_ASSIGNMENTS.agc); returns agc_ | 1 | 0 |
| `modecadr` | C(MODECADR +dev) (ERASABLE_ASSIGNMENTS.agc "MODE SWITCHING ERASABLE").; returns agc_sp_t -- the word (+0 none, positive a waiting job) | 2 | 1 |
| `modreg` | MODREG major mode (program) | 1 | 0 |
| `omegapd` | none -- accessor of the DAP erasable OMEGAPD (ERASABLE_ASSIGNMENTS.agc).; returns agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h | 0 | 1 |
| `omegaqd` | none -- accessor of the DAP erasable OMEGAQD (ERASABLE_ASSIGNMENTS.agc).; returns agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h | 0 | 1 |
| `omegard` | none -- accessor of the DAP erasable OMEGARD (ERASABLE_ASSIGNMENTS.agc).; returns agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h | 0 | 1 |
| `optind` | OPTIND (ERASABLE_ASSIGNMENTS.agc, CM) -- coarse optics state, read RETURNS: OPTIND: -1, 0, 1, or AGC_OPTIND_NEG0 for -0; returns OPTIND: -1, 0, 1, or  | 0 | 2 |
| `pipax` | PIPA X pulse count since last read (register PIPAX, 037), unscaled pulses: 1 pulse = 5.85 cm/s (CSM SERVICER207), 1 cm/s (LM) | 1 | 3 |
| `pipay` | PIPA Y pulse count (register PIPAY) | 1 | 3 |
| `pipaz` | PIPA Z pulse count (register PIPAZ) | 1 | 3 |
| `piptime` | time of the PIPA reading / this vehicle's state vector, centiseconds, scale 2^28 (PIPTIME, B(2)PRM) | 1 | 0 |
| `thetad` | desired gimbal angles THETAD +0..+2 (CPHI/CTHETA/CPSI), raw 15-bit CDU-style words, 2^15 = 1 revolution Index 0..2. | 5 | 3 |
| `time1` | ---- time: TIME1/TIME2 (centiseconds, DP), TIME3..TIME6 are exec-private | 2 | 0 |

### AGC labels -> C functions

The "AGC" column above names the label each listed function translates; the complete label -> function -> file:line index for the lane (and the tree) is [docs/overview/xref.md](../overview/xref.md), generated by `tools/xref.py`.
