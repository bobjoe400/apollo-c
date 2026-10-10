# imu -- IMU moding, compensation, alignment, attitude maneuvers

## Scope
AGC sources (CM = Comanche055, LM = Luminary099):
* IMU_MODE_SWITCHING_ROUTINES.agc (CM pp.1420-1448, LM pp.1309-1337) -> src/imu/imu_mode.c
* IMU_COMPENSATION_PACKAGE.agc (CM pp.297-306, LM pp.326-337) -> src/imu/imu_comp.c
* INFLIGHT_ALIGNMENT_ROUTINES.agc (CM pp.1355-1363, LM same text) + CALCSMSC
  (CM P51-P53.agc p.763, LM POWERED_FLIGHT_SUBROUTINES.agc p.1265 FLESHPOT) -> src/imu/alignment.c
* GIMBAL_LOCK_AVOIDANCE.agc, KALCMANU_STEERING.agc, ANGLFIND.agc (CM) /
  ATTITUDE_MANEUVER_ROUTINE.agc (LM) -> src/imu/maneuver.c
* R60_62.agc (both) -> src/imu/r60.c
* IMU_CALIBRATION_AND_ALIGNMENT.agc (CM only, pp.423-455) -> src/csm/prelaunch_align.c
* Public header include/agc/agc_imu.h; pending externals in src/imu/imu_pending.h.

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
| R02BOTH (p.1443-1444) | REFSMFLG, IMODES30 | alarm 210/220 | agc_imu_r02both |
| P06/PRESTAND/SCALPREP/POSTAND (p.1445-1448) | TIME1/2, scaler | | agc_imu_p06 (stub), agc_imu_postand (stub) |
| 1/PIPA (ICP p.326) | DELV (pulses 2^14), PIPABIAS (pulses/cs 2^-5 LM, 2^-8 CM), PIPASCF (ppm 2^-9), 1/PIPADT (cs 2^8) | DELV compensated | agc_imu_comp_pipa |
| IRIGCOMP/IRIGX/Y/Z/GCOMPSUB/DRIFTSUB/DRFTSUB2 (p.328-332) | DELV, ADIA/ADSRA (gyro pulses/pipa pulse 2^-6 LM, 2^-3 CM), NBD (pulses/cs 2^-5) | GCOMP (gyro pulses 2^14), GCOMPSW | agc_imu_comp_irig |
| 1/GYRO/GCOMP1 (p.333) | GCOMP | IMUPULSE | agc_job_1_gyro, agc_job_gcomp1 |
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
| R60CSM/R60LEM/R61TEST/CHKLINUS/RELINUS (R60 p.390-392) | | displays | agc_job_r60 ... |
| VECPOINT/VECPNT1/PICKAXIS/ROT180 (p.393-396) | SCAXIS, POINTVSM half-unit | CPHI.. SP | agc_imu_vecpoint |
| BALLANGS (LM) | CPHI.. | FDAIX.. | agc_imu_ballangs |
| R62DISP/GOMOVE (p.398) | | | agc_job_r62disp |
| prelaunch: POSN17C, ERTHRVSE, EARTHR, ALWAYSG/COMPGS, RESTAIER, TAR/EREF, CHKCOMED (pp.430-450) | LATITUDE, AZIMUTH rev | ERCOMP1 rev | agc_csm_prelaunch_* |

## Progress
(updated as files complete)
* imu_comp.c: TODO
* alignment.c: TODO
* imu_mode.c: TODO
* maneuver.c: TODO
* r60.c: TODO
* prelaunch_align.c: TODO
* tests/test_imu.c: TODO

## Deviations
## Stubs
## Requested core additions
## Requested jobs-tasks
## Tests written
## Open questions
* IMUMON and the ISS fail monitors are in T4RUPT_PROGRAM.agc (sys-misc lane), not in
  IMU_MODE_SWITCHING_ROUTINES.agc as the assignment states.
