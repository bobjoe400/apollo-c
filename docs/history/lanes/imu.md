# imu -- lane history

Process diary of the imu lane, moved verbatim out of [docs/modules/imu.md](../../modules/imu.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-14 (first and last commit touching the note).

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

## Open questions
* IMUMON and the ISS fail monitors are in T4RUPT_PROGRAM.agc (sys-misc lane), not in
  IMU_MODE_SWITCHING_ROUTINES.agc as the assignment states.
* CALCSMSC is also implemented by prog-align (agc_align_calcsmsc); one should go.
* GYROCMD and CDUXCMD..CDUZCMD are hardware counters: the sim must drain them
  (agc_imu_hw_set_gyrocmd / agc_imu_hw_set_cducmd) or 8192AUG will see them grow.
* The LM DAP header names the KALCMANU increments DELCDUX..Z (LM erasable DELDCDU = DELCDUX); written
  as the negative 0.1 s increment, as the comment there says.

