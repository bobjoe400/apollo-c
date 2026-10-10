/*
 * exec_pending.h -- job and task identifiers that the exec lane names but
 * other lanes own (restart-table entries, display restarts, drift
 * compensation, autopilot T5RUPT programs).
 *
 * INTEGRATION: each constant below is PROVISIONAL (0x0E00 range). The lead
 * replaces it with the owning lane's agc_jobs.h member and adds the case to
 * src/exec/dispatch.c; then delete the line here. The same list is in
 * docs/modules/exec-sched.md under "Requested jobs-tasks".
 *
 * Names follow docs/agents/_common.md: AGC_JOB_<LABEL> / AGC_TASK_<LABEL>,
 * with '.', '/', '-', ',' in the AGC label replaced by '_'.
 */
#ifndef AGC_EXEC_PENDING_H
#define AGC_EXEC_PENDING_H

#include "agc/agc_jobs.h"

/* ---- jobs started by the executive services themselves ---------------- */
#define AGC_JOB_INITDSP    ((agc_job_id_t)0x0E01) /* X.1 display restart (pinball) */
#define AGC_JOB_DSPMMJB    ((agc_job_id_t)0x0E02) /* DSPMMJOB = DSPMMJB (pinball) */
#define AGC_JOB_NBDONLY    ((agc_job_id_t)0x0E03) /* IMU bias drift comp (imu)    */

/* ---- jobs named in the restart tables (both vehicles) ---------------- */
#define AGC_JOB_STATINT1   ((agc_job_id_t)0x0E10) /* 2.5  nav-integ               */
#define AGC_JOB_RELINUS    ((agc_job_id_t)0x0E11) /* CSM 1.7, LM 2.13 (imu/pinball)*/
#define AGC_JOB_S40_13     ((agc_job_id_t)0x0E12) /* CSM 3.3, LM 3.5 guid-powered */
#define AGC_JOB_POSTBURN   ((agc_job_id_t)0x0E13) /* CSM 4.35, LM 4.13            */
#define AGC_JOB_NORMLIZE   ((agc_job_id_t)0x0E14) /* 5.2  guid-powered            */
#define AGC_JOB_SERVICER   ((agc_job_id_t)0x0E15) /* 5.4  guid-powered            */
#define AGC_JOB_TIMEDIDR   ((agc_job_id_t)0x0E16) /* 6.5  P27 (sys-misc)          */
/* CSM only */
#define AGC_JOB_REDO40_9   ((agc_job_id_t)0x0E20) /* 1.5  REDO40.9                */
#define AGC_JOB_PIKUP20    ((agc_job_id_t)0x0E21) /* 1.11                         */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA        ((agc_job_id_t)0x0E22) /* 2.7                          */
#define AGC_JOB_ALTERNATE_LOS_SIGHTING_MARK_ENTER   ((agc_job_id_t)0x0E23) /* 2.11                         */
#define AGC_JOB_REDO_RENDEZVOUS_TRACKING_DATA    ((agc_job_id_t)0x0E24) /* 2.13                         */
#define AGC_JOB_MATRXJOB   ((agc_job_id_t)0x0E25) /* 3.7                          */
#define AGC_JOB_RESTART_EARTH_ORBIT_INSERTION_MONITOR      ((agc_job_id_t)0x0E26) /* 3.11                         */
#define AGC_JOB_REP11A     ((agc_job_id_t)0x0E27) /* 3.13                         */
#define AGC_JOB_THRUST_MONITOR_BODY    ((agc_job_id_t)0x0E28) /* 4.2 second part              */
#define AGC_JOB_COMPVER    ((agc_job_id_t)0x0E29) /* 4.15                         */
#define AGC_JOB_AZMTHCG1   ((agc_job_id_t)0x0E2A) /* 4.17                         */
#define AGC_JOB_SPS_BURN_S_SV    ((agc_job_id_t)0x0E2B) /* 4.23 P40S/SV                 */
#define AGC_JOB_PROG52     ((agc_job_id_t)0x0E2C) /* 4.25                         */
#define AGC_JOB_FINE_ALIGN        ((agc_job_id_t)0x0E2D) /* 4.31 (R51 +1)                */
#define AGC_JOB_ENTRY_FINAL_PHASE_1      ((agc_job_id_t)0x0E2E) /* 4.41 P67.1                   */
#define AGC_JOB_S61_1A     ((agc_job_id_t)0x0E2F) /* 4.45 (S61.1A -1)             */
#define AGC_JOB_PRE_HUNT   ((agc_job_id_t)0x0E30) /* 4.47 PRE-HUNT                */
#define AGC_JOB_ENTRY_UPCONTROL_1      ((agc_job_id_t)0x0E31) /* 4.55 P65.1                   */
#define AGC_JOB_RSTGTS1    ((agc_job_id_t)0x0E32) /* 5.7  prelaunch               */
#define AGC_JOB_WTLISTNT   ((agc_job_id_t)0x0E33) /* 5.13                         */
#define AGC_JOB_RESTEST1   ((agc_job_id_t)0x0E34) /* 5.15                         */
#define AGC_JOB_GEOSTRT4   ((agc_job_id_t)0x0E35) /* 5.17                         */
#define AGC_JOB_ALFLT1     ((agc_job_id_t)0x0E36) /* 5.21                         */
#define AGC_JOB_RESTEST3   ((agc_job_id_t)0x0E37) /* 5.25                         */
#define AGC_JOB_RESTAIER   ((agc_job_id_t)0x0E38) /* 5.27                         */
#define AGC_JOB_RESCHNG    ((agc_job_id_t)0x0E39) /* 5.33                         */
/* LM only */
#define AGC_JOB_PREFERRED_TRACKING_ATTITUDE_LEM1    ((agc_job_id_t)0x0E40) /* 2.11                         */
#define AGC_JOB_RENDEZVOUS_TRACKING_DATA_RSTRT   ((agc_job_id_t)0x0E41) /* 2.15                         */
#define AGC_JOB_GOABORT    ((agc_job_id_t)0x0E42) /* 4.3                          */
#define AGC_JOB_FINE_ALIGN_1      ((agc_job_id_t)0x0E43) /* 4.21 (R51.1 +1)              */
#define AGC_JOB_DPS_ABORT_A       ((agc_job_id_t)0x0E44) /* 4.27                         */
#define AGC_JOB_APS_ABORT_A       ((agc_job_id_t)0x0E45) /* 4.31                         */
#define AGC_JOB_COMFAIL    ((agc_job_id_t)0x0E46) /* 4.37                         */
#define AGC_JOB_REDO6_7    ((agc_job_id_t)0x0E47) /* 6.7                          */

/* ---- tasks named in the restart tables ------------------------------- */
#define AGC_TASK_STATEINT  ((agc_task_id_t)0x0E80) /* 2.3 LONGCALL (nav-integ)    */
#define AGC_TASK_REREADAC  ((agc_task_id_t)0x0E81) /* 5.2/5.3/5.4 guid-powered    */
#define AGC_TASK_REDO5_5   ((agc_task_id_t)0x0E82) /* 5.5                         */
#define AGC_TASK_CLOKTASK  ((agc_task_id_t)0x0E83) /* 6.3 (CSM also 6.2)          */
#define AGC_TASK_TIG_5     ((agc_task_id_t)0x0E84) /* TIG-5                       */
#define AGC_TASK_TIG_0     ((agc_task_id_t)0x0E85) /* TIG-0                       */
#define AGC_TASK_IGNITION  ((agc_task_id_t)0x0E86)
/* CSM only */
#define AGC_TASK_SETJTAG   ((agc_task_id_t)0x0E90) /* 1.3                         */
#define AGC_TASK_ENGINOFF  ((agc_task_id_t)0x0E91) /* 3.15                        */
#define AGC_TASK_PRECHECK  ((agc_task_id_t)0x0E92) /* 4.2/4.4/4.6 first part      */
#define AGC_TASK_TTG_0     ((agc_task_id_t)0x0E93) /* 4.4 TTG/0                   */
#define AGC_TASK_DOTVCON   ((agc_task_id_t)0x0E94) /* 4.3                         */
#define AGC_TASK_DOSTRULL  ((agc_task_id_t)0x0E95) /* 4.5                         */
#define AGC_TASK_ENGINE_FAIL_PROCEDURE_E40_6  ((agc_task_id_t)0x0E96) /* 4.11 V97E40.6               */
#define AGC_TASK_ENGINE_FAIL_ENABL  ((agc_task_id_t)0x0E97) /* 4.13                        */
#define AGC_TASK_TIGBLNK   ((agc_task_id_t)0x0E98) /* 4.21 LONGCALL               */
#define AGC_TASK_DOTVCRCS  ((agc_task_id_t)0x0E99) /* 4.27                        */
#define AGC_TASK_WAKE_CM_SM_SEPARATION   ((agc_task_id_t)0x0E9A) /* 4.33                        */
#define AGC_TASK_TIGAVEG   ((agc_task_id_t)0x0E9B) /* 4.37                        */
#define AGC_TASK_S61_1C    ((agc_task_id_t)0x0E9C) /* 4.43 S61.1C                 */
#define AGC_TASK_ATERTASK  ((agc_task_id_t)0x0E9D) /* 4.51                        */
#define AGC_TASK_ENGINE_FAIL_PROCEDURE_ETASK  ((agc_task_id_t)0x0E9E) /* 4.53                        */
#define AGC_TASK_TIGON     ((agc_task_id_t)0x0E9F) /* 4.57                        */
#define AGC_TASK_DOSPSOFF  ((agc_task_id_t)0x0EA0) /* 4.63                        */
#define AGC_TASK_ENGINE_FAIL_PROCEDURE_TTASK  ((agc_task_id_t)0x0EA1) /* 4.67                        */
#define AGC_TASK_ENGINE_FAIL_PROCEDURE_TRCS   ((agc_task_id_t)0x0EA2) /* 4.71                        */
#define AGC_TASK_ENGINE_FAIL_PROCEDURE_PTASK  ((agc_task_id_t)0x0EA3) /* 4.73                        */
#define AGC_TASK_SPSOFF97  ((agc_task_id_t)0x0EA4) /* 4.75                        */
#define AGC_TASK_ALLOOP1   ((agc_task_id_t)0x0EA5) /* 5.11                        */
#define AGC_TASK_SPECSTS   ((agc_task_id_t)0x0EA6) /* 5.23                        */
#define AGC_TASK_CHEKAVEG  ((agc_task_id_t)0x0EA7) /* 5.37                        */
#define AGC_TASK_PREREAD   ((agc_task_id_t)0x0EA8) /* 5.41                        */
#define AGC_TASK_PRE40_6   ((agc_task_id_t)0x0EA9) /* 6.2 PRE40.6                 */
#define AGC_TASK_READGYMB  ((agc_task_id_t)0x0EAA) /* 6.11                        */
#define AGC_TASK_REDORCS   ((agc_task_id_t)0x0EAB) /* T5LOC: RCS DAP (dap-csm)    */
#define AGC_TASK_REDOTVC   ((agc_task_id_t)0x0EAC) /* T5LOC: TVC DAP (dap-csm)    */
#define AGC_TASK_REDOSAT   ((agc_task_id_t)0x0EAD) /* T5LOC: Saturn (dap-csm)     */
#define AGC_TASK_TVCEXEC   ((agc_task_id_t)0x0EAE) /* ENEMA, TVC on (dap-csm)     */
#define AGC_TASK_CA_ECE    ((agc_task_id_t)0x0EAF) /* GOPROG NO ATT (CA+ECE, imu) */
/* LM only */
#define AGC_TASK_ULLGTASK  ((agc_task_id_t)0x0EC0) /* 1.3                         */
#define AGC_TASK_RENDEZVOUS_NAVIGATION_LEMC1  ((agc_task_id_t)0x0EC1) /* 2.7                         */
#define AGC_TASK_REDO2_17  ((agc_task_id_t)0x0EC2) /* 2.17                        */
#define AGC_TASK_LANDING_RADAR_MONITOR_RESTART   ((agc_task_id_t)0x0EC3) /* 2.21 R10,R11                */
#define AGC_TASK_ZOOM      ((agc_task_id_t)0x0EC4) /* 3.3                         */
#define AGC_TASK_REDO4_2   ((agc_task_id_t)0x0EC5) /* 4.2 second part             */
#define AGC_TASK_ULLAGOFF  ((agc_task_id_t)0x0EC6) /* 4.5                         */
#define AGC_TASK_ENGOFTSK  ((agc_task_id_t)0x0EC7) /* 4.11                        */
#define AGC_TASK_TIG_30    ((agc_task_id_t)0x0EC8) /* 4.15 TIG-30                 */
#define AGC_TASK_TIG_35    ((agc_task_id_t)0x0EC9) /* 4.25 LONGCALL TIG-35        */
#define AGC_TASK_BIBIBIAS  ((agc_task_id_t)0x0ECA) /* 5.7                         */
#define AGC_TASK_DAPIDLER  ((agc_task_id_t)0x0ECB) /* T5ADR after STARTSB1 (dap-lm)*/

#endif /* AGC_EXEC_PENDING_H */
