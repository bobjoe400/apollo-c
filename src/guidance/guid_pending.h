/*
 * guid_pending.h -- symbols guid-powered needs from other lanes that did not
 * exist when this module was written (see docs/modules/guid-powered.md,
 * "Requested core additions"). Everything here follows the naming
 * convention of docs/agents/_common.md. At integration, delete each block
 * whose real header exists and include that header instead; a duplicate
 * definition error points at the block to remove.
 *
 * Flag ids are encoded (flagword << 4) | AGC bit number (1..15), computed
 * from the interpretive switch numbers in the AGC listings.
 */
#ifndef GUID_PENDING_H
#define GUID_PENDING_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_exec.h"
#include "agc/agc_jobs.h"
#include "agc/agc_state.h"
#include "agc/agc_channels.h"
#include "agc/agc_guid.h"

/* ---- flags with the same position in Comanche and Luminary ------------- */
#define AGC_FLAG_RNDVZFLG ((agc_flag_id_t)((0 << 4) | 7))   /* sw 8   */
#define AGC_FLAG_NJETSFLG ((agc_flag_id_t)((1 << 4) | 15))  /* sw 15  */
#define AGC_FLAG_ERADFLAG ((agc_flag_id_t)((1 << 4) | 13))  /* sw 17  */
#define AGC_FLAG_UPDATFLG ((agc_flag_id_t)((1 << 4) | 7))   /* sw 23  */
#define AGC_FLAG_TRACKFLG ((agc_flag_id_t)((1 << 4) | 5))   /* sw 25  */
#define AGC_FLAG_GUESSW   ((agc_flag_id_t)((1 << 4) | 2))   /* sw 28  */
#define AGC_FLAG_DRIFTFLG ((agc_flag_id_t)((2 << 4) | 15))  /* sw 30  */
#define AGC_FLAG_STEERSW  ((agc_flag_id_t)((2 << 4) | 11))  /* sw 34  */
#define AGC_FLAG_IMPULSW  ((agc_flag_id_t)((2 << 4) | 9))   /* sw 36  */
#define AGC_FLAG_XDELVFLG ((agc_flag_id_t)((2 << 4) | 8))   /* sw 37  */
#define AGC_FLAG_AVFLAG   ((agc_flag_id_t)((2 << 4) | 5))   /* sw 40  */
#define AGC_FLAG_PFRATFLG ((agc_flag_id_t)((2 << 4) | 4))   /* sw 41  */
#define AGC_FLAG_REFSMFLG ((agc_flag_id_t)((3 << 4) | 13))  /* sw 47  */
#define AGC_FLAG_RNGSCFLG ((agc_flag_id_t)((5 << 4) | 10))  /* sw 80  */
#define AGC_FLAG_ENGONFLG ((agc_flag_id_t)((5 << 4) | 7))   /* sw 83  */
#define AGC_FLAG_3AXISFLG ((agc_flag_id_t)((5 << 4) | 6))   /* sw 84  */
#define AGC_FLAG_IGNFLAG  ((agc_flag_id_t)((7 << 4) | 13))  /* sw 107 */
#define AGC_FLAG_ASTNFLAG ((agc_flag_id_t)((7 << 4) | 12))  /* sw 108 */
#define AGC_FLAG_NORMSW   ((agc_flag_id_t)((7 << 4) | 10))  /* sw 110 */
#define AGC_FLAG_V37FLAG  ((agc_flag_id_t)((7 << 4) | 6))   /* sw 114 */
#define AGC_FLAG_SURFFLAG ((agc_flag_id_t)((8 << 4) | 8))   /* sw 127 */

/* ---- shared erasables (state-flags lane) -------------------------------- */
agc_mat_t agc_erase_refsmmat(void);        /* REFSMMAT half unit 2^1        */
uint16_t  agc_erase_cdux(void);            /* CDUX 2's comp counts          */
uint16_t  agc_erase_cduy(void);
uint16_t  agc_erase_cduz(void);
agc_sp_t  agc_erase_pipax(void);           /* PIPAX counter, pulses         */
agc_sp_t  agc_erase_pipay(void);
agc_sp_t  agc_erase_pipaz(void);
void      agc_erase_set_pipax(agc_sp_t v);
void      agc_erase_set_pipay(agc_sp_t v);
void      agc_erase_set_pipaz(agc_sp_t v);
agc_vec_t agc_erase_delv(void);            /* DELV SM pulses 2^14           */
void      agc_erase_set_delv(agc_vec_t v);
agc_vec_t agc_erase_vgtig(void);           /* VGTIG ref m/cs 2^7            */
void      agc_erase_set_vgtig(agc_vec_t v);
agc_dp_t  agc_erase_ttogo(void);           /* TTOGO cs 2^28                 */
void      agc_erase_set_ttogo(agc_dp_t t);
void      agc_erase_set_1_pipadt(agc_sp_t v); /* 1/PIPADT (IMU compensation) */

/* ---- executive extras (exec-sched lane) --------------------------------- */
uint16_t     agc_phase_get(int group);     /* PHASEn (>0 active)            */
agc_status_t agc_phase_kill(int group);    /* DXCH -PHASEn with NEG0        */
agc_status_t agc_waitlist_kill(agc_task_id_t task); /* KILLTASK (LM p.750)  */
agc_status_t agc_exec_delayjob(uint16_t delay_cs, agc_job_id_t wake_as); /* DELAYJOB */
agc_status_t agc_job_gotopooh(agc_job_ctx_t *ctx);  /* GOTOPOOH             */
agc_status_t agc_job_canv37(agc_job_ctx_t *ctx);    /* CANV37               */

/* ---- DSKY (io-pinball lane) --------------------------------------------- */
agc_status_t agc_dsky_display(uint8_t verb, uint8_t noun, agc_job_id_t on_terminate,
                              agc_job_id_t on_proceed, agc_job_id_t on_enter);
agc_status_t agc_dsky_please_perform(uint16_t checklist, agc_job_id_t on_terminate,
                                     agc_job_id_t on_proceed, agc_job_id_t on_enter);
agc_status_t agc_dsky_clear(void);         /* CLEANDSP                      */

/* ---- IMU (imu lane) ----------------------------------------------------- */
agc_status_t agc_imu_1pipa(void);          /* 1/PIPA: compensate DELV       */
agc_status_t agc_imu_pipfree(void);        /* PIPFREE                       */
agc_status_t agc_imu_r02both(void);        /* R02BOTH IMU status check      */
agc_status_t agc_imu_r60_start(agc_job_id_t on_done); /* R60CSM / R60LEM    */

/* ---- navigation (nav-integ lane) ---------------------------------------- */
agc_status_t agc_integ_thisprec(agc_dp_t tdec1, agc_guid_prec_t *out);
agc_status_t agc_integ_csmprec(agc_dp_t tdec1, agc_guid_prec_t *out);
agc_status_t agc_integ_lemprec(agc_dp_t tdec1, agc_guid_prec_t *out);
/* MIDTOAV1/MIDTOAV2: integrate to TDEC1 (or now+), leave RN1 VN1 PIPTIME1;
 * *dt = time to PREREAD (cs, 2^28); *late = integration time exceeded.    */
agc_status_t agc_integ_midtoav1(agc_dp_t tdec1, agc_dp_t *dt, bool *late);
agc_status_t agc_integ_midtoav2(agc_dp_t *dt);
agc_status_t agc_integ_avetomid(void);     /* AVETOMID                      */

/* ---- P34-35 utilities (prog-rendz lane) --------------------------------- */
typedef struct agc_initvel {
    agc_vec_t rinit;     /* RINIT  m 2^29                                   */
    agc_vec_t vinit;     /* VINIT  m/cs 2^7                                 */
    agc_vec_t rtarg;     /* RTARG  m 2^29                                   */
    agc_dp_t  dellt4;    /* DELLT4 cs 2^28                                  */
    agc_dp_t  eps;       /* EPSILON4 rev (pushed at 2D)                     */
    agc_sp_t  numit;     /* iteration count (pushed at 0D)                  */
    int       rtx1;
    int       rtx2;
    agc_vec_t viprime;   /* out: VIPRIME m/cs 2^7                           */
    agc_vec_t delveet3;  /* out: DELVEET3 m/cs 2^7                          */
} agc_initvel_t;
agc_status_t agc_rendz_initvel(agc_interp_t *ip, agc_initvel_t *io);   /* INITVEL  */
agc_status_t agc_rendz_havegues(agc_interp_t *ip, agc_initvel_t *io);  /* HAVEGUES */
agc_status_t agc_rendz_periapo1(agc_interp_t *ip, agc_vec_t r, agc_vec_t v,
                                int rtx1, int rtx2, agc_dp_t *hper, agc_dp_t *hapo);
agc_dp_t     agc_rendz_shiftr1(agc_interp_t *ip, agc_dp_t x, int rtx1); /* SHIFTR1 */
agc_dp_t     agc_rendz_maxchk(agc_dp_t x);                        /* MAXCHK    */
agc_mat_t    agc_rendz_lomat(agc_interp_t *ip, agc_vec_t ract3, agc_vec_t unrm); /* LOMAT */
/* MIDGIM: DELVLVC (LV coords) or +MGA depending on AVFLAG/COMPUTER.        */
agc_status_t agc_rendz_midgim(agc_interp_t *ip, agc_vec_t dv, agc_vec_t rinit,
                              agc_vec_t vinit, agc_vec_t *delvlvc, agc_dp_t *mga);
agc_status_t agc_rendz_get_lvc(agc_interp_t *ip, agc_vec_t dv, agc_vec_t rinit,
                               agc_vec_t vinit, agc_vec_t *delvlvc);   /* GET.LVC */
agc_status_t agc_rendz_get_mga(agc_interp_t *ip, agc_vec_t dv, agc_dp_t *mga); /* GET+MGA */

/* ---- job and task ids (lead adds them to agc_jobs.h) -------------------- */
#define AGC_JOB_GUID_BASE 0x400
#define AGC_JOB_SERVICER      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 1))  /* SERVICER  */
#define AGC_JOB_NORMLIZE      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 2))  /* NORMLIZE  */
#define AGC_JOB_SERVEXIT      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 3))  /* SERVEXIT  */
#define AGC_JOB_AVGEND        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 4))  /* AVGEND    */
#define AGC_JOB_LASTBIAS      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 5))  /* LASTBIAS (imu) */
#define AGC_JOB_STEERING      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 6))  /* STEERING  */
#define AGC_JOB_CALCN85       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 7))  /* CALCN85   */
#define AGC_JOB_CALCN83       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 8))  /* CALCN83   */
#define AGC_JOB_S40_9         ((agc_job_id_t)(AGC_JOB_GUID_BASE + 9))  /* S40.9     */
#define AGC_JOB_S40_13        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 10)) /* S40.13    */
#define AGC_JOB_P30           ((agc_job_id_t)(AGC_JOB_GUID_BASE + 11)) /* P30       */
#define AGC_JOB_P31           ((agc_job_id_t)(AGC_JOB_GUID_BASE + 12)) /* P31       */
#define AGC_JOB_P30_N81       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 13)) /* P30 after N33 PRO */
#define AGC_JOB_P30_CALC      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 14)) /* P30 after N81 PRO */
#define AGC_JOB_P30_PARAM     ((agc_job_id_t)(AGC_JOB_GUID_BASE + 15)) /* PARAM30 display   */
#define AGC_JOB_P30_REFTEST   ((agc_job_id_t)(AGC_JOB_GUID_BASE + 16)) /* REFTEST / N42 PRO */
#define AGC_JOB_P30_END45     ((agc_job_id_t)(AGC_JOB_GUID_BASE + 17)) /* END45     */
#define AGC_JOB_P31_CALC      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 18)) /* P31 after N33 PRO */
#define AGC_JOB_P31_N81       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 19)) /* P31 N81 then PARAM30 */
#define AGC_JOB_REVN1645      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 20)) /* REVN1645 (prog-rendz) */
#define AGC_JOB_P30_DISP45    ((agc_job_id_t)(AGC_JOB_GUID_BASE + 21)) /* DISP45 redisplay */
#define AGC_JOB_POSTBURN      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 22)) /* POSTBURN  */
#define AGC_JOB_TIGNOW        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 23)) /* TIGNOW    */
#define AGC_JOB_POST41        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 24)) /* POST41/TERM40 */
#define AGC_JOB_CLOCKJOB      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 25)) /* CLOCKJOB/CLOKJOB */
#define AGC_JOB_P40           ((agc_job_id_t)(AGC_JOB_GUID_BASE + 26)) /* P40CSM / P40LM */
#define AGC_JOB_P41           ((agc_job_id_t)(AGC_JOB_GUID_BASE + 27)) /* P41CSM / P41LM */
#define AGC_JOB_P42           ((agc_job_id_t)(AGC_JOB_GUID_BASE + 28)) /* P42LM     */
#define AGC_JOB_P47           ((agc_job_id_t)(AGC_JOB_GUID_BASE + 29)) /* P47CSM / P47LM */
#define AGC_JOB_P47BODY       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 30)) /* P47BODY   */
#define AGC_JOB_P40_AFTER_R60 ((agc_job_id_t)(AGC_JOB_GUID_BASE + 31)) /* return from R60 */
#define AGC_JOB_P40_GMBTEST   ((agc_job_id_t)(AGC_JOB_GUID_BASE + 32)) /* P40GMB (V50N25 R1 204) */
#define AGC_JOB_P40_TRIMONLY  ((agc_job_id_t)(AGC_JOB_GUID_BASE + 33)) /* TRIMONLY  */
#define AGC_JOB_P40_TSTTRIM   ((agc_job_id_t)(AGC_JOB_GUID_BASE + 34)) /* TST,TRIM  */
#define AGC_JOB_P40S_SV       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 35)) /* P40S/SV   */
#define AGC_JOB_P40BLNKR      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 36)) /* P40BLNKR  */
#define AGC_JOB_P40RCS        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 37)) /* P40RCS    */
#define AGC_JOB_P40MINDB      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 38)) /* P40MINDB  */
#define AGC_JOB_DYNDISP       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 39)) /* DYNDISP / DYNMDISP */
#define AGC_JOB_V99E          ((agc_job_id_t)(AGC_JOB_GUID_BASE + 40)) /* V99E      */
#define AGC_JOB_V99T          ((agc_job_id_t)(AGC_JOB_GUID_BASE + 41)) /* V99T      */
#define AGC_JOB_V99P          ((agc_job_id_t)(AGC_JOB_GUID_BASE + 42)) /* V99P      */
#define AGC_JOB_V97T          ((agc_job_id_t)(AGC_JOB_GUID_BASE + 43)) /* V97T      */
#define AGC_JOB_V97P          ((agc_job_id_t)(AGC_JOB_GUID_BASE + 44)) /* V97P      */
#define AGC_JOB_V97E          ((agc_job_id_t)(AGC_JOB_GUID_BASE + 45)) /* V97E      */
#define AGC_JOB_P40_DSKY_DONE ((agc_job_id_t)(AGC_JOB_GUID_BASE + 46)) /* generic display ENDOFJOB */
#define AGC_JOB_COMFAIL       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 47)) /* COMFAIL (LM) */
#define AGC_JOB_LRHJOB        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 48)) /* LRHJOB    */
#define AGC_JOB_LRVJOB        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 49)) /* LRVJOB    */
#define AGC_JOB_HIGATJOB      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 50)) /* HIGATJOB  */
#define AGC_JOB_LRHJOB_DONE   ((agc_job_id_t)(AGC_JOB_GUID_BASE + 51)) /* LRHJOB after RADSTALL */
#define AGC_JOB_LRVJOB_DONE   ((agc_job_id_t)(AGC_JOB_GUID_BASE + 52)) /* LRVJOB after RADSTALL */
#define AGC_JOB_HIGATJOB_DONE ((agc_job_id_t)(AGC_JOB_GUID_BASE + 53)) /* HIGATJOB after RADSTALL */
#define AGC_JOB_POSALARM_PRO  ((agc_job_id_t)(AGC_JOB_GUID_BASE + 54)) /* POSALARM PROCEED */
#define AGC_JOB_P41BLANK      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 55)) /* P41BLANK  */
#define AGC_JOB_TIG_30A       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 56)) /* TIG-30A   */
#define AGC_JOB_BURNBABY      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 57)) /* BURNBABY  */
#define AGC_JOB_P40AUTO_RET   ((agc_job_id_t)(AGC_JOB_GUID_BASE + 58)) /* P40A/P recycle */
#define AGC_JOB_GOCUTOFF      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 59)) /* CUTOFF (ascent) */
#define AGC_JOB_ASTNRET       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 60)) /* ASTNRET (landing) */
#define AGC_JOB_CLOK_V97      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 61)) /* CLOKJOB V97 responses */
#define AGC_JOB_CLOK_V99      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 62)) /* CLOKJOB V99 responses */
#define AGC_JOB_CLOK_N61      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 63)) /* CLOKJOB V06N61 responses */
#define AGC_JOB_TRIMDONE      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 64)) /* TRIMDONE (R03) */
#define AGC_JOB_P40_ALARM     ((agc_job_id_t)(AGC_JOB_GUID_BASE + 65)) /* REP40ALM  */
#define AGC_JOB_P40_ALARM_PRO ((agc_job_id_t)(AGC_JOB_GUID_BASE + 66)) /* REP40ALM PROCEED */
#define AGC_JOB_P30_ALL_DONE  ((agc_job_id_t)(AGC_JOB_GUID_BASE + 67)) /* P30/P31 exit */
#define AGC_JOB_P47BOD        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 68)) /* P47BOD    */
#define AGC_JOB_P40_POSTDISP  ((agc_job_id_t)(AGC_JOB_GUID_BASE + 69)) /* POSTBURN PROCEED */

#define AGC_TASK_GUID_BASE 0x400
#define AGC_TASK_READACCS     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 1))  /* READACCS  */
#define AGC_TASK_PREREAD      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 2))  /* PREREAD   */
#define AGC_TASK_ONMNITOR     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 3))  /* ONMNITOR  */
#define AGC_TASK_QUIKREAD     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 4))  /* QUIKREAD  */
#define AGC_TASK_SETJTAG      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 5))  /* SETJTAG (entry) */
#define AGC_TASK_CLOKTASK     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 6))  /* CLOKTASK  */
#define AGC_TASK_TIGBLNK      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 7))  /* TIGBLNK   */
#define AGC_TASK_TIGAVEG      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 8))  /* TIGAVEG (TIG-30) */
#define AGC_TASK_TIG_5        ((agc_task_id_t)(AGC_TASK_GUID_BASE + 9))  /* TIG-5     */
#define AGC_TASK_TIG_0        ((agc_task_id_t)(AGC_TASK_GUID_BASE + 10)) /* TIG-0     */
#define AGC_TASK_IGNITION     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 11)) /* IGNITION  */
#define AGC_TASK_DOTVCON      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 12)) /* DOTVCON   */
#define AGC_TASK_DOSTRULL     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 13)) /* DOSTRULL  */
#define AGC_TASK_ENGINOFF     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 14)) /* ENGINOFF / ENGOFTSK */
#define AGC_TASK_DOTVCRCS     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 15)) /* DOTVCRCS  */
#define AGC_TASK_TTG_0        ((agc_task_id_t)(AGC_TASK_GUID_BASE + 16)) /* TTG/0     */
#define AGC_TASK_TIGON        ((agc_task_id_t)(AGC_TASK_GUID_BASE + 17)) /* TIGON / STARTP47 */
#define AGC_TASK_S40_6        ((agc_task_id_t)(AGC_TASK_GUID_BASE + 18)) /* S40.6     */
#define AGC_TASK_PRE40_6      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 19)) /* PRE40.6   */
#define AGC_TASK_S40_6_B      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 20)) /* S40.6 after 60 ms */
#define AGC_TASK_S40_6_C      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 21)) /* RSTRTST   */
#define AGC_TASK_S40_6_OUT    ((agc_task_id_t)(AGC_TASK_GUID_BASE + 22)) /* OUTPUT return */
#define AGC_TASK_GIMTRIM2     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 23)) /* GIMTRIM +2 */
#define AGC_TASK_V97TTASK     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 24)) /* V97TTASK  */
#define AGC_TASK_V97TRCS      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 25)) /* V97TRCS   */
#define AGC_TASK_V97PTASK     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 26)) /* V97PTASK  */
#define AGC_TASK_R40ENABL     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 27)) /* R40ENABL  */
#define AGC_TASK_V97ETASK     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 28)) /* V97ETASK  */
#define AGC_TASK_V97E40_6     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 29)) /* V97E40.6  */
#define AGC_TASK_V99FLASH     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 30)) /* V99FLASH  */
#define AGC_TASK_TIG_35       ((agc_task_id_t)(AGC_TASK_GUID_BASE + 31)) /* TIG-35    */
#define AGC_TASK_TIG_30       ((agc_task_id_t)(AGC_TASK_GUID_BASE + 32)) /* TIG-30    */
#define AGC_TASK_TIG_30_1     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 33)) /* TIG-30.1  */
#define AGC_TASK_ULLGTASK     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 34)) /* ULLGTASK  */
#define AGC_TASK_ULLAGOFF     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 35)) /* ULLAGOFF  */
#define AGC_TASK_P63ZOOM      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 36)) /* P63ZOOM   */
#define AGC_TASK_P40ZOOM      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 37)) /* P40ZOOM   */
#define AGC_TASK_TIGTASK      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 38)) /* TIGTASK   */
#define AGC_TASK_COASTSET     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 39)) /* COASTSET  */
#define AGC_TASK_IMPLBURN2    ((agc_task_id_t)(AGC_TASK_GUID_BASE + 40)) /* IMPLBURN after 0.5 s */
#define AGC_TASK_TRIMGIMB     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 41)) /* TRIMGIMB  */
#define AGC_TASK_TRIMGIMB2    ((agc_task_id_t)(AGC_TASK_GUID_BASE + 42)) /* after 60 s */
#define AGC_TASK_ROLLOFF      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 43)) /* roll off (VARDELAY) */
#define AGC_TASK_PITCHOFF     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 44)) /* PITCHOFF  */
#define AGC_TASK_LRHTASK      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 45)) /* LRHTASK   */
#define AGC_TASK_RDGIMS       ((agc_task_id_t)(AGC_TASK_GUID_BASE + 46)) /* RDGIMS    */
#define AGC_TASK_R10R11       ((agc_task_id_t)(AGC_TASK_GUID_BASE + 47)) /* R10,R11 (landing) */
#define AGC_TASK_IMPLCONT     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 48)) /* IMPLCONT  */
#define AGC_TASK_CLOKTASK_LM  ((agc_task_id_t)(AGC_TASK_GUID_BASE + 49)) /* CLOKTASK (LM) */
#define AGC_TASK_IGNITION_LM  ((agc_task_id_t)(AGC_TASK_GUID_BASE + 50)) /* IGNITION (LM) */
#define AGC_TASK_TIG_5_LM     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 51)) /* TIG-5 (LM) */
#define AGC_TASK_TIG_0_LM     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 52)) /* TIG-0 (LM) */

/* Priorities: agc_prio_t = PRIOnn (octal) >> 9, i.e. the "nn" in octal.     */
#define AGC_PRIO5  ((agc_prio_t)005)
#define AGC_PRIO6  ((agc_prio_t)006)
#define AGC_PRIO10 ((agc_prio_t)010)
#define AGC_PRIO12 ((agc_prio_t)012)
#define AGC_PRIO13 ((agc_prio_t)013)
#define AGC_PRIO14 ((agc_prio_t)014)
#define AGC_PRIO15 ((agc_prio_t)015)
#define AGC_PRIO16 ((agc_prio_t)016)
#define AGC_PRIO17 ((agc_prio_t)017)
#define AGC_PRIO20 ((agc_prio_t)020)
#define AGC_PRIO21 ((agc_prio_t)021)
#define AGC_PRIO22 ((agc_prio_t)022)
#define AGC_PRIO23 ((agc_prio_t)023)
#define AGC_PRIO25 ((agc_prio_t)025)
#define AGC_PRIO27 ((agc_prio_t)027)
#define AGC_PRIO30 ((agc_prio_t)030)
#define AGC_PRIO31 ((agc_prio_t)031)
#define AGC_PRIO32 ((agc_prio_t)032)

/* Channel bits used here (AGC_CH<octal chan>_<NAME>, state-flags lane).    */
#define AGC_CH011_TESTCONN  ((uint16_t)0x0100)  /* DSALMOUT BIT9 test connector outbit */
#define AGC_CH011_ENGINEON  ((uint16_t)0x1000)  /* DSALMOUT BIT13 engine on      */
#define AGC_CH011_ENGINEOFF ((uint16_t)0x2000)  /* DSALMOUT BIT14 engine off (LM) */

#endif /* GUID_PENDING_H */
