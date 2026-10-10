/*
 * entry_pending.h -- PENDING core symbols used by the guid-entry lane.
 *
 * Every declaration here is a request to another lane (see
 * docs/modules/guid-entry.md, "Requested core additions" and "Requested
 * jobs-tasks"). Names follow docs/agents/_common.md so that, at
 * integration, this file is deleted and the includes below are replaced by
 * agc/agc_flags_csm.h, agc/agc_dsky.h, agc/agc_nav.h, agc/agc_imu.h and the
 * extended agc_state.h / agc_jobs.h. Nothing here is DEFINED by this lane.
 *
 * Flag ids: (flagword << 4) | AGC bit number (1..15, BIT1 = LSB), from
 * Comanche055 ERASABLE_ASSIGNMENTS.agc: flag N lives in FLAGWRD(N / 15),
 * BIT(15 - N % 15). The state-flags lane owns the real encoding.
 */
#ifndef AGC_ENTRY_PENDING_H
#define AGC_ENTRY_PENDING_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_state.h"
#include "agc/agc_jobs.h"

/* ---- flags (agc_flags_csm.h, state-flags lane) ------------------------ */
#define AGC_FLAG_CPHIFLAG  ((agc_flag_id_t)((0u << 4) | 15u))  /* 000D */
#define AGC_FLAG_AMOONFLG  ((agc_flag_id_t)((0u << 4) | 2u))   /* 013D */
#define AGC_FLAG_NJETSFLG  ((agc_flag_id_t)((1u << 4) | 15u))  /* 015D */
#define AGC_FLAG_ERADFLAG  ((agc_flag_id_t)((1u << 4) | 13u))  /* 017D */
#define AGC_FLAG_NODOP01   ((agc_flag_id_t)((1u << 4) | 12u))  /* 018D */
#define AGC_FLAG_AVEGFLAG  ((agc_flag_id_t)((1u << 4) | 1u))   /* 029D */
#define AGC_FLAG_XDELVFLG  ((agc_flag_id_t)((2u << 4) | 8u))   /* 037D */
#define AGC_FLAG_FINALFLG  ((agc_flag_id_t)((2u << 4) | 6u))   /* 039D */
#define AGC_FLAG_NODOFLAG  ((agc_flag_id_t)((2u << 4) | 1u))   /* 044D */
#define AGC_FLAG_REFSMFLG  ((agc_flag_id_t)((3u << 4) | 13u))  /* 047D */
#define AGC_FLAG_LUNAFLAG  ((agc_flag_id_t)((3u << 4) | 12u))  /* 048D */
#define AGC_FLAG_RETROFLG  ((agc_flag_id_t)((5u << 4) | 14u))  /* 076D */
#define AGC_FLAG_SLOWFLG   ((agc_flag_id_t)((5u << 4) | 13u))  /* 077D */
#define AGC_FLAG_DAPBIT1   ((agc_flag_id_t)((6u << 4) | 15u))  /* 090D */
#define AGC_FLAG_DAPBIT2   ((agc_flag_id_t)((6u << 4) | 14u))  /* 091D */
#define AGC_FLAG_ENTRYDSP  ((agc_flag_id_t)((6u << 4) | 13u))  /* 092D = STRULLSW */
#define AGC_FLAG_CMDAPARM  ((agc_flag_id_t)((6u << 4) | 12u))  /* 093D */
#define AGC_FLAG_GAMDIFSW  ((agc_flag_id_t)((6u << 4) | 11u))  /* 094D */
#define AGC_FLAG_GONEPAST  ((agc_flag_id_t)((6u << 4) | 10u))  /* 095D */
#define AGC_FLAG_RELVELSW  ((agc_flag_id_t)((6u << 4) | 9u))   /* 096D */
#define AGC_FLAG_EGSW      ((agc_flag_id_t)((6u << 4) | 8u))   /* 097D */
#define AGC_FLAG_NOSWITCH  ((agc_flag_id_t)((6u << 4) | 7u))   /* 098D */
#define AGC_FLAG_HIND      ((agc_flag_id_t)((6u << 4) | 6u))   /* 099D */
#define AGC_FLAG_INRLSW    ((agc_flag_id_t)((6u << 4) | 5u))   /* 100D */
#define AGC_FLAG_LATSW     ((agc_flag_id_t)((6u << 4) | 4u))   /* 101D */
#define AGC_FLAG__05GSW    ((agc_flag_id_t)((6u << 4) | 3u))   /* 102D .05GSW */
#define AGC_FLAG_CM_DSTBY  ((agc_flag_id_t)((6u << 4) | 2u))   /* 103D CM/DSTBY */
#define AGC_FLAG_GYMDIFSW  ((agc_flag_id_t)((6u << 4) | 1u))   /* 104D */
#define AGC_FLAG_NORMSW    ((agc_flag_id_t)((7u << 4) | 10u))  /* 110D */
#define AGC_FLAG_RVSW      ((agc_flag_id_t)((7u << 4) | 9u))   /* 111D */
#define AGC_FLAG_GONEBY    ((agc_flag_id_t)((7u << 4) | 8u))   /* 112D */

/* ---- channel bits (agc_channels.h, state-flags lane) -------------------- */
#define AGC_CH030_SCCONTROL_OF_SATURN ((uint16_t)01000) /* ch 30 BIT10 (P11) */
#define AGC_CH012_SIVB_TAKEOVER       ((uint16_t)00400) /* ch 12 BIT9  (P11) */

/* ---- shared erasables (agc_state.h, state-flags lane) ------------------ */
agc_vec_t agc_erase_unitw(void);     /* UNITW  full unit polar axis, scale 2^0   */
agc_vec_t agc_erase_unitr(void);     /* UNITR  .5 unit RN (SERVICER), scale 2^1  */
agc_vec_t agc_erase_delv(void);      /* DELV   PIPA counts, 5.85 x 2^14 cm/s     */
agc_vec_t agc_erase_delvref(void);   /* DELVREF DELV in ref coords (SERVICER)    */
agc_mat_t agc_erase_refsmmat(void);  /* REFSMMAT .5 unit rows, scale 2^1         */
agc_sp_t  agc_erase_cdux(void);      /* CDUX  2s comp, angle/180 deg             */
agc_sp_t  agc_erase_cduy(void);      /* CDUY                                     */
agc_sp_t  agc_erase_cduz(void);      /* CDUZ                                     */
agc_sp_t  agc_erase_imodes33(void);  /* IMODES33 (BIT6 = CDU fine align mode)    */
agc_tp_t  agc_erase_tephem(void);    /* TEPHEM  cs, TP B42                       */
void      agc_erase_set_tephem(agc_tp_t t);
agc_dp_t  agc_erase_tliftoff(void);  /* TLIFTOFF cs B28                          */
void      agc_erase_set_tliftoff(agc_dp_t t);
void      agc_erase_set_refsmmat(const agc_mat_t *m);
agc_status_t agc_erase_set_avegexit(agc_job_id_t exit_job); /* AVEGEXIT 2CADR  */
void      agc_erase_set_dnlstcod(agc_sp_t code);            /* DNLSTCOD        */

/* ---- DSKY (agc_dsky.h, io-pinball lane) --------------------------------- */
/* GOFLASH(R) / GODSP(R) / REGODSP(R): flash = true for a flashing (response)
 * display. The continuation jobs replace the AGC's TC GOTOPOOH / proceed /
 * enter return slots. AGC_JOB_NONE = no continuation (non-flashing).       */
agc_status_t agc_dsky_display(uint8_t verb, uint8_t noun, bool flash,
                              agc_job_id_t on_terminate,
                              agc_job_id_t on_proceed,
                              agc_job_id_t on_enter);
agc_status_t agc_dsky_clean(void);   /* CLEANDSP                              */
agc_status_t agc_dsky_blank(uint16_t mask); /* BLANKET                        */

/* ---- navigation (agc_nav.h, nav-aux / nav-conic lanes) ------------------ */
/* LALOTORV: lat, long (revs B0), alt (m B29), time (cs B28) -> r (m B29).
 * ERADFLAG/LUNAFLAG as set by the caller.                                   */
agc_status_t agc_nav_lalotorv(agc_dp_t lat, agc_dp_t lng, agc_dp_t alt,
                              agc_dp_t t, agc_vec_t *r_out);
/* LAT-LONG: r (m B29) at time t -> LAT, LONG (revs), ALT (m B29).           */
agc_status_t agc_nav_latlong(agc_vec_t r, agc_dp_t t, agc_dp_t *lat,
                             agc_dp_t *lng, agc_dp_t *alt);
/* GETERAD: .5 sin(lat) in alphav z -> Fischer radius ERADM (m B29).        */
agc_status_t agc_nav_geterad(agc_vec_t alphav, agc_dp_t *eradm);

/* ---- IMU (agc_imu.h, imu lane) ------------------------------------------ */
/* CALCGA: nav-base axes (xnb,ynb,znb .5 unit) and SM rows -> gimbal angles
 * (O,I,M), 2s comp SP angle/180, stored as CPHI (CPHIFLAG clear) or THETAD. */
agc_status_t agc_imu_calcga(agc_vec_t xnb, agc_vec_t ynb, agc_vec_t znb,
                            const agc_mat_t *sm, agc_sp_t gimbal_out[3]);
/* CALCGTA: desired SM axes (xdc,ydc,zdc .5 unit) -> OGC,IGC,MGC (DP revs).  */
agc_status_t agc_imu_calcgta(agc_vec_t xdc, agc_vec_t ydc, agc_vec_t zdc,
                             agc_dp_t ogc_igc_mgc_out[3]);

/* ---- CSM RCS DAP (dap-csm lane) ----------------------------------------- */
/* NEEDLER: FDAI attitude error needles from AK, AK1, AK2 (angle/180 x .25
 * style per caller); init = RCSFLAGS BIT3 initialisation pass.             */
agc_status_t agc_dap_needler(agc_sp_t ak, agc_sp_t ak1, agc_sp_t ak2, bool init);

/* ---- jobs and tasks (agc_jobs.h, lead) ---------------------------------- */
#define AGC_ENTRY_JOB_BASE  ((int)AGC_JOB_COUNT_PLACEHOLDER + 100)
#define AGC_ENTRY_TASK_BASE ((int)AGC_TASK_COUNT_PLACEHOLDER + 100)
#define AGC_JOB_GOTOPOOH  ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 0))
#define AGC_JOB_SERVEXIT  ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 1))
#define AGC_JOB_CM_POSE   ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 2))
#define AGC_JOB_P61       ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 3))
#define AGC_JOB_P61_4     ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 4))
#define AGC_JOB_P61_1     ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 5))
#define AGC_JOB_P61_2     ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 6))
#define AGC_JOB_P61_3     ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 7))
#define AGC_JOB_P62       ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 8))
#define AGC_JOB_P62_1     ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 9))
#define AGC_JOB_P62_2     ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 10))
#define AGC_JOB_P62_2B    ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 11))
#define AGC_JOB_P63       ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 12))
#define AGC_JOB_P65_1     ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 13))
#define AGC_JOB_P65_1B    ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 14))
#define AGC_JOB_P67_1     ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 15))
#define AGC_JOB_P67_1B    ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 16))
#define AGC_JOB_PRE_HUNT  ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 17))
#define AGC_JOB_S61_1A    ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 18))
#define AGC_JOB_CM_DAPON  ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 19))
#define AGC_JOB_P37       ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 20))
#define AGC_JOB_P11       ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 21))
#define AGC_JOB_MATRXJOB  ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 22))
#define AGC_JOB_VHHDOT    ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 23))
#define AGC_JOB_ATERJOB   ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 24))
#define AGC_JOB_NORMLIZE  ((agc_job_id_t)(AGC_ENTRY_JOB_BASE + 25))
#define AGC_TASK_READGYMB ((agc_task_id_t)(AGC_ENTRY_TASK_BASE + 0))
#define AGC_TASK_SETJTAG  ((agc_task_id_t)(AGC_ENTRY_TASK_BASE + 1))
#define AGC_TASK_JETCALL  ((agc_task_id_t)(AGC_ENTRY_TASK_BASE + 2))
#define AGC_TASK_WAKEP62  ((agc_task_id_t)(AGC_ENTRY_TASK_BASE + 3))
#define AGC_TASK_S61_1C   ((agc_task_id_t)(AGC_ENTRY_TASK_BASE + 4))
#define AGC_TASK_ATERTASK ((agc_task_id_t)(AGC_ENTRY_TASK_BASE + 5))
#define AGC_TASK_READACCS ((agc_task_id_t)(AGC_ENTRY_TASK_BASE + 6))

/* Priorities (PRIOnn octal >> 9 is the agc_prio_t convention of agc_exec.h) */
#define AGC_ENTRY_PRIO13 ((agc_prio_t)013)
#define AGC_ENTRY_PRIO16 ((agc_prio_t)016)
#define AGC_ENTRY_PRIO1  ((agc_prio_t)001)

#endif /* AGC_ENTRY_PENDING_H */
