/*
 * align_pending.h -- interfaces the alignment code (prog-align) uses that
 * other lanes own and had not published when this module was written.
 *
 * INTEGRATION: every declaration below is an ASSUMED interface. When the
 * owning header appears, delete the matching block here and include that
 * header instead (or adapt the call sites). Each block names its owner.
 * Listed again in docs/modules/prog-align.md under "Requested core
 * additions".
 *
 * Job/task identifiers: object-like constants AGC_JOB_<LABEL> /
 * AGC_TASK_<LABEL> with provisional values in the 0x0A00 range; the lead
 * replaces them with real agc_jobs.h members and dispatch.c cases.
 */
#ifndef ALIGN_PENDING_H
#define ALIGN_PENDING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_exec.h"
#include "agc/agc_state.h"

/* ==== state-flags (agc_state.h extension; matches src/core/agc_erasable.c) */
agc_mat_t agc_erase_refsmmat(void);            /* REFSMMAT, rows half-unit  */
void      agc_erase_set_refsmmat(agc_mat_t v);
uint16_t  agc_erase_cdux(void);                /* CDUX..CDUS raw cdu words  */
uint16_t  agc_erase_cduy(void);
uint16_t  agc_erase_cduz(void);
uint16_t  agc_erase_cdut(void);                /* optics trunnion (CM)      */
uint16_t  agc_erase_cdus(void);                /* optics shaft (CM)         */
uint16_t  agc_erase_markstat(void);            /* MARKSTAT                  */
void      agc_erase_set_markstat(uint16_t v);
uint16_t  agc_erase_imodes30(void);            /* IMODES30                  */
uint16_t  agc_erase_thetad(int index);         /* THETAD +index, cdu        */
agc_status_t agc_erase_set_thetad(int index, uint16_t v);
agc_vec_t agc_erase_rn(void);                  /* RN  m B29                 */
void      agc_erase_set_rn(agc_vec_t v);
agc_vec_t agc_erase_vn(void);                  /* VN  m/cs B7               */
void      agc_erase_set_vn(agc_vec_t v);
agc_dp_t  agc_erase_piptime(void);             /* PIPTIME cs B28            */
void      agc_erase_set_piptime(agc_dp_t v);
agc_vec_t agc_erase_rrectcsm(void);            /* RRECTCSM m B29            */
agc_vec_t agc_erase_vrectcsm(void);            /* VRECTCSM m/cs B7          */
agc_vec_t agc_erase_delv(void);                /* DELV PIPA counts          */
void      agc_erase_set_pipax(agc_sp_t v);
void      agc_erase_set_pipay(agc_sp_t v);
void      agc_erase_set_pipaz(agc_sp_t v);

/* Further shared erasables, named by the _common.md convention. */
agc_mat_t agc_erase_xsmd(void);                /* XSMD,YSMD,ZSMD half-unit  */
void      agc_erase_set_xsmd(agc_mat_t v);
uint16_t  agc_erase_extvbact(void);            /* EXTVBACT                  */
void      agc_erase_set_extvbact(uint16_t v);
uint16_t  agc_erase_optmodes(void);            /* OPTMODES (CM)             */
void      agc_erase_set_optmodes(uint16_t v);
void      agc_erase_set_optind(agc_sp_t v);    /* OPTIND (CM)               */
agc_sp_t  agc_erase_option1(int index);        /* OPTION1 +index (N06)      */
void      agc_erase_set_option1(int index, agc_sp_t v);
agc_sp_t  agc_erase_option2(void);             /* OPTION2                   */
void      agc_erase_set_option2(agc_sp_t v);
agc_dp_t  agc_erase_dsptem1(void);             /* DSPTEM1,+1 as DP          */
void      agc_erase_set_dsptem1(agc_dp_t v);
agc_dp_t  agc_erase_samptime(void);            /* SAMPTIME cs B28           */
void      agc_erase_set_samptime(agc_dp_t v);
agc_dp_t  agc_erase_tland(void);               /* TLAND cs B28              */
void      agc_erase_set_tland(agc_dp_t v);
agc_vec_t agc_erase_rls(void);                 /* RLS m B27 moon-fixed      */
void      agc_erase_set_rls(agc_vec_t v);
agc_dp_t  agc_erase_lat(void);                 /* LAT revs B0               */
void      agc_erase_set_lat(agc_dp_t v);
agc_dp_t  agc_erase_long(void);                /* LONG revs B0              */
void      agc_erase_set_long(agc_dp_t v);
agc_dp_t  agc_erase_alt(void);                 /* ALT m B29                 */
void      agc_erase_set_alt(agc_dp_t v);
agc_vec_t agc_erase_alphav(void);              /* ALPHAV                    */
void      agc_erase_set_alphav(agc_vec_t v);
uint16_t  agc_erase_aotaz(int index);          /* AOTAZ +index, cdu (pad)   */
uint16_t  agc_erase_aotel(int index);          /* AOTEL +index, cdu (pad)   */
uint16_t  agc_erase_radmodes(void);            /* RADMODES (LM)             */
agc_sp_t  agc_erase_landmark(void);            /* LANDMARK (CM, ADVTRACK)   */
agc_sp_t  agc_erase_cduchkwd(void);            /* CDUCHKWD cs (CM)          */
agc_sp_t  agc_erase_swsample(void);            /* SWSAMPLE (CM)             */
uint16_t  agc_erase_dnlstcod(void);            /* DNLSTCOD                  */
void      agc_erase_set_dnlstcod(uint16_t v);
uint16_t  agc_erase_agsword(void);             /* AGSWORD                   */
agc_sp_t  agc_erase_imucadr(void);             /* IMUCADR                   */
void      agc_erase_set_rhosb(agc_dp_t v);     /* RHOSB  (CM N51 pitch)     */
void      agc_erase_set_gammasb(agc_dp_t v);   /* GAMMASB (CM N51 yaw)      */
void      agc_erase_set_pitchang(agc_dp_t v);  /* PITCHANG (LM N51)         */
void      agc_erase_set_yawang(agc_dp_t v);    /* YAWANG (LM N51)           */
void      agc_erase_set_fdai(int index, uint16_t v); /* FDAIX,Y,Z (N18)     */
void      agc_erase_set_mrkbuf(int buf, int index, uint16_t v); /* MRKBUF1/2 (R21/R22) */
uint16_t  agc_erase_mrkbuf(int buf, int index);
void      agc_erase_set_markdown(int index, uint16_t v); /* MARKDOWN +0..6  */
void      agc_erase_set_mark2dwn(int index, uint16_t v); /* MARK2DWN +0..6  */
void      agc_erase_set_tsight_lm(agc_dp_t v); /* TSIGHT written by MARKRUPT */
agc_sp_t  agc_erase_1_pipadt(void);
void      agc_erase_set_1_pipadt(agc_sp_t v);  /* 1/PIPADT                  */
void      agc_erase_set_gcomp_zero(void);      /* GCOMP = ZEROVEC           */
agc_dp_t  agc_erase_agsk(void);                /* AGSK cs B28               */
void      agc_erase_set_agsk(agc_dp_t v);
void      agc_erase_set_agsbuff(int index, agc_dp_t v); /* AGSBUFF DP words */

/* ==== io-pinball (agc_dsky.h) ============================================ */
/* Response codes passed as ctx->arg to a display continuation job: the
 * AGC returned to L+1 (terminate), L+2 (proceed) or L+3 (enter).          */
#define AGC_DSKY_RESP_TERMINATE ((agc_sp_t)1)   /* V34                     */
#define AGC_DSKY_RESP_PROCEED   ((agc_sp_t)2)   /* V33                     */
#define AGC_DSKY_RESP_ENTER     ((agc_sp_t)3)   /* ENTER / data loaded     */
#define AGC_DSKY_RESP_RECYCLE   ((agc_sp_t)4)   /* V32 (L+3 with MPAC=V32) */
agc_status_t agc_dsky_goflash(uint16_t verbnoun, agc_job_id_t cont);  /* GOFLASH(R)     */
agc_status_t agc_dsky_goperf1(uint16_t code, agc_job_id_t cont);      /* GOPERF1 V50N25 */
agc_status_t agc_dsky_goperf4(uint16_t code, agc_job_id_t cont);      /* GOPERF4R V04N06*/
agc_status_t agc_dsky_gomark(uint16_t verbnoun, agc_job_id_t cont);   /* GOMARK2/3/4/F/FR */
agc_status_t agc_dsky_godsp(uint16_t verbnoun);                       /* GODSP(R/RET)   */
agc_status_t agc_dsky_exdspret(uint16_t verbnoun);                    /* EXDSPRET       */
agc_status_t agc_dsky_priolarm(uint16_t code, agc_job_id_t cont);     /* PRIOLARM       */
agc_status_t agc_dsky_cleandsp(void);                                 /* CLEANDSP/KLEENEX */
agc_status_t agc_dsky_blanket(uint16_t mask);                         /* BLANKET        */
agc_status_t agc_dsky_clearmrk(void);                                 /* CLEARMRK       */
agc_status_t agc_dsky_endmark(void);                                  /* ENDMARK        */
agc_status_t agc_dsky_endext(void);                                   /* ENDEXT         */
agc_status_t agc_dsky_falton(void);                                   /* FALTON         */
agc_status_t agc_dsky_keycom(void);                                   /* KEYCOM         */
agc_status_t agc_dsky_set_dsptem1_word(uint16_t v);                   /* DSPTEM1 (R1)   */

/* ==== exec-sched ========================================================= */
agc_status_t agc_exec_gotopooh(void);                                 /* GOTOPOOH       */
agc_status_t agc_exec_delayjob(uint16_t delay_cs, agc_job_id_t cont); /* DELAYJOB       */
agc_status_t agc_exec_vac_claim(int *vac_index);                      /* VACnUSE search */
agc_status_t agc_exec_vac_release(int vac_index);                     /* VAC available  */
agc_status_t agc_exec_resume(void);                                   /* RESUME         */

/* ==== imu (agc_imu.h) ==================================================== */
agc_status_t agc_imu_calcgta(agc_interp_t *ip, const agc_mat_t *dc,
                             agc_dp_t gyro[3]);          /* CALCGTA -> OGC,IGC,MGC */
agc_status_t agc_imu_calcga(agc_interp_t *ip, const agc_mat_t *nb,
                            const agc_mat_t *sm, uint16_t thetad[3]); /* CALCGA */
agc_status_t agc_imu_axisgen(agc_interp_t *ip, const agc_vec_t a[2],
                             const agc_vec_t b[2], agc_mat_t *a_wrt_b); /* AXISGEN */
agc_dp_t     agc_imu_arctrig(agc_interp_t *ip, agc_dp_t sinth,
                             agc_dp_t costh);           /* ARCTRIG -> revs      */
agc_status_t agc_imu_r02both(void);      /* R02BOTH: OK, else alarm + P00 done   */
agc_status_t agc_imu_r02zero(void);      /* R02ZERO                              */
agc_status_t agc_imu_coarse(const uint16_t thetad[3]);  /* IMUCOARS             */
agc_status_t agc_imu_fine(void);         /* IMUFINE                              */
agc_status_t agc_imu_fin20(void);        /* IMUFIN20                             */
agc_status_t agc_imu_zero(void);         /* IMUZERO                              */
agc_status_t agc_imu_pulse(const agc_dp_t gyro[3]);     /* IMUPULSE (ECADR OGC) */
agc_status_t agc_imu_stall(agc_job_id_t cont);   /* IMUSTALL: arg 0 bad, 1 good */
agc_status_t agc_imu_optstall(agc_job_id_t cont);/* OPTSTALL/AOTSTALL idem      */
agc_status_t agc_imu_goodend(void);      /* GOODEND (wake stalled job, good)     */
agc_status_t agc_imu_curtains(void);     /* CURTAINS (bad end of a stall)        */
agc_status_t agc_imu_vecpoint(agc_interp_t *ip, agc_vec_t pointvsm,
                              agc_vec_t scaxis, uint16_t gimbal[3]); /* VECPOINT */
agc_status_t agc_imu_ballangs(const uint16_t gimbal[3]);  /* BALLANGS -> FDAI */
agc_status_t agc_imu_r60lem(agc_job_id_t cont);           /* R60LEM           */
agc_status_t agc_imu_one_over_pipa(agc_vec_t *delv);      /* 1/PIPA -> DELV   */
agc_status_t agc_imu_gcompzer(void);                      /* GCOMPZER         */

/* ==== guid-powered (POWERED_FLIGHT_SUBROUTINES) ========================== */
agc_vec_t agc_pfs_smnb(agc_interp_t *ip, const uint16_t cdu[3], agc_vec_t v); /* *SMNB* */
agc_vec_t agc_pfs_nbsm(agc_interp_t *ip, const uint16_t cdu[3], agc_vec_t v); /* *NBSM* */
agc_status_t agc_pfs_pipasr(void);                                            /* PIPASR */

/* ==== nav-integ (agc_integ.h) ============================================ */
/* Conic/precision update to TDEC1; out->r = RATT, out->v = VATT,
 * out->t = TAT, out->moon = (X2 == 2).                                    */
agc_status_t agc_integ_csmconic(agc_interp_t *ip, agc_dp_t tdec1, agc_state_vector_t *out);
agc_status_t agc_integ_lemconic(agc_interp_t *ip, agc_dp_t tdec1, agc_state_vector_t *out);
agc_status_t agc_integ_csmprec(agc_interp_t *ip, agc_dp_t tdec1, agc_state_vector_t *out);
agc_status_t agc_integ_lemprec(agc_interp_t *ip, agc_dp_t tdec1, agc_state_vector_t *out);

/* ==== nav-aux (agc_navaux.h) ============================================= */
agc_status_t agc_lsposition(agc_interp_t *ip, agc_dp_t time_cs,
                            agc_vec_t *moon_pos, agc_vec_t *sun_pos);   /* LSPOS */
agc_status_t agc_star_vector(int index, agc_vec_t *vec);  /* CATLOG star 1..37, half-unit ref */
agc_status_t agc_navaux_rp_to_r(agc_interp_t *ip, agc_vec_t rp, agc_dp_t t,
                                bool moon, agc_vec_t *r);              /* RP-TO-R */
agc_status_t agc_navaux_r_to_rp(agc_interp_t *ip, agc_vec_t r, agc_dp_t t,
                                bool moon, agc_vec_t *rp);             /* R-TO-RP */
agc_status_t agc_navaux_lat_long(agc_interp_t *ip, agc_vec_t alphav, agc_dp_t t,
                                 agc_dp_t *lat, agc_dp_t *lng, agc_dp_t *alt); /* LAT-LONG */
agc_status_t agc_navaux_lalotorv(agc_interp_t *ip, agc_dp_t lat, agc_dp_t lng,
                                 agc_dp_t alt, agc_dp_t t, agc_vec_t *alphav); /* LALOTORV */

/* ==== prog-rendz (agc_programs.h) ======================================== */
agc_status_t agc_rendz_llasrd(void);     /* LLASRD  (P20-P25 p. CM)          */
agc_status_t agc_rendz_llasrda(void);    /* LLASRDA                          */
agc_status_t agc_rendz_r61csm(void);     /* R61CSM                           */

/* ==== guid-landing ======================================================= */
agc_status_t agc_lm_descbits(void);      /* DESCBITS (rate-of-descent switch) */

/* ==== sys-misc / downlink ================================================ */
#define AGC_DNLIST_LAGSLIST ((uint16_t)1) /* LAGSLIST = ONE                  */

/* ==== provisional job and task identifiers ================================ */
#define AGC_JOB_PROG52_RESP      ((agc_job_id_t)0x0A01) /* P52B display return     */
#define AGC_JOB_P52T_RESP        ((agc_job_id_t)0x0A02)
#define AGC_JOB_P52D_RESP        ((agc_job_id_t)0x0A03)
#define AGC_JOB_COARSTYP_RESP    ((agc_job_id_t)0x0A04)
#define AGC_JOB_CAL53A_STALL1    ((agc_job_id_t)0x0A05)
#define AGC_JOB_CAL53A_STALL2    ((agc_job_id_t)0x0A06)
#define AGC_JOB_GYCRS_STALL      ((agc_job_id_t)0x0A07)
#define AGC_JOB_P52C_RESP        ((agc_job_id_t)0x0A08)
#define AGC_JOB_P52I_RESP        ((agc_job_id_t)0x0A09)
#define AGC_JOB_LSDISP_RESP      ((agc_job_id_t)0x0A0A)
#define AGC_JOB_R51DSP_RESP      ((agc_job_id_t)0x0A0B)
#define AGC_JOB_R51A             ((agc_job_id_t)0x0A0C)
#define AGC_JOB_R51_AFTER_R54    ((agc_job_id_t)0x0A0D)
#define AGC_JOB_R51_AFTER_R55    ((agc_job_id_t)0x0A0E)
#define AGC_JOB_R51K_RESP        ((agc_job_id_t)0x0A0F)
#define AGC_JOB_CHKSDATA_RESP    ((agc_job_id_t)0x0A10)
#define AGC_JOB_R55_RESP         ((agc_job_id_t)0x0A11)
#define AGC_JOB_R55_PULSED       ((agc_job_id_t)0x0A12)
#define AGC_JOB_P51_PERF_RESP    ((agc_job_id_t)0x0A13)
#define AGC_JOB_P51_COARSE_STALL ((agc_job_id_t)0x0A14)
#define AGC_JOB_P51_FINE_STALL   ((agc_job_id_t)0x0A15)
#define AGC_JOB_P51C_MARKED      ((agc_job_id_t)0x0A16)
#define AGC_JOB_P51_AFTER_R54    ((agc_job_id_t)0x0A17)
#define AGC_JOB_R53_STALL        ((agc_job_id_t)0x0A18)
#define AGC_JOB_R53C_RESP        ((agc_job_id_t)0x0A19)
#define AGC_JOB_R56_RESP         ((agc_job_id_t)0x0A1A)
#define AGC_JOB_R56A1_RESP       ((agc_job_id_t)0x0A1B)
#define AGC_JOB_R56_PERF_RESP    ((agc_job_id_t)0x0A1C)
#define AGC_JOB_PLANET_RESP      ((agc_job_id_t)0x0A1D)
#define AGC_JOB_R52C             ((agc_job_id_t)0x0A1E)
#define AGC_JOB_R52L_RESP        ((agc_job_id_t)0x0A1F)
#define AGC_JOB_R53JOB           ((agc_job_id_t)0x0A20)
#define AGC_JOB_MKVB51           ((agc_job_id_t)0x0A21)
#define AGC_JOB_MKVB51_RESP      ((agc_job_id_t)0x0A22)
#define AGC_JOB_MARKDISP         ((agc_job_id_t)0x0A23)
#define AGC_JOB_GETDAT           ((agc_job_id_t)0x0A24)
#define AGC_JOB_GETDAT_RESP      ((agc_job_id_t)0x0A25)
#define AGC_JOB_CODE7_RESP       ((agc_job_id_t)0x0A26)
#define AGC_JOB_GETMKS_RESP      ((agc_job_id_t)0x0A27)
#define AGC_JOB_CHANGEVB         ((agc_job_id_t)0x0A28)
#define AGC_JOB_DSPV6N79_RESP    ((agc_job_id_t)0x0A29)
#define AGC_JOB_R51C_RESP        ((agc_job_id_t)0x0A2A)
#define AGC_JOB_R51I_RESP        ((agc_job_id_t)0x0A2B)
#define AGC_JOB_R51_LM_MARKED    ((agc_job_id_t)0x0A2C)
#define AGC_JOB_R51P63_RESP      ((agc_job_id_t)0x0A2D)
#define AGC_JOB_R52A_RESP        ((agc_job_id_t)0x0A2E)
#define AGC_JOB_GETAZEL_RESP     ((agc_job_id_t)0x0A2F)
#define AGC_JOB_R52_AFTER_R60    ((agc_job_id_t)0x0A30)
#define AGC_JOB_R59_RESP         ((agc_job_id_t)0x0A31)
#define AGC_JOB_R59ALM_RESP      ((agc_job_id_t)0x0A32)
#define AGC_JOB_79DISP_RESP      ((agc_job_id_t)0x0A33)
#define AGC_JOB_R59_MARKED       ((agc_job_id_t)0x0A34)
#define AGC_JOB_P57OPT_RESP      ((agc_job_id_t)0x0A35)
#define AGC_JOB_P57AA_RESP       ((agc_job_id_t)0x0A36)
#define AGC_JOB_DSPOPTN_RESP     ((agc_job_id_t)0x0A37)
#define AGC_JOB_OPTALM_RESP      ((agc_job_id_t)0x0A38)
#define AGC_JOB_GVDETER          ((agc_job_id_t)0x0A39)
#define AGC_JOB_GRAVEL           ((agc_job_id_t)0x0A3A)
#define AGC_JOB_LUNG_STALL1      ((agc_job_id_t)0x0A3B)
#define AGC_JOB_LUNG_STALL2      ((agc_job_id_t)0x0A3C)
#define AGC_JOB_ADDGRAV          ((agc_job_id_t)0x0A3D)
#define AGC_JOB_DISGRVER_RESP    ((agc_job_id_t)0x0A3E)
#define AGC_JOB_GYROTRIM_STALL1  ((agc_job_id_t)0x0A3F)
#define AGC_JOB_GYROTRIM_STALL2  ((agc_job_id_t)0x0A40)
#define AGC_JOB_DISPGYRO_RESP    ((agc_job_id_t)0x0A41)
#define AGC_JOB_COATRIM_RESP     ((agc_job_id_t)0x0A42)
#define AGC_JOB_CORSIT_STALL1    ((agc_job_id_t)0x0A43)
#define AGC_JOB_CORSIT_STALL2    ((agc_job_id_t)0x0A44)
#define AGC_JOB_SURFDISP         ((agc_job_id_t)0x0A45)
#define AGC_JOB_P57POST_RESP     ((agc_job_id_t)0x0A46)
#define AGC_JOB_LM_COARSE_STALL1 ((agc_job_id_t)0x0A47)
#define AGC_JOB_LM_COARSE_STALL2 ((agc_job_id_t)0x0A48)
#define AGC_JOB_GYCOARS_STALL    ((agc_job_id_t)0x0A49)
#define AGC_JOB_P51_LM_MARKED    ((agc_job_id_t)0x0A4A)
#define AGC_JOB_SBANDANT_CM_RESP ((agc_job_id_t)0x0A4B)
#define AGC_JOB_SBANDANT_CM      ((agc_job_id_t)0x0A4C)
#define AGC_JOB_SBANDANT_LM_RESP ((agc_job_id_t)0x0A4D)
#define AGC_JOB_SBANDANT_LM      ((agc_job_id_t)0x0A4E)
#define AGC_JOB_V89CALL_RESP     ((agc_job_id_t)0x0A4F)
#define AGC_JOB_V89RECL          ((agc_job_id_t)0x0A50)
#define AGC_JOB_V89CALL1_RESP    ((agc_job_id_t)0x0A51)
#define AGC_JOB_AGSDISPK_RESP    ((agc_job_id_t)0x0A52)
#define AGC_JOB_AGSVCALC_DELAY   ((agc_job_id_t)0x0A53)
#define AGC_JOB_AGSINIT_CKSTALL  ((agc_job_id_t)0x0A54)
#define AGC_JOB_AGSINIT_ZEROED   ((agc_job_id_t)0x0A55)
#define AGC_JOB_AGSEND_RESP      ((agc_job_id_t)0x0A56)
#define AGC_JOB_R52_CM_DELAY     ((agc_job_id_t)0x0A57)
#define AGC_JOB_P52_RETURN       ((agc_job_id_t)0x0A58) /* ENDP50S after R51  */
#define AGC_JOB_P51_RETURN       ((agc_job_id_t)0x0A59)
#define AGC_JOB_R53_FROM_R52     ((agc_job_id_t)0x0A5A)
#define AGC_JOB_P57_SURFLINE     ((agc_job_id_t)0x0A5B)
#define AGC_JOB_P57_AFTER_R54    ((agc_job_id_t)0x0A5C)
#define AGC_JOB_P57_AFTER_TRIM   ((agc_job_id_t)0x0A5D)
#define AGC_JOB_R51_LM_R52DONE   ((agc_job_id_t)0x0A5E)
#define AGC_JOB_P52_LM_AFTER_LS  ((agc_job_id_t)0x0A5F)
#define AGC_JOB_P52_CM_AFTER_LS  ((agc_job_id_t)0x0A60)
#define AGC_JOB_R56_ALT_RETURN   ((agc_job_id_t)0x0A61)
#define AGC_JOB_DSPV6N79         ((agc_job_id_t)0x0A62)

#define AGC_TASK_MARKDIF         ((agc_task_id_t)0x0A01)
#define AGC_TASK_ENDMARKS        ((agc_task_id_t)0x0A02) /* ENDMARKS (CM)     */
#define AGC_TASK_AOT_MKRELEAS    ((agc_task_id_t)0x0A03) /* MKRELEAS (LM)     */
#define AGC_TASK_GRABGRAV        ((agc_task_id_t)0x0A04)

#endif /* ALIGN_PENDING_H */
