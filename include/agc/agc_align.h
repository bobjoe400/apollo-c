/*
 * agc_align.h -- IMU alignment programs (P51-P54, P57), optics marking
 * (SXTMARK / AOTMARK), CSM and LM geometry, S-band antenna (R05), R63 and
 * AGS initialisation (R47).  Owner: prog-align (docs/modules/prog-align.md).
 *
 * Scale conventions used throughout this header:
 *   half-unit   DP vector of magnitude 0.5 (the AGC UNIT result, B-1)
 *   revs        DP fraction of one revolution, B0 (0.25 = 90 degrees)
 *   cdu         raw 15-bit 2's-complement CDU word, scaled in half
 *               revolutions: 040000 octal = 180 degrees, one count =
 *               360/32768 degrees (CDUX, THETAD, SAC, PAC, ...)
 *   cos/4       a cosine stored as value/4 (dot product of two half-unit
 *               vectors)
 *   cs B28      centiseconds, DP scaled 2^28
 */
#ifndef AGC_ALIGN_H
#define AGC_ALIGN_H

#include <stdbool.h>
#include <stdint.h>
#include "agc_status.h"
#include "agc_word.h"
#include "agc_interp.h"
#include "agc_exec.h"

/* ---- sizes ---------------------------------------------------------------- */
#define AGC_ALIGN_NSTARS       37    /* catalogue stars 1..37 (STAR_TABLES)     */
#define AGC_ALIGN_STAR_CODE_MAX 050  /* star codes 0..47 octal accepted (CHKSCODE) */
#define AGC_ALIGN_MKVAC_WORDS  44    /* mark VAC area (AGC_VAC_WORDS)           */
#define AGC_ALIGN_AGSBUFF_WORDS 14   /* AGSBUFF (14D)                           */

/* ---- CDUTRIG-equivalent result: sines and cosines of the IMU CDUs ------- */
typedef struct agc_align_trig {
    agc_dp_t sinx, siny, sinz;   /* SINCDUX.. scale 2 (value = sin/2)          */
    agc_dp_t cosx, cosy, cosz;   /* COSCDUX.. scale 2 (value = cos/2)          */
    uint16_t cdu[3];             /* CDUSPOT: X, Y, Z CDU words used (cdu)      */
} agc_align_trig_t;

/* ---- LOCSAM outputs (S50, P51-P53) -------------------------------------- */
typedef struct agc_align_locsam {
    agc_vec_t vearth;            /* half-unit LOS to earth centre (ref)        */
    agc_vec_t vsun;              /* half-unit LOS to sun (ref)                 */
    agc_vec_t vmoon;             /* half-unit LOS to moon centre (ref)         */
    agc_dp_t  cearth;            /* occultation cone cosines, cos/4: CEARTH    */
    agc_dp_t  csun;              /*   CSUN                                     */
    agc_dp_t  cmoon;             /*   CMOON                                    */
    agc_vec_t vel_c;             /* VEL/C aberration vector, B-1 (CSM only)    */
} agc_align_locsam_t;

/*
 * The alignment erasables (ERASABLE_ASSIGNMENTS "ALIGNMENT STORAGE",
 * "P50'S, R50'S Q STORES", the overlays on STARAD, and the mark VAC area).
 * DEVIATION: one module-owned block reached through agc_align_ram() instead
 * of one accessor per erasable; these words are shared only by the
 * alignment/marking code and the noun tables (N06 N05 N22 N70 N71 N79 N87
 * N88 N89 N93 N94).
 */
typedef struct agc_align_ram {
    /* orientation matrices, rows are half-unit vectors */
    agc_mat_t xsm;               /* XSM,YSM,ZSM  desired SM (S52.2 output)     */
    agc_mat_t xnb;               /* XNB,YNB,ZNB  nav base wrt SM or ref        */
    agc_mat_t xdc;               /* XDC,YDC,ZDC  desired SM wrt present SM     */
    agc_dp_t  gyro[3];           /* OGC,IGC,MGC  gyro torque angles, revs      */
    agc_dp_t  ogct[3];           /* OGCT  saved torque angles (P57), revs      */
    /* star data */
    agc_vec_t starad[3];         /* STARAD, +6, +12D  half-unit                */
    agc_vec_t starsav1;          /* STARSAV1  1st observed LOS, half-unit      */
    agc_vec_t starsav2;          /* STARSAV2  2nd observed LOS, half-unit      */
    agc_vec_t starsav3;          /* STARSAV3  planet vector (N88), half-unit   */
    agc_vec_t planvec;           /* PLANVEC   1st catalogue LOS, half-unit     */
    agc_vec_t vac6;              /* VAC 6D  catalogue/stored star 1 (R54)      */
    agc_vec_t vac12;             /* VAC 12D catalogue/stored star 2 (R54)      */
    agc_vec_t star;              /* STAR      LOS in SM (CALCSXA input)        */
    agc_vec_t sax;               /* SAX       optics shaft axis (ref)          */
    agc_vec_t vec1, vec2;        /* VEC1, VEC2 P57 reference vectors           */
    agc_vec_t gsav;              /* GSAV  gravity, NB, half-unit (P57)         */
    agc_vec_t gout;              /* GOUT  gravity sample, NB (P57)             */
    agc_vec_t gacc;              /* GACC  accumulated DELV (P57)               */
    agc_vec_t ynbsav, znbsav;    /* YNBSAV ZNBSAV attitude, moon fixed (P57)   */
    agc_vec_t scaxis;            /* SCAXIS  AOT optic axis (NB), half-unit     */
    agc_vec_t pointvsm;          /* POINTVSM  LOS for VECPOINT (SM)            */
    agc_vec_t mark_xp;           /* VAC 18D X-mark plane 1/4 vec (NB) (LM)     */
    agc_vec_t mark_yp;           /* VAC 12D Y-mark plane 1/4 vec (NB) (LM)     */
    agc_vec_t mark_uxp;          /* VAC 24D UXP (OANB) half-unit (LM)          */
    agc_vec_t mark_uyp;          /* VAC 18D UYP (OANB) half-unit (LM)          */
    agc_align_locsam_t locsam;   /* VEARTH VSUN VMOON (STARAD overlay), CSS    */
    agc_sp_t  besti, bestj;      /* BESTI BESTJ  star number * 6               */
    agc_sp_t  starind;           /* STARIND 0 = 1st star, 1 = 2nd             */
    agc_sp_t  markindx;          /* MARKINDX number of marks wanted (CM)       */
    agc_sp_t  starcode;          /* STARCODE (CM N70)/AOTCODE (LM N71) bits 1-6 star, 7-9 detent */
    agc_dp_t  tsight;            /* TSIGHT  time of sighting, cs B28           */
    agc_dp_t  talign;            /* TALIGN  alignment time, cs B28 (LM)        */
    agc_dp_t  theta;             /* THETA   angle, revs (CHKSDATA)             */
    agc_dp_t  theta18;           /* VAC 18D first CHKSDATA angle, revs         */
    agc_dp_t  normtem1;          /* NORMTEM1 star angle difference, revs (N05) */
    agc_dp_t  aoptime;           /* AOPTIME  cs B28 (R52 CM)                    */
    agc_dp_t  dsptem1;           /* DSPTEM1 display temporary (time, cs B28)   */
    /* optics (CM) */
    uint16_t  sac, pac;          /* SAC PAC desired shaft / trunnion, cdu      */
    uint16_t  desoptt, desopts;  /* DESOPTT DESOPTS                            */
    /* AOT (LM) */
    uint16_t  az, el;            /* AZ EL keyed-in optic axis (N87), cdu       */
    uint16_t  cursor, spiral;    /* CURSOR SPIRAL (N79), cdu                   */
    agc_sp_t  poscode;           /* POSCODE AOT detent position 1..6           */
    agc_sp_t  xymark;            /* XYMARK                                     */
    agc_sp_t  markcntr;          /* MARKCNTR mark pair counter (0..4)          */
    agc_sp_t  mkdex;             /* MKDEX                                      */
    agc_sp_t  gctr;              /* GCTR gravity sample counter (P57)          */
    uint16_t  rotang;            /* VAC 10D AOT apparent rotation, cdu         */
    /* mark VAC area (SXTMARK / AOTMARK / R56): see sxtmark.c, aotmark.c */
    uint16_t  mkvac[AGC_ALIGN_MKVAC_WORDS];
    int       mkvac_index;       /* executive VAC claimed, -1 when none        */
    /* return points (QMIN, QMAJ, SAVQR52, R53EXIT, QPRET held a return
     * address in the AGC; here the continuation job)                        */
    agc_job_id_t qmin;           /* QMIN                                       */
    agc_job_id_t qmaj;           /* QMAJ                                       */
    agc_job_id_t savqr52;        /* SAVQR52                                    */
    agc_job_id_t r53exit;        /* R53EXIT                                    */
    agc_sp_t  qpret_marks;       /* QPRET count of marks made (SXTMARK)        */
    agc_sp_t  vflag_last;        /* unused padding for alignment of debug dumps */
} agc_align_ram_t;

agc_align_ram_t *agc_align_ram(void);
void             agc_align_ram_reset(void);

/* ---- word-format helpers (RTB op equivalents) ----------------------------- */
agc_dp_t agc_align_cdulogic(uint16_t cdu);          /* cdu -> revs           */
uint16_t agc_align_1sto2s(agc_dp_t revs);           /* revs -> cdu           */

/* ---- shared alignment math (src/programs/) ------------------------------- */
agc_status_t agc_align_cdutrig(agc_interp_t *ip, const uint16_t cdu[3],
                               agc_align_trig_t *out);
agc_status_t agc_align_calcsmsc(agc_interp_t *ip, const agc_align_trig_t *t,
                                agc_mat_t *xnb);
agc_status_t agc_align_s52_2(agc_interp_t *ip, const agc_mat_t *smd,
                             const agc_mat_t *refsmmat, const uint16_t cdu[3],
                             agc_mat_t *xsm, agc_mat_t *xnb, uint16_t thetad[3]);
agc_status_t agc_align_s52_3(agc_interp_t *ip, agc_dp_t talign,
                             agc_mat_t *smd);
agc_status_t agc_align_lsorient(agc_interp_t *ip, agc_vec_t xsmd,
                                agc_vec_t rcsm, agc_vec_t vcsm, agc_mat_t *smd);
bool         agc_align_cal53a_need_coarse(const uint16_t thetad[3],
                                          const uint16_t cdu[3]);
agc_status_t agc_align_gyro_coarse_dc(agc_interp_t *ip, const agc_mat_t *smd,
                                      const agc_mat_t *refsmmat, agc_mat_t *xdc);
agc_status_t agc_align_locsam(agc_interp_t *ip, agc_dp_t time_cs,
                              agc_align_locsam_t *out);
bool         agc_align_occult(agc_interp_t *ip, const agc_align_locsam_t *ls,
                              agc_vec_t star);
agc_status_t agc_align_picapar(agc_interp_t *ip, const agc_align_locsam_t *ls,
                               const agc_mat_t *refsmmat, const uint16_t cdu[3],
                               agc_sp_t *besti, agc_sp_t *bestj, bool *found);
agc_status_t agc_align_planet_vector(agc_interp_t *ip, agc_sp_t star6,
                                     agc_sp_t starind, agc_dp_t time_cs,
                                     agc_vec_t *los);
agc_status_t agc_align_chksdata_angle(agc_interp_t *ip, const agc_vec_t obs[2],
                                      const agc_vec_t cat[2], agc_dp_t *diff);
agc_status_t agc_align_r51_solve(agc_interp_t *ip, const agc_vec_t cat_ref[2],
                                 const agc_vec_t obs_sm[2],
                                 const agc_mat_t *refsmmat, agc_mat_t *xdc);
agc_status_t agc_align_svdwn(agc_interp_t *ip, bool this_vehicle, int x2,
                             agc_vec_t rcv, agc_vec_t vcv, agc_vec_t tdeltav,
                             agc_vec_t tnuv, agc_dp_t tet);

/* ---- shared alignment jobs (continuations, see agc_jobs.h requests) ------ */
agc_status_t agc_align_chksdata(agc_job_id_t qmin_return);       /* R54      */
agc_status_t agc_job_chksdata_resp(agc_job_ctx_t *ctx);
agc_status_t agc_align_r55(agc_job_id_t qmin_return);            /* R55      */
agc_status_t agc_job_r55_resp(agc_job_ctx_t *ctx);
agc_status_t agc_job_r55_pulsed(agc_job_ctx_t *ctx);

/* ---- CSM geometry (src/csm/csm_geometry.c) ------------------------------- */
agc_dp_t     agc_csm_sxtlogic(uint16_t trunnion_cdu);
agc_status_t agc_csm_sxtnb(agc_interp_t *ip, uint16_t shaft_cdu,
                           uint16_t trunnion_cdu, agc_vec_t *starm_nb);
agc_status_t agc_csm_calcsxa(agc_interp_t *ip, agc_vec_t star_sm,
                             const agc_align_trig_t *t, uint16_t *sac,
                             uint16_t *pac, bool *cult);
agc_status_t agc_csm_sxtang(agc_interp_t *ip, agc_vec_t star,
                            const agc_mat_t *nb, uint16_t *sac,
                            uint16_t *pac, bool *cult);
agc_status_t agc_csm_log(agc_interp_t *ip, agc_dp_t x, agc_dp_t *neg_log);
agc_status_t agc_csm_earrot1(agc_interp_t *ip, agc_dp_t dtearot);
agc_status_t agc_csm_earrot2(agc_interp_t *ip, agc_dp_t dtearot);
agc_status_t agc_csm_earrot_set_inputs(agc_vec_t rtinit, agc_vec_t unitw);
agc_vec_t    agc_csm_earrot_rt(void);
agc_dp_t     agc_csm_earrot_dtearot(void);

/* ---- LM geometry (src/lm/lem_geometry.c) --------------------------------- */
agc_status_t agc_lm_rrangles(agc_interp_t *ip, agc_vec_t los_nb, bool rr_mode2,
                             uint16_t modea[2], uint16_t modeb[2], bool *along_y);
agc_status_t agc_lm_rrnb(agc_interp_t *ip, uint16_t trunnion_cdu,
                         uint16_t shaft_cdu, agc_vec_t *los_nb);

/* ---- S-band antenna R05 --------------------------------------------------- */
agc_status_t agc_sband_cm_angles(agc_interp_t *ip, agc_vec_t r_nb,
                                 agc_dp_t *pitch, agc_dp_t *yaw);
agc_status_t agc_sband_lm_angles(agc_interp_t *ip, agc_vec_t ue_sm,
                                 const uint16_t cdu[3], agc_dp_t *pitch,
                                 agc_dp_t *yaw);
agc_status_t agc_sband_lm_angles_nb(agc_interp_t *ip, agc_vec_t r_nb,
                                    agc_dp_t *pitch, agc_dp_t *yaw);
agc_status_t agc_job_sbandant_cm(agc_job_ctx_t *ctx);
agc_status_t agc_job_sbandant_cm_resp(agc_job_ctx_t *ctx);
agc_status_t agc_job_sbandant_lm(agc_job_ctx_t *ctx);
agc_status_t agc_job_sbandant_lm_resp(agc_job_ctx_t *ctx);

/* ---- R63 (LM) ------------------------------------------------------------ */
agc_status_t agc_r63_los(agc_interp_t *ip, agc_vec_t rcsm, agc_vec_t rlem,
                         const agc_mat_t *refsmmat, agc_vec_t *pointvsm);
agc_status_t agc_job_v89call(agc_job_ctx_t *ctx);
agc_status_t agc_job_v89call_resp(agc_job_ctx_t *ctx);
agc_status_t agc_job_v89recl(agc_job_ctx_t *ctx);
agc_status_t agc_job_v89call1_resp(agc_job_ctx_t *ctx);

/* ---- R47 AGS initialisation (LM) ----------------------------------------- */
agc_status_t agc_ags_scalevec(agc_interp_t *ip, agc_vec_t ratt1, agc_vec_t vatt1,
                              const agc_mat_t *refsmmat,
                              agc_dp_t out[6]);
agc_status_t agc_ags_time(agc_interp_t *ip, agc_dp_t tat, agc_dp_t agsk,
                          agc_dp_t *out);
agc_status_t agc_job_agsinit(agc_job_ctx_t *ctx);
agc_status_t agc_job_agsdispk_resp(agc_job_ctx_t *ctx);
agc_status_t agc_job_agsvcalc_after_delay(agc_job_ctx_t *ctx);
agc_status_t agc_job_agsinit_ckstall(agc_job_ctx_t *ctx);
agc_status_t agc_job_agsinit_zeroed(agc_job_ctx_t *ctx);
agc_status_t agc_job_agsend_resp(agc_job_ctx_t *ctx);
const agc_dp_t *agc_ags_buffer(void);   /* AGSBUFF, 7 DP words, read-only */

/* ---- SXTMARK (CSM) ---------------------------------------------------------- */
agc_status_t agc_sxtmark(agc_sp_t nmarks);                /* SXTMARK       */
agc_status_t agc_sxtmark_reserve(void);                   /* SXTMARK +2    */
agc_status_t agc_sxtmark_mkreleas(void);                  /* MKRELEAS      */
agc_status_t agc_sxtmark_markrupt(void);                  /* MARKRUPT      */
agc_status_t agc_task_markdif(void);                      /* MARKDIF task  */
agc_status_t agc_job_mkvb51(agc_job_ctx_t *ctx);
agc_status_t agc_job_mkvb51_resp(agc_job_ctx_t *ctx);
agc_status_t agc_job_mkvb50(agc_job_ctx_t *ctx);
agc_status_t agc_task_endmarks_csm(void);
agc_status_t agc_job_markdisp(agc_job_ctx_t *ctx);        /* external (P23) */
agc_sp_t     agc_sxtmark_marks_made(void);                /* QPRET count   */
agc_status_t agc_sxtmark_mark_data(int mark, uint16_t data[7]);

/* ---- AOTMARK (LM) ----------------------------------------------------------- */
agc_status_t agc_aotmark(void);                           /* AOTMARK       */
agc_status_t agc_aotmark_markrupt(void);                  /* MARKRUPT      */
agc_status_t agc_task_aot_mkreleas(void);                 /* MKRELEAS task */
agc_status_t agc_job_getdat(agc_job_ctx_t *ctx);
agc_status_t agc_job_getdat_resp(agc_job_ctx_t *ctx);
agc_status_t agc_job_code7_resp(agc_job_ctx_t *ctx);
agc_status_t agc_job_getmks_resp(agc_job_ctx_t *ctx);
agc_status_t agc_job_changevb(agc_job_ctx_t *ctx);
agc_status_t agc_job_dspv6n79_resp(agc_job_ctx_t *ctx);
agc_status_t agc_aot_oanb(agc_interp_t *ip, uint16_t az, uint16_t el,
                          agc_vec_t *oa, agc_vec_t *uxp, agc_vec_t *uyp);
agc_status_t agc_aot_optaxis(agc_interp_t *ip, uint16_t az, uint16_t el,
                             uint16_t rot, agc_vec_t *oa,
                             agc_vec_t *xpnb, agc_vec_t *ypnb);
agc_status_t agc_aot_surfstar(agc_interp_t *ip, const uint16_t mark[6],
                              agc_vec_t oa, agc_vec_t xpnb, agc_vec_t ypnb,
                              agc_vec_t *star_sm);
agc_status_t agc_aot_flight_los(agc_interp_t *ip, const uint16_t xmark_cdu[3],
                                const uint16_t ymark_cdu[3], agc_vec_t xpnb,
                                agc_vec_t ypnb, agc_vec_t *star_sm);
agc_status_t agc_aot_aveit(agc_interp_t *ip, agc_sp_t n, agc_vec_t star,
                           agc_vec_t *ave);

/* ---- P51/P52/P53/P54 job entry points ------------------------------------ */
agc_status_t agc_job_prog52(agc_job_ctx_t *ctx);          /* P52 (and P54 CM) */
agc_status_t agc_job_p51(agc_job_ctx_t *ctx);             /* P51 (and P53 CM) */
agc_status_t agc_job_p57(agc_job_ctx_t *ctx);             /* P57 (LM)        */

#endif /* AGC_ALIGN_H */
