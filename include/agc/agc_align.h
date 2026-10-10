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
 *
 * Job entry points (agc_job_*) are the AGC labels at which a job starts or
 * resumes after a display, a stall or a delay; the lead registers them in
 * agc_jobs.h / dispatch.c (provisional ids in src/programs/align_pending.h).
 */
#ifndef AGC_ALIGN_H
#define AGC_ALIGN_H

#include <stdbool.h>
#include <stdint.h>
#include "agc_status.h"
#include "agc_word.h"
#include "agc_interp.h"
#include "agc_exec.h"
#include "agc_state.h"
#include "agc_guid.h"

/* ---- sizes ---------------------------------------------------------------- */
#define AGC_ALIGN_NSTARS        37   /* catalogue stars 1..37 (STAR_TABLES)     */
#define AGC_ALIGN_STAR_CODE_MAX 050  /* star codes 0..50 octal (CHKSCODE)       */
#define AGC_ALIGN_SXT_MARKS     6    /* SXTMARK marks that fit in a VAC (7 wd)  */
#define AGC_ALIGN_AOT_PAIRS     5    /* AOTMARK mark pairs (MARKCNTR 0..4)      */
#define AGC_ALIGN_AGSBUFF_WORDS 14   /* AGSBUFF (14D)                           */

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

/* One SXTMARK mark (7 words in the mark VAC: MKT2T1, CDUY, CDUS, CDUZ,
 * CDUT, CDUX), CSM.                                                        */
typedef struct agc_align_sxt_mark {
    agc_dp_t t;                  /* MKT2T1 time of mark, cs B28                */
    uint16_t cduy, cdus, cduz, cdut, cdux;   /* raw CDU words (cdu)           */
} agc_align_sxt_mark_t;

/* One AOTMARK mark pair (6 words: X-mark CDUY,Z,X at +0,+2,+4 and Y-mark
 * CDUY,Z,X at +1,+3,+5; a surface mark keeps CURSOR at +1, SPIRAL at +3).  */
typedef struct agc_align_aot_pair {
    uint16_t xcdu[3];            /* X mark: CDUY, CDUZ, CDUX (cdu)             */
    uint16_t ycdu[3];            /* Y mark (flight) or CURSOR, -, SPIRAL       */
} agc_align_aot_pair_t;

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
    agc_mat_t xsmd;              /* XSMD,YSMD,ZSMD desired SM (ref)            */
    agc_mat_t xsm;               /* XSM,YSM,ZSM   desired SM (S52.2 output)    */
    agc_mat_t xnb;               /* XNB,YNB,ZNB   nav base wrt SM or ref       */
    agc_mat_t xdc;               /* XDC,YDC,ZDC   desired SM wrt present SM    */
    agc_dp_t  ogc[3];            /* OGC,IGC,MGC   gyro torque angles, revs     */
    agc_dp_t  ogct[3];           /* OGCT  saved torque angles (P57), revs      */
    /* star data */
    agc_vec_t starad[3];         /* STARAD, +6, +12D  half-unit                */
    agc_vec_t starsav1;          /* STARSAV1  1st observed LOS, half-unit      */
    agc_vec_t starsav2;          /* STARSAV2  2nd observed LOS, half-unit      */
    agc_vec_t starsav3;          /* STARSAV3  planet vector (N88), half-unit   */
    agc_vec_t planvec;           /* PLANVEC   1st catalogue LOS, half-unit     */
    agc_vec_t vac6;              /* VAC 6D  (R54 second pair, star 1)          */
    agc_vec_t vac12;             /* VAC 12D (R54 second pair, star 2)          */
    agc_vec_t star;              /* STAR      LOS (CALCSXA input / temp)       */
    agc_vec_t sax;               /* SAX       optics shaft axis (ref)          */
    agc_vec_t vec1, vec2;        /* VEC1, VEC2 P57 reference vectors           */
    agc_vec_t gsav;              /* GSAV  gravity, NB, half-unit (P57)         */
    agc_vec_t gout;              /* GOUT  gravity sample, NB (P57)             */
    agc_vec_t gacc;              /* GACC  accumulated DELV (P57)               */
    agc_vec_t ynbsav, znbsav;    /* YNBSAV ZNBSAV attitude, moon fixed (P57)   */
    agc_vec_t scaxis;            /* SCAXIS  AOT optic axis (NB), half-unit     */
    agc_vec_t pointvsm;          /* POINTVSM  LOS for VECPOINT (SM)            */
    agc_vec_t xpnb;              /* VAC 18D X-mark plane 1/4 vec (NB) (LM)     */
    agc_vec_t ypnb;              /* VAC 12D Y-mark plane 1/4 vec (NB) (LM)     */
    agc_vec_t uxp;               /* VAC 24D UXP (OANB) half-unit (LM)          */
    agc_vec_t uyp;               /* VAC 18D UYP (OANB) half-unit (LM)          */
    agc_align_locsam_t locsam;   /* VEARTH VSUN VMOON (STARAD overlay), CSS    */
    agc_sp_t  besti, bestj;      /* BESTI BESTJ  star number * 6               */
    agc_sp_t  starind;           /* STARIND 0 = 1st star, 1 = 2nd             */
    agc_sp_t  markindx;          /* MARKINDX number of marks wanted (CM)       */
    agc_sp_t  starcode;          /* STARCODE (N70) / AOTCODE (N71): bits 1-6
                                    star, 7-9 AOT detent (LM)                  */
    agc_dp_t  tsight;            /* TSIGHT  time of sighting, cs B28           */
    agc_dp_t  talign;            /* TALIGN  alignment time, cs B28 (LM)        */
    agc_dp_t  theta;             /* THETA   angle of marked pair, revs         */
    agc_dp_t  theta18;           /* VAC 18D angle of catalogue pair, revs      */
    agc_dp_t  normtem1;          /* NORMTEM1 star angle difference, revs (N05) */
    agc_dp_t  aoptime;           /* AOPTIME  cs B28 (SR52.1) / angle (ROTA)    */
    agc_dp_t  landlat, landlong, landalt; /* N89: LAT revs, LONG/2 revs, ALT m B29 */
    /* optics (CM) */
    uint16_t  sac, pac;          /* SAC PAC desired shaft / trunnion, cdu      */
    uint16_t  desoptt, desopts;  /* DESOPTT DESOPTS desired optics (T4 drive)  */
    /* AOT (LM) */
    uint16_t  az, el;            /* AZ EL keyed-in optic axis (N87), cdu       */
    uint16_t  cursor, spiral;    /* CURSOR SPIRAL (N79), cdu                   */
    uint16_t  rotang;            /* VAC 10D AOT apparent rotation, cdu         */
    agc_sp_t  poscode;           /* POSCODE AOT detent position 1..6           */
    agc_sp_t  xymark;            /* XYMARK  mark kind / detent                 */
    agc_sp_t  markcntr;          /* MARKCNTR mark pair counter (0..4)          */
    agc_sp_t  mkdex;             /* MKDEX                                      */
    agc_sp_t  gctr;              /* GCTR gravity sample counter (P57)          */
    /* mark storage (the mark VAC area) */
    agc_align_sxt_mark_t sxt[AGC_ALIGN_SXT_MARKS]; /* SXTMARK marks             */
    agc_sp_t  qpret;             /* QPRET next mark slot / marks made (CM)     */
    agc_align_aot_pair_t aot[AGC_ALIGN_AOT_PAIRS]; /* AOTMARK mark pairs        */
    int       mkvac_index;       /* VAC area held by the mark system, -1 none  */
    /* return points (QMIN, QMAJ, SAVQR52, R53EXIT, QPRET held a return
     * address in the AGC; here the continuation job)                        */
    agc_job_id_t qmin;           /* QMIN                                       */
    agc_job_id_t qmaj;           /* QMAJ                                       */
    agc_job_id_t savqr52;        /* SAVQR52                                    */
    agc_job_id_t r53exit;        /* R53EXIT                                    */
    agc_job_id_t planet_ret;     /* return of PLANET when it flashes V06N88    */
} agc_align_ram_t;

agc_align_ram_t *agc_align_ram(void);
void             agc_align_ram_reset(void);

/* ---- word-format helpers (RTB op equivalents) ----------------------------- */
agc_dp_t agc_align_cdulogic(uint16_t cdu);          /* CDULOGIC cdu -> revs  */
uint16_t agc_align_1sto2s(agc_dp_t revs);           /* 1STO2S revs -> cdu    */

/* CD*TR*GS of explicit X, Y, Z CDU words (CDUTRIG with a CDUSPOT).        */
agc_status_t agc_align_cdutrig(const uint16_t cdu_xyz[3], agc_cdutrig_t *t);
/* CALCSMSC on CDUTRIG output: rows XNB, YNB, ZNB half-unit wrt the SM.    */
agc_status_t agc_align_calcsmsc(agc_interp_t *ip, const agc_cdutrig_t *t,
                                agc_mat_t *nb);

/* ---- shared alignment math (src/programs/) ------------------------------- */
agc_status_t agc_align_s52_2(agc_interp_t *ip, const agc_mat_t *smd,
                             const agc_mat_t *refsmmat, const uint16_t cdu[3],
                             agc_mat_t *xsm, agc_mat_t *xnb, uint16_t thetad[3]);
/* S52.3 axes from a state (r m, v m/cs, any common scale).                */
agc_status_t agc_align_s52_3_axes(agc_interp_t *ip, agc_vec_t r, agc_vec_t v,
                                  agc_mat_t *smd);
/* S52.3 with the CSMPREC (CM) / LEMCONIC (LM) call; BUSY = job asleep in
 * INTSTALL and resumed as resume_job.                                     */
agc_status_t agc_align_s52_3(agc_interp_t *ip, agc_dp_t talign,
                             agc_job_id_t resume_job, agc_mat_t *smd);
agc_status_t agc_align_lsorient(agc_interp_t *ip, agc_vec_t xsmd,
                                agc_vec_t rcsm, agc_vec_t vcsm, agc_mat_t *smd);
bool         agc_align_cal53a_need_coarse(const uint16_t thetad[3],
                                          const uint16_t cdu[3]);
agc_status_t agc_align_gyro_coarse_dc(agc_interp_t *ip, const agc_mat_t *smd,
                                      const agc_mat_t *refsmmat, agc_mat_t *xdc);
/* LOCSAM from an integrated state and LSPOS output (pure).               */
agc_status_t agc_align_locsam_from(agc_interp_t *ip, const agc_state_vector_t *sv,
                                   agc_vec_t moon, agc_vec_t sun,
                                   agc_align_locsam_t *out);
agc_status_t agc_align_locsam(agc_interp_t *ip, agc_dp_t time_cs,
                              agc_job_id_t resume_job, agc_align_locsam_t *out);
bool         agc_align_occult(agc_interp_t *ip, const agc_align_locsam_t *ls,
                              agc_vec_t star);
agc_status_t agc_align_picapar(agc_interp_t *ip, const agc_align_locsam_t *ls,
                               const agc_mat_t *refsmmat, const uint16_t cdu[3],
                               agc_sp_t *besti, agc_sp_t *bestj, bool *found);
/* PLANET for a star6 that needs no display (catalogue, sun, earth, moon,
 * or a planet whose vector is already in STARSAV3 / STARAD).              */
agc_status_t agc_align_planet_vector(agc_interp_t *ip, agc_sp_t star6,
                                     agc_sp_t starind, agc_dp_t time_cs,
                                     agc_job_id_t resume_job, agc_vec_t *los);
agc_status_t agc_align_chksdata_angle(agc_interp_t *ip, const agc_vec_t cat[2],
                                      const agc_vec_t obs[2], agc_dp_t *diff);
agc_status_t agc_align_r51_solve(agc_interp_t *ip, const agc_vec_t a[2],
                                 const agc_vec_t b[2], agc_mat_t *xdc);
/* CHKSCODE: true when 0 <= STARCODE <= 50 octal.                          */
bool         agc_align_chkscode(agc_sp_t starcode);

/* SVDWN1/SVDWN2 (CSM_GEOMETRY / LEM_GEOMETRY), implemented in src/nav/
 * svdwn.c because the integration package (nav) calls it from ATOPCSM /
 * ATOPLEM.  this_vehicle: RN, VN, PIPTIME (skipped while AVEMIDSW);
 * otherwise R-OTHER, V-OTHER.  x2: 0 earth (B29/B7), 2 moon (B27/B5).    */
agc_status_t agc_align_svdwn(agc_interp_t *ip, bool this_vehicle, int x2,
                             agc_vec_t rcv, agc_vec_t vcv, agc_vec_t tdeltav,
                             agc_vec_t tnuv, agc_dp_t tet);
agc_vec_t    agc_align_r_other(void);    /* R-OTHER  m B29 (downlink)      */
agc_vec_t    agc_align_v_other(void);    /* V-OTHER  m/cs B7 (downlink)    */

/* ---- shared alignment jobs: R54 (CHKSDATA) and R55 (src/programs/) ------ */
agc_status_t agc_align_r54(const agc_job_ctx_t *ctx, agc_job_id_t qmin_return);
agc_status_t agc_job_chksdata_proceed(agc_job_ctx_t *ctx);   /* CHKSDA      */
agc_status_t agc_job_chksdata_enter(agc_job_ctx_t *ctx);     /* V32: CLEAR  */
agc_status_t agc_align_r55(const agc_job_ctx_t *ctx, agc_job_id_t qmin_return);
agc_status_t agc_job_r55_1(agc_job_ctx_t *ctx);              /* R55.1       */
agc_status_t agc_job_r55_2(agc_job_ctx_t *ctx);              /* R55.2       */
agc_status_t agc_job_r55_pulsed(agc_job_ctx_t *ctx);         /* after STALL */
agc_status_t agc_job_r55ret(agc_job_ctx_t *ctx);             /* R55RET      */

/* ---- CSM geometry (src/csm/csm_geometry.c) ------------------------------- */
agc_dp_t     agc_csm_sxtlogic(uint16_t trunnion_cdu);
agc_status_t agc_csm_sxtnb(agc_interp_t *ip, uint16_t trunnion_cdu,
                           uint16_t shaft_cdu, agc_vec_t *starm_nb);
agc_status_t agc_csm_calcsxa(agc_interp_t *ip, agc_vec_t star_sm,
                             const agc_cdutrig_t *t, uint16_t *sac,
                             uint16_t *pac, bool *cult);
agc_status_t agc_csm_sxtang(agc_interp_t *ip, agc_vec_t star,
                            const agc_mat_t *nb, uint16_t *sac,
                            uint16_t *pac, bool *cult);
agc_status_t agc_csm_log(agc_interp_t *ip, agc_dp_t x, agc_dp_t *neg_log);
typedef struct agc_csm_earrot {
    agc_vec_t rtinit;            /* RTINIT  half-unit                          */
    agc_vec_t unitw;             /* UNITW   full unit (earth rotation axis)    */
    agc_vec_t rteast;            /* RTEAST  half-unit                          */
    agc_vec_t rtnorm;            /* RTNORM  half-unit                          */
    agc_dp_t  dtearot;           /* DTEAROT cs B28                             */
    agc_vec_t rt;                /* RT      half-unit target vector            */
} agc_csm_earrot_t;
agc_status_t agc_csm_earrot1(agc_interp_t *ip, agc_csm_earrot_t *e);
agc_status_t agc_csm_earrot2(agc_interp_t *ip, agc_csm_earrot_t *e);

/* ---- LM geometry (src/lm/lem_geometry.c) --------------------------------- */
/* RRANGLES: los_nb half-unit; modea/modeb = (trunnion, shaft) cdu for RR
 * mode 1 and mode 2 (swapped when RADMODES antenna bit says mode 2);
 * *along_y when the LOS is along Y (the AGC's BOVB LUNDESCH).             */
agc_status_t agc_lm_rrangles(agc_interp_t *ip, agc_vec_t los_nb, bool rr_mode2,
                             uint16_t modea[2], uint16_t modeb[2], bool *along_y);
agc_status_t agc_lm_rrnb(agc_interp_t *ip, uint16_t trunnion_cdu,
                         uint16_t shaft_cdu, agc_vec_t *los_nb);

/* ---- S-band antenna R05 --------------------------------------------------- */
/* CM: r_nb = vehicle-to-earth vector in NB axes (any scale) -> RHOSB pitch,
 * GAMMASB yaw (revs).  LM: idem -> PITCHANG, YAWANG (revs).               */
agc_status_t agc_sband_cm_angles(agc_interp_t *ip, agc_vec_t r_nb,
                                 agc_dp_t *pitch, agc_dp_t *yaw);
agc_status_t agc_sband_lm_angles(agc_interp_t *ip, agc_vec_t r_nb,
                                 agc_dp_t *pitch, agc_dp_t *yaw);
agc_dp_t     agc_sband_rhosb(void);     /* RHOSB  (CM N51 pitch), revs    */
agc_dp_t     agc_sband_gammasb(void);   /* GAMMASB (CM N51 yaw), revs     */
agc_dp_t     agc_sband_pitchang(void);  /* PITCHANG (LM N51), revs        */
agc_dp_t     agc_sband_yawang(void);    /* YAWANG   (LM N51), revs        */
agc_status_t agc_job_sbandant_cm(agc_job_ctx_t *ctx);        /* SBANDANT CM */
agc_status_t agc_job_sbandant_cm_recycle(agc_job_ctx_t *ctx);/* SBANDANT+2  */
agc_status_t agc_job_sbandant_lm(agc_job_ctx_t *ctx);        /* SBANDANT LM */
agc_status_t agc_job_sbandant_lm_recycle(agc_job_ctx_t *ctx);/* SBANDANT+2  */
agc_status_t agc_job_sband_b5off(agc_job_ctx_t *ctx);        /* TC B5OFF    */

/* ---- R63 (LM) ------------------------------------------------------------ */
agc_status_t agc_r63_los(agc_interp_t *ip, agc_vec_t rcsm, agc_vec_t rlem,
                         const agc_mat_t *refsmmat, agc_vec_t *pointvsm);
agc_status_t agc_job_v89call(agc_job_ctx_t *ctx);            /* V89CALL     */
agc_status_t agc_job_v89recl(agc_job_ctx_t *ctx);            /* V89RECL     */
agc_status_t agc_job_v89_maneuver(agc_job_ctx_t *ctx);       /* V06N18 V33  */

/* ---- R47 AGS initialisation (LM) ----------------------------------------- */
agc_status_t agc_ags_scalevec(agc_interp_t *ip, agc_vec_t ratt1, agc_vec_t vatt1,
                              bool moon, const agc_mat_t *refsmmat,
                              uint16_t out[6]);
agc_status_t agc_ags_time(agc_interp_t *ip, agc_dp_t tat, agc_dp_t agsk,
                          agc_dp_t *out);
const uint16_t *agc_ags_buffer(void);    /* AGSBUFF, 14 raw words           */
agc_dp_t     agc_ags_agsk(void);         /* AGSK cs B28                     */
agc_status_t agc_ags_set_agsk(agc_dp_t t);
agc_status_t agc_job_agsinit(agc_job_ctx_t *ctx);            /* AGSINIT     */
agc_status_t agc_job_agsdispk(agc_job_ctx_t *ctx);           /* AGSDISPK    */
agc_status_t agc_job_agsdispk_enter(agc_job_ctx_t *ctx);     /* V32 / load  */
agc_status_t agc_job_agsvcalc(agc_job_ctx_t *ctx);           /* AGSVCALC    */
agc_status_t agc_job_ags_lemprec(agc_job_ctx_t *ctx);        /* INTSTALL+1  */
agc_status_t agc_job_ags_csmprec(agc_job_ctx_t *ctx);        /* INTSTALL+1  */
agc_status_t agc_job_ags_after_delay(agc_job_ctx_t *ctx);    /* 20 s later  */
agc_status_t agc_job_ags_ckstall(agc_job_ctx_t *ctx);        /* CKSTALL     */
agc_status_t agc_job_agsend(agc_job_ctx_t *ctx);             /* AGSEND      */

/* ---- SXTMARK (CSM, src/csm/sxtmark.c) ------------------------------------ */
agc_status_t agc_sxtmark(agc_sp_t nmarks);                /* SXTMARK       */
agc_status_t agc_sxtmark_reserve(void);                   /* SXTMARK +2    */
agc_status_t agc_sxtmark_mkreleas(void);                  /* MKRELEAS      */
agc_status_t agc_sxtmark_markrupt(void);                  /* MARKRUPT      */
agc_status_t agc_task_markdif(void);                      /* MARKDIF task  */
agc_status_t agc_task_endmarks_csm(void);                 /* ENDMARKS task */
agc_status_t agc_job_mkvb51(agc_job_ctx_t *ctx);          /* MKVB51        */
agc_status_t agc_job_mkvb50(agc_job_ctx_t *ctx);          /* MKVB50        */
agc_status_t agc_job_termsxt(agc_job_ctx_t *ctx);         /* TERMSXT       */
agc_status_t agc_job_entanswr(agc_job_ctx_t *ctx);        /* ENTANSWR      */
agc_status_t agc_job_mkvb5x(agc_job_ctx_t *ctx);          /* MKVB5X        */
agc_sp_t     agc_sxtmark_marks_made(void);                /* QPRET count   */
agc_status_t agc_sxtmark_mark(int mark, agc_align_sxt_mark_t *out);

/* ---- AOTMARK (LM, src/lm/aotmark.c) -------------------------------------- */
agc_status_t agc_aotmark(void);                           /* AOTMARK       */
agc_status_t agc_aotmark_markrupt(void);                  /* MARKRUPT      */
agc_status_t agc_task_aot_mkreleas(void);                 /* MKRELEAS task */
agc_status_t agc_job_getdat(agc_job_ctx_t *ctx);          /* GETDAT        */
agc_status_t agc_job_dodat(agc_job_ctx_t *ctx);           /* DODAT         */
agc_status_t agc_job_code7(agc_job_ctx_t *ctx);           /* CODE7         */
agc_status_t agc_job_code7_proceed(agc_job_ctx_t *ctx);   /* CODE7 +5      */
agc_status_t agc_job_getmks(agc_job_ctx_t *ctx);          /* GETMKS        */
agc_status_t agc_job_markchex(agc_job_ctx_t *ctx);        /* MARKCHEX      */
agc_status_t agc_job_killaot(agc_job_ctx_t *ctx);         /* KILLAOT       */
agc_status_t agc_job_changevb(agc_job_ctx_t *ctx);        /* CHANGEVB      */
agc_status_t agc_job_dspv6n79(agc_job_ctx_t *ctx);        /* DSPV6N79      */
agc_status_t agc_job_surfend(agc_job_ctx_t *ctx);         /* SURFEND       */
agc_status_t agc_job_dspv6n79_enter(agc_job_ctx_t *ctx);  /* V32 / ENTER   */
agc_status_t agc_aot_oanb(agc_interp_t *ip, uint16_t az, uint16_t el,
                          agc_vec_t *oa, agc_vec_t *uxp, agc_vec_t *uyp);
agc_status_t agc_aot_optaxis(agc_interp_t *ip, agc_vec_t uxp, agc_vec_t uyp,
                             uint16_t rot, agc_vec_t *xpnb, agc_vec_t *ypnb);
agc_status_t agc_aot_surfstar(agc_interp_t *ip, const agc_align_aot_pair_t *m,
                              agc_vec_t oa, agc_vec_t uxp, agc_vec_t uyp,
                              agc_vec_t *star_sm);
agc_status_t agc_aot_flight_los(agc_interp_t *ip, const agc_align_aot_pair_t *m,
                                agc_vec_t xpnb, agc_vec_t ypnb,
                                agc_vec_t *star_sm);
agc_status_t agc_aot_aveit(agc_interp_t *ip, agc_sp_t n, agc_vec_t star,
                           agc_vec_t *ave);

/* ---- CSM P51/P52/P53/P54 and R51-R53, R56 (src/csm/) ---------------------- */
agc_status_t agc_job_prog52(agc_job_ctx_t *ctx);          /* PROG52 = P54  */
agc_status_t agc_job_p52b(agc_job_ctx_t *ctx);            /* P52B          */
agc_status_t agc_job_p52_option(agc_job_ctx_t *ctx);      /* P52B +9 (V33) */
agc_status_t agc_job_p52t(agc_job_ctx_t *ctx);            /* P52T          */
agc_status_t agc_job_p52t_proceed(agc_job_ctx_t *ctx);    /* P52T +10      */
agc_status_t agc_job_p52_s52_3(agc_job_ctx_t *ctx);       /* S52.3 call    */
agc_status_t agc_job_p52d(agc_job_ctx_t *ctx);            /* P52D          */
agc_status_t agc_job_coarstyp(agc_job_ctx_t *ctx);        /* COARSTYP      */
agc_status_t agc_job_cal53a(agc_job_ctx_t *ctx);          /* CAL53A        */
agc_status_t agc_job_cal53a_coarse_done(agc_job_ctx_t *ctx); /* IMUCOARS stall */
agc_status_t agc_job_cal53a_fine_done(agc_job_ctx_t *ctx);   /* IMUFIN20 stall */
agc_status_t agc_job_gycrs(agc_job_ctx_t *ctx);           /* GYCRS/GYCOARS */
agc_status_t agc_job_gycrs_pulsed(agc_job_ctx_t *ctx);    /* after STALL   */
agc_status_t agc_job_p52c(agc_job_ctx_t *ctx);            /* P52C / R51C   */
agc_status_t agc_job_p52c_proceed(agc_job_ctx_t *ctx);    /* LOCSAM PICAPAR*/
agc_status_t agc_job_p52f(agc_job_ctx_t *ctx);            /* P52F -> R51   */
agc_status_t agc_job_p52ls(agc_job_ctx_t *ctx);           /* P52LS         */
agc_status_t agc_job_lsdisp(agc_job_ctx_t *ctx);          /* LSDISP        */
agc_status_t agc_job_lsdisp_proceed(agc_job_ctx_t *ctx);  /* LSDISP +5     */
agc_status_t agc_job_p52ls_prec(agc_job_ctx_t *ctx);      /* CSMPREC call  */
agc_status_t agc_job_endp50s(agc_job_ctx_t *ctx);         /* ENDP50S       */
agc_status_t agc_job_p51(agc_job_ctx_t *ctx);             /* P51 = P53     */
agc_status_t agc_job_p51aa(agc_job_ctx_t *ctx);           /* P51AA         */
agc_status_t agc_job_p51_coarse(agc_job_ctx_t *ctx);      /* V33 skipped   */
agc_status_t agc_job_p51_coarse_done(agc_job_ctx_t *ctx); /* IMUCOARS stall*/
agc_status_t agc_job_p51_fine_done(agc_job_ctx_t *ctx);   /* IMUFIN20 stall*/
agc_status_t agc_job_p51b(agc_job_ctx_t *ctx);            /* P51B          */
agc_status_t agc_job_p51c(agc_job_ctx_t *ctx);            /* P51C          */
agc_status_t agc_job_p51c2(agc_job_ctx_t *ctx);           /* P51C.2        */
agc_status_t agc_job_p51e(agc_job_ctx_t *ctx);            /* P51E          */
agc_status_t agc_job_p51f(agc_job_ctx_t *ctx);            /* P51F          */
agc_status_t agc_job_p51g(agc_job_ctx_t *ctx);            /* P51G          */
agc_status_t agc_job_r51(agc_job_ctx_t *ctx);             /* R51 / R51.3   */
agc_status_t agc_job_r51dsp(agc_job_ctx_t *ctx);          /* R51DSP        */
agc_status_t agc_job_r51dsp_proceed(agc_job_ctx_t *ctx);  /* CHKSCODE      */
agc_status_t agc_job_r51st(agc_job_ctx_t *ctx);           /* R51ST         */
agc_status_t agc_job_r51a(agc_job_ctx_t *ctx);            /* R51A          */
agc_status_t agc_job_r51a_planet(agc_job_ctx_t *ctx);     /* after PLANET  */
agc_status_t agc_job_r51_axisgen(agc_job_ctx_t *ctx);     /* after R54     */
agc_status_t agc_job_r51k(agc_job_ctx_t *ctx);            /* R51K          */
agc_status_t agc_job_planet_flash(agc_job_ctx_t *ctx);    /* PLANET V06N88 */
agc_status_t agc_job_planet_loaded(agc_job_ctx_t *ctx);   /* PLANET +N88   */
agc_status_t agc_job_r52(agc_job_ctx_t *ctx);             /* R52 (CM)      */
agc_status_t agc_job_r52c(agc_job_ctx_t *ctx);            /* R52C          */
agc_status_t agc_job_r52f(agc_job_ctx_t *ctx);            /* R52F delay    */
agc_status_t agc_job_r52_after_delay(agc_job_ctx_t *ctx); /* R52F +3       */
agc_status_t agc_job_r52_r61_done(agc_job_ctx_t *ctx);    /* after R61CSM  */
agc_status_t agc_job_term52(agc_job_ctx_t *ctx);          /* TERM52        */
agc_status_t agc_job_r53job(agc_job_ctx_t *ctx);          /* R53JOB        */
agc_status_t agc_job_r53a(agc_job_ctx_t *ctx);            /* R53A          */
agc_status_t agc_job_r53_marked(agc_job_ctx_t *ctx);      /* OPTSTALL good */
agc_status_t agc_job_r53c1(agc_job_ctx_t *ctx);           /* R53C1         */
agc_status_t agc_job_r53c(agc_job_ctx_t *ctx);            /* R53C          */
agc_status_t agc_job_r53z(agc_job_ctx_t *ctx);            /* R53Z          */
agc_status_t agc_job_r53d(agc_job_ctx_t *ctx);            /* R53D          */
agc_status_t agc_job_r56(agc_job_ctx_t *ctx);             /* R56 (CM)      */
agc_status_t agc_job_r56a(agc_job_ctx_t *ctx);            /* R56A          */
agc_status_t agc_job_r56a1(agc_job_ctx_t *ctx);           /* R56A1         */
agc_status_t agc_job_r56_mark(agc_job_ctx_t *ctx);        /* R56A1 ENTER   */
agc_status_t agc_job_r56_recycle(agc_job_ctx_t *ctx);     /* R56A +2       */
agc_status_t agc_csm_sxtsm(agc_interp_t *ip, const agc_align_sxt_mark_t *m,
                           agc_vec_t *los_sm);            /* SXTSM         */
/* SR52.1: compute SAC/PAC for the target; result 0 = trunnion > 90 deg,
 * 1 = > 50 deg, 2 = < 50 deg (the AGC's three returns).                   */
agc_status_t agc_csm_sr52_1(agc_interp_t *ip, agc_job_id_t resume_job,
                            int *result);
agc_status_t agc_csm_advtrack(agc_interp_t *ip, agc_job_id_t resume_job,
                              agc_vec_t *los);            /* ADVTRACK      */
agc_vec_t    agc_csm_rota(agc_interp_t *ip, agc_vec_t axis, agc_vec_t los,
                          agc_dp_t angle);                /* ROTA          */

/* ---- LM P51/P52/P57, R51, R52, R59 (src/lm/) ------------------------------ */
agc_status_t agc_job_lm_p52t_proceed(agc_job_ctx_t *ctx); /* P52T (LM)     */
agc_status_t agc_job_lm_p52ls(agc_job_ctx_t *ctx);        /* P52LS (LM)    */
agc_status_t agc_job_n89disp(agc_job_ctx_t *ctx);         /* N89DISP LSDISP*/
agc_status_t agc_job_n89disp_proceed(agc_job_ctx_t *ctx); /* LSDISP V33    */
agc_status_t agc_job_lm_coarse_done(agc_job_ctx_t *ctx);  /* COARSE stall 1*/
agc_status_t agc_job_lm_fine_done(agc_job_ctx_t *ctx);    /* COARSE stall 2*/
agc_status_t agc_job_lm_r51(agc_job_ctx_t *ctx);          /* R51 (LM)      */
agc_status_t agc_job_lm_r51_locsam(agc_job_ctx_t *ctx);   /* R51C V33      */
agc_status_t agc_job_lm_r51e(agc_job_ctx_t *ctx);         /* R51E          */
agc_status_t agc_job_lm_r51_3(agc_job_ctx_t *ctx);        /* R51.3         */
agc_status_t agc_job_lm_r51_marked(agc_job_ctx_t *ctx);   /* after AOTMARK */
agc_status_t agc_job_lm_r51_planet(agc_job_ctx_t *ctx);   /* after PLANET  */
agc_status_t agc_job_lm_r51_4(agc_job_ctx_t *ctx);        /* R51.4 PLANET  */
agc_status_t agc_job_lm_r51k(agc_job_ctx_t *ctx);         /* R51K/R51P63   */
agc_status_t agc_job_lm_r52(agc_job_ctx_t *ctx);          /* R52 (LM)      */
agc_status_t agc_job_lm_r52a(agc_job_ctx_t *ctx);         /* R52A          */
agc_status_t agc_job_lm_r52b(agc_job_ctx_t *ctx);         /* R52B          */
agc_status_t agc_job_getazel(agc_job_ctx_t *ctx);         /* GETAZEL       */
agc_status_t agc_job_azel_keyed(agc_job_ctx_t *ctx);      /* GETAZEL +5    */
agc_status_t agc_job_lm_r52_planet(agc_job_ctx_t *ctx);   /* after PLANET  */
agc_status_t agc_job_lm_r52_r60done(agc_job_ctx_t *ctx);  /* after R60LEM  */
agc_status_t agc_job_lm_p51(agc_job_ctx_t *ctx);          /* P51 (LM)      */
agc_status_t agc_job_lm_p51_perf(agc_job_ctx_t *ctx);     /* P51 +2        */
agc_status_t agc_job_lm_p51_coarse(agc_job_ctx_t *ctx);   /* P51 coarse    */
agc_status_t agc_job_lm_p51_coarsed(agc_job_ctx_t *ctx);  /* after COARSE  */
agc_status_t agc_job_lm_p51b(agc_job_ctx_t *ctx);         /* P51B          */
agc_status_t agc_job_lm_p51c(agc_job_ctx_t *ctx);         /* P51C          */
agc_status_t agc_job_lm_p51_marked(agc_job_ctx_t *ctx);   /* after AOTMARK */
agc_status_t agc_job_lm_p51_planet(agc_job_ctx_t *ctx);   /* P51D PLANET   */
agc_status_t agc_job_lm_p51e(agc_job_ctx_t *ctx);         /* P51E          */
agc_status_t agc_job_lm_p51g(agc_job_ctx_t *ctx);         /* P51G          */
agc_status_t agc_job_r59(agc_job_ctx_t *ctx);             /* R59           */
agc_status_t agc_job_r59a(agc_job_ctx_t *ctx);            /* R59A          */
agc_status_t agc_job_r59e(agc_job_ctx_t *ctx);            /* R59E          */
agc_status_t agc_job_r59out(agc_job_ctx_t *ctx);          /* R59OUT        */
agc_status_t agc_job_r59ret(agc_job_ctx_t *ctx);          /* R59RET        */
agc_status_t agc_job_p57(agc_job_ctx_t *ctx);             /* P57           */
agc_status_t agc_job_p57opt(agc_job_ctx_t *ctx);          /* P57OPT        */
agc_status_t agc_job_alignopt(agc_job_ctx_t *ctx);        /* ALIGNOPT      */
agc_status_t agc_job_p57aa(agc_job_ctx_t *ctx);           /* P57AA         */
agc_status_t agc_job_p57aa_proceed(agc_job_ctx_t *ctx);   /* P57AA +5      */
agc_status_t agc_job_p57d_done(agc_job_ctx_t *ctx);       /* LEMPREC +1    */
agc_status_t agc_job_packoptn(agc_job_ctx_t *ctx);        /* PACKOPTN      */
agc_status_t agc_job_dspoptn(agc_job_ctx_t *ctx);         /* DSPOPTN       */
agc_status_t agc_job_dspoptn_proceed(agc_job_ctx_t *ctx); /* DSPOPTN +5    */
agc_status_t agc_job_gvdeter(agc_job_ctx_t *ctx);         /* GVDETER       */
agc_status_t agc_job_gravel(agc_job_ctx_t *ctx);          /* GRAVEL        */
agc_status_t agc_job_gvdeter_lung2(agc_job_ctx_t *ctx);   /* 2nd LUNG ret  */
agc_status_t agc_job_prograv(agc_job_ctx_t *ctx);         /* PROGRAV       */
agc_status_t agc_job_prograv_recycle(agc_job_ctx_t *ctx); /* V32 FREEFLAG  */
agc_status_t agc_job_lung(agc_job_ctx_t *ctx);            /* LUNG          */
agc_status_t agc_job_lung_coarse_done(agc_job_ctx_t *ctx);
agc_status_t agc_job_lung_fine_done(agc_job_ctx_t *ctx);
agc_status_t agc_task_grabgrav(void);                     /* GRABGRAV task */
agc_status_t agc_job_addgrav(agc_job_ctx_t *ctx);         /* ADDGRAV       */
agc_status_t agc_job_attchk(agc_job_ctx_t *ctx);          /* ATTCHK        */
agc_status_t agc_job_p57jump(agc_job_ctx_t *ctx);         /* P57JUMP       */
agc_status_t agc_job_surfline(agc_job_ctx_t *ctx);        /* SURFLINE      */
agc_status_t agc_job_surfline_r54(agc_job_ctx_t *ctx);    /* after R54     */
agc_status_t agc_job_5degtest(agc_job_ctx_t *ctx);        /* 5DEGTEST      */
agc_status_t agc_job_surfdisp(agc_job_ctx_t *ctx);        /* SURFDISP      */
agc_status_t agc_job_p57post(agc_job_ctx_t *ctx);         /* P57POST       */
agc_status_t agc_job_p57post_proc(agc_job_ctx_t *ctx);    /* P57POST +5    */
agc_status_t agc_job_p57post_ls(agc_job_ctx_t *ctx);      /* after N89DISP */
agc_status_t agc_job_coatrim(agc_job_ctx_t *ctx);         /* COATRIM       */
agc_status_t agc_job_corsit(agc_job_ctx_t *ctx);          /* CORSIT        */
agc_status_t agc_job_corsit_coarse_done(agc_job_ctx_t *ctx);
agc_status_t agc_job_corsit_fine_done(agc_job_ctx_t *ctx);
agc_status_t agc_job_gyrotrim(agc_job_ctx_t *ctx);        /* GYROTRIM      */
agc_status_t agc_job_justtrim(agc_job_ctx_t *ctx);        /* JUSTTRIM      */
agc_status_t agc_job_justtrim_fine_done(agc_job_ctx_t *ctx);
agc_status_t agc_job_justtrim_pulsed(agc_job_ctx_t *ctx);
agc_status_t agc_job_2stars(agc_job_ctx_t *ctx);          /* 2STARS        */
agc_status_t agc_job_1star(agc_job_ctx_t *ctx);           /* 1STAR         */

#endif /* AGC_ALIGN_H */
