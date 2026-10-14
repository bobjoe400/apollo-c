/**********************************************************************
 * FILE NAME: alignment.h
 *
 * PURPOSE:
 *   IMU alignment programs (P51-P54, P57), optics marking (SXTMARK /
 *   AOTMARK), CSM and LM geometry, S-band antenna (R05), R63 and AGS
 *   initialisation (R47): the alignment erasable block, the routines
 *   the navigation, radar, display and noun code call, and the job entry
 *   points (agc_job_*: the AGC labels at which a job starts or resumes
 *   after a display, a stall or a delay; registered in tools/registry.tsv).
 *   Owner: prog-align (docs/modules/prog-align.md).
 *
 *   Scale conventions used throughout this header:
 *     half-unit   DP vector of magnitude 0.5 (the AGC UNIT result, 2^1)
 *     revs        DP fraction of one revolution, 2^0 (0.25 = 90 degrees)
 *     cdu         raw 15-bit 2's-complement CDU word, scaled in half
 *                 revolutions: 040000 octal = 180 degrees, one count =
 *                 360/32768 degrees (CDUX, THETAD, SAC, PAC, ...)
 *     cos/4       a cosine stored as value/4 (dot product of two
 *                 half-unit vectors, scale 2^2)
 *     cs 2^28     centiseconds, DP
 *
 * AGC SOURCE:
 *   Comanche055 P51-P53.agc pp. 737-784, SXTMARK.agc pp. 222-235,
 *   CSM_GEOMETRY.agc pp. 285-296, S-BAND_ANTENNA_FOR_CM.agc pp. 934-935,
 *   IMU_CALIBRATION_AND_ALIGNMENT.agc pp. 423-454 (prelaunch)
 *   Luminary099 P51-P53.agc pp. 926-983, AOTMARK.agc pp. 244-261,
 *   LEM_GEOMETRY.agc pp. 320-325, S-BAND_ANTENNA_FOR_LM.agc pp. 486-489,
 *   AGS_INITIALIZATION.agc pp. 206-210, R60_62.agc pp. 338-341 (R63)
 *
 * REQUIRES:
 *   none (self-contained; includes what it uses)
 *
 * GLOBAL VARIABLES:
 *   Name          Type             Scale     Description
 *   ------------  ---------------  --------  -------------------------
 *   none (the alignment erasables are one module block reached through
 *   agc_align_ram(); its members carry their scale in a short comment)
 **********************************************************************/
#ifndef AGC_ALIGN_H
#define AGC_ALIGN_H

#include <stdbool.h>
#include <stdint.h>

#include <stddef.h>
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/nav/integration.h"

/* ---- sizes ---------------------------------------------------------------- */
#define AGC_ALIGN_NSTARS 37        /* catalogue stars 1..37 (STAR_TABLES)     */
#define AGC_ALIGN_STAR_CODE_MAX \
    050                            /* star codes 0..50 octal (CHKSCODE)       */
#define AGC_ALIGN_SXT_MARKS     6  /* SXTMARK marks that fit in a VAC (7 wd)  */
#define AGC_ALIGN_AOT_PAIRS     5  /* AOTMARK mark pairs (MARKCNTR 0..4)      */
#define AGC_ALIGN_AGSBUFF_WORDS 14 /* AGSBUFF (14D)                           */

/* ---- LOCSAM outputs (S50, P51-P53) -------------------------------------- */
typedef struct agc_align_locsam
{
    agc_vec_t vearth; /* VEARTH: LOS to the earth centre (ref), half-unit (2^1) */
    agc_vec_t vsun;   /* VSUN: LOS to the sun (ref), half-unit (2^1) */
    agc_vec_t vmoon;  /* VMOON: LOS to the moon centre (ref), half-unit (2^1) */
    agc_dp_t  cearth; /* CEARTH: occultation cone cosine / 4, scale 2^2 */
    agc_dp_t  csun;   /* CSUN: occultation cone cosine / 4, scale 2^2 */
    agc_dp_t  cmoon;  /* CMOON: occultation cone cosine / 4, scale 2^2 */
    agc_vec_t vel_c;  /* VEL/C aberration vector, B-1 (CSM only)    */
} agc_align_locsam_t;

/* One SXTMARK mark (7 words in the mark VAC: MKT2T1, CDUY, CDUS, CDUZ,
 * CDUT, CDUX), CSM.                                                        */
typedef struct agc_align_sxt_mark
{
    agc_dp_t t;    /* MKT2T1 time of mark, cs B28                */
    uint16_t cduy; /* CDUY at the mark, CDU word */
    uint16_t cdus; /* CDUS (shaft) */
    uint16_t cduz; /* CDUZ */
    uint16_t cdut; /* CDUT (trunnion) */
    uint16_t cdux; /* CDUX */
} agc_align_sxt_mark_t;

/* One AOTMARK mark pair (6 words: X-mark CDUY,Z,X at +0,+2,+4 and Y-mark
 * CDUY,Z,X at +1,+3,+5; a surface mark keeps CURSOR at +1, SPIRAL at +3).  */
typedef struct agc_align_aot_pair
{
    uint16_t xcdu[3]; /* X mark: CDUY, CDUZ, CDUX (cdu)             */
    uint16_t ycdu[3]; /* Y mark (flight) or CURSOR, SPIRAL, -       */
} agc_align_aot_pair_t;

/*
 * The alignment erasables (ERASABLE_ASSIGNMENTS "ALIGNMENT STORAGE",
 * "P50'S, R50'S Q STORES", the overlays on STARAD, and the mark VAC area).
 * DEVIATION: one module-owned block reached through agc_align_ram() instead
 * of one accessor per erasable; these words are shared only by the
 * alignment/marking code and the noun tables (N06 N05 N22 N70 N71 N79 N87
 * N88 N89 N93 N94).
 */
typedef struct agc_align_ram
{
    /* orientation matrices, rows are half-unit vectors */
    agc_mat_t xsmd;    /* XSMD,YSMD,ZSMD desired SM (ref)            */
    agc_mat_t xsm;     /* XSM,YSM,ZSM   desired SM (S52.2 output)    */
    agc_mat_t xnb;     /* XNB,YNB,ZNB   nav base wrt SM or ref       */
    agc_mat_t xdc;     /* XDC,YDC,ZDC   desired SM wrt present SM    */
    agc_dp_t  ogct[3]; /* OGCT: saved torque angles (P57), revs 2^0 */

    /* star data */
    agc_vec_t          starad[3]; /* STARAD, +6, +12D, half-unit (2^1) */
    agc_vec_t          starsav1;  /* STARSAV1: 1st observed LOS, half-unit (2^1) */
    agc_vec_t          starsav2;  /* STARSAV2: 2nd observed LOS, half-unit (2^1) */
    agc_vec_t          starsav3;  /* STARSAV3: planet vector (N88), half-unit (2^1) */
    agc_vec_t          planvec;   /* PLANVEC: 1st catalogue LOS, half-unit (2^1) */
    agc_vec_t          vac6;      /* VAC 6D (R54 second pair, star 1), half-unit (2^1) */
    agc_vec_t          vac12;     /* VAC 12D (R54 second pair, star 2), half-unit (2^1) */
    agc_vec_t          star;      /* STAR: LOS (CALCSXA input / temp), half-unit (2^1) */
    agc_vec_t          sax;       /* SAX: optics shaft axis (ref), half-unit (2^1) */
    agc_vec_t          vec1;      /* VEC1: P57 reference vector 1, half-unit (2^1) */
    agc_vec_t          vec2;      /* VEC2: P57 reference vector 2, half-unit (2^1) */
    agc_vec_t          gsav;      /* GSAV: gravity, NB, half-unit (2^1) (P57) */
    agc_vec_t          gout;      /* GOUT: gravity sample, NB, half-unit (2^1) (P57) */
    agc_vec_t          gacc;      /* GACC: accumulated DELV, PIPA counts 2^14 (P57) */
    agc_vec_t          ynbsav;    /* YNBSAV: Y attitude, moon fixed, half-unit (2^1) (P57) */
    agc_vec_t          znbsav;    /* ZNBSAV: Z attitude, moon fixed, half-unit (2^1) (P57) */
    agc_vec_t          scaxis;    /* SCAXIS: AOT optic axis (NB), half-unit (2^1) */
    agc_vec_t          pointvsm;  /* POINTVSM: LOS for VECPOINT (SM), half-unit (2^1) */
    agc_vec_t          xpnb;      /* VAC 18D: X-mark plane vector (NB), half-unit (2^1) (LM) */
    agc_vec_t          ypnb;      /* VAC 12D: Y-mark plane vector (NB), half-unit (2^1) (LM) */
    agc_vec_t          uxp;       /* VAC 24D: UXP (OANB), half-unit (2^1) (LM) */
    agc_vec_t          uyp;       /* VAC 18D: UYP (OANB), half-unit (2^1) (LM) */
    agc_align_locsam_t locsam;    /* VEARTH VSUN VMOON (STARAD overlay), CSS    */
    agc_sp_t           besti;     /* BESTI: first star number x 6 (unscaled) */
    agc_sp_t           bestj;     /* BESTJ: second star number x 6 (unscaled) */
    agc_sp_t           starind;   /* STARIND: 0 = 1st star, 1 = 2nd (unscaled) */
    agc_sp_t           markindx;  /* MARKINDX: number of marks wanted (CM), unscaled */
    agc_sp_t           starcode;  /* STARCODE (N70) / AOTCODE (N71): bits 1-6 star, 7-9 AOT detent (LM), unscaled */
    agc_dp_t           tsight;    /* TSIGHT  time of sighting, cs B28           */
    agc_dp_t           talign;    /* TALIGN  alignment time, cs B28 (LM)        */
    agc_dp_t           theta;     /* THETA: angle of the marked pair, revs 2^0 */
    agc_dp_t           theta18;   /* VAC 18D: angle of the catalogue pair, revs 2^0 */
    agc_dp_t           normtem1;  /* NORMTEM1: star angle difference (N05), revs 2^0 */
    agc_dp_t           aoptime;   /* AOPTIME  cs B28 (SR52.1) / angle (ROTA)    */
    agc_dp_t           landlat;   /* LANDLAT (N89): latitude, revs 2^0 */
    agc_dp_t           landlong;  /* LANDLONG (N89): longitude / 2, revs 2^0 */
    agc_dp_t           landalt;   /* LANDALT (N89): altitude, m 2^29 */

    /* optics (CM) */
    uint16_t sac; /* SAC: desired shaft, CDU word */
    uint16_t pac; /* PAC: desired trunnion, CDU word */

    /* AOT (LM) */
    uint16_t az;       /* AZ: keyed-in optic axis azimuth (N87), CDU word */
    uint16_t el;       /* EL: keyed-in optic axis elevation (N87), CDU word */
    uint16_t cursor;   /* CURSOR (N79), CDU word */
    uint16_t spiral;   /* SPIRAL (N79), CDU word */
    uint16_t rotang;   /* VAC 10D AOT apparent rotation, cdu         */
    agc_sp_t poscode;  /* POSCODE: AOT detent position 1..6 (unscaled) */
    agc_sp_t xymark;   /* XYMARK: mark kind / detent (unscaled) */
    agc_sp_t markcntr; /* MARKCNTR: mark pair counter 0..4 (unscaled) */
    agc_sp_t mkdex;    /* MKDEX: mark index (unscaled) */
    agc_sp_t gctr;     /* GCTR: gravity sample counter, -20 .. -0 (unscaled) */

    /* mark storage (the mark VAC area) */
    agc_align_sxt_mark_t sxt[AGC_ALIGN_SXT_MARKS]; /* SXTMARK marks             */
    agc_sp_t             qpret;                    /* QPRET: next mark slot / marks made (CM), unscaled */
    agc_align_aot_pair_t aot[AGC_ALIGN_AOT_PAIRS]; /* AOTMARK mark pairs        */
    int                  mkvac_index;              /* VAC area held by the mark system, -1 none  */

    /* return points (QMIN, QMAJ, SAVQR52, R53EXIT, QPRET held a return
     * address in the AGC; here the continuation job)                        */
    agc_job_id_t qmin;                 /* QMIN                                       */
    agc_job_id_t qmaj;                 /* QMAJ                                       */
    agc_job_id_t auto_optics_return;   /* SAVQR52                                    */
    agc_job_id_t sighting_mark_return; /* R53EXIT                                    */
    agc_job_id_t planet_ret;           /* return of PLANET when it flashes V06N88    */
    agc_job_id_t coarse_ret;           /* return of COARSE (QPRET of CALL COARSE)    */
    agc_vec_t    planet_los;           /* PLANET result (MPAC): reference axes, half-unit (2^1) */
    agc_dp_t     landing_site_time;    /* GDT/2 +4: N89DISP time, cs B28 (LM)        */
    agc_vec_t    alphav;               /* ALPHAV  landing site / LALOTORV result, m B29 */
    agc_job_id_t lunar_gravity_return; /* QMIN of LUNG / GYROTRIM (P57)        */
} agc_align_ram_t;

agc_align_ram_t *agc_align_ram(void);
void             agc_align_ram_reset(void);

/* ---- shared alignment math (src/programs/) ------------------------------- */

/* S52.3 axes from a state (r m, v m/cs, any common scale).                */
void agc_align_nominal_orientation_axes(agc_interp_t *ip, agc_vec_t r, agc_vec_t v, agc_mat_t *smd);

/* S52.3 with the CSMPREC (CM) / LEMCONIC (LM) call; BUSY = job asleep in
 * INTSTALL and resumed as resume_job.                                     */
void agc_align_lsorient(agc_interp_t *ip, agc_vec_t xsmd, agc_vec_t rcsm, agc_vec_t vcsm, agc_mat_t *smd);
void agc_align_gyro_coarse_dc(agc_interp_t *ip, const agc_mat_t *smd, const agc_mat_t *refsmmat, agc_mat_t *xdc);

/* LOCSAM from an integrated state and LSPOS output (pure).               */
agc_status_t agc_align_locsam(agc_interp_t *ip, agc_dp_t time_cs, agc_job_id_t resume_job, agc_align_locsam_t *out);
agc_status_t agc_align_picapar(agc_interp_t *ip, const agc_align_locsam_t *ls, const agc_mat_t *refsmmat, const uint16_t cdu[3], agc_sp_t *besti, agc_sp_t *bestj, bool *found);

/* PLANET for a star6 that needs no display (catalogue, sun, earth, moon,
 * or a planet whose vector is already in STARSAV3 / STARAD).              */
agc_dp_t     agc_align_chksdata_angle(agc_interp_t *ip, const agc_vec_t cat[2], const agc_vec_t obs[2]);
agc_status_t agc_align_fine_solve(agc_interp_t *ip, const agc_vec_t a[2], const agc_vec_t b[2], agc_mat_t *xdc);

/* CHKSCODE: true when 0 <= STARCODE <= 50 octal.                          */
bool agc_align_chkscode(agc_sp_t starcode);

/* ---- COARSE (both vehicles, src/programs/alignment/common.c) ---------------- */

/* ---- shared alignment jobs: R54 (CHKSDATA) and R55 (src/programs/) ------ */
agc_status_t agc_align_sighting_data_display(const agc_job_ctx_t *ctx, agc_job_id_t qmin_return);
agc_status_t agc_align_gyro_torquing(const agc_job_ctx_t *ctx, agc_job_id_t qmin_return);

/* ---- CSM geometry (src/programs/alignment/geometry_csm.c) ------------------- */
agc_dp_t     agc_csm_sxtlogic(uint16_t trunnion_cdu);
void         agc_csm_sxtnb(agc_interp_t *ip, uint16_t trunnion_cdu, uint16_t shaft_cdu, agc_vec_t *starm_nb);
void         agc_csm_calcsxa(agc_interp_t *ip, agc_vec_t star_sm, const agc_cdutrig_t *t, uint16_t *sac, uint16_t *pac, bool *cult);
void         agc_csm_sxtang(agc_interp_t *ip, agc_vec_t star, const agc_mat_t *nb, uint16_t *sac, uint16_t *pac, bool *cult);
agc_status_t agc_csm_log(agc_interp_t *ip, agc_dp_t x, agc_dp_t *neg_log);

typedef struct agc_csm_earrot
{
    agc_vec_t rtinit;  /* RTINIT: half-unit (2^1) */
    agc_vec_t unitw;   /* UNITW: earth rotation axis, full unit (2^0) */
    agc_vec_t rteast;  /* RTEAST: half-unit (2^1) */
    agc_vec_t rtnorm;  /* RTNORM: half-unit (2^1) */
    agc_dp_t  dtearot; /* DTEAROT cs B28                             */
    agc_vec_t rt;      /* RT: target vector, half-unit (2^1) */
} agc_csm_earrot_t;

void agc_csm_earrot1(agc_interp_t *ip, agc_csm_earrot_t *e);
void agc_csm_earrot2(agc_interp_t *ip, agc_csm_earrot_t *e);

/* ---- LM geometry (src/lm/optics/lem_geometry.c) --------------------------------- */
/* RRANGLES: los_nb half-unit; modea/modeb = (trunnion, shaft) cdu for RR
 * mode 1 and mode 2 (swapped when RADMODES antenna bit says mode 2);
 * *along_y when the LOS is along Y (the AGC's BOVB LUNDESCH).             */
void         agc_lm_rrangles(agc_interp_t *ip, agc_vec_t los_nb, bool rr_mode2, uint16_t modea[2], uint16_t modeb[2], bool *along_y);
void         agc_lm_rrnb(agc_interp_t *ip, uint16_t trunnion_cdu, uint16_t shaft_cdu, agc_vec_t *los_nb);
agc_status_t agc_job_rr_los_display(void);      /* RRLOSDSP (V85): RR-AZ, RR-ELEV once a second */
agc_status_t agc_job_rr_los_display_next(void); /* RRLOSDSP after DELAYJOB                 */

/* ---- S-band antenna R05 --------------------------------------------------- */
/* CM: r_nb = vehicle-to-earth vector in NB axes (any scale) -> RHOSB pitch,
 * GAMMASB yaw (revs).  LM: idem -> PITCHANG, YAWANG (revs).               */
void     agc_sband_cm_angles(agc_interp_t *ip, agc_vec_t r_nb, agc_dp_t *pitch, agc_dp_t *yaw);
void     agc_sband_lm_angles(agc_interp_t *ip, agc_vec_t r_nb, agc_dp_t *pitch, agc_dp_t *yaw);
agc_dp_t agc_sband_rhosb(void);    /* RHOSB  (CM N51 pitch), revs    */
agc_dp_t agc_sband_gammasb(void);  /* GAMMASB (CM N51 yaw), revs     */
agc_dp_t agc_sband_pitchang(void); /* PITCHANG (LM N51), revs        */
agc_dp_t agc_sband_yawang(void);   /* YAWANG   (LM N51), revs        */

/* ---- R63 (LM) ------------------------------------------------------------ */
void agc_rendezvous_final_attitude_los(agc_interp_t *ip, agc_vec_t rcsm, agc_vec_t rlem, const agc_mat_t *refsmmat, agc_vec_t *pointvsm);

/* ---- R47 AGS initialisation (LM) ----------------------------------------- */
/* SCALEVEC: RATT1/VATT1 (moon sphere m B27, m/cs B5) -> SM axes (half-
 * unit REFSMMAT), AEA words of 512 ft and 0.5 ft/s, rounded for the AEA's
 * 2's complement; out = RX, VX, RY, VY, RZ, VZ (high words).              */
void            agc_ags_scalevec(agc_interp_t *ip, agc_vec_t ratt1, agc_vec_t vatt1, const agc_mat_t *refsmmat, uint16_t out[6]);
agc_dp_t        agc_ags_time(agc_interp_t *ip, agc_dp_t tat, agc_dp_t agsk);
const uint16_t *agc_ags_buffer(void); /* AGSBUFF, 14 raw words           */

/* ---- SXTMARK (CSM, src/csm/optics/sxtmark.c) ------------------------------------ */
agc_status_t agc_sxtmark(agc_sp_t nmarks); /* SXTMARK       */
agc_status_t agc_sxtmark_mkreleas(void);   /* MKRELEAS      */
agc_status_t agc_sxtmark_free_vac(void);   /* MARKSTAT = 0  */
agc_status_t agc_sxtmark_markrupt(void);   /* MARKRUPT      */
agc_status_t agc_sxtmark_mark2(void);      /* MARK2/VACSTOR */
agc_sp_t     agc_sxtmark_marks_made(void); /* QPRET count   */
void         agc_sxtmark_mark(int mark, agc_align_sxt_mark_t *out);

/* ---- AOTMARK (LM, src/lm/optics/aotmark.c) -------------------------------------- */
agc_status_t agc_aotmark(void);          /* AOTMARK       */
agc_status_t agc_aotmark_markrupt(void); /* MARKRUPT      */
void         agc_aot_oanb(agc_interp_t *ip, uint16_t az, uint16_t el, agc_vec_t *oa, agc_vec_t *uxp, agc_vec_t *uyp);
void         agc_aot_optaxis(agc_interp_t *ip, agc_vec_t uxp, agc_vec_t uyp, uint16_t rot, agc_vec_t *xpnb, agc_vec_t *ypnb);
void         agc_aot_surfstar(agc_interp_t *ip, const agc_align_aot_pair_t *m, agc_vec_t oa, agc_vec_t xpnb, agc_vec_t ypnb, agc_vec_words_t *star_sm);
void         agc_aot_aveit(agc_interp_t *ip, agc_sp_t n, agc_vec_words_t star, agc_vec_t *ave);

/* ---- CSM sighting and optics routines (src/csm/optics/) ---------------- */
/* SXTSM line of sight of one mark (half-unit, SM axes).                   */
void agc_csm_sxtsm(agc_interp_t *ip, const agc_align_sxt_mark_t *m, agc_vec_t *los_sm); /* SXTSM         */

/* SXTSM as called: TSIGHT = first mark time, MARKDOWN / MARK2DWN, LOS.     */
void agc_csm_sxtsm_marks(agc_interp_t *ip, agc_vec_t *los_sm);

/* Interpretive subroutines entered from a job: ret is the job continued
 * as on return (SAVQR52 / R53EXIT).                                       */
agc_status_t agc_csm_auto_optics_positioning(const agc_job_ctx_t *ctx, agc_job_id_t ret);     /* R52    */
agc_status_t agc_job_optics_coarse_drive(agc_job_ctx_t *ctx);                                 /* V41 N91 OPTC1 +4 */
agc_status_t agc_csm_auto_optics_advanced_orbit(const agc_job_ctx_t *ctx, agc_job_id_t ret);  /* ADVORB */
agc_status_t agc_csm_sighting_mark(agc_job_id_t ret);                                         /* R53    */
agc_status_t agc_csm_alternate_los_sighting_mark(const agc_job_ctx_t *ctx, agc_job_id_t ret); /* R56    */

/* ---- LM P51/P52/P57, R51, R52, R59 (src/lm/) ------------------------------ */
/* IMUCHK: OK with the IMU on (IMUSE set); AGC_STATUS_ALARM after alarm 210
 * and GOTOPOOH.                                                           */

/* MFREF / REFMF (P57): VEC1, VEC2 moon fixed -> reference; YNBSAV, ZNBSAV
 * (moon fixed) from the present attitude, SET ATTFLAG.                    */
agc_status_t agc_align_mfref(agc_interp_t *ip);
agc_status_t agc_align_refmf(agc_interp_t *ip);

/* P57 pieces entered from more than one file (src/lm/alignment/).        */

/* ==== job and task entry points (BEGIN generated list) ==== */
/* src/csm/alignment/fine_align.c */
agc_status_t agc_job_csm_fine_align(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_fine_align_star_code(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_fine_align_star_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_fine_align_star_accepted(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_fine_align_star_located(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_fine_align_marked(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_fine_align_star_at_mark(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_fine_align_data_checked(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_fine_align_torqued(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_fine_align_recheck(agc_job_ctx_t *ctx);

/* src/csm/alignment/imu_orientation_determination.c */
agc_status_t agc_job_csm_imu_orientation_determination(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_orientation_acquire_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_orientation_coarse(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_orientation_coarse_verb(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_orientation_coarse_start(void);
agc_status_t agc_job_csm_orientation_coarse_done(void);
agc_status_t agc_job_csm_orientation_fine_done(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_orientation_mark_setup(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_orientation_mark_star(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_orientation_marked(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_orientation_first_star(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_orientation_second_star(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_orientation_data_checked(agc_job_ctx_t *ctx);

/* src/csm/alignment/imu_realign.c */
agc_status_t agc_job_csm_realign_star_selection(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_realign_pick_stars(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_realign_stars_located(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_imu_realign_landing_site(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_landing_site_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_csm_landing_site_accepted(void);
agc_status_t agc_job_csm_landing_site_axes(void);

/* src/csm/optics/auto_optics_positioning.c */
agc_status_t agc_job_auto_optics_positioning_resume(agc_job_ctx_t *ctx);
agc_status_t agc_job_auto_optics_positioning_wait(void);
agc_status_t agc_job_auto_optics_positioning_delayed(agc_job_ctx_t *ctx);
agc_status_t agc_job_auto_optics_positioning_tracked(agc_job_ctx_t *ctx);
agc_status_t agc_job_auto_optics_sighting(void);
agc_status_t agc_job_auto_optics_positioning_terminate(agc_job_ctx_t *ctx);

/* src/csm/optics/s_band_antenna.c */
agc_status_t agc_job_sbandant_cm_recycle(agc_job_ctx_t *ctx);
agc_status_t agc_job_sbandant_cm(agc_job_ctx_t *ctx);

/* src/csm/optics/sighting_mark.c */
agc_status_t agc_job_sighting_mark_marked(agc_job_ctx_t *ctx);
agc_status_t agc_job_sighting_mark_code_clean(agc_job_ctx_t *ctx);
agc_status_t agc_job_sighting_mark_code_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_sighting_mark_code_entered(agc_job_ctx_t *ctx);
agc_status_t agc_job_alternate_los_angles_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_alternate_los_reserve(agc_job_ctx_t *ctx);
agc_status_t agc_job_alternate_los_mark_request(agc_job_ctx_t *ctx);
agc_status_t agc_job_alternate_los_marked(agc_job_ctx_t *ctx);
agc_status_t agc_job_alternate_los_remark(agc_job_ctx_t *ctx);

/* src/csm/optics/sxtmark.c */
agc_status_t agc_task_markdif(void);
agc_status_t agc_job_mark_request_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_mark_request_flash(agc_job_ctx_t *ctx);
agc_status_t agc_job_mark_complete_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_mark_redisplay(agc_job_ctx_t *ctx);
agc_status_t agc_job_termsxt(agc_job_ctx_t *ctx);
agc_status_t agc_job_entanswr(void);
agc_status_t agc_task_endmarks(void);

/* src/lm/ags_initialization.c */
agc_status_t agc_job_ags_initialization(agc_job_ctx_t *ctx);
agc_status_t agc_job_ags_initialization_new_zero(agc_job_ctx_t *ctx);
agc_status_t agc_job_ags_initialization_compute(agc_job_ctx_t *ctx);
agc_status_t agc_job_ags_initialization_lm_state(void);
agc_status_t agc_job_ags_initialization_csm_state(void);
agc_status_t agc_job_ags_initialization_zero_imu(agc_job_ctx_t *ctx);
agc_status_t agc_job_ags_initialization_sent(agc_job_ctx_t *ctx);
agc_status_t agc_job_ags_initialization_imu_zeroed(agc_job_ctx_t *ctx);
agc_status_t agc_job_ags_initialization_end(void);

/* src/lm/alignment/auto_optics_positioning.c */
agc_status_t agc_job_lm_auto_optics_star_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_auto_optics_orient(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_auto_optics_az_el_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_auto_optics_az_el_loaded(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_auto_optics_star_located(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_auto_optics_maneuvered(agc_job_ctx_t *ctx);

/* src/lm/alignment/fine_align.c */
agc_status_t agc_job_lm_fine_align(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_fine_align_perform(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_fine_align_star_acquire(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_fine_align_pick_stars(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_fine_align_stars_located(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_fine_align_first_star(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_fine_align_mark(void);
agc_status_t agc_job_lm_fine_align_marked(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_fine_align_first_located(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_fine_align_second_located(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_fine_align_data_checked(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_fine_align_torqued(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_fine_align_recheck(agc_job_ctx_t *ctx);

/* src/lm/alignment/imu_orientation_determination.c */
agc_status_t agc_job_lm_imu_orientation_determination(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_orientation_coarse(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_orientation_coarse_verb(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_orientation_coarse_start(void);
agc_status_t agc_job_lm_orientation_coarse_done(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_orientation_mark_setup(void);
agc_status_t agc_job_lm_orientation_marked(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_orientation_first_star(void);
agc_status_t agc_job_lm_orientation_second_star(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_orientation_data_checked(agc_job_ctx_t *ctx);

/* src/lm/alignment/imu_realign.c */
agc_status_t agc_job_lm_landing_site_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_landing_site_accepted(void);
agc_status_t agc_job_lm_imu_realign_landing_site(agc_job_ctx_t *ctx);
agc_status_t agc_job_lm_realign_site_orientation(agc_job_ctx_t *ctx);

/* src/lm/alignment/lunar_surface_align.c */
agc_status_t agc_job_lunar_surface_align(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_option_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_option(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_time_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_time_entered(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_site_orientation(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_options_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_options_chosen(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_realign(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_data_checked(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_torque_test(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_aligned(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_post(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_post_enter(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_site_updated(void);

/* src/lm/alignment/lunar_surface_gravity.c */
agc_status_t agc_job_lunar_gravity_coarsed(void);
agc_status_t agc_task_lunar_gravity_sample(void);
agc_status_t agc_job_lunar_gravity_accumulate(void);
agc_status_t agc_job_lunar_gravity_rotate(void);
agc_status_t agc_job_lunar_gravity_measured(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_gravity_recycle(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_gravity_accepted(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_trim_pulse(void);
agc_status_t agc_job_lunar_surface_trim_retry(void);
agc_status_t agc_job_lunar_surface_trimmed(void);
agc_status_t agc_job_lunar_surface_coarse_trim_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_coarse_trim_go(void);
agc_status_t agc_job_lunar_surface_coarse_trimmed(void);

/* src/lm/alignment/lunar_surface_sighting.c */
bool         agc_lm_aot_find_detent(agc_interp_t *ip, agc_vec_t star); /* INCAZ..R59D */
agc_status_t agc_job_lunar_surface_star_acquisition(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_star_chosen(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_star_positioned(void);
agc_status_t agc_job_lunar_surface_star_mark(void);
agc_status_t agc_job_lunar_surface_star_marked(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_first_star(agc_job_ctx_t *ctx);
agc_status_t agc_job_lunar_surface_second_star(agc_job_ctx_t *ctx);

/* src/lm/optics/aotmark.c */
agc_status_t agc_task_aot_mark_release(void);
agc_status_t agc_job_aot_mark_kill(void);
agc_status_t agc_job_aot_mark_star_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_aot_mark_detent(agc_job_ctx_t *ctx);
agc_status_t agc_job_aot_mark_coas_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_aot_mark_coas_loaded(agc_job_ctx_t *ctx);
agc_status_t agc_job_aot_mark_check(agc_job_ctx_t *ctx);
agc_status_t agc_job_aot_mark_next_action(agc_job_ctx_t *ctx);
agc_status_t agc_job_aot_mark_surface_end(agc_job_ctx_t *ctx);
agc_status_t agc_job_aot_mark_surface_answer(agc_job_ctx_t *ctx);

/* src/lm/optics/s_band_antenna.c */
agc_status_t agc_job_sbandant_lm_recycle(agc_job_ctx_t *ctx);
agc_status_t agc_job_sbandant_lm(agc_job_ctx_t *ctx);

/* src/lm/rendezvous_final_attitude.c */
agc_status_t agc_job_rendezvous_final_attitude(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_final_attitude_option(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_final_attitude_compute(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_final_attitude_csm_state(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_final_attitude_lm_state(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_final_attitude_maneuver(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_final_attitude_end(void);

/* src/programs/alignment/common.c */
agc_status_t agc_job_align_coarse_done(void);
agc_status_t agc_job_align_fine_done(void);

/* src/programs/alignment/imu_realign.c */
agc_status_t agc_job_imu_realign(agc_job_ctx_t *ctx);
agc_status_t agc_job_imu_realign_option_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_imu_realign_option(agc_job_ctx_t *ctx);
agc_status_t agc_job_realign_time(agc_job_ctx_t *ctx);
agc_status_t agc_job_realign_time_flash(agc_job_ctx_t *ctx);
agc_status_t agc_job_realign_time_proceed(agc_job_ctx_t *ctx);
agc_status_t agc_job_realign_nominal_orientation(agc_job_ctx_t *ctx);
agc_status_t agc_job_realign_gimbal_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_coarstyp(agc_job_ctx_t *ctx);
agc_status_t agc_job_coarse_align(agc_job_ctx_t *ctx);
agc_status_t agc_job_coarse_align_coarsed(agc_job_ctx_t *ctx);
agc_status_t agc_job_coarse_align_fineonly(agc_job_ctx_t *ctx);
agc_status_t agc_job_gycrs(agc_job_ctx_t *ctx);
agc_status_t agc_job_gycrs_pulse(agc_job_ctx_t *ctx);
agc_status_t agc_job_gycrs_pulsed(agc_job_ctx_t *ctx);
agc_status_t agc_job_gycrs_torqued(agc_job_ctx_t *ctx);

/* src/programs/alignment/planet.c */
agc_status_t agc_job_planet_flash(agc_job_ctx_t *ctx);
agc_status_t agc_job_planet_loaded(void);
agc_status_t agc_job_planet_resume(void);

/* src/programs/alignment/sighting_data_and_gyro_torquing.c */
agc_status_t agc_job_chksdata_flash(agc_job_ctx_t *ctx);
agc_status_t agc_job_chksdata_proceed(void);
agc_status_t agc_job_chksdata_enter(void);
agc_status_t agc_job_gyro_torquing_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_gyro_torquing_pulse(void);
agc_status_t agc_job_gyro_torquing_pulsed(void);
agc_status_t agc_job_gyro_torquing_return(void);

/* ==== (END generated list) ==== */

/* ======================================================================
 * Helpers shared by the alignment and marking files (src/programs/alignment, src/csm/{alignment,optics}, src/lm/{alignment,optics}); formerly alignment_internal.h
 * ==================================================================== */
/* Integration entrances used by the alignment code (INTEGRATION_INIT.).   */
typedef enum align_integ_kind
{
    ALIGN_CSMPREC = 0,
    ALIGN_LEMPREC,
    ALIGN_CSMCONIC,
    ALIGN_LEMCONIC
} align_integ_kind_t;

/* CSMPREC/LEMPREC/CSMCONIC/LEMCONIC to tdec1 (cs B28) in the shared
 * integration context; *out = RATT.. X2.  BUSY: the job sleeps in INTSTALL
 * and is resumed as resume_job, which must call this again.              */
agc_status_t align_integ(agc_interp_t *ip, align_integ_kind_t kind, agc_dp_t tdec1, agc_job_id_t resume_job, agc_integ_out_t *out);

/* A flashing display with the three astronaut returns (call CADR +1, +2,
 * +3).  For GOPERF1 / GOPERF4 kinds vn is the checklist / option code.   */
agc_status_t align_flash(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind, uint16_t vn, agc_job_id_t term, agc_job_id_t proceed, agc_job_id_t enter);

/* A non-flashing display (GODSP, GODSPR, GODSPRET, CLEANDSP, KLEENEX).    */
agc_status_t align_show(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind, uint16_t vn);

/* The answer to the last flashing display was V32 (recycle).             */
bool align_resp_recycle(void);

/* GOTO / TC to another resumption point of the same job (CHANG1-like:
 * the job continues as `next`); the caller returns its status.          */
agc_status_t align_goto(agc_job_id_t next);

/* IMUSTALL / OPTSTALL (AOTSTALL).  *done = true: the event already came,
 * go on now.  *done = false: the job sleeps and continues as wake_as.
 * A bad end does CURTAINS and returns AGC_STATUS_ALARM.                  */
agc_status_t align_stall(agc_imu_device_t dev, agc_job_id_t wake_as, bool *done);

/* In the job woken by a stall: AGC_STATUS_OK for GOODEND, otherwise
 * CURTAINS and AGC_STATUS_ALARM.                                         */
agc_status_t align_stall_outcome(agc_imu_device_t dev);

/* PHASCHNG with a type C code and a job restart point at priority prio. */
void align_phase_job(uint16_t code, agc_job_id_t job, agc_prio_t prio);

/* PLANET (planet.c): the line of sight of STARCODE at t, left in
 * agc_align_ram()->planet_los; the job continues as ret.                 */
agc_status_t align_planet(const agc_job_ctx_t *ctx, agc_dp_t t, agc_job_id_t ret);

/* BOV +1: test (and so clear) OVFIND, going on either way.               */
void align_clear_ovf(agc_interp_t *ip);

/* PHASCHNG with a fixed (type A) group 4 code.                           */
void align_phase4(uint16_t code);

/* NCOARSE (Luminary p. 951): SET1/PDT plus PIPAX/Y/Z = -0.               */
agc_status_t align_ncoarse(void);

/* COARSE (Luminary p. 951; the same sequence is inline in the Comanche
 * callers): IMUCOARS to THETAD, IMUSTALL, IMUFINE, IMUSTALL, then the job
 * continues as ret.  A bad end does CURTAINS.  The caller returns the
 * status.                                                                */
agc_status_t align_coarse(agc_job_id_t ret);

/* LOADTIME: TIME2, TIME1 as a DP, cs B28.                                */

/* DSPTEM1, DSPTEM1 +1 as a DP (cs B28), and the store.                    */
agc_dp_t align_dsptem1(void);
void     align_set_dsptem1(agc_dp_t v);

/* Raw 15-bit 1's complement word <-> signed SP.                           */
agc_sp_t align_word_to_sp(uint16_t w);
uint16_t align_sp_to_word(agc_sp_t v);

/* ---- MPAC word forms (src/programs/alignment/word_forms.c) ----------- */

/* ARCTRIG (INFLIGHT_ALIGNMENT_ROUTINES p. 1349) run on the MPAC words:
 * the angle (revs) in the words the AGC leaves (ASIN/ACOS/DSU results
 * whose signs may disagree).                                             */
agc_dp_words_t align_arctrig_words(agc_interp_t *ip, agc_dp_t sinth, agc_dp_t costh);

/* DOT / DMP followed by SLn: the shift acts on the TP MPAC (MPAC +2's top
 * bits reach the stored DP); result truncated to DP.                    */
agc_dp_t align_dot_shl(agc_interp_t *ip, agc_vec_t u, agc_vec_t v, int n);
agc_dp_t align_dmp_shl(agc_interp_t *ip, agc_dp_t a, agc_dp_t b, int n);

/* The present IMU CDUs X, Y, Z (raw words).                               */
void align_read_cdus(uint16_t cdu[3]);

/* CDUTRIG + CALCSMSC of the present CDUs: rows XNB.. wrt the present SM.  */
agc_status_t align_nb_present(agc_interp_t *ip, agc_cdutrig_t *t, agc_mat_t *nb);

/* SET1/PDT (CM) / the NCOARSE preamble (LM): 1/PIPADT = TIME1, GCOMP = 0,
 * DRIFTFLG set.                                                          */
agc_status_t align_set1_pdt(void);

/* The VN pseudo-op: verb and noun as decimal pairs packed 0VVVVVVVNNNNNNN
 * (same packing as agc_dsky_vn(), usable in constant expressions).       */
#define ALIGN_VN(verb, noun) ((uint16_t) (((verb) << 7) | (noun)))

/* Priorities used by the mark system and the jobs it creates.            */
#define ALIGN_PRIO13 ((agc_prio_t) 013) /* PRIO13 */
#define ALIGN_PRIO15 ((agc_prio_t) 015) /* PRIO15 */
#define ALIGN_PRIO24 ((agc_prio_t) 024) /* PRIO24 */
#define ALIGN_PRIO32 ((agc_prio_t) 032) /* PRIO32 */
#define ALIGN_PRIO5  ((agc_prio_t) 05)  /* PRIO5 */
#define ALIGN_PRIO4  ((agc_prio_t) 04)  /* PRIO4 */

/* Restart group 4 phase codes used by P50-P57.                           */
#define ALIGN_PH_04024 ((uint16_t) 04024)  /* type A: group 4, phase 2     */
#define ALIGN_PH_00014 ((uint16_t) 000014) /* type A: group 4, phase 1     */
#define ALIGN_PH_05024 ((uint16_t) 05024)  /* type C: group 4 + job PRIO13 */
#define ALIGN_PH_00254 ((uint16_t) 000254) /* PROG52 (CM): group 4 phase 25*/

#endif /* AGC_ALIGN_H */
