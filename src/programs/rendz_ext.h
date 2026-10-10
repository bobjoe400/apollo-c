/*
 * rendz_ext.h -- interfaces the rendezvous programs (prog-rendz) use that
 * other lanes own and had not published when this module was written.
 *
 * INTEGRATION: every declaration below is an ASSUMED interface. When the
 * owning header appears, delete the matching block here and include that
 * header instead. All calls go through src/programs/rendz_glue.c, so a
 * signature difference is fixed in that one file. Declarations shared with
 * src/programs/align_pending.h are spelled identically so both can be
 * resolved by one implementation. Listed in docs/modules/prog-rendz.md
 * under "Requested core additions".
 *
 * Job ids: object-like constants AGC_JOB_<LABEL> with provisional values in
 * the 0x0B00 range (prog-align uses 0x0A00); the lead replaces them with
 * agc_jobs.h members and dispatch.c cases.
 */
#ifndef RENDZ_EXT_H
#define RENDZ_EXT_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_exec.h"
#include "agc/agc_state.h"

/* ==== state-flags: shared erasables not (yet) in agc_state.h ============ */
agc_vec_t agc_erase_rls(void);                 /* RLS m 2^27 moon-fixed      */
uint16_t  agc_erase_extvbact(void);            /* EXTVBACT                   */
void      agc_erase_set_extvbact(uint16_t v);
agc_sp_t  agc_erase_option1(int index);        /* OPTION1 +index (N06)       */
void      agc_erase_set_option1(int index, agc_sp_t v);
agc_sp_t  agc_erase_option2(void);             /* OPTION2 (OPTIONX +1)       */
void      agc_erase_set_option2(agc_sp_t v);
agc_dp_t  agc_erase_dsptem1(void);             /* DSPTEM1,+1 as DP           */
void      agc_erase_set_dsptem1(agc_dp_t v);
agc_vec_t agc_erase_alphav(void);              /* ALPHAV                     */
void      agc_erase_set_alphav(agc_vec_t v);

/* ECSTEER (P30-P37 AVFLAGA, CM): cross-product steering constant c,
 * set to 1 (BIT13, scale 2^1) when the CSM is the active vehicle.        */
void      agc_erase_set_ecsteer(agc_sp_t v);
/* DISPDEX (TTOGO clock display index) and TIMRFLAG side of COMPTGO.     */
void      agc_erase_set_dispdex(agc_sp_t v);

/* ==== exec-sched ========================================================== */
agc_status_t agc_exec_gotopooh(void);                                 /* GOTOPOOH */
agc_status_t agc_exec_delayjob(uint16_t delay_cs, agc_job_id_t cont); /* DELAYJOB */
agc_status_t agc_dsky_endext(void);                                   /* ENDEXT   */

/* ==== nav-integ (agc_integ.h) ============================================ */
/* Conic/precision update of the permanent state vector to TDEC1:
 * out->r = RATT (m 2^29), out->v = VATT (m/cs 2^7), out->t = TAT,
 * out->moon = (X2 == 2), i.e. RATT1/VATT1 are at body scale.           */
agc_status_t agc_integ_csmconic(agc_interp_t *ip, agc_dp_t tdec1, agc_state_vector_t *out);
agc_status_t agc_integ_lemconic(agc_interp_t *ip, agc_dp_t tdec1, agc_state_vector_t *out);
agc_status_t agc_integ_csmprec(agc_interp_t *ip, agc_dp_t tdec1, agc_state_vector_t *out);
agc_status_t agc_integ_lemprec(agc_interp_t *ip, agc_dp_t tdec1, agc_state_vector_t *out);

/* INTEGRVS: integrate a caller-supplied state (RCV, VCV at body scale,
 * TET) to TDEC1; conic = INTYPFLG set; moon = MOONFLAG. Output as above. */
typedef struct agc_integrvs_in {
    agc_vec_t rcv;      /* RCV m, 2^29 earth / 2^27 moon                  */
    agc_vec_t vcv;      /* VCV m/cs, 2^7 earth / 2^5 moon                 */
    agc_dp_t  tet;      /* TET cs 2^28                                    */
    agc_dp_t  tdec1;    /* TDEC1 cs 2^28                                  */
    bool      moon;     /* MOONFLAG                                       */
    bool      conic;    /* INTYPFLG                                       */
} agc_integrvs_in_t;
agc_status_t agc_integ_integrvs(agc_interp_t *ip, const agc_integrvs_in_t *in,
                                agc_state_vector_t *out);
agc_status_t agc_integ_intstall(void);         /* INTSTALL (integration lock) */
agc_status_t agc_integ_intwake(void);          /* INTWAKE / INTWAKE1          */
/* THISPREC / OTHPREC: this / other vehicle permanent state to TDEC1.    */
agc_status_t agc_integ_thisprec(agc_interp_t *ip, agc_dp_t tdec1, agc_state_vector_t *out);
agc_status_t agc_integ_othprec(agc_interp_t *ip, agc_dp_t tdec1, agc_state_vector_t *out);
/* ATOPCSM / ATOPLEM (via ATOPOTH): store RATT/VATT/TAT as the permanent
 * state of the other vehicle (P76).                                     */
agc_status_t agc_integ_atopoth(const agc_state_vector_t *sv);

/* ==== nav-conic (agc_nav.h) =============================================== */
/* TIMETHET (CONIC_SUBROUTINES p.1269): r, v at body scale, snth/csth 2^1.
 * rvsw set: time only. Output t cs 2^28; when !rvsw the new state at body
 * scale. Debris kept: r1 (body scale), r1a (2^6), p (2^4).              */
typedef struct agc_timethet_in {
    agc_vec_t rvec, vvec;
    agc_dp_t  snth, csth;
    bool      rvsw;
    bool      moon;
} agc_timethet_in_t;
typedef struct agc_timethet_out {
    agc_dp_t  t;
    agc_vec_t rvec, vvec;
    agc_dp_t  r1, r1a, p;
} agc_timethet_out_t;
agc_status_t agc_conic_timethet(agc_interp_t *ip, const agc_timethet_in_t *in,
                                agc_timethet_out_t *out);

/* TIMERAD (p.1272): rdesired body scale, sgnrdot +.5/-.5 (SP).         */
typedef struct agc_timerad_in {
    agc_vec_t rvec, vvec;
    agc_dp_t  rdesired;
    agc_sp_t  sgnrdot;
    bool      rvsw;
    bool      moon;
} agc_timerad_in_t;
agc_status_t agc_conic_timerad(agc_interp_t *ip, const agc_timerad_in_t *in,
                               agc_timethet_out_t *out);

/* LAMBERT (p.1266): r1vec, r2vec body scale, tdesired cs 2^28,
 * geomsgn +.5/-.5, un 2^1 used when normsw. Output vvec, vtarget body
 * scale; solnsw set when no solution.                                  */
typedef struct agc_lambert_in {
    agc_vec_t r1vec, r2vec;
    agc_dp_t  tdesired;
    agc_sp_t  geomsgn;
    bool      guessw;
    agc_dp_t  coga;
    bool      normsw;
    agc_vec_t un;
    bool      vtarget_wanted;  /* VTARGTAG == 0                           */
    bool      moon;
} agc_lambert_in_t;
typedef struct agc_lambert_out {
    agc_vec_t vvec, vtarget;
    agc_dp_t  coga;
    agc_dp_t  r1, r1a;         /* debris: R1 body scale, R1A 2^6           */
    bool      solnsw;
} agc_lambert_out_t;
agc_status_t agc_conic_lambert(agc_interp_t *ip, const agc_lambert_in_t *in,
                               agc_lambert_out_t *out);

/* APSIDES (p.1275): r, v body scale -> rapo, rperi body scale, ecc 2^3. */
agc_status_t agc_conic_apsides(agc_interp_t *ip, agc_vec_t rvec,
                               agc_vec_t vvec, bool moon, agc_dp_t *rapo,
                               agc_dp_t *rperi, agc_dp_t *ecc);

/* ==== nav-aux (agc_navaux.h) TIME_OF_FREE_FALL ============================ */
/* The TFF routines keep their working set (RMAG1, NRMAG, TFFNP, TFFALFA,
 * TFFRTALF, TFF1/ALF, TFFVSQ, X1, X2 ...) in a context between calls.   */
typedef struct agc_tff_ctx {
    agc_dp_t w[20];            /* push-list area 10D..39D, opaque here      */
    agc_sp_t x1, x2;
} agc_tff_ctx_t;
/* TFFCONMU: rone/vone at body scale, tff_rtmu 2^17 / 2^14.              */
agc_status_t agc_tff_conmu(agc_interp_t *ip, agc_vec_t rone, agc_vec_t vone,
                           agc_dp_t tff_rtmu, agc_tff_ctx_t *ctx);
/* TFFRP/RA: rapo, rper at body scale (RAPO limited to scale max).      */
agc_status_t agc_tff_rpra(agc_interp_t *ip, agc_tff_ctx_t *ctx,
                          agc_dp_t *rapo, agc_dp_t *rper);
/* CALCTPER: time to pericenter (rper input), cs 2^28.                  */
agc_status_t agc_tff_calctper(agc_interp_t *ip, agc_tff_ctx_t *ctx,
                              agc_dp_t rterm, agc_dp_t *tper);
/* CALCTFF: time of free fall to rterm, cs 2^28 (59m59s if none).      */
agc_status_t agc_tff_calctff(agc_interp_t *ip, agc_tff_ctx_t *ctx,
                             agc_dp_t rterm, agc_dp_t *tff);
/* LAT-LONG, same spelling as align_pending.h.                          */
agc_status_t agc_navaux_lat_long(agc_interp_t *ip, agc_vec_t alphav, agc_dp_t t,
                                 agc_dp_t *lat, agc_dp_t *lng, agc_dp_t *alt);

/* ==== guid-powered (POWERED_FLIGHT_SUBROUTINES), same as align_pending ==== */
agc_vec_t agc_pfs_nbsm(agc_interp_t *ip, const uint16_t cdu[3], agc_vec_t v); /* *NBSM* */

/* ==== provisional job ids (prog-rendz) ==================================== */
#define AGC_JOB_P32_N11       ((agc_job_id_t)0x0B01) /* P32/P72A V06N11 TCSI   */
#define AGC_JOB_P32_N55       ((agc_job_id_t)0x0B02) /* V06N55 NN ELEV         */
#define AGC_JOB_P32_N37       ((agc_job_id_t)0x0B03) /* V06N37 TTPI            */
#define AGC_JOB_P32_CALC      ((agc_job_id_t)0x0B04) /* P32/P72B compute       */
#define AGC_JOB_P32_N75       ((agc_job_id_t)0x0B05) /* P32/P72F after N75     */
#define AGC_JOB_P32_N81       ((agc_job_id_t)0x0B06) /* after V06N81           */
#define AGC_JOB_P32_N82       ((agc_job_id_t)0x0B07) /* after V06N82           */
#define AGC_JOB_P32_ALARM     ((agc_job_id_t)0x0B08) /* ALMXIT V05N09 answer   */
#define AGC_JOB_P33_N13       ((agc_job_id_t)0x0B09) /* P33/P73A V06N13 TCDH   */
#define AGC_JOB_P33_CALC      ((agc_job_id_t)0x0B0A) /* P33/P73B compute       */
#define AGC_JOB_P33_N75       ((agc_job_id_t)0x0B0B)
#define AGC_JOB_P33_N81       ((agc_job_id_t)0x0B0C)
#define AGC_JOB_P34_N37       ((agc_job_id_t)0x0B0D) /* P34/P74A V06N37        */
#define AGC_JOB_P34_N55       ((agc_job_id_t)0x0B0E) /* DISPLAYE V06N55        */
#define AGC_JOB_P34_CALC      ((agc_job_id_t)0x0B0F) /* P34/P74C compute       */
#define AGC_JOB_P34_N58       ((agc_job_id_t)0x0B10)
#define AGC_JOB_P35_CALC      ((agc_job_id_t)0x0B11) /* P35/P75B               */
#define AGC_JOB_S3435_N81     ((agc_job_id_t)0x0B12) /* S34/35.5 after V06N81  */
#define AGC_JOB_S3435_N59     ((agc_job_id_t)0x0B13) /* S34/35.5 after V06N59  */
#define AGC_JOB_VN1645_DSP    ((agc_job_id_t)0x0B14) /* VN1645 after DELAYJOB  */
#define AGC_JOB_VN1645_TERM   ((agc_job_id_t)0x0B15) /* KILCLOCK               */
#define AGC_JOB_VN1645_PROC   ((agc_job_id_t)0x0B16) /* N45PROC                */
#define AGC_JOB_VN1645_RECYC  ((agc_job_id_t)0x0B17) /* CLUPDATE               */
#define AGC_JOB_RENDZ_ALARM   ((agc_job_id_t)0x0B18) /* V05N09 after 611       */
#define AGC_JOB_P38_N33       ((agc_job_id_t)0x0B19)
#define AGC_JOB_P38_N55       ((agc_job_id_t)0x0B1A)
#define AGC_JOB_P38_N06       ((agc_job_id_t)0x0B1B)
#define AGC_JOB_P38_N57       ((agc_job_id_t)0x0B1C)
#define AGC_JOB_P38_N34       ((agc_job_id_t)0x0B1D)
#define AGC_JOB_P38_RECYCLE   ((agc_job_id_t)0x0B1E)
#define AGC_JOB_P38_N58       ((agc_job_id_t)0x0B1F)
#define AGC_JOB_P38_N81       ((agc_job_id_t)0x0B20)
#define AGC_JOB_P39_CALC      ((agc_job_id_t)0x0B21)
#define AGC_JOB_P17_N37       ((agc_job_id_t)0x0B22)
#define AGC_JOB_P17_N72       ((agc_job_id_t)0x0B23)
#define AGC_JOB_P17_N58       ((agc_job_id_t)0x0B24)
#define AGC_JOB_P17_N55       ((agc_job_id_t)0x0B25)
#define AGC_JOB_P31_N33       ((agc_job_id_t)0x0B26)
#define AGC_JOB_P31_N81       ((agc_job_id_t)0x0B27)
#define AGC_JOB_P31_N42       ((agc_job_id_t)0x0B28)
#define AGC_JOB_P76_N84       ((agc_job_id_t)0x0B29)
#define AGC_JOB_P76_N33       ((agc_job_id_t)0x0B2A)
#define AGC_JOB_P21_OPTION    ((agc_job_id_t)0x0B2B)
#define AGC_JOB_P21_N34       ((agc_job_id_t)0x0B2C)
#define AGC_JOB_P21_N43       ((agc_job_id_t)0x0B2D)
#define AGC_JOB_V82_OPTION    ((agc_job_id_t)0x0B2E) /* V82GOFF V04N06 answer  */
#define AGC_JOB_V82GOFF1      ((agc_job_id_t)0x0B2F) /* FINDVAC V82GOFF1       */
#define AGC_JOB_V82_N44       ((agc_job_id_t)0x0B30) /* V16N44 monitor answer  */
#define AGC_JOB_V82GON1       ((agc_job_id_t)0x0B31) /* FINDVAC V82GON1 / loop */
#define AGC_JOB_V83CALL       ((agc_job_id_t)0x0B32) /* FINDVAC V83CALL        */
#define AGC_JOB_V83_DISP      ((agc_job_id_t)0x0B33) /* DISPN5X                */
#define AGC_JOB_V90_OPTION    ((agc_job_id_t)0x0B34) /* R36 V04N12 answer      */
#define AGC_JOB_V90_N16       ((agc_job_id_t)0x0B35) /* R36P3 V06N16 answer    */
#define AGC_JOB_V90_N90       ((agc_job_id_t)0x0B36) /* R36TAG2 V06N90 answer  */
#define AGC_JOB_ENDEXT        ((agc_job_id_t)0x0B37) /* TERM of ext verb       */
#define AGC_JOB_GOTOPOOH      ((agc_job_id_t)0x0B38) /* TERM -> P00            */
#define AGC_JOB_DORSAMP       ((agc_job_id_t)0x0B39) /* LM radar sample        */

/* ==== provisional task ids ================================================ */
#define AGC_TASK_TICKTEST     ((agc_task_id_t)0x0B01) /* V82 1 s tick          */
#define AGC_TASK_DSPDELAY     ((agc_task_id_t)0x0B02) /* LM R31 display delay  */
#define AGC_TASK_RADSAMP      ((agc_task_id_t)0x0B03) /* LM radar sampling     */
#define AGC_TASK_CLOKTASK     ((agc_task_id_t)0x0B04) /* COMPTGO -> CLOKTASK   */

#endif /* RENDZ_EXT_H */
