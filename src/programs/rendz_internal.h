/*
 * rendz_internal.h -- private interface between the files of the
 * rendezvous programs (src/programs/rendz_*.c, r30.c, r31.c, ...).
 */
#ifndef RENDZ_INTERNAL_H
#define RENDZ_INTERNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_exec.h"
#include "agc/agc_state.h"
#include "agc/agc_dsky.h"
#include "agc/agc_programs.h"
#include "rendz_ext.h"

/* ---- fixed constants shared by several files ---------------------------- */
/* TWOPI 2DEC 6.283185307 B-4 (P34-35 p.485), radians scale 2^4          */
#define RZ_TWOPI_B4      ((agc_dp_t)105414357)
/* 2PISC 2DEC 6.28318530 B-6 (CONIC_SUBROUTINES), radians scale 2^6        */
#define RZ_2PISC_B6      ((agc_dp_t)26353589)
/* DP-.01: OCT 77777, 61337 = -.01 degree in revolutions (2^0); the +MGA
 * "not computed" display value (-.02 deg for FINALFLG)                     */
#define RZ_DP_MINUS_01   ((agc_dp_t)-7456)
/* DPPOSMAX: OCT 37777 37777, the largest DP fraction                      */
#define RZ_DPPOSMAX      AGC_DP_MAX
/* MAXNM 2OCT 0106505603: 9999.9 NM, m 2^29 (R30 p.523)                    */
#define RZ_MAXNM         ((agc_dp_t)9259907)
/* 600SEC / TMIN 2DEC 60000 (cs, 2^28)                                     */
#define RZ_600SEC        ((agc_dp_t)60000)
/* 60MIN 2DEC 360000                                                       */
#define RZ_60MIN         ((agc_dp_t)360000)

/* ---- vehicle-dependent flags -------------------------------------------- */
/* agc_flags_csm.h and agc_flags_lm.h cannot be included together; these
 * flags differ in position (or exist in one vehicle only). Each vehicle's
 * id is taken from its own header in rendz_flags_csm.c / rendz_flags_lm.c. */
typedef enum rz_vflag {
    RZ_VF_ITSWICH = 0,  /* CSM 106D, LM 105D                               */
    RZ_VF_AVEGFLAG,     /* CSM 029D, LM 115D                               */
    RZ_VF_V82EMFLG,     /* CSM 137D, LM 118D                               */
    RZ_VF_P21FLAG,      /* CSM 033D, LM 004D                               */
    RZ_VF_COUNT
} rz_vflag_t;
agc_flag_id_t rz_vflag(rz_vflag_t f);
agc_flag_id_t rz_vflag_csm(rz_vflag_t f);
agc_flag_id_t rz_vflag_lm(rz_vflag_t f);
bool          rz_vflag_test(rz_vflag_t f);
agc_status_t  rz_vflag_set(rz_vflag_t f, bool on);
bool          rz_is_csm(void);
/* MOONTHIS: CMOONFLG in the CM, LMOONFLG in the LM                       */
bool          rz_moonthis(void);
agc_status_t  rz_flag_put(agc_flag_id_t f, bool on);

/* ---- glue to other lanes (rendz_glue.c) --------------------------------- */
agc_status_t rz_timethet(agc_interp_t *ip, const agc_timethet_in_t *in,
                         agc_timethet_out_t *out);
agc_status_t rz_timerad(agc_interp_t *ip, const agc_timerad_in_t *in,
                        agc_timethet_out_t *out);
agc_status_t rz_lambert(agc_interp_t *ip, const agc_lambert_in_t *in,
                        agc_lambert_out_t *out);
agc_status_t rz_apsides(agc_interp_t *ip, agc_vec_t r, agc_vec_t v, bool moon,
                        agc_dp_t *rapo, agc_dp_t *rperi, agc_dp_t *ecc);
typedef enum rz_integ_kind {
    RZ_CSMPREC = 0, RZ_LEMPREC, RZ_CSMCONIC, RZ_LEMCONIC, RZ_THISPREC, RZ_OTHPREC
} rz_integ_kind_t;
agc_status_t rz_integ(agc_interp_t *ip, rz_integ_kind_t kind, agc_dp_t tdec1,
                      agc_state_vector_t *out);
agc_status_t rz_integrvs(agc_interp_t *ip, const agc_integrvs_in_t *in,
                         agc_state_vector_t *out);
agc_status_t rz_tff_conmu(agc_interp_t *ip, agc_vec_t rone, agc_vec_t vone,
                          agc_dp_t tff_rtmu, agc_tff_ctx_t *ctx);
agc_status_t rz_tff_rpra(agc_interp_t *ip, agc_tff_ctx_t *ctx, agc_dp_t *rapo,
                         agc_dp_t *rper);
agc_status_t rz_tff_calctper(agc_interp_t *ip, agc_tff_ctx_t *ctx,
                             agc_dp_t rterm, agc_dp_t *tper);
agc_status_t rz_tff_calctff(agc_interp_t *ip, agc_tff_ctx_t *ctx,
                            agc_dp_t rterm, agc_dp_t *tff);
agc_vec_t    rz_rls(void);
/* Flashing display; terminate/proceed/enter continuation jobs.           */
agc_status_t rz_flash(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind,
                      uint8_t verb, uint8_t noun, agc_job_id_t term,
                      agc_job_id_t proceed, agc_job_id_t enter);
agc_status_t rz_gotopooh(void);
agc_status_t rz_endext(void);
agc_status_t rz_delayjob(uint16_t delay_cs, agc_job_id_t cont);
agc_dp_t     rz_loadtime(void);    /* LOADTIME: TIME2,TIME1 as DP cs 2^28 */

/* ---- common helpers (rendz_common.c) ------------------------------------ */
agc_rendz_t *rz_state(void);
agc_vec_t    rz_unit(agc_interp_t *ip, agc_vec_t v);
agc_dp_t     rz_abval(agc_interp_t *ip, agc_vec_t v);
/* NORM: left-justify a, *n = number of left shifts (AGC index = -n).     */
agc_dp_t     rz_norm(agc_dp_t a, int *n);
/* Shift by a signed count: k > 0 left (SL), k < 0 right (SR, truncating). */
agc_dp_t     rz_shift(agc_interp_t *ip, agc_dp_t a, int k);
agc_vec_t    rz_vshift(agc_interp_t *ip, agc_vec_t v, int k);
/* Body scale <-> earth scale for vectors (VSR* 0,2 / VSL* 0,2).          */
agc_vec_t    rz_vec_from_body(agc_interp_t *ip, const agc_rendz_t *rz,
                              agc_vec_t v);
/* AVFLAGA / AVFLAGP: AVFLAG reset = CSM active, set = LM active.         */
agc_status_t rz_avflag(bool this_active);
agc_status_t rz_p20flgon(void);    /* P20FLGON: set UPDATFLG, TRACKFLG    */
/* PRECSET: precision update of both vehicles to tdec1 (ACTIVE/PASSIVE).  */
agc_status_t rz_precset(agc_interp_t *ip, agc_rendz_t *rz, agc_dp_t tdec1);
/* INTINT: integrate r (2^29) v (2^7) from t1 to t2, conic or precision.  */
agc_status_t rz_intint(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t r,
                       agc_vec_t v, agc_dp_t t1, agc_dp_t t2, bool conic,
                       agc_state_vector_t *out);
/* TIMETHET with the AGC call set-up (VECSHIFT of r, v and X1 = RTX1).    */
agc_status_t rz_timethet_rv(agc_interp_t *ip, const agc_rendz_t *rz,
                            agc_vec_t r, agc_vec_t v, agc_dp_t snth,
                            agc_dp_t csth, bool rvsw, agc_timethet_out_t *out);
/* VN1645: TRKMKCNT, TTOGO, +MGA monitor with the recycle/proceed logic;
 * resume_as runs after PROCEED/RECYCLE (the AGC's QSAVED return).        */
agc_status_t rz_vn1645(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                       agc_rendz_t *rz, agc_job_id_t resume_as);
/* VNPOOH: flash verb/noun; TERM -> P00, PROCEED -> next, ENTER -> same. */
agc_status_t rz_vnpooh(const agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun,
                       agc_job_id_t self, agc_job_id_t next);
agc_status_t rz_s34_35_5(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                         agc_rendz_t *rz, agc_job_id_t resume_as);
agc_status_t rz_rendz_alarm(const agc_job_ctx_t *ctx, agc_alarm_code_t code,
                            agc_job_id_t recycle);

/* Job bodies registered for dispatch (see rendz_ext.h ids).               */
agc_status_t agc_job_rendz_vn1645_dsp(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendz_vn1645_term(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendz_vn1645_proc(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendz_vn1645_recyc(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendz_s3435_n81(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendz_s3435_n59(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendz_gotopooh(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendz_endext(agc_job_ctx_t *ctx);

#endif /* RENDZ_INTERNAL_H */
