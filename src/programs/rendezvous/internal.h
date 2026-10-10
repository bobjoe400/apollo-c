/*
 * internal.h -- private interface between the files of the
 * rendezvous programs (src/programs/rendezvous/rendz_*.c, r30.c, r31_*.c, ...).
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
#include "agc/agc_nav.h"
#include "agc/agc_integ.h"
#include "agc/agc_programs.h"
#include "ext.h"

/* ---- fixed constants shared by several files ---------------------------- */
/* TWOPI 2DEC 6.283185307 B-4 (P34-35 p.485), radians scale 2^4; the CM
 * P32 "2PISC SL2" (6.28318530 B-6 shifted left 2) is the same value.    */
#define RZ_TWOPI_B4      ((agc_dp_t)105414357)
/* DP-.01: OCT 77777, 61337 = -.01 degree in revolutions (2^0); the +MGA
 * "not computed" display value (-.02 deg for FINALFLG)                    */
#define RZ_DP_MINUS_01   ((agc_dp_t)-7456)
/* DPPOSMAX: OCT 37777 37777, the largest DP fraction                      */
#define RZ_DPPOSMAX      AGC_DP_MAX
/* DPHALF 2DEC .5                                                          */
#define RZ_DPHALF        AGC_DP_HALF
/* MAXNM 2OCT 0106505603: 9999.9 NM, m 2^29 (R30 p.523)                    */
#define RZ_MAXNM         ((agc_dp_t)9259907)
/* 600SEC / TMIN 2DEC 60000 (cs, 2^28)                                     */
#define RZ_600SEC        ((agc_dp_t)60000)
/* 60MIN 2DEC 360000                                                       */
#define RZ_60MIN         ((agc_dp_t)360000)
/* 1SEC: DELAYJOB / WAITLIST delay 100 cs                                  */
#define RZ_1SEC_CS       ((uint16_t)100)
/* RPAD 2DEC 6373338 B-29: standard radius of pad 37-B (P34-35 p.496)     */
#define RZ_RPAD_B29      ((agc_dp_t)3186669)
/* SN359+ 2DEC -.000086601 / CS359+ 2DEC +.499999992: sin, cos of 359.99
 * degrees at scale 2^1 (P32-P33 p.668)                                     */
#define RZ_SN359         ((agc_dp_t)-23247)
#define RZ_CS359         ((agc_dp_t)134217726)

/* ---- vehicle-dependent flags -------------------------------------------- */
/* agc_flags_csm.h and agc_flags_lm.h cannot be included together; these
 * flags differ in position (or exist in one vehicle only, id 0 = absent).
 * Each vehicle's id comes from flags_csm.c / flags_lm.c.     */
typedef enum rz_vflag {
    RZ_VF_ITSWICH = 0,  /* CSM 106D, LM 105D                               */
    RZ_VF_AVEGFLAG,     /* CSM 029D, LM 115D                               */
    RZ_VF_ORBIT_DISPLAY_MOON,     /* CSM 137D, LM 118D                               */
    RZ_VF_GROUND_TRACK,      /* CSM 033D, LM 004D                               */
    RZ_VF_CSI_F1,      /* CSI indicators, both vehicles, other positions  */
    RZ_VF_CSI_F2,
    RZ_VF_CSI_F3A,
    RZ_VF_CSI_F3B,
    RZ_VF_NTARGFLG,     /* LM only                                         */
    RZ_VF_COMPUTER,     /* CSM only (MIDGIM)                               */
    RZ_VF_AMOONFLG,     /* CSM only (R30)                                  */
    RZ_VF_RENDZ_DISPLAY,      /* CSM only (R31/R34)                              */
    RZ_VF_CHANGE_PROGRAM,      /* both                                            */
    RZ_VF_MUNFLAG,      /* LM only (R31)                                   */
    RZ_VF_TIMRFLAG,     /* CSM only (COMPTGO)                              */
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

/* ---- glue to other lanes (glue.c) --------------------------------- */
/* Conic body from RTX1 (-2 earth, -10 moon).                              */
agc_conic_body_t rz_body(const agc_rendz_t *rz);
agc_status_t rz_timethet(agc_interp_t *ip, const agc_timetheta_in_t *in,
                         agc_conic_time_out_t *out);
agc_status_t rz_timerad(agc_interp_t *ip, const agc_timerad_in_t *in,
                        agc_conic_time_out_t *out);
/* LAMBERT (CM) / SETITCTR + LAMBERT (LM).                                 */
agc_status_t rz_lambert(agc_interp_t *ip, agc_lambert_in_t *in,
                        agc_lambert_out_t *out);
agc_status_t rz_apsides(agc_interp_t *ip, agc_vec_t r, agc_vec_t v,
                        agc_conic_body_t body, agc_apsides_out_t *out);
typedef enum rz_integ_kind {
    RZ_CSMPREC = 0, RZ_LEMPREC, RZ_CSMCONIC, RZ_LEMCONIC, RZ_THISPREC,
    RZ_OTHPREC, RZ_INTEG_KIND_COUNT
} rz_integ_kind_t;
/* Permanent state to tdec1; *out = the integration push-list output
 * (RATT/VATT earth scale, RATT1/VATT1 body scale, TAT, X1, X2).
 * AGC_STATUS_BUSY: INTSTALL put the job to sleep; it resumes as `resume`,
 * which must redo the computation step from its start.                   */
agc_status_t rz_integ(agc_interp_t *ip, rz_integ_kind_t kind, agc_dp_t tdec1,
                      agc_job_id_t resume, agc_integ_out_t *out);
/* INTSTALL; RCV, VCV (body scale), TET, MOONFLAG, INTYPFLG; INTEGRVS.    */
typedef struct rz_integrvs_in {
    agc_vec_t rcv;      /* RCV m, 2^29 earth / 2^27 moon                  */
    agc_vec_t vcv;      /* VCV m/cs, 2^7 earth / 2^5 moon                 */
    agc_dp_t  tet;      /* TET cs 2^28                                    */
    agc_dp_t  tdec1;    /* TDEC1 cs 2^28                                  */
    bool      moon;     /* MOONFLAG                                       */
    bool      conic;    /* INTYPFLG                                       */
} rz_integrvs_in_t;
agc_status_t rz_integrvs(agc_interp_t *ip, const rz_integrvs_in_t *in,
                         agc_job_id_t resume, agc_integ_out_t *out);
agc_vec_t    rz_rls(void);
/* GOFLASH-type display with the three astronaut returns.                  */
agc_status_t rz_flash(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind,
                      uint8_t verb, uint8_t noun, agc_job_id_t term,
                      agc_job_id_t proceed, agc_job_id_t enter);
agc_status_t rz_gotopooh(void);
agc_status_t rz_endext(void);
agc_status_t rz_delayjob(uint16_t delay_cs, agc_job_id_t cont);
agc_dp_t     rz_loadtime(void);    /* LOADTIME: TIME2,TIME1 as DP cs 2^28 */
agc_status_t rz_alarm(agc_alarm_code_t code);
/* COMPTGO; the restart (group 4 type C) resumes as job cont.           */
agc_status_t rz_comptgo(agc_job_id_t cont);
/* DISPDEX = 0 (KILCLOCK / CLUPDATE).                                     */
void         rz_dispdex_zero(void);
/* OPTION1 / OPTION2 (= DSPTEMX = DSPTEM2 +1, +2), raw SP words.          */
uint16_t     rz_option(int which);
void         rz_set_option(int which, uint16_t v);

/* ---- common helpers (common.c) ------------------------------------ */
agc_rendz_t *rz_state(void);
agc_dp_t     rz_mutable_mu(bool moon);  /* MUE 2^36 / MUM 2^30            */
agc_vec_t    rz_unit(agc_interp_t *ip, agc_vec_t v);
agc_dp_t     rz_abval(agc_interp_t *ip, agc_vec_t v);
agc_vec_t    rz_vsign(agc_vec_t v, agc_dp_t s);        /* vector SIGN   */
/* NORM: left-justify a; returns the AGC index register value (-shifts).  */
agc_dp_t     rz_norm(agc_dp_t a, int *xreg);
/* Shift by a signed count: k > 0 left (SL), k < 0 right (SR, truncating). */
agc_dp_t     rz_shift(agc_interp_t *ip, agc_dp_t a, int k);
/* SL* addr,X / SR* addr,X: count = addr - C(X) (AGC indexing subtracts). */
agc_dp_t     rz_sl_star(agc_interp_t *ip, agc_dp_t a, int addr, int xreg);
agc_dp_t     rz_sr_star(agc_interp_t *ip, agc_dp_t a, int addr, int xreg);
agc_vec_t    rz_vshift(agc_interp_t *ip, agc_vec_t v, int k);
/* Body scale <-> earth scale for vectors (VSR* 0,2 / VSL* 0,2).          */
agc_vec_t    rz_vec_from_body(agc_interp_t *ip, const agc_rendz_t *rz,
                              agc_vec_t v);
/* AVFLAGA / AVFLAGP: AVFLAG reset = CSM active, set = LM active.         */
agc_status_t rz_avflag(bool this_active);
agc_status_t rz_tracking_flags_on(void);    /* P20FLGON: set UPDATFLG, TRACKFLG    */
/* PRECSET: precision update of both vehicles to tdec1 (ACTIVE/PASSIVE).  */
agc_status_t rz_precset(agc_interp_t *ip, agc_rendz_t *rz, agc_dp_t tdec1,
                        agc_job_id_t resume);
/* INTINT: integrate r (2^29) v (2^7) from t1 to t2, conic or precision;
 * *ratt, *vatt earth scale (RATT, VATT).                                 */
agc_status_t rz_intint(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t r,
                       agc_vec_t v, agc_dp_t t1, agc_dp_t t2, bool conic,
                       agc_job_id_t resume, agc_vec_t *ratt, agc_vec_t *vatt);
/* TIMETHET with the AGC call set-up (VECSHIFT of r, v and X1 = RTX1).    */
agc_status_t rz_timethet_rv(agc_interp_t *ip, const agc_rendz_t *rz,
                            agc_vec_t r, agc_vec_t v, agc_dp_t snth,
                            agc_dp_t csth, bool rvsw,
                            agc_conic_time_out_t *out);
/* ACTIVE / PASSIVE: store an earth-scale state as RACT3/VACT3 or
 * RPASS3/VPASS3.                                                         */
void         rz_store_active(agc_rendz_t *rz, agc_vec_t r, agc_vec_t v);
void         rz_store_passive(agc_rendz_t *rz, agc_vec_t r, agc_vec_t v);

/* ---- displays (display.c) ----------------------------------------- */
/* VNPOOH: flash verb/noun; TERM -> P00, PROCEED -> next, ENTER -> again. */
agc_status_t rz_vnpooh(const agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun,
                       agc_job_id_t next);
/* VN1645: +MGA, COMPTGO, DELAYJOB 1 s, V16N45 flash; the job continues as
 * resume_as after PROCEED (first time) or ENTER (QSAVED return).         */
agc_status_t rz_ttogo_display(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                              agc_rendz_t *rz, agc_job_id_t resume_as);
/* DISPLAYE: V06N55 flash, return = next.                                 */
agc_status_t rz_displaye(const agc_job_ctx_t *ctx, agc_job_id_t next);
/* S34/35.5 (FINALFLG / N81 / N59 logic), continues as resume_as.         */
agc_status_t rz_tpi_final_displays(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                                   agc_rendz_t *rz, agc_job_id_t resume_as);
/* FLAGOFF part of S34/35.5: S34/35.4, V06N59, return to NORMEX.          */
agc_status_t rz_tpi_flagoff(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                            agc_rendz_t *rz);
/* ALARM code then V05N09 flash: TERM -> P00, PROCEED -> proceed,
 * ENTER -> recycle.                                                      */
agc_status_t rz_alarm_flash(const agc_job_ctx_t *ctx, agc_alarm_code_t code,
                            agc_job_id_t proceed, agc_job_id_t recycle);
/* DISDVLVC: DELVLVC = dv rotated to the CSI/CDH display frame
 * (S32/33.X), V06N81 or V06N82 (noun), continue as next.                 */
agc_status_t rz_disdvlvc(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                         agc_rendz_t *rz, agc_vec_t dv, uint8_t noun,
                         agc_job_id_t next);
/* P3XORP7X: true when MODREG is a P3x program (this vehicle active).     */
bool         rz_active_vehicle_program(void);
/* Continue the running job at another entry (agc_exec_job_yield).        */
agc_status_t rz_continue(agc_job_id_t next);

#endif /* RENDZ_INTERNAL_H */
