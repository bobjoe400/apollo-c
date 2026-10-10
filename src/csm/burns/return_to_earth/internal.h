/*
 * internal.h -- shared between the P37 files of src/csm/burns/return_to_earth/
 * (program.c, conic.c, velocity.c,
 * precision.c). Not part of the public API.
 *
 * The P37 erasables (ERASABLE_ASSIGNMENTS.agc, EBANK7 "RTE" block from
 * RTEDVD to BETA12) are module storage: P37 is the only user. P37 refuses
 * a state in the moon's sphere (alarm 612), so every "B29/B27" pair of the
 * listing is the earth scale B29 here (CONICX1 = 2, X1 = -2).
 */
#ifndef RETURN_TO_EARTH_INTERNAL_H
#define RETURN_TO_EARTH_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/interp.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"

/* Iteration limits of the listing (counters run up to zero).               */
#define RTE_CONIC_PASSES 9   /* M9RTEB28: conic RCON / X(T2) passes      */
#define RTE_PREC_OUTER   10  /* 10RTE: NN1A precision RCON passes       */
#define RTE_PREC_INNER   15  /* M15RTE: NN2 DT21 passes per RCON pass   */
#define RTE_GAMDV_PASSES 144 /* M144RTE: X(T1)-DV iterator passes       */
/* Bound on precision integrations: NN1A can be reset to 10 once (V2T115),
 * so at most 2 x 11 outer passes of at most 16 integrations each.          */
#define RTE_PREC_MAX_STEPS 400

/* Alarm codes (octal, as in the listing).                                  */
#define RTE_ALARM_ITERATIONS \
    ((agc_alarm_code_t) 00605) /* OCT605 too many iterations */
#define RTE_POODOO_NO_SOLUTION \
    ((agc_alarm_code_t) 00610) /* V2TERROR lambda < 1     */
#define RTE_ALARM_MOON_SPHERE \
    ((agc_alarm_code_t) 00612) /* OCT612 not in earth sphere */
#define RTE_ALARM_ENTRY_ANGLE \
    ((agc_alarm_code_t) 00613) /* OCT613 reentry angle limits */
#define RTE_POODOO_SQRT \
    ((agc_alarm_code_t) 01302) /* SQRT of a negative number */

/* PREC100 resumption point (where an INTSTALL sleep returns to).           */
typedef enum rte_prec_step
{
    RTE_PREC_FROM_TIG = 0, /* PREC120: RTENCK3, R(T1)/ V2(T1)/ T1 -> T2 */
    RTE_PREC_TIME_STEP     /* PREC174: RTENCK2, R(T2)/ V(T2)/ T2 + DT21 */
} rte_prec_step_t;

/* The P37 erasables. Scales: m B29, m/cs B7, cs B28 unless noted.          */
typedef struct rte_erasable
{
    /* astronaut inputs */
    agc_dp_t rtedvd; /* RTEDVD   desired delta V (0 = minimum fuel), m/cs B7 */
    agc_dp_t
        rtegam2d; /* RTEGAM2D desired reentry angle (0 = computed), revs B0 */
    /* initial vector (INVC100) */
    agc_vec_t r_t1v; /* R(T1)/   position at TIG, m B29                      */
    agc_dp_t  r_t1;  /* R(T1)    |R(T1)/|, m B29                             */
    agc_vec_t v_t1v; /* V(T1)/   velocity at TIG (pre impulse), m/cs B7      */
    agc_dp_t  t1;    /* T1       time of R(T1)/ (TIG), cs B28                */
    agc_sp_t  p_t1;  /* P(T1)    primary body index X2: 0 earth, 2 moon      */
    agc_vec_t ur1;   /* UR1/     unit R(T1)/, B1                             */
    agc_vec_t uv1;   /* UV1/     unit V(T1)/, B1 (UH/ overlays it)           */
    agc_vec_t uh;    /* UH/      unit horizontal vector, B1                  */
    agc_dp_t  cfpa;  /* CFPA     cos of initial flight path angle, B1        */
    /* conic and precision solution */
    agc_dp_t  mamax1; /* MAMAX1   major axis lower-bound limit, m B30         */
    agc_dp_t  mamax2; /* MAMAX2   major axis upper-bound limit, m B30         */
    agc_dp_t  rcon;   /* RCON     conic final radius, m B29                   */
    agc_dp_t  x_t2;   /* X(T2)    cot of final flight path angle (from
                           *          the vertical), B0                          */
    agc_dp_t  beta1;  /* BETA1    1 + X(T2)^2, B1                             */
    agc_dp_t  x_t1;   /* X(T1)    cot of post impulse flight path angle, B5   */
    agc_dp_t  pcon;   /* PCON     semi-latus rectum, m B28                    */
    agc_vec_t v2_t1v; /* V2(T1)/  post impulse velocity, m/cs B7              */
    agc_dp_t  dv;     /* DV       |V2(T1)/ - V(T1)/|, m/cs B7                 */
    agc_vec_t r_t2v;  /* R(T2)/   final position, m B29                       */
    agc_vec_t v_t2v;  /* V(T2)/   final velocity, m/cs B7                     */
    agc_dp_t  t2;     /* T2       final time, cs B28                          */
    agc_dp_t  t12;    /* T12      transfer time (TMRAD100), cs B28            */
    agc_dp_t conic_t; /* T (30D)  time left by the last conic routine, cs B28 */
    agc_dp_t drcon;   /* DRCON    conic: previous X(T2)ERR B0; precision:
                           *          RCON step, m B29                            */
    agc_dp_t rpre_prev; /* RPRE'    conic: previous X(T2) B0; precision:
                           *          previous RPRE, m B29                        */
    int      nn1a;  /* NN1A     iteration counter 1 (DP B28 integer)        */
    int      nn2;   /* NN2      iteration counter 2 (DP B28 integer)        */
    agc_dp_t phi2;  /* PHI2     -1 perigee / +1 apogee entry, B2            */
    bool     f2rte; /* F2RTE    interpretive switch: set = time critical    */
    /* precision trajectory (PREC100) */
    agc_dp_t rd;      /* RD       final R desired, m B29                      */
    agc_dp_t dt21pr;  /* DT21PR   previous DT21, cs B28                       */
    agc_dp_t dt21;    /* DT21     time step for RTENCK2 (push list), cs B28   */
    agc_dp_t rpre;    /* RPRE     |R(T2)/| (24D), m B29                       */
    agc_dp_t p_rpre;  /* P/RPRE   P/R (26D), B4                               */
    agc_dp_t r_apre;  /* R/APRE   R/A (28D), B6                               */
    agc_dp_t x_t2pre; /* X(T2)PRE precision cot gamma2 (overlays T12), B0     */
    agc_dp_t beta12;  /* BETA12   sign for TIMERAD, B1                        */
    rte_prec_step_t
        prec_step;  /* where PREC100 resumes after INTSTALL           */
    int prec_steps; /* integrations done (bound RTE_PREC_MAX_STEPS)        */
    /* program bookkeeping */
    agc_vec_t rtarg; /* target position kept for P37W (popped vector)        */
    agc_job_id_t
        vnstore; /* VNSTORE  where RTEVN returns                         */
    agc_alarm_code_t
        alarm_code; /* MPAC alarm code for P370ALRM / POODOO          */
} rte_erasable_t;

/* V2T100 push-down list (00D..28D) handed to GAMDV10, DVCALC and XT1LIM.    */
typedef struct rte_v2t
{
    agc_dp_t lambda;   /* 00D  R(T1)/RCON normalised: lambda x 2^(x1-1)      */
    agc_dp_t x_t1pp;   /* 00D  X(T1),, previous precision X(T1), B5 (V2T110) */
    agc_tp_t theta1;   /* 02D  BETA5 LAMBDA - 1, TP B17                      */
    agc_tp_t theta2;   /* 05D  2 R(T1) (LAMBDA - 1), TP m B38                */
    agc_dp_t theta3;   /* 08D  sqrt(MU)/R(T1), B-4                           */
    agc_dp_t x_t1min;  /* 10D  lower bound on X(T1), B5                      */
    agc_dp_t dx_t1max; /* 12D  largest step of X(T1), B5                     */
    agc_dp_t x_t1max;  /* 14D  upper bound on X(T1), B5                      */
    agc_dp_t dx_t1;    /* 16D  iterator step of X(T1), B5                    */
    agc_dp_t beta5;    /* 28D  BETA1 LAMBDA, B(2 - x1)                       */
    int      x1;       /* X1   normalisation difference of R(T1) and RCON    */
} rte_v2t_t;

/* RTEDISP output (stored by the job layer into the shared erasables).     */
typedef struct rte_display
{
    agc_dp_t vpred;   /* VPRED    speed at 400,000 ft, m/cs B7                */
    agc_dp_t t3tot4;  /* T3TOT4   TIG to 400,000 ft, cs B28                   */
    agc_dp_t gammaei; /* GAMMAEI  flight path angle at 400,000 ft, revs B0    */
    agc_vec_t
        delvlvc;     /* DELVLVC  delta V, local vertical coordinates, m/cs B7 */
    agc_dp_t vgdisp; /* VGDISP   |DELVLVC|, m/cs B7                          */
    agc_dp_t lat_spl; /* LAT(SPL) landing site latitude, revs B0              */
    agc_dp_t lng_spl; /* LNG(SPL) landing site longitude, revs B0             */
} rte_display_t;

/* velocity.c (module storage lives here so the arithmetic
 * files link without the job layer).                                       */
rte_erasable_t *rte_state(void);
agc_interp_t   *rte_ip(void);
agc_status_t    rte_post_impulse_velocity(agc_interp_t   *ip,
                                          rte_erasable_t *rs); /* V2T100 */
agc_status_t    rte_dv_calc(agc_interp_t *ip, rte_erasable_t *rs,
                            const rte_v2t_t *pl); /* DVCALC */
agc_status_t rte_xt1lim(agc_interp_t *ip, const rte_erasable_t *rs, agc_dp_t ma,
                        const rte_v2t_t *pl, agc_dp_t *xt1lim); /* XT1LIM */

/* conic.c */
agc_status_t rte_initial_vectors(agc_interp_t *ip, rte_erasable_t *rs,
                                 agc_vec_t ratt, agc_vec_t vatt, agc_dp_t tat,
                                 int x2); /* INVC100 tail */
agc_status_t rte_conic_setup(agc_interp_t   *ip,
                             rte_erasable_t *rs); /* RTE299 tail */
agc_status_t rte_conic_search(agc_interp_t *ip, rte_erasable_t *rs,
                              agc_dp_t rted1); /* RTE317-390 */
agc_status_t rte_time_radius(agc_interp_t *ip, rte_erasable_t *rs,
                             agc_vec_t rvec, agc_vec_t vvec,
                             agc_dp_t rdesired); /* TMRAD100   */
agc_status_t rte_display_values(agc_interp_t *ip, rte_erasable_t *rs,
                                agc_dp_t tig, agc_sp_t range_override,
                                rte_display_t *out); /* RTEDISP */
agc_status_t rte_augekugl(agc_interp_t *ip, agc_dp_t v, agc_dp_t abs_gamma,
                          agc_dp_t *phie, agc_dp_t *te); /* AUGEKUGL   */
agc_status_t rte_entry_side(agc_interp_t   *ip,
                            rte_erasable_t *rs); /* RTE505     */

/* precision.c */
agc_status_t rte_precision_start(rte_erasable_t *rs); /* PREC100    */
agc_status_t rte_precision_run(agc_interp_t *ip, rte_erasable_t *rs,
                               agc_job_id_t resume); /* PREC120..  */
agc_status_t rte_integrate(agc_interp_t *ip, rte_erasable_t *rs, agc_vec_t r,
                           agc_vec_t v, agc_dp_t tet, agc_dp_t tdec1,
                           bool conic, agc_job_id_t resume); /* RTENCK     */

#endif /* RETURN_TO_EARTH_INTERNAL_H */
