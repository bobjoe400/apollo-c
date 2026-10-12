/**********************************************************************
 * FILE NAME: conic.h
 *
 * PURPOSE:
 *   Public interface of the conic subroutines (KEPLER, KEPPREP, LAMBERT,
 *   ITERATOR, TIME-THETA, TIME-RADIUS, APSIDES, PARAM), used by the
 *   orbital integration, the targeting and rendezvous programs and P37.
 *   The AGC routines read and write fixed erasables and the push-down
 *   list; here every routine takes an explicit input struct and fills an
 *   explicit output struct. The conic subroutines are not reentrant in
 *   the AGC (shared push-down list and switches); these C versions keep
 *   all work cells on the stack and are reentrant, but they still mirror
 *   the AGC output switches SOLNSW, INFINFLG, COGAFLAG and APSESW into the
 *   flagwords.
 *
 * AGC SOURCE:
 *   Comanche055 CONIC_SUBROUTINES.agc pp. 1262-1308, ORBITAL_INTEGRATION
 *   .agc pp. 1334-1335 (KEPPREP)
 *   Luminary099 CONIC_SUBROUTINES.agc pp. 1159-1204 (LAMBERT differs:
 *   see src/nav/conic/lambert.c)
 *
 * REQUIRES:
 *   none (self-contained; includes what it uses)
 *
 * GLOBAL VARIABLES:
 *   Name     Type       Scale  Description
 *   -------  ---------  -----  --------------------------------------
 *   UN       agc_vec_t  2^1    agc_erase_un(): LAMBERT's unit normal
 *   GEOMSGN  agc_sp_t   2^0    agc_erase_geomsgn(): LAMBERT's +-.5
 *
 * Scale factors are given as 2^N; where two are given the first is for
 * the earth and the second for the moon as central body (MISSION
 * PROGRAMMING DEFINITION MEMO NO. 10).
 **********************************************************************/
#ifndef AGC_NAV_CONIC_H
#define AGC_NAV_CONIC_H

#include <stdbool.h>

#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"

/* Central body: index register X1 = -2 (earth) or -10D (moon) in the AGC. */
typedef enum agc_conic_body
{
    AGC_CONIC_EARTH = 0, /* X1 = -2, MUTABLE+2 */
    AGC_CONIC_MOON  = 1  /* X1 = -10D, MUTABLE+10 */
} agc_conic_body_t;

#define AGC_CONIC_KEPLER_ITERCTR       20   /* SSP ITERCTR 20D at KEPLERN */
#define AGC_CONIC_LAMBERT_ITERCTR      20   /* SSP ITERCTR 20D, SETITCTR  */
#define AGC_CONIC_LAMBERT_ITERCTR_AVEG 5    /* LM SETITCTR with AVEGFLAG  */
#define AGC_CONIC_ALARM_ABTCONIC       0607 /* ABTCONIC: POODOO 00607     */

/* ---- KEPLER (KEPLERN), pp. 1263-1265 --------------------------------- */
typedef struct agc_kepler_in
{
    agc_vec_t        rrect;   /* RRECT initial position, 2^29 / 2^27 m   */
    agc_vec_t        vrect;   /* VRECT initial velocity, 2^7 / 2^5 m/cs  */
    agc_dp_t         tau;     /* TAU. transfer time (+-), 2^28 cs        */
    agc_dp_t         xkepnew; /* XKEPNEW guess of x, 2^17 / 2^16 sqrt(m) */
    agc_dp_t         tc;      /* TC previous time, 2^28 cs               */
    agc_dp_t         xprev;   /* XPREV previous x, 2^17 / 2^16 sqrt(m)   */
    agc_conic_body_t body;    /* central body (X1)                       */
} agc_kepler_in_t;

typedef struct agc_kepler_out
{
    agc_vec_t rcv;        /* RCV terminal position, 2^29 / 2^27 m       */
    agc_vec_t vcv;        /* VCV terminal velocity, 2^7 / 2^5 m/cs      */
    agc_dp_t  tc;         /* TC time converged to, 2^28 cs              */
    agc_dp_t  xprev;      /* XPREV x converged to, 2^17 / 2^16 sqrt(m)  */
    agc_vec_t urrect;     /* URRECT unit initial position, 2^1          */
    agc_dp_t  r1;         /* R1 |RRECT|, 2^29 / 2^27 m                  */
    agc_dp_t  alpha;      /* ALPHA 1/semimajor axis, 2^-22 / 2^-20 1/m  */
    agc_dp_t  tau;        /* TAU. after the period modulo, 2^28 cs      */
    agc_dp_t  tmodulo;    /* input TAU. - reduced TAU., 2^28 cs (the 1969 code no longer stores TMODULO; it is reconstructed for callers) */
    int       iterations; /* KEPLOOP passes used, <= 20 (unscaled)      */
} agc_kepler_out_t;

/*
 * KEPLER: update a state along a conic by time TAU.
 * Returns AGC_STATUS_OK on EPSILONT convergence (or DELX = 0).
 * Returns AGC_STATUS_NO_CONVERGE when ITERCTR ran out: the AGC treats that
 * as converged (BHIZ KEPCONVG) and sets no flag or alarm, and all outputs
 * are filled exactly as the AGC leaves them, so callers that want the AGC's
 * behaviour accept this status. The same status, with *out not filled,
 * when PERIODCH reaches its C pass bound (AGC_CONIC_PERIOD_MAX_PASSES).
 * Returns AGC_STATUS_OVERFLOW on the TIMEOVFL path when DELX halves to zero:
 * the AGC then returns through KEPRTN WITHOUT computing RCV/VCV (they keep
 * their previous contents, which after RECTIFY are RRECT/VRECT); here
 * out->rcv/vcv = in->rrect/vrect, out->tc = TC as last set by the
 * iteration and out->xprev = in->xprev.
 */
agc_status_t agc_conic_kepler(agc_interp_t *ip, const agc_kepler_in_t *in, agc_kepler_out_t *out);

/* KEPPREP: first guess XKEPNEW (2^17 / 2^16) from in->rrect, vrect, tau,
 * tc and xprev (= XKEP). in->xkepnew is ignored.                          */
void agc_conic_kepprep(agc_interp_t *ip, const agc_kepler_in_t *in, agc_dp_t *xkepnew);

/* ---- LAMBERT, pp. 1266-1268 ------------------------------------------ */
typedef struct agc_lambert_in
{
    agc_vec_t        r1vec;    /* R1VEC initial position, 2^29 / 2^27 m  */
    agc_vec_t        r2vec;    /* R2VEC target position, 2^29 / 2^27 m   */
    agc_dp_t         tdesired; /* TDESIRED transfer time, 2^28 cs        */
    agc_sp_t         geomsgn;  /* GEOMSGN SP 2^0: +.5 below 180 deg, -.5 above */
    bool             guessw;   /* GUESSW set: no COGA guess available    */
    agc_dp_t         coga;     /* COGA guess, cot flight-path angle, 2^5 */
    bool             normsw;   /* NORMSW set: UN is an input             */
    agc_vec_t        un;       /* UN unit normal (if normsw), 2^1        */
    agc_sp_t         vtargtag; /* VTARGTAG 0: also VTARGET (unscaled)    */
    agc_sp_t         iterctr;  /* LM only: ITERCTR from SETITCTR, 20 or 5 with AVEGFLAG (count, unscaled); the CSM sets 20 itself */
    agc_conic_body_t body;     /* central body (X1)                      */
} agc_lambert_in_t;

typedef struct agc_lambert_out
{
    agc_vec_t vvec;       /* VVEC required initial velocity, 2^7 / 2^5 */
    agc_vec_t vtarget;    /* VTARGET velocity at R2VEC (vtargtag == 0), 2^7 / 2^5 m/cs */
    bool      solnsw;     /* SOLNSW set: no acceptable solution        */
    agc_dp_t  snth;       /* SNTH sin transfer angle, 2^1              */
    agc_dp_t  csth;       /* CSTH cos transfer angle, 2^1              */
    agc_dp_t  one_m_csth; /* 1-CSTH, 2^2                               */
    agc_dp_t  coga;       /* COGA, 2^5                                 */
    agc_dp_t  p;          /* P semilatus rectum / R1, 2^4              */
    agc_dp_t  r1a;        /* R1A R1 / semimajor axis, 2^6              */
    agc_dp_t  r1;         /* R1, 2^29 / 2^27 m                         */
    agc_vec_t ur1;        /* UR1, 2^1                                  */
    agc_vec_t u2;         /* U2, 2^1                                   */
    agc_vec_t un;         /* UN, 2^1                                   */
} agc_lambert_out_t;

/* LAMBERT. AGC_STATUS_OK when SOLNSW is clear. AGC_STATUS_NO_CONVERGE when
 * SOLNSW is set (transfer near 0/360 deg, or iteration did not reach the
 * SUFFCHEK tolerance); the flag SOLNSW is also set. On the LM, VVEC (and
 * VTARGET) are still computed when SUFFCHEK fails (Luminary 099); on the
 * CSM, and for 360LAMB, the AGC leaves them unchanged and here they are
 * zero. Debris is filled in every case.                                  */
agc_status_t agc_conic_lambert(agc_interp_t *ip, const agc_lambert_in_t *in, agc_lambert_out_t *out);

/* ---- ITERATOR, pp. 1285-1286 (used by LAMBERT); the variables are the
 * words as stored (signs may disagree after a DAS), see interp_words.h  */
typedef struct agc_iterator
{
    agc_dp_words_t indep;    /* INDEP independent variable (COGA, 2^5)      */
    agc_dp_words_t dep;      /* DEP dependent variable (T, 2^28 cs)         */
    agc_dp_words_t deprev;   /* DEPREV previous DEP (TPREV, 2^28 cs)        */
    agc_dp_words_t deldep;   /* DELDEP error in DEP (TERRLAMB, 2^28 cs)     */
    agc_dp_words_t delindep; /* DELINDEP in/out step (DCOGA, 2^5)           */
    agc_dp_words_t max;      /* MAX upper bound (COGAMAX, 2^5)              */
    agc_dp_words_t min;      /* MIN lower bound (COGAMIN, 2^5)              */
    agc_sp_t       tweekit;  /* TWEEKIT (40D) first-step fraction of MAX-MIN, SP 2^0; DMP reads 40D,41D as DP and 41D is S2 = 1 (set by GETX) */
    bool           slopesw;  /* SLOPESW set: first pass (cleared here)      */
    bool           ordersw;  /* ORDERSW set: 2nd order (never in Apollo 11) */
} agc_iterator_t;

/* ITERATOR: new DELINDEP (and MAX/MIN, SLOPESW) in *it. */
agc_status_t agc_conic_iterator(agc_interp_t *ip, agc_iterator_t *it);

/* ---- TIMETHET / TIMERAD / APSIDES / PARAM ----------------------------- */
typedef struct agc_conic_time_out
{
    agc_dp_t  t;        /* T (30D) transfer time, 2^28 cs              */
    agc_vec_t rnew;     /* terminal position (0D) if !rvsw, 2^29/2^27  */
    agc_vec_t vnew;     /* terminal velocity (MPAC) if !rvsw, 2^7/2^5  */
    bool      infinflg; /* INFINFLG: closure through infinity          */
    bool      cogaflag; /* COGAFLAG: COGA overflow (restriction 1)     */
    bool      apsesw;   /* APSESW (TIMERAD): RDESIRED outside apsides  */
    bool      solnsw;   /* SOLNSW (TIMERAD): orbit too nearly circular */
    agc_dp_t  r1;       /* R1, 2^29 / 2^27 m                           */
    agc_dp_t  r1a;      /* R1A, 2^6                                    */
    agc_dp_t  p;        /* P, 2^4                                      */
    agc_dp_t  coga;     /* COGA, 2^5                                   */
    agc_dp_t  csth;     /* CSTH, 2^1 (TIMERAD output)                  */
    agc_dp_t  snth;     /* SNTH, 2^1 (TIMERAD output)                  */
    agc_vec_t ur1;      /* UR1, 2^1                                    */
    agc_vec_t u2;       /* U2, 2^1                                     */
    agc_vec_t un;       /* UN, 2^1                                     */
} agc_conic_time_out_t;

typedef struct agc_timetheta_in
{
    agc_vec_t        rvec; /* RVEC, 2^29 / 2^27 m                         */
    agc_vec_t        vvec; /* VVEC, 2^7 / 2^5 m/cs                        */
    agc_dp_t         snth; /* SNTH sin of true-anomaly difference, 2^1    */
    agc_dp_t         csth; /* CSTH cos of true-anomaly difference, 2^1    */
    bool             rvsw; /* RVSW set: time only                         */
    agc_conic_body_t body; /* central body (X1)                           */
} agc_timetheta_in_t;

typedef struct agc_timerad_in
{
    agc_vec_t        rvec;     /* RVEC, 2^29 / 2^27 m                     */
    agc_vec_t        vvec;     /* VVEC, 2^7 / 2^5 m/cs                    */
    agc_dp_t         rdesired; /* RDESIRED, 2^29 / 2^27 m                 */
    agc_sp_t         sgnrdot;  /* SGNRDOT SP 2^0: +-.5, sign of RDOT      */
    bool             rvsw;     /* RVSW set: time only                     */
    agc_conic_body_t body;     /* central body (X1)                       */
} agc_timerad_in_t;

/* TIMETHET / TIMERAD. On COGAFLAG/INFINFLG (and for TIMERAD SOLNSW) the AGC
 * goes to ABTCONIC = POODOO 00607 and never returns; here agc_poodoo(0607)
 * is called, the flags are reported in *out and AGC_STATUS_POODOO (or
 * POODOO's AGC_STATUS_BAILOUT) is returned.                               */
agc_status_t agc_conic_timethet(agc_interp_t *ip, const agc_timetheta_in_t *in, agc_conic_time_out_t *out);
agc_status_t agc_conic_timerad(agc_interp_t *ip, const agc_timerad_in_t *in, agc_conic_time_out_t *out);

typedef struct agc_apsides_out
{
    agc_dp_t  rapo;    /* MPAC radius of apocenter, 2^29 / 2^27 m (LDPOSMAX if beyond scaling or hyperbolic) */
    agc_dp_t  rperi;   /* 0D radius of pericenter, 2^29 / 2^27 m       */
    agc_dp_t  ecc;     /* ECC eccentricity, 2^3                        */
    agc_dp_t  r1;      /* R1, 2^29 / 2^27 m                            */
    agc_dp_t  r1a;     /* R1A, 2^6                                     */
    agc_dp_t  p;       /* P, 2^4                                       */
    agc_dp_t  coga;    /* COGA, 2^5                                    */
    agc_dp_t  magvec2; /* MAGVEC2 |VVEC|, 2^7 / 2^5 m/cs               */
    agc_vec_t ur1;     /* UR1, 2^1                                     */
    agc_vec_t u2;      /* U2, 2^1                                      */
    agc_vec_t un;      /* UN, 2^1                                      */
} agc_apsides_out_t;

void agc_conic_apsides(agc_interp_t *ip, agc_vec_t rvec, agc_vec_t vvec, agc_conic_body_t body, agc_apsides_out_t *out);

/* PARAM (also called directly by P37). Leaves OVFIND set in *ip if COGA
 * overflowed (the caller's BOV), exactly as the AGC does.                 */
typedef struct agc_conic_param_out
{
    agc_dp_t  p;       /* P, 2^4                                       */
    agc_dp_t  r1a;     /* R1A, 2^6                                     */
    agc_dp_t  coga;    /* COGA, 2^5                                    */
    agc_dp_t  r1;      /* R1, 2^29 / 2^27 m                            */
    agc_dp_t  magvec2; /* MAGVEC2 |V|, 2^7 / 2^5 m/cs                  */
    agc_dp_t  sngamma; /* sin(angle R,V) (36D), 2^1                    */
    agc_vec_t ur1;     /* UR1, 2^1                                     */
    agc_vec_t u2;      /* U2, 2^1                                      */
    agc_vec_t un;      /* UN, 2^1                                      */
} agc_conic_param_out_t;

void agc_conic_param(agc_interp_t *ip, agc_vec_t rvec, agc_vec_t vvec, agc_conic_body_t body, agc_conic_param_out_t *out);

/* Body from RTX1 (the integrator's X1): -2 earth, -10 moon.             */
agc_conic_body_t agc_conic_body_of_x1(int x1);

/* UN and GEOMSGN of the last LAMBERT (erasables read by LM RASTEER).    */
agc_vec_t agc_erase_un(void);      /* UN, unit normal, 2^1              */
agc_sp_t  agc_erase_geomsgn(void); /* GEOMSGN, SP +.5 short / -.5 long  */

#endif /* AGC_NAV_CONIC_H */
