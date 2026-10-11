/*
 * agc_nav.h -- navigation interfaces.
 *
 * Section "conic" (agent nav-conic): CONIC_SUBROUTINES.agc, Comanche 055
 * pp. 1262-1308 / Luminary 099 pp. 1159-1204, plus KEPPREP
 * (ORBITAL_INTEGRATION.agc p. 1334) and the conic path of CSMCONIC/LEMCONIC
 * (INTEGRATION_INITIALIZATION.agc pp. 1318-1322).
 *
 * The AGC routines read and write fixed erasables and the push-down list;
 * here every routine takes an explicit input struct and fills an explicit
 * output struct. Scale factors are given as 2^N; where two are given the
 * first is for the earth and the second for the moon as central body
 * (MISSION PROGRAMMING DEFINITION MEMO NO. 10).
 *
 * The conic subroutines are not reentrant in the AGC (shared push-down
 * list and switches); these C versions keep all work cells on the stack and
 * are reentrant, but they still mirror the AGC output switches SOLNSW,
 * INFINFLG, COGAFLAG and APSESW into the flagwords.
 */
#ifndef AGC_NAV_H
#define AGC_NAV_H

#include <stdbool.h>

#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"

/* ======================================================================= */
/* Section "conic"                                                         */
/* ======================================================================= */

/* Central body: index register X1 = -2 (earth) or -10D (moon) in the AGC. */
typedef enum agc_conic_body
{
    AGC_CONIC_EARTH = 0,
    AGC_CONIC_MOON  = 1
} agc_conic_body_t;

#define AGC_CONIC_KEPLER_ITERCTR       20 /* SSP ITERCTR 20D at KEPLERN         */
#define AGC_CONIC_LAMBERT_ITERCTR      20 /* SSP ITERCTR 20D (CSM) / SETITCTR   */
#define AGC_CONIC_LAMBERT_ITERCTR_AVEG 5 /* LM SETITCTR when AVEGFLAG set    */
#define AGC_CONIC_ALARM_ABTCONIC       0607 /* ABTCONIC: POODOO 00607 (octal)    */

/* ---- KEPLER (KEPLERN), p. 1263-1265 ---------------------------------- */
typedef struct agc_kepler_in
{
    agc_vec_t        rrect; /* RRECT  initial position, m,      2^29 / 2^27     */
    agc_vec_t        vrect; /* VRECT  initial velocity, m/cs,   2^7  / 2^5      */
    agc_dp_t         tau;   /* TAU.   desired transfer time, cs, 2^28 (+ or -)  */
    agc_dp_t         xkepnew; /* XKEPNEW guess of x, sqrt(m),     2^17 / 2^16     */
    agc_dp_t         tc;    /* TC     previous time, cs,        2^28            */
    agc_dp_t         xprev; /* XPREV  previous x, sqrt(m),      2^17 / 2^16     */
    agc_conic_body_t body;
} agc_kepler_in_t;

typedef struct agc_kepler_out
{
    agc_vec_t rcv;   /* RCV    terminal position, m,     2^29 / 2^27     */
    agc_vec_t vcv;   /* VCV    terminal velocity, m/cs,  2^7  / 2^5      */
    agc_dp_t  tc;    /* TC     time converged to, cs,    2^28            */
    agc_dp_t  xprev; /* XPREV  x converged to, sqrt(m),  2^17 / 2^16     */
    /* debris of use */
    agc_vec_t urrect;     /* URRECT unit initial position,    2^1             */
    agc_dp_t  r1;         /* R1     |RRECT|, m,               2^29 / 2^27     */
    agc_dp_t  alpha;      /* ALPHA  1/semimajor axis, 1/m,    2^-22 / 2^-20   */
    agc_dp_t  tau;        /* TAU.   after period modulo, cs,  2^28            */
    agc_dp_t  tmodulo;    /* TMODULO = input TAU. - reduced TAU., cs, 2^28.
                           * (The 1969 code no longer stores TMODULO; it is
                           * reconstructed here for callers.)                */
    int iterations; /* KEPLOOP passes used (<= AGC_CONIC_KEPLER_ITERCTR)*/
} agc_kepler_out_t;

/*
 * KEPLER: update a state along a conic by time TAU.
 * Returns AGC_STATUS_OK on EPSILONT convergence (or DELX = 0).
 * Returns AGC_STATUS_NO_CONVERGE when ITERCTR ran out: the AGC treats that
 * as converged (BHIZ KEPCONVG) and sets no flag or alarm, and all outputs
 * are filled exactly as the AGC leaves them, so callers that want the AGC's
 * behaviour accept this status.
 * Returns AGC_STATUS_OVERFLOW on the TIMEOVFL path when DELX halves to zero:
 * the AGC then returns through KEPRTN WITHOUT computing RCV/VCV (they keep
 * their previous contents, which after RECTIFY are RRECT/VRECT); here
 * out->rcv/vcv = in->rrect/vrect, out->tc = TC as last set by the
 * iteration and out->xprev = in->xprev.
 */
agc_status_t agc_conic_kepler(agc_interp_t *ip, const agc_kepler_in_t *in,
                              agc_kepler_out_t *out);

/* KEPPREP: first guess XKEPNEW (2^17 / 2^16) from in->rrect, vrect, tau,
 * tc and xprev (= XKEP). in->xkepnew is ignored.                          */
agc_status_t agc_conic_kepprep(agc_interp_t *ip, const agc_kepler_in_t *in,
                               agc_dp_t *xkepnew);

/* RVCON conic path: RECTIFY (TC = XKEP = 0), KEPPREP, KEPLERN, TET += TC.
 * sv->r/v/t/moon are the state at TET; out gets RCV, VCV and the new TET.
 * kout (may be NULL) receives the Kepler outputs.                        */
agc_status_t agc_conic_rvcon(agc_interp_t *ip, const agc_state_vector_t *sv,
                             agc_dp_t tdec, agc_state_vector_t *out,
                             agc_kepler_out_t *kout);

/* CSMCONIC / LEMCONIC: conic extrapolation of the CSM / LM state vector to
 * TDEC1 (cs, 2^28). The INTSTALL lock, A-memory and STATEFLG handling are
 * the integration package's (nav-integ); this is the arithmetic only.     */
agc_status_t agc_conic_csmconic(agc_interp_t *ip, agc_dp_t tdec1,
                                agc_state_vector_t *out);
agc_status_t agc_conic_lemconic(agc_interp_t *ip, agc_dp_t tdec1,
                                agc_state_vector_t *out);

/* ---- LAMBERT, p. 1266-1268 ------------------------------------------- */
typedef struct agc_lambert_in
{
    agc_vec_t r1vec;    /* R1VEC  initial position, m,      2^29 / 2^27     */
    agc_vec_t r2vec;    /* R2VEC  target position, m,       2^29 / 2^27     */
    agc_dp_t  tdesired; /* TDESIRED transfer time, cs,      2^28            */
    agc_sp_t  geomsgn;  /* GEOMSGN SP +.5 (< 180 deg) / -.5 (> 180 deg)     */
    bool      guessw;   /* GUESSW set: no COGA guess available             */
    agc_dp_t  coga;     /* COGA   guess, cot flight-path angle, 2^5        */
    bool      normsw;   /* NORMSW set: UN is an input                      */
    agc_vec_t un;       /* UN     unit normal (if normsw), 2^1             */
    agc_sp_t  vtargtag; /* VTARGTAG 0: also compute VTARGET                */
    agc_sp_t  iterctr;  /* LM only: ITERCTR as set by SETITCTR (20, or 5
                         * when AVEGFLAG). The CSM sets 20 itself.        */
    agc_conic_body_t body;
} agc_lambert_in_t;

typedef struct agc_lambert_out
{
    agc_vec_t vvec;    /* VVEC   required initial velocity, 2^7 / 2^5     */
    agc_vec_t vtarget; /* VTARGET velocity at R2VEC (vtargtag == 0)        */
    bool      solnsw;  /* SOLNSW set: no acceptable solution               */
    /* debris of use */
    agc_dp_t  snth;       /* SNTH  sin transfer angle, 2^1                    */
    agc_dp_t  csth;       /* CSTH  cos transfer angle, 2^1                    */
    agc_dp_t  one_m_csth; /* 1-CSTH, 2^2                                      */
    agc_dp_t  coga;       /* COGA  2^5                                        */
    agc_dp_t  p;          /* P     semilatus rectum / R1, 2^4                 */
    agc_dp_t  r1a;        /* R1A   R1 / semimajor axis, 2^6                   */
    agc_dp_t  r1;         /* R1    m, 2^29 / 2^27                             */
    agc_vec_t ur1;        /* UR1   2^1                                        */
    agc_vec_t u2;         /* U2    2^1                                        */
    agc_vec_t un;         /* UN    2^1                                        */
} agc_lambert_out_t;

/* LAMBERT. AGC_STATUS_OK when SOLNSW is clear. AGC_STATUS_NO_CONVERGE when
 * SOLNSW is set (transfer near 0/360 deg, or iteration did not reach the
 * SUFFCHEK tolerance); the flag SOLNSW is also set. On the LM, VVEC (and
 * VTARGET) are still computed when SUFFCHEK fails (Luminary 099); on the
 * CSM, and for 360LAMB, the AGC leaves them unchanged and here they are
 * zero. Debris is filled in every case.                                  */
agc_status_t agc_conic_lambert(agc_interp_t *ip, const agc_lambert_in_t *in,
                               agc_lambert_out_t *out);

/* ---- ITERATOR, p. 1285-1286 (used by LAMBERT) ------------------------- */
typedef struct agc_iterator
{
    agc_dp_t indep;    /* INDEP    independent variable (LAMBERT: COGA)    */
    agc_dp_t dep;      /* DEP      dependent variable   (LAMBERT: T)       */
    agc_dp_t deprev;   /* DEPREV   previous DEP          (TPREV)            */
    agc_dp_t deldep;   /* DELDEP   error in DEP          (TERRLAMB)         */
    agc_dp_t delindep; /* DELINDEP in/out step           (DCOGA)            */
    agc_dp_t max;     /* MAX      upper bound           (COGAMAX)          */
    agc_dp_t min;     /* MIN      lower bound           (COGAMIN)          */
    agc_sp_t tweekit; /* TWEEKIT (40D = S1) first-step fraction of
                       * MAX-MIN, SP; DMP reads 40D,41D as DP and 41D is
                       * S2 = 1 (set by GETX), so the DP is (tweekit, 1) */
    bool slopesw;     /* SLOPESW  set: first pass (cleared by ITERATOR)   */
    bool ordersw;     /* ORDERSW  set: 2nd-order (never set in Apollo 11) */
} agc_iterator_t;

/* ITERATOR: new DELINDEP (and MAX/MIN, SLOPESW) in *it. */
agc_status_t agc_conic_iterator(agc_interp_t *ip, agc_iterator_t *it);

/* ---- TIMETHET / TIMERAD / APSIDES / PARAM ----------------------------- */
typedef struct agc_conic_time_out
{
    agc_dp_t  t;        /* T (30D) transfer time, cs, 2^28                  */
    agc_vec_t rnew;     /* terminal position (0D) if !rvsw, 2^29 / 2^27     */
    agc_vec_t vnew;     /* terminal velocity (MPAC) if !rvsw, 2^7 / 2^5     */
    bool      infinflg; /* INFINFLG: closure through infinity               */
    bool      cogaflag; /* COGAFLAG: COGA overflow (restriction 1)          */
    bool      apsesw;   /* APSESW (TIMERAD): RDESIRED outside apsides       */
    bool      solnsw;   /* SOLNSW (TIMERAD): orbit too nearly circular      */
    /* debris of use */
    agc_dp_t  r1;   /* R1  2^29 / 2^27                                  */
    agc_dp_t  r1a;  /* R1A 2^6                                          */
    agc_dp_t  p;    /* P   2^4                                          */
    agc_dp_t  coga; /* COGA 2^5                                         */
    agc_dp_t  csth; /* CSTH 2^1 (TIMERAD output)                        */
    agc_dp_t  snth; /* SNTH 2^1 (TIMERAD output)                        */
    agc_vec_t ur1;  /* UR1 2^1                                          */
    agc_vec_t u2;   /* U2  2^1                                          */
    agc_vec_t un;   /* UN  2^1                                          */
} agc_conic_time_out_t;

typedef struct agc_timetheta_in
{
    agc_vec_t        rvec; /* RVEC 2^29 / 2^27 m                               */
    agc_vec_t        vvec; /* VVEC 2^7 / 2^5 m/cs                              */
    agc_dp_t         snth; /* SNTH sin of true-anomaly difference, 2^1         */
    agc_dp_t         csth; /* CSTH cos of true-anomaly difference, 2^1         */
    bool             rvsw; /* RVSW set: time only                              */
    agc_conic_body_t body;
} agc_timetheta_in_t;

typedef struct agc_timerad_in
{
    agc_vec_t        rvec; /* RVEC 2^29 / 2^27 m                               */
    agc_vec_t        vvec; /* VVEC 2^7 / 2^5 m/cs                              */
    agc_dp_t         rdesired; /* RDESIRED 2^29 / 2^27 m                           */
    agc_sp_t         sgnrdot; /* SGNRDOT SP +.5 / -.5 sign of radial velocity     */
    bool             rvsw;    /* RVSW set: time only                              */
    agc_conic_body_t body;
} agc_timerad_in_t;

/* TIMETHET / TIMERAD. On COGAFLAG/INFINFLG (and for TIMERAD SOLNSW) the AGC
 * goes to ABTCONIC = POODOO 00607 and never returns; here agc_poodoo(0607)
 * is called, the flags are reported in *out and AGC_STATUS_POODOO is
 * returned.                                                                */
agc_status_t agc_conic_timethet(agc_interp_t *ip, const agc_timetheta_in_t *in,
                                agc_conic_time_out_t *out);
agc_status_t agc_conic_timerad(agc_interp_t *ip, const agc_timerad_in_t *in,
                               agc_conic_time_out_t *out);

typedef struct agc_apsides_out
{
    agc_dp_t rapo;  /* MPAC radius of apocenter, 2^29 / 2^27 (DPPOSMAX
                     * if beyond scaling or hyperbolic)               */
    agc_dp_t rperi; /* 0D   radius of pericenter, 2^29 / 2^27          */
    agc_dp_t ecc;   /* ECC  eccentricity, 2^3                           */
    /* debris of use */
    agc_dp_t  r1;      /* R1 2^29 / 2^27                                   */
    agc_dp_t  r1a;     /* R1A 2^6                                          */
    agc_dp_t  p;       /* P 2^4                                            */
    agc_dp_t  coga;    /* COGA 2^5                                         */
    agc_dp_t  magvec2; /* MAGVEC2 |VVEC| 2^7 / 2^5                         */
    agc_vec_t ur1, u2, un; /* 2^1                                            */
} agc_apsides_out_t;

agc_status_t agc_conic_apsides(agc_interp_t *ip, agc_vec_t rvec, agc_vec_t vvec,
                               agc_conic_body_t body, agc_apsides_out_t *out);

/* PARAM (also called directly by P37). Leaves OVFIND set in *ip if COGA
 * overflowed (the caller's BOV), exactly as the AGC does.                 */
typedef struct agc_conic_param_out
{
    agc_dp_t  p;       /* P   2^4                                          */
    agc_dp_t  r1a;     /* R1A 2^6                                          */
    agc_dp_t  coga;    /* COGA 2^5                                         */
    agc_dp_t  r1;      /* R1  2^29 / 2^27                                  */
    agc_dp_t  magvec2; /* MAGVEC2 |V| 2^7 / 2^5                            */
    agc_dp_t  sngamma; /* sin(angle R,V), 2^1 (36D)                        */
    agc_vec_t ur1, u2, un; /* 2^1                                            */
} agc_conic_param_out_t;

agc_status_t agc_conic_param(agc_interp_t *ip, agc_vec_t rvec, agc_vec_t vvec,
                             agc_conic_body_t body, agc_conic_param_out_t *out);

/* Body from RTX1 (the integrator's X1): -2 earth, -10 moon.             */
agc_conic_body_t agc_conic_body_of_x1(int x1);

/* UN and GEOMSGN of the last LAMBERT (erasables read by LM RASTEER).    */
agc_vec_t agc_erase_un(void);      /* UN, unit normal, 2^1              */
agc_sp_t  agc_erase_geomsgn(void); /* GEOMSGN, SP +.5 short / -.5 long  */

#endif /* AGC_NAV_H */
