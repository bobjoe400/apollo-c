/*
 * agc_integ.h -- orbital integration (agent nav-integ).
 *
 * Translated from INTEGRATION_INITIALIZATION.agc (Comanche055 pp. 1309-1333,
 * Luminary099 pp. 1203-1226) and ORBITAL_INTEGRATION.agc (Comanche055
 * pp. 1334-1354, Luminary099 pp. 1227-1248), plus STATEUP/THISVINT from
 * FRESH_START_AND_RESTART.agc (the body of the STATEINT job).
 *
 * The AGC integrates by Encke's method: the state is a conic (RCV, VCV,
 * found by KEPLER from the rectified state RRECT, VRECT at TET-TC) plus a
 * deviation (TDELTAV, TNUV) driven by the disturbing accelerations (earth
 * J2..J4, lunar J2/J3/J22/C31, and the moon/earth/sun third-body terms
 * outside 800 km altitude) through a Nystrom step.  The W (error
 * transition) matrix is integrated column by column along the same path.
 *
 * Scale notation: "E/M" = earth / moon as primary body (PBODY, MOONFLAG).
 *   position  m     E 2^29  M 2^27     deviation TDELTAV  m    E 2^22  M 2^18
 *   velocity  m/cs  E 2^7   M 2^5      deviation TNUV     m/cs E 2^3   M 2^-1
 *   time      cs    2^28               XKEP sqrt(m) E 2^17 M 2^16
 *   DT/2, H   cs    2^19 (half step; the bits equal the full step at 2^20)
 * RATT/VATT are always at the earth scales (2^29 m, 2^7 m/cs).
 *
 * The single set of AGC integration erasables is one shared context
 * (agc_integ_shared_ctx()); its use is serialised by the INTSTALL lock.
 */
#ifndef AGC_INTEG_H
#define AGC_INTEG_H

#include <stdbool.h>

#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/conic.h"

/* ---- limits --------------------------------------------------------- */
#define AGC_INTEG_AREAS_CSM 3 /* INTSTALL, ERASTAL1, ERASTAL2 (CM)     */
#define AGC_INTEG_AREAS_LM  1 /* INTSTALL only (LM)                    */
#define AGC_INTEG_AREAS_MAX 3

/* Bound on the TESTLOOP state machine (DEVIATION: the AGC has none).  A
 * step is 2 passes (TESTLOOP, TIMESTEP), 4 with W (WMATRIX, RELOAD).
 * The shortest step (NORFINAL at the earth's surface: .3 sqrt(R^3/MU) =
 * 24192 cs) over the whole TDEC range, 2^28 cs, is 11096 steps = 44384
 * passes with W.                                                         */
#define AGC_INTEG_MAX_PASSES 60000
#define AGC_INTEG_W_COLUMNS  9 /* W columns (6 for 6x6, 9 for 9x9)      */
#define AGC_INTEG_VECTAB     6 /* VECTAB: 3 stages x (R, R - RPQ)       */

/* Octal codes (kept as in the listing). */
#define AGC_INTEG_ALARM_INT_ABRT \
    ((agc_alarm_code_t) 00430)                            /* POODOO: INT-ABRT */
#define AGC_INTEG_ALARM_WMATEND \
    ((agc_alarm_code_t) 00421)                            /* W overflow       */
#define AGC_INTEG_ALARM_NOTIME ((agc_alarm_code_t) 01703) /* MIDTOAV too late */

/* ---- the 42-word state vector block (RRECT .. XKEP) -------------------
 * Same layout for the temporary set (RRECT...) and the permanent sets
 * (RRECTCSM..XKEPCSM, RRECTLEM..XKEPLEM).                               */
typedef struct agc_integ_sv
{
    agc_vec_t rrect;  /* RRECT   rectified position, m,     E 2^29 M 2^27 */
    agc_vec_t vrect;  /* VRECT   rectified velocity, m/cs,  E 2^7  M 2^5  */
    agc_dp_t  tet;    /* TET     time of the state, cs,     2^28          */
    agc_vec_t deltav; /* TDELTAV position deviation, m,     E 2^22 M 2^18 */
    agc_vec_t nuv;    /* TNUV    velocity deviation, m/cs,  E 2^3  M 2^-1 */
    agc_vec_t rcv;    /* RCV     conic position, m,         E 2^29 M 2^27 */
    agc_vec_t vcv;    /* VCV     conic velocity, m/cs,      E 2^7  M 2^5  */
    agc_dp_t  tc;     /* TC      time since rectification, cs, 2^28       */
    agc_dp_t  xkep;   /* XKEP    Kepler x, sqrt(m),         E 2^17 M 2^16 */
} agc_integ_sv_t;

/* ---- output left in the push-down list (section 3.0, p. 1311) -------- */
typedef struct agc_integ_out
{
    bool      valid; /* RECTOUT ran (INTEXIT reached without it leaves the push list untouched: QUITFLAG, backward request) */
    agc_vec_t ratt;  /* 0D  RATT  position, m, 2^29 (both bodies)        */
    agc_vec_t vatt;  /* 6D  VATT  velocity, m/cs, 2^7 (both bodies)      */
    agc_dp_t  tat;   /* 12D TAT   time, cs, 2^28                         */
    agc_vec_t ratt1; /* 14D RATT1 position, m, E 2^29 M 2^27             */
    agc_vec_t vatt1; /* 20D VATT1 velocity, m/cs, E 2^7 M 2^5            */
    agc_dp_t  mu_p;  /* 26D MU(P) m^3/cs^2, E 2^36 M 2^30                */
    int       x1;    /* X1  MUTABLE entry: -2 earth, -10 moon            */
    int       x2;    /* X2  coordinate origin: 0 earth, 2 moon           */
} agc_integ_out_t;

/* ---- the integration erasables ------------------------------------- */
typedef struct agc_integ_ctx
{
    agc_integ_sv_t  sv;                       /* temporary state vector RRECT..XKEP      */
    int             pbody;                    /* PBODY (X2): 0 earth, 2 moon             */
    agc_dp_t        tdec1;                    /* TDEC1 (PD 32D) time to integrate to, cs 2^28 */
    agc_dp_t        tdec;                     /* TDEC  working target time, cs 2^28      */
    agc_dp_t        tau;                      /* TAU.  Kepler transfer time, cs 2^28     */
    agc_dp_t        dt2;                      /* DT/2  half step, cs 2^19                */
    agc_dp_t        h;                        /* H     time into the step, cs 2^19       */
    agc_dp_t        dtmax;                    /* 12D   step limit (DT/2 format)          */
    agc_dp_t        rc;                       /* 10D   |RCV|, m, E 2^29 M 2^27           */
    int             stage;                    /* DIFEQCNT: Nystrom stage 0, 1, 2         */
    int             colreg;                   /* COLREG: W column being integrated 0..8  */
    agc_vec_t       alphav;                   /* ALPHAV deviation / unit vector          */
    agc_dp_t        alpham;                   /* ALPHAM |ALPHAV| / |R|                   */
    agc_vec_t       betav;                    /* BETAV  R or perturbing-body vector      */
    agc_dp_t        betam;                    /* BETAM  |BETAV| or MU                    */
    agc_vec_t phiv, psiv;                     /* PHIV PSIV Nystrom sums                  */
    agc_vec_t       fv;                       /* FV     disturbing acceleration, m/cs^2 E 2^-16 M 2^-20 (= YV/H^2 scale) */
    agc_vec_t yv, zv;                         /* YV ZV  deviation pos/vel at step start  */
    agc_vec_t       vectab[AGC_INTEG_VECTAB]; /* VECTAB: [2k] R at stage k (E 2^29 M 2^27), [2k+1] R - RPQ, 2^29 (W use) */
    agc_vec_t       rpqv;                     /* RPQV primary to secondary body, m 2^29  */
    agc_vec_t       rpsv;                     /* RPSV primary to sun, m 2^38             */
    agc_vec_t       rqvv;                     /* RQVV secondary body to vehicle, m 2^29 (LM only stores it) */
    agc_incorp_w_t  w;                        /* W  error transition matrix: blk[0][m] = position part of column m, blk[1][m] = velocity part (stored at YV/ZV scale x8) */
    agc_dp_t        t_to_add;                 /* T-TO-ADD (CSM MIDTOAV), cs 2^28         */
    bool            notime;                   /* NOTIME taken (IRETURN1 incremented)     */
    agc_integ_out_t out;                      /* RATT .. X2 (push list output)           */
} agc_integ_ctx_t;

/* MIDTOAV1/2 output (the AGC leaves it in RN1, VN1, PIPTIME1, MPAC,
 * RTX1, RTX2).                                                          */
typedef struct agc_integ_midtoav_out
{
    agc_vec_t rn1;      /* RN1 = RATT, m 2^29                              */
    agc_vec_t vn1;      /* VN1 = VATT, m/cs 2^7                            */
    agc_dp_t  piptime1; /* PIPTIME1 = TAT, cs 2^28                         */
    agc_dp_t  dt;       /* MPAC: TAT - TIME2,1, cs 2^28                    */
    int       rtx1;     /* RTX1 = X1                                       */
    int       rtx2;     /* RTX2 = X2                                       */
    bool      slipped;  /* NOTIME: alarm 1703, return to CALLOC +2         */
} agc_integ_midtoav_out_t;

/* ---- context and permanent state ------------------------------------ */
agc_integ_ctx_t *agc_integ_shared_ctx(void);
agc_status_t     agc_integ_ctx_init(agc_integ_ctx_t *ctx);

/* Permanent state vector of `veh` (RRECTCSM.. / RRECTLEM..). get reads the
 * RRECT/VRECT/TET part through agc_erase_rrectcsm() etc.; set writes both. */
agc_status_t agc_integ_perm_get(agc_vehicle_t veh, agc_integ_sv_t *out);
agc_status_t agc_integ_perm_set(agc_vehicle_t veh, const agc_integ_sv_t *in);

/* Lunar potential pad loads used by QUALITY2/3 (SP):
 * E3J22R2M (J22 term, B61 x R^4 product) and E32C31RM (C31 term).       */
agc_status_t agc_integ_set_lunar_coeffs(agc_sp_t e3j22r2m, agc_sp_t e32c31rm);

/* ---- INTSTALL lock (p. 1324-1326) ----------------------------------- */
/* INTSTALL (area 0), ERASTAL1 (1), ERASTAL2 (2; CM only). OK: the area is
 * grabbed. BUSY: the caller's job has been put to sleep (JOBSLEEP) and
 * will be resumed as `resume_job`, which must call this again.          */
agc_status_t agc_integ_install(int area, agc_job_id_t resume_job);

/* INTWAKE: restart-protected release of area 0 (REINTFLG logic).        */
agc_status_t agc_integ_intwake(void);

/* INTWAKE0/INTWAKE1: release area 0 and wake its sleepers.              */
agc_status_t agc_integ_intwake0(void);

/* ERASWAK1/ERASWAK2: release area 1 or 2 (CM).                          */
agc_status_t agc_integ_eraswake(int area);
bool         agc_integ_area_busy(int area);

/* ---- entrances (p. 1317-1320) --------------------------------------- */
/* CSMPREC/LEMPREC/CSMCONIC/LEMCONIC: INTSTALL (BUSY as above), integrate
 * the permanent CSM/LM state to tdec1 (cs 2^28); result in ctx->out.    */
agc_status_t agc_integ_csmprec(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1, agc_job_id_t resume_job);
agc_status_t agc_integ_lemprec(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1, agc_job_id_t resume_job);

/* THISPREC / OTHPREC: CSMPREC or LEMPREC for this / the other vehicle.  */
agc_status_t agc_integ_thisprec(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1, agc_job_id_t resume_job);
agc_status_t agc_integ_othprec(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1, agc_job_id_t resume_job);
agc_status_t agc_integ_csmconic(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1, agc_job_id_t resume_job);
agc_status_t agc_integ_lemconic(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1, agc_job_id_t resume_job);

/* INTEGRVS: the caller holds INTSTALL and has set ctx->sv.rcv/vcv/tet,
 * MOONFLAG and INTYPFLG.                                                */
agc_status_t agc_integ_integrvs(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1);

/* INTEGRV: the caller holds INTSTALL and has set VINTFLAG, INTYPFLG,
 * DIM0FLAG, D6OR9FLG (and STATEFLG if wanted).                          */
agc_status_t agc_integ_integrv(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1);

/* ---- state vector moves (p. 1314-1317, 1347) ------------------------ */
/* ATOPCSM/ATOPLEM: temporary -> permanent, C/LMOONFLG from MOONFLAG,
 * SVDWN downlink copy. x2 = the AGC X2 used by SVDWN: 0 or 2 (PBODY), or
 * AGC_INTEG_X2_ORIGEX right after an origin change in A-PCHK.           */
agc_status_t agc_integ_atopcsm(agc_interp_t *ip, agc_integ_ctx_t *ctx, int x2);
agc_status_t agc_integ_atoplem(agc_interp_t *ip, agc_integ_ctx_t *ctx, int x2);

/* PTOACSM/PTOALEM: permanent -> temporary, MOONFLAG and PBODY set. For
 * the LM on the surface (SURFFLAG) PTOALEM goes to USEPIOS instead and
 * *surface is true: the state was built from RLS at ctx->tdec1 and the
 * integration must continue at A-PCHK. surface may be NULL for PTOACSM. */
agc_status_t agc_integ_ptoacsm(agc_integ_ctx_t *ctx);
agc_status_t agc_integ_ptoalem(agc_interp_t *ip, agc_integ_ctx_t *ctx, bool *surface);

/* RECTIFY: new conic through RCV + TDELTAV, VCV + TNUV (uses ctx->pbody). */
agc_status_t agc_integ_rectify(agc_interp_t *ip, agc_integ_ctx_t *ctx);

/* MINIRECT: VRECT = VCV = v; TDELTAV = TNUV = 0; TC = XKEP = 0.          */
void agc_integ_minirect(agc_integ_ctx_t *ctx, agc_vec_t v);

/* ---- transitions (p. 1327-1333) ------------------------------------- */
/* AVETOMID: thrust -> coast (W through the burn, RN/VN/PIPTIME into the
 * permanent state, other vehicle to PIPTIME). BUSY as for INTSTALL.     */
agc_status_t agc_integ_avetomid(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_job_id_t resume_job);

/* MIDTOAV1: precision-integrate this vehicle to tdec1 (cs 2^28).
 * MIDTOAV2: to now + TIMEDELT steps. BUSY as for INTSTALL.              */
agc_status_t agc_integ_midtoav1(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1, agc_job_id_t resume_job, agc_integ_midtoav_out_t *out);
agc_status_t agc_integ_midtoav2(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_job_id_t resume_job, agc_integ_midtoav_out_t *out);

/* INTWAKEU: finish a P27 state vector update (UPSVFLAG: 0 none, +1 CSM
 * earth, +2 CSM moon, -1 LM earth, -2 LM moon). RRECT/VRECT/TET of the
 * temporary set hold the new state. *upsvflag is zeroed.                 */
agc_status_t agc_integ_intwakeu(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_sp_t *upsvflag);

/* GRP2PC: PHASCHNG OCT 04022 (group 2).                                  */
void agc_integ_grp2pc(void);

/* SETIFLGS: SET STATEFLG, CLEAR INTYPFLG DIM0FLAG D6OR9FLG.              */
void agc_integ_setiflgs(void);

/* THISVINT: VINTFLAG for this vehicle (CM: set, LM: clear).             */
void agc_integ_thisvint(void);

/* ---- STATEINT (periodic P00 integration), jobs and tasks ------------- */
agc_status_t agc_task_stateint(void);                   /* STATEINT task */
agc_status_t agc_job_statint1(agc_job_ctx_t *jc);       /* STATINT1 job  */
agc_status_t agc_job_statint1_stall(agc_job_ctx_t *jc); /* INTSTALL +1   */
agc_status_t agc_job_stateup_stall(agc_job_ctx_t *jc);  /* 2nd INTSTALL  */

/* ---- LM only: SETITCTR (ORBITAL_INTEGRATION LM p. 1245) -------------- */
/* ITERCTR = 20, or 5 when AVEGFLAG is set, then LAMBERT.                 */
agc_status_t agc_integ_setitctr(agc_interp_t *ip, const agc_lambert_in_t *in, agc_lambert_out_t *out);

/* ---- ephemeris (agent nav-aux, src/nav/auxiliary/ephemerides.c) ---------------
 * time_cs: cs scale 2^28.  CM: *moon_pos = moon position earth-centred
 * m 2^29, *sun_pos = sun position earth-centred m 2^38.  LM: unit vectors
 * VMOON/VSUN scale 2^1 (only reachable in the LM through MIDFLAG, which
 * Luminary099 never sets: RME = RMM = DPPOSMAX).  Same declaration as in
 * agc_navaux.h.                                                         */
agc_status_t agc_lsposition(agc_interp_t *ip, agc_dp_t time_cs, agc_vec_t *moon_pos, agc_vec_t *sun_pos);

/* X2 left by ORIGCHNG (LXA,2 ORIGEX): the interpretive return address of
 * CALL ORIGCHNG in A-PCHK, Comanche055 27205 (ANDOUT; yaAGC oracle case
 * origchng_apchk).  ANDOUT -> ATOPCSM -> SVDWN1 runs before RECTIFY
 * reloads X2 from PBODY, so its indexed shifts VSL* 0-7,2 / 0,2 become
 * right shifts of ~27200 places: RN = VN = 0 (R-OTHER = V-OTHER = 0).     */
#define AGC_INTEG_X2_ORIGEX 027205

/* SVDWN1/SVDWN2 (CSM_GEOMETRY / LEM_GEOMETRY) (src/nav/integration/
 * svdwn.c): the integration package calls it from ATOPCSM / ATOPLEM.
 * this_vehicle: RN, VN, PIPTIME (skipped while AVEMIDSW); otherwise
 * R-OTHER, V-OTHER.  x2: 0 earth (B29/B7), 2 moon (B27/B5), or
 * AGC_INTEG_X2_ORIGEX (vectors zeroed).                                  */
agc_status_t agc_integ_svdwn(agc_interp_t *ip, bool this_vehicle, int x2, agc_vec_t rcv, agc_vec_t vcv, agc_vec_t tdeltav, agc_vec_t tnuv, agc_dp_t tet);
agc_vec_t    agc_integ_r_other(void); /* R-OTHER  m B29 (downlink)      */
agc_vec_t    agc_integ_v_other(void); /* V-OTHER  m/cs B7 (downlink)    */

#endif /* AGC_INTEG_H */
