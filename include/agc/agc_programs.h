/*
 * agc_programs.h -- the programs layer (src/programs/).
 *
 * Section "rendezvous" (agent prog-rendz): rendezvous targeting and
 * navigation programs of both vehicles --
 *   P32/P72 CSI, P33/P73 CDH            (P32-P33_P72-P73.agc, P32-P35_P72-P75.agc)
 *   P34/P74 TPI, P35/P75 TPM, R36        (P34-35_P74-75.agc)
 *   P38/P78, P39/P79 stable orbit        (STABLE_ORBIT.agc)
 *   P17/P77 TPI search (Comanche)        (TPI_SEARCH.agc)
 *   P31 general Lambert aimpoint (LM)    (GENERAL_LAMBERT_AIMPOINT_GUIDANCE.agc)
 *   P76 target delta V                   (P76.agc)
 *   P21 ground track                     (GROUND_TRACKING_DETERMINATION_PROGRAM.agc)
 *   R30 (V82) orbit parameters           (R30.agc)
 *   R31 (V83) rendezvous parameters      (R31.agc)
 *   P20-P25 rendezvous / orbital nav     (P20-P25.agc)
 *   radar sampling lead-in (LM)          (RADAR_LEADIN_ROUTINES.agc)
 *
 * Scale conventions used throughout (from the AGC listings):
 *   positions  meters, scale 2^29 (earth-centred scale, used for all the
 *              RACT/RPASS/RTIG/RTARG erasables regardless of primary body)
 *   velocities meters/centisecond, scale 2^7
 *   times      centiseconds, scale 2^28
 *   angles     revolutions, scale 2^0 (0.5 = 180 degrees)
 *   unit vectors scale 2^1 (the AGC "half unit" vectors)
 * The conic subroutines work at "body scale": 2^29 / 2^7 about the earth,
 * 2^27 / 2^5 about the moon. RTX2 selects the shift (0 earth, 2 moon).
 *
 * Other sections of the programs layer (alignment) live in agc_align.h.
 */
#ifndef AGC_PROGRAMS_H
#define AGC_PROGRAMS_H

#include <stdbool.h>
#include <stdint.h>
#include "agc_status.h"
#include "agc_word.h"
#include "agc_interp.h"
#include "agc_exec.h"

/* ======================================================================== */
/* ==== section "rendezvous" (prog-rendz) ================================= */
/* ======================================================================== */

/* ---- constants (value, AGC label, octal/decimal source) ----------------- */
#define AGC_RENDZ_CSI_LOOPMX       16      /* LOOPMX 2DEC 16: CSI iteration limit  */
#define AGC_RENDZ_TPI_MAXITER      15      /* TITER: OCT 40000 shifted 14 times     */
#define AGC_RENDZ_INITVEL_MAXITER  15      /* bound on INITVEL offset iterations    */
#define AGC_RENDZ_TPISRCH_MAXITER  64      /* bound on TPI SEARCH sector loop       */
#define AGC_RENDZ_600SEC_CS        60000   /* 600SEC/TMIN 2DEC 60000 (10 min, cs)   */
#define AGC_RENDZ_60MIN_CS         360000  /* 60MIN 2DEC 360000 (1 hour, cs)        */

/* CSI alarm codes, ALARM/TB OCT 00600..00606 (P32-P33 p.663)                */
#define AGC_ALARM_CSI_00600  00600   /* no solution, first pass of elevation    */
#define AGC_ALARM_CSI_00601  00601   /* post-CSI perigee below minimum          */
#define AGC_ALARM_CSI_00602  00602   /* post-CDH perigee below minimum          */
#define AGC_ALARM_CSI_00603  00603   /* CSI to CDH less than 10 minutes         */
#define AGC_ALARM_CSI_00604  00604   /* CDH to TPI less than 10 minutes         */
#define AGC_ALARM_CSI_00605  00605   /* too many iterations (LOOPCT = 16)       */
#define AGC_ALARM_CSI_00606  00606   /* DV exceeds maximum (1000 fps)           */
#define AGC_ALARM_00611      00611   /* no TPI time for given elevation (S33/34.1) */
#define AGC_ALARM_00124      00124   /* P17: no safe pericenter in this sector  */
#define AGC_ALARM_01211      01211   /* P23/R57: marking system busy (BAILOUT)  */

/* ---- primary-body selection, set by SELECTMU ---------------------------- */
typedef struct agc_rendz_body {
    bool     moon;      /* CMOONFLG at SELECTMU time                          */
    agc_sp_t rtx1;      /* RTX1: -2 earth, -10 moon (MUTABLE index, X1 of conics) */
    agc_sp_t rtx2;      /* RTX2: 0 earth, 2 moon (VSR* 0,2 shift count)       */
    agc_dp_t rtmu;      /* RTMU: mu, m^3/cs^2, scale 2^36 (moon value SR 6)   */
    agc_dp_t rtsr1mu;   /* RTSR1/MU: 1/sqrt(mu), scale 2^17 earth, 2^14 moon  */
} agc_rendz_body_t;

/* ---- rendezvous erasables (Comanche/Luminary ERASABLE_ASSIGNMENTS) ------ */
typedef struct agc_rendz {
    agc_rendz_body_t body;

    /* active / passive states at the CSI (1), CDH (2), TPI (3) points,
     * position meters 2^29, velocity m/cs 2^7 */
    agc_vec_t ract1, ract2, ract3;
    agc_vec_t vact1, vact2, vact3, vact4;     /* VACT4: post-CSI velocity    */
    agc_vec_t rpass1, rpass2, rpass3, rpass4;
    agc_vec_t vpass1, vpass2, vpass3, vpass4;
    agc_vec_t up1;          /* UP1: unit normal, passive orbit plane, 2^1    */
    agc_vec_t unvec;        /* UNVEC: unit(RACT2), 2^1                       */
    agc_vec_t rtig, vtig;   /* RTIG/VTIG: active state at TIG, 2^29 / 2^7    */

    /* maneuver results */
    agc_vec_t delveet1;     /* DELVEET1: CSI delta V, reference, m/cs 2^7    */
    agc_vec_t delveet2;     /* DELVEET2: CDH delta V, reference, m/cs 2^7    */
    agc_vec_t delveet3;     /* DELVEET3: TPI/TPM delta V, reference, 2^7     */
    agc_vec_t delvsin;      /* DELVSIN: delta V for thrust program, 2^7      */
    agc_dp_t  delvsab;      /* DELVSAB: |DELVSIN|, m/cs 2^7                  */
    agc_vec_t delvlvc;      /* DELVLVC: delta V local vertical, m/cs 2^7     */
    agc_vec_t dvlos;        /* DVLOS: delta V along LOS frame, m/cs 2^7      */
    agc_vec_t gdt_2;        /* GDT/2 (LM S34/35.5 overwrite copy), m/cs 2^7  */
    agc_vec_t ulos;         /* ULOS: unit line of sight active->passive 2^1  */
    agc_vec_t unrm;         /* UNRM: unit(RACT3 x VACT3), 2^1                */
    agc_vec_t rtarg;        /* RTARG: (offset) target position, m 2^29       */
    agc_vec_t viprime;      /* VIPRIME: required initial velocity, 2^7       */
    agc_vec_t vtprime;      /* VTPRIME: velocity at target after burn, 2^7   */
    agc_vec_t rinit, vinit; /* RINIT/VINIT: INITVEL input state 2^29 / 2^7   */
    agc_vec_t raprec, vaprec, rpprec, vpprec; /* S33/34.1 saved states       */

    /* times, centiseconds 2^28 */
    agc_dp_t tcsi, tcdh, ttpio, tpass4, intime, tstrt, dellt4, nomtpi;
    agc_dp_t t1tot2, t2tot3;  /* CSI->CDH and CDH->TPI intervals            */
    agc_dp_t kt;              /* KT: ATIGINC/PTIGINC preparation time       */
    agc_dp_t atiginc;         /* ATIGINC: P35 TIG increment, this vehicle
                               * active (pad loaded), cs 2^28               */
    agc_dp_t ptiginc;         /* PTIGINC: P75 TIG increment, other vehicle
                               * active (pad loaded), cs 2^28               */
    agc_dp_t hafpa1;          /* HAFPA1: half period after CSI (2^28)       */

    /* angles, revolutions 2^0 */
    agc_dp_t elev;            /* ELEV: desired LOS elevation at TPI         */
    agc_dp_t centang;         /* CENTANG: transfer central angle            */
    agc_dp_t actcent;         /* ACTCENT: active vehicle central angle      */
    agc_dp_t plus_mga;        /* +MGA: middle gimbal angle (-.01/-.02 flags)*/

    agc_sp_t nn;              /* NN: apsidal crossings / number of offsets  */
    agc_dp_t diffalt;         /* DIFFALT: delta altitude at CDH, m 2^29     */
    agc_dp_t postcsi, postcdh, posttpi;  /* perigee altitudes, m 2^29       */
    agc_dp_t delvtpi, delvtpf;           /* |dV| at TPI / TPF, m/cs 2^7      */
    agc_dp_t delvcsi;         /* DELVCSI: CSI horizontal delta V, m/cs 2^7  */
    agc_dp_t deldv, dvprev;   /* CSI iteration step / previous dV, 2^7      */
    agc_dp_t gamprev;         /* GAMPREV: previous gamma, revolutions 2^1   */
    agc_dp_t rdotv;           /* RDOTV: RACT1.VACT4, m^2/cs 2^36            */
    agc_sp_t csialrm;         /* CSIALRM: CSI failure index 1..7            */
    agc_sp_t loopct;          /* LOOPCT: CSI iteration counter              */
    agc_dp_t delel, delelo;   /* S33/34.1 elevation errors, revs 2^0        */
    agc_dp_t delteeo;         /* DELTEEO: last time step, cs 2^28           */
    agc_dp_t secmax;          /* SECMAX: maximum time step, cs 2^28         */
    agc_sp_t titer;           /* TITER: S33/34.1 iteration bit              */
    agc_sp_t trkmkcnt;        /* TRKMKCNT: rendezvous tracking marks        */

    /* R36 / R31 / R34 displays */
    agc_dp_t range;           /* RANGE: m 2^29 (R36: Y out of plane)        */
    agc_dp_t rrate;           /* RRATE: m/cs 2^7 (R36: YDOT)                */
    agc_dp_t rtheta;          /* RTHETA: revolutions 2^0 (R36: PSI)         */
    agc_vec_t basethp, basethv, baseotp, baseotv; /* R31 base vectors       */
    agc_dp_t basetime;

    /* P38/P39 stable orbit */
    agc_dp_t deltar;          /* DELTAR: desired separation, m 2^29         */
    agc_dp_t delttime;        /* DELTTIME: time to traverse DELTAR, cs 2^28 */
    agc_dp_t tint, tintsoi, targtime;   /* cs 2^28                          */

    /* P17 TPI search (Comanche) */
    agc_dp_t delhite;         /* DELHITE: |RP| - |RA|, m 2^29               */
    agc_vec_t e2;             /* E2: unit passive orbit normal, 2^1         */
    agc_dp_t thetzero;        /* THETZERO: phase angle, revolutions         */
    agc_dp_t tf;              /* TF: transfer time, cs 2^28                 */
    agc_dp_t hp;              /* HP: pericenter altitude, m 2^29            */
    agc_dp_t hpermin;         /* HPERMIN: minimum pericenter, m 2^29        */
    agc_dp_t delvee;          /* DELVEE: total delta V, m/cs 2^7            */
    agc_dp_t magvtpi, reldelv;/* m/cs 2^7                                   */
    agc_sp_t nn1;             /* NN1: pericenter code for display           */

    /* P76 */
    agc_vec_t delvov;         /* DELVOV: other vehicle delta V (LV), 2^7    */

    /* working cells */
    agc_dp_t tdec2;           /* TDEC2: PRECSET time, cs 2^28               */
    agc_job_id_t qsaved;      /* QSAVED: VN1645 return (job to resume)      */
    agc_job_id_t normex;      /* NORMEX/SUBEXIT: S34/35.5, DISDVLVC return  */
    agc_dp_t rtmag;           /* RTMAG: |RTARG| (LM INITVEL), m 2^29        */
    agc_dp_t mu_a;            /* MU/A (LM INITVEL exit), m^2/cs^2 2^?       */
    agc_dp_t muasteer;        /* MUASTEER (LM INITVEL exit)                 */
    agc_sp_t numit;           /* ITCTR: INITVEL offset iterations done      */

    /* R30 (V82) */
    agc_vec_t rone, vone;     /* RONE m 2^29, VONE m/cs 2^7 (moon: SL2)     */
    agc_dp_t hapox, hperx;    /* HAPOX HPERX: m 2^29                        */
    agc_dp_t tff;             /* TFF: cs 2^28 (negative)                    */
    agc_dp_t neg_tper;        /* -TPER: cs 2^28                             */
    agc_dp_t tstart_orbit;        /* TSTART82: cs 2^28                          */
    agc_dp_t rpadtem;         /* RPADTEM: pad / landing site radius         */
    agc_dp_t orbit_parameter_display_hpermin;     /* HPERMIN (R30 use): m 2^29 / 2^27           */
    agc_dp_t tff_rtmu;        /* TFF/RTMU: 1/sqrt(mu)                       */
    agc_sp_t orbit_display_flags;        /* V82FLAGS: bit1 tick -TPER, bit2 tick TFF   */
} agc_rendz_t;

/* TRKMKCNT (B(1)PRM, rendezvous tracking marks): owned here, zeroed by
 * AVETOMID (nav-integ) and counted by P20.                               */
agc_sp_t agc_erase_trkmkcnt(void);
void     agc_erase_set_trkmkcnt(agc_sp_t v);

/* The rendezvous erasables used by the programs (module storage).          */
agc_rendz_t *agc_rendz_erasable(void);
void         agc_rendz_reset(agc_rendz_t *rz);

/* ---- R30 orbital parameters (SR30.1) ------------------------------------ */
typedef struct agc_orbit_params_in {
    agc_vec_t rone;      /* RONE: m, 2^29 (earth or moon centred)          */
    agc_vec_t vone;      /* VONE: m/cs, 2^7                                */
    bool      moon;      /* V82EMFLG: set = moon (RONE/VONE rescaled SL2)  */
    agc_dp_t  tff_rtmu;  /* TFF/RTMU: 1/sqrt(mu) 2^17 earth / 2^14 moon    */
    agc_dp_t  rpadtem;   /* RPADTEM: pad / landing site radius 2^29 / 2^27 */
    agc_dp_t  hpermin;   /* HPERMIN: 300 kft 2^29 earth / 35 kft 2^27 moon */
} agc_orbit_params_in_t;

typedef struct agc_orbit_params_out {
    agc_dp_t rapo;       /* RAPO: m 2^29 earth / 2^27 moon                  */
    agc_dp_t rper;       /* RPER: m 2^29 earth / 2^27 moon                  */
    agc_dp_t hapox;      /* HAPOX: m 2^29, limited to 9999.9 NM             */
    agc_dp_t hperx;      /* HPERX: m 2^29, limited to 9999.9 NM             */
    agc_dp_t tff;        /* TFF: cs 2^28 (negative quantity)                */
    agc_dp_t neg_tper;   /* -TPER: cs 2^28 (negative, or +0)                */
} agc_orbit_params_out_t;

/* SR30.1 (R30.agc pp.521-523): apogee/perigee, heights, TFF and TPER.     */
agc_status_t agc_orbit_params_compute(agc_interp_t *ip, const agc_orbit_params_in_t *in,
                            agc_orbit_params_out_t *out);
/* MAXCHK (R30.agc p.523): limit a height to MAXNM (9999.9 NM), m 2^29.     */
agc_dp_t     agc_orbit_params_maxchk(agc_interp_t *ip, agc_dp_t h);
/* V82 earth/moon parameter selection (V82GON1 MOONGON/EARTHGON).          */
agc_status_t agc_orbit_params_select_body(bool moon, agc_dp_t rls_mag_or_rpad,
                                 agc_orbit_params_in_t *in);

/* ---- rendezvous subroutines (P34-35_P74-75.agc pp.469-500) ------------- */
agc_status_t agc_rendz_selectmu(agc_rendz_t *rz, bool cmoonflg);
agc_vec_t    agc_rendz_vec_to_body(agc_interp_t *ip, const agc_rendz_t *rz,
                                   agc_vec_t v);          /* VSR* 0,2      */
agc_dp_t     agc_rendz_shiftr1(const agc_rendz_t *rz, agc_dp_t a); /* SHIFTR1 */

typedef struct agc_rendz_periapo {
    agc_dp_t rapo;      /* 2D: apocenter radius, body scale                */
    agc_dp_t hapo;      /* 4D: apocenter altitude, body scale              */
    agc_dp_t rperi;     /* 6D: pericenter radius, body scale               */
    agc_dp_t hperi;     /* 8D: pericenter altitude, body scale (MPAC)      */
    agc_dp_t ecc;       /* ECC: eccentricity, scale 2^3                    */
    agc_dp_t xxxalt;    /* XXXALT: primary body radius, body scale         */
    agc_dp_t r1;        /* R1 (APSIDES debris): |RVEC|, body scale         */
    agc_dp_t p;         /* P  (APSIDES debris): semilatus rectum/R1, 2^4   */
} agc_rendz_periapo_t;
/* PERIAPO1: r, v at earth scale (shifted to body scale first).             */
agc_status_t agc_rendz_periapo1(agc_interp_t *ip, const agc_rendz_t *rz,
                                agc_vec_t r, agc_vec_t v,
                                agc_rendz_periapo_t *out);
/* PERIAPO: r, v already at body scale (RVEC, VVEC).                        */
agc_status_t agc_rendz_periapo(agc_interp_t *ip, const agc_rendz_t *rz,
                               agc_vec_t r, agc_vec_t v,
                               agc_rendz_periapo_t *out);
agc_status_t agc_rendz_los_and_normal(agc_interp_t *ip, agc_rendz_t *rz);
agc_status_t agc_rendz_lomat(agc_interp_t *ip, const agc_rendz_t *rz,
                             agc_mat_t *m);
agc_status_t agc_rendz_get_lvc(agc_interp_t *ip, agc_vec_t rinit,
                               agc_vec_t vinit, agc_vec_t dv,
                               agc_vec_t *delvlvc);
agc_status_t agc_rendz_get_mga(agc_interp_t *ip, agc_vec_t dv,
                               const agc_mat_t *refsmmat, agc_dp_t *mga);

/* ---- CSI / CDH (P32-P33_P72-P73.agc pp.663-682) ------------------------ */
agc_status_t agc_rendz_rotate(agc_interp_t *ip, agc_vec_t up1,
                              agc_vec_t v, agc_vec_t *out);
/* Calls that integrate take `resume`: AGC_STATUS_BUSY means INTSTALL put
 * the job to sleep and it will be resumed as that job (which redoes the
 * step from its start).                                                   */
/* ADVANCE: PRECSET to TIG, passive plane UP1, active rotated into it.     */
agc_status_t agc_rendz_advance(agc_interp_t *ip, agc_rendz_t *rz,
                               agc_job_id_t resume);
/* CSI/A: on success returns OK with DELVEET1/TCDH/... set; on failure
 * returns AGC_STATUS_ALARM with *alarm = 00600..00606.                    */
agc_status_t agc_rendz_csi_a(agc_interp_t *ip, agc_rendz_t *rz,
                             agc_sp_t *alarm, agc_job_id_t resume);
agc_status_t agc_rendz_cdhmvr(agc_interp_t *ip, agc_rendz_t *rz);
agc_status_t agc_rendz_csi_cdh_lv_matrix(agc_interp_t *ip, const agc_rendz_t *rz,
                                agc_mat_t *m);
/* S32/33.1 after the V06N81 display: DELVSIN = M^T DELVLVC, DELVSAB.      */
agc_status_t agc_rendz_csi_cdh_dv_to_reference(agc_interp_t *ip, agc_rendz_t *rz,
                                agc_vec_t *delvsin);

/* ---- TPI / TPM (P34-35_P74-75.agc) ------------------------------------- */
/* Every routine that integrates takes `resume`: AGC_STATUS_BUSY means
 * INTSTALL put the job to sleep; it is resumed as that job.               */
/* S33/34.1 (pp.469-473): iterate the TPI time for the elevation ELEV
 * (ITSWICH set) or compute ELEV for TTPI (ITSWICH clear). OK = the AGC's
 * zero MPAC exit, ALARM = non-zero MPAC (caller raises 00611).           */
agc_status_t agc_rendz_tpi_time_search(agc_interp_t *ip, agc_rendz_t *rz,
                                       agc_job_id_t resume);
/* The same, continued at the ADTIME/GOINT step after an INTSTALL sleep. */
agc_status_t agc_rendz_tpi_time_search_cont(agc_interp_t *ip, agc_rendz_t *rz,
                                            agc_job_id_t resume);
/* S34/35.2 (p.475-476): passive to TPASS4, ACTCENT, INITVEL, DELVLVC.   */
agc_status_t agc_rendz_tpi_required_velocity(agc_interp_t *ip, agc_rendz_t *rz,
                                             agc_job_id_t resume);
/* S34/35.3 (p.477): crew-loaded DELVLVC -> DELVEET3, RTARG, DVLOS.      */
agc_status_t agc_rendz_tpi_crew_overwrite(agc_interp_t *ip, agc_rendz_t *rz,
                                          agc_job_id_t resume);
/* S34/35.4 / NOVRWRT (p.477-478): DVLOS from DELVEET3.                  */
agc_status_t agc_rendz_tpi_los_delta_v(agc_interp_t *ip, agc_rendz_t *rz);
/* INITVEL (pp.486-491): numit Lambert/precision iterations, cone e4.    */
agc_status_t agc_rendz_initvel(agc_interp_t *ip, agc_rendz_t *rz,
                               agc_sp_t numit, agc_dp_t e4,
                               agc_job_id_t resume);
/* P34/P74E (pp.464-465): TPASS4 from CENTANG, S34/35.2, DELVTPI,
 * DELVTPF, POSTTPI, TIG.                                                 */
agc_status_t agc_rendz_tpi_compute(agc_interp_t *ip, agc_rendz_t *rz,
                                   agc_job_id_t resume);

/* ---- R36 out-of-plane parameters ---------------------------------------- */
agc_status_t agc_rendz_out_of_plane_compute(agc_interp_t *ip, agc_rendz_t *rz,
                                   agc_vec_t rp, agc_vec_t vp,
                                   agc_vec_t ra, agc_vec_t va);

/* ---- R31 rendezvous parameters (COMPDISP) ------------------------------- */
agc_status_t agc_rendz_params_compdisp(agc_interp_t *ip, agc_rendz_t *rz,
                              agc_vec_t rone, agc_vec_t vone,
                              agc_vec_t ratt, agc_vec_t vatt,
                              agc_vec_t axis_nb_sm, const agc_mat_t *refsmmat);

/* ---- P38/P39 stable orbit ----------------------------------------------- */
agc_status_t agc_rendz_stable_orbit_deltt(agc_interp_t *ip, agc_rendz_t *rz,
                                 agc_vec_t ratt, agc_vec_t vatt);

/* ---- P17 TPI search (Comanche) ------------------------------------------ */
agc_status_t agc_rendz_tpi_search_sector(agc_interp_t *ip, agc_rendz_t *rz, bool *kflag);
agc_status_t agc_rendz_transang(agc_interp_t *ip, agc_rendz_t *rz);

/* ---- P21 ground track ---------------------------------------------------- */
agc_status_t agc_ground_track_flight_path(agc_interp_t *ip, agc_vec_t r, agc_vec_t v,
                                 agc_sp_t rtx2, agc_dp_t *vel,
                                 agc_dp_t *gamma);

/* ---- P76 ---------------------------------------------------------------- */
agc_status_t agc_target_delta_v_lv_to_ref(agc_interp_t *ip, agc_vec_t ratt,
                               agc_vec_t vatt, agc_vec_t dv_lv,
                               agc_vec_t *v_new);

/* ---- P20-P25 measurement processing -------------------------------------- */
/* BVECTORS (CSM R22, P20-P25.agc p.576): B vector and DELTAQ for an optics
 * mark. ustar, um, b: unit vectors 2^1; rclp: LM-CSM, body scale.         */
agc_status_t agc_rendz_tracking_bvectors(agc_interp_t *ip, agc_vec_t rclp,
                              agc_vec_t um, agc_vec_t *ustar,
                              agc_vec_t *bvec, agc_dp_t *deltaq);

/* ---- program entry points and continuation jobs -------------------------
 * Programs run as executive jobs (agc_jobs.h convention): each entry job
 * runs to its first display; the AGC's returns from flashing displays,
 * DELAYJOB and INTSTALL sleeps are separate continuation jobs. Job ids
 * are listed in docs/modules/prog-rendz.md ("Requested jobs-tasks").    */
/* shared displays (display.c) */
agc_status_t agc_job_rendz_vnpooh(agc_job_ctx_t *ctx);        /* VNPOOH TC -5 */
agc_status_t agc_job_rendz_ttogo_display(agc_job_ctx_t *ctx); /* VN1645 V16N45 */
agc_status_t agc_job_rendz_kilclock(agc_job_ctx_t *ctx);      /* KILCLOCK     */
agc_status_t agc_job_rendz_ttogo_proceed(agc_job_ctx_t *ctx); /* N45PROC      */
agc_status_t agc_job_rendz_clupdate(agc_job_ctx_t *ctx);      /* CLUPDATE     */
agc_status_t agc_job_rendz_final_dv_checked(agc_job_ctx_t *ctx); /* S34/35.5 */
/*@JOBS@*/
/* imu_realign.c */
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
/* csi_cdh_targeting.c */
agc_status_t agc_job_csi_targeting(agc_job_ctx_t *ctx);
agc_status_t agc_job_csi_targeting_other_vehicle(agc_job_ctx_t *ctx);
agc_status_t agc_job_csi_input(agc_job_ctx_t *ctx);
agc_status_t agc_job_csi_after_tcsi(agc_job_ctx_t *ctx);
agc_status_t agc_job_csi_after_nn_elev(agc_job_ctx_t *ctx);
agc_status_t agc_job_csi_after_ttpi(agc_job_ctx_t *ctx);
agc_status_t agc_job_csi_alarm_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_csi_compute(agc_job_ctx_t *ctx);
agc_status_t agc_job_csi_after_results(agc_job_ctx_t *ctx);
agc_status_t agc_job_csi_after_dv_lv(agc_job_ctx_t *ctx);
agc_status_t agc_job_csi_after_cdh_dv(agc_job_ctx_t *ctx);
agc_status_t agc_job_cdh_targeting(agc_job_ctx_t *ctx);
agc_status_t agc_job_cdh_targeting_other_vehicle(agc_job_ctx_t *ctx);
agc_status_t agc_job_cdh_input(agc_job_ctx_t *ctx);
agc_status_t agc_job_cdh_after_tcdh(agc_job_ctx_t *ctx);
agc_status_t agc_job_cdh_compute(agc_job_ctx_t *ctx);
agc_status_t agc_job_cdh_time_search(agc_job_ctx_t *ctx);
agc_status_t agc_job_cdh_alarm_proceed(agc_job_ctx_t *ctx);
agc_status_t agc_job_cdh_after_results(agc_job_ctx_t *ctx);
agc_status_t agc_job_cdh_after_dv_lv(agc_job_ctx_t *ctx);
/* tpi_tpm_targeting.c */
agc_status_t agc_job_tpi_targeting(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_targeting_other_vehicle(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_input(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_after_ttpi(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_after_elev(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_cycle(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_intloop(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_time_search(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_alarm(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_compute(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_after_results(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_after_final(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpm_targeting(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpm_targeting_other_vehicle(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpm_compute(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpm_advance(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpm_after_final(agc_job_ctx_t *ctx);
/* sighting_data_and_gyro_torquing.c */
agc_status_t agc_job_chksdata_flash(agc_job_ctx_t *ctx);
agc_status_t agc_job_chksdata_proceed(agc_job_ctx_t *ctx);
agc_status_t agc_job_chksdata_enter(agc_job_ctx_t *ctx);
agc_status_t agc_job_gyro_torquing_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_gyro_torquing_pulse(agc_job_ctx_t *ctx);
agc_status_t agc_job_gyro_torquing_pulsed(agc_job_ctx_t *ctx);
agc_status_t agc_job_gyro_torquing_return(agc_job_ctx_t *ctx);
/*@JOBS-END@*/

/* Noun data for the rendezvous erasables (delegated from agc_noun_read). */
agc_status_t agc_rendz_noun_read(uint16_t data_id, agc_dp_t *value);
agc_status_t agc_rendz_noun_write(uint16_t data_id, agc_dp_t value);

#endif /* AGC_PROGRAMS_H */
