/**********************************************************************
 * FILE NAME: rendezvous.h
 *
 * PURPOSE:
 *   The programs layer, section "rendezvous" (agent prog-rendz):
 *   rendezvous targeting and navigation programs of both vehicles --
 *     P32/P72 CSI, P33/P73 CDH       (P32-P33_P72-P73.agc,
 *                                     P32-P35_P72-P75.agc)
 *     P34/P74 TPI, P35/P75 TPM, R36  (P34-35_P74-75.agc)
 *     P38/P78, P39/P79 stable orbit  (STABLE_ORBIT.agc)
 *     P17/P77 TPI search (Comanche)  (TPI_SEARCH.agc)
 *     P31 Lambert aimpoint (LM)      (GENERAL_LAMBERT_AIMPOINT_
 *                                     GUIDANCE.agc)
 *     P76 target delta V             (P76.agc)
 *     P21 ground track               (GROUND_TRACKING_DETERMINATION_
 *                                     PROGRAM.agc)
 *     R30 (V82) orbit parameters     (R30.agc)
 *     R31 (V83) rendezvous params    (R31.agc)
 *     P20-P25 rendezvous / orbit nav (P20-P25.agc)
 *     radar sampling lead-in (LM)    (RADAR_LEADIN_ROUTINES.agc)
 *   Source folders: src/programs/{navigation,targeting,displays},
 *   src/csm/navigation, src/lm/navigation, src/lm/radar. Other sections
 *   of the programs layer (alignment) live in agc/programs/alignment.h.
 *
 * AGC SOURCE:
 *   Comanche055 and Luminary099, the files listed above
 *
 * REQUIRES:
 *   agc/core (interp, status, word), agc/exec (job and task ids)
 *
 * GLOBAL VARIABLES:
 *   Name          Type      Scale          Description
 *   ------------  --------  -------------  -------------------------
 *   none (the rendezvous erasables are reached through
 *   agc_rendz_erasable() and the agc_erase_* accessors; struct members
 *   carry their scale)
 *
 * Scale conventions used throughout (from the AGC listings):
 *   positions  meters, scale 2^29 (earth-centred scale, used for all the
 *              RACT/RPASS/RTIG/RTARG erasables regardless of body)
 *   velocities meters/centisecond, scale 2^7
 *   times      centiseconds, scale 2^28
 *   angles     revolutions, scale 2^0 (0.5 = 180 degrees)
 *   unit vectors scale 2^1 (the AGC "half unit" vectors)
 * The conic subroutines work at "body scale": 2^29 / 2^7 about the
 * earth, 2^27 / 2^5 about the moon. RTX2 selects the shift (0 earth, 2
 * moon).
 **********************************************************************/
#ifndef AGC_PROGRAMS_H
#define AGC_PROGRAMS_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/interp.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"

/* ======================================================================== */
/* ==== section "rendezvous" (prog-rendz) ================================= */
/* ======================================================================== */

/* ---- constants (value, AGC label, octal/decimal source) ----------------- */
#define AGC_RENDZ_CSI_LOOPMX      16     /* LOOPMX 2DEC 16: CSI iteration limit  */
#define AGC_RENDZ_TPI_MAXITER     15     /* TITER: OCT 40000 shifted 14 times     */
#define AGC_RENDZ_INITVEL_MAXITER 15     /* bound on INITVEL offset iterations    */
#define AGC_RENDZ_TPISRCH_MAXITER 64     /* bound on TPI SEARCH sector loop       */
#define AGC_RENDZ_600SEC_CS       60000  /* 600SEC/TMIN: 10 min, cs */
#define AGC_RENDZ_60MIN_CS        360000 /* 60MIN: 1 hour, cs */

/* CSI alarm codes, ALARM/TB OCT 00600..00606 (P32-P33 p.663)                */
#define AGC_ALARM_CSI_00600 00600 /* no solution, first pass of elevation    */
#define AGC_ALARM_CSI_00601 00601 /* post-CSI perigee below minimum          */
#define AGC_ALARM_CSI_00602 00602 /* post-CDH perigee below minimum          */
#define AGC_ALARM_CSI_00603 00603 /* CSI to CDH less than 10 minutes         */
#define AGC_ALARM_CSI_00604 00604 /* CDH to TPI less than 10 minutes         */
#define AGC_ALARM_CSI_00605 00605 /* too many iterations (LOOPCT = 16)       */
#define AGC_ALARM_CSI_00606 00606 /* DV exceeds maximum (1000 fps)           */
#define AGC_ALARM_00611     00611 /* S33/34.1: no TPI time */
#define AGC_ALARM_00124     00124 /* P17: no safe pericenter in this sector  */
#define AGC_ALARM_01211     01211 /* P23/R57: marking system busy (BAILOUT)  */

/* ---- primary-body selection, set by SELECTMU ---------------------------- */
typedef struct agc_rendz_body
{
    bool     moon;    /* CMOONFLG at SELECTMU time                         */
    agc_sp_t rtx1;    /* RTX1: -2 earth, -10 moon (X1 of conics), unscaled */
    agc_sp_t rtx2;    /* RTX2: 0 earth, 2 moon (VSR* 0,2 shift), unscaled  */
    agc_dp_t rtmu;    /* RTMU: mu, m^3/cs^2, scale 2^36 (moon value SR 6)  */
    agc_dp_t rtsr1mu; /* RTSR1/MU: 1/sqrt(mu), scale 2^17 earth, 2^14 moon */
} agc_rendz_body_t;

/* ---- rendezvous erasables (Comanche/Luminary ERASABLE_ASSIGNMENTS) ------ */
typedef struct agc_rendz
{
    agc_rendz_body_t body; /* primary body (SELECTMU)                   */

    /* active / passive states at the CSI (1), CDH (2), TPI (3) points,
     * position meters 2^29, velocity m/cs 2^7 */
    agc_vec_t ract1;  /* RACT1: active at CSI, m 2^29                     */
    agc_vec_t ract2;  /* RACT2: active at CDH, m 2^29                     */
    agc_vec_t ract3;  /* RACT3: active at TPI, m 2^29                     */
    agc_vec_t vact1;  /* VACT1: active at CSI, m/cs 2^7                   */
    agc_vec_t vact2;  /* VACT2: active at CDH, m/cs 2^7                   */
    agc_vec_t vact3;  /* VACT3: active at TPI, m/cs 2^7                   */
    agc_vec_t vact4;  /* VACT4: post-CSI velocity, m/cs 2^7               */
    agc_vec_t rpass1; /* RPASS1: passive at CSI, m 2^29                   */
    agc_vec_t rpass2; /* RPASS2: passive at CDH, m 2^29                   */
    agc_vec_t rpass3; /* RPASS3: passive at TPI, m 2^29                   */
    agc_vec_t rpass4; /* RPASS4: passive at the intercept, m 2^29         */
    agc_vec_t vpass1; /* VPASS1: passive at CSI, m/cs 2^7                 */
    agc_vec_t vpass2; /* VPASS2: passive at CDH, m/cs 2^7                 */
    agc_vec_t vpass3; /* VPASS3: passive at TPI, m/cs 2^7                 */
    agc_vec_t vpass4; /* VPASS4: passive at the intercept, m/cs 2^7       */
    agc_vec_t up1;    /* UP1: unit normal, passive orbit plane, 2^1       */
    agc_vec_t unvec;  /* UNVEC: unit(RACT2), 2^1                          */
    agc_vec_t rtig;   /* RTIG: active position at TIG, m 2^29             */
    agc_vec_t vtig;   /* VTIG: active velocity at TIG, m/cs 2^7           */

    /* maneuver results */
    agc_vec_t delveet1; /* DELVEET1: CSI delta V, reference, m/cs 2^7    */
    agc_vec_t delveet2; /* DELVEET2: CDH delta V, reference, m/cs 2^7    */
    agc_vec_t delveet3; /* DELVEET3: TPI/TPM delta V, reference, 2^7     */
    agc_vec_t delvsin;  /* DELVSIN: delta V for thrust program, 2^7      */
    agc_dp_t  delvsab;  /* DELVSAB: |DELVSIN|, m/cs 2^7                  */
    agc_vec_t delvlvc;  /* DELVLVC: delta V local vertical, m/cs 2^7     */
    agc_vec_t dvlos;    /* DVLOS: delta V along LOS frame, m/cs 2^7      */
    agc_vec_t gdt_2;    /* GDT/2 (LM S34/35.5 overwrite copy), m/cs 2^7  */
    agc_vec_t ulos;     /* ULOS: unit line of sight active->passive 2^1  */
    agc_vec_t unrm;     /* UNRM: unit(RACT3 x VACT3), 2^1                */
    agc_vec_t rtarg;    /* RTARG: (offset) target position, m 2^29       */
    agc_vec_t viprime;  /* VIPRIME: required initial velocity, 2^7       */
    agc_vec_t vtprime;  /* VTPRIME: velocity at target after burn, 2^7   */
    agc_vec_t rinit;    /* RINIT: INITVEL initial position, m 2^29       */
    agc_vec_t vinit;    /* VINIT: INITVEL initial velocity, m/cs 2^7     */
    agc_vec_t raprec;   /* S33/34.1 saved active position, m 2^29        */
    agc_vec_t vaprec;   /* S33/34.1 saved active velocity, m/cs 2^7      */
    agc_vec_t rpprec;   /* S33/34.1 saved passive position, m 2^29       */
    agc_vec_t vpprec;   /* S33/34.1 saved passive velocity, m/cs 2^7     */

    /* times, centiseconds 2^28 */
    agc_dp_t tcsi;    /* TCSI: CSI time, cs 2^28                          */
    agc_dp_t tcdh;    /* TCDH: CDH time, cs 2^28                          */
    agc_dp_t ttpio;   /* TTPIO: TPI time of the CSI solution, cs 2^28     */
    agc_dp_t tpass4;  /* TPASS4 (= TINT): intercept time, cs 2^28         */
    agc_dp_t intime;  /* INTIME: time of RINIT, VINIT, cs 2^28            */
    agc_dp_t tstrt;   /* TSTRT: P35 start time, cs 2^28                   */
    agc_dp_t dellt4;  /* DELLT4: transfer time, cs 2^28                   */
    agc_dp_t nomtpi;  /* NOMTPI: TPI time step, cs 2^28                   */
    agc_dp_t t1tot2;  /* T1TOT2: CSI to CDH interval, cs 2^28             */
    agc_dp_t t2tot3;  /* T2TOT3: CDH to TPI interval, cs 2^28             */
    agc_dp_t kt;      /* KT: ATIGINC/PTIGINC preparation time, cs 2^28    */
    agc_dp_t atiginc; /* ATIGINC: P35 TIG increment, this vehicle active (pad loaded), cs 2^28 */
    agc_dp_t ptiginc; /* PTIGINC: P75 TIG increment, other vehicle active (pad loaded), cs 2^28 */
    agc_dp_t hafpa1;  /* HAFPA1: half period after CSI, cs 2^28           */

    /* angles, revolutions 2^0 */
    agc_dp_t elev;     /* ELEV: desired LOS elevation at TPI, rev 2^0     */
    agc_dp_t centang;  /* CENTANG: transfer central angle, rev 2^0        */
    agc_dp_t actcent;  /* ACTCENT: active central angle, rev 2^0          */
    agc_dp_t plus_mga; /* +MGA: middle gimbal angle (-.01/-.02 flags), rev 2^0 */

    agc_sp_t nn;      /* NN: apsidal crossings / offsets, unscaled (B14) */
    agc_dp_t diffalt; /* DIFFALT: delta altitude at CDH, m 2^29          */
    agc_dp_t postcsi; /* POSTCSI: perigee altitude after CSI, m 2^29     */
    agc_dp_t postcdh; /* POSTCDH: perigee altitude after CDH, m 2^29     */
    agc_dp_t posttpi; /* POSTTPI: perigee altitude after TPI, m 2^29     */
    agc_dp_t delvtpi; /* DELVTPI: |dV| at TPI, m/cs 2^7                  */
    agc_dp_t delvtpf; /* DELVTPF: |dV| at TPF, m/cs 2^7                  */
    agc_dp_t delvcsi; /* DELVCSI: CSI horizontal delta V, m/cs 2^7       */
    agc_dp_t deldv;   /* DELDV: CSI iteration step, m/cs 2^7             */
    agc_dp_t dvprev;  /* DVPREV: previous DELVCSI, m/cs 2^7              */
    agc_dp_t gamprev; /* GAMPREV: previous gamma, revolutions 2^1        */
    agc_dp_t rdotv;   /* RDOTV: RACT1.VACT4, m^2/cs 2^36                 */
    agc_sp_t csialrm; /* CSIALRM: CSI failure index 1..7, unscaled       */
    agc_sp_t loopct;  /* LOOPCT: CSI iteration counter, unscaled         */
    agc_dp_t delel;   /* DELEL: S33/34.1 elevation error, rev 2^0        */
    agc_dp_t delelo;  /* DELELO: previous elevation error, rev 2^0       */
    agc_dp_t delteeo; /* DELTEEO: last time step, cs 2^28                */
    agc_dp_t secmax;  /* SECMAX: maximum time step, cs 2^28              */
    agc_sp_t titer;   /* TITER: S33/34.1 iteration bit, unscaled         */
    agc_sp_t vhfcnt;  /* VHFCNT (CM): VHF range marks, unscaled          */

    /* R36 / R31 / R34 displays */
    agc_dp_t  range;    /* RANGE: m 2^29 (R36: Y out of plane)           */
    agc_dp_t  rrate;    /* RRATE: m/cs 2^7 (R36: YDOT)                   */
    agc_dp_t  rtheta;   /* RTHETA: revolutions 2^0 (R36: PSI)            */
    agc_vec_t basethp;  /* BASETHP: this base position, body scale */
    agc_vec_t basethv;  /* BASETHV: this base velocity, body scale */
    agc_vec_t baseotp;  /* BASEOTP: other base position, body scale */
    agc_vec_t baseotv;  /* BASEOTV: other base velocity, body scale */
    agc_dp_t  basetime; /* BASETIME: time of the base vectors, cs 2^28   */

    /* P38/P39 stable orbit */
    agc_dp_t deltar;   /* DELTAR: desired separation, m 2^29         */
    agc_dp_t delttime; /* DELTTIME: time to traverse DELTAR, cs 2^28 */
    agc_dp_t targtime; /* TARGTIME: passive at the intercept point, cs 2^28 (TINT EQUALS TPASS4 and TINTSOI EQUALS DELTAR in the AGC: rz->tpass4, rz->deltar) */

    /* P17 TPI search (Comanche) */
    agc_dp_t  delhite;  /* DELHITE: |RP| - |RA|, m 2^29                   */
    agc_vec_t e2;       /* E2: unit passive orbit normal, 2^1             */
    agc_dp_t  thetzero; /* THETZERO: phase angle, revolutions 2^0         */
    agc_dp_t  tf;       /* TF: transfer time, cs 2^28                     */
    agc_dp_t  hp;       /* HP: pericenter altitude, m 2^29                */
    agc_dp_t  hpermin;  /* HPERMIN: minimum pericenter, m 2^29            */
    agc_dp_t  delvee;   /* DELVEE: total delta V, m/cs 2^7                */
    agc_dp_t  magvtpi;  /* MAGVTPI: |dV| at TPI, m/cs 2^7                 */
    agc_dp_t  reldelv;  /* RELDELV: |dV| at TPF, m/cs 2^7                 */
    agc_sp_t  nn1;      /* NN1: pericenter code for display, unscaled     */
    agc_dp_t  csth;     /* CSTH: cos of the last Lambert transfer angle (INITVEL debris), 2^1 */
    agc_dp_t  snth;     /* SNTH: sin of that angle, 2^1                   */

    /* P21 ground track determination (P21TIME .. P21ALT) */
    agc_dp_t  ground_track_time;   /* P21TIME: base vector time, cs 2^28   */
    agc_vec_t ground_track_base_r; /* P21BASER: m 2^29 earth / 2^27 moon   */
    agc_vec_t ground_track_base_v; /* P21BASEV: m/cs 2^7 earth / 2^5 moon  */
    agc_sp_t  ground_track_origin; /* P21ORIG: 0 earth, 2 moon, unscaled   */
    agc_dp_t  ground_track_vel;    /* P21VEL: |V| m/cs 2^7 (noun 73)       */
    agc_dp_t  ground_track_gamma;  /* P21GAM: flight path angle, rev 2^0   */
    agc_dp_t  ground_track_alt;    /* P21ALT: altitude / 100, m 2^29       */

    /* P76 */
    agc_vec_t delvov; /* DELVOV: other vehicle delta V (LV), 2^7    */

    /* working cells */
    agc_dp_t     tdec2;    /* TDEC2: PRECSET time, cs 2^28               */
    agc_job_id_t qsaved;   /* QSAVED: VN1645 return (job to resume)      */
    agc_job_id_t normex;   /* NORMEX/SUBEXIT: S34/35.5, DISDVLVC return  */
    agc_dp_t     rtmag;    /* RTMAG: |RTARG| (LM INITVEL), m 2^29        */
    agc_dp_t     mu_a;     /* MU/A (LM INITVEL exit), m^2/cs^2 2^?       */
    agc_dp_t     muasteer; /* MUASTEER: MU/A SR 6 (LM INITVEL), MU/A 2^6 */
    agc_sp_t     numit;    /* ITCTR: INITVEL offsets done, unscaled      */

    /* R30 (V82) */
    agc_vec_t rone;                            /* RONE: m 2^29 (moon: SL2)                   */
    agc_vec_t vone;                            /* VONE: m/cs 2^7 (moon: SL2)                 */
    agc_dp_t  hapox;                           /* HAPOX: apogee height, m 2^29               */
    agc_dp_t  hperx;                           /* HPERX: perigee height, m 2^29              */
    agc_dp_t  tff;                             /* TFF: cs 2^28 (negative)                    */
    agc_dp_t  neg_tper;                        /* -TPER: cs 2^28                             */
    agc_dp_t  tstart_orbit;                    /* TSTART82: cs 2^28                          */
    agc_dp_t  rpadtem;                         /* RPADTEM: pad / site radius, m 2^29 / 2^27  */
    agc_dp_t  orbit_parameter_display_hpermin; /* HPERMIN (R30), m 2^29  */
    agc_dp_t  tff_rtmu;                        /* TFF/RTMU: 1/sqrt(mu), 2^17 / 2^14          */
    agc_sp_t  orbit_display_flags;             /* V82FLAGS: tick bits, unscaled      */
} agc_rendz_t;

/* TRKMKCNT (B(1)PRM, rendezvous tracking marks): owned here, zeroed by
 * AVETOMID (nav-integ) and counted by P20.                               */
/* VHFCNT, MRKBUF2 (CM, owned here; the guidance lane clears them).        */
agc_sp_t agc_erase_vhfcnt(void);
void     agc_erase_set_vhfcnt(agc_sp_t v);
agc_sp_t agc_erase_mrkbuf2(int index); /* index 0..6        */

/* MRKBUF1 / MRKBUF2 raw words (buf 1 or 2, index 0..6); same declarations
 * as include/agc/programs/alignment_pending.h.                           */
uint16_t agc_erase_mrkbuf(int buf, int index);
void     agc_erase_set_mrkbuf(int buf, int index, uint16_t v);
void     agc_erase_set_mrkbuf2(int index, agc_sp_t v);

/* The rendezvous erasables used by the programs (module storage).          */
agc_rendz_t *agc_rendz_erasable(void);
void         agc_rendz_reset(agc_rendz_t *rz);

/* ---- R30 orbital parameters (SR30.1) ------------------------------------ */
typedef struct agc_orbit_params_in
{
    agc_vec_t rone;     /* RONE: m, 2^29 (earth or moon centred)          */
    agc_vec_t vone;     /* VONE: m/cs, 2^7                                */
    bool      moon;     /* V82EMFLG: set = moon (RONE/VONE rescaled SL2)  */
    agc_dp_t  tff_rtmu; /* TFF/RTMU: 1/sqrt(mu) 2^17 earth / 2^14 moon    */
    agc_dp_t  rpadtem;  /* RPADTEM: pad / landing site radius 2^29 / 2^27 */
    agc_dp_t  hpermin;  /* HPERMIN: 300 kft 2^29 earth / 35 kft 2^27 moon */
} agc_orbit_params_in_t;

typedef struct agc_orbit_params_out
{
    agc_dp_t rapo;     /* RAPO: m 2^29 earth / 2^27 moon                  */
    agc_dp_t rper;     /* RPER: m 2^29 earth / 2^27 moon                  */
    agc_dp_t hapox;    /* HAPOX: m 2^29, limited to 9999.9 NM             */
    agc_dp_t hperx;    /* HPERX: m 2^29, limited to 9999.9 NM             */
    agc_dp_t tff;      /* TFF: cs 2^28 (negative quantity)                */
    agc_dp_t neg_tper; /* -TPER: cs 2^28 (negative, or +0)                */
} agc_orbit_params_out_t;

/* SR30.1 (R30.agc pp.521-523): apogee/perigee, heights, TFF and TPER.     */
agc_status_t agc_orbit_params_compute(agc_interp_t                *ip,
                                      const agc_orbit_params_in_t *in,
                                      agc_orbit_params_out_t      *out);

/* MAXCHK (R30.agc p.523): limit a height to MAXNM (9999.9 NM), m 2^29.     */
agc_dp_t agc_orbit_params_maxchk(agc_interp_t *ip, agc_dp_t h);

/* V82 earth/moon parameter selection (V82GON1 MOONGON/EARTHGON).          */
agc_status_t agc_orbit_params_select_body(bool moon, agc_dp_t rls_mag_or_rpad, agc_orbit_params_in_t *in);

/* ---- rendezvous subroutines (P34-35_P74-75.agc pp.469-500) ------------- */
agc_status_t agc_rendz_selectmu(agc_rendz_t *rz, bool cmoonflg);
agc_vec_t    agc_rendz_vec_to_body(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t v); /* VSR* 0,2      */

/* SHIFTR1 */
agc_dp_t agc_rendz_scalar_from_body(const agc_rendz_t *rz, agc_dp_t a);

typedef struct agc_rendz_periapo
{
    agc_dp_t rapo;   /* 2D: apocenter radius, body scale                */
    agc_dp_t hapo;   /* 4D: apocenter altitude, body scale              */
    agc_dp_t rperi;  /* 6D: pericenter radius, body scale               */
    agc_dp_t hperi;  /* 8D: pericenter altitude, body scale (MPAC)      */
    agc_dp_t ecc;    /* ECC: eccentricity, scale 2^3                    */
    agc_dp_t xxxalt; /* XXXALT: primary body radius, body scale         */
    agc_dp_t r1;     /* R1 (APSIDES debris): |RVEC|, body scale         */
    agc_dp_t p;      /* P  (APSIDES debris): semilatus rectum/R1, 2^4   */
} agc_rendz_periapo_t;

/* PERIAPO1: r, v at earth scale (shifted to body scale first).             */
agc_status_t agc_rendz_periapo_body_scale(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t r, agc_vec_t v, agc_rendz_periapo_t *out);

/* PERIAPO: r, v already at body scale (RVEC, VVEC).                        */
agc_status_t agc_rendz_periapo(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t r, agc_vec_t v, agc_rendz_periapo_t *out);
agc_status_t agc_rendz_los_and_normal(agc_interp_t *ip, agc_rendz_t *rz);
agc_status_t agc_rendz_lomat(agc_interp_t *ip, const agc_rendz_t *rz, agc_mat_t *m);
agc_status_t agc_rendz_get_lvc(agc_interp_t *ip, agc_vec_t rinit, agc_vec_t vinit, agc_vec_t dv, agc_vec_t *delvlvc);
agc_status_t agc_rendz_get_mga(agc_interp_t *ip, agc_vec_t dv, const agc_mat_t *refsmmat, agc_dp_t *mga);

/* ---- CSI / CDH (P32-P33_P72-P73.agc pp.663-682) ------------------------ */
agc_status_t agc_rendz_rotate(agc_interp_t *ip, agc_vec_t up1, agc_vec_t v, agc_vec_t *out);

/* Calls that integrate take `resume`: AGC_STATUS_BUSY means INTSTALL put
 * the job to sleep and it will be resumed as that job (which redoes the
 * step from its start).                                                   */
/* ADVANCE: PRECSET to TIG, passive plane UP1, active rotated into it.     */
agc_status_t agc_rendz_advance(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume);

/* CSI/A: on success returns OK with DELVEET1/TCDH/... set; on failure
 * returns AGC_STATUS_ALARM with *alarm = 00600..00606.                    */
agc_status_t agc_rendz_csi_a(agc_interp_t *ip, agc_rendz_t *rz, agc_sp_t *alarm, agc_job_id_t resume);
agc_status_t agc_rendz_cdhmvr(agc_interp_t *ip, agc_rendz_t *rz);
agc_status_t agc_rendz_csi_cdh_lv_matrix(agc_interp_t *ip, const agc_rendz_t *rz, agc_mat_t *m);

/* S32/33.1 after the V06N81 display: DELVSIN = M^T DELVLVC, DELVSAB.      */
agc_status_t agc_rendz_csi_cdh_dv_to_reference(agc_interp_t *ip,
                                               agc_rendz_t  *rz,
                                               agc_vec_t    *delvsin);

/* ---- TPI / TPM (P34-35_P74-75.agc) ------------------------------------- */
/* Every routine that integrates takes `resume`: AGC_STATUS_BUSY means
 * INTSTALL put the job to sleep; it is resumed as that job.               */
/* S33/34.1 (pp.469-473): iterate the TPI time for the elevation ELEV
 * (ITSWICH set) or compute ELEV for TTPI (ITSWICH clear). OK = the AGC's
 * zero MPAC exit, ALARM = non-zero MPAC (caller raises 00611).           */
agc_status_t agc_rendz_tpi_time_search(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume);

/* The same, continued at the ADTIME/GOINT step after an INTSTALL sleep. */
agc_status_t agc_rendz_tpi_time_search_cont(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume);

/* S34/35.2 (p.475-476): passive to TPASS4, ACTCENT, INITVEL, DELVLVC.   */
agc_status_t agc_rendz_tpi_required_velocity(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume);

/* S3435.25 (p.475-476): RTARG VPASS4 given -> ACTCENT, DELLT4, INITVEL,
 * DELVLVC (also P38/P39 MAINRTNE).                                     */
agc_status_t agc_rendz_intercept_velocity(agc_interp_t *ip, agc_rendz_t *rz, agc_vec_t rtarg, agc_vec_t vpass4, agc_job_id_t resume);

/* S34/35.3 (p.477): crew-loaded DELVLVC -> DELVEET3, RTARG, DVLOS.      */
agc_status_t agc_rendz_tpi_crew_overwrite(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume);

/* S34/35.4 / NOVRWRT (p.477-478): DVLOS from DELVEET3.                  */
agc_status_t agc_rendz_tpi_los_delta_v(agc_interp_t *ip, agc_rendz_t *rz);

/* INITVEL (pp.486-491): numit Lambert/precision iterations, cone e4.    */
agc_status_t agc_rendz_initvel(agc_interp_t *ip, agc_rendz_t *rz, agc_sp_t numit, agc_dp_t e4, agc_job_id_t resume);

/* HAVEGUES (p.487): INITVEL with the previous COGA as Lambert's guess.  */
agc_status_t agc_rendz_havegues(agc_interp_t *ip, agc_rendz_t *rz, agc_sp_t numit, agc_dp_t e4, agc_job_id_t resume);

/* P34/P74E (pp.464-465): TPASS4 from CENTANG, S34/35.2, DELVTPI,
 * DELVTPF, POSTTPI, TIG.                                                 */
agc_status_t agc_rendz_tpi_compute(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume);

/* ---- R36 out-of-plane parameters ---------------------------------------- */
agc_status_t agc_rendz_out_of_plane_compute(agc_interp_t *ip, agc_rendz_t *rz, agc_vec_t rp, agc_vec_t vp, agc_vec_t ra, agc_vec_t va);

/* ---- R31 rendezvous parameters (COMPDISP) ------------------------------- */
/* COMPDISP (R31.agc): RANGE, RRATE of the other vehicle (RATT VATT) seen
 * from this one (RONE VONE), m 2^29 / m/cs 2^7, and RTHETA, the angle
 * of the vehicle axis (stable-member half unit, before REFSMMAT) above
 * the local horizontal, revolutions.                                    */
agc_status_t agc_rendezvous_parameter_display_compdisp(agc_interp_t *ip, agc_rendz_t *rz, agc_vec_t rone, agc_vec_t vone, agc_vec_t ratt, agc_vec_t vatt, agc_vec_t axis_sm, const agc_mat_t *refsmmat);

/* ---- P38/P39 stable orbit ----------------------------------------------- */
/* DELTTIME (P38 RECYCLE, STABLE_ORBIT p.528): time to traverse DELTAR at
 * the passive horizontal speed, |RP| DELTAR / |VP x RP|, cs 2^28.        */
agc_status_t agc_rendz_stable_orbit_deltt(agc_interp_t *ip, agc_rendz_t *rz, agc_vec_t ratt, agc_vec_t vatt);

/* ---- P17 TPI search (Comanche) ------------------------------------------ */
/* S17.1: both vehicles to TTPI (conic), DELHITE, KFLAG, E2, THETZERO.   */
agc_status_t agc_rendz_tpi_search_phase(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume);

/* S17.2: the search; OK solution, ALARM no safe pericenter (00124).     */
agc_status_t agc_rendz_tpi_search_sector(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume);

/* TRANSANG: CENTANG of the passive vehicle over TF.                    */
agc_status_t agc_rendz_transang(agc_interp_t *ip, agc_rendz_t *rz);

/* ---- P21 ground track ---------------------------------------------------- */
/* P21VSAVE (p.458): |V| (VATT1 SL* 0,2, m/cs 2^7) and the flight path
 * angle asin(unit(RATT) . VATT / |V|), revolutions.                       */
void agc_ground_track_determination_flight_path(agc_interp_t *ip, agc_vec_t ratt, agc_vec_t vatt, agc_vec_t vatt1, int x2, agc_dp_t *vel, agc_dp_t *gamma);

/* ---- P76 ---------------------------------------------------------------- */
agc_status_t agc_target_delta_v_lv_to_ref(agc_interp_t *ip, agc_vec_t ratt, agc_vec_t vatt, agc_vec_t dv_lv, agc_vec_t *v_new);

/* ---- P20-P25 measurement processing -------------------------------------- */
/* BVECTORS (CSM R22, P20-P25.agc p.576): B vector and DELTAQ for an optics
 * mark. ustar, um, b: unit vectors 2^1; rclp: LM-CSM, body scale.         */
agc_status_t agc_rendz_tracking_bvectors(agc_interp_t *ip, agc_vec_t rclp, agc_vec_t um, agc_vec_t *ustar, agc_vec_t *bvec, agc_dp_t *deltaq);

/* ---- navigation pad loads (P20-P25; ERASABLE_ASSIGNMENTS "PL") --------
 * Initial W-matrix values, maximum state vector changes and measurement
 * variances, SP unless noted. Both vehicles keep their own subset.      */
typedef struct agc_rendz_nav_pad
{
    agc_sp_t wrendpos;      /* WRENDPOS: rendezvous W position (LM km 2^-7, CM m 2^-14) */
    agc_sp_t wrendvel;      /* WRENDVEL: rendezvous W velocity (LM km^-1/2 2^11, CM m/cs 2^0) */
    agc_sp_t wshaft;        /* WSHAFT (LM): RR shaft bias W, km 2^-7         */
    agc_sp_t wtrun;         /* WTRUN (LM): RR trunnion bias W, km 2^-7       */
    agc_sp_t wsurfpos;      /* WSURFPOS (LM): W position, scale as WRENDPOS */
    agc_sp_t wsurfvel;      /* WSURFVEL (LM): W velocity, scale as WRENDVEL */
    agc_sp_t rmax;          /* RMAX: max position change, m 2^19             */
    agc_sp_t vmax;          /* VMAX: max velocity change, m/cs 2^7           */
    agc_sp_t shaftvar;      /* SHAFTVAR (LM): rad^2 2^12                     */
    agc_sp_t trunvar;       /* TRUNVAR (LM): rad^2 2^10                      */
    agc_dp_t rangevar;      /* RANGEVAR (LM, DP): RR range variance 2^12     */
    agc_dp_t ratevar;       /* RATEVAR (LM, DP): RR range-rate variance 2^12 */
    agc_sp_t rvarmin;       /* RVARMIN: min range variance (LM 2^-12)        */
    agc_sp_t vvarmin;       /* VVARMIN (LM): min range-rate variance 2^12    */
    agc_sp_t worbpos;       /* WORBPOS (CM): orbit nav W position, m 2^-14   */
    agc_sp_t worbvel;       /* WORBVEL (CM): orbit nav W velocity, m/cs 2^0  */
    agc_sp_t s22wsubl;      /* S22WSUBL (CM): landmark W, m 2^-14            */
    agc_dp_t rpvar;         /* RPVAR (CM, DP): landmark radius variance, scale not stated (B(2)PL) */
    agc_dp_t horizalt;      /* HORIZALT (CM, DP): horizon altitude, m 2^29    */
    agc_sp_t wmidpos;       /* WMIDPOS (CM): cislunar W position, scale not stated (I(1)PL) */
    agc_sp_t wmidvel;       /* WMIDVEL (CM): cislunar W velocity, scale not stated (I(1)PL) */
    agc_sp_t rvarmin_cm[3]; /* RVARMIN +0..+2 (CM, VHF radar): the minimum variance, TP, stored negative; scale not stated (I(3)PL) */
    agc_dp_t rvar;          /* RVAR (CM, DP): VHF range variance, scale not stated (I(2)PL) */
    agc_sp_t altvar;        /* ALTVAR (CM): COAS (R23) sighting variance, milliradians squared 2^-16 */
    agc_sp_t intvar;        /* INTVAR (CM): expected integration variance B-15 */
} agc_rendz_nav_pad_t;

agc_rendz_nav_pad_t *agc_rendz_nav_pad(void);

/* X789 (both): the 7th-9th state components of the W matrix (radar or
 * landmark biases).                                                       */
agc_vec_t agc_erase_x789(void);
void      agc_erase_set_x789(agc_vec_t v);

/* ---- LM radar (P20-P25.agc pp.520-563, 594-613) ------------------------
 * Rendezvous radar (RR) moding and designate, radar reading, landing
 * radar position and lamps (src/programs/rendezvous/rr_*_lm.c,
 * lm/radar/read.c, lm/radar/powered_flight_designate.c, lm/radar/leadin.c).
 * A program that starts a radar operation (RRZERO, RRDESSM, RRDESNB,
 * LRPOS2, a read lead-in) then waits with agc_imu_stall_dev(
 * AGC_IMU_DEV_RADAR, ...) (RADSTALL); RGOODEND / RDBADEND wake it.     */
/* Results of RRDESSM / RRDESNB (the AGC's return points).               */
typedef enum agc_rr_designate_result
{
    AGC_RR_DESIGNATE_NO_COVERAGE = 0,
    /* RRDESSM L+2: LOS outside mode 2
     * coverage on the lunar surface     */
    AGC_RR_DESIGNATE_MANEUVER,
    /* RRDESSM L+3: vehicle maneuver
     * required                          */
    AGC_RR_DESIGNATE_STARTED,
    /* RRDESSM L+4 / RRDESNB L+2: designate
     * begun; the caller RADSTALLs       */
    AGC_RR_DESIGNATE_END_JOB,
    /* ENDOFJOB: a job already waits in
     * RADSTALL, or RRDESNB rejected the
     * angles (alarm 502)                */
    AGC_RR_DESIGNATE_RESULT_COUNT
} agc_rr_designate_result_t;

/* RRLIMCHK: in limits */
bool         agc_rr_limchk(uint16_t cdut, uint16_t cdus);
void         agc_rr_settrkf(void);              /* SETTRKF       */
void         agc_rendz_flip(uint16_t lamp_bit); /* FLIP          */
void         agc_rendz_clradmod(void);          /* CLRADMOD      */
agc_status_t agc_rr_zero(void);                 /* RRZERO        */

/* RRDESSM: designate to RRTARGET (half unit, SM; set by LPS20.1).       */
agc_status_t agc_rr_designate_sm(agc_interp_t *ip, agc_rr_designate_result_t *res);

/* RRDESNB: designate to RR gimbal angles (trunnion, shaft CDU words).   */
agc_status_t agc_rr_designate_nb(agc_interp_t *ip, uint16_t trunnion, uint16_t shaft, agc_rr_designate_result_t *res);
agc_status_t agc_rendz_goto_terminate_tracking(void); /* GOTOV56       */

/* read lead-ins (one sample; LRVEL five), RADAREAD, SAMPLSUM            */
agc_status_t agc_radar_rrrange(void);                                 /* RRRANGE       */
agc_status_t agc_radar_rrrdot(void);                                  /* RRRDOT        */
void         agc_radar_lralt(void);                                   /* LRALT         */
void         agc_radar_lrvel(agc_sp_t beam2);                         /* LRVEL 0,2,4 = Z,Y,X */
void         agc_radar_lr_velocity_beam(int beam, agc_sp_t nsamples); /* LRVELZ/Y/X */
agc_status_t agc_rendz_radar_read(void);                              /* RADAREAD (rupt) */
agc_dp_t     agc_radar_samplsum(void);                                /* SAMPLSUM counts */
uint16_t     agc_erase_dnrrange(int index);                           /* DNRRANGE +index */
agc_status_t agc_radar_lrpos2(void);                                  /* LRPOS2        */
void         agc_radar_landing_radar_lights(void);                    /* R12LITES      */
agc_status_t agc_radar_powered_flight_designate(void);                /* R29 (SERVICER) */

/* waitlist tasks and jobs of the radar routines                         */
agc_status_t agc_task_rrturnon(void);            /* RRTURNON      */
agc_status_t agc_task_rr_zero_counters(void);    /* RRZEROSB      */
agc_status_t agc_task_rr_zero_settled(void);
agc_status_t agc_task_rr_turnon_done(void);
agc_status_t agc_task_rr_zero(void);             /* RRZ2          */
agc_status_t agc_task_rr_goodend(void);          /* RGOODEND      */
agc_status_t agc_task_rr_one_axis(void);         /* NXTRR1AX      */
agc_status_t agc_task_dorrepos(void);            /* DORREPOS      */
agc_status_t agc_task_rr_reposition_drive(void);
agc_status_t agc_task_rr_remode(void);           /* REMODE        */
agc_status_t agc_task_rr_designate_begin(void);  /* BEGDES        */
agc_status_t agc_task_rr_designate_loop(void);   /* DESLOOP       */
agc_status_t agc_job_rr_designate_command(void); /* DODES     */
agc_status_t agc_task_lr_position_scan(void);    /* LRPOSCAN      */
agc_status_t agc_task_lr_position_check(void);   /* LRPOSNXT      */
agc_status_t agc_task_lr_position_settled(void); /* LASTLRDT      */

agc_status_t agc_task_radsamp(void);      /* RADSAMP       */
agc_status_t agc_job_radar_sample(void);  /* DORSAMP       */
agc_status_t agc_job_radar_sample_done(void);
agc_dp_t     agc_erase_rstack(int index); /* RSTACK, 0..3  */
void         agc_erase_set_rstack(int index, agc_dp_t v);

/* RRDESNB to TANG, TANG +1 (verb 41 noun 72); *started false when the
 * job ended instead (angles rejected, or a job already in RADSTALL).    */
agc_status_t agc_rr_designate_gimbal(bool *started);
agc_sp_t     agc_erase_tang(int index); /* TANG +index   */
void         agc_erase_set_tang(int index, agc_sp_t v);

/* R29 downlink set (RM RDOTM MKTIME TANGNB AIG AMG AOG, P20-P25 p.606);
 * TRKMKCNT = 1 when it is consistent.                                   */
typedef struct agc_rr_downlink
{
    uint16_t rm;        /* RM: RR range, raw count                      */
    uint16_t rdotm;     /* RDOTM: RR range rate, raw count              */
    agc_dp_t mktime;    /* MKTIME: time of the reading, cs 2^28         */
    uint16_t tangnb[2]; /* TANGNB: RR trunnion, shaft CDU words         */
    uint16_t aig;       /* AIG: IMU CDUY word at the reading            */
    uint16_t amg;       /* AMG: IMU CDUZ word                           */
    uint16_t aog;       /* AOG: IMU CDUX word                           */
} agc_rr_downlink_t;

const agc_rr_downlink_t *agc_rendz_rr_downlink(void);

/* R29REM0J */
agc_status_t agc_job_powered_flight_remode(void);

/* PREPOS29 */
agc_status_t agc_task_powered_flight_preposition(void);

/* BEGDES29 */
agc_status_t agc_task_powered_flight_designate(void);

/* R29DLOOP */
agc_status_t agc_task_powered_flight_designate_loop(void);

/* R29DODES */
agc_status_t agc_job_powered_flight_designate(void);
agc_status_t agc_task_powered_flight_read(void); /* R29READ  */
agc_status_t agc_task_powered_flight_read_loop(void);

/* R29RDJOB */
agc_status_t agc_job_powered_flight_read(void);
agc_status_t agc_job_powered_flight_read_range(void);
agc_status_t agc_job_powered_flight_read_done(void);

/* ---- program entry points and continuation jobs -------------------------
 * Programs run as executive jobs (agc_jobs.h convention): each entry job
 * runs to its first display; the AGC's returns from flashing displays,
 * DELAYJOB and INTSTALL sleeps are separate continuation jobs. Job ids
 * are listed in docs/modules/prog-rendz.md ("Requested jobs-tasks").    */
/* shared displays (display.c) */
agc_status_t agc_job_rendz_vnpooh(agc_job_ctx_t *ctx); /* VNPOOH TC -5 */

/* VN1645 V16N45 */
agc_status_t agc_job_rendz_ttogo_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendz_kilclock(void);                       /* KILCLOCK     */
agc_status_t agc_job_rendz_ttogo_proceed(void);                  /* N45PROC      */
agc_status_t agc_job_rendz_clupdate(void);                       /* CLUPDATE     */
agc_status_t agc_job_rendz_final_dv_checked(agc_job_ctx_t *ctx); /* S34/35.5 */

/* VN1645 entry for other programs (+MGA, COMPTGO, QSAVED, V16N45).     */
agc_status_t agc_rendz_ttogo_display_start(const agc_job_ctx_t *ctx,
                                           agc_interp_t        *ip,
                                           agc_job_id_t         resume_as);

/* LLASRD / LLASRDA (P22): LAT, LONG, ALT <-> the noun 89 registers.    */
agc_status_t agc_rendz_llasrd(void);
agc_status_t agc_rendz_llasrda(void);

/* ---- job entry points (generated list follows) ---- */
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

/* orbit_parameter_display.c (R30, verb 82) */
agc_status_t agc_job_orbit_parameter_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_orbit_display_vehicle_select(agc_job_ctx_t *ctx);
agc_status_t agc_job_orbit_display_vehicle_selected(agc_job_ctx_t *ctx);
agc_status_t agc_job_orbit_display_recycle(agc_job_ctx_t *ctx);
agc_status_t agc_job_orbit_display_stall(agc_job_ctx_t *ctx);
agc_status_t agc_job_orbit_display_update(void);
agc_status_t agc_job_orbit_display_average_g(void);
agc_status_t agc_job_orbit_display_repeat(void);
agc_status_t agc_job_orbit_display_splash(void);
agc_status_t agc_job_orbit_display_monitor(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendz_display_terminate(void);
agc_status_t agc_job_rendz_endext(void);
agc_status_t agc_task_ticktest(void);

/* target_delta_v.c (P76) */
agc_status_t agc_job_target_delta_v(agc_job_ctx_t *ctx);
agc_status_t agc_job_target_delta_v_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_target_delta_v_tig(agc_job_ctx_t *ctx);
agc_status_t agc_job_target_delta_v_update(void);
agc_status_t agc_job_target_delta_v_integrate(void);
agc_status_t agc_job_target_delta_v_store(void);
agc_status_t agc_job_target_delta_v_end(void);

/* ground_track_determination.c (P21) */
agc_status_t agc_job_ground_track_determination(agc_job_ctx_t *ctx);
agc_status_t agc_job_ground_track_vehicle(agc_job_ctx_t *ctx);
agc_status_t agc_job_ground_track_time(agc_job_ctx_t *ctx);
agc_status_t agc_job_ground_track_compute(agc_job_ctx_t *ctx);
agc_status_t agc_job_ground_track_recycle(agc_job_ctx_t *ctx);

/* stable_orbit.c (P38/P78, P39/P79) */
agc_status_t agc_job_stable_orbit_rendezvous(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_rendezvous_other_vehicle(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_tig(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_centang(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_option(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_option_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_option_chosen(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_deltar(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_intercept_time(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_start(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_recycle(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_target(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_dv_lv(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_dv_accepted(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_midcourse(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_midcourse_other_vehicle(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_midcourse_compute(agc_job_ctx_t *ctx);
agc_status_t agc_job_stable_orbit_midcourse_advance(agc_job_ctx_t *ctx);

/* out_of_plane_display.c (R36, verb 90) */
agc_status_t agc_job_out_of_plane_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_out_of_plane_vehicle(agc_job_ctx_t *ctx);
agc_status_t agc_job_out_of_plane_start(agc_job_ctx_t *ctx);
agc_status_t agc_job_out_of_plane_time(agc_job_ctx_t *ctx);
agc_status_t agc_job_out_of_plane_compute(agc_job_ctx_t *ctx);

/* rendezvous_parameter_display.c (R31 verb 83, CM R34) */
agc_status_t agc_job_rendezvous_parameter_display(void);
agc_status_t agc_task_rendezvous_parameter_display(void); /* R31CALL (LM) */
agc_status_t agc_job_rendezvous_params_wait(agc_job_ctx_t *ctx);
agc_status_t agc_task_rendezvous_params_wait(void);
agc_status_t agc_job_rendezvous_params_monitor(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_params_start(void);
agc_status_t agc_job_rendezvous_params_update(void);

/* csm/navigation/tpi_search.c (P17/P77, CSM) */
agc_status_t agc_job_tpi_search(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_search_other_vehicle(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_search_input(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_search_start(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_search_compute(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_search_results(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_search_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_tpi_search_response(agc_job_ctx_t *ctx);

/* lambert_aimpoint_guidance_lm.c (P31, LM) */
/* P31 (LM) */
agc_status_t agc_job_lambert_aimpoint_guidance_lm(agc_job_ctx_t *ctx);
agc_status_t agc_job_lambert_aimpoint_compute_lm(agc_job_ctx_t *ctx);
agc_status_t agc_job_lambert_aimpoint_apsides(agc_job_ctx_t *ctx);
agc_status_t agc_job_marks_tfi_mga_monitor(agc_job_ctx_t *ctx);

/* sighting_data_and_gyro_torquing.c */
agc_status_t agc_job_chksdata_flash(agc_job_ctx_t *ctx);
agc_status_t agc_job_chksdata_proceed(void);
agc_status_t agc_job_chksdata_enter(void);
agc_status_t agc_job_gyro_torquing_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_gyro_torquing_pulse(void);
agc_status_t agc_job_gyro_torquing_pulsed(void);
agc_status_t agc_job_gyro_torquing_return(void);

/* lm/navigation/navigation.c (P20 = P22 LM, P25 LM) */
/* P20: by vehicle */
agc_status_t agc_job_rendezvous_navigation(agc_job_ctx_t *ctx);

/* PROG20 (LM) */
agc_status_t agc_job_rendezvous_navigation_lm(agc_job_ctx_t *ctx);

/* ORBCHGO */
agc_status_t agc_job_rendezvous_navigation_orbit_option(agc_job_ctx_t *ctx);

/* ORBCHG1 */
agc_status_t agc_job_rendezvous_navigation_orbit_chosen(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_navigation_launch_time(agc_job_ctx_t *ctx);

/* ORBCHG3 */
agc_status_t agc_job_rendezvous_navigation_orbit_change(agc_job_ctx_t *ctx);

/* PROG20A */
agc_status_t agc_job_rendezvous_navigation_start(agc_job_ctx_t *ctx);

/* P20LEM1 */
agc_status_t agc_job_rendezvous_navigation_range_check(agc_job_ctx_t *ctx);

/* 526ALARM */
agc_status_t agc_job_rendezvous_navigation_range_alarm(agc_job_ctx_t *ctx);

/* GOTOV56 */
agc_status_t agc_job_rendezvous_navigation_terminate(void);

/* P20LEMA */
agc_status_t agc_job_rendezvous_navigation_attitude(agc_job_ctx_t *ctx);

/* P20LEMB */
agc_status_t agc_job_rendezvous_navigation_acquire(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_navigation_manual_request(agc_job_ctx_t *ctx);

/* P20LEMB3 */
agc_status_t agc_job_rendezvous_navigation_cdu_zero_wait(void);

/* P20LEMC3 */
agc_status_t agc_job_rendezvous_navigation_update_states(void);

/* P20LEMC */
agc_status_t agc_job_rendezvous_navigation_designate(void);
agc_status_t agc_task_rendezvous_navigation_track_check(void); /* P20LEMC1 */
agc_status_t agc_task_rendezvous_navigation_track_wait(void);  /* P20LEMD1 */

/* PROG25 */
agc_status_t agc_job_preferred_tracking_attitude(void);

/* P25LEM1 */
agc_status_t agc_job_preferred_tracking_attitude_check(void);
agc_status_t agc_task_preferred_tracking_attitude_wait(void); /* P25LEM2 */

/* lm/navigation/tracking_attitude.c (R61LEM / R65LEM) */
/* R61C+L01 */
agc_status_t agc_job_tracking_attitude_compute(agc_job_ctx_t *ctx);

/* after R60LEM */
agc_status_t agc_job_tracking_attitude_maneuvered(void);
agc_status_t agc_task_tracking_attitude_wait(void); /* R61C+L2 */

/* lm/radar/acquisition.c (R21LEM, R23LEM, R24LEM / LRS24.1) */
agc_status_t agc_job_rendezvous_sighting_mark(void);               /* R21LEM */
agc_status_t agc_job_rendezvous_sighting_mark_centered(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_sighting_mark_designate_los(void); /* R21LEM10 */

/* R21LEM2 */
agc_status_t agc_job_rendezvous_sighting_mark_new_los(void);
agc_status_t agc_job_rendezvous_sighting_mark_lockon(agc_job_ctx_t *ctx);

/* R21-503 */
agc_status_t agc_job_rendezvous_sighting_mark_alarm(agc_job_ctx_t *ctx);

/* R21DISP */
agc_status_t agc_job_rendezvous_sighting_mark_verify(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_sighting_mark_wait(void);

/* R21LEM12 */
agc_status_t agc_job_rendezvous_sighting_mark_reposition(agc_job_ctx_t *ctx);

/* R21LEMB */
agc_status_t agc_job_rendezvous_sighting_mark_ahead(void);
agc_status_t agc_task_rendezvous_sighting_mark_redesignate(void);                 /* R21LEM9 */
agc_status_t agc_job_rendezvous_sighting_mark_coverage_alarm(agc_job_ctx_t *ctx); /* R21LEM11 */
agc_status_t agc_job_manual_acquisition(agc_job_ctx_t *ctx);                      /* R23LEM */
agc_status_t agc_job_manual_acquisition_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_manual_acquisition_check(agc_job_ctx_t *ctx);
agc_status_t agc_job_manual_acquisition_limit_alarm(agc_job_ctx_t *ctx);
agc_status_t agc_job_manual_acquisition_terminate(void);
agc_status_t agc_job_manual_acquisition_maneuver(void);
agc_status_t agc_job_radar_search(agc_job_ctx_t *ctx);                            /* R24LEM */
agc_status_t agc_job_radar_search_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_radar_search_accept(agc_job_ctx_t *ctx);                     /* R24END */
agc_status_t agc_job_radar_search_maneuver(void);                                 /* R24LEM3 */
agc_status_t agc_job_radar_search_maneuver_done(agc_job_ctx_t *ctx);
agc_status_t agc_job_radar_search_resume(agc_job_ctx_t *ctx);                     /* R24LEM4 */
agc_status_t agc_job_radar_search_point(void);                                    /* CHKSRCH */
agc_status_t agc_job_radar_search_los(void);                                      /* LRS24.11 */
agc_status_t agc_task_radar_search_next_point(void);                              /* CALLDGCH */
agc_status_t agc_job_radar_search_data_check(void);                               /* DATGDCHK */

/* lm/navigation/{tracking_data,measurement_update}.c                 */
/* (R22LEM, LRS22.1/2, LSR22.3/4) */
agc_status_t agc_job_rendezvous_tracking_data(agc_job_ctx_t *ctx); /* R22LEM */

/* LRS22.1 */
agc_status_t agc_job_rendezvous_tracking_data_range(void);
agc_status_t agc_job_rendezvous_tracking_data_mark(agc_job_ctx_t *ctx);

/* 525 */
agc_status_t agc_job_rendezvous_tracking_data_los_alarm(agc_job_ctx_t *ctx);

/* R22LEM1 */
agc_status_t agc_job_rendezvous_tracking_data_dtheta(agc_job_ctx_t *ctx);

/* R22LEM2 */
agc_status_t agc_job_rendezvous_tracking_data_accept(void);

/* R22WAIT */
agc_status_t agc_job_rendezvous_tracking_data_wait(void);

/* R22LEM3 */
agc_status_t agc_job_rendezvous_tracking_data_update(void);

/* R22LEM42 */
agc_status_t agc_job_rendezvous_tracking_data_next(void);

/* R22LEM93 */
agc_status_t agc_job_rendezvous_tracking_data_done(void);

/* N49DSP */
agc_status_t agc_job_rendezvous_tracking_data_residuals(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_tracking_data_residuals_proceed(void);
agc_status_t agc_job_rendezvous_tracking_data_residuals_recycle(void);
agc_status_t agc_job_rendezvous_tracking_data_residuals_wait(agc_job_ctx_t *ctx);

/* R22RSTRT */
agc_status_t agc_job_rendezvous_tracking_data_restart(void);
agc_status_t agc_job_rendezvous_tracking_data_restart_read(agc_job_ctx_t *ctx);

/* LSR22.3 */
agc_status_t agc_job_rendezvous_tracking_data_incorporate(void);

/* RANGEBQ1 */
agc_status_t agc_job_rendezvous_tracking_data_measure(void);

/* w_matrix_monitor.c (V67, both vehicles) */
agc_status_t agc_job_w_matrix_monitor(agc_job_ctx_t *ctx); /* V67CALL */

/* V06N99DS */
agc_status_t agc_job_w_matrix_monitor_display(agc_job_ctx_t *ctx);

/* V6N99PRO */
agc_status_t agc_job_w_matrix_monitor_proceed(void);

/* csm/navigation/tracking_attitude.c (R61CSM, CRS61.1/.2) */
agc_status_t agc_rendz_tracking_attitude_csm(agc_job_id_t ret_job);
agc_status_t agc_job_tracking_attitude_csm(agc_job_ctx_t *ctx);
agc_status_t agc_job_tracking_attitude_csm_maneuvered(void);
agc_sp_t     agc_erase_r61cntr(void);                             /* R61CNTR (CM)       */
void         agc_erase_set_r61cntr(agc_sp_t v);
uint16_t     agc_rendz_tracking_angles(bool preferred, int axis); /* PRAXIS / CPHIX */

/* csm/navigation/navigation.c (P20 CSM, R21CSM V57, R23CSM V54, dispatch) */
/* P22: by vehicle */
agc_status_t agc_job_orbital_navigation(agc_job_ctx_t *ctx);

/* PROG22 (CM) */
agc_status_t agc_job_orbital_navigation_csm(agc_job_ctx_t *ctx);

/* PROG20 (CM) */
agc_status_t agc_job_rendezvous_navigation_csm(void);
agc_status_t agc_job_rendezvous_navigation_csm_integrate(void);

/* PIKUP20 */
agc_status_t agc_job_rendezvous_navigation_csm_track(void);

/* P20R52JB */
agc_status_t agc_job_rendezvous_navigation_csm_optics(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_navigation_csm_optics_done(void);

/* CONTCHK */
agc_status_t agc_job_rendezvous_navigation_csm_check(void);

/* R21CSM V57 */
agc_status_t agc_job_rendezvous_sighting_mark_csm(agc_job_ctx_t *ctx);
agc_status_t agc_job_backup_sighting_mark(agc_job_ctx_t *ctx); /* R23CSM V54 */

/* R21C1 */
agc_status_t agc_job_rendezvous_sighting_mark_request(agc_job_ctx_t *ctx);

/* R23CSM1 */
agc_status_t agc_job_backup_sighting_mark_request(agc_job_ctx_t *ctx);
agc_status_t agc_job_backup_sighting_mark_taken(agc_job_ctx_t *ctx);

/* R21CSMA */
agc_status_t agc_job_rendezvous_sighting_mark_done(agc_job_ctx_t *ctx);

/* R21END */
agc_status_t agc_job_rendezvous_sighting_mark_end(agc_job_ctx_t *ctx);
agc_sp_t     agc_erase_landmark(void); /* LANDMARK */
void         agc_erase_set_landmark(agc_sp_t v);
agc_sp_t     agc_erase_trunbias(void); /* TRUNBIAS (R57), CDU units  */
uint16_t     agc_erase_horizon(void);  /* HORIZON (noun 70 R3)       */
void         agc_erase_set_horizon(uint16_t v);

/* csm/navigation/tracking_data.c (R22 CSM, VHFREAD) */
agc_status_t agc_job_rendezvous_tracking_data_csm(void); /* R22 */

/* REND1A */
agc_status_t agc_job_rendezvous_tracking_data_csm_mark(void);

/* REND3 */
agc_status_t agc_job_rendezvous_tracking_data_csm_vhf(void);

/* RANGERD */
agc_status_t agc_job_rendezvous_tracking_data_csm_range(void);
agc_status_t agc_job_rendezvous_tracking_data_csm_incorporate(void); /* REND4 */

/* RENDISP */
agc_status_t agc_job_rendezvous_tracking_data_csm_display(void);
agc_status_t agc_job_rendezvous_tracking_data_csm_residuals(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_tracking_data_csm_residuals_proceed(void);
agc_status_t agc_job_rendezvous_tracking_data_csm_residuals_recycle(void);
agc_status_t agc_job_rendezvous_tracking_data_csm_residuals_wait(void);
agc_status_t agc_rendz_vhf_read(void); /* VHFREAD (rupt) */

/* V89 (CM) */
agc_status_t agc_job_rendezvous_final_attitude_csm(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_final_attitude_csm_option(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_final_attitude_csm_chosen(agc_job_ctx_t *ctx);

/* V89RECL */
agc_status_t agc_job_rendezvous_final_attitude_csm_compute(agc_job_ctx_t *ctx);
agc_status_t agc_job_rendezvous_final_attitude_csm_maneuver(agc_job_ctx_t *ctx);

/* csm/navigation/landmark_navigation.c (S22.1) */
agc_status_t agc_job_orbital_navigation_marks(agc_job_ctx_t *ctx); /* S22.1 */
agc_status_t agc_job_orbital_navigation_mark(agc_job_ctx_t *ctx);  /* S22NXTIN */

/* S22BOX42 */
agc_status_t agc_job_orbital_navigation_delta_accept(agc_job_ctx_t *ctx);

/* S22EXEX */
agc_status_t agc_job_orbital_navigation_marks_exit(void);
agc_status_t agc_job_orbital_navigation_site(agc_job_ctx_t *ctx); /* S22F244 */
agc_status_t agc_job_orbital_navigation_site_accept(void);

/* S22.981X */
agc_status_t agc_job_orbital_navigation_site_done(void);

/* S22GTP */
agc_status_t agc_job_orbital_navigation_site_terminate(void);

/* csm/navigation/orbital_navigation.c (P22 CSM) */
/* V06N45 */
agc_status_t agc_job_orbital_navigation_orbit_display(agc_job_ctx_t *ctx);

/* PROG22A */
agc_status_t agc_job_orbital_navigation_landmark(agc_job_ctx_t *ctx);
agc_status_t agc_job_orbital_navigation_code(agc_job_ctx_t *ctx); /* S22DSPPA */
agc_status_t agc_job_orbital_navigation_code_check(agc_job_ctx_t *ctx);

/* P22SUBRB */
agc_status_t agc_job_orbital_navigation_landing_site(agc_job_ctx_t *ctx);
agc_status_t agc_job_orbital_navigation_site_check(agc_job_ctx_t *ctx);

/* CALLR52 */
agc_status_t agc_job_orbital_navigation_optics(agc_job_ctx_t *ctx);

/* DE-GR-50 */
agc_status_t agc_job_orbital_navigation_advanced_optics(agc_job_ctx_t *ctx);

/* DOV5N71 */
agc_status_t agc_job_orbital_navigation_marked(agc_job_ctx_t *ctx);

/* PROG22C */
agc_status_t agc_job_orbital_navigation_process(void);
agc_status_t agc_job_orbital_navigation_over(agc_job_ctx_t *ctx); /* P22OVER */

/* P23 / R57 (csm/navigation/cislunar_navigation.c) */
agc_status_t agc_job_optics_calibration(agc_job_ctx_t *ctx);
agc_status_t agc_job_optics_calibration_mark_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_optics_calibration_bias(void);
agc_status_t agc_job_optics_calibration_end(void);
agc_status_t agc_job_optics_calibration_end_mark(void);
agc_status_t agc_job_cislunar_midcourse_navigation(agc_job_ctx_t *ctx);
agc_status_t agc_job_cislunar_midcourse_navigation_sight(void);
agc_status_t agc_job_cislunar_midcourse_navigation_ident(agc_job_ctx_t *ctx);
agc_status_t agc_job_cislunar_midcourse_navigation_ident_check(agc_job_ctx_t *ctx);
agc_status_t agc_job_cislunar_midcourse_navigation_star(agc_job_ctx_t *ctx);
agc_status_t agc_job_cislunar_midcourse_navigation_planet(agc_job_ctx_t *ctx);
agc_status_t agc_job_cislunar_midcourse_navigation_checklist(agc_job_ctx_t *ctx);
agc_status_t agc_job_cislunar_midcourse_navigation_calibrate(agc_job_ctx_t *ctx);
agc_status_t agc_job_cislunar_midcourse_navigation_point(agc_job_ctx_t *ctx);
agc_status_t agc_job_cislunar_midcourse_navigation_point_axes(agc_job_ctx_t *ctx);
agc_status_t agc_job_cislunar_midcourse_navigation_maneuvered(agc_job_ctx_t *ctx);
agc_status_t agc_job_cislunar_midcourse_navigation_optics(agc_job_ctx_t *ctx);
agc_status_t agc_job_cislunar_midcourse_navigation_optics_done(agc_job_ctx_t *ctx);
agc_status_t agc_job_cislunar_midcourse_navigation_marked(agc_job_ctx_t *ctx);
agc_status_t agc_job_cislunar_midcourse_navigation_mark_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_cislunar_midcourse_navigation_save(agc_job_ctx_t *ctx);
agc_status_t agc_job_cislunar_midcourse_navigation_incorporate(agc_job_ctx_t *ctx);
agc_status_t agc_job_cislunar_midcourse_navigation_update(void);

/* ---- end of job entry points ---- */

/* Noun data for the rendezvous erasables (delegated from agc_noun_read). */
agc_status_t agc_rendz_noun_read(uint16_t data_id, agc_dp_t *value);

/* The vehicle-specific rendezvous noun data (lm/navigation, csm/navigation). */
agc_status_t agc_lm_rendz_noun_read(uint16_t data_id, agc_dp_t *value);
agc_status_t agc_csm_rendz_noun_read(uint16_t data_id, agc_dp_t *value);
agc_status_t agc_rendz_noun_write(uint16_t data_id, agc_dp_t value);

#endif /* AGC_PROGRAMS_H */
