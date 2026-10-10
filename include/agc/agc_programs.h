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
} agc_rendz_t;

/* The rendezvous erasables used by the programs (module storage).          */
agc_rendz_t *agc_rendz_erasable(void);
void         agc_rendz_reset(agc_rendz_t *rz);

/* ---- R30 orbital parameters (SR30.1) ------------------------------------ */
typedef struct agc_r30_in {
    agc_vec_t rone;      /* RONE: m, 2^29 (earth or moon centred)          */
    agc_vec_t vone;      /* VONE: m/cs, 2^7                                */
    bool      moon;      /* V82EMFLG: set = moon (RONE/VONE rescaled SL2)  */
    agc_dp_t  tff_rtmu;  /* TFF/RTMU: 1/sqrt(mu) 2^17 earth / 2^14 moon    */
    agc_dp_t  rpadtem;   /* RPADTEM: pad / landing site radius 2^29 / 2^27 */
    agc_dp_t  hpermin;   /* HPERMIN: 300 kft 2^29 earth / 35 kft 2^27 moon */
} agc_r30_in_t;

typedef struct agc_r30_out {
    agc_dp_t rapo;       /* RAPO: m 2^29 earth / 2^27 moon                  */
    agc_dp_t rper;       /* RPER: m 2^29 earth / 2^27 moon                  */
    agc_dp_t hapox;      /* HAPOX: m 2^29, limited to 9999.9 NM             */
    agc_dp_t hperx;      /* HPERX: m 2^29, limited to 9999.9 NM             */
    agc_dp_t tff;        /* TFF: cs 2^28 (negative quantity)                */
    agc_dp_t neg_tper;   /* -TPER: cs 2^28 (negative, or +0)                */
} agc_r30_out_t;

/* SR30.1 (R30.agc pp.521-523): apogee/perigee, heights, TFF and TPER.     */
agc_status_t agc_r30_sr30_1(agc_interp_t *ip, const agc_r30_in_t *in,
                            agc_r30_out_t *out);
/* MAXCHK (R30.agc p.523): limit a height to MAXNM (9999.9 NM), m 2^29.     */
agc_dp_t     agc_r30_maxchk(agc_interp_t *ip, agc_dp_t h);
/* V82 earth/moon parameter selection (V82GON1 MOONGON/EARTHGON).          */
agc_status_t agc_r30_select_body(bool moon, agc_dp_t rls_mag_or_rpad,
                                 agc_r30_in_t *in);

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
} agc_rendz_periapo_t;
agc_status_t agc_rendz_periapo1(agc_interp_t *ip, const agc_rendz_t *rz,
                                agc_vec_t r, agc_vec_t v,
                                agc_rendz_periapo_t *out);
agc_status_t agc_rendz_s34_35_1(agc_interp_t *ip, agc_rendz_t *rz);
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
agc_status_t agc_rendz_advance(agc_interp_t *ip, agc_rendz_t *rz);
agc_status_t agc_rendz_advance_from(agc_interp_t *ip, agc_rendz_t *rz,
                                    agc_vec_t ra, agc_vec_t va,
                                    agc_vec_t rp, agc_vec_t vp);
/* CSI/A: on success returns OK with DELVEET1/TCDH/... set; on failure
 * returns AGC_STATUS_ALARM with *alarm = 00600..00606.                    */
agc_status_t agc_rendz_csi_a(agc_interp_t *ip, agc_rendz_t *rz,
                             agc_sp_t *alarm);
agc_status_t agc_rendz_cdhmvr(agc_interp_t *ip, agc_rendz_t *rz);
agc_status_t agc_rendz_s32_33_x(agc_interp_t *ip, const agc_rendz_t *rz,
                                agc_mat_t *m);
agc_status_t agc_rendz_s32_33_1(agc_interp_t *ip, agc_rendz_t *rz,
                                agc_vec_t delveet);

/* ---- TPI / TPM (P34-35_P74-75.agc) ------------------------------------- */
/* S33/34.1: iterate TPI time (or elevation); returns AGC_STATUS_ALARM with
 * alarm 00611 when no solution (the AGC's non-zero MPAC exit).           */
agc_status_t agc_rendz_s33_34_1(agc_interp_t *ip, agc_rendz_t *rz,
                                bool itswich, bool etpiflag,
                                agc_dp_t *ttpi);
agc_status_t agc_rendz_s34_35_2(agc_interp_t *ip, agc_rendz_t *rz);
agc_status_t agc_rendz_s34_35_3(agc_interp_t *ip, agc_rendz_t *rz);
agc_status_t agc_rendz_s34_35_4(agc_interp_t *ip, agc_rendz_t *rz);
agc_status_t agc_rendz_initvel(agc_interp_t *ip, agc_rendz_t *rz,
                               agc_sp_t numit, agc_dp_t e4);
agc_status_t agc_rendz_tpi_compute(agc_interp_t *ip, agc_rendz_t *rz,
                                   agc_dp_t ttpi);

/* ---- R36 out-of-plane parameters ---------------------------------------- */
agc_status_t agc_rendz_r36_compute(agc_interp_t *ip, agc_rendz_t *rz,
                                   agc_vec_t rp, agc_vec_t vp,
                                   agc_vec_t ra, agc_vec_t va);

/* ---- R31 rendezvous parameters (COMPDISP) ------------------------------- */
agc_status_t agc_r31_compdisp(agc_interp_t *ip, agc_rendz_t *rz,
                              agc_vec_t rone, agc_vec_t vone,
                              agc_vec_t ratt, agc_vec_t vatt,
                              agc_vec_t axis_nb_sm, const agc_mat_t *refsmmat);

/* ---- P38/P39 stable orbit ----------------------------------------------- */
agc_status_t agc_rendz_p38_deltt(agc_interp_t *ip, agc_rendz_t *rz,
                                 agc_vec_t ratt, agc_vec_t vatt);

/* ---- P17 TPI search (Comanche) ------------------------------------------ */
agc_status_t agc_rendz_s17_1(agc_interp_t *ip, agc_rendz_t *rz, bool *kflag);
agc_status_t agc_rendz_transang(agc_interp_t *ip, agc_rendz_t *rz);

/* ---- P21 ground track ---------------------------------------------------- */
agc_status_t agc_p21_flight_path(agc_interp_t *ip, agc_vec_t r, agc_vec_t v,
                                 agc_sp_t rtx2, agc_dp_t *vel,
                                 agc_dp_t *gamma);

/* ---- P76 ---------------------------------------------------------------- */
agc_status_t agc_p76_lv_to_ref(agc_interp_t *ip, agc_vec_t ratt,
                               agc_vec_t vatt, agc_vec_t dv_lv,
                               agc_vec_t *v_new);

/* ---- P20-P25 measurement processing -------------------------------------- */
/* BVECTORS (CSM R22, P20-P25.agc p.576): B vector and DELTAQ for an optics
 * mark. ustar, um, b: unit vectors 2^1; rclp: LM-CSM, body scale.         */
agc_status_t agc_r22_bvectors(agc_interp_t *ip, agc_vec_t rclp,
                              agc_vec_t um, agc_vec_t *ustar,
                              agc_vec_t *bvec, agc_dp_t *deltaq);

/* ---- program entry points (executive jobs) ------------------------------ */
/* Each returns the status of its first step; later steps run as
 * continuation jobs after DSKY responses (see docs/modules/prog-rendz.md). */
agc_status_t agc_prog_p32(bool this_active);   /* P32 / P72 */
agc_status_t agc_prog_p33(bool this_active);   /* P33 / P73 */
agc_status_t agc_prog_p34(bool this_active);   /* P34 / P74 */
agc_status_t agc_prog_p35(bool this_active);   /* P35 / P75 */
agc_status_t agc_prog_p38(bool this_active);   /* P38 / P78 */
agc_status_t agc_prog_p39(bool this_active);   /* P39 / P79 */
agc_status_t agc_prog_p17(bool this_active);   /* P17 / P77 (CSM)  */
agc_status_t agc_prog_p31(void);               /* P31 (LM)         */
agc_status_t agc_prog_p76(void);
agc_status_t agc_prog_p21(void);
agc_status_t agc_prog_p20(void);
agc_status_t agc_prog_p22(void);
agc_status_t agc_prog_p23(void);               /* CSM              */
agc_status_t agc_prog_p25(void);               /* LM               */
agc_status_t agc_verb82_r30(void);             /* V82E             */
agc_status_t agc_verb83_r31(void);             /* V83E             */
agc_status_t agc_verb90_r36(void);             /* V90E             */

/* Noun data for the rendezvous erasables (delegated from agc_noun_read). */
agc_status_t agc_rendz_noun_read(uint16_t data_id, agc_dp_t *value);
agc_status_t agc_rendz_noun_write(uint16_t data_id, agc_dp_t value);

#endif /* AGC_PROGRAMS_H */
