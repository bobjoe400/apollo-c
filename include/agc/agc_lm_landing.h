/*
 * agc_lm_landing.h -- Luminary 099 lunar landing, landing aborts, lunar
 * ascent, DPS throttle, landing analog displays and the guidance/DAP
 * interface FINDCDUW.  Owned by the guid-landing lane (src/lm/).
 *
 * AGC sources: THE_LUNAR_LANDING.agc (pp. 785-792), THROTTLE_CONTROL_
 * ROUTINES.agc (793-797), LUNAR_LANDING_GUIDANCE_EQUATIONS.agc (798-828),
 * P70-P71.agc (829-837), P12.agc (838-842), ASCENT_GUIDANCE.agc (843-856),
 * LANDING_ANALOG_DISPLAYS.agc (898-907), FINDCDUW--GUIDAP_INTERFACE.agc
 * (908-925).
 *
 * Scale notation: "2^N u" means the stored fraction times 2^N gives the
 * quantity in unit u (cs = centiseconds). "pulses" are DPS throttle
 * command pulses (BITPERF = 0.0797959872 pulse/N). Where the AGC text does
 * not state a scale, the value given was derived from the code and is
 * marked "(derived)".
 */
#ifndef AGC_LM_LANDING_H
#define AGC_LM_LANDING_H

#include <stdbool.h>
#include <stdint.h>
#include "agc_status.h"
#include "agc_word.h"
#include "agc_interp.h"

/* ---- guidance phases (WCHPHASE, p. 798) --------------------------------- */
#define AGC_LM_PHASE_IGNALG   (-1)   /* ignition algorithm                  */
#define AGC_LM_PHASE_BRAKQUAD   0    /* braking phase, P63                  */
#define AGC_LM_PHASE_APPRQUAD   1    /* approach phase, P64                 */
#define AGC_LM_PHASE_VERTICAL   2    /* P65, P66, P67                       */

/* ---- throttle constants (CONTROLLED_CONSTANTS, THROTTLE p. 797) --------- */
#define AGC_LM_FMAXODD   3841        /* FSAT 4.81454413E+4 N, pulses        */
#define AGC_LM_FMAXPOS   3467        /* FMAX 4.34546769E+4 N, pulses        */
#define AGC_LM_FEXTRA    4096        /* BIT13, FEXT 5.13309020E+4 N         */
#define AGC_LM_THROTLAG  20          /* TAU(TH) 0.2 s, cs                   */
#define AGC_LM_3SECS     300         /* cs                                  */

/* ---- one quadratic-guidance target set (W-matrix pad load, p. 121) ------
 * Index 0 = braking (RBRFG..TCGIBRAK), 1 = approach (RAPFG..TCGIAPPR).     */
typedef struct agc_lm_target {
    agc_vec_t rfg;        /* RBRFG  target position, guidance frame, 2^24 m */
    agc_vec_t vfg;        /* VBRFG  target velocity, 2^10 m/cs              */
    agc_vec_t afg;        /* ABRFG  target acceleration, 2^-4 m/cs^2        */
    agc_dp_t  vfg_star;   /* VBRFG* = 18 VDG(z)/8, 2^10 m/cs (TTF cubic)    */
    agc_dp_t  afg_star;   /* ABRFG* = 6 ADG(z), 2^-4 m/cs^2 (TTF cubic)     */
    agc_dp_t  jfg_star;   /* JBRFG* = 8 JDG(z), 2^-18 m/cs^3 (derived)      */
    agc_dp_t  gain;       /* GAINBRAK, CG look-ahead gain ("numero mysterioso") */
    agc_sp_t  tcgf;       /* TCGFBRAK, TTF/8 bound for CG update, 2^14 cs   */
    agc_sp_t  tcgi;       /* TCGIBRAK, TTF/8 bound for CG update, 2^14 cs   */
} agc_lm_target_t;

/* ---- pad loads used by this lane ---------------------------------------- */
typedef struct agc_lm_pad {
    agc_dp_t        tland;      /* TLAND nominal landing time, 2^28 cs      */
    agc_lm_target_t target[2];  /* braking, approach                        */
    agc_dp_t        vign;       /* VIGN desired ignition speed, 2^10 m/cs   */
    agc_dp_t        rignx;      /* RIGNX desired ignition "altitude", 2^24 m */
    agc_dp_t        rignz;      /* RIGNZ desired ignition ground range, 2^24 m */
    agc_dp_t        kignx_b4;   /* KIGNX/B4 (negative)                      */
    agc_dp_t        kigny_b8;   /* KIGNY/B8 (negative)                      */
    agc_dp_t        kignv_b4;   /* KIGNV/B4 (negative)                      */
    agc_sp_t        lowcrit;    /* LOWCRIT throttle region bound, pulses    */
    agc_sp_t        highcrit;   /* HIGHCRIT throttle region bound, pulses   */
    agc_vec_t       v2fg;       /* V2FG P65 desired velocity, 2^10 m/cs     */
    agc_dp_t        tauvert;    /* TAUVERT P65 time constant, 2^14 cs       */
    agc_sp_t        rodscale;   /* RODSCALE click scale factor for ROD      */
    agc_dp_t        taurod;     /* TAUROD ROD time constant                 */
    agc_dp_t        lag_tau;    /* LAG/TAU                                  */
    agc_dp_t        minforce;   /* MINFORCE P66 min force                   */
    agc_dp_t        maxforce;   /* MAXFORCE P66 max force                   */
    agc_dp_t        abtcof[8];  /* ABTCOF D,C,B,A (DPS) then D,C,B,A (APS)  */
    agc_dp_t        vmin;       /* VMIN min abort injection velocity, 2^7 m/cs */
    agc_dp_t        ylim;       /* YLIM max abort cross-range, 2^24 m       */
    agc_dp_t        abtrdot;    /* ABTRDOT desired abort RDOT, 2^7 m/cs     */
    agc_dp_t        costhet1;   /* COSTHET1 cos(cone 1) for aborts          */
    agc_dp_t        costhet2;   /* COSTHET2 cos(cone 2) for aborts          */
    agc_sp_t        zoomtime;   /* ZOOMTIME time of DPS throttle-up, cs     */
    agc_sp_t        tendbrak;   /* TENDBRAK phase switch criterion, 2^14 cs */
    agc_sp_t        tendappr;   /* TENDAPPR                                 */
    agc_sp_t        delttfap;   /* DELTTFAP TTF/8 increment at P63->P64     */
    agc_sp_t        leadtime;   /* LEADTIME (negative), 2^14 cs            */
} agc_lm_pad_t;

/* ---- landing guidance erasables ----------------------------------------- */
typedef struct agc_lm_ldg {
    agc_vec_t land;       /* LAND landing site, SM, 2^24 m                  */
    agc_vec_t landtemp;   /* LANDTEMP                                       */
    agc_dp_t  land_mag;   /* /LAND/ lunar radius at site, 2^24 m            */
    agc_vec_t wm;         /* WM lunar rotation vector (SM), 2^-17 rad/cs (derived) */
    agc_dp_t  ttf_8;      /* TTF/8 time-to-go/8 (negative), 2^14 cs         */
    agc_dp_t  ttf_8tmp;   /* TTF/8TMP                                       */
    agc_dp_t  tpip;       /* TPIP time of PIPA reading, 2^28 cs             */
    agc_dp_t  tpipold;    /* TPIPOLD                                        */
    agc_mat_t cg;         /* CG guidance->SM rows (half unit)               */
    agc_vec_t rgu;        /* RGU position, guidance frame, 2^24 m           */
    agc_vec_t vgu;        /* VGU velocity, guidance frame, 2^10 m/cs        */
    agc_vec_t angterm;    /* ANGTERM V + R x WM, 2^9 m/cs                   */
    agc_dp_t  vhoriz;     /* VHORIZ horizontal velocity, 2^7 m/cs           */
    agc_dp_t  rangedsp;   /* RANGEDSP slant range, 2^24 m                   */
    agc_sp_t  lookangl;   /* LOOKANGL LPD angle, degrees (integer)          */
    agc_vec_t unfc_2;     /* UNFC/2 thrust command, 2^-4 m/cs^2             */
    agc_vec_t unwc_2;     /* UNWC/2 window command (half unit)              */
    agc_dp_t  afc;        /* /AFC/ commanded acceleration, 2^-4 m/cs^2      */
    agc_sp_t  wchphase;   /* WCHPHASE  -1..2                                */
    agc_sp_t  wchphold;   /* WCHPHOLD                                       */
    agc_sp_t  flpass0;    /* FLPASS0 pass counter in phase                  */
    agc_sp_t  nguidsub;   /* NGUIDSUB IGNALG guidance passes left           */
    agc_sp_t  nignloop;   /* NIGNLOOP IGNALG iterations left                */
    agc_sp_t  wchvert;    /* WCHVERT -2 P65, 0 P66, 10 P67                  */
    agc_sp_t  rodcount;   /* RODCOUNT ROD switch clicks                     */
    agc_dp_t  vdgvert;    /* VDGVERT desired altitude rate, 2^7 m/cs        */
    agc_dp_t  ttfdisp;    /* TTFDISP 2^28 cs                                */
    agc_sp_t  tredes;     /* TREDES redesignation time left, s              */
    agc_dp_t  elincr;     /* ELINCR elevation increment (pi rad scale)      */
    agc_dp_t  azincr;     /* AZINCR azimuth increment                       */
    agc_sp_t  elincr1;    /* ELINCR1 accumulating increments (PITFALL)      */
    agc_sp_t  azincr1;    /* AZINCR1                                        */
    agc_sp_t  elvira;     /* ELVIRA LPD bits last pass                      */
    agc_sp_t  zerlina;    /* ZERLINA monitor countdown                      */
    agc_dp_t  outofpln;   /* OUTOFPLN out-of-plane distance, 2^24 m (derived) */
    agc_vec_t r60vsave;   /* R60VSAVE                                       */
    agc_dp_t  deltah;     /* DELTAH display init, 2^24 m                    */
    agc_dp_t  hcalc1;     /* HCALC1 P66 altitude, 2^24 m                    */
    agc_vec_t delvrod;    /* DELVROD P66 PIPA delta, PIPA counts            */
    agc_vec_t vbias;      /* VBIAS PIPA bias velocity                       */
    agc_sp_t  oldpipa[3]; /* OLDPIPAX/Y/Z                                   */
    agc_dp_t  lasttpip;   /* LASTTPIP 2^28 cs                               */
    agc_dp_t  thistpip;   /* THISTPIP 2^28 cs                               */
    agc_sp_t  rodscal1;   /* RODSCAL1                                       */
} agc_lm_ldg_t;

/* ---- throttle erasables (THROTTLE, p. 793) ------------------------------ */
typedef struct agc_lm_thr {
    agc_dp_t fp;          /* FP present thrust, high word = pulses          */
    agc_dp_t fc;          /* FC thrust desired by guidance (high word used) */
    agc_dp_t fcodd;       /* FCODD what it is going to get                  */
    agc_sp_t fcold;       /* FCOLD previous FCODD, pulses                   */
    agc_dp_t pif;         /* PIF throttle increment, high word = pulses     */
    agc_sp_t pifpset;     /* PIFPSET pulse offset (FMAXODD-FP or FEXTRA)    */
    agc_dp_t fweight;     /* FWEIGHT thrust lag weighting, pulses           */
    agc_sp_t tthrot;      /* TTHROT TIME1 of last throttling, cs            */
    agc_sp_t pseudo55;    /* PSEUDO55 throttle command for downlink         */
} agc_lm_thr_t;

/* ---- ascent / abort guidance erasables (ASCENT_GUIDANCE, P12, P70) ------ */
typedef struct agc_lm_asc {
    agc_dp_t  tgo;        /* TGO time to go, 2^17 cs                        */
    agc_dp_t  tgo1;       /* TGO1 (=VGBODY) saved TGO for P71 after P70     */
    agc_dp_t  rco;        /* RCO injection radius, 2^24 m                   */
    agc_dp_t  txo;        /* TXO end of vertical rise, 2^28 cs              */
    agc_dp_t  yco;        /* YCO desired cross range, 2^24 m                */
    agc_dp_t  rdotd;      /* RDOTD desired radial rate, 2^7 m/cs            */
    agc_dp_t  ydotd;      /* YDOTD desired cross-range rate, 2^7 m/cs       */
    agc_dp_t  zdotd;      /* ZDOTD desired downrange rate, 2^7 m/cs         */
    agc_vec_t qaxis;      /* QAXIS unit normal to CSM orbit (half unit)     */
    agc_vec_t zaxis1;     /* ZAXIS1 downrange axis (half unit)              */
    agc_vec_t laxis;      /* LAXIS cross-range axis, 2^1                    */
    agc_dp_t  y;          /* Y cross range, 2^24 m                          */
    agc_dp_t  ydot;       /* YDOT 2^7 m/cs                                  */
    agc_dp_t  zdot;       /* ZDOT 2^7 m/cs                                  */
    agc_dp_t  rdot;       /* RDOT 2^7 m/cs                                  */
    agc_dp_t  r_mag;      /* /R/MAG 2^24 m                                  */
    agc_dp_t  geff;       /* GEFF effective gravity, 2^-10 m/cs^2           */
    agc_dp_t  dzdot;      /* DZDOT 2^7 m/cs                                 */
    agc_dp_t  dydot;      /* DYDOT 2^7 m/cs                                 */
    agc_dp_t  drdot;      /* DRDOT 2^7 m/cs                                 */
    agc_vec_t vgvect;     /* VGVECT velocity to be gained, 2^7 m/cs         */
    agc_vec_t vgbody;     /* VGBODY body-frame VG for N85, 2^7 m/cs         */
    agc_dp_t  ttogo;      /* TTOGO 2^28 cs (negative)                       */
    agc_dp_t  prate;      /* PRATE B, 2^-8 (derived)                        */
    agc_dp_t  yrate;      /* YRATE D, 2^-8                                  */
    agc_dp_t  pcons;      /* PCONS A, 2^9                                   */
    agc_dp_t  ycons;      /* YCONS C, 2^9                                   */
    agc_dp_t  atr;        /* ATR radial thrust accel, 2^-9 m/cs^2           */
    agc_dp_t  aty;        /* ATY cross-range thrust accel, 2^-9 m/cs^2      */
    agc_dp_t  atp;        /* ATP downrange thrust accel, 2^-9 m/cs^2        */
    agc_dp_t  at;         /* AT thrust acceleration, 2^-9 m/cs^2            */
    agc_dp_t  tbup;       /* TBUP burn-up time, 2^17 cs                     */
    agc_dp_t  ve;         /* VE exhaust velocity, 2^7 m/cs                  */
    agc_dp_t  tto;        /* TTO tail-off time, 2^17 cs                     */
    agc_dp_t  dv[4];      /* 1/DV0..1/DV3 (1/DV0 = MASS1)                   */
    agc_dp_t  xrange;     /* XRANGE cross range for N76, 2^29 m             */
    agc_vec_t v1s;        /* V1S velocity at TIG, 2^7 m/cs                  */
    agc_dp_t  yaw;        /* YAW revolutions                                */
    agc_dp_t  pitch;      /* PITCH revolutions                              */
    agc_sp_t  engoffdt;   /* ENGOFFDT delay to engine cut-off, cs           */
    agc_dp_t  rdotv;      /* RDOT used by P12 (alias of RDOT)               */
} agc_lm_asc_t;

/* ---- lane state accessors (landing_state.c) ----------------------------- */
agc_lm_pad_t *agc_lm_pad(void);
agc_lm_ldg_t *agc_lm_ldg(void);
agc_lm_thr_t *agc_lm_thr(void);
agc_lm_asc_t *agc_lm_asc(void);
void          agc_lm_landing_reset(void);

/* basic-language helpers shared by the lane (landing_state.c) */
agc_status_t agc_lm_phaschng(uint16_t octal_code);           /* PHASCHNG  */
agc_status_t agc_lm_fastchng(void);                          /* FASTCHNG  */
agc_sp_t     agc_lm_time1(void);                             /* TIME1     */
agc_sp_t     agc_lm_dv(agc_dp_t dividend, agc_sp_t divisor); /* DV        */
agc_sp_t     agc_lm_mp_hi(agc_sp_t a, agc_sp_t b);           /* MP, high  */
agc_sp_t     agc_lm_ad(agc_sp_t a, agc_sp_t b);              /* AD, sat.  */
bool         agc_lm_chan_bit_present(int chan, uint16_t bit); /* inverted ch30-33 */

/* ---- THROTTLE_CONTROL_ROUTINES (throttle.c) ----------------------------- */
agc_status_t agc_lm_throttle(void);
agc_status_t agc_lm_throttle_measured(agc_dp_t accel);
agc_status_t agc_lm_throttle_flatout(void);
agc_status_t agc_lm_throttle_whatout(agc_sp_t pulses);
agc_dp_t     agc_lm_massmult(agc_dp_t accel);

/* ---- FINDCDUW--GUIDAP_INTERFACE (findcduw.c) ---------------------------- */
agc_status_t agc_lm_initcduw(void);
agc_status_t agc_lm_findcduw(agc_vec_t unfc_2, agc_vec_t unwc_2);
agc_sp_t     agc_lm_sparcsin(agc_sp_t x);        /* SPARCSIN arg scale 1 */
agc_sp_t     agc_lm_arctrgsp(agc_sp_t s, agc_sp_t c);
agc_sp_t     agc_lm_limitsub(agc_sp_t x, agc_sp_t limit);
agc_sp_t     agc_lm_onesto2s(agc_sp_t a);

/* ---- LUNAR_LANDING_GUIDANCE_EQUATIONS (landing_guidance.c) -------------- */
#define AGC_LM_ROOT_MAX_DEGREE 5   /* ROOTPSRS erasables allow N <= 5      */
#define AGC_LM_ROOT_MAX_PASSES 8   /* "Klumpp says give up after 8 passes" */
agc_status_t agc_lm_rootpsrs(const agc_dp_t *coef, int degree, agc_dp_t guess,
                             agc_sp_t precroot, agc_dp_t *root, int *passes);
agc_status_t agc_lm_lunland(void);
agc_status_t agc_lm_guidpass_from(int entry);
agc_status_t agc_lm_ttfincr(agc_interp_t *ip);
agc_status_t agc_lm_calcrgvg(agc_interp_t *ip);
agc_status_t agc_lm_rgvgcalc(agc_interp_t *ip);
agc_status_t agc_lm_ttf8cl(agc_interp_t *ip);
agc_status_t agc_lm_quadguid(agc_interp_t *ip);
agc_status_t agc_lm_afccalc(agc_interp_t *ip, agc_vec_t acg);
agc_status_t agc_lm_cgcalc(agc_interp_t *ip);
agc_status_t agc_lm_vertguid(agc_interp_t *ip);
agc_status_t agc_lm_dispexit(bool kill_group3);
agc_status_t agc_lm_steer_exit(agc_interp_t *ip);
void         agc_lm_tdispset(void);
#define AGC_LM_ENTRY_GUILDRET   0  /* normal pass via LUNLAND/GUILDEN     */
#define AGC_LM_ENTRY_GUIDSUB    1  /* IGNALG passes after the first       */
#define AGC_LM_ENTRY_QGUIDSUB   2  /* ?GUIDSUB first IGNALG pass          */
#define AGC_LM_ENTRY_VERTGUID   3  /* straight to VERTGUID                */

/* ---- landing program flow (lunar_landing.c) ----------------------------- */
agc_status_t agc_lm_job_p63lm(void);
agc_status_t agc_lm_ignalg(void);
agc_status_t agc_lm_job_ignalgrt(void);
agc_status_t agc_lm_job_astnret(void);
agc_status_t agc_lm_job_p63spot3(void);
agc_status_t agc_lm_job_p63spot4(void);
agc_status_t agc_lm_guildenstern(int *entry);
agc_status_t agc_lm_startp64(void);
agc_status_t agc_lm_p65start(void);
agc_status_t agc_lm_redesig(agc_interp_t *ip);
agc_status_t agc_lm_p64disps(void);
agc_status_t agc_lm_job_p64ceed(void);
agc_status_t agc_lm_p66verta(void);
agc_status_t agc_lm_p67vert(agc_interp_t *ip);
agc_status_t agc_lm_task_rodtask(void);
agc_status_t agc_lm_job_rodcomp(void);
agc_status_t agc_lm_pitfall(void);
agc_status_t agc_lm_task_redesmon(void);
agc_status_t agc_lm_descbits(uint16_t chan16_bits);
agc_status_t agc_lm_job_p68(void);
agc_status_t agc_lm_job_p68b(void);

/* ---- P70-P71 (p70_p71.c) ------------------------------------------------ */
agc_status_t agc_lm_task_r10r11(void);
agc_status_t agc_lm_p70(void);
agc_status_t agc_lm_p71(void);
agc_status_t agc_lm_abrtjask(bool p71);
agc_status_t agc_lm_job_goabort(void);
agc_status_t agc_lm_tgocomp(void);
agc_status_t agc_lm_throtup(void);
agc_status_t agc_lm_bothpoly(agc_interp_t *ip, int coef_base);

/* ---- P12 and ASCENT_GUIDANCE (ascent.c) --------------------------------- */
agc_status_t agc_lm_guidinit(agc_interp_t *ip);
agc_status_t agc_lm_p12init(agc_interp_t *ip);
agc_status_t agc_lm_comminit(agc_interp_t *ip);
agc_status_t agc_lm_job_p12lm(void);
agc_status_t agc_lm_job_p12lm2(void);
agc_status_t agc_lm_job_newload(void);
agc_status_t agc_lm_job_newload2(void);
agc_status_t agc_lm_atmag(void);
agc_status_t agc_lm_ascent(agc_interp_t *ip);
agc_dp_t     agc_lm_ycomp(agc_interp_t *ip);
agc_dp_t     agc_lm_logsub(agc_interp_t *ip, agc_dp_t x);
agc_status_t agc_lm_task_engoff1(void);
agc_status_t agc_lm_job_cutoff(void);
agc_status_t agc_lm_job_cutoff1(void);
agc_status_t agc_lm_job_cutoff2(void);
agc_status_t agc_lm_job_termasc(void);

/* ---- LANDING_ANALOG_DISPLAYS (landing_displays.c) ----------------------- */
agc_status_t agc_lm_landisp(void);
agc_status_t agc_lm_task_intlze(void);
agc_status_t agc_lm_task_speedrun2(void);
agc_status_t agc_lm_vmonitor(void);

#endif /* AGC_LM_LANDING_H */
