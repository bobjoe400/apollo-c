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

#include "agc/core/interp.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"

/* ---- guidance phases (WCHPHASE, p. 798) --------------------------------- */
#define AGC_LM_PHASE_IGNALG   (-1) /* ignition algorithm                  */
#define AGC_LM_PHASE_BRAKQUAD 0    /* braking phase, P63                  */
#define AGC_LM_PHASE_APPRQUAD 1    /* approach phase, P64                 */
#define AGC_LM_PHASE_VERTICAL 2    /* P65, P66, P67                       */

/* ---- throttle constants (CONTROLLED_CONSTANTS, THROTTLE p. 797) --------- */
#define AGC_LM_FMAXODD  3841 /* FSAT 4.81454413E+4 N, pulses        */
#define AGC_LM_FMAXPOS  3467 /* FMAX 4.34546769E+4 N, pulses        */
#define AGC_LM_FEXTRA   4096 /* BIT13, FEXT 5.13309020E+4 N         */
#define AGC_LM_THROTLAG 20   /* TAU(TH) 0.2 s, cs                   */
#define AGC_LM_3SECS    300  /* cs                                  */

/* ---- one quadratic-guidance target set (W-matrix pad load, p. 121) ------
 * Index 0 = braking (RBRFG..TCGIBRAK), 1 = approach (RAPFG..TCGIAPPR).     */
typedef struct agc_lm_target
{
    agc_vec_t rfg;      /* RBRFG  target position, guidance frame, 2^24 m */
    agc_vec_t vfg;      /* VBRFG  target velocity, 2^10 m/cs              */
    agc_vec_t afg;      /* ABRFG  target acceleration, 2^-4 m/cs^2        */
    agc_dp_t  vfg_star; /* VBRFG* = 18 VDG(z)/8, 2^10 m/cs (TTF cubic)    */
    agc_dp_t  afg_star; /* ABRFG* = 6 ADG(z), 2^-4 m/cs^2 (TTF cubic)     */
    agc_dp_t  jfg_star; /* JBRFG* = 8 JDG(z), 2^-18 m/cs^3 (derived)      */
    agc_dp_t  gain;     /* GAINBRAK, CG look-ahead gain ("numero mysterioso") */
    agc_sp_t  tcgf;     /* TCGFBRAK, TTF/8 bound for CG update, 2^14 cs   */
    agc_sp_t  tcgi;     /* TCGIBRAK, TTF/8 bound for CG update, 2^14 cs   */
} agc_lm_target_t;

/* ---- pad loads used by this lane ---------------------------------------- */
typedef struct agc_lm_pad
{
    agc_dp_t        tland;     /* TLAND nominal landing time, 2^28 cs      */
    agc_lm_target_t target[2]; /* braking, approach                        */
    agc_dp_t        vign;      /* VIGN desired ignition speed, 2^10 m/cs   */
    agc_dp_t        rignx;     /* RIGNX desired ignition "altitude", 2^24 m */
    agc_dp_t        rignz;     /* RIGNZ desired ignition ground range, 2^24 m */
    agc_dp_t        kignx_b4;  /* KIGNX/B4 (negative)                      */
    agc_dp_t        kigny_b8;  /* KIGNY/B8 (negative)                      */
    agc_dp_t        kignv_b4;  /* KIGNV/B4 (negative)                      */
    agc_sp_t        lowcrit;   /* LOWCRIT throttle region bound, pulses    */
    agc_sp_t        highcrit;  /* HIGHCRIT throttle region bound, pulses   */
    agc_vec_t       v2fg;      /* V2FG P65 desired velocity, 2^10 m/cs     */
    agc_dp_t        tauvert;   /* TAUVERT P65 time constant, 2^14 cs       */
    agc_sp_t        rodscale;  /* RODSCALE click scale factor for ROD      */
    agc_dp_t        taurod;    /* TAUROD ROD time constant                 */
    agc_dp_t        lag_tau;   /* LAG/TAU                                  */
    agc_dp_t        minforce;  /* MINFORCE P66 min force                   */
    agc_dp_t        maxforce;  /* MAXFORCE P66 max force                   */
    agc_dp_t        abtcof[8]; /* ABTCOF D,C,B,A (DPS) then D,C,B,A (APS)  */
    agc_dp_t        vmin;      /* VMIN min abort injection velocity, 2^7 m/cs */
    agc_dp_t        ylim;      /* YLIM max abort cross-range, 2^24 m       */
    agc_dp_t        abtrdot;   /* ABTRDOT desired abort RDOT, 2^7 m/cs     */
    agc_dp_t        costhet1;  /* COSTHET1 cos(cone 1) for aborts          */
    agc_dp_t        costhet2;  /* COSTHET2 cos(cone 2) for aborts          */
    agc_sp_t        tendbrak;  /* TENDBRAK phase switch criterion, 2^14 cs */
    agc_sp_t        tendappr;  /* TENDAPPR                                 */
    agc_sp_t        delttfap;  /* DELTTFAP TTF/8 increment at P63->P64     */
    agc_sp_t        leadtime;  /* LEADTIME (negative), 2^14 cs            */
} agc_lm_pad_t;

/* ---- landing guidance erasables ----------------------------------------- */
typedef struct agc_lm_ldg
{
    agc_vec_t land;     /* LAND landing site, SM, 2^24 m                  */
    agc_vec_t landtemp; /* LANDTEMP                                       */
    agc_dp_t  land_mag; /* /LAND/ lunar radius at site, 2^24 m            */
    agc_vec_t wm;    /* WM lunar rotation vector (SM), 2^-17 rad/cs (derived) */
    agc_dp_t  ttf_8; /* TTF/8 time-to-go/8 (negative), 2^14 cs         */
    agc_dp_t  ttf_8tmp; /* TTF/8TMP                                       */
    agc_sp_t  ttf_8_hi; /* TTF/8 high word as the last DAS left it        */
    agc_sp_t  ttf_8tmp_hi; /* TTF/8TMP high word, likewise                */
    agc_dp_t  tpip;     /* TPIP time of PIPA reading, 2^28 cs             */
    agc_dp_t  tpipold;  /* TPIPOLD                                        */
    agc_mat_t cg;       /* CG guidance->SM rows (half unit)               */
    agc_vec_t rgu;      /* RGU position, guidance frame, 2^24 m           */
    agc_vec_t vgu;      /* VGU velocity, guidance frame, 2^10 m/cs        */
    agc_vec_t angterm;  /* ANGTERM V + R x WM, 2^9 m/cs                   */
    agc_dp_t  vhoriz;   /* VHORIZ horizontal velocity, 2^7 m/cs           */
    agc_dp_t  rangedsp; /* RANGEDSP slant range, 2^24 m                   */
    agc_sp_t  lookangl; /* LOOKANGL LPD angle, degrees (integer)          */
    agc_vec_t unfc_2;   /* UNFC/2 thrust command, 2^-4 m/cs^2 (any length;
                             FINDCDUW -2 input, also used by P40 / ascent) */
    agc_vec_t unwc_2;   /* UNWC/2 window command (half unit)              */
    agc_dp_t  afc;      /* /AFC/ commanded acceleration, 2^-4 m/cs^2      */
    agc_sp_t  wchphase; /* WCHPHASE  -1..2                                */
    agc_sp_t  wchphold; /* WCHPHOLD                                       */
    agc_sp_t  flpass0;  /* FLPASS0 pass counter in phase                  */
    agc_sp_t  nguidsub; /* NGUIDSUB IGNALG guidance passes left           */
    agc_sp_t  nignloop; /* NIGNLOOP IGNALG iterations left                */
    agc_sp_t  wchvert;  /* WCHVERT -2 P65, 0 P66, 10 P67                  */
    agc_sp_t  rodcount; /* RODCOUNT ROD switch clicks                     */
    agc_dp_t  vdgvert;  /* VDGVERT desired altitude rate, 2^7 m/cs        */
    agc_dp_t  ttfdisp;  /* TTFDISP 2^28 cs                                */
    agc_sp_t  tredes;   /* TREDES redesignation time left, s              */
    agc_dp_t  elincr;   /* ELINCR elevation increment (pi rad scale)      */
    agc_dp_t  azincr;   /* AZINCR azimuth increment                       */
    agc_sp_t  elincr1;  /* ELINCR1 accumulating increments (PITFALL)      */
    agc_sp_t  azincr1;  /* AZINCR1                                        */
    agc_sp_t  elvira;   /* ELVIRA LPD bits last pass                      */
    agc_sp_t  zerlina;  /* ZERLINA monitor countdown                      */
    agc_dp_t  outofpln; /* OUTOFPLN out-of-plane distance, 2^24 m (derived) */
    agc_vec_t
              pointvsm_save; /* R60VSAVE                                       */
    agc_dp_t  deltah;       /* DELTAH display init, 2^24 m                    */
    agc_dp_t  hcalc1;       /* HCALC1 P66 altitude, 2^24 m                    */
    agc_vec_t delvrod;      /* DELVROD P66 PIPA delta, PIPA counts            */
    agc_vec_t vbias;        /* VBIAS PIPA bias velocity                       */
    agc_sp_t  oldpipa[3];   /* OLDPIPAX/Y/Z                                   */
    agc_dp_t  lasttpip;     /* LASTTPIP 2^28 cs                               */
    agc_dp_t  thistpip;     /* THISTPIP 2^28 cs                               */
    agc_sp_t  rodscal1;     /* RODSCAL1                                       */
} agc_lm_ldg_t;

/* ---- throttle erasables (THROTTLE, p. 793) ------------------------------ */
/* A DP as the AGC stores it: two words whose signs may disagree after a
 * DAS (oracle finding F9). Basic-language code that reads the high word
 * alone (CA, AD, MP, CCS) sees `hi`, not the high word of the value.
 * Value = hi * 2^14 + lo.                                                */
typedef struct agc_lm_words
{
    agc_sp_t hi; /* major word                                        */
    agc_sp_t lo; /* minor word                                        */
} agc_lm_words_t;

typedef struct agc_lm_thr
{
    agc_lm_words_t fp;    /* FP present thrust, hi = pulses (2^14 pulses) */
    agc_dp_t       fc;    /* FC thrust desired, hi = pulses (TS FC: hi)   */
    agc_lm_words_t fcodd; /* FCODD what it is going to get, 2^14 pulses   */
    agc_sp_t       fcold; /* FCOLD previous FCODD, pulses                 */
    agc_lm_words_t pif;     /* PIF = FCODD - FP by DAS, hi = pulses         */
    agc_sp_t       pifpset; /* PIFPSET pulse offset (FMAXODD-FP or FEXTRA)  */
    agc_lm_words_t fweight; /* FWEIGHT thrust lag weighting, 2^14 pulses  */
    agc_sp_t       tthrot; /* TTHROT TIME1 of last throttling, cs            */
    agc_sp_t       pseudo55; /* PSEUDO55 throttle command for downlink         */
} agc_lm_thr_t;

/* ---- ascent / abort guidance erasables (ASCENT_GUIDANCE, P12, P70) ------ */
typedef struct agc_lm_asc
{
    agc_dp_t  tgo;      /* TGO time to go, 2^17 cs                        */
    agc_dp_t  tgo1;     /* TGO1 (=VGBODY) saved TGO for P71 after P70     */
    agc_dp_t  rco;      /* RCO injection radius, 2^24 m                   */
    agc_dp_t  txo;      /* TXO end of vertical rise, 2^28 cs              */
    agc_dp_t  yco;      /* YCO desired cross range, 2^24 m                */
    agc_dp_t  rdotd;    /* RDOTD desired radial rate, 2^7 m/cs            */
    agc_dp_t  ydotd;    /* YDOTD desired cross-range rate, 2^7 m/cs       */
    agc_dp_t  zdotd;    /* ZDOTD desired downrange rate, 2^7 m/cs         */
    agc_vec_t qaxis;    /* QAXIS unit normal to CSM orbit (half unit)     */
    agc_vec_t zaxis1;   /* ZAXIS1 downrange axis (half unit)              */
    agc_vec_t laxis;    /* LAXIS cross-range axis, 2^1                    */
    agc_dp_t  y;        /* Y cross range, 2^24 m                          */
    agc_dp_t  ydot;     /* YDOT 2^7 m/cs                                  */
    agc_dp_t  zdot;     /* ZDOT 2^7 m/cs                                  */
    agc_dp_t  rdot;     /* RDOT 2^7 m/cs                                  */
    agc_dp_t  r_mag;    /* /R/MAG 2^24 m                                  */
    agc_dp_t  geff;     /* GEFF effective gravity, 2^-10 m/cs^2           */
    agc_dp_t  dzdot;    /* DZDOT 2^7 m/cs                                 */
    agc_dp_t  dydot;    /* DYDOT 2^7 m/cs                                 */
    agc_dp_t  drdot;    /* DRDOT 2^7 m/cs                                 */
    agc_vec_t vgvect;   /* VGVECT velocity to be gained, 2^7 m/cs         */
    agc_vec_t vgbody;   /* VGBODY body-frame VG for N85, 2^7 m/cs         */
    agc_dp_t  ttogo;    /* TTOGO 2^28 cs (negative)                       */
    agc_dp_t  prate;    /* PRATE B, 2^-8 (derived)                        */
    agc_dp_t  yrate;    /* YRATE D, 2^-8                                  */
    agc_dp_t  pcons;    /* PCONS A, 2^9                                   */
    agc_dp_t  ycons;    /* YCONS C, 2^9                                   */
    agc_dp_t  atr;      /* ATR radial thrust accel, 2^-9 m/cs^2           */
    agc_dp_t  aty;      /* ATY cross-range thrust accel, 2^-9 m/cs^2      */
    agc_dp_t  atp;      /* ATP downrange thrust accel, 2^-9 m/cs^2        */
    agc_dp_t  at;       /* AT thrust acceleration, 2^-9 m/cs^2            */
    agc_dp_t  tbup;     /* TBUP burn-up time, 2^17 cs                     */
    agc_dp_t  ve;       /* VE exhaust velocity, 2^7 m/cs                  */
    agc_dp_t  tto;      /* TTO tail-off time, 2^17 cs                     */
    agc_dp_t  dv[4];    /* 1/DV0..1/DV3 (1/DV0 = MASS1)                   */
    agc_dp_t  xrange;   /* XRANGE cross range for N76, 2^29 m             */
    agc_vec_t v1s;      /* V1S velocity at TIG, 2^7 m/cs                  */
    agc_dp_t  yaw;      /* YAW revolutions                                */
    agc_dp_t  pitch;    /* PITCH revolutions                              */
    agc_sp_t  engoffdt; /* ENGOFFDT delay to engine cut-off, cs           */
    agc_dp_t  rdotv;    /* RDOT used by P12 (alias of RDOT)               */
} agc_lm_asc_t;

/* ---- lane state accessors (state.c) ----------------------------- */
agc_lm_pad_t *agc_lm_pad(void);
agc_lm_ldg_t *agc_lm_ldg(void);
agc_lm_thr_t *agc_lm_thr(void);
agc_lm_asc_t *agc_lm_asc(void);
void          agc_lm_landing_reset(void);

/* basic-language helpers shared by the lane (state.c) */
void           agc_lm_phaschng(uint16_t octal_code); /* PHASCHNG  */
void           agc_lm_fastchng(void);                /* FASTCHNG  */
agc_sp_t       agc_lm_time1(void);                   /* TIME1     */
agc_sp_t       agc_lm_dv(agc_dp_t dividend, agc_sp_t divisor); /* DV        */
agc_sp_t       agc_lm_mp_hi(agc_sp_t a, agc_sp_t b); /* MP, high  */
agc_sp_t       agc_lm_ad(agc_sp_t a, agc_sp_t b);    /* AD, wraps */
agc_lm_words_t agc_lm_words(agc_dp_t v);             /* sign-agreeing split */
agc_dp_t       agc_lm_words_value(agc_lm_words_t w); /* hi * 2^14 + lo    */
agc_lm_words_t agc_lm_das(agc_lm_words_t x, agc_lm_words_t y); /* DAS     */
agc_lm_words_t agc_lm_das_ovf(agc_lm_words_t x, agc_lm_words_t y,
                              int *ovf); /* DAS, A = net overflow    */
bool agc_lm_chan_bit_present(int chan, uint16_t bit); /* inverted ch30-33 */

/* ---- THROTTLE_CONTROL_ROUTINES (throttle.c) ----------------------------- */
agc_status_t agc_lm_throttle(void);
agc_status_t agc_lm_throttle_measured(agc_dp_t accel);
agc_status_t agc_lm_throttle_flatout(void);

/* ---- FINDCDUW--GUIDAP_INTERFACE (findcduw.c) ---------------------------- */
void agc_lm_initcduw(void);
/* FINDCDUW: thrust command (any length) in MPAC; window command is
 * agc_lm_ldg()->unwc_2. FINDCDUW -2: thrust command from UNFC/2
 * (agc_lm_ldg()->unfc_2).                                        */
agc_status_t agc_lm_findcduw(agc_interp_t *ip, agc_vec_t unfc_2);
agc_status_t agc_lm_findcduw_from_unfc(agc_interp_t *ip);
agc_sp_t     agc_lm_sparcsin(agc_sp_t x);    /* arg scale 1, angle 180 deg */
agc_sp_t     agc_lm_sparcsin_doubled(agc_sp_t x); /* SPARCSIN -1: arg scale 2  */
agc_sp_t     agc_lm_arctrgsp(agc_sp_t s, agc_sp_t c); /* 2's comp, pi    */
agc_sp_t     agc_lm_limitsub(agc_sp_t x, agc_sp_t limit);
uint16_t     agc_lm_onesto2s(agc_sp_t a); /* -> 15-bit 2's comp word    */

/* ---- LUNAR_LANDING_GUIDANCE_EQUATIONS (guidance.c) --------------
 * Job functions take the executive's job context (dispatch.c calls them);
 * display continuations are job ids.                                     */
#define AGC_LM_ROOT_MAX_DEGREE 5 /* ROOTPSRS erasables allow N <= 5      */
#define AGC_LM_ROOT_MAX_PASSES 8 /* "Klumpp says give up after 8 passes" */
agc_status_t agc_lm_rootpsrs(const agc_dp_t *coef, int degree, agc_dp_t guess,
                             agc_sp_t precroot, agc_dp_t *root,
                             agc_sp_t *root_hi, int *passes);
#define AGC_LM_ENTRY_GUILDRET 0 /* normal pass via LUNLAND/GUILDEN     */
#define AGC_LM_ENTRY_GUIDSUB  1 /* IGNALG passes after the first       */
#define AGC_LM_ENTRY_QGUIDSUB 2 /* ?GUIDSUB first IGNALG pass          */
#define AGC_LM_ENTRY_VERTGUID 3 /* straight to VERTGUID                */
agc_status_t agc_lm_job_lunland(agc_job_ctx_t *ctx); /* LUNLAND      */
agc_status_t agc_lm_guidpass(const agc_job_ctx_t *ctx, int entry);
agc_status_t agc_lm_start_approach_phase(void);               /* STARTP64     */
agc_status_t agc_lm_start_velocity_nulling(void);             /* P65START     */
agc_status_t agc_lm_ttfincr(agc_interp_t *ip);                /* TTFINCR      */
agc_status_t agc_lm_calcrgvg(agc_interp_t *ip);               /* CALCRGVG     */
agc_status_t agc_lm_rgvgcalc(agc_interp_t *ip);               /* RGVGCALC     */
agc_status_t agc_lm_ttf8cl(agc_interp_t *ip);                 /* TTF/8CL      */
agc_status_t agc_lm_quadguid(agc_interp_t *ip);               /* QUADGUID     */
agc_status_t agc_lm_afccalc(agc_interp_t *ip, agc_vec_t acg); /* AFCCALC1 */
agc_status_t agc_lm_cgcalc(agc_interp_t *ip);                 /* CGCALC       */
agc_status_t agc_lm_vertguid(const agc_job_ctx_t *ctx, agc_interp_t *ip);
agc_status_t agc_lm_steer_exit(const agc_job_ctx_t *ctx, agc_interp_t *ip);
agc_status_t agc_lm_dispexit(const agc_job_ctx_t *ctx, bool kill_group3);
agc_status_t agc_lm_job_braking_phase_display(agc_job_ctx_t *ctx); /* P63DISPS -2  */
agc_status_t agc_lm_job_approach_phase_display(agc_job_ctx_t *ctx); /* P64DISPS     */
agc_status_t agc_lm_job_approach_phase_proceed(agc_job_ctx_t *ctx); /* P64CEED      */
void         agc_lm_tdispset(void);                         /* TDISPSET     */
agc_sp_t     agc_lm_ttf8_high(void); /* CA TTF/8: the raw high word      */

/* ---- landing program flow (lunar_landing.c) ----------------------------- */
agc_status_t agc_lm_job_braking_phase(agc_job_ctx_t *ctx); /* P63LM        */
agc_status_t agc_lm_job_ignalg_prec(agc_job_ctx_t *ctx); /* IGNALG LEMPREC retry */
agc_status_t agc_lm_job_ignaloop(agc_job_ctx_t *ctx); /* IGNALOOP (INTSTALL retry) */
agc_status_t agc_lm_job_ignalgrt(agc_job_ctx_t *ctx); /* IGNALGRT     */
agc_status_t agc_lm_job_astnret(agc_job_ctx_t *ctx);  /* ASTNRET      */
agc_status_t agc_lm_job_braking_phase_burn_attitude(agc_job_ctx_t *ctx); /* P63SPOT2     */
agc_status_t agc_lm_job_braking_phase_antenna_check(agc_job_ctx_t *ctx); /* P63SPOT3     */
agc_status_t agc_lm_job_braking_phase_radar_init(agc_job_ctx_t *ctx); /* P63SPOT4     */
agc_status_t agc_lm_guildenstern(const agc_job_ctx_t *ctx,
                                 int                 *entry);  /* R13 */
agc_status_t agc_lm_redesig(agc_interp_t *ip); /* REDESIG      */
agc_status_t agc_lm_rate_of_descent_vertical(const agc_job_ctx_t *ctx); /* P66VERTA     */
agc_status_t agc_lm_manual_landing_vertical(const agc_job_ctx_t *ctx,
                                            agc_interp_t        *ip);
agc_status_t agc_lm_task_rodtask(void);              /* RODTASK      */
agc_status_t agc_lm_job_rodcomp(agc_job_ctx_t *ctx); /* RODCOMP      */
agc_status_t agc_lm_pitfall(void);                   /* PITFALL (RUPT10) */
agc_status_t agc_lm_task_redesmon(void);             /* REDESMON     */
agc_status_t agc_lm_descbits(void);                  /* DESCBITS (ch 16) */
agc_status_t agc_lm_job_landing_confirmation(agc_job_ctx_t *ctx); /* LANDJUNK     */
agc_status_t agc_lm_job_landing_confirmation_flash(agc_job_ctx_t *ctx); /* V06N43 flash */
agc_status_t agc_lm_job_landing_confirmation_proceed(
    agc_job_ctx_t *ctx); /* after V06N43 PROCEED */

/* ---- P70-P71 (aborts.c) ------------------------------------------------ */
agc_status_t agc_lm_task_abort_monitor(void);          /* R10,R11      */
agc_status_t agc_lm_abort_monitor_resume(void);        /* R10,R11A     */
agc_status_t agc_lm_job_dps_abort(agc_job_ctx_t *ctx); /* P70          */
agc_status_t agc_lm_job_aps_abort(agc_job_ctx_t *ctx); /* P71          */
agc_status_t agc_lm_job_dps_abort_commanded(agc_job_ctx_t *ctx); /* P70A restart */
agc_status_t agc_lm_job_aps_abort_commanded(agc_job_ctx_t *ctx);  /* P71A restart */
agc_status_t agc_lm_job_goabort(agc_job_ctx_t *ctx); /* GOABORT      */
agc_status_t agc_lm_job_abort_auto_control_check(agc_job_ctx_t *ctx); /* UPTHROT1 -3  */
agc_status_t agc_lm_job_abort_to_ascent_guidance(agc_job_ctx_t *ctx);    /* UPTHROT1     */
agc_status_t agc_lm_job_aps_abort_oldtime(agc_job_ctx_t *ctx); /* OLDTIME restart */
agc_sp_t     agc_lm_pipctr1(void);             /* PIPCTR1 for LANDISP */
void         agc_lm_tgocomp(agc_interp_t *ip); /* TGOCOMP      */
void         agc_lm_throtup(void);     /* THROTUP      */

/* ---- P12 and ASCENT_GUIDANCE (ascent.c) --------------------------------- */
agc_status_t agc_lm_guidinit(agc_interp_t *ip);
void         agc_lm_powered_ascent_init(agc_interp_t *ip);
void         agc_lm_comminit(agc_interp_t *ip);
agc_status_t agc_lm_job_powered_ascent(agc_job_ctx_t *ctx);
agc_status_t agc_lm_job_powered_ascent_proceed(agc_job_ctx_t *ctx);
agc_status_t agc_lm_job_newload(agc_job_ctx_t *ctx);
agc_status_t agc_lm_job_newload_proceed(agc_job_ctx_t *ctx);
agc_status_t agc_lm_job_powered_ascent_flash(agc_job_ctx_t *ctx); /* V06N33 flash */
agc_status_t agc_lm_job_powered_ascent_at_ignition(agc_job_ctx_t *ctx); /* P12LMB       */
agc_status_t agc_lm_powered_ascent_return(const agc_job_ctx_t *ctx,
                                          agc_interp_t        *ip);
agc_status_t agc_lm_job_powered_ascent_burn(agc_job_ctx_t *ctx); /* P12RET tail  */
agc_status_t agc_lm_job_atmag(agc_job_ctx_t *ctx);  /* ATMAG (AVGEXIT) */
agc_status_t agc_lm_ascent(const agc_job_ctx_t *ctx, agc_interp_t *ip);
agc_dp_t     agc_lm_ycomp(agc_interp_t *ip);
agc_dp_t     agc_lm_logsub(agc_interp_t *ip, agc_dp_t x);
agc_status_t agc_lm_task_ascent_engine_off(void);
agc_status_t agc_lm_job_cutoff(agc_job_ctx_t *ctx);
agc_status_t agc_lm_job_cutoff_display(agc_job_ctx_t *ctx); /* CUTOFF -5    */
agc_status_t agc_lm_job_cutoff_proceed(agc_job_ctx_t *ctx);
agc_status_t agc_lm_job_cutoff_trim_display(agc_job_ctx_t *ctx);
agc_status_t agc_lm_job_termasc(agc_job_ctx_t *ctx);

/* ---- LANDING_ANALOG_DISPLAYS (displays.c) ----------------------- */
agc_status_t agc_lm_landisp(void);
agc_status_t agc_lm_task_intlze(void);
agc_status_t agc_lm_task_speedrun_resume(void);
agc_status_t agc_lm_vmonitor(void);

/* ---- alarm codes raised by the landing programs (octal) ---------------------------- */
#define AGC_ALARM_00401 \
    ((agc_alarm_code_t) 00401) /* FINDCDUW: gimbal lock     */
#define AGC_ALARM_00402 \
    ((agc_alarm_code_t) 00402) /* FINDCDUW: NORMUNIT ovf    */
#define AGC_ALARM_01406 \
    ((agc_alarm_code_t) 01406) /* TTF/8 no convergence      */
#define AGC_ALARM_01410 \
    ((agc_alarm_code_t) 01410) /* overflow in guidance      */
#define AGC_ALARM_01412 \
    ((agc_alarm_code_t) 01412) /* IGNALG NIGNLOOP exhausted */

#endif /* AGC_LM_LANDING_H */
