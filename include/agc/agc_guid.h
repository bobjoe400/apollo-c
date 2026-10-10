/*
 * agc_guid.h -- powered-flight guidance interfaces (guid-powered agent).
 *
 * Covers POWERED_FLIGHT_SUBROUTINES (CDU trig, SM<->NB axis rotation,
 * SMCDURES, FLESHPOT), the average-G package of SERVICER207 (CM) and
 * SERVICER (LM), the P30/P31 targeting subroutines S30.1/S31.1 and the
 * P40-P47 steering subroutines S40.1, S40.2,3, S40.8, S40.9, S40.13, S41.1.
 *
 * Erasable memory owned by these routines lives in one static block in
 * src/guidance/guid_state.c and is reached through agc_guid_state(). Each
 * field carries the AGC name and scale ("2^n unit": stored fraction times
 * 2^n is the value in that unit). Vehicle-specific blocks hold the fields
 * that exist only in Comanche (csm) or Luminary (lm).
 *
 * Shared erasables owned by the state layer (REFSMMAT, CDUX/Y/Z, PIPAX/Y/Z,
 * DELV, VGTIG, TTOGO, TIG) are read through agc_state.h accessors; RN, VN,
 * PIPTIME are agc_sv_get(AGC_SV_THIS) (scaled 2^29 m, 2^7 m/cs, 2^28 cs
 * during average-G, whatever the primary body: AVETOMID/MIDTOAVE rescale).
 */
#ifndef AGC_GUID_H
#define AGC_GUID_H

#include <stdbool.h>
#include <stdint.h>
#include "agc_status.h"
#include "agc_word.h"
#include "agc_interp.h"
#include "agc_jobs.h"
#include "agc_state.h"

/* ======================================================================
 * POWERED_FLIGHT_SUBROUTINES
 * ====================================================================== */

/* Order of the CDUSPOT / SINCDU / COSCDU triples in the AGC: Y, Z, X.  */
#define AGC_TRIG_Y 0
#define AGC_TRIG_Z 1
#define AGC_TRIG_X 2

typedef struct agc_cdutrig {
    agc_dp_t cduspot[3];   /* CDUSPOT  Y Z X, 1's comp angle, rev, scale 2^0   */
    agc_sp_t cduraw[3];    /* CDUSPOT  Y Z X, 2's comp CDU counts as loaded    */
    agc_dp_t sincdu[3];    /* SINCDU   Y Z X, sin(angle)/2, scale 2^1          */
    agc_dp_t coscdu[3];    /* COSCDU   Y Z X, cos(angle)/2, scale 2^1          */
} agc_cdutrig_t;

/* AX*SR*T direction: A = +3 for NB->SM, -3 for SM->NB.                     */
typedef enum agc_axisrot_dir {
    AGC_AXISROT_SMNB = -3,
    AGC_AXISROT_NBSM = 3
} agc_axisrot_dir_t;

agc_dp_t     agc_guid_cdulogic(uint16_t cdu);                     /* CDULOGIC */
agc_status_t agc_guid_cdtrigs(agc_cdutrig_t *t, const uint16_t cdu_yzx[3]); /* CD*TR*GS */
agc_status_t agc_guid_cdutrig(agc_cdutrig_t *t);                  /* CDUTRIG(S) */
agc_status_t agc_guid_quictrig(agc_cdutrig_t *t, const uint16_t cdu_yzx[3]); /* QUICTRIG */
agc_status_t agc_guid_axisrot(const agc_cdutrig_t *t, agc_axisrot_dir_t dir,
                              agc_vec_t *v);                      /* AX*SR*T  */
agc_status_t agc_guid_smnb(const agc_cdutrig_t *t, agc_vec_t *v); /* *SMNB*   */
agc_status_t agc_guid_nbsm(const agc_cdutrig_t *t, agc_vec_t *v); /* *NBSM*   */
agc_status_t agc_guid_cdu_smnb(agc_cdutrig_t *t, agc_vec_t *v);   /* CDU*SMNB */
agc_status_t agc_guid_cdu_nbsm(agc_cdutrig_t *t, agc_vec_t *v);   /* CDU*NBSM */
agc_status_t agc_guid_trg_smnb(agc_cdutrig_t *t, const uint16_t cdu_yzx[3],
                               agc_vec_t *v);                     /* TRG*SMNB */
agc_status_t agc_guid_trg_nbsm(agc_cdutrig_t *t, const uint16_t cdu_yzx[3],
                               agc_vec_t *v);                     /* TRG*NBSM */
agc_status_t agc_guid_smcdures(agc_interp_t *ip, const agc_cdutrig_t *t,
                               agc_vec_t dthetasm, agc_vec_t *dcdu); /* SMCDURES (CM) */
agc_status_t agc_guid_fleshpot(const agc_cdutrig_t *t, agc_mat_t *xnb); /* FLESHPOT (LM) */

/* ======================================================================
 * Average-G (SERVICER207 / SERVICER)
 * ====================================================================== */

typedef struct agc_avg_g {
    agc_vec_t rn1;        /* RN1      ref, m,    scale 2^29                  */
    agc_vec_t vn1;        /* VN1      ref, m/cs, scale 2^7                   */
    agc_dp_t  piptime1;   /* PIPTIME1 cs, scale 2^28                         */
    agc_vec_t gdt_2;      /* GDT/2    ref, m/cs, scale 2^7                   */
    agc_vec_t gdt1_2;     /* GDT1/2   ref, m/cs, scale 2^7                   */
    agc_vec_t gobl_2;     /* GOBL/2   (CM) oblateness term, m/cs-ish 2^7     */
    agc_vec_t gobl1_2;    /* GOBL1/2  (CM) / UNITGOBL (LM), half-unit 2^1    */
    agc_vec_t unitr;      /* UNITR (CM) / UNIT/R/ (LM), half unit, 2^1       */
    agc_dp_t  rmag;       /* 36D after UNIT: |R|, scale as R                 */
    agc_dp_t  rmagsq;     /* 34D after UNIT: |R|^2, scale (R scale)*2        */
    agc_dp_t  resq_rsq;   /* 32D: (RE/R)^2 /2 (CM) or (RE/R)^2 (LM)          */
    agc_vec_t delvref;    /* DELVREF  ref, m/cs, scale 2^7                   */
    agc_dp_t  pguide;     /* PGUIDE   (LM) guidance period, cs, 2^28         */
    agc_dp_t  dvtotal;    /* DVTOTAL  m/cs, scale 2^7                        */
    agc_vec_t unitw;      /* UNITW    (CM) earth rotation axis, full unit 2^0 */
    agc_sp_t  pipage;     /* PIPAGE   +1 pips read, 0/-0 reading in progress */
    agc_sp_t  pipctr;     /* PIPCTR   (CM) ONMNITOR counter / (LM) R10 rate  */
    agc_job_id_t avgexit; /* AVGEXIT  2CADR of routine run after AVERAGEG    */
    uint16_t  cdutemp[3]; /* CDUTEMPX/Y/Z CDUs read at PIPTIME (X Y Z)       */
    agc_sp_t  pipatmp[3]; /* PIPATMPX/Y/Z (LM) raw pulses for the DAP        */
} agc_avg_g_t;

/* PIPASR (CM p.832, LM p.869): read and zero the PIPAs, set PIPTIME1.     */
agc_status_t agc_guid_pipasr(agc_avg_g_t *a, bool set_piptime1);
/* REREADAC tail (CM p.833, LM p.871): finish an interrupted PIPA read.
 * Returns AGC_STATUS_NOT_FOUND when the read had not started (go READACCS). */
agc_status_t agc_guid_rereadac_resume(agc_avg_g_t *a);
/* CALCGRAV (CM pp.835-836, LM p.880). r scaled 2^29 m.                    */
agc_status_t agc_guid_calcgrav(agc_interp_t *ip, agc_vehicle_t veh, bool moon,
                               agc_vec_t r, agc_avg_g_t *a);
/* CALCRVG (CM p.836, LM pp.880-881). delv: PIPA pulses scale 2^14.        */
agc_status_t agc_guid_calcrvg(agc_interp_t *ip, agc_vehicle_t veh, bool moon,
                              const agc_mat_t *refsmmat, agc_vec_t delv,
                              const agc_state_vector_t *sv, agc_avg_g_t *a);
/* Copy cycle: RN1,VN1,PIPTIME1,GDT1/2(,GOBL1/2) -> RN,VN,PIPTIME,GDT/2.   */
agc_status_t agc_guid_copycycle(agc_vehicle_t veh, bool with_gdt, agc_avg_g_t *a);

/* ======================================================================
 * Targeting and steering (P30-P47)
 * ====================================================================== */

/* Result of an Encke precision update (THISPREC/CSMPREC/LEMPREC).        */
typedef struct agc_guid_prec {
    agc_vec_t ratt;       /* RATT  m 2^29 (2^27 moon)                        */
    agc_vec_t vatt;       /* VATT  m/cs 2^7 (2^5 moon)                       */
    agc_dp_t  tat;        /* TAT   cs 2^28                                   */
    int       rtx1;       /* RTX1  -2 moon, -1 earth? (scale selector)       */
    int       rtx2;       /* RTX2  0 earth, 2 moon                           */
} agc_guid_prec_t;

typedef struct agc_burn {
    agc_vec_t rtig;       /* RTIG     ref m 2^29                             */
    agc_vec_t vtig;       /* VTIG     ref m/cs 2^7                           */
    agc_vec_t delvslv;    /* DELVSLV  LV coords m/cs 2^7 (crew input N81)    */
    agc_vec_t delvsin;    /* DELVSIN  ref m/cs 2^7                           */
    agc_dp_t  delvsab;    /* DELVSAB  |DELVSIN| m/cs 2^7                     */
    agc_dp_t  vgdisp;     /* VGDISP   m/cs 2^7                               */
    agc_dp_t  hapo;       /* HAPO     m 2^29                                 */
    agc_dp_t  hper;       /* HPER     m 2^29                                 */
    agc_dp_t  mga;        /* +MGA     rev 2^0                                */
    agc_vec_t delvlvc;    /* DELVLVC  LV coords m/cs 2^7                     */
    agc_vec_t rtarg;      /* RTARG    ref m 2^29                             */
    agc_dp_t  tpass4;     /* TPASS4   cs 2^28                                */
    agc_dp_t  dellt4;     /* DELLT4   cs 2^28                                */
    agc_vec_t ut;         /* UT       thrust direction, half unit 2^1        */
    agc_vec_t vg;         /* VG       m/cs 2^7                               */
    agc_vec_t vgprev;     /* VGPREV   m/cs 2^7                               */
    agc_vec_t bdt;        /* BDT      m/cs 2^7                               */
    agc_dp_t  csteer;     /* CSTEER   steer-law C, 2^1 (CM) / 2^2 (LM)       */
    agc_dp_t  ecsteer;    /* ECSTEER  pad load, 2^1                          */
    agc_dp_t  f;          /* F        thrust, 10^4 N (m.newton) 2^7          */
    agc_dp_t  weight_g;   /* WEIGHT/G mass kg 2^16 (CM: SP in high word)     */
    agc_dp_t  nomtig;     /* NOMTIG   cs 2^28                                */
    agc_dp_t  tgo;        /* TGO      cs 2^28                                */
    agc_vec_t pointvsm;   /* POINTVSM SM half unit 2^1                       */
    agc_vec_t scaxis;     /* SCAXIS   half unit 2^1                          */
    agc_vec_t xscref;     /* XSCREF   half unit 2^1                          */
    agc_vec_t yscref;     /* YSCREF                                          */
    agc_vec_t zscref;     /* ZSCREF                                          */
    agc_vec_t rinit;      /* RINIT    m 2^29                                 */
    agc_vec_t vinit;      /* VINIT    m/cs 2^7                               */
    agc_dp_t  tnit;       /* TNIT     cs 2^28                                */
    agc_dp_t  tnitprev;   /* TNITPREV cs 2^28                                */
    agc_vec_t vrprev;     /* VRPREV   m/cs 2^7                               */
    agc_vec_t viprime;    /* VIPRIME  m/cs 2^7                               */
    agc_vec_t delveet3;   /* DELVEET3 m/cs 2^7                               */
    agc_vec_t vgtemp;     /* VGTEMP   m/cs 2^7                               */
    agc_vec_t delvsum;    /* DELVSUM  m/cs 2^7 (CM)                          */
    agc_vec_t delvsump;   /* DELVSUMP m/cs 2^7 (CM)                          */
    agc_sp_t  nbrcycls;   /* NBRCYCLS (CM) count of 2 s cycles               */
    agc_sp_t  nbrcyclp;   /* NBRCYCLP (CM)                                   */
    agc_vec_t vgbody;     /* VGBODY   body/control m/cs 2^7 (N85)            */
    agc_vec_t delvimu;    /* DELVIMU  m/cs 2^7 (N83)                         */
    agc_vec_t delvctl;    /* DELVCTL  m/cs 2^7                               */
    agc_vec_t omegac;     /* OMEGAC   (CM) rate cmd, 1/(2 TVCDT) rev/s       */
    agc_dp_t  tevent;     /* TEVENT   cs (TIME2,TIME1)                       */
    agc_dp_t  unused0;    /* keeps the struct layout stable for additions    */
    int       rtx1;       /* RTX1 left by the last precision update          */
    int       rtx2;       /* RTX2                                            */
} agc_burn_t;

/* Comanche-only guidance erasables.                                         */
typedef struct agc_guid_csm {
    agc_vec_t xpipbuf;    /* X/Y/ZPIPBUF raw PIPA counts (high words)       */
    agc_vec_t xoldbuf;    /* X/Y/ZOLDBUF                                     */
    agc_dp_t  emdot;      /* EMDOT    SPS flow, kg/cs 2^3 (SP high word)     */
    agc_dp_t  etdecay;    /* ETDECAY  tail-off, cs 2^14 (SP high word)       */
    agc_sp_t  erepfrac[2];/* EREPFRAC LM-off, LM-on                          */
    agc_sp_t  repfrac;    /* REPFRAC  TVC CG tracking fraction               */
    agc_dp_t  kprimedt;   /* KPRIMEDT steering gain, pi/8 rad                */
    agc_sp_t  dvthresh;   /* DVTHRESH m/cs 2^-2                              */
    agc_dp_t  ek1val;     /* EK1VAL   N-s/100 2^23                           */
    agc_dp_t  ek2val;     /* EK2VAL   N-s/100 2^23                           */
    agc_dp_t  ek3val;     /* EK3VAL   N/10^4  2^9                            */
    agc_sp_t  nvword1;    /* NVWORD1  CLOCKJOB display selector              */
    agc_sp_t  mrkrtmp;    /* MRKRTMP  S40.6 test/trim selector               */
    agc_sp_t  cntr;       /* CNTR     S40.6 / TVC counter                    */
    agc_sp_t  vcntr;      /* VCNTR    TVC half-second counter                */
    agc_sp_t  vcntrtmp;   /* VCNTRTMP                                        */
    agc_sp_t  v97vcntr;   /* V97VCNTR                                        */
    agc_sp_t  masstmp;    /* MASSTMP  kg 2^16                                */
    agc_dp_t  p40tmp;     /* P40TMP   cs                                     */
    agc_sp_t  ogad;       /* OGAD     roll reference CDUX at ignition        */
    agc_dp_t  rsp_rrec;   /* RSP-RREC rev (DELRSPL)                          */
    agc_sp_t  tvcpitch;   /* TVCPITCH output counter image, 85.41"/bit       */
    agc_sp_t  tvcyaw;     /* TVCYAW                                          */
} agc_guid_csm_t;

/* Luminary-only guidance erasables.                                         */
typedef struct agc_guid_lm {
    agc_dp_t  mass;       /* MASS     kg 2^16                                */
    agc_dp_t  mass1;      /* MASS1    kg 2^16 (also DVCNTR1 temp)            */
    agc_sp_t  abdelv;     /* ABDELV   cm/s 2^14                              */
    agc_dp_t  abdvconv;   /* ABDVCONV m/cs 2^5                               */
    agc_sp_t  dvthrush;   /* DVTHRUSH pulses per 2 s                         */
    agc_sp_t  dvcntr;     /* DVCNTR                                          */
    agc_job_id_t outroute;/* OUTROUTE return after AVGEND                    */
    agc_vec_t r;          /* R        SM m 2^24                              */
    agc_vec_t v;          /* V        SM m/cs 2^7                            */
    agc_vec_t r1s;        /* R1S      SM m 2^24                              */
    agc_vec_t v1s;        /* V1S      SM m/cs 2^7                            */
    agc_vec_t g_csm;      /* G(CSM)   SM m/cs 2^7                            */
    agc_vec_t r_csm;      /* R(CSM)   SM m 2^24                              */
    agc_vec_t v_csm;      /* V(CSM)   SM m/cs 2^7                            */
    agc_vec_t wm;         /* WM       lunar rotation, SM                     */
    agc_dp_t  abvel;      /* ABVEL    m/cs 2^7                               */
    agc_dp_t  hdotdisp;   /* HDOTDISP m/cs 2^7                               */
    agc_vec_t delvs;      /* DELVS    lunar rotation correction m/cs 2^5     */
    agc_dp_t  hcalc;      /* HCALC    m 2^24                                 */
    agc_dp_t  hcalc1;     /* HCALC1   m 2^24                                 */
    agc_dp_t  land_r;     /* /LAND/   landing-site radius m 2^24             */
    agc_vec_t uhyp;       /* UHYP     half unit                              */
    agc_vec_t uhzp;       /* UHZP     downrange half unit (R10)              */
    agc_dp_t  altbits;    /* ALTBITS  altitude, R10 bit units 2^28           */
    agc_sp_t  runit[3];   /* RUNIT    SP copy of UNIT/R/ for R10             */
    agc_sp_t  daltrate;   /* DALTRATE R10 altitude-rate compensation         */
    agc_mat_t xnbpip;     /* XNBPIP   NB->SM at PIPTIME, half unit 2^1       */
    /* landing radar (LR) update, pp.874-897 */
    agc_vec_t hbeamnb;    /* HBEAMNB  range beam, NB half unit               */
    agc_vec_t vbeamnb[3]; /* VZBEAMNB VYBEAMNB VXBEAMNB (index = VSELECT/... )*/
    agc_dp_t  hmeas;      /* HMEAS    LR range counts (1.079 ft/bit)         */
    agc_dp_t  vmeas;      /* VMEAS    LR velocity counts                     */
    agc_dp_t  deltah;     /* DELTAH   m 2^24                                 */
    agc_dp_t  delqfix;    /* DELQFIX  m 2^24 reasonableness (50 ft nom.)     */
    agc_sp_t  lrlctr, lrrctr, lrmctr, lrsctr; /* LR good/bad counters        */
    agc_sp_t  vselect;    /* VSELECT  0 Z, 1 Y, 2 X beam                     */
    agc_sp_t  stilbadh;   /* STILBADH                                        */
    agc_sp_t  stilbadv;   /* STILBADV                                        */
    agc_dp_t  lrvtime;    /* LRVTIME  cs 2^28                                */
    uint16_t  lrcdu[3];   /* LRXCDU LRYCDU LRZCDU                            */
    agc_sp_t  piptem[3];  /* PIPTEM   raw PIPAs at RDGIMS                    */
    agc_vec_t gnur;       /* GNUR/GNUV updated R or V                        */
    agc_dp_t  radskal;    /* RADSKAL  doppler bias                           */
    agc_sp_t  skalskal;   /* SKALSKAL alt scale ratio (.2 nom)               */
    agc_sp_t  lrhmax;     /* LRHMAX   m 2^14                                 */
    agc_sp_t  lrwh;       /* LRWH     altitude weight                        */
    agc_sp_t  lrvmax;     /* LRVMAX                                          */
    agc_sp_t  lrvf;       /* LRVF                                            */
    agc_sp_t  lrwv[3];    /* LRWVZ LRWVY LRWVX                               */
    agc_sp_t  lrwvf[3];   /* LRWVFZ LRWVFY LRWVFX                            */
    agc_sp_t  lrwvff;     /* LRWVFF   P65-P67 weight                         */
    agc_sp_t  lralpha[2]; /* LRALPHA POS1, POS2 (X rotation)                 */
    agc_sp_t  lrbeta[2];  /* LRBETA  POS1, POS2 (Y rotation)                 */
    agc_sp_t  rpcrtime;   /* RPCRTIME repositioning criterion (time)         */
    agc_sp_t  rpcrtqsw;   /* RPCRTQSW repositioning criterion (angle)        */
    agc_sp_t  ttf_8;      /* TTF/8    (landing guidance, read here)          */
    agc_dp_t  mktime;     /* MKTIME   cs                                     */
    agc_sp_t  aig, amg, aog; /* gimbal angles at LR alt measurement          */
    /* P40-P47 / BURNBABY */
    agc_sp_t  which;      /* WHICH    selected ignition table (enum)         */
    agc_sp_t  dispdex;    /* DISPDEX  CLOKJOB display index (neg: active)    */
    agc_dp_t  savet_30;   /* SAVET-30 cs                                     */
    agc_dp_t  gobltime;   /* GOBLTIME nominal TIG for oblateness, cs 2^28    */
    agc_dp_t  tigsave;    /* TIGSAVE  cs 2^28                                */
    agc_dp_t  tigsavep;   /* TIGSAVEP cs 2^28                                */
    agc_dp_t  tnewa;      /* TNEWA    Lambert cycle, cs 2^28                 */
    agc_vec_t unfc_2;     /* UNFC/2   thrust command for FINDCDUW            */
    agc_dp_t  mdot;       /* MDOT     kg/cs 2^3                              */
    agc_dp_t  tdecay;     /* TDECAY   cs 2^28                                */
    agc_dp_t  vex;        /* VEX      m/cs 2^6 (2VEXHUST)                    */
    agc_sp_t  zoomtime;   /* ZOOMTIME cs (pad load)                          */
    agc_sp_t  ignaosq;    /* IGNAOSQ                                         */
    agc_sp_t  ignaosr;    /* IGNAOSR                                         */
    agc_sp_t  pittime;    /* PITTIME  cs                                     */
    agc_sp_t  rolltime;   /* ROLLTIME cs                                     */
    agc_sp_t  wchphase;   /* WCHPHASE                                        */
    agc_sp_t  flpass0;    /* FLPASS0                                         */
    agc_dp_t  rmag;       /* RMAG     (RASTEER1)                             */
    agc_dp_t  rtmag;      /* RTMAG    |RTARG|                                */
    agc_vec_t unitn;      /* UN       orbit normal (RASTEER1)                */
    agc_dp_t  geomsgn;    /* GEOMSGN  sign selector                          */
    agc_dp_t  muasteer;   /* MUASTEER                                        */
    agc_dp_t  mu_a;       /* MU/A                                            */
    agc_sp_t  trkmkcnt;   /* TRKMKCNT rendezvous mark counter                */
} agc_guid_lm_t;

typedef struct agc_guid {
    agc_cdutrig_t  trig;  /* CDUSPOT / SINCDU / COSCDU                       */
    agc_avg_g_t    avg;   /* average-G package                               */
    agc_burn_t     burn;  /* P30-P47 targeting and steering                  */
    agc_guid_csm_t csm;
    agc_guid_lm_t  lm;
} agc_guid_t;

agc_guid_t  *agc_guid_state(void);          /* the one guidance block      */
agc_status_t agc_guid_init(agc_vehicle_t veh); /* fresh start: zero + pad defaults */

/* ---- shared steering math (p40.c) ------------------------------------- */

/* S40.8 TGOCALC: TGO from VG, DELVREF, BDT, 2*Vex and tail-off (2^28 cs). */
typedef struct agc_tgo_in {
    agc_vec_t vg;         /* m/cs 2^7                                        */
    agc_vec_t delvref;    /* m/cs 2^7                                        */
    agc_vec_t bdt;        /* m/cs 2^7                                        */
    agc_dp_t  two_vex;    /* 2VEXHUST m/cs 2^7                               */
    agc_dp_t  tailoff;    /* ETDECAY/TDECAY cs 2^28                          */
} agc_tgo_in_t;
agc_status_t agc_guid_s40_8_tgo(agc_interp_t *ip, const agc_tgo_in_t *in,
                                agc_dp_t *tgo);

/* S40.8 XPRODUCT (CM): OMEGAC = KPRIMEDT * SMNB(REFSMMAT * (uVG x uDV')) */
typedef struct agc_xprod_in {
    agc_vec_t vg;         /* m/cs 2^7                                        */
    agc_vec_t delvref;    /* m/cs 2^7                                        */
    agc_vec_t bdt;        /* m/cs 2^7                                        */
    agc_dp_t  csteer;     /* 2^1                                             */
    agc_dp_t  kprimedt;   /* pi/8 rad                                        */
} agc_xprod_in_t;
agc_status_t agc_guid_s40_8_xproduct(agc_interp_t *ip, const agc_xprod_in_t *in,
                                     const agc_mat_t *refsmmat,
                                     const agc_cdutrig_t *trig, agc_vec_t *omegac);

agc_status_t agc_guid_s30_1(agc_interp_t *ip, agc_vehicle_t veh, agc_burn_t *b);
agc_status_t agc_guid_s31_1(agc_interp_t *ip, agc_burn_t *b);
agc_status_t agc_guid_s40_1(agc_interp_t *ip, agc_vehicle_t veh, agc_burn_t *b);
agc_status_t agc_guid_s40_1b_lm(agc_interp_t *ip, agc_burn_t *b);
agc_status_t agc_guid_again(agc_interp_t *ip, agc_burn_t *b);
agc_status_t agc_guid_calcut(agc_interp_t *ip, agc_burn_t *b);
agc_status_t agc_guid_s41_1(agc_interp_t *ip, agc_vehicle_t veh, agc_vec_t vg_ref,
                            agc_vec_t *vg_ctl);
agc_status_t agc_guid_s40_13_csm(agc_interp_t *ip, agc_burn_t *b, bool add_ullage,
                                 bool four_jets);
agc_status_t agc_guid_s40_13_lm(agc_interp_t *ip, agc_burn_t *b);

#endif /* AGC_GUID_H */
