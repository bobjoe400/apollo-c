/**********************************************************************
 * FILE NAME: guidance.h
 *
 * PURPOSE:
 *   The powered-flight guidance interfaces: POWERED_FLIGHT_SUBROUTINES
 *   (CDU trig, SM<->NB axis rotation on values and on VBUF words, SMCDURES,
 *   FLESHPOT), the average-G package of SERVICER207 (CM) and SERVICER (LM),
 *   the P30/P31 targeting subroutines S30.1/S31.1 and the P40-P47 steering
 *   subroutines S40.1, S40.2,3, S40.8, S40.9, S40.13, S41.1, and the burn
 *   program jobs and tasks of both vehicles.
 *
 * AGC SOURCE:
 *   Comanche055 POWERED_FLIGHT_SUBROUTINES.agc, SERVICER207.agc,
 *     P30-P37.agc, P40-P47.agc
 *   Luminary099 POWERED_FLIGHT_SUBROUTINES.agc, SERVICER.agc, P30-P37.agc,
 *     P40-P47.agc, BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc
 *
 * REQUIRES:
 *   the project headers it includes (self-contained)
 *
 * GLOBAL VARIABLES:
 *   none: the erasables owned by these routines live in one static block
 *   in src/guidance/guid_state.c, reached through agc_guid_state(). Each
 *   field carries the AGC name and scale ("2^n unit": stored fraction times
 *   2^n is the value in that unit).
 *
 * NOTES:
 *   Vehicle-specific blocks hold the fields that exist only in Comanche
 *   (csm) or Luminary (lm). VGPREV is the same erasable as VGTIG in both
 *   programs (agc_erase_vgtig); Comanche's VGTEMP is DELVEET3 (agc_rendz_t)
 *   and DV47TEMP is VG. Shared erasables owned by the state layer
 *   (REFSMMAT, CDUX/Y/Z, PIPAX/Y/Z, DELV, DELVSLV, VGTIG, TTOGO, TIG, TGO,
 *   TEVENT, RN, VN, PIPTIME, CSMMASS) are read through state.h accessors
 *   (RN 2^29 m, VN 2^7 m/cs, PIPTIME 2^28 cs during average-G, whatever the
 *   primary body: AVETOMID/MIDTOAVE rescale). The targeting erasables shared
 *   with the rendezvous programs (RTIG VTIG DELVSIN DELVSAB DELVLVC RTARG
 *   TPASS4 DELLT4 RINIT VINIT VIPRIME DELVEET3 +MGA RTX1 RTX2) live in
 *   agc_rendz_t (rendezvous.h, agc_rendz_erasable()), because P34/P35 and
 *   P30/P31 hand them to P40/P41. The CSM TVC/RCS DAP values P40 reads or
 *   writes (WEIGHT/G, EMDOT, EREPFRAC, PACTOFF, YACTOFF, OGAD, KPRIMEDT,
 *   OMEGAC, REPFRAC, VCNTR, V97VCNTR, MASSTMP, CNTR, TVCPHASE, ADB,
 *   THETADX) are in csm/dap.h.
 **********************************************************************/
#ifndef AGC_GUID_H
#define AGC_GUID_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/exec/jobs.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"

/* ======================================================================
 * POWERED_FLIGHT_SUBROUTINES
 * ====================================================================== */

/* Order of the CDUSPOT / SINCDU / COSCDU triples in the AGC: Y, Z, X.  */
#define AGC_TRIG_Y 0 /* inner gimbal (IGA) */
#define AGC_TRIG_Z 1 /* middle gimbal (MGA) */
#define AGC_TRIG_X 2 /* outer gimbal (OGA) */

typedef struct agc_cdutrig
{
    agc_dp_t cduspot[3]; /* CDUSPOT  Y Z X, 1's comp angle, rev, scale 2^0   */
    agc_sp_t cduraw[3];  /* CDUSPOT  Y Z X, 2's comp CDU counts as loaded; unscaled */
    agc_dp_t sincdu[3];  /* SINCDU   Y Z X, sin(angle)/2, scale 2^1          */
    agc_dp_t coscdu[3];  /* COSCDU   Y Z X, cos(angle)/2, scale 2^1          */
} agc_cdutrig_t;

/* AX*SR*T leaves VBUF as agc_vec_words_t (include/agc/core/word.h): the
 * DAS/DDOUBL passes leave words whose signs may disagree.                  */

/* AX*SR*T direction: A = +3 for NB->SM, -3 for SM->NB.                     */
typedef enum agc_axisrot_dir
{
    AGC_AXISROT_SMNB = -3,
    AGC_AXISROT_NBSM = 3
} agc_axisrot_dir_t;

agc_dp_t agc_guid_cdulogic(uint16_t cdu);                                                                  /* CDULOGIC */
void     agc_guid_cdtrigs(agc_cdutrig_t *t, const uint16_t cdu_yzx[3]);                                    /* CD*TR*GS */
void     agc_guid_cdutrig(agc_cdutrig_t *t);                                                               /* CDUTRIG(S) */
void     agc_guid_quictrig(agc_cdutrig_t *t, const uint16_t cdu_yzx[3]);                                   /* QUICTRIG */
void     agc_guid_axisrot(const agc_cdutrig_t *t, agc_axisrot_dir_t dir, agc_vec_t *v);                    /* AX*SR*T  */
void     agc_guid_axisrot_words(const agc_cdutrig_t *t, agc_axisrot_dir_t dir, agc_vec_words_t *v);        /* AX*SR*T on VBUF words */
void     agc_guid_smnb(const agc_cdutrig_t *t, agc_vec_t *v);                                              /* *SMNB*   */
void     agc_guid_nbsm(const agc_cdutrig_t *t, agc_vec_t *v);                                              /* *NBSM*   */
void     agc_guid_cdu_smnb(agc_cdutrig_t *t, agc_vec_t *v);                                                /* CDU*SMNB */
void     agc_guid_cdu_nbsm(agc_cdutrig_t *t, agc_vec_t *v);                                                /* CDU*NBSM */
void     agc_guid_trg_smnb(agc_cdutrig_t *t, const uint16_t cdu_yzx[3], agc_vec_t *v);                     /* TRG*SMNB */
void     agc_guid_trg_nbsm(agc_cdutrig_t *t, const uint16_t cdu_yzx[3], agc_vec_t *v);                     /* TRG*NBSM */
void     agc_guid_trg_nbsm_words(agc_cdutrig_t *t, const uint16_t cdu_yzx[3], agc_vec_words_t *v);         /* TRG*NBSM, VBUF words */
void     agc_guid_smcdures(agc_interp_t *ip, const agc_cdutrig_t *t, agc_vec_t dthetasm, agc_vec_t *dcdu); /* SMCDURES (CM) */
void     agc_guid_fleshpot(const agc_cdutrig_t *t, agc_mat_t *xnb);                                        /* FLESHPOT (LM) */

/* ======================================================================
 * Average-G (SERVICER207 / SERVICER)
 * ====================================================================== */

typedef struct agc_avg_g
{
    agc_vec_t    rn1;        /* RN1      ref, m,    scale 2^29                  */
    agc_vec_t    vn1;        /* VN1      ref, m/cs, scale 2^7                   */
    agc_dp_t     piptime1;   /* PIPTIME1 cs, scale 2^28                         */
    agc_vec_t    gdt_2;      /* GDT/2    ref, m/cs, scale 2^7                   */
    agc_vec_t    gdt1_2;     /* GDT1/2   ref, m/cs, scale 2^7                   */
    agc_vec_t    gobl_2;     /* GOBL/2   (CM) oblateness term, m/cs-ish 2^7     */
    agc_vec_t    gobl1_2;    /* GOBL1/2  (CM) / UNITGOBL (LM), half-unit 2^1    */
    agc_vec_t    unitr;      /* UNITR (CM) / UNIT/R/ (LM), half unit, 2^1       */
    agc_dp_t     rmag;       /* 36D after UNIT: |R|, scale as R                 */
    agc_dp_t     rmagsq;     /* 34D after UNIT: |R|^2, scale (R scale)*2        */
    agc_dp_t     resq_rsq;   /* 32D: (RE/R)^2 /2 (CM) or (RE/R)^2 (LM); scale 2^0 */
    agc_vec_t    delvref;    /* DELVREF  ref, m/cs, scale 2^7                   */
    agc_dp_t     pguide;     /* PGUIDE   (LM) guidance period, cs, 2^28         */
    agc_dp_t     dvtotal;    /* DVTOTAL  m/cs, scale 2^7                        */
    agc_vec_t    unitw;      /* UNITW    (CM) earth rotation axis, full unit 2^0 */
    agc_sp_t     pipage;     /* PIPAGE   +1 pips read, 0/-0 reading in progress; unscaled */
    agc_sp_t     pipctr;     /* PIPCTR   (CM) ONMNITOR counter / (LM) R10 rate; unscaled */
    agc_job_id_t avgexit;    /* AVGEXIT  2CADR of routine run after AVERAGEG    */
    uint16_t     cdutemp[3]; /* CDUTEMPX/Y/Z CDUs read at PIPTIME (X Y Z)       */
    agc_sp_t     pipatmp[3]; /* PIPATMPX/Y/Z (LM) raw pulses for the DAP; unscaled */
} agc_avg_g_t;

/* PIPASR (CM p.832, LM p.869): read and zero the PIPAs into the high words
 * of DELV (pulses, i.e. DP scale 2^14 pulses), and, unless entered at
 * PIPASR +3 (LM BIBIBIAS), store TIME2,TIME1 in PIPTIME1 (cs 2^28).       */
void agc_guid_pipasr(agc_avg_g_t *a, bool set_piptime1);

/* REREADAC decision (CM p.833, LM p.871): AGC_STATUS_NOT_FOUND when the
 * last completed reading is already marked (PIPAGE > 0: go to READACCS),
 * AGC_STATUS_OK when this cycle's reading was taken (go to PIPSDONE).      */
agc_status_t agc_guid_rereadac_resume(agc_avg_g_t *a);

/* CALCGRAV (CM pp.835-836, LM p.880). r: ref m 2^29. moon: RTX2 = 2.      */
agc_status_t agc_guid_calcgrav(agc_interp_t *ip, agc_vehicle_t veh, bool moon, agc_vec_t r, agc_avg_g_t *a);

/* CALCRVG (CM p.836, LM pp.880-881). delv: SM PIPA pulses, scale 2^14;
 * sv: RN (2^29 m) VN (2^7 m/cs). Leaves DELVREF RN1 VN1 GDT1/2.           */
agc_status_t agc_guid_calcrvg(agc_interp_t *ip, agc_vehicle_t veh, bool moon, const agc_mat_t *refsmmat, agc_vec_t delv, const agc_state_vector_t *sv, agc_avg_g_t *a);

/* Copy cycles of the average-G package (GENTRAN in Comanche, COPYCYC in
 * Luminary): which words of RN1.. go to RN.. under INHINT.                */
typedef enum agc_copy_kind
{
    AGC_COPY_RVT = 0, /* RN VN PIPTIME          (CM NORMLIZE, 14 words)  */
    AGC_COPY_RVTG,    /* + GDT/2 (CM: + GOBL/2) (CM AVERAGEG 26, LM NORMLIZ2 20) */
    AGC_COPY_ALL      /* LM COPYCYC: + MASS (22 words)                   */
} agc_copy_kind_t;

void agc_guid_copycycle(agc_vehicle_t veh, agc_copy_kind_t kind);

/* ======================================================================
 * Targeting and steering (P30-P47)
 * ====================================================================== */

/* P30-P47 erasables not shared with the rendezvous programs.               */
typedef struct agc_burn
{
    agc_dp_t  vgdisp;   /* VGDISP   |VG| m/cs, scale 2^7                   */
    agc_dp_t  hapo;     /* HAPO     apogee altitude m, scale 2^29          */
    agc_dp_t  hper;     /* HPER     perigee altitude m, scale 2^29         */
    agc_vec_t ut;       /* UT       thrust direction, half unit, scale 2^1 */
    agc_vec_t vg;       /* VG       velocity to be gained m/cs, 2^7        */
    agc_vec_t bdt;      /* BDT      m/cs, scale 2^7                        */
    agc_dp_t  csteer;   /* CSTEER   steering-law C, 2^1 (CM) / 2^2 (LM)    */
    agc_dp_t  ecsteer;  /* ECSTEER  pad load (CM), scale 2^1               */
    agc_dp_t  f;        /* F        thrust, m.newton (10^4 N), scale 2^7   */
    agc_dp_t  weight_g; /* WEIGHT/G kg 2^16: copied by the P40 programs from the CSM DAP's WEIGHT/G or the LM's MASS before S40.1 / S40.13 (keeps guidance below the DAPs) */
    agc_dp_t  nomtig;   /* NOMTIG   cs, scale 2^28 (CM)                    */
    agc_vec_t pointvsm; /* POINTVSM SM, half unit 2^1                      */
    agc_vec_t scaxis;   /* SCAXIS   S/C axes, half unit 2^1                */
    agc_vec_t xscref;   /* XSCREF   ref, half unit 2^1                     */
    agc_vec_t yscref;   /* YSCREF; half unit (2^1) */
    agc_vec_t zscref;   /* ZSCREF; half unit (2^1) */
    agc_dp_t  tnit;     /* TNIT     cs, scale 2^28 (CM)                    */
    agc_dp_t  tnitprev; /* TNITPREV cs, scale 2^28 (CM)                    */
    agc_vec_t vrprev;   /* VRPREV   m/cs, scale 2^7 (CM)                   */
    agc_vec_t delvsum;  /* DELVSUM  m/cs, scale 2^7 (CM)                   */
    agc_vec_t delvsump; /* DELVSUMP m/cs, scale 2^7 (CM)                   */
    agc_sp_t  nbrcycls; /* NBRCYCLS (CM) 2-s cycles since S40.9; -1 = none; unscaled */
    agc_sp_t  nbrcyclp; /* NBRCYCLP (CM); unscaled */
    agc_vec_t vgbody;   /* VGBODY   control/body axes m/cs 2^7 (N85)       */
    agc_vec_t delvimu;  /* DELVIMU  m/cs 2^7 (N83)                         */
    agc_vec_t delvctl;  /* DELVCTL  m/cs 2^7                               */
} agc_burn_t;

/* Comanche-only guidance erasables.                                         */
typedef struct agc_guid_csm
{
    agc_sp_t xpipbuf[3];       /* X/Y/ZPIPBUF PIPA counts read by QUIKREAD/READACCS; unscaled */
    agc_sp_t xoldbuf[3];       /* X/Y/ZOLDBUF previous readings; unscaled */
    agc_sp_t etdecay;          /* ETDECAY  SPS tail-off, cs, scale 2^14 (PL)      */
    agc_sp_t fang;             /* FANG     SPS thrust for impulsive burns, m.newton 2^7 (PL) */
    agc_sp_t dvthresh;         /* DVTHRESH low-thrust threshold, m/cs 2^-2 (PL)   */
    agc_dp_t ek1val;           /* EK1VAL   1-s SPS impulse, N-s/100, 2^23 (PL)    */
    agc_dp_t ek2val;           /* EK2VAL   N-s/100, scale 2^23 (PL)               */
    agc_dp_t ek3val;           /* EK3VAL   SPS thrust, N/10^4, scale 2^9 (PL)     */
    agc_sp_t nvword1;          /* NVWORD1  CLOCKJOB: +VN display, 0 none, -BIT9 V99 flash, -1 V97 flash (the AGC's -0, see note); unscaled */
    agc_sp_t mrkrtmp;          /* MRKRTMP  S40.6: + test and trim, - trim only; unscaled */
    agc_sp_t gimbal_test_step; /* S40.6 gimbal-drive-test step (TEMPR60 return point); unscaled */
    agc_sp_t updcaller;        /* caller of UPDATEVG (QTEMP1): 1 STEERING, 2 CALCN85; unscaled */
    agc_dp_t rsp_rrec;         /* RSP-RREC splash range error, rev (DELRSPL); scale 2^0 */
} agc_guid_csm_t;

/* Luminary-only guidance erasables. /LAND/, WM, TTF/8, UNFC/2, UNWC/2,
 * DELTAH, HCALC1, WCHPHASE, FLPASS0 are guid-landing's (agc_lm_ldg()).      */
typedef struct agc_guid_lm
{
    agc_dp_t     mass;     /* MASS = WEIGHT/G, kg, scale 2^16                 */
    agc_dp_t     mass1;    /* MASS1 kg 2^16 (also DVCNTR1 temporary)          */
    agc_sp_t     abdelv;   /* ABDELV   |DELV| cm/s, scale 2^14                */
    agc_dp_t     abdvconv; /* ABDVCONV |DELV| m/cs, scale 2^5                 */
    agc_sp_t     dvthrush; /* DVTHRUSH DV monitor threshold, PIPA pulses; unscaled */
    agc_sp_t     dvcntr;   /* DVCNTR   low-thrust passes left; unscaled */
    agc_job_id_t outroute; /* OUTROUTE where AVGEND returns                   */
    agc_vec_t    r;        /* R        SM, m, scale 2^24                      */
    agc_vec_t    v;        /* V        SM, m/cs, scale 2^7                    */
    agc_vec_t    r1s;      /* R1S      SM, m, scale 2^24                      */
    agc_vec_t    v1s;      /* V1S      SM, m/cs, scale 2^7                    */
    agc_vec_t    g_csm;    /* G(CSM)   SM, m/cs, scale 2^7                    */
    agc_vec_t    r_csm;    /* R(CSM)   SM, m, scale 2^24                      */
    agc_vec_t    v_csm;    /* V(CSM)   SM, m/cs, scale 2^7                    */
    agc_dp_t     abvel;    /* ABVEL    speed m/cs, scale 2^7                  */
    agc_dp_t     hdotdisp; /* HDOTDISP altitude rate m/cs, scale 2^7          */
    agc_vec_t    delvs;    /* DELVS    lunar rotation correction m/cs 2^5     */
    agc_dp_t     hcalc;    /* HCALC    altitude m, scale 2^24                 */
    agc_vec_t    uhyp;     /* UHYP     cross-range half unit (R10); (2^1) */
    agc_vec_t    uhzp;     /* UHZP     downrange half unit (R10); (2^1) */
    agc_dp_t     altbits;  /* ALTBITS  altitude, R10 bit units; 2.34 ft/bit, unscaled */
    agc_sp_t     runit[3]; /* RUNIT    SP copy of UNIT/R/ for R10; scale 2^1 */
    agc_sp_t     daltrate; /* DALTRATE R10 altitude-rate compensation; R10 bits, unscaled */
    agc_mat_t    xnbpip;   /* XNBPIP   NB->SM at PIPTIME, half unit 2^1       */

    /* landing radar (LR) update, pp.874-897 */
    agc_vec_t hbeamnb;    /* HBEAMNB  range beam, NB, half unit; (2^1) */
    agc_vec_t vbeamnb[3]; /* VZBEAMNB VYBEAMNB VXBEAMNB, NB half unit; (2^1) */
    agc_dp_t  hmeas;      /* HMEAS    LR range, 1.079 ft/bit; unscaled */
    agc_dp_t  vmeas;      /* VMEAS    LR velocity counts; unscaled */
    agc_dp_t  delqfix;    /* DELQFIX  m, scale 2^24 (50 ft nominal) (PL)     */
    agc_sp_t  lrlctr;     /* LRLCTR   altitude (H) passes counted (INCR); unscaled */
    agc_sp_t  lrrctr;     /* LRRCTR   LRLCTR at the last good H (NORLITE); unscaled */
    agc_sp_t  lrmctr;     /* LRMCTR   velocity (V) passes counted (INCR); unscaled */
    agc_sp_t  lrsctr;     /* LRSCTR   LRMCTR at the last good V (NOLITE); unscaled */
    agc_sp_t  vselect;    /* VSELECT  0 Z, 1 Y, 2 X beam; unscaled */
    agc_sp_t  stilbadh;   /* STILBADH 2-s passes to wait after bad H; unscaled */
    agc_sp_t  stilbadv;   /* STILBADV; 2-s passes to wait after bad V, unscaled */
    agc_dp_t  lrvtime;    /* LRVTIME  cs, scale 2^28                         */
    uint16_t  lrcdu[3];   /* LRXCDU LRYCDU LRZCDU                            */
    agc_sp_t  piptem[3];  /* PIPTEM   raw PIPAs at RDGIMS; unscaled */
    agc_vec_t gnur;       /* GNUR = GNUV updated R1S or V1S; scale of R1S (m 2^24) or V1S (m/cs 2^7) */
    agc_dp_t  radskal;    /* RADSKAL  doppler scale (PL)                     */
    agc_sp_t  skalskal;   /* SKALSKAL low/high altitude scale ratio (PL)     */
    agc_sp_t  lrhmax;     /* LRHMAX   m, scale 2^14 (PL)                     */
    agc_sp_t  lrwh;       /* LRWH     altitude weight (PL); scale 2^0 */
    agc_sp_t  lrvmax;     /* LRVMAX   m/cs 2^7 (PL)                          */
    agc_sp_t  lrvf;       /* LRVF     m/cs 2^7 (PL)                          */
    agc_sp_t  lrwv[3];    /* LRWVZ LRWVY LRWVX (PL); scale 2^0 */
    agc_sp_t  lrwvf[3];   /* LRWVFZ LRWVFY LRWVFX (PL); scale 2^0 */
    agc_sp_t  lrwvff;     /* LRWVFF   P65-P67 weight (PL); scale 2^0 */
    agc_sp_t  lralpha[2]; /* LRALPHA POS1, POS2 (X rotation) (PL); scale 2^0 half revolutions */
    agc_sp_t  lrbeta[2];  /* LRBETA  POS1, POS2 (Y rotation) (PL); scale 2^0 half revolutions */
    agc_sp_t  rpcrtime;   /* RPCRTIME repositioning criterion, TTF/8 units; unscaled */
    agc_sp_t  rpcrtqsw;   /* RPCRTQSW repositioning criterion (angle); scale 2^0 */
    agc_dp_t  mktime;     /* MKTIME   cs, scale 2^28                         */
    agc_sp_t  aig;        /* AIG      inner gimbal at the LR range mark; CDU counts, unscaled */
    agc_sp_t  amg;        /* AMG      middle gimbal at the LR range mark; CDU counts, unscaled */
    agc_sp_t  aog;        /* AOG      outer gimbal at the LR range mark; CDU counts, unscaled */

    /* P40-P47 / BURNBABY */
    agc_sp_t     which;                /* WHICH    ignition table (agc_lm_which_t); unscaled */
    agc_sp_t     dispdex;              /* DISPDEX  CLOKJOB display: >0 none, <0 -index; unscaled */
    agc_dp_t     gobltime;             /* GOBLTIME nominal TIG for oblateness, cs 2^28    */
    agc_dp_t     tigsave;              /* TIGSAVE  cs, scale 2^28                         */
    agc_dp_t     tigsavep;             /* TIGSAVEP cs, scale 2^28                         */
    agc_dp_t     tnewa;                /* TNEWA    Lambert cycle, cs 2^28                 */
    agc_dp_t     mdot;                 /* MDOT     kg/cs, scale 2^3                       */
    agc_dp_t     tdecay;               /* TDECAY   cs, scale 2^28                         */
    agc_dp_t     vex;                  /* VEX      exhaust velocity, m/cs 2^6             */
    agc_sp_t     ignaosq;              /* IGNAOSQ  (PL); upper word of AOSQ, scale of AOSQ */
    agc_sp_t     ignaosr;              /* IGNAOSR  (PL); upper word of AOSR, scale of AOSR */
    agc_sp_t     pittime;              /* PITTIME  cs; unscaled */
    agc_sp_t     rolltime;             /* ROLLTIME cs; unscaled */
    agc_dp_t     rmag;                 /* RMAG     |RN| m (RASTEER1 working scale)        */
    agc_vec_t    ic;                   /* IC       unit(RTARG - RN) (= DELVSIN storage); half unit (2^1) */
    agc_dp_t     r1c;                  /* R1C      RMAG * |C|, normalised; scale Rb^2 2^-(k1+k2) */
    agc_dp_t     ss;                   /* SS       (R1 + R2 + C)/2; scale Rb */
    agc_sp_t     muscale;              /* MUSCALE  normalisation shift                    */
    agc_job_id_t auto_control_recycle; /* TEMPR60: where P40AUTO returns (BANKJUMP)   */
} agc_guid_lm_t;

typedef struct agc_guid
{
    agc_cdutrig_t  trig; /* CDUSPOT / SINCDU / COSCDU                       */
    agc_avg_g_t    avg;  /* average-G package                               */
    agc_burn_t     burn; /* P30-P47 targeting and steering                  */
    agc_guid_csm_t csm;  /* Comanche-only erasables */
    agc_guid_lm_t  lm;   /* Luminary-only erasables */
} agc_guid_t;

agc_guid_t *agc_guid_state(void); /* the one guidance block    */

/* The job that resumes after an INTSTALL wait inside a targeting
 * subroutine that has no job context (S30.1, S31.1, AGAIN, S40.1): set by
 * the job that calls it (guid_state.c).                                   */
void         agc_guid_nav_set_resume(agc_job_id_t job);
agc_job_id_t agc_guid_nav_resume(void);

/* Store the MIDTOAV1/2 output where the AGC leaves it: RN1 VN1 PIPTIME1
 * in the average-G block, RTX1 RTX2 in the rendezvous body block.          */
void agc_guid_store_midtoav(const agc_integ_midtoav_out_t *o);
void agc_guid_init(agc_vehicle_t veh); /* fresh start: zero + pad loads */

/* ---- shared targeting (external_delta_v.c) ----------------------------------------- */
agc_status_t agc_guid_external_delta_v_vectors(agc_interp_t *ip, agc_vehicle_t veh); /* S30.1 */
agc_status_t agc_guid_lambert_aimpoint_vectors(agc_interp_t *ip);                    /* S31.1 (CM) */

/* DELRSPL (CM): tff = the TFF debris of the last SR30.1, tff_cs = TFF
 * (cs 2^28, negative); sets RSP-RREC and does INTWAKE0.                  */
agc_status_t agc_guid_delrspl(agc_interp_t *ip, const agc_tff_t *tff, agc_dp_t tff_cs);

agc_status_t agc_guid_midgim(agc_interp_t *ip, agc_vec_t dv); /* MIDGIM (CM) */

/* ---- shared steering math (burn_steering.c) ------------------------------------- */
agc_status_t agc_guid_initial_velocity_to_gain(agc_interp_t *ip, agc_vehicle_t veh); /* S40.1 */
agc_status_t agc_guid_again(agc_interp_t *ip, agc_dp_t tdec1);                       /* AGAIN (CM) */
agc_status_t agc_guid_calcut(agc_interp_t *ip);                                      /* CALCUT (CM) */

/* S40.2,3 (CM): rcs = ENG2FLAG; pactoff/yactoff: trims, 85.41"/bit.      */
agc_status_t agc_guid_preferred_thrust_attitude_csm(agc_interp_t *ip, bool rcs, agc_sp_t pactoff, agc_sp_t yactoff);
agc_status_t agc_guid_preferred_thrust_attitude_lm(agc_interp_t *ip); /* S40.2,3 (LM) */

/* S41.1: VG (ref, 2^7) to control axes (CM, *10 and QUADROT) or LM body.  */
void agc_guid_reference_to_body(agc_interp_t *ip, agc_vehicle_t veh, agc_vec_t vg_ref, agc_vec_t *vg_ctl);

/* S40.8 TGOCALC: TGO (cs 2^28) from VG, DELVREF, BDT, 2*Vex, tail-off.    */
typedef struct agc_tgo_in
{
    agc_vec_t vg;      /* m/cs 2^7                                        */
    agc_vec_t delvg;   /* DELVREF - BDT (CM) / -DELVREF (LM), m/cs 2^7    */
    agc_dp_t  vex;     /* 2VEXHUST m/cs 2^7 (CM) or VEX m/cs 2^6 (LM)     */
    agc_dp_t  tailoff; /* -ETDECAY 2^28 (CM) or TDECAY 2^28 (LM), cs      */
} agc_tgo_in_t;

agc_status_t agc_guid_steering_tgo_csm(agc_interp_t *ip, const agc_tgo_in_t *in, agc_dp_t *tgo);
agc_status_t agc_guid_steering_tgo_lm(agc_interp_t *ip, const agc_tgo_in_t *in, agc_dp_t *tgo);

/* S40.8 XPRODUCT (CM): OMEGAC = KPRIMEDT * SMNB(REFSMMAT (uDV' x uVG)).    */
typedef struct agc_xprod_in
{
    agc_vec_t vg;       /* m/cs 2^7                                        */
    agc_vec_t delvref;  /* m/cs 2^7                                        */
    agc_vec_t bdt;      /* m/cs 2^7                                        */
    agc_dp_t  csteer;   /* 2^1                                             */
    agc_dp_t  kprimedt; /* pi/8 rad; scale 2^0 (KPRIMEDT) */
} agc_xprod_in_t;

void agc_guid_steering_cross_product(agc_interp_t         *ip,
                                     const agc_xprod_in_t *in,
                                     const agc_mat_t      *refsmmat,
                                     const agc_cdutrig_t  *trig,
                                     agc_vec_t            *omegac);

/* S40.13 TIMEBURN (CM). ullage: +X translation on; four_jets: NJETSFLG
 * clear; emdot: SPS flow kg/cs 2^3; fang: SPS thrust m.newton 2^7 (SP).  */
agc_status_t agc_guid_timeburn_csm(agc_interp_t *ip, bool ullage, bool four_jets, agc_sp_t emdot, agc_sp_t fang);

/* ---- average-G jobs and tasks: Comanche (src/csm/burns/servicer.c) ------ */
void         agc_guid_preread1_csm(void);              /* PREREAD1 (P11 liftoff)  */
agc_status_t agc_task_preread_csm(void);               /* PREREAD     */
agc_status_t agc_task_readaccs_csm(void);              /* READACCS    */
agc_status_t agc_task_rereadac_csm(void);              /* REREADAC    */
agc_status_t agc_task_redo5_5_csm(void);               /* REDO5.5     */
agc_status_t agc_task_chekaveg(void);                  /* CHEKAVEG    */
agc_status_t agc_task_onmnitor(void);                  /* ONMNITOR    */
agc_status_t agc_task_quikread(void);                  /* QUIKREAD    */
agc_status_t agc_job_normlize_csm(void);               /* NORMLIZE    */
agc_status_t agc_job_servicer_csm(void);               /* SERVICER    */
agc_status_t agc_job_servexit(void);                   /* SERVEXIT (both) */
agc_status_t agc_csm_avgexit(void);                    /* dispatch on AVGEXIT */
agc_status_t agc_job_serv_dvtotup_csm(void);           /* DVTOTUP restart */
agc_status_t agc_job_serv_averageg_csm(void);          /* AVERAGEG restart */
agc_status_t agc_job_serv_copy_csm(void);              /* GENTRAN restart  */
agc_status_t agc_job_avgend_csm(agc_job_ctx_t *ctx);   /* AVGEND      */
agc_status_t agc_job_avetomid_csm(agc_job_ctx_t *ctx); /* AVGEND 2.2 restart */

/* ---- average-G jobs and tasks: Luminary (src/lm/burns/servicer/) -------- */
agc_status_t agc_task_preread_lm(void);               /* PREREAD     */
agc_status_t agc_task_redo5_5_lm(void);               /* REDO5.5     */
agc_status_t agc_task_bibibias(void);                 /* BIBIBIAS    */
agc_status_t agc_task_readaccs_lm(void);              /* READACCS    */
agc_status_t agc_task_rereadac_lm(void);              /* REREADAC    */
agc_status_t agc_job_normlize_lm(void);               /* NORMLIZE    */
agc_status_t agc_job_servicer_lm(agc_job_ctx_t *ctx); /* SERVICER    */
void         agc_job_servidle(void);                  /* SERVIDLE    */
void         agc_lm_servidle(void);                   /* SERVIDLE (STRTIDLE)  */
agc_status_t agc_lm_avgexit(void);                    /* dispatch on AVGEXIT */
agc_status_t agc_job_avgend_lm(agc_job_ctx_t *ctx);   /* AVGEND      */
agc_status_t agc_job_avetomid_lm(agc_job_ctx_t *ctx); /* AVGEND 2.2 restart */

/* Restart points set by QUIKFAZ5 (AGC_JOB_SERV_*): one entry, by ctx->id. */
agc_status_t agc_job_serv_resume_lm(agc_job_ctx_t *ctx);
void         agc_lm_munrvg(agc_interp_t *ip);               /* MUNRVG      */
agc_status_t agc_lm_rvboth(agc_interp_t *ip);               /* RVBOTH      */
agc_vec_t    agc_lm_mungrav(agc_interp_t *ip, agc_vec_t r); /* MUNGRAV (r SM 2^24) -> GDT1/2 */
void         agc_lm_altchk(agc_sp_t index, bool *below);    /* ALTCHK      */
void         agc_lm_tmptospt(void);                         /* TMPTOSPT    */
agc_status_t agc_lm_lrhtask(void);                          /* LRHTASK     */
void         agc_lm_setpos1(void);                          /* SETPOS1     */
agc_status_t agc_lm_setpos2(void);                          /* SETPOS2     */
void         agc_task_rdgims(void);                         /* RDGIMS      */
agc_status_t agc_job_lrvjob(void);                          /* LRVJOB      */
void         agc_job_lrvjob_done(void);                     /* after RADSTALL */
agc_status_t agc_job_lrhjob(void);                          /* LRHJOB      */
agc_status_t agc_job_lrhjob_done(void);                     /* after RADSTALL */
agc_status_t agc_job_higatjob(agc_job_ctx_t *ctx);          /* HIGATJOB    */
agc_status_t agc_job_higatjob_done(agc_job_ctx_t *ctx);     /* after RADSTALL */
agc_status_t agc_job_posalarm_pro(agc_job_ctx_t *ctx);      /* POSALARM +3 */

/* ---- targeting programs (src/guidance/external_delta_v.c) -------------------------- */
agc_status_t agc_job_external_delta_v(agc_job_ctx_t *ctx);                    /* P30 (both)  */
agc_status_t agc_job_external_delta_v_display_dv(agc_job_ctx_t *ctx);         /* after V06N33 PRO */
agc_status_t agc_job_external_delta_v_compute(agc_job_ctx_t *ctx);            /* after V06N81 PRO */
agc_status_t agc_job_external_delta_v_display_parameters(agc_job_ctx_t *ctx); /* PARAM30     */
agc_status_t agc_job_external_delta_v_accept(void);                           /* LM PARAM30 PROCEED */
agc_status_t agc_job_external_delta_v_reftest(agc_job_ctx_t *ctx);            /* REFTEST (CM) */
agc_status_t agc_job_marks_tfi_mga_done(void);                                /* END45 (CM)  */
agc_status_t agc_job_marks_tfi_mga_display(agc_job_ctx_t *ctx);               /* DISP45 (CM) */
agc_status_t agc_job_lambert_aimpoint_guidance_csm(agc_job_ctx_t *ctx);       /* P31 (CM)    */
agc_status_t agc_job_lambert_aimpoint_compute_csm(agc_job_ctx_t *ctx);        /* after V06N33 PRO */
agc_status_t agc_job_lambert_aimpoint_display_dv(agc_job_ctx_t *ctx);         /* after V06N81 PRO */

/* ---- CSM P40-P47 (src/csm/burns/sps/) ---------------------------------- */
agc_status_t agc_job_sps_burn(agc_job_ctx_t *ctx);                      /* P40CSM      */
agc_status_t agc_job_rcs_burn_csm(agc_job_ctx_t *ctx);                  /* P41CSM      */
agc_status_t agc_job_thrust_monitor_csm(agc_job_ctx_t *ctx);            /* P47CSM      */
agc_status_t agc_job_sps_burn_after_maneuver(agc_job_ctx_t *ctx);       /* P40SXTY +4  */
agc_status_t agc_job_gimbal_test_and_trim(void);                        /* TST,TRIM (V33) */
agc_status_t agc_job_gimbal_trim_only(void);                            /* TRIMONLY    */
agc_status_t agc_job_preburn_state_vector(agc_job_ctx_t *ctx);          /* P40S/SV     */
agc_status_t agc_job_preburn_blank_dsky(agc_job_ctx_t *ctx);            /* P40BLNKR    */
agc_status_t agc_job_postburn_csm(agc_job_ctx_t *ctx);                  /* POSTBURN    */
agc_status_t agc_job_sps_burn_rcs_trim(void);                           /* P40RCS      */
agc_status_t agc_job_sps_burn_min_deadband(agc_job_ctx_t *ctx);         /* P40MINDB    */
agc_status_t agc_job_tignow_csm(agc_job_ctx_t *ctx);                    /* TIGNOW      */
agc_status_t agc_job_post41(void);                                      /* POST41      */
agc_status_t agc_job_calc_vg_body_csm(void);                            /* CALCN85     */
agc_status_t agc_job_calc_delta_v_body_csm(void);                       /* CALCN83     */
agc_status_t agc_job_thrust_monitor_reset_csm(agc_job_ctx_t *ctx);      /* P47BODY     */
agc_status_t agc_job_thrust_monitor_display_csm(agc_job_ctx_t *ctx);    /* P47BOD      */
agc_status_t agc_job_steering_csm(void);                                /* STEERING    */
agc_status_t agc_job_vtogain_csm(agc_job_ctx_t *ctx);                   /* S40.9       */
void         agc_job_redo40_9(void);                                    /* REDO40.9    */
agc_status_t agc_job_timeburn_csm(void);                                /* S40.13      */
agc_status_t agc_job_clockjob(agc_job_ctx_t *ctx);                      /* CLOCKJOB    */
agc_status_t agc_job_dyndisp(agc_job_ctx_t *ctx);                       /* DYNDISP     */
agc_status_t agc_job_engine_enable_bypass(void);                        /* V99E        */
agc_status_t agc_job_engine_enable_terminate(void);                     /* V99T        */
agc_status_t agc_job_engine_enable_proceed(void);                       /* V99P        */
agc_status_t agc_job_engine_fail_terminate(void);                       /* V97T        */
agc_status_t agc_job_engine_fail_proceed(void);                         /* V97P        */
agc_status_t agc_job_engine_fail_reignite(void);                        /* V97E        */
agc_status_t agc_job_dapfig(void);                                      /* DAPFIG      */
agc_status_t agc_job_preburn_start_clock(agc_job_ctx_t *ctx);           /* after TRIM DELAYJOB */
agc_status_t agc_job_cross_product_restart(agc_job_ctx_t *ctx);         /* S40.8B / UPDATEVG_B / SURELY9_B */
agc_status_t agc_job_steering_restart(void);                            /* STEERING restart */
void         agc_job_endlamb(void);                                     /* ENDLAMB     */
void         agc_job_copy40_9(void);                                    /* COPY40.9    */
agc_status_t agc_job_calc_delta_v_restart(void);                        /* CALCN83 restart */
agc_status_t agc_task_engine_fail_shutdown_restart(void);               /* V97TTASK after 5.1 C */
agc_status_t agc_task_quickign(void);                                   /* QUICKIGN    */
agc_status_t agc_csm_comptgo(void);                                     /* COMPTGO     */
agc_status_t agc_csm_cross_product_steering(agc_interp_t *ip);          /* S40.8 (CM)  */
agc_status_t agc_csm_updatevg(agc_interp_t *ip, bool *vtogain_started); /* UPDATEVG */
void         agc_csm_spsoff(void);                                      /* SPSOFF/MASSBACK */
void         agc_csm_tvczap(void);                                      /* TVCZAP      */
agc_status_t agc_task_ttg_0(void);                                      /* TTG/0       */
agc_status_t agc_task_tigblnk(void);                                    /* TIGBLNK     */
agc_status_t agc_task_tigon(void);                                      /* TIGON       */
agc_status_t agc_task_tigaveg(void);                                    /* TIGAVEG     */
agc_status_t agc_task_precheck(void);                                   /* PRECHECK    */
agc_status_t agc_task_tig_5_csm(void);                                  /* TIG-5       */
agc_status_t agc_task_tig_0_csm(void);                                  /* TIG-0       */
agc_status_t agc_task_ignition_csm(void);                               /* IGNITION    */
agc_status_t agc_task_dotvcon(void);                                    /* DOTVCON     */
void         agc_task_dostrull(void);                                   /* DOSTRULL    */
agc_status_t agc_task_implcont(void);                                   /* IMPLCONT    */
agc_status_t agc_task_enginoff_csm(void);                               /* ENGINOFF    */
agc_status_t agc_task_dospsoff(void);                                   /* DOSPSOFF    */
agc_status_t agc_task_dotvcrcs(void);                                   /* DOTVCRCS    */
agc_status_t agc_task_cloktask_csm(void);                               /* CLOKTASK    */
agc_status_t agc_task_engine_fail_shutdown(void);                       /* V97TTASK    */
agc_status_t agc_task_engine_fail_rcs_dap(void);                        /* V97TRCS     */
agc_status_t agc_task_engine_fail_resume(void);                         /* V97PTASK    */
void         agc_task_engine_fail_reenable(void);                       /* R40ENABL    */
agc_status_t agc_task_engine_fail_reignite_prep(void);                  /* V97ETASK    */
agc_status_t agc_task_spsoff97(void);                                   /* SPSOFF97    */
agc_status_t agc_task_engine_fail_reignite_trim(void);                  /* V97E40.6    */
agc_status_t agc_task_engine_enable_flash(void);                        /* V99FLASH    */
agc_status_t agc_task_gimbal_test_trim(void);                           /* S40.6       */
agc_status_t agc_task_gimbal_trim_restart(void);                        /* PRE40.6     */
agc_status_t agc_task_gimbal_test_relays(void);                         /* S40.6 after 60 ms */
agc_status_t agc_task_gimbal_test_rstrtst(void);                        /* RSTRTST     */
agc_status_t agc_task_gimbal_test_step(void);                           /* OUTPUT return / GIMTRIM */

/* ---- LM P40-P47 (src/lm/burns/dps_burn.c) ------------------------------------- */
agc_status_t agc_job_dps_burn(agc_job_ctx_t *ctx);                  /* P40LM       */
agc_status_t agc_job_rcs_burn_lm(agc_job_ctx_t *ctx);               /* P41LM       */
agc_status_t agc_job_aps_burn(agc_job_ctx_t *ctx);                  /* P42LM       */
agc_status_t agc_job_thrust_monitor_lm(agc_job_ctx_t *ctx);         /* P47LM       */
agc_status_t agc_job_rcs_burn_after_maneuver(agc_job_ctx_t *ctx);   /* P41 after R60LEM */
agc_status_t agc_job_postburn_lm(agc_job_ctx_t *ctx);               /* POSTBURN    */
agc_status_t agc_job_tignow_lm(agc_job_ctx_t *ctx);                 /* TIGNOW      */
agc_status_t agc_job_term40(void);                                  /* TERM40      */
agc_status_t agc_job_dynmdisp(void);                                /* DYNMDISP    */
agc_status_t agc_job_redo6_7(agc_job_ctx_t *ctx);                   /* REDO6.7 / BLNKWAIT */
agc_status_t agc_job_rcs_burn_vg_displayed(void);                   /* P41 after GODSPRET */
agc_status_t agc_job_redo6_7_vg_displayed(void);                    /* REDO6.7 after GODSPRET */
agc_status_t agc_job_calc_vg_body_lm(void);                         /* CALCN85     */
agc_status_t agc_job_calc_delta_v_body_lm(void);                    /* CALCN83     */
agc_status_t agc_job_thrust_monitor_reset_lm(agc_job_ctx_t *ctx);   /* P47BODY     */
agc_status_t agc_job_thrust_monitor_display_lm(agc_job_ctx_t *ctx); /* P47BOD      */
agc_status_t agc_job_steering_lm(void);                             /* STEERING    */
agc_status_t agc_job_vtogain_lm(agc_job_ctx_t *ctx);                /* S40.9       */
agc_status_t agc_job_timeburn_lm(void);                             /* S40.13      */
agc_status_t agc_task_start_thrust_monitor(void);                   /* STARTP47    */
agc_status_t agc_task_implburn_lm(void);                            /* IMPLBURN    */
void         agc_task_implburn2(void);                              /* IMPLBURN +0.5 s */
agc_status_t agc_task_engoftsk(void);                               /* ENGOFTSK    */
agc_status_t agc_task_coastset(void);                               /* COASTSET    */
agc_status_t agc_task_trimgimb(void);                               /* TRIMGIMB    */
agc_status_t agc_task_trimgimb2(void);                              /* TRIMGIMB after 60 s */
agc_status_t agc_task_rolloff(void);                                /* roll done (VARDELAY) */
agc_status_t agc_task_pitchoff(void);                               /* PITCHOFF    */
agc_status_t agc_lm_cross_product_steering(agc_interp_t *ip);       /* S40.8 (LM)  */
agc_status_t agc_lm_rasteer1(agc_interp_t *ip);                     /* RASTEER1    */
agc_status_t agc_lm_updatevg(agc_interp_t *ip);                     /* UPDATEVG    */
agc_status_t agc_lm_enginoff(void);                                 /* ENGINOFF    */
void         agc_lm_enginof1(void);                                 /* ENGINOF1    */
agc_status_t agc_lm_enginof2(void);                                 /* ENGINOF2    */
void         agc_lm_enginof3(void);                                 /* ENGINOF3    */
void         agc_lm_enginof4(void);                                 /* ENGINOF4    */

/* ---- LM BURN BABY BURN (src/lm/burns/ignition/) ----------------------------- */
typedef enum agc_lm_which
{
    AGC_LM_WHICH_POWERED_ASCENT = 0, /* P12TABLE */
    AGC_LM_WHICH_DPS_BURN,           /* P40TABLE */
    AGC_LM_WHICH_RCS_BURN,           /* P41TABLE */
    AGC_LM_WHICH_APS_BURN,           /* P42TABLE */
    AGC_LM_WHICH_BRAKING_PHASE,      /* P63TABLE */
    AGC_LM_WHICH_ABORT,              /* ABRTABLE */
    AGC_LM_WHICH_COUNT
} agc_lm_which_t;

agc_status_t agc_lm_burnbaby(const agc_job_ctx_t *ctx);                                 /* BURNBABY    */
agc_status_t agc_lm_check_auto_control(const agc_job_ctx_t *ctx, agc_job_id_t ret_job); /* P40AUTO */
agc_status_t agc_lm_stclok2(agc_sp_t dispdex, agc_job_id_t ret_job);                    /* STCLOK2     */
agc_status_t agc_lm_stclok1(void);                                                      /* STCLOK1     */
void         agc_lm_nullclok(void);                                                     /* NULLCLOK    */
void         agc_lm_noullage(void);                                                     /* NOULLAGE    */
void         agc_lm_onullage(void);                                                     /* ONULLAGE    */
agc_status_t agc_job_burnbaby_countdown(void);                                          /* B*RNB*B* as a job (after P40AUTO) */
agc_status_t agc_job_preburn_state_vector_lm(agc_job_ctx_t *ctx);                       /* P41SPOT: state to TIG-30, TIG-35 call */
agc_status_t agc_job_auto_control_recycle(agc_job_ctx_t *ctx);                          /* P40A/P recycle */
agc_status_t agc_job_rcs_burn_blank_dsky(agc_job_ctx_t *ctx);                           /* P41BLANK    */
agc_status_t agc_job_restore_burn_display(agc_job_ctx_t *ctx);                          /* TIG-30A: V16N85 again (P41) */
agc_status_t agc_job_clokjob(agc_job_ctx_t *ctx);                                       /* CLOKJOB     */
void         agc_job_clokjob_engine_fail(void);                                         /* V97 PROCEED (COMFAIL1) */
agc_status_t agc_job_clokjob_engine_fail_shutdown(void);                                /* V97 ENTER (COMFAIL2) */
agc_status_t agc_job_clokjob_event_timer_proceed(agc_job_ctx_t *ctx);                   /* V06N61 proceed */
agc_status_t agc_job_clokjob_engine_enable(void);                                       /* V99 *PROCEED */
agc_status_t agc_job_clokjob_engine_enable_bypass(agc_job_ctx_t *ctx);                  /* V99 *ENTER */
agc_status_t agc_job_stopclok(void);                                                    /* STOPCLOK    */
agc_status_t agc_job_comfail(void);                                                     /* COMFAIL     */
agc_status_t agc_job_stage_alarm_display(agc_job_ctx_t *ctx);                           /* REP40ALM    */
agc_status_t agc_job_stage_alarm_proceed(void);                                         /* REP40ALM PROCEED */
agc_status_t agc_task_tig_35(void);                                                     /* TIG-35      */
agc_status_t agc_task_restore_burn_display(void);                                       /* TIG-30.1    */
agc_status_t agc_task_tig_30(void);                                                     /* TIG-30      */
agc_status_t agc_task_redo4_2(void);                                                    /* REDO4.2     */
agc_status_t agc_task_redo2_17(void);                                                   /* REDO2.17    */
void         agc_task_ullgtask(void);                                                   /* ULLGTASK    */
agc_status_t agc_task_tig_5_lm(void);                                                   /* TIG-5       */
agc_status_t agc_task_tig_0_lm(void);                                                   /* TIG-0       */
agc_status_t agc_task_ignition_if_enabled(void);                                        /* IGNYET?     */
agc_status_t agc_task_ignition_lm(void);                                                /* IGNITION    */
agc_status_t agc_task_ignition_finish(void);                                            /* P42IGN: common ignition tail */
void         agc_task_ullagoff(void);                                                   /* ULLAGOFF    */
agc_status_t agc_task_tigtask(void);                                                    /* TIGTASK     */
void         agc_task_braking_phase_zoom(void);                                         /* P63ZOOM     */
void         agc_task_dps_burn_zoom(void);                                              /* P40ZOOM     */
void         agc_task_zoom(void);                                                       /* ZOOM = P40ZOOMA */
agc_status_t agc_task_cloktask_lm(void);                                                /* CLOKTASK    */

/* Alarm codes raised by the powered-flight programs (octal, as in the listings).             */
#define AGC_ALARM_00205 \
    ((agc_alarm_code_t) 00205) /* CM: PIPA saturated        */
#define AGC_ALARM_00511 \
    ((agc_alarm_code_t) 00511) /* LM: LR not in position    */
#define AGC_ALARM_00523 \
    ((agc_alarm_code_t) 00523) /* LM: LR repositioning failed */
#define AGC_ALARM_01407 \
    ((agc_alarm_code_t) 01407) /* VG increasing (thrust > 90 deg off) */
#define AGC_ALARM_01706 \
    ((agc_alarm_code_t) 01706) /* LM: P40/P42 vs. stage config */

#endif /* AGC_GUID_H */
