/*
 * agc_csm_dap.h -- Comanche055 (CSM) RCS and TVC digital autopilots.
 *
 * Owner: dap-csm lane (src/csm/dap/). Sources, Comanche055:
 *   MYSUBS (pp. 999-1000), RCS-CSM_DIGITAL_AUTOPILOT (pp. 1002-1024),
 *   AUTOMATIC_MANEUVERS (pp. 1025-1036), RCS-CSM_DAP_EXECUTIVE_PROGRAMS
 *   (pp. 1037-1038), JET_SELECTION_LOGIC (pp. 1039-1062), TVCINITIALIZE
 *   (pp. 937-944), TVCEXECUTIVE (pp. 945-950), TVCMASSPROP (pp. 951-955),
 *   TVCRESTARTS (pp. 956-960), TVCDAPS (pp. 961-978), TVCSTROKETEST
 *   (pp. 979-983), TVCROLLDAP (pp. 984-998); plus S41.2 / S40.14 / RCSDAPON
 *   from P40-P47 (pp. 731-736).
 *
 * The EBANK6 autopilot erasables are module-private (see the state-flags
 * owner table) and live in three structs below. Other lanes reach them
 * through agc_csm_dap_rcs() / _tvc() / _com(); the pointers stay valid for
 * the life of the program (static storage, rule 3).
 *
 * Scales: angles written "180 deg" are SP fractions with 2^14 = 180 deg
 * (the ones' complement view of a CDU word); "1/1600 s" means one bit is
 * one TIME6 count (625 microseconds).
 */
#ifndef AGC_CSM_DAP_H
#define AGC_CSM_DAP_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"

#define AGC_CSM_AXES        3  /* index 0 = roll (X), 1 = pitch (Y), 2 = yaw (Z) */
#define AGC_CSM_N10_WORDS   15 /* N10 .. D32 filter coefficient block       */
#define AGC_CSM_VARST_WORDS 10 /* VARST0 .. VARST9 (MASSPROP)               */

/* RCSFLAGS bits (RCS-CSM_DIGITAL_AUTOPILOT.agc). */
#define AGC_CSM_RCSF_T6INIT \
    ((uint16_t) 000001) /* BIT1: T6 channels initialised */
#define AGC_CSM_RCSF_NEEDLE2 \
    ((uint16_t) 000002) /* BIT2: needles pass 2           */
#define AGC_CSM_RCSF_NEEDLE1 \
    ((uint16_t) 000004) /* BIT3: needles initialise pass  */
#define AGC_CSM_RCSF_FDAIALT \
    ((uint16_t) 000010) /* BIT4: FDAIDSP1/2 alternation   */
#define AGC_CSM_RCSF_FORCED \
    ((uint16_t) 001760) /* BITS 10-5: forced firing       */
#define AGC_CSM_RCSF_DAMPING \
    ((uint16_t) 016000) /* BITS 13-11: rate damping       */
#define AGC_CSM_RCSF_NORATE \
    ((uint16_t) 020000) /* BIT14: rate filter not init    */
#define AGC_CSM_RCSF_HIRATE \
    ((uint16_t) 040000) /* BIT15: high-rate auto maneuver */

/* T5PHASE (p. 1003): + initialise, +0 phase 2, - restart DAP, -0 phase 1.
 * Ones' complement +0/-0 are distinct states, so an enum stands in.       */
typedef enum agc_csm_t5phase
{
    AGC_CSM_T5P_RATE_FILTER = 0,  /* -0 */
    AGC_CSM_T5P_ATTITUDE_CONTROL, /* +0 */
    AGC_CSM_T5P_RESTART,          /* -  */
    AGC_CSM_T5P_INIT              /* +  */
} agc_csm_t5phase_t;

/* Branch index stored in the low word of BLAST/BLAST1/BLAST2 (p. 1059). */
typedef enum agc_csm_jet_axis
{
    AGC_CSM_JET_ROLL  = 0, /* REPLACER  (BLAST +1 = 0)   */
    AGC_CSM_JET_PITCH = 4, /* REPLACEP  (BLAST1 +1 = 4)  */
    AGC_CSM_JET_YAW   = 11 /* REPLACEY  (BLAST2 +1 = 11) */
} agc_csm_jet_axis_t;

typedef struct agc_csm_blast
{
    agc_sp_t           time; /* on-time or interval, 1/1600 s; <0 = done */
    agc_csm_jet_axis_t axis; /* which word to replace when it expires     */
} agc_csm_blast_t;

/* ---- RCS DAP erasables (RCSATT .. T6START) -------------------------- */
typedef struct agc_csm_rcs
{
    agc_csm_t5phase_t t5phase;                  /* T5PHASE                                  */
    agc_sp_t          t5time;                   /* T5TIME, TIME5 counts (10 ms)             */
    agc_sp_t          holdflag;                 /* HOLDFLAG: + grab, +0 hold, - auto steer  */
    agc_dp_t          wbody[AGC_CSM_AXES];      /* WBODY commanded rate, 450 deg/s      */
    agc_dp_t          adot[AGC_CSM_AXES];       /* ADOT estimated rate, 450 deg/s       */
    agc_dp_t          merror[AGC_CSM_AXES];     /* MERRORX manual att. error, 180 deg   */
    agc_sp_t          dft[AGC_CSM_AXES];        /* DFT expected 1-jet firing, 1/1600 s  */
    agc_dp_t          drho[AGC_CSM_AXES];       /* DRHO filter residual, 180 deg        */
    agc_sp_t          attsec;                   /* ATTSEC 0.1 s pass counter            */
    agc_sp_t          tau[AGC_CSM_AXES];        /* TAU rotation command, 1/1600 s       */
    agc_sp_t          bias[AGC_CSM_AXES];       /* BIAS KALCMANU lead, 180 deg          */
    agc_sp_t          error[AGC_CSM_AXES];      /* ERRORX attitude error, 180 deg       */
    uint16_t          thetad[AGC_CSM_AXES];     /* THETADX..Z hold reference: raw CDU word (GRABANG) or 1's comp DPADD high word (DCDUINCR), 180 deg */
    agc_dp_t          cdud[AGC_CSM_AXES];       /* CDUXD desired gimbal DP, 180 deg     */
    agc_dp_t          delcdu[AGC_CSM_AXES];     /* DELCDUX increment per 0.1 s, 180 deg*/
    uint16_t          rho[AGC_CSM_AXES];        /* RHO, RHO1, RHO2 previous CDU (raw)   */
    agc_sp_t amgb1, amgb4, amgb5, amgb7, amgb8; /* AMGB elements, scale 1   */
    agc_sp_t          capsi;                    /* CAPSI cos(psi), scale 1              */
    agc_sp_t          kmj[AGC_CSM_AXES];        /* KMJ torque/inertia (S40.14)          */
    agc_sp_t          jm[AGC_CSM_AXES];         /* J/M inertia/torque (S40.14)          */
    agc_sp_t          attkalmn;                 /* ATTKALMN gain index -3..11           */
    agc_sp_t          ratindx;                  /* RATEINDX 0,2,4,6 (S41.2)             */
    agc_sp_t          adb;                      /* ADB deadband, 180 deg (46 or 455)    */
    agc_sp_t          slope;                    /* SLOPE switch-line slope .24          */
    agc_sp_t racfail, rbdfail;                  /* +: A(B) failed, -: C(D) failed, 0 ok */
    agc_sp_t          acorbd;                   /* - AC roll, + BD roll                 */
    agc_sp_t          xtrans;                   /* +1 BD-only X, -1 AC-only X, 0 both   */
    uint16_t          ch31temp;                 /* CH31TEMP last channel 31 image       */
    uint16_t          chantemp;                 /* CHANTEMP last channel 32 image       */
    agc_sp_t          manndx[AGC_CSM_AXES];     /* RMANNDX,PMANNDX,YMANNDX 0 none 1 + 2 -*/
    uint16_t rword1, rword2;                    /* RWORD1/2 channel 6 images            */
    uint16_t pword1, pword2;                    /* PWORD1/2 channel 5 pitch part        */
    uint16_t yword1, yword2;                    /* YWORD1/2 channel 5 yaw part          */
    agc_csm_blast_t   blast[AGC_CSM_AXES];      /* BLAST, BLAST1, BLAST2           */
} agc_csm_rcs_t;

/* ---- RCS/TVC common storage and pad loads (EBANK6) ------------------- */
typedef struct agc_csm_dapcom
{
    agc_sp_t ak[AGC_CSM_AXES];         /* AK, AK1, AK2 needle errors, 180 deg  */
    uint16_t rcsflags;                 /* RCSFLAGS                             */
    agc_sp_t edrive[AGC_CSM_AXES];     /* EDRIVEX.. DAC contents, 1 bit = .044 deg */
    agc_sp_t ixx;                      /* IXX, kg m^2, scale 2^20              */
    agc_sp_t iavg;                     /* IAVG, kg m^2, scale 2^20             */
    agc_sp_t iavg_tlx;                 /* IAVG/TLX, s^2, scale 2^2             */
    agc_sp_t weight_g;                 /* WEIGHT/G, kg, scale 2^16             */
    agc_sp_t emdot;                    /* EMDOT SPS flow, kg/cs, scale 2^3 (PL)*/
    agc_sp_t estroker;                 /* ESTROKER burst size, 85.41"/bit (PL) */
    agc_sp_t ekprime[2];               /* EKPRIME LM-off/on, (100 pi)/16 (PL)  */
    agc_sp_t ektlx_i[3];               /* EKTLX/I off/on/low-bw, 1/(8 ascrev) (PL) */
    agc_sp_t erepfrac[2];              /* EREPFRAC off/on, 2^2 (PL)            */
    agc_sp_t pactoff, yactoff;         /* PACTOFF/YACTOFF trims, 85.41"/bit    */
    agc_sp_t hbn10[AGC_CSM_N10_WORDS]; /* HBN10.. high-bandwidth coeffs (PL)*/
    uint16_t ogad;                     /* OGAD outer gimbal at ignition (raw)  */
    uint16_t cphix[AGC_CSM_AXES];      /* CPHIX N17 attitude (V60/V25N17), raw */
} agc_csm_dapcom_t;

/* An AGC double-precision erasable B(2) as its two words are stored.
 * DAS adds the low words (carrying into the high word) but never makes
 * the two signs agree, and the TVC DAP later reads or overwrites the high
 * word alone (ERRORLIM, ACTLIM, PCMD, AK1): the split, not just the
 * value hi * 2^14 + lo, decides the result (oracle finding F9).          */
typedef struct agc_csm_dp_pair
{
    agc_sp_t hi; /* upper word (the DP's scale)                   */
    agc_sp_t lo; /* lower word, 2^-14 of the upper word's scale   */
} agc_csm_dp_pair_t;

/* ---- TVC DAP erasables ------------------------------------------------ */
typedef struct agc_csm_tvc
{
    agc_dp_t          omegac[AGC_CSM_AXES];       /* OMEGAXC.. steering rate cmd (S40.8)  */
    agc_csm_dp_pair_t omegab[AGC_CSM_AXES];       /* OMEGAXB.. body rate (Y,Z), 1/2 TVCDT revs */
    agc_csm_dp_pair_t ptmp[6];                    /* PTMP1-6 pitch filter nodes, scale 1    */
    agc_csm_dp_pair_t ytmp[6];                    /* YTMP1-6 yaw filter nodes, scale 1      */
    agc_sp_t          rollfire;                   /* ROLLFIRE: +-POSMAX on, 0 off         */
    uint16_t          rollword;                   /* ROLLWORD alternation bits 1,2        */
    agc_sp_t          temreg;                     /* TEMREG roll firing time (1/1600 s)   */
    agc_sp_t          stroker;                    /* STROKER stroke burst, 0 = no test    */
    agc_csm_dp_pair_t perrb;                      /* PERRB pitch error integrator, B-1 rev */
    agc_csm_dp_pair_t yerrb;                      /* YERRB yaw error integrator, B-1 rev   */
    agc_csm_dp_pair_t delpbar;                    /* DELPBAR pitch offset tracker, 1 ascrev */
    agc_csm_dp_pair_t delybar;                    /* DELYBAR yaw offset tracker, 1 ascrev  */
    agc_csm_dp_pair_t pdeloff;                    /* PDELOFF pitch trim estimate, 1 ascrev */
    agc_csm_dp_pair_t ydeloff;                    /* YDELOFF yaw trim estimate, 1 ascrev   */
    agc_dp_t ttmp1, ttmp2;                        /* TTMP1/TTMP2 (PHASETMP / RTRNLOC)     */
    agc_csm_dp_pair_t dap1;                       /* DAP1 first cascade output, B-1 rev    */
    agc_csm_dp_pair_t dap2;                       /* DAP2 second cascade output, B-1 rev   */
    agc_csm_dp_pair_t dap3;                       /* DAP3 third cascade output, B-1 rev    */
    agc_sp_t pcmd, ycmd;                          /* PCMD/YCMD last actuator commands     */
    agc_sp_t          t5tvcdt;                    /* T5TVCDT TIME5 word for TVCDT/2       */
    agc_dp_t          kprimedt;                   /* KPRIMEDT steering gain, pi/8         */
    agc_sp_t          ktlx_i;                     /* KTLX/I, 1/(8 ascrev)                 */
    agc_sp_t          tenmdot;                    /* TENMDOT 10-s mass loss, kg 2^16      */
    agc_sp_t          one_conacc;                 /* 1/CONACC, s^2/rev, scale 2^9         */
    agc_sp_t          vark;                       /* VARK, (2^3 ascrev)                   */
    agc_sp_t          repfrac;                    /* REPFRAC, 2^2                         */
    agc_sp_t vcntr, engine_fail_vcntr;            /* VCNTR, V97VCNTR gain update counters; any negative value stands for the AGC's -0 (V97/R40 LOTHRUST) */
    agc_sp_t          tvcphase;                   /* TVCPHASE -2..3                       */
    uint16_t pcduypst, pcduzpst;                  /* PCDUYPST/PCDUZPST past CDUs (raw)    */
    agc_sp_t mcduydot, mcduzdot;                  /* MCDUYDOT/MCDUZDOT minus CDU rates    */
    agc_sp_t          tvcexphs;                   /* TVCEXPHS 0..6                        */
    agc_sp_t masstmp, vcntrtmp;                   /* MASSTMP, VCNTRTMP                    */
    agc_sp_t strktime, caddy, n, revs, card;      /* stroke test                 */
    bool              bunker_task;                /* BUNKER = TC TASKOVER (vs. DAP return)*/
    uint16_t oganow, ogapast;                     /* OGANOW/OGAPAST outer gimbal (raw)    */
    agc_sp_t          ogaerr;                     /* OGA/OGAERR fly-from error, rev 2^0   */
    agc_sp_t          ogarate;                    /* OGARATE rev/s, scale 2^-4            */
    agc_sp_t delogart, sgnrt, deloga, iogarate;   /* roll logic temporaries   */
    int               i_sign;                     /* I: +1 (CA), -1 (CS), 0 (MOREROLL)    */
    agc_csm_dp_pair_t pacttmp;                    /* PACTTMP pitch trim (CG.CORR), 1 ascrev */
    agc_csm_dp_pair_t yacttmp;                    /* YACTTMP yaw trim (CG.CORR), 1 ascrev  */
    agc_sp_t cntr, cntrtmp;                       /* CNTR/CNTRTMP one-shot counter        */
    agc_csm_dp_pair_t delbrtmp;                   /* DELBRTMP offset tracker, 1 ascrev   */
    agc_csm_dp_pair_t errbtmp;                    /* ERRBTMP body error, B-1 rev; before ATTINIT the two words hold -ERRORY, -ERRORZ (180 deg) from P40 DOTVCON */
    agc_csm_dp_pair_t cmdtmp;                     /* CMDTMP actuator command, 1 ascrev    */
    agc_csm_dp_pair_t tmp[6];                     /* TMP1..TMP6 working filter nodes      */
    agc_sp_t          n10[AGC_CSM_N10_WORDS];     /* N10..D32 active filter coefficients  */
    agc_sp_t          tempdap[2];                 /* TEMPDAP, TEMPDAP+1 correction fraction*/
    agc_sp_t          phasetmp;                   /* PHASETMP (V46 switch-over)           */
    agc_sp_t          varst[AGC_CSM_VARST_WORDS]; /* VARST0..9 MASSPROP breakpoints  */
} agc_csm_tvc_t;

/* ---- jet selection as a pure function (JETSLECT .. T6SETUP) ---------- */
typedef struct agc_csm_jet_in
{
    uint16_t chan31;            /* channel 31 image (THC bits 7-12, active 0) */
    bool     lem_on;            /* DAPDATR1 BIT14                           */
    agc_sp_t tau[AGC_CSM_AXES]; /* TAU, TAU1, TAU2 (1/1600 s)               */
    agc_sp_t racfail, rbdfail, acorbd, xtrans;
} agc_csm_jet_in_t;

typedef struct agc_csm_jet_out
{
    uint16_t rword1, rword2, pword1, pword2, yword1, yword2;
    agc_csm_blast_t blast[AGC_CSM_AXES]; /* sorted: [0] first expiry, then deltas */
    agc_sp_t        dft[AGC_CSM_AXES];   /* DFT, DFT1, DFT2                 */
    agc_sp_t        tau[AGC_CSM_AXES];   /* updated TAU, TAU1, TAU2         */
    agc_sp_t        attkalmn_xln;        /* ATTKALMN for translation, or 0x7FFF if unchanged */
} agc_csm_jet_out_t;

agc_status_t agc_csm_jet_select(const agc_csm_jet_in_t *in, agc_csm_jet_out_t *out);

/* ---- state access ----------------------------------------------------- */
agc_csm_rcs_t    *agc_csm_dap_rcs(void);
agc_csm_tvc_t    *agc_csm_dap_tvc(void);
agc_csm_dapcom_t *agc_csm_dap_com(void);

/* ---- entry points used by other lanes -------------------------------- */
agc_status_t agc_csm_dap_decode_dap_data(void); /* S41.2 decode DAPDATR1/2   */
agc_status_t agc_csm_dap_rcs_gains(void);       /* S40.14 RCS gains          */
agc_status_t agc_csm_dap_rcsdapon(void);        /* RCSDAPON (call under INHINT) */
void         agc_csm_dap_massprop(void);        /* MASSPROP                  */
void         agc_csm_dap_fixcw(void);           /* FIXCW (MASSPROP +)        */
void         agc_csm_dap_zerojet(void);         /* ZEROJET                   */
agc_status_t agc_csm_tvc_preswtch(void);        /* PRESWTCH (V46 CSM/LM switch-over) */
void         agc_csm_dap_setmaxdb(void);        /* SETMAXDB (P40-P47 p.687)  */
void         agc_csm_dap_setmindb(void);        /* SETMINDB (P40-P47 p.687)  */

/* RCSDAPON +1 (R00TOP00 entry): TIME5 word supplied by the caller.       */
agc_status_t agc_csm_dap_rcsdapon_t5(uint16_t time5_word);

/* NEEDLER with the errors given (stored in AK..AK2 first); init sets
 * RCSFLAGS BIT3 first (the user's initialisation pass). For guid-entry.   */
agc_status_t agc_dap_needler(agc_sp_t ak, agc_sp_t ak1, agc_sp_t ak2, bool init);

/* STICKCHK: RHC command bits 1-6 of channel 31 (active 1 after the
 * inversion, bits 1,2 pitch, 3,4 yaw, 5,6 roll) -> RMANNDX..YMANNDX.
 * For SATSTICK (guid-entry / Saturn takeover).                            */
agc_status_t agc_csm_dap_stickchk(uint16_t ch31_bits);

/* ---- T5 / T6 interrupt entries (T5LOC / T6LOC targets) --------------- */
agc_status_t agc_task_redorcs(void);
agc_status_t agc_task_rcsatt(void);
agc_status_t agc_task_jetslect(void);
agc_status_t agc_task_t6start(void);
agc_status_t agc_task_tvcdapon(void);
agc_status_t agc_task_tvc_initialize(void);
agc_status_t agc_task_dapinit(void);
agc_status_t agc_task_pitchdap(void);
agc_status_t agc_task_yawdap(void);
agc_status_t agc_task_redotvc(void);
agc_status_t agc_task_optics_counter_enable(void);
agc_status_t agc_task_cmdsout(void);
agc_status_t agc_task_roll_jets_off(void);
agc_status_t agc_task_rcsup(void); /* RCSUP (P40-P47 p.736)     */

/* ---- waitlist tasks and jobs ------------------------------------------ */
agc_status_t agc_task_tvcexec(void);
agc_status_t agc_task_rolldap(void);
agc_status_t agc_task_hackwlst(void);
agc_status_t agc_task_exrstrt(void);
agc_status_t agc_job_ambgupdt(agc_job_ctx_t *ctx);
agc_status_t agc_job_strktsti(agc_job_ctx_t *ctx);

#endif /* AGC_CSM_DAP_H */
