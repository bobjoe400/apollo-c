/*
 * agc_csm_entry.h -- Comanche (CM) entry: P61-P67 entry guidance, the CM
 * body-attitude computation, the entry digital autopilot, P37 return to
 * earth and P11 earth orbit insertion monitor.     (guid-entry lane)
 *
 * Sources: Comanche055 ENTRY_LEXICON.agc (pp. 837-843), REENTRY_CONTROL.agc
 * (pp. 844-881), P61-P67.agc (pp. 789-818), CM_BODY_ATTITUDE.agc
 * (pp. 883-889), CM_ENTRY_DIGITAL_AUTOPILOT.agc (pp. 1063-1092),
 * P37_P70.agc (pp. 890-933), P11.agc (pp. 533-550).
 *
 * Units used in the scale comments (ENTRY_LEXICON p. 837):
 *   VS   = VSAT = 25766.1973 ft/s, so "/2VS" means value / 51532.3946 ft/s
 *   805  = 25 g = 805 ft/s^2 (drag and acceleration scale)
 *   "revs" = revolutions (angle / 360 deg); 21600 NM = 1 rev of range
 *   .5 unit = half-unit vector (each component is u/2, scale 2^1)
 */
#ifndef AGC_CSM_ENTRY_H
#define AGC_CSM_ENTRY_H

#include <stdbool.h>
#include <stdint.h>
#include "agc_status.h"
#include "agc_word.h"
#include "agc_interp.h"

/* GOTOADDR: the entry phase sequencer (REENTRY_CONTROL p. 847).          */
typedef enum agc_entry_goto {
    AGC_ENTRY_GOTO_INITROLL = 0,  /* P63/P64 initial roll                  */
    AGC_ENTRY_GOTO_HUNTEST,       /* P64 up-control solution search        */
    AGC_ENTRY_GOTO_UPCONTRL,      /* P65 up-control                        */
    AGC_ENTRY_GOTO_KEP2,          /* P66 ballistic (Kepler) phase          */
    AGC_ENTRY_GOTO_PREFINAL,      /* entry to P67 final phase              */
    AGC_ENTRY_GOTO_PREDICT3,      /* P67 final phase                       */
    AGC_ENTRY_GOTO_ENDEXIT,       /* sidetracked: skip steering this pass  */
    AGC_ENTRY_GOTO_P67_2,         /* after VQUIT: lat/long only            */
    AGC_ENTRY_GOTO_COUNT
} agc_entry_goto_t;

/* POSEXIT: where CM/POSE continues after the attitude update (p. 888).   */
typedef enum agc_entry_posexit {
    AGC_ENTRY_POSEXIT_P62_3 = 0,  /* desired gimbal angles (P62, KEP2)     */
    AGC_ENTRY_POSEXIT_STARTENT,   /* first pass of entry guidance (P63)    */
    AGC_ENTRY_POSEXIT_SCALEPOP,   /* normal entry guidance pass            */
    AGC_ENTRY_POSEXIT_ENDEXIT,    /* attitude only, then entry display     */
    AGC_ENTRY_POSEXIT_COUNT
} agc_entry_posexit_t;

/* CMDAPMOD (p. 1077). The AGC distinguishes +0 and -0; -0 is modelled as
 * its own value (DEVIATION, see docs/modules/guid-entry.md).             */
typedef enum agc_entry_dapmod {
    AGC_ENTRY_DAPMOD_MINUS1 = -1, /* in atmosphere (D > .05 g)             */
    AGC_ENTRY_DAPMOD_PLUS0  = 0,  /* ext. atmosphere, |cos alfa| > .707, cos alfa > 0 */
    AGC_ENTRY_DAPMOD_PLUS1  = 1,  /* ext. atmosphere, |cos alfa| <= .707   */
    AGC_ENTRY_DAPMOD_MINUS0 = 2   /* ext. atmosphere, cos alfa < 0: damping only */
} agc_entry_dapmod_t;

/* Entry erasables (ENTRY_LEXICON.agc). DP = agc_dp_t, SP = agc_sp_t.      */
typedef struct agc_entry_state {
    /* -- target and trajectory vectors, reference coordinates ---------- */
    agc_vec_t rtinit;     /* RTINIT  initial target vector, .5 unit         */
    agc_vec_t rteast;     /* RTEAST  east at RTINIT, .5 unit                */
    agc_vec_t rtnorm;     /* RTNORM  normal to RTEAST and UZ, .5 unit       */
    agc_vec_t rt;         /* RT      target vector (earth-rotated), .5 unit */
    agc_vec_t uni;        /* UNI     unit normal to trajectory plane, .5 unit */
    agc_vec_t unitv;      /* UNITV   unit V, .5 unit                        */
    agc_vec_t vel;        /* VEL     velocity, scale 2VS                    */
    agc_vec_t mvrel;      /* -VREL   negative velocity rel. atmosphere, 2VS */
    agc_vec_t uxa, uya, uza;   /* UXA/2.. trajectory triad, .5 unit        */
    agc_vec_t olduya;     /* OLDUYA  UYA saved for V < VQUIT, .5 unit       */
    agc_vec_t ubx, uby, ubz;   /* UBX/2.. CM body triad, .5 unit           */
    /* -- guidance scalars (DP) ------------------------------------------ */
    agc_dp_t vsquare;     /* VSQUARE V^2/VS^2, scale 4                       */
    agc_dp_t leq;         /* LEQ  (VSQ-1) g, scale 4 g                       */
    agc_dp_t v;           /* V    velocity magnitude /2VS                    */
    agc_dp_t rdot;        /* RDOT altitude rate /2VS                         */
    agc_dp_t d;           /* D    drag acceleration /805 fpss                */
    agc_dp_t a0, a1;      /* A0 A1 drag for upcontrol /805                   */
    agc_dp_t alp;         /* ALP  upcontrol constant, scale 1                */
    agc_dp_t askep, asp1, aspup, asp3, aspdwn; /* range terms, revs        */
    agc_dp_t cosg_2;      /* COSG/2                                          */
    agc_dp_t c_d0;        /* C/D0 = (-4/D0) B-8                              */
    agc_dp_t d0;          /* D0   controlled constant drag /805              */
    agc_dp_t dhook;       /* DHOOK /805                                      */
    agc_dp_t diff, diffold; /* DIFF DIFFOLD (ASP - THETAH), revs             */
    agc_dp_t dlewd;       /* DLEWD change in LEWD, scale 1                   */
    agc_dp_t dvl;         /* DVL  VS1 - VL, /2VS                             */
    agc_dp_t fact1;       /* FACT1 /2VS                                      */
    agc_dp_t fact2;       /* FACT2 x 805                                     */
    agc_dp_t factor;      /* FACTOR scale 1                                  */
    agc_dp_t gammal;      /* GAMMAL flight path angle at VL, rad             */
    agc_dp_t gammal1;     /* GAMMAL1 simple form, rad                        */
    agc_dp_t kat;         /* KAT  drag to lift up if down, /805              */
    agc_dp_t klat;        /* KLAT lateral switch gain, scale 1               */
    agc_dp_t k2roll;      /* K2ROLL roll switch indicator, +-DPOSMAX         */
    agc_dp_t lad;         /* LAD  max L/D, scale 1                           */
    agc_dp_t lod;         /* LOD  final phase L/D, scale 1                   */
    agc_dp_t l_d;         /* L/D  desired L/D (vertical plane), scale 1      */
    agc_dp_t l_d1;        /* L/D1 temp L/D in lateral, scale 1               */
    agc_dp_t l_dcminr;    /* L/DCMINR LAD cos 15, scale 1                    */
    agc_dp_t l_dcalc;     /* L/DCALC measured L/D (TM), scale 1              */
    agc_dp_t latang;      /* LATANG lateral range /4 rad                     */
    agc_dp_t lewd;        /* LEWD upcontrol reference L/D, scale 1           */
    agc_dp_t q2;          /* Q2   final phase range term, revs               */
    agc_dp_t q7;          /* Q7   min drag for upcontrol /805                */
    agc_dp_t rdotref;     /* RDOTREF /2VS                                    */
    agc_dp_t vref;        /* VREF /2VS                                       */
    agc_dp_t rollc;       /* ROLLC roll command, revs                        */
    agc_dp_t thetah;      /* THETAH desired range, revs                      */
    agc_dp_t v1, vl, vs1; /* V1 VL VS1 /2VS                                  */
    agc_dp_t vbars;       /* VBARS VL^2/VS^2 /4                              */
    agc_dp_t tem1b;       /* TEM1B (HUNTEST) L/D, scale 1                    */
    agc_dp_t ahookdv;     /* AHOOKDV /16                                     */
    agc_dp_t time_rto;    /* TIME/RTO time of RTINIT, cs B28                 */
    agc_dp_t dtearot;     /* DTEAROT time RTINIT -> RT, cs B28               */
    agc_dp_t rtgon67;     /* RTGON67 range to go, revs (neg = short)         */
    agc_dp_t dnrngerr;    /* DNRNGERR (PREDANG - THETAH), revs               */
    agc_sp_t asps_tm[5];  /* ASPS(TM) high words of ASKEP..ASPDWN (TM)       */
    /* -- final phase table look-up (SP) -------------------------------- */
    agc_sp_t predang;     /* PREDANG predicted range /2700 NM               */
    agc_sp_t fx[6];       /* FX..FX+5: F3, AREF, RTOGO, RDOTREF, F2, F1     */
    agc_sp_t grad;        /* GRAD interpolation factor                       */
    int      jj;          /* JJ   table index 0..12                          */
    /* -- pad loads and DSKY inputs ------------------------------------- */
    agc_sp_t headsup;     /* HEADSUP +1 lift down, -1 lift up                */
    agc_sp_t ladpad;      /* LADPAD nominal L/D (.3), scale 1                */
    agc_sp_t lodpad;      /* LODPAD final phase L/D (.18), scale 1           */
    agc_sp_t alfapad;     /* ALFAPAD trim alfa /180 (negative)               */
    agc_dp_t lat_spl;     /* LAT(SPL) target latitude, revs                  */
    agc_dp_t lng_spl;     /* LNG(SPL) target longitude, revs                 */
    agc_dp_t emsalt;      /* EMSALT .05 g altitude above Fischer, m B29      */
    /* -- sequencing ---------------------------------------------------- */
    agc_entry_goto_t    gotoaddr;  /* GOTOADDR                               */
    agc_entry_posexit_t posexit;   /* POSEXIT                                */
    uint8_t  entry_verb;  /* ENTRYVN verb for the entry display              */
    uint8_t  entry_noun;  /* ENTRYVN noun                                    */
    /* -- displays (P61, entry) ----------------------------------------- */
    agc_dp_t vmagi;       /* VMAGI inertial speed, m/cs B7                   */
    agc_dp_t gmax;        /* GMAX 100 gmax B-14 g                            */
    agc_dp_t vpred;       /* VPRED velocity at 400 kft, m/cs B7              */
    agc_dp_t gammaei;     /* GAMMAEI gamma at 400 kft, revs                  */
    agc_dp_t rtgo;        /* RTGO range from EMSALT, revs                    */
    agc_dp_t vio;         /* VIO velocity at EMSALT, m/cs B7                 */
    agc_dp_t tte;         /* TTE  time to EMSALT, cs B28 (negative)          */
    agc_dp_t tte1;        /* TTE1 base for TTE count down, cs B28            */
    agc_dp_t lat, lng, alt; /* LAT LONG present position (P67), revs / m B29 */
    agc_sp_t cphi[3];     /* CPHI desired gimbal angles (N22), /180          */
    /* -- body attitude (CM_BODY_ATTITUDE, SP 2s comp /180 unless noted) - */
    agc_sp_t aog_pip, aig_pip, amg_pip; /* CDUs at PIPA time             */
    agc_sp_t roll_pip, alfa_pip, beta_pip; /* DAP angles at PIPA time    */
    agc_sp_t gama;        /* GAMA flight path angle /360                     */
    agc_sp_t gamdot;      /* GAMDOT gamma dot x TCDU /180                    */
    agc_sp_t roll_180, alfa_180, beta_180; /* ROLL/180 ALFA/180 BETA/180    */
    agc_sp_t rolltm;      /* ROLLTM roll /180 for TM                         */
    /* -- entry DAP values shared with guidance and P62 ------------------- */
    agc_sp_t rollhold;    /* ROLLHOLD roll /180 for attitude hold            */
    agc_sp_t p63flag;     /* P63FLAG -1, +0, +1                              */
    agc_entry_dapmod_t cmdapmod; /* CMDAPMOD                                 */
    agc_sp_t alfacom, betacom;   /* ALFACOM BETACOM /180                     */
    agc_sp_t calfa, salfa;       /* CALFA SALFA cos/sin alfa, scale 1       */
} agc_entry_state_t;

/* ---- lexicon (entry_lexicon.c) ----------------------------------------- */
agc_entry_state_t *agc_entry_state(void);       /* the one entry erasable set */
void               agc_entry_lexicon_reset(void);

/* ---- entry guidance (reentry.c) ----------------------------------------- */
agc_status_t agc_entry_startent(void);   /* STARTENT  first guidance pass    */
agc_status_t agc_entry_scalepop(void);   /* SCALEPOP  TARGETNG + sequencer   */
agc_status_t agc_entry_targeting(void);  /* TARGETNG                         */
agc_status_t agc_entry_pre_hunt(void);   /* PRE-HUNT  restart job (group 4)  */
agc_status_t agc_entry_steeroff_lat_long(void); /* P67.2 + P67.3            */

/* HUNTEST and RANGER without the sequencing side effects (unit tests and
 * the oracle). huntest_predict runs HUNTEST from the current state up to
 * HUNTEST3 and returns GAMMAL; range_predict runs RANGER on that GAMMAL
 * and leaves DIFF = (ASP - THETAH), revs, in *diff.                        */
typedef enum agc_entry_hunt_result {
    AGC_ENTRY_HUNT_RANGER = 0,    /* solution found: continue at RANGER     */
    AGC_ENTRY_HUNT_PREFINAL,      /* VL < VLMIN: go to final phase          */
    AGC_ENTRY_HUNT_CONSTD         /* VL > VSAT: constant drag               */
} agc_entry_hunt_result_t;
agc_status_t agc_entry_huntest_predict(agc_entry_hunt_result_t *result,
                                       agc_dp_t *gammal);
agc_status_t agc_entry_range_predict(agc_dp_t gammal, agc_dp_t *diff);

/* ---- body attitude (cm_body_attitude.c) --------------------------------- */
agc_status_t agc_entry_cm_pose(void);    /* CM/POSE: AVEGEXIT during entry   */

/* ---- P61-P67 (p61_p67.c) ------------------------------------------------ */
agc_status_t agc_entry_p61(void);        /* P61      V37 entry               */
agc_status_t agc_entry_p61_4(void);      /* P61.4    after N61 PROCEED       */
agc_status_t agc_entry_p61_1(void);      /* P61.1    N60 display             */
agc_status_t agc_entry_p61_2(void);      /* P61.2    N63 display             */
agc_status_t agc_entry_p61_3(void);      /* P61.3    phase change, ENDOFJOB  */
agc_status_t agc_entry_p62(void);        /* P62                              */
agc_status_t agc_entry_p62_2(void);      /* P62.2    separation request      */
agc_status_t agc_entry_p62_2b(void);     /* P62.2 +3 PROCEED: CM/DAPON       */
agc_status_t agc_entry_p62_1(void);      /* P62.1    N61 then entry attitude */
agc_status_t agc_entry_p63(void);        /* P63                              */
agc_status_t agc_entry_p65_1(void);      /* P65.1    N69 flashing display    */
agc_status_t agc_entry_p65_1b(void);     /* P65.1    PROCEED                 */
agc_status_t agc_entry_p67_1(void);      /* P67.1    N67 flashing display    */
agc_status_t agc_entry_p67_1b(void);     /* P67.1    PROCEED: end entry      */
agc_status_t agc_entry_s61_1(void);      /* S61.1    state vector, IMU check */
agc_status_t agc_entry_s61_1a(void);     /* S61.1A   IMU orientation check   */
agc_status_t agc_entry_s61_2(void);      /* S61.2    N60/N63 data (stub)     */
agc_status_t agc_entry_s62_3(void);      /* S62.3    desired gimbal angles   */
agc_status_t agc_entry_wakep62(void);    /* WAKEP62  task                    */

/* ---- entry DAP (entry_dap.c) -------------------------------------------- */
agc_status_t agc_entry_dap_readgymb(void); /* READGYMB  .1 s task            */
agc_status_t agc_entry_dap_setjtag(void);  /* SETJTAG   2 s task             */
agc_status_t agc_entry_dap_jetcall(void);  /* JETCALL   roll jet task        */
agc_status_t agc_entry_dap_on(void);       /* CM/DAPON  job                  */
agc_status_t agc_entry_dap_ic(void);       /* CM/DAPIC  DAP initialisation   */
agc_status_t agc_entry_dap_flushjet(void); /* FLUSHJET  all jets off         */

/* Roll phase-plane solution of CM/RCS (p. 1081-1086) for a roll error and
 * roll rate, as a pure function (used by CM/RCS and by unit tests).       */
typedef struct agc_entry_roll_cmd {
    agc_sp_t ton1;        /* TON1 first jet-on interval, cs (<= 0: none)    */
    agc_sp_t toff;        /* TOFF drift interval, cs                         */
    agc_sp_t ton2;        /* TON2 second jet-on interval, cs                 */
    int      jndx;        /* JNDX  -1 = positive roll jets, +1 = negative    */
    int      jndx1;       /* JNDX1 jet index for TON2                        */
    bool     in_deadzone; /* DZCALL taken: all roll jets off                 */
} agc_entry_roll_cmd_t;
/* lcx_360: roll error (ROLLC - ROLL)/360, already corrected for cos alfa;
 * mvt_180: -2 V T/180 (roll rate term as formed at CM/RCS entry).          */
agc_status_t agc_entry_dap_roll_phase_plane(agc_sp_t lcx_360, agc_sp_t mvt_180,
                                            agc_entry_roll_cmd_t *out);
uint16_t agc_entry_dap_roll_jet_bits(int jndx); /* P/RJCODE[jndx]            */

/* ---- P37 return to earth (p37.c) ---------------------------------------- */
agc_status_t agc_p37(void);
agc_status_t agc_p37_xt1lim(agc_dp_t ma, agc_dp_t rcon, agc_dp_t r_t1,
                            agc_dp_t beta5, agc_dp_t *xt1lim);
agc_status_t agc_p37_tig_bias(agc_dp_t dv, agc_sp_t option2, agc_dp_t t1,
                              agc_dp_t *tig);

/* ---- P11 earth orbit insertion monitor (p11.c) -------------------------- */
agc_status_t agc_p11(void);
agc_status_t agc_p11_matrxjob(void);
agc_status_t agc_p11_vhhdot(void);
agc_status_t agc_p11_aterjob(void);
agc_status_t agc_p11_s11_1(agc_dp_t *vmagi, agc_dp_t *hdot, agc_dp_t *alti);

#endif /* AGC_CSM_ENTRY_H */
