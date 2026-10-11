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

#include "agc/core/interp.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/nav/auxiliary.h"

/* GOTOADDR: the entry phase sequencer (REENTRY_CONTROL p. 847).          */
typedef enum agc_entry_goto
{
    AGC_ENTRY_GOTO_INITROLL = 0, /* P63/P64 initial roll                  */
    AGC_ENTRY_GOTO_HUNTEST,      /* P64 up-control solution search        */
    AGC_ENTRY_GOTO_UPCONTRL,     /* P65 up-control                        */
    AGC_ENTRY_GOTO_KEP2,         /* P66 ballistic (Kepler) phase          */
    AGC_ENTRY_GOTO_PREFINAL,     /* entry to P67 final phase              */
    AGC_ENTRY_GOTO_PREDICT3,     /* P67 final phase                       */
    AGC_ENTRY_GOTO_ENDEXIT,      /* sidetracked: skip steering this pass  */
    AGC_ENTRY_GOTO_FINAL_PHASE_LATLONG, /* after VQUIT: lat/long only            */
    AGC_ENTRY_GOTO_COUNT
} agc_entry_goto_t;

/* POSEXIT: where CM/POSE continues after the attitude update (p. 888).   */
typedef enum agc_entry_posexit
{
    AGC_ENTRY_POSEXIT_DESIRED_GIMBALS =
        0,                      /* desired gimbal angles (P62, KEP2)     */
    AGC_ENTRY_POSEXIT_STARTENT, /* first pass of entry guidance (P63)    */
    AGC_ENTRY_POSEXIT_SCALEPOP, /* normal entry guidance pass            */
    AGC_ENTRY_POSEXIT_ENDEXIT,  /* attitude only, then entry display     */
    AGC_ENTRY_POSEXIT_COUNT
} agc_entry_posexit_t;

/* CMDAPMOD (p. 1077). The AGC distinguishes +0 and -0; -0 is modelled as
 * its own value (DEVIATION, see docs/modules/guid-entry.md).             */
typedef enum agc_entry_dapmod
{
    AGC_ENTRY_DAPMOD_MINUS1 = -1, /* in atmosphere (D > .05 g)             */
    AGC_ENTRY_DAPMOD_PLUS0  =
        0, /* ext. atmosphere, |cos alfa| > .707, cos alfa > 0 */
    AGC_ENTRY_DAPMOD_PLUS1  = 1, /* ext. atmosphere, |cos alfa| <= .707   */
    AGC_ENTRY_DAPMOD_MINUS0 =
        2 /* ext. atmosphere, cos alfa < 0: damping only */
} agc_entry_dapmod_t;

/* Entry erasables (ENTRY_LEXICON.agc). DP = agc_dp_t, SP = agc_sp_t.      */
typedef struct agc_entry_state
{
    /* -- target and trajectory vectors, reference coordinates ---------- */
    agc_vec_t rtinit; /* RTINIT  initial target vector, .5 unit         */
    agc_vec_t rteast; /* RTEAST  east at RTINIT, .5 unit                */
    agc_vec_t rtnorm; /* RTNORM  normal to RTEAST and UZ, .5 unit       */
    agc_vec_t rt;     /* RT      target vector (earth-rotated), .5 unit */
    agc_vec_t uni;    /* UNI     unit normal to trajectory plane, .5 unit */
    agc_vec_t unitv;  /* UNITV   unit V, .5 unit                        */
    agc_vec_t vel;    /* VEL     velocity, scale 2VS                    */
    agc_vec_t mvrel;  /* -VREL   negative velocity rel. atmosphere, 2VS */
    agc_vec_t uxa, uya, uza; /* UXA/2.. trajectory triad, .5 unit        */
    agc_vec_t olduya; /* OLDUYA  UYA saved for V < VQUIT, .5 unit       */
    agc_vec_t ubx, uby, ubz; /* UBX/2.. CM body triad, .5 unit           */
    /* -- guidance scalars (DP) ------------------------------------------ */
    agc_dp_t vsquare; /* VSQUARE V^2/VS^2, scale 4                       */
    agc_dp_t leq;     /* LEQ  (VSQ-1) g, scale 4 g                       */
    agc_dp_t v;       /* V    velocity magnitude /2VS                    */
    agc_dp_t rdot;    /* RDOT altitude rate /2VS                         */
    agc_dp_t d;       /* D    drag acceleration /805 fpss                */
    agc_dp_t a0, a1;  /* A0 A1 drag for upcontrol /805                   */
    agc_dp_t alp;     /* ALP  upcontrol constant, scale 1                */
    agc_dp_t askep, asp1, aspup, asp3, aspdwn; /* range terms, revs        */
    agc_dp_t cosg_2; /* COSG/2                                          */
    agc_dp_t c_d0;   /* C/D0 = (-4/D0) B-8                              */
    agc_dp_t d0;     /* D0   controlled constant drag /805              */
    agc_dp_t dhook;  /* DHOOK /805                                      */
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
    agc_dp_t
             range_to_target; /* RTGON67 range to go, revs (neg = short)         */
    agc_dp_t dnrngerr;   /* DNRNGERR (PREDANG - THETAH), revs               */
    agc_sp_t asps_tm[5]; /* ASPS(TM) high words of ASKEP..ASPDWN (TM)       */
    /* -- final phase table look-up (SP) -------------------------------- */
    agc_sp_t predang; /* PREDANG predicted range /2700 NM               */
    agc_sp_t fx[6];   /* FX..FX+5: F3, AREF, RTOGO, RDOTREF, F2, F1     */
    agc_sp_t grad;    /* GRAD interpolation factor                       */
    int      jj;      /* JJ   table index 0..12                          */
    /* -- pad loads and DSKY inputs ------------------------------------- */
    agc_sp_t headsup; /* HEADSUP +1 lift down, -1 lift up                */
    agc_sp_t ladpad;  /* LADPAD nominal L/D (.3), scale 1                */
    agc_sp_t lodpad;  /* LODPAD final phase L/D (.18), scale 1           */
    agc_sp_t alfapad; /* ALFAPAD trim alfa /180 (negative)               */
    agc_dp_t lat_spl; /* LAT(SPL) target latitude, revs                  */
    agc_dp_t lng_spl; /* LNG(SPL) target longitude, revs                 */
    agc_dp_t emsalt;  /* EMSALT .05 g altitude above Fischer, m B29      */
    /* -- sequencing ---------------------------------------------------- */
    agc_entry_goto_t    gotoaddr; /* GOTOADDR                               */
    agc_entry_posexit_t posexit;  /* POSEXIT                                */
    uint8_t             entry_verb; /* ENTRYVN verb for the entry display              */
    uint8_t             entry_noun; /* ENTRYVN noun                                    */
    /* -- displays (P61, entry) ----------------------------------------- */
    agc_dp_t vmagi;   /* VMAGI inertial speed, m/cs B7                   */
    agc_dp_t gmax;    /* GMAX 100 gmax B-14 g                            */
    agc_dp_t vpred;   /* VPRED velocity at 400 kft, m/cs B7              */
    agc_dp_t gammaei; /* GAMMAEI gamma at 400 kft, revs                  */
    agc_dp_t rtgo;    /* RTGO range from EMSALT, revs                    */
    agc_dp_t vio;     /* VIO velocity at EMSALT, m/cs B7                 */
    agc_dp_t tte;     /* TTE  time to EMSALT, cs B28 (negative)          */
    agc_dp_t tte1;    /* TTE1 base for TTE count down, cs B28            */
    agc_dp_t lat, lng, alt; /* LAT LONG present position (P67), revs / m B29 */
    agc_sp_t cphi[3]; /* CPHI desired gimbal angles (N22), /180          */
    /* -- body attitude (CM_BODY_ATTITUDE, SP 2s comp /180 unless noted) - */
    agc_sp_t aog_pip, aig_pip, amg_pip;    /* CDUs at PIPA time             */
    agc_sp_t roll_pip, alfa_pip, beta_pip; /* DAP angles at PIPA time    */
    agc_sp_t gama;   /* GAMA flight path angle /360                     */
    agc_sp_t gamdot; /* GAMDOT gamma dot x TCDU /180                    */
    agc_sp_t roll_180, alfa_180, beta_180; /* ROLL/180 ALFA/180 BETA/180    */
    agc_sp_t rolltm; /* ROLLTM roll /180 for TM                         */
    /* -- entry DAP values shared with guidance and P62 ------------------- */
    agc_sp_t rollhold; /* ROLLHOLD roll /180 for attitude hold            */
    agc_sp_t
                       wake_separation_interlock; /* P63FLAG -1, +0, +1                              */
    agc_entry_dapmod_t cmdapmod; /* CMDAPMOD                                 */
    agc_sp_t           alfacom, betacom; /* ALFACOM BETACOM /180                     */
    agc_sp_t           calfa, salfa;     /* CALFA SALFA cos/sin alfa, scale 1       */
} agc_entry_state_t;

/* ---- lexicon (lexicon.c) ----------------------------------------- */
agc_entry_state_t *agc_entry_state(void); /* the one entry erasable set */
void               agc_entry_lexicon_reset(void);

/* ---- entry guidance (reentry.c) -----------------------------------------
 * The steering runs inside the CM/POSE job (AVGEXIT during entry). The
 * pass functions return *display = true when the AGC ends at ENDEXIT
 * (entry display ENTRYVN, then SERVNOUT) and false for SERVNOUT alone.    */
agc_status_t agc_entry_startent(void); /* STARTENT first pass  */
agc_status_t agc_entry_scalepop(bool *display); /* SCALEPOP: TARGETNG + REFAZE10 */
agc_status_t agc_entry_refaze10(bool *display);    /* REFAZE10: GOTO GOTOADDR */
agc_status_t agc_entry_targeting(void);            /* TARGETNG             */
agc_status_t agc_job_pre_hunt(agc_job_ctx_t *ctx); /* PRE-HUNT  4.47 restart */
agc_status_t agc_job_refaze10(agc_job_ctx_t *ctx); /* REFAZE10  5.3 restart  */

/* HUNTEST and RANGER without the sequencing side effects (unit tests and
 * the oracle). huntest_predict runs HUNTEST from the current state up to
 * HUNTEST3 and returns GAMMAL; range_predict runs RANGER on that GAMMAL
 * and leaves DIFF = (ASP - THETAH), revs, in *diff.                        */
typedef enum agc_entry_hunt_result
{
    AGC_ENTRY_HUNT_RANGER = 0, /* solution found: continue at RANGER     */
    AGC_ENTRY_HUNT_PREFINAL,   /* VL < VLMIN: go to final phase          */
    AGC_ENTRY_HUNT_CONSTD      /* VL > VSAT: constant drag               */
} agc_entry_hunt_result_t;

agc_status_t agc_entry_huntest_predict(agc_entry_hunt_result_t *result,
                                       agc_dp_t                *gammal);
agc_status_t agc_entry_range_predict(agc_dp_t gammal, agc_dp_t *diff);

/* ---- body attitude (cm_body_attitude.c) --------------------------------- */
agc_status_t agc_job_cm_pose(agc_job_ctx_t *ctx);  /* CM/POSE (AVGEXIT)    */
agc_status_t agc_job_redopose(agc_job_ctx_t *ctx); /* REDOPOSE 5.x restart */
/* CM/POSE, CM/POSE2: trajectory triad UXA UYA UZA and -VREL (no CDUs).    */
agc_status_t agc_entry_cm_pose_triad(void);
/* CM/TRIO: body triad and -(ROLL, BETA, ALFA)/180 /2 at PIPA time, and
 * -GAMA/180 /2, from AOG/AIG/AMG at PIP and REFSMMAT; angles[0..3] DP.    */
agc_status_t agc_entry_cm_trio(agc_dp_t angles[4]);

/* ---- P61-P67 (programs.c) ------------------------------------------------
 * Jobs: the AGC program text split at every DSKY / sleep resumption.      */
agc_status_t agc_job_entry_preparation(agc_job_ctx_t *ctx); /* P61                 */
agc_status_t agc_job_entry_preparation_target_display(
    agc_job_ctx_t *ctx); /* P61 V06N61          */
agc_status_t agc_job_entry_preparation_compute(agc_job_ctx_t *ctx); /* P61.4               */
agc_status_t agc_job_entry_preparation_prediction_display(
    agc_job_ctx_t *ctx); /* P61.1 V06N60        */
agc_status_t agc_job_entry_preparation_ems_display(
    agc_job_ctx_t *ctx); /* P61.2 V06N63        */
agc_status_t agc_job_entry_display_protect(agc_job_ctx_t *ctx); /* P61.3               */
agc_status_t agc_job_cm_sm_separation_newmode(agc_job_ctx_t *ctx); /* P62 -3 (from P61)   */
agc_status_t agc_job_cm_sm_separation(agc_job_ctx_t *ctx); /* P62 (V37)           */
agc_status_t agc_job_cm_sm_separation_after_check(
    agc_job_ctx_t *ctx); /* P62 after S61.1     */
agc_status_t agc_job_cm_sm_separation_request(agc_job_ctx_t *ctx); /* P62.2 GOPERF1R 41   */
agc_status_t agc_job_cm_sm_separation_proceed(agc_job_ctx_t *ctx); /* P62.2 +3 CM/DAPON   */
agc_status_t agc_job_cm_sm_separation_target_display(
    agc_job_ctx_t *ctx); /* P62.1 V06N61        */
agc_status_t agc_job_cm_sm_separation_end_hold(agc_job_ctx_t *ctx); /* P62.1 PROCEED       */
agc_status_t agc_job_entry_initialization(agc_job_ctx_t *ctx); /* P63                 */
agc_status_t agc_job_entry_upcontrol_display(agc_job_ctx_t *ctx); /* P65.1 V16N69        */
agc_status_t agc_job_entry_upcontrol_proceed(agc_job_ctx_t *ctx); /* P65.1 PROCEED       */
agc_status_t agc_job_entry_final_phase_display(agc_job_ctx_t *ctx); /* P67.1 V16N67        */
agc_status_t agc_job_entry_final_phase_end(agc_job_ctx_t *ctx); /* P67.1 PROCEED       */
agc_status_t agc_job_entry_imu_check(
    agc_job_ctx_t *ctx); /* S61.1 (return job set by caller) */
agc_status_t agc_job_entry_imu_axis_test(agc_job_ctx_t *ctx); /* S61.1A  4.45        */
agc_status_t agc_job_entry_imu_check_extrapolated(
    agc_job_ctx_t *ctx); /* S61.1 MIDTOAV2 retry*/
agc_status_t agc_job_entry_imu_check_return(agc_job_ctx_t *ctx); /* RETRN1 after delay  */
agc_status_t agc_task_entry_start_average_g(void);  /* S61.1C  4.43        */
agc_status_t agc_task_wake_cm_sm_separation(void);  /* WAKEP62 4.33        */
agc_status_t agc_entry_ems_data(void);              /* S61.2 N60/N63 data  */
agc_status_t agc_entry_desired_gimbal_angles(void); /* S62.3 CPHI for N22  */
agc_status_t agc_entry_final_phase_latlong(void);   /* P67.2 LAT LONG ALT  */
/* S61.2 helpers (pure, used by the oracle).                               */
agc_dp_t     agc_entry_vrcalc(agc_vec_t urh, agc_vec_t rt); /* VRCALC, revs */
agc_status_t agc_entry_gmaxcalc(agc_dp_t vgam, agc_dp_t gam,
                                agc_dp_t *gmax); /* GMAX 100g B-14      */
/* TFF subroutines on the debris CALCTFF leaves in *t (also DELRSPL's).  */
agc_dp_t agc_entry_tff_trig(agc_interp_t *ip, const agc_tff_t *t,
                            agc_dp_t *cdelf2); /* TFF/TRIG: .5 sin   */
agc_dp_t agc_entry_vgamcalc(agc_interp_t *ip, agc_tff_t *t, agc_dp_t rterm,
                            agc_dp_t *vgam); /* PREVGAM: gamma, revs */

/* ---- entry DAP (dap.c) -------------------------------------------- */
agc_status_t agc_task_readgymb(void); /* READGYMB  .1 s task             */
agc_status_t agc_task_attrates(void); /* ATTRATES  T5RUPT after BODYRATE */
agc_status_t agc_task_setjtag(void);               /* SETJTAG   2 s task (READACCS)   */
agc_status_t agc_task_jetcall(void);               /* JETCALL   roll jet task         */
agc_status_t agc_job_cm_dapon(agc_job_ctx_t *ctx); /* CM/DAPON        */
agc_status_t agc_job_cm_dapon_wait(agc_job_ctx_t *ctx); /* NOTYET +4       */
agc_status_t agc_entry_dap_ic(agc_job_ctx_t *ctx); /* CM/DAPIC        */
void         agc_entry_dap_flushjet(void);         /* FLUSHJET        */

/* Roll phase-plane solution of CM/RCS (pp. 1081-1086) for a roll error and
 * roll rate, as a pure function (used by CM/RCS and by unit tests).       */
typedef struct agc_entry_roll_cmd
{
    agc_sp_t ton1;        /* TON1 first jet-on interval, cs (<= 0: none)    */
    agc_sp_t toff;        /* TOFF drift interval, cs                         */
    agc_sp_t ton2;        /* TON2 second jet-on interval, cs                 */
    int      jndx;        /* JNDX  -1 = positive roll jets, +1 = negative    */
    int      jndx1;       /* JNDX1 jet index for TON2                        */
    bool     in_deadzone; /* DZCALL taken: all roll jets off                 */
} agc_entry_roll_cmd_t;

/* lcx_360: roll error (ROLLC - ROLL)/360, already corrected for cos alfa
 * (COMPAT); mvt_180: -2 V T/180 as formed at CM/RCS entry.                */
agc_status_t agc_entry_dap_roll_phase_plane(agc_sp_t lcx_360, agc_sp_t mvt_180,
                                            agc_entry_roll_cmd_t *out);
uint16_t agc_entry_dap_roll_jet_bits(int jndx); /* P/RJCODE[jndx]           */
uint16_t agc_entry_dap_yaw_jet_bits(int jndx);  /* YJETCODE[jndx]           */

/* Entry DAP erasables other lanes read (AOG = BCDU in EBANK6, CM/GYMDT). */
agc_sp_t agc_erase_aog(void);      /* AOG outer gimbal /180, 2s comp  */
agc_sp_t agc_erase_aig(void);      /* AIG inner gimbal                */
agc_sp_t agc_erase_amg(void);      /* AMG middle gimbal               */

/* ---- P37 return to earth (p37.c) ---------------------------------------- */
agc_status_t agc_job_return_to_earth(agc_job_ctx_t *ctx); /* P37                 */
/* Noun data RTEDVD, RTEGAM2D (N56) of P37; NOT_FOUND for other ids.     */
agc_status_t agc_return_to_earth_noun(uint16_t data_id, bool write,
                                      agc_dp_t *value);
/* P37 continuation jobs (src/csm/burns/return_to_earth/program.c).     */
agc_status_t agc_job_return_to_earth_tig_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_entry_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_conic_search(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_transit_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_entry_conditions_display(
    agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_delta_v_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_precision(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_precision_resume(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_propulsion_option(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_option_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_ignition_time(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_biased_tig_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_targeting(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_target_store(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_alarm_display(agc_job_ctx_t *ctx);

/* XT1LIM (p. 930): X(T1) limit (B5) from MA (B30), RCON and R(T1) (B29),
 * BETA5 (B9).                                                             */
agc_status_t agc_return_to_earth_xt1lim(agc_dp_t ma, agc_dp_t rcon,
                                        agc_dp_t r_t1, agc_dp_t beta5,
                                        agc_dp_t *xt1lim);

/* ---- P11 earth orbit insertion monitor (earth_orbit_insertion.c) -------------------------- */
agc_status_t agc_job_earth_orbit_insertion_monitor(
    agc_job_ctx_t *ctx); /* P11                 */
agc_status_t agc_job_earth_orbit_insertion_monitor_restart(
    agc_job_ctx_t *ctx); /* REP11  3.11         */
agc_status_t agc_job_earth_orbit_insertion_monitor_ephemeris(
    agc_job_ctx_t *ctx);                           /* REP11A 3.13         */
agc_status_t agc_job_matrxjob(agc_job_ctx_t *ctx); /* MATRXJOB 3.7        */
agc_status_t agc_job_vhhdot(agc_job_ctx_t *ctx);   /* VHHDOT (AVGEXIT)    */
agc_status_t agc_job_aterjob(agc_job_ctx_t *ctx);  /* ATERJOB             */
agc_status_t agc_task_atertask(void);              /* ATERTASK 4.51       */
agc_status_t agc_task_redosat(void);               /* REDOSAT (T5)        */
agc_status_t agc_task_satstick(void);              /* SATSTICK (T5)       */
agc_status_t agc_csm_satstkon(const agc_job_ctx_t *ctx); /* SATSTKON (V46)  */
/* S11.1: VMAGI = |VN| (m/cs B7), HDOT = UNIT(RN).VN (B7), ALTI = |RN| -
 * RPAD (earth) or - |RLS| (moon, AMOONFLG) (m B29), stored for N62;
 * agc_earth_orbit_insertion_monitor_s11_1 reads the stored values (noun tables).                    */
void         agc_csm_velocity_altitude_rate(agc_interp_t *ip);
agc_status_t agc_earth_orbit_insertion_monitor_readout(agc_dp_t *vmagi,
                                                       agc_dp_t *hdot,
                                                       agc_dp_t *alti);
agc_status_t agc_job_earth_orbit_insertion_monitor_preread(
    agc_job_ctx_t *ctx); /* P11 TEPHEM restart */
agc_status_t agc_job_earth_orbit_insertion_monitor_setup(
    agc_job_ctx_t *ctx); /* P11 after PREREAD1 */
agc_status_t agc_job_atreset_after_delay(agc_job_ctx_t *ctx); /* ATRESET +5        */

/* P11 pad loads (ERASABLE_ASSIGNMENTS: P11 storage, SATURN BOOST storage). */
#define AGC_EARTH_ORBIT_INSERTION_MONITOR_POLY_MAX 6

typedef struct agc_earth_orbit_insertion_monitor_padload
{
    agc_dp_t latitude; /* LATITUDE geodetic latitude of the pad, revs     */
    agc_dp_t padlong;  /* PADLONG  longitude of the pad, revs             */
    agc_dp_t pgncsalt; /* PGNCSALT altitude of the PGNCS, m B29           */
    agc_dp_t launchaz; /* LAUNCHAZ launch azimuth (X SM), revs            */
    agc_dp_t azimuth;  /* AZIMUTH  -(north to Z S/C), revs                */
    agc_dp_t satrlrt;  /* SATRLRT  Saturn roll rate, rev/cs B-? (pad)     */
    agc_sp_t rpstart;  /* RPSTART  pitch/roll start, cs after liftoff     */
    agc_sp_t polystop; /* POLYSTOP -(polynomial cutoff - RPSTART), cs     */
    agc_sp_t polynum;  /* POLYNUM  polynomial degree N (<= 6)             */
    agc_dp_t poly[AGC_EARTH_ORBIT_INSERTION_MONITOR_POLY_MAX +
                  1];    /* A0..A6 pitch, 32 revs (B5)      */
    agc_sp_t satscale;   /* SATSCALE Saturn steering scale factor           */
    agc_sp_t satrate[4]; /* SATRATE  Saturn stick rates (AK units) indexed by
                             the STICKCHK index 0 none, 1 +, 2 -, 3 both     */
} agc_earth_orbit_insertion_monitor_padload_t;

agc_status_t agc_earth_orbit_insertion_monitor_set_padload(
    const agc_earth_orbit_insertion_monitor_padload_t *pad);
/* ATERJOB attitude-error kernel (pp. 543-545): desired gimbal angles
 * (OGC IGC MGC, DP revs) and present CDUs -> AK AK1 AK2 (SP, error/180).  */
void agc_earth_orbit_insertion_monitor_attitude_errors(
    const agc_dp_t desired[3], const uint16_t cdu[3], agc_sp_t ak[3]);

/* Priorities: PRIOnn octal (agc_prio_t = PRIOnn >> 9, see agc_exec.h).     */
#define AGC_ENTRY_PRIO1  ((agc_prio_t) 001)
#define AGC_ENTRY_PRIO13 ((agc_prio_t) 013)
#define AGC_ENTRY_PRIO16 ((agc_prio_t) 016)
#define AGC_ENTRY_PRIO20 ((agc_prio_t) 020)

/* ---- telemetry (sys-misc) -------------------------------------------- */
#define AGC_ENTRY_DNLSTCOD_ENTRY   1 /* DNLSTCOD 1: ENTRY/UPDATE list     */
#define AGC_ENTRY_DNLSTCOD_POWERED 3 /* POWDNCOD = THREE                  */

/* ======================================================================
 * Helpers shared by the entry files (src/csm/entry); formerly entry_internal.h
 * ==================================================================== */
/* reentry.c */
agc_interp_t *entry_ip(void);            /* shared interpreter state (OVFIND) */
agc_dp_t      entry_signmpac(agc_dp_t x); /* SIGNMPAC: +-DPOSMAX               */
agc_dp_t      entry_log(agc_interp_t *ip,
                        agc_dp_t      x); /* LOG: -ln(x)/32               */
void entry_earrot(agc_interp_t *ip, agc_entry_state_t *es,
                  bool first);         /* EARROT1/2 */
agc_status_t entry_endexit(agc_job_ctx_t     *ctx,
                           agc_entry_state_t *es); /* OVERNOUT */
agc_status_t entry_starten1(agc_interp_t      *ip,
                            agc_entry_state_t *es); /* STARTEN1 */

/* cm_body_attitude.c: SERVNOUT (CHANG1 if a job waits, then SERVEXIT).  */
agc_status_t entry_servnout(agc_job_ctx_t *ctx);

/* programs.c: mode/display changes called from the steering via RTB.        */
agc_status_t entry_post_0_05g(agc_entry_state_t *es);
agc_status_t entry_upcontrol(agc_entry_state_t *es);
agc_status_t entry_ballistic(agc_entry_state_t *es);
agc_status_t entry_final_phase(agc_entry_state_t *es);
agc_status_t entry_killgrp4(void);

/* dap.c */
/* ANGOVCOR / CORANGOV: an SP angle/180 sum (at most one overflow) wrapped
 * back into [-1, 1) (i.e. modulo 360 deg).                               */
agc_sp_t entry_angle_wrap(int32_t sum);
agc_sp_t entry_sp_mp(agc_sp_t a, agc_sp_t b); /* MP: high word of a b      */
agc_sp_t entry_sp_sat(int32_t sum);           /* AD whose result is kept
                                                  (overflow not expected)  */

#endif /* AGC_CSM_ENTRY_H */
