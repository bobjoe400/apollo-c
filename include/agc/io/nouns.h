/*
 * agc_nouns.h -- PINBALL noun tables, scale types and noun data resolution.
 *
 * Source: PINBALL_NOUN_TABLES.agc (Comanche055 pp. 268-284, Luminary099
 * pp. 301-319). The AGC keeps four parallel tables (NNADTAB, NNTYPTAB,
 * IDADDTAB, RUTMXTAB) plus the SFINTAB/SFOUTAB scale-factor constants; here
 * each noun is resolved into one flat agc_noun_t.
 *
 * Consumed by io-pinball (src/io). Implemented in src/io/dsky/noun_tables_csm.c,
 * noun_tables_lm.c, noun_scales.c and noun_data.c (agent io-nouns).
 *
 * Component conventions
 * ---------------------
 *  - ncomp is the number of display registers used (0 for a spare noun).
 *  - scale_type[k] / data_id[k] describe component k (R1..R3). Unused
 *    components hold AGC_SCALE_NONE / AGC_DATA_NONE.
 *  - HMS nouns (scale AGC_SCALE_HMS) are a single DP centisecond word shown
 *    across R1,R2,R3: scale_type[0..2] are all AGC_SCALE_HMS but only
 *    data_id[0] is set; data_id[1], data_id[2] are AGC_DATA_NONE.
 *  - Value convention of agc_noun_read()/agc_noun_write(), by data kind
 *    (the id's comment in agc_noun_data_t and agc_scale_info_t.is_dp/is_raw):
 *      DP    the DP erasable as the agc_dp_t itself;
 *      SP    one SP word in the high word of the agc_dp_t (agc_dp_from_sp /
 *            agc_dp_high);
 *      SP raw  a scale whose AGC word uses all 15 bits (octal words, CDU
 *            angles, 360-CDU, Y optics, channel images -- OCTAL, CDU_DEG,
 *            CDU360, Y_OPTICS_DEG, LR_POSITION): the unsigned word 0..0x7FFF
 *            as a plain integer in the agc_dp_t (no ones'-complement
 *            interpretation, so 0o77777 stays 0o77777). An id used by both
 *            a raw and a signed scale (CSM OPTION2: octal, and WHOLE in noun
 *            72) is raw; the two agree for words 0..0x3FFF.
 *  - Nouns 01-03, 10, 15 do not name a fixed erasable: their data ids are
 *    AGC_DATA_SPECIFY_ADDR / _SPECIFY_CHANNEL / _INCREMENT_ADDR and the
 *    pinball module resolves them itself (agc_noun_read returns NOT_FOUND).
 *  - dec_only / no_load are bits 4 and 5 of the AGC component code.
 */
#ifndef AGC_NOUNS_H
#define AGC_NOUNS_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/core/state.h"

#define AGC_NOUN_COUNT        100   /* nouns 00..99                          */
#define AGC_NOUN_FIRST_MIXED   40   /* MIXCON: first mixed noun              */
#define AGC_NOUN_MAX_COMP       3

/*
 * SF routine code numbers (NNTYPTAB bits 10-6 for normal nouns, 5-bit fields
 * of RUTMXTAB for mixed nouns; PINBALL_NOUN_TABLES.agc "SF ROUTINE CODE
 * NUMBER"). Codes 6 and 13 differ between the vehicles.
 */
typedef enum agc_sf_routine {
    AGC_SFR_OCTAL = 0,              /* octal only */
    AGC_SFR_FRACTION = 1,           /* straight fractional */
    AGC_SFR_CDU_DEG = 2,            /* CDU degrees (xxx.xx) */
    AGC_SFR_ARITH = 3,              /* arithmetic SF */
    AGC_SFR_DP1 = 4,                /* ARITH DP1: out multiply by 2^14 at end, in straight */
    AGC_SFR_DP2 = 5,                /* ARITH DP2: out straight, in SL 7 at end */
    AGC_SFR_Y_OPTICS = 6,           /* CSM: Y optics degrees (xx.xxx max 89.999) */
    AGC_SFR_LR_POSITION = 6,        /* LM: landing radar position (+0000x) */
    AGC_SFR_DP3 = 7,                /* ARITH DP3: out SL 7 at end, in straight */
    AGC_SFR_HMS = 8,                /* hours R1, minutes R2, seconds R3 (alarms if used with octal) */
    AGC_SFR_MIN_SEC = 9,            /* minutes D1D2, blank D3, seconds D4D5 (limits to 59B59) */
    AGC_SFR_DP4 = 10,               /* ARITH DP4: out straight, in SL 3 at end */
    AGC_SFR_ARITH1 = 11,            /* ARITH1 SF: out multiply by 2^14 at end, in straight */
    AGC_SFR_TWO_INT = 12,           /* 2 integers D1D2, D4D5, D3 blank */
    AGC_SFR_DP_FRACTION = 13,       /* CSM: DP straight fractional */
    AGC_SFR_CDU360 = 13,            /* LM: 360-CDU degrees (xxx.xx) */
} agc_sf_routine_t;

/*
 * Scale types: one per (SF routine, SF constant) pair the noun tables use,
 * named after the engineering quantity. The letter is the scale type of
 * ASSEMBLY_AND_OPERATION_INFORMATION "NOUN SCALES AND FORMATS"; the vehicle
 * column says which program defines it. agc_scale_info() gives the AGC
 * routine/constant codes and the SFINTAB/SFOUTAB constants.
 */
typedef enum agc_scale {
    AGC_SCALE_NONE = 0,       /* spare / unused component                  */
    AGC_SCALE_OCTAL,              /* A   CSM+LM xxxxx octal */
    AGC_SCALE_FRACTION,           /* B   CSM+LM .xxxxx straight fractional */
    AGC_SCALE_WHOLE,              /* C   CSM+LM xxxxx. integer */
    AGC_SCALE_CDU_DEG,            /* D   CSM+LM xxx.xx deg (CDU) */
    AGC_SCALE_ELEV_DEG,           /* E   CSM+LM xx.xxx deg (elevation, max 89.999) */
    AGC_SCALE_DEG180,             /* F   CSM+LM xxx.xx deg (180) */
    AGC_SCALE_DP_DEG90,           /* G   CSM+LM xx.xxx deg (DP, 90) */
    AGC_SCALE_DP_DEG360,          /* H   CSM+LM xxx.xx deg (DP, 360) */
    AGC_SCALE_Y_OPTICS_DEG,       /* J   CSM    xx.xxx deg (optics Y; +19.775 deg bias on display, no negative load) */
    AGC_SCALE_HMS,                /* K   CSM+LM hhhhh. hrs / mmmmm. min / ss.ss sec across R1,R2,R3 (dec only, all 3 loaded) */
    AGC_SCALE_MIN_SEC,            /* L   CSM+LM xxBxx min/sec (display only, caps at 59B59) */
    AGC_SCALE_TIME_SEC_DP,        /* N   CSM+LM xxx.xx sec (DP) */
    AGC_SCALE_VEL2,               /* P   CSM+LM xxxxx. ft/sec */
    AGC_SCALE_POS4,               /* Q   CSM+LM xxxx.x nautical miles */
    AGC_SCALE_VEL3,               /* S   CSM+LM xxxx.x ft/sec */
    AGC_SCALE_POS5,               /* JJ  CSM+LM xxx.xx nautical miles */
    AGC_SCALE_WEIGHT2,            /* KK  CSM+LM xxxxx. lbs */
    AGC_SCALE_TWO_INT,            /* PP  CSM+LM +xxByy two integers (display only) */
    AGC_SCALE_POS9,               /* XX  CSM+LM xxxxx. feet */
    AGC_SCALE_VEL4,               /* YY  CSM+LM xxxx.x ft/sec (max 328.0) */
    AGC_SCALE_TRIM_DEG,           /* FF  CSM    xxx.xx deg (trim) */
    AGC_SCALE_INERTIA,            /* GG  CSM    xxxxxBB. slug ft^2 */
    AGC_SCALE_THRUST_MOMENT,      /* II  CSM    xxxxxBB. ft lbs */
    AGC_SCALE_VEL_2VS,            /* UU  CSM    xxxxx. ft/sec (max 51532) */
    AGC_SCALE_POS6,               /* LL  CSM    xxxx.x nautical miles */
    AGC_SCALE_DRAG_ACCEL,         /* MM  CSM    xxx.xx g (max 24.99) */
    AGC_SCALE_POS8,               /* VV  CSM    xxxx.x nautical miles */
    AGC_SCALE_DP_FRACTION,        /* ZZ  CSM    .xxxxx DP fractional */
    AGC_SCALE_LR_POSITION,        /* TT  LM     +0000x landing radar position 1/2 (display only; reads channel 33 bits 6,7) */
    AGC_SCALE_CDU360,             /* WW  LM     xxx.xx deg (360 - CDU) */
    AGC_SCALE_TRACKER_AZ_DEG,     /* EE  LM     xxx.xx deg (optical tracker azimuth) */
    AGC_SCALE_RR_RANGE,           /* U   LM     xxx.xx nautical miles (rendezvous radar range) */
    AGC_SCALE_RR_RATE,            /* V   LM     xxxxx. ft/sec (rendezvous radar range rate) */
    AGC_SCALE_LR_ALT,             /* W   LM     xxxxx. feet (landing radar altitude) */
    AGC_SCALE_INIT_FINAL_ALT,     /* AA  LM     xxxxx. feet (initial/final altitude) */
    AGC_SCALE_ALT_RATE,           /* BB  LM     xxxxx. ft/sec (altitude rate, max 8191) */
    AGC_SCALE_FWD_LAT_VEL,        /* CC  LM     xxxxx. ft/sec (forward/lateral velocity, max 9126) */
    AGC_SCALE_RHC_RATE,           /* DD  LM     xxxxx. deg/sec (rotational hand controller rates, max 44) */
    AGC_SCALE_LR_VELX,            /* X   LM     xxxxx. ft/sec (landing radar VELX) */
    AGC_SCALE_LR_VELY,            /* Y   LM     xxxxx. ft/sec (landing radar VELY) */
    AGC_SCALE_LR_VELZ,            /* Z   LM     xxxxx. ft/sec (landing radar VELZ) */
    AGC_SCALE_POS7,               /* QQ  LM     xxxx.x nautical miles (max 9058.9) */
    AGC_SCALE_TRIM_DEG2,          /* NN  LM     xxx.xx deg (trim, max 32.76) */
    AGC_SCALE_COMP_ALT,           /* RR  LM     xxxxx. feet (computed altitude) */
    AGC_SCALE_DP_DEGREES,         /* SS  LM     xxxx.x deg (DP degrees) */
    AGC_SCALE_RADIANS,            /* AAA LM     xx.xxx radians (max 31.999) */
    AGC_SCALE_COUNT
} agc_scale_t;

/*
 * Data ids: one per erasable the noun tables reference, named after the AGC
 * erasable ("X +2" becomes X_P2; "-TPER" NEG_TPER; "+MGA" PLUS_MGA). The
 * trailing comment gives SP/DP, the scale type(s) the nouns display it with
 * and the nouns using it per vehicle. Ids with a _DP suffix are the DP view
 * of a display buffer word that other nouns use as SP (DSPTEM1, DSPTEMX,
 * DSPTEM2 +1). DSPTEMX and OPTIONX are the same storage as DSPTEM2 +1 in the
 * AGC; they keep separate ids because the tables name them separately.
 */
typedef enum agc_noun_data {
    AGC_DATA_NONE = 0,
    AGC_DATA_SPECIFY_ADDR,    /* nouns 01-03: operator-specified address     */
    AGC_DATA_SPECIFY_CHANNEL, /* noun 10: operator-specified channel         */
    AGC_DATA_INCREMENT_ADDR,  /* noun 15: increment machine address          */
    AGC_DATA_CHANNEL33,       /* SP: I/O channel 33 image (LM noun 66 LR pos)*/
    AGC_DATA_ABVEL,                             /* DP VEL3; LM:62,63 */
    AGC_DATA_ACTCENT,                           /* DP DP_DEG360; CSM:52 LM:52 */
    AGC_DATA_ALMCADR,                           /* SP raw OCTAL; CSM:08 LM:08 */
    AGC_DATA_ALMCADR_P1,                        /* SP raw OCTAL; CSM:08 LM:08 */
    AGC_DATA_ALMCADR_P2,                        /* SP raw OCTAL; CSM:08 LM:08 */
    AGC_DATA_ALPHASB,                           /* DP DP_DEG360; LM:51 */
    AGC_DATA_ALT,                               /* DP POS4; CSM:43 LM:43 */
    AGC_DATA_ALTI,                              /* DP POS4; CSM:62 */
    AGC_DATA_AOTCODE,                           /* SP raw OCTAL; LM:70,71 */
    AGC_DATA_AOTCODE_P1,                        /* SP raw OCTAL; LM:70,71 */
    AGC_DATA_AOTCODE_P2,                        /* SP raw OCTAL; LM:70,71 */
    AGC_DATA_AZ,                                /* SP raw CDU_DEG; LM:87 */
    AGC_DATA_BETASB,                            /* DP DP_DEG360; LM:51 */
    AGC_DATA_CDUS,                              /* SP raw CDU_DEG; CSM:91 LM:72 */
    AGC_DATA_CDUT,                              /* SP raw Y_OPTICS_DEG,CDU360; CSM:91 LM:72 */
    AGC_DATA_CDUX,                              /* SP raw CDU_DEG; CSM:20 LM:20 */
    AGC_DATA_CDUY,                              /* SP raw CDU_DEG; CSM:20 LM:20 */
    AGC_DATA_CDUZ,                              /* SP raw CDU_DEG; CSM:20 LM:20 */
    AGC_DATA_CENTANG,                           /* DP DP_DEG360; CSM:55 LM:55 */
    AGC_DATA_CPHIX,                             /* SP raw CDU_DEG; CSM:17,96 */
    AGC_DATA_CPHIX_P1,                          /* SP raw CDU_DEG; CSM:17,96 */
    AGC_DATA_CPHIX_P2,                          /* SP raw CDU_DEG; CSM:17,96 */
    AGC_DATA_CSMMASS,                           /* SP WEIGHT2; CSM:47 LM:47 */
    AGC_DATA_CURSOR,                            /* SP raw CDU_DEG; LM:79 */
    AGC_DATA_D,                                 /* DP DRAG_ACCEL; CSM:64,74 */
    AGC_DATA_DAPDATR1,                          /* SP raw OCTAL; CSM:46 LM:46 */
    AGC_DATA_DAPDATR2,                          /* SP raw OCTAL; CSM:46 */
    AGC_DATA_DATAGOOD,                          /* SP WHOLE; LM:80 */
    AGC_DATA_DELHITE,                           /* DP POS4; CSM:72 */
    AGC_DATA_DELTAH,                            /* DP COMP_ALT; LM:68 */
    AGC_DATA_DELTAR,                            /* DP POS4; CSM:57 LM:57 */
    AGC_DATA_DELVIMU,                           /* DP VEL3; CSM:83 LM:83 */
    AGC_DATA_DELVIMU_P2,                        /* DP VEL3; CSM:83 LM:83 */
    AGC_DATA_DELVIMU_P4,                        /* DP VEL3; CSM:83 LM:83 */
    AGC_DATA_DELVLVC,                           /* DP VEL3,VEL2; CSM:81,82,86 LM:81,82,86 */
    AGC_DATA_DELVLVC_P2,                        /* DP VEL3,VEL2; CSM:81,82,86 LM:81,82,86 */
    AGC_DATA_DELVLVC_P4,                        /* DP VEL3,VEL2; CSM:81,82,86 LM:81,82,86 */
    AGC_DATA_DELVOV,                            /* DP VEL3; CSM:84 LM:84 */
    AGC_DATA_DELVOV_P2,                         /* DP VEL3; CSM:84 LM:84 */
    AGC_DATA_DELVOV_P4,                         /* DP VEL3; CSM:84 LM:84 */
    AGC_DATA_DELVTPF,                           /* DP VEL3; CSM:58 LM:58 */
    AGC_DATA_DELVTPI,                           /* DP VEL3; CSM:58 LM:58 */
    AGC_DATA_DIFFALT,                           /* DP POS4; CSM:75 LM:75 */
    AGC_DATA_DNRNGERR,                          /* DP POS6; CSM:66 */
    AGC_DATA_DSPTEM1,                           /* SP raw WHOLE,OCTAL,CDU_DEG; CSM:25,26,29,30,41,97 LM:25,26,41,97 */
    AGC_DATA_DSPTEM1_DP,                        /* DP DP_DEG360,HMS; CSM:05,31,34 LM:04,05,34 */
    AGC_DATA_DSPTEM1_P1,                        /* SP raw WHOLE,OCTAL,ELEV_DEG; CSM:25,26,30,41,97 LM:25,26,41,97 */
    AGC_DATA_DSPTEM1_P2,                        /* SP raw WHOLE,OCTAL; CSM:25,26,30,97 LM:25,26,97 */
    AGC_DATA_DSPTEM2,                           /* SP WHOLE; CSM:98 LM:98 */
    AGC_DATA_DSPTEM2_P1,                        /* SP FRACTION; CSM:98 LM:98 */
    AGC_DATA_DSPTEM2_P1_DP,                     /* DP HMS; CSM:24 LM:24 */
    AGC_DATA_DSPTEM2_P2,                        /* SP WHOLE; CSM:98 LM:98 */
    AGC_DATA_DSPTEMX,                           /* SP WHOLE; LM:14 */
    AGC_DATA_DSPTEMX_DP,                        /* DP HMS; CSM:16 LM:16 */
    AGC_DATA_DSPTEMX_P1,                        /* SP WHOLE; LM:14 */
    AGC_DATA_DSPTEMX_P2,                        /* SP WHOLE; LM:14 */
    AGC_DATA_DVLOS,                             /* DP VEL3; CSM:59 LM:59 */
    AGC_DATA_DVLOS_P2,                          /* DP VEL3; CSM:59 LM:59 */
    AGC_DATA_DVLOS_P4,                          /* DP VEL3; CSM:59 LM:59 */
    AGC_DATA_DVTOTAL,                           /* DP VEL3,VEL2; CSM:40,80 LM:40,62 */
    AGC_DATA_EL,                                /* SP raw CDU_DEG; LM:87 */
    AGC_DATA_ELEV,                              /* DP DP_DEG360; CSM:55 LM:55 */
    AGC_DATA_FAILREG,                           /* SP raw OCTAL; CSM:09 LM:09 */
    AGC_DATA_FAILREG_P1,                        /* SP raw OCTAL; CSM:09 LM:09 */
    AGC_DATA_FAILREG_P2,                        /* SP raw OCTAL; CSM:09 LM:09 */
    AGC_DATA_FDAIX,                             /* SP raw CDU_DEG; LM:18 */
    AGC_DATA_FDAIY,                             /* SP raw CDU_DEG; LM:18 */
    AGC_DATA_FDAIZ,                             /* SP raw CDU_DEG; LM:18 */
    AGC_DATA_FUNNYDSP,                          /* DP TWO_INT; LM:64 */
    AGC_DATA_GAMMAEI,                           /* DP DP_DEG360; CSM:60 */
    AGC_DATA_GAMMASB,                           /* DP DP_DEG360; CSM:51 */
    AGC_DATA_GMAX,                              /* SP WHOLE; CSM:60 */
    AGC_DATA_HAPO,                              /* DP POS4; CSM:42 LM:42 */
    AGC_DATA_HAPOX,                             /* DP POS4; CSM:44 LM:44 */
    AGC_DATA_HCALC,                             /* DP COMP_ALT; LM:60,64 */
    AGC_DATA_HCALC1,                            /* DP COMP_ALT; LM:63 */
    AGC_DATA_HDOT,                              /* DP VEL2; CSM:62 */
    AGC_DATA_HDOTDISP,                          /* DP VEL3; LM:60,63,64 */
    AGC_DATA_HEADSUP,                           /* SP WHOLE; CSM:61 */
    AGC_DATA_HORIZON,                           /* SP raw OCTAL; CSM:70,71 */
    AGC_DATA_HPER,                              /* DP POS4; CSM:42 LM:42 */
    AGC_DATA_HPERX,                             /* DP POS4; CSM:44,50 LM:44 */
    AGC_DATA_LANDALT,                           /* DP POS5; CSM:89 LM:89 */
    AGC_DATA_LANDLAT,                           /* DP DP_DEG90; CSM:89 LM:89 */
    AGC_DATA_LANDLONG,                          /* DP DP_DEG90; CSM:89 LM:89 */
    AGC_DATA_LANDMARK,                          /* SP raw OCTAL; CSM:70,71 */
    AGC_DATA_LAT,                               /* DP DP_DEG360; CSM:43,67 LM:43 */
    AGC_DATA_LAT_SPL,                           /* DP DP_DEG360; CSM:61 */
    AGC_DATA_LEMMASS,                           /* SP WEIGHT2; CSM:47 LM:47 */
    AGC_DATA_LNG_SPL,                           /* DP DP_DEG360; CSM:61 */
    AGC_DATA_LONG,                              /* DP DP_DEG360; CSM:43,67 LM:43 */
    AGC_DATA_MRKBUF1_P3,                        /* SP raw CDU_DEG; CSM:87,94 */
    AGC_DATA_MRKBUF1_P5,                        /* SP raw Y_OPTICS_DEG; CSM:87,94 */
    AGC_DATA_MARK_DELTA_R,                           /* DP POS4; CSM:49 */
    AGC_DATA_MARK_DELTA_V,                        /* DP VEL3; CSM:49 */
    AGC_DATA_MARK_SOURCE_CODE,                        /* SP WHOLE; CSM:49 */
    AGC_DATA_NEG_TPER,                          /* DP HMS; CSM:32 LM:32 */
    AGC_DATA_NN,                                /* SP WHOLE; LM:55 */
    AGC_DATA_NN1,                               /* SP WHOLE; CSM:55 */
    AGC_DATA_OGC,                               /* DP DP_DEG90; CSM:93 LM:93 */
    AGC_DATA_OGC_P2,                            /* DP DP_DEG90; CSM:93 LM:93 */
    AGC_DATA_OGC_P4,                            /* DP DP_DEG90; CSM:93 LM:93 */
    AGC_DATA_OMEGAD,                            /* DP DP_DEG360; LM:80 */
    AGC_DATA_OPTION1,                           /* SP raw OCTAL; CSM:06 LM:06 */
    AGC_DATA_OPTION2,                           /* SP raw OCTAL,WHOLE; CSM:06,72 LM:06 */
    AGC_DATA_OPTION3,                           /* SP raw OCTAL; LM:06 */
    AGC_DATA_OPTIONX,                           /* SP raw OCTAL; CSM:12 LM:12 */
    AGC_DATA_OPTIONX_P1,                        /* SP raw OCTAL; CSM:12 LM:12 */
    AGC_DATA_OUTOFPLN,                          /* DP POS7; LM:61 */
    AGC_DATA_GROUND_TRACK_DETERMINATION_ALT,                            /* DP POS4; CSM:73 LM:91 */
    AGC_DATA_GROUND_TRACK_DETERMINATION_GAM,                            /* DP DP_DEG360; CSM:73 LM:91 */
    AGC_DATA_GROUND_TRACK_DETERMINATION_VEL,                            /* DP VEL2; CSM:73 LM:91 */
    AGC_DATA_PAC,                               /* SP raw Y_OPTICS_DEG; CSM:92 */
    AGC_DATA_PACTOFF,                           /* SP TRIM_DEG; CSM:48 */
    AGC_DATA_PIPAX,                             /* SP WHOLE; CSM:21 LM:21 */
    AGC_DATA_PIPAY,                             /* SP WHOLE; CSM:21 LM:21 */
    AGC_DATA_PIPAZ,                             /* SP WHOLE; CSM:21 LM:21 */
    AGC_DATA_PITCH,                             /* DP DP_DEG360; LM:74 */
    AGC_DATA_PITTIME,                           /* SP TRIM_DEG2; LM:48 */
    AGC_DATA_PLUS_MGA,                          /* DP DP_DEG360; CSM:45 LM:45 */
    AGC_DATA_POSCODE,                           /* SP WHOLE; LM:79 */
    AGC_DATA_POSTTPI,                           /* DP POS4; CSM:58 LM:58 */
    AGC_DATA_PRAXIS,                            /* SP raw CDU_DEG; CSM:95 */
    AGC_DATA_PRAXIS_P1,                         /* SP raw CDU_DEG; CSM:95 */
    AGC_DATA_PRAXIS_P2,                         /* SP raw CDU_DEG; CSM:95 */
    AGC_DATA_Q7,                                /* DP DRAG_ACCEL; CSM:69 */
    AGC_DATA_RADAR_DELTA_R,                           /* DP POS4; LM:49 */
    AGC_DATA_RADAR_DELTA_V,                        /* DP VEL3; LM:49 */
    AGC_DATA_RANGE,                             /* DP POS5; CSM:53,54,90 LM:54,90 */
    AGC_DATA_RANGEDSP,                          /* DP POS7; LM:68 */
    AGC_DATA_RDOT,                              /* DP VEL_2VS; CSM:68 */
    AGC_DATA_RDOTD,                             /* DP VEL3; LM:76 */
    AGC_DATA_RHOSB,                             /* DP DP_DEG360; CSM:51 */
    AGC_DATA_ROLLC,                             /* DP DP_DEG360; CSM:66,68,69,74 */
    AGC_DATA_ROLLTIME,                          /* SP TRIM_DEG2; LM:48 */
    AGC_DATA_RRATE,                             /* DP VEL3; CSM:53,54,90 LM:54,90 */
    AGC_DATA_RR_AZ,                             /* DP DP_DEG360; LM:56 */
    AGC_DATA_RR_ELEV,                           /* DP DP_DEG360; LM:56 */
    AGC_DATA_RSP_RREC,                          /* DP POS6; CSM:50 */
    AGC_DATA_RSTACK,                            /* DP LR_VELX,RR_RANGE; LM:67,78 */
    AGC_DATA_RSTACK_P2,                         /* DP LR_VELY,RR_RATE; LM:67,78 */
    AGC_DATA_RSTACK_P4,                         /* DP LR_VELZ; LM:67 */
    AGC_DATA_RSTACK_P6,                         /* DP LR_ALT; LM:66 */
    AGC_DATA_RTEDVD,                            /* DP VEL2; CSM:56 */
    AGC_DATA_RTEGAM2D,                          /* DP DP_DEG360; CSM:56 */
    AGC_DATA_RTGO,                              /* DP POS6; CSM:63 */
    AGC_DATA_RANGE_TO_SPLASH,                           /* DP POS6; CSM:64 */
    AGC_DATA_RANGE_TO_TARGET,                           /* DP POS6; CSM:67 */
    AGC_DATA_RTHETA,                            /* DP DP_DEG360; CSM:53,54,90 LM:54,90 */
    AGC_DATA_SAC,                               /* SP raw CDU_DEG; CSM:92 */
    AGC_DATA_SAMPTIME,                          /* DP HMS; CSM:65 LM:65 */
    AGC_DATA_SMODE,                             /* SP WHOLE; CSM:27 LM:27 */
    AGC_DATA_SPIRAL,                            /* SP raw CDU_DEG; LM:79 */
    AGC_DATA_STARAD,                            /* SP FRACTION; LM:88 */
    AGC_DATA_STARAD_P2,                         /* SP FRACTION; LM:88 */
    AGC_DATA_STARAD_P4,                         /* SP FRACTION; LM:88 */
    AGC_DATA_STARCODE,                          /* SP raw OCTAL; CSM:70,71 */
    AGC_DATA_STARSAV3,                          /* DP DP_FRACTION; CSM:88 */
    AGC_DATA_STARSAV3_P2,                       /* DP DP_FRACTION; CSM:88 */
    AGC_DATA_STARSAV3_P4,                       /* DP DP_FRACTION; CSM:88 */
    AGC_DATA_T1TOT2,                            /* DP MIN_SEC; CSM:75 LM:75 */
    AGC_DATA_T2TOT3,                            /* DP MIN_SEC; CSM:75 LM:75 */
    AGC_DATA_T3TOT4,                            /* DP HMS; CSM:39 */
    AGC_DATA_TANG,                              /* SP raw CDU360; LM:73 */
    AGC_DATA_TANG_P1,                           /* SP raw CDU_DEG; LM:73 */
    AGC_DATA_TCDH,                              /* DP HMS; CSM:13 LM:13 */
    AGC_DATA_TCSI,                              /* DP HMS; CSM:11 LM:11 */
    AGC_DATA_TET,                               /* DP HMS; CSM:38 LM:38 */
    AGC_DATA_TFF,                               /* DP MIN_SEC; CSM:44,50 LM:44 */
    AGC_DATA_THETAD,                            /* SP raw CDU_DEG; CSM:18,19,22 LM:22 */
    AGC_DATA_THETAD_P1,                         /* SP raw CDU_DEG; CSM:18,19,22 LM:22 */
    AGC_DATA_THETAD_P2,                         /* SP raw CDU_DEG; CSM:18,19,22 LM:22 */
    AGC_DATA_THETZERO,                          /* DP DP_DEG360; CSM:72 */
    AGC_DATA_TIG,                               /* DP HMS; CSM:33 LM:33 */
    AGC_DATA_TIME2,                             /* DP HMS; CSM:36 LM:36 */
    AGC_DATA_TRKMKCNT,                          /* SP WHOLE; LM:45 */
    AGC_DATA_TTE,                               /* DP MIN_SEC; CSM:63 */
    AGC_DATA_TTFDISP,                           /* DP MIN_SEC; LM:61,68 */
    AGC_DATA_TTOGO,                             /* DP HMS,MIN_SEC; CSM:35,40,45,80 LM:35,40,45,61,62,74,77 */
    AGC_DATA_TTPI,                              /* DP HMS; CSM:37 LM:37 */
    AGC_DATA_VGBODY,                            /* DP VEL3; CSM:85 LM:85 */
    AGC_DATA_VGBODY_P2,                         /* DP VEL3; CSM:85 LM:85 */
    AGC_DATA_VGBODY_P4,                         /* DP VEL3; CSM:85 LM:85 */
    AGC_DATA_VGDISP,                            /* DP VEL3,VEL2; CSM:40,42,80 LM:40,42 */
    AGC_DATA_VHFCNT,                            /* DP TWO_INT; CSM:45 */
    AGC_DATA_VHORIZ,                            /* DP VEL3; LM:60 */
    AGC_DATA_VIO,                               /* DP VEL2; CSM:63 */
    AGC_DATA_VL,                                /* DP VEL_2VS; CSM:69 */
    AGC_DATA_VMAGI,                             /* DP VEL2; CSM:62,64,68,74 */
    AGC_DATA_VPRED,                             /* DP VEL2; CSM:60 */
    AGC_DATA_WHCHREAD,                          /* SP WHOLE; LM:49 */
    AGC_DATA_WWBIAS,                            /* DP RADIANS; LM:99 */
    AGC_DATA_WWOPT,                             /* SP WHOLE; CSM:99 */
    AGC_DATA_WWPOS,                             /* DP POS9; CSM:99 LM:99 */
    AGC_DATA_WWVEL,                             /* DP VEL4; CSM:99 LM:99 */
    AGC_DATA_XRANGE,                            /* DP POS4; LM:76 */
    AGC_DATA_XREG,                              /* SP raw OCTAL; CSM:07 LM:07 */
    AGC_DATA_XRNGERR,                           /* DP POS8; CSM:66 */
    AGC_DATA_YACTOFF,                           /* SP TRIM_DEG; CSM:48 */
    AGC_DATA_YAW,                               /* DP DP_DEG360; LM:74 */
    AGC_DATA_YDOT,                              /* DP VEL3; LM:77 */
    AGC_DATA_YREG,                              /* SP raw OCTAL; CSM:07 LM:07 */
    AGC_DATA_ZDOTD,                             /* DP VEL3; LM:76 */
    AGC_DATA_ZREG,                              /* SP raw OCTAL; CSM:07 LM:07 */
    AGC_DATA_COUNT
} agc_noun_data_t;

/* Ids below this are special (not fixed erasables).                       */
#define AGC_DATA_FIRST_ERASABLE AGC_DATA_ABVEL

/* One noun (interface agreed with io-pinball). */
typedef struct agc_noun {
    uint8_t  noun;            /* noun number 00..99                         */
    uint8_t  ncomp;           /* 1..3 components; 0 = spare noun            */
    uint8_t  scale_type[3];   /* agc_scale_t per component                  */
    uint16_t data_id[3];      /* agc_noun_data_t per component              */
    bool     dec_only;        /* component code bit 4: decimal only         */
    bool     no_load;         /* component code bit 5: no load (implies dec)*/
} agc_noun_t;

/* Static tables, indexed by noun number (entry n has .noun == n).         */
extern const agc_noun_t agc_noun_table_csm[AGC_NOUN_COUNT];
extern const agc_noun_t agc_noun_table_lm[AGC_NOUN_COUNT];

/*
 * LODNNTAB: fetch the table row for a noun of the vehicle selected by
 * agc_state_vehicle(). BAD_ARG: noun > 99 or out NULL. NOT_FOUND: spare
 * noun (out is still filled, with ncomp == 0).
 */
agc_status_t agc_noun_lookup(uint8_t noun, agc_noun_t *out);
agc_status_t agc_noun_lookup_vehicle(agc_vehicle_t veh, uint8_t noun,
                                     agc_noun_t *out);

/*
 * Resolve a data id to the erasable behind it (see conventions above).
 * BAD_ARG: id out of range or value NULL (write: value out of range).
 * NOT_FOUND: special id the caller resolves. UNIMPLEMENTED: erasable not yet
 * exposed by core (list in docs/modules/io-nouns.md).
 */
agc_status_t agc_noun_read(uint16_t data_id, agc_dp_t *value);
agc_status_t agc_noun_write(uint16_t data_id, agc_dp_t value);

/* ---- scale types: SFRUTAB codes and SFINTAB / SFOUTAB constants -------- */
typedef struct agc_scale_info {
    agc_scale_t scale;
    uint8_t     routine;      /* agc_sf_routine_t code for this vehicle     */
    uint8_t     sf_code;      /* SF constant code number (SFINTAB index)    */
    bool        is_dp;        /* erasable is a DP pair (else one SP word)   */
    bool        is_raw;       /* SP word passed raw, 15 bits (see top)      */
    agc_dp_t    sf_in;        /* SFINTAB entry for loads (2DEC, scale 2^28) */
    agc_dp_t    sf_out;       /* SFOUTAB entry for displays                 */
} agc_scale_info_t;

/* NOT_FOUND if the scale type does not exist in that vehicle's program.   */
agc_status_t agc_scale_info_vehicle(agc_vehicle_t veh, agc_scale_t scale,
                                    agc_scale_info_t *out);
agc_status_t agc_scale_info(agc_scale_t scale, agc_scale_info_t *out);

/* GTSFIN / GTSFOUT: the raw constant pair for an SF constant code.        */
agc_status_t agc_scale_sfconst(agc_vehicle_t veh, uint8_t sf_code,
                               agc_dp_t *sf_in, agc_dp_t *sf_out);

#endif /* AGC_NOUNS_H */
