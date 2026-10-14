/**********************************************************************
 * FILE NAME: entry_lexicon.h
 *
 * PURPOSE:
 *   Entry constants and gains, each the raw agc_dp_t / agc_sp_t word of
 *   the rope with the AGC text and the engineering meaning in its comment.
 *
 * AGC SOURCE:
 *   Comanche055 ENTRY_LEXICON.agc pp. 837-843 (meanings);
 *   REENTRY_CONTROL.agc pp. 879-881, P61-P67.agc pp. 805-811,
 *   CM_BODY_ATTITUDE.agc p. 889, CSM_GEOMETRY.agc pp. 291-294,
 *   CM_ENTRY_DIGITAL_AUTOPILOT.agc pp. 1079-1092 (values)
 *
 * REQUIRES:
 *   none (self-contained; includes what it uses)
 *
 * GLOBAL VARIABLES:
 *   none (constants only)
 *
 * Each DP constant is the raw agc_dp_t (value x 2^28 after the AGC's own
 * B-scaling). Units: /2VS = /51532.3946 fps, /805 = /25 g, revs = /360
 * deg = /21600 NM. Values were checked word for word against the rope
 * (review-guid-entry).
 **********************************************************************/
#ifndef AGC_ENTRY_LEXICON_H
#define AGC_ENTRY_LEXICON_H

#include "agc/core/word.h"
#include "agc/csm/entry.h"

/* ---- DP constants ------------------------------------------------------ */
#define AGC_ENTRY_LEWD1 \
    ((agc_dp_t) 40265318)                          /* 2DEC .15: nominal upcontrol L/D, scale 1 */
#define AGC_ENTRY_POINT1 \
    ((agc_dp_t) 26843546)                          /* 2DEC .1: factor to reduce upcontrol gain */
#define AGC_ENTRY_DLEWD0 \
    ((agc_dp_t) -13421773)                         /* 2DEC -.05: initial variation in LEWD */
#define AGC_ENTRY_GMAX_HALF \
    ((agc_dp_t) 42949673)                          /* 2DEC .16: GMAX/2: 8 g /2, scale 805 fpss (25 g) */
#define AGC_ENTRY_C18 ((agc_dp_t) 2604531)         /* 2DEC .0097026346: 500 fps / 2VS */
#define AGC_ENTRY_Q7FKDMIN \
    ((agc_dp_t) 2167491)                           /* 2DEC .0080745342: (Q7F+KDMIN)/805 = 6.5/805 */
#define AGC_ENTRY_Q3  ((agc_dp_t) 44829562)        /* 2DEC .167003132: .07 2VS/21600 */
#define AGC_ENTRY_Q5  ((agc_dp_t) 87614350)        /* 2DEC .326388889 .3x23500/21600 */
#define AGC_ENTRY_Q6  ((agc_dp_t) 9368397)         /* 2DEC .0349: 2 deg approx, rad */
#define AGC_ENTRY_Q7F ((agc_dp_t) 2000761)         /* 2DEC .0074534161: 6/805 fpss */
#define AGC_ENTRY_Q21 ((agc_dp_t) 6213784)         /* 2DEC .0231481481: 500/21600 NM */
#define AGC_ENTRY_Q22 \
    ((agc_dp_t) -14316558)                         /* 2DEC -.053333333: -1152/21600 NM */
#define AGC_ENTRY_VLMIN \
    ((agc_dp_t) 93763122)                          /* 2DEC .34929485: 18000 fps / 2VS */
#define AGC_ENTRY_C12 \
    ((agc_dp_t) 1837636)                           /* 2DEC .00684572901: 32 28500/(21202900 2 PI) */
#define AGC_ENTRY_ONE_OVER_KB1 \
    ((agc_dp_t) 78951605)                          /* 2DEC .29411765: 1/KB1 = 1/3.4 */
#define AGC_ENTRY_MINUS_ONE_OVER_KB2 \
    ((agc_dp_t) -24513235)                         /* 2DEC -.0057074322 B4: -1/(.0034 2VS) x 2^4 */
#define AGC_ENTRY_VQUIT \
    ((agc_dp_t) 5209062)                           /* 2DEC .019405269: 1000 fps / 2VS */
#define AGC_ENTRY_C20 \
    ((agc_dp_t) 58355533)                          /* 2DEC .21739130: 175/805 fpss: lift up if above */
#define AGC_ENTRY_C21 ((agc_dp_t) 46684426)        /* 2DEC .17391304: 140/805 fpss */
#define AGC_ENTRY_TOL_25NM \
    ((agc_dp_t) 310689)                            /* 2DEC .0011574074: 25/21600 NM */
#define AGC_ENTRY_K1D \
    ((agc_dp_t) 8441037)                           /* 2DEC .0314453125: C16 805/256 = .01 805/256 */
#define AGC_ENTRY_K2D \
    ((agc_dp_t) -54035633)                         /* 2DEC -.201298418: -C17 2VS/256 = -.001 2VS/256 */
#define AGC_ENTRY_KVSCALE \
    ((agc_dp_t) 218753271)                         /* 2DEC .81491944: 12800/(2VS .3048) */
#define AGC_ENTRY_KASCALE \
    ((agc_dp_t) 262146974)                         /* 2DEC .97657358: 5.85 16384/(4 .3048 100 805) */
#define AGC_ENTRY_KTETA \
    ((agc_dp_t) 628319)                            /* 2DEC* .383495203 E2 B-14*: 1000 2PI/16384 (163.84) */
#define AGC_ENTRY_KT1 \
    ((agc_dp_t) 258520)                            /* 2DEC* .157788327 E2 B-14*: RE 2PI/2VS/16384 163.84 */
#define AGC_ENTRY_G05 ((agc_dp_t) 536871)          /* 2DEC .002: .05 g/25 g */
#define AGC_ENTRY_LATBIAS \
    ((agc_dp_t) 8053)                              /* 2DEC .00003: approx .5 NM / 4(21600/2PI) */
#define AGC_ENTRY_KWE \
    ((agc_dp_t) 16113731)                          /* 2DEC .120056652 B-1: earth rate velocity factor */
#define AGC_ENTRY_KACOS ((agc_dp_t) 1335088)       /* 2DEC .004973592: 1/32(2PI) */
#define AGC_ENTRY_CHOOK ((agc_dp_t) 4194304)       /* 2DEC 1 B-6: .25/16 */
#define AGC_ENTRY_ONE_24TH \
    ((agc_dp_t) 11184811)                          /* 2DEC .0833333333 B-1: 1/24 x 2^-1 */
#define AGC_ENTRY_CH1 \
    ((agc_dp_t) 171798692)                         /* 2DEC .32 B1: 16 CH1/25 = 16 (1)/25 */
#define AGC_ENTRY_KC3 \
    ((agc_dp_t) -6647059)                          /* 2DEC -.0247622232: -(4 VS VS/2 PI 805 RE) */
#define AGC_ENTRY_VRCONT \
    ((agc_dp_t) 3646344)                           /* 2DEC .0135836886: 700 fps / 2VSAT */
#define AGC_ENTRY_TWO_HS \
    ((agc_dp_t) 4638205)                           /* 2DEC .0172786611: 2HS: 2 28500 25 32.2/(4 VS VS) */
#define AGC_ENTRY_TWO_HS_GMAX_SQ \
    ((agc_dp_t) 8207)                              /* 2DEC .0000305717 2HSGMXSQ: (2 28500 8 32.2/4VS^2)^2 */
#define AGC_ENTRY_C001 \
    ((agc_dp_t) -167772)                           /* 2DEC -.000625: -(4/25)/256  LEQ/D0 const */
#define AGC_ENTRY_TWO_C1_HS \
    ((agc_dp_t) 5797757)                           /* 2DEC .0215983264: 2C1HS: 2 1.25 28500 805/(2VS)^2 */
#define AGC_ENTRY_PT1_16 ((agc_dp_t) 1677722)      /* 2DEC .1 B-4: PT1/16 */
#define AGC_ENTRY_ONE_OVER_K44 \
    ((agc_dp_t) 700427)                            /* 2DEC .00260929464: 1/K44 = 2VS/19749550 */
#define AGC_ENTRY_VFINAL \
    ((agc_dp_t) 138561057)                         /* 2DEC .51618016: 26600 fps / 2VS */
#define AGC_ENTRY_VFINAL1 \
    ((agc_dp_t) 140644683)                         /* 2DEC .523942273: 27000 fps / 2VS */
#define AGC_ENTRY_ONE_OVER_KA1 \
    ((agc_dp_t) 80659693)                          /* 2DEC .30048077: 1/KA1 = 25/(1.3 64) */
#define AGC_ENTRY_KA2 ((agc_dp_t) 2147484)         /* 2DEC .008: .2/25 */
#define AGC_ENTRY_KA3 ((agc_dp_t) 120045670)       /* 2DEC .44720497: 90 4/805 */
#define AGC_ENTRY_KA4 \
    ((agc_dp_t) 13338408)                          /* 2DEC .049689441: 40/805 (also Q7MIN) */
#define AGC_ENTRY_KALIM ((agc_dp_t) 16106127)      /* 2DEC .06: 1.5/25 */
#define AGC_ENTRY_MINUS_HSCALED \
    ((agc_dp_t) -148458277)                        /* 2DEC -.55305018: -HSCALED = -28500/2VS */
#define AGC_ENTRY_MINUS_KSCALE \
    ((agc_dp_t) -8386590)                          /* 2DEC -.0312424837: -KSCALE = -805/VS */
#define AGC_ENTRY_COS15     ((agc_dp_t) 259040215) /* 2DEC .965: cos(15 deg) */
#define AGC_ENTRY_ONE_THIRD ((agc_dp_t) 89478485)  /* 2DEC .3333333333: 1/3RD */
#define AGC_ENTRY_ONE_16TH \
    ((agc_dp_t) 16777216)                          /* DP2(-4): 1/16TH, also C1/16 */
#define AGC_ENTRY_SEC500 \
    ((agc_dp_t) 50000)                             /* 2DEC 50000 B-28: 500SEC nominal entry time, cs */
#define AGC_ENTRY_NEARONE \
    ((agc_dp_t) 268435455)                         /* 2DEC .999999999: NEARONE / BARELY1 */
#define AGC_ENTRY_MKVSCALE \
    ((agc_dp_t) -218753271)                        /* 2DEC -.81491944: -KVSCALE (CM_BODY_ATTITUDE) */
#define AGC_ENTRY_ONE_OVER_WIE \
    ((agc_dp_t) 8616410)                           /* 2DEC 8616410: 1/WIE, cs per radian (CSM_GEOMETRY) */
#define AGC_ENTRY_C30LIM \
    ((agc_dp_t) 152198877)                         /* 2DEC .566985: C(30)LIM = 1.0 - .5 cos(30) */
#define AGC_ENTRY_SEC60_DP ((agc_dp_t) 6000)       /* 2DEC 6000 B-28: 60SECDP, cs */
#define AGC_ENTRY_KFT290   ((agc_dp_t) 44196)      /* 2DEC 88392.0 B-29: 290KFT, m */
#define AGC_ENTRY_KTETA1 \
    ((agc_dp_t) 691150)                            /* 2DEC* .421844723 E2 B-14*: 110 2PI/16384(163.84) */
#define AGC_ENTRY_KFT36_PER_S \
    ((agc_dp_t) 230116295)                         /* 2DEC 109.728 B-7: 36 kft/s in m/cs */
#define AGC_ENTRY_KFT20_PER_S \
    ((agc_dp_t) 255684772)                         /* 2DEC 121.92 B-7: 2 x 20 kft/s in m/cs */
#define AGC_ENTRY_KR1 ((agc_dp_t) -7158279)        /* 2DEC -.026666667: -2.4 4/360 */
#define AGC_ENTRY_M6_05DEG \
    ((agc_dp_t) -4511207)                          /* 2DEC -.016805556: -6.05/360 */
#define AGC_ENTRY_KR2 \
    ((agc_dp_t) 147456001)                         /* 2DEC .54931641: (360/4) 100 B-14 */
#define AGC_ENTRY_KR3 \
    ((agc_dp_t) 16384000)                          /* 2DEC 1000 B-14: 100 (10.0) B-14 g */
#define AGC_ENTRY_RTRIAL \
    ((agc_dp_t) 3230049)                           /* 2DEC 6460097.18 B-29: RPAD + 264643 ft, m */
#define AGC_ENTRY_KFT400 ((agc_dp_t) 60960)        /* 2DEC 121920 B-29: 400 kft in m */
#define AGC_ENTRY_KR4 \
    ((agc_dp_t) 223696213)                         /* 2DEC .833333333: GMAX coefficient */
#define AGC_ENTRY_KFT300 \
    ((agc_dp_t) 45720)                             /* 2DEC 91440 B-29: 300KFT = MINPERE, m */
#define AGC_ENTRY_VEMSCON \
    ((agc_dp_t) -638)                              /* 2DEC -.0389676 B-14: -HS D/2PI, m^2/cs^2 */
#define AGC_ENTRY_RTMU_INV_EARTH \
    ((agc_dp_t) 176124180)                         /* 2DEC* .5005750271 E-5 B17*: 1/RTMU (earth) */
#define AGC_ENTRY_RTMU_INV_LUNENT \
    ((agc_dp_t) 176229826)                         /* 2DEC* .50087529 E-5 B17*: 1/RTMUE */
#define AGC_ENTRY_HALVE \
    ((agc_dp_t)         \
     AGC_DP_HALF)                                  /* HALVE = HIDPHALF .5; also Q19, 1/GMAX, VSAT/2VS */
#define AGC_ENTRY_FOURTH \
    ((agc_dp_t)          \
     AGC_DP_QUARTER)                               /* FOURTH = HIDP1/4 .25; also VMIN = (VS/2)/2VS */
#define AGC_ENTRY_NEAR1_4 \
    ((agc_dp_t) (07777 *  \
                 16384))                           /* 2OCT 0777700000: 1/4 less 1 bit in upper part */
#define AGC_ENTRY_MAXRNG          \
    ((agc_dp_t) (016631 * 16384 + \
                 06755))                           /* 2OCT 1663106755: DNRNGERR = 9999.9 NM */
#define AGC_ENTRY_ONE_12TH      \
    ((agc_dp_t) (1365 * 16384 + \
                 5461))                            /* 1/12TH: DEC .083333 + hi word of 1/3RD (LATSLOPE) */
#define AGC_ENTRY_ONEBIT_DP \
    ((agc_dp_t) 1)                                 /* 1BITDP  2^-28                          */
#define AGC_ENTRY_ONEBIT_DP_P1 \
    ((agc_dp_t) 16384)                             /* 1BITDP +1 as DP = (1, 0) = 2^-14        */

/* ---- loop bounds (Power of Ten rule 2) --------------------------------- */
#define AGC_ENTRY_HUNTEST_MAX_ITER \
    25  /* DEVIATION: AGC iterates HUNTEST without a bound */
#define AGC_ENTRY_SEQ_MAX_STEPS \
    200 /* max sequencer steps in one 2 s pass (>= 6 per HUNTEST iteration) */
#define AGC_ENTRY_VREFER_N \
    13  /* points per VREFER function                       */
#define AGC_ENTRY_VREFER_TABLES \
    7   /* VREF plus six functions of V                     */

/* ---- SP constants (DAP, body attitude) --------------------------------- */
#define AGC_ENTRY_SPVQUIT \
    ((agc_sp_t) 318)         /* DEC .019405  1000 fps/2VS (SP)        */
#define AGC_ENTRY_TCDU \
    ((agc_sp_t) 1638)        /* DEC .1  TCDU = .1 s                   */
#define AGC_ENTRY_NSEC \
    ((uint16_t) 2100)        /* DEC 2100 cs: 65 deg / 3 deg/s         */
#define AGC_ENTRY_4D_SLIM \
    ((agc_sp_t) 16348)       /* DEC 16348 = 1.0 - 4/1800 B14          */
#define AGC_ENTRY_YDOTLIM \
    ((agc_sp_t) 16366)       /* DEC 16366 = 1.0 - 2 deg/s dead zone   */
#define AGC_ENTRY_CM_BIAS \
    ((agc_sp_t) 55)          /* DEC 55 = .6/180 B14                   */
#define AGC_ENTRY_YAWLIM \
    ((agc_sp_t) 16055)       /* DEC 16055 = 1.0 - 3.6/180             */
#define AGC_ENTRY_C45LIM \
    ((agc_sp_t) 4799)        /* DEC .29289 = 1.0 - cos 45             */
#define AGC_ENTRY_SINTRIM \
    ((agc_sp_t) -5604)       /* DEC -.34202 sin(-20 deg)              */
#define AGC_ENTRY_COSTRIM \
    ((agc_sp_t) 15396)       /* DEC .93969  cos(-20 deg)              */
#define AGC_ENTRY_T_M3 \
    ((agc_sp_t) -203)        /* -T-3 DEC -203 cs                      */
#define AGC_ENTRY_VSQMIN \
    ((agc_sp_t) 10)          /* DEC .61050061 E-3 = 4/(4 (9.1) 180)   */
#define AGC_ENTRY_2T_TCDU \
    ((agc_sp_t) 050)         /* 2T/TCDU = OCT50 (40): 2 T/TCDU B-14, T = 2 s */
#define AGC_ENTRY_180_8ATT \
    ((agc_sp_t) 10127)       /* DEC .61813187 = 180/(8 (9.1) 4) B-3 (also 1/16A1) */
#define AGC_ENTRY_2JETT \
    ((agc_sp_t) 400)         /* 2JETT = 4SECS: 2 (2) 100 cs           */
#define AGC_ENTRY_4JETT \
    ((agc_sp_t) 800)         /* DEC 800 cs                            */
#define AGC_ENTRY_XMIN_360 \
    ((agc_sp_t) 182)         /* DEC 182 = 4/360 B14                   */
#define AGC_ENTRY_MVM_360K \
    ((agc_sp_t) -3641)       /* DEC -.22222222 = -20/(360 .25) (also -VMT/180) */
#define AGC_ENTRY_XS_360 \
    ((agc_sp_t) 91)          /* DEC 91 = 2/360 B14 (also BUFLIM)      */
#define AGC_ENTRY_KTRCS \
    ((agc_sp_t) AGC_SP_HALF) /* KTRCS = HALF: KT = (.25) 2       */

#endif /* AGC_ENTRY_LEXICON_H */
