/**********************************************************************
 * FILE NAME: scales.c
 *
 * PURPOSE:
 *   SF routine / SF constant tables for the PINBALL nouns (SFINTAB,
 *   SFOUTAB) and the scale-type lookup agc_scale_info().
 *
 * AGC SOURCE:
 *   Comanche055 PINBALL_NOUN_TABLES.agc pp. 271, 277-278 (GTSFIN, GTSFOUT,
 *     SFINTAB, SFOUTAB)
 *   Luminary099 PINBALL_NOUN_TABLES.agc pp. 304-305, 311-312 (differs from
 *     code 12 up: each vehicle has its own pair of tables)
 *
 * EXTERNAL VARIABLES:
 *   none (constant tables)
 *
 * EXTERNAL REFERENCES:
 *   Name               Description
 *   -----------------  -----------------------------------------------
 *   agc_state_vehicle  vehicle for agc_scale_info()
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   AGC_STATUS_BAD_ARG for an unknown scale or SF code.
 *
 * NOTES:
 *   Every entry is the AGC's two 15-bit words (high, low), written as the
 *   octal words of the listing and checked against the assembled core rope
 *   (the 2DEC entries are the rope words of the assembler's conversion of
 *   the 2DEC literal quoted in the comment). The trailing "DP" number in
 *   each comment is the pair read as a signed DP fraction (value / 2^28).
 *   Entries whose constants live in the SF routine itself (CDU degrees:
 *   DEGINSF, DEGOUTSF, OPTDEGIN/OUT, 360 CDUO) are zero here. Scale types
 *   (agc_scale_t) map to their (SF routine, SF constant) pair per vehicle
 *   in scale_defs[]; each row's comment gives the AGC scale-type letter,
 *   the display format and the erasable's scale from
 *   ASSEMBLY_AND_OPERATION_INFORMATION "NOUN SCALES AND FORMATS".
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/io/nouns.h"

#define SF_COUNT_CSM 20
#define SF_COUNT_LM  29

typedef struct sf_pair
{
    uint16_t hi; /* high-order AGC word (15 bits, 1s compl.) */
    uint16_t lo; /* low-order AGC word                       */
} sf_pair_t;

typedef struct scale_def
{
    int8_t csm_routine; /* SF routine code, -1: not in Comanche     */
    int8_t csm_code;    /* SF constant code                         */
    int8_t lm_routine;  /* -1: not in Luminary                      */
    int8_t lm_code; /* LM SF constant code, -1 none */
    bool   is_dp;  /* DP pair (else one SP word)               */
    bool   is_raw; /* full 15-bit SP word, passed raw          */
} scale_def_t;

/* SFINTAB, Comanche055 (CSM): one DP pair (high, low) per SF constant code. */
static const sf_pair_t sfintab_csm[20] = {
    /*  0  WHOLE, DP TIME (SEC)
     *     scale types: OCTAL, FRACTION, WHOLE, HMS, MIN_SEC, TIME_SEC_DP, TWO_INT, DP_FRACTION
     *     as DP fraction: 0.0003725290298 */
    {06, 03240},
    /*  1  SPARE
     *     scale types: spare
     *     as DP fraction: 0 */
    {0, 0},
    /*  2  CDU DEGREES, Y OPTICS DEGREES
     *     scale types: CDU_DEG, Y_OPTICS_DEG
     *     as DP fraction: 0 */
    {0, 0},
    /*  3  DP DEGREES (90)
     *     scale types: DP_DEG90
     *     as DP fraction: 0.2777777798 */
    {010707, 03435},
    /*  4  DP DEGREES (360)(POINT BETWN BITS 11-12)
     *     scale types: DP_DEG360
     *     as DP fraction: 0.3472222276 */
    {013070, 034345},
    /*  5  DEGREES (180)
     *     scale types: DEG180
     *     as DP fraction: 0.0003390833735 */
    {05, 021616},
    /*  6  WEIGHT2
     *     scale types: WEIGHT2
     *     as DP fraction: 0.6921270303 */
    {026113, 031713},
    /*  7  POSITION5
     *     scale types: POS5
     *     as DP fraction: 0.003449618816 */
    {070, 020460},
    /*  8  POSITION4
     *     scale types: POS4
     *     as DP fraction: 0.03449618816 */
    {01065, 05740},
    /*  9  VELOCITY2 (POINT BETWN BITS 11-12)
     *     scale types: VEL2
     *     as DP fraction: 0.2976562493 */
    {011414, 031463},
    /* 10  VELOCITY3
     *     scale types: VEL3
     *     as DP fraction: 0.2381250001 */
    {07475, 016051},
    /* 11  ELEVATION DEGREES
     *     scale types: ELEV_DEG
     *     as DP fraction: 6.781518459e-05 */
    {01, 03434},
    /* 12  TRIM DEGREES
     *     scale types: TRIM_DEG
     *     as DP fraction: 0.0001570172608 */
    {02, 022245},
    /* 13  INERTIA, THRUST MOMENT
     *     scale types: INERTIA, THRUST_MOMENT
     *     as DP fraction: 0.0007891915739 */
    {014, 035607},
    /* 14  VELOCITY/2VS
     *     scale types: VEL_2VS
     *     as DP fraction: 0.2425658703 */
    {07606, 06300},
    /* 15  POSITION 6
     *     scale types: POS6
     *     as DP fraction: 0.4624812864 */
    {016631, 011307},
    /* 16  DRAG ACCELERATION (POINT BETWN BITS 7-8)
     *     scale types: DRAG_ACCEL
     *     as DP fraction: 0.3125 */
    {012000, 0},
    /* 17  POSITION 8
     *     scale types: POS8
     *     as DP fraction: 0.7264639027 */
    {027176, 014235},
    /* 18  POSITION 9 [2DEC 30480 B-19]
     *     scale types: POS9
     *     as DP fraction: 0.05813598633 */
    {01670, 020000},
    /* 19  VELOCITY4 [2DEC 30.48 B-7]
     *     scale types: VEL4
     *     as DP fraction: 0.2381250001 */
    {07475, 016051},
};

/* SFOUTAB, Comanche055 (CSM): one DP pair (high, low) per SF constant code. */
static const sf_pair_t sfoutab_csm[20] = {
    /*  0  WHOLE, DP TIME (SEC)
     *     scale types: OCTAL, FRACTION, WHOLE, HMS, MIN_SEC, TIME_SEC_DP, TWO_INT, DP_FRACTION
     *     as DP fraction: 0.1638399996 */
    {05174, 013261},
    /*  1  SPARE
     *     scale types: spare
     *     as DP fraction: 0 */
    {0, 0},
    /*  2  CDU DEGREES, Y OPTICS DEGREES
     *     scale types: CDU_DEG, Y_OPTICS_DEG
     *     as DP fraction: 0 */
    {0, 0},
    /*  3  DP DEGREES (90) (POINT BETWN BITS 7-8)
     *     scale types: DP_DEG90
     *     as DP fraction: 0.02812499925 */
    {0714, 031463},
    /*  4  DP DEGREES (360)
     *     scale types: DP_DEG360
     *     as DP fraction: 0.3599999994 */
    {013412, 07534},
    /*  5  DEGREES (180)
     *     scale types: DEG180
     *     as DP fraction: 0.1799999997 */
    {05605, 03656},
    /*  6  WEIGHT2
     *     scale types: WEIGHT2
     *     as DP fraction: 8.818507195e-05 */
    {01, 016170},
    /*  7  POSITION5
     *     scale types: POS5
     *     as DP fraction: 0.01769330353 */
    {0441, 034306},
    /*  8  POSITION4
     *     scale types: POS4
     *     as DP fraction: 0.2264742963 */
    {07176, 021603},
    /*  9  VELOCITY2
     *     scale types: VEL2
     *     as DP fraction: 0.419947505 */
    {015340, 015340},
    /* 10  VELOCITY3 (POINT BETWN BITS 7-8)
     *     scale types: VEL3
     *     as DP fraction: 0.03280840069 */
    {01031, 021032},
    /* 11  ELEVATION DEGREES
     *     scale types: ELEV_DEG
     *     as DP fraction: 0.8999999985 */
    {034631, 023146},
    /* 12  TRIM DEGREES
     *     scale types: TRIM_DEG
     *     as DP fraction: 0.3887103982 */
    {014340, 024145},
    /* 13  INERTIA, THRUST MOMENT
     *     scale types: INERTIA, THRUST_MOMENT
     *     as DP fraction: 0.07733899727 */
    {02363, 03721},
    /* 14  VELOCITY/ZVS
     *     scale types: VEL_2VS
     *     as DP fraction: 0.5153239444 */
    {020373, 02122},
    /* 15  POSITION 6 (POINT BETWN BITS 7-8)
     *     scale types: POS6
     *     as DP fraction: 0.01689257473 */
    {0424, 030446},
    /* 16  DRAG ACCELERATION
     *     scale types: DRAG_ACCEL
     *     as DP fraction: 0.02499999851 */
    {0631, 023146},
    /* 17  POSITION 8
     *     scale types: POS8
     *     as DP fraction: 0.01075414941 */
    {0260, 06213},
    /* 18  POSITION 9 [2DEC 17.2010499 B-7]
     *     scale types: POS9
     *     as DP fraction: 0.1343832016 */
    {04231, 027400},
    /* 19  VELOCITY4 [2DEC .032808399]
     *     scale types: VEL4
     *     as DP fraction: 0.03280840069 */
    {01031, 021032},
};

/* SFINTAB, Luminary099 (LM): one DP pair (high, low) per SF constant code. */
static const sf_pair_t sfintab_lm[29] = {
    /*  0  WHOLE, DP TIME (SEC)
     *     scale types: OCTAL, FRACTION, WHOLE, HMS, MIN_SEC, TIME_SEC_DP, TWO_INT, LR_POSITION
     *     as DP fraction: 0.0003725290298 */
    {06, 03240},
    /*  1  SPARE
     *     scale types: spare
     *     as DP fraction: 0 */
    {0, 0},
    /*  2  CDU DEGREES, 360-CDU DEGREES
     *     scale types: CDU_DEG, CDU360
     *     as DP fraction: 0 */
    {0, 0},
    /*  3  DP DEGREES (90)
     *     scale types: DP_DEG90
     *     as DP fraction: 0.2777777798 */
    {010707, 03435},
    /*  4  DP DEGREES (360) (POINT BETWN BITS 11-12)
     *     scale types: DP_DEG360
     *     as DP fraction: 0.3472222276 */
    {013070, 034345},
    /*  5  DEGREES (180)
     *     scale types: DEG180, TRACKER_AZ_DEG
     *     as DP fraction: 0.0003390833735 */
    {05, 021616},
    /*  6  WEIGHT2
     *     scale types: WEIGHT2
     *     as DP fraction: 0.6921270303 */
    {026113, 031713},
    /*  7  POSITION5
     *     scale types: POS5
     *     as DP fraction: 0.003449618816 */
    {070, 020460},
    /*  8  POSITION4
     *     scale types: POS4
     *     as DP fraction: 0.03449618816 */
    {01065, 05740},
    /*  9  VELOCITY2 (POINT BETWN BITS 11-12)
     *     scale types: VEL2
     *     as DP fraction: 0.2976562493 */
    {011414, 031463},
    /* 10  VELOCITY3
     *     scale types: VEL3
     *     as DP fraction: 0.2381250001 */
    {07475, 016051},
    /* 11  ELEVATION DEGREES
     *     scale types: ELEV_DEG
     *     as DP fraction: 6.781518459e-05 */
    {01, 03434},
    /* 12  RENDEZVOUS RADAR RANGE
     *     scale types: RR_RANGE
     *     as DP fraction: 0.002413142473 */
    {047, 021135},
    /* 13  RENDESVOUS RADAR RANGE RATE
     *     scale types: RR_RATE
     *     as DP fraction: -0.0005933865905 */
    {077766, 050711},
    /* 14  LANDING RADAR ALTITUDE [2DEC .9267840599 E5 B-28*]
     *     scale types: LR_ALT
     *     as DP fraction: 0.0003452524543 */
    {05, 025006},
    /* 15  INITIAL/FINAL ALTITUDE
     *     scale types: INIT_FINAL_ALT
     *     as DP fraction: 0.0001588612795 */
    {02, 023224},
    /* 16  ALTITUDE RATE
     *     scale types: ALT_RATE
     *     as DP fraction: 0.0007450580597 */
    {014, 06500},
    /* 17  FORWARD/LATERAL VELOCITY
     *     scale types: FWD_LAT_VEL
     *     as DP fraction: 0.0006686933339 */
    {012, 036455},
    /* 18  ROT HAND CONT ANGLE RATE
     *     scale types: RHC_RATE
     *     as DP fraction: 0.135633681 */
    {04256, 07071},
    /* 19  LANDING RADAR VELX [2DEC -1.552795030 E5 B-28*]
     *     scale types: LR_VELX
     *     as DP fraction: -0.0005784630775 */
    {077766, 060557},
    /* 20  LANDING RADAR VELY [2DEC .8250825087 E5 B-28*]
     *     scale types: LR_VELY
     *     as DP fraction: 0.0003073662519 */
    {05, 01114},
    /* 21  LANDING RADAR VELZ [2DEC 1.153668673 E5 B-28*]
     *     scale types: LR_VELZ
     *     as DP fraction: 0.0004297755659 */
    {07, 01247},
    /* 22  POSITION7
     *     scale types: POS7
     *     as DP fraction: 0.1379847527 */
    {04324, 027600},
    /* 23  TRIM DEGREES2
     *     scale types: TRIM_DEG2
     *     as DP fraction: 0.001862645149 */
    {036, 020440},
    /* 24  COMPUTED ALTITUDE
     *     scale types: COMP_ALT
     *     as DP fraction: 0.001816749573 */
    {035, 030400},
    /* 25  DP DEGREES
     *     scale types: DP_DEGREES
     *     as DP fraction: 0.6103515625 */
    {023420, 0},
    /* 26  POSITION 9 [2DEC 30480 B-19]
     *     scale types: POS9
     *     as DP fraction: 0.05813598633 */
    {01670, 020000},
    /* 27  VELOCITY4 [2DEC 30.48 B-7]
     *     scale types: VEL4
     *     as DP fraction: 0.2381250001 */
    {07475, 016051},
    /* 28  RADIANS [2DEC 100 B-8]
     *     scale types: RADIANS
     *     as DP fraction: 0.390625 */
    {014400, 0},
};

/* SFOUTAB, Luminary099 (LM): one DP pair (high, low) per SF constant code. */
static const sf_pair_t sfoutab_lm[29] = {
    /*  0  WHOLE, DP TIME (SEC)
     *     scale types: OCTAL, FRACTION, WHOLE, HMS, MIN_SEC, TIME_SEC_DP, TWO_INT, LR_POSITION
     *     as DP fraction: 0.1638399996 */
    {05174, 013261},
    /*  1  SPARE
     *     scale types: spare
     *     as DP fraction: 0 */
    {0, 0},
    /*  2  CDU DEGREES, 360-CDU DEGREES
     *     scale types: CDU_DEG, CDU360
     *     as DP fraction: 0 */
    {0, 0},
    /*  3  DP DEGREES (90) (POINT BETWN BITS 7-8)
     *     scale types: DP_DEG90
     *     as DP fraction: 0.02812499925 */
    {0714, 031463},
    /*  4  DP DEGREES (360)
     *     scale types: DP_DEG360
     *     as DP fraction: 0.3599999994 */
    {013412, 07534},
    /*  5  DEGREES (180)
     *     scale types: DEG180, TRACKER_AZ_DEG
     *     as DP fraction: 0.1799999997 */
    {05605, 03656},
    /*  6  WEIGHT2
     *     scale types: WEIGHT2
     *     as DP fraction: 8.818507195e-05 */
    {01, 016170},
    /*  7  POSITION5
     *     scale types: POS5
     *     as DP fraction: 0.01769330353 */
    {0441, 034306},
    /*  8  POSITION4 (POINT BETWN BITS 7-8)
     *     scale types: POS4
     *     as DP fraction: 0.2264742963 */
    {07176, 021603},
    /*  9  VELOCITY2
     *     scale types: VEL2
     *     as DP fraction: 0.419947505 */
    {015340, 015340},
    /* 10  VELOCITY3 (POINT BETWN BITS 7-8)
     *     scale types: VEL3
     *     as DP fraction: 0.03280840069 */
    {01031, 021032},
    /* 11  ELEVATION DETREES
     *     scale types: ELEV_DEG
     *     as DP fraction: 0.8999999985 */
    {034631, 023146},
    /* 12  RENDEZVOUS RADAR RANGE
     *     scale types: RR_RANGE
     *     as DP fraction: 0.02529279143 */
    {0636, 014552},
    /* 13  RENDEZVOUS RADAR RANGE RATE
     *     scale types: RR_RATE
     *     as DP fraction: -0.102858752 */
    {074552, 070307},
    /* 14  LANDING RADAR ALTITUDE [2DEC 1.079 E-5 B14]
     *     scale types: LR_ALT
     *     as DP fraction: 0.1767833605 */
    {05520, 015312},
    /* 15  INITIAL/FINAL ALTITUDE
     *     scale types: INIT_FINAL_ALT
     *     as DP fraction: 0.3842048012 */
    {014226, 031757},
    /* 16  ALTITUDE RATE
     *     scale types: ALT_RATE
     *     as DP fraction: 0.08192000166 */
    {02476, 05531},
    /* 17  FORWARD/LATERAL VELOCTY
     *     scale types: FWD_LAT_VEL
     *     as DP fraction: 0.09127526358 */
    {02727, 016415},
    /* 18  ROT HAND CONT ANGLE RATE
     *     scale types: RHC_RATE
     *     as DP fraction: 0.0004500001669 */
    {07, 013734},
    /* 19  LANDING RADAR VELX [2DEC -.6440 E-5 B14]
     *     scale types: LR_VELX
     *     as DP fraction: -0.1055129617 */
    {074477, 050643},
    /* 20  LANDING RADAR VELY [2DEC 1.212 E-5 B14]
     *     scale types: LR_VELY
     *     as DP fraction: 0.1985740811 */
    {06265, 016004},
    /* 21  LANDING RADAR VELZ [2DEC .8668 E-5 B14]
     *     scale types: LR_VELZ
     *     as DP fraction: 0.1420165114 */
    {04426, 031433},
    /* 22  POSITION7
     *     scale types: POS7
     *     as DP fraction: 0.9058971927 */
    {034772, 07016},
    /* 23  TRIM DEGREES2
     *     scale types: TRIM_DEG2
     *     as DP fraction: 0.03276799992 */
    {01030, 033675},
    /* 24  COMPUTED ALTITUDE
     *     scale types: COMP_ALT
     *     as DP fraction: 0.0335958004 */
    {01046, 015700},
    /* 25  DP DEGREES
     *     scale types: DP_DEGREES
     *     as DP fraction: 0.01280000061 */
    {0321, 026706},
    /* 26  POSITION 9 [2DEC 17.2010499 B-7]
     *     scale types: POS9
     *     as DP fraction: 0.1343832016 */
    {04231, 027400},
    /* 27  VELOCITY4 [2DEC .032808399]
     *     scale types: VEL4
     *     as DP fraction: 0.03280840069 */
    {01031, 021032},
    /* 28  RADIANS [2DEC .32]
     *     scale types: RADIANS
     *     as DP fraction: 0.3200000003 */
    {012172, 034122},
};

static const scale_def_t scale_defs[AGC_SCALE_COUNT] = {
    /* A   xxxxx octal: SP, octal word */
    [AGC_SCALE_OCTAL] = {0, 0, 0, 0, false, true},
    /* B   .xxxxx straight fractional: SP, bit 1 = 2^-14 unit */
    [AGC_SCALE_FRACTION] = {1, 0, 1, 0, false, false},
    /* C   xxxxx. integer: SP, bit 1 = 1 unit (max 16383) */
    [AGC_SCALE_WHOLE] = {3, 0, 3, 0, false, false},
    /* D   xxx.xx deg (CDU): SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) */
    [AGC_SCALE_CDU_DEG] = {2, 2, 2, 2, false, true},
    /* E   xx.xxx deg (elevation, max 89.999): SP, bit 1 = 90/2^14 deg */
    [AGC_SCALE_ELEV_DEG] = {3, 11, 3, 11, false, false},
    /* F   xxx.xx deg (180): SP, bit 1 = 180/2^14 deg */
    [AGC_SCALE_DEG180] = {3, 5, 3, 5, false, false},
    /* G   xx.xxx deg (DP, 90): DP, bit 1 of low word = 360/2^28 deg */
    [AGC_SCALE_DP_DEG90] = {7, 3, 7, 3, true, false},
    /* H   xxx.xx deg (DP, 360): DP, bit 1 of low word = 360/2^28 deg */
    [AGC_SCALE_DP_DEG360] = {10, 4, 10, 4, true, false},
    /* J   xx.xxx deg (optics Y; +19.775 deg bias on display, no negative load): SP, bit 1 = 90/2^15 deg */
    [AGC_SCALE_Y_OPTICS_DEG] = {6, 2, -1, -1, false, true},
    /* K   hhhhh. hrs / mmmmm. min / ss.ss sec across R1,R2,R3 (dec only, all 3 loaded): DP centiseconds, bit 1 of low word = 10^-2 s (max 745h39m14.55s) */
    [AGC_SCALE_HMS] = {8, 0, 8, 0, true, false},
    /* L   xxBxx min/sec (display only, caps at 59B59): DP centiseconds, bit 1 of low word = 10^-2 s */
    [AGC_SCALE_MIN_SEC] = {9, 0, 9, 0, true, false},
    /* N   xxx.xx sec (DP): DP centiseconds, bit 1 of low word = 10^-2 s */
    [AGC_SCALE_TIME_SEC_DP] = {4, 0, 4, 0, true, false},
    /* P   xxxxx. ft/sec: DP, bit 1 of high word = 2^-7 m/cs */
    [AGC_SCALE_VEL2] = {10, 9, 10, 9, true, false},
    /* Q   xxxx.x nautical miles: DP, bit 1 of low word = 2 m */
    [AGC_SCALE_POS4] = {7, 8, 7, 8, true, false},
    /* S   xxxx.x ft/sec: DP, bit 1 of high word = 2^-7 m/cs */
    [AGC_SCALE_VEL3] = {7, 10, 7, 10, true, false},
    /* JJ  xxx.xx nautical miles: DP, bit 1 of low word = 2 m */
    [AGC_SCALE_POS5] = {4, 7, 4, 7, true, false},
    /* KK  xxxxx. lbs: SP, fractional part of 2^16 kg */
    [AGC_SCALE_WEIGHT2] = {11, 6, 11, 6, false, false},
    /* PP  +xxByy two integers (display only): DP, high word = XX, low word = YY, each a positive integer < 100 */
    [AGC_SCALE_TWO_INT] = {12, 0, 12, 0, true, false},
    /* XX  xxxxx. feet: DP, bit 1 of low word = 2^-9 m */
    [AGC_SCALE_POS9] = {7, 18, 7, 26, true, false},
    /* YY  xxxx.x ft/sec (max 328.0): DP, fractional part of m/cs */
    [AGC_SCALE_VEL4] = {5, 19, 5, 27, true, false},
    /* FF  xxx.xx deg (trim): SP, low-order bit = 85.41 arc-sec */
    [AGC_SCALE_TRIM_DEG] = {3, 12, -1, -1, false, false},
    /* GG  xxxxxBB. slug ft^2: SP, fractional part of 2^20 kg m^2 */
    [AGC_SCALE_INERTIA] = {3, 13, -1, -1, false, false},
    /* II  xxxxxBB. ft lbs: SP, fractional part of 2^20 N m */
    [AGC_SCALE_THRUST_MOMENT] = {3, 13, -1, -1, false, false},
    /* UU  xxxxx. ft/sec (max 51532): DP, fractional part of 2VS ft/sec (VS = 25766.1973) */
    [AGC_SCALE_VEL_2VS] = {10, 14, -1, -1, true, false},
    /* LL  xxxx.x nautical miles: DP, bit 1 of low word = (6373338)(2 PI) 2^-28 / 1852 nmi */
    [AGC_SCALE_POS6] = {7, 15, -1, -1, true, false},
    /* MM  xxx.xx g (max 24.99): DP, bit 1 of low word = 25 x 2^-28 g */
    [AGC_SCALE_DRAG_ACCEL] = {5, 16, -1, -1, true, false},
    /* VV  xxxx.x nautical miles: DP, bit 1 of low word = 4 x 6373338 x 2^-28 / 1852 nmi */
    [AGC_SCALE_POS8] = {7, 17, -1, -1, true, false},
    /* ZZ  .xxxxx DP fractional: DP, bit 1 of high word = 2^-14 unit */
    [AGC_SCALE_DP_FRACTION] = {13, 0, -1, -1, true, false},
    /* TT  +0000x landing radar position 1/2 (display only; reads channel 33 bits 6,7): channel 33: bit 6 = not position 1, bit 7 = not position 2 */
    [AGC_SCALE_LR_POSITION] = {-1, -1, 6, 0, false, true},
    /* WW  xxx.xx deg (360 - CDU): SP, bit 1 = 360 - 360/2^15 deg */
    [AGC_SCALE_CDU360] = {-1, -1, 13, 2, false, true},
    /* EE  xxx.xx deg (optical tracker azimuth): DP, low bit of low word = 360/2^15 deg */
    [AGC_SCALE_TRACKER_AZ_DEG] = {-1, -1, 4, 5, true, false},
    /* U   xxx.xx nautical miles (rendezvous radar range): DP, low bit of low word = 9.38 ft */
    [AGC_SCALE_RR_RANGE] = {-1, -1, 4, 12, true, false},
    /* V   xxxxx. ft/sec (rendezvous radar range rate): DP, low bit of low word = -0.6278 ft/sec */
    [AGC_SCALE_RR_RATE] = {-1, -1, 4, 13, true, false},
    /* W   xxxxx. feet (landing radar altitude): DP, low bit of low word = 1.079 ft */
    [AGC_SCALE_LR_ALT] = {-1, -1, 4, 14, true, false},
    /* AA  xxxxx. feet (initial/final altitude): DP, low bit of low word = 2.345 ft */
    [AGC_SCALE_INIT_FINAL_ALT] = {-1, -1, 4, 15, true, false},
    /* BB  xxxxx. ft/sec (altitude rate, max 8191): SP, low bit = 0.5 ft/sec */
    [AGC_SCALE_ALT_RATE] = {-1, -1, 3, 16, false, false},
    /* CC  xxxxx. ft/sec (forward/lateral velocity, max 9126): SP, low bit = 0.5571 ft/sec */
    [AGC_SCALE_FWD_LAT_VEL] = {-1, -1, 3, 17, false, false},
    /* DD  xxxxx. deg/sec (rotational hand controller rates, max 44): SP, fractional part of PI rad per 4 sec */
    [AGC_SCALE_RHC_RATE] = {-1, -1, 3, 18, false, false},
    /* X   xxxxx. ft/sec (landing radar VELX): DP, low bit of low word = -0.6440 ft/sec */
    [AGC_SCALE_LR_VELX] = {-1, -1, 4, 19, true, false},
    /* Y   xxxxx. ft/sec (landing radar VELY): DP, low bit of low word = 1.212 ft/sec */
    [AGC_SCALE_LR_VELY] = {-1, -1, 4, 20, true, false},
    /* Z   xxxxx. ft/sec (landing radar VELZ): DP, low bit of low word = 0.8668 ft/sec */
    [AGC_SCALE_LR_VELZ] = {-1, -1, 4, 21, true, false},
    /* QQ  xxxx.x nautical miles (max 9058.9): DP, bit 1 of low word = 2^-4 m */
    [AGC_SCALE_POS7] = {-1, -1, 10, 22, true, false},
    /* NN  xxx.xx deg (trim, max 32.76): SP, bit 1 = 0.01 sec of time */
    [AGC_SCALE_TRIM_DEG2] = {-1, -1, 3, 23, false, false},
    /* RR  xxxxx. feet (computed altitude): DP, bit 1 of low word = 2^-4 m */
    [AGC_SCALE_COMP_ALT] = {-1, -1, 4, 24, true, false},
    /* SS  xxxx.x deg (DP degrees): DP, bit 1 of high word = 1 degree */
    [AGC_SCALE_DP_DEGREES] = {-1, -1, 7, 25, true, false},
    /* AAA xx.xxx radians (max 31.999): DP, bit 1 of high word = 2^-9 rad */
    [AGC_SCALE_RADIANS] = {-1, -1, 10, 28, true, false},
};

/**********************************************************************
 * AGC: none (table decode)
 *
 * Ones' complement 15-bit AGC word -> signed value (-0 maps to 0).
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   w         uint16_t  I    15-bit ones' complement word
 *
 * RETURNS: int32_t -- signed value of the word (-0 gives 0)
 **********************************************************************/
static int32_t sf_word_value(uint16_t w)
{
    if (!AGC_ASSERT(w <= 077777u))
    {
        return (0);
    }

    if (w >= 040000u)
    {
        return (-(int32_t) (077777u - w));
    }

    return ((int32_t) w);
}

/**********************************************************************
 * AGC: GTSFIN, GTSFOUT (PINBALL_NOUN_TABLES.agc, CM p. 271)
 *
 * A DP pair as the agc_dp_t fraction value (high * 2^14 + low).
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -------------------
 *   p         const sf_pair_t *  I    SFINTAB/SFOUTAB word pair (high,
 *                                     low)
 *
 * RETURNS: agc_dp_t -- high x 2^14 + low
 **********************************************************************/
static agc_dp_t sf_pair_value(const sf_pair_t *p)
{
    if (!AGC_ASSERT(p != NULL))
    {
        return (0);
    }

    return ((agc_dp_t) (sf_word_value(p->hi) * 16384 + sf_word_value(p->lo)));
}

/**********************************************************************
 * GTSFIN / GTSFOUT (PINBALL_NOUN_TABLES.agc p. 271 / 305): table fetch.
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  -------------------
 *   veh       agc_vehicle_t  I    vehicle whose tables are used (CSM or
 *                                 LM)
 *   sf_code   uint8_t        I    SF constant code (agc_sfconst_t)
 *   sf_in     agc_dp_t *     O    input constant (SFINTAB), DP
 *   sf_out    agc_dp_t *     O    output constant (SFOUTAB), DP
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_scale_sfconst(agc_vehicle_t veh, uint8_t sf_code,
                               agc_dp_t *sf_in, agc_dp_t *sf_out)
{
    if (!AGC_ASSERT(sf_in != NULL) || !AGC_ASSERT(sf_out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (veh == AGC_VEHICLE_CSM)
    {
        if (!AGC_ASSERT(sf_code < SF_COUNT_CSM))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        *sf_in  = sf_pair_value(&sfintab_csm[sf_code]);
        *sf_out = sf_pair_value(&sfoutab_csm[sf_code]);

        return (AGC_STATUS_OK);
    }

    if (!AGC_ASSERT(veh == AGC_VEHICLE_LM) ||
        !AGC_ASSERT(sf_code < SF_COUNT_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *sf_in  = sf_pair_value(&sfintab_lm[sf_code]);
    *sf_out = sf_pair_value(&sfoutab_lm[sf_code]);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: GTSFIN, GTSFOUT (PINBALL_NOUN_TABLES.agc, CM p. 271)
 *
 * SF routine / constant codes and constants for a scale type, per vehicle.
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  -------------------
 *   veh       agc_vehicle_t       I    vehicle whose tables are used (CSM
 *                                      or LM)
 *   scale     agc_scale_t         I    scale type of a noun component
 *   out       agc_scale_info_t *  O    routine and constants of the scale
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition,
 *          NOT_FOUND; else the status of the routine it calls
 **********************************************************************/
agc_status_t agc_scale_info_vehicle(agc_vehicle_t veh, agc_scale_t scale,
                                    agc_scale_info_t *out)
{
    if (!AGC_ASSERT(out != NULL) || !AGC_ASSERT(scale > AGC_SCALE_NONE) ||
        !AGC_ASSERT(scale < AGC_SCALE_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(veh == AGC_VEHICLE_CSM || veh == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const scale_def_t *def     = &scale_defs[scale]; /* scale definition row */
    const int          routine = /* SF routine of the vehicle */
                                 (veh ==
                                  AGC_VEHICLE_CSM) ? def->csm_routine : def->
                                 lm_routine;
    const int code = /* SF constant code of the vehicle */
                     (veh == AGC_VEHICLE_CSM) ? def->csm_code : def->lm_code;

    if (routine < 0)
    {
        return (AGC_STATUS_NOT_FOUND); /* not defined in this program */
    }

    agc_status_t st; /* status so far */

    st = agc_scale_sfconst(veh, (uint8_t) code, &out->sf_in, &out->sf_out);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    out->scale   = scale;
    out->routine = (uint8_t) routine;
    out->sf_code = (uint8_t) code;
    out->is_dp   = def->is_dp;
    out->is_raw  = def->is_raw;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: GTSFIN, GTSFOUT (PINBALL_NOUN_TABLES.agc, CM p. 271)
 *
 * Same, for the vehicle selected by agc_state_init().
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  -------------------
 *   scale     agc_scale_t         I    scale type of a noun component
 *   out       agc_scale_info_t *  O    routine and constants of the scale
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t agc_scale_info(agc_scale_t scale, agc_scale_info_t *out)
{
    if (!AGC_ASSERT(out != NULL) || !AGC_ASSERT(scale < AGC_SCALE_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_scale_info_vehicle(agc_state_vehicle(), scale, out));
}
