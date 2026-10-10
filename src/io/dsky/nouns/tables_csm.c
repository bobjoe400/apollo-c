/*
 * noun_tables_csm.c -- PINBALL noun tables for the Command Module (Comanche 055).
 *
 * Translated from Comanche055/PINBALL_NOUN_TABLES.agc (pp. 268-284). NNADTAB / NNTYPTAB /
 * IDADDTAB / RUTMXTAB are resolved here into one flat agc_noun_t per noun:
 *
 *   normal nouns 00-39: NNTYPTAB gives component count, DEC-only / NO-LOAD
 *     bits, and ONE (SF routine, SF constant) pair shared by every component;
 *     NNADTAB gives the address of the first component, so components 1..2
 *     are the following erasable words (CDUX,CDUY,CDUZ ...).
 *   mixed nouns 40-99: NNADTAB gives the component code and the IDADDTAB
 *     index (3 words per noun); NNTYPTAB carries three 5-bit SF constant
 *     codes and RUTMXTAB three 5-bit SF routine codes (component 1 in the
 *     low field); IDADDTAB gives one erasable per component.
 *
 * Every (routine, constant) pair is mapped to an agc_scale_t (the letter in
 * ASSEMBLY_AND_OPERATION_INFORMATION "NOUN SCALES AND FORMATS"). The octal
 * words quoted in the comments were verified against the assembled core
 * rope. The table is indexed by noun number (entry n has .noun == n); spare
 * nouns have ncomp == 0.
 */
#include "agc/io/nouns.h"

const agc_noun_t agc_noun_table_csm[AGC_NOUN_COUNT] = {
    /* 00  Not in use */
    {0,
     0,
     {AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE},
     {AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE},
     false,
     false},
    /* 01  Specify machine address (fractional)
     *     3 comp; NNADTAB 40000, NNTYPTAB 04040 */
    {1,
     3,
     {AGC_SCALE_FRACTION, AGC_SCALE_FRACTION, AGC_SCALE_FRACTION},
     {AGC_DATA_SPECIFY_ADDR, AGC_DATA_SPECIFY_ADDR, AGC_DATA_SPECIFY_ADDR},
     false,
     false},
    /* 02  Specify machine address (whole)
     *     3 comp; NNADTAB 40000, NNTYPTAB 04140 */
    {2,
     3,
     {AGC_SCALE_WHOLE, AGC_SCALE_WHOLE, AGC_SCALE_WHOLE},
     {AGC_DATA_SPECIFY_ADDR, AGC_DATA_SPECIFY_ADDR, AGC_DATA_SPECIFY_ADDR},
     false,
     false},
    /* 03  Specify machine address (degrees)
     *     3 comp; NNADTAB 40000, NNTYPTAB 04102 */
    {3,
     3,
     {AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG},
     {AGC_DATA_SPECIFY_ADDR, AGC_DATA_SPECIFY_ADDR, AGC_DATA_SPECIFY_ADDR},
     false,
     false},
    /* 04  Spare */
    {4,
     0,
     {AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE},
     {AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE},
     false,
     false},
    /* 05  Angular error/difference
     *     1 comp; NNADTAB ECADR DSPTEM1, NNTYPTAB 00504 */
    {5,
     1,
     {AGC_SCALE_DP_DEG360, AGC_SCALE_NONE, AGC_SCALE_NONE},
     {AGC_DATA_DSPTEM1_DP, AGC_DATA_NONE, AGC_DATA_NONE},
     false,
     false},
    /* 06  Option code
     *     2 comp; NNADTAB ECADR OPTION1, NNTYPTAB 02000 */
    {6,
     2,
     {AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_NONE},
     {AGC_DATA_OPTION1, AGC_DATA_OPTION2, AGC_DATA_NONE},
     false,
     false},
    /* 07  ECADR of word to be modified / Ones for bits to be modified / 1 to set or 0 to reset selected bits
     *     3 comp; NNADTAB ECADR XREG, NNTYPTAB 04000 */
    {7,
     3,
     {AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_OCTAL},
     {AGC_DATA_XREG, AGC_DATA_YREG, AGC_DATA_ZREG},
     false,
     false},
    /* 08  Alarm data
     *     3 comp; NNADTAB ECADR ALMCADR, NNTYPTAB 04000 */
    {8,
     3,
     {AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_OCTAL},
     {AGC_DATA_ALMCADR, AGC_DATA_ALMCADR_P1, AGC_DATA_ALMCADR_P2},
     false,
     false},
    /* 09  Alarm codes
     *     3 comp; NNADTAB ECADR FAILREG, NNTYPTAB 04000 */
    {9,
     3,
     {AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_OCTAL},
     {AGC_DATA_FAILREG, AGC_DATA_FAILREG_P1, AGC_DATA_FAILREG_P2},
     false,
     false},
    /* 10  Channel to be specified
     *     1 comp; NNADTAB 77776, NNTYPTAB 00000 */
    {10,
     1,
     {AGC_SCALE_OCTAL, AGC_SCALE_NONE, AGC_SCALE_NONE},
     {AGC_DATA_SPECIFY_CHANNEL, AGC_DATA_NONE, AGC_DATA_NONE},
     false,
     false},
    /* 11  TIG of CSI (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR TCSI, NNTYPTAB 24400 */
    {11,
     3,
     {AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS},
     {AGC_DATA_TCSI, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
    /* 12  Option code / (used by extended verbs only)
     *     2 comp; NNADTAB ECADR OPTIONX, NNTYPTAB 02000 */
    {12,
     2,
     {AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_NONE},
     {AGC_DATA_OPTIONX, AGC_DATA_OPTIONX_P1, AGC_DATA_NONE},
     false,
     false},
    /* 13  TIG of CDH (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR TCDH, NNTYPTAB 24400 */
    {13,
     3,
     {AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS},
     {AGC_DATA_TCDH, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
    /* 14  Spare */
    {14,
     0,
     {AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE},
     {AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE},
     false,
     false},
    /* 15  Increment machine address
     *     1 comp; NNADTAB 77777, NNTYPTAB 00000 */
    {15,
     1,
     {AGC_SCALE_OCTAL, AGC_SCALE_NONE, AGC_SCALE_NONE},
     {AGC_DATA_INCREMENT_ADDR, AGC_DATA_NONE, AGC_DATA_NONE},
     false,
     false},
    /* 16  Time of event (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR DSPTEMX, NNTYPTAB 24400 */
    {16,
     3,
     {AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS},
     {AGC_DATA_DSPTEMX_DP, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
    /* 17  Astronaut total attitude
     *     3 comp; NNADTAB ECADR CPHIX, NNTYPTAB 04102 */
    {17,
     3,
     {AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG},
     {AGC_DATA_CPHIX, AGC_DATA_CPHIX_P1, AGC_DATA_CPHIX_P2},
     false,
     false},
    /* 18  Auto maneuver ball angles
     *     3 comp; NNADTAB ECADR THETAD, NNTYPTAB 04102 */
    {18,
     3,
     {AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG},
     {AGC_DATA_THETAD, AGC_DATA_THETAD_P1, AGC_DATA_THETAD_P2},
     false,
     false},
    /* 19  Bypass attitude trim maneuver
     *     3 comp; NNADTAB ECADR THETAD, NNTYPTAB 04102 */
    {19,
     3,
     {AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG},
     {AGC_DATA_THETAD, AGC_DATA_THETAD_P1, AGC_DATA_THETAD_P2},
     false,
     false},
    /* 20  ICDU angles
     *     3 comp; NNADTAB ECADR CDUX, NNTYPTAB 04102 */
    {20,
     3,
     {AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG},
     {AGC_DATA_CDUX, AGC_DATA_CDUY, AGC_DATA_CDUZ},
     false,
     false},
    /* 21  PIPAS
     *     3 comp; NNADTAB ECADR PIPAX, NNTYPTAB 04140 */
    {21,
     3,
     {AGC_SCALE_WHOLE, AGC_SCALE_WHOLE, AGC_SCALE_WHOLE},
     {AGC_DATA_PIPAX, AGC_DATA_PIPAY, AGC_DATA_PIPAZ},
     false,
     false},
    /* 22  New ICDU angles
     *     3 comp; NNADTAB ECADR THETAD, NNTYPTAB 04102 */
    {22,
     3,
     {AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG},
     {AGC_DATA_THETAD, AGC_DATA_THETAD_P1, AGC_DATA_THETAD_P2},
     false,
     false},
    /* 23  Spare */
    {23,
     0,
     {AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE},
     {AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE},
     false,
     false},
    /* 24  Delta time for AGC clock(HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR DSPTEM2 +1, NNTYPTAB 24400 */
    {24,
     3,
     {AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS},
     {AGC_DATA_DSPTEM2_P1_DP, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
    /* 25  Checklist / (used with please perform only)
     *     3 comp; NNADTAB ECADR DSPTEM1, NNTYPTAB 04140 */
    {25,
     3,
     {AGC_SCALE_WHOLE, AGC_SCALE_WHOLE, AGC_SCALE_WHOLE},
     {AGC_DATA_DSPTEM1, AGC_DATA_DSPTEM1_P1, AGC_DATA_DSPTEM1_P2},
     false,
     false},
    /* 26  Prio/delay, ADRES, BBCON
     *     3 comp; NNADTAB ECADR DSPTEM1, NNTYPTAB 04000 */
    {26,
     3,
     {AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_OCTAL},
     {AGC_DATA_DSPTEM1, AGC_DATA_DSPTEM1_P1, AGC_DATA_DSPTEM1_P2},
     false,
     false},
    /* 27  Self test on/off switch
     *     1 comp; NNADTAB ECADR SMODE, NNTYPTAB 00140 */
    {27,
     1,
     {AGC_SCALE_WHOLE, AGC_SCALE_NONE, AGC_SCALE_NONE},
     {AGC_DATA_SMODE, AGC_DATA_NONE, AGC_DATA_NONE},
     false,
     false},
    /* 28  Spare */
    {28,
     0,
     {AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE},
     {AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE},
     false,
     false},
    /* 29  XSM launch azimuth
     *     1 comp, dec only; NNADTAB ECADR DSPTEM1, NNTYPTAB 20102 */
    {29,
     1,
     {AGC_SCALE_CDU_DEG, AGC_SCALE_NONE, AGC_SCALE_NONE},
     {AGC_DATA_DSPTEM1, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
    /* 30  Target codes
     *     3 comp; NNADTAB ECADR DSPTEM1, NNTYPTAB 04140 */
    {30,
     3,
     {AGC_SCALE_WHOLE, AGC_SCALE_WHOLE, AGC_SCALE_WHOLE},
     {AGC_DATA_DSPTEM1, AGC_DATA_DSPTEM1_P1, AGC_DATA_DSPTEM1_P2},
     false,
     false},
    /* 31  Time of landing site (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR DSPTEM1, NNTYPTAB 24400 */
    {31,
     3,
     {AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS},
     {AGC_DATA_DSPTEM1_DP, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
    /* 32  Time to perigee (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR -TPER, NNTYPTAB 24400 */
    {32,
     3,
     {AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS},
     {AGC_DATA_NEG_TPER, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
    /* 33  Time of ignition (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR TIG, NNTYPTAB 24400 */
    {33,
     3,
     {AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS},
     {AGC_DATA_TIG, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
    /* 34  Time of event (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR DSPTEM1, NNTYPTAB 24400 */
    {34,
     3,
     {AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS},
     {AGC_DATA_DSPTEM1_DP, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
    /* 35  Time to go to event (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR TTOGO, NNTYPTAB 24400 */
    {35,
     3,
     {AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS},
     {AGC_DATA_TTOGO, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
    /* 36  Time of AGC clock (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR TIME2, NNTYPTAB 24400 */
    {36,
     3,
     {AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS},
     {AGC_DATA_TIME2, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
    /* 37  TIG of TPI (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR TTPI, NNTYPTAB 24400 */
    {37,
     3,
     {AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS},
     {AGC_DATA_TTPI, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
    /* 38  Time of state vector
     *     3 comp, dec only; NNADTAB ECADR TET, NNTYPTAB 24400 */
    {38,
     3,
     {AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS},
     {AGC_DATA_TET, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
    /* 39  Delta time to transfer (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR T3TOT4, NNTYPTAB 24400 */
    {39,
     3,
     {AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS},
     {AGC_DATA_T3TOT4, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
    /* 40  Time to ignition/cutoff / Vg / Delta V (accumulated)
     *     3 comp, dec only, no load; NNADTAB 64000, NNTYPTAB 24500, RUTMXTAB 16351 */
    {40,
     3,
     {AGC_SCALE_MIN_SEC, AGC_SCALE_VEL3, AGC_SCALE_VEL3},
     {AGC_DATA_TTOGO, AGC_DATA_VGDISP, AGC_DATA_DVTOTAL},
     true,
     true},
    /* 41  Target azimuth / Elevation
     *     2 comp; NNADTAB 02003, NNTYPTAB 00542, RUTMXTAB 00142 */
    {41,
     2,
     {AGC_SCALE_CDU_DEG, AGC_SCALE_ELEV_DEG, AGC_SCALE_NONE},
     {AGC_DATA_DSPTEM1, AGC_DATA_DSPTEM1_P1, AGC_DATA_NONE},
     false,
     false},
    /* 42  Apogee / Perigee / Delta V (required)
     *     3 comp, dec only; NNADTAB 24006, NNTYPTAB 24410, RUTMXTAB 16347 */
    {42,
     3,
     {AGC_SCALE_POS4, AGC_SCALE_POS4, AGC_SCALE_VEL3},
     {AGC_DATA_HAPO, AGC_DATA_HPER, AGC_DATA_VGDISP},
     true,
     false},
    /* 43  Latitude / Longitude / Altitude
     *     3 comp, dec only; NNADTAB 24011, NNTYPTAB 20204, RUTMXTAB 16512 */
    {43,
     3,
     {AGC_SCALE_DP_DEG360, AGC_SCALE_DP_DEG360, AGC_SCALE_POS4},
     {AGC_DATA_LAT, AGC_DATA_LONG, AGC_DATA_ALT},
     true,
     false},
    /* 44  Apogee / Perigee / TFF
     *     3 comp, dec only, no load; NNADTAB 64014, NNTYPTAB 00410, RUTMXTAB 22347 */
    {44,
     3,
     {AGC_SCALE_POS4, AGC_SCALE_POS4, AGC_SCALE_MIN_SEC},
     {AGC_DATA_HAPOX, AGC_DATA_HPERX, AGC_DATA_TFF},
     true,
     true},
    /* 45  Marks (VHF - optics) / TTI of next burn / MGA
     *     3 comp, dec only, no load; NNADTAB 64017, NNTYPTAB 10000, RUTMXTAB 24454 */
    {45,
     3,
     {AGC_SCALE_TWO_INT, AGC_SCALE_MIN_SEC, AGC_SCALE_DP_DEG360},
     {AGC_DATA_VHFCNT, AGC_DATA_TTOGO, AGC_DATA_PLUS_MGA},
     true,
     true},
    /* 46  Autopilot configuration
     *     2 comp; NNADTAB 02022, NNTYPTAB 00000, RUTMXTAB 00000 */
    {46,
     2,
     {AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_NONE},
     {AGC_DATA_DAPDATR1, AGC_DATA_DAPDATR2, AGC_DATA_NONE},
     false,
     false},
    /* 47  This vehicle weight / Other vehicle weight
     *     2 comp, dec only; NNADTAB 22025, NNTYPTAB 00306, RUTMXTAB 00553 */
    {47,
     2,
     {AGC_SCALE_WEIGHT2, AGC_SCALE_WEIGHT2, AGC_SCALE_NONE},
     {AGC_DATA_CSMMASS, AGC_DATA_LEMMASS, AGC_DATA_NONE},
     true,
     false},
    /* 48  Pitch trim / Yaw trim
     *     2 comp, dec only; NNADTAB 22030, NNTYPTAB 00614, RUTMXTAB 00143 */
    {48,
     2,
     {AGC_SCALE_TRIM_DEG, AGC_SCALE_TRIM_DEG, AGC_SCALE_NONE},
     {AGC_DATA_PACTOFF, AGC_DATA_YACTOFF, AGC_DATA_NONE},
     true,
     false},
    /* 49  Delta R / Delta V / VHF or optics code
     *     3 comp, dec only; NNADTAB 24033, NNTYPTAB 00510, RUTMXTAB 06347 */
    {49,
     3,
     {AGC_SCALE_POS4, AGC_SCALE_VEL3, AGC_SCALE_WHOLE},
     {AGC_DATA_MARK_DELTA_R, AGC_DATA_MARK_DELTA_V, AGC_DATA_MARK_SOURCE_CODE},
     true,
     false},
    /* 50  Splash error / Perigee / TFF
     *     3 comp, dec only, no load; NNADTAB 64036, NNTYPTAB 00417, RUTMXTAB 22347 */
    {50,
     3,
     {AGC_SCALE_POS6, AGC_SCALE_POS4, AGC_SCALE_MIN_SEC},
     {AGC_DATA_RSP_RREC, AGC_DATA_HPERX, AGC_DATA_TFF},
     true,
     true},
    /* 51  S-band antenna pitch / Yaw
     *     2 comp, dec only; NNADTAB 22041, NNTYPTAB 00204, RUTMXTAB 00512 */
    {51,
     2,
     {AGC_SCALE_DP_DEG360, AGC_SCALE_DP_DEG360, AGC_SCALE_NONE},
     {AGC_DATA_RHOSB, AGC_DATA_GAMMASB, AGC_DATA_NONE},
     true,
     false},
    /* 52  Central angle of active vehicle
     *     1 comp; NNADTAB 00044, NNTYPTAB 00004, RUTMXTAB 00012 */
    {52,
     1,
     {AGC_SCALE_DP_DEG360, AGC_SCALE_NONE, AGC_SCALE_NONE},
     {AGC_DATA_ACTCENT, AGC_DATA_NONE, AGC_DATA_NONE},
     false,
     false},
    /* 53  Range / Range rate / Phi
     *     3 comp, dec only; NNADTAB 24047, NNTYPTAB 10507, RUTMXTAB 24344 */
    {53,
     3,
     {AGC_SCALE_POS5, AGC_SCALE_VEL3, AGC_SCALE_DP_DEG360},
     {AGC_DATA_RANGE, AGC_DATA_RRATE, AGC_DATA_RTHETA},
     true,
     false},
    /* 54  Range / Range rate / Theta
     *     3 comp, dec only; NNADTAB 24052, NNTYPTAB 10507, RUTMXTAB 24344 */
    {54,
     3,
     {AGC_SCALE_POS5, AGC_SCALE_VEL3, AGC_SCALE_DP_DEG360},
     {AGC_DATA_RANGE, AGC_DATA_RRATE, AGC_DATA_RTHETA},
     true,
     false},
    /* 55  Perigee code / Elevation angle / Central angle
     *     3 comp, dec only; NNADTAB 24055, NNTYPTAB 10200, RUTMXTAB 24503 */
    {55,
     3,
     {AGC_SCALE_WHOLE, AGC_SCALE_DP_DEG360, AGC_SCALE_DP_DEG360},
     {AGC_DATA_NN1, AGC_DATA_ELEV, AGC_DATA_CENTANG},
     true,
     false},
    /* 56  Reentry angle, / Delta V
     *     2 comp, dec only; NNADTAB 22060, NNTYPTAB 00444, RUTMXTAB 00512 */
    {56,
     2,
     {AGC_SCALE_DP_DEG360, AGC_SCALE_VEL2, AGC_SCALE_NONE},
     {AGC_DATA_RTEGAM2D, AGC_DATA_RTEDVD, AGC_DATA_NONE},
     true,
     false},
    /* 57  Delta R
     *     1 comp, dec only; NNADTAB 20063, NNTYPTAB 00010, RUTMXTAB 00007 */
    {57,
     1,
     {AGC_SCALE_POS4, AGC_SCALE_NONE, AGC_SCALE_NONE},
     {AGC_DATA_DELTAR, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
    /* 58  Perigee alt / Delta V TPI / Delta V TPF
     *     3 comp, dec only; NNADTAB 24066, NNTYPTAB 24510, RUTMXTAB 16347 */
    {58,
     3,
     {AGC_SCALE_POS4, AGC_SCALE_VEL3, AGC_SCALE_VEL3},
     {AGC_DATA_POSTTPI, AGC_DATA_DELVTPI, AGC_DATA_DELVTPF},
     true,
     false},
    /* 59  Delta velocity LOS
     *     3 comp, dec only; NNADTAB 24071, NNTYPTAB 24512, RUTMXTAB 16347 */
    {59,
     3,
     {AGC_SCALE_VEL3, AGC_SCALE_VEL3, AGC_SCALE_VEL3},
     {AGC_DATA_DVLOS, AGC_DATA_DVLOS_P2, AGC_DATA_DVLOS_P4},
     true,
     false},
    /* 60  GMAX / VPRED / Gamma EI
     *     3 comp, dec only; NNADTAB 24074, NNTYPTAB 10440, RUTMXTAB 24503 */
    {60,
     3,
     {AGC_SCALE_WHOLE, AGC_SCALE_VEL2, AGC_SCALE_DP_DEG360},
     {AGC_DATA_GMAX, AGC_DATA_VPRED, AGC_DATA_GAMMAEI},
     true,
     false},
    /* 61  Impact latitude / Impact longitude / Heads up/down
     *     3 comp, dec only; NNADTAB 24077, NNTYPTAB 00204, RUTMXTAB 06512 */
    {61,
     3,
     {AGC_SCALE_DP_DEG360, AGC_SCALE_DP_DEG360, AGC_SCALE_WHOLE},
     {AGC_DATA_LAT_SPL, AGC_DATA_LNG_SPL, AGC_DATA_HEADSUP},
     true,
     false},
    /* 62  Inertial vel mag (V1) / Alt rate change (HDOT) / Alt above pad radius (H)
     *     3 comp, dec only; NNADTAB 24102, NNTYPTAB 20451, RUTMXTAB 16512 */
    {62,
     3,
     {AGC_SCALE_VEL2, AGC_SCALE_VEL2, AGC_SCALE_POS4},
     {AGC_DATA_VMAGI, AGC_DATA_HDOT, AGC_DATA_ALTI},
     true,
     false},
    /* 63  Range 297,431 to splash (RTGO) / Predicted inert vel (VIO) / Time to go to 297,431 (TTE)
     *     3 comp, dec only, no load; NNADTAB 64105, NNTYPTAB 00457, RUTMXTAB 22507 */
    {63,
     3,
     {AGC_SCALE_POS6, AGC_SCALE_VEL2, AGC_SCALE_MIN_SEC},
     {AGC_DATA_RTGO, AGC_DATA_VIO, AGC_DATA_TTE},
     true,
     true},
    /* 64  Drag acceleration / Inertial velocity (VI) / Range to splash
     *     3 comp, dec only; NNADTAB 24110, NNTYPTAB 36460, RUTMXTAB 16505 */
    {64,
     3,
     {AGC_SCALE_DRAG_ACCEL, AGC_SCALE_VEL2, AGC_SCALE_POS6},
     {AGC_DATA_D, AGC_DATA_VMAGI, AGC_DATA_RANGE_TO_SPLASH},
     true,
     false},
    /* 65  Sampled AGC time (HRS,MIN,SEC) / (fetched in interrupt)
     *     3 comp, dec only; NNADTAB 24113, NNTYPTAB 00000, RUTMXTAB 20410 */
    {65,
     3,
     {AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS},
     {AGC_DATA_SAMPTIME, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
    /* 66  Command bank angle (beta) / Cross range error / Down range error
     *     3 comp, dec only; NNADTAB 24116, NNTYPTAB 37044, RUTMXTAB 16352 */
    {66,
     3,
     {AGC_SCALE_DP_DEG360, AGC_SCALE_POS8, AGC_SCALE_POS6},
     {AGC_DATA_ROLLC, AGC_DATA_XRNGERR, AGC_DATA_DNRNGERR},
     true,
     false},
    /* 67  Range to target / Present latitude / Present longitude
     *     3 comp, dec only; NNADTAB 24121, NNTYPTAB 10217, RUTMXTAB 24507 */
    {67,
     3,
     {AGC_SCALE_POS6, AGC_SCALE_DP_DEG360, AGC_SCALE_DP_DEG360},
     {AGC_DATA_RANGE_TO_TARGET, AGC_DATA_LAT, AGC_DATA_LONG},
     true,
     false},
    /* 68  Command bank angle (beta) / Inertial velocity (VI) / Alt rate change (RDOT)
     *     3 comp, dec only; NNADTAB 24124, NNTYPTAB 34444, RUTMXTAB 24512 */
    {68,
     3,
     {AGC_SCALE_DP_DEG360, AGC_SCALE_VEL2, AGC_SCALE_VEL_2VS},
     {AGC_DATA_ROLLC, AGC_DATA_VMAGI, AGC_DATA_RDOT},
     true,
     false},
    /* 69  Beta / DL / VL
     *     3 comp, dec only; NNADTAB 24127, NNTYPTAB 35004, RUTMXTAB 24252 */
    {69,
     3,
     {AGC_SCALE_DP_DEG360, AGC_SCALE_DRAG_ACCEL, AGC_SCALE_VEL_2VS},
     {AGC_DATA_ROLLC, AGC_DATA_Q7, AGC_DATA_VL},
     true,
     false},
    /* 70  Star code / Landmark data / Horizon data
     *     3 comp; NNADTAB 04132, NNTYPTAB 00000, RUTMXTAB 00000 */
    {70,
     3,
     {AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_OCTAL},
     {AGC_DATA_STARCODE, AGC_DATA_LANDMARK, AGC_DATA_HORIZON},
     false,
     false},
    /* 71  Star code / Landmark / Horizon
     *     3 comp; NNADTAB 04135, NNTYPTAB 00000, RUTMXTAB 0 */
    {71,
     3,
     {AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_OCTAL},
     {AGC_DATA_STARCODE, AGC_DATA_LANDMARK, AGC_DATA_HORIZON},
     false,
     false},
    /* 72  Delt ang / Delt alt / Search option
     *     3 comp, dec only; NNADTAB 24140, NNTYPTAB 00404, RUTMXTAB 06352 */
    {72,
     3,
     {AGC_SCALE_DP_DEG360, AGC_SCALE_POS4, AGC_SCALE_WHOLE},
     {AGC_DATA_THETZERO, AGC_DATA_DELHITE, AGC_DATA_OPTION2},
     true,
     false},
    /* 73  Altitude / Velocity / Flight path angle
     *     3 comp; NNADTAB 04143, NNTYPTAB 10450, RUTMXTAB 24507 */
    {73,
     3,
     {AGC_SCALE_POS4, AGC_SCALE_VEL2, AGC_SCALE_DP_DEG360},
     {AGC_DATA_GROUND_TRACK_DETERMINATION_ALT,
      AGC_DATA_GROUND_TRACK_DETERMINATION_VEL,
      AGC_DATA_GROUND_TRACK_DETERMINATION_GAM},
     false,
     false},
    /* 74  Command bank angle (beta) / Inertial velocity (VI) / Drag acceleration
     *     3 comp; NNADTAB 04146, NNTYPTAB 40444, RUTMXTAB 12512 */
    {74,
     3,
     {AGC_SCALE_DP_DEG360, AGC_SCALE_VEL2, AGC_SCALE_DRAG_ACCEL},
     {AGC_DATA_ROLLC, AGC_DATA_VMAGI, AGC_DATA_D},
     false,
     false},
    /* 75  Delta altitude CDH / Delta time (CDH-CSI or TPI-CDH) / Delta time (TPI-CDH or TPI-NOMTPI)
     *     3 comp, dec only, no load; NNADTAB 64151, NNTYPTAB 00010, RUTMXTAB 22447 */
    {75,
     3,
     {AGC_SCALE_POS4, AGC_SCALE_MIN_SEC, AGC_SCALE_MIN_SEC},
     {AGC_DATA_DIFFALT, AGC_DATA_T1TOT2, AGC_DATA_T2TOT3},
     true,
     true},
    /* 76  Spare */
    {76,
     0,
     {AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE},
     {AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE},
     false,
     false},
    /* 77  Spare */
    {77,
     0,
     {AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE},
     {AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE},
     false,
     false},
    /* 78  Spare */
    {78,
     0,
     {AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE},
     {AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE},
     false,
     false},
    /* 79  Spare */
    {79,
     0,
     {AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE},
     {AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE},
     false,
     false},
    /* 80  Time to ignition/cutoff / Vg / Delta V (accumulated)
     *     3 comp, dec only, no load; NNADTAB 64170, NNTYPTAB 22440, RUTMXTAB 24511 */
    {80,
     3,
     {AGC_SCALE_MIN_SEC, AGC_SCALE_VEL2, AGC_SCALE_VEL2},
     {AGC_DATA_TTOGO, AGC_DATA_VGDISP, AGC_DATA_DVTOTAL},
     true,
     true},
    /* 81  Delta V (LV)
     *     3 comp, dec only; NNADTAB 24173, NNTYPTAB 24512, RUTMXTAB 16347 */
    {81,
     3,
     {AGC_SCALE_VEL3, AGC_SCALE_VEL3, AGC_SCALE_VEL3},
     {AGC_DATA_DELVLVC, AGC_DATA_DELVLVC_P2, AGC_DATA_DELVLVC_P4},
     true,
     false},
    /* 82  Delta V (LV)
     *     3 comp, dec only; NNADTAB 24176, NNTYPTAB 24512, RUTMXTAB 16347 */
    {82,
     3,
     {AGC_SCALE_VEL3, AGC_SCALE_VEL3, AGC_SCALE_VEL3},
     {AGC_DATA_DELVLVC, AGC_DATA_DELVLVC_P2, AGC_DATA_DELVLVC_P4},
     true,
     false},
    /* 83  Delta V (body)
     *     3 comp, dec only; NNADTAB 24201, NNTYPTAB 24512, RUTMXTAB 16347 */
    {83,
     3,
     {AGC_SCALE_VEL3, AGC_SCALE_VEL3, AGC_SCALE_VEL3},
     {AGC_DATA_DELVIMU, AGC_DATA_DELVIMU_P2, AGC_DATA_DELVIMU_P4},
     true,
     false},
    /* 84  Delta V (other vehicle)
     *     3 comp, dec only; NNADTAB 24204, NNTYPTAB 24512, RUTMXTAB 16347 */
    {84,
     3,
     {AGC_SCALE_VEL3, AGC_SCALE_VEL3, AGC_SCALE_VEL3},
     {AGC_DATA_DELVOV, AGC_DATA_DELVOV_P2, AGC_DATA_DELVOV_P4},
     true,
     false},
    /* 85  Vg (body)
     *     3 comp, dec only; NNADTAB 24207, NNTYPTAB 24512, RUTMXTAB 16347 */
    {85,
     3,
     {AGC_SCALE_VEL3, AGC_SCALE_VEL3, AGC_SCALE_VEL3},
     {AGC_DATA_VGBODY, AGC_DATA_VGBODY_P2, AGC_DATA_VGBODY_P4},
     true,
     false},
    /* 86  Delta V (LV)
     *     3 comp, dec only; NNADTAB 24212, NNTYPTAB 22451, RUTMXTAB 24512 */
    {86,
     3,
     {AGC_SCALE_VEL2, AGC_SCALE_VEL2, AGC_SCALE_VEL2},
     {AGC_DATA_DELVLVC, AGC_DATA_DELVLVC_P2, AGC_DATA_DELVLVC_P4},
     true,
     false},
    /* 87  Mark data shaft / Trunion
     *     2 comp; NNADTAB 02215, NNTYPTAB 00102, RUTMXTAB 00302 */
    {87,
     2,
     {AGC_SCALE_CDU_DEG, AGC_SCALE_Y_OPTICS_DEG, AGC_SCALE_NONE},
     {AGC_DATA_MRKBUF1_P3, AGC_DATA_MRKBUF1_P5, AGC_DATA_NONE},
     false,
     false},
    /* 88  Half unit sun or planet vector
     *     3 comp, dec only; NNADTAB 24220, NNTYPTAB 00000, RUTMXTAB 32655 */
    {88,
     3,
     {AGC_SCALE_DP_FRACTION, AGC_SCALE_DP_FRACTION, AGC_SCALE_DP_FRACTION},
     {AGC_DATA_STARSAV3, AGC_DATA_STARSAV3_P2, AGC_DATA_STARSAV3_P4},
     true,
     false},
    /* 89  Landmark latitude / Longitude/2 / Altitude
     *     3 comp, dec only; NNADTAB 24223, NNTYPTAB 16143, RUTMXTAB 10347 */
    {89,
     3,
     {AGC_SCALE_DP_DEG90, AGC_SCALE_DP_DEG90, AGC_SCALE_POS5},
     {AGC_DATA_LANDLAT, AGC_DATA_LANDLONG, AGC_DATA_LANDALT},
     true,
     false},
    /* 90  Y / Y dot / Psi
     *     3 comp, dec only; NNADTAB 24226, NNTYPTAB 10507, RUTMXTAB 24344 */
    {90,
     3,
     {AGC_SCALE_POS5, AGC_SCALE_VEL3, AGC_SCALE_DP_DEG360},
     {AGC_DATA_RANGE, AGC_DATA_RRATE, AGC_DATA_RTHETA},
     true,
     false},
    /* 91  OCDU angles shaft / Trunion
     *     2 comp; NNADTAB 02231, NNTYPTAB 00102, RUTMXTAB 00302 */
    {91,
     2,
     {AGC_SCALE_CDU_DEG, AGC_SCALE_Y_OPTICS_DEG, AGC_SCALE_NONE},
     {AGC_DATA_CDUS, AGC_DATA_CDUT, AGC_DATA_NONE},
     false,
     false},
    /* 92  New optics angles shaft / Trunion
     *     2 comp; NNADTAB 02234, NNTYPTAB 00102, RUTMXTAB 00302 */
    {92,
     2,
     {AGC_SCALE_CDU_DEG, AGC_SCALE_Y_OPTICS_DEG, AGC_SCALE_NONE},
     {AGC_DATA_SAC, AGC_DATA_PAC, AGC_DATA_NONE},
     false,
     false},
    /* 93  Delta gyro angles
     *     3 comp; NNADTAB 04237, NNTYPTAB 06143, RUTMXTAB 16347 */
    {93,
     3,
     {AGC_SCALE_DP_DEG90, AGC_SCALE_DP_DEG90, AGC_SCALE_DP_DEG90},
     {AGC_DATA_OGC, AGC_DATA_OGC_P2, AGC_DATA_OGC_P4},
     false,
     false},
    /* 94  New optics angles shaft / Trunnion
     *     2 comp; NNADTAB 02242, NNTYPTAB 00102, RUTMXTAB 00302 */
    {94,
     2,
     {AGC_SCALE_CDU_DEG, AGC_SCALE_Y_OPTICS_DEG, AGC_SCALE_NONE},
     {AGC_DATA_MRKBUF1_P3, AGC_DATA_MRKBUF1_P5, AGC_DATA_NONE},
     false,
     false},
    /* 95  Preferred attitude ICDU angles
     *     3 comp; NNADTAB 04245, NNTYPTAB 04102, RUTMXTAB 04102 */
    {95,
     3,
     {AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG},
     {AGC_DATA_PRAXIS, AGC_DATA_PRAXIS_P1, AGC_DATA_PRAXIS_P2},
     false,
     false},
    /* 96  +X-axis attitude ICDU angles
     *     3 comp; NNADTAB 04250, NNTYPTAB 04102, RUTMXTAB 04102 */
    {96,
     3,
     {AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG},
     {AGC_DATA_CPHIX, AGC_DATA_CPHIX_P1, AGC_DATA_CPHIX_P2},
     false,
     false},
    /* 97  System test inputs
     *     3 comp; NNADTAB 04253, NNTYPTAB 00000, RUTMXTAB 06143 */
    {97,
     3,
     {AGC_SCALE_WHOLE, AGC_SCALE_WHOLE, AGC_SCALE_WHOLE},
     {AGC_DATA_DSPTEM1, AGC_DATA_DSPTEM1_P1, AGC_DATA_DSPTEM1_P2},
     false,
     false},
    /* 98  System test results
     *     3 comp; NNADTAB 04256, NNTYPTAB 00000, RUTMXTAB 06043 */
    {98,
     3,
     {AGC_SCALE_WHOLE, AGC_SCALE_FRACTION, AGC_SCALE_WHOLE},
     {AGC_DATA_DSPTEM2, AGC_DATA_DSPTEM2_P1, AGC_DATA_DSPTEM2_P2},
     false,
     false},
    /* 99  RMS in position / RMS in velocity / RMS option
     *     3 comp, dec only; NNADTAB 24261, NNTYPTAB 01162, RUTMXTAB 06247 */
    {99,
     3,
     {AGC_SCALE_POS9, AGC_SCALE_VEL4, AGC_SCALE_WHOLE},
     {AGC_DATA_WWPOS, AGC_DATA_WWVEL, AGC_DATA_WWOPT},
     true,
     false},
};
