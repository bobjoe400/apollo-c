/*
 * noun_tables_lm.c -- PINBALL noun tables for the Lunar Module (Luminary 099).
 *
 * Translated from Luminary099/PINBALL_NOUN_TABLES.agc (pp. 301-319). NNADTAB / NNTYPTAB /
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

const agc_noun_t agc_noun_table_lm[AGC_NOUN_COUNT] = {
    /* 00  Not in use */
    { 0, 0, { AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE },
         { AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE }, false, false },
    /* 01  Specify machine address (fractional)
     *     3 comp; NNADTAB 40000, NNTYPTAB 04040 */
    { 1, 3,
      { AGC_SCALE_FRACTION, AGC_SCALE_FRACTION, AGC_SCALE_FRACTION },
      { AGC_DATA_SPECIFY_ADDR, AGC_DATA_SPECIFY_ADDR, AGC_DATA_SPECIFY_ADDR },
      false, false },
    /* 02  Specify machine address (whole)
     *     3 comp; NNADTAB 40000, NNTYPTAB 04140 */
    { 2, 3,
      { AGC_SCALE_WHOLE, AGC_SCALE_WHOLE, AGC_SCALE_WHOLE },
      { AGC_DATA_SPECIFY_ADDR, AGC_DATA_SPECIFY_ADDR, AGC_DATA_SPECIFY_ADDR },
      false, false },
    /* 03  Specify machine address (degrees)
     *     3 comp; NNADTAB 40000, NNTYPTAB 04102 */
    { 3, 3,
      { AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG },
      { AGC_DATA_SPECIFY_ADDR, AGC_DATA_SPECIFY_ADDR, AGC_DATA_SPECIFY_ADDR },
      false, false },
    /* 04  Angular error/difference
     *     1 comp; NNADTAB ECADR DSPTEM1, NNTYPTAB 00504 */
    { 4, 1,
      { AGC_SCALE_DP_DEG360, AGC_SCALE_NONE, AGC_SCALE_NONE },
      { AGC_DATA_DSPTEM1_DP, AGC_DATA_NONE, AGC_DATA_NONE },
      false, false },
    /* 05  Angular error/difference
     *     1 comp; NNADTAB ECADR DSPTEM1, NNTYPTAB 00504 */
    { 5, 1,
      { AGC_SCALE_DP_DEG360, AGC_SCALE_NONE, AGC_SCALE_NONE },
      { AGC_DATA_DSPTEM1_DP, AGC_DATA_NONE, AGC_DATA_NONE },
      false, false },
    /* 06  Option code
     *     3 comp; NNADTAB ECADR OPTION1, NNTYPTAB 04000 */
    { 6, 3,
      { AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_OCTAL },
      { AGC_DATA_OPTION1, AGC_DATA_OPTION2, AGC_DATA_OPTION3 },
      false, false },
    /* 07  ECADR of word to be modified / Ones for bits to be modified / 1 to set or 0 to reset selected bits
     *     3 comp; NNADTAB ECADR XREG, NNTYPTAB 04000 */
    { 7, 3,
      { AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_OCTAL },
      { AGC_DATA_XREG, AGC_DATA_YREG, AGC_DATA_ZREG },
      false, false },
    /* 08  Alarm data
     *     3 comp; NNADTAB ECADR ALMCADR, NNTYPTAB 04000 */
    { 8, 3,
      { AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_OCTAL },
      { AGC_DATA_ALMCADR, AGC_DATA_ALMCADR_P1, AGC_DATA_ALMCADR_P2 },
      false, false },
    /* 09  Alarm codes
     *     3 comp; NNADTAB ECADR FAILREG, NNTYPTAB 04000 */
    { 9, 3,
      { AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_OCTAL },
      { AGC_DATA_FAILREG, AGC_DATA_FAILREG_P1, AGC_DATA_FAILREG_P2 },
      false, false },
    /* 10  Channel to be specified
     *     1 comp; NNADTAB 77776, NNTYPTAB 00000 */
    { 10, 1,
      { AGC_SCALE_OCTAL, AGC_SCALE_NONE, AGC_SCALE_NONE },
      { AGC_DATA_SPECIFY_CHANNEL, AGC_DATA_NONE, AGC_DATA_NONE },
      false, false },
    /* 11  TIG of CSI (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR TCSI, NNTYPTAB 24400 */
    { 11, 3,
      { AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS },
      { AGC_DATA_TCSI, AGC_DATA_NONE, AGC_DATA_NONE },
      true, false },
    /* 12  Option code / (used by extended verbs only)
     *     2 comp; NNADTAB ECADR OPTIONX, NNTYPTAB 02000 */
    { 12, 2,
      { AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_NONE },
      { AGC_DATA_OPTIONX, AGC_DATA_OPTIONX_P1, AGC_DATA_NONE },
      false, false },
    /* 13  TIG of CDH (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR TCDH, NNTYPTAB 24400 */
    { 13, 3,
      { AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS },
      { AGC_DATA_TCDH, AGC_DATA_NONE, AGC_DATA_NONE },
      true, false },
    /* 14  Checklist / (used by extended verbs only)
     *     3 comp; NNADTAB ECADR DSPTEMX, NNTYPTAB 04140 */
    { 14, 3,
      { AGC_SCALE_WHOLE, AGC_SCALE_WHOLE, AGC_SCALE_WHOLE },
      { AGC_DATA_DSPTEMX, AGC_DATA_DSPTEMX_P1, AGC_DATA_DSPTEMX_P2 },
      false, false },
    /* 15  Increment machine address
     *     1 comp; NNADTAB 77777, NNTYPTAB 00000 */
    { 15, 1,
      { AGC_SCALE_OCTAL, AGC_SCALE_NONE, AGC_SCALE_NONE },
      { AGC_DATA_INCREMENT_ADDR, AGC_DATA_NONE, AGC_DATA_NONE },
      false, false },
    /* 16  Time of event (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR DSPTEMX, NNTYPTAB 24400 */
    { 16, 3,
      { AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS },
      { AGC_DATA_DSPTEMX_DP, AGC_DATA_NONE, AGC_DATA_NONE },
      true, false },
    /* 17  Spare */
    { 17, 0, { AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE },
         { AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE }, false, false },
    /* 18  Auto maneuver ball angles
     *     3 comp; NNADTAB ECADR FDAIX, NNTYPTAB 04102 */
    { 18, 3,
      { AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG },
      { AGC_DATA_FDAIX, AGC_DATA_FDAIY, AGC_DATA_FDAIZ },
      false, false },
    /* 19  Spare */
    { 19, 0, { AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE },
         { AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE }, false, false },
    /* 20  ICDU angles
     *     3 comp; NNADTAB ECADR CDUX, NNTYPTAB 04102 */
    { 20, 3,
      { AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG },
      { AGC_DATA_CDUX, AGC_DATA_CDUY, AGC_DATA_CDUZ },
      false, false },
    /* 21  PIPAS
     *     3 comp; NNADTAB ECADR PIPAX, NNTYPTAB 04140 */
    { 21, 3,
      { AGC_SCALE_WHOLE, AGC_SCALE_WHOLE, AGC_SCALE_WHOLE },
      { AGC_DATA_PIPAX, AGC_DATA_PIPAY, AGC_DATA_PIPAZ },
      false, false },
    /* 22  New ICDU angles
     *     3 comp; NNADTAB ECADR THETAD, NNTYPTAB 04102 */
    { 22, 3,
      { AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG },
      { AGC_DATA_THETAD, AGC_DATA_THETAD_P1, AGC_DATA_THETAD_P2 },
      false, false },
    /* 23  Spare */
    { 23, 0, { AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE },
         { AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE }, false, false },
    /* 24  Delta time for AGC clock (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR DSPTEM2 +1, NNTYPTAB 24400 */
    { 24, 3,
      { AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS },
      { AGC_DATA_DSPTEM2_P1_DP, AGC_DATA_NONE, AGC_DATA_NONE },
      true, false },
    /* 25  Checklist / (used wtih please perform only)
     *     3 comp; NNADTAB ECADR DSPTEM1, NNTYPTAB 04140 */
    { 25, 3,
      { AGC_SCALE_WHOLE, AGC_SCALE_WHOLE, AGC_SCALE_WHOLE },
      { AGC_DATA_DSPTEM1, AGC_DATA_DSPTEM1_P1, AGC_DATA_DSPTEM1_P2 },
      false, false },
    /* 26  Prio/delay, ADRES, BBCON
     *     3 comp; NNADTAB ECADR DSPTEM1, NNTYPTAB 04000 */
    { 26, 3,
      { AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_OCTAL },
      { AGC_DATA_DSPTEM1, AGC_DATA_DSPTEM1_P1, AGC_DATA_DSPTEM1_P2 },
      false, false },
    /* 27  Self test on/off switch
     *     1 comp; NNADTAB ECADR SMODE, NNTYPTAB 00140 */
    { 27, 1,
      { AGC_SCALE_WHOLE, AGC_SCALE_NONE, AGC_SCALE_NONE },
      { AGC_DATA_SMODE, AGC_DATA_NONE, AGC_DATA_NONE },
      false, false },
    /* 28  Spare */
    { 28, 0, { AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE },
         { AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE }, false, false },
    /* 29  Spare */
    { 29, 0, { AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE },
         { AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE }, false, false },
    /* 30  Spare */
    { 30, 0, { AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE },
         { AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE }, false, false },
    /* 31  Spare */
    { 31, 0, { AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE },
         { AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE }, false, false },
    /* 32  Time to perigee (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR -TPER, NNTYPTAB 24400 */
    { 32, 3,
      { AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS },
      { AGC_DATA_NEG_TPER, AGC_DATA_NONE, AGC_DATA_NONE },
      true, false },
    /* 33  Time of ignition (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR TIG, NNTYPTAB 24400 */
    { 33, 3,
      { AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS },
      { AGC_DATA_TIG, AGC_DATA_NONE, AGC_DATA_NONE },
      true, false },
    /* 34  Time of event (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR DSPTEM1, NNTYPTAB 24400 */
    { 34, 3,
      { AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS },
      { AGC_DATA_DSPTEM1_DP, AGC_DATA_NONE, AGC_DATA_NONE },
      true, false },
    /* 35  Time to go to event (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR TTOGO, NNTYPTAB 24400 */
    { 35, 3,
      { AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS },
      { AGC_DATA_TTOGO, AGC_DATA_NONE, AGC_DATA_NONE },
      true, false },
    /* 36  Time of AGC clock (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR TIME2, NNTYPTAB 24400 */
    { 36, 3,
      { AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS },
      { AGC_DATA_TIME2, AGC_DATA_NONE, AGC_DATA_NONE },
      true, false },
    /* 37  TIG of TPI (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR TTPI, NNTYPTAB 24400 */
    { 37, 3,
      { AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS },
      { AGC_DATA_TTPI, AGC_DATA_NONE, AGC_DATA_NONE },
      true, false },
    /* 38  Time of state being integrated
     *     3 comp, dec only; NNADTAB ECADR TET, NNTYPTAB 24400 */
    { 38, 3,
      { AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS },
      { AGC_DATA_TET, AGC_DATA_NONE, AGC_DATA_NONE },
      true, false },
    /* 39  Spare */
    { 39, 0, { AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE },
         { AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE }, false, false },
    /* 40  Time to ignition/cutoff / Vg / Delta V (accumulated)
     *     3 comp, dec only, no load; NNADTAB 64000, NNTYPTAB 24500, RUTMXTAB 16351 */
    { 40, 3,
      { AGC_SCALE_MIN_SEC, AGC_SCALE_VEL3, AGC_SCALE_VEL3 },
      { AGC_DATA_TTOGO, AGC_DATA_VGDISP, AGC_DATA_DVTOTAL },
      true, true },
    /* 41  Target azimuth / Elevation
     *     2 comp; NNADTAB 02003, NNTYPTAB 00542, RUTMXTAB 00142 */
    { 41, 2,
      { AGC_SCALE_CDU_DEG, AGC_SCALE_ELEV_DEG, AGC_SCALE_NONE },
      { AGC_DATA_DSPTEM1, AGC_DATA_DSPTEM1_P1, AGC_DATA_NONE },
      false, false },
    /* 42  Apogee / Perigee / Delta V (required)
     *     3 comp, dec only; NNADTAB 24006, NNTYPTAB 24410, RUTMXTAB 16347 */
    { 42, 3,
      { AGC_SCALE_POS4, AGC_SCALE_POS4, AGC_SCALE_VEL3 },
      { AGC_DATA_HAPO, AGC_DATA_HPER, AGC_DATA_VGDISP },
      true, false },
    /* 43  Latitude / Longitude / Altitude
     *     3 comp, dec only; NNADTAB 24011, NNTYPTAB 20204, RUTMXTAB 16512 */
    { 43, 3,
      { AGC_SCALE_DP_DEG360, AGC_SCALE_DP_DEG360, AGC_SCALE_POS4 },
      { AGC_DATA_LAT, AGC_DATA_LONG, AGC_DATA_ALT },
      true, false },
    /* 44  Apogee / Perigee / TFF
     *     3 comp, dec only, no load; NNADTAB 64014, NNTYPTAB 00410, RUTMXTAB 22347 */
    { 44, 3,
      { AGC_SCALE_POS4, AGC_SCALE_POS4, AGC_SCALE_MIN_SEC },
      { AGC_DATA_HAPOX, AGC_DATA_HPERX, AGC_DATA_TFF },
      true, true },
    /* 45  Marks / TTI of next burn / MGA
     *     3 comp, dec only, no load; NNADTAB 64017, NNTYPTAB 10000, RUTMXTAB 24443 */
    { 45, 3,
      { AGC_SCALE_WHOLE, AGC_SCALE_MIN_SEC, AGC_SCALE_DP_DEG360 },
      { AGC_DATA_TRKMKCNT, AGC_DATA_TTOGO, AGC_DATA_PLUS_MGA },
      true, true },
    /* 46  Autopilot configuration
     *     1 comp; NNADTAB 00022, NNTYPTAB 00000, RUTMXTAB 00000 */
    { 46, 1,
      { AGC_SCALE_OCTAL, AGC_SCALE_NONE, AGC_SCALE_NONE },
      { AGC_DATA_DAPDATR1, AGC_DATA_NONE, AGC_DATA_NONE },
      false, false },
    /* 47  LEM weight / CSM weight
     *     2 comp, dec only; NNADTAB 22025, NNTYPTAB 00306, RUTMXTAB 00553 */
    { 47, 2,
      { AGC_SCALE_WEIGHT2, AGC_SCALE_WEIGHT2, AGC_SCALE_NONE },
      { AGC_DATA_LEMMASS, AGC_DATA_CSMMASS, AGC_DATA_NONE },
      true, false },
    /* 48  Gimbal pitch trim / Gimbal roll trim
     *     2 comp, dec only; NNADTAB 22030, NNTYPTAB 01367, RUTMXTAB 00143 */
    { 48, 2,
      { AGC_SCALE_TRIM_DEG2, AGC_SCALE_TRIM_DEG2, AGC_SCALE_NONE },
      { AGC_DATA_PITTIME, AGC_DATA_ROLLTIME, AGC_DATA_NONE },
      true, false },
    /* 49  Delta R / Delta V / Radar data source code
     *     3 comp, dec only; NNADTAB 24033, NNTYPTAB 00510, RUTMXTAB 06347 */
    { 49, 3,
      { AGC_SCALE_POS4, AGC_SCALE_VEL3, AGC_SCALE_WHOLE },
      { AGC_DATA_RADAR_DELTA_R, AGC_DATA_RADAR_DELTA_V, AGC_DATA_WHCHREAD },
      true, false },
    /* 50  Spare */
    { 50, 0, { AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE },
         { AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE }, false, false },
    /* 51  S-band antenna pitch / Yaw
     *     2 comp, dec only; NNADTAB 22041, NNTYPTAB 00204, RUTMXTAB 00512 */
    { 51, 2,
      { AGC_SCALE_DP_DEG360, AGC_SCALE_DP_DEG360, AGC_SCALE_NONE },
      { AGC_DATA_ALPHASB, AGC_DATA_BETASB, AGC_DATA_NONE },
      true, false },
    /* 52  Central angle of active vehicle
     *     1 comp; NNADTAB 00044, NNTYPTAB 00004, RUTMXTAB 00012 */
    { 52, 1,
      { AGC_SCALE_DP_DEG360, AGC_SCALE_NONE, AGC_SCALE_NONE },
      { AGC_DATA_ACTCENT, AGC_DATA_NONE, AGC_DATA_NONE },
      false, false },
    /* 53  Spare */
    { 53, 0, { AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE },
         { AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE }, false, false },
    /* 54  Range / Range rate / Theta
     *     3 comp, dec only; NNADTAB 24052, NNTYPTAB 10507, RUTMXTAB 24344 */
    { 54, 3,
      { AGC_SCALE_POS5, AGC_SCALE_VEL3, AGC_SCALE_DP_DEG360 },
      { AGC_DATA_RANGE, AGC_DATA_RRATE, AGC_DATA_RTHETA },
      true, false },
    /* 55  No. of apsidal crossings / Elevation angle / Central angle
     *     3 comp, dec only; NNADTAB 24055, NNTYPTAB 10200, RUTMXTAB 24503 */
    { 55, 3,
      { AGC_SCALE_WHOLE, AGC_SCALE_DP_DEG360, AGC_SCALE_DP_DEG360 },
      { AGC_DATA_NN, AGC_DATA_ELEV, AGC_DATA_CENTANG },
      true, false },
    /* 56  RR LOS azimuth / Elevation
     *     2 comp; NNADTAB 02060, NNTYPTAB 00204, RUTMXTAB 00512 */
    { 56, 2,
      { AGC_SCALE_DP_DEG360, AGC_SCALE_DP_DEG360, AGC_SCALE_NONE },
      { AGC_DATA_RR_AZ, AGC_DATA_RR_ELEV, AGC_DATA_NONE },
      false, false },
    /* 57  Delta R
     *     1 comp, dec only; NNADTAB 20063, NNTYPTAB 00010, RUTMXTAB 00007 */
    { 57, 1,
      { AGC_SCALE_POS4, AGC_SCALE_NONE, AGC_SCALE_NONE },
      { AGC_DATA_DELTAR, AGC_DATA_NONE, AGC_DATA_NONE },
      true, false },
    /* 58  Perigee alt / Delta V TPI / Delta V TPF
     *     3 comp, dec only; NNADTAB 24066, NNTYPTAB 24510, RUTMXTAB 16347 */
    { 58, 3,
      { AGC_SCALE_POS4, AGC_SCALE_VEL3, AGC_SCALE_VEL3 },
      { AGC_DATA_POSTTPI, AGC_DATA_DELVTPI, AGC_DATA_DELVTPF },
      true, false },
    /* 59  Delta velocity LOS
     *     3 comp, dec only; NNADTAB 24071, NNTYPTAB 24512, RUTMXTAB 16347 */
    { 59, 3,
      { AGC_SCALE_VEL3, AGC_SCALE_VEL3, AGC_SCALE_VEL3 },
      { AGC_DATA_DVLOS, AGC_DATA_DVLOS_P2, AGC_DATA_DVLOS_P4 },
      true, false },
    /* 60  Horizontal velocity / Altitude rate / Computed altitude
     *     3 comp, dec only; NNADTAB 24074, NNTYPTAB 60512, RUTMXTAB 10347 */
    { 60, 3,
      { AGC_SCALE_VEL3, AGC_SCALE_VEL3, AGC_SCALE_COMP_ALT },
      { AGC_DATA_VHORIZ, AGC_DATA_HDOTDISP, AGC_DATA_HCALC },
      true, false },
    /* 61  Time to go in braking phase / Time to ignition / Cross range distance
     *     3 comp, dec only, no load; NNADTAB 64077, NNTYPTAB 54000, RUTMXTAB 24451 */
    { 61, 3,
      { AGC_SCALE_MIN_SEC, AGC_SCALE_MIN_SEC, AGC_SCALE_POS7 },
      { AGC_DATA_TTFDISP, AGC_DATA_TTOGO, AGC_DATA_OUTOFPLN },
      true, true },
    /* 62  Absolute value of velocity / Time to ignition / Delta V (accumulated)
     *     3 comp, dec only, no load; NNADTAB 64102, NNTYPTAB 24012, RUTMXTAB 16447 */
    { 62, 3,
      { AGC_SCALE_VEL3, AGC_SCALE_MIN_SEC, AGC_SCALE_VEL3 },
      { AGC_DATA_ABVEL, AGC_DATA_TTOGO, AGC_DATA_DVTOTAL },
      true, true },
    /* 63  Absolute value of velocity / Altitude rate / Computed altitude
     *     3 comp, dec only; NNADTAB 24105, NNTYPTAB 60512, RUTMXTAB 10347 */
    { 63, 3,
      { AGC_SCALE_VEL3, AGC_SCALE_VEL3, AGC_SCALE_COMP_ALT },
      { AGC_DATA_ABVEL, AGC_DATA_HDOTDISP, AGC_DATA_HCALC1 },
      true, false },
    /* 64  Time left for redesignation -- LPD angle / Altitude rate / Computed altitude
     *     3 comp, dec only, no load; NNADTAB 64110, NNTYPTAB 60500, RUTMXTAB 10354 */
    { 64, 3,
      { AGC_SCALE_TWO_INT, AGC_SCALE_VEL3, AGC_SCALE_COMP_ALT },
      { AGC_DATA_FUNNYDSP, AGC_DATA_HDOTDISP, AGC_DATA_HCALC },
      true, true },
    /* 65  Sampled AGC time (HRS,MIN,SEC) / (fetched in interrupt)
     *     3 comp, dec only; NNADTAB 24113, NNTYPTAB 00000, RUTMXTAB 20410 */
    { 65, 3,
      { AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS },
      { AGC_DATA_SAMPTIME, AGC_DATA_NONE, AGC_DATA_NONE },
      true, false },
    /* 66  LR range / Position
     *     2 comp, dec only, no load; NNADTAB 62116, NNTYPTAB 00016, RUTMXTAB 00304 */
    { 66, 2,
      { AGC_SCALE_LR_ALT, AGC_SCALE_LR_POSITION, AGC_SCALE_NONE },
      { AGC_DATA_RSTACK_P6, AGC_DATA_CHANNEL33, AGC_DATA_NONE },
      true, true },
    /* 67  LRVX / LRVY / LRVZ
     *     3 comp; NNADTAB 04121, NNTYPTAB 53223, RUTMXTAB 10204 */
    { 67, 3,
      { AGC_SCALE_LR_VELX, AGC_SCALE_LR_VELY, AGC_SCALE_LR_VELZ },
      { AGC_DATA_RSTACK, AGC_DATA_RSTACK_P2, AGC_DATA_RSTACK_P4 },
      false, false },
    /* 68  Slant range to landing sight / Time to go in braking phase / LR altitude -- computed altitude
     *     3 comp, dec only, no load; NNADTAB 64124, NNTYPTAB 60026, RUTMXTAB 10452 */
    { 68, 3,
      { AGC_SCALE_POS7, AGC_SCALE_MIN_SEC, AGC_SCALE_COMP_ALT },
      { AGC_DATA_RANGEDSP, AGC_DATA_TTFDISP, AGC_DATA_DELTAH },
      true, true },
    /* 69  Spare */
    { 69, 0, { AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE },
         { AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE }, false, false },
    /* 70  AOT detent code/star code
     *     3 comp; NNADTAB 04132, NNTYPTAB 00000, RUTMXTAB 0 */
    { 70, 3,
      { AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_OCTAL },
      { AGC_DATA_AOTCODE, AGC_DATA_AOTCODE_P1, AGC_DATA_AOTCODE_P2 },
      false, false },
    /* 71  AOT detent code/star code
     *     3 comp; NNADTAB 04135, NNTYPTAB 00000, RUTMXTAB 0 */
    { 71, 3,
      { AGC_SCALE_OCTAL, AGC_SCALE_OCTAL, AGC_SCALE_OCTAL },
      { AGC_DATA_AOTCODE, AGC_DATA_AOTCODE_P1, AGC_DATA_AOTCODE_P2 },
      false, false },
    /* 72  RR 360 -- trunnion angle / Shaft angle
     *     2 comp; NNADTAB 02140, NNTYPTAB 00102, RUTMXTAB 00115 */
    { 72, 2,
      { AGC_SCALE_CDU360, AGC_SCALE_CDU_DEG, AGC_SCALE_NONE },
      { AGC_DATA_CDUT, AGC_DATA_CDUS, AGC_DATA_NONE },
      false, false },
    /* 73  New RR 360 -- trunnion angle / Shaft angle
     *     2 comp; NNADTAB 02143, NNTYPTAB 00102, RUTMXTAB 00115 */
    { 73, 2,
      { AGC_SCALE_CDU360, AGC_SCALE_CDU_DEG, AGC_SCALE_NONE },
      { AGC_DATA_TANG, AGC_DATA_TANG_P1, AGC_DATA_NONE },
      false, false },
    /* 74  Time to ignition / Yawafter vehicle rise / Pitch after vehicle rise
     *     3 comp, dec only, no load; NNADTAB 64146, NNTYPTAB 10200, RUTMXTAB 24511 */
    { 74, 3,
      { AGC_SCALE_MIN_SEC, AGC_SCALE_DP_DEG360, AGC_SCALE_DP_DEG360 },
      { AGC_DATA_TTOGO, AGC_DATA_YAW, AGC_DATA_PITCH },
      true, true },
    /* 75  Delta altitude CDH / Delta time (CDH-CSI or TPI-CDH) / Delta time (TPI-CDH or TPI-NOMTPI)
     *     3 comp, dec only, no load; NNADTAB 64151, NNTYPTAB 00010, RUTMXTAB 22447 */
    { 75, 3,
      { AGC_SCALE_POS4, AGC_SCALE_MIN_SEC, AGC_SCALE_MIN_SEC },
      { AGC_DATA_DIFFALT, AGC_DATA_T1TOT2, AGC_DATA_T2TOT3 },
      true, true },
    /* 76  Desired horizontal velocity / Desired radial velocity / Cross-range distance
     *     3 comp, dec only; NNADTAB 24154, NNTYPTAB 20512, RUTMXTAB 16347 */
    { 76, 3,
      { AGC_SCALE_VEL3, AGC_SCALE_VEL3, AGC_SCALE_POS4 },
      { AGC_DATA_ZDOTD, AGC_DATA_RDOTD, AGC_DATA_XRANGE },
      true, false },
    /* 77  Time to engine cutoff / Velocity normal to CSM plane
     *     2 comp, dec only, no load; NNADTAB 62157, NNTYPTAB 00500, RUTMXTAB 00351 */
    { 77, 2,
      { AGC_SCALE_MIN_SEC, AGC_SCALE_VEL3, AGC_SCALE_NONE },
      { AGC_DATA_TTOGO, AGC_DATA_YDOT, AGC_DATA_NONE },
      true, true },
    /* 78  RR range / Range rate
     *     2 comp; NNADTAB 02162, NNTYPTAB 00654, RUTMXTAB 00204 */
    { 78, 2,
      { AGC_SCALE_RR_RANGE, AGC_SCALE_RR_RATE, AGC_SCALE_NONE },
      { AGC_DATA_RSTACK, AGC_DATA_RSTACK_P2, AGC_DATA_NONE },
      false, false },
    /* 79  Cursor angle / Spiral angle / Position code
     *     3 comp, dec only; NNADTAB 24165, NNTYPTAB 00102, RUTMXTAB 06102 */
    { 79, 3,
      { AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG, AGC_SCALE_WHOLE },
      { AGC_DATA_CURSOR, AGC_DATA_SPIRAL, AGC_DATA_POSCODE },
      true, false },
    /* 80  Data indicator / Omega
     *     2 comp; NNADTAB 02170, NNTYPTAB 00200, RUTMXTAB 00503 */
    { 80, 2,
      { AGC_SCALE_WHOLE, AGC_SCALE_DP_DEG360, AGC_SCALE_NONE },
      { AGC_DATA_DATAGOOD, AGC_DATA_OMEGAD, AGC_DATA_NONE },
      false, false },
    /* 81  Delta V (LV)
     *     3 comp, dec only; NNADTAB 24173, NNTYPTAB 24512, RUTMXTAB 16347 */
    { 81, 3,
      { AGC_SCALE_VEL3, AGC_SCALE_VEL3, AGC_SCALE_VEL3 },
      { AGC_DATA_DELVLVC, AGC_DATA_DELVLVC_P2, AGC_DATA_DELVLVC_P4 },
      true, false },
    /* 82  Delta V (LV)
     *     3 comp, dec only; NNADTAB 24176, NNTYPTAB 24512, RUTMXTAB 16347 */
    { 82, 3,
      { AGC_SCALE_VEL3, AGC_SCALE_VEL3, AGC_SCALE_VEL3 },
      { AGC_DATA_DELVLVC, AGC_DATA_DELVLVC_P2, AGC_DATA_DELVLVC_P4 },
      true, false },
    /* 83  Delta V (body)
     *     3 comp, dec only; NNADTAB 24201, NNTYPTAB 24512, RUTMXTAB 16347 */
    { 83, 3,
      { AGC_SCALE_VEL3, AGC_SCALE_VEL3, AGC_SCALE_VEL3 },
      { AGC_DATA_DELVIMU, AGC_DATA_DELVIMU_P2, AGC_DATA_DELVIMU_P4 },
      true, false },
    /* 84  Delta V (other vehicle)
     *     3 comp, dec only; NNADTAB 24204, NNTYPTAB 24512, RUTMXTAB 16347 */
    { 84, 3,
      { AGC_SCALE_VEL3, AGC_SCALE_VEL3, AGC_SCALE_VEL3 },
      { AGC_DATA_DELVOV, AGC_DATA_DELVOV_P2, AGC_DATA_DELVOV_P4 },
      true, false },
    /* 85  Vg (body)
     *     3 comp, dec only; NNADTAB 24207, NNTYPTAB 24512, RUTMXTAB 16347 */
    { 85, 3,
      { AGC_SCALE_VEL3, AGC_SCALE_VEL3, AGC_SCALE_VEL3 },
      { AGC_DATA_VGBODY, AGC_DATA_VGBODY_P2, AGC_DATA_VGBODY_P4 },
      true, false },
    /* 86  Vg (LV)
     *     3 comp, dec only; NNADTAB 24212, NNTYPTAB 24512, RUTMXTAB 16347 */
    { 86, 3,
      { AGC_SCALE_VEL3, AGC_SCALE_VEL3, AGC_SCALE_VEL3 },
      { AGC_DATA_DELVLVC, AGC_DATA_DELVLVC_P2, AGC_DATA_DELVLVC_P4 },
      true, false },
    /* 87  Backup optics LOS azimuth / Elevation
     *     2 comp; NNADTAB 02215, NNTYPTAB 00102, RUTMXTAB 00102 */
    { 87, 2,
      { AGC_SCALE_CDU_DEG, AGC_SCALE_CDU_DEG, AGC_SCALE_NONE },
      { AGC_DATA_AZ, AGC_DATA_EL, AGC_DATA_NONE },
      false, false },
    /* 88  Half unit sun or planet vector
     *     3 comp, dec only; NNADTAB 24220, NNTYPTAB 00000, RUTMXTAB 02041 */
    { 88, 3,
      { AGC_SCALE_FRACTION, AGC_SCALE_FRACTION, AGC_SCALE_FRACTION },
      { AGC_DATA_STARAD, AGC_DATA_STARAD_P2, AGC_DATA_STARAD_P4 },
      true, false },
    /* 89  Landmark latitude / Longitude/2 / Altitude
     *     3 comp, dec only; NNADTAB 24223, NNTYPTAB 16143, RUTMXTAB 10347 */
    { 89, 3,
      { AGC_SCALE_DP_DEG90, AGC_SCALE_DP_DEG90, AGC_SCALE_POS5 },
      { AGC_DATA_LANDLAT, AGC_DATA_LANDLONG, AGC_DATA_LANDALT },
      true, false },
    /* 90  Y / Y dot / Psi
     *     3 comp, dec only; NNADTAB 24226, NNTYPTAB 10507, RUTMXTAB 24344 */
    { 90, 3,
      { AGC_SCALE_POS5, AGC_SCALE_VEL3, AGC_SCALE_DP_DEG360 },
      { AGC_DATA_RANGE, AGC_DATA_RRATE, AGC_DATA_RTHETA },
      true, false },
    /* 91  Altitude / Velocity / Flight path angle
     *     3 comp; NNADTAB 04231, NNTYPTAB 10450, RUTMXTAB 24507 */
    { 91, 3,
      { AGC_SCALE_POS4, AGC_SCALE_VEL2, AGC_SCALE_DP_DEG360 },
      { AGC_DATA_GROUND_TRACK_DETERMINATION_ALT, AGC_DATA_GROUND_TRACK_DETERMINATION_VEL, AGC_DATA_GROUND_TRACK_DETERMINATION_GAM },
      false, false },
    /* 92  Spare */
    { 92, 0, { AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE },
         { AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE }, false, false },
    /* 93  Delta gyro angles
     *     3 comp; NNADTAB 04237, NNTYPTAB 06143, RUTMXTAB 16347 */
    { 93, 3,
      { AGC_SCALE_DP_DEG90, AGC_SCALE_DP_DEG90, AGC_SCALE_DP_DEG90 },
      { AGC_DATA_OGC, AGC_DATA_OGC_P2, AGC_DATA_OGC_P4 },
      false, false },
    /* 94  Spare */
    { 94, 0, { AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE },
         { AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE }, false, false },
    /* 95  Spare */
    { 95, 0, { AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE },
         { AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE }, false, false },
    /* 96  Spare */
    { 96, 0, { AGC_SCALE_NONE, AGC_SCALE_NONE, AGC_SCALE_NONE },
         { AGC_DATA_NONE, AGC_DATA_NONE, AGC_DATA_NONE }, false, false },
    /* 97  System test inputs
     *     3 comp; NNADTAB 04253, NNTYPTAB 00000, RUTMXTAB 06143 */
    { 97, 3,
      { AGC_SCALE_WHOLE, AGC_SCALE_WHOLE, AGC_SCALE_WHOLE },
      { AGC_DATA_DSPTEM1, AGC_DATA_DSPTEM1_P1, AGC_DATA_DSPTEM1_P2 },
      false, false },
    /* 98  System test results
     *     3 comp; NNADTAB 04256, NNTYPTAB 00000, RUTMXTAB 06043 */
    { 98, 3,
      { AGC_SCALE_WHOLE, AGC_SCALE_FRACTION, AGC_SCALE_WHOLE },
      { AGC_DATA_DSPTEM2, AGC_DATA_DSPTEM2_P1, AGC_DATA_DSPTEM2_P2 },
      false, false },
    /* 99  RMS in position / RMS in velocity / RMS in bias
     *     3 comp, dec only; NNADTAB 24261, NNTYPTAB 71572, RUTMXTAB 24247 */
    { 99, 3,
      { AGC_SCALE_POS9, AGC_SCALE_VEL4, AGC_SCALE_RADIANS },
      { AGC_DATA_WWPOS, AGC_DATA_WWVEL, AGC_DATA_WWBIAS },
      true, false },
};
