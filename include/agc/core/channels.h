/*
 * agc_channels.h -- I/O channel image.
 *
 * The AGC has 15-bit I/O channels 1..35 (octal). READ/WRITE/ROR/WOR/WAND/
 * RXOR map onto the functions below. Bit-name constants come from
 * Luminary099/INPUT_OUTPUT_CHANNEL_BIT_DESCRIPTIONS.agc (LM) and, for the
 * CSM, from the usage comments in Comanche055 (T4RUPT_PROGRAM, P11, P20-P25,
 * P40-P47, RCS-CSM_DIGITAL_AUTOPILOT, SXTMARK, AUTOMATIC_MANEUVERS):
 * Comanche055/ERASABLE_ASSIGNMENTS.agc itself only names the channels and
 * has no per-bit comments.
 *
 * Bit constants: AGC_CH<octal channel>_<NAME>, e.g. AGC_CH030_IMUOPER.
 * Each value is the single-bit mask (bit 1 = 000001 ... bit 15 = 040000).
 * Where the vehicles assign different functions to one bit, both names
 * exist with the same value; the comment starts "CSM:" or "LM:".
 *
 * Inverted-logic inputs: all bits of channels 030-033 are inverted as
 * sensed by the program: a value of ZERO means the signal is present.
 * The channel image holds exactly what a READ returns, i.e. the inverted
 * sense. agc_chan_set_input() therefore takes the raw sensed value;
 * agc_chan_set_signal() takes "signal present" and does the inversion.
 */
#ifndef AGC_CHANNELS_H
#define AGC_CHANNELS_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/core/status.h"

#define AGC_CHAN_COUNT 040           /* channels 0..037                     */
#define AGC_CHAN_MASK  077777u       /* channels are 15 bits wide           */
#define AGC_CHAN_FIRST_INVERTED 030  /* channels 030..033 read inverted     */
#define AGC_CHAN_LAST_INVERTED  033

enum agc_channel_number {
    AGC_CHAN_L        = 001,
    AGC_CHAN_Q        = 002,
    AGC_CHAN_HISCALAR = 003,         /* input: TIME counter bits (B23 cs)   */
    AGC_CHAN_LOSCALAR = 004,         /* input: TIME counter bits (B9 cs)    */
    AGC_CHAN_PYJETS   = 005,
    AGC_CHAN_ROLLJETS = 006,
    AGC_CHAN_SUPERBNK = 007,
    AGC_CHAN_OUT0     = 010,         /* DSKY display relays               */
    AGC_CHAN_DSALMOUT = 011,         /* DSKY lamps                        */
    AGC_CHAN_CHAN12   = 012,         /* IMU / optics moding               */
    AGC_CHAN_CHAN13   = 013,
    AGC_CHAN_CHAN14   = 014,         /* CDU drive, gyro, thrust           */
    AGC_CHAN_MNKEYIN  = 015,         /* main DSKY keycode                 */
    AGC_CHAN_NAVKEYIN = 016,         /* nav DSKY keycode / mark buttons   */
    AGC_CHAN_CHAN30   = 030,
    AGC_CHAN_CHAN31   = 031,
    AGC_CHAN_CHAN32   = 032,
    AGC_CHAN_CHAN33   = 033,
    AGC_CHAN_DNTM1    = 034,         /* downlink word 1                   */
    AGC_CHAN_DNTM2    = 035
};

/* Channel bit masks (see the header comment for naming and polarity).     */
enum agc_channel_bits {
    /* ---- channel 005 ---- */
    /* bit  1: PYJETS jet-driver bit 1 (pitch/yaw RCS jets) */
    AGC_CH005_PYJET1 = 000001,
    /* bit  2: PYJETS jet-driver bit 2 (pitch/yaw RCS jets) */
    AGC_CH005_PYJET2 = 000002,
    /* bit  3: PYJETS jet-driver bit 3 (pitch/yaw RCS jets) */
    AGC_CH005_PYJET3 = 000004,
    /* bit  4: PYJETS jet-driver bit 4 (pitch/yaw RCS jets) */
    AGC_CH005_PYJET4 = 000010,
    /* bit  5: PYJETS jet-driver bit 5 (pitch/yaw RCS jets) */
    AGC_CH005_PYJET5 = 000020,
    /* bit  6: PYJETS jet-driver bit 6 (pitch/yaw RCS jets) */
    AGC_CH005_PYJET6 = 000040,
    /* bit  7: PYJETS jet-driver bit 7 (pitch/yaw RCS jets) */
    AGC_CH005_PYJET7 = 000100,
    /* bit  8: PYJETS jet-driver bit 8 (pitch/yaw RCS jets) */
    AGC_CH005_PYJET8 = 000200,

    /* ---- channel 006 ---- */
    /* bit  1: ROLLJETS jet-driver bit 1 (roll RCS jets) */
    AGC_CH006_ROLLJET1 = 000001,
    /* bit  2: ROLLJETS jet-driver bit 2 (roll RCS jets) */
    AGC_CH006_ROLLJET2 = 000002,
    /* bit  3: ROLLJETS jet-driver bit 3 (roll RCS jets) */
    AGC_CH006_ROLLJET3 = 000004,
    /* bit  4: ROLLJETS jet-driver bit 4 (roll RCS jets) */
    AGC_CH006_ROLLJET4 = 000010,
    /* bit  5: ROLLJETS jet-driver bit 5 (roll RCS jets) */
    AGC_CH006_ROLLJET5 = 000020,
    /* bit  6: ROLLJETS jet-driver bit 6 (roll RCS jets) */
    AGC_CH006_ROLLJET6 = 000040,
    /* bit  7: ROLLJETS jet-driver bit 7 (roll RCS jets) */
    AGC_CH006_ROLLJET7 = 000100,
    /* bit  8: ROLLJETS jet-driver bit 8 (roll RCS jets) */
    AGC_CH006_ROLLJET8 = 000200,

    /* ---- channel 007 ---- */
    /* bit  5: fixed-extension (superbank) select, low bit */
    AGC_CH007_FEXT0 = 000020,
    /* bit  6: fixed-extension (superbank) select, middle bit */
    AGC_CH007_FEXT1 = 000040,
    /* bit  7: fixed-extension (superbank) select, high bit */
    AGC_CH007_FEXT2 = 000100,
    /* all three fixed-extension bits (bits 5-7) */
    AGC_CH007_FEXT_MASK = 000160,

    /* ---- channel 011 ---- */
    /* bit  1: ISS warning lamp */
    AGC_CH011_ISSWARN = 000001,
    /* bit  2: light computer activity lamp */
    AGC_CH011_COMPACT = 000002,
    /* bit  3: light uplink activity lamp */
    AGC_CH011_UPACT = 000004,
    /* bit  4: light temp caution lamp */
    AGC_CH011_TEMPCAUT = 000010,
    /* bit  5: light keyboard release lamp */
    AGC_CH011_KEYREL = 000020,
    /* bit  6: flash verb and noun lamps */
    AGC_CH011_VNFLASH = 000040,
    /* bit  7: light operator error lamp */
    AGC_CH011_OPERERR = 000100,
    /* bit  8: spare */
    AGC_CH011_SPARE8 = 000200,
    /* bit  9: test connector outbit */
    AGC_CH011_TESTCONN = 000400,
    /* bit 10: caution reset */
    AGC_CH011_CAUTRESET = 001000,
    /* bit 11: spare */
    AGC_CH011_SPARE11 = 002000,
    /* bit 12: spare */
    AGC_CH011_SPARE12 = 004000,
    /* bit 13: engine on */
    AGC_CH011_ENGINEON = 010000,
    /* bit 14: engine off */
    AGC_CH011_ENGINEOFF = 020000,
    /* bit 15: spare */
    AGC_CH011_SPARE15 = 040000,

    /* ---- channel 012 ---- */
    /* bit  1: CSM: zero optics CDUs */
    AGC_CH012_ZEROOCDU = 000001,
    /* bit  1: LM: zero RR CDUs (CDUs give rendezvous radar information) */
    AGC_CH012_ZERORRCDU = 000001,
    /* bit  2: CSM: enable optics CDU error counters */
    AGC_CH012_ENBOCDUERR = 000002,
    /* bit  2: LM: enable CDU radar error counters */
    AGC_CH012_ENBRRCDUERR = 000002,
    /* bit  3: not used */
    AGC_CH012_NOTUSED3 = 000004,
    /* bit  4: coarse align enable of IMU */
    AGC_CH012_COARSE = 000010,
    /* bit  5: zero IMU CDUs */
    AGC_CH012_ZEROICDU = 000020,
    /* bit  6: enable IMU error counter (CDU error counter) */
    AGC_CH012_ENBIMUERR = 000040,
    /* bit  7: spare */
    AGC_CH012_SPARE7 = 000100,
    /* bit  8: CSM: TVC enable (SPS servo amplifiers see DAC voltages) */
    AGC_CH012_TVCENABLE = 000200,
    /* bit  8: LM: display inertial data */
    AGC_CH012_DISPINERT = 000200,
    /* bit  9: CSM: enable S-IVB takeover (CMC steering of Saturn) */
    AGC_CH012_SIVBTAKEOVER = 000400,
    /* bit  9: LM: -pitch gimbal trim (bell motion) descent engine */
    AGC_CH012_TRIMPITCHM = 000400,
    /* bit 10: LM: +pitch gimbal trim (bell motion) descent engine */
    AGC_CH012_TRIMPITCHP = 001000,
    /* bit 11: CSM: disengage optics DAC */
    AGC_CH012_DISOPTDAC = 002000,
    /* bit 11: LM: -roll gimbal trim (bell motion) descent engine */
    AGC_CH012_TRIMROLLM = 002000,
    /* bit 12: LM: +roll gimbal trim (bell motion) descent engine */
    AGC_CH012_TRIMROLLP = 004000,
    /* bit 13: LM: LR position 2 command */
    AGC_CH012_LRPOS2CMD = 010000,
    /* bit 14: CSM: S-IVB cutoff command (backup use) */
    AGC_CH012_SIVCUTOFF = 020000,
    /* bit 14: LM: enable rendezvous radar lock-on; auto angle tracking */
    AGC_CH012_RRLOCKON = 020000,
    /* bit 15: ISS turn-on delay complete */
    AGC_CH012_ISSDELAYDONE = 040000,

    /* ---- channel 013 ---- */
    /* bit  1: radar C (A,B,C matrix setting selects radar parameters to be read) */
    AGC_CH013_RADARC = 000001,
    /* bit  2: radar B */
    AGC_CH013_RADARB = 000002,
    /* bit  3: radar A */
    AGC_CH013_RADARA = 000004,
    /* bit  4: radar activity */
    AGC_CH013_RADARACT = 000010,
    /* bit  5: not used (connects an alternate input to uplink) */
    AGC_CH013_NOTUSED5 = 000020,
    /* bit  6: block inputs to uplink cell */
    AGC_CH013_ULKBLOCK = 000040,
    /* bit  7: downlink telemetry word order code bit */
    AGC_CH013_DNTMWORD = 000100,
    /* bit  8: RHC counter enable (read hand controller angles) */
    AGC_CH013_RHCCNTENB = 000200,
    /* bit  9: start RHC read into counters if bit 8 set */
    AGC_CH013_RHCSTART = 000400,
    /* bit 10: test alarms, test DSKY lights */
    AGC_CH013_TESTALM = 001000,
    /* bit 11: enable standby */
    AGC_CH013_STANDBYENB = 002000,
    /* bit 12: reset trap 31-A (always appears set to 0) */
    AGC_CH013_RSTTRAP31A = 004000,
    /* bit 13: reset trap 31-B (always appears set to 0) */
    AGC_CH013_RSTTRAP31B = 010000,
    /* bit 14: reset trap 32 (always appears set to 0) */
    AGC_CH013_RSTTRAP32 = 020000,
    /* bit 15: enable T6 rupt */
    AGC_CH013_ENBT6RUPT = 040000,

    /* ---- channel 014 ---- */
    /* bit  1: outlink activity (not used) */
    AGC_CH014_OUTLINKACT = 000001,
    /* bit  2: altitude rate or altitude selector (LM) */
    AGC_CH014_ALTRATESEL = 000002,
    /* bit  3: altitude meter activity (LM) */
    AGC_CH014_ALTMETERACT = 000004,
    /* bit  4: thrust drive activity for descent engine (LM) */
    AGC_CH014_THRUSTACT = 000010,
    /* bit  5: spare */
    AGC_CH014_SPARE5 = 000020,
    /* bit  6: gyro enable power for pulses */
    AGC_CH014_GYROPWR = 000040,
    /* bit  7: gyro select B (with A identifies the gyro axis to be torqued) */
    AGC_CH014_GYROSELB = 000100,
    /* bit  8: gyro select A */
    AGC_CH014_GYROSELA = 000200,
    /* bit  9: gyro torquing command in negative direction */
    AGC_CH014_GYRONEG = 000400,
    /* bit 10: gyro activity */
    AGC_CH014_GYROACT = 001000,
    /* bit 11: drive CDU S (CSM: optics shaft / TVC pitch) */
    AGC_CH014_DRIVECDUS = 002000,
    /* bit 12: drive CDU T (CSM: optics trunnion / TVC yaw) */
    AGC_CH014_DRIVECDUT = 004000,
    /* bit 13: drive CDU Z */
    AGC_CH014_DRIVECDUZ = 010000,
    /* bit 14: drive CDU Y */
    AGC_CH014_DRIVECDUY = 020000,
    /* bit 15: drive CDU X */
    AGC_CH014_DRIVECDUX = 040000,

    /* ---- channel 016 ---- */
    /* bit  3: LM: optics X-axis mark signal for alignment optical telescope */
    AGC_CH016_XAXISMARK = 000004,
    /* bit  4: LM: optics Y-axis mark signal for AOT */
    AGC_CH016_YAXISMARK = 000010,
    /* bit  5: LM: optics mark reject signal */
    AGC_CH016_AOTREJECT = 000020,
    /* bit  6: CSM: optics mark (SXTMARK tests bit 6) */
    AGC_CH016_MARK = 000040,
    /* bit  6: LM: descent+ (crew desired slowing of rate of descent) */
    AGC_CH016_DESCENTUP = 000040,
    /* bit  7: CSM: mark reject (SXTMARK tests bit 7) */
    AGC_CH016_MARKREJECT = 000100,
    /* bit  7: LM: descent- (crew desired speeding up of rate of descent) */
    AGC_CH016_DESCENTDN = 000100,
    /* CSM: navigation DSKY key code (bits 5-1) */
    AGC_CH016_NAVKEY_MASK = 000037,

    /* ---- channel 030 ---- */
    /* bit  1: LM: abort with descent stage */
    AGC_CH030_ABORTDPS = 000001,
    /* bit  2: unused */
    AGC_CH030_UNUSED2 = 000002,
    /* bit  3: LM: engine armed signal */
    AGC_CH030_ENGARMED = 000004,
    /* bit  4: LM: abort with ascent engine stage */
    AGC_CH030_ABORTAPS = 000010,
    /* bit  5: CSM: liftoff discrete (Comanche CKLFTBTS tests it) */
    AGC_CH030_LIFTOFF = 000020,
    /* bit  5: LM: auto throttle; computer control of descent engine */
    AGC_CH030_AUTOTHROT = 000020,
    /* bit  6: LM: display inertial data */
    AGC_CH030_DISPINERT = 000040,
    /* bit  7: CSM: optics CDU fail (T4RUPT OPTMON) */
    AGC_CH030_OCDUFAIL = 000100,
    /* bit  7: LM: RR CDU fail */
    AGC_CH030_RRCDUFAIL = 000100,
    /* bit  8: spare */
    AGC_CH030_SPARE8 = 000200,
    /* bit  9: IMU operate with no malfunction */
    AGC_CH030_IMUOPER = 000400,
    /* bit 10: CSM: S/C control of Saturn panel switch (P11 ATERJOB) */
    AGC_CH030_SCCTLSATURN = 001000,
    /* bit 10: LM: LM computer (not AGS) has control of LM */
    AGC_CH030_LMCONTROL = 001000,
    /* bit 11: IMU cage command to drive IMU gimbal angles to 0 */
    AGC_CH030_IMUCAGE = 002000,
    /* bit 12: IMU CDU fail (malfunction of IMU CDUs) */
    AGC_CH030_ICDUFAIL = 004000,
    /* bit 13: IMU fail (malfunction of IMU stabilization loops) */
    AGC_CH030_IMUFAIL = 010000,
    /* bit 14: ISS turn-on requested */
    AGC_CH030_ISSTURNON = 020000,
    /* bit 15: temperature of stable member within design limits */
    AGC_CH030_IMUTEMPOK = 040000,
    /* all channel 30 bits are inverted (0 = signal present) */
    AGC_CH030_INVERT_ALL = 077777,

    /* ---- channel 031 ---- */
    /* bit  1: rotation (RHC) commanded in positive pitch; must be in minimum impulse mode; LM also positive elevation change for landing point designator */
    AGC_CH031_PLUSPITCH = 000001,
    /* bit  2: as bit 1 except negative pitch and elevation */
    AGC_CH031_MINUSPITCH = 000002,
    /* bit  3: rotation (RHC) commanded in positive yaw; must be in minimum impulse mode */
    AGC_CH031_PLUSYAW = 000004,
    /* bit  4: as bit 3 except negative yaw */
    AGC_CH031_MINUSYAW = 000010,
    /* bit  5: rotation (RHC) commanded in positive roll; LM also positive azimuth change for landing point designator */
    AGC_CH031_PLUSROLL = 000020,
    /* bit  6: as bit 5 except negative roll and azimuth */
    AGC_CH031_MINUSROLL = 000040,
    /* bit  7: translation in +X commanded by THC */
    AGC_CH031_PLUSX = 000100,
    /* bit  8: translation in -X commanded by THC */
    AGC_CH031_MINUSX = 000200,
    /* bit  9: translation in +Y commanded by THC */
    AGC_CH031_PLUSY = 000400,
    /* bit 10: translation in -Y commanded by THC */
    AGC_CH031_MINUSY = 001000,
    /* bit 11: translation in +Z commanded by THC */
    AGC_CH031_PLUSZ = 002000,
    /* bit 12: translation in -Z commanded by THC */
    AGC_CH031_MINUSZ = 004000,
    /* bit 13: attitude hold mode on SCS mode control switch (CSM Comanche tests it as HOLD) */
    AGC_CH031_ATTHOLD = 010000,
    /* bit 14: CSM: free function (Comanche tests it as FREE mode) */
    AGC_CH031_FREE = 020000,
    /* bit 14: LM: auto stabilization of attitude on SCS mode switch */
    AGC_CH031_AUTOSTAB = 020000,
    /* bit 15: CSM: 0 if IMU power is on and S/C control switch is in CMC (G/C autopilot fully enabled) */
    AGC_CH031_CMCCONTROL = 040000,
    /* bit 15: LM: attitude control out of detent (RHC not in neutral) */
    AGC_CH031_ATTOUTDETENT = 040000,
    /* RHC rotation command bits 1-6 (Comanche MANROT) */
    AGC_CH031_ROT_MASK = 000077,
    /* THC translation command bits 7-12 (Comanche XLNMASK) */
    AGC_CH031_XLATE_MASK = 007700,

    /* ---- channel 032 ---- */
    /* bit  1: LM: thrusters 2 & 4 disabled by crew */
    AGC_CH032_THR24DIS = 000001,
    /* bit  2: LM: thrusters 5 & 8 disabled by crew */
    AGC_CH032_THR58DIS = 000002,
    /* bit  3: LM: thrusters 1 & 3 disabled by crew */
    AGC_CH032_THR13DIS = 000004,
    /* bit  4: LM: thrusters 6 & 7 disabled by crew */
    AGC_CH032_THR67DIS = 000010,
    /* bit  5: LM: thrusters 14 & 16 disabled by crew */
    AGC_CH032_THR1416DIS = 000020,
    /* bit  6: LM: thrusters 13 & 15 disabled by crew */
    AGC_CH032_THR1315DIS = 000040,
    /* bit  7: LM: thrusters 9 & 12 disabled by crew */
    AGC_CH032_THR912DIS = 000100,
    /* bit  8: LM: thrusters 10 & 11 disabled by crew */
    AGC_CH032_THR1011DIS = 000200,
    /* bit  9: LM: descent engine gimbals disabled by crew */
    AGC_CH032_DPSGIMBDIS = 000400,
    /* bit 10: LM: apparent descent engine gimbal failure */
    AGC_CH032_DPSGIMBFAIL = 001000,
    /* bit 14: proceed key is depressed */
    AGC_CH032_PROCEED = 020000,

    /* ---- channel 033 ---- */
    /* bit  1: spare */
    AGC_CH033_SPARE1 = 000001,
    /* bit  2: CSM: VHF ranging data good (P20 RANGERD reads it) */
    AGC_CH033_VHFRANGEGOOD = 000002,
    /* bit  2: LM: RR auto-power on */
    AGC_CH033_RRAUTOPWR = 000002,
    /* bit  3: LM: RR range low scale */
    AGC_CH033_RRRANGELOW = 000004,
    /* bit  4: CSM: optics mode switch sense bit (OPTMON, with bit 5) */
    AGC_CH033_OPTMODE1 = 000010,
    /* bit  4: LM: RR data good */
    AGC_CH033_RRDATAGOOD = 000010,
    /* bit  5: CSM: optics mode switch sense bit (OPTMON, set = CSC after inversion) */
    AGC_CH033_OPTMODE2 = 000020,
    /* bit  5: LM: LR range data good */
    AGC_CH033_LRRANGEGOOD = 000020,
    /* bit  6: LM: LR position 1 */
    AGC_CH033_LRPOS1 = 000040,
    /* bit  7: LM: LR position 2 */
    AGC_CH033_LRPOS2 = 000100,
    /* bit  8: LM: LR velocity data good */
    AGC_CH033_LRVELGOOD = 000200,
    /* bit  9: LM: LR range low scale */
    AGC_CH033_LRRANGELOW = 000400,
    /* bit 10: block uplink input */
    AGC_CH033_BLOCKUPLINK = 001000,
    /* bit 11: uplink too fast (flip-flop) */
    AGC_CH033_UPLINKFAST = 002000,
    /* bit 12: downlink too fast (flip-flop) */
    AGC_CH033_DOWNLINKFAST = 004000,
    /* bit 13: PIPA fail (flip-flop) */
    AGC_CH033_PIPAFAIL = 010000,
    /* bit 14: warning of repeated alarms: restart, counter fail, voltage fail, scaler double (flip-flop) */
    AGC_CH033_REPEATEDALM = 020000,
    /* bit 15: LGC oscillator stopped (flip-flop) */
    AGC_CH033_OSCSTOPPED = 040000,
    /* flip-flop bits 11-15, reset by a channel write pulse */
    AGC_CH033_FLIPFLOP_MASK = 076000,
};

uint16_t     agc_chan_read(int chan);                   /* READ          */
agc_status_t agc_chan_write(int chan, uint16_t value);  /* WRITE         */
agc_status_t agc_chan_or(int chan, uint16_t mask);      /* WOR           */
agc_status_t agc_chan_and(int chan, uint16_t mask);     /* WAND          */
agc_status_t agc_chan_set_input(int chan, uint16_t value); /* sim: raw sensed value */
agc_status_t agc_chan_set_signal(int chan, uint16_t mask, bool present); /* sim: signal on/off */
void         agc_chan_reset(void);                      /* power-up image */

#endif /* AGC_CHANNELS_H */
