/**********************************************************************
 * FILE NAME: state.h
 *
 * PURPOSE:
 *   Shared erasable memory: vehicle selection, the TIME2/TIME1 clock,
 *   the flagwords and the accessors agc_erase_<name>() /
 *   agc_erase_set_<name>() for erasables read or written by more than
 *   one module. Module-private erasables stay static in their module.
 *
 * AGC SOURCE:
 *   Comanche055 ERASABLE_ASSIGNMENTS.agc pp. 39-124 (registers, STATE,
 *   PRM/DSP erasables); Luminary099 ERASABLE_ASSIGNMENTS.agc pp. 92-147
 *   and FLAGWORD_ASSIGNMENTS.agc pp. 61-88. Each accessor comment gives
 *   the AGC name and its scale.
 *
 * REQUIRES:
 *   none (includes interp.h, status.h, word.h)
 *
 * GLOBAL VARIABLES:
 *   none (storage is static in src/core/state.c, erasable.c and the
 *   files named in the section headings below)
 *
 * NOTES:
 *   Flag API: a flag is an agc_flag_id_t = (flagword << 4) | bit, where
 *   bit is the AGC bit number 1..15 (bit 1 = least significant). The
 *   constants AGC_FLAG_<AGCNAME> are in flags_common.h (same name and
 *   position in CSM and LM), flags_csm.h and flags_lm.h (include only
 *   the one for the vehicle, never both). The name -> accessor/owner
 *   table for every AGC erasable is in docs/modules/state-flags.md.
 **********************************************************************/
#ifndef AGC_STATE_H
#define AGC_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/interp.h"
#include "agc/core/status.h"
#include "agc/core/word.h"

/* Which program this build models. Set once by the sim harness.           */
typedef enum agc_vehicle
{
    AGC_VEHICLE_CSM = 0,
    AGC_VEHICLE_LM  = 1
} agc_vehicle_t;

void          agc_state_init(agc_vehicle_t v); /* select vehicle, zero ALL shared state (fresh start) */
agc_vehicle_t agc_state_vehicle(void);
void          agc_erase_reset(void);           /* zero every shared erasable below (used by agc_state_init) */

/* ---- time: TIME1/TIME2 (centiseconds, DP), TIME3..TIME6 are exec-private */
#define AGC_TIME_MODULUS_CS \
    0x10000000u                               /* TIME2,TIME1 wrap at 2^28 cs (about 745.7 h) */
uint32_t agc_time_now_cs(void);               /* C(TIME2,TIME1) as 28-bit  */
void     agc_time_set_cs(uint32_t t_cs);      /* t_cs < 2^28, else ignored */
void     agc_time_advance_cs(uint16_t dt_cs); /* wraps modulo 2^28 like TIME2 overflow */
agc_sp_t agc_erase_time1(void);               /* TIME1: low 14 bits of the count (register 025) */
agc_sp_t agc_erase_time2(void);               /* TIME2: high 14 bits of the count (register 024) */

/* ---- flagwords: FLAGWRD0..FLAGWRD13, bit names in agc_flags_*.h --------- */
typedef uint16_t agc_flag_id_t;                  /* (flagword << 4) | bitindex  */
#define AGC_FLAGWORDS     14                     /* LM has FLGWRD0..13 (STATE+0..+13D); CSM has 0..11 */
#define AGC_FLAGWORDS_CSM 12                     /* Comanche055: FLAGWRD0..FLGWRD11     */
#define AGC_FLAGWORDS_LM  14                     /* Luminary099: FLAGWRD0..FLGWRD13     */
bool     agc_flag_valid(agc_flag_id_t f);        /* word/bit exist for this vehicle */
int      agc_flag_word(agc_flag_id_t f);         /* flagword index of f   */
int      agc_flag_bit(agc_flag_id_t f);          /* AGC bit number 1..15  */
uint16_t agc_flag_mask(agc_flag_id_t f);         /* 1 << (bit - 1)        */
bool     agc_flag_test(agc_flag_id_t f);         /* BON/BOFF        */
void     agc_flag_set(agc_flag_id_t f);          /* SET             */
void     agc_flag_clear(agc_flag_id_t f);        /* CLEAR           */
void     agc_flag_invert(agc_flag_id_t f);       /* INVERT          */
void     agc_flag_put(agc_flag_id_t f, bool on); /* SET if on, else CLEAR */
uint16_t agc_flagword_get(int word);
void     agc_flagword_set(int word, uint16_t value);

/* ---- state vectors shared by navigation, guidance and programs --------- */
typedef struct agc_state_vector
{
    agc_vec_t r;    /* position, meters, scale 2^29 always (RN); conic-local copies use 2^27 for moon */
    agc_vec_t v;    /* velocity, m/cs,   scale 2^7 always (VN); conic-local copies use 2^5 for moon  */
    agc_dp_t  t;    /* time of state, centiseconds, scale 2^28        */
    bool      moon; /* primary body (MOONFLAG equivalent)             */
} agc_state_vector_t;

/*
 * AGC_SV_THIS  = RN, VN, PIPTIME with moon = MOONFLAG (the working state of
 *                the integrated/servicer vehicle).
 * AGC_SV_OTHER = the permanent state vector of the other vehicle:
 *                CSM build -> RRECTLEM, VRECTLEM, TETLEM, moon = LMOONFLG;
 *                LM build  -> RRECTCSM, VRECTCSM, TETCSM, moon = CMOONFLG.
 * agc_sv_set stores the vectors and time AND sets/clears the matching moon
 * flag (documented deviation from the AGC, where the flag is set by the
 * integration code; the struct bundles them).
 */
typedef enum agc_vehicle_sv
{
    AGC_SV_THIS  = 0,
    AGC_SV_OTHER = 1
} agc_vehicle_sv_t;

void agc_sv_get(agc_vehicle_sv_t which, agc_state_vector_t *out); /* RN,VN,PIPTIME / other */
void agc_sv_set(agc_vehicle_sv_t which, const agc_state_vector_t *in);

/* ---- shared erasables: agc_erase_<agcname>() / agc_erase_set_<agcname>() --
 * Names follow the AGC erasable name, lower case. Values are held exactly as
 * the AGC holds them (scale factors in the comments, from the AGC text).
 * The accessors hold no checks (callers pass the AGC's values). Everything not
 * listed here is module-private to the module that owns it; see
 * docs/modules/state-flags.md for the full name -> accessor/owner table. */

agc_sp_t agc_erase_modreg(void); /* MODREG major mode (program)    */
void     agc_erase_set_modreg(agc_sp_t mode);

/* time of the PIPA reading / this vehicle's state vector, centiseconds, scale
 * 2^28 (PIPTIME, B(2)PRM) */
agc_dp_t agc_erase_piptime(void);
void     agc_erase_set_piptime(agc_dp_t v);

/* time of ignition, centiseconds, scale 2^28 (TIG; DSP noun 33) */
agc_dp_t agc_erase_tig(void);
void     agc_erase_set_tig(agc_dp_t v);

/* time to go, centiseconds, scale 2^28 (TTOGO; DSP nouns 35,40,45,59,99) */
agc_dp_t agc_erase_ttogo(void);
void     agc_erase_set_ttogo(agc_dp_t v);

/* time to go in the burn / guidance, centiseconds, scale 2^28 (TGO) */
agc_dp_t agc_erase_tgo(void);
void     agc_erase_set_tgo(agc_dp_t v);

/* TPI time, centiseconds, scale 2^28 (TTPI; DSP noun 37) */
agc_dp_t agc_erase_ttpi(void);
void     agc_erase_set_ttpi(agc_dp_t v);

/* time of event for downlist, centiseconds, scale 2^28 (TEVENT; TLIFTOFF in
 * P11) */
agc_dp_t agc_erase_tevent(void);
void     agc_erase_set_tevent(agc_dp_t v);

/* time of the permanent CSM state vector, centiseconds, scale 2^28 (TETCSM) */
agc_dp_t agc_erase_tetcsm(void);
void     agc_erase_set_tetcsm(agc_dp_t v);

/* time of the permanent LM state vector, centiseconds, scale 2^28 (TETLEM) */
agc_dp_t agc_erase_tetlem(void);
void     agc_erase_set_tetlem(agc_dp_t v);

/* this vehicle's position, meters, scale 2^29 (earth) or 2^27 (moon) (RN,
 * B(6)PRM) */
agc_vec_t agc_erase_rn(void);
void      agc_erase_set_rn(agc_vec_t v);

/* this vehicle's velocity, m/cs, scale 2^7 (earth) or 2^5 (moon) (VN, B(6)PRM) */
agc_vec_t agc_erase_vn(void);
void      agc_erase_set_vn(agc_vec_t v);

/* permanent CSM position, meters, scale 2^29 earth / 2^27 moon (RRECTCSM) */
agc_vec_t agc_erase_rrectcsm(void);
void      agc_erase_set_rrectcsm(agc_vec_t v);

/* permanent CSM velocity, m/cs, scale 2^7 earth / 2^5 moon (VRECTCSM) */
agc_vec_t agc_erase_vrectcsm(void);
void      agc_erase_set_vrectcsm(agc_vec_t v);

/* permanent LM position, meters, scale 2^29 earth / 2^27 moon (RRECTLEM) */
agc_vec_t agc_erase_rrectlem(void);
void      agc_erase_set_rrectlem(agc_vec_t v);

/* permanent LM velocity, m/cs, scale 2^7 earth / 2^5 moon (VRECTLEM) */
agc_vec_t agc_erase_vrectlem(void);
void      agc_erase_set_vrectlem(agc_vec_t v);

/* accumulated PIPA delta-V, stable-member axes (DELV, I(6)): PIPA pulses
 * scale 2^14 (count in the high word); 1 pulse = 5.85 cm/s CSM, 1 cm/s LM.
 * KPIP1 rescales it to 2^7 m/cs (SERVICER207 p. 821, LM CONTROLLED_CONSTANTS) */
agc_vec_t agc_erase_delv(void);
void      agc_erase_set_delv(agc_vec_t v);

/* DELVSLV has no accessor: DELVSLV = DELVLVC (same six words in both
 * ropes), held in agc_rendz_erasable()->delvlvc, m/cs scale 2^7. */

/* velocity to be gained at TIG, m/cs, scale 2^7 (VGTIG, P40-P47: 'VG IGNITION
 * SCALED AT 2(+7) M/CS') */
agc_vec_t agc_erase_vgtig(void);
void      agc_erase_set_vgtig(agc_vec_t v);

/* landing site vector, moon-fixed reference, meters, scale 2^27 (RLS; R30
 * 'SCALED AT (-27)M'); used by P20/P22/P52 lat-long, R30, P57, landing */
agc_vec_t agc_erase_rls(void);
void      agc_erase_set_rls(agc_vec_t v);

/* reference-to-stable-member matrix, row-major, DP elements, scale 2^1
 * (REFSMMAT, I(18D)PRM; 'REFSMMAT SCALE OF B1') */
agc_mat_t agc_erase_refsmmat(void);
void      agc_erase_set_refsmmat(agc_mat_t v);

/* time of the ephemeris epoch (TEPHEM = TIMSUBO, TP, centiseconds, scale
 * 2^42: L "CSEC B-42 (TRIPLE PRECISION)"; P11 sets it at liftoff) */
agc_tp_t agc_erase_tephem(void);
void     agc_erase_set_tephem(agc_tp_t v);

/* CSM mass, kg, scale 2^16 (CSMMASS; TVCMASSPROP 'B+16 KILOGRAMS'; noun 47 R1) */
agc_sp_t agc_erase_csmmass(void);
void     agc_erase_set_csmmass(agc_sp_t v);

/* LM mass, kg, scale 2^16 (LEMMASS; DSP noun 47 R2) */
agc_sp_t agc_erase_lemmass(void);
void     agc_erase_set_lemmass(agc_sp_t v);

/* LM vehicle mass, kg, scale 2^16, DP (MASS: SERVICER MASSMON, 1/ACCS) */
agc_dp_t agc_erase_mass(void);
void     agc_erase_set_mass(agc_dp_t v);

/* |DELV| of the last PIPA reading, cm/s, scale 2^14 (ABDELV, LM) */
agc_sp_t agc_erase_abdelv(void);
void     agc_erase_set_abdelv(agc_sp_t v);

/* MRKBUF1 / MRKBUF2 (CM): two seven-word mark buffers (time DP, CDUY,
 * CDUS, CDUZ, CDUT, CDUX), raw 15-bit words, buf 1 or 2, index 0..6;
 * word 0 = -1 (77776) marks a buffer empty. mrkbuf2 / set_mrkbuf2 read
 * and store MRKBUF2 +index as a signed SP number (ones' complement).     */
uint16_t agc_erase_mrkbuf(int buf, int index);
void     agc_erase_set_mrkbuf(int buf, int index, uint16_t v);
agc_sp_t agc_erase_mrkbuf2(int index);
void     agc_erase_set_mrkbuf2(int index, agc_sp_t v);

/* LANDMARK (CM P22 code ABCDE, V52), a signed SP number                  */
agc_sp_t agc_erase_landmark(void);
void     agc_erase_set_landmark(agc_sp_t v);

/* PIPA X pulse count since last read (register PIPAX, 037), unscaled
 * pulses: 1 pulse = 5.85 cm/s (CSM SERVICER207), 1 cm/s (LM) */
agc_sp_t agc_erase_pipax(void);
void     agc_erase_set_pipax(agc_sp_t v);

/* PIPA Y pulse count (register PIPAY) */
agc_sp_t agc_erase_pipay(void);
void     agc_erase_set_pipay(agc_sp_t v);

/* PIPA Z pulse count (register PIPAZ) */
agc_sp_t agc_erase_pipaz(void);
void     agc_erase_set_pipaz(agc_sp_t v);

/* CDU X (outer gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution
 * (register CDUX, 032) */
uint16_t agc_erase_cdux(void);
void     agc_erase_set_cdux(uint16_t v);

/* CDU Y (inner gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution
 * (CDUY) */
uint16_t agc_erase_cduy(void);
void     agc_erase_set_cduy(uint16_t v);

/* CDU Z (middle gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution
 * (CDUZ) */
uint16_t agc_erase_cduz(void);
void     agc_erase_set_cduz(uint16_t v);

/* CSM optics trunnion CDU raw word (CDUT / OPTY); LM: rendezvous radar
 * trunnion */
uint16_t agc_erase_cdut(void);
void     agc_erase_set_cdut(uint16_t v);

/* CSM optics shaft CDU raw word (CDUS / OPTX); LM: rendezvous radar shaft */
uint16_t agc_erase_cdus(void);
void     agc_erase_set_cdus(uint16_t v);

/* mark status bit word (MARKSTAT, B(1)PRM; SXTMARK/R21/R22/P20/P23) */
uint16_t agc_erase_markstat(void);
void     agc_erase_set_markstat(uint16_t v);

/* last-sampled channel 30 bits, T4RUPT IMUMON (IMODES30, B(1)PRM) */
uint16_t agc_erase_imodes30(void);
void     agc_erase_set_imodes30(uint16_t v);

/* last-sampled channel 33 / DAP-enable bits, T4RUPT C33TEST (IMODES33,
 * B(1)PRM) */
uint16_t agc_erase_imodes33(void);
void     agc_erase_set_imodes33(uint16_t v);

/* DAP configuration word 1 (DAPDATR1, DSP noun 46 R1) */
uint16_t agc_erase_dapdatr1(void);
void     agc_erase_set_dapdatr1(uint16_t v);

/* DAP configuration word 2 (DAPDATR2, DSP noun 46 R2; Comanche only, storage
 * exists in both builds) */
uint16_t agc_erase_dapdatr2(void);
void     agc_erase_set_dapdatr2(uint16_t v);

/* display panel relay buffer DSPTAB +0..+10D, +11D is the C/S lights word
 * (15-bit relay words) Index 0..11. */
uint16_t agc_erase_dsptab(int index);
void     agc_erase_set_dsptab(int index, uint16_t v);

/* three alarm code registers FAILREG +0..+2 (octal alarm codes) Index 0..2. */
uint16_t agc_erase_failreg(int index);
void     agc_erase_set_failreg(int index, uint16_t v);

/* desired gimbal angles THETAD +0..+2 (CPHI/CTHETA/CPSI), raw 15-bit CDU-style
 * words, 2^15 = 1 revolution Index 0..2. */
uint16_t agc_erase_thetad(int index);
void     agc_erase_set_thetad(int index, uint16_t v);

/* ---- hardware counter cells (src/core/counters.c) ------------------------
 * P-, Q-, R-RHCCTR (LM 042-044): rotational hand controller counts, SP.   */
agc_sp_t agc_erase_p_rhcctr(void);
agc_sp_t agc_erase_q_rhcctr(void);
agc_sp_t agc_erase_r_rhcctr(void);
void     agc_erase_set_p_rhcctr(agc_sp_t v);
void     agc_erase_set_q_rhcctr(agc_sp_t v);
void     agc_erase_set_r_rhcctr(agc_sp_t v);

/* RNRAD (046): the radar reading (RR, LR or VHF range), raw 15-bit word.   */
uint16_t agc_erase_rnrad(void);
void     agc_erase_set_rnrad(uint16_t v);

/* INLINK (045): uplink word for UPRUPT, raw 15-bit word.                   */
uint16_t agc_erase_inlink(void);
void     agc_erase_set_inlink(uint16_t v);

/* CDUTCMD (053) / CDUSCMD (054): optics (CSM) or RR (LM) CDU error counter
 * drive, SP pulse counts; which 0 = CDUTCMD, 1 = CDUSCMD. In TVC mode the
 * same cells are TVCYAW / TVCPITCH, 85.41 arc-sec per pulse.               */
agc_sp_t agc_erase_cdutcmd(int which);
void     agc_erase_set_cdutcmd(int which, agc_sp_t v);
agc_sp_t agc_erase_tvcyaw(void);   /* = CDUTCMD */
void     agc_erase_set_tvcyaw(agc_sp_t v);
agc_sp_t agc_erase_tvcpitch(void); /* = CDUSCMD */
void     agc_erase_set_tvcpitch(agc_sp_t v);

/* THRUST (LM 055): descent engine throttle command pulses, SP.             */
agc_sp_t agc_erase_thrust(void);
void     agc_erase_set_thrust(agc_sp_t pulses);

/* ALTM (LM 060): altitude / altitude-rate meter drive, raw 15-bit word.    */
uint16_t agc_erase_altm(void);
void     agc_erase_set_altm(uint16_t word);

/* PIPASR +3: read PIPAX..Z into TEMX..Z and the high words of DELV, zero
 * the PIPAs (no PIPTIME1). TEMX..TEMZ (axis 0..2): those counts, SP.       */
void     agc_pipasr_noscale(void);
agc_sp_t agc_erase_tem(int axis);
void     agc_erase_set_tem(int axis, agc_sp_t v);

/* ---- display and downlink registers (src/core/display_erasables.c) ------
 * OPTION1 +index (index 0..2 = OPTION1, OPTION2, OPTION3; noun 06), octal. */
agc_sp_t agc_erase_option1(int index);
void     agc_erase_set_option1(int index, agc_sp_t v);

/* SAMPTIME: sampled TIME2,TIME1 (noun 65), cs 2^28.                         */
agc_dp_t agc_erase_samptime(void);
void     agc_erase_set_samptime(agc_dp_t v);

/* SFAIL (= SELFERAS): self-check failure address, raw word.                */
uint16_t agc_erase_sfail(void);
void     agc_erase_set_sfail(uint16_t v);

/* MARKDOWN +0..+6 / MARK2DWN +0..+6: mark data for the downlink (time
 * high/low, CDUY, CDUS(OPTX), CDUZ, CDUT(OPTY), CDUX), raw words.          */
uint16_t agc_erase_markdown(int index);
void     agc_erase_set_markdown(int index, uint16_t v);
uint16_t agc_erase_mark2dwn(int index);
void     agc_erase_set_mark2dwn(int index, uint16_t v);

/* TRKMKCNT: rendezvous tracking mark count (noun 45 R1), whole number.     */
agc_sp_t agc_erase_trkmkcnt(void);
void     agc_erase_set_trkmkcnt(agc_sp_t v);

/* T3TOT4 (= RELDELV +2): transfer time for noun 39, cs 2^28.               */
agc_dp_t agc_erase_t3tot4(void);
void     agc_erase_set_t3tot4(agc_dp_t v);

/* RR-AZ, RR-ELEV (LM, noun 56): rendezvous radar LOS angles, revs 2^0.    */
agc_dp_t agc_erase_rr_az(void);
void     agc_erase_set_rr_az(agc_dp_t v);
agc_dp_t agc_erase_rr_elev(void);
void     agc_erase_set_rr_elev(agc_dp_t v);

/* ---- pad loads (src/core/pad_loads.c) ------------------------------------
 * AOTAZ -1+n, AOTEL -1+n (LM, detent n = 1..6): AOT azimuth / elevation,
 * CDU words (2^15 = 1 revolution).                                         */
uint16_t agc_erase_aotaz(int detent);
void     agc_erase_set_aotaz(int detent, uint16_t v);
uint16_t agc_erase_aotel(int detent);
void     agc_erase_set_aotel(int detent, uint16_t v);

/* CDUCHKWD (CSM): CDU check delay, cs (0 = use the default).               */
agc_sp_t agc_erase_cduchkwd(void);
void     agc_erase_set_cduchkwd(agc_sp_t v);

/* RTED1 (CSM): P37 VGAMMA polynomial coefficient D1, scale 2^-3.           */
agc_dp_t agc_erase_rted1(void);
void     agc_erase_set_rted1(agc_dp_t v);

/* P37RANGE (CSM, = R61CNTR +1): P37 entry range override, revs (0 = use
 * the computed range).                                                     */
agc_sp_t agc_erase_return_to_earth_range(void);
void     agc_erase_set_return_to_earth_range(agc_sp_t v);

/* TLAND (LM, = W): nominal time of landing, cs 2^28.                       */
agc_dp_t agc_erase_tland(void);
void     agc_erase_set_tland(agc_dp_t v);

/* ---- raw image of the address space (src/core/ecadr.c) ------------------
 * Address 00000-77777 (bit 16 ignored). Erasable access by address, with
 * the flagwords and named erasables mapped onto typed storage, is
 * agc/erasable_map/address_map.h; this is only the word image it keeps. */
void agc_erase_raw_read(uint16_t ecadr, uint16_t *word);
void agc_erase_raw_write(uint16_t ecadr, uint16_t word);

/* ---- restart-table delay times (src/core/restart_delays.c) ---------------
 * P40TMP (CSM, DP cs), SAVET-30 (LM, DP cs), ZOOMTIME (LM pad load, cs),
 * S61DT and CM/GYMDT (CSM entry, cs): read by the restart tables.          */
agc_dp_t agc_erase_p40tmp(void);
void     agc_erase_set_p40tmp(agc_dp_t v);
agc_dp_t agc_erase_savet_30(void);
void     agc_erase_set_savet_30(agc_dp_t v);
agc_sp_t agc_erase_zoomtime(void);
void     agc_erase_set_zoomtime(agc_sp_t v);
agc_sp_t agc_erase_s61dt(void);
void     agc_erase_set_s61dt(agc_sp_t v);
agc_sp_t agc_erase_cm_gymdt(void);
void     agc_erase_set_cm_gymdt(agc_sp_t v);

/* Fresh-start zeroing of the files above (called by agc_erase_reset).     */
void agc_erase_reset_ecadr(void);

#endif /* AGC_STATE_H */
