/*
 * agc_state.h -- shared erasable memory and flagwords.
 *
 * The state-flags agent owns src/core/agc_state.c, src/core/agc_erasable.c,
 * agc_flags_*.h and agc_channels.h, and may extend THIS header with further
 * accessors for erasables that are shared by more than one module.
 * Module-private erasables stay static inside their module.
 *
 * Flag API: a flag is an agc_flag_id_t = (flagword << 4) | bit, where bit is
 * the AGC bit number 1..15 (bit 1 = least significant). The enum constants
 * AGC_FLAG_<AGCNAME> are in agc_flags_common.h (same name+position in CSM and
 * LM), agc_flags_csm.h and agc_flags_lm.h (vehicle-specific; include only the
 * one for the build, never both).
 */
#ifndef AGC_STATE_H
#define AGC_STATE_H

#include <stdbool.h>
#include <stdint.h>
#include "agc_status.h"
#include "agc_word.h"
#include "agc_interp.h"

/* Which program this build models. Set once by the sim harness.           */
typedef enum agc_vehicle { AGC_VEHICLE_CSM = 0, AGC_VEHICLE_LM = 1 } agc_vehicle_t;
void          agc_state_init(agc_vehicle_t v);  /* select vehicle, zero ALL shared state (fresh start) */
agc_vehicle_t agc_state_vehicle(void);
void          agc_erase_reset(void);   /* zero every shared erasable below (used by agc_state_init) */

/* ---- time: TIME1/TIME2 (centiseconds, DP), TIME3..TIME6 are exec-private */
#define AGC_TIME_MODULUS_CS 0x10000000u /* TIME2,TIME1 wrap at 2^28 cs (about 745.7 h) */
uint32_t agc_time_now_cs(void);             /* C(TIME2,TIME1) as 28-bit  */
agc_status_t agc_time_set_cs(uint32_t t_cs);    /* t_cs < 2^28 */
void     agc_time_advance_cs(uint16_t dt_cs);   /* wraps modulo 2^28 like TIME2 overflow */
agc_sp_t agc_erase_time1(void);             /* TIME1: low 14 bits of the count (register 025) */
agc_sp_t agc_erase_time2(void);             /* TIME2: high 14 bits of the count (register 024) */

/* ---- flagwords: FLAGWRD0..FLAGWRD13, bit names in agc_flags_*.h --------- */
typedef uint16_t agc_flag_id_t;             /* (flagword << 4) | bitindex  */
#define AGC_FLAGWORDS       14  /* LM has FLGWRD0..13 (STATE+0..+13D); CSM has 0..11 */
#define AGC_FLAGWORDS_CSM   12
#define AGC_FLAGWORDS_LM    14
bool         agc_flag_valid(agc_flag_id_t f);          /* word/bit exist for this vehicle */
int          agc_flag_word(agc_flag_id_t f);           /* flagword index of f   */
int          agc_flag_bit(agc_flag_id_t f);            /* AGC bit number 1..15  */
uint16_t     agc_flag_mask(agc_flag_id_t f);           /* 1 << (bit - 1)        */
bool         agc_flag_test(agc_flag_id_t f);           /* BON/BOFF        */
agc_status_t agc_flag_set(agc_flag_id_t f);            /* SET             */
agc_status_t agc_flag_clear(agc_flag_id_t f);          /* CLEAR           */
agc_status_t agc_flag_invert(agc_flag_id_t f);         /* INVERT          */
uint16_t     agc_flagword_get(int word);
agc_status_t agc_flagword_set(int word, uint16_t value);

/* ---- state vectors shared by navigation, guidance and programs --------- */
typedef struct agc_state_vector {
    agc_vec_t r;          /* position, meters, scale 2^29 earth / 2^27 moon */
    agc_vec_t v;          /* velocity, m/cs,   scale 2^7  earth / 2^5  moon */
    agc_dp_t  t;          /* time of state, centiseconds, scale 2^28        */
    bool      moon;       /* primary body (MOONFLAG equivalent)             */
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
typedef enum agc_vehicle_sv { AGC_SV_THIS = 0, AGC_SV_OTHER = 1 } agc_vehicle_sv_t;
agc_status_t agc_sv_get(agc_vehicle_sv_t which, agc_state_vector_t *out); /* RN,VN,PIPTIME / other */
agc_status_t agc_sv_set(agc_vehicle_sv_t which, const agc_state_vector_t *in);

/* ---- shared erasables: agc_erase_<agcname>() / agc_erase_set_<agcname>() --
 * Names follow the AGC erasable name, lower case. Values are held exactly as
 * the AGC holds them (scale factors in the comments, from the AGC text).
 * Setters ignore (and AGC_ASSERT-report) out-of-range values. Everything not
 * listed here is module-private to the module that owns it; see
 * docs/modules/state-flags.md for the full name -> accessor/owner table. */

agc_sp_t     agc_erase_modreg(void);        /* MODREG major mode (program)    */
void         agc_erase_set_modreg(agc_sp_t mode);

/* time of the PIPA reading / this vehicle's state vector, centiseconds, scale
 * 2^28 (PIPTIME, B(2)PRM) */
agc_dp_t     agc_erase_piptime(void);
void         agc_erase_set_piptime(agc_dp_t v);

/* time of ignition, centiseconds, scale 2^28 (TIG; DSP noun 33) */
agc_dp_t     agc_erase_tig(void);
void         agc_erase_set_tig(agc_dp_t v);

/* time to go, centiseconds, scale 2^28 (TTOGO; DSP nouns 35,40,45,59,99) */
agc_dp_t     agc_erase_ttogo(void);
void         agc_erase_set_ttogo(agc_dp_t v);

/* time to go in the burn / guidance, centiseconds, scale 2^28 (TGO) */
agc_dp_t     agc_erase_tgo(void);
void         agc_erase_set_tgo(agc_dp_t v);

/* TPI time, centiseconds, scale 2^28 (TTPI; DSP noun 37) */
agc_dp_t     agc_erase_ttpi(void);
void         agc_erase_set_ttpi(agc_dp_t v);

/* time of event for downlist, centiseconds, scale 2^28 (TEVENT; TLIFTOFF in
 * P11) */
agc_dp_t     agc_erase_tevent(void);
void         agc_erase_set_tevent(agc_dp_t v);

/* time of the permanent CSM state vector, centiseconds, scale 2^28 (TETCSM) */
agc_dp_t     agc_erase_tetcsm(void);
void         agc_erase_set_tetcsm(agc_dp_t v);

/* time of the permanent LM state vector, centiseconds, scale 2^28 (TETLEM) */
agc_dp_t     agc_erase_tetlem(void);
void         agc_erase_set_tetlem(agc_dp_t v);

/* this vehicle's position, meters, scale 2^29 (earth) or 2^27 (moon) (RN,
 * B(6)PRM) */
agc_vec_t    agc_erase_rn(void);
void         agc_erase_set_rn(agc_vec_t v);

/* this vehicle's velocity, m/cs, scale 2^7 (earth) or 2^5 (moon) (VN, B(6)PRM) */
agc_vec_t    agc_erase_vn(void);
void         agc_erase_set_vn(agc_vec_t v);

/* permanent CSM position, meters, scale 2^29 earth / 2^27 moon (RRECTCSM) */
agc_vec_t    agc_erase_rrectcsm(void);
void         agc_erase_set_rrectcsm(agc_vec_t v);

/* permanent CSM velocity, m/cs, scale 2^7 earth / 2^5 moon (VRECTCSM) */
agc_vec_t    agc_erase_vrectcsm(void);
void         agc_erase_set_vrectcsm(agc_vec_t v);

/* permanent LM position, meters, scale 2^29 earth / 2^27 moon (RRECTLEM) */
agc_vec_t    agc_erase_rrectlem(void);
void         agc_erase_set_rrectlem(agc_vec_t v);

/* permanent LM velocity, m/cs, scale 2^7 earth / 2^5 moon (VRECTLEM) */
agc_vec_t    agc_erase_vrectlem(void);
void         agc_erase_set_vrectlem(agc_vec_t v);

/* accumulated PIPA delta-V, stable-member axes (DELV; SERVICER207: scaled like
 * PIPAX,Y,Z via KPIP1 to 2^4 m/cs) */
agc_vec_t    agc_erase_delv(void);
void         agc_erase_set_delv(agc_vec_t v);

/* delta-V vector in local-vertical coordinates (DELVSLV, I(6)); P40 uses it as
 * the VG input */
agc_vec_t    agc_erase_delvslv(void);
void         agc_erase_set_delvslv(agc_vec_t v);

/* velocity to be gained at TIG, m/cs, scale 2^7 (VGTIG, P40-P47: 'VG IGNITION
 * SCALED AT 2(+7) M/CS') */
agc_vec_t    agc_erase_vgtig(void);
void         agc_erase_set_vgtig(agc_vec_t v);

/* reference-to-stable-member matrix, row-major, DP elements, scale 2^1
 * (REFSMMAT, I(18D)PRM; 'REFSMMAT SCALE OF B1') */
agc_mat_t    agc_erase_refsmmat(void);
void         agc_erase_set_refsmmat(agc_mat_t v);

/* time of the ephemeris epoch (TEPHEM, 3 words, TP; P11 sets it at liftoff).
 * Scale: centiseconds, see LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES */
agc_tp_t     agc_erase_tephem(void);
void         agc_erase_set_tephem(agc_tp_t v);

/* CSM mass, kg, scale 2^16 (CSMMASS 'B+16 KILOGRAMS'; DSP noun 47 R1) */
agc_sp_t     agc_erase_csmmass(void);
void         agc_erase_set_csmmass(agc_sp_t v);

/* LM mass, kg, scale 2^16 (LEMMASS; DSP noun 47 R2) */
agc_sp_t     agc_erase_lemmass(void);
void         agc_erase_set_lemmass(agc_sp_t v);

/* PIPA X pulse count since last read (register PIPAX, address 037); 1 pulse =
 * 5.85 cm/s in SERVICER207 */
agc_sp_t     agc_erase_pipax(void);
void         agc_erase_set_pipax(agc_sp_t v);

/* PIPA Y pulse count (register PIPAY) */
agc_sp_t     agc_erase_pipay(void);
void         agc_erase_set_pipay(agc_sp_t v);

/* PIPA Z pulse count (register PIPAZ) */
agc_sp_t     agc_erase_pipaz(void);
void         agc_erase_set_pipaz(agc_sp_t v);

/* CDU X (outer gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution
 * (register CDUX, 032) */
uint16_t     agc_erase_cdux(void);
void         agc_erase_set_cdux(uint16_t v);

/* CDU Y (inner gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution
 * (CDUY) */
uint16_t     agc_erase_cduy(void);
void         agc_erase_set_cduy(uint16_t v);

/* CDU Z (middle gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution
 * (CDUZ) */
uint16_t     agc_erase_cduz(void);
void         agc_erase_set_cduz(uint16_t v);

/* CSM optics trunnion CDU raw word (CDUT / OPTY); LM: rendezvous radar
 * trunnion */
uint16_t     agc_erase_cdut(void);
void         agc_erase_set_cdut(uint16_t v);

/* CSM optics shaft CDU raw word (CDUS / OPTX); LM: rendezvous radar shaft */
uint16_t     agc_erase_cdus(void);
void         agc_erase_set_cdus(uint16_t v);

/* mark status bit word (MARKSTAT, B(1)PRM; SXTMARK/R21/R22/P20/P23) */
uint16_t     agc_erase_markstat(void);
void         agc_erase_set_markstat(uint16_t v);

/* last-sampled channel 30 bits, T4RUPT IMUMON (IMODES30, B(1)PRM) */
uint16_t     agc_erase_imodes30(void);
void         agc_erase_set_imodes30(uint16_t v);

/* last-sampled channel 33 / DAP-enable bits, T4RUPT C33TEST (IMODES33,
 * B(1)PRM) */
uint16_t     agc_erase_imodes33(void);
void         agc_erase_set_imodes33(uint16_t v);

/* DAP configuration word 1 (DAPDATR1, DSP noun 46 R1) */
uint16_t     agc_erase_dapdatr1(void);
void         agc_erase_set_dapdatr1(uint16_t v);

/* DAP configuration word 2 (DAPDATR2, DSP noun 46 R2; Comanche only, storage
 * exists in both builds) */
uint16_t     agc_erase_dapdatr2(void);
void         agc_erase_set_dapdatr2(uint16_t v);

/* display panel relay buffer DSPTAB +0..+10D, +11D is the C/S lights word
 * (15-bit relay words) Index 0..11. */
uint16_t     agc_erase_dsptab(int index);
agc_status_t agc_erase_set_dsptab(int index, uint16_t v);

/* three alarm code registers FAILREG +0..+2 (octal alarm codes) Index 0..2. */
uint16_t     agc_erase_failreg(int index);
agc_status_t agc_erase_set_failreg(int index, uint16_t v);

/* desired gimbal angles THETAD +0..+2 (CPHI/CTHETA/CPSI), raw 15-bit CDU-style
 * words, 2^15 = 1 revolution Index 0..2. */
uint16_t     agc_erase_thetad(int index);
agc_status_t agc_erase_set_thetad(int index, uint16_t v);

#endif /* AGC_STATE_H */
