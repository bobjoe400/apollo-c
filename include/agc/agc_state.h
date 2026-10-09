/*
 * agc_state.h -- shared erasable memory and flagwords.
 *
 * The state-flags agent owns src/core/agc_state.c, agc_flags_csm.h and
 * agc_flags_lm.h, and may extend THIS header with further accessors for
 * erasables that are shared by more than one module. Module-private
 * erasables stay static inside their module.
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
void          agc_state_init(agc_vehicle_t v);
agc_vehicle_t agc_state_vehicle(void);

/* ---- time: TIME1/TIME2 (centiseconds, DP), TIME3..TIME6 -------------- */
uint32_t agc_time_now_cs(void);             /* C(TIME2,TIME1) as 28-bit  */
void     agc_time_set_cs(uint32_t t_cs);
void     agc_time_advance_cs(uint16_t dt_cs);

/* ---- flagwords: FLAGWRD0..FLAGWRD11, bit names in agc_flags_{csm,lm}.h  */
typedef uint16_t agc_flag_id_t;             /* (flagword << 4) | bitindex  */
#define AGC_FLAGWORDS 12
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

typedef enum agc_vehicle_sv { AGC_SV_THIS = 0, AGC_SV_OTHER = 1 } agc_vehicle_sv_t;
agc_status_t agc_sv_get(agc_vehicle_sv_t which, agc_state_vector_t *out); /* RN,VN,PIPTIME / other */
agc_status_t agc_sv_set(agc_vehicle_sv_t which, const agc_state_vector_t *in);

/* ---- a few universally shared erasables (extend here, not elsewhere) --- */
agc_dp_t     agc_erase_tig(void);           /* TIG  time of ignition cs 2^28 */
void         agc_erase_set_tig(agc_dp_t t);
agc_sp_t     agc_erase_modreg(void);        /* MODREG major mode (program)    */
void         agc_erase_set_modreg(agc_sp_t mode);

#endif /* AGC_STATE_H */
