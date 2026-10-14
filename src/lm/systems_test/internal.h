/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface between the files of the LM P07 systems test
 *   (src/lm/systems_test): the rope constants, the word offsets of the
 *   test erasables from W, the step and entry ids the jobs resume at,
 *   and the functions one file of the test calls in another.
 *
 * AGC SOURCE:
 *   Luminary099 IMU_PERFORMANCE_TEST_2.agc pp.373-381,
 *   IMU_PERFORMANCE_TESTS_4.agc pp.382-389, ERASABLE_ASSIGNMENTS.agc
 *   pp.127-128
 *
 * REQUIRES:
 *   the agc/ public headers included below
 *
 * GLOBAL VARIABLES:
 *   Name          Type      Scale          Description
 *   ------------  --------  -------------  -------------------------
 *   none (the test erasables are reached through st_words())
 **********************************************************************/
#ifndef LM_SYSTEMS_TEST_INTERNAL_H
#define LM_SYSTEMS_TEST_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/interp.h"
#include "agc/core/interp_words.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/exec/exec.h"

#define POSMAX            037777u                    /* POSMAX */
#define WORD15            077777u                    /* one AGC word, 15 bits */
#define ZEROING_MAX       0400                       /* longest block cleared (one EBANK)     */
#define SYSTEMS_TEST_MODE 07                         /* NEWMODEX, MM 07                        */
#define MODREG_NONE       ((agc_sp_t) -1)            /* CS ZERO, NEWMODEA: -0 (exec's code)   */
#define ONEDPP            ((agc_dp_t) 1)             /* ONEDPP: 2^-28 */
#define PRIO20            ((agc_prio_t) 020)         /* PIPJOBB, ALFLT                  */
#define ALARM_BAD_IMU     ((agc_alarm_code_t) 01601) /* OCT1601: bad IMU moding */

/* ECADRs (Luminary099 symtab): the test erasables and the gain tables.    */
#define EC_W          02400u /* W = AZIMUTH, E5,1400                       */
#define EC_TRANSM1    02000u /* TRANSM1 = WRENDPOS, E4,1400                 */
#define EC_ALFDK      02022u /* ALFDK = TRANSM1 +18D                        */
#define EC_ERASABLE   03777u /* last erasable ECADR                         */
#define ALFDK_WORDS   144    /* 12 stages of 12 words (ALKCG)               */
#define TRANSM1_WORDS 18     /* 3 x 3 DP                                     */

/* Word offsets from W (ERASABLE_ASSIGNMENTS p.127-128).                   */
#define W_AZIMUTH     0   /* AZIMUTH, DP rev                           */
#define W_LATITUDE    2   /* LATITUDE, DP rev                          */
#define W_ERVECTOR    4   /* ERVECTOR, earth rate (rev/cs) x 2^k        */
#define W_LENGTHOT    10  /* LENGTHOT, passes left                      */
#define W_LOSVEC      11  /* LOSVEC (troubleshooting CDU words)         */
#define W_NDXCTR      12  /* NDXCTR, 1 in gimbal lock (= LOSVEC +1)      */
#define W_PIPINDEX    13  /* PIPINDEX, PIPA under test 0..2              */
#define W_POSITON     14  /* POSITON, test table position                */
#define W_QPLACE      15  /* QPLACE: SHOW / CHECKG return step           */
#define W_QPLACES     16  /* QPLACES: EARTHR* return step                */
#define W_SOUTHDR     17  /* SOUTHDR, 7 south drift results              */
#define W_TEMPTIME    24  /* TEMPTIME, cs 2^28                           */
#define W_TMARK       26  /* TMARK, cs 2^28                              */
#define W_WANGO       28  /* AINLA = GENPL: WANGO, sin LAT               */
#define W_WANGI       30  /* WANGI, -cos LAT                             */
#define W_TORQNDX     32  /* TORQNDX (= WANGT), POSMAX in vertical drift */
#define W_DRIFTT      34  /* DRIFTT                                      */
#define W_ALX1S       36  /* ALX1S, gain schedule index                  */
#define W_CMPX1       37  /* CMPX1, ALKLP index (+-1)                    */
#define W_ALK         38  /* ALK, filter gains                           */
#define W_INTY        56  /* INTY, INTZ, ANG.. filter states             */
#define W_DATAPL      58  /* DATAPL (= ANGZ): PIP test samples           */
#define W_ANGX        64  /* ANGX (ANGY -2, ANGZ -6), drift angles       */
#define W_DRIFTO      66  /* DRIFTO, vertical drift                      */
#define W_DRIFTI      68  /* DRIFTI, south drift                         */
#define W_VLAUN       72  /* VLAUN                                       */
#define W_ACCWD       74  /* ACCWD                                       */
#define W_POSNV       80  /* POSNV                                       */
#define W_DPIPAY      82  /* DPIPAY (DPIPAZ +4), level PIPA increments   */
#define W_ALTIM       88  /* ALTIM, gain counter                         */
#define W_ALTIMS      89  /* ALTIMS                                      */
#define W_ALDK        90  /* ALDK, gain increments                       */
#define W_RESULTCT    95  /* RESULTCT (= ALDK +5), PIP sample index     */
#define W_DELM        104 /* DELM, measurements                          */
#define W_GEOCOMPS    114 /* GEOCOMPS, 0 in the LM                       */
#define W_ZERONDX     121 /* ZERONDX, CHECKG sample / ZEROING count      */
#define W_1SECXT      122 /* 1SECXT, sample period, cs                   */
#define W_PERFDLAY    124 /* PERFDLAY, DP delay before ESTIMS, cs        */
#define W_OVFLOWCK    126 /* OVFLOWCK, set: overflow in the drift test   */
#define W_WORDS       128 /* W .. AINLA +99D                             */
#define ESTIMS_ZEROED 78  /* 77DECML + 1 words from ALX1S -1            */

/* Rope constants (IMU_PERFORMANCE_TESTS_4 p.388-389, CONTROLLED_CONSTANTS),
 * DP = high x 2^14 + low as assembled.                                     */
#define DP_OF(hi, lo) ((agc_dp_t) (hi) * 040000 + (agc_dp_t) (lo))
#define PIPASC        DP_OF(2139, 1205)   /* 2DEC .13055869                 */
#define VELSC         DP_OF(-8556, -4822) /* 2DEC -.52223476 (512/980.402)  */
#define ALSK0         DP_OF(2839, 5503)   /* ALSK 2DEC .17329931            */
#define ALSK1         DP_OF(-136, -14205) /* ALSK +2 2DEC -.00835370        */
#define GEORGEJ       DP_OF(10430, 6198)  /* 2DEC .63661977 (2/pi)          */
#define DEC585        DP_OF(3200, 0)      /* OCT 06200, 3200 B+14           */
#define OMEG_MS       DP_OF(3987, 11626)  /* OMEG/MS 2DEC .24339048         */
#define DEC17         17                  /* overflows of the PIPA counter  */
#define DEC58         58                  /* PIP test passes of 5.12 s      */
#define DEC3990       3990                /* about 1 h of vertical drift    */
#define BIT5          020                 /* ERCOMP offset (PON2 / PON4)    */
#define PIP_PERIOD_CS 01000u              /* BIT10: 5.12 s                  */
#define PIP_START_CS  2u                  /* CAF TWO, TWIDDLE PIPATASK      */

/* QPLACE / QPLACES: where the job goes on (see DEVIATION above).          */
typedef enum st_step
{
    STEP_NONE = 0,
    STEP_PIPACHK,        /* TORQUE's SHOW returns into PIPACHK      */
    STEP_PIPACHK_ARM,    /* PIPACHK after EARTHR*                   */
    STEP_PIPATASK_START, /* PIPACHK after CHECKG                    */
    STEP_PIPJOBB_CATCH,  /* PIPJOBB after EARTHR*                   */
    STEP_PIPJOBB_RESULT, /* PIPJOBB after CHECKG                    */
    STEP_VERTDRFT,       /* PIPJOBB's SHOW returns into VERTDRFT    */
    STEP_GUESS1,         /* PON after EARTHR*                       */
    STEP_ENDTEST1,       /* VALMIS's SHOW returns into ENDTEST1     */
    STEP_ENDOFJOB,       /* SLEEPIE after EARTHR*                   */
    STEP_COUNT
} st_step_t;

/* The job or task of the test that is starting (st_enter).                */
typedef enum st_entry
{
    ENTRY_REDO = 0,
    ENTRY_IMU_ZEROED,
    ENTRY_SITE_DISPLAY,
    ENTRY_SITE_LOADED,
    ENTRY_END,
    ENTRY_COARSE_ALIGNED,
    ENTRY_FINE_ALIGNED,
    ENTRY_DRIFT_START,
    ENTRY_RESUME,
    ENTRY_EARTH_RATE_RETRY,
    ENTRY_EARTH_RATE_DONE,
    ENTRY_PIP_CATCH,
    ENTRY_PIP_RESULT,
    ENTRY_FILTER,
    ENTRY_TORQUE_RETRY,
    ENTRY_TORQUED,
    ENTRY_GIMBALS_ZEROED,
    ENTRY_SHOW,
    ENTRY_DRIFT_WAKE,
    ENTRY_PIP_TIMER,
    ENTRY_SAMPLE
} st_entry_t;

/* CCS: which of the four branches, and the diminished magnitude in A.     */
typedef struct st_ccs
{
    int      branch; /* 0 > 0, 1 +0, 2 < 0, 3 -0                  */
    uint16_t a;      /* |x| - 1 (0 for +-0)                       */
} st_ccs_t;

/* ---- test_erasables.c: the test erasables as AGC words, CCS, the PIPA under test */
uint16_t      *st_words(void); /* W .. AINLA +99D (W_WORDS words) */
agc_sp_t       w_sp(int off);
void           w_set_sp(int off, agc_sp_t v);
agc_dp_t       w_dp(int off);
void           w_set_dp(int off, agc_dp_t v);
agc_dp_words_t w_words(int off);
void           w_store(int off, agc_tp_words_t m);
st_ccs_t       ccs(uint16_t x);
agc_status_t   pipa_index(int *axis);
agc_sp_t       pipa_get(int axis);
void           pipa_set(int axis, agc_sp_t v);
agc_status_t   image_words(int ecadr, agc_dp_words_t *v);
void           st_fetch(void);
agc_status_t   st_flush(void);

/* ---- imu_performance_test.c: IMU_PERFORMANCE_TEST_2 steps and subroutines */
agc_status_t agc_systems_test_zeroing(uint16_t *block, int n);
agc_status_t endtest1(void);
agc_status_t imu_stall(agc_job_id_t wake_as, bool *good);
void         imu_stall_outcome(bool *good);
void         erthrvse(agc_interp_t *ip);
agc_status_t earthr_stall(agc_status_t pulse);
agc_status_t earthr_star(st_step_t next);
agc_status_t show1(agc_job_ctx_t *ctx);
agc_status_t show(st_step_t next, agc_job_ctx_t *ctx);
agc_status_t site_display(agc_job_ctx_t *ctx);
agc_status_t imuback(agc_job_ctx_t *ctx);
agc_status_t redo(agc_job_ctx_t *ctx);
agc_status_t fine_aligned(void);
agc_status_t pipachk_arm(void);
agc_status_t pipachk(void);
agc_status_t coarse_aligned(void);
agc_status_t posgmbl(void);
agc_status_t pipjobb_catch(void);
agc_status_t guess1(void);
agc_status_t vertdrft(agc_sp_t lengthot);
agc_status_t pipjobb_result(agc_job_ctx_t *ctx);
agc_status_t torque(agc_job_ctx_t *ctx);
agc_status_t pip_catch(agc_job_ctx_t *ctx);
agc_status_t pip_timer(void);

/* ---- drift_filter.c: IMU_PERFORMANCE_TESTS_4 steps */
void         somerr2(void);
agc_status_t estims(void);
agc_status_t geostrt4(agc_job_ctx_t *ctx);
agc_status_t setuper1_stall(agc_status_t pulse, agc_job_ctx_t *ctx);
agc_status_t alflt(agc_job_ctx_t *ctx);
agc_status_t alloop(void);

/* ---- job_entries.c: the step dispatch */
agc_status_t run_step(st_step_t step, agc_job_ctx_t *ctx);

#endif /* LM_SYSTEMS_TEST_INTERNAL_H */
