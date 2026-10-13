/**********************************************************************
 * FILE NAME: systems_test.c
 *
 * PURPOSE:
 *   P07 SYSTEMS TEST in the LM (V92): the IMU performance tests -- site
 *   latitude / azimuth, coarse and fine align, the gyro drift filter, the
 *   PIPA scale test, the vertical drift test -- and the subroutines other
 *   code may use (ZEROING, FINETIME, OVERFFIX).
 *
 * AGC SOURCE:
 *   Luminary099 SYSTEM_TEST_STANDARD_LEAD_INS.agc pp.370-372,
 *   IMU_PERFORMANCE_TEST_2.agc pp.373-381, IMU_PERFORMANCE_TESTS_4.agc
 *   pp.382-389 (Comanche055 falls into IMUTEST, src/csm/prelaunch_align.c)
 *
 * EXTERNAL VARIABLES:
 *   (AGC erasables reached through accessors or by ECADR)
 *   Name          Type          I/O  Scale     Description
 *   ------------  ------------  ---  --------  ----------------------------
 *   W .. AINLA+99 uint16_t[]    I/O  words     the test erasables (ECADR image)
 *   TRANSM1,ALFDK uint16_t[]    I    words     filter matrix and gains (E4)
 *   XNB, XSM      agc_mat_t     I/O  2^1       navigation base, stable member
 *   OGC           agc_dp_t[3]   O    rev 2^0   gyro torquing angles
 *   THETAD        uint16_t[3]   O    CDU       coarse align angles
 *   DELV, PIPAs   agc_vec_t     I/O  pulses    PIPA counts
 *   DSPTEM1/2     uint16_t      O    words     V06N41, V06N98 registers
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_imu_zero/coarse/  IMUZERO, IMUCOARS, IMUFINE, IMUSTALL,
 *    fine/stall           IMUPULSE (imu lane)
 *   agc_imu_calcga        CALCGA
 *   agc_dsky_display      GOFLASH V06N41 / V06N98
 *   agc_mpac_*            the interpreter on AGC words (interp_words.h)
 *   agc_waitlist_*        TWIDDLE, LONGCALL
 *   agc_exec_*            FINDVAC, JOBSLEEP, JOBWAKE, CHANG1, VARALARM
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   VARALARM 01601 (bad IMU moding, SOMERR2); CCSHOLE when the first PIPA
 *   count is +0; the test ends with ENDTEST1 (V34) or SOMERR2.
 *
 * NOTES:
 *   The LM IMU performance test.  REDO sets MM 07, zeroes the ISS
 *   (GEOIMUTT) and asks for the site latitude and the azimuth of the
 *   test table (V06N41).  The platform is then coarse aligned to the
 *   navigation base (POSGMBL); in gimbal lock only a PIPA test is made
 *   (PIPACHK..PIPJOBB: count the pulses of one PIPA for 58 x 5.12 s,
 *   V06N98 shows the rate). Otherwise it is fine aligned and, after
 *   PERFDLAY, the drift filter runs once a second (ESTIMS, ALLOOP,
 *   ALFLT) until LENGTHOT passes are done; the drift angles then torque
 *   the gyros (SETUPER1), the south drift is shown (TORQUE), the PIPA
 *   test follows, then the vertical drift test (VERTDRFT, ESTIMS again
 *   with TORQNDX = POSMAX) ends with its result (VALMIS) and ENDTEST1.
 *
 *   What the rope lacks: the drift filter's calculation proper.
 *   PERFERAS (p.387) does TC ATIGINC -- "GOTO ERASABLE TO CALCULATE" --
 *   into a program at E7,1400 that "MUST BE LOADED BEFORE ANY ATTEMPT IS
 *   MADE TO RUN THE IMU PERFORMANCE TEST"; it is not in Luminary099.
 *   The filter's gains are erasable too: the transition matrix TRANSM1
 *   and the 144-word gain schedule ALFDK (E4,1400 on; ALKCG loads 12
 *   words per stage), and so are the test parameters PERFDLAY, 1SECXT,
 *   LENGTHOT, POSITON and PIPINDEX (the ground loads them with V21 N01).
 *   Everything in fixed memory is translated; the erasable words are
 *   read by address (src/core/ecadr.c), so a V21 N01 load reaches them.
 *   DEVIATION (ATIGINC): the erasable program is not run (nothing in C
 *   can execute erasable memory); PERFERAS goes on as if it had
 *   returned. DEVIATION (storage): the system test erasables (W .. AINLA
 *   +99D, E5,1400 on, 128 words) are kept as AGC words, fetched from the
 *   ECADR image when a job or task of the test starts and written back
 *   when it ends, because the code depends on their layout (DATAPL
 *   overlays ANGZ, ALKCG stores ALTIM and ALTIMS with ALDK, LOSVEC +1 is
 *   NDXCTR).  The AGC overlays the rendezvous W matrix with them; the C
 *   W matrix is separate storage.  The earth-rate commands ERCOMP live
 *   in the IMU lane's ERCOMP gyro buffer. DEVIATION (returns): QPLACE
 *   and QPLACES hold a step number (st_step_t) instead of a return
 *   address; a job resumes at the step. DEVIATION (CHECKG): the AGC
 *   catches a PIPA pulse between two reads a few microseconds apart; in
 *   C the PIPA can only change between two runs of the job, so the job
 *   yields (CHANG1) between the reads.
 *
 *   Also translated: ZEROING, FINETIME, OVERFFIX, ERTHRVSE / EARTHR (the
 *   same text as the CM prelaunch ones, src/csm/prelaunch_align.c).  The
 *   erasable-memory program helpers in SYSTEM_TEST_STANDARD_LEAD_INS
 *   (E/BKCALL, E/CALL, E/SWITCH, E/JOBWAK) exist only so that programs
 *   loaded into erasable memory can make bank calls; they vanish in C
 *   like the inter-bank routines (src/io/service/interbank.c).  SOMEERRR
 *   (alarm 1600) has no caller in fixed memory (the erasable program's).
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/interp_words.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"

#define POSMAX            037777u                    /* POSMAX */
#define WORD15            077777u                    /* one AGC word, 15 bits */
#define ZEROING_MAX       0400                       /* longest block cleared (one EBANK)     */
#define FINETIME_TRIES    3                          /* READ, re-READ, and the POSMAX retry   */
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

/* INTVAL +2 .. +7: ALX1S, CMPX1, ALK, ALK +2 (SOUPLY) as words.           */
static const uint16_t intval_words[6] = {000220u, 077776u, 035730u, 000035u, 010317u, 017550u};

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

static uint16_t w[W_WORDS];      /* W .. AINLA +99D as AGC words           */
static agc_dp_t perferas_vac[6]; /* VAC 10D..20D: sin (2^-3) and cos (2^0) of ANGX..ANGZ */

/* ---- AGC words ---------------------------------------------------------- */

/**********************************************************************
 * AGC: none -- A ones'-complement word as a signed value (-0 reads as 0).
 *
 * ARGUMENTS:
 *   off  int  I   word offset from W
 *
 * RETURNS: the word's value
 **********************************************************************/
static agc_sp_t w_sp(int off)
{
    uint16_t x = w[off]; /* the word as held */

    return ((agc_sp_t) (((x & 040000u) != 0u) ? -(int) (WORD15 - x) : (int) x)); /* negative: -(77777 - x) */
}

/**********************************************************************
 * AGC: none -- store a value as the AGC word at W +off
 *
 * ARGUMENTS:
 *   off  int       I   word offset from W
 *   v    agc_sp_t  I   value, |v| <= 16383 (-0 not stored)
 **********************************************************************/
static void w_set_sp(int off, agc_sp_t v)
{
    w[off] = (uint16_t) ((v >= 0) ? (unsigned int) v : (unsigned int) ((int) WORD15 + v)); /* negative: 77777 + v */
}

/**********************************************************************
 * AGC: none -- the DP value held at W +off
 *
 * ARGUMENTS:
 *   off  int  I   offset of the major word from W
 *
 * RETURNS: the DP value of w[off], w[off +1] (2^-14 per minor unit)
 **********************************************************************/
static agc_dp_t w_dp(int off)
{
    return (agc_dp_pack(w_sp(off), w_sp(off + 1)));
}

/**********************************************************************
 * AGC: none -- store a DP at W +off, W +off +1
 *
 * ARGUMENTS:
 *   off  int       I   offset of the major word from W
 *   v    agc_dp_t  I   DP value, stored as sign-agreeing words
 **********************************************************************/
static void w_set_dp(int off, agc_dp_t v)
{
    w_set_sp(off, agc_dp_high(v));
    w_set_sp(off + 1, agc_dp_low(v));
}

/**********************************************************************
 * AGC: none -- the DP at W +off as the two words the AGC holds
 *
 * ARGUMENTS:
 *   off  int  I   offset of the major word from W
 *
 * RETURNS: the two words as held (signs may disagree)
 **********************************************************************/
static agc_dp_words_t w_words(int off)
{
    agc_dp_words_t x = {w_sp(off), w_sp(off + 1)};

    return (x);
}

/**********************************************************************
 * AGC: none -- STORE: MPAC, MPAC +1 into w[off], w[off +1] as held.
 *
 * ARGUMENTS:
 *   off  int             I   offset of the major word from W
 *   m    agc_tp_words_t  I   MPAC; MPAC, MPAC +1 are stored
 **********************************************************************/
static void w_store(int off, agc_tp_words_t m)
{
    agc_dp_words_t x = agc_mpac_store(m); /* MPAC, MPAC +1 as held */

    w_set_sp(off, x.hi);
    w_set_sp(off + 1, x.lo);
}

/**********************************************************************
 * AGC: none -- CCS on a word: the branch and what the AGC leaves in A
 *
 * ARGUMENTS:
 *   x  uint16_t  I   the word CCS tests
 *
 * RETURNS: the branch (0 > 0, 1 +0, 2 < 0, 3 -0) and A = |x| - 1
 **********************************************************************/
static st_ccs_t ccs(uint16_t x)
{
    st_ccs_t c = {1, 0};

    if (x == WORD15)
    {
        c.branch = 3; /* -0 */
    }
    else if ((x & 040000u) != 0u)
    {
        c.branch = 2;
        c.a      = (uint16_t) (WORD15 - x - 1u);
    }
    else if (x != 0u)
    {
        c.branch = 0;
        c.a      = (uint16_t) (x - 1u);
    }

    return (c);
}

/**********************************************************************
 * AGC: none -- The PIPA counter PIPINDEX names (INDEX PIPINDEX, PIPAX).
 *
 * ARGUMENTS:
 *   axis  int *  O   PIPINDEX, 0..2 (X, Y, Z)
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG when PIPINDEX (ground loaded) is not
 *          0..2
 **********************************************************************/
static agc_status_t pipa_index(int *axis)
{
    *axis = (int) w_sp(W_PIPINDEX);

    if (!AGC_ASSERT(*axis >= 0 && *axis <= 2))
    {
        return (AGC_STATUS_BAD_ARG); /* PIPINDEX is a ground-loaded word */
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none -- the PIPA counter INDEX PIPINDEX names (PIPAX +axis)
 *
 * ARGUMENTS:
 *   axis  int  I   0..2 (X, Y, Z)
 *
 * RETURNS: that PIPA counter, pulses
 **********************************************************************/
static agc_sp_t pipa_get(int axis)
{
    agc_sp_t p = agc_erase_pipaz(); /* PIPAZ, pulses 2^0 */

    if (axis == 0)
    {
        p = agc_erase_pipax();
    }
    else if (axis == 1)
    {
        p = agc_erase_pipay();
    }

    return (p);
}

/**********************************************************************
 * AGC: none -- store into the PIPA counter INDEX PIPINDEX names
 *
 * ARGUMENTS:
 *   axis  int       I   0..2 (X, Y, Z)
 *   v     agc_sp_t  I   new count, pulses
 **********************************************************************/
static void pipa_set(int axis, agc_sp_t v)
{
    if (axis == 0)
    {
        agc_erase_set_pipax(v);
    }
    else if (axis == 1)
    {
        agc_erase_set_pipay(v);
    }
    else
    {
        agc_erase_set_pipaz(v);
    }
}

/**********************************************************************
 * AGC: none -- A DP as the two words the ECADR image holds (TRANSM1, ALFDK).
 *
 * ARGUMENTS:
 *   ecadr  int               I   ECADR of the major word (an index from
 *                                ground-loaded words)
 *   v      agc_dp_words_t *  O   the two words as stored
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG for an address outside erasable memory,
 *          or the image's read status
 **********************************************************************/
static agc_status_t image_words(int ecadr, agc_dp_words_t *v)
{
    uint16_t hi = 0; /* major word as read */
    uint16_t lo = 0; /* minor word as read */

    if (!AGC_ASSERT(ecadr >= 0 && ecadr < (int) EC_ERASABLE))
    {
        return (AGC_STATUS_BAD_ARG); /* an index from ground-loaded words */
    }

    agc_erase_read_ecadr((uint16_t) ecadr, &hi);
    agc_erase_read_ecadr((uint16_t) (ecadr + 1), &lo);
    hi    = (uint16_t) (hi & WORD15);
    lo    = (uint16_t) (lo & WORD15);
    v->hi = (agc_sp_t) (((hi & 040000u) != 0u) ? -(int) (WORD15 - hi) : (int) hi); /* ones' complement */
    v->lo = (agc_sp_t) (((lo & 040000u) != 0u) ? -(int) (WORD15 - lo) : (int) lo); /* ones' complement */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none -- Fetch / write back the test erasables (see DEVIATION above).
 *
 * RETURNS: none
 **********************************************************************/
static void st_fetch(void)
{
    for (int i = 0; i < W_WORDS; i++)
    {
        agc_erase_read_ecadr((uint16_t) (EC_W + (unsigned int) i), &w[i]);
        w[i] = (uint16_t) (w[i] & WORD15);
    }
}

/**********************************************************************
 * AGC: none -- write the test erasables back to the ECADR image
 *
 * RETURNS: AGC_STATUS_OK or the image's write status
 **********************************************************************/
static agc_status_t st_flush(void)
{
    agc_status_t st = AGC_STATUS_OK; /* write status */

    for (int i = 0; i < W_WORDS && st == AGC_STATUS_OK; i++)
    {
        st = agc_erase_write_ecadr((uint16_t) (EC_W + (unsigned int) i), w[i]);
    }

    return (st);
}

/**********************************************************************
 * ZEROING (IMU_PERFORMANCE_TEST_2 p.379): store +0 in n consecutive words
 * (the AGC takes the address in A and the count - 1 in ZERONDX1).
 *
 * ARGUMENTS:
 *   block  uint16_t *  I   the first word to clear
 *   n      int         I   number of words, 1..256
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG for a count outside 1..256
 **********************************************************************/
agc_status_t agc_systems_test_zeroing(uint16_t *block, int n)
{
    if (!AGC_ASSERT(n >= 1 && n <= ZEROING_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int i = 0; i < n; i++)
    {
        block[i] = 0;
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * FINETIME (p.380): LOSCALAR read twice so a carry between the reads is
 * seen; if it is POSMAX (about to carry into HISCALAR) read again.  Returns
 * with interrupts inhibited, like the AGC.  *t = HISCALAR, LOSCALAR as a DP
 * count of the scalar (1/1600 s per LOSCALAR count, scale as channel 3/4).
 * BUSY when LOSCALAR still reads POSMAX: the caller tries again later.
 *
 * ARGUMENTS:
 *   t  agc_dp_t *  O   HISCALAR, LOSCALAR as one DP count
 *
 * RETURNS: AGC_STATUS_OK, BUSY while LOSCALAR reads POSMAX
 **********************************************************************/
agc_status_t agc_systems_test_finetime(agc_dp_t *t)
{
    agc_inhint();
    uint16_t lo = POSMAX; /* LOSCALAR, scaler counts */

    for (int k = 0; k < FINETIME_TRIES && lo == POSMAX; k++)
    {
        lo             = (uint16_t) (agc_chan_read((int) AGC_CHAN_LOSCALAR) & POSMAX);
        uint16_t again = (uint16_t) (agc_chan_read((int) AGC_CHAN_LOSCALAR) & POSMAX); /* second read: carry seen? */

        if (again != lo)
        { /* RXOR, BZF               */
            lo = again;
        }
    }

    uint16_t hi = (uint16_t) (agc_chan_read((int) AGC_CHAN_HISCALAR) & POSMAX); /* HISCALAR, scaler counts x 2^14 */

    *t = agc_dp_pack((agc_sp_t) hi, (agc_sp_t) lo);

    return ((lo == POSMAX) ? AGC_STATUS_BUSY : AGC_STATUS_OK); /* POSMAX: about to carry */
}

/**********************************************************************
 * OVERFFIX (p.378): DAD DPPOSMAX, DAD ONEDPP -- add 2^28 to a time
 * difference that went negative across TIME2 overflow (wraps like DAD).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I   interpreter state (OVFIND)
 *   a   agc_dp_t        I   the negative time difference, cs 2^28
 *
 * RETURNS: a + 2^28, cs 2^28
 **********************************************************************/
agc_dp_t agc_systems_test_overffix(agc_interp_t *ip, agc_dp_t a)
{
    agc_dp_t s = agc_dp_add(ip, a, AGC_DP_MAX); /* running sum, cs 2^28 */

    s = agc_dp_add(ip, s, ONEDPP);

    return (s);
}

/**********************************************************************
 * ENDTEST1 (p.377): IMUSE down, MODREG -0 (NEWMODEA), ENDEXT.
 *
 * RETURNS: the NEWMODEA or ENDEXT status
 **********************************************************************/
static agc_status_t endtest1(void)
{
    agc_flag_clear(AGC_FLAG_IMUSE);
    agc_status_t st = agc_phase_newmode(MODREG_NONE); /* NEWMODEA status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_endmark());
}

/**********************************************************************
 * SOMERR2 (p.388): VARALARM 1601 (bad IMU moding), IMUSE down, ENDOFJOB.
 *
 * RETURNS: none
 **********************************************************************/
static void somerr2(void)
{
    agc_exec_varalarm(ALARM_BAD_IMU);
    agc_flag_clear(AGC_FLAG_IMUSE);
}

/**********************************************************************
 * ALIGNCOA / IMUSLLLG (p.378): IMUSTALL; bad -> SOMERR2.  *good: the mode
 * switch is over and good, go on now (else the job sleeps as wake_as).
 *
 * ARGUMENTS:
 *   wake_as  agc_job_id_t  I   the job that goes on after a stall that
 *                              sleeps
 *   good     bool *        O   true: the mode switch is over and good, go
 *                              on now
 *
 * RETURNS: AGC_STATUS_OK (asleep, good, or SOMERR2 done), or the IMUSTALL
 *          status
 **********************************************************************/
static agc_status_t imu_stall(agc_job_id_t wake_as, bool *good)
{
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING;       /* STALL return */
    agc_status_t           st  = agc_imu_stall(wake_as, &res); /* IMUSTALL status */

    *good = false;

    if ((st != AGC_STATUS_OK) || (res == AGC_IMU_STALL_SLEEPING))
    {
        return (st);
    }

    if (res == AGC_IMU_STALL_BAD)
    {
        somerr2();

        return (AGC_STATUS_OK);
    }

    *good = true;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none -- The woken side of IMUSLLLG: bad -> SOMERR2.
 *
 * ARGUMENTS:
 *   good  bool *  O   true when the stalled mode switch ended good
 *
 * RETURNS: none
 **********************************************************************/
static void imu_stall_outcome(bool *good)
{
    *good = (agc_imu_stall_outcome(AGC_IMU_DEV_IMU) == AGC_IMU_STALL_GOOD);

    if (!*good)
    {
        somerr2(); /* bad: SOMERR2 */
    }
}

/**********************************************************************
 * ERTHRVSE (p.380): ERVECTOR = (SIN LAT, -COS LAT, 0) OMEG/MS, TMARK =
 * LOADTIME, ERCOMP = 0.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I   interpreter state (OVFIND)
 **********************************************************************/
static void erthrvse(agc_interp_t *ip)
{
    const agc_dp_t zero[3] = {0, 0, 0};
    agc_dp_t       lat     = w_dp(W_LATITUDE);                                              /* LATITUDE, rev 2^0 */
    agc_vec_t      er      = agc_vec_make(agc_dp_sin(lat), agc_dp_neg(agc_dp_cos(lat)), 0); /* (SIN, -COS, 0) LAT, then x OMEG/MS (rev/cs 2^-23) */

    er = agc_vec_scale(ip, er, OMEG_MS);

    for (int i = 0; i < 3; i++)
    {
        w_set_dp(W_ERVECTOR + 2 * i, er.c[i]);
    }

    w_set_dp(W_TMARK, agc_rtb_loadtime());
    agc_imu_gyro_buf_set(AGC_IMU_GBUF_ERCOMP, zero); /* STORE ERCOMP */
}

/**********************************************************************
 * EARTHR (p.380): ERCOMP += XSM (ERVECTOR (TIME - TMARK) 2^9), TMARK =
 * TIME, PULSEIMU ERCOMP (on GYROBUSY the job sleeps as busy_wake).
 *
 * ARGUMENTS:
 *   ip         agc_interp_t *  I   interpreter state (OVFIND)
 *   busy_wake  agc_job_id_t    I   the job run again when the gyros are
 *                                  busy
 *
 * RETURNS: the PULSEIMU status (BUSY: asleep; ALARM: IMU caged), or the
 *          gyro buffer read status
 **********************************************************************/
static agc_status_t earthr(agc_interp_t *ip, agc_job_id_t busy_wake)
{
    agc_dp_t  ercomp[3];                                                                             /* ERCOMP gyro commands, rev 2^0 */
    agc_dp_t  temptime = agc_rtb_loadtime();                                                         /* TEMPTIME = LOADTIME, cs 2^28 */
    agc_dp_t  d        = agc_dp_sub(ip, temptime, w_dp(W_TMARK));                                    /* TIME - TMARK, cs 2^28 */
    agc_vec_t er       = agc_vec_make(w_dp(W_ERVECTOR), w_dp(W_ERVECTOR + 2), w_dp(W_ERVECTOR + 4)); /* ERVECTOR, rev/cs (scale of OMEG/MS) */

    w_set_dp(W_TEMPTIME, temptime);

    if (d < 0)
    {
        d = agc_systems_test_overffix(ip, d); /* BPL ERTHR, CALL OVERFFIX */
    }

    agc_vec_t v = agc_vec_scale(ip, er, agc_dp_shl(ip, d, 9)); /* ERTHR: SL 9D, VXSC: rev 2^0 */

    v = agc_mat_vec(ip, &agc_align_ram()->xsm, v);
    agc_imu_gyro_buf_get(AGC_IMU_GBUF_ERCOMP, ercomp);

    for (int i = 0; i < 3; i++)
    {
        ercomp[i] = agc_dp_add(ip, v.c[i], ercomp[i]); /* VAD ERCOMP */
    }

    agc_imu_gyro_buf_set(AGC_IMU_GBUF_ERCOMP, ercomp);
    w_set_dp(W_TMARK, temptime);

    return (agc_imu_pulse_buf(AGC_IMU_GBUF_ERCOMP, busy_wake)); /* PULSEIMU */
}

/**********************************************************************
 * EARTHR* (p.380) after PULSEIMU: IMUSLLLG, then TC QPLACES.
 *
 * ARGUMENTS:
 *   pulse  agc_status_t  I   the PULSEIMU status
 *
 * RETURNS: AGC_STATUS_OK (asleep, or the job goes on as the QPLACES
 *          return), or the failure
 **********************************************************************/
static agc_status_t earthr_stall(agc_status_t pulse)
{
    bool good = false; /* torquing over and good */

    if (pulse == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* GYROBUSY: asleep          */
    }

    if ((pulse != AGC_STATUS_OK) && (pulse != AGC_STATUS_ALARM))
    {
        return (pulse); /* ALARM: caged, the stall sees it */
    }

    agc_status_t st = imu_stall(AGC_JOB_SYSTEMS_TEST_EARTH_RATE_DONE, &good); /* IMUSLLLG status */

    if (!good)
    {
        return (st);
    }

    /* TC QPLACES as the job's next run (see DEVIATION: keeps the call
     * graph free of the cycle through the step dispatch).                 */
    return (agc_exec_job_yield(AGC_JOB_SYSTEMS_TEST_EARTH_RATE_DONE));
}

/**********************************************************************
 * EARTHR* (p.380): QPLACES = next, EARTHR, IMUSLLLG, TC QPLACES.
 *
 * ARGUMENTS:
 *   next  st_step_t  I   QPLACES, the step after the torquing
 *
 * RETURNS: the EARTHR / IMUSLLLG status
 **********************************************************************/
static agc_status_t earthr_star(st_step_t next)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    w_set_sp(W_QPLACES, (agc_sp_t) next);

    return (earthr_stall(earthr(&ip, AGC_JOB_SYSTEMS_TEST_EARTH_RATE_RETRY)));
}

/**********************************************************************
 * SHOW1 (p.381): DSPTEM2 +2 = POSITON, flash V06N98: V34 ENDTEST1, V33 TC
 * QPLACE, ENTER again.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the display request status
 **********************************************************************/
static agc_status_t show1(agc_job_ctx_t *ctx)
{
    agc_dsky_cont_t cont = {AGC_JOB_SYSTEMS_TEST_END, AGC_JOB_SYSTEMS_TEST_RESUME, AGC_JOB_SYSTEMS_TEST_SHOW};

    agc_dsky_set_dsptem(5, w[W_POSITON]);

    return (agc_dsky_display(ctx, AGC_DSKY_GOFLASH, agc_dsky_vn(6, 98), &cont));
}

/**********************************************************************
 * SHOW (p.380): QPLACE = the step V33 goes on at, SHOW1.
 *
 * ARGUMENTS:
 *   next  st_step_t        I   QPLACE, the step V33 goes on at
 *   ctx   agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the display request status
 **********************************************************************/
static agc_status_t show(st_step_t next, agc_job_ctx_t *ctx)
{
    w_set_sp(W_QPLACE, (agc_sp_t) next);

    return (show1(ctx));
}

/**********************************************************************
 * LATAZCHK (p.373): flash V06N41 (R1 azimuth, R2 latitude): V34 ENDTEST1,
 * V33 POSGMBL, ENTER (new data) again (TC -5).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the display request status
 **********************************************************************/
static agc_status_t site_display(agc_job_ctx_t *ctx)
{
    agc_dsky_cont_t cont = {AGC_JOB_SYSTEMS_TEST_END, AGC_JOB_SYSTEMS_TEST_SITE_LOADED, AGC_JOB_SYSTEMS_TEST_SITE_DISPLAY};

    return (agc_dsky_display(ctx, AGC_DSKY_GOFLASH, agc_dsky_vn(6, 41), &cont));
}

/**********************************************************************
 * IMUBACK / NBPOSPL / GUESS / LATAZCHK (p.373): clear the indices, XNB =
 * (1/2, 0, 0), YNB = ZNB = 0, show LATITUDE SL2 and AZIMUTH as a CDU.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the display request status
 **********************************************************************/
static agc_status_t imuback(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                         /* interpreter state */
    agc_mat_t   *nb = &agc_align_ram()->xnb; /* XNB, YNB, ZNB, half-unit */

    agc_interp_init(&ip);
    w[W_NDXCTR]      = 0;
    w[W_TORQNDX]     = 0;
    w[W_TORQNDX + 1] = 0;
    w[W_OVFLOWCK]    = 0;
    nb->row[0]       = agc_vec_make(AGC_DP_HALF, 0, 0);      /* ZEROING 18D, CA HALF */
    nb->row[1]       = agc_vec_zero();
    nb->row[2]       = agc_vec_zero();
    agc_dp_t lat2    = agc_dp_shl(&ip, w_dp(W_LATITUDE), 2); /* DLOAD SL2: rev 2^-2 */
    agc_dp_t az      = w_dp(W_AZIMUTH);                      /* AZIMUTH, rev 2^0 */

    agc_erase_set_dsptem1(1, agc_dp_high(lat2)); /* STODL DSPTEM1 +1 */
    agc_erase_set_dsptem1(2, agc_dp_low(lat2));
    agc_dsky_set_dsptem(0, agc_rtb_1sto2s(az));  /* RTB 1STO2S, TS DSPTEM1 */

    return (site_display(ctx));
}

/**********************************************************************
 * REDO / GEOIMUTT / IMUZERR (p.373, p.378): MM 07, IMUZERO, IMUSTALL.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the NEWMODEX, IMUZERO or IMUSTALL status, or that of IMUBACK
 **********************************************************************/
static agc_status_t redo(agc_job_ctx_t *ctx)
{
    bool         good = false;                                           /* IMUZERO over and good */
    agc_status_t st   = agc_phase_newmode((agc_sp_t) SYSTEMS_TEST_MODE); /* NEWMODEX 07 */

    if (st == AGC_STATUS_OK)
    {
        st = agc_imu_zero(); /* GEOIMUTT: IMUZERR */
    }

    if ((st != AGC_STATUS_OK) && (st != AGC_STATUS_BUSY))
    {
        return (st);
    }

    st = imu_stall(AGC_JOB_SYSTEMS_TEST_IMU_ZEROED, &good);

    if (!good)
    {
        return (st);
    }

    return (imuback(ctx));
}

/**********************************************************************
 * FINIMUDD done (p.374): LONGCALL PERFDLAY GOESTIMS, JOBSLEEP ESTICADR.
 *
 * RETURNS: AGC_STATUS_OK (asleep until GOESTIMS), or the LONGCALL status
 *          (PERFDLAY out of range)
 **********************************************************************/
static agc_status_t fine_aligned(void)
{
    agc_status_t st = agc_waitlist_longcall((uint32_t) w_dp(W_PERFDLAY), AGC_TASK_SYSTEMS_TEST_DRIFT_WAKE); /* LONGCALL status */

    if (st != AGC_STATUS_OK)
    {
        return (st); /* PERFDLAY (ground loaded) out of LONGCALL range */
    }

    agc_exec_job_sleep(AGC_JOB_SYSTEMS_TEST_DRIFT_START); /* CA ESTICADR, TC JOBSLEEP */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * CHECKG (p.378): QPLACE = next, ZERONDX = -PIPA, then CHECKG1 (yield).
 *
 * ARGUMENTS:
 *   next  st_step_t  I   QPLACE, the step after the pulse is caught
 *
 * RETURNS: the CHANG1 status, or BAD_ARG for a bad PIPINDEX
 **********************************************************************/
static agc_status_t checkg(st_step_t next)
{
    int          axis = 0;                 /* PIPINDEX */
    agc_status_t st   = pipa_index(&axis); /* PIPINDEX check */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    w_set_sp(W_QPLACE, (agc_sp_t) next);
    w_set_sp(W_ZERONDX, (agc_sp_t) -pipa_get(axis)); /* CS PIPAX, TS ZERONDX */

    return (agc_exec_job_yield(AGC_JOB_SYSTEMS_TEST_PIP_CATCH));
}

/**********************************************************************
 * PIPACHK +2 (p.375): 17 counter overflows, 58 passes, sample 1, PIPA and
 * DATAPL zeroed, CHECKG, then PIPATASK in 20 ms.
 *
 * RETURNS: the CHECKG status
 **********************************************************************/
static agc_status_t pipachk_arm(void)
{
    int          axis = 0;                 /* PIPINDEX */
    agc_status_t st   = pipa_index(&axis); /* PIPINDEX check */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    w_set_sp(W_DATAPL + 4, DEC17);
    w_set_sp(W_LENGTHOT, DEC58);
    w_set_sp(W_RESULTCT, 1);
    pipa_set(axis, 0);
    w[W_DATAPL] = 0;

    return (checkg(STEP_PIPATASK_START));
}

/**********************************************************************
 * PIPACHK (p.375): INDEX NDXCTR -- earth rate first unless in gimbal lock.
 *
 * RETURNS: the EARTHR* or CHECKG status
 **********************************************************************/
static agc_status_t pipachk(void)
{
    if (w[W_NDXCTR] == 0u)
    {
        return (earthr_star(STEP_PIPACHK_ARM));
    }

    return (pipachk_arm());
}

/**********************************************************************
 * CCS NDXCTR (p.374): gimbal lock -> PIPA test only, else FINIMUDD.
 *
 * RETURNS: the status of PIPACHK, IMUFINE, the stall or FINIMUDD's
 *          LONGCALL
 **********************************************************************/
static agc_status_t coarse_aligned(void)
{
    bool good = false; /* IMUFINE over and good */

    if (ccs(w[W_NDXCTR]).branch == 0)
    {
        return (pipachk()); /* CCS NDXCTR: PIPA test only */
    }

    agc_status_t st = agc_imu_fine(); /* FINIMUDD: IMUFINE */

    if ((st != AGC_STATUS_OK) && (st != AGC_STATUS_BUSY))
    {
        return (st);
    }

    st = imu_stall(AGC_JOB_SYSTEMS_TEST_FINE_ALIGNED, &good);

    if (!good)
    {
        return (st);
    }

    return (fine_aligned());
}

/**********************************************************************
 * LATAZCHK +12 / POSGMBL (pp.373-374): AZIMUTH and LATITUDE from the
 * display, WANGI / WANGO, YNB, ZNB from AZIMUTH; CALCGA, IMUCOARS; count
 * a gimbal lock in NDXCTR; IMUSLLLG.
 *
 * RETURNS: the CALCGA, IMUCOARS or stall status, or that of what follows
 **********************************************************************/
static agc_status_t posgmbl(void)
{
    agc_interp_t     ip;                                                                                                          /* interpreter state */
    agc_align_ram_t *ar   = agc_align_ram();                                                                                      /* XNB, XSM, half-unit */
    uint16_t         thetad[3];                                                                                                   /* THETAD, CDU patterns */
    bool             good = false;                                                                                                /* IMUCOARS over and good */
    const uint16_t   r2   = agc_dsky_dsptem(1);                                                                                   /* DSPTEM1 +1 */
    const agc_dp_t   lat  = agc_dp_shr(agc_dp_from_sp((agc_sp_t) (((r2 & 040000u) != 0u) ? -(int) (WORD15 - r2) : (int) r2)), 2); /* SLOAD, SR2: rev 2^0 */
    const agc_dp_t   az   = agc_rtb_cdulogic(agc_dsky_dsptem(0));                                                                 /* SLOAD DSPTEM1, CDULOGIC: rev 2^0 */

    agc_interp_init(&ip);
    w_set_dp(W_AZIMUTH, az);
    w_set_dp(W_LATITUDE, lat);
    w_set_dp(W_WANGI, agc_dp_shl(&ip, agc_dp_neg(agc_dp_cos(lat)), 1));    /* COS DCOMP SL1 */
    w_set_dp(W_WANGO, agc_dp_shl(&ip, agc_dp_sin(lat), 1));                /* SIN SL1       */
    ar->xnb.row[1].c[1] = agc_dp_sin(az);                                  /* YNB +2        */
    ar->xnb.row[2].c[2] = agc_dp_sin(az);                                  /* ZNB +4        */
    ar->xnb.row[1].c[2] = agc_dp_cos(az);                                  /* YNB +4        */
    ar->xnb.row[2].c[1] = agc_dp_neg(agc_dp_cos(az));                      /* ZNB +2        */
    agc_status_t st     = agc_imu_calcga(&ip, &ar->xnb, &ar->xsm, thetad); /* CALCGA, IMUCOARS, stall status */

    if (st == AGC_STATUS_OK)
    {
        st = agc_imu_coarse(thetad); /* IMUCOARS */
    }

    if ((st != AGC_STATUS_OK) && (st != AGC_STATUS_BUSY))
    {
        return (st);
    }

    if (agc_flag_test(AGC_FLAG_GLOKFAIL))
    {
        w[W_NDXCTR] = (uint16_t) ((w[W_NDXCTR] + 1u) & WORD15); /* INCR NDXCTR: gimbal lock */
    }

    agc_flag_clear(AGC_FLAG_GLOKFAIL);
    st = imu_stall(AGC_JOB_SYSTEMS_TEST_COARSE_ALIGNED, &good);

    if (!good)
    {
        return (st);
    }

    return (coarse_aligned());
} /* posgmbl */

/**********************************************************************
 * PIPJOBB +3 (p.376): end unless the passes are over; sample 5, CHECKG.
 *
 * RETURNS: AGC_STATUS_OK (ENDOFJOB while passes are left) or the CHECKG
 *          status
 **********************************************************************/
static agc_status_t pipjobb_catch(void)
{
    if ((w[W_LENGTHOT] & 040000u) == 0u && w[W_LENGTHOT] != 0u)
    {
        return (AGC_STATUS_OK); /* LENGTHOT > 0: ENDOFJOB  */
    }

    w_set_sp(W_RESULTCT, 5);

    return (checkg(STEP_PIPJOBB_RESULT));
}

/**********************************************************************
 * SLEEPIE +1 (p.388): CCS TORQNDX -- vertical drift: EARTHR*; ENDOFJOB.
 *
 * RETURNS: AGC_STATUS_OK (ENDOFJOB) or the EARTHR* status
 **********************************************************************/
static agc_status_t sleepie_tail(void)
{
    if (w_sp(W_TORQNDX) > 0)
    {
        return (earthr_star(STEP_ENDOFJOB));
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * ESTIMS (p.383): ALLOOP every 1SECXT, PIPAs and the filter zeroed, the
 * initial gains, GCOMPSW, DELV, GCOMP zero; ERTHRVSE unless vertical;
 * SLEEPIE +1.
 *
 * RETURNS: the TWIDDLE (1SECXT out of range) or ZEROING status, or that
 *          of SLEEPIE
 **********************************************************************/
static agc_status_t estims(void)
{
    const agc_dp_t zero[3] = {0, 0, 0};
    agc_interp_t   ip; /* interpreter state */

    agc_inhint();
    agc_status_t st = agc_waitlist_call(w[W_1SECXT], AGC_TASK_SYSTEMS_TEST_SAMPLE); /* TWIDDLE ALLOOP */

    agc_erase_set_pipax(0);
    agc_erase_set_pipay(0);
    agc_erase_set_pipaz(0);
    agc_relint();

    if (st == AGC_STATUS_OK)
    {
        st = agc_systems_test_zeroing(&w[W_ALX1S - 1], ESTIMS_ZEROED); /* ALXXXZ, 77DECML */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    for (int i = 0; i < 6; i++)
    {
        w[W_ALX1S + i] = intval_words[i]; /* STOVL ALX1S: INTVAL +2 */
    }

    st = agc_imu_set_gcompsw(0); /* STOVL GCOMPSW -1      */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_erase_set_delv(agc_vec_zero());
    agc_imu_gyro_buf_set(AGC_IMU_GBUF_GCOMP, zero);

    if (w_sp(W_TORQNDX) <= 0)
    {
        agc_interp_init(&ip);
        erthrvse(&ip); /* DCOMP BMN VERTSKIP */
    }

    return (sleepie_tail());
} /* estims */

/**********************************************************************
 * GUESS1 (p.377): only the south gyro gets earth rate; TORQNDX = POSMAX
 * (vertical drift), LOSVEC = CDUX, ESTIMS.
 *
 * RETURNS: the ESTIMS status
 **********************************************************************/
static agc_status_t guess1(void)
{
    w[W_ERVECTOR]     = 0; /* ERVECTOR, +1: south gyro only */
    w[W_ERVECTOR + 1] = 0;
    w[W_TORQNDX]      = POSMAX;
    w[W_TORQNDX + 1]  = POSMAX;
    w[W_LOSVEC]       = agc_erase_cdux();

    return (estims());
}

/**********************************************************************
 * VERTDRFT / PON2 / PON4 / PON (pp.376-377): LENGTHOT = 3990 (about 1 h),
 * DRIFTT = -SOUTHDR(POSITON - 2), offset ERCOMP to miss the PIP dead
 * zones (X up: Y -16, Z +16; Z up in position 4: Y -16, X +16), EARTHR*.
 *
 * ARGUMENTS:
 *   lengthot  agc_sp_t  I   LENGTHOT, 3990 passes (or 0 from a -0 NDXCTR)
 *
 * RETURNS: the EARTHR* status, BAD_ARG for a bad POSITON, or the gyro
 *          buffer status
 **********************************************************************/
static agc_status_t vertdrft(agc_sp_t lengthot)
{
    int      k = W_SOUTHDR - 2 + (int) w_sp(W_POSITON); /* SOUTHDR -2 +POSITON, word offset */
    agc_dp_t er[3];                                     /* ERCOMP gyro commands, rev 2^0 */

    if (!AGC_ASSERT(k >= 0 && k < W_WORDS))
    {
        return (AGC_STATUS_BAD_ARG); /* POSITON is a ground-loaded word */
    }

    w_set_sp(W_LENGTHOT, lengthot);
    w[W_DRIFTT] = (uint16_t) (WORD15 - w[k]); /* INDEX POSITON, CS SOUTHDR -2 */
    agc_imu_gyro_buf_get(AGC_IMU_GBUF_ERCOMP, er);

    /* CCS PIPINDEX: Z up in position 4 (PON4) or X up (PON2).            */
    int up = (w_sp(W_PIPINDEX) > 0) ? 0 : 2; /* ERCOMP component of the up axis */

    er[1]  = er[1] - (agc_dp_t) BIT5 * 040000;  /* CS BIT5, ADS ERCOMP +2 */
    er[up] = er[up] + (agc_dp_t) BIT5 * 040000; /* CA BIT5, ADS ERCOMP or +4 */
    agc_imu_gyro_buf_set(AGC_IMU_GBUF_ERCOMP, er);

    return (earthr_star(STEP_GUESS1)); /* PON */
}

/**********************************************************************
 * AINGOTN's rate (pp.376): DATAPL +4 = the pulse count since sample 1
 * (DCS DATAPL, DAS DATAPL +4: 17 counter overflows plus the difference),
 * time = DATAPL +6 - DATAPL +2 (OVERFFIX across a scaler overflow), rate
 * = count / time (DDV) x DEC585 (DMPR), word for word.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I   interpreter state (OVFIND, DVSIGN)
 *
 * RETURNS: MPAC: pulses per scaler count x DEC585, as DMPR leaves it
 **********************************************************************/
static agc_tp_words_t pip_rate(agc_interp_t *ip)
{
    static const agc_dp_words_t dec585   = {3200, 0};                                          /* OCT 06200, 0 */
    static const agc_dp_words_t dpposmax = {037777, 037777};                                   /* DPPOSMAX     */
    static const agc_dp_words_t onedpp   = {0, 1};                                             /* ONEDPP       */
    agc_dp_words_t              first    = w_words(W_DATAPL);                                  /* DATAPL, +1: sample 1 count */
    agc_dp_words_t              neg      = {(agc_sp_t) -first.hi, (agc_sp_t) -first.lo};
    int                         ovf      = 0;                                                  /* DAS overflow (counts are far below 1) */
    agc_dp_words_t              count    = agc_dp_words_das(w_words(W_DATAPL + 4), neg, &ovf); /* pulses since sample 1, 2^-28 per pulse */

    w_set_sp(W_DATAPL + 4, count.hi);
    w_set_sp(W_DATAPL + 5, count.lo);
    agc_tp_words_t t = agc_mpac_dsu(ip, agc_mpac_dload(w_words(W_DATAPL + 6)), w_words(W_DATAPL + 2)); /* scaler time, 2^-28 per count */

    if (agc_tp_from_words(t) < 0)
    {
        t = agc_mpac_dad(ip, agc_mpac_dad(ip, t, dpposmax), onedpp); /* CALL OVERFFIX */
    }

    agc_tp_words_t m = agc_mpac_ddv(ip, agc_mpac_dload(count), agc_mpac_store(t)); /* PDDL DDV */

    return (agc_mpac_dmpr(ip, m, dec585));
}

/**********************************************************************
 * AINGOTN (p.376): pulses / time x DEC585 into DSPTEM2; COAALIGN in gimbal
 * lock; SHOW, then VERTDRFT.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the CCSHOLE, IMUCOARS, stall or display status, or that of
 *          VERTDRFT
 **********************************************************************/
static agc_status_t pipjobb_result(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                          /* interpreter state */
    st_ccs_t     c    = ccs(w[W_DATAPL + 1]); /* CCS DATAPL +1: first count */
    bool         good = false;                /* IMUCOARS over and good */

    if (c.branch == 1)
    {
        return (agc_ccshole()); /* TC CCSHOLE */
    }

    if (c.branch >= 2)
    {
        w[W_DATAPL + 4] = (c.branch == 2) ? (uint16_t) (WORD15 - w[W_DATAPL + 4]) : 0u; /* CS / TS DATAPL +4 */
    }

    agc_interp_init(&ip);
    agc_dp_words_t rate = agc_dp_to_words(agc_tp_to_dp(agc_tp_from_words(pip_rate(&ip)))); /* RTB SGNAGREE */

    agc_erase_set_dsptem1(3, rate.hi); /* STORE DSPTEM2 */
    agc_erase_set_dsptem1(4, rate.lo);
    st_ccs_t n = ccs(w[W_NDXCTR]);     /* CCS NDXCTR */

    if (n.branch >= 2)
    {
        return (vertdrft((n.branch == 2) ? DEC3990 : 0)); /* CCS NDXCTR falls into VERTDRFT */
    }

    if (n.branch == 1)
    {
        return (show(STEP_VERTDRFT, ctx));
    }

    /* COAALIGN (p.378): THETAD = 0, IMUCOARS, IMUSTALL; then SHOW.        */
    const uint16_t zero[3] = {0, 0, 0};

    for (int i = 0; i < 3; i++)
    {
        agc_erase_set_thetad(i, 0);
    }

    agc_status_t st = agc_imu_coarse(zero); /* IMUCOARS, stall status */

    if ((st != AGC_STATUS_OK) && (st != AGC_STATUS_BUSY))
    {
        return (st);
    }

    st = imu_stall(AGC_JOB_SYSTEMS_TEST_GIMBALS_ZEROED, &good);

    return (good ? show(STEP_VERTDRFT, ctx) : st); /* asleep or SOMERR2: st */
} /* pipjobb_result */

/**********************************************************************
 * TORQUE (p.374): DSPTEM2 = 0, DRIFTI; SOUTHDR(POSITON - 1) = DRIFTI;
 * SHOW, then PIPACHK.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the display status, BAD_ARG for a bad POSITON
 **********************************************************************/
static agc_status_t torque(agc_job_ctx_t *ctx)
{
    int k = W_SOUTHDR - 1 + (int) w_sp(W_POSITON); /* SOUTHDR -1 +POSITON, word offset */

    if (!AGC_ASSERT(k >= 0 && k < W_WORDS))
    {
        return (AGC_STATUS_BAD_ARG); /* POSITON is a ground-loaded word */
    }

    agc_dsky_set_dsptem(3, 0);
    agc_dsky_set_dsptem(4, w[W_DRIFTI]);
    w[k] = w[W_DRIFTI]; /* INDEX POSITON, TS SOUTHDR -1 */

    return (show(STEP_PIPACHK, ctx));
}

/**********************************************************************
 * GEOSTRT4 (p.388): vertical drift -> VALMIS; else ERTHRVSE, TORQUE.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the display status (TORQUE or VALMIS)
 **********************************************************************/
static agc_status_t geostrt4(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    if (w_sp(W_TORQNDX) > 0)
    {
        /* VALMIS (p.377): DSPTEM2 = 0, DRIFTO; SHOW, then ENDTEST1.         */
        agc_dsky_set_dsptem(4, w[W_DRIFTO]);
        agc_dsky_set_dsptem(3, 0);

        return (show(STEP_ENDTEST1, ctx));
    }

    agc_interp_init(&ip);
    erthrvse(&ip);

    return (torque(ctx));
}

/**********************************************************************
 * SETUPER1 (p.388) after IMUPULSE: IMUSLLLG, GEOSTRT4.
 *
 * ARGUMENTS:
 *   pulse  agc_status_t     I   the IMUPULSE status
 *   ctx    agc_job_ctx_t *  I   the running job
 *
 * RETURNS: AGC_STATUS_OK (asleep), the IMUPULSE or stall status, or that
 *          of GEOSTRT4
 **********************************************************************/
static agc_status_t setuper1_stall(agc_status_t pulse, agc_job_ctx_t *ctx)
{
    bool good = false; /* torquing over and good */

    if (pulse == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* GYROBUSY: asleep          */
    }

    if ((pulse != AGC_STATUS_OK) && (pulse != AGC_STATUS_ALARM))
    {
        return (pulse);
    }

    agc_status_t st = imu_stall(AGC_JOB_SYSTEMS_TEST_TORQUED, &good); /* IMUSLLLG status */

    if (!good)
    {
        return (st);
    }

    return (geostrt4(ctx));
}

/**********************************************************************
 * SETUPER1 (p.388): OGC = XSM (-(ANGX, ANGY, ANGZ) GEORGEJ) / 2, IMUPULSE.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the IMUPULSE / IMUSLLLG / GEOSTRT4 status
 **********************************************************************/
static agc_status_t setuper1(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                                                                   /* interpreter state */
    agc_vec_t    ang = agc_vec_make(w_dp(W_ANGX), w_dp(W_ANGX - 2), w_dp(W_ANGX - 6)); /* VDEF ANGX, ANGY, ANGZ: rev 2^0 */

    agc_interp_init(&ip);
    ang = agc_vec_scale(&ip, agc_vec_neg(ang), GEORGEJ);                /* VCOMP, VXSC */
    ang = agc_vec_shr(agc_mat_vec(&ip, &agc_align_ram()->xsm, ang), 1); /* MXV, VSR1   */

    for (int i = 0; i < 3; i++)
    {
        agc_align_ram()->ogc[i] = ang.c[i]; /* STORE OGC */
    }

    return (setuper1_stall(agc_imu_pulse_buf(AGC_IMU_GBUF_OGC, AGC_JOB_SYSTEMS_TEST_TORQUE_RETRY), ctx));
}

/**********************************************************************
 * PERFERAS (p.387): TC ATIGINC (not run, see DEVIATION); CCS LENGTHOT --
 * passes left: SLEEPIE (TS LENGTHOT = LENGTHOT - 1); done: LOSVEC +1 =
 * CDUX when vertical (or LENGTHOT < 0), SETUPER1.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the SLEEPIE or SETUPER1 status
 **********************************************************************/
static agc_status_t perferas(agc_job_ctx_t *ctx)
{
    st_ccs_t len = ccs(w[W_LENGTHOT]); /* CCS LENGTHOT */

    if (len.branch == 0)
    {
        w[W_LENGTHOT] = len.a; /* SLEEPIE: TS LENGTHOT */

        return (sleepie_tail());
    }

    st_ccs_t torq = ccs(w[W_TORQNDX]); /* CCS TORQNDX (from LENGTHOT +0) */

    if (len.branch == 2 || (len.branch == 1 && torq.branch != 1))
    {
        w[W_LOSVEC + 1] = (len.branch == 1 && torq.branch == 3) ? 0u : agc_erase_cdux(); /* FOR TROUBLESHOOTING */
    }

    return (setuper1(ctx));
}

/**********************************************************************
 * ALFLT / NORMLOP (p.385): 1/PIPA unless GEOCOMPS is +0; DPIPAY = -(DELV
 * XSM 2).y, DPIPAZ = (DELV XSM 2).z.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I   interpreter state (OVFIND)
 *
 * RETURNS: AGC_STATUS_OK or the 1/PIPA status
 **********************************************************************/
static agc_status_t normlop(agc_interp_t *ip)
{
    agc_vec_t d = agc_erase_delv(); /* DELV, PIPA pulses x 2^14 */

    if (ccs(w[W_GEOCOMPS]).branch != 1)
    {
        agc_status_t st = agc_imu_one_over_pipa(&d); /* CCS GEOCOMPS: 1/PIPA */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        agc_erase_set_delv(d);
    }

    /* VXM XSM: component j = sum over i of DELV_i XSM_ij (DMP products as
     * DOT sums them), then VSL1; DPIPAY = DCOMP of Y, DPIPAZ = Z.         */
    const agc_mat_t *xsm = &agc_align_ram()->xsm; /* XSM, half-unit */

    for (int j = 1; j < 3; j++)
    {
        agc_tp_words_t m = agc_mpac_dmp(agc_mpac_dload(agc_dp_to_words(d.c[0])), agc_dp_to_words(xsm->row[0].c[j])); /* component j of DELV XSM, pulses x 2^14 */

        for (int i = 1; i < 3; i++)
        {
            m = agc_mpac_tad(ip, m, agc_mpac_dmp(agc_mpac_dload(agc_dp_to_words(d.c[i])), agc_dp_to_words(xsm->row[i].c[j])));
        }

        m = agc_mpac_shift(ip, agc_mpac_dload(agc_mpac_store(m)), 1);

        if (j == 1)
        {
            w_store(W_DPIPAY, agc_mpac_dcomp(m)); /* DCOMP, STORE DPIPAY */
        }
        else
        {
            w_store(W_DPIPAY + 4, m); /* STORE DPIPAZ */
        }
    }

    return (AGC_STATUS_OK);
} /* normlop */

/**********************************************************************
 * ALKCG / ALKCG2 (p.385): the next 12 gain words from ALFDK (ALTIM, ALTIMS
 * and ALDK .. ALDK +9), ALX1S -= 12.
 *
 * RETURNS: AGC_STATUS_OK, or the image's status (an ALX1S that points out
 *          of erasable memory)
 **********************************************************************/
static agc_status_t alkcg(void)
{
    int          x1 = (int) w_sp(W_ALX1S); /* X1 = ALX1S, gain word index */
    agc_status_t st = AGC_STATUS_OK;       /* image status */

    for (int k = 0; k < 6 && st == AGC_STATUS_OK; k++)
    {
        agc_dp_words_t g = {0, 0};

        st = image_words((int) EC_ALFDK + ALFDK_WORDS - x1 + 2 * k, &g); /* DLOAD* ALFDK +144D,1 */
        w_set_sp(W_ALDK - 2 + 2 * k, g.hi);                              /* STORE ALDK +10D,2    */
        w_set_sp(W_ALDK - 1 + 2 * k, g.lo);
    }

    w_set_sp(W_ALX1S, (agc_sp_t) (x1 - 12)); /* SXA,1 ALX1S */

    return (st);
}

/**********************************************************************
 * DELMLP / ALILP (pp.385-386): level PIPA increments into INTY, INTZ and
 * the measurements DELM; ALK, ALK +2 scaled by ALDK.  Word for word: the
 * DAD / DSU results keep the sign-disagreeing words the AGC stores.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I   interpreter state (OVFIND)
 **********************************************************************/
static void delmlp(agc_interp_t *ip)
{
    static const agc_dp_words_t pipasc = {2139, 1205};   /* 2DEC .13055869  */
    static const agc_dp_words_t velsc  = {-8556, -4822}; /* 2DEC -.52223476 */

    for (int k = 0; k <= 4; k += 4)
    {
        agc_tp_words_t m = agc_mpac_shift_round(ip, agc_mpac_dmp(agc_mpac_dload(w_words(W_DPIPAY + k)), pipasc), 9); /* DMP PIPASC, SLR 9D */

        m                     = agc_mpac_bdsu(ip, m, w_words(W_INTY + k));                                              /* BDSU* INTY */
        w_store(W_INTY + k, m);
        agc_dp_words_t pushed = agc_mpac_store(m);                                                                      /* PDDL           */
        agc_tp_words_t v      = agc_mpac_shift_round(ip, agc_mpac_dmp(agc_mpac_dload(velsc), w_words(W_VLAUN + k)), 2); /* DMP* VLAUN, SL2R */

        v = agc_mpac_dsu(ip, v, pushed); /* DSU (push-down), STADR */
        w_store(W_DELM + k, v);
        w_store(W_DELM + k + 2, v);
    }

    for (int k = 0; k <= 2; k += 2)
    {
        w_store(W_ALK + k, agc_mpac_dmpr(ip, agc_mpac_dload(w_words(W_ALK + k)), w_words(W_ALDK + k))); /* ALILP: DMPR* ALDK */
    }
}

/**********************************************************************
 * ALKLP (p.386): the four filter updates (CMPX1 alternates ALK / ALK +2).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I   interpreter state (OVFIND)
 **********************************************************************/
static void alklp(agc_interp_t *ip)
{
    static const agc_dp_words_t alsk[2] = {{2839, 5503}, {-136, -14205}}; /* ALSK, ALSK +2 */

    for (int o = 0; o <= 6; o += 2)
    {
        w_set_sp(W_CMPX1, (agc_sp_t) -w_sp(W_CMPX1));                                /* LXC,1 SXA,1 CMPX1 */
        int            a = (w_sp(W_CMPX1) > 0) ? 0 : 2;                              /* ALK +1 - X1        */
        agc_dp_words_t d = w_words(W_DELM + o);                                      /* DELM measurement */
        agc_tp_words_t m = agc_mpac_dmpr(ip, agc_mpac_dload(w_words(W_ALK + a)), d); /* MPAC */

        w_store(W_INTY + o, agc_mpac_dad(ip, m, w_words(W_INTY + o)));                 /* DAD* INTY */
        m = agc_mpac_dad(ip, agc_mpac_dload(w_words(W_ALK + 4 + o)), w_words(W_ALDK + 4 + o));
        w_store(W_ALK + 4 + o, m);
        m = agc_mpac_dad(ip, agc_mpac_dmpr(ip, m, d), w_words(W_INTY + 8 + o));        /* DMPR* DELM, DAD* INTY +16D */
        w_store(W_INTY + 8 + o, m);
        m = agc_mpac_shift_round(ip, agc_mpac_dmp(agc_mpac_dload(alsk[a / 2]), d), 1); /* DMP* DELM, SL1R */
        w_store(W_VLAUN + o, agc_mpac_dad(ip, m, w_words(W_VLAUN + o)));
    }
}

/**********************************************************************
 * LOOSE / BOOP (pp.386-387): (POSNV, VLAUN, ACCWD) through TRANSM1 for Y
 * and Z; sines and cosines of the drift angles for the erasable program.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I   interpreter state (OVFIND)
 *
 * RETURNS: AGC_STATUS_OK or the image's read status of TRANSM1
 **********************************************************************/
static agc_status_t loose_boop(agc_interp_t *ip)
{
    agc_dp_words_t t[TRANSM1_WORDS / 2]; /* TRANSM1 row by row, as stored */
    agc_status_t   st = AGC_STATUS_OK;   /* image status */

    for (int i = 0; i < TRANSM1_WORDS / 2 && st == AGC_STATUS_OK; i++)
    {
        st = image_words((int) EC_TRANSM1 + 2 * i, &t[i]);
    }

    for (int k = 0; k <= 4 && st == AGC_STATUS_OK; k += 4)
    {
        const int      out[3] = {W_POSNV + k, W_VLAUN + k, W_ACCWD + k};
        agc_dp_words_t v[3]   = {w_words(out[0]), w_words(out[1]), w_words(out[2])}; /* VDEF */
        agc_tp_words_t r[3];                                                         /* MXV components (TP MPAC) */

        for (int i = 0; i < 3; i++)
        {
            /* MXV: row i . v, the three DMP products summed (DOT), VSL1.    */
            r[i] = agc_mpac_dmp(agc_mpac_dload(v[0]), t[3 * i]); /* DMPSUB: MPAC x operand */

            for (int j = 1; j < 3; j++)
            {
                r[i] = agc_mpac_tad(ip, r[i], agc_mpac_dmp(agc_mpac_dload(v[j]), t[3 * i + j]));
            }
        }

        for (int i = 0; i < 3; i++)
        {
            w_store(out[i], agc_mpac_shift(ip, agc_mpac_dload(agc_mpac_store(r[i])), 1));
        }
    }

    for (int j = 0; j < 3; j++)
    {
        static const int ang[3] = {W_ANGX, W_ANGX - 2, W_ANGX - 6};                           /* ANGX, ANGY, ANGZ */
        agc_dp_t         m      = agc_dp_shr_round(agc_dp_mul(ip, w_dp(ang[j]), GEORGEJ), 2); /* DMPR GEORGEJ, SR2R: rev x 2/pi 2^2 */

        perferas_vac[j]     = agc_dp_shl_round(ip, agc_dp_sin(m), 3); /* SIN SL3R: 10D, 12D, 14D */
        perferas_vac[j + 3] = agc_dp_cos(m);                          /* COS: 16D, 18D, 20D       */
    }

    return (st);
} /* loose_boop */

/**********************************************************************
 * ALFLT (pp.385-387): the drift filter, one pass per ALLOOP sample.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the NORMLOP, ALKCG or LOOSE status, or that of PERFERAS
 **********************************************************************/
static agc_status_t alflt(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                 /* interpreter state */
    agc_status_t st = AGC_STATUS_OK; /* status of the filter steps */

    agc_interp_init(&ip);
    st = normlop(&ip);

    if (st == AGC_STATUS_OK && w_sp(W_GEOCOMPS) <= 0)
    {
        /* ALCGKK: new gains unless ALTIMS < 0; ALFLT3.                      */
        if (w_sp(W_ALTIMS) >= 0)
        {
            st = alkcg();
        }

        if (st == AGC_STATUS_OK)
        {
            delmlp(&ip);
            alklp(&ip);
            st = loose_boop(&ip);
        }
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (perferas(ctx));
}

/**********************************************************************
 * ALLOOP (p.384): stop on OVFLOWCK; step ALTIM; next sample in 1SECXT
 * while passes are left; DELV = PIPAs (zeroed); SPECSTS: FINDVAC ALFLT.
 *
 * RETURNS: AGC_STATUS_OK (TASKOVER), or the TWIDDLE or FINDVAC status
 **********************************************************************/
static agc_status_t alloop(void)
{
    if (w[W_OVFLOWCK] != 0u)
    {
        return (AGC_STATUS_OK); /* TASKOVER */
    }

    st_ccs_t c = ccs(w[W_ALTIM]); /* CCS ALTIM */

    if (c.branch <= 1)
    {
        w[W_ALTIMS] = c.a;                       /* CA A, TS ALTIMS (+0: 0) */
        w[W_ALTIM]  = (uint16_t) (WORD15 - c.a); /* CS A, TS ALTIM          */
    }
    else
    {
        w[W_ALTIM] = (c.branch == 2) ? (uint16_t) (WORD15 - c.a) : 0u; /* -0: TS ALTIM with A = 0 */
    }

    agc_status_t st  = AGC_STATUS_OK;           /* TWIDDLE status */
    bool         geo = (w_sp(W_GEOCOMPS) == 1); /* GEOCOMPS = 1: sample on */

    if (geo || (((w[W_LENGTHOT] & 040000u) == 0u) && w[W_LENGTHOT] != 0u))
    {
        st = agc_waitlist_call((uint16_t) w[W_1SECXT], AGC_TASK_SYSTEMS_TEST_SAMPLE); /* TWIDDLE ALLOOP */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_vec_t      d       = agc_erase_delv(); /* DELV, PIPA pulses x 2^14 */
    const agc_sp_t pipa[3] = {agc_erase_pipax(), agc_erase_pipay(), agc_erase_pipaz()};

    for (int i = 0; i < 3; i++)
    {
        pipa_set(i, 0);                                    /* CAF ZERO, XCH PIPAX */
        d.c[i] = agc_dp_pack(pipa[i], agc_dp_low(d.c[i])); /* TS DELVX            */
    }

    agc_erase_set_delv(d);

    return (agc_exec_findvac(PRIO20, AGC_JOB_SYSTEMS_TEST_FILTER)); /* SPECSTS */
} /* alloop */

/**********************************************************************
 * AGC: none -- Forward: the step a QPLACE / QPLACES return goes on at.
 *
 * ARGUMENTS:
 *   step  st_step_t        I   the step from QPLACE or QPLACES
 *   ctx   agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the step's status, BAD_ARG for a word that is no step
 **********************************************************************/
static agc_status_t run_step(st_step_t step, agc_job_ctx_t *ctx)
{
    switch (step)
    {
        case STEP_PIPACHK:
            return (pipachk());
        case STEP_PIPACHK_ARM:
            return (pipachk_arm());
        case STEP_PIPATASK_START:
            /* PIPACHK after CHECKG: TWIDDLE PIPATASK in 20 ms, ENDOFJOB.    */
            return (agc_waitlist_call(PIP_START_CS, AGC_TASK_SYSTEMS_TEST_PIP_TIMER));
        case STEP_PIPJOBB_CATCH:
            return (pipjobb_catch());
        case STEP_PIPJOBB_RESULT:
            return (pipjobb_result(ctx));
        case STEP_VERTDRFT:
            return (vertdrft(DEC3990));
        case STEP_GUESS1:
            return (guess1());
        case STEP_ENDTEST1:
            return (endtest1());
        case STEP_ENDOFJOB:
            return (AGC_STATUS_OK);
        case STEP_NONE:
        case STEP_COUNT:
        default:
            break;
    }

    return (AGC_STATUS_BAD_ARG); /* QPLACE / QPLACES overwritten */
}

/**********************************************************************
 * CHECKG1 (pp.378-379): no pulse yet -> sample again (CHANG1 between);
 * a pulse: DATAPL +RESULTCT = PIPA, +1, +2 = FINETIME; TC QPLACE.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the CHANG1 status (no pulse yet), BAD_ARG for a bad PIPINDEX
 *          or RESULTCT, or the step's status
 **********************************************************************/
static agc_status_t pip_catch(agc_job_ctx_t *ctx)
{
    int          axis = 0;                                 /* PIPINDEX */
    agc_dp_t     t    = 0;                                 /* FINETIME HISCALAR, LOSCALAR, counts 2^-28 */
    agc_status_t st   = pipa_index(&axis);                 /* PIPINDEX / FINETIME status */
    int          r    = W_DATAPL + (int) w_sp(W_RESULTCT); /* INDEX RESULTCT */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_sp_t p = pipa_get(axis); /* PIPA count now, pulses 2^0 */

    if (p == (agc_sp_t) -w_sp(W_ZERONDX))
    {
        w_set_sp(W_ZERONDX, (agc_sp_t) -p); /* AD ZERONDX, BZF CHECKG1 */

        return (agc_exec_job_yield(AGC_JOB_SYSTEMS_TEST_PIP_CATCH));
    }

    st = agc_systems_test_finetime(&t); /* returns with INHINT */

    if (st == AGC_STATUS_BUSY || !AGC_ASSERT(r >= W_DATAPL && r + 2 < W_WORDS))
    {
        agc_relint();

        return ((st == AGC_STATUS_BUSY) ? agc_exec_job_yield(AGC_JOB_SYSTEMS_TEST_PIP_CATCH) : AGC_STATUS_BAD_ARG); /* RESULTCT overwritten */
    }

    w_set_sp(r, p);
    w_set_sp(r + 1, agc_dp_high(t));
    w_set_sp(r + 2, agc_dp_low(t));
    agc_relint();

    return (run_step((st_step_t) w_sp(W_QPLACE), ctx)); /* ENDCHKG: TC QPLACE */
}

/**********************************************************************
 * PIPATASK (p.375): DIM LENGTHOT; passes left -> again in 5.12 s, else
 * STARTPIP: FINDVAC PIPJOBB.
 *
 * RETURNS: the TWIDDLE or FINDVAC status
 **********************************************************************/
static agc_status_t pip_timer(void)
{
    st_ccs_t c = ccs(w[W_LENGTHOT]); /* CCS LENGTHOT */

    if (c.branch == 0)
    {
        w[W_LENGTHOT] = c.a; /* DIM: one count toward zero */
    }
    else if (c.branch == 2)
    {
        w[W_LENGTHOT] = (uint16_t) (WORD15 - c.a);
    }

    if (c.branch == 0 && c.a != 0u)
    {
        return (agc_waitlist_call(PIP_PERIOD_CS, AGC_TASK_SYSTEMS_TEST_PIP_TIMER)); /* TWIDDLE */
    }

    return (agc_exec_findvac(PRIO20, AGC_JOB_SYSTEMS_TEST_PIP_RESULT)); /* STARTPIP */
}

/**********************************************************************
 * AGC: none -- The body of a job or task of the test, with the test erasables fetched.
 *
 * ARGUMENTS:
 *   e    st_entry_t       I   the job or task starting
 *   ctx  agc_job_ctx_t *  I   the running job (NULL for a task)
 *
 * RETURNS: the status of what runs; AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
static agc_status_t st_body(st_entry_t e, agc_job_ctx_t *ctx)
{
    bool good = false; /* stall over and good */

    switch (e)
    {
        case ENTRY_REDO:
            return (redo(ctx));
        case ENTRY_IMU_ZEROED:
            imu_stall_outcome(&good);

            return (good ? imuback(ctx) : AGC_STATUS_OK); /* stall over and good; else SOMERR2 */
        case ENTRY_SITE_DISPLAY:
            return (site_display(ctx));
        case ENTRY_SITE_LOADED:
            return (posgmbl());
        case ENTRY_END:
            return (endtest1());
        case ENTRY_COARSE_ALIGNED:
            imu_stall_outcome(&good);

            return (good ? coarse_aligned() : AGC_STATUS_OK); /* stall over and good; else SOMERR2 */
        case ENTRY_FINE_ALIGNED:
            imu_stall_outcome(&good);

            return (good ? fine_aligned() : AGC_STATUS_OK); /* stall over and good; else SOMERR2 */
        case ENTRY_DRIFT_START:
            return (estims());
        case ENTRY_RESUME:
            return (run_step((st_step_t) w_sp(W_QPLACE), ctx));
        case ENTRY_EARTH_RATE_RETRY:
            return (earthr_stall(agc_imu_pulse_buf(AGC_IMU_GBUF_ERCOMP, AGC_JOB_SYSTEMS_TEST_EARTH_RATE_RETRY)));
        case ENTRY_EARTH_RATE_DONE:
            imu_stall_outcome(&good);

            return (good ? run_step((st_step_t) w_sp(W_QPLACES), ctx) : AGC_STATUS_OK);       /* stall over and good; else SOMERR2 */
        case ENTRY_PIP_CATCH:
            return (pip_catch(ctx));
        case ENTRY_PIP_RESULT:
            return ((w[W_NDXCTR] == 0u) ? earthr_star(STEP_PIPJOBB_CATCH) : pipjobb_catch()); /* PIPJOBB */
        case ENTRY_FILTER:
            return (alflt(ctx));
        case ENTRY_TORQUE_RETRY:
            return (setuper1_stall(agc_imu_pulse_buf(AGC_IMU_GBUF_OGC, AGC_JOB_SYSTEMS_TEST_TORQUE_RETRY), ctx));
        case ENTRY_TORQUED:
            imu_stall_outcome(&good);

            return (good ? geostrt4(ctx) : AGC_STATUS_OK); /* stall over and good; else SOMERR2 */
        case ENTRY_GIMBALS_ZEROED:
            imu_stall_outcome(&good);

            return (good ? show(STEP_VERTDRFT, ctx) : AGC_STATUS_OK);     /* stall over and good; else SOMERR2 */
        case ENTRY_SHOW:
            return (show1(ctx));
        case ENTRY_DRIFT_WAKE:
            return (agc_exec_job_wake(AGC_JOB_SYSTEMS_TEST_DRIFT_START)); /* GOESTIMS */
        case ENTRY_PIP_TIMER:
            return (pip_timer());
        case ENTRY_SAMPLE:
            return (alloop());
        default:
            break;
    } /* switch (e) */

    return (AGC_STATUS_BAD_ARG);
} /* st_body */

/**********************************************************************
 * AGC: none -- Run one job or task of the test on the test erasables (fetch, body,
 * write back; the body's failure is reported first).
 *
 * ARGUMENTS:
 *   e    st_entry_t       I   the job or task starting
 *   ctx  agc_job_ctx_t *  I   the running job (NULL for a task)
 *
 * RETURNS: the body's status, else the write-back status
 **********************************************************************/
static agc_status_t st_enter(st_entry_t e, agc_job_ctx_t *ctx)
{
    st_fetch();

    agc_status_t st = st_body(e, ctx);
    agc_status_t fl = st_flush(); /* write-back status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (fl);
}

/**********************************************************************
 * REDO (V92, LM p.373 / Comanche SYSTEM_TEST_STANDARD_LEAD_INS): NEWMODEX
 * 07, then the IMU performance tests.  Comanche falls into IMUTEST (imu
 * lane, prelaunch alignment).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the V92 job
 *
 * RETURNS: the status of REDO (CSM: of IMUTEST)
 **********************************************************************/
agc_status_t agc_job_systems_test(agc_job_ctx_t *ctx)
{
    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        agc_status_t st = agc_phase_newmode((agc_sp_t) SYSTEMS_TEST_MODE); /* NEWMODEX status */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        return (agc_csm_prelaunch_imutest(ctx)); /* FALL INTO IMUTEST       */
    }

    return (st_enter(ENTRY_REDO, ctx));
}

/**********************************************************************
 * IMUBACK after IMUZERR's IMUSTALL (p.373).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the woken job
 *
 * RETURNS: the status of IMUBACK
 **********************************************************************/
agc_status_t agc_job_systems_test_imu_zeroed(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_IMU_ZEROED, ctx));
}

/**********************************************************************
 * LATAZCHK again after ENTER on V06N41 (p.373, TC -5).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display status
 **********************************************************************/
agc_status_t agc_job_systems_test_site_display(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_SITE_DISPLAY, ctx));
}

/**********************************************************************
 * LATAZCHK V33: POSGMBL (pp.373-374).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the status of POSGMBL
 **********************************************************************/
agc_status_t agc_job_systems_test_site_loaded(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_SITE_LOADED, ctx));
}

/**********************************************************************
 * ENDTEST1 (p.377): V34 on V06N41 or V06N98.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the ENDTEST1 status
 **********************************************************************/
agc_status_t agc_job_systems_test_end(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_END, ctx));
}

/**********************************************************************
 * POSGMBL after IMUSLLLG (p.374).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the woken job
 *
 * RETURNS: the status of PIPACHK or FINIMUDD
 **********************************************************************/
agc_status_t agc_job_systems_test_coarse_aligned(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_COARSE_ALIGNED, ctx));
}

/**********************************************************************
 * FINIMUDD after IMUSTALL (p.374).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the woken job
 *
 * RETURNS: the LONGCALL status
 **********************************************************************/
agc_status_t agc_job_systems_test_fine_aligned(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_FINE_ALIGNED, ctx));
}

/**********************************************************************
 * ESTICADR (p.374): the job GOESTIMS wakes runs ESTIMS (p.383).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the woken job
 *
 * RETURNS: the ESTIMS status
 **********************************************************************/
agc_status_t agc_job_systems_test_drift_start(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_DRIFT_START, ctx));
}

/**********************************************************************
 * TC QPLACE (pp.378, 381): V33 on V06N98, or a PIPA pulse caught.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the status of the step in QPLACE
 **********************************************************************/
agc_status_t agc_job_systems_test_resume(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_RESUME, ctx));
}

/**********************************************************************
 * EARTHR's PULSEIMU again after GYROBUSY (p.380).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the woken job
 *
 * RETURNS: the PULSEIMU / IMUSLLLG status
 **********************************************************************/
agc_status_t agc_job_systems_test_earth_rate_retry(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_EARTH_RATE_RETRY, ctx));
}

/**********************************************************************
 * EARTHR* after IMUSLLLG: TC QPLACES (p.380).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the status of the step in QPLACES
 **********************************************************************/
agc_status_t agc_job_systems_test_earth_rate_done(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_EARTH_RATE_DONE, ctx));
}

/**********************************************************************
 * CHECKG1 (p.378): wait for a PIPA pulse.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the CHECKG1 status
 **********************************************************************/
agc_status_t agc_job_systems_test_pip_catch(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_PIP_CATCH, ctx));
}

/**********************************************************************
 * PIPJOBB (p.375): the end of the PIPA count.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the EARTHR* or CHECKG status
 **********************************************************************/
agc_status_t agc_job_systems_test_pip_result(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_PIP_RESULT, ctx));
}

/**********************************************************************
 * ALFLT (p.385): one pass of the drift filter.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the ALFLT status
 **********************************************************************/
agc_status_t agc_job_systems_test_filter(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_FILTER, ctx));
}

/**********************************************************************
 * SETUPER1's IMUPULSE again after GYROBUSY (p.388).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the woken job
 *
 * RETURNS: the IMUPULSE / IMUSLLLG status
 **********************************************************************/
agc_status_t agc_job_systems_test_torque_retry(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_TORQUE_RETRY, ctx));
}

/**********************************************************************
 * GEOSTRT4 after SETUPER1's IMUSLLLG (p.388).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the woken job
 *
 * RETURNS: the GEOSTRT4 status
 **********************************************************************/
agc_status_t agc_job_systems_test_torqued(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_TORQUED, ctx));
}

/**********************************************************************
 * COAALIGN after IMUSTALL (p.378): SHOW, then VERTDRFT.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the woken job
 *
 * RETURNS: the SHOW status
 **********************************************************************/
agc_status_t agc_job_systems_test_gimbals_zeroed(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_GIMBALS_ZEROED, ctx));
}

/**********************************************************************
 * SHOW1 again after ENTER on V06N98 (p.381).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display status
 **********************************************************************/
agc_status_t agc_job_systems_test_show(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_SHOW, ctx));
}

/**********************************************************************
 * GOESTIMS (p.374): JOBWAKE ESTICADR, TASKOVER.
 *
 * RETURNS: the JOBWAKE status
 **********************************************************************/
agc_status_t agc_task_systems_test_drift_wake(void)
{
    return (st_enter(ENTRY_DRIFT_WAKE, NULL));
}

/**********************************************************************
 * PIPATASK (p.375): every 5.12 s until LENGTHOT runs out, then PIPJOBB.
 *
 * RETURNS: the TWIDDLE or FINDVAC status
 **********************************************************************/
agc_status_t agc_task_systems_test_pip_timer(void)
{
    return (st_enter(ENTRY_PIP_TIMER, NULL));
}

/**********************************************************************
 * ALLOOP (p.384): the drift test's once-per-sample task.
 *
 * RETURNS: the ALLOOP status
 **********************************************************************/
agc_status_t agc_task_systems_test_sample(void)
{
    return (st_enter(ENTRY_SAMPLE, NULL));
}
