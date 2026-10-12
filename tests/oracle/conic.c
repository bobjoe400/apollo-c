/**********************************************************************
 * FILE NAME: conic.c
 *
 * PURPOSE:
 *   nav-conic against the real AGC code (yaAGC oracle). Expected values
 *   are the dumps in tools/oracle/cases/nav_conic.txt (Comanche055) and
 *   nav_conic_lm.txt (Luminary099), produced from nav_conic.spec /
 *   nav_conic_lm.spec by tools/oracle/bin/agc_probe; the inputs below
 *   are the same DP words the specs poke. Each row:
 *     kind K = KEPPREP + KEPLER, N = KEPLER (KEPLERN entry), C / L =
 *     LAMBERT on the CSM / LM build (C: in[10] NORMSW, in[11..13] UN;
 *     L: in[10] ITERCTR), T = TIMETHET, R = TIMERAD, A = APSIDES;
 *     in[] inputs (see run_kepler etc.), exp[] AGC outputs, fl5/fl8
 *     FLAGWRD5/8.
 *
 * AGC SOURCE:
 *   Comanche055 / Luminary099 CONIC_SUBROUTINES.agc (KEPLERN, LAMBERT,
 *   TIMETHET, TIMERAD, APSIDES), ORBITAL_INTEGRATION.agc (KEPPREP)
 *
 * EXTERNAL VARIABLES:
 *   none
 *
 * EXTERNAL REFERENCES:
 *   Name                Description
 *   ------------------  --------------------------------------------
 *   agc_conic_*         the routines under test (agc/nav/conic.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   exit status 1 when any check fails
 *
 * NOTES:
 *   Every output must match the AGC word for word (tolerance 0): the
 *   conic MPAC is modelled on the stored words (agc_mpac_*), so the
 *   sign-disagreeing pairs that DAS/DSU/BDSU leave (R1A, COGA, X, KEPC2,
 *   DELX, ...) reach DMP, SR, SQRT and ROUND as in the AGC
 *   (review-nav-conic R1). aps_circ is the circular orbit of
 *   review-prog-rendz P6 (ECC 0 only on the stored R1A words).
 *   The rows kep_leo_q, kep_leo_mq, kep_leo_2p25, kepn_leo_badx and
 *   lam_leo_q take their values from nav_conic_dvfix.txt, the same spec
 *   run on the DV-patched probe (oracle finding F1: yaAGC's DV fast path
 *   reads a -0 minor word as a borrow; the real AGC does not).
 **********************************************************************/
#include <stdio.h>
#include <string.h>

#include "agc/core/assertion.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/nav/conic.h"

typedef struct oracle_row
{
    char        kind;    /* K N C L T R A (see PURPOSE)       */
    const char *tag;     /* case name in the dump             */
    agc_dp_t    in[16];  /* input DP words, scales per kind   */
    agc_dp_t    exp[10]; /* AGC output DP words, same scales  */
    int         fl5;     /* FLAGWRD5 after the run            */
    int         fl8;     /* FLAGWRD8 after the run            */
} oracle_row_t;

static const oracle_row_t ROWS[] = { /* AGC cases, DP words */
    {'K',
     "kep_leo_q",
     {0, 3339068, 0, 0, 0, 162021496, 0, 135779, 0, 0, 0, 0, 0, 0, 0, 0},
     {8313381, 0, 3339072, 0, -162021376, 0, 0, 135779, 8313381, 135779},
     0,
     0},
    {'K',
     "kep_leo_mq",
     {0, 3339068, 0, 0, 0, 162021496, 0, -135779, 0, 0, 0, 0, 0, 0, 0, 0},
     {-8313381, 0, -3339072, 0, 162021376, 0, 0, -135779, -8313381, -135779},
     0,
     0},
    {'K',
     "kep_leo_2p25",
     {0, 3339068, 0, 0, 0, 162021496, 0, 1222011, 0, 0, 0, 0, 0, 0, 0, 0},
     {74820429, 0, 3339072, 0, -162021376, 0, 0, 135779, 8313377, 135779},
     0,
     0},
    {'K',
     "kep_ell_3d",
     {0, 2500000, 2000000, 1000000, -83886080, 115343360, 52428800, 300000, 0,
      0, 0, 0, 0, 0, 0, 0},
     {20228322, -596128, -2148704, -1026304, 200471552, -42566144, -15451392,
      300000, 19578244, 300000},
     0,
     0},
    {'K',
     "kep_hyp",
     {0, 3500000, 0, 0, 0, 261117046, 0, 360000, 0, 0, 0, 0, 0, 0, 0, 0},
     {-2783852, -3777792, 15584640, 0, -93211136, 142611712, 0, 360000, 9994392,
      360000},
     0,
     0},
    {'K',
     "kep_moon",
     {1, 3696000, 0, 0, 0, 136634393, 0, 100000, 0, 0, 0, 0, 0, 0, 0, 0},
     {4907712, 2350944, 2851904, 0, -105431040, 86909952, 0, 100000, 4907712,
      100000},
     0,
     0},
    {'K',
     "kep_leo_10min",
     {0, 3339069, 0, 0, 0, 142187239, 77677243, 60000, 0, 0, 0, 0, 0, 0, 0, 0},
     {3673636, 2566432, 1874560, 1024096, -103648256, 109286656, 59703552, 60000, 3673636, 60000},
     0,
     0},
    {'N',
     "kepn_leo_badx",
     {3339068, 0, 0, 0, 162021496, 0, 135779, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     {0, 0, 3339072, 0, -162021376, 0, 0, 135779, 8313377, 135779},
     0,
     0},
    {'N',
     "kepn_hyp_badx",
     {3500000, 0, 0, 0, 261117046, 0, 360000, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     {0, -3777792, 15584640, 0, -93211136, 142611712, 0, 360000, 9994392,
      360000},
     0,
     0},
    {'N',
     "kepn_ell_wrongsign",
     {2500000, 2000000, 1000000, -83886080, 115343360, 52428800, 300000,
      -6144000, 0, 0, 0, 0, 0, 0, 0, 0},
     {0, -596064, -2148704, -1026304, 200473088, -42565888, -15451136, 300000,
      19578286, 300000},
     0,
     0},
    {'N',
     "kepn_ell_back",
     {2500000, 2000000, 1000000, -83886080, 115343360, 52428800, -700000, 0, 0,
      0, 0, 0, 0, 0, 0, 0},
     {0, -2838912, -45632, -86752, 36706560, -160080640, -74596608, -236498,
      -17696804, -236498},
     0,
     0},
    {'N',
     "kepn_ell_dxch",
     {3500000, 0, 0, 0, 261117046, 0, 5000000, 5000000, 0, 0, 0, 0, 0, 0, 0, 0},
     {0, -95428768, 145383040, 0, -80181248, 112576000, 0, 5000000, 23949810, 5000000},
     0,
     0},
    {'N',
     "kepn_ell_dxch2",
     {3500000, 0, 0, 0, 174078031, 0, 5000000, 20000000, 0, 0, 0, 0, 0, 0, 0, 0},
     {0, 3422624, 805952, 0, -32976384, 170247424, 0, 19563, 1140981, 19562},
     0,
     0},
    {'C',
     "lam_leo_q",
     {3339068, 0, 0, 0, 3339068, 0, 135779, 8192, 1, 0, 0, 0, 0, 0, 0, 0},
     {540, 162021244, 0, -162021376, -256, 0, 28, 16777160, 4194318, 0},
     0,
     0},
    {'C',
     "lam_3d",
     {3500000, 0, 0, -1000000, 4000000, 500000, 250000, 8192, 1, 0, 0, 0, 0, 0,
      0, 0},
     {44422934, 149455768, 18681972, -116959488, -55258880, -6907392, 2474103,
      15197637, 4258699, 0},
     0,
     0},
    {'C',
     "lam_3d_guess",
     {3500000, 0, 0, -1000000, 4000000, 500000, 250000, 8192, 0, 1677722, 0, 0,
      0, 0, 0, 0},
     {44422890, 149455800, 18681976, -116959488, -55258880, -6907392, 2474100,
      15197642, 4258698, 0},
     0,
     0},
    {'C',
     "lam_gt180",
     {3339068, 0, 0, -500000, -3339068, 0, 407337, -8192, 1, 0, 0, 0, 0, 0, 0,
      0},
     {969584, 163227392, 0, 160020736, -21414656, 0, 49829, 17027880, 4131488,
      0},
     0,
     0},
    {'C',
     "lam_fast",
     {3500000, 0, 0, 0, 3750000, 0, 60000, 8192, 1, 0, 0, 0, 0, 0, 0, 0},
     {-17099908, 26494310, 0, -151897344, 18866176, 0, -5414160, 58271278,
      -12247649, 0},
     0,
     0},
    {'C',
     "lam_slow170",
     {3500000, 0, 0, -3450000, 600000, 0, 900000, 8192, 1, 0, 0, 0, 0, 0, 0, 0},
     {97880160, 154104406, 0, 70035200, -168517888, 0, 5328065, 15909167,
      2806791, 0},
     0,
     0},
    {'C',
     "lam_badguess",
     {3500000, 0, 0, -1000000, 4000000, 500000, 250000, 8192, 0, -7549747, 0, 0,
      0, 0, 0, 0},
     {44422790, 149455824, 18681978, -116960000, -55258880, -6907392, 2474094,
      15197649, 4258698, 0},
     0,
     0},
    {'C',
     "lam_un_given",
     {3500000, 0, 0, -1000000, 4000000, 0, 250000, 8192, 1, 0, 1, 0, 0,
      134217728, 0, 0},
     {44639366, 150175912, 0, -117145344, -57033728, 0, 2493490, 15108380,
      4277785, 0},
     0,
     0},
    {'C',
     "lam_un_gt180",
     {3339068, 0, 0, -500000, -3339068, 0, 407337, -8192, 1, 0, 1, 0, 0,
      134217728, 0, 0},
     {969584, 163227390, 0, 160020736, -21414656, 0, 49829, 17027880, 4131488,
      0},
     0,
     0},
    {'C',
     "lam_hien",
     {3500000, 0, 0, -1000000, 4000000, 500000, 20000, 8192, 1, 0, 0, 0, 0, 0, 0, 0},
     {0, 0, 0, 0, 0, 0, -8694019, 268414828, -130793953, 0},
     4,
     0},
    {'C',
     "lam_hien_g",
     {3500000, 0, 0, -1000000, 4000000, 500000, 20000, 8192, 0, 2516582, 0, 0, 0, 0, 0, 0},
     {0, 0, 0, 0, 0, 0, -8694063, 268432448, -130803820, 0},
     4,
     0},
    {'C',
     "lam_bigtime",
     {3500000, 0, 0, -1000000, 4000000, 500000, 8000000, 8192, 0, 2516582, 0, 0, 0, 0, 0, 0},
     {186175912, 104565838, 13070730, -44485120, -188030976, -23503872, 14820296, 7439273, 723760, 0},
     0,
     0},
    {'C',
     "lam_small_uplim",
     {3500000, 0, 0, 3600000, 200000, 0, 2000, 8192, 1, 0, 0, 0, 0, 0, 0, 0},
     {47123966, 93509830, 0, 43848192, 93489408, 0, 4227411, 117857765, -28558688, 0},
     0,
     0},
    {'C',
     "lam_small_loen",
     {3500000, 0, 0, 3600000, 20000, 0, 60000, 8192, 1, 0, 0, 0, 0, 0, 0, 0},
     {47323388, 1820270, 0, -29107712, 1607936, 0, 218087066, 2220, 8012931, 0},
     0,
     0},
    {'L',
     "lmlam_3d_ctr20",
     {3500000, 0, 0, -1000000, 4000000, 500000, 250000, 8192, 1, 0, 20, 0, 0, 0,
      0, 0},
     {44422934, 149455768, 18681972, -116959488, -55258880, -6907392, 2474103,
      15197637, 4258699, 0},
     0,
     0},
    {'L',
     "lmlam_3d_ctr5",
     {3500000, 0, 0, -1000000, 4000000, 500000, 250000, 8192, 1, 0, 5, 0, 0, 0,
      0, 0},
     {44312330, 149499198, 18687400, -117022976, -55155712, -6894592, 2467226,
      15206471, 4258134, 0},
     4,
     0},
    {'L',
     "lmlam_3d_ctr2",
     {3500000, 0, 0, -1000000, 4000000, 500000, 250000, 8192, 1, 0, 2, 0, 0, 0,
      0, 0},
     {-71615802, 202860506, 25357564, -190512640, 52038400, 6504704, -2938560,
      27999209, 529842, 0},
     4,
     0},
    {'L',
     "lmlam_slow_ctr3",
     {3500000, 0, 0, -3450000, 600000, 0, 900000, 8192, 1, 0, 3, 0, 0, 0, 0, 0},
     {118448002, 153242770, 0, 90446336, -171194112, 0, 6483920, 15731761,
      2105966, 0},
     4,
     0},
    {'T',
     "tth_leo_90",
     {3339068, 0, 0, 0, 162021496, 0, 134217728, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     {135779, 0, 3339072, 0, -162021376, 0, 0, 0, 0, 0},
     0,
     0},
    {'T',
     "tth_ell_120",
     {3500000, 0, 0, 0, 174078031, 0, 116235962, -67108864, 1, 0, 0, 0, 0, 0, 0,
      0},
     {224865, 0, 0, 119632148, 224865, 204442107, 176229826, 0, 0, 0},
     0,
     0},
    {'T',
     "tth_hyp_inf",
     {3500000, 0, 0, 0, 261117046, 0, 23306664, -132178659, 1, 0, 0, 0, 0, 0, 0,
      0},
     {0, 1210952, 5462, 210272256, 0, 204442107, 176229826, 0, 0, 0},
     0,
     64},
    {'T',
     "tth_ell_350",
     {3500000, 0, 0, 0, 174078031, 0, -23306664, 132178659, 0, 0, 0, 0, 0, 0, 0,
      0},
     {815328, 3455936, -609408, 0, 24983040, 171892224, 0, 0, 0, 0},
     0,
     0},
    {'T',
     "tth_radial",
     {3339068, 0, 0, 104857600, 0, 0, 134217728, 0, 1, 0, 0, 0, 0, 0, 0, 0},
     {0, 1756768, 0, 0, 0, 204442107, 176229826, 0, 0, 0},
     0,
     8},
    {'T',
     "tth_ell_355",
     {3500000, 0, 0, 0, 174078031, 0, -11697846, 133706989, 0, 0, 0, 0, 0, 0, 0, 0},
     {822710, 3488992, -305248, 0, 12539392, 173531136, 0, 0, 0, 0},
     0,
     0},
    {'T',
     "tth_ell_358",
     {3500000, 0, 0, 0, 174078031, 0, -4684131, 134135966, 0, 0, 0, 0, 0, 0, 0, 0},
     {827128, 3498272, -122176, 0, 5021184, 173991424, 0, 0, 0, 0},
     0,
     1},
    {'T',
     "tth_ell_359",
     {3500000, 0, 0, 0, 174078031, 0, -2342422, 134197286, 0, 0, 0, 0, 0, 0, 0, 0},
     {828601, 3499584, -61056, 0, 2511360, 174057472, 0, 0, 0, 0},
     0,
     1},
    {'T',
     "tth_ell_359p5",
     {3500000, 0, 0, 0, 174078031, 0, -1171256, 134212617, 0, 0, 0, 0, 0, 0, 0, 0},
     {829336, 3499904, -30560, 0, 1255424, 174073344, 0, 0, 0, 0},
     0,
     1},
    {'T',
     "tth_ell_359p9",
     {3500000, 0, 0, 0, 174078031, 0, -234254, 134217524, 0, 0, 0, 0, 0, 0, 0, 0},
     {829925, 3500000, -6112, 0, 250880, 174077952, 0, 0, 0, 0},
     0,
     1},
    {'T',
     "tth_ell_0p3",
     {3500000, 0, 0, 0, 174078031, 0, 702759, 134215888, 0, 0, 0, 0, 0, 0, 0, 0},
     {442, 3499968, 18336, 0, -753152, 174076416, 0, 0, 0, 0},
     0,
     0},
    {'T',
     "tth_ell_1",
     {3500000, 0, 0, 0, 174078031, 0, 2342422, 134197286, 0, 0, 0, 0, 0, 0, 0, 0},
     {1472, 3499552, 61088, 0, -2510848, 174055680, 0, 0, 0, 0},
     0,
     0},
    {'T',
     "tth_ell_3",
     {3500000, 0, 0, 0, 174078031, 0, 7024413, 134033787, 0, 0, 0, 0, 0, 0, 0, 0},
     {4416, 3496032, 183200, 0, -7529472, 173880832, 0, 0, 0, 0},
     0,
     0},
    {'T',
     "tth_leo_355",
     {3339068, 0, 0, 0, 162021496, 0, -11697846, 133706989, 0, 0, 0, 0, 0, 0, 0, 0},
     {535572, 3326368, -291040, 0, 14121472, 161405184, 0, 0, 0, 0},
     0,
     0},
    {'T',
     "tth_leo_358",
     {3339068, 0, 0, 0, 162021496, 0, -4684131, 134135966, 0, 0, 0, 0, 0, 0, 0, 0},
     {540098, 3337056, -116544, 0, 5654016, 161922816, 0, 0, 0, 0},
     0,
     1},
    {'T',
     "tth_leo_359",
     {3339068, 0, 0, 0, 162021496, 0, -2342422, 134197286, 0, 0, 0, 0, 0, 0, 0, 0},
     {541607, 3338560, -58240, 0, 2827776, 161996800, 0, 0, 0, 0},
     0,
     1},
    {'T',
     "tth_leo_359p5",
     {3339068, 0, 0, 0, 162021496, 0, -1171256, 134212617, 0, 0, 0, 0, 0, 0, 0, 0},
     {542361, 3338976, -29152, 0, 1414144, 162016000, 0, 0, 0, 0},
     0,
     1},
    {'T',
     "tth_leo_359p9",
     {3339068, 0, 0, 0, 162021496, 0, -234254, 134217524, 0, 0, 0, 0, 0, 0, 0, 0},
     {542964, 3339072, -5856, 0, 283648, 162021376, 0, 0, 0, 0},
     0,
     1},
    {'T',
     "tth_leo_0p3",
     {3339068, 0, 0, 0, 162021496, 0, 702759, 134215888, 0, 0, 0, 0, 0, 0, 0, 0},
     {452, 3339040, 17472, 0, -848384, 162019072, 0, 0, 0, 0},
     0,
     0},
    {'T',
     "tth_leo_1",
     {3339068, 0, 0, 0, 162021496, 0, 2342422, 134197286, 0, 0, 0, 0, 0, 0, 0, 0},
     {1509, 3338560, 58304, 0, -2827776, 161996800, 0, 0, 0, 0},
     0,
     0},
    {'T',
     "tth_leo_3",
     {3339068, 0, 0, 0, 162021496, 0, 7024413, 134033787, 0, 0, 0, 0, 0, 0, 0, 0},
     {4526, 3334496, 174752, 0, -8479744, 161799424, 0, 0, 0, 0},
     0,
     0},

    {'R',
     "trad_ell_a",
     {3500000, 0, 0, 0, 174078031, 0, 4430380, 8192, 0, 0, 0, 0, 0, 0, 0, 0},
     {179775, -930400, 4331584, 0, -140658176, -256, 0, -28185752, 131224854,
      0},
     0,
     0},
    {'R',
     "trad_ell_beyond",
     {3500000, 0, 0, 0, 174078031, 0, 13291139, -8192, 1, 0, 0, 0, 0, 0, 0, 0},
     {415036, 0, 0, 238177272, 415036, 0, 0, -134217727, 0, 0},
     0,
     16},
    {'R',
     "trad_circ",
     {3339068, 0, 0, 0, 162021496, 0, 3339068, 8192, 1, 0, 0, 0, 0, 0, 0, 0},
     {0, 0, 0, 0, 268435455, 0, 0, 0, 0, 0},
     4,
     0},
    {'R',
     "trad_inbound",
     {3500000, 0, 0, 0, 174078031, 0, 4200000, -8192, 0, 0, 0, 0, 0, 0, 0, 0},
     {682491, 166656, -4196672, 0, 143754240, 35919616, 0, 5326096, -134112010,
      0},
     0,
     0},
    {'A',
     "aps_ell",
     {0, 3500000, 0, 0, 0, 174078031, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     {5360757, 3500003, 7046398, 20300432, 3313500, 0, 0, 0, 0, 0},
     0,
     0},
    {'A',
     "aps_hyp",
     {0, 3500000, 0, 0, 0, 261117046, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     {268435455, 3499999, 57797510, 45675972, -3030385, 0, 0, 0, 0, 0},
     0,
     0},
    {'A',
     "aps_moon",
     {1, 3800000, 0, 0, 0, 141489272, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     {4667964, 3800003, 3439290, 18496879, 3764388, 0, 0, 0, 0, 0},
     0,
     0},
    {'A',
     "aps_circ",
     {0, 3339068, 0, 0, 0, 162021496, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     {3339072, 3339064, 0, 16777212, 4194305, 0, 0, 0, 0, 0},
     0,
     0},
};

static int g_checks;   /* checks made    */
static int g_failures; /* checks failed  */

static void      cmp(const char *tag, int idx, agc_dp_t got, agc_dp_t want);
static void      cmp_flag(const char *tag, const char *name, bool got, bool want);
static agc_vec_t vec3(const agc_dp_t *a);
static int       run_kepler(const oracle_row_t *r, agc_dp_t got[10]);
static void      run_lambert(const oracle_row_t *r, agc_dp_t got[10], bool *solnsw);
static void      run_time(const oracle_row_t *r, agc_dp_t got[10], agc_conic_time_out_t *o);
static void      check_time(const oracle_row_t *r, agc_dp_t got[10], int *n);
static void      check_row(const oracle_row_t *r);

/**********************************************************************
 * AGC: none (test driver) -- run every row, CSM rows on the CSM build
 * and L rows on the LM build
 *
 * FUNCTION NAME: main
 *
 * ARGUMENTS:
 *   none
 *
 * RETURNS: int -- 0 when every check passed, 1 otherwise
 **********************************************************************/
int main(void)
{
    agc_state_init(AGC_VEHICLE_CSM);

    for (size_t i = 0; i < sizeof (ROWS) / sizeof (ROWS[0]); i++)
    {
        if (ROWS[i].kind != 'L')
        {
            check_row(&ROWS[i]);
        }
    }

    agc_state_init(AGC_VEHICLE_LM);

    for (size_t i = 0; i < sizeof (ROWS) / sizeof (ROWS[0]); i++)
    {
        if (ROWS[i].kind == 'L')
        {
            check_row(&ROWS[i]);
        }
    }

    (void) printf("oracle_conic: %d checks, %d failures\n", g_checks, g_failures);

    return ((g_failures == 0) ? 0 : 1); /* exit status */
}

/**********************************************************************
 * AGC: none (test support) -- the host assert hook: print and go on
 *
 * FUNCTION NAME: agc_assert_report
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description
 *   --------  ------------  ---  ---------------------------------
 *   file      const char *  I    source file of the failed assert
 *   line      int           I    its line
 *   expr      const char *  I    its expression text
 *
 * RETURNS: bool -- false (the assertion stays failed)
 **********************************************************************/
bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) printf("assert %s:%d %s\n", file, line, expr);

    return (false);
}

/**********************************************************************
 * AGC: none (test support) -- compare one DP output with the AGC's
 *
 * FUNCTION NAME: cmp
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description
 *   --------  ------------  ---  ---------------------------------
 *   tag       const char *  I    case name
 *   idx       int           I    output index in exp[]
 *   got       agc_dp_t      I    C value
 *   want      agc_dp_t      I    AGC value
 **********************************************************************/
static void cmp(const char *tag, int idx, agc_dp_t got, agc_dp_t want)
{
    agc_dp_t d = got - want; /* C - AGC, DP LSB (2^-28 of the output scale) */

    g_checks++;

    if (d != 0)
    {
        g_failures++;
        (void) printf("FAIL %s[%d]: C %ld AGC %ld (diff %ld)\n", tag, idx, (long int) got, (long int) want, (long int) d);
    }
}

/**********************************************************************
 * AGC: none (test support) -- compare one output switch
 *
 * FUNCTION NAME: cmp_flag
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description
 *   --------  ------------  ---  ---------------------------------
 *   tag       const char *  I    case name
 *   name      const char *  I    switch name
 *   got       bool          I    C value
 *   want      bool          I    AGC value
 **********************************************************************/
static void cmp_flag(const char *tag, const char *name, bool got, bool want)
{
    g_checks++;

    if (got != want)
    {
        g_failures++;
        (void) printf("FAIL %s: %s C %d AGC %d\n", tag, name, (int) got, (int) want);
    }
}

/**********************************************************************
 * AGC: none (test support) -- three DP words as a vector
 *
 * FUNCTION NAME: vec3
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description
 *   --------  ----------------  ---  ---------------------------------
 *   a         const agc_dp_t *  I    x, y, z words (scale of the row)
 *
 * RETURNS: agc_vec_t -- the vector, same scale
 **********************************************************************/
static agc_vec_t vec3(const agc_dp_t *a)
{
    return (agc_vec_make(a[0], a[1], a[2]));
}

/**********************************************************************
 * AGC: none (test support) -- KEPPREP + KEPLER (K) or KEPLERN (N) for
 * one row; K: in[0] body, in[1..6] RRECT/VRECT, in[7] TAU.; N: in[0..5]
 * RRECT/VRECT, in[6] TAU., in[7] XKEPNEW
 *
 * FUNCTION NAME: run_kepler
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description
 *   --------  --------------------  ---  ----------------------------
 *   r         const oracle_row_t *  I    the row
 *   got       agc_dp_t []           O    XKEPNEW, RCV, VCV, TC, XPREV,
 *                                        TAU. (AGC scales)
 *
 * RETURNS: int -- number of outputs to compare (10 K, 9 N)
 **********************************************************************/
static int run_kepler(const oracle_row_t *r, agc_dp_t got[10])
{
    agc_interp_t     ip;                                  /* interpreter state */
    agc_kepler_in_t  in;                                  /* KEPLER inputs     */
    agc_kepler_out_t o;                                   /* KEPLER outputs    */
    const bool       prep = (r->kind == 'K');             /* KEPPREP first     */
    const agc_dp_t  *p    = prep ? &r->in[1] : &r->in[0]; /* skip body word */

    agc_interp_init(&ip);
    in.rrect   = vec3(&p[0]);
    in.vrect   = vec3(&p[3]);
    in.tau     = p[6];
    in.tc      = 0;
    in.xprev   = 0;
    in.xkepnew = prep ? 0 : p[7];                                              /* KEPPREP computes it */
    in.body    = (prep && (r->in[0] != 0)) ? AGC_CONIC_MOON : AGC_CONIC_EARTH; /* moon row */

    if (prep)
    {
        agc_conic_kepprep(&ip, &in, &in.xkepnew);
    }

    (void) agc_conic_kepler(&ip, &in, &o);
    agc_dp_t v[10] = {in.xkepnew, o.rcv.c[0], o.rcv.c[1], o.rcv.c[2], o.vcv.c[0], o.vcv.c[1], o.vcv.c[2], o.tc, o.xprev, o.tau}; /* outputs in exp[] order */

    memcpy(got, v, sizeof (v));

    return (prep ? 10 : 9); /* N rows have no XKEPNEW */
}

/**********************************************************************
 * AGC: none (test support) -- LAMBERT for one C or L row
 *
 * FUNCTION NAME: run_lambert
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description
 *   --------  --------------------  ---  ----------------------------
 *   r         const oracle_row_t *  I    the row
 *   got       agc_dp_t []           O    VVEC, VTARGET, COGA, P, R1A
 *   solnsw    bool *                O    SOLNSW
 **********************************************************************/
static void run_lambert(const oracle_row_t *r, agc_dp_t got[10], bool *solnsw)
{
    agc_interp_t      ip;                     /* interpreter state */
    agc_lambert_in_t  in;                     /* LAMBERT inputs    */
    agc_lambert_out_t o;                      /* LAMBERT outputs   */
    const bool        csm = (r->kind == 'C'); /* CSM row           */

    agc_interp_init(&ip);
    in.r1vec       = vec3(&r->in[0]);
    in.r2vec       = vec3(&r->in[3]);
    in.tdesired    = r->in[6];
    in.geomsgn     = (agc_sp_t) r->in[7];
    in.guessw      = (r->in[8] != 0);
    in.coga        = r->in[9];
    in.normsw      = csm && (r->in[10] != 0);                                                                                        /* C: in[10] NORMSW */
    in.un          = csm ? vec3(&r->in[11]) : agc_vec_zero();                                                                        /* C: in[11..13] UN */
    in.vtargtag    = 0;
    in.iterctr     = csm ? (agc_sp_t) AGC_CONIC_LAMBERT_ITERCTR : (agc_sp_t) r->in[10];                                              /* L: in[10] */
    in.body        = AGC_CONIC_EARTH;
    (void) agc_conic_lambert(&ip, &in, &o);
    agc_dp_t v[10] = {o.vvec.c[0], o.vvec.c[1], o.vvec.c[2], o.vtarget.c[0], o.vtarget.c[1], o.vtarget.c[2], o.coga, o.p, o.r1a, 0}; /* outputs in exp[] order */

    memcpy(got, v, sizeof (v));
    *solnsw = o.solnsw;
}

/**********************************************************************
 * AGC: none (test support) -- TIMETHET (T) or TIMERAD (R) for one row
 *
 * FUNCTION NAME: run_time
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description
 *   --------  ----------------------  ---  --------------------------
 *   r         const oracle_row_t *    I    the row
 *   got       agc_dp_t []             O    T, RNEW, VNEW, CSTH, SNTH
 *   o         agc_conic_time_out_t *  O    full output record
 **********************************************************************/
static void run_time(const oracle_row_t *r, agc_dp_t got[10], agc_conic_time_out_t *o)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);

    if (r->kind == 'T')
    {
        agc_timetheta_in_t in; /* TIMETHET inputs */

        in.rvec = vec3(&r->in[0]);
        in.vvec = vec3(&r->in[3]);
        in.snth = r->in[6];
        in.csth = r->in[7];
        in.rvsw = (r->in[8] != 0);
        in.body = AGC_CONIC_EARTH;
        (void) agc_conic_timethet(&ip, &in, o);
    }
    else
    {
        agc_timerad_in_t in; /* TIMERAD inputs */

        in.rvec     = vec3(&r->in[0]);
        in.vvec     = vec3(&r->in[3]);
        in.rdesired = r->in[6];
        in.sgnrdot  = (agc_sp_t) r->in[7];
        in.rvsw     = (r->in[8] != 0);
        in.body     = AGC_CONIC_EARTH;
        (void) agc_conic_timerad(&ip, &in, o);
    }

    agc_dp_t v[10] = { o->t, o->rnew.c[0], o->rnew.c[1], o->rnew.c[2], o->vnew.c[0], o->vnew.c[1], o->vnew.c[2], o->csth, o->snth, 0 }; /* outputs in exp[] order */

    memcpy(got, v, sizeof (v));
}

/**********************************************************************
 * AGC: none (test support) -- flags of a T or R row, CSTH/SNTH of an R
 * row, and how many leading outputs to compare
 *
 * FUNCTION NAME: check_time
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description
 *   --------  --------------------  ---  ----------------------------
 *   r         const oracle_row_t *  I    the row
 *   got       agc_dp_t []           O    outputs (filled by run_time)
 *   n         int *                 O    outputs to compare: 0 after an
 *                                        abort, 1 with RVSW, else 7
 **********************************************************************/
static void check_time(const oracle_row_t *r, agc_dp_t got[10], int *n)
{
    agc_conic_time_out_t o;                                                       /* routine outputs    */
    const bool           rvsw  = (r->in[8] != 0);                                 /* time only        */
    const bool           abort = ((r->fl8 & 0110) != 0) || ((r->fl5 & 004) != 0); /* ABTCONIC   */

    run_time(r, got, &o);
    cmp_flag(r->tag, "INFINFLG", o.infinflg, (r->fl8 & 0100) != 0);
    cmp_flag(r->tag, "COGAFLAG", o.cogaflag, (r->fl8 & 010) != 0);

    if (r->kind == 'R')
    {
        cmp_flag(r->tag, "APSESW", o.apsesw, (r->fl8 & 020) != 0);
        cmp_flag(r->tag, "SOLNSW", o.solnsw, (r->fl5 & 004) != 0);
    }

    if (abort)
    {
        *n = 0;
    }
    else
    {
        *n = rvsw ? 1 : 7; /* T only, or T and the state */
    }

    if (!abort && (r->kind == 'R'))
    {
        cmp(r->tag, 7, got[7], r->exp[7]);
        cmp(r->tag, 8, got[8], r->exp[8]);
    }
}

/**********************************************************************
 * AGC: none (test driver) -- run one row and compare its outputs
 *
 * FUNCTION NAME: check_row
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description
 *   --------  --------------------  ---  ----------------------------
 *   r         const oracle_row_t *  I    the row
 **********************************************************************/
static void check_row(const oracle_row_t *r)
{
    agc_dp_t got[10] = {0}; /* C outputs, exp[] scales  */
    int      n       = 0;   /* outputs to compare       */
    int      first   = 0;   /* first output to compare  */

    if ((r->kind == 'K') || (r->kind == 'N'))
    {
        n     = run_kepler(r, got);
        first = (r->kind == 'K') ? 0 : 1; /* N: no XKEPNEW */
    }
    else if ((r->kind == 'C') || (r->kind == 'L'))
    {
        bool solnsw = false; /* SOLNSW from LAMBERT */

        run_lambert(r, got, &solnsw);
        n = 9;
        cmp_flag(r->tag, "SOLNSW", solnsw, (r->fl5 & 004) != 0);
    }
    else if ((r->kind == 'T') || (r->kind == 'R'))
    {
        check_time(r, got, &n);
    }
    else
    {
        agc_interp_t      ip; /* interpreter state */
        agc_apsides_out_t o;  /* APSIDES outputs   */

        agc_interp_init(&ip);
        agc_conic_apsides(&ip, vec3(&r->in[1]), vec3(&r->in[4]), (r->in[0] != 0) ? AGC_CONIC_MOON : AGC_CONIC_EARTH, &o); /* moon row */
        agc_dp_t v[5] = {o.rapo, o.rperi, o.ecc, o.p, o.r1a};                                                             /* outputs in exp[] order */

        memcpy(got, v, sizeof (v));
        n = 5;
    }

    for (int i = first; i < n; i++)
    {
        cmp(r->tag, i, got[i], r->exp[i]);
    }
}
