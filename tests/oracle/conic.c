/*
 * conic.c -- nav-conic against the real AGC code (yaAGC oracle).
 *
 * Expected values are the dumps in tools/oracle/cases/nav_conic.txt
 * (Comanche055) and nav_conic_lm.txt (Luminary099), produced from
 * nav_conic.spec / nav_conic_lm.spec by tools/oracle/bin/agc_probe; the
 * inputs below are the same DP words the specs poke. Each row:
 *   kind K = KEPPREP + KEPLER, N = KEPLER (KEPLERN entry), C / L = LAMBERT
 *   on the CSM / LM build (C: in[10] NORMSW, in[11..13] UN; L: in[10]
 *   ITERCTR), T = TIMETHET, R = TIMERAD, A = APSIDES;
 *   in[]  inputs (see run_case), exp[] AGC outputs, fl5/fl8 FLAGWRD5/8.
 *
 * Tolerance: 16384 LSB (one single-precision LSB of the DP word). The C
 * model has no ones' complement -0; the AGC's DDV of a dividend whose minor
 * word is -0 (KEPLERN ALPHA for an exactly circular orbit, 00200 77777)
 * comes out 2^-14 low, which moves the KEPLER results of the circular
 * cases by up to ~12000 LSB and the 2.25-period case (period modulo uses
 * ALPHA) much more; that row gets 2^19 LSB.
 */
#include <stdio.h>
#include <string.h>
#include "agc/core/assert.h"
#include "agc/core/interp.h"
#include "agc/nav/conic.h"
#include "agc/core/state.h"

bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void)printf("assert %s:%d %s\n", file, line, expr);
    return false;
}

typedef struct oracle_row {
    char        kind;
    const char *tag;
    agc_dp_t    in[16];
    agc_dp_t    exp[10];
    int         fl5;
    int         fl8;
} oracle_row_t;

static const oracle_row_t ROWS[] = {
    { 'K', "kep_leo_q", { 0, 3339068, 0, 0, 0, 162021496, 0, 135779, 0, 0, 0, 0, 0, 0, 0, 0 }, { 8313381, -96, 3339040, 0, -162033152, -5632, 0, 135779, 8313381, 135779 }, 0, 0 },
    { 'K', "kep_leo_mq", { 0, 3339068, 0, 0, 0, 162021496, 0, -135779, 0, 0, 0, 0, 0, 0, 0, 0 }, { -8313381, -96, -3339040, 0, 162033152, -5632, 0, -135779, -8313381, -135779 }, 0, 0 },
    { 'K', "kep_leo_2p25", { 0, 3339068, 0, 0, 0, 162021496, 0, 1222011, 0, 0, 0, 0, 0, 0, 0, 0 }, { 74820429, 7552, 3339008, 0, -162033152, 363776, 0, 135581, 8301270, 135581 }, 0, 0 },
    { 'K', "kep_ell_3d", { 0, 2500000, 2000000, 1000000, -83886080, 115343360, 52428800, 300000, 0, 0, 0, 0, 0, 0, 0, 0 }, { 20228322, -596128, -2148704, -1026304, 200471552, -42566144, -15451392, 300000, 19578244, 300000 }, 0, 0 },
    { 'K', "kep_hyp", { 0, 3500000, 0, 0, 0, 261117046, 0, 360000, 0, 0, 0, 0, 0, 0, 0, 0 }, { -2783852, -3777792, 15584640, 0, -93211136, 142611712, 0, 360000, 9994392, 360000 }, 0, 0 },
    { 'K', "kep_moon", { 1, 3696000, 0, 0, 0, 136634393, 0, 100000, 0, 0, 0, 0, 0, 0, 0, 0 }, { 4907712, 2350944, 2851904, 0, -105431040, 86909952, 0, 100000, 4907712, 100000 }, 0, 0 },
    { 'N', "kepn_leo_badx", { 3339068, 0, 0, 0, 162021496, 0, 135779, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 0, -96, 3339040, 0, -162033152, -6400, 0, 135779, 8313394, 135779 }, 0, 0 },
    { 'N', "kepn_hyp_badx", { 3500000, 0, 0, 0, 261117046, 0, 360000, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 0, -3777792, 15584640, 0, -93211136, 142611712, 0, 360000, 9994392, 360000 }, 0, 0 },
    { 'N', "kepn_ell_wrongsign", { 2500000, 2000000, 1000000, -83886080, 115343360, 52428800, 300000, -6144000, 0, 0, 0, 0, 0, 0, 0, 0 }, { 0, -596064, -2148704, -1026304, 200473088, -42565888, -15451136, 300000, 19578286, 300000 }, 0, 0 },
    { 'N', "kepn_ell_back", { 2500000, 2000000, 1000000, -83886080, 115343360, 52428800, -700000, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 0, -2838912, -45632, -86752, 36706560, -160080640, -74596608, -236498, -17696804, -236498 }, 0, 0 },
    { 'C', "lam_leo_q", { 3339068, 0, 0, 0, 3339068, 0, 135779, 8192, 1, 0, 0, 0, 0, 0, 0, 0 }, { 850, 162021090, 0, -162021376, -1024, 0, 44, 16777128, 4194326, 0 }, 0, 0 },
    { 'C', "lam_3d", { 3500000, 0, 0, -1000000, 4000000, 500000, 250000, 8192, 1, 0, 0, 0, 0, 0, 0, 0 }, { 44422934, 149455768, 18681972, -116959488, -55258880, -6907392, 2474103, 15197637, 4258699, 0 }, 0, 0 },
    { 'C', "lam_3d_guess", { 3500000, 0, 0, -1000000, 4000000, 500000, 250000, 8192, 0, 1677722, 0, 0, 0, 0, 0, 0 }, { 44422890, 149455800, 18681976, -116959488, -55258880, -6907392, 2474100, 15197642, 4258698, 0 }, 0, 0 },
    { 'C', "lam_gt180", { 3339068, 0, 0, -500000, -3339068, 0, 407337, -8192, 1, 0, 0, 0, 0, 0, 0, 0 }, { 969584, 163227392, 0, 160020736, -21414656, 0, 49829, 17027880, 4131488, 0 }, 0, 0 },
    { 'C', "lam_fast", { 3500000, 0, 0, 0, 3750000, 0, 60000, 8192, 1, 0, 0, 0, 0, 0, 0, 0 }, { -17099908, 26494310, 0, -151897344, 18866176, 0, -5414160, 58271278, -12247649, 0 }, 0, 0 },
    { 'C', "lam_slow170", { 3500000, 0, 0, -3450000, 600000, 0, 900000, 8192, 1, 0, 0, 0, 0, 0, 0, 0 }, { 97880160, 154104406, 0, 70035200, -168517888, 0, 5328065, 15909167, 2806791, 0 }, 0, 0 },
    { 'C', "lam_badguess", { 3500000, 0, 0, -1000000, 4000000, 500000, 250000, 8192, 0, -7549747, 0, 0, 0, 0, 0, 0 }, { 44422790, 149455824, 18681978, -116960000, -55258880, -6907392, 2474094, 15197649, 4258698, 0 }, 0, 0 },
    { 'C', "lam_un_given", { 3500000, 0, 0, -1000000, 4000000, 0, 250000, 8192, 1, 0, 1, 0, 0, 134217728, 0, 0 }, { 44639366, 150175912, 0, -117145344, -57033728, 0, 2493490, 15108380, 4277785, 0 }, 0, 0 },
    { 'C', "lam_un_gt180", { 3339068, 0, 0, -500000, -3339068, 0, 407337, -8192, 1, 0, 1, 0, 0, 134217728, 0, 0 }, { 969584, 163227390, 0, 160020736, -21414656, 0, 49829, 17027880, 4131488, 0 }, 0, 0 },
    { 'L', "lmlam_3d_ctr20", { 3500000, 0, 0, -1000000, 4000000, 500000, 250000, 8192, 1, 0, 20, 0, 0, 0, 0, 0 }, { 44422934, 149455768, 18681972, -116959488, -55258880, -6907392, 2474103, 15197637, 4258699, 0 }, 0, 0 },
    { 'L', "lmlam_3d_ctr5", { 3500000, 0, 0, -1000000, 4000000, 500000, 250000, 8192, 1, 0, 5, 0, 0, 0, 0, 0 }, { 44312330, 149499198, 18687400, -117022976, -55155712, -6894592, 2467226, 15206471, 4258134, 0 }, 4, 0 },
    { 'L', "lmlam_3d_ctr2", { 3500000, 0, 0, -1000000, 4000000, 500000, 250000, 8192, 1, 0, 2, 0, 0, 0, 0, 0 }, { -71615802, 202860506, 25357564, -190512640, 52038400, 6504704, -2938560, 27999209, 529842, 0 }, 4, 0 },
    { 'L', "lmlam_slow_ctr3", { 3500000, 0, 0, -3450000, 600000, 0, 900000, 8192, 1, 0, 3, 0, 0, 0, 0, 0 }, { 118448002, 153242770, 0, 90446336, -171194112, 0, 6483920, 15731761, 2105966, 0 }, 4, 0 },
    { 'T', "tth_leo_90", { 3339068, 0, 0, 0, 162021496, 0, 134217728, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 135779, 0, 3339072, 0, -162021376, 0, 0, 0, 0, 0 }, 0, 0 },
    { 'T', "tth_ell_120", { 3500000, 0, 0, 0, 174078031, 0, 116235962, -67108864, 1, 0, 0, 0, 0, 0, 0, 0 }, { 224865, 0, 0, 119632148, 224865, 204442107, 176229826, 0, 0, 0 }, 0, 0 },
    { 'T', "tth_hyp_inf", { 3500000, 0, 0, 0, 261117046, 0, 23306664, -132178659, 1, 0, 0, 0, 0, 0, 0, 0 }, { 0, 1210952, 5462, 210272256, 0, 204442107, 176229826, 0, 0, 0 }, 0, 64 },
    { 'T', "tth_ell_350", { 3500000, 0, 0, 0, 174078031, 0, -23306664, 132178659, 0, 0, 0, 0, 0, 0, 0, 0 }, { 815328, 3455936, -609408, 0, 24983040, 171892224, 0, 0, 0, 0 }, 0, 0 },
    { 'T', "tth_radial", { 3339068, 0, 0, 104857600, 0, 0, 134217728, 0, 1, 0, 0, 0, 0, 0, 0, 0 }, { 0, 1756768, 0, 0, 0, 204442107, 176229826, 0, 0, 0 }, 0, 8 },
    { 'R', "trad_ell_a", { 3500000, 0, 0, 0, 174078031, 0, 4430380, 8192, 0, 0, 0, 0, 0, 0, 0, 0 }, { 179775, -930400, 4331584, 0, -140658176, -256, 0, -28185752, 131224854, 0 }, 0, 0 },
    { 'R', "trad_ell_beyond", { 3500000, 0, 0, 0, 174078031, 0, 13291139, -8192, 1, 0, 0, 0, 0, 0, 0, 0 }, { 415036, 0, 0, 238177272, 415036, 0, 0, -134217727, 0, 0 }, 0, 16 },
    { 'R', "trad_circ", { 3339068, 0, 0, 0, 162021496, 0, 3339068, 8192, 1, 0, 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 268435455, 0, 0, 0, 0, 0 }, 4, 0 },
    { 'R', "trad_inbound", { 3500000, 0, 0, 0, 174078031, 0, 4200000, -8192, 0, 0, 0, 0, 0, 0, 0, 0 }, { 682491, 166656, -4196672, 0, 143754240, 35919616, 0, 5326096, -134112010, 0 }, 0, 0 },
    { 'A', "aps_ell", { 0, 3500000, 0, 0, 0, 174078031, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 5360757, 3500003, 7046398, 20300432, 3313500, 0, 0, 0, 0, 0 }, 0, 0 },
    { 'A', "aps_hyp", { 0, 3500000, 0, 0, 0, 261117046, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 268435455, 3499999, 57797510, 45675972, -3030385, 0, 0, 0, 0, 0 }, 0, 0 },
    { 'A', "aps_moon", { 1, 3800000, 0, 0, 0, 141489272, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 4667964, 3800003, 3439290, 18496879, 3764388, 0, 0, 0, 0, 0 }, 0, 0 },
};

static int g_checks;
static int g_failures;

static void cmp(const char *tag, int idx, agc_dp_t got, agc_dp_t want, agc_dp_t tol)
{
    g_checks++;
    agc_dp_t d = got - want;
    if (d > tol || d < -tol) {
        g_failures++;
        (void)printf("FAIL %s[%d]: C %ld AGC %ld (diff %ld)\n", tag, idx,
                     (long)got, (long)want, (long)d);
    }
}

static void cmp_flag(const char *tag, const char *name, bool got, bool want)
{
    g_checks++;
    if (got != want) {
        g_failures++;
        (void)printf("FAIL %s: %s C %d AGC %d\n", tag, name, (int)got, (int)want);
    }
}

static agc_vec_t vec3(const agc_dp_t *a)
{
    return agc_vec_make(a[0], a[1], a[2]);
}

static int run_kepler(const oracle_row_t *r, agc_dp_t got[10])
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_kepler_in_t in;
    agc_kepler_out_t o;
    const bool prep = (r->kind == 'K');
    const agc_dp_t *p = prep ? &r->in[1] : &r->in[0];
    in.rrect = vec3(&p[0]);
    in.vrect = vec3(&p[3]);
    in.tau = p[6];
    in.tc = 0;
    in.xprev = 0;
    in.xkepnew = prep ? 0 : p[7];
    in.body = (prep && r->in[0] != 0) ? AGC_CONIC_MOON : AGC_CONIC_EARTH;
    if (prep) {
        (void)agc_conic_kepprep(&ip, &in, &in.xkepnew);
    }
    (void)agc_conic_kepler(&ip, &in, &o);
    agc_dp_t v[10] = { in.xkepnew, o.rcv.c[0], o.rcv.c[1], o.rcv.c[2], o.vcv.c[0],
                       o.vcv.c[1], o.vcv.c[2], o.tc, o.xprev, o.tau };
    memcpy(got, v, sizeof(v));
    return prep ? 10 : 9;
}

static void run_lambert(const oracle_row_t *r, agc_dp_t got[10], bool *solnsw)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_lambert_in_t in;
    agc_lambert_out_t o;
    in.r1vec = vec3(&r->in[0]);
    in.r2vec = vec3(&r->in[3]);
    in.tdesired = r->in[6];
    in.geomsgn = (agc_sp_t)r->in[7];
    in.guessw = (r->in[8] != 0);
    in.coga = r->in[9];
    in.normsw = (r->kind == 'C') && (r->in[10] != 0);   /* C: in[10] NORMSW */
    in.un = (r->kind == 'C') ? vec3(&r->in[11]) : agc_vec_zero();
    in.vtargtag = 0;
    in.iterctr = (r->kind == 'L') ? (agc_sp_t)r->in[10] : (agc_sp_t)AGC_CONIC_LAMBERT_ITERCTR;
    in.body = AGC_CONIC_EARTH;
    (void)agc_conic_lambert(&ip, &in, &o);
    agc_dp_t v[10] = { o.vvec.c[0], o.vvec.c[1], o.vvec.c[2], o.vtarget.c[0],
                       o.vtarget.c[1], o.vtarget.c[2], o.coga, o.p, o.r1a, 0 };
    memcpy(got, v, sizeof(v));
    *solnsw = o.solnsw;
}

static void run_time(const oracle_row_t *r, agc_dp_t got[10], agc_conic_time_out_t *o)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    if (r->kind == 'T') {
        agc_timetheta_in_t in;
        in.rvec = vec3(&r->in[0]);
        in.vvec = vec3(&r->in[3]);
        in.snth = r->in[6];
        in.csth = r->in[7];
        in.rvsw = (r->in[8] != 0);
        in.body = AGC_CONIC_EARTH;
        (void)agc_conic_timethet(&ip, &in, o);
    } else {
        agc_timerad_in_t in;
        in.rvec = vec3(&r->in[0]);
        in.vvec = vec3(&r->in[3]);
        in.rdesired = r->in[6];
        in.sgnrdot = (agc_sp_t)r->in[7];
        in.rvsw = (r->in[8] != 0);
        in.body = AGC_CONIC_EARTH;
        (void)agc_conic_timerad(&ip, &in, o);
    }
    agc_dp_t v[10] = { o->t, o->rnew.c[0], o->rnew.c[1], o->rnew.c[2], o->vnew.c[0],
                       o->vnew.c[1], o->vnew.c[2], o->csth, o->snth, 0 };
    memcpy(got, v, sizeof(v));
}

static void check_row(const oracle_row_t *r)
{
    agc_dp_t got[10] = { 0 };
    const agc_dp_t tol = (strcmp(r->tag, "kep_leo_2p25") == 0) ? 524288 : 16384;
    int n = 0;
    int first = 0;
    if (r->kind == 'K' || r->kind == 'N') {
        n = run_kepler(r, got);
        first = (r->kind == 'K') ? 0 : 1;
    } else if (r->kind == 'C' || r->kind == 'L') {
        bool solnsw = false;
        run_lambert(r, got, &solnsw);
        n = 9;
        cmp_flag(r->tag, "SOLNSW", solnsw, (r->fl5 & 004) != 0);
    } else if (r->kind == 'T' || r->kind == 'R') {
        agc_conic_time_out_t o;
        run_time(r, got, &o);
        const bool rvsw = (r->in[8] != 0);
        const bool abort = (r->fl8 & 0110) != 0 || (r->fl5 & 004) != 0;
        cmp_flag(r->tag, "INFINFLG", o.infinflg, (r->fl8 & 0100) != 0);
        cmp_flag(r->tag, "COGAFLAG", o.cogaflag, (r->fl8 & 010) != 0);
        if (r->kind == 'R') {
            cmp_flag(r->tag, "APSESW", o.apsesw, (r->fl8 & 020) != 0);
            cmp_flag(r->tag, "SOLNSW", o.solnsw, (r->fl5 & 004) != 0);
        }
        n = abort ? 0 : (rvsw ? 1 : 7);
        if (!abort && r->kind == 'R') {
            cmp(r->tag, 7, got[7], r->exp[7], tol);
            cmp(r->tag, 8, got[8], r->exp[8], tol);
        }
    } else {
        agc_interp_t ip;
        agc_interp_init(&ip);
        agc_apsides_out_t o;
        (void)agc_conic_apsides(&ip, vec3(&r->in[1]), vec3(&r->in[4]),
                                (r->in[0] != 0) ? AGC_CONIC_MOON : AGC_CONIC_EARTH, &o);
        agc_dp_t v[5] = { o.rapo, o.rperi, o.ecc, o.p, o.r1a };
        memcpy(got, v, sizeof(v));
        n = 5;
    }
    for (int i = first; i < n; i++) {
        cmp(r->tag, i, got[i], r->exp[i], tol);
    }
}

int main(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    for (size_t i = 0; i < sizeof(ROWS) / sizeof(ROWS[0]); i++) {
        if (ROWS[i].kind != 'L') {
            check_row(&ROWS[i]);
        }
    }
    agc_state_init(AGC_VEHICLE_LM);
    for (size_t i = 0; i < sizeof(ROWS) / sizeof(ROWS[0]); i++) {
        if (ROWS[i].kind == 'L') {
            check_row(&ROWS[i]);
        }
    }
    (void)printf("oracle_conic: %d checks, %d failures\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
