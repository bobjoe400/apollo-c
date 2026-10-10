/*
 * numeric.c -- unit tests for the numeric core (agc_word.h,
 * agc_interp.h, agc_constants.h). Returns non-zero on failure.
 *
 * Bit-exact expectations come from yaAGC runs of the real Comanche055 /
 * Luminary099 ropes (tools/oracle/cases/interp_sqrt_sin.txt, spsin.txt and
 * core_numeric.txt). Accuracy against libm, measured over 2^21 arguments:
 *   SIN/COS  |err| <= 2.1e-8   (result scaled at 2; ~5.4 LSB of 2^-28)
 *   ASIN/ACOS|err| <= 1.1e-8   revolutions
 *   SQRT     |err| <= 2.9e-8   (x in (0,1))
 *   SPSIN/SPCOS |err| <= 2.6e-4 (~4.2 LSB of 2^-14)
 *   UNIT     |err| <= 2.5e-8 per component
 */
#include <math.h>
#include <stdio.h>
#include "agc/core/constants.h"
#include "agc/core/interp.h"
#include "agc/core/word.h"

#define PI 3.14159265358979323846

static int failures;

static void check(int cond, const char *what, long long got, long long exp)
{
    if (!cond) {
        failures++;
        printf("FAIL %s: got %lld expected %lld\n", what, got, exp);
    }
}

static void eq(const char *what, long long got, long long exp)
{
    check(got == exp, what, got, exp);
}

static void near(const char *what, double got, double exp, double tol)
{
    if (fabs(got - exp) > tol) {
        failures++;
        printf("FAIL %s: got %.12g expected %.12g (tol %.3g)\n", what, got, exp, tol);
    }
}

static agc_dp_t oct2(int hi, int lo)          /* AGC words -> DP value     */
{
    int h = (hi < 040000) ? hi : -(077777 - hi);
    int l = (lo < 040000) ? lo : -(077777 - lo);
    return agc_dp_pack((agc_sp_t)h, (agc_sp_t)l);
}

static void test_words(void)
{
    agc_ovf_t ovf = 0;
    agc_dp_t v = oct2(012345, 054321);
    eq("pack hi", agc_dp_high(oct2(012345, 012345)), 012345);
    eq("pack lo", agc_dp_low(oct2(012345, 012345)), 012345);
    eq("disagreeing pair value", v, 012345 * 16384 - (077777 - 054321));
    eq("split agrees", agc_dp_high(v) >= 0 && agc_dp_low(v) >= 0, 1);
    eq("from_sp", agc_dp_from_sp(AGC_SP_HALF), AGC_DP_HALF);
    eq("tp_to_dp truncates", agc_tp_to_dp(-((agc_tp_t)5 * 16384 + 16383)), -5);
    eq("tp_round_dp up", agc_tp_round_dp((agc_tp_t)5 * 16384 + 8192), 6);
    eq("tp_round_dp down", agc_tp_round_dp((agc_tp_t)5 * 16384 + 8191), 5);
    eq("tp_round_dp neg", agc_tp_round_dp(-((agc_tp_t)5 * 16384 + 8192)), -6);
    eq("from_double", agc_dp_from_double(0.25, 0), AGC_DP_QUARTER);
    eq("from_double scale", agc_dp_from_double(3.0, 3), (agc_dp_t)(0.375 * 268435456.0));
    eq("from_double clamp", agc_dp_from_double(2.0, 0), AGC_DP_MAX);
    near("to_double", agc_dp_to_double(AGC_DP_HALF, 5), 16.0, 0.0);
    eq("sp_add sat", agc_sp_add(AGC_SP_MAX, 1, &ovf), AGC_SP_MAX);
    eq("sp_add ovf", ovf, 1);
    eq("dp_sub sat", agc_dp_sub_raw(AGC_DP_MIN, 5, &ovf), AGC_DP_MIN);
    eq("dp_sub ovf", ovf, -1);
    eq("dp_add ok", agc_dp_add_raw(5, 7, &ovf), 12);
    eq("dp_add no ovf", ovf, 0);
    eq("shl_raw sat", agc_dp_shl_raw(AGC_DP_HALF, 1, &ovf), AGC_DP_MAX);
    eq("shl_raw ovf", ovf, 1);
    eq("sp_mul", agc_sp_mul(AGC_SP_HALF, -AGC_SP_HALF), -(AGC_DP_QUARTER));
    eq("sign", agc_dp_sign(7, -1), -7);
    eq("abs", agc_dp_abs(-7), 7);
    eq("sgn", agc_dp_sgn(-3) + agc_dp_sgn(0) + 10 * agc_dp_sgn(9), 9);
    eq("round to SP", agc_dp_round(3 * 16384 + 8192), 4 * 16384);
    eq("round to SP neg", agc_dp_round(-(3 * 16384 + 8191)), -3 * 16384);
}

static void test_shifts(void)
{
    agc_interp_t ip;
    int sh = 0;
    agc_interp_init(&ip);
    eq("SR truncates", agc_dp_shr(-7, 1), -3);
    eq("SR 14", agc_dp_shr(5 * 16384 + 3, 14), 5);
    eq("SR 29", agc_dp_shr(AGC_DP_MAX, 29), 0);
    eq("SRR rounds", agc_dp_shr_round(-7, 1), -4);
    eq("SRR 14", agc_dp_shr_round(5 * 16384 + 8192, 14), 6);
    eq("SL", agc_dp_shl(&ip, 3, 4), 48);
    eq("SL no ovf", ip.ovfind, 0);
    eq("SL wraps", agc_dp_shl(&ip, AGC_DP_QUARTER * 3, 1), AGC_DP_HALF);
    eq("SL ovf", agc_interp_test_ovf(&ip), 1);
    eq("BOV clears", agc_interp_test_ovf(&ip), 0);
    eq("NORM", agc_dp_norm(3, &sh), 3 << 26);
    eq("NORM count", sh, 26);
    eq("NORM zero", agc_dp_norm(0, &sh) + sh, 0);
    eq("VSR rounds", agc_vec_shr(agc_vec_make(-7, 7, 6), 1).c[0], -4);
    eq("VSL wraps", agc_vec_shl(&ip, agc_vec_make(0, AGC_DP_HALF, 1), 1).c[1], 0);
    eq("VSL ovf", ip.ovfind, 1);
    eq("TP shift right", (long long)agc_tp_shift(&ip, (agc_tp_t)1 << 40, -20), 1LL << 20);
    eq("TP norm", (long long)agc_tp_norm(1, &sh), 1LL << 41);
    eq("TP norm count", sh, 41);
}

static void test_add_mul(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    /* DAD wraps like the AGC: .75 + .5 = 1.25 -> .25, OVFIND = +1.        */
    eq("DAD wrap", agc_dp_add(&ip, 3 * AGC_DP_QUARTER, AGC_DP_HALF), AGC_DP_QUARTER);
    eq("DAD ovf", ip.ovfind, 1);
    agc_interp_init(&ip);
    eq("DSU wrap", agc_dp_sub(&ip, -3 * AGC_DP_QUARTER, AGC_DP_HALF), -AGC_DP_QUARTER);
    eq("DSU ovf", ip.ovfind, -1);
    agc_interp_init(&ip);
    eq("DAD plain", agc_dp_add(&ip, 100, -300), -200);
    eq("TAD", (long long)agc_tp_add(&ip, 5, -7), -2);
    eq("no ovf", ip.ovfind, 0);
    /* DMP of 1/2 by (1/2 + 2^-28) = 1/4 + 2^-29: DMPR rounds the exact
     * half away from zero (ROUNDSUB: DOUBLE of MPAC+2 overflows).         */
    eq("DMP tp", (long long)agc_dp_mul_tp(AGC_DP_HALF, AGC_DP_HALF + 1),
       ((long long)1 << 40) + (1LL << 13));
    eq("DMPR", agc_dp_mul(&ip, AGC_DP_HALF, AGC_DP_HALF + 1), AGC_DP_QUARTER + 1);
    eq("DMP trunc", agc_dp_mul_trunc(AGC_DP_HALF, AGC_DP_HALF + 1), AGC_DP_QUARTER);
    eq("DMPR rounds up", agc_dp_mul(&ip, AGC_DP_HALF, 3), 2);
    eq("DMPR rounds away", agc_dp_mul(&ip, AGC_DP_HALF, -3), -2);
    eq("DMP truncates", agc_dp_mul_trunc(AGC_DP_HALF, -3), -1);
    eq("DSQ", agc_dp_sq(&ip, AGC_DP_HALF), AGC_DP_QUARTER);
    eq("DMPNSUB", agc_dp_mul_int(1000, 7), 7000);
    eq("SHORTMP", (long long)agc_tp_mul_sp((agc_tp_t)1 << 41, AGC_SP_HALF), 1LL << 40);
    eq("ROUND", agc_tp_round(&ip, ((agc_tp_t)7 << 14) + 8192), 8);
}

static void test_divide(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    eq("DDV", agc_dp_div(&ip, AGC_DP_QUARTER, AGC_DP_HALF), AGC_DP_HALF);
    eq("DDV signs", agc_dp_div(&ip, -AGC_DP_QUARTER, AGC_DP_HALF), -AGC_DP_HALF);
    eq("DDV no ovf", ip.ovfind, 0);
    eq("DDV 1/3", agc_dp_div(&ip, 1, 3), 89478485);  /* (2^28 - 1) / 3   */
    eq("DDV overflow", agc_dp_div(&ip, AGC_DP_HALF, -AGC_DP_QUARTER), AGC_DP_MIN);
    eq("DDV ovf = +1", ip.ovfind, 1);
    agc_interp_init(&ip);
    eq("DDV by zero", agc_dp_div(&ip, -5, 0), AGC_DP_MIN);
    eq("DDV by zero ovf", ip.ovfind, 1);
    agc_interp_init(&ip);
    eq("DDV 0/x", agc_dp_div(&ip, 0, 12345), 0);
    eq("DDV equal", agc_dp_div(&ip, 99, 99), AGC_DP_MAX);
    eq("DDV equal ovf", ip.ovfind, 1);
    agc_interp_init(&ip);
    near("DDV accuracy", agc_dp_to_double(agc_dp_div(&ip, 123456789, 223456789), 0),
         123456789.0 / 223456789.0, 2.0e-8);
    /* MAXDV: equal major words, dividend minor word smaller.              */
    near("MAXDV", agc_dp_to_double(agc_dp_div(&ip, 9000 * 16384 + 5, 9000 * 16384 + 900), 0),
         (9000.0 * 16384 + 5) / (9000.0 * 16384 + 900), 2.0e-8);
    eq("no ovf after MAXDV", ip.ovfind, 0);
}

static void test_sqrt_trig(void)
{
    agc_interp_t ip;
    agc_dp_t r = 0;
    int k;
    agc_interp_init(&ip);
    /* yaAGC (interp_sqrt_sin.txt).                                         */
    eq("SQRT .5", agc_dp_sqrt(&ip, AGC_DP_HALF), oct2(026501, 007463));
    eq("SQRT .25", agc_dp_sqrt(&ip, AGC_DP_QUARTER), AGC_DP_HALF);
    eq("SQRT 2^-28", agc_dp_sqrt(&ip, 1), oct2(000001, 000000));
    eq("SQRT max", agc_dp_sqrt(&ip, AGC_DP_MAX), AGC_DP_MAX);
    eq("SIN 1/8", agc_dp_sin(&ip, 04000 * 16384), oct2(013240, 023631));
    eq("SIN 1/4", agc_dp_sin(&ip, 010000 * 16384), AGC_DP_HALF);
    eq("SIN -2047", agc_dp_sin(&ip, oct2(074000, 0)), oct2(064541, 063230));
    eq("COS 1/8", agc_dp_cos(&ip, 04000 * 16384), oct2(013240, 023631));
    eq("COS 0", agc_dp_cos(&ip, 0), AGC_DP_HALF);
    eq("SQRT abort", agc_dp_sqrt_chk(&ip, -2 * 16384, &r), AGC_STATUS_POODOO);
    eq("SQRT tiny neg", (long long)agc_dp_sqrt_chk(&ip, -5, &r) * 100 + r, 0);
    eq("ACOS 0", agc_dp_acos(&ip, 0), AGC_DP_QUARTER);
    eq("ASIN 0", agc_dp_asin(&ip, 0), 0);
    eq("ACOS alarm", agc_dp_acos_chk(&ip, AGC_DP_HALF + 2 * 16384, &r), AGC_STATUS_ALARM);
    eq("ACOS alarm value", r, 0);
    eq("ASIN -range", agc_dp_asin_chk(&ip, -AGC_DP_MAX, &r), AGC_STATUS_ALARM);
    eq("ASIN -range value", r, -AGC_DP_QUARTER);
    for (k = -1000; k <= 1000; k++) {
        double x = k / 1000.0 * 0.999;           /* revolutions             */
        agc_dp_t a = agc_dp_from_double(x, 0);
        double xa = agc_dp_to_double(a, 0);
        near("SIN", agc_dp_to_double(agc_dp_sin(&ip, a), 0), 0.5 * sin(2 * PI * xa), 2.1e-8);
        near("COS", agc_dp_to_double(agc_dp_cos(&ip, a), 0), 0.5 * cos(2 * PI * xa), 2.1e-8);
        if (fabs(xa) <= 0.5) {
            near("ASIN", agc_dp_to_double(agc_dp_asin(&ip, a), 0), asin(2 * xa) / (2 * PI), 1.2e-8);
            near("ACOS", agc_dp_to_double(agc_dp_acos(&ip, a), 0), acos(2 * xa) / (2 * PI), 1.2e-8);
        }
        if (xa > 0) {
            near("SQRT", agc_dp_to_double(agc_dp_sqrt(&ip, a), 0), sqrt(xa), 2.9e-8);
        }
    }
    for (k = -16383; k <= 16383; k += 7) {
        near("SPSIN", agc_sp_sin((agc_sp_t)k) / 16384.0, sin(PI * k / 16384.0), 2.6e-4);
        near("SPCOS", agc_sp_cos((agc_sp_t)k) / 16384.0, cos(PI * k / 16384.0), 2.6e-4);
    }
    eq("SPSIN 1/2", agc_sp_sin(AGC_SP_HALF), AGC_SP_MAX);  /* ARG90 path     */
    eq("SPCOS 0", agc_sp_cos(0), AGC_SP_MAX);
    eq("no ovf from trig", ip.ovfind, 0);
}

static void test_vectors(void)
{
    agc_interp_t ip;
    agc_dp_t lv = 0;
    agc_dp_t lvsq = 0;
    agc_vec_t u = agc_vec_make(agc_dp_from_double(0.3, 0), agc_dp_from_double(-0.2, 0),
                               agc_dp_from_double(0.1, 0));
    agc_vec_t v = agc_vec_make(agc_dp_from_double(0.05, 0), agc_dp_from_double(0.4, 0),
                               agc_dp_from_double(-0.25, 0));
    agc_vec_t r;
    agc_mat_t m = agc_mat_identity_half();
    agc_mat_t mm;
    int k;
    agc_interp_init(&ip);
    near("DOT", agc_dp_to_double(agc_vec_dot(&ip, u, v), 0),
         0.3 * 0.05 - 0.2 * 0.4 - 0.1 * 0.25, 1.0e-8);
    r = agc_vec_cross(&ip, u, v);
    near("VXV x", agc_dp_to_double(r.c[0], 0), -0.2 * -0.25 - 0.1 * 0.4, 1.0e-8);
    near("VXV y", agc_dp_to_double(r.c[1], 0), 0.1 * 0.05 - 0.3 * -0.25, 1.0e-8);
    near("VXV z", agc_dp_to_double(r.c[2], 0), 0.3 * 0.4 - -0.2 * 0.05, 1.0e-8);
    near("VSQ", agc_dp_to_double(agc_vec_sq(&ip, u), 0), 0.14, 1.0e-8);
    near("ABVAL", agc_dp_to_double(agc_vec_abval(&ip, u), 0), sqrt(0.14), 3.0e-8);
    r = agc_vec_unit(&ip, u, &lv, &lvsq);
    for (k = 0; k < AGC_VEC_N; k++) {
        near("UNIT", agc_dp_to_double(r.c[k], 0),
             agc_dp_to_double(u.c[k], 0) / (2.0 * sqrt(0.14)), 2.5e-8);
    }
    near("UNIT LV", agc_dp_to_double(lv, 0), sqrt(0.14), 3.0e-8);
    near("UNIT LVSQUARE", agc_dp_to_double(lvsq, 0), 0.14, 1.0e-8);
    eq("UNIT no ovf", ip.ovfind, 0);
    r = agc_vec_unit(&ip, agc_vec_zero(), &lv, &lvsq);
    eq("UNIT zero -> DPPOSMAX", r.c[0], AGC_DP_MAX);
    eq("UNIT zero ovf", agc_interp_test_ovf(&ip), 1);
    r = agc_mat_vec(&ip, &m, u);
    eq("MXV I/2", r.c[1], u.c[1] / 2);
    m.row[0] = v;
    r = agc_mat_vec(&ip, &m, u);
    near("MXV row", agc_dp_to_double(r.c[0], 0), 0.3 * 0.05 - 0.2 * 0.4 - 0.1 * 0.25, 1.0e-8);
    r = agc_vec_mat(&ip, u, &m);
    near("VXM col", agc_dp_to_double(r.c[0], 0), 0.3 * 0.05, 1.0e-8);
    mm = agc_mat_mul(&ip, &m, &m);
    near("MXM3", agc_dp_to_double(mm.row[0].c[0], 0), 0.05 * 0.05, 1.0e-8);
    r = agc_vec_scale(&ip, u, AGC_DP_HALF);
    eq("VXSC", r.c[2], (u.c[2] + 1) / 2);
    r = agc_vec_div(&ip, v, AGC_DP_HALF);
    eq("V/SC", r.c[0], 2 * v.c[0]);
    eq("V/SC no ovf", ip.ovfind, 0);
    r = agc_vec_div(&ip, v, AGC_DP_QUARTER);           /* y: .4 / .25       */
    eq("V/SC ovf", ip.ovfind, 1);
    eq("V/SC ovf slot", r.c[0], AGC_DP_MAX);
    agc_interp_init(&ip);
    r = agc_vec_proj(&ip, u, agc_k_unitx());
    near("VPROJ", agc_dp_to_double(r.c[0], 0), 0.3 * 0.5 * 0.5, 1.0e-8);
    r = agc_vec_add(&ip, u, agc_vec_neg(u));
    eq("VAD", r.c[0] + r.c[1] + r.c[2], 0);
    r = agc_vec_sub(&ip, u, u);
    eq("VSU", r.c[2], 0);
}

static void test_constants(void)
{
    eq("POSMAX", AGC_K_POSMAX, AGC_SP_MAX);
    eq("NEGMAX", AGC_K_NEGMAX, AGC_SP_MIN);
    eq("NEG1/2", AGC_K_NEG1_2, -AGC_SP_HALF);
    eq("C1/2", AGC_K_C1_2, 031103);
    eq("C3/2", AGC_K_C3_2, -(077777 - 065552));
    eq("-ENDERAS", AGC_K_MINUS_ENDERAS, -2001);
    eq("V06N22", AGC_K_NEW_GIMBAL_ANGLES_DISPLAY, 6 * 128 + 22);
    eq("DPPOSMAX", AGC_K_DPPOSMAX, AGC_DP_MAX);
    eq("DP1/4TH", AGC_K_DP1_4TH, AGC_DP_QUARTER);
    eq("UNITY", agc_k_unity().c[1], AGC_DP_HALF);
    eq("UNITZ", agc_k_unitz().c[2] + agc_k_unitz().c[0], AGC_DP_HALF);
    eq("ZEROVEC", agc_k_zerovec().c[1], 0);
}

int main(void)
{
    test_words();
    test_shifts();
    test_add_mul();
    test_divide();
    test_sqrt_trig();
    test_vectors();
    test_constants();
    if (failures != 0) {
        printf("test_core_numeric: %d failures\n", failures);
        return 1;
    }
    printf("test_core_numeric: all passed\n");
    return 0;
}
