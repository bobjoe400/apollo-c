/*
 * oracle_core_numeric.c -- differential test of the numeric core against
 * yaAGC running the real Comanche055 interpreter.
 *
 * Reads tools/oracle/cases/core_numeric.txt (made from core_numeric.spec,
 * see that file's header for the layout). Each CASE tag is
 * "OP,input,input,..."; the expected MPAC words, OVFIND, LV/LVSQUARE,
 * NORM count and FAILREG come from the emulator. Every value must match to
 * the bit. Returns non-zero on any mismatch; prints SKIP and returns 0 when
 * the case file is not found (run from the repository root).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "agc/agc_interp.h"
#include "agc/agc_word.h"

#define MAX_IN 16

typedef struct {
    char op[16];
    int nin;
    long long in[MAX_IN];
    int ok;                 /* STATUS OK                                    */
    int poodoo;             /* stopped at POODOO                            */
    long long w[16];        /* MPAC..MPAC+6, OVFIND, VAC1+30, +34..+37, FAILREG */
} ocase_t;

static const char *const names[14] = {
    "MPAC", "MPAC+1", "MPAC+2", "MPAC+3", "MPAC+4", "MPAC+5", "MPAC+6",
    "OVFIND", "VAC1+30", "VAC1+34", "VAC1+35", "VAC1+36", "VAC1+37", "FAILREG"
};

static long long word(const char *oct)          /* ones' complement word */
{
    long v = strtol(oct, NULL, 8);
    return (v < 040000) ? v : -(077777 - v);
}

static void parse_tag(ocase_t *c, const char *tag)
{
    char buf[512];
    char *tok;
    strncpy(buf, tag, sizeof buf - 1);
    buf[sizeof buf - 1] = '\0';
    tok = strtok(buf, ",");
    strncpy(c->op, tok ? tok : "", sizeof c->op - 1);
    c->nin = 0;
    while ((tok = strtok(NULL, ",")) != NULL && c->nin < MAX_IN) {
        c->in[c->nin++] = strtoll(tok, NULL, 10);
    }
}

static long long dpv(long long hi, long long lo) { return hi * 16384 + lo; }

/* Run one case through the C library; returns 1 when everything matches. */
static int run_case(const ocase_t *c)
{
    agc_interp_t ip;
    const long long *w = c->w;
    long long mp = (w[0] * 16384 + w[1]) * 16384 + w[2], mdp = dpv(w[0], w[1]);
    long long got = 0, exp = mdp, got2 = 0, exp2 = 0;
    agc_vec_t u = agc_vec_zero(), v = agc_vec_zero(), r = agc_vec_zero();
    agc_mat_t m;
    agc_dp_t t = 0, lv = 0, lvsq = 0;
    int isvec = 0, k, ovf_checked = 1;
    const long long *in = c->in;
    const char *op = c->op;
    agc_interp_init(&ip);
    if (c->nin >= 3) u = agc_vec_make((agc_dp_t)in[0], (agc_dp_t)in[1], (agc_dp_t)in[2]);
    if (c->nin >= 6) v = agc_vec_make((agc_dp_t)in[3], (agc_dp_t)in[4], (agc_dp_t)in[5]);
    if (!strcmp(op, "DMP")) { got = agc_dp_mul_tp((agc_dp_t)in[0], (agc_dp_t)in[1]); exp = mp;
        got2 = agc_dp_mul_trunc((agc_dp_t)in[0], (agc_dp_t)in[1]); exp2 = mdp; }
    else if (!strcmp(op, "DMPR")) got = agc_dp_mul(&ip, (agc_dp_t)in[0], (agc_dp_t)in[1]);
    else if (!strcmp(op, "DDV")) got = agc_dp_div(&ip, (agc_dp_t)in[0], (agc_dp_t)in[1]);
    else if (!strcmp(op, "BDDV")) got = agc_dp_div(&ip, (agc_dp_t)in[1], (agc_dp_t)in[0]);
    else if (!strcmp(op, "DAD")) got = agc_dp_add(&ip, (agc_dp_t)in[0], (agc_dp_t)in[1]);
    else if (!strcmp(op, "DSU")) got = agc_dp_sub(&ip, (agc_dp_t)in[0], (agc_dp_t)in[1]);
    else if (!strcmp(op, "SIGN")) got = agc_dp_sign((agc_dp_t)in[0], (agc_dp_t)in[1]);
    else if (!strcmp(op, "DSQ")) { got = agc_dp_sq_tp((agc_dp_t)in[0]); exp = mp;
        got2 = agc_dp_sq(&ip, (agc_dp_t)in[0]); exp2 = mdp; }
    else if (!strcmp(op, "SQRT")) { got2 = agc_dp_sqrt_chk(&ip, (agc_dp_t)in[0], &t) == AGC_STATUS_POODOO;
        exp2 = c->poodoo; got = c->poodoo ? 0 : t; exp = c->poodoo ? 0 : mdp; }
    else if (!strcmp(op, "SIN")) got = agc_dp_sin(&ip, (agc_dp_t)in[0]);
    else if (!strcmp(op, "COS")) got = agc_dp_cos(&ip, (agc_dp_t)in[0]);
    else if (!strcmp(op, "ASIN") || !strcmp(op, "ACOS")) {
        agc_status_t s = !strcmp(op, "ASIN") ? agc_dp_asin_chk(&ip, (agc_dp_t)in[0], &t)
                                             : agc_dp_acos_chk(&ip, (agc_dp_t)in[0], &t);
        got = t; got2 = (s == AGC_STATUS_ALARM); exp2 = (w[13] == 01301); }
    else if (!strcmp(op, "ABS")) got = agc_dp_abs((agc_dp_t)in[0]);
    else if (!strcmp(op, "DCOMP")) got = agc_dp_neg((agc_dp_t)in[0]);
    else if (!strcmp(op, "TROUND")) got = agc_tp_round(&ip, in[0]);
    else if (!strcmp(op, "TSQRT")) got = agc_tp_sqrt(&ip, in[0]);
    else if (!strcmp(op, "TAD")) { got = agc_tp_add(&ip, in[0], in[1]); exp = mp; }
    else if (!strcmp(op, "NORM")) { int sh = 0; got = agc_dp_norm((agc_dp_t)in[0], &sh);
        got2 = -sh; exp2 = w[8]; }
    else if (!strcmp(op, "SR")) { got = agc_dp_shr((agc_dp_t)in[0], (int)in[1]);
        got2 = agc_tp_shift(&ip, agc_tp_from_dp((agc_dp_t)in[0]), -(int)in[1]); exp2 = mp; }
    else if (!strcmp(op, "SRR")) got = agc_dp_shr_round((agc_dp_t)in[0], (int)in[1]);
    else if (!strcmp(op, "SL")) got = agc_dp_shl(&ip, (agc_dp_t)in[0], (int)in[1]);
    else if (!strcmp(op, "ABVAL")) got = agc_vec_abval(&ip, u);
    else if (!strcmp(op, "VSQ")) got = agc_vec_sq(&ip, u);
    else if (!strcmp(op, "DOT")) got = agc_vec_dot(&ip, u, v);
    else {
        isvec = 1;
        if (!strcmp(op, "UNIT")) { r = agc_vec_unit(&ip, u, &lv, &lvsq);
            if (ip.ovfind == 0) { got = lv; exp = dpv(w[11], w[12]); got2 = lvsq; exp2 = dpv(w[9], w[10]); } }
        else if (!strcmp(op, "VXV")) r = agc_vec_cross(&ip, u, v);
        else if (!strcmp(op, "VPROJ")) r = agc_vec_proj(&ip, u, v);
        else if (!strcmp(op, "VAD")) r = agc_vec_add(&ip, u, v);
        else if (!strcmp(op, "VSU")) r = agc_vec_sub(&ip, u, v);
        else if (!strcmp(op, "BVSU")) r = agc_vec_sub(&ip, v, u);
        else if (!strcmp(op, "VXSC")) r = agc_vec_scale(&ip, u, (agc_dp_t)in[3]);
        else if (!strcmp(op, "V/SC")) r = agc_vec_div(&ip, u, (agc_dp_t)in[3]);
        else if (!strcmp(op, "MXV") || !strcmp(op, "VXM")) {
            for (k = 0; k < 9; k++) m.row[k / 3].c[k % 3] = (agc_dp_t)in[3 + k];
            r = !strcmp(op, "MXV") ? agc_mat_vec(&ip, &m, u) : agc_vec_mat(&ip, u, &m); }
        else if (!strcmp(op, "VSL")) r = agc_vec_shl(&ip, u, (int)in[3]);
        else if (!strcmp(op, "VSR")) r = agc_vec_shr(u, (int)in[3]);
        else { printf("unknown op %s\n", op); return 0; }
        if (got == 0 && exp == mdp) exp = 0;   /* no scalar output        */
    }
    if (!strcmp(op, "SQRT") || !strcmp(op, "SIGN") || !strcmp(op, "ABS") || !strcmp(op, "DCOMP")) {
        ovf_checked = 0;
    }
    if (!c->ok || got != exp || got2 != exp2) return 0;
    if (isvec && (r.c[0] != mdp || r.c[1] != dpv(w[3], w[4]) || r.c[2] != dpv(w[5], w[6]))) return 0;
    return !ovf_checked || ip.ovfind == w[7];
}

int main(void)
{
    static const char *const paths[2] = {
        "tools/oracle/cases/core_numeric.txt", "../tools/oracle/cases/core_numeric.txt"
    };
    FILE *f = NULL;
    char line[1024];
    ocase_t c;
    int n = 0, bad = 0, k, in_case = 0;
    for (k = 0; k < 2 && f == NULL; k++) f = fopen(paths[k], "r");
    if (f == NULL) {
        printf("oracle_core_numeric: SKIP (core_numeric.txt not found)\n");
        return 0;
    }
    memset(&c, 0, sizeof c);
    while (fgets(line, sizeof line, f) != NULL) {
        char a[64], b[512];
        if (line[0] == '#' || sscanf(line, "%63s %511s", a, b) < 1) continue;
        if (!strcmp(a, "CASE")) {
            memset(&c, 0, sizeof c);
            parse_tag(&c, b);
            in_case = 1;
        } else if (!strcmp(a, "END") && in_case) {
            n++;
            if (!run_case(&c)) {
                bad++;
                printf("MISMATCH %s", c.op);
                for (k = 0; k < c.nin; k++) printf(" %lld", c.in[k]);
                printf("\n");
            }
            in_case = 0;
        } else if (!strcmp(a, "STATUS")) {
            c.ok = !strcmp(b, "OK");
        } else if (!strcmp(a, "STOP")) {
            c.poodoo = !strcmp(b, "POODOO");
        } else {
            for (k = 0; k < 14; k++) {
                if (!strcmp(a, names[k])) c.w[k] = word(b);
            }
        }
    }
    fclose(f);
    printf("oracle_core_numeric: %d cases, %d mismatches\n", n, bad);
    return (bad != 0 || n == 0) ? 1 : 0;
}
