/**********************************************************************
 * FILE NAME: core_numeric.c
 *
 * PURPOSE:
 *   Differential test of the numeric core against yaAGC running the
 *   real Comanche055 interpreter: every value must match to the bit.
 *
 * AGC SOURCE:
 *   Comanche055 INTERPRETER.agc pp.1131-1191 (the instructions in the
 *   case tags), entered through INTPRET with a snippet in VAC2USE, or
 *   directly for the basic routines SHORTMP, DMPNSUB and POWRSERS.
 *
 * EXTERNAL VARIABLES:
 *   none
 *
 * EXTERNAL REFERENCES:
 *   Name     Description
 *   -------  ----------------------------------------------------
 *   agc_*    the numeric core under test (interp.h, word.h)
 *   stdio    fopen / fgets / sscanf of the case file
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; returns non-zero on any mismatch, prints SKIP and returns 0
 *   when the case file is not found (run from the repository root).
 *
 * NOTES:
 *   Reads tools/oracle/cases/core_numeric.txt (made from
 *   core_numeric.spec, whose header gives the layout). Each CASE tag is
 *   "OP,input,input,..."; a tag "OP1+OP2[+OP3],..." runs the leading
 *   instructions first on the same interpreter state (DVSIGN carries
 *   over). Expected MPAC words, OVFIND, LV/LVSQUARE, NORM count and
 *   FAILREG come from the emulator.
 **********************************************************************/
#include <stdio.h>
#include <stdlib.h>

#include <string.h>

#include "agc/core/interp.h"
#include "agc/core/word.h"

#define MAX_IN    16 /* inputs in a case tag                     */
#define N_OUTPUTS 14 /* emulator words read per case (names[])   */

typedef struct
{
    char          op[16];     /* OP of the tag (with any OP+ prefix)   */
    int           nin;        /* inputs in the tag                     */
    long long int in[MAX_IN]; /* inputs, DP 2^-28 / TP 2^-42 units     */
    int           ok;         /* STATUS OK                             */
    int           poodoo;     /* stopped at POODOO                     */
    long long int w[16];      /* names[] words, signed                 */
} ocase_t;

typedef struct
{
    long long int got;         /* scalar result, DP or TP units         */
    long long int exp;         /* expected scalar                       */
    long long int got2;        /* second result (count, flag, DP form)  */
    long long int exp2;        /* expected second result                */
    int           isvec;       /* the op leaves a vector in MPAC        */
    int           ovf_checked; /* OVFIND is compared                    */
    agc_vec_t     r;           /* vector result, 2^-28 units            */
} result_t;

static const char *const names[N_OUTPUTS] = { "MPAC", "MPAC+1", "MPAC+2", "MPAC+3", "MPAC+4", "MPAC+5", "MPAC+6", "OVFIND", "VAC1+30", "VAC1+34", "VAC1+35", "VAC1+36", "VAC1+37", "FAILREG" };

static long long int word_value(const char *oct);
static void          parse_tag(ocase_t *c, const char *tag);
static void          read_line(ocase_t *c, const char *a, const char *b);
static long long int dpv(long long int hi, long long int lo);
static agc_vec_t     vec_at(const long long int *in);
static int           run_prefix(agc_interp_t *ip, const ocase_t *c, char *last, size_t n);
static int           scalar_op(agc_interp_t *ip, const char *op, const long long int *in, const ocase_t *c, result_t *res);
static int           shift_op(agc_interp_t *ip, const char *op, const long long int *in, const ocase_t *c, result_t *res);
static int           vector_op(agc_interp_t *ip, const char *op, const long long int *in, const ocase_t *c, result_t *res);
static int           run_case(const ocase_t *c);

/**********************************************************************
 * AGC: none (test driver) -- read the case file and run every case.
 *
 * RETURNS: int -- 0 when every case matched (or SKIP), 1 otherwise
 **********************************************************************/
int main(void)
{
    static const char *const paths[2] = {
        "tools/oracle/cases/core_numeric.txt",
        "../tools/oracle/cases/core_numeric.txt"
    };
    FILE   *f                         = NULL; /* the case file              */
    char    line[1024];                       /* one line of it             */
    char    a[64];                            /* first token                */
    char    b[512];                           /* second token               */
    ocase_t c;                                /* case being read            */
    int     n                         = 0;    /* cases run                  */
    int     bad                       = 0;    /* cases that mismatched      */
    int     in_case                   = 0;    /* inside CASE ... END        */
    int     k;                                /* path / input index         */

    for (k = 0; (k < 2) && (f == NULL); k++)
    {
        f = fopen(paths[k], "r");
    }

    if (f == NULL)
    {
        printf("oracle_core_numeric: SKIP (core_numeric.txt not found)\n");

        return (0);
    }

    memset(&c, 0, sizeof (c));

    while (fgets(line, sizeof (line), f) != NULL)
    {
        if ((line[0] == '#') || (sscanf(line, "%63s %511s", a, b) < 1))
        {
            continue;
        }

        if (strcmp(a, "CASE") == 0)
        {
            memset(&c, 0, sizeof (c));
            parse_tag(&c, b);
            in_case = 1;
        }
        else if ((strcmp(a, "END") == 0) && (in_case != 0))
        {
            n++;
            in_case = 0;

            if (run_case(&c) == 0)
            {
                bad++;
                printf("MISMATCH %s", c.op);

                for (k = 0; k < c.nin; k++)
                {
                    printf(" %lld", c.in[k]);
                }

                printf("\n");
            }
        }
        else
        {
            read_line(&c, a, b);
        }
    } /* while (fgets(...)) */

    (void) fclose(f);
    printf("oracle_core_numeric: %d cases, %d mismatches\n", n, bad);

    return (((bad != 0) || (n == 0)) ? 1 : 0); /* fail when none ran */
} /* main */

/**********************************************************************
 * AGC: none (test helper) -- one "NAME value" line of a case: STATUS,
 * STOP, or one of the emulator words in names[].
 *
 * ARGUMENTS:
 *   c  ocase_t *     IO the case being read
 *   a  const char *  I  first token
 *   b  const char *  I  second token
 **********************************************************************/
static void read_line(ocase_t *c, const char *a, const char *b)
{
    int k; /* names[] index */

    if (strcmp(a, "STATUS") == 0)
    {
        c->ok = (strcmp(b, "OK") == 0);

        return;
    }

    if (strcmp(a, "STOP") == 0)
    {
        c->poodoo = (strcmp(b, "POODOO") == 0);

        return;
    }

    for (k = 0; k < N_OUTPUTS; k++)
    {
        if (strcmp(a, names[k]) == 0)
        {
            c->w[k] = word_value(b);
        }
    }
}

/**********************************************************************
 * AGC: none (test helper) -- a 15-bit ones' complement octal word.
 *
 * ARGUMENTS:
 *   oct  const char *  I  octal text
 *
 * RETURNS: long long int -- the signed value (-0 reads as 0)
 **********************************************************************/
static long long int word_value(const char *oct)
{
    long int v = strtol(oct, NULL, 8); /* raw 15-bit word */

    return ((v < 040000) ? v : -(077777 - v)); /* complement if negative */
}

/**********************************************************************
 * AGC: none (test helper) -- split "OP,in,in,..." into the case.
 *
 * ARGUMENTS:
 *   c    ocase_t *     O  op and inputs
 *   tag  const char *  I  the CASE tag
 **********************************************************************/
static void parse_tag(ocase_t *c, const char *tag)
{
    char  buf[512]; /* writable copy for strtok */
    char *tok;      /* current token            */

    strncpy(buf, tag, sizeof (buf) - 1);
    buf[sizeof (buf) - 1] = '\0';
    tok                   = strtok(buf, ",");
    strncpy(c->op, (tok != NULL) ? tok : "", sizeof (c->op) - 1); /* OP */
    c->nin                = 0;
    tok                   = strtok(NULL, ",");

    while ((tok != NULL) && (c->nin < MAX_IN))
    {
        c->in[c->nin] = strtoll(tok, NULL, 10);
        c->nin++;
        tok           = strtok(NULL, ",");
    }
}

/**********************************************************************
 * AGC: none (test helper) -- value of a word pair.
 *
 * ARGUMENTS:
 *   hi  long long int  I  major word (or major value)
 *   lo  long long int  I  minor word
 *
 * RETURNS: long long int -- hi 2^14 + lo
 **********************************************************************/
static long long int dpv(long long int hi, long long int lo)
{
    return (hi * 16384 + lo);
}

/**********************************************************************
 * AGC: none (test helper) -- three tag inputs as a vector.
 *
 * ARGUMENTS:
 *   in  const long long int *  I  x, y, z in 2^-28 units
 *
 * RETURNS: agc_vec_t -- (x, y, z)
 **********************************************************************/
static agc_vec_t vec_at(const long long int *in)
{
    return (agc_vec_make((agc_dp_t) in[0], (agc_dp_t) in[1], (agc_dp_t) in[2]));
}

/**********************************************************************
 * AGC: none (test helper) -- multi-instruction tags ("DDV+UNIT",
 * "V/SC+UNIT", "DDV+DOT+UNIT", ...): run every instruction before the
 * last on the same interpreter state, so DVSIGN carries over as in the
 * AGC, consuming its inputs (DDV a,b; V/SC x,y,z,s; UNIT x,y,z; DOT dots
 * the next vector with itself; ACOS takes the DDV quotient).
 *
 * ARGUMENTS:
 *   ip    agc_interp_t *   IO the case's interpreter state
 *   c     const ocase_t *  I  the case
 *   last  char *           O  the last op name
 *   n     size_t           I  size of last
 *
 * RETURNS: int -- the number of inputs used
 **********************************************************************/
static int run_prefix(agc_interp_t *ip, const ocase_t *c, char *last, size_t n)
{
    const char          *p    = c->op;          /* current op name     */
    const char          *plus = strchr(p, '+'); /* end of it           */
    const long long int *in   = c->in;          /* the inputs          */
    int                  used = 0;              /* inputs consumed     */
    agc_dp_t             q    = 0;              /* DDV quotient, 2^0   */
    size_t               len;                   /* op name length      */

    while ((plus != NULL) && (used < c->nin))
    {
        len = (size_t) (plus - p);

        if ((len == 3) && (strncmp(p, "DDV", len) == 0))
        {
            q    = agc_dp_div(ip, (agc_dp_t) in[used], (agc_dp_t) in[used + 1]);
            used += 2;
        }
        else if ((len == 3) && (strncmp(p, "DOT", len) == 0))
        {
            (void) agc_vec_dot(ip, vec_at(in + used), vec_at(in + used));
        }
        else if ((len == 4) && (strncmp(p, "ACOS", len) == 0))
        {
            q = agc_dp_acos(ip, q);
        }
        else if ((len == 4) && (strncmp(p, "V/SC", len) == 0))
        {
            (void) agc_vec_div(ip, vec_at(in + used), (agc_dp_t) in[used + 3]);
            used += 4;
        }
        else if ((len == 4) && (strncmp(p, "UNIT", len) == 0))
        {
            (void) agc_vec_unit(ip, vec_at(in + used), NULL, NULL);
            used += 3;
        }
        else
        {
            /* unknown prefix: ignored, the last op reports it */
        }

        p    = plus + 1;
        plus = strchr(p, '+');
    } /* while (plus != NULL ...) */

    (void) snprintf(last, n, "%s", p);

    return (used);
} /* run_prefix */

/**********************************************************************
 * AGC: none (test helper) -- the scalar arithmetic ops: DMP, DMPR, DDV,
 * BDDV, DAD, DSU, DSQ, SQRT, ASIN, ACOS, SIN, COS, SIGN, ABS, DCOMP,
 * TSQRT, TAD, SHORTMP, DMPNSUB, POLY.
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *         IO interpreter state
 *   op   const char *           I  op name
 *   in   const long long int *  I  inputs after any prefix
 *   c    const ocase_t *        I  the case (emulator words)
 *   res  result_t *             IO results to compare
 *
 * RETURNS: int -- 1 when op is one of these, else 0
 **********************************************************************/
static int scalar_op(agc_interp_t *ip, const char *op, const long long int *in, const ocase_t *c, result_t *res)
{
    const long long int *w   = c->w;                       /* outputs   */
    const long long int  mp  = dpv(dpv(w[0], w[1]), w[2]); /* TP MPAC   */
    const agc_dp_t       a   = (agc_dp_t) in[0];           /* 1st DP operand, 2^-28 units */
    const agc_dp_t       b   = (agc_dp_t) in[1];           /* 2nd DP operand, 2^-28 units */
    agc_dp_t             t   = 0;                          /* out arg, 2^-28 units */
    agc_dp_t             coef[MAX_IN];                     /* POLY A(i), 2^-28 units */
    agc_status_t         s;                                /* _chk      */
    int                  k;                                /* index     */
    int                  hit = 1;                          /* handled   */

    if (strcmp(op, "DMP") == 0)
    {
        res->got  = agc_dp_mul_tp(a, b);
        res->exp  = mp;
        res->got2 = agc_dp_mul_trunc(a, b);
        res->exp2 = dpv(w[0], w[1]);
    }
    else if (strcmp(op, "DMPR") == 0)
    {
        res->got = agc_dp_mul(ip, a, b);
    }
    else if (strcmp(op, "DDV") == 0)
    {
        res->got = agc_dp_div(ip, a, b);
    }
    else if (strcmp(op, "BDDV") == 0)
    {
        res->got = agc_dp_div(ip, b, a);
    }
    else if (strcmp(op, "DAD") == 0)
    {
        res->got = agc_dp_add(ip, a, b);
    }
    else if (strcmp(op, "DSU") == 0)
    {
        res->got = agc_dp_sub(ip, a, b);
    }
    else if (strcmp(op, "DSQ") == 0)
    {
        res->got  = agc_dp_sq_tp(a);
        res->exp  = mp;
        res->got2 = agc_dp_sq(ip, a);
        res->exp2 = dpv(w[0], w[1]);
    }
    else if (strcmp(op, "SQRT") == 0)
    {
        res->got2 = (agc_dp_sqrt_chk(a, &t) == AGC_STATUS_POODOO);
        res->exp2 = c->poodoo;
        res->got  = (c->poodoo != 0) ? 0 : t;               /* aborted */
        res->exp  = (c->poodoo != 0) ? 0 : dpv(w[0], w[1]); /* aborted */
    }
    else if ((strcmp(op, "ASIN") == 0) || (strcmp(op, "ACOS") == 0))
    {
        s         = (op[1] == 'S') ? agc_dp_asin_chk(ip, a, &t) : agc_dp_acos_chk(ip, a, &t); /* ASIN, ACOS */
        res->got  = t;
        res->got2 = (s == AGC_STATUS_ALARM);
        res->exp2 = (w[13] == 01301);
    }
    else if (strcmp(op, "SIN") == 0)
    {
        res->got = agc_dp_sin(ip, a);
    }
    else if (strcmp(op, "COS") == 0)
    {
        res->got = agc_dp_cos(ip, a);
    }
    else if (strcmp(op, "SIGN") == 0)
    {
        res->got = agc_dp_sign(a, b);
    }
    else if (strcmp(op, "ABS") == 0)
    {
        res->got = agc_dp_abs(a);
    }
    else if (strcmp(op, "DCOMP") == 0)
    {
        res->got = agc_dp_neg(a);
    }
    else if (strcmp(op, "TSQRT") == 0)
    {
        res->got = agc_tp_sqrt(ip, in[0]);
    }
    else if (strcmp(op, "TAD") == 0)
    {
        res->got = agc_tp_add(ip, in[0], in[1]);
        res->exp = mp;
    }
    else if (strcmp(op, "SHORTMP") == 0)
    {
        res->got = agc_tp_mul_sp(in[0], (agc_sp_t) in[1]);
        res->exp = mp;
    }
    else if (strcmp(op, "DMPNSUB") == 0)
    {
        res->got = agc_dp_mul_int(a, (agc_sp_t) in[1]);
    }
    else if (strcmp(op, "POLY") == 0)
    {
        for (k = 0; (k <= (int) in[1]) && (k + 2 < c->nin); k++)
        {
            coef[k] = (agc_dp_t) in[2 + k];
        }

        res->got = agc_dp_poly(a, coef, (int) in[1]);
    }
    else
    {
        hit = 0;
    }

    return (hit);
} /* scalar_op */

/**********************************************************************
 * AGC: none (test helper) -- shifts, NORM and ROUND: SR, SRR, SL, SLR,
 * TSL, TSR (TLOAD then SL / SR), NORM, TNORM, TROUND.
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *         IO interpreter state
 *   op   const char *           I  op name
 *   in   const long long int *  I  inputs after any prefix
 *   c    const ocase_t *        I  the case (emulator words)
 *   res  result_t *             IO results to compare
 *
 * RETURNS: int -- 1 when op is one of these, else 0
 **********************************************************************/
static int shift_op(agc_interp_t *ip, const char *op, const long long int *in, const ocase_t *c, result_t *res)
{
    const long long int *w   = c->w;                       /* outputs   */
    const long long int  mp  = dpv(dpv(w[0], w[1]), w[2]); /* TP MPAC   */
    const agc_dp_t       a   = (agc_dp_t) in[0];           /* DP input, 2^-28 units */
    const int            n   = (int) in[1];                /* count     */
    int                  sh  = 0;                          /* NORM out  */
    int                  hit = 1;                          /* handled   */

    if (strcmp(op, "SR") == 0)
    {
        res->got  = agc_dp_shr(a, n);
        res->got2 = agc_tp_shift(ip, agc_tp_from_dp(a), -n);
        res->exp2 = mp;
    }
    else if (strcmp(op, "SRR") == 0)
    {
        res->got = agc_dp_shr_round(a, n);
    }
    else if (strcmp(op, "SL") == 0)
    {
        res->got = agc_dp_shl(ip, a, n);
    }
    else if (strcmp(op, "SLR") == 0)
    {
        res->got = agc_dp_shl_round(ip, a, n);
    }
    else if ((strcmp(op, "TSL") == 0) || (strcmp(op, "TSR") == 0))
    {
        res->got = agc_tp_shift(ip, in[0], (op[2] == 'L') ? n : -n); /* L/R */
        res->exp = mp;
    }
    else if (strcmp(op, "NORM") == 0)
    {
        res->got  = agc_dp_norm(a, &sh);
        res->got2 = -sh;
        res->exp2 = w[8];
    }
    else if (strcmp(op, "TNORM") == 0)
    {
        res->got  = agc_tp_norm(in[0], &sh);
        res->exp  = mp;
        res->got2 = -sh;
        res->exp2 = w[8];
    }
    else if (strcmp(op, "TROUND") == 0)
    {
        res->got = agc_tp_round(ip, in[0]);
    }
    else
    {
        hit = 0;
    }

    return (hit);
} /* shift_op */

/**********************************************************************
 * AGC: none (test helper) -- the ops that leave a vector in MPAC: UNIT
 * (with LV, LVSQUARE), VXV, VPROJ, VAD, VSU, BVSU, VXSC, V/SC, MXV, VXM,
 * VSL, VSR.
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *         IO interpreter state
 *   op   const char *           I  op name
 *   in   const long long int *  I  inputs after any prefix
 *   c    const ocase_t *        I  the case (emulator words)
 *   res  result_t *             IO results to compare
 *
 * RETURNS: int -- 1 when op is one of these, else 0
 **********************************************************************/
static int vector_op(agc_interp_t *ip, const char *op, const long long int *in, const ocase_t *c, result_t *res)
{
    const agc_vec_t u    = vec_at(in);       /* first vector, 2^-28 units */
    const agc_vec_t v    = vec_at(in + 3);   /* second vector (if any), 2^-28 units */
    const agc_dp_t  s    = (agc_dp_t) in[3]; /* scalar (2^-28 units) / shift count */
    agc_mat_t       m;                       /* MXV / VXM matrix          */
    agc_dp_t        lv   = 0;                /* UNIT LV, scale of u       */
    agc_dp_t        lvsq = 0;                /* UNIT LVSQUARE, 2x scale   */
    int             k;                       /* matrix element index      */

    res->isvec = 1;

    if (strcmp(op, "UNIT") == 0)
    {
        res->r = agc_vec_unit(ip, u, &lv, &lvsq);

        if (ip->ovfind == 0)
        {
            res->got  = lv;
            res->exp  = dpv(c->w[11], c->w[12]);
            res->got2 = lvsq;
            res->exp2 = dpv(c->w[9], c->w[10]);
        }
    }
    else if (strcmp(op, "VXV") == 0)
    {
        res->r = agc_vec_cross(ip, u, v);
    }
    else if (strcmp(op, "VPROJ") == 0)
    {
        res->r = agc_vec_proj(ip, u, v);
    }
    else if (strcmp(op, "VAD") == 0)
    {
        res->r = agc_vec_add(ip, u, v);
    }
    else if (strcmp(op, "VSU") == 0)
    {
        res->r = agc_vec_sub(ip, u, v);
    }
    else if (strcmp(op, "BVSU") == 0)
    {
        res->r = agc_vec_sub(ip, v, u);
    }
    else if (strcmp(op, "VXSC") == 0)
    {
        res->r = agc_vec_scale(ip, u, s);
    }
    else if (strcmp(op, "V/SC") == 0)
    {
        res->r = agc_vec_div(ip, u, s);
    }
    else if ((strcmp(op, "MXV") == 0) || (strcmp(op, "VXM") == 0))
    {
        for (k = 0; k < 9; k++)
        {
            m.row[k / 3].c[k % 3] = (agc_dp_t) in[3 + k];
        }

        res->r = (op[0] == 'M') ? agc_mat_vec(ip, &m, u) : agc_vec_mat(ip, u, &m); /* MXV, VXM   */
    }
    else if (strcmp(op, "VSL") == 0)
    {
        res->r = agc_vec_shl(ip, u, (int) s);
    }
    else if (strcmp(op, "VSR") == 0)
    {
        res->r = agc_vec_shr(u, (int) s);
    }
    else
    {
        res->isvec = 0;
    }

    return (res->isvec);
} /* vector_op */

/**********************************************************************
 * AGC: none (test helper) -- run one case through the C library.
 *
 * ARGUMENTS:
 *   c  const ocase_t *  I  the case
 *
 * RETURNS: int -- 1 when everything matches, else 0
 **********************************************************************/
static int run_case(const ocase_t *c)
{
    agc_interp_t         ip;                    /* interpreter state */
    const long long int *w   = c->w;            /* emulator words    */
    const long long int  mdp = dpv(w[0], w[1]); /* DP MPAC           */
    result_t             res;                   /* results           */
    char                 op[sizeof (c->op)];    /* last op of tag    */
    const long long int *in;                    /* inputs after it   */

    memset(&res, 0, sizeof (res));
    res.exp         = mdp;
    res.ovf_checked = 1;
    agc_interp_init(&ip);
    in              = c->in + run_prefix(&ip, c, op, sizeof (op));

    if (strcmp(op, "ABVAL") == 0)
    {
        res.got = agc_vec_abval(&ip, vec_at(in));
    }
    else if (strcmp(op, "VSQ") == 0)
    {
        res.got = agc_vec_sq(&ip, vec_at(in));
    }
    else if (strcmp(op, "DOT") == 0)
    {
        res.got = agc_vec_dot(&ip, vec_at(in), vec_at(in + 3));
    }
    else if ((scalar_op(&ip, op, in, c, &res) == 0) && (shift_op(&ip, op, in, c, &res) == 0) && (vector_op(&ip, op, in, c, &res) == 0))
    {
        printf("unknown op %s\n", op);

        return (0);
    }
    else
    {
        /* handled */
    }

    if ((res.isvec != 0) && (res.got == 0) && (res.exp == mdp))
    {
        res.exp = 0; /* no scalar output */
    }

    if ((strcmp(op, "SQRT") == 0) || (strcmp(op, "SIGN") == 0) || (strcmp(op, "ABS") == 0) || (strcmp(op, "DCOMP") == 0))
    {
        res.ovf_checked = 0;
    }

    if ((c->ok == 0) || (res.got != res.exp) || (res.got2 != res.exp2))
    {
        return (0);
    }

    if ((res.isvec != 0) && ((res.r.c[0] != mdp) || (res.r.c[1] != dpv(w[3], w[4])) || (res.r.c[2] != dpv(w[5], w[6]))))
    {
        return (0);
    }

    return ((res.ovf_checked == 0) || (ip.ovfind == w[7]));
} /* run_case */
