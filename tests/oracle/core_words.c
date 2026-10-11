/**********************************************************************
 * FILE NAME: core_words.c
 *
 * PURPOSE:
 *   Differential test of the word-level MPAC API (interp_words.h)
 *   against yaAGC running the real Comanche055 interpreter: MPAC,
 *   MPAC+1, MPAC+2, OVFIND, DVSIGN and stored words must match to the
 *   bit, for operands whose words disagree in sign.
 *
 * AGC SOURCE:
 *   Comanche055 INTERPRETER.agc pp.1107-1191 (single instructions);
 *   lane sequences from CONIC_SUBROUTINES (PARAM, WLOOP, POLYCOEF,
 *   GETECC), ANGLFIND (DELCOMP), Luminary099 ASCENT_GUIDANCE (ATMAG,
 *   MAINENG, ASCENT), REENTRY (HUNTEST1), LATITUDE_LONGITUDE (SR1 ASIN).
 *
 * EXTERNAL VARIABLES:
 *   none
 *
 * EXTERNAL REFERENCES:
 *   Name        Description
 *   ----------  --------------------------------------------------
 *   agc_mpac_*  the API under test (interp_words.h)
 *   oracle_*    case reader (oracle_txt.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; returns non-zero on any mismatch, prints SKIP and returns 0
 *   when the case files are not found.
 *
 * NOTES:
 *   Reads tools/oracle/cases/core_words.{spec,txt}, made by
 *   tools/oracle/cases/gen/gen_core_words.py. A CASE tag is an
 *   instruction or sequence name and a serial number; the inputs are
 *   the spec's set lines (MPAC, VAC1+n), the shift count is the low
 *   seven bits of the snippet's address word, less one.
 **********************************************************************/
#include "oracle_txt.h"

#include "agc/core/interp.h"
#include "agc/core/interp_words.h"
#include "agc/core/word.h"

#define TAG_MAX 64 /* longest case tag */
#define OP_MAX  16 /* longest op name  */

static int            run_case(const char *tag);
static bool           run_single(agc_interp_t *ip, const char *op, const char *tag, agc_tp_words_t *m);
static bool           run_unary(agc_interp_t *ip, const char *op, agc_tp_words_t *m);
static bool           run_shift(agc_interp_t *ip, const char *op, const char *tag, agc_tp_words_t *m);
static bool           run_conic(agc_interp_t *ip, const char *op, const char *tag, agc_tp_words_t *m);
static bool           run_lane(agc_interp_t *ip, const char *op, const char *tag, agc_tp_words_t *m);
static bool           run_ascent(agc_interp_t *ip, const char *op, const char *tag, agc_tp_words_t *m);
static agc_dp_words_t in_dp(const char *tag, const char *name);
static void           check(const char *tag, const char *what, long int got, long int exp);

static int g_bad; /* mismatching checks */

/**********************************************************************
 * AGC: none (test driver) -- run every CASE of core_words.txt.
 *
 * RETURNS: int -- 0 when every check matched (or SKIP), 1 otherwise
 **********************************************************************/
int main(void)
{
    const char *p = oracle_txt; /* scan position in the dump */
    char        tag[TAG_MAX];   /* current case tag */
    int         n = 0;          /* cases run */

    if (!oracle_load("core_words"))
    {
        printf("oracle_core_words: SKIP (core_words.spec/.txt not found)\n");

        return (0);
    }

    for (p = strstr(p, "CASE "); p != NULL; p = strstr(p + 5, "CASE "))
    {
        if ((p == oracle_txt) || (p[-1] == '\n'))
        {
            (void) sscanf(p + 5, "%63s", tag);
            n += run_case(tag);
        }
    }

    printf("oracle_core_words: %d cases, %d checks, %d failures, %d missing\n", n, oracle_checks, g_bad, oracle_missing);

    return (((g_bad != 0) || (oracle_missing != 0) || (n == 0)) ? 1 : 0); /* fail when none ran */
}

/**********************************************************************
 * AGC: none (test) -- one case: rebuild the instruction or sequence from
 * the tag, run it on the words of the spec, compare with the dump.
 *
 * ARGUMENTS:
 *   tag  const char *  I  CASE tag, e.g. "DDV12"
 *
 * RETURNS: int -- 1 (one case run)
 **********************************************************************/
static int run_case(const char *tag)
{
    agc_interp_t   ip;         /* interpreter state */
    agc_tp_words_t m;          /* MPAC */
    char           op[OP_MAX]; /* tag without its serial number */
    size_t         k;          /* length of the op name */
    bool           known;      /* the op is one this test runs */

    (void) snprintf(op, sizeof (op), "%.15s", tag);

    for (k = strlen(op); (k > 0) && (op[k - 1] >= '0') && (op[k - 1] <= '9'); k--)
    {
        op[k - 1] = '\0'; /* drop the serial number */
    }

    agc_interp_init(&ip);
    m.hi  = (agc_sp_t) oracle_in(tag, "MPAC", 0);
    m.mid = (agc_sp_t) oracle_in(tag, "MPAC", 1);
    m.lo  = (agc_sp_t) oracle_in(tag, "MPAC", 2);
    known = run_single(&ip, op, tag, &m) || run_unary(&ip, op, &m) || run_shift(&ip, op, tag, &m) || run_conic(&ip, op, tag, &m) || run_lane(&ip, op, tag, &m) || run_ascent(&ip, op, tag, &m);

    if (!known)
    {
        check(tag, "unknown op", 0, 1);

        return (1);
    }

    check(tag, "MPAC", m.hi, oracle_out_sp(tag, "MPAC"));
    check(tag, "MPAC+1", m.mid, oracle_out_sp(tag, "MPAC+1"));
    check(tag, "MPAC+2", m.lo, oracle_out_sp(tag, "MPAC+2"));
    check(tag, "OVFIND", ip.ovfind, oracle_out_sp(tag, "OVFIND"));

    if ((strcmp(op, "DDV") == 0) || (strcmp(op, "BDDV") == 0))
    { /* DVSIGN -0 = negative quotient (stale-DVSIGN model) */
        check(tag, "DVSIGN", ip.dvsign_neg, oracle_out_raw(tag, "DVSIGN") == 077777u);
    }

    return (1);
}

/**********************************************************************
 * AGC: none (test) -- the instructions with a DP or TP operand at 10D.
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *    IO interpreter state
 *   op   const char *      I  instruction name
 *   tag  const char *      I  case tag (inputs)
 *   m    agc_tp_words_t *  IO MPAC
 *
 * RETURNS: bool -- true when op is one of these
 **********************************************************************/
static bool run_single(agc_interp_t *ip, const char *op, const char *tag, agc_tp_words_t *m)
{
    agc_dp_words_t x = in_dp(tag, "VAC1+10"); /* DP operand */
    agc_tp_words_t t;                         /* TP operand */

    if (strcmp(op, "DAD") == 0)
    {
        *m = agc_mpac_dad(ip, *m, x);
    }
    else if (strcmp(op, "DSU") == 0)
    {
        *m = agc_mpac_dsu(ip, *m, x);
    }
    else if (strcmp(op, "BDSU") == 0)
    {
        *m = agc_mpac_bdsu(ip, *m, x);
    }
    else if (strcmp(op, "DMP") == 0)
    {
        *m = agc_mpac_dmp(*m, x);
    }
    else if (strcmp(op, "DMPR") == 0)
    {
        *m = agc_mpac_dmpr(ip, *m, x);
    }
    else if (strcmp(op, "SIGN") == 0)
    {
        *m = agc_mpac_sign(*m, x);
    }
    else if (strcmp(op, "DDV") == 0)
    {
        *m = agc_mpac_ddv(ip, *m, x);
    }
    else if (strcmp(op, "BDDV") == 0)
    {
        *m = agc_mpac_bddv(ip, *m, x);
    }
    else if (strcmp(op, "TAD") == 0)
    {
        t.hi  = x.hi;
        t.mid = x.lo;
        t.lo  = (agc_sp_t) oracle_in(tag, "VAC1+10", 2);
        *m    = agc_mpac_tad(ip, *m, t);
    }
    else
    {
        return (false);
    }

    return (true);
} /* run_single */

/**********************************************************************
 * AGC: none (test) -- the unary instructions.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *    IO interpreter state
 *   op  const char *      I  instruction name
 *   m   agc_tp_words_t *  IO MPAC
 *
 * RETURNS: bool -- true when op is one of these
 **********************************************************************/
static bool run_unary(agc_interp_t *ip, const char *op, agc_tp_words_t *m)
{
    if (strcmp(op, "DSQ") == 0)
    {
        *m = agc_mpac_dsq(*m);
    }
    else if (strcmp(op, "ROUND") == 0)
    {
        *m = agc_mpac_round(ip, *m);
    }
    else if (strcmp(op, "DCOMP") == 0)
    {
        *m = agc_mpac_dcomp(*m);
    }
    else if (strcmp(op, "ABS") == 0)
    {
        *m = agc_mpac_abs(*m);
    }
    else if (strcmp(op, "SIN") == 0)
    {
        *m = agc_mpac_sin(*m);
    }
    else if (strcmp(op, "COS") == 0)
    {
        *m = agc_mpac_cos(*m);
    }
    else if (strcmp(op, "SQRT") == 0)
    {
        *m = agc_mpac_sqrt(*m);
    }
    else if (strcmp(op, "ASIN") == 0)
    {
        *m = agc_mpac_asin(ip, *m);
    }
    else if (strcmp(op, "ACOS") == 0)
    {
        *m = agc_mpac_acos(ip, *m);
    }
    else
    {
        return (false);
    }

    (void) agc_interp_take_alarm(); /* FAILREG is not compared */

    return (true);
} /* run_unary */

/**********************************************************************
 * AGC: none (test) -- SR, SRR, SL, SLR with the count of the snippet's
 * address word, and NORM with its stored count.
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *    IO interpreter state
 *   op   const char *      I  instruction name
 *   tag  const char *      I  case tag
 *   m    agc_tp_words_t *  IO MPAC
 *
 * RETURNS: bool -- true when op is one of these
 **********************************************************************/
static bool run_shift(agc_interp_t *ip, const char *op, const char *tag, agc_tp_words_t *m)
{
    int n     = (int) (oracle_in_raw(tag, "VAC2USE", 1) & 0177u) - 1; /* shift count */
    int count = 0;                                                    /* NORM shifts */

    if (strcmp(op, "SR") == 0)
    {
        *m = agc_mpac_shift(ip, *m, -n);
    }
    else if (strcmp(op, "SRR") == 0)
    {
        *m = agc_mpac_shift_round(ip, *m, -n);
    }
    else if (strcmp(op, "SL") == 0)
    {
        *m = agc_mpac_shift(ip, *m, n);
    }
    else if (strcmp(op, "SLR") == 0)
    {
        *m = agc_mpac_shift_round(ip, *m, n);
    }
    else if (strcmp(op, "NORM") == 0)
    {
        *m = agc_mpac_norm(*m, &count);
        check(tag, "NORM count", -count, oracle_out_sp(tag, "VAC1+30"));
    }
    else
    {
        return (false);
    }

    return (true);
} /* run_shift */

/**********************************************************************
 * AGC: none (test) -- the conic sequences: PARAM's R1A (DLOAD BDSU
 * D1/32, STODL), then a WLOOP pass, POLYCOEF's DSQ NORM DMP R1A, or
 * GETECC's DMP SL4 R1A BDSU SQRT D1/64.
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *    IO interpreter state
 *   op   const char *      I  sequence name
 *   tag  const char *      I  case tag
 *   m    agc_tp_words_t *  IO MPAC
 *
 * RETURNS: bool -- true when op is one of these
 **********************************************************************/
static bool run_conic(agc_interp_t *ip, const char *op, const char *tag, agc_tp_words_t *m)
{
    agc_dp_words_t r1a;       /* R1A as PARAM stores it */
    agc_tp_words_t wsq;       /* WSQ pushed by PDDL */
    int            count = 0; /* NORM shifts */

    if ((strcmp(op, "WLOOP") != 0) && (strcmp(op, "POLYCOEF") != 0) && (strcmp(op, "GETECC") != 0))
    {
        return (false);
    }

    *m  = agc_mpac_bdsu(ip, agc_mpac_dload(in_dp(tag, "VAC1+20")), in_dp(tag, "VAC1+22"));
    r1a = agc_mpac_store(*m);
    check(tag, "R1A", agc_dp_from_words(r1a), oracle_out_dp(tag, "VAC1+24"));
    check(tag, "R1A hi", r1a.hi, oracle_out_sp(tag, "VAC1+24"));

    if (strcmp(op, "WLOOP") == 0)
    { /* PUSH DSQ; TLOAD PDDL MPAC R1A; SR4 TAD; SQRT ROUND; DAD */
        wsq = agc_mpac_dsq(agc_mpac_dload(in_dp(tag, "VAC1+10")));
        *m  = agc_mpac_shift(ip, agc_mpac_dload(r1a), -4);
        *m  = agc_mpac_round(ip, agc_mpac_sqrt(agc_mpac_tad(ip, *m, wsq)));
        *m  = agc_mpac_dad(ip, *m, in_dp(tag, "VAC1+10"));
    }
    else if (strcmp(op, "POLYCOEF") == 0)
    { /* DLOAD W; DSQ NORM; DMP R1A */
        *m = agc_mpac_norm(agc_mpac_dsq(agc_mpac_dload(in_dp(tag, "VAC1+10"))), &count);
        *m = agc_mpac_dmp(*m, r1a);
        check(tag, "NORM count", -count, oracle_out_sp(tag, "VAC1+30"));
    }
    else
    { /* DLOAD P; DMP SL4 R1A; BDSU SQRT D1/64 */
        *m = agc_mpac_shift(ip, agc_mpac_dmp(agc_mpac_dload(in_dp(tag, "VAC1+26")), r1a), 4);
        *m = agc_mpac_sqrt(agc_mpac_bdsu(ip, *m, in_dp(tag, "VAC1+28")));
    }

    return (true);
} /* run_conic */

/**********************************************************************
 * AGC: none (test) -- DELCOMP's 1 - cos A (SIN, COS, SR2, BDSU DPHALF;
 * COF DSQ DMP DAD SL3), HUNTEST1's DMP DDV DDV DDV, SR1 then ASIN.
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *    IO interpreter state
 *   op   const char *      I  sequence name
 *   tag  const char *      I  case tag
 *   m    agc_tp_words_t *  IO MPAC
 *
 * RETURNS: bool -- true when op is one of these
 **********************************************************************/
static bool run_lane(agc_interp_t *ip, const char *op, const char *tag, agc_tp_words_t *m)
{
    agc_dp_words_t pd2; /* push-down 2D */
    agc_dp_words_t pd4; /* push-down 4D */
    agc_dp_words_t v1;  /* V1 */

    if (strcmp(op, "DELCOMP") == 0)
    {
        (void) agc_mpac_sin(*m);                                /* SIN PDDL: PD0 = SIN(A) */
        *m  = agc_mpac_cos(agc_mpac_dload(agc_mpac_store(*m))); /* MPAC = A; COS          */
        pd2 = agc_mpac_store(*m);                                   /* PUSH: PD2 = COS(A)     */
        *m  = agc_mpac_shift(ip, *m, -2);                           /* SR2                    */
        v1  = agc_mpac_store(*m);                                   /* PDDL: PD2 = COS/4 ...  */
        *m  = agc_mpac_bdsu(ip, agc_mpac_dload(pd2), in_dp(tag, "VAC1+22"));
        pd2 = v1;                                                   /* ... MPAC = COS(A)      */
        pd4 = agc_mpac_store(*m);                                   /* PDDL: PD4 = 1-COS(A)   */
        *m  = agc_mpac_dmp(agc_mpac_dsq(agc_mpac_dload(in_dp(tag, "VAC1+20"))), pd4);
        *m  = agc_mpac_shift(ip, agc_mpac_dad(ip, *m, pd2), 3);     /* DAD 2; SL3             */
    }
    else if (strcmp(op, "HUNTEST") == 0)
    {
        v1 = in_dp(tag, "VAC1+24");
        *m = agc_mpac_dmp(agc_mpac_dload(in_dp(tag, "VAC1+20")), in_dp(tag, "VAC1+22"));
        *m = agc_mpac_ddv(ip, agc_mpac_ddv(ip, *m, v1), v1);
        *m = agc_mpac_ddv(ip, *m, in_dp(tag, "VAC1+26"));
    }
    else if (strcmp(op, "SR1ASIN") == 0)
    {
        *m = agc_mpac_shift(ip, agc_mpac_dload(in_dp(tag, "VAC1+20")), -1);
        *m = agc_mpac_asin(ip, *m);
    }
    else
    {
        return (false);
    }

    return (true);
} /* run_lane */

/**********************************************************************
 * AGC: none (test) -- the ascent sequences: ATMAG's TBUP and AT,
 * MAINENG's TGO and TTOGO, ASCENT's 1 - TGO/TBUP.
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *    IO interpreter state
 *   op   const char *      I  sequence name
 *   tag  const char *      I  case tag
 *   m    agc_tp_words_t *  IO MPAC
 *
 * RETURNS: bool -- true when op is one of these
 **********************************************************************/
static bool run_ascent(agc_interp_t *ip, const char *op, const char *tag, agc_tp_words_t *m)
{
    agc_dp_words_t pd0; /* push-down 0D */
    agc_dp_words_t st;  /* stored TBUP / TGO */

    if (strcmp(op, "ATMAG") == 0)
    { /* DMP DMP VE 2SEC(9); SL3 PDDL TBUP; SR1 DAD; DSU 6SEC(18); STODL TBUP VE; SR1 DDV TBUP */
        *m  = agc_mpac_dmp(agc_mpac_dmp(*m, in_dp(tag, "VAC1+22")), in_dp(tag, "VAC1+24"));
        pd0 = agc_mpac_store(agc_mpac_shift(ip, *m, 3));
        *m  = agc_mpac_dad(ip, agc_mpac_shift(ip, agc_mpac_dload(in_dp(tag, "VAC1+26")), -1), pd0);
        st  = agc_mpac_store(agc_mpac_dsu(ip, *m, in_dp(tag, "VAC1+28")));
        *m  = agc_mpac_ddv(ip, agc_mpac_shift(ip, agc_mpac_dload(in_dp(tag, "VAC1+22")), -1), st);
        check(tag, "TBUP", agc_dp_from_words(st), oracle_out_dp(tag, "VAC1+26"));
        check(tag, "TBUP hi", st.hi, oracle_out_sp(tag, "VAC1+26"));
    }
    else if (strcmp(op, "MAINENG") == 0)
    { /* DMP DSU TBUP TTO; STORE TGO; SR DCOMP 11D */
        *m = agc_mpac_dsu(ip, agc_mpac_dmp(*m, in_dp(tag, "VAC1+20")), in_dp(tag, "VAC1+22"));
        st = agc_mpac_store(*m);
        *m = agc_mpac_dcomp(agc_mpac_shift(ip, *m, -11));
        check(tag, "TGO hi", st.hi, oracle_out_sp(tag, "VAC1+24"));
        check(tag, "TGO lo", st.lo, oracle_out_sp(tag, "VAC1+25"));
    }
    else if (strcmp(op, "ASCENT") == 0)
    { /* DLOAD DSU TBUP TGO; DDV TBUP */
        *m = agc_mpac_dsu(ip, agc_mpac_dload(in_dp(tag, "VAC1+20")), in_dp(tag, "VAC1+22"));
        *m = agc_mpac_ddv(ip, *m, in_dp(tag, "VAC1+20"));
    }
    else
    {
        return (false);
    }

    return (true);
} /* run_ascent */

/**********************************************************************
 * AGC: none (test) -- a DP operand's two words from the spec.
 *
 * ARGUMENTS:
 *   tag   const char *  I  case tag
 *   name  const char *  I  erasable name in the set line
 *
 * RETURNS: agc_dp_words_t -- the words as poked (0 if not set)
 **********************************************************************/
static agc_dp_words_t in_dp(const char *tag, const char *name)
{
    agc_dp_words_t x; /* the two words */

    x.hi = (agc_sp_t) oracle_in(tag, name, 0);
    x.lo = (agc_sp_t) oracle_in(tag, name, 1);

    return (x);
}

/**********************************************************************
 * AGC: none (test) -- count one check; report a mismatch.
 *
 * ARGUMENTS:
 *   tag   const char *  I  case tag
 *   what  const char *  I  quantity compared
 *   got   long int      I  C value
 *   exp   long int      I  yaAGC value
 **********************************************************************/
static void check(const char *tag, const char *what, long int got, long int exp)
{
    oracle_checks++;

    if (got != exp)
    {
        g_bad++;
        printf("MISMATCH %s %s: C %ld, AGC %ld\n", tag, what, got, exp);
    }
}
