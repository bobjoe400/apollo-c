/*
 * landmark_navigation.c -- S22.1 ORBITAL NAVIGATION ROUTINE, CSM
 * (P20-P25.agc, Comanche055 pp.598-614): S22.1, S22F2410, S22CALRC,
 * S2231X13, GETTF, S22FLGS, 9DWTO6DW, ROWDOT.
 *
 * "1. Update CSM state vector. 2. Update landmark position. 3. Convert W
 * matrix from 9 to 6 dimensions." The marks of a P22 landmark sighting
 * (up to five, SVMRKDAT) are processed in order: the CSM is integrated to
 * each mark time with its W matrix; the first mark of an unknown landmark
 * sets the landmark position RL (formula 2.4.10) and the landmark rows of
 * a 9x9 W; every other mark is incorporated (twice, like R22) updating the
 * CSM and RL (X789). An offset point mark is kept for the end. Then the
 * landmark is shown (noun 89) and W converted back to 6x6.
 *
 * Scales: positions body scale (earth 2^29, moon 2^27; S22EORM = 0 earth,
 * -2 moon), UM half unit, U matrix B-3, RHO B-30, VARIANCE TP B-40.
 */
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/core/state.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"
#include "agc/programs/alignment_pending.h"

#define LN_SCTVAR     ((agc_dp_t) 70368744) /* SCTVAR 2DEC 1.0 E-6 B+18   */
#define LN_IMUVARR    ((agc_dp_t) 2814750)  /* IMUVARR 2DEC 0.04 E-6 B+18 */
#define LN_MARK_WORDS 7
#define LN_SVMRK      36 /* SVMRKDAT I(36)             */
#define LN_ROWS       9
#define LN_EMATRIX    21 /* EMATRIX B(42): 21 DP       */
#define LN_PRIO13     ((agc_dp_t) 013)

/* S22.1 erasables.                                                       */
typedef struct ln_state
{
    uint16_t     svmrkdat[LN_SVMRK]; /* SVMRKDAT: marks (time DP, 5 CDUs)   */
    agc_sp_t     i;           /* 8KK: present mark, 1..N                  */
    agc_sp_t     n;           /* 8NN: number of marks                     */
    int          loc;         /* S22LOC: word index of mark I             */
    int          eorm;        /* S22EORM: 0 earth, -2 moon                */
    agc_dp_t     toff, tprim; /* S22TOFF, S22TPRIM: cs 2^28               */
    agc_vec_t    uoff;        /* S22UOFF: UM of the offset mark           */
    agc_vec_t    csmpos, um, rclp, ustar; /* CSMPOS UM RCLP USTAR            */
    agc_dp_t     d, rho;     /* S22D (B-29), S22RHO                      */
    agc_dp_t     n49disp[2]; /* N49DISP: |dR| B-29, |dV| B-7             */
    agc_incorp_t inc;        /* BVECTOR DELTAQ VARIANCE DELTAX           */
    agc_job_id_t ret;        /* S22RTNEX                                 */
    agc_sp_t     cxoff;      /* CXOFF: offset point mark (LANDMARK B)    */
    bool         first;      /* FIG2EXIT (first mark) or S2INTS1         */
} ln_state_t;

static ln_state_t ln;

/* The DP element of W at word offset o (W + 54b + 6m + 2c).              */
static agc_dp_t *ln_w(int o)
{
    agc_incorp_w_t *w = &agc_integ_shared_ctx()->w;
    if (!AGC_ASSERT(o >= 0 && o < 162 && (o % 2) == 0))
    {
        return &w->blk[0][0].c[0];
    }
    int b = o / 54;
    int r = o % 54;
    return &w->blk[b][r / 6].c[(r % 6) / 2];
}

/* GETTF: the time of mark I (SVMRKDAT at S22LOC).                        */
static agc_dp_t ln_gettf(void)
{
    (void) AGC_ASSERT(ln.loc >= 0 && ln.loc + 1 < LN_SVMRK);
    return agc_dp_pack((agc_sp_t) ln.svmrkdat[ln.loc],
                       (agc_sp_t) ln.svmrkdat[ln.loc + 1]);
}

/* S22FLGS: DIM0FLAG, D6OR9FLG, VINTFLAG (CSM), STATEFLG on, precision.   */
static void ln_flags(void)
{
    rz_flag_put(AGC_FLAG_DIM0FLAG, true);
    rz_flag_put(AGC_FLAG_D6OR9FLG, true);
    rz_flag_put(AGC_FLAG_VINTFLAG, true);
    rz_flag_put(AGC_FLAG_STATEFLG, true);
    rz_flag_put(AGC_FLAG_INTYPFLG, false);
}

/* S22CALRC: RC = TDELTAV (VSR* 7,1) + RCV of the CSM, body scale.        */
static agc_status_t ln_calrc(agc_interp_t *ip)
{
    agc_integ_sv_t sv;
    agc_status_t   st = agc_integ_perm_get(AGC_VEHICLE_CSM, &sv);
    ln.csmpos =
        agc_vec_add(ip, rz_vshift(ip, sv.deltav, -(7 - ln.eorm)), sv.rcv);
    return st;
}

/* S22F2410 (formula 2.4.10): RHO = |RC| (COSA - SQRT((R0/RC)^2 - SINA^2)),
 * COSA = -UM . UNIT(RC); RL = RC + RHO UM (X789). R0 = ERADM (B-29; SL2
 * in the moon's sphere).                                                 */
static void ln_f2410(agc_interp_t *ip, agc_dp_t eradm)
{
    agc_dp_t  rc   = 0;
    agc_vec_t u    = agc_vec_unit(ip, ln.csmpos, &rc, NULL);
    agc_dp_t  cosa = agc_dp_neg(agc_dp_shl(ip, agc_vec_dot(ip, u, ln.um), 1));
    agc_dp_t  sin2 = agc_dp_sub(ip, AGC_DP_QUARTER, agc_dp_sq(ip, cosa));
    agc_dp_t  r0   = (ln.eorm != 0) ? agc_dp_shl(ip, eradm, 2) : eradm;
    agc_dp_t  q    = agc_dp_div(ip, agc_dp_shr_round(r0, 1), rc); /* B-1 */
    agc_dp_t  s    = agc_dp_sqrt(ip, agc_dp_sub(ip, agc_dp_sq(ip, q), sin2));
    ln.rho         = agc_dp_mul_trunc(agc_dp_sub(ip, cosa, s), rc);
    agc_vec_t rl   = agc_vec_shl(ip, agc_vec_scale(ip, ln.um, ln.rho), 2);
    agc_erase_set_x789(agc_vec_add(ip, rl, ln.csmpos));
}

/* S2231X13: the 3x3 (a)(b T): row k = b x a[k].                          */
static agc_mat_t ln_outer(agc_interp_t *ip, agc_vec_t a, agc_vec_t b)
{
    agc_mat_t m;
    for (int k = 0; k < 3; k++)
    {
        m.row[k] = agc_vec_scale(ip, b, a.c[k]);
    }
    (void) AGC_ASSERT(m.row[0].c[0] >= AGC_DP_MIN);
    return m;
}

/* W72 .. W78 (pp.602-603): the landmark rows 6-8 of W (column block 2)
 * as the lower triangular square root of E (E0..E5 = (0,0) (1,0) (1,1)
 * (2,0) (2,1) (2,2)); a negative or zero radicand, or an overflowing
 * quotient, leaves the element(s) zero; then the rows SR1 (B-19).      */
static void ln_cholesky(agc_interp_t *ip, const agc_mat_t *e)
{
    rz_clear_ovf(ip);
    if (e->row[2].c[2] >= 0)
    {
        agc_dp_t w74 = agc_dp_sqrt(ip, e->row[2].c[2]);
        if (w74 != 0)
        {
            *ln_w(148)   = w74;
            agc_dp_t w73 = agc_dp_div(ip, e->row[2].c[1], w74);
            if (!agc_interp_test_ovf(ip))
            {
                *ln_w(146) = w73; /* W73      */
            }
            agc_dp_t w72 = agc_dp_div(ip, e->row[2].c[0], w74); /* S22W72X */
            if (!agc_interp_test_ovf(ip))
            {
                *ln_w(144) = w72;
            }
        }
    }
    agc_dp_t r76 = agc_dp_sub(ip, e->row[1].c[1], agc_dp_sq(ip, *ln_w(146)));
    agc_dp_t w76 = (r76 >= 0) ? agc_dp_sqrt(ip, r76) : 0; /* S22W76X  */
    if (w76 != 0)
    {
        *ln_w(152)   = w76;
        agc_dp_t num = agc_dp_sub(ip, e->row[1].c[0],
                                  agc_dp_mul_trunc(*ln_w(144), *ln_w(146)));
        agc_dp_t w75 = agc_dp_div(ip, num, w76);
        if (!agc_interp_test_ovf(ip))
        {
            *ln_w(150) = w75;
        }
    }
    agc_dp_t r78 = agc_dp_sub(
        ip, e->row[0].c[0], /* S22W78X  */
        agc_dp_add(ip, agc_dp_sq(ip, *ln_w(144)), agc_dp_sq(ip, *ln_w(150))));
    if (r78 >= 0)
    {
        *ln_w(156) = agc_dp_sqrt(ip, r78);
    }
    agc_incorp_w_t *w = &agc_integ_shared_ctx()->w; /* S22SCLW  */
    for (int m = 6; m < LN_ROWS; m++)
    {
        w->blk[2][m] = agc_vec_shr(w->blk[2][m], 1);
    }
}

/* The small E matrix (pp.601-602): 1/2 RHO^2 VARSCT (U U T) (RHO NORM,
 * X2 doubled, VSR* 0-12D,2) + RPVAR (R0 SR3 / D)^2 (UM UM T); the
 * landmark rows of W cleared, then W72 .. W78.                          */
static agc_status_t ln_e_matrix(agc_interp_t *ip, const agc_mat_t *u,
                                agc_dp_t eradm)
{
    agc_dp_t rho = (ln.eorm != 0) ? agc_dp_shr(ln.rho, 2) : ln.rho; /* B-30 */
    int      x2  = 0;
    agc_dp_t n   = rz_norm(rho, &x2);
    x2           = 2 * x2; /* XAD,2 X2 */
    agc_dp_t  r2 = agc_dp_shr(agc_dp_mul_trunc(agc_dp_sq(ip, n), LN_SCTVAR), 1);
    agc_mat_t e;
    for (int r = 0; r < 3; r++)
    { /* S22NXXA  */
        agc_vec_t v = agc_vec_scale(ip, agc_mat_vec(ip, u, u->row[r]), r2);
        e.row[r]    = rz_vshift(ip, v, -(-12 - x2));
    }
    agc_mat_t umum = ln_outer(ip, ln.um, ln.um); /* B-2      */
    agc_dp_t  q    = agc_dp_div(ip, agc_dp_shr(eradm, 3), ln.d);
    agc_dp_t s = agc_dp_mul_trunc(agc_dp_sq(ip, q), agc_rendz_nav_pad()->rpvar);
    agc_incorp_w_t *w = &agc_integ_shared_ctx()->w;
    for (int r = 0; r < 3; r++)
    { /* S22NXXB  */
        e.row[r] = agc_vec_add(ip, agc_vec_scale(ip, umum.row[r], s), e.row[r]);
        w->blk[2][6 + r] = agc_vec_zero();
    }
    rz_clear_ovf(ip); /* BOV +1   */
    ln_cholesky(ip, &e);
    return AGC_STATUS_OK;
}

/* DMPINTEG, unknown landmark (pp.600-603): RL from formula 2.4.10 (X789);
 * U = I - (UM)(RL T)/D (B-3), D = UM . RL; W rows 0-5 of column block 2
 * = U x (rows 0-5 of block 0) VSL3; E = 1/2 RHO^2 VARSCT U U T + RPVAR
 * (R0/D)^2 UM UM T; the landmark rows from E (W72 .. W78).             */
static agc_status_t ln_unknown_landmark(agc_interp_t *ip)
{
    agc_navaux_padload_t pad;
    agc_status_t         st  = agc_navaux_get_padload(&pad);
    agc_latlong_opt_t    opt = {agc_flag_test(AGC_FLAG_CMOONFLG), true};
    rz_flag_put(AGC_FLAG_LUNAFLAG, opt.lunar);
    agc_vec_t alphav = agc_vec_unit(ip, ln.csmpos, NULL, NULL);
    agc_dp_t  eradm  = 0;
    if (st == AGC_STATUS_OK)
    {
        st = agc_latlong_setre(ip, &pad, opt, alphav.c[2], &eradm); /* SETRE */
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    ln_f2410(ip, eradm);
    agc_vec_t rl   = agc_erase_x789();
    rl             = (ln.eorm != 0) ? agc_vec_shr(rl, 2) : rl; /* S22RL B-29 */
    ln.d           = agc_dp_shl(ip, agc_vec_dot(ip, rl, ln.um), 1);
    agc_mat_t umrl = ln_outer(ip, ln.um, rl); /* B-30     */
    agc_mat_t u;
    for (int k = 0; k < 3; k++)
    { /* S22NXTU  */
        agc_vec_t id = agc_vec_zero();
        id.c[k]      = AGC_DP_HALF >> 2; /* 1 B-3    */
        u.row[k]     = agc_vec_sub(
                ip, id, agc_vec_div(ip, agc_vec_shr(umrl.row[k], 2), ln.d));
    }
    agc_incorp_w_t *w = &agc_integ_shared_ctx()->w;
    for (int m = 0; m < 6; m++)
    { /* S22NXTWI */
        w->blk[2][m] = agc_vec_shl(ip, agc_mat_vec(ip, &u, w->blk[0][m]), 3);
    }
    return ln_e_matrix(ip, &u, eradm);
}

/* ---- 9DWTO6DW / ROWDOT (pp.610-614) ------------------------------------ */
/* ROWDOT: sum over the nine W rows of W(word I) W(word J) (DMPR, DAD); an
 * overflow invalidates W (ORBWFLAG off) and ends the sum.               */
static agc_dp_t ln_rowdot(agc_interp_t *ip, int wi, int wj)
{
    rz_clear_ovf(ip);
    agc_dp_t sum = 0;
    for (int k = 0; k < LN_ROWS; k++)
    {
        sum = agc_dp_add(
            ip, agc_dp_mul(ip, *ln_w(wi + 6 * k), *ln_w(wj + 6 * k)), sum);
        if (agc_interp_test_ovf(ip))
        {
            rz_flag_put(AGC_FLAG_ORBWFLAG, false); /* ROWDOT3  */
            break;
        }
    }
    return sum;
}

/* The next lower column word offset of the 6x6 part: 58 56 54 4 2 0
 * (TEST I=26: 52 becomes 4).                                             */
static int ln_9dw_next(int x)
{
    int n = x - 2; /* 9DWID    */
    (void) AGC_ASSERT(x > 0);
    return (n == 52) ? 4 : n;
}

/* 9DWTO6DW: E = the 21 dot products of the six position and velocity
 * columns of W (lower triangle, EMATRIX), then the new 6x6 W by a
 * column-wise square root of E; the landmark block cleared.           */
static void ln_9dw_to_6dw(agc_interp_t *ip)
{
    agc_dp_t e[LN_EMATRIX];
    int      p = 0;
    for (int j = 58; p < LN_EMATRIX; j = ln_9dw_next(j))
    { /* 9DWI=J   */
        for (int i = j; p < LN_EMATRIX; i = ln_9dw_next(i))
        {
            e[LN_EMATRIX - 1 - p++] = ln_rowdot(ip, i, j); /* 9DWEPCAL */
            if (i == 0)
            {
                break;
            }
        }
        if (j == 0)
        {
            break;
        }
    }
    rz_phase(2, 04022); /* 9DWFIG6  */
    agc_incorp_w_t *w = &agc_integ_shared_ctx()->w;
    for (int m = 0; m < LN_ROWS; m++)
    { /* CLEARW54 */
        w->blk[0][m] = agc_vec_zero();
        w->blk[1][m] = agc_vec_zero();
    }
    p     = 0;
    int n = 0;
    for (int j = 58; p < LN_EMATRIX; j = ln_9dw_next(j), n += 6)
    {
        agc_dp_t wsq =
            agc_dp_sub(ip, e[LN_EMATRIX - 1 - p++], ln_rowdot(ip, j, j));
        agc_dp_t workw = (wsq >= 0) ? agc_dp_sqrt(ip, wsq) : 0; /* 9DWAAA */
        *ln_w(j + n)   = workw;
        for (int i = j; i != 0 && p < LN_EMATRIX;)
        { /* TST2I=0  */
            i = ln_9dw_next(i);
            rz_clear_ovf(ip);
            agc_dp_t v = agc_dp_div(
                ip,
                agc_dp_sub(ip, e[LN_EMATRIX - 1 - p++], ln_rowdot(ip, i, j)),
                workw);
            *ln_w(i + n) = agc_interp_test_ovf(ip) ? 0 : v; /* SETWIN=0 */
        }
        if (j == 0)
        {
            break;
        }
    }
    rz_phase(2, 04022); /* 9DWEXITX */
    for (int m = 0; m < LN_ROWS; m++)
    {
        w->blk[2][m] = agc_vec_zero();
    }
}

/* ---- S22.1 entry (p.598): the marks to SVMRKDAT (safe against a restart),
 * P22MKFLG, MKRELEAS, restart at priority 13; S22EORM.                  */
agc_status_t rz_landmark_marks_start(agc_sp_t nmarks, agc_job_id_t ret)
{
    if (!AGC_ASSERT(nmarks >= 0 && nmarks <= 5) ||
        !AGC_ASSERT(ret != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    ln.n   = nmarks;
    ln.ret = ret;
    return rz_continue(AGC_JOB_ORBITAL_NAVIGATION_MARKS);
}

agc_status_t agc_job_orbital_navigation_marks(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    ln.i   = 1;
    ln.loc = 0;
    for (int k = 0; k < 5; k++)
    { /* S22.111  */
        agc_align_sxt_mark_t m;
        agc_status_t         st = agc_sxtmark_mark(k, &m);
        uint16_t            *d  = &ln.svmrkdat[LN_MARK_WORDS * k];
        d[0]                    = rz_word_of(agc_dp_high(m.t));
        d[1]                    = rz_word_of(agc_dp_low(m.t));
        d[2]                    = m.cduy;
        d[3]                    = m.cdus;
        d[4]                    = m.cduz;
        d[5]                    = m.cdut;
        d[6]                    = m.cdux;
        (void) AGC_ASSERT(st == AGC_STATUS_OK || k >= ln.n);
    }
    rz_flag_put(AGC_FLAG_ORBITAL_NAVIGATION_MARK_TAKEN, true); /* P22MKFLG */
    agc_status_t st = agc_sxtmark_mkreleas();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    rz_phase_job(04U, 05022U, AGC_JOB_ORBITAL_NAVIGATION_MARK, 013);
    ln.eorm  = agc_flag_test(AGC_FLAG_CMOONFLG) ? -2 : 0;
    ln.first = true;
    return agc_job_orbital_navigation_mark(ctx);
}

/* FIG2EXIT (first mark): W invalid: 6x6 W from WORBPOS / WORBVEL with no
 * W integration (D = 0); else D = 6. S2INTS1 (other marks): W integrated
 * 9x9 once landmark rows exist (DMENFLG), else 6x6 when valid.          */
static void ln_w_setup(void)
{
    if (!ln.first)
    {
        if (!agc_flag_test(AGC_FLAG_DMENFLG))
        {
            rz_flag_put(AGC_FLAG_DIM0FLAG, agc_flag_test(AGC_FLAG_ORBWFLAG));
            rz_flag_put(AGC_FLAG_D6OR9FLG, false);
        }
        return;
    }
    if (!agc_flag_test(AGC_FLAG_ORBWFLAG))
    { /* SETWW5D  */
        const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad();
        agc_incorp_w_t            *w = &agc_integ_shared_ctx()->w;
        rz_flag_put(AGC_FLAG_DIM0FLAG, false);
        rz_flag_put(AGC_FLAG_RENDWFLG, false);
        for (int m = 0; m < LN_ROWS; m++)
        { /* CLEARWW5 */
            w->blk[0][m] = agc_vec_zero();
            w->blk[1][m] = agc_vec_zero();
        }
        for (int k = 0; k < 3; k++)
        {
            w->blk[0][k].c[k]     = agc_dp_from_sp(p->worbpos);
            w->blk[1][3 + k].c[k] = agc_dp_from_sp(p->worbvel);
        }
    }
    else
    {
        rz_flag_put(AGC_FLAG_D6OR9FLG, false);
    }
    rz_flag_put(AGC_FLAG_DMENFLG, false); /* SETVANDI */
}

/* GETUM for mark I (SVMRKDAT at S22LOC): UM in reference axes.          */
static agc_status_t ln_getum(agc_interp_t *ip)
{
    const uint16_t      *d = &ln.svmrkdat[ln.loc];
    agc_align_sxt_mark_t m;
    m.t    = ln_gettf();
    m.cduy = d[2];
    m.cdus = d[3];
    m.cduz = d[4];
    m.cdut = d[5];
    m.cdux = d[6];
    for (int k = 0; k < LN_MARK_WORDS; k++)
    {
        agc_erase_set_markdown(k, d[k]);
    }
    agc_vec_t    los      = agc_vec_zero();
    agc_status_t st       = agc_csm_sxtsm(ip, &m, &los);
    agc_mat_t    refsmmat = agc_erase_refsmmat();
    ln.um                 = agc_vec_shl(ip, agc_vec_mat(ip, los, &refsmmat), 1);
    return st;
}

/* S22BOX12: (for the CSM, 9x9) VARIANCE = (SCTVAR + IMUVARR) |RCLP|^2
 * (NORM, X1 doubled + 2 S22EORM, SR* 0,1; TP); BVECTORS with B2 = -B0;
 * INCORP1. True when the first incorporation wants the noun 49 display. */
static bool ln_box12(agc_interp_t *ip, agc_status_t *st)
{
    rz_flag_put(AGC_FLAG_DMENFLG, true);
    rz_flag_put(AGC_FLAG_VEHUPFLG, true);
    int      x1 = 0;
    agc_dp_t n  = rz_norm(agc_vec_abval(ip, ln.rclp), &x1);
    agc_tp_t v =
        agc_dp_mul_tp(agc_dp_sq(ip, n), agc_dp_add(ip, LN_SCTVAR, LN_IMUVARR));
    x1              = 2 * x1 + 2 * ln.eorm;
    ln.inc.variance = agc_tp_shift(ip, v, x1);           /* SR* 0,1  */
    rz_bvectors(ip, ln.rclp, ln.um, &ln.ustar, &ln.inc); /* S2BVTRS  */
    ln.inc.bvector[2] = agc_vec_neg(ln.inc.bvector[0]);  /* B2 = -B0 */
    ln.inc.dim9       = true;
    *st               = agc_incorp1(ip, &agc_integ_shared_ctx()->w, &ln.inc);
    rz_phase(2, 04022);
    if (*st != AGC_STATUS_OK || !agc_flag_test(AGC_FLAG_22DSPFLG))
    {
        return false;
    }
    rz_flag_put(AGC_FLAG_22DSPFLG, false);
    rz_phase(2, 04022);
    ln.n49disp[0] = rz_shift(ip, agc_vec_abval(ip, ln.inc.deltax[0]), ln.eorm);
    ln.n49disp[1] = rz_shift(ip, agc_vec_abval(ip, ln.inc.deltax[1]), ln.eorm);
    return true;
}

/* S22BOX32: X789 = RL; first pass; RCLP = RL - RC; USTAR =
 * UNIT(UNIT(RCLP) x UM). False on an overflow (the mark is skipped).    */
static bool ln_box32(agc_interp_t *ip, agc_vec_t rl)
{
    agc_erase_set_x789(rl);
    rz_flag_put(AGC_FLAG_INCORFLG, true);
    rz_clear_ovf(ip);
    ln.rclp     = agc_vec_sub(ip, rl, ln.csmpos);
    agc_vec_t u = agc_vec_unit(ip, ln.rclp, NULL, NULL);
    ln.ustar    = agc_vec_unit(ip, agc_vec_cross(ip, u, ln.um), NULL, NULL);
    return !agc_interp_test_ovf(ip);
}

/* S22SAVET / S22I=N: TPRIM = TF; after the last mark S22F244, else the
 * next mark (S2INTS1).                                                   */
static agc_status_t ln_next(void)
{
    ln.tprim = ln_gettf(); /* S22SAVET */
    rz_phase(2, 04022);
    if (ln.i >= ln.n)
    {
        return rz_continue(AGC_JOB_ORBITAL_NAVIGATION_SITE); /* S22F244X */
    }
    ln.i++;
    ln.loc += LN_MARK_WORDS;
    ln.first = false;
    rz_phase(2, 04022);
    return rz_continue(AGC_JOB_ORBITAL_NAVIGATION_MARK);
}

/* S22BOX12 .. S22BOX42: incorporate the mark twice (the second pass with
 * the updated CSM and landmark); the first incorporation of a sighting
 * may stop for the noun 49 display.                                      */
static agc_status_t ln_incorporate(agc_job_ctx_t *ctx, agc_interp_t *ip,
                                   bool at_box42)
{
    for (int pass = 0; pass < 3; pass++)
    {
        agc_status_t st = AGC_STATUS_OK;
        if (!at_box42 && ln_box12(ip, &st))
        {
            return rz_flash(ctx, AGC_DSKY_GOFLASHR, 6U, 49U, AGC_JOB_GOTOPOOH,
                            AGC_JOB_ORBITAL_NAVIGATION_DELTA_ACCEPT,
                            AGC_JOB_ORBITAL_NAVIGATION_MARKS_EXIT);
        }
        if (st != AGC_STATUS_OK)
        {
            return st;
        }
        at_box42 = false;
        st       = rz_incorp2(ip, &ln.inc,
                              agc_flag_test(AGC_FLAG_CMOONFLG)); /* S22BOX42 */
        if (st == AGC_STATUS_OK)
        {
            st = ln_calrc(ip);
        }
        if (st != AGC_STATUS_OK)
        {
            return st;
        }
        if (!agc_flag_test(AGC_FLAG_INCORFLG))
        { /* DMPINCP2 */
            return ln_next();
        }
        rz_phase(2, 04022);
        rz_flag_put(AGC_FLAG_INCORFLG, false);
        ln.rclp = agc_vec_sub(ip, agc_erase_x789(), ln.csmpos);
    }
    (void) AGC_ASSERT(false);
    return AGC_STATUS_NO_CONVERGE;
}

/* S22BOX22, a known landmark: the landmark rows of W from S22WSUBL; RL
 * from LAT LONG ALT at TF (LALOTORV, SL2 in the moon's sphere).        */
static agc_status_t ln_known_landmark(agc_interp_t *ip, agc_vec_t *rl)
{
    agc_incorp_w_t *w = &agc_integ_shared_ctx()->w;
    for (int m = 0; m < LN_ROWS; m++)
    { /* CLRW678  */
        w->blk[2][m] = agc_vec_zero();
    }
    agc_dp_t wl = agc_dp_from_sp(agc_rendz_nav_pad()->s22wsubl);
    *ln_w(144)  = wl;
    *ln_w(152)  = wl;
    *ln_w(160)  = wl;
    bool moon   = agc_flag_test(AGC_FLAG_CMOONFLG);
    rz_flag_put(AGC_FLAG_LUNAFLAG, moon);
    agc_navaux_padload_t pad;
    agc_latlong_t        ll;
    agc_status_t         st = agc_navaux_get_padload(&pad);
    if (st == AGC_STATUS_OK)
    {
        st = agc_navaux_get_latlong(&ll);
    }
    agc_latlong_opt_t opt = {moon, agc_flag_test(AGC_FLAG_ERADFLAG)};
    if (st == AGC_STATUS_OK)
    {
        st = agc_latlong_to_r(ip, &pad, opt, &ll, ln_gettf(), rl, NULL);
    }
    *rl = moon ? agc_vec_shl(ip, *rl, 2) : *rl; /* S22BX22B */
    return st;
}

/* DMPINTEG (p.600): the offset point mark is kept (S22OFF=I); with the
 * landmark rows in W the landmark is moved to this mark time (R-TO-RP,
 * RP-TO-R) and the mark incorporated (S22D=9); on the first other mark
 * the landmark is established.                                          */
static agc_status_t ln_dmpinteg(agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    if (ln.i == (agc_sp_t) (ln.cxoff >> 9))
    { /* S22OFF=I */
        ln.toff = ln_gettf();
        ln.uoff = ln.um;
        return ln_next();
    }
    agc_vec_t    rl = agc_erase_x789();
    agc_status_t st = AGC_STATUS_OK;
    if (agc_flag_test(AGC_FLAG_DMENFLG))
    { /* S22D=9   */
        agc_navaux_padload_t pad;
        agc_vec_t            rp;
        bool                 moon = (ln.eorm != 0);
        st                        = agc_navaux_get_padload(&pad);
        if (st == AGC_STATUS_OK)
        {
            st = agc_planet_r_to_rp(ip, &pad, moon, rl, ln.tprim, &rp);
        }
        if (st == AGC_STATUS_OK)
        {
            st = agc_planet_rp_to_r(ip, &pad, moon, rp, ln_gettf(), &rl);
        }
    }
    else
    {
        rz_phase(2, 04022);
        rz_flag_put(AGC_FLAG_ORBWFLAG, true);
        rz_flag_put(AGC_FLAG_DMENFLG, true);
        rz_flag_put(AGC_FLAG_22DSPFLG, true);
        rz_flag_put(AGC_FLAG_ERADFLAG, true);
        if (!agc_flag_test(AGC_FLAG_KNOWNFLG))
        {
            st = ln_unknown_landmark(ip);
            return (st != AGC_STATUS_OK) ? st : ln_next();
        }
        st = ln_known_landmark(ip, &rl);
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    if (!ln_box32(ip, rl))
    {
        return ln_next(); /* S22SAVET */
    }
    return ln_incorporate(ctx, ip, false);
}

/* FIG2EXIT / S2INTS1 / S22NXTIN: INTSTALL, flags, the CSM (with W) to the
 * mark time, RC, UM; then DMPINTEG.                                      */
agc_status_t agc_job_orbital_navigation_mark(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_integ_install(0, AGC_JOB_ORBITAL_NAVIGATION_MARK);
    if (st != AGC_STATUS_OK)
    {
        return st; /* BUSY: asleep in INTSTALL  */
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    ln_flags();
    ln_w_setup();
    st = agc_integ_integrv(&ip, agc_integ_shared_ctx(),
                           ln_gettf()); /* S22NXTIN */
    if (st == AGC_STATUS_OK)
    {
        st = ln_calrc(&ip);
    }
    if (st == AGC_STATUS_OK)
    {
        st = ln_getum(&ip); /* S2GETUM  */
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return ln_dmpinteg(ctx, &ip);
}

/* PROCEED on V06N49: incorporate (S22BOX42) and go on.                  */
agc_status_t agc_job_orbital_navigation_delta_accept(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    return ln_incorporate(ctx, &ip, true);
}

/* S22EXEX (RECYCLE on V06N49): back to P22 without converting W.         */
agc_status_t agc_job_orbital_navigation_marks_exit(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_continue(ln.ret);
}

void rz_landmark_set_offset(agc_sp_t cxoff)
{
    (void) AGC_ASSERT(cxoff >= 0);
    ln.cxoff = cxoff;
}

/* ---- the landmark at the end (S22F244 .. S22.981X pp.607-608) ---------- */
/* S22F244: with an offset point mark (OFF within the marks), the CSM at
 * its time (CSMPREC) and RL along its UM (formula 2.4.10 with R0 =
 * |X789|); LAT LONG ALT of RL at TOFF (else now) for noun 89 (LLASRD).  */
agc_status_t agc_job_orbital_navigation_site(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    bool         moon = agc_flag_test(AGC_FLAG_CMOONFLG);
    agc_sp_t     off  = (agc_sp_t) (ln.cxoff >> 9);
    agc_status_t st   = AGC_STATUS_OK;
    if (off != 0 && ln.n - off >= 0)
    {
        rz_phase(2, 04022);
        agc_integ_out_t o;
        st = rz_integ(&ip, RZ_CSMPREC, ln.toff, AGC_JOB_ORBITAL_NAVIGATION_SITE,
                      &o);
        if (st != AGC_STATUS_OK)
        {
            return st; /* BUSY: asleep in INTSTALL  */
        }
        ln.csmpos   = o.ratt1;
        ln.um       = ln.uoff;
        agc_dp_t r0 = agc_vec_abval(&ip, agc_erase_x789());
        ln_f2410(&ip, moon ? agc_dp_shr(r0, 2) : r0);
    }
    else
    {
        ln.toff = ln_gettf(); /* S22BOX44 */
    }
    rz_flag_put(AGC_FLAG_LUNAFLAG, moon); /* S22BX44A */
    agc_vec_t alphav =
        moon ? agc_vec_shr(agc_erase_x789(), 2) : agc_erase_x789();
    agc_navaux_padload_t pad;
    agc_latlong_t        ll;
    agc_latlong_opt_t    opt = {moon, agc_flag_test(AGC_FLAG_ERADFLAG)};
    st                       = agc_navaux_get_padload(&pad);
    if (st == AGC_STATUS_OK)
    {
        st = agc_latlong_from_r(&ip, &pad, opt, alphav, ln.toff, &ll);
    }
    if (st == AGC_STATUS_OK)
    {
        st = agc_navaux_set_latlong(&ll);
    }
    if (st == AGC_STATUS_OK)
    {
        st = agc_rendz_llasrd();
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return rz_flash(ctx, AGC_DSKY_GOFLASH, 6U, 89U,
                    AGC_JOB_ORBITAL_NAVIGATION_SITE_TERMINATE,
                    AGC_JOB_ORBITAL_NAVIGATION_SITE_ACCEPT,
                    AGC_JOB_ORBITAL_NAVIGATION_SITE_DONE);
}

/* PROCEED on V06N89: the landing site RLS = R-TO-RP(RL, TOFF) (moon
 * fixed), then S22.981X.                                                 */
agc_status_t agc_job_orbital_navigation_site_accept(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_navaux_padload_t pad;
    agc_status_t         st = agc_navaux_get_padload(&pad);
    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_r_to_rp(&ip, &pad, true, agc_erase_x789(), ln.toff,
                                &pad.rls);
    }
    if (st == AGC_STATUS_OK)
    {
        st = agc_navaux_set_padload(&pad);
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return agc_job_orbital_navigation_site_done(ctx);
}

/* S22.981X (also RECYCLE): W back to 6x6; return to P22 (point A).       */
agc_status_t agc_job_orbital_navigation_site_done(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    ln_9dw_to_6dw(&ip);
    return rz_continue(ln.ret);
}

/* S22GTP (TERMINATE on V06N89): W back to 6x6, then P00.                 */
agc_status_t agc_job_orbital_navigation_site_terminate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    ln_9dw_to_6dw(&ip);
    return rz_gotopooh();
}
