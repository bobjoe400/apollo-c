/**********************************************************************
 * FILE NAME: landmark_navigation.c
 *
 * PURPOSE:
 *   S22.1 ORBITAL NAVIGATION ROUTINE, CSM: S22.1, S22F2410, S22CALRC,
 *   S2231X13, GETTF, S22FLGS, 9DWTO6DW, ROWDOT. "1. Update CSM state
 *   vector. 2. Update landmark position. 3. Convert W matrix from 9 to 6
 *   dimensions." The marks of a P22 landmark sighting (up to five,
 *   SVMRKDAT) are processed in order: the CSM is integrated to each mark
 *   time with its W matrix; the first mark of an unknown landmark sets
 *   the landmark position RL (formula 2.4.10) and the landmark rows of a
 *   9x9 W; every other mark is incorporated (twice, like R22) updating
 *   the CSM and RL (X789). An offset point mark is kept for the end. Then
 *   the landmark is shown (noun 89) and W converted back to 6x6.
 *
 * AGC SOURCE:
 *   Comanche055 P20-P25.agc pp. 598-614 (S22.1, S22.111, FIG2EXIT,
 *   SETWW5D, S2INTS1, S22NXTIN, DMPINTEG, S22F2410, S22NXTU, S22NXTWI,
 *   S22NXXA, S22NXXB, S22W72X .. S22W78X, S22SCLW, S22BOX12 .. S22BOX44,
 *   S22OFF=I, S22D=9, S22SAVET, S22F244, S22BX44A, S22.981X, S22GTP,
 *   S22EXEX, 9DWTO6DW, 9DWEPCAL, 9DWFIG6, 9DWAAA, SETWIN=0, ROWDOT);
 *   CSM only
 *
 * EXTERNAL VARIABLES:
 *   Name        Type       I/O  Scale      Description
 *   ----------  ---------  ---  ---------  ---------------------------
 *   the marks (SXTMARK)    I    raw        time and CDUs of each mark
 *   W           agc_incorp_w_t I/O  as integrated  6x6 / 9x9
 *   X789        agc_vec_t  I/O  body scale landmark position RL
 *   LAT LONG ALT  agc_dp_t I/O  rev, m 2^29  noun 89 landmark
 *   RLS         agc_vec_t  O    m 2^27     landing site (PROCEED)
 *   WORBPOS WORBVEL S22WSUBL RPVAR  pad loads  I
 *   N49DISP     agc_dp_t   O    m 2^29, m/cs 2^7  noun 49
 *   ORBWFLAG DMENFLG 22DSPFLG KNOWNFLG ERADFLAG LUNAFLAG INCORFLG
 *   VEHUPFLG P22MKFLG CMOONFLG DIM0FLAG D6OR9FLG RENDWFLG  flags  I/O
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ----------------------------------------
 *   agc_integ_integrv           INTEGRV (the CSM with W)
 *   agc_csm_sxtsm               SXTNB / NBSM of a mark
 *   agc_latlong_setre           SETRE: the radius at the landmark
 *   agc_planet_r_to_rp          R-TO-RP / RP-TO-R
 *   rz_bvectors                 BVECTORS
 *   agc_incorp_state_deviation  INCORP1
 *   rz_incorp2                  INCORP2
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. An overflow skips a mark (S22BOX32) or invalidates W
 *   (ROWDOT). AGC_STATUS_NO_CONVERGE after three incorporation passes
 *   (cannot happen: two). Integration, planetary and display statuses
 *   are passed back.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Jobs of P22 (CSM); at most five marks.
 *
 * NOTES:
 *   Scales: positions body scale (earth 2^29, moon 2^27; S22EORM = 0
 *   earth, -2 moon), UM half unit, U matrix B-3, RHO B-30, VARIANCE TP
 *   B-40.
 **********************************************************************/
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

#define LN_SCTVAR     ((agc_dp_t) 70368744) /* SCTVAR 2DEC 1.0 E-6 B+18   */
#define LN_IMUVARR    ((agc_dp_t) 2814750)  /* IMUVARR 2DEC 0.04 E-6 B+18 */
#define LN_MARK_WORDS 7                     /* time DP + 5 CDUs */
#define LN_SVMRK      36                    /* SVMRKDAT I(36)             */
#define LN_ROWS       9                     /* W rows */
#define LN_EMATRIX    21                    /* EMATRIX B(42): 21 DP       */

/* S22.1 erasables.                                                       */
typedef struct ln_state
{
    uint16_t     svmrkdat[LN_SVMRK]; /* SVMRKDAT: marks (time DP, 5 CDUs)   */
    agc_sp_t     i;                  /* 8KK: present mark, 1..N, unscaled */
    agc_sp_t     n;                  /* 8NN: number of marks, unscaled */
    int          loc;                /* S22LOC: word index of mark I             */
    int          eorm;               /* S22EORM: 0 earth, -2 moon                */
    agc_dp_t     toff;               /* S22TOFF: offset mark time, cs 2^28       */
    agc_dp_t     tprim;              /* S22TPRIM: last mark time, cs 2^28        */
    agc_vec_t    uoff;               /* S22UOFF: UM of the offset mark, 2^1 */
    agc_vec_t    csmpos;             /* CSMPOS: CSM position, body scale         */
    agc_vec_t    um;                 /* UM: optics LOS, reference, 2^1           */
    agc_vec_t    rclp;               /* RCLP: landmark - CSM, body scale         */
    agc_vec_t    ustar;              /* USTAR: star direction, 2^1               */
    agc_dp_t     d;                  /* S22D: UM . RL, B-29                      */
    agc_dp_t     rho;                /* S22RHO: range to the landmark, B-30      */
    agc_dp_t     n49disp[2];         /* N49DISP: |dR| B-29, |dV| B-7             */
    agc_incorp_t inc;                /* BVECTOR DELTAQ VARIANCE DELTAX           */
    agc_job_id_t ret;                /* S22RTNEX                                 */
    agc_sp_t     cxoff;              /* CXOFF: offset point mark (code B), unscaled */
    bool         first;              /* FIG2EXIT (first mark) or S2INTS1         */
} ln_state_t;

static ln_state_t ln; /* S22.1 erasables */

/**********************************************************************
 * The DP element of W at word offset o (W + 54b + 6m + 2c).
 * AGC: none (W word addressing, W + 54b + 6m + 2c)
 *
 * FUNCTION NAME: ln_w
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   o         int   I    word offset in W, even, 0 .. 160
 *
 * RETURNS: agc_dp_t * -- the DP element (W +0 for a bad offset)
 **********************************************************************/
static agc_dp_t *ln_w(int o)
{
    agc_incorp_w_t *w = &agc_integ_shared_ctx()->w; /* W matrix */

    int b = o / 54; /* column block */
    int r = o % 54; /* word in the block */

    return (&w->blk[b][r / 6].c[(r % 6) / 2]);
}

/**********************************************************************
 * GETTF: the time of mark I (SVMRKDAT at S22LOC).
 * AGC: GETTF (P20-P25.agc p.606)
 *
 * FUNCTION NAME: ln_gettf
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_dp_t -- the time of mark I, cs 2^28
 **********************************************************************/
static agc_dp_t ln_gettf(void)
{
    return (agc_dp_pack((agc_sp_t) ln.svmrkdat[ln.loc], (agc_sp_t) ln.svmrkdat[ln.loc + 1]));
}

/**********************************************************************
 * S22FLGS: DIM0FLAG, D6OR9FLG, VINTFLAG (CSM), STATEFLG on, precision.
 * AGC: S22FLGS (P20-P25.agc p.606)
 *
 * FUNCTION NAME: ln_flags
 *
 * ARGUMENTS: none
 **********************************************************************/
static void ln_flags(void)
{
    rz_flag_put(AGC_FLAG_DIM0FLAG, true);
    rz_flag_put(AGC_FLAG_D6OR9FLG, true);
    rz_flag_put(AGC_FLAG_VINTFLAG, true);
    rz_flag_put(AGC_FLAG_STATEFLG, true);
    rz_flag_put(AGC_FLAG_INTYPFLG, false);
}

/**********************************************************************
 * S22CALRC: RC = TDELTAV (VSR* 7,1) + RCV of the CSM, body scale.
 * AGC: S22CALRC (P20-P25.agc p.606)
 *
 * FUNCTION NAME: ln_calrc
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: none
 **********************************************************************/
static void ln_calrc(agc_interp_t *ip)
{
    agc_integ_sv_t sv; /* permanent state */

    agc_integ_perm_get(AGC_VEHICLE_CSM, &sv);

    ln.csmpos = agc_vec_add(ip, rz_vshift(ip, sv.deltav, -(7 - ln.eorm)), sv.rcv);
}

/**********************************************************************
 * S22F2410 (formula 2.4.10): RHO = |RC| (COSA - SQRT((R0/RC)^2 - SINA^2)),
 * COSA = -UM . UNIT(RC); RL = RC + RHO UM (X789). R0 = ERADM (B-29; SL2
 * in the moon's sphere).
 * AGC: S22F2410 (P20-P25.agc p.601)
 *
 * FUNCTION NAME: ln_landmark_on_surface
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   eradm     agc_dp_t        I    R0, the landmark radius, m 2^29
 **********************************************************************/
static void ln_landmark_on_surface(agc_interp_t *ip, agc_dp_t eradm)
{
    agc_dp_t  rc   = 0;                                      /* |RC|, body scale */
    agc_vec_t u    = agc_vec_unit(ip, ln.csmpos, &rc, NULL); /* UNIT RC, 2^1 */
    agc_dp_t  c    = agc_vec_dot(ip, u, ln.um);              /* UM . URC, 2^2  */
    agc_dp_t  cosa = agc_dp_neg(agc_dp_shl(ip, c, 1));       /* COSA, 2^1      */
    agc_dp_t  cos2 = agc_dp_sq(cosa);                        /* COSA^2, 2^2 */
    agc_dp_t  sin2 = agc_dp_sub(ip, AGC_DP_QUARTER, cos2);   /* SINA^2, 2^2 */
    agc_dp_t  r0   = eradm;                                  /* R0, m 2^29     */

    if (ln.eorm != 0)
    {
        r0 = agc_dp_shl(ip, eradm, 2); /* moon: SL2, m 2^27 */
    }

    agc_dp_t q = agc_dp_div(ip, agc_dp_shr_round(r0, 1), rc); /* B-1 */
    agc_dp_t s = agc_dp_sub(ip, agc_dp_sq(q), sin2);          /* 2^2 */

    s            = agc_dp_sqrt(s);                   /* 2^1 */
    ln.rho       = agc_dp_mul_trunc(agc_dp_sub(ip, cosa, s), rc);
    agc_vec_t rl = agc_vec_scale(ip, ln.um, ln.rho); /* RHO UM, body scale */

    rl = agc_vec_shl(ip, rl, 2);

    agc_erase_set_x789(agc_vec_add(ip, rl, ln.csmpos));
}

/**********************************************************************
 * S2231X13: the 3x3 (a)(b T): row k = b x a[k].
 * AGC: S2231X13 (P20-P25.agc p.606)
 *
 * FUNCTION NAME: ln_outer
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   a         agc_vec_t       I    column vector
 *   b         agc_vec_t       I    row vector
 *
 * RETURNS: agc_mat_t -- the 3x3 (a)(b T), scale of a x b
 **********************************************************************/
static agc_mat_t ln_outer(agc_interp_t *ip, agc_vec_t a, agc_vec_t b)
{
    agc_mat_t m; /* matrix rows, unit 2^1 */

    for (int k = 0; k < 3; k++)
    {
        m.row[k] = agc_vec_scale(ip, b, a.c[k]);
    }

    return (m);
}

/**********************************************************************
 * The small E matrix (pp.601-602): 1/2 RHO^2 VARSCT (U U T) (RHO NORM,
 * X2 doubled, VSR* 0-12D,2) + RPVAR (R0 SR3 / D)^2 (UM UM T); the
 * landmark rows of W cleared, then W72 .. W78.
 * AGC: S22NXXA, S22NXXB, S22W72X .. S22W78X, S22SCLW (P20-P25.agc
 * pp.601-603)
 *
 * FUNCTION NAME: ln_e_matrix
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND)
 *   u         const agc_mat_t *  I    U, B-3
 *   eradm     agc_dp_t           I    R0, m 2^29
 *
 * RETURNS: none
 **********************************************************************/
static void ln_e_matrix(agc_interp_t *ip, const agc_mat_t *u, agc_dp_t eradm)
{
    agc_dp_t rho = ln.rho; /* RHO, B-30 */

    if (ln.eorm != 0)
    {
        rho = agc_dp_shr(ln.rho, 2); /* moon: to B-30 */
    }

    int      x2 = 0;                 /* NORM index, -(left shifts) */
    agc_dp_t n  = rz_norm(rho, &x2); /* RHO, 2^(-30+X2) */

    x2 = 2 * x2; /* XAD,2 X2 */
    agc_dp_t r2; /* 1/2 RHO^2 VARSCT, B-42 */

    r2 = agc_dp_shr(agc_dp_mul_trunc(agc_dp_sq(n), LN_SCTVAR), 1);
    agc_mat_t e; /* E, B-38 */

    for (int r = 0; r < 3; r++)
    { /* S22NXXA  */
        agc_vec_t v; /* velocity, m/cs 2^7 */

        v = agc_vec_scale(ip, agc_mat_vec(ip, u, u->row[r]), r2);

        e.row[r] = rz_vshift(ip, v, -(-12 - x2));
    }

    agc_mat_t umum = ln_outer(ip, ln.um, ln.um); /* B-2      */
    agc_dp_t  q;                                 /* R0 SR3 / D, 2^0 */

    q                 = agc_dp_div(ip, agc_dp_shr(eradm, 3), ln.d);
    agc_dp_t        s = agc_dp_mul_trunc(agc_dp_sq(q), agc_rendz_nav_pad()->rpvar); /* RPVAR scale */
    agc_incorp_w_t *w = &agc_integ_shared_ctx()->w;                                 /* W matrix */

    for (int r = 0; r < 3; r++)
    { /* S22NXXB  */
        e.row[r]         = agc_vec_add(ip, agc_vec_scale(ip, umum.row[r], s), e.row[r]);
        w->blk[2][6 + r] = agc_vec_zero();
    }

    /* W72 .. W78 (pp.602-603): the landmark rows 6-8 of W (column block 2)
     * as the lower triangular square root of E (E0..E5 = (0,0) (1,0) (1,1)
     * (2,0) (2,1) (2,2)); a negative or zero radicand, or an overflowing
     * quotient, leaves the element(s) zero; then the rows SR1 (B-19).      */
    agc_interp_clear_ovf(ip); /* BOV +1   */

    if (e.row[2].c[2] >= 0)
    {
        agc_dp_t w74 = agc_dp_sqrt(e.row[2].c[2]); /* W74, B-19 */

        if (w74 != 0)
        {
            *ln_w(148)   = w74;
            agc_dp_t w73 = agc_dp_div(ip, e.row[2].c[1], w74); /* W73, B-19 */

            if (!agc_interp_test_ovf(ip))
            {
                *ln_w(146) = w73; /* W73      */
            }

            agc_dp_t w72; /* S22W72X, B-19 */

            w72 = agc_dp_div(ip, e.row[2].c[0], w74);

            if (!agc_interp_test_ovf(ip))
            {
                *ln_w(144) = w72;
            }
        }
    }

    agc_dp_t r76; /* B-38 */

    r76          = agc_dp_sub(ip, e.row[1].c[1], agc_dp_sq(*ln_w(146)));
    agc_dp_t w76 = 0; /* S22W76X, B-19 */

    if (r76 >= 0)
    {
        w76 = agc_dp_sqrt(r76);
    }

    if (w76 != 0)
    {
        *ln_w(152)   = w76;
        agc_dp_t num = agc_dp_sub(ip, e.row[1].c[0], agc_dp_mul_trunc(*ln_w(144), *ln_w(146))); /* B-38 */
        agc_dp_t w75 = agc_dp_div(ip, num, w76);                                                /* W75, B-19 */

        if (!agc_interp_test_ovf(ip))
        {
            *ln_w(150) = w75;
        }
    }

    agc_dp_t r78 = agc_dp_sub(ip, e.row[0].c[0], agc_dp_add(ip, agc_dp_sq(*ln_w(144)), agc_dp_sq(*ln_w(150)))); /* S22W78X, B-38 */

    if (r78 >= 0)
    {
        *ln_w(156) = agc_dp_sqrt(r78);
    }

    for (int m = 6; m < LN_ROWS; m++)
    { /* S22SCLW  */
        w->blk[2][m] = agc_vec_shr(w->blk[2][m], 1);
    }
} /* ln_e_matrix */

/**********************************************************************
 * DMPINTEG, unknown landmark (pp.600-603): RL from formula 2.4.10 (X789);
 * U = I - (UM)(RL T)/D (B-3), D = UM . RL; W rows 0-5 of column block 2
 * = U x (rows 0-5 of block 0) VSL3; E = 1/2 RHO^2 VARSCT U U T + RPVAR
 * (R0/D)^2 UM UM T; the landmark rows from E (W72 .. W78).
 * AGC: DMPINTEG unknown landmark, S22NXTU, S22NXTWI (P20-P25.agc
 * pp.600-603)
 *
 * FUNCTION NAME: ln_unknown_landmark
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: none
 **********************************************************************/
static void ln_unknown_landmark(agc_interp_t *ip)
{
    agc_navaux_padload_t pad; /* pad loads */
    agc_latlong_opt_t    opt = {agc_flag_test(AGC_FLAG_CMOONFLG), true};

    agc_navaux_get_padload(&pad);
    rz_flag_put(AGC_FLAG_LUNAFLAG, opt.lunar);
    agc_vec_t alphav; /* ALPHAV, 2^1 */

    alphav          = agc_vec_unit(ip, ln.csmpos, NULL, NULL);
    agc_dp_t  eradm = 0;                                   /* R0, m 2^29 */
    agc_latlong_setre(ip, &pad, opt, alphav.c[2], &eradm); /* SETRE */
    ln_landmark_on_surface(ip, eradm);
    agc_vec_t rl    = agc_erase_x789();                    /* RL, body scale */

    if (ln.eorm != 0)
    {
        rl = agc_vec_shr(rl, 2); /* S22RL, moon: to B-29 */
    }

    ln.d           = agc_dp_shl(ip, agc_vec_dot(ip, rl, ln.um), 1);
    agc_mat_t umrl = ln_outer(ip, ln.um, rl); /* B-30     */
    agc_mat_t u;                              /* U, B-3 */

    for (int k = 0; k < 3; k++)
    { /* S22NXTU  */
        agc_vec_t id = agc_vec_zero(); /* row of I, B-3 */

        id.c[k]  = AGC_DP_HALF >> 2; /* 1 B-3    */
        u.row[k] = agc_vec_sub(ip, id, agc_vec_div(ip, agc_vec_shr(umrl.row[k], 2), ln.d));
    }

    agc_incorp_w_t *w = &agc_integ_shared_ctx()->w; /* W matrix */

    for (int m = 0; m < 6; m++)
    { /* S22NXTWI */
        w->blk[2][m] = agc_vec_shl(ip, agc_mat_vec(ip, &u, w->blk[0][m]), 3);
    }

    ln_e_matrix(ip, &u, eradm);
} /* ln_unknown_landmark */

/* ---- 9DWTO6DW / ROWDOT (pp.610-614) ------------------------------------ */
/**********************************************************************
 * ROWDOT: sum over the nine W rows of W(word I) W(word J) (DMPR, DAD); an
 * overflow invalidates W (ORBWFLAG off) and ends the sum.
 * AGC: ROWDOT (P20-P25.agc p.614)
 *
 * FUNCTION NAME: ln_rowdot
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   wi        int             I    word offset I in a W row
 *   wj        int             I    word offset J in a W row
 *
 * RETURNS: agc_dp_t -- the sum of the products over the nine rows, W scale
 *          squared
 **********************************************************************/
static agc_dp_t ln_rowdot(agc_interp_t *ip, int wi, int wj)
{
    agc_interp_clear_ovf(ip);
    agc_dp_t sum = 0; /* W scale squared */

    for (int k = 0; k < LN_ROWS; k++)
    {
        sum = agc_dp_add(ip, agc_dp_mul(ip, *ln_w(wi + 6 * k), *ln_w(wj + 6 * k)), sum);

        if (agc_interp_test_ovf(ip))
        {
            rz_flag_put(AGC_FLAG_ORBWFLAG, false); /* ROWDOT3  */
            break;
        }
    }

    return (sum);
}

/**********************************************************************
 * The next lower column word offset of the 6x6 part: 58 56 54 4 2 0
 * (TEST I=26: 52 becomes 4).
 * AGC: 9DWID, TEST I=26 (P20-P25.agc p.611)
 *
 * FUNCTION NAME: ln_next_lower_column
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   x         int   I    word offset
 *
 * RETURNS: int -- the next lower column word offset
 **********************************************************************/
static int ln_next_lower_column(int x)
{
    int n = x - 2; /* 9DWID    */

    if (n == 52)
    {
        return (4); /* TEST I=26: from the velocity to the position block */
    }

    return (n);
}

/**********************************************************************
 * 9DWTO6DW: E = the 21 dot products of the six position and velocity
 * columns of W (lower triangle, EMATRIX), then the new 6x6 W by a
 * column-wise square root of E; the landmark block cleared.
 * AGC: 9DWTO6DW (P20-P25.agc pp.610-613)
 *
 * FUNCTION NAME: ln_reduce_w_to_six
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 **********************************************************************/
static void ln_reduce_w_to_six(agc_interp_t *ip)
{
    agc_dp_t e[LN_EMATRIX]; /* EMATRIX, W scale squared */
    int      p = 0;         /* EMATRIX index */

    for (int j = 58; p < LN_EMATRIX; j = ln_next_lower_column(j))
    { /* 9DWI=J   */
        for (int i = j; p < LN_EMATRIX; i = ln_next_lower_column(i))
        {
            e[LN_EMATRIX - 1 - p] = ln_rowdot(ip, i, j); /* 9DWEPCAL */
            p++;

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

    agc_phase_change(04022);                        /* 9DWFIG6  */
    agc_incorp_w_t *w = &agc_integ_shared_ctx()->w; /* W matrix */

    for (int m = 0; m < LN_ROWS; m++)
    { /* CLEARW54 */
        w->blk[0][m] = agc_vec_zero();
        w->blk[1][m] = agc_vec_zero();
    }

    p     = 0;
    int n = 0; /* row offset of column j */

    for (int j = 58; p < LN_EMATRIX; j = ln_next_lower_column(j), n += 6)
    {
        agc_dp_t wsq; /* W scale squared */

        wsq = agc_dp_sub(ip, e[LN_EMATRIX - 1 - p], ln_rowdot(ip, j, j));

        p++;
        agc_dp_t workw = 0; /* 9DWAAA: W diagonal, W scale */

        if (wsq >= 0)
        {
            workw = agc_dp_sqrt(wsq);
        }

        *ln_w(j + n) = workw;

        for (int i = j; i != 0 && p < LN_EMATRIX;)
        { /* TST2I=0  */
            i          = ln_next_lower_column(i);
            agc_interp_clear_ovf(ip);
            agc_dp_t v = agc_dp_div(ip, agc_dp_sub(ip, e[LN_EMATRIX - 1 - p], ln_rowdot(ip, i, j)), workw); /* W scale */

            p++;
            *ln_w(i + n) = v;

            if (agc_interp_test_ovf(ip))
            {
                *ln_w(i + n) = 0; /* SETWIN=0 */
            }
        }

        if (j == 0)
        {
            break;
        }
    }

    agc_phase_change(04022); /* 9DWEXITX */

    for (int m = 0; m < LN_ROWS; m++)
    {
        w->blk[2][m] = agc_vec_zero();
    }
} /* ln_reduce_w_to_six */

/**********************************************************************
 * S22.1 entry (p.598): the marks to SVMRKDAT (safe against a restart),
 * P22MKFLG, MKRELEAS, restart at priority 13; S22EORM.
 * AGC: S22.1 (P20-P25.agc p.598)
 *
 * FUNCTION NAME: rz_landmark_marks_start
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------
 *   nmarks    agc_sp_t      I    8NN, the marks taken (0 .. 5)
 *   ret       agc_job_id_t  I    S22RTNEX: the job that continues
 *                                afterwards
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
agc_status_t rz_landmark_marks_start(agc_sp_t nmarks, agc_job_id_t ret)
{
    if (!AGC_ASSERT(nmarks >= 0 && nmarks <= 5) || !AGC_ASSERT(ret != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    ln.n   = nmarks;
    ln.ret = ret;

    return (rz_continue(AGC_JOB_ORBITAL_NAVIGATION_MARKS));
}

/**********************************************************************
 * S22.111: the marks to SVMRKDAT, P22MKFLG, MKRELEAS, restart at priority
 * 13, S22EORM; the first mark.
 * AGC: S22.111 (P20-P25.agc p.598)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_marks
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_marks(agc_job_ctx_t *ctx)
{
    ln.i   = 1;
    ln.loc = 0;

    for (int k = 0; k < 5; k++)
    { /* S22.111  */
        agc_align_sxt_mark_t m; /* the mark */
        uint16_t            *d; /* SVMRKDAT words */

        agc_sxtmark_mark(k, &m); /* mark k */
        d = &ln.svmrkdat[LN_MARK_WORDS * k];

        d[0] = rz_word_of(agc_dp_high(m.t));
        d[1] = rz_word_of(agc_dp_low(m.t));
        d[2] = m.cduy;
        d[3] = m.cdus;
        d[4] = m.cduz;
        d[5] = m.cdut;
        d[6] = m.cdux;
    }

    rz_flag_put(AGC_FLAG_ORBITAL_NAVIGATION_MARK_TAKEN, true); /* P22MKFLG */
    agc_status_t st = agc_sxtmark_mkreleas();                  /* MKRELEAS */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_phase_job(04U, 05022U, AGC_JOB_ORBITAL_NAVIGATION_MARK, 013);
    ln.eorm = 0; /* S22EORM: earth */

    if (agc_flag_test(AGC_FLAG_CMOONFLG))
    {
        ln.eorm = -2; /* moon */
    }

    ln.first = true;

    return (agc_job_orbital_navigation_mark(ctx));
} /* agc_job_orbital_navigation_marks */

/**********************************************************************
 * FIG2EXIT (first mark): W invalid: 6x6 W from WORBPOS / WORBVEL with no
 * W integration (D = 0); else D = 6. S2INTS1 (other marks): W integrated
 * 9x9 once landmark rows exist (DMENFLG), else 6x6 when valid.
 * AGC: FIG2EXIT, SETWW5D, S2INTS1 (P20-P25.agc p.599)
 *
 * FUNCTION NAME: ln_w_setup
 *
 * ARGUMENTS: none
 **********************************************************************/
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
        const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad(); /* pad loads */
        agc_incorp_w_t            *w;                       /* W matrix */

        w = &agc_integ_shared_ctx()->w;

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

/**********************************************************************
 * GETUM for mark I (SVMRKDAT at S22LOC): UM in reference axes.
 * AGC: S2GETUM (P20-P25.agc p.599)
 *
 * FUNCTION NAME: ln_getum
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 **********************************************************************/
static void ln_getum(agc_interp_t *ip)
{
    const uint16_t      *d = &ln.svmrkdat[ln.loc]; /* mark I words */
    agc_align_sxt_mark_t m;                        /* the mark */

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

    agc_vec_t los      = agc_vec_zero();       /* SM LOS, 2^1 */
    agc_mat_t refsmmat = agc_erase_refsmmat(); /* REFSMMAT, 2^1 */

    agc_csm_sxtsm(ip, &m, &los); /* SXTNB, NBSM */
    ln.um = agc_vec_shl(ip, agc_vec_mat(ip, los, &refsmmat), 1);
}

/**********************************************************************
 * S22BOX12: (for the CSM, 9x9) VARIANCE = (SCTVAR + IMUVARR) |RCLP|^2
 * (NORM, X1 doubled + 2 S22EORM, SR* 0,1; TP); BVECTORS with B2 = -B0;
 * INCORP1. True when the first incorporation wants the noun 49 display.
 * AGC: S22BOX12 (P20-P25.agc p.604)
 *
 * FUNCTION NAME: ln_variance_and_incorporate
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   st        agc_status_t *  O    the INCORP1 status
 *
 * RETURNS: bool -- true when the first incorporation wants the noun 49
 *          display
 **********************************************************************/
static bool ln_variance_and_incorporate(agc_interp_t *ip, agc_status_t *st)
{
    rz_flag_put(AGC_FLAG_DMENFLG, true);
    rz_flag_put(AGC_FLAG_VEHUPFLG, true);
    int      x1 = 0;                                        /* NORM index, -(left shifts) */
    agc_dp_t n  = rz_norm(agc_vec_abval(ip, ln.rclp), &x1); /* 2^(S+X1) */
    agc_tp_t v;                                             /* TP, B-40 after the SR* */

    v = agc_dp_mul_tp(agc_dp_sq(n), agc_dp_add(ip, LN_SCTVAR, LN_IMUVARR)); /* TP, B+18 */

    x1                = 2 * x1 + 2 * ln.eorm;
    ln.inc.variance   = agc_tp_shift(ip, v, x1);         /* SR* 0,1  */
    rz_bvectors(ip, ln.rclp, ln.um, &ln.ustar, &ln.inc); /* S2BVTRS  */
    ln.inc.bvector[2] = agc_vec_neg(ln.inc.bvector[0]);  /* B2 = -B0 */
    ln.inc.dim9       = true;
    *st               = agc_incorp_state_deviation(ip,
                                                   &agc_integ_shared_ctx()->w,
                                                   &ln.inc);
    agc_phase_change(04022);

    if (*st != AGC_STATUS_OK || !agc_flag_test(AGC_FLAG_22DSPFLG))
    {
        return (false);
    }

    rz_flag_put(AGC_FLAG_22DSPFLG, false);
    agc_phase_change(04022);
    ln.n49disp[0] = rz_shift(ip, agc_vec_abval(ip, ln.inc.deltax[0]), ln.eorm);
    ln.n49disp[1] = rz_shift(ip, agc_vec_abval(ip, ln.inc.deltax[1]), ln.eorm);

    return (true);
}

/**********************************************************************
 * S22SAVET / S22I=N: TPRIM = TF; after the last mark S22F244, else the
 * next mark (S2INTS1).
 * AGC: S22SAVET, S22I=N (P20-P25.agc p.605)
 *
 * FUNCTION NAME: ln_next
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the next job
 **********************************************************************/
static agc_status_t ln_next(void)
{
    ln.tprim = ln_gettf();                      /* S22SAVET */
    rz_grp2pc(AGC_JOB_ORBITAL_NAVIGATION_SITE); /* the next GRP2PC renames it when a mark follows */

    if (ln.i >= ln.n)
    {
        return (rz_continue(AGC_JOB_ORBITAL_NAVIGATION_SITE)); /* S22F244X */
    }

    ln.i++;
    ln.loc   += LN_MARK_WORDS;
    ln.first = false;
    rz_grp2pc(AGC_JOB_ORBITAL_NAVIGATION_MARK); /* S2INTS1: the next mark */

    return (rz_continue(AGC_JOB_ORBITAL_NAVIGATION_MARK));
}

/**********************************************************************
 * S22BOX12 .. S22BOX42: incorporate the mark twice (the second pass with
 * the updated CSM and landmark); the first incorporation of a sighting
 * may stop for the noun 49 display.
 * AGC: S22BOX12 .. S22BOX42 (P20-P25.agc pp.604-605)
 *
 * FUNCTION NAME: ln_incorporate
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *   ip        agc_interp_t *   I/O  interpreter state (OVFIND)
 *   at_box42  bool             I    true: start at S22BOX42 (after a noun
 *                                   49 PROCEED)
 *
 * RETURNS: agc_status_t -- the status of the next step, the display, or
 *          the step that failed; AGC_STATUS_NO_CONVERGE when its step
 *          bound is exhausted
 **********************************************************************/
static agc_status_t ln_incorporate(agc_job_ctx_t *ctx, agc_interp_t *ip, bool at_box42)
{
    for (int pass = 0; pass < 3; pass++)
    {
        agc_status_t st = AGC_STATUS_OK; /* status of the last step */

        if (!at_box42 && ln_variance_and_incorporate(ip, &st))
        {
            return (rz_flash(ctx, AGC_DSKY_GOFLASHR, 6U, 49U, AGC_JOB_GOTOPOOH, AGC_JOB_ORBITAL_NAVIGATION_DELTA_ACCEPT, AGC_JOB_ORBITAL_NAVIGATION_MARKS_EXIT));
        }

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        at_box42 = false;
        st       = rz_incorp2(ip, &ln.inc, agc_flag_test(AGC_FLAG_CMOONFLG)); /* S22BOX42 */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        ln_calrc(ip);

        if (!agc_flag_test(AGC_FLAG_INCORFLG))
        { /* DMPINCP2 */
            return (ln_next());
        }

        agc_phase_change(04022);
        rz_flag_put(AGC_FLAG_INCORFLG, false);
        ln.rclp = agc_vec_sub(ip, agc_erase_x789(), ln.csmpos);
    }

    return (AGC_STATUS_NO_CONVERGE); /* step bound exhausted */
} /* ln_incorporate */

/**********************************************************************
 * DMPINTEG (p.600): the offset point mark is kept (S22OFF=I); with the
 * landmark rows in W the landmark is moved to this mark time (R-TO-RP,
 * RP-TO-R) and the mark incorporated (S22D=9); on the first other mark
 * the landmark is established.
 * AGC: DMPINTEG, S22OFF=I, S22D=9 (P20-P25.agc p.600)
 *
 * FUNCTION NAME: ln_dmpinteg
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *   ip        agc_interp_t *   I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- the status of the step
 **********************************************************************/
static agc_status_t ln_dmpinteg(agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    if (ln.i == (agc_sp_t) (ln.cxoff >> 9))
    { /* S22OFF=I */
        ln.toff = ln_gettf();
        ln.uoff = ln.um;

        return (ln_next());
    }

    agc_vec_t    rl = agc_erase_x789(); /* RL, body scale */
    agc_status_t st = AGC_STATUS_OK;    /* status of the last step */

    if (agc_flag_test(AGC_FLAG_DMENFLG))
    { /* S22D=9   */
        agc_navaux_padload_t pad;                   /* pad loads */
        agc_vec_t            rp;                    /* RP, planet fixed, body scale */
        bool                 moon = (ln.eorm != 0); /* moon sphere */

        agc_navaux_get_padload(&pad);
        st = agc_planet_r_to_rp(ip, &pad, moon, rl, ln.tprim, &rp);

        if (st == AGC_STATUS_OK)
        {
            st = agc_planet_rp_to_r(ip, &pad, moon, rp, ln_gettf(), &rl);
        }
    }
    else
    {
        agc_phase_change(04022);
        rz_flag_put(AGC_FLAG_ORBWFLAG, true);
        rz_flag_put(AGC_FLAG_DMENFLG, true);
        rz_flag_put(AGC_FLAG_22DSPFLG, true);
        rz_flag_put(AGC_FLAG_ERADFLAG, true);

        if (!agc_flag_test(AGC_FLAG_KNOWNFLG))
        {
            ln_unknown_landmark(ip);

            return (ln_next());
        }

        /* S22BOX22, a known landmark: the landmark rows of W from S22WSUBL; RL
         * from LAT LONG ALT at TF (LALOTORV, SL2 in the moon's sphere).
         * AGC: S22BOX22, S22BX22B (P20-P25.agc p.604) */
        agc_incorp_w_t *w = &agc_integ_shared_ctx()->w; /* W matrix */

        for (int m = 0; m < LN_ROWS; m++)
        { /* CLRW678  */
            w->blk[2][m] = agc_vec_zero();
        }

        agc_dp_t wl; /* S22WSUBL, pad scale */

        wl = agc_dp_from_sp(agc_rendz_nav_pad()->s22wsubl);

        *ln_w(144)         = wl;
        *ln_w(152)         = wl;
        *ln_w(160)         = wl;
        bool moon_landmark = agc_flag_test(AGC_FLAG_CMOONFLG); /* moon sphere */

        rz_flag_put(AGC_FLAG_LUNAFLAG, moon_landmark);
        agc_navaux_padload_t pad_landmark; /* pad loads */
        agc_latlong_t        ll;           /* LAT LONG ALT */

        agc_navaux_get_padload(&pad_landmark);
        agc_navaux_get_latlong(&ll);
        agc_latlong_opt_t opt         = {moon_landmark, agc_flag_test(AGC_FLAG_ERADFLAG)};
        agc_status_t      st_landmark = agc_latlong_to_r(ip, &pad_landmark, opt, &ll, ln_gettf(), &rl, NULL); /* LALOTORV */

        if (moon_landmark)
        {
            rl = agc_vec_shl(ip, rl, 2); /* S22BX22B: SL2 */
        }

        st = st_landmark;
    } /* end else */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    /* S22BOX32: X789 = RL; first pass; RCLP = RL - RC; USTAR =
     * UNIT(UNIT(RCLP) x UM). False on an overflow (the mark is skipped).
     * AGC: S22BOX32 (P20-P25.agc p.605) */
    agc_erase_set_x789(rl);
    rz_flag_put(AGC_FLAG_INCORFLG, true);
    agc_interp_clear_ovf(ip);
    ln.rclp     = agc_vec_sub(ip, rl, ln.csmpos);
    agc_vec_t u = agc_vec_unit(ip, ln.rclp, NULL, NULL); /* 2^1 */

    ln.ustar = agc_vec_unit(ip, agc_vec_cross(ip, u, ln.um), NULL, NULL);

    if (!(!agc_interp_test_ovf(ip)))
    {
        return (ln_next()); /* S22SAVET */
    }

    return (ln_incorporate(ctx, ip, false));
} /* ln_dmpinteg */

/**********************************************************************
 * FIG2EXIT / S2INTS1 / S22NXTIN: INTSTALL, flags, the CSM (with W) to the
 * mark time, RC, UM; then DMPINTEG.
 * AGC: FIG2EXIT, S2INTS1, S22NXTIN (P20-P25.agc p.599)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_mark
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_mark(agc_job_ctx_t *ctx)
{
    agc_status_t st; /* INTSTALL */

    st = agc_integ_install(0, AGC_JOB_ORBITAL_NAVIGATION_MARK);

    if (st != AGC_STATUS_OK)
    {
        return (st); /* BUSY: asleep in INTSTALL  */
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    ln_flags();
    ln_w_setup();
    st = agc_integ_integrv(&ip, agc_integ_shared_ctx(), ln_gettf()); /* S22NXTIN */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ln_calrc(&ip);

    ln_getum(&ip); /* S2GETUM  */

    return (ln_dmpinteg(ctx, &ip));
}

/**********************************************************************
 * PROCEED on V06N49: incorporate (S22BOX42) and go on.
 * AGC: S22BOX42 (P20-P25.agc p.605)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_delta_accept
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_delta_accept(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);

    return (ln_incorporate(ctx, &ip, true));
}

/**********************************************************************
 * S22EXEX (RECYCLE on V06N49): back to P22 without converting W.
 * AGC: S22EXEX (P20-P25.agc p.605)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_marks_exit
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_marks_exit(void)
{
    return (rz_continue(ln.ret));
}

/**********************************************************************
 * CXOFF: the offset point mark (B of the landmark code, bits 10-12).
 * AGC: CXOFF (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: rz_landmark_set_offset
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   cxoff     agc_sp_t  I    B of ABCDE, in place (0 .. 05000 octal)
 **********************************************************************/
void rz_landmark_set_offset(agc_sp_t cxoff)
{
    ln.cxoff = cxoff;
}

/* ---- the landmark at the end (S22F244 .. S22.981X pp.607-608) ---------- */
/**********************************************************************
 * S22F244: with an offset point mark (OFF within the marks), the CSM at
 * its time (CSMPREC) and RL along its UM (formula 2.4.10 with R0 =
 * |X789|); LAT LONG ALT of RL at TOFF (else now) for noun 89 (LLASRD).
 * AGC: S22F244, S22BOX44, S22BX44A (P20-P25.agc pp.607-608)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_site
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_site(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    bool     moon = agc_flag_test(AGC_FLAG_CMOONFLG); /* moon sphere */
    agc_sp_t off;                                     /* offset mark number, unscaled */

    off             = (agc_sp_t) (ln.cxoff >> 9);
    agc_status_t st = AGC_STATUS_OK; /* status of the last step */

    if (off != 0 && ln.n - off >= 0)
    {
        agc_phase_change(04022);
        agc_integ_out_t o; /* integration result */

        st = rz_integ(&ip, RZ_CSMPREC, ln.toff, AGC_JOB_ORBITAL_NAVIGATION_SITE, &o);

        if (st != AGC_STATUS_OK)
        {
            return (st); /* BUSY: asleep in INTSTALL  */
        }

        ln.csmpos = o.ratt1;
        ln.um     = ln.uoff;
        agc_dp_t r0; /* |X789|, body scale */

        r0 = agc_vec_abval(&ip, agc_erase_x789());

        if (moon)
        {
            r0 = agc_dp_shr(r0, 2); /* to B-29 */
        }

        ln_landmark_on_surface(&ip, r0);
    }
    else
    {
        ln.toff = ln_gettf(); /* S22BOX44 */
    }

    rz_flag_put(AGC_FLAG_LUNAFLAG, moon); /* S22BX44A */
    agc_vec_t alphav = agc_erase_x789();  /* RL, body scale */

    if (moon)
    {
        alphav = agc_vec_shr(alphav, 2); /* to m 2^29 */
    }

    agc_navaux_padload_t pad; /* pad loads */
    agc_latlong_t        ll;  /* LAT LONG ALT */
    agc_latlong_opt_t    opt = {moon, agc_flag_test(AGC_FLAG_ERADFLAG)};

    agc_navaux_get_padload(&pad);
    st = agc_latlong_from_r(&ip, &pad, opt, alphav, ln.toff, &ll);

    if (st == AGC_STATUS_OK)
    {
        agc_navaux_set_latlong(&ll);
        agc_rendz_llasrd();
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (rz_flash(ctx, AGC_DSKY_GOFLASH, 6U, 89U, AGC_JOB_ORBITAL_NAVIGATION_SITE_TERMINATE, AGC_JOB_ORBITAL_NAVIGATION_SITE_ACCEPT, AGC_JOB_ORBITAL_NAVIGATION_SITE_DONE));
} /* agc_job_orbital_navigation_site */

/**********************************************************************
 * PROCEED on V06N89: the landing site RLS = R-TO-RP(RL, TOFF) (moon
 * fixed), then S22.981X.
 * AGC: S22F244 PROCEED (P20-P25.agc p.608)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_site_accept
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_site_accept(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_navaux_padload_t pad; /* pad loads */

    agc_navaux_get_padload(&pad);
    agc_status_t st = agc_planet_r_to_rp(&ip, &pad, true, agc_erase_x789(), ln.toff, &pad.rls); /* R-TO-RP */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_navaux_set_padload(&pad);

    return (agc_job_orbital_navigation_site_done());
}

/**********************************************************************
 * S22.981X (also RECYCLE): W back to 6x6; return to P22 (point A).
 * AGC: S22.981X (P20-P25.agc p.608)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_site_done
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_site_done(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    ln_reduce_w_to_six(&ip);

    return (rz_continue(ln.ret));
}

/**********************************************************************
 * S22GTP (TERMINATE on V06N89): W back to 6x6, then P00.
 * AGC: S22GTP (P20-P25.agc p.608)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_site_terminate
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_site_terminate(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    ln_reduce_w_to_six(&ip);

    return (agc_exec_gotopooh());
}
