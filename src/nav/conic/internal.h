/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface shared by the src/nav/conic C files: constants,
 *   the conic context and the interpretive-MPAC helpers. agc_conic_ctx_t
 *   is the C image of the erasables and push-down cells that the conic
 *   subroutines share. In the AGC several of them alias one another
 *   (COGAMAX = 1/MU = 14D, COGAMIN = ALPHA = 8D, DCOGA = XMIN = 12D,
 *   COSF = XI = 24D, R2 = MAGVEC2, ...). The aliases that carry data from
 *   one routine to the next are kept as one field; the rest get separate
 *   fields because no AGC path reads one name after writing the other.
 *
 *   The interpretive MPAC is modelled as an agc_tp_t (three words): DMP and
 *   DSQ leave a TP product, DAD/DSU/BDSU change only MPAC,+1 and keep
 *   MPAC+2, shifts, NORM, TAD and ROUND work on all three words, and a
 *   DP-mode STORE or PUSH drops MPAC+2. The agc_conic_m*_ helpers are one
 *   function per interpretive instruction on that model, so the C reads in
 *   the same order as the AGC listing. Names ending in '_' are
 *   module-internal (not part of conic.h).
 *
 * AGC SOURCE:
 *   Comanche055 CONIC_SUBROUTINES.agc pp. 1288 (constants), 1305-1308
 *   (MUTABLE, erasable assignments)
 *   Luminary099 CONIC_SUBROUTINES.agc pp. 1184, 1201-1204 (same words)
 *
 * REQUIRES:
 *   none (self-contained; includes what it uses)
 *
 * GLOBAL VARIABLES:
 *   none (struct members carry their scale in a short comment)
 *********************************************************************/
#ifndef AGC_CONIC_INTERNAL_H
#define AGC_CONIC_INTERNAL_H

#include <stdbool.h>
#include <stddef.h>

#include "agc/core/interp.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/nav/conic.h"

/* ---- constants, CONIC_SUBROUTINES.agc p. 1288 (assembled octal) ------- */
#define AGC_CONIC_D1_8    ((agc_dp_t) 33554432) /* D1/8   04000 00000 1.0 B-3 */
#define AGC_CONIC_D1_128  ((agc_dp_t) 2097152)  /* D1/128 00200 00000 1.0 B-7 */
#define AGC_CONIC_D1_64   ((agc_dp_t) 4194304)  /* D1/64  00400 00000 1.0 B-6 */
#define AGC_CONIC_D1_4    ((agc_dp_t) 67108864) /* D1/4   10000 00000 1.0 B-2 */
#define AGC_CONIC_D1_16   ((agc_dp_t) 16777216) /* D1/16  02000 00000 1.0 B-4 */
#define AGC_CONIC_D1_32   ((agc_dp_t) 8388608)  /* D1/32  01000 00000 1.0 B-5 */
#define AGC_CONIC_D1_1024 ((agc_dp_t) 262144) /* D1/1024 00020 00000 1.0 B-10 */
#define AGC_CONIC_D1_256  ((agc_dp_t) 1048576) /* D1/256 00100 00000 1.0 B-8 */
#define AGC_CONIC_DP9_10  ((agc_dp_t) 241591910) /* DP9/10 34631 23146 .9 */
#define AGC_CONIC_M50SC   ((agc_dp_t) -3276800) /* -50SC 77467 77777 B-12 */
#define AGC_CONIC_2PISC \
    ((agc_dp_t) 26353589) /* 2PISC  03110 17665 6.2831853 B-6 */
#define AGC_CONIC_BEE17 \
    ((agc_dp_t) 2048) /* BEE17  00000 04000 1.0 B-17 (overlay) */
#define AGC_CONIC_BEE19 \
    ((agc_dp_t) 512) /* BEE19  00000 01000 1.0 B-19 (overlay) */
#define AGC_CONIC_BEE22 \
    ((agc_dp_t) 64) /* BEE22  00000 00100 1.0 B-22 (overlay) */
#define AGC_CONIC_ONEBIT ((agc_dp_t) 1) /* ONEBIT 00000 00001 1.0 B-28 */
#define AGC_CONIC_COGUPLIM \
    ((agc_dp_t) 268304351) /* COGUPLIM 37767 37737 .999511597 */
#define AGC_CONIC_COGLOLIM \
    ((agc_dp_t) -268304351) /* COGLOLIM 40010 40040 -.999511597 */
#define AGC_CONIC_DP1_4 \
    ((agc_dp_t) 67108864) /* DP1/4 (fixed-fixed pool) .25   */
#define AGC_CONIC_LODPHALF \
    ((agc_dp_t) 134217728) /* LODPHALF = XUNIT 20000 00000 .5 */
#define AGC_CONIC_LDPOSMAX \
    ((agc_dp_t) 268435455) /* LDPOSMAX = LODPMAX 37777 37777 */
#define AGC_CONIC_DP2_3 \
    ((agc_dp_t) 178956971)               /* DP2/3 25252 25253 .6666666667 */
#define AGC_CONIC_KEPZERO ((agc_dp_t) 0) /* KEPZERO = LO6ZEROS         */
/* SSP constants: YUL assembled an interpretive octal operand of 10000 or
 * more as the operand less 10000 (an address-constant encoding): the
 * ropes hold 27777 for "SSP GEOMSGN 37777" (Comanche 04,3646, Luminary
 * 04,3073) and 10000 for "SSP TWEEKIT 20000" (Comanche 12,3501, Luminary
 * 12,3471); both ropes match the binsource transcriptions of the flight
 * listings. The assembled words are what the flight code used.           */
#define AGC_CONIC_GEOMSGN_POS   ((agc_sp_t) 027777) /* .75, positive     */
#define AGC_CONIC_TWEEKIT_GUESS ((agc_sp_t) 1)      /* SSP TWEEKIT 00001 */
#define AGC_CONIC_TWEEKIT_NOGUESS \
    ((agc_sp_t) 010000) /* SSP TWEEKIT "20000": .25 */

#define AGC_CONIC_GETX_PASSES 3 /* TIX,2 with X2 = 3, S2 = 1: (1/)WLOOP */
/* POLY coefficient counts: "DEC N" in the listing is the count minus 2.  */
#define AGC_CONIC_DELTIME_NCOEF  10 /* DEC 8 */
#define AGC_CONIC_POLYCOEF_NCOEF 7  /* DEC 5 */

/* Bound on LAMBLOOP passes. Passes through NEGP/HIENERGY/LOENERGY do not
 * decrement ITERCTR; each halves DCOGA, which reaches zero within 28
 * halvings, so (ITERCTR max 20 + 1) * 30 passes cover every AGC path.   */
#define AGC_CONIC_LAMBERT_MAX_PASSES 630 /* 21 x 30 LAMBLOOP passes */
/* PERIODCH: TAU. is reduced by one period per pass; the AGC loop has no
 * bound. 4096 passes cover |TAU.| = 2^28 cs (31 days) for any period of
 * at least 2^16 cs (11 min), far shorter than any real orbit. DEVIATION.  */
#define AGC_CONIC_PERIOD_MAX_PASSES 4096 /* PERIODCH passes */

/* Output switches mirrored into the flagwords. */
typedef enum agc_conic_flag
{
    AGC_CONIC_FLAG_SOLNSW = 0,
    AGC_CONIC_FLAG_INFINFLG,
    AGC_CONIC_FLAG_COGAFLAG,
    AGC_CONIC_FLAG_APSESW
} agc_conic_flag_t;

typedef struct agc_conic_ctx
{
    agc_conic_body_t body;        /* central body (X1)                 */
    agc_dp_t         inv_mu;      /* 1/MU (14D), 2^-34 / 2^-28         */
    agc_dp_t         root_mu;     /* ROOTMU (16D), 2^18 / 2^15         */
    agc_dp_t         inv_root_mu; /* 1/ROOTMU (18D), 2^-17 / 2^-14     */
    agc_dp_t         r1;      /* R1 (32D), 2^29 / 2^27 m           */
    agc_vec_t        ur1;     /* UR1 (KEPLER: URRECT), 2^1         */
    agc_vec_t        u2;      /* U2, 2^1                           */
    agc_vec_t        un;      /* UN, 2^1                           */
    agc_vec_t        vvec;    /* VVEC (KEPLER: VRECT), 2^7 / 2^5   */
    agc_dp_t         magvec2; /* MAGVEC2 = R2, scale of V2VEC      */
    agc_dp_t         snth;    /* SNTH, 2^1                         */
    agc_dp_t         csth;    /* CSTH, 2^1                         */
    agc_dp_t         p;       /* P, 2^4                            */
    agc_dp_t         r1a;     /* R1A, 2^6                          */
    agc_dp_t         coga;    /* COGA, 2^5                         */
    agc_dp_t         x;       /* X (20D), 2^17 / 2^16 sqrt(m)      */
    agc_dp_t         xi;      /* XI (24D), 2^6                     */
    agc_dp_t         s_xi;    /* S(XI) (26D), 2^1                  */
    agc_dp_t         xsqc_xi; /* XSQC(XI) (28D), 2^33 / 2^31       */
    agc_dp_t         t;       /* T (30D), 2^28 cs                  */
    agc_dp_t         kepc1;   /* KEPC1 (34D), 2^17 / 2^16          */
    agc_dp_t         kepc2;   /* KEPC2 (36D), 2^6                  */
    agc_dp_t         xsq;     /* 0D on entry to DELTIME: X^2
                               * normalized, 2^(34-N1) / 2^(32-N1) */
    int      xsq_n1;          /* N1: that NORM leaves X1 = -N1     */
    agc_sp_t geomsgn;         /* GEOMSGN, SP 2^0 (+-.5)            */
    bool     normsw;          /* NORMSW                            */
    bool     sw360;           /* 360SW                             */
    bool     infinflg;        /* INFINFLG                          */
    bool     cogaflag;        /* COGAFLAG                          */
} agc_conic_ctx_t;

/* ---- common.c: MUTABLE and switches ------------------------------ */
agc_status_t agc_conic_ctx_init_(agc_conic_ctx_t *ctx, agc_conic_body_t body);
agc_dp_t     agc_conic_mu_(agc_conic_body_t body); /* MUEARTH / MUM */
void         agc_conic_mirror_flag_(agc_conic_flag_t f, bool set);
agc_status_t agc_conic_abtconic_(void); /* ABTCONIC     */

agc_tp_t agc_conic_mdad_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d);  /* DAD  */
agc_tp_t agc_conic_mdsu_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d);  /* DSU  */
agc_tp_t agc_conic_mbdsu_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d); /* BDSU */
agc_tp_t agc_conic_mdmp_(agc_tp_t m, agc_dp_t d);                    /* DMP  */
agc_tp_t agc_conic_mdmpr_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d); /* DMPR */
agc_tp_t agc_conic_mdsq_(agc_tp_t m);                                /* DSQ  */
agc_tp_t agc_conic_mddv_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d);  /* DDV  */
agc_tp_t agc_conic_mbddv_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d); /* BDDV */
agc_tp_t agc_conic_msqrt_(agc_interp_t *ip, agc_tp_t m);             /* SQRT */
agc_tp_t agc_conic_mshiftr_(agc_interp_t *ip, agc_tp_t m,
                            int n);                        /* SLR n / SRR -n */
agc_tp_t agc_conic_mround_(agc_interp_t *ip, agc_tp_t m); /* ROUND */
agc_tp_t agc_conic_mnorm_(agc_tp_t m, int *nshift);    /* NORM: X = -nshift */
agc_tp_t agc_conic_mabs_(agc_tp_t m);                  /* ABS  */
agc_tp_t agc_conic_msign_(agc_tp_t m, agc_dp_t s);     /* SIGN, DP word */
agc_tp_t agc_conic_msign_sp_(agc_tp_t m, agc_sp_t s);  /* SIGN, SP tag  */
bool     agc_conic_mbmn_(agc_tp_t m);                  /* BMN taken     */
bool     agc_conic_mbze_(agc_tp_t m);                  /* BZE taken     */
agc_dp_t agc_conic_poly_(agc_dp_t x, const agc_dp_t *coef,
                         int ncoef);  /* POLY */

/* ---- universal.c: DELTIME, NEWSTATE, LAMENTER --------------------- */
agc_status_t agc_conic_deltime_(agc_interp_t *ip, agc_conic_ctx_t *ctx);
agc_status_t agc_conic_newstate_(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                 agc_vec_t *r2vec, agc_vec_t *v2vec);
agc_status_t agc_conic_lamenter_(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                 agc_dp_t r2mag, agc_vec_t *v2vec);
agc_status_t agc_conic_kepconvg_(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                 agc_vec_t *rcv, agc_vec_t *vcv);

/* ---- geom.c: GEOM, PARAM, GETX ------------------------------------ */
agc_status_t agc_conic_geom_(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                             agc_vec_t r1vec, agc_vec_t v2vec, agc_dp_t *snth,
                             agc_dp_t *csth);
agc_status_t agc_conic_param_(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                              agc_vec_t rvec, agc_vec_t vvec,
                              agc_dp_t *sngamma);
agc_status_t agc_conic_getx_(agc_interp_t *ip, agc_conic_ctx_t *ctx);

/* ---- timetheta.c: COMMNOUT (shared with TIMERAD) ------------------ */
agc_status_t agc_conic_commnout_(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                 bool rvsw, agc_conic_time_out_t *out);
agc_status_t agc_conic_cogaovfl_(agc_conic_ctx_t      *ctx,
                                 agc_conic_time_out_t *out);
void agc_conic_time_out_fill_(const agc_conic_ctx_t *ctx,
                              agc_conic_time_out_t  *out);
void agc_conic_time_out_clear_(agc_conic_time_out_t *out);

#endif /* AGC_CONIC_INTERNAL_H */
