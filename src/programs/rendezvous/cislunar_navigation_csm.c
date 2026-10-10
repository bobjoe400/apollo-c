/*
 * cislunar_navigation_csm.c -- P23 CISLUNAR MIDCOURSE NAVIGATION and R57
 * OPTICS CALIBRATION ROUTINE, CSM (P20-P25.agc, Comanche055 pp.570-571,
 * 619-631): P23, P23.00 .. P23.85, V94ENTER, POINTAXS (R23.05 .. R23.5),
 * R23.55 .. R23.END, R23.10, HORIZ, DIVIDE, RECT.1, LDPLANET, R57.
 *
 * "Do midcourse navigation by incorporation of star/earth and star/moon
 * optical measurements." The crew names a star and an earth or moon
 * landmark or horizon (noun 70); the optics measure the angle between
 * them; the measured minus computed angle, corrected for aberration of
 * light, updates the CSM state. R57 "measures the effect of solar
 * radiation on the SXT trunnion angle and stores the measured trunnion
 * bias for P23".
 *
 * Scales: RZC, RCLL m 2^29 (earth centred at the measured body), VZC m/cs
 * 2^7, unit vectors 2^1, angles revolutions, DELTAQ m 2^29 (2^27 moon).
 */
#include <stddef.h>
#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/imu/imu.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"
#include "agc/programs/alignment_pending.h"
#include "agc/io/service_internal.h"
#include "internal.h"

#define CN_ONE_C     ((agc_dp_t)187779866)  /* ONE/C 2DEC* .333564049 E-6 B+21* */
#define CN_AEARTH    ((agc_dp_t)3189083)    /* AEARTH 2DEC 6378166 B-29     */
#define CN_RADMOON   ((agc_dp_t)869045)     /* RADMOON 2DEC 1738090 B-29    */
#define CN_TRUN19    ((agc_dp_t)(01604 * 16384)) /* TRUN19 OCT 01604, 00000  */
#define CN_1_0B18    ((agc_dp_t)1024)       /* 1.0B18 2DEC 1.0 B-18         */
#define CN_VARSUBL3  ((agc_dp_t)13719616)   /* VARSUBL3 2DEC* 3.4299040 E+6 B-26* */
#define CN_TRUNVAR   ((agc_dp_t)175922)     /* TRUNVAR 2DEC 2.5 E-9 B+18    */
#define CN_PI_4      ((agc_dp_t)210828714)  /* PI/4.0                       */
#define CN_JC_X      ((agc_dp_t)72160567)   /* JCAXIS 1/2 sin 32.523 deg    */
#define CN_JC_Z      ((agc_dp_t)113169126)  /* JCAXIS 1/2 cos 32.523 deg    */
#define CN_1_SQR3    ((agc_dp_t)154981267)  /* 1/SQR3 2DEC .57735021        */
#define CN_19_77DEG  061740u                /* 19.77DEG OCT 61740           */
#define CN_V50N25P   0202U                  /* V50N25P OCT 00202 (GOPERF1)  */
#define CN_BITS7_9   0700u
#define CN_BITS4_6   0070u
#define CN_PRIO14    ((agc_prio_t)014)
#define CN_ALARM_BUSY ((agc_alarm_code_t)01211)  /* marking system busy    */

/* P23 erasables.                                                         */
typedef struct cn_state {
    agc_sp_t  trunbias;      /* TRUNBIAS: R57 trunnion bias, CDU units    */
    uint16_t  trunion;       /* TRUNION: trunnion CDU of the mark         */
    agc_dp_t  marktime;      /* MARKTIME                                  */
    agc_vec_t rzc, vzc;      /* RZC m 2^29, VZC m/cs 2^7                  */
    agc_mat_t ubar;          /* UBAR0 UBAR1 UBAR2: horizon frame          */
    agc_vec_t rcll;          /* RCLL: CSM to landmark/horizon, m 2^29     */
    agc_dp_t  n49disp[2];    /* N49DISP                                   */
    agc_incorp_t inc;
    agc_job_id_t r57_ret;    /* EGRESS of R57                             */
} cn_state_t;

static cn_state_t cn;

agc_sp_t agc_erase_trunbias(void)
{
    (void)AGC_ASSERT(cn.trunbias >= AGC_SP_MIN);
    return cn.trunbias;
}

/* ---- R57 (pp.570-571) -------------------------------------------------- */
/* R57: abort (BAILOUT 1211) when the marking system or an extended verb is
 * in use; else marking system in use, V59 (calibration mark, GOMARKFR).
 * The caller continues as ret after ENDR57.                             */
agc_status_t rz_optics_calibration(const agc_job_ctx_t *ctx, agc_job_id_t ret)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(ret != AGC_JOB_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    cn.r57_ret = ret;
    uint16_t x = agc_erase_extvbact();
    if ((x & 6U) != 0U) {                                     /* P23ABRT  */
        agc_status_t st = agc_bailout(CN_ALARM_BUSY);
        return (st == AGC_STATUS_OK) ? AGC_STATUS_ALARM : st;
    }
    agc_erase_set_extvbact((uint16_t)(x | 2U));
    return rz_continue(AGC_JOB_OPTICS_CALIBRATION);           /* R57A     */
}

/* R57A: V59FLAG (calibration mark); V59 (GOMARKFR, R1-R3 blanked):
 * TERMINATE P00, PROCEED or ENTER end R57.                               */
agc_status_t agc_job_optics_calibration(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rz_flag_put(AGC_FLAG_CALIBRATION_MARKING, true);          /* V59FLAG  */
    agc_status_t st = rz_flash(ctx, AGC_DSKY_GOMARKFR, 59U, 0U, AGC_JOB_GOTOPOOH,
                               AGC_JOB_OPTICS_CALIBRATION_END,
                               AGC_JOB_OPTICS_CALIBRATION_END);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_dsky_blanket(7U);
}

/* MARKDISP: after a calibration mark, V06N87 (the trunnion) GOMARKFR:
 * PROCEED keeps the bias (R57B), ENTER marks again (R57A).               */
agc_status_t agc_job_optics_calibration_mark_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = rz_flash(ctx, AGC_DSKY_GOMARKFR, 6U, 87U, AGC_JOB_GOTOPOOH,
                               AGC_JOB_OPTICS_CALIBRATION_BIAS,
                               AGC_JOB_OPTICS_CALIBRATION);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_dsky_blanket(5U);
}

/* R57B: TRUNBIAS = the marked trunnion (MRKBUF1 +5) MSU 19.77 degrees
 * (two's to ones' complement about the trunnion zero).                  */
agc_status_t agc_job_optics_calibration_bias(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    int d = ((int)agc_erase_mrkbuf(1, 5) - (int)CN_19_77DEG) & 077777;
    d = ((d & 040000) != 0) ? d - 0100000 : d;
    cn.trunbias = (agc_sp_t)((d < AGC_SP_MIN) ? AGC_SP_MIN : d);
    return agc_job_optics_calibration_end(ctx);
}

/* ENDR57: V59FLAG off; ENDMARK as a PRIO14 job (so that R53 can take
 * over the marking); back to the caller.                                 */
agc_status_t agc_job_optics_calibration_end(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rz_flag_put(AGC_FLAG_CALIBRATION_MARKING, false);
    agc_status_t st = agc_exec_novac(CN_PRIO14, AGC_JOB_OPTICS_CALIBRATION_END_MARK);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return rz_continue(cn.r57_ret);
}

agc_status_t agc_job_optics_calibration_end_mark(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_dsky_endmark();
}

/* ---- RECT.1 (p.630): the CSM state from the permanent conic + deviation
 * at 2^29 / 2^7 (SR2 from the moon scale); ZMEASURE when the measured
 * body is not the primary body.                                          */
static agc_status_t cn_rect(agc_interp_t *ip)
{
    bool moon = agc_flag_test(AGC_FLAG_CMOONFLG);
    int x2 = moon ? -2 : 0;
    rz_flag_put(AGC_FLAG_ZMEASURE, moon != agc_flag_test(AGC_FLAG_LUNAFLAG));
    agc_integ_sv_t sv;
    agc_status_t st = agc_integ_perm_get(AGC_VEHICLE_CSM, &sv);
    agc_vec_t r = agc_vec_add(ip, rz_vshift(ip, agc_vec_shr(sv.deltav, 7), x2), sv.rcv);
    cn.rzc = rz_vshift(ip, r, x2);                            /* B29      */
    agc_vec_t v = agc_vec_add(ip, rz_vshift(ip, agc_vec_shr(sv.nuv, 4), x2), sv.vcv);
    cn.vzc = rz_vshift(ip, v, x2);                            /* B7       */
    return st;
}
