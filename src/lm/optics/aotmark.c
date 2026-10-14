/**********************************************************************
 * FILE NAME: aotmark.c
 *
 * PURPOSE:
 *   AOTMARK, the LM alignment optical telescope mark routine.  AOTMARK
 *   (R53 in the LM) reserves a VAC area and starts GETDAT: the star code
 *   and AOT detent (V01N71), the optic axis of the detent (OANB,
 *   OPTAXIS), then the crew's X and Y reticle marks (V54/V53/V52 N71)
 *   taken by MARKRUPT, up to five pairs.  MARKCHEX / AVESTAR turn each
 *   pair into a star line of sight in SM axes and average them into
 *   STARAD +6 (also STARSAV2).  On the lunar surface (SURFFLAG) a mark is
 *   one CDU sample plus the CURSOR and SPIRAL angles read off the reticle
 *   (V06N79), turned into a line of sight by SURFSTAR.
 *
 *   MARKSTAT:
 *     bits 1-9  VAC area (DEVIATION: VAC index + 1, not its address)
 *     bit 10    X mark made         bit 11  Y mark made (both: surface mark)
 *     bit 12    marks not wanted now (set while a display is up)
 *     bit 13    a reject since the last mark
 *     bit 14    mark pair complete (surface: marking ended)
 *   The mark pairs are agc_align_ram()->aot[MARKCNTR] (6 words each).
 *
 * AGC SOURCE:
 *   Luminary099 AOTMARK.agc pp. 244-261
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   MARKSTAT, EXTVBACT  uint16_t I/O bits        mark system state
 *   AOTAZ, AOTEL  uint16_t  I    CDU words      detent pad loads
 *   CDUX..CDUZ, TIME2/1  I                      sampled at the button
 *   STARCODE, SCAXIS, UXP, UYP, XPNB, YPNB, ROTANG, CURSOR, SPIRAL, MARKCNTR, MKDEX, STARAD, STARSAV2: agc_align_ram()
 *   SURFFLAG      flag      I                   surface marks
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dsky_endmark      DSKY displays, astronaut returns (dsky.h)
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_*           alignment routines of this lane (alignment.h)
 *   agc_exec_*            executive: jobs, VAC areas, sleep / wake (exec.h)
 *   agc_waitlist_call     WAITLIST (exec.h)
 *   agc_imu_goodend       IMU mode switching, CALCGA / CALCGTA / AXISGEN, gyro torquing (imu.h)
 *   agc_imu_trg_nbsm     CDUTRIG, *NBSM* / *SMNB*, PIPASR (imu.h)
 *   agc_vec_*             vector arithmetic of the interpreter (interp.h)
 *   agc_dp_*              DP arithmetic of the interpreter (interp.h, word.h)
 *   agc_interp_init       interpreter state: OVFIND (interp.h)
 *   agc_flag_test         flag words (flags_common.h)
 *   agc_erase_*           erasable accessors (state.h)
 *   agc_chan_read         I/O channels (channels.h)
 *   agc_alarm             ALARM (exec.h)
 *   agc_bailout           BAILOUT (exec.h)
 *   agc_poodoo            POODOO (exec.h)
 *   agc_inhint            INHINT / RELINT (exec.h)
 *   agc_relint            INHINT / RELINT (exec.h)
 *   agc_lm_descbits       LM alignment and optics routines (alignment.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Alarms: AGC_ALARM_01203; ALM_BUSY 00105: marks busy; ALM_EXTVB 01211:
 *   extended verb busy; ALM_FIVE 00107: more than five mark pairs;
 *   ALM_NOT_TAKEN 00112: marks not being accepted; ALM_NO_INBITS 00113:
 *   no input bits; ALM_NO_PAIR 00111: mark not made in pairs;
 *   ALM_NO_REJECT 00115: no mark to reject; ALM_NO_VAC 01207: no VAC
 *   area; ALM_UNWANTED 00114: mark not wanted.
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 *
 * NOTES:
 *   DODAT keeps detent bits 7-9 (DEVIATION D1); the field rotation uses
 *   MSU (finding A12).
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp_words.h"
#include "agc/io/service.h"
#include "agc/lm/landing.h"
#include "agc/programs/alignment.h"

static agc_status_t agc_job_aot_mark_surface_display(agc_job_ctx_t *ctx);

#define MS_VAC         ((uint16_t) 0777)          /* LOW9                            */
#define MS_X           ((uint16_t) 01000)         /* BIT10                           */
#define MS_Y           ((uint16_t) 02000)         /* BIT11                           */
#define MS_XY          ((uint16_t) 03000)         /* PRIO3                           */
#define MS_NOT_WANTED  ((uint16_t) 04000)         /* BIT12                           */
#define MS_REJECTED    ((uint16_t) 010000)        /* BIT13                           */
#define MS_PAIR        ((uint16_t) 020000)        /* BIT14                           */
#define MS_REJECT_CLR  ((uint16_t) 030000)        /* PRIO30: bits 13, 14             */
#define MS_PAIR_CLR    ((uint16_t) 023000)        /* PRIO23: bits 10, 11, 14         */
#define EXTVB_BUSY     ((uint16_t) 06)            /* SIX                             */
#define EXTVB_MARKING  ((uint16_t) 02)            /* BIT2                            */
#define MARK_KEYS      ((uint16_t) 034)           /* OCT34: X, Y mark, mark reject   */
#define DESCENT_KEYS   ((uint16_t) 0140)          /* OCT140: rate of descent switch  */
#define MAX_PAIR       ((agc_sp_t) 4)             /* FOUR: MARKCNTR 0..4             */
#define DETENT_SHIFT   6                          /* MP BIT9                         */
#define DETENT_MASK    ((agc_sp_t) 07)            /* three detent bits */
#define DETENT_COAS    ((agc_sp_t) 7)             /* code 7: COAS */
#define ENDMARKS_DT    ((uint16_t) 5)             /* FIVE cs                         */
#define VN_STAR_CODE   ALIGN_VN(1, 71)            /* V01N71 VN 171                   */
#define VN_AZ_EL       ALIGN_VN(6, 87)            /* V06N87* VN 687                  */
#define VN_CURSOR      ALIGN_VN(6, 79)            /* V06N79* VN 679                  */
#define VN_MARK_XY     ALIGN_VN(54, 71)           /* MKVB54 VN 5471: X or Y mark     */
#define VN_MARK_Y      ALIGN_VN(53, 71)           /* MKVB53 VN 5371: Y mark          */
#define VN_MARK_X      ALIGN_VN(52, 71)           /* MKVB52 VN 5271: X mark          */
#define ALM_BUSY       ((agc_alarm_code_t) 00105) /* marks busy */
#define ALM_NO_VAC     ((agc_alarm_code_t) 01207) /* no VAC area */
#define ALM_EXTVB      ((agc_alarm_code_t) 01211) /* extended verb busy */
#define ALM_FIVE       ((agc_alarm_code_t) 00107) /* more than five mark pairs */
#define ALM_NO_PAIR    ((agc_alarm_code_t) 00111) /* mark not made in pairs */
#define ALM_NOT_TAKEN  ((agc_alarm_code_t) 00112) /* marks not being accepted */
#define ALM_NO_INBITS  ((agc_alarm_code_t) 00113) /* no input bits */
#define ALM_UNWANTED   ((agc_alarm_code_t) 00114) /* mark not wanted */
#define ALM_NO_REJECT  ((agc_alarm_code_t) 00115) /* no mark to reject */
#define DP_ONE_EIGHTH  ((agc_dp_t) 33554432)      /* DP1/8 2DEC .125               */
#define DP_ONE_TWELFTH ((agc_dp_t) 22369621)      /* DP1/12 = DEG30 .08333333      */
#define ABOUTONE       ((agc_dp_t) 268435453)     /* 2DEC .99999999                */
#define CDU_MOD        0100000                    /* one revolution, CDU counts */
#define CDU_HALF       040000                     /* half a revolution, CDU counts */

/* ITEMP1..ITEMP5 of MARKRUPT: the time and IMU CDUs at the button. */
typedef struct aot_sample
{
    agc_dp_t t;          /* TIME2/1 at the button, cs 2^28 */
    uint16_t cdu_yzx[3]; /* CDUY, CDUZ, CDUX words        */
} aot_sample_t;

/**********************************************************************
 * AGC: MARKTYPE (Luminary099 AOTMARK.agc p. 249)
 *
 * MARKTYPE: SURFFLAG set means surface marks (CURSOR, SPIRAL).
 *
 * FUNCTION NAME: surface_marks
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true for surface marks
 **********************************************************************/
static bool surface_marks(void) /* MARKTYPE         */
{
    return (agc_flag_test(AGC_FLAG_SURFFLAG));
}

/**********************************************************************
 * OANB (p. 253): the AOT optic axis for azimuth az and elevation el (CDU
 * words): OA = 1/2 (sin EL, cos EL sin AZ, cos EL cos AZ); the Y-plane
 * vector UYP = 1/2 (0, cos AZ, -sin AZ); UXP = UNIT(UYP x OA).  NB axes.
 *
 * FUNCTION NAME: agc_aot_oanb
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   az        uint16_t        I    AOT azimuth, CDU word (2^15 = 1 rev)
 *   el        uint16_t        I    AOT elevation, CDU word (2^15 = 1 rev)
 *   oa        agc_vec_t *     I    optic axis in NB, half-unit
 *   uxp       agc_vec_t *     O    UXP = UNIT(UYP x OA), half-unit (2^1)
 *   uyp       agc_vec_t *     O    UYP = 1/2 (0, cos AZ, -sin AZ),
 *                                  half-unit (2^1)
 *
 * RETURNS: none
 **********************************************************************/
void agc_aot_oanb(agc_interp_t *ip, uint16_t az, uint16_t el, agc_vec_t *oa, agc_vec_t *uxp, agc_vec_t *uyp)
{
    const agc_dp_t elr = agc_rtb_cdulogic(el); /* EL, revs 2^0 */
    const agc_dp_t azr = agc_rtb_cdulogic(az); /* AZ, revs 2^0 */
    const agc_dp_t ce  = agc_dp_cos(elr);      /* cos EL, scale 2^1 */
    const agc_dp_t ca  = agc_dp_cos(azr);      /* cos AZ, scale 2^1 */
    const agc_dp_t sa  = agc_dp_sin(azr);      /* sin AZ, scale 2^1 */

    /* DMP SL1: the shift acts on the truncating DMP's TP product.          */
    *oa  = agc_vec_make(agc_dp_sin(elr),
                        align_dmp_shl(ip, sa, ce, 1),
                        align_dmp_shl(ip, ca, ce, 1));
    *uyp = agc_vec_make(0, ca, agc_dp_neg(sa));
    *uxp = agc_vec_unit(ip, agc_vec_cross(ip, *uyp, *oa), NULL, NULL);
}

/**********************************************************************
 * OPTAXIS (p. 252): the X and Y reticle planes in NB axes for an apparent
 * field rotation rot (SP 1's complement, half revolutions): YPNB = 1/4 (cos
 * ROT UYP - sin ROT UXP), XPNB = 1/4 (cos ROT UXP + sin ROT UYP).
 *
 * FUNCTION NAME: agc_aot_optaxis
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   uxp       agc_vec_t       I    UXP, half-unit (2^1)
 *   uyp       agc_vec_t       I    UYP, half-unit (2^1)
 *   rot       uint16_t        I    field rotation ROT, CDU word (2^15 = 1
 *                                  rev)
 *   xpnb      agc_vec_t *     I    X reticle plane normal, NB, half-unit
 *   ypnb      agc_vec_t *     I    Y reticle plane normal, NB, half-unit
 *
 * RETURNS: none
 **********************************************************************/
void agc_aot_optaxis(agc_interp_t *ip, agc_vec_t uxp, agc_vec_t uyp, uint16_t rot, agc_vec_t *xpnb, agc_vec_t *ypnb)
{
    const agc_dp_t r = agc_dp_shr(agc_dp_pack(align_word_to_sp(rot), 0), 1); /* ROT SR1, revs 2^1 */
    const agc_dp_t s = agc_dp_sin(r);                                        /* sin r, scale 2^1 */
    const agc_dp_t c = agc_dp_cos(r);                                        /* cos r, scale 2^1 */

    *ypnb = agc_vec_sub(ip, agc_vec_scale(ip, uyp, c), agc_vec_scale(ip, uxp, s));
    *xpnb = agc_vec_add(ip, agc_vec_scale(ip, uxp, c), agc_vec_scale(ip, uyp, s));
}

/**********************************************************************
 * AVESTAR flight mark (p. 257): the Y and X reticle planes at their mark
 * attitudes in SM axes (TRG*NBSM); the star is UNIT(XPSM x YPSM).
 *
 * FUNCTION NAME: agc_aot_flight_los
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ------------------------
 *   ip        agc_interp_t *                IO   interpreter state (OVFIND,
 *                                                MPAC words)
 *   m         const agc_align_aot_pair_t *  I    the stored mark
 *   xpnb      agc_vec_t                     I    X reticle plane normal,
 *                                                NB, half-unit
 *   ypnb      agc_vec_t                     I    Y reticle plane normal,
 *                                                NB, half-unit
 *   star_sm   agc_vec_t *                   O    star line of sight in SM
 *                                                axes, half-unit (2^1)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
static agc_status_t agc_aot_flight_los(agc_interp_t *ip, const agc_align_aot_pair_t *m, agc_vec_t xpnb, agc_vec_t ypnb, agc_vec_t *star_sm)
{
    if (!AGC_ASSERT((ip != NULL) && (m != NULL)) || !AGC_ASSERT(star_sm != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_cdutrig_t t;         /* CDUTRIG sines and cosines */
    agc_vec_t     yp = ypnb; /* Y reticle plane normal at the Y mark, then SM, half-unit (2^1) */
    agc_vec_t     xp = xpnb; /* X reticle plane normal at the X mark, then SM, half-unit (2^1) */

    agc_imu_trg_nbsm(&t, m->ycdu, &yp);
    agc_imu_trg_nbsm(&t, m->xcdu, &xp);
    *star_sm = agc_vec_unit(ip, agc_vec_cross(ip, xp, yp), NULL, NULL);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * SURFSTAR (p. 254): from CURSOR (YROT) and SPIRAL (SROT), SEP = (360 +
 * SROT - YROT)/12 is the star's distance off the optic axis; the star is
 * UNIT(sin SEP (UPP x OA) + cos SEP OA) in NB (OA itself when both are
 * zero), then TRG*NBSM with the mark's CDUs.
 *
 * FUNCTION NAME: agc_aot_surfstar
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ------------------------
 *   ip        agc_interp_t *                IO   interpreter state (OVFIND,
 *                                                MPAC words)
 *   m         const agc_align_aot_pair_t *  I    the stored mark
 *   oa        agc_vec_t                     I    optic axis in NB,
 *                                                half-unit
 *   xpnb      agc_vec_t                     I    X reticle plane normal,
 *                                                NB, half-unit
 *   ypnb      agc_vec_t                     I    Y reticle plane normal,
 *                                                NB, half-unit
 *   star_sm   agc_vec_words_t *             O    star line of sight in SM
 *                                                axes as TRG*NBSM leaves
 *                                                its words, half-unit (2^1)
 *
 * RETURNS: none
 **********************************************************************/
void agc_aot_surfstar(agc_interp_t *ip, const agc_align_aot_pair_t *m, agc_vec_t oa, agc_vec_t xpnb, agc_vec_t ypnb, agc_vec_words_t *star_sm)
{
    const agc_dp_t yrot = agc_rtb_cdulogic(m->ycdu[0]); /* CURSOR, revs 2^0 */
    const agc_dp_t srot = agc_rtb_cdulogic(m->ycdu[1]); /* SPIRAL, revs 2^0 */
    agc_vec_t      v    = oa;                           /* OA (YSZERO), then the star in NB, half-unit (2^1) */

    if ((yrot != 0) || (srot != 0))
    {
        /* JUSTZY of SURFSTAR: UNIT(UPP x OA), with UPP = the reticle line at
         * angle YROT: UNIT(cos YROT YPNB - sin YROT XPNB).                       */
        const agc_vec_t s   = agc_vec_scale(ip, xpnb, agc_dp_sin(yrot));                                              /* XPNB sin YROT, scale 2^2 */
        const agc_vec_t c   = agc_vec_scale(ip, ypnb, agc_dp_cos(yrot));                                              /* YPNB cos YROT, scale 2^2 */
        const agc_vec_t upp = agc_vec_unit(ip, agc_vec_sub(ip, c, s), NULL, NULL);                                    /* UPP: the reticle line at YROT, half-unit (2^1) */
        const agc_vec_t px  = agc_vec_unit(ip, agc_vec_cross(ip, upp, oa), NULL, NULL);                               /* UNIT(UPP x OA), half-unit (2^1) */
        const agc_dp_t  sep = agc_dp_mul_trunc(agc_dp_add(ip, agc_dp_sub(ip, srot, yrot), ABOUTONE), DP_ONE_TWELFTH); /* (SPIRAL - CURSOR + ABOUTONE) / 12: separation, revs 2^0 */
        const agc_vec_t a   = agc_vec_shl(ip, agc_vec_scale(ip, px, agc_dp_sin(sep)), 1);                             /* PX sin SEP VSL1, half-unit (2^1) */
        const agc_vec_t b   = agc_vec_shl(ip, agc_vec_scale(ip, oa, agc_dp_cos(sep)), 1);                             /* OA cos SEP VSL1, half-unit (2^1) */

        v = agc_vec_add(ip, b, a);
    }

    agc_cdutrig_t t; /* CDUTRIG sines and cosines */

    v = agc_vec_unit(ip, v, NULL, NULL); /* JUSTOA   */

    for (int i = 0; i < 3; i++)
    {
        star_sm->c[i] = agc_dp_to_words(v.c[i]); /* MPACVBUF */
    }

    agc_imu_trg_nbsm_words(&t, m->xcdu, star_sm); /* STCALL 24D keeps VBUF's words */
}

/**********************************************************************
 * AVEIT (p. 258): the running average of n star vectors: AVE = (n-1)/n AVE
 * + STAR/n    (VSR3, V/SC n/8; (n/8 - 1/8)/(n/8)).
 *
 * FUNCTION NAME: agc_aot_aveit
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   n         agc_sp_t        I    shift count
 *   star      agc_vec_words_t I    star line of sight as stored at 24D
 *                                  (words), half-unit (2^1)
 *   ave       agc_vec_t *     IO   running average star vector, half-unit
 *
 * RETURNS: none
 **********************************************************************/
void agc_aot_aveit(agc_interp_t *ip, agc_sp_t n, agc_vec_words_t star, agc_vec_t *ave)
{
    const agc_dp_t n8 = (agc_dp_t) n * DP_ONE_EIGHTH; /* MKDEX / 8, scale 2^0 */
    agc_vec_t      sr3;                               /* STAR VSR3 (rounds on the stored words), scale 2^4 */

    for (int i = 0; i < 3; i++)
    {
        sr3.c[i] = agc_dp_from_words(agc_mpac_store(agc_mpac_shift_round(ip, agc_mpac_dload(star.c[i]), -3)));
    }

    const agc_vec_t part = agc_vec_div(ip, sr3, n8);                              /* STAR VSR3 / (n/8): STAR/n, half-unit (2^1) */
    const agc_dp_t  keep = agc_dp_div(ip, agc_dp_sub(ip, n8, DP_ONE_EIGHTH), n8); /* (n/8 - 1/8) / (n/8): (n-1)/n, scale 2^0 */

    *ave = agc_vec_add(ip, agc_vec_scale(ip, *ave, keep), part);
}

/**********************************************************************
 * AOTMARK (p. 244): POODOO 105 if the mark system is busy, BAILOUT 1211 if
 * an extended verb is; EXTVBACT bit 2; a VAC area for the marks (BAILOUT
 * 1207 if none); FINDVAC PRIO15 GETDAT.  Returns to the caller, which then
 * waits in AOTSTALL.
 *
 * FUNCTION NAME: agc_aotmark
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_BAILOUT after the BAILOUT; AGC_STATUS_POODOO after
 *          the POODOO
 **********************************************************************/
agc_status_t agc_aotmark(void)
{
    agc_inhint();

    if (agc_erase_markstat() != 0u)
    {
        agc_relint();
        const agc_status_t pd = agc_poodoo(ALM_BUSY); /* status of POODOO */

        return ((pd == AGC_STATUS_OK) ? AGC_STATUS_POODOO : pd); /* POODOO done: report it */
    }

    if ((agc_erase_extvbact() & EXTVB_BUSY) != 0u)
    {
        agc_relint();
        const agc_status_t bo = agc_bailout(ALM_EXTVB); /* MKABORT  */

        return ((bo == AGC_STATUS_OK) ? AGC_STATUS_BAILOUT : bo); /* BAILOUT done: report it */
    }

    agc_erase_set_extvbact((uint16_t) (agc_erase_extvbact() | EXTVB_MARKING));
    int vac = -1; /* VAC area index claimed */

    if (agc_exec_vac_claim(&vac) != AGC_STATUS_OK)
    {
        agc_relint();
        const agc_status_t bo = agc_bailout(ALM_NO_VAC); /* status of BAILOUT */

        return ((bo == AGC_STATUS_OK) ? AGC_STATUS_BAILOUT : bo); /* BAILOUT done: report it */
    }

    agc_align_ram()->mkvac_index = vac;
    agc_erase_set_markstat((uint16_t) (vac + 1));                                                 /* MKVACFND */
    const agc_status_t st        = agc_exec_findvac(ALIGN_PRIO15, AGC_JOB_AOT_MARK_STAR_DISPLAY); /* status of the call */

    agc_relint();

    return (st);
}

/**********************************************************************
 * AGC: MKRELEAS (Luminary099 AOTMARK.agc p. 245)
 *
 * MKRELEAS task: MARKSTAT = 0, the VAC area freed; GOODEND (optics).
 *
 * FUNCTION NAME: agc_task_aot_mark_release
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_task_aot_mark_release(void)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    agc_erase_set_markstat(0);
    agc_status_t st = AGC_STATUS_OK; /* status of the call */

    if (ram->mkvac_index >= 0)
    {
        st = agc_exec_vac_release(ram->mkvac_index);
    }

    ram->mkvac_index = -1;

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_imu_goodend(AGC_IMU_DEV_OPTICS));
}

/**********************************************************************
 * AGC: KILLAOT (Luminary099 AOTMARK.agc p. 246)
 *
 * KILLAOT (V34 on a mark display): EXTVBACT = 0, GOTOPOOH.
 *
 * FUNCTION NAME: agc_job_aot_mark_kill
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_aot_mark_kill(void)
{
    agc_erase_set_extvbact(0);

    return (agc_exec_gotopooh());
}

/**********************************************************************
 * AGC: GETDAT (Luminary099 AOTMARK.agc p. 246)
 *
 * GETDAT: MARKSTAT bit 12 (no marks now); GOMARKF V01N71 (star code and
 * detent); V33 DODAT, ENTER shows it again.
 *
 * FUNCTION NAME: agc_job_aot_mark_star_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_aot_mark_star_display(agc_job_ctx_t *ctx)
{
    agc_erase_set_markstat((uint16_t) (agc_erase_markstat() | MS_NOT_WANTED));

    return (align_flash(ctx, AGC_DSKY_GOMARKF, VN_STAR_CODE, AGC_JOB_AOT_MARK_KILL, AGC_JOB_AOT_MARK_DETENT, AGC_JOB_AOT_MARK_STAR_DISPLAY));
}

/**********************************************************************
 * AGC: PASTIT (Luminary099 AOTMARK.agc p. 252)
 *
 * PASTIT: GOMARK4 with a mark verb; V33 MARKCHEX, ENTER GETDAT.
 *
 * FUNCTION NAME: request_marks
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *   vn        uint16_t               I    verb and noun, VN packing
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t request_marks(const agc_job_ctx_t *ctx, uint16_t vn)
{
    return (align_flash(ctx, AGC_DSKY_GOMARK4, vn, AGC_JOB_AOT_MARK_KILL, AGC_JOB_AOT_MARK_CHECK, AGC_JOB_AOT_MARK_STAR_DISPLAY));
}

/**********************************************************************
 * AGC: GETMKS (Luminary099 AOTMARK.agc p. 252)
 *
 * GETMKS +3: MARKSTAT keeps only its VAC area; V54N71.
 *
 * FUNCTION NAME: request_more_marks
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t request_more_marks(const agc_job_ctx_t *ctx)
{
    agc_erase_set_markstat((uint16_t) (agc_erase_markstat() & MS_VAC));

    return (request_marks(ctx, VN_MARK_XY));
}

/**********************************************************************
 * OPTAXIS (p. 252) as a step of GETDAT: OANB (SCAXIS, UXP, UYP), the
 * reticle planes XPNB, YPNB for the field rotation rot, STARAD +6 = 0;
 * GETMKS: XYMARK = MARKCNTR = 0, V54N71.
 *
 * FUNCTION NAME: optic_axis
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *   az        uint16_t               I    AOT azimuth, CDU word (2^15 = 1
 *                                         rev)
 *   el        uint16_t               I    AOT elevation, CDU word (2^15 = 1
 *                                         rev)
 *   rot       uint16_t               I    field rotation ROT, CDU word
 *                                         (2^15 = 1 rev)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t optic_axis(const agc_job_ctx_t *ctx, uint16_t az, uint16_t el, uint16_t rot)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    agc_aot_oanb(&ip, az, el, &ram->scaxis, &ram->uxp, &ram->uyp);
    agc_aot_optaxis(&ip, ram->uxp, ram->uyp, rot, &ram->xpnb, &ram->ypnb);
    ram->rotang    = rot;
    ram->starad[1] = agc_vec_zero(); /* LO6ZEROS */
    ram->xymark    = 0;              /* GETMKS   */
    ram->markcntr  = 0;

    return (request_more_marks(ctx));
}

/**********************************************************************
 * AGC: DODAT (Luminary099 AOTMARK.agc p. 246)
 *
 * DODAT: detent = AOTCODE bits 7-9 (XYMARK); 0 (COAS calibration) is not
 * valid here (GETDAT); 7 is the COAS (CODE7, AZ and EL keyed in, no
 * rotation); 1-6: AZ, EL of the detent and the apparent field rotation
 * AOTAZ +1 - AZ (CODE1TO6). DEVIATION: the AGC takes AOTCODE bits 7-15
 * (MASK HIGH9, MP BIT9), so a code keyed with a nonzero digit above the
 * detent digit indexes past AOTAZ/AOTEL; the C keeps bits 7-9 (the N71
 * detent digit).
 *
 * FUNCTION NAME: agc_job_aot_mark_detent
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_aot_mark_detent(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram();                                            /* alignment erasables */
    const agc_sp_t   d   = (agc_sp_t) ((ram->starcode >> DETENT_SHIFT) & DETENT_MASK); /* detent code 0..7 (STARCODE bits 7-9), unscaled */

    ram->xymark = d;

    if (d <= 0)
    {
        return (agc_job_aot_mark_star_display(ctx));
    }

    if (d == DETENT_COAS)
    {
        return (agc_job_aot_mark_coas_display(ctx));
    }

    const uint16_t az = agc_erase_aotaz((int) d); /* AOTAZ of the detent, CDU word */
    const uint16_t el = agc_erase_aotel((int) d); /* AOTEL of the detent, CDU word */

    /* MSU: the 2's complement difference of two CDU words, delivered as
     * the 1's complement word of the same value (yaAGC MSU; oracle
     * code1to6).                                                       */
    int32_t d_msu = (int32_t) (agc_erase_aotaz(2) & 077777u) - (int32_t) (az & 077777u); /* AOTAZ +1 - AZ (MSU), CDU counts */

    if (d_msu >= CDU_HALF)
    {
        d_msu -= CDU_MOD;
    }
    else if (d_msu < -CDU_HALF)
    {
        d_msu += CDU_MOD;
    }
    else
    {
        /* in range */
    }

    return (optic_axis(ctx, az, el, align_sp_to_word((agc_sp_t) d_msu)));
}

/**********************************************************************
 * AGC: CODE7 (Luminary099 AOTMARK.agc p. 246)
 *
 * CODE7: GOMARKF V06N87 (AZ, EL of the COAS).
 *
 * FUNCTION NAME: agc_job_aot_mark_coas_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_aot_mark_coas_display(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOMARKF, VN_AZ_EL, AGC_JOB_AOT_MARK_KILL, AGC_JOB_AOT_MARK_COAS_LOADED, AGC_JOB_AOT_MARK_COAS_DISPLAY));
}

/**********************************************************************
 * AGC: CODE7 (Luminary099 AOTMARK.agc p. 246)
 *
 * CODE7 V33: AZ, EL as loaded; COASCODE: no apparent rotation.
 *
 * FUNCTION NAME: agc_job_aot_mark_coas_loaded
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_aot_mark_coas_loaded(agc_job_ctx_t *ctx)
{
    const agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    return (optic_axis(ctx, ram->az, ram->el, 0));
}

/**********************************************************************
 * AGC: AVESTAR (Luminary099 AOTMARK.agc p. 252)
 *
 * AVESTAR loop: each pair from MARKCNTR down to 0 into the average.
 *
 * FUNCTION NAME: average_marks
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t average_marks(agc_interp_t *ip)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_status_t     st  = AGC_STATUS_OK;   /* status of the call */

    for (int k = 0; (k < AGC_ALIGN_AOT_PAIRS) && (st == AGC_STATUS_OK); k++)
    {
        ram->mkdex                    = (agc_sp_t) (ram->mkdex + 1); /* ADS MKDEX */
        const agc_align_aot_pair_t *m = &ram->aot[ram->markcntr];    /* the mark */
        agc_vec_words_t             v;                               /* star line of sight of this pair (SM) at 24D, words, half-unit (2^1) */

        if (surface_marks())
        {
            agc_aot_surfstar(ip, m, ram->scaxis, ram->xpnb, ram->ypnb, &v);
        }
        else
        {
            agc_vec_t los; /* VXV UNIT: the flight LOS, half-unit (2^1) */

            st = agc_aot_flight_los(ip, m, ram->xpnb, ram->ypnb, &los);

            for (int i = 0; i < 3; i++)
            {
                v.c[i] = agc_dp_to_words(los.c[i]); /* STORE 24D */
            }
        }

        if (st == AGC_STATUS_OK)
        {
            agc_aot_aveit(ip, ram->mkdex, v, &ram->starad[1]);
        }

        ram->starsav2 = ram->starad[1];

        if (ram->markcntr <= 0)
        {
            break; /* ENDMARKS */
        }

        ram->markcntr = (agc_sp_t) (ram->markcntr - 1);
    }

    return (st);
} /* average_marks */

/**********************************************************************
 * MARKCHEX (p. 256): bit 12 (no more marks); MKDEX = 0.  With the last pair
 * incomplete it is dropped (MARKCNTR - 1), and with none at all alarm 111
 * and GETMKS.  AVESTAR, then ENDMARKS: MKRELEAS in 50 ms and ENDMARK.
 *
 * FUNCTION NAME: agc_job_aot_mark_check
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_BAILOUT after the BAILOUT
 **********************************************************************/
agc_status_t agc_job_aot_mark_check(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    agc_erase_set_markstat((uint16_t) (agc_erase_markstat() | MS_NOT_WANTED));
    ram->mkdex = 0;

    if ((agc_erase_markstat() & MS_XY) != MS_XY)
    { /* CNTCHK   */
        if (ram->markcntr <= 0)
        {
            agc_alarm(ALM_NO_PAIR); /* MKALARM  */

            ram->xymark   = 0;
            ram->markcntr = 0;

            return (request_more_marks(ctx)); /* GETMKS   */
        }

        ram->markcntr = (agc_sp_t) (ram->markcntr - 1);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st = average_marks(&ip); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_inhint();
    st = agc_waitlist_call(ENDMARKS_DT, AGC_TASK_AOT_MARK_RELEASE);
    agc_relint();

    if (st != AGC_STATUS_OK)
    {
        const agc_status_t bo = agc_bailout(AGC_ALARM_01203); /* status of BAILOUT */

        return ((bo == AGC_STATUS_OK) ? AGC_STATUS_BAILOUT : bo); /* BAILOUT done: report it */
    }

    return (agc_dsky_endmark()); /* ENDMARK  */
} /* agc_job_aot_mark_check */

/**********************************************************************
 * AGC: CHANGEVB (Luminary099 AOTMARK.agc p. 260)
 *
 * CHANGEVB: surface -> V06N79; flight -> V54/V53/V52 N71 by MKDEX.
 *
 * FUNCTION NAME: agc_job_aot_mark_next_action
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_aot_mark_next_action(agc_job_ctx_t *ctx)
{
    if (surface_marks())
    {
        return (agc_job_aot_mark_surface_display(ctx));
    }

    const agc_sp_t k  = agc_align_ram()->mkdex; /* MKDEX: 1 Y wanted, 2 X wanted (unscaled) */
    uint16_t       vn = VN_MARK_XY;             /* V54: X or Y mark */

    if (k == 1)
    {
        vn = VN_MARK_Y;
    }
    else if (k == 2)
    {
        vn = VN_MARK_X;
    }
    else
    {
        /* both */
    }

    return (request_marks(ctx, vn));
}

/**********************************************************************
 * AGC: DSPV6N79 (Luminary099 AOTMARK.agc p. 261)
 *
 * DSPV6N79: GOMARKF V06N79 (CURSOR, SPIRAL, POSCODE).
 *
 * FUNCTION NAME: agc_job_aot_mark_surface_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t agc_job_aot_mark_surface_display(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOMARKF, VN_CURSOR, AGC_JOB_AOT_MARK_KILL, AGC_JOB_AOT_MARK_SURFACE_END, AGC_JOB_AOT_MARK_SURFACE_ANSWER));
}

/**********************************************************************
 * AGC: SURFAGAN (Luminary099 AOTMARK.agc p. 261)
 *
 * SURFAGAN: CURSOR, SPIRAL into the pair MKDEX; marking ended (bit 14) ->
 * MARKCHEX; otherwise another mark, unless five are in (5MKALARM: alarm 107
 * and V06N79 again).
 *
 * FUNCTION NAME: surface_again
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
static agc_status_t surface_again(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram  = agc_align_ram(); /* alignment erasables */
    const agc_sp_t   pair = ram->mkdex;      /* MKDEX: the pair index (unscaled) */

    if (!AGC_ASSERT((pair >= 0) && (pair < AGC_ALIGN_AOT_PAIRS)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    ram->aot[pair].ycdu[0] = ram->cursor;
    ram->aot[pair].ycdu[1] = ram->spiral;

    if ((agc_erase_markstat() & MS_PAIR) != 0u)
    {
        return (agc_job_aot_mark_check(ctx));
    }

    if ((ram->markcntr + 1) >= AGC_ALIGN_AOT_PAIRS)
    {
        agc_alarm(ALM_FIVE); /* 5MKALARM */

        return (agc_job_aot_mark_surface_display(ctx));
    }

    ram->markcntr = (agc_sp_t) (ram->markcntr + 1);

    return (request_more_marks(ctx)); /* GETMKS +3 */
}

/**********************************************************************
 * AGC: DSPV6N79 (Luminary099 AOTMARK.agc p. 261)
 *
 * DSPV6N79 V33 (SURFEND): bit 14 (end of marking), then SURFAGAN.
 *
 * FUNCTION NAME: agc_job_aot_mark_surface_end
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_aot_mark_surface_end(agc_job_ctx_t *ctx)
{
    agc_erase_set_markstat((uint16_t) (agc_erase_markstat() | MS_PAIR));

    return (surface_again(ctx));
}

/**********************************************************************
 * AGC: DSPV6N79 (Luminary099 AOTMARK.agc p. 261)
 *
 * DSPV6N79 V32 -> SURFAGAN (another mark); a data ENTER shows it again.
 *
 * FUNCTION NAME: agc_job_aot_mark_surface_answer
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_aot_mark_surface_answer(agc_job_ctx_t *ctx)
{
    if (align_resp_recycle())
    {
        return (surface_again(ctx));
    }

    return (agc_job_aot_mark_surface_display(ctx));
}

/**********************************************************************
 * VACSTOR (p. 260): TSIGHT = the mark time; the CDUs (Y, Z, X) into pair
 * MARKCNTR, X or Y half.  Surface: MKDEX = the pair, CHANGEVB.  Flight: bit
 * 13 off, the mark's bit on; both bits -> bit 14 (pair complete); REMARK:
 * MKDEX = verb index (bits 10-11), CHANGEVB.
 *
 * FUNCTION NAME: vacstor
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description (scale)
 *   --------  --------------------  ---  --------------------------------
 *   s         const aot_sample_t *  I    the button sample (time, IMU CDUs)
 *   y_mark    bool                  I    true for a Y mark
 *   mark_bit  uint16_t              I    MARKSTAT bit of this mark (X or Y)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
static agc_status_t vacstor(const aot_sample_t *s, bool y_mark, uint16_t mark_bit)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    if (!AGC_ASSERT((ram->markcntr >= 0) && (ram->markcntr < AGC_ALIGN_AOT_PAIRS)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    ram->tsight             = s->t;
    agc_align_aot_pair_t *p = &ram->aot[ram->markcntr]; /* the pair slot (MARKCNTR) */

    for (int i = 0; i < 3; i++)
    {
        if (y_mark)
        {
            p->ycdu[i] = s->cdu_yzx[i];
        }
        else
        {
            p->xcdu[i] = s->cdu_yzx[i];
        }
    }

    if (surface_marks())
    {
        ram->mkdex = ram->markcntr;
    }
    else
    {
        uint16_t ms = (uint16_t) ((agc_erase_markstat() & ~(uint16_t) (MS_REJECTED | mark_bit)) | mark_bit); /* MARKSTAT with this mark's bit set */

        if ((ms & MS_XY) == MS_XY)
        {
            ms = (uint16_t) (ms | MS_PAIR);
        }

        agc_erase_set_markstat(ms);
        ram->mkdex = (agc_sp_t) ((ms & MS_XY) >> 9); /* REMARK   */
    }

    return (agc_exec_novac(ALIGN_PRIO15, AGC_JOB_AOT_MARK_NEXT_ACTION)); /* SURFJOB */
} /* vacstor */

/**********************************************************************
 * XMKRUPT / YMKRUPT (p. 259): a surface mark only stores its CDUs (both
 * bits set, SURFSTOR).  In flight, after a complete pair a new pair is
 * started (MARKCNTR + 1, bits 10, 11, 14 cleared) unless five are in (alarm
 * 107); a mark already made is not wanted (alarm 114).
 *
 * FUNCTION NAME: mark_button
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description (scale)
 *   --------  --------------------  ---  --------------------------------
 *   s         const aot_sample_t *  I    the button sample (time, IMU CDUs)
 *   y_mark    bool                  I    true for the Y mark button
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_ALARM after an alarm
 **********************************************************************/
static agc_status_t mark_button(const aot_sample_t *s, bool y_mark)
{
    agc_align_ram_t *ram = agc_align_ram();      /* alignment erasables */
    const uint16_t   bit = y_mark ? MS_Y : MS_X; /* MARKSTAT bit of this mark; Y or X */

    ram->xymark = (agc_sp_t) bit;

    if (surface_marks())
    { /* SURFSTOR */
        agc_erase_set_markstat((uint16_t) (agc_erase_markstat() | MS_XY));

        return (vacstor(s, false, bit));
    }

    if ((agc_erase_markstat() & MS_PAIR) != 0u)
    {
        if ((MAX_PAIR - ram->markcntr) <= 0)
        {
            agc_alarm(ALM_FIVE);

            return (AGC_STATUS_ALARM); /* 5MKALARM */
        }

        ram->markcntr = (agc_sp_t) (ram->markcntr + 1);
        agc_erase_set_markstat((uint16_t) (agc_erase_markstat() & ~MS_PAIR_CLR));
    }

    if ((agc_erase_markstat() & bit) != 0u)
    { /* VERIFYMK */
        agc_alarm(ALM_UNWANTED);

        return (AGC_STATUS_ALARM);
    }

    return (vacstor(s, y_mark, bit));
}

/**********************************************************************
 * MARKRUPT (p. 258): sample CDUY, CDUZ, CDUX and TIME2/1 at once; then only
 * the X / Y mark and reject buttons count, and only while marks are wanted
 * (bit 12 clear) and being accepted (MARKSTAT non-zero, else alarm 112).
 * RESUME is the return.
 *
 * FUNCTION NAME: agc_aotmark_markrupt
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_ALARM after an alarm
 **********************************************************************/
agc_status_t agc_aotmark_markrupt(void)
{
    aot_sample_t s; /* the sample: TIME2/1 and CDUY, CDUZ, CDUX */

    s.cdu_yzx[0]      = agc_erase_cduy();
    s.cdu_yzx[1]      = agc_erase_cduz();
    s.cdu_yzx[2]      = agc_erase_cdux();
    s.t               = agc_rtb_loadtime();
    const uint16_t in = agc_chan_read(AGC_CHAN_NAVKEYIN); /* channel 16 (NAVKEYIN) */

    if ((in & MARK_KEYS) == 0u)
    {
        /* SOMEKEY: the rate-of-descent switch (DESCBITS), else alarm 113. */
        if ((in & DESCENT_KEYS) != 0u)
        {
            return (agc_lm_descbits()); /* POSTJUMP */
        }

        agc_alarm(ALM_NO_INBITS);

        return (AGC_STATUS_ALARM);
    }

    const uint16_t ms = agc_erase_markstat(); /* MARKSTAT */

    if ((ms & MS_NOT_WANTED) != 0u)
    {
        return (AGC_STATUS_OK); /* RESUME   */
    }

    if (ms == 0u)
    {
        agc_alarm(ALM_NOT_TAKEN);

        return (AGC_STATUS_ALARM);
    }

    if ((in & AGC_CH016_AOTREJECT) != 0u)
    {
        /*
         * MKREJ (p. 259): surface: one mark fewer (alarm 115 if none).  Flight:
         * with no mark made alarm 115; otherwise bit 13 set and, if the last
         * thing was a mark, that mark taken back, or after a second reject both;
         * then REMARK.
         */
        agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

        if (surface_marks())
        { /* SURFREJ  */
            if (ram->markcntr <= 0)
            {
                agc_alarm(ALM_NO_REJECT);

                return (AGC_STATUS_ALARM);
            }

            ram->markcntr = (agc_sp_t) (ram->markcntr - 1);

            return (AGC_STATUS_OK);
        }

        const uint16_t old = agc_erase_markstat(); /* MARKSTAT before the reject */

        if ((old & MS_XY) == 0u)
        {
            agc_alarm(ALM_NO_REJECT);

            return (AGC_STATUS_ALARM); /* REJALM   */
        }

        uint16_t       ms_reject = (uint16_t) ((old & ~MS_REJECT_CLR) + MS_REJECTED);            /* MARKSTAT with the reject bit set */
        const uint16_t drop      = ((old & MS_REJECTED) != 0u) ? MS_XY : (uint16_t) ram->xymark; /* the mark bits the reject removes; second reject: both marks of the pair */

        ms_reject  = (uint16_t) (ms_reject & ~drop);        /* RENEWMK  */
        agc_erase_set_markstat(ms_reject);
        ram->mkdex = (agc_sp_t) ((ms_reject & MS_XY) >> 9); /* REMARK   */

        return (agc_exec_novac(ALIGN_PRIO15, AGC_JOB_AOT_MARK_NEXT_ACTION));
    }

    if ((in & AGC_CH016_YAXISMARK) != 0u)
    {
        return (mark_button(&s, true));
    }

    return (mark_button(&s, false)); /* XMKRUPT  */
} /* agc_aotmark_markrupt */
