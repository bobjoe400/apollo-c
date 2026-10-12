/**********************************************************************
 * FILE NAME: w_matrix_monitor.c
 *
 * PURPOSE:
 *   V67 W-MATRIX MONITOR, both vehicles. The extended verb shows on noun
 *   99 the RMS position and velocity errors of the W matrix (WWPOS =
 *   SQRT(sum of the squares of the position rows), WWVEL; LM also WWBIAS
 *   for the radar bias rows), and lets the crew load new initial values:
 *     CM: R3 = WWOPT selects which pad pair gets WWPOS, WWVEL (1
 *         rendezvous WRENDPOS/WRENDVEL, 2 orbital WORBPOS/WORBVEL, 3
 *         midcourse WMIDPOS/WMIDVEL); 0 changes nothing.
 *     LM: a change of any displayed value (V67FLAG) loads WRENDPOS/
 *         WRENDVEL (WSURFPOS/WSURFVEL on the surface) and WSHAFT = WTRUN
 *         = WWBIAS.
 *   The W matrix is then invalid (RENDWFLG, CM also ORBWFLAG, cleared).
 *
 * AGC SOURCE:
 *   Comanche055 P20-P25.agc pp. 632-634 (V67CALL, V06N99DS, V06N9933,
 *   V6N99INP, V67WORB, V67WMID, V67EXITX, V67WW, NXPOSVEL, V67XXX)
 *   Luminary099 P20-P25.agc pp. 608-611 (differs: WWBIAS, V67FLAG,
 *   V6N99PRO, V67SURF, V67CLRF)
 *
 * EXTERNAL VARIABLES:
 *   Name       Type        I/O  Scale      Description
 *   ---------  ----------  ---  ---------  ----------------------------
 *   W          agc_incorp_w_t I  as integrated  error transition matrix
 *   WWPOS (= RANGE)  agc_dp_t I/O  m 2^19  RMS position error (N99)
 *   WWVEL (= RRATE)  agc_dp_t I/O  m/cs    RMS velocity error (N99)
 *   WWOPT / WWBIAS (= RTHETA) agc_dp_t I/O B-5  CM option / LM bias
 *   WRENDPOS WRENDVEL WORBPOS WORBVEL WMIDPOS WMIDVEL WSURFPOS WSURFVEL
 *   WTRUN WSHAFT  agc_sp_t  O    pad loads  W initial values
 *   RENDWFLG ORBWFLAG SURFFLAG V67FLAG  flags  I/O  -
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  ---------------------------------------------
 *   agc_integ_install      INTSTALL / INTWAKE around the W read
 *   agc_rendz_nav_pad      the navigation pad loads
 *   rz_flash               GOXDSPF (GOMARKF) display request
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. An overflow makes all displayed values DPPOSMAX. Jobs
 *   return AGC_STATUS_BAD_ARG for a NULL ctx, else the INTSTALL or
 *   display status.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Extended verb jobs; the W read sleeps when the integration is busy.
 *
 * NOTES:
 *   Scales: WWPOS m 2^19 (limited to 99999 ft), WWVEL m/cs, WWBIAS noun
 *   scale B-5 (internal B+5); the W elements as integrated.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "internal.h"

#define WM_FT99999 ((agc_dp_t) 15605248) /* FT99999 2DEC 30479 B-19      */
#define WM_DEC2    ((agc_sp_t) 2)        /* V67DEC2 2DEC 2 B-14 (option 2) */

/* LM: the noun 99 values before the display (WWBIAS +2 .. +7).          */
static agc_dp_t wm_saved[3]; /* N99 values shown (LM), N99 scales */

/**********************************************************************
 * Sum of |row|^2 over rows 0..rows-1 of a W column block.
 * AGC: NXPOSVEL "VLOAD* VSQ / DAD" (P20-P25.agc p.633)
 *
 * FUNCTION NAME: wm_sum
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ------------------------
 *   ip        agc_interp_t *          I/O  interpreter state (OVFIND)
 *   w         const agc_incorp_w_t *  I    the W matrix
 *   b         int                     I    column block (0 position, 1
 *                                          velocity, 2 LM bias)
 *   rows      int                     I    rows summed (CM 6, LM 9)
 *
 * RETURNS: agc_dp_t -- the sum of the row squares, W scale squared (0 for a
 *          bad block)
 **********************************************************************/
static agc_dp_t wm_sum(agc_interp_t *ip, const agc_incorp_w_t *w, int b, int rows)
{
    agc_dp_t s = 0; /* sum, W scale squared */

    for (int m = 0; m < rows && m < AGC_INCORP_ROWS; m++)
    {
        s = agc_dp_add(ip, agc_vec_sq(ip, w->blk[b][m]), s); /* VSQ, DAD */
    }

    return (s);
}

/**********************************************************************
 * V67WW: under INTSTALL, the RMS values; an overflow anywhere makes them
 * all DPPOSMAX; WWPOS limited to 99999 ft; INTWAKE.
 * AGC: V67WW (P20-P25.agc p.633)
 *
 * FUNCTION NAME: wm_compute
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t wm_compute(agc_interp_t *ip, agc_rendz_t *rz)
{
    agc_status_t st = agc_integ_install(0, AGC_JOB_W_MATRIX_MONITOR); /* W */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_interp_clear_ovf(ip);                                /* BOV +1   */
    const agc_incorp_w_t *w    = &agc_integ_shared_ctx()->w; /* W matrix */
    bool                  csm  = rz_is_csm();                /* CM listing */
    int                   rows = csm ? 6 : 9;                /* X1 36/54 */
    agc_dp_t              pos  = wm_sum(ip, w, 0, rows);     /* m^2 2^38 */
    agc_dp_t              vel  = wm_sum(ip, w, 1, rows);     /* W scale^2 */

    rz->rtheta = 0; /* CM: WWOPT = 0 */

    if (!csm)
    { /* LM WWBIAS, B-5 */
        rz->rtheta = agc_dp_shr(agc_dp_sqrt(wm_sum(ip, w, 2, rows)), 10);
    }

    rz->rrate = agc_dp_sqrt(vel);
    rz->range = agc_dp_sqrt(pos);

    if (agc_interp_test_ovf(ip))
    {
        rz->range = RZ_DPPOSMAX;
        rz->rrate = RZ_DPPOSMAX;

        if (!csm)
        {
            rz->rtheta = RZ_DPPOSMAX;
        }
    }

    if (agc_dp_sub(ip, rz->range, WM_FT99999) >= 0)
    { /* V67XXX   */
        rz->range = WM_FT99999;
    }

    return (agc_integ_intwake());
} /* wm_compute */

/**********************************************************************
 * V67CALL: compute, (LM) keep the values, V06N99 (GOXDSPF): TERMINATE
 * ends the verb, PROCEED takes the loads, ENTER shows it again.
 * AGC: V67CALL (P20-P25.agc p.632)
 *
 * FUNCTION NAME: agc_job_w_matrix_monitor
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
agc_status_t agc_job_w_matrix_monitor(agc_job_ctx_t *ctx)
{
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */
    agc_interp_t ip;                        /* interpreter state */

    agc_interp_init(&ip);

    if (!rz_is_csm())
    {
        rz_flag_put(AGC_FLAG_W_MATRIX_OVERWRITE, false); /* V67FLAG  */
    }

    agc_status_t st = wm_compute(&ip, rz); /* V67WW */

    if (st != AGC_STATUS_OK)
    {
        return (st); /* BUSY: asleep in INTSTALL  */
    }

    wm_saved[0] = rz->range;
    wm_saved[1] = rz->rrate;
    wm_saved[2] = rz->rtheta;

    return (agc_job_w_matrix_monitor_display(ctx));
}

/**********************************************************************
 * V06N99DS: V06N99 (GOXDSPF); TERMINATE ends the verb, PROCEED takes the
 * loads, ENTER shows it again.
 * AGC: V06N99DS (P20-P25.agc p.632)
 *
 * FUNCTION NAME: agc_job_w_matrix_monitor_display
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
agc_status_t agc_job_w_matrix_monitor_display(agc_job_ctx_t *ctx)
{
    return (rz_flash(ctx, AGC_DSKY_GOMARKF, 6U, 99U, AGC_JOB_RENDZ_ENDEXT, AGC_JOB_W_MATRIX_MONITOR_PROCEED, AGC_JOB_W_MATRIX_MONITOR_DISPLAY)); /* GOXDSPF */
}

/**********************************************************************
 * PROCEED on V06N99: (LM V6N99PRO) any displayed value changed sets
 * V67FLAG; load the pads; W invalid (RENDWFLG off); ENDEXT.
 * AGC: V06N9933, V6N99PRO (P20-P25.agc p.632)
 *
 * FUNCTION NAME: agc_job_w_matrix_monitor_proceed
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_w_matrix_monitor_proceed(void)
{
    agc_rendz_t         *rz = agc_rendz_erasable(); /* rendezvous erasables */
    agc_rendz_nav_pad_t *p  = agc_rendz_nav_pad();  /* pad loads */
    agc_interp_t         ip;                        /* interpreter state */

    agc_interp_init(&ip);

    if (rz_is_csm())
    {
        if (agc_dp_high(rz->rtheta) == 0)
        { /* BHIZ     */
            return (agc_dsky_endmark());
        }

        /**********************************************************************
         * CM V06N9933 / V6N99INP: option 0 changes nothing; else the pad pair
         * of the option gets the high words of WWPOS, WWVEL.
         * AGC: V06N9933, V6N99INP (P20-P25.agc pp.632-633)
         *
         * FUNCTION NAME: wm_load_csm
         *
         * ARGUMENTS:
         *   Argument  Type                   I/O  Description (scale)
         *   --------  ---------------------  ---  -------------------------
         *   rz        const agc_rendz_t *    I    rendezvous erasables
         *   p         agc_rendz_nav_pad_t *  O    the pad pair of the option
         **********************************************************************/
        agc_sp_t opt = agc_dp_high(rz->rtheta); /* WWOPT, unscaled */
        agc_sp_t x1  = agc_dp_high(rz->range);  /* LXA,1, pad scale */
        agc_sp_t x2  = agc_dp_high(rz->rrate);  /* LXA,2, pad scale */

        if (opt == WM_DEC2)
        { /* V67WORB  */
            p->worbpos = x1;
            p->worbvel = x2;
        }
        else if (opt > WM_DEC2)
        { /* V67WMID  */
            p->wmidpos = x1;
            p->wmidvel = x2;
        }
        else
        {
            p->wrendpos = x1;
            p->wrendvel = x2;
        }

        rz_flag_put(AGC_FLAG_ORBWFLAG, false); /* V67EXITX */
    }
    else
    {
        if (rz->range != wm_saved[0] || rz->rrate != wm_saved[1] || rz->rtheta != wm_saved[2])
        {
            rz_flag_put(AGC_FLAG_W_MATRIX_OVERWRITE, true);
        }

        if (!agc_flag_test(AGC_FLAG_W_MATRIX_OVERWRITE))
        {
            return (agc_dsky_endmark());
        }

        /**********************************************************************
         * LM V06N9933: WWPOS SL5 and WWVEL to WRENDPOS/WRENDVEL (WSURF.. on the
         * surface), WWBIAS SL10 to WTRUN and WSHAFT.
         * AGC: Luminary099 V06N9933 (P20-P25.agc p.609)
         *
         * FUNCTION NAME: wm_load_lm
         *
         * ARGUMENTS:
         *   Argument  Type                   I/O  Description (scale)
         *   --------  ---------------------  ---  -------------------------
         *   ip        agc_interp_t *         I/O  interpreter state (OVFIND)
         *   rz        const agc_rendz_t *    I    rendezvous erasables
         *   p         agc_rendz_nav_pad_t *  O    WREND / WSURF pair, WTRUN, WSHAFT
         **********************************************************************/
        agc_sp_t pos; /* SL5, pad scale */

        pos          = agc_dp_high(agc_dp_shl(&ip, agc_dp_shl(&ip, rz->range, 4), 1));
        agc_sp_t vel = agc_dp_high(rz->rrate); /* pad scale */
        agc_sp_t bias;                         /* SL10, pad scale */

        bias = agc_dp_high(agc_dp_shl(&ip, rz->rtheta, 10));

        if (agc_flag_test(AGC_FLAG_SURFFLAG))
        { /* V67SURF  */
            p->wsurfpos = pos;
            p->wsurfvel = vel;
        }
        else
        {
            p->wrendpos = pos;
            p->wrendvel = vel;
        }

        p->wtrun  = bias; /* V67CLRF  */
        p->wshaft = bias;
    }

    rz_flag_put(AGC_FLAG_RENDWFLG, false);

    return (agc_dsky_endmark());
}
