/**********************************************************************
 * FILE NAME: sxtmark.c
 *
 * PURPOSE:
 *   SXTMARK, the CSM optics mark system.  SXTMARK reserves a VAC area for
 *   the marks and asks the crew for them (V51 "please mark", V50N25 when
 *   all are in); MARKRUPT (the MARK and MARK REJECT buttons on the nav
 *   DSKY) samples the CDUs and time; MARKDIF checks that the vehicle did
 *   not move more than 3 counts in CDUCHKWD cs; MARKCONT / MARK2 /
 *   VACSTOR store the mark; MKREJECT takes the last one back.
 *
 *   MARKSTAT (agc_erase_markstat):
 *     bits 1-9   VAC area in use (DEVIATION: VAC index + 1, not the address)
 *     bit 10     a mark has been made since the last reject (reject allowed)
 *     bits 12-14 number of marks still wanted
 *   QPRET (agc_align_ram()->qpret): next free mark slot while marking, the
 *   number of marks made after ENTANSWR.  Each mark is 7 words (MKT2T1,
 *   CDUY, CDUS, CDUZ, CDUT, CDUX), agc_align_sxt_mark_t.
 *
 * AGC SOURCE:
 *   Comanche055 SXTMARK.agc pp. 222-235
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   MARKSTAT      uint16_t  I/O  bits           agc_erase_markstat()
 *   EXTVBACT      uint16_t  I/O  bits           bit 2: marking
 *   CDUX..CDUT, TIME2/1  I                      sampled at the button
 *   CDUCHKWD      agc_sp_t  I    cs             MARKDIF delay
 *   MRKBUF1/2, MARKDOWN  uint16_t O raw words   R21 / P23 marks
 *   OPTMODES, OPTIND  O                          MKRELEAS
 *   R21MARK, V59FLAG, TRM03FLG  flags (I/O)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dsky_*            DSKY displays, astronaut returns (dsky.h)
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_ram         alignment routines of this lane (alignment.h)
 *   agc_exec_*            executive: jobs, VAC areas, sleep / wake (exec.h)
 *   agc_waitlist_call     WAITLIST (exec.h)
 *   agc_imu_goodend       IMU mode switching, CALCGA / CALCGTA / AXISGEN, gyro torquing (imu.h)
 *   agc_dp_*              DP arithmetic of the interpreter (interp.h, word.h)
 *   agc_flag_*            flag words (flags_common.h)
 *   agc_phase_checkmm     restart phase tables, CHECKMM (exec.h)
 *   agc_erase_*           erasable accessors (state.h)
 *   agc_chan_*            I/O channels (channels.h)
 *   agc_alarm             ALARM (exec.h)
 *   agc_bailout           BAILOUT (exec.h)
 *   agc_inhint            INHINT / RELINT (exec.h)
 *   agc_relint            INHINT / RELINT (exec.h)
 *   agc_job_auto_optics_positioning_terminate  see its header
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Alarms: ALM_BUSY 01211: marks or extended verb busy; ALM_MOVED 00121:
 *   vehicle rate at mark; ALM_NO_INBITS 00113: no input bits in channel
 *   16; ALM_NO_REJECT 00112: marks not being accepted; ALM_NO_VAC 01207:
 *   no VAC area; ALM_REJECT2 00110: two rejects together; ALM_UNCALLED
 *   00122: marking not called for; ALM_UNWANTED 00114: mark not wanted.
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 *
 * NOTES:
 *   Confirmed against the listing (finding C7).  In P53 / P54 SXTMARK
 *   returns with INHINT still on; the C releases it (no observable
 *   effect).
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_csm.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"

static agc_status_t agc_sxtmark_markcont(void);
static agc_status_t agc_sxtmark_mkreject(void);

#define MS_VAC_MASK     ((uint16_t) 0777)          /* LOW9: VAC area               */
#define MS_REJECT_OK    ((uint16_t) 01000)         /* BIT10                        */
#define MS_WANTED_ONE   ((uint16_t) 04000)         /* BIT12: one mark wanted       */
#define MS_WANTED_MASK  ((uint16_t) 034000)        /* PRIO34: bits 12-14           */
#define MS_WANTED_MAX   7                          /* three bits                   */
#define EXTVB_MARKING   ((uint16_t) 02)            /* EXTVBACT BIT2: marks in use  */
#define EXTVB_BUSY      ((uint16_t) 06)            /* SIX: marks or ext verb busy  */
#define OPTM_COARSE_RET ((uint16_t) 0400)          /* OPTMODES BIT9                */
#define OPT_NONE        ((agc_sp_t) -1)            /* OPTIND -1: no coarse optics  */
#define VN_PLEASE_MARK  ALIGN_VN(51, 0)            /* VB51  VN 5100                */
#define VN_MARKS_DONE   ALIGN_VN(50, 25)           /* V50N25 VN 5025               */
#define CHK_MARKS_DONE  ((uint16_t) 016)           /* R1D1 OCT 16: checklist code  */
#define MARK_WORDS      7                          /* SIX + 1 words per mark       */
#define DIFCHK_MAX      3                          /* counts the CDU may move      */
#define ENDMARKS_DT     ((uint16_t) 5)             /* FIVE cs                      */
#define MARKDIF_MIN_DT  ((uint16_t) 1)             /* 10 ms if no check            */
#define ALM_BUSY        ((agc_alarm_code_t) 01211) /* marks or extended verb busy */
#define ALM_NO_VAC      ((agc_alarm_code_t) 01207) /* no VAC area */
#define ALM_REJECT2     ((agc_alarm_code_t) 00110) /* two rejects together    */
#define ALM_NO_REJECT   ((agc_alarm_code_t) 00112) /* marks not being accepted*/
#define ALM_NO_INBITS   ((agc_alarm_code_t) 00113) /* no input bits in channel 16 */
#define ALM_UNWANTED    ((agc_alarm_code_t) 00114) /* mark not wanted         */
#define ALM_MOVED       ((agc_alarm_code_t) 00121) /* vehicle rate at mark    */
#define ALM_UNCALLED    ((agc_alarm_code_t) 00122) /* marking not called for  */
#define CDU_MASK        077777u                    /* 15-bit CDU word */
#define CDU_MOD         0100000                    /* one revolution, CDU counts */

/* MKT2T1, MKCDUY, MKCDUS, MKCDUZ, MKCDUT, MKCDUX: the mark being processed. */
static agc_align_sxt_mark_t mk; /* MKT2T1, MKCDUY, MKCDUS, MKCDUZ, MKCDUT, MKCDUX: the mark being processed */

/**********************************************************************
 * AGC: MKT2T1 (Comanche055 ERASABLE_ASSIGNMENTS.agc p. 73)
 *
 * The mark as the 7 raw words of MKT2T1 .. MKCDUX.
 *
 * FUNCTION NAME: mark_words
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ------------------------
 *   m         const agc_align_sxt_mark_t *  I    the stored mark
 *   w         uint16_t                      O    the 7 raw words MKT2T1 .. MKCDUX
 *
 * RETURNS: none
 **********************************************************************/
static void mark_words(const agc_align_sxt_mark_t *m, uint16_t w[MARK_WORDS])
{
    w[0] = align_sp_to_word(agc_dp_high(m->t));
    w[1] = align_sp_to_word(agc_dp_low(m->t));
    w[2] = m->cduy;
    w[3] = m->cdus;
    w[4] = m->cduz;
    w[5] = m->cdut;
    w[6] = m->cdux;
}

/**********************************************************************
 * AGC: MARKOK (Comanche055 SXTMARK.agc p. 223)
 *
 * MARKOK / MKVACFND: claim a VAC area for the marks (BAILOUT 01207).
 *
 * FUNCTION NAME: markok
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_ALARM after an alarm
 **********************************************************************/
static agc_status_t markok(void)
{
    agc_align_ram_t   *ram = agc_align_ram();          /* alignment erasables */
    int                vac = -1;                       /* VAC area index claimed */
    const agc_status_t st  = agc_exec_vac_claim(&vac); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        const agc_status_t bo = agc_bailout(ALM_NO_VAC); /* status of BAILOUT */

        return ((bo == AGC_STATUS_OK) ? AGC_STATUS_ALARM : bo); /* BAILOUT done: report the alarm */
    }

    ram->mkvac_index = vac;
    agc_erase_set_markstat((uint16_t) (vac + 1));
    ram->qpret       = 0; /* next mark slot       */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * SXTMARK / SXTMARK +2 (p. 222): abort 01211 if the marks or an extended
 * verb are busy; set EXTVBACT bit 2; find a VAC area.  P53 and P54 (the
 * backup programs, R56) only want the area; otherwise MARKSTAT gets the
 * number of marks wanted and MKVB51 is started at PRIO32.
 *
 * FUNCTION NAME: sxtmark_common
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   nmarks    agc_sp_t  I    number of marks wanted (bits 1-3)
 *   want      bool      I    false for SXTMARK +2 (R56): only the VAC area
 *                            is wanted
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_ALARM after an alarm
 **********************************************************************/
static agc_status_t sxtmark_common(agc_sp_t nmarks, bool want)
{
    agc_inhint();

    if ((agc_erase_extvbact() & EXTVB_BUSY) != 0u)
    {
        agc_relint();
        const agc_status_t bo = agc_bailout(ALM_BUSY); /* status of BAILOUT */

        return ((bo == AGC_STATUS_OK) ? AGC_STATUS_ALARM : bo); /* BAILOUT done: report the alarm */
    }

    agc_erase_set_extvbact((uint16_t) (agc_erase_extvbact() | EXTVB_MARKING));
    agc_status_t st = markok(); /* status of the call */

    if ((st == AGC_STATUS_OK) && want && !agc_phase_checkmm(53) && !agc_phase_checkmm(54))
    {
        agc_erase_set_markstat((uint16_t) (agc_erase_markstat() + (uint16_t) nmarks * MS_WANTED_ONE));
        st = agc_exec_novac(ALIGN_PRIO32, AGC_JOB_MARK_REQUEST_DISPLAY);
    }

    agc_relint();

    return (st);
}

/**********************************************************************
 * AGC: SXTMARK (Comanche055 SXTMARK.agc p. 222)
 *
 * SXTMARK (p. 222) with the number of marks wanted.
 *
 * FUNCTION NAME: agc_sxtmark
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   nmarks    agc_sp_t  I    number of marks wanted (bits 1-3)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_sxtmark(agc_sp_t nmarks)
{
    return (sxtmark_common(nmarks, true));
}

/**********************************************************************
 * AGC: SXTMARK (Comanche055 SXTMARK.agc p. 222)
 *
 * SXTMARK +2 (R56): only reserve the mark VAC area.
 *
 * FUNCTION NAME: agc_sxtmark_reserve
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_sxtmark_reserve(void)
{
    return (sxtmark_common(0, false));
}

/**********************************************************************
 * MKRELEAS first half, also R53A's housekeeping when no mark was made
 * (Comanche p. 771): MARKSTAT = 0 and its VAC area freed.
 *
 * FUNCTION NAME: agc_sxtmark_free_vac
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_sxtmark_free_vac(void)
{
    agc_align_ram_t *ram = agc_align_ram();      /* alignment erasables */
    const uint16_t   old = agc_erase_markstat(); /* MARKSTAT before the clear */

    agc_erase_set_markstat(0);

    if (((old & MS_VAC_MASK) != 0u) && (ram->mkvac_index >= 0))
    {
        const agc_status_t st = agc_exec_vac_release(ram->mkvac_index); /* status of the call */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    ram->mkvac_index = -1;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * MKRELEAS / MKRLEES (p. 223): MARKSTAT = 0 and the VAC area freed; kill
 * the coarse optics (OPTMODES bit 9 off, OPTIND = -1) and disable the
 * optics error counter (channel 12 bit 2).
 *
 * FUNCTION NAME: agc_sxtmark_mkreleas
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_sxtmark_mkreleas(void)
{
    agc_status_t st = agc_sxtmark_free_vac(); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_inhint(); /* MKRLEES  */
    agc_erase_set_optmodes((uint16_t) (agc_erase_optmodes() & ~OPTM_COARSE_RET));
    agc_erase_set_optind(OPT_NONE);
    agc_chan_and(012, (uint16_t) ~AGC_CH012_ENBOCDUERR);
    agc_relint();

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * MARKRUPT (p. 224): sample CDUT, CDUS, CDUY, CDUZ, CDUX and TIME2/1 (also
 * SAMPTIME for N65); channel 16 bit 6 = MARK (MARKIT), bit 7 = MARK REJECT
 * (MKREJECT); otherwise a nav DSKY key code goes to KEYCOM, or alarm 113
 * when no input bit is on.
 *
 * FUNCTION NAME: agc_sxtmark_markrupt
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_sxtmark_markrupt(void)
{
    mk.cdut           = agc_erase_cdut();
    mk.cdus           = agc_erase_cdus();
    mk.cduy           = agc_erase_cduy();
    mk.cduz           = agc_erase_cduz();
    mk.cdux           = agc_erase_cdux();
    mk.t              = align_loadtime();
    agc_erase_set_samptime(mk.t);
    const uint16_t in = agc_chan_read(016); /* channel 16 (NAVKEYIN) */

    if ((in & AGC_CH016_MARK) != 0u)
    {
        uint16_t       dt  = MARKDIF_MIN_DT;       /* MARKIT   */
        const agc_sp_t chk = agc_erase_cduchkwd(); /* CDUCHKWD, cs (unscaled count) */

        if (chk > 0)
        {
            dt = (uint16_t) chk;
        }

        return (agc_waitlist_call(dt, AGC_TASK_MARKDIF));
    }

    if ((in & AGC_CH016_MARKREJECT) != 0u)
    {
        return (agc_sxtmark_mkreject());
    }

    if ((in & AGC_CH016_NAVKEY_MASK) != 0u)
    {
        return (agc_dsky_keyrupt2()); /* KEYCOM   */
    }

    agc_alarm(ALM_NO_INBITS);

    return (AGC_STATUS_OK); /* RESUME   */
}

/**********************************************************************
 * AGC: DIFCHK (Comanche055 SXTMARK.agc p. 234)
 *
 * DIFCHK: |mark CDU - present CDU| <= 3 counts (MSU, modular).
 *
 * FUNCTION NAME: difchk
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   mark_cdu  uint16_t  I    CDU word at the mark
 *   now_cdu   uint16_t  I    CDU word now
 *
 * RETURNS: bool -- true when |mark - now| <= 3 counts
 **********************************************************************/
static bool difchk(uint16_t mark_cdu, uint16_t now_cdu)
{
    int32_t d = (int32_t) (mark_cdu & CDU_MASK) - (int32_t) (now_cdu & CDU_MASK); /* mark - now, CDU counts (modulo one revolution) */

    if (d >= CDU_MOD / 2)
    {
        d -= CDU_MOD;
    }
    else if (d < -(CDU_MOD / 2))
    {
        d += CDU_MOD;
    }
    else
    {
        /* in range */
    }

    return ((d <= DIFCHK_MAX) && (d >= -DIFCHK_MAX));
}

/**********************************************************************
 * MARKDIF task (p. 235): with CDUCHKWD > 0 the IMU CDUs X, Y, Z must not
 * have moved more than 3 counts since the mark (alarm 121 and the mark is
 * dropped); then MARKCONT.
 *
 * FUNCTION NAME: agc_task_markdif
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_task_markdif(void)
{
    if (agc_erase_cduchkwd() > 0)
    {
        const bool ok = difchk(mk.cdux, agc_erase_cdux()) && difchk(mk.cduy, agc_erase_cduy()) && difchk(mk.cduz, agc_erase_cduz()); /* outcome of the check */

        if (!ok)
        {
            agc_alarm(ALM_MOVED);

            return (AGC_STATUS_OK); /* TASKOVER */
        }
    }

    return (agc_sxtmark_markcont());
}

/**********************************************************************
 * AGC: GENTRAN (Comanche055 SERVICE_ROUTINES.agc p. 1490)
 *
 * GENTRAN of the mark into MRKBUF1 (and MRKBUF1 into MRKBUF2 first).
 *
 * FUNCTION NAME: to_mrkbuf1
 *
 * ARGUMENTS:
 *   Argument       Type  I/O  Description (scale)
 *   -------------  ----  ---  -------------------------------------------
 *   shift_to_buf2  bool  I    true: MRKBUF1 goes to MRKBUF2 first (R21)
 *
 * RETURNS: none
 **********************************************************************/
static void to_mrkbuf1(bool shift_to_buf2)
{
    uint16_t w[MARK_WORDS]; /* MKT2T1 .. MKCDUX as raw words */

    mark_words(&mk, w);

    for (int i = 0; i < MARK_WORDS; i++)
    {
        if (shift_to_buf2)
        {
            agc_erase_set_mrkbuf(2, i, agc_erase_mrkbuf(1, i));
        }

        agc_erase_set_mrkbuf(1, i, w[i]);
    }
}

/**********************************************************************
 * MARKCONT (p. 225): R21MARK set: MRKBUF2 = MRKBUF1, MRKBUF1 = this mark
 * (R21).  MARKET: marks wanted -> MARK2; else with the mark system up and
 * V59FLAG set (P23 calibration) start MARKDISP and copy the mark into
 * MARKDOWN and MRKBUF1; otherwise alarm 122.
 *
 * FUNCTION NAME: agc_sxtmark_markcont
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t agc_sxtmark_markcont(void)
{
    if (agc_flag_test(AGC_FLAG_MARKRUPT_OPTION_ONE))
    {
        to_mrkbuf1(true); /* MARKIT1  */

        return (AGC_STATUS_OK);
    }

    if (agc_erase_markstat() != 0u)
    { /* CCS      */
        return (agc_sxtmark_mark2());
    }

    if (((agc_erase_extvbact() & EXTVB_MARKING) == 0u) || !agc_flag_test(AGC_FLAG_CALIBRATION_MARKING))
    {
        agc_alarm(ALM_UNCALLED); /* MARKET3  */

        return (AGC_STATUS_OK);
    }

    const agc_status_t st = agc_exec_novac(ALIGN_PRIO5, AGC_JOB_OPTICS_CALIBRATION_MARK_DISPLAY); /* status of the call */
    uint16_t           w[MARK_WORDS];                                                             /* MKT2T1 .. MKCDUX as raw words */

    mark_words(&mk, w);

    for (int i = 0; i < MARK_WORDS; i++)
    {
        agc_erase_set_markdown(i, w[i]);
    }

    to_mrkbuf1(false);

    return (st);
}

/**********************************************************************
 * MARK2 / VACSTOR (p. 226): one fewer mark wanted (alarm 114 if none was),
 * reject enabled, the mark stored in the next slot; when no more are
 * wanted, MKVB50 (V50N25) is started.
 *
 * FUNCTION NAME: agc_sxtmark_mark2
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_sxtmark_mark2(void)
{
    agc_align_ram_t *ram = agc_align_ram();      /* alignment erasables */
    uint16_t         ms  = agc_erase_markstat(); /* MARKSTAT */

    if ((ms & MS_WANTED_MASK) == 0u)
    {
        agc_alarm(ALM_UNWANTED); /* 114ALM   */

        return (AGC_STATUS_OK);
    }

    ms = (uint16_t) ((ms - MS_WANTED_ONE) | MS_REJECT_OK);
    agc_erase_set_markstat(ms);

    ram->sxt[ram->qpret] = mk; /* VACSTOR  */
    ram->qpret++;

    if ((ms & MS_WANTED_MASK) != 0u)
    {
        return (AGC_STATUS_OK); /* TASKOVER */
    }

    return (agc_exec_novac(ALIGN_PRIO32, AGC_JOB_MARK_COMPLETE_DISPLAY));
}

/**********************************************************************
 * MKREJECT / MRKREJCT / REJECT2 / REJECT3 (pp. 226-227): R21: MRKBUF1 time
 * = -1 flags a rejected mark.  Otherwise alarm 112 if no marks are being
 * taken, 110 for two rejects in a row; else the last slot is freed, one
 * more mark is wanted, and V51 is shown again if all had been in.
 *
 * FUNCTION NAME: agc_sxtmark_mkreject
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
static agc_status_t agc_sxtmark_mkreject(void)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    if (agc_flag_test(AGC_FLAG_MARKRUPT_OPTION_ONE))
    {
        agc_erase_set_mrkbuf(1, 0, align_sp_to_word(-1));

        return (AGC_STATUS_OK);
    }

    const uint16_t ms = agc_erase_markstat(); /* MARKSTAT */

    if (ms == 0u)
    {
        agc_alarm(ALM_NO_REJECT);

        return (AGC_STATUS_OK);
    }

    agc_erase_set_markstat((uint16_t) (ms & ~MS_REJECT_OK));

    if ((ms & MS_REJECT_OK) == 0u)
    {
        agc_alarm(ALM_REJECT2);

        return (AGC_STATUS_OK);
    }

    if (!AGC_ASSERT(ram->qpret > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    ram->qpret--; /* REJECT3  */
    agc_erase_set_markstat((uint16_t) ((ms & ~MS_REJECT_OK) + MS_WANTED_ONE));

    if ((ms & MS_WANTED_MASK) != 0u)
    {
        return (AGC_STATUS_OK); /* RESUME   */
    }

    return (agc_exec_novac(ALIGN_PRIO32, AGC_JOB_MARK_REQUEST_DISPLAY));
} /* agc_sxtmark_mkreject */

/**********************************************************************
 * MKVB51 (p. 227): KLEENEX (clear extended verbs), then the V51 flash.
 *
 * FUNCTION NAME: agc_job_mark_request_display
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
agc_status_t agc_job_mark_request_display(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_KLEENEX, 0, AGC_JOB_MARK_REQUEST_FLASH, AGC_JOB_MARK_REQUEST_FLASH, AGC_JOB_MARK_REQUEST_FLASH));
}

/**********************************************************************
 * AGC: GOMARK4 (Comanche055 DISPLAY_INTERFACE_ROUTINES.agc p. 1466)
 *
 * GOMARK4 with VB51 or V50N25: V34 TERMSXT, V33 ENTANSWR, ENTER MKVB5X.
 *
 * FUNCTION NAME: mark_flash
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
static agc_status_t mark_flash(const agc_job_ctx_t *ctx, uint16_t vn)
{
    return (align_flash(ctx, AGC_DSKY_GOMARK4, vn, AGC_JOB_TERMSXT, AGC_JOB_ENTANSWR, AGC_JOB_MARK_REDISPLAY));
}

/**********************************************************************
 * AGC: MKVB51 (Comanche055 SXTMARK.agc p. 232)
 *
 * MKVB51 +2: please mark (V51).
 *
 * FUNCTION NAME: agc_job_mark_request_flash
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
agc_status_t agc_job_mark_request_flash(agc_job_ctx_t *ctx)
{
    return (mark_flash(ctx, VN_PLEASE_MARK));
}

/**********************************************************************
 * AGC: MKVB50 (Comanche055 SXTMARK.agc p. 233)
 *
 * MKVB50: R1 = 00016, V50N25 (all marks in), without KLEENEX.
 *
 * FUNCTION NAME: agc_job_mark_complete_display
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
agc_status_t agc_job_mark_complete_display(agc_job_ctx_t *ctx)
{
    agc_dsky_set_dsptem(0, CHK_MARKS_DONE);

    return (mark_flash(ctx, VN_MARKS_DONE));
}

/**********************************************************************
 * AGC: MKVB5X (Comanche055 SXTMARK.agc p. 233)
 *
 * MKVB5X (ENTER): V51 again if marks are still wanted, else V50N25.
 *
 * FUNCTION NAME: agc_job_mark_redisplay
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
agc_status_t agc_job_mark_redisplay(agc_job_ctx_t *ctx)
{
    if ((agc_erase_markstat() & MS_WANTED_MASK) != 0u)
    {
        return (agc_job_mark_request_display(ctx));
    }

    return (agc_job_mark_complete_display(ctx));
}

/**********************************************************************
 * TERMSXT (V34, p. 228): CLEARMRK; in P03 set TRM03FLG and finish as
 * ENTANSWR, otherwise TERM52 (end the alignment optics, P00).
 *
 * FUNCTION NAME: agc_job_termsxt
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
agc_status_t agc_job_termsxt(agc_job_ctx_t *ctx)
{
    agc_dsky_clearmrk();

    if (!agc_phase_checkmm(3))
    {
        return (agc_job_auto_optics_positioning_terminate(ctx)); /* TERM52   */
    }

    agc_flag_set(AGC_FLAG_TRM03FLG); /* TERMP03  */

    return (agc_job_entanswr());
}

/**********************************************************************
 * ENTANSWR (V33, p. 228): MARKSTAT keeps only the VAC area, QPRET the
 * number of marks made; ENDMARKS in 50 ms gives the OPTSTALL good end;
 * ENDMARK clears the mark display flag and ends the job.
 *
 * FUNCTION NAME: agc_job_entanswr
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_entanswr(void)
{
    agc_erase_set_markstat((uint16_t) (agc_erase_markstat() & MS_VAC_MASK));
    agc_inhint();
    agc_status_t st = agc_waitlist_call(ENDMARKS_DT, AGC_TASK_ENDMARKS); /* status of the call */

    agc_relint();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_dsky_endmark(); /* ENDMARK  */

    return (st);
}

/**********************************************************************
 * AGC: ENDMARKS (Comanche055 SXTMARK.agc p. 233)
 *
 * ENDMARKS task: GOODEND for the optics (OPTSTALL).
 *
 * FUNCTION NAME: agc_task_endmarks
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_task_endmarks(void)
{
    return (agc_imu_goodend(AGC_IMU_DEV_OPTICS));
}

/**********************************************************************
 * AGC: none (QPRET accessor, SXTMARK.agc)
 *
 * QPRET after ENTANSWR: number of marks made.
 *
 * FUNCTION NAME: agc_sxtmark_marks_made
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- QPRET: the number of marks made (after ENTANSWR)
 **********************************************************************/
agc_sp_t agc_sxtmark_marks_made(void)
{
    const agc_sp_t n = agc_align_ram()->qpret; /* QPRET (unscaled count) */

    return (n);
}

/**********************************************************************
 * AGC: none (VAC area accessor; SXTMARK.agc VACSTOR layout)
 *
 * One stored mark (slot 0 = the first); the slot comes from the caller.
 *
 * FUNCTION NAME: agc_sxtmark_mark
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ------------------------------
 *   mark      int                     I    mark slot, 0 = first
 *   out       agc_align_sxt_mark_t *  O    the mark
 *
 * RETURNS: none
 **********************************************************************/
void agc_sxtmark_mark(int mark, agc_align_sxt_mark_t *out)
{
    *out = agc_align_ram()->sxt[mark];
}
