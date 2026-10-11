/*
 * extended_verbs_csm.c -- Comanche055 EXTENDED_VERBS.agc (pp. 235-262),
 * the verbs that exist only in the CMC or differ from the LGC's:
 *   V40/V41 N91 optics CDUs, V43 IMU attitude error meters, V44/V45
 *   surface flag, V46 establish G&N autopilot, V47 LM state vector into
 *   CM, V48 load autopilot data (R03), V51/V53/V59 please-perform, V52 P22
 *   offset, V54/V57 R23/R21 marking, V56 terminate tracking, V58 enable
 *   R61 maneuvers, V60-V63 attitude error displays, V65 optical
 *   verification, V68 stroke test, V75 liftoff flag, V76/V77 preferred
 *   attitude, V78 gyrocompass azimuth, V80/V81 vector to update, V83/V85
 *   rendezvous displays (R31/R34), V86 R23 mark reject, V87/V88 VHF range,
 *   V92 IMU performance test, V93 W matrix reset, V94 R64, V96 quit
 *   integration.  Spares and illegal nouns end in ALM/END.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/csm/dap.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/nav/integration.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

#define WORD15                077777u
#define BIT4                  000010u
#define BIT5                  000020u
#define BIT14                 020000u
#define OCT30                 000030u /* CHAN12 bits 4,5: coarse align, zero ICDU   */
#define OCT50                 000050u /* CHAN12 bits 4,6: coarse align, ECTR enable */
#define OCT60000              060000u /* FLAGWRD6 DAPBITS 1 and 2                   */
#define TVC_BITS              040000u /* DAPBITS = 10: TVC autopilot               */
#define PRIO30BITS            030000u /* DAPDATR1 configuration bits 13,12         */
#define PRIO7BITS             007000u
#define PRIO5BITS             005000u
#define NOUN_OCDU             91      /* N91 optics CDUs                            */
#define ALARM_NO_REND_TRACK   ((agc_alarm_code_t) 00406)
#define ALARM_NO_SYSTEMS_TEST ((agc_alarm_code_t) 01521)
#define ALARM_OPTICS_SWITCH \
    ((agc_alarm_code_t) 00115)        /* optics not at computer */
#define ALARM_OPTICS_RESERVED \
    ((agc_alarm_code_t) 00117)        /* OPTIND = -0          */
#define MRKBUF_REJECT  077776u        /* NEGONE                             */
#define PHASE_R64      0112u          /* group 2: do R64                     */
#define PHASE_AZIMUTH  0174u          /* group 4 phase 15                    */
#define PRIO5          ((agc_prio_t) 005)
#define PRIO10         ((agc_prio_t) 010)
#define PRIO16         ((agc_prio_t) 016)
#define PRIO20         ((agc_prio_t) 020)
#define PRIO30         ((agc_prio_t) 030)
#define ATTCK_DELAY_CS 2u             /* CAF TWO, WAITLIST                  */

/* FLAGWRD6 DAPBITS = 10: the TVC autopilot is on.                         */
static bool tvc_on(void)
{
    uint16_t f6 = agc_flagword_get(6);

    if (!AGC_ASSERT(f6 <= WORD15))
    {
        return (false);
    }

    return ((f6 & OCT60000) == TVC_BITS);
}

/* CKLFTBTS: true after liftoff (GRRBKFLG, or channel 30 bit 5 clear).    */
static bool after_liftoff(void)
{
    if (agc_flag_test((agc_flag_id_t) AGC_FLAG_GRRBKFLG))
    {
        return (true);
    }

    uint16_t c30 = agc_chan_read((int) AGC_CHAN_CHAN30);

    if (!AGC_ASSERT(c30 <= WORD15))
    {
        return (false);
    }

    return ((c30 & BIT5) == 0u);
}

/* IMUATTCK (V43, p.243): P00 only, not while coarse align or zero ICDU;
 * after liftoff the extended-verb display must be free.                   */
static agc_status_t verb_attitude_error(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || !agc_extverb_chkpooh())
    {
        return (AGC_STATUS_OK);
    }

    if ((agc_chan_read((int) AGC_CHAN_CHAN12) & OCT30) != 0u)
    {
        return (agc_extverb_alm_end());
    }

    if (after_liftoff())
    {
        bool         ok = false;
        agc_status_t st = agc_extverb_testxact(&ok);

        if (!ok)
        {
            return (st);
        }
    }

    agc_chan_and((int) AGC_CHAN_CHAN12, (uint16_t) (WORD15 & ~OCT50));
    agc_dsky_cont_t cont = {AGC_JOB_ATTITUDE_ERROR_TERMINATE,
                            AGC_JOB_ATTITUDE_ERROR_LOAD,
                            AGC_JOB_ATTITUDE_ERROR_LOAD};

    return (agc_dsky_display(ctx, AGC_DSKY_GOMARKF, agc_dsky_vn(25, 22), &cont));
}

/* TRMATTCK: after liftoff ENDEXT, before liftoff GOPIN.                   */
agc_status_t agc_job_attitude_error_terminate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (after_liftoff() ? agc_dsky_endmark() : agc_extverb_gopin());
}

/* V43 after the load: NEEDLE11 / NEEDLER2 (initialise the DACs, enable
 * the error counters), ATTCK1 in 20 ms, then TRMATTCK.                   */
agc_status_t agc_job_attitude_error_load_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_dap_needler(0, 0, 0, true);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_waitlist_call(ATTCK_DELAY_CS, AGC_TASK_ATTITUDE_ERROR_OUTPUT);

    if (st != AGC_STATUS_OK)
    {
        return (st); /* WAITLIST did its alarm  */
    }

    return (agc_job_attitude_error_terminate(ctx));
}

/* ATTCK1 (task): AK, AK1, AK2 = THETAD; NEEDLES sends them (limited to
 * +-384 pulses, error counters left enabled).                             */
static agc_status_t attck1(void)
{
    agc_sp_t ak[3];

    for (int i = 0; i < 3; i++)
    {
        /* Signed value of a two's-complement CDU word (half-revolutions, 2^14). */
        uint16_t w = agc_erase_thetad(i);

        if (!AGC_ASSERT(w <= WORD15))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        int v = ((w & 040000u) != 0u) ? (int) w - 0100000 : (int) w;

        if (!AGC_ASSERT(v >= -040000 && v < 040000))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        ak[i] = (agc_sp_t) ((v < AGC_SP_MIN) ? AGC_SP_MIN : v);
    }

    agc_status_t st = agc_dap_needler(ak[0], ak[1], ak[2], false);

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* STABLISH (V46, p.245): RCS/entry/Saturn DAP by DAPFIG; with TVC on,
 * only the CSM/LM switch-over to low bandwidth (else ALM/END).            */
static agc_status_t verb_establish_dap(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!tvc_on())
    {
        return (agc_job_dapfig(ctx)); /* POSTJUMP DAPFIG         */
    }

    if ((agc_erase_dapdatr1() & BIT14) == 0u)
    {
        return (agc_extverb_alm_end()); /* CSM alone               */
    }

    agc_status_t st = agc_csm_tvc_preswtch();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_extverb_gopin()); /* PRESWTCH exits via PINBRNCH */
}

/* ---- V48 load autopilot data (R03, DAPDISP pp.246-247) ----------------- */

/* GOXDSPF of a V04/V06 display with proceed -> next, enter -> again.     */
static agc_status_t dap_data_display(agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun, agc_job_id_t next, agc_job_id_t again)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dsky_cont_t cont = {AGC_JOB_EXTENDED_VERB_END, next, again};

    return (agc_dsky_display(ctx, AGC_DSKY_GOMARKF, agc_dsky_vn(verb, noun), &cont));
}

/* DAPDISP: not with TVC; TESTXACT; priority 10; V04N46.                   */
static agc_status_t verb_dap_data_load(agc_job_ctx_t *ctx)
{
    bool ok = false;

    if (!AGC_ASSERT(ctx != NULL) || tvc_on())
    {
        return (agc_extverb_alm_end());
    }

    agc_status_t st = agc_extverb_testxact(&ok);

    if (!ok)
    {
        return (st);
    }

    agc_exec_prio_change(PRIO10);

    return (agc_job_dap_data_load_noun46_csm(ctx));
}

/* DONOUN46: V04N46 DAPDATR1, DAPDATR2.                                     */
agc_status_t agc_job_dap_data_load_noun46_csm(agc_job_ctx_t *ctx)
{
    return (dap_data_display(ctx, 4, 46, AGC_JOB_DAP_DATA_LOAD_CONFIG, AGC_JOB_DAP_DATA_LOAD_NOUN46));
}

/* N46 proceed: MAXDBFLG from DAPDATR1 bit 4, S41.2, then V06N47.          */
agc_status_t agc_job_dap_data_load_config_csm(agc_job_ctx_t *ctx)
{
    agc_flag_id_t f = (agc_flag_id_t) AGC_FLAG_MAXDBFLG;

    agc_flag_put(f, (agc_erase_dapdatr1() & BIT4) != 0u);
    agc_status_t st = agc_csm_dap_decode_dap_data(); /* S41.2 (MAXOUT)          */

    if (!AGC_ASSERT(ctx != NULL) || st != AGC_STATUS_OK)
    {
        return ((st != AGC_STATUS_OK) ? st : AGC_STATUS_BAD_ARG);
    }

    return (agc_job_dap_data_load_noun47_csm(ctx));
}

/* DONOUN47: V06N47 CSM and LM weights.                                     */
agc_status_t agc_job_dap_data_load_noun47_csm(agc_job_ctx_t *ctx)
{
    return (dap_data_display(ctx, 6, 47, AGC_JOB_DAP_DATA_LOAD_MASS, AGC_JOB_DAP_DATA_LOAD_NOUN47));
}

/* N47 proceed: MASSPROP and S40.14 only for configurations 1, 2, 6
 * (DAPDATR1 bits 13,12 neither 00 nor 11); then V06N48.                    */
agc_status_t agc_job_dap_data_load_mass_csm(agc_job_ctx_t *ctx)
{
    uint16_t cfg = (uint16_t) (agc_erase_dapdatr1() & PRIO30BITS);

    if (cfg != 0u && cfg != PRIO30BITS)
    {
        agc_inhint();
        agc_csm_dap_massprop(); /* IXX, IAVG, IAVG/TLX */
        agc_status_t st = AGC_STATUS_OK;

        agc_relint();
        st = agc_csm_dap_rcs_gains(); /* S40.14              */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_dap_data_load_noun48_csm(ctx));
}

/* DONOUN48: V06N48 pitch and yaw trim; proceed or terminate end R03.      */
agc_status_t agc_job_dap_data_load_noun48_csm(agc_job_ctx_t *ctx)
{
    return (dap_data_display(ctx, 6, 48, AGC_JOB_EXTENDED_VERB_END, AGC_JOB_DAP_DATA_LOAD_NOUN48));
}

/* ---- marking and tracking ---------------------------------------------- */

/* GOTOR21 / GOTOR23 (V57 / V54, p.250): R23FLG, then R21 or R23 if P20
 * and tracking are on (else alarm 406 and ENDEXT).                         */
static agc_status_t verb_marking(bool backup)
{
    agc_flag_id_t r23 = (agc_flag_id_t) AGC_FLAG_BACKUP_MARKING;

    agc_flag_put(r23, backup);
    bool         ok = false;
    agc_status_t st = agc_extverb_testxact(&ok);

    if (!ok)
    {
        return (st);
    }

    if (!agc_flag_test((agc_flag_id_t) AGC_FLAG_RNDVZFLG) || !agc_flag_test((agc_flag_id_t) AGC_FLAG_TRACKFLG))
    {
        agc_alarm(ALARM_NO_REND_TRACK); /* R22ALARM           */

        return (agc_dsky_endmark());
    }

    return (agc_extverb_schedule(PRIO16, backup ? AGC_JOB_BACKUP_SIGHTING_MARK : AGC_JOB_RENDEZVOUS_SIGHTING_MARK_CSM, false));
}

/* TRACKTRM (V56, p.251): knock down the rendezvous, track, update and
 * IMUSE flags; then (job) stop the rates, release the optics, wait for
 * integration, kill groups 2 and 1 and restart (ENEMA).                    */
static agc_status_t verb_terminate_tracking(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || !agc_flag_test((agc_flag_id_t) AGC_FLAG_RNDVZFLG))
    {
        return (agc_extverb_gopin());
    }

    agc_flag_clear((agc_flag_id_t) AGC_FLAG_RNDVZFLG);

    if (!agc_flag_test((agc_flag_id_t) AGC_FLAG_TRACKFLG))
    {
        return (agc_extverb_gopin());
    }

    agc_flag_clear((agc_flag_id_t) AGC_FLAG_TRACKFLG);
    agc_flag_clear((agc_flag_id_t) AGC_FLAG_UPDATFLG);
    agc_flag_clear((agc_flag_id_t) AGC_FLAG_IMUSE);
    agc_inhint();
    agc_kalc_stoprate();      /* STOPRATE           */
    agc_erase_set_optind(-1); /* OPTIND = NEGONE    */
    agc_relint();

    return (agc_job_tracking_terminate_csm(ctx));
}

/* V56 after INTSTALL: kill groups 2 and 1, CLEANOUT (ENEMA).              */
agc_status_t agc_job_tracking_terminate_csm(agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_integ_install(0, AGC_JOB_TRACKING_TERMINATE);

    if (!AGC_ASSERT(ctx != NULL) || st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* sleeps in INTSTALL */
    }

    agc_phase_change(2, 02); /* 2PHSCHNG OCT 2, 1  */
    agc_phase_change(1, 01);
    agc_inhint();            /* CLEANOUT           */

    return (agc_exec_enema());
}

/* ---- small verbs --------------------------------------------------------- */

/* V52 (p.257): in P22, LANDMARK bits 12-10 = 5 - the mark number in
 * MARKSTAT (the shifted bits 14-12), the offset number.                    */
static agc_status_t verb_landmark_offset(void)
{
    if (!agc_phase_checkmm(22))
    {
        return (agc_extverb_alm_end());
    }

    int lm   = (int) agc_erase_landmark() & ~(int) PRIO7BITS;   /* CS PRIO7 MASK */
    int mark = (int) ((agc_erase_markstat() >> 1) & PRIO7BITS); /* TS SR, CA SR */
    int v    = lm + ((int) PRIO5BITS - mark);                   /* CS A, AD PRIO5, ADS */

    if (!AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_erase_set_landmark((agc_sp_t) v);

    return (agc_extverb_gopin());
}

/* V60: astronaut total attitude (N17, CPHIX) = present CDUs.              */
static agc_status_t verb_attitude_to_cdus(void)
{
    agc_csm_dapcom_t *c = agc_csm_dap_com();

    if (!AGC_ASSERT(c != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    c->cphix[0] = agc_erase_cdux();
    c->cphix[1] = agc_erase_cduy();
    c->cphix[2] = agc_erase_cduz();

    return (agc_extverb_gopin());
}

/* V62 / V63: NEEDLFLG set (total attitude error), N22ORN17 set (wrt N22)
 * or clear (wrt N17).                                                      */
static agc_status_t verb_total_error(bool wrt_n22)
{
    agc_flag_set((agc_flag_id_t) AGC_FLAG_NEEDLFLG);
    agc_flag_id_t f = (agc_flag_id_t) AGC_FLAG_ATTITUDE_ERROR_DESIRED_ANGLES;

    return (wrt_n22 ? agc_extverb_upflag(f) : agc_extverb_downflag(f));
}

/* CKOPTVB (V65) / CHAZFOGC (V78): only in P02 (prelaunch).               */
static agc_status_t verb_prelaunch(bool azimuth)
{
    if (!agc_phase_checkmm(2))
    {
        return (agc_extverb_alm_end());
    }

    agc_status_t st = agc_extverb_schedule(PRIO16, azimuth ? AGC_JOB_GYROCOMPASS_AZIMUTH_CHANGE : AGC_JOB_COMPVER, true);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (azimuth)
    {
        agc_phase_change(4, PHASE_AZIMUTH);
    }

    return (agc_extverb_gopin());
}

/* STROKON (V68): TVC only; NOVAC STRKTSTI at priority 30.                 */
static agc_status_t verb_stroke_test(void)
{
    if (!tvc_on())
    {
        return (agc_extverb_alm_end());
    }

    agc_status_t st = agc_extverb_schedule(PRIO30, AGC_JOB_STRKTSTI, false);

    return ((st == AGC_STATUS_OK) ? agc_extverb_gopin() : st);
}

/* V83PERF / V85PERF: R31 (R31FLAG set) or R34 (clear), NOVAC priority 5.  */
static agc_status_t verb_rendezvous_display(bool r31)
{
    bool         ok = false;
    agc_status_t st = agc_extverb_testxact(&ok);

    if (!ok)
    {
        return (st);
    }

    agc_flag_id_t f = (agc_flag_id_t) AGC_FLAG_RENDEZVOUS_DISPLAY_THETA;

    agc_flag_put(f, r31);

    return (agc_extverb_schedule(PRIO5, AGC_JOB_RENDEZVOUS_PARAMETER_DISPLAY, false));
}

/* V86PERF: R23 mark reject (MRKBUF1 = -1).                                 */
static agc_status_t verb_mark_reject(void)
{
    agc_erase_set_mrkbuf(1, 0, MRKBUF_REJECT);

    if (!AGC_ASSERT(agc_erase_mrkbuf(1, 0) == MRKBUF_REJECT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_extverb_gopin());
}

/* SYSTEST (V92, p.240): P00, not after P11 (NODOP01: POODOO 1521); turn
 * the DAP off (EXDAPOFF) and FINDVAC REDO at priority 20.                  */
static agc_status_t verb_systems_test(void)
{
    if (!agc_extverb_chkpooh())
    {
        return (AGC_STATUS_OK);
    }

    if (agc_flag_test((agc_flag_id_t) AGC_FLAG_NO_PRELAUNCH_INITIALIZATION))
    {
        return (agc_poodoo(ALARM_NO_SYSTEMS_TEST));
    }

    agc_rupt_set_t5_task(AGC_TASK_T5IDLOC); /* EXDAPOFF */
    agc_flagword_set(6, (uint16_t) (agc_flagword_get(6) & ~OCT60000));
    agc_status_t st = agc_extverb_schedule(PRIO20, AGC_JOB_SYSTEMS_TEST, true);

    return ((st == AGC_STATUS_OK) ? agc_extverb_gopin() : st);
}

/* VERB94: R64 in P23 if V94FLAG allows it (restart via group 2).          */
static agc_status_t verb_r64(void)
{
    agc_flag_id_t f = (agc_flag_id_t) AGC_FLAG_ALTERNATE_LOS_ALLOWED;

    if (!agc_flag_test(f))
    {
        return (agc_extverb_alm_end());
    }

    agc_flag_clear(f);

    if (!agc_phase_checkmm(23))
    {
        return (agc_extverb_alm_end());
    }

    agc_phase_change(2, PHASE_R64);
    agc_inhint(); /* CLEANOUT           */

    return (agc_exec_enema());
}

/* VERB96: QUITFLAG, V96ONFLG, then V37 to P00.                             */
static agc_status_t verb_quit_integration(void)
{
    agc_flag_set((agc_flag_id_t) AGC_FLAG_QUITFLAG);
    agc_flag_set((agc_flag_id_t) AGC_FLAG_IDLE_INTEGRATION_INHIBITED);

    return (agc_change_program(0));
}

/* OPTCOARK (V41 N91, p.242): "temporary routine to run the optics CDUs
 * from the keyboard".  MODECADR (optics) must be free; optics switch not
 * at computer -> OPR ERR and alarm 115 (and carry on); OPTIND = -0
 * (optics reserved) -> alarm 117, ENDEXT; else load V24N92.              */
static agc_status_t verb_optics_coarse(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(agc_extverb_noun() == NOUN_OCDU))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_erase_modecadr((int) AGC_IMU_DEV_OPTICS) != 0)
    {
        return (agc_extverb_alm_end()); /* CKMODCAD +1            */
    }

    bool         ok = false;
    agc_status_t st = agc_extverb_testxact(&ok);

    if (!ok)
    {
        return (st);
    }

    if (agc_erase_swsample() <= 0)
    { /* not at computer        */
        agc_dsky_falton();
        st              = AGC_STATUS_OK;
        agc_alarm(ALARM_OPTICS_SWITCH);
        agc_status_t sa = AGC_STATUS_ALARM;

        if (!AGC_ASSERT(sa != AGC_STATUS_BAD_ARG))
        {
            return (AGC_STATUS_BAD_ARG);
        }
    }

    if (agc_erase_optind() == AGC_OPTIND_NEG0)
    {
        agc_alarm(ALARM_OPTICS_RESERVED);

        return (agc_dsky_endmark());
    }

    return (agc_extverb_flash(ctx, 24, 92, AGC_JOB_OPTICS_COARSE_DRIVE, AGC_JOB_OPTICS_COARSE_DRIVE)); /* OPTC1        */
}

/* OPTC1 after the load (V33 or ENTER): DESOPTS = SAC, DESOPTT = PAC; show
 * V41 again.                                                              */
agc_status_t agc_job_optics_coarse_drive(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ar = agc_align_ram();

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(ar != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    ar->desopts = ar->sac;
    ar->desoptt = ar->pac;

    return (agc_extverb_redisplay(ctx, 41, AGC_JOB_OPTICS_COARSE_START));
}

/* OPTC1 after EXDSPRET: OPTIND = 1 (coarse optics working; T4RUPT's
 * OPTDRIVE takes it from here), ENDEXT.                                   */
agc_status_t agc_job_optics_coarse_start(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_erase_set_optind(1);

    if (!AGC_ASSERT(agc_erase_optind() == 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_endmark());
}

/* Verbs 40-47.                                                             */
static agc_status_t fan_40s(agc_job_ctx_t *ctx, int verb)
{
    switch (verb)
    {
        case 40:
            return (agc_extverb_alm_end()); /* N91 "not in use yet"    */
        case 41:
            return ((agc_extverb_noun() == NOUN_OCDU) ? verb_optics_coarse(ctx) : agc_extverb_alm_end());
        case 43:
            return (verb_attitude_error(ctx));
        case 44:
            return (agc_extverb_upflag((agc_flag_id_t) AGC_FLAG_SURFFLAG));
        case 45:
            return (agc_extverb_downflag((agc_flag_id_t) AGC_FLAG_SURFFLAG));
        case 46:
            return (verb_establish_dap(ctx));
        case 47:
            return (agc_extverb_state_vector_copy(true));
        case 48:
            return (verb_dap_data_load(ctx));
        default:
            break;
    }

    if (!AGC_ASSERT(verb >= 40 && verb < 50))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_extverb_alm_end());
}

/* Verbs 51-68.                                                             */
static agc_status_t fan_50s_60s(agc_job_ctx_t *ctx, int verb)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    switch (verb)
    {
        case 51:
        case 53:
        case 59:
            return (agc_extverb_goloadlv());
        case 52:
            return (verb_landmark_offset());
        case 54:
            return (verb_marking(true));
        case 56:
            return (verb_terminate_tracking(ctx));
        case 57:
            return (verb_marking(false));
        case 58:
            return (agc_extverb_downflag((agc_flag_id_t) AGC_FLAG_STIKFLAG));
        case 60:
            return (verb_attitude_to_cdus());
        case 61:
            return (agc_extverb_downflag((agc_flag_id_t) AGC_FLAG_NEEDLFLG));
        case 62:
            return (verb_total_error(true));
        case 63:
            return (verb_total_error(false));
        case 65:
            return (verb_prelaunch(false));
        case 68:
            return (verb_stroke_test());
        default:
            break;
    }

    return (agc_extverb_alm_end());
}

/* Verbs 75-98.                                                             */
static agc_status_t fan_70s_90s(int verb)
{
    switch (verb)
    {
        case 75:
            return (agc_extverb_upflag((agc_flag_id_t) AGC_FLAG_GRRBKFLG));
        case 76:
            return (agc_extverb_upflag((agc_flag_id_t) AGC_FLAG_PRFTRKAT));
        case 77:
            return (agc_extverb_downflag((agc_flag_id_t) AGC_FLAG_PRFTRKAT));
        case 78:
            return (verb_prelaunch(true));
        case 80:
            return (agc_extverb_downflag((agc_flag_id_t) AGC_FLAG_VEHUPFLG));
        case 81:
            return (agc_extverb_upflag((agc_flag_id_t) AGC_FLAG_VEHUPFLG));
        case 83:
            return (verb_rendezvous_display(true));
        case 85:
            return (verb_rendezvous_display(false));
        case 86:
            return (verb_mark_reject());
        case 87:
            return (agc_extverb_upflag((agc_flag_id_t) AGC_FLAG_VHFRFLAG));
        case 88:
            /* V88: VHFRFLAG clear, tracker fail light off.                             */
        {
            agc_flag_clear((agc_flag_id_t) AGC_FLAG_VHFRFLAG);
            agc_status_t st = agc_svc_trfailof();

            if (!AGC_ASSERT(st == AGC_STATUS_OK))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            return (agc_extverb_gopin());
        }
        case 92:
            return (verb_systems_test());
        case 93:
            /* V93 (WMATRXNG): reset RENDWFLG and ORBWFLAG.                             */
        {
            agc_flag_clear((agc_flag_id_t) AGC_FLAG_RENDWFLG);

            return (agc_extverb_downflag((agc_flag_id_t) AGC_FLAG_ORBWFLAG));
        }
        case 94:
            return (verb_r64());
        case 96:
            return (verb_quit_integration());
        default:
            break;
    }

    if (!AGC_ASSERT(verb >= 70 && verb <= 99))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_extverb_alm_end()); /* V79, V84, V95, V98     */
}

/* LST2FAN, CMC entries not shared with the LGC.                            */
agc_status_t agc_extverb_csm(agc_job_ctx_t *ctx, int verb)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(verb >= 40 && verb <= 99))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (verb < 50)
    {
        return (fan_40s(ctx, verb));
    }

    if (verb < 70)
    {
        return (fan_50s_60s(ctx, verb));
    }

    return (fan_70s_90s(verb));
}

/* Tasks of the extended verbs (dispatch).                                 */
agc_status_t agc_task_attitude_error_output(void)
{
    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        return (agc_extverb_lm_attck2());
    }

    return (attck1());
}
