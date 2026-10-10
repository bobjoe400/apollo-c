/*
 * lunar_landing.c -- the landing program flow.
 *
 *   THE_LUNAR_LANDING.agc (Luminary 099, pp. 785-792): P63 braking phase
 *     start, the ignition algorithm IGNALG, R51/R60 before ignition, and
 *     P68 landing confirmation.
 *   LUNAR_LANDING_GUIDANCE_EQUATIONS.agc (pp. 800-822): GUILDENSTERN (R13,
 *     the auto-modes monitor), the P66/P67 starts, the landing site
 *     redesignation equations, P66 rate-of-descent guidance (RODCOMP), P67,
 *     the redesignator trap (PITFALL/REDESMON) and the R.O.D. trap.
 *
 * Jobs that wait for the crew, the integration lock or another routine
 * are split at those points; each piece is a job function and the AGC's
 * return address becomes the continuation job id (TRANSLATION_GUIDE s.4).
 */
#include "agc/agc_assert.h"
#include "agc/agc_channels.h"
#include "agc/agc_lm_landing.h"
#include "landing_pending.h"
#include "../io/sysmisc.h"

/* ---- constants (pp. 789, 820-822) -------------------------------------- */
#define LL_DPSTHRSH  36          /* DPSTHRSH: THRESH1 + THRESH3 for P63     */
#define LL_ASTNDEX   025         /* ASTNDEX = MD1: CLOKTASK display index   */
#define LL_CODE500   00500       /* CODE500: crank the LR antenna           */
#define LL_99999CON  ((agc_dp_t)487675)  /* 2DEC 30479.7 B-24: 99999 ft     */
#define LL_GUIDDURN  ((agc_dp_t)66440)   /* 2DEC +66440: 664.4 s, cs 2^28   */
#define LL_DDUMCRIT  ((agc_dp_t)8)       /* 2DEC +8 B-28: IGNALG converged  */
#define LL_NIGNLOOP  40          /* SSP NIGNLOOP 40D                        */
#define LL_BIASFACT  ((agc_dp_t)655)     /* 2DEC 655.36 B-28                */
#define LL_KPIP1     ((agc_dp_t)3435974) /* 2DEC .0128: DELV to 2^7 m/cs    */
#define LL_4SEC28    ((agc_dp_t)400)     /* 2DEC 400 B-28                   */
#define LL_GSCALE    ((agc_dp_t)13107200)/* 2DEC 100 B-11                   */
#define LL_SHFTFACT  ((agc_dp_t)2048)    /* 2DEC 1 B-17                     */
#define LL_BIT1H     ((agc_dp_t)16384)   /* DMP BIT1H: DP (OCT 1, 0)        */
#define LL_SCALEFAC  ((agc_dp_t)3268444) /* +7.97959872 E+2 B-16            */
#define LL_DEPRCRIT  ((agc_dp_t)-2684355)/* 2DEC -.02 B-1                   */
#define LL_ALL4BITS  00063       /* ALL4BITS: channel 31 bits 1, 2, 5, 6    */
#define LL_AZEACH    572         /* DEC .03491: 2 degrees                   */
#define LL_ELEACH    143         /* DEC .00873: 1/2 degree                  */
#define LL_PLUSELBIT 00002       /* +ELBIT = BIT2 (-pitch)                  */
#define LL_MINUSELBIT 00001      /* -ELBIT = BIT1 (+pitch)                  */
#define LL_PLUSAZBIT 00020       /* +AZBIT = BIT5                           */
#define LL_MINUSAZBIT 00040      /* -AZBIT = BIT6                           */
#define LL_1SEC      100
#define LL_PRIO22    022
#define LL_IGN_MAX_LOOPS 64      /* bound on IGNALOOP (AGC: none)           */

/* PHASCHNG OCT 04024: group 4 restarts at this point of this job. */
static void ll_restart_here(agc_job_id_t job)
{
    const agc_phase_var_t v = { AGC_RESTART_JOB, job, AGC_TASK_NONE, 0,
                                false, 0U };
    agc_status_t st = agc_phase_change_var(04024, &v);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    (void)AGC_ASSERT(job != AGC_JOB_NONE);
}

/* The crew display continuations shared by this file. */
static agc_status_t ll_display(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind,
                               uint16_t vn, agc_job_id_t proceed,
                               agc_job_id_t enter)
{
    const agc_dsky_cont_t cont = { AGC_JOB_GOTOPOOH, proceed, enter };
    (void)AGC_ASSERT(ctx != NULL);
    (void)AGC_ASSERT(proceed != AGC_JOB_NONE && enter != AGC_JOB_NONE);
    return agc_dsky_display(ctx, kind, vn, &cont);
}

/* ======================================================================
 * P63 and the ignition algorithm (THE_LUNAR_LANDING, pp. 785-789)
 * ====================================================================== */

/* IGNALG, first part (pp. 785-786): the landing site in stable member
 * coordinates at TLAND, WM and /LAND/, and the LM state integrated to
 * TLAND - GUIDDURN (LEMPREC; may wait for the integration lock).       */
static agc_status_t ll_ignalg_prec(agc_job_ctx_t *ctx, agc_interp_t *ip);

static agc_status_t ll_ignalg_start(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_lm_ldg_t *l = agc_lm_ldg();
    const agc_lm_pad_t *p = agc_lm_pad();
    agc_navaux_padload_t nav;
    agc_status_t st = agc_navaux_get_padload(&nav);
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    agc_vec_t r = agc_vec_zero();
    st = agc_planet_rp_to_r(&ip, &nav, true, agc_erase_rls(), p->tland, &r);
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    l->tpip = p->tland;                  /* TPIP for first guidance pass  */
    agc_mat_t m = agc_erase_refsmmat();
    l->land = agc_vec_shl(&ip, agc_mat_vec(&ip, &m, r), 4);   /* 2^24 m */
    st = agc_lm_guidinit(&ip);           /* WM and /LAND/                 */
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return ll_ignalg_prec(ctx, &ip);
}

/* IGNALOOP (pp. 786-788) with DDUMCALC; on convergence DDUMGOOD/R60INIT
 * and IGNALGRT. from_install: resuming after INTSTALL was busy.        */
static agc_status_t ll_ignaloop(agc_job_ctx_t *ctx, bool from_install);

static agc_status_t ll_ignalg_prec(agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_ldg_t *l = agc_lm_ldg();
    agc_dp_t tdec1 = agc_dp_sub(ip, agc_lm_pad()->tland, LL_GUIDDURN);
    agc_status_t st = agc_integ_lemprec(ip, agc_integ_shared_ctx(), tdec1,
                                        AGC_JOB_IGNALGP);
    if (st == AGC_STATUS_BUSY) {
        return AGC_STATUS_OK;            /* asleep; resumes as IGNALGP    */
    }
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    l->nignloop = LL_NIGNLOOP;
    l->cg = agc_mat_identity_half();     /* UNITX, UNITY, UNITZ           */
    l->deltah = LL_99999CON;             /* for the V16N68 display        */
    l->unfc_2 = agc_vec_zero();          /* trim velocity correction term */
    l->ttf_8 = 0;
    return ll_ignaloop(ctx, false);
}

/* DDUMCALC (pp. 786-787): the ignition time correction
 *          (RIGNZ - RGU2)/16 + 16 RGU1^2 KIGNY/B8 + (RGU0 - RIGNX) KIGNX/B4
 *          + (|VGU| - VIGN) KIGNV/B4
 *   DDUM = ----------------------------------------------------------
 *                     2^10 (VGU2 - 16 VGU0 KIGNX/B4)
 * numerator 2^28 m, denominator 2^10 m/cs, quotient rescaled to 2^28 cs. */
static agc_dp_t ll_ddum(agc_interp_t *ip)
{
    const agc_lm_ldg_t *l = agc_lm_ldg();
    const agc_lm_pad_t *p = agc_lm_pad();
    agc_dp_t den = agc_dp_shl_round(ip, agc_dp_mul(ip, l->vgu.c[0], p->kignx_b4), 4);
    den = agc_dp_sub(ip, l->vgu.c[2], den);
    agc_dp_t a = agc_dp_shr_round(agc_dp_sub(ip, p->rignz, l->rgu.c[2]), 4);
    agc_dp_t b = agc_dp_shl_round(ip, agc_dp_mul(ip, agc_dp_sq(ip, l->rgu.c[1]),
                                                 p->kigny_b8), 4);
    agc_dp_t c = agc_dp_mul(ip, agc_dp_sub(ip, l->rgu.c[0], p->rignx), p->kignx_b4);
    agc_dp_t d = agc_dp_mul(ip, agc_dp_sub(ip, agc_vec_abval(ip, l->vgu), p->vign),
                            p->kignv_b4);
    agc_dp_t num = agc_dp_add(ip, agc_dp_add(ip, agc_dp_add(ip, d, c), b), a);
    agc_dp_t q = agc_dp_shr_round(agc_dp_div(ip, num, den), 10);
    (void)AGC_ASSERT(den != 0);
    return q;
}

/* DDUMGOOD and R60INIT (p. 788): TIG = TDEC1 - ZOOMTIME; the landing
 * site's distance out of the orbital plane (+ right/north); UNFC/2 kept
 * in R60VSAVE for the attitude maneuver.                               */
static void ll_ddumgood(agc_interp_t *ip, agc_dp_t tdec1)
{
    agc_lm_ldg_t *l = agc_lm_ldg();
    const agc_guid_lm_t *g = &agc_guid_state()->lm;
    agc_dp_t zt = (agc_dp_t)g->zoomtime;                  /* SLOAD, SR 14 */
    agc_erase_set_tig(agc_dp_sub(ip, tdec1, zt));
    agc_vec_t n = agc_vec_unit(ip, agc_vec_cross(ip, g->v, g->r), NULL, NULL);
    l->outofpln = agc_dp_shl(ip, agc_vec_dot(ip, n, l->land), 1);
    l->r60vsave = l->unfc_2;
    (void)AGC_ASSERT(zt >= 0);
    (void)AGC_ASSERT(tdec1 >= 0);
}

/* One IGNALOOP iteration up to DDUMCALC: state from the integration
 * output, gravity, three passes of quadratic guidance (?GUIDSUB, then
 * GUIDSUB while NGUIDSUB counts down), NIGNLOOP.                      */
static agc_status_t ll_ign_guidance(agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_ldg_t *l = agc_lm_ldg();
    agc_guid_t *g = agc_guid_state();
    const agc_integ_out_t *o = &agc_integ_shared_ctx()->out;
    g->avg.piptime1 = o->tat;
    agc_mat_t m = agc_erase_refsmmat();
    g->lm.r = agc_vec_shl(ip, agc_mat_vec(ip, &m, o->ratt1), 4);
    g->avg.gdt_2 = agc_lm_mungrav(ip, g->lm.r);
    l->nguidsub = 2;                                      /* N = 3        */
    agc_status_t st = agc_lm_guidpass(ctx, AGC_LM_ENTRY_QGUIDSUB);
    for (int n = 0; n < 2 && st == AGC_STATUS_OK && l->nguidsub > 0; n++) {
        l->nguidsub = (agc_sp_t)(l->nguidsub - 1);        /* CCS NGUIDSUB */
        st = agc_lm_guidpass(ctx, AGC_LM_ENTRY_GUIDSUB);
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if (l->nignloop > 0) {
        l->nignloop = (agc_sp_t)(l->nignloop - 1);        /* CCS NIGNLOOP */
    } else {
        (void)agc_alarm(AGC_ALARM_01412);  /* DEVIATION: NIGNLOOP stays 0 */
    }
    return AGC_STATUS_OK;
}

static agc_status_t ll_ignaloop(agc_job_ctx_t *ctx, bool from_install)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_integ_ctx_t *ic = agc_integ_shared_ctx();
    bool resume = from_install;
    for (int n = 0; n < LL_IGN_MAX_LOOPS; n++) {
        if (!resume) {
            agc_status_t st = ll_ign_guidance(ctx, &ip);
            if (st != AGC_STATUS_OK) {
                return st;
            }
            agc_dp_t ddum = ll_ddum(&ip);
            ic->tdec1 = agc_dp_add(&ip, ddum, agc_guid_state()->avg.piptime1);
            if (agc_dp_abs(ddum) < LL_DDUMCRIT) {
                ll_ddumgood(&ip, ic->tdec1);
                return agc_lm_job_ignalgrt(ctx);
            }
        }
        resume = false;
        agc_status_t st = agc_integ_install(0, AGC_JOB_IGNALOOP);
        if (st == AGC_STATUS_BUSY) {
            return AGC_STATUS_OK;            /* asleep; resumes IGNALOOP  */
        }
        (void)agc_flag_set(AGC_FLAG_INTYPFLG);
        (void)agc_flag_set(AGC_FLAG_MOONFLAG);
        ic->sv.tet = agc_guid_state()->avg.piptime1;
        ic->sv.rcv = ic->out.ratt1;          /* hopefully ?GUIDSUB did not */
        ic->sv.vcv = ic->out.vatt1;          /* clobber RATT1 and VATT1    */
        st = agc_integ_integrvs(&ip, ic, ic->tdec1);
        if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
            return st;
        }
    }
    return AGC_STATUS_NO_CONVERGE;           /* DEVIATION: loop bound      */
}

/* P63LM and FLAGORGY (p. 785): "Dionysian flag waving", then IGNALG. */
agc_status_t agc_lm_job_p63lm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    ll_restart_here(AGC_JOB_P63LM);
    if (agc_imu_r02both() != AGC_STATUS_OK) {
        return AGC_STATUS_OK;                /* R02 alarmed and went to P00 */
    }
    agc_guid_lm_t *g = &agc_guid_state()->lm;
    g->which = AGC_LM_WHICH_P63;             /* for BURNBABY               */
    g->dvthrush = LL_DPSTHRSH;               /* initialize DVMON           */
    g->dvcntr = 4;
    agc_lm_ldg()->wchphase = AGC_LM_PHASE_IGNALG;
    agc_lm_ldg()->flpass0 = 0;
    agc_status_t st = agc_chan_and(AGC_CHAN_CHAN12,
                                   (uint16_t)(077777u & ~(unsigned)AGC_CH012_RRLOCKON));
    (void)AGC_ASSERT(st == AGC_STATUS_OK);   /* remove track-enable        */
    (void)agc_flag_clear(AGC_FLAG_NOTHROTL);
    (void)agc_flag_clear(AGC_FLAG_REDFLAG);
    (void)agc_flag_clear(AGC_FLAG_LRBYPASS);
    (void)agc_flag_set(AGC_FLAG_MUNFLAG);
    (void)agc_flag_clear(AGC_FLAG_P25FLAG);  /* terminate P25 if running   */
    (void)agc_flag_clear(AGC_FLAG_RNDVZFLG); /* terminate P20 if running   */
    return ll_ignalg_start(ctx);
}

/* LEMPREC was busy: try again (same job, resumed by INTWAKE). */
agc_status_t agc_lm_job_ignalg_prec(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    (void)AGC_ASSERT(agc_lm_ldg()->wchphase == AGC_LM_PHASE_IGNALG);
    return ll_ignalg_prec(ctx, &ip);
}

/* INTSTALL was busy in IGNALOOP: continue with the integration. */
agc_status_t agc_lm_job_ignaloop(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(agc_lm_ldg()->wchphase == AGC_LM_PHASE_IGNALG);
    return ll_ignaloop(ctx, true);
}

/* IGNALGRT / ASTNCLOK (p. 788): prevent repeating IGNALG; start the clock
 * display (STCLOK2), returning in a new job at ASTNRET.               */
agc_status_t agc_lm_job_ignalgrt(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    ll_restart_here(AGC_JOB_IGNALGRT);
    agc_status_t st = agc_lm_stclok2(ctx, (agc_sp_t)-LL_ASTNDEX, AGC_JOB_ASTNRET);
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BUSY);
    return AGC_STATUS_OK;                                 /* ENDOFJOB    */
}

/* ASTNRET (p. 788): pick up the display at the end of R51; PROCEED does a
 * fine alignment, ENTER returns to P63SPOT2 (QMAJ).                   */
agc_status_t agc_lm_job_astnret(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_align_r51p63(ctx, AGC_JOB_P63SPOT2);
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_UNIMPLEMENTED);
    return st;
}

/* P63SPOT2 (pp. 788-789): KALCMANU set for the burn attitude (+X along
 * UNIT(R60VSAVE)), 1 degree deadband, R60LEM; returns at P63SPOT3.    */
agc_status_t agc_lm_job_p63spot2(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_status_t st = agc_imu_set_pointvsm(
        agc_vec_unit(&ip, agc_lm_ldg()->r60vsave, NULL, NULL));
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    st = agc_imu_set_scaxis(agc_vec_make(AGC_DP_HALF, 0, 0));
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    agc_inhint();
    (void)agc_lm_dap_pflitedb();
    agc_relint();
    return agc_imu_r60(ctx, AGC_JOB_P63SPOT3);
}

/* P63SPOT3 (p. 789): is the LR antenna in position 1 yet? If not, ask the
 * astronaut to crank the silly thing around (V50N25 R1 = 00500).      */
agc_status_t agc_lm_job_p63spot3(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    ll_restart_here(AGC_JOB_P63SPOT3);       /* prevent recalling R60     */
    if (agc_lm_chan_bit_present(AGC_CHAN_CHAN33, AGC_CH033_LRPOS1)) {
        return agc_lm_job_p63spot4(ctx);
    }
    return ll_display(ctx, AGC_DSKY_GOPERF1, LL_CODE500,
                      AGC_JOB_P63SPOT3,      /* PROCEED: see if he's lying */
                      AGC_JOB_P63SPOT4);     /* ENTER                      */
}

/* P63SPOT4 (p. 789): initialize the landing radar, off to see the wizard. */
agc_status_t agc_lm_job_p63spot4(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_lm_setpos1();
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_UNIMPLEMENTED);
    return agc_lm_burnbaby(ctx);
}

/* ======================================================================
 * GUILDENSTERN: auto-modes monitor R13 (pp. 800-801)
 * ====================================================================== */

/* STRTP66A (p. 800): ROD initialisation (also after a restart in P66,
 * leaving VDGVERT as is).                                            */
static void ll_strtp66a(void)
{
    agc_lm_ldg_t *l = agc_lm_ldg();
    agc_imu_comp_params_t c;
    agc_status_t st = agc_imu_comp_get(&c);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_vec_t b = agc_vec_make(agc_dp_from_sp(c.pipabias[0]),
                               agc_dp_from_sp(c.pipabias[1]),
                               agc_dp_from_sp(c.pipabias[2]));
    l->vbias = agc_vec_scale(&ip, b, LL_BIASFACT);
    (void)agc_flag_set(AGC_FLAG_RODFLAG);
    for (int i = 0; i < 3; i++) {                 /* VCOMP of TEMX..TEMZ  */
        l->oldpipa[i] = (agc_sp_t)-agc_erase_tem(i);
    }
    l->delvrod = agc_vec_zero();
    l->rodscal1 = agc_lm_pad()->rodscale;
    l->lasttpip = agc_erase_piptime();
    agc_lm_thr()->fcold = 0;
    agc_lm_thr()->fweight = 0;
    (void)AGC_ASSERT(l->rodscal1 == agc_lm_pad()->rodscale);
}

/* VRTSTART (pp. 800-801): WCHPHASE = 2 (P65, P66, P67), STOPRATE ("tem-
 * porary, I hope hope hope"), X-axis override and redesignation off. */
static void ll_vrtstart(agc_sp_t wchvert)
{
    agc_lm_ldg_t *l = agc_lm_ldg();
    l->wchvert = wchvert;
    l->wchphold = AGC_LM_PHASE_VERTICAL;
    l->wchphase = AGC_LM_PHASE_VERTICAL;
    agc_status_t st = agc_lm_dap_stoprate();
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_UNIMPLEMENTED);
    (void)agc_flag_clear(AGC_FLAG_XOVINFLG);
    (void)agc_flag_clear(AGC_FLAG_REDFLAG);
    (void)AGC_ASSERT(wchvert >= 0);
}

/* STARTP66 (p. 800): P66 with desired altitude rate = current rate. */
static void ll_startp66(void)
{
    (void)agc_lm_fastchng();
    agc_status_t st = agc_phase_newmode(66);
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_UNIMPLEMENTED);
    agc_lm_ldg()->vdgvert = agc_guid_state()->lm.hdotdisp;
    ll_strtp66a();
    ll_vrtstart(0);
}

/* STARTP67 (p. 801): no harm in "starting" P67 over and over. */
static void ll_startp67(void)
{
    agc_status_t st = agc_phase_newmode(67);
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_UNIMPLEMENTED);
    agc_lm_ldg()->rodcount = 0;
    ll_vrtstart(10);
}

/* GUILDEN(STERN) (pp. 800-801). On every appearance or disappearance of
 * the manual throttle discrete select P67 or P66; on every appearance of
 * attitude hold select P66, unless in P67. *entry: GUILDRET (continue
 * the automatic landing) or VERTGUID.                                 */
agc_status_t agc_lm_guildenstern(const agc_job_ctx_t *ctx, int *entry)
{
    if (!AGC_ASSERT(ctx != NULL && entry != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_lm_ldg_t *l = agc_lm_ldg();
    *entry = AGC_LM_ENTRY_VERTGUID;
    if (!agc_lm_chan_bit_present(AGC_CHAN_CHAN30, AGC_CH030_AUTOTHROT)) {
        ll_startp67();                       /* un-auto-throttle present  */
        return AGC_STATUS_OK;
    }
    if (agc_phase_checkmm(67)) {             /* P67NOW?                   */
        ll_startp66();
        return AGC_STATUS_OK;
    }
    if (!agc_lm_chan_bit_present(AGC_CHAN_CHAN31, AGC_CH031_ATTHOLD)) {
        *entry = AGC_LM_ENTRY_GUILDRET;      /* STABL?: all's well        */
        return AGC_STATUS_OK;
    }
    if (agc_erase_modreg() == 66) {          /* P66NOW? -> RESTART?       */
        if (!agc_flag_test(AGC_FLAG_RODFLAG)) {
            ll_strtp66a();                   /* restart: reinitialize     */
            ll_vrtstart(0);
        }
        return AGC_STATUS_OK;
    }
    if (l->rodcount == 0) {
        *entry = AGC_LM_ENTRY_GUILDRET;      /* ROD switch not clicked    */
        return AGC_STATUS_OK;
    }
    ll_startp66();                           /* switch into the ROD mode  */
    return AGC_STATUS_OK;
}

/* ======================================================================
 * Landing site perturbation equations (pp. 804-805)
 * ====================================================================== */

/* REDES1 (p. 805): the new site where the perturbed line of sight W
 * (half size) meets the plane of the old site's X coordinate.        */
static agc_vec_t ll_redes1(agc_interp_t *ip, agc_vec_t w)
{
    agc_lm_ldg_t *l = agc_lm_ldg();
    const agc_vec_t r = agc_guid_state()->lm.r;
    agc_dp_t q = agc_dp_div(ip, agc_dp_sub(ip, l->land.c[0], r.c[0]), w.c[0]);
    agc_vec_t n = agc_vec_add(ip, agc_vec_scale(ip, w, q), r);
    n = agc_vec_unit(ip, n, NULL, NULL);
    (void)AGC_ASSERT(w.c[0] != 0);
    return agc_vec_shl(ip, agc_vec_scale(ip, n, l->land_mag), 1);
}

/* REDESIG (pp. 804-805): with REDFLAG set and TREDES not zero, move the
 * landing site by the accumulated LPD increments ELINCR (about the Y
 * body axis) and AZINCR, keeping the line of sight below DEPRCRIT.    */
agc_status_t agc_lm_redesig(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_lm_ldg_t *l = agc_lm_ldg();
    if (!agc_flag_test(AGC_FLAG_REDFLAG) || l->tredes == 0) {
        return agc_lm_rgvgcalc(ip);          /* skip redesignation logic  */
    }
    agc_inhint();
    l->elincr = agc_dp_from_sp(l->elincr1);  /* low words zeroed          */
    l->azincr = agc_dp_from_sp(l->azincr1);
    (void)agc_lm_fastchng();
    l->elincr1 = 0;
    l->azincr1 = 0;
    agc_relint();
    const agc_guid_lm_t *g = &agc_guid_state()->lm;
    int sh = 0;
    agc_vec_t u = agc_rtb_normunit(ip, agc_vec_sub(ip, l->land, g->r), &sh,
                                   NULL, NULL);
    agc_vec_t ynb = g->xnbpip.row[1];
    agc_vec_t e = agc_vec_scale(ip, agc_vec_shl(ip, agc_vec_cross(ip, u, ynb), 1),
                                l->elincr);
    agc_vec_t w = agc_vec_add(ip, agc_vec_sub(ip, agc_vec_scale(ip, ynb, l->azincr),
                                              e), u);
    if (agc_dp_sub(ip, w.c[0], LL_DEPRCRIT) >= 0) {
        w.c[0] = LL_DEPRCRIT;                /* not too close to horizon  */
    }
    l->landtemp = ll_redes1(ip, w);
    (void)agc_lm_fastchng();
    l->land = l->landtemp;
    (void)AGC_ASSERT(sh >= 0);
    return agc_lm_rgvgcalc(ip);
}

/* ======================================================================
 * P66 (rate of descent) and P67 (pp. 816-819)
 * ====================================================================== */

/* VHORCOMP (p. 819): VHORIZ from the given velocity (2^7 m/cs) corrected
 * for lunar rotation, minus its vertical part; then DISPEXIT +3.      */
static agc_status_t ll_vhorcomp(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                                agc_vec_t vel)
{
    const agc_guid_lm_t *g = &agc_guid_state()->lm;
    agc_vec_t v = agc_vec_shr(agc_vec_add(ip, agc_vec_shl(ip, vel, 2), g->delvs), 2);
    agc_vec_t ur = agc_vec_unit(ip, g->r, NULL, NULL);
    agc_vec_t vv = agc_vec_shl(ip, agc_vec_scale(ip, ur, g->hdotdisp), 1);
    agc_lm_ldg()->vhoriz = agc_vec_abval(ip, agc_vec_sub(ip, v, vv));
    (void)AGC_ASSERT(agc_lm_ldg()->vhoriz >= 0);
    return agc_lm_dispexit(ctx, false);      /* avoid the PHASCHNG        */
}

/* RODCOMP first part (pp. 816-817): ROD clicks into VDGVERT, PIPA
 * snapshot at THISTPIP, DELVROD = counts since the last snapshot.
 * Returns PIPA + PIPATMP, the counts since PIPTIME (high words).     */
static agc_vec_t ll_rod_pipas(void)
{
    agc_lm_ldg_t *l = agc_lm_ldg();
    agc_inhint();
    agc_sp_t clicks = l->rodcount;
    l->rodcount = 0;
    l->vdgvert = (agc_dp_t)(l->vdgvert + agc_sp_mul(clicks, l->rodscal1));
    const agc_sp_t pipa[3] = { agc_erase_pipax(), agc_erase_pipay(),
                               agc_erase_pipaz() };
    agc_sp_t rupt[3];
    for (int i = 0; i < 3; i++) {
        rupt[i] = l->oldpipa[i];             /* RUPTREG1..3 = old OLDPIPA */
        l->oldpipa[i] = pipa[i];
    }
    l->thistpip = (agc_dp_t)agc_time_now_cs();
    const agc_sp_t *tmp = agc_guid_state()->avg.pipatmp;
    agc_vec_t m;
    for (int i = 0; i < 3; i++) {
        m.c[i] = agc_dp_from_sp(agc_lm_ad(pipa[i], tmp[i]));
        l->delvrod.c[i] = agc_dp_from_sp(agc_lm_ad(agc_lm_ad(
            (agc_sp_t)-pipa[i], agc_erase_tem(i)), rupt[i]));
        agc_erase_set_tem(i, 0);             /* know when READACCS changes */
    }
    agc_relint();
    (void)AGC_ASSERT(clicks >= -AGC_SP_MAX);
    return m;
}

/* RODCOMP velocity and altitude (pp. 817-818): V propagated to THISTPIP,
 * HDOTDISP and HCALC1 for noun 63. Returns the updated velocity.     */
static agc_vec_t ll_rod_state(agc_interp_t *ip, agc_vec_t pip, agc_dp_t *dt)
{
    agc_lm_ldg_t *l = agc_lm_ldg();
    agc_guid_t *g = agc_guid_state();
    agc_vec_t dvp = agc_vec_scale(ip, pip, LL_KPIP1);       /* 2^7 m/cs  */
    *dt = agc_dp_sub(ip, l->thistpip, agc_erase_piptime()); /* 30D       */
    agc_dp_t ratio = agc_dp_div(ip, *dt, LL_4SEC28);
    agc_vec_t v = agc_vec_scale(ip, agc_vec_sub(ip, g->avg.gdt_2, l->vbias), ratio);
    v = agc_vec_add(ip, agc_vec_add(ip, agc_vec_shl(ip, v, 2), g->lm.v), dvp);
    agc_dp_t rmag = 0;
    agc_vec_t ur = agc_vec_unit(ip, g->lm.r, &rmag, NULL);
    g->lm.hdotdisp = agc_dp_shl(ip, agc_vec_dot(ip, ur, v), 1);
    agc_dp_t h = agc_dp_mul_trunc(agc_dp_shl(ip, *dt, 11), g->lm.hdotdisp);
    l->hcalc1 = agc_dp_sub(ip, agc_dp_add(ip, h, rmag), l->land_mag);
    (void)AGC_ASSERT(*dt >= 0);
    return v;
}

/* RODCOMP acceleration command (pp. 818-819):
 *   /AFC/ = ((VDGVERT - HDOTDISP)/TAUROD + |G|) / (X.UR), then corrected
 *   by LAG/TAU for the measured acceleration and limited to MINFORCE/MASS
 *   .. MAXFORCE/MASS. Returns the measured acceleration (2^-4 m/cs^2). */
static agc_dp_t ll_rod_afc(agc_interp_t *ip, agc_vec_t ur)
{
    agc_lm_ldg_t *l = agc_lm_ldg();
    agc_guid_t *g = agc_guid_state();
    const agc_lm_pad_t *p = agc_lm_pad();
    agc_dp_t arod = agc_dp_div(ip, agc_dp_sub(ip, l->vdgvert, g->lm.hdotdisp), p->taurod);
    agc_dp_t gm = agc_dp_shr(agc_dp_div(ip, agc_vec_abval(ip, g->avg.gdt_2),
                                        LL_GSCALE), 2);          /* 20D  */
    agc_vec_t x = agc_vec_make(AGC_DP_HALF, 0, 0);
    agc_status_t st = agc_guid_cdu_nbsm(&g->trig, &x);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    agc_dp_t xur = agc_vec_dot(ip, x, ur);                       /* 22D  */
    l->afc = agc_dp_div(ip, agc_dp_add(ip, gm, arod), xur);
    agc_vec_t dv = agc_vec_add(ip, agc_vec_scale(ip, l->delvrod, LL_KPIP1), l->vbias);
    agc_dp_t dt2 = agc_dp_sub(ip, l->thistpip, l->lasttpip);
    l->lasttpip = l->thistpip;
    agc_dp_t meas = agc_dp_div(ip, agc_vec_abval(ip, dv),
                               agc_dp_div(ip, dt2, LL_SHFTFACT));
    agc_dp_t fw = agc_dp_mul_trunc(agc_lm_thr()->fweight, LL_BIT1H);
    fw = agc_dp_div(ip, agc_dp_div(ip, fw, g->lm.mass), LL_SCALEFAC);
    agc_dp_t e = agc_dp_sub(ip, agc_dp_div(ip, gm, xur), agc_dp_add(ip, fw, meas));
    agc_dp_t afc = agc_dp_add(ip, agc_dp_mul_trunc(e, p->lag_tau), l->afc);
    agc_dp_t amax = agc_dp_div(ip, p->maxforce, g->lm.mass);
    agc_dp_t amin = agc_dp_div(ip, p->minforce, g->lm.mass);
    l->afc = (afc < amin) ? amin : ((afc >= amax) ? amax : afc);  /* AFCSPOT */
    (void)AGC_ASSERT(xur != 0);
    return meas;
}

/* RODCOMP (pp. 816-819): P66 rate-of-descent guidance, every second
 * (from P66VERTA in the guidance job, and 1 s later from RODTASK).   */
static agc_status_t ll_rodcomp(const agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_vec_t pip = ll_rod_pipas();
    agc_dp_t dt = 0;
    agc_vec_t v = ll_rod_state(&ip, pip, &dt);
    agc_vec_t ur = agc_vec_unit(&ip, agc_guid_state()->lm.r, NULL, NULL);
    agc_dp_t meas = ll_rod_afc(&ip, ur);
    agc_status_t st = agc_lm_throttle_measured(meas);   /* THROTTLE +3   */
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BAD_ARG);
    (void)AGC_ASSERT(dt >= 0);
    return ll_vhorcomp(ctx, &ip, v);
}

/* P66VERTA (p. 816): terminate group 3, RODTASK in one second, RODCOMP. */
agc_status_t agc_lm_p66verta(const agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_phase_change(3, 00003);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    st = agc_waitlist_call(LL_1SEC, AGC_TASK_RODTASK);    /* TWIDDLE      */
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return ll_rodcomp(ctx);
}

/* RODTASK (p. 816): FINDVAC RODCOMP at priority 22. */
agc_status_t agc_lm_task_rodtask(void)
{
    agc_status_t st = agc_exec_findvac(LL_PRIO22, AGC_JOB_RODCOMP);
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL);
    (void)AGC_ASSERT(agc_lm_ldg()->wchphase == AGC_LM_PHASE_VERTICAL);
    return AGC_STATUS_OK;                                 /* TASKOVER     */
}

/* RODCOMP as a job (from RODTASK). */
agc_status_t agc_lm_job_rodcomp(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(agc_lm_ldg()->wchvert == 0);
    return ll_rodcomp(ctx);
}

/* P67VERT (p. 816): terminate group 3; VHORIZ from SERVICER's V. */
agc_status_t agc_lm_p67vert(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    if (!AGC_ASSERT(ctx != NULL && ip != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_phase_change(3, 00003);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return ll_vhorcomp(ctx, ip, agc_guid_state()->lm.v);
}

/* ======================================================================
 * Redesignator trap and R.O.D. trap (pp. 820-822)
 * ====================================================================== */

/* Channel 31 LPD bits that are present (inverted inputs). */
static uint16_t ll_lpd_bits(void)
{
    uint16_t c = (uint16_t)(~agc_chan_read(AGC_CHAN_CHAN31) & LL_ALL4BITS);
    (void)AGC_ASSERT((c & ~(unsigned)LL_ALL4BITS) == 0u);
    (void)AGC_ASSERT(LL_ALL4BITS == 063);
    return c;
}

/* PITFALL (p. 820): RUPT10 from the hand controller. Only in P64: note
 * the bits and start REDESMON in 50 ms.                              */
agc_status_t agc_lm_pitfall(void)
{
    if (!agc_phase_checkmm(64)) {
        return AGC_STATUS_OK;                             /* RESUME       */
    }
    agc_lm_ldg_t *l = agc_lm_ldg();
    l->elvira = (agc_sp_t)ll_lpd_bits();
    l->zerlina = 2;
    agc_status_t st = agc_waitlist_call(5, AGC_TASK_REDESMON);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    (void)AGC_ASSERT(l->elvira >= 0);
    return AGC_STATUS_OK;
}

/* RESETRPT (p. 821): re-enable RUPT10 and end the monitor. */
static agc_status_t ll_resetrpt(void)
{
    agc_status_t st = agc_chan_or(AGC_CHAN_CHAN13, AGC_CH013_RSTTRAP31A);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    (void)AGC_ASSERT(AGC_CH013_RSTTRAP31A == 04000);
    return AGC_STATUS_OK;                                 /* TASKOVER     */
}

/* COUNT'EM (p. 821): each released LPD click moves AZINCR1 by 2 degrees
 * or ELINCR1 by 1/2 degree, unless in attitude hold.                */
static agc_status_t ll_countem(uint16_t bits)
{
    agc_lm_ldg_t *l = agc_lm_ldg();
    if (agc_lm_chan_bit_present(AGC_CHAN_CHAN31, AGC_CH031_ATTHOLD)) {
        return ll_resetrpt();
    }
    if ((bits & LL_MINUSAZBIT) != 0u) {
        l->azincr1 = agc_lm_ad(l->azincr1, (agc_sp_t)-LL_AZEACH);
    }
    if ((bits & LL_PLUSAZBIT) != 0u) {
        l->azincr1 = agc_lm_ad(l->azincr1, LL_AZEACH);
    }
    if ((bits & LL_MINUSELBIT) != 0u) {
        l->elincr1 = agc_lm_ad(l->elincr1, (agc_sp_t)-LL_ELEACH);
    }
    if ((bits & LL_PLUSELBIT) != 0u) {
        l->elincr1 = agc_lm_ad(l->elincr1, LL_ELEACH);
    }
    (void)AGC_ASSERT(bits <= LL_ALL4BITS);
    return ll_resetrpt();
}

/* REDESMON (pp. 820-821): every 70 ms while bits are present; count them
 * when released; give up when ZERLINA runs out.                     */
agc_status_t agc_lm_task_redesmon(void)
{
    agc_lm_ldg_t *l = agc_lm_ldg();
    uint16_t last = (uint16_t)l->elvira;
    l->elvira = (agc_sp_t)ll_lpd_bits();
    if (l->elvira != 0) {
        return agc_waitlist_call(7, AGC_TASK_REDESMON);  /* PREMON2       */
    }
    if (last != 0u) {
        return ll_countem(last);
    }
    if (l->zerlina > 0) {
        l->zerlina = (agc_sp_t)(l->zerlina - 1);          /* PREMON1      */
        return agc_waitlist_call(7, AGC_TASK_REDESMON);
    }
    (void)AGC_ASSERT(l->zerlina == 0);
    return ll_resetrpt();
}

/* DESCBITS (p. 822): from the MARKRUPT coding with channel 16 bit 7
 * (- rate increment) or bit 6 (+ increment). The trap is reset when the
 * switch is released.                                               */
agc_status_t agc_lm_descbits(void)
{
    uint16_t c16 = agc_chan_read(AGC_CHAN_NAVKEYIN);
    agc_sp_t inc = ((c16 & AGC_CH016_DESCENTDN) != 0u) ? -1 : 1;
    agc_lm_ldg_t *l = agc_lm_ldg();
    l->rodcount = agc_lm_ad(l->rodcount, inc);
    (void)AGC_ASSERT((c16 & (AGC_CH016_DESCENTDN | AGC_CH016_DESCENTUP)) != 0u);
    return AGC_STATUS_OK;                                 /* RESUME       */
}

/* ======================================================================
 * P68: landing confirmation (pp. 791-792)
 * ====================================================================== */

/* LANDJUNK (p. 791): zero attitude error, 5 degree deadband, surface
 * flags, LAT/LONG/ALT of the landing point and RLS = R-TO-RP(RN).    */
static agc_status_t ll_landjunk(agc_interp_t *ip)
{
    agc_inhint();
    (void)agc_lm_dap_zatteror();
    (void)agc_lm_dap_setmaxdb();
    agc_relint();
    (void)agc_flag_set(AGC_FLAG_SURFFLAG);
    (void)agc_flag_clear(AGC_FLAG_LETABORT);
    (void)agc_flag_set(AGC_FLAG_APSFLAG);
    (void)agc_flag_set(AGC_FLAG_LUNAFLAG);
    agc_navaux_padload_t nav;
    agc_status_t st = agc_navaux_get_padload(&nav);
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    agc_vec_t rn = agc_erase_rn();                        /* ALPHAV       */
    agc_dp_t t = agc_erase_piptime();
    agc_latlong_opt_t opt = { true, agc_flag_test(AGC_FLAG_ERADFLAG) };
    agc_latlong_t ll;
    st = agc_latlong_from_r(ip, &nav, opt, rn, t, &ll);
    if (st == AGC_STATUS_OK) {
        st = agc_navaux_set_latlong(&ll);                 /* N43 data     */
    }
    agc_vec_t rls = agc_vec_zero();
    st = agc_planet_r_to_rp(ip, &nav, true, agc_vec_shl(ip, rn, 2), t, &rls);
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    agc_erase_set_rls(rls);
    return AGC_STATUS_OK;
}

/* LANDJUNK job (P68). */
agc_status_t agc_lm_job_p68(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    ll_restart_here(AGC_JOB_LANDJUNK);
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_status_t st = ll_landjunk(&ip);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_lm_job_p68_flash(ctx);
}

/* V06N43 flash: "astronaut: now look where you ended up". */
agc_status_t agc_lm_job_p68_flash(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(agc_flag_test(AGC_FLAG_SURFFLAG));
    return ll_display(ctx, AGC_DSKY_GOFLASH, agc_dsky_vn(6, 43),
                      AGC_JOB_LANDJNK2, AGC_JOB_LANDJNKF);
}

/* After PROCEED (p. 792): GSAV and (using REFMF) YNBSAV, ZNBSAV and
 * ATTFLAG for P57; then "astronaut: please select P57".             */
agc_status_t agc_lm_job_p68b(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_align_ram()->gsav = agc_vec_make(AGC_DP_HALF, 0, 0);   /* UNITX  */
    agc_status_t st = agc_align_refmf(&ip);
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_UNIMPLEMENTED);
    return agc_exec_gotopooh();
}
