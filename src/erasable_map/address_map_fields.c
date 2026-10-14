/**********************************************************************
 * FILE NAME: address_map_fields.c
 *
 * PURPOSE:
 *   The named erasables the downlists send that live in an owner's
 *   storage (address map items EC_F_TIME and after): each item's words
 *   from its typed value, and a store from words where the owner keeps a
 *   writable value. The addresses are in the maps of address_map.c.
 *
 * AGC SOURCE:
 *   none (the erasables are those of Comanche055 DOWNLINK_LISTS.agc
 *   pp.170-180 and Luminary099 DOWNLINK_LISTS.agc pp.193-205, addresses
 *   from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name           Type      I/O  Scale        Description
 *   -------------  --------  ---  -----------  ---------------------------
 *   rendezvous     agc_dp_t  I/O  each its own RTARG, DELVEET1-3, TCDH ...
 *    erasables                                 (agc_rendz_erasable())
 *   CSM DAP, entry various   I/O  each its own AK, WBODY, ADOT, ERRORX, PCMD,
 *    and burn                                  ALFA/180, LATANG, VPRED ...
 *   LM guidance,   various   I/O  each its own RGU, VGU, LAND, TTF/8, HMEAS,
 *    landing, align                            GSAV, STARSAV1/2, TALIGN ...
 *   value-only     various   I(O) each its own TIME2/1, the counters, UPBUFF,
 *    owners                                    REDOCTR, AGSBUFF, DOWNTORK ...
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   ec_word_of_sp, ...    word conversions (address_map.c)
 *   the owners' accessors see the includes
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ec_field_store returns an owner setter's status (CDUXD, OMEGAPD).
 *
 * NOTES:
 *   Two kinds of item. A field item is a value in an owner's struct the
 *   map reaches by reference (fref_t), so reading and storing are the
 *   same code for every one. A value item is reached only through the
 *   owner's getter: it reads; it stores where the owner has a setter
 *   (TIME2/1, the counters, OPTMODES, LANDMARK, CMDAPMOD, CDUXD,
 *   OMEGAPD, DNLSTCOD, AGSWORD, REDOCTR, the CSM optics monitor words,
 *   GCOMP, 1/PIPADT, the LM DAP's OMEGAP .. ALPHAR and DOWNTORK),
 *   otherwise a store leaves the typed value alone and the
 *   address keeps the stored word until the owner writes it again.
 *   DP values give sign-agreeing words; ADOT and OMEGAB keep the two
 *   words as stored (agc_dp_words_t).
 *   DEVIATION: TTF/8 sends the DP value's sign-agreeing words, not the
 *   words DAS left (ttf_8_hi); CMDAPMOD's -0 (AGC_ENTRY_DAPMOD_MINUS0) is
 *   the word 77777.
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/state.h"
#include "agc/core/word.h"
#include "agc/csm/dap.h"
#include "agc/csm/entry.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/nav/integration.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"
#include "internal.h"

#define F_WORD15  077777u /* one AGC word                  */
#define F_LOW14   037777u /* TIME1 / TIME2: 14-bit counts  */
#define F_TIME_SH 14      /* TIME2 is the count over 2^14  */
#define F_NEG0    077777u /* -0                            */
#define F_NEG1    077776u /* -1                            */
#define F_SIGN    040000u /* sign bit                      */

/* How an element of a field item is held.                                */
typedef enum fk
{
    FK_SP = 0, /* agc_sp_t: one signed word             */
    FK_RAW,    /* uint16_t: one bit-pattern word        */
    FK_DP,     /* agc_dp_t: two words (sign-agreeing)   */
    FK_DPW     /* agc_dp_words_t: two words as stored   */
} fk_t;

/* One element of a field item in its owner's storage.                    */
typedef struct fref
{
    fk_t            kind; /* which pointer is set */
    agc_sp_t       *sp;   /* FK_SP                */
    uint16_t       *raw;  /* FK_RAW               */
    agc_dp_t       *dp;   /* FK_DP                */
    agc_dp_words_t *dpw;  /* FK_DPW               */
} fref_t;

/**********************************************************************
 * AGC: none -- a reference to an SP element
 *
 * ARGUMENTS:
 *   p  agc_sp_t *  I   the element
 *
 * RETURNS: fref_t -- the reference
 **********************************************************************/
static fref_t ref_sp(agc_sp_t *p)
{
    fref_t r = {FK_SP, p, NULL, NULL, NULL}; /* SP element */

    return (r);
}

/**********************************************************************
 * AGC: none -- a reference to a raw-word element
 *
 * ARGUMENTS:
 *   p  uint16_t *  I   the element
 *
 * RETURNS: fref_t -- the reference
 **********************************************************************/
static fref_t ref_raw(uint16_t *p)
{
    fref_t r = {FK_RAW, NULL, p, NULL, NULL}; /* raw element */

    return (r);
}

/**********************************************************************
 * AGC: none -- a reference to a DP element
 *
 * ARGUMENTS:
 *   p  agc_dp_t *  I   the element
 *
 * RETURNS: fref_t -- the reference
 **********************************************************************/
static fref_t ref_dp(agc_dp_t *p)
{
    fref_t r = {FK_DP, NULL, NULL, p, NULL}; /* DP element */

    return (r);
}

/**********************************************************************
 * AGC: none -- a reference to a DP element kept as its two words
 *
 * ARGUMENTS:
 *   p  agc_dp_words_t *  I   the element
 *
 * RETURNS: fref_t -- the reference
 **********************************************************************/
static fref_t ref_dpw(agc_dp_words_t *p)
{
    fref_t r = {FK_DPW, NULL, NULL, NULL, p}; /* DP words element */

    return (r);
}

/**********************************************************************
 * AGC: none -- the rendezvous erasables (agc_rendz_erasable()), element k
 *
 * ARGUMENTS:
 *   item  ec_item_t  I   the item
 *   k     int        I   element, from 0
 *   r     fref_t *   O   the element
 *
 * RETURNS: bool -- true for a rendezvous item
 **********************************************************************/
static bool ref_rendezvous(ec_item_t item, int k, fref_t *r)
{
    agc_rendz_t *rz       = agc_rendz_erasable();                  /* rendezvous erasables */
    agc_dp_t    *range[3] = {&rz->range, &rz->rrate, &rz->rtheta}; /* RANGE, RRATE, RTHETA */
    agc_dp_t    *apo[2]   = {&rz->hapox, &rz->hperx};              /* HAPOX, HPERX */
    bool         found    = true;                                  /* a rendezvous item */

    switch ((int) item)
    {
        case EC_F_RTARG:
            *r    = ref_dp(&rz->rtarg.c[k]);
            break;
        case EC_F_DELVEET1:
            *r    = ref_dp(&rz->delveet1.c[k]);
            break;
        case EC_F_DELVEET2:
            *r    = ref_dp(&rz->delveet2.c[k]);
            break;
        case EC_F_DELVEET3:
            *r    = ref_dp(&rz->delveet3.c[k]);
            break;
        case EC_F_DELVSIN:
            *r    = ref_dp(&rz->delvsin.c[k]);
            break;
        case EC_F_DELLT4:
            *r    = ref_dp(&rz->dellt4);
            break;
        case EC_F_ELEV:
            *r    = ref_dp(&rz->elev);
            break;
        case EC_F_CENTANG:
            *r    = ref_dp(&rz->centang);
            break;
        case EC_F_DELTAR:
            *r    = ref_dp(&rz->deltar);
            break;
        case EC_F_DIFFALT:
            *r    = ref_dp(&rz->diffalt);
            break;
        case EC_F_TCDH:
            *r    = ref_dp(&rz->tcdh);
            break;
        case EC_F_TCSI:
            *r    = ref_dp(&rz->tcsi);
            break;
        case EC_F_TPASS4:
            *r    = ref_dp(&rz->tpass4);
            break;
        case EC_F_DELVTPF:
            *r    = ref_dp(&rz->delvtpf);
            break;
        case EC_F_NN:
            *r    = ref_sp(&rz->nn);
            break;
        case EC_F_VHFCNT:
            *r    = ref_sp(&rz->vhfcnt);
            break;
        case EC_F_RANGE:
            *r    = ref_dp(range[k]);
            break;
        case EC_F_HAPOX:
            *r    = ref_dp(apo[k]);
            break;
        default:
            found = false;
            break;
    } /* switch */

    return (found);
} /* ref_rendezvous */

/**********************************************************************
 * AGC: none -- the CSM DAP (RCS, TVC, common), entry and burn erasables
 * and the alignment / gyro words both ropes send, element k
 *
 * ARGUMENTS:
 *   item  ec_item_t  I   the item
 *   k     int        I   element, from 0
 *   r     fref_t *   O   the element
 *
 * RETURNS: bool -- true for one of these items
 **********************************************************************/
static bool ref_csm(ec_item_t item, int k, fref_t *r)
{
    agc_csm_dapcom_t  *dc     = agc_csm_dap_com();                     /* RCS/TVC common */
    agc_csm_rcs_t     *rcs    = agc_csm_dap_rcs();                     /* RCS DAP */
    agc_csm_tvc_t     *tvc    = agc_csm_dap_tvc();                     /* TVC DAP */
    agc_entry_state_t *es     = agc_entry_state();                     /* entry */
    agc_dp_t          *lat[3] = {&es->latang, &es->rdot, &es->thetah}; /* LATANG, RDOT, THETAH */
    agc_dp_t          *vio[2] = {&es->vio, &es->tte};                  /* VIO, TTE */
    agc_dp_t          *vpr[2] = {&es->vpred, &es->gammaei};            /* VPRED, GAMMAEI */
    bool               found  = true;                                  /* one of these */

    switch ((int) item)
    {
        case EC_F_AK:
            *r    = ref_sp(&dc->ak[k]);
            break;
        case EC_F_RCSFLAGS:
            *r    = ref_raw(&dc->rcsflags);
            break;
        case EC_F_WBODY:
            *r    = ref_dp(&rcs->wbody[k]);
            break;
        case EC_F_OMEGAC:
            *r    = ref_dp(&tvc->omegac[k]);
            break;
        case EC_F_ADOT:
            *r    = ref_dpw(&rcs->adot[k]);
            break;
        case EC_F_OMEGAB:
            *r    = ref_dpw(&tvc->omegab[k]);
            break;
        case EC_F_ERRORX:
            *r    = ref_sp(&rcs->error[k]);
            break;
        case EC_F_PCMD:
            *r    = ref_sp((k == 0) ? &tvc->pcmd : &tvc->ycmd);                           /* PCMD, YCMD */
            break;
        case EC_F_ALFA_180:
            *r    = ref_sp((k == 0) ? &es->alfa_180 : &es->beta_180);                     /* ALFA/180, BETA/180 */
            break;
        case EC_F_ROLLTM:
            *r    = ref_sp(&es->rolltm);
            break;
        case EC_F_ROLLC:
            *r    = ref_dp(&es->rollc);
            break;
        case EC_F_L_D1:
            *r    = ref_dp(&es->l_d1);
            break;
        case EC_F_LATANG:
            *r    = ref_dp(lat[k]);
            break;
        case EC_F_VIO:
            *r    = ref_dp(vio[k]);
            break;
        case EC_F_VPRED:
            *r    = ref_dp(vpr[k]);
            break;
        case EC_F_ECSTEER:
            *r    = ref_dp(&agc_guid_state()->burn.ecsteer);
            break;
        case EC_F_CSTEER:
            *r    = ref_dp(&agc_guid_state()->burn.csteer);
            break;
        case EC_F_BESTI:
            *r    = ref_sp((k == 0) ? &agc_align_ram()->besti : &agc_align_ram()->bestj); /* BESTI, BESTJ */
            break;
        case EC_F_OGC:
            *r    = ref_dp(&agc_imu_ogc()[k]);
            break;
        default:
            found = false;
            break;
    } /* switch */

    return (found);
} /* ref_csm */

/**********************************************************************
 * AGC: none -- the LM guidance, landing, throttle and alignment
 * erasables, element k
 *
 * ARGUMENTS:
 *   item  ec_item_t  I   the item
 *   k     int        I   element, from 0
 *   r     fref_t *   O   the element
 *
 * RETURNS: bool -- true for one of these items
 **********************************************************************/
static bool ref_lm(ec_item_t item, int k, fref_t *r)
{
    agc_guid_lm_t   *lm     = &agc_guid_state()->lm;                                          /* LM guidance */
    agc_lm_ldg_t    *ldg    = agc_lm_ldg();                                                   /* landing guidance */
    agc_align_ram_t *ram    = agc_align_ram();                                                /* alignment */
    agc_sp_t        *gim[3] = {&lm->aig, &lm->amg, &lm->aog};                                 /* AIG AMG AOG of the LR mark */
    agc_dp_t        *nbs[2] = {&ram->ynbsav.c[k % AGC_VEC_N], &ram->znbsav.c[k % AGC_VEC_N]}; /* YNBSAV, ZNBSAV */
    bool             found  = true;                                                           /* one of these */

    switch ((int) item)
    {
        case EC_F_GSAV:
            *r    = ref_dp(&ram->gsav.c[k]);
            break;
        case EC_F_YNBSAV:
            *r    = ref_dp(nbs[k / AGC_VEC_N]);
            break;
        case EC_F_STARSAV1:
            *r    = ref_dp(&ram->starsav1.c[k]);
            break;
        case EC_F_STARSAV2:
            *r    = ref_dp(&ram->starsav2.c[k]);
            break;
        case EC_F_TALIGN:
            *r    = ref_dp(&ram->talign);
            break;
        case EC_F_AOTCODE:
            *r    = ref_sp(&ram->starcode);
            break;
        case EC_F_LRCDUDL:
            *r    = ref_raw(&lm->lrcdudl[k]);
            break;
        case EC_F_LRVTIMDL:
            *r    = ref_dp(&lm->lrvtimdl);
            break;
        case EC_F_VSELECT:
            *r    = ref_sp(&lm->vselect);
            break;
        case EC_F_VMEAS:
            *r    = ref_dp(&lm->vmeas);
            break;
        case EC_F_HMEAS:
            *r    = ref_dp(&lm->hmeas);
            break;
        case EC_F_LR_GIMBALS:
            *r    = ref_sp(gim[k]);
            break;
        case EC_F_LR_MKTIME:
            *r    = ref_dp(&lm->mktime);
            break;
        case EC_F_RGU:
            *r    = ref_dp(&ldg->rgu.c[k]);
            break;
        case EC_F_VGU:
            *r    = ref_dp(&ldg->vgu.c[k]);
            break;
        case EC_F_LAND:
            *r    = ref_dp(&ldg->land.c[k]);
            break;
        case EC_F_TTF_8:
            *r    = ref_dp(&ldg->ttf_8);
            break;
        case EC_F_DELTAH:
            *r    = ref_dp(&ldg->deltah);
            break;
        case EC_F_PSEUDO55:
            *r    = ref_sp(&agc_lm_thr()->pseudo55);
            break;
        case EC_F_FC:
            *r    = ref_dp(&agc_lm_thr()->fc);
            break;
        default:
            found = false;
            break;
    } /* switch */

    return (found);
} /* ref_lm */

/**********************************************************************
 * AGC: none -- element k of a field item, when the item is one
 *
 * ARGUMENTS:
 *   item  ec_item_t  I   the item
 *   k     int        I   element, from 0
 *   r     fref_t *   O   the element
 *
 * RETURNS: bool -- true for a field item, false for a value item
 **********************************************************************/
static bool ref_of(ec_item_t item, int k, fref_t *r)
{
    return (ref_rendezvous(item, k, r) || ref_csm(item, k, r) || ref_lm(item, k, r));
}

/**********************************************************************
 * AGC: none -- word j (0 high, 1 low) of a DP value
 *
 * ARGUMENTS:
 *   v  agc_dp_t  I   the value
 *   j  int       I   0 or 1
 *
 * RETURNS: uint16_t -- the word (sign-agreeing pair)
 **********************************************************************/
static uint16_t dp_word(agc_dp_t v, int j)
{
    uint16_t w[2]; /* high, low */

    ec_dp_to(v, w);

    return (w[j]);
}

/**********************************************************************
 * AGC: none -- word j of a value item of both ropes and the CSM: the
 * clock, the counters, the uplink buffer, REDOCTR, and the CSM words
 * whose owners give a value only
 *
 * ARGUMENTS:
 *   item  ec_item_t  I   the item
 *   j     int        I   word, from 0
 *   w     uint16_t * O   the word
 *
 * RETURNS: bool -- true for one of these items
 **********************************************************************/
static bool value_word_csm(ec_item_t item, int j, uint16_t *w)
{
    uint16_t           cnt[8] = {agc_erase_cdux(), agc_erase_cduy(), agc_erase_cduz(), agc_erase_cdut(), agc_erase_cdus(), /* CDUX .. CDUS */
                       ec_word_of_sp(agc_erase_pipax()), ec_word_of_sp(agc_erase_pipay()), ec_word_of_sp(agc_erase_pipaz())}; /* PIPAX .. PIPAZ, pulses unscaled */
    uint32_t           t      = agc_time_now_cs();                                                                            /* TIME2,TIME1 */
    agc_vec_t          other  = (item == EC_F_R_OTHER) ? agc_integ_r_other() : agc_integ_v_other();                           /* R-OTHER m 2^29 or V-OTHER m/cs 2^7 */
    bool               found  = true;                                                                                         /* one of these */
    agc_entry_dapmod_t m      = agc_entry_state()->cmdapmod;                                                                  /* CMDAPMOD */
    agc_dp_t           g[3];                                                                                                  /* GCOMP, gyro pulses 2^14 */

    switch ((int) item)
    {
        case EC_F_TIME:
            *w    = (uint16_t) ((j == 0) ? ((t >> F_TIME_SH) & F_LOW14) : (t & F_LOW14));                            /* TIME2, TIME1 */
            break;
        case EC_F_COUNTERS:
            *w    = cnt[j];
            break;
        case EC_F_UPLINK:
            *w    = agc_uplink_update_dnword(j);
            break;
        case EC_F_REDOCTR:
            *w    = agc_restart_redoctr();
            break;
        case EC_F_OPTMODES:
            *w    = agc_erase_optmodes();
            break;
        case EC_F_VHFTIME:
            *w    = dp_word(agc_erase_vhftime(), j);
            break;
        case EC_F_LAUNCHAZ:
            *w    = dp_word(agc_erase_launchaz(), j);
            break;
        case EC_F_8NN:
            *w    = ec_word_of_sp(agc_erase_8nn());
            break;
        case EC_F_LANDMARK:
            *w    = ec_word_of_sp(agc_erase_landmark());
            break;
        case EC_F_SVMRKDAT:
            *w    = agc_erase_svmrkdat(j);
            break;
        case EC_F_R_OTHER:
        case EC_F_V_OTHER:
            *w    = dp_word(other.c[j / 2], j % 2);
            break;
        case EC_F_CMDAPMOD:
            *w    = (m == AGC_ENTRY_DAPMOD_MINUS0) ? (uint16_t) F_NEG0 : ec_word_of_sp((agc_sp_t) m);                /* -1, +0, +1 or -0 */
            break;
        case EC_F_PREL:
            *w    = ec_word_of_sp(agc_erase_prel(j));
            break;
        case EC_F_PAXERR1:
            *w    = ec_word_of_sp(agc_erase_paxerr1());
            break;
        case EC_F_DNLSTCOD:
            *w    = (j == 0) ? agc_erase_dnlstcod() : agc_erase_dumpcnt();                                           /* DNLSTCOD, DUMPCNT */
            break;
        case EC_F_OPTIND:
            *w    = (agc_erase_optind() == AGC_OPTIND_NEG0) ? (uint16_t) F_NEG0 : ec_word_of_sp(agc_erase_optind()); /* -0: TVC takeover */
            break;
        case EC_F_OPTZONE:
            *w    = ec_word_of_sp(agc_erase_optmon(AGC_OPTMON_ZONE));
            break;
        case EC_F_OPTMON:
            *w    = ec_word_of_sp(agc_erase_optmon((agc_optmon_word_t) j));
            break;
        case EC_F_DESOPT:
            *w    = (j == 0) ? agc_erase_desoptt() : agc_erase_desopts();                                            /* DESOPTT, DESOPTS */
            break;
        case EC_F_GCOMP:
            agc_imu_gyro_buf_get(AGC_IMU_GBUF_GCOMP, g);
            *w    = dp_word(g[j / 2], j % 2);
            break;
        case EC_F_PIPADT:
            *w    = ec_word_of_sp(agc_imu_pipa_interval());
            break;
        default:
            found = false;
            break;
    } /* switch */

    return (found);
} /* value_word_csm */

/**********************************************************************
 * AGC: none -- word j of an LM value item: words whose owners give a
 * value only (radar, AGS, DAP, the R29 and LRS22.1 mark words)
 *
 * ARGUMENTS:
 *   item  ec_item_t  I   the item
 *   j     int        I   word, from 0
 *
 * RETURNS: uint16_t -- the word
 **********************************************************************/
static uint16_t value_word_lm(ec_item_t item, int j)
{
    const agc_rr_downlink_t *rr       = agc_rendz_rr_downlink();     /* R29 words */
    const uint16_t           rrg[3]   = {rr->aig, rr->amg, rr->aog}; /* AIG AMG AOG */
    const agc_sp_t           dap[8]   = {agc_erase_omegap(), agc_erase_omegaq(), agc_erase_omegar(), /* OMEGAP Q R */
                             agc_erase_alphaq(), agc_erase_alphar(),                                 /* ALPHAQ R   */
                             agc_erase_omegapd(), agc_erase_omegaqd(), agc_erase_omegard()};       /* OMEGAPD QD RD */
    const uint16_t           cduxd[3] = {agc_erase_cduxd(), agc_erase_cduyd(), agc_erase_cduzd()}; /* CDUXD YD ZD */
    uint16_t                 w        = 0;                                                         /* the word */

    switch ((int) item)
    {
        case EC_F_LASTCMD:
            w = ec_word_of_sp(agc_erase_lastycmd(j));
            break;
        case EC_F_DNRADAR:
            w = (j == 2) ? agc_erase_dnindex() : agc_erase_dnrrange(j); /* DNINDEX between DNRRDOT and DNLRVELX */
            break;
        case EC_F_AGSK:
            w = dp_word(agc_erase_agsk(), j);
            break;
        case EC_F_AGSBUFF:
            w = agc_ags_buffer()[j];
            break;
        case EC_F_OMEGAP:
            w = ec_word_of_sp(dap[j]);
            break;
        case EC_F_ALPHAQ:
            w = ec_word_of_sp(dap[3 + j]);
            break;
        case EC_F_OMEGAPD:
            w = ec_word_of_sp(dap[5 + j]);
            break;
        case EC_F_CDUXD:
            w = cduxd[j];
            break;
        case EC_F_DOWNTORK:
            w = ec_word_of_sp(agc_erase_downtork(j));
            break;
        case EC_F_RR_GIMBALS:
            w = rrg[j];
            break;
        case EC_F_RR_TANGNB:
            w = rr->tangnb[j];
            break;
        case EC_F_RR_MKTIME:
            w = dp_word(rr->mktime, j);
            break;
        case EC_F_NAV_MKTIME:
            w = dp_word(agc_erase_mktime(), j);
            break;
        case EC_F_RM:
            w = dp_word(agc_erase_rm(), j);
            break;
        case EC_F_AGSWORD:
            w = agc_erase_agsword();
            break;
        default:
            w = agc_erase_rangrdot(j);                                  /* EC_F_RANGRDOT */
            break;
    } /* switch */

    return ((uint16_t) (w & F_WORD15));
} /* value_word_lm */

/**********************************************************************
 * AGC: none -- the element layout of a value item
 *
 * ARGUMENTS:
 *   item  ec_item_t  I   the item
 *
 * RETURNS: ec_layout_t -- EC_SP, EC_RAW or EC_DP
 **********************************************************************/
static ec_layout_t value_layout(ec_item_t item)
{
    ec_layout_t l = EC_RAW; /* the clock, counters, buffers, CDU words */

    switch ((int) item)
    {
        case EC_F_VHFTIME:
        case EC_F_LAUNCHAZ:
        case EC_F_R_OTHER:
        case EC_F_V_OTHER:
        case EC_F_AGSK:
        case EC_F_GCOMP:
        case EC_F_RR_MKTIME:
        case EC_F_NAV_MKTIME:
        case EC_F_RM:
            l = EC_DP;
            break;
        case EC_F_8NN:
        case EC_F_LANDMARK:
        case EC_F_CMDAPMOD:
        case EC_F_PREL:
        case EC_F_PAXERR1:
        case EC_F_LASTCMD:
        case EC_F_OMEGAP:
        case EC_F_ALPHAQ:
        case EC_F_OMEGAPD:
        case EC_F_DOWNTORK:
        case EC_F_OPTIND:
        case EC_F_OPTZONE:
        case EC_F_OPTMON:
        case EC_F_PIPADT:
            l = EC_SP;
            break;
        default:
            break;
    } /* switch */

    return (l);
} /* value_layout */

/**********************************************************************
 * AGC: none -- how a field item's words group into values
 *
 * ARGUMENTS:
 *   item  ec_item_t  I   a field item (EC_F_TIME and after)
 *
 * RETURNS: ec_layout_t -- EC_SP, EC_RAW or EC_DP
 **********************************************************************/
ec_layout_t ec_field_layout(ec_item_t item)
{
    fref_t      r;         /* its first element */
    ec_layout_t l = EC_DP; /* DP and DP words */

    if (!ref_of(item, 0, &r))
    {
        l = value_layout(item);
    }
    else if (r.kind == FK_SP)
    {
        l = EC_SP;
    }
    else if (r.kind == FK_RAW)
    {
        l = EC_RAW;
    }
    else
    {
        /* FK_DP, FK_DPW */
    }

    return (l);
}

/**********************************************************************
 * AGC: none -- a field item's words from its typed value
 *
 * ARGUMENTS:
 *   item   ec_item_t   I   a field item
 *   words  int         I   its words (the map entry)
 *   w      uint16_t *  O   the words
 **********************************************************************/
void ec_field_words(ec_item_t item, int words, uint16_t *w)
{
    int    n = (ec_field_layout(item) == EC_DP) ? 2 : 1; /* words per element */
    fref_t r;                                            /* element reference */

    for (int k = 0; k * n < words; k++)
    {
        if (!ref_of(item, k, &r))
        {
            for (int j = k * n; j < (k + 1) * n; j++)
            {
                if (!value_word_csm(item, j, &w[j]))
                {
                    w[j] = value_word_lm(item, j);
                }
            }
        }
        else if (r.kind == FK_SP)
        {
            w[k] = ec_word_of_sp(*r.sp);
        }
        else if (r.kind == FK_RAW)
        {
            w[k] = (uint16_t) (*r.raw & F_WORD15);
        }
        else if (r.kind == FK_DP)
        {
            ec_dp_to(*r.dp, &w[2 * k]);
        }
        else
        {
            w[2 * k]     = ec_word_of_sp(r.dpw->hi);
            w[2 * k + 1] = ec_word_of_sp(r.dpw->lo);
        }
    }
} /* ec_field_words */

/**********************************************************************
 * AGC: none -- the CMDAPMOD state a word stands for (the entry DAP
 * stores -1, +0, +1 or -0; any other negative word reads as -1, any
 * other positive one as +1)
 *
 * ARGUMENTS:
 *   w  uint16_t  I   the word
 *
 * RETURNS: agc_entry_dapmod_t -- the state
 **********************************************************************/
static agc_entry_dapmod_t cmdapmod_of(uint16_t w)
{
    agc_entry_dapmod_t m = AGC_ENTRY_DAPMOD_PLUS1; /* positive */

    if (w == F_NEG0)
    {
        m = AGC_ENTRY_DAPMOD_MINUS0;
    }
    else if ((w & F_SIGN) != 0u)
    {
        m = AGC_ENTRY_DAPMOD_MINUS1;
    }
    else if (w == 0u)
    {
        m = AGC_ENTRY_DAPMOD_PLUS0;
    }
    else
    {
        /* +1 */
    }

    return (m);
}

/**********************************************************************
 * AGC: none -- store CDUXD..ZD or OMEGAPD..RD through the LM DAP's
 * setters, stopping at the first refusal
 *
 * ARGUMENTS:
 *   item  ec_item_t         I   EC_F_CDUXD or EC_F_OMEGAPD
 *   w     const uint16_t *  I   the three words
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the first setter's refusal
 **********************************************************************/
static agc_status_t lm_dap_store(ec_item_t item, const uint16_t *w)
{
    bool         cdu = (item == EC_F_CDUXD);                                                         /* CDUXD..ZD, else OMEGAPD..RD */
    agc_status_t st  = cdu ? agc_erase_set_cduxd(w[0]) : agc_erase_set_omegapd(ec_sp_of_word(w[0])); /* X / P */

    if (st == AGC_STATUS_OK)
    {
        st = cdu ? agc_erase_set_cduyd(w[1]) : agc_erase_set_omegaqd(ec_sp_of_word(w[1])); /* Y / Q */
    }

    if (st == AGC_STATUS_OK)
    {
        st = cdu ? agc_erase_set_cduzd(w[2]) : agc_erase_set_omegard(ec_sp_of_word(w[2])); /* Z / R */
    }

    return (st);
}

/**********************************************************************
 * AGC: none -- store a word an image load restores to its owner (the
 * list code and dump count, the restart count, the CSM optics monitor,
 * GCOMP, 1/PIPADT, the LM DAP's rates and torques, AGSWORD); any other
 * value item keeps its typed value (the address keeps the stored word)
 *
 * ARGUMENTS:
 *   item  ec_item_t         I   a value item
 *   w     const uint16_t *  I   its words
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or 1/PIPADT's setter status
 **********************************************************************/
static agc_status_t load_store(ec_item_t item, const uint16_t *w)
{
    agc_status_t st    = AGC_STATUS_OK;                 /* the setter's status */
    int          n     = (item == EC_F_OMEGAP) ? 3 : 2; /* words: OMEGAP..R, or ALPHAQ..R */
    int          first = (item == EC_F_OMEGAP) ? 0 : 3; /* OMEGAP +first */

    switch ((int) item)
    {
        case EC_F_DNLSTCOD:
            agc_telem_select_list((int) w[0]);                                              /* NEWLIST refuses a code outside DNTABLE */
            agc_erase_set_dumpcnt(w[1]);
            break;
        case EC_F_REDOCTR:
            agc_restart_set_redoctr(w[0]);
            break;
        case EC_F_OPTIND:
            agc_erase_set_optind((w[0] == F_NEG0) ? AGC_OPTIND_NEG0 : ec_sp_of_word(w[0])); /* -0: TVC takeover */
            break;
        case EC_F_OPTZONE:
            agc_erase_set_optmon(AGC_OPTMON_ZONE, ec_sp_of_word(w[0]));
            break;
        case EC_F_OPTMON:

            for (int k = 0; k < (int) AGC_OPTMON_ZONE; k++)
            {
                agc_erase_set_optmon((agc_optmon_word_t) k, ec_sp_of_word(w[k]));
            }

            break;
        case EC_F_DESOPT:
            agc_erase_set_desoptt(w[0]);
            agc_erase_set_desopts(w[1]);
            break;
        case EC_F_GCOMP:
        {
            const agc_dp_t g[3] = {ec_dp_of(&w[0]), ec_dp_of(&w[2]), ec_dp_of(&w[4])}; /* gyro pulses 2^14 */

            agc_imu_gyro_buf_set(AGC_IMU_GBUF_GCOMP, g);
            break;
        }
        case EC_F_PIPADT:
            st = agc_imu_set_pipa_interval(ec_sp_of_word(w[0]));
            break;
        case EC_F_OMEGAP:
        case EC_F_ALPHAQ:

            for (int k = 0; k < n; k++)
            {
                agc_erase_set_omegap_k(first + k, ec_sp_of_word(w[k])); /* OMEGAP +k / ALPHAQ +k */
            }

            break;
        case EC_F_DOWNTORK:

            for (int k = 0; k < 6; k++)
            {
                agc_erase_set_downtork(k, ec_sp_of_word(w[k]));
            }

            break;
        case EC_F_AGSWORD:
            agc_erase_set_agsword(w[0]);
            break;
        default:
            break; /* read-only owner: the typed value stays */
    } /* switch */

    return (st);
} /* load_store */

/**********************************************************************
 * AGC: none -- store a value item whose owner has a setter; the others
 * keep their typed value (the address keeps the stored word)
 *
 * ARGUMENTS:
 *   item  ec_item_t         I   a value item
 *   w     const uint16_t *  I   its words
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or an owner setter's status
 **********************************************************************/
static agc_status_t value_store(ec_item_t item, const uint16_t *w)
{
    agc_status_t st = AGC_STATUS_OK; /* first refusal */

    switch ((int) item)
    {
        case EC_F_TIME:
            agc_time_set_cs(((uint32_t) (w[0] & F_LOW14) << F_TIME_SH) | (uint32_t) (w[1] & F_LOW14)); /* TIME2, TIME1 */
            break;
        case EC_F_COUNTERS:
            agc_erase_set_cdux(w[0]);
            agc_erase_set_cduy(w[1]);
            agc_erase_set_cduz(w[2]);
            agc_erase_set_cdut(w[3]);
            agc_erase_set_cdus(w[4]);
            agc_erase_set_pipax(ec_sp_of_word(w[5]));
            agc_erase_set_pipay(ec_sp_of_word(w[6]));
            agc_erase_set_pipaz(ec_sp_of_word(w[7]));
            break;
        case EC_F_OPTMODES:
            agc_erase_set_optmodes(w[0]);
            break;
        case EC_F_LANDMARK:
            agc_erase_set_landmark(ec_sp_of_word(w[0]));
            break;
        case EC_F_CMDAPMOD:
            agc_entry_state()->cmdapmod = cmdapmod_of(w[0]);
            break;
        case EC_F_CDUXD:
            st                          = lm_dap_store(item, w);
            break;
        case EC_F_OMEGAPD:
            st                          = lm_dap_store(item, w);
            break;
        default:
            st                          = load_store(item, w);                                         /* the words an image load restores, or a read-only owner */
            break;
    } /* switch */

    return (st);
} /* value_store */

/**********************************************************************
 * AGC: none -- store a field item from its words (signs free to
 * disagree)
 *
 * ARGUMENTS:
 *   item   ec_item_t         I   a field item
 *   words  int               I   its words (the map entry)
 *   w      const uint16_t *  I   the words
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or an owner setter's status
 **********************************************************************/
agc_status_t ec_field_store(ec_item_t item, int words, const uint16_t *w)
{
    int          n  = (ec_field_layout(item) == EC_DP) ? 2 : 1; /* words per element */
    fref_t       r;                                             /* element reference */
    agc_status_t st = AGC_STATUS_OK;                            /* outcome */

    if (!ref_of(item, 0, &r))
    {
        st = value_store(item, w);
    }
    else
    {
        for (int k = 0; k * n < words && ref_of(item, k, &r); k++)
        {
            if (r.kind == FK_SP)
            {
                *r.sp = ec_sp_of_word(w[k]);
            }
            else if (r.kind == FK_RAW)
            {
                *r.raw = w[k];
            }
            else if (r.kind == FK_DP)
            {
                *r.dp = ec_dp_of(&w[2 * k]);
            }
            else
            {
                r.dpw->hi = ec_sp_of_word(w[2 * k]);
                r.dpw->lo = ec_sp_of_word(w[2 * k + 1]);
            }
        }
    }

    return (st);
} /* ec_field_store */
