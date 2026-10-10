/*
 * jet_select.c -- RCS-CSM jet selection logic and T6 jet timing.
 * JET_SELECTION_LOGIC.agc (Comanche055 pp. 1039-1062): JETSLECT, NOXLNCMD,
 * PWORD, TABPCOM, YWORD, TABYCOM, RWORD, ACROLL, RXLNS, TABRCOM, BDZCHECK,
 * TAUCHECK, NOBDZ, BDROLL, RZXLNS, TABRZCMD, ACYCHECK, TAUCHCK, NOACY,
 * PITCHTIM, YAWTIME, ROLLTIME, RBLAST, ASMBLWP, PBYPASS, ASMBLWR, ACBD2Y,
 * AC2Y, ACBD2Z, BDF2Z, RBYPASS, ASMBLWY, YBYPASS, T6SETUP, DXCHT12,
 * DXCHT23, CALCDT6, ENDJETS; T6START, T6RUPTOR, REPLACE*, ZBLAST*, ENABT6.
 *
 * Quads: A and C carry the pitch jets (channel 5 bits 1-4) and B and D the
 * yaw jets (bits 5-8); roll jets are on channel 6 (AC roll bits 1-4, BD
 * roll bits 5-8). X-translation uses pitch or yaw jets, Y-translation the
 * AC roll jets, Z-translation the BD roll jets. The rotation commands TAU
 * (one-jet on-times, 1/1600 s) are divided by the number of jets used,
 * limited to 0.1 s (one T5 cycle) and to at least 14 ms; DFT returns the
 * expected one-jet firing time to the rate filter. The second words
 * (xWORD2) continue the translations after the rotations end; BLAST holds
 * the sorted T6 intervals with the branch index in its low word.
 *
 * agc_csm_jet_select() is the pure table logic (JETSLECT after the timer
 * set-up through CALCDT6); agc_task_jetslect() is the T5 program around it.
 */
#include "../internal.h"

#define XLNMASK ((uint16_t) 007700) /* XLNMASK OCT 7700: THC bits 7-12     */
#define PJETS   ((uint16_t) 001417) /* pitch, X-translation jets + count   */
#define YJETS   ((uint16_t) 006360) /* yaw, X-translation jets + count     */
#define ACRJETS ((uint16_t) 003760) /* RTABLE mask, AC roll                */
#define BDRJETS ((uint16_t) 034017) /* RTABLE mask, BD roll                */
#define BDZJETS ((uint16_t) 003417) /* YZTABLE mask, BD Z-translation      */
#define ACYJETS ((uint16_t) 034360) /* YZTABLE mask, AC Y-translation      */
#define TENTH   ((agc_sp_t) 160)    /* =+.1SEC DEC 160 (1/1600 s)          */
#define MIN14MS ((agc_sp_t) 23)     /* =14MS DEC 23                        */
#define T5_60MS ((uint16_t) 037772) /* DELTATT3 DEC 16378 = 60 MS          */
#define T5_20MS ((uint16_t) 037776) /* DELATT20 DEC 16382 = 20 MS          */
#define T6_14MS ((uint16_t) 23)     /* =14MS: initialise jet channels      */
#define NJ_BIAS 3                   /* NJET, DFTMAX indexed -3 .. +3       */
#define NO_ATTK ((agc_sp_t) 0x7FFF) /* attkalmn_xln: ATTKALMN unchanged    */

/* PYTABLE (p. 1042): bits 4-1 pitch/X jets, 10,9 number of pitch jets;
 * bits 8-5 yaw/X jets, 12,11 number of yaw jets.  ROT TRANS QUAD BIAS    */
static const uint16_t pytable[15] = {
    00000, 05125, 05252, /* 0/+/-   0     -     0                  */
    00231, 02421, 02610, /* 0/+/-   +     -     3                  */
    00146, 02504, 02442, /* 0/+/-   -     -     6                  */
    00000, 02421, 02442, /* 0/+/-   -    A(B)   9                  */
    00000, 02504, 02610  /* 0/+/-   -    C(D)  12                  */
};
/* RTABLE (p. 1047): AC roll word (bits 9,10,11 net roll torque, coded
 * 0..4 = -2..+2 jets) and BD roll word (bits 12,13,14) combined.         */
static const uint16_t rtable[15] = {
    011000, 022125, 000252, /* 0/+/-   0     -     0                  */
    011231, 015421, 004610, /* 0/+/-  +Y(+Z) -     3                  */
    011146, 015504, 004442, /* 0/+/-  -Y(-Z) -     6                  */
    011000, 015504, 004610, /* 0/+/-   -    A(B)   9                  */
    011000, 015421, 004442  /* 0/+/-   -    C(D)  12                  */
};
/* YZTABLE (p. 1049): Y-Z translation words, coded like RTABLE.           */
static const uint16_t yztable[9] = {
    011000, 011231, 011146, /* 0, +Z(+Y), -Z(-Y); no failure     0    */
    011000, 004610, 015504, /* B(A) failed                        3    */
    011000, 015421, 004442  /* D(C) failed                        6    */
};
static const int xlnndx[4]  = {0, 3, 6, 0}; /* XLNNDX                  */
static const int xln1ndx[4] = {0, 1, 2, 0}; /* XLN1NDX                 */
/* NJET -3 .. +3 (p. 1054): 1/(number of jets), sign of the rotation.     */
static const agc_sp_t njet[7] = {-5461, -8192, -16383, 0, 16383, 8192, 5461};
/* DFTMAX -3 .. +3 (p. 1053): 0.1 s times the number of jets.             */
static const agc_sp_t dftmax[7] = {-480, -320, -160, 0, 160, 320, 480};

typedef struct js_work
{                                 /* RCS interrupt true temporaries          */
    int xndx1, xndx2, yndx, zndx; /* 0 none, 1 +, 2 -, 3 both            */
    int pindex, yindex, rindex;   /* 0 none, 1 +, 2 - rotation           */
    int npjets, nyjets, nrjets;   /* jets used (NRJETS: net, signed)     */
} js_work_t;

static int rot_index(agc_sp_t tau) /* CCS TAU: 0, +1, +2                  */
{
    if (!AGC_ASSERT(tau >= AGC_SP_MIN))
    {
        return 0; /* no valid result */
    }
    return (tau > 0) ? 1 : ((tau < 0) ? 2 : 0);
}

static int fail_base(agc_sp_t fail, int pos, int neg) /* CCS xxFAIL       */
{
    if (!AGC_ASSERT(pos >= 0))
    {
        return 0; /* no valid result */
    }
    return (fail > 0) ? pos : ((fail < 0) ? neg : 0);
}

static int nj(int n) /* NJET/DFTMAX table index             */
{
    if (!AGC_ASSERT((n >= -NJ_BIAS) && (n <= NJ_BIAS)))
    {
        n = (n < 0) ? -NJ_BIAS : NJ_BIAS;
    }
    return n + NJ_BIAS;
}

/* JETSLECT .. NOXLNCMD (pp. 1039-1040): translation command indices.     */
static void js_translation(const agc_csm_jet_in_t *in, js_work_t *w,
                           agc_csm_jet_out_t *out)
{
    unsigned x = ((unsigned) in->chan31 ^ XLNMASK) & XLNMASK; /* RXOR CHAN31 */
    w->xndx1   = (int) ((x >> 6) & 3u); /* +X 1, -X 2 (MP BIT9; MASK THREE)  */
    w->xndx2   = w->xndx1;
    w->yndx    = (int) ((x >> 8) & 3u);  /* MP BIT7                           */
    w->zndx    = (int) ((x >> 10) & 3u); /* MP BIT5                           */
    out->attkalmn_xln = NO_ATTK;
    if (x != 0u)
    {
        out->attkalmn_xln = in->lem_on ? -3 : -2; /* translation gains     */
        if (in->xtrans > 0)
        {
            w->xndx1 = 0; /* using BD-X                        */
        }
        else if (in->xtrans < 0)
        {
            w->xndx2 = 0; /* using AC-X                        */
        }
        else
        {
            /* both quads translate in X */
        }
    }
    if (!AGC_ASSERT(w->zndx <= 3))
    {
        return;
    }
}

/* PWORD, TABPCOM, YWORD, TABYCOM (pp. 1040-1042).                        */
static void js_pitch_yaw(const agc_csm_jet_in_t *in, js_work_t *w,
                         agc_csm_jet_out_t *out)
{
    w->pindex   = rot_index(in->tau[1]);
    int a       = fail_base(in->racfail, 9, 12); /* AFAILP / CFAILP       */
    a           = ((a == 0) ? xlnndx[w->xndx1] : a) + w->pindex;
    out->pword1 = (uint16_t) (pytable[a] & PJETS);
    w->npjets   = (int) (out->pword1 >> 8); /* MP BIT7: bits 10,9    */
    w->yindex   = rot_index(in->tau[2]);
    a           = fail_base(in->rbdfail, 9, 12); /* BFAILY / DFAILY       */
    a           = ((a == 0) ? xlnndx[w->xndx2] : a) + w->yindex;
    out->yword1 = (uint16_t) (pytable[a] & YJETS);
    w->nyjets   = (int) (out->yword1 >> 10); /* MP BIT5: bits 12,11   */
    if (!AGC_ASSERT(w->npjets <= 3))
    {
        return;
    }
}

/* BDZCHECK .. NOBDZ (pp. 1043-1044), with AC roll: BD Z-translation is
 * included if the net roll stays non-zero, or if no roll is commanded.   */
static void js_bdz(const agc_csm_jet_in_t *in, js_work_t *w,
                   agc_csm_jet_out_t *out)
{
    if (w->zndx > 0)
    {
        int      a = fail_base(in->rbdfail, 3, 6) + xln1ndx[w->zndx];
        unsigned t = (unsigned) (yztable[a] & BDZJETS) + out->rword1;
        w->nrjets  = (int) (t >> 8) - 4; /* net roll jets         */
        if ((w->nrjets != 0) || (in->tau[0] == 0))
        {                               /* TAUCHECK        */
            out->rword1 = (uint16_t) t; /* ACRBDZ          */
            return;
        }
    }
    w->nrjets = (int) (out->rword1 >> 8) - 2; /* NOBDZ                 */
    if (!AGC_ASSERT(w->nrjets >= -4))
    {
        return;
    }
}

/* ACYCHECK .. NOACY (pp. 1045-1046), with BD roll and AC Y-translation. */
static void js_acy(const agc_csm_jet_in_t *in, js_work_t *w,
                   agc_csm_jet_out_t *out)
{
    if (w->yndx != 0)
    {
        int      a = fail_base(in->racfail, 3, 6) + xln1ndx[w->yndx];
        unsigned t = (unsigned) (yztable[a] & ACYJETS) + out->rword1;
        w->nrjets  = (int) (t >> 11) - 4; /* MP BIT4               */
        if ((w->nrjets != 0) || (in->tau[0] == 0))
        {                               /* TAUCHCK         */
            out->rword1 = (uint16_t) t; /* BDRACZ          */
            return;
        }
    }
    w->nrjets = (int) (out->rword1 >> 11) - 2; /* NOACY                 */
    if (!AGC_ASSERT(w->nrjets >= -4))
    {
        return;
    }
}

/* RWORD, ACROLL/BDROLL, TABRCOM/TABRZCMD (pp. 1043-1045).                */
static void js_roll(const agc_csm_jet_in_t *in, js_work_t *w,
                    agc_csm_jet_out_t *out)
{
    w->rindex = rot_index(in->tau[0]);
    if (in->acorbd < 0)
    { /* ACROLL                */
        int a = fail_base(in->racfail, 9, 12);
        a     = ((a == 0) ? xlnndx[w->yndx] : a) + w->rindex; /* RXLNS        */
        out->rword1 = (uint16_t) (rtable[a] & ACRJETS);
        js_bdz(in, w, out);
    }
    else
    { /* BDROLL (+, +0)        */
        int a = fail_base(in->rbdfail, 9, 12);
        a     = ((a == 0) ? xlnndx[w->zndx] : a) + w->rindex; /* RZXLNS       */
        out->rword1 = (uint16_t) (rtable[a] & BDRJETS);
        js_acy(in, w, out);
    }
    if (!AGC_ASSERT(w->rindex <= 2))
    {
        return;
    }
}

/* RBLAST/PTAUPOS/YTAUPOS and the .1 s and 14 ms limits (pp. 1050-1053):
 * one axis with a non-zero rotation command and n signed jets.           */
static void js_ontime(agc_sp_t *tau, int n, agc_sp_t *blast, agc_sp_t *dft)
{
    int k  = nj(n);
    *blast = csm_dap_mp_hi(*tau, njet[k]); /* TAU / jets            */
    if (*blast - TENTH > 0)
    { /* longer than .1 s      */
        *dft   = dftmax[k];
        *tau   = csm_dap_ads(*tau, (agc_sp_t) - *dft); /* update TAU          */
        *blast = TENTH; /* only translations continue on switch-over */
        return;
    }
    if (MIN14MS - *blast > 0)
    {
        *blast = MIN14MS; /* minimum impulse       */
    }
    *dft = csm_dap_mp_lo(*blast, (agc_sp_t) n); /* LXCH DFT              */
    *tau = csm_dap_mp_hi(*blast, (agc_sp_t) n); /* zero TAU              */
    if (!AGC_ASSERT(*tau == 0))
    {
        return;
    }
}

/* ASMBLWR .. BDF2Z (pp. 1056-1057): second roll word RWORD2 with the Y
 * and Z translations that continue after the roll firing.                */
static void js_rword2(const agc_csm_jet_in_t *in, js_work_t *w,
                      agc_csm_jet_out_t *out)
{
    agc_sp_t rest =
        (agc_sp_t) (TENTH - out->blast[0].time); /* CS BLAST; AD .1 */
    out->rword2 = 0u;                            /* NO2Y        */
    if (w->yndx > 0)
    { /* ACBD2Y      */
        if (in->acorbd >= 0)
        { /* AC2Y        */
            int a       = fail_base(in->racfail, 3, 6) + xln1ndx[w->yndx];
            out->rword2 = (uint16_t) (yztable[a] & ACYJETS);
            w->nrjets   = (int) (out->rword2 >> 11) - 2;
            out->dft[0] = csm_dap_ads(
                out->dft[0], csm_dap_mp_lo(rest, (agc_sp_t) w->nrjets));
        }
        else if (in->racfail == 0)
        { /* AC roll ok  */
            out->rword2 = (uint16_t) (rtable[xlnndx[w->yndx]] & ACRJETS);
        }
        else
        {
            /* using AC and AC has failed: no Y */
        }
    }
    if (w->zndx > 0)
    { /* ACBD2Z      */
        if (in->acorbd >= 0)
        { /* BDF2Z       */
            if (in->rbdfail == 0)
            {
                out->rword2 = (uint16_t) (out->rword2 +
                                          (rtable[xlnndx[w->zndx]] & BDRJETS));
            }
        }
        else
        {
            int a       = fail_base(in->rbdfail, 3, 6) + xln1ndx[w->zndx];
            out->rword2 = (uint16_t) (out->rword2 + (yztable[a] & BDZJETS));
            w->nrjets   = (int) (out->rword2 >> 8) - 2;
            out->dft[0] = csm_dap_ads(
                out->dft[0], csm_dap_mp_lo(rest, (agc_sp_t) w->nrjets));
        }
    }
    if (!AGC_ASSERT(out->rword2 <= CSM_DAP_NEG0))
    {
        return;
    }
}

/* ROLLTIME, PITCHTIM, YAWTIME with ASMBLW* / *BYPASS (pp. 1050-1058).     */
static void js_times(const agc_csm_jet_in_t *in, js_work_t *w,
                     agc_csm_jet_out_t *out)
{
    unsigned  mask[2]  = {PJETS, YJETS};
    int       xn[2]    = {w->xndx1, w->xndx2};
    agc_sp_t  fail[2]  = {in->racfail, in->rbdfail};
    uint16_t *w1[2]    = {&out->pword1, &out->yword1};
    uint16_t *w2[2]    = {&out->pword2, &out->yword2};
    int       njets[2] = {w->npjets, w->nyjets};
    if (out->tau[0] == 0)
    {                                        /* RBYPASS     */
        out->dft[0] = dftmax[nj(w->nrjets)]; /* update DFT even so      */
        out->rword2 = out->rword1;
        out->blast[0].time = 0;
    }
    else
    {
        js_ontime(&out->tau[0], w->nrjets, &out->blast[0].time, &out->dft[0]);
        js_rword2(in, w, out);
    }
    for (int k = 0; k < 2; k++)
    { /* pitch, yaw  */
        int ax = k + 1;
        if (out->tau[ax] == 0)
        { /* xBYPASS     */
            out->dft[ax]        = 0;
            *w2[k]              = *w1[k];
            out->blast[ax].time = 0;
            continue;
        }
        int n = (out->tau[ax] < 0) ? -njets[k] : njets[k]; /* xTAUNEG     */
        js_ontime(&out->tau[ax], n, &out->blast[ax].time, &out->dft[ax]);
        int a  = (fail[k] != 0) ? 0 : xlnndx[xn[k]]; /* FPX2/FYX2   */
        *w2[k] = (uint16_t) (pytable[a] & mask[k]);
    }
    if (!AGC_ASSERT(out->blast[0].time >= 0))
    {
        return;
    }
}

static void js_swap(agc_csm_blast_t *b, int i, int j) /* DXCH x3     */
{
    if (!AGC_ASSERT((i >= 0) && (j < AGC_CSM_AXES)))
    {
        return;
    }
    agc_csm_blast_t t = b[i];
    b[i]              = b[j];
    b[j]              = t;
}

/* T6SETUP, DXCHT12, CHECKT23, DXCHT23, CALCDT6 (pp. 1059-1060): sort the
 * on-times and turn them into successive T6 intervals.                   */
static void js_sort(agc_csm_blast_t *b)
{
    b[0].axis = AGC_CSM_JET_ROLL; /* branch indices        */
    b[1].axis = AGC_CSM_JET_PITCH;
    b[2].axis = AGC_CSM_JET_YAW;
    if (b[1].time - b[0].time <= 0)
    {
        js_swap(b, 0, 1); /* T1 GR T2              */
    }
    if (b[2].time - b[1].time <= 0)
    {
        js_swap(b, 1, 2);
        if (b[1].time - b[0].time <= 0)
        {
            js_swap(b, 0, 1);
        }
    }
    b[2].time = (agc_sp_t) (b[2].time - b[1].time);
    b[1].time = (agc_sp_t) (b[1].time - b[0].time); /* end of sorting       */
    if (!AGC_ASSERT(b[1].time >= 0))
    {
        return;
    }
}

/* The jet selection logic proper (JETSLECT after the timers .. CALCDT6). */
agc_status_t agc_csm_jet_select(const agc_csm_jet_in_t *in,
                                agc_csm_jet_out_t      *out)
{
    if (!AGC_ASSERT((in != NULL) && (out != NULL)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT((in->racfail >= -1) && (in->racfail <= 1) &&
                    (in->rbdfail >= -1) && (in->rbdfail <= 1)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    js_work_t w;
    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        out->tau[i]        = in->tau[i];
        out->dft[i]        = 0;
        out->blast[i].time = 0;
    }
    js_translation(in, &w, out);
    js_pitch_yaw(in, &w, out);
    js_roll(in, &w, out);
    js_times(in, &w, out);
    js_sort(out->blast);
    return AGC_STATUS_OK;
}

/* JETSLECT (p. 1039) and ENDJETS (p. 1059): T5 program phase 3.          */
agc_status_t agc_task_jetslect(void)
{
    agc_csm_rcs_t    *r = agc_csm_dap_rcs();
    agc_csm_dapcom_t *c = agc_csm_dap_com();
    int t = (int) T5_60MS + (int) r->t5time; /* reset to execute phase 1 */
    agc_status_t st =
        csm_dap_set_time5((t > AGC_SP_MAX) ? T5_20MS : (uint16_t) t);
    if (st == AGC_STATUS_OK)
    { /* jet channels in 14 ms */
        agc_rupt_set_time(CSM_DAP_TIME6, T6_14MS);
        st = AGC_STATUS_OK;
    }
    if (st == AGC_STATUS_OK)
    {
        agc_chan_or(AGC_CHAN_CHAN13, (uint16_t) AGC_CH013_ENBT6RUPT);
        st = AGC_STATUS_OK;
    }
    agc_csm_jet_in_t  in;
    agc_csm_jet_out_t out;
    in.chan31 = agc_chan_read(AGC_CHAN_CHAN31);
    in.lem_on = (agc_erase_dapdatr1() & CSM_DAP_BIT14) != 0u;
    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        in.tau[i] = r->tau[i];
    }
    in.racfail      = r->racfail;
    in.rbdfail      = r->rbdfail;
    in.acorbd       = r->acorbd;
    in.xtrans       = r->xtrans;
    agc_status_t s2 = agc_csm_jet_select(&in, &out);
    if ((st != AGC_STATUS_OK) || (s2 != AGC_STATUS_OK))
    {
        return (st != AGC_STATUS_OK) ? st : s2;
    }
    if (out.attkalmn_xln != NO_ATTK)
    {
        r->attkalmn = out.attkalmn_xln;
    }
    r->rword1 = out.rword1;
    r->rword2 = out.rword2;
    r->pword1 = out.pword1;
    r->pword2 = out.pword2;
    r->yword1 = out.yword1;
    r->yword2 = out.yword2;
    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        r->tau[i]   = out.tau[i];
        r->dft[i]   = out.dft[i];
        r->blast[i] = out.blast[i];
    }
    c->rcsflags = (uint16_t) (c->rcsflags & ~(unsigned) AGC_CSM_RCSF_T6INIT);
    r->t5phase  = AGC_CSM_T5P_PHASE1;             /* reset for phase 1     */
    agc_rupt_set_t5_task(AGC_TASK_RCSATT);
    return AGC_STATUS_OK; /* RCS2CADR             */
}

/* REPLACER / REPLACEP / REPLACEY (p. 1062): the second word of an axis
 * replaces the first when its rotation is complete.                      */
static agc_status_t t6_replace(agc_csm_jet_axis_t axis)
{
    const agc_csm_rcs_t *r   = agc_csm_dap_rcs();
    uint16_t             ch5 = agc_chan_read(AGC_CHAN_PYJETS);
    agc_status_t         st  = AGC_STATUS_BAD_ARG;
    switch (axis)
    {
        case AGC_CSM_JET_ROLL:
            agc_chan_write(AGC_CHAN_ROLLJETS, r->rword2);
            break;
        case AGC_CSM_JET_PITCH:
            agc_chan_write(AGC_CHAN_PYJETS,
                                (uint16_t) ((ch5 & YJETS) + r->pword2));
            break;
        case AGC_CSM_JET_YAW:
            agc_chan_write(AGC_CHAN_PYJETS,
                                (uint16_t) ((ch5 & PJETS) + r->yword2));
            st = AGC_STATUS_OK;
            break;
        default:
            break; /* unreachable: cannot happen */
    }
    return st;
}

/* T6START, T6RUPTOR, T6L1, T6L2, REPLACE*, ZBLAST*, ENABT6 (pp. 1061-1062). */
agc_status_t agc_task_t6start(void)
{
    agc_csm_rcs_t    *r  = agc_csm_dap_rcs();
    agc_csm_dapcom_t *c  = agc_csm_dap_com();
    uint16_t          t6 = agc_rupt_time(CSM_DAP_TIME6);
    if ((t6 != 0u) && (t6 != CSM_DAP_NEG0))
    {
        return AGC_STATUS_OK; /* TIME6 reset after T6RUPT (in T5RUPT): wait */
    }
    agc_status_t st = AGC_STATUS_OK;
    if ((c->rcsflags & AGC_CSM_RCSF_T6INIT) == 0u)
    { /* initialise channels */
        c->rcsflags = (uint16_t) (c->rcsflags | AGC_CSM_RCSF_T6INIT);
        agc_chan_write(AGC_CHAN_ROLLJETS, r->rword1);
        agc_chan_write(AGC_CHAN_PYJETS, (uint16_t) (r->pword1 + r->yword1));
        agc_status_t s2 = AGC_STATUS_OK;
        st = (st == AGC_STATUS_OK) ? s2 : st;
    }
    for (int k = 0; (k < AGC_CSM_AXES) && (st == AGC_STATUS_OK); k++)
    {
        agc_sp_t b = r->blast[k].time;
        if (b > 0)
        { /* ZBLAST: wait b    */
            r->blast[k].time = 0;
            agc_rupt_set_time(CSM_DAP_TIME6, (uint16_t) b);
            st = AGC_STATUS_OK;
            agc_chan_or(AGC_CHAN_CHAN13,
                             (uint16_t) AGC_CH013_ENBT6RUPT);
            return AGC_STATUS_OK; /* ENABT6: RESUME    */
        }
        if (b == 0)
        { /* REPLACE           */
            st               = t6_replace(r->blast[k].axis);
            r->blast[k].time = -1;
        }
    }
    return st;
}
