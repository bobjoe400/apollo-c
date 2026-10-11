/**********************************************************************
 * FILE NAME: jet_select.c
 *
 * PURPOSE:
 *   RCS-CSM jet selection logic (phase 3 of the T5 program) and the T6
 *   jet timing. Quads A and C carry the pitch jets (channel 5 bits 1-4)
 *   and B and D the yaw jets (bits 5-8); roll jets are on channel 6 (AC
 *   roll bits 1-4, BD roll bits 5-8). X-translation uses pitch or yaw
 *   jets, Y-translation the AC roll jets, Z-translation the BD roll jets.
 *   The rotation commands TAU (one-jet on-times, 1/1600 s) are divided by
 *   the number of jets used, limited to 0.1 s (one T5 cycle) and to at
 *   least 14 ms; DFT returns the expected one-jet firing time to the rate
 *   filter. The second words (xWORD2) continue the translations after the
 *   rotations end; BLAST holds the sorted T6 intervals with the branch
 *   index in its low word. Labels: JETSLECT, NOXLNCMD, PWORD, TABPCOM,
 *   YWORD, TABYCOM, RWORD, ACROLL, RXLNS, TABRCOM, BDZCHECK, TAUCHECK,
 *   NOBDZ, BDROLL, RZXLNS, TABRZCMD, ACYCHECK, TAUCHCK, NOACY, PITCHTIM,
 *   YAWTIME, ROLLTIME, RBLAST, ASMBLWP, PBYPASS, ASMBLWR, ACBD2Y, AC2Y,
 *   ACBD2Z, BDF2Z, RBYPASS, ASMBLWY, YBYPASS, T6SETUP, DXCHT12, DXCHT23,
 *   CALCDT6, ENDJETS; T6START, T6RUPTOR, T6L1, T6L2, REPLACE*, ZBLAST*,
 *   ENABT6.
 *
 * AGC SOURCE:
 *   Comanche055 JET_SELECTION_LOGIC.agc pp. 1039-1062
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   TAU..TAU2     agc_sp_t  I/O  1/1600 s       rotation commands
 *   RACFAIL, RBDFAIL, ACORBD, XTRANS  agc_sp_t  I  unscaled  S41.2
 *   xWORD1/2      uint16_t  O    bits           channel 5/6 images
 *   BLAST..       struct    I/O  1/1600 s       T6 intervals, index
 *   DFT..DFT2     agc_sp_t  O    1/1600 s       expected firing times
 *   ATTKALMN      agc_sp_t  O    index          translation gains
 *   T5TIME        agc_sp_t  I    10 ms          delay compensation
 *   CHAN31        uint16_t  I    bits           THC bits 7-12
 *   CHAN5, CHAN6  uint16_t  I/O  bits           jet channels
 *   CHAN13        uint16_t  O    bit 15         T6RUPT enable
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   csm_dap_ads, _mp_hi,  AD/ADS, MP (mysubs.c)
 *   _mp_lo
 *   agc_rupt_set_time     TIME5, TIME6
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none in the AGC. AGC_STATUS_BAD_ARG for missing jet selection
 *   arguments or a failure word outside -1 .. +1.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   agc_csm_jet_select() is the pure table logic (JETSLECT after the
 *   timer set-up through CALCDT6); agc_task_jetslect() is the T5 program
 *   around it; agc_task_t6start() the T6 interrupt.
 **********************************************************************/
#include "../internal.h"

#define XLNMASK ((uint16_t) 007700) /* XLNMASK OCT 7700: THC bits 7-12     */
#define PJETS   ((uint16_t) 001417) /* pitch, X-translation jets + count   */
#define YJETS   ((uint16_t) 006360) /* yaw, X-translation jets + count     */
#define ACRJETS ((uint16_t) 003760) /* RTABLE mask, AC roll                */
#define BDRJETS ((uint16_t) 034017) /* RTABLE mask, BD roll                */
#define BDZJETS ((uint16_t) 003417) /* YZTABLE mask, BD Z-translation      */
#define ACYJETS ((uint16_t) 034360) /* YZTABLE mask, AC Y-translation      */
#define TENTH   ((agc_sp_t) 160)    /* =+.1SEC DEC 160, unscaled 1/1600 s  */
#define MIN14MS ((agc_sp_t) 23)     /* =14MS DEC 23, unscaled 1/1600 s     */
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

static const int xlnndx[4]  = {0, 3, 6, 0}; /* XLNNDX: table row by translation  */
static const int xln1ndx[4] = {0, 1, 2, 0}; /* XLN1NDX: YZTABLE column           */

/* NJET -3 .. +3 (p. 1054): 1/(number of jets), sign of the rotation.     */
static const agc_sp_t njet[7] = {-5461, -8192, -16383, 0, 16383, 8192, 5461};

/* DFTMAX -3 .. +3 (p. 1053): 0.1 s times the number of jets, 1/1600 s.   */
static const agc_sp_t dftmax[7] = {-480, -320, -160, 0, 160, 320, 480};

/* The RCS interrupt true temporaries of one jet selection.               */
typedef struct js_work
{
    int xndx1;  /* XNDX1 AC quad X-translation: 0 none, 1 +, 2 -, 3 both */
    int xndx2;  /* XNDX2 BD quad X-translation                         */
    int yndx;   /* YNDX Y-translation                                  */
    int zndx;   /* ZNDX Z-translation                                  */
    int pindex; /* PINDEX pitch rotation: 0 none, 1 +, 2 -             */
    int yindex; /* YINDEX yaw rotation                                 */
    int rindex; /* RINDEX roll rotation                                */
    int npjets; /* NPJETS pitch jets used                              */
    int nyjets; /* NYJETS yaw jets used                                */
    int nrjets; /* NRJETS net roll jets, signed                        */
} js_work_t;

static int  rot_index(agc_sp_t tau);
static int  fail_base(agc_sp_t fail, int pos, int neg);
static void js_translation(const agc_csm_jet_in_t *in, js_work_t *w, agc_csm_jet_out_t *out);
static void js_pitch_yaw(const agc_csm_jet_in_t *in, js_work_t *w, agc_csm_jet_out_t *out);
static void js_bdz(const agc_csm_jet_in_t *in, js_work_t *w, agc_csm_jet_out_t *out);
static void js_acy(const agc_csm_jet_in_t *in, js_work_t *w, agc_csm_jet_out_t *out);
static void js_roll(const agc_csm_jet_in_t *in, js_work_t *w, agc_csm_jet_out_t *out);
static void js_ontime(agc_sp_t *tau, int n, agc_sp_t *blast, agc_sp_t *dft);
static void js_rword2(const agc_csm_jet_in_t *in, js_work_t *w, agc_csm_jet_out_t *out);
static void js_times(const agc_csm_jet_in_t *in, js_work_t *w, agc_csm_jet_out_t *out);
static void js_swap(agc_csm_blast_t *b, int i, int j);
static void js_sort(agc_csm_blast_t *b);
static void t6_replace(agc_csm_jet_axis_t axis);

/**********************************************************************
 * CCS TAUx (JET_SELECTION_LOGIC.agc pp. 1040-1043) -- the rotation
 * index of a command.
 *
 * ARGUMENTS:
 *   tau  agc_sp_t  I  TAU, TAU1 or TAU2, unscaled 1/1600 s
 *
 * RETURNS: int -- 0 none, 1 positive, 2 negative
 **********************************************************************/
static int rot_index(agc_sp_t tau)
{
    if (tau > 0)
    {
        return (1);
    }

    return ((tau < 0) ? 2 : 0); /* ?: CCS: -, else +0                  */
}

/**********************************************************************
 * CCS RACFAIL / RBDFAIL (JET_SELECTION_LOGIC.agc pp. 1040-1049) -- the
 * table row of a quad failure.
 *
 * ARGUMENTS:
 *   fail  agc_sp_t  I  +1 A (B) failed, -1 C (D) failed, 0 none
 *   pos   int       I  row for a positive failure word
 *   neg   int       I  row for a negative failure word
 *
 * RETURNS: int -- pos, neg, or 0 with no failure
 **********************************************************************/
static int fail_base(agc_sp_t fail, int pos, int neg)
{
    if (fail > 0)
    {
        return (pos);
    }

    return ((fail < 0) ? neg : 0); /* ?: CCS: -, else +0               */
}

/**********************************************************************
 * JETSLECT .. NOXLNCMD (JET_SELECTION_LOGIC.agc pp. 1039-1040) -- the
 * translation command indices from the THC (channel 31 bits 7-12, active
 * 0): X (MP BIT9), Y (MP BIT7), Z (MP BIT5); with a translation the
 * translation filter gains (ATTKALMN -2, or -3 with the LM on) and, for
 * a single-quad X-translation (XTRANS), only that quad's index.
 *
 * ARGUMENTS:
 *   in   const agc_csm_jet_in_t *  I  channel 31, LM on, XTRANS
 *   w    js_work_t *                O  XNDX1, XNDX2, YNDX, ZNDX
 *   out  agc_csm_jet_out_t *        O  ATTKALMN for translation
 **********************************************************************/
static void js_translation(const agc_csm_jet_in_t *in, js_work_t *w, agc_csm_jet_out_t *out)
{
    unsigned int x = ((unsigned int) in->chan31 ^ XLNMASK) & XLNMASK; /* CAF XLNMASK; RXOR CHAN31; MASK */

    w->xndx1          = (int) ((x >> 6) & 3u);  /* MP BIT9; MASK THREE: +X 1, -X 2 */
    w->xndx2          = w->xndx1;
    w->yndx           = (int) ((x >> 8) & 3u);  /* MP BIT7                          */
    w->zndx           = (int) ((x >> 10) & 3u); /* MP BIT5                          */
    out->attkalmn_xln = NO_ATTK;

    if (x != 0u)
    {
        out->attkalmn_xln = in->lem_on ? -3 : -2; /* ?: translation gains   */

        if (in->xtrans > 0)
        {
            w->xndx1 = 0; /* using BD-X                                  */
        }
        else if (in->xtrans < 0)
        {
            w->xndx2 = 0; /* using AC-X                                  */
        }
        else
        {
            /* both quads translate in X */
        }
    }
}

/**********************************************************************
 * PWORD, TABPCOM, YWORD, TABYCOM (JET_SELECTION_LOGIC.agc pp.
 * 1040-1042) -- the first pitch and yaw jet words from PYTABLE: the
 * failure row (AFAILP/CFAILP, BFAILY/DFAILY) or the X-translation row,
 * plus the rotation index; the jet counts from bits 10,9 and 12,11.
 *
 * ARGUMENTS:
 *   in   const agc_csm_jet_in_t *  I  TAU1, TAU2, RACFAIL, RBDFAIL
 *   w    js_work_t *                IO X indices in; PINDEX, YINDEX,
 *                                      NPJETS, NYJETS out
 *   out  agc_csm_jet_out_t *        O  PWORD1, YWORD1
 **********************************************************************/
static void js_pitch_yaw(const agc_csm_jet_in_t *in, js_work_t *w, agc_csm_jet_out_t *out)
{
    w->pindex = rot_index(in->tau[1]);       /* CCS TAU1                 */
    int a     = fail_base(in->racfail, 9, 12); /* AFAILP / CFAILP        */

    a           = ((a == 0) ? xlnndx[w->xndx1] : a) + w->pindex; /* ?: TABPCOM row */
    out->pword1 = (uint16_t) (pytable[a] & PJETS);
    w->npjets   = (int) (out->pword1 >> 8);      /* MP BIT7: bits 10,9    */
    w->yindex   = rot_index(in->tau[2]);         /* CCS TAU2              */
    a           = fail_base(in->rbdfail, 9, 12); /* BFAILY / DFAILY       */
    a           = ((a == 0) ? xlnndx[w->xndx2] : a) + w->yindex; /* ?: TABYCOM row */
    out->yword1 = (uint16_t) (pytable[a] & YJETS);
    w->nyjets   = (int) (out->yword1 >> 10); /* MP BIT5: bits 12,11       */
}

/**********************************************************************
 * BDZCHECK, TAUCHECK, NOBDZ (JET_SELECTION_LOGIC.agc pp. 1043-1044) --
 * with AC roll: the BD Z-translation is included if the net roll stays
 * non-zero, or if no roll is commanded; NRJETS the net roll jets.
 *
 * ARGUMENTS:
 *   in   const agc_csm_jet_in_t *  I  TAU, RBDFAIL
 *   w    js_work_t *                IO ZNDX in; NRJETS out
 *   out  agc_csm_jet_out_t *        IO RWORD1
 **********************************************************************/
static void js_bdz(const agc_csm_jet_in_t *in, js_work_t *w, agc_csm_jet_out_t *out)
{
    if (w->zndx > 0)
    {
        int          a = fail_base(in->rbdfail, 3, 6) + xln1ndx[w->zndx];   /* YZTABLE index    */
        unsigned int t = (unsigned int) (yztable[a] & BDZJETS) + out->rword1; /* AD RWORD1        */

        w->nrjets = (int) (t >> 8) - 4; /* net roll jets                 */

        if ((w->nrjets != 0) || (in->tau[0] == 0))
        { /* TAUCHECK */
            out->rword1 = (uint16_t) t; /* ACRBDZ                         */

            return;
        }
    }

    w->nrjets = (int) (out->rword1 >> 8) - 2; /* NOBDZ                   */
}

/**********************************************************************
 * ACYCHECK, TAUCHCK, NOACY (JET_SELECTION_LOGIC.agc pp. 1045-1046) --
 * with BD roll: the AC Y-translation is included if the net roll stays
 * non-zero, or if no roll is commanded; NRJETS the net roll jets.
 *
 * ARGUMENTS:
 *   in   const agc_csm_jet_in_t *  I  TAU, RACFAIL
 *   w    js_work_t *                IO YNDX in; NRJETS out
 *   out  agc_csm_jet_out_t *        IO RWORD1
 **********************************************************************/
static void js_acy(const agc_csm_jet_in_t *in, js_work_t *w, agc_csm_jet_out_t *out)
{
    if (w->yndx != 0)
    {
        int          a = fail_base(in->racfail, 3, 6) + xln1ndx[w->yndx];   /* YZTABLE index    */
        unsigned int t = (unsigned int) (yztable[a] & ACYJETS) + out->rword1; /* AD RWORD1        */

        w->nrjets = (int) (t >> 11) - 4; /* MP BIT4                     */

        if ((w->nrjets != 0) || (in->tau[0] == 0))
        { /* TAUCHCK */
            out->rword1 = (uint16_t) t; /* BDRACZ                         */

            return;
        }
    }

    w->nrjets = (int) (out->rword1 >> 11) - 2; /* NOACY                  */
}

/**********************************************************************
 * RWORD, ACROLL, RXLNS, TABRCOM, BDROLL, RZXLNS, TABRZCMD
 * (JET_SELECTION_LOGIC.agc pp. 1043-1045) -- the first roll word from
 * RTABLE with the AC (ACORBD -) or BD quads, then the other quads'
 * translation (BDZCHECK or ACYCHECK).
 *
 * ARGUMENTS:
 *   in   const agc_csm_jet_in_t *  I  TAU, ACORBD, RACFAIL, RBDFAIL
 *   w    js_work_t *                IO Y, Z indices in; RINDEX, NRJETS
 *   out  agc_csm_jet_out_t *        O  RWORD1
 **********************************************************************/
static void js_roll(const agc_csm_jet_in_t *in, js_work_t *w, agc_csm_jet_out_t *out)
{
    w->rindex = rot_index(in->tau[0]); /* CCS TAU                       */

    if (in->acorbd < 0)
    { /* ACROLL */
        int a = fail_base(in->racfail, 9, 12); /* CCS RACFAIL            */

        a           = ((a == 0) ? xlnndx[w->yndx] : a) + w->rindex; /* ?: RXLNS */
        out->rword1 = (uint16_t) (rtable[a] & ACRJETS);
        js_bdz(in, w, out);
    }
    else
    { /* BDROLL (+, +0) */
        int a = fail_base(in->rbdfail, 9, 12); /* CCS RBDFAIL            */

        a           = ((a == 0) ? xlnndx[w->zndx] : a) + w->rindex; /* ?: RZXLNS */
        out->rword1 = (uint16_t) (rtable[a] & BDRJETS);
        js_acy(in, w, out);
    }
}

/**********************************************************************
 * RBLAST, PTAUPOS, YTAUPOS and the .1 s and 14 ms limits
 * (JET_SELECTION_LOGIC.agc pp. 1050-1053) -- one axis with a non-zero
 * rotation command and n signed jets: BLAST = TAU / jets; over .1 s the
 * firing is .1 s, DFT = DFTMAX and TAU keeps the rest; otherwise at
 * least 14 ms, DFT = BLAST x jets (lower word) and TAU = +0.
 *
 * ARGUMENTS:
 *   tau    agc_sp_t *  IO  TAUx, unscaled 1/1600 s
 *   n      int         I   signed number of jets, -3 .. +3
 *   blast  agc_sp_t *  O   the firing time, unscaled 1/1600 s
 *   dft    agc_sp_t *  O   DFTx, unscaled 1/1600 s
 **********************************************************************/
static void js_ontime(agc_sp_t *tau, int n, agc_sp_t *blast, agc_sp_t *dft)
{
    int k = n + NJ_BIAS; /* NJET / DFTMAX index                       */

    *blast = csm_dap_mp_hi(*tau, njet[k]); /* INDEX; MP NJET: TAU / jets */

    if (*blast - TENTH > 0)
    { /* longer than .1 s */
        *dft   = dftmax[k];
        *tau   = csm_dap_ads(*tau, (agc_sp_t) -*dft); /* update TAU     */
        *blast = TENTH; /* only translations continue on switch-over  */

        return;
    }

    if (MIN14MS - *blast > 0)
    {
        *blast = MIN14MS; /* minimum impulse                            */
    }

    *dft = csm_dap_mp_lo(*blast, (agc_sp_t) n); /* LXCH DFT              */
    *tau = csm_dap_mp_hi(*blast, (agc_sp_t) n); /* zero TAU              */
}

/**********************************************************************
 * ASMBLWR, ACBD2Y, AC2Y, ACBD2Z, BDF2Z (JET_SELECTION_LOGIC.agc pp.
 * 1056-1057) -- the second roll word RWORD2 with the Y and Z
 * translations that continue after the roll firing; their jets add to
 * DFT for the rest of the .1 s.
 *
 * ARGUMENTS:
 *   in   const agc_csm_jet_in_t *  I  ACORBD, RACFAIL, RBDFAIL
 *   w    js_work_t *                IO Y, Z indices in; NRJETS
 *   out  agc_csm_jet_out_t *        IO BLAST in; RWORD2, DFT out
 **********************************************************************/
static void js_rword2(const agc_csm_jet_in_t *in, js_work_t *w, agc_csm_jet_out_t *out)
{
    agc_sp_t rest = (agc_sp_t) (TENTH - out->blast[0].time); /* CS BLAST; AD .1, unscaled 1/1600 s */

    out->rword2 = 0u; /* NO2Y                                         */

    if (w->yndx > 0)
    { /* ACBD2Y */
        if (in->acorbd >= 0)
        { /* AC2Y */
            int a = fail_base(in->racfail, 3, 6) + xln1ndx[w->yndx]; /* YZTABLE index */

            out->rword2 = (uint16_t) (yztable[a] & ACYJETS);
            w->nrjets   = (int) (out->rword2 >> 11) - 2;
            out->dft[0] = csm_dap_ads(out->dft[0], csm_dap_mp_lo(rest, (agc_sp_t) w->nrjets));
        }
        else if (in->racfail == 0)
        { /* AC roll ok */
            out->rword2 = (uint16_t) (rtable[xlnndx[w->yndx]] & ACRJETS);
        }
        else
        {
            /* using AC and AC has failed: no Y */
        }
    }

    if (w->zndx > 0)
    { /* ACBD2Z */
        if (in->acorbd >= 0)
        { /* BDF2Z */
            if (in->rbdfail == 0)
            {
                out->rword2 = (uint16_t) (out->rword2 + (rtable[xlnndx[w->zndx]] & BDRJETS));
            }
        }
        else
        {
            int a = fail_base(in->rbdfail, 3, 6) + xln1ndx[w->zndx]; /* YZTABLE index */

            out->rword2 = (uint16_t) (out->rword2 + (yztable[a] & BDZJETS));
            w->nrjets   = (int) (out->rword2 >> 8) - 2;
            out->dft[0] = csm_dap_ads(out->dft[0], csm_dap_mp_lo(rest, (agc_sp_t) w->nrjets));
        }
    }
} /* js_rword2 */

/**********************************************************************
 * ROLLTIME, PITCHTIM, YAWTIME, ASMBLWP, ASMBLWY, RBYPASS, PBYPASS,
 * YBYPASS (JET_SELECTION_LOGIC.agc pp. 1050-1058) -- the firing times
 * and second words of the three axes; an axis without a command keeps
 * its first word and fires 0.
 *
 * ARGUMENTS:
 *   in   const agc_csm_jet_in_t *  I  failures, ACORBD
 *   w    js_work_t *                IO indices and jet counts
 *   out  agc_csm_jet_out_t *        IO TAU in/out; BLAST, DFT, xWORD2
 **********************************************************************/
static void js_times(const agc_csm_jet_in_t *in, js_work_t *w, agc_csm_jet_out_t *out)
{
    unsigned int mask[2]  = {PJETS, YJETS};                 /* ASMBLWP / ASMBLWY masks */
    int          xn[2]    = {w->xndx1, w->xndx2};           /* X-translation indices   */
    agc_sp_t     fail[2]  = {in->racfail, in->rbdfail};     /* RACFAIL, RBDFAIL (unscaled) */
    uint16_t    *w1[2]    = {&out->pword1, &out->yword1};   /* first words             */
    uint16_t    *w2[2]    = {&out->pword2, &out->yword2};   /* second words            */
    int          njets[2] = {w->npjets, w->nyjets};         /* jets used               */

    if (out->tau[0] == 0)
    { /* RBYPASS */
        out->dft[0]        = dftmax[w->nrjets + NJ_BIAS]; /* update DFT even so */
        out->rword2        = out->rword1;
        out->blast[0].time = 0;
    }
    else
    {
        js_ontime(&out->tau[0], w->nrjets, &out->blast[0].time, &out->dft[0]);
        js_rword2(in, w, out);
    }

    for (int k = 0; k < 2; k++)
    { /* pitch, yaw */
        int ax = k + 1; /* TAU1, TAU2                                    */

        if (out->tau[ax] == 0)
        { /* xBYPASS */
            out->dft[ax]        = 0;
            *w2[k]              = *w1[k];
            out->blast[ax].time = 0;
            continue;
        }

        int n = (out->tau[ax] < 0) ? -njets[k] : njets[k]; /* ?: xTAUNEG */

        js_ontime(&out->tau[ax], n, &out->blast[ax].time, &out->dft[ax]);
        int a = (fail[k] != 0) ? 0 : xlnndx[xn[k]]; /* ?: FPX2 / FYX2    */

        *w2[k] = (uint16_t) (pytable[a] & mask[k]);
    }
}

/**********************************************************************
 * DXCH BLAST x3 (JET_SELECTION_LOGIC.agc p. 1060, DXCHT12 / DXCHT23)
 * -- exchange two BLAST entries with their branch indices.
 *
 * ARGUMENTS:
 *   b  agc_csm_blast_t *  IO  BLAST, BLAST1, BLAST2
 *   i  int                I   first entry
 *   j  int                I   second entry
 **********************************************************************/
static void js_swap(agc_csm_blast_t *b, int i, int j)
{
    agc_csm_blast_t t = b[i]; /* DXCH BLAST(i)                          */

    b[i] = b[j];
    b[j] = t;
}

/**********************************************************************
 * T6SETUP, DXCHT12, CHECKT23, DXCHT23, CALCDT6 (JET_SELECTION_LOGIC.agc
 * pp. 1059-1060) -- sort the on-times with their branch indices and turn
 * them into successive T6 intervals.
 *
 * ARGUMENTS:
 *   b  agc_csm_blast_t *  IO  BLAST, BLAST1, BLAST2
 **********************************************************************/
static void js_sort(agc_csm_blast_t *b)
{
    b[0].axis = AGC_CSM_JET_ROLL; /* branch indices                    */
    b[1].axis = AGC_CSM_JET_PITCH;
    b[2].axis = AGC_CSM_JET_YAW;

    if (b[1].time - b[0].time <= 0)
    {
        js_swap(b, 0, 1); /* T1 GR T2                                  */
    }

    if (b[2].time - b[1].time <= 0)
    {
        js_swap(b, 1, 2);

        if (b[1].time - b[0].time <= 0)
        {
            js_swap(b, 0, 1);
        }
    }

    b[2].time = (agc_sp_t) (b[2].time - b[1].time); /* CS BLAST1; ADS BLAST2 */
    b[1].time = (agc_sp_t) (b[1].time - b[0].time); /* CS BLAST; ADS BLAST1  */
}

/**********************************************************************
 * JETSLECT .. CALCDT6 (JET_SELECTION_LOGIC.agc pp. 1039-1060) -- the
 * jet selection logic proper as a pure function (no timers).
 *
 * ARGUMENTS:
 *   in   const agc_csm_jet_in_t *  I  commands, failures, channel 31
 *   out  agc_csm_jet_out_t *       O  jet words, BLASTs, DFTs, TAUs
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG for a
 *          missing argument or a failure word outside -1 .. +1
 **********************************************************************/
agc_status_t agc_csm_jet_select(const agc_csm_jet_in_t *in, agc_csm_jet_out_t *out)
{
    js_work_t w; /* the interrupt temporaries                          */

    if (!AGC_ASSERT((in != NULL) && (out != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT((in->racfail >= -1) && (in->racfail <= 1) && (in->rbdfail >= -1) && (in->rbdfail <= 1)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

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

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * JETSLECT, ENDJETS (JET_SELECTION_LOGIC.agc pp. 1039, 1059) -- T5
 * program phase 3: phase 1 in 60 ms + T5TIME (20 ms if that overflows),
 * T6 in 14 ms to initialise the jet channels, the jet selection, then
 * T5LOC = RCSATT, RCSFLAGS BIT1 reset, T5PHASE -0.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the T5 set-up or jet selection status
 **********************************************************************/
agc_status_t agc_task_jetslect(void)
{
    agc_csm_rcs_t    *r = agc_csm_dap_rcs();                 /* RCS DAP erasables       */
    agc_csm_dapcom_t *c = agc_csm_dap_com();                 /* RCSFLAGS                */
    int               t = (int) T5_60MS + (int) r->t5time; /* CAF DELTATT3; AD T5TIME, 10 ms */
    agc_csm_jet_in_t  in;                                  /* jet selection inputs    */
    agc_csm_jet_out_t out;                                 /* jet selection outputs   */

    agc_status_t st = csm_dap_set_time5((t > AGC_SP_MAX) ? T5_20MS : (uint16_t) t); /* ?: TS TIME5 overflow: DELATT20 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_rupt_set_time(CSM_DAP_TIME6, T6_14MS);                    /* CAF =14MS; TS TIME6 */
    agc_chan_or(AGC_CHAN_CHAN13, (uint16_t) AGC_CH013_ENBT6RUPT); /* CAF NEGMAX; WOR CHAN13 */
    in.chan31 = agc_chan_read(AGC_CHAN_CHAN31);
    in.lem_on = (agc_erase_dapdatr1() & CSM_DAP_BIT14) != 0u;

    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        in.tau[i] = r->tau[i];
    }

    in.racfail = r->racfail;
    in.rbdfail = r->rbdfail;
    in.acorbd  = r->acorbd;
    in.xtrans  = r->xtrans;
    st         = agc_csm_jet_select(&in, &out);

    if (st != AGC_STATUS_OK)
    {
        return (st);
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

    agc_rupt_set_t5_task(AGC_TASK_RCSATT);                                          /* DCA RCS2CADR; DXCH T5LOC */
    c->rcsflags = (uint16_t) (c->rcsflags & ~(unsigned int) AGC_CSM_RCSF_T6INIT); /* ENDJETS: CS BIT1; MASK   */
    r->t5phase  = AGC_CSM_T5P_RATE_FILTER;                                        /* CS ZERO; TS T5PHASE      */

    return (AGC_STATUS_OK); /* RESUME                                   */
} /* agc_task_jetslect */

/**********************************************************************
 * REPLACER, REPLACEP, REPLACEY (JET_SELECTION_LOGIC.agc p. 1062) -- the
 * second word of an axis replaces the first when its rotation is
 * complete (INDEX BLASTx +1; TC REPLACER).
 *
 * ARGUMENTS:
 *   axis  agc_csm_jet_axis_t  I  the branch index from BLASTx +1
 **********************************************************************/
static void t6_replace(agc_csm_jet_axis_t axis)
{
    const agc_csm_rcs_t *r   = agc_csm_dap_rcs();              /* jet words */
    uint16_t             ch5 = agc_chan_read(AGC_CHAN_PYJETS); /* CHAN5     */

    switch (axis)
    {
        case AGC_CSM_JET_ROLL:
            agc_chan_write(AGC_CHAN_ROLLJETS, r->rword2); /* CA RWORD2; WRITE CHAN6 */
            break;
        case AGC_CSM_JET_PITCH:
            agc_chan_write(AGC_CHAN_PYJETS, (uint16_t) ((ch5 & YJETS) + r->pword2)); /* RAND CHAN5; AD PWORD2 */
            break;
        case AGC_CSM_JET_YAW:
        default:
            agc_chan_write(AGC_CHAN_PYJETS, (uint16_t) ((ch5 & PJETS) + r->yword2)); /* RAND CHAN5; AD YWORD2 */
            break;
    }
}

/**********************************************************************
 * T6START, T6RUPTOR, T6L1, T6L2, REPLACE, REPLACE1, REPLACE2, ZBLAST*,
 * ENABT6 (JET_SELECTION_LOGIC.agc pp. 1061-1062) -- T6 interrupt: wait
 * if TIME6 was reset after the T6RUPT (in T5RUPT); on the first entry
 * initialise channels 5 and 6 with the first words; then for BLAST,
 * BLAST1, BLAST2 in turn: a positive interval is the next T6 wait
 * (ZBLAST: zero it, TIME6, enable T6RUPT, RESUME); a zero interval
 * means that axis' rotation is complete: its second word replaces the
 * first and the entry is set to -1.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (RESUME)
 **********************************************************************/
agc_status_t agc_task_t6start(void)
{
    agc_csm_rcs_t    *r  = agc_csm_dap_rcs();              /* jet words, BLASTs */
    agc_csm_dapcom_t *c  = agc_csm_dap_com();              /* RCSFLAGS          */
    uint16_t          t6 = agc_rupt_time(CSM_DAP_TIME6);   /* CCS TIME6         */

    if ((t6 != 0u) && (t6 != CSM_DAP_NEG0))
    {
        return (AGC_STATUS_OK); /* TIME6 reset after T6RUPT: wait        */
    }

    if ((c->rcsflags & AGC_CSM_RCSF_T6INIT) == 0u)
    { /* CS RCSFLAGS; MASK BIT1; BZF T6RUPTOR: initialise the channels */
        c->rcsflags = (uint16_t) (c->rcsflags | AGC_CSM_RCSF_T6INIT);
        agc_chan_write(AGC_CHAN_ROLLJETS, r->rword1);                       /* WRITE CHAN6 */
        agc_chan_write(AGC_CHAN_PYJETS, (uint16_t) (r->pword1 + r->yword1)); /* WRITE CHAN5 */
    }

    for (int k = 0; k < AGC_CSM_AXES; k++)
    { /* T6RUPTOR, T6L1, T6L2: CCS BLASTk */
        agc_sp_t b = r->blast[k].time; /* interval, unscaled 1/1600 s  */

        if (b > 0)
        { /* ZBLASTk: CAF ZERO; XCH BLASTk; ENABT6 */
            r->blast[k].time = 0;
            agc_rupt_set_time(CSM_DAP_TIME6, (uint16_t) b);                /* TS TIME6               */
            agc_chan_or(AGC_CHAN_CHAN13, (uint16_t) AGC_CH013_ENBT6RUPT); /* CAF NEGMAX; WOR CHAN13 */

            return (AGC_STATUS_OK); /* RESUME                           */
        }

        if (b == 0)
        { /* REPLACEk */
            t6_replace(r->blast[k].axis);
            r->blast[k].time = -1; /* CS ONE; TS BLASTk                  */
        }
    }

    return (AGC_STATUS_OK); /* RESUME                                   */
}
