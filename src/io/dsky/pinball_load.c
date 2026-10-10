/*
 * pinball_load.c -- PINBALL load verbs V21-V25 and the data request /
 * store sequence.
 *
 * PINBALL_GAME_BUTTONS_AND_LIGHTS.agc CM pp. 328, 343-348 (LM pp. 410,
 * 425-430): REQDATX, REQDATY, REQDATZ, REQCOM, ABCLOAD, PUTXYZ (with the
 * noun 07 flag-bit set/reset), ABLOAD, PUTXY, ALOAD, BLOAD, CLOAD, LOADLV,
 * ALLDC/OC, PUTCOM, PUTNORM, PUTDPCOM, PUTCOM2, CHANLOAD.
 *
 * The AGC routine waits for each data word by leaving its return address,
 * negated, in REQRET and ending the job (REQDATX..Z then TC ENTEXIT); the
 * operator's ENTER does TC REQRET (ACCEPTWD). Here REQRET holds a
 * pin_reqret_t naming the point after the REQDAT call and
 * pin_load_resume() continues from it.
 */
#include <stddef.h>
#include "pinball_internal.h"

#define PIN_VBSP1LD 21                     /* VB21 = ALOAD                  */
#define PIN_VBSP2LD 22                     /* VB22 = BLOAD                  */
#define PIN_VBSP3LD 23                     /* VB23 = CLOAD                  */
#define PIN_SUPERBNK_CHAN 7u               /* don't load chan 7 (superbank) */

/* ---- REQDATX / REQDATY / REQDATZ (p. 328) ----------------------------- */

/* REQCOM: DSPCOUNT to the register's first digit, REQRET = -(return),
 * blank the register, flash on, and exit through ENTEXIT to wait.       */
agc_status_t pin_reqdat(int inrel, pin_reqret_t step)
{
    static const int16_t rnd1[3] = { PIN_R1D1, PIN_R2D1, PIN_R3D1 };
    pin_state_t *s = pin_st();
    if (!AGC_ASSERT(inrel >= PIN_INREL_X && inrel <= PIN_INREL_Z)
        || !AGC_ASSERT(step > PIN_REQ_PASS0 && step < PIN_REQ_COUNT)) {
        return AGC_STATUS_BAD_ARG;
    }
    pin_set_dspcount(rnd1[inrel - PIN_INREL_X]);
    s->reqret = step;
    s->reqret_neg = true;
    agc_status_t st = pin_5blank();
    pin_flashon();
    return (st == AGC_STATUS_OK) ? pin_entexit() : st;      /* ENDRQDAT    */
}

/* CLPASHI's "+3 to - number, backs data requests": the request for the
 * previous register of the same load verb.                               */
pin_reqret_t pin_reqret_backup(pin_reqret_t step)
{
    (void)AGC_ASSERT(pin_invariants());
    switch (step) {
    case PIN_REQ_ABC_Y: return PIN_REQ_ABC_X;
    case PIN_REQ_ABC_Z: return PIN_REQ_ABC_Y;
    case PIN_REQ_AB_Y:  return PIN_REQ_AB_X;
    case PIN_REQ_PASS0: case PIN_REQ_ABC_X: case PIN_REQ_AB_X: case PIN_REQ_A_X:
    case PIN_REQ_B_Y:   case PIN_REQ_C_Z:   case PIN_REQ_MCTBS: case PIN_REQ_MM:
    case PIN_REQ_COUNT:
    default:
        (void)AGC_ASSERT(false);               /* CLPASS keeps these out */
        return step;
    }
}

/* ---- LOADLV (p. 346) -------------------------------------------------- */

/* LOADLV: after a completed load, release the display for the priority
 * display problem, block numerical characters and clears, and go to
 * RECALTST to see if there is recall from ENDIDLE.                       */
agc_status_t pin_loadlv(void)
{
    (void)AGC_ASSERT(pin_invariants());
    pin_state_t *s = pin_st();
    s->decbrnch = 0;
    s->loadstat = PIN_LS_DATAIN;               /* CS ZERO TS LOADSTAT      */
    agc_status_t st = pin_reldsp();
    pin_block_dspcount(PIN_VD1);
    (void)AGC_ASSERT(s->dspcount_neg);
    return (st == AGC_STATUS_OK) ? pin_recaltst() : st;
}

agc_status_t agc_dsky_loadlv(void)
{
    return pin_loadlv();
}

/* ---- PUTCOM (pp. 347-348) --------------------------------------------- */

/* PUTCOM, octal: alarm and recycle if decimal only; for a DP scale (DPTEST)
 * +0 into the major part and the octal word into the minor part; CHANLOAD
 * for noun 10.                                                           */
static agc_status_t pin_putcom_octal(int k, const agc_scale_info_t *info, bool *finished)
{
    (void)AGC_ASSERT(pin_invariants());
    pin_state_t *s = pin_st();
    if (!pin_dectest()) {                                     /* DCTSTCYC  */
        *finished = true;
        return pin_almcycle();
    }
    agc_status_t st = AGC_STATUS_OK;
    uint16_t word = s->xyz_oct[k];
    if (s->nntype == PIN_NNTYPE_CHAN) {                       /* CHANLOAD  */
        *finished = true;
        if (s->nouncadr != PIN_SUPERBNK_CHAN) {
            st = pin_comp_write(k, (agc_dp_t)word);
        }
        return (st == AGC_STATUS_OK) ? pin_loadlv() : st;
    }
    agc_dp_t v;
    if (info->is_raw) {
        v = (agc_dp_t)word;
    } else if (info->is_dp && pin_sf_is_dp(info->routine)) {
        v = agc_dp_pack(0, pin_sp_from_ones(word));           /* PUTDPCOM  */
    } else if (info->is_dp) {
        agc_dp_t old = 0;                                     /* major part */
        st = pin_comp_read(k, &old);
        v = agc_dp_pack(pin_sp_from_ones(word), agc_dp_low(old));
    } else {
        v = agc_dp_from_sp(pin_sp_from_ones(word));
    }
    (void)AGC_ASSERT(word <= PIN_WORD);
    return (st == AGC_STATUS_OK) ? pin_comp_write(k, v) : st;
}

/* PUTCOM: store component k from XREG.. (octal) or XREG,XREGLP.. (decimal
 * via PUTDECSF). *finished when the load ended inside (alarm, HMSIN or
 * CHANLOAD going on to LOADLV).                                          */
static agc_status_t pin_putcom(int k, bool *finished)
{
    (void)AGC_ASSERT(pin_invariants());
    pin_state_t *s = pin_st();
    agc_scale_info_t info;
    agc_status_t st = pin_comp_scale(k, &info);
    if (!AGC_ASSERT(finished != NULL) || st != AGC_STATUS_OK) {
        return (st != AGC_STATUS_OK) ? st : AGC_STATUS_BAD_ARG;
    }
    *finished = false;
    if (s->decbrnch == 0u) {
        return pin_putcom_octal(k, &info, finished);
    }
    if (info.routine == (uint8_t)PIN_SF_HMS) {
        *finished = true;
        return pin_hmsin();                    /* stores all three, LOADLV */
    }
    agc_dp_t v = 0;
    st = pin_putdecsf(k, s->xyz_frac[k], s->xyz_negzero[k], &v, finished);
    if (*finished) {
        return st;
    }
    return pin_comp_write(k, v);
}

/* ALLDC/OC: tests that the data words loaded are either all decimal or
 * all octal; alarms (and recycles) if not.                               */
static agc_status_t pin_alldc_oc(int ncomp)
{
    (void)AGC_ASSERT(pin_invariants());
    const pin_state_t *s = pin_st();
    if (!AGC_ASSERT(ncomp == 2 || ncomp == 3)) {
        return AGC_STATUS_BAD_ARG;
    }
    uint16_t dec = (uint16_t)((s->decbrnch >> 2) & 7u);      /* dec comp bits */
    uint16_t all = (ncomp == 3) ? 7u : 6u;
    if (dec == 0u || dec == all) {
        return AGC_STATUS_OK;
    }
    return pin_almcycle();
}

/* PUTXYZ noun 07 part: XREG = ECADR of the flag word, YREG = bits to be
 * processed, ZREG zero to reset bits, non-zero to set bits.              */
static agc_status_t pin_noun7_bits(void)
{
    (void)AGC_ASSERT(pin_invariants());
    const pin_state_t *s = pin_st();
    uint16_t ecadr = s->xyz_oct[0];
    uint16_t bits = s->xyz_oct[1];
    uint16_t word = 0;
    (void)AGC_ASSERT(s->nounreg == 7u);
    agc_inhint();
    agc_status_t st = agc_erase_read_ecadr(ecadr, &word);
    if (st == AGC_STATUS_OK) {
        word = (s->xyz_oct[2] != 0u && s->xyz_oct[2] != PIN_NEGZERO)
                   ? (uint16_t)(word | bits)                  /* set bits   */
                   : (uint16_t)(word & ~bits & PIN_WORD);    /* BITSOFF    */
        st = agc_erase_write_ecadr(ecadr, word);
    }
    agc_relint();
    return st;
}

/* PUTXYZ (ncomp 3) / PUTXY (ncomp 2) / single-component store. */
static agc_status_t pin_putn(int first, int ncomp)
{
    if (!AGC_ASSERT(first >= 0 && first < 3)
        || !AGC_ASSERT(ncomp >= 1 && first + ncomp <= 3)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = (ncomp > 1) ? pin_alldc_oc(ncomp) : AGC_STATUS_OK;
    bool finished = false;
    for (int k = first; k < first + ncomp && st == AGC_STATUS_OK && !finished; k++) {
        st = pin_putcom(k, &finished);
    }
    if (st != AGC_STATUS_OK || finished) {
        return st;
    }
    /* If noun 7 has just been loaded, set flag bits as specified.        */
    if (ncomp == 3 && pin_st()->nounreg == 7u) {
        st = pin_noun7_bits();
    }
    return (st == AGC_STATUS_OK) ? pin_loadlv() : st;
}

/* ---- the load verbs (pp. 343-346) ------------------------------------- */

/* ALOAD (V21), BLOAD (V22), CLOAD (V23), ABLOAD (V24), ABCLOAD (V25):
 * component tests, then the first data request.                          */
agc_status_t pin_load_verb(uint8_t verb)
{
    (void)AGC_ASSERT(pin_invariants());
    pin_state_t *s = pin_st();
    if (!AGC_ASSERT(verb >= 21u && verb <= 25u)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (verb == 21u) {
        return pin_reqdat(PIN_INREL_X, PIN_REQ_A_X);          /* ALOAD     */
    }
    int comp = (verb == 22u || verb == 24u) ? 1 : 2;
    if (!pin_comptest(comp)) {
        return pin_dspalarm();
    }
    if (verb == 22u || verb == 23u) {
        s->clpass = PIN_CLPASS_NOBACK;         /* set CLPASS for pass0 only */
        return (verb == 22u) ? pin_reqdat(PIN_INREL_Y, PIN_REQ_B_Y)
                             : pin_reqdat(PIN_INREL_Z, PIN_REQ_C_Z);
    }
    if (!pin_nountest()) {                     /* test if noun can be loaded */
        return pin_dspalarm();
    }
    agc_status_t st = pin_updatvb(PIN_VBSP1LD);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return pin_reqdat(PIN_INREL_X, (verb == 24u) ? PIN_REQ_AB_X : PIN_REQ_ABC_X);
}

/* Show the next load verb and request the next register. */
static agc_status_t pin_next_request(uint8_t verb, int inrel, pin_reqret_t step)
{
    (void)AGC_ASSERT(pin_invariants());
    (void)AGC_ASSERT(verb == PIN_VBSP2LD || verb == PIN_VBSP3LD);
    agc_status_t st = pin_updatvb(verb);
    return (st == AGC_STATUS_OK) ? pin_reqdat(inrel, step) : st;
}

/* TC REQRET from ACCEPTWD: continue the load verb after its data word. */
agc_status_t pin_load_resume(pin_reqret_t step)
{
    (void)AGC_ASSERT(pin_invariants());
    if (!AGC_ASSERT(step > PIN_REQ_PASS0 && step < PIN_REQ_COUNT)) {
        return AGC_STATUS_BAD_ARG;
    }
    switch (step) {
    case PIN_REQ_A_X:   return pin_putn(0, 1);                       /* ALOAD  */
    case PIN_REQ_B_Y:   return pin_putn(1, 1);                       /* BLOAD  */
    case PIN_REQ_C_Z:   return pin_putn(2, 1);                       /* CLOAD  */
    case PIN_REQ_AB_X:  return pin_next_request(PIN_VBSP2LD, PIN_INREL_Y, PIN_REQ_AB_Y);
    case PIN_REQ_AB_Y:  return pin_putn(0, 2);                       /* PUTXY  */
    case PIN_REQ_ABC_X: return pin_next_request(PIN_VBSP2LD, PIN_INREL_Y, PIN_REQ_ABC_Y);
    case PIN_REQ_ABC_Y: return pin_next_request(PIN_VBSP3LD, PIN_INREL_Z, PIN_REQ_ABC_Z);
    case PIN_REQ_ABC_Z: return pin_putn(0, 3);                       /* PUTXYZ */
    case PIN_REQ_MCTBS: return pin_reqadd_resume();
    case PIN_REQ_MM:    return pin_mmchang_resume();
    case PIN_REQ_PASS0:
    case PIN_REQ_COUNT:
    default:
        return AGC_STATUS_BAD_ARG;
    }
}
