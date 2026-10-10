/*
 * t4rupt_csm.c -- Comanche055 T4RUPT_PROGRAM.agc, optics part:
 * OPTICS MONITORING AND ZERO ROUTINES (pp. 160-165) and the OPTICS CDU
 * DRIVING PROGRAM (pp. 166-169).
 *
 * OPTMON runs every 480 ms (T4JUMP phases 1, 5): it watches the optics CDU
 * fail bit (channel 30 bit 7) and the optics mode switch (channel 33 bits
 * 4, 5: ZERO OPTICS / MANUAL / CMC) and sequences optics zeroing (a 32
 * sample wait, then 200 ms of ZERO OCDU, then 200 ms for the CDUs to
 * resynchronise).  OPTDRIVE runs at phases 0, 4: it drives the trunnion
 * and shaft CDUs towards DESOPTT / DESOPTS with at most 165 pulses per
 * pass, keeping the shaft away from its stops.
 *
 * Erasables owned here (module-private, AGC names, all B(1) bit words or
 * SP counts): OPTMODES, OPTIND, SWSAMPLE, DESOPMOD, WTOPTION, ZOPTCNT,
 * ZONE, COMMANDO +0/+1, and the optics drive out-counters CDUTCMD /
 * CDUSCMD (053/054).  DESOPTT / DESOPTS live in prog-align's agc_align_ram.
 *
 * OPTIND: -1 no coarse optics, 0 / 1 coarse optics working (also the
 * trunnion/shaft index in OPTDRIVE), -0 optics inhibited (TVC takeover,
 * P40 S40.6).  DEVIATION: -0 is represented as AGC_OPTIND_NEG0 (-2).
 * SWSAMPLE / DESOPMOD: +1 CMC (CSC), 0 manual, -1 zero optics.
 *
 * Note (p.161): the AGC labels "ZOPFIN1" are typos for ZOPFINI in the
 * listing; the header comment there gives the L+1/L+2 returns reversed:
 * the code returns L+2 when zero optics is processing.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/io/service_internal.h"
#include "agc/programs/alignment.h"
#include "sysmisc_ext.h"

#define BIT1                 000001u
#define BIT2                 000002u
#define BIT5                 000020u
#define BIT7                 000100u
#define BIT8                 000200u
#define BIT9                 000400u
#define BIT10                001000u
#define BIT15                040000u
#define WORD15               077777u
#define OCTHIRTY             000030u /* BITS4&5: optics mode switch bits          */
#define OPT_SIX              000006u /* zero optics processing + OCDUFAIL inhibit */
#define OPT_SEVEN            000007u
#define OCT11_12             006000u /* 11,12 = PRIO6: drive CDU S and T          */
#define OPTINITF             000130u /* fresh start OPTMODES                      */
#define OPTINITR             000430u /* restart: OPTMODES bits kept               */
#define ZOPT_SAMPLES         32  /* OCT40: 32-sample wait                     */
#define WTOPT_RETURN         11  /* OCT13: 5 s return option (480 ms passes)  */
#define ZOPT_HOLD_CS         20u /* 200MS = OCT24                             */
#define DEG20_CDU            7199 /* 20DEGS DEC 7199                           */
#define MAXPLS               165 /* MAXPLS DEC -165 (was -80)                 */
#define DEG45_CDU            4096 /* 13,14,15: -45 deg in the CDU scale        */
#define DEG90_CDU            8192 /* NEG1/2                                    */
#define CDU_SIGN             040000u
#define LAMPWORD             11
#define ALARM_ZOPT_SWITCH    ((agc_alarm_code_t) 00116) /* switch moved in ZOPT */
#define ALARM_OPT_NOT_ZEROED ((agc_alarm_code_t) 00120)

static uint16_t optmodes = OPTINITF; /* OPTMODES                           */
static agc_sp_t optind   = -1;       /* OPTIND (AGC_OPTIND_NEG0 = -0)      */
static int      swsample;            /* SWSAMPLE current switch +1/0/-1    */
static int      desopmod;            /* DESOPMOD previous switch setting   */
static int      wtoption;            /* WTOPTION return option countdown   */
static int      zoptcnt;             /* ZOPTCNT 32-sample counter          */
static int      zone;                /* ZONE shaft monitor, CDUS (ones')   */
static int      commando[2];         /* COMMANDO trunnion, shaft (pulses)  */
static agc_sp_t optcmd[2];           /* CDUTCMD, CDUSCMD out-counters      */

/* ---- accessors ---------------------------------------------------------- */
uint16_t agc_erase_optmodes(void)
{
    if (!AGC_ASSERT(optmodes <= WORD15))
    {
        return 0; /* no valid result */
    }
    return optmodes;
}

void agc_erase_set_optmodes(uint16_t v)
{
    if (AGC_ASSERT(v <= WORD15))
    {
        optmodes = v;
    }
}

agc_sp_t agc_erase_optind(void)
{
    if (!AGC_ASSERT(optind >= AGC_OPTIND_NEG0 && optind <= 1))
    {
        return 0; /* no valid result */
    }
    return optind;
}

void agc_erase_set_optind(agc_sp_t v)
{
    if (AGC_ASSERT(v >= AGC_OPTIND_NEG0 && v <= 1))
    {
        optind = v;
    }
}

/* SWSAMPLE: optics mode switch as last sampled by OPTMON (V41 N91 reads
 * it): +1 computer (CSC), 0 manual, -1 zero optics.                       */
agc_sp_t agc_erase_swsample(void)
{
    if (!AGC_ASSERT(swsample >= -1 && swsample <= 1))
    {
        return 0; /* no valid result */
    }
    return (agc_sp_t) swsample;
}

/* Optics drive out-counters CDUTCMD (0) / CDUSCMD (1) for the sim, which
 * drains them; 0 stands for the AGC's -0 (no pulses).                    */
agc_sp_t agc_t4_optics_cmd(int which)
{
    if (!AGC_ASSERT(which == 0 || which == 1))
    {
        return 0;
    }
    return optcmd[which];
}

/* Fresh start (DOFSTART C p.184: OPTIND = -1, OPTMODES = OPTINITF) or
 * hardware restart (GOPROG C p.188: OPTMODES = (OPTMODES & OPTINITR) +
 * BIT7).                                                                  */
agc_status_t agc_t4_optics_reset(bool fresh)
{
    if (fresh)
    {
        optind   = -1;
        optmodes = OPTINITF;
        wtoption = 0;
    }
    else
    {
        optmodes = (uint16_t) ((optmodes & OPTINITR) + BIT7);
    }
    if (!AGC_ASSERT(optmodes <= WORD15))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK;
}

/* ---- helpers ----------------------------------------------------------- */

static void ch_or(int chan, uint16_t m)
{
    agc_chan_or(chan, m);
    if (!AGC_ASSERT(m <= WORD15))
    {
        return;
    }
}

static void ch_clear(int chan, uint16_t bits)
{
    agc_chan_and(chan, (uint16_t) (~(uint32_t) bits & WORD15));
    if (!AGC_ASSERT(bits <= WORD15))
    {
        return;
    }
}

/* A CDU counter word read as the AGC's ones' complement SP value.         */
static int cdu_ones(uint16_t cdu)
{
    if (!AGC_ASSERT(cdu <= WORD15))
    {
        return 0; /* no valid result */
    }
    int v = ((cdu & CDU_SIGN) != 0u) ? -(int) (WORD15 - cdu) : (int) cdu;
    if (!AGC_ASSERT(v >= -037777 && v <= 037777))
    {
        return 0; /* no valid result */
    }
    return v;
}

static int iabs(int v)
{
    if (!AGC_ASSERT(v > -0100000 && v < 0100000))
    {
        return 0; /* no valid result */
    }
    return (v < 0) ? -v : v;
}

/* ZOPFINI (p.161): 1 = ENDZOPT task working (the AGC RESUMEs at once),
 * 2 = zero optics processing (L+2), 0 = not processing (L+1).            */
static int t4_zopfini(void)
{
    if (!AGC_ASSERT(optmodes <= WORD15))
    {
        return 0; /* no valid result */
    }
    if ((optmodes & BIT1) != 0u)
    {
        return 1;
    }
    return ((optmodes & 000004u) != 0u) ? 2 : 0;
}

/* CANZOPT (p.161).                                                        */
static void t4_canzopt(void)
{
    optmodes = (uint16_t) (optmodes & ~(uint32_t) OPT_SIX);
    ch_clear((int) AGC_CHAN_CHAN12, (uint16_t) BIT1);
    if (!AGC_ASSERT((optmodes & OPT_SIX) == 0u))
    {
        return;
    }
}

/* INITZOPT (p.164): zero optics processing, OCDUFAIL inhibited.          */
static void t4_initzopt(void)
{
    wtoption = 0;
    optmodes = (uint16_t) (optmodes | OPT_SIX);
    if (!AGC_ASSERT((optmodes & OPT_SIX) == OPT_SIX))
    {
        return;
    }
}

/* SETDESMD (p.163).                                                       */
static agc_status_t t4_setdesmd(void)
{
    desopmod = swsample;
    if (!AGC_ASSERT(desopmod >= -1 && desopmod <= 1))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK;
}

/* MANTOCSC (p.162) from +3 (COARSLOK) when full is false.                 */
static agc_status_t t4_mantocsc(bool full)
{
    if (full)
    {
        wtoption = 0;
        zoptcnt  = 0;
    }
    if ((optmodes & BIT9) != 0u)
    {               /* COARSLOK: coarse since fstart */
        optind = 1; /* set coarse optics working     */
        ch_or((int) AGC_CHAN_CHAN12, (uint16_t) BIT2);
    }
    return t4_setdesmd();
}

/* CANCOARS (p.163).                                                       */
static agc_status_t t4_cancoars(void)
{
    optind = -1;
    ch_clear((int) AGC_CHAN_CHAN12, (uint16_t) BIT2);
    optmodes = (uint16_t) (optmodes | BIT9); /* return-to-coarse bit      */
    return t4_setdesmd();
}

/* SETZOEND (p.163): send ZERO OCDU for 200 ms (task ENDZOPT).            */
static agc_status_t t4_setzoend(void)
{
    ch_or((int) AGC_CHAN_CHAN12, (uint16_t) BIT1);
    agc_status_t st = agc_waitlist_call(ZOPT_HOLD_CS, AGC_TASK_ENDZOPT);
    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return AGC_STATUS_BAD_ARG;
    }
    optmodes = (uint16_t) (optmodes | BIT1); /* ZOPT task working         */
    return t4_setdesmd();
}

/* ZOPTDES / ZTOMAN / ZTOCSC (p.161): previous setting zero optics.       */
static agc_status_t t4_zoptdes(void)
{
    int z = t4_zopfini();
    if (z == 1)
    {
        return AGC_STATUS_OK; /* TC RESUME                 */
    }
    if (swsample < 0)
    { /* still zero optics         */
        if (z == 0)
        {
            return t4_setdesmd();
        }
        if (zoptcnt > 0)
        {
            zoptcnt--; /* SETCNT                    */
            return t4_setdesmd();
        }
        return t4_setzoend();
    }
    if (swsample == 0)
    { /* ZTOMAN                    */
        if (z == 0)
        {
            return t4_setdesmd();
        }
        agc_alarm(ALARM_ZOPT_SWITCH); /* ZOPALARM  */
        wtoption = WTOPT_RETURN;
        t4_canzopt();
        return t4_setdesmd();
    }
    if (z == 0)
    { /* ZTOCSC                    */
        return t4_mantocsc(false);
    }
    agc_alarm(ALARM_ZOPT_SWITCH);
    t4_canzopt();
    return t4_mantocsc(true);
}

/* MANUDES / MANTOMAN (p.162): previous setting manual.                   */
static agc_status_t t4_manudes(void)
{
    if (!AGC_ASSERT(wtoption >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (swsample > 0)
    {
        return t4_mantocsc(true);
    }
    if (swsample == 0)
    { /* MANTOMAN                  */
        if (wtoption > 0)
        {
            wtoption--;
        }
        return t4_setdesmd();
    }
    if (wtoption > 0)
    { /* 5 s return: continue ZOPT */
        t4_initzopt();
        return t4_setdesmd();
    }
    t4_initzopt(); /* OPTZERO: start anew       */
    zoptcnt = ZOPT_SAMPLES;
    return t4_setdesmd();
}

/* CSCDES / CSCTOZOP / CSCTOMAN (p.162): previous setting CMC.            */
static agc_status_t t4_cscdes(void)
{
    if (swsample > 0)
    {
        return t4_setdesmd();
    }
    if (swsample < 0)
    { /* CSCTOZOP                  */
        zoptcnt = ZOPT_SAMPLES;
        t4_initzopt();
    }
    if (optind >= 0)
    { /* coarse optics working     */
        return t4_cancoars();
    }
    return t4_setdesmd();
}

/* OCDUFTST / OPFAILON / OPFAILOF (p.165): the OPTICS CDU FAIL lamp,
 * DSPTAB +11D bit 8, follows channel 30 bit 7 (0 = fail).                */
static void t4_ocduftst(void)
{
    uint16_t w       = agc_erase_dsptab(LAMPWORD);
    bool     lamp_on = (w & BIT8) != 0u;
    bool     want_on;
    if (!AGC_ASSERT(w <= WORD15))
    {
        return;
    }
    if ((agc_chan_read((int) AGC_CHAN_CHAN30) & BIT7) != 0u)
    {
        if ((agc_erase_imodes33() & BIT1) != 0u)
        {
            return; /* lamp test in progress     */
        }
        want_on = false;
    }
    else
    {
        if ((optmodes & BIT2) != 0u)
        {
            return; /* OCDUFAIL inhibited        */
        }
        want_on = true;
    }
    if (want_on != lamp_on)
    { /* SETOFF                    */
        agc_erase_set_dsptab(
            LAMPWORD, (uint16_t) (((w ^ BIT8) & 037777u) + BIT15));
    }
}

/* OPTMON / 33OPTMON / SETSAMP / PROCESSW (p.160).                        */
agc_status_t agc_t4_optics_monitor(void)
{
    uint16_t chg =
        (uint16_t) ((optmodes ^ agc_chan_read((int) AGC_CHAN_CHAN30)) & BIT7);
    if (chg != 0u)
    {
        t4_ocduftst();
    }
    if (optind == AGC_OPTIND_NEG0)
    {
        return AGC_STATUS_OK; /* bypass if TVC takeover    */
    }
    chg = (uint16_t) (chg + ((optmodes ^ agc_chan_read((int) AGC_CHAN_CHAN33)) &
                             OCTHIRTY));
    optmodes         = (uint16_t) (optmodes ^ chg);
    uint16_t present = (uint16_t) (~(uint32_t) optmodes & OCTHIRTY);
    if (present == 0u)
    {
        swsample = 0; /* manual                    */
    }
    else
    {
        swsample = ((present & BIT5) != 0u) ? 1 : -1; /* CSC / ZOPTICS    */
    }
    if (desopmod > 0)
    {
        return t4_cscdes();
    }
    return (desopmod == 0) ? t4_manudes() : t4_zoptdes();
}

/* ENDZOPT / ZEROPCDU (p.163-164) and its continuation after VARDELAY.    */
agc_status_t agc_t4_optics_task(agc_task_id_t task)
{
    if (task == AGC_TASK_ENDZOPT)
    {
        agc_erase_set_cdus(0); /* ZEROPCDU                  */
        zone = 0;
        agc_erase_set_cdut((uint16_t) (WORD15 - DEG20_CDU)); /* -20 deg   */
        ch_clear((int) AGC_CHAN_CHAN12, (uint16_t) BIT1);
        return agc_waitlist_vardelay(ZOPT_HOLD_CS, AGC_TASK_ENDZOPT2);
    }
    if (!AGC_ASSERT(task == AGC_TASK_ENDZOPT2))
    {
        return AGC_STATUS_BAD_ARG;
    }
    optmodes = (uint16_t) ((optmodes | BIT10) & ~(uint32_t) OPT_SEVEN);
    t4_ocduftst(); /* OCDUFAIL after enable     */
    return AGC_STATUS_OK;
}

/* GETOPCMD (p.167): command = round(diff / 4) of the desired minus the
 * actual CDU (MSU, MP BIT13, DOUBLE of the low word for the rounding).  */
static int t4_getopcmd(uint16_t desired, uint16_t actual)
{
    int diff = (int) desired - (int) actual; /* MSU (mod 2^15)  */
    if (diff >= 040000)
    {
        diff -= 0100000;
    }
    else if (diff < -040000)
    {
        diff += 0100000;
    }
    int q = diff / 4;
    int r = diff % 4;
    if (r >= 2)
    {
        q++;
    }
    else if (r <= -2)
    {
        q--;
    }
    if (!AGC_ASSERT(q >= -010000 && q <= 010000))
    {
        return 0; /* no valid result */
    }
    return q;
}

/* OPTDRIVE zone update (p.166): ZONE = CDUS when the shaft first passes
 * 45 deg, 0 inside 45 deg.                                               */
static void t4_shaft_zone(void)
{
    int cdus = cdu_ones(agc_erase_cdus());
    if (iabs(cdus) <= DEG45_CDU)
    {
        zone = 0;
    }
    else if (zone == 0)
    {
        zone = cdus;
    }
    if (!AGC_ASSERT(iabs(zone) <= 037777))
    {
        return;
    }
}

/* OPT2 / TRUNCMD / shaft stop avoidance (pp.167-168): fill COMMANDO.     */
static void t4_opt_commands(void)
{
    const agc_align_ram_t *ar = agc_align_ram();
    if (!AGC_ASSERT(ar != NULL))
    {
        return;
    }
    commando[1] = t4_getopcmd(ar->desopts, agc_erase_cdus()); /* shaft */
    int tdiff   = cdu_ones(ar->desoptt) - cdu_ones(agc_erase_cdut());
    if (tdiff > 037777 || tdiff < -037777)
    { /* TS Q overflow: +-POSMAX   */
        commando[0] = (tdiff > 0) ? 037777 : -037777;
    }
    else
    {
        commando[0] = t4_getopcmd(ar->desoptt, agc_erase_cdut());
    }
    int cdus = cdu_ones(agc_erase_cdus());
    if (iabs(cdus) - 1 - DEG90_CDU <= 0 || zone == 0)
    {
        return; /* CMDSETUP                 */
    }
    if ((zone < 0) != (commando[1] < 0))
    {
        return; /* sign zone != sign command */
    }
    if (iabs(cdu_ones(ar->desopts)) - 1 - DEG90_CDU <= 0)
    {
        commando[1] = -commando[1]; /* reverse regular command  */
    }
}

/* CMDSETUP / POSOPCMD / NEGOPCMD / NEXTOPT / SENDOCMD (pp.168-169).       */
static void t4_opt_send(void)
{
    int sends = 0; /* ITEMP1 send indicator    */
    for (int i = 1; i >= 0; i--)
    {
        int c = commando[i];
        if (c > MAXPLS)
        {
            c = MAXPLS;
        }
        else if (c < -MAXPLS)
        {
            c = -MAXPLS;
        }
        if (c != 0)
        {
            sends++;
        }
        optcmd[i] = (agc_sp_t) c; /* CDUTCMD / CDUSCMD        */
    }
    optind = 0;
    if (sends > 0)
    {
        ch_or((int) AGC_CHAN_CHAN14, (uint16_t) OCT11_12);
    }
    if (!AGC_ASSERT(sends <= 2))
    {
        return;
    }
}

/* OPTDRIVE / CONTDRVE (pp.166-167).                                       */
agc_status_t agc_t4_optics_drive(void)
{
    t4_shaft_zone();
    if (optind < 0)
    {
        return AGC_STATUS_OK; /* no coarse optics         */
    }
    if (swsample <= 0)
    {
        return AGC_STATUS_OK; /* switch not at CMC        */
    }
    if ((optmodes & BIT10) == 0u)
    {
        agc_alarm(ALARM_OPT_NOT_ZEROED);
    }
    if ((agc_chan_read((int) AGC_CHAN_CHAN12) & BIT2) == 0u)
    {
        ch_or((int) AGC_CHAN_CHAN12, (uint16_t) BIT2); /* SETBIT           */
        return AGC_STATUS_OK;
    }
    t4_opt_commands();
    t4_opt_send();
    return AGC_STATUS_OK;
}
