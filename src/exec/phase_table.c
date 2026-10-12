/**********************************************************************
 * FILE NAME: phase_table.c
 *
 * PURPOSE:
 *   PHASE TABLE MAINTENANCE: the restart-group phase words and their
 *   variable restart points (PHASCHNG, PHSCHNGA, 2PHSCHNG, NEWPHASE), the
 *   group kills (MR.KLEAN, P00KLEAN, V37KLEAN), and the major-mode register
 *   (NEWMODEX, NEWMODEA, MMDSPLAY, SETUPDSP, CHECKMM).
 *
 * AGC SOURCE:
 *   Comanche055 PHASE_TABLE_MAINTENANCE.agc pp. 1404-1413;
 *   RESTART_TABLES.agc p. 220 (NEWPHASE); FRESH_START_AND_RESTART.agc
 *   p. 185 (KLEANs)
 *   Luminary099 PHASE_TABLE_MAINTENANCE.agc pp. 1294-1302 (differs: adds
 *   PHSCHNGA); FRESH_START_AND_RESTART.agc p. 213 (KLEANs differ)
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   MODREG        agc_sp_t  I/O  B0 integer     agc_erase_modreg()
 *   TIME1, TIME2  uint32_t  I    cs             agc_time_now_cs() (TBASE)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_exec_novac        SETUPDSP: NOVAC PRIO30 DSPMMJOB (executive.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   BAILOUT 1202 through NOVAC (SETUPDSP); BAD_ARG on a failed
 *   precondition (bad group, code or mode).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Every update runs with interrupts inhibited. Both vehicles.
 *
 * NOTES:
 *   The two PHASE_TABLE_MAINTENANCE listings are the same program; Luminary
 *   adds the entry PHSCHNGA (code word in A, used by V37RET), which is
 *   agc_phase_change_var() with the code passed explicitly. The KLEANs
 *   differ: Comanche's P00KLEAN kills group 1 and V37KLEAN does not;
 *   Luminary's V37KLEAN kills group 1 (see agc_phase_change_program_klean).
 *
 *   Each of the six restart groups n has the erasable pair -PHASEn, PHASEn
 *   (they must stay complements; GOPROG3 checks this before restarting),
 *   TBASEn (TIME1 when the phase was set, for timed task restarts), and the
 *   variable restart point PHSNAMEn / PHSPRDTn. LONGBASE (TIME2,TIME1) is
 *   shared by every LONGCALL restart.
 *
 *   The code word after TC PHASCHNG (15 bits, octal):
 *     type A  TL0 00P PPP PPP GGG   fixed phase P of group G
 *     type B  TL1 DAP PPP PPP GGG   variable job + the fixed phase P
 *     type C  TL0 1AD XXX CJW GGG   variable job (J), task (W) or LONGCALL (C)
 *   T: set TBASEG, L: set LONGBASE, D: a new priority / delta time is given,
 *   A: a restart 2CADR is given. PHASEn receives bits 13-4 of the code word
 *   shifted down 3 (MP BIT12), so the type bits stay visible to RESTARTS:
 *     type A  phase 0..127, type B  01000 + D 0400 + A 0200 + phase,
 *     type C  0400 + A 0200 + D 0100 + XXX 070 + CJW.
 **********************************************************************/
#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "internal.h"

#define PH_WORD15 077777u            /* 15-bit word; also -0                     */
#define PH_NEG0   077777u            /* -0 (NEG0) in -PHASEn                     */
#define PH_TIME1  037777u            /* TIME1 is the low 14 bits of the clock    */
#define PH_GROUP  07u                /* GGG                                      */
#define PH_TBIT   040000u            /* T: set TBASE                             */
#define PH_LBIT   020000u            /* L: set LONGBASE                          */
#define PH_TYPEB  010000u            /* bit 13: type B                           */
#define PH_TYPEC  004000u            /* bit 12: type C (when bit 13 is clear)    */
#define PH_ABIT   002000u            /* A: 2CADR given (types B and C)           */
#define PH_DBIT_B 004000u            /* D in type B (bit 12)                     */
#define PH_DBIT_C 001000u            /* D in type C (bit 10)                     */
#define PH_PMASK  017770u            /* OCT17770, bits 13-4                      */
#define PH_PSHIFT 3                  /* MP BIT12 = shift right 3                 */
#define PH_PRIO30 ((agc_prio_t) 030) /* PRIO30: SETUPDSP */

typedef enum ph_type
{
    PH_TYPE_A = 0,
    PH_TYPE_B,
    PH_TYPE_C
} ph_type_t;

typedef struct ph_code
{
    int       group; /* 1..6 (GGG)                                  */
    uint16_t  tempp; /* TEMPP: what goes into PHASEn                */
    ph_type_t type;  /* A, B or C */
    bool      tbit;  /* T: set TBASEG (bit 15) */
    bool      lbit;  /* L: set LONGBASE (bit 14) */
    bool      dbit;  /* D: new priority / delta time given */
} ph_code_t;

/* Group n is index n; index 0 is unused. A fresh AGC's DOFSTART kills
 * every group, so the static image starts with all groups killed. */
static uint16_t          phase_w[AGC_RESTART_GROUPS + 1]; /* PHASEn      */
static uint16_t          nphase_w[AGC_RESTART_GROUPS + 1] = { /* -PHASEn     */
             PH_NEG0, PH_NEG0, PH_NEG0,
             PH_NEG0, PH_NEG0, PH_NEG0,
             PH_NEG0
         };
static uint16_t          tbase[AGC_RESTART_GROUPS + 1]; /* TIME1 at set, cs     */
static phase_var_store_t var[AGC_RESTART_GROUPS + 1];   /* PHSNAMEn/PRDTn */
static uint32_t          longbase;                      /* TIME2,TIME1 at set cs */

/**********************************************************************
 * Group number check for the PHASEn tables (PHASE_TABLE_MAINTENANCE.agc
 * C p.1410 / L p.1299: TEMPG = 2 x (GGG)).
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 *
 * RETURNS: bool -- true for a group 1..6
 **********************************************************************/
static bool group_ok(int group)
{
    return (group >= 1 && group <= AGC_RESTART_GROUPS);
}

/**********************************************************************
 * DXCH -PHASEn (PHASE_TABLE_MAINTENANCE.agc C p.1412 / L p.1301) with
 * (-phase, phase): both words of the pair at once.
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 *   phase -- the PHASEn word (type A: the phase number, octal)
 **********************************************************************/
void phase_set_fixed(int group, uint16_t phase)
{
    phase_w[group]  = phase;
    nphase_w[group] = (uint16_t) (~phase & PH_WORD15);
}

/**********************************************************************
 * EXTEND / DCA NEG0 / DXCH -PHASEn (FRESH_START_AND_RESTART.agc C p.185
 * / L p.213): -PHASEn = -0, PHASEn = +0, the group is inactive.
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 **********************************************************************/
void phase_kill(int group)
{
    nphase_w[group] = PH_NEG0;
    phase_w[group]  = 0;
}

/**********************************************************************
 * Kill restart group `group` with interrupts inhibited (MR.KLEAN form,
 * FRESH_START_AND_RESTART.agc C p.185 / L p.213).
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 **********************************************************************/
void agc_phase_kill(int group)
{
    agc_inhint();
    phase_kill(group);
    agc_relint();
}

/**********************************************************************
 * CS TIME1 / TS TBASEn (PHASE_TABLE_MAINTENANCE.agc C p.1411 / L p.1300).
 * DEVIATION: the AGC stores -TIME1 and RESTARTS adds it back; TIME1
 * itself is stored here (same elapsed time).
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 **********************************************************************/
static void set_tbase(int group)
{
    tbase[group] = (uint16_t) (agc_time_now_cs() & PH_TIME1);
}

/**********************************************************************
 * CS TIME1 / TS TBASEn by itself (LM SERVICER R10,R11 set-up: TBASE2;
 * PHASE_TABLE_MAINTENANCE.agc L p.1300 form).
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 **********************************************************************/
void agc_phase_set_tbase(int group)
{
    set_tbase(group);
}

/**********************************************************************
 * TS PHSPRDTn (V37 POOH: PRIO5 in group 2, C p.200 / L p.229; V37XEQ:
 * the new program's priority in group 4, C p.202 / L p.232).
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 *   prio -- priority number, 1..037 (PRIOnn >> 9)
 **********************************************************************/
void agc_phase_set_priority(int group, agc_prio_t prio)
{
    var[group].prdt      = (int16_t) prio;
    var[group].immediate = false;
}

/**********************************************************************
 * PHSCHNG2 decoding (C p.1410 / L p.1299): TEMPG, TEMPP, TEMPSW.
 *
 * ARGUMENTS:
 *   code -- PHASCHNG code word (octal, see the file prolog)
 *   out -- receives the decoded fields / the stored restart point
 *
 * The group of every code word is 1..6 (the codes are program constants).
 **********************************************************************/
static void decode(uint16_t code, ph_code_t *out)
{
    out->group = (int) (code & PH_GROUP);
    out->tempp = (uint16_t) ((code & PH_PMASK) >> PH_PSHIFT);
    out->tbit  = (code & PH_TBIT) != 0U;
    out->lbit  = (code & PH_LBIT) != 0U;

    if ((code & PH_TYPEB) != 0U)
    {
        out->type = PH_TYPE_B; /* CHECKB            */
        out->dbit = (code & PH_DBIT_B) != 0U;
    }
    else if ((code & PH_TYPEC) != 0U)
    {
        out->type = PH_TYPE_C;
        out->dbit = (code & PH_DBIT_C) != 0U;
    }
    else
    {
        out->type = PH_TYPE_A;
        out->dbit = false;
    }
}

/**********************************************************************
 * The restart kind a type C phase names by its CJW bits, as RESTARTS
 * (RESTARTS_ROUTINE.agc C p.1414 / L p.1303) decodes it: (CJW) - 2 > 0
 * LONGCALL, 0 job, < 0 task.
 *
 * ARGUMENTS:
 *   tempp -- TEMPP: bits 13-4 of the code word shifted down 3
 *
 * RETURNS: agc_restart_kind_t -- AGC_RESTART_LONGCALL, _JOB or _TASK; NONE on a failed
 *          postcondition
 **********************************************************************/
static agc_restart_kind_t cjw_kind(uint16_t tempp)
{
    int                cjw = (int) (tempp & PH_GROUP) - 2; /* (CJW) - 2: > 0 LONGCALL, 0 job, < 0 task */
    agc_restart_kind_t k   = AGC_RESTART_TASK;             /* W: waitlist task unless CJW says else */

    if (cjw > 0)
    {
        k = AGC_RESTART_LONGCALL;
    }
    else if (cjw == 0)
    {
        k = AGC_RESTART_JOB;
    }
    else
    {
        /* W: waitlist task */
    }

    return (k);
}

/**********************************************************************
 * GETPRIO / OLDPRIO / GETNEWNM / CON2 (C pp.1410-1412 / L pp.1299-1301):
 * store the variable restart point of a type B or C phase. With D clear
 * the old PHSPRDTn is kept; v == NULL keeps the old PHSNAMEn as well.
 * DEVIATION: the AGC takes the restart point from the return address
 * when A is clear; C cannot name a return address, so the caller always
 * names it in v (and an indirect delta time is resolved by the caller,
 * at the time of the phase change, into v->prio_or_dt / long_dt_cs).
 *
 * ARGUMENTS:
 *   c -- the decoded code word
 *   v -- the variable restart point (job, task or LONGCALL); NULL keeps
     the stored one
 **********************************************************************/
static void store_var(const ph_code_t *c, const agc_phase_var_t *v)
{
    phase_var_store_t *s = &var[c->group];  /* PHSNAMEn / PHSPRDTn of the group */
    agc_restart_kind_t k = AGC_RESTART_JOB; /* type B: always a job */

    if (c->type == PH_TYPE_C)
    {
        k = cjw_kind(c->tempp); /* type C: CJW names it */
    }

    if (v != 0)
    {
        s->kind = k;
        s->job  = v->job;
        s->task = v->task;

        if (c->dbit)
        {
            s->prdt       = v->prio_or_dt;
            s->immediate  = v->immediate;
            s->long_dt_cs = v->long_dt_cs;
        }
    }
}

/* BELOW1 for 2PHSCHNG: the type A phase of the second group.          */
typedef struct ph_second
{
    bool     present; /* 2PHSCHNG: a second group is set */
    int      group;   /* TEMPG2 / 2                                  */
    uint16_t phase;   /* TEMPP2                                      */
    bool     tbit;    /* TEMPSW2 = BIT15 of the first code word      */
} ph_second_t;

/**********************************************************************
 * PHASCHNG / PHSCHNGA / 2PHSCHNG common part (PHASE_TABLE_MAINTENANCE.agc
 * C pp.1409-1412 / L pp.1298-1301): decode, store the variable part of
 * a type B/C phase, then BELOW1 .. BELOW4, all with interrupts inhibited.
 *
 * ARGUMENTS:
 *   code -- PHASCHNG code word (octal, see the file prolog)
 *   v -- the variable restart point (job, task or LONGCALL); NULL keeps
     the stored one
 *   two -- the second group of a 2PHSCHNG (present = false for PHASCHNG)
 **********************************************************************/
static void phaschng_common(uint16_t code, const agc_phase_var_t *v, const ph_second_t *two)
{
    ph_code_t c; /* the decoded code word */

    decode(code, &c);
    agc_inhint();

    if (c.type != PH_TYPE_A)
    {
        store_var(&c, v);
    }

    /**********************************************************************
     * BELOW1 .. BELOW4 (C pp.1411-1412 / L pp.1300-1301).
     *
     * ARGUMENTS:
     *   c -- the decoded code word
     *   two -- the second group of a 2PHSCHNG (present = false for PHASCHNG)
     **********************************************************************/
    if (two->present && AGC_ASSERT(group_ok(two->group)))
    {
        phase_set_fixed(two->group, two->phase); /* DXCH -PHASE(G2)   */

        if (two->tbit)
        {
            set_tbase(two->group);
        }
    }

    if (c.tbit)
    {
        set_tbase(c.group); /* BELOW2 -> TBASE   */
    }

    if (c.lbit)
    {
        longbase = agc_time_now_cs(); /* BELOW3: DCA TIME2 */
    }

    phase_set_fixed(c.group, c.tempp); /* BELOW4: -PHASE    */
    agc_relint();
}

/**********************************************************************
 * PHASCHNG (C p.1409 / L p.1298), fixed form: the group is the GGG of
 * the code word. A type B/C code given here keeps the stored restart
 * point and priority (as if D = 0 and the old 2CADR were still right).
 *
 * ARGUMENTS:
 *   phase_code -- PHASCHNG code word (octal); GGG = restart group 1..6
 **********************************************************************/
void agc_phase_change(uint16_t phase_code)
{
    ph_second_t none = {false, 0, 0, false};

    phaschng_common(phase_code, 0, &none);
}

/**********************************************************************
 * PHASCHNG / PHSCHNGA (PHASE_TABLE_MAINTENANCE.agc C p.1409 / L p.1298)
 * type B or C with the variable restart point.
 *
 * ARGUMENTS:
 *   code -- PHASCHNG code word (octal, see the file prolog)
 *   v -- the variable restart point (job, task or LONGCALL); NULL keeps
     the stored one
 **********************************************************************/
void agc_phase_change_var(uint16_t code, const agc_phase_var_t *v)
{
    ph_second_t none = {false, 0, 0, false};

    phaschng_common(code, v, &none);
}

/**********************************************************************
 * 2PHSCHNG (C p.1409 / L p.1298): code_a (type A only; its L bit is
 * disregarded) for one group, code_b (A, B or C) for another. v may be
 * NULL when code_b is type A.
 *
 * ARGUMENTS:
 *   code_a -- type A code word of the second group (the first inline word)
 *   code_b -- code word of the main group (A, B or C)
 *   v -- the variable restart point (job, task or LONGCALL); NULL keeps
     the stored one
 **********************************************************************/
void agc_phase_change2(uint16_t code_a, uint16_t code_b, const agc_phase_var_t *v)
{
    ph_code_t a; /* the second group's code word */

    decode(code_a, &a);
    ph_second_t two = {true, a.group, a.tempp, a.tbit};

    phaschng_common(code_b, v, &two);
}

/**********************************************************************
 * NEWPHASE (RESTART_TABLES C p.220; Comanche only): quick fixed phase change
 * of `group` to `phase`, optionally setting TBASE (the AGC passes the
 * phase negative in A to ask for that).
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 *   phase -- the PHASEn word (type A: the phase number, octal)
 *   set_tb -- true: also CS TIME1 / TS TBASEn (A negative in the AGC)
 **********************************************************************/
void agc_phase_newphase(int group, uint16_t phase, bool set_tb)
{
    agc_inhint();

    if (set_tb)
    {
        set_tbase(group); /* NUFAZ+10          */
    }

    phase_set_fixed(group, phase);
    agc_relint();
}

/**********************************************************************
 * Contract helper for PHASCHNG (PHASE_TABLE_MAINTENANCE.agc C p.1409 /
 * L p.1298): make `group` restart `job` as a FINDVAC job of priority
 * `prio`, i.e. TC PHASCHNG / OCT 07020+G / OCT prio / 2CADR job (type C,
 * A, D, J).
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 *   job -- job id the group restarts (AGC_JOB_*)
 *   prio -- priority number, 1..037 (PRIOnn >> 9)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_phase_new(int group, agc_job_id_t job, agc_prio_t prio)
{
    if (!AGC_ASSERT(group_ok(group)) || !AGC_ASSERT(job != AGC_JOB_NONE) || !AGC_ASSERT(prio >= 1 && prio <= 037))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_var_t v = {AGC_RESTART_JOB, job, AGC_TASK_NONE, (int16_t) prio, false, 0U};

    agc_phase_change_var((uint16_t) (07020U | (uint16_t) group), &v);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * C(PHASEn) (PHASE_TABLE_MAINTENANCE.agc C p.1412 / L p.1301).
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 *
 * RETURNS: uint16_t -- the PHASEn word (0 = inactive); 0 on a bad group
 **********************************************************************/
uint16_t agc_phase_get(int group)
{
    return (phase_w[group]);
}

/**********************************************************************
 * PCLOOP test (FRESH_START C p.188 / L p.218): -PHASEn RXOR PHASEn must
 * be -0, i.e. the two words are complements.
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 *
 * RETURNS: bool -- true when -PHASEn and PHASEn are complements
 **********************************************************************/
bool phase_group_agrees(int group)
{
    return ((uint16_t) (nphase_w[group] ^ phase_w[group]) == PH_NEG0);
}

/**********************************************************************
 * PHSNAMEn / PHSPRDTn (RESTARTS_ROUTINE.agc C p.1414 / L p.1303 reads
 * them): the stored variable restart point of `group`.
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 *   out -- receives the decoded fields / the stored restart point
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; BAD_ARG on a bad group or NULL out
 **********************************************************************/
agc_status_t phase_var_get(int group, phase_var_store_t *out)
{
    if (!AGC_ASSERT(group_ok(group)) || !AGC_ASSERT(out != 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *out = var[group];

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * TBASEn (FINDTIME, RESTARTS_ROUTINE.agc C p.1415 / L p.1304).
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 *
 * RETURNS: uint16_t -- TIME1 when TBASEn was set, cs (14 bits); 0 on a bad group
 **********************************************************************/
uint16_t phase_tbase(int group)
{
    return (tbase[group]);
}

/**********************************************************************
 * LONGBASE (ITSLGCL2, RESTARTS_ROUTINE.agc C p.1417 / L p.1306).
 *
 * ARGUMENTS: none
 *
 * RETURNS: uint32_t -- TIME2,TIME1 when LONGBASE was set, cs; 0 on a failed check
 **********************************************************************/
uint32_t phase_longbase(void)
{
    return (longbase);
}

/**********************************************************************
 * AGC: none (test hook) -- Test hook: corrupt -PHASEn (as a restart between the two stores of a
 * PHASCHNG would leave it) so the GOPROG3 table check can be exercised.
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 *   neg_word -- the word to put in -PHASEn
 **********************************************************************/
void phase_debug_set_negative(int group, uint16_t neg_word)
{
    nphase_w[group] = neg_word;
}

/**********************************************************************
 * V37KLEAN (FRESH_START C p.185 / L p.213): kill groups 3, 5, 6, and in
 * Luminary also group 1.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; BAD_ARG on a failed postcondition
 **********************************************************************/
agc_status_t agc_phase_change_program_klean(void)
{
    agc_inhint();

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        phase_kill(1);
    }

    phase_kill(3);
    phase_kill(5);
    phase_kill(6);
    agc_relint();

    if (!AGC_ASSERT(phase_w[3] == 0U && phase_w[5] == 0U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * P00KLEAN (FRESH_START_AND_RESTART.agc C p.185 / L p.213): kill group 4
 * (Comanche also group 1), then V37KLEAN.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the V37KLEAN status; BAD_ARG on a failed postcondition
 **********************************************************************/
agc_status_t agc_phase_idle_klean(void)
{
    agc_inhint();
    phase_kill(4);

    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        phase_kill(1);
    }

    agc_relint();

    if (!AGC_ASSERT(phase_w[4] == 0U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_phase_change_program_klean());
}

/**********************************************************************
 * MR.KLEAN (FRESH_START_AND_RESTART.agc C p.185 / L p.213): kill group 2,
 * then P00KLEAN -- every group is inactive.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the P00KLEAN status; BAD_ARG on a failed postcondition
 **********************************************************************/
agc_status_t agc_phase_mr_klean(void)
{
    agc_inhint();
    phase_kill(2);
    agc_relint();
    agc_status_t st = agc_phase_idle_klean(); /* P00KLEAN outcome */

    if (!AGC_ASSERT(phase_w[1] == 0U && phase_w[6] == 0U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * MMDSPLAY -> SETUPDSP (C p.1404 / L p.1294): NOVAC PRIO30 DSPMMJOB to
 * show MODREG in the PROG lights.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAILOUT (1202) from NOVAC; BAD_ARG
 *          otherwise
 **********************************************************************/
agc_status_t fresh_mmdsplay(void)
{
    agc_inhint();
    agc_status_t st = agc_exec_novac(PH_PRIO30, AGC_JOB_DSPMMJB); /* NOVAC DSPMMJOB outcome */

    agc_relint();

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BAILOUT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * NEWMODEX / NEWMODEA (C p.1404 / L p.1294): store the major mode in
 * MODREG and display it.
 *
 * ARGUMENTS:
 *   mode -- major mode number (0..0177; EXEC_MODREG_NEG0 for -0)
 *
 * RETURNS: agc_status_t -- the MMDSPLAY status; BAD_ARG on a bad mode
 **********************************************************************/
agc_status_t agc_phase_newmode(agc_sp_t mode)
{
    if (!AGC_ASSERT(mode >= EXEC_MODREG_NEG0 && mode <= 0177))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_erase_set_modreg(mode); /* TS MODREG         */

    return (fresh_mmdsplay());
}

/**********************************************************************
 * CHECKMM (C p.1404 / L p.1294): does MODREG equal `mode`?
 *
 * ARGUMENTS:
 *   mode -- major mode number (0..0177; EXEC_MODREG_NEG0 for -0)
 *
 * RETURNS: bool -- true when MODREG = mode
 **********************************************************************/
bool agc_phase_checkmm(agc_sp_t mode)
{
    agc_sp_t mm = agc_erase_modreg(); /* C(MODREG), B0 integer */

    return (mm == mode);
}
