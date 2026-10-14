# 6. The crew interface

The DSKY (chapter 1) is a keyboard and a few rows of digits, and the
code behind it -- called PINBALL_GAME_BUTTONS_AND_LIGHTS in the listing,
"pinball" for short -- is one of the largest pieces of the program. This
chapter follows a key press from the hardware to the program, describes
the verb and noun tables, the display loop and its lamps, and then the
other direction: how a program asks the crew something and waits for
the answer.

## 6.1 From a key to a job

Pressing a key puts its 5-bit code on input channel 15 (main DSKY) or
16 (the CM's navigation DSKY and the mark buttons) and raises KEYRUPT1
or KEYRUPT2. The key codes are in `include/agc/io/dsky.h` (`enum
agc_dsky_key`: 1-9, 0 = 020, VERB = 021, RSET = 022, KEY REL = 031, +, -,
ENTR = 034, CLR = 036, NOUN = 037). The interrupt does almost nothing:
it reads the code and starts a job to deal with it. The listing
(KEYRUPT_UPRUPT.agc, C p.1449) and the C
(`io/dsky/pinball/keys.c:agc_dsky_keyrupt1`, `pin_keycom`):

```
KEYRUPT1   TS      BANKRUPT
           XCH     Q
           TS      QRUPT
           TC      LODSAMPT     # TIME IS SNATCHED IN RUPT FOR NOUN 65.
           CAF     LOW5
           EXTEND
           RAND    MNKEYIN      # CHECK IF KEYS 5M-1M ON
KEYCOM     TS      RUPTREG4
           CS      FLAGWRD5
           MASK    BIT15
           ADS     FLAGWRD5
ACCEPTUP   CAF     CHRPRIO      # (NOTE: RUPTREG4 = KEYTEMP1)
           TC      NOVAC
           2CADR   CHARIN
           CA      RUPTREG4
           INDEX   LOCCTR
```

```c
    pin_lodsampt(); /* time snatched for N65  */
    ...
    code = (uint16_t) (agc_chan_read((int) AGC_CHAN_MNKEYIN) & PIN_LOW5);
    ...
    return (pin_keycom(code));
}

static agc_status_t pin_keycom(uint16_t code)
{
    ...
    agc_flag_set((agc_flag_id_t) AGC_FLAG_DSKYFLAG);
    ...
    st = agc_exec_novac(PIN_CHRPRIO, AGC_JOB_CHARIN);
    if (st == AGC_STATUS_OK)
    {
        st = agc_exec_set_new_job_arg((agc_sp_t) code); /* INDEX LOCCTR TS MPAC */
    }
```

The `INDEX LOCCTR / TS MPAC` that hands the new job its key code
through the job's own MPAC becomes `agc_exec_set_new_job_arg`; the CHARIN
job reads it from `ctx->arg`. The "CS FLAGWRD5 / MASK BIT15 / ADS
FLAGWRD5" sequence -- set bit 15 of FLAGWRD5 if it is clear -- is
DSKYFLAG.

CHARIN (`io/dsky/pinball/keys.c:pin_charin`, CM p.315) is where the keyboard grammar
lives. It first marks the display system busy, and its comment explains
a piece of crew interface design:

```c
 * CHARIN: block display system, but save old C(DSPLOCK) for error light
 * reset. All keys except ER turn on KR lite if CADRSTOR is full; this
 * reminds the operator to re-establish a flashing display which he has
 * obscured with displays of his own.
```

Then it fans the key out: digits go into the register being keyed, VERB
and NOUN start a two-digit entry, `+`/`-` start a signed decimal, CLR
clears, ENTR executes, KEY REL releases the display, RSET (error reset)
turns off the warning lamps. A key that makes no sense at that point is
CHARALRM: the OPR ERR lamp (`io/dsky/pinball/verbs.c:pin_charalrm`).

## 6.2 Verbs

ENTER after a verb and noun (`io/dsky/pinball/keys.c:pin_enter`) goes to VERBFAN
(`io/dsky/pinball/verbs.c:pin_verbfan`, CM p.325). "Verbs 40 and up are extended verbs":
pinball releases the display and starts the extended verb's own code
(`io/verbs/extended_verbs.c`). Below 40, the regular verbs are a table
(VERBTAB, `pin_vbfandir`):

| Verbs | Action |
|---|---|
| 01-07 | display a noun once, in octal or decimal |
| 11-17 | monitor: redisplay every second (MONREQ waitlist task) |
| 21-25 | load: the crew keys new values into the noun's components |
| 32, 33, 34 | recycle, proceed, terminate (answers to a flashing display, 6.6) |
| 35 | lamp test |
| 36 | fresh start |
| 37 | change program (chapter 7) |

A refused combination (an octal-only noun with a decimal verb, a load
of a no-load noun) is DSPALARM: OPR ERR when keyed, but POODOO 01501
when a *program* asked for it, because then the program is wrong
(`docs/modules/io-pinball.md`, "Program flow and decisions").

## 6.3 Nouns: tables and scaling

A noun is a row in a table: up to three components, each an erasable
and a scale routine that converts between the AGC's fixed-point word and
five decimal digits. Nouns 01-39 are *normal* nouns (one scale for all
components), 40-99 *mixed* nouns (a scale and an address per
component). The listing's PINBALL_NOUN_TABLES.agc keeps these as
parallel arrays of packed octal words; for noun 36 (C pp.272, 275):

```
           ECADR   TIME2          # 36  TIME OF AGC CLOCK (HRS,MIN,SEC)
...
           OCT     24400          # 36  3COMP HMS (DEC ONLY)
```

The C unpacks each into a row of `agc_noun_table_csm[]` (and `_lm`),
keeping the packed words in the comment (`io/dsky/nouns/tables_csm.c`):

```c
    /* 36  Time of AGC clock (HRS,MIN,SEC)
     *     3 comp, dec only; NNADTAB ECADR TIME2, NNTYPTAB 24400 */
    {36,
     3,
     {AGC_SCALE_HMS, AGC_SCALE_HMS, AGC_SCALE_HMS},
     {AGC_DATA_TIME2, AGC_DATA_NONE, AGC_DATA_NONE},
     true,
     false},
```

and a mixed noun, 40, the burn display (time to ignition, velocity to
be gained, velocity gained):

```c
    /* 40  Time to ignition/cutoff / Vg / Delta V (accumulated)
     *     3 comp, dec only, no load;
     *     NNADTAB 64000, NNTYPTAB 24500, RUTMXTAB 16351 */
    {40,
     3,
     {AGC_SCALE_MIN_SEC, AGC_SCALE_VEL3, AGC_SCALE_VEL3},
     {AGC_DATA_TTOGO, AGC_DATA_VGDISP, AGC_DATA_DVTOTAL},
     true,
     true},
```

The noun table cannot hold an address in C (chapter 5), so each
component names a *data id* (`AGC_DATA_TTOGO`), and one function maps
ids to owners: core's erasables directly
(`io/dsky/nouns/data.c:noun_data_access`), everything above the io layer
through `exec/nouns.c:agc_dispatch_noun`, the documented exception to the
layering. The machine-address nouns 01-03, which let the crew display
any address, go through the address map.

The scale routines turn a fraction into digits. HMSOUT, for a time,
divides the centisecond count into hours, minutes and seconds
(`io/dsky/pinball/scale.c:pin_hmsout`, CM p.338):

```c
    int64_t      part[3] = {mag / PIN_CS_HOUR, (mag / PIN_CS_MIN) % 60, mag % PIN_CS_MIN};
```

Others multiply the fraction by a scale-factor constant from the rope's
SFOUTAB and show five digits ("F x 10^5, rounded by 5 E-6", scale.c
ALGORITHM); loading (`io/dsky/pinball/sfin.c`) is the reverse. A component that is shown
but not stored back is a dropped load: the crew's N55 entry never
reached the erasable in one flight bug, so a test now loads every
loadable component and reads the erasable back (flight bug 16,
`docs/TRANSLATION_GUIDE.md`, section 8).

## 6.4 The display loop and the lamps

Nothing writes the display digits directly. Pinball keeps an 11-word
relay table, DSPTAB, one word per pair of digit positions, and marks a
word as pending by making it negative. T4RUPT, every 120 ms
(chapter 4), sends one pending word to output channel 10, where the
DSKY's relays latch it, and then makes a few quick 20 ms passes while
words are pending (`io/service/t4rupt.c`, ALGORITHM). The word format
is documented in `include/agc/io/dsky.h`:

```c
 *   DSPTAB (11 words) feeds output channel 010 as AAAABCCCCCDDDDD: AAAA
 *   relay word = DSPTAB index + 1, B sign relay, CCCCC left char, DDDDD
 *   right char.
```

Each digit is a 5-bit relay code, not a binary number: "0 25, 1 03, 2 31,
3 33, ..." (`agc_dsky_relay_code`). The lamps are bits of channel 11:
KEY REL (bit 5), VERB/NOUN flash (bit 6), OPR ERR (bit 7), UPLINK ACTY
(bit 3) (`AGC_DSKY_LAMP_*`). The PRO key is not a key code at all but a
bit of channel 32, which T4RUPT samples and turns into the PROCKEY job
(`agc_dsky_prockey_request`).

## 6.5 How a program talks to the crew

A program never touches DSPTAB either. It calls the **display interface
routines** (DISPLAY_INTERFACE_ROUTINES.agc, CM pp.1452-1484, LM
pp.1341-1373), which in C are one function with a kind
(`include/agc/io/dsky.h`):

```c
agc_status_t agc_dsky_display(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind, uint16_t vn_or_value, const agc_dsky_cont_t *cont);
```

The kinds are the listing's entry points: GODSP (show), GOFLASH (flash
and wait for an answer), GOPERF1/2/4 ("please perform": a checklist
code or an option code, flashing V50 or V04), GOMARK (mark displays),
PRIODSP (priority displays), CLOCPLAY (a clock-driven burn display),
and an *R* variant of most of them (GOFLASHR, GOPERF4R, ...), which
returns at once and runs the display as a separate job, MAKEPLAY, so
the caller can carry on computing.

Several jobs may want the DSKY at once -- a program, an extended verb
the crew keyed, a mark routine, an alarm. MAKEPLAY arbitrates: a
priority display over another is POODOO 01502 (PRIOBORT), a displaced
display sleeps until the one in front ends, and when the crew has been
using the keyboard, KEY REL lights; pressing it (or finishing their own
display) brings the program's flashing display back (PINBRNCH,
`pin_pinbrnch`) (`docs/modules/io-pinball.md`).

## 6.6 A question and its three answers

A flashing display is a question. The crew answers with V33 (PRO,
proceed), V34 (terminate) or by loading data and ENTER (or V32,
recycle). The AGC routine that flashed returns to one of three places
-- the instructions after the call are three `TC`s, one per answer
(the listing says "call CADR +1/+2/+3"). In C the caller passes three
job ids, one per answer (`include/agc/io/dsky.h`):

```c
typedef struct agc_dsky_cont
{
    agc_job_id_t terminate; /* V34: call CADR +1 */
    agc_job_id_t proceed;   /* V33: call CADR +2 */
    agc_job_id_t enter;     /* data in / V32: CADR +3 */
} agc_dsky_cont_t;
```

and returns: "the job ... wakes at cont->terminate/proceed/enter".

The first question of P52 (IMU realign) shows the pattern. The listing
(P51-P53.agc, C p.739) flashes the option code with GOPERF4R and lists
its three answers, then its immediate return:

```
P52B       CAF     BIT1
           TC      BANKCALL    # FLASH OPTION CODE AND ORIENTATION CODE
           CADR    GOPERF4R
           TC      GOTOPOOH
           TC      +5
           TC      P52B        # NEW CODE -- NEW ORIENTATION CODE INPUT
           TC      PHASCHNG
           OCT     00014
           TC      ENDOFJOB
```

The three `TC`s after the call are the answers: V34 goes to GOTOPOOH
(P00), V33 skips to the option fan five lines on, and a newly loaded
option code re-enters P52B. The fourth return, the immediate one of an
R-kind display, records restart point 4.1 (octal 00014: group 4, phase
1) and ends the job.

The C (`programs/alignment/imu_realign.c:agc_job_imu_realign_option_display`):

```c
agc_status_t agc_job_imu_realign_option_display(agc_job_ctx_t *ctx)
{
    const agc_status_t st = align_flash(ctx, AGC_DSKY_GOPERF4R, OPT_CODE_1, AGC_JOB_GOTOPOOH, AGC_JOB_IMU_REALIGN_OPTION, AGC_JOB_IMU_REALIGN_OPTION_DISPLAY); /* status of the call */

    if (st == AGC_STATUS_OK)
    {
        align_phase4(PH_OPTION_RETURN);
    }

    return (st); /* ENDOFJOB  */
}
```

The three `TC`s became the three ids: terminate goes to
`AGC_JOB_GOTOPOOH` (P00), proceed to `AGC_JOB_IMU_REALIGN_OPTION` (the
code after `TC +5`), enter re-runs this same job (the `TC P52B`). The
PHASCHNG and ENDOFJOB of the immediate return are the last two
statements. Every crew interaction in the program has this shape, which
is why so many C job names end in `_proceed`, `_terminate` or
`_display` (`docs/NAMING.md`).

## 6.7 Paste displays

Sometimes a program shows one verb's data under another verb. Before
ignition the CSM burn shows V06 N40 (time to ignition, VG, delta-V) and
then asks the crew to enable the engine by flashing **V99** -- with N40's
data still in the registers. This is a *paste*: the display interface
shows the first VN and pastes the flashing verb, taken from the erasable
NVWORD1, as the second part. In C (`csm/burns/sps/clock.c:cknvwrd1`,
CKNVWRD1 C pp.705-706):

```c
    agc_dsky_cont_t c99 = {AGC_JOB_ENGINE_ENABLE_TERMINATE,
                           AGC_JOB_ENGINE_ENABLE_PROCEED,
                           AGC_JOB_ENGINE_ENABLE_BYPASS}; /* ENGREQST */

    agc_dsky_set_nvword1((uint16_t) (077777U & ~0400U)); /* NVWORD1 = -BIT9: pastes V99 */

    return (agc_dsky_display(ctx, AGC_DSKY_CLOCPLAY, agc_dsky_vn(6, 40), &c99));
```

`agc_dsky_vn(6, 40)` packs verb and noun "as the AGC VN pseudo-op does";
NVWORD1 = -BIT9 selects V99 (-0 would select V97, the engine-fail
question). PRO on V99 enables ignition; ENTER means "do not use the
SPS", which becomes an RCS trim; terminate ends the program (chapter 9).
Getting the order of the two parts wrong made pinball *execute* V99 as
an extended verb instead of displaying it (`docs/TIPS.md`, section 4:
"A 'paste' display is two NVSUB calls").

## 6.8 The other links: uplink and downlink

The ground talks to the AGC the same way the crew does. An uplinked
word arrives in INLINK and raises UPRUPT (`io/dsky/pinball/keys.c:agc_dsky_uprupt`),
which checks the triple-redundant format (key, complement, key) and
feeds the key to CHARIN exactly as a key press; a bad triple locks the
uplink out until the crew presses RSET. So V71 (a block load into
erasable) is "typed" by the ground. The downlink runs the other way:
DOWNRUPT asks for the next pair of words, and the down-telemetry program
sends them from lists of erasable addresses, one list per program
phase, read through the address map (`io/telemetry/down_telemetry.c`,
`io/telemetry/downlink_lists_csm.c`, `io/telemetry/downlink_lists_lm.c`).

Next: [7. Restarts and the phase tables](07-restarts.md).
