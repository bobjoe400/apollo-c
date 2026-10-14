# 2. Numbers

The AGC has no floating point. Every quantity in the flight program --
a position in metres, a velocity, a time, an angle, a gravitational
constant -- is a fixed-point fraction in a 15-bit word or a pair or
triple of them, multiplied by a power of two that the programmer chose
and wrote in a comment. To read the C you have to read numbers the way
the AGC did. This chapter explains the word, negative zero, single,
double and triple precision, scale factors and overflow, then shows how
the C types carry them and where carrying them as plain integers went
wrong.

## 2.1 The 15-bit ones'-complement word

A word is a sign bit and 14 magnitude bits. Read as a fraction it is a
number in (-1, +1): the word with value *v* means *v* / 2^14.
`include/agc/core/word.h` states the model:

```c
 *   The AGC is a 15-bit ones' complement machine. Every quantity is a
 *   signed fraction in [-1, +1) scaled by a documented power of two:
 *
 *     SP  agc_sp_t  int16_t  sign + 14 magnitude bits   value = v / 2^14
 *     DP  agc_dp_t  int32_t  sign + 28 magnitude bits   value = v / 2^28
 *     TP  agc_tp_t  int64_t  sign + 42 magnitude bits   value = v / 2^42
```

*Ones' complement* means a negative number is the bitwise complement of
its magnitude: +3 is 00003 octal, -3 is 77774. Two consequences matter
everywhere:

* The largest word is POSMAX = 37777 octal = 1 - 2^-14
  (`AGC_SP_MAX`, `AGC_K_POSMAX` in `include/agc/core/constants.h`); half
  is 20000 octal (`AGC_SP_HALF`, the constant HALF). The constant pool's
  names are bit numbers: BIT14 is 1/2, BIT13 is 1/4 (`include/agc/core/constants.h`:
  "BIT13: 1/4").
* There are **two zeros**: +0 (00000) and -0 (77777).

## 2.2 Negative zero

-0 is not an error state; it is a value the hardware produces and the
program tests. The basic instruction CCS ("count, compare and skip")
branches four ways: positive, +0, negative, -0. Adding +1 and -1 gives
-0; subtracting equal counts can give -0. Programs use this. The LM's
jet-timing interrupt (T6RUPT) fires when the TIME6 counter steps from +0
to -0, and the DAP's jet list relies on a subtraction of equal times
leaving -0 (`lm/dap/rcs/t6rupt.c`, comment at line 87):

```c
 * A zero T6 time is -0. The list words reach zero only as -0: MIDORLST's
 * `EXTEND / SU TIME6` of equal counts gives -0 in T6NEXT, LASTCHG's
 * `AD NEG0` gives -0 in T6FURTHA (Q_R-AXIS_RCS_AUTOPILOT L p.1457), and
 * T6JOB moves them into TIME6 unchanged.
```

C integers have one zero, so the C does not represent -0 in its value
types (`docs/TRANSLATION_GUIDE.md`, section 1: "Where the AGC relies on
-0 ... comment it and model the observable behaviour"). At the T6 site
the C stores the 15-bit word 77777 when the list says zero:

```c
agc_rupt_set_time(6, (v == 0) ? (uint16_t) LM_T6_NEGZERO : (uint16_t) v); /* TS TIME6; zero is -0 */
```

Getting this wrong had visible effects. In the first closed-loop runs a
plain integer list gave +0 there, CCS took its "impossible" branch, and
every RCS firing ended in POODOO 1103 (`docs/TIPS.md`, section 1,
closed-loop-fixes L3). The NASSP mission found three more, all the same
lesson -- "A -0 is data, not an error" (`docs/TRANSLATION_GUIDE.md`,
section 8):

* *Flight bug 7.* During powered descent, FINDCDUW limits the middle
  gimbal angle (MGA) to a 70 degree cone. The CDU counters are *two's*
  complement words that the code reads as ones' complement, so a two's
  -1 is a ones' -0. A conversion that sent it to +0 and back produced a
  spurious alarm 00401 once per landing. The C now does the
  reinterpretation the listing does (`lm/landing/findcduw.c:
  agc_lm_findcduw_limit_mga`, label MGA limit, L p.914):

  ```c
  agc_sp_t limited = mga; /* 2's, scale pi; inside the cone L itself (two's -1 is ones' -0, not +0) */
  ```

* *Flight bug 8.* After touchdown the landing analog-display routine
  ALTOUT takes a -0 high word (an altitude just below zero) as new data;
  an assertion that the altitude was non-negative refused it every half
  second (`lm/landing/displays.c`).
* *Flight bug 2.* SKIPTPER stores -TPER of either sign; asserting its
  sign blanked the DSKY after V82 on the escape hyperbola.

## 2.3 Single, double and triple precision

One word gives about four decimal digits. The navigation needs more, so
most quantities are **double precision** (DP): two consecutive words,
the major (high) word first, the minor word worth 2^-14 of it. A DP
number holds 28 magnitude bits, about eight and a half digits.
**Triple precision** (TP) is three words; the interpreter's accumulator
MPAC is three words long so that a DP x DP product keeps its low part.

In C:

| AGC | C type | Range of the integer | Value |
|---|---|---|---|
| SP word | `agc_sp_t` (int16_t) | +-(2^14 - 1) | v / 2^14 |
| DP pair | `agc_dp_t` (int32_t) | +-(2^28 - 1) | v / 2^28 |
| TP triple | `agc_tp_t` (int64_t) | +-(2^42 - 1) | v / 2^42 |
| vector | `agc_vec_t` | three `agc_dp_t` at one scale | |
| matrix | `agc_mat_t` | three row vectors | |

The C integer is *the value of the pair*: `hi * 2^14 + lo`. That is
convenient (C arithmetic on it is AGC arithmetic on the number) and it
is the default throughout the tree. Its limit is the subject of 2.6.

## 2.4 Scale factors

A word only holds fractions, so each quantity is stored divided by a
power of two chosen to make its largest expected value fit. The AGC
programmers wrote the scale as "B*n*" (binary point *n* places to the
right) or "2(*n*)" beside every erasable and constant. The C keeps the
same numbers and writes the scale in a comment on every variable
(`docs/CODING_STANDARD.md`: "every DP/SP variable carries a comment
`scale 2^N unit`").

The best example is the state vector, the position and velocity that
everything else is computed from. `include/agc/core/state.h`:

```c
typedef struct agc_state_vector
{
    agc_vec_t r;    /* position, meters, scale 2^29 always (RN); conic-local copies use 2^27 for moon */
    agc_vec_t v;    /* velocity, m/cs,   scale 2^7 always (VN); conic-local copies use 2^5 for moon  */
    agc_dp_t  t;    /* time of state, centiseconds, scale 2^28        */
    bool      moon; /* primary body (MOONFLAG equivalent)             */
} agc_state_vector_t;
```

Why 2^29 m for position? A stored fraction *f* means *f* x 2^29 m, so
the largest representable distance is 2^29 m = 536,871 km: more than the
Earth-Moon distance, so one scale serves the whole flight from launch to
the Moon. The cost is resolution: one unit of the 28-bit DP is 2^29 /
2^28 = 2 m. Near the Moon the conic routines work on copies at 2^27 m
(134,218 km of range, 0.5 m per unit); the listing writes such
variables' scale as "M B29/B27" (Comanche `ERASABLE_ASSIGNMENTS.agc`,
e.g. `R(T1)/ ... # I(6)TMP POSITION VECTOR AT TIG M B29/B27`). Velocity
is in metres per centisecond at 2^7: up to 128 m/cs = 12.8 km/s, which
covers translunar injection and entry. Time is a count of centiseconds
at 2^28, which wraps after 2^28 cs, about 745.7 hours (`include/agc/core/state.h`,
`AGC_TIME_MODULUS_CS`).

The unit is part of the scaling: the AGC measures time in centiseconds,
so a velocity is m/cs and an acceleration m/cs^2. The gravitational
parameter of the Earth, 3.986 x 10^14 m^3/s^2, appears as 3.986032 x
10^10 m^3/cs^2 scaled 2^36. Side by side
(`nav/conic/common.c`, table `mutable_tab`, and CONIC_SUBROUTINES.agc
p.1305, label MUTABLE):

```c
static const agc_dp_words_t mutable_tab[8] = {
    {022437, 016067}, /* MUE         3.986032 E10 B-36  m^3/cs^2 */
    {015625, 021042}, /* 1/MUE       .25087606 E-10 B+34         */
    ...
```

```
MUTABLE    2DEC*   3.986032 E10 B-36*   # MUE
           2DEC*   .25087606 E-10 B+34* # 1/MUE
```

The major word 022437 octal is 9503, and 9503 / 2^14 = 0.5800, which is
3.986032 x 10^10 / 2^36. The table keeps the two words exactly as the
assembler stored them, a choice explained in 2.6.

Angles are fractions of a revolution: a DP angle *x* means 2 pi *x*
radians, so 0.5 is 180 degrees. The interpretive SIN and COS take such
an argument and return the result halved, "scaled at 2", so that +1
fits (`include/agc/core/interp.h`, NOTES: "result = sin(2 pi x)/2").
Single-precision SPSIN takes its argument scaled at pi instead
(`include/agc/core/word.h`: "Argument scaled at pi").

Scaling is why so much AGC arithmetic ends with a shift: the product of
a 2^29 m position and a 2^-7 cs/m inverse velocity is at 2^22 cs, and the
programmer shifts it to the scale the next equation expects. When you
see `agc_dp_shl(ip, x, 2)` in the C, the comment beside it says which
scale it moves to.

Conversions to and from engineering units exist
(`agc_dp_from_double(x, scale_pow2)` returns round(x 2^(28-N)), `core/word.c`),
but only for tests, the host and the DSKY's decimal conversion:
"Use the double helpers only in tests, the sim harness, and DSKY
conversion, never inside flight algorithms" (`docs/TRANSLATION_GUIDE.md`).
Every flight computation runs on the fixed-point operations.

## 2.5 Overflow

Adding two fractions near 1 gives a result of 1 or more, which a word
cannot hold. The hardware's accumulator has a sixteenth bit for this:
after an add, A holds the overflow, and the instruction that stores A
(TS, "transfer to storage") stores the *overflow-corrected* word -- the
sum minus 1, wrapped -- and tells the program, by skipping the next
instruction, that it overflowed. The C models it word for word
(`core/mpac.c:agc_w_ts`, basic instruction TS):

```c
    if (a16 >= AGC_W_ONE)
    {
        a16 -= AGC_W_ONE;
        o   = 1;
    }
```

The interpreter keeps an overflow indicator, OVFIND, which the
interpretive branch BOV ("branch on overflow") tests and clears. The C
keeps it in the interpreter state `agc_interp_t` (chapter 3), and
`agc_interp_test_ovf(ip)` is BOV. The rule the oracle established
against the emulator, and which surprises a reader who expects
saturating arithmetic, is in `include/agc/core/interp.h`:

```c
 *   DAD/DSU/VAD/VSU/SL/VSL: on overflow the AGC keeps the overflow-corrected
 *             (wrapped, value -+ 1.0) result and sets OVFIND = +-1; these
 *             functions do the same (they do NOT saturate).
```

Only divides and unit vectors saturate (DDV, BDDV, V/SC, UNIT return
DPPOSMAX). So the flight code that expects an overflow checks for it.
A typical pattern, in the time-of-free-fall routine (`nav/auxiliary/
free_fall.c:tff_el1`, TIME_OF_FREE_FALL), is "shift, and if it
overflowed load the signed maximum" -- the interpretive `BOVB SIGNMPAC`:

```c
    /* DMP SL* 0,2 BOVB SIGNMPAC: X = 1/(ALFA Z^2); on overflow (X = 1.0)
     * SIGNMPAC loads +-DPPOSMAX with the sign of the overflow.            */
    ...
    if (agc_interp_test_ovf(ip))
    {
        x = AGC_DP_MAX; /* SIGNMPAC        */
```

## 2.6 Word pairs whose signs disagree

Here is the subtle part. The AGC adds two DP numbers with DAS, which
adds the minor words, carries into the major words, and stores each
word -- **without making the signs agree**. The pair (+3, -100) is a
perfectly good DP number: 3 x 2^-14 - 100 x 2^-28 of its scale. Most
of the time this is invisible, because the value is the same as the
sign-agreeing pair (+2, 16284). But the *next* instruction may read the
words, not the value:

* a multiply (DMP) multiplies word by word, and the minor x minor
  partial product keeps only its high half, so the low bits of the
  product depend on how the value was split;
* a divide, a square root, a right shift or an arcsine does the same;
* a sign test (CCS, BZMF, BPL, the interpreter's SIGNMPAC) looks at the
  **major word only**.

`include/agc/core/word.h` explains why the value type cannot hold this:

```c
/* A DP quantity as the AGC stores it: two ones'-complement words whose
 * signs may disagree after a DAS, DCS or BDSU (hi = +3, lo = -100 is
 * 3 2^-14 - 100 2^-28 of the scale). agc_dp_t keeps only the value
 * hi 2^14 + lo; code whose low bits depend on the split (a following MP,
 * DV, SR, SQRT, or a basic-language read of the high word) keeps the
 * words (include/agc/core/interp_words.h).                               */
typedef struct agc_dp_words
{
    agc_sp_t hi; /* major word, the quantity's scale                */
    agc_sp_t lo; /* minor word, 2^-14 of the major word's scale     */
} agc_dp_words_t;
```

So the C has two forms:

* The **value API** (`include/agc/core/interp.h`): `agc_dp_t`, functions
  such as `agc_dp_add`, `agc_dp_mul`, `agc_vec_unit`. It is the default
  and is exact wherever the words agree in sign. Internally it splits the
  value into sign-agreeing words and runs the same word code
  (`core/interp/scalar.c:agc_dp_add` is
  `agc_w_join(agc_w_dad(agc_w_split(a), agc_w_split(b), &ip->ovfind))`).
* The **word API** (`include/agc/core/interp_words.h`): `agc_dp_words_t`,
  `agc_tp_words_t` (MPAC, MPAC+1, MPAC+2) and one `agc_mpac_*` function
  per interpretive instruction, which "takes MPAC (and an operand as
  stored) and returns MPAC as the AGC leaves it". A sequence `DLOAD X /
  DMP Y / SR 4 / STORE Z` is written

  ```c
  m = agc_mpac_dload(x); m = agc_mpac_dmp(m, y);
  m = agc_mpac_shift(ip, m, -4); z = agc_mpac_store(m);
  ```

The word form is used where the oracle showed the split matters: the
conic routines went from 64 residual cases to none when their whole
chain moved to words (`docs/TIPS.md`, section 1), and constants are kept
as the assembled words (the `mutable_tab` above) for the same reason.

The most consequential case was a sign. The attitude-manoeuvre code
(KALCMANU's DELCOMP, ANGLFIND.agc) uses `BOVB SIGNMPAC`: on overflow,
load plus or minus the maximum, with the sign of MPAC. SIGNMPAC takes
the sign by CCS on the **major word**. After a shift that overflows to
exactly 1, the corrected major word is +0 while the minor words can be
negative: the AGC's answer is "+". The first C took the sign of the DP
value, which was negative, made a diagonal element of the rotation
matrix -1 instead of +1, and during rendezvous in the NASSP mission the
LM swung 40 to 60 degrees in roll for a 0.01 degree correction (flight
bug 15, `docs/TRANSLATION_GUIDE.md`, section 8). The fix is written on
the words (`imu/maneuver_calc.c:store_signmpac`):

```c
    if (agc_interp_test_ovf(ip))
    {
        const bool neg = (w.hi != 0) ? (w.hi < 0) : (ovf < 0);            /* CCS A on the major word */
        w              = agc_dp_to_words(neg ? -AGC_DP_MAX : AGC_DP_MAX); /* SIGNMPAC: DCA / DCS DPOSMAX */
    }
```

The prolog states the rule: "SIGNMPAC (RTB_OP_CODES.agc p.1399) takes
the sign by CCS on the major word alone." (The C words keep no -0, so a
zero major word takes the sign of the overflow, which is what the
corrected +0 or -0 would have said.)

## 2.7 Reading a line of C arithmetic

Putting it together, a line such as (from `nav/auxiliary/free_fall.c`)

```c
agc_dp_t inv_az = agc_tp_to_dp(agc_tp_shift(ip, agc_dp_mul_tp(dn, tff->tff1_alf), na)); /* 1/(ALFA Z), E 2^(11+NA) */
```

reads: multiply two DP fractions keeping the triple-precision product
(DMP), shift the TP left by NA places setting OVFIND if it overflows
(SL*), keep the top two words (STORE); the result is 1/(alpha Z) scaled
2^(11+NA). The trailing comment is the scale. Every arithmetic function
takes `ip` when the AGC instruction can set OVFIND; functions that
cannot overflow (`agc_dp_mul_tp`, `agc_dp_sq`) do not.

Next: [3. The interpreter](03-interpreter.md).
