# 3. The interpreter

The navigation and guidance in the AGC are vector and matrix algebra on
double-precision numbers. The machine's own instructions work on one
15-bit word at a time, and a DP vector add written in them is dozens of
instructions. So most of the flight program is written in a second,
denser language -- the **interpretive language** -- and executed by an
interpreter that lives in the rope (INTERPRETER.agc, C pp.1107-1199, L
pp.1002-1094). This chapter describes that language, how the C turns it
into an ordinary function library, and walks through one interpretive
routine beside its C.

## 3.1 The interpretive language

An interpretive program is a list of words in the rope. Each word holds
either a *pair* of 7-bit op codes or an operand address; the
interpreter picks a pair, executes the first operation, keeps the second
for the next round, and fetches addresses from the following words as
the operations need them. The listing's main loop, DANZIG and NEWOPS,
says so (INTERPRETER.agc, C pp.1108):

```
# AT THE END OF MOST INSTRUCTIONS, CONTROL IS GIVEN TO DANZIG TO DISPATCH THE NEXT OPERATION.
...
DANZIG     CA     BANKSET     # SET BBANK BEFORE TESTING NEWJOB SO THAT
           ...
NOIBNKSW   CCS    EDOP        # SEE IF AN ORDER CODE IS LEFT OVER FROM
           TCF    OPJUMP      # THE LAST PAIR RETRIEVED.  IF SO, EXECUTE.
           CCS    NEWJOB      # SEE IF A JOB OF HIGHER PRIORITY IS
           TCF    CHANG2      # PRESENT, AND IF SO, CHANGE JOBS.
```

Note the second test: between every pair of interpretive operations the
interpreter looks at NEWJOB and gives up the processor to a
higher-priority job. Chapter 4 returns to this.

The interpreter's machine model is small:

* **MPAC**, the multi-purpose accumulator: three words, holding a DP or
  TP scalar, or in vector mode six words, a DP vector. Every operation
  works on MPAC and (at most) one operand. The interpreter tracks a
  *mode* (DP, TP or vector) so that `VLOAD` and `DLOAD` set MPAC up
  differently.
* The **push-down list**: the first 43 words of the job's VAC area, used
  as a stack. `PUSH` stores MPAC on it; an operation whose address word
  is missing *pops* its operand from it ("ADDRESSES LESS THAN 45D ARE
  TAKEN TO BE RELATIVE TO THE WORK AREA", C p.1110).
* **OVFIND**, the overflow indicator (chapter 2), set by an operation
  that overflows and tested and cleared by BOV/BOVB.
* Index registers X1, X2 and step registers S1, S2, for indexed
  addresses (`DLOAD* MUTABLE -2,1` reads MUTABLE minus X1 minus 2).
* Interpretive **switches**: bits in the flagwords set, cleared and
  tested by `SET`, `CLEAR`, `BON`, `BOFF`.
* Calls: `CALL` saves the return in QPRET; `RVQ` returns; `ITA`/`STQ`
  save the return address in an erasable; `EXIT` leaves interpretive
  mode and continues in basic code ("+0 OP CODE IS EXIT", C p.1108).
* **RTB** ("return to basic"): call a basic-code subroutine from the
  middle of interpretive code, the op codes of RT8_OP_CODES.agc
  (LOADTIME, SGNAGREE, CDULOGIC, NORMUNIT, ...).

Typical operations, with their semantics in the C library
(`include/agc/core/interp.h` NOTES):

| Op | Meaning | Note |
|---|---|---|
| DLOAD, VLOAD | load MPAC with a DP / vector | |
| STORE, STODL, STOVL, STCALL | store MPAC, then load / call | the store happens first |
| DAD, DSU, BDSU | add, subtract, subtract from | wrap on overflow, set OVFIND |
| DMP, DMPR | multiply (truncated / rounded) | scales add |
| DDV, BDDV | divide (by / into) | saturates on overflow |
| SQRT, SIN, COS, ASIN, ACOS | functions | angles in revolutions |
| SL, SR, SLR, SRR, NORM | shifts, normalise | |
| VAD, VSU, VXSC, V/SC | vector add, subtract, scale, divide by scalar | |
| DOT, VXV, ABVAL, VSQ, UNIT, VPROJ | vector products, magnitude, unit vector | UNIT gives a *half*-unit vector |
| MXV, VXM | matrix times vector, vector times matrix | three DOTs, no shift |

## 3.2 The language as a C library

The C does not emulate the interpreter. Each interpretive instruction
became one C function on the fixed-point types of chapter 2, and an
interpretive routine became an ordinary C function that calls them on
C local variables. The header's purpose line says it plainly
(`include/agc/core/interp.h`): "The interpretive language as a C
library: one function per interpretive instruction of INTERPRETER.agc".
`docs/TRANSLATION_GUIDE.md` (section 2) has the full table; a few rows:

| Interpretive | C |
|---|---|
| `DLOAD X` | `agc_dp_t a = X;` |
| `DAD` / `DSU` | `agc_dp_add(ip,a,b)` / `agc_dp_sub(ip,a,b)` |
| `DMP` / `DMPR` | `agc_dp_mul_trunc(a,b)` / `agc_dp_mul(ip,a,b)` |
| `VXSC` | `agc_vec_scale(ip,v,s)` |
| `UNIT` | `agc_vec_unit(ip,v,&abval,&vsq)` |
| `MXV` | `agc_mat_vec(ip,m,v)` |
| `BOV` | `if (agc_interp_test_ovf(ip)) {...}` |
| `SETPD`/`PUSH`/`PDDL`/`PDVL` | C locals (the push list is a spill area) |
| `CALL` / `RVQ` | C call / return |
| `EXIT` | end of interpretive section (no-op) |

What disappears: the op-code pairs, the mode, address decoding, bank
switching, the push-down pointer. What remains is the interpreter state
that changes results, kept in one struct that the caller passes by
pointer (`include/agc/core/interp.h`):

```c
typedef struct agc_interp
{
    agc_ovf_t ovfind;                  /* OVFIND: 0, or +-1 (sign of the overflow)    */
    agc_sp_t  x1;                      /* X1 index register, unscaled (as data)       */
    agc_sp_t  x2;                      /* X2 index register, unscaled                 */
    agc_sp_t  s1;                      /* S1 step register, unscaled                  */
    agc_sp_t  s2;                      /* S2 step register, unscaled                  */
    bool      sw[AGC_INTERP_SWITCHES]; /* interpretive switch bits    */
    bool      dvsign_neg;              /* DVSIGN (= DOTINC) holds -0: last divide gave a negative quotient (see DVOVF) */
    agc_dp_t  mpac;                    /* MPAC as left by a DAD/DDV whose result only sets OVFIND (BOV next), scale of its operands */
} agc_interp_t;
```

Even a stale register is kept when it shows: DVSIGN is left by the last
divide, and "V/SC and UNIT overflow exits use the DVSIGN of the previous
divide" (`docs/TIPS.md`, section 1). The arithmetic functions reproduce
the interpreter's own subroutines (DMPSUB, ROUNDSUB, DDV, SQRTSUB, the
SIN polynomial) word by word, which is why the oracle comparison
against the emulator is bit-exact for sign-agreeing operands
(`include/agc/core/interp.h`, NOTES; chapter 12).

## 3.3 DMP and DMPR: truncate or round

The interpreter has two DP multiplies. DMP leaves the full TP product
in MPAC; if the next instruction stores it, the third word is simply
dropped: the result is *truncated*. DMPR rounds the product to DP. In C
they are two functions (`core/interp/scalar.c`), `agc_dp_mul` for DMPR
and `agc_dp_mul_trunc` for "DMP (DMP1, INTERPRETER.agc p.1149) used as a
DP":

```c
agc_dp_t agc_dp_mul(agc_interp_t *ip, agc_dp_t a, agc_dp_t b)
{
    ...
    m = agc_w_dmpsub(agc_w_split(a), agc_w_split(b));
    m = agc_w_roundsub(m, &ip->ovfind);
    return (agc_w_join(agc_w3_top(m)));
}

agc_dp_t agc_dp_mul_trunc(agc_dp_t a, agc_dp_t b)
{
    agc_w3_t m = agc_w_dmpsub(agc_w_split(a), agc_w_split(b)); /* DMPSUB TP product */
    return (agc_w_join(agc_w3_top(m)));
}
```

The difference is one bit in 2^28, and it matters. Most guidance code
uses DMP. Over an entry trajectory, rounding where the listing
truncated grew to hundreds of LSB ("DTEAROT one cs -> RT.y 98 LSB",
`docs/TIPS.md`, section 1), so the review of every lane checked each
multiply against the listing ("grep the listing for DMPR before writing
a rounded multiply"). When the next instruction reads the third word
-- a `DMP` followed by `SL1`, `SQRT` or `ROUND` -- the C keeps the TP
product (`agc_dp_mul_tp`) or the word form (`agc_mpac_dmp`), because the
shift brings the third word's bits into the result.

VXSC, the vector-times-scalar, rounds each component (VROUND); in the
C it is `agc_vec_scale` (`core/interp/vector.c`, VXSC p.1145):

```c
    for (k = 0; k < AGC_VEC_N; k++)
    {
        m      = agc_w_dmpsub(agc_w_split(v.c[k]), agc_w_split(s));
        m      = agc_w_roundsub(m, &ip->ovfind); /* VROUND           */
        r.c[k] = agc_w_join(agc_w3_top(m));
    }
```

## 3.4 UNIT and half-unit vectors

A unit vector has components up to 1, which a fraction cannot hold. The
AGC's UNIT returns a **half-unit** vector, v / (2|v|), "scaled 2^1"
(`core/vec_div.c:agc_vec_unit`, UNIT pp.1175-1179), and as a side
effect leaves |v| and |v|^2 in the push-down cells 34D and 36D, which
code after a UNIT often reads. The C returns them through two optional
out-parameters:

```c
/* UNIT: returns unit vector; *abval = |v| (scaled as v), *vsq = |v|^2 as
 * left at 34D/36D by the AGC. Either out pointer may be NULL.            */
agc_vec_t agc_vec_unit(agc_interp_t *ip, agc_vec_t v, agc_dp_t *abval, agc_dp_t *vsq);
```

The same convention runs through the whole program: the orientation
matrix REFSMMAT is stored "SCALE OF B1" (`include/agc/core/state.h`),
its rows half-unit vectors, and since MXV is "three plain DOTs, no
shift", multiplying by it halves a vector, which later code doubles back
(`docs/TIPS.md`, section 3).

## 3.5 The push-down list becomes local variables

Interpretive code pushes intermediate results and pops them later, often
implicitly: an instruction written without an address takes the top of
the list. In C each pushed value is a named local where it was pushed.
This is a reading exercise, because the listing does not say what is
popped. An example from Luminary's INITVELX (P34-35_P74-75.agc, L
p.491), which prepares the LM's steering constants:

```
INITVELX   LXA,1   DLOAD*
                   RTX1
                   MUTABLE -2,1
           PUSH    DMP
                   R1A
           SR1     DDV
                   R1
           STODL   MU/A
           SR
                   6
           STORE   MUASTEER
```

`DLOAD* MUTABLE -2,1` loads MU of the Earth or the Moon (indexed by
X1), `PUSH` saves it on the push-down list, and `DMP R1A / SR1 / DDV R1`
forms MU R1A / 2 R1.

The `STODL MU/A` has no load address, so it loads the value pushed at
the top, MU. MUASTEER is therefore MU shifted right 6 -- not MU/A
shifted. The C (`programs/targeting/initvel.c:iv_finish`) names the
pushed value and uses it:

```c
        agc_dp_t mu = agc_dp_from_words(agc_conic_mu_(...)); /* MUTABLE: MU, body scale */
        ...
        a = agc_dp_shr(agc_dp_mul_trunc(mu, w->lout.r1a), 1);
        rz->mu_a     = agc_dp_div(ip, a, w->lout.r1); /* STODL MU/A  */
        rz->muasteer = agc_dp_shr(mu, 6);             /* (DLOAD of the pushed MU) SR 6 */
```

Read the other way it took the square root of a negative number during
the rendezvous burn and aborted with 01302 (flight bug 17,
`docs/TRANSLATION_GUIDE.md`, section 8: "Follow the push-down list
through the routine before writing the expression").

## 3.6 Walkthrough: CALCGTA, gyro torquing angles

CALCGTA computes the three angles through which the gyros must be
torqued to turn the IMU's stable member to a desired orientation. The
input is the desired stable-member axes expressed in the present ones
(XDC, YDC, ZDC, the rows of a direction-cosine matrix, half-unit); the
outputs are the outer, inner and middle gyro angles OGC, IGC, MGC in
revolutions. It is called by every fine alignment (P52, P57) and is
short enough to read whole.

The listing (INFLIGHT_ALIGNMENT_ROUTINES.agc, C pp.1355-1356; the same
routine is in Luminary at L p.1249), shortened only by dropping blank
lines:

```
CALCGTA    ITA     DLOAD           # PUSHDOWN  00-03,16D-27D,34D-37D
                   S2              # XDC = (XD1 XD2 XD3)
                   XDC             # YDC = (YD1 YD2 YD3)
           PDDL    PDDL            # ZDC = (ZD1 ZD2 ZD3)
                   HI6ZEROS
                   XDC     +4
           DCOMP   VDEF
           UNIT
           STODL   ZPRIME          # ZP = UNIT(-XD3 0 XD1) = (ZP1 ZP2 ZP3)
                   ZPRIME
           SR1
           STODL   SINTH           # SIN(IGC) = ZP1
                   ZPRIME  +4
           SR1
           STCALL  COSTH           # COS(IGC) = ZP3
                   ARCTRIG
           STODL   IGC             # Y GYRO TORQUING ANGLE   FRACTION OF REV.
                   XDC     +2
           SR1
           STODL   SINTH           # SIN(MGC) = XD2
                   ZPRIME
           DMP     PDDL
                   XDC     +4      # PD00 = (ZP1)(XD3)
                   ZPRIME  +4
           DMP     DSU
                   XDC             # MPAC = (ZP3)(XD1)
           STADR
           STCALL  COSTH           # COS(MGC) = MPAC - PD00
                   ARCTRIG
           STOVL   MGC             # Z GYRO TORQUING ANGLE   FRACTION OF REV.
                   ZPRIME
           DOT
                   ZDC
           STOVL   COSTH           # COS(OGC) = ZP . ZDC
                   ZPRIME
           DOT
                   YDC
           STCALL  SINTH           # SIN(OGC) = ZP . YDC
                   ARCTRIG
           STCALL  OGC             # X GYRO TORQUING ANGLE   FRACTION OF REV.
                   S2
```

The C (`imu/alignment.c:agc_imu_calcgta`), complete except for its
prolog:

```c
void agc_imu_calcgta(agc_interp_t *ip, const agc_mat_t *dc, agc_dp_t gyro[3])
{
    agc_vec_t xdc = dc->row[0]; /* XDC, B1 */

    /* ZP = UNIT(-XD3, 0, XD1).                                           */
    agc_vec_t zp = agc_vec_unit(ip, agc_vec_make(agc_dp_neg(xdc.c[2]), 0, xdc.c[0]), NULL, NULL); /* ZPRIME, B1 */

    /* SIN(IGC) = ZP1, COS(IGC) = ZP3 (SR1: half-unit -> /4).             */
    agc_dp_t igc = agc_imu_arctrig(ip, agc_dp_shr(zp.c[0], 1), agc_dp_shr(zp.c[2], 1)); /* IGC, rev 2^0 */

    /* SIN(MGC) = XD2;  COS(MGC) = (ZP3)(XD1) - (ZP1)(XD3).                */
    /* DMP PDDL / DMP DSU / STCALL: the DSU acts on MPAC, MPAC+1 and the
     * store drops MPAC+2, so both products are truncated (DMP).          */
    agc_dp_t pd00 = agc_dp_mul_trunc(zp.c[0], xdc.c[2]);                       /* ZP1 XD3, 2^-2 */
    agc_dp_t cosm = agc_dp_sub(ip, agc_dp_mul_trunc(zp.c[2], xdc.c[0]), pd00); /* COS(MGC)/4, 2^0 */
    agc_dp_t mgc  = agc_imu_arctrig(ip, agc_dp_shr(xdc.c[1], 1), cosm);        /* MGC, rev 2^0 */

    /* COS(OGC) = ZP . ZDC,  SIN(OGC) = ZP . YDC.                          */
    agc_dp_t coso = agc_vec_dot(ip, zp, dc->row[2]); /* COS(OGC)/4, 2^0 */
    agc_dp_t sino = agc_vec_dot(ip, zp, dc->row[1]); /* SIN(OGC)/4, 2^0 */
    agc_dp_t ogc  = agc_imu_arctrig(ip, sino, coso); /* OGC, rev 2^0 */

    gyro[0] = ogc;
    gyro[1] = igc;
    gyro[2] = mgc;
    agc_imu_gyro_buf_set(AGC_IMU_GBUF_OGC, gyro);
}
```

Line by line:

* `ITA S2` saves the return address in S2; `STCALL OGC / S2` at the end
  stores OGC and returns through it. In C that is the function's entry
  and its end.
* `DLOAD XDC / PDDL / PDDL / DCOMP VDEF` builds the vector (-XD3, 0, XD1)
  through the push-down list (the two PDDLs push XD1 and a zero,
  `HI6ZEROS`). In C, `agc_vec_make(agc_dp_neg(xdc.c[2]), 0, xdc.c[0])`.
* `UNIT / STODL ZPRIME` is `agc_vec_unit`; the half-unit result is why
  `SR1` then gives sin/4 and cos/4, the inputs ARCTRIG expects ("THE
  INPUTS ARE SIN/4 AND COS/4", p.1357).
* `DMP PDDL ... DMP DSU` computes ZP3 XD1 - ZP1 XD3 with the first
  product pushed. The C names it `pd00`, after the push-down cell 00D the
  listing comment uses, and calls the truncating multiply because no
  rounding instruction is involved; the comment says why.
* `DOT ZDC` and `DOT YDC` are `agc_vec_dot` with the matrix rows.
* The C has one extra line: CALCGTA's three output words are also the
  buffer the gyro pulse routine IMUPULSE reads (OGC, IGC, MGC are
  consecutive erasables), so the C stores the angles into that buffer.

ARCTRIG, the subroutine it calls, chooses between ARCCOS and ARCSIN to
keep accuracy (`imu/alignment.c:agc_imu_arctrig`, ARCTRIG / TRIG1 /
TRIG2, p.1357):

```c
    if (agc_dp_sub(ip, agc_dp_abs(sinth), QTSN45) >= 0)
    {
        /* (45,135) or (-135,-45): X = ARCCOS(COS) WITH SIGN(SIN).        */
        theta = agc_dp_acos(ip, agc_dp_shl(ip, costh, 1));
        return (agc_dp_sign(theta, sinth));
    }
    /* TRIG1: (-45,45) or (135,-135): X = ARCSIN(SIN).                    */
    theta = agc_dp_asin(ip, agc_dp_shl(ip, sinth, 1));
```

The listing's `DLOAD ABS / DSU BMN QTSN45` -- "ABS(SIN/4) - SIN(45)/4",
branch if negative -- is the `if`; its comments survive as the C's
comments.

## 3.7 Where the word form appears

When a routine's results depend on how MPAC's words split (chapter 2,
2.6), its C is written with `agc_mpac_*` calls on `agc_tp_words_t`,
one call per instruction, so it reads even closer to the listing. A
DCMTOCDU fragment (`imu/maneuver_calc.c`):

```c
    agc_tp_words_t m = agc_mpac_dmp(agc_mpac_dload(x), y); /* DMP, TP MPAC */
```

is `DLOAD x / DMP y`, leaving the three words of the product in `m` for
the shift that follows. The two forms agree wherever the words agree;
the review lanes moved a chain to words only where the oracle showed a
difference.

Next: [4. Time and work](04-time-and-work.md).
