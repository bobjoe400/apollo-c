# TRANSLATION_GUIDE.md -- how AGC assembly maps onto apollo-c

Read this before translating a module. The headers in include/agc/ are the
contract; this document explains the intent behind them.

## 1. Numeric model (include/agc/agc_word.h)

| AGC            | C type       | Representation                                   |
|----------------|--------------|--------------------------------------------------|
| SP word        | `agc_sp_t`   | int16_t; sign + 14 bits; fraction = v / 2^14     |
| DP word pair   | `agc_dp_t`   | int32_t; sign + 28 bits; fraction = v / 2^28     |
| TP (MPAC 3 wd) | `agc_tp_t`   | int64_t; sign + 42 bits; fraction = v / 2^42     |
| Overflow flag  | `agc_ovf_t`  | +1 / 0 / -1 (OVFIND / A overflow bit)            |

Ones' complement -0 is not represented. Where the AGC relies on -0 (CCS on a
negative zero, sign of a zero result, `DCOMP` of zero) comment it and model
the observable behaviour, not the bit pattern.

All AGC quantities are fractions scaled by a power of two. **Preserve the
AGC scale factor** and write it in a comment. Helpers:

    agc_dp_from_double(double x, int scale_pow2)   /* x / 2^scale -> dp  */
    agc_dp_to_double(agc_dp_t v, int scale_pow2)   /* dp * 2^scale -> x  */

Use the double helpers only in tests, the sim harness, and DSKY conversion,
never inside flight algorithms (they must run on the fixed-point ops).

## 2. Interpreter (include/agc/agc_interp.h)

The interpretive language is not emulated. Each interpretive instruction
becomes a call on the fixed-point library, operating on C locals instead of
MPAC and the push-down list. The interpreter's state that *does* matter is
kept in `agc_interp_t`: `ovfind` (OVFIND), the four index registers
X1,X2,S1,S2 when they are used as data, and the 10 interpretive switches.

Mapping (D = scalar DP, V = vector):

| Interpretive  | C                                              |
|---------------|------------------------------------------------|
| DLOAD X       | `agc_dp_t a = X;`                              |
| DAD / DSU     | `agc_dp_add(ip,a,b)` / `agc_dp_sub(ip,a,b)`    |
| BDSU          | `agc_dp_sub(ip,b,a)`                           |
| DMP           | `agc_dp_mul(ip,a,b)` (TP product rounded like DMPR; use `agc_tp_mul` when the AGC keeps TP) |
| DDV / BDDV    | `agc_dp_div(ip,a,b)` / `agc_dp_div(ip,b,a)`    |
| DSQ           | `agc_dp_sq(ip,a)`                              |
| SQRT          | `agc_dp_sqrt(ip,a)`                            |
| SIN / COS     | `agc_dp_sin(ip,a)` / `agc_dp_cos(ip,a)` -- arg scaled at 2^0 revolutions (i.e. radians / 2pi), result scaled at 2 (so result = sin/2) |
| ASIN / ACOS   | `agc_dp_asin(ip,a)` -- arg = x/2 ... see header; result = angle/2pi |
| SL n / SR n   | `agc_dp_shl(ip,a,n)` / `agc_dp_shr(a,n)`; SLR/SRR = rounding variants |
| ROUND         | `agc_dp_round(a)` (DP -> SP in high word, result still agc_dp_t with low word zero) |
| DCOMP / ABS   | `agc_dp_neg(a)` / `agc_dp_abs(a)`              |
| NORM X        | `agc_dp_norm(a,&shift)`                        |
| SIGN X        | `agc_dp_sign(a, X)`                            |
| VLOAD/STORE   | `agc_vec_t v = ...`                            |
| VAD / VSU     | `agc_vec_add(ip,u,v)` / `agc_vec_sub(ip,u,v)`  |
| VXSC / V/SC   | `agc_vec_scale(ip,v,s)` / `agc_vec_div(ip,v,s)`|
| DOT / VXV     | `agc_vec_dot(ip,u,v)` / `agc_vec_cross(ip,u,v)`|
| ABVAL / VSQ   | `agc_vec_abval(ip,v)` / `agc_vec_sq(ip,v)`     |
| UNIT          | `agc_vec_unit(ip,v,&mag)` (also leaves |v| and |v|^2 as the AGC does) |
| VPROJ         | `agc_vec_proj(ip,u,v)`                         |
| MXV / VXM     | `agc_mat_vec(ip,m,v)` / `agc_vec_mat(ip,v,m)`  |
| VSL n / VSR n | `agc_vec_shl(ip,v,n)` / `agc_vec_shr(v,n)`     |
| VCOMP / VDEF  | `agc_vec_neg(v)` / struct init                 |
| BOV / BOVB    | `if (agc_interp_test_ovf(ip)) {...}` (clears OVFIND like the AGC) |
| BZE/BPL/BMN/BHIZ | ordinary comparisons                        |
| SET/CLEAR/INVERT/BON/BOFF switch | `agc_flag_set/clear/test()` on the flagword API |
| CALL / RVQ    | C call / return                                |
| STCALL/STODL  | assignment then call                           |
| SETPD/PUSH/PDDL/PDVL | C locals (the push list is a spill area)  |
| EXIT          | end of interpretive section (no-op)            |

Overflow semantics: `agc_dp_*` operations saturate the way the AGC does and
set `ip->ovfind`. Code that tests BOV must call `agc_interp_test_ovf`, which
clears the flag, mirroring the AGC.

## 3. Basic (machine) code

Translate meaning, not instructions. `CCS X` followed by four branches is a
sign test. `INDEX`/table lookups are array indexing with a bounds assertion.
`INHINT`/`RELINT` become `agc_inhint()` / `agc_relint()`. Bank switching
(BANK, EBANK=, SETLOC, 2CADR, BBCON, superbank) is dropped; there is one flat
address space.

Shared erasables: use the accessors in `agc_state.h`. Module-private
erasables: `static` variables in the module's `.c` file, named after the AGC
label (lower case), with the AGC scale in a comment.

Flags: the AGC flagwords FLAGWRD0..FLAGWRD11 are bit-named in
`agc_flags_csm.h` / `agc_flags_lm.h`. Use `agc_flag_test(AGC_FLAG_xxx)`.

## 4. Executive and waitlist (include/agc/agc_exec.h)

* A **job** (NOVAC/FINDVAC) is an `agc_job_id_t` enum member; its body is a
  `agc_status_t agc_job_<name>(agc_job_ctx_t *ctx)` function registered in
  the `switch` in `src/exec/dispatch.c`. Priority is kept.
* A **task** (WAITLIST/TWIDDLE, VARDELAY, LONGCALL) is an `agc_task_id_t`,
  dispatched likewise. Delay is in centiseconds like the AGC.
* `ENDOFJOB` = return `AGC_STATUS_OK` from the job function. `JOBSLEEP`/
  `JOBWAKE` = `agc_exec_job_sleep()` / `agc_exec_job_wake()`. `CHANG1/2`
  (pre-emption) is handled inside the executive; job bodies run to completion
  or to a sleep/yield point.
* A job that in the AGC continues after an asynchronous event (e.g. a DSKY
  response, an IMU pulse train) is split into separate job functions, one per
  resumption point, with the continuation `agc_job_id_t` passed to the waiting
  primitive. This replaces the AGC pattern of storing a return address.
* `PHASCHNG`, restart groups, `RESTART` tables: `agc_phase_change()` with the
  same group/phase codes, so the restart logic in exec is faithful.
* `ALARM`, `BAILOUT`, `POODOO`, `ABORT`: `agc_alarm()`, `agc_bailout()`,
  `agc_poodoo()` then return an error status.

## 5. I/O channels (include/agc/agc_channels.h)

`READ`/`WRITE`/`ROR`/`WAND`/etc. become `agc_chan_read(n)` /
`agc_chan_write(n, v)` / `agc_chan_or` / `agc_chan_and` on the 15-bit channel
image kept in core. Bit names are in `agc_channels.h` (from the LM
INPUT_OUTPUT_CHANNEL_BIT_DESCRIPTIONS and the CM erasable comments).

## 6. DSKY / pinball (include/agc/agc_dsky.h)

Verb/noun tables stay tables (static const arrays). The display registers
R1-R3, VERB, NOUN, PROG are a struct. Noun scaling routines translate to
functions that convert between DP erasable and 5-digit decimal/octal.

## 7. What to keep from the AGC text

Keep: labels (as comments and as function names), page numbers, the author's
functional descriptions, scale factors, alarm codes, iteration bounds,
constants with their decimal and octal forms. Drop: bank bookkeeping,
COUNT/SETLOC/BLOCK pseudo-ops, address arithmetic.
