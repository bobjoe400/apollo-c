# TRANSLATION_GUIDE.md -- how AGC assembly maps onto apollo-c

Read this before translating a module. The headers in include/agc/ are the
contract; this document explains the intent behind them.

## 1. Numeric model (include/agc/core/word.h)

| AGC            | C type       | Representation                                   |
|----------------|--------------|--------------------------------------------------|
| SP word        | `agc_sp_t`   | int16_t; sign + 14 bits; fraction = v / 2^14     |
| DP word pair   | `agc_dp_t`   | int32_t; sign + 28 bits; fraction = v / 2^28     |
| TP (MPAC 3 wd) | `agc_tp_t`   | int64_t; sign + 42 bits; fraction = v / 2^42     |
| Overflow flag  | `agc_ovf_t`  | +1 / 0 / -1 (OVFIND / A overflow bit)            |

Ones' complement -0 is not represented in the value types. Where the AGC
relies on -0 (CCS on a negative zero, sign of a zero result, `DCOMP` of
zero) comment it and model the observable behaviour; where it depends on
the words themselves, keep the words (`agc_dp_words_t`, below, and section
8: a -0 is data).

All AGC quantities are fractions scaled by a power of two. **Preserve the
AGC scale factor** and write it in a comment. Helpers:

    agc_dp_from_double(double x, int scale_pow2)   /* x / 2^scale -> dp  */
    agc_dp_to_double(agc_dp_t v, int scale_pow2)   /* dp * 2^scale -> x  */

Use the double helpers only in tests, the sim harness, and DSKY conversion,
never inside flight algorithms (they must run on the fixed-point ops).

**Word-level values (added 2026-10-11).** The AGC stores a DP/TP value as
two or three 15-bit words whose signs may disagree (e.g. R1A = (00001,
40057)); `agc_dp_t` collapses that to one integer and loses up to a few LSB
where the AGC later shifts, divides, square-roots or arc-sines the stored
words. Where an oracle case shows such a residual, keep the value as
`agc_dp_words_t` / `agc_tp_words_t` (include/agc/core/word.h) and use the
per-instruction `agc_mpac_*` functions in include/agc/core/interp_words.h
(DLOAD STORE DAD DSU BDSU TAD DMP DMPR DSQ DDV BDDV SQRT ROUND SL SR SLR
SRR NORM DCOMP ABS SIGN SIN COS ASIN ACOS DOT VSQ, plus DAS; and on
`agc_vec_words_t` vectors DOT VXM MXV VXV VXSC VAD VSL VSR). Convert at
the boundary with the value API. Default to `agc_dp_t`; go word-level only
where the AGC's behaviour depends on the split.

## 2. Interpreter (include/agc/core/interp.h)

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
| DMP           | `agc_dp_mul_trunc(ip,a,b)` (bit-exact DMP, TP product truncated); `agc_dp_mul_tp` when the AGC keeps the TP product |
| DMPR          | `agc_dp_mul(ip,a,b)` (rounded product) |
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

Overflow semantics (verified against yaAGC by core-numeric): DAD, DSU, BDSU,
TAD, SL, SLR, VAD, VSU, VSL **wrap** modulo the word and set `ip->ovfind`,
exactly as the AGC does; they do not saturate. Only DDV/BDDV, V/SC and UNIT
produce DPPOSMAX on overflow. Code that tests BOV must call
`agc_interp_test_ovf`, which clears the flag, mirroring the AGC. SQRT of a
negative argument and ASIN/ACOS out of range are reported by the `*_chk`
variants as status codes; the caller raises POODOO 01302 / ALARM 01301.
See docs/modules/core-numeric.md for the per-op table.

## 3. Basic (machine) code

Translate meaning, not instructions. `CCS X` followed by four branches is a
sign test. `INDEX`/table lookups are array indexing with a bounds assertion.
`INHINT`/`RELINT` become `agc_inhint()` / `agc_relint()`. Bank switching
(BANK, EBANK=, SETLOC, 2CADR, BBCON, superbank) is dropped; there is one flat
address space.

Shared erasables: use the accessors in `include/agc/core/state.h` (or the
owning subsystem's public header). Module-private erasables: `static`
variables in the module's `.c` file, named after the AGC label (lower
case), with the AGC scale in a comment. One AGC erasable is one C object,
whatever names the listing gives it (section 8).

Flags: the AGC flagwords FLAGWRD0..FLAGWRD11 (CSM) / ..FLGWRD13 (LM) are
bit-named in `include/agc/core/flags_common.h`, `flags_csm.h`,
`flags_lm.h`. Use `agc_flag_test(AGC_FLAG_xxx)`.

## 4. Executive and waitlist (include/agc/exec/exec.h)

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

## 5. I/O channels (include/agc/core/channels.h)

`READ`/`WRITE`/`ROR`/`WAND`/etc. become `agc_chan_read(n)` /
`agc_chan_write(n, v)` / `agc_chan_or` / `agc_chan_and` on the 15-bit channel
image kept in core. Bit names are in `include/agc/core/channels.h` (from the LM
INPUT_OUTPUT_CHANNEL_BIT_DESCRIPTIONS and the CM erasable comments).

## 6. DSKY / pinball (include/agc/io/dsky.h)

Verb/noun tables stay tables (static const arrays). The display registers
R1-R3, VERB, NOUN, PROG are a struct. Noun scaling routines translate to
functions that convert between DP erasable and 5-digit decimal/octal.

## 7. What to keep from the AGC text

Keep: labels (as comments and as function names), page numbers, the author's
functional descriptions, scale factors, alarm codes, iteration bounds,
constants with their decimal and octal forms. Drop: bank bookkeeping,
COUNT/SETLOC/BLOCK pseudo-ops, address arithmetic.

## 8. Rules the mission taught (NASSP, 2026-10-13/14)

Flying Apollo 11 in NASSP on this code (docs/modules/nassp-integration.md,
"Flight bugs found by the mission") found seventeen faults that the unit
and oracle tests had passed. Each rule below names the bug that taught it
(bug numbers are that table's); each fix came with a test that failed
before it.

* **An EQUALS / `=` alias is one C variable.** When ERASABLE_ASSIGNMENTS
  says `QAXIS EQUALS UHYP`, `VGDISP = DELVSAB`, `TGO EQUALS DVCNTR +1`, the
  two names are one word of memory: a write under one name is a read
  under the other. Give the erasable one owner and one C object, name the
  alias in the declaration's comment, and make every reader and writer
  use it. Two copies stay apart silently, and each lane's tests pass.
  Bug 3 (DOI: P30 N42 DV REQ 0; the LM wrote DELVSAB, N42 read VGDISP),
  bug 4 (PDI: alarm 01406; TLAND = W loaded by address never reached
  P63's own copy), bug 9 (liftoff overshoot; QAXIS = UHYP and ZAXIS1 =
  UHZP kept apart, so the ascent's downrange axis never moved), bug 10
  (P12 N85 00000; VGBODY written to one copy, shown from the other), bug
  11 (P12 TIG from a DPS burn's TGO; TGO and DVCNTR +1 apart). Bug 14 is
  the restore side of the same rule: UHYP (= QAXIS), /LAND/ and the
  average-G start state had to be mapped at their one address. Check:
  grep the listing's ERASABLE_ASSIGNMENTS for EQUALS / `=` on any
  erasable you translate.
* **A -0 is data, not an error.** The AGC reads a -0 word as a value
  (CCS takes it as the -0 branch; a -0 high word is a small negative
  number with a zero major part). Never assert on a sign, a range or a
  zero the listing accepts, and never round-trip a word through a
  conversion that cannot hold -0. Bug 7 (PDI: spurious 00401; FINDCDUW's
  MGA limit converted a two's -1 to ones' -0 and back to +0), bug 8 (after
  touchdown: SPEEDRN2 BAD_ARG every 0.5 s; an assertion refused the -0
  high word ALTOUT takes as new data), and bug 2 (TEI: DSKY blank after
  V82; an assertion of `-TPER <= 0` where SKIPTPER stores -TPER of either
  sign).
* **A sign is taken from the major word.** SIGNMPAC, CCS, BZMF, BPL and
  the sign tests of the RTB ops look at the high word of MPAC, not at the
  DP value: after an overflow the major word can be +0 while the minor
  words are negative. Write such a test on the words (`agc_dp_words_t`).
  Bug 15 (rendezvous P20: the LM swung 40-60 deg in roll; BOVB SIGNMPAC in
  COMPMATX took the sign of the DP value, so DELCOMP's diagonal became -1
  and VECPNT1's outer gimbal flipped 180 deg).
* **Crew loads reach their erasable.** Every noun component the DSKY can
  display and the listing lets the crew load (V24/V25) must be stored
  into the same erasable by the noun's write path; a component readable
  but not writable is a load silently dropped. Bug 16 (rendezvous P32:
  alarms 00603/00604, no CSI; the crew's N55 R1 never reached NN, which
  kept the scenario's 7512).
* **Constants come from the value the listing pushed.** An interpretive
  STODL or STCALL with no address loads the push-down list, so the value
  is the one pushed earlier in the routine, not the variable named in
  the comment beside it. Follow the push-down list through the routine
  before writing the expression. Bug 17 (TPI: ABORT 01302 at P41; INITVELX's
  `STODL MU/A / SR 6 / STORE MUASTEER` was read as MU/A SR 6 where it
  pops the pushed MU: MUASTEER = MU SR 6).
* **Erasable restore goes through the address map.** A host that loads
  or saves the AGC as memory (NASSP scenarios, V71/V72, the downlink
  dump) reaches erasable only through `src/erasable_map/address_map.c`.
  An erasable a program keeps across a restart, or a loaded program
  reads later, must have its ECADR in the map with its one owner, or the
  load drops it. Bug 1 (MCC-2: the map put EK3VAL over FANG), bug 5 (PDI:
  the load restart re-read RADMODES bit 6 from an unpowered LR), bug 12
  (the CSM RCS DAP state unmapped: the CMC never held the CSM after a
  load), bug 13 (UNITW unmapped, vector stores misplaced: splashdown 2.6
  deg short), bug 14 (the average-G start state, MUNFLAG's CSM vectors,
  /LAND/ unmapped: the resumed P12 never reached orbit).
* **Test at the mission level too.** Unit and oracle tests prove a
  routine against the listing on the inputs someone chose; only a closed
  loop exercises the paths a flight takes (overflow to exactly 1, a -0
  altitude at touchdown, a crew load, a scenario restore). The tiers are
  `make test`, `make scenarios`, `make lm-sim`, `make nassp-replay` and
  the NASSP mission tables (docs/ARCHITECTURE.md, "Verification tiers");
  every flight bug is reproduced offline (the replay harness or a saved
  state on the debug library) and fixed with a test that fails first.

