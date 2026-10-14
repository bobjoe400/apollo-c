# core-numeric -- fixed-point word model and interpretive arithmetic

History of this lane: [docs/history/lanes/core-numeric.md](../history/lanes/core-numeric.md)

## Scope

include/agc/core/word.h and include/agc/core/interp.h implementations, plus
include/agc/core/constants.h + src/core/constants.c.
AGC sources: SINGLE_PRECISION_SUBROUTINES.agc (p.1207), INTERPRETER.agc
(Comanche055 pp.1131-1191, the arithmetic sections), FIXED_FIXED_CONSTANT_POOL.agc,
INTERPRETIVE_CONSTANTS.agc. Comanche055 and Luminary099 copies are identical
apart from whitespace, SQRTABRT (LM: DXCH LOC / POODOO1), ACOSABRT (LM:
DCA LOC / ALARM1), 5B10, LM-only OCT33/DEC27/OCT35/DEC29 and DEC-6 -> DFC-6.

## Design

Approach: a private word-level MPAC emulation (src/core/agc_mpac.[ch]):
three ones'-complement words held as int32 in (-2^14, 2^14), signs may
disagree exactly as on the AGC. MP, DAS, DV, OVERFLOW, ROUNDSUB, DMPSUB,
DOTSUB, DSQSUB, VSQSUB, POLY, SQRTSUB, GENDDV are reproduced instruction by
instruction, so results are bit-exact with the AGC for any input whose words
agree in sign (all agc_dp_t inputs do: a DP int32 always splits into
sign-agreeing words). -0 is never produced as a value; where the AGC's
branching depends on a -0 (x + (-x) = -0 before CCS/BZMF, MAXDVSW, GENSCR
count 14) the observable branch is reproduced. Files: agc_mpac.[ch]
(instructions + DMPSUB/ROUNDSUB/DOTSUB/DSQSUB/VSQSUB/POLY), agc_shift.c
(GENSCR/MPACSHR/MPACSRND/VSHRRND/VRIGHT2/TSSL/TSLC), agc_divide.c
(DDV/GENDDV/MAXDV/SQRTSUB), agc_word.c (agc_word.h), agc_sp_trig.c,
interp/scalar.c, interp/trig.c, interp/vector.c, agc_vec_div.c
(V/SC, UNIT), agc_constants.c.

| AGC routine (label, page) | C function | file |
|---|---|---|
| SPSIN/SPCOS/POLLEY/ARG90 p.1207 | agc_sp_sin, agc_sp_cos | agc_sp_trig.c |
| DAD DSU BDSU ENDVXV OVERFLOW p.1131-1134 | agc_dp_add, agc_dp_sub (BDSU = swapped) | interp/scalar.c |
| TAD p.1135 | agc_tp_add | interp/scalar.c |
| DMPSUB DMP1 DMPR ROUNDSUB p.1136-1137,1149 | agc_dp_mul_tp, agc_dp_mul_raw, agc_dp_mul, agc_dp_mul_trunc, agc_tp_round_dp, agc_tp_round | agc_mpac.c, agc_word.c, interp/scalar.c |
| DOTSUB DOT p.1138,1143 | agc_vec_dot, agc_vec_dot_tp | agc_mpac.c, interp/vector.c |
| POLY/POWRSERS p.1139-1140 | agc_dp_poly | agc_mpac.c, interp/scalar.c |
| SHORTMP, DMPNSUB p.1141-1142 | agc_tp_mul_sp, agc_dp_mul_int | interp/scalar.c |
| MXV VXM p.1143-1144; MXM3 (ANGLFIND p.404) | agc_mat_vec, agc_vec_mat, agc_mat_mul, agc_mat_transpose | interp/vector.c |
| VXSC VVXSC DVXSC VPROJ VROUND p.1145-1146 | agc_vec_scale, agc_vec_proj | interp/vector.c |
| VXV p.1147 | agc_vec_cross | interp/vector.c |
| VAD VSU BVSU p.1131-1133 | agc_vec_add, agc_vec_sub | interp/vector.c |
| DDV BDDV DDV/BDDV GENDDV MAXDV p.1149,1162-1170 | agc_dp_div, agc_dp_div_raw | agc_divide.c, interp/scalar.c |
| V/SC V/SC2 V/SCDV p.1150,1171-1173 | agc_vec_div | agc_vec_div.c |
| SIGN COMP DCOMP VCOMP p.1151 | agc_dp_sign, agc_dp_neg, agc_vec_neg | agc_word.c, interp/vector.c |
| SR SRR SL SLR VSR VSL GENSHIFT GENSCR TSSL VSSL VRIGHT2 p.1152-1161 | agc_dp_shr, agc_dp_shr_round, agc_dp_shl, agc_dp_shl_round, agc_dp_shl_raw, agc_vec_shr, agc_vec_shl, agc_tp_shift | agc_shift.c, agc_word.c, interp/scalar.c, interp/vector.c |
| TSLC (NORM) p.1157 | agc_dp_norm, agc_tp_norm | agc_shift.c, agc_word.c, interp/scalar.c |
| UNIT UNITDV SMALL LARGE p.1175-1179 | agc_vec_unit | agc_vec_div.c |
| DSQ DSQSUB VSQ VSQSUB ABVAL ABS p.1180-1183 | agc_dp_sq, agc_dp_sq_tp, agc_vec_sq, agc_vec_sq_tp, agc_vec_abval, agc_dp_abs | agc_interp_*.c, agc_word.c |
| SQRT SQRTSUB SQRTSHFT p.1181-1186 | agc_dp_sqrt, agc_dp_sqrt_chk, agc_tp_sqrt | agc_divide.c, interp/trig.c |
| SINE COSINE PRESINE SN1 p.1187-1188 | agc_dp_sin, agc_dp_cos | interp/trig.c |
| ARCSIN ARCCOS ACOSST2 ACOS3 SUBTR ASINEX p.1189-1191 | agc_dp_asin(_chk), agc_dp_acos(_chk) | interp/trig.c |
| BOV(B) p.1196 | agc_interp_test_ovf | interp/scalar.c |
| FIXED_FIXED_CONSTANT_POOL pp.1200-1204, INTERPRETIVE_CONSTANTS pp.1205-1206 | AGC_K_* macros, agc_k_unitx/y/z, agc_k_zerovec | agc_constants.h/.c |

## Operation semantics (what other lanes rely on)

Scale notation: a DP holds v/2^28 of its stated scale 2^N.

| Op | C | Scaling | Rounding | Overflow |
|---|---|---|---|---|
| DAD/DSU/BDSU | agc_dp_add/sub | a, b same scale | exact | wraps (value -+1.0, overflow-corrected), OVFIND = +-1 (sign of the overflow); NOT saturating |
| TAD | agc_tp_add | TP | exact | wraps, OVFIND = +-1 |
| DMPR | agc_dp_mul | scales add | DMPSUB then round half away from zero (MPAC+2 >= 1/2 LSB) | none possible except carry at ~1.0 (OVFIND) |
| DMP | agc_dp_mul_trunc / agc_dp_mul_tp | scales add | top two words of DMPSUB (minor*minor keeps only its major half: TP may differ from the exact product by < 2^-42) | none |
| DSQ | agc_dp_sq / _tp | 2x scale | as DMP (truncated) | none |
| DDV/BDDV | agc_dp_div(ip, dividend, divisor) | quotient scale = dividend scale - divisor scale | GENDDV first-order approximation, low 2 bits per the AGC | \|a\| >= \|b\| or b = 0: +-DPPOSMAX (sign of quotient; 0/0 gives +), OVFIND = +1 always |
| SR n | agc_dp_shr | scale +n | truncates toward zero; n >= 14 moves words first | none |
| SRR n | agc_dp_shr_round | scale +n | half away from zero | none |
| SL n / SLR n | agc_dp_shl / _round | scale -n | exact | each overflowing step wraps and sets OVFIND = +-1 |
| VSL n | agc_vec_shl | scale -n | exact | per component per step: DAS + OVERFLOW routine, wraps, OVFIND |
| VSR n | agc_vec_shr | scale +n | ALWAYS rounds (VSHRRND); n > 14: rounds at the 14-word step and again at the end, as the AGC ("one bit error in a shift right 15") | none |
| NORM | agc_dp_norm / agc_tp_norm | left-justify to \|major\| >= 1/2 | exact | none; shift count returned positive (AGC stores its complement) |
| ROUND | agc_tp_round | TP -> DP | half away from zero | carry out -> OVFIND |
| SQRT | agc_dp_sqrt(_chk), agc_tp_sqrt | same binary point: arg 2^2n -> root 2^n | Newton x2 from linear guess; ~7.5 LSB of 2^-28 worst | a < -2^-14: POODOO 01302 (agc_dp_sqrt_chk -> AGC_STATUS_POODOO, value 0); -2^-14 <= a < 0 -> 0 |
| SIN/COS | agc_dp_sin/cos | arg revolutions at 2^0 (any value); result sin(2 pi x)/2 at 2^1 | Hastings 4th order, \|err\| <= 2.1e-8 | none |
| ASIN/ACOS | agc_dp_asin/acos(_chk) | arg sin/2 at 2^1 (\|a\| <= 1/2); result revolutions at 2^0 (ACOS [0,.5], ASIN [-.25,.25]) | Hastings 7th order x sqrt, \|err\| <= 1.1e-8 | major word > HALF+1: result 0 (ACOS) / .25-0 (ASIN), negated per sign, ALARM 01301 (AGC_STATUS_ALARM from _chk); major = HALF+1 or HALF with minor >= 0: result 0 without alarm |
| VAD/VSU/BVSU | agc_vec_add/sub | same scale | exact | per component wrap; OVFIND set in AGC order y, z, x |
| DOT/VSQ | agc_vec_dot, agc_vec_sq (+ _tp) | scales add | truncated top words of the TP | wraps, OVFIND = +-1 |
| VXV | agc_vec_cross(u, v) = u x v | scales add | each partial product truncated | per component wrap, OVFIND |
| VXSC | agc_vec_scale | scales add | each component rounded (VROUND) | carry -> OVFIND |
| VPROJ | agc_vec_proj(u, v) = (u.v) v | u.v truncated, then each component rounded | | OVFIND from DOT/VROUND |
| V/SC | agc_vec_div(v, s) | quotient scale = v scale - s scale | per component GENDDV | \|v_i\| >= \|s\| or s = 0: OVFIND = 1, the slot being divided gets +-DPPOSMAX and the AGC aborts the instruction: result (sat_i, x', z), (sat_z, x', y') etc. exactly as left in MPAC/MPAC+3/MPAC+5 |
| UNIT | agc_vec_unit(v, &lv, &lvsq) | result v/(2\|v\|) at 2^1 (half-unit); LV = \|v\| scaled as v; LVSQUARE = \|v\|^2 at 2x scale | ~6.6 LSB | \|v\|^2 >= 1 or v = 0 (or so small that VSQSUB is 0): OVFIND = 1, result (DPPOSMAX, 0, k*2^-14) as the AGC leaves MPAC; LV/LVSQUARE not written on \|v\|^2 overflow (LV = 0 for zero) |
| MXV/VXM | agc_mat_vec(M, v) = M v, agc_vec_mat(v, M) = v^T M | scales add | DOT truncation per component | OVFIND from DOTSUB |
| MXM3 | agc_mat_mul(A, B) = A B | scales add | MXV per column | OVFIND |
| SPSIN/SPCOS | agc_sp_sin/cos | arg at pi (half-revolutions), result at 1 | \|err\| <= 2.6e-4 (4.2 LSB) | +-1 -> +-POSMAX |

Raw agc_word.h operations (agc_sp_add/sub, agc_dp_add_raw/sub_raw,
agc_dp_shl_raw) saturate at +-POSMAX and report overflow through *ovf, as
the agc_word.h contract says; they are utilities, not AGC instructions.
agc_dp_mul_raw = DMPR, agc_dp_div_raw = DDV (OVF = +1 on divide overflow).
agc_dp_round rounds a DP to SP precision (half away from zero), it is not
the interpretive ROUND (that is agc_tp_round). agc_dp_pack accepts words
of either sign. Use agc_dp_from_double/to_double only in tests and the
sim; they round half away from zero and clamp.

## Oracle (yaAGC) verification

Labels exercised through INTPRET (snippet in VAC2USE, operands in VAC1, see
tools/oracle/cases/core_numeric.spec): DMP DMPR DDV BDDV DAD DSU SIGN DSQ
SQRT SIN COS ASIN ACOS ABS DCOMP ROUND(TLOAD) SQRT(TLOAD) TAD NORM SR SRR
SL UNIT ABVAL VSQ DOT VXV VPROJ VAD VSU BVSU VXSC V/SC MXV VXM VSL VSR,
which reach DMPSUB ROUNDSUB DOTSUB DSQSUB VSQSUB POLY SQRTSUB GENDDV MAXDV
GENSCR MPACSHR MPACSRND VSHRRND VRIGHT2 TSSL TSLC SINE COSINE SN1 ARCSIN
ARCCOS UNITDV SMALL LARGE V/SCDV OVERFLOW. Basic: SPSIN, SPCOS (entry
SPSIN/SPCOS, q/stop ENDOFJOB, A in/out). Stops: ENDOFJOB, POODOO.
Outputs: MPAC..MPAC+6, OVFIND, VAC1+30 (NORM X), VAC1+34 LVSQUARE,
VAC1+36 LV, FAILREG.

Results (scratch harness, not committed): 20,720 random + edge-case
interpretive runs (all counts 1-45 for SR/SRR/VSR, near-equal DDV/MAXDV,
tiny vectors for UNIT SMALL, ACOS at +-1/2 +- 1 LSB) bit-identical to yaAGC
with its DV bug (below) patched; exhaustive SPSIN and SPCOS (all 32,767
inputs each) bit-identical to stock yaAGC. Committed: 287 cases that stock
yaAGC computes correctly (tests/oracle/core_numeric.c, 0 mismatches).

## Deviations

* DEVIATION: V/SC and UNIT reach DVOVF without setting DVSIGN in some paths
  (zero scalar major word, |v|^2 overflow, zero vector); the AGC then uses
  a stale DVSIGN from the previous division. C returns +DPPOSMAX there.
* agc_vec_unit leaves *abval/*vsq unchanged on |v|^2 overflow (the AGC
  does not store LV/LVSQUARE then) -- initialise them if you read them.
* ALARM 01301 (ACOS/ASIN) and POODOO 01302 (SQRT) are not raised by the
  core (it does not call the executive); the _chk variants return
  AGC_STATUS_ALARM / AGC_STATUS_POODOO and the caller raises them. The
  plain functions return the value the AGC leaves (0 after the abort).
* Input word pairs whose signs disagree are not representable in agc_dp_t
  (they become the sign-agreeing pair of the same value). Where an AGC
  caller stored such a pair (e.g. a DAD result) and later multiplies it,
  the minor x minor truncation can differ in the last bit.
* LM differences (SQRTABRT via POODOO1, ACOSABRT via ALARM1 with LOC) only
  affect the alarm bookkeeping, not the values.

## Stubs

none

## Tests written

* tests/core/numeric.c: pack/unpack, saturation and *ovf of the raw
  ops, wrap + OVFIND of DAD/DSU/SL/VSL, BOV clearing, DMP/DMPR rounding
  (half away from zero), SHORTMP/DMPNSUB/ROUND, DDV incl. overflow, 0/x,
  x/0, MAXDV, SQRT/SIN/COS against yaAGC words, SQRT abort, ACOS/ASIN alarm,
  SIN/COS/ASIN/ACOS/SQRT/SPSIN/SPCOS vs libm (bounds above), UNIT (+LV,
  LVSQUARE, zero vector), DOT, VXV, VSQ, ABVAL, MXV, VXM, MXM3, VXSC, V/SC
  (+overflow slot), VPROJ, VAD/VSU, constants.
* tests/oracle/core_numeric.c: 287 yaAGC cases, bit-exact.

## Program flow and decisions

_Documentation wave (2026-10-12); prose and tables brought to the final tree (HEAD 3b97dc8) by the final pass (2026-10-14). The prose was written by reading the code; the two tables were extracted from the function bodies (flag, alarm, channel and scheduling calls; `agc_erase_*` accessors) and list only what the code does. Files: `src/core/divide.c`, `src/core/mpac.c`, `src/core/shift.c`, `src/core/word.c`, `src/core/sp_trig.c`, `src/core/constants.c`, `src/core/vec_div.c`, `src/core/interp/*.c`._

This lane has no scheduling, flags or alarms of its own: it is the AGC's
arithmetic, and its decisions are the branches inside the interpreter
routines it models. Values are signed fractions (SP v/2^14, DP v/2^28,
TP v/2^42; `src/core/word.c`), and the word-level API keeps the AGC's
two or three ones'-complement words with signs free to disagree
(`src/core/mpac.c`, `src/core/interp/words.c`, `vector_words.c`).

* **Overflow is data, not failure.** Every add, shift and divide reports
  overflow the way the AGC does: the wrapped (overflow-corrected) result
  plus OVFIND = +-1 in `agc_interp_t` (`mpac.c:agc_w_overflow`,
  `agc_w_dad`; `shift.c:agc_w_tssl` sets OVFIND per overflowing step).
  OVFIND is only cleared by BOV (`interp/scalar.c:agc_interp_test_ovf`,
  `agc_interp_clear_ovf`); callers branch on it as the AGC branches on BOV.
  The raw utilities of `word.c` (`agc_dp_add_raw`, `agc_dp_shl_raw`,
  `saturate32`) saturate instead and return the overflow through `*ovf`.
* **Divide** (`divide.c:agc_w_ddv`): a zero major divisor word goes to
  BUFZERO (`ddv_bufzero`: overflow unless the dividend's major word is 0,
  then a 14-place shift); signs are forced positive and DVSIGN remembered;
  `ddv_normalise` doubles both until the divisor's major word is >= HALF;
  MAXTEST picks GENDDV or MAXDV (`agc_w_genddv`); |a| >= |b| exits through
  DVOVF (`ddv_overflow`: +-DPPOSMAX, OVFIND = 1). V/SC and UNIT use the same
  kernel per component (`vec_div.c:agc_vec_div`, `unit_core`), and their
  early DVOVF exits take DVSIGN from the previous divide (`dvsign_neg`
  carried in `agc_interp_t`).
* **Square root and inverse trig** (`interp/trig.c`): SQRT of a < -2^-14
  is ABORT 01302 and ASIN/ACOS of |a| more than one LSB above 1/2 is
  ALARM 01301 (result 0 / .25). The `_chk` variants (`agc_dp_sqrt_chk`,
  `agc_dp_acos_chk`, `agc_dp_asin_chk`) return POODOO / ALARM to the
  caller; the plain ones latch the code (`interp_flag_alarm`, abort wins
  over alarm) and the executive raises it when the job or task returns
  (`src/exec/alarm.c:exec_service_interp_alarm` reads
  `agc_interp_take_alarm`). `divide.c:sqrt_positive` chooses ARGHI or
  ARGLO by normalising two places at a time, then `sqrt_newton`.
* **Sine** (`sp_trig.c:sp_sine`, `interp/trig.c:sine_words`): arguments
  beyond +-pi/2 are reflected using the overflow of DOUBLE; +-1 exactly
  goes to ARG90 (+-POSMAX). The reflected words disagree in sign and the
  polynomial sees them exactly as the AGC does.
* **Rounding and normalising**: ROUNDSUB rounds half away from zero on
  MPAC+2's sign (`mpac.c:agc_w_roundsub`); NORM shifts until the major word
  is >= .5 and returns the count (`shift.c:agc_w_tslc`,
  `word.c:agc_dp_norm_count`); right shifts truncate word by word unless the
  rounding form is used (`shift.c:agc_w_mpacshr`, `agc_w_genscr`).

Data: only the interpreter state `agc_interp_t` (include/agc/core/interp.h:
OVFIND, DVSIGN, index registers X1 X2 S1 S2) and the constant pool
(`constants.c`: UNITX/Y/Z half-unit vectors scaled 2^1, ZEROVECS). Every
routine is checked bit for bit against yaAGC (tests/oracle/core_numeric.c,
core_words.c).

### Decision points by function

None of the lane's 198 functions tests a flag, raises an alarm or starts a job or task; its decisions are the numeric branches described above.

### Shared erasables read and written

The lane calls no `agc_erase_*` accessor; its data are its arguments and its own static storage (scales on the declarations).

### AGC labels -> C functions

The "AGC" column above names the label each listed function translates; the complete label -> function -> file:line index for the lane (and the tree) is [docs/overview/xref.md](../overview/xref.md), generated by `tools/xref.py`.
