# core-numeric -- fixed-point word model and interpretive arithmetic

## Scope

include/agc/agc_word.h and include/agc/agc_interp.h implementations, plus
include/agc/agc_constants.h + src/core/agc_constants.c.
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
sign-agreeing words). -0 is never produced.

| AGC routine (label, page) | C function | file |
|---|---|---|
| SPSIN/SPCOS/POLLEY/ARG90 p.1207 | agc_sp_sin, agc_sp_cos | agc_sp_trig.c |
| DAD DSU BDSU / OVERFLOW p.1131-1134 | agc_dp_add, agc_dp_sub | agc_interp_scalar.c |
| TAD p.1135 | agc_tp_add | agc_interp_scalar.c |
| DMPSUB DMP1 DMPR ROUNDSUB p.1136-1137,1149 | agc_dp_mul_tp, agc_dp_mul_raw, agc_dp_mul, agc_dp_mul_trunc | agc_mpac.c, agc_word.c, agc_interp_scalar.c |
| DOTSUB p.1138 | agc_vec_dot(_tp) | agc_mpac.c, agc_interp_vector.c |
| POLY/POWRSERS p.1139-1140 | agc_dp_poly | agc_interp_scalar.c |
| SHORTMP, DMPNSUB p.1141-1142 | agc_tp_mul_sp, agc_dp_mul_int | agc_interp_scalar.c |
| MXV VXM p.1143-1144 | agc_mat_vec, agc_vec_mat | agc_interp_vector.c |
| VXSC VVXSC DVXSC VPROJ p.1145-1146 | agc_vec_scale, agc_vec_proj | agc_interp_vector.c |
| VXV p.1147 | agc_vec_cross | agc_interp_vector.c |
| DDV BDDV DDV/BDDV GENDDV MAXDV p.1149,1162-1170 | agc_dp_div, agc_dp_div_raw | agc_divide.c |
| V/SC V/SC2 V/SCDV p.1150,1171-1173 | agc_vec_div | agc_interp_vector.c |
| SIGN COMP DCOMP p.1151 | agc_dp_sign, agc_dp_neg, agc_vec_neg | agc_word.c, agc_interp_vector.c |
| SR SRR SL SLR VSR VSL GENSHIFT p.1152-1161 | agc_dp_shr, agc_dp_shr_round, agc_dp_shl, agc_dp_shl_round, agc_vec_shr, agc_vec_shl, agc_tp_shift | agc_word.c, agc_interp_scalar.c, agc_interp_vector.c |
| TSLC (NORM) p.1157 | agc_dp_norm, agc_tp_norm | agc_word.c, agc_interp_scalar.c |
| UNIT UNITDV SMALL LARGE p.1175-1179 | agc_vec_unit | agc_interp_vector.c |
| DSQ DSQSUB VSQ VSQSUB ABVAL ABS p.1180-1183 | agc_dp_sq, agc_vec_sq(_tp), agc_vec_abval, agc_dp_abs | agc_interp_*.c |
| SQRT SQRTSUB p.1181-1186 | agc_dp_sqrt(_chk), agc_tp_sqrt | agc_divide.c, agc_interp_scalar.c |
| SINE COSINE SN1 p.1187-1188 | agc_dp_sin, agc_dp_cos | agc_interp_scalar.c |
| ARCSIN ARCCOS p.1189-1191 | agc_dp_asin(_chk), agc_dp_acos(_chk) | agc_interp_scalar.c |
| BOV(B) p.1192ff | agc_interp_test_ovf | agc_interp_scalar.c |
| FIXED_FIXED_CONSTANT_POOL, INTERPRETIVE_CONSTANTS | AGC_K_* macros, agc_k_* vectors | agc_constants.h/.c |

## Progress

* DONE word-level MPAC model (MP DAS DV TS DCS OVERFLOW DMPSUB ROUNDSUB DSQSUB
  DOTSUB VSQSUB TSSL-step POLY) src/core/agc_mpac.c + agc_mpac.h (compiles)
* TODO SQRTSUB, DDV/GENDDV/MAXDV -> src/core/agc_divide.c
* TODO agc_word.c, agc_sp_trig.c, agc_interp_scalar.c, agc_interp_trig.c,
  agc_interp_vector.c, agc_constants.[ch], tests/test_core_numeric.c
* TODO oracle cases tools/oracle/cases/core_numeric.spec (yaAGC differential)
* Resume notes: interp-level DAD/DSU/SL/VAD WRAP like the AGC (OVERFLOW routine),
  raw agc_word.h ops saturate (header contract). CCS leaves DABS in A, so
  TS MAXDVSW after an equal-majors CCS stores +0 (-> MAXDV), not -0.

## Deviations

(filled in as found)

## Stubs

none planned

## Requested core additions

none (this lane owns the core numeric headers)

## Requested jobs-tasks

none

## Tests written

tests/test_core_numeric.c (planned)

## Open questions

(none yet)
