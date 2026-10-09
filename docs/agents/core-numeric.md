# Assignment: core-numeric   (tier: translation)

Implement the numeric core declared in include/agc/agc_word.h and
include/agc/agc_interp.h, in src/core/agc_word.c, src/core/agc_sp_trig.c,
src/core/agc_interp_scalar.c, src/core/agc_interp_vector.c (split further as
needed; every function <= 60 lines). Also create src/core/agc_constants.c
and include/agc/agc_constants.h (you MAY create that header) from
FIXED_FIXED_CONSTANT_POOL.agc and INTERPRETIVE_CONSTANTS.agc, keeping the
AGC names and octal values as comments.

AGC sources: SINGLE_PRECISION_SUBROUTINES.agc (SPSIN/SPCOS polynomial) and
the arithmetic sections of INTERPRETER.agc: DAD DSU BDSU DMP DMPR DDV BDDV
DSQ SQRT SIN COS ASIN ACOS ROUND NORM SIGN ABS DCOMP SL SR SLR SRR VSL VSR
VXSC V/SC VAD VSU BVSU DOT VXV VPROJ UNIT ABVAL VSQ MXV VXM TAD, and the
overflow handling (OVFIND, DPPOSMAX saturation, DDV on divide overflow).
Verify the exact scaling conventions of SIN/COS/ASIN/ACOS/SQRT and write
them into the header comments of agc_interp.h (the ONE header you may edit:
comments, plus any missing prototypes you need).

Reproduce the AGC algorithms (polynomial coefficients C1/2 C3/2 C5/2, the
SQRT iteration with NORM pre-scaling, the ARCSIN/ARCCOS method, UNIT's
normalisation) rather than calling libm, so results match the real AGC to
the bit where feasible. libm is allowed only in tests for comparison.

Tests: tests/test_core_numeric.c (plain C, returns non-zero on failure):
pack/unpack, saturation and overflow flags, DMP/DMPR rounding, DDV, SQRT,
SIN/COS vs libm within AGC precision (state the achieved error bound),
UNIT, cross/dot, MXV. Run `make lib` and `make test`; if another lane's
file breaks the build, compile and link your own objects directly instead.

Note: docs/modules/core-numeric.md must state each op's scaling and overflow
semantics precisely; other agents rely on it. Also list which labels an
oracle (yaAGC) test should hit: see tools/oracle/README.md if present.
