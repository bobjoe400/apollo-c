# review-guid-powered -- review of the powered-flight guidance lane

NOTE (21:30): two review-guid-powered agents are running (the first was not
killed by the cutoff). Run 1 owns the items marked "run1"; a second run
should take only TODO items and update this table before starting one.

## Scope

Lane files (docs/modules/guid-powered.md, after restructure):
src/guidance/{powered_flight,servicer_common,external_delta_v,burn_steering,guid_state,guid_nav_glue}.c,
src/csm/burns/{servicer,external_delta_v}.c, src/csm/burns/sps/*.c,
src/lm/burns/servicer/*.c, src/lm/burns/dps_burn*.c, src/lm/burns/ignition/*.c,
include/agc/guidance/{guidance,pending}.h, tests/guidance/, tests/oracle/guidance.c.

## Progress

| item | status |
|---|---|
| F7 CM J2 constants (servicer_common.c) | DONE run1: words from rope 37,3464/3466 (B1 = x2); CALCGRAV DMPs truncated (agc_dp_mul_trunc); KNOWN flipped, CALCRVG cases bit-exact |
| F8 S40.2,3 UNIT(RTIG) (burn_steering.c) | DONE run1: SPS path crosses with UNIT(RTIG); DMP/SL1 on the TP (dmp_sl1); tol 128 -> 16 (residual <= 12 LSB, UNIT of a short UT x R in the radial case) |
| F9 at TDISPSET | DONE run1: TTF/8 raw high word carried through ROOTPSRS / TTFINCR DAS (ldg_das, agc_lm_ttf8_high); TTFDISP bit-exact (tol 8 -> 0); also QUADGUID, CGCALC, EXTLOGIC, HIGATCHK read it |
| DELRSPL translate | DONE run1 (src/csm/burns/external_delta_v.c, AUGEKUGL tables from rope 32,2201-2257); test pending |
| ZOOM/P40ZOOMA | run1 in progress |
| octal-written table nouns grep | run1 in progress |
| fidelity pass per file | TODO |
| style debt (check_style) | TODO |

## Findings

| # | where | AGC | finding | action |
|---|---|---|---|---|
| R1 | servicer_common.c J20_CM/J2_CM | SERVICER207 p.836 20J, 2J `2DEC* ... B1*` | constants were 1/4 of the rope (B1 read as B-1) -> CM GOBL1/2 4x small (F7) | fixed from rope words 17431770, 1743177 |
| R2 | servicer_common.c CALCGRAV | DMP DMP 20J / DMP DMP 2J 32D | rounded DMPR used for DMP -> GOBL1/2 1 LSB | agc_dp_mul_trunc |
| R3 | burn_steering.c S40.2,3 SPS | p.716 PDVL UNIT RTIG / VXV VCOMP | crossed UT with raw RTIG (F8) | UNIT(RTIG) for SPS; RCS (S40.2,3B) keeps raw RTIG as coded |
| R4 | burn_steering.c S40.2,3 bell axes | p.715 DMP SL1 | DMP rounded then shifted; AGC shifts the TP | dmp_sl1 (TP shift) |
| R5 | src/lm/landing/guidance.c TDISPSET (+ROOTPSRS) | p.825 CA TTF/8 | ROOTPSRS's DAS ROOTPS leaves TTF/8 sign-disagreeing; CA reads the raw high word (F9) | raw high word tracked (ttf_8_hi) |
| R6 | external_delta_v.c DELRSPL | pp.643-647 | stub | translated |

## Lessons
