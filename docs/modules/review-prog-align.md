# review-prog-align -- review of the prog-align lane

## Scope

docs/agents/review.md for lane prog-align (docs/modules/prog-align.md):
src/programs/alignment, src/csm/alignment, src/csm/optics, src/lm/alignment,
src/lm/optics, src/lm/ags_initialization.c, src/lm/rendezvous_final_attitude.c,
include/agc/programs/alignment.h, tests/programs/align.c.

## Progress

| item | status |
|---|---|
| R63 UNIT precision (open question) | DONE: AGC uses RTB NORMUNIT; C fixed |
| N70/N71 AOTCODE orphan components | DONE |
| realistic-range tests | DONE: R63 300 m..50 km; P52 fine align with 0.5 deg drift, non-identity REFSMMAT (test_realign_realistic_drift) |
| oracle test tests/oracle/align.c (+ gen_align.py, align{,_lm}.spec/.txt) | DONE: RRNB, RRANGLES (18 cases), SCALEVEC, SXTNB, SXTANG (17), LOG bit-exact but PA-W2 |
| fidelity pass per AGC file | IN PROGRESS: R63, R47, LEM_GEOMETRY, CSM_GEOMETRY done; next S-band, P51-P53, SXTMARK, AOTMARK |
| style findings to zero | TODO |
| rule-5 density | TODO |

## Findings

| id | where | AGC | finding | fix |
|---|---|---|---|---|
| A1 | src/lm/rendezvous_final_attitude.c | R63 V89RECL p.339 `MXV RTB REFSMMAT NORMUNIT` | C called UNIT, not NORMUNIT: 15.7 mrad LOS error at 1 km, 1.8 rad at 100 m | agc_rtb_normunit; test at 300 m..50 km within 1 m/range |
| A2 | src/exec/nouns.c (lane data) | N70/N71, ERASABLE_ASSIGNMENTS (AOTCODE = STARCODE; STARALGN follows) | STARCODE/AOTCODE were read-only (V21N70/V21N71 star-code loads returned NOT_FOUND on both vehicles); LM AOTCODE +1/+2 had no owner | read/write STARCODE; +1/+2 = SINCDUY high/low (guidance block) |
| A3 | src/lm/ags_initialization.c ags_words | SCALEVEC p.209 VAD AGSRND1, VAD AGSRND2, VECSGNAG | a negative rounded word under one LSB gave +0 (AEA reads 0); the AGC's two-step add leaves -0 (77777, AEA -1): oracle scalevec_small | -0 major word when r < 0 and hi == 0; test in align.c and oracle |
| A4 | src/lm/optics/lem_geometry.c RRNB | RRNB1 `COS DMP / SL1`, `SIN DMP / SL1` | SL1 after DMP shifts the TP product (MPAC +2's top bit into the DP); C shifted the truncated DP: 1 LSB off in 4 of 6 components | agc_tp_shift(agc_dp_mul_tp) then agc_tp_to_dp: bit-exact |
| A5 | src/lm/optics/lem_geometry.c RRANGLES, src/csm/optics/geometry.c SXTANG1 | 2V1STO2S / 1STO2S of ASIN, ARCTRIG, ACOS SL2 DSU results | 1TO2SUB reads the major word after DDOUBL; ASINEX (DCS, AD QUARTER) and SUBTR leave sign-disagreeing words, so the AGC rounds such angles up (+,-) or down (-,+) where the value truncates: MODEA/MODEB/SAC/PAC 1 count off in 7 of 10 first cases | new src/programs/alignment/word_forms.c (ASIN/ACOS/ARCTRIG word forms, DAS on words, 1TO2SUB on words); 24 random oracle cases bit-exact |
| W2 | src/csm/optics/geometry.c SXTNB | `SIN SL1` (TSSL shifts MPAC +2 in) | agc_dp_sin returns two words, so sin TA is 1 LSB low half the time; 32D 2 LSB off (sxtnb_a, sxtnb_c) | KNOWN PA-W2 in tests/oracle/align.c (hard at 2 LSB); request to core-numeric: a SIN/COS that returns the TP MPAC |
| C1 | R63 rendezvous_final_attitude.c | V89CALL..ALINEZ p.338-341 | confirmed: responses, OPTIONX, DP1MIN 6000 (rope 00000 13560), CPHI = THETAD, NORMUNIT | -- |
| C2 | R47 ags_initialization.c | AGSINIT..SCALEVEC p.206-210 | confirmed: CKSTALL (wait while IMUCADR != +0), TAT for CSMPREC, constants RSCALE/VSCALE/TSCALE/20SEC/AGSRND1/2 against the rope; SCALEVEC bit-exact on the oracle | A3 |
| C3 | lem_geometry.c | RRANGLES, RRNB, RRNBMPAC p.322-324 | confirmed apart from A4/A5: ASIN(-Y), BDSU LODPHALF, ARCTRIG of the X-Z unit, mode-2 swap on ANTENBIT 04000, BOVB LUNDESCH | A4, A5 |
| C4 | geometry.c | SXTNB, SXTLOGIC, CALCSXA, SXTANG(1), ZNB=S1, LOG, EARROT1/2, OVERADAY p.285-294 | confirmed apart from A5/W2: NB1NB2 = NB2NB1 transposed (rope), 10DEGS- 3600, 20DEGS- -7199 (PAC 61740 on the oracle), CULTFLAG set/cleared as SETGO/CLRGO; LOG bit-exact (4 cases) | A5, W2 |

