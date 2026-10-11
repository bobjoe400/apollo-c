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
| fidelity pass per AGC file | IN PROGRESS (order: R63, R47, LEM_GEOMETRY, CSM_GEOMETRY, S-band, P51-P53, SXTMARK, AOTMARK) |
| style findings to zero | TODO |
| rule-5 density | TODO |

## Findings

| id | where | AGC | finding | fix |
|---|---|---|---|---|
| A1 | src/lm/rendezvous_final_attitude.c | R63 V89RECL p.339 `MXV RTB REFSMMAT NORMUNIT` | C called UNIT, not NORMUNIT: 15.7 mrad LOS error at 1 km, 1.8 rad at 100 m | agc_rtb_normunit; test at 300 m..50 km within 1 m/range |
| A2 | src/exec/nouns.c (lane data) | N70/N71, ERASABLE_ASSIGNMENTS (AOTCODE = STARCODE; STARALGN follows) | STARCODE/AOTCODE were read-only (V21N70/V21N71 star-code loads returned NOT_FOUND on both vehicles); LM AOTCODE +1/+2 had no owner | read/write STARCODE; +1/+2 = SINCDUY high/low (guidance block) |
