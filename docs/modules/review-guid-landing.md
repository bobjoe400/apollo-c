# review-guid-landing -- review of the guid-landing lane

Scope: src/lm/landing/{throttle,guidance,lunar_landing,aborts,displays,findcduw,state}.c,
src/lm/ascent.c, include/agc/lm/landing.h, tests/lm/landing.c,
tests/oracle/landing.c. AGC: Luminary099 pp. 785-856, 898-925.

## Progress

(resumed 2026-10-10: no earlier note on disk; the previous session's ROOTPSRS/TDISPSET work was in commit 4f35a43)

- DONE THROTTLE pp. 793-797 line by line: throttle.c rewritten (word pairs agc_lm_words_t, ones'-complement clock differences, MASSMULT overflow L = 02410, prologs); oracle throttle KNOWN F9 x12 flipped to hard checks
- DONE ROOTPSRS/TDISPSET F9: ttf_8_hi tracking + word-level DMPSUB/POWRSERS (guidance.c ldg_dmpsub, ldg_powrsers), DXCRIT from guess words, value stop test; TDISPSET TREDES idiom verified; oracle landing 172 checks, 0 failures, 0 KNOWN
- DONE state.c: agc_lm_words / agc_lm_words_value / agc_lm_das / agc_lm_das_ovf added (declared in include/agc/lm/landing.h); prologs; 0 style findings
- DONE aborts.c vs pp. 829-836 (R10,R11, R10,R11A, P70/P71, ABRTJASK, GOABORT, P70INIT, BOTHPOLY, UPTHROT, P71RET, OLDTIME, LEGAL?, TGOCOMP, THROTUP): confirmed; PIPCTR assertion; ternaries removed; 0 style findings
- DONE LANDING_TEST_FAKE_THRUST: gone from code; guid-landing.md Tests/Open questions/Requested core additions brought up to date
- DONE R10/R11 naming conforms to NAMING.md (abort_monitor / abort_monitor_resume / LANDISP); headers reworded
- DONE digit-suffixed names: 7 job/task ids + 16 functions renamed (docs/RENAMES.md "review-guid-landing"); registry.tsv, include/agc/exec/jobs.h, src/exec/dispatch.c, src/lm/burns/dps_burn.c updated by word-boundary replace; gen_registry --check clean
- DONE tests/lm/landing.c (TTHROT as a valid TIME1 word; word-pair fields), tests/oracle/landing.c (word-pair inputs/compares, F9 KNOWNs now hard + high-word checks)
- DONE DMP vs DMPR / shift audit (review_items "ALL remaining lanes"), all lane files vs LUNAR_LANDING_GUIDANCE_EQUATIONS, THE_LUNAR_LANDING, ASCENT_GUIDANCE, P12, P70-P71, FINDCDUW, THROTTLE, LANDING_ANALOG_DISPLAYS: no DMP/DMPR mislabels (DMPR only in DDUMCALC, as the AGC); SR/SRR/VSR roundings match. 15 sites changed where the AGC shifts/square-roots the TP MPAC (DOT SL, DMP [DAD] SL, DSQ [DSU] SQRT, SQRT SL1) or reads a sign-disagreeing word pair (FWEIGHT DMP BIT1H): lunar_landing.c 3, ascent.c 10, guidance.c 1 (AFCCALC3), findcduw.c 1 (NB2CDUSP). Scratch scanner: review-guid-landing/tpscan.py
- DONE oracle: new cases ascent_a/b (ATMAG->ASCENT->P12RET, stop PHASCHNG) and rodcomp_a/b (RODCOMP to THROTTLE +3) in gen_landing.py/landing.spec/.txt + tests/oracle/landing.c; findcduw tol 1 -> 0 (CPHI compared as raw two's-complement words; the 1 LSB was the test reading them as ones' complement). oracle_landing 270 checks, 0 failures. RODCOMP exact; ascent exact through TGO, rest within documented F9-class tolerances (sign-disagreeing TBUP/TGO; proven by running the rope from T2TEST on canonical words = C exactly)
- PARTIAL style debt: 0 findings in state.c, throttle.c, aborts.c. TODO displays.c (96; was reading it vs pp. 898-907 -- no edits made yet, file unchanged), findcduw.c (174), guidance.c (~205), lunar_landing.c (~235), ascent.c (~201), include/agc/lm/landing.h (~55), tests (76)
- TODO rule-5 density warnings (aborts 1.0, displays 1.1, findcduw 1.9, lunar_landing 1.7, state 1.4, throttle 1.6): split combined asserts onto separate lines, add real pre/postconditions while doing the style pass
- TODO fidelity pass of lunar_landing.c (R13, P66 RODCOMP, REDESIG), ascent.c, findcduw.c, displays.c beyond the oracle coverage
- Build state: every touched file compiles (private full build: scratch review-guid-landing/full.sh; `make` in the shared tree is currently broken by other lanes' mid-edit files, e.g. src/csm/prelaunch_*.c); oracle_landing 172/0/0, test_landing 17/0
- Scratch tools (review-guid-landing/): prolog.py, annot.py, braces.py, longfix.py, fileprolog.py, rep.py, full.sh

## Findings

| # | where | AGC | finding | action |
|---|---|---|---|---|
| 1 | throttle.c DOPIF/DOIT/FWEIGHT | THROTTLE p. 795 | F9: PIF = FCODD - FP by DAS left sign-disagreeing; CA PIF / MP PIF read the high word alone (THRUST 1 pulse, FWEIGHT up to 1.4 pulses) | FP, FCODD, PIF, FWEIGHT kept as word pairs (agc_lm_words_t, agc_lm_das); oracle KNOWN F9 x12 flipped to hard checks (+ high-word checks) |
| 2 | throttle.c AFDUMP | p. 793 | FP after DAS FWEIGHT is sign-disagreeing too; region logic (AD FP, CA FP) read the value's high word | uses fp.hi |
| 3 | throttle.c time tests | pp. 793, 795 | TIME1 - TTHROT and TIME1 - PIPTIME+1 + THROTLAG taken in two's complement; the AGC's CS/AD/MASK gives -0 (all ones) for equal times and one less after a TIME1 wrap | thr_csad models the 16-bit ones'-complement A register |
| 4 | throttle.c MASSMULT | p. 796 | overflow low word "whatever L held" is in fact TPAGREE's return address 02410 (probe-verified) | exact constant THR_MASSMULT_L |
| 5 | throttle.c FLATOUT1/LOWFCOLD | p. 794 | FCODD low word = L = BUF+1 (H*GHCR*T = BUF+1 shares scratch BUF with ROOTPSRS etc.); not determinable in the lane model | DEVIATION kept (0), reworded; affects PIF low word only |
| 6 | guidance.c ROOTPSRS | pp. 823-825 | POWRSERS ran on the canonical value of ROOTPS; the AGC multiplies the stored (sign-disagreeing) words: TTF/8 202 LSB in guid_brak_b | word-level DMPSUB/POWRSERS (ldg_dmpsub, ldg_powrsers); exact |
| 7 | guidance.c ROOTPSRS stop test | p. 825 | previous session compared high words; AGC CCS/TESTLODX on ABS(DX)-DXCRIT is a value test | value test restored |
| 8 | guidance.c QUADGUID | pp. 810-811 | UNFC/2 <= 60, /AFC/ <= 81 LSB: stale push-list low words | documented DEVIATION kept (oracle tol 128) |
| 9 | aborts.c R10,R11 | p. 829 | CCS PIPCTR < 0 path stores |PIPCTR|-1 (C went to LRHTASK); PIPCTR never negative | assertion added |
| 10 | throttle.c | p. 796 | MASSMULT, WHATOUT only called inside THROTTLE_CONTROL_ROUTINES | made static (thr_massmult, thr_whatout), removed from landing.h |

## Lessons
