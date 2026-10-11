# review-guid-landing -- review of the guid-landing lane

Scope: src/lm/landing/{throttle,guidance,lunar_landing,aborts,displays,findcduw,state}.c,
src/lm/ascent.c, include/agc/lm/landing.h, tests/lm/landing.c,
tests/oracle/landing.c. AGC: Luminary099 pp. 785-856, 898-925.

## Progress

- (resumed 2026-10-10: no earlier note on disk; restarted at THROTTLE)
- DONE THROTTLE pp. 793-797 line by line (throttle.c rewritten: word pairs, ones'-complement clock differences, MASSMULT overflow L, prologs); oracle throttle 0 KNOWN
- DONE ROOTPSRS/TDISPSET F9: ttf_8_hi tracking (previous session, 4f35a43) + word-level DMPSUB/POWRSERS on the stored ROOTPS words, DXCRIT from the guess words, value stop test restored; oracle landing 172 checks, 0 failures, 0 KNOWN
- DONE R10,R11 / R10,R11A / P70 / P71 / ABRTJASK / GOABORT / P70INIT / BOTHPOLY / UPTHROT / P71RET / OLDTIME / LEGAL? / TGOCOMP / THROTUP vs pp. 829-836: confirmed (see findings)
- DONE LANDING_TEST_FAKE_THRUST: gone from code; stale text in guid-landing.md fixed
- DONE R10/R11 naming: conforms to NAMING.md rows R10/R11 (task = abort_monitor, R10,R11A = abort_monitor_resume, LANDISP keeps its label); headers reworded
- DONE digit-suffixed names: 7 job/task ids + 16 functions renamed (docs/RENAMES.md section review-guid-landing); registry.tsv rows, jobs.h, dispatch.c, dps_burn.c callers updated by word-boundary replace; gen_registry --check clean
- TODO LANDING_TEST_FAKE_THRUST
- TODO R10/R11 abort-monitor naming
- TODO style debt (check_style)

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
