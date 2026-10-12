# fix-b6 -- flight bug B6: the V99 / V97 engine flashes (CLOCPLAY paste)

## Scope
B6 (docs/modules/sim.md "Bugs found"): at TIG-5 the CSM CLOCKJOB passed
V99N40 as the VN of a CLOCPLAY display, so pinball executed extended verb
99 (GOLOADLV -> LOADLV -> RECALTST), which woke the display's ENDIDLE at
once with the ENTER return: V99E, SPS bypassed (P40RCS), V16N85 with the
full VG, no ignition. The V97 engine-fail flash and the LM CLOKJOB had the
same defect (V99Nxx / V97Nxx passed as the first part).

## AGC reference
* Comanche055 P40-P47.agc p.705, CKNVWRD1: `CCS NVWORD1` -> NOFLASH (+VN,
  REGODSP) / ENDOFJOB (0) / ENGREQST (-BIT9) / FAILDSP (-0). Both
  ENGREQST and FAILDSP are `CAF V06N40 / TC BANKCALL / CADR CLOCPLAY`
  followed by the TERMINATE / PROCEED / ENTER returns (V99T/P/E,
  V97T/P/E).
* Luminary099 BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc p.745, CLOKJOB
  -13 (`CS BIT9 / TS NVWORD1 / INDEX WHICH / CA 0 / CLOCPLAY`: the
  countdown V06Nxx) and -35 (`CS ZERO / TS NVWORD1 / CA NVWORD +2 /
  CLOCPLAY`).
* DISPLAY_INTERFACE_ROUTINES.agc CM p.707 CLOCPLAY (CLOCKCON: flash,
  perform, clock bit 12), CM p.1477-1478 1STOR2ND: the second part of the
  perform is `CS NVWORD1 / AD V97N00` (NVWORD1 = -0 is V97, -400 is V99),
  noun 00 = keep the noun: V06N40 is displayed, then V99 (V97) is pasted
  flashing over the verb.

## Fix
* src/csm/burns/sps/clock.c cknvwrd1(): first part agc_dsky_vn(6, 40); the
  display's NVWORD1 is set to -BIT9 (077377) / -0 (077777) just before the
  CLOCPLAY. The display interface already modelled the paste
  (src/io/dsky/display_if.c dif_second_vn, DIF_CLOCK); nothing new there.
* src/lm/burns/ignition/clock.c: -13 and -35 pass agc_dsky_vn(6, noun)
  (the countdown noun) instead of V99/V97 (NVWORD1 was already set there).

## Deviations
* The display interface keeps its own NVWORD1 (agc_dsky_set_nvword1); the
  CSM guidance word csm.nvword1 holds -1 for the AGC's -0. CKNVWRD1
  copies the flash code into the display's word at each CLOCPLAY, as the
  LM CLOKJOB does (comment in the clock.c prolog).

## Stubs / Requested core additions / Requested jobs-tasks
None.

## Tests written
tests/guidance/guidance.c test 5 (fails 7 checks on the old code):
* test_engine_enable_request_csm: NVWORD1 = -BIT9, CLOCKJOB -> V99N40
  flashing, one job asleep in ENDIDLE, ASTNFLAG clear; PRO -> V99P:
  ASTNFLAG set, NVWORD1 = V06N40, no sleeper.
* test_engine_fail_csm: NVWORD1 = -0 -> V97N40 flashing, asleep.
* test_engine_enable_request_lm: WHICH = P40TABLE, DISPDEX = -VB99DEX ->
  V99N40 flashing, asleep.
sim/scenarios/external_delta_v_burn_csm.txt: the five `xexpect B6` lines
are `expect` (V99 flash, FLASH on, ENGINE ON, R1/R2 residuals ~0).

## Open questions
None.

## Lessons
* A "paste" display (CLOCPLAY, MARKPLAY, GOPERF) is two NVSUB calls: the
  first VN is the one in A at the call; the flashing verb comes from
  1STOR2ND. Passing the pasted verb as the first VN makes pinball run it
  as a verb -- for an extended verb (V97/V99 are extended verbs too) that
  runs the verb's own code.

## Progress
- DONE cknvwrd1 (CSM ENGREQST/FAILDSP) src/csm/burns/sps/clock.c
- DONE CLOKJOB -13/-35 (LM) src/lm/burns/ignition/clock.c
- DONE unit test, scenario flip
