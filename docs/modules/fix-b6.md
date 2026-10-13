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
are `expect` (V99 flash, FLASH on, ENGINE ON, R1/R2 residuals ~0); 42
checks, 0 failed, 0 xfail (V16N85 R1-R3 -00000 / +00000 / +00000). With
B6 fixed the scenario reached code it had never run, and needed:
* V48 (R03): DAPDATR1 11102 (config 1, CSM alone), DAPDATR2 01111, CSM
  weight 57320 lb. With WEIGHT/G = 0, S40.13 (TIMEBURN, p.726) divides by
  zero: TGO = 16383 cs, IGNITION moved TIG 163 s out and WAITLIST refused
  ENGINOFF (16383 > 16250 cs), so the engine stayed on.
* A constant-rate PIPA feed (11 x / 7 y per 2 s for 8 s) in place of the
  single 44 / 28 burst: the pad has no SPS impulse data (EK1VAL-EK3VAL,
  FANG = 0; the sim pad file cannot set them), so S40.13 finds the burn
  non-impulsive and the AGC steers; a steering cycle with no PIPA counts
  would make TGOCALC divide by |BDT - DELVREF| = 0. S40.81 (TGO < 4 s)
  then sets up ENGINOFF at the end of the feed.
* POSTBURN (p.686): V16N40 flashes after SPSOFF; PRO goes to P40RCS
  (V16N85). Under B6 the V99E bypass went to P40RCS directly.
* `run 50` after each `until` that is followed by `expect`s on other
  fields: the display goes out one relay word per T4RUPT, so an `until`
  can match before the rest of the display is up (two checks failed on
  that race once the run's timing moved).

## Open questions
* WAITLIST with a delay above DTMAX: the AGC's WAITLIST tests only for a
  non-positive delay (POODOO 1204); src/exec/waitlist.c:281 asserts
  delay <= AGC_WAITLIST_MAX_DELAY and returns an error, which IGNITION
  passes up (sim "2 errors"). Seen only with WEIGHT/G = 0; not changed
  here (exec lane).
* DONE by loose-ends (`pad sps_impulse`, scenario external_delta_v_impulsive_burn_csm.txt). Was: a pad keyword for the SPS impulse data (EK1VAL-EK3VAL, FANG, EMDOT) in
  src/sim/pad.c would let the scenario run the impulsive-burn path
  (IMPLBURN, ENGINOFF at TIG + TGO) as a real 10 ft/s SPS burn would.

## Lessons
* A "paste" display (CLOCPLAY, MARKPLAY, GOPERF) is two NVSUB calls: the
  first VN is the one in A at the call; the flashing verb comes from
  1STOR2ND. Passing the pasted verb as the first VN makes pinball run it
  as a verb -- for an extended verb (V97/V99 are extended verbs too) that
  runs the verb's own code.

## Progress
- DONE cknvwrd1 (CSM ENGREQST/FAILDSP) src/csm/burns/sps/clock.c
- DONE CLOKJOB -13/-35 (LM) src/lm/burns/ignition/clock.c
- DONE unit test, scenario flip (+ V48, constant-rate feed, POSTBURN PRO, display settle)

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ B6 changed one decision: the CLOCKJOB display word NVWORD1 (`src/csm/burns/sps/clock.c`, CKNVWRD1) selects the V99 engine-enable request or the V97 engine-fail flash, and the display interface pastes V97 / V99 from it on a CLOCPLAY display (`src/io/dsky/display_if.c:dif_second_vn`, `agc_dsky_set_nvword1`). Flows: [guid-powered](guid-powered.md#program-flow-and-decisions), [io-pinball](io-pinball.md#program-flow-and-decisions).
