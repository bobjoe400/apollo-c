# closed-loop-fixes -- LM Simulator bugs (L1-L3, S-L1..S-L3, PROG after POODOO) and the document wave's code findings

History of this lane: [history/lanes/closed-loop-fixes.md](../../history/lanes/closed-loop-fixes.md)

## Scope
history/agents/review_items.md "From lm-simulator" and "From the document
wave". Reference: the AGC listing (SRC/Apollo-11/Luminary099,
Comanche055) for flight code; yaAGC (virtualagc yaAGC/agc_engine.c) for
the sim protocol. Acceptance: `make lm-sim` exits 0, with make, make test
(incl. scenarios), make check, prolog-report 0, always-ok-report 0 green.

## Design table
| item | reference | files | fix | test (fails before, passes after) |
|---|---|---|---|---|
| L1 | V37BAD L p.227, ALMCYCLE C p.364 | src/io/dsky/pinball/{internal.h,keys.c,nvsub.c,verbs.c} | ALMCYCLE sets a mark (`almcycle`); only a marked ALARM re-enters ENTER (pin_recycle) | tests/io/dsky.c test_change_program_refused |
| L2 | FINEK2 L p.273 (`CAF LGYROBIN` = ECADR OGC) | src/imu/gyro.c, src/lm/systems_test.c | the OGC gyro buffer IS agc_align_ram()->ogc (one erasable); the redundant copy in SETUPER1 removed | tests/imu/imu.c test_gyro_pulse_loaded_angles |
| L3 | MIDORLST / LASTCHG L p.1457, T6JOB L p.1404 | src/lm/dap/rcs/t6rupt.c | a zero T6 list delta is -0 and loads TIME6 = -0; the DOT6RUPT assertion that forbade TIME6 = -0 after T6JOB deleted | tests/lm/dap.c test_t6_list_zero_delta |
| S-L1 | yaAGC CpuWriteIO -> ChannelOutput | src/core/channels.c, include/agc/core/channels.h, src/sim/channels.c | core logs each WRITE/WOR/WAND with the word after it (agc_chan_next_write, 64-entry ring); the sim sends one packet per output write, in order | tests/sim/harness.c test_output_writes |
| S-L2 / S-L3 | yaAGC PushCduFifo / ServiceCduFifo | src/sim/counters.c, src/sim/channels.c, src/sim/internal.h | CDUX-CDUZ PCDU/MCDU queued per CDU, applied at 400 (slow) / 6400 (fast) counts a second | tests/sim/harness.c test_cdu_fifo |
| P40TMP | P40SET C p.686, CALLT-35 L p.734, LONGCALL/WAITLIST | src/csm/burns/sps/burn.c (+internal.h), src/lm/burns/ignition/burnbaby.c (+internal.h) | a delay <= 0 goes to LONGCALL as 0: POODOO 1204 as in the AGC (was: silently 1 cs; LM: BAD_ARG / 1 cs) | tests/guidance/guidance.c test_preburn_late |
| D1 PROG after POODOO | GOTOPOOH / GOP00FIX L p.223 (C p.194), NOVAC2 / SPECTEST L pp.1106-1107, STARTSB2 DSPOFF L p.221, DSPMMJB L p.458 | src/exec/executive.c, src/exec/internal.h, src/exec/fresh_start.c | restart code reaching GOTOPOOH runs on as the job SPECTEST put in core set 0 (MMDSPLAY's DSPMMJOB): GOP00FIX replaces it (exec_core0_continue_as) | tests/exec/exec.c test_poodoo_lm, test_waitlist_zero_delay (+ test_goprog updated); sim/lm_simulator/tutorial.txt poodoo_p00 |
| WOTEST (found here) | DODOWNTM / WOTEST L p.997 (Comanche C p.1096 writes every time) | src/io/telemetry/down_telemetry.c | LM: WOR CHAN13 only when the word order bit is 0 | tests/io/service_verbs_telemetry.c test_downrupt_word_order_writes |
| stale comments | -- | src/core/erasable.c, src/core/state.c, include/agc/core/state.h, src/programs/alignment/imu_realign.c, src/exec/executive.c | prologs say what the code does | -- |
| repeated assertions | rule 5 | src/lm/ascent.c (x3), src/lm/landing/lunar_landing.c, src/lm/landing/guidance.c, src/exec/change_program/idle.c | deleted | -- |

## Tests written
- tests/io/dsky.c test_change_program_refused (L1)
- tests/imu/imu.c test_gyro_pulse_loaded_angles (L2)
- tests/lm/dap.c test_t6_list_zero_delta (L3: MIDORLST and LASTCHG)
- tests/exec/exec.c test_poodoo_lm, test_waitlist_zero_delay (D1; test_goprog's DSPMMJB expectation corrected)
- tests/guidance/guidance.c test_preburn_late (P40SET, CALLT-35)
- tests/io/service_verbs_telemetry.c test_downrupt_word_order_writes (WOTEST)
- tests/sim/harness.c test_output_writes (S-L1), test_cdu_fifo (S-L2);
  test_counters' PCDU/MCDU wrap moved to CDUT (CDUX-Z are paced now)
Each fails on the code before its fix (checked by running it against the
old code, or for P40TMP/CALLT-35 by the old inline logic: 1 cs / BAD_ARG).

## Program flow and decisions

_Documentation wave (2026-10-12), from the code at HEAD 99b2c6e; citations checked at the final tree (HEAD 3b97dc8). Each fix
changed a decision point; the full flow of each is in the owning lane's
section (linked)._

| fix | decision as the code now takes it | lane section |
|---|---|---|
| L1 | `src/io/dsky/pinball/keys.c:pin_enter` re-enters ENTER only while `verbs.c:pin_recycle` sees the `almcycle` mark that `pin_almcycle` sets; an ALARM status from a called routine (V37BAD) ends the job | [io-pinball](io-pinball.md#program-flow-and-decisions) |
| L2 | `src/imu/gyro.c:gyro_words`: the OGC buffer is one erasable, which V25N93 loads and IMUPULSE reads; first `agc_align_ram()->ogc`, now owned by imu, `src/imu/gyro.c:agc_imu_ogc` (layering) | [imu](imu.md#program-flow-and-decisions) |
| L3 | `src/lm/dap/rcs/t6rupt.c:set_time6`: a zero T6 delta loads TIME6 = -0; `agc_lm_dap_t6jobchk`: -0 runs T6JOB, +0 is the CCS hole | [dap-lm](dap-lm.md#program-flow-and-decisions) |
| S-L1 | `src/core/channels.c:wlog_add` logs every WRITE/WOR/WAND; `src/sim/channels.c:send_writes` sends one packet per entry | [state-flags](state-flags.md#program-flow-and-decisions), [sim](sim.md#program-flow-and-decisions) |
| S-L2/3 | `src/sim/counters.c:sim_counters_tick`: CDUX-Z counts queued per CDU, 400 / 6400 counts a second | [sim](sim.md#program-flow-and-decisions) |
| P40TMP | `src/csm/burns/sps/burn.c:agc_csm_preburn_call_tigblnk` (P40SET) and `src/lm/burns/ignition/burnbaby.c:ign_call_tig_35` (CALLT-35): a delay <= 0 goes to LONGCALL as 0 = POODOO 1204 | [guid-powered](guid-powered.md#program-flow-and-decisions) |
| D1 | `src/exec/fresh_start.c:agc_exec_gotopooh`: from restart code, `src/exec/executive.c:exec_core0_continue_as` makes GOP00FIX replace the DSPMMJOB SPECTEST put in core set 0 (PROG stays blank) | [exec-sched](exec-sched.md#program-flow-and-decisions) |
| WOTEST | `src/io/telemetry/down_telemetry.c:agc_telem_downrupt`: LM writes the channel 13 word-order bit only when it is 0 | [sys-misc](sys-misc.md#program-flow-and-decisions) |
