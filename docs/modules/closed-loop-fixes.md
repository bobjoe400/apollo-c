# closed-loop-fixes -- LM Simulator bugs (L1-L3, S-L1..S-L3, PROG after POODOO) and the document wave's code findings

## Scope
docs/agents/review_items.md "From lm-simulator" and "From the document
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

## Progress
- DONE L1 pin_recycle() (verbs.c), pin_enter / pin_charin (keys.c), pin_nvsub1 (nvsub.c)
- DONE L2 gyro_words() (gyro.c); systems_test.c SETUPER1 copy removed
- DONE L3 set_time6() zero delta = -0; agc_lm_dap_t6rupt is T6JOBCHK; agc_lm_dap_time6_count +1 -> -0
- DONE S-L1 core write log + sim send_writes()
- DONE S-L2 / S-L3 CDU FIFO (sim_counters_tick)
- DONE P40TMP agc_csm_preburn_call_tigblnk (P40SET), ign_call_tig_35 (CALLT-35)
- DONE stale comments (erasable.c 38 getter prologs + file prolog, state.c 3 + prolog, state.h, imu_realign.c S52.3 / S52.2, executive.c vac_claim)
- DONE six repeated assertions
- DONE D1 exec_core0_continue_as (executive.c), agc_exec_gotopooh (fresh_start.c)
- DONE WOTEST agc_telem_downrupt (LM only, as Luminary)
- DONE harness: sim/lm_simulator/lm_sim.py =skip / hold=R; tutorial tolerances from yaAGC's own spread; POODOO checkpoint
- DONE `make lm-sim` exit 0: 34 checkpoints, all matched, twice in a row (2026-10-12); make test 0 (all unit/oracle tests, 14 scenarios)

## Findings
- L1 cause, generalised: any pinball load-resume routine that returned
  AGC_STATUS_ALARM was recycled as ALMCYCLE (status aliasing), not only
  V37's CANTR00. The fix keys the recycle on ALMCYCLE having run (a
  pinball-local mark), not on the status value; pinball callers still see
  ALARM and stop. A refused V37 now returns ALARM from agc_dsky_keys
  (V37BAD ends the job; the test checks that).
- L2: OGC/IGC/MGC were two stores (the noun-93 erasable and gyro.c's
  buffer); CALCGTA and SETUPER1 copied one into the other, the keyboard
  load did not. One store now; every IMUPULSE on OGC (P52/P54 torquing,
  realign, lunar surface trim, systems test, FINEK2) reads the erasable.
- L3: besides MIDORLST's `SU TIME6` -0, LASTCHG's `CS A / AD NEG0`
  (T6FURTHA) gives -0 when the new firing ends with the second one; the
  same fix covers both (a zero T6 delta loads TIME6 = -0). DOT6RUPT's
  assertion "TIME6 != -0 after T6JOB" was the C's own invariant, not the
  AGC's (T6JOB loads -0 to fire at the next DINC): deleted. The sim
  harness helper agc_lm_dap_time6_count (unused; the sim counts TIME6 in
  clock.c) also made +0 from +1: fixed to -0 as the adder does.
- P40TMP (and the LM's SAVET-30): MIDTOAV1 guarantees TIMEDELT (12.5 s CM,
  20 s LM) before TIG-30 only at its start; a slow integration leaves
  less than 5 s, and both ropes then end in POODOO 1204 (Comanche via
  LONGCYCL/LASTTIME into WAITLIST's WATLST0-, Luminary in LONGCALL's
  LONGPOOH). The C clamped the CSM delay to 1 cs silently and returned
  BAD_ARG (negative) or 1 cs (zero) on the LM; both now pass 0 to
  LONGCALL. P40SET and CALLT-35 are their own functions (named labels).
- S-L1: the LM Simulator's process_data sets its jets from bo(5/6) before
  it stores the packet's word: it fires the jets of the previous ch5/6
  packet. yaAGC writes ch5 twice per QR pass (WRITEU, WRITEV) and ch6 once
  per P pass: 20 + 10 packets a second while the DAP runs. Change-only
  output sent none while idle; a per-sample "written" flag sent 10 + 10
  (the U and V writes collapse). Only a per-write log gives yaAGC's
  packet stream (now 100/50 in 5 s of hold, as yaAGC).
- S-L2 / S-L3: with the CDU FIFO the coarse-align CDUs lag the gimbals as
  under yaAGC and CHKCORS raises 00211 in both; COARS itself was right.
- WOTEST (found by the S-L1 write log): Luminary's DODOWNTM sets the
  word-order bit with WOR CHAN13 only when it is 0 (WOTEST, "only if it
  already isn't"); Comanche writes it on every DOWNRUPT. The C wrote it
  every time on both, harmless for the channel value but 50 extra ch13
  packets a second on the LM (ours 66/s, yaAGC 2.5/s; measured with a
  counting peripheral on an idle LM). Now per vehicle; the idle packet mix
  matches yaAGC per channel (10, 12, 13, 5, 6, 34, 35).
- One `make lm-sim` run (before the WOTEST fix) ended with the simulator's
  Tcl failing "error writing sock...: broken pipe" on its AGC socket at
  358 s: agc_sim had dropped the simulator's connection. The only drop
  paths are EOF/read error and a send that is neither complete nor
  EAGAIN (src/sim/socket.c send_to: a partial send closes the client).
  The simulator runs a dynamics step per received packet, so 40% more
  packets than yaAGC (the ch13 surplus) can let its input back up until
  a send is partial. Not reproduced after the WOTEST fix (and not under
  gdb). Suggested for the sim/socket owner (nassp-integration edits
  socket.c now): keep a partial packet's remainder per client instead of
  closing, as a stream peer expects.
- D1 (PROG after a POODOO): root cause found and reproduced with a fixed
  sequence (V25N26 0/1/0, V31E in P00: POODOO 1204) on both AGCs: yaAGC
  PROG blank under the V37 flash, ours PROG 00. In the AGC the software
  restart blanks DSPTAB (STARTSB2 DSPOFF), frees every core set
  (PRIORITY = -0, NEWJOB = -0); GOPROG3's MMDSPLAY NOVACs DSPMMJOB, which
  NOVAC2 puts in core set 0 and SPECTEST makes the current job (NEWJOB =
  +0, LOC = DSPMMJOB). With no group active and MODREG = +0 the restart
  code goes on to GOTOPOOH -> POSTJUMP GOP00FIX in that same core set;
  GOFLASH's ENDIDLE -> JOBSLEEP stores GOP00FIX's LOC there, so DSPMMJOB
  never runs and PROG stays blank. The C ran the restart outside any job,
  NOVAC'd GOP00FIX separately, and DSPMMJOB displayed 00. Fix: GOTOPOOH
  from restart code takes over core set 0 when SPECTEST made it current.
  The same holds after a hardware restart (GOPROG) in P00 (test_goprog
  encoded the old behaviour and was corrected). The original lm-simulator
  observation mixed two different POODOOs (ours 1103 from T6RUPT, yaAGC
  1204); with L3 fixed neither occurs in the tutorial any more, so the
  tutorial ends with the V31 POODOO as a checkpoint.
- lm-sim repeatability (harness): with L1-L3, S-L1, S-L2 fixed, every
  checkpoint up to the first RHC detent matches yaAGC (displays, alarms
  incl. 00211, gimbals, fine align 11.02/22.00/28.52, packet cadence
  100/50 ch5/6 in 5 s of hold, rate during the deflection). After the
  first detent the closed loop is not repeatable even for yaAGC against
  itself: two yaAGC runs of the same sequence gave pitch rate 10 s after
  detent 0.90 vs -0.16 deg/s, yaw 0.01 vs 0.54 deg/s, pitch gimbal at the
  yaw detent 48.0 vs 35.3 deg, V49 end attitude 27.8/22.4/356.4 vs
  25.6/32.6/360.0, needles 90 counts apart (real-time AGC, Tcl timer,
  host scheduling, wall-clock RHC timing; the DAP leaves a free drift
  inside its 5 deg deadband). The tutorial's per-snap tolerances there
  (rate 0.2-0.4, deg 2-5, needle 30-80) were tighter than the reference's
  own spread, so `make lm-sim` could not pass for yaAGC against yaAGC.
  sim/lm_simulator/lm_sim.py gained `=skip` for deg/rate/needle and `hold=R` (each
  AGC on its own: every body rate within R deg/s); from the first detent
  the tutorial compares displays, alarms, jets used and the rate during a
  deflection, and checks that each AGC holds its attitude (hold=1.5,
  yaAGC's worst seen 0.90). What that still catches: L3 (V05N09 alarms),
  S-L1 (no rate during a deflection, tumbling after it), a missing
  manoeuvre display flow. What it no longer catches: an attitude-hold or
  manoeuvre accuracy difference smaller than yaAGC's own scatter (the V49
  start is within ~6 deg of the 30/30/0 target in this model, so the end
  attitude does not discriminate anyway).
- Not changed (other lane's files): `make format-check` fails on
  src/core/ecadr.c, src/sim/internal.h (lines 111-112), src/sim/main.c,
  tests/core/ecadr.c and src/nassp/ (nassp-integration's work in progress);
  every file of this lane passes.

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

## Requested core additions (made; the assignment covers S-L1)
- include/agc/core/channels.h: `AGC_CHAN_WLOG_MAX` and
  `bool agc_chan_next_write(uint16_t *chan, uint16_t *value)` (the
  program's channel writes in order, for the sim's peripheral packets).

## Lessons
- A status value is not a control-flow token: when one routine's return
  means "do X again" and the same value means "an alarm happened"
  elsewhere, the caller cannot tell them apart. Mark the event at its
  source (ALMCYCLE ran) instead of overloading a status.
- One AGC erasable must be one C store. Where two modules each wanted a
  copy (noun table vs gyro driver), every writer that forgets the copy is
  a bug that only a closed loop shows.
- Ones' complement -0 is a value with meaning, not a rounding of 0: T6JOB
  relies on -0 in TIME6 (rupt at the next DINC) and +0 there is the
  CCSHOLE. A C list holding plain integers must say which zero it means.
- A peripheral's behaviour can depend on how many packets arrive, not just
  their content; count packets per second against the reference before
  comparing physics.

## Program flow and decisions

_Documentation wave (2026-10-12), from the code at HEAD 99b2c6e. Each fix
changed a decision point; the full flow of each is in the owning lane's
section (linked)._

| fix | decision as the code now takes it | lane section |
|---|---|---|
| L1 | `src/io/dsky/pinball/keys.c:pin_enter` re-enters ENTER only while `verbs.c:pin_recycle` sees the `almcycle` mark that `pin_almcycle` sets; an ALARM status from a called routine (V37BAD) ends the job | [io-pinball](io-pinball.md#program-flow-and-decisions) |
| L2 | `src/imu/gyro.c:gyro_words`: the OGC buffer is `agc_align_ram()->ogc`, the one erasable V25N93 loads and IMUPULSE reads | [imu](imu.md#program-flow-and-decisions) |
| L3 | `src/lm/dap/rcs/t6rupt.c:set_time6`: a zero T6 delta loads TIME6 = -0; `agc_lm_dap_t6jobchk`: -0 runs T6JOB, +0 is the CCS hole | [dap-lm](dap-lm.md#program-flow-and-decisions) |
| S-L1 | `src/core/channels.c:wlog_add` logs every WRITE/WOR/WAND; `src/sim/channels.c:send_writes` sends one packet per entry | [state-flags](state-flags.md#program-flow-and-decisions), [sim](sim.md#program-flow-and-decisions) |
| S-L2/3 | `src/sim/counters.c:sim_counters_tick`: CDUX-Z counts queued per CDU, 400 / 6400 counts a second | [sim](sim.md#program-flow-and-decisions) |
| P40TMP | `src/csm/burns/sps/burn.c:agc_csm_preburn_call_tigblnk` (P40SET) and `src/lm/burns/ignition/burnbaby.c:ign_call_tig_35` (CALLT-35): a delay <= 0 goes to LONGCALL as 0 = POODOO 1204 | [guid-powered](guid-powered.md#program-flow-and-decisions) |
| D1 | `src/exec/fresh_start.c:agc_exec_gotopooh`: from restart code, `src/exec/executive.c:exec_core0_continue_as` makes GOP00FIX replace the DSPMMJOB SPECTEST put in core set 0 (PROG stays blank) | [exec-sched](exec-sched.md#program-flow-and-decisions) |
| WOTEST | `src/io/telemetry/down_telemetry.c:agc_telem_downrupt`: LM writes the channel 13 word-order bit only when it is 0 | [sys-misc](sys-misc.md#program-flow-and-decisions) |
