# apollo-c

A module-by-module transposition of the Apollo 11 Guidance Computer (AGC) flight
software into C:

* **Comanche 055** (Colossus 2A) -- Command Module Computer (CMC)
* **Luminary 099** (Luminary 1A) -- Lunar Module Guidance Computer (LGC)

Source of truth: https://github.com/chrislgarry/Apollo-11 (AGC assembly, yaYUL
syntax, transcribed by Ron Burkey / Virtual AGC).

Coding standard: NASA/JPL "Power of Ten" rules (Holzmann), see
`docs/CODING_STANDARD.md`.

## Goals, in priority order

1. **Functional clarity** -- a reader should be able to follow what the AGC
   routine does from the C alone, with the AGC labels and page numbers kept as
   cross-references.
2. **Functional accuracy** -- the same numerics: 15-bit single precision,
   28-bit double precision, the AGC's power-of-two scaling, the AGC's own
   polynomial sin/cos, Kepler/Lambert iteration logic, limiting, overflow
   handling.
3. **Power of Ten compliance** -- no recursion, bounded loops, no heap, short
   functions, assertions, checked returns, limited preprocessor, limited
   pointers, warning-free pedantic builds.

## Layout

See `docs/ARCHITECTURE.md`. Status of each module: `docs/MODULE_STATUS.md`.
Per-module working notes: `docs/modules/<module>.md`.

## Build

    make            # builds libagc and all modules with -Werror -Wpedantic
    make test       # runs unit tests
    make check      # check_rules.py (Power of Ten), format-check, style
    make format     # tools/format.sh: fix_comments.py + uncrustify over src include tests (docs/STYLE.md)
    make style      # tools/check_style.py report (NASA SEL-94-003 rules)

## Run it

    make sim && make scenarios      # build/agc_sim; headless scenarios (sim/scenarios, tools/sim_client.py)

`build/agc_sim` is a drop-in for Virtual AGC's `yaAGC`: same command line,
same peripheral sockets. The stock VirtualAGC launcher always starts
`../bin/yaAGC`, so point that name at our binary and run the launcher
(Apollo 11 Command Module or Lunar Module, DSKY, Telemetry, Run!):

    ln -sf $PWD/build/agc_sim ~/VirtualAGC/bin/yaAGC && ~/VirtualAGC/bin/VirtualAGC

Without the launcher, from a Virtual AGC `Resources` directory:
`$REPO/build/agc_sim --core=source/Comanche055/Comanche055.bin --port=19697 &
../bin/yaDSKY2 --cfg=CM.ini --port=19697` (LM: Luminary099, 19797, LM.ini).
Details, the pad files the launcher's "Custom" core-file option accepts,
and screenshots: `docs/modules/sim.md`.

## Status

The definition of done (`docs/PLAN.md`, items 1-9) is met: every routine
of both ropes is translated or a named, justified stub; `make`, `make test`
(unit, oracle and 14 headless scenarios) and `make check` pass; the
numeric modules match yaAGC within documented tolerances (bit-exact where
the ones'-complement -0 allows); every lane was reviewed against its AGC
source; and `build/agc_sim` runs under the stock VirtualAGC GUI. The
gates were green at the code freeze (`docs/modules/finish.md`) and again
after the closed-loop fixes.

Closed loops (PLAN item 10, stretch), against the proven tools only:

* **LM Simulator** (Virtual AGC `Contributed/LM_Simulator`, unmodified):
  34 of 34 checkpoints match yaAGC flying the same crew sequence (IMU
  zero, coarse and fine align, RCS DAP attitude hold, rate command,
  minimum impulse, V49 manoeuvre, a POODOO restart):

      make lm-sim

  (about 11 minutes real time; the first run fetches the LM Simulator, a
  sparse checkout of virtualagc at the verified commit, into
  `build/virtualagc`; `LMSIM_DIR=<virtualagc>/Contributed/LM_Simulator`
  uses your own; `LMSIM_ARGS="--gui --xshot DIR"` shows the windows). Details: `docs/modules/lm-simulator.md`,
  `docs/modules/closed-loop-fixes.md`.
* **NASSP 9** (Orbiter 2024, Apollo 11, on the Windows host via WSL
  interop): NASSP's Saturn5/LEM modules built against libagc through the
  engine shim `src/nassp/` (CMake project plus a patch to NASSP's top
  CMakeLists). Milestone 1 (earth orbit insertion and the first MCC
  state-vector uplink) and milestone 2 (TLI) pass against the stock NASSP
  run, unattended; milestone 3 (MCC-2 SPS burn) is in progress. The
  integrated build lives in `C:\NASSP9-dev` (stock baseline `C:\NASSP9`);
  runs are scripted scenarios driven by NASSP's own checklist workbook.
  Details: `docs/modules/nassp-integration.md`,
  `docs/overview/nassp_integration.md`.

How to run each: `make scenarios` (headless, both vehicles);
`make lm-sim` (above); the VirtualAGC launcher with `bin/yaAGC` linked to
`build/agc_sim` (Run it, above); NASSP: start Orbiter from
`C:\NASSP9-dev` with an `apollo-c` scenario (one Orbiter session at a
time).

## Documentation

Generated from the final code (documentation wave, `docs/modules/document.md`):

* `docs/overview/README.md` -- the system: the two vehicles, executive and
  waitlist, interrupts and clock, erasable and flag layout, the DSKY loop,
  how a program starts, runs, displays and ends, restart groups, and the
  module map (`python3 -I tools/depgraph.py`).
* `docs/overview/flowcharts/` -- Mermaid flowcharts from the C control
  flow: fresh start, software restart, job pre-emption, waitlist tick, V37
  program change, CSM P40 burn, P52 realign, LM P63-P66 landing, CSM
  P61-P67 entry, LM P12 ascent.
* `docs/overview/xref.md` -- AGC label -> C function -> file:line, for both
  ropes (`python3 -I tools/xref.py`).
* `docs/modules/<lane>.md` -- per lane: scope, decisions, findings, and
  "Program flow and decisions" (decision points by function with flags,
  alarms, channel bits and the jobs/tasks started; shared erasables with
  scale; AGC labels -> C functions).
* `docs/TIPS.md` -- tips for translating AGC assembly into C, by topic.
* Reference: `docs/TRANSLATION_GUIDE.md`, `docs/CODING_STANDARD.md`,
  `docs/STYLE.md`, `docs/NAMING.md`, `docs/ARCHITECTURE.md`,
  `docs/MODULE_STATUS.md`; history in `docs/WORKLOG.md`.
