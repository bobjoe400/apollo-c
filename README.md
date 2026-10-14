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

See `docs/ARCHITECTURE.md`.
Per-module notes: `docs/modules/<module>.md`. Everything under `docs/` describes the
final program; how it was made is in the git log.

## Build

    make            # builds libagc and all modules with -Werror -Wpedantic
    make test       # runs unit tests
    make check      # check_rules.py (Power of Ten), format-check, style
    make format     # tools/format.sh: fix_comments.py + uncrustify over src include tests (docs/CODING_STANDARD.md)
    make style      # tools/check_style.py report (NASA SEL-94-003 rules)

## Run it

    make sim && make scenarios      # build/agc_sim; headless scenarios (sim/scenarios, sim/sim_client.py)

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

**Complete** (2026-10-14). The definition of done (items 1-9) is met: every routine of both ropes is translated or a named,
justified stub; `make`, `make test` and `make check` pass with no
upward module call; the numeric modules match yaAGC within documented
tolerances (bit-exact where the ones'-complement -0 allows); every lane
was reviewed against its AGC source; `build/agc_sim` runs under the
stock VirtualAGC GUI; and the documentation is generated from the final
code. The stretch goal (item 10) is done too.

The mission: all ten Apollo 11 milestones -- insertion and the first
state-vector uplink, TLI, MCC-2, LOI, DOI, PDI to touchdown, lunar
liftoff, rendezvous, TEI, and entry to splashdown -- were flown unattended
in NASSP 9 (Orbiter 2024) on this AGC and matched the stock NASSP run
(NASSP's own yaAGC) on every decision and on the end state, the remaining
differences coming from AGC execution time, which the C does not model.
Seventeen flight bugs that the unit and oracle tests had passed were found
on the way and fixed, each with a test that failed before the fix
(`docs/modules/nassp-integration.md`, "Mission report").

How to run each tier (what each proves: `docs/ARCHITECTURE.md`,
"Verification tiers"):

| tier | command | time, needs |
|---|---|---|
| build, rules | `make && make check` | seconds |
| unit, oracle, scenarios | `make test` (38 test binaries, then the 14 headless scenarios of `make scenarios`) | minutes |
| VirtualAGC GUI | `ln -sf $PWD/build/agc_sim ~/VirtualAGC/bin/yaAGC && ~/VirtualAGC/bin/VirtualAGC` (Run it, above) | a Virtual AGC install |
| LM Simulator | `make lm-sim` (`LMSIM_ARGS="--gui --xshot DIR"` shows the windows) | about 11 minutes; Tcl/Tk; fetches the simulator on first use |
| NASSP replay | `make nassp-replay` | about a minute; no Orbiter; fetches NASSP's yaAGC at the pinned commit |
| NASSP mission | the host procedure in `docs/modules/nassp-integration.md` ("Host procedure": build the DLLs into `C:\NASSP9-dev`, stock reference `C:\NASSP9-ref`, one Orbiter session at a time), then `make nassp-mission NASSP_RUNS=<dir>` | Windows host with Orbiter 2024 + NASSP 9 |

Known limitations (`docs/ARCHITECTURE.md`, "Known limitations"): the
zero-time executive and its two ordering deviations, the limits of
scenario restore, and the downlist words the C cannot hold
(DOWNLIST_SKIP).

## Documentation

Start at `docs/README.md`. Generated from the final code:

* [Reader's guide](docs/guide/README.md) (`docs/guide/`) -- the AGC and
  this translation of it, in twelve chapters and a glossary, for a reader
  who knows C and nothing about the AGC.
* `docs/overview/README.md` -- the system: the two vehicles, executive and
  waitlist, interrupts and clock, erasable and flag layout (and erasable
  by address), the DSKY loop, how a program starts, runs, displays and
  ends, restart groups, and the module map (`python3 -I tools/depgraph.py`).
* `docs/overview/flowcharts/` -- Mermaid flowcharts from the C control
  flow: fresh start, software restart, job pre-emption, waitlist tick, V37
  program change, CSM P40 burn, P52 realign, LM P63-P66 landing, CSM
  P61-P67 entry, LM P12 ascent.
* `docs/overview/xref.md` -- AGC label -> C function -> file:line, for both
  ropes (`python3 -I tools/xref.py`).
* `build/overview/` -- the generated call graph and one-liner tables
  (`make callgraph`, `python3 -I tools/one_liners.py`); not tracked.
* `docs/modules/<lane>.md` -- per lane: scope, design, deviations, stubs,
  tests and "Program flow and decisions" (decision points by function with flags,
  alarms, channel bits and the jobs/tasks started; shared erasables with
  scale; AGC labels -> C functions). The development record is the git log. The mission:
  `docs/modules/nassp-integration.md`; the study before it:
  `docs/overview/nassp_integration.md`.
* `docs/ARCHITECTURE.md` -- the layout, the layering, the verification
  tiers and the known limitations.
* `docs/TIPS.md` -- tips for translating AGC assembly into C, by topic.
* Reference: `docs/TRANSLATION_GUIDE.md` (with the rules the mission
  taught), `docs/CODING_STANDARD.md`, `docs/NAMING.md`;
  the development record is in the git log.

## Licence

GPL-2.0-or-later (see `LICENSE`); the Apollo Guidance Computer source it transposes is in the public domain. The patches under `sim/nassp/patches/` carry NASSP's licence.
