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
