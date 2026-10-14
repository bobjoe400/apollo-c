# Reader's guide: the Apollo guidance computer through its C

Status: complete (12 chapters and the glossary, written from the final code).

This guide is for a reader who knows C and basic orbital mechanics and
nothing about the Apollo Guidance Computer (AGC). It explains how the
AGC's flight software works by walking through its translation into C in
this repository: Comanche 055 (the Command Module program flown on
Apollo 11) and Luminary 099 (the Lunar Module program). Every statement
is taken from the code or from the AGC listing it translates, and names
the C function (`file:function`, paths under `src/` unless shown) and the
AGC label and listing page beside it, so it can be checked.

## Contents

| # | Chapter | What it covers |
|---|---|---|
| 1 | [What the AGC is and what this code is](01-the-agc.md) | the machine, the two ropes, the DSKY, programs and routines, what was and was not translated |
| 2 | [Numbers](02-numbers.md) | 15-bit ones' complement, -0, SP/DP/TP, scale factors, overflow, word pairs |
| 3 | [The interpreter](03-interpreter.md) | the interpretive language as a C library; CALCGTA beside its listing |
| 4 | [Time and work](04-time-and-work.md) | timers, interrupts, the waitlist, the executive, zero-time jobs |
| 5 | [Memory and flags](05-memory-and-flags.md) | erasable as typed storage, EQUALS aliases, the address map, flagwords, pad loads, restarts |
| 6 | [The crew interface](06-crew-interface.md) | keys, verbs, nouns, the display loop, flashing displays, paste displays |
| 7 | [Restarts and the phase tables](07-restarts.md) | alarms, GOJAM, ENEMA, restart groups, fresh start, V37 |
| 8 | [Navigation](08-navigation.md) | state vectors, conics, Encke integration, the W matrix; one LM radar mark |
| 9 | [Guidance and control](09-guidance-and-control.md) | SERVICER, P30, P40/P41, the DAPs, KALCMANU; P40 from TIG-5 to the residuals |
| 10 | [Lunar descent and ascent](10-lunar-descent-and-ascent.md) | P63/P64/P66, quadratic guidance, throttle, landing radar, LPD; P12 |
| 11 | [Entry](11-entry.md) | P61-P67, the entry phases, the roll command, the entry DAP |
| 12 | [Reading further](12-reading-further.md) | label to C and back, how the tests show fidelity, deviations, history |
| A | [Glossary](appendix-glossary.md) | AGC terms, labels and erasables |

## Reading order

Chapters 1 to 4 are the foundation: read them in order. Chapter 2
(numbers) and chapter 4 (jobs and tasks) are needed for everything
after; chapter 3 is needed to read any navigation or guidance code.
Chapters 5 to 7 describe the machinery every program uses (memory, the
DSKY, restarts) and can be read in any order. Chapters 8 to 11 are the
flight: 8 and 9 first, then 10 (LM) or 11 (CM). Chapter 12 and the
glossary are for reference.

## Conventions

* `dir/file.c:function` is a path under `src/`; other paths
  (`include/`, `docs/`, `tests/`, `sim/`, `tools/`) are from the
  repository root.
* "C p.*n*" and "L p.*n*" are pages of the Comanche 055 and Luminary
  099 listings; the label is given beside them.
* Listing excerpts are quoted as transcribed, without blank lines and
  `EBANK=` bank-bookkeeping lines, long ones shortened with `...`; C
  excerpts are from the final tree, shortened with `...` where only part
  of a function matters.
* Program, routine, verb and noun numbers (P40, R60, V37, N40) appear in
  text; the code names them by what they do (`docs/NAMING.md`).
