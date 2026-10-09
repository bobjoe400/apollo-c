# oracle -- Virtual AGC differential-test rig

## Scope

Build yaYUL and yaAGC, assemble the Apollo 11 ropes, and provide
`tools/oracle/agc_probe` so module agents can run real AGC routines in the
emulator and compare erasable outputs with the C translation.

## What works

| Item | Result |
|------|--------|
| yaYUL, yaAGC | built from virtualagc c239e9c with plain GNU make (no cmake on host); binaries in `tools/oracle/bin/`; commands in `tools/oracle/BUILD.md` |
| Ropes | `tools/oracle/ropes/{Comanche055,Luminary099}.{bin,symtab}`, 0 assembly errors, verified word-for-word against the official `.binsource` transcriptions (`tools/oracle/check_binsource.py`, 0 mismatches) |
| `agc_probe` | `tools/oracle/agc_probe.c` + `Makefile`; links the yaAGC engine (vendored in `tools/oracle/yaAGC-src/`, GPLv2); spec on stdin, `NAME octal` output; symbols, `+n` offsets, banked addresses, `@LABEL` values, `setsp/setdp` fractions, multiple stop labels, cycle limit, per-instruction trace |
| Basic routines | `cases/spsin.spec` -> `spsin.txt`: SPSIN/SPCOS at 24 arguments on Luminary099; `spsin_comanche.spec` identical results on Comanche055. A injected with `set A=`, return trapped with `q ENDOFJOB` / `stop ENDOFJOB` |
| Interpretive routines | `cases/interp_sqrt_sin.spec` -> `.txt`: SQRT (4 args), SIN (4), COS (2) through the real `INTPRET` entry; snippet (SQRT, EXIT, TC ENDOFJOB) poked into the VAC2 work area, `Q` = snippet, FIXLOC/PUSHLOC = VAC1. Results: sqrt(.5)=0.707106780, (1/2)sin(2pi/8)=0.353553388, sin(1/4 rev)/2 = 020000,77777 (DP +0.5 with -0 low word) |

Exact commands:

    cd tools/oracle && make && make cases
    AGC_ORACLE_ROPES=ropes ./bin/agc_probe < cases/spsin.spec

## What does not work / deviations

* The chrislgarry/Apollo-11 repo sources (as cloned) do not assemble with
  current yaYUL (Comanche055: multiply-defined `-1`; Luminary099: 2572
  fatal errors from TAGS_FOR_RELATIVE_SETLOC.agc on).  Ropes were built
  from virtualagc's own Comanche055/Luminary099 transcriptions of the same
  programs (fallback allowed by the brief).  Label names and page numbers
  agree with the Apollo-11 repo files the translators read.
* No peripherals: the engine is linked with NullAPI.c (no sockets, no DSKY,
  IMU, radar).  Interrupts are suppressed by default; routines whose
  behaviour depends on T4RUPT/WAITLIST timing cannot be oracled this way.
* Fixed memory is read-only from the spec by design; interpretive code
  under test is injected into erasable (the EMP technique), never patched
  into the rope.
* The vendored engine is compiled with `-w`; it is upstream code and is
  not held to the project coding standard (nor is `agc_probe.c`).
* Hardware alarms are inhibited (`InhibitAlarms=1`), so a runaway routine
  ends with `STATUS LIMIT` rather than a GOJAM restart.

## Stubs

None.

## Requested core additions

None.  (Tests will want a tiny parser for the `NAME octal` format; the
format is documented in `tools/oracle/README.md` section 2.)

## Requested jobs-tasks

None.

## Tests written

Oracle cases only (`tools/oracle/cases/*.spec|txt`); `tests/oracle_*.c`
are for the module agents / integrate agent.

## Open questions

* The `.symtab` files are 11.8 MB each (yaYUL dumps every source line);
  keep them, or regenerate on demand from `BUILD.md`?
* Routines that live in superbank (FBANK 40-43) are addressed through the
  symtab's `Super` bit and channel 7; exercised in code, not yet by a case.
