# Building the oracle tool-chain

`make oracle-tools` (repository root) performs every step below: sparse clone
of virtualagc at the commit named here into build/virtualagc, yaYUL and yaAGC
into tools/oracle/bin/, both ropes assembled and the symbol tables written to
tools/oracle/ropes/ (the assembled .bin is compared with the tracked one).  The
binaries and symbol tables are not tracked.  The manual steps follow.

Sources: clone of https://github.com/virtualagc/virtualagc at commit
c239e9c96ebd52b81c1834cd91d12827bec7f620 (2026-10-09), referred to as
`$SRC/virtualagc` below.  Host: Debian 12, gcc 12.2.0, GNU make.  `cmake` is
NOT installed on this host, so the per-directory GNU Makefiles were used
instead of the top-level CMake build (only yaYUL and yaAGC are needed, no GUI).

## yaYUL (assembler)

    cd $SRC/virtualagc/yaYUL
    make NATIVE=yes cc=gcc NVER='""'
    cp yaYUL tools/oracle/bin/

(`cc=gcc`: the Makefile uses the lower-case variable `${cc}` that the
top-level Makefile normally sets.  `NATIVE=yes` disables the IMCROSS
cross-compiler prefixes.)  No missing packages.

## yaAGC (emulator)

    cd $SRC/virtualagc/yaAGC
    make NATIVE=yes cc=gcc NVER='""' yaAGC
    cp yaAGC tools/oracle/bin/

readline is permanently disabled by the Makefile (`NOREADLINE=yes`), so no
dev packages were needed.  The build also produces `libyaAGC.a`.

## Ropes

    cd $SRC/virtualagc/Comanche055 && .../tools/oracle/bin/yaYUL --unpound-page MAIN.agc > Comanche055.lst
    cd $SRC/virtualagc/Luminary099 && .../tools/oracle/bin/yaYUL --unpound-page MAIN.agc > Luminary099.lst
    cp .../Comanche055/MAIN.agc.bin    tools/oracle/ropes/Comanche055.bin
    cp .../Comanche055/MAIN.agc.symtab tools/oracle/ropes/Comanche055.symtab
    cp .../Luminary099/MAIN.agc.bin    tools/oracle/ropes/Luminary099.bin
    cp .../Luminary099/MAIN.agc.symtab tools/oracle/ropes/Luminary099.symtab

Both assemble with 0 fatal errors, 0 unresolved symbols (Luminary099: 1
warning) and yaYUL reports the expected bank checksums ("bugger words").
Both ropes were verified word-for-word against the official
`Comanche055.binsource` / `Luminary099.binsource` octal transcriptions
(36598 and 36640 words, 0 mismatches):

    python3 -I tools/oracle/check_binsource.py $SRC/virtualagc/Comanche055/Comanche055.binsource tools/oracle/ropes/Comanche055.bin
    python3 -I tools/oracle/check_binsource.py $SRC/virtualagc/Luminary099/Luminary099.binsource tools/oracle/ropes/Luminary099.bin

(virtualagc's own `Tools/oct2bin` does not link standalone without the
CMake build, hence the small Python checker.)

### Why not the chrislgarry/Apollo-11 repo sources

The lead asked for `$SRC/Apollo-11/{Comanche055,Luminary099}/MAIN.agc`
(commit 911e5c0, 2026-07-14).  With this yaYUL they do NOT assemble:

* Comanche055: "Symbol "-1" (namespace 0) is multiply-defined", 1 fatal
  error.  That repo keeps `-1` as a label in several files
  (DISPLAY_INTERFACE_ROUTINES.agc:1308, INTERPRETER.agc:207, ...); the
  virtualagc transcription uses relative ` -1` syntax.
* Luminary099: 2572 fatal errors / 237 unresolved symbols, starting at
  TAGS_FOR_RELATIVE_SETLOC.agc:317 ("ZEROVECS undefined or offset bad");
  the file layout of that repo pre-dates the current yaYUL.

So the fallback in the brief was taken: the ropes come from the virtualagc
copies of the same programs (Comanche 55 = CM Apollo 11, Luminary 99 =
LM Apollo 11).  Page numbers quoted by translation agents from the
Apollo-11 repo are the same scans, so label names and pages agree.

## agc_probe

    cd tools/oracle && make        # builds bin/agc_probe from yaAGC-src/
    make cases                     # regenerates cases/*.txt from cases/*.spec

`yaAGC-src/` is a verbatim copy (GPLv2, COPYING included) of the seven yaAGC
files the probe needs: agc_engine.c agc_engine_init.c agc_utilities.c
agc_symtab.c rfopen.c NullAPI.c and the headers.  `YAAGC_SRC=path` on the
make command line points at a live virtualagc/yaAGC checkout instead.
