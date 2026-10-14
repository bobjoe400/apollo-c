# review-io-dsky -- review of io-pinball + io-nouns

History of this lane: [history/lanes/review-io-dsky.md](../../history/lanes/review-io-dsky.md)

## Scope
src/io/dsky/display_interface.c, src/io/dsky/pinball/*.c, src/io/dsky/nouns/*.c,
include/agc/io/dsky.h, include/agc/io/nouns.h, tests/io/dsky.c,
tests/io/nouns.c, tests/oracle/dsky.c. AGC: PINBALL_GAME_BUTTONS_AND_LIGHTS,
PINBALL_NOUN_TABLES, DISPLAY_INTERFACE_ROUTINES, KEYRUPT_UPRUPT (CM + LM).

## V27 (DSPFMEM) stub -- justification
V27 shows a word of fixed memory addressed by an FCADR plus superbank bits
(noun 26 / 02). The C translation has no fixed-memory image: constants
are compiled C values and code is C functions, so there is no word at an
FCADR to show. The only rope image in the project is the test oracle's
(tools/oracle), which the flight library must not depend on. pin_dspfmem
checks its preconditions and returns AGC_STATUS_UNIMPLEMENTED; listed in
io-pinball.md "Stubs". Not used by any flight procedure (ground checkout
verb).

## Noun write-path gaps
Loadable (no NNTYPTAB no-load bit) components whose read works but whose
write returns NOT_FOUND from agc_dispatch_noun (src/exec/nouns.c, not this
lane): the values are computed on read (noun_derived) or owned by a
module that exposes a reader only.
* CSM: N17/N96 CPHIX..+2, N38 TET, N49 DELTAR/DELTAV/source code, N51
  RHOSB/GAMMASB, N55 NN1, N62 HDOT/ALTI (P11 readout), N87/N94 MRKBUF1+3/+5,
  N95 PRAXIS..+2.
* LM: N18 FDAIX..Z, N38 TET, N49 DELTAR/DELTAV/WHCHREAD, N51 ALPHASB/BETASB,
  N55 NN, N67/N78 RSTACK words, N80 DATAGOOD/OMEGAD.
In the AGC a V21/V25 on these stores into the erasable; in C the load
stops with NOT_FOUND. All are displays the programs overwrite (FDAI
angles, mark buffers, P11 readout, TET) except N55 NN/NN1 (P34/P35
iteration count, which the crew may load) -- see Requests.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [io-pinball](io-pinball.md#program-flow-and-decisions), [io-nouns](io-nouns.md#program-flow-and-decisions).
