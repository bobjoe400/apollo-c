# review-prog-rendz -- review of the prog-rendz lane

History of this lane: [history/lanes/review-prog-rendz.md](../../history/lanes/review-prog-rendz.md)

## Scope

history/assignments/review.md items 1-5 for lane prog-rendz (docs/modules/prog-rendz.md):
src/programs/{navigation,targeting,displays}/*.c, src/csm/navigation/*.c,
src/lm/navigation/*.c, src/lm/radar/*.c, include/agc/programs/rendezvous.h,
tests/programs/rendezvous.c, tests/oracle/targeting.c.
Routed items (history/agents/review_items.md): F13 CDH DIFFALT 62 m;
targeting findings; P23/R22LEM restart priority (with exec review);
programs/navigation/noun_data.c -> lm/radar upward dependency.

Scratch: SCRATCH/review-prog-rendz/ (f13fix/ holds the F13 evidence: the
word-level WLOOP experiment diff and the oracle specs).

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [prog-rendz](prog-rendz.md#program-flow-and-decisions).
