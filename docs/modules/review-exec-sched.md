# review-exec-sched -- review of the exec-sched lane

History of this lane: [history/lanes/review-exec-sched.md](../../history/lanes/review-exec-sched.md)

## Scope
history/assignments/review.md items 1-5 for src/exec (executive, waitlist, phase
table, restarts, restart tables, alarm/abort, interrupts, fresh start,
change_program/, nouns.c, dispatch.c via tools/gen_registry.py),
include/agc/exec/{exec,jobs}.h (exec-owned declarations), tests/exec/exec.c.
Routed items (history/agents/review_items.md): deferred-restart vs immediate
ENEMA; V37 handoff completeness; activity-light mask fix; P23/R22LEM
restart priority; ZOOM/P40ZOOMA group-3 question.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [exec-sched](exec-sched.md#program-flow-and-decisions).
