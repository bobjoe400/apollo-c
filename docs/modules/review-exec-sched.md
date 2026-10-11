# review-exec-sched -- review of the exec-sched lane

## Scope
docs/agents/review.md items 1-5 for src/exec (executive, waitlist, phase
table, restarts, restart tables, alarm/abort, interrupts, fresh start,
change_program/, nouns.c, dispatch.c via tools/gen_registry.py),
include/agc/exec/{exec,jobs}.h (exec-owned declarations), tests/exec/exec.c.
Routed items (docs/agents/review_items.md): deferred-restart vs immediate
ENEMA; V37 handoff completeness; activity-light mask fix; P23/R22LEM
restart priority; ZOOM/P40ZOOMA group-3 question.

## Progress
- TODO fidelity: executive.c
- TODO fidelity: waitlist.c
- TODO fidelity: phase_table.c
- TODO fidelity: restarts.c, restart_tables.c
- TODO fidelity: alarm.c
- TODO fidelity: interrupts.c, erasable.c
- TODO fidelity: fresh_start.c
- TODO fidelity: change_program/{v37,csm,lm,idle}.c
- TODO routed items (5)
- TODO style debt (1917 findings at start; dispatch.c 1026 is generated)
- TODO make / make test / make check

## Findings
| # | file:line | AGC (label, page) | finding | action |
|---|---|---|---|---|

## Routed items

## Lessons
