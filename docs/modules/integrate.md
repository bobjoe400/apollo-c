# integrate -- wave 3: one registry, no scaffolding, make/test/check clean

## Scope
docs/agents/integrate.md items 1-6 (definition of done items 2, 4, 5), plus
the two GODSPRET continuation jobs of status-handling and its open items
(CLEANDSP continuation; ALARM 1301 / ABORT 1302 raised by the callers of
sqrt/asin/acos).

## Progress
- TODO 1 job/task registry (jobs.h + dispatch.c), delete pending/ext headers
- TODO 2 dissolve glue.c / guid_nav_glue.c, fold *_internal.h
- TODO 3 missing accessors
- TODO 4 cross-lane wiring
- TODO 5 build: sim stub, make test via real lib, debug lib, check
- TODO status-handling open items (N85 jobs, CLEANDSP, 1301/1302)
- TODO 6 note, MODULE_STATUS
