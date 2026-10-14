# Assignment: dispatch   (tier: mechanical)   -- run after format2

Make the dispatch tables readable: tools/gen_registry.py must emit exactly
four functions in src/exec/dispatch.c -- agc_dispatch_job_csm,
agc_dispatch_task_csm, agc_dispatch_job_lm, agc_dispatch_task_lm -- each a
single flat `switch` with one `case AGC_JOB_X: return (agc_job_x(ctx));`
per registry row, ordered as in tools/registry.tsv (group the cases by
subsystem with a one-line comment per group, no chunking by count), plus
the two public entry points that pick the vehicle. Put the marker
`/* GENERATED-DISPATCH: rule 4 exempt */` within the first 5 lines of the
generated file (docs/CODING_STANDARD.md rule 4 exemption). Delete the 89
chunk functions. Regenerate include/agc/exec/jobs.h unchanged in content.
Run `make format`, `make`, `make test`, `make check` (0 hard). Update
docs/modules/integrate.md (registry section) and docs/ARCHITECTURE.md
(dispatch sentence). Report <= 8 lines.
