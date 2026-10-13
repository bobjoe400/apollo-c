# format3 -- brace initialisers joined unless they are tables

## Scope
User example: src/csm/burns/return_to_earth/conic.c, the `rte_kfit_t`
initialisers wrapped mid-brace. `tools/fix_comments.py` used to leave every
multi-line `{...}` untouched; it now joins a partially wrapped initialiser
onto one line like any other statement (docs/CODING_STANDARD.md "Brace
initializers follow the same rule as call arguments").

Files: `tools/fix_comments.py` (new `init_is_table`, used by `fix_segment`
and `long_statements`; docstring), 60 files under src include tests
(layout only; the rest of the tree was already a fixed point). Run on
everything EXCEPT src/core, include/agc/core and tests/core, because
review-core-numeric was editing those; `make format` covers core later.

## Design table
| Piece | What it does |
|-------|--------------|
| `init_is_table(src, o, c)` | A multi-line outermost `{...}` is a table, kept as written, when (a) any comment lies inside it (documented rows with trailing comments, e.g. PYTABLE, NOLEMVAL), or (b) one element per line: each top-level element starts its own line (the first may share the `{` line; a nested brace is one element) and the braces span >= 3 lines (two continuation lines). |
| `fix_segment` | A multi-line outermost initialiser that is not a table no longer aborts the segment: the whole statement is joined by the existing join path (one line, trailing comment kept). Nested braces inside a table are not examined. |
| align (phase 3) | Unchanged. The joined declaration is now a one-line declaration, so its `=` and trailing-comment columns join the paragraph's, as in the conic.c example. |

## Before / after (src/csm/burns/return_to_earth/conic.c)

    static const rte_kfit_t K1_BELOW_21K   = {2556848, 0,
                                            189866};    /* V(400), 0, 5500 */
    static const rte_kfit_t K1_21K_28K     = {178979340, -642218,
                                          82850};       /* V(28K), -.443, 2400 */
    ...
    static const rte_kfit_t K2_ABOVE_28_4K = {204547817, 8924363,
                                              1789570}; /* V(32K), .000285, 2.4 */
    ->
    static const rte_kfit_t K1_BELOW_21K   = {2556848, 0, 189866};            /* V(400), 0, 5500 */
    static const rte_kfit_t K1_21K_28K     = {178979340, -642218, 82850};     /* V(28K), -.443, 2400 */
    ...
    static const rte_kfit_t K2_ABOVE_28_4K = {204547817, 8924363, 1789570};   /* V(32K), .000285, 2.4 */
    static const agc_dp_t   V21K           = 134217728;                       /* V(21K)   64.000 B-7 m/cs  */

(The lead's commit 3b08ab3 had already carried conic.c to this shape; the
tool now produces it from the wrapped text.) Another case, steer.c:

    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_COPY40_9,
                         AGC_TASK_NONE, (int16_t) AGC_PRIO10,
                         false, 0U};
    ->  agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_COPY40_9, AGC_TASK_NONE, (int16_t) AGC_PRIO10, false, 0U};

Kept as tables: one-element-per-line tables, and any initialiser with a
comment inside (survey of the non-core tree: 154 multi-line outermost
initialisers kept as tables, 152 candidates joined; 60 files touched).

## Validation
* `gcc -E -P` (with and without -DAGC_HOST_DEBUG) of the touched .c files
  against f638a45: token-identical (white space stripped).
* `tools/format.sh` twice on the touched files: second run changes nothing;
  `tools/format.sh --check` of the 283 non-core files passes.
* `make`, `make test`: green (core was building at the time).

## Deviations
* Documented tables (a comment inside) are kept even with several elements
  per line; joining them would merge a column of trailing comments into one
  line. Nested rows of a kept table are not individually joined.
* Row-per-line numeric tables without comments but with several elements
  per line (e.g. reentry.c `map[]`) are joined: the standard allows only
  one line or one element per line.

## Stubs / Requested core additions / Requested jobs-tasks
None.

## Tests written
None (formatter). Scratch: SCRATCH/format3/ (survey.py, parity.sh).

## Open questions (for the lead)
* `make format` has not run on src/core, include/agc/core, tests/core.

## Progress
- DONE fix_comments.py init handling; non-core tree formatted; parity, fixed point, build, tests
- TODO (lead) `make format` once review-core-numeric is done

## Lessons
* Another agent renames files and edits headers while you run; a
  `--check` FAIL on a file you did not touch is usually a concurrent edit:
  rerun it before investigating.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ A tree-wide pass (brace initialisers): it changed code owned by the translation lanes without changing what the program decides, so it has no program flow of its own. The flows as the code now stands are in the lane sections: [exec-sched](exec-sched.md#program-flow-and-decisions), [core-numeric](core-numeric.md#program-flow-and-decisions), [state-flags](state-flags.md#program-flow-and-decisions), [nav-conic](nav-conic.md#program-flow-and-decisions), [nav-integ](nav-integ.md#program-flow-and-decisions), [nav-aux](nav-aux.md#program-flow-and-decisions), [io-pinball](io-pinball.md#program-flow-and-decisions), [io-nouns](io-nouns.md#program-flow-and-decisions), [imu](imu.md#program-flow-and-decisions), [guid-powered](guid-powered.md#program-flow-and-decisions), [guid-landing](guid-landing.md#program-flow-and-decisions), [guid-entry](guid-entry.md#program-flow-and-decisions), [p37](p37.md#program-flow-and-decisions), [dap-lm](dap-lm.md#program-flow-and-decisions), [dap-csm](dap-csm.md#program-flow-and-decisions), [prog-rendz](prog-rendz.md#program-flow-and-decisions), [prog-align](prog-align.md#program-flow-and-decisions), [sys-misc](sys-misc.md#program-flow-and-decisions), [sim](sim.md#program-flow-and-decisions); the system view is [docs/overview/README.md](../overview/README.md) and its flowcharts.
