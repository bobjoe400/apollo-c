# CODING_STANDARD.md -- NASA/JPL "Power of Ten" applied to apollo-c

Source: G. J. Holzmann, "The Power of Ten -- Rules for Developing Safety
Critical Code", NASA/JPL LaRS (the PDF in bigbadcyborg/NASA-coding-guidelines).

Language: C11, no extensions. Build flags (Makefile):
`-std=c11 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow
-Wstrict-prototypes -Wmissing-prototypes -Wundef -Werror`.

## The ten rules, as they apply here

1. **Simple control flow.** No `goto`, `setjmp`/`longjmp`, no direct or
   indirect recursion. AGC code is full of jumps between labels; translate
   them into `if`/`else`, loops, early `return`, or small helper functions.
   AGC routines that "fall into" each other become sequential calls.
2. **Bounded loops.** Every loop has a compile-time constant bound. Iterative
   solvers (Kepler, Lambert, UNIT normalisation) use the AGC's own iteration
   counters as the bound (e.g. `AGC_KEPLER_MAX_ITER`). The scheduler's
   run-forever loop is the single documented exception and lives in `src/sim/`.
3. **No heap.** No `malloc`/`calloc`/`realloc`/`free`/`strdup`. All erasable
   memory is static storage declared in `src/core/state.c` or in the
   owning module's `.c` file, mirroring the AGC's fixed erasable banks.
4. **Short functions.** <= 60 lines of code per function (blank lines and
   comments excluded). `tools/check_rules.py` enforces this. Split AGC
   routines at their natural labels, never by count.
   **One exemption:** the generated dispatch tables in src/exec/dispatch.c
   (one `switch` per vehicle per kind: CSM jobs, CSM tasks, LM jobs, LM
   tasks). They are the C form of the AGC's 2CADR tables; the unit a
   reviewer verifies is tools/registry.tsv plus tools/gen_registry.py, not
   the switch. A file claims the exemption only with the marker
   `/* GENERATED-DISPATCH: rule 4 exempt */` in its first 5 lines, and
   check_rules.py accepts the marker only for src/exec/dispatch.c.
5. **Assertions.** Average >= 2 assertions per function, measured as the
   rule states: an AVERAGE over the project, reported by check_rules as a
   hard target; the per-file figure is a warning only. A file of pure
   fixed-point arithmetic whose preconditions are guaranteed by
   construction may sit below 2.0, but it must carry the real
   postconditions a bug could break (overflow flag consistent, iteration
   count within its bound, |unit| within bounds, sign agreement after
   NORM). Never add an assertion that cannot fail to raise the number.
   Use `AGC_ASSERT(cond)`
   from `include/agc/core/assert.h`: it is side-effect free, evaluates to a
   `bool`, and callers recover by returning `AGC_STATUS_*` error codes. Never
   use `<assert.h>`. Assertions that cannot fail do not count.
6. **Smallest scope.** Declare variables where used. Module-private state is
   `static` in its `.c` file. Shared AGC erasables are accessed through the
   accessor API in the core headers, not via `extern` globals.
7. **Check returns, validate params.** Every non-void call's return is
   used. **Casting an `agc_status_t` to `(void)` is forbidden** in src/
   (tools/check_rules.py reports it as a hard violation). Two consequences:
   * If an operation cannot fail when called correctly (setting a flag by
     enum, OR-ing a constant mask into a channel, a phase change with
     constant codes, clearing OVFIND), it returns `void` and validates its
     arguments with `AGC_ASSERT` internally. Do not return a status nobody
     can act on.
   * If an operation can fail (waitlist full, no VAC area, no core set,
     bad erasable address, no convergence), the caller does what the AGC
     does at that site: raise the AGC's alarm/bailout/poodoo, or propagate
     the status to its own caller. Read the AGC source for the failure
     path; it is usually explicit (TC ABORT / OCT 1203).
   `(void)` on a standard-library call (fprintf in the sim) is still
   allowed with a comment.
   * **No silent failures (user direction, 2026-10-11).** A precondition
     is one of two things. Either it is guaranteed by construction (a
     static helper only ever called with `&local`, an enum straight out of
     a `switch`, an index from a bounded loop), and then there is no guard
     at all: deleting it is the fix, and the function may be `void` or
     return a value. Or it can fail, and then the function returns
     `agc_status_t`, delivers its value through an out-parameter, and every
     caller checks the status. A `void` function whose assertion fails and
     silently `return`s, or a value function that returns a sentinel such
     as `0` on a failed assertion, hides the failure from everyone and is a
     hard violation (tools/check_rules.py). Making a function `void` is
     never the way to remove a cast. Every function validates its inputs (pointer
   non-null, enum in range, scaled values in range) with `AGC_ASSERT`.
8. **Limited preprocessor.** Headers: include guards. Macros: only object-like
   constants and `AGC_ASSERT`. No token pasting, no variadics, no recursive
   macros, no function-like macros that hide control flow. Conditional
   compilation only for the include guard and the single `AGC_HOST_DEBUG`
   switch in agc_assert.h.
9. **Limited pointers.** At most one level of dereference. No function
   pointers: the executive dispatches jobs/tasks through an `enum` + `switch`
   (`src/exec/dispatch.c`). No pointer arithmetic beyond array indexing. No
   pointers hidden in typedefs.
10. **Zero warnings.** The pedantic flag set above, with `-Werror`. Also run
    `cppcheck`/`clang-tidy` when available (`make lint`).

## Project conventions

* **No program/routine/page numbers in identifiers.** P30, R60, V37, page
  numbers go in comments only. Use the canonical descriptive names in
  docs/NAMING.md for files, functions, types, enums and job ids.

* Names: `agc_<module>_<verb>()`. Types end in `_t`. Constants `AGC_UPPER`.
* Every translated function has a header comment giving the AGC label,
  source file, page number(s), and a one-line description, e.g.

      /* KEPLER (CONIC_SUBROUTINES.agc, pp. 1263-1265, label KEPLER) */

* Keep the AGC's own comments where they explain intent; translate the
  all-caps prose into a readable sentence case.
* Scale factors: every DP/SP variable carries a comment `scale 2^N unit`,
  and the struct definitions in headers repeat them. Do not silently rescale.
* Status returns: `agc_status_t` from `agc_status.h`. AGC `ALARM`/`BAILOUT`/
  `POODOO` become `agc_alarm(code)` / `agc_bailout(code)` calls followed by an
  error return; the octal alarm codes are preserved (e.g. `AGC_ALARM_01201`).
* No `printf` in library code; the sim harness owns all I/O.

## Tooling

* `make format` (tools/format.sh) is the only formatter: tools/fix_comments.py
  (comment placement, wraps, paragraphs), Uncrustify 0.72 twice
  (`uncrustify.cfg`, NASA SEL-94-003 layout), tools/fix_comments.py again
  (paragraph column alignment). It rewrites src include tests and changes
  only white space, line breaks and comment positions; `make format-check`
  runs it on copies and fails on any difference (docs/STYLE.md).
* `spatch` (Coccinelle) for any tree-wide rename: generate a semantic patch,
  never sed identifiers.
* `ctags -R --c-kinds=+px -x` for symbol inventories.
* `clangd` (with `bear -- make` or a generated compile_commands.json) for
  cross-reference checks once the tree compiles.
* `gcc -fcallgraph-info=su,da` for the authoritative per-TU call graph
  (nodes carry static/extern; edges carry call-site line:col); `cflow` for
  cross-TU edges; `python3-clang` (libclang) for function extents and AST
  comparisons. Never derive a call graph by reading or grepping.

## Comment placement and wrapping (user direction, 2026-10-11)

* A comment never sits inside an expression: not between `=` and the
  initializer, not inside an argument list, not between operands. Scale and
  meaning notes for a statement go trailing after the `;` (the statement
  is on one line), or on their own line ABOVE the statement (block style,
  same indentation) when they introduce a group. `tools/fix_comments.py`
  merges every comment inside a statement of a function body into the
  statement's one trailing comment, in source order, separated by `; `
  (struct/enum bodies, initialiser tables, `#define` lines and `case`
  labels are left alone). A trailing comment is one line.
* **No column limit. One statement per line.** The NASA guide sets no line
  length; the 80-column limit is withdrawn. Nested-call chains stay on one
  line; the formatter joins wrapped statements back onto one line.
  (Reference: the user's hand-formatted agc_guid_calcut.)
* Within a paragraph (consecutive non-blank lines), at one brace level,
  one-line statements line up as columns: declaration type, name (the `*`
  dangles left of the name), assignment operator, expression, trailing
  comment; the comment column is one space past the longest code line of
  the paragraph (not a fixed column). A comment after `{` is not aligned.
* A standalone comment that introduces a group of statements or
  declarations begins a paragraph: a blank line precedes it (after a line
  ending in `;`, `}`, a table row's `,` or a preprocessor line; never after
  `{`), at file scope and in functions alike. Consecutive comment lines
  stay together. The declarations that open a function body are followed
  by a blank line; later declarations may share a paragraph with the
  statements that use them.
* Wrapping is allowed in exactly one shape: the OUTERMOST call of a
  statement may have its arguments one per line, each aligned under the
  first argument, with the statement's trailing comment on the first line
  (the CALCGRAV call). A wrap is never placed inside a nested call: an
  inner call stays whole on whichever line it is on. The formatter keeps
  outer-call wraps that already have this shape -- every break after a
  comma of that call, at least two continuation lines, one argument on
  each continuation line, at most two on the first line of a call (`ip`
  and the first real argument) or one on the first line of a function
  definition/prototype -- joins every other wrap back onto one line, and
  never introduces wraps itself.
* Brace initializers follow the same rule as call arguments: entirely on
  one line, or one element per line aligned under the first element (tables
  with one row per line). Never a partial wrap inside the braces.
* A statement that is unreadable on one line because of nesting is split
  with named temporaries (review item), not wrapped.
  docs/overview/long_statements.tsv lists the candidates (wider than 120
  columns or kept on more than three lines).
