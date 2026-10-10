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
   memory is static storage declared in `src/core/agc_state.c` or in the
   owning module's `.c` file, mirroring the AGC's fixed erasable banks.
4. **Short functions.** <= 60 lines of code per function (blank lines and
   comments excluded). `tools/check_rules.py` enforces this. Split AGC
   routines at their natural labels.
5. **Assertions.** Average >= 2 assertions per function. Use `AGC_ASSERT(cond)`
   from `include/agc/agc_assert.h`: it is side-effect free, evaluates to a
   `bool`, and callers recover by returning `AGC_STATUS_*` error codes. Never
   use `<assert.h>`. Assertions that cannot fail do not count.
6. **Smallest scope.** Declare variables where used. Module-private state is
   `static` in its `.c` file. Shared AGC erasables are accessed through the
   accessor API in the core headers, not via `extern` globals.
7. **Check returns, validate params.** Every non-void call's return is used
   or explicitly `(void)`-cast with a comment. Every function validates its
   inputs (pointer non-null, enum in range, scaled values in range) with
   `AGC_ASSERT`.
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

* `spatch` (Coccinelle) for any tree-wide rename: generate a semantic patch,
  never sed identifiers.
* `ctags -R --c-kinds=+px -x` for symbol inventories.
* `clangd` (with `bear -- make` or a generated compile_commands.json) for
  cross-reference checks once the tree compiles.
