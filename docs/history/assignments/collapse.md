# Assignment: collapse   (tier: translation)

Runs after `integrate`, before sim/oracle-cases/review. Goal: remove
single-use helper functions that exist only as line-count padding, so a
reader follows one function instead of hopping through four.

1. **Call graph from tools, not from reading.** Add a Makefile target
   `callgraph` that compiles every src/ TU with
   `-fcallgraph-info=su,da` (GCC 12 writes one <name>.ci VCG file per TU
   next to the object, in build/) and have tools/callgraph.py parse the
   .ci files: nodes = functions (with static/extern from the node
   attributes), edges = call sites with counts. Cross-check with
   `cflow -b --omit-arguments -i s src/**/*.c` for extern calls across
   TUs. Use libclang (python3 `clang.cindex`, package python3-clang-14)
   for exact function source extents, call-site offsets, and the AST of
   every `if (!AGC_ASSERT(cond))` guard condition (so duplicate guards are
   compared structurally, not textually). Mark functions referenced from
   src/exec/dispatch.c (job/task entries), from tests/, and from public
   headers.
   Output docs/overview/data/callgraph.tsv: callee | callers | sites | lines |
   static? | agc_label? (true if the function's header comment names an
   AGC label or page). Also emit a Graphviz .dot (handy for the document
   wave).
2. **Candidates:** functions with exactly one caller, not a dispatch entry,
   not referenced by tests, not declared in a public header, and whose
   header comment does NOT carry its own AGC label (helpers named after
   an AGC label stay: the label is the cross-reference). Sort by lines.
3. **Collapse mechanically where the shape allows.** tools/inline_one.py
   takes (helper, caller) and, using the libclang extents, splices the
   helper body in place of the call when: the call is a full statement,
   each argument is the same identifier as the matching parameter (or a
   constant), the helper has a single trailing return, and no local name
   collides. It de-duplicates guards per step below, re-indents, deletes
   the helper definition and prototype, writes the file, compiles it, and
   reverts on failure. The agent handles only the candidates the script
   refuses (renamed parameters, early returns, value results used in an
   expression), and reviews the script's diffs for readability.
   Produce the candidate list (docs/overview/data/collapse_candidates.tsv) and
   STOP for the lead's review before applying, as the naming lane did.
   **Collapse** each candidate into its caller when the caller stays
   <= 60 code lines after inlining (brace-only lines don't count) and the
   result reads better; otherwise leave it and note why.
   **Guard de-duplication (user direction):** before counting, remove from
   the helper every `if (!AGC_ASSERT(cond)) { return ...; }` guard whose
   condition the caller has already established on the same object at
   that point (same parameter range, same non-NULL pointer, same enum
   range, same state flag). One check per fact per function. The merged
   function's length is measured AFTER that de-duplication, so identical
   guards repeated across a family of helpers never block a collapse.
   Guards that check something new stay and count. Where several
   candidates share one identical guard preamble, the preamble appears
   once at the top of the merged function. Preserve the
   helper's comments as a paragraph comment at the inlined site. Where a
   caller had been split 3-4 ways purely for length, prefer re-splitting
   it along the AGC's own labels/paragraphs rather than by arbitrary
   count, and name the pieces per docs/NAMING.md. Keep assertion density.
4. After each file: compile with the Makefile flags, `make format`,
   `make check` 0 hard. At the end: `make`, `make test`, oracle tests
   unchanged. Report counts (candidates, collapsed, kept-with-reason) and
   the 10 biggest readability wins in docs/history/waves/collapse.md with a
   Lessons section. Add `make callgraph` to regenerate the tsv/dot.

Scope boundary (parallel run): do not touch src/sim/ or tests/; the sim and
oracle-cases lanes own those. Public signatures in include/agc/ must not
change (other lanes link against them).

## Refined protection rule (user, 2026-10-11; example js_acy / js_bdz)
A single-caller helper is protected by its AGC label ONLY if that label is
entered from more than one place in the rope: a subroutine called with TC,
or a jump target with >= 2 references (grep the .agc files for `TC`, `TCF`,
`BZF`, `BZMF`, `CCS`/`INDEX`-table uses, `ADRES`/`CADR`/`2CADR`). A label
with 0 references (fall-through paragraph) or 1 protects nothing: inline
the helper and keep the label as a comment at that spot. ACYCHECK and
BDZCHECK (0 refs) -> inline; NOACY (3 refs, a shared tail) may stay a
helper if it is called from >= 2 C sites.
