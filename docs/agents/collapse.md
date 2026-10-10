# Assignment: collapse   (tier: translation)

Runs after `integrate`, before sim/oracle-cases/review. Goal: remove
single-use helper functions that exist only as line-count padding, so a
reader follows one function instead of hopping through four.

1. **Call graph.** Build tools/callgraph.py: definitions from `ctags -R
   --c-kinds=f --fields=+n -x src`, call sites by scanning each function
   body for `identifier(` against the definition set (ignore names inside
   comments and strings; count a call once per site). Also mark functions
   referenced from src/exec/dispatch.c (job/task entries) and from tests.
   Output docs/overview/callgraph.tsv: callee | callers | sites | lines |
   static? | agc_label? (true if the function's header comment names an
   AGC label or page). Also emit a Graphviz .dot (handy for the document
   wave).
2. **Candidates:** functions with exactly one caller, not a dispatch entry,
   not referenced by tests, not declared in a public header, and whose
   header comment does NOT carry its own AGC label (helpers named after
   an AGC label stay: the label is the cross-reference). Sort by lines.
3. **Collapse** each candidate into its caller when the caller stays
   <= 60 code lines after inlining (brace-only lines don't count) and the
   result reads better; otherwise leave it and note why. Preserve the
   helper's comments as a paragraph comment at the inlined site. Where a
   caller had been split 3-4 ways purely for length, prefer re-splitting
   it along the AGC's own labels/paragraphs rather than by arbitrary
   count, and name the pieces per docs/NAMING.md. Keep assertion density.
4. After each file: compile with the Makefile flags, `make format`,
   `make check` 0 hard. At the end: `make`, `make test`, oracle tests
   unchanged. Report counts (candidates, collapsed, kept-with-reason) and
   the 10 biggest readability wins in docs/modules/collapse.md with a
   Lessons section. Add `make callgraph` to regenerate the tsv/dot.
