# Assignment: format3   (tier: mechanical)   -- next free slot

User example: src/csm/burns/return_to_earth/conic.c lines ~103-114, the
rte_kfit_t initializers `{2556848, 0,\n 189866};` wrapped mid-brace.

Extend tools/fix_comments.py's join step to brace initializers: an
initializer list `{ ... }` is joined onto one line unless it already has
the one-element-per-line shape (each element on its own line, aligned
under the first, at least two continuation lines) -- that shape is kept
(tables). Nested braces (array of structs, one struct per line) count as
one element per line. Then the paragraph alignment step aligns the
trailing comments of the joined declarations with the rest of their
paragraph. Run tools/format.sh tree-wide, confirm idempotence and `gcc -E`
token parity, `make`, `make test`. Show the user's example before/after in
docs/modules/format3.md. Report <= 6 lines.
