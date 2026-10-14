# Assignment: format2   (tier: translation -- tooling + judgment)

You own the whole tree; nothing else runs. The user found the formatted code
unreadable for two reasons (see docs/CODING_STANDARD.md "Comment placement
and wrapping"): comments inside expressions, and parenthesis-aligned
continuation lines. Examples to use as acceptance tests (before/after in
your note): src/programs/targeting/*.c calcthet (`agc_dp_t th = /* theta/2,
rev 2^0 */ agc_dp_div(...)`, `/* UP */` inside the unit() argument list),
src/guidance/burn_steering.c or wherever agc_guid_calcut lives
(`agc_vec_t gv = /* g, m/cs^2 2^-5 */ agc_vec_div(ip, agc_vec_shl(...),\n
CS200_B12);`), and the `agc_dp_t rad = /* (F/m)^2 - Q^2, 2^-10 */
agc_dp_sub(ip,` line.

1. tools/fix_comments.py (libclang token stream, python3 clang.cindex):
   for every C file, find block/line comments whose position is inside a
   statement: after `=` before the initializer, inside `( ... )` of a call
   or declaration initializer, between `,`-separated arguments, between
   binary operands, or on a continuation line of a multi-line statement.
   Move each to a comment line immediately ABOVE the statement at the
   statement's indentation; several from one statement are merged into one
   comment, in order, separated by "; ". Trailing comments after the final
   `;` of a one-line statement stay. Preserve the text verbatim. Function
   header comments, `#define` trailing comments, struct field comments and
   `case` comments are untouched. Idempotent. Verify: `gcc -E` output with
   comments stripped is token-identical before/after for every file.
2. uncrustify.cfg: continuation indent fixed at 8 (indent_continue = 8,
   indent_align_paren = false, indent_paren_nl / indent_paren_close as
   needed, align_func_params = false, nl_func_call_args / nl_func_decl_args
   = ignore, no alignment to the open paren anywhere; keep the existing
   paragraphing and Allman rules; keep `return (x)`). Make sure a nested
   call that wraps shows each argument start at +8 from the statement, not
   under the paren. Verify idempotence (3 passes) and token parity.
3. Run both tree-wide (`make format` must run fix_comments.py then
   Uncrustify twice; add the script to the Makefile target), then `make`,
   `make test`, `make check` (0 hard), oracle tests unchanged.
4. Count the remaining statements with more than two continuation lines
   (docs/overview/long_statements.tsv: file:line, lines) so the review wave
   can split them with temporaries; do not split them yourself.
5. docs/modules/format2.md with before/after for the three user examples,
   counts (comments moved, files changed), Lessons. Update docs/STYLE.md.
