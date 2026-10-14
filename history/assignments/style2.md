# Assignment: style2 -- port the formatter to Uncrustify   (tier: translation)

The user wants statement paragraphing (NASA SEL-94-003 section 5.5 and the
blank-line guidance in 2.2.1) applied automatically. clang-format cannot
insert blank lines; Uncrustify can. Replace clang-format with Uncrustify as
the project's single formatter. Do not layer the two.

The status-handling agent is editing src/ concurrently: do NOT reformat the
tree. Work on a scratch copy (SCRATCH/style2/) for all experiments.

1. Read docs/STYLE.md (every row marked `clang-format`) and .clang-format;
   that is the spec. Read docs/reference/NASA-SEL-94-003-C-Style-Guide.txt
   sections 2.2.1, 5.4, 5.5, 7.1-7.3 again for the blank-line rules.
2. Write uncrustify.cfg at the repo root reproducing every clang-format
   rule (Allman braces everywhere, 4-space indent, no tabs, 80 columns,
   the 6.7 spacing rules, space after cast, space after keyword, no space
   before call parens, `case` indented one level inside the switch braces,
   3-line `for` split, break after `&&`/`||`, aligned trailing comments and
   consecutive declarations, `return (x)` kept, one statement per line)
   PLUS paragraphing:
     - blank line after the block of local declarations (nl_func_var_def_blk)
     - blank line before `if`, `for`, `while`, `do`, `switch` statements
       when preceded by another statement (nl_before_if etc.), but NOT
       between a leading comment and its statement, and not after `{`
     - blank line after a closing brace that ends an if/for/while/switch
       block when followed by another statement (nl_after_if/for/...),
       none between `}` and `else`
     - blank line before `return` when preceded by a statement
       (nl_before_return)
     - a run of consecutive `if (!AGC_ASSERT(...)) { return ...; }` guards
       at the top of a function: one paragraph (blank after the run, not
       between them) -- if Uncrustify cannot express that exactly, choose
       the closest and document the difference
     - at most 1 consecutive blank line (nl_max), none after `{` or before
       `}` (eat_blanks_after_open_brace, eat_blanks_before_close_brace)
   Set the identifier-related options so Uncrustify never changes code
   tokens. Document every option you set with the guide section it serves.
3. Validate on the scratch copy: run on all of src include tests; all
   .c files must still compile with the brief's flags (-c); a second run
   must change nothing (idempotent); `git diff --stat` on the scratch copy
   should show formatting only (no token changes: compare `gcc -E` output
   with whitespace stripped before/after on a sample of 20 files).
   Show before/after for src/csm/dap/tvc/exec.c csm_dap_dpadd (the user's
   example) in your note.
4. Makefile: `format` = `uncrustify -c uncrustify.cfg --replace --no-backup`
   over src include tests (use -q); `format-check` = `--check`. Remove
   clang-format from the Makefile and delete .clang-format once
   uncrustify.cfg is proven equivalent-or-better (keep it in git history
   only). Update docs/STYLE.md enforcement column (`clang-format` ->
   `uncrustify`), docs/CODING_STANDARD.md Tooling, README.
5. Report <= 12 lines: option count, rules you could not express, compile/
   idempotence results. Do not reformat the real tree; the lead runs
   `make format` after status-handling finishes.
