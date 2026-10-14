# Assignment: style   (tier: translation -- read the whole guide, judge each rule)

Source: docs/reference/NASA-SEL-94-003-C-Style-Guide.txt (text of the PDF
beside it; 100 pages, ~3500 lines). READ ALL OF IT, every chapter, before
writing anything. The lead skimmed it and was told not to; do not repeat
that.

Deliverables:
1. docs/STYLE.md -- every rule in the guide, one row each, in the guide's
   order, with columns: section | rule (one line, your words) | how enforced:
   `uncrustify` / `clang-format` (give the option), `check_rules` (describe the check),
   `review` (manual), or `n/a` (explain: e.g. K&R-era argument declarations,
   PDL, malloc rules that Power of Ten already forbids) | conflict with
   docs/CODING_STANDARD.md or docs/NAMING.md, if any, and which wins
   (Power of Ten and the user's naming rule win; note the conflict).
   Include chapters 3 and 4 (file organisation, README, header files, file
   prolog, include order, defines/typedefs, externs, function sequence),
   6.1-6.6 (names, constants, declarations, casts, pointers), 7.4 (error
   handling) and 8 (portability/performance), not only the formatting
   chapters.
2. .clang-format at the repo root encoding every rule marked
   `clang-format`: Braces-Stand-Alone (Allman) everywhere; 4-space indent,
   no tabs; the spacing rules of 6.7 (no space before call parens, space
   after keywords, space after casts, spaces around binary and assignment
   operators, one space after commas/semicolons); break AFTER logical
   operators in long conditions; `for` split over three lines; switch
   cases indented one level inside the braces; aligned consecutive
   declarations and trailing comments; column limit per the guide (find it;
   if the guide is silent, use 80 and say so); include ordering per 4.3;
   one declaration per line. Verify each option exists in clang-format 14
   (`clang-format --dump-config` once it is installed; until then write it
   for version 14 and say which options you could not verify).
3. tools/check_style.py -- the `check_rules` rows that are cheap to check
   mechanically (e.g. parentheses around return expressions, `== 0`
   instead of `!x` on numeric tests, one statement per line, a comment on
   each internal variable declaration, file prolog present, function prolog
   present with the fields the guide lists adapted to our header-comment
   convention -- keep the AGC label/page line). Report, do not edit.
4. Templates: docs/templates/file_prolog.c and function_prolog.c showing the
   boxed prolog format from 2.3/4.1/5.1 adapted to this project (project
   name, AGC source file and pages, purpose, scale factors of globals).
5. Makefile targets: `make format` (clang-format -i over src include tests),
   `make format-check` (--dry-run --Werror), `make style` (check_style.py);
   add format-check and style to `make check`. If clang-format is not
   installed, say so in your report; do not run it.
Do not reformat any code in this task. Write docs/modules/style.md with a
Lessons section. Report <= 15 lines.
