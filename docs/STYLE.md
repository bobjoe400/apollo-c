# STYLE.md -- NASA SEL-94-003 "C Style Guide" (GSFC, Aug 1994) applied to apollo-c

Source: docs/reference/NASA-SEL-94-003-C-Style-Guide.{pdf,txt}. Every rule of
the guide is listed once, in the guide's order. The text extraction drops
leading whitespace, so the layout rules (case indentation, continuation
indent, brace placement) were read from the PDF's own text-positioning
operators. All chapters were read; chapter 1 and the bibliography carry no
rules.

**Precedence.** docs/CODING_STANDARD.md (Power of Ten, "CS" below) and
docs/NAMING.md win over this guide. Where they disagree the row says so, and
the guide's rule is recorded as overridden, not silently dropped.

**Enforcement column**

* `clang-format` -- `.clang-format` (clang-format 14.0.6, every option
  verified with `--dump-config` and a probe file); `make format` applies,
  `make format-check` fails on any difference.
* `check_rules` -- `tools/check_style.py`, rule id in brackets; `make style`.
  (Power of Ten checks stay in `tools/check_rules.py`.)
* `compiler` -- the project's gcc flag set with `-Werror` already rejects it.
* `review` -- a person (or the lead) judges it; not mechanically checkable at
  reasonable cost.
* `n/a` -- does not apply here; the reason is given.

Templates: docs/templates/file_prolog.c, docs/templates/header_prolog.h,
docs/templates/function_prolog.c (all clang-format clean and compiling with
the full warning set).

## 2 Readability and maintainability

| § | Rule | How enforced | Conflict / note |
|---|------|--------------|-----------------|
| 2 | Follow ANSI C. | compiler: `-std=c11 -Wpedantic` | |
| 2.1 | Encapsulate one idea per header; group data and functions into files, related functions together. | review | Matches docs/ARCHITECTURE.md module layout. |
| 2.1 | Hide information: include a header only where needed; make external variables visible only where declared. | review; CS rule 6 | CS goes further: shared erasables only through accessors, module state `static`. |
| 2.2 | Use white space (blank lines, spaces, indentation) for readability. | clang-format | |
| 2.2.1 | A single blank line separates program parts; do not overuse blank lines. | clang-format `MaxEmptyLinesToKeep: 1`, `KeepEmptyLinesAtTheStartOfBlocks: false`, `SeparateDefinitionBlocks: Always` | |
| 2.2.2 | Space around binary operators (avoids `*total/*count` opening a comment); one space after a comma. | clang-format (built-in operator spacing) | |
| 2.2.3 | Indent four spaces. | clang-format `IndentWidth: 4`, `UseTab: Never`, `TabWidth: 4` | |
| 2.3 | Comment at program (README), file (prolog), function (prolog) and data-declaration level. | check_rules [file-prolog], [function-prolog], [decl-comment]; README: review | |
| 2.3 | Comments add information; do not paraphrase the code or repeat the design. | review | CS: keep the AGC's own comments that explain intent. |
| 2.3 | Boxed comments for prologs and section separators. | check_rules [function-separator]; templates | Box drawn without a right-hand border (the guide's own 9.2 example does this); avoids re-padding on every edit. |
| 2.3 | Block comments (`/*`, ` * text`, ` */`, full sentences) introduce major sections. | review | |
| 2.3 | Short comments on the same line as the code/data they describe. | check_rules [decl-comment] for declarations; review otherwise | |
| 2.3 | Inline comments at the indentation of the code they describe. | clang-format (re-indents comments) | |
| 2.3 | Several short comments in a block start in one column (and end in one). | clang-format `AlignTrailingComments: true` (start column); end column: review | clang-format 14 aligns the start only; right-edge `*/` alignment is not maintained. `SpacesBeforeTrailingComments` affects only `//` comments in v14, so `/* */` gets one space minimum. |
| 2.3 | Use short comments for variable definitions, block comments for computations. | review | |
| 2.4 | Precise names, used consistently throughout the program. | review | NAMING.md rule 7: one name per concept tree-wide. |
| 2.4 | A uniform abbreviation/prefix scheme (e.g. `dr_` for one subsystem). | review | CS: `agc_<module>_<verb>()`. |
| 2.4 | Avoid abbreviations with unintended meanings (`inch`). | review | |
| 2.4 | Underscores separate words in names. | review | Consistent with CS. |
| 2.4 | Names unique within the significant-character limit. | compiler (C11 guarantees 31 significant external chars; gcc uses all) | |
| 2.4 | Long names for clarity, but not so long that indentation breaks. | review | NAMING.md rule 4: 2-4 words after the prefix. |
| 2.4 | Names longer than four characters differ by at least two characters. | review | NAMING.md rule 5 keeps AGC labels (KEPC1/KEPC2, R1/R2 erasables); those win. Mechanical check would flag most AGC erasable pairs. |
| 2.4 | Names unique irrespective of case. | check_rules [case-unique] | NAMING.md rule 7 requires `AGC_JOB_X` (enum) and `agc_job_x` (function) to share a stem; all-upper vs all-lower pairs are exempt, mixed-case variants are reported. |
| 2.4 | No variable and typedef/struct with the same name. | review | The `_t` suffix (CS) makes clashes rare. |
| 2.4.1 | Standard short names `c`, `i j k`, `n`, `p q`, `s` are acceptable; longer names preferred. | review | |
| 2.4.1 | Standard suffixes `_ptr`, `_file`, `_fd`. | review | Not adopted as mandatory; the project has no file I/O in the library. |
| 2.4.2 | Locals never reuse a global's name (hidden variables). | compiler `-Wshadow` | |
| 2.4.2 | Same name in different functions only for the same meaning. | review | |
| 2.4.3 | Variables: lower case words with underscores. | check_rules [variable-case] | |
| 2.4.3 | Function names: capitalised words, no underscores (`ProcessError`). | n/a -- overridden | CS (`agc_<module>_<verb>()`) and NAMING.md win. The guide contradicts itself: 5.1 asks for lower-case names with underscores. |
| 2.4.3 | Constants: upper-case words with underscores. | check_rules [constant-case] (macros, enum constants) | |
| 2.4.3 | C bindings: `c_` prefix. | n/a | No foreign-language bindings in the project. |
| 2.4.4 | typedef names follow the global-variable rule (lower case). | review | CS adds the `_t` suffix. |
| 2.4.4 | enum constants and `const` constants follow the constant rule (upper case). | check_rules [constant-case] for enums; review for `const` | Local `const` values computed at run time (`const agc_dp_t p0 = ...`) are variables, not constants, and stay lower case. |

## 3 Program organization

| § | Rule | How enforced | Conflict / note |
|---|------|--------------|-----------------|
| 3.1 | A README documents the program. | review | README.md exists. |
| 3.1 | `main` grouped with related functions in a program file; other functions in module files. | review | Library has no `main`; tests and the sim harness do. |
| 3.1 | Header files encapsulate related definitions and declarations. | review | |
| 3.1 | Write a Makefile. | review | Makefile exists. |
| 3.2 | README lists all conditional-compilation flags, machine-dependent files, paths to reused components. | review | **Finding:** README.md does not list `AGC_HOST_DEBUG` (agc/core/assert.h) or the reused yaAGC/yaYUL tree under tools/oracle. |
| 3.3 | Include only the standard libraries a file needs. | review | CS: no `<stdio.h>` in library code. |
| 3.4 | `#include <...>` for system headers, `#include "..."` for project headers. | clang-format `IncludeCategories` (groups by form); review for the choice of form | |
| 3.4 | Headers hold the data definitions, declarations, typedefs and enums shared by more than one file. | review | |
| 3.4 | Organise headers by function; one subsystem per header. | review | include/agc/<subsystem>/. |
| 3.4 | Platform-dependent declarations in a separate header. | review | agc/core/word.h holds the fixed-width numeric model. |
| 3.4 | No private header named like a library header. | check_rules [header-name] | **Finding:** include/agc/core/assert.h shares its basename with `<assert.h>` (harmless here because it is always included as "agc/core/assert.h", but the guide forbids it). |
| 3.4 | Include the declaring header in the file that defines the function/variable. | compiler `-Wmissing-prototypes` (a definition without a visible prototype fails) | |
| 3.4 | Do not nest headers; each program file includes every header it needs. | n/a -- overridden | Project headers are self-contained (include what they use, include guards per CS rule 8); nesting is required for that. |
| 3.4 | A header's prolog says which other headers must be included first. | check_rules [file-prolog-fields] (`REQUIRES` field) | |
| 3.5 | Module files hold related functions, constants, types and data, without `main`. | review | |
| 3.6 | The Makefile lists the files of the program, comments library membership, and states dependencies. | review | Files found by `find`; dependencies by `-MMD -MP`. |
| 3.7 | Standard suffixes (`.c`, `.h`, `.s`, `.o`, ...); unique base names. | review | `Makefile` is named the GNU-make way, not `.mak`. |

## 4 File organization

| § | Rule | How enforced | Conflict / note |
|---|------|--------------|-----------------|
| 4 | File order: prolog; usage (program files); includes; defines/typedefs; external data; functions; further external data; further functions (Figure 4). | review; parts by the rows below | |
| 4.1 | Every file has a prolog. | check_rules [file-prolog] | |
| 4.1 | Prolog fields: FILE NAME, PURPOSE, FILE REFERENCES, EXTERNAL VARIABLES (source, name, type, I/O, description), EXTERNAL REFERENCES, ABNORMAL TERMINATION / ERROR AND WARNING MESSAGES, ASSUMPTIONS/CONSTRAINTS/RESTRICTIONS, NOTES, REQUIREMENTS REFERENCES, DEVELOPMENT HISTORY, ALGORITHM (PDL); "none" when empty. | check_rules [file-prolog-fields]: FILE NAME (must match the file), PURPOSE, AGC SOURCE, EXTERNAL VARIABLES, EXTERNAL REFERENCES, ABNORMAL TERMINATION | Adapted (docs/templates/file_prolog.c): REQUIREMENTS REFERENCES becomes **AGC SOURCE** (rope, .agc file, pages, labels -- the requirement we trace to); EXTERNAL VARIABLES gains a **Scale** column (CS scale-factor rule); ABNORMAL TERMINATION lists AGC alarm codes and `agc_status_t` returns; FILE REFERENCES dropped (no file I/O in the library); DEVELOPMENT HISTORY dropped (git log); ALGORITHM kept as prose (no PDL, see 4.2). |
| 4.1 | Header files carry the abbreviated prolog: NAME, PURPOSE, GLOBAL VARIABLES, DEVELOPMENT HISTORY (Figure 6). | check_rules [file-prolog-fields]: FILE NAME, PURPOSE, AGC SOURCE, REQUIRES, GLOBAL VARIABLES | docs/templates/header_prolog.h; history is git. |
| 4.2 | The prolog describes the algorithm in the SEL PDL. | n/a | The AGC listing cited in AGC SOURCE is the design; the prolog's ALGORITHM field summarises it in prose. |
| 4.2 | PDL indents 4, aligns control keywords, continuation one column right. | n/a | No PDL. |
| 4.2.1 | PDL sequence statements: imperative verb + object; CALL; RETURN. | n/a | No PDL. |
| 4.2.2 | PDL IF THEN ELSE / IF THEN / DO CASE map to `if`/`else`/`switch`. | n/a | No PDL; the C forms are covered by 7.2. |
| 4.2.3 | PDL DO WHILE / DO FOR / DO UNTIL map to `while`/`for`/`do`. | n/a | No PDL; C forms in 7.3. |
| 4.2.4.1 | ABORT: jump to an error block at the end of the routine (`goto`). | n/a -- overridden | CS rule 1 forbids `goto` (tools/check_rules.py). Use structured error flags or an early `return` of an `agc_status_t`. |
| 4.2.4.2 | UNDO / `break`: avoid if you can. | review | Bounded loops (CS rule 2) often exit with `break` at the AGC's own test; allowed. |
| 4.3 | Comment an `#include` whose reason is not obvious. | review | |
| 4.3 | Include order: `<stdio.h>`/`<stdlib.h>`, other system headers, then project headers. | clang-format `SortIncludes: CaseSensitive`, `IncludeBlocks: Regroup`, `IncludeCategories` (1 stdio/stdlib, 2 `<...>`, 3 `"..."`), `IncludeIsMainRegex` disabled | Groups are separated by a blank line and sorted alphabetically. A file's own header no longer floats to the top. |
| 4.4 | After includes: enums, typedefs, constant macros, function macros, in that order. | review | |
| 4.5 | External data order: `extern` declarations, non-static definitions, static definitions. | review | CS rule 6: no `extern` variables at all (accessors); see 5.3. |
| 4.6 | `main` is the first function of its file. | check_rules [main-first] | |
| 4.6 | Related functions in the same file, in a meaningful order: breadth-first; alphabetical for many independent utilities. | review | AGC listing order is usually the meaningful order. |
| 4.6 | Separate functions with a row of asterisks. | check_rules [function-separator] (the boxed prolog's top row is the separator) | |
| 4.6 | Functions last, unless data hiding needs an external variable between functions. | review | |

## 5 Function organization

| § | Rule | How enforced | Conflict / note |
|---|------|--------------|-----------------|
| 5 | Function order: prolog, argument declarations, extern declarations, internal (automatic, then static) declarations, statement paragraphs, return (Figure 7). | review | Declaration placement overridden; see 5.4. |
| 5.1 | Every function has a prolog. | check_rules [function-prolog] (a standalone comment ending on the line above the definition) | |
| 5.1 | Prolog gives the function name (lower case with underscores, then a short description). | review (template field FUNCTION NAME) | |
| 5.1 | Prolog lists arguments one per line with type, I/O and description. | check_rules [function-prolog-args] (every parameter named in the prolog) | Description column carries the scale factor (CS). |
| 5.1 | Prolog describes the return value. | check_rules [function-prolog-returns] (non-void functions) | |
| 5.1 (CS) | Prolog's first line gives the AGC label, .agc file and page(s), or `AGC: none` for helpers. | check_rules [function-prolog-agc] | Project rule (CS "Project conventions"), kept as the first line of the box. |
| 5.1 | Name: imperative verb phrase or noun phrase; predicate clause for boolean results (`stack_is_empty`). | review | NAMING.md rule 6 agrees (verbs by function, nouns by content). |
| 5.2 | Declare argument types in the definition (even `int`); definition starts in column 1. | compiler `-Wstrict-prototypes`, C11 (no implicit int); clang-format (column 1) | K&R-style argument lists are not valid in this build. |
| 5.2 | Arguments are explained in the prolog, not by comments after the declaration. | review | |
| 5.3 | Declare external variables right after the function's opening brace. | n/a -- overridden; check_rules [extern-in-c] | CS rule 6: no `extern` variables; shared state via accessors, declarations in headers. |
| 5.4 | Internal variables after externs, with names aligned in one column. | clang-format `AlignConsecutiveDeclarations: Consecutive` (alignment) | Placement overridden: CS rule 6 says declare where used (smallest scope), not at the top of the function. |
| 5.4 | One internal variable per line, each with an explanatory comment; loop indices may share a line and a comment. | check_rules [one-declaration] (`i, j, k` exempt), [decl-comment] | |
| 5.4 | Same parameter/variable name for the same thing across a group of functions. | review | |
| 5.4 | No declarations that hide outer ones. | compiler `-Wshadow` | |
| 5.5 | Blank lines separate statement paragraphs; inline comments explain them. | review; clang-format keeps single blank lines | |
| 5.6 | Use expressions in `return` sparingly (debuggability). | review | |
| 5.6 | One `return` per function unless that makes the logic convoluted. | n/a -- overridden | CS rule 1 prescribes early `return` to replace AGC jumps; the guide's own exception covers it. |
| 5.6 | Always declare the return type; `void` when nothing is returned. | compiler (C11: no implicit int; `-Wall`) | |

## 6 Data types, operators and expressions

| § | Rule | How enforced | Conflict / note |
|---|------|--------------|-----------------|
| 6 | One variable or constant per line. | check_rules [one-declaration] | |
| 6 | Short comments explain all variables and constants. | check_rules [decl-comment] (locals, file scope, struct members), [define-comment] | CS strengthens this: SP/DP/TP/vector comments give the scale -> check_rules [scale-comment]. |
| 6 | Group related variables and constants. | review | |
| 6.1 | Several variables on one line only if related and self-explanatory (`int year, month, day;`). | check_rules [one-declaration] reports every case; review decides | |
| 6.2 | Constant names in capitals, with comments, components aligned. | check_rules [constant-case], [define-comment]; clang-format `AlignConsecutiveMacros: Consecutive`, `AlignConsecutiveAssignments: Consecutive` | |
| 6.2.1 | Use the `const` modifier for constants. | review | The guide's example `const int SIZE 32;` is not valid C (missing `=`). |
| 6.2.2 | No hard-coded numbers or array bounds; name them with `#define`. | review | CS: give the AGC octal/decimal and the engineering value in a comment where a literal stays inline. |
| 6.2.3 | Enums: one identifier per line, aligned braces, assigned values aligned. | clang-format `AllowShortEnumsOnASingleLine: false`, `AlignConsecutiveAssignments`, Allman braces | |
| 6.2.4 | Prefer `const` to `#define` for simple constants (the `10 + 10` pitfall). | check_rules [define-parens] | Overridden in part: in C11 a `const int` is not a constant expression (array sizes, `case` labels, static initialisers), and CS rule 8 allows object-like macros. Macros stay; any value containing an operator is fully parenthesised, which removes the pitfall. |
| 6.3.1 | Floating literals have a digit on both sides of the point (`0.5`, `5.0`). | check_rules [number-format] | |
| 6.3.1 | Hex literals: `0x` with upper-case `A-F`. | check_rules [number-format] | |
| 6.3.1 | Long constants end in upper-case `L`. | check_rules [number-format] | |
| 6.3.2 | Qualifiers with their base type: `short int`, `long int`, `unsigned int`. | check_rules [qualifier-base] | Prefer `<stdint.h>`/`agc_*_t` types anyway (8.1). |
| 6.3.3 | Use structures to group related data instead of parallel vectors. | review | |
| 6.3.4 | Initialise an automatic where declared if used within a page, else just before use. | review; compiler `-Wmaybe-uninitialized` catches the unsafe case | CS rule 6 (declare where used) usually makes both the same place. |
| 6.4 | Make type conversions explicit with casts. | compiler `-Wconversion -Wsign-conversion` | The guide's example `f = (int) i;` casts to the wrong type; read it as `f = (float) i;`. |
| 6.5 | Declare pointer entities with pointer type; `*` goes with the name (`char *s`). | clang-format `PointerAlignment: Right`, `DerivePointerAlignment: false` | CS rule 9: no pointers hidden in typedefs. |
| 6.6 | No pointer conversions except NULL, allocation results, and narrow-and-back-again. | review; compiler `-Wcast-qual` | Allocation exception n/a (CS rule 3, no heap). |
| 6.7 | No space around `->`, `.`, `[]`. | clang-format (built-in), `SpacesInSquareBrackets: false`, `SpaceBeforeSquareBrackets: false` | |
| 6.7 | No space before a function call's `(`; none inside the parentheses. | clang-format `SpaceBeforeParens: ControlStatements`, `SpacesInParentheses: false`, `SpaceInEmptyParentheses: false` | |
| 6.7 | No space between a unary operator and its operand. | clang-format (built-in), `SpaceAfterLogicalNot: false` | |
| 6.7 | One space between a cast and its operand: `(long) m`. | clang-format `SpaceAfterCStyleCast: true`, `SpacesInCStyleCastParentheses: false` | Turns the tree's `(void)AGC_ASSERT(...)` into `(void) AGC_ASSERT(...)`. |
| 6.7 | Spaces around assignment operators. | clang-format `SpaceBeforeAssignmentOperators: true` | |
| 6.7 | Spaces around the conditional operator `?:`. | clang-format (built-in) | |
| 6.7 | One space (or newline) after commas and semicolons. | clang-format (built-in) | |
| 6.7 | One space either side of other binary operators. | clang-format (built-in) | The guide's tolerated `a+1` (tighter-binding operand without spaces) is not reproduced; clang-format always spaces. |
| 6.7 | At most one side effect (`=`, `op=`, `++`, `--`) per expression. | check_rules [multiple-side-effects] | Designated initialisers (`.field =`) and `for` headers are not counted. |
| 6.7 | No side-effect operators inside relational expressions. | check_rules [side-effect-condition] (in `if`/`while`/`switch` tests) | |
| 6.7 | Comma operator only where essential (e.g. `for` headers). | review | |
| 6.7 | Parenthesise liberally, especially masks with shifts. | compiler `-Wparentheses` (in `-Wall`) for the common traps; review | |
| 6.7 | A long condition is split AFTER the logical operators. | clang-format `BreakBeforeBinaryOperators: None`, `AlignOperands: Align`, `ContinuationIndentWidth: 4` | The guide's 7.1.2 and 7.4.2 examples break before `&&`; the 6.7 rule text wins. `BreakBeforeTernaryOperators: false` keeps `?`/`:` consistent with it. |
| 6.8 | Embedded assignments very sparingly (the `while ((c = getchar()) != EOF)` idiom at most). | check_rules [side-effect-condition] reports them all | |
| 6.9 | Avoid conditional expressions unless very clear; comment the ones you use. | check_rules [ternary-comment] (uncommented or nested `?:`) | |
| 6.10 | `* / %` before `+ -`; parenthesise everything else. | review; compiler `-Wparentheses` | |

## 7 Statements and control flow

| § | Rule | How enforced | Conflict / note |
|---|------|--------------|-----------------|
| 7 | Blank lines group statements into paragraphs. | review | |
| 7 | Break complex statements into simpler ones. | review | |
| 7 | Indent to show logical structure. | clang-format | |
| 7.1.1 | One statement per line (except inside a `for` header). | check_rules [one-statement]; clang-format `AllowShort*OnASingleLine` = Never/false | |
| 7.1.1 | Do not rely on side-effect order: `++`/`--` on lines of their own (`a[i] = b[i++]` is wrong). | check_rules [multiple-side-effects] | |
| 7.1.1 | Explicit comparison for numeric tests: `(bufsize % sizeof (int)) == 0`, not `!(...)`. | check_rules [numeric-test] (`!` or bare test on an integer variable or an arithmetic/bitwise expression) | Integer variables are recognised from declarations in the file; `!` on a `bool` or a predicate call is fine. |
| 7.1.2 | Braces-Stand-Alone: each brace on its own line, aligned with the statement that opens the block. | clang-format `BreakBeforeBraces: Custom` with all `BraceWrapping` = true (Allman, plus `BeforeWhile`) | The tree is K&R today; `make format` converts it. |
| 7.1.2 | Use braces where they help, especially nested conditionals and loops. | review | clang-format 14 has no `InsertBraces` (added in 15). |
| 7.1.2 | Blocks over ~40 lines or deeply nested: comment the closing brace. | check_rules [long-block-comment] (function, control and plain blocks > 40 lines) | |
| 7.1.2 | Dummy loop body: `;` on the next line, commented as deliberate. | clang-format (moves `;` to its own line); check_rules [dummy-body] (comment) | |
| 7.1.2 | Space between a reserved word and its `(`. | clang-format `SpaceBeforeParens: ControlStatements` (`if`, `for`, `while`, `switch`, `return`) | Exception: clang-format 14 writes `sizeof(x)` with no space and has no option for it; clang-format wins so `make format-check` and the rule agree. |
| 7.1.2 | Parentheses around the objects of `sizeof` and `return`. | check_rules [sizeof-parens], [return-parens] | The tree currently writes `return x;` almost everywhere (~7800 findings). |
| 7.2.1 | `if`: a single statement indented one level; a block indented one level inside braces. | clang-format `AllowShortIfStatementsOnASingleLine: Never` | |
| 7.2.2 | `if`/`else` with simple statements needs no braces; a compound branch gets braces. | clang-format (layout); review (brace choice) | |
| 7.2.3 | `else if` on one line, the chain aligned. | clang-format `BeforeElse: true` | |
| 7.2.4.1 | Nest `if`s only when there are alternatives or an action must be undone; otherwise combine conditions. | review | |
| 7.2.4.2 | Always brace an `if` nested in an `if`/`else` (dangling else). | compiler `-Wdangling-else` (in `-Wall` via `-Wparentheses`) | |
| 7.2.5 | `switch` layout: braces on their own lines, `case` one level inside, statements one level further. | clang-format `IndentCaseLabels: true`, `AfterCaseLabel: true`, `IndentCaseBlocks: false` | Read from the PDF: the prose examples (2.3, 4.2.2.3, 7.1.1, 7.2.5) indent `case`; only the appendix listing 9.2 does not. The prose wins. |
| 7.2.5 | Comment every fall-through. | compiler `-Wimplicit-fallthrough` (in `-Wextra`; a `/* fall through */` comment satisfies it) | Empty grouped labels (`case A: case B:`) need no comment for gcc; review. |
| 7.2.5 | Every `switch` has a `default`, placed last, with a `break`. | check_rules [switch-default] (present and last); `break`: review | Compatible with `-Wswitch-enum` (still warns for missing enumerators despite a default). |
| 7.3.1 | `while` formats (single statement / block). | clang-format | |
| 7.3.2 | `for` formats; a header that does not fit goes on three lines, not two. | clang-format (built-in: once one section wraps, every section wraps) | Verified on a probe file. |
| 7.3.3 | `do` / block / `while (expr);` on its own line after the closing brace. | clang-format `BraceWrapping.BeforeWhile: true` | |
| 7.4 | Prefer structured code to `goto`. | check_rules.py rule 1 | CS forbids `goto` outright. |
| 7.4.1 | `goto` only to leave deeply nested loops. | n/a -- overridden | CS rule 1: no `goto`. |
| 7.4.2 | `break` may leave a loop at a logical point instead of the loop test. | review | |

## 8 Portability and performance

| § | Rule | How enforced | Conflict / note |
|---|------|--------------|-----------------|
| 8.1 | Use ANSI C. | compiler `-std=c11 -Wpedantic` | |
| 8.1 | Write portable code first; isolate and document optimisations. | review | |
| 8.1 | Some code is inherently non-portable (device handlers). | review | Hardware access is confined to the channel layer (agc/core/channels.h). |
| 8.1 | Separate machine-dependent from machine-independent files. | review | |
| 8.1 | Do not rely on built-in type sizes; use defined sized types. | review | Satisfied by design: `<stdint.h>` and `agc_sp_t`/`agc_dp_t`/`agc_tp_t` (agc/core/word.h). |
| 8.1 | Pointers and integers differ in size; use `sizeof`, not hard-coded sizes. | review | |
| 8.1 | Beware two's-complement tricks; do not replace `*`/`/` with shifts. | review | Special care here: the AGC is ones' complement. Shifts that model AGC SR/SL instructions are deliberate and go through the core API (CS). |
| 8.1 | Use the standard library; do not reimplement it. | review | |
| 8.1 | Hide non-portable quirks behind centrally placed `#ifdef`s. | n/a -- overridden | CS rule 8: conditional compilation only for include guards and `AGC_HOST_DEBUG`. |
| 8.2 | Code must be maintained: clarity over speed when performance is not an issue. | review | |
| 8.2 | When speed matters, comment the "tricky" code. | review | |
| 8.2 | Minimise opens, closes and I/O. | n/a | No I/O in the library (CS); the sim harness owns it. |
| 8.2 | Free allocated memory as soon as possible. | n/a | CS rule 3: no heap. |
| 8.2 | Use `++`, `--`, `+=`, `*=` where no side-effect issue arises. | review | |
| 8.2 | Use structure assignment rather than field-by-field copies. | review | |
| 8.2 | Pass structures by pointer. | review | CS rule 9: one level of dereference. |

## 9 Code examples

| § | Rule | How enforced | Conflict / note |
|---|------|--------------|-----------------|
| 9.1-9.3 | Example Makefile, program file and header illustrate the rules. | n/a | Illustrations only. Where they disagree with the rule text (`case` not indented, `getline (` with a space, `double int`, uncommented locals in 9.2) the rule text wins. |

## Not encoded / tool limits (clang-format 14)

* One declaration per line: clang-format cannot split `int a, b;` ->
  check_rules [one-declaration].
* `sizeof (x)` spacing: no option; clang-format writes `sizeof(x)`.
* Braces around single statements: `InsertBraces` arrives in clang-format 15.
* Right edge of aligned trailing comments: not maintained.
* Column limit: the guide states none; its printed examples fit 80 columns,
  so `ColumnLimit: 80`. Today 4786 lines in src/include/tests exceed 80.
* `ReflowComments: false`: AGC comment layouts and boxed prologs are left as
  written, so over-long comment lines are not rewrapped.

## Interaction with Power of Ten rule 4 (60-line functions)

`make format` was run on a scratch copy of the tree. Compile results did not
change (all 241 .c files pass `gcc -fsyntax-only` with the full flag set
before and after). A second pass
is a no-op. tools/check_rules.py, however, counts a lone `{` as a code line,
so Allman braces push five functions over 60 lines (average_g.c
`agc_job_serv_resume_lm` 62, reentry.c `entry_step_a` 69 and `entry_step_b`
63, lambert.c `agc_conic_iterator` 66, noun_data.c `nd_dp` 64). Either split
them or have check_rules ignore brace-only lines (Holzmann counts statements
and declarations, not braces). That decision belongs to the lead.
