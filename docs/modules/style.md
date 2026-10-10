# style -- NASA SEL-94-003 C Style Guide applied to apollo-c

## Scope
Read all of docs/reference/NASA-SEL-94-003-C-Style-Guide.txt (chapters 1-9
and the index). Where the text extraction lost indentation, the PDF's
LZW-compressed content streams were decoded and the text-positioning
offsets read directly. From that came docs/STYLE.md (one row per rule),
.clang-format, tools/check_style.py, the templates and the Makefile
targets. No source code was reformatted.

## Deliverables
| File | What |
|------|------|
| docs/STYLE.md | 173 rows, guide order, enforcement + conflicts |
| .clang-format | clang-format 14: all 69 keys confirmed in `--dump-config`; behaviours confirmed on a probe file |
| tools/check_style.py | 30 rule ids (`--list-rules`), `--summary`, `--rule`, `--exit-zero`; read-only |
| docs/templates/file_prolog.c, header_prolog.h, function_prolog.c | boxed prologs adapted (AGC SOURCE, scale column, AGC label/page line); clang-format clean; compile with the full flag set |
| Makefile | `format`, `format-check`, `style` (`STYLE_FLAGS`), `check-rules`; `check` = check-rules + format-check + style |
| README.md | build section lists the new targets |

## Progress
- DONE guide read in full (txt + PDF streams for 2.3, 4.2.2.3, 6.7, 7.1.1, 7.1.2, 7.2.5, 7.3.2, 7.3.3)
- DONE docs/templates/*
- DONE docs/STYLE.md
- DONE .clang-format (verified against clang-format 14.0.6)
- DONE tools/check_style.py (probe file exercises every rule)
- DONE Makefile targets

## Baseline (check_style on src include tests, before any reformat)
    file-prolog-fields       4.1/3.4       300
    function-prolog          5.1           847
    function-separator       4.6          3301
    function-prolog-agc      5.1/CS       1492
    function-prolog-args     5.1          2317
    function-prolog-returns  5.1          2766
    return-parens            7.1.2        7836
    sizeof-parens            7.1.2          79
    numeric-test             7.1.2           2
    one-statement            7.1.1         785
    one-declaration          6.1           180
    decl-comment             5.4/6        6412
    scale-comment            CS           1093
    define-comment           6.2           835
    number-format            6.3.1          10
    qualifier-base           6.3.2         105
    side-effect-condition    6.7/6.8         1
    multiple-side-effects    6.7/7.1.1      61
    ternary-comment          6.9           985
    long-block-comment       7.1.2          26
    header-name              3.4             1
    main-first               4.6            21
    29455 style findings in 300 files

clang-format: 297 of 300 files would change. A scratch-copy `make format`
left the compile results unchanged (all 241 .c files pass), and a second
pass is a no-op. It does push 5 functions over check_rules' 60-line limit
(brace-only lines are counted); details are at the end of docs/STYLE.md.

## Deviations from the guide (project rules win)
Function-name capitalisation (2.4.3), no nested headers (3.4), PDL (4.2),
goto/ABORT (4.2.4.1, 7.4.1), extern inside functions (5.3), declarations at
the top of the function (5.4), single return (5.6), const instead of
#define (6.2.4), #ifdef portability layer (8.1), and malloc/free/I-O rules
(6.6, 8.2). Each row in STYLE.md gives the reason.
Tool-imposed: `sizeof(x)` has no space (clang-format 14 has no option for it).

## Requested core additions
None. Findings for the lead:
include/agc/core/assert.h shares its name with <assert.h> (3.4).
README.md does not list AGC_HOST_DEBUG or the reused yaAGC tree (3.2).

## Open questions
- Allman braces and Power of Ten rule 4: split the 5 functions, or make
  check_rules.py ignore brace-only lines?
- Adopting every finding means ~7800 `return (x)` edits and ~6400
  declaration comments. These are mechanical but large; one option is a
  Coccinelle/scripted pass for return-parens and per-lane cleanup for the
  rest.

## Lessons
- The guide's .txt has no leading whitespace, so its layout rules cannot be
  read from it. The PDF is PDF-1.1 with /LZWDecode streams. A 30-line LZW
  decoder (EarlyChange=1) exposes `TD` x-offsets: +22/23 pt in 10 pt
  Courier is one 4-column indent level. That showed `case` is indented
  inside the switch braces (the prose examples), unlike appendix 9.2.
- The guide contradicts itself in places: function names (2.4.3 CapWords
  vs 5.1 lower_case), break before vs after `&&` (6.7 text vs the
  7.1.2/7.4.2 examples), `return (x)` vs `return(x)`. Rule text beats
  examples, and STYLE.md records each case.
- In clang-format 14, `SpacesBeforeTrailingComments` applies only to `//`
  comments. Do not rely on it for `/* */`.
- clang-format natively splits an over-long `for` header into three lines,
  moves a dummy body's `;` to its own line, and rewrites `return(x)` as
  `return (x)`; no options are needed for any of these.
- `IncludeIsMainRegex` set to a never-matching suffix stops a file's own
  header from jumping ahead of the system headers (guide 4.3 order).
- The tree is being moved by other lanes while it is read, so
  check_style.py skips files that disappear mid-walk.
- The NAMING.md stem rule (`AGC_JOB_X` / `agc_job_x`) collides with
  guide 2.4 "unique irrespective of case"; only mixed-case variants are
  reported.
