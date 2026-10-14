# style2 -- port the formatter to Uncrustify (statement paragraphing)

## Scope
Replace clang-format 14 with Uncrustify 0.72.0 as the project's only
formatter, reproduce every `.clang-format` rule, and add NASA SEL-94-003
statement paragraphing (2.2.1, 5.4, 5.5, 7). The real tree was NOT
reformatted; the lead runs `make format` after status-handling finishes.

Files: `uncrustify.cfg` (new), `Makefile` (format / format-check),
`.clang-format` (deleted; in git history), `docs/STYLE.md` (enforcement
column, tool limits, rule-4 section), `docs/CODING_STANDARD.md` (Tooling),
`README.md`, `tools/check_style.py` (docstring; sizeof regex, see below),
`docs/templates/function_prolog.c` (reformatted: two paragraph blank lines).

## Design table (uncrustify.cfg, 164 options + `file_ext C .c .h`)

| Guide | clang-format had | uncrustify.cfg |
|-------|------------------|----------------|
| 2.2.3 | IndentWidth 4, UseTab Never | indent_columns 4, indent_with_tabs 0 |
| 80 col | ColumnLimit 80 | code_width 80 |
| 7.1.2 / 7.3.3 | Allman, BeforeWhile | nl_*_brace force, nl_brace_else, nl_brace_while |
| 7.2.5 | IndentCaseLabels, AfterCaseLabel | indent_switch_case 4, nl_case_colon_brace |
| 6.7 | spacing set, SpaceAfterCStyleCast | sp_* (arith/assign/compare/bool/cond, member, unary, after_cast force) |
| 7.1.2 | SpaceBeforeParens ControlStatements | sp_before_sparen, sp_return_paren, sp_sizeof_paren force |
| 6.5 | PointerAlignment Right | sp_before_ptr_star force, sp_after_ptr_star remove |
| 6.7 | break after && / \|\|, AlignOperands | pos_bool trail, pos_conditional trail, indent_align_paren |
| 7.3.2 | 3-line for split | ls_for_split_full |
| 7.1.1 / 7.2.1 | AllowShort* Never | nl_after_semicolon, nl_split_*_one_liner, nl_*_leave_one_liners false |
| 5.4 / 6.2 / 6.2.3 | AlignConsecutive Decl/Assign/Macros | align_var_def/struct/func_params/func_proto/assign/enum_equ/pp_define span 1 |
| 2.3 | AlignTrailingComments | align_right_cmt_span 2, align_right_cmt_gap 2 |
| 4.3 | SortIncludes, Regroup, categories | mod_sort_include, case-sensitive, angle-over-quotes (per block) |
| 2.2.1 | MaxEmptyLines 1, no blank at block start | nl_max 2, eat_blanks_after_open_brace / before_close_brace |
| 4.6 | SeparateDefinitionBlocks | nl_after_func_body 2 |
| 5.4/5.5 | (none) | nl_func_var_def_blk 1, nl_var_def_blk_end 2 |
| 5.5/7 | (none) | nl_before_/nl_after_ if/for/while/do/switch = add |
| 5.6 | (none) | nl_before_return |
| tokens | -- | mod_full_brace_* ignore, mod_paren_on_return ignore, cmt_width 0, ... |

Every option is commented in uncrustify.cfg with the section it serves.

## Deviations (could not be expressed exactly in Uncrustify 0.72)
* Guard runs: consecutive `if (!AGC_ASSERT(..)) { return ..; }` get a blank
  line BETWEEN guards (no "consecutive if" exception; nl_before_if /
  nl_after_if apply to each). Closest available; merge a run into one
  `if (!AGC_ASSERT(a) || !AGC_ASSERT(b))` where wanted.
* An if/for/while/switch directly after `case X:` gets a blank line after
  the label (247 sites in the current tree): Uncrustify treats the label as
  a preceding statement. `return` after a label is handled correctly.
* Trailing-comment alignment has no column limit. Chosen rule: a comment
  joins the aligned column only if written with >= 2 spaces before it
  (align_right_cmt_gap 2), which keeps clang-format's existing choices.
  New code must use two spaces to get alignment. Groups that clang-format
  split are merged when the separating line is out of the group, so lines
  over 80 columns rise 447 -> 920 (nearly all comments in structs and
  prototype blocks). gap 0 instead would align everything and give ~2,000.
* Alignment runs before wrapping, so a line wrapped in pass 1 can regroup
  in pass 2: `make format` runs two passes (11-12 files change in pass 2;
  pass 3 changes nothing on all three sample trees).
* `<stdio.h>`/`<stdlib.h>` first cannot be ranked; blocks are sorted
  individually (grouping would sort stdio after math.h, so it is off).
* Continuation after a line ending in a nested call's `(` aligns under the
  enclosing paren and may rewrap (a handful of lines, e.g.
  earth_orbit_insertion.c ak_out[]).
* Assignment alignment includes compound operators (`x  -= 1` aligns with
  `=`); a decl after a multi-line `= { ... };` initialiser aligns with it
  (11 sites). Cosmetic.
* `sizeof(x)` -> `sizeof (x)` (guide 7.1.2; clang-format could not). Fixed
  tools/check_style.py [sizeof-parens] regex, which backtracked and flagged
  `sizeof (x)`: `\bsizeof\b\s*(?!\()` -> `\bsizeof\b(?!\s*\()`.
* .h files were parsed as C++ by default (`in` treated as a keyword:
  `agc_orbit_params_in_t * in`); `file_ext C .c .h` in the cfg fixes it.

## Validation (scratch copies only)
Trees: e30693d (last commit where everything compiled), HEAD, and the
working tree snapshot (status-handling mid-edit).

| Tree | pass1/2/3 files changed | compile ok before/after (-c, brief flags) | gcc -E tokens (241 .c) | >80 cols |
|------|-------------------------|-------------------------------------------|------------------------|----------|
| e30693d | 273 / 11 / 0 | 241 / 241, same set | 0 differ | 447 -> 920 |
| HEAD | 273 / 12 / 0 | 152 / 152, same set (HEAD mid status-handling) | 0 differ | 451 -> 923 |
| snapshot | 273 / 12 / 0 | 152 / 152, same set | 0 differ | 451 -> 923 |

Token check: `gcc -E -P -D__LINE__=0` (also with -DAGC_HOST_DEBUG),
whitespace stripped, every .c file (not just 20). `--check` passes after
`make format`; `make format` twice changes nothing. check_rules: identical
(blank lines are not counted; rule 4 unchanged). check_style: only
[long-block-comment] moves, 122 -> 292 (>40-line blocks now longer).

### Before / after: csm_dap_dpadd (src/csm/dap/mysubs.c; the assignment
said tvc/exec.c, the function lives in mysubs.c)

Before (clang-format):

    {
        (void) AGC_ASSERT((kmpac >= AGC_DP_MIN) && (kmpac <= AGC_DP_MAX));
        (void) AGC_ASSERT((al >= AGC_DP_MIN) && (al <= AGC_DP_MAX));
        int64_t s = (int64_t) kmpac + (int64_t) al;
        if (s > AGC_DP_MAX)
        {
            s -= 2 * W_DP_ONE; /* + overflow: subtract 360 deg       */
        }
        else if (s < AGC_DP_MIN)
        {
            s += 2 * W_DP_ONE; /* - overflow: add 360 deg            */
        }
        else
        {
            /* no overflow */
        }
        return (agc_dp_t) s;
    }

After (uncrustify):

    {
        (void) AGC_ASSERT((kmpac >= AGC_DP_MIN) && (kmpac <= AGC_DP_MAX));
        (void) AGC_ASSERT((al >= AGC_DP_MIN) && (al <= AGC_DP_MAX));
        int64_t s = (int64_t) kmpac + (int64_t) al;

        if (s > AGC_DP_MAX)
        {
            s -= 2 * W_DP_ONE; /* + overflow: subtract 360 deg       */
        }
        else if (s < AGC_DP_MIN)
        {
            s += 2 * W_DP_ONE; /* - overflow: add 360 deg            */
        }
        else
        {
            /* no overflow */
        }

        return (agc_dp_t) s;
    }

## Stubs
None.

## Requested core additions / jobs-tasks
None.

## Tests written
None (formatter). Scratch scripts in SCRATCH/style2/ (run.sh, validate.sh,
cc.sh, tok.sh).

## Open questions (for the lead)
* Accept ~470 extra >80-column comment lines, or shorten/move them after
  `make format` (list: `awk 'length>80'` over src include tests)?
* check_style [long-block-comment] now fires on 292 blocks; count only
  non-blank lines there too?
* Status-handling's guard runs: merge into one `if` per function, or accept
  one blank line between guards?

## Progress
- DONE uncrustify.cfg (164 options) -- validated on 3 trees
- DONE Makefile format (2 passes) / format-check (--check)
- DONE .clang-format deleted; STYLE.md, CODING_STANDARD.md, README updated
- DONE templates reformatted; check_style sizeof regex fixed

## Lessons
* Uncrustify parses .h as C++ unless told: `file_ext C .c .h` in the cfg.
* `cmt_multi_check_last` (default true) strips the leading space of ` * `
  continuation lines when a block comment's first and last lines have the
  same length -- i.e. every right-padded `... */` comment. Turn it off.
* `nl_after_brace_open = true` also breaks compound literals
  (`(agc_vec_t){{0, 0, 0}}`) over four lines; Allman needs only the
  per-statement nl_*_brace options.
* align_right_cmt_span counts differently from the other spans: 2 means
  "consecutive lines", 1 aligns nothing.
* align_right_cmt_gap tests ORIGINAL columns, so it is not idempotent in one
  pass; neither is width wrapping after alignment. Format twice.
* `mod_sort_incl_import_grouping_enabled` merges blank-separated include
  blocks before sorting.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ A tree-wide pass (the Uncrustify formatter): it changed code owned by the translation lanes without changing what the program decides, so it has no program flow of its own. The flows as the code now stands are in the lane sections: [exec-sched](../../modules/exec-sched.md#program-flow-and-decisions), [core-numeric](../../modules/core-numeric.md#program-flow-and-decisions), [state-flags](../../modules/state-flags.md#program-flow-and-decisions), [nav-conic](../../modules/nav-conic.md#program-flow-and-decisions), [nav-integ](../../modules/nav-integ.md#program-flow-and-decisions), [nav-aux](../../modules/nav-aux.md#program-flow-and-decisions), [io-pinball](../../modules/io-pinball.md#program-flow-and-decisions), [io-nouns](../../modules/io-nouns.md#program-flow-and-decisions), [imu](../../modules/imu.md#program-flow-and-decisions), [guid-powered](../../modules/guid-powered.md#program-flow-and-decisions), [guid-landing](../../modules/guid-landing.md#program-flow-and-decisions), [guid-entry](../../modules/guid-entry.md#program-flow-and-decisions), [p37](../../modules/p37.md#program-flow-and-decisions), [dap-lm](../../modules/dap-lm.md#program-flow-and-decisions), [dap-csm](../../modules/dap-csm.md#program-flow-and-decisions), [prog-rendz](../../modules/prog-rendz.md#program-flow-and-decisions), [prog-align](../../modules/prog-align.md#program-flow-and-decisions), [sys-misc](../../modules/sys-misc.md#program-flow-and-decisions), [sim](../../modules/sim.md#program-flow-and-decisions); the system view is [docs/overview/README.md](../../overview/README.md) and its flowcharts.
