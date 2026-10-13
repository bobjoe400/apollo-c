# format2 -- comment placement, one statement per line, paragraph alignment

## Scope
User direction 2026-10-11 (docs/CODING_STANDARD.md "Comment placement and
wrapping", NASA SEL-94-003 2.2.1 / 2.3 / 5.4). The direction changed three
times during the run; the final rules are: no comment inside an
expression; no column limit, one statement per line; the only wrap is the
outermost call with one argument per line (CALCGRAV); paragraph-scoped
columns (type | name | `=` | expression | comment); a blank line before a
comment that starts a paragraph. The whole tree was reformatted.

Files: `tools/fix_comments.py` (new, libclang tokens), `tools/format.sh`
(new: the pipeline), `uncrustify.cfg`, `Makefile` (format/format-check),
`tools/check_rules.py` (assertion count per occurrence), docs/STYLE.md,
docs/CODING_STANDARD.md, README.md, docs/overview/long_statements.tsv
(new), and 306 files under src include tests (layout only).

## Design table

| Piece | What it does |
|-------|--------------|
| fix_comments phase 1 | per statement (`;`-, `{`- or label-terminated token run; braces classified block/record/enum/init): keep the allowed wrap (breaks only after commas of the outermost call, >= 2 continuation lines, one argument each, <= 2 on a call's first line / 1 on a definition's); otherwise join the statement onto one line. In function bodies every comment inside the statement merges with its trailing comment, source order, `; `-separated (in a kept wrap: on the first line). Struct/enum bodies, initialiser tables, `#define`s, labels untouched. |
| fix_comments phase 2 | blank line before a standalone comment after `;`, `}`, an init-table `,` or a pp line; blank after the declarations that open a function body; a trailing comment spanning lines is put on one line. |
| fix_comments phase 3 | per paragraph and enclosing brace: declarations' type and name columns (`*` dangles), assignment-operator column, trailing-comment column = longest code line + 1 (comment after `{` excluded); continuation lines of a kept wrap shift with their first line. |
| uncrustify.cfg | `code_width = 0`; `indent_align_paren = true` (args under the first), `indent_continue = 0`; `nl_func_call_*`/`nl_func_d*_args = ignore`; `align_var_def_span`, `align_var_struct_span`, `align_assign_span`, `align_right_cmt_span = 0` (gap 1, same_level, mix false, at_col 0 kept for reference); `nl_func_var_def_blk = nl_var_def_blk_end = 0`; `nl_before_c_comment`/`nl_before_block_comment = 0` (reasons in Deviations). |
| tools/format.sh | fix_comments, Uncrustify x2, fix_comments; `--check` runs it on copies and prints `FAIL: path`. `make format` / `make format-check` call it. |

## Before / after (src/guidance/burn_steering.c)

The three in-expression examples, from the tree as it was (a7398ed) to the
formatter's output from that same text:

    agc_dp_t th  =
        /* theta/2, rev 2^0 */ agc_dp_div(ip, agc_dp_mul(ip, h, THETACON), rsq);
    ->  agc_dp_t th  = agc_dp_div(ip, agc_dp_mul(ip, h, THETACON), rsq);                  /* theta/2, rev 2^0 */

    agc_vec_t gv = /* g, m/cs^2 2^-5 */
                   agc_vec_div(ip, agc_vec_shl(ip, g->avg.gdt1_2, 1),
                               CS200_B12);
    ->  agc_vec_t gv  = agc_vec_div(ip, agc_vec_shl(ip, g->avg.gdt1_2, 1), CS200_B12);    /* g, m/cs^2 2^-5 */

    agc_dp_t rad = /* (F/m)^2 - Q^2, 2^-10 */ agc_dp_sub(ip, agc_dp_sq(ip,
                                                                       fm), agc_vec_sq(
                                                             ip,
                                                             q));
    ->  agc_dp_t  rad = agc_dp_sub(ip, agc_dp_sq(ip, fm), agc_vec_sq(ip, q));             /* (F/m)^2 - Q^2, 2^-10 */

From a7398ed the formatter alone also joins CALCGRAV (its old wrap had 3
arguments on the first line) and writes `/* (DV.UP) UP, m/cs 2^7; PDL 0 */`
and `/* B.C, scale 2^-15; 12D */` in source order; the user reordered
those two by hand.

Acceptance: `make format` on the user's hand-formatted version (HEAD
7ef499b). agc_guid_calcut is line for line the user's, CALCGRAV wrap kept;
only spacing changed (columns exact, trailing blanks gone):

    agc_guid_t  *g  = agc_guid_state();                      /* avg, burn */
    agc_rendz_t *rz = agc_rendz_erasable();                  /* RTIG VIPRIME */
    agc_status_t st = agc_guid_calcgrav(ip, AGC_VEHICLE_CSM, /* CALCGRAV at RTIG: GDT1/2 */
                                        rz->body.rtx2 != 0,
                                        rz->rtig,
                                        &g->avg);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_vec_t gv  = agc_vec_div(ip, agc_vec_shl(ip, g->avg.gdt1_2, 1), CS200_B12);                           /* g, m/cs^2 2^-5 */
    agc_vec_t dvr = agc_vec_div(ip, agc_vec_sub(ip, rz->viprime, g->burn.ut), CS200_B12);                    /* dV/dt, m/cs^2 2^-5 */
    agc_vec_t bc  = agc_vec_shl(ip, agc_vec_scale(ip, agc_vec_sub(ip, dvr, gv), g->burn.csteer), 2);         /* 12D B.C, scale 2^-15 */
    agc_vec_t ug  = agc_vec_unit(ip, agc_erase_vgtig(), NULL, NULL);                                         /* UG PDL 0, 2^1 */
    agc_vec_t q   = agc_vec_sub(ip, bc, agc_vec_shl(ip, agc_vec_scale(ip, ug, agc_vec_dot(ip, ug, bc)), 2)); /* Q, scale 2^-5 */
    agc_dp_t  fm  = agc_dp_div(ip, agc_dp_shr_round(g->burn.f, 4), g->burn.weight_g);                        /* F/m, scale 2^-5 */
    agc_dp_t  rad = agc_dp_sub(ip, agc_dp_sq(ip, fm), agc_vec_sq(ip, q));                                    /* (F/m)^2 - Q^2, 2^-10 */
    agc_vec_t t   = agc_vec_shl(ip, agc_vec_scale(ip, ug, agc_dp_sqrt(ip, rad)), 1);                         /* UG sqrt(), 2^-5 */

calcthet, same shape as the user's (same lines and paragraphs; the `=`
and comment columns of each paragraph now line up exactly):

    agc_rendz_t *rz = agc_rendz_erasable();    /* targeting */
    agc_burn_t  *b  = &agc_guid_state()->burn; /* UT, F, WEIGHT/G */

    rz->vinit  = rz->vtig;
    agc_dp_t h = 0;                                                                 /* |V x R| (36D), m^2/cs 2^36 */
    b->ut      = agc_vec_unit(ip, agc_vec_cross(ip, rz->vtig, rz->rtig), &h, NULL); /* UP */

    rz->rinit    = rz->rtig;
    agc_dp_t rsq = agc_vec_sq(ip, rz->rtig);                         /* VSQ PDDL 36D, m^2 2^58 */
    agc_dp_t th  = agc_dp_div(ip, agc_dp_mul(ip, h, THETACON), rsq); /* theta/2, rev 2^0 */
    th           = agc_dp_mul_trunc(agc_dp_mul_trunc(th, rz->delvsab), b->weight_g);
    th           = agc_dp_div(ip, th, b->f);                         /* 14D         */

    agc_vec_t dvup   = agc_vec_shl(ip, agc_vec_scale(ip, b->ut, agc_vec_dot(ip, rz->delvsin, b->ut)), 2);  /* PDL 0 (DV.UP) UP, m/cs 2^7 */
    agc_vec_t dvp    = agc_vec_sub(ip, rz->delvsin, dvup);                                                 /* PDL 6, m/cs 2^7 */
    agc_dp_t  sn     = agc_dp_sin(ip, th);                                                                 /* PDL 12, scale 2^1 */
    agc_vec_t vg     = agc_vec_scale(ip, agc_vec_unit(ip, agc_vec_cross(ip, dvp, b->ut), NULL, NULL), sn); /* VGTIG, m/cs 2^7 */
    agc_dp_t  dvpmag = 0;                                                                                  /* |DVP|, m/cs 2^7 */
    agc_vec_t udvp   = agc_vec_unit(ip, dvp, &dvpmag, NULL);                                               /* PDL 6, scale 2^1 */

    vg = agc_vec_add(ip, agc_vec_scale(ip, udvp, agc_dp_cos(ip, th)), vg);
    vg = agc_vec_add(ip, agc_vec_shl(ip, agc_vec_scale(ip, vg, dvpmag), 2), dvup);

    agc_erase_set_vgtig(vg);                  /* VG IGNITION */
    b->ut = agc_vec_unit(ip, vg, NULL, NULL); /* THRUST DIR. */

Constant block (THETACON ... SEC5_14): unchanged paragraphs (comment +
`static const` lines, one blank between), `=` and trailing comments now
aligned within each paragraph, e.g.

    /* TRIMSCAL 2DEC 1.07975111 B-1: 85.41"/bit trims to rev; YBIAS, PBIAS:
     * yaw +0.95 deg and pitch -2.15 deg mechanical bias, thrust on.          */
    static const agc_dp_t TRIMSCAL = 144921741; /* scale 2^1 */
    static const agc_dp_t YBIAS    = 708371;    /* +0.95 deg, rev 2^0 */
    static const agc_dp_t PBIAS    = -1603156;  /* -2.15 deg, rev 2^0 */

## Counts (tree at 7ef499b -> after `make format`)
* 306 files changed (+13286 / -16605 lines); 3661 statements rewritten
  (wraps joined and/or comments merged); about 620 comments inside
  statements merged into trailing comments; 69 multi-line trailing comments
  put on one line; 1253 paragraph blank lines added; 50 outer-call wraps
  kept (CALCGRAV shape).
* docs/overview/long_statements.tsv: 777 rows (file:line, lines, width) --
  11 kept wraps with > 2 continuation lines, 766 one-line statements wider
  than 120 columns (616 in src/) -- for the review wave to split with
  named temporaries. Not split here.

## Validation
* `gcc -E -P -D__LINE__=0` (with and without -DAGC_HOST_DEBUG) of all 270
  .c files: token-identical before/after, except 6 files whose difference
  is white space inside `#x`-stringified AGC_ASSERT arguments (string
  literals compared with white space removed: 0 differ).
* `make`, `make test` green; test output identical except two assert
  report line numbers (executive.c, init.c) that moved.
* `make format` twice: second run changes nothing; `make format-check`
  passes. Two pipeline runs on a scratch copy: identical.
* check_rules: 50 hard violations before and after (all pre-existing;
  `make check` was already red on check-rules at HEAD). Warnings 113 ->
  144: assertion density now counts every `AGC_ASSERT(` on a line and
  sees function headers that used to be wrapped (more functions counted).
  check_style: 8754 -> 8702 findings (ternary-comment and
  long-block-comment down, nothing up).

## Deviations
* Uncrustify 0.72 alignment spans (`align_var_def_span`,
  `align_assign_span`, `align_right_cmt_span`) count lines and run across
  blank lines (tested: span 16 aligned comments two paragraphs apart), so
  they are 0 and fix_comments does paragraph alignment.
* `nl_before_c_comment = 2` / `nl_before_block_comment = 2` were tried and
  dropped: they skip a comment that follows a trailing comment and insert a
  blank between two comments (section banner + prolog, block comment +
  next comment), against "consecutive comment lines stay together".
* `nl_func_var_def_blk` / `nl_var_def_blk_end` put a blank after every run
  of declarations, which splits the user's calcthet paragraphs
  (`agc_dp_t th = ...; th = ...;`); both 0, and fix_comments adds a blank
  only after the declarations that open a function body.
* The allowed-wrap test is structural (see Design); a hand wrap of another
  shape is joined. Comment merge order is source order (the user wrote
  "PDL 0 (DV.UP) ..." by hand; the formatter keeps that text as it is).
* Enum values and `#define` values are still aligned by Uncrustify over
  consecutive lines (span 1), not per paragraph.

## Stubs / Requested core additions / Requested jobs-tasks
None.

## Tests written
None (formatter). Scratch scripts: SCRATCH/format2/ (pipe.sh, toks.py).

## Open questions (for the lead)
* `make check` still fails on 50 pre-existing check_rules hard violations
  (rule 7 `(void)` casts etc.), not formatting.
* 766 statements wider than 120 columns: the review wave's split list.

## Progress
- DONE tools/fix_comments.py, tools/format.sh, uncrustify.cfg, Makefile
- DONE tree formatted; build, tests, token parity, fixed point verified
- DONE STYLE.md, CODING_STANDARD.md, README, long_statements.tsv

## Lessons
* Uncrustify 0.72 cannot scope alignment to a paragraph and cannot indent
  continuations inside plain parentheses (`return (`) except +4; anything
  it cannot express goes in a token-level post-pass, and `format-check`
  must then run the whole pipeline (a post-pass after Uncrustify makes
  `uncrustify --check` meaningless).
* Uncrustify keeps every existing line break; it never joins. Joining has
  to be done before it (fix_comments phase 1).
* libclang lexing does not need the headers: parse with `-nostdinc` and
  PARSE_SKIP_FUNCTION_BODIES and read `tu.get_tokens(extent=...)`; comments
  come back as tokens. 4 ms per file.
* Joining a wrapped macro argument changes the white space of its `#x`
  stringification (AGC_ASSERT text); compare preprocessed string literals
  with white space removed.
* Token-level brace classification: `{` after `=`, `,`, `(`, `return`,
  inside parentheses, or after a same-line `)` is an initialiser (compound
  literal in an Allman tree); otherwise a block, record or enum body.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ A tree-wide pass (comment placement and statement layout): it changed code owned by the translation lanes without changing what the program decides, so it has no program flow of its own. The flows as the code now stands are in the lane sections: [exec-sched](exec-sched.md#program-flow-and-decisions), [core-numeric](core-numeric.md#program-flow-and-decisions), [state-flags](state-flags.md#program-flow-and-decisions), [nav-conic](nav-conic.md#program-flow-and-decisions), [nav-integ](nav-integ.md#program-flow-and-decisions), [nav-aux](nav-aux.md#program-flow-and-decisions), [io-pinball](io-pinball.md#program-flow-and-decisions), [io-nouns](io-nouns.md#program-flow-and-decisions), [imu](imu.md#program-flow-and-decisions), [guid-powered](guid-powered.md#program-flow-and-decisions), [guid-landing](guid-landing.md#program-flow-and-decisions), [guid-entry](guid-entry.md#program-flow-and-decisions), [p37](p37.md#program-flow-and-decisions), [dap-lm](dap-lm.md#program-flow-and-decisions), [dap-csm](dap-csm.md#program-flow-and-decisions), [prog-rendz](prog-rendz.md#program-flow-and-decisions), [prog-align](prog-align.md#program-flow-and-decisions), [sys-misc](sys-misc.md#program-flow-and-decisions), [sim](sim.md#program-flow-and-decisions); the system view is [docs/overview/README.md](../overview/README.md) and its flowcharts.
