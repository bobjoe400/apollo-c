# style-final -- clear the remaining `make style` findings (no behaviour change)

## Scope

Definition-of-done item 3: `make style` clean. Start state: 646 findings in
325 files (`make style-report`). The assignment named 11 kinds (395
findings); the other 251 were four prolog kinds in files that never got the
project prolog format (file-prolog-fields 10, function-prolog 117,
function-separator 62, function-prolog-agc 62) and are cleared too, because
`make style` fails on any finding.

Excluded (the nassp-integration lane is editing them): src/nassp,
src/erasable_map, src/sim/socket.c, src/io/dsky/pinball/{load,scale}.c.
Only the first two have findings; their leftovers are listed below.

## Method

* long-block-comment (75): `} /* name */` after the closing brace of every
  function, control and plain block over 40 lines (script from check_style's
  own block finder; function name, or `end <header>` for control blocks).
* qualifier-base (20), sizeof-parens (14), side-effect-condition (2),
  define-comment (4): tests/oracle/oracle_txt.h (`long int`, `unsigned int`,
  `sizeof (x)`; the two `while ((p = f(...)) != NULL)` become
  `p = f(...); while (p != NULL) { ... p++; p = f(...); }`, same iteration
  order, no `continue` in either body), src/core/counters.c (the unused
  CTR_WORD15_MAX macro deleted).
* decl-comment (104), scale-comment (30): trailing comments from the
  surrounding code; scales from the listing (ERASABLE_ASSIGNMENTS.agc B-
  and 2^ notes) or the neighbouring declarations. "unscaled (int)" only for
  counts, pulses, indices and delays in cs.
* ternary-comment (53): a trailing comment saying what each arm means;
  no rewrite was needed.
* function-prolog / separator / agc / args / returns, file-prolog-fields:
  prologs generated from the signature (tools: a small script reading
  check_style's parser; descriptions hand-written per function) for
  src/core/{counters,display_erasables,pad_loads,restart_delays,ecadr,
  assert}.c, tests/oracle/oracle_txt.h, vec_div.c (two missing ARGUMENTS
  tables), alarm.c (ALMCADR page cite), headers assertion.h and status.h.
  Page cites for the erasables come from the `# Page` markers of both
  ropes' ERASABLE_ASSIGNMENTS.agc. The old file comments that claimed "a
  range check on every store" (no such check exists) were dropped.
* `tools/format.sh` run on every edited file.

## Progress

| Kind | Start | Now (outside excluded files) |
|---|---|---|
| long-block-comment | 75 | 0 |
| qualifier-base, sizeof-parens, define-comment, side-effect-condition | 40 | 0 |
| decl-comment, scale-comment | 134 | 0 |
| ternary-comment | 53 | 0 |
| prolog kinds (6) | 314 | 0 |

Excluded-file leftovers (for the lane that owns them, after it finishes):
src/erasable_map/address_map.c 169 findings (function-prolog 29,
function-separator 24, function-prolog-agc 24, ternary-comment 25,
function-prolog-returns 20, function-prolog-args 19, scale-comment 14,
long-block-comment 11, decl/define/file-prolog 1 each);
src/nassp/nassp_engine.c 67 (function-prolog 18, separator 10, agc 10,
ternary 8, args 8, long-block 6, returns 5, scale 1, case-unique 1).
The pinball and socket files have none.

## Open questions / Lessons

* check_style's long-block rule counts blank lines, so every function over
  40 lines wants a closing-brace comment; the tree now has them.
* A standalone comment above a declaration does not satisfy decl-comment
  (the checker looks at the declaration's own lines); the fix is a trailing
  comment, which tools/format.sh keeps.

## Gates (2026-10-13)

make OK; make test exit 0 (incl. scenarios); make check exit 0;
make prolog-report 0; make always-ok-report 0; make lm-sim 34/34 matched
(LMSIM_DIR=scratchpad/src/virtualagc/Contributed/LM_Simulator).
`make style`: 646 -> 236, all in src/erasable_map/address_map.c (169) and
src/nassp/nassp_engine.c (67).
