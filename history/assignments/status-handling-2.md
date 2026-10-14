# Assignment: status-handling-2 -- no silent or sentinel failures  (tier: translation)

Runs ALONE (no other lane editing). Baseline: ~499 void functions with 805
silent assert-returns and ~945 value functions with 1,605 sentinel
assert-returns (tools/check_rules.py reports each as "silent failure" /
"sentinel failure"). Rule: docs/CODING_STANDARD.md rule 7, "No silent
failures". Target: 0 of both, with no (void) casts reintroduced.

Principle (user direction): a condition is checked once, where the value
ENTERS the program (erasable read, channel read, pad load, sim input,
public API boundary). Any later use reached without an intervening write
inherits the fact; a re-check there is dead code and is deleted.

Method (tooling first, hand work last):
1a. If `frama-c` is installed (package frama-c-base): build a Frama-C EVA
   run over libagc (`frama-c -eva` with the project's -I, each test/sim
   entry point as -main in turn, AGC_ASSERT mapped to an ACSL assert via a
   small shim header so EVA sees `//@ assert cond;`). EVA reports each
   assertion as VALID (proven true on all paths -> GUARANTEED, delete),
   UNKNOWN or INVALID (-> FALLIBLE, convert to agc_status_t). Record the
   command and the per-assertion verdict in docs/overview/data/guards.tsv.
   Precision knobs: -eva-precision, -eva-slevel on the interpreter core.
1b. Fallback, or for guards EVA leaves UNKNOWN because of imprecision:
   classify with libclang (reuse tools/cgast.py and the call graph from
   `make callgraph`): a guard is GUARANTEED when every call site of the
   function satisfies it structurally -- pointer arguments are
   `&local`/`&field`/the address of a static object (non-NULL), enum
   arguments come from a `switch` case or a constant, index arguments come
   from a loop bounded by the array size or a constant below it, scaled
   values come straight from a function documented to produce that range.
   Everything else is FALLIBLE. Write the classification to
   docs/overview/data/guards.tsv (file:line, function, condition, class, why).
2. GUARANTEED guards: delete them (the precondition is established by
   construction; say so in a one-line comment only where it is not obvious
   from the call site). Keep rule-5 density with real postconditions
   instead where a file would drop below 2.0.
3. FALLIBLE guards in void or value functions: change the function to
   return agc_status_t, move the value to an out-parameter, and update every
   caller to check the status and act as the AGC would at that site (the
   same discipline as the first status-handling pass; its Coccinelle
   patterns in tools/*.cocci are a starting point). Public headers may be
   edited for this. The oracle tests and sim scenarios must keep passing.
4. Also catch: `return (0)`/`false`/`NULL` after a failed assertion in a
   value function (sentinel), and any helper whose return value every
   caller ignores (that return is a sentinel in disguise; make it void only
   if the guard is GUARANTEED).
5. Add `make prove` that reruns the EVA classification so the boundary-
   check discipline is mechanically re-checkable later. Then `make`,
   `make test`, `make check` (0 hard except possibly the project assertion
   density: report the final density and list where REAL postconditions
   could still be added; the lead decides whether to relax the 2.0 target),
   `make format`. Report counts
   (guaranteed-deleted / fallible-converted / callers updated) in
   docs/history/waves/status-handling-2.md with Lessons.

6. One-line functions (user direction, 2026-10-11). Using libclang (body
   is a single return/expression statement) and the call graph, classify
   every one-line function:
   - PLAIN: the body is an expression over its own parameters/constants
     with no state access and no AGC-named semantics -> inline at every
     caller and delete, whatever the caller count.
   - NAMED: the name carries an AGC meaning the expression does not show
     (overflow-corrected store, a scale conversion with its scale, a
     channel-bit test named after the AGC bit) -> keep.
   - ACCESSOR: reads/writes erasable or module state -> keep (module
     boundary; rule 6 data hiding; it is where the entry check lives).
   Record the classification in docs/overview/data/one_liners.tsv and the counts
   in your note. Add the classification rule to docs/CODING_STANDARD.md.

7. Collapse re-run under the refined label rule (docs/history/assignments/collapse.md,
   "Refined protection rule"): extend tools/callgraph.py with an AGC
   reference count per label (from the .agc sources via the header
   comments' labels; `tools/agc_label.py --grep` or a direct grep), mark
   label-protected single-caller helpers whose label has < 2 references as
   candidates, and inline them with tools/inline_one.py (hand cases as
   before). Start with src/csm/dap/rcs/jet_select.c js_acy / js_bdz.
   Report the count.

8. Dead and normal-flow early returns (user, 2026-10-11; example the
   `return;` in js_acy's TAUCHCK branch, which after inlining has nothing
   after it). With libclang: (a) delete any `return;` / `return (x);` that
   is the last reachable statement of the function on its path (nothing
   follows it at any enclosing level); (b) where a normal-flow early return
   (not an error/status return) only skips a short tail, restructure as
   if/else so the function has one return at the end (guide 5.6); keep
   early returns for error paths (rule 1 allows them). Run this AFTER the
   inlining steps. Report counts.

## Order of work (user, 2026-10-11)
Structural first (steps 1b, 2, 3, 4, 6, 7, 8), format, make/test green;
THEN one `make prove` on the finished tree; apply VALID in reviewed lanes,
record `eva-valid-deferred` in unreviewed lanes. Never prove a tree you are
about to restructure: verdict positions go stale and guards slated for
deletion get analysed for nothing.
