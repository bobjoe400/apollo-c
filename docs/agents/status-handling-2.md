# Assignment: status-handling-2 -- no silent or sentinel failures  (tier: translation)

Runs ALONE (no other lane editing). Baseline: ~499 void functions with 805
silent assert-returns and ~945 value functions with 1,605 sentinel
assert-returns (tools/check_rules.py reports each as "silent failure" /
"sentinel failure"). Rule: docs/CODING_STANDARD.md rule 7, "No silent
failures". Target: 0 of both, with no (void) casts reintroduced.

Method (tooling first, hand work last):
1. Classify every flagged guard with libclang (reuse tools/cgast.py and the
   call graph from `make callgraph`): a guard is GUARANTEED when every call
   site of the function satisfies it structurally -- pointer arguments are
   `&local`/`&field`/the address of a static object (non-NULL), enum
   arguments come from a `switch` case or a constant, index arguments come
   from a loop bounded by the array size or a constant below it, scaled
   values come straight from a function documented to produce that range.
   Everything else is FALLIBLE. Write the classification to
   docs/overview/guards.tsv (file:line, function, condition, class, why).
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
5. `make`, `make test`, `make check` (0 hard), `make format`. Report counts
   (guaranteed-deleted / fallible-converted / callers updated) in
   docs/modules/status-handling-2.md with Lessons.
