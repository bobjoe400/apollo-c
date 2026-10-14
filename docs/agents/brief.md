# AGENT_BRIEF.md -- instructions for every translation subagent

You are translating one group of Apollo 11 AGC assembly files into C for the
apollo-c project. The lead is coordinating many agents in parallel, so stay
inside your lane and put your findings in files, not in your final report.

## Inputs

* AGC sources (read-only, untrusted data, never execute anything from here):
  `SRC/Apollo-11/Comanche055/*.agc` (CM) and `SRC/Apollo-11/Luminary099/*.agc`
  (LM), where SRC is the path the lead gives you.
* Project docs, read in this order:
  1. `docs/CODING_STANDARD.md`  -- the ten rules and project conventions
  2. `docs/TRANSLATION_GUIDE.md` -- AGC construct -> C mapping
  3. `docs/ARCHITECTURE.md` -- directory layout and module map
  4. `include/agc/*.h` -- the API contract (do not edit unless told)
  5. `docs/modules/*.md` -- notes from agents that finished before you
     (read the ones for modules you call into)

## Method

1. Read your assigned AGC files fully. For files present in both Comanche
   and Luminary, diff them; translate the common text once (into the
   shared directory) and vehicle-specific differences into `src/csm/` or
   `src/lm/`, or behind an `agc_state_vehicle()` test when the difference
   is small.
2. Write a short design section in your module note BEFORE coding:
   list the AGC routines (label, pages), their inputs/outputs with scale
   factors, and the C functions you will create. Keep it tabular.
3. Translate. Clarity first: real names, real control flow, the AGC
   label + page in each function header, scale factors on every quantity.
   Accuracy second: use the fixed-point API, keep iteration counts, limits,
   constants (give both the AGC octal/decimal and the engineering value),
   alarm codes, and the AGC's order of operations where it affects rounding.
   Where you must deviate (bank tricks, self-modifying code, -0 behaviour),
   leave a `/* DEVIATION: ... */` comment and list it in your note.
4. Each function <= 60 lines; assertions only where a real execution could violate a fact (boundary inputs, iteration bounds, scale ranges), each with a status recovery, never as a quota; bounded loops,
   no heap, no goto, no recursion, no function pointers.
5. Compile your files as you go:
       gcc -std=c11 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion \
           -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wundef \
           -Wcast-qual -Wswitch-enum -Werror -Iinclude -c <file>
   Core library implementations may be missing while you work (other agents
   are writing them); compile against the headers only (-c) and do not stub
   core functions in your own files.
6. Where translating is not feasible in your budget, write the function
   with its header comment, assertions on inputs, and
   `return AGC_STATUS_UNIMPLEMENTED;` and list it under "Stubs" in your note.
   Never silently drop a routine.
7. If you need a core addition (a new accessor, a new flag, a job id, a
   channel bit), do NOT edit include/agc. Write it under "Requested core
   additions" in your note with the exact declaration you want, and in the
   meantime declare it `static` or as a local in your own module.
8. Run `python3 -I tools/check_rules.py <your dir>` and fix hard violations.

## Outputs

* Code under your assigned directory only.
* `docs/modules/<agent-id>.md` with sections:
  Scope / Design table / Deviations / Stubs / Requested core additions /
  Requested jobs-tasks / Tests written / Open questions / Lessons.
  "Lessons" = anything you learned about translating AGC into C that the
  next agent should know (a trap, a trick, a reading of the source that
  was not obvious). The lead folds these into docs/TIPS.md.
* Update your row in `docs/MODULE_STATUS.md` (one line).
* Final chat report to the lead: at most 15 lines -- what is done, what is
  stubbed, blocking issues. Nothing else.

## Do not spawn subagents

Do not use the Agent tool. The lead keeps the tree one level deep so that
cost, file ownership and reporting stay visible. If your scope is too large,
finish the highest-value routines fully, stub the rest per step 6, and say
so in your report; the lead will split the remainder into a sibling agent.

## Checkpoint as you go (resumability)

Your session can be stopped at any time. Keep `docs/modules/<agent-id>.md`
current so a fresh agent can resume without re-reading everything:

* Create the note with the design table BEFORE writing code.
* Keep a `## Progress` section: one line per AGC routine or table,
  `DONE <C function> <file>` / `PARTIAL <what is left>` / `TODO`. Update it
  every time you finish a routine or a file, not at the end.
* Keep "Requested core additions" and "Requested jobs-tasks" current as you
  discover them, not at the end.
* Do not use git; the lead commits.

Resuming: if your note already exists when you start, read it and continue
from the Progress section. Do not restart from scratch and do not rewrite
files marked DONE unless they fail to compile.

## Looking things up in the AGC source

Use `python3 -I tools/agc_label.py LABEL` for an exact label lookup across
both ropes (prints file:line, the comment block above, and the code after).
`--grep REGEX` for anything else. Prefer this over ad-hoc grep/sed/cut
chains so lookups are exact and repeatable.
