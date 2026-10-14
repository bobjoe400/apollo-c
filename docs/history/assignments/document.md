# Assignment: document   (tier: translation; runs LAST, after review)

Definition-of-done item 9. Generated from the final code, never ahead of
it. Deliverables:
a. docs/overview/README.md: system overview (two vehicles; executive and
   waitlist model; interrupt and clock model; erasable/flag layout; the
   DSKY loop; how a program starts, runs, displays, ends; restart groups).
   Module map with real dependencies, generated from includes/calls
   (ctags + a script in tools/depgraph.py), rendered as Mermaid.
b. docs/overview/flowcharts/*.md: Mermaid flowcharts derived from the C
   control flow for: fresh start, software restart, job pre-emption,
   waitlist tick, V37 program change, CSM P40 burn, P52 realign, LM
   P63-P66 landing, CSM P61-P67 entry, LM P12 ascent.
c. Each docs/modules/<lane>.md gains "Program flow and decisions": the
   decision points (flags tested, alarms raised, branches), data read/
   written with scale, AGC labels -> C functions.
d. docs/overview/xref.md: generated AGC label -> C function -> file:line
   index (tools/xref.py scanning the header comments).
e. docs/TIPS.md consolidated from every lane note's Lessons, organised by
   topic, deduplicated, with the first-hand examples kept.
Diagrams must match the code: regenerate a and d by script; b and c are
written by reading the code, and each names the functions it describes
so a reader can verify.
