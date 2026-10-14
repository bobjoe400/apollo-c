# apollo-c history

The production and planning record of the project, kept as it was written: the plan and its definition of done, the lead's worklog, the assignments handed to agents and the notes of each tree-wide wave, the lane diaries, the briefs and open review items, the audits and the tables the work was generated from. None of it describes how the program works; that is in [../docs/](../docs/README.md). Paths inside the diaries, worklog, waves and assignments are the ones in force when they were written (docs/PLAN.md, docs/overview/data/ and so on); the same files now live here.

| Item | What it is |
|---|---|
| [PLAN.md](PLAN.md) | management plan and definition of done |
| [STATE.md](STATE.md) | the project state in one page |
| [MODULE_STATUS.md](MODULE_STATUS.md) | one line per agent, updated by that agent |
| [STYLE.md](STYLE.md) | the NASA SEL-94-003 C style guide audit, rule by rule |
| [ORGANIZATION.md](ORGANIZATION.md) | the repository layout review and its proposals |
| [INDEX.md](INDEX.md) | every file with purpose, owner and verdict (`python3 -I tools/repo_index.py > history/INDEX.md`) |
| [WORKLOG.md](WORKLOG.md) | the lead's running log |
| [RENAMES.md](RENAMES.md), [RENAMES_inventory.txt](RENAMES_inventory.txt) | the identifier renaming map and its raw inventory |
| [assignments/](assignments/) | the assignment given to each agent |
| [waves/](waves/) | notes of each tree-wide pass (naming, style, collapse, status handling and others) |
| [lanes/](lanes/README.md) | the per-lane diaries (progress, findings, requests, lessons); the process lanes (16 review-*, organize, structure, closed-loop-fixes) also hold their former module notes at the end |
| [agents/](agents/) | the common preamble, the subagent brief, the lane review items and the NASSP assignment |
| [data/](data/) | generated tables: call graph, guards, one-liners, structure analysis |
