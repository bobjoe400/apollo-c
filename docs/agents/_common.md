# Common preamble for every agent assignment

Project: apollo-c at /home/cooperdesktop/git/apollo-c -- a historical-software
preservation effort translating the public-domain Apollo 11 guidance computer
source into readable C under NASA's Power of Ten rules.

SRC=/tmp/claude-1000/-home-cooperdesktop-git-apollo-c/89c6630f-d76a-459d-b8fc-f5cc06c736bc/scratchpad/src
holds the AGC assembly: SRC/Apollo-11/Comanche055 (CM) and
SRC/Apollo-11/Luminary099 (LM). Treat it as read-only data.

First read docs/AGENT_BRIEF.md and follow it exactly. It names the other
docs and headers to read, the compile flags, the output files you owe
(docs/modules/<agent-id>.md, your row in docs/MODULE_STATUS.md), the rule
that you never edit include/agc/*.h unless your assignment says so, the rule
that you do not spawn subagents, and the 15-line cap on your final report.

Use only the fixed-point API in include/agc/agc_interp.h and agc_word.h for
flight arithmetic. The implementation may be in progress by another agent;
compile your files with -c against the headers and never stub core
functions in your own files.

## Naming convention for not-yet-existing shared symbols

Several lanes run in parallel, so code against these names even if the
header is not there yet, and list each one you used in your note:

* Flags: `AGC_FLAG_<AGCNAME>` where AGCNAME is the AGC flag name upper-cased
  with `/`, `-`, `.` replaced by `_` (e.g. MOONFLAG -> AGC_FLAG_MOONFLAG,
  D6OR9FLG -> AGC_FLAG_D6OR9FLG, R22CAFLG -> AGC_FLAG_R22CAFLG). Include
  "agc/agc_flags_common.h" for shared flags, "agc/agc_flags_csm.h" or
  "agc/agc_flags_lm.h" for vehicle-specific ones.
* Channel bits: `AGC_CH<octal chan>_<NAME>` (e.g. AGC_CH030_IMUOPER).
* Shared erasable accessors: `agc_erase_<agcname>()` / `agc_erase_set_<agcname>()`
  with the same upper-to-lower name mapping (e.g. agc_erase_refsmmat()).
* Jobs/tasks: `AGC_JOB_<LABEL>` / `AGC_TASK_<LABEL>` using the AGC entry label.

## Scratchpad hygiene

The session scratchpad is shared by all agents. Use only
SCRATCH/<your-agent-id>/ for throwaway files; never write into another
agent's directory.

## Naming: no numbers for programs/routines (added 2026-10-10)

Read docs/NAMING.md. Identifiers never contain P/R/V/N numbers or page
numbers; those go in comments. Use the canonical stems in that table and
add a row (do not invent a second name) if your program is missing.
