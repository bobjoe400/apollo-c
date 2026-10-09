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
