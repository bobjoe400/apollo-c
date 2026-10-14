# Assignment: oracle   (tier: lead/translation)

(Already running from the first launch; file kept for reference and
relaunch.) See docs/PLAN.md section "Test oracle". Build yaYUL and yaAGC
from SRC/virtualagc, assemble both ropes, write tools/oracle/agc_probe.c
(load rope + symtab, poke erasables from a spec on stdin, run to a label or
cycle limit, dump requested erasables as `NAME octal` lines), prove it on
SPSIN and one interpretive unary, write tools/oracle/README.md and
docs/modules/oracle.md.
