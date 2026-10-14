# review-sys-misc -- review of the sys-misc lane

History of this lane: [history/lanes/review-sys-misc.md](../../history/lanes/review-sys-misc.md)

Lane files: src/io/service/{rtb_ops,service,t4rupt,t4rupt_csm,t4rupt_lm,
self_check,interbank}.c; src/io/telemetry/{downlink_lists_csm,
downlink_lists_lm,down_telemetry}.c; src/io/verbs/{extended_verbs,
extended_verbs_csm,extended_verbs_lm,extended_verbs_lm_radar,
uplink_update}.c; src/lm/systems_test.c; include/agc/io/service.h (lane
declarations); tests/io/service_verbs_telemetry.c.
Review items (history/agents/review_items.md): LM P07 REDO stub; N08 ALMCADR orphans;
RADMODES one word; sysmisc test free of scratch stand-ins.

(Restarted 2026-10-10 after a cutoff: the earlier run left no note.)

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [sys-misc](sys-misc.md#program-flow-and-decisions).
