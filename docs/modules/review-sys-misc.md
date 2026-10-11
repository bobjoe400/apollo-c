# review-sys-misc -- review of the sys-misc lane

Lane files: src/io/service/{rtb_ops,service,t4rupt,t4rupt_csm,t4rupt_lm,
self_check,interbank}.c; src/io/telemetry/{downlink_lists_csm,
downlink_lists_lm,down_telemetry}.c; src/io/verbs/{extended_verbs,
extended_verbs_csm,extended_verbs_lm,extended_verbs_lm_radar,
uplink_update}.c; src/lm/systems_test.c; include/agc/io/service.h (lane
declarations); tests/io/sysmisc.c.
Review items (review_items.md): LM P07 REDO stub; N08 ALMCADR orphans;
RADMODES one word; sysmisc test free of scratch stand-ins.

(Restarted 2026-10-10 after a cutoff: the earlier run left no note.)

## Progress
* [ ] t4rupt_lm.c fidelity
* [ ] t4rupt.c / t4rupt_csm.c fidelity
* [ ] rtb_ops.c, service.c, self_check.c, interbank.c
* [ ] down_telemetry.c, downlink lists
* [ ] extended verbs (shared, csm, lm, lm_radar), uplink_update.c
* [ ] systems_test.c: P07 fixed-memory part translated
* [ ] RADMODES one word (verified)
* [ ] N08 ALMCADR orphans
* [ ] tests/io/sysmisc.c free of scratch stand-ins
* [ ] style debt to zero, naming

## Findings
| # | file / routine | finding | fix |
|---|---|---|---|

## Lessons
