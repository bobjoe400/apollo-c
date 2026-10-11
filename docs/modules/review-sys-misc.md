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
* [x] t4rupt_lm.c: fidelity, prologs, density (DONE)
* [x] t4rupt.c: fidelity, prologs, density (DONE)
* [ ] t4rupt_csm.c
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
| S1 | t4rupt.c t4_jump (LM T4JUMP p.159) | integration had made phase 0/4 run RCSMONIT *then* RRAUTCHK+GPMATRIX; RCSMON exits through RCSMONEX = RESUME on every path (RCS_FAILURE_MONITOR p.190), the TC never returns | slot 0 runs agc_lm_dap_rcsmon() only |
| S2 | t4rupt.c CDRVE p.134 | DSPTAB+11D = -0: CCS skips the XCH and sends/stores +0 lamps; C sent LOW11 of 077777 (all 11 lamps) | -0 sends 0 |
| S3 | t4rupt.c QUIKDSP p.137 | AGC tests NOUT only; agc_dsky_dspout also tests DSKYFLAG | DEVIATION comment (DSPOUTSB is pinball's) |
| S4 | t4rupt.c C33TEST | WAND CHAN33 was written as a no-op AND with 077777 | agc_chan_and(CHAN33, 33RDMSK) as in the AGC (the channel model resets the flip-flops on any write) |
| S5 | t4rupt_lm.c | RRAUTCHK/RRCDUCHK/RRGIMON/MONREPOS/GPMATRIX checked line by line (OCT05776, OCT10001, OCT32002, OCT20002, RCDUFBIT, REPOSBIT, NORRMBIT verified in the rope) | confirmed; M11..COSMG accessors now assert the matrix invariants |
| S6 | t4rupt.c | CDRVE, DSPOUT, HANG20, QUIKDSP/QUIKOFF/NODSPY/SYNCT4, PROCEEDE, IMUMON/NXTIFBIT, TLIM, ITURNON(2), IMUCAGE, IMUOP(2), PIPFAIL, DNTMFAST, UPTMFAST, SETISSW, CAGESUB(1,2), TNONTEST, PROCTNON, OPONLY, ISSZERO, ENDTNON(2), RETNON, UNZ2, ISSUP, C33TEST, GLOCKMON/SETGLOCK/GLINVERT, LAMPTEST: both ropes diffed, constants -70DEGS -15DEGS 4SECS 90SECS 30RDMSK checked | confirmed |

## Lessons
