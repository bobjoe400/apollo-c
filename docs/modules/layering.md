# layering -- upward module edges removed (definition of done item 2)

## Scope

docs/agents/review_items.md "From the document wave, final": the
imu -> programs call from closed-loop fix L2 (gyro.c -> agc_align_ram),
every other upward edge tools/depgraph.py reports except those of the
NASSP address map (src/core/ecadr.c, moved by nassp-integration to
src/erasable_map/address_map.c while this lane ran; not touched here),
and the comment leftovers of the document wave.

Baseline (HEAD ef2c141): 68 upward module edges, 197 caller/callee
function pairs; 9 of them from core/ecadr.c.

## Design: one decision per kind of upward edge

| Kind | AGC structure | Decision |
|------|---------------|----------|
| E: noun data held above io (exec/nouns.c, agc_dispatch_noun) | PINBALL_NOUN_TABLES: an ECADR per noun component, any erasable | documented exception (ARCHITECTURE.md already lists it): the id -> owner switch is the C form of the address column (rule 9); depgraph treats it like dispatch.c |
| E: V37 and the fresh-start / restart reset fan-out (exec/change_program/) | FRESH_START_AND_RESTART: GOPROG, STARTSUB, DOFSTART and V37 reach every program's erasables | documented exception (ARCHITECTURE.md lists it); fresh_start.c's one direct up-call (STARTSB1 -> LM DAP) goes through the change_program hooks like the others |
| L: guidance <-> programs | P30/P40/P41/SERVICER and P20-P35/P51-P53/R30-R36 are all major-mode programs; targeting hands the burn programs their erasables, the programs write the burn displays | one layer ("programs layer"): no AGC structure orders them |
| L: csm/dap, lm/dap | T5RUPT/T6RUPT servos commanded by KALCMANU (imu), monitored by T4RUPT (io/service), downlinked (io/telemetry) | io/imu layer, once their three reads above it move down (MASS, ABDELV -> core; SINCDU/COSCDU -> imu) |
| M: an erasable read below its owner | one erasable bank (ERASABLE_ASSIGNMENTS) | storage moves to the lowest module that operates on it: OGC/IGC/MGC -> imu (gyro buffer); DESOPTT/DESOPTS -> io/service t4rupt_csm (OPTDRIVE's input); MASS, ABDELV -> core; SINCDU/COSCDU/CDUSPOT -> imu; the rendezvous erasable block -> nav (DELVSLV is downlinked) |
| M: a routine filed above its lowest caller | the routine's own log section | FINETIME (IMU_CALIBRATION / IMU_PERFORMANCE_TEST_2) -> imu; POWERED_FLIGHT_SUBROUTINES CDU trig and AX*SR*T -> imu; GOTOV56 (a jump to V56 TRACKTRM) -> io/verbs; RRLIMCHK, SETTRKF with FLIP/TRKFLON -> io/service t4rupt_lm (T4RUPT's RR monitor); SXTNB -> programs/alignment |
| V: vehicle-specific code filed in a shared folder | each rope has its own EXTENDED_VERBS fan (LST2FAN) | io/verbs/extended_verbs_{csm,lm,lm_radar}.c -> src/csm, src/lm, src/lm/radar; the extended-verb, DAP-data-load, attitude-error and tracking-terminate jobs get a CSM and an LM handler in tools/registry.tsv instead of a shared wrapper that branches on the vehicle |
| D: shared code continuing in a vehicle's job | TC/TCF to a label of the rope being flown | agc_dispatch_job_as(id, ctx) (generated): the registry row names the CSM and LM label; the shared code depends on the id, not on the vehicle module |

## Progress
- TODO depgraph exceptions/layers, ARCHITECTURE.md
- TODO M OGC -> imu (task 1)
- TODO M FINETIME, CDU trig, GOTOV56, DESOPTT/S, RRLIMCHK/SETTRKF, SXTNB, MASS/ABDELV, rendezvous block
- TODO V extended verbs to vehicle folders
- TODO D P52 seams, R30 CANDEL
- TODO comments (six prologs, erasable.c line 9, keys.c ERROR RESET)
- TODO gates, review_items, MODULE_STATUS

## Decisions per edge
(filled in as each edge is removed)

## Deviations
none (no behaviour change is intended anywhere)

## Stubs
none

## Requested core additions
none (this lane was told to edit include/agc for the moves)

## Tests written
none new: the moves change no behaviour; tests/imu, tests/programs/align,
oracle_imu, oracle_align and `make lm-sim` are the acceptance.

## Open questions

## Lessons
