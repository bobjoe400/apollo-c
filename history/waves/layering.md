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
(resumed 2026-10-13 after the weekly cutoff; the first run's moves are in
checkpoint 7826e91. Bounded scope from the lead: smallest change per edge,
no extended-verb folder moves or P52/R30 restructuring unless unavoidable.)
- DONE depgraph: exec/nouns and exec/change_program as exceptions, guidance =
  programs layer, csm/dap and lm/dap on the io/imu layer (tools/depgraph.py)
- DONE M OGC/IGC/MGC -> imu (agc_imu_ogc, src/imu/gyro.c; task 1); lm-sim 34/34
- DONE M FINETIME -> imu/mode.c; CDU trig + AX*SR*T -> imu/cdu_trig.c; GOTOV56 ->
  io/verbs/extended_verbs.c; DESOPTT/S -> io/service/t4rupt_csm.c; RRLIMCHK,
  SETTRKF/FLIP/TRKFLON -> io/service/t4rupt_lm.c; SXTNB ->
  programs/alignment/geometry_csm.c; MASS, ABDELV -> core/erasable.c
- DONE erasable_map callers: the address column (tools/depgraph.py ADDRESS_COLUMN); ARCHITECTURE.md layering + tree
- DONE io/verbs: DAPFIG, V82CALL (agc_exec_jump); LANDMARK/MRKBUF -> core; OPTC1 job ->
  csm/optics; CLRADMOD, RRZERO, LRPOS2, RADNOOP -> io/service/t4rupt_lm.c; RRDESK2 job ->
  lm/radar/designate.c; RRLOSDSP jobs -> lm/optics/lem_geometry.c
- DONE programs/alignment P52 seams: three registry rows + agc_exec_jump
- DONE io/telemetry DELVSLV (noun data dispatch); programs/displays CANDEL -> csm/burns
- DONE depgraph: no upward edge (2026-10-13); lm-sim baseline at HEAD 34/34 (task 1)
- DONE comments: six file prologs, erasable.c line 9, keys.c CLUPLOCK -> ACCEPTUP
- DONE gates (2026-10-13, private BUILD): make, make test (unit + 14 scenarios), make
  lm-sim 34/34 matched, make check (0 hard; format clean; style 646 = HEAD),
  prolog-report 0, always-ok-report 0, depgraph upward table "(none)";
  docs/overview/README.md regenerated; review_items struck; MODULE_STATUS row

## Decisions per edge
Smallest change per edge (lead, 2026-10-13); behaviour identical (gates
below). First run (7826e91): the M rows of the design table.

| Edge (caller -> callee) | AGC structure | Change |
|---|---|---|
| imu -> programs/alignment (gyro_words -> agc_align_ram, L2) | OGC/IGC/MGC: one erasable, ECADR OGC at IMUPULSE | OGC buffer owned by imu: agc_imu_ogc() (src/imu/gyro.c); CALCGTA callers, P57, P07, V25N93 noun write through it |
| core -> csm/guidance/programs (ecadr.c) | ERASABLE_ASSIGNMENTS address column | nassp-integration moved the map to src/erasable_map; calls INTO it (pinball machine-address nouns, V71/V72, DNDUMP, LM P07) are the address column: tools/depgraph.py ADDRESS_COLUMN, ARCHITECTURE.md |
| io/verbs -> csm/burns (V46 -> agc_job_dapfig) | POSTJUMP DAPFIG | agc_exec_jump(ctx, AGC_JOB_DAPFIG) (new exec primitive: the job goes on at a body by id) |
| io/verbs -> programs/displays (V82 -> R30) | POSTJUMP V82CALL | agc_exec_jump(ctx, AGC_JOB_ORBIT_PARAMETER_DISPLAY) |
| io/verbs -> programs/navigation (LANDMARK, MRKBUF1/2) | shared erasables | storage and the agc_erase_* accessors moved to src/core/erasable.c (not reset at power-on, as before) |
| io/verbs -> programs/alignment (OPTC1: SAC/PAC -> DESOPTS/T) | V41 N91 continuation | the OPTC1 job body moved to src/csm/optics/auto_optics_positioning.c (R52 makes the same hand-over); registry CSM only |
| io/verbs -> lm/radar (RRZEROK, LRPOS2K, RRDESEND, V56) | RRZERO, LRPOS2, RADNOOP, CLRADMOD: RADMODES + channel bit + waitlist task | moved to src/io/service/t4rupt_lm.c with RADMODES (as RRLIMCHK/SETTRKF in 7826e91); tasks stay in lm/radar, by id |
| io/verbs -> lm/radar (RRDESK2 -> RRDESNB) | FINDVAC'd job | job body moved to src/lm/radar/designate.c; registry LM only |
| io/verbs -> lm/optics (RRLOSDSP -> RRNBMPAC) | FINDVAC'd job | RRLOSDSP and its DELAYJOB continuation moved to src/lm/optics/lem_geometry.c; registry LM only |
| io/telemetry -> programs/navigation (DELVSLV) | DNADR DELVSLV (= DELVLVC) | read by noun data id through agc_dispatch_noun (exec/nouns, the documented noun-data exception); same dp value, same words |
| programs/alignment -> csm/alignment, lm/alignment (P52 seams, 8 pairs) | TC/TCF to the rope's own label | three registry rows with a CSM and an LM body (AGC_JOB_IMU_REALIGN_STARS, _RECHECK, _LANDING_SITE) + agc_exec_jump |
| programs/displays -> csm/burns (CANDEL -> DELRSPL) | CANDEL is CM-only (LM DELRSPL EQUALS SPLRET) | CANDEL/SPLRET job moved next to DELRSPL (src/csm/burns/external_delta_v.c); V82GOFF1 / V82GON1 jump to it by id (they now take ctx); R30 exports SPLRET1 and the TFF erasables |

Not done (out of the bounded scope): the extended-verb files stay in
io/verbs (design row V); no new upward edge needed it.

## Deviations
none (no behaviour change). Registry rows of moved vehicle-only bodies
(OPTC1, RRDESK2, RRLOSDSP, CANDEL) lost the other vehicle's handler, which
no path reached; job enum values shifted (ids are symbolic everywhere).

## Stubs
none

## Requested core additions
none requested; made directly (the lane edits include/agc for the moves):
agc_exec_jump (exec.h); agc_erase_mrkbuf/_set_mrkbuf/_mrkbuf2/_set_mrkbuf2,
agc_erase_landmark/_set_landmark (core/state.h); agc_rendz_clradmod,
agc_rr_zero, agc_radar_lrpos2 (io/service.h); agc_orbit_display_start_ticking,
agc_orbit_display_tff_debris (programs/rendezvous.h)

## Tests written
none new: the moves change no behaviour; tests/imu, tests/programs/align,
oracle_imu, oracle_align and `make lm-sim` are the acceptance.

## Open questions

## Lessons
- "Continue in a vehicle's job body" is TC/TCF/POSTJUMP in the AGC; in C
  it is agc_exec_jump(ctx, id) with a two-handler registry row, not a
  direct call into src/csm or src/lm. Use it only for jumps, never to
  launder a subroutine CALL (DELRSPL is a CALL: its caller CANDEL moved).
- Small mode-switch starters (RRZERO, LRPOS2, CLRADMOD) belong with the
  word they set (RADMODES, io/service), not with the program that also
  uses them; their waitlist tasks stay put and are reached by id.
- A downlist item whose owner sits above io is noun data: read it by id
  (agc_dispatch_noun) rather than moving the owner's storage.

## Program flow and decisions

_Final documentation pass (2026-10-14), HEAD 3b97dc8._ This lane moved
storage and job bodies so that no module calls a higher one; every move
kept the decision it moved (gates and lm-sim unchanged). The one new
control-flow primitive:

* **`src/exec/executive.c:agc_exec_jump(ctx, id)`**: a job goes on at a
  registry body by id, the C form of the AGC's TC/TCF/POSTJUMP to a
  label of the rope being flown. The registry row names the CSM and the
  LM body. Used for V46's DAPFIG (`src/io/verbs/extended_verbs_csm.c`),
  V82CALL (`src/io/verbs/extended_verbs.c`) and the three P52 seams
  (`src/programs/alignment/imu_realign.c`: AGC_JOB_IMU_REALIGN_STARS,
  _RECHECK, _LANDING_SITE; [imu-realign
  flowchart](../../docs/overview/flowcharts/imu-realign.md)).

Where the moved code now decides (each in its owner's section):
OGC/IGC/MGC buffers in `src/imu/gyro.c` ([imu](../../docs/modules/imu.md#program-flow-and-decisions));
FINETIME in `src/imu/mode.c`, the CDU trig and AX*SR*T in
`src/imu/cdu_trig.c`; DESOPTT/S in `src/io/service/t4rupt_csm.c`;
RRLIMCHK, SETTRKF, RRZERO, LRPOS2, RADNOOP and CLRADMOD with RADMODES in
`src/io/service/t4rupt_lm.c` ([sys-misc](../../docs/modules/sys-misc.md#program-flow-and-decisions));
MASS, ABDELV, MRKBUF1/2 and LANDMARK in `src/core/erasable.c`
([state-flags](../../docs/modules/state-flags.md#program-flow-and-decisions));
SXTNB in `src/programs/alignment/geometry_csm.c`; DELVSLV read for the
downlink by noun id through `src/exec/nouns.c:agc_dispatch_noun`.
