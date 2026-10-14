# 12. Reading further

The earlier chapters walked through the program by topic. This one is
about finding your own way: from any AGC label to its C and back, how
the project shows that the C does what the listing does, which
deviations it admits, and where the record of how it was made is kept.

## 12.1 From an AGC label to the C, and back

**Label to C.** `docs/overview/xref.md` is generated from the function
prologs (`python3 -I tools/xref.py`) and lists 3095 AGC labels with the
C function, file and line that translate each, and the rope file and
page where the label is defined. The row for chapter 3's routine:

```
| `CALCGTA` | C INFLIGHT_ALIGNMENT_ROUTINES p.1355; L INFLIGHT_ALIGNMENT_ROUTINES p.1249 | `agc_imu_calcgta` | [imu/alignment.c:139](../../src/imu/alignment.c#L139) | prolog |
```

"Kind" says whether the function *is* the label (`prolog`) or contains
its code as a marked section (`section`): small AGC routines that only
one caller used were often folded into that caller, with a block
comment naming the label and page where they begin. A label with
several rows is translated once per vehicle, or as a shared routine and
a vehicle variant.

**C to label.** Every function's prolog starts with the AGC label, the
listing file and the page, C for Comanche 055 and L for Luminary 099:

```c
 * CALCGTA (INFLIGHT_ALIGNMENT_ROUTINES.agc pp.1355-1356) -- gyro
 * torquing angles
```

and every file's prolog has an "AGC SOURCE" section naming the pages
the file covers and the differences between the two ropes. Trailing
comments on statements often quote the instruction they translate
(`/* VXM VSL1 */`, `/* BOV */`), which makes a line-by-line comparison
quick. Functions with no AGC counterpart say "AGC: none" and why.

**Names.** Programs, routines, verbs and nouns are named by what they
do, never by number (`docs/NAMING.md`, with the table of canonical
stems: P52 is `imu_realign`, R60 `attitude_maneuver`, V37
`change_program`). Labels that are already words keep them (KEPLER,
LAMBERT, SERVICER, BURNBABY). A job's id, function and file share the
stem: `AGC_JOB_IMU_REALIGN`, `agc_job_imu_realign`, `imu_realign.c`.

**The listings.** The Virtual AGC transcriptions of Comanche 055 and
Luminary 099 (`README.md`, "Source of truth"). Page numbers are the
original listing's "# Page" markers, which the transcriptions keep.

**The flowcharts and module notes.** `docs/overview/flowcharts/` draws
the scheduler paths and the major program sequences from the C control
flow; `docs/modules/<module>.md` has, for each part of the program, a
"Program flow and decisions" section listing every function that tests
a flag, raises an alarm or starts a job, and every shared erasable it
reads or writes.

## 12.2 How the tests show fidelity

The project's claim is not "it looks like the listing" but "it computes
what the listing computes". Several independent tiers check it
(`docs/ARCHITECTURE.md`, "Verification tiers"); each proves something
the one before cannot.

1. **Build and rules** (`make`, `make check`): the pedantic compiler
   with `-Werror`, and `tools/check_rules.py` for the Power of Ten
   rules -- no recursion, bounded loops, no heap, checked returns.
2. **Unit tests** (`make test`; `tests/<subsystem>/`, one binary per
   file): each routine on chosen inputs, linked against a debug library
   in which every failed `AGC_ASSERT` is reported.
3. **The oracle** (`tests/oracle/`, `tools/oracle/`). The real ropes,
   assembled from the listings by Virtual AGC's yaYUL, run in its
   emulator yaAGC. A probe (`tools/oracle/agc_probe.c`) enters a single
   routine at its label with chosen erasable contents and dumps the
   result; a spec file says what to poke and what to read
   (`tools/oracle/cases/nav_conic.spec`):

   ```
   # Each routine is entered at its label through INTPRET (Q = label, FBANK = its bank).
   ...
   q KEPPREP
   ...
   set RCV=00313,31474,00000,00000,00000,00000
   ```

   The C test feeds the same words to the C routine and compares the
   outputs word for word: "Every output must match the AGC word for word
   (tolerance 0)" (`tests/oracle/conic.c`, NOTES). About ten thousand
   such checks run in `make test`; where a residual of a few least
   significant bits remains it is explained (usually the -0 the C
   does not model) and the tolerance is documented per case.
4. **Scenarios** (`make scenarios`; `sim/scenarios/*.txt`). The whole
   program, built as `build/agc_sim`, driven headless through the same
   socket protocol the DSKY uses: key sequences in, displays and
   downlink out. Each script states the AGC behaviour it checks with
   listing pages -- for example P52 option 3 with two star marks
   (`sim/scenarios/imu_realign_csm.txt`).
5. **Virtual AGC's own GUI.** `build/agc_sim` is a drop-in for yaAGC
   (`README.md`, "Run it"): the unmodified DSKY and telemetry windows run
   against it.
6. **The LM Simulator** (`make lm-sim`): Virtual AGC's closed-loop LM
   simulator flown on this program and on yaAGC with one crew sequence
   -- IMU align, gyro torquing, the RCS DAP and T6 jet timing, a POODOO
   restart -- compared at 34 checkpoints.
7. **NASSP** (`make nassp-replay`, and the mission on a Windows host).
   NASSP, the Apollo add-on for the Orbiter simulator, links the library
   in place of yaAGC (`src/nassp/engine.c`). The replay restores five
   scenario images and compares uplinks and downlists word for word with
   NASSP's own emulator; the mission flew all ten milestones of Apollo
   11 -- insertion to splashdown -- and matched the stock run on every
   decision and on the end state (`README.md`, "Status";
   `docs/modules/nassp-integration.md`).

The mission tier found seventeen faults that every lower tier had
passed. They are the reason for chapter 2's word pairs and -0, chapter
5's "one erasable, one C object" and address map, chapter 3's push-down
list, chapter 6's loadable nouns; `docs/TRANSLATION_GUIDE.md` (section
8) lists each with the rule it taught.

## 12.3 Deviations and limitations

Wherever the C does something the AGC does not, the code says so with
the word DEVIATION in a comment -- 197 of them in 94 files
(`grep -rn DEVIATION src`). Most are local and small (a bound on a loop
the AGC left unbounded, a field split that the AGC packed into one
word). The ones that shape the whole program:

| Deviation | Where | Chapter |
|---|---|---|
| A job takes no AGC time; interrupts come between jobs | `sim/clock.c`, `exec/executive.c` NOTES | 4 |
| A self-rescheduling job runs at most once a centisecond | `exec/executive.c:agc_exec_run_one` | 4 |
| The landing-radar velocity read waits while the altitude read holds the radar | `lm/burns/servicer/landing_radar.c:agc_job_lrvjob` | 4, 10 |
| -0 is not a value of `agc_dp_t`; modelled where observable | `include/agc/core/word.h` | 2 |
| A software restart is deferred to the next service point | `exec/alarm.c`, `docs/overview/flowcharts/software-restart.md` | 7 |
| A restart "at the return address of PHASCHNG" reruns the job segment | `exec/phase_table.c:store_return_point` | 7 |
| V37 uses a type C phase in group 4 where the AGC uses 4.1 | `exec/change_program/sequence.c` NOTES | 7 |

And the known limitations (`docs/ARCHITECTURE.md`, "Known
limitations"): behaviour that exists only because of execution time
(1201/1202 overloads, how long a computation takes) is not reproduced;
a host scenario load restarts the computer rather than resuming it, so
a variable restart phase that names a rope address cannot be resumed;
a few downlink words the C cannot hold (the A and Q registers, rope
addresses) are skipped in comparisons; and two stubs remain, V27 and
the CM prelaunch IMU-test branches that need an erasable program not
in the rope (chapter 1).

## 12.4 Where the history lives

This guide and the rest of `docs/` describe the program as it is. How
it was made -- the plan, the order of work, the assignments of the
agents that translated each part, the reviews, every bug and how it was
found -- is kept separately in `history/` (`history/README.md`):

* `history/WORKLOG.md`, the lead's running log;
* `history/lanes/`, one diary per translation or review lane, with its
  findings and lessons;
* `history/waves/`, the notes of each tree-wide pass (naming, layout,
  status handling, formatting);
* `history/PLAN.md` and `history/STATE.md`, the plan with its definition
  of done and the final state.

`docs/TIPS.md` distils what translating AGC assembly taught, by topic,
for anyone who wants to do the same for another rope.

Back to the [table of contents](README.md).
