# 1. What the AGC is, and what this code is

The Apollo Guidance Computer flew in both Apollo spacecraft: one in the
Command Module (the CMC) and one in the Lunar Module (the LGC). The C
models one machine for both (one library, section 1.3); what differs is
the program held in each one's fixed memory. This chapter describes the machine as the software
sees it, the two programs of Apollo 11, the crew's keyboard, the
vocabulary of programs and routines, and what this C translation keeps
and leaves out.

## 1.1 The machine as the software sees it

**Words.** Every memory cell and register holds a 15-bit word in ones'
complement: a sign bit and 14 magnitude bits. The hardware stored a
sixteenth bit, parity, which the program never sees; the rope loader
drops it (`sim/rope.c`, NOTES: "each word shifted left by one (bit 0 =
parity, dropped)"). Chapter 2 is about what can be done with 15 bits.

**Two memories.**

* *Erasable* memory is the read-write store: 2048 words in eight banks
  E0..E7 at addresses 0000-3777 octal (`core/ecadr.c`, file prolog).
  Everything the program remembers -- state vectors, flags, timers,
  display buffers, the executive's tables -- has to fit there.
* *Fixed* memory, the **rope**, is read-only: 36 banks (044 octal) of 1024
  words, about 36 K words (`include/agc/io/service.h`, `AGC_ROPE_BANKS
  044`; `sim/rope.c`, `g_rope[...] /* fixed memory, bank b at b * 1024 */`).
  The program and all its constants live there and cannot change in
  flight. Any number a mission needs that is not known when the rope is
  made (a landing site, a vehicle mass) is an erasable that the ground
  loads before launch -- a *pad load* (chapter 5).

**Channels and counters.** The computer talks to the spacecraft through
15-bit I/O channels (numbered 1 to 35 octal) and a set of *counter
cells*: erasable words that the hardware itself increments or decrements
when a pulse arrives. Channels carry discrete signals (a switch, a lamp,
a jet on or off: `include/agc/core/channels.h`); counters carry
accumulated quantities: the accelerometer (PIPA) pulses, the gimbal-angle
(CDU) readings, the radar reading RNRAD, the throttle command THRUST, the
uplinked word INLINK (`core/counters.c`, file prolog). Some counters are
timers: TIME1-TIME6 count clock pulses and interrupt the program when
they overflow (chapter 4).

**What it is connected to.** The inertial measurement unit (IMU: a
gimballed platform holding three gyros and three accelerometers), the
CDUs that digitise its gimbal angles, the optics (the CM's sextant and
scanning telescope; the LM's alignment optical telescope), the LM's
rendezvous and landing radars, the engines and their gimbals, the
reaction-control jets, the hand controllers, the telemetry links to the
ground, and the **DSKY**.

## 1.2 The DSKY

The DSKY (display and keyboard) is the crew's only window into the
program. The comment at the top of `include/agc/io/dsky.h` quotes the
listing's own definition:

> "THE LANGUAGE OF COMMUNICATION WITH THE PROGRAM IS A PAIR OF WORDS
> KNOWN AS VERB AND NOUN."

The display has three two-digit fields and three signed five-digit
registers; in outline (the header lists the digit positions, from
PINBALL):

```
  PROG  MD1 MD2                    (major mode: the running program)
  VERB  VD1 VD2      NOUN  ND1 ND2
  R1    +- D1 D2 D3 D4 D5
  R2    +- D1 D2 D3 D4 D5
  R3    +- D1 D2 D3 D4 D5
```

plus warning lamps (OPR ERR, KEY REL, UPLINK ACTY, PROG, RESTART, ...).
The keys are the digits, `+`, `-`, VERB, NOUN, ENTR, CLR, RSET, KEY REL
and PRO (`enum agc_dsky_key`). A **verb** says what to do (display,
monitor, load, start a program); a **noun** says what to do it to (a
time, an attitude, a velocity). "V16 N36 ENTER" means "monitor (16) the
clock (36)": R1-R3 show hours, minutes and hundredths of seconds, and
the monitor refreshes them every second (MONREQ, a one-second waitlist
task, `io/dsky/pinball/monitor.c`). The program uses the same vocabulary towards the
crew: it shows V06 N40 to display a burn's time to go, or *flashes* the
VERB and NOUN to ask a question, which the crew answers with PRO
(proceed), V34 (terminate) or data and ENTER. Chapter 6 is the full
story.

## 1.3 Two programs: Comanche 055 and Luminary 099

Apollo 11's CMC ran **Comanche 055** (Colossus 2A) and its LGC ran
**Luminary 099** (Luminary 1A). They share about 45 log sections -- the
executive, the waitlist, the interpreter, the conic routines, the
orbital integration, pinball, much of the alignment code -- and each has
its own vehicle-specific programs (`docs/overview/README.md`, section 1).

The C keeps one library, `libagc.a`, for both. Shared code is written
once; code only one rope has lives in `src/csm/` or `src/lm/`; small
differences inside shared code test `agc_state_vehicle()`
(`core/state.c`), which is set once at power-on by `agc_power_on(v)`
(`exec/change_program/idle.c`). The vehicle is named by the rope the
host loads: `sim/rope.c` refuses any rope other than Comanche055 or
Luminary099, because "this program is those two translated and cannot
run another rope" (NOTES).

## 1.4 Programs, routines, verbs, nouns

The flight software is organised around a small set of words that recur
through this guide (`docs/NAMING.md` has the full table):

* A **program** (P*nn*) is a *major mode*: a self-contained activity such
  as P52 (IMU realign), P40 (SPS burn), P63 (lunar braking phase). Exactly
  one program runs at a time; its number is in the erasable MODREG
  (`agc_erase_modreg()`, `include/agc/core/state.h`: "MODREG major mode
  (program)") and is shown in PROG. The crew starts a program with V37
  (chapter 7); some programs start the next one themselves (the landing
  programs P63 -> P64 -> P66, the entry programs P63 -> P67). The list of
  programs a crew may select is the rope's table FCADRMM1, one row per
  program with its start priority and downlist, highest number first;
  in C it is `cp_modes_csm[]` (`exec/change_program/csm.c`) and its LM
  twin in `exec/change_program/lm.c`:

  ```c
  /* FCADRMM1 / PREMM1 / DNLADMM1 (C pp.206-209), highest mode first. P53
   * EQUALS P51 and P54 EQUALS PROG52; GTSCPSS1 is the P01 lead-in.        */
  static const exec_cp_mode_t cp_modes_csm[] = {
      {79, CP_PRIO13, CP_RENDEZVU, AGC_JOB_STABLE_ORBIT_MIDCOURSE_OTHER_VEHICLE},
      ...
      {40, CP_PRIO13, CP_POWERED, AGC_JOB_SPS_BURN},
      ...
      {1, CP_PRIO20, CP_COSTALIN, AGC_JOB_PRELAUNCH_INITIALIZATION}
  };
  ```

  beside the listing's (FRESH_START_AND_RESTART.agc, C p.206)

  ```
  FCADRMM1   EQUALS
             FCADR   P79
             FCADR   P78
  ```

* A **routine** (R*nn*) is a piece that programs or verbs call and that
  does not change the major mode: R30 orbit parameter display (started by
  V82, `io/verbs/extended_verbs.c`, case 82), R60 attitude manoeuvre, R02
  IMU status check. Many programs are mostly a sequence of routines.
* A **verb** (V*nn*) is a crew command. Verbs below 40 are *regular*
  verbs handled by pinball itself (display, monitor, load, V37 change
  program); "verbs 40 and up are extended verbs" that pinball hands to
  a job of their own (`io/dsky/pinball/verbs.c`, VERBFAN, CM p.325):
  V82 orbit parameters, V99 engine enable, and so on.
* A **noun** (N*nn*) names data to show or load: a noun table entry gives
  up to three components, each an erasable address and a scale routine
  (chapter 6).

The C uses none of these numbers in identifiers; it names each thing by
what the listing says it does: P40 is `agc_job_sps_burn`, R30 is
`orbit_parameter_display`, V37 is `agc_change_program`. The number stays
in the comment. Throughout this guide the number comes first, since
that is what the crew and the listing use, and the C name follows.

## 1.5 Jobs, tasks and interrupts in one paragraph

The AGC does many things at once on one processor. Long computations
are **jobs**, scheduled by priority by the *executive*; short actions
that must happen at a given time are **tasks**, run by the *waitlist*
when a timer expires; hardware events (a key press, a timer overflow, a
radar reading) raise **interrupts**. A program is never one function: it
is a chain of jobs and tasks that start one another, sleep while they
wait for the crew or a device, and record restart points so that a
power glitch does not lose the burn. Chapter 4 explains this machinery,
chapter 7 the restarts.

## 1.6 Two languages: basic and interpretive

The AGC's instruction set (*basic* code) is small: add, subtract,
multiply, divide on single words, a few transfers and tests. The
navigation and guidance equations need double-precision vectors and
matrices, so most of the flight program is written in a second language,
the **interpretive** language, executed by an interpreter in the rope:
one interpretive instruction is "add these two double-precision
vectors" or "unit vector of this". Chapter 3 shows how the C turns that
language into a library (`include/agc/core/interp.h`).

## 1.7 What this C is

This is a **translation, not an emulator**. No AGC instruction is ever
executed. The rope image can be loaded, but only as data for the
self-check's bank sums: "no instruction is ever fetched from it"
(`sim/rope.c`, NOTES). Each AGC routine is a C function that computes
what the routine computes, on the same fixed-point numbers, with the
same branches, alarms and scheduling. The rules followed are in
`docs/TRANSLATION_GUIDE.md`: "Translate meaning, not instructions.
`CCS X` followed by four branches is a sign test."

Every translated function carries a prolog that names the AGC label,
the listing file and page, so the C can be read beside the listing. For
example the job-creation routine NOVAC (`exec/executive.c`):

```c
/**********************************************************************
 * NOVAC (C p.1208 / L p.1103): enter a basic job that needs no VAC area.
 ...
agc_status_t agc_exec_novac(agc_prio_t prio, agc_job_id_t job)
```

"C p.1208" is page 1208 of the Comanche 055 listing, "L p.1103" page 1103
of Luminary 099. The listings are the Virtual AGC transcriptions
(`README.md`). To go the other way, from an AGC label to its C,
`docs/overview/xref.md` lists every label with its function and line
(chapter 12).

The code also follows NASA/JPL's "Power of Ten" rules
(`docs/CODING_STANDARD.md`), and three of them shape what the reader
sees:

* *No function pointers.* Every job and task is an enum id
  (`AGC_JOB_SPS_BURN`, `AGC_TASK_TIGBLNK`), and one generated `switch`
  per vehicle and kind (`exec/dispatch.c`) calls its function. This is
  the C form of the AGC's 2CADR (two-word address) tables.
* *No recursion.* Where the AGC jumps from deep inside a job into the
  restart code, the C records a request and serves it when the job
  returns (chapter 7).
* *Bounded loops, no heap, short functions.* Iterative solvers use the
  AGC's own iteration limits; erasable memory is static storage; long
  AGC routines are split at their own labels.

## 1.8 What was deliberately not translated

* **The machine itself.** Registers, banks and addresses are gone:
  "Bank switching (BANK, EBANK=, SETLOC, 2CADR, BBCON, superbank) is
  dropped; there is one flat address space" (`docs/TRANSLATION_GUIDE.md`,
  section 3). Where something outside the program does see erasable as
  numbered memory (pinball's machine-address nouns, the uplink, the
  downlink, a scenario load), an explicit address map stands in for it
  (`erasable_map/address_map.c`, chapter 5).
* **Timing.** A C job takes no AGC time: everything it computes happens
  at the instant it is dispatched, and interrupts are taken between jobs
  and between clock counts, never inside one (`sim/clock.c`, NOTES,
  "DEVIATION (timing)"). Behaviour that exists only because the
  1960s processor took time to compute -- `docs/ARCHITECTURE.md`
  ("Known limitations") names "1201/1202 overloads" and "how long a
  computation takes before a deadline" -- is therefore an expected
  deviation and is not modelled; the
  alarms themselves are translated and are raised if the executive's
  tables really fill. Chapter 4 lists the two places where an ordering
  had to be imposed to stand in for execution time.
* **The world.** There is no spacecraft model here. `src/sim/` is only
  what makes the library a drop-in for Virtual AGC's `yaAGC`: the clock
  and interrupt driver and the socket protocol to the existing DSKY,
  telemetry and simulator programs (`sim/clock.c`, "the harness's
  software clock and interrupt driver"). The vehicles are flown by
  existing tools: Virtual AGC's LM Simulator and NASSP (Orbiter's Apollo
  add-on), which link this library in place of the emulator (chapter 12).
* **Two stubs.** V27 (display fixed memory) has no fixed memory to show,
  and the CM prelaunch IMU performance-test drift branches end in an
  erasable-memory program that is not in the rope; each stub says so
  in its prolog (`io/dsky/pinball/monitor.c:pin_dspfmem`,
  `csm/prelaunch/gyrocompass.c`, `csm/prelaunch/align.c`).

## 1.9 Finding your way around the tree

| Folder | What it holds | Chapters |
|---|---|---|
| `src/core/` | the numeric model, the interpreter library, shared erasables, flags, channels | 2, 3, 5 |
| `src/exec/` | executive, waitlist, interrupts, alarms, restarts, V37 | 4, 7 |
| `src/io/` | DSKY and pinball, noun tables, T4RUPT, telemetry, extended verbs | 6 |
| `src/imu/` | IMU modes, gyro torquing, compensation, attitude manoeuvres | 8, 9 |
| `src/nav/` | conic routines, orbital integration, measurement incorporation | 8 |
| `src/guidance/`, `src/programs/` | shared burn, targeting, rendezvous and alignment code | 8, 9 |
| `src/csm/`, `src/lm/` | vehicle-only programs: burns, DAPs, entry, landing, ascent | 9, 10, 11 |
| `src/erasable_map/` | erasable by AGC address | 5 |
| `src/sim/`, `src/nassp/` | the host: clock, sockets, NASSP engine (not flight code) | 4, 12 |

The layering (`docs/ARCHITECTURE.md`) is core < exec < io/imu/nav <
guidance = programs < csm|lm; a lower layer never calls a higher one
except through the documented exceptions (the job switch, the interrupt
switch, the noun data, V37).

Next: [2. Numbers](02-numbers.md).
