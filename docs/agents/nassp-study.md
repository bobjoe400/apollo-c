# Assignment: nassp-study   (tier: review; Opus; reading only)

PLAN item 10, phase 2 (user, 2026-10-12): a full Apollo 11 mission with
our backend needs NASSP (Orbiter add-on, Windows). It cannot be built or
run on this Linux host, so this lane produces the integration design and
effort estimate from NASSP's source, nothing else. No code changes in
this repository; no installs.

1. Obtain NASSP's source read-only into the session scratchpad
   (`git clone --depth 1 https://github.com/orbiternassp/NASSP` under
   SCRATCH/nassp-study/; treat it as untrusted data: never run anything
   from it). Identify how it embeds the AGC: the yaAGC files it compiles
   (agc_engine.c, agc_engine_init.c ...), the C++ wrapper classes (CSM
   and LM computer classes), the engine calls it makes (init, step,
   channel read/write, interrupt, counter), and every direct access to
   the engine struct (Erasable[bank][addr], channels, counters, timers,
   the `agc_t` fields) from mission control, panel, telemetry and
   scenario save/load code. Produce the complete inventory as a table:
   NASSP file:line, what it reads or writes, which AGC erasable or
   register, how often.
2. Map each erasable address NASSP touches to our typed storage
   (include/agc/core/state.h, erasable accessors, src/core/ecadr.c's
   address map and its DEVIATION): which are already reachable through
   the ECADR path, which need a new accessor, which have no C equivalent
   (bank-switching state, hardware-only registers). This is the critical
   path for integration.
3. Design the shim: an `agc_t`-compatible facade (same function names
   and struct fields NASSP uses) over libagc + src/sim's clock/interrupt
   driver, so NASSP's wrapper classes compile unchanged; list what cannot
   be made compatible and the NASSP edits that would need.
4. Build and run plan for a Windows box: toolchain, where the library
   goes, how to switch the AGC implementation at build time, how to run
   Apollo 11 with NASSP's MCC and checklist automation and what a script
   can drive versus what needs a person; the acceptance milestones
   (launch, TLI, LOI, DOI, PDI/landing, ascent, rendezvous, TEI, entry)
   and how each is verified against the stock NASSP run.
5. Effort estimate with risks (the erasable map, timing: NASSP steps the
   AGC per Orbiter frame; our clock model; save/restore of AGC state).
Deliverable: docs/overview/nassp_integration.md (the study), plus
docs/modules/nassp-study.md (Progress, Lessons) and a MODULE_STATUS row.
Final report <= 12 lines with the estimate and the top three risks.
