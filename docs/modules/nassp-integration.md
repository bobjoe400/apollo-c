# nassp-integration -- Apollo 11 in NASSP 9 on our AGC (PLAN item 10b+10c)

Assignment: docs/agents/nassp-integration.md. Study: docs/overview/nassp_integration.md.

Host layout (Windows, reached through WSL interop):
- C:\NASSP9 -- stock baseline (untouched).
- C:\NASSP9-dev -- robocopy /E /XJ of C:\NASSP9 (2026-10-12 14:28) with the five
  Modules\Server junctions recreated to point inside C:\NASSP9-dev; Orbiter_NG.cfg
  synced from C:\NASSP9 (ProjectApolloMFD + ApolloRTCCMFD active).
- C:\Users\cnama\Downloads\nassp-src\NASSP -- clone of rcflyinghokie/NASSP at 5470bf1
  (untrusted data; only its own CMake build is run).
- C:\Users\cnama\nassp-work -- our .bat drivers and build trees (vcvars64 + cmake):
  build-stock.bat (stock configure+build), build-stock\ (tree).
- Scratch: SCRATCH/nassp-integration/ (logs, symbol tables sym_csm.txt / sym_lm.txt
  from the yaYUL listings, tools/symtab.py, runs/*.scn saved states).

## Progress
- DONE phase 1 STUDY: docs/overview/nassp_integration.md (embedding, engine API,
  callbacks, every direct agc_t / Erasable access with file:line, uplink paths,
  pad loads, erasable map classes A-E, shim design, build/run plan, risks).
- DONE phase 2 STOCK BUILD (2026-10-12): NASSP 9 at 5470bf1 configured with CMake 4.4
  ("Visual Studio 17 2022" x64, RelWithDebInfo, ORBITER_INSTALL_DIR=FINAL_INSTALL_DIR=
  C:/NASSP9-dev) and built with MSVC 19.44: 0 errors, 36 DLLs installed straight into
  C:\NASSP9-dev\Modules\{ProjectApollo,Plugin,Startup} (Saturn5, Saturn1B, LEM, MCC,
  SIVB, ... ProjectApolloMFD, ApolloRTCCMFD). Run: `Orbiter_ng.exe -s "Project Apollo -
  NASSP\Apollo 11 - Launch" -T 120` from C:\NASSP9-dev: the session log (module loads,
  mission file, vessels, XRSound, scene) is line-for-line the stock C:\NASSP9 session
  log except the texture-memory figure; "Saturn5.dll [Build 261012]" = our build.
  Saved state kept as runs/stockdev_launch_120s.scn (CMC unpowered at T-4h: CYCLECOUNTER 0,
  DSKY blank -- the launch scenario starts with the CMC off).
- DONE phase 3 SHIM (first cut, 2026-10-12):
  - src/core/ecadr.c (now src/erasable_map/address_map.c): address map onto typed erasables (both ropes, 45/44 entries: REFSMMAT,
    TEPHEM, state vectors, UPSVFLAG+RRECT/VRECT/TET uplink buffer, times, masses, DAP words,
    display registers); raw words kept for mixed-sign V71 loads; tests/core/ecadr.c (112 checks).
  - src/sim: clock.c sim_clock_count (one scaler count; tick = 32 of them), sim_irq_pending,
    host hooks sim_host_downrupt_external / sim_host_radar_read (socket.c: agc_sim values);
    rope.c made portable (the /proc default-rope lookup moved to main.c).
  - src/nassp/nassp_engine.c: yaAGC API (agc_engine_init, agc_load_binfile, agc_engine,
    WriteIO, UnprogrammedIncrement, ...) over libagc + clock; Erasable mirror with shadow
    diff; NASSP channel renumbering; LM thrust/ALTM drives. Linux smoke harness (scratch
    harness/smoke.c against NASSP's header): power-on P00 V37 flash, RSET, V37E00E, V35,
    V16N36 on both ropes.
  - src/nassp/CMakeLists.txt (apollo_c_agc static lib) + patches/0001-apollo-c-agc-backend.patch
    (NASSP top CMake: APOLLO_C_DIR cache path; Saturn5/Saturn1B/LEM drop agc_engine*.c,
    agc_utilities.c, Backtrace.c and link apollo_c_agc).
- DONE phase 4 build: C:\Users\cnama\nassp-work\build-agc.bat (patched copy nassp-agc\,
  sources synced to nassp-work\apollo-c by SCRATCH/nassp-integration/sync.sh): libagc compiles
  with MSVC 19.44 /std:c11, 0 warnings; Saturn5.dll + LEM.dll on our AGC installed in C:\NASSP9-dev.

- Commit note: the ecadr map, tests/core/ecadr.c and the src/sim changes landed in b0a29df
  (swept into the lm-simulator commit by the lead); src/nassp is excluded from LIB_SRCS (e48fec9).
- DONE ecadr map extended: DELVSLV (external dV uplink, C 3404 / L 3433) and XSMD (desired
  REFSMMAT, C 0306 / L 3606).
- Host test rig (SCRATCH/nassp-integration): C:\NASSP9-ref = second copy of stock C:\NASSP9
  for reference runs (C:\NASSP9 itself never run); NonsphericalGravitySources = TRUE in both
  dev and ref Orbiter_NG.cfg; test scenarios Scenarios\apollo-c\*.scn = NASSP mission scenarios
  + "Script apollo-c/<name>" (Lua in Script\apollo-c: lua/common.lua helpers: DSKY keys via
  set_panelswitch press+release, switches, CAPCOM via send_bufferedkey TAB/2, CMC dumps via
  get_agcerasable/get_agcchannel to apollo-c-<name>.log). start.sh launches ONE session (kills
  any first); poll.sh polls <= 50 s, closes Orbiter gracefully at the script's "end" line and
  collects Orbiter.log, the Lua log and (Current state).scn into runs/<tag>/.
- Runs so far: T-20m stock vs ours (no crash; ours restarts with PROG 02 + V37 flash + RESTART
  lamp where stock keeps P02 running: running jobs cannot be restored, documented). Launch from
  the pad dropped (lead: no real-time phases). Insertion uplink: first stock attempt needed the
  UPLINK switch (CMUplinkSwitch) at ACCEPT; now scripted.

- DONE hang fix (2026-10-12): the dev Insertion run hung at 50x (lead killed it). Cause: shim cost,
  not a loop: full 2048-word mirror export through the ECADR map every cs = ~15 ms CPU per AGC
  second; NASSP steps the AGC by warp x real dt, so frames grew until Not Responding. Now counters
  per scaler count, mirror import per cs, full export per second: ~6 ms CPU per AGC second.
  Watchdog sidecar apollo-c-agc-<vehicle>.log (heartbeat per AGC minute, slow counts, GOJAMs).
  Linux replay harness SCRATCH/nassp-integration/harness/scn.c (scenario AGC_BEGIN image through
  the shim, NASSP cadence) reproduces and times it.
- DONE restore fix: the map now carries the whole 42-word state vector blocks (temp 01502, CSM
  01554, LM 01626: RRECT..XKEP via agc_integ_perm_get/set) and LAT/LONG/ALT. Before, RCV/DELTAV
  etc. stayed 0 after a scenario restore and P00 integration (STATINT1) exited at once; the replay
  now steps TETCSM 5,120 -> 13,5120 -> 22,33320 -> 32,21520 exactly as stock does.
- DONE map: LAT(SPL)/LNG(SPL) (C 03400/03402 -> agc_entry_state()): R30's DELRSPL copies them
  into LAT/LONG (P30-P37 C p.643 GETARG), so unrestored they gave LAT=0 and HA/HP ~5 nmi high.
  Raw words now refreshed element by element (unchanged elements keep the AGC's mixed signs).
- DONE milestone 1 (Insertion + first MCC CMC uplink, Lua scenario "apollo-c\A11 insertion uplink",
  stock C:\NASSP9-ref then dev, zero human input, 20x warp): V82 after load stock 104.3/101.9 nmi,
  ours 104.1/102.0; MCC CSM SV uplink (V71 1501): both P27 with the same V21N01 sequence, back to
  P00; V82 after the uplink stock 110.2/107.0, ours 109.7/107.3 (each MCC computed its own vector);
  V06N43 identical. Logs: SCRATCH/nassp-integration/runs/{ref,agc}_v82c/.
- DONE milestone 2 (TLI, scenario 03, Lua "apollo-c\A11 TLI", stock then dev): V82 at load stock
  111.2/104.0, ours 111.1/104.0; P47 (V37E47E) through the S-IVB burn on both; post-TLI CSM vector
  TET 77,2641 (stock) vs 77,2626 (ours): 0.4-1.0 km apart = the 0.11 s of travel. PASS.
- DONE map for the burn/entry milestones: IMU compensation (1452-1470, GCOMPSW 1477), AZO/-AYO/AXO/
  504LM, CSM ephemeris TIMEMO/VECOEM/RESO/VESO/OMEGAES, EMDOT, DVTHRESH, EK1-3VAL, FANG, ETDECAY,
  ESTROKER, EKPRIME, EKTLX/I, EREPFRAC, PACTOFF, YACTOFF, HBN10..HBD32, LADPAD/LODPAD/ALFAPAD/EMSALT,
  E3J22R2M/E32C31RM (new getter agc_integ_get_lunar_coeffs in include/agc/nav/integration.h).
- Checklist automation: SCRATCH tools/cl2lua.py compiles NASSP's own "Apollo 11 Checklists.xls"
  (134 sheets: Item = panel switch name, Position, HIDDEN_DELAY / MISSION_TIME, Call Group) into
  lua/cl_csm.lua; lua/common.lua ac.run(sheet) executes a sheet on either build: switch rows via
  set_panelswitch, DSKY rows via keys (PRO as V33E, the AGC's PROCEED verb), "F VV NN" / "VV NN"
  rows wait for that display, "T-m:ss" rows wait on TIG - TIME read from the CMC, "Engine Cutoff"
  waits on channel 11 bit 13, manual rows collected for the report; ac.subst fills the digits the
  crew types by hand (V48 DAP data).
- DONE src/sim/socket.c send_to (closed-loop-fixes finding): a partial or EAGAIN send is retried
  (rest of the packet after up to 10 ms for buffer space, 20 tries); a client still not draining
  loses the packet (none sent) or is dropped (part sent), both reported; tests/sim/harness 59/59.
- DONE layering (lead, 2026-10-12): the address map left core. src/erasable_map/address_map.c +
  include/agc/erasable_map/address_map.h (agc_erase_read_ecadr / agc_erase_write_ecadr, the
  flagwords and every named erasable; top of the graph, reaches csm/lm/guidance/programs/nav/
  imu/io storage); src/core/ecadr.c keeps only the raw image (agc_erase_raw_read/_write, reset),
  no include above core. Test moved to tests/erasable_map/address_map.c (125 checks). Callers
  (pinball load/scale, telemetry, extended verbs, uplink update, LM systems test, NASSP shim, 3 tests)
  include the new header.
- Offline stock reference: NASSP's yaAGC engine compiled on Linux (SCRATCH yaagc_nassp/, GPL, not in
  the repo) + harness/ref.c replays a scenario's AGC image beside harness/scn.c (our shim): the P00
  integration step of the Insertion image gives the CSM state vector block identical to stock.
  (Keys into ref.c do not yet reach pinball; integration-only checks work.)
- Do not run Linux test loads during a host run: the stock AGC at 50x went to garbage (MODREG 227,
  FAILREG 207) when frames stretched under CPU load; that run was discarded.

## Lessons
- Orbiter_ng.exe only launches Modules\Server\Orbiter.exe and exits: wait on Orbiter.exe.
- Orbiter.exe -s SCN -T SECONDS runs a scenario unattended and quits, saving
  Scenarios\(Current state).scn (AGC section included).
- NASSP sources trip grep's binary detection; use grep -a.
- Never run two Orbiter sessions at once: NASSP's PCM telemetry binds TCP 14242 (MCC uplinks
  ride it). One session, then kill, then the next.
- NASSP pushbuttons fire callbacks on state CHANGE: a Lua key press is set 1 then set 0.
- CMC uplink needs CMUplinkSwitch AND UPTLMSwitch at ACCEPT (csm_telecom handle_uplink), and
  the CAPCOM "Ready for uplink" answer (TAB, 2 on the focused Saturn).
- NASSP's Lua set_agcerasable stores the address as the value (LuaCall_SetAGCErasable reads
  argument 3 twice): unusable for pokes.
