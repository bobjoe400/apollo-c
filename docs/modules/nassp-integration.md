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

## How to run it

Light tier, Linux only, about a minute, no Orbiter:

    make nassp-replay

replays the CMC images of three NASSP 9 Apollo 11 scenarios (sim/nassp/images:
02 Insertion, 03 TLI T-15m, 05 MCC-2 T-15m; the scenario's AGC_BEGIN..AGC_END block,
GPL data from NASSP) on two engines built from sim/nassp/replay.c: ours (src/nassp/
nassp_engine.c, the shim NASSP's vessels link, over build/libagc.a) and stock (NASSP's own
yaAGC, src_sys/yaAGC at 5470bf1). Each image: RSET, V37E00E, P00 for 20 minutes of AGC
time (one integration step of each state vector), then an uplink sent as NASSP's PCM sends
MCC uplinks (INLINK + UPRUPT, 0.1 s apart): the CSM state vector (V71 21 words at 1501) or
a P30 target (V71 12 words at 3404), V33. The CSM/LM state-vector blocks after each step,
UPBUFF, UPSVFLAG, DELVSLV/TIG, MODREG and DSPTAB are compared word for word; DP pairs by
value (a -0/+0 high word is representation) and within TOL_LSB = 2 for the integrator
residuals documented in review-nav-integ.md (every pair off is listed). NASSP's yaAGC is
not in this repository: the target downloads the eight src_sys/yaAGC files from GitHub at
the pinned commit into build/nassp/yaagc, or uses NASSP_YAAGC_DIR=<NASSP>/Orbitersdk/
samples/ProjectApollo/src_sys/yaAGC; with neither it prints SKIP and exits 0.
`python3 -I tools/nassp_replay.py --keep` leaves both transcripts in build/nassp.

(The other closed loop, `make lm-sim`, needs no setup either: it fetches Virtual AGC's
Contributed/LM_Simulator, a sparse checkout of virtualagc at c239e9c, into
build/virtualagc on first use; LMSIM_DIR=<virtualagc>/Contributed/LM_Simulator uses
another checkout. Tcl/Tk (tclsh) must be installed.)

Full tier, Windows host (C:\NASSP9-dev, one Orbiter session at a time): build the
DLLs with C:\Users\cnama\nassp-work\build-agc.bat after syncing the sources
(SCRATCH sync.sh), then run an `apollo-c\<scenario>` with `Orbiter_ng.exe -s ... -T N`;
each scenario's Lua script (Script\apollo-c) drives the checklist workbook unattended
and writes apollo-c-<name>.log; compare with the same run from C:\NASSP9-ref (stock).

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
- 2026-10-13 resume: tree green at HEAD with the map move (callers include
  agc/erasable_map/address_map.h; pinball load.c / scale.c include order fixed by
  tools/format.sh). Host DLLs are now built from `git archive HEAD` plus this lane's
  working-tree files (SCRATCH sync.sh), never from another lane's uncommitted edits.
- FLIGHT BUG (milestone 3, found 2026-10-13): MCC-2 on our build burned 2.53 s where stock
  burned 2.21 s (N85 -2.4/-0.2/-0.2 vs -0.3/-0.1/0.0 ft/s). Cause: the CSM address map had
  EK3VAL as 2 words (1773-1774) ahead of FANG (1774), so the FANG pad load landed in
  K3VAL's low word and FANG stayed 0: S40.13 TIMEBURN (P40-P47 p.727) then found TEMP2 = 0,
  took S40.133 (steer, IMPULSW clear, TGO unchanged) and the cutoff came from STEERING
  instead of the impulsive TGO waitlist. Fix: src/erasable_map/address_map.c FANG entry
  first, a FANG store also sets K3VAL's low word (K3VAL = EK3VAL,FANG as DDV reads it);
  tests/erasable_map/address_map.c +5 checks. Offline (Linux replay of the MCC-2 image,
  V37E40E .. V99 PRO): TGO 0,341 (225 cs) and engine off at the new TIG, as stock.
- `make nassp-replay` (light tier, 2026-10-13): sim/nassp/replay.c + tools/nassp_replay.py,
  images sim/nassp/images/{insertion,tli,mcc2}.agc. Results: insertion 2 integration steps +
  CSM SV uplink identical (23 words differ only as -0/+0 high words of mixed-sign pairs);
  TLI 2 steps + P30 target uplink identical except DELTAV.y -1, NUV.x +2, NUV.y -2 LSB (the
  documented integrator residuals, review-nav-integ N4/N9/N10); MCC-2 P30 target uplink
  identical. ~25-50 s.
- DONE milestone 3 (MCC-2, 2026-10-13, after the FANG fix): stock vs ours VG/dVM 0.3/15.9 vs
  0.3/15.9 ft/s, N85 -0.3/-0.1/0.0 vs -0.3/-0.1/-0.1, V82 HP 49.7 vs 49.8 nmi, V99 at TIG-5 and
  SPS on time on both. Logs runs/{ref_mcc2,agc_mcc2}.
- Script finding (2026-10-13, user screenshot): NASSP's PRO key is "DskySwitchProg" on CSM and
  LEM; the helper pressed "DskySwitchProceed" (unknown: NASSP only prints "Switch not found")
  so every PRO of milestones 1-3 fell back to V33E on both builds (equivalent for P30/P40).
  common.lua now presses DskySwitchProg, checks every switch name (get_panelswitch = -1) and
  preflights the sheets: an unknown name aborts the run loudly (FATAL ..., end).
- DONE milestone 4 (LOI-1, scenario 06, real PRO presses, 2026-10-13): stock vs ours burn
  373.90 vs 373.97 s, VG/dVM 0.4/2924.9 vs 0.4/2925.0 ft/s, N85 -0.4/+0.1/+0.1 vs -0.5/-0.1/-0.1,
  V82 HA/HP 169.9/59.5 vs 169.7/59.5 nmi, P30 N81/N42 identical, V99 at TIG-5 on both; 0 shim
  errors. runs/{ref,agc}_loi1 (stock with V33E PROs: runs/ref_loi1_v33, same result).
  Display: V06N40 R1 counts down identically before ignition; during the burn it is time to
  cutoff, re-estimated every 2 s (jumps early in the burn on both). A 1 s Lua read of DSPTAB on
  ours aliases against the shim's once-per-second erasable export (repeats/skips a second);
  the on-screen DSKY follows the channel 010 relay words (every count), unaffected.
- MAP FINDING (2026-10-13, TEI dev run, LGC sidecar "errors 1"): restored LM images left the
  LM DAP pad loads at 0 (no address): 1/ACCSET then clamped LEMMASS to HIASCENT = 0 and
  lm_alone's AGC_ASSERT(LEMMASS > 0) returned BAD_ARG (job 652 = 1/ACCSET, accs.c:364; found by
  the Linux replay of the TEI scenario's Eagle image on the debug library). Comparing the map
  with NASSP's LM pad-load list (lmpad.txt, 269 words) found 73 unmapped words. Mapped now (LM):
  HIASCENT, ROLLTIME/PITTIME, DKTRAP..DKDB, IGNAOSQ/R, MASS, RADSKAL/SKALSKAL, DELQFIX,
  LRALPHA..LRWVFF, LRHMAX/LRWH, RPCRTIME/RPCRTQSW, TNEWA, X789, RANGEVAR/RATEVAR, RVARMIN/
  VVARMIN, WRENDPOS..TRUNVAR, ATIGINC/PTIGINC (new accessor agc_lm_dap_padload in
  include/agc/lm/dap.h); tests/erasable_map/address_map.c 157 checks. The DOI, PDI, liftoff
  and LM-powerup Eagle images now replay without a BAD_ARG. Still unmapped (no typed home or
  not flight-relevant): DUMPCNT, AGSK (AGS clock, getter only), DOWNTORK..NEGTORKV (jet torque
  telemetry sums); CSM: ENTRYVN, PGNCSALT, HORIZALT, TRANSM1, the CSM W/variance pad,
  AZIMUTH/LATITUDE/LAUNCHAZ, TAZEL1, DELVTPI, the Saturn takeover block (AOG .. POSEXIT,
  SATRATE ..), ECSTEER (milestones 1-4 and TEI matched without them; mapped next if a milestone needs it).
- src/nassp/nassp_engine.c now reopens stderr into apollo-c-agc-stderr.log, so src/sim's
  BAD_ARG diagnostics from inside NASSP land on disk.
- DONE milestone 6 (TEI, scenario 13, 2026-10-13): stock vs ours burn 160.26 vs 160.28 s, VG/dVM
  1.1/3306.8 vs 0.9/3306.7 ft/s, N85 -0.8/+0.8/0.0 vs -0.6/+0.7/-0.1, F16N44 HP 57.8 and TFF
  -59 59 on both; V99/ignition/cutoff within 0.04 s (AGC clock). Stock refused the first two
  V37E30E after RSET/V37E00E (1520, its P40 still ending); ours took the first (restore restarts).
  Scripts retry V37 (ac.select) on both.
- FLIGHT BUG 2 (TEI, ours only, fixed 2026-10-13): after the burn, P40 at F37, V82 then PRO on
  F16N44 left the DSKY blank (stock: F37 again); apollo-c-agc-stderr.log: job 135 =
  ORBIT_DISPLAY_AVERAGE_G returned BAD_ARG. V82GON1 runs SR30.1 on RN/VN; past pericenter on
  the escape hyperbola CALCTPER gives TPER < 0 and SKIPTPER stores -TPER > 0 as is (R30.agc
  p.523), but opd_orbit_params asserted -TPER <= 0 and ended the V82 job, so ENDEXT never
  restored the display. Fix: src/programs/displays/orbit_parameter_display.c stores -TPER of any
  sign (no assertion); tests/programs/rendezvous.c test 6 (V82GON1 on a departing earth
  hyperbola; fails before the fix). Reproduced offline from the run's saved state
  (SCRATCH harness/v82g.c on runs/agc_tei/state.scn, debug library).
- DONE milestone 7 (LM DOI, scenario 09, LEM Lua hook 0002 on both, 2026-10-13; warp schedule
  below): DPS ignition at TIG on both (AGC 366224.1 vs 366224.3, 1 s log sampling), cutoff
  366299.3 vs 366299.9; F16N40 VG/dVM 0.8/71.4 vs 0.2/71.9 ft/s; N85 +0.8/+0.1/0.0 vs
  +0.3/+0.1/-0.1; resulting orbit (Orbiter elements) peri/apo radius 1753.1/1851.4 km vs
  1752.5/1851.4 km. P30 N81 identical. Not automatable: manual throttle to 40% at TIG+15 s
  (DPS stays at the 10% start throttle on both), ullage, DEDA (AGS) entries, attitude checks.
- FLIGHT BUG 3 (DOI, ours only, fixed): P30 N42 R3 (DV REQ) showed 0 (stock +0072.1). Luminary
  ERASABLE_ASSIGNMENTS makes VGDISP = DELVSAB one erasable; the C kept burn.vgdisp and
  rz->delvsab apart and the LM S30.1 set only delvsab. Fix: the LM writers keep them equal
  (external_delta_v.c LM S30.1, dps_burn.c BDTOK, lambert_aimpoint_guidance_lm.c, csi.c on the
  LM); tests/guidance/guidance.c test_external_dv_vgdisp_lm (fails before). Offline replay of the
  DOI LGC image now shows N42 R3 +00721 as stock.
- Observation (not a bug): LM V83 (R31) recomputes continuously; the C pass takes no AGC time,
  so the job runs thousands of times a second (sidecar "busy" up to 76%) where the AGC runs a
  few; displays and results are the same; R31 ends at the next PRO/ENTER as in the AGC.
- Warp schedule (user, 2026-10-13), in common.lua for every milestone: script sets and holds the
  warp (a hand change is logged "WARP CHANGED" and re-asserted, every change logged); waits for
  T-m:ss rows more than 60 s out at 20x until 15 s before the row; 1x from TIG-75 s through
  cutoff, N85 and V82; MISSION_TIME rows 20x until 30 s before; entry EI waits 20x until 60 s
  before; DSKY/switch/flash rows and uplinks at 1x. The dev DOI run the user re-warped by hand
  was killed and discarded; DOI was flown again on both builds with this schedule.
- Milestone 8 (PDI, scenario 10) in progress. Script fixes on the way (stock runs that did not
  ignite were discarded, runs/discarded/): the "(PDI -N)" rows of the PDI sheet are timeline marks
  (now waits like T-N:00); an ENTER/PRO row that comes during a V06N18 attitude maneuver waits
  for the flash (R60 shows F50N18 again when done); the TIG-5 wait accepts V99 with any noun
  (P63 flashes V99N62, P40 V99N40; waiting for N40 delayed the PRO past TIG and stock P63 then
  showed V97 thrust fail). Alarm 00206 seen at PDI-5 on those runs came from the warp tumble
  (gimbal lock), see below; at 1x there is none.
- FLIGHT BUG 4 (PDI, ours only, fixed 2026-10-13): P63 on our build failed at once with alarm
  01406 (IGNALG: no TTF/8 root) where stock showed V06N61. Luminary has one TLAND (= W,
  ERASABLE_ASSIGNMENTS); the C kept two: core pad_loads.c g_tland (address map, nouns, P57) and
  agc_lm_pad()->tland (read by P63 IGNALG). A TLAND loaded by address never reached P63, so the
  landing site was rotated to time 0 (LAND 47 deg off) and ROOTPSRS diverged. Found with the
  Linux replay of the PDI LGC image (debug library, gdb at the WHATALM branch: REFSMMAT equal
  to the image, LAND not). Fix: the pad field is gone; src/lm/landing/lunar_landing.c reads
  agc_erase_tland(). tests/lm/landing.c test 3 (fails before). Replay: N61 now TG -09 50,
  TFI -14 33, CR +0003.6 (stock -09 50, -14 18 a little later, +0003.6).
- PDI: the earlier stock "V97, no braking" result was a warp artefact, not the checklist. With
  attitude logging (Orbiter pitch/bank/yaw, thrust in the horizon frame, RCS jets, LGC CDUs) the
  stock LM tumbled under 20x (16 jets, CDUs frozen at gimbal lock, alarm 00206 at PDI-5) and at
  10x a DAP pulse grew into a continuous 4-jet rotation (PDI-9); the DPS then fired upward (LM
  climbing at 60 m/s at full throttle). At 1x the LM holds attitude (jets 0). The LM's
  PDI runs at 1x throughout (pdi.lua ac.WAIT_WARP = 1); the 10x/20x runs are in runs/discarded/.
- CHECKPOINT 2026-10-13 17:22 (host restart; resume from here). Bug 4 committed (HEAD c97631d).
  Uncommitted, complete (no half edits): src/nassp/patches/0002 regenerated (git diff of tree
  nassp-ref) with set_ttca_throttle + set_agcinputbit (LEM.cpp/LEM.h, identical in the
  nassp-ref and nassp-agc trees; reverse-applies cleanly on nassp-ref); docs/overview note;
  this file. No ACA setter (not needed: the landing ran P65 auto). Installed builds:
  C:\NASSP9-ref LEM.dll 16:19 = stock 5470bf1 + new 0002; C:\NASSP9-dev Saturn5+LEM 16:20 =
  HEAD c97631d (sync.sh, no overlays) + 0001 + new 0002 (later hook edits were comments only).
  STOCK PDI LANDED, unattended, 1x (runs/ref_pdi): ignition sim 903.0 (AGC 369646.9), throttle-up
  TIG+26.6 s, P64 at TIG+496.7 s (galt 2205 m, hdot -41.3, vh 152.8), V16N68 at TIG+5m
  R1 +00477 R2 -04 04 R3 +99999 and at TIG+8m +00052 / -01 07 / -00048, script handover at
  TIG+9m02 in P64, P65 (auto, MODREG 65) at TIG+660.8 s (galt 46 m), ground contact TIG+706.9 s
  at -0.85 m/s, touchdown lng 23.7265 lat 0.6695 (Orbiter equ pos), mass 7512 kg, ENG STOP.
  The checklist flies P63/P64 up to "At TIG +10m"; the LGC goes to P65 by itself; the P66/ROD
  fallback (ATT HOLD + set_agcinputbit ROD clicks below 30 m if no P65) was not needed.
  DEV PDI run: killed by the host D3D error at sim 247 (invalid; rerun from scenario load).
  NEXT: bash start.sh 'C:\NASSP9-dev' 'apollo-c\A11 PDI' 4000 (pp.sh polls; poll.sh ... agc_pdi
  collects), then tools/pdisum.py on both logs, compare, report to main, commit 0002 + docs.
  Scratch lives in /tmp (lost on restart): copies of lua/ (pdi.lua 1x + hooks, common.lua with
  ac.run 'to' row and ac.hook), tools/ (pdisum.py, dsky.py, cl2lua.py), start/poll/pp/sync/
  mkscn.sh and runs/ref_pdi* are in C:\Users\cnama\nassp-work\scratch-backup; the deployed
  pdi.lua/common.lua/cl_lm.lua are in C:\NASSP9-{ref,dev}\Script\apollo-c. Linux gates not
  rerun since the last commit (only docs and the patch file changed).
- FLIGHT BUG 5 (PDI, ours only, fixed 2026-10-13): our 1x PDI (runs/discarded/agc_pdi_1x_bug5)
  matched stock to P64 (ignition sim 902.3 vs 903.0, throttle-up TIG+26.6 s both, P64 TIG+496.9 vs
  +496.7 s) but took no LR data: alarm 00522 (LRPOSCHK, P20-P25 p.556) on every LR read from
  TIG-26 s, so at P64 the LGC altitude was 7660 ft against 6027 ft true; R12 reads then collided
  (BAILOUT 01210 in the yaw maneuver) and the LM hit the ground at 35 m/s horizontal in P64/P66.
  Cause: the shim restarts a running LGC on scenario load (GOJAM; yaAGC resumes), and STARTSUB
  (L p.219) sets RADMODES bit 6 (LR position 2 wanted) from channel 33; the scenario's LR breaker
  is open until PDI-5, the LR reports no position, so our LGC wanted position 2 (RADMODES 0142;
  stock 04102, both logged in NASSP, runs/{agc,ref}_lrdbg); once the LR came on in position 1
  LRPOSCHK failed. Fix (shim only, libagc unchanged): src/nassp/nassp_engine.c keeps the image's
  RADMODES bit 6 across the load restart of the LGC (not on a power-on, Z = 04000); the sidecar
  logs channel 33 input changes. Test: make nassp-replay case pdi_lm (sim/nassp/images/pdi_lm.agc,
  RADMODES bit 6 after the load = the image's; FAIL before, PASS after; this harness's yaAGC
  restarts the image on a TC trap, so stock is no reference for that line).
- LM coasting warp (lead request 2026-10-13), stock, PDI scenario load to TIG-75 s, body rates,
  jets, CDUs, middle gimbal, alarms every 2 s (lua/pdiw.lua): 5x PASS (max 0.28 deg/s, commanded
  maneuvers only; FAILREG 0; middle gimbal < 4 deg; runs/ref_pdiw5); 10x no tumble this time but
  a deadband firing overshot to 0.35 deg/s and left 3x the drift (runs/ref_pdiw10), and the
  earlier 10x run tumbled (4-jet rotation, PDI-9). Adopted: 5x for every LM coasting wait, 1x from
  TIG-75 s through the burn/landing, both builds. Both runs logged a warp drop to 1x at sim 195 s
  that NASSP or Orbiter makes (re-asserted by the script).
- DONE milestone 8 (PDI/landing, 2026-10-13, headless fixed step, both builds, the table generated
  by tools/edits_diff.py from the edit CSVs; SCRATCH runs/{ref,agc}_pdi_H): ignition GET 369646.9
  both; throttle-up TIG+25.5 s both; P64 TIG+495.7 s both (galt 2202 vs 2116 m); P65 auto
  TIG+630.4 vs 657.9 s (galt 46 / 45 m); contact TIG+673.2 vs 701.8 s at -0.88 vs -0.92 m/s
  (vh 0.40 both); touchdown lng/lat 23.7410/0.6689 vs 23.7245/0.6696 (windowed 1x stock:
  23.7265/0.6695, P65 +660.8 s); no alarms, no assertion reports, no BAD_ARG on ours. Stock
  windowed vs stock headless differ by 30 s / 0.015 deg, the envelope ours lies in. Open: ours'
  true altitude runs ~30 m above stock from TIG+120 s and 85 m below at P64 (LR updates).
- FLIGHT BUG 7 (PDI, ours only, fixed 2026-10-13): a spurious alarm 00401 once per PDI (TIG+267..
  292 s). FINDCDUW's MGA limit (p. 914) read the two's MGA as ones' and converted back after
  LIMITSUB; for MGA = two's -1 (ones' -0) the C got +0 back and the MSU was non-zero. LIMITSUB
  returns L itself inside the cone. Fix: agc_lm_findcduw_limit_mga (src/lm/landing/findcduw.c)
  keeps the word unless limited; tests/lm/landing.c test 4 (fails before). Found by mapping
  UNFC/2, UNWC/2 (LM 03253, 03261) into the address map and comparing them in the edit CSVs
  (identical within 0.1 deg on both builds, so the alarm came from the limit, not the inputs).
- FLIGHT BUG 8 (after touchdown, ours only, fixed 2026-10-13): R10's SPEEDRN2 task returned
  BAD_ARG every 0.5 s (T3RUPT diagnostic now names the waitlist head task: 137 SPEEDRUN_RESUME;
  the NASSP build with -DAPOLLO_C_ASSERT_REPORTS=ON printed displays.c:329 'ld_altsave >= 0').
  ALTOUT's CCS ALTBITS takes a -0 high word (LGC altitude just below zero) as new data and
  ZERODATA zeroes the meter (pp. 899-900); the C asserted a non-negative ALTSAVE. Fix: assertion
  gone (src/lm/landing/displays.c); tests/lm/landing.c test 5 (fails before).
- Headless harness (lead 2026-10-13): Modules\Server\Orbiter.exe -s SCN -f 0.02 -t T -x with
  D3D9Client and XRSound removed from Orbiter_NG.cfg (SCRATCH hstart.sh / hrestore.sh / hrun.sh),
  about 20x real time, deterministic (two runs identical). NASSP needs focus on a vessel that is not
  a Saturn/LEM (focus MCC; a focused LEM crashes console mode in its cockpit setup) and no Saturn in
  the scenario (Columbia crashes it even unfocused: Orbiter.exe 0xc0000005 at 0x113557 after
  "Finished setting up render state"). An unfocused LEM skips JoystickTimestep (channel 31 and the
  TTCA throttle stay as saved). Scripts write edit CSVs (ac.edits: MODREG, FAILREG, channels
  11/12/14/30/33, TIME, DSPTAB, vessel truth, UNFC/2, UNWC/2) diffed by tools/edits_diff.py with
  per-quantity tolerances.
- LM test hook: src/nassp/patches/0002-lem-lua-test-interface.patch (LEM gets Saturn's
  get/set_panelswitch, get_agcerasable, get_agcchannel; Lua link only; 2026-10-13 also
  set_ttca_throttle and set_agcinputbit, the TTCA throttle and the ROD bits of channel 016 as
  LEM.cpp's keyboard/VESIM handlers set them, for the landing). Reference LEM.dll in
  C:\NASSP9-ref = stock 5470bf1 + 0002 (C:\Users\cnama\nassp-work\build-ref.bat, tree
  nassp-ref; release DLL kept as LEM.dll.release-437); ours = 0001 + 0002 (nassp-agc).
- Closing list from the lead done early: `make style` clean on address_map.c, nassp_engine.c,
  ecadr.c (boxed prologs, no behaviour change; 140 map checks pass); `make lm-sim` fetches the
  LM Simulator itself (sparse virtualagc c239e9c into build/virtualagc).
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
