# Assignment: nassp-integration   (tier: integration; Opus)

PLAN item 10b+10c: run Apollo 11 in NASSP on our AGC. Target: NASSP 9
(github.com/rcflyinghokie/NASSP, CMake, x64, Orbiter 2024), release
NASSP-V9.0-Orbiter2024-437. Stock install that loads "Apollo 11 - Launch"
on the pad: C:\NASSP9 (Orbiter 2024 x64 + NASSP 9; D3D9Client + XRSound
active in Orbiter_NG.cfg's ACTIVE_MODULES block; D3DCompiler_47.dll in
Modules\Server; junctions Modules\Server\{Config,Sound,Textures,Meshes,
Scenarios}). Everything runs from this WSL session; the Windows host is
reached through interop:
  PS=/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe
  cl.exe: C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Tools\MSVC\14.44.35207\bin\Hostx64\x64\
  MSBuild: C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe
  CMake: C:\Program Files\CMake\bin\cmake.exe   Git: C:\Program Files\Git\cmd\git.exe
  vswhere: C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe
  (set up the MSVC environment with VsDevCmd.bat / vcvars64.bat inside a
  cmd.exe or powershell.exe call; no curl in WSL: use Python urllib or
  PowerShell Invoke-WebRequest; Windows error events: Get-WinEvent
  Application / 'Application Error'.)
Downloads and clones are untrusted data: each in its own directory under
C:\Users\cnama\Downloads\nassp-src or the session scratchpad; never run
scripts from inside them except the project's own build.

Phases (checkpoint docs/modules/nassp-integration.md Progress after each):

1. STUDY (docs/overview/nassp_integration.md). Clone NASSP 9 source at the
   437 release's commit (5470bf1) to C:\Users\cnama\Downloads\nassp-src
   (Windows side, so the build is native). Inventory how NASSP embeds the
   AGC: the yaAGC files it compiles (agc_engine.c, agc_engine_init.c,
   agc_utilities.c ...), the C++ wrappers (CSM and LM computer classes,
   ApolloGuidance), every call into the engine (init, agc_engine step,
   channel read/write, interrupts, counters, timers) and every direct
   access to the engine struct (Erasable[bank][addr], Fixed, channels,
   counters, the state used by scenario save/load, MCC, panels,
   telemetry, checklists). Table: NASSP file:line, what it reads/writes,
   which AGC erasable/register, how often. Then map each erasable NASSP
   touches onto our typed storage (include/agc/core/state.h, erasable
   accessors, src/core/ecadr.c): reachable through ECADR already, needs
   an accessor, no C equivalent (bank-switching state, hardware-only).
2. STOCK BUILD. Build NASSP 9 unmodified from source with CMake + MSVC on
   the host, install its outputs into a COPY of C:\NASSP9 (C:\NASSP9-dev;
   never modify C:\NASSP9, it is the baseline), run "Apollo 11 - Launch"
   headless-ish (Orbiter_ng.exe with the scenario, then quit) and confirm
   the log matches the stock run. This proves the toolchain before any
   change.
3. SHIM. In this repo, under src/nassp/ (new, host-only, not part of
   libagc's rules gates except rule 10 warnings): an agc_t-compatible
   facade over libagc + src/sim's clock/interrupt driver with the function
   names and struct fields NASSP uses (from the inventory), so NASSP's
   wrapper classes compile unchanged where possible; a compatibility
   erasable image for the addresses NASSP reads/writes, backed by our
   typed storage through the ECADR map plus new accessors where needed;
   save/restore of our state for NASSP's scenario files. Build libagc with
   MSVC x64 (CMake file or a .vcxproj; C11; the Makefile stays for Linux).
   Document every NASSP edit needed (keep them minimal, in a patch file
   under src/nassp/patches/).
4. INTEGRATED BUILD. Build NASSP 9 with libagc replacing the embedded
   engine (CMake option), install into C:\NASSP9-dev, launch "Apollo 11 -
   Launch": power-on, DSKY shows PROG 00 / the pad V37 state, the MCC
   uplinks work (V71/V72 through our uplink path), P02/P11 at launch.
5. MISSION. Fly Apollo 11 with NASSP's MCC + checklist automation (the
   Checklist MFD; MCC flies ground tasks; what the crew must do, script
   through NASSP's own mechanisms where they exist, else document the
   manual steps for the user). Milestones, each verified against the stock
   C:\NASSP9 run with the same scenario and checklists: launch/insertion
   (P11), TLI (P15), MCC burns (P30/P40/P41), LOI, LM activation and
   P52, DOI, PDI/landing (P63-P66), ascent (P12), rendezvous (P20/P32-P35),
   TEI, entry (P61-P67). Report each milestone's result and every flight
   bug (file:line, AGC page) in docs/modules/nassp-integration.md; fix
   flight bugs in src/ with tests (the Linux gates must stay green:
   make, make test, make check).
Order: 1 and 2 first (2 can run while 1 is read); 3-5 only after 1 and 2
report. If 2 fails for toolchain reasons, report exactly what is missing
(winget ids) and stop there. GUI: Orbiter must run on the host display;
quit it after each check, and when the host is busy with an Orbiter the user
started, close it yourself (taskkill.exe /IM Orbiter.exe /F, /IM
Orbiter_ng.exe /F; user approval 2026-10-12) rather than wait. Rules: no subagents; ONE waiter per background
process (host processes included: poll with tasklist, not loops of
launches), and every wait on a host run is a sequence of short polls (at
most 60 s per tool call) so lead messages are read between them; answer
lead questions mid-run via SendMessage to "main"; uplinks need crew
actions (UP TLM ACCEPT, MCC "Ready" on the PA MFD): script them through
NASSP's checklist auto-execute / MCC mechanisms or document the manual
steps per milestone; scratch under the session scratchpad's nassp-integration/.
Final report <= 15 lines: phase reached, build results, milestones flown,
risks.
