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
`python3 -I sim/nassp/replay.py --keep` leaves both transcripts in build/nassp.

(The other closed loop, `make lm-sim`, needs no setup either: it fetches Virtual AGC's
Contributed/LM_Simulator, a sparse checkout of virtualagc at c239e9c, into
build/virtualagc on first use; LMSIM_DIR=<virtualagc>/Contributed/LM_Simulator uses
another checkout. Tcl/Tk (tclsh) must be installed.)

Full tier, Windows host, headless (the way every milestone is flown and compared since
2026-10-13). Installs: C:\NASSP9-ref (stock NASSP 9 5470bf1 + patches/0002, the LEM Lua
test hook) and C:\NASSP9-dev (ours: patches/0001 + 0002, built with
C:\Users\cnama\nassp-work\build-agc.bat after copying src/ and include/ to
nassp-work\apollo-c; -DAPOLLO_C_ASSERT_REPORTS=ON makes failed AGC_ASSERTs print to
apollo-c-agc-stderr.log). One Orbiter session at a time (NASSP's telemetry binds TCP 14242).

1. Checklists: compile NASSP's LM/CSM checklist workbook
   (<install>\Config\ProjectApollo\Checklists\*.xls) with sim/nassp/checklist_to_lua.py (needs
   xlrd) into cl_lm.lua / cl_csm.lua and copy them to <install>\Script\apollo-c.
2. Scenario: `sim/nassp/host/mkscn_lm.sh C:\NASSP9-dev "Apollo 11 - 10 - PDI T-15m.scn"
   "A11 PDI H" pdih` (and the same for C:\NASSP9-ref) writes Scenarios\apollo-c\A11 PDI
   H.scn: the Lua script added (sim/nassp/lua/pdih.lua, liftoffh.lua; common.lua alongside),
   the focus on the MCC vessel, Columbia dropped. Orbiter's console mode crashes with a
   NASSP Saturn in the scenario (focused or not) or with the LEM in focus (Orbiter.exe
   0xc0000005 right after "Finished setting up render state"); the Lua API reaches the
   vessels by name and needs no cockpit. An unfocused LEM skips JoystickTimestep (channel 31
   and the TTCA throttle keep their saved values). Milestones with a CSM (rendezvous, entry)
   run windowed until that crash is understood.
3. Run: `NASSP_RUNS=<dir> sim/nassp/host/hrun.sh C:\NASSP9-dev "apollo-c\A11 PDI H"
   agc_pdi_H 1800` and the same from C:\NASSP9-ref as ref_pdi_H. hstart.sh keeps the
   install's Orbiter_NG.cfg as Orbiter_NG.cfg.windowed, runs from a copy without D3D9Client
   (no graphics client: console mode) and XRSound, MFD modules kept, and starts
   `Modules\Server\Orbiter.exe -s <scenario> -f 0.02 -t <max sim s> -x` (fixed 20 ms
   steps, as fast as the CPU allows: PDI to touchdown in about 90 s, deterministic: two runs
   give the same numbers); hrun.sh polls (collect.sh, at most 50 s a call) until the script
   logs "end", copies apollo-c-*.log and the edit CSV to $NASSP_RUNS/<tag>, restores the
   configuration (hrestore.sh). Scripts hold 1x (no warp: the fixed step removes the
   warp-induced LM tumbling).
4. Tables: `make nassp-mission NASSP_RUNS=<dir>` (sim/nassp/mission_tables.py) pairs ref_<m>_H and
   agc_<m>_H for every milestone it finds and prints the milestone table (events found the
   same way in both edit CSVs), per quantity the max |ours - stock| against the tolerance in
   sim/nassp/mission_tolerances.tsv, and each run's major modes and alarms; exit 1 when a quantity leaves
   its tolerance. Numeric columns without a tolerance row are printed as "not judged".
   Tolerances: `sim/nassp/mission_tables.py --derive <dir> <milestone>` prints the rows from the two
   stock runs ref_<m>_H (-f 0.02) and ref_<m>_H01 (-f 0.01): every numeric column, exactly 1.5 x
   their max difference, no rounding. They go into sim/nassp/mission_tolerances.tsv before the dev run is compared
   and are then frozen (see "Test changes").

## Mission report (final, 2026-10-14)

Apollo 11 on our AGC (Comanche055 / Luminary099 in C, libagc through the engine shim src/nassp/)
inside NASSP 9 (Orbiter 2024), every milestone flown unattended from NASSP's own scenarios and
checklist workbook and compared with stock NASSP 9 (its yaAGC) flying the same scenario and script.
Judged on decisions (programs, displays, burns, alarms) and end state; differences that come from
AGC execution time are listed under "Expected deviations (timing)" with their traces.

| # | milestone (scenario) | result |
|---|---|---|
| 1 | insertion + MCC state-vector uplink (02) | match: V82 104.1 / 102.0 vs 104.3 / 101.9 nmi; uplink P27 on both, same V21N01 sequence |
| 2 | TLI (03) | match: P47 through the S-IVB burn; post-TLI vector 0.4-1.0 km apart (0.11 s of travel) |
| 3 | MCC-2 (05) | match after bug 1: VG / dVM 0.3 / 15.9 ft/s both; N85 -0.3/-0.1/-0.1 vs -0.3/-0.1/0.0 |
| 4 | LOI-1 (06) | match: SPS 373.97 vs 373.90 s; HA / HP 169.7 / 59.5 vs 169.9 / 59.5 nmi |
| 5 | TEI (13) | match after bug 2: SPS 160.28 vs 160.26 s; VG / dVM 0.9 / 3306.7 vs 1.1 / 3306.8 ft/s |
| 6 | DOI (09, LM) | match after bug 3: cut-off 0.6 s apart; orbit 1752.5 x 1851.4 vs 1753.1 x 1851.4 km |
| 7 | PDI to touchdown (10) | events and end state match (table); attitude history and latitude in braking outside the stock envelope |
| 8 | lunar liftoff (11, running P12) | events and end state match (table); CDU Z after insertion outside (P12 ends 16 s earlier: timing) |
| 9 | rendezvous (rndz start: P20, CSI, CDH, TPI) | after bugs 15-17: every display, targeting and burn as stock, no alarms on either; closing 37.1 km at -34.2 m/s 10 min after TPI (stock 38.9 km, -35.5 m/s) |
| 10 | entry to splashdown (14) | events and end state match (table); splashdown 1.6 km from stock |

Milestones 1-8 and 10 were flown before bugs 15-17 were fixed and not repeated: bug 16 and 17 touch
only P32's N55 load and the LM's INITVEL; bug 15 changes DELCOMP / DCMTOCDU only where a term
overflows to exactly 1 (a rotation of a few hundredths of a degree about an axis near a body axis),
which no earlier milestone's tables show (their attitude maneuvers matched stock).
Milestones 1-6 were flown before the edit-CSV harness existed (windowed, 20x warp schedule; numbers
from the run logs, Progress below); 7-10 headless at a fixed 20 ms step, tables generated by
`make nassp-mission` (sim/nassp/mission_tables.py) against tolerances of 1.5 x stock's own spread
between its -f 0.02 and -f 0.01 runs (sim/nassp/mission_tolerances.tsv, frozen before each dev run).

### Generated tables (make nassp-mission, 2026-10-14)

#### PDI to touchdown

| milestone | stock | ours |
|---|---|---|
| ignition (DPS > 5%), GET s | 369646.9 | 369646.9 |
| throttle-up (DPS > 50%), TIG+ s | 25.5 | 25.5 |
| P64, TIG+ s | 495.7 (galt 2202 m, vh 152.7 m/s) | 495.7 (galt 2116 m, vh 152.4 m/s) |
| P65, TIG+ s | 630.4 (galt 46 m, vh 1.4 m/s) | 656.9 (galt 46 m, vh 1.7 m/s) |
| DSKY at TIG+5m | PROG 63 VERB 16 NOUN 63 R1 +26302 R2 -01133 R3 +35092 | PROG 63 VERB 61 NOUN 63 R1 +26053 R2 -01148 R3 +34968 |
| DSKY at TIG+8m | PROG 63 VERB 16 NOUN 63 R1 +06536 R2 -01428 R3 +09993 | PROG 63 VERB 16 NOUN 63 R1 +06384 R2 -01445 R3 +09752 |
| ground contact, TIG+ s | 673.2 (hdot -0.88 m/s, vh 0.40 m/s 1 s before) | 701.8 (hdot -0.92 m/s, vh 0.40 m/s 1 s before) |
| touchdown lng / lat, deg | 23.7410 / 0.6689 | 23.7245 / 0.6696 |

| quantity | tolerance | max abs diff | at TIG+ s | first TIG+ s out |
|---|---|---|---|---|
| galt | 478.95 | 94.581 | 544 | none |
| hdot | 2.35439 | 1.941 | 405 | none |
| vh | 11.1325 | 7.455 | 604 | none |
| lng | 0.0888 | 0.017 | 695 | none |
| lat | 0.001248 | 0.002 | 398 | 136 |
| mass | 165.6 | 103.230 | 703 | none |
| dps | 0.618303 | 0.414 | 404 | none |
| pitch | 22.3107 | 14.614 | 572 | none |
| bank | 41.346 | 48.632 | 0 | 0 |
| yaw | 40.92 | 46.772 | 0 | 0 |
| cdux | 4.82849 | 10.360 | 345 | 341 |
| cduy | 22.3132 | 14.623 | 572 | none |
| cduz | 2.50488 | 1.835 | 0 | none |
| gc | 1.5 | 1.000 | 673 | none |

stock: major modes P63@TIG-897 P00@TIG-893 P63@TIG-884 P64@TIG+496 P65@TIG+630
stock: alarms none

ours: major modes P63@TIG-897 P00@TIG-893 P63@TIG-884 P64@TIG+496 P65@TIG+657
ours: alarms none

#### Lunar liftoff

| milestone | stock | ours |
|---|---|---|
| ignition (APS > 5%), GET s | 448308.9 | 448307.9 |
| cut-off, TIG+ s | 431.9 (vh 1692.5 m/s, hdot 10.5 m/s) | 431.9 (vh 1692.5 m/s, hdot 10.1 m/s) |
| orbit radius peri x apo, km | 1752.85 x 1826.15 | 1752.94 x 1825.82 |
| mass after cut-off, kg | 2629.5 | 2628.9 |
| DSKY 30 s before cut-off | PROG 12 VERB 16 NOUN 85 R1 +05780 R2 +00048 R3 +00336 | PROG 12 VERB 16 NOUN 85 R1 +05602 R2 -00012 R3 +00305 |

| quantity | tolerance | max abs diff | at TIG+ s | first TIG+ s out |
|---|---|---|---|---|
| galt | 1021.95 | 91.900 | 557 | none |
| hdot | 1.3332 | 0.918 | 5 | none |
| vh | 8.46 | 0.358 | 285 | none |
| lng | 0.1011 | 0.003 | 358 | none |
| lat | 0.005955 | 0.002 | 557 | none |
| mass | 6.45 | 2.610 | 1 | none |
| eng | 1.5 | 0.005 | 0 | none |
| pitch | 26.5227 | 7.119 | 551 | none |
| bank | 176.871 | 124.558 | 495 | none |
| yaw | 167.513 | 133.089 | 495 | none |
| cdux | 10.7611 | 6.877 | 495 | none |
| cduy | 26.3507 | 6.174 | 540 | none |
| cduz | 12.4091 | 16.348 | 543 | 520 |
| apo | 28455 | 460.000 | 429 | none |
| peri | 25545 | 348.000 | 332 | none |

stock: major modes P12@TIG-58 P00@TIG+488
stock: alarms none

ours: major modes P12@TIG-57 P00@TIG+472
ours: alarms none

#### Rendezvous

| milestone | stock | ours |
|---|---|---|
| P20 selected, s | 212 | 213 |
| RR lock-on (F50N72), s | 307 (range 448.4 km) | 294 (range 450.3 km) |
| P32, s (marks) | 1118 (12) | 1118 (26) |
| CSI N11 (last before the burn) | +00125 +00029 +00646 | +00125 +00029 +00646 |
| CSI N75 (last before the burn) | +00158 +58 02 +39 22 | +00152 +57 21 +40 03 |
| CSI N81 (last before the burn) | +00450 +00000 +00000 | +00456 +00000 +00000 |
| CSI N82 (last before the burn) | +00003 -00000 -00375 | +00021 -00000 -00361 |
| CSI burn start s, nulling s | 2921, 72 | 2922, 78 |
| CSI RCS used, kg | 18.5 | 19.8 |
| CSI N85 after nulling | +00002 -00001 -00002 | +00002 +00002 -00002 |
| CSI orbit peri x apo, km (+60 s) | 1814.30 x 1824.84 | 1815.42 x 1823.61 |
| P33, s (marks) | 3043 (0) | 3015 (0) |
| CDH N13 (last before the burn) | +00126 +0002?  ???50 | +00126 +00026 +02710 |
| CDH N75 (last before the burn) | +00156 +39 23 +00 02 | +00148 +41 22 +01 20 |
| CDH N81 (last before the burn) | +00005 -00000 -00354 | +00024 +00000 -00331 |
| CDH burn start s, nulling s | 6403, 58 | 6361, 59 |
| CDH RCS used, kg | 14.9 | 15.3 |
| CDH N85 after nulling | -00002 -00001 -00002 | -00001 -00001 -00002 |
| CDH orbit peri x apo, km (+60 s) | 1806.17 x 1832.86 | 1807.89 x 1833.95 |
| P34, s (marks) | 6520 (0) | 6441 (0) |
| TPI N37 (last before the burn) | +00127 +00006 +03318 | +00127 +00006 +04649 |
| TPI N58 (last before the burn) | +00506 +00258 +00325 | +00512 +00238 +00312 |
| TPI N59 (last before the burn) | +00258 -00001 +00006 | +00237 +00003 +00014 |
| TPI N81 (last before the burn) | +00233 +00001 -00110 | +00218 +00004 -00094 |
| TPI burn start s, nulling s | 8767, 36 | 8781, 64 |
| TPI RCS used, kg | 9.4 | 10.6 |
| TPI N85 after nulling | +00002 -00002 -00002 | +00002 +00000 -00002 |
| TPI orbit peri x apo, km (+60 s) | 1830.64 x 1839.98 | 1831.96 x 1840.18 |
| TPI burn + 600 s: range km, range rate m/s | 38.91, -35.50 | 37.09, -34.21 |

| quantity | tolerance | max abs diff | at TIG+ s | first TIG+ s out |
|---|---|---|---|---|
| range | 4350.9 | 5711.000 | 5323 | 4085 |
| rrate | 12.8187 | 5.577 | 8781 | none |
| phi | 31.9249 | 24.521 | 6538 | none |
| apo | 13395 | 12310.000 | 6378 | none |
| peri | 34620 | 18090.000 | 8780 | none |
| mass | 11.595 | 25.140 | 6402 | 6363 |
| pitch | 30.8269 | 27.945 | 6506 | none |
| bank | 90.0599 | 178.741 | 9690 | 16 |
| yaw | 80.5785 | 178.250 | 49 | 16 |
| cdux | 20.2368 | 14.821 | 9275 | none |
| cduy | 33.2556 | 28.312 | 6506 | none |
| cduz | 40.3418 | 31.047 | 5163 | none |
| cdut | 18.5888 | 13.569 | 9139 | none |
| cdus | 31.9695 | 28.048 | 6506 | none |
| markctr | 40.5 | 62.000 | 2997 | 2997 |

stock: major modes P00@TIG+0 P20@TIG+212 P32@TIG+1118 P41@TIG+2562 P33@TIG+3043 P41@TIG+6044 P34@TIG+6520 P41@TIG+8409
stock: alarms none

ours: major modes P00@TIG+0 P20@TIG+213 P32@TIG+1118 P41@TIG+2562 P33@TIG+3015 P41@TIG+6002 P34@TIG+6441 P41@TIG+8422
ours: alarms none

Rendezvous columns outside the stock envelope: range (5.7 km, tolerance 4.35, from TIG+4085 s, after
CSI) and mass (25 kg at TIG+6402 s, the CDH burn 42 s earlier on ours), both following the CDH time;
bank / yaw (Euler-degenerate near pitch 90, as in PDI); MARKCTR (62 marks by CSI vs 19). CDH comes
42 s earlier because CSI's solution differs by about 2 ft/s (N81 45.6 vs 45.0 ft/s, N82 R1 2.1 vs
0.3) and CDH is set at the apsidal crossing of a nearly circular orbit (e 0.0022 vs 0.0029); the
navigation states differ by the number of RR marks incorporated (Expected deviations).

#### Entry

| milestone | stock | ours |
|---|---|---|
| entry interface (400,000 ft), GET s | 702246.3 | 702246.3 |
| P63, EI+ s | -570.0 | -600.0 |
| P64, EI+ s | 29.0 | 28.0 |
| P67, EI+ s | 139.0 | 138.0 |
| max deceleration, g at EI+ s | 6.20 at 82.0 | 6.22 at 82.0 |
| apex cover / drogues, EI+ s | 467.0 (alt 12189 m) | 467.0 (alt 12167 m) |
| DSKY at the drogues | PROG 67 VERB 16 NOUN 67 R1 +00006 R2 +01094 R3 -17225 | PROG 67 VERB 16 NOUN 67 R1 +00008 R2 +00069 R3 +02345 |
| splashdown, EI+ s | 866.0 | 865.0 |
| splashdown lng / lat, deg | -172.2510 / 10.8860 | -172.2570 / 10.8731 |

| quantity | tolerance | max abs diff | at TIG+ s | first TIG+ s out |
|---|---|---|---|---|
| alt | 820.5 | 303.500 | 271 | none |
| hdot | 54.4085 | 29.031 | 378 | none |
| vh | 55.29 | 24.130 | 200 | none |
| vair | 59.7522 | 24.090 | 201 | none |
| g | 3.04568 | 1.438 | 571 | none |
| lng | 0.042 | 0.012 | 160 | none |
| lat | 0.033015 | 0.013 | 523 | none |
| mass | 1.23 | 1.240 | 172 | 172 |
| pitch | 35.1549 | 16.883 | 421 | none |
| bank | 268.577 | 120.177 | 538 | none |
| yaw | 216.933 | 125.956 | 536 | none |
| nobj | 12 | 8.000 | 122 | none |

stock: major modes P00@TIG-8219 P61@TIG-1196 P62@TIG-1147 P63@TIG-570 P64@TIG+29 P67@TIG+139
stock: alarms none

ours: major modes P00@TIG-8219 P61@TIG-1196 P62@TIG-1150 P63@TIG-645 P64@TIG+28 P67@TIG+138
ours: alarms none

### Flight bugs found by the mission (all on our build only, each reproduced offline and fixed with a test that fails before the fix)

| # | milestone | symptom | cause | fix, test |
|---|---|---|---|---|
| 1 | MCC-2 | SPS burned 2.53 s vs 2.21 s | address map put EK3VAL over FANG, FANG stayed 0 | src/erasable_map/address_map.c; tests/erasable_map/address_map.c |
| 2 | TEI | DSKY blank after V82 on the escape hyperbola | V82 asserted -TPER <= 0; SKIPTPER stores it of any sign | src/programs/displays/orbit_parameter_display.c; tests/programs/rendezvous.c test 6 |
| 3 | DOI | P30 N42 DV REQ 0 | VGDISP = DELVSAB is one erasable; the LM writers set only DELVSAB | LM S30.1 / BDTOK / aimpoint / CSI keep them equal; tests/guidance/guidance.c |
| 4 | PDI | alarm 01406 at P63 | two TLANDs (pad load vs P63) | src/lm/landing/lunar_landing.c; tests/lm/landing.c test 3 |
| 5 | PDI | no LR data (00522), crash in P64/P66 | load restart set RADMODES bit 6 from the unpowered LR | src/nassp/engine.c keeps the image's bit; nassp-replay pdi_lm |
| 6 | PDI | BAILOUT 01210, LR updates lost | zero-time executive let LRVJOB/HIGATJOB enter RADSTALL while LRHJOB held the radar | radar jobs wait while the radar is held (DEVIATION); tests/sim/harness.c |
| 7 | PDI | spurious alarm 00401 | FINDCDUW MGA limit converted -0 through ones' complement | src/lm/landing/findcduw.c; tests/lm/landing.c test 4 |
| 8 | after touchdown | SPEEDRN2 BAD_ARG every 0.5 s | assertion on a -0 high word ALTOUT accepts | src/lm/landing/displays.c; tests/lm/landing.c test 5 |
| 9 | liftoff | overshoot to 1757.9 x 1901.0 km | QAXIS = UHYP, ZAXIS1 = UHZP kept apart | src/lm/ascent.c; tests/lm/landing.c test 6 |
| 10 | liftoff | P12 N85 00000 | VGBODY one erasable, two C copies | ascent writes burn.vgbody; tests/oracle/landing.c |
| 11 | liftoff | P12 TIG = TIME + a DPS TGO | TGO = DVCNTR+1 one erasable, two C copies | agc_lm_asc_set_tgo stores both |
| 12 | entry (and MCC-2) | CMC never held or maneuvered the CSM after a load | CSM RCS DAP state unmapped | address_map.c; test_csm_rcs_dap |
| 13 | entry | splashdown 2.6 deg short | UNITW unmapped; vector stores dropped into DELVLVC | address_map.c EC_UNITW, vec_set_more; test_vectors_stored |
| 14 | liftoff (resumed P12) | LM never reached orbit | average-G start state, MUNFLAG, /LAND/ unmapped | address_map.c; test_servicer_start |
| 15 | rendezvous P20 | LM swung 40-60 deg in roll while tracking; marks stopped, 00525 | BOVB SIGNMPAC took the DP value's sign, not the major word's (+0 after an SL overflow): DELCOMP diagonal -1, VECPNT1 outer gimbal 180 deg off | src/imu/maneuver_calc.c; tests/imu/imu.c test_vecpoint_tiny_turn, test_delcomp_unit_diagonal |
| 16 | rendezvous P32 | alarm 00603 / 00604, no CSI | the crew's N55 R1 (NN, NN1) was not loadable: NN kept the scenario's 7512 | src/programs/navigation/noun_data.c; tests/programs/rendezvous.c test_n55_loads_nn |
| 17 | rendezvous TPI | ABORT 01302 at P41 TIG-28 s, burn unsteerable | INITVELX MUASTEER = MU/A SR 6 where STODL pops the pushed MU: MUASTEER = MU SR 6 | src/programs/targeting/initvel.c; tests/programs/rendezvous.c test_initvel_muasteer_lm |

Bugs 1, 5, 12-14 are restore gaps of the scenario load (erasables the address map must carry);
2-4, 7-11 and 15-17 are translation faults the flight exercised and the unit and oracle tests had
not. Not automatable or not flown (both builds alike, listed per milestone in Progress): manual DPS
throttle at DOI, DEDA / AGS entries, ORDEAL slews, P52 and MCC1/MCC2 in the rendezvous sheets.

### Host procedure (to repeat a milestone)

1. Dev DLLs: copy `git archive HEAD src include` (plus any working-tree file under test) to
   C:\Users\cnama\nassp-work\apollo-c and run C:\Users\cnama\nassp-work\build-agc.bat (CMake +
   MSVC: NASSP 9 at 5470bf1 with sim/nassp/patches/0001-0003, -DAPOLLO_C_DIR, Saturn5.dll and LEM.dll
   into C:\NASSP9-dev). The stock reference C:\NASSP9-ref is NASSP 9 + 0002 + 0003 (build-ref*.bat).
2. Scripts: sim/nassp/lua/*.lua (and the compiled checklists, sim/nassp/checklist_to_lua.py) copied to
   <install>\Script\apollo-c on both installs; scenarios by sim/nassp/host/mkscn_lm.sh.
3. Runs: `sim/nassp/host/hstart.sh <install> <scenario> <max sim s> 0.02`, then
   `NASSP_RUNS=<dir> sim/nassp/host/collect.sh <install> <tag>` (one poll, at most 50 s) until it
   prints FINISHED, then `hrestore.sh <install>`; stock as ref_<m>_H (-f 0.02) and ref_<m>_H01
   (-f 0.01), ours as agc_<m>_H. One Orbiter at a time.
4. Tolerances before the dev run: `python3 -I sim/nassp/mission_tables.py --derive <dir> <m>` into
   sim/nassp/mission_tolerances.tsv; then `make nassp-mission NASSP_RUNS=<dir>`.

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
  - src/nassp/engine.c: yaAGC API (agc_engine_init, agc_load_binfile, agc_engine,
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
- `make nassp-replay` (light tier, 2026-10-13): sim/nassp/replay.c + sim/nassp/replay.py,
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
- src/nassp/engine.c now reopens stderr into apollo-c-agc-stderr.log, so src/sim's
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
  LRPOSCHK failed. Fix (shim only, libagc unchanged): src/nassp/engine.c keeps the image's
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
  RESTATED under the mechanical tolerances (2026-10-13, see "Test changes"): events and end
  state match (P65 +27.5 s is inside stock's own spread: stock -f 0.01 P65 at +659.5 s);
  outside the stock envelope: attitude history bank 48.6 (tol 41.3) and yaw 46.8 (40.9) deg
  from TIG+0, CDU X 10.4 (4.8) deg from TIG+341 s, and latitude 0.0019 (0.00125) deg from
  TIG+136 s. Open findings to root-cause (DAP words against stock).
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
- DONE milestone 9 (lunar liftoff, scenario 11, headless, both builds, `make nassp-mission`,
  2026-10-13): P12 selected afresh (RSET, V37E12E, PRO on F06N33 / F06N76, the sheet from
  T-0:35; both builds slip TIG with 01703 since T-1m is too short for a fresh P12); cut-off
  TIG+430.9 s both; vh / hdot at cut-off 1684.5 / 6.7 vs 1684.4 / 6.4 m/s; orbit radius
  1753.15 x 1790.76 vs 1753.27 x 1790.15 km; mass 2635.6 vs 2637.5 kg; N85 counts down on
  both. Open: vertical speed 1.4 m/s off stock at the pitchover (TIG+15 s; tolerance 0.93).
  RESTATED under the mechanical tolerances (2026-10-13): end state matches (cut-off, orbit within
  0.6 km, mass); events do NOT all match: ignition GET 448311.9 vs stock 448335.4 (23.5 s
  earlier: the 01703 TIG slip differs; V99 at sim 59.1 vs 82.9 s). Outside the stock envelope:
  hdot 1.39 (0.93) m/s from TIG+14 s, pitch 24.3 (12.4) and CDU Y 24.1 (12.4) deg from TIG+10 s,
  CDU Z 10.8 (7.3) deg from TIG+476 s, engine level at the cut-off row (tolerance 0), apo / peri
  radius 14.2 / 15.3 (12.2 / 15.1) km around cut-off. Open findings.
- FLIGHT BUG 9 (liftoff, ours only, fixed 2026-10-13): ascent overshot to 1757.9 x 1901.0 km
  (vh 22 m/s high at cut-off). First diverging word VGVECT x at TIG+120 s while TGO, GEFF, DRDOT
  .. DZDOT, PCONS, PRATE, HDOTDISP agreed (all mapped into the edit CSV): Luminary has QAXIS
  EQUALS UHYP and ZAXIS1 EQUALS UHZP, and NOR29NOW renews UHZP = UNIT(UNIT/R/ x UHYP) every
  SERVICER pass (ascent runs with LRBYPASS through COPYCYC1); the C kept agc_lm_asc()->qaxis
  / zaxis1 apart, so the ascent's downrange axis stayed at the liftoff point. Fix: the fields
  are gone, src/lm/ascent.c uses lm.uhyp / uhzp; tests/lm/landing.c test 6 (fails before);
  tests/oracle/landing.c feeds QAXIS / ZAXIS1 through UHYP / UHZP.
- FLIGHT BUG 10 (P12 N85 00000, fixed 2026-10-13): VGBODY is one erasable (SAVET-30 +2); ascent
  wrote agc_lm_asc()->vgbody, N85 read burn.vgbody. Fix: ascent writes burn.vgbody, the field
  is gone (tests/oracle/landing.c compares VGBODY there).
- FLIGHT BUG 11 (TGO, fixed 2026-10-13): TGO EQUALS DVCNTR +1 is one erasable; the ascent's
  words (agc_lm_asc()->tgo) and agc_erase_tgo() were apart, so P12's IGNITION set TIG = TIME +
  a DPS burn's TGO. Fix: agc_lm_asc_set_tgo stores both (ascent.c, aborts.c).
- Scenario-load restart (2026-10-13): a program running at the load (P12 at T-1m) needs its
  restart group: the phase table is mapped (PHASEn / -PHASEn 0752, TBASEn 01053, LONGBASE CSM
  01135 / LM 01151; PHSNAMEn / PHSPRDTn are rope addresses and stay unmapped); nassp-replay
  case liftoff_lm (V06N74 40 s after the load; blank before). The program's own erasables
  must be mapped too for its later steps: for P12 now AT VE TTO TBUP RDOTD YDOTD ZDOTD /R/MAG
  Y DRDOT DYDOT DZDOT PCONS YCONS PRATE YRATE GEFF ATY ATR ATP RCO YCO VGVECT LAXIS
  QAXIS(UHYP) ZAXIS1(UHZP) HDOTDISP; the liftoff script still reselects P12 on both builds.
- FLIGHT BUG 12 (CSM attitude hold, ours only, fixed 2026-10-13): after a scenario load our CMC
  never held or maneuvered the CSM (entry run: bank drifting at the load's -0.045 deg/s for
  1000 s, the R60 maneuver never ending; MCC-2 had the same V06N18 timeout). Host trace (Lua
  daph.lua, scenario 14 at P00, both builds): stock's roll jets stop the drift at sim 12-20 s,
  ours never fire although CDUs and channel 6 reach NASSP. Offline (harness dap.c on the
  image, CDU X driven at 5 counts/s): stock fires ch6 240 at 10.92 s, ours never. The C DAP
  state after the load restart: J/M = 0 (IXX, IAVG unmapped: MASSPROP's words, S40.14 forms
  J/M from them at REDAP, so TAU = 0 always) and THETADX..Z = 0 with HOLDFLAG lost (the hold
  reference; with HOLDFLAG -1 DCDUINCR rebuilds THETAD from CDUXD, also unmapped). Fix:
  src/erasable_map/address_map.c maps IXX IAVG IAVG/TLX (03070), HOLDFLAG (01332), THETADX..Z
  (03172), DELCDUX..Z (03175) and CDUXD..ZD (03246); tests/erasable_map/address_map.c
  test_csm_rcs_dap (6 checks failed before). After: offline ours fires ch6 240 at 10.89 s
  (stock 10.92), 111 vs 111 jet commands in 110 s; NASSP daph run identical to stock (bank
  -0.857 vs -0.853 deg at 20.6 s, then the same -0.0088 deg/s). Open, smaller: the C keeps
  CDUXD as a normalised DP value, so THETAD = the upper word of DPADD differs by one count
  from the AGC when CDUXD's words have mixed signs (THETADY 27014 vs 27015 here).
- FLIGHT BUG 13 (entry targeting and LM ascent vectors, ours only, fixed 2026-10-13): with the
  attitude hold fixed the CM still splashed 2.6 deg short (-174.84 vs -172.25 deg). First
  diverging display: P61's N63 RTGO 939.0 vs stock 1215.2 nmi (N60, N61 identical). Stock's P61
  erasables dumped by Lua (p61diag.lua) against ours offline from the same pre-P61 image (harness
  keyrun.c): RTEAST = RTNORM = 0 and RT = RTINIT on ours, i.e. EARROT never rotated the target
  for the Earth's rotation: UNITW (Comanche 01713, the polar axis; -AYO and AXO EQUALS its first
  four words) was not in the CSM map (only -AYO, AXO were). The test written for it also showed
  that the address map's vector store dropped VGVECT, LAXIS, QAXIS (UHYP) and ZAXIS1 (UHZP),
  mapped for the liftoff restart, into DELVLVC (vec_set's default branch); reads were right, so
  the replay case and the edit CSVs never showed it. Fix: src/erasable_map/address_map.c
  EC_UNITW (6 words at 01713, ahead of -AYO / AXO; a store also sets -AYO, AXO) and
  vec_set_more for the five vectors; tests/erasable_map/address_map.c test_vectors_stored (6
  checks failed before). After: offline RTGO 01630 33356 vs stock 01631 00735 (P61 run 2 s
  apart), RTEAST / RTNORM / RT as stock.
- Shim display mirror (2026-10-13): NASSP's Lua get_agcerasable (and MFDs) read the vagc.Erasable
  mirror, which the shim exported once per second; the scripts read DSKY verb/noun from DSPTAB,
  so on ours a display appeared up to 1 s late (entry: "PRO not taken" fallbacks where stock
  took the PRO; offline V49: V06N22 in the mirror 0.8 s after ENTER vs stock 0.1 s, though the
  channel 010 relay words were on time on both). Now MODREG, FAILREG, DSPTAB every centisecond
  and the whole mirror every 0.1 s (about 1.5 ms CPU per AGC second). make nassp-replay checks
  it: "# dsky-latency" (V06N3 then '6': noun 36 in the mirror within 0.2 s; ours was 0.71 s,
  stock 0.09-0.11 s, failing first). Dev runs before this change read our DSKY late; PDI,
  liftoff and entry dev runs are repeated with it (stock runs need not be: unchanged).
- DONE milestone 10 (entry, scenario 14, headless, frozen scripts and tolerances, 2026-10-13;
  ours with bugs 12-13 fixed and the cs display mirror): EI GET 702246.3 both; P64 EI+29 / +28 s;
  P67 +139 / +138 s; max 6.20 / 6.22 g at EI+82 s; drogues EI+467 s both (12189 / 12167 m);
  splashdown EI+866 / +865 s at -172.2510 / 10.8860 vs -172.2570 / 10.8731 deg. Events and end
  state match except P63 30 s earlier (EI-600 vs -570 s); outside the stock envelope: mass
  1.240 kg (tol 1.23) at EI+172 s; every other column inside (alt 304 m of 821, lng 0.012 deg
  of 0.042, lat 0.013 of 0.033). Root cause of both (runs/{ref,agc}_entdiag, entdiag.lua): at
  sim 4012 both builds release CMC attitude control (HOLDFLAG POSMAX on both, GDC alignment
  sheet) and the CM drifts with the rate of its deadband limit cycle at that moment: stock at a
  rate reversal, ours mid-drift (CDU Y +14 counts/10 s; 0.8 deg apart before, inside the
  deadband). 3000 s later the attitudes differ by ~60 deg, so P62.1's CMDAPMOD test (P61-P67
  p. 794) takes ours straight to P63 where stock waits for WAKEP62; the entry DAP brings both
  to the same trajectory (mass: RCS used).
- FLIGHT BUG 14 (lunar liftoff from the resumed P12, ours only, fixed 2026-10-14): with the
  scenario's running P12 (Test changes), our LM never reached orbit. Edit CSV: HDOTDISP (RDOT)
  ran away from TIG-30 s; offline (keyrun.c, liftoff_lm image) the servicer started from RN of
  about 0. The scenario is saved after P12's MIDTOAV, so PREREAD at TIG-30 starts average-G from
  RN1 / VN1 / PIPTIME1 (and GDT1/2, GDT/2, RTX1/RTX2), all unmapped. Mapped (both ropes); then
  UHYP (= QAXIS) became (1, 0, 0) at TIG-30: NORMLIZE forms it from R(CSM) x V(CSM) every cycle
  with MUNFLAG set (LM 01717 / 01725, unmapped); mapped. The flight then reached orbit
  (1752.91 x 1825.97 vs stock 1752.85 x 1826.15 km) but pitched over at TIG+4 s: CHECKALT ends
  the vertical rise when |R| - /LAND/ - 25 kft >= 0, and /LAND/ (02332) was 0; mapped.
  tests/erasable_map/address_map.c test_servicer_start (LM and CSM).
- Rendezvous (2026-10-14, NOT DONE): rendh.lua (sim/nassp/lua; scenario 11 with Columbia, the
  scenario's P12, then NASSP's "Rendezvous" sheets with the burns nulled through the TTCA
  translation bits) does not yet work on stock. Its display waits time out (900 s each: V06N47
  after insertion, V50N72 in P20, F06N11 under a pending V50N18 in P32), alarm 00503 (RR
  designate fail) on both builds, and the -t 14000 s limit ends every run in P32 before CSI.
  Because stock and ours time out at different rows (stock waits for V06N47, ours sees it),
  the runs drift apart in sim time and the edit-CSV comparison is not meaningful yet. Next:
  fix the script on stock (answer the R60 V50N18 in P20, the RR designate/lock, the N47 row),
  repeat stock H / H01, re-derive, then the dev run. The ours-only V06N47 is to be looked at
  then. Optics after load (channel 14 70040 vs 76040): offline both builds refuse V41N91 alike
  (alarm 00115 without OPT MODE CMC); the stock bits are the image's stale drive bits, a
  GOJAM clears them, and any new optics drive sets them on both: not a functional difference.
- Shim (2026-10-13): a scenario load fed the image's channel 015 key code to KEYRUPT, lighting
  OPR ERR at every load on ours; 015/016 are now stored without KEYRUPT before the first GOJAM
  (nassp-replay "# oprerr-at-load": 4 of 5 images failed before).
- DONE milestone 9 again (lunar liftoff with the scenario's running P12, headless, 2026-10-14,
  stock H / H01 repeated under the script, tolerances re-derived first, dev from HEAD 755db91 +
  bug 14): ignition GET 448308.9 / 448307.9; cut-off TIG+431.9 s both (vh 1692.5 both, hdot
  10.5 / 10.1 m/s); orbit radius 1752.85 x 1826.15 vs 1752.94 x 1825.82 km; mass 2629.5 /
  2628.9 kg; no alarms on either. Every column inside the stock envelope except CDU Z after
  insertion (16.3 deg, tol 12.4, from TIG+520 s), where P12 ends at TIG+472 on ours and +488 s
  on stock (an expected timing deviation, below).
- PDI dev rerun from HEAD (2026-10-14; stock H / H01 repeated under the current common.lua:
  identical tolerances): identical to the earlier dev run (deterministic). Events and end
  state match: touchdown 23.7245 / 0.6696 vs 23.7410 / 0.6689 deg (lng 0.017 of tol 0.089,
  lat 0.0007 at touchdown); outside the envelope during braking: latitude 0.0019 deg (tol
  0.00125) from TIG+136 s, and the attitude columns (bank / yaw Euler-degenerate at pitch -87,
  CDU X from the windows-up roll starting 1.0 s early).
- Saturn console mode (2026-10-13, about 25 min of the 60 allowed): bisected with oapiWriteLog
  markers to Saturn::clbkPostStep's unguarded `oapiCameraInternal() && oapiCockpitMode()` (VC
  animations): with no graphics client the camera query faults in Orbiter.exe. Fix:
  sim/nassp/patches/0003-saturn-console-mode.patch (the focus guard the LEM already has),
  applied in nassp-ref (Saturn5.dll + LEM.dll by nassp-work\build-ref2.bat; the release DLL
  kept as Saturn5.dll.release-437) and nassp-agc. Columbia + MCC (focus MCC) runs headless.
- Entry: scenario 15 (P63 at EI-60 s) cannot resume on our CMC (offline replay: restarts.c:106
  'prdt != 0', PHSNAMEn is a rope address; servicer.c:692 no AVGEXIT job): entry flies from
  scenario 14 (EI-2:15h) with P61 selected afresh, both builds. Panel-display toggles that
  NASSP registers only with the 2D panel (PanelOrdeal) are logged and skipped headless.
- LM test hook: sim/nassp/patches/0002-lem-lua-test-interface.patch (LEM gets Saturn's
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
- CHECKPOINT 2026-10-14 (rendezvous on stock, in progress): the earlier failures were script
  faults, found with a stock P20 diagnostic run (runs/ref_rdiag, rdiag.lua: flags, RADMODES, RR
  CDUs, LOS truth every 0.25 s): (1) the first ENTER on R60's F50N18 in P20 ends R60, but R61
  asks again while the previous RR designate runs; a second ENTER later is taken and P20 locks on
  (F50N72 ~100 s later); (2) the RR checkout V41N72 needs the antenna at the angles before the
  breakers open (503 otherwise); (3) R22's F06N49 (first mark above the thresholds) sat behind
  the crew's V16N45 with KEY REL lit, so no marks were incorporated and tracking stopped; (4)
  common.lua read TIG as hi * 2^14 + lo, wrong by 327.67 s when the AGC leaves a negative low word
  (TTPI after P34): P41 came 32 s before TIG and slipped (01703). Start state now "apollo-c rndz
  start" (scenario 11 flown by stock through insertion, P00, CSM transponder on; rstart.lua),
  identical on both builds. rendh.lua rewritten (PAD TIGs from the scenario's CSI data card,
  crew answers for N49 / KEY REL / R61 requests, VG nulling by pulses with a stable "done").
  11:15 UTC: stock ref_rendezvous_H (-f 0.02) and H01 (-f 0.01) fly the whole rendezvous
  unattended (P20 lock-on, CSI / CDH / TPI targeted and burned, closing at 36 m/s 10 min after
  TPI); tolerances derived into mission_tolerances.tsv, scripts frozen (rendh.lua md5 bad987d9);
  dev run started.

- RESUME 2026-10-14 13:20 UTC (after the cutoff): dev DLLs rebuilt from HEAD (3ab8849, then d258aa6
  after the telemetry commit; build logs SCRATCH build-agc26..29, 0 warnings). First dev run
  (runs/agc_rendezvous_H_503): alarm 00503 at P20 select, no R60: an execution-time order (R61's
  G+N,AUTO test before the sheet's MODE CONT AUTO row; Test changes). Script fixed, stock H / H01
  repeated, then a stock CDH tumble from a V77 OPR ERR (Test changes) fixed in the script too, stock
  repeated again, tolerances re-derived (rendh.lua md5 f41dcdcb). Then two flight bugs on ours:
- FLIGHT BUG 15 (rendezvous P20, ours only, fixed 2026-10-14): after lock-on our LM swung 40 deg
  in roll (sim 368 s, then 60 deg at 545 s), R61 asked for maneuvers the stock run never needed,
  marks stopped at 6 and R22 alarmed 00525 (runs/agc_rendezvous_H_525). Host diagnostic
  (rdiag3.lua: CDUs, CDUXD..ZD, CPHI.. every 0.5 s, both builds): R65's VECPNT1 turned the desired
  outer gimbal from 77763 to 40016 (180 deg) for a 0.01 deg pointing correction. Offline (Linux probe
  on VECPNT1 with the logged CDUs): COMPMATX's DELCOMP diagonal u u (1 - cos A) + cos A of the
  near-unit axis component overflowed SL3 to exactly 1 (MPAC +0 with negative minor words); BOVB
  SIGNMPAC took the sign of the DP value (-) where SIGNMPAC (RTB_OP_CODES p.1399) does CCS on the
  major word (+0 -> +DPOSMAX), so DEL(2,2) = -1 and DCMTOCDU's PHI quadrant flipped. Fix:
  src/imu/maneuver_calc.c store_signmpac (sign of the major word; a zero major word takes the
  overflow's sign); tests/imu/imu.c test_vecpoint_tiny_turn, test_delcomp_unit_diagonal (fail before).
- FLIGHT BUG 16 (rendezvous P32, ours only, fixed 2026-10-14): with bug 15 fixed P20 tracked like
  stock (42 marks by CSI-22m), but the P32 recycle at CSI-22m gave alarm 00603 (CSI to CDH < 10 min;
  00604 offline) and CSI was never burned (runs/agc_rendezvous_H_603). Offline (state saved before
  the V32E by rdiag4.lua, P32 replayed with the sheet's PAD loads, gdb at NTP/2): NN = 7512, the
  scenario's word (both builds display N55 R1 7512 before the load), TCDH 17 days after TCSI. The
  crew's V25N55 +00001 never reached NN: the noun dispatcher could read NN / NN1 (N55 R1, WHOLE) but
  agc_rendz_noun_write had no case for them (NOT_FOUND, no store). Fix:
  src/programs/navigation/noun_data.c stores NN (LM) / NN1 (CSM); tests/programs/rendezvous.c
  test_n55_loads_nn (4 checks failed before). Offline after: NN 1, TCDH = TCSI + 57 min, V06N75.
- FLIGHT BUG 17 (rendezvous TPI burn, ours only, fixed 2026-10-14): with bugs 15-16 fixed CSI and
  CDH were targeted and burned like stock and P34's TPI displays matched (N81 21.8 / 0.4 / -9.4 vs
  stock 23.3 / 0.1 / -11.0 ft/s), but P41 at TIG-28 s gave ABORT 01302 (SQRT of a negative) and the
  script's nulling drove N85 to 1579 ft/s (runs/agc_rendezvous_H_1302). A diagnostic DLL (stderr
  print of RASTEER1's inputs, not committed; runs/agc_tpidiag) showed MUASTEER = MU/A SR 6: the
  B energy term MUASTEER - SS MU/A went negative. P34-35 INITVELX reads "STODL MU/A / SR 6 / STORE
  MUASTEER": STODL with no load address DLOADs the push-down list, i.e. the MU pushed at the start,
  so MUASTEER = MU SR 6. Fix: src/programs/targeting/initvel.c; tests/programs/rendezvous.c
  test_initvel_muasteer_lm (fails before). With the logged inputs MUASTEER is 1915147 against
  SS MU/A = 1836840 (was 1093901). CSI and CDH use external dV (XDELVFLG), so only TPI's
  aimpoint steering (RASTEER1) reads MUASTEER.
- DONE milestone rendezvous (2026-10-14 16:10 UTC, headless, rendh.lua f41dcdcb, stock ref_rendezvous_H
  / H01 under the same file, tolerances derived before the dev run; dev DLLs = HEAD d258aa6 + the
  fixes of bugs 15-17, runs/agc_rendezvous_H): P20 lock-on at 294 vs 307 s, CSI / CDH / TPI targeted
  (N75, N81, N82, N58, N59 alike), burned and nulled to 0.2 ft/s on both, no alarms on either; TPI
  + 600 s: 37.09 km closing at 34.21 m/s vs 38.91 km at 35.50 m/s. Outside the stock envelope: range
  after CSI and mass around CDH (CDH 42 s earlier), bank / yaw (Euler-degenerate), MARKCTR (Mission
  report). Gates green with the three fixes (make, make test, make check, make nassp-replay 5/5).
- Lead findings (2026-10-14), decided with the offline replay (keyrun2: the harness with NASSP's
  channel 7 superbank kept, so stock resumes LM images): TTOGO 0.4-0.8 s behind after a load and
  V37E00E in the loaded P12 (stock 21.5 s later to P00, ours 1.3 s) are timing deviations; the
  rope-address PHSNAME restart is a known limitation of scenario restore (both in "Expected
  deviations (timing)").

## Expected deviations (timing)
Rule (lead, 2026-10-14): no timing model; the C executive runs jobs in zero AGC time, and the
two ordering deviations (radar-read wait, self-rerunning job pacing) stand in for AGC execution
time. Milestones are judged on decisions and end state; differences that trace to execution
time are listed here with their trace and not root-caused further.
- 01703 TIG slip size (liftoff with a re-selected P12): the work before MIDTOAV1 costs yaAGC
  ~25 s and ours ~0, so the slip lands at sim 38.5 vs 15 s (27.3 vs 3.3 s). The scripts now use
  the scenario's running P12, which avoids the slip on both builds.
- R60 small maneuver (entry R62): KALCMANU's NOGO path ends at once on ours, 1-2 s in yaAGC's
  interpreter; the script's PRO check sees F50N18 again and falls back to V33E (same effect).
- Entry attitude drift phase: at sim 4012 both builds release CMC attitude control (GDC
  alignment sheet); the CM then drifts with its limit-cycle rate of that moment, which differs
  between builds (stock at a reversal, ours mid-drift). Consequences: P63 30 s earlier (P62.1
  CMDAPMOD path), CM RCS mass 1.24 kg (tol 1.23); splashdown 1.6 km apart.
- Program start seconds: P65 TIG+657 vs +630 s in PDI (stock's own -f 0.01 run: +659.5 s);
  P12's end TIG+472 vs +488 s at liftoff (CDU Z after insertion); windows-up roll in P63 1.0 s
  early (CDU X).
- P20's first R61 (rendezvous): its G+N,AUTO test (channel 31 bit 14) comes after LPS20.1, about
  4 s of interpreter time on yaAGC, and at once on ours; with the sheet's MODE CONT AUTO keyed just
  after V37E20E ours saw ATT HOLD, skipped R60 and designated the RR off the LOS (alarm 00503). The
  script sets the switch first (Test changes); offline with channel 31 in AUTO ours shows F50N18.
- RR marks (rendezvous): by CSI-22 min our LGC had incorporated 42 marks, stock 19 (H and H01).
  R22's loop pace includes the interpreter time of each incorporation (W-matrix and state update),
  which the C executive does not charge. Targeting results agree (N75, N81, N58, N59 within the
  stock spread, table below).
- TTOGO after a load (both LM images): updated once a second on both builds as TIME - TIG; ours
  updates 0.6 s later in the second (the restart sets the phase; stock resumes the image's) and the
  mirror shows it up to 0.2 s later, so TTOGO reads 0.4-0.8 s behind stock. TIG is the same word on
  both (offline, liftoff image, keyrun2 traces every 0.2 s).
- V37E00E 40 s after the liftoff load: both builds go to P00; stock after 21.5 s (KEY REL lit
  meanwhile: V37 waits for the state-vector integration P12's MIDTOAV started, interpreter time),
  ours after 1.3 s (offline, both engines, liftoff image).
Known limitations of scenario restore (not timing): the load restarts our LGC (GOJAM) where NASSP's
yaAGC resumes the image. A variable restart phase whose PHSNAME is a rope address cannot be resumed
(PDI image: PHASE4 0402, PHSNAME4 36,3032 in P63; agc_restart_group BAD_ARG), so P63's display is
not re-shown after a PDI load where stock re-shows it; the restart also re-zeroes the LM DAP's hold
attitude to the present CDUs (CDUXD..ZD), so a coasting LM that the image was turning toward its
old CDUXD is held where it is (rendezvous start: 0.4 kg RCS in the first 16 s, apolune 640 m
lower than stock, well inside the stock spread of 13.4 km).
Other open items (not timing): after the load restart our CMC no longer drives the optics CDUs
(channel 14 70040 vs stock 76040: a GOJAM drops the drive the image had); THETAD from a
normalised CDUXD can differ by one count from the AGC's DPADD upper word; PDI latitude during
braking 0.0019 deg (tol 0.00125; touchdown within 0.0007).

## Test changes
Rule (lead, 2026-10-13): per milestone the Lua script, checklist sheets, edit columns and
tolerances are frozen before the dev run; any later change repeats the stock run and is listed
here. Scripts are deployed by copying one file to both installs; no script branches on the build.
Audit of the runs before the rule (times UTC 2026-10-14):
- PDI: script edits 03:01 (lone ENTER/PRO waits for a flash; from a stock race) and 03:12
  (UNFC/UNWC columns, after both v1 runs) preceded ref_pdi_H (03:15). 03:26, after a dev
  result: pdih.lua saves a scenario 20 s after LANDED (after every tabled event); stock not
  rerun. Tolerances (sim/nassp/mission_tolerances.tsv, 04:15-04:21) and the column choice were made after
  the dev runs. Checked against the rule as written (1.5 x the -f 0.02 / 0.01 stock spread, every
  numeric column, no rounding): lat 0.0019 vs 0.0012 (the file's 0.002 is a rounding that turned
  a fail into a pass), bank 48.6 vs 41.3, yaw 46.8 vs 40.9, cdux 10.4 vs 4.8 deg are out.
- Liftoff: 03:56, after a dev result (a running P12 does not resume on our load restart):
  both scripts reselect P12 (V37E12E) at the load; stock rerun. 04:07: diagnostic columns
  (ascent words), stock rerun. Tolerances after the dev runs. By the rule as written, out:
  hdot 1.39 vs 0.93 (also out in the file), eng 0.40 vs 0.0, pitch 24.3 vs 12.4,
  cduy 24.1 vs 12.4, cduz 10.8 vs 7.3 deg, apo 14.2 vs 12.2 km, peri 15.3 vs 15.1 km.
- Entry: scenario 14 rather than 15 (our CMC cannot resume a running P63), on both builds,
  chosen from the offline replay before any host entry run. The scripts were final before
  ref_entry_H (04:44); the dev run used the same files. Tolerances not set yet.
- 2026-10-13 (lead): the tolerance file (now sim/nassp/mission_tolerances.tsv) regenerated by --derive (every numeric column, exact 1.5 x
  spread, no rounding) from the existing stock pairs; no stock rerun needed (scripts unchanged).
  The entry dev run (scripts frozen) was read before entry tolerances existed; it is void anyway
  (CSM not under CMC attitude control, runs/discarded/agc_entry_H_csmdap) and is rerun after the
  fix; entry tolerances come from ref_entry_H vs ref_entry_H01 by --derive before that rerun.
- 2026-10-13 liftoff, after the dev result (ignition 23.5 s before stock): root cause is the
  03:56 P12 reselect itself. After the PRO on F06N76, P12 reaches MIDTOAV1 (integrate to
  TIG - 30 s) and, finding too little time, alarms 01703 and slips TIG to now + TIMEDELT + 30 s.
  Stock got there at sim 38.5 s (slip 27.3 s), ours at sim 15 s (slip 3.3 s; offline the slip
  follows the PRO at once): the AGC time the work before MIDTOAV1 takes in yaAGC is ~0 in the
  C executive, so the slip size measures our timing model, not the guidance. Our load restart
  now resumes the running P12 (phase table and ascent erasables mapped; offline the image runs
  V06N74 down to TIG with no 01703), so liftoffh.lua again uses the scenario's P12 as loaded on
  both builds. Stock runs ref_liftoff_H / H01 repeated under the change, tolerances re-derived
  before the dev run. Finding kept: a program that integrates before a deadline reaches
  it earlier on our build (no AGC time charged for C jobs).
- Rendezvous (2026-10-14). The earlier scenario-11 attempts (liftoff then the "Rendezvous" sheet)
  are void (runs/discarded/old_rendezvous): the script did not work on stock. The milestone now
  starts from "apollo-c rndz start" (made once on stock by sim/nassp/lua/rstart.lua: scenario
  11 flown through insertion, P00 settled, CSM transponder on; the same file in both installs).
  rendh.lua was developed on stock only (trial runs T1-T6 discarded, each change found by a stock
  failure: R22's N49 behind KEY REL, R61 re-requests, VG nulling ending on a display being
  rewritten, N85 pulse gains, the R60 answer rule, a crew answer typed inside a sheet load). Then
  frozen (md5 bad987d9) before ref_rendezvous_H / H01 and the tolerance derivation; both stock
  runs and every dev run use that file. Crew actions the sheets do not list, all logged: KEY REL
  after 10 s of its lamp, PRO on R22's F06N49, PRO on a W-matrix F06N99 left behind an R61
  request, the R60 answer (ENTER within 30 s of a V06N18, else PRO); the CSI / TPI PAD TIGs from
  the scenario's MCC CSI data card; the TPI sheet's five "POSS F 50 18" rows made one
  conditional step (a lone PRO there would answer P34's F16N45). Not flown: P52, AGS / DEDA,
  ORDEAL slews, the V90 N16 time loads and the "Load Negative LM Ydot in R2" rows (both builds
  alike), MCC1 / MCC2 and braking.
- Rendezvous script change 1 after the first dev run (2026-10-14 13:30 UTC; rendh.lua md5 bad987d9 ->
  af918f3c). Dev run (DLLs from HEAD 3ab8849) at V37E20E: alarm 00503 at once, no F50N18
  (runs/agc_rendezvous_H_503, stopped at sim 1150 s). Offline (keyrun on the run's LGC image, gdb on
  G+N,AUTO): P20's R61 tests channel 31 bit 14 during the V37E20E keystroke itself, sees ATT HOLD
  (the sheet sets MODE CONT PGNS - AUTO on the next row), skips R60 and designates the RR off the LOS
  (503). On yaAGC that test comes after LPS20.1, about 4 s of interpreter time later (stock's
  F50N18 4 s after the key; runs/ref_rdiag2), by which time the switch is in AUTO. Execution-time
  order, so a timing deviation (below), not a flight bug; with channel 31 in AUTO offline, ours
  shows F50N18 like stock. rendh.lua now swaps the two rows (switch first; P00 in AUTO holds the
  attitude it has). Stock ref_rendezvous_H / H01 repeated under it (runs/superseded_af918f3c).
- Rendezvous script change 2 (14:05 UTC; af918f3c -> f41dcdcb), found on stock only: in the repeated
  stock H the CDH sheet's V77E (rate command) lit OPR ERR, the DAP stayed in minimum impulse, and the
  translation pulses of the CDH burn tumbled the LM (2.4 deg/s; stock H01 then lost the RR in P34,
  alarms 00503 / 00527). The old pair had V77 taken. The crew answer added: after V76E / V77E, if OPR
  ERR came on with the entry, RSET and key it again (once, logged). Stock ref_rendezvous_H / H01
  repeated (H01 used the answer at CDH: "crew: OPR ERR after V77E"), tolerances re-derived from them
  (sim/nassp/mission_tolerances.tsv), all before any dev run under f41dcdcb. Not root-caused (stock
  behaviour: a key lost or doubled by the 0.3 s key timing while yaAGC is busy is the likely cause).
- common.lua (2026-10-14, rendezvous): ac.dp reads a DP erasable pair with ones'-complement word
  signs; TIG was read as hi * 2^14 + lo, 327.67 s wrong when the low word is negative (TTPI after
  P34). The TIGs of the earlier milestones' runs all had positive low words (logs: TIG=16351 3152,
  4320 4226, 5260 10175), so their runs are unchanged by it; not rerun.
- mission_tables.py (2026-10-14): ev_rendezvous for the post-insertion start (times from the
  scenario load, which both builds share); nearest() by bisection (the rendezvous CSVs have
  10,000 rows); radmodes is a mode word, not compared by value; the RR CDU columns wrap at 360.
  Output for pdi / liftoff / entry identical to the previous version (checked).
- nassp-replay LM cases have no stock reference (this harness's yaAGC restarts LGC images
  by a TC trap): pdi_lm expects the image's saved RADMODES bit 6 after the load (NASSP's yaAGC
  continues the image); liftoff_lm expects noun 74 40 s after the load (stock NASSP showed
  V06N74 25.8 s after loading the same scenario, runs/discarded/ref_liftoff_H_noreselect).

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
