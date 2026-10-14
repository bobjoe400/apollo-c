# nassp-integration -- Apollo 11 in NASSP 9 on our AGC (PLAN item 10b+10c)

Study: docs/overview/nassp_integration.md.

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
engine.c, the shim NASSP's vessels link, over build/libagc.a) and stock (NASSP's own
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
   and are then frozen (see "Test changes" in the git history).

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
from the run logs, Progress in the git history); 7-10 headless at a fixed 20 ms step, tables generated by
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

## Program flow and decisions

_Final documentation pass (2026-10-14), from `src/nassp/engine.c`,
`src/erasable_map/address_map.c`, `sim/nassp/replay.py` and
`sim/nassp/mission_tables.py` at HEAD 3b97dc8. The flight code the
mission exercised is in the owning lanes' sections; the bugs it found are
in the table above, each with its file and test._

* **Engine API** (`src/nassp/engine.c`, host-only: not in libagc, built
  by NASSP's CMake through `src/nassp/CMakeLists.txt` and
  `sim/nassp/patches/0001`). NASSP's C++ calls yaAGC's API; the shim
  answers it over libagc and the src/sim clock. `agc_engine_init` sets
  yaAGC's power-on inputs (channels 030-033, Z = 04000, RESTART lamp).
  `agc_load_binfile` takes the vehicle from the rope's name
  (`sim_rope_vehicle`; anything but Comanche055 / Luminary099 returns 1
  and the computer never starts), then `agc_power_on`, `sim_rope_load`,
  `sim_clock_init`, `sim_channels_init`, and opens the sidecar trace
  `apollo-c-agc-<vehicle>.log` (`trace`). `agc_engine` is one memory
  cycle (11.71875 us); every 80/3 calls is one 1/3200 s scaler count,
  `one_count`, the only place the shim works.
* **One scaler count** (`one_count`): import NASSP's stores
  (`import_erasable`: the registers and counters every count, the whole
  mirror once a centisecond, each changed cell stored by address through
  `agc_erase_write_ecadr`; Z = 04000 means power came back),
  `import_channels` (a key code left in channel 015/016 by a scenario
  load is stored without KEYRUPT; otherwise `sim_input`), then GOJAM
  when due (`power_up` -> `agc_rupt_go`: first count after the load, or a
  power cycle; an LGC scenario load keeps the image's RADMODES bit 6,
  flight bug 5), `import_rupts`, one `sim_clock_count`, `trace_alarms`,
  `lm_drives` (THRUST on 0142, ALTM on 0143 when channel 14 enables
  them), then export: the registers every count, the display words every
  centisecond (`export_display`), the whole erasable every 0.1 s
  (`export_erasable`, through `agc_erase_read_ecadr`), all channels and
  the superbank (`export_channels`), pending interrupts (`export_rupts`),
  Standby.
* **Host hooks** (src/sim's `internal.h`, defined here instead of
  src/sim/socket.c): `sim_socket_send` renumbers the optics/TVC drives to
  NASSP's 0140/0141 and mirrors the DSKY lamp channel 0163;
  `sim_host_downrupt_external` returns true (NASSP's PCM telemetry raises
  DOWNRUPT, not the clock); `sim_host_radar_read` asks NASSP for the
  radar word (`RequestRadarData`) and stores RNRAD.
* **Scenario restore** (`src/erasable_map/address_map.c`): a scenario's
  AGC_BEGIN block reaches every erasable through the map; a flagword goes
  to `agc_flagword_get/set`, a mapped address to its owner (the first map
  entry holding it, `first_holder`), anything else to the raw image. The
  executive, waitlist and phase-table continuations have no address, so
  the load restarts the computer (GOJAM -> GOPROG) with the erasable
  kept, and the restart groups re-run from PHASEn / TBASEn. Five of the
  seventeen flight bugs (1, 5, 12-14) were erasables the map did not
  carry. Known limitation: a type C phase whose PHSNAMEn is a rope
  address is not resumed (the C keeps job ids; `agc_restart_group` ends
  BAD_ARG for that group).
* **Offline tier** (`make nassp-replay`, `sim/nassp/replay.py:main`):
  builds `sim/nassp/replay.c` twice, over our shim and over NASSP's yaAGC
  (downloaded at the pinned commit, or SKIP), replays the five images of
  `sim/nassp/images/` (RSET, V37E00E, 20 minutes of P00 integration, an
  MCC uplink by INLINK + UPRUPT, V33) and `compare`s the transcripts word
  for word: the state-vector blocks, UPBUFF, UPSVFLAG, MODREG, DSPTAB, and
  each image's downlist (`check_downlist`, `DOWNLIST_SKIP` for the words
  the C cannot hold; docs/modules/telemetry.md). DP pairs compare by
  value within TOL_LSB = 2.
* **Mission tier** (`make nassp-mission`, `sim/nassp/mission_tables.py`):
  pairs the stock and dev edit CSVs of each milestone (`compare`), finds
  the events the same way in both (`ev_pdi`, `ev_liftoff`,
  `ev_rendezvous`, `ev_entry`), and checks every column against
  `sim/nassp/mission_tolerances.tsv`, derived by `derive` from two stock
  runs (1.5 x their spread) and frozen before the dev run; exit 1 when a
  quantity leaves its tolerance or an event is missing.
