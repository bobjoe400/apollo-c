# guid-landing -- Luminary landing, aborts, ascent, throttle, FINDCDUW

## Scope

Luminary 099 only. AGC sources (pages are the Luminary 1A listing pages):

| AGC file                               | pages    | C file                         |
|----------------------------------------|----------|--------------------------------|
| THE_LUNAR_LANDING.agc                  | 785-792  | src/lm/landing/lunar_landing.c         |
| THROTTLE_CONTROL_ROUTINES.agc          | 793-797  | src/lm/landing/throttle.c              |
| LUNAR_LANDING_GUIDANCE_EQUATIONS.agc   | 798-828  | src/lm/landing/guidance.c (equations) + src/lm/landing/lunar_landing.c (R13, P64/P66/P67 entry, redesignation, ROD) |
| P70-P71.agc                            | 829-837  | src/lm/landing/aborts.c               |
| P12.agc                                | 838-842  | src/lm/ascent.c                |
| ASCENT_GUIDANCE.agc                    | 843-856  | src/lm/ascent.c                |
| LANDING_ANALOG_DISPLAYS.agc            | 898-907  | src/lm/landing/displays.c      |
| FINDCDUW--GUIDAP_INTERFACE.agc         | 908-925  | src/lm/landing/findcduw.c              |

Shared lane state (pad loads and erasables used by more than one of the
files above) lives in src/lm/landing/state.c behind accessors declared in
include/agc/lm/landing.h. Symbols owned by other lanes that do not exist
yet are declared in ONE place, include/agc/agc_landing_pending.h (see "Requested core
additions"); integration replaces that header with the real includes.

## Design table

Scale notation: "2^N u" = fraction * 2^N in unit u. cs = centiseconds.

### Throttle (throttle.c)

| AGC label | C function | inputs (scale) | outputs (scale) |
|---|---|---|---|
| THROTTLE | agc_lm_throttle | ABDELV (2^14 cm/s per 2 s), MASS (2^16 kg) | via AFDUMP |
| THROTTLE +3 / AFDUMP | agc_lm_throttle_measured | measured accel (2^-4 m/cs^2), /AFC/ (2^-4 m/cs^2) | FP, FC, FCODD (DP, high word = pulses) |
| WHERETO/LOWFCOLD/FCOMPSET/FLATOUT1/2 | throttle_region | FCOLD, FCODD, FP, LOWCRIT, HIGHCRIT (pulses) | PIFPSET, FCODD |
| DOPIF/DOIT | throttle_dopif/throttle_doit | FCODD, FP, PIFPSET | PIF, THRUST counter, CH14 bit 4, TTHROT |
| FWEIGHT calc | throttle_fweight | PIF, TIME1, PIPTIME/THISTPIP, THROTLAG | FWEIGHT (pulses DP) |
| FLATOUT / WHATOUT | agc_lm_throttle_flatout / _whatout | pulses | THRUST |
| MASSMULT | agc_lm_massmult | accel (2^-4 m/cs^2) | force pulses DP (hi word = pulses) |

### Landing guidance (guidance.c)

| AGC label | C function | key data (scale) |
|---|---|---|
| LUNLAND, GUILDRET | agc_lm_lunland, ldg_guildret | TPIP, TPIPOLD, PIPTIME1 (2^28 cs) |
| NEWPHASE/PREGUIDE/WHATGUID/AFTRGUID/WHATEXIT/WHATDISP tables | switch on WCHPHASE (-1..2) | |
| TTFINCR | ldg_ttfincr | TTF/8 (2^14 cs, i.e. TTF at 2^17), LAND (2^24 m), WM (rad/cs) |
| CALCRGVG / RGVGCALC | ldg_calcrgvg / ldg_rgvgcalc | V (2^7 m/cs), ANGTERM (2^9), VGU (2^10 m/cs), RGU (2^24 m), VHORIZ (2^7), LOOKANGL (deg, SP integer) |
| TTF/8CL + ROOTPSRS | ldg_ttf8cl, agc_lm_rootpsrs | cubic in TTF/8, 8 Newton passes max |
| QUADGUID, AFCCALC1-3 | ldg_quadguid, agc_lm_afccalc | ACG, UNFC/2 (2^-4 m/cs^2), /AFC/ |
| CGCALC | ldg_cgcalc | CG rows (half unit) |
| EXTLOGIC, EXGSUB, EXBRAK, EXNORM | ldg_extlogic etc. | UNWC/2 |
| STEER?, EXVERT, RATESTOP, GDUMP1, DISPEXIT, P63DISPS, VERTDISP | ldg_steer_exit, ldg_dispexit | |
| P65VERT | ldg_velocity_nulling_vertical | V2FG, TAUVERT |
| TDISPSET | agc_lm_tdispset | TTFDISP (2^28 cs), TREDES (s, SP integer) |
| 1406P00 / 1406ALM | in ldg_ttf8cl | alarm 01406 |
| FASTCHNG | agc_lm_fastchng | group 3 |

### Landing program flow (lunar_landing.c)

| AGC label | C function |
|---|---|
| P63LM, FLAGORGY | agc_lm_job_braking_phase |
| IGNALG, IGNALOOP, ?GUIDSUB, GUIDSUB, EXGSUB loop | agc_lm_ignalg (bounded loop) |
| DDUMCALC, DDUMGOOD, R60INIT | ign_ddumcalc, ign_ddumgood |
| IGNALGRT/ASTNCLOK, ASTNRET/P63SPOT2..4 | agc_lm_job_ignalgrt, agc_lm_job_astnret, agc_lm_job_braking_phase_antenna_check |
| GUILDENSTERN (R13) | agc_lm_guildenstern |
| STARTP66, STRTP66A, VRTSTART, STARTP67 | lnd_start_rate_of_descent_landing / lnd_rate_of_descent_setup / lnd_vrtstart / lnd_start_manual_landing |
| STARTP64, P65START | agc_lm_start_approach_phase, agc_lm_start_velocity_nulling |
| REDESIG | agc_lm_redesig |
| P64DISPS, P64CEED, RED-OVER, REDES-OK | agc_lm_approach_phase_display, agc_lm_job_approach_phase_proceed |
| P66VERTA, RODTASK, RODCOMP, VHORCOMP | agc_lm_rate_of_descent_vertical, agc_lm_task_rodtask, agc_lm_job_rodcomp |
| P67VERT | agc_lm_manual_landing_vertical |
| PITFALL, REDESMON, COUNT'EM | agc_lm_pitfall, agc_lm_task_redesmon |
| DESCBITS | agc_lm_descbits |
| LANDJUNK (P68) | agc_lm_job_landing_confirmation, agc_lm_job_landing_confirmation_proceed |

### Aborts (aborts.c), ascent (ascent.c), R10 (displays.c), FINDCDUW (findcduw.c)

| AGC label | C function |
|---|---|
| R10,R11, R10,R11A, FLASHH?, 10,11, P71NOW?, P70NOW? | agc_lm_task_abort_monitor |
| P70, P71, P70A, P71A, LEGAL?, ABORTALM | agc_lm_dps_abort, agc_lm_aps_abort, lm_legal |
| CONTABRT, ABRTJASK | agc_lm_abrtjask |
| GOABORT, P70INIT, BOTHPOLY, UPRATE, YOK, UPTHROT, UPTHROT1, P71RET, OLDTIME | agc_lm_job_goabort and helpers |
| TGOCOMP, THROTUP | agc_lm_tgocomp, agc_lm_throtup |
| P12LM..P12RET, P12INIT, COMMINIT, GUIDINIT | agc_lm_job_powered_ascent, agc_lm_job_powered_ascent_newload, agc_lm_powered_ascent_init, agc_lm_comminit, agc_lm_guidinit |
| ATMAG, ASCENT ... AIMER, MAINLINE, ASCTERM*, CHECKALT, EXITVR, ANG1CHEK, CHECKYAW, KEEPVR | agc_lm_atmag, agc_lm_ascent and helpers |
| ENGOFF, ENGOFF1, CUTOFF, CUTOFF1, CUTOFF2, TERMASC | asc_engoff, agc_lm_task_engoff1, agc_lm_job_cutoff... |
| YCOMP, LOGSUB | agc_lm_ycomp, agc_lm_logsub |
| LANDISP .. ABORTON | agc_lm_landisp and helpers, agc_lm_task_intlze, agc_lm_task_speedrun2 |
| INITCDUW, FINDCDUW, FLTRSUB, UNWCTEST, DCMCL, NB2CDUSP, DVBYCOSM, ARCTRGSP, SPARCSIN, LIMITSUB, ONESTO2S, NOATTCNT, ALARMMGA | agc_lm_initcduw, agc_lm_findcduw, static helpers, agc_lm_sparcsin, agc_lm_limitsub |

## Deviations

- FASTCHNG (state.c): group 3 re-armed at the LUNLAND phase; a restart repeats the whole guidance pass (C cannot restart mid-function).
- THROTTLE (throttle.c): -0 values (PIFPSET initial, WHATOUT FCOLD/PIF) modelled as +0 (same pulses, FWEIGHT 0); FLATOUT1/LOWFCOLD FCODD low word = L = BUF+1 (scratch shared with ROOTPSRS etc.), modelled as 0. (MASSMULT overflow is exact since review: L = 02410.)
- ROOTPSRS (guidance.c): DXCRIT ignores the stale MPAC+2 product SHORTMP adds (at most a 1-LSB carry).
- guidance.c QUADGUID coefficients: stored SP over stale low words in the AGC, here as clean DP (low word 0).
- IGNALG (lunar_landing.c): loop bounded; on exhaustion alarm 01412 and NIGNLOOP left 0; returns AGC_STATUS_NO_CONVERGE.
- FINDCDUW (findcduw.c): DV's L after LIMITSUB modelled as A.
- R10 (displays.c): LANDISP's "LXCH PIPCTR1 / DXCH TBASE2" done via agc_phase_newphase(2, current phase, set_tbase) (no TBASE2 setter in agc_exec.h); VMONITOR's AD NEG0 (+0 -> -0 for the DINC hardware) omitted; DDOUBL overflow kept wrapped; LADQSAVE return address modelled as a static enum (ALTROUT +1 / ALTOUT +1) resumed by task SPEEDRN2 after the 40 ms VARDELAY.

## Stubs

None in this lane. Calls into other lanes that may still return AGC_STATUS_UNIMPLEMENTED are tolerated by assertion (REFMF, R60/R51 continuation paths).

## Tests written

tests/lm/landing.c (LM). Compiles with the project flags + -DAGC_HOST_DEBUG.
1. Throttle law in the 10%-60% band: MASS 8000 kg, commanded 2.0 m/s^2 (FC 1276.7 pulses), measured 1.5 m/s^2 (FP 957.6) -> 319 pulses to THRUST (PIFPSET 0); above HIGHCRIT -> FCODD = FMAXPOS, PIFPSET = FEXTRA.
2. QUADGUID with the vehicle exactly on r = RDG + VDG T + ADG T^2/2 (T = -400 s, LEADTIME -1 cs) -> ACG = ADG to 1e-4 relative (zero correction); 10 m x offset -> correction -12 dx/T^2 = -0.00075 m/s^2 exactly as expected.
`make test` builds and runs it (17 checks). The oracle differential test
tests/oracle/landing.c (oracle-cases lane) checks one guidance pass,
CGCALC, six THROTTLE cases and FINDCDUW against Luminary099 in yaAGC:
172 checks, no KNOWN entries since review-guid-landing (F9 fixed).

## Program flow and decisions

_Documentation wave (2026-10-12); prose and tables brought to the final tree (HEAD 3b97dc8) by the final pass (2026-10-14). The prose was written by reading the code; the two tables were extracted from the function bodies (flag, alarm, channel and scheduling calls; `agc_erase_*` accessors) and list only what the code does. Files: `src/lm/landing/*.c`, `src/lm/ascent.c`._

The P63-P66 sequence is drawn in
[flowcharts/lunar-landing.md](../overview/flowcharts/lunar-landing.md)
and P12 in [flowcharts/powered-ascent.md](../overview/flowcharts/powered-ascent.md).
The decision points:

* **P63 ignition** (`src/lm/landing/lunar_landing.c`): IGNALG iterates
  the ignition time (`ll_ign_guidance`: three passes of quadratic
  guidance per iteration); 40 passes without convergence (NIGNLOOP) is
  ALARM 01412 (IGNALOOP bounded at 64 passes, NO_CONVERGE, C only). R51
  fine alignment and R60 follow; ASTNRET (`agc_lm_job_astnret`): PROCEED
  aligns, ENTER returns to P63SPOT2. Ignition itself is BURN BABY BURN
  (guid-powered).
* **Guidance cycle** (`src/lm/landing/guidance.c`), every 2 s from
  SERVICER, indexed by WCHPHASE (-1 IGNALG, 0 BRAKQUAD, 1 APPRQUAD, 2
  VERTICAL): `ldg_quadratic` (WHATGUID) computes TTF/8 with ROOTPSRS
  (`agc_lm_rootpsrs`: eight passes then NO_CONVERGE); no convergence is
  POODOO 01406 in IGNALG and ALARM 01406 + RATESTOP in flight.
  `ldg_extlogic` (EXTLOGIC) moves to the next phase when TTF/8 passes
  TENDBRAK / TENDAPPR (FASTCHNG, WCHPHASE = WCHPHOLD + 1): BRAKQUAD ->
  STARTP64 (P64, LPD redesignation enabled, RUPT10 on) -> P65 velocity
  nulling (`agc_lm_start_velocity_nulling`, WCHVERT = -2). `agc_lm_steer_exit`
  (STEER?): STEERSW down -> RATESTOP, no outputs; any overflow in
  guidance -> ALARM 01410 (non-abortive) and RATESTOP; otherwise THROTTLE
  and FINDCDUW, then the phase display (V06N63 / V06N64 unless FLUNDISP).
* **Crew modes** (`lunar_landing.c:agc_lm_guildenstern`, R13 every 2 s):
  each change of the manual throttle discrete selects P67 or P66; each
  appearance of attitude hold selects P66 (rate-of-descent: P66VERTA,
  RODTASK each second -> RODCOMP, `agc_lm_rate_of_descent_vertical`,
  `agc_lm_job_rodcomp`). LPD: hand-controller RUPT10 in P64 only
  (`agc_lm_pitfall`) starts REDESMON (`agc_lm_task_redesmon`, every 70
  ms, counts clicks); REDESIG (`agc_lm_redesig`) moves the site with
  REDFLAG set and TREDES not zero. P68 (`agc_lm_job_landing_confirmation*`):
  V06N43 flash, PROCEED saves the attitude for P57.
* **Throttle** (`throttle.c:agc_lm_throttle`, `agc_lm_throttle_measured`):
  FC (desired) vs FP (present) force in throttle pulse units; the 10%..94%
  region logic with a throttle-up to the fixed full-thrust region; PIF =
  FC - FP + PIFPSET to the THRUST counter; FLATOUT (`agc_lm_throttle_flatout`)
  goes to full.
* **FINDCDUW** (`findcduw.c`): thrust and window command vectors to
  gimbal increments and commanded rates for the DAP; NORMUNIT overflow is
  ALARM 00402 (`fc_noattcnt`) then STOPRATE; CDUZ limited to 70 deg is
  ALARM 00401 (`fc_gimbal_changes`).
* **Displays** (`displays.c`): R10 four times a second drives ALTM
  (alternating altitude and rate) and the cross-pointers (CDUSCMD /
  CDUTCMD) when the mode switch is PGNCS (`ld_disindat`: first pass sets
  DIDFLAG and initialises after 80 ms).
* **Aborts** (`aborts.c`): R10,R11 (`agc_lm_task_abort_monitor`, every
  0.25 s while AVEGFLAG) watches ABORT / ABORT STAGE and flashes the LR
  lamps; GOABORT selects P70 (DPS) or P71 (APS); an illegal V37 P70/P71 is
  OPR ERR (ABORTALM). Both hand over to ascent guidance.
* **Ascent** (`src/lm/ascent.c`): ATMAG / ASCENT (`agc_lm_ascent`) per
  cycle; ENGOFF (`as_engoff`) schedules the engine-off task at TGO
  (group 4, variable delay ENGOFFDT) and sets IDLEFLAG; CUTOFF
  (`agc_lm_job_cutoff`) shows V16N63 then V16N85 until PROCEED; P12 ends
  through TERMASC in P00.

### Decision points by function

67 of the lane's 179 functions test or change a flag, raise an alarm, test a channel bit or start a job/task (AGC = the label(s) the prolog names; "+F" sets flag F, "-F" clears it; "starts" = NOVAC/FINDVAC, WAITLIST, LONGCALL, DELAYJOB targets).

| C function | AGC | flags tested | flags set (+) / cleared (-) | alarms | channel bits | starts |
|---|---|---|---|---|---|---|
| `agc_lm_powered_ascent_init` (ascent.c) | P12INIT | FLAP | - | - | - | - |
| `as_t2test` (ascent.c) | T2TEST CONST | FLPC | +FLPC | - | - | - |
| `as_ascterm_display` (ascent.c) | ASCTERM1 | FLRCS FLUNDISP | - | - | - | - |
| `as_ascterm` (ascent.c) | ASCTERM | FLRCS | - | - | - | - |
| `as_mainline` (ascent.c) | MAINLINE ANG1CHEK CLRXFLAG | ROTFLAG | -ROTFLAG -NO_POWERED_DESIGNATE -XOVINFLG | - | - | - |
| `as_exitvr` (ascent.c) | EXITVR | ROTFLAG | -FLVR | - | - | - |
| `as_engoff` (ascent.c) | ENGOFF | - | +IDLEFLAG | - | - | task ascent_engine_off |
| `agc_lm_ascent` (ascent.c) | ASCENT | FLPI FLRCS FLVR IDLEFLAG | +XOVINFLG -ROTFLAG | - | - | - |
| `agc_lm_job_atmag` (ascent.c) | ATMAG | FLRCS | -SURFFLAG -RENDWFLG | - | - | - |
| `agc_lm_task_ascent_engine_off` (ascent.c) | ENGOFF1 | - | - | - | - | job cutoff |
| `agc_lm_job_cutoff` (ascent.c) | CUTOFF | - | +FLRCS | - | - | - |
| `agc_lm_job_cutoff_display` (ascent.c) | CUTOFF | FLRCS | - | - | - | - |
| `agc_lm_job_cutoff_proceed` (ascent.c) | CUTOFF1 | FLRCS | - | - | - | - |
| `agc_lm_job_cutoff_trim_display` (ascent.c) | CUTOFF2 | FLRCS | - | - | - | - |
| `agc_lm_job_termasc` (ascent.c) | TERMASC | - | -LETABORT | - | - | - |
| `agc_lm_job_powered_ascent` (ascent.c) | P12LM | - | +MUNFLAG +ACC4_2FL +ALTITUDE_METERS_ONLY -RNDVZFLG | - | - | - |
| `agc_lm_job_powered_ascent_flash` (ascent.c) | P12 | MUNFLAG | - | - | - | - |
| `agc_lm_job_powered_ascent_proceed` (ascent.c) | PROCEED | - | +FLPI | - | - | - |
| `agc_lm_job_newload` (ascent.c) | NEWLOAD | FLPI | - | - | - | - |
| `agc_lm_job_newload_proceed` (ascent.c) | PROCEED | - | +FLVR | - | - | - |
| `agc_lm_job_powered_ascent_burn` (ascent.c) | P12RET | - | -FLPI | - | - | - |
| `agc_lm_task_abort_monitor` (aborts.c) | R10 R11 | AVEGFLAG | - | - | - | task abort_monitor |
| `ab_abrtjask` (aborts.c) | ABRTJASK | ENGONFLG | +APSFLAG -ULLAGFLG -DRIFTDFL -XOVINFLG +ENGONFLG +ALTITUDE_METERS_ONLY | - | - | - |
| `agc_lm_abort_monitor_resume` (aborts.c) | R10 | HFLSHFLG LETABORT VFLSHFLG | - | - | ch30 ABORTAPS, ch30 ABORTDPS | - |
| `ab_legal` (aborts.c) | - | AVEGFLAG LETABORT | - | - | - | - |
| `agc_lm_throtup` (aborts.c) | THROTUP | - | - | - | ch14 THRUSTACT | - |
| `ab_upthrot` (aborts.c) | UPTHROT | FLVR ROTFLAG | +FLVR +ROTFLAG | - | - | - |
| `agc_lm_job_abort_auto_control_check` (aborts.c) | UPTHROT1 | FLAP FLVR | - | - | - | - |
| `agc_lm_job_aps_abort_oldtime` (aborts.c) | OLDTIME | FLAP | - | - | - | - |
| `agc_lm_job_goabort` (aborts.c) | GOABORT | FLAP | -FLRCS -FLUNDISP -IDLEFLAG +ACC4_2FL -LETABORT +FLAP | - | - | - |
| `ld_disprset` (displays.c) | DISPRSET ABORTON | ALTITUDE_METERS_ONLY DIDFLAG | -DIDFLAG | - | ch12 DISPINERT, ch12 ENBRRCDUERR | - |
| `ld_dataout` (displays.c) | DATAOUT | - | - | - | ch14 ALTMETERACT | - |
| `ld_continue` (displays.c) | LADQSAVE | - | - | - | ch14 ALTRATESEL | - |
| `ld_disindat` (displays.c) | DISINDAT | ALTITUDE_METERS_ONLY DIDFLAG | +DIDFLAG | - | ch12 DISPINERT, ch30 DISPINERT | task speedrun_resume, task intlze |
| `agc_lm_landisp` (displays.c) | LANDISP | DIDFLAG SWANDISP | - | - | - | - |
| `agc_lm_task_intlze` (displays.c) | INTLZE | - | - | - | ch12 ENBRRCDUERR | - |
| `agc_lm_vmonitor` (displays.c) | VMONITOR | - | - | - | ch14 DRIVECDUS, ch14 DRIVECDUT | - |
| `agc_lm_task_speedrun_resume` (displays.c) | SPEEDRUN | ALTITUDE_METERS_ONLY | - | - | - | - |
| `fc_fetch_basic` (findcduw.c) | - | CSMDKFLG XOVINFLG | - | - | ch30 LMCONTROL, ch31 AUTOSTAB | - |
| `fc_noattcnt` (findcduw.c) | NOATTCNT | - | - | ALARM 0402 | - | - |
| `agc_lm_findcduw_limit_mga` (findcduw.c) | - | - | - | ALARM 0401 | - | - |
| `agc_lm_findcduw` (findcduw.c) | FINDCDUW | ENGONFLG | - | - | - | - |
| `ldg_ratestop` (guidance.c) | RATESTOP | - | - | - | ch31 ATTHOLD | - |
| `agc_lm_steer_exit` (guidance.c) | EXVERT EXOVFLOW RATESTOP | STEERSW | - | ALARM 1410 | - | - |
| `agc_lm_job_approach_phase_display` (guidance.c) | P64DISPS REDES-OK | REDFLAG | -REDFLAG | - | - | - |
| `agc_lm_job_approach_phase_proceed` (guidance.c) | P64CEED | - | +REDFLAG | - | - | - |
| `agc_lm_dispexit` (guidance.c) | DISPEXIT | FLUNDISP | - | - | - | - |
| `agc_lm_start_approach_phase` (guidance.c) | STARTP64 | - | -REDFLAG | - | ch13 RSTTRAP31A | - |
| `agc_lm_start_velocity_nulling` (guidance.c) | P65START | - | -XOVINFLG | - | - | - |
| `ldg_quadratic` (guidance.c) | WHATGUID WHATEXIT | - | - | POODOO 1406, ALARM 1406 | - | - |
| `ll_ign_guidance` (lunar_landing.c) | IGNALOOP EXGSUB | - | - | ALARM 1412 | - | - |
| `ll_ignaloop` (lunar_landing.c) | IGNALOOP | - | +INTYPFLG +MOONFLAG | - | - | - |
| `agc_lm_job_braking_phase` (lunar_landing.c) | P63LM | - | -NOTHROTL -REDFLAG -LRBYPASS +MUNFLAG -PREFERRED_TRACKING_ATTITUDE_ACTIVE -RNDVZFLG | - | ch12 RRLOCKON | - |
| `agc_lm_job_braking_phase_antenna_check` (lunar_landing.c) | P63SPOT3 | - | - | - | ch33 LRPOS1 | - |
| `ll_rate_of_descent_setup` (lunar_landing.c) | STRTP66A | - | +RODFLAG | - | - | - |
| `ll_vrtstart` (lunar_landing.c) | VRTSTART | - | -XOVINFLG -REDFLAG | - | - | - |
| `agc_lm_guildenstern` (lunar_landing.c) | GUILDEN | RODFLAG | - | - | ch30 AUTOTHROT, ch31 ATTHOLD | - |
| `agc_lm_redesig` (lunar_landing.c) | REDESIG | REDFLAG | - | - | - | - |
| `agc_lm_rate_of_descent_vertical` (lunar_landing.c) | P66VERTA | - | - | - | - | task rodtask |
| `agc_lm_task_rodtask` (lunar_landing.c) | RODTASK | - | - | - | - | job rodcomp |
| `agc_lm_pitfall` (lunar_landing.c) | PITFALL | - | - | - | - | task redesmon |
| `ll_resetrpt` (lunar_landing.c) | RESETRPT | - | - | - | ch13 RSTTRAP31A | - |
| `agc_lm_task_redesmon` (lunar_landing.c) | REDESMON | - | - | - | ch31 ATTHOLD | task redesmon |
| `agc_lm_descbits` (lunar_landing.c) | DESCBITS | - | - | - | ch16 DESCENTDN, ch16 DESCENTUP | - |
| `ll_landjunk` (lunar_landing.c) | LANDJUNK | ERADFLAG | +SURFFLAG -LETABORT +APSFLAG +LUNAFLAG | - | - | - |
| `agc_lm_job_landing_confirmation_flash` (lunar_landing.c) | LANDJUNK | SURFFLAG | - | - | - | - |
| `thr_doit` (throttle.c) | DOIT | - | - | - | ch14 THRUSTACT | - |

### Shared erasables read and written

Accessors of `include/agc/core/state.h` and the lanes' public headers called by this lane, with the number of its functions that read / write each. Lane-private erasables are `static` in the lane's files or in its state struct; their scales are on the declarations.

| erasable (accessor) | meaning and scale (accessor prolog) | read by | written by |
|---|---|---|---|
| `1jacc` | none -- accessor of the DAP erasable 1JACC (ERASABLE_ASSIGNMENTS.agc).; returns agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h | 1 | 0 |
| `1jaccq` | none -- accessor of the DAP erasable 1JACCQ (ERASABLE_ASSIGNMENTS.agc).; returns agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h | 1 | 0 |
| `1jaccr` | none -- accessor of the DAP erasable 1JACCR (ERASABLE_ASSIGNMENTS.agc).; returns agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h | 1 | 0 |
| `abdelv` | /DELV/ of the last PIPA reading, cm/s, scale 2^14 (ABDELV, LM) | 1 | 0 |
| `altm` | ALTM (LM 060): altitude / altitude-rate meter drive, raw 15-bit word. | 1 | 1 |
| `cdutcmd` | CDUTCMD (053) / CDUSCMD (054): optics (CSM) or RR (LM) CDU error counter drive, SP pulse counts; which 0 = CDUTCMD, 1 = CDUSCMD. In TVC mode the same  | 0 | 1 |
| `cdux` | CDU X (outer gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (register CDUX, 032) | 1 | 0 |
| `cduxd` | none -- accessor of the DAP erasable CDUXD (ERASABLE_ASSIGNMENTS.agc).; returns uint16_t -- the erasable, at the scale given in agc/lm/dap.h | 2 | 0 |
| `cduy` | CDU Y (inner gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUY) | 1 | 0 |
| `cduyd` | none -- accessor of the DAP erasable CDUYD (ERASABLE_ASSIGNMENTS.agc).; returns uint16_t -- the erasable, at the scale given in agc/lm/dap.h | 2 | 0 |
| `cduz` | CDU Z (middle gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUZ) | 1 | 0 |
| `cduzd` | none -- accessor of the DAP erasable CDUZD (ERASABLE_ASSIGNMENTS.agc).; returns uint16_t -- the erasable, at the scale given in agc/lm/dap.h | 2 | 0 |
| `delcdux` | - | 0 | 1 |
| `delcduy` | - | 0 | 1 |
| `delcduz` | - | 0 | 1 |
| `delperor` | - | 0 | 1 |
| `delqeror` | - | 0 | 1 |
| `delreror` | - | 0 | 1 |
| `delv` | accumulated PIPA delta-V, stable-member axes (DELV, I(6)): PIPA pulses scale 2^14 (count in the high word); 1 pulse = 5.85 cm/s CSM, 1 cm/s LM. KPIP1  | 1 | 0 |
| `imodes33` | last-sampled channel 33 / DAP-enable bits, T4RUPT C33TEST (IMODES33, B(1)PRM) | 6 | 1 |
| `m22` | M22 (GPMATRIX, T4RUPT_PROGRAM.agc p.188) -- SIN(OG), also MR22 RETURNS: M22, scaled at 1; returns M22, scaled at 1 | 1 | 0 |
| `m32` | M32 (GPMATRIX, T4RUPT_PROGRAM.agc p.189) -- COS(OG), also MR23 RETURNS: M32, scaled at 1; returns M32, scaled at 1 | 1 | 0 |
| `mass` | LM vehicle mass, kg, scale 2^16, DP (MASS: SERVICER MASSMON, 1/ACCS) | 4 | 0 |
| `modreg` | MODREG major mode (program) | 11 | 1 |
| `omegapd` | none -- accessor of the DAP erasable OMEGAPD (ERASABLE_ASSIGNMENTS.agc).; returns agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h | 1 | 1 |
| `omegaqd` | none -- accessor of the DAP erasable OMEGAQD (ERASABLE_ASSIGNMENTS.agc).; returns agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h | 1 | 1 |
| `omegard` | none -- accessor of the DAP erasable OMEGARD (ERASABLE_ASSIGNMENTS.agc).; returns agc_sp_t -- the erasable, at the scale given in agc/lm/dap.h | 1 | 1 |
| `pipax` | PIPA X pulse count since last read (register PIPAX, 037), unscaled pulses: 1 pulse = 5.85 cm/s (CSM SERVICER207), 1 cm/s (LM) | 2 | 0 |
| `pipay` | PIPA Y pulse count (register PIPAY) | 2 | 0 |
| `pipaz` | PIPA Z pulse count (register PIPAZ) | 2 | 0 |
| `piptime` | time of the PIPA reading / this vehicle's state vector, centiseconds, scale 2^28 (PIPTIME, B(2)PRM) | 8 | 0 |
| `refsmmat` | reference-to-stable-member matrix, row-major, DP elements, scale 2^1 (REFSMMAT, I(18D)PRM; 'REFSMMAT SCALE OF B1') | 6 | 0 |
| `rls` | landing site vector, moon-fixed reference, meters, scale 2^27 (RLS; R30 'SCALED AT (-27)M'); used by P20/P22/P52 lat-long, R30, P57, landing | 2 | 1 |
| `rn` | this vehicle's position, meters, scale 2^29 (earth) or 2^27 (moon) (RN, B(6)PRM) | 1 | 0 |
| `rrectcsm` | permanent CSM position, meters, scale 2^29 earth / 2^27 moon (RRECTCSM) | 1 | 0 |
| `tem` | PIPASR +3: read PIPAX..Z into TEMX..Z and the high words of DELV, zero the PIPAs (no PIPTIME1). TEMX..TEMZ (axis 0..2): those counts, SP. | 2 | 1 |
| `tevent` | time of event for downlist, centiseconds, scale 2^28 (TEVENT; TLIFTOFF in P11) | 0 | 1 |
| `tgo` | time to go in the burn / guidance, centiseconds, scale 2^28 (TGO) | 0 | 1 |
| `thetad` | desired gimbal angles THETAD +0..+2 (CPHI/CTHETA/CPSI), raw 15-bit CDU-style words, 2^15 = 1 revolution Index 0..2. | 0 | 1 |
| `thrust` | THRUST (LM 055): descent engine throttle command pulses, SP. | 1 | 2 |
| `tig` | time of ignition, centiseconds, scale 2^28 (TIG; DSP noun 33) | 2 | 1 |
| `tland` | TLAND (LM, = W): nominal time of landing, cs 2^28. | 2 | 0 |
| `trkmkcnt` | TRKMKCNT: rendezvous tracking mark count (noun 45 R1), whole number. | 0 | 1 |
| `ttogo` | time to go, centiseconds, scale 2^28 (TTOGO; DSP nouns 35,40,45,59,99) | 1 | 1 |
| `vrectcsm` | permanent CSM velocity, m/cs, scale 2^7 earth / 2^5 moon (VRECTCSM) | 1 | 0 |
| `zoomtime` | P40TMP (CSM, DP cs), SAVET-30 (LM, DP cs), ZOOMTIME (LM pad load, cs), S61DT and CM/GYMDT (CSM entry, cs): read by the restart tables. | 3 | 0 |

### AGC labels -> C functions

The "AGC" column above names the label each listed function translates; the complete label -> function -> file:line index for the lane (and the tree) is [docs/overview/xref.md](../overview/xref.md), generated by `tools/xref.py`.
