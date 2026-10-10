# guid-landing -- Luminary landing, aborts, ascent, throttle, FINDCDUW

## Scope

Luminary 099 only. AGC sources (pages are the Luminary 1A listing pages):

| AGC file                               | pages    | C file                         |
|----------------------------------------|----------|--------------------------------|
| THE_LUNAR_LANDING.agc                  | 785-792  | src/lm/lunar_landing.c         |
| THROTTLE_CONTROL_ROUTINES.agc          | 793-797  | src/lm/throttle.c              |
| LUNAR_LANDING_GUIDANCE_EQUATIONS.agc   | 798-828  | src/lm/landing_guidance.c (equations) + src/lm/lunar_landing.c (R13, P64/P66/P67 entry, redesignation, ROD) |
| P70-P71.agc                            | 829-837  | src/lm/landing_aborts.c               |
| P12.agc                                | 838-842  | src/lm/ascent.c                |
| ASCENT_GUIDANCE.agc                    | 843-856  | src/lm/ascent.c                |
| LANDING_ANALOG_DISPLAYS.agc            | 898-907  | src/lm/landing_displays.c      |
| FINDCDUW--GUIDAP_INTERFACE.agc         | 908-925  | src/lm/findcduw.c              |

Shared lane state (pad loads and erasables used by more than one of the
files above) lives in src/lm/landing_state.c behind accessors declared in
include/agc/agc_lm_landing.h. Symbols owned by other lanes that do not exist
yet are declared in ONE place, src/lm/landing_pending.h (see "Requested core
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

### Landing guidance (landing_guidance.c)

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

### Aborts (landing_aborts.c), ascent (ascent.c), R10 (landing_displays.c), FINDCDUW (findcduw.c)

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

## Progress

Infrastructure:
- DONE src/lm/landing_pending.h (all not-yet-existing shared symbols, one place)
- DONE include/agc/agc_lm_landing.h (lane API + state structs; may grow)
- DONE src/lm/landing_state.c (state, PHASCHNG/FASTCHNG/DV/MP helpers)
- DONE landing_pending.h reconciled 2026-10-09 with agc_guid.h, agc_lm_dap.h, agc_integ.h, agc_exec.h, exec_pending.h (job ids moved to 0x0F00/0x0F80)

Per file (TODO unless marked):
- DONE throttle.c: agc_lm_throttle, agc_lm_throttle_measured (AFDUMP..DOPIF), thr_region (WHERETO..FLATOUT2), thr_doit, thr_fweight, agc_lm_throttle_flatout, agc_lm_throttle_whatout, agc_lm_massmult
- DONE findcduw.c: agc_lm_initcduw, agc_lm_findcduw (+ _m2 = FINDCDUW -2), fc_fltrsub, fc_unwctest, fc_dcmcl, fc_nb2cdusp, fc_dvbycosm, agc_lm_arctrgsp, agc_lm_sparcsin, agc_lm_limitsub, agc_lm_onesto2s, fc_noattcnt, ALARMMGA in fc_gimbal_changes
- DONE landing_guidance.c: agc_lm_job_lunland, agc_lm_guidpass (+ldg_front = GUILDRET+2..BRSPOT2, ldg_quadratic), agc_lm_ttfincr, agc_lm_calcrgvg, agc_lm_rgvgcalc, agc_lm_ttf8cl, agc_lm_rootpsrs, agc_lm_quadguid (+ldg_quadcoef), agc_lm_afccalc (+ldg_afclimit), agc_lm_cgcalc, ldg_extlogic, ldg_exgsub, EXBRAK, ldg_exnorm, agc_lm_steer_exit (STEER?/EXVERT/RATESTOP/GDUMP1), agc_lm_dispexit, agc_lm_job_braking_phase_display, agc_lm_job_approach_phase_display, agc_lm_job_approach_phase_proceed, agc_lm_vertguid (P65VERT), agc_lm_start_approach_phase, agc_lm_start_velocity_nulling, agc_lm_tdispset
- DONE lunar_landing.c: agc_lm_job_braking_phase (+FLAGORGY), ll_ignalg_start/ll_ignalg_prec (IGNALG, LEMPREC wait job IGNALGP), ll_ignaloop/ll_ign_guidance (IGNALOOP, INTSTALL wait job IGNALOOP), ll_ddum (DDUMCALC), ll_ddumgood (DDUMGOOD/R60INIT), agc_lm_job_ignalgrt (IGNALGRT/ASTNCLOK), agc_lm_job_astnret, agc_lm_job_braking_phase_burn_attitude/3/4, agc_lm_guildenstern (R13 incl. STARTP66/STRTP66A/VRTSTART/STARTP67/STABL?/P66NOW?/RESTART?), agc_lm_redesig (+ll_redes1), agc_lm_rate_of_descent_vertical, agc_lm_task_rodtask, agc_lm_job_rodcomp (ll_rod_pipas/ll_rod_state/ll_rod_afc), ll_vhorcomp, agc_lm_manual_landing_vertical, agc_lm_pitfall, agc_lm_task_redesmon (+COUNT'EM, RESETRPT), agc_lm_descbits, agc_lm_job_landing_confirmation / _landing_confirmation_flash / _landing_confirmation_proceed (LANDJUNK)
- DONE landing_aborts.c: agc_lm_task_abort_monitor, agc_lm_abort_monitor_resume (FLASHH?/FLASHV?/10,11/P71NOW?/P70NOW?), ab_abrtjask (CONTABRT/ABRTJASK), ab_legal (LEGAL?/ABORTALM), agc_lm_job_dps_abort/p71/p70a/p71a, agc_lm_job_goabort, ab_dps_abort_init, ab_bothpoly (BOTHPOLY..YOK), ab_upthrot, agc_lm_job_abort_auto_control_check (UPTHROT1 -3), agc_lm_job_upthrot1 (+GRP4OFF), ab_aps_abort_return, agc_lm_job_aps_abort_oldtime (OLDTIME), agc_lm_tgocomp, agc_lm_throtup
- DONE ascent.c: agc_lm_guidinit, agc_lm_comminit, agc_lm_powered_ascent_init, agc_lm_ycomp, agc_lm_logsub, as_vgvect (ASCENT state/GEFF/VGVECT/VGBODY), as_maineng, as_t2test (T2TEST/LOGSUB/NORATES/CONST), as_rates (RATES/CHKBMAG/PROK), as_cmponent (CMPONENT/NO-ATP/AIMER), as_mainline (MAINLINE/ANG1CHEK/OFFROT/CLRXFLAG), as_exitvr, as_checkalt (CHECKALT/CHECKYAW/KEEPVR/EXITVR1), as_keepvr1, as_ascterm/as_ascterm1 (ASCTERM..4), as_engoff, agc_lm_ascent, agc_lm_job_atmag, agc_lm_task_engoff1, agc_lm_job_cutoff/_cutoff5/_cutoff1/_cutoff2/_termasc, agc_lm_job_powered_ascent/_powered_ascent_flash/_powered_ascent_proceed/_powered_ascent_at_ignition/_newload/_newload2, agc_lm_powered_ascent_return, agc_lm_job_powered_ascent_burn
- DONE landing_displays.c: agc_lm_landisp (LANDISP/ALTROUT/ALTOUT dispatch), ld_disindat (DISINDAT), agc_lm_task_intlze (INTLZE), ld_speedrun (SPEEDRUN to VARDELAY), agc_lm_task_speedrun2 (SPEEDRUN after the pause, GET22/32, LATFWDV), agc_lm_vmonitor + ld_vmon_delta (VMONITOR..ZEROLSTY, checked branch by branch against pp. 904-907), ld_altrout (ARCOMP), ld_altout/ld_extrapolate (OLDDATA/NEWDATA/ZERODATA), ld_dataout (DATAOUT), ld_disprset (DISPRSET/ABORTON)
- DONE tests/test_landing.c (compiles; see Tests written)
- DONE 2026-10-10 reconciliation after wave 1: ASTNRET now sets agc_align_ram()->qmaj and calls agc_job_lm_fine_align_recheck (agc_align.h R51K/R51P63) instead of a private agc_align_fine_align_recheck / agc_imu_fine_align_recheck; 1JACCQ/1JACCR taken from agc_lm_dap.h, TRKMKCNT from agc_programs.h; agc_imu.h signatures used as declared (agc_imu_status_check, agc_imu_attitude_maneuver, agc_imu_set_pointvsm/_scaxis, agc_imu_comp_get); all displays go through agc_dsky_display()

## Deviations

- FASTCHNG (landing_state.c): group 3 re-armed at the LUNLAND phase; a restart repeats the whole guidance pass (C cannot restart mid-function).
- THROTTLE (throttle.c): -0 values (PIFPSET initial, WHATOUT FCOLD/PIF) modelled as +0; MASSMULT overflow and FLATOUT1 low words returned as 0 (AGC leaves stale L).
- landing_guidance.c QUADGUID coefficients: stored SP over stale low words in the AGC, here as clean DP (low word 0).
- IGNALG (lunar_landing.c): loop bounded; on exhaustion alarm 01412 and NIGNLOOP left 0; returns AGC_STATUS_NO_CONVERGE.
- FINDCDUW (findcduw.c): DV's L after LIMITSUB modelled as A.
- R10 (landing_displays.c): LANDISP's "LXCH PIPCTR1 / DXCH TBASE2" done via agc_phase_newphase(2, current phase, set_tbase) (no TBASE2 setter in agc_exec.h); VMONITOR's AD NEG0 (+0 -> -0 for the DINC hardware) omitted; DDOUBL overflow kept wrapped; LADQSAVE return address modelled as a static enum (ALTROUT +1 / ALTOUT +1) resumed by task SPEEDRN2 after the 40 ms VARDELAY.

## Stubs

None in this lane. Calls into other lanes that may still return AGC_STATUS_UNIMPLEMENTED are tolerated by assertion (REFMF, R60/R51 continuation paths).

## Tests written

tests/test_landing.c (LM). Compiles with the project flags + -DAGC_HOST_DEBUG.
1. Throttle law in the 10%-60% band: MASS 8000 kg, commanded 2.0 m/s^2 (FC 1276.7 pulses), measured 1.5 m/s^2 (FP 957.6) -> 319 pulses to THRUST (PIFPSET 0); above HIGHCRIT -> FCODD = FMAXPOS, PIFPSET = FEXTRA.
2. QUADGUID with the vehicle exactly on r = RDG + VDG T + ADG T^2/2 (T = -400 s, LEADTIME -1 cs) -> ACG = ADG to 1e-4 relative (zero correction); 10 m x offset -> correction -12 dx/T^2 = -0.00075 m/s^2 exactly as expected.
Link status: libagc does not link yet (other lanes' pending symbols: radar, DSKY ecadr, exec wake_as, etc.), so `make test` cannot build it until integration. Verified in scratch by linking with --unresolved-symbols=ignore-all: 17 checks, 0 failures. The test defines agc_erase_set_thrust itself under LANDING_TEST_FAKE_THRUST (set to 0 once state-flags provides it).

## Open questions

- THRUST / ALTM / CDUTCMD / CDUSCMD counters: state-flags should own accessors (declared provisionally below); ALTM takes a raw 15-bit word (bit 15 = output sign/high bit), not an agc_sp_t.
- TBASE2 setter for LANDISP (exec-sched), if exact TBASE2 = -TIME1 semantics are wanted.

## Requested core additions

Exact list = src/lm/landing_pending.h (re-checked 2026-10-10 against every include/agc header; everything else it used to hold now comes from real headers):
- state-flags: `void agc_erase_set_thrust(agc_sp_t pulses);` (THRUST 055), `void agc_erase_set_altm(uint16_t word);` (ALTM 060, raw word), `void agc_erase_set_cdutcmd(int which, agc_sp_t v);` (0 CDUTCMD 053, 1 CDUSCMD 054)
- guid-powered: `agc_sp_t agc_erase_tem(int axis); void agc_erase_set_tem(int axis, agc_sp_t v);` (TEMX/Y/Z)
- prog-align: `agc_status_t agc_align_refmf(agc_interp_t *ip);` (REFMF)
- nav-aux / io-nouns: `agc_status_t agc_navaux_set_latlong(const agc_latlong_t *ll);` (LAT/LONG/ALT for N43)
- prog-rendz: `agc_status_t agc_rendz_flip(uint16_t lamp_bit); agc_status_t agc_rendz_clradmod(void);`
- sys-misc / dap-lm: `agc_sp_t agc_erase_m22(void); agc_sp_t agc_erase_m32(void);` (also in src/lm/dap/lm_dap_pending.h)
- alarms 00401 00402 01406 01410 01412 as AGC_ALARM_* constants.

## Requested jobs-tasks

Provisional ids in landing_pending.h (jobs 0x0F01-0x0F25, tasks 0x0F81-0x0F86; GOABORT/P70A/P71A/R10,R11/ZOOM from exec_pending.h). Labels:
jobs P63LM IGNALGRT ASTNRET P63SPOT3 P63SPOT4 P64CEED P64DISPS RODCOMP
LANDJUNK LANDJUNK2 GOABORT P12LM P12LM2 NEWLOAD NEWLOAD2 CUTOFF CUTOFF1
CUTOFF2 TERMASC LUNLAND ATMAG; tasks RODTASK REDESMON ENGOFF1 ABORT_MONITOR INTLZE
SPEEDRUN2.
