# guid-landing -- Luminary landing, aborts, ascent, throttle, FINDCDUW

## Scope

Luminary 099 only. AGC sources (pages are the Luminary 1A listing pages):

| AGC file                               | pages    | C file                         |
|----------------------------------------|----------|--------------------------------|
| THE_LUNAR_LANDING.agc                  | 785-792  | src/lm/lunar_landing.c         |
| THROTTLE_CONTROL_ROUTINES.agc          | 793-797  | src/lm/throttle.c              |
| LUNAR_LANDING_GUIDANCE_EQUATIONS.agc   | 798-828  | src/lm/landing_guidance.c (equations) + src/lm/lunar_landing.c (R13, P64/P66/P67 entry, redesignation, ROD) |
| P70-P71.agc                            | 829-837  | src/lm/p70_p71.c               |
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
| P65VERT | ldg_p65vert | V2FG, TAUVERT |
| TDISPSET | agc_lm_tdispset | TTFDISP (2^28 cs), TREDES (s, SP integer) |
| 1406P00 / 1406ALM | in ldg_ttf8cl | alarm 01406 |
| FASTCHNG | agc_lm_fastchng | group 3 |

### Landing program flow (lunar_landing.c)

| AGC label | C function |
|---|---|
| P63LM, FLAGORGY | agc_lm_job_p63lm |
| IGNALG, IGNALOOP, ?GUIDSUB, GUIDSUB, EXGSUB loop | agc_lm_ignalg (bounded loop) |
| DDUMCALC, DDUMGOOD, R60INIT | ign_ddumcalc, ign_ddumgood |
| IGNALGRT/ASTNCLOK, ASTNRET/P63SPOT2..4 | agc_lm_job_ignalgrt, agc_lm_job_astnret, agc_lm_job_p63spot3 |
| GUILDENSTERN (R13) | agc_lm_guildenstern |
| STARTP66, STRTP66A, VRTSTART, STARTP67 | lnd_startp66 / lnd_strtp66a / lnd_vrtstart / lnd_startp67 |
| STARTP64, P65START | agc_lm_startp64, agc_lm_p65start |
| REDESIG | agc_lm_redesig |
| P64DISPS, P64CEED, RED-OVER, REDES-OK | agc_lm_p64disps, agc_lm_job_p64ceed |
| P66VERTA, RODTASK, RODCOMP, VHORCOMP | agc_lm_p66verta, agc_lm_task_rodtask, agc_lm_job_rodcomp |
| P67VERT | agc_lm_p67vert |
| PITFALL, REDESMON, COUNT'EM | agc_lm_pitfall, agc_lm_task_redesmon |
| DESCBITS | agc_lm_descbits |
| LANDJUNK (P68) | agc_lm_job_p68, agc_lm_job_p68b |

### Aborts (p70_p71.c), ascent (ascent.c), R10 (landing_displays.c), FINDCDUW (findcduw.c)

| AGC label | C function |
|---|---|
| R10,R11, R10,R11A, FLASHH?, 10,11, P71NOW?, P70NOW? | agc_lm_task_r10r11 |
| P70, P71, P70A, P71A, LEGAL?, ABORTALM | agc_lm_p70, agc_lm_p71, lm_legal |
| CONTABRT, ABRTJASK | agc_lm_abrtjask |
| GOABORT, P70INIT, BOTHPOLY, UPRATE, YOK, UPTHROT, UPTHROT1, P71RET, OLDTIME | agc_lm_job_goabort and helpers |
| TGOCOMP, THROTUP | agc_lm_tgocomp, agc_lm_throtup |
| P12LM..P12RET, P12INIT, COMMINIT, GUIDINIT | agc_lm_job_p12lm, agc_lm_job_p12newload, agc_lm_p12init, agc_lm_comminit, agc_lm_guidinit |
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
- DONE landing_guidance.c: agc_lm_job_lunland, agc_lm_guidpass (+ldg_front = GUILDRET+2..BRSPOT2, ldg_quadratic), agc_lm_ttfincr, agc_lm_calcrgvg, agc_lm_rgvgcalc, agc_lm_ttf8cl, agc_lm_rootpsrs, agc_lm_quadguid (+ldg_quadcoef), agc_lm_afccalc (+ldg_afclimit), agc_lm_cgcalc, ldg_extlogic, ldg_exgsub, EXBRAK, ldg_exnorm, agc_lm_steer_exit (STEER?/EXVERT/RATESTOP/GDUMP1), agc_lm_dispexit, agc_lm_job_p63disps_m2, agc_lm_job_p64disps, agc_lm_job_p64ceed, agc_lm_vertguid (P65VERT), agc_lm_startp64, agc_lm_p65start, agc_lm_tdispset
- DONE lunar_landing.c: agc_lm_job_p63lm (+FLAGORGY), ll_ignalg_start/ll_ignalg_prec (IGNALG, LEMPREC wait job IGNALGP), ll_ignaloop/ll_ign_guidance (IGNALOOP, INTSTALL wait job IGNALOOP), ll_ddum (DDUMCALC), ll_ddumgood (DDUMGOOD/R60INIT), agc_lm_job_ignalgrt (IGNALGRT/ASTNCLOK), agc_lm_job_astnret, agc_lm_job_p63spot2/3/4, agc_lm_guildenstern (R13 incl. STARTP66/STRTP66A/VRTSTART/STARTP67/STABL?/P66NOW?/RESTART?), agc_lm_redesig (+ll_redes1), agc_lm_p66verta, agc_lm_task_rodtask, agc_lm_job_rodcomp (ll_rod_pipas/ll_rod_state/ll_rod_afc), ll_vhorcomp, agc_lm_p67vert, agc_lm_pitfall, agc_lm_task_redesmon (+COUNT'EM, RESETRPT), agc_lm_descbits, agc_lm_job_p68 / _p68_flash / _p68b (LANDJUNK)
- DONE p70_p71.c: agc_lm_task_r10r11, agc_lm_r10r11a (FLASHH?/FLASHV?/10,11/P71NOW?/P70NOW?), ab_abrtjask (CONTABRT/ABRTJASK), ab_legal (LEGAL?/ABORTALM), agc_lm_job_p70/p71/p70a/p71a, agc_lm_job_goabort, ab_p70init, ab_bothpoly (BOTHPOLY..YOK), ab_upthrot, agc_lm_job_abtp40a (UPTHROT1 -3), agc_lm_job_upthrot1 (+GRP4OFF), ab_p71ret, agc_lm_job_p71old (OLDTIME), agc_lm_tgocomp, agc_lm_throtup
- DONE ascent.c: agc_lm_guidinit, agc_lm_comminit, agc_lm_p12init, agc_lm_ycomp, agc_lm_logsub, as_vgvect (ASCENT state/GEFF/VGVECT/VGBODY), as_maineng, as_t2test (T2TEST/LOGSUB/NORATES/CONST), as_rates (RATES/CHKBMAG/PROK), as_cmponent (CMPONENT/NO-ATP/AIMER), as_mainline (MAINLINE/ANG1CHEK/OFFROT/CLRXFLAG), as_exitvr, as_checkalt (CHECKALT/CHECKYAW/KEEPVR/EXITVR1), as_keepvr1, as_ascterm/as_ascterm1 (ASCTERM..4), as_engoff, agc_lm_ascent, agc_lm_job_atmag, agc_lm_task_engoff1, agc_lm_job_cutoff/_cutoff5/_cutoff1/_cutoff2/_termasc, agc_lm_job_p12lm/_p12flash/_p12lm2/_p12lmb/_newload/_newload2, agc_lm_p12ret, agc_lm_job_p12burn
- TODO landing_displays.c: LANDISP .. ABORTON, VMONITOR
- TODO tests/test_landing.c

## Requested core additions

All are declared (provisionally) in src/lm/landing_pending.h, grouped by
owning lane: LM flags (AGC_FLAG_*), channel bits (AGC_CH0nn_*), DAPBOOLS bits,
shared erasable accessors (agc_erase_*), exec (newmodex, gotopooh, enema),
DSKY (godspr, regodspr, goflash, goperf1, falton, reldsp, pinbrnch), nav
(rp_to_r, r_to_rp, lat_long, lemprec, intstall, integrvs), guid-powered
(mungrav, quictrig, smnb, cdu_nbsm, servicer avgexit/setpos1/lrhtask,
burnbaby, stclok2, enginof2, p40auto), dap-lm (stoprate, zatteror,
set/restore deadbands, pflitedb), imu/align (r02both, r60lem, r51, refmf).
Final exact list is that header.

## Requested jobs-tasks

Placeholder enums in landing_pending.h (lm_pending_job / lm_pending_task):
jobs P63LM IGNALGRT ASTNRET P63SPOT3 P63SPOT4 P64CEED P64DISPS RODCOMP
LANDJUNK LANDJUNK2 GOABORT P12LM P12LM2 NEWLOAD NEWLOAD2 CUTOFF CUTOFF1
CUTOFF2 TERMASC LUNLAND ATMAG; tasks RODTASK REDESMON ENGOFF1 R10_R11 INTLZE
SPEEDRUN2.
