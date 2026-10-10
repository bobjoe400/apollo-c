# dap-csm -- CSM RCS and TVC digital autopilots (Comanche055)

## Scope

Comanche055 files (pages): RCS-CSM_DIGITAL_AUTOPILOT (1002-1024),
AUTOMATIC_MANEUVERS (1025-1036; despite the name this is the RCS DAP
attitude-hold / phase-plane / jet-on-time logic, not KALCMANU),
RCS-CSM_DAP_EXECUTIVE_PROGRAMS (1037-1038, AMBGUPDT), JET_SELECTION_LOGIC
(1039-1062), MYSUBS (999-1000), TVCINITIALIZE (937-944), TVCEXECUTIVE
(945-950), TVCMASSPROP (951-955), TVCRESTARTS (956-960), TVCDAPS (961-978),
TVCSTROKETEST (979-983), TVCROLLDAP (984-998).

Also translated here because they only read/write DAP-private erasables
(lead to arbitrate with guid-powered, whose note lists them under sps_burn.c):
S41.2 (DAPDATR1/2 decode, P40-P47 p.731-733), S40.14 (RCS gains, p.735),
RCSDAPON (p.736). R03 itself (DAPDISP/DONOUN46-48, EXTENDED_VERBS
p.253-254) is display flow and stays with sys-misc; it calls our S41.2,
MASSPROP and S40.14.

C target: src/csm/dap/, public header include/agc/agc_csm_dap.h.

## Design

Interrupt model. T5LOC/T6LOC are task ids set with agc_rupt_set_t5_task /
agc_rupt_set_t6_task (agc_exec.h). TIME5/TIME6 have no API yet: requested
below (raw AGC counter words). Each T5/T6 entry label is a task body
`agc_task_<label>(void)`; waitlist tasks likewise; jobs `agc_job_<label>(ctx)`.

State. EBANK6 DAP erasables are module-private structs (state-flags table
marks them "module-private (dap-csm)"): agc_csm_rcs_t (RCS DAP), agc_csm_tvc_t
(TVC pitch/yaw/roll DAP, restarts, stroke test), agc_csm_dapcom_t (RCS/TVC
common storage: AK, RCSFLAGS, EDRIVE, IXX.., pad loads). Other lanes reach
them through agc_csm_dap_rcs()/tvc()/com() pointers (see header).
Shared erasables come from agc_state.h (CDUX/Y/Z raw, DAPDATR1/2, CSMMASS,
LEMMASS, IMODES33, THETAD) and flags from agc_flags_csm.h.

| AGC routine (page) | inputs (scale) | outputs (scale) | C function (file) |
|---|---|---|---|
| SMALLMP (999) | KMPAC DP, A SP | KMPAC DP | csm_dap_smallmp (mysubs.c) |
| DPADD (999-1000) | KMPAC, A,L DP angles /180 deg | KMPAC mod 360 deg | csm_dap_dpadd (mysubs.c) |
| REDORCS, RCSATT, SETT5, phase 1 (1002-1004) | CHAN31, T5PHASE, IMODES33 | T5 sequencing | agc_task_redorcs, agc_task_rcsatt (t5rupt.c) |
| RATEFILT/DRHOLOOP/ADOTLOOP (1004-1006) | CDUs (2^15=rev), AMGB (1), GAIN1/2, KMJ, DFT | DRHO (180 deg), ADOT (450 deg/s), MERROR (180 deg) | csm_rcs_ratefilt (t5rupt.c) |
| KMATRIX/DCDUINCR (1006-1008) | ATTSEC, CDUXD, DELCDUX (DP 180 deg) | THETAD (180 deg) | csm_rcs_kmatrix, csm_rcs_dcduincr |
| DACNDLS/FDAIDSP1/2/GETAKS/WRTN17/WRTN22 (1009, 1022-1023) | ERRORX.. (180 deg), N22/N17 | AK.. (180 deg) | csm_rcs_dacndls |
| NEEDLER/NEEDLES (1010-1012) | AK (180 deg), RCSFLAGS b2,b3, CHAN12 | CDUXCMD.., EDRIVE, CHAN14 | agc_dap_needler, csm_dap_needler (needles.c) |
| FRESHDAP/REDAP/ZEROT5/IMUAOK/ZEROJET (1014-1016) | DAPDATR | DAP state zeroed, T5/T6 set | csm_rcs_freshdap, csm_rcs_zerojet |
| T5PHASE2/KALUPDT (1016, 1021) | ATTKALMN | T5 timing | csm_rcs_t5phase2 |
| manual rotation, STICKCHK, SETWBODY, MERUPDAT, FREEFUNC (1017-1021) | CHAN31 (RHC), RATEINDX | WBODY (450 deg/s), MERROR, ERROR, TAU | csm_rcs_manual (t5rupt.c) |
| AHFNOROT/FREECONT/ATTHOLD/HOLDFUNC/GRABANG (1025-1028) | CHAN31/32, HOLDFLAG, THETAD, BIAS | ERROR (180 deg) | csm_rcs_ahfnorot (attitude.c) |
| JETS/J6..J24/JTIME (1029-1036) | ADOT, WBODY (450 deg/s DP), ERROR, ADB, SLOPE, J/M | TAU (1 bit = 1/1600 s) | csm_rcs_jets, csm_rcs_switch_axis |
| AMBGUPDT (1037-1038) | CDUX, CDUZ | AMGB1,4,5,7,8 (scale 1) | agc_job_ambgupdt (exec.c) |
| JETSLECT .. T6SETUP (1039-1060) | CHAN31 THC, TAU, RACFAIL, RBDFAIL, ACORBD, XTRANS | RWORD/PWORD/YWORD 1,2 (chan 6/5 bits), BLAST (1/1600 s), DFT | agc_csm_jet_select (pure), agc_task_jetslect (jet_select.c) |
| T6START/T6RUPTOR/REPLACE*/ZBLAST* (1061-1062) | BLAST, words | CHAN5, CHAN6, TIME6 | agc_task_t6start (jet_select.c) |
| S41.2, S40.14, RCSDAPON (P40-P47 731-736) | DAPDATR1/2, IXX, IAVG | RATEINDX, ADB, XTRANS, ACORBD, RACFAIL, RBDFAIL, J/M, KMJ | agc_csm_dap_decode_dap_data, agc_csm_dap_rcs_gains, agc_csm_dap_rcsdapon (dapdata.c) |
| TVCDAPON/MRCLEAN/TVCINIT1-4/ATTINIT/NEEDLEIN (937-941) | pad loads, DAPDATR1, CDUX | TVC state, T5, waitlist | agc_task_tvcdapon, agc_task_tvcinit1, csm_tvc_mrclean, csm_tvc_init4 (init.c) |
| PRESWTCH/SWICHOVR/MCLEANJR/LOADCOEF/S40.15 (941-944) | IXX (2^20 kg m^2), KTLX/I, IAVG/TLX | 1/CONACC (2^9 s^2/rev), VARK, N10.. | agc_csm_tvc_preswtch, csm_tvc_swichovr, csm_tvc_loadcoef, csm_tvc_gains |
| TVCEXEC .. TVCEXFIN (946-949) | CDUX, OGAD, engine-on, VCNTR, CNTR, REPFRAC | OGAERR, AK, CSMMASS, PACTOFF/YACTOFF | agc_task_tvcexec, csm_tvc_exec_from (exec.c) |
| MASSPROP/FIXCW (951-955) | CSMMASS, LEMMASS (2^16 kg), DAPDATR1 | IXX, IAVG (2^20 kg m^2), IAVG/TLX (2^2 s^2), WEIGHT/G | agc_csm_dap_massprop, agc_csm_dap_fixcw (massprop.c) |
| REDOTVC/ENABL1/ENABL2/CMDSOUT/PHSCHK2/CHKSTRK/FINCOPY/TRIM/CMD/EXRSTRT (958-960) | TVCPHASE, TVCEXPHS, STROKER | restart sequencing | agc_task_redotvc, agc_task_enabl2, agc_task_cmdsout, agc_task_exrstrt (restarts.c) |
| PITCHDAP/YAWDAP/PCDUDOTS/RLIMTEST/ERRORLIM/ACTLIM/FWDFLTR/OPTVARK/PRECOMP/PCOPY/YCOPY/DAPINIT (963-977) | CDUY/Z, SIN/COSCDU (2), OMEGAYC/ZC, N10.. | TVCPITCH/TVCYAW (85.41 arcsec/bit), PCMD, YCMD, filter nodes | agc_task_pitchdap, agc_task_yawdap, agc_task_dapinit, csm_tvc_fwdfltr, csm_tvc_precomp (daps.c) |
| STRKTSTI/TSTINIT/HACK/HACKWLST (981-983) | ESTROKER, FCARD tables | TVCPITCH bursts | agc_job_strktsti, csm_tvc_hack, agc_task_hackwlst (stroke.c) |
| ROLLDAP/NOROLL1 (989-997) | OGANOW/OGAPAST/OGAERR (rev), 1/CONACC | CHAN6 roll jets, TIME6 | agc_task_rolldap, agc_task_noroll1 (rolldap.c) |

## Progress
- DONE include/agc/agc_csm_dap.h (resume: thetad/rho/ogad/oganow/pcdu*pst now raw uint16_t words, tempdap[2], errbinit[2], cphix[3], setmaxdb/setmindb/rcsdapon_t5/rcsup), include/agc/agc_csm_dap_pending.h, csm_dap_internal.h, state.c
- NOTE constants verified against tools/oracle/ropes/Comanche055.bin (scratchpad rope.py reads banked fixed words via agc_probe whatis)
- DONE MYSUBS csm_dap_smallmp, csm_dap_dpadd + word helpers (MSU, ADS, DAS, MP, DV) mysubs.c
- DONE RCS-CSM_DIGITAL_AUTOPILOT: t5rupt.c (REDORCS RCSATT SETT5 FREECHK REINIT FRESHDAP/REDAP ZEROJET T5PHASE2/KALUPDT), filter.c (RATEFILT KMATRIX DCDUINCR DACNDLS FDAIDSP1/2 GETAKS), manual.c (manual rotation, SETWBODY, FREEFUNC, STICKCHK), needles.c (NEEDLER..DACLOOP, agc_dap_needler)
- DONE AUTOMATIC_MANEUVERS attitude.c (AHFNOROT FREECONT ATTHOLD HOLDFUNC GRABANG JETS J6..J24 JTIME T6PROG T6PROGM)
- DONE AMBGUPDT agc_job_ambgupdt exec.c
- DONE JET_SELECTION_LOGIC agc_csm_jet_select, agc_task_jetslect, agc_task_t6start jet_select.c (re-verified vs source on resume)
- DONE SETMINDB/SETMAXDB/S41.2/S40.14/RCSDAPON/RCSUP dapdata.c
- DONE TVCINITIALIZE init.c (agc_task_tvcdapon, csm_tvc_mrclean, agc_task_tvcinit1, csm_tvc_init4, csm_tvc_swichovr_body, agc_csm_tvc_preswtch, csm_tvc_gains, LOADCOEF)
- DONE TVCEXECUTIVE exec.c (agc_task_tvcexec, csm_tvc_exec_restart; step machine over the TVCEXPHS restart points)
- DONE TVCMASSPROP massprop.c (agc_csm_dap_massprop, agc_csm_dap_fixcw)
- DONE TVCRESTARTS restarts.c (agc_task_redotvc, agc_task_enabl2, agc_task_cmdsout, agc_task_exrstrt)
- TODO TVCDAPS daps.c
- TODO TVCSTROKETEST stroke.c
- TODO TVCROLLDAP rolldap.c
- TODO tests/test_csm_dap.c

## Deviations
## Stubs
## Requested core additions
## Requested jobs-tasks
## Tests written
## Open questions
