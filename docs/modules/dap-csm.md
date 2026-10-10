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

C target: src/csm/dap/, public header include/agc/csm/dap.h.

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
| PITCHDAP/YAWDAP/PCDUDOTS/RLIMTEST/ERRORLIM/ACTLIM/FWDFLTR/OPTVARK/PRECOMP/PCOPY/YCOPY/DAPINIT (963-977) | CDUY/Z, SIN/COSCDU (2), OMEGAYC/ZC, N10.. | TVCPITCH/TVCYAW (85.41 arcsec/bit), PCMD, YCMD, filter nodes | agc_task_pitchdap, agc_task_yawdap, agc_task_dapinit, csm_tvc_pcopy, csm_tvc_ycopy; static tvcd_fwdfltr, tvcd_precomp (daps.c) |
| STRKTSTI/TSTINIT/HACK/HACKWLST (981-983) | ESTROKER, FCARD tables | TVCPITCH bursts | agc_job_strktsti, csm_tvc_hack, agc_task_hackwlst (stroke.c) |
| ROLLDAP/NOROLL1 (989-997) | OGANOW/OGAPAST/OGAERR (rev), 1/CONACC | CHAN6 roll jets, TIME6 | agc_task_rolldap, agc_task_noroll1 (rolldap.c) |

## Progress
- DONE include/agc/csm/dap.h (resume: thetad/rho/ogad/oganow/pcdu*pst now raw uint16_t words, tempdap[2], errbinit[2], cphix[3], setmaxdb/setmindb/rcsdapon_t5/rcsup), src/csm/dap/pending.h, src/csm/dap/internal.h, state.c (paths after the header-layout change; include lines fixed in include/agc/csm/dap.h)
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
- DONE TVCDAPS daps.c (agc_task_pitchdap, agc_task_yawdap, agc_task_dapinit, csm_tvc_pcopy, csm_tvc_ycopy; ERRORLIM ACTLIM FWDFLTR OPTVARK PRECOMP 2/3CASFLTR DELBARP/Y) re-verified vs source on resume
- DONE TVCSTROKETEST stroke.c (agc_job_strktsti, csm_tvc_tstinit, csm_tvc_hack, agc_task_hackwlst) re-verified
- DONE TVCROLLDAP rolldap.c (agc_task_rolldap, agc_task_noroll1; all regions, ROLLSET, MOREROLL/NEWROLL/FIREPLUG, +-TORQUE) re-verified, constants checked
- DONE tests/csm/dap.c: 791 checks, 0 failures (see Tests written)
- DONE check_rules: 0 hard; density >= 2 in daps.c, rolldap.c, stroke.c; the earlier RCS/TVC files still warn (1.1-1.7)

## Deviations
- daps.c (whole file): DP filter quantities are agc_dp_t (sign-agreed). Where the AGC reads or overwrites only the upper word of a DP pair (ERRORLIM, ACTLIM, PCMD/YCMD, AK1/AK2), C can differ by one unit of the low word carried into the upper word.
- daps.c ACTLIM: the rounding (DOUBLE of the low word overflowing into TS L) is used only for the limit test, as in the AGC; the unrounded upper word is the output.
- rolldap.c ROLLSET: the AGC doubles twice in A (one overflow bit) and the TS limits on overflow. C computes the exact product x4 and limits it with its true sign.
- rolldap.c: register I (a CA or CS instruction word built for INDEX I; 0 X) is held as +1 / -1 / 0 (0 = MOREROLL).
- stroke.c: after STROKILL the pending HACKWLST still runs once. The AGC's CARD/REVS bookkeeping on that pass (INCR CARD from -0) is not reproduced bit for bit; STROKER stays +0 and nothing else is affected.
- restarts.c CMDSOUT: the AGC writes -0 + PCMD to avoid a +0 command word. -0 is not represented.
- mysubs.c DV: overflow (|dividend| >= |divisor|) is limited to +-POSMAX. The AGC result is undefined.
- mysubs.c DPADD: the ones' complement +-0 handling in the "upper = 0" branches has the same value but is not modelled bit for bit.
- DAS/DDOUBL (mysubs.c) wrap on overflow like the AGC's overflow-corrected DAS. A large filter memory can therefore wrap DAP1 instead of saturating (see Lessons).

## Stubs
None. Every routine in scope is translated.

## Requested core additions
- Erasable accessors (declared in src/csm/dap/pending.h until published; guid-powered asks for the same setters in src/csm/burns/guid_pending.h):
  `agc_sp_t agc_erase_tvcpitch(void); void agc_erase_set_tvcpitch(agc_sp_t v);` (TVCPITCH = CDUSCMD 054)
  `agc_sp_t agc_erase_tvcyaw(void);   void agc_erase_set_tvcyaw(agc_sp_t v);`   (TVCYAW = CDUTCMD 053), 85.41 arcsec/bit.
  The library has no definitions yet, so any binary that links the TVC DAP fails to link without them.
- TIME5/TIME6 raw-word access: now provided by agc_rupt_time / agc_rupt_set_time (exec.h). Resolved.

## Requested jobs-tasks
All in src/csm/dap/pending.h (provisional 0x0D00 range; REDORCS, REDOTVC, TVCEXEC match exec_pending.h): T5 AGC_TASK_RCSATT, JETSLECT, TVCDAPON, TVCINIT1, DAPINIT, PITCHDAP, YAWDAP, ENABL2, CMDSOUT, RCSUP, REDORCS, REDOTVC; T6 AGC_TASK_T6START, NOROLL1; waitlist AGC_TASK_ROLLDAP, HACKWLST, EXRSTRT, TVCEXEC; jobs AGC_JOB_AMBGUPDT (PRIO34), AGC_JOB_STRKTSTI (V68). Dispatch cases are needed for each.

## Tests written
tests/csm/dap.c (791 checks, 0 failures; linked with scratch stubs for the missing symbols of other lanes):
1. Jet selection: +pitch with all quads good gives channel 5 bits 1 and 3 (PYTABLE masked by PJETS, OCT 1005) for TAU/2. Also checked: -pitch, quad A failed (one jet, full TAU), the 0.1 s limit with DFTMAX, and +X translation.
2. TVC pitch DAP step response, CSM/LM: a positive error gives a negative PCMD (sign change in OPTVARK) and a negative error gives a positive one. TVCPHASE ends at 2 and T5 is re-armed. With filter memory in TMP3, PCMD is limited to ACTSAT = -253. The earlier failure (line 144) was the test, not the TVC erasables: it put 16000 into TMP1, which overflows DAP1 (DAS wraps, as in the AGC), so the command came out at -107. setup now clears PTMP and saturates through TMP3.
3. Stroke test: set 1 is 80 bursts (10 in 1/2 amplitude, 4 reversals) and returns to zero. Set 2 is then loaded (REVS 5, N -6). After 364 bursts STROKER is +0. These checks do not depend on the TVCPITCH accessor.
4. Roll DAP: a +10 deg OGA error at rest is region 6 and fires -TORQUE jets 12/10 (chan 6 OCT 12). NOROLL1 clears the jets. Inside the deadband the DAP coasts.
5. MASSPROP: CSM alone, IXX from VARST0/VARST5.

## Open questions
- Names with label digits remain in the DONE files (agc_task_noroll1, agc_task_tvcinit1, agc_task_enabl2, csm_tvc_init4, csm_rcs_phase1, csm_rcs_t5phase2, csm_rcs_t6progm, agc_csm_dap_rcsdapon_t5). The naming phase-2 map did not rename them. T5/T6 are hardware timer names, but NAMING.md rule 2 may want the others spelled out. I left them for the naming owner, because renaming touches pending.h ids and other lanes.
- The header-layout change left old-style includes ("agc_status.h" etc.) in include/agc/core/*.h, exec/exec.h, guidance, imu, io/nouns.h, nav/*, lm/*, csm/entry.h, programs/rendezvous.h. Nothing compiles until those are fixed. I fixed csm/dap.h (mine) and verified the lane with a scratch include shim.
- FWDFLTR tests DAPDATR1 bit 14 (LM on) and PRECOMP tests bit 13 (LM off) exactly as the AGC does. A config with neither bit set therefore runs the third cascade's precompute without its output.

## Lessons
- DAS/DDOUBL overflow wraps (overflow-corrected), so filter states near 1.0 wrap instead of saturating. To test a limiter, drive it through a stage with headroom, not by putting a huge value in the first node.
- After DAS, A holds the overflow indicator and L is +0. "DXCH TMPn" right after a DAS therefore stores (overflow, 0) into TMPn. That is how the TVC filter clears its node and makes PRECOMP's rewrite necessary.
- ACTLIM rounds only for the limit test. Read which register a TS actually writes before assuming a rounded value is stored.
- INDEX I; 0 X with I = CA/CS instruction words is a runtime sign selector. An int +1/-1 (0 for a "same jets" switch) translates it cleanly.
- TS L with overflow skips the next instruction. Look for the "TS x; TCF +3; INDEX A; CS LIMITS" overflow-protect idiom (ROLLSET) and the TS L skip in ACTLIM.
- When the build is broken by other lanes, link the test against an archive of whatever compiles plus generated zero-return stubs (scratch only). This tells your own failures apart from link noise.
