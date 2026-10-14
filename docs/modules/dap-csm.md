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
- (resolved by review-dap-csm, 2026-10-11) The TVC filter DP erasables and the RCS DRHO, ADOT, MERRORX, EDOT, KMPAC are agc_dp_words_t (the words as stored; DAS without sign agreement through csm_dap_words_das / agc_dp_words_das): oracle finding F9 resolved, PITCHDAP/RATEFILT/JETS bit-exact.
- daps.c ACTLIM: the rounding (DOUBLE of the low word overflowing into TS L) is used only for the limit test, as in the AGC; the unrounded upper word is the output.
- (resolved by review-dap-csm) rolldap.c ROLLSET: the 16-bit accumulator wrap of the second DDOUBL and the opposite-sign overflow limit are modelled (oracle roll_wrap, roll_wrap_neg).
- rolldap.c RATENUM: a DV by TEMREG = 0 is taken as the overflow (MINLIMAP) with the numerator's sign; the AGC's TEMREG is then usually -0 and the sign follows -0.
- jet_select.c/attitude.c: SIGNCK1..5 test a word pair with CCS on the upper then the lower word; a pair whose lower word is -0 counts as + here.
- massprop.c DXTEST: the AGC's DXCH VARST0 +8D leaves junk (A, L) in VARST8, VARST9; nothing reads them, the C keeps the computed values.
- needles.c: CDUXCMD is written through the imu lane's agc_imu_hw_set_cducmd, which refuses counts beyond +-192; a full needle swing (up to +-768) is then not written (request to imu).
- rolldap.c: register I (a CA or CS instruction word built for INDEX I; 0 X) is held as +1 / -1 / 0 (0 = MOREROLL).
- stroke.c: after STROKILL the pending HACKWLST still runs once. The AGC's CARD/REVS bookkeeping on that pass (INCR CARD from -0) is not reproduced bit for bit; STROKER stays +0 and nothing else is affected.
- restarts.c CMDSOUT: the AGC writes -0 + PCMD to avoid a +0 command word. -0 is not represented.
- mysubs.c DV: overflow (|dividend| >= |divisor|) is limited to +-POSMAX. The AGC result is undefined.
- mysubs.c DPADD: the ones' complement +-0 handling in the "upper = 0" branches has the same value but is not modelled bit for bit; a sum of exactly +-180 deg gives POSMAX, POSMAX as in the AGC (review-dap-csm).
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

## Program flow and decisions

_Documentation wave (2026-10-12); prose and tables brought to the final tree (HEAD 3b97dc8) by the final pass (2026-10-14). The prose was written by reading the code; the two tables were extracted from the function bodies (flag, alarm, channel and scheduling calls; `agc_erase_*` accessors) and list only what the code does. Files: `src/csm/dap/*.c`, `src/csm/dap/*/*.c`._

The CSM RCS and TVC autopilots, both T5 programs (only one owns T5 at a
time; FLAGWRD6 bits 15-14 = DAPBIT1/DAPBIT2 say which: 01 RCS, 10 TVC,
11 Saturn takeover, 00 none). Basic code on stored words
(`src/csm/dap/mysubs.c`: AD/ADS/DAS with overflow correction, MSU on
counter words, MP upper/lower words).

* **RCS T5 cycle** (`src/csm/dap/rcs/t5rupt.c`): RCSATT
  (`agc_task_rcsatt`) runs the DAP only when the G/C autopilot is fully
  enabled (channel 31 bit 15 = 0: IMU powered and the S/C control switch
  in CMC); otherwise the jets are zeroed (`agc_csm_dap_zerojet`) and T5
  idles. Phase 1 every 100 ms (`csm_rcs_rate_filter_phase` ->
  `filter.c`): rate filter RATEFILT, the 1-s AMGB request (`csm_rcs_kmatrix`
  NOVACs AMBGUPDT, `exec.c:agc_job_ambgupdt`, PRIO34), automatic steering
  increments, the FDAI needles (NEEDLFLG / ATTITUDE_ERROR_DESIRED_ANGLES
  select which errors, `needles.c:csm_dap_needler`, EDRIVE limited to
  +-384 counts). Phase 2 20 ms later: RHC out of detent (channel 31 bits
  1-6) -> manual rate (`manual.c:csm_rcs_manual`: RATEINDX 0.05 / 0.2 /
  0.5 / 2 deg/s, STIKFLAG); otherwise `attitude.c:csm_rcs_ahfnorot`:
  channel 31 FREE -> free drift, ATTHOLD -> attitude hold, else automatic
  steering (biases added only during KALCMANU maneuvers). `csm_rcs_jets`
  locates (rate error, attitude error) per axis in the phase plane (dead
  zone ADB, switch lines 0.3-0.5 deg/s, slopes 0.6 / 0.8) and computes the
  one-jet on-time (1/1600 s bits). Phase 3 (`jet_select.c:agc_task_jetslect`):
  translation and rotation jets per quad (pitch/yaw on channel 5, roll on
  channel 6; failed quads from DAPDATR2), T6 started 14 ms later
  (`agc_task_t6start`: channel 13 ENBT6RUPT, replace or stop jets as each
  on-time ends).
* **DAP data** (`dapdata.c`): S41.2 (`agc_csm_dap_decode_dap_data`)
  decodes DAPDATR1/2 (configuration CSM / CSM-LM / ascent-only, quads,
  deadband 0.5 or 5 deg, rate) into ATTCHFLG / NJETSFLG and the gains;
  RCSDAPON (`agc_csm_dap_rcsdapon_t5`) sets DAPBIT1/2 = 01 and starts T5;
  a negative inertia or a bad TIME5 word is BAD_ARG.
* **TVC** (`src/csm/dap/tvc/`): TVCDAPON / MRCLEAN (`init.c`) zero the
  filter state and run TVCINIT1-4 by T5 (gains, mass properties
  `massprop.c`, trims to trackers, CSM-alone ATTINIT limits the RCS errors
  to 1.5 deg), then TVCEXEC by waitlist. Pitch and yaw DAPs (`daps.c`)
  alternate as self-perpetuating T5 tasks: body-axis rate from OMEGAYC/ZC
  of S40.8 (or hold), error integrated and limited to 45 deg, sixth-order
  filter (two cascades CSM, three CSM/LM; DAPDATR1 bit 14), trim offset,
  command limited to 6 deg and sent to channel 14 DRIVECDUS/T. TVCEXEC
  (`exec.c:agc_task_tvcexec`, every 0.5 s) ends unless DAPBIT1/2 read 10;
  calls the roll DAP (`rolldap.c`: OGA within 5 deg of OGAD, phase-plane
  regions 1-6, roll jets timed by T6) and NEEDLER; with the engine on
  every 10 s updates mass and gains; CNTR = 0 makes the one-shot trim
  correction, CNTR < 0 the repetitive one. V46 switch-over to CSM/LM gains
  (`csm_tvc_swichovr_body`, SWTOVER). V68 stroke test (`stroke.c`, pitch,
  CSM/LM only). Restarts (`restarts.c`) complete an interrupted copycycle,
  re-enable TVC (channel 12 TVCENABLE) and optics error counters, and
  restart through TVCINIT4 or MRCLEAN.

### Decision points by function

25 of the lane's 128 functions test or change a flag, raise an alarm, test a channel bit or start a job/task (AGC = the label(s) the prolog names; "+F" sets flag F, "-F" clears it; "starts" = NOVAC/FINDVAC, WAITLIST, LONGCALL, DELAYJOB targets).

| C function | AGC | flags tested | flags set (+) / cleared (-) | alarms | channel bits | starts |
|---|---|---|---|---|---|---|
| `csm_dap_engine_on` (mysubs.c) | - | - | - | - | ch11 ENGINEON | - |
| `csm_rcs_ahfnorot` (attitude.c) | AHFNOROT AUTOCONT | - | - | - | ch31 ATTHOLD, ch31 FREE | - |
| `agc_csm_dap_decode_dap_data` (dapdata.c) | S41.2 | - | =ATTCHFLG =NJETSFLG | - | - | - |
| `agc_csm_dap_rcsdapon_t5` (dapdata.c) | RCSDAPON | - | -DAPBIT1 +DAPBIT2 | - | - | - |
| `agc_job_ambgupdt` (exec.c) | AMBGUPDT | DAPBIT1 DAPBIT2 | - | - | - | - |
| `csm_rcs_kmatrix` (filter.c) | KMATRIX TENTHSEK | ATTITUDE_ERROR_DESIRED_ANGLES NEEDLFLG | - | - | - | job ambgupdt |
| `agc_task_jetslect` (jet_select.c) | JETSLECT ENDJETS | - | - | - | ch13 ENBT6RUPT | - |
| `agc_task_t6start` (jet_select.c) | T6START T6RUPTOR T6L1 | - | - | - | ch13 ENBT6RUPT | - |
| `csm_rcs_manual` (manual.c) | NOCHANGE | - | +STIKFLAG | - | ch31 FREE | - |
| `csm_dap_needler` (needles.c) | NEEDLER NEEDLER1 NEEDLER2 | - | - | - | ch12 COARSE, ch12 ENBIMUERR | - |
| `agc_task_rcsatt` (t5rupt.c) | RCSATT | - | - | - | ch31 CMCCONTROL, ch31 FREE | - |
| `csm_rcs_rate_filter_phase` (t5rupt.c) | SETT5 | - | - | - | ch31 FREE | - |
| `rcs_zerot5` (t5rupt.c) | ZEROT5 IMUAOK RCSSWIT | - | - | - | - | job ambgupdt |
| `agc_csm_dap_zerojet` (t5rupt.c) | ZEROJET | - | - | - | ch13 ENBT6RUPT | - |
| `tvcd_entry` (daps.c) | PITCHDAP YAWDAP STROKER | - | - | - | ch14 DRIVECDUS, ch14 DRIVECDUT | - |
| `agc_task_tvcexec` (exec.c) | TVCEXEC | DAPBIT1 DAPBIT2 | - | - | - | task tvcexec, task rolldap |
| `tvc_init_filter_timing` (init.c) | TVCINIT2 | - | -SWTOVER | - | - | - |
| `csm_tvc_init_complete` (init.c) | TVCINIT4 ENDTVCIN | - | - | - | - | task tvcexec |
| `csm_tvc_swichovr_body` (init.c) | SWICHOVR RTRNLOC | - | +SWTOVER | - | - | - |
| `tvcr_tvc_enable` (restarts.c) | ENABL1 | - | - | - | ch12 TVCENABLE | - |
| `agc_task_redotvc` (restarts.c) | REDOTVC EXECPHS TVCDAPHS | - | - | - | - | task exrstrt |
| `agc_task_optics_counter_enable` (restarts.c) | ENABL2 | - | - | - | ch12 ENBOCDUERR | - |
| `roll_jetroll` (rolldap.c) | JETROLL SAMEJETS T6ENABL | - | - | - | ch13 ENBT6RUPT | - |
| `stroke_burst` (stroke.c) | HACKWLST REVUP | - | - | - | ch14 DRIVECDUS | - |
| `csm_tvc_hack` (stroke.c) | HACK | - | - | - | - | task hackwlst |

### Shared erasables read and written

Accessors of `include/agc/core/state.h` and the lanes' public headers called by this lane, with the number of its functions that read / write each. Lane-private erasables are `static` in the lane's files or in its state struct; their scales are on the declarations.

| erasable (accessor) | meaning and scale (accessor prolog) | read by | written by |
|---|---|---|---|
| `cdux` | CDU X (outer gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (register CDUX, 032) | 6 | 0 |
| `cduy` | CDU Y (inner gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUY) | 5 | 0 |
| `cduz` | CDU Z (middle gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUZ) | 6 | 0 |
| `csmmass` | CSM mass, kg, scale 2^16 (CSMMASS; TVCMASSPROP 'B+16 KILOGRAMS'; noun 47 R1) | 3 | 2 |
| `dapdatr1` | DAP configuration word 1 (DAPDATR1, DSP noun 46 R1) | 12 | 2 |
| `dapdatr2` | DAP configuration word 2 (DAPDATR2, DSP noun 46 R2; Comanche only, storage exists in both builds) | 1 | 0 |
| `imodes33` | last-sampled channel 33 / DAP-enable bits, T4RUPT C33TEST (IMODES33, B(1)PRM) | 2 | 0 |
| `lemmass` | LM mass, kg, scale 2^16 (LEMMASS; DSP noun 47 R2) | 2 | 0 |
| `thetad` | desired gimbal angles THETAD +0..+2 (CPHI/CTHETA/CPSI), raw 15-bit CDU-style words, 2^15 = 1 revolution Index 0..2. | 1 | 0 |
| `tvcpitch` | CDUTCMD (053) / CDUSCMD (054): optics (CSM) or RR (LM) CDU error counter drive, SP pulse counts; which 0 = CDUTCMD, 1 = CDUSCMD. In TVC mode the same  | 2 | 3 |
| `tvcyaw` | CDUTCMD (053) / CDUSCMD (054): optics (CSM) or RR (LM) CDU error counter drive, SP pulse counts; which 0 = CDUTCMD, 1 = CDUSCMD. In TVC mode the same  | 1 | 2 |

### AGC labels -> C functions

The "AGC" column above names the label each listed function translates; the complete label -> function -> file:line index for the lane (and the tree) is [docs/overview/xref.md](../overview/xref.md), generated by `tools/xref.py`.
