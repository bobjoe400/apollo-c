# dap-csm -- lane history

Process diary of the dap-csm lane, moved verbatim out of [docs/modules/dap-csm.md](../../modules/dap-csm.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-14 (first and last commit touching the note).

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

## Requested core additions
- Erasable accessors (declared in src/csm/dap/pending.h until published; guid-powered asks for the same setters in src/csm/burns/guid_pending.h):
  `agc_sp_t agc_erase_tvcpitch(void); void agc_erase_set_tvcpitch(agc_sp_t v);` (TVCPITCH = CDUSCMD 054)
  `agc_sp_t agc_erase_tvcyaw(void);   void agc_erase_set_tvcyaw(agc_sp_t v);`   (TVCYAW = CDUTCMD 053), 85.41 arcsec/bit.
  The library has no definitions yet, so any binary that links the TVC DAP fails to link without them.
- TIME5/TIME6 raw-word access: now provided by agc_rupt_time / agc_rupt_set_time (exec.h). Resolved.

## Requested jobs-tasks
All in src/csm/dap/pending.h (provisional 0x0D00 range; REDORCS, REDOTVC, TVCEXEC match exec_pending.h): T5 AGC_TASK_RCSATT, JETSLECT, TVCDAPON, TVCINIT1, DAPINIT, PITCHDAP, YAWDAP, ENABL2, CMDSOUT, RCSUP, REDORCS, REDOTVC; T6 AGC_TASK_T6START, NOROLL1; waitlist AGC_TASK_ROLLDAP, HACKWLST, EXRSTRT, TVCEXEC; jobs AGC_JOB_AMBGUPDT (PRIO34), AGC_JOB_STRKTSTI (V68). Dispatch cases are needed for each.

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

