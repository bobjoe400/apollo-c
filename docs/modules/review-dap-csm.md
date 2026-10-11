# review-dap-csm -- review of the dap-csm lane

Lane files: src/csm/dap/{mysubs,state}.c, src/csm/dap/{internal,pending}.h,
src/csm/dap/rcs/*.c, src/csm/dap/tvc/*.c, include/agc/csm/dap.h,
tests/csm/dap.c, tests/oracle/dap_csm.c.
AGC (Comanche055): MYSUBS, RCS-CSM_DIGITAL_AUTOPILOT, AUTOMATIC_MANEUVERS,
RCS-CSM_DAP_EXECUTIVE_PROGRAMS, JET_SELECTION_LOGIC, TVCINITIALIZE,
TVCEXECUTIVE, TVCMASSPROP, TVCRESTARTS, TVCDAPS, TVCSTROKETEST, TVCROLLDAP,
P40-P47 S41.2/S40.14/RCSDAPON.

(Earlier run left no note; restarted 2026-10-10 from scratch.)

## Progress
Build/test: shared `make` is broken by nav-conic work in progress; scratch review-dap-csm/t3.sh builds the tree at 4b46b01 with this lane's files overlaid and runs the lane tests. test_csm_dap 790/0, oracle_dap_csm 468 checks 0 fail 0 known.

DONE
* F9 PITCHDAP: TVC filter DP quantities are agc_csm_dp_pair_t (include/agc/csm/dap.h; AGC word pairs, DAS without sign agreement; helpers csm_dap_pair_* in mysubs.c / internal.h). daps.c, exec.c (CG.CORR), init.c rewritten (full prologs, style-clean). Oracle pitch cases bit-exact word by word; the 8 KNOWN F9 entries are hard checks now.
* ATTINIT bug: DOTVCON writes -ERRORY,-ERRORZ into ERRBTMP, init.c read errbinit[] that nobody wrote. errbinit removed; ATTINIT reads errbtmp.hi/.lo (ignition.c already adapted by guid-powered).
* daps.c tvcd_dap returned an uninitialised status on the pitch pass (fixed in rewrite).
* MCLEANJR leaves CNTRTMP = 1 (modelled in csm_tvc_swichovr_body).
* ROLLSET: second DDOUBL wraps the 16-bit A when product >= 1/2; the TS limit takes the opposite sign. Modelled (rolldap.c roll_fire_time). New oracle cases roll_wrap/roll_wrap_neg (tools/oracle/cases/gen/gen_dap_csm.py, regenerated dap_csm.spec/.txt) pass; they fail with the old true-sign limit. rolldap.c rewritten, style-clean.
* Renames applied tree-wide, registry regenerated (gen_registry.py + uncrustify on jobs.h/dispatch.c): agc_task_noroll1->agc_task_roll_jets_off, agc_task_tvcinit1->agc_task_tvc_initialize, agc_task_enabl2->agc_task_optics_counter_enable (+AGC_TASK_ ids), csm_tvc_init4->csm_tvc_init_complete, tvc_init2/3->tvc_init_filter_timing/tvc_init_trims, tvcr_enabl1->tvcr_tvc_enable, csm_rcs_phase1->csm_rcs_rate_filter_phase, csm_rcs_t5phase2->csm_rcs_attitude_control_phase, AGC_CSM_T5P_PHASE1/2->AGC_CSM_T5P_RATE_FILTER/ATTITUDE_CONTROL, csm_rcs_t6progm->csm_rcs_t6prog_manual, TVCX_* step names in exec.c. Not yet in docs/RENAMES.md (TODO).
* Rule-7: 32 `(void) AGC_ASSERT` postconditions in src/csm/dap/tvc/* turned into guards with recovery (void: return; status: return st / BAD_ARG; step: TVCX_FIN; CG.CORR: uncorrected DELOFF; cascade: zero output). check_rules 0 hard for the lane.
* Word-level API adoption (TVC): agc_csm_dp_pair_t removed; the TVC filter erasables are agc_dp_words_t, csm_dap_words_das wraps agc_dp_words_das (in-place DAS K returning A's overflow), csm_dap_pair/_value replaced by agc_dp_to_words/agc_dp_from_words; helpers renamed csm_dap_pair_* -> csm_dap_words_*. Oracle 302/0 unchanged.
* BUG QUARTER fixed: filter.c QUARTER and needles.c MQUARTER were 8192 (HALF); QUARTER = BIT13 = 4096 (rope). DRHOLOOP's -(.1)ADOT term was doubled; NEEDLES drove EDRIVE = -AK/2 instead of -AK/4. New oracle cases rate_* (RATEFILT -> KMATRIX, 5 cases) and needles_* (NEEDLES, 4 cases) pass word-exact; with 8192 rate_kalman etc. fail.
* RCS DAP F9 class: DRHO, ADOT, MERRORX are agc_dp_words_t (dap.h); RATEFILT/ADOTLOOP, MERUPDAT/ZEROER/SPNDXCHK, JETS..JTIME (EDOT, EDOTVEL, WTEMP, KMPAC as words, SIGNCK1-5 as CCS on upper then lower word, TAU = KMPAC+1 when the upper word is 0) rewritten on words. New oracle cases pp_* (JETS -> T6PROG, 8 cases incl. sign-disagreeing ADOT/WBODY words, forced firing, saturation) pass. filter.c and attitude.c rewritten with full prologs.
* DPADD exact +-180 deg edge: the AGC's COM/AD POSMAX path gives POSMAX,POSMAX; C gave -2^28 (out of range, assert). Fixed in csm_dap_dpadd.
* Rule 7 (user, 2026-10-11): 31 `(void) csm_dap_words_das(...)` casts removed. csm_dap_words_das is void (A discarded: every DAS site in the lane either overwrites A next or ends the routine); csm_dap_words_das_carry returns A for the one kind of site that keeps it: 1/2/3DAPCAS "DAS DAPn; DXCH TMPn" now stores (overflow, +0) in TMPn as the AGC does (was hard-coded 0).
* tests/oracle/dap_csm.c rewritten (style-clean, word-exact pair compares).
* Fidelity verified vs source (no change needed beyond the above): TVCEXECUTIVE, TVCINITIALIZE, TVCRESTARTS, TVCROLLDAP, TVCDAPS; RCS: RCSATT/SETT5/ZEROT5/ZEROJET/KALUPDT/T5PHASE2 (t5rupt.c), manual rotation/STICKCHK/FREEFUNC (manual.c), AUTOMATIC_MANEUVERS (attitude.c). Constants checked in the rope (review-dap-csm/rope.py, fixed for the bank 2/3 file order).

TODO (in this order)
* restarts.c: logic verified; style rewrite pending (Write failed because the file changed by the rename sed; re-read and rewrite). Currently compiles.
* Review + style: stroke.c, massprop.c, needles.c, jet_select.c (verify njet and table constants in rope), dapdata.c, rcs/exec.c, t5rupt.c, manual.c, mysubs.c, state.c, internal.h, include/agc/csm/dap.h, tests/csm/dap.c (add ATTINIT / pair-DAS / QUARTER tests).
* Rule-5 density >= 2.0 in every file; check_style findings to zero for the lane.
* docs: RENAMES.md section, dap-csm.md (deviations: ROLLSET removed, F9 resolved, pairs), oracle-cases.md dap-csm rows, MODULE_STATUS row review-dap-csm, findings table + Lessons here.
* Request to imu lane (note): agc_imu_hw_set_cducmd asserts |pulses| <= 192 (coarse-align COMMAX), but NEEDLES' ADS CDUXCMD legitimately writes up to +-768 (also LM needles); the write is refused. Widen to the SP range.
* make, make test, make check.
