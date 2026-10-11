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
Build/test: lane builds clean (scratch review-dap-csm/cc.sh, t.sh link the lane against a private copy of the tree; the shared `make` is intermittently broken by other lanes). test_csm_dap 790/0, oracle_dap_csm 302 checks 0 fail 0 known.

DONE
* F9 PITCHDAP: TVC filter DP quantities are agc_csm_dp_pair_t (include/agc/csm/dap.h; AGC word pairs, DAS without sign agreement; helpers csm_dap_pair_* in mysubs.c / internal.h). daps.c, exec.c (CG.CORR), init.c rewritten (full prologs, style-clean). Oracle pitch cases bit-exact word by word; the 8 KNOWN F9 entries are hard checks now.
* ATTINIT bug: DOTVCON writes -ERRORY,-ERRORZ into ERRBTMP, init.c read errbinit[] that nobody wrote. errbinit removed; ATTINIT reads errbtmp.hi/.lo (ignition.c already adapted by guid-powered).
* daps.c tvcd_dap returned an uninitialised status on the pitch pass (fixed in rewrite).
* MCLEANJR leaves CNTRTMP = 1 (modelled in csm_tvc_swichovr_body).
* ROLLSET: second DDOUBL wraps the 16-bit A when product >= 1/2; the TS limit takes the opposite sign. Modelled (rolldap.c roll_fire_time). New oracle cases roll_wrap/roll_wrap_neg (tools/oracle/cases/gen/gen_dap_csm.py, regenerated dap_csm.spec/.txt) pass; they fail with the old true-sign limit. rolldap.c rewritten, style-clean.
* Renames applied tree-wide, registry regenerated (gen_registry.py + uncrustify on jobs.h/dispatch.c): agc_task_noroll1->agc_task_roll_jets_off, agc_task_tvcinit1->agc_task_tvc_initialize, agc_task_enabl2->agc_task_optics_counter_enable (+AGC_TASK_ ids), csm_tvc_init4->csm_tvc_init_complete, tvc_init2/3->tvc_init_filter_timing/tvc_init_trims, tvcr_enabl1->tvcr_tvc_enable, csm_rcs_phase1->csm_rcs_rate_filter_phase, csm_rcs_t5phase2->csm_rcs_attitude_control_phase, AGC_CSM_T5P_PHASE1/2->AGC_CSM_T5P_RATE_FILTER/ATTITUDE_CONTROL, csm_rcs_t6progm->csm_rcs_t6prog_manual, TVCX_* step names in exec.c. Not yet in docs/RENAMES.md (TODO).
* tests/oracle/dap_csm.c rewritten (style-clean, word-exact pair compares).
* Fidelity verified vs source (no change needed beyond the above): TVCEXECUTIVE, TVCINITIALIZE, TVCRESTARTS, TVCROLLDAP, TVCDAPS; RCS: RCSATT/SETT5/ZEROT5/ZEROJET/KALUPDT/T5PHASE2 (t5rupt.c), manual rotation/STICKCHK/FREEFUNC (manual.c), AUTOMATIC_MANEUVERS (attitude.c). Constants checked in the rope (review-dap-csm/rope.py, fixed for the bank 2/3 file order).

TODO (in this order)
* BUG QUARTER: filter.c QUARTER and needles.c MQUARTER use 8192; QUARTER = BIT13 = 010000 = 4096 (rope). Fix both (-(.1)ADOT in DRHOLOOP; CS QUARTER in NEEDLES).
* restarts.c: logic verified; style rewrite pending (Write failed because the file changed by the rename sed; re-read and rewrite). Currently compiles.
* RCS DAP F9 class: DRHO/ADOT/MERROR DAS results used by upper word (CS/CA DRHO, MERRORX->ERRORX), EDOT/KMPAC in JETS/JTIME, SMALLMP/DPADD -> pairs (drho/adot/merror lane-internal; wbody/cdud/delcdu stay agc_dp_t, written by imu/navigation). Also DPADD exactly +180 deg edge (AGC POSMAX,POSMAX vs C -2^28 -> csm_dap_sp_word assert).
* Review + style: stroke.c, massprop.c, needles.c, jet_select.c (verify njet and table constants in rope), dapdata.c, rcs/exec.c, t5rupt.c, filter.c, manual.c, attitude.c, mysubs.c, state.c, internal.h, include/agc/csm/dap.h, tests/csm/dap.c (add ATTINIT / pair-DAS / QUARTER tests).
* Rule-5 density >= 2.0 in every file; check_style findings to zero for the lane.
* docs: RENAMES.md section, dap-csm.md (deviations: ROLLSET removed, F9 resolved, pairs), oracle-cases.md dap-csm rows, MODULE_STATUS row review-dap-csm, findings table + Lessons here.
* make, make test, make check.
