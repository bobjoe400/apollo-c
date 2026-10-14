# review-dap-csm -- lane history

Process diary of the review-dap-csm lane, moved verbatim out of [docs/modules/review-dap-csm.md](../../modules/review-dap-csm.md), which keeps the reader-facing explanation. Note dates: 2026-10-10 to 2026-10-13 (first and last commit touching the note).

## Progress
STATUS: DONE (2026-10-11). `make`, `make test` green (test_csm_dap 800/0, oracle_dap_csm 533 checks / 40 cases, 0 failures, 0 KNOWN); `make format-check` clean; check_rules: 0 hard violations in the lane (rule-5 density warnings only, see Lessons); check_style: 0 findings in all 21 lane files. While the shared tree was broken by another lane, scratch review-dap-csm/t3.sh built the tree at 4b46b01 with this lane's files overlaid.

DONE
* F9 PITCHDAP: TVC filter DP quantities are agc_csm_dp_pair_t (include/agc/csm/dap.h; AGC word pairs, DAS without sign agreement; helpers csm_dap_pair_* in mysubs.c / internal.h). daps.c, exec.c (CG.CORR), init.c rewritten (full prologs, style-clean). Oracle pitch cases bit-exact word by word; the 8 KNOWN F9 entries are hard checks now.
* ATTINIT bug: DOTVCON writes -ERRORY,-ERRORZ into ERRBTMP, init.c read errbinit[] that nobody wrote. errbinit removed; ATTINIT reads errbtmp.hi/.lo (ignition.c already adapted by guid-powered).
* daps.c tvcd_dap returned an uninitialised status on the pitch pass (fixed in rewrite).
* MCLEANJR leaves CNTRTMP = 1 (modelled in csm_tvc_swichovr_body).
* ROLLSET: second DDOUBL wraps the 16-bit A when product >= 1/2; the TS limit takes the opposite sign. Modelled (rolldap.c roll_fire_time). New oracle cases roll_wrap/roll_wrap_neg (tools/oracle/cases/gen/gen_dap_csm.py, regenerated dap_csm.spec/.txt) pass; they fail with the old true-sign limit. rolldap.c rewritten, style-clean.
* Renames applied tree-wide, registry regenerated (gen_registry.py + uncrustify on jobs.h/dispatch.c): agc_task_noroll1->agc_task_roll_jets_off, agc_task_tvcinit1->agc_task_tvc_initialize, agc_task_enabl2->agc_task_optics_counter_enable (+AGC_TASK_ ids), csm_tvc_init4->csm_tvc_init_complete, tvc_init2/3->tvc_init_filter_timing/tvc_init_trims, tvcr_enabl1->tvcr_tvc_enable, csm_rcs_phase1->csm_rcs_rate_filter_phase, csm_rcs_t5phase2->csm_rcs_attitude_control_phase, AGC_CSM_T5P_PHASE1/2->AGC_CSM_T5P_RATE_FILTER/ATTITUDE_CONTROL, csm_rcs_t6progm->csm_rcs_t6prog_manual, TVCX_* step names in exec.c. Not yet in docs/history/RENAMES.md (TODO).
* Rule-7: 32 `(void) AGC_ASSERT` postconditions in src/csm/dap/tvc/* turned into guards with recovery (void: return; status: return st / BAD_ARG; step: TVCX_FIN; CG.CORR: uncorrected DELOFF; cascade: zero output). check_rules 0 hard for the lane.
* Word-level API adoption (TVC): agc_csm_dp_pair_t removed; the TVC filter erasables are agc_dp_words_t, csm_dap_words_das wraps agc_dp_words_das (in-place DAS K returning A's overflow), csm_dap_pair/_value replaced by agc_dp_to_words/agc_dp_from_words; helpers renamed csm_dap_pair_* -> csm_dap_words_*. Oracle 302/0 unchanged.
* BUG QUARTER fixed: filter.c QUARTER and needles.c MQUARTER were 8192 (HALF); QUARTER = BIT13 = 4096 (rope). DRHOLOOP's -(.1)ADOT term was doubled; NEEDLES drove EDRIVE = -AK/2 instead of -AK/4. New oracle cases rate_* (RATEFILT -> KMATRIX, 5 cases) and needles_* (NEEDLES, 4 cases) pass word-exact; with 8192 rate_kalman etc. fail.
* RCS DAP F9 class: DRHO, ADOT, MERRORX are agc_dp_words_t (dap.h); RATEFILT/ADOTLOOP, MERUPDAT/ZEROER/SPNDXCHK, JETS..JTIME (EDOT, EDOTVEL, WTEMP, KMPAC as words, SIGNCK1-5 as CCS on upper then lower word, TAU = KMPAC+1 when the upper word is 0) rewritten on words. New oracle cases pp_* (JETS -> T6PROG, 8 cases incl. sign-disagreeing ADOT/WBODY words, forced firing, saturation) pass. filter.c and attitude.c rewritten with full prologs.
* DPADD exact +-180 deg edge: the AGC's COM/AD POSMAX path gives POSMAX,POSMAX; C gave -2^28 (out of range, assert). Fixed in csm_dap_dpadd.
* Rule 7 (user, 2026-10-11): 31 `(void) csm_dap_words_das(...)` casts removed. csm_dap_words_das is void (A discarded: every DAS site in the lane either overwrites A next or ends the routine); csm_dap_words_das_carry returns A for the one kind of site that keeps it: 1/2/3DAPCAS "DAS DAPn; DXCH TMPn" now stores (overflow, +0) in TMPn as the AGC does (was hard-coded 0).
* Rule 7 "no silent failures" (user, 2026-10-11): every guard in a void or value function of the lane was classified. All were guaranteed by construction (word ranges of agc_sp_t values produced by the lane's own overflow-corrected arithmetic, &local / constant / bounded-loop arguments, enum arguments, postconditions on values just assigned) and were deleted (106 AGC_ASSERT lines removed, incl. the positive-form guards in csm_dap_words_neg/_mp/_smallmp and the cascade range check that zeroed DAPn); csm_dap_words_das stays void (A discarded), csm_dap_words_das_carry returns the real carry. One guard hid a real case: ROLLDAP RATENUM's DV by TEMREG = 0 (den == 0) returned 0; now modelled as the DV overflow (MINLIMAP) with a DEVIATION note on the -0 divisor sign. Fallible checks stay in agc_status_t functions (ATTKALMN table index, TVCEXPHS restart point, STICKCHK indices, set_t5 arguments). check_rules: 0 hard. Rule-5 density warnings are the price (most remaining functions have no failure path).
* Style: mysubs.c (full prologs; unused csm_dap_das/csm_dap_smallmp on values removed), internal.h, state.c, include/agc/csm/dap.h (prolog, one declaration per line, scales on every member), rcs/exec.c, needles.c, t5rupt.c, manual.c (re-verified against the listing; agc_csm_dap_stickchk forwarding wrapper removed, csm_rcs_stickchk renamed to it) at 0 check_style findings.
* massprop.c and dapdata.c re-verified and rewritten (prologs; DXTEST DAS through csm_dap_words_das; dead status asserts removed). New oracle cases mass_* (MASSPROP -> TC Q, 5 cases: CSM alone above/below the breakpoint, docked, docked light, descent stage off) pass word-exact; oracle 533 checks.
* stroke.c re-verified (TSTINIT, HACK/HACKWLST, REVUP, STROKILL; CHKSTRK in restarts.c) and rewritten; stroke_burst has no failure path (void).
* BUG T6START: t6_replace returned AGC_STATUS_BAD_ARG for REPLACER and REPLACEP (only the yaw case set OK), and the T6RUPTOR scan stopped on a non-OK status, so after a roll or pitch replacement the later BLAST entries were not processed in that T6 interrupt (a yaw or pitch firing ran until the next T6/T5 pass). t6_replace is void now (enum switch, no failure path); new unit test 6 (all three words replaced in one T6 interrupt; a positive interval becomes TIME6). jet_select.c rewritten with prologs; all tables and constants checked in the rope (PYTABLE, RTABLE, YZTABLE, XLNNDX, XLN1NDX, NJET, DFTMAX, masks, timers).
* tests/csm/dap.c: prologs, main first, test 6; 800 checks.
* tests/oracle/dap_csm.c rewritten (style-clean, word-exact pair compares).
* Fidelity verified vs source (no change needed beyond the above): TVCEXECUTIVE, TVCINITIALIZE, TVCRESTARTS, TVCROLLDAP, TVCDAPS; RCS: RCSATT/SETT5/ZEROT5/ZEROJET/KALUPDT/T5PHASE2 (t5rupt.c), manual rotation/STICKCHK/FREEFUNC (manual.c), AUTOMATIC_MANEUVERS (attitude.c). Constants checked in the rope (review-dap-csm/rope.py, fixed for the bank 2/3 file order).


## Findings

| # | where | finding | resolution |
|---|---|---|---|
| D1 | rcs/filter.c, rcs/needles.c | QUARTER written as 8192 (HALF); BIT13 = 4096: DRHOLOOP's -(.1)ADOT doubled, NEEDLES EDRIVE = -AK/2 | fixed; oracle rate_*, needles_* |
| D2 | tvc/daps.c, exec.c, init.c | F9: TVC DP erasables carried as sign-agreed values; the AGC uses upper words alone | agc_dp_words_t; 8 KNOWN -> hard checks |
| D3 | rcs/filter.c, manual.c, attitude.c | same class in the RCS DAP: DRHO, ADOT, MERRORX, EDOT, KMPAC; JTIME takes TAU from KMPAC+1 | agc_dp_words_t; oracle pp_* (JETS) |
| D4 | tvc/init.c | ATTINIT read errbinit[] that nobody wrote (DOTVCON stores -ERRORY,-ERRORZ in ERRBTMP) | reads ERRBTMP |
| D5 | tvc/daps.c | tvcd_dap returned an uninitialised status on the pitch pass | fixed |
| D6 | tvc/init.c | MCLEANJR leaves CNTRTMP = 1 | modelled |
| D7 | tvc/rolldap.c | ROLLSET: 16-bit A wrap of the second DDOUBL and the opposite-sign limit | modelled; oracle roll_wrap* |
| D8 | tvc/rolldap.c | RATENUM DV by TEMREG = 0 returned 0 behind an assertion | MINLIMAP modelled (deviation: -0 sign) |
| D9 | mysubs.c | DPADD of exactly +-180 deg gave -2^28 (out of range) | POSMAX, POSMAX as the AGC |
| D10 | rcs/jet_select.c | T6START: REPLACER/REPLACEP returned BAD_ARG, ending the T6RUPTOR scan after a roll or pitch replacement | t6_replace void; unit test 6 |
| D11 | tvc/daps.c | 1/2/3DAPCAS: DXCH TMPn stores the first DAS's overflow; C hard-coded 0 | csm_dap_words_das_carry |
| D12 | lane | 32 `(void) AGC_ASSERT`, 31 `(void) csm_dap_words_das`, 106 guaranteed guards with silent/sentinel returns | rule 7: casts removed, guards deleted, real paths kept as status |
| D13 | rcs/manual.c | agc_csm_dap_stickchk forwarded to csm_rcs_stickchk | wrapper removed |

DMP/DMPR audit: the lane has no interpretive code (all multiplies are basic MP, SMALLMP, DV); 0 sites. Shift rounding: no SR/SRR/VSR. Overflow stores: AD/ADS/DAS are overflow-corrected wraps; the only limits are the AGC's own (ERRORLIM, ACTLIM, the "TS; INDEX A; CS LIMITS" idiom in ROLLSET/DESIRED).

Constants and tables checked in the rope: GAIN1/GAIN2, MANTABLE, FREETAU, PYTABLE, RTABLE, YZTABLE, XLNNDX, XLN1NDX, NJET, DFTMAX, jet masks and timer words, NOLEMVAL/INTVALUE/SLOPEVAL, NEGBPW, DXITFIX, TVC filter and roll constants.

New oracle cases (tools/oracle/cases/gen/gen_dap_csm.py): rate_* (5), needles_* (4), pp_* (8), mass_* (5), roll_wrap* (2).

## Requests
* imu: agc_imu_hw_set_cducmd asserts |pulses| <= 192 (coarse-align COMMAX), but NEEDLES' ADS CDUXCMD legitimately writes up to +-768 (CSM and LM needles); such a write is refused (the oracle run prints the assertion). Widen it to the SP range.

## Lessons
* A constant named after a fraction (QUARTER) must be checked against the equate in the rope, not against intuition about 2^14 = 1: BIT13 is 1/4, BIT14 is 1/2.
* A status-returning helper that is "OK" only in its last switch case silently truncates a caller's loop that stops on non-OK; functions with no failure path should not return a status at all.
* Word-level erasables need oracle cases with sign-disagreeing input words; value-based code passes every sign-agreeing case.
* yaAGC drains CDUXCMD through its coarse-align burst model once channel 14 is set, so counter outputs cannot be compared at a fixed stop label.
* Deleting guaranteed guards drops rule-5 density far below 2.0 in arithmetic-heavy files; the density target and the no-silent-failure rule pull in opposite directions, and the latter wins.

