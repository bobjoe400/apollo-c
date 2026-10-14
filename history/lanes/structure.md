# structure -- lane history

Process diary of the structure lane, moved verbatim out of docs/modules/structure.md. The rest of that note (its explanation) is appended at the end, under "Module note (kept explanation)". Note dates: 2026-10-14 to 2026-10-14 (first and last commit touching the note).

## Task 1 results

Comparison (scratch harness SCRATCH/structure/diff/, both copies linked
side by side; static copies reached by #including their file):

| Group | Inputs compared | Result |
|---|---|---|
| CDULOGIC: rtb vs imu vs align | all 65,536 16-bit words | identical |
| 1STO2S on DP values: rtb vs align vs words form | 4,000,000 values (200,000 around 0, the rest random) + 15 edges (+-0, +-1, +-8192, +-DPOSMAX, +-1/2 rev) | identical |
| 1STO2S on MPAC words (signs free) | 1.6 M word pairs | differ in 231,709 pairs (finding F2) |
| new rtb 1TO2SUB (words) vs old align_words_to_cdu | 93.7 M word pairs | identical |
| LOADTIME x3 | 200,000 TIME2/TIME1 | identical |
| SIGNMPAC: rtb vs entry | 2,000,000 values | identical |
| LOG: agc_csm_log vs entry_log | 2,000,000 x in (0, 1): value and OVFIND | identical |
| QUALITY2: rtb (DP) vs lunar_j22_term (words) | 200,000 random URPV, E3J22R2M | differ in 60,600 of 600,000 components, up to 8 LSB (finding F1) |
| MUTABLE: rz constants vs conic table | the four words | identical (155704375, 176229826, 122569450, 198627193) |
| SELECTMU + VN1645 (csi, tpi, stable orbit x2); LEMSTORE/CSMSTORE (tpi_search vs PRECSET) | code | identical text |

Fidelity findings:
- F1 QUALITY1/2. agc_rtb_quality1/2 (io/service/rtb_ops.c) computed the
  lunar J22 term with DP-value arithmetic (agc_dp_sq, agc_dp_mul_trunc);
  the listing's DSQ, DMP, VXSC run on the MPAC words, and
  lunar_j22_term (nav/integration/orbital_integration.c) does that and
  is exact against yaAGC (tests/oracle/nav_integration.c step_moon,
  integ_moon_*, tolerance 0). The rtb copy was up to 8 LSB off and had
  no caller (no flight effect). Kept lunar_j22_term (Luminary files
  QUALITY1-3 in ORBITAL_INTEGRATION, nav/integration's section); the
  rtb copy is deleted.
- F2 1STO2S / 1TO2SUB. agc_rtb_1sto2s took an agc_dp_t, which cannot
  hold words whose signs disagree; the listing's DDOUBL acts on the
  words, so for such words (ASIN, ARCTRIG, DAD results) the value form
  is one count off. The alignment lane's align_words_to_cdu was the
  faithful copy (oracle-exact in RRANGLES, SXTANG). It is now the RTB
  op's only 1TO2SUB (rtb_ops.c one_to_two on words): agc_rtb_1sto2s
  (values; unchanged on every value), agc_rtb_1sto2s_words (words),
  V1STO2S and 2V1STO2S (now on words) all use it.

Changes: callers moved by Coccinelle (SCRATCH/structure/cocci/
t1_calls.cocci; three test lines inside CHECK() by hand, spatch cannot
rewrite macro arguments); deleted agc_imu_cdulogic, agc_align_cdulogic,
agc_align_ones_to_twos, align_words_to_cdu, agc_integ_loadtime,
align_loadtime, entry_signmpac, entry_log, agc_rtb_quality1/2,
agc_align_cdutrig, agc_align_calcsmsc, rz_mutable_mu + 4 constants,
csi_selectmu, tpi_selectmu, tps_store, td_bvectors, mu_incorp2, the
entry lexicon's LOG/DPOSMAX copies; agc_imu_calcsmsc reads the trig
block (SINCDU/COSCDU, as the AGC) so the alignment adapter goes;
new: agc_rtb_1sto2s_words, rz_selectmu_display, rz_vehicle_store,
agc_conic_inv_root_mu_ (agc_conic_mu_ now public). RANGER
(entry_ranger_core) returns LOG's status (ST_ERROR on failure).
Tests: tests/programs/alignment.c checks 1STO2S on mixed words and
2V1STO2S.

## Task 2 results

| # | Routine | Done | Behaviour |
|---|---|---|---|
| 1 | SERVEXIT | agc_job_servexit and PH_5_3_A moved to guidance/servicer_common.c (prototype already in guidance.h) | none |
| 2 | IMUTEST | the CSM REDO is agc_job_systems_test_csm in csm/prelaunch/align.c (NEWMODEX 07, then IMUTEST, now file-static prelaunch_imutest) and is the registry's CSM handler of AGC_JOB_SYSTEMS_TEST; the LM REDO (lm/systems_test.c) lost its CSM arm | wiring fix: the CSM V92 job (SYSTEST FINDVAC REDO, extended_verbs_csm.c) had no CSM handler and ended NOT_FOUND |
| 3 | PREREAD | PRECHECK (csm/burns/sps/burn.c) calls agc_task_preread_csm; the LM arm of its vehicle switch could not run (AGC_TASK_PRECHECK is CSM-only in the registry) | none |
| 4 | P20/P22 entry | the vehicle switch functions agc_job_rendezvous_navigation / agc_job_orbital_navigation (csm/navigation/navigation.c) deleted; the registry dispatches by vehicle (CSM: PROG20 agc_job_rendezvous_navigation_csm, PROG22 agc_job_orbital_navigation_csm; LM: both agc_job_rendezvous_navigation_lm) | wiring fix: the LM columns were empty, so V37E20E / V37E22E (and the LM's group-4 restart of PROG20) ended NOT_FOUND on the LM |
| 5 | B5OFF | agc_job_sband_b5off moved to io/service/service.c beside agc_svc_b5off; prototype to io/service.h | none |

Registry: only handler columns changed (group column kept: it orders the
job ids, which stay as they were; jobs.h differs in three comments).
Test: tests/programs/rendezvous.c test_vehicle_dispatch (LM P20, P22
and CSM V92 dispatched, MODREG 07).

## Task 3 results: the 39 labelled routines nothing called

Method: every code reference to the label in both ropes
(SCRATCH/structure/refs.py: operand tokens, comments stripped), the
enclosing label of each site and the C function xref gives for it; then
the C at that site. Classes: (i) entered from outside the code by
design (interrupt vector, registry, downlink/noun table, harness input);
(ii) a call the AGC makes that the C did not: wired; (iii) AGC code no
Apollo 11 path enters: kept, documented; (iv) the AGC's uses are
translated through another C function (the file's static or word form,
an equated or fall-through label, a primitive used everywhere, or the
owner reading its own storage): the public entry is redundant, kept.

Counts: (i) 5, (ii) 2 wired, (iii) 8, (iv) 23, and 1 removed as a
duplicate in task 1 (agc_rtb_quality1). Total 39.

| Function | Class | Evidence |
|---|---|---|
| agc_erase_sfail | iv | SFAIL is read only by TC SFAIL (self-check continuation), which the C self-check does by control flow |
| agc_dp_div_ovf_only | iv | DDV/BOV sites use agc_dp_div / agc_mpac_ddv |
| agc_dp_mul_raw | iv | DMPR sites use agc_dp_mul / agc_mpac_dmpr |
| agc_earth_orbit_insertion_monitor_set_padload | i | pad-load entry (ground loads; src/sim/pad.c has no P11 item) |
| agc_erase_trunbias | iv | R57B writes and R23.55 reads TRUNBIAS, both in cislunar_navigation.c, which owns it |
| agc_abort_restart | iii | ABORT (EQUALS BAILOUT / WHIMPER) appears in comments only, no TC ABORT in either rope |
| agc_fresh_dofstart | iv | DOFSTART is entered from PTBAD / NONAVKEY inside FRESH_START_AND_RESTART: static dofstart() |
| agc_rupt_t3 | i | T3RUPT vector; agc_sim takes T3 through its own lead-in (src/sim/clock.c rupt_t3, the overflowing PINC) |
| agc_phase_new | iv | PHASCHNG type C/A/D/J sites use agc_phase_change_var |
| agc_imu_comp_irig | iv | IRIGCOMP is reached from 1/PIPA: comp_irig_words |
| pin_dspdecnr | iii | DSPDECNR: no reference in either rope |
| pin_dsp2bit | iii | DSP2BIT: only a SETLOC patch address (DSP2BIT +10D), no call |
| agc_dsky_nvmonopt | iv | NV50DSP's TC NVMONOPT is display_interface.c's pin_nvmonopt call |
| agc_dsky_nvsbwait | iii | NVSBWAIT: no reference |
| agc_dsky_jamterm | iv | CLEANEND's FINDVAC 2CADR JAMTERM: dif_cleanend starts agc_job_jamterm |
| agc_dsky_jamproc | iii | JAMPROC: no reference |
| agc_rtb_2v1sto2s | ii | RRANGLES (LEM_GEOMETRY.agc p.323) RTB 2V1STO2S twice: agc_lm_rrangles now calls it on the MPAC words (was two align_words_to_cdu each); test: oracle RRANGLES (exact), tests/programs/alignment.c |
| agc_rtb_incrcdus | iii | INCRCDUS: no reference (Comanche) |
| agc_rtb_readpips | iii | READPIPS: no reference (Comanche) |
| agc_rtb_pulseimu | iv | ERTHR's PULSEIMU: agc_imu_pulse_buf on the ERCOMP buffer (imu owns the gyro buffers) |
| agc_rtb_transp | ii | SXTANG (CSM_GEOMETRY.agc p.287) RTB TRANSP1, TRANSP1, TRANSP2: agc_csm_sxtang now calls it (was agc_mat_transpose on copies; same words); test: oracle SXTANG (exact) |
| agc_rtb_vecsgnag | iv | AGS SCALEVEC's VECSGNAG needs disagreeing words: ags_words (lm/ags_initialization.c) does it on the words; the rtb form on agc_vec_t is the identity |
| agc_rtb_quality1 | task 1 | duplicate of lunar_j22_term, deleted (F1) |
| agc_svc_upent2, agc_svc_downent2 | iv | the 7 display-interface sites use display_interface.c's FLAGWRD4 helpers f4_up / f4_down (which also serve UPFLAG/DOWNFLAG and direct masks there): a second translation, noted for the pinball review |
| agc_ags_buffer | i | AGSBUFF is read by the LMAGSIDL downlink list; see finding T1 |
| agc_lm_dap_needles_init | iv | NEEDLER's calling sequence (set RCSFLAGS bit 3) is inline at its AGC sites (dapidler.c, needles.c), as in the AGC |
| agc_lm_dap_t5rupt | i | T5RUPT: exec/interrupts.c agc_rupt_t5 runs T5ADR's task (lm_dap_set_t5adr registers it) |
| agc_lm_dap_t6_pending | iv | helper (AGC: none) |
| agc_erase_stiksens | iv | RATERROR / RHCACTIV read the DAP's own STIKSENS |
| agc_rendz_rr_downlink | i | the R29 downlink words are read by the downlink lists; see finding T1 |
| agc_ephem_csm_solpos | iii | SOLPOS: no reference (Comanche) |
| agc_planet_newangle, agc_planet_earthmx | iv | MOONMX / EARTHMX / RP-TO-R use newangle_words / earthmx_words |
| agc_navaux_tp_low_dp | iv | instruction-form helper (DLOAD X+1) |
| agc_conic_iterator | iv | LAMBLOOP calls iterator_run |
| agc_integ_othprec | iv | R30 OTHSHIP, R36, P76 OTHPREC: rz_integ(RZ_OTHPREC) -> rz_resolve_kind |
| agc_job_gyro_torquing_display | iv | R55.1 is entered by falling through from R55: agc_align_gyro_torquing -> gyro_angles_flash |
| rz_vec_from_body | iv | helper (VSL* 0,2); its sites use rz_vshift |

Finding T1 (telemetry lane): 92 of the 128 downlink items the lists name
are sent as zero (down_telemetry.c DEVIATION "items whose owner exports
no accessor yet"); AGSBUFF and the R29 words are two of them, and their
accessors sit above io (lm/), so the downlink cannot call them without
an upward edge. The fix is storage below io (core erasable, as the
layering lane did for MRKBUF) or the address column. Not changed here.
Items: SPARE RSBBQ ADOT AK THETADX ERRORX WBODY DELLT4 RTARG PIPTIME1
PACTOFF PCMD CSTEER DELVEET1-3 ELEV CENTANG DELTAR BESTI MARKDOWN MARK2DWN
HAPOX OGC LAUNCHAZ VHFTIME VHFCNT ECSTEER DELVTPF TCDH TPASS4 RANGE
DIFFALT RTHETA LAT_SPL VPRED CMDAPMOD L_D1 PAXERR1 LATANG ALFA_180
ROLLTM GAMMAEI TTE VIO SVMRKDAT LANDMARK LONG ALT LAT 8NN TCSI DNLRVELZ
DNLRVELX POSTORKU DNRRANGE POSTORKP AGSK TALIGN AOTCODE X789 LASTYCMD
STARSAV1 STARSAV2 AIG AOG TANGNB MKTIME RANGRDOT NN LRZCDUDL VSELECT
LRVTIMDL VMEAS HMEAS RM LRXCDUDL UNFC_2 VGVECT TTF_8 DELTAH RGU VGU LAND
AT TLAND FC PSEUDO55 TTOGO YNBSAV GSAV AGSBUFF (SPARE is zero in the AGC
too).

## Task 4 results: the three files split

Split by function (SCRATCH/structure/split.py: each function with its
prolog and the statics before it), one new FILE NAME prolog per file;
static functions one file of a folder calls in another became extern
with the prototype in the folder's internal.h (new for csm/entry and
lm/systems_test); unused includes dropped only where the file uses no
identifier of the header and the object code is unchanged
(SCRATCH/structure/incprune.sh). No function body changed except: the
systems-test steps that touch the test erasables take them through
st_words() (the array stays file-static in test_erasables.c).

| Was | Now (lines) | AGC section |
|---|---|---|
| src/programs/navigation/common.c (1607) | rendezvous_erasables.c (436): the rendezvous and navigation erasables, vehicle flags, word conversions; rendezvous_subroutines.c (832): P34-35 subroutines SELECTMU .. MIDGIM, AVFLAGA/P, P20FLGON and the interpretive helpers; state_vector_update.c (322): THISPREC/OTHPREC, INTSTALL/INTEGRVS set-up, INCORP2, BVECTORS, restart phases | ERASABLE_ASSIGNMENTS; P34-35_P74-75 (+ P30-P37); INTEGRATION_INITIALIZATION, MEASUREMENT_INCORPORATION, P20-P25 |
| src/csm/entry/reentry.c (2337) | entry_guidance.c (SCALEPOP, the sequencer, ENDEXIT, REFAZE10), entry_targeting.c (STARTENT, TARGETNG), entry_initial_roll.c (INITROLL .. KATEST, PREHUNT), entry_huntest.c (HUNTEST, RANGER), entry_upcontrol.c (UPCONTRL .. DOWNCNTL, KEP2), entry_final_phase.c (PREFINAL, PREDICT3, G limiter, lateral logic, roll command); internal.h (step ids, pass state, phase steps) | REENTRY_CONTROL pp. 847-874, by its own phase boundaries (the file has one AGC section once task 1 removed the LOG and SIGNMPAC copies) |
| src/lm/systems_test.c (2161) | src/lm/systems_test/: test_erasables.c (the W image, CCS, PIPA), imu_performance_test.c (858), drift_filter.c, job_entries.c (the registry's entries and the step dispatch); internal.h (constants, offsets, step ids, cross-file prototypes) | IMU_PERFORMANCE_TEST_2; IMU_PERFORMANCE_TESTS_4; SYSTEM_TEST_STANDARD_LEAD_INS (REDO) |

## Task 5 results: folder-internal prototypes

The organize list (123) re-checked at this tree (SCRATCH/structure/t5.py:
every identifier use in src/, tests/, sim/, include/ and the registry):
118 still have their definition and every user in one folder and no
test, harness (src/sim, sim/), NASSP (src/nassp, src/erasable_map) or
registry/dispatch user; 5 no longer apply (agc_align_cdutrig and
agc_align_calcsmsc deleted in task 1; agc_job_gopoodoo,
agc_job_gotopooh and agc_job_orbital_navigation_csm are registry
handlers, which src/exec/dispatch.c reaches through the public
headers). In every folder two or more files share the symbols, so:

- moved into an existing internal.h (8 folders, 29 prototypes):
  csm/dap (tvc's FIXCW), csm/entry (14, the task 4 header), csm/prelaunch,
  exec, imu, io/dsky/pinball, nav/integration, programs/navigation;
- new internal.h (13 folders, 89 prototypes): core, csm/burns,
  csm/navigation, csm/optics, guidance, io/service, io/verbs (34),
  lm/alignment, lm/burns, lm/landing, lm/radar, programs/alignment,
  programs/targeting.

A comment that served only the moved prototype moved with it; section
headers stayed. Each user file includes its folder's internal.h.

## Findings for other lanes

- T1 (telemetry): 92 downlink items are sent as zero (task 3 table).
- Pinball review: display_interface.c's f4_up / f4_down are a second
  translation of UPENT2 / DOWNENT2 (with UPFLAG / DOWNFLAG and direct
  FLAGWRD4 masks); the service routines' agc_svc_upent2 / downent2
  have no caller.
- The 23 class-(iv) public entries of task 3 have no caller by design of
  the C (their AGC uses go through another function); a later lane may
  delete them.
- tools/registry.tsv's group column no longer names the folder of three
  moved handlers (SERVEXIT csm/burns -> guidance, SBAND_B5OFF csm/optics
  -> io/service, SYSTEMS_TEST lm, now with a csm/prelaunch CSM handler);
  it orders the job ids, so it was left (as organize did for prelaunch).
- AGC_JOB_SBAND_B5OFF and AGC_JOB_RR_LOS_DISPLAY_STOP are both "B5OFF as
  a job" (two ids for one AGC routine).
- agc_job_gyro_torquing_display is declared in both alignment.h and
  rendezvous.h.

## Lessons

- Most of xref's "one label, several functions" rows are adapters (they
  marshal erasables and call the owner) or vehicle variants; read each
  before deduplicating, and compare outputs where there is real
  arithmetic: exhaustively when the input is a word, otherwise millions
  of values plus the edges (+-0, the overflow boundaries).
- A value-typed copy of a routine the AGC runs on MPAC words (1STO2S,
  QUALITY2, VECSGNAG) cannot be exact when the words' signs disagree;
  the word form is the faithful one, and the value form should call it.
- A vehicle switch inside one registry handler hides dispatch gaps: the
  other vehicle's column stays '-' and the job ends NOT_FOUND. Give each
  vehicle its own column entry.
- Coccinelle cannot rewrite inside macro arguments (the tests' CHECK());
  those few sites are edited by hand.
- Splitting a file whose code indexes a file-static array: give the
  array an accessor and a local pointer in each function that uses it
  (st_words() in lm/systems_test), which keeps rule 6 and leaves every
  expression as it was.

## Progress
- DONE task 1 (16 groups: 10 with copies or adapters removed, 6 with no copy: adapters, flag primitive, storage model, vehicle variants)
- DONE task 2 (5 routines; 2 were dispatch gaps, now wired with a test)
- DONE task 3 (39: i 5, ii 2 wired, iii 8, iv 23, 1 removed in task 1)
- DONE task 4 (3 files -> 3 + 6 + 4 files, 2 new internal.h)
- DONE task 5 (118 of 123 moved; 13 internal.h created, 8 extended)
- CHECKPOINT task 1, main tree: make OK; make test 36 binaries + 14 scenarios, 0 failures; make check 0 hard, format clean, 0 style findings; prolog-report 0; always-ok-report 0; lm-sim 34/34 matched; nassp-replay 5/5; depgraph no upward edge
- CHECKPOINT task 2, main tree: same gates green (make; test 36 binaries + 14 scenarios; check 0/clean/0; prolog 0; always-ok 0; lm-sim 34/34; nassp-replay 5/5; no upward edge)
- CHECKPOINT task 3, main tree: same gates green (36 binaries + 14 scenarios, check, prolog 0, always-ok 0, lm-sim 34/34, nassp-replay 5/5, no upward edge)
- CHECKPOINT task 4, main tree: same gates green (36 binaries + 14 scenarios, check 0 in 337 files, prolog 0, always-ok 0, lm-sim 34/34, nassp-replay 5/5, no upward edge)
- CHECKPOINT task 5 and final, main tree: make OK; make test 36 binaries + 14 scenarios, 0 failures; make check 0 hard, format clean, 0 style findings in 350 files; prolog-report 0; always-ok-report 0; lm-sim 34/34 matched; nassp-replay 5/5; depgraph no upward edge
- DONE docs/overview regenerated (depgraph.py -> README.md, xref.py -> xref.md, repo_index.py -> INDEX.md), MODULE_STATUS row
- Work was done in a scratch copy (SCRATCH/structure/wt, one commit per task) and written into this tree task by task (SCRATCH/structure/sync.sh), each followed by the gates above. No file under src/nassp, sim/nassp or src/erasable_map was changed.



## Module note (kept explanation)

Scope: history/ORGANIZATION.md proposals 2-5 on the tree after
tools/apply_layout.sh (HEAD 755db91): (1) the 16 duplicate-translation
groups, (2) the 5 cross-vehicle routines, (3) the 39 labelled routines
nothing calls, (4) splitting programs/navigation/common.c,
csm/entry/reentry.c and lm/systems_test.c at their AGC section
boundaries, (5) the 123 folder-internal prototypes. Out of bounds:
src/nassp/**, sim/nassp/**, src/erasable_map/** (nassp-integration lane).

Method: differential tests of the copies (scratch harness that #includes
the source file to reach static copies, links build/libagc_debug.a),
exhaustive where the domain is a word (65,536 CDU words), otherwise
millions of random DP inputs plus edge values; the oracle cases (yaAGC)
decide which copy matches the listing where copies differ. Caller
rewrites by Coccinelle (spatch) rules in SCRATCH/structure/.

### Design: task 1, the 16 groups

Reading every row of the organize table: a "copy" is either an
independent translation of the routine's arithmetic (a real duplicate),
or an adapter that marshals erasables and calls the owner (not a copy).
Real duplicates are compared, the callers moved to the owner's copy and
the copy deleted; one-line adapters are inlined at their call sites;
marshalling adapters stay and are listed.

| # | AGC routine | Owner kept | Copies | Kind | Action |
|---|---|---|---|---|---|
| 1 | CDULOGIC | agc_rtb_cdulogic (io/service/rtb_ops.c) | agc_imu_cdulogic (own code), agc_align_cdulogic (mask + imu call) | duplicate | callers -> owner, delete both |
| 2 | 1STO2S / 1TO2SUB | agc_rtb_1sto2s (rtb_ops.c), now on the words | agc_align_ones_to_twos (own code), align_words_to_cdu (words form) | duplicate | words form moves into rtb_ops as the one 1TO2SUB; callers -> owner, delete copies. lm_dap_ones_to_twos is 1STOTWOS (P-AXIS DAP's own SP subroutine), not a copy |
| 3 | CDUTRIG / CALCSMSC | agc_imu_cdtrigs, agc_imu_calcsmsc | agc_align_cdutrig, agc_align_calcsmsc | adapters (order X,Y,Z; vectors from the trig block) | callers -> owner, delete |
| 4 | LOADTIME | agc_rtb_loadtime | agc_integ_loadtime, align_loadtime (own code) | duplicate | callers -> owner, delete |
| 5 | SIGNMPAC | agc_rtb_signmpac | entry_signmpac (own code); bovb_signmpac / store_signmpac call the owner (BOVB forms) | duplicate | callers -> owner, delete entry copy |
| 6 | QUALITY1/2 | lunar_j22_term (nav/integration, Luminary's ORBITAL_INTEGRATION section, word level) | agc_rtb_quality1/2 (DP level, never called) | duplicate | see fidelity finding F1; delete the rtb copy |
| 7 | LOG; EARROT1/2 | agc_csm_log; agc_csm_earrot1/2 (programs/alignment/geometry_csm.c) | entry_log (own code); entry_earrot (marshals entry erasables, calls the owner) | duplicate; adapter | LOG callers -> owner, delete; entry_earrot kept |
| 8 | KEPPREP | agc_conic_kepprep | agc_integ_kepprep: owns the integration erasables, calls KEPPREP and KEPLERN | adapter | kept |
| 9 | LUNPOS / LUNVEL | agc_ephem_csm_lunpos/lunvel | lunpos/lunvel (static): vehicle switch (LM: LSPOS / RVQ) | adapter | kept |
| 10 | SELECTMU; MUTABLE | agc_rendz_selectmu; nav/conic MUTABLE table | csi_selectmu, tpi_selectmu (identical SELECTMU + CLEAR FINALFLG + VN1645 wrappers); rz_mutable_mu and four rz constants (a second copy of the MUTABLE words) | duplicate | one shared SELECTMU-with-VN1645 in programs/navigation; MUTABLE words from nav/conic |
| 11 | BVECTORS; PERIAPO; CSMSTORE/LEMSTORE | rz_bvectors; agc_rendz_periapo; rz_precset | td_bvectors (one-line adapter); tps_trial (calls PERIAPO1, not a copy); tps_store (the LEMSTORE/CSMSTORE test, also inline in rz_precset) | adapter; none; duplicate | inline td_bvectors; one store helper shared by rz_precset and tpi_search |
| 12 | LALOTORV | agc_latlong_to_r | cn_landmark (R23.3: pad + LAT LONG ALT, calls the owner) | adapter | kept |
| 13 | INCORP2; INTEGRVS | agc_incorp_update_w (+ rz_incorp2, the erasable driver); agc_integ_integrvs | mu_incorp2 (one-line adapter); rpd_from_base, rz_integrvs (INTSTALL + erasable set-up, call the owner) | adapters | inline mu_incorp2; others kept |
| 14 | DOWNFLAG; GENTRAN; SHOWSUM | agc_svc_downflag; agc_svc_gentran; agc_selfcheck_showsum_start | agc_extverb_downflag (TC DOWNFLAG, ADRES, TC GOPIN: the flag primitive agc_flag_clear, as at every other DOWNFLAG site in C); nc_mrkbuf_copy, to_mrkbuf1, agc_guid_copycycle (GENTRAN over accessor-held erasables); verb_show_banksum (V91 driver, calls the owner) | not copies | kept: the C flag primitive is how every UPFLAG/DOWNFLAG site is written; GENTRAN needs a contiguous block, which accessor-held erasables are not; SHOWSUM driver |
| 15 | PIPASR | agc_pipasr_noscale (core, PIPASR +3) | agc_guid_pipasr (INHINT, PIPTIME1, PIPAGE, then the owner) | adapter | kept |
| 16 | REDO5.5 | -- | redo5_5 csm/burns/servicer.c and lm/burns/servicer/average_g.c | vehicle variants: the listings differ (Comanche CM/DSTBY entry-DAP copy; Luminary FINDVAC SERVICER, AVEGOUT, R10/R11) | kept |

### Design: task 2, the 5 cross-vehicle routines

| # | Routine | Finding at HEAD | Action |
|---|---|---|---|
| 1 | SERVEXIT agc_job_servexit (csm/burns/servicer.c) | shared, LM calls it | move to guidance/servicer_common.c with PH_5_3_A |
| 2 | IMUTEST | REDO (V92) lives in lm/systems_test.c with a CSM arm calling the CSM's IMUTEST; registry CSM column is '-' so the CSM V92 job is never dispatched (NOT_FOUND) | the CSM REDO (NEWMODEX 07, fall into IMUTEST) goes to csm/prelaunch beside IMUTEST and into the registry's CSM column; the LM REDO loses the CSM arm |
| 3 | PREREAD (agc_task_precheck, csm/burns/sps/burn.c) | PRECHECK is CSM-only in the registry; the LM arm of its vehicle switch is unreachable | call agc_task_preread_csm |
| 4 | P20/P22 entry (csm/navigation/navigation.c) | vehicle switch calls lm/navigation; registry LM column '-' so LM P20/P22 (V37E20E, E22E) are not dispatched | registry dispatches by vehicle; vehicle-switch functions removed |
| 5 | B5OFF agc_job_sband_b5off (csm/optics) | registry handler of both vehicles | move to io/service/service.c |

### Program flow and decisions

_Final documentation pass (2026-10-14). This lane changed where code
lives and wired two dispatch paths; the decisions themselves are in the
owning lanes' sections._

| change | decision as the code now takes it | lane section |
|---|---|---|
| task 1, 1STO2S | `src/io/service/rtb_ops.c`: one 1TO2SUB on the MPAC words for every caller (`agc_rtb_1sto2s`, `agc_rtb_1sto2s_words`, V1STO2S, 2V1STO2S); `src/lm/optics/lem_geometry.c:agc_lm_rrangles` and `src/programs/alignment/geometry_csm.c:agc_csm_sxtang` call the RTB ops as the listing does | [sys-misc](../../docs/modules/sys-misc.md#program-flow-and-decisions), [prog-align](../../docs/modules/prog-align.md#program-flow-and-decisions) |
| task 1, QUALITY1/2 | the lunar J22 term only in `src/nav/integration/orbital_integration.c:lunar_j22_term` (word level, exact against yaAGC) | [nav-integ](../../docs/modules/nav-integ.md#program-flow-and-decisions) |
| task 1, SELECTMU | one SELECTMU with the VN1645 display, `src/programs/navigation/rendezvous_subroutines.c:rz_selectmu_display`, used by CSI, TPI and stable orbit; MUTABLE words from nav/conic | [prog-rendz](../../docs/modules/prog-rendz.md#program-flow-and-decisions) |
| task 2, V92 | the CSM REDO is `src/csm/prelaunch/align.c:agc_job_systems_test_csm` (NEWMODEX 07, IMUTEST), the registry's CSM handler of AGC_JOB_SYSTEMS_TEST | [imu](../../docs/modules/imu.md#program-flow-and-decisions) |
| task 2, P20/P22 | the registry dispatches V37 N20 / N22 by vehicle (CSM `agc_job_rendezvous_navigation_csm`, `agc_job_orbital_navigation_csm`; LM `agc_job_rendezvous_navigation_lm` for both) | [prog-rendz](../../docs/modules/prog-rendz.md#program-flow-and-decisions) |
| task 2, SERVEXIT / B5OFF | `src/guidance/servicer_common.c:agc_job_servexit` (both vehicles), `src/io/service/service.c:agc_job_sband_b5off` | [guid-powered](../../docs/modules/guid-powered.md#program-flow-and-decisions), [sys-misc](../../docs/modules/sys-misc.md#program-flow-and-decisions) |
| task 4, splits | REENTRY_CONTROL in `src/csm/entry/entry_{guidance,targeting,initial_roll,huntest,upcontrol,final_phase}.c`; the rendezvous common code in `src/programs/navigation/{rendezvous_erasables,rendezvous_subroutines,state_vector_update}.c`; P07 in `src/lm/systems_test/` | [guid-entry](../../docs/modules/guid-entry.md#program-flow-and-decisions), [prog-rendz](../../docs/modules/prog-rendz.md#program-flow-and-decisions), [sys-misc](../../docs/modules/sys-misc.md#program-flow-and-decisions) |
