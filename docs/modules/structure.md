# structure -- duplicates, cross-vehicle placement, uncalled labels, splits, internal.h

History of this lane: [history/lanes/structure.md](../../history/lanes/structure.md)

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

## Design: task 1, the 16 groups

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

## Design: task 2, the 5 cross-vehicle routines

| # | Routine | Finding at HEAD | Action |
|---|---|---|---|
| 1 | SERVEXIT agc_job_servexit (csm/burns/servicer.c) | shared, LM calls it | move to guidance/servicer_common.c with PH_5_3_A |
| 2 | IMUTEST | REDO (V92) lives in lm/systems_test.c with a CSM arm calling the CSM's IMUTEST; registry CSM column is '-' so the CSM V92 job is never dispatched (NOT_FOUND) | the CSM REDO (NEWMODEX 07, fall into IMUTEST) goes to csm/prelaunch beside IMUTEST and into the registry's CSM column; the LM REDO loses the CSM arm |
| 3 | PREREAD (agc_task_precheck, csm/burns/sps/burn.c) | PRECHECK is CSM-only in the registry; the LM arm of its vehicle switch is unreachable | call agc_task_preread_csm |
| 4 | P20/P22 entry (csm/navigation/navigation.c) | vehicle switch calls lm/navigation; registry LM column '-' so LM P20/P22 (V37E20E, E22E) are not dispatched | registry dispatches by vehicle; vehicle-switch functions removed |
| 5 | B5OFF agc_job_sband_b5off (csm/optics) | registry handler of both vehicles | move to io/service/service.c |

## Program flow and decisions

_Final documentation pass (2026-10-14). This lane changed where code
lives and wired two dispatch paths; the decisions themselves are in the
owning lanes' sections._

| change | decision as the code now takes it | lane section |
|---|---|---|
| task 1, 1STO2S | `src/io/service/rtb_ops.c`: one 1TO2SUB on the MPAC words for every caller (`agc_rtb_1sto2s`, `agc_rtb_1sto2s_words`, V1STO2S, 2V1STO2S); `src/lm/optics/lem_geometry.c:agc_lm_rrangles` and `src/programs/alignment/geometry_csm.c:agc_csm_sxtang` call the RTB ops as the listing does | [sys-misc](sys-misc.md#program-flow-and-decisions), [prog-align](prog-align.md#program-flow-and-decisions) |
| task 1, QUALITY1/2 | the lunar J22 term only in `src/nav/integration/orbital_integration.c:lunar_j22_term` (word level, exact against yaAGC) | [nav-integ](nav-integ.md#program-flow-and-decisions) |
| task 1, SELECTMU | one SELECTMU with the VN1645 display, `src/programs/navigation/rendezvous_subroutines.c:rz_selectmu_display`, used by CSI, TPI and stable orbit; MUTABLE words from nav/conic | [prog-rendz](prog-rendz.md#program-flow-and-decisions) |
| task 2, V92 | the CSM REDO is `src/csm/prelaunch/align.c:agc_job_systems_test_csm` (NEWMODEX 07, IMUTEST), the registry's CSM handler of AGC_JOB_SYSTEMS_TEST | [imu](imu.md#program-flow-and-decisions) |
| task 2, P20/P22 | the registry dispatches V37 N20 / N22 by vehicle (CSM `agc_job_rendezvous_navigation_csm`, `agc_job_orbital_navigation_csm`; LM `agc_job_rendezvous_navigation_lm` for both) | [prog-rendz](prog-rendz.md#program-flow-and-decisions) |
| task 2, SERVEXIT / B5OFF | `src/guidance/servicer_common.c:agc_job_servexit` (both vehicles), `src/io/service/service.c:agc_job_sband_b5off` | [guid-powered](guid-powered.md#program-flow-and-decisions), [sys-misc](sys-misc.md#program-flow-and-decisions) |
| task 4, splits | REENTRY_CONTROL in `src/csm/entry/entry_{guidance,targeting,initial_roll,huntest,upcontrol,final_phase}.c`; the rendezvous common code in `src/programs/navigation/{rendezvous_erasables,rendezvous_subroutines,state_vector_update}.c`; P07 in `src/lm/systems_test/` | [guid-entry](guid-entry.md#program-flow-and-decisions), [prog-rendz](prog-rendz.md#program-flow-and-decisions), [sys-misc](sys-misc.md#program-flow-and-decisions) |
