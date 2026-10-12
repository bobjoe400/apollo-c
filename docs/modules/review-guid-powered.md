# review-guid-powered -- review of the powered-flight guidance lane

STATUS: DONE (resume 2, 2026-10-12). Open: the requests below.

## Scope

Lane files (docs/modules/guid-powered.md, after restructure):
src/guidance/{powered_flight,servicer_common,external_delta_v,burn_steering,guid_state,guid_nav_glue}.c,
src/csm/burns/{servicer,external_delta_v}.c, src/csm/burns/sps/*.c,
src/lm/burns/servicer/*.c, src/lm/burns/dps_burn*.c, src/lm/burns/ignition/*.c,
include/agc/guidance/{guidance,pending}.h, tests/guidance/, tests/oracle/guidance.c.

## Progress

| item | status |
|---|---|
| F7 CM J2 constants (servicer_common.c) | DONE run1: words from rope 37,3464/3466 (B1 = x2); CALCGRAV DMPs truncated (agc_dp_mul_trunc); KNOWN flipped, CALCRVG cases bit-exact |
| F8 S40.2,3 UNIT(RTIG) (burn_steering.c) | DONE run1: SPS path crosses with UNIT(RTIG); DMP/SL1 on the TP (dmp_sl1); tol 128 -> 16 (residual <= 12 LSB, UNIT of a short UT x R in the radial case) |
| F9 at TDISPSET | DONE run1: TTF/8 raw high word carried through ROOTPSRS / TTFINCR DAS (ldg_das, agc_lm_ttf8_high); TTFDISP bit-exact (tol 8 -> 0); also QUADGUID, CGCALC, EXTLOGIC, HIGATCHK read it |
| DELRSPL translate | DONE (src/csm/burns/external_delta_v.c, AUGEKUGL tables from rope 32,2201-2257); tests/guidance 3rd test (16 checks) |
| ZOOM/P40ZOOMA | DONE: as flown, confirmed from the rope (see Open questions) |
| octal-written table nouns grep | DONE: none left (BURNBABY tables 74/40/40/62/63 decimal; all agc_dsky_vn args decimal) |
| fidelity: constants vs rope | DONE: 72 lane constants read from the ropes, all match |
| fidelity: DMP vs DMPR | DONE: 25 agc_dp_mul (DMPR) sites checked; 22 were DMP -> agc_dp_mul_trunc / TP (R7); S40.1 oracle now bit-exact (tol 64 -> 0) |
| fidelity: label coverage | DONE: every label of the 9 lane AGC files is named in the C or is data/internal (list in Findings R9) |
| style: guid_state.c, servicer_common.c, burn_steering.c, powered_flight.c, guidance/external_delta_v.c, csm/burns/external_delta_v.c | DONE (0 check_style findings each) |
| fidelity: CALCGRAV moon test (R10) | DONE: CM NORMLIZE/AVERAGEG and LM CALCRVG/NORMLIZE pass RTX2 != 0 (CM LXC,1 SLOAD X1 BMN ITISMOON; LM SLOAD RTX2 DCOMP BMN CALCGRV1); MUNFLAG kept only where the AGC tests MUNFLAG (RVBOTH, MUNRVG choice) |
| *NBSM* word-level form (prog-align request) | DONE (R11): AX*SR*T on VBUF words (agc_guid_axisrot_words, agc_vec_words_t in guidance.h); value form = the same code on sign-agreeing words; agc_guid_trg_nbsm_words; agc_guid_trg_nbsm made void (cannot fail; callers in aotmark.c, measurement_update.c, lunar_surface_gravity.c, sighting_mark.c updated); SURFSTAR/AVEIT carry the words (VSR3 = SRR3 on the stored words); oracle KNOWN PA-W4 flipped to hard at tol 0, oracle_align 570/0, all oracles unchanged |
| dead guards (rule 5/7 spirit) | DONE: 70 by-construction guards deleted (ctx->id != NONE, vehicle checks in vehicle-only code, NULL locals) + ~100 tautological post-asserts (a flag/value/channel bit tested right after being set, table constants, switch-case values) and AGC-unchecked state asserts that returned BAD_ARG where the AGC goes on (DAS DVTOTAL overflow, NVWORD1, PIPCTR); kept: erasable/pad-load/sensor boundaries (ZOOMTIME, WEIGHT/G, EMDOT, PIPAGE, RTX2, AVGEXIT dispatch, HMEAS, SQRT/DDV operands). Lane density 302 -> 131 assertions / 423 functions (report only). 43 jobs/helpers lost an unused ctx (now (void)); dispatch regenerated (tools/gen_registry.py; jobs.h ids unchanged) -- callers outside the lane touched: csm/burns/earth_orbit_insertion.c (MATRXJOB, NORMLIZE call), csm/entry/cm_body_attitude.c (SERVNOUT, SERVEXIT call), include/agc/csm/entry.h (2 decls) |
| style: sps/*, lm/burns/**, csm/burns/servicer.c, guidance/*, guidance.h, tests | DONE: 1276 -> 0 check_style findings in the 23 lane files: function prologs boxed (AGC citation, RETURNS, args), file prologs to the template (12 .c, 3 internal.h, guidance.h, 2 tests), declaration/scale/#define/ternary comments, long-block closers, qualifier and one-declaration fixes, tests main-first with prototypes; prolog page citations corrected to the listing page markers (R14); PRIO23_LR alias and two forwarding wrappers removed (agc_job_burnbaby, agc_lm_burnbaby_countdown; registry: AGC_JOB_BURNBABY -> agc_lm_burnbaby, AGC_JOB_BURNBABY_COUNTDOWN -> the routine itself; dispatch regenerated). Scripts: SCRATCH/review-guid-powered/{prolog,fill,fileprolog*,autodecl,applydecl,applyscale,fixpages}.py |
| status-handling-2 diff check (git diff 1b72892 a02850c) | DONE (R12): 21 files, 1673 deletions read. Deleted guards (NULL ctx/ip/out pointers, enum ranges, post-hoc sentinels) all by construction: confirmed. Status conversions (agc_sv_get void, agc_dp_sin/cos/sqrt without ip, agc_phase_change(code), ctx-less jobs) confirmed. Repaired: helpers with their own AGC label that the collapse inlined (CALCTHET, PIPCHECK, P41/DSP, S40.81, LOTHRUST, SETUP.9, DVMON, HIGATASK, P63IGN, P40IGN) restored as static functions, pasted header boxes gone; NULLCLOK relinted only on a failed assertion (bug: removed the tautological guard); the two AGC_ASSERT(st == OK) sites now plain propagation; tautological post-asserts deleted in the restored code; single-return S40.1, UPDATEVG, COPYCYCL, COPYCYC1, P40SXTY |
| word-level adoption in servicers / steering | DONE (R13): S40.2,3 bell axes on words (DMP SL1 / DCOMP / COS / SIN as agc_mpac_*; SCAXIS exact); oracle s40_2 tol 16 -> 2 (sps, sps_b; residual = VXM of the stored VDEF words, needs a DOT on stored words: core request) and 10 (radial: UNIT of a short UT x UNIT(R), core UNIT-at-B27 item); FLESHPOT on words (AX*SR*T helpers; value helpers dmpsub/das/ddoubl deleted). CALCRVG/CALCGRAV, S40.1, DELTIME already exact; LM servicer has no word-split residual on record |
| fidelity: SR/SRR/VSR audit (ALL-lanes item) | DONE (R15): every rounding shift of the lane listings (CM SRR in CALCUT, VSR* in SETUP.9, VSR3 in BADVCTOR; LM VSR4 in COPYCYC2, SLR* in CALCGRV1, VSR1 in AVERAGEG and RASTEER1/GETVRVG2/NEGPROD) matches its C (shr_round, shl for the negative VSR* count, vec_shr, shl_round); SR after DMP in the S40.13 F/MASS terms truncates the DP, the same as the TP for sign-agreeing words |
| make / make test / make scenarios / make check / check_rules | DONE: make OK, make test 0 failures (34 binaries), scenarios 8/8, check_rules src include 0 hard, format-check clean on the lane files (make check's format-check fails only on review-nav-integ's in-progress src/nav/integration/orbital_integration.c), gen_registry --check clean, one-liners PLAIN 0 |

## Findings

| # | where | AGC | finding | action |
|---|---|---|---|---|
| R1 | servicer_common.c J20_CM/J2_CM | SERVICER207 p.836 20J, 2J `2DEC* ... B1*` | constants were 1/4 of the rope (B1 read as B-1) -> CM GOBL1/2 4x small (F7) | fixed from rope words 17431770, 1743177 |
| R2 | servicer_common.c CALCGRAV | DMP DMP 20J / DMP DMP 2J 32D | rounded DMPR used for DMP -> GOBL1/2 1 LSB | agc_dp_mul_trunc |
| R3 | burn_steering.c S40.2,3 SPS | p.716 PDVL UNIT RTIG / VXV VCOMP | crossed UT with raw RTIG (F8) | UNIT(RTIG) for SPS; RCS (S40.2,3B) keeps raw RTIG as coded |
| R4 | burn_steering.c S40.2,3 bell axes | p.715 DMP SL1 | DMP rounded then shifted; AGC shifts the TP | dmp_sl1 (TP shift) |
| R5 | src/lm/landing/guidance.c TDISPSET (+ROOTPSRS) | p.825 CA TTF/8 | ROOTPSRS's DAS ROOTPS leaves TTF/8 sign-disagreeing; CA reads the raw high word (F9) | raw high word tracked (ttf_8_hi) |
| R6 | external_delta_v.c DELRSPL | pp.643-647 | stub | translated (CM file); VGAMCALC/TFF/TRIG made public in src/csm/entry/programs.c (agc_entry_vgamcalc, agc_entry_tff_trig); SR30.1 keeps its TFF debris for it (orbit_parameter_display.c) |
| R7 | burn_steering.c (CALCTHET, TGOCALC, TGDCALC, S40.13), powered_flight.c SMCDURES, csm/burns/servicer.c DVTOTUP, sps/steer.c, lm/burns/servicer/*.c | interpretive DMP | agc_dp_mul is DMPR (rounded); DMP truncates, and NORM/SL/SR after a DMP act on the TP | agc_dp_mul_trunc, TP shifts; CALCTHET DMPR THETACON and LM DMPR ALTCONV/ARCONV1 kept rounded |
| R8 | src/csm/burns/sps/ignition.c DOTVCON, MASSBACK | DCS ERRORY; CAE DELPBAR | build break after dap-csm made ERRBTMP/DELPBAR word pairs | each word complemented; high word read |
| R10 | csm/burns/servicer.c NORMLIZE, AVERAGEG; lm/burns/servicer/average_g.c CALCRVG, NORMLIZE | CALCGRAV `LXC,1 SLOAD RTX2` / `SLOAD RTX2 DCOMP BMN` | moon test read MOONFLAG; the AGC tests the integrator's RTX2 | RTX2 != 0 |
| R11 | powered_flight.c AX*SR*T; aotmark.c SURFSTAR/AVEIT (prog-align W4) | DMPSUB, DAS TERM1TMP, DDOUBL; STCALL 24D; VSR3 | components left in sign-disagreeing words were collapsed to values, so AVEIT's VSR3 rounded an exact half the other way | word-level AX*SR*T; SURFSTAR/AVEIT keep the words; PA-W4 hard |
| R12 | status-handling-2 diff (lane files) | -- | inlined labelled helpers, NULLCLOK relint-on-failure, assert-on-status | see Progress row |
| R13 | burn_steering.c S40.2,3 bell axes; powered_flight.c FLESHPOT | DMP SL1 on the TP, VDEF/PUSH keep MPAC, MPAC+1; basic DMP/DDOUBL/DAS | value collapse lost the split (SCAXIS 2 LSB) | word API; tolerances 16 -> 2 / 10 |
| R14 | sps/*.c, lm/burns/dps_burn.c, ignition/burnbaby.c prologs | # Page markers | page citations 4-9 pages ahead (CM sps), +-1..3 (LM) | first-line citations corrected (78); a few body references keep the old numbering (noted in burn.c prolog) |
| R15 | shift audit | SRR / VSR* / SLR* / VSRn | all rounding shifts match | none |
| R9 | coverage | P30-P37 AVFLAGA/P, P20FLGON | implemented in prog-rendz (rz_avflag) / P30 flag set-up; S41.2/BDFAIL/TOGETHER are dap-csm's; MDOTFAIL is unreferenced in Comanche | none |

## Open questions (resolved)

* ZOOM / P40ZOOMA (lane note; exec review routed item): the rope gives ZOOM
  and P40ZOOMA the same address (17,2022); 3.3SPOT is 76355 75755 41710
  (-GENADR ZOOMTIME, -2CADR 17,2022). So the 3.3 restart reschedules only
  P40ZOOMA (PHASCHNG 3 off), and COMFAIL2's KILLTASK ZOOM removes only a
  task scheduled under that address: a restart between ignition and
  ZOOMTIME loses the throttle-up (P40ZOOM / P63ZOOM), as flown in
  Luminary 099. The C (restart_tables.c 3.3 -> AGC_TASK_ZOOM,
  clock.c KILLTASK AGC_TASK_ZOOM, burnbaby.c agc_task_zoom) matches; no
  change. Exec review: nothing to do in restart_tables.c.
* ABRTABLE (5)-(7) NOOPs: P70/P71 never enter BURNBABY (left as is).

## Requests to other lanes

* core-numeric: (1) a DOT / VXM / MXV on stored words (agc_vec_words_t
  inputs) -- the last 2 LSB of S40.2,3 XSCREF..ZSCREF come from the VXM of
  the VDEF words; (2) move agc_vec_words_t (now in guidance.h, used by
  AX*SR*T, SURFSTAR, AVEIT) to include/agc/core/word.h; (3) the s40_2_radial
  10 LSB is the UNIT-at-B27 item for a short UT x UNIT(R).
* prog-align (done lane, edited for the word form): aotmark.c SURFSTAR now
  returns VBUF words, AVEIT takes the words (VSR3 = SRR3 per component),
  FLIGHT LOS converts its value at STORE 24D; alignment.h decls updated;
  KNOWN PA-W4 flipped to a hard check at tol 0 (oracle_align 570/0). Also
  agc_aot_flight_los / agc_aot_surfstar can no longer fail after TRG*NBSM
  became void (they still return AGC_STATUS_OK for their BAD_ARG guards).
* guid-entry / nav lanes (call sites only): agc_guid_trg_nbsm is void
  (lm/navigation/measurement_update.c, lm/alignment/lunar_surface_gravity.c,
  csm/optics/sighting_mark.c); agc_job_normlize_csm, agc_job_servexit,
  agc_job_matrxjob and entry_servnout take no ctx (earth_orbit_insertion.c,
  cm_body_attitude.c, entry.h). measurement_update.c still carries a pasted
  MXMYMZ header box inside a case body (document lane item).
* oracle-cases: docs/modules/oracle-cases.md s40_2 rows: tolerances now 2
  (sps, sps_b) and 10 (radial).

* oracle-cases: docs/modules/oracle-cases.md rows s40_1 (tol now 0) and
  s40_2 (tol 16, residual <= 10 LSB) and the F7/F8/F9-TDISPSET findings are
  closed by this review.
* guid-entry: agc_entry_vgamcalc / agc_entry_tff_trig (made public for
  DELRSPL) use agc_dp_mul (DMPR) where VGAMCALC and TFF/TRIG have DMP;
  same R7 class.
* dap-csm: sps/ignition.c adapted to the agc_csm_dp_pair_t ERRBTMP,
  DELPBAR, DELYBAR (R8).

## Lessons

* A basic-language routine that ends in DAS/DDOUBL (AX*SR*T, FLESHPOT)
  leaves words whose signs disagree; the next pass multiplies those words
  as stored. Run the whole routine on agc_dp_words_t and collapse to a
  value only at the API boundary: the value form is then the same code.
* VSR on a stored vector rounds per component on the words (MPACSRND with
  MPAC+2 = 0): agc_mpac_shift_round(ip, agc_mpac_dload(w), -n).
* The collapse pass inlined single-caller helpers even when their header
  named their own AGC label; restore those (CALCTHET, DVMON, ...) rather
  than keep a pasted header box inside the caller.
* Prolog page numbers drift: check each first-line citation against the
  listing's "# Page" marker of the label (fixpages.py); the CM P40-P47
  file was 4-9 pages ahead.

* "B1" in a 2DEC* means the stored value is TWICE the number (B-n halves).
  The F7 constants were read as B-1. Always read the word from the rope.
* agc_dp_mul is DMPR. Interpretive DMP truncates and leaves the third
  word in MPAC+2, which NORM, SL, SR (and SIN/COS via DAD not touching it)
  see. Check every multiply against the listing's opcode.
* DAS never re-signs a DP: a later CA of the high word can read hi+-1 of
  the canonical split (TTF/8, F9). Carry the raw high word where a CA/AD/DV
  of it follows.
* A rope reader is 20 lines of Python (whatis via agc_probe + .bin offset,
  banks 02/03 first); use it for every constant, table and alias question.
