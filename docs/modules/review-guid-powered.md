# review-guid-powered -- review of the powered-flight guidance lane

NOTE: a duplicate second run was stopped by the lead at 21:35; run 1 owns
the whole lane.

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
| style: csm/burns/servicer.c | PARTIAL: prologs converted (tools: SCRATCH/review-guid-powered/prolog.py, blocks.py, annot.py); left: decl comments, GRP defines, file prolog, pipas_saturated RETURNS TODO, serv_phase "AGC: none (TODO)" line |
| fidelity: CALCGRAV moon test | IN PROGRESS: CM servicer.c NORMLIZE/AVERAGEG and LM average_g.c CALCRVG/MUNRVG pass MOONFLAG; the AGC tests RTX2 (LXC,1 SLOAD RTX2) -> use agc_rendz_erasable()->body.rtx2 != 0 (R10, not yet edited) |
| style: sps/{burn,clock,ignition,steer}.c, lm/burns/{dps_burn,dps_burn_rasteer}.c, lm/burns/servicer/*.c, lm/burns/ignition/*.c, tests/guidance/guidance.c, include/agc/guidance/guidance.h | TODO (run prolog.py -w, blocks.py, then hand-fix TODO lines + decl comments) |
| rule-5 density warnings (sps/*, dps_burn*, servicer/*, ignition/*) | TODO |
| make / make test / make check | PARTIAL: all touched files compile; the shared tree's lib currently fails on other lanes' WIP (pinball/internal.h "/*" in comment, dap-csm tvc); a private copy (HEAD header patched) passes oracle guidance 203/0, landing 172/0, imu 211/0, tests/guidance 29/0 |

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

* oracle-cases: docs/modules/oracle-cases.md rows s40_1 (tol now 0) and
  s40_2 (tol 16, residual <= 10 LSB) and the F7/F8/F9-TDISPSET findings are
  closed by this review.
* guid-entry: agc_entry_vgamcalc / agc_entry_tff_trig (made public for
  DELRSPL) use agc_dp_mul (DMPR) where VGAMCALC and TFF/TRIG have DMP;
  same R7 class.
* dap-csm: sps/ignition.c adapted to the agc_csm_dp_pair_t ERRBTMP,
  DELPBAR, DELYBAR (R8).

## Lessons

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
