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
| style: guid_state.c, servicer_common.c, burn_steering.c | DONE (0 findings) |
| style: remaining lane files | IN PROGRESS |

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

## Lessons
