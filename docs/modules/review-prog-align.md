# review-prog-align -- review of the prog-align lane

## Scope

docs/agents/review.md for lane prog-align (docs/modules/prog-align.md):
src/programs/alignment, src/csm/alignment, src/csm/optics, src/lm/alignment,
src/lm/optics, src/lm/ags_initialization.c, src/lm/rendezvous_final_attitude.c,
include/agc/programs/alignment.h, tests/programs/align.c.

## Progress

| item | status |
|---|---|
| R63 UNIT precision (open question) | DONE: AGC uses RTB NORMUNIT; C fixed (A1) |
| N70/N71 AOTCODE orphan components | DONE (A2) |
| realistic-range tests | DONE: R63 300 m..50 km; P52 fine align with 0.5 deg drift, non-identity REFSMMAT (tests/programs/align.c test_realign_realistic_drift) |
| oracle test tests/oracle/align.c + tools/oracle/cases/gen/gen_align.py -> cases/align{,_lm}.spec/.txt (regenerate: cd tools/oracle/cases/gen && python3 -I gen_align.py && cd ../.. && make cases) | DONE: RRNB, RRANGLES (18), SCALEVEC, SXTNB (KNOWN PA-W2), SXTANG (17), LOG, S-band CM (4) bit-exact; S-band LM exact but KNOWN PA-W3 (sband_lm_c pitch 1 LSB) |
| new src/programs/alignment/word_forms.c (MPAC word forms, align_dot_shl, align_dmp_shl) + decls in include/agc/programs/alignment.h | DONE, compiles |
| align_dot_shl / DMP sites: CHKSDATA (locsam.c, done earlier), R59 INCAZ/R59D DOT SL1 x2 + DMP SL4 (lunar_surface_sighting.c), OANB DMP SL1 x2 + SURFSTAR DMP (aotmark.c), SR52 SLOAD DMP MPERIOD (csm auto_optics_positioning.c) | DONE (resume 2) |
| KNOWN PA-W2 (SXTNB SIN SL1) and PA-W3 (LM S-band ABVAL SIGN ASIN): word-level agc_mpac_* | DONE: hard at 0 |
| oracle cases OANB (7), OPTAXIS (7), SURFSTAR+AVEIT (10) in gen_align.py lm() | DONE: exact except KNOWN PA-W4 (1 case, TRG*NBSM words); oracle_align 437 checks |
| P57 gravity-error DOT SL1 ACOS (lunar_surface_gravity.c) | DONE |
| word-level adoption: word_forms.c on agc_dp_words_t / agc_mpac_* (align_words_t and its helpers deleted; ARCTRIG run on words; RRANGLES, SXTANG1 TA, R59D CURSOR/SPIRAL on words) | DONE: RRANGLES 18, SXTANG 17 still exact |
| R59 INCAZ..R59D as agc_lm_aot_find_detent (public, for the oracle) + 14 oracle cases (entry INCAZ basic, stop 79DISP 15,2437 / R59ALM) | DONE: exact; oracle_align 504 checks, 1 known (W4) |
| oracle code1to6 (DODAT/CODE1TO6/OPTAXIS through the job, 12 cases) | DONE: found A12 |
| item 2a: `AGC_ASSERT(st == OK) -> return BAD_ARG/st` sites -> plain status propagation | DONE: 52 by script + 10 by hand (OK-or-BUSY/ALARM/FULL/NOT_FOUND sites, R02BOTH POODOO = job ends after P00, B5OFF, FINDVAC) |
| item 2b: guards that cannot fail (vehicle checks in vehicle-only jobs, ctx != NULL in jobs, constants) and silent recoveries (value patched in the assert branch) | PARTIAL: done for prelaunch_*.c (21 postcondition/module-invariant guards), aotmark.c, geometry.c (SXTANG1/CALCSXA/SXTANG void, CULTFLAG stored in SXTANG1; LOG NORM/ovf guards), 26 vehicle/ctx guards, 6 silent recoveries; OANB/OPTAXIS void. TODO: the rest (programs/alignment, csm+lm alignment, sxtmark, sighting_mark, rendezvous_final_attitude) |
| tree builds again (lead committed c7a3ecb); plain make / make test | -- |
| PAUSED HERE (resume 2, lead usage-limit pause). Left green: make, make test (oracle_align 600 checks 1 known W4, test_align 128), check_rules 0 hard, make format on all 28 edited files | -- |
| NEXT: (a) item 2b remaining guards: sxtmark.c (84 vac-range after claim -> delete; 138 reserve BAD_ARG wrapper -> return sxtmark_common; 170 postcondition; 455 static constant vn; keep qpret/erasable-index checks), sighting_mark.c, programs/alignment/*.c (locsam catlog sentinel 199, 331; sighting 120/157; common 256/366), csm+lm alignment, rendezvous_final_attitude.c; (b) SXTMARK fidelity read: AGC read done up to DIFCHK, C read to MKRELEAS -- compare MARKRUPT/MARKCONT/MARK2/VACSTOR/REJECT2/3/ENTANSWR/MKVB5X/MARKDIF/DIFCHK next; (c) fidelity P51-P53 CM+LM and P57 family; (d) inlined-helper header blocks pasted into bodies (prelaunch_align.c PIPACHK, prelaunch_gyrocompass.c NORMLOP/ALCGKK/ENDGTSAL, prelaunch_verify.c NEXTBNKS/NEXBNKSS+5) -> section comments; (e) style/prologs | -- |
| status-handling-2 diff (1b72892..a02850c) so far: 43 manual.tsv GUARANTEED rows in the lane checked, all hold (planet star code masked to 6 bits, crew-range check kept in locsam); leftovers repaired: dead `st = OK` stores before void calls (lunar_surface_gravity, csm auto_optics), assertion-wrapped callee statuses (62 sites), silent recoveries (6), `if (st != OK) return st; return st;` (B5OFF); inline_one.py header blocks still to rewrite (d) | -- |
| requests (other lanes): agc_csm_sxtnb, agc_lm_rrnb, agc_lm_rrangles now cannot fail -> void, but callers are in src/lm/radar, src/lm/navigation, src/programs/displays, src/io/verbs (not this lane); guidance owner: TRG*NBSM / *NBSM* on agc_dp_words_t (W4) | -- |
| fidelity pass per AGC file | PARTIAL: R63, R47, LEM_GEOMETRY, CSM_GEOMETRY, S-band CM done S-band LM (W3); TODO P51-P53 (CM+LM), SXTMARK, AOTMARK, P57 family |
| style findings to zero (2279 at start: mostly function-prolog boxes/args/returns 365, decl-comment 676) | TODO |
| rule-5 density (14 files < 2.0) | TODO |
| make / make test / make check | tree builds; plain make / make test (resume 2) |

Files touched this session (all compile): src/lm/ags_initialization.c, src/lm/optics/lem_geometry.c, src/csm/optics/geometry.c, src/csm/optics/s_band_antenna.c, src/programs/alignment/word_forms.c (new), include/agc/programs/alignment.h, tests/programs/align.c, tests/oracle/align.c (new), tools/oracle/cases/gen/gen_align.py (new), tools/oracle/cases/align{,_lm}.{spec,txt} (new). Not yet `make format`ted.

## Findings

| id | where | AGC | finding | fix |
|---|---|---|---|---|
| A1 | src/lm/rendezvous_final_attitude.c | R63 V89RECL p.339 `MXV RTB REFSMMAT NORMUNIT` | C called UNIT, not NORMUNIT: 15.7 mrad LOS error at 1 km, 1.8 rad at 100 m | agc_rtb_normunit; test at 300 m..50 km within 1 m/range |
| A2 | src/exec/nouns.c (lane data) | N70/N71, ERASABLE_ASSIGNMENTS (AOTCODE = STARCODE; STARALGN follows) | STARCODE/AOTCODE were read-only (V21N70/V21N71 star-code loads returned NOT_FOUND on both vehicles); LM AOTCODE +1/+2 had no owner | read/write STARCODE; +1/+2 = SINCDUY high/low (guidance block) |
| A3 | src/lm/ags_initialization.c ags_words | SCALEVEC p.209 VAD AGSRND1, VAD AGSRND2, VECSGNAG | a negative rounded word under one LSB gave +0 (AEA reads 0); the AGC's two-step add leaves -0 (77777, AEA -1): oracle scalevec_small | -0 major word when r < 0 and hi == 0; test in align.c and oracle |
| A4 | src/lm/optics/lem_geometry.c RRNB | RRNB1 `COS DMP / SL1`, `SIN DMP / SL1` | SL1 after DMP shifts the TP product (MPAC +2's top bit into the DP); C shifted the truncated DP: 1 LSB off in 4 of 6 components | agc_tp_shift(agc_dp_mul_tp) then agc_tp_to_dp: bit-exact |
| A5 | src/lm/optics/lem_geometry.c RRANGLES, src/csm/optics/geometry.c SXTANG1 | 2V1STO2S / 1STO2S of ASIN, ARCTRIG, ACOS SL2 DSU results | 1TO2SUB reads the major word after DDOUBL; ASINEX (DCS, AD QUARTER) and SUBTR leave sign-disagreeing words, so the AGC rounds such angles up (+,-) or down (-,+) where the value truncates: MODEA/MODEB/SAC/PAC 1 count off in 7 of 10 first cases | new src/programs/alignment/word_forms.c (ASIN/ACOS/ARCTRIG word forms, DAS on words, 1TO2SUB on words); 24 random oracle cases bit-exact |
| W2 | src/csm/optics/geometry.c SXTNB | `SIN SL1` (TSSL shifts MPAC +2 in) | agc_dp_sin returns two words, so sin TA is 1 LSB low half the time; 32D 2 LSB off (sxtnb_a, sxtnb_c) | KNOWN PA-W2 in tests/oracle/align.c (hard at 2 LSB); request to core-numeric: a SIN/COS that returns the TP MPAC |
| C1 | R63 rendezvous_final_attitude.c | V89CALL..ALINEZ p.338-341 | confirmed: responses, OPTIONX, DP1MIN 6000 (rope 00000 13560), CPHI = THETAD, NORMUNIT | -- |
| C2 | R47 ags_initialization.c | AGSINIT..SCALEVEC p.206-210 | confirmed: CKSTALL (wait while IMUCADR != +0), TAT for CSMPREC, constants RSCALE/VSCALE/TSCALE/20SEC/AGSRND1/2 against the rope; SCALEVEC bit-exact on the oracle | A3 |
| C3 | lem_geometry.c | RRANGLES, RRNB, RRNBMPAC p.322-324 | confirmed apart from A4/A5: ASIN(-Y), BDSU LODPHALF, ARCTRIG of the X-Z unit, mode-2 swap on ANTENBIT 04000, BOVB LUNDESCH | A4, A5 |
| C4 | geometry.c | SXTNB, SXTLOGIC, CALCSXA, SXTANG(1), ZNB=S1, LOG, EARROT1/2, OVERADAY p.285-294 | confirmed apart from A5/W2: NB1NB2 = NB2NB1 transposed (rope), 10DEGS- 3600, 20DEGS- -7199 (PAC 61740 on the oracle), CULTFLAG set/cleared as SETGO/CLRGO; LOG bit-exact (4 cases) | A5, W2 |
| W3 | src/lm/optics/s_band_antenna.c (pitch) | `ABVAL SIGN ASIN` | ABVAL leaves MPAC +2 = 20000 and ARCSIN's SQRTSUB includes it (the C ASIN takes a DP): PITCHANG 1 LSB low in sband_lm_c (oracle stop at ARCSIN: MPAC 17657 11171 20000) | KNOWN PA-W3 (hard at 1 LSB); request to core-numeric: ASIN/ACOS of a TP MPAC |
| A6 | src/csm/optics/s_band_antenna.c, geometry.c SXTANG1 | `DOT ... SL1` (R05 yaw/pitch, SXTANG trunnion) | SL1 after DOT shifts the TP dot product; C shifted the DP: RHOSB 1 LSB off (sband_cm_a) | align_dot_shl (word_forms.c): S-band CM bit-exact |
| C5 | s_band_antenna.c (CM) | SBANDANT..NOADJUST p.934-935 | confirmed: EISOI/moon branch (note: the AGC adds LUNPOS REM, 2^29, to moon-sphere RATT, 2^27, unscaled; the C does the same), BOV NOADJUST leaves yaw 0, DPPOSMAX - yaw, display loop GOMARKFR/B5OFF/ENDOFJOB/BLANKET R3/DELAYJOB 1 | A6 |
| C6 | s_band_antenna.c (LM) | SBANDANT..SBANDEX p.486-489 | confirmed: NBSA 45 deg premultiply with DMP 1OVSQRT2, XZ projection, BOV SBANDEX, pitch = ASIN(sign(X.UY)|X|), HIDPHALF - pitch when URP.UZ < 0, yaw = ASIN(sign(YAW.Z)|Z|); 3 of 4 oracle cases bit-exact | W3 |
| A7 | src/lm/optics/aotmark.c OANB | `DMP SL1` (OAY, OAZ) | C did DMPR then shifted the DP: OAZ 1 LSB off in 6 of 7 oracle cases | align_dmp_shl (TP product shifted): OANB, OPTAXIS bit-exact (14 cases) |
| A8 | aotmark.c SURFSTAR | `DMP DP1/12` (SEP) | DMPR used for a DMP | agc_dp_mul_trunc; SURFSTAR oracle (10 cases) |
| A9 | src/lm/alignment/lunar_surface_sighting.c R59 INCAZ | `DOT SL1 ARCCOS` x2, `DMP SL4 3/4` | DOT shifted as a DP; DMPR for DMP | align_dot_shl, align_dmp_shl |
| A10 | src/csm/optics/auto_optics_positioning.c SR52 landmark | `SLOAD DMP MPERIOD` | DMPR for DMP | agc_dp_mul_trunc |
| A11 | src/lm/alignment/lunar_surface_gravity.c (P57 GRAVEL..) | `DOT GSAV SL1 ACOS` | DOT shifted as a DP | align_dot_shl |
| W2-fixed | geometry.c SXTNB | `SIN SL1`, PUSH, `COS DMP 2` | resolved with interp_words.h: SIN as the TP MPAC, SL1, stored words into DMP | hard at 0 (was KNOWN PA-W2) |
| W3-fixed | lm s_band_antenna.c | `ABVAL SIGN ASIN` | resolved: agc_mpac_vsq, agc_mpac_sqrt, agc_mpac_sign, agc_mpac_asin | hard at 0 (was KNOWN PA-W3) |
| W4 | aotmark.c AVEIT after SURFSTAR | `VSR3 V/SC` on TRG*NBSM's vector | *NBSM* (src/guidance/powered_flight.c) returns values; the AGC's VAD can leave a component in sign-disagreeing words, and VSR3 then rounds an exact half toward zero (surfstar_r6: -4262,+1412) | KNOWN PA-W4 (hard at 8 = 1 LSB before V/SC); request to the guidance/imu owner: TRG*NBSM / *NBSM* returning agc_dp_words_t |
| DMP audit | whole lane | every agc_dp_mul (DMPR) site checked against the listing | 6 changed (A7 x2, A8, A9 x1 DMP + 2 DOT, A10); 6 DMPR confirmed (ALWAYSG x3, COMPGS x3) | -- |
| A12 | aotmark.c DODAT/CODE1TO6 | `CA AOTAZ +1, MSU 8D` (field rotation) | MSU delivers the 2's complement difference as the 1's complement word of the same value; the C added 1 to a negative difference: ROT 1 count (0.011 deg) off for detents 3-5, XPNB/YPNB ~9000 LSB off | fixed; oracle code1to6 (12 cases: entry CODE1TO6, stop GETMKS) exact |
| D1 | aotmark.c DODAT | `MASK HIGH9, MP BIT9` | AGC keeps AOTCODE bits 7-15 as the detent (a code > 7 indexes past AOTAZ/AOTEL) | DEVIATION comment: C keeps bits 7-9 |
