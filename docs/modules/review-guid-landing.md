# review-guid-landing -- review of the guid-landing lane

Scope: src/lm/landing/{throttle,guidance,lunar_landing,aborts,displays,findcduw,state}.c,
src/lm/ascent.c, include/agc/lm/landing.h, tests/lm/landing.c,
tests/oracle/landing.c. AGC: Luminary099 pp. 785-856, 898-925.

## Progress

(resumed 2026-10-10: no earlier note on disk; the previous session's ROOTPSRS/TDISPSET work was in commit 4f35a43)

- DONE THROTTLE pp. 793-797 line by line: throttle.c rewritten (word pairs agc_lm_words_t, ones'-complement clock differences, MASSMULT overflow L = 02410, prologs); oracle throttle KNOWN F9 x12 flipped to hard checks
- DONE ROOTPSRS/TDISPSET F9: ttf_8_hi tracking + word-level DMPSUB/POWRSERS (guidance.c ldg_dmpsub, ldg_powrsers), DXCRIT from guess words, value stop test; TDISPSET TREDES idiom verified; oracle landing 172 checks, 0 failures, 0 KNOWN
- DONE state.c: agc_lm_words / agc_lm_words_value / agc_lm_das / agc_lm_das_ovf added (declared in include/agc/lm/landing.h); prologs; 0 style findings
- DONE aborts.c vs pp. 829-836 (R10,R11, R10,R11A, P70/P71, ABRTJASK, GOABORT, P70INIT, BOTHPOLY, UPTHROT, P71RET, OLDTIME, LEGAL?, TGOCOMP, THROTUP): confirmed; PIPCTR assertion; ternaries removed; 0 style findings
- DONE LANDING_TEST_FAKE_THRUST: gone from code; guid-landing.md Tests/Open questions/Requested core additions brought up to date
- DONE R10/R11 naming conforms to NAMING.md (abort_monitor / abort_monitor_resume / LANDISP); headers reworded
- DONE digit-suffixed names: 7 job/task ids + 16 functions renamed (docs/history/RENAMES.md "review-guid-landing"); registry.tsv, include/agc/exec/jobs.h, src/exec/dispatch.c, src/lm/burns/dps_burn.c updated by word-boundary replace; gen_registry --check clean
- DONE tests/lm/landing.c (TTHROT as a valid TIME1 word; word-pair fields), tests/oracle/landing.c (word-pair inputs/compares, F9 KNOWNs now hard + high-word checks)
- DONE DMP vs DMPR / shift audit (review_items "ALL remaining lanes"), all lane files vs LUNAR_LANDING_GUIDANCE_EQUATIONS, THE_LUNAR_LANDING, ASCENT_GUIDANCE, P12, P70-P71, FINDCDUW, THROTTLE, LANDING_ANALOG_DISPLAYS: no DMP/DMPR mislabels (DMPR only in DDUMCALC, as the AGC); SR/SRR/VSR roundings match. 15 sites changed where the AGC shifts/square-roots the TP MPAC (DOT SL, DMP [DAD] SL, DSQ [DSU] SQRT, SQRT SL1) or reads a sign-disagreeing word pair (FWEIGHT DMP BIT1H): lunar_landing.c 3, ascent.c 10, guidance.c 1 (AFCCALC3), findcduw.c 1 (NB2CDUSP). Scratch scanner: review-guid-landing/tpscan.py
- DONE oracle: new cases ascent_a/b (ATMAG->ASCENT->P12RET, stop PHASCHNG) and rodcomp_a/b (RODCOMP to THROTTLE +3) in gen_landing.py/landing.spec/.txt + tests/oracle/landing.c; findcduw tol 1 -> 0 (CPHI compared as raw two's-complement words; the 1 LSB was the test reading them as ones' complement). oracle_landing 270 checks, 0 failures. RODCOMP exact; ascent exact through TGO, rest within documented F9-class tolerances (sign-disagreeing TBUP/TGO; proven by running the rope from T2TEST on canonical words = C exactly)
- DONE displays.c vs pp. 898-907 line by line (ARCOMP, DATAOUT, ALTOUT/OLDDATA/NEWDATA sign-agreement idiom, DISINDAT, SPEEDRUN, LATFWDV, VMONITOR all branch paths, DISPRSET/ABORTON): confirmed; ld_sat -> ld_wrap (TS/XCH of an overflowed A stores the overflow-corrected word, not POSMAX); agc_lm_ad now wraps the same way; 0 style findings; density 2.0 (32/16)
- DONE rule-5 density >= 2.0 in aborts.c (40/20), displays.c (32/16), state.c (32/16), throttle.c (18/9): combined `!A || !B` asserts split, real pre/postconditions added (MODREG after ABRTJASK, OPR ERR after FALTON, THRUST after THROTUP, IMODES33 bits, ALTM word, TRAK range, ...)
- DONE findcduw.c style: template file prolog + function prologs, 0 style findings, density 2.0 (42/21); fc_mp_dbl stores overflow-corrected (was saturating); nested abs ternaries -> abs(); fidelity: oracle findcduw_a/b exact (CPHI) through NB2CDUSP/ARCTRGSP; the PNGCS-auto tail (limits, rates, CDUWXFR) is not reachable in the probe (channels 30/31 cannot be set) -- reviewed by reading only
- DONE guidance.c style: template file prolog + all function prologs, 0 style findings, density >= 2; ldg_sat -> ldg_wrap (TS stores overflow-corrected)
- DONE lunar_landing.c: fidelity of IGNALG/IGNALOOP/?GUIDSUB pass count/NIGNLOOP, GUILDENSTERN (R13) all branches, STRTP66A/VRTSTART/STARTP67, REDESIG/REDES1, PITFALL/REDESMON/COUNT'EM, DESCBITS, LANDJUNK vs the listing: confirmed; RODCOMP oracle-exact; nested AFCSPOT ternary unrolled; RODCOMP first part split out (ll_rod_pipas, rule 4); 0 style findings; density 2.0
- DONE ascent.c: fidelity of MAINLINE/ANG1CHEK/OFFROT, CHECKALT/EXITVR/CHECKYAW/KEEPVR, ENGOFF (TPAGREE/CCS ENGOFFDT), ENGOFF1, CUTOFF..TERMASC vs the listing: confirmed; RATES..PROK split out (as_rates, rule 4 after the TP edits); template prologs, 0 style findings, density >= 2
- DONE style debt: include/agc/lm/landing.h (header prolog, field scales; unused rdotv removed), tests/lm/landing.c and tests/oracle/landing.c (template prologs, main first, decl comments). 0 style findings in all 11 lane files; tools/check_rules.py 0 hard / 0 warnings on src/lm/landing and src/lm/ascent.c; uncrustify --check clean
- Build state (2026-10-11): `make` and `make test` green (test_landing 17/0, oracle_landing 270 checks / 0 failures / 0 KNOWN). `make check` fails only on other lanes' files (src/lm/dap/accont.c rule 4, src/csm/dap/tvc/daps.c rule 7)
- Scratch tools (review-guid-landing/): prolog.py, annot.py, braces.py, longfix.py, movecom.py, comabove.py, fileprolog.py, acount.py (assert density per function), tpscan.py (TP-consumer scanner), snip.py + yul/asm.sh (run assembled interpretive snippets in the probe), stage/ (stage-by-stage C harness)

## Findings

| # | where | AGC | finding | action |
|---|---|---|---|---|
| 1 | throttle.c DOPIF/DOIT/FWEIGHT | THROTTLE p. 795 | F9: PIF = FCODD - FP by DAS left sign-disagreeing; CA PIF / MP PIF read the high word alone (THRUST 1 pulse, FWEIGHT up to 1.4 pulses) | FP, FCODD, PIF, FWEIGHT kept as word pairs (agc_lm_words_t, agc_lm_das); oracle KNOWN F9 x12 flipped to hard checks (+ high-word checks) |
| 2 | throttle.c AFDUMP | p. 793 | FP after DAS FWEIGHT is sign-disagreeing too; region logic (AD FP, CA FP) read the value's high word | uses fp.hi |
| 3 | throttle.c time tests | pp. 793, 795 | TIME1 - TTHROT and TIME1 - PIPTIME+1 + THROTLAG taken in two's complement; the AGC's CS/AD/MASK gives -0 (all ones) for equal times and one less after a TIME1 wrap | thr_csad models the 16-bit ones'-complement A register |
| 4 | throttle.c MASSMULT | p. 796 | overflow low word "whatever L held" is in fact TPAGREE's return address 02410 (probe-verified) | exact constant THR_MASSMULT_L |
| 5 | throttle.c FLATOUT1/LOWFCOLD | p. 794 | FCODD low word = L = BUF+1 (H*GHCR*T = BUF+1 shares scratch BUF with ROOTPSRS etc.); not determinable in the lane model | DEVIATION kept (0), reworded; affects PIF low word only |
| 6 | guidance.c ROOTPSRS | pp. 823-825 | POWRSERS ran on the canonical value of ROOTPS; the AGC multiplies the stored (sign-disagreeing) words: TTF/8 202 LSB in guid_brak_b | word-level DMPSUB/POWRSERS (ldg_dmpsub, ldg_powrsers); exact |
| 7 | guidance.c ROOTPSRS stop test | p. 825 | previous session compared high words; AGC CCS/TESTLODX on ABS(DX)-DXCRIT is a value test | value test restored |
| 8 | guidance.c QUADGUID | pp. 810-811 | UNFC/2 <= 60, /AFC/ <= 81 LSB: stale push-list low words | documented DEVIATION kept (oracle tol 128) |
| 9 | aborts.c R10,R11 | p. 829 | CCS PIPCTR < 0 path stores |PIPCTR|-1 (C went to LRHTASK); PIPCTR never negative | assertion added |
| 10 | throttle.c | p. 796 | MASSMULT, WHATOUT only called inside THROTTLE_CONTROL_ROUTINES | made static (thr_massmult, thr_whatout), removed from landing.h |
| 11 | lunar_landing.c RODCOMP, DDUMGOOD; ascent.c ASCENT, YCOMP, NEWLOAD, P12RET | pp. 788, 817, 840, 845, 854 | DOT then SL1/SL2: DOT leaves a TP and the interpretive SL shifts MPAC+2 into MPAC+1; C shifted the truncated DP (HDOTDISP 1 LSB, ZDOT/YDOT/Y/RDOT/OUTOFPLN/pitch argument) | agc_vec_dot_tp + agc_tp_shift (6 sites); oracle rodcomp/ascent exact |
| 12 | ascent.c ATMAG, RATES, CHKBMAG | pp. 843, 847-848 | DMP [DAD] SL: DAD keeps the DMP's MPAC+2, SL takes it in (TBUP, AT, PRATE, YRATE) | agc_dp_mul_tp / agc_tp_add / agc_tp_shift (4 sites) |
| 13 | ascent.c P12RET; guidance.c AFCCALC3; findcduw.c NB2CDUSP | pp. 809, 840, 920 | DSQ [DAD/DSU/BDSU] SQRT: SQRT reads MPAC+2 (BDSU complements MPAC,+1 only, so the low word keeps its sign); P12RET SQRT SL1 takes back the bit SQRT's unnormalising shift left in MPAC+2 | agc_tp_sqrt of the TP; SQRT SL1 = agc_tp_sqrt(4 s) (same normalised root, one shift fewer; s < 1/8 asserted) |
| 14 | lunar_landing.c RODCOMP | p. 818 | DLOAD FWEIGHT, DMP BIT1H with FWEIGHT sign-disagreeing (THROTTLE's DAS): the product's top words are (0, FWEIGHT hi); C multiplied the value (/AFC/ 148 LSB) | fw = (0, fweight.hi); oracle rodcomp exact |
| 15 | ascent.c MAINENG, ATMAG; aborts.c YOK | pp. 834, 843, 846 | DSU leaves TBUP, TGO (and XRANGE's operand) sign-disagreeing; SR 11D shifts each word (TTOGO -1 cs) and GENDDV divides the words (AT 3, PRATE..ATY <= 20 LSB) | DEVIATION (agc_dp_t holds the value only); proven by running the rope from T2TEST with canonical words = C exactly; oracle tolerances AT 4, TTOGO 1, PRATE/YRATE 32, PCONS/YCONS 4, ATR..PITCH, UNFC/2 8. Core request R1 |
| 16 | displays.c, guidance.c QUADGUID, findcduw.c, state.c agc_lm_ad | pp. 808, 898-907, 916-917 | TS/XCH of an overflowed A stores the overflow-corrected (wrapped) word; C saturated at POSMAX (review item "do not saturate") | ld_wrap, ldg_wrap, fc_mp_dbl, agc_lm_ad wrap modulo 2^14 (asserted; never reached on valid data) |
| 17 | tests/oracle/landing.c findcduw | p. 920 | CPHI..CPSI are two's-complement words; the test read them as ones' complement (the 1 LSB tolerance) | compare raw 15-bit words, tolerance 0 |
| 18 | exec/nouns.c AGC_DATA_RDOT (not this lane) | P12 p. 840 | RDOT = HDOTDISP in the AGC; the noun table points at agc_lm_asc()->rdot, which nothing writes | request R2 to the exec/nouns owner |

## Requested core additions / requests to other lanes

- R1 (core-numeric): DDV and SR on stored word pairs, e.g. `agc_dp_t agc_dp_div_words(agc_interp_t *ip, agc_sp_t hi, agc_sp_t lo, agc_dp_t divisor)` (or an agc_dp_t variant that keeps the representation), so ATMAG/MAINENG/T2TEST can divide the sign-disagreeing TBUP/TGO as GENDDV does (finding 15).
- R2 (exec/nouns, src/exec/nouns.c:155): AGC_DATA_RDOT should return &agc_guid_state()->lm.hdotdisp (RDOT = HDOTDISP, ERASABLE_ASSIGNMENTS); agc_lm_asc_t.rdot can then be removed.

## Oracle cases added (tools/oracle/cases/gen/gen_landing.py)

| case | entry / stop | inputs -> compared |
|---|---|---|
| ascent_a, ascent_b | INTPRET, Q = 34,3645 (ATMAG's first interpretive word) -> stop PHASCHNG (the TC after P12RET's EXIT); FLAGWRD9 = FLPI | ABDVCONV MASS1 1/DV1..3 (E7,1634/1636/1640: the names start with a digit, give the address) VE TBUP TTO R V UNIT/R/ GDT1/2 QAXIS ZAXIS1 RCO YCO TGO ZDOTD YDOTD RDOTD HDOTDISP XNBPIP -> TBUP AT 1/DV0..3 /R/MAG ZDOT YDOT Y GEFF LAXIS DZDOT DYDOT DRDOT VGVECT VGBODY TGO TTOGO PRATE YRATE PCONS YCONS ATR ATY ATP UNFC/2 YAW PITCH FLPC |
| rodcomp_a, rodcomp_b | RODCOMP 32,3275 (EBANK 7) -> stop 31,2221 (THROTTLE +3, reached by BANKCALL) | RODCOUNT RODSCAL1 VDGVERT PIPAX..Z OLDPIPAX..Z PIPATMPX..Z TEMX..Z TIME2/1 PIPTIME LASTTPIP GDT/2 VBIAS V R /LAND/ HDOTDISP TAUROD LAG/TAU MAXFORCE MINFORCE MASS FWEIGHT (sign-disagreeing) CDUX..Z -> VDGVERT THISTPIP LASTTPIP HDOTDISP HCALC1 /AFC/ DELVROD (A,L = measured acceleration recorded) |


## Lessons

- **The interpretive MPAC is three words, and more ops read MPAC+2 than DMP's users.** DOT, DSQ and DMP leave a TP; DAD, DSU and BDSU change MPAC,+1 and keep MPAC+2 (DCOMP negates all three); SL, SQRT, ROUND and TAD read it, and SQRT leaves the bits of its own unnormalising shift there (so SQRT SL1 is not shl(sqrt)). DDV, DLOAD and DMPR zero it. Scan for producer -> consumer pairs through the pass-through ops and branch pairs (scratch tpscan.py), then use the _tp API.
- **Interpretive labels never appear in Z**, so the probe cannot stop on them; stop on the next basic-language call (PHASCHNG, THROTTLE +3 via BANKCALL) and enter interpretive code at INTPRET with Q = the label's address (or +3 past TC PHASCHNG / TC INTPRET).
- **To split a residual between translation error and word representation, rerun the rope from the middle with canonical words** (here T2TEST with TBUP/TGO/DRDOT canonicalised): if the rope then equals the C, the C is right and the gap is the F9 class. Snippets assembled with yaYUL and poked into VAC2USE isolate one op (DDV of a sign-disagreeing divisor differs by 1 LSB; SR of (4637, -3965) stores (2, 4327) = 37095, not 37094).
- **Erasable names that start with a digit (1/DV1) are parsed as numbers by agc_probe**: give the E-bank address.
- **TS/XCH of an overflowed A stores the overflow-corrected word** (16383 + 1 -> +0); a saturating C helper hides the AGC behaviour even when it is asserted unreachable.

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ This review lane owns no code of its own; the program flow, decision points and data of the code it reviewed, as the code stands after its fixes, are in [guid-landing](guid-landing.md#program-flow-and-decisions).
