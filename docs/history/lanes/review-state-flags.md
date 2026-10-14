# review-state-flags -- lane history

Process diary of the review-state-flags lane, moved verbatim out of [docs/modules/review-state-flags.md](../../modules/review-state-flags.md), which keeps the reader-facing explanation. Note dates: 2026-10-10 to 2026-10-12 (first and last commit touching the note).

## Progress

- DONE flag values vs assembled ropes (every enumerator in the three flag headers)
- DONE typo list confirmed against the symbol tables
- DONE CSM channel bits (channels.h) + tests
- DONE RADMODES one word (verified; AGC_FLAGWORD_RADMODES/DAPBOOLS added)
- DONE DELVSLV = DELVLVC alias bug fixed (accessor removed, 3 callers)
- DONE noun write-path gaps: verified closed (io-dsky side), nothing left here
- DONE prologs/comments/style: 0 check_style findings in all 10 lane files
- DONE scale comments (DELV, PIPA, TEPHEM, CSMMASS); stale state-flags.md passages
- DONE tests: test_state 146 checks, test_state_lm 44 checks, both pass
- DEFERRED assert.h rename (see "Requests")

## Method

Symbol tables of both ropes were dumped from the yaYUL builds
(SCRATCH/review-state-flags/{Comanche055,Luminary099}.sym, symdump.c);
chkflags.py compares every `AGC_FLAG_*` enumerator's word/bit/nnnD with
the assembled value, flagdefs.py checks that every flag defined in the
source (BIT n FLAG m / nnnD) has an enumerator. Result: 148 CSM and 174 LM
definitions, all present, all values equal to the rope. The only name
differences are the NAMING.md renames (V37FLAG -> SERVICER_RUNNING etc.).

## Findings

| # | Where | Finding | Action |
|---|---|---|---|
| 1 | erasable.c / state.h DELVSLV | **Bug.** Both ropes alias DELVSLV and DELVLVC (C: `DELVLVC = DELVSLV`, L: `DELVSLV = DELVLVC`, one six-word vector). The C kept DELVSLV in erasable.c and DELVLVC in agc_rendz_t; nothing ever wrote DELVSLV, so S30.1 (P30 external delta-V) and the downlink always used a zero vector while the crew's N81 load went to DELVLVC. | Accessor removed; external_delta_v.c (S30.1 CSM and LM) and down_telemetry.c read `agc_rendz_erasable()->delvlvc`. |
| 2 | channels.h | CSM bits that Comanche never touches had LM names only. | Added from the Block II CM channel list (virtualagc piPeripheral/humanizeScript.py): ch12 b10 ZEROOPTICS, b13 SIVBINJECT; ch30 b1-4, b6 (ULLAGETHRUST, CMSMSEP, SPSREADY, SIVBSEPABORT, GRR); ch32 b1-6 MINIMP* + MINIMP_MASK (Comanche FREECONT reads them), b11 LMATTACHED; "CSM: unused" on the rest. |
| 3 | channels.h ch33 b4/b5 | Named OPTMODE1/OPTMODE2 ("sense bits"). OPTMON (C p. 160) complements 033 & 030: bit 5 present = CSC (CMC), bit 4 = ZERO OPTICS, neither = MANUAL. | Renamed AGC_CH033_ZEROOPTICS / AGC_CH033_CMCOPTICS (no users). |
| 4 | state.h DELV comment | "scaled like PIPAX via KPIP1 to 2^4 m/cs" was wrong. SERVICER207 p. 821: DELV at 2^14 pulses x 5.85e-4 m/cs; LM 1 cm/s per pulse; KPIP1 (.074880 C, .0128 L) rescales to 2^7 m/cs. | Comment fixed; PIPA comment now gives both pulse weights. |
| 5 | state.h TEPHEM | Scale was "see ephemeris routines". L TIMSUBO = TEPHEM "CSEC B-42 (TRIPLE PRECISION)" (C text says B-14, per word). | Comment: TP cs, scale 2^42. |
| 6 | flag typo list | Confirmed with the rope: CSM sorted list's DMENFLAG, ENG1FLAG, IDLEFLAG, V67FLAG are not defined in Comanche055 (no symbol); 018D is NODOP01 only. LM's DIMOFLAG is DIM0FLAG (073). STEERSW 034D = FLAGWRD2 bit 11 (the source just lacks a "BIT 11" header line). PRIODFLG 061D = FLAGWRD4 bit 14 in both. | Headers already right; recorded here. |
| 7 | RADMODES | RADMODES EQUALS FLGWRD12 (L p. 85), DAPBOOLS EQUALS FLGWRD13 (p. 86); rope addresses 0110 / 0111 = STATE+12/+13. The C already uses flagword 12 through agc_erase_radmodes (io/service/t4rupt_lm.c) and flagword 13 in dap-lm: one word each. | AGC_FLAGWORD_RADMODES / AGC_FLAGWORD_DAPBOOLS added to flags_lm.h for the literal 12/13 users. |
| 8 | noun write path | VHFCNT/TRKMKCNT and FUNNYDSP writes and negative TIG/TTOGO: src/exec/nouns.c noun_two_int_write and src/io/dsky/nouns/data.c noun_access_time already handle them; the TIG/TTOGO setters here accept any signed DP. | Nothing to change in this lane. |
| 9 | state.c / channels.c / erasable.c assertions | Many post-store assertions were tautologies (`g_x = v; AGC_ASSERT(g_x == v)`). | Replaced by real ones: range of the stored value on read, input range and vehicle selected on write, SET/CLEAR/INVERT postconditions via BON, 28-bit wrap after advance, moon flag after agc_sv_set, display-file reset ran (SAMPTIME = 0). Density >= 2.0 in all three files. |
| 10 | state.c set_moon | Two-line helper (collapse candidate). | Folded into agc_sv_set. |
| 11 | tools/check_style.py | Two false positives: enumerator lists counted as "multiple side effects"; `g_tig = v;` parsed as a declaration of type `g_t` (decl-comment on every `g_t*` store). | Enum bodies skipped; `\w+_t\b` in TYPE_WORD. Tree total 22,489 -> 22,422 findings, no new ones. |
| 12 | tools/check_rules.py:82 | The rule-8 allowance for the assert header still tests `agc_assert.h` (pre-restructure name). | Not changed (harmless: no warning today); fix with the rename below. |

Deviations kept (documented in the file prologs): flagwords as an array (no
STATE+n arithmetic), agc_sv_set sets the moon flag, no channel 0 / L,Q
aliasing, no E-bank overlay layout (TIG EQUALS DELVSLV +6 etc.).

## Requests (outside this lane)

* assert.h rename (guide 3.4): not done during the review wave because it
  rewrites the include line of ~237 files that 16 agents are editing. After
  the wave: `git mv include/agc/core/assert.h include/agc/core/assertion.h`,
  `grep -rl '"agc/core/assertion.h"' src tests include docs tools | xargs sed -i
  's|"agc/core/assertion.h"|"agc/core/assertion.h"|'`, and change
  tools/check_rules.py:82 `agc_assert.h` -> `core/assertion.h`.
* lm/burns/servicer/landing_radar.c:280, src/lm/dap/state.c:27,
  src/io/verbs/extended_verbs_lm.c:43 use literal 12/13: switch to
  AGC_FLAGWORD_RADMODES / AGC_FLAGWORD_DAPBOOLS (or agc_erase_radmodes()).
* RDOTREF (C entry) = THETAD in the rope (downlist sends THETAD during
  entry): the C keeps es->rdotref separately, so the entry downlist words
  for THETAD do not carry RDOTREF. Telemetry only; guid-entry/io may decide.
* docs/overview/callgraph.* still list agc_erase_delvslv until `make
  callgraph` is rerun.

## Lessons

* Check EQUALS/= aliases of every shared erasable against the C storage:
  the DELVSLV/DELVLVC split compiled, passed every test and silently zeroed
  P30's input. A rope symbol dump (name -> address) makes such pairs a
  one-line query.
* The assembled symbol table is a better oracle for tables than the source
  text: the sorted flag lists in both ropes contain names that were never
  assembled.
* Style checkers with regex type parsing need word boundaries; `\w+_t`
  matched `g_t` of `g_tig` and produced dozens of false findings.

