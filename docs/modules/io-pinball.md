# io-pinball -- DSKY / PINBALL / display interface / KEYRUPT

## Scope

PINBALL_GAME_BUTTONS_AND_LIGHTS.agc (CM pp. 307-389, LM pp. 390-471),
DISPLAY_INTERFACE_ROUTINES.agc (CM pp. 1452-1484, LM pp. 1341-1373),
KEYRUPT_UPRUPT.agc (CM pp. 1449-1451, LM pp. 1338-1340).
Noun tables are io-nouns' (agc_nouns.h). DSPOUT/DSPOUTSB (T4RUPT p. 133) is
implemented here as agc_dsky_dspout() because DSPTAB is pinball-private;
sys-misc's T4RUPT/CDRVE should call it instead of duplicating it.

CM vs LM (code-only diff): PINBALL differs only in SF routine 6 (CM OPDEGOUT
/ OPTDEGIN optics degrees, LM LRPOSOUT landing-radar position, no load),
SF routine 13 (CM DPFRACOT / DPFRACIN, LM 360-CDUO / DEGINSF + 360-CDU),
DPTEST for 13 (DP in CM only), ENDSCALE (LM 360-CDU fix-up), VBTSTLTS
(CM calls CHKPOOH; TSTCON2 40650 vs 40674; OPTMODES bit 7 vs RADMODES
RFAILS2/RCDUFBIT) and ERROR (OPTMODES vs RADMODES). KEYRUPT: LM uses
DSKYFBIT/UPLOCBIT names (same bits). DISPLAY_INTERFACE: LM adds REFLASHR,
XDSPBIT/MRKNVBIT/MKOVBIT/PRIODBIT names (same bits). All handled with
agc_state_vehicle() tests; one translation.

## Design table

| AGC routine (label, CM page) | In / out, scale | C function (file) |
|---|---|---|
| KEYRUPT1, KEYCOM, ACCEPTUP (1449) | ch15 low5 key; sets DSKYFLAG; NOVAC CHARIN | agc_dsky_keyrupt1, agc_dsky_keyrupt2 (pinball_keys.c) |
| UPRUPT, UPTEST, UPOK, TMFAIL2 (1450) | INLINK 15 bits = C, ~C, C triple; UPLOCKFL | agc_dsky_uprupt (pinball_keys.c) |
| CHARIN (315) | key code 5 bits | pin_charin, agc_job_charin |
| NUM, 89TEST, DECTOBIN, ENDNMTST, DECEND (316-318) | octal 3 bits/char; decimal N -> N/10^5 frac (DECON 2^14/10^5) | pin_num (pinball_keys.c) |
| GETINREL / INRELTAB (318) | DSPCOUNT -> 0 VERB 1 NOUN 2-4 R1-R3 | pin_getinrel (pinball_state.c) |
| VERB, NOUN, NVCOM (319) | | pin_verb_key, pin_noun_key |
| POSGN, NEGSGN, SIGNTEST, +ON, -ON (319-320) | DECBRNCH sign bits | pin_sign_key, pin_plus_on/minus_on |
| CLEAR, CLPASHI, LEGALTST, 5BLANK, 2BLANK (321-322) | CLPASS | pin_clear_key, pin_5blank, pin_2blank |
| ENTER, ENTPASHI, ACCEPTWD (323) | REQRET | pin_enter |
| ENTPAS0, TESTVB, TESTNN, REQADD, USEADD, INTMCTBS, VERBFAN (324-325) | | pin_entpas0, pin_testnn (pinball_verbs.c) |
| VERBTAB VB00-VB39 (325-326) | | pin_vbfandir switch |
| MIXNOUN, DPTEST (327) | flat agc_noun_t replaces IDADDTAB | pin_comp_read, pin_sf_is_dp |
| REQDATX/Y/Z, UPDATNN, UPDATVB (328) | | pin_reqdat, pin_updatnn, pin_updatvb |
| DSPA..DSPABC, COMPTEST, DCOMPTST, DECTEST, NOUNTEST, TSTFORDP, CHANDSP (331-333) | octal display, minor part of DP | pin_dsp_octal (pinball_verbs.c) |
| DECDSP, DSPDCPUT, SFOUTABR (333-334) | SF routine 0-13 | pin_decdsp, pin_decdsp_comp (pinball_scale.c) |
| DEGOUTSF, OPDEGOUT, ARTOUTSF, AROUT1SF, DP1/2/3OUTSF, DPOUT, 2INTOUT, DPFRACOT, HMSOUT, M/SOUT, SEPSEC, SEPMIN (335-341); LM 360-CDUO, LRPOSOUT | DP fraction for DSPDECWD | pin_sfout_* (pinball_scale.c) |
| DSPDPDEC V07 (341) | DP as 10 digits | pin_dspdpdec |
| ABCLOAD..CLOAD, PUTXYZ, PUTXY, LOADLV, ALLDC/OC, PUTCOM, PUTDECSF, CHANLOAD (343-348) | | pin_load_verb, pin_load_resume, pin_putcom (pinball_load.c) |
| SFINTABR, DEGINSF, ARTHINSF, BINROUND, ARTIN1SF, OPTDEGIN, DPINSF, DPINSF2/4, DPFRACIN, HMSIN, 2ROUND, TESTOFUF (349-354) | decimal frac -> erasable word(s) | pin_sfin_* (pinball_scale.c) |
| MONITOR, MONREQ, KILLMON, MONDO, PASTEVB (355-357) | 1 s waitlist (MONDEL 144 oct) | pin_monitor, agc_task_monreq, agc_job_mondo, pin_pastevb |
| DSPFMEM V27 (358) | fixed memory | stub |
| DSPSIGN, DSPRND, DSPDECWD, DSPDECNR, DSPDC2NR, DSP2DEC, DSPDECVN, DSPOCTWO, DSP2BIT (359-362) | DP fraction -> sign + 5 digits, round 5E-6 | pinball_display.c |
| DSPIN, DSPIN1, 11DSPIN (362-363) | relay codes into DSPTAB, NOUT | pin_dspin, pin_11dspin (pinball_state.c) |
| DSPALARM, CHARALRM, ALMCYCLE (364) | POODOO 01501 for NVSUB | pin_dspalarm, pin_almcycle |
| MMCHANG, REQMM (365) | V37 -> agc_v37_change_program | pin_mmchang |
| VBRQEXEC, VBRQWAIT (366) | N26 DSPTEM1: prio, job/task id, (BBCON) | pin_vbrqexec, pin_vbrqwait |
| VBPROC, VBTERM, PROCKEY, VBRESEQ (366-367) | LOADSTAT | pin_vbproc_common, agc_job_prockey |
| VBRELDSP (368) | | pin_vbreldsp |
| NVSUB, NVMONOPT, NVSBWAIT, BLANKDSP, NVSUB1, NVSUB2 (370-372) | | pinball_nvsub.c |
| KILMONON, ENDIDLE, ISCADR+0, ISLIST+0, JAMTERM, JAMPROC, BLANKSUB (372-375) | ENDIDLE: 3 continuation job ids | pinball_nvsub.c |
| DSPMM, DSPMMJB, RECALTST (375-377) | | agc_dsky_dspmm, agc_job_dspmmjb, pin_recaltst |
| PRENVBSY, NVSUBUSY, RELDSP, RELDSP1 (382-383) | | pinball_nvsub.c |
| VBTSTLTS, TSTLTS2, TSTLTS3 (385-387) | 5 s | pin_vbtstlts, agc_task_tstlts2, agc_job_tstlts3 |
| ERROR (RSET) (388-389) | | pin_error_key (pinball_keys.c) |
| Display interface: GODSP..GOMARK4R, MAKEPLAY, MAKEPRIO, MAKEMARK, NVDSP, FLASHSUB, ENDOUT, ENDRET, PINBRNCH, LINUSCHR, WITCHONE, JOBXCHS, GOSLEEPS (1465-1484) | FLAGWRD4 masks, 3 display classes | display_if.c |

Data model: DSPTAB kept as raw 15-bit AGC words (negative = still to be
sent); continuation ids replace return addresses (TRANSLATION_GUIDE s4).

## Progress

(updated as files are finished)

- DONE design; include/agc/agc_dsky.h; src/io/pinball_internal.h (control-flow model in its header comment)
- DONE pinball_state.c: erasables, RELTAB, INRELTAB, DSPIN/DSPIN1/11DSPIN, 2BLANK, 5BLANK (+2), lamps, LODSAMPT, DSPOUT/DSPOUTSB, agc_dsky_get_state, agc_dsky_init
- DONE pinball_display.c: +ON/-ON, DSPSIGN, DSPRND, DSPDECWD/NR/DC2NR, DSP2DEC, DSPDECVN, DSPOCTWO, DSP2BIT, UPDATVB, UPDATNN, DSPMMJB
- DONE pinball_keys.c: CHARIN, NUM, 89TEST, DECTOBIN, ENDNMTST, DECEND, VERB/NOUN/NVCOM, POSGN/NEGSGN/SIGNTEST, CLEAR/CLPASHI/LEGALTST, ENTER/ENTPASHI/ACCEPTWD, VBRELDSP/UNSUSPEN/TSTLTS4, ERROR/TSTAB, PROCKEY, KEYRUPT1/2, UPRUPT, agc_dsky_key(s), agc_dsky_falton
- DONE pinball_verbs.c: LODNNTAB, ENTPAS0, TESTVB, TESTNN, REQADD (+resume), USEADD, INTMCTBS, VERBFAN, VBFANDIR, DSPA..DSPABC, COMPTEST/DECTEST/DCTSTCYC/NOUNTEST, DSPALARM, PREDSPAL, CHARALRM, ALMCYCLE, ENTEXIT
- DONE pinball_monitor.c: MONITOR, MONREQ, KILLMON, MONDO, PASTEVB, DSPFMEM (stub), VBRQEXEC, VBRQWAIT, VBPROC/VBTERM/VBRESEQ, REQMM, MMCHANG+1, VBTSTLTS, TSTLTS2, TSTLTS3
- DONE pinball_load.c: REQDATX/Y/Z, ALOAD..ABCLOAD, PUTXYZ (+noun 07 bits), PUTXY, PUTCOM/PUTNORM/PUTDPCOM, CHANLOAD, ALLDC/OC, LOADLV
- DONE pinball_scale.c: component access (MIXNOUN/TSTFORDP/CHANDSP/READLO equivalents, pinball-owned ids), DPTEST, DECDSP, DSPDCPUT, SF out routines (DEGOUTSF, OPDEGOUT, LRPOSOUT, 360-CDUO, ARTOUTSF, AROUT1SF, DP1/2/3OUTSF, 2INTOUT, DPFRACOT, HMSOUT, M/SOUT), DSPDPDEC
- DONE pinball_sfin.c: PUTDECSF, BINROUND, DEGINSF(+SIGNFIX, LM ENDSCALE), ARTHINSF, ARTIN1SF, OPTDEGIN, DPINSF/2/4, DPFRACIN, HMSIN
- DONE pinball_nvsub.c: NVSUB, NVMONOPT, NVSBWAIT, BLANKDSP, NVSUB1/2, KILMONON, ENDIDLE, ISCADR+0/ISLIST+0/DSPABORT, NVSUBUSY, RECALTST, RELDSP, RELDSP1, BLANKSUB, JAMTERM, JAMPROC, DSPMM, agc_dsky_vn
- DONE display_if.c: all GO*/RE*/PRIODSP/GOMARK*/EXDSPRET/KLEENEX/CLOCPLAY entries, GODSPRS1, MAKEPLAY, MAKEPRIO, MAKEMARK, IFLEGAL/OKTOPLAY, LINUSCHR, JOBXCHS, WITCHONE, GOSLEEPS/XCHSLEEP, NVDSP/PERFCHEK/1STOR2ND, REST, GOANIDLE, FLASHSUB, CLEANEND, ENDOUT/TIMECHEK/NORMRET/MARKRET/ENDRET/ENDIT, TERMATE/PROCEED/IDLERET1+2/LOADITIS, PINBRNCH/NORMBNCH/ERASER, CLEARMRK, MARKOVER, BLANKET, INITDSP
- TODO check_rules, tests/test_dsky.c, note sections

## Deviations

(filled in as code is written)

## Stubs

## Requested core additions

- io-nouns, agc_nouns.h: `agc_status_t agc_noun_scale(uint8_t scale_type, uint8_t *sf_routine, agc_dp_t *sfout, agc_dp_t *sfin);`
  maps a scale_type to the AGC SF routine number (SFOUTABR/SFINTABR index 0-13,
  enum pin_sfrout) and the SFOUTAB / SFINTAB DP constant pair (hi,lo packed).
- io-nouns convention assumed: agc_noun_read/write of an SP component carries
  the SP word in the high word (agc_dp_from_sp); DP components are full DP.
- sys-misc: `agc_status_t agc_extverb_fan(uint8_t verb_minus_40);` (GOEXTVB).
- exec-sched: `agc_status_t agc_v37_change_program(uint8_t new_mm);` (V37/MODROUTB),
  `agc_status_t agc_fresh_start_slap1(void);` (SLAP1, V36).
- state-flags: `agc_erase_read_ecadr(uint16_t ecadr, uint16_t *word)`,
  `agc_erase_write_ecadr(uint16_t ecadr, uint16_t word)` (nouns 01-03/15 machine address),
  `agc_erase_imodes33/set_imodes33`, `agc_erase_imodes30/set_imodes30`,
  `agc_erase_optmodes/set_optmodes` (CSM), `agc_erase_radmodes/set_radmodes` (LM),
  `agc_erase_set_samptime(uint32_t)` (LODSAMPT, N65), `agc_erase_set_option1(agc_sp_t)` (GOPERF4).
- exec-sched (ALARM_AND_ABORT): `void agc_erase_clear_failreg(void);` (ERROR clears FAILREG..+2, SFAIL).
- channel bit names (local PIN_CH11_* in pinball_internal.h until agc_channels.h names them):
  AGC_CH011_UPACT 0004, AGC_CH011_KEYREL 0020, AGC_CH011_VNFLASH 0040,
  AGC_CH011_OPRERR 0100, AGC_CH011_CAUTRSET 01000, AGC_CH013_TESTALM 01000.

## Requested jobs-tasks

Jobs (agc_status_t f(agc_job_ctx_t *)): AGC_JOB_CHARIN (exists, prio 30),
AGC_JOB_PROCKEY (30), AGC_JOB_MONDO (30), AGC_JOB_DSPMMJB (30), AGC_JOB_TSTLTS3 (30),
AGC_JOB_MAKEPLAY (user prio), AGC_JOB_PLAYJUM1, AGC_JOB_MARKPLAY, AGC_JOB_REDOPRIO,
AGC_JOB_TERMATE, AGC_JOB_PROCEED, AGC_JOB_IDLERET3 (display ENDIDLE returns),
AGC_JOB_JAMTERM (prio 32, FINDVAC). Tasks (agc_status_t f(void)):
AGC_TASK_MONREQ, AGC_TASK_TSTLTS2. Placeholders PIN_JOB_* / PIN_TASK_* in
src/io/pinball_internal.h are to be replaced at integration.

## Tests written

## Open questions
