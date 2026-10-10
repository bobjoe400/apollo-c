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
| KEYRUPT1, KEYCOM, ACCEPTUP (1449) | ch15 low5 key; sets DSKYFLAG; NOVAC CHARIN | agc_dsky_keyrupt1, agc_dsky_keyrupt2 (keys.c) |
| UPRUPT, UPTEST, UPOK, TMFAIL2 (1450) | INLINK 15 bits = C, ~C, C triple; UPLOCKFL | agc_dsky_uprupt (keys.c) |
| CHARIN (315) | key code 5 bits | pin_charin, agc_job_charin |
| NUM, 89TEST, DECTOBIN, ENDNMTST, DECEND (316-318) | octal 3 bits/char; decimal N -> N/10^5 frac (DECON 2^14/10^5) | pin_num (keys.c) |
| GETINREL / INRELTAB (318) | DSPCOUNT -> 0 VERB 1 NOUN 2-4 R1-R3 | pin_getinrel (state.c) |
| VERB, NOUN, NVCOM (319) | | pin_verb_key, pin_noun_key |
| POSGN, NEGSGN, SIGNTEST, +ON, -ON (319-320) | DECBRNCH sign bits | pin_sign_key, pin_plus_on/minus_on |
| CLEAR, CLPASHI, LEGALTST, 5BLANK, 2BLANK (321-322) | CLPASS | pin_clear_key, pin_5blank, pin_2blank |
| ENTER, ENTPASHI, ACCEPTWD (323) | REQRET | pin_enter |
| ENTPAS0, TESTVB, TESTNN, REQADD, USEADD, INTMCTBS, VERBFAN (324-325) | | pin_entpas0, pin_testnn (verbs.c) |
| VERBTAB VB00-VB39 (325-326) | | pin_vbfandir switch |
| MIXNOUN, DPTEST (327) | flat agc_noun_t replaces IDADDTAB | pin_comp_read, pin_sf_is_dp |
| REQDATX/Y/Z, UPDATNN, UPDATVB (328) | | pin_reqdat, pin_updatnn, pin_updatvb |
| DSPA..DSPABC, COMPTEST, DCOMPTST, DECTEST, NOUNTEST, TSTFORDP, CHANDSP (331-333) | octal display, minor part of DP | pin_dsp_octal (verbs.c) |
| DECDSP, DSPDCPUT, SFOUTABR (333-334) | SF routine 0-13 | pin_decdsp, pin_decdsp_comp (scale.c) |
| DEGOUTSF, OPDEGOUT, ARTOUTSF, AROUT1SF, DP1/2/3OUTSF, DPOUT, 2INTOUT, DPFRACOT, HMSOUT, M/SOUT, SEPSEC, SEPMIN (335-341); LM 360-CDUO, LRPOSOUT | DP fraction for DSPDECWD | pin_degout, pin_opdegout, pin_artout, pin_dpout, pin_hmsout, pin_msout.. (scale.c) |
| DSPDPDEC V07 (341) | DP as 10 digits | pin_dspdpdec |
| ABCLOAD..CLOAD, PUTXYZ, PUTXY, LOADLV, ALLDC/OC, PUTCOM, CHANLOAD (343-348) | | pin_load_verb, pin_load_resume, pin_putcom, pin_putn (load.c) |
| SFINTABR, DEGINSF, ARTHINSF, BINROUND, ARTIN1SF, OPTDEGIN, DPINSF, DPINSF2/4, DPFRACIN, HMSIN, 2ROUND, TESTOFUF (349-354) | decimal frac -> erasable word(s) | pin_putdecsf, pin_deginsf, pin_dpinsf, pin_hmsin.. (sfin.c) |
| MONITOR, MONREQ, KILLMON, MONDO, PASTEVB (355-357) | 1 s waitlist (MONDEL 144 oct) | pin_monitor, agc_task_monreq, agc_job_mondo, pin_pastevb (monitor.c) |
| DSPFMEM V27 (358) | fixed memory | stub |
| DSPSIGN, DSPRND, DSPDECWD, DSPDECNR, DSPDC2NR, DSP2DEC, DSPDECVN, DSPOCTWO, DSP2BIT (359-362) | DP fraction -> sign + 5 digits, round 5E-6 | display.c |
| DSPIN, DSPIN1, 11DSPIN (362-363) | relay codes into DSPTAB, NOUT | pin_dspin, pin_11dspin (state.c) |
| DSPALARM, CHARALRM, ALMCYCLE (364) | POODOO 01501 for NVSUB | pin_dspalarm, pin_almcycle |
| MMCHANG, REQMM (365) | V37 -> agc_change_program | pin_mmchang |
| VBRQEXEC, VBRQWAIT (366) | N26 DSPTEM1: prio, job/task id, (BBCON) | pin_vbrqexec, pin_vbrqwait |
| VBPROC, VBTERM, PROCKEY, VBRESEQ (366-367) | LOADSTAT | pin_vbproc_common, agc_job_prockey |
| VBRELDSP (368) | | pin_vbreldsp |
| NVSUB, NVMONOPT, NVSBWAIT, BLANKDSP, NVSUB1, NVSUB2 (370-372) | | nvsub.c |
| KILMONON, ENDIDLE, ISCADR+0, ISLIST+0, JAMTERM, JAMPROC, BLANKSUB (372-375) | ENDIDLE: 3 continuation job ids | nvsub.c |
| DSPMM, DSPMMJB, RECALTST (375-377) | | agc_dsky_dspmm, agc_job_dspmmjb, pin_recaltst |
| PRENVBSY, NVSUBUSY, RELDSP, RELDSP1 (382-383) | | nvsub.c |
| VBTSTLTS, TSTLTS2, TSTLTS3 (385-387) | 5 s | pin_vbtstlts, agc_task_tstlts2, agc_job_tstlts3 (monitor.c) |
| ERROR (RSET) (388-389) | | pin_error_key (keys.c) |
| Display interface: GODSP..GOMARK4R, MAKEPLAY, MAKEPRIO, MAKEMARK, NVDSP, FLASHSUB, ENDOUT, ENDRET, PINBRNCH, LINUSCHR, WITCHONE, JOBXCHS, GOSLEEPS (1465-1484) | FLAGWRD4 masks, 3 display classes | display_if.c |

Data model: DSPTAB kept as raw 15-bit AGC words (negative = still to be
sent); continuation ids replace return addresses (TRANSLATION_GUIDE s4).

## Progress

(updated as files are finished)

- DONE design; include/agc/io/dsky.h; src/io/dsky/pinball/internal.h (control-flow model in its header comment)
- DONE state.c: erasables, RELTAB, INRELTAB, DSPIN/DSPIN1/11DSPIN, 2BLANK, 5BLANK (+2), lamps, LODSAMPT, DSPOUT/DSPOUTSB, agc_dsky_get_state, agc_dsky_init
- DONE display.c: +ON/-ON, DSPSIGN, DSPRND, DSPDECWD/NR/DC2NR, DSP2DEC, DSPDECVN, DSPOCTWO, DSP2BIT, UPDATVB, UPDATNN, DSPMMJB
- DONE keys.c: CHARIN, NUM, 89TEST, DECTOBIN, ENDNMTST, DECEND, VERB/NOUN/NVCOM, POSGN/NEGSGN/SIGNTEST, CLEAR/CLPASHI/LEGALTST, ENTER/ENTPASHI/ACCEPTWD, VBRELDSP/UNSUSPEN/TSTLTS4, ERROR/TSTAB, PROCKEY, KEYRUPT1/2, UPRUPT, agc_dsky_key(s), agc_dsky_falton
- DONE verbs.c: LODNNTAB, ENTPAS0, TESTVB, TESTNN, REQADD (+resume), USEADD, INTMCTBS, VERBFAN, VBFANDIR, DSPA..DSPABC, COMPTEST/DECTEST/DCTSTCYC/NOUNTEST, DSPALARM, PREDSPAL, CHARALRM, ALMCYCLE, ENTEXIT
- DONE monitor.c: MONITOR, MONREQ, KILLMON, MONDO, PASTEVB, DSPFMEM (stub), VBRQEXEC, VBRQWAIT, VBPROC/VBTERM/VBRESEQ, REQMM, MMCHANG+1, VBTSTLTS, TSTLTS2, TSTLTS3
- DONE load.c: REQDATX/Y/Z, ALOAD..ABCLOAD, PUTXYZ (+noun 07 bits), PUTXY, PUTCOM/PUTNORM/PUTDPCOM, CHANLOAD, ALLDC/OC, LOADLV
- DONE scale.c: component access (MIXNOUN/TSTFORDP/CHANDSP/READLO equivalents, pinball-owned ids), DPTEST, DECDSP, DSPDCPUT, SF out routines (DEGOUTSF, OPDEGOUT, LRPOSOUT, 360-CDUO, ARTOUTSF, AROUT1SF, DP1/2/3OUTSF, 2INTOUT, DPFRACOT, HMSOUT, M/SOUT), DSPDPDEC
- DONE sfin.c: PUTDECSF, BINROUND, DEGINSF(+SIGNFIX, LM ENDSCALE), ARTHINSF, ARTIN1SF, OPTDEGIN, DPINSF/2/4, DPFRACIN, HMSIN
- DONE nvsub.c: NVSUB, NVMONOPT, NVSBWAIT, BLANKDSP, NVSUB1/2, KILMONON, ENDIDLE, ISCADR+0/ISLIST+0/DSPABORT, NVSUBUSY, RECALTST, RELDSP, RELDSP1, BLANKSUB, JAMTERM, JAMPROC, DSPMM, agc_dsky_vn
- DONE display_if.c: all GO*/RE*/PRIODSP/GOMARK*/EXDSPRET/KLEENEX/CLOCPLAY entries, GODSPRS1, MAKEPLAY, MAKEPRIO, MAKEMARK, IFLEGAL/OKTOPLAY, LINUSCHR, JOBXCHS, WITCHONE, GOSLEEPS/XCHSLEEP, NVDSP/PERFCHEK/1STOR2ND, REST, GOANIDLE, FLASHSUB, CLEANEND, ENDOUT/TIMECHEK/NORMRET/MARKRET/ENDRET/ENDIT, TERMATE/PROCEED/IDLERET1+2/LOADITIS, PINBRNCH/NORMBNCH/ERASER, CLEARMRK, MARKOVER, BLANKET, INITDSP
- DONE agc_dsky_endext (ENDEXT EQUALS ENDMARK)
- DONE tests/io/dsky.c (182 checks pass, see Tests written); check_rules: 0 hard
  violations, assertion density 1.25-1.93 per file (warn)
- Files: include/agc/io/dsky.h; src/io/dsky/pinball/internal.h, state.c,
  display.c, keys.c, verbs.c, monitor.c,
  load.c, scale.c, sfin.c, nvsub.c, display_if.c

## Deviations

- Return addresses become continuation job ids (TRANSLATION_GUIDE s4):
  REQRET is an enum of resume points (pin_reqret_t), ENTRET an enum of exits,
  CADRSTOR/DSPLIST hold job ids. ENDIDLE/NVSUBUSY/GOSLEEPS use the real
  agc_exec_job_sleep; RECAL2 and XCHNYLOC (changing a sleeping job's LOC) use
  the requested agc_exec_job_wake_as. ENDIT = PRIOCHNG + agc_exec_job_yield(cont).
- ALMCYCLE's POSTJUMP ENTER is a bounded loop in pin_enter (PIN_RECYCLE_MAX 3)
  instead of re-entering ENTER (rule 1); ALMCYCLE returns PIN_STATUS_RECYCLE.
- The component tests (COMPTEST, DECTEST, DCTSTCYC, NOUNTEST) are predicates; the
  caller takes the AGC's alarm exit.
- An AGC path that ends the job (ENDOFJOB) inside an internal NVSUB returns OK to
  the NVSUB caller (the AGC would have ended the program's job).
- No ECADRs: noun components are agc_noun_t data ids; NOUNCADR is kept only for
  nouns 01-03 (machine address), 10 (channel), 15 (increment); MIXNOUN/MIXTEMP,
  NOUNADD, IDADDTAB, EBANK/superbank bookkeeping are dropped. Octal display of a
  DP scale shows the minor part (TSTFORDP/MIXNOUN) per component.
- Per-id value convention: an id used by a raw and a signed scale is raw
  (DSPTEM1 +0..+2, OPTION2); pin_comp_scale adjusts is_raw for that until io-nouns
  offers a per-id query.
- DSKY decimal conversions (DECEND, SF out/in routines) are done in 64-bit
  integers with the AGC constants (DECON, DEGTAB, DEGCON1/2, 20BIAS, NEG.2, WHOLECON,
  HRCON, MINCON, DECROUND) on sign and magnitude with the AGC's truncation and
  rounding points; DMP is taken as the exact product (the AGC's dropped low
  partial product can change MPAC+2 by one unit). HMSOUT and M/SOUT derive their
  digits by integer division of the centisecond count (same digits as the AGC's
  fraction chain; ones' complement -0 shown as "-" for negative times).
- OPTDEGIN (CM): the BIT12 rounding is added to the signed DP; with words of
  disagreeing sign the AGC's per-word shifts may differ by one LSB.
- -0 is not representable in agc_sp_t: SP -0 data display as +0 (DSPSIGN shows +);
  a typed -00000 is kept (xyz_negzero) and shown/handled as -0 where pinball sees it.
- MODREG -0/-NZ: DSPMMJB blanks for any negative MODREG.
- V30/V31 (VBRQEXEC/VBRQWAIT): DSPTEM1 +1 is an agc_job_id_t / agc_task_id_t
  number instead of an ADRES; BBCON ignored.
- R display requests (GODSPR, GOFLASHR, ...) pass their PLAYTEM1/3/4 to the
  MAKEPLAY job through a 4-slot table indexed by the job's MPAC
  (agc_exec_set_new_job_arg); BLANKET ORs into the last such slot.
- The display-interface job temporaries kept in the sleeping job's MPAC/MODE
  across ENDIDLE (COPMPAC, USERPRIO) are kept in one static (only one job can be
  in ENDIDLE); USERPRIO of sleeping display classes is kept per class.
- R1SAVE repeat-and-return (FLASHSUB -> UNSETR1 -> IMMEDRET) not modelled: R1SAVE
  is only ever zeroed (fresh start) in Comanche055 and Luminary099.
- INITDSP re-issues the saved normal request (NVSAVE/EBANKTEM/TEMPFLSH) through
  MAKEPLAY instead of jumping back to the user's TC BANKCALL (TEMPFLSH -3).
- MPAC+2 of display-interface users (MARK2PAC) is not carried (always 0).
- KEYRUPT1 via agc_dsky_key()/agc_dsky_keys() runs CHARIN at once (sim
  convenience); agc_dsky_keyrupt1/2 and agc_dsky_uprupt NOVAC CHARIN like the AGC.
- CHANDSP/CHANLOAD for components 2, 3 of noun 10 use channel NOUNCADR+k.

## Stubs

- DSPFMEM (V27 fixed memory display): pin_dspfmem returns
  AGC_STATUS_UNIMPLEMENTED (the rope is not modelled).

## Requested core additions

- exec-sched: `agc_status_t agc_exec_job_wake_as(agc_job_id_t loc, agc_job_id_t new_loc);`
  JOBWAKE the job sleeping at LOC loc, then INDEX LOCCTR / TS LOC = new_loc
  (RECAL2: terminate/proceed/data-in return; XCHNYLOC: XCHTOEND / XCHSLEEP).
  NOT_FOUND if no job sleeps at loc. Declared in internal.h.
- sys-misc: `agc_status_t agc_extverb(int verb, int noun);` (GOEXTVB, already in
  include/agc/io/service_internal.h) and `void agc_erase_set_extvbact(uint16_t)` (CLEARMRK; in
  sysmisc_ext.h). GOPIN (ALM/END -> PINBRNCH) stays sys-misc's; it should call
  agc_dsky_pinbrnch(). T4RUPT's DSPOUT should call agc_dsky_dspout(); PRO key ->
  agc_dsky_prockey_request(); MARKRUPT nav-DSKY keys -> agc_dsky_keyrupt2().
- state-flags: `agc_status_t agc_erase_read_ecadr(uint16_t ecadr, uint16_t *word)`,
  `agc_status_t agc_erase_write_ecadr(uint16_t ecadr, uint16_t word)` (nouns 01-03,
  15, noun 07 flag-bit set/reset), `uint16_t agc_erase_optmodes(void)` /
  `void agc_erase_set_optmodes(uint16_t)` (CSM), `agc_erase_radmodes` /
  `agc_erase_set_radmodes` (LM), `void agc_erase_set_samptime(uint32_t t_cs)`
  (LODSAMPT, N65), `void agc_erase_set_option1(agc_sp_t)` (GOPERF4 PURRS4),
  `void agc_erase_set_sfail(uint16_t)` (ERROR). (IMODES30/33, FAILREG, DSPTAB,
  MODREG, TIME1 accessors exist and are used.)
- io-nouns: a per-id query `bool agc_noun_data_is_raw(uint16_t data_id)` (replaces
  pin_id_is_raw). agc_noun_read/write must return NOT_FOUND (not UNIMPLEMENTED)
  for DSPTEM1.., DSPTEMX.., OPTIONX.., XREG/YREG/ZREG: pinball resolves them first
  through agc_dsky_noun_read/write (it does today).
- agc_dsky.h additions made here (owned by this lane): agc_dsky_falton (sys-misc
  request), agc_dsky_loadlv (GOLOADLV of V50/V51), agc_dsky_set_nvword1 (P30-P47),
  agc_dsky_endext, agc_job_xchsleepm/n.
- Other lanes declared different DSKY entry points in their pending headers
  (include/agc/agc_align_pending.h: agc_dsky_goflash/goperf1/goperf4/gomark/godsp/
  exdspret/priolarm/cleandsp/keycom/set_dsptem1_word; src/csm/burns/guid_pending.h:
  agc_dsky_gopin). They map onto agc_dsky_display(ctx, kind, vn, cont),
  agc_dsky_set_dsptem(0, w), agc_dsky_keyrupt1, agc_dsky_pinbrnch; the lead should
  rewrite those call sites at integration.

## Requested jobs-tasks

Jobs (agc_status_t f(agc_job_ctx_t *), all in agc_dsky.h): AGC_JOB_CHARIN (exists,
prio 30, key code in ctx->arg), AGC_JOB_PROCKEY (30), AGC_JOB_MONDO (30),
AGC_JOB_DSPMMJB (30), AGC_JOB_TSTLTS3 (30), AGC_JOB_MAKEPLAY (user prio, NOVAC or
FINDVAC; request slot+1 in ctx->arg), AGC_JOB_PLAYJUM1 (wake point, also NOVAC
PRIO15), AGC_JOB_MARKPLAY (wake point), AGC_JOB_REDOPRIO (wake point),
AGC_JOB_TERMATE, AGC_JOB_PROCEED, AGC_JOB_IDLERET3 (ENDIDLE returns of the display
interface), AGC_JOB_JAMTERM (FINDVAC PRIO32), AGC_JOB_XCHSLEEPM, AGC_JOB_XCHSLEEPN
(XCHSLEEP with FACEREG 0 / 1). XCHTOEND uses the existing AGC_JOB_ENDOFJOB.
Tasks (agc_status_t f(void)): AGC_TASK_MONREQ (agc_task_monreq),
AGC_TASK_TSTLTS2 (agc_task_tstlts2). Placeholders PIN_JOB_* / PIN_TASK_* in
src/io/dsky/pinball/internal.h are to be replaced at integration.

## Tests written

tests/io/dsky.c (182 checks; all pass when linked with core, exec, noun tables
and scratch stand-ins for the requested accessors): V06N36E clock display (CSM and
LM), DSPOUT relay words for digit pairs (AAAA/B/CCCCC/DDDDD), V21N01E12345E +
octal load to a machine address, V25N22 decimal CDU-degree load (180, 90, -45) and
V06N22 display, 360.00 overflow -> alarm and recycle as V25, CLEAR pass 0 and
backup, OPR ERR / RSET, short decimal entry alarm, V16 monitor (MONREQ, MONDO,
paste, external-monitor NVSUB lockout, KEY REL), NVSUB busy and BLANKDSP, UPRUPT
triple check and UPLOCKFL, V25N36 HMSIN (and > 59 min alarm, FORCEV25), V01N01 and
V25N07 flag bits, V35 lights test and TSTLTS3.
Also exercised (scratch harness with a patched dispatch and wake_as, not in the
tree): GOFLASH answered by V33/V34/V32 and PRO, external display + KEY REL
re-flash (PINBRNCH), GODSPRET, GODSP, GOPERF1 (V01N25 then V50N25), GOFLASHR,
NVSUB V21 + ENDIDLE + data load (response verb/noun), PRIODSP interrupting a
flashing normal display (TIMECHEK < 2 s redisplay, NORMWAKE afterwards), GOMARKF
over a normal display and ENDMARK, LOADITIS (V21 of another noun -> redisplay).

## Open questions

- DSPOUT is implemented here (agc_dsky_dspout) because DSPTAB/NOUT handling is
  pinball's; sys-misc's T4RUPT should call it rather than duplicate it.
- V37 (agc_change_program) currently returns UNIMPLEMENTED in the exec lane.
- Nouns whose data ids io-nouns has not wired (UNIMPLEMENTED) make V06/V0x/loads
  stop with that status; the display interface then drops the display.
