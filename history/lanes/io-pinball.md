# io-pinball -- lane history

Process diary of the io-pinball lane, moved verbatim out of [docs/modules/io-pinball.md](../../docs/modules/io-pinball.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-14 (first and last commit touching the note).

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

## Open questions

- DSPOUT is implemented here (agc_dsky_dspout) because DSPTAB/NOUT handling is
  pinball's; sys-misc's T4RUPT should call it rather than duplicate it.
- V37 (agc_change_program) currently returns UNIMPLEMENTED in the exec lane.
- Nouns whose data ids io-nouns has not wired (UNIMPLEMENTED) make V06/V0x/loads
  stop with that status; the display interface then drops the display.

