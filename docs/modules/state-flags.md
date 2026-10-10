# state-flags -- module note

## Scope

The shared state layer: flagword bit enums, the I/O channel image and bit
names, `agc_state.c` (time, flagwords, state vectors, vehicle selection),
`agc_erasable.c` (accessors for the widely shared erasables), and the lookup
table AGC erasable name -> C accessor / owner (end of this note).

Files: `include/agc/core/flags_common.h`, `agc_flags_csm.h`, `agc_flags_lm.h`,
`agc_channels.h` (edited), `agc_state.h` (extended), `src/core/state.c`,
`src/core/channels.c`, `src/core/erasable.c`, `tests/test_state.c`,
`tests/test_state_lm.c`.

## Progress

- DONE agc_flags_common.h (83 flags with identical name and position in CSM and LM)
- DONE agc_flags_csm.h (CSM-only and position-differing flags; FLAGWRD0..11)
- DONE agc_flags_lm.h (LM-only and position-differing flags; FLAGWRD0..13)
- DONE agc_channels.h: `AGC_CH<octal>_<NAME>` bit constants for channels 5,6,7,10-16,30-33
- DONE agc_channels.c (15-bit image, read/write/or/and, sim input side, inverted 030-033)
- DONE agc_state.h / agc_state.c (vehicle, TIME1/2, flagwords, state vectors)
- DONE agc_erasable.c (36 scalar/vector/matrix + 3 array erasables, generated)
- DONE tests: test_state (163 checks), test_state_lm (48 checks), both pass, -Werror clean
- DONE lookup table (below). `tools/check_rules.py`: 0 violations, density >= 2.0 in all my files
- DONE MODULE_STATUS row
- Nothing TODO. Generated pieces came from throw-away scripts parsing the AGC text; the
  headers are now the source of truth (edit by hand if the AGC text is re-read).

## Design table

| Item | AGC source | C |
|---|---|---|
| Flag id | FLAGWRDn bit b ("BIT b FLAG n"), decimal number nnnD = 15n + 15 - b | `agc_flag_id_t = (n << 4) \| b`, b = AGC bit number 1..15 (bit 1 = LSB). `AGC_FLAG_<AGCNAME>` enum constants |
| Flag API | SET/CLEAR/INVERT/BON/BOFF | `agc_flag_set/clear/invert/test`, `agc_flagword_get/set`, `agc_flag_valid/word/bit/mask` |
| Flagwords | STATE +0..+11D (CSM, 12 words), STATE +0..+13D (LM; words 12,13 = old RADMODES, DAPBOOLS) | `g_flagword[14]`; ids in word 12/13 are invalid in a CSM build (BAD_ARG) |
| Channels | 15-bit I/O channels | `g_chan[32]`; output latches 1,2,5-7,010-014,034,035; inputs 3,4,015,016,030-033 |
| TIME1/TIME2 | 28-bit centisecond counter, wraps at 2^28 | `agc_time_*_cs`, `agc_erase_time1/2` (14-bit halves) |
| State vectors | RN,VN,PIPTIME and RRECTCSM/VRECTCSM/TETCSM, RRECTLEM/VRECTLEM/TETLEM | `agc_sv_get/set(AGC_SV_THIS/OTHER)` |
| Shared erasables | see "Which erasables" | `agc_erase_<name>()` / `agc_erase_set_<name>()` |

### Flag naming and headers

* C name `AGC_FLAG_<AGCNAME>`: upper case, `/`, `-`, `.` -> `_` (per `_common.md`):
  `.05GSW` -> `AGC_FLAG__05GSW`, `CM/DSTBY` -> `AGC_FLAG_CM_DSTBY`, `P39/79SW` ->
  `AGC_FLAG_STABLE_ORBIT_MIDCOURSE_ACTIVE`, `S32.1F1` -> `AGC_FLAG_S32_1F1`, `ACC4-2FL` -> `AGC_FLAG_ACC4_2FL`.
* Each header comment gives the AGC name, flagword, bit number, AGC decimal number nnnD,
  and the AGC's SET / RESET text (transcribed, upper case as in the listing).
* `agc_flags_common.h` holds a flag when the same AGC name sits at the same flagword and
  bit in Comanche055 and Luminary099. `agc_flags_csm.h` and `agc_flags_lm.h` include it and
  add only the rest. Code for both vehicles includes the common header; vehicle code
  includes exactly one vehicle header. Including both fails to compile (duplicate
  enumerators) by design.
* Flags that exist in both vehicles but at DIFFERENT positions (so they are in the
  vehicle headers, with a "[LM: ...]" / "[CSM: ...]" cross-reference):
  AVEGFLAG (CSM FLAGWRD1 bit 1 / LM FLAGWRD7 bit 5); ITSWICH (CSM FLAGWRD7 bit 14 / LM FLAGWRD7 bit 15); NEEDLFLG (CSM FLAGWRD0 bit 9 / LM FLAGWRD0 bit 4); P21FLAG (CSM FLAGWRD2 bit 12 / LM FLAGWRD0 bit 11); S32.1F1 (CSM FLAGWRD11 bit 15 / LM FLAGWRD6 bit 15); S32.1F2 (CSM FLAGWRD11 bit 14 / LM FLAGWRD6 bit 14); S32.1F3A (CSM FLAGWRD11 bit 13 / LM FLAGWRD6 bit 13); S32.1F3B (CSM FLAGWRD11 bit 12 / LM FLAGWRD6 bit 12); V82EMFLG (CSM FLAGWRD9 bit 13 / LM FLAGWRD7 bit 2).
  Shared code needing one of these must pick the right vehicle header in vehicle-specific
  code (or compare against `agc_flag_word/bit` after a `agc_state_vehicle()` switch); the
  common header never contains them.
* Counts: CSM 154 names, LM 176 names, common 83, CSM-only 62, LM-only 84.
* Aliases (several names for one bit) are separate enumerators with the same value:
  ETPIFLAG/FIRSTFLG/OPTNSW, EGSW/KNOWNFLG/R57FLAG, STRULLSW/ENTRYDSP, RNGSCFLG/PRFTRKAT
  (CSM); ETPIFLAG/OPTNSW, R04FLAG/READRFLG (LM). Note CSM `TARG2FLG` and `R23FLG` share
  021D with different texts.
* Source quirks handled: the AGC "sorted list" tables contain typos (e.g. CSM lists
  NODOP01 and ENG1FLAG at 018D, DMENFLAG, IDLEFLAG, V67FLAG that are never defined; LM lists
  DIMOFLAG for DIM0FLAG); the per-flag definitions are authoritative and were used. The CSM
  `STEERSW` comment says bit 12 but its number 034D means bit 11 (used). `PRIODFLG` is in
  FLAGWRD4 in both listings (the LM sorted list says FLAG 1). Unnamed bits are omitted.

### Channels

* Constants `AGC_CH<3-digit octal channel>_<NAME>`, each the single-bit mask (bit 1 =
  000001 ... bit 15 = 040000), plus a few `_MASK` constants (e.g. `AGC_CH033_FLIPFLOP_MASK`).
* LM names come from `Luminary099/INPUT_OUTPUT_CHANNEL_BIT_DESCRIPTIONS.agc`.
  **`Comanche055/ERASABLE_ASSIGNMENTS.agc` contains no per-bit channel comments** (only the
  channel names, lines 176-200), so CSM-specific bits were derived from the Comanche code
  comments: T4RUPT_PROGRAM (IMUMON bits 9,11-15; C33TEST 11-13; OCDU bit 7; liftoff bit 5;
  OPTMON), P11 (SIVB takeover, S/C control of Saturn), P40-P47 (TVC enable, optics DAC,
  SIV cutoff), RCS-CSM_DIGITAL_AUTOPILOT / AUTOMATIC_MANEUVERS (HOLD/FREE/CMC control on
  channel 31), SXTMARK (MARK/MARK REJECT on channel 16), P20-P25 (VHF range data good on
  channel 33 bit 2). Where the vehicles differ the same bit has two constants (e.g.
  `AGC_CH012_ZEROOCDU` CSM / `AGC_CH012_ZERORRCDU` LM, both 000001); the comment starts "CSM:" or "LM:".
* Bits I could not confirm for the CSM from the Comanche text are left with LM names only
  (channel 12 bits 10,12,13; channel 30 bits 1,3,4,6; channel 32 bits 1-10; channel 33
  bits 3,6-9). The Block II CM hardware may assign them; see Open questions.
* Inverted logic: channels 030-033 read ZERO when a signal is present. The image stores what
  READ returns (power-up 077777). `agc_chan_set_input` takes the raw sensed value;
  `agc_chan_set_signal(chan, mask, present)` does the inversion (requested addition, now in the header).
* Channel 033 bits 11-15 are flip-flops that any write pulse to channel 033 resets
  (the AGC uses `WAND CHAN33`): modelled by returning those bits to 1 and leaving 1-10.
* Program writes to input channels (3,4,015,016,030-032) return `AGC_STATUS_BAD_ARG`.

### Which erasables got accessors, and why

Chosen by (a) being persistent ("PRM"/display) state read or written in several
log sections owned by different agents (counted from non-comment references in both
rope sources, see the table), and (b) being the quantities the assignment names.
Everything else stays module-private in its owner.

* Time-of-state/event: PIPTIME, TIG, TTOGO, TGO, TTPI, TEVENT, TETCSM, TETLEM (all DP cs, scale 2^28).
* Vectors: RN, VN, RRECTCSM, VRECTCSM, RRECTLEM, VRECTLEM (state vectors, nav/guidance/programs/downlink),
  DELV, DELVSLV, VGTIG (burn programs and SERVICER), RLS (landing site; nav-aux, prog-rendz,
  R30, landing).
* REFSMMAT (18 DP, scale 2^1), TEPHEM (TP), CSMMASS/LEMMASS (kg, 2^16), MODREG.
* Hardware-register images: CDUX/Y/Z/T/S (raw 15-bit, 2^15 counts = 1 rev), PIPAX/Y/Z.
* Bit-word state shared by T4RUPT, DAPs, IMU, pinball: MARKSTAT, IMODES30, IMODES33,
  DAPDATR1, DAPDATR2.
* Arrays: DSPTAB[12] (pinball/T4RUPT/IMU lamps), FAILREG[3] (alarm registers), THETAD[3].
* Referenced from many files but deliberately NOT promoted (interpreter or scratch/overlay use,
  or owner-local context structs per the assignment of those lanes): COUNT/TC (assembler tokens),
  FIXLOC/PUSHLOC/LOC (interpreter, core-numeric), ITEMP*/RUPTREG*/BUF/BUF2 (scratch),
  RCV/VCV/TET (nav-integ/nav-conic contexts), OGC/IGC/MGC (temporaries), TRKMKCNT/VHFCNT
  (P20 owner).
  If the lead wants any of them shared, add them to the spec table in this note.
* TIME3..TIME6 are not exposed: owned by exec-sched (waitlist, T4RUPT, T5RUPT, T6RUPT).
  CDUxCMD / GYROCMD are not exposed (imu/DAP owners).
* Setters ignore out-of-range input after an `AGC_ASSERT` report (void setters cannot return
  status). Array setters return `agc_status_t`.

## Deviations

* `agc_sv_set` also sets/clears the matching moon flag (MOONFLAG for THIS; LMOONFLG or
  CMOONFLG for OTHER), since the struct bundles `moon`. In the AGC the integration code sets
  the flags. `agc_sv_get` reads them.
* Flags are stored in a 14-word array, not at consecutive erasable addresses, so STATE+n
  address arithmetic is not available (use ids).
* `agc_state_init` zeroes all flags, shared erasables, time and channels. The AGC fresh
  start (FRESH_START_AND_RESTART, exec-sched) sets selected flags/erasables afterwards and
  some flags are "protected from fresh start" (REFSMFLG, CMOONFLG, LMOONFLG, SURFFLAG, APSFLAG):
  that logic belongs in exec-sched, which can use the flag API.
* Channel 0 (indexed-operation-only) and the L/Q register behaviour of channels 1,2 are not
  modelled beyond a 15-bit latch. Superbank channel 7 "not reset by restart" is not modelled
  (`agc_chan_reset` is a power-up reset).
* CDU and bit-pattern words are raw `uint16_t` 15-bit values (the SP model `agc_sp_t`
  cannot hold 0100000..077777 as a positive count).

## Stubs

None.

## Requested core additions

I own the headers below, so these were made directly; other lanes should know:

* `agc_state.h`: `AGC_FLAGWORDS` raised 12 -> 14 (LM has FLGWRD12/13); added
  `AGC_FLAGWORDS_CSM/LM`; `agc_flag_valid/word/bit/mask`; `agc_erase_reset`;
  `AGC_TIME_MODULUS_CS`; `agc_erase_time1/2`; all `agc_erase_*` accessors listed above.
  **`agc_time_set_cs` now returns `agc_status_t`** (was `void`; BAD_ARG if t >= 2^28). Call sites that
  ignored the result should use `(void)` or check it.
* `agc_channels.h`: bit constants, `AGC_CHAN_MASK`, `AGC_CHAN_FIRST/LAST_INVERTED`,
  `agc_chan_set_signal`; `agc_chan_set_input` now returns BAD_ARG for non-input channels.
* No requests for lead-owned headers.
* Names used from `_common.md` convention: `AGC_FLAG_<AGCNAME>`, `AGC_CH<oct>_<NAME>`,
  `agc_erase_<name>()` / `agc_erase_set_<name>()`. Note `AGC_CH030_IMUOPER` is the name for
  channel 30 bit 9 (the example in `_common.md` spelled it that way and I used it). Other lanes
  guessing names should grep `include/agc/core/channels.h`; names are in AGC-source spirit,
  not the AGC symbols (the AGC has none for individual bits).

## Requested jobs-tasks

None.

## Tests written

* `tests/test_state.c` (CSM): flag id decoding vs AGC decimal numbers, aliases, set/clear/invert/test,
  whole-word get/set, invalid ids, channel write/or/and/read masks, inverted inputs, 033 flip-flops,
  bit constants, TIME1/TIME2 split and wrap, state-vector round trip (THIS and OTHER), erasable
  accessors including range rejection. Defines its own `agc_assert_report` to count AGC_ASSERT reports.
* `tests/test_state_lm.c` (LM): LM flag numbers (including FLGWRD12/13), vehicle-dependent validity
  of flag ids, OTHER = CSM state vector, LM channel names.
* Build used: `gcc ... -DAGC_HOST_DEBUG tests/test_state.c src/core/state.c src/core/channels.c
  src/core/erasable.c` (does not depend on other lanes' objects).

## Open questions

* CSM channel bits not documented in the Comanche source (listed above): confirm against the Block II
  CM channel list if CSM lanes (sys-misc, dap-csm) need more names.
* Which of the module-private multi-reader erasables should be promoted? Candidates in the
  "Which erasables" section.
* PIPA scaling: `agc_erase_delv` says "scaled like PIPAX,Y,Z via KPIP1 to 2^4 m/cs" quoting
  SERVICER207; guid-powered should confirm the stored scale when it translates SERVICER.
* TEPHEM scale quoted only as TIMSUBO "CSEC B-14 (triple precision)".

## Lookup table: AGC erasable name -> C accessor or owner

Generated from both `ERASABLE_ASSIGNMENTS.agc` files (CSM = Comanche055, LM = Luminary099).
"size/def" is the word count of an `ERASE` or `eq <definition>` for an `EQUALS`/`=` alias. The owner is the
agent whose files reference the name most (non-comment references, both ropes); "also" lists other
agents that reference it. "unreferenced" names are overlay aliases or spares. "region" is the nearest
preceding section header in the erasable log (unswitched E0-E2 and the E3-E7 shared regions are all
covered; the E-bank/overlay structure is NOT reproduced because the C model has no banking).

Special registers and channels (not in the erasable log sections above):

| AGC name | C |
|---|---|
| A, L, Q, Z, EBANK, FBANK, BBANK, ARUPT..BRUPT, CYR, SR, CYL, EDOP, SAMPTIME | not modelled (machine registers); L/Q are channels 1,2 in `agc_channels.h` |
| TIME1, TIME2 | `agc_time_*_cs`, `agc_erase_time1/2` |
| TIME3, TIME4, TIME5, TIME6 | module-private (exec-sched) |
| CDUX, CDUY, CDUZ, CDUT/OPTY, CDUS/OPTX | `agc_erase_cdux()` ... `agc_erase_cdus()` |
| PIPAX, PIPAY, PIPAZ | `agc_erase_pipax()` ... |
| BMAGX..Z, INLINK, RNRAD, GYROCTR/GYROCMD, CDUXCMD..CDUSCMD, THRUST/EMSD, LEMONM, LOCALARM, BANKALRM | module-private (imu / dap-csm / dap-lm / sys-misc); hardware counters |
| LVSQUARE, LV, X1, X2, S1, S2, QPRET | `agc_interp_t` / C locals (core-numeric) |
| LCHAN..DNTM2 | `agc_chan_*()` with `AGC_CHAN_*` numbers |
| FLAGWRD0..FLGWRD13, STATE | flag API |

| AGC name | vehicles | size/def | C accessor or owner | region |
|---|---|---|---|---|
| +MGA | CSM+LM | eq VTPRIME +6 | module-private (prog-rendz; also guid-powered, io-nouns, exec-sched) | INITVEL/MIDGIM STORAGE. (34D) |
| -2JETLIM | LM | 2 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| -AYO | CSM+LM | eq UNITW | module-private (nav-aux; also prog-rendz) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| -COSB | CSM+LM | eq XSM +2 | module-private (unreferenced in code; overlay alias or spare) | OVERLAYS WITHIN ALIGNMENT/SYSTEST/CALCSMSC COMMON  |
| -DELAIG | CSM | eq -DELAOG +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| -DELAMG | CSM | eq -DELAIG +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| -DELAOG | CSM | eq BETA/PIP +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| -DELGMB | LM | eq UNFV/2 +6 | module-private (guid-landing) | OVERLAYING KALCMANU STORAGE: (26D) |
| -PHASE1 | CSM+LM | 1 | module-private (exec-sched; also guid-powered, guid-landing) | PHASE TABLE AND RESTART COUNTERS (12D) |
| -PHASE2 | CSM+LM | 1 | module-private (exec-sched; also guid-powered) | PHASE TABLE AND RESTART COUNTERS (12D) |
| -PHASE3 | CSM+LM | 1 | module-private (exec-sched; also guid-landing, guid-powered, guid-entry) | PHASE TABLE AND RESTART COUNTERS (12D) |
| -PHASE4 | CSM+LM | 1 | module-private (exec-sched; also guid-powered, guid-landing) | PHASE TABLE AND RESTART COUNTERS (12D) |
| -PHASE5 | CSM+LM | 1 | module-private (exec-sched; also guid-powered, guid-entry) | PHASE TABLE AND RESTART COUNTERS (12D) |
| -PHASE6 | CSM+LM | 1 | module-private (guid-powered; also exec-sched, sys-misc, guid-landing) | PHASE TABLE AND RESTART COUNTERS (12D) |
| -RATEDB | LM | eq -2JETLIM +1 | module-private (dap-lm; also exec-sched, sys-misc) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| -TPER | CSM+LM | eq TFF +2 | module-private (io-nouns; also prog-rendz) | MORE VERB 82 NOT SHARING WITH VERB 83 (9D) |
| -VREL | CSM | eq TIME/RTO +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| -VT/180 | CSM | eq NUJET +1 | module-private (guid-entry) | THE FOLLOWING FEW REGISTERS USED ONCE EACH 2 SEC |
| -VT/180E | CSM | eq ERRORY | module-private (guid-entry) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| /AFC/ | LM | eq NORMEX | module-private (guid-landing) | LUNAR LANDING OVERLAYS (6D) |
| /LAND/ | LM | eq WM +6 | module-private (guid-landing; also guid-powered) | SERVICER FOR LUNAR ASCENT AND DESCENT (14D) |
| /R/MAG | LM | eq ZDOTD +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| 1-CSTH | CSM+LM | eq CSTH +2 | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| 1/ACOAST | LM | eq 1/ANET1 +4 | module-private (dap-lm) | INPUT TO TJET LAW (PERMANENT ERASABLES). (48D) |
| 1/ANET1 | LM | eq BLOCKTOP +16D | module-private (dap-lm) | INPUT TO TJET LAW (PERMANENT ERASABLES). (48D) |
| 1/ANET2 | LM | eq 1/ANET1 +1 | module-private (dap-lm) | INPUT TO TJET LAW (PERMANENT ERASABLES). (48D) |
| 1/CONACC | CSM | eq TENMDOT +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| 1/DV1 | LM | eq YCO +2 | module-private (guid-landing) | ASCENT GUIDANCE ERASABLES. (21D) |
| 1/DV2 | LM | eq 1/DV1 +2 | module-private (guid-landing) | ASCENT GUIDANCE ERASABLES. (21D) |
| 1/DV3 | LM | eq 1/DV2 +2 | module-private (guid-landing) | ASCENT GUIDANCE ERASABLES. (21D) |
| 1/PIPADT | CSM+LM | 1 | module-private (imu; also guid-powered, prog-align, guid-entry, sys-misc) | IMU COMPENSATION UNSWITCHED ERASABLE. (1D) |
| 1JACC | LM | 5 | module-private (dap-lm; also guid-landing, imu) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| 1JACCQ | LM | eq 1JACC +1 | module-private (dap-lm) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| 1JACCR | LM | eq 1JACC +2 | module-private (dap-lm) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| 1JACCU | LM | eq 1JACC +3 | module-private (dap-lm) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| 1JACCV | LM | eq 1JACC +4 | module-private (dap-lm) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| 1SECXT | CSM+LM | eq AINLA +128D | module-private (sys-misc) | SYSTEM TEST STORAGE. (174) |
| 1SECXT1 | CSM | eq AINLA +128D | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| 21/22REG | CSM+LM | eq INTBIT15 | module-private (io-pinball) | UNUSED ERASABLES ********(2) |
| 22SUBSCL | CSM | eq ZI +18D | module-private (prog-rendz) | P22-P23 STORAGE. (8D) |
| 504LM | CSM+LM | 6 | module-private (nav-aux; also nav-integ) | CONISEX STORAGE. -- PAD LOADED -- (6D) |
| 60GENRET | CSM | eq TEMPBETA +1 | module-private (guid-entry) | ENTRY RESTART PROTECTION STORAGE. --KEEP TEMPS IN  |
| 8KK | CSM | eq CXOFF +1 | module-private (prog-rendz) | P22-P23 STORAGE. (8D) |
| 8NN | CSM | eq 8KK +1 | module-private (prog-rendz; also sys-misc) | P22-P23 STORAGE. (8D) |
| 9X9LOC1 | CSM | eq W +44 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX STORAGE. (162D) |
| 9X9LOC2 | CSM | eq 9X9LOC1 +66 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX STORAGE. (162D) |
| A0 | CSM | eq V1 +2 | module-private (guid-entry) | MORE P27 STORAGE. (2D) |
| A1 | CSM | eq DVL +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| A2CNTRAL | LM | eq GTSTEMPS +7 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| AAPFG | LM | eq VAPFG +6 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| AAPFG* | LM | eq VAPFG* +2 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| ABDELV | LM | eq MASS +2 | module-private (dap-lm; also exec-sched, guid-landing, guid-powered) | SERVICER -- MUST FOLLOW PIPTIME -- (19D) |
| ABDVCONV | LM | eq GOBLTIME +2 | module-private (guid-landing; also guid-powered) | SERVICER STORAGE. (69D) |
| ABRFG | LM | eq VBRFG +6 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| ABRFG* | LM | eq VBRFG* +2 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| ABSEDOTP | LM | eq DAPTEMP1 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| ABSTJ | LM | eq DAPTEMP1 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| ABTCOF | LM | eq MAXFORCE +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| ABTRDOT | LM | eq YLIM +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| ABVEL | LM | eq E7OVERLA | module-private (guid-powered; also io-nouns) | SERVICER STORAGE (6D) |
| ABVEL* | LM | eq BUF | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| ACCDOTQ | LM | 4 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| ACCDOTR | LM | eq ACCDOTQ +2 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| ACCFCTZ1 | LM | eq 1/ANET1 +6 | module-private (dap-lm) | INPUT TO TJET LAW (PERMANENT ERASABLES). (48D) |
| ACCFCTZ5 | LM | eq 1/ANET1 +7 | module-private (dap-lm) | INPUT TO TJET LAW (PERMANENT ERASABLES). (48D) |
| ACCSWU | LM | eq BLOCKTOP | module-private (dap-lm) | INPUT TO TJET LAW (PERMANENT ERASABLES). (48D) |
| ACCSWV | LM | eq ACCSWU +1 | module-private (dap-lm) | INPUT TO TJET LAW (PERMANENT ERASABLES). (48D) |
| ACCWD | CSM+LM | eq AINLA +46D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| ACENTRAL | LM | eq GTSTEMPS +5 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| ACORBD | CSM | eq RBDFAIL +1 | module-private (dap-csm; also guid-powered) | OVERLAYS WITHIN TVC DAP |
| ACTCENT | CSM+LM | eq ULOS +6 | module-private (io-nouns; also prog-rendz) | RENDEZVOUS -P34-35 (26D) |
| ADB | CSM | eq SLOPE +1 | module-private (dap-csm; also guid-powered) | OVERLAYS WITHIN TVC DAP |
| ADBVEL | CSM | eq AERRVEL +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| ADDRWD | CSM+LM | 1 | module-private (core-numeric; also guid-powered, guid-landing) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| ADIAX | CSM+LM | 1 | module-private (imu) | IMU COMPENSATION PARAMETERS (22D) |
| ADIAY | CSM+LM | 1 | module-private (imu) | IMU COMPENSATION PARAMETERS (22D) |
| ADIAZ | CSM+LM | 1 | module-private (imu) | IMU COMPENSATION PARAMETERS (22D) |
| ADOT | CSM | eq WBODY2 +2 | module-private (dap-csm; also sys-misc) | OVERLAYS WITHIN TVC DAP |
| ADOT1 | CSM | eq ADOT +2 | module-private (unreferenced in code; overlay alias or spare) | OVERLAYS WITHIN TVC DAP |
| ADOT2 | CSM | eq ADOT1 +2 | module-private (unreferenced in code; overlay alias or spare) | OVERLAYS WITHIN TVC DAP |
| ADRSDIF1 | LM | eq DAPTREG2 | module-private (dap-lm) | THE FOLLOWING LM DAP ERASABLES ARE ZEROED IN THE S |
| ADRSDIF2 | LM | eq DAPTEMP6 | module-private (dap-lm) | THE FOLLOWING LM DAP ERASABLES ARE ZEROED IN THE S |
| ADSRAX | CSM+LM | 1 | module-private (imu) | IMU COMPENSATION PARAMETERS (22D) |
| ADSRAY | CSM+LM | 1 | module-private (imu) | IMU COMPENSATION PARAMETERS (22D) |
| ADSRAZ | CSM+LM | 1 | module-private (imu) | IMU COMPENSATION PARAMETERS (22D) |
| AERR | CSM | eq EDOT +2 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| AERRVEL | CSM | eq EDOTVEL +2 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| AGSBUFF | LM | eq RANGE | module-private (sys-misc; also prog-align) | V47 (R47) AGS INITIALIZATION PROGRAM STORAGE. (OVE |
| AGSBUFFE | LM | eq AGSBUFF +13D | module-private (unreferenced in code; overlay alias or spare) | V47 (R47) AGS INITIALIZATION PROGRAM STORAGE. (OVE |
| AGSK | LM | 2 | module-private (prog-align; also sys-misc) | V47 (R47) AGS INITIALIZATION STORAGE. -- PAD LOADE |
| AGSWORD | LM | 1 | module-private (exec-sched; also prog-align) | AGS DUMMY ID WORD. (1D) |
| AHOOKDV | CSM | eq DHOOK +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| AIG | CSM+LM | eq AOG +1 | module-private (guid-entry; also guid-powered, prog-rendz, sys-misc) | ANGLE REGISTERS FOR ENTRY DAPS |
| AIG/PIP | CSM | eq AOG/PIP +1 | module-private (unreferenced in code; overlay alias or spare) | ANGLE REGISTERS FOR ENTRY DAPS |
| AINLA | CSM+LM | eq GENPL | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| AK | CSM+LM | 1 | module-private (dap-csm; also sys-misc, dap-lm, exec-sched, guid-entry) | R22 PAD LOADS (5D). |
| AK1 | CSM+LM | 1 | module-private (dap-csm; also guid-entry, dap-lm) | R22 PAD LOADS (5D). |
| AK2 | CSM+LM | 1 | module-private (dap-csm; also guid-entry, dap-lm) | R22 PAD LOADS (5D). |
| ALDK | CSM+LM | eq AINLA +62D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| ALFA/180 | CSM | eq ROLL/180 +1 | module-private (guid-entry; also sys-misc) | ANGLE REGISTERS FOR ENTRY DAPS |
| ALFA/PIP | CSM | eq ROLL/PIP +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| ALFACOM | CSM | eq DCDU | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| ALFAPAD | CSM | 1 | module-private (guid-entry) | R22 PAD LOADS (5D). |
| ALFDK | CSM+LM | eq TRANSM1 +18D | module-private (imu; also sys-misc) | SOME P34 STORAGE. (OVERLAYS P35.1 STORAGE) (2D) |
| ALK | CSM+LM | eq AINLA +10D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| ALLOWGTS | LM | eq NEGUQ +1 | module-private (dap-lm) | VARIABLES FOR GTS-QRAXIS CONTROL EXCHANGE. (4) |
| ALMCADR | CSM+LM | eq SMODE +1 | module-private (exec-sched; also io-nouns, sys-misc) | SELF-CHECK ASSIGNMENTS (17D) |
| ALP | CSM | eq THETAH +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| ALPHAM | CSM+LM | eq XKEPNEW +2 | module-private (nav-aux; also nav-integ) | CONIC INTEGRATION STORAGE. -- MAY NOT SHARE WITH S |
| ALPHAQ | LM | eq OMEGAP +3 | module-private (dap-lm; also sys-misc) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| ALPHAR | LM | eq OMEGAP +4 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| ALPHASB | LM | eq RANGE | module-private (io-nouns; also sys-misc) | S-BAND ANTENNA GIMBAL ANGLES. DISPLAYED BY R05 (V6 |
| ALPHAV | CSM+LM | eq PBODY +1 | module-private (prog-rendz; also guid-entry, nav-aux, nav-integ, prog-align...) | INTEGRATION STORAGE. (95D) |
| ALT | CSM+LM | eq LONG +2 | module-private (io-nouns; also nav-aux, guid-powered, prog-align, prog-rendz...) | UNSWITCHED FOR ORBIT INTEGRATION (21D) |
| ALTBITS | LM | eq DELVS +6 | module-private (guid-landing; also guid-powered) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| ALTI | CSM | eq TTE1 +2 | module-private (guid-entry; also io-nouns) | SOME P11 DISPLAY REGISTERS (6D) |
| ALTIM | CSM+LM | eq AINLA +60D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| ALTIMS | CSM+LM | eq AINLA +61D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| ALTRATE | LM | eq VVECT +3 | module-private (guid-landing) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| ALTSAVE | LM | eq ALTRATE +1 | module-private (guid-landing) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| ALTVAR | CSM | 1 | module-private (prog-rendz) | P20 ALTERNATE LOS VARIANCE PAD LOAD **** (1D) |
| ALX1S | CSM+LM | eq AINLA +8D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| AM | CSM+LM | eq POINTVSM +6 | module-private (imu) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| AMEMORY | LM | eq  | module-private (unreferenced in code; overlay alias or spare) | P32-P33 STORAGE (2D) |
| AMG | CSM+LM | eq AIG +1 | module-private (guid-entry; also guid-powered, prog-rendz) | ANGLE REGISTERS FOR ENTRY DAPS |
| AMG/PIP | CSM | eq AIG/PIP +1 | module-private (guid-powered) | ANGLE REGISTERS FOR ENTRY DAPS |
| AMGB1 | CSM | eq RHO2 +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| AMGB4 | CSM | eq AMGB1 +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| AMGB5 | CSM | eq AMGB4 +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| AMGB7 | CSM | eq AMGB5 +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| AMGB8 | CSM | eq AMGB7 +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| ANGTERM | LM | eq VHORIZ +2 | module-private (guid-landing) | SECOND DPS GUIDANCE (LUNAR LANDING) (OVERLAY P32-3 |
| ANGX | CSM+LM | eq AINLA +36D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| ANGY | CSM+LM | eq AINLA +34D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| ANGZ | CSM+LM | eq AINLA +30D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| AOG | CSM+LM | eq BCDU | module-private (guid-entry; also exec-sched, guid-powered, prog-rendz, sys-misc) | ANGLE REGISTERS FOR ENTRY DAPS |
| AOG/PIP | CSM | eq BETA/180 +1 | module-private (guid-entry; also guid-powered) | ANGLE REGISTERS FOR ENTRY DAPS |
| AOPTIME | CSM | eq HPERX +2 | module-private (prog-align) | P22 DISPLAY REGISTERS (06D) |
| AOSQ | LM | 6 | module-private (dap-lm; also exec-sched, guid-powered) | THE FOLLOWING LM DAP ERASABLES ARE ZEROED IN THE S |
| AOSQTERM | LM | 2 | module-private (dap-lm) | THE FOLLOWING LM DAP ERASABLES ARE ZEROED IN THE S |
| AOSR | LM | eq AOSQ +2 | module-private (dap-lm; also guid-powered) | THE FOLLOWING LM DAP ERASABLES ARE ZEROED IN THE S |
| AOSRTERM | LM | eq AOSQTERM +1 | module-private (dap-lm) | THE FOLLOWING LM DAP ERASABLES ARE ZEROED IN THE S |
| AOSU | LM | eq AOSQ +4 | module-private (dap-lm) | THE FOLLOWING LM DAP ERASABLES ARE ZEROED IN THE S |
| AOSV | LM | eq AOSQ +5 | module-private (unreferenced in code; overlay alias or spare) | THE FOLLOWING LM DAP ERASABLES ARE ZEROED IN THE S |
| AOTAZ | LM | 6 | module-private (prog-align; also exec-sched) | AOTMARK STORAGE. -- PAD LOADED -- (12D) |
| AOTCODE | LM | eq STARCODE | module-private (prog-align; also io-nouns, sys-misc) | STARALIGN ERASABLES. (13D) |
| AOTEL | LM | 6 | module-private (prog-align) | AOTMARK STORAGE. -- PAD LOADED -- (12D) |
| ASECXT | LM | eq AINLA +95D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST ERASABLES. CAN OVERLAY W MATRIX. (127D |
| ASKEP | CSM | eq ASPS | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| ASP1 | CSM | eq ASPS +1 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| ASP3 | CSM | eq ASPS +4 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| ASPDWN | CSM | eq ASPS +3 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| ASPS | CSM | eq TTE +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| ASPS(TM) | CSM | eq WBODY | module-private (unreferenced in code; overlay alias or spare) | ENTRY RESTART PROTECTION STORAGE. --KEEP TEMPS IN  |
| ASPUP | CSM | eq ASPS +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| AT | LM | eq PIPTEM +3 | module-private (guid-landing; also core-numeric, sys-misc) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| ATIGINC | CSM+LM | 2 | module-private (prog-rendz; also sys-misc) | P35 CONSTANTS. -- PAD LOADED -- (4D) |
| ATP | LM | eq ATR +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| ATR | LM | eq ATY +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| ATTCADR | CSM+LM | 3 | module-private (imu; also exec-sched) | MODE SWITCHING ERASABLE (14D) |
| ATTKALMN | CSM | eq DTHETASM +6 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| ATTPRIO | CSM+LM | eq ATTCADR +2 | module-private (imu) | MODE SWITCHING ERASABLE (14D) |
| ATTSEC | CSM | eq DRHO2 +2 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| ATY | LM | eq YRATE +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| AVEGEXIT | CSM+LM | eq GOBL/2 +6 | module-private (guid-powered; also guid-entry) | EXCEED THE ERASABLE STORAGE REQUIRED BY RENDEZVOUS |
| AVGEXIT | CSM+LM | eq AVEGEXIT | module-private (guid-powered; also guid-entry, guid-landing) | EXCEED THE ERASABLE STORAGE REQUIRED BY RENDEZVOUS |
| AXISCODE | CSM | eq TNITPREV +2 | module-private (unreferenced in code; overlay alias or spare) | S40.2,3 STORAGE. (1D) |
| AXISCTR | LM | eq KQ +1 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| AXISDIST | LM | eq 1/ANET1 +14D | module-private (dap-lm) | INPUT TO TJET LAW (PERMANENT ERASABLES). (48D) |
| AXO | CSM+LM | eq UNITW +2 | module-private (nav-aux; also prog-rendz) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| AZ | LM | 2 | module-private (prog-align; also io-nouns) | NOUN 87 (2D) |
| AZIDUMMY | LM | eq ELIDUMMY +1 | module-private (unreferenced in code; overlay alias or spare) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| AZIMUTH | CSM+LM | 2 | module-private (sys-misc; also guid-entry, imu) | SYSTEM TEST STORAGE. (174) |
| AZINCR | LM | eq ELINCR +2 | module-private (guid-landing) | THESE ERASABLES MAY BE SHARED WITH CARE |
| AZINCR1 | LM | eq ELVIRA +1 | module-private (guid-landing) | ERASABLES FOR P64: OVERLAY OF DTHETASM, WHICH IS U |
| AZO | CSM+LM | 2 | module-private (nav-aux) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| BALLEXIT | LM | 1 | module-private (imu) | INCORPORATION UNSWITCHED (2D) |
| BANKSET | CSM+LM | 1 | module-private (exec-sched; also core-numeric, io-pinball, sys-misc) | DYNAMICALLY ALLOCATED CORE SETS FOR JOBS (84D) |
| BASEOTP | CSM+LM | eq VECTAB +6 | module-private (prog-rendz) | R31 (V83) STORAGE. -- SHARES WITH INTEGRATION STOR |
| BASEOTV | CSM+LM | eq VECTAB +18D | module-private (prog-rendz) | R31 (V83) STORAGE. -- SHARES WITH INTEGRATION STOR |
| BASETHP | CSM+LM | eq VECTAB +30D | module-private (prog-rendz) | R31 (V83) STORAGE. -- SHARES WITH INTEGRATION STOR |
| BASETHV | CSM+LM | eq RPQV | module-private (prog-rendz) | R31 (V83) STORAGE. -- SHARES WITH INTEGRATION STOR |
| BASETIME | CSM+LM | eq VONE +6 | module-private (prog-rendz) | VERB 83 STORAGE. (20D) |
| BCDU | CSM+LM | eq YMANNDX +1 | module-private (imu; also sys-misc) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| BDT | CSM | eq CSTEER +2 | module-private (guid-powered) | S40.1 STORAGE. (23D) |
| BESTI | CSM+LM | eq MARKINDX +1 | module-private (exec-sched; also prog-align, sys-misc, prog-rendz) | ALIGNMENT STORAGE. (5D) |
| BESTJ | CSM+LM | eq BESTI +1 | module-private (prog-align) | ALIGNMENT STORAGE. (5D) |
| BETA/180 | CSM | eq ALFA/180 +1 | module-private (guid-entry; also guid-powered) | ANGLE REGISTERS FOR ENTRY DAPS |
| BETA/PIP | CSM | eq ALFA/PIP +1 | module-private (guid-entry; also guid-powered) | ANGLE REGISTERS FOR ENTRY DAPS |
| BETA1 | CSM | eq UV1/ +6 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| BETA12 | CSM | eq VNSTORE +1 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| BETACOM | CSM | eq ALFACOM +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| BETADOT | CSM | eq RREL +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| BETAM | CSM+LM | eq FV +6 | module-private (nav-integ) | INTEGRATION STORAGE. (95D) |
| BETASB | LM | eq ALPHASB +2 | module-private (io-nouns) | S-BAND ANTENNA GIMBAL ANGLES. DISPLAYED BY R05 (V6 |
| BETAV | CSM+LM | eq ALPHAV +6 | module-private (nav-integ) | INTEGRATION STORAGE. (95D) |
| BIAS | CSM | eq TAU2 +1 | module-private (dap-csm; also imu) | OVERLAYS WITHIN TVC DAP |
| BIAS1 | CSM | eq BIAS +1 | module-private (dap-csm; also imu) | OVERLAYS WITHIN TVC DAP |
| BIAS2 | CSM | eq BIAS1 +1 | module-private (dap-csm; also imu) | OVERLAYS WITHIN TVC DAP |
| BIASAK | CSM | eq SATSW +1 | module-private (guid-entry) | P11 SATURN I/F (9D) |
| BIASTEMP | CSM+LM | eq TMIS +6 | module-private (imu) | FIRST-ORDER OVERLAYS IN KALCMANU |
| BLANKRET | CSM+LM | eq INTB15+ | module-private (unreferenced in code; overlay alias or spare) | UNUSED ERASABLES ********(2) |
| BLAST | CSM | eq YWORD2 +1 | module-private (dap-csm) | R22 PAD LOADS (5D). |
| BLAST1 | CSM | eq BLAST +2 | module-private (dap-csm) | R22 PAD LOADS (5D). |
| BLAST2 | CSM | eq BLAST1 +2 | module-private (dap-csm) | R22 PAD LOADS (5D). |
| BLOCKTOP | LM | 48 | module-private (dap-lm) | INPUT TO TJET LAW (PERMANENT ERASABLES). (48D) |
| BODY1 | CSM | eq BODY2 +1 | module-private (unreferenced in code; overlay alias or spare) | STORAGE FOR VHHDOT AND ATTDSP |
| BODY2 | CSM | eq BODY3 +1 | module-private (unreferenced in code; overlay alias or spare) | STORAGE FOR VHHDOT AND ATTDSP |
| BODY3 | CSM | eq POLYSTOP +1 | module-private (exec-sched; also guid-entry) | STORAGE FOR VHHDOT AND ATTDSP |
| BOOSTEMP | CSM | eq HDOTDISP +2 | module-private (guid-entry) | STORAGE FOR S11.1 |
| BRATE | CSM+LM | eq COFSKEW | module-private (imu) | FIRST-ORDER OVERLAYS IN KALCMANU |
| BUF | CSM+LM | 3 | module-private (guid-landing; also guid-powered, core-numeric, exec-sched, imu...) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| BUF2 | CSM+LM | 2 | module-private (sys-misc; also imu, core-numeric, exec-sched, guid-powered...) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| BUNKER | CSM | eq N +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| BVECTOR | CSM+LM | eq OMEGA +18D | module-private (nav-aux; also prog-rendz) | MEASUREMENT INCORPORATION STORAGE (R22) STORAGE. ( |
| BZERO | CSM | eq ERRBTMP | module-private (dap-csm; also exec-sched) | OVERLAYS WITHIN TVC DAP |
| C/D0 | CSM | eq ALP +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| C1MP | LM | eq KV3 +4 | module-private (unreferenced in code; overlay alias or spare) | SECOND-ORDER OVERLAYS IN KALCMANU (24D) |
| C1PP | LM | eq KV3 +2 | module-private (unreferenced in code; overlay alias or spare) | SECOND-ORDER OVERLAYS IN KALCMANU (24D) |
| C2MP | LM | eq KV3 | module-private (unreferenced in code; overlay alias or spare) | SECOND-ORDER OVERLAYS IN KALCMANU (24D) |
| C2PP | LM | eq KV2 +4 | module-private (unreferenced in code; overlay alias or spare) | SECOND-ORDER OVERLAYS IN KALCMANU (24D) |
| C2SQM | LM | eq KV2 +2 | module-private (unreferenced in code; overlay alias or spare) | SECOND-ORDER OVERLAYS IN KALCMANU (24D) |
| C2SQP | LM | eq KV2 | module-private (unreferenced in code; overlay alias or spare) | SECOND-ORDER OVERLAYS IN KALCMANU (24D) |
| CADDY | CSM | eq STRKTIME +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| CADRFLSH | CSM+LM | 1 | module-private (io-pinball; also sys-misc) | UNSWITCHED FOR DISPLAY INTERFACE ROUTINES. (10D) |
| CADRMARK | CSM+LM | 1 | module-private (unreferenced in code; overlay alias or spare) | UNSWITCHED FOR DISPLAY INTERFACE ROUTINES. (10D) |
| CADRSTOR | CSM+LM | 1 | module-private (io-pinball; also exec-sched) | PINBALL EXECUTIVE ACTION (44D) |
| CALCDIR | CSM+LM | eq GENPL +20D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| CALFA | CSM | eq SPNDX | module-private (guid-entry) | MISCELLANEOUS PERMANENT ERASABLE. |
| CAM | CSM+LM | eq COFSKEW +6 | module-private (imu) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| CAPSI | CSM | eq AMGB8 +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| CARD | CSM | eq REVS +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| CDUANG | CSM+LM | eq GENPL +71D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| CDUCHKWD | CSM | 1 | module-private (prog-align) | ERASABLE FOR SXTMARK CDU CHECK DELAY. -- PAD LOADE |
| CDUDANG | LM | eq GENPL +4 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST ERASABLES. CAN OVERLAY W MATRIX. (127D |
| CDUFLAG | CSM+LM | eq GENPL +21D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| CDUIND | CSM+LM | eq GCOMP +3 | module-private (imu; also sys-misc, exec-sched) | IMU COMPENSATION PARAMETERS (22D) |
| CDULIMIT | CSM+LM | eq GENPL +7 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| CDUNDX | CSM+LM | eq GENPL +66D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| CDUREADF | CSM+LM | eq GENPL +5 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| CDUREADI | CSM+LM | eq GENPL +6 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| CDUSPOT | CSM+LM | 6 | module-private (guid-powered; also imu, prog-align, guid-entry, prog-rendz) | A**SR*T STORAGE (6D) |
| CDUSPOTX | CSM+LM | eq CDUSPOT +4 | module-private (guid-powered; also guid-landing) | A**SR*T STORAGE (6D) |
| CDUSPOTY | CSM+LM | eq CDUSPOT | module-private (guid-powered; also guid-landing) | A**SR*T STORAGE (6D) |
| CDUSPOTZ | CSM+LM | eq CDUSPOT +2 | module-private (guid-powered; also guid-landing) | A**SR*T STORAGE (6D) |
| CDUTEMPX | LM | 1 | module-private (guid-powered) | MISCELLANEOUS UNSWITCHED. (20D) |
| CDUTEMPY | LM | 1 | module-private (guid-powered) | MISCELLANEOUS UNSWITCHED. (20D) |
| CDUTEMPZ | LM | 1 | module-private (guid-powered) | MISCELLANEOUS UNSWITCHED. (20D) |
| CDUTIMEF | CSM+LM | eq GENPL +2 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| CDUTIMEI | CSM+LM | eq GENPL | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| CDUXD | CSM+LM | eq CAPSI +1 | module-private (dap-lm; also imu, prog-rendz, dap-csm, guid-landing...) | OVERLAYS WITHIN TVC DAP |
| CDUYD | CSM+LM | eq CDUXD +2 | module-private (dap-lm; also imu, prog-rendz, guid-landing) | OVERLAYS WITHIN TVC DAP |
| CDUZD | CSM+LM | eq CDUYD +2 | module-private (dap-lm; also imu, prog-rendz, guid-landing) | OVERLAYS WITHIN TVC DAP |
| CENTANG | CSM+LM | eq RTSR1/MU +2 | module-private (prog-rendz; also io-nouns, sys-misc) | RENDEZVOUS GUIDANCE STORAGE. -- P32...P35 -- (10D) |
| CFPA | CSM | eq P(T1) +1 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| CG | LM | eq COSTHET2 +2 | module-private (guid-landing) | SOME VARIABLES FOR SECOND DPS GUIDANCE. (34D) |
| CGF | CSM | eq TMIS +12D | module-private (unreferenced in code; overlay alias or spare) | FIRST-ORDER OVERLAYS IN KALCMANU |
| CH31TEMP | CSM+LM | eq XTRANS +1 | module-private (dap-lm; also dap-csm) | OVERLAYS WITHIN TVC DAP |
| CH5MASK | LM | 1 | module-private (dap-lm; also exec-sched) | PERMANENT LEM DAP STORAGE (12D) |
| CH6MASK | LM | 1 | module-private (dap-lm; also exec-sched) | PERMANENT LEM DAP STORAGE (12D) |
| CHAN | CSM+LM | eq GENPL +7 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| CHANTEMP | CSM | eq CH31TEMP +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| CHAR | CSM+LM | eq POLISH | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| CLPASS | CSM+LM | 1 | module-private (exec-sched; also io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| CM/GYMDT | CSM | eq POSEXIT +1 | module-private (exec-sched; also guid-entry, guid-powered) | MISCELLANEOUS PERMANENT ERASABLE. |
| CM/SAVE | CSM | eq T1BITS +1 | module-private (guid-entry) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| CMDAPMOD | CSM | eq -DELAMG +1 | module-private (guid-entry; also sys-misc) | ANGLE REGISTERS FOR ENTRY DAPS |
| CMDTMP | CSM | eq ERRBTMP +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| CMPX1 | CSM+LM | eq AINLA +9D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| CMTMTIME | CSM | eq UPBUFF | module-private (guid-entry) | MORE P27 STORAGE. (2D) |
| CNTR | CSM | 1 | module-private (dap-csm; also guid-powered) | R22 PAD LOADS (5D). |
| CNTRTMP | CSM | eq YACTTMP +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| COASTDB | LM | eq 1/ANET1 +12D | module-private (dap-lm) | INPUT TO TJET LAW (PERMANENT ERASABLES). (48D) |
| CODE | CSM+LM | eq VBUF +2 | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| COEFFADR | CSM | eq TMP6 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| COEFFQ | LM | 2 | module-private (dap-lm) | INPUT TO TJET LAW (PERMANENT ERASABLES). (48D) |
| COEFFR | LM | eq COEFFQ +1 | module-private (dap-lm) | INPUT TO TJET LAW (PERMANENT ERASABLES). (48D) |
| COF | CSM+LM | eq MIS +18D | module-private (imu) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| COFSKEW | CSM+LM | eq TMIS +18D | module-private (imu) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| COGA | CSM+LM | eq 3774 | module-private (nav-conic; also guid-entry) | MORE CONICS STORAGE. (4) |
| COLREG | CSM+LM | eq TDEC +2 | module-private (nav-integ) | UNSWITCHED FOR ORBIT INTEGRATION (21D) |
| COMMAND | CSM+LM | eq GCOMP | module-private (imu) | IMU COMPENSATION PARAMETERS (22D) |
| COMMANDO | CSM | 2 | module-private (sys-misc) | T4RUPT ERASABLE (6D) |
| COMPNUMB | CSM+LM | 24 | module-private (sys-misc) | P27 (UPDATE PROGRAM) STORAGE. -- OVERLAYS SPEC DNL |
| CONICX1 | CSM | eq RTENCKEX +1 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| COSCDU | CSM+LM | eq STARALGN +6 | module-private (guid-powered) | STARALIGN ERASABLES. (13D) |
| COSCDUX | CSM+LM | eq COSCDU +4 | module-private (dap-csm; also guid-entry, guid-landing, guid-powered, imu...) | STARALIGN ERASABLES. (13D) |
| COSCDUY | CSM+LM | eq COSCDU | module-private (guid-powered; also imu, prog-align) | STARALIGN ERASABLES. (13D) |
| COSCDUZ | CSM+LM | eq COSCDU +2 | module-private (guid-powered; also dap-csm, guid-entry, guid-landing, imu...) | STARALIGN ERASABLES. (13D) |
| COSG/2 | CSM | eq VBARS +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| COSM | CSM | eq SINM +1 | module-private (guid-entry) | MISCELLANEOUS PERMANENT ERASABLE. |
| COSO | CSM | eq SINO +1 | module-private (guid-entry) | MISCELLANEOUS PERMANENT ERASABLE. |
| COSOCOSM | CSM | eq SINOCOSM +1 | module-private (guid-entry) | MISCELLANEOUS PERMANENT ERASABLE. |
| COSTH | CSM+LM | eq 16D | module-private (imu; also nav-aux, prog-align, prog-rendz, guid-entry...) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| COSTHET1 | LM | eq ABTRDOT +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| COSTHET2 | LM | eq COSTHET1 +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| COTROLER | LM | 1 | module-private (dap-lm) | VARIABLES FOR GTS-QRAXIS CONTROL EXCHANGE. (4) |
| COUNT | CSM+LM | eq TEM3 | module-private (sys-misc; also prog-rendz, exec-sched, dap-csm, imu...) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| COUNTPL | CSM+LM | eq GENPL +70D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| COZY4 | CSM+LM | eq +MGA +2 | module-private (prog-rendz) | INITVEL/MIDGIM STORAGE. (34D) |
| CPHI | CSM+LM | eq THETAD | module-private (imu; also prog-rendz, dap-csm, dap-lm, guid-landing...) | STANDBY VERB ERASABLES. REDOCTR BEFORE THETAD (DWN |
| CPHIX | CSM | 3 | module-private (dap-csm; also io-nouns, prog-rendz, sys-misc) | CRS61.1 STORAGE. -- USED IN R63 (VERB 89) -- (5D) |
| CPSI | CSM+LM | eq THETAD +2 | module-private (imu; also dap-lm, prog-rendz) | STANDBY VERB ERASABLES. REDOCTR BEFORE THETAD (DWN |
| CSIALRM | CSM+LM | eq TITER | module-private (prog-rendz) | THE FOLLOWING OVERLAY MEAS. INCORP. ARE AND IN USE |
| CSMMASS | CSM+LM | 1 | `agc_erase_csmmass()` / `agc_erase_set_csmmass()` | R22 PAD LOADS (5D). |
| CSMPOS | CSM | 58 | module-private (prog-rendz) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| CSTEER | CSM | eq DELVIMU +6 | module-private (guid-powered; also sys-misc) | S40.1 STORAGE. (23D) |
| CSTH | CSM+LM | eq SNTH +2 | module-private (prog-rendz; also nav-conic, guid-entry) | CONICS ROUTINE STORAGE. (87D) |
| CSTH-RHO | CSM+LM | eq 1-CSTH +2 | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| CTHETA | CSM+LM | eq THETAD +1 | module-private (imu; also dap-csm, dap-lm) | STANDBY VERB ERASABLES. REDOCTR BEFORE THETAD (DWN |
| CULTRIX | CSM+LM | eq VEARTH | module-private (prog-align) | ALIGNMENT/SYSTEST/AUTO OPTICS COMMON STORAGE. (17D |
| CXOFF | CSM | eq 22SUBSCL +1 | module-private (prog-rendz) | P22-P23 STORAGE. (8D) |
| CZERO | CSM | eq ERRBTMP | module-private (unreferenced in code; overlay alias or spare) | OVERLAYS WITHIN TVC DAP |
| D | CSM | eq L/D1 +2 | module-private (guid-entry; also io-nouns) | REENTRY ERASABLES. (206D) |
| D0 | CSM | eq C/D0 +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| D21 | CSM+LM | eq KV1 +2 | module-private (unreferenced in code; overlay alias or spare) | SECOND-ORDER OVERLAYS IN KALCMANU |
| DALTRATE | LM | eq DT +1 | module-private (guid-landing; also guid-powered) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| DAP1 | CSM | eq TTMP2 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| DAP2 | CSM | eq DAP1 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| DAP3 | CSM | eq DAP2 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| DAPARUPT | LM | eq DAPTEMP1 +12D | module-private (dap-lm) | END OF FINDCDUW ERASABLES |
| DAPBQRPT | LM | eq DAPARUPT +2 | module-private (dap-lm) | END OF FINDCDUW ERASABLES |
| DAPDATR1 | CSM+LM | 1 | `agc_erase_dapdatr1()` / `agc_erase_set_dapdatr1()` | R22 PAD LOADS (5D). |
| DAPDATR2 | CSM | 1 | `agc_erase_dapdatr2()` / `agc_erase_set_dapdatr2()` | R22 PAD LOADS (5D). |
| DAPLRUPT | LM | eq DAPARUPT +1 | module-private (unreferenced in code; overlay alias or spare) | END OF FINDCDUW ERASABLES |
| DAPTEMP1 | LM | 18 | module-private (dap-lm) | END OF FINDCDUW ERASABLES |
| DAPTEMP2 | LM | eq DAPTEMP1 +1 | module-private (dap-lm) | END OF FINDCDUW ERASABLES |
| DAPTEMP3 | LM | eq DAPTEMP1 +2 | module-private (dap-lm) | END OF FINDCDUW ERASABLES |
| DAPTEMP4 | LM | eq DAPTEMP1 +3 | module-private (dap-lm) | END OF FINDCDUW ERASABLES |
| DAPTEMP5 | LM | eq DAPTEMP1 +4 | module-private (dap-lm) | END OF FINDCDUW ERASABLES |
| DAPTEMP6 | LM | eq DAPTEMP1 +5 | module-private (dap-lm) | END OF FINDCDUW ERASABLES |
| DAPTREG1 | LM | eq DAPTEMP1 +6 | module-private (dap-lm) | END OF FINDCDUW ERASABLES |
| DAPTREG2 | LM | eq DAPTEMP1 +7 | module-private (unreferenced in code; overlay alias or spare) | END OF FINDCDUW ERASABLES |
| DAPTREG3 | LM | eq DAPTEMP1 +8D | module-private (unreferenced in code; overlay alias or spare) | END OF FINDCDUW ERASABLES |
| DAPTREG4 | LM | eq DAPTEMP1 +9D | module-private (dap-lm) | END OF FINDCDUW ERASABLES |
| DAPTREG5 | LM | eq DAPTEMP1 +10D | module-private (dap-lm) | END OF FINDCDUW ERASABLES |
| DAPTREG6 | LM | eq DAPTEMP1 +11D | module-private (dap-lm) | END OF FINDCDUW ERASABLES |
| DAPZRUPT | LM | eq DAPARUPT +4 | module-private (dap-lm) | END OF FINDCDUW ERASABLES |
| DATAGOOD | LM | eq UYVECT +6 | module-private (io-nouns; also prog-rendz) | LRS24.1 STORAGE. (CAN SHARE WITH P30'S) (40D) |
| DATAPL | CSM+LM | eq GENPL +30D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| DATATEST | CSM | eq TRUNX | module-private (exec-sched) | S22.1 STORAGE. (36D) |
| DB | LM | 1 | module-private (dap-lm; also exec-sched) | SOME LEM DAP STORAGE. (4D) |
| DCDU | CSM+LM | eq DELCDUZ +2 | module-private (guid-powered; also prog-rendz) | OVERLAYS WITHIN TVC DAP |
| DECBRNCH | CSM+LM | 1 | module-private (io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| DECOUNT | CSM+LM | eq POLISH | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| DECRET | CSM+LM | eq INTBIT15 | module-private (io-pinball) | UNUSED ERASABLES ********(2) |
| DECTEM | CSM+LM | eq VBUF | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| DEL | CSM+LM | eq MFS | module-private (imu; also dap-lm) | KALCMANU STORAGE. (18D) |
| DELAYLOC | CSM+LM | 4 | module-private (exec-sched; also sys-misc) | MISCELLANEOUS UNSWITCHED. (16D) |
| DELBRTMP | CSM | eq STRKTTMP +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| DELCDUX | CSM+LM | eq THETADZ +1 | module-private (dap-lm; also dap-csm, guid-landing, imu, prog-rendz) | OVERLAYS WITHIN TVC DAP |
| DELCDUY | CSM+LM | eq DELCDUX +2 | module-private (dap-lm; also imu, prog-rendz) | OVERLAYS WITHIN TVC DAP |
| DELCDUZ | CSM+LM | eq DELCDUY +2 | module-private (dap-lm; also imu, prog-rendz) | OVERLAYS WITHIN TVC DAP |
| DELDCDU | LM | eq DELCDUX | module-private (imu) | SECOND-ORDER OVERLAYS IN KALCMANU (24D) |
| DELDCDU1 | LM | eq DELCDUY | module-private (imu) | SECOND-ORDER OVERLAYS IN KALCMANU (24D) |
| DELDCDU2 | LM | eq DELCDUZ | module-private (imu) | SECOND-ORDER OVERLAYS IN KALCMANU (24D) |
| DELDEP | CSM+LM | eq RDESIRED +2 | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| DELDV | CSM+LM | eq DELVCSI +2 | module-private (prog-rendz) | THE FOLLOWING OVERLAY MEAS. INCORP. ARE AND IN USE |
| DELEL | CSM+LM | eq DELTEEO +2 | module-private (prog-rendz) | RENDEZVOUS GUIDANCE STORAGE. -- P32 ... P35 -- (8D |
| DELHITE | CSM | eq TF +2 | module-private (io-nouns; also prog-rendz) | TPI SEARCH (S17.1, S17.2) P17 STORAGE.(10D) |
| DELLT4 | CSM+LM | eq RTARG +6 | module-private (prog-rendz; also guid-powered, sys-misc) | EXTERNAL DELTA-V UPDATE. (21D) |
| DELM | CSM+LM | eq AINLA +76D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| DELOGA | CSM | eq SGNRT +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| DELOGART | CSM | eq OGA +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| DELPBAR | CSM | eq YERRB +2 | module-private (dap-csm; also guid-powered) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| DELPEROR | LM | 1 | module-private (dap-lm; also guid-landing, imu) | KALCMANU/DAP INTERFACE (3D) |
| DELQEROR | LM | 1 | module-private (dap-lm; also imu) | KALCMANU/DAP INTERFACE (3D) |
| DELQFIX | LM | eq TAUVERT +2 | module-private (guid-powered) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| DELREROR | LM | 1 | module-private (dap-lm; also imu) | KALCMANU/DAP INTERFACE (3D) |
| DELT | CSM+LM | eq DELX +2 | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| DELTACSM | CSM+LM | 6 | module-private (prog-rendz) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| DELTAH | LM | eq VN2 +6 | module-private (guid-landing; also guid-powered, io-nouns, sys-misc) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| DELTALEM | CSM+LM | 6 | module-private (prog-rendz) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| DELTAQ | CSM+LM | eq BVECTOR +18D | module-private (nav-aux; also prog-rendz) | MEASUREMENT INCORPORATION STORAGE (R22) STORAGE. ( |
| DELTAR | CSM+LM | eq DVLOS | module-private (io-nouns; also prog-rendz, sys-misc) | RENDEZVOUS -P34-35 (26D) |
| DELTAT | CSM | eq T12 +2 | module-private (unreferenced in code; overlay alias or spare) | RETURN-TO-EARTH STORAGE. (93D) |
| DELTATM | LM | eq REPOSTM +2 | module-private (prog-rendz) | P22 STORAGE -- OVERLAYS LANDING PADLOADS -- (5D) |
| DELTAX | CSM+LM | eq ZIXB +1 | module-private (nav-aux; also prog-rendz) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| DELTEE | CSM+LM | eq MRKBUF2 +7 | module-private (prog-rendz) | TPI SEARCH (S17.1, S17.2) P17 STORAGE.(10D) |
| DELTEEO | CSM+LM | eq END-KALC | module-private (prog-rendz) | RENDEZVOUS GUIDANCE STORAGE. -- P32 ... P35 -- (8D |
| DELTEMPX | CSM | eq WTEMP +2 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| DELTEMPY | CSM | eq DELTEMPX +2 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| DELTEMPZ | CSM | eq DELTEMPY +2 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| DELTTFAP | LM | 1 | module-private (guid-landing) | P63 AND P64 CONSTANTS. -- PAD LOADED -- (4D) |
| DELTTIME | CSM+LM | eq DVLOS +2 | module-private (prog-rendz) | RENDEZVOUS -P34-35 (26D) |
| DELV | CSM+LM | 6 | `agc_erase_delv()` / `agc_erase_set_delv()` | STANDBY VERB ERASABLES. REDOCTR BEFORE THETAD (DWN |
| DELVCSI | CSM+LM | eq POSTCSI +2 | module-private (prog-rendz) | THE FOLLOWING OVERLAY MEAS. INCORP. ARE AND IN USE |
| DELVCTL | CSM+LM | eq VGBODY | module-private (guid-powered) | P40 STORAGE. (8D) |
| DELVEE | CSM | eq TFI +2 | module-private (prog-rendz) | TPI SEARCH (26D) |
| DELVEET1 | CSM+LM | eq 9X9LOC1 | module-private (prog-rendz; also sys-misc) | P32,P33 STORAGE OVERLAYING 9X9 W-MATRIX LOCATIONS  |
| DELVEET2 | CSM+LM | eq S22WUNL | module-private (prog-rendz; also sys-misc) | THE FOLLOWING OVERLAY MEAS. INCORP. ARE AND IN USE |
| DELVEET3 | CSM+LM | eq DELVSIN | module-private (prog-rendz; also guid-powered, sys-misc) | P30, P40 INTERFACE. (20D) |
| DELVEO | CSM | eq HPO +2 | module-private (prog-rendz) | TPI SEARCH (26D) |
| DELVIMU | CSM+LM | eq P40TMP +2 | module-private (guid-powered; also io-nouns, exec-sched) | P47 STORAGE. |
| DELVLVC | CSM+LM | eq DELVSLV | module-private (prog-rendz; also io-nouns, guid-entry) | EXTERNAL DELTA-V UPDATE. (21D) |
| DELVMID | LM | eq DELVTPI | module-private (unreferenced in code; overlay alias or spare) | RENDEZVOUS GUIDANCE STORAGE --P32.....P35-- (89D) |
| DELVOV | CSM+LM | eq RACT3 | module-private (io-nouns; also prog-rendz) | P76, N84 DISPLAY (6D) |
| DELVREF | CSM+LM | eq DVCNTR +1 | module-private (guid-powered; also guid-entry) | SERVICER STORAGE. (13D) |
| DELVROD | LM | eq OLDPIPAZ +1 | module-private (guid-landing) | P66 ERASABLES (R.O.D.) (14D) |
| DELVS | LM | eq UHZP +6 | module-private (guid-landing; also guid-powered) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| DELVSAB | CSM+LM | eq DELVSIN +6 | module-private (guid-powered; also prog-rendz) | P30, P40 INTERFACE. (20D) |
| DELVSIN | CSM+LM | eq VTIG +6 | module-private (prog-rendz; also guid-powered, guid-entry) | P30, P40 INTERFACE. (20D) |
| DELVSLV | CSM+LM | eq LNG(SPL) +2 | `agc_erase_delvslv()` / `agc_erase_set_delvslv()` | EXTERNAL DELTA-V UPDATE. (21D) |
| DELVSUM | CSM | eq NBRCYCLP +1 | module-private (guid-powered) | OVERLAYS WITHIN TVC DAP |
| DELVSUMP | CSM | eq DELVSUM +6 | module-private (guid-powered) | OVERLAYS WITHIN TVC DAP |
| DELVTPF | CSM+LM | eq DELVTPI +2 | module-private (prog-rendz; also io-nouns, sys-misc) | RENDEZVOUS -P34-35 (26D) |
| DELVTPI | CSM+LM | eq ACTCENT +2 | module-private (prog-rendz; also io-nouns) | RENDEZVOUS -P34-35 (26D) |
| DELVX | CSM+LM | eq DELV | module-private (imu; also guid-powered, sys-misc) | STANDBY VERB ERASABLES. REDOCTR BEFORE THETAD (DWN |
| DELVY | CSM+LM | eq DELV +2 | module-private (imu; also guid-powered, sys-misc) | STANDBY VERB ERASABLES. REDOCTR BEFORE THETAD (DWN |
| DELVZ | CSM+LM | eq DELV +4 | module-private (imu; also guid-powered, sys-misc) | STANDBY VERB ERASABLES. REDOCTR BEFORE THETAD (DWN |
| DELX | CSM+LM | eq END-W | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| DELYBAR | CSM | eq DELPBAR +2 | module-private (dap-csm; also guid-powered) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| DEPREV | CSM+LM | eq DELDEP +2 | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| DERCOF+1 | LM | eq MPAC -1 | module-private (unreferenced in code; overlay alias or spare) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| DERCOF-1 | LM | eq MPAC -3 | module-private (unreferenced in code; overlay alias or spare) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| DERCOF-2 | LM | eq MPAC -4 | module-private (unreferenced in code; overlay alias or spare) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| DERCOF-3 | LM | eq MPAC -5 | module-private (unreferenced in code; overlay alias or spare) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| DERCOF-4 | LM | eq MPAC -6 | module-private (unreferenced in code; overlay alias or spare) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| DERCOF-5 | LM | eq MPAC -7 | module-private (unreferenced in code; overlay alias or spare) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| DERCOF-6 | LM | eq MPAC -10 | module-private (unreferenced in code; overlay alias or spare) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| DERCOF-7 | LM | eq MPAC -11 | module-private (unreferenced in code; overlay alias or spare) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| DERCOF-8 | LM | eq MPAC -12 | module-private (unreferenced in code; overlay alias or spare) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| DERCOFN | LM | eq MPAC -2 | module-private (guid-landing) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| DERPTR | LM | eq TEM1 | module-private (guid-landing) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| DESCOUNT | LM | 1 | module-private (prog-rendz) | UNSWITCHED RADAR ERASABLE |
| DESOPMOD | CSM | 1 | module-private (sys-misc) | MODE SWITCHING ERASABLE (14D) |
| DESOPTS | CSM | 1 | module-private (sys-misc; also imu, prog-align) | STANDBY VERB ERASABLES. REDOCTR BEFORE THETAD (DWN |
| DESOPTT | CSM | 1 | module-private (sys-misc; also imu, prog-align) | STANDBY VERB ERASABLES. REDOCTR BEFORE THETAD (DWN |
| DESRET | LM | 1 | module-private (prog-rendz; also sys-misc) | UNSWITCHED RADAR ERASABLE |
| DEX1 | CSM+LM | eq TEM3 | module-private (guid-powered) | AX*SR*T STORAGE. (6D) |
| DEX2 | CSM+LM | eq TEM4 | module-private (unreferenced in code; overlay alias or spare) | AX*SR*T STORAGE. (6D) |
| DEXDEX | CSM+LM | eq TEM2 | module-private (guid-powered) | AX*SR*T STORAGE. (6D) |
| DEXI | CSM+LM | eq DEX1 | module-private (guid-powered) | AX*SR*T STORAGE. (6D) |
| DFT | CSM | eq MERRORZ +2 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| DFT1 | CSM | eq DFT +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| DFT2 | CSM | eq DFT1 +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| DHOOK | CSM | eq LEQ +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| DIFEQCNT | CSM+LM | 44 | module-private (nav-integ) | STATE VECTORS FOR ORBIT INTEGRATION. (44D) |
| DIFF | CSM | eq DTEAROT +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| DIFFALT | CSM+LM | eq RPASS2 +6 | module-private (io-nouns; also prog-rendz, sys-misc) | P32 --- P33 (26D) |
| DIFFOLD | CSM | eq DIFF +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| DINDX | LM | eq ITEMP3 | module-private (dap-lm) | NEEDLER (ATTITUDE ERROR EIGHT BALL DISPLAY) STORAG |
| DISPDEX | CSM+LM | eq A | module-private (prog-rendz; also guid-powered, guid-landing) | SELF-CHECK ASSIGNMENTS (17D) |
| DISTEM | CSM+LM | eq VBUF | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| DKDB | LM | 1 | module-private (dap-lm; also exec-sched) | DAP PAD-LOADED DATA. (10D) |
| DKKAOSN | LM | 1 | module-private (exec-sched) | DAP PAD-LOADED DATA. (10D) |
| DKOMEGAN | LM | 1 | module-private (dap-lm; also exec-sched) | DAP PAD-LOADED DATA. (10D) |
| DKTRAP | LM | 1 | module-private (dap-lm; also exec-sched) | DAP PAD-LOADED DATA. (10D) |
| DLEWD | CSM | eq D +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| DMPNTEMP | CSM+LM | eq MPTEMP | module-private (core-numeric) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| DNINDEX | LM | eq DNRRDOT +1 | module-private (prog-rendz) | RENDEZVOUS AND LANDING RADAR DOWNLINK STORAGE. (7D |
| DNLRALT | LM | eq DNLRVELZ +1 | module-private (unreferenced in code; overlay alias or spare) | RENDEZVOUS AND LANDING RADAR DOWNLINK STORAGE. (7D |
| DNLRVELX | LM | eq DNINDEX +1 | module-private (sys-misc) | RENDEZVOUS AND LANDING RADAR DOWNLINK STORAGE. (7D |
| DNLRVELY | LM | eq DNLRVELX +1 | module-private (unreferenced in code; overlay alias or spare) | RENDEZVOUS AND LANDING RADAR DOWNLINK STORAGE. (7D |
| DNLRVELZ | LM | eq DNLRVELY +1 | module-private (sys-misc) | RENDEZVOUS AND LANDING RADAR DOWNLINK STORAGE. (7D |
| DNLSTADR | CSM+LM | eq DNLSTCOD | module-private (unreferenced in code; overlay alias or spare) | DOWNLINK STORAGE (18D) |
| DNLSTCOD | CSM+LM | 1 | module-private (sys-misc; also exec-sched, guid-entry, prog-align) | DOWNLINK STORAGE (18D) |
| DNQ | CSM+LM | 1 | module-private (sys-misc) | DOWNLINK STORAGE (18D) |
| DNRNGERR | CSM | eq RTGO +2 | module-private (guid-entry; also io-nouns) | REENTRY ERASABLES. (206D) |
| DNRRANGE | LM | 7 | module-private (prog-rendz; also sys-misc) | RENDEZVOUS AND LANDING RADAR DOWNLINK STORAGE. (7D |
| DNRRDOT | LM | eq DNRRANGE +1 | module-private (unreferenced in code; overlay alias or spare) | RENDEZVOUS AND LANDING RADAR DOWNLINK STORAGE. (7D |
| DNTMBUFF | CSM+LM | 12 | module-private (sys-misc; also exec-sched) | DOWNLINK STORAGE (18D) |
| DNTMGOTO | CSM+LM | 1 | module-private (sys-misc; also exec-sched) | DOWNLINK STORAGE (18D) |
| DOTINC | CSM+LM | 1 | module-private (core-numeric) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| DOTRET | CSM+LM | 1 | module-private (core-numeric) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| DOWNTORK | LM | 6 | module-private (dap-lm) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| DPIPAY | CSM+LM | eq AINLA +54D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| DPIPAZ | CSM+LM | eq AINLA +58D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| DPNDX | CSM | eq SPNDX +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| DPSBURN | LM | eq DAPTREG4 | module-private (unreferenced in code; overlay alias or spare) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| DRCON | CSM | eq RD +2 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| DRDOT | LM | eq Y +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| DRHO | CSM | eq DFT2 +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| DRHO1 | CSM | eq DRHO +2 | module-private (unreferenced in code; overlay alias or spare) | OVERLAYS WITHIN TVC DAP |
| DRHO2 | CSM | eq DRHO1 +2 | module-private (unreferenced in code; overlay alias or spare) | OVERLAYS WITHIN TVC DAP |
| DRIFTI | CSM+LM | eq AINLA +40D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| DRIFTO | CSM+LM | eq AINLA +38D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| DRIFTT | CSM+LM | eq AINLA +6D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| DSEXIT | CSM+LM | eq INTB15+ | module-private (io-pinball) | UNUSED ERASABLES ********(2) |
| DSMAG | CSM+LM | eq TEM2 | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| DSPCNT | CSM+LM | 1 | module-private (exec-sched; also sys-misc) | PINBALL INTERRUPT ACTION (1D) |
| DSPCOUNT | CSM+LM | 1 | module-private (io-pinball; also exec-sched, sys-misc) | PINBALL EXECUTIVE ACTION (44D) |
| DSPLIST | CSM+LM | 1 | module-private (exec-sched; also io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| DSPLOCK | CSM+LM | 1 | module-private (io-pinball; also exec-sched) | PINBALL EXECUTIVE ACTION (44D) |
| DSPMMTEM | CSM+LM | eq MATINC | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| DSPTAB | CSM+LM | 12 | `agc_erase_dsptab(i)` / `agc_erase_set_dsptab(i,v)` (0..11) | PINBALL EXECUTIVE ACTION (44D) |
| DSPTEM1 | CSM+LM | 3 | module-private (prog-rendz; also prog-align, io-nouns, io-pinball, sys-misc...) | PINBALL EXECUTIVE ACTION (44D) |
| DSPTEM2 | CSM+LM | 3 | module-private (sys-misc; also io-nouns, imu) | PINBALL EXECUTIVE ACTION (44D) |
| DSPTEMX | CSM+LM | eq DSPTEM2 +1 | module-private (io-nouns; also prog-rendz, prog-align) | PINBALL EXECUTIVE ACTION (44D) |
| DSPWDRET | CSM+LM | eq TEM4 | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| DSREL | CSM+LM | eq TEM1 | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| DSRUPTEM | CSM+LM | eq RUPTREG4 | module-private (sys-misc) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| DSRUPTSW | CSM+LM | 1 | module-private (exec-sched; also sys-misc) | T4RUPT ERASABLE (6D) |
| DT | LM | eq LADQSAVE +1 | module-private (guid-landing) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| DT/2 | CSM+LM | eq TAU. +2 | module-private (nav-integ) | CONIC INTEGRATION STORAGE. -- MAY NOT SHARE WITH S |
| DT21PR | CSM | eq R(T1) +2 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| DTEAROT | CSM | eq UBZ/2 +6 | module-private (guid-entry; also prog-align) | REENTRY ERASABLES. (206D) |
| DTHETASM | CSM+LM | eq DCDU +6 | module-private (guid-powered; also prog-rendz) | OVERLAYS WITHIN TVC DAP |
| DUMPCNT | CSM+LM | 1 | module-private (sys-misc; also exec-sched) | DOWNLINK STORAGE (18D) |
| DUMPLOC | CSM+LM | eq TMINDEX | module-private (sys-misc) | DOWNLINK STORAGE (18D) |
| DV | CSM | eq V2(T1)/ +6 | module-private (guid-entry; also dap-csm, core-numeric, guid-powered, sys-misc) | RETURN-TO-EARTH STORAGE. (93D) |
| DV47TEMP | CSM | eq VG | module-private (guid-powered) | P47 STORAGE. |
| DVCNTR | CSM+LM | eq TGO +2 | module-private (guid-landing; also guid-powered, exec-sched, guid-entry, sys-misc) | SERVICER STORAGE. (13D) |
| DVL | CSM | eq AHOOKDV +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| DVLOS | CSM+LM | eq TSIGHT +2 | module-private (io-nouns; also prog-rendz) | RENDEZVOUS -P34-35 (26D) |
| DVNORMCT | CSM+LM | eq DOTRET | module-private (core-numeric) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| DVPREV | CSM+LM | eq GAMPREV +2 | module-private (prog-rendz) | THE FOLLOWING OVERLAY MEAS. INCORP. ARE AND IN USE |
| DVSIGN | CSM+LM | eq DOTINC | module-private (core-numeric) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| DVTHRESH | CSM | 1 | module-private (guid-powered) | P40 *** STEERING ROUTINE *** PAD LOAD (1D) |
| DVTHRUSH | LM | eq PGUIDE +2 | module-private (guid-landing; also guid-powered) | SERVICER -- MUST FOLLOW PIPTIME -- (19D) |
| DVTOTAL | CSM+LM | eq END-DELV | module-private (guid-powered; also io-nouns) | SERVICER STORAGE. (13D) |
| DXCRIT | LM | eq VBUF +2 | module-private (guid-landing) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| DXCRIT+1 | LM | eq VBUF +3 | module-private (unreferenced in code; overlay alias or spare) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| DXERROR | LM | 6 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| DYDOT | LM | eq DRDOT +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| DYERROR | LM | eq DXERROR +2 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| DZDOT | LM | eq DYDOT +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| DZERROR | LM | eq DXERROR +4 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| E | LM | eq DAPTREG6 | module-private (dap-lm) | THE FOLLOWING LM DAP ERASABLES ARE ZEROED IN THE S |
| E0 | LM | eq MX | module-private (unreferenced in code; overlay alias or spare) | LRS22.3 STORAGE. (CAN SHARE WITH P30'S AND OVERLAY |
| E01 | LM | eq MFS | module-private (unreferenced in code; overlay alias or spare) | KALCMANU STORAGE, CAN OVERLAY GSAV. (18D) |
| E02 | LM | eq E01 +6 | module-private (unreferenced in code; overlay alias or spare) | KALCMANU STORAGE, CAN OVERLAY GSAV. (18D) |
| E1 | LM | eq MX +2 | module-private (unreferenced in code; overlay alias or spare) | LRS22.3 STORAGE. (CAN SHARE WITH P30'S AND OVERLAY |
| E2 | CSM+LM | eq VPASS4 +6 | module-private (guid-entry; also nav-integ, prog-align, prog-rendz) | TPI SEARCH (P17) (6D) |
| E2DPS | LM | eq OURPERMS | module-private (guid-landing) | THESE ERASABLES MAY BE SHARED WITH CARE |
| E3 | LM | eq E2 +2 | module-private (nav-integ; also prog-rendz) | LRS22.3 STORAGE. (CAN SHARE WITH P30'S AND OVERLAY |
| E32C31RM | CSM+LM | eq E3J22R2M +1 | module-private (nav-integ) | P40 PAD LOADS (6D) |
| E3J22R2M | CSM+LM | eq FANG +2 | module-private (nav-integ; also sys-misc) | P40 PAD LOADS (6D) |
| E7OVERLA | LM | eq  | module-private (unreferenced in code; overlay alias or spare) | P32-P35, P72-P75 STORAGE. -- PERMANENT -- (6) |
| EBANKSAV | CSM+LM | 1 | module-private (io-pinball) | UNSWITCHED FOR DISPLAY INTERFACE ROUTINES. (5D) |
| EBANKTEM | CSM+LM | 1 | module-private (exec-sched; also io-pinball) | UNSWITCHED FOR DISPLAY INTERFACE ROUTINES. (5D) |
| EBUF2 | CSM+LM | 2 | module-private (sys-misc) | FOR EXCLUSIVE USE OF SYS TEST STANDARD LEAD INS (2 |
| ECC | CSM+LM | eq VVEC +6 | module-private (nav-conic; also prog-rendz) | CONICS ROUTINE STORAGE. (87D) |
| ECDUW | LM | eq MIS | module-private (guid-landing) | OVERLAYING KALCMANU STORAGE: (26D) |
| ECDUWUSR | LM | eq ECDUW | module-private (guid-landing) | OVERLAYING KALCMANU STORAGE: (26D) |
| ECSTEER | CSM | eq DELLT4 +2 | module-private (guid-powered; also guid-entry, sys-misc) | EXTERNAL DELTA-V UPDATE. (21D) |
| EDOT | CSM+LM | eq YNDX | module-private (dap-lm; also dap-csm) | OVERLAYS WITHIN TVC DAP |
| EDOTP | LM | eq EDOT | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| EDOTQ | LM | 2 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| EDOTR | LM | eq EDOTQ +1 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| EDOTSQ | LM | eq DAPTEMP1 | module-private (dap-lm) | THE FOLLOWING LM DAP ERASABLES ARE ZEROED IN THE S |
| EDOTVEL | CSM | eq AERR +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| EDRIVEX | CSM+LM | 1 | module-private (dap-csm; also dap-lm) | R22 PAD LOADS (5D). |
| EDRIVEY | CSM+LM | 1 | module-private (dap-csm; also dap-lm) | R22 PAD LOADS (5D). |
| EDRIVEZ | CSM+LM | 1 | module-private (dap-csm; also dap-lm) | R22 PAD LOADS (5D). |
| EGRESS | CSM+LM | eq P21TIME +2 | module-private (nav-aux; also nav-integ, prog-rendz) | INTEGRATION STORAGE (1D) |
| EK1VAL | CSM | 2 | module-private (guid-powered) | P40 PAD LOADS (6D) |
| EK2VAL | CSM | 2 | module-private (guid-powered) | P40 PAD LOADS (6D) |
| EK3VAL | CSM | 1 | module-private (guid-powered) | P40 PAD LOADS (6D) |
| EKPRIME | CSM | 2 | module-private (dap-csm) | R22 PAD LOADS (5D). |
| EKTLX/I | CSM | 3 | module-private (dap-csm) | R22 PAD LOADS (5D). |
| EL | LM | eq AZ +1D | module-private (io-nouns) | NOUN 87 (2D) |
| ELEV | CSM+LM | eq MRKBUF2 +7 | module-private (prog-rendz; also io-nouns, sys-misc) | RENDEZVOUS GUIDANCE STORAGE. -- P32...P35 -- (10D) |
| ELIDUMMY | LM | eq TTF/8 +2 | module-private (unreferenced in code; overlay alias or spare) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| ELINCR | LM | eq TTF/8TMP +2 | module-private (guid-landing) | THESE ERASABLES MAY BE SHARED WITH CARE |
| ELINCR1 | LM | eq AZINCR1 +1 | module-private (guid-landing) | ERASABLES FOR P64: OVERLAY OF DTHETASM, WHICH IS U |
| ELVDUMMY | LM | eq ZERDUMMY +1 | module-private (unreferenced in code; overlay alias or spare) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| ELVIRA | LM | eq ZERLINA +1 | module-private (exec-sched; also guid-landing) | ERASABLES FOR P64: OVERLAY OF DTHETASM, WHICH IS U |
| EMATRIX | CSM | eq W +120D | module-private (prog-rendz) | W-MATRIX STORAGE. (162D) |
| EMDOT | CSM | 1 | module-private (dap-csm; also guid-entry, guid-powered) | FLAGWORD RESERVATIONS. (12D) |
| EMSALT | CSM | 2 | module-private (guid-entry; also prog-align) | ENTRY STORAGE -- PAD LOADED -- (2D) |
| END-ALIG | LM | eq ZSMD +6 | module-private (unreferenced in code; overlay alias or spare) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| END-DELV | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | EXTERNAL DELTA-V UPDATE. (21D) |
| END-E3 | CSM+LM | eq E32C31RM | module-private (unreferenced in code; overlay alias or spare) | P40 PAD LOADS (6D) |
| END-E4 | CSM+LM | eq INTVAR | module-private (unreferenced in code; overlay alias or spare) | PAD LOAD INTEGRATION ERROR INCLUDED IN VARIANCE BY |
| END-E5 | CSM+LM | eq QMIN | module-private (unreferenced in code; overlay alias or spare) | P32,P33 STORAGE OVERLAYING 9X9 W-MATRIX LOCATIONS  |
| END-E6 | CSM+LM | eq P37RANGE +1 | module-private (unreferenced in code; overlay alias or spare) | P37 PAD LOADS (1) |
| END-E7 | CSM+LM | eq WHOCARES | module-private (exec-sched; also nav-aux) | P21 STORAGE (19D) |
| END-E7.0 | LM | eq IRETURN1 +1 | module-private (unreferenced in code; overlay alias or spare) | END OF THE ASCENT GUIDANCE ERASABLES |
| END-E7.1 | LM | eq N49FLAG +1 | module-private (unreferenced in code; overlay alias or spare) | END OF THE ASCENT GUIDANCE ERASABLES |
| END-E7.2 | LM | eq POINTVSM +6 | module-private (unreferenced in code; overlay alias or spare) | END OF THE ASCENT GUIDANCE ERASABLES |
| END-E7.3 | LM | eq END-ALIG | module-private (unreferenced in code; overlay alias or spare) | END OF THE ASCENT GUIDANCE ERASABLES |
| END-E7.4 | LM | eq 3777 | module-private (unreferenced in code; overlay alias or spare) | END OF THE ASCENT GUIDANCE ERASABLES |
| END-E7.5 | LM | eq TXO +2 | module-private (unreferenced in code; overlay alias or spare) | END OF THE ASCENT GUIDANCE ERASABLES |
| END-IN/M | CSM+LM | eq COZY4 +2 | module-private (unreferenced in code; overlay alias or spare) | THE FOLLOWING OVERLAYS MEASUREMENT INCORP AND CANN |
| END-KALC | CSM | eq MFS +18D | module-private (unreferenced in code; overlay alias or spare) | KALCMANU STORAGE. (18D) |
| END-P30S | CSM | eq TTPI +2 | module-private (unreferenced in code; overlay alias or spare) | P30-P40 STORAGE. (4D) |
| END-SVCR | CSM | eq NOMTIG +2 | module-private (unreferenced in code; overlay alias or spare) | SERVICER STORAGE. (13D) |
| END-UE | CSM+LM | eq SELFERAS +16D | module-private (unreferenced in code; overlay alias or spare) | P20 ALTERNATE LOS VARIANCE PAD LOAD **** (1D) |
| END-W | CSM | eq W +162D | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX STORAGE. (162D) |
| ENDBUF | CSM | eq CMTMTIME +16D | module-private (guid-entry) | MORE P27 STORAGE. (2D) |
| ENDW | LM | eq W +162D | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| ENGOFFDT | LM | eq XRANGE +2 | module-private (guid-landing) | ASCENT GUIDANCE ERASABLES. (21D) |
| ENTRET | CSM+LM | eq DOTINC | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| ENTRYVN | CSM | eq GOBL1/2 +6 | module-private (guid-entry) | ENTRY STORAGE (1D) |
| EOURPERM | LM | eq FUNNYDSP +2 | module-private (unreferenced in code; overlay alias or spare) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| EPSILONL | CSM+LM | eq COGA +2 | module-private (nav-conic) | MORE CONICS STORAGE. (4) |
| EPSILONT | CSM+LM | 2 | module-private (nav-conic) | KEPLER STORAGE (6D) |
| ERADM | CSM+LM | eq VECTAB +18D | module-private (nav-aux; also guid-entry, prog-rendz) | THESE PROBABLY CAN SHARE INTEGRATION VARIABLES (9D |
| ERASFILL | CSM | 2 | module-private (unreferenced in code; overlay alias or spare) | UNUSED ERASABLES ********(2) |
| ERCNT | CSM+LM | eq POLISH | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| ERCOMP | CSM+LM | eq AINLA +131D | module-private (guid-entry; also imu, sys-misc) | SYSTEM TEST STORAGE. (174) |
| ERCOMP1 | CSM | eq AINLA +131D | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| ERCOUNT | CSM+LM | eq ALMCADR +2 | module-private (exec-sched; also sys-misc) | SELF-CHECK ASSIGNMENTS (17D) |
| ERECTIM1 | CSM | eq AINLA +130D | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| ERECTIME | CSM | eq AINLA +130D | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| EREPFRAC | CSM | 2 | module-private (dap-csm; also guid-powered) | R22 PAD LOADS (5D). |
| ERESTORE | CSM+LM | eq SFAIL +1 | module-private (exec-sched; also sys-misc) | SELF-CHECK ASSIGNMENTS (17D) |
| ERRBTMP | CSM | eq DELBRTMP +2 | module-private (dap-csm; also guid-powered) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| ERRORX | CSM | eq BIAS2 +1 | module-private (dap-csm; also sys-misc) | OVERLAYS WITHIN TVC DAP |
| ERRORY | CSM | eq ERRORX +1 | module-private (dap-csm; also guid-powered) | OVERLAYS WITHIN TVC DAP |
| ERRORZ | CSM | eq ERRORY +1 | module-private (dap-csm; also guid-entry) | OVERLAYS WITHIN TVC DAP |
| ERVECTOR | CSM+LM | 6 | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| ESCAPE | CSM+LM | eq DOTINC | module-private (core-numeric) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| ESCAPE2 | CSM+LM | eq DOTRET | module-private (core-numeric) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| ESTROKER | CSM | 1 | module-private (exec-sched; also dap-csm, prog-rendz) | R22 PAD LOADS (5D). |
| ETDECAY | CSM | 1 | module-private (guid-powered) | R22 PAD LOADS (5D). |
| EXECTEM1 | CSM+LM | eq ITEMP1 | module-private (exec-sched) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| EXECTEM2 | CSM+LM | eq ITEMP2 | module-private (exec-sched) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| EXITEM | CSM+LM | eq INTB15+ | module-private (io-pinball) | UNUSED ERASABLES ********(2) |
| EXTVBACT | CSM+LM | 1 | module-private (prog-rendz; also prog-align, sys-misc, exec-sched, io-pinball...) | PINBALL EXECUTIVE ACTION (44D) |
| F | CSM+LM | eq VGTIG +6 | module-private (guid-powered; also dap-lm) | S40.1 STORAGE. (23D) |
| FACT1 | CSM | eq FACTOR +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| FACT2 | CSM | eq FACT1 +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| FACTOR | CSM | eq DIFFOLD +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| FAILREG | CSM+LM | 3 | `agc_erase_failreg(i)` / `agc_erase_set_failreg(i,v)` (0..2) | UNSWITCHED FOR DISPLAY INTERFACE ROUTINES. (10D) |
| FANG | CSM | 1 | module-private (guid-powered) | P40 PAD LOADS (6D) |
| FC | LM | eq PSEUDO55 +1 | module-private (guid-landing; also sys-misc) | THESE ERASABLES MUST NOT OVERLAY GOBLTIME OR SERVI |
| FCODD | LM | eq /AFC/ +2 | module-private (guid-landing) | LUNAR LANDING OVERLAYS (6D) |
| FCOLD | LM | eq TTHROT +1 | module-private (guid-landing) | THESE ERASABLES MUST NOT OVERLAY GOBLTIME OR SERVI |
| FDAIX | LM | 1 | module-private (imu; also io-nouns) | VARIOUS DISPLAY REGISTERS. BALLANGS (3D) |
| FDAIY | LM | 1 | module-private (unreferenced in code; overlay alias or spare) | VARIOUS DISPLAY REGISTERS. BALLANGS (3D) |
| FDAIZ | LM | 1 | module-private (unreferenced in code; overlay alias or spare) | VARIOUS DISPLAY REGISTERS. BALLANGS (3D) |
| FILDELV | CSM+LM | eq THETAN +6 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| FILDELV1 | CSM | eq THETAN1 +6 | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| FILLER | LM | eq WCHPHOLD +1 | module-private (unreferenced in code; overlay alias or spare) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| FIREDB | LM | eq 1/ANET1 +10D | module-private (dap-lm) | INPUT TO TJET LAW (PERMANENT ERASABLES). (48D) |
| FIREFCT | LM | eq DAPTEMP3 | module-private (dap-lm) | THE FOLLOWING LM DAP ERASABLES ARE ZEROED IN THE S |
| FIXLOC | CSM+LM | 1 | module-private (sys-misc; also prog-align, prog-rendz, core-numeric, exec-sched...) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| FLAGOODW | LM | eq NDXCDUW +1 | module-private (guid-landing) | OVERLAYING KALCMANU STORAGE: (26D) |
| FLAT | LM | eq BLOCKTOP +6 | module-private (dap-lm) | INPUT TO TJET LAW (PERMANENT ERASABLES). (48D) |
| FLPASS0 | LM | eq FILLER +1 | module-private (guid-landing; also guid-powered) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| FLPAUTNO | LM | eq FLAGOODW +1 | module-private (guid-landing) | OVERLAYING KALCMANU STORAGE: (26D) |
| FORVEL | LM | eq LATVEL +1 | module-private (guid-landing) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| FORVMETR | LM | eq LATVMETR +1 | module-private (guid-landing) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| FP | LM | eq FCODD +2 | module-private (guid-landing) | LUNAR LANDING OVERLAYS (6D) |
| FREERET | CSM+LM | eq TEM4 | module-private (unreferenced in code; overlay alias or spare) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| FUELNEED | LM | eq FUNNYDSP | module-private (unreferenced in code; overlay alias or spare) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| FUNCTION | LM | eq GTSTEMPS +10D | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| FUNNYDSP | LM | eq DELTAH +2 | module-private (io-nouns) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| FV | CSM+LM | eq PSIV +6 | module-private (nav-integ) | INTEGRATION STORAGE. (95D) |
| FWEIGHT | LM | eq RTNHOLD +1 | module-private (guid-landing) | THESE ERASABLES MUST NOT OVERLAY GOBLTIME OR SERVI |
| FX | CSM | eq GRAD +1 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| G(CSM) | LM | eq GEFF +2 | module-private (unreferenced in code; overlay alias or spare) | SERVICER FOR LUNAR ASCENT AND DESCENT (14D) |
| G21 | CSM+LM | eq KV1 +4 | module-private (unreferenced in code; overlay alias or spare) | SECOND-ORDER OVERLAYS IN KALCMANU |
| GACC | LM | eq STARAD | module-private (prog-align) | P57 ALIGNMENT (OVERLAY OF ALIGNMENT/SYSTEST COMMON |
| GAINAPPR | LM | eq JAPFG* +2 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| GAINBRAK | LM | eq JBRFG* +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| GAMA | CSM | eq JETEM +2 | module-private (guid-entry) | MISCELLANEOUS PERMANENT ERASABLE. |
| GAMDOT | CSM | eq GAMA +1 | module-private (guid-entry) | MISCELLANEOUS PERMANENT ERASABLE. |
| GAMMA | CSM+LM | eq TX789 +6 | module-private (nav-aux) | MEASUREMENT INCORPORATION STORAGE (R22) STORAGE. ( |
| GAMMAEI | CSM | eq VPRED +2 | module-private (guid-entry; also io-nouns, sys-misc) | REENTRY, RETURN TO EARTH COMMON DISPLAY (4D) |
| GAMMAL | CSM | eq GAMMAEI | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| GAMMAL1 | CSM | eq 22D | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| GAMMASB | CSM | eq RHOSB +2 | module-private (io-nouns; also prog-align) | S-BAND ANTENNA GIMBAL ANGLES. DISPLAYED BY R05 (V6 |
| GAMPREV | CSM+LM | eq DELDV +2 | module-private (prog-rendz) | THE FOLLOWING OVERLAY MEAS. INCORP. ARE AND IN USE |
| GBIASX | CSM | eq NBDX | module-private (unreferenced in code; overlay alias or spare) | IMU COMPENSATION PARAMETERS (22D) |
| GCOMP | CSM+LM | 6 | module-private (imu; also prog-align, sys-misc) | IMU COMPENSATION PARAMETERS (22D) |
| GCOMPSW | CSM+LM | 1 | module-private (imu; also sys-misc) | IMU COMPENSATION PARAMETERS (22D) |
| GCTR | LM | eq STAR +6 | module-private (prog-align) | ALIGNMENT/SYSTEST COMMON STORAGE. (31D) |
| GDT/2 | CSM+LM | eq PIPTIME +2 | module-private (guid-powered; SERVICER, must follow PIPTIME) | EXCEED THE ERASABLE STORAGE REQUIRED BY RENDEZVOUS |
| GDT1/2 | CSM+LM | eq PIPTIME1 +2 | module-private (guid-powered; also guid-landing) | EXCEED THE ERASABLE STORAGE REQUIRED BY RENDEZVOUS |
| GEFF | LM | eq ZDOT +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| GENPL | CSM+LM | 135 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| GENRET | CSM+LM | eq RAD +2 | module-private (prog-rendz) | P21 STORAGE. (1D) |
| GEOCOMP1 | CSM | eq AINLA +127D | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| GEOCOMPS | CSM+LM | eq AINLA +127D | module-private (sys-misc) | SYSTEM TEST STORAGE. (174) |
| GEOMSGN | CSM+LM | eq TDESIRED +2 | module-private (nav-conic; also prog-rendz, guid-powered) | CONICS ROUTINE STORAGE. (87D) |
| GEOSAVE1 | CSM | eq AINLA +117D | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| GEOSAVED | CSM | eq AINLA +117D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| GMAX | CSM | eq KAT +2 | module-private (guid-entry; also io-nouns) | REENTRY ERASABLES. (206D) |
| GMODE | CSM+LM | eq H +2 | module-private (unreferenced in code; overlay alias or spare) | INTEGRATION STORAGE. (95D) |
| GNUR | LM | eq VN2 | module-private (guid-powered) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| GNUV | LM | eq VN2 | module-private (guid-powered) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| GOBL/2 | CSM | eq GDT/2 +6 | module-private (guid-powered) | EXCEED THE ERASABLE STORAGE REQUIRED BY RENDEZVOUS |
| GOBL1/2 | CSM | eq GDT1/2 +6 | module-private (guid-powered) | EXCEED THE ERASABLE STORAGE REQUIRED BY RENDEZVOUS |
| GOBLTIME | LM | eq DVTOTAL +2 | module-private (guid-powered) | SERVICER STORAGE. (69D) |
| GOTOADDR | CSM | eq K2ROLL +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| GOUT | LM | eq STARAD +6 | module-private (prog-align) | P57 ALIGNMENT (OVERLAY OF ALIGNMENT/SYSTEST COMMON |
| GRAD | CSM | eq MM +1 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| GRP2SVQ | CSM+LM | eq RCLP +6 | module-private (nav-integ) | MEASUREMENT INCORPORATION STORAGE (R22) STORAGE. ( |
| GSAV | LM | 18 | module-private (guid-landing; also prog-align, sys-misc) | ALIGNMENT PLANETARY -- INERTIAL TRANSFORMATION STO |
| GTSOPNDZ | CSM | eq ZERONDX | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| GTSTEMPS | LM | eq DAPTEMP1 | module-private (unreferenced in code; overlay alias or spare) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| GTSWTLT1 | CSM | eq AINLA +129D | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| GTSXTLST | CSM | eq AINLA +129D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| GYTOBETQ | CSM+LM | eq GENPL +22D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| H | CSM+LM | eq BETAM +2 | module-private (nav-integ; also guid-landing, guid-powered) | INTEGRATION STORAGE. (95D) |
| H*GHCR*T | LM | eq BUF +1 | module-private (unreferenced in code; overlay alias or spare) | SOME VARIABLES FOR SECOND DPS GUIDANCE. (34D) |
| HAFPA1 | CSM+LM | eq POSTCDH | module-private (prog-rendz) | THE FOLLOWING OVERLAY MEAS. INCORP. ARE AND IN USE |
| HALFARG | LM | eq GTSTEMPS +8D | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| HALFY | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | SINGLE PRECISION SUBROUTINE TEMPORARIES (3D) |
| HAPO | CSM+LM | eq KT | module-private (guid-powered; also io-nouns, prog-rendz, sys-misc) | P30 DISPLAY (4D) |
| HAPOX | CSM+LM | eq THETA(1) +2 | module-private (io-nouns; also prog-rendz, sys-misc) | V82 DISPLAY (4D) |
| HBD11/2 | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | R22 PAD LOADS (5D). |
| HBD12 | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | R22 PAD LOADS (5D). |
| HBD21/2 | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | R22 PAD LOADS (5D). |
| HBD22 | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | R22 PAD LOADS (5D). |
| HBD31/2 | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | R22 PAD LOADS (5D). |
| HBD32 | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | R22 PAD LOADS (5D). |
| HBEAMNB | LM | eq ANGTERM +6 | module-private (guid-powered) | SECOND DPS GUIDANCE (LUNAR LANDING) (OVERLAY P32-3 |
| HBN10 | CSM | 1 | module-private (dap-csm) | R22 PAD LOADS (5D). |
| HBN11/2 | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | R22 PAD LOADS (5D). |
| HBN12 | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | R22 PAD LOADS (5D). |
| HBN20 | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | R22 PAD LOADS (5D). |
| HBN21/2 | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | R22 PAD LOADS (5D). |
| HBN22 | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | R22 PAD LOADS (5D). |
| HBN30 | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | R22 PAD LOADS (5D). |
| HBN31/2 | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | R22 PAD LOADS (5D). |
| HBN32 | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | R22 PAD LOADS (5D). |
| HCALC | LM | eq DELVREF +6 | module-private (guid-powered; also io-nouns) | SERVICER STORAGE. (69D) |
| HCALC1 | LM | eq DELVROD +6 | module-private (guid-landing; also guid-powered, io-nouns) | NOUN 63 COMPONENT (2D) |
| HDISP | CSM | eq VDISP +2 | module-private (unreferenced in code; overlay alias or spare) | STORAGE FOR S11.1 |
| HDOT | CSM | eq ALTI +2 | module-private (guid-entry; also io-nouns) | SOME P11 DISPLAY REGISTERS (6D) |
| HDOTDISP | CSM+LM | eq HDISP +2 | module-private (guid-landing; also guid-powered, io-nouns) | STORAGE FOR S11.1 |
| HEADSUP | CSM | eq CM/GYMDT +1 | module-private (guid-entry; also io-nouns) | MISCELLANEOUS PERMANENT ERASABLE. |
| HH | LM | eq DAPTREG3 | module-private (dap-lm) | THE FOLLOWING LM DAP ERASABLES ARE ZEROED IN THE S |
| HIASCENT | LM | 1 | module-private (dap-lm; also exec-sched) | DAP PAD-LOADED DATA. (10D) |
| HIGHCRIT | LM | eq LOWCRIT +1 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| HITEMIN | CSM+LM | eq VBUF +1 | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| HITEMOUT | CSM+LM | eq YREGLP | module-private (io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| HMEAS | LM | eq VMEAS +2 | module-private (guid-powered; also sys-misc) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| HOLDFLAG | CSM | 1 | module-private (dap-csm; also imu, guid-powered, prog-rendz) | RCSDAP ERASABLE (1D) |
| HOLDQ | LM | eq DAPTREG1 | module-private (dap-lm) | THE FOLLOWING LM DAP ERASABLES ARE ZEROED IN THE S |
| HOLDW | CSM+LM | eq OMEGAM3 +6 | module-private (nav-aux) | MEASUREMENT INCORPORATION STORAGE. (66D) |
| HORIZALT | CSM | 2 | module-private (prog-rendz) | P23 *** PAD LOAD **** (2D) |
| HORIZON | CSM | eq LANDMARK +1 | module-private (io-nouns; also prog-rendz) | P22-P23 STORAGE. (8D) |
| HP | CSM | eq DELVEE +2 | module-private (prog-rendz) | TPI SEARCH (26D) |
| HPER | CSM+LM | eq HAPO +2 | module-private (guid-powered; also io-nouns, prog-rendz) | P30 DISPLAY (4D) |
| HPERMIN | CSM+LM | eq RANGE | module-private (prog-rendz) | EXTENDED VERB 82 STORAGE. (6D) |
| HPERX | CSM+LM | eq HAPOX +2 | module-private (io-nouns; also prog-rendz) | V82 DISPLAY (4D) |
| HPO | CSM | eq TFO +2 | module-private (prog-rendz) | TPI SEARCH (26D) |
| I | CSM | eq DELOGA +1 | module-private (dap-csm; also prog-rendz) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| IAVG | CSM | 1 | module-private (guid-powered) | R22 PAD LOADS (5D). |
| IAVG/TLX | CSM | 1 | module-private (dap-csm) | R22 PAD LOADS (5D). |
| IC | LM | eq DELVSIN | module-private (guid-powered) | ASTEER STORAGE. (22D) |
| IDAD1TEM | CSM+LM | 1 | module-private (io-nouns; also io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| IDAD2TEM | CSM+LM | 1 | module-private (io-nouns) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| IDAD3TEM | CSM+LM | 1 | module-private (unreferenced in code; overlay alias or spare) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| IDADDTEM | CSM+LM | eq TEM2 | module-private (unreferenced in code; overlay alias or spare) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| IDOFLMK | CSM | eq HORIZON +1 | module-private (unreferenced in code; overlay alias or spare) | P22-P23 STORAGE. (8D) |
| IG | LM | eq COFSKEW | module-private (unreferenced in code; overlay alias or spare) | FIRST-ODER OVERLAYS IN KALCMANU (25D) |
| IGC | CSM+LM | eq OGC +2 | module-private (imu; also guid-entry) | ALIGNMENT/SYSTEST/AUTO OPTICS COMMON STORAGE. (17D |
| IGNAOSQ | LM | 1 | module-private (guid-powered) | PADLOADS FOR INTITIALIZATION OF DAP BIAS ACCELERAT |
| IGNAOSR | LM | 1 | module-private (guid-powered) | PADLOADS FOR INTITIALIZATION OF DAP BIAS ACCELERAT |
| IGRET | LM | eq LGRET | module-private (prog-rendz) | LRS22.3 STORAGE. (CAN SHARE WITH P30'S AND OVERLAY |
| IMODES30 | CSM+LM | 1 | `agc_erase_imodes30()` / `agc_erase_set_imodes30()` | MODE SWITCHING ERASABLE (14D) |
| IMODES33 | CSM+LM | 1 | `agc_erase_imodes33()` / `agc_erase_set_imodes33()` | MODE SWITCHING ERASABLE (14D) |
| IMU/OPT | CSM | eq GENPL +4 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| IMUCADR | CSM+LM | eq MODECADR | module-private (exec-sched; also imu, prog-align) | MODE SWITCHING ERASABLE (14D) |
| INCORPEX | CSM+LM | eq ERADM +2 | module-private (nav-aux; also guid-entry) | THESE PROBABLY CAN SHARE INTEGRATION VARIABLES (9D |
| INDEP | CSM+LM | eq COGA | module-private (nav-conic) | MORE CONICS STORAGE. (4) |
| INDEXLOC | CSM+LM | eq BUF | module-private (core-numeric) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| INGTS | LM | eq QGIMTIMR +1 | module-private (dap-lm) | VARIABLES FOR GTS-QRAXIS CONTROL EXCHANGE. (4) |
| ININDEX | LM | eq GTSTEMPS +5 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| INREL | CSM+LM | eq DOTRET | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| INTB15+ | CSM+LM | 1 | module-private (dap-lm; also prog-rendz) | UNUSED ERASABLES ********(2) |
| INTBIT15 | CSM+LM | 1 | module-private (core-numeric) | UNUSED ERASABLES ********(2) |
| INTIME | CSM+LM | eq AXISCODE +3 | module-private (prog-rendz) | THE FOLLOWING OVERLAYS MEASUREMENT INCORP AND CANN |
| INTTEMP | CSM | 15 | module-private (unreferenced in code; overlay alias or spare) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| INTVAR | CSM | eq VONE' +6 | module-private (prog-rendz) | PAD LOAD INTEGRATION ERROR INCLUDED IN VARIANCE BY |
| INTVEC | CSM+LM | eq FILDELV +2 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| INTVEC1 | CSM | eq FILDELV1 +2 | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| INTWAK1Q | CSM+LM | eq UPTEMP | module-private (nav-integ) | MORE P27 STORAGE. (2D) |
| INTY | CSM+LM | eq AINLA +28D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| INTZ | CSM+LM | eq AINLA +32D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| IOGARATE | CSM | eq I +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| IRETURN | CSM+LM | eq GMODE +1 | module-private (nav-integ) | INTEGRATION STORAGE. (95D) |
| IRETURN1 | CSM+LM | 1 | module-private (nav-integ) | MISCELLANEOUS UNSWITCHED. (16D) |
| IT | CSM | eq DVLOS | module-private (prog-rendz) | TPI SEARCH (26D) |
| ITCTR | CSM+LM | eq INTIME +2 | module-private (prog-rendz) | THE FOLLOWING OVERLAYS MEASUREMENT INCORP AND CANN |
| ITEMP1 | CSM+LM | 1 | module-private (exec-sched; also guid-powered, sys-misc, imu, prog-align...) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| ITEMP2 | CSM+LM | 1 | module-private (exec-sched; also guid-powered, imu, sys-misc, dap-lm...) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| ITEMP3 | CSM+LM | 1 | module-private (exec-sched; also sys-misc, dap-lm, guid-landing, guid-powered...) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| ITEMP4 | CSM+LM | 1 | module-private (exec-sched; also imu, guid-landing, guid-powered, prog-align...) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| ITEMP5 | CSM+LM | 1 | module-private (exec-sched; also guid-landing, prog-align, prog-rendz) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| ITEMP6 | CSM+LM | 1 | module-private (exec-sched; also sys-misc, dap-lm, guid-landing, prog-rendz) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| IXX | CSM | 1 | module-private (dap-csm; also guid-powered) | R22 PAD LOADS (5D). |
| J/M | CSM | eq KMJ2 +1 | module-private (dap-csm; also guid-powered) | OVERLAYS WITHIN TVC DAP |
| J/M1 | CSM | eq J/M +1 | module-private (guid-powered) | OVERLAYS WITHIN TVC DAP |
| J/M2 | CSM | eq J/M1 +1 | module-private (guid-powered) | OVERLAYS WITHIN TVC DAP |
| JAPFG* | LM | eq AAPFG* +2 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| JBRFG* | LM | eq ABRFG* +2 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| JETAG | CSM | eq OLDELR +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| JETEM | CSM | eq ROLLHOLD +1 | module-private (guid-entry) | MISCELLANEOUS PERMANENT ERASABLE. |
| JETEM2 | CSM | eq CM/SAVE +1 | module-private (guid-entry) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| JETRATE | LM | eq DAPTREG1 | module-private (dap-lm) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| JETRATEQ | LM | eq JETRATE +1 | module-private (dap-lm) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| JETRATER | LM | eq JETRATE +2 | module-private (dap-lm) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| JJ | CSM | eq PREDANG +1 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| JNDX | CSM | eq VSQ/4API +1 | module-private (guid-entry) | THE FOLLOWING FEW REGISTERS USED ONCE EACH 2 SEC |
| JNDX1 | CSM | eq JNDX +1 | module-private (guid-entry) | THE FOLLOWING FEW REGISTERS USED ONCE EACH 2 SEC |
| JZERO | CSM | eq CMDTMP | module-private (unreferenced in code; overlay alias or spare) | OVERLAYS WITHIN TVC DAP |
| K1 | LM | eq KV1 | module-private (unreferenced in code; overlay alias or spare) | SECOND-ORDER OVERLAYS IN KALCMANU (24D) |
| K2 | LM | eq KV2 | module-private (unreferenced in code; overlay alias or spare) | SECOND-ORDER OVERLAYS IN KALCMANU (24D) |
| K2CNTRAL | LM | eq GTSTEMPS +3 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| K2ROLL | CSM | eq DLEWD +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| K2THETA | LM | eq GTSTEMPS | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| K3 | LM | eq KV3 | module-private (unreferenced in code; overlay alias or spare) | SECOND-ORDER OVERLAYS IN KALCMANU (24D) |
| KAT | CSM | eq DNRNGERR +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| KCENTRAL | LM | eq GTSTEMPS +2 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| KDPNDX | CSM+LM | eq KSPNDX +1 | module-private (imu) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| KEEP-2 | LM | eq AZINCR +2 | module-private (unreferenced in code; overlay alias or spare) | THESE ERASABLES MAY BE SHARED WITH CARE |
| KEL | LM | eq MFS | module-private (imu) | KALCMANU STORAGE, CAN OVERLAY GSAV. (18D) |
| KEPRTN | CSM+LM | eq ORIGEX | module-private (nav-conic; also nav-integ) | INTEGRATION STORAGE. (95D) |
| KEYTEMP1 | CSM+LM | eq RUPTREG4 | module-private (exec-sched; also io-pinball) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| KEYTEMP2 | CSM+LM | eq RUPTAGN | module-private (io-pinball) | WAITLIST REPEAT FLAG. (1D) |
| KIGNV/B4 | LM | eq KIGNY/B8 +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| KIGNX/B4 | LM | eq RIGNZ +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| KIGNY/B8 | LM | eq KIGNX/B4 +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| KLAT | CSM | eq L/DCMINR +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| KMJ | CSM | eq ATTKALMN +1 | module-private (dap-csm; also guid-powered) | OVERLAYS WITHIN TVC DAP |
| KMJ1 | CSM | eq KMJ +1 | module-private (guid-powered) | OVERLAYS WITHIN TVC DAP |
| KMJ2 | CSM | eq KMJ1 +1 | module-private (guid-powered) | OVERLAYS WITHIN TVC DAP |
| KMPAC | CSM | eq DPNDX +1 | module-private (dap-csm; also exec-sched) | OVERLAYS WITHIN TVC DAP |
| KMPTEMP | CSM | eq KMPAC +2 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| KPRIMEDT | CSM | eq MDT +6 | module-private (dap-csm; also guid-powered) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| KQ | LM | 3 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| KRDAP | LM | eq KQ +2 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| KSPNDX | CSM+LM | eq BCDU +3 | module-private (imu) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| KT | CSM+LM | eq LANDALT +2 | module-private (prog-rendz; also exec-sched) | S34/35.5,P34-P35 STORAGE. (6D) |
| KTLX/I | CSM | eq KPRIMEDT +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| KV1 | CSM+LM | eq TMIS | module-private (unreferenced in code; overlay alias or spare) | FIRST-ORDER OVERLAYS IN KALCMANU |
| KV2 | CSM+LM | eq TMIS +6 | module-private (unreferenced in code; overlay alias or spare) | FIRST-ORDER OVERLAYS IN KALCMANU |
| KV3 | CSM+LM | eq TMIS +12D | module-private (unreferenced in code; overlay alias or spare) | FIRST-ORDER OVERLAYS IN KALCMANU |
| L*WCR*T | LM | eq BUF | module-private (unreferenced in code; overlay alias or spare) | SOME VARIABLES FOR SECOND DPS GUIDANCE. (34D) |
| L,PVT-CG | LM | 1 | module-private (unreferenced in code; overlay alias or spare) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| L/D | CSM | eq KLAT +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| L/D1 | CSM | eq L/D +2 | module-private (guid-entry; also sys-misc) | REENTRY ERASABLES. (206D) |
| L/DCALC | CSM | eq TTE | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| L/DCMINR | CSM | eq LOD +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| LAD | CSM | eq VSQUARE +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| LADPAD | CSM | 1 | module-private (guid-entry) | R22 PAD LOADS (5D). |
| LADQSAVE | LM | eq ALTSAVE +2 | module-private (guid-landing) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| LAG/TAU | LM | eq TAUROD +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| LAND | LM | eq VGU +6 | module-private (guid-landing; also sys-misc) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| LANDALT | CSM+LM | eq LANDLONG +2 | module-private (io-nouns; also prog-align, prog-rendz) | P22 DISPLAY REGISTERS (06D) |
| LANDLAT | CSM+LM | eq LAT | module-private (io-nouns; also prog-align, prog-rendz) | UNSWITCHED FOR ORBIT INTEGRATION (21D) |
| LANDLONG | CSM+LM | eq AOPTIME +2 | module-private (io-nouns; also prog-align, prog-rendz) | P22 DISPLAY REGISTERS (06D) |
| LANDMARK | CSM | eq S22LOC +1 | module-private (exec-sched; also sys-misc, io-nouns, prog-align, prog-rendz) | P22-P23 STORAGE. (8D) |
| LANDTEMP | LM | eq OURTEMPS | module-private (guid-landing) | THESE ERASABLES MAY BE SHARED WITH CARE |
| LASTLADW | LM | eq RUNIT +2 | module-private (unreferenced in code; overlay alias or spare) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| LASTMASP | CSM | eq VARST0 +9D | module-private (unreferenced in code; overlay alias or spare) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| LASTTPIP | LM | eq RODSCAL1 +1 | module-private (guid-landing) | P66 ERASABLES (R.O.D.) (14D) |
| LASTXCMD | CSM+LM | eq OPTY | module-private (prog-rendz) | T4RUPT ERASABLE (6D) |
| LASTYCMD | CSM+LM | eq OPTY | module-private (prog-rendz; also sys-misc) | T4RUPT ERASABLE (6D) |
| LAT | CSM+LM | eq COLREG +1 | module-private (guid-entry; also io-nouns, nav-aux, guid-powered, imu...) | UNSWITCHED FOR ORBIT INTEGRATION (21D) |
| LAT(SPL) | CSM | 21 | module-private (unreferenced in code; overlay alias or spare) | EXTERNAL DELTA-V UPDATE. (21D) |
| LATANG | CSM | eq V +2 | module-private (guid-entry; also sys-misc) | REENTRY ERASABLES. (206D) |
| LATITUDE | CSM+LM | 2 | module-private (guid-entry; also imu, sys-misc) | SYSTEM TEST STORAGE. (174) |
| LATVEL | LM | eq FORVMETR +1 | module-private (guid-landing) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| LATVMETR | LM | eq STILBADV +1 | module-private (guid-landing) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| LAUNCHAZ | CSM | eq AINLA +119D | module-private (guid-entry; also imu, sys-misc) | SYSTEM TEST STORAGE. (174) |
| LAXIS | LM | eq /R/MAG +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| LCX/360 | CSM | eq -VT/180 +1 | module-private (guid-entry) | THE FOLLOWING FEW REGISTERS USED ONCE EACH 2 SEC |
| LDATALST | CSM+LM | 1 | module-private (sys-misc) | DOWNLINK STORAGE (18D) |
| LEADTIME | LM | 1 | module-private (guid-landing) | P63 AND P64 CONSTANTS. -- PAD LOADED -- (4D) |
| LEMMASS | CSM+LM | 1 | `agc_erase_lemmass()` / `agc_erase_set_lemmass()` | R22 PAD LOADS (5D). |
| LEMPOS | CSM | eq CSMPOS +6 | module-private (prog-rendz) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| LENGTHOT | CSM+LM | 1 | module-private (sys-misc; also imu) | SYSTEM TEST STORAGE. (174) |
| LEQ | CSM | eq FX +1 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| LEWD | CSM | eq VIO | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| LGRET | LM | eq RLMSRCH | module-private (prog-rendz) | LRS22.3 STORAGE. (CAN SHARE WITH P30'S AND OVERLAY |
| LGYRO | CSM+LM | 1 | module-private (imu; also exec-sched) | T4RUPT ERASABLE (6D) |
| LIFTTEMP | CSM | eq PADLONG +2 | module-private (guid-entry) | P11 STORAGE. (9D) |
| LMKAOSN | LM | 1 | module-private (exec-sched) | DAP PAD-LOADED DATA. (10D) |
| LMOMEGAN | LM | 1 | module-private (dap-lm; also exec-sched) | DAP PAD-LOADED DATA. (10D) |
| LMPOS | LM | eq RTSTDEX | module-private (prog-rendz) | LPS20.1 STORAGE. (12D) |
| LMTRAP | LM | 1 | module-private (dap-lm; also exec-sched) | DAP PAD-LOADED DATA. (10D) |
| LMVEL | LM | eq LMPOS +6 | module-private (prog-rendz) | LPS20.1 STORAGE. (12D) |
| LNCHTM | LM | eq NEWPOS +6 | module-private (prog-rendz) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| LNG(SPL) | CSM | eq LAT(SPL) +2 | module-private (unreferenced in code; overlay alias or spare) | EXTERNAL DELTA-V UPDATE. (21D) |
| LOADSTAT | CSM+LM | 1 | module-private (io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| LOC | CSM+LM | 1 | module-private (exec-sched; also io-pinball, sys-misc, core-numeric, imu...) | DYNAMICALLY ALLOCATED CORE SETS FOR JOBS (84D) |
| LOCCTR | CSM+LM | eq ITEMP4 | module-private (io-pinball; also exec-sched, imu, nav-integ, sys-misc) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| LOD | CSM | eq LAD +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| LODPAD | CSM | 1 | module-private (guid-entry) | R22 PAD LOADS (5D). |
| LONG | CSM+LM | eq LAT +2 | module-private (io-nouns; also nav-aux, guid-entry, prog-align, prog-rendz...) | UNSWITCHED FOR ORBIT INTEGRATION (21D) |
| LONGBASE | CSM+LM | 2 | module-private (exec-sched) | MISCELLANEOUS UNSWITCHED. (16D) |
| LONGCADR | CSM+LM | 2 | module-private (exec-sched) | MISCELLANEOUS UNSWITCHED. (16D) |
| LONGEXIT | CSM+LM | 2 | module-private (exec-sched) | MORE LONGCALL STORAGE. (MUST BE IN LST1'S BANK. (2 |
| LONGTIME | CSM+LM | 2 | module-private (exec-sched) | MISCELLANEOUS UNSWITCHED. (16D) |
| LOOKANGL | LM | eq FUNNYDSP +1 | module-private (guid-landing) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| LOOPCT | CSM+LM | eq RPASS1 +6 | module-private (prog-rendz) | ADDITIONAL CSI - CDH STORAGE. (10D) |
| LOS1 | CSM+LM | eq GENPL +8D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| LOS2 | CSM+LM | eq GENPL +14D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| LOSCOUNT | LM | 1 | module-private (sys-misc; also exec-sched, prog-rendz) | P30-P40 INTERFACE UNSHARED. (3D) |
| LOSDESRD | LM | eq VXRCM +6 | module-private (prog-rendz) | LRS24.1 STORAGE. (CAN SHARE WITH P30'S) (40D) |
| LOSVEC | CSM+LM | 6 | module-private (sys-misc; also imu) | SYSTEM TEST STORAGE. (174) |
| LOSVEL | LM | 6 | module-private (prog-rendz) | LPS20.1 STORAGE -- ALL ARE PRM -- (9D) |
| LOTEMIN | CSM+LM | eq VBUF +2 | module-private (unreferenced in code; overlay alias or spare) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| LOTEMOUT | CSM+LM | eq ZREGLP | module-private (io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| LOWCRIT | LM | eq KIGNV/B4 +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| LRADRET | LM | eq ELVDUMMY +1 | module-private (guid-powered) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| LRADRET1 | LM | eq VN2 | module-private (guid-powered) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| LRALPHA | LM | eq DELQFIX +2 | module-private (guid-powered) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| LRALPHA2 | LM | eq LRBETA1 +1 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| LRBETA1 | LM | eq LRALPHA +1 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| LRBETA2 | LM | eq LRALPHA2 +1 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| LRHMAX | LM | 1 | module-private (guid-powered) | LANDING RADAR -- PAD LOADED -- (2D) |
| LRLCTR | LM | eq EOURPERM | module-private (guid-powered) | R12 (FOR LUNAR LANDING) (6D) |
| LRMCTR | LM | eq LRRCTR +1 | module-private (guid-powered) | R12 (FOR LUNAR LANDING) (6D) |
| LRRCTR | LM | eq LRLCTR +1 | module-private (guid-powered) | R12 (FOR LUNAR LANDING) (6D) |
| LRS22.1X | LM | eq RRSHAFT +2 | module-private (prog-rendz) | LRS22.1 STORAGE. (MUST NOT SHARE WITH P30'S) (13D) |
| LRSCTR | LM | eq LRMCTR +1 | module-private (guid-powered) | R12 (FOR LUNAR LANDING) (6D) |
| LRVF | LM | eq LRVMAX +1 | module-private (guid-powered) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| LRVMAX | LM | eq LRBETA2 +1 | module-private (guid-powered) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| LRVTIMDL | LM | eq LRZCDUDL +1 | module-private (guid-powered; also sys-misc) | R12 DOWNLINK QUANTITIES (5D) |
| LRVTIME | LM | eq VXBEAMNB +6 | module-private (guid-powered) | CAN OVERLAY GSAV WITH CARE, USED DURING POWERED DE |
| LRWH | LM | 1 | module-private (guid-powered) | LANDING RADAR -- PAD LOADED -- (2D) |
| LRWVFF | LM | eq LRWVFX +1 | module-private (guid-powered) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| LRWVFX | LM | eq LRWVFY +1 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| LRWVFY | LM | eq LRWVFZ +1 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| LRWVFZ | LM | eq LRWVX +1 | module-private (guid-powered) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| LRWVX | LM | eq LRWVY +1 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| LRWVY | LM | eq LRWVZ +1 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| LRWVZ | LM | eq LRVF +1 | module-private (guid-powered) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| LRXCDU | LM | eq LRVTIME +2 | module-private (guid-powered) | CAN OVERLAY GSAV WITH CARE, USED DURING POWERED DE |
| LRXCDUDL | LM | eq /LAND/ +2 | module-private (guid-powered; also sys-misc) | R12 DOWNLINK QUANTITIES (5D) |
| LRYCDU | LM | eq LRXCDU +1 | module-private (guid-powered) | CAN OVERLAY GSAV WITH CARE, USED DURING POWERED DE |
| LRYCDUDL | LM | eq LRXCDUDL +1 | module-private (unreferenced in code; overlay alias or spare) | R12 DOWNLINK QUANTITIES (5D) |
| LRZCDU | LM | eq LRYCDU +1 | module-private (guid-powered) | CAN OVERLAY GSAV WITH CARE, USED DURING POWERED DE |
| LRZCDUDL | LM | eq LRYCDUDL +1 | module-private (guid-powered; also sys-misc) | R12 DOWNLINK QUANTITIES (5D) |
| LS21X | LM | 1 | module-private (prog-rendz) | LPS20.1 STORAGE -- ALL ARE PRM -- (9D) |
| LST1 | CSM+LM | 8 | module-private (exec-sched; also sys-misc) | WAITLIST TASK LISTS. (26D) |
| LST2 | CSM+LM | 18 | module-private (exec-sched; also guid-powered) | WAITLIST TASK LISTS. (26D) |
| LSTPTR | CSM+LM | eq TEM4 | module-private (unreferenced in code; overlay alias or spare) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| LUNCHAZ1 | CSM | eq AINLA +119D | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| M11 | LM | 1 | module-private (dap-lm; also exec-sched, sys-misc) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| M21 | LM | 1 | module-private (dap-lm; also sys-misc) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| M22 | LM | 1 | module-private (dap-lm; also guid-landing, sys-misc) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| M31 | LM | 1 | module-private (dap-lm; also sys-misc) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| M32 | LM | 1 | module-private (dap-lm; also guid-landing, sys-misc) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| MAGVEC2 | CSM+LM | eq U2 +6 | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| MAGVTPI | CSM | eq DELVEO +2 | module-private (prog-rendz) | TPI SEARCH (26D) |
| MAMAX1 | CSM | eq DT21PR +2 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| MAMAX2 | CSM | eq MAMAX1 +2 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| MARK2DWN | CSM | eq R53EXIT +1 | module-private (prog-align; also sys-misc) | ALIGNMENT MARKDATA (DOWNLINK) ******* (7D) |
| MARK2PAC | CSM+LM | 1 | module-private (io-pinball) | UNSWITCHED FOR DISPLAY INTERFACE ROUTINES. (5D) |
| MARKCNTR | LM | eq DELTAQ +2 | module-private (prog-align) | AOTMARK STORAGE (3D) |
| MARKCTR | CSM+LM | eq TRKMKCNT | module-private (prog-rendz) | MARK STORAGE. (2) |
| MARKDATA | CSM | eq UM +6 | module-private (prog-rendz) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| MARKDOWN | CSM | eq S22EORM +1 | module-private (prog-align; also sys-misc, prog-rendz) | DOWNLINK ERASABLES FOR P22, P20 MARK DATA. (8D) |
| MARKEBAN | CSM+LM | 1 | module-private (io-pinball) | UNSWITCHED FOR DISPLAY INTERFACE ROUTINES. (5D) |
| MARKINDX | CSM | eq QMAJ +1 | module-private (exec-sched; also prog-align, prog-rendz) | ALIGNMENT STORAGE. (5D) |
| MARKNV | CSM+LM | 1 | module-private (io-pinball) | UNSWITCHED FOR DISPLAY INTERFACE ROUTINES. (10D) |
| MARKSTAT | CSM+LM | 1 | `agc_erase_markstat()` / `agc_erase_set_markstat()` | MODE SWITCHING ERASABLE (14D) |
| MARKTIME | CSM | eq RCL +2 | module-private (prog-rendz) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| MASKREG | CSM+LM | eq GENPL +64D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| MASS | CSM+LM | eq WEIGHT/G | module-private (guid-landing; also dap-lm, guid-powered, sys-misc) | R22 PAD LOADS (5D). |
| MASS1 | LM | eq GDT1/2 +6 | module-private (guid-landing; also guid-powered) | SERVICER STORAGE. (69D) |
| MASSTMP | CSM | eq TVCEXPHS +1 | module-private (dap-csm; also exec-sched, guid-powered) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| MATINC | CSM+LM | 1 | module-private (core-numeric) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| MAXDVSW | CSM+LM | eq MATINC | module-private (core-numeric) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| MAXFORCE | LM | eq MINFORCE +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| MCDUYDOT | CSM | eq PCDUZPST +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| MCDUZDOT | CSM | eq MCDUYDOT +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| MDOT | LM | eq F +2 | module-private (guid-powered) | P40 STORAGE. (6D) |
| MDT | CSM | eq T5TVCDT +1 | module-private (unreferenced in code; overlay alias or spare) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| MERRORX | CSM | eq ADOT2 +2 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| MERRORY | CSM | eq MERRORX +2 | module-private (unreferenced in code; overlay alias or spare) | OVERLAYS WITHIN TVC DAP |
| MERRORZ | CSM | eq MERRORY +2 | module-private (unreferenced in code; overlay alias or spare) | OVERLAYS WITHIN TVC DAP |
| MFI | CSM+LM | eq MFS | module-private (imu) | KALCMANU STORAGE. (18D) |
| MFISYM | CSM+LM | eq TMIS | module-private (imu) | FIRST-ORDER OVERLAYS IN KALCMANU |
| MFS | CSM+LM | eq END-DELV | module-private (imu) | KALCMANU STORAGE. (18D) |
| MGC | CSM+LM | eq IGC +2 | module-private (imu; also guid-entry) | ALIGNMENT/SYSTEST/AUTO OPTICS COMMON STORAGE. (17D |
| MINDEX | CSM+LM | 1 | module-private (exec-sched) | VERB 37 STORAGE (2D) |
| MINFORCE | LM | eq LAG/TAU +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| MIS | CSM+LM | eq CAM +2 | module-private (imu) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| MIXBR | CSM+LM | eq MATINC | module-private (io-nouns; also io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| MIXTEMP | CSM+LM | eq VBUF +3 | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| MKCDUS | CSM | 1 | module-private (prog-align) | OPTICS MARKING, UNSHARED. (8D) |
| MKCDUT | CSM | 1 | module-private (prog-align) | OPTICS MARKING, UNSHARED. (8D) |
| MKCDUX | CSM | 1 | module-private (prog-align) | OPTICS MARKING, UNSHARED. (8D) |
| MKCDUY | CSM | 1 | module-private (prog-align) | OPTICS MARKING, UNSHARED. (8D) |
| MKCDUZ | CSM | 1 | module-private (prog-align) | OPTICS MARKING, UNSHARED. (8D) |
| MKDEX | LM | eq XYMARK +1 | module-private (prog-align) | AOTMARK STORAGE (3D) |
| MKNDX | CSM | 1 | module-private (prog-align) | OPTICS MARKING, UNSHARED. (8D) |
| MKT2T1 | CSM | 2 | module-private (prog-align) | OPTICS MARKING, UNSHARED. (8D) |
| MKTIME | LM | eq TANGNB +2 | module-private (guid-powered; also prog-rendz, sys-misc) | LRS22.1 (SAME AS PREVIOUS SECTION) ALSO DOWNLINK F |
| MLOSV | LM | 2 | module-private (prog-rendz) | LPS20.1 STORAGE -- ALL ARE PRM -- (9D) |
| MM | CSM | eq TEM1B +2 | module-private (guid-entry; also prog-align, sys-misc, exec-sched, guid-powered...) | REENTRY ERASABLES. (206D) |
| MMNUMBER | CSM+LM | 1 | module-private (exec-sched; also guid-landing) | VERB 37 STORAGE (2D) |
| MODE | CSM+LM | 1 | module-private (core-numeric; also imu, io-pinball, guid-entry, guid-landing...) | DYNAMICALLY ALLOCATED CORE SETS FOR JOBS (84D) |
| MODEA | LM | eq TANG | module-private (prog-align; also prog-rendz) | UNSWITCHED RADAR ERASABLE |
| MODEB | LM | 2 | module-private (prog-align; also prog-rendz) | UNSWITCHED RADAR ERASABLE |
| MODECADR | CSM+LM | 3 | module-private (imu; also sys-misc) | MODE SWITCHING ERASABLE (14D) |
| MODREG | CSM+LM | 1 | `agc_erase_modreg()` / `agc_erase_set_modreg()` | PINBALL EXECUTIVE ACTION (44D) |
| MONSAVE | CSM+LM | 1 | module-private (exec-sched; also io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| MONSAVE1 | CSM+LM | 1 | module-private (exec-sched; also io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| MONSAVE2 | CSM+LM | 1 | module-private (io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| MPAC | CSM+LM | 7 | module-private (core-numeric: MPAC is the interpreter accumulator, replaced by C locals and `agc_interp_t`) | DYNAMICALLY ALLOCATED CORE SETS FOR JOBS (84D) |
| MPTEMP | CSM+LM | 1 | module-private (core-numeric; also io-pinball, sys-misc, guid-powered) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| MRKBUF1 | CSM | eq PRAXIS +3 | module-private (prog-align; also sys-misc, exec-sched, io-nouns, prog-rendz) | MARK ROUTINE (R21) STORAGE. -- IS SUBSET OF R22 -- |
| MRKBUF2 | CSM | eq MRKBUF1 +7 | module-private (prog-rendz; also exec-sched, guid-powered, prog-align) | MARK ROUTINE (R21) STORAGE. -- IS SUBSET OF R22 -- |
| MRKRTMP | CSM | eq TEMPDAP | module-private (guid-powered) | R22 PAD LOADS (5D). |
| MU/A | LM | eq MUASTEER +2 | module-private (guid-powered; also prog-rendz) | ASTEER STORAGE. (22D) |
| MUASTEER | LM | eq RMAG +2 | module-private (guid-powered; also prog-rendz) | ASTEER STORAGE. (22D) |
| MUSCALE | LM | eq SCAXIS +2 | module-private (guid-powered) | ASTEER STORAGE. (22D) |
| MX | LM | eq RDRET +1 | module-private (prog-rendz) | LRS22.3 STORAGE. (CAN SHARE WITH P30'S AND OVERLAY |
| MY | LM | eq MX +6 | module-private (prog-rendz) | LRS22.3 STORAGE. (CAN SHARE WITH P30'S AND OVERLAY |
| MZ | LM | eq MY +6 | module-private (prog-rendz) | LRS22.3 STORAGE. (CAN SHARE WITH P30'S AND OVERLAY |
| N | CSM | eq CADDY +1 | module-private (dap-csm; also prog-rendz) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| N10 | CSM | eq COEFFADR +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| N49DISP | CSM | eq BVECTOR | module-private (io-nouns; also prog-rendz) | P20, P22, P23 DSP NOUN (5D) |
| N49FLAG | LM | eq RDOTMSAV | module-private (prog-rendz) | LRS22.3 STORAGE. (CAN SHARE WITH P30'S AND OVERLAY |
| NBDX | CSM+LM | 1 | module-private (exec-sched; also guid-powered, imu) | IMU COMPENSATION PARAMETERS (22D) |
| NBDY | CSM+LM | 1 | module-private (imu) | IMU COMPENSATION PARAMETERS (22D) |
| NBDZ | CSM+LM | 1 | module-private (imu) | IMU COMPENSATION PARAMETERS (22D) |
| NBRCYCLP | CSM | eq NBRCYCLS +1 | module-private (guid-powered) | OVERLAYS WITHIN TVC DAP |
| NBRCYCLS | CSM | eq N10 +15D | module-private (guid-powered) | OVERLAYS WITHIN TVC DAP |
| NCDU | CSM+LM | eq TMIS | module-private (imu) | FIRST-ORDER OVERLAYS IN KALCMANU |
| NCSMVEL | LM | eq TRANSTM +2 | module-private (prog-rendz) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| NDXCDUW | LM | eq QCDUWUSR +1 | module-private (guid-landing) | OVERLAYING KALCMANU STORAGE: (26D) |
| NDXCTR | CSM+LM | 1 | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| NEGTORKU | LM | eq DOWNTORK +3 | module-private (unreferenced in code; overlay alias or spare) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| NEGTORKV | LM | eq DOWNTORK +5 | module-private (unreferenced in code; overlay alias or spare) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| NEGUQ | LM | 3 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| NEGUR | LM | eq NEGUQ +2 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| NETTOTKP | LM | eq DOWNTORK +1 | module-private (unreferenced in code; overlay alias or spare) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| NEWAZ1 | CSM | eq LUNCHAZ1 +2 | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| NEWAZMTH | CSM | eq AINLA +121D | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| NEWJOB | CSM+LM | 1 | module-private (exec-sched; also prog-rendz, sys-misc, core-numeric, guid-entry...) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| NEWLOC | CSM+LM | eq ITEMP5 | module-private (exec-sched) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| NEWLOC+1 | CSM+LM | eq ITEMP6 | module-private (unreferenced in code; overlay alias or spare) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| NEWPOS | LM | eq NEWVEL +6 | module-private (prog-rendz) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| NEWPRIO | CSM+LM | eq ITEMP3 | module-private (exec-sched; also imu, io-pinball) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| NEWVEL | LM | eq UCSM +6 | module-private (prog-rendz) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| NEXTIME | CSM+LM | eq TMIS +3 | module-private (imu) | FIRST-ORDER OVERLAYS IN KALCMANU |
| NEXTP | LM | 3 | module-private (dap-lm; also exec-sched) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| NEXTU | LM | eq NEXTP +1 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| NEXTV | LM | eq NEXTP +2 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| NGUIDSUB | LM | eq ELVDUMMY | module-private (guid-landing) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| NIGNLOOP | LM | eq ZERDUMMY | module-private (guid-landing) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| NN | CSM+LM | eq LOOPCT +2 | module-private (prog-rendz; also io-nouns, sys-misc) | ADDITIONAL CSI - CDH STORAGE. (10D) |
| NN1 | CSM | eq NN | module-private (io-nouns; also prog-rendz) | P17,P34 (2D) |
| NN1A | CSM | eq DELTAT +2 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| NN2 | CSM | eq NN1A +2 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| NNADTEM | CSM+LM | 1 | module-private (io-nouns; also io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| NNTYPTEM | CSM+LM | 1 | module-private (io-nouns; also io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| NO.PJETS | LM | 3 | module-private (dap-lm) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| NO.UJETS | LM | eq NO.PJETS +1 | module-private (dap-lm) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| NO.VJETS | LM | eq NO.UJETS +1 | module-private (unreferenced in code; overlay alias or spare) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| NOBITS | CSM+LM | eq GENPL +6 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| NOMTIG | CSM | eq END-KALC | module-private (guid-powered) | SERVICER STORAGE. (13D) |
| NOMTPI | CSM+LM | eq KT | module-private (prog-rendz) | SOME P34 STORAGE. (OVERLAYS P35.1 STORAGE) (2D) |
| NORMEX | CSM+LM | 1 | module-private (prog-rendz) | P34-P35 STORAGE (1D) |
| NORMGAM | CSM+LM | eq IRETURN +1 | module-private (nav-aux) | INTEGRATION STORAGE. (95D) |
| NORMTEM1 | CSM+LM | eq DSPTEM1 | module-private (io-pinball; also prog-align) | PINBALL EXECUTIVE ACTION (44D) |
| NOUNADD | CSM+LM | eq TEM5 | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| NOUNCADR | CSM+LM | 1 | module-private (io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| NOUNREG | CSM+LM | 1 | module-private (exec-sched; also io-nouns, io-pinball, sys-misc) | PINBALL EXECUTIVE ACTION (44D) |
| NOUNTEM | CSM+LM | eq VBUF | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| NOUT | CSM+LM | 1 | module-private (exec-sched; also io-pinball, sys-misc) | PINBALL EXECUTIVE ACTION (44D) |
| NPJETS | CSM | eq NRJETS +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| NPTRAPS | LM | eq TRAPEDP +3 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| NQTRAPS | LM | eq TRAPEDP +4 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| NRJETS | CSM | eq YINDEX +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| NRTRAPS | LM | eq TRAPEDP +5 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| NSAMP | LM | eq MODEB | module-private (prog-rendz) | UNSWITCHED RADAR ERASABLE |
| NSRCHPNT | LM | eq OMEGDISP +2 | module-private (prog-rendz) | LRS24.1 STORAGE. (CAN SHARE WITH P30'S) (40D) |
| NUJET | CSM | eq OUTTAG +1 | module-private (guid-entry) | MISCELLANEOUS PERMANENT ERASABLE. |
| NUMBERT | LM | eq DAPTEMP5 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| NUVCSM | CSM+LM | 6 | module-private (prog-rendz) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| NUVLEM | CSM+LM | 6 | module-private (prog-rendz) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| NVBNKTEM | CSM+LM | 1 | module-private (io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| NVQTEM | CSM+LM | 1 | module-private (io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| NVSAVE | CSM+LM | 1 | module-private (exec-sched; also io-pinball) | UNSWITCHED FOR DISPLAY INTERFACE ROUTINES. (10D) |
| NVTEMP | CSM+LM | eq VBUF +1 | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| NVWORD | CSM+LM | 1 | module-private (io-pinball; also guid-powered) | UNSWITCHED FOR DISPLAY INTERFACE ROUTINES. (10D) |
| NVWORD1 | CSM+LM | 1 | module-private (guid-powered; also io-pinball) | MISCELLANEOUS UNSWITCHED. (16D) |
| NXT6ADR | LM | 1 | module-private (dap-lm; also exec-sched) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| NYJETS | CSM | eq NPJETS +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| OGA | CSM | eq OGAPAST +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| OGAD | CSM | 1 | module-private (dap-csm; also guid-powered) | R22 PAD LOADS (5D). |
| OGAERR | CSM | eq OGA | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| OGANOW | CSM | eq CARD +1 | module-private (dap-csm; also exec-sched) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| OGAPAST | CSM | eq OGANOW +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| OGARATE | CSM | eq OMEGAB | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| OGC | CSM+LM | eq STARAD +18D | module-private (sys-misc; also imu, io-nouns, prog-align, exec-sched...) | ALIGNMENT/SYSTEST/AUTO OPTICS COMMON STORAGE. (17D |
| OGCT | LM | eq QMAJ +1 | module-private (prog-align) | ALIGNMENT STORAGE. (23D) |
| OGF | LM | eq TMIS +12D | module-private (unreferenced in code; overlay alias or spare) | FIRST-ODER OVERLAYS IN KALCMANU (25D) |
| OLDATAGD | LM | eq DESRET | module-private (prog-rendz) | UNSWITCHED RADAR ERASABLE |
| OLDAZ1 | CSM | eq LUNCHAZ1 +4 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| OLDAZMTH | CSM | eq AINLA +123D | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| OLDBODY1 | CSM | eq EDRIVEX | module-private (unreferenced in code; overlay alias or spare) | STORAGE FOR VHHDOT AND ATTDSP |
| OLDBODY2 | CSM | eq EDRIVEY | module-private (unreferenced in code; overlay alias or spare) | STORAGE FOR VHHDOT AND ATTDSP |
| OLDBODY3 | CSM | eq EDRIVEZ | module-private (unreferenced in code; overlay alias or spare) | STORAGE FOR VHHDOT AND ATTDSP |
| OLDBT1 | CSM | eq 1/PIPADT | module-private (guid-powered) | IMU COMPENSATION UNSWITCHED ERASABLE. (1D) |
| OLDELP | CSM | eq PHIDOT +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| OLDELQ | CSM | eq OLDELP +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| OLDELR | CSM | eq OLDELQ +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| OLDPIPAX | LM | eq THISTPIP +2 | module-private (guid-landing) | P66 ERASABLES (R.O.D.) (14D) |
| OLDPIPAY | LM | eq OLDPIPAX +1 | module-private (guid-landing) | P66 ERASABLES (R.O.D.) (14D) |
| OLDPIPAZ | LM | eq OLDPIPAY +1 | module-private (guid-landing) | P66 ERASABLES (R.O.D.) (14D) |
| OLDPMIN | LM | 1 | module-private (dap-lm) | OTHER VARIABLES (5D) |
| OLDQRMIN | LM | 1 | module-private (dap-lm) | OTHER VARIABLES (5D) |
| OLDUYA | CSM | eq -VREL +6 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| OLDXFORP | LM | 3 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| OLDYFORP | LM | eq OLDXFORP +1 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| OLDZFORQ | LM | eq OLDXFORP +2 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| OMEGA | CSM+LM | eq GAMMA +2 | module-private (nav-aux) | MEASUREMENT INCORPORATION STORAGE (R22) STORAGE. ( |
| OMEGAB | CSM | eq TVCRCS +6 | module-private (unreferenced in code; overlay alias or spare) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| OMEGAC | CSM | eq TVCRCS | module-private (guid-powered) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| OMEGAD | LM | eq OMEGDISP | module-private (io-nouns; also prog-rendz) | LRS24.1 STORAGE. (CAN SHARE WITH P30'S) (40D) |
| OMEGAES | CSM | eq VESO +6 | module-private (nav-aux) | CONISEX (LUNAR AND SOLAR EPHEM) STORAGE. -- PAD LO |
| OMEGAM1 | CSM+LM | eq END-W | module-private (nav-aux) | MEASUREMENT INCORPORATION STORAGE. (66D) |
| OMEGAM2 | CSM+LM | eq OMEGAM1 +6 | module-private (nav-aux) | MEASUREMENT INCORPORATION STORAGE. (66D) |
| OMEGAM3 | CSM+LM | eq OMEGAM2 +6 | module-private (nav-aux) | MEASUREMENT INCORPORATION STORAGE. (66D) |
| OMEGAP | LM | 5 | module-private (dap-lm; also sys-misc) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| OMEGAPD | LM | 3 | module-private (dap-lm; also guid-landing, imu, sys-misc) | KALCMANU:DAP INTERFACE (9D) |
| OMEGAQ | LM | eq OMEGAP +1 | module-private (dap-lm; also guid-powered) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| OMEGAQD | LM | eq OMEGAPD +1 | module-private (dap-lm; also guid-landing, imu) | KALCMANU:DAP INTERFACE (9D) |
| OMEGAR | LM | eq OMEGAP +2 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| OMEGARD | LM | eq OMEGAPD +2 | module-private (dap-lm; also guid-landing, imu) | KALCMANU:DAP INTERFACE (9D) |
| OMEGAU | LM | 2 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| OMEGAV | LM | eq OMEGAU +1 | module-private (unreferenced in code; overlay alias or spare) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| OMEGAXB | CSM | eq OMEGAB | module-private (unreferenced in code; overlay alias or spare) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| OMEGAXC | CSM | eq OMEGAC | module-private (unreferenced in code; overlay alias or spare) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| OMEGAYB | CSM | eq OMEGAB +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| OMEGAYC | CSM | eq OMEGAC +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| OMEGAZB | CSM | eq OMEGAB +4 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| OMEGAZC | CSM | eq OMEGAC +4 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| OMEGDISP | LM | eq DATAGOOD +1 | module-private (prog-rendz) | LRS24.1 STORAGE. (CAN SHARE WITH P30'S) (40D) |
| OPTCADR | CSM+LM | eq MODECADR +1 | module-private (exec-sched; also prog-align, sys-misc) | MODE SWITCHING ERASABLE (14D) |
| OPTIND | CSM | 1 | module-private (imu; also prog-align, sys-misc, exec-sched, guid-powered) | T4RUPT ERASABLE (6D) |
| OPTION1 | CSM+LM | 1 | module-private (prog-rendz; also io-nouns, io-pinball, guid-entry, prog-align) | MISCELLANEOUS UNSWITCHED. (16D) |
| OPTION2 | CSM+LM | 1 | module-private (prog-rendz; also prog-align, guid-entry, io-nouns) | MISCELLANEOUS UNSWITCHED. (16D) |
| OPTION3 | LM | 1 | module-private (unreferenced in code; overlay alias or spare) | MISCELLANEOUS UNSWITCHED. (20D) |
| OPTIONX | CSM+LM | eq DSPTEMX | module-private (prog-rendz; also io-nouns, prog-align, sys-misc) | DISPLAY FOR EXTENDED VERBS (2D) |
| OPTIONY | CSM | eq UNP36 +6 | module-private (prog-rendz) | R36 SCRATCHPAD STORAGE (13D) |
| OPTMODES | CSM | 1 | module-private (sys-misc; also exec-sched, io-pinball, prog-align) | MODE SWITCHING ERASABLE (14D) |
| OPTNREG | CSM+LM | eq GENPL +23D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| ORIG | LM | eq RQVV +2 | module-private (unreferenced in code; overlay alias or spare) | R32(V83) STORAGE. -- SHARES WITH INTEGRATION STORA |
| ORIGEX | CSM+LM | eq RPQV +6 | module-private (nav-integ) | INTEGRATION STORAGE. (95D) |
| ORIGIN | LM | eq P30EXIT | module-private (unreferenced in code; overlay alias or spare) | P30/P31 STORAGE. (1D) AND ONE OVERLAY |
| OURPERMS | LM | eq FCOLD +1 | module-private (unreferenced in code; overlay alias or spare) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| OURTEMPS | LM | eq RN1 | module-private (unreferenced in code; overlay alias or spare) | THESE ERASABLES MAY BE SHARED WITH CARE |
| OUTOFPLN | LM | eq RANGEDSP +2 | module-private (guid-landing; also io-nouns) | SOME VARIABLES FOR SECOND DPS GUIDANCE. (34D) |
| OUTROUTE | LM | eq PIPAGE +1 | module-private (exec-sched; also guid-powered) | SERVICER -- MUST FOLLOW PIPTIME -- (19D) |
| OUTTAG | CSM | eq T2BITS +1 | module-private (guid-entry) | MISCELLANEOUS PERMANENT ERASABLE. |
| OVFIND | CSM+LM | 1 | module-private (core-numeric; also exec-sched, guid-entry, guid-landing) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| OVFLOWCK | LM | eq AINLA +98D | module-private (sys-misc) | SYSTEM TEST ERASABLES. CAN OVERLAY W MATRIX. (127D |
| P | CSM+LM | eq CSTH-RHO +2 | module-private (guid-powered; also nav-conic, nav-integ, prog-rendz, guid-entry) | CONICS ROUTINE STORAGE. (87D) |
| P(T1) | CSM | eq BETA1 +2 | module-private (unreferenced in code; overlay alias or spare) | RETURN-TO-EARTH STORAGE. (93D) |
| P/RPRE | CSM | eq 26D | module-private (guid-entry) | OVERLAYS WITHIN RETURN-TO-EARTH STORAGE. |
| P21 | CSM+LM | eq KV1 | module-private (unreferenced in code; overlay alias or spare) | SECOND-ORDER OVERLAYS IN KALCMANU |
| P21ALT | CSM+LM | eq P21BASEV +6 | module-private (io-nouns; also prog-rendz) | P21 STORAGE (19D) |
| P21BASER | CSM+LM | eq P21ORIG +1 | module-private (prog-rendz) | P21 STORAGE (19D) |
| P21BASEV | CSM+LM | eq P21BASER +6 | module-private (prog-rendz) | P21 STORAGE (19D) |
| P21GAM | CSM+LM | eq P21VEL +2 | module-private (io-nouns; also prog-rendz) | P21 STORAGE (19D) |
| P21ORIG | CSM+LM | eq TRUNX | module-private (prog-rendz) | P21 STORAGE (19D) |
| P21TIME | CSM+LM | eq DT/2 +2 | module-private (prog-rendz; also exec-sched, sys-misc) | P21, R61 STORAGE. (2D) |
| P21VEL | CSM+LM | eq P21ALT +2 | module-private (io-nouns; also prog-rendz) | P21 STORAGE (19D) |
| P30/31RT | CSM | eq TITER | module-private (guid-powered) | S34.1 STORAGE. (OVERLAYS S35.1 STORAGE) (1) |
| P30/RET | CSM | eq IRETURN1 | module-private (guid-powered) | MISCELLANEOUS UNSWITCHED. (16D) |
| P30EXIT | LM | eq EGRESS +1 | module-private (unreferenced in code; overlay alias or spare) | P30/P31 STORAGE. (1D) AND ONE OVERLAY |
| P37RANGE | CSM | eq R61CNTR +1 | module-private (guid-entry) | P37 PAD LOADS (1) |
| P40/RET | LM | 1 | module-private (guid-powered) | MISCELLANEOUS UNSWITCHED. (20D) |
| P40TMP | CSM | eq VGBODY +6 | module-private (exec-sched; also guid-powered, sys-misc) | P40 STORAGE. (8D) |
| P63FLAG | CSM | eq HEADSUP +1 | module-private (guid-entry) | MISCELLANEOUS PERMANENT ERASABLE. |
| PAC | CSM | eq SAC +2 | module-private (prog-align; also imu, io-nouns, sys-misc) | ALIGNMENT/SYSTEST/AUTO OPTICS COMMON STORAGE. (17D |
| PACTOFF | CSM | 1 | module-private (dap-csm; also exec-sched, sys-misc, guid-powered, io-nouns) | R22 PAD LOADS (5D). |
| PACTTMP | CSM | eq IOGARATE +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| PADLONG | CSM | eq ENTRYVN | module-private (guid-entry) | P11 STORAGE. (9D) |
| PAXERR | CSM | eq AK | module-private (guid-entry) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| PAXERR1 | CSM | eq TUSED +1 | module-private (guid-entry; also sys-misc) | ANGLE REGISTERS FOR ENTRY DAPS |
| PBIASX | CSM+LM | 1 | module-private (guid-landing) | IMU COMPENSATION PARAMETERS (22D) |
| PBIASY | CSM+LM | 1 | module-private (guid-landing) | IMU COMPENSATION PARAMETERS (22D) |
| PBIASZ | CSM+LM | 1 | module-private (guid-landing) | IMU COMPENSATION PARAMETERS (22D) |
| PBIT | LM | eq BIT10 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| PBODY | CSM+LM | 1 | module-private (nav-integ; also nav-aux, sys-misc) | INTEGRATION STORAGE. (95D) |
| PCDUYPST | CSM | eq TVCPHASE +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| PCDUZPST | CSM | eq PCDUYPST +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| PCMD | CSM | eq DAP3 +2 | module-private (dap-csm; also sys-misc) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| PCON | CSM | eq T1 +2 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| PCONS | LM | eq DZDOT +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| PDA | CSM+LM | eq 22D | module-private (prog-align) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| PDELOFF | CSM | eq DELYBAR +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| PERFDLAY | CSM+LM | eq AINLA +138D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| PERRB | CSM | eq STROKER +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| PERROR | LM | 1 | module-private (dap-lm; also exec-sched) | OTHER VARIABLES (5D) |
| PGNCSALT | CSM | eq TEPHEM1 +3 | module-private (guid-entry) | P11 STORAGE. (9D) |
| PGUIDE | LM | eq ABDELV +1 | module-private (guid-powered) | SERVICER -- MUST FOLLOW PIPTIME -- (19D) |
| PHASE1 | CSM+LM | 1 | module-private (exec-sched) | PHASE TABLE AND RESTART COUNTERS (12D) |
| PHASE2 | CSM+LM | 1 | module-private (guid-powered) | PHASE TABLE AND RESTART COUNTERS (12D) |
| PHASE3 | CSM+LM | 1 | module-private (guid-powered) | PHASE TABLE AND RESTART COUNTERS (12D) |
| PHASE4 | CSM+LM | 1 | module-private (guid-powered) | PHASE TABLE AND RESTART COUNTERS (12D) |
| PHASE5 | CSM+LM | 1 | module-private (guid-powered; also guid-entry) | PHASE TABLE AND RESTART COUNTERS (12D) |
| PHASE6 | CSM+LM | 1 | module-private (guid-powered) | PHASE TABLE AND RESTART COUNTERS (12D) |
| PHASETMP | CSM | eq TTMP1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| PHI2 | CSM | eq CFPA +2 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| PHI333 | CSM | eq INTTEMP | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| PHIDOT | CSM | eq BETADOT +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| PHIV | CSM+LM | eq BETAV +6 | module-private (nav-integ) | INTEGRATION STORAGE. (95D) |
| PHSBB1 | CSM+LM | 1 | module-private (unreferenced in code; overlay alias or spare) | PHASE-CHANGE LISTS PART II. (12D) |
| PHSBB2 | CSM+LM | 1 | module-private (unreferenced in code; overlay alias or spare) | PHASE-CHANGE LISTS PART II. (12D) |
| PHSBB3 | CSM+LM | 1 | module-private (unreferenced in code; overlay alias or spare) | PHASE-CHANGE LISTS PART II. (12D) |
| PHSBB4 | CSM+LM | 1 | module-private (unreferenced in code; overlay alias or spare) | PHASE-CHANGE LISTS PART II. (12D) |
| PHSBB5 | CSM+LM | 1 | module-private (unreferenced in code; overlay alias or spare) | PHASE-CHANGE LISTS PART II. (12D) |
| PHSBB6 | CSM+LM | 1 | module-private (unreferenced in code; overlay alias or spare) | PHASE-CHANGE LISTS PART II. (12D) |
| PHSNAME1 | CSM+LM | 1 | module-private (exec-sched) | PHASE-CHANGE LISTS PART II. (12D) |
| PHSNAME2 | CSM+LM | 1 | module-private (guid-landing) | PHASE-CHANGE LISTS PART II. (12D) |
| PHSNAME3 | CSM+LM | 1 | module-private (guid-landing) | PHASE-CHANGE LISTS PART II. (12D) |
| PHSNAME4 | CSM+LM | 1 | module-private (unreferenced in code; overlay alias or spare) | PHASE-CHANGE LISTS PART II. (12D) |
| PHSNAME5 | CSM+LM | 1 | module-private (guid-entry; also guid-powered) | PHASE-CHANGE LISTS PART II. (12D) |
| PHSNAME6 | CSM+LM | 1 | module-private (unreferenced in code; overlay alias or spare) | PHASE-CHANGE LISTS PART II. (12D) |
| PHSPRDT1 | CSM+LM | 1 | module-private (exec-sched) | TBASE'S AND PHSPRDT'S. (12D) |
| PHSPRDT2 | CSM+LM | 1 | module-private (exec-sched; also prog-rendz, guid-landing) | TBASE'S AND PHSPRDT'S. (12D) |
| PHSPRDT3 | CSM+LM | 1 | module-private (exec-sched) | TBASE'S AND PHSPRDT'S. (12D) |
| PHSPRDT4 | CSM+LM | 1 | module-private (exec-sched) | TBASE'S AND PHSPRDT'S. (12D) |
| PHSPRDT5 | CSM+LM | 1 | module-private (guid-entry) | TBASE'S AND PHSPRDT'S. (12D) |
| PHSPRDT6 | CSM+LM | 1 | module-private (guid-powered) | TBASE'S AND PHSPRDT'S. (12D) |
| PIF | LM | eq FWEIGHT +2 | module-private (guid-landing) | THESE ERASABLES MUST NOT OVERLAY GOBLTIME OR SERVI |
| PIFPSET | LM | eq XSMD | module-private (guid-landing) | THESE ERASABLES MUST NOT OVERLAY GOBLTIME OR SERVI |
| PINDEX | CSM | eq RINDEX +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| PIPABIAS | CSM+LM | eq PBIASX | module-private (imu) | IMU COMPENSATION PARAMETERS (22D) |
| PIPAGE | CSM+LM | eq PIPCTR +1 | module-private (guid-powered) | EXCEED THE ERASABLE STORAGE REQUIRED BY RENDEZVOUS |
| PIPASCF | CSM+LM | eq PIPASCFX | module-private (imu) | IMU COMPENSATION PARAMETERS (22D) |
| PIPASCFX | CSM+LM | 1 | module-private (unreferenced in code; overlay alias or spare) | IMU COMPENSATION PARAMETERS (22D) |
| PIPASCFY | CSM+LM | 1 | module-private (unreferenced in code; overlay alias or spare) | IMU COMPENSATION PARAMETERS (22D) |
| PIPASCFZ | CSM+LM | 1 | module-private (unreferenced in code; overlay alias or spare) | IMU COMPENSATION PARAMETERS (22D) |
| PIPATMPX | LM | 1 | module-private (guid-landing; also guid-powered) | MISCELLANEOUS UNSWITCHED. (20D) |
| PIPATMPY | LM | 1 | module-private (guid-landing; also guid-powered) | MISCELLANEOUS UNSWITCHED. (20D) |
| PIPATMPZ | LM | 1 | module-private (guid-landing; also guid-powered) | MISCELLANEOUS UNSWITCHED. (20D) |
| PIPCTR | CSM | eq TEMZ +1 | module-private (guid-powered) | EXCEED THE ERASABLE STORAGE REQUIRED BY RENDEZVOUS |
| PIPINDEX | CSM+LM | 1 | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| PIPTEM | LM | eq LRZCDU +1 | module-private (guid-powered) | CAN OVERLAY GSAV WITH CARE, USED DURING POWERED DE |
| PIPTIME | CSM+LM | 2 | `agc_erase_piptime()` / `agc_erase_set_piptime()` | PERM STATE VECTORS FOR BOOST AND DOWNLINK -- WHOLE |
| PIPTIME1 | CSM+LM | eq VN1 +6 | module-private (guid-powered; also guid-landing, nav-integ, sys-misc, imu) | EXCEED THE ERASABLE STORAGE REQUIRED BY RENDEZVOUS |
| PITCH | LM | eq YAW +2 | module-private (guid-landing; also io-nouns) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| PITCHANG | LM | eq ALPHASB | module-private (prog-align) | S-BAND ANTENNA GIMBAL ANGLES. DISPLAYED BY R05 (V6 |
| PITTIME | LM | 1 | module-private (guid-powered; also io-nouns) | DAP PAD-LOADED DATA. (10D) |
| PJETCTR | LM | 3 | module-private (dap-lm) | DOCKED JET INHIBITION COUNTERS (3D) |
| PLANVEC | CSM+LM | eq SAVQR52 +2 | module-private (prog-align) | AUTO-OPTICS STORAGE -R52- |
| PLAST | LM | 1 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| PMANNDX | CSM | eq RMANNDX +1 | module-private (dap-csm; also guid-entry) | OVERLAYS WITHIN TVC DAP |
| POINTEX | CSM | eq TIME2SAV | module-private (prog-rendz) | P20, CONICS (SHARING WITH TIME 2 SAV AND SCAL SAV  |
| POINTVSM | CSM+LM | eq SCAXIS +6 | module-private (guid-powered; also imu, prog-align, prog-rendz, guid-landing) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| POLISH | CSM+LM | 1 | module-private (core-numeric) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| POLYCNT | CSM+LM | eq MATINC | module-private (core-numeric) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| POLYLOC | CSM | eq POLYNUM +10D | module-private (guid-entry) | SATURN BOOST STORAGE. SAVE TILL RCS DAP OPERATION. |
| POLYNUM | CSM | eq BCDU | module-private (guid-entry) | SATURN BOOST STORAGE. SAVE TILL RCS DAP OPERATION. |
| POLYRET | CSM+LM | eq TEM1 | module-private (core-numeric) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| POLYSTOP | CSM | eq RPSTART +1 | module-private (guid-entry) | MORE P11 STORAGE --PAD LOADED-- (2D) |
| POLYTEMP | LM | eq DAPTEMP3 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| POSEXIT | CSM | eq GAMDOT +1 | module-private (guid-entry) | MISCELLANEOUS PERMANENT ERASABLE. |
| POSITON | CSM+LM | 1 | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| POSNV | CSM+LM | eq AINLA +52D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| POSTCDH | CSM+LM | eq DVPREV +2 | module-private (prog-rendz) | THE FOLLOWING OVERLAY MEAS. INCORP. ARE AND IN USE |
| POSTCSI | CSM+LM | eq VG | module-private (prog-rendz) | THE FOLLOWING OVERLAY MEAS. INCORP. ARE AND IN USE |
| POSTORKP | LM | eq DOWNTORK | module-private (sys-misc) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| POSTORKU | LM | eq DOWNTORK +2 | module-private (sys-misc) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| POSTORKV | LM | eq DOWNTORK +4 | module-private (unreferenced in code; overlay alias or spare) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| POSTTPI | CSM+LM | eq DELVTPF +2 | module-private (prog-rendz; also io-nouns) | RENDEZVOUS -P34-35 (26D) |
| PRATE | LM | eq YCONS +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| PRAXIS | CSM | eq SAVEVEL +6 | module-private (io-nouns; also prog-rendz) | ATTITUDE MANEUVER -- CALLED BY P20,R61,R63,CRS61.1 |
| PREDANG | CSM | eq GAMMAEI | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| PREL | CSM | eq CMDAPMOD +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| PREMTRX1 | CSM | eq AINLA +118D | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| PREMTRXC | CSM | eq AINLA +118D | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| PRIORITY | CSM+LM | 1 | module-private (exec-sched; also imu, io-pinball, guid-powered, sys-misc) | DYNAMICALLY ALLOCATED CORE SETS FOR JOBS (84D) |
| PRIOTIME | CSM+LM | 1 | module-private (io-pinball) | MISCELLANEOUS UNSWITCHED. (16D) |
| PROJ | LM | eq 18D | module-private (unreferenced in code; overlay alias or spare) | ERASABLES CONVENIENTLY DEFINABLE IN THE WORK AREA |
| PSEUDO55 | LM | eq PIF +2 | module-private (guid-landing; also sys-misc) | THESE ERASABLES MUST NOT OVERLAY GOBLTIME OR SERVI |
| PSI333 | CSM | eq PHI333 +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| PSIV | CSM+LM | eq PHIV +6 | module-private (nav-integ) | INTEGRATION STORAGE. (95D) |
| PTIGINC | CSM+LM | 2 | module-private (prog-rendz) | P35 CONSTANTS. -- PAD LOADED -- (4D) |
| PTMP1 | CSM | eq OMEGAC +12D | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| PTMP2 | CSM | eq PTMP1 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| PTMP3 | CSM | eq PTMP2 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| PTMP4 | CSM | eq PTMP3 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| PTMP5 | CSM | eq PTMP4 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| PTMP6 | CSM | eq PTMP5 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| PUSHLOC | CSM+LM | 1 | module-private (core-numeric; also exec-sched, guid-landing, guid-powered, prog-align...) | DYNAMICALLY ALLOCATED CORE SETS FOR JOBS (84D) |
| PVALVEST | LM | 1 | module-private (dap-lm; also exec-sched) | RCS FAILURE MONITOR STORAGE (1) |
| PWORD1 | CSM | eq RWORD2 +1 | module-private (dap-csm) | R22 PAD LOADS (5D). |
| PWORD2 | CSM | eq PWORD1 +1 | module-private (dap-csm) | R22 PAD LOADS (5D). |
| PWRCNT | LM | eq MATINC | module-private (guid-landing) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| PWRPTR | LM | eq POLISH | module-private (guid-landing) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| Q2 | CSM | eq D0 +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| Q611 | CSM | eq RM +1 | module-private (prog-rendz) | CRS61.1 STORAGE. --A SUBSET OF P20-- (14D) |
| Q6111 | CSM | eq Q611 +1 | module-private (prog-rendz) | CRS61.1 STORAGE. --A SUBSET OF P20-- (14D) |
| Q7 | CSM | eq THETADZ | module-private (guid-entry; also io-nouns) | ENTRY RESTART PROTECTION STORAGE. --KEEP TEMPS IN  |
| QACCDOT | LM | eq ACCDOTQ +1 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| QAXERR | CSM | eq THETADX | module-private (guid-entry) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| QAXIS | LM | eq UHYP | module-private (guid-landing) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| QCDUWUSR | LM | eq ECDUWUSR +1 | module-private (guid-landing) | OVERLAYING KALCMANU STORAGE: (26D) |
| QDIFF | LM | eq QERROR | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| QERROR | LM | eq DYERROR | module-private (dap-lm) | OTHER VARIABLES (5D) |
| QGIMTIMR | LM | 3 | module-private (dap-lm) | VARIABLES FOR GTS-QRAXIS CONTROL EXCHANGE. (4) |
| QLAST | LM | 1 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| QMAJ | CSM+LM | eq COMPNUMB | module-private (prog-align; also guid-landing) | ALIGNMENT STORAGE. (5D) |
| QMIN | CSM+LM | eq PAC +2 | module-private (prog-align; also prog-rendz) | ALIGNMENT/SYSTEST/AUTO OPTICS COMMON STORAGE. (17D |
| QPLAC | CSM | 1 | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| QPLACE | CSM+LM | 1 | module-private (sys-misc; also exec-sched, imu) | SYSTEM TEST STORAGE. (174) |
| QPLACES | CSM+LM | 1 | module-private (sys-misc; also guid-entry, imu) | SYSTEM TEST STORAGE. (174) |
| QRATEDIF | LM | eq EDOTQ | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| QRBIT | LM | eq BIT11 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| QRCNTR | LM | eq GTSTEMPS +9D | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| QREL | CSM | eq PREL +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| QSAVED | CSM+LM | eq VERBNOUN +1 | module-private (prog-rendz) | S34/35.5,P34-P35 STORAGE. (6D) |
| QTEMP | CSM+LM | eq TPASS4 +2 | module-private (guid-powered; also prog-rendz) | P30-P40 COMMON STORAGE. (3D) |
| QTEMP1 | CSM+LM | eq F +2 | module-private (guid-powered) | S40.1 STORAGE. (23D) |
| R | LM | eq TGO +2 | module-private (guid-landing; also guid-powered, prog-rendz, exec-sched) | SERVICER STORAGE. (69D) |
| R(CSM) | LM | eq R-OTHER | module-private (unreferenced in code; overlay alias or spare) | SERVICER FOR LUNAR ASCENT AND DESCENT (14D) |
| R(T1) | CSM | eq R(T1)/ +6 | module-private (unreferenced in code; overlay alias or spare) | RETURN-TO-EARTH STORAGE. (93D) |
| R(T1)/ | CSM | eq RCON +2 | module-private (unreferenced in code; overlay alias or spare) | RETURN-TO-EARTH STORAGE. (93D) |
| R(T2)/ | CSM | eq MAMAX2 +2 | module-private (unreferenced in code; overlay alias or spare) | RETURN-TO-EARTH STORAGE. (93D) |
| R-OTHER | CSM+LM | 6 | `agc_sv_get(AGC_SV_OTHER)` (downlink copy) | STATE VECTORS FOR DOWNLINK (12D) |
| R/APRE | CSM | eq 28D | module-private (guid-entry) | OVERLAYS WITHIN RETURN-TO-EARTH STORAGE. |
| R1A | CSM+LM | eq P +2 | module-private (prog-rendz; also nav-conic, guid-entry) | CONICS ROUTINE STORAGE. (87D) |
| R1C | LM | eq RTMAG +2 | module-private (guid-powered) | ASTEER STORAGE. (22D) |
| R1S | LM | eq MASS1 +2 | module-private (guid-powered) | SERVICER STORAGE. (69D) |
| R1SAVE | CSM+LM | 1 | module-private (exec-sched; also io-pinball) | UNSWITCHED FOR DISPLAY INTERFACE ROUTINES. (5D) |
| R1VEC | CSM+LM | eq RCNORM +2 | module-private (nav-conic; also prog-rendz) | CONICS ROUTINE STORAGE. (87D) |
| R2 | CSM+LM | eq MAGVEC2 | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| R22DISP | LM | eq TIME2SAV | module-private (io-nouns; also prog-rendz) | INCORP STORAGE: R22 (N29) (SHARES WITH FOLLOWING S |
| R2VEC | CSM+LM | eq R1VEC +6 | module-private (nav-conic; also prog-rendz) | CONICS ROUTINE STORAGE. (87D) |
| R53EXIT | CSM | eq VEL/C +6 | module-private (prog-align) | ALIGNMENT STORAGE. (25D) |
| R60VSAVE | LM | eq OUTOFPLN +2 | module-private (guid-landing) | SOME VARIABLES FOR SECOND DPS GUIDANCE. (34D) |
| R61CNTR | CSM | eq SAVEDCDU +3 | module-private (prog-rendz) | R61 STORAGE. (1D) |
| R65CNTR | LM | eq RRBORSIT +5 | module-private (prog-rendz) | R61LEM -- PREFERRED TRACKING ATTITUDE ROUTINE **IN |
| RACCDOT | LM | eq ACCDOTQ +3 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| RACFAIL | CSM | eq J/M2 +1 | module-private (dap-csm; also guid-powered) | OVERLAYS WITHIN TVC DAP |
| RACT1 | CSM+LM | eq VACT2 +6 | module-private (prog-rendz) | P32,P33 STORAGE OVERLAYING 9X9 W-MATRIX LOCATIONS  |
| RACT2 | CSM+LM | eq DELVEET1 +6 | module-private (prog-rendz) | P32,P33 STORAGE OVERLAYING 9X9 W-MATRIX LOCATIONS  |
| RACT3 | CSM+LM | eq GRP2SVQ +1 | module-private (prog-rendz; also guid-powered) | P30'S-P17 COMMON STORAGE. (24D) |
| RAD | CSM | eq AM +2 | module-private (unreferenced in code; overlay alias or spare) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| RADCADR | CSM+LM | eq MODECADR +2 | module-private (exec-sched; also prog-rendz) | MODE SWITCHING ERASABLE (14D) |
| RADSKAL | LM | 2 | module-private (guid-powered) | ERASABLES FOR THE R2 LUNAR POTENTIAL MODEL (2D) |
| RANGE | CSM+LM | eq EGRESS +1 | module-private (prog-rendz; also io-nouns, sys-misc) | VERB 83 STORAGE. (20D) |
| RANGEDSP | LM | eq CG +18D | module-private (guid-landing; also io-nouns) | SOME VARIABLES FOR SECOND DPS GUIDANCE. (34D) |
| RANGEVAR | LM | 2 | module-private (prog-rendz) | PADLOADED ERASABLES FOR P20/P22 (6D) |
| RANGRDOT | LM | eq RM +2 | module-private (prog-rendz; also sys-misc) | LRS22.1 (SAME AS PREVIOUS SECTION) ALSO DOWNLINK F |
| RAPFG | LM | eq TCGIBRAK +1 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| RAPREC | CSM+LM | eq VINIT | module-private (prog-rendz) | P34 AND P33 STORAGE. (OVERLAYS INITVEL/MIDGIM) (24 |
| RATEINDX | CSM+LM | 1 | module-private (imu; also exec-sched, dap-csm, guid-powered, sys-misc) | MISCELLANEOUS UNSWITCHED. (16D) |
| RATEVAR | LM | 2 | module-private (prog-rendz) | PADLOADED ERASABLES FOR P20/P22 (6D) |
| RAXERR | CSM | eq QAXERR +1 | module-private (guid-entry) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| RBDFAIL | CSM | eq RACFAIL +1 | module-private (dap-csm; also guid-powered) | OVERLAYS WITHIN TVC DAP |
| RBRFG | LM | eq TLAND +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| RCL | CSM | eq LEMPOS +6 | module-private (unreferenced in code; overlay alias or spare) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| RCLP | CSM | eq VARIANCE +3 | module-private (prog-rendz) | MEASUREMENT INCORPORATION STORAGE (R22) STORAGE. ( |
| RCNORM | CSM+LM | eq URRECT +6 | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| RCO | LM | eq END-ALIG | module-private (guid-landing) | ASCENT GUIDANCE ERASABLES. (21D) |
| RCON | CSM | eq RTEGAM2D +2 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| RCSFLAGS | CSM+LM | 1 | module-private (dap-csm; also dap-lm, guid-entry, exec-sched, guid-powered...) | R22 PAD LOADS (5D). |
| RCV | CSM+LM | eq TNUV +6 | module-private (prog-rendz; also nav-integ, nav-aux, nav-conic, prog-align...) | STATE VECTORS FOR ORBIT INTEGRATION. (44D) |
| RCVCSM | CSM+LM | 6 | module-private (prog-rendz) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| RCVLEM | CSM+LM | 6 | module-private (prog-rendz) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| RD | CSM | eq R(T2)/ +6 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| RDES | LM | eq RRRET +1 | module-private (prog-rendz) | RENDEZVOUS RADAR TASK STORAGE (3D) |
| RDESIRED | CSM+LM | eq SGNRDOT +1 | module-private (nav-conic; also guid-entry, prog-rendz) | CONICS ROUTINE STORAGE. (87D) |
| RDIFF | LM | eq RERROR | module-private (unreferenced in code; overlay alias or spare) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| RDOT | CSM+LM | eq LATANG +2 | module-private (guid-landing; also guid-entry, io-nouns) | REENTRY ERASABLES. (206D) |
| RDOTD | LM | eq TBUP +2 | module-private (guid-landing; also io-nouns) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| RDOTM | LM | eq RDOTMSAV +2 | module-private (prog-rendz) | LRS22.1 (SAME AS PREVIOUS SECTION) ALSO DOWNLINK F |
| RDOTMSAV | LM | eq RRBORSIT +6 | module-private (prog-rendz) | LRS22.1 STORAGE. (MUST NOT SHARE WITH P30'S) (13D) |
| RDOTREF | CSM | eq THETAD | module-private (guid-entry) | STANDBY VERB ERASABLES. REDOCTR BEFORE THETAD (DWN |
| RDOTV | CSM+LM | eq VACT4 +6 | module-private (prog-rendz) | THE FOLLOWING OVERLAY MEAS. INCORP. ARE AND IN USE |
| RDRET | LM | eq LGRET | module-private (prog-rendz) | LRS22.3 STORAGE. (CAN SHARE WITH P30'S AND OVERLAY |
| RDSP | CSM+LM | eq GENPL | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| REDOCTR | CSM+LM | 1 | module-private (exec-sched; also sys-misc) | STANDBY VERB ERASABLES. REDOCTR BEFORE THETAD (DWN |
| REFSMMAT | CSM+LM | 18 | `agc_erase_refsmmat()` / `agc_erase_set_refsmmat()` | REFSMMAT. (18D) |
| RELDELV | CSM | eq MAGVTPI +2 | module-private (prog-rendz) | TPI SEARCH (26D) |
| RELRET | CSM+LM | eq TEM4 | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| REPFRAC | CSM | eq VARK +1 | module-private (dap-csm; also guid-powered) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| REPOSCNT | LM | eq TENDBRAK | module-private (prog-rendz) | P22 STORAGE -- OVERLAYS LANDING PADLOADS -- (5D) |
| REPOSTM | LM | eq REPOSCNT +1 | module-private (prog-rendz) | P22 STORAGE -- OVERLAYS LANDING PADLOADS -- (5D) |
| REQRET | CSM+LM | 1 | module-private (exec-sched; also io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| RERROR | LM | eq DZERROR | module-private (dap-lm) | OTHER VARIABLES (5D) |
| RESO | CSM | eq VECOEM +60D | module-private (nav-aux) | CONISEX (LUNAR AND SOLAR EPHEM) STORAGE. -- PAD LO |
| RESTARPT | CSM | eq AINLA +91D | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| RESTREG | CSM+LM | 1 | module-private (exec-sched; also io-pinball) | UNSWITCHED FOR DISPLAY INTERFACE ROUTINES. (10D) |
| RESULTCT | CSM+LM | eq GENPL +67D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| RETJADR | LM | 1 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| RETROOT | LM | eq BUF +2 | module-private (guid-landing) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| REVS | CSM | eq BUNKER +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| RFAILCNT | LM | 1 | module-private (prog-rendz; also sys-misc) | SOME R04(V63)-R77 RADAR TEST STORAGE. (6D) |
| RGEXIT | CSM+LM | eq SUBEXIT | module-private (guid-powered) | S34/35.5,P34-P35 STORAGE. (6D) |
| RGIMTIMR | LM | eq QGIMTIMR +2 | module-private (dap-lm) | VARIABLES FOR GTS-QRAXIS CONTROL EXCHANGE. (4) |
| RGU | LM | eq R60VSAVE +6 | module-private (guid-landing; also sys-misc) | SOME VARIABLES FOR SECOND DPS GUIDANCE. (34D) |
| RHO | CSM | eq T5TIME +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| RHO1 | CSM | eq RHO +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| RHO2 | CSM | eq RHO1 +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| RHOSB | CSM | eq RANGE | module-private (io-nouns; also prog-align, sys-misc) | S-BAND ANTENNA GIMBAL ANGLES. DISPLAYED BY R05 (V6 |
| RIGNX | LM | eq VIGN +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| RIGNZ | LM | eq RIGNX +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| RINDEX | CSM | eq ZNDX +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| RINIT | CSM+LM | eq VPASS3 +6 | module-private (prog-rendz; also guid-powered) | INITVEL/MIDGIM STORAGE. (34D) |
| RLAST | LM | 1 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| RLM | LM | eq BETASB +2 | module-private (prog-align) | S-BAND ANTENNA GIMBAL ANGLES. DISPLAYED BY R05 (V6 |
| RLMSRCH | LM | eq INCORPEX +1 | module-private (prog-rendz) | LRS24.1 STORAGE. (CAN SHARE WITH P30'S) (40D) |
| RLS | CSM+LM | 6 | `agc_erase_rls()` / `agc_erase_set_rls()` | LUNAR LANDING SIGHT DATA. -- PAD LOADED -- (6D) |
| RM | CSM+LM | eq S22RTNEX | module-private (prog-rendz; also sys-misc) | DOWNLINK ERASABLES FOR P22, P20 MARK DATA. (8D) |
| RMAG | CSM+LM | 2 | module-private (guid-powered) | AVERAGEG INTEGRATOR STORAGE. (8D) |
| RMANNDX | CSM | eq ADB +1 | module-private (dap-csm; also guid-entry) | OVERLAYS WITHIN TVC DAP |
| RMAX | CSM+LM | 1 | module-private (prog-rendz) | P20 STORAGE. -- PAD LOADED -- (4D) |
| RN | CSM+LM | 6 | `agc_erase_rn()` / `agc_erase_set_rn()` | PERM STATE VECTORS FOR BOOST AND DOWNLINK -- WHOLE |
| RN1 | CSM+LM | eq PIPAGE +1 | module-private (guid-powered; also nav-integ, guid-entry, sys-misc) | EXCEED THE ERASABLE STORAGE REQUIRED BY RENDEZVOUS |
| RODCOUNT | LM | eq RUNIT +3 | module-private (guid-landing) | P66 ERASABLES (R.O.D.) (1D) |
| RODSCAL1 | LM | eq RM | module-private (guid-landing) | P66 ERASABLES (R.O.D.) (14D) |
| RODSCALE | LM | eq LRWVFF +1 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| ROLL/180 | CSM | eq AMG +1 | module-private (guid-entry; also guid-powered) | ANGLE REGISTERS FOR ENTRY DAPS |
| ROLL/PIP | CSM | eq AMG/PIP +1 | module-private (guid-entry; also guid-powered) | ANGLE REGISTERS FOR ENTRY DAPS |
| ROLLC | CSM | eq ROLLTM +1 | module-private (guid-entry; also io-nouns) | ANGLE REGISTERS FOR ENTRY DAPS |
| ROLLFIRE | CSM | eq YTMP6 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| ROLLHOLD | CSM | eq ROLLC +2 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| ROLLTIME | LM | 1 | module-private (guid-powered; also io-nouns, sys-misc) | DAP PAD-LOADED DATA. (10D) |
| ROLLTM | CSM | eq PAXERR1 +1 | module-private (guid-entry; also sys-misc) | ANGLE REGISTERS FOR ENTRY DAPS |
| ROLLWORD | CSM | eq ROLLFIRE +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| RONE | CSM+LM | eq RTHETA +2 | module-private (prog-rendz; also nav-aux, guid-entry, guid-powered, prog-align...) | VERB 83 STORAGE. (20D) |
| ROOTPS | LM | eq VBUF +4 | module-private (guid-landing) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| ROOTPS+1 | LM | eq VBUF +5 | module-private (unreferenced in code; overlay alias or spare) | THE FOLLOWING 10 REGISTERS ARE USED FOR TEMPORARY  |
| ROOTRET | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | SINGLE PRECISION SUBROUTINE TEMPORARIES (3D) |
| ROTEMP1 | LM | eq DAPTEMP1 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| ROTEMP2 | LM | eq DAPTEMP2 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| ROTINDEX | LM | eq DAPTEMP6 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| ROTSENSE | LM | eq DAPTEMP2 | module-private (dap-lm) | THE FOLLOWING LM DAP ERASABLES ARE ZEROED IN THE S |
| RPADTEM | CSM+LM | eq HPERMIN +2 | module-private (prog-rendz) | EXTENDED VERB 82 STORAGE. (6D) |
| RPASS1 | CSM+LM | eq CENTANG +2 | module-private (prog-rendz) | ADDITIONAL CSI - CDH STORAGE. (10D) |
| RPASS2 | CSM+LM | eq VPASS2 +6 | module-private (prog-rendz) | P32 --- P33 (26D) |
| RPASS3 | CSM+LM | eq VACT3 +6 | module-private (prog-rendz) | P30'S-P17 COMMON STORAGE. (24D) |
| RPASS36 | CSM+LM | eq RONE | module-private (prog-rendz; also sys-misc) | R36 SCRATCHPAD STORAGE (13D) |
| RPASS4 | CSM | eq RTIG | module-private (prog-rendz) | P35-P40 INTERFACE STORAGE. (OVERLAYS P30-P40 I/F S |
| RPCRTIME | LM | 1 | module-private (guid-powered) | LANDING RADAR. -- PAD LOADED -- (2D) |
| RPCRTQSW | LM | 1 | module-private (guid-powered) | LANDING RADAR. -- PAD LOADED -- (2D) |
| RPPREC | CSM+LM | eq VTPRIME | module-private (prog-rendz) | P34 AND P33 STORAGE. (OVERLAYS INITVEL/MIDGIM) (24 |
| RPQV | CSM+LM | eq VECTAB +36D | module-private (nav-integ) | INTEGRATION STORAGE. (95D) |
| RPRE | CSM | eq 24D | module-private (guid-entry) | OVERLAYS WITHIN RETURN-TO-EARTH STORAGE. |
| RPRE' | CSM | eq DRCON +2 | module-private (unreferenced in code; overlay alias or spare) | RETURN-TO-EARTH STORAGE. (93D) |
| RPSTART | CSM | eq SATRLRT +2 | module-private (guid-entry) | MORE P11 STORAGE --PAD LOADED-- (2D) |
| RPSV | CSM+LM | eq RQVV +6 | module-private (nav-integ) | INTEGRATION STORAGE. (95D) |
| RPVAR | CSM | 2 | module-private (prog-rendz) | P22 STORAGE. -- PAD LOADED -- (5D) |
| RQVV | CSM+LM | eq ORIGEX +1 | module-private (nav-integ) | INTEGRATION STORAGE. (95D) |
| RR-AZ | LM | eq PITCHANG | module-private (io-nouns; also sys-misc) | S-BAND ANTENNA GIMBAL ANGLES. DISPLAYED BY R05 (V6 |
| RR-ELEV | LM | eq RR-AZ +2 | module-private (io-nouns; also sys-misc) | S-BAND ANTENNA GIMBAL ANGLES. DISPLAYED BY R05 (V6 |
| RRATE | CSM+LM | eq RANGE +2 | module-private (prog-rendz; also io-nouns) | VERB 83 STORAGE. (20D) |
| RRATEDIF | LM | eq EDOTR | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| RRBORSIT | LM | eq LRS22.1X +1 | module-private (prog-rendz) | LRS22.1 STORAGE. (MUST NOT SHARE WITH P30'S) (13D) |
| RRECT | CSM+LM | eq UPSVFLAG +1 | module-private (nav-integ; also prog-rendz, nav-conic) | STATE VECTORS FOR ORBIT INTEGRATION. (44D) |
| RRECT1 | CSM | eq 18D | module-private (unreferenced in code; overlay alias or spare) | STATE VECTORS FOR ORBIT INTEGRATION. (44D) |
| RRECTCSM | CSM+LM | 6 | `agc_erase_rrectcsm()` / `agc_erase_set_rrectcsm()` | PERMANENT STATE VECTORS AND TIMES. (101D) |
| RRECTHIS | CSM+LM | eq RRECTCSM | alias: rrectcsm (this = CSM build) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| RRECTLEM | CSM+LM | 6 | `agc_erase_rrectlem()` / `agc_erase_set_rrectlem()` | PERMANENT STATE VECTORS AND TIMES. (101D) |
| RRECTOTH | CSM+LM | eq RRECTLEM | alias: rrectlem | PERMANENT STATE VECTORS AND TIMES. (101D) |
| RREL | CSM | eq QREL +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| RRINDEX | LM | eq RDES +1 | module-private (prog-rendz) | RENDEZVOUS RADAR TASK STORAGE (3D) |
| RRRET | LM | 3 | module-private (prog-rendz) | RENDEZVOUS RADAR TASK STORAGE (3D) |
| RRSHAFT | LM | eq RRTRUN +2 | module-private (prog-rendz) | LRS22.1 STORAGE. (MUST NOT SHARE WITH P30'S) (13D) |
| RRTARGET | LM | eq SAMPLSUM | module-private (prog-rendz) | UNSWITCHED RADAR ERASABLE |
| RRTRUN | LM | eq SINTHETA +2 | module-private (prog-rendz) | LRS22.1 STORAGE. (MUST NOT SHARE WITH P30'S) (13D) |
| RSAMPDT | LM | 1 | module-private (prog-rendz; also sys-misc) | SOME R04(V63)-R77 RADAR TEST STORAGE. (6D) |
| RSBBQ | CSM+LM | 2 | module-private (exec-sched; also sys-misc) | RESTART STORAGE. (2D) |
| RSP-RREC | CSM | eq AOPTIME | module-private (guid-powered; also io-nouns) | MORE VERB 82 NOT SHARING WITH VERB 83 (9D) |
| RSTACK | LM | eq RANGE | module-private (io-nouns; also prog-rendz, sys-misc) | R04 (V62) RADAR TEST STORAGE. |
| RSTKLOC | LM | eq RTSTLOC | module-private (unreferenced in code; overlay alias or spare) | SOME R04(V63)-R77 RADAR TEST STORAGE. (6D) |
| RSUBC | LM | eq RRTARGET | module-private (prog-rendz) | UNSWITCHED RADAR ERASABLE |
| RSUBL | LM | eq END-ALIG | module-private (prog-rendz) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| RT | CSM | eq RTNORM +6 | module-private (guid-entry; also prog-align) | REENTRY ERASABLES. (206D) |
| RTARG | CSM+LM | eq TIG +2 | module-private (prog-rendz; also sys-misc, guid-entry, guid-powered) | EXTERNAL DELTA-V UPDATE. (21D) |
| RTARG1 | CSM+LM | eq VINIT +6 | module-private (prog-rendz) | INITVEL/MIDGIM STORAGE. (34D) |
| RTEAST | CSM | eq RTINIT +6 | module-private (prog-align) | REENTRY ERASABLES. (206D) |
| RTED1 | CSM | 2 | module-private (guid-entry) | P37 ** RETURN TO EARTH (PAD LOAD **** (2D) |
| RTEDVD | CSM | eq END-IN/M | module-private (guid-entry; also io-nouns) | RETURN-TO-EARTH STORAGE. (93D) |
| RTEGAM2D | CSM | eq RTEDVD +2 | module-private (guid-entry; also io-nouns) | RETURN-TO-EARTH STORAGE. (93D) |
| RTENCKEX | CSM | eq NN2 +2 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| RTGO | CSM | eq Q2 +2 | module-private (guid-entry; also io-nouns) | REENTRY ERASABLES. (206D) |
| RTGON64 | CSM | eq RTGO | module-private (io-nouns) | REENTRY ERASABLES. (206D) |
| RTGON67 | CSM | eq RTGO | module-private (guid-entry; also io-nouns) | REENTRY ERASABLES. (206D) |
| RTHETA | CSM+LM | eq RRATE +2 | module-private (prog-rendz; also io-nouns, sys-misc) | VERB 83 STORAGE. (20D) |
| RTIG | CSM+LM | eq END-IN/M | module-private (guid-powered; also prog-rendz) | P30, P40 INTERFACE. (20D) |
| RTINIT | CSM | eq END-SVCR | module-private (guid-entry; also exec-sched, prog-align) | REENTRY ERASABLES. (206D) |
| RTMAG | LM | eq MU/A +2 | module-private (guid-powered; also prog-rendz) | ASTEER STORAGE. (22D) |
| RTMU | CSM+LM | eq RTX2 +1 | module-private (prog-rendz) | RENDEZVOUS GUIDANCE STORAGE. -- P32...P35 -- (10D) |
| RTNAPSE | CSM+LM | eq RTNLAMB | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| RTNHOLD | LM | eq PIFPSET +1 | module-private (guid-landing) | THESE ERASABLES MUST NOT OVERLAY GOBLTIME OR SERVI |
| RTNLAMB | CSM+LM | eq VTARGET +6 | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| RTNORM | CSM | eq RTEAST +6 | module-private (prog-align) | REENTRY ERASABLES. (206D) |
| RTNPRM | CSM+LM | eq ECC +2 | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| RTNSAVER | CSM+LM | eq TEM5 | module-private (guid-powered) | AX*SR*T STORAGE. (6D) |
| RTNTR | CSM+LM | eq RTNLAMB | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| RTNTT | CSM+LM | eq RTNLAMB | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| RTRN | CSM+LM | eq QSAVED +1 | module-private (prog-rendz) | S34/35.5,P34-P35 STORAGE. (6D) |
| RTRNLOC | CSM | eq TTMP2 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| RTSR1/MU | CSM+LM | eq RTMU +2 | module-private (prog-rendz) | RENDEZVOUS GUIDANCE STORAGE. -- P32...P35 -- (10D) |
| RTSTBASE | LM | 1 | module-private (prog-rendz; also sys-misc) | SOME R04(V63)-R77 RADAR TEST STORAGE. (6D) |
| RTSTDEX | LM | 1 | module-private (prog-rendz; also sys-misc) | SOME R04(V63)-R77 RADAR TEST STORAGE. (6D) |
| RTSTLOC | LM | 1 | module-private (prog-rendz; also sys-misc) | SOME R04(V63)-R77 RADAR TEST STORAGE. (6D) |
| RTSTMAX | LM | 1 | module-private (prog-rendz; also sys-misc) | SOME R04(V63)-R77 RADAR TEST STORAGE. (6D) |
| RTX1 | CSM+LM | eq ELEV +2 | module-private (prog-rendz; also guid-powered, nav-integ) | RENDEZVOUS GUIDANCE STORAGE. -- P32...P35 -- (10D) |
| RTX2 | CSM+LM | eq RTX1 +1 | module-private (guid-powered; also prog-rendz, nav-integ, guid-entry) | RENDEZVOUS GUIDANCE STORAGE. -- P32...P35 -- (10D) |
| RUN | CSM | 1 | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| RUNIT | LM | eq ALTBITS +2 | module-private (guid-landing; also guid-powered) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| RUPTAGN | CSM+LM | 1 | module-private (exec-sched; also prog-rendz) | WAITLIST REPEAT FLAG. (1D) |
| RUPTREG1 | CSM+LM | 1 | module-private (sys-misc; also exec-sched, guid-landing, imu, prog-align...) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| RUPTREG2 | CSM+LM | 1 | module-private (dap-lm; also exec-sched, imu, sys-misc, guid-landing...) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| RUPTREG3 | CSM+LM | 1 | module-private (sys-misc; also exec-sched, guid-landing, imu, io-pinball...) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| RUPTREG4 | CSM+LM | 1 | module-private (exec-sched; also io-pinball, sys-misc) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| RUPTSTOR | CSM+LM | eq ITEMP3 | module-private (unreferenced in code; overlay alias or spare) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| RUTMXTEM | CSM+LM | 1 | module-private (io-nouns; also io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| RVAR | CSM | 2 | module-private (prog-rendz) | R22 PAD LOADS (5D). |
| RVARMIN | CSM+LM | 3 | module-private (prog-rendz) | R22 PAD LOADS (5D). |
| RVEC | CSM+LM | eq R1VEC | module-private (prog-rendz; also nav-conic, guid-entry) | CONICS ROUTINE STORAGE. (87D) |
| RWORD1 | CSM | 13 | module-private (dap-csm) | R22 PAD LOADS (5D). |
| RWORD2 | CSM | eq RWORD1 +1 | module-private (dap-csm) | R22 PAD LOADS (5D). |
| RXZ | LM | eq SCALSHFT +1 | module-private (prog-rendz) | LRS22.3 STORAGE. (CAN SHARE WITH P30'S AND OVERLAY |
| RZC | CSM | eq UBAR2 +6 | module-private (prog-rendz) | S22.1 STORAGE. (36D) |
| S22EORM | CSM | eq S22TPRIM +2 | module-private (prog-rendz) | P22 STORAGE. (6D) |
| S22LOC | CSM | eq 8NN +1 | module-private (prog-rendz) | P22-P23 STORAGE. (8D) |
| S22RTNEX | CSM | eq MARKDOWN +7 | module-private (prog-rendz) | S22.1 (1D) |
| S22TOFF | CSM | eq S22WUNL +1 | module-private (prog-rendz) | P22 STORAGE. (6D) |
| S22TPRIM | CSM | eq S22TOFF +2 | module-private (prog-rendz) | P22 STORAGE. (6D) |
| S22WSUBL | CSM | 1 | module-private (prog-rendz) | P22 STORAGE. -- PAD LOADED -- (5D) |
| S22WUNL | CSM | eq TSTRT +2 | module-private (unreferenced in code; overlay alias or spare) | P22 STORAGE. (6D) |
| S61DT | CSM | eq 60GENRET +1 | module-private (exec-sched; also guid-entry) | ENTRY RESTART PROTECTION STORAGE. --KEEP TEMPS IN  |
| SAC | CSM | eq STAR +6 | module-private (prog-align; also imu, io-nouns, sys-misc) | ALIGNMENT/SYSTEST/AUTO OPTICS COMMON STORAGE. (17D |
| SALFA | CSM | eq CALFA +1 | module-private (guid-entry) | MISCELLANEOUS PERMANENT ERASABLE. |
| SAMPLIM | LM | 1 | module-private (exec-sched; also prog-rendz) | UNSWITCHED RADAR ERASABLE |
| SAMPLSUM | LM | 4 | module-private (prog-rendz; also guid-powered) | UNSWITCHED RADAR ERASABLE |
| SATRATE | CSM | eq BOOSTEMP +3 | module-private (guid-entry) | P11 SATURN I/F (9D) |
| SATRLRT | CSM | eq POLYNUM +15D | module-private (guid-entry) | SATURN BOOST STORAGE. SAVE TILL RCS DAP OPERATION. |
| SATSCALE | CSM | eq BIASAK +3 | module-private (guid-entry) | P11 SATURN I/F (9D) |
| SATSW | CSM | eq SATRATE +4 | module-private (guid-entry) | P11 SATURN I/F (9D) |
| SAVBNK | CSM | eq GENRET +1 | module-private (prog-rendz) | R61CSM STORAGE. (1D) |
| SAVE | CSM+LM | eq GENPL +24D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| SAVEDCDU | CSM | eq SAVBNK +1 | module-private (prog-rendz) | CRS61.1 STORAGE FOR AUTOPILOT BANK. (3D) |
| SAVEHAND | LM | 2 | module-private (dap-lm) | OTHER VARIABLES (5D) |
| SAVEPOS | CSM | eq Q6111 +1 | module-private (prog-rendz) | CRS61.1 STORAGE. --A SUBSET OF P20-- (14D) |
| SAVESR | LM | eq AXISCTR | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| SAVET-30 | LM | eq TTFDISP +2 | module-private (exec-sched; also guid-powered) | BURN PROG STORAGE. (2D) |
| SAVEVEL | CSM | eq SAVEPOS +6 | module-private (prog-rendz) | CRS61.1 STORAGE. --A SUBSET OF P20-- (14D) |
| SAVLEMV | LM | eq NSRCHPNT +1 | module-private (prog-rendz) | LRS24.1 STORAGE. (CAN SHARE WITH P30'S) (40D) |
| SAVQR52 | CSM+LM | eq ZNB1 +6 | module-private (prog-align) | AUTO-OPTICS STORAGE -R52- |
| SAX | CSM+LM | eq VMOON +6 | module-private (prog-align) | OVERLAYS WITHIN ALIGNMENT/SYSTEST COMMON STORAGE ( |
| SCALSAVE | CSM+LM | 2 | module-private (imu) | STANDBY VERB ERASABLES. REDOCTR BEFORE THETAD (DWN |
| SCALSHFT | LM | eq MZ +6 | module-private (prog-rendz) | LRS22.3 STORAGE. (CAN SHARE WITH P30'S AND OVERLAY |
| SCAXIS | CSM+LM | eq COF +6 | module-private (prog-align; also guid-powered, imu, prog-rendz, guid-landing) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| SCOUNT | CSM+LM | eq ERCOUNT +1 | module-private (sys-misc) | SELF-CHECK ASSIGNMENTS (17D) |
| SCRATCH | LM | eq GTSTEMPS +7 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| SECMAX | CSM+LM | eq DELEL +2 | module-private (prog-rendz) | RENDEZVOUS GUIDANCE STORAGE. -- P32 ... P35 -- (8D |
| SELFERAS | CSM+LM | 1 | module-private (unreferenced in code; overlay alias or spare) | SELF-CHECK ASSIGNMENTS (17D) |
| SELFRET | CSM+LM | eq ERESTORE +1 | module-private (exec-sched; also sys-misc) | SELF-CHECK ASSIGNMENTS (17D) |
| SENSETYP | LM | 1 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| SEPMNRET | CSM+LM | eq TEM4 | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| SEPSCRET | CSM+LM | eq TEM4 | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| SFAIL | CSM+LM | eq SELFERAS | module-private (io-pinball; also sys-misc) | SELF-CHECK ASSIGNMENTS (17D) |
| SFCONST1 | CSM+LM | eq GENPL +27D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| SFTEMP1 | CSM+LM | eq VBUF +1 | module-private (io-nouns; also io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| SFTEMP2 | CSM+LM | eq VBUF +2 | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| SGNOFF | CSM+LM | eq VBUF +1 | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| SGNON | CSM+LM | eq VBUF | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| SGNRDOT | CSM+LM | eq RTNPRM +1 | module-private (nav-conic; also guid-entry) | CONICS ROUTINE STORAGE. (87D) |
| SGNRT | CSM | eq DELOGART +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| SHAFTA | CSM | eq DESOPTS | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| SHAFTVAR | LM | 1 | module-private (prog-rendz) | P22 STORAGE. -- PAD LOADED -- (2D) |
| SHFTFLAG | LM | eq GTSTEMPS +2 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| SIGNRET | CSM+LM | eq VBUF +3 | module-private (unreferenced in code; overlay alias or spare) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| SINB | CSM+LM | eq -COSB +2 | module-private (unreferenced in code; overlay alias or spare) | OVERLAYS WITHIN ALIGNMENT/SYSTEST/CALCSMSC COMMON  |
| SINCDU | CSM+LM | eq STARALGN | module-private (guid-powered) | STARALIGN ERASABLES. (13D) |
| SINCDUX | CSM+LM | eq SINCDU +4 | module-private (dap-csm; also guid-entry, guid-landing, guid-powered, imu...) | STARALIGN ERASABLES. (13D) |
| SINCDUY | CSM+LM | eq SINCDU | module-private (guid-powered; also imu, prog-align) | STARALIGN ERASABLES. (13D) |
| SINCDUZ | CSM+LM | eq SINCDU +2 | module-private (guid-powered; also guid-entry, guid-landing, imu, prog-align) | STARALIGN ERASABLES. (13D) |
| SINM | CSM | eq SALFA +1 | module-private (guid-entry) | MISCELLANEOUS PERMANENT ERASABLE. |
| SINO | CSM | eq COSM +1 | module-private (guid-entry) | MISCELLANEOUS PERMANENT ERASABLE. |
| SINOCOSM | CSM | eq COSO +1 | module-private (guid-entry) | MISCELLANEOUS PERMANENT ERASABLE. |
| SINTH | CSM+LM | eq 18D | module-private (imu; also nav-aux, prog-align, prog-rendz, guid-entry...) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| SINTHETA | LM | eq ULC +6 | module-private (prog-rendz) | LRS22.3 STORAGE. (CAN SHARE WITH P30'S AND OVERLAY |
| SKALSKAL | LM | 1 | module-private (guid-powered) | ERASABLES FOR THE R2 LUNAR POTENTIAL MODEL (2D) |
| SKEEP1 | CSM+LM | eq SCOUNT +3 | module-private (sys-misc) | SELF-CHECK ASSIGNMENTS (17D) |
| SKEEP2 | CSM+LM | eq SKEEP1 +1 | module-private (sys-misc) | SELF-CHECK ASSIGNMENTS (17D) |
| SKEEP3 | CSM+LM | eq SKEEP2 +1 | module-private (sys-misc) | SELF-CHECK ASSIGNMENTS (17D) |
| SKEEP4 | CSM+LM | eq SKEEP3 +1 | module-private (exec-sched; also sys-misc) | SELF-CHECK ASSIGNMENTS (17D) |
| SKEEP5 | CSM+LM | eq SKEEP4 +1 | module-private (exec-sched; also sys-misc) | SELF-CHECK ASSIGNMENTS (17D) |
| SKEEP6 | CSM+LM | eq SKEEP5 +1 | module-private (sys-misc) | SELF-CHECK ASSIGNMENTS (17D) |
| SKEEP7 | CSM+LM | eq SKEEP6 +1 | module-private (exec-sched; also sys-misc) | SELF-CHECK ASSIGNMENTS (17D) |
| SKIPU | LM | 2 | module-private (dap-lm) | ASCENT VARIABLES (10D) |
| SKIPV | LM | eq SKIPU +1 | module-private (dap-lm) | ASCENT VARIABLES (10D) |
| SLOPE | CSM | eq CDUZD +2 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| SMODE | CSM+LM | eq SELFRET +1 | module-private (sys-misc; also exec-sched, io-nouns) | SELF-CHECK ASSIGNMENTS (17D) |
| SNTH | CSM+LM | eq UR1 +6 | module-private (prog-rendz; also nav-conic, guid-entry) | CONICS ROUTINE STORAGE. (87D) |
| SOUTHDR | CSM+LM | 1 | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| SPNDX | CSM+LM | eq INTTEMP | module-private (dap-csm; also imu) | OVERLAYS WITHIN TVC DAP |
| SPOLYARG | CSM | eq BODY1 +1 | module-private (guid-entry) | STORAGE FOR VHHDOT AND ATTDSP |
| SPRTETIG | CSM | eq TIG | module-private (guid-entry) | OVERLAYS WITHIN RETURN-TO-EARTH STORAGE. |
| SPRTEX | CSM | eq PHI2 +2 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| SQ | CSM+LM | eq ROOTRET | module-private (core-numeric) | SINGLE PRECISION SUBROUTINE TEMPORARIES (3D) |
| SQRARG | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | SINGLE PRECISION SUBROUTINE TEMPORARIES (3D) |
| SRRETURN | CSM | eq USSTAR +6 | module-private (prog-rendz) | S22.1 STORAGE. (36D) |
| SS | LM | eq R1C +6 | module-private (guid-powered) | ASTEER STORAGE. (22D) |
| STAR | CSM+LM | eq MGC +2 | module-private (prog-align; also exec-sched, imu) | ALIGNMENT/SYSTEST/AUTO OPTICS COMMON STORAGE. (17D |
| STARAD | CSM+LM | eq ZDC +6 | module-private (imu; also prog-align, io-nouns) | ALIGNMENT/SYSTEST COMMON STORAGE (18D) |
| STARALGN | CSM+LM | 12 | module-private (unreferenced in code; overlay alias or spare) | STARALIGN ERASABLES. (13D) |
| STARCODE | CSM+LM | 1 | module-private (prog-align; also imu, io-nouns, prog-rendz) | STARALIGN ERASABLES. (13D) |
| STARIND | CSM+LM | eq BESTJ +1 | module-private (prog-align; also exec-sched, guid-powered, prog-rendz) | ALIGNMENT STORAGE. (5D) |
| STARM | CSM+LM | eq 32D | module-private (prog-align) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| STARSAV1 | CSM+LM | eq DVLOS | module-private (prog-align; also sys-misc) | ALIGNMENT (12D) |
| STARSAV2 | CSM+LM | eq STARSAV1 +6 | module-private (prog-align; also prog-rendz, sys-misc) | ALIGNMENT (12D) |
| STARSAV3 | CSM | eq S22RTNEX +1 | module-private (io-nouns; also prog-align, prog-rendz) | P22 STORAGE (6D) |
| STATE | CSM+LM | 12 | flag API: `agc_flag_*()`, `agc_flagword_get/set()` | FLAGWORD RESERVATIONS. (12D) |
| STATEXIT | CSM+LM | 1 | module-private (unreferenced in code; overlay alias or spare) | FLAGWORD RESERVATIONS. (12D) |
| STIKSENS | LM | 1 | module-private (dap-lm; also exec-sched, sys-misc) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| STILBADH | LM | eq LRSCTR +1 | module-private (guid-powered) | R12 (FOR LUNAR LANDING) (6D) |
| STILBADV | LM | eq STILBADH +1 | module-private (guid-powered) | R12 (FOR LUNAR LANDING) (6D) |
| STOREPL | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| STRKTIME | CSM | eq VCNTRTMP +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| STRKTTMP | CSM | eq CNTRTMP +1 | module-private (unreferenced in code; overlay alias or spare) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| STROKER | CSM | eq TEMREG +1 | module-private (dap-csm; also sys-misc) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| SUBEXIT | CSM+LM | eq RTRN +1 | module-private (prog-rendz; also exec-sched, guid-powered, sys-misc) | S34/35.5,P34-P35 STORAGE. (6D) |
| SVMRKDAT | CSM | eq GRP2SVQ +1 | module-private (prog-rendz; also sys-misc) | S22.1 STORAGE. (36D) |
| SW/NDX | CSM | eq CMTMTIME +1 | module-private (guid-entry) | MORE P27 STORAGE. (2D) |
| SWBIT | CSM+LM | eq BUF +1 | module-private (core-numeric) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| SWSAMPLE | CSM | 1 | module-private (sys-misc; also prog-align) | MODE SWITCHING ERASABLE (14D) |
| SWWORD | CSM+LM | eq BUF | module-private (core-numeric) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| SXTOPTN | CSM | eq LOSVEC | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| T-OTHER | CSM+LM | eq TETLEM | `agc_sv_get(AGC_SV_OTHER)` (alias of TETLEM) | STATE VECTORS FOR DOWNLINK (12D) |
| T-TO-ADD | CSM | eq QTEMP1 +1 | module-private (nav-integ) | R41 (2D) |
| T1 | CSM | eq V(T2)/ +6 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| T12 | CSM | eq X(T1) +2 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| T1BITS | CSM | eq TON1 +1 | module-private (guid-entry) | THE FOLLOWING FEW REGISTERS USED ONCE EACH 2 SEC |
| T1TOT2 | CSM+LM | eq RACT1 +6 | module-private (io-nouns; also prog-rendz) | P32,P33 STORAGE OVERLAYING 9X9 W-MATRIX LOCATIONS  |
| T2 | CSM | eq CONICX1 +1 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| T2BITS | CSM | eq TON2 +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| T2TOT3 | CSM+LM | eq TPASS4 | module-private (io-nouns; also prog-rendz) | THE FOLLOWING OVERLAY MEAS. INCORP. ARE AND IN USE |
| T3TOT4 | CSM | eq RELDELV +2 | module-private (guid-entry; also io-nouns) | TPI SEARCH (26D) |
| T5ADR | LM | 2 | module-private (dap-lm; also exec-sched) | PERMANENT LEM DAP STORAGE (12D) |
| T5LOC | CSM | 2 | module-private (dap-csm; also exec-sched, guid-entry, guid-powered, sys-misc) | UNSWITCHED DAP ERASABLE. (4D) |
| T5PHASE | CSM | eq BLAST2 +2 | module-private (dap-csm; also guid-powered) | R22 PAD LOADS (5D). |
| T5TEMP | CSM+LM | 1 | module-private (dap-csm; also dap-lm) | R22 PAD LOADS (5D). |
| T5TIME | CSM | eq CHANTEMP +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| T5TVCDT | CSM | eq YCMD +1 | module-private (dap-csm; also sys-misc) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| T6ADR | CSM | 1 | module-private (unreferenced in code; overlay alias or spare) | UNSWITCHED DAP ERASABLE. (4D) |
| T6FURTHA | LM | 2 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| T6LOC | CSM | 1 | module-private (dap-csm; also exec-sched, guid-entry) | UNSWITCHED DAP ERASABLE. (4D) |
| T6NEXT | LM | 2 | module-private (dap-lm; also exec-sched) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| TABLTTF | LM | eq KEEP-2 +2 | module-private (guid-landing) | THESE ERASABLES MAY BE SHARED WITH CARE |
| TALIGN | LM | eq STARSAV2 +6 | module-private (prog-align; also sys-misc) | ALIGNMENT STORAGE. (23D) |
| TANG | LM | 2 | module-private (io-nouns; also prog-rendz) | UNSWITCHED RADAR ERASABLE |
| TANGNB | LM | eq RDOTM +2 | module-private (prog-align; also prog-rendz, sys-misc) | LRS22.1 (SAME AS PREVIOUS SECTION) ALSO DOWNLINK F |
| TARG1/2 | CSM | eq SOUTHDR | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| TARGETDB | LM | eq -RATEDB | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| TARGTIME | CSM+LM | eq DVLOS +4 | module-private (prog-rendz) | RENDEZVOUS -P34-35 (26D) |
| TAU | CSM | eq ATTSEC +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| TAU. | CSM+LM | eq ALPHAM +2 | module-private (nav-integ; also nav-conic) | CONIC INTEGRATION STORAGE. -- MAY NOT SHARE WITH S |
| TAU1 | CSM | eq TAU +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| TAU2 | CSM | eq TAU1 +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| TAUROD | LM | eq RODSCALE +1 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| TAUVERT | LM | eq V2FG +6 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| TAZEL1 | CSM | 6 | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| TBASE1 | CSM+LM | 1 | module-private (exec-sched; also guid-powered, imu) | TBASE'S AND PHSPRDT'S. (12D) |
| TBASE2 | CSM+LM | 1 | module-private (nav-integ; also guid-landing, guid-powered, imu) | TBASE'S AND PHSPRDT'S. (12D) |
| TBASE3 | CSM+LM | 1 | module-private (unreferenced in code; overlay alias or spare) | TBASE'S AND PHSPRDT'S. (12D) |
| TBASE4 | CSM+LM | 1 | module-private (exec-sched; also guid-powered) | TBASE'S AND PHSPRDT'S. (12D) |
| TBASE5 | CSM+LM | 1 | module-private (guid-entry; also guid-powered, prog-rendz) | TBASE'S AND PHSPRDT'S. (12D) |
| TBASE6 | CSM+LM | 1 | module-private (guid-powered) | TBASE'S AND PHSPRDT'S. (12D) |
| TBITS | CSM | eq TOFF +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| TBUP | LM | eq TTO +2 | module-private (guid-landing; also core-numeric) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| TC | CSM+LM | eq VCV +6 | module-private (sys-misc; also prog-rendz, exec-sched, imu, dap-csm...) | STATE VECTORS FOR ORBIT INTEGRATION. (44D) |
| TCCSM | CSM+LM | 2 | module-private (unreferenced in code; overlay alias or spare) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| TCDH | CSM+LM | eq DIFFALT +2 | module-private (io-nouns; also prog-rendz, sys-misc) | P32 --- P33 (26D) |
| TCGFAPPR | LM | eq GAINAPPR +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| TCGFBRAK | LM | eq GAINBRAK +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| TCGIAPPR | LM | eq TCGFAPPR +1 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| TCGIBRAK | LM | eq TCGFBRAK +1 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| TCLEM | CSM+LM | 2 | module-private (unreferenced in code; overlay alias or spare) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| TCP | LM | 1 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| TCQR | LM | 1 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| TCSI | CSM+LM | eq TCDH +2 | module-private (io-nouns; also prog-rendz, sys-misc) | P32 --- P33 (26D) |
| TDEC | CSM+LM | 21 | module-private (nav-integ) | UNSWITCHED FOR ORBIT INTEGRATION (21D) |
| TDEC2 | CSM+LM | eq DELVTPI | module-private (prog-rendz) | RENDEZVOUS -P34-35 (26D) |
| TDECAY | LM | eq MDOT +2 | module-private (guid-powered) | P40 STORAGE. (6D) |
| TDELTAV | CSM+LM | eq TET +2 | module-private (nav-integ; also nav-aux, prog-align) | STATE VECTORS FOR ORBIT INTEGRATION. (44D) |
| TDESIRED | CSM+LM | eq R2VEC +6 | module-private (nav-conic; also prog-rendz) | CONICS ROUTINE STORAGE. (87D) |
| TDPOS | CSM+LM | eq HOLDW +18D | module-private (unreferenced in code; overlay alias or spare) | MEASUREMENT INCORPORATION STORAGE. (66D) |
| TDVEL | CSM+LM | eq TDPOS +6 | module-private (unreferenced in code; overlay alias or spare) | MEASUREMENT INCORPORATION STORAGE. (66D) |
| TEM1 | CSM+LM | 1 | module-private (core-numeric; also io-pinball, guid-landing, guid-powered, imu) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| TEM1B | CSM | eq GOTOADDR +1 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| TEM2 | CSM+LM | 1 | module-private (guid-powered; also sys-misc, guid-landing) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| TEM3 | CSM+LM | 1 | module-private (guid-powered; also guid-landing) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| TEM4 | CSM+LM | 1 | module-private (io-pinball; also guid-landing) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| TEM5 | CSM+LM | 1 | module-private (guid-landing) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| TEMK | CSM+LM | eq HALFY | module-private (core-numeric) | SINGLE PRECISION SUBROUTINE TEMPORARIES (3D) |
| TEMP | CSM+LM | eq GENPL +5 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| TEMP31 | LM | eq DAPTEMP1 | module-private (dap-lm) | OTHER VARIABLES (5D) |
| TEMP333 | CSM | eq PSI333 +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| TEMPADD | CSM+LM | eq GENPL +4 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| TEMPALFA | CSM | eq TEMPROLL +1 | module-private (guid-entry) | ENTRY RESTART PROTECTION STORAGE. --KEEP TEMPS IN  |
| TEMPBETA | CSM | eq TEMPALFA +1 | module-private (guid-entry) | ENTRY RESTART PROTECTION STORAGE. --KEEP TEMPS IN  |
| TEMPDAP | CSM | 2 | module-private (dap-csm) | R22 PAD LOADS (5D). |
| TEMPFLSH | CSM+LM | 1 | module-private (exec-sched; also io-pinball, prog-rendz) | UNSWITCHED FOR DISPLAY INTERFACE ROUTINES. (10D) |
| TEMPNUM | LM | eq DAPTEMP4 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| TEMPOR1 | CSM | 2 | module-private (prog-rendz) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| TEMPR60 | CSM+LM | 1 | module-private (guid-powered; also imu, guid-landing) | MISCELLANEOUS UNSWITCHED. (16D) |
| TEMPROLL | CSM | eq GENRET | module-private (guid-entry) | ENTRY RESTART PROTECTION STORAGE. --KEEP TEMPS IN  |
| TEMPTIME | CSM+LM | 2 | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| TEMPVAR | CSM+LM | eq DELTAX +3 | module-private (nav-aux) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| TEMREG | CSM | eq ROLLWORD +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| TEMX | CSM+LM | eq AVEGEXIT +2 | module-private (guid-powered; also guid-landing) | EXCEED THE ERASABLE STORAGE REQUIRED BY RENDEZVOUS |
| TEMY | CSM+LM | eq TEMX +1 | module-private (guid-powered; also guid-landing) | EXCEED THE ERASABLE STORAGE REQUIRED BY RENDEZVOUS |
| TEMZ | CSM+LM | eq TEMY +1 | module-private (guid-powered; also guid-landing) | EXCEED THE ERASABLE STORAGE REQUIRED BY RENDEZVOUS |
| TENDAPPR | LM | 1 | module-private (unreferenced in code; overlay alias or spare) | P63 AND P64 CONSTANTS. -- PAD LOADED -- (4D) |
| TENDBRAK | LM | 1 | module-private (guid-landing) | P63 AND P64 CONSTANTS. -- PAD LOADED -- (4D) |
| TENMDOT | CSM | eq KTLX/I +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| TEPHEM | CSM+LM | 3 | `agc_erase_tephem()` / `agc_erase_set_tephem()` | PERMANENT STATE VECTORS AND TIMES. (101D) |
| TEPHEM1 | CSM | eq LIFTTEMP +2 | module-private (guid-entry) | P11 STORAGE. (9D) |
| TERM1TMP | CSM+LM | eq BUF2 | module-private (guid-powered) | AX*SR*T STORAGE. (6D) |
| TERRLAMB | CSM+LM | eq DELDEP | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| TET | CSM+LM | eq VRECT +6 | module-private (prog-rendz; also nav-integ, io-nouns, prog-align, guid-entry...) | STATE VECTORS FOR ORBIT INTEGRATION. (44D) |
| TET1 | CSM | eq 30D | module-private (unreferenced in code; overlay alias or spare) | STATE VECTORS FOR ORBIT INTEGRATION. (44D) |
| TETCSM | CSM+LM | 2 | `agc_erase_tetcsm()` / `agc_erase_set_tetcsm()` | PERMANENT STATE VECTORS AND TIMES. (101D) |
| TETLEM | CSM+LM | 2 | `agc_erase_tetlem()` / `agc_erase_set_tetlem()` | PERMANENT STATE VECTORS AND TIMES. (101D) |
| TETOTHER | CSM+LM | eq TETLEM | alias: tetlem | PERMANENT STATE VECTORS AND TIMES. (101D) |
| TETTHIS | CSM+LM | eq TETCSM | alias: tetcsm | PERMANENT STATE VECTORS AND TIMES. (101D) |
| TEVENT | CSM+LM | 2 | `agc_erase_tevent()` / `agc_erase_set_tevent()` | CRS61.1 STORAGE. -- USED IN R63 (VERB 89) -- (5D) |
| TF | CSM | eq THETL +2 | module-private (prog-rendz) | TPI SEARCH (S17.1, S17.2) P17 STORAGE.(10D) |
| TFF | CSM+LM | eq V82FLAGS +1 | module-private (io-nouns; also prog-rendz) | MORE VERB 82 NOT SHARING WITH VERB 83 (9D) |
| TFI | CSM+LM | eq THETZERO +2 | module-private (prog-rendz) | TPI SEARCH (26D) |
| TFO | CSM | eq HP +2 | module-private (prog-rendz) | TPI SEARCH (26D) |
| TGO | CSM+LM | eq DVTOTAL +2 | `agc_erase_tgo()` / `agc_erase_set_tgo()` | SERVICER STORAGE. (13D) |
| THETA | CSM+LM | eq 20D | module-private (imu; also nav-aux, prog-align, guid-powered) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| THETA(1) | CSM | eq -TPER +2 | module-private (unreferenced in code; overlay alias or spare) | MORE VERB 82 NOT SHARING WITH VERB 83 (9D) |
| THETAD | CSM+LM | 3 | `agc_erase_thetad(i)` / `agc_erase_set_thetad(i,v)` (0..2) | STANDBY VERB ERASABLES. REDOCTR BEFORE THETAD (DWN |
| THETADX | CSM | eq ERRORZ +1 | module-private (dap-csm; also guid-powered, sys-misc) | OVERLAYS WITHIN TVC DAP |
| THETADY | CSM | eq THETADX +1 | module-private (dap-csm; also guid-powered) | OVERLAYS WITHIN TVC DAP |
| THETADZ | CSM | eq THETADY +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| THETAH | CSM | eq RDOT +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| THETAN | CSM+LM | eq THETAX +6 | module-private (guid-entry) | SYSTEM TEST STORAGE. (174) |
| THETAN1 | CSM | eq THETAX1 +6 | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| THETAX | CSM | eq ALK +2 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| THETAX1 | CSM | eq ALK +2 | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| THETL | CSM | eq XRS +2 | module-private (prog-rendz) | TPI SEARCH (S17.1, S17.2) P17 STORAGE.(10D) |
| THETZERO | CSM | eq IT +6 | module-private (io-nouns; also prog-rendz) | TPI SEARCH (26D) |
| THISTPIP | LM | eq LASTTPIP +2 | module-private (guid-landing) | P66 ERASABLES (R.O.D.) (14D) |
| TIG | CSM+LM | eq DELVSLV +6 | `agc_erase_tig()` / `agc_erase_set_tig()` | EXTERNAL DELTA-V UPDATE. (21D) |
| TIGSAVE | LM | eq P21TIME | module-private (guid-powered) | ASTEER STORAGE. (22D) |
| TIGSAVEP | LM | eq SCAXIS | module-private (guid-powered) | ASTEER STORAGE. (22D) |
| TIME/RTO | CSM | eq VEL +6 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| TIME2SAV | CSM+LM | 2 | module-private (imu; also exec-sched) | STANDBY VERB ERASABLES. REDOCTR BEFORE THETAD (DWN |
| TIMEHOLD | LM | 2 | module-private (prog-rendz) | UNSWITCHED RADAR ERASABLE |
| TIMEMO | CSM | 77 | module-private (nav-aux) | CONISEX (LUNAR AND SOLAR EPHEM) STORAGE. -- PAD LO |
| TIMER | CSM+LM | eq GENPL +28D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| TIMSUBO | CSM+LM | eq TEPHEM | module-private (nav-aux) | P40 PAD LOADS (6D) |
| TINT | CSM+LM | eq TPASS4 | module-private (prog-rendz) | P30-P40 COMMON STORAGE. (3D) |
| TINTSOI | CSM+LM | eq DELTAR | module-private (prog-rendz) | RENDEZVOUS -P34-35 (26D) |
| TITER | CSM+LM | eq TSTRT | module-private (prog-rendz) | S34.1 STORAGE. (OVERLAYS S35.1 STORAGE) (1) |
| TJETU | LM | eq TJU | module-private (dap-lm) | INPUT TO TJET LAW (PERMANENT ERASABLES). (48D) |
| TJP | LM | 3 | module-private (dap-lm) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| TJU | LM | eq TJP +1 | module-private (dap-lm) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| TJV | LM | eq TJP +2 | module-private (dap-lm) | TORQUE VECTOR RECONSTRUCTION VARIABLES: (18D) |
| TLAND | LM | eq W | module-private (guid-landing; also prog-align, sys-misc) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| TLIFTOFF | CSM | eq TEVENT | module-private (guid-entry) | CRS61.1 STORAGE. -- USED IN R63 (VERB 89) -- (5D) |
| TM | CSM+LM | eq CAM | module-private (imu; also guid-entry) | FIRST-ORDER OVERLAYS IN KALCMANU |
| TMARK | CSM+LM | 2 | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| TMFI | CSM+LM | eq TMIS | module-private (imu) | FIRST-ORDER OVERLAYS IN KALCMANU |
| TMINDEX | CSM+LM | 1 | module-private (sys-misc) | DOWNLINK STORAGE (18D) |
| TMIS | CSM+LM | eq KDPNDX +1 | module-private (imu) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| TMODULO | CSM | 2 | module-private (unreferenced in code; overlay alias or spare) | KEPLER STORAGE (6D) |
| TMP1 | CSM | eq CMDTMP +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| TMP2 | CSM | eq TMP1 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| TMP3 | CSM | eq TMP2 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| TMP4 | CSM | eq TMP3 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| TMP5 | CSM | eq TMP4 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| TMP6 | CSM | eq TMP5 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| TNEWA | LM | 2 | module-private (guid-powered) | ASTEER. -- PAD LOADED -- (2D) |
| TNIT | CSM | eq VRPREV +6 | module-private (guid-powered) | S40.9 STORAGE (16D) |
| TNITPREV | CSM | eq TNIT +2 | module-private (guid-powered) | S40.9 STORAGE (16D) |
| TNUV | CSM+LM | eq TDELTAV +6 | module-private (nav-integ; also nav-aux, prog-align) | STATE VECTORS FOR ORBIT INTEGRATION. (44D) |
| TOFF | CSM | eq BETACOM +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| TOLDAZ1 | CSM | eq LUNCHAZ1 +6 | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| TOLDAZMT | CSM | eq AINLA +125D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| TON1 | CSM | eq JNDX1 +1 | module-private (guid-entry) | THE FOLLOWING FEW REGISTERS USED ONCE EACH 2 SEC |
| TON2 | CSM | eq TBITS +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| TORQNDX | CSM+LM | eq WANGT | module-private (sys-misc; also imu) | SYSTEM TEST STORAGE. (174) |
| TPASS4 | CSM+LM | eq DELVSAB +2 | module-private (guid-powered; also prog-rendz, sys-misc, guid-entry) | P30-P40 COMMON STORAGE. (3D) |
| TPIP | LM | eq FLPASS0 +1 | module-private (guid-landing) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| TPIPOLD | LM | eq TABLTTF +9D | module-private (guid-landing) | THESE ERASABLES MAY BE SHARED WITH CARE |
| TPREV | CSM+LM | eq DEPREV | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| TRAKFWDV | LM | eq TRAKLATV +1 | module-private (guid-landing) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| TRAKLATV | LM | eq FORVEL +1 | module-private (guid-landing) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| TRANSM1 | CSM+LM | eq  | module-private (imu; also sys-misc) | SOME P34 STORAGE. (OVERLAYS P35.1 STORAGE) (2D) |
| TRANSTM | LM | eq LNCHTM +2 | module-private (prog-rendz) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| TRAPEDP | LM | 6 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| TRAPEDQ | LM | eq TRAPEDP +1 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| TRAPEDR | LM | eq TRAPEDP +2 | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| TREDES | LM | eq FUNNYDSP | module-private (guid-landing) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| TRIPA | CSM+LM | eq DELTAX | module-private (nav-aux) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| TRKMKCNT | CSM+LM | 1 | module-private (guid-powered; also exec-sched, nav-integ, prog-rendz, guid-entry...) | MARK STORAGE. (2) |
| TRUNA | CSM | eq DESOPTT | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| TRUNBIAS | CSM | 1 | module-private (prog-rendz) | R57 STORAGE. -- MUST BE UNSHARED EXCEPT IN BOOST O |
| TRUNION | CSM | eq IDOFLMK +1 | module-private (prog-rendz) | P22-P23 STORAGE. (8D) |
| TRUNVAR | LM | 1 | module-private (prog-rendz) | P22 STORAGE. -- PAD LOADED -- (2D) |
| TRUNX | CSM | eq SVMRKDAT +36D | module-private (unreferenced in code; overlay alias or spare) | S22.1 STORAGE. (36D) |
| TSIGHT | CSM+LM | eq PLANVEC +6 | module-private (prog-align) | AUTO-OPTICS STORAGE -R52- |
| TSTART82 | CSM+LM | eq RPADTEM +2 | module-private (prog-rendz; also guid-powered, prog-align) | EXTENDED VERB 82 STORAGE. (6D) |
| TSTRT | CSM+LM | eq END-P30S | module-private (prog-rendz; also sys-misc) | S35.1 STORAGE. (2D) |
| TTE | CSM | eq VIO +2 | module-private (guid-entry; also io-nouns, sys-misc) | REENTRY ERASABLES. (206D) |
| TTE1 | CSM | eq ASPS +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| TTEMP | CSM+LM | eq TMIS +4 | module-private (imu) | FIRST-ORDER OVERLAYS IN KALCMANU |
| TTF/8 | LM | eq LAND +6 | module-private (guid-landing; also guid-powered, sys-misc) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| TTF/8TMP | LM | eq LANDTEMP +6 | module-private (guid-landing) | THESE ERASABLES MAY BE SHARED WITH CARE |
| TTFDISP | LM | eq HDOTDISP +2 | module-private (guid-landing; also io-nouns) | SERVICER STORAGE (6D) |
| TTHROT | LM | eq FC +2 | module-private (guid-landing) | THESE ERASABLES MUST NOT OVERLAY GOBLTIME OR SERVI |
| TTMP1 | CSM | eq YDELOFF +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| TTMP2 | CSM | eq TTMP1 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| TTO | LM | eq VE +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| TTOAXIS | LM | eq DAPTEMP4 | module-private (dap-lm) | THE FOLLOWING LM DAP ERASABLES ARE ZEROED IN THE S |
| TTOGO | CSM+LM | eq QTEMP +1 | `agc_erase_ttogo()` / `agc_erase_set_ttogo()` | P30-P40 STORAGE. (4D) |
| TTPI | CSM+LM | eq TTOGO +2 | `agc_erase_ttpi()` / `agc_erase_set_ttpi()` | P30-P40 STORAGE. (4D) |
| TTPIO | CSM+LM | eq TCSI +2 | module-private (prog-rendz) | P32 --- P33 (26D) |
| TUSED | CSM | eq JETAG +1 | module-private (guid-entry) | ANGLE REGISTERS FOR ENTRY DAPS |
| TVCEXPHS | CSM | eq MCDUZDOT +1 | module-private (dap-csm; also guid-powered) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| TVCPHASE | CSM | eq VCNTR +1 | module-private (dap-csm; also guid-powered) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| TVCRCS | CSM | 12 | module-private (unreferenced in code; overlay alias or spare) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| TVCTMP1 | CSM | eq LASTMASP +1 | module-private (unreferenced in code; overlay alias or spare) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| TX789 | CSM+LM | eq END-KALC | module-private (nav-aux) | MEASUREMENT INCORPORATION STORAGE (R22) STORAGE. ( |
| TXO | LM | eq VGVECT +6 | module-private (guid-landing) | ASCENT GUIDANCE ERASABLES. (21D) |
| U2 | CSM+LM | eq RTNLAMB +1 | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| UBAR0 | CSM | eq TRUNX +1 | module-private (prog-rendz) | S22.1 STORAGE. (36D) |
| UBAR1 | CSM | eq UBAR0 +6 | module-private (prog-rendz) | S22.1 STORAGE. (36D) |
| UBAR2 | CSM | eq UBAR1 +6 | module-private (prog-rendz) | S22.1 STORAGE. (36D) |
| UBX/2 | CSM | eq UZA/2 +6 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| UBY/2 | CSM | eq UBX/2 +6 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| UBZ/2 | CSM | eq UBY/2 +6 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| UCL | CSM | eq DELTAX +12D | module-private (prog-rendz) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| UCLSTAR | CSM | eq VZC +6 | module-private (prog-rendz) | S22.1 STORAGE. (36D) |
| UCSM | LM | eq RSUBL +6 | module-private (prog-rendz) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| UERROR | LM | eq DAPTREG5 | module-private (dap-lm) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| UH/ | CSM | eq UV1/ | module-private (guid-entry) | OVERLAYS WITHIN RETURN-TO-EARTH STORAGE. |
| UHYP | LM | eq DALTRATE +1 | module-private (guid-landing; also guid-powered) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| UHZP | LM | eq UHYP +6 | module-private (guid-landing; also guid-powered) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| UJETCTR | LM | eq PJETCTR +1 | module-private (dap-lm) | DOCKED JET INHIBITION COUNTERS (3D) |
| ULC | LM | eq RXZ +2 | module-private (prog-rendz) | LRS22.3 STORAGE. (CAN SHARE WITH P30'S AND OVERLAY |
| ULOS | CSM+LM | eq UNRM +6 | module-private (prog-rendz) | RENDEZVOUS -P34-35 (26D) |
| UM | CSM | eq VTEMP +6 | module-private (prog-rendz) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| UN | CSM+LM | eq GEOMSGN +1 | module-private (nav-conic; also prog-rendz, guid-powered) | CONICS ROUTINE STORAGE. (87D) |
| UNFC/2 | LM | eq FLPAUTNO +1 | module-private (guid-landing; also guid-powered, sys-misc) | OVERLAYING KALCMANU STORAGE: (26D) |
| UNFV/2 | LM | eq UNWC/2 +6 | module-private (guid-landing) | OVERLAYING KALCMANU STORAGE: (26D) |
| UNFVX/2 | LM | eq UNFV/2 | module-private (unreferenced in code; overlay alias or spare) | OVERLAYING KALCMANU STORAGE: (26D) |
| UNFVY/2 | LM | eq UNFV/2 +2 | module-private (guid-landing) | OVERLAYING KALCMANU STORAGE: (26D) |
| UNFVZ/2 | LM | eq UNFV/2 +4 | module-private (guid-landing) | OVERLAYING KALCMANU STORAGE: (26D) |
| UNI | CSM | eq RT +6 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| UNIT/R/ | LM | eq HCALC +2 | module-private (guid-landing; also guid-powered) | SERVICER STORAGE. (69D) |
| UNITGOBL | LM | eq R | module-private (guid-powered) | SERVICER STORAGE. (69D) |
| UNITR | CSM | 6 | module-private (guid-entry; also guid-powered) | AVERAGEG INTEGRATOR STORAGE. (8D) |
| UNITV | CSM | eq UNI +6 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| UNITW | CSM | 6 | module-private (guid-entry; also guid-powered, prog-align) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| UNLR/2 | LM | eq 20D | module-private (unreferenced in code; overlay alias or spare) | ERASABLES CONVENIENTLY DEFINABLE IN THE WORK AREA |
| UNLRB/2 | LM | eq 20D | module-private (unreferenced in code; overlay alias or spare) | ERASABLES CONVENIENTLY DEFINABLE IN THE WORK AREA |
| UNP36 | CSM+LM | eq RPASS36 +6 | module-private (prog-rendz) | R36 SCRATCHPAD STORAGE (13D) |
| UNRM | CSM+LM | eq DVLOS +6 | module-private (prog-rendz; also guid-powered) | RENDEZVOUS -P34-35 (26D) |
| UNVEC | CSM+LM | eq VACT3 | module-private (prog-rendz) | THE FOLLOWING OVERLAY MEAS. INCORP. ARE AND IN USE |
| UNWC/2 | LM | eq UNFC/2 +6 | module-private (guid-landing) | OVERLAYING KALCMANU STORAGE: (26D) |
| UNX/2 | LM | eq 0 | module-private (guid-landing) | DEFINED IN THE WORK AREA: (18D) |
| UNY/2 | LM | eq 6 | module-private (guid-landing) | DEFINED IN THE WORK AREA: (18D) |
| UNZ/2 | LM | eq 14 | module-private (guid-landing) | DEFINED IN THE WORK AREA: (18D) |
| UP1 | CSM+LM | eq DVLOS | module-private (prog-rendz) | P32 --- P33 (26D) |
| UPBUFF | CSM+LM | eq UPCOUNT +1 | module-private (sys-misc) | P27 (UPDATE PROGRAM) STORAGE. -- OVERLAYS SPEC DNL |
| UPCOUNT | CSM+LM | eq UPVERB +1 | module-private (sys-misc) | P27 (UPDATE PROGRAM) STORAGE. -- OVERLAYS SPEC DNL |
| UPDATRET | CSM+LM | eq POLISH | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| UPOLDMOD | CSM+LM | eq COMPNUMB +1 | module-private (sys-misc) | P27 (UPDATE PROGRAM) STORAGE. -- OVERLAYS SPEC DNL |
| UPSVFLAG | CSM+LM | eq DIFEQCNT +1 | module-private (exec-sched; also nav-integ) | STATE VECTORS FOR ORBIT INTEGRATION. (44D) |
| UPTEMP | CSM+LM | 1 | module-private (sys-misc) | MORE P27 STORAGE. (2D) |
| UPVERB | CSM+LM | eq UPOLDMOD +1 | module-private (sys-misc) | P27 (UPDATE PROGRAM) STORAGE. -- OVERLAYS SPEC DNL |
| UPVERBSV | CSM+LM | 1 | module-private (sys-misc) | MORE P27 STORAGE. (2D) |
| UR1 | CSM+LM | eq MAGVEC2 +2 | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| UR1/ | CSM | eq T2 +2 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| URATEDIF | LM | eq OMEGAU | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| URH | CSM | eq UXA/2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| URONE | CSM | eq V82FLAGS | module-private (guid-entry) | REENTRY CONICS (6D) |
| URRECT | CSM+LM | eq DELT +2 | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| US | CSM | eq STARSAV2 | module-private (prog-rendz) | ALIGNMENT (12D) |
| USSTAR | CSM | eq UCLSTAR +6 | module-private (prog-rendz) | S22.1 STORAGE. (36D) |
| USTAR | CSM | eq MARKDATA +2 | module-private (prog-rendz) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| UT | CSM+LM | eq BDT +6 | module-private (guid-powered) | S40.1 STORAGE. (23D) |
| UV1/ | CSM | eq UR1/ +6 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| UXA/2 | CSM | eq OLDUYA +6 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| UXVECT | LM | eq LOSDESRD +6 | module-private (prog-rendz) | LRS24.1 STORAGE. (CAN SHARE WITH P30'S) (40D) |
| UYA/2 | CSM | eq UXA/2 +6 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| UYVECT | LM | eq UXVECT +6 | module-private (prog-rendz) | LRS24.1 STORAGE. (CAN SHARE WITH P30'S) (40D) |
| UZA/2 | CSM | eq UYA/2 +6 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| V | CSM+LM | eq VS1 +2 | module-private (guid-landing; also guid-powered, guid-entry, prog-rendz) | REENTRY ERASABLES. (206D) |
| V(CSM) | LM | eq V-OTHER | module-private (unreferenced in code; overlay alias or spare) | SERVICER FOR LUNAR ASCENT AND DESCENT (14D) |
| V(T1)/ | CSM | eq RPRE' +2 | module-private (unreferenced in code; overlay alias or spare) | RETURN-TO-EARTH STORAGE. (93D) |
| V(T2)/ | CSM | eq DV +2 | module-private (unreferenced in code; overlay alias or spare) | RETURN-TO-EARTH STORAGE. (93D) |
| V-OTHER | CSM+LM | 6 | `agc_sv_get(AGC_SV_OTHER)` (downlink copy) | STATE VECTORS FOR DOWNLINK (12D) |
| V1 | CSM | eq ENDBUF +1 | module-private (guid-entry) | MORE P27 STORAGE. (2D) |
| V1S | LM | eq R1S +6 | module-private (guid-landing; also guid-powered) | SERVICER STORAGE. (69D) |
| V2(T1)/ | CSM | eq V(T1)/ +6 | module-private (unreferenced in code; overlay alias or spare) | RETURN-TO-EARTH STORAGE. (93D) |
| V2FG | LM | eq HIGHCRIT +1 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| V82FLAGS | CSM+LM | eq VONE +6 | module-private (prog-rendz) | MORE VERB 82 NOT SHARING WITH VERB 83 (9D) |
| V97VCNTR | CSM | 1 | module-private (dap-csm; also exec-sched, guid-powered) | R22 PAD LOADS (5D). |
| VAC1 | CSM+LM | 43 | module-private (unreferenced in code; overlay alias or spare) | VAC AREAS. -- BE CAREFUL OF PLACEMENT -- (220D) |
| VAC1USE | CSM+LM | 1 | module-private (exec-sched; also prog-align) | VAC AREAS. -- BE CAREFUL OF PLACEMENT -- (220D) |
| VAC2 | CSM+LM | 43 | module-private (unreferenced in code; overlay alias or spare) | VAC AREAS. -- BE CAREFUL OF PLACEMENT -- (220D) |
| VAC2USE | CSM+LM | 1 | module-private (exec-sched; also prog-align) | VAC AREAS. -- BE CAREFUL OF PLACEMENT -- (220D) |
| VAC3 | CSM+LM | 43 | module-private (unreferenced in code; overlay alias or spare) | VAC AREAS. -- BE CAREFUL OF PLACEMENT -- (220D) |
| VAC3USE | CSM+LM | 1 | module-private (exec-sched; also prog-align) | VAC AREAS. -- BE CAREFUL OF PLACEMENT -- (220D) |
| VAC4 | CSM+LM | 43 | module-private (unreferenced in code; overlay alias or spare) | VAC AREAS. -- BE CAREFUL OF PLACEMENT -- (220D) |
| VAC4USE | CSM+LM | 1 | module-private (exec-sched; also prog-align) | VAC AREAS. -- BE CAREFUL OF PLACEMENT -- (220D) |
| VAC5 | CSM+LM | 43 | module-private (exec-sched) | VAC AREAS. -- BE CAREFUL OF PLACEMENT -- (220D) |
| VAC5USE | CSM+LM | 1 | module-private (exec-sched; also prog-align) | VAC AREAS. -- BE CAREFUL OF PLACEMENT -- (220D) |
| VACT1 | CSM+LM | eq RDOTV +2 | module-private (prog-rendz) | THE FOLLOWING OVERLAY MEAS. INCORP. ARE AND IN USE |
| VACT2 | CSM+LM | eq 9X9LOC2 | module-private (prog-rendz) | P32,P33 STORAGE OVERLAYING 9X9 W-MATRIX LOCATIONS  |
| VACT3 | CSM+LM | eq RACT3 +6 | module-private (prog-rendz) | P30'S-P17 COMMON STORAGE. (24D) |
| VACT4 | CSM+LM | eq POSTCDH +2 | module-private (prog-rendz) | THE FOLLOWING OVERLAY MEAS. INCORP. ARE AND IN USE |
| VACX | CSM+LM | eq VECTAB +6 | module-private (unreferenced in code; overlay alias or spare) | THESE PROBABLY CAN SHARE INTEGRATION VARIABLES (9D |
| VACY | CSM+LM | eq VACX +2 | module-private (unreferenced in code; overlay alias or spare) | THESE PROBABLY CAN SHARE INTEGRATION VARIABLES (9D |
| VACZ | CSM+LM | eq VACY +2 | module-private (unreferenced in code; overlay alias or spare) | THESE PROBABLY CAN SHARE INTEGRATION VARIABLES (9D |
| VAPFG | LM | eq RAPFG +6 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| VAPFG* | LM | eq AAPFG +6 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| VAPREC | CSM+LM | eq RINIT | module-private (prog-rendz) | P34 AND P33 STORAGE. (OVERLAYS INITVEL/MIDGIM) (24 |
| VARIANCE | CSM+LM | eq DELTAQ +2 | module-private (nav-aux; also prog-rendz) | MEASUREMENT INCORPORATION STORAGE (R22) STORAGE. ( |
| VARK | CSM | eq 1/CONACC +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| VARST0 | CSM | eq TEMP333 +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| VARST5 | CSM | eq VARST0 +5 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| VBARS | CSM | eq A1 +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| VBIAS | LM | eq R60VSAVE | module-private (guid-landing) | SOME VARIABLES FOR SECOND DPS GUIDANCE. (34D) |
| VBRFG | LM | eq RBRFG +6 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| VBRFG* | LM | eq ABRFG +6 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| VBUF | CSM+LM | 6 | module-private (core-numeric; also guid-powered, imu, dap-lm, prog-rendz) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| VCNTR | CSM | eq REPFRAC +1 | module-private (dap-csm; also guid-powered) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| VCNTRTMP | CSM | eq MASSTMP +1 | module-private (dap-csm; also guid-powered) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| VCV | CSM+LM | eq RCV +6 | module-private (prog-rendz; also nav-integ, nav-aux, nav-conic, prog-align...) | STATE VECTORS FOR ORBIT INTEGRATION. (44D) |
| VCVCSM | CSM+LM | 6 | module-private (prog-rendz) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| VCVLEM | CSM+LM | 6 | module-private (prog-rendz) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| VDGVERT | LM | eq ELIDUMMY | module-private (guid-landing) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| VDISP | CSM | eq SPOLYARG +1 | module-private (unreferenced in code; overlay alias or spare) | STORAGE FOR S11.1 |
| VDT/180 | CSM | eq ERRORX | module-private (guid-entry) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| VE | LM | eq AT +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| VEARTH | CSM+LM | eq STARAD | module-private (prog-align) | OVERLAYS WITHIN ALIGNMENT/SYSTEST COMMON STORAGE ( |
| VEC1 | LM | eq STARAD +12D | module-private (prog-align) | P50'S, R50'S Q STORES (2D) |
| VEC2 | LM | eq STAR | module-private (prog-align) | P50'S, R50'S Q STORES (2D) |
| VECOEM | CSM | eq TIMEMO +3 | module-private (nav-aux) | CONISEX (LUNAR AND SOLAR EPHEM) STORAGE. -- PAD LO |
| VECQTEMP | CSM+LM | eq COFSKEW | module-private (imu) | MISCELLANEOUS REGISTERS USED EACH UPDATE. |
| VECTAB | CSM+LM | eq NORMGAM +1 | module-private (nav-integ) | INTEGRATION STORAGE. (95D) |
| VECTABND | LM | eq VECTAB +35D | module-private (unreferenced in code; overlay alias or spare) | INTEGRATION STORAGE. (102D) |
| VEL | CSM | eq UNITV +6 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| VEL/C | CSM | eq ZSCD +6 | module-private (prog-align) | ALIGNMENT STORAGE. (25D) |
| VERBNOUN | CSM+LM | eq KT +2 | module-private (prog-rendz) | S34/35.5,P34-P35 STORAGE. (6D) |
| VERBREG | CSM+LM | 1 | module-private (exec-sched; also io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| VERBSAVE | CSM+LM | 1 | module-private (io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| VERROR | LM | eq UERROR +1 | module-private (unreferenced in code; overlay alias or spare) | JET STATE CHANGE VARIABLES -- TIME (TOFJTCHG), JET |
| VESO | CSM | eq RESO +6 | module-private (nav-aux; also prog-rendz) | CONISEX (LUNAR AND SOLAR EPHEM) STORAGE. -- PAD LO |
| VEX | LM | 2 | module-private (guid-powered) | P40 STORAGE. (6D) |
| VG | CSM+LM | eq XXXALT +2 | module-private (guid-powered) | S40.9 STORAGE (16D) |
| VGBODY | CSM+LM | eq END-P30S | module-private (guid-landing; also guid-powered, io-nouns) | P40 STORAGE. (8D) |
| VGDISP | CSM+LM | eq GAMMAEI +2 | module-private (guid-powered; also io-nouns, guid-entry, prog-rendz) | DISPLAY REGISTER FOR VG (2D) |
| VGPREV | CSM+LM | eq VGTIG | module-private (guid-powered; also exec-sched) | S40.1 STORAGE. (23D) |
| VGTEMP | CSM | eq DELVEET3 | module-private (guid-powered) | P30, P40 INTERFACE. (20D) |
| VGTIG | CSM+LM | eq UT +6 | `agc_erase_vgtig()` / `agc_erase_set_vgtig()` | S40.1 STORAGE. (23D) |
| VGU | LM | eq TPIP +2 | module-private (guid-landing; also sys-misc) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| VGVECT | LM | eq ENGOFFDT +1 | module-private (guid-landing; also sys-misc) | ASCENT GUIDANCE ERASABLES. (21D) |
| VHFCNT | CSM | 1 | module-private (prog-rendz; also exec-sched, guid-entry, guid-powered, io-nouns...) | MARK STORAGE. (2) |
| VHFRANGE | CSM | eq DELTAX | module-private (prog-rendz) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| VHFTIME | CSM | eq POINTEX +1 | module-private (prog-rendz; also sys-misc) | P20, CONICS (SHARING WITH TIME 2 SAV AND SCAL SAV  |
| VHORIZ | LM | eq PIPTEM +3 | module-private (guid-landing; also io-nouns) | SECOND DPS GUIDANCE (LUNAR LANDING) (OVERLAY P32-3 |
| VHY | LM | eq TRAKFWDV +1 | module-private (guid-landing) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| VHZ | LM | eq VHY +1 | module-private (guid-landing) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| VIGN | LM | eq TCGIAPPR +1 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| VINIT | CSM+LM | eq RINIT +6 | module-private (prog-rendz; also guid-powered) | INITVEL/MIDGIM STORAGE. (34D) |
| VIO | CSM | eq VMAGI +2 | module-private (guid-entry; also io-nouns, sys-misc) | REENTRY ERASABLES. (206D) |
| VIPRIME | CSM+LM | eq RTARG1 +6 | module-private (prog-rendz; also guid-powered) | INITVEL/MIDGIM STORAGE. (34D) |
| VJETCTR | LM | eq PJETCTR +2 | module-private (dap-lm) | DOCKED JET INHIBITION COUNTERS (3D) |
| VL | CSM | eq VPRED | module-private (guid-entry; also io-nouns) | REENTRY ERASABLES. (206D) |
| VLAUN | CSM+LM | eq AINLA +44D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| VLAUNS | CSM+LM | eq AINLA +22D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| VMAGI | CSM | eq GMAX +1 | module-private (guid-entry; also io-nouns) | REENTRY ERASABLES. (206D) |
| VMAX | CSM+LM | 1 | module-private (prog-rendz) | P20 STORAGE. -- PAD LOADED -- (4D) |
| VMEAS | LM | eq VSELECT +1 | module-private (guid-powered; also sys-misc) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| VMIN | LM | eq ABTCOF +16D | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| VMOON | CSM+LM | eq VSUN +6 | module-private (prog-align; also nav-aux) | OVERLAYS WITHIN ALIGNMENT/SYSTEST COMMON STORAGE ( |
| VN | CSM+LM | 6 | `agc_erase_vn()` / `agc_erase_set_vn()` | PERM STATE VECTORS FOR BOOST AND DOWNLINK -- WHOLE |
| VN1 | CSM+LM | eq RN1 +6 | module-private (guid-powered; also nav-integ, guid-entry) | EXCEED THE ERASABLE STORAGE REQUIRED BY RENDEZVOUS |
| VN2 | LM | eq HMEAS +2 | module-private (unreferenced in code; overlay alias or spare) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| VNSTORE | CSM | eq SPRTEX +1 | module-private (guid-entry) | RETURN-TO-EARTH STORAGE. (93D) |
| VONE | CSM+LM | eq RONE +6 | module-private (prog-rendz; also nav-aux, guid-entry) | VERB 83 STORAGE. (20D) |
| VONE' | CSM+LM | eq RGEXIT +1 | module-private (unreferenced in code; overlay alias or spare) | END OF PERF. TEST ERASABLE IN BANK 4 |
| VPASS1 | CSM+LM | eq VACT1 +6 | module-private (prog-rendz) | THE FOLLOWING OVERLAY MEAS. INCORP. ARE AND IN USE |
| VPASS2 | CSM+LM | eq UP1 +6 | module-private (prog-rendz) | P32 --- P33 (26D) |
| VPASS3 | CSM+LM | eq RPASS3 +6 | module-private (prog-rendz) | P30'S-P17 COMMON STORAGE. (24D) |
| VPASS4 | CSM+LM | eq RPASS4 +6 | module-private (prog-rendz) | P35-P40 INTERFACE STORAGE. (OVERLAYS P30-P40 I/F S |
| VPPREC | CSM+LM | eq VIPRIME | module-private (prog-rendz) | P34 AND P33 STORAGE. (OVERLAYS INITVEL/MIDGIM) (24 |
| VPRED | CSM | eq BETA12 +2 | module-private (guid-entry; also io-nouns, sys-misc) | REENTRY, RETURN TO EARTH COMMON DISPLAY (4D) |
| VRATEDIF | LM | eq OMEGAV | module-private (dap-lm) | AXIS TRANSFORMATION MATRIX -- GIMBAL TO PILOT AXES |
| VRECT | CSM+LM | eq RRECT +6 | module-private (nav-integ; also nav-conic, prog-rendz) | STATE VECTORS FOR ORBIT INTEGRATION. (44D) |
| VRECT1 | CSM | eq 24D | module-private (unreferenced in code; overlay alias or spare) | STATE VECTORS FOR ORBIT INTEGRATION. (44D) |
| VRECTCSM | CSM+LM | 6 | `agc_erase_vrectcsm()` / `agc_erase_set_vrectcsm()` | PERMANENT STATE VECTORS AND TIMES. (101D) |
| VRECTLEM | CSM+LM | 6 | `agc_erase_vrectlem()` / `agc_erase_set_vrectlem()` | PERMANENT STATE VECTORS AND TIMES. (101D) |
| VREF | CSM | eq RDOTREF +2 | module-private (guid-entry) | STANDBY VERB ERASABLES. REDOCTR BEFORE THETAD (DWN |
| VRPREV | CSM | eq VG +6 | module-private (guid-powered) | S40.9 STORAGE (16D) |
| VS1 | CSM | eq COSG/2 +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| VSELECT | LM | eq LRADRET +1 | module-private (guid-powered; also sys-misc) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| VSELECT* | LM | eq BUF +1 | module-private (unreferenced in code; overlay alias or spare) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| VSQ/4API | CSM | eq XD/360 +1 | module-private (guid-entry) | THE FOLLOWING FEW REGISTERS USED ONCE EACH 2 SEC |
| VSQUARE | CSM | eq FACT2 +2 | module-private (guid-entry) | REENTRY ERASABLES. (206D) |
| VSUBC | LM | eq LOSVEL | module-private (prog-rendz) | LPS20.1 STORAGE -- ALL ARE PRM -- (9D) |
| VSUN | CSM+LM | eq VEARTH +6 | module-private (prog-align; also nav-aux) | OVERLAYS WITHIN ALIGNMENT/SYSTEST COMMON STORAGE ( |
| VTARGET | CSM+LM | eq VTARGTAG +1 | module-private (nav-conic; also prog-rendz) | CONICS ROUTINE STORAGE. (87D) |
| VTARGTAG | CSM+LM | eq UN +6 | module-private (nav-conic; also prog-rendz) | CONICS ROUTINE STORAGE. (87D) |
| VTEMP | CSM | eq MARKTIME +2 | module-private (prog-rendz) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| VTIG | CSM+LM | eq RTIG +6 | module-private (guid-powered; also prog-rendz) | P30, P40 INTERFACE. (20D) |
| VTPRIME | CSM+LM | eq VIPRIME +6 | module-private (prog-rendz) | INITVEL/MIDGIM STORAGE. (34D) |
| VVARMIN | LM | 1 | module-private (prog-rendz) | PADLOADED ERASABLES FOR P20/P22 (6D) |
| VVEC | CSM+LM | eq R1A +2 | module-private (prog-rendz; also nav-conic, guid-entry) | CONICS ROUTINE STORAGE. (87D) |
| VVECT | LM | eq VHZ +1 | module-private (guid-landing) | LANDING ANALOGS DISPLAY STORAGE. (40D) |
| VXBEAMNB | LM | eq VYBEAMNB +6 | module-private (guid-powered) | CAN OVERLAY GSAV WITH CARE, USED DURING POWERED DE |
| VXRCM | LM | eq RLMSRCH +6 | module-private (prog-rendz) | LRS24.1 STORAGE. (CAN SHARE WITH P30'S) (40D) |
| VYBEAMNB | LM | eq VZBEAMNB +6 | module-private (guid-powered) | CAN OVERLAY GSAV WITH CARE, USED DURING POWERED DE |
| VZBEAMNB | LM | eq GSAV | module-private (guid-powered) | CAN OVERLAY GSAV WITH CARE, USED DURING POWERED DE |
| VZC | CSM | eq RZC +6 | module-private (prog-rendz) | S22.1 STORAGE. (36D) |
| W | CSM+LM | eq  | module-private (nav-aux; also nav-integ, prog-rendz, sys-misc) | W-MATRIX STORAGE. (162D) |
| W.IND | LM | eq PIPAGE | module-private (prog-rendz) | INCORPORATION UNSWITCHED (2D) |
| W.INDI | LM | eq W.IND +1 | module-private (unreferenced in code; overlay alias or spare) | INCORPORATION UNSWITCHED (2D) |
| WAITADR | CSM+LM | eq ITEMP3 | module-private (exec-sched) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| WAITBANK | CSM+LM | eq ITEMP2 | module-private (exec-sched) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| WAITEXIT | CSM+LM | eq ITEMP1 | module-private (exec-sched; also sys-misc) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| WAITTEMP | CSM+LM | eq ITEMP4 | module-private (exec-sched) | INTERRUPT TEMPORARY STORAGE POOL. (11D) |
| WANGI | CSM+LM | eq AINLA +2D | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| WANGO | CSM+LM | eq AINLA | module-private (imu; also sys-misc) | SYSTEM TEST STORAGE. (174) |
| WANGT | CSM+LM | eq AINLA +4D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| WBODY | CSM | eq TVCRCS | module-private (dap-csm; also imu, prog-rendz, sys-misc) | OVERLAYS WITHIN TVC DAP |
| WBODY1 | CSM | eq WBODY +2 | module-private (dap-csm; also imu, prog-rendz) | OVERLAYS WITHIN TVC DAP |
| WBODY2 | CSM | eq WBODY +4 | module-private (dap-csm; also imu, prog-rendz) | OVERLAYS WITHIN TVC DAP |
| WCENTRAL | LM | eq GTSTEMPS +4 | module-private (dap-lm) | TRIM GIMBAL CONTROL LAW ERASABLES: (11D) |
| WCHPHASE | LM | 1 | module-private (guid-landing; also guid-powered) | P63, P64, P65, P66, AND P67. (1D) |
| WCHPHOLD | LM | eq OURPERMS | module-private (guid-landing; also guid-powered) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| WCHVERT | LM | eq ELVDUMMY | module-private (guid-landing) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| WDCNT | CSM+LM | eq DOTRET | module-private (io-pinball) | THE REGISTERS BETWEEN ADDRWD AND PRIORITY MUST STA |
| WDRET | CSM+LM | eq INTBIT15 | module-private (io-pinball) | UNUSED ERASABLES ********(2) |
| WEIGHT/G | CSM+LM | 1 | module-private (guid-powered; also dap-csm, guid-entry) | R22 PAD LOADS (5D). |
| WHCHREAD | LM | eq R65CNTR | module-private (io-nouns; also prog-rendz) | R61LEM -- PREFERRED TRACKING ATTITUDE ROUTINE **IN |
| WHICH | LM | 1 | module-private (guid-landing; also guid-powered, exec-sched, prog-rendz) | P30-P40 INTERFACE UNSHARED. (3D) |
| WHOCARES | CSM+LM | eq  | module-private (exec-sched; also guid-powered, io-pinball, prog-rendz, prog-align) | P21 STORAGE (19D) |
| WIXA | CSM+LM | eq USTAR +6 | module-private (nav-aux) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| WIXB | CSM+LM | eq WIXA +1 | module-private (nav-aux) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| WM | LM | eq G(CSM) +6 | module-private (guid-landing; also guid-powered) | SERVICER FOR LUNAR ASCENT AND DESCENT (14D) |
| WMIDPOS | CSM | 1 | module-private (prog-rendz) | P23 PAD LOADS *** (2D). |
| WMIDVEL | CSM | 1 | module-private (prog-rendz) | P23 PAD LOADS *** (2D). |
| WORBPOS | CSM | 1 | module-private (prog-rendz) | P22 STORAGE. -- PAD LOADED -- (5D) |
| WORBVEL | CSM | 1 | module-private (prog-rendz) | P22 STORAGE. -- PAD LOADED -- (5D) |
| WPLATI | CSM+LM | eq AINLA +84D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| WPLATO | CSM+LM | eq AINLA +24D | module-private (unreferenced in code; overlay alias or spare) | SYSTEM TEST STORAGE. (174) |
| WRDRET | CSM+LM | eq INTBIT15 | module-private (unreferenced in code; overlay alias or spare) | UNUSED ERASABLES ********(2) |
| WRENDPOS | CSM+LM | 1 | module-private (prog-rendz) | P20 STORAGE. -- PAD LOADED -- (4D) |
| WRENDVEL | CSM+LM | 1 | module-private (prog-rendz) | P20 STORAGE. -- PAD LOADED -- (4D) |
| WSHAFT | LM | 1 | module-private (prog-rendz) | P20 STORAGE. -- PAD LOADED -- (6D) |
| WSURFPOS | LM | 1 | module-private (prog-rendz) | LUNAR SURFACE NAVIGATION (2D) |
| WSURFVEL | LM | 1 | module-private (prog-rendz) | LUNAR SURFACE NAVIGATION (2D) |
| WTEMP | CSM | eq XNDX1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| WTOPTION | CSM | 1 | module-private (exec-sched; also sys-misc) | MODE SWITCHING ERASABLE (14D) |
| WTRUN | LM | 1 | module-private (prog-rendz) | P20 STORAGE. -- PAD LOADED -- (6D) |
| WWBIAS | LM | eq RTHETA | module-private (io-nouns; also prog-rendz) | VERB 67 STORAGE |
| WWPOS | LM | eq RANGE | module-private (io-nouns; also prog-rendz, sys-misc) | VERB 67 STORAGE |
| WWVEL | LM | eq RRATE | module-private (io-nouns; also prog-rendz) | VERB 67 STORAGE |
| X(T1) | CSM | eq PCON +2 | module-private (unreferenced in code; overlay alias or spare) | RETURN-TO-EARTH STORAGE. (93D) |
| X(T2) | CSM | eq DELTAT | module-private (unreferenced in code; overlay alias or spare) | OVERLAYS WITHIN RETURN-TO-EARTH STORAGE. |
| X(T2)PRE | CSM | eq T12 | module-private (unreferenced in code; overlay alias or spare) | OVERLAYS WITHIN RETURN-TO-EARTH STORAGE. |
| X1INPUT | LM | eq DELDV | module-private (unreferenced in code; overlay alias or spare) | INITVEL STORAGE. (IN OVERLAY 0 AND OVERLAY 1. (2D) |
| X789 | CSM+LM | 6 | module-private (nav-aux; also prog-rendz, sys-misc) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| XD/360 | CSM | eq LCX/360 +1 | module-private (guid-entry) | THE FOLLOWING FEW REGISTERS USED ONCE EACH 2 SEC |
| XDC | CSM+LM | eq ZSM +6 | module-private (imu; also prog-align, guid-entry) | ALIGNMENT/SYSTEST/CALCSMSC/CRS61.1 COMMON STORAGE  |
| XKEP | CSM+LM | eq TC +2 | module-private (nav-integ) | STATE VECTORS FOR ORBIT INTEGRATION. (44D) |
| XKEPCSM | CSM+LM | 2 | module-private (unreferenced in code; overlay alias or spare) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| XKEPLEM | CSM+LM | 2 | module-private (unreferenced in code; overlay alias or spare) | PERMANENT STATE VECTORS AND TIMES. (101D) |
| XKEPNEW | CSM+LM | eq RPSV +6 | module-private (nav-conic; also nav-integ) | INTEGRATION STORAGE. (95D) |
| XMODULO | CSM | 2 | module-private (unreferenced in code; overlay alias or spare) | KEPLER STORAGE (6D) |
| XNB | CSM+LM | eq XDC | module-private (imu; also prog-align, sys-misc, guid-entry, guid-powered) | ALIGNMENT/SYSTEST/CALCSMSC/CRS61.1 COMMON STORAGE  |
| XNB1 | CSM | eq W +108D | module-private (prog-align; also sys-misc) | AUTO-OPTICS STORAGE -R52- |
| XNBPIP | LM | eq VECTAB +12D | module-private (guid-landing; also guid-powered) | SERVICER STORAGE (USED BY ALL POWERED FLIGHT PROGS |
| XNDX1 | CSM | eq KMPTEMP +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| XNDX2 | CSM | eq XNDX1 +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| XOLDBUF | CSM | eq ZPIPBUF +1 | module-private (guid-powered) | ENTRY RESTART PROTECTION STORAGE. --KEEP TEMPS IN  |
| XPIPBUF | CSM | eq ADOT | module-private (guid-powered) | ENTRY RESTART PROTECTION STORAGE. --KEEP TEMPS IN  |
| XPREV | CSM+LM | eq XKEP | module-private (nav-conic) | CONICS ROUTINE STORAGE. (87D) |
| XRANGE | LM | eq 1/DV3 +2 | module-private (guid-landing; also io-nouns) | ASCENT GUIDANCE ERASABLES. (21D) |
| XREG | CSM+LM | 1 | module-private (io-nouns; also io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| XREGLP | CSM+LM | 1 | module-private (io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| XRNGERR | CSM | eq LATANG | module-private (io-nouns) | REENTRY ERASABLES. (206D) |
| XRS | CSM | eq DELTEE +2 | module-private (prog-rendz) | TPI SEARCH (S17.1, S17.2) P17 STORAGE.(10D) |
| XSCD | CSM | eq END-SVCR | module-private (unreferenced in code; overlay alias or spare) | ALIGNMENT STORAGE. (25D) |
| XSCI | LM | eq STARAD | module-private (unreferenced in code; overlay alias or spare) | P50'S, R50'S Q STORES (2D) |
| XSCREF | CSM+LM | eq XSMD | module-private (guid-powered) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| XSM | CSM+LM | eq END-W +23D | module-private (sys-misc; also imu, prog-align, guid-entry, exec-sched) | ALIGNMENT/SYSTEST/CALCSMSC/CRS61.1 COMMON STORAGE  |
| XSMD | CSM+LM | eq UPBUFF +2 | module-private (prog-align) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| XTRANS | CSM | eq ACORBD +1 | module-private (dap-csm; also guid-powered) | OVERLAYS WITHIN TVC DAP |
| XXXALT | CSM+LM | eq SECMAX +2 | module-private (prog-rendz) | RENDEZVOUS GUIDANCE STORAGE. -- P32 ... P35 -- (8D |
| XYMARK | LM | eq MARKCNTR +1 | module-private (prog-align) | AOTMARK STORAGE (3D) |
| Y | LM | eq /LAND/ +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| YACTOFF | CSM | 1 | module-private (dap-csm; also guid-powered, io-nouns) | R22 PAD LOADS (5D). |
| YACTTMP | CSM | eq PACTTMP +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| YAW | LM | eq ATP +2 | module-private (guid-landing; also io-nouns) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| YAWANG | LM | eq BETASB | module-private (prog-align) | S-BAND ANTENNA GIMBAL ANGLES. DISPLAYED BY R05 (V6 |
| YCMD | CSM | eq PCMD +1 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| YCO | LM | eq RCO +2 | module-private (guid-landing) | ASCENT GUIDANCE ERASABLES. (21D) |
| YCONS | LM | eq PCONS +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| YDC | CSM+LM | eq XDC +6 | module-private (imu; also prog-align, guid-entry) | ALIGNMENT/SYSTEST/CALCSMSC/CRS61.1 COMMON STORAGE  |
| YDELOFF | CSM | eq PDELOFF +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| YDOT | LM | eq LAXIS +6 | module-private (guid-landing; also io-nouns) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| YDOTD | LM | eq RDOTD +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| YERRB | CSM | eq PERRB +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| YINDEX | CSM | eq PINDEX +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| YLIM | LM | eq VMIN +2 | module-private (guid-landing) | W-MATRIX. ESSENTIALLY UNSHARABLE. (162D) |
| YMANNDX | CSM | eq PMANNDX +1 | module-private (dap-csm; also guid-entry) | OVERLAYS WITHIN TVC DAP |
| YNB | CSM+LM | eq YDC | module-private (imu; also prog-align, guid-entry, sys-misc) | ALIGNMENT/SYSTEST/CALCSMSC/CRS61.1 COMMON STORAGE  |
| YNB1 | CSM | eq XNB1 +6 | module-private (prog-align) | AUTO-OPTICS STORAGE -R52- |
| YNBPIP | LM | eq XNBPIP +6 | module-private (guid-landing) | SERVICER STORAGE (USED BY ALL POWERED FLIGHT PROGS |
| YNBSAV | LM | eq GSAV +6 | module-private (prog-align; also sys-misc) | ALIGNMENT PLANETARY -- INERTIAL TRANSFORMATION STO |
| YNDX | CSM | eq XNDX2 +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| YOLDBUF | CSM | eq XOLDBUF +1 | module-private (guid-powered) | ENTRY RESTART PROTECTION STORAGE. --KEEP TEMPS IN  |
| YPIPBUF | CSM | eq XPIPBUF +1 | module-private (guid-powered) | ENTRY RESTART PROTECTION STORAGE. --KEEP TEMPS IN  |
| YRATE | LM | eq PRATE +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| YREG | CSM+LM | 1 | module-private (io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| YREGLP | CSM+LM | 1 | module-private (io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| YSCD | CSM | eq XSCD +6 | module-private (unreferenced in code; overlay alias or spare) | ALIGNMENT STORAGE. (25D) |
| YSCI | LM | eq XSCI +6 | module-private (unreferenced in code; overlay alias or spare) | P50'S, R50'S Q STORES (2D) |
| YSCREF | CSM+LM | eq YSMD | module-private (guid-powered) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| YSM | CSM+LM | eq XSM +6 | module-private (imu; also guid-entry) | ALIGNMENT/SYSTEST/CALCSMSC/CRS61.1 COMMON STORAGE  |
| YSMD | CSM+LM | eq XSMD +6 | module-private (prog-align) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| YTMP1 | CSM | eq PTMP6 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| YTMP2 | CSM | eq YTMP1 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| YTMP3 | CSM | eq YTMP2 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| YTMP4 | CSM | eq YTMP3 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| YTMP5 | CSM | eq YTMP4 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| YTMP6 | CSM | eq YTMP5 +2 | module-private (dap-csm) | INTEMP THRU INTEMP+14D ARE RESERVED FOR OVERLAYED  |
| YV | CSM+LM | eq ALT +2 | module-private (nav-integ) | UNSWITCHED FOR ORBIT INTEGRATION (21D) |
| YWORD1 | CSM | eq PWORD2 +1 | module-private (dap-csm) | R22 PAD LOADS (5D). |
| YWORD2 | CSM | eq YWORD1 +1 | module-private (dap-csm) | R22 PAD LOADS (5D). |
| YZERO | CSM | eq CMDTMP | module-private (unreferenced in code; overlay alias or spare) | OVERLAYS WITHIN TVC DAP |
| ZAXIS1 | LM | eq UHZP | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| ZDC | CSM+LM | eq YDC +6 | module-private (imu; also prog-align, guid-entry) | ALIGNMENT/SYSTEST/CALCSMSC/CRS61.1 COMMON STORAGE  |
| ZDOT | LM | eq YDOT +2 | module-private (guid-landing) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| ZDOTD | LM | eq YDOTD +2 | module-private (guid-landing; also io-nouns) | ASCENT GUIDANCE FOR LUNAR LANDING (54D) |
| ZERDUMMY | LM | eq AZIDUMMY +1 | module-private (unreferenced in code; overlay alias or spare) | THESE ERASABLES SHOULD NOT BE SHARED DURING P63, P |
| ZERLINA | LM | eq DTHETASM | module-private (guid-landing) | ERASABLES FOR P64: OVERLAY OF DTHETASM, WHICH IS U |
| ZERONDX | CSM+LM | eq AINLA +137D | module-private (sys-misc; also imu) | SYSTEM TEST STORAGE. (174) |
| ZERONDX1 | CSM | eq AINLA +137D | module-private (imu) | SYSTEM TEST STORAGE. (174) |
| ZI | CSM+LM | eq TDVEL +6 | module-private (nav-aux) | MEASUREMENT INCORPORATION STORAGE. (66D) |
| ZIXA | CSM+LM | eq WIXB +1 | module-private (nav-aux) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| ZIXB | CSM+LM | eq ZIXA +1 | module-private (nav-aux) | RENDEZVOUS NAVIGATION STORAGE. (SEE COMMENT IN SER |
| ZNB | CSM+LM | eq ZDC | module-private (imu; also prog-align, guid-entry, sys-misc) | ALIGNMENT/SYSTEST/CALCSMSC/CRS61.1 COMMON STORAGE  |
| ZNB1 | CSM | eq YNB1 +6 | module-private (prog-align) | AUTO-OPTICS STORAGE -R52- |
| ZNBPIP | LM | eq YNBPIP +6 | module-private (guid-landing) | SERVICER STORAGE (USED BY ALL POWERED FLIGHT PROGS |
| ZNBSAV | LM | eq YNBSAV +6 | module-private (prog-align) | ALIGNMENT PLANETARY -- INERTIAL TRANSFORMATION STO |
| ZNDX | CSM | eq YNDX +1 | module-private (dap-csm) | OVERLAYS WITHIN TVC DAP |
| ZOLDBUF | CSM | eq YOLDBUF +1 | module-private (guid-powered) | ENTRY RESTART PROTECTION STORAGE. --KEEP TEMPS IN  |
| ZONE | CSM | 1 | module-private (sys-misc) | T4RUPT ERASABLE (6D) |
| ZONE3LIM | LM | eq BLOCKTOP +7 | module-private (dap-lm) | INPUT TO TJET LAW (PERMANENT ERASABLES). (48D) |
| ZOOMTIME | LM | 1 | module-private (guid-landing; also exec-sched, guid-powered) | THROTTLE STORAGE. -- PAD LOADED -- (1D) |
| ZOPTCNT | CSM | 1 | module-private (sys-misc) | MODE SWITCHING ERASABLE (14D) |
| ZPIPBUF | CSM | eq YPIPBUF +1 | module-private (guid-powered) | ENTRY RESTART PROTECTION STORAGE. --KEEP TEMPS IN  |
| ZPRIME | CSM+LM | eq 22D | module-private (imu) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| ZREG | CSM+LM | 1 | module-private (io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| ZREGLP | CSM+LM | 1 | module-private (io-pinball) | PINBALL EXECUTIVE ACTION (44D) |
| ZSCD | CSM | eq YSCD +6 | module-private (unreferenced in code; overlay alias or spare) | ALIGNMENT STORAGE. (25D) |
| ZSCI | LM | eq YSCI | module-private (unreferenced in code; overlay alias or spare) | P50'S, R50'S Q STORES (2D) |
| ZSCREF | CSM+LM | eq ZSMD | module-private (guid-powered) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| ZSM | CSM+LM | eq YSM +6 | module-private (imu; also guid-entry) | ALIGNMENT/SYSTEST/CALCSMSC/CRS61.1 COMMON STORAGE  |
| ZSMD | CSM+LM | eq YSMD +6 | module-private (prog-align) | ALIGNMENT/S40.2,3 COMMON STORAGE. (18D) |
| ZV | CSM+LM | eq YV +6 | module-private (nav-integ) | UNSWITCHED FOR ORBIT INTEGRATION (21D) |
