# telemetry -- every downlist sends what the AGC sends

History of this lane: [history/lanes/telemetry.md](../../history/lanes/telemetry.md)

Lane note (agent `telemetry`, 2026-10-14). Assignment: the downlists send
the words the AGC sends, read by address through the erasable map
(structure finding T1: 92 of 128 downlink items were sent as zero).

## Scope

* src/io/telemetry/{downlink_lists_csm,downlink_lists_lm,down_telemetry}.c,
  include/agc/io/service.h (the list entry type).
* src/erasable_map/ (the address map: overlays, the named erasables the
  lists send), include/agc/core/state.h + src/core/ecadr.c (the map's
  per-entry shadow words beside the raw image), owner accessors where an
  owner had none.
* tests/io/service_verbs_telemetry.c, tests/erasable_map/address_map.c,
  tools/oracle (a DOWNRUPT case), sim/nassp/replay.c (a downlink case),
  sim/scenarios (the yaTelemetry check).

## Design

A downlist is the rope's table of DNADR words: count and ECADR.

| AGC | C |
|---|---|
| `nDNADR X +k` (DOWNLINK_LISTS) | `{n, neg, ECADR(X)+k}`; ECADR from the rope's symbol table (yaYUL symtab, both ropes), named by a `#define` per list file |
| `DNPTR L` / `DNCHAN n` | `{AGC_DN_OP_PTR, neg, sublist id}` / `{AGC_DN_OP_CHAN, neg, n}` |
| FETCH2WD `DCA 1400 +ECADR` | `agc_erase_read_ecadr(ecadr)`, `(ecadr+1)` |
| SNAPLOOP `DXCH DNTMBUFF +i` | `agc_erase_write_ecadr(DNTMBUFF+i)` (DNTMBUFF is erasable, as in the AGC) |
| DNDUMP2 | unchanged (already by address) |

The erasable map (src/erasable_map) is the top of the layering, so the
telemetry (io) reading through it is the documented address column, not
an upward call.

### Overlays: one address, several C owners

The AGC reuses erasable between phases (CSM UPBUFF+2.. = XSMD, BESTI =
UPVERB; LM DELVEET1 = TTO.. of ascent; VGU = XSMD +12; MARKDOWN / LATANG /
CSTEER). The C keeps each in its owner's typed storage, and some AGC
erasables have two C copies (MKTIME, AIG/AMG/AOG). The memory word is
whatever was written last. The map keeps, per entry, the typed words it
last saw (a shadow beside the raw image, core's, cleared with it); a read
of an address syncs every entry that holds it: an element whose typed
value changed since the last sync writes its words into the raw image.
So the last writer wins, as in the AGC. A write still goes to the first
entry that holds the address (unchanged behaviour for the NASSP pad loads).

### Items with no C storage

The C has no rope addresses and no BBANK/Q registers: CADRFLSH, CADRMARK,
TEMPFLSH (continuation CADRs) and RSBBQ (BBANK and Q at the last restart)
read the raw image; ARUPT (SPARE +1) likewise. Listed under Deviations.

## Per-list counts (known state, tests/io/downlink_oracle.c)

Data words (99 per list, the ID words excluded) the C sends non-zero, and
words equal to yaAGC's (101 incl. both IDs), before (HEAD da52cdc) and
after. The words still zero after are zero in the AGC too: SPARE
(ERASZERO, ARUPT) and the channels 11-14 (zero in the run).

| List | code | non-zero before | non-zero after | equal before | equal after |
|---|---|---|---|---|---|
| CSM Coast and Align (CMCSTADL) | 0 | 48 | 98 | 38 | 101 |
| CSM Entry and Update (CMENTRDL) | 1 | 33 | 98 | 28 | 101 |
| CSM Rendezvous and Prethrust (CMRENDDL) | 2 | 36 | 98 | 30 | 101 |
| CSM Powered (CMPOWEDL) | 3 | 46 | 98 | 35 | 101 |
| CSM P22 (CMPG22DL) | 4 | 33 | 84 | 42 | 101 |
| LM Coast and Align (LMCSTADL) | 0 | 47 | 98 | 39 | 101 |
| LM AGS Initialization (LMAGSIDL) | 1 | 35 | 95 | 35 | 101 |
| LM Rendezvous and Prethrust (LMRENDDL) | 2 | 35 | 95 | 33 | 101 |
| LM Orbital Maneuvers (LMORBMDL) | 3 | 46 | 96 | 38 | 101 |
| LM Descent and Ascent (LMDSASDL) | 4 | 37 | 97 | 31 | 101 |
| LM Lunar Surface Align (LMLSALDL) | 5 | 40 | 96 | 35 | 101 |

The known-state test loads every named word by address, so it proves the
list structure, the addresses, the snapshot and word-order logic against the
rope; that the words follow the C program's typed values is
tests/erasable_map/address_map.c (a value an owner stores shows at its
address; overlays; stores by address reach the owner).

On the NASSP mission images (make nassp-replay; of 99 data words; "DP" =
a DP word within 2 LSB, "known" = the TIME word and the DOWNLIST_SKIP words):

| Image | list | first session (vs stock resumed) | now (vs stock GOJAM'd at the load) |
|---|---|---|---|
| insertion | CSM CMCSTADL 77777 | 68 -> 89 equal | 91 equal, 5 DP, 3 known |
| tli | CSM CMCSTADL 77777 | 61 -> 88 equal | 91 equal, 6 DP, 2 known |
| mcc2 | CSM CMCSTADL 77777 | 57 -> 81 equal | 83 equal, 9 DP, 7 known |
| pdi_lm | LM LMDSASDL 77773 | not compared (stock broken; ours sent 77777) | 84 equal, 4 DP, 11 known |
| liftoff_lm | LM LMDSASDL 77773 | not compared (stock broken; ours sent 77777) | 85 equal, 5 DP, 9 known |

Scenarios: imu_realign_csm (P52) checks every non-zero word of CMCSTADL,
exact where the scenario fixes the value (83 checks); fresh_start_lm
checks LMCSTADL after V36 (ID, PROG 00, channel 11, the non-zero words).
The erasable dump: tests/io/downlink_dump_oracle.c, 1989/1989 (CSM) and
1993/1993 (LM) halves equal yaAGC's DNDUMP.

### DOWNLIST_SKIP (sim/nassp/replay.py), final

| Vehicle, list | words | reason |
|---|---|---|
| CSM 77777 | 67 | RSBBQ: BBANK and Q at the restart (no bank or Q register in the C) |
| CSM 77777 | 68-70 | CADRFLSH, CADRMARK, TEMPFLSH: continuation CADRs (no rope addresses) |
| CSM 77777 | 16, 66, 83 | THETADZ (and ERRORZ from it): the high word of CDUZD as DAS left it, signs disagreeing in the image; the C keeps the DP value (DEVIATION) |
| LM 77773 | 25 | SPARE +1 = ARUPT (no A register) |
| LM 77773 | 31 | RSBBQ |
| LM 77773 | 60-62 | CADRFLSH, CADRMARK, TEMPFLSH |
| LM 77773 | 40, 44-48, 90 | pdi: FLAGWRD4, DSPTAB, VN FLASH of the display a variable restart re-shows (PHASE4 0402, PHSNAME4 = a rope 2CADR the C cannot restart); liftoff: the noun 74 digits follow TTOGO, COMP ACTY |
| LM 77773 | 63 | RADMODES bit 6: the shim keeps the image's LR position (flight bug 5); a GOJAM'd AGC re-reads it from channel 33 |
| LM 77773 | 79 | LEMMASS = the major word of MASS as stored (signs disagree in the image); the C keeps MASS as a value (DEVIATION) |
| LM 77773 | 99 | OPEN: TTOGO 0.4-0.8 s behind stock after the load restart |

## Deviations

* Overlays: a read shows the item whose typed value changed last since
  the map last looked. An owner that stores the value it already had,
  after another owner wrote the address, is not seen (the C cannot tell
  a store of the same value); two owners changed between two reads: the
  first map entry wins. A store by address goes to the first entry only
  (unchanged; the NASSP pad loads rely on it).
* Words with no C storage read the raw image (what a load or V21 put
  there): CADRFLSH, CADRMARK, TEMPFLSH (continuation CADRs; the C has no
  rope addresses), RSBBQ (BBANK and Q at the last restart), ARUPT (SPARE
  +1: the interrupted A register).
* Value items whose owner has no setter (UPBUFF block, REDOCTR, VHFTIME,
  LAUNCHAZ, 8NN, SVMRKDAT, R-OTHER, V-OTHER, PREL..RREL, PAXERR1,
  LASTYCMD, the radar words, AGSK, AGSBUFF, OMEGAP.., DOWNTORK, the R29
  and mark words): a store by address keeps the word at the address, not
  in the owner, until the owner writes again.
* TTF/8 is sent as its DP value's sign-agreeing words, not the words DAS
  left (agc_lm_ldg()->ttf_8_hi); CMDAPMOD -0 is the word 77777, and a
  store of any other negative word reads as -1, positive as +1 (the C
  keeps an enum).
* OGARATE (= OMEGAB, DP in the AGC) is an SP in the C TVC; the address
  shows OMEGAB's words (agc_dp_words_t).
* A DP the AGC left with disagreeing signs (DAS) and later reads by its
  high word alone: the C keeps the value, so the high word is the
  sign-agreeing one, 1 LSB off: LEMMASS from MASS (1/ACCS), THETADX..Z
  from CDUXD..ZD (DCDUINCR). Seen on the NASSP images; owners' types
  (agc_dp_t), not the telemetry.
* The map re-forms a DP's stored words (sign-agreeing) when a later store
  touches a coupled owner (AZO / -AYO / AXO pad load): the dump oracle
  leaves those words out (32 CSM, 28 LM of ~1950).
* OPTIND -0 is the C's AGC_OPTIND_NEG0 (-2); the map turns it into 77777
  and back.

## Stubs

none

## Tests written

* tests/io/downlink_oracle.c (new): the 11 lists word for word against
  tools/oracle/cases/downlink_{csm,lm}.txt, 6186 checks.
* tests/erasable_map/address_map.c: test_map_size,
  test_downlink_fields_csm, test_downlink_entry_dap_csm,
  test_downlink_fields_lm (366 checks).
* tools/oracle: agc_probe `downlink N`; gen/gen_downlink.py.
* sim/nassp/replay.{c,py}: the downlist case; sim/sim_client.py
  `downlist`; sim/scenarios/imu_realign_csm.txt (5 downlist checks).
* second session: tests/io/downlink_dump_oracle.c (new, DNDUMP vs yaAGC,
  both ropes; tools/oracle/cases/gen/gen_dump.py, dndump_{csm,lm});
  tests/erasable_map/address_map.c test_load_words_csm/_lm (401 checks);
  replay: both LGC images' LMDSASDL against stock, stock GOJAM reference;
  sim_client `downlist word N HI LO`, `downlist dump`; imu_realign_csm
  (19 downlist checks, 83 in all), fresh_start_lm (LMCSTADL).

## Program flow and decisions

_Final documentation pass (2026-10-14), from
`src/io/telemetry/down_telemetry.c`, the two list files and
`src/erasable_map/address_map.c` at HEAD 3b97dc8. The DOWNRUPT
decisions are also in [sys-misc](sys-misc.md#program-flow-and-decisions)._

* **DOWNRUPT** (`down_telemetry.c:agc_telem_downrupt`, every 20 ms under
  agc_sim, when NASSP's PCM asks under NASSP): the word-order bit (channel
  13 bit 7) is set every time on the CSM and only when it is 0 on the LM
  (WOTEST), then the state DNTMGOTO decides: PHASE1 starts the list
  (`dnphase1` -> `newlist`: the control list of DNLSTCOD, its ID word
  sent first); PHASE2 sends the next DP word: a DNADR still in progress
  (`fetch2wd`), the next sublist entry (NEXTINSL) or control-list entry
  (NEXTINCL), and at the end of the control list a new list (CHKLIST);
  the erasable dump states (V74) step DUMPLOC one DP word at a time,
  send each EBANK's ID (`dndump_id`), and after DUMPCNT complete dumps go
  back to the downlist (`dnphase1`). An unknown state is BAD_ARG.
* **An entry** (`do_entry`): the TIME2 entry of the control list resets
  the word-order bit (SETWO); a DNPTR names a sublist (`list_lookup`;
  a snapshot sublist is copied into DNTMBUFF first by `snapshot`,
  SNAPLOOP); a DNCHAN sends two channels; anything else is a DNADR: the
  count and the ECADR, read two words at a time by address (`word_at`,
  `fetch2wd` -> `agc_erase_read_ecadr`).
* **The lists** (`downlink_lists_csm.c`, `downlink_lists_lm.c`): the
  rope's DNADR tables as `{count, neg, ECADR}` rows, the addresses from
  the yaYUL symbol tables. What each word holds is decided by the address
  map, not here: an overlaid address shows the owner that changed last
  (`sync_holders`); words with no C storage (CADRFLSH, CADRMARK,
  TEMPFLSH, RSBBQ, ARUPT) show the raw image (Deviations).
* **List selection**: `agc_telem_select_list` (DNLSTCOD, set by the
  programs' NEWMODE and by V74's dump request `agc_telem_start_dump`);
  a fresh start, SLAP1 and a restart reset it (`agc_telem_fresh_start`,
  `agc_telem_slap1`, `agc_telem_restart`; GOPROG takes DNLSTCOD from
  AGSWORD on the LM).
