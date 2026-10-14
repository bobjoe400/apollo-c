# telemetry -- every downlist sends what the AGC sends

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

## Progress

* DONE core shadow store: src/core/ecadr.c agc_erase_shadow_read/_write,
  AGC_EC_SHADOW_WORDS (include/agc/core/state.h), cleared with the image
* DONE address_map.c: overlay sync (sync_holders / sync_entry / note_store,
  first_holder for stores), element compare by bits against the shadow;
  enum, layouts and word helpers to src/erasable_map/internal.h;
  agc_erase_map_words() (address_map.h); a duplicated (dead) block in
  item_store removed
* DONE address_map_fields.c: 89 field items (EC_F_*), 52 CSM and 56 LM map
  entries (some overlays: WBODY/OMEGAC, ADOT/OMEGAB, DELVEET3/DELVSIN,
  TCSI/DELVTPF, BESTI/UPVERB, AIG..AOG of R29 and of the LR mark, three
  MKTIME owners)
* DONE owner accessors (read side; see Requested core additions)
* DONE LR downlink copies: LRVTIMDL, LRXCDUDL, LRYCDUDL, LRZCDUDL now
  kept (agc_guid_lm_t lrvtimdl, lrcdudl[3]; landing_radar.c
  agc_job_lrvjob_done, SERVICER p.892: was a DEVIATION "not kept")
* DONE list tables as ECADRs, both ropes (every entry transformed from the
  reviewed item/offset form; 81 CSM and 90 LM operands, addresses from
  the yaYUL symbol tables via agc_probe `whatis`)
* DONE down_telemetry.c: FETCH2WD / SNAPLOOP / DNDUMP2 / the TIME1 word by
  address; DNTMBUFF is erasable (0340); item_word & co. removed
* DONE oracle: agc_probe `downlink N` (DOWNRUPT-only run, LOAD/CHAN/DNLINK
  lines) and symbol names with a digit or '-' (8NN, R-OTHER); cases
  downlink_{csm,lm}.{spec,txt} from gen/gen_downlink.py
* DONE tests/io/downlink_oracle.c: all 11 lists 101/101 words equal
  yaAGC (TIME1 aside); tests/erasable_map/address_map.c: typed owners,
  overlays, stores, map size
* DONE nassp-replay downlink case: sim/nassp/replay.c raises DOWNRUPT 50/s
  as NASSP's PCM does and keeps channels 34/35 from ChannelOutput; replay.py
  check_downlist compares the last complete list word by word (CSM images;
  the LM images have no stock reference in this harness)
* DONE scenario check: sim_client.py decodes the downlink (`downlist id`,
  `downlist nonzero`); imu_realign_csm.txt checks CMCSTADL after P52
* DONE gates (see Tests written)

Resumed 2026-10-14 (second session):

* DONE replay stock LM fix: sim/nassp/replay.c handles channel 7 (the
  superbank) in ChannelOutput as NASSP's apolloguidance.cpp does; this
  harness's yaAGC had run Luminary tasks of banks 30-37 in 40-47 (the
  "TC trap" restarts). Stock now resumes the LGC images.
* DONE replay LM downlist: taken 40 s after the load (the resumed
  program's list, LMDSASDL on both images); the reference is stock run
  again with a GOJAM at the load ('gojam' arg, yaAGC ParityFail path), so
  a restart's DAP, counters and channels compare. DOWNLIST_SKIP keyed by
  (vehicle, ID).
* DONE map: DNLSTCOD (both, 0332), AGSWORD (LM 1324): the LGC images
  sent coast and align instead of descent/ascent (GOPROG DNLSTCOD =
  AGSWORD read the C's 0); REDOCTR setter; CSM optics monitor (OPTIND,
  ZONE, SWSAMPLE..ZOPTCNT, DESOPTT/S: the C re-zeroed the optics after a
  load, OPTMODES bit 10); GCOMP, 1/PIPADT (both: the LM torqued gyros
  after a load, channel 14 bit 6); LM OMEGAP..ALPHAR, DOWNTORK setters
  (the image's rates stayed in memory, the owner's equal 0 never seen)
* DONE shim: src/nassp/engine.c reg_set takes the image's PIPA counts at
  a load (they read 0)
* DONE map: DUMPCNT (0333, with DNLSTCOD) so a load or V21 sets the dump count
* DONE tests/erasable_map/address_map.c test_load_words_csm / _lm (401 checks)
* DONE DNDUMP oracle: tools/oracle/cases/gen/gen_dump.py, dndump_{csm,lm};
  tests/io/downlink_dump_oracle.c: 1989/1989 and 1993/1993 halves equal
* DONE scenario imu_realign_csm: sim_client `downlist word N HI LO` /
  `downlist dump`; every non-zero item checked, exact where the scenario
  fixes the value (stars, marks, REFSMMAT, PROG 00, REDOCTR): 83 checks
* DONE fresh_start_lm: LMCSTADL after V36 (ID, PROG 00, channel 11, non-zero words)
* DONE gates 2026-10-14: make, make test (0 failures, 14 scenarios), make
  check (0 hard, 0 style), prolog 0, always-ok 0, nassp-replay 5/5,
  depgraph no upward call, lm-sim 34/34

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

## Requested core additions

Made in this lane (assignment: "add it to the map, with its owner
accessor"):

* include/agc/core/state.h: AGC_EC_SHADOW_WORDS, agc_erase_shadow_read,
  agc_erase_shadow_write (src/core/ecadr.c; cleared by agc_erase_reset_ecadr)
* include/agc/erasable_map/address_map.h: agc_erase_map_words
* include/agc/io/service.h: agc_dn_entry_t is { op, neg, ref } (ref = the
  ECADR); agc_dn_item_t removed
* owners' read accessors: agc_erase_vhftime, agc_erase_8nn,
  agc_erase_svmrkdat (programs/rendezvous.h; csm/navigation),
  agc_erase_dnindex, agc_erase_lastycmd, agc_erase_rm, agc_erase_mktime,
  agc_erase_rangrdot (programs/rendezvous.h; lm/radar, lm/navigation),
  agc_erase_paxerr1, agc_erase_prel (csm/entry.h), agc_erase_launchaz
  (imu/imu.h), agc_erase_downtork (lm/dap.h), agc_erase_agsk
  (programs/alignment.h)
* include/agc/guidance/guidance.h agc_guid_lm_t: lrvtimdl, lrcdudl[3]
* second session: agc_erase_dnlstcod, agc_erase_dumpcnt,
  agc_erase_set_dumpcnt (io/service.h; down_telemetry.c);
  agc_restart_set_redoctr (exec/exec.h); agc_optmon_word_t,
  agc_erase_optmon, agc_erase_set_optmon (io/service.h; t4rupt_csm.c);
  agc_imu_pipa_interval (imu/imu.h); agc_erase_set_omegap_k,
  agc_erase_set_downtork (lm/dap.h); map items EC_F_DNLSTCOD (x2),
  EC_F_AGSWORD, EC_F_OPTIND, EC_F_OPTZONE, EC_F_OPTMON, EC_F_DESOPT,
  EC_F_GCOMP, EC_F_PIPADT (src/erasable_map/internal.h)

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

## Findings

* T1 (structure note) resolved: every list word is read by address.
* One AGC erasable, several C copies (each mapped as an overlay; one owner
  would be cleaner): MKTIME (R29 agc_rendz_rr_downlink, LRS22.1
  lmnav_state, LR guidance agc_guid_lm_t), AIG/AMG/AOG (R29, LR mark,
  lmnav_state), TANGNB (rr_state, R29), RM (lmnav_state DP, R29 raw),
  LAUNCHAZ (prelaunch pl, P11 insertion_pad), DELVEET3 / DELVSIN (CSM:
  rz.delveet3 and rz.delvsin, one address in Comanche).
* Overlays the map does not know (not on a downlist path the tests hit,
  but the AGC sends them): KALCMANU's TMIS/KV3 (E6,1666-1711, CSM) over
  ALFA/180 .. PAXERR1 of the entry list; XSCREF (= XSMD = UPBUFF +2, CSM,
  agc_burn_t xscref); TVC ERRBTMP / DELBRTMP over CMDAPMOD..RREL.
* nassp-replay: the shim's load restart leaves the PIPAs at 0 and restarts
  the RCS DAP (ADOT, THETADX, ERRORX differ from stock, which resumes);
  the channel 11-14 image differs. Recorded in replay.py DOWNLIST_SKIP.
* agc_probe resolved names that start with a digit or contain '-' (8NN,
  R-OTHER) to wrong addresses (8NN -> 0, R-OTHER -> R): fixed (whole name
  first, then NAME+n / NAME-n).
* The scenarios take minutes of wall time here with seconds of CPU
  (lockstep round trips; external_delta_v_burn_csm 5 min at HEAD too).
* The LR downlink copies (LRVTIMDL, LRXCDUDL..LRZCDUDL) were a documented
  DEVIATION in landing_radar.c ("not kept"): now kept as SERVICER does.

* Second session (2026-10-14):
  * This harness's stock yaAGC "TC trap" on the LGC images was the
    superbank: NASSP's ChannelOutput keeps channel 7 (apolloguidance.cpp),
    replay.c's did not, so a Luminary task in banks 30-37 ran in 40-47
    (zeros, TC 0 loops). Fixed in replay.c; stock now resumes both LGC
    images, so the LM lists compare.
  * The NASSP image load lost erasables the C owns but the map did not
    hold, and each showed on the downlink: DNLSTCOD / AGSWORD (the LM sent
    coast and align instead of descent/ascent after the load restart),
    the CSM optics monitor (the C re-zeroed the optics: OPTMODES bit 10),
    GCOMP and 1/PIPADT (the LM torqued gyros: channel 14 bit 6), REDOCTR,
    the PIPA counts (the shim skipped them). All mapped / imported now.
  * A value item with no setter keeps a loaded word in memory until its
    owner's value changes; an owner that writes the value it already had
    (the LM DAP zeroing OMEGAP..R after the restart) is never seen, so the
    image's stale rates went out. OMEGAP..ALPHAR and DOWNTORK have setters
    now. Still setter-less: the UPBUFF block, VHFTIME, LAUNCHAZ, 8NN,
    SVMRKDAT, R-OTHER, V-OTHER, PREL..RREL, PAXERR1, LASTYCMD (rendezvous
    files, not touched), the radar words, AGSK, AGSBUFF, the R29 and mark
    words.
  * FOR THE NASSP LANE: a scenario saved during a variable restart phase
    (pdi_lm: PHASE4 = 0402, PHSNAME4 = 36,3032 in P63) cannot be resumed by
    the C's load restart: PHSNAMEn is a rope 2CADR, the C keeps a job id;
    agc_restart_group returns BAD_ARG (start_job with AGC_JOB_NONE) and P63's
    display is not re-shown (stock re-shows it, flashing). A table of
    2CADR -> job id generated from tools/registry.tsv and the yaYUL symtab
    would let the map translate PHSNAMEn.
  * OPEN: TTOGO 0.4-0.8 s behind stock after the load restart on both LGC
    images (the 1 s task that counts it restarts at another phase).
  * Noted, not chased: on liftoff_lm (P12 at TIG-60 s) the resumed stock
    lights KEY REL and stays in P12 after the replay's RSET, V37E00E; ours
    goes to P00 and integrates. replay.py compares only the LGC downlist
    (taken before the keys) and its own checks, as before.

## Open questions

* Should a store by address go to every owner holding the address (the
  AGC's memory is one word)? Kept to the first owner for the NASSP pad
  loads; the read side already follows the last writer.

## Lessons

* A table of addresses is the rope's own form of a downlist; translating
  it to items lost the overlays the AGC relies on. Keep tables of
  addresses as addresses and put the C ownership in one place (the map).
* agc_probe can be an oracle for interrupt-driven code: run the real
  vector with every other interrupt suppressed and watch yaAGC's own
  DOWNRUPT timer (DownruptTimeValid) instead of hooking ChannelOutput.
* With shadows, compare an element by bits against what the map last saw,
  not by value against the raw image: values hide -0 vs +0 (CMDAPMOD).
* To compare a restarted engine with a reference, restart the reference
  the same way (yaAGC: ParityFail = 1 gives a GOJAM), then every word that
  still differs is a real difference, not the restart.
* A stock yaAGC harness must do what the host's callbacks do: NASSP's
  ChannelOutput owns channel 7 (the superbank). Without it Luminary runs
  banks 40-47 code from the wrong place and looks like a TC trap.
* Downlink differences on a loaded image are a good probe of what the
  address map misses: every unmapped owner word the restart reads shows
  up as a downlist word.
