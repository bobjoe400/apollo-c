# telemetry -- lane history

Process diary of the telemetry lane, moved verbatim out of [docs/modules/telemetry.md](../../modules/telemetry.md), which keeps the reader-facing explanation. Note dates: 2026-10-14 to 2026-10-14 (first and last commit touching the note).

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

