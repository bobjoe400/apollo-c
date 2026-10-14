# state-flags -- lane history

Process diary of the state-flags lane, moved verbatim out of [docs/modules/state-flags.md](../../docs/modules/state-flags.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-14 (first and last commit touching the note).

## Progress

- DONE agc_flags_common.h (83 flags with identical name and position in CSM and LM)
- DONE agc_flags_csm.h (CSM-only and position-differing flags; FLAGWRD0..11)
- DONE agc_flags_lm.h (LM-only and position-differing flags; FLAGWRD0..13)
- DONE agc_channels.h: `AGC_CH<octal>_<NAME>` bit constants for channels 5,6,7,10-16,30-33
- DONE agc_channels.c (15-bit image, read/write/or/and, sim input side, inverted 030-033)
- DONE agc_state.h / agc_state.c (vehicle, TIME1/2, flagwords, state vectors)
- DONE agc_erasable.c (36 scalar/vector/matrix + 3 array erasables, generated)
- DONE tests: test_state (146 checks after review), test_state_lm (44), both pass, -Werror clean
- DONE lookup table (below). `tools/check_rules.py`: 0 violations, density >= 2.0 in all my files
- DONE MODULE_STATUS row
- Nothing TODO. Generated pieces came from throw-away scripts parsing the AGC text; the
  headers are now the source of truth (edit by hand if the AGC text is re-read).

## Requested core additions

I own the headers below, so these were made directly; other lanes should know:

* `agc_state.h`: `AGC_FLAGWORDS` raised 12 -> 14 (LM has FLGWRD12/13); added
  `AGC_FLAGWORDS_CSM/LM`; `agc_flag_valid/word/bit/mask`; `agc_erase_reset`;
  `AGC_TIME_MODULUS_CS`; `agc_erase_time1/2`; all `agc_erase_*` accessors listed above.
  `agc_time_set_cs` is `void` (status-handling lane); t >= 2^28 is an assertion report and ignored.
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

## Open questions

Resolved in review (docs/modules/review-state-flags.md): CSM channel bits; DELV is PIPA pulses
at 2^14 (5.85 cm/s per pulse CSM, 1 cm/s LM; KPIP1 -> 2^7 m/cs); TEPHEM is TP cs at 2^42
(L "CSEC B-42"); DELVSLV = DELVLVC is one vector (accessor removed, use
`agc_rendz_erasable()->delvlvc`). Promotion of further erasables: other lanes have since added
the accessors they needed to state.h (counters, display, pad loads, restart delays).

