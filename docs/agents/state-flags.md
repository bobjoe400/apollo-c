# Assignment: state-flags   (tier: mechanical)

Scope: the shared state layer. Mostly careful transcription.

1. include/agc/agc_flags_csm.h and include/agc/agc_flags_lm.h: every
   flagword bit (FLAGWRD0..FLAGWRD11; Comanche055/ERASABLE_ASSIGNMENTS.agc
   and Luminary099/FLAGWORD_ASSIGNMENTS.agc) as enum values of type
   agc_flag_id_t encoded (word << 4) | bit, each with the AGC name, bit
   number and the AGC one-line meaning as a comment. Flags with the same
   name and position in both vehicles also go in agc_flags_common.h.
2. include/agc/agc_channels.h (you MAY edit it): add named bit constants
   AGC_CH<chan>_<NAME> for every bit in Luminary099/
   INPUT_OUTPUT_CHANNEL_BIT_DESCRIPTIONS.agc and the Comanche channel
   comments in ERASABLE_ASSIGNMENTS.agc. Implement src/core/agc_channels.c
   (15-bit channel image; read/write/or/and; input side for the sim; note
   that channels 030-033 are inverted-logic inputs).
3. include/agc/agc_state.h (you MAY extend it) and src/core/agc_state.c:
   time (TIME1/TIME2 centisecond counter), flagword storage and the flag
   API, state-vector accessors (this vehicle's RN/VN/PIPTIME and the other
   vehicle's set), TIG, MODREG, plus accessors for widely shared erasables
   (REFSMMAT, CDUX/Y/Z, PIPAX/Y/Z, DELV, VGTIG, TTOGO, MARKSTAT, DSPTAB and
   similar). Choose by reading which erasables are referenced across many
   log sections; justify the list in your note. Accessors stay trivial;
   scale factors in comments from the AGC text.
4. docs/modules/state-flags.md: a lookup table AGC erasable name ->
   C accessor or "module-private (owner)" covering the unswitched and
   shared E3-E7 regions, so later agents can look names up fast.

Tests: tests/test_state.c: flags set/clear/invert/test for both vehicles,
channel write/or/and/read masks, time, state-vector round trip.
