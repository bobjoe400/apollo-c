# Assignment: io-pinball   (tier: translation)

Scope: the DSKY logic into src/io/ with a header you create,
include/agc/io/dsky.h. The noun TABLES are a separate agent (io-nouns):
you consume include/agc/io/nouns.h, which that agent creates; until it
exists, write your code against the interface described below and note it.

- PINBALL_GAME_BUTTONS_AND_LIGHTS.agc (diff CM vs LM; translate once):
  CHARIN key decoding, verb/noun entry state machine, DSPTAB and the relay
  word output to channel 010 (DSPOUT), monitor verbs, load verbs V21-V25
  with data-entry logic, DSPDECVN/DSPOCTWD and the +/- and 5-digit scaling,
  the noun scaling routines (SFRUTAB/SFINTAB consumers), please-perform /
  please-mark verbs, KEY REL / PRO / ENTR / CLR / RSET, the ENDIDLE/
  JOBSLEEP display-wait protocol (continuation ids per
  docs/TRANSLATION_GUIDE.md section 4), V37 program change handoff, the
  flashing and lamp bits on channel 011.
- DISPLAY_INTERFACE_ROUTINES.agc (GOFLASH GODSP GOPERF GOXDSP, PRIODSP,
  LINUS, regular vs priority display interplay with the executive).
- KEYRUPT_UPRUPT.agc (KEYRUPT1/2, UPRUPT with the triple-redundant uplink
  word check).

Noun-table interface you may assume (io-nouns implements it):
  typedef struct { uint8_t noun; uint8_t ncomp; uint8_t scale_type[3];
                   uint16_t data_id[3]; bool dec_only; bool no_load; } agc_noun_t;
  agc_status_t agc_noun_lookup(uint8_t noun, agc_noun_t *out);
  agc_status_t agc_noun_read(uint16_t data_id, agc_dp_t *value);
  agc_status_t agc_noun_write(uint16_t data_id, agc_dp_t value);

Public API: a sim can type keys (agc_dsky_key(code)), read the display
state (struct: VERB NOUN PROG, R1 R2 R3 as signed digit arrays, lamp bits),
and programs can request displays (agc_dsky_display(verb, noun,
continuation ids)). Preserve verb/noun numbers, the 5-bit relay segment
codes, alarm codes, and the AGC's own comments.

Tests: tests/io/dsky.c (compiles now): "V06N36E" sets verb 06 noun 36;
"V21N01E12345E" loads; relay word for digit pairs.
