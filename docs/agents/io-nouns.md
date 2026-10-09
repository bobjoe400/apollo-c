# Assignment: io-nouns   (tier: mechanical)

Scope: PINBALL_NOUN_TABLES.agc for both vehicles (they differ) into
src/io/noun_tables_csm.c and src/io/noun_tables_lm.c plus
include/agc/agc_nouns.h (you create it) and src/io/noun_data.c.

Header interface (agent io-pinball codes against exactly this):
  typedef struct { uint8_t noun; uint8_t ncomp; uint8_t scale_type[3];
                   uint16_t data_id[3]; bool dec_only; bool no_load; } agc_noun_t;
  agc_status_t agc_noun_lookup(uint8_t noun, agc_noun_t *out);
  agc_status_t agc_noun_read(uint16_t data_id, agc_dp_t *value);
  agc_status_t agc_noun_write(uint16_t data_id, agc_dp_t value);
plus an enum of scale types (the AGC SF routine codes: octal, straight
fractional, CDU degrees, arc-seconds, time hr/min/sec, velocity, etc. --
take the list from the NNTYPTAB/SFINTAB/SFOUTAB comments) and an enum of
data ids, one per erasable the noun tables reference, named after the AGC
erasable.

Tables are static const arrays with the noun number, component count,
scale types, data ids, and a comment with the AGC noun description
(from ASSEMBLY_AND_OPERATION_INFORMATION.agc noun list). Mixed nouns
(NNADTAB with the IDADDTAB/RUTMXTAB indirections) must be resolved into the
same flat struct. noun_data.c resolves data ids through ONE switch that
calls the accessors in include/agc/agc_state.h; for erasables that core
does not expose yet, return AGC_STATUS_UNIMPLEMENTED and list every one of
them under "Requested core additions" with the AGC scale.

Also transcribe the SFINTAB/SFOUTAB scale-factor constant tables (octal
values with their engineering meaning as comments) into noun_scales.c.

Tests: tests/test_nouns.c: lookup noun 36 (time of AGC clock) gives 3
components with the hr/min/sec scale types; an unknown noun fails; the
table is sorted/complete for 00-99.
