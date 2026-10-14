# oracle -- lane history

Process diary of the oracle lane, moved verbatim out of [docs/modules/oracle.md](../../modules/oracle.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-12 (first and last commit touching the note).

## Requested core additions

None.  (Tests will want a tiny parser for the `NAME octal` format; the
format is documented in `tools/oracle/README.md` section 2.)

## Requested jobs-tasks

None.

## Open questions

* The `.symtab` files are 11.8 MB each (yaYUL dumps every source line);
  keep them, or regenerate on demand from `BUILD.md`?
* Routines that live in superbank (FBANK 40-43) are addressed through the
  symtab's `Super` bit and channel 7; exercised in code, not yet by a case.

