# Read this when resuming after the naming reset (2026-10-10)

While you were paused the tree changed in three ways:

1. **Renames.** docs/NAMING.md rules were applied tree-wide (docs/history/RENAMES.md
   is the full map, with the AGC comment behind each name). Your files,
   your note and your pending header were renamed too. Grep docs/history/RENAMES.md
   for any identifier you remember before searching the tree for it.
   New code you write must follow NAMING.md; add rows for new stems.
2. **Restructure.** Files moved into subsystem subfolders; see the layout in
   docs/ARCHITECTURE.md and the move table in docs/history/waves/restructure.md.
   Your lane's directory is now deeper (e.g. src/csm/entry/, src/lm/dap/rcs/).
   include/agc/ is unchanged.
3. **Lookup tool.** Use `python3 -I tools/agc_label.py LABEL` for AGC
   source lookups (docs/agents/brief.md).

Do NOT merge the job/task id lists in guid_pending.h / exec_pending.h /
entry_pending.h yourself; integration does that. Keep using the ids that
exist. Continue from your note's Progress section and finish your
assignment; add a Lessons section.
