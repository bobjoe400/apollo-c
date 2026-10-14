#!/bin/sh
# apply_layout.sh -- apply the layout proposal of history/ORGANIZATION.md
# (organize lane, 2026-10-13).  ONE-OFF: after it has run and been committed,
# it archives itself (tools/archive/apply_layout/).
#
# Usage (repo root, clean tree, no lane editing src/ include/ tests/ sim/nassp/):
#   sh tools/apply_layout.sh --dry-run     # list every step, change nothing
#   sh tools/apply_layout.sh               # apply, then run the gates
#   sh tools/apply_layout.sh --no-gates    # apply only
#   sh tools/apply_layout.sh --lm-sim      # gates + `make lm-sim` (11 min)
#
# Steps (each idempotent: a step already done reports so and changes nothing):
#   1. delete the functions in tools/layout/dead_functions.tsv (no caller
#      anywhere, no AGC label) and the prototypes never defined
#      (stale_prototypes.tsv);
#   2. make_static.tsv: functions only their own file calls lose their
#      public prototype and become static;
#   3. unused_includes.tsv: #include lines whose removal leaves the object
#      code identical;
#   4. moves.tsv: git mv, then every reference (paths, #include spellings,
#      Markdown links, Makefile, src/nassp/CMakeLists.txt) rewritten;
#   5. tools/format.sh over the touched C files (layout only);
#   6. regenerate docs/overview/xref.md and the module map (needs the AGC
#      sources, see tools/agc_label.py; skipped with a note otherwise);
#   7. gates: make, make test, make check, make nassp-replay [, make lm-sim];
#   8. archive this script and tools/layout/.
# Nothing here changes what a function computes; see history/ORGANIZATION.md for the
# findings that would (they are not applied).
set -eu
DRY=; GATES=1; LMSIM=
for a in "$@"; do
  case "$a" in
    --dry-run) DRY=--dry-run ;;
    --no-gates) GATES= ;;
    --lm-sim) LMSIM=1 ;;
    *) echo "unknown option $a"; exit 2 ;;
  esac
done
L=tools/layout
[ -d "$L" ] || { echo "run from the repo root (tools/layout missing)"; exit 2; }
PY="python3 -I $L/edits.py"
rows() { grep -v '^#' "$1" | grep -v '^[[:space:]]*$'; }

if [ -z "$DRY" ] && ! git diff --quiet -- src include tests sim tools Makefile; then
  echo "uncommitted changes under src include tests sim tools Makefile: commit or stash first"; exit 2
fi

echo "== 1. dead functions and stale prototypes"
rows $L/dead_functions.tsv | while IFS="$(printf '\t')" read -r fn file; do
  $PY delete-func "$fn" "$file" $DRY
done
rows $L/stale_prototypes.tsv | while IFS="$(printf '\t')" read -r fn hdr; do
  $PY delete-proto "$fn" "$hdr" $DRY
done

echo "== 2. file-static functions"
rows $L/make_static.tsv | while IFS="$(printf '\t')" read -r fn file hdr; do
  $PY make-static "$fn" "$file" "$hdr" $DRY
done

echo "== 3. unused includes"
rows $L/unused_includes.tsv | while IFS="$(printf '\t')" read -r file hdr; do
  $PY drop-include "$file" "$hdr" $DRY
done

echo "== 4. moves"
$PY move $L/moves.tsv $DRY --protect $L/moves.tsv
$PY doc-edits $DRY

if [ -n "$DRY" ]; then echo "dry run: nothing changed"; exit 0; fi

echo "== 5. format the touched C files"
FILES=$(git diff --name-only -- src include tests | grep '\.[ch]$' || true)
NEW=$(git diff --cached --name-only --diff-filter=R -- src include tests | grep '\.[ch]$' || true)
# shellcheck disable=SC2086
[ -z "$FILES$NEW" ] || sh tools/format.sh $(printf '%s\n' $FILES $NEW | sort -u | while read -r f; do [ -f "$f" ] && echo "$f"; done)

echo "== 6. generated docs"
if python3 -I tools/xref.py > /dev/null 2>&1; then echo "xref.md regenerated"; else echo "NOTE: tools/xref.py needs the AGC sources (AGC_SRC); rerun it"; fi
if python3 -I tools/depgraph.py > /dev/null 2>&1; then echo "module map regenerated"; else echo "NOTE: rerun python3 -I tools/depgraph.py"; fi

if [ -n "$GATES" ]; then
  echo "== 7. gates"
  make -j"$(nproc)"
  make -j"$(nproc)" test
  make check
  make prolog-report
  make always-ok-report
  make nassp-replay
  [ -z "$LMSIM" ] || make lm-sim
fi

echo "== 8. archive the one-off"
mkdir -p tools/archive/apply_layout
git mv tools/apply_layout.sh tools/archive/apply_layout/apply_layout.sh
for f in $L/*; do git mv "$f" tools/archive/apply_layout/; done
echo "done: review 'git status', commit; tell the nassp lane its new paths (history/ORGANIZATION.md, NASSP area)"
