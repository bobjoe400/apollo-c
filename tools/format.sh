#!/bin/sh
# format.sh -- the project formatter pipeline (docs/STYLE.md).
#
#   tools/format.sh FILE...           rewrite the files in place
#   tools/format.sh --check FILE...   format copies; list each file that
#                                     would change (FAIL: path), exit 1 if any
#
# Pipeline: tools/fix_comments.py (comments out of expressions, wraps
# joined, paragraphs), Uncrustify twice (layout; 0.72 needs a second pass to
# settle), tools/fix_comments.py again (paragraph column alignment, which
# Uncrustify cannot scope to a paragraph).  The result is a fixed point:
# running the pipeline on its own output changes nothing.
set -e
UNCRUSTIFY=${UNCRUSTIFY:-uncrustify}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
CFG="$ROOT/uncrustify.cfg"
FIX="$ROOT/tools/fix_comments.py"
command -v "$UNCRUSTIFY" >/dev/null || { echo "$UNCRUSTIFY not installed"; exit 1; }

check=0
if [ "$1" = "--check" ]; then
    check=1
    shift
fi
[ $# -gt 0 ] || { echo "usage: $0 [--check] FILE..."; exit 2; }

run() {
    python3 -I "$FIX" "$@"
    "$UNCRUSTIFY" -q -c "$CFG" --replace --no-backup "$@"
    "$UNCRUSTIFY" -q -c "$CFG" --replace --no-backup "$@"
    python3 -I "$FIX" "$@"
}

if [ $check -eq 0 ]; then
    run "$@"
    exit 0
fi

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
for f in "$@"; do
    mkdir -p "$tmp/$(dirname "$f")"
    cp "$f" "$tmp/$f"
done
(cd "$tmp" && run "$@")
rc=0
for f in "$@"; do
    if ! cmp -s "$f" "$tmp/$f"; then
        echo "FAIL: $f"
        rc=1
    fi
done
exit $rc
