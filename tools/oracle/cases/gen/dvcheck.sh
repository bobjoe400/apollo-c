#!/bin/sh
# dvcheck.sh SPEC...  -- list the runs whose dump changes when yaAGC's DV
# fast path is fixed (core-numeric finding: SpToDecent reads L sign-
# extended, so a -0 minor word is taken as a borrow and the quotient comes
# out one major LSB low).  Builds a patched agc_probe in $DVTMP (default
# ./dvtmp) from ../../yaAGC-src; the committed binaries are not touched.
# A run listed here exercises the emulator bug, not the AGC: change its
# inputs (or document it) before committing the case.
set -e
here=$(cd "$(dirname "$0")" && pwd)
oracle=$(cd "$here/../.." && pwd)
tmp=${DVTMP:-$here/dvtmp}
if [ ! -x "$tmp/bin/agc_probe" ]; then
    mkdir -p "$tmp"
    cp -r "$oracle/yaAGC-src" "$oracle/agc_probe.c" "$oracle/Makefile" "$tmp/"
    sed -i 's/^  Msb = LsbSP\[-1\];/  Msb = LsbSP[-1] \& 077777;/; s/^  Lsb = \*LsbSP;/  Lsb = *LsbSP \& 077777;/' \
        "$tmp/yaAGC-src/agc_engine.c"
    grep -q 'LsbSP\[-1\] & 077777' "$tmp/yaAGC-src/agc_engine.c"
    make -C "$tmp" >/dev/null
fi
for spec in "$@"; do
    a=$(cd "$oracle" && AGC_ORACLE_ROPES=ropes ./bin/agc_probe < "$spec" | grep -v '^#' || true)
    b=$(cd "$oracle" && AGC_ORACLE_ROPES=ropes "$tmp/bin/agc_probe" < "$spec" | grep -v '^#' || true)
    printf '%s\n' "$a" > "$tmp/a.txt"
    printf '%s\n' "$b" > "$tmp/b.txt"
    awk 'NR==FNR { if ($1 == "CASE") c = $2; else k[c, $1] = $2; next }
         $1 == "CASE" { c = $2; next }
         (c, $1) in k && k[c, $1] != $2 && !seen[c]++ { print "DVBUG " c }' \
        "$tmp/a.txt" "$tmp/b.txt"
done
