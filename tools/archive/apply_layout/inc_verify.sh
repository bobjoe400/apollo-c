#!/bin/sh
# inc_verify.sh FILE HEADER -- prints "OK FILE HEADER" when FILE without its
# #include of HEADER compiles with the project flags and (src/) yields the same
# object code (objdump -d, no -g) as before.
f=$1; h=$2
W="-std=c11 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wundef -Wcast-qual -Wswitch-enum -Werror -Iinclude"
t=$(mktemp -d); trap 'rm -rf $t' EXIT
base=$(basename "$h")
rel=${h#include/}
# the include line as written: "agc/x/y.h" or a relative "internal.h"/"../internal.h"
line=$(grep -n "^[[:space:]]*#[[:space:]]*include[[:space:]]*\"[^\"]*$base\"" "$f" | while IFS=: read n rest; do p=$(echo "$rest" | sed 's/.*"\(.*\)".*/\1/'); r=$(cd "$(dirname "$f")" && realpath -m "$p" 2>/dev/null); a=$(realpath -m "include/$p"); here=$(realpath -m "$h"); if [ "$r" = "$here" ] || [ "$a" = "$here" ]; then echo $n; fi; done | head -1)
[ -n "$line" ] || { echo "NOLINE $f $h"; exit 0; }
mod="$(dirname "$f")/.incv_$$_$(basename "$f")"
sed "${line}d" "$f" > "$mod"
ok=1
case "$f" in
  tests/*)
    gcc $W -DAGC_HOST_DEBUG -fsyntax-only "$mod" 2>/dev/null || ok=0 ;;
  *)
    gcc $W -O2 -c -o $t/a.o "$f" 2>/dev/null && gcc $W -O2 -c -o $t/b.o "$mod" 2>/dev/null || ok=0
    if [ $ok = 1 ]; then
      objdump -d $t/a.o | tail -n +3 > $t/a.s; objdump -d $t/b.o | tail -n +3 > $t/b.s
      cmp -s $t/a.s $t/b.s || ok=0
      gcc $W -DAGC_HOST_DEBUG -fsyntax-only "$mod" 2>/dev/null || ok=0
    fi ;;
esac
rm -f "$mod"
[ $ok = 1 ] && echo "OK $f $h $line" || echo "KEEP $f $h"
