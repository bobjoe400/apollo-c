#!/bin/bash
# collect.sh ROOT TAG: one poll (<= 60 s) of a session; when its script has logged "end" (or Orbiter
# exited) close it and copy the logs and edit CSVs to $NASSP_RUNS/TAG (default ./runs/TAG).
ROOT=$1; TAG=$2
OUT=${NASSP_RUNS:-./runs}/$TAG
PS=/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe
TL=/mnt/c/Windows/System32/tasklist.exe
R=$(echo "$ROOT" | sed 's|C:|/mnt/c|; s|\\|/|g')
mkdir -p "$OUT"
for i in 1 2 3 4 5 6 7 8 9 10; do
  running=0; $TL /FI "IMAGENAME eq Orbiter.exe" | grep -qi orbiter.exe && running=1
  ended=0; grep -qsP " end\r?$" "$R"/apollo-c-*.log && ended=1
  if [ $running = 0 ] || [ $ended = 1 ]; then break; fi
  sleep 5
done
if [ $running = 1 ] && [ $ended = 0 ]; then echo "RUNNING"; exit 0; fi
if [ $running = 1 ]; then
  $PS -NoProfile -Command "Get-Process Orbiter | ForEach-Object { \$_.CloseMainWindow() | Out-Null }"
  for k in 1 2 3 4 5 6 7 8; do sleep 3; $TL /FI "IMAGENAME eq Orbiter.exe" | grep -qi orbiter.exe || break; done
  /mnt/c/Windows/System32/taskkill.exe /IM Orbiter.exe /F >/dev/null 2>&1
fi
cp "$R/Orbiter.log" "$OUT/Orbiter.log"; cp "$R"/apollo-c-*.log "$R"/apollo-c-*.csv "$OUT/" 2>/dev/null
echo "FINISHED (script ended: $ended) -> $OUT"
