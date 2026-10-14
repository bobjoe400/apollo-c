#!/bin/bash
# hrun.sh ROOT SCENARIO TAG MAXSIMTIME [STEP]: one headless run to its script's "end", collected
# into $NASSP_RUNS/TAG, the install's configuration restored. Polls in 5 s steps (each call of
# collect.sh waits at most 50 s).
H=$(dirname "$0")
bash "$H/hstart.sh" "$1" "$2" "$4" "${5:-0.02}" >/dev/null || exit 1
for i in $(seq 1 40); do
  r=$(bash "$H/collect.sh" "$1" "$3")
  case "$r" in RUNNING) ;; *) echo "$r"; break ;; esac
done
bash "$H/hrestore.sh" "$1" >/dev/null
