#!/bin/bash
# mkscn_lm.sh ROOT "<NASSP Apollo 11 scenario file>" "<test name>" <lua name>: a headless LM test
# scenario in ROOT/Scenarios/apollo-c: the Lua script added, the focus on the MCC vessel (a focused
# NASSP LEM or any NASSP Saturn crashes Orbiter's console mode) and Columbia dropped. The script
# (sim/nassp/lua/<lua name>.lua, common.lua) is copied to ROOT/Script/apollo-c.
R=$(echo "$1" | sed 's|C:|/mnt/c|; s|\\|/|g'); H=$(dirname "$0")
D="$R/Scenarios/Project Apollo - NASSP/Apollo - Mission Scenarios/Apollo 11/$2"
mkdir -p "$R/Scenarios/apollo-c" "$R/Script/apollo-c"
tr -d '\r' < "$D" | awk -v s="  Script apollo-c/$4" '{print} /^  Date MJD/{print s}' | sed 's/^  Ship .*/  Ship MCC/' \
  | awk 'BEGIN{s=0} /^BEGIN_SHIPS/{print; s=1; next} /^END_SHIPS/{s=0} s && /^[A-Za-z0-9-]+:/{split($0,a,":"); k=(a[1]!="Columbia")} s{ if(k) print; next} {print}' \
  | sed 's/$/\r/' > "$R/Scenarios/apollo-c/$3.scn"
cp "$H/../lua/$4.lua" "$H/../lua/common.lua" "$R/Script/apollo-c/"
echo "$R/Scenarios/apollo-c/$3.scn"
