#!/bin/bash
# hstart.sh ROOT SCENARIO MAXSIMTIME [STEP]: start one headless NASSP session (Orbiter console mode,
# no graphics client, fixed time step) from WSL and return. Kills any Orbiter first (one session at a
# time: NASSP's telemetry binds TCP 14242). The install's Orbiter_NG.cfg is kept as
# Orbiter_NG.cfg.windowed and replaced by a copy without D3D9Client and XRSound; hrestore.sh puts it
# back. ROOT is a Windows path (C:\NASSP9-dev), SCENARIO relative to its Scenarios folder.
ROOT=$1; SCN=$2; T=$3; DT=${4:-0.02}
PS=/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe
TL=/mnt/c/Windows/System32/tasklist.exe
/mnt/c/Windows/System32/taskkill.exe /IM Orbiter.exe /F >/dev/null 2>&1
/mnt/c/Windows/System32/taskkill.exe /IM Orbiter_ng.exe /F >/dev/null 2>&1
sleep 2; if $TL | grep -qi orbiter; then echo "Orbiter still running: abort"; exit 1; fi
R=$(echo "$ROOT" | sed 's|C:|/mnt/c|; s|\\|/|g')
[ -f "$R/Orbiter_NG.cfg.windowed" ] || cp "$R/Orbiter_NG.cfg" "$R/Orbiter_NG.cfg.windowed"
awk '/^ACTIVE_MODULES/{m=1} m && /^  (D3D9Client|XRSound)\r?$/{next} /^END_MODULES/{m=0} {print}' "$R/Orbiter_NG.cfg.windowed" > "$R/Orbiter_NG.cfg"
rm -f "$R"/apollo-c-*.log "$R"/apollo-c-*.csv "$R/Scenarios/(Current state).scn"
$PS -NoProfile -Command "Start-Process -FilePath $ROOT\\Modules\\Server\\Orbiter.exe -WorkingDirectory $ROOT -ArgumentList '-s','\"$SCN\"','-f','$DT','-t','$T','-x'" >/dev/null
echo "started headless $ROOT $SCN dt $DT to sim $T"
