#!/bin/bash
# hrestore.sh ROOT: put the install's own Orbiter_NG.cfg back after a headless session.
R=$(echo "$1" | sed 's|C:|/mnt/c|; s|\\|/|g')
[ -f "$R/Orbiter_NG.cfg.windowed" ] && cp "$R/Orbiter_NG.cfg.windowed" "$R/Orbiter_NG.cfg" && rm "$R/Orbiter_NG.cfg.windowed" && echo restored
