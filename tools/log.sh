#!/usr/bin/env bash
# One command to log a robot live:
#   tools/log.sh shadow        (or sonic / knuckles; optional 2nd arg: minutes, default 60)
# Joins the robot's Wi-Fi (and rejoins after robot restarts), shows the live viewer
# (status, events, commands) and records everything to crashlogs/ (wifi_*.txt,
# run_*.csv, watch_*.txt). Marks: type !3 (slower), !4 (stuck), !5 (stopped) or
# !<text> + Enter. Quit with q or Ctrl+C - the laptop goes back to its Wi-Fi.
cd "$(dirname "$0")/.." || exit 1
ROBOT="${1:-shadow}"
MIN="${2:-60}"
PY="$HOME/.platformio/penv/bin/python"
"$PY" tools/watch_robot.py --robot "$ROBOT" --minutes "$MIN" --no-recorder > /dev/null 2>&1 &
WATCH=$!
trap 'kill -INT $WATCH 2>/dev/null; wait $WATCH 2>/dev/null' EXIT
sleep 6   # let it join the robot's Wi-Fi
START=$(date +%s)
"$PY" tools/wifi_log.py
# a drive with laps was recorded during this log -> open the track view
NEW=$(find crashlogs -name 'run_*.csv' -newermt "@$START" 2>/dev/null | head -1)
if [ -n "$NEW" ]; then
  "$PY" tools/track_map.py > /dev/null 2>&1 && echo "track view opened in the browser"
fi
