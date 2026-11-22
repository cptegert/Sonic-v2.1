#!/usr/bin/env bash
# One command to log a robot live:
#   tools/log.sh shadow          (or sonic / knuckles)  -> dashboard in the browser
#   tools/log.sh shadow term     -> old terminal viewer (wifi_log.py) + track page
# Joins the robot's Wi-Fi (and rejoins after robot restarts) and records everything
# to crashlogs/ (wifi_*.txt, run_*.csv, watch_*.txt).
# Dashboard (http://localhost:8770): live status, marks (wrong way / stuck / ...),
# laps, the track (draw your own, the robot's data is placed on it), named tracks,
# speed calibration. Quit with Ctrl+C - the laptop goes back to its Wi-Fi.
cd "$(dirname "$0")/.." || exit 1
ROBOT="${1:-shadow}"
MODE="${2:-dashboard}"
PY="$HOME/.platformio/penv/bin/python"
"$PY" tools/watch_robot.py --robot "$ROBOT" --minutes 180 --no-recorder > /dev/null 2>&1 &
WATCH=$!
trap 'kill -INT $WATCH 2>/dev/null; wait $WATCH 2>/dev/null' EXIT
echo "joining the robot's Wi-Fi ..."
sleep 6   # let it join the robot's Wi-Fi

if [ "$MODE" != "term" ]; then
  "$PY" tools/dashboard.py --robot "$ROBOT"
  exit 0
fi

START=$(date +%s)
# live track view: rebuilt every 10 s from the drive being recorded, page reloads itself
LIVE=crashlogs/track_live.html
echo '<!doctype html><meta http-equiv="refresh" content="5"><body style="font:16px system-ui;background:#161615;color:#ececea;padding:24px"><h2>Track map - live</h2><p>waiting for a drive ...</p>' > "$LIVE"
(
  OPENED=0
  while sleep 10; do
    NEW=$(find crashlogs -name 'run_*.csv' -newermt "@$START" 2>/dev/null | xargs -r ls -t 2>/dev/null | head -1)
    [ -n "$NEW" ] && "$PY" tools/track_map.py "$NEW" --no-open --out "$LIVE" --refresh 10 > /dev/null 2>&1
    if [ $OPENED = 0 ]; then xdg-open "$LIVE" > /dev/null 2>&1; OPENED=1; fi
  done
) &
LIVEPID=$!
trap 'kill $LIVEPID 2>/dev/null; kill -INT $WATCH 2>/dev/null; wait $WATCH 2>/dev/null' EXIT
"$PY" tools/wifi_log.py
kill $LIVEPID 2>/dev/null
# final track view of the newest drive recorded during this log
NEW=$(find crashlogs -name 'run_*.csv' -newermt "@$START" 2>/dev/null | xargs -r ls -t 2>/dev/null | head -1)
if [ -n "$NEW" ]; then
  "$PY" tools/track_map.py "$NEW" > /dev/null 2>&1 && echo "final track view opened in the browser"
fi
