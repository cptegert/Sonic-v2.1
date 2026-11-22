#!/usr/bin/env python3
"""Find the speed with the best real lap rate: drives the robot at several speeds
(interleaved, repeated) and scores each by LAPS PER MINUTE from the robot's own lap
counter - crashes, reverses and getting stuck cost time and so count automatically.

  ~/.platformio/penv/bin/python ~/Projects/Sonic-v2.1/tools/speed_test.py --robot shadow \
      --speeds 50,55,60,65,70 [--repeat 2] [--duration 45] [--kp 7.05 --kd 7.5]

Needs: robot firmware with lap detection, robot on the track, laptop near it and on
a second internet line (USB tethering) if Claude should follow along. Every run uses
tools/batch_run.py / auto_run.py with their safety (5 s countdown, stops by itself,
stops when the Wi-Fi link is lost, batch aborts when stuck / fallen over).
Ends with the saved speed unchanged: set the winner in the app and press Save.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    with open(os.path.join(HERE, "robots.json")) as f:
        known = [k for k in json.load(f) if not k.startswith("_")]
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", choices=known + ["auto"], default="auto")
    ap.add_argument("--speeds", required=True, help="comma separated, e.g. 50,55,60,65,70")
    ap.add_argument("--repeat", type=int, default=2)
    ap.add_argument("--duration", type=int, default=45, help="seconds per run (>= 30 for a few laps)")
    ap.add_argument("--kp", type=float)
    ap.add_argument("--kd", type=float)
    a = ap.parse_args()
    speeds = [float(x) for x in a.speeds.split(",")]
    if any(not 20 <= s <= 100 for s in speeds):
        sys.exit("speeds must be 20..100")
    extra = "".join(f",{k}={getattr(a, k)}" for k in ("kp", "kd") if getattr(a, k) is not None)
    out = os.path.join(HERE, "..", "crashlogs", time.strftime("speedtest_%Y-%m-%d_%H%M%S.json"))
    cmd = [sys.executable, os.path.join(HERE, "batch_run.py"), "--robot", a.robot,
           "--repeat", str(a.repeat), "--duration", str(a.duration), "--json", out]
    for s in speeds:
        cmd += ["--config", f"speed={s:g}{extra}"]
    print(f"speed test: {speeds}, {a.repeat}x each, {a.duration} s per run\n", flush=True)
    subprocess.run(cmd)
    if not os.path.exists(out):
        sys.exit("no results")
    data = json.load(open(out))

    print("\n" + "=" * 78 + "\nSPEED TEST RESULT (score = laps per minute, higher is better)\n")
    print(f"{'speed':>6s} {'runs':>4s} {'laps/min':>9s} {'lap s (median)':>15s} {'reverses':>9s} {'trapped':>8s} {'near wall %':>12s}")
    best = None
    for i, cfg in enumerate(data["configs"]):
        rs = data["results"].get(str(i), [])
        if not rs:
            print(f"{cfg['speed']:6g} {0:4d}   (no completed run)")
            continue
        def mean(k):
            v = [r[k] for r in rs if not (isinstance(r[k], float) and math.isnan(r[k]))]
            return sum(v) / len(v) if v else float("nan")
        lpm = mean("laps_per_min")
        print(f"{cfg['speed']:6g} {len(rs):4d} {lpm:9.2f} {mean('lap_median'):15.2f} {mean('rev_count'):9.1f}"
              f" {mean('trapped'):8.1f} {mean('near_pct'):12.1f}")
        if not math.isnan(lpm) and (best is None or lpm > best[0]):
            best = (lpm, cfg["speed"])
    if data.get("aborted"):
        print(f"\nNOTE: batch aborted - {data['aborted'].splitlines()[0]}")
    if best:
        print(f"\nBest: speed {best[1]:g} ({best[0]:.2f} laps/min). Set it in the app and press Save.")
    else:
        print("\nNo lap data - does the robot have the firmware with lap detection (robot upload)?")
    print(f"details: {out}")


if __name__ == "__main__":
    main()
