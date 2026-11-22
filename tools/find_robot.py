#!/usr/bin/env python3
"""Which robot is in Wi-Fi reach? Prints its key (sonic / shadow / knuckles).

  find_robot.py            wait until a robot's Wi-Fi appears (Ctrl+C to give up)
  find_robot.py --once     look once, exit 1 if none
Prefers the robot network the laptop is already on; with several in reach, the
strongest signal. Linux (NetworkManager / nmcli).
"""
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "robots.json")) as f:
    ROBOTS = {k: v["ssid"] for k, v in json.load(f).items() if not k.startswith("_")}


def scan():
    try:
        out = subprocess.run(["nmcli", "-t", "-f", "ACTIVE,SIGNAL,SSID", "dev", "wifi", "list", "--rescan", "yes"],
                             capture_output=True, text=True, timeout=30).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    best = None
    for line in out.splitlines():
        active, _, rest = line.partition(":")
        sig, _, ssid = rest.partition(":")
        for key, s in ROBOTS.items():
            if ssid == s:
                score = (active == "yes", int(sig or 0))
                if best is None or score > best[0]:
                    best = (score, key)
    return best[1] if best else None


def main():
    once = "--once" in sys.argv
    told = False
    while True:
        key = scan()
        if key:
            print(key)
            return 0
        if once:
            return 1
        if not told:
            print("no robot Wi-Fi in reach yet - switch a robot on (waiting, Ctrl+C to stop) ...", file=sys.stderr)
            told = True
        time.sleep(5)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(1)
