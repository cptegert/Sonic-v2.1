#!/usr/bin/env python3
"""ESC sweep: find where a robot's motor starts and where it stops getting faster.

WHEELS OFF THE GROUND. Battery on, USB unplugged, main switch OFF.
  ~/.platformio/penv/bin/python ~/Projects/Sonic-v2.1/tools/motor_sweep.py --robot shadow
  ... --steps 1700,1720,1740,1760,1780,1800      (own values; below 1500 = reverse)

Joins the robot's Wi-Fi, then sends fixed motor signals (raw, not scaled), each
for 2 s with a 2 s pause, and prints which step is running. Note:
  - the first step where the wheels turn
  - the first step from which they do not get faster any more
Switches the laptop back to its previous Wi-Fi at the end (also on Ctrl+C).
"""
import argparse
import json
import os
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_IP = ("192.168.4.1", 4210)
PASSWORD = "sonicx123"
STEPS = [1520, 1540, 1560, 1580, 1600, 1650, 1700, 1800]
ON_S, PAUSE_S = 2.0, 2.0


def nmcli(*args):
    r = subprocess.run(["nmcli", *args], capture_output=True, text=True, timeout=40)
    return r.returncode, (r.stdout + r.stderr).strip()


def active_wifi():
    _, out = nmcli("-t", "-f", "NAME,TYPE", "connection", "show", "--active")
    for line in out.splitlines():
        name, _, typ = line.rpartition(":")
        if typ == "802-11-wireless":
            return name
    return None


def main():
    with open(os.path.join(HERE, "robots.json")) as f:
        robots = {k: v for k, v in json.load(f).items() if not k.startswith("_")}
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", choices=sorted(robots), required=True)
    ap.add_argument("--steps", help="comma separated us values, e.g. 1700,1720,1740 (default 1520..1800)")
    a = ap.parse_args()
    steps = [int(x) for x in a.steps.split(",")] if a.steps else STEPS
    if any(not 1000 <= x <= 2000 for x in steps):
        sys.exit("steps must be 1000..2000 us")
    ssid = robots[a.robot]["ssid"]
    home = active_wifi()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.3)
    sock.bind(("", 0))

    def ask(cmd, expect, tries=5):
        for _ in range(tries):
            sock.sendto(cmd.encode(), ROBOT_IP)
            end = time.time() + 0.8
            while time.time() < end:
                try:
                    line = sock.recvfrom(4096)[0].decode("utf-8", "replace")
                except socket.timeout:
                    continue
                for l in line.splitlines():
                    if expect in l:
                        return l
        return None

    try:
        print(f"Joining {ssid} ...", flush=True)
        rc, out = 1, ""
        for attempt in range(6):
            nmcli("dev", "wifi", "rescan")
            time.sleep(4)
            rc, out = nmcli("dev", "wifi", "connect", ssid, "password", PASSWORD)
            if rc == 0:
                break
            rc2, out2 = nmcli("connection", "up", "id", ssid)  # saved profile
            if rc2 == 0:
                rc, out = 0, out2
                break
            print(f"  attempt {attempt + 1}: {out}", flush=True)
        if rc != 0:
            sys.exit(f"Could not join {ssid}: {out}  (robot on? battery on?)")
        params = None
        for _ in range(10):
            params = ask("params", "params:")
            if params:
                break
            time.sleep(1)
        if not params:
            sys.exit("Robot not answering")
        print(params)
        if "main_switch=1" in params:
            sys.exit("Main switch is ON - switch it OFF in the app first")
        print("\nStarting in 5 s - WHEELS OFF THE GROUND\n", flush=True)
        time.sleep(5)
        for n, us in enumerate(steps, 1):
            r = ask(f"motor {us} {ON_S}", "motor test:")
            print(f"  >>> STEP {n}/{len(steps)}:  {us} us  (motor ON for {ON_S:.0f} s)"
                  + ("" if r else "   [no reply from robot!]"), flush=True)
            time.sleep(ON_S)
            print("      pause", flush=True)
            time.sleep(PAUSE_S)
        print("\nDone. Which step did the wheels START turning, and from which step"
              " did they NOT get faster any more?")
    except KeyboardInterrupt:
        print("\nCtrl+C")
    finally:
        try:
            sock.sendto(b"motor 1500 0.1", ROBOT_IP)
        except OSError:
            pass
        if home and home != ssid:
            print(f"Switching Wi-Fi back to {home} ...")
            nmcli("connection", "up", "id", home)


if __name__ == "__main__":
    main()
