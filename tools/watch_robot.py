#!/usr/bin/env python3
"""Long-term monitor for one robot: records everything and tells apart
"ESP32 turned off" / "ESP32 restarted" / "just out of Wi-Fi range".

  ~/.platformio/penv/bin/python ~/Projects/Sonic-v2.1/tools/watch_robot.py --robot shadow --minutes 20

Writes to crashlogs/:
  watch_<time>_<robot>.txt   laptop-side timeline, one line per second plus events:
                             connected?, ping, link quality, and when the robot is
                             silent: is its Wi-Fi still in the scan? did it reboot?
  wifi_<time>.txt + run_*.csv  the robot's own events / status / telemetry (wifi_log.py)
Reconnects to the robot's Wi-Fi by itself after a robot restart. At the end it
switches the laptop back to the Wi-Fi it was on before (also on Ctrl+C).
"""
import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.join(HERE, "..")
PASSWORD = "sonicx123"
ROBOT_IP = "192.168.4.1"


def nmcli(*args, timeout=40):
    try:
        r = subprocess.run(["nmcli", *args], capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout + r.stderr).strip()
    except subprocess.TimeoutExpired:
        return 1, "timeout"


def active_wifi():
    _, out = nmcli("-t", "-f", "NAME,TYPE", "connection", "show", "--active")
    for line in out.splitlines():
        name, _, typ = line.rpartition(":")
        if typ == "802-11-wireless":
            return name
    return None


def link_info():
    r = subprocess.run(["iw", "dev"], capture_output=True, text=True)
    dev = re.search(r"Interface (\S+)", r.stdout)
    if not dev:
        return None, None
    r = subprocess.run(["iw", "dev", dev.group(1), "link"], capture_output=True, text=True)
    ssid = re.search(r"SSID: (.+)", r.stdout)
    sig = re.search(r"signal: (-?\d+) dBm", r.stdout)
    return (ssid.group(1).strip() if ssid else None), (int(sig.group(1)) if sig else None)


def ping_once():
    r = subprocess.run(["ping", "-c", "1", "-W", "1", ROBOT_IP], capture_output=True, text=True)
    m = re.search(r"time=([\d.]+) ms", r.stdout)
    return float(m.group(1)) if m else None


def ssid_visible(ssid):
    nmcli("dev", "wifi", "rescan")
    time.sleep(2.5)
    _, out = nmcli("-t", "-f", "SSID,SIGNAL", "dev", "wifi", "list")
    for line in out.splitlines():
        name, _, sig = line.rpartition(":")
        if name == ssid:
            return int(sig) if sig.isdigit() else 0
    return None


def main():
    with open(os.path.join(HERE, "robots.json")) as f:
        robots = {k: v for k, v in json.load(f).items() if not k.startswith("_")}
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", choices=sorted(robots), required=True)
    ap.add_argument("--minutes", type=float, default=20)
    a = ap.parse_args()
    ssid = robots[a.robot]["ssid"]
    home = active_wifi()
    os.chdir(PROJECT)
    os.makedirs("crashlogs", exist_ok=True)
    path = time.strftime(f"crashlogs/watch_%Y-%m-%d_%H%M%S_{a.robot}.txt")
    log = open(path, "a", buffering=1)

    def say(msg):
        line = time.strftime("%H:%M:%S ") + msg
        log.write(line + "\n")
        print(line, flush=True)

    say(f"watching {a.robot} ({ssid}) for {a.minutes:g} min, home Wi-Fi {home}")
    viewer = None
    end = time.time() + a.minutes * 60
    state = "start"           # connected / silent
    silent_since = None
    last_ok = None
    try:
        while time.time() < end:
            cur, sig = link_info()
            if cur != ssid:
                vis = ssid_visible(ssid)
                if vis is None:
                    if state != "absent":
                        say(f"EVENT robot Wi-Fi NOT in scan (laptop on {cur}) -> robot off or rebooting")
                    state = "absent"
                    continue
                rc, out = nmcli("dev", "wifi", "connect", ssid, "password", PASSWORD)
                if rc != 0:
                    rc, out = nmcli("connection", "up", "id", ssid)
                say(f"EVENT joining {ssid} (scan signal {vis}%): {'ok' if rc == 0 else out[:80]}")
                if rc == 0 and viewer is None:
                    viewer = subprocess.Popen(
                        [sys.executable, os.path.join(HERE, "wifi_log.py"), "--plain"],
                        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    say("viewer/recorder started (wifi_log.py)")
                continue
            if viewer is None:
                viewer = subprocess.Popen(
                    [sys.executable, os.path.join(HERE, "wifi_log.py"), "--plain"],
                    stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                say("viewer/recorder started (wifi_log.py)")
            rtt = ping_once()
            if rtt is not None:
                if state == "silent":
                    say(f"EVENT robot answering again after {time.time() - silent_since:.1f} s")
                elif state != "connected":
                    say("EVENT robot answering")
                state = "connected"
                last_ok = time.time()
                say(f"ok ping {rtt:.0f} ms, link {sig} dBm")
                time.sleep(max(0.0, 1.0 - rtt / 1000))
            else:
                if state != "silent":
                    silent_since = time.time()
                    say(f"EVENT robot SILENT (no ping), link {sig} dBm - checking scan ...")
                    state = "silent"
                    vis = ssid_visible(ssid)
                    say(f"EVENT  robot Wi-Fi in scan: {'YES, signal %d%%' % vis if vis is not None else 'NO -> robot off/rebooting'}")
                else:
                    say(f"silent {time.time() - silent_since:.0f} s, link {sig} dBm")
    except KeyboardInterrupt:
        say("Ctrl+C")
    finally:
        if viewer:
            viewer.send_signal(signal.SIGINT)
            try:
                viewer.wait(5)
            except subprocess.TimeoutExpired:
                viewer.kill()
        if home and home != ssid:
            nmcli("connection", "up", "id", home)
            say(f"back on {home}")
        say(f"done. timeline: {path}")


if __name__ == "__main__":
    main()
