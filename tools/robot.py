#!/usr/bin/env python3
"""Which robot is on USB? Upload the matching firmware.

  ~/.platformio/penv/bin/python ~/Projects/Sonic-v2.1/tools/robot.py info
  ~/.platformio/penv/bin/python ~/Projects/Sonic-v2.1/tools/robot.py upload
  ~/.platformio/penv/bin/python ~/Projects/Sonic-v2.1/tools/robot.py upload --robot shadow [--erase]

One firmware for all robots (PlatformIO env "robot"); the robot identity (name,
Bluetooth, GUI, motor scale) is stored on the board. After the upload this tool
sets it ("name <robot>" over serial) if the board does not have it yet.

Detection, in this order:
  1. ask the running firmware ("params" -> robot=<name>), without resetting it
  2. the board's MAC address (esptool; resets the board) against tools/robots.json
A board that is not known (new / replaced ESP32) is only flashed when you name
it with --robot; its MAC is then added to robots.json. --robot that contradicts
the detected robot is refused (protects against flashing Shadow's firmware onto
Sonic). --erase wipes the flash first (needed once for a used ESP32 that has an
old file system on it). After the upload, the robot must report the right name.
"""
import argparse
import glob
import json
import os
import re
import subprocess
import sys
import time

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.join(HERE, "..")
ROBOTS_FILE = os.path.join(HERE, "robots.json")
PIO = os.path.expanduser("~/.platformio/penv/bin/pio")
ESPTOOL = os.path.expanduser("~/.platformio/packages/tool-esptool/esptool")


def load_robots():
    with open(ROBOTS_FILE) as f:
        data = json.load(f)
    return {k: v for k, v in data.items() if not k.startswith("_")}, data


def save_robots(data):
    with open(ROBOTS_FILE, "w") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def find_port(port=None):
    if port:
        return port
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("No robot on USB (/dev/ttyUSB*, /dev/ttyACM*)")
    if len(ports) > 1:
        sys.exit(f"Several USB serial ports {ports} - give one with --port")
    return ports[0]


def ask_firmware(port, cmd="params", timeout=2.5):
    """Send a command without resetting the ESP32 (DTR/RTS stay asserted)."""
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, 0.2
    s.dtr = s.rts = True
    s.open()
    time.sleep(0.3)
    s.reset_input_buffer()
    s.write((cmd + "\n").encode())
    end, buf = time.time() + timeout, b""
    while time.time() < end:
        buf += s.read(4096)
        if b"params:" in buf and buf.endswith(b"\n"):
            break
    s.close()
    return buf.decode("utf-8", "replace")


def read_mac(port):
    r = subprocess.run([ESPTOOL, "--port", port, "read-mac"], capture_output=True, text=True)
    m = re.search(r"MAC:\s*([0-9a-f:]{17})", r.stdout + r.stderr)
    return m.group(1) if m else None


def detect(port, robots, allow_reset=True):
    """Returns (robot_key or None, how, mac or None)."""
    out = ask_firmware(port)
    m = re.search(r"params: robot=(\S+)", out)
    if m:
        for key, r in robots.items():
            if r["name"] == m.group(1):
                return key, f"running firmware says robot={m.group(1)}", None
    if not allow_reset:
        return None, "no answer from firmware", None
    mac = read_mac(port)
    if mac:
        for key, r in robots.items():
            if mac in r.get("macs", []):
                return key, f"board MAC {mac}", mac
        return None, f"unknown board MAC {mac}", mac
    return None, "no answer from firmware, MAC not readable", None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["info", "upload"])
    ap.add_argument("--robot", help="sonic / shadow (needed for a new board)")
    ap.add_argument("--port")
    ap.add_argument("--erase", action="store_true", help="erase the whole flash first")
    a = ap.parse_args()
    robots, data = load_robots()
    if a.robot and a.robot not in robots:
        sys.exit(f"--robot must be one of {sorted(robots)}")
    port = find_port(a.port)

    key, how, mac = detect(port, robots)
    if key:
        print(f"Connected: {robots[key]['name']}  ({how})")
    else:
        print(f"Connected: UNKNOWN robot ({how})")
    if a.action == "info":
        return 0

    if a.robot and key and a.robot != key:
        sys.exit(f"REFUSED: you said --robot {a.robot}, but this is {robots[key]['name']} ({how}).")
    target = a.robot or key
    if not target:
        sys.exit("Unknown board: say which robot it is, e.g.  robot.py upload --robot shadow")
    if mac is None:
        mac = read_mac(port)
    if mac and mac not in robots[target].setdefault("macs", []):
        for other in robots.values():  # a board moved to another robot
            if mac in other.get("macs", []):
                other["macs"].remove(mac)
        robots[target]["macs"].append(mac)
        data.update(robots)
        save_robots(data)
        print(f"Board {mac} registered as {robots[target]['name']} in tools/robots.json")

    env = robots[target].get("env", "robot")
    if a.erase:
        print("Erasing flash ...")
        subprocess.run([PIO, "run", "-d", PROJECT, "-e", env, "-t", "erase", "--upload-port", port], check=True)
    print(f"Uploading environment '{env}' to {robots[target]['name']} on {port} ...")
    r = subprocess.run([PIO, "run", "-d", PROJECT, "-e", env, "-t", "upload", "--upload-port", port])
    if r.returncode != 0:
        sys.exit("Upload FAILED")
    time.sleep(6)
    out = ask_firmware(port)
    m = re.search(r"params: robot=(\S+)", out)
    if not (m and m.group(1) == robots[target]["name"]):
        # One firmware for all robots: the identity is stored on the board. Set it.
        print(f"Robot reports {m.group(1) if m else '?'} - storing identity '{target}' on the board ...")
        ask_firmware(port, f"name {target}", timeout=1.5)
        time.sleep(7)  # the robot restarts to apply it
        out = ask_firmware(port)
        m = re.search(r"params: robot=(\S+)", out)
    if m and m.group(1) == robots[target]["name"]:
        print(f"OK: robot answers as {m.group(1)}")
        print(re.search(r"params:.*", out).group(0))
        return 0
    print("WARNING: robot did not confirm its name after the upload. Output:\n" + out[-500:])
    return 1


if __name__ == "__main__":
    sys.exit(main())
