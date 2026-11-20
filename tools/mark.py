#!/usr/bin/env python3
"""Mark what you see during a drive, with an exact timestamp.

  ~/.platformio/penv/bin/python ~/Projects/Sonic-v2.1/tools/mark.py

Type a number + Enter for a quick mark, or any text + Enter for your own note.
Run it next to tools/watch_robot.py (or wifi_log.py). The marks are saved to
crashlogs/marks_<date>_<time>.txt with the laptop time (to 0.1 s); the robot's
status lines in crashlogs/wifi_*.txt carry the same laptop time, so each mark
lines up exactly with the robot's data. Ctrl+C or "q" to quit.
"""
import os
import sys
import time

QUICK = {
    "1": "tipped over",
    "2": "turned around / driving the wrong way",
    "3": "getting slower",
    "4": "stuck / pushing against a wall",
    "5": "stopped completely",
    "6": "app (RemoteXY) lost connection",
    "7": "hit something / crash",
    "8": "back to normal",
    "0": "test start",
}

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(os.path.join(HERE, ".."))
os.makedirs("crashlogs", exist_ok=True)
path = time.strftime("crashlogs/marks_%Y-%m-%d_%H%M%S.txt")


def stamp():
    t = time.time()
    return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int(t * 10) % 10}"


print("Quick marks (number + Enter):")
for k, v in QUICK.items():
    print(f"  {k} = {v}")
print("Or type any note + Enter.  q = quit\n")
n = 0
with open(path, "a", buffering=1) as f:
    try:
        while True:
            line = input("> ").strip()
            if not line:
                continue
            if line.lower() in ("q", "quit", "exit"):
                break
            text = QUICK.get(line, line)
            ts = stamp()
            f.write(f"{ts} MARK {text}\n")
            n += 1
            print(f"  [{ts}] {text}")
    except (KeyboardInterrupt, EOFError):
        print()
print(f"{n} marks saved to {path}")
