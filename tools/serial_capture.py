#!/usr/bin/env python3
"""Passive serial capture: listen to the ESP32 WITHOUT resetting it.

Normal serial monitors toggle DTR/RTS on open, which resets the ESP32 through
the auto-reset circuit and destroys the state you wanted to look at. This keeps
DTR and RTS de-asserted so a frozen / crash-looping robot stays as it is.

Usage:
  ~/.platformio/penv/bin/python tools/serial_capture.py [PORT] [SECONDS] [COMMAND]
  (PORT defaults to the first /dev/ttyUSB* or /dev/ttyACM*, SECONDS to 30;
   SECONDS=0 means run until Ctrl+C)
  COMMAND is sent once after connecting, e.g.  log  or  status  (see src/logger.h)

  Example - download the flash log without resetting the robot:
    ~/.platformio/penv/bin/python tools/serial_capture.py /dev/ttyUSB0 15 log

Output is also saved to crashlogs/.
"""
import glob
import os
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] not in ("", "auto") else None
command = sys.argv[3] if len(sys.argv) > 3 else None
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
if port is None:
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("No /dev/ttyUSB* or /dev/ttyACM* found - is the robot plugged in?")
    port = ports[0]

# crashlogs/ next to the project, no matter where the script is started from
os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
os.makedirs("crashlogs", exist_ok=True)
out_path = time.strftime("crashlogs/serial_%Y-%m-%d_%H%M%S.txt")

s = serial.Serial()
s.port = port
s.baudrate = 115200
s.timeout = 0.2
# Linux asserts DTR+RTS when the port opens. With the ESP32 auto-reset
# circuit, "both asserted" and "both released" are harmless, but releasing them
# one after another pulses EN (= reset). So keep both asserted, change nothing.
s.dtr = True
s.rts = True
s.open()

print(f"Listening on {port} (no reset) for {'ever' if seconds == 0 else f'{seconds:.0f} s'} -> {out_path}")
start = time.time()
total = 0
with open(out_path, "wb") as f:
    try:
        while seconds == 0 or time.time() - start < seconds:
            if command and time.time() - start > 0.5:
                s.write((command + "\n").encode())
                command = None
            try:
                data = s.read(4096)
            except serial.SerialException:
                data = b""  # USB hiccup (e.g. board reset); keep listening
                time.sleep(0.2)
            if data:
                total += len(data)
                f.write(data)
                f.flush()
                sys.stdout.write(data.decode("utf-8", "replace"))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
s.close()
print(f"\n--- {total} bytes received, saved to {out_path}")
if total == 0:
    print("Silence: the CPU is probably hung (not crash-looping). That is useful info too.")
