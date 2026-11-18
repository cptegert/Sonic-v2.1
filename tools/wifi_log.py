#!/usr/bin/env python3
"""Live Wi-Fi log viewer for the Sonic robot.

1. Connect the laptop to the robot's Wi-Fi:  Sonic-X-log  (password sonicx123)
2. Run:  ~/.platformio/penv/bin/python ~/Projects/Sonic-v2.1/tools/wifi_log.py

Screen layout:
  top     live status (updated 5x per second)
  middle  events / command output (PgUp / PgDn to scroll back)
  bottom  command line: type  status, sensors, log, clear  + Enter
Quit with Ctrl+C (or type  quit).  --plain = simple scrolling text, no layout.
Everything received is also saved to crashlogs/wifi_<date>.txt.

PID telemetry: while the robot drives, every loop is saved as one CSV row to
crashlogs/run_<date>_<time>_r<N>.csv (one file per drive). Summarise a run with:
  ~/.platformio/penv/bin/python ~/Projects/Sonic-v2.1/tools/analyze_run.py <csv>

Status fields: up=uptime  cyc=loop ms  max=slowest loop since last heartbeat
sw=main switch  drive=driving  man=manual  ble=app connected  rev=reversing
spd/str=motor/servo us  dist=7 sensors cm [FFL FL L M R FR FFR]  i2cErr=I2C errors
"""
import curses
import os
import queue
import socket
import sys
import textwrap
import threading
import time

ROBOT = (os.environ.get("SONIC_IP", "192.168.4.1"), 4210)  # SONIC_IP only for testing
MAX_LINES = 5000

# crashlogs/ next to the project, no matter where the script is started from
os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
os.makedirs("crashlogs", exist_ok=True)
out_path = time.strftime("crashlogs/wifi_%Y-%m-%d_%H%M%S.txt")
out = open(out_path, "a", buffering=1)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", 0))
sock.settimeout(0.5)

rx = queue.Queue()  # ("status", text) / ("line", text) / ("rec", text)


class Recorder:
    """Writes telemetry rows (T,...) to one CSV file per run (TH,<run>,<header>)."""
    def __init__(self):
        self.run = None
        self.last_t = None
        self.file = None
        self.rows = 0
        self.path = None

    def header(self, line):
        _, run, header = line.split(",", 2)
        if run == self.run and self.file is not None:
            return  # repeated header of the same run
        self._open(run, header)

    def _open(self, run, header):
        self.header_line = header
        if self.file:
            self.file.close()
        self.run = run
        self.rows = 0
        self.path = time.strftime(f"crashlogs/run_%Y-%m-%d_%H%M%S_r{run}.csv")
        self.file = open(self.path, "w", buffering=1)
        self.file.write(header + "\n")
        rx.put(("line", time.strftime("%H:%M:%S ") + f"# recording run {run} -> {self.path}"))

    def row(self, line):
        if not self.file:
            return  # header not seen yet (comes within 2 s)
        try:
            t = float(line[2:].split(",", 1)[0])
        except ValueError:
            return
        if self.last_t is not None and t < self.last_t - 1000:
            # robot time went backwards = the robot restarted: its run numbers start
            # again at 1, so start a new file instead of appending to the old one
            rx.put(("line", time.strftime("%H:%M:%S ") + "# robot restarted -> new recording file"))
            self.file.close()
            self.file = None
            self.run = None
            self.last_t = None
            return
        self.last_t = t
        self.file.write(line[2:] + "\n")
        self.rows += 1
        if self.rows % 25 == 0:
            rx.put(("rec", f"REC run {self.run}: {self.rows} rows -> {self.path}"))


recorder = Recorder()


def send(cmd):
    try:
        sock.sendto(cmd.encode(), ROBOT)
    except OSError as e:
        rx.put(("line", f"# send failed: {e} (on the Sonic-X-log Wi-Fi?)"))


def keepalive():
    while True:
        send("hello")
        time.sleep(2)


def receiver():
    last_rx = 0.0
    connected = False
    while True:
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            if connected and time.time() - last_rx > 3:
                connected = False
                msg = "# NO DATA from robot for 3 s (out of range / reset / power?)"
                out.write(time.strftime("%H:%M:%S ") + msg + "\n")
                rx.put(("line", time.strftime("%H:%M:%S ") + msg))
                rx.put(("status", "(no data)"))
            continue
        except OSError:
            time.sleep(0.5)
            continue
        if not connected:
            connected = True
            rx.put(("line", time.strftime("%H:%M:%S ") + "# robot connected"))
        last_rx = time.time()
        stamp = time.strftime("%H:%M:%S")
        for line in data.decode("utf-8", "replace").splitlines():
            if line.startswith("T,"):
                recorder.row(line)
                continue
            if line.startswith("TH,"):
                recorder.header(line)
                continue
            out.write(f"{stamp} {line}\n")
            if line.startswith("S "):
                rx.put(("status", line[2:]))
            else:
                rx.put(("line", f"{stamp} {line}"))


def run_plain():
    """Old style: everything scrolls, status only every 2 s."""
    def stdin_commands():
        for line in sys.stdin:
            if line.strip():
                send(line.strip())
    threading.Thread(target=stdin_commands, daemon=True).start()
    last_status = 0.0
    while True:
        kind, text = rx.get()
        if kind == "rec":
            continue
        if kind == "status":
            if time.time() - last_status < 2:
                continue
            last_status = time.time()
            text = "S " + text
        print(text, flush=True)


def run_curses(scr):
    curses.curs_set(1)
    curses.use_default_colors()
    for i, color in enumerate([curses.COLOR_CYAN, curses.COLOR_YELLOW, curses.COLOR_RED, curses.COLOR_GREEN], 1):
        curses.init_pair(i, color, -1)
    scr.timeout(50)
    scr.keypad(True)

    lines = [f"# talking to robot {ROBOT[0]}:{ROBOT[1]}, saving to {out_path}",
             "# commands: status, sensors, log, clear, quit   |  PgUp/PgDn scroll"]
    status = "(waiting for robot...)"
    rec = ""
    typed = ""
    scroll = 0  # lines scrolled back from the bottom

    def color_for(line):
        if "NO DATA" in line or "ERROR" in line.upper() or "BOOT" in line or "CRASH" in line:
            return curses.color_pair(3)
        if "TRAPPED" in line or "slow loop" in line or "DISCONNECTED" in line or "FAIL" in line:
            return curses.color_pair(2)
        if "connected" in line or "driving started" in line:
            return curses.color_pair(4)
        return 0

    while True:
        # drain received data
        try:
            while True:
                kind, text = rx.get_nowait()
                if kind == "status":
                    status = text
                elif kind == "rec":
                    rec = text
                else:
                    lines.append(text)
                    if scroll:
                        scroll += 1  # keep the view still while scrolled back
            del lines[:-MAX_LINES]
        except queue.Empty:
            pass

        h, w = scr.getmaxyx()
        scr.erase()
        # status block
        status_rows = textwrap.wrap(status, w - 1)[:3] or [""]
        for i, row in enumerate(status_rows):
            scr.addnstr(i, 0, row, w - 1, curses.color_pair(1) | curses.A_BOLD)
        top = len(status_rows)
        if rec:
            scr.addnstr(top, 0, rec, w - 1, curses.color_pair(4))
            top += 1
        scr.hline(top, 0, curses.ACS_HLINE, w)
        # log area
        area = h - top - 3
        if area > 0:
            end = len(lines) - scroll
            visible = lines[max(0, end - area):end]
            for i, row in enumerate(visible):
                scr.addnstr(top + 1 + i, 0, row, w - 1, color_for(row))
        scr.hline(h - 2, 0, curses.ACS_HLINE, w)
        if scroll:
            scr.addnstr(h - 2, 2, f" scrolled back {scroll} lines (PgDn) ", w - 4, curses.A_REVERSE)
        prompt = "> " + typed
        scr.addnstr(h - 1, 0, prompt, w - 1)
        scr.move(h - 1, min(len(prompt), w - 1))
        scr.refresh()

        ch = scr.getch()
        if ch == -1:
            continue
        if ch in (curses.KEY_ENTER, 10, 13):
            cmd = typed.strip()
            typed = ""
            scroll = 0
            if cmd in ("quit", "exit", "q"):
                return
            if cmd:
                lines.append(f"> {cmd}")
                send(cmd)
        elif ch in (curses.KEY_BACKSPACE, 127, 8):
            typed = typed[:-1]
        elif ch == curses.KEY_PPAGE:
            scroll = min(scroll + max(1, area - 1), max(0, len(lines) - 1))
        elif ch == curses.KEY_NPAGE:
            scroll = max(0, scroll - max(1, area - 1))
        elif 32 <= ch < 127:
            typed += chr(ch)


threading.Thread(target=keepalive, daemon=True).start()
threading.Thread(target=receiver, daemon=True).start()
try:
    if "--plain" in sys.argv or not sys.stdout.isatty():
        run_plain()
    else:
        curses.wrapper(run_curses)
except KeyboardInterrupt:
    pass
print(f"saved to {out_path}")
