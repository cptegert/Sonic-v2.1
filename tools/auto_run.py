#!/usr/bin/env python3
"""Automated test run: switch laptop Wi-Fi to the robot, set PID values, drive,
record telemetry, stop, switch Wi-Fi back, analyse.

  ~/.platformio/penv/bin/python ~/Projects/Sonic-v2.1/tools/auto_run.py --kp 6 --kd 7.5 --duration 25
  ~/.platformio/penv/bin/python ~/Projects/Sonic-v2.1/tools/auto_run.py --no-start   (connection test only)

Options: --robot sonic|shadow|auto (default auto: the robot network the laptop
is on, or the only one in reach; asks if both robots are on),
--kp --ki --kd --speed  (omitted = unchanged), --duration seconds of
driving after the 5 s countdown (default 25, max 120), --no-start, --no-wifi-switch
(laptop already on Sonic-X-log), --home NAME (NetworkManager connection to return to;
default = the Wi-Fi connection active when the script starts).

Safety (enforced by the robot): the run stops by itself after --duration, and
within 1 s if this script stops sending keepalives. Someone must still be near
the track. Close the RemoteXY phone app, or it may overwrite the values.
Wi-Fi is ALWAYS switched back at the end, also on errors / Ctrl+C.

Permission: switching Wi-Fi from a non-interactive process (e.g. Claude Code)
needs a polkit rule, otherwise nmcli says "Not authorized to control networking".
One-time setup (lets user reimo switch networks without a password prompt):
  sudo tee /etc/polkit-1/rules.d/50-reimo-networkmanager.rules <<'RULE'
  polkit.addRule(function(action, subject) {
    if (action.id.indexOf("org.freedesktop.NetworkManager.") == 0 && subject.user == "reimo") {
      return polkit.Result.YES;
    }
  });
  RULE
Undo: sudo rm /etc/polkit-1/rules.d/50-reimo-networkmanager.rules
"""
import argparse
import os
import socket
import subprocess
import sys
import threading
import time

ROBOT = (os.environ.get("SONIC_IP", "192.168.4.1"), 4210)
import json as _json
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "robots.json")) as _f:
    ROBOTS = {k: v["ssid"] for k, v in _json.load(_f).items() if not k.startswith("_")}
ROBOT_SSID = None  # set from --robot in main() ("auto" = the robot network in reach)
ROBOT_PASSWORD = "sonicx123"
COUNTDOWN_S = 5

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(os.path.join(HERE, ".."))
os.makedirs("crashlogs", exist_ok=True)
log_path = time.strftime("crashlogs/auto_%Y-%m-%d_%H%M%S.txt")
log_file = open(log_path, "a", buffering=1)


def say(msg):
    line = time.strftime("%H:%M:%S ") + msg
    print(line, flush=True)
    log_file.write(line + "\n")


def nmcli(*args, timeout=40):
    r = subprocess.run(["nmcli", *args], capture_output=True, text=True, timeout=timeout)
    return r.returncode, (r.stdout + r.stderr).strip()


def current_ssid():
    r = subprocess.run(["iw", "dev"], capture_output=True, text=True)
    for line in r.stdout.splitlines():
        line = line.strip()
        if line.startswith("ssid "):
            return line[5:]
    return None


def join_robot_wifi():
    """Join the robot's network (rescan + retries). Returns (ok, message)."""
    rc, out = 1, ""
    for attempt in range(5):  # the robot's network may not be in the scan list yet
        nmcli("dev", "wifi", "rescan")
        time.sleep(3)
        rc, out = nmcli("dev", "wifi", "connect", ROBOT_SSID, "password", ROBOT_PASSWORD)
        if rc == 0 or "Not authorized" in out:
            break
    return rc == 0, out


def pick_robot(choice):
    """--robot auto: the robot network the laptop is on, else the only one in reach."""
    if choice != "auto":
        return choice, "given"
    cur = current_ssid()
    for key, ssid in ROBOTS.items():
        if cur == ssid:
            return key, "laptop is on its Wi-Fi"
    nmcli("dev", "wifi", "rescan")
    time.sleep(3)
    _, out = nmcli("-t", "-f", "SSID", "dev", "wifi", "list")
    seen = {l.strip() for l in out.splitlines()}
    found = [k for k, ssid in ROBOTS.items() if ssid in seen]
    if len(found) == 1:
        return found[0], "only robot network in reach"
    if not found:
        return None, "no robot Wi-Fi in reach (robot off?)"
    return None, f"several robots in reach ({', '.join(found)}) - choose with --robot"


def active_wifi():
    _, out = nmcli("-t", "-f", "NAME,TYPE", "connection", "show", "--active")
    for line in out.splitlines():
        name, _, typ = line.rpartition(":")
        if typ == "802-11-wireless":
            return name
    return None


class Link:
    """UDP link to the robot: keepalive every 0.2 s, collects replies and telemetry."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", 0))
        self.sock.settimeout(0.2)
        self.lines = []          # non-telemetry lines
        self.lock = threading.Lock()
        self.running = True
        self.csv = None
        self.csv_path = None
        self.run_id = None
        self.rows = 0
        self.last_rx = 0.0
        threading.Thread(target=self._keepalive, daemon=True).start()
        threading.Thread(target=self._rx, daemon=True).start()

    def send(self, cmd):
        try:
            self.sock.sendto(cmd.encode(), ROBOT)
        except OSError:
            pass

    def _keepalive(self):
        while self.running:
            self.send("hello")
            time.sleep(0.2)

    def _rx(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(4096)
            except (socket.timeout, OSError):
                continue
            self.last_rx = time.time()
            for line in data.decode("utf-8", "replace").splitlines():
                if line.startswith("TH,"):
                    _, run, header = line.split(",", 2)
                    if run != self.run_id:
                        self.run_id = run
                        if self.csv:
                            self.csv.close()
                        self.csv_path = time.strftime(f"crashlogs/run_%Y-%m-%d_%H%M%S_r{run}.csv")
                        self.csv = open(self.csv_path, "w", buffering=1)
                        self.csv.write(header + "\n")
                elif line.startswith("T,"):
                    if self.csv:
                        self.csv.write(line[2:] + "\n")
                        self.rows += 1
                elif line.startswith("S "):
                    pass
                else:
                    with self.lock:
                        self.lines.append(line)
                    log_file.write(time.strftime("%H:%M:%S ") + line + "\n")
                    if not line.startswith("# robot got") and not line.startswith("# robot: viewer"):
                        print("   robot: " + line, flush=True)

    def ask(self, cmd, expect, timeout=1.5, tries=3):
        """Send a command, wait for a reply line containing `expect` (str or tuple).
        UDP can drop packets, so the command is re-sent up to `tries` times.
        Only use for commands that are safe to repeat (start answers "Already ON")."""
        expects = (expect,) if isinstance(expect, str) else expect
        with self.lock:
            start = len(self.lines)
        for _ in range(tries):
            self.send(cmd)
            end = time.time() + timeout
            while time.time() < end:
                with self.lock:
                    for line in self.lines[start:]:
                        if any(e in line for e in expects):
                            return line
                time.sleep(0.05)
        return None

    def close(self):
        self.running = False
        if self.csv:
            self.csv.close()


def main():
    ap = argparse.ArgumentParser()
    for k in ("kp", "ki", "kd", "speed"):
        ap.add_argument("--" + k, type=float)
    ap.add_argument("--duration", type=int, default=25)
    ap.add_argument("--no-start", action="store_true")
    ap.add_argument("--no-wifi-switch", action="store_true")
    ap.add_argument("--home")
    ap.add_argument("--robot", choices=sorted(ROBOTS) + ["auto"], default="auto")
    a = ap.parse_args()
    global ROBOT_SSID
    if "SONIC_IP" in os.environ:  # test against a local fake robot
        key, why = (a.robot if a.robot != "auto" else "sonic"), "test"
    else:
        key, why = pick_robot(a.robot)
    if not key:
        say("robot: " + why)
        return 1
    ROBOT_SSID = ROBOTS[key]
    say(f"robot: {key} ({why}) -> Wi-Fi {ROBOT_SSID}")
    if not 1 <= a.duration <= 120:
        sys.exit("--duration 1..120")

    home = a.home or active_wifi()
    say(f"log: {log_path}   home Wi-Fi: {home}")
    link = None
    csv_path = None
    try:
        # --no-wifi-switch: don't switch back at the end, but still (re)join the
        # robot if the laptop drifted to another network (e.g. after a robot reboot)
        testing = "SONIC_IP" in os.environ  # fake robot on this laptop, no Wi-Fi needed
        if not testing and (not a.no_wifi_switch or current_ssid() != ROBOT_SSID):
            say(f"switching Wi-Fi to {ROBOT_SSID} ...")
            ok, out = join_robot_wifi()
            if not ok:
                say(f"could not join {ROBOT_SSID}: {out}")
                if "Not authorized" in out:
                    say("NetworkManager needs permission - see README note in tools/auto_run.py")
                return 1
        link = Link()
        params = None
        for _ in range(20):  # up to ~20 s for the link to come up
            params = link.ask("params", "params:", timeout=1.0, tries=1)
            if params:
                break
        if not params:
            say("robot not answering on Wi-Fi")
            return 1
        say("robot: " + params)

        for k in ("kp", "ki", "kd", "speed"):
            v = getattr(a, k)
            if v is not None:
                r = link.ask(f"set {k} {v}", "params:")
                if not r:
                    say(f"set {k} failed (main switch ON?)")
                    return 1
                params = r
        say("using: " + params)

        if a.no_start:
            say("--no-start: connection test only, not driving")
            return 0

        # A lost reply + resend gives "Already ON" = the first start arrived
        r = link.ask(f"start {a.duration}", ("starting", "Already ON", "refusing"))
        if not r or "refusing" in r:
            say("start refused: " + (link.lines[-1] if link.lines else "no reply"))
            return 1
        say(f"countdown {COUNTDOWN_S} s, then driving {a.duration} s (robot stops by itself)")
        end = time.time() + COUNTDOWN_S + a.duration + 3
        while time.time() < end:
            time.sleep(1)
            if time.time() - link.last_rx > 3:
                say("no data from robot for 3 s!")
        link.ask("stop", "stopped")  # repeating stop is always safe
        time.sleep(0.5)
        csv_path = link.csv_path
        say(f"recorded {link.rows} rows -> {csv_path}")
        return 0
    except KeyboardInterrupt:
        say("Ctrl+C")
        return 1
    finally:
        if link:
            link.send("stop")  # belt and braces; the robot also stops when keepalives end
            time.sleep(0.3)
            link.close()
        if not a.no_wifi_switch and home:
            say(f"switching Wi-Fi back to {home} ...")
            rc, out = nmcli("connection", "up", "id", home)
            say("back on " + home if rc == 0 else f"could not switch back: {out}")
        if csv_path:
            print(flush=True)
            subprocess.run([sys.executable, os.path.join(HERE, "analyze_run.py"), csv_path])


if __name__ == "__main__":
    sys.exit(main())
