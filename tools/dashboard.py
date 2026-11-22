#!/usr/bin/env python3
"""Robot dashboard: everything in one browser page.

  ~/.platformio/penv/bin/python ~/Projects/Sonic-v2.1/tools/dashboard.py --robot shadow
  (or: roblog shadow  - joins the robot's Wi-Fi and starts this)

The laptop must be on the robot's Wi-Fi (Shadow-log / Sonic-X-log / Knuckles-log,
password sonicx123). Opens http://localhost:8770 with:
  - live status, sensors, events, a command line
  - mark buttons (wrong way / right way again / tipped / stuck / ...): saved in the
    log with the exact time AND sent to the robot ("wrong way" pauses its learning)
  - laps with their trouble, lap time chart
  - the track: robot's own estimate, or your drawing / description with the robot's
    data placed on it, live robot position, turn list, hills, speed map
  - named tracks: the robot keeps a speed map + lap mark per track
  - speed calibration: drives several speeds, scores laps per minute, apply the best
Records like wifi_log.py: crashlogs/wifi_*.txt (log + marks), crashlogs/run_*.csv
(telemetry, one file per drive). Only ONE program can talk to the robot at a time:
close wifi_log.py / auto_run.py while this runs. Ctrl+C to quit.
"""
import argparse
import collections
import json
import math
import os
import re
import socket
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_run  # noqa: E402
import track_model as tm  # noqa: E402

os.chdir(os.path.join(HERE, ".."))
os.makedirs("crashlogs", exist_ok=True)
ROBOT_ADDR = (os.environ.get("SONIC_IP", "192.168.4.1"), 4210)
ROBOTS_JSON = os.path.join(HERE, "robots.json")
MARKS = {  # code -> (log text, robot command)
    "wrong": ("wrong way", "mark wrong"),
    "ok": ("right way again", "mark ok"),
    "tipped": ("tipped over", "mark tipped"),
    "stuck": ("stuck / pushing against a wall", "mark stuck"),
    "crash": ("hit something / crash", "mark crash"),
    "slower": ("getting slower", "mark slower"),
    "stopped": ("stopped completely", "mark stopped"),
    "normal": ("back to normal", "mark normal"),
    "start": ("test start", "mark start"),
    "cleaned": ("wheels cleaned", "mark cleaned"),
}


def now_stamp(tenths=False):
    t = time.time()
    return time.strftime("%H:%M:%S", time.localtime(t)) + (f".{int(t * 10) % 10}" if tenths else "")


def load_robots():
    with open(ROBOTS_JSON) as f:
        return json.load(f)


def save_robot_field(robot, field, value):
    data = load_robots()
    data.setdefault(robot, {})[field] = value
    tmp = ROBOTS_JSON + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
        f.write("\n")
    os.replace(tmp, ROBOTS_JSON)


def to_num(v):
    try:
        return float(v)
    except ValueError:
        return 0.0


# =============================================================================== link
class Link:
    """UDP link: keepalive, log file, telemetry recording, parsed state."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", 0))
        self.sock.settimeout(0.3)
        self.lock = threading.Lock()
        self.log_path = time.strftime("crashlogs/wifi_%Y-%m-%d_%H%M%S.txt")
        self.log = open(self.log_path, "a", buffering=1)
        self.events = collections.deque(maxlen=600)    # (id, stamp, text)
        self.ev_id = 0
        self.status = {}
        self.status_at = 0.0
        self.last_rx = 0.0
        self.connected = False
        self.laps = collections.deque(maxlen=200)       # parsed LAPSUM
        self.marks = collections.deque(maxlen=100)      # (laptop time, robot ms, text)
        self.ww_events = []                             # (laptop time, detected?) from the robot
        self.ww_marks = []                              # (laptop time, wrong?) your marks
        self.series = collections.deque(maxlen=900)     # (laptop time, fwd, back, detected, your flag) ~3 min
        # recording
        self.run = None
        self.header = None
        self.csv = None
        self.csv_path = None
        self.rows = []          # parsed rows of the current drive
        self.rows_ver = 0
        self.last_t = None
        self.finished = {}      # csv path -> rows of finished drives (for the speed test)
        self.waiters = []
        self.capture = None     # list while reading the robot's diary (lines are not events then)
        self.run_started = 0.0
        self.clock_sent = False
        threading.Thread(target=self._keepalive, daemon=True).start()
        threading.Thread(target=self._rx, daemon=True).start()

    def send(self, cmd):
        try:
            self.sock.sendto(cmd.encode(), ROBOT_ADDR)
        except OSError as e:
            self.event(f"# send failed: {e} (laptop on the robot's Wi-Fi?)")

    def _keepalive(self):
        while True:
            self.send("hello")    # also keeps a laptop-started run alive (robot stops after 1 s without)
            time.sleep(0.25)

    def event(self, text, write=True):
        stamp = now_stamp()
        with self.lock:
            self.ev_id += 1
            self.events.append((self.ev_id, stamp, text))
        if write:
            self.log.write(f"{stamp} {text}\n")

    def mark(self, text, robot_cmd=None):
        stamp = now_stamp(True)
        self.log.write(f"{stamp} MARK {text}\n")
        with self.lock:
            self.ev_id += 1
            self.events.append((self.ev_id, stamp, f"MARK {text}"))
            robot_ms = self.rows[-1]["t_ms"] if self.rows else None
            self.marks.append((time.time(), robot_ms, text))
        if robot_cmd:
            self.send(robot_cmd[:31])

    def _rx(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                if self.connected and time.time() - self.last_rx > 3:
                    self.connected = False
                    self.event("# NO DATA from robot for 3 s (out of range / reset / power?)")
                continue
            except OSError:
                time.sleep(0.5)
                continue
            self.last_rx = time.time()
            if not self.connected:
                self.connected = True
                self.clock_sent = False
                self.event("# robot connected")
            for line in data.decode("utf-8", "replace").splitlines():
                if line.startswith("T,"):
                    self._row(line)
                elif line.startswith("TH,"):
                    self._header(line)
                elif line.startswith("S "):
                    self.log.write(f"{now_stamp()} {line}\n")
                    self._status(line[2:])
                    if not self.clock_sent:   # robot's diary gets real date / time
                        self.clock_sent = True
                        tz = time.localtime().tm_gmtoff
                        self.send(f"clock {int(time.time())} {tz}")
                elif self.capture is not None:
                    self.capture.append(line)
                else:
                    self.event(line)
                    self._parse_event(line)

    def _status(self, text):
        st = {}
        for k, v in re.findall(r"(\w+)=(\[[^\]]*\]|\S+)", text):
            st[k] = v
        if "dist" in st:
            st["dist"] = [int(x) for x in st["dist"].strip("[]").split()]
        with self.lock:
            self.status = st
            self.status_at = time.time()

    def _parse_event(self, line):
        if "WRONG WAY? detected" in line:
            with self.lock:
                self.ww_events.append((time.time(), True))
        elif "right way again (detected" in line:
            with self.lock:
                self.ww_events.append((time.time(), False))
        m = re.search(r"LAPSUM (\d+) ([\d.]+)s rev(\d+) trap(\d+) notmov(\d+) blind(\d+) minwall(-?\d+)cm "
                      r"lock(\d+)% minspd(\d+)% spd(\d+)us(?: margin(\d+)cm)?", line)
        if m:
            v = [float(x) if x is not None else None for x in m.groups()]
            with self.lock:
                self.laps.append({"lap": int(v[0]), "s": v[1], "rev": int(v[2]), "trap": int(v[3]),
                                  "notmov": int(v[4]), "blind": int(v[5]), "minwall": int(v[6]),
                                  "lock": int(v[7]), "minspd": int(v[8]), "spd": int(v[9]),
                                  "margin": int(v[10]) if v[10] is not None else None,
                                  "at": time.time(), "run": self.csv_path})
        with self.lock:
            for w in self.waiters:
                w.append(line)

    # ---------------------------------------------------------------- recording
    def _header(self, line):
        _, run, header = line.split(",", 2)
        if run == self.run and self.csv is not None:
            return
        self._close_run()
        self.run = run
        self.header = header.split(",")
        self.csv_path = time.strftime(f"crashlogs/run_%Y-%m-%d_%H%M%S_r{run}.csv")
        self.run_started = time.time()
        self.csv = open(self.csv_path, "w", buffering=1)
        self.csv.write(header + "\n")
        self.last_t = None
        with self.lock:
            self.rows = []
            self.rows_ver += 1
        self.event(f"# recording drive {run} -> {self.csv_path}", write=False)

    def _close_run(self):
        if self.csv:
            self.csv.close()
            with self.lock:
                self.finished[self.csv_path] = self.rows
                while len(self.finished) > 6:
                    self.finished.pop(next(iter(self.finished)))
        self.csv = None

    def _row(self, line):
        if not self.csv or not self.header:
            return
        vals = line[2:].split(",")
        if len(vals) != len(self.header):
            return
        r = {k: to_num(v) for k, v in zip(self.header, vals)}
        if self.last_t is not None and r["t_ms"] < self.last_t - 1000:   # robot restarted
            self.event("# robot restarted -> new recording file", write=False)
            self._close_run()
            self.run = None
            self.last_t = None
            return
        self.last_t = r["t_ms"]
        self.csv.write(line[2:] + "\n")
        with self.lock:
            self.rows.append(r)
            self.rows_ver += 1
            now = time.time()
            if "wfwd" in r and (not self.series or now - self.series[-1][0] >= 0.2):
                self.series.append((round(now, 1), r["wfwd"], r["wback"], int(r["wwd"]), int(r.get("ww", 0))))

    def ask(self, cmd, expect, timeout=1.5, tries=3):
        """Send a command, wait for a reply line containing one of `expect`."""
        expects = (expect,) if isinstance(expect, str) else expect
        got = []
        with self.lock:
            self.waiters.append(got)
        try:
            for _ in range(tries):
                self.send(cmd)
                end = time.time() + timeout
                while time.time() < end:
                    for line in list(got):
                        if any(e in line for e in expects):
                            return line
                    time.sleep(0.05)
            return None
        finally:
            with self.lock:
                self.waiters.remove(got)

    def driving(self):
        return self.status.get("drive") == "1" and time.time() - self.status_at < 3


# =============================================================================== speed test
class SpeedTest:
    SKIP_START_S = 2.0
    MAX_REVERSING_PCT = 15.0
    MAX_TRAPPED_LOOPS = 40

    def __init__(self, app):
        self.app = app
        self.phase, self.phase_until = "", 0.0   # what happens now, until when (laptop time)
        self.state = "idle"     # idle / running / paused / stopping / done / aborted
        self.log = collections.deque(maxlen=200)
        self.results = {}
        self.cfg = None
        self.run_no = 0
        self.total = 0
        self.pause_req = False
        self.stop_req = False
        self.best = None
        self.thread = None

    def say(self, text):
        self.log.append(f"{now_stamp()} {text}")
        self.app.link.event(f"# speed test: {text}", write=True)

    def start(self, speeds, repeat, duration, pause_s):
        if self.state in ("running", "paused", "stopping"):
            return "already running"
        if not speeds or any(not 20 <= s <= 100 for s in speeds):
            return "speeds must be 20..100"
        if not 15 <= duration <= 120:
            return "duration 15..120 s"
        if not 1 <= repeat <= 5 or len(speeds) * repeat > 20:
            return "max 20 runs (speeds x repeat)"
        self.cfg = {"speeds": speeds, "repeat": repeat, "duration": duration, "pause": pause_s}
        self.results = {s: [] for s in speeds}
        self.log.clear()
        self.run_no, self.total = 0, len(speeds) * repeat
        self.pause_req = self.stop_req = False
        self.best = None
        self.state = "running"
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return None

    def _sleep(self, s):
        end = time.time() + s
        while time.time() < end and not self.stop_req:
            time.sleep(0.2)

    def _run(self):
        link = self.app.link
        orig = None
        aborted = None
        try:
            p = link.ask("params", "params:", timeout=1.0)
            if not p:
                aborted = "robot not answering (Wi-Fi?)"
                return
            m = re.search(r"speed=([\d.]+)", p)
            orig = float(m.group(1)) if m else None
            order = [s for _ in range(self.cfg["repeat"]) for s in self.cfg["speeds"]]
            self.say(f"{len(order)} runs x {self.cfg['duration']} s, speeds {self.cfg['speeds']} (your speed now: {orig})")
            for n, speed in enumerate(order, 1):
                self.run_no = n
                if self.stop_req:
                    aborted = "stopped by you"
                    break
                if link.driving():
                    self.phase, self.phase_until = "waiting for the robot to stop", 0
                    self.say("waiting for the robot to stop ...")
                    while link.driving() and not self.stop_req:
                        time.sleep(0.3)
                r = link.ask(f"set speed {speed:g}", ("params:", "Not while"))
                if not r or "Not while" in r:
                    aborted = f"set speed failed ({r or 'no reply'}) - main switch OFF?"
                    break
                before = link.csv_path
                r = link.ask(f"start {self.cfg['duration']}", ("starting", "Already ON", "refusing"))
                if not r or "refusing" in r:
                    aborted = f"start refused: {r or 'no reply'}"
                    break
                self.say(f"run {n}/{len(order)}: speed {speed:g} - 5 s countdown, then {self.cfg['duration']} s")
                t_go = time.time() + 5
                self.phase, self.phase_until = f"countdown – speed {speed:g}", t_go
                end = t_go + self.cfg["duration"] + 3
                while time.time() < t_go and not self.stop_req:
                    time.sleep(0.2)
                self.phase, self.phase_until = f"driving at speed {speed:g}", end - 3
                while time.time() < end and not self.stop_req:
                    time.sleep(0.3)
                link.ask("stop", "stopped")
                if self.stop_req:
                    aborted = "stopped by you"
                    break
                time.sleep(1.0)
                path = link.csv_path
                with link.lock:
                    rows = link.finished.get(path) or (link.rows if path != before else [])
                if path == before or not rows:
                    aborted = f"run {n}: nothing recorded (Wi-Fi link?)"
                    break
                t0 = rows[0]["t_ms"]
                rows = [r for r in rows if (r["t_ms"] - t0) / 1000 >= self.SKIP_START_S]
                st = analyze_run.stats(rows) if len(rows) > 10 else None
                if not st or st["dur"] < 0.8 * (self.cfg["duration"] - self.SKIP_START_S):
                    aborted = f"run {n}: only {st['dur'] if st else 0:.0f} s recorded (link lost? robot stopped itself)"
                    break
                res = {k: (None if isinstance(v, float) and math.isnan(v) else round(v, 3))
                       for k, v in st.items() if k in ("laps_per_min", "lap_median", "rev_count", "trapped",
                                                        "near_pct", "rev_pct", "speed_mean", "laps", "dur")}
                res["csv"] = path
                self.results[speed].append(res)
                self.say(f"run {n}: speed {speed:g} -> {res['laps_per_min'] or 0:.1f} laps/min, "
                         f"lap {res['lap_median'] or 0:.2f} s, reverses {res['rev_count']:.0f}")
                if st["rev_pct"] > self.MAX_REVERSING_PCT or st["trapped"] > self.MAX_TRAPPED_LOOPS:
                    aborted = f"run {n}: robot probably stuck / fallen over (reversing {st['rev_pct']:.0f}%)"
                    break
                if n == len(order):
                    break
                if self.pause_req:
                    self.state = "paused"
                    self.phase, self.phase_until = "paused – press Continue when the robot is back on the track", 0
                    self.say("PAUSED - swap the battery / fix the robot, put it on the track, then Continue")
                    while self.pause_req and not self.stop_req:
                        time.sleep(0.3)
                    if self.stop_req:
                        aborted = "stopped by you"
                        break
                    self.state = "running"
                    self.say("continuing in 5 s")
                    self._sleep(5)
                else:
                    self.phase, self.phase_until = "short break before the next speed", time.time() + self.cfg["pause"]
                    self._sleep(self.cfg["pause"])
        except Exception as e:  # never leave the robot driving
            aborted = f"error: {e}"
        finally:
            link.send("stop")
            if orig is not None:
                link.ask(f"set speed {orig:g}", "params:")
            self._finish(aborted, orig)

    def summary(self):
        rows = []
        for s, rs in self.results.items():
            vals = [r["laps_per_min"] for r in rs if r["laps_per_min"] is not None]
            laps = [r["lap_median"] for r in rs if r["lap_median"] is not None]
            rows.append({"speed": s, "runs": len(rs),
                         "laps_per_min": round(sum(vals) / len(vals), 2) if vals else None,
                         "lap_s": round(sum(laps) / len(laps), 2) if laps else None,
                         "reverses": round(sum(r["rev_count"] for r in rs) / len(rs), 1) if rs else None,
                         "trapped": round(sum(r["trapped"] for r in rs) / len(rs), 1) if rs else None,
                         "near_pct": round(sum(r["near_pct"] for r in rs) / len(rs), 1) if rs else None})
        return rows

    def _finish(self, aborted, orig):
        self.phase, self.phase_until = "", 0
        rows = self.summary()
        scored = [r for r in rows if r["laps_per_min"]]
        self.best = max(scored, key=lambda r: r["laps_per_min"])["speed"] if scored else None
        self.state = "aborted" if aborted else "done"
        if aborted:
            self.say("ABORTED - " + aborted)
        self.say(f"finished. best: {self.best}" if self.best else "finished, no lap data")
        if any(r["runs"] for r in rows):
            app = self.app
            with app.track_lock:
                t = tm.load_track(app.track)
                t["speedtests"].append({"when": time.strftime("%Y-%m-%d %H:%M"), "robot": app.robot,
                                        "cfg": self.cfg, "results": rows, "best": self.best,
                                        "aborted": aborted, "speed_before": orig})
                if self.best and not aborted:
                    t.setdefault("best_speed", {})
                    if not isinstance(t["best_speed"], dict):
                        t["best_speed"] = {}
                    t["best_speed"][app.robot] = self.best
                tm.save_track(t)
            app.analysis_dirty = True

    def to_json(self):
        return {"state": self.state, "run": self.run_no, "total": self.total, "cfg": self.cfg,
                "phase": self.phase, "phase_left": max(0, round(self.phase_until - time.time())) if self.phase_until else None,
                "log": list(self.log)[-40:], "results": self.summary() if self.cfg else [], "best": self.best,
                "pause_req": self.pause_req}


# =============================================================================== app
class App:
    def __init__(self, robot):
        self.robot = robot
        self.robots = load_robots()
        self.link = Link()
        self.track = "default"
        self.track_lock = threading.Lock()
        self.analysis = None
        self.analysis_ver = 0
        self.analysis_dirty = True
        self.live_profile = None
        self.live_key = None
        self.speedtest = SpeedTest(self)
        self.robot_track_seen = None
        self.placement = None       # where you put the robot for the NEXT drive (tile + direction)
        self.run_start = {}         # run file -> placement used for that drive
        self.test = None            # the test drive running now (dict)
        self.last_test = None       # summary of the last one
        self.ww_want = None         # what you last said: True wrong way / False right way
        self.ww_sent = 0.0
        threading.Thread(target=self._analysis_loop, daemon=True).start()

    def geom(self):
        return self.robots.get(self.robot, {}).get("geometry")

    def calib(self):
        return self.robots.get(self.robot, {}).get("turn_calib")

    def set_track(self, name):
        name = tm.safe_name(name)
        if name != self.track:
            self.track = name
            self.live_profile = None
            self.analysis_dirty = True
            self.link.event(f"# dashboard: track '{name}'", write=True)

    def _analysis_loop(self):
        last_ver = -1
        while True:
            time.sleep(2.0)
            # the robot's own idea of the track wins (it learns per track)
            rt = self.link.status.get("trk")
            if rt and rt != self.robot_track_seen:
                self.robot_track_seen = rt
                self.set_track(rt)
            ver = self.link.rows_ver
            key = self.link.csv_path
            if key and key not in self.run_start:      # a new drive started
                self.run_start[key] = self.placement
                if self.placement:
                    self.link.event(f"# drive starts where you placed the robot: tile ({self.placement['x']},"
                                    f"{self.placement['y']}) facing {tm.SIDE_NAME[self.placement['dir']]}")
                self.placement = None                 # used up: mark it again for the next drive
                self.ww_want = None                   # the robot starts every drive "right way"
            self._test_tick()
            # the robot did not get your wrong-way / right-way button (Wi-Fi gap): send again
            st = self.link.status
            if (self.ww_want is not None and self.link.driving() and "ww" in st
                    and (st["ww"] == "1") != self.ww_want and time.time() - self.ww_sent > 2.5):
                self.ww_sent = time.time()
                self.link.send("mark wrong" if self.ww_want else "mark ok")
                self.link.event("# robot missed your wrong-way button - sent again", write=True)
            try:
                if ver != last_ver:
                    last_ver = ver
                    with self.link.lock:
                        rows = list(self.link.rows)
                        key = self.link.csv_path
                    if len(rows) > 200:
                        p = tm.run_profile(rows)
                        if p:
                            p["robot"] = self.robot
                            if self.run_start.get(key):
                                p["start"] = {k: self.run_start[key][k] for k in ("x", "y", "dir")}
                            self.live_profile, self.live_key = p, os.path.basename(key)
                            with self.track_lock:
                                t = tm.load_track(self.track)
                                t["runs"][self.live_key] = p
                                if len(t["runs"]) > 30:   # keep the newest 30 drives
                                    for k in sorted(t["runs"], key=lambda k: t["runs"][k].get("when", ""))[:-30]:
                                        del t["runs"][k]
                                tm.save_track(t)
                            self.analysis_dirty = True
                if self.analysis_dirty:
                    self.analysis_dirty = False
                    with self.track_lock:
                        t = tm.load_track(self.track)
                    a = tm.analyse(t, self.robot, self.geom(), calib=self.calib())
                    m = a.get("match")
                    p = a.get("profile")
                    if (m and m.get("calib") and m["quality"] >= tm.CALIB_MIN_QUALITY and p
                            and p["laps"] >= tm.CALIB_MIN_LAPS):
                        old = self.calib() or {}
                        better = (not old or old.get("track") == t["name"]
                                  or m["quality"] > old.get("quality", 0))
                        if better and abs(old.get("offset", 99) - m["calib"]["offset"]) > 1e-4:
                            c = dict(m["calib"], track=t["name"], when=time.strftime("%Y-%m-%d %H:%M"))
                            save_robot_field(self.robot, "turn_calib", c)
                            self.robots = load_robots()
                    a["calib"] = self.calib()
                    self.analysis = a
                    self.analysis_ver += 1
            except Exception as e:      # keep the dashboard alive, show the problem
                self.link.event(f"# dashboard analysis error: {e!r}", write=False)

    def live_position(self):
        """Where the robot is in the lap now: its own estimate (timing, what the speed map
        uses), the sensor match, and the laptop's timing estimate. Fractions 0..1 or None."""
        with self.link.lock:
            row = self.link.rows[-1] if self.link.rows else None
            allrows = self.link.rows
            t_lap = None
            for i in range(len(allrows) - 1, max(0, len(allrows) - 800), -1):
                if allrows[i]["lap"] > allrows[i - 1]["lap"]:
                    t_lap = allrows[i]["t_ms"]
                    break
        if not row or not self.link.driving() or time.time() - self.link.last_rx > 2:
            return {"why": "not driving" if row else "no data"}
        out = {"age": round(time.time() - self.link.last_rx, 2)}
        if row.get("ph", -1) >= 0:
            out["robot"] = row["ph"] / 1000
        if row.get("wpos", -1) >= 0:
            out["sensors"] = row["wpos"] / 1000
        lap_s = (self.live_profile or {}).get("lap_s") or (
            self.analysis and self.analysis.get("profile") and self.analysis["profile"]["lap_s"])
        if lap_s and t_lap is not None:
            f = (row["t_ms"] - t_lap) / 1000 / lap_s
            if 0 <= f <= 1.2:
                out["laptop"] = round(min(f, 0.999), 3)
        out["lap_s"] = lap_s
        if "robot" not in out:
            out["why"] = ("robot: lap mark not found yet / laps not regular" if "ph" in row
                          else "old firmware: no position from the robot")
        return out

    # ------------------------------------------------------------------ test drives
    def test_start(self, mode, minutes, note):
        L = self.link
        if self.test:
            return "a test is already running - End it first"
        if not 0.5 <= minutes <= 30:
            return "duration 0.5 .. 30 min"
        if mode == "laptop":
            st = L.status
            if not (L.connected and time.time() - L.status_at < 3):
                return "robot not connected (Wi-Fi)"
            if L.driving() or st.get("sw") == "1":
                return "the robot is already driving / main switch ON - stop it first"
            r = L.ask(f"start {int(minutes * 60)} free", ("starting", "Already ON", "refusing", "start 1.."))
            if not r or "refusing" in r or "start 1.." in r:
                return f"robot refused: {r or 'no reply'}"
        self.test = {"id": time.strftime("%Y-%m-%d_%H%M%S"), "start": time.time(), "mode": mode,
                     "minutes": minutes, "note": note[:200], "track": self.track, "robot": self.robot,
                     "drive_start": None, "runs": []}
        L.mark(f"TEST START ({'laptop starts it' if mode == 'laptop' else 'start with app / button'}, "
               f"{minutes:g} min){': ' + note if note else ''}", "mark test start")
        return None

    def _test_tick(self):
        t = self.test
        if not t:
            return
        L = self.link
        if L.csv_path and L.csv_path not in t["runs"] and L.run_started >= t["start"] - 2:
            t["runs"].append(L.csv_path)
        if t["drive_start"] is None and L.driving():
            t["drive_start"] = time.time()
        if t["drive_start"] and time.time() >= t["drive_start"] + t["minutes"] * 60:
            L.send("stop")            # time is up (the robot also stops itself if the laptop started it)
            self.test_end("time is up")

    def test_end(self, why):
        t = self.test
        if not t:
            return None
        self.test = None
        L = self.link
        L.send("stop")
        t["end"] = time.time()
        t["ended"] = why
        with L.lock:
            laps = [dict(l) for l in L.laps if t["start"] <= l["at"] <= t["end"]]
            marks = [(round(a - t["start"], 1), txt) for a, _, txt in L.marks if t["start"] <= a <= t["end"]]
        times = sorted(l["s"] for l in laps)
        drove = (t["end"] - t["drive_start"]) if t["drive_start"] else 0
        t["summary"] = {
            "laps": len(laps), "best": times[0] if times else None,
            "median": times[len(times) // 2] if times else None,
            "laps_per_min": round(len(laps) / (drove / 60), 2) if drove > 30 else None,
            "drove_s": round(drove), "reverses": sum(l["rev"] for l in laps),
            "trapped": sum(l["trap"] for l in laps), "not_moving": sum(l["notmov"] for l in laps),
            "margin_first": next((l["margin"] for l in laps if l.get("margin") is not None), None),
            "margin_last": next((l["margin"] for l in reversed(laps) if l.get("margin") is not None), None),
            "marks": marks, "lap_times": [l["s"] for l in laps],
        }
        L.mark(f"TEST END ({why}): {len(laps)} laps" + (f", best {times[0]:.2f} s" if times else ""), "mark test end")
        with open(f"crashlogs/test_{t['id']}_{t['robot']}.json", "w") as f:
            json.dump(t, f, indent=1)
        with self.track_lock:
            tr = tm.load_track(t["track"])
            tr.setdefault("tests", []).append({k: t[k] for k in ("id", "robot", "mode", "minutes", "note", "ended", "summary")})
            tr["tests"] = tr["tests"][-40:]
            tm.save_track(tr)
        self.last_test = t
        self.analysis_dirty = True
        return t

    def test_json(self):
        t = self.test
        if t:
            now = time.time()
            with self.link.lock:
                laps = [l for l in self.link.laps if l["at"] >= t["start"]]
            left = (t["drive_start"] + t["minutes"] * 60 - now) if t["drive_start"] else None
            return {"running": True, "mode": t["mode"], "minutes": t["minutes"], "note": t["note"],
                    "waiting": t["drive_start"] is None, "elapsed": round(now - (t["drive_start"] or now)),
                    "left": round(left) if left is not None else None, "laps": len(laps),
                    "best": min((l["s"] for l in laps), default=None)}
        return {"running": False, "last": self.last_test and {k: self.last_test[k] for k in
                                                              ("id", "mode", "minutes", "note", "ended", "summary")}}

    # ------------------------------------------------------------------ robot's diary
    def read_diary(self):
        L = self.link
        if L.driving():
            return {"error": "stop the robot first"}
        L.capture = []
        try:
            L.send("rounds")
            end = time.time() + 25
            last_n, quiet_since = 0, time.time()
            while time.time() < end:
                time.sleep(0.3)
                lines = L.capture
                if any("end of rounds" in x for x in lines):
                    break
                if len(lines) != last_n:
                    last_n, quiet_since = len(lines), time.time()
                elif lines and time.time() - quiet_since > 4:
                    break
            lines = list(L.capture)
        finally:
            L.capture = None
        if not lines:
            return {"error": "no answer (robot connected? new firmware?)"}
        path = time.strftime(f"crashlogs/diary_{self.robot}_%Y-%m-%d_%H%M%S.txt")
        with open(path, "w") as f:
            f.write("\n".join(lines) + "\n")
        complete = any("end of rounds" in x for x in lines)
        return {"ok": True, "path": path, "complete": complete, "drives": parse_diary(lines)}

    def ww_report(self):
        """Your wrong-way marks vs the robot's own detection (this session)."""
        L = self.link
        with L.lock:
            marks, det = list(L.ww_marks), list(L.ww_events)
        periods, start = [], None
        for t, wrong in marks:
            if wrong and start is None:
                start = t
            elif not wrong and start is not None:
                periods.append((start, t))
                start = None
        if start is not None:
            periods.append((start, time.time()))
        starts = [t for t, d in det if d]
        hits, delays = 0, []
        for a, b in periods:
            ds = [t for t in starts if a - 3 <= t <= b]
            if ds:
                hits += 1
                delays.append(max(0.0, ds[0] - a))
        false = sum(1 for t in starts if not any(a - 3 <= t <= b + 3 for a, b in periods))
        return {"periods": len(periods), "detected": hits, "false_alarms": false,
                "delay": round(sum(delays) / len(delays), 1) if delays else None,
                "series": [list(x) for x in list(L.series)[-450:]],
                "marks": [[round(a, 1), round(b, 1)] for a, b in periods],
                "detections": [[round(t, 1), d] for t, d in det[-40:]]}

    def state(self, since):
        L = self.link
        with L.lock:
            ev = [e for e in L.events if e[0] > since]
            st = dict(L.status)
            laps = list(L.laps)[-60:]
            row = dict(L.rows[-1]) if L.rows else None
            rows_n = len(L.rows)
        age = time.time() - L.last_rx if L.last_rx else None
        return {
            "robot": self.robot, "robot_name": self.robots.get(self.robot, {}).get("name", self.robot),
            "ssid": self.robots.get(self.robot, {}).get("ssid"),
            "link": "live" if age is not None and age < 3 else ("lost" if L.last_rx else "waiting"),
            "age": round(age, 1) if age is not None else None,
            "status": st, "driving": L.driving(), "row": row,
            "events": ev[-300:], "laps": laps, "track": self.track,
            "recording": {"path": L.csv_path, "rows": rows_n}, "log_path": L.log_path,
            "pos": self.live_position(), "analysis_ver": self.analysis_ver,
            "speedtest": self.speedtest.to_json(), "tracks": tm.list_tracks(),
            "geometry": self.geom() or {}, "calib": self.calib(), "placement": self.placement,
            "ww": self.ww_report(), "now": time.time(), "test": self.test_json(),
        }


def parse_diary(lines):
    """Robot diary lines -> drives: [{when, track, laps:[...], best, median, events:[...]}]"""
    drives, cur = [], None
    for line in lines:
        m = re.match(r"(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d|boot#\d+ \+[\d.]+s) (.*)", line)
        if not m:
            continue
        when, msg = m.groups()
        if msg.startswith("driving started"):
            tr = re.search(r"track '([^']*)'", msg)
            cur = {"when": when, "track": tr.group(1) if tr else "?", "laps": [], "events": []}
            drives.append(cur)
            continue
        if cur is None:
            continue
        lm = re.match(r"LAPSUM (\d+) ([\d.]+)s rev(\d+) trap(\d+) notmov(\d+) blind(\d+) minwall(-?\d+)cm "
                      r"lock(\d+)% minspd(\d+)% spd(\d+)us(?: margin(\d+)cm)?", msg)
        if lm:
            g = lm.groups()
            cur["laps"].append({"lap": int(g[0]), "s": float(g[1]), "rev": int(g[2]), "trap": int(g[3]),
                                "notmov": int(g[4]), "spd": int(g[9]), "margin": int(g[10]) if g[10] else None})
        elif msg.startswith("driving stopped"):
            cur["end"] = when
            cur = None
        elif not msg.startswith(("laps this run", "lap mark")):
            cur["events"].append(f"{when[-8:]} {msg}")
    for d in drives:
        ts = sorted(l["s"] for l in d["laps"])
        d["best"] = ts[0] if ts else None
        d["median"] = ts[len(ts) // 2] if ts else None
        d["trouble"] = sum(l["rev"] + l["trap"] + l["notmov"] for l in d["laps"])
        d["events"] = d["events"][-30:]
    return drives


# =============================================================================== http
APP = None
PAGE = os.path.join(HERE, "dashboard.html")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path, _, q = self.path.partition("?")
        if path in ("/", "/index.html"):
            body = open(PAGE, "rb").read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif path == "/api/state":
            m = re.search(r"since=(\d+)", q)
            self._json(APP.state(int(m.group(1)) if m else 0))
        elif path == "/api/track":
            self._json({"ver": APP.analysis_ver, "analysis": APP.analysis})
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        try:
            d = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return self._json({"error": "bad json"}, 400)
        try:
            self._json(self.route(self.path, d))
        except Exception as e:
            self._json({"error": repr(e)}, 500)

    def route(self, path, d):
        L = APP.link
        if path == "/api/cmd":
            cmd = str(d.get("cmd", "")).strip()[:31]
            if cmd:
                L.event(f"> {cmd}")
                L.send(cmd)
            return {"ok": True}
        if path == "/api/mark":
            code = d.get("code")
            if code in MARKS:
                text, cmd = MARKS[code]
                if code in ("wrong", "ok"):
                    APP.ww_want = code == "wrong"
                    APP.ww_sent = time.time()
                    with L.lock:
                        L.ww_marks.append((time.time(), code == "wrong"))
            else:
                text = re.sub(r"\s+", " ", str(d.get("text", ""))).strip()[:60]
                if not text:
                    return {"error": "empty mark"}
                cmd = "mark " + text[:26]
            L.mark(text, cmd)
            return {"ok": True}
        if path == "/api/track/select":
            name = tm.safe_name(d.get("name"))
            if L.driving():
                return {"error": "stop the robot first (the robot switches its speed map)"}
            r = L.ask(f"track {name}", ("track:", "not while"), timeout=1.5)
            APP.robot_track_seen = name
            APP.set_track(name)
            t = tm.load_track(name)
            if not os.path.exists(tm.track_path(name)):
                tm.save_track(t)
            return {"ok": True, "robot": r or "robot did not answer (offline?) - it will be told next time"}
        if path == "/api/track/save":
            with APP.track_lock:
                t = tm.load_track(APP.track)
                if "layout" in d:
                    t["layout"] = d["layout"]
                if "notes" in d:
                    t["notes"] = str(d["notes"])[:4000]
                tm.save_track(t)
            APP.analysis_dirty = True
            return {"ok": True}
        if path == "/api/test/start":
            try:
                err = APP.test_start(d.get("mode", "app"), float(d.get("minutes", 5)), str(d.get("note", "")))
            except ValueError:
                err = "minutes?"
            return {"error": err} if err else {"ok": True}
        if path == "/api/test/end":
            t = APP.test_end("you pressed End")
            return {"ok": True, "summary": t and t["summary"]}
        if path == "/api/diary/read":
            return APP.read_diary()
        if path == "/api/diary/clear":
            if L.driving():
                return {"error": "stop the robot first"}
            r = L.ask("rounds clear", "rounds cleared", timeout=2)
            return {"ok": bool(r)}
        if path == "/api/placement":
            if d.get("clear"):
                APP.placement = None
                return {"ok": True}
            pl = {"x": int(d["x"]), "y": int(d["y"]), "dir": int(d["dir"]) % 4, "when": time.time()}
            APP.placement = pl
            side = tm.SIDE_NAME[pl["dir"]]
            L.mark(f"robot placed at tile ({pl['x']},{pl['y']}) facing {side}", f"mark placed {pl['x']},{pl['y']} {side}")
            with APP.track_lock:
                t = tm.load_track(APP.track)
                if t.get("layout") and t["layout"].get("tiles"):
                    t["layout"]["placement"] = {k: pl[k] for k in ("x", "y", "dir")}
                    tm.save_track(t)
            return {"ok": True}
        if path == "/api/layout/preview":
            lay_in = d.get("layout") or {}
            if lay_in.get("tiles"):
                tr = tm.trace_tiles(lay_in["tiles"], lay_in.get("placement"))
                if not tr["closed"]:
                    return {"ok": False, "error": tr["error"], "tiles": tr["tiles"]}
            lay = tm.layout_points(lay_in)
            if not lay:
                return {"ok": False}
            return {"ok": True, "xy": lay["xy"], "hills": lay["hills"], "turns": tm.turns(tm.heading_changes(lay["up"])),
                    "tiles": lay.get("tiles"), "length_tiles": lay.get("length_tiles")}
        if path == "/api/robot/geometry":
            g = {k: float(d[k]) for k in ("wheelbase_cm", "turn_radius_cm", "length_cm", "width_cm")
                 if d.get(k) not in (None, "")}
            save_robot_field(APP.robot, "geometry", g)
            APP.robots = load_robots()
            APP.analysis_dirty = True
            return {"ok": True}
        if path == "/api/speedtest/start":
            try:
                speeds = [float(x) for x in str(d.get("speeds", "")).replace(" ", "").split(",") if x]
                err = APP.speedtest.start(speeds, int(d.get("repeat", 2)), int(d.get("duration", 40)),
                                          int(d.get("pause", 10)))
            except ValueError:
                err = "speeds like 50,55,60"
            return {"error": err} if err else {"ok": True}
        if path == "/api/speedtest/pause":
            APP.speedtest.pause_req = True
            APP.speedtest.say("pause requested: waits after this run")
            return {"ok": True}
        if path == "/api/speedtest/resume":
            APP.speedtest.pause_req = False
            return {"ok": True}
        if path == "/api/speedtest/stop":
            APP.speedtest.stop_req = True
            L.send("stop")
            return {"ok": True}
        if path == "/api/speed/apply":
            if L.driving():
                return {"error": "stop the robot first"}
            sp = float(d["speed"])
            r1 = L.ask(f"set speed {sp:g}", ("params:", "Not while"))
            r2 = L.ask("save", ("params:", "Not while")) if r1 and "params:" in r1 else None
            ok = bool(r2 and "params:" in r2)
            return {"ok": ok, "reply": r2 or r1 or "no reply",
                    "note": "Saved on the robot. If the phone app is connected, set the same speed there too "
                            "(the app slider would overwrite it)."}
        return {"error": "unknown"}


def main():
    global APP
    robots = [k for k in load_robots() if not k.startswith("_")]
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", choices=robots, default="shadow")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--no-open", action="store_true")
    a = ap.parse_args()
    APP = App(a.robot)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    url = f"http://localhost:{a.port}"
    print(f"dashboard for {a.robot}: {url}   (log: {APP.link.log_path})")
    print(f"laptop must be on the robot's Wi-Fi ({APP.robots[a.robot].get('ssid')}). Ctrl+C to quit.", flush=True)
    if not a.no_open:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        APP.link.send("stop") if APP.speedtest.state in ("running", "paused") else None
        APP.link._close_run()
        print(f"\nsaved: {APP.link.log_path}")


if __name__ == "__main__":
    main()
