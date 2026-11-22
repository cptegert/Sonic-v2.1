#!/usr/bin/env python3
"""Per-lap report and track profile from a telemetry recording.

  ~/.platformio/penv/bin/python ~/Projects/Sonic-v2.1/tools/lap_report.py [run.csv] [--bins 20]
  (no file = newest crashlogs/run_*.csv)

1. LAPS: one row per complete lap - lap time, reverses, TRAPPED, closest wall,
   share of time at full steering lock / minimum speed / near a wall, average
   speed command, and your marks (!3, !4 ... from the viewer) in that lap.
2. TRACK PROFILE: every lap is cut into N equal parts (the same place on the
   track each lap). Per part, averaged over all laps: speed, steering, closest
   wall, % of laps at full lock / near a wall / reversing. Recurring trouble
   spots stand out - places where a targeted change helps.

Laps come from the robot's own lap counter (telemetry column "lap"); older
recordings without it are cut with the same fingerprint method as the robot.
"""
import argparse
import csv
import glob
import math
import os
import re
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CRASHLOGS = os.path.join(HERE, "..", "crashlogs")
FULL_LOCK_US = 615          # |steer - 1500| at full lock (max 625)
NEAR_WALL_CM = 15
SENSORS = ["FFL", "FL", "L", "M", "R", "FR", "FFR"]


def load(path):
    rows = []
    for r in csv.DictReader(open(path)):
        try:
            rows.append({k: float(v) for k, v in r.items() if k is not None})
        except (TypeError, ValueError):
            pass
    return rows


# --- same fingerprint lap detection as src/lap_detect.cpp (for old recordings) ---
def detect_laps(rows, dt=100, w=25, settle=4000, th=0.70, min_lap=2500, relearn=20000, min_std=4.0):
    def ncc(a, b):
        ma, mb = sum(a) / len(a), sum(b) / len(b)
        sa = math.sqrt(sum((x - ma) ** 2 for x in a))
        sb = math.sqrt(sum((x - mb) ** 2 for x in b))
        return 0 if sa < 1e-6 or sb < 1e-6 else sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (sa * sb)
    t0 = rows[0]["t_ms"]
    buf, tb, dist = [], [], []
    tmpl, last, prev, laps, nxt = None, None, [0, 0], [], t0
    for r in rows:
        if r["t_ms"] < nxt:
            continue
        nxt = r["t_ms"] + dt
        buf.append(r["right"] - r["left"]); tb.append(r["t_ms"])
        dist.append(r.get("rev", 0) or r.get("trapped", 0) or r.get("manual", 0))
        if len(buf) < w:
            continue
        if tmpl is None or tb[-1] - last > relearn:
            if r["t_ms"] - t0 >= settle and not any(dist[-w:]):
                win = buf[-w:]; m = sum(win) / w
                if math.sqrt(sum((x - m) ** 2 for x in win) / w) >= min_std:
                    tmpl, last, prev = win[:], tb[-1], [0, 0]
                    if not laps:
                        laps.append(last)  # start of the first counted lap
            if tmpl is None or tb[-1] == last:
                continue
        c = ncc(tmpl, buf[-w:])
        if prev[1] > th and prev[1] >= c and prev[1] > prev[0] and tb[-2] - last >= min_lap:
            last = tb[-2]
            laps.append(last)
        prev = [prev[1], c]
    return laps  # times (ms) of lap boundaries


def lap_boundaries(rows):
    if "lap" in rows[0]:
        b = []
        for a, r in zip(rows, rows[1:]):
            if r["lap"] > a["lap"]:
                b.append(r["t_ms"])
        return b, "robot lap counter"
    return detect_laps(rows), "fingerprint (recomputed)"


def load_marks(csv_path, rows):
    """Marks from the viewer log (laptop time) -> robot time via the status lines."""
    wifis = sorted(glob.glob(os.path.join(CRASHLOGS, "wifi_*.txt")), key=os.path.getmtime)
    csv_m = os.path.getmtime(csv_path)
    marks = []
    for w in wifis:
        if abs(os.path.getmtime(w) - csv_m) > 4 * 3600:
            continue
        mapping, ms = [], []
        for line in open(w, errors="replace"):
            m = re.match(r"(\d\d):(\d\d):(\d\d)(?:\.\d)? S #\d+ up=([\d.]+)s", line)
            if m:
                mapping.append((int(m[1]) * 3600 + int(m[2]) * 60 + int(m[3]), float(m[4]) * 1000))
            m = re.match(r"(\d\d):(\d\d):(\d\d)\.?\d? MARK (.*)", line)
            if m:
                ms.append((int(m[1]) * 3600 + int(m[2]) * 60 + int(m[3]), m[4].strip()))
        for t, text in ms:
            near = [p for p in mapping if abs(p[0] - t) <= 5]
            if near:
                lt, up = min(near, key=lambda p: abs(p[0] - t))
                robot_ms = up + (t - lt) * 1000
                if rows[0]["t_ms"] - 5000 <= robot_ms <= rows[-1]["t_ms"] + 5000:
                    marks.append((robot_ms, text))
    return marks


def lap_stats(seg):
    n = len(seg)
    fwd = [r for r in seg if not r.get("rev", 0)]
    revs = sum(1 for a, b in zip(seg, seg[1:]) if not a.get("rev", 0) and b.get("rev", 0))
    traps = sum(1 for a, b in zip(seg, seg[1:]) if not a.get("trapped", 0) and b.get("trapped", 0))
    minwall = min(min(r[k] for k in SENSORS) for r in seg)
    lock = 100 * sum(abs(r["steer_us"] - 1500) >= FULL_LOCK_US for r in fwd) / max(1, len(fwd))
    near = 100 * sum(min(r["left"], r["right"]) < NEAR_WALL_CM for r in fwd) / max(1, len(fwd))
    minspd = 100 * sum(1500 < r["speed_us"] <= 1605 for r in fwd) / max(1, len(fwd))
    spd = sum(r["speed_us"] for r in fwd) / max(1, len(fwd))
    return revs, traps, minwall, lock, near, minspd, spd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?")
    ap.add_argument("--bins", type=int, default=20)
    a = ap.parse_args()
    path = a.csv or sorted(glob.glob(os.path.join(CRASHLOGS, "run_*.csv")), key=os.path.getmtime)[-1]
    rows = load(path)
    if len(rows) < 100:
        raise SystemExit(f"{path}: too few rows")
    bounds, how = lap_boundaries(rows)
    print(f"run: {os.path.basename(path)}  ({len(rows)} rows, laps from: {how})")
    if len(bounds) < 2:
        raise SystemExit("fewer than 1 complete lap found")
    marks = load_marks(path, rows)

    laps = []
    for i, (s, e) in enumerate(zip(bounds, bounds[1:]), 1):
        seg = [r for r in rows if s <= r["t_ms"] < e]
        if len(seg) < 10:
            continue
        laps.append((i, (e - s) / 1000, seg, [t for tm, t in marks if s <= tm < e]))
    times = sorted(l[1] for l in laps)
    median = times[len(times) // 2]

    print(f"\nLAPS  (median lap {median:.2f} s; '*' = over 1.5x median: slow lap or missed lap boundary)")
    print(" lap   time  rev trap minwall  lock%  near%  minspd%  speed  marks")
    for i, t, seg, mk in laps:
        revs, traps, minwall, lock, near, minspd, spd = lap_stats(seg)
        flag = "*" if t > 1.5 * median else " "
        print(f"{i:4d} {t:6.2f}{flag} {revs:3d} {traps:4d} {minwall:6.0f}cm {lock:5.0f}  {near:5.0f}  {minspd:6.0f}  {spd:5.0f}  {'; '.join(mk)}")

    # track profile over normal laps only (skip slow / merged laps)
    normal = [l for l in laps if l[1] <= 1.3 * median]
    if len(normal) < 2:
        return
    nb = a.bins
    acc = [{"n": 0, "spd": 0, "steer": 0, "wall": 0, "lock": 0, "near": 0, "rev": 0, "laps_lock": set(),
            "laps_near": set(), "laps_rev": set()} for _ in range(nb)]
    for i, t, seg, mk in normal:
        s, e = seg[0]["t_ms"], seg[-1]["t_ms"]
        for r in seg:
            b = min(nb - 1, int((r["t_ms"] - s) / max(1, e - s) * nb))
            x = acc[b]
            x["n"] += 1
            x["spd"] += r["speed_us"]
            x["steer"] += r["steer_us"] - 1500
            x["wall"] += min(r["left"], r["right"])
            if abs(r["steer_us"] - 1500) >= FULL_LOCK_US:
                x["laps_lock"].add(i)
            if min(r["left"], r["right"]) < NEAR_WALL_CM:
                x["laps_near"].add(i)
            if r.get("rev", 0):
                x["laps_rev"].add(i)
    nl = len(normal)
    print(f"\nTRACK PROFILE  ({nl} normal laps, each cut into {nb} parts; part 1 = right after the lap mark)")
    print(" part  speed  steer(+=left)  wall   %laps: full-lock  near-wall  reversing")
    worst = []
    for b, x in enumerate(acc, 1):
        if not x["n"]:
            continue
        lock, near, rev = (100 * len(x[k]) / nl for k in ("laps_lock", "laps_near", "laps_rev"))
        score = lock + 2 * near + 3 * rev
        worst.append((score, b))
        print(f" {b:4d}  {x['spd'] / x['n']:5.0f}  {x['steer'] / x['n']:+8.0f}     {x['wall'] / x['n']:5.0f}cm"
              f"     {lock:5.0f}      {near:5.0f}      {rev:5.0f}")
    worst.sort(reverse=True)
    spots = [f"part {b}" for s, b in worst[:3] if s > 30]
    print("\nTrouble spots (recurring every lap): " + (", ".join(spots) if spots else "none stand out"))


if __name__ == "__main__":
    main()
