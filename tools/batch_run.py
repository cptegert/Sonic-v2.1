#!/usr/bin/env python3
"""Several automated runs in one go, settings interleaved, with a summary table.

  ~/.platformio/penv/bin/python ~/Projects/Sonic-v2.1/tools/batch_run.py \
      --config "kp=6,kd=7.5" --config "kp=6,kd=9" --repeat 3 --duration 40

Each --config is one setting (kp, ki, kd, speed; omitted = unchanged). Runs are
interleaved (A B A B ...) so battery / tyre drift hits all settings equally.
Every run uses tools/auto_run.py (same safety: the robot stops by itself after
--duration, or within 1 s if the link is lost).

The batch ABORTS when a run looks wrong (so nothing keeps driving blindly):
  - auto_run failed (robot not answering, start refused, ...)
  - run much shorter than requested (link lost -> robot stopped itself)
  - lots of reversing / TRAPPED (robot stuck or fallen over)
The laptop must already be on the robot's Wi-Fi with internet another way
(USB tethering); auto_run rejoins the robot's Wi-Fi if needed.
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_run  # noqa: E402

SKIP_START_S = 2.0          # ignore the first seconds (start from standstill)
MAX_REVERSING_PCT = 15.0    # more = probably stuck / fallen over -> abort
MAX_TRAPPED_LOOPS = 40
MIN_LENGTH_FRACTION = 0.8   # run shorter than 80% of --duration -> abort
MAX_RUNS = 20


def parse_config(text):
    cfg = {}
    for part in text.split(","):
        k, _, v = part.partition("=")
        k = k.strip()
        if k not in ("kp", "ki", "kd", "speed"):
            sys.exit(f"unknown setting '{k}' in --config {text!r} (use kp, ki, kd, speed)")
        cfg[k] = float(v)
    return cfg


def label(cfg):
    return " ".join(f"{k}={v:g}" for k, v in cfg.items()) or "(unchanged)"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", action="append", required=True)
    ap.add_argument("--repeat", type=int, default=2)
    ap.add_argument("--duration", type=int, default=40)
    ap.add_argument("--pause", type=int, default=15, help="seconds between runs")
    with open(os.path.join(HERE, "robots.json")) as f:
        known = [k for k in json.load(f) if not k.startswith("_")]
    ap.add_argument("--robot", choices=known + ["auto"], default="auto")
    a = ap.parse_args()
    configs = [parse_config(c) for c in a.config]
    order = [c for _ in range(a.repeat) for c in configs]
    if len(order) > MAX_RUNS:
        sys.exit(f"{len(order)} runs requested, max {MAX_RUNS}")
    total_min = len(order) * (a.duration + 5 + a.pause + 10) / 60
    print(f"batch: {len(order)} runs x {a.duration} s, about {total_min:.0f} min", flush=True)

    results = {i: [] for i in range(len(configs))}
    aborted = None
    for n, cfg in enumerate(order, 1):
        idx = configs.index(cfg)
        print(f"\n=== run {n}/{len(order)}: {label(cfg)} ===", flush=True)
        cmd = [sys.executable, os.path.join(HERE, "auto_run.py"), "--no-wifi-switch", "--robot", a.robot,
               "--duration", str(a.duration)]
        for k, v in cfg.items():
            cmd += ["--" + k, str(v)]
        p = subprocess.run(cmd, capture_output=True, text=True)
        out = p.stdout + p.stderr
        m = re.search(r"recorded (\d+) rows -> (\S+\.csv)", out)
        if p.returncode != 0 or not m:
            tail = "\n".join(out.splitlines()[-12:])
            aborted = f"run {n}: auto_run failed\n{tail}"
            break
        path = m.group(2)
        rows = analyze_run.load(path)
        if rows:
            t0 = rows[0]["t_ms"]
            rows = [r for r in rows if (r["t_ms"] - t0) / 1000 >= SKIP_START_S]
        st = analyze_run.stats(rows) if len(rows) > 10 else None
        if not st or st["dur"] < MIN_LENGTH_FRACTION * (a.duration - SKIP_START_S):
            aborted = f"run {n}: only {st['dur'] if st else 0:.0f} s recorded (link lost? robot stopped itself)"
            break
        print(f"   {path}: near-wall {st['near_pct']:.1f}%  full-lock {st['sat_pct']:.1f}%  "
              f"swings {st['osc_per_s']:.2f}/s  D-jumps {st['d_jump_rms']:.0f}us  "
              f"reversing {st['rev_pct']:.1f}%  trapped {st['trapped']:.0f}  lap {st['lap_s']:.2f}s"
              + (f"  battery {rows[-1]['batt_v']:.2f} V" if rows[-1].get("batt_v", 0) > 5 else ""), flush=True)
        if rows[-1].get("batt_v", 99) < 10.5 and rows[-1].get("batt_v", 0) > 5:
            aborted = f"run {n}: battery low ({rows[-1]['batt_v']:.2f} V) - charge it"
            results[idx].append(st)
            break
        results[idx].append(st)
        if st["rev_pct"] > MAX_REVERSING_PCT or st["trapped"] > MAX_TRAPPED_LOOPS:
            aborted = f"run {n}: robot probably stuck or fallen over (reversing {st['rev_pct']:.0f}%, trapped {st['trapped']:.0f})"
            break
        if n < len(order):
            time.sleep(a.pause)

    print("\n" + "=" * 100)
    if aborted:
        print("BATCH ABORTED - " + aborted)
        print("=" * 100)
    keys = [("lap_s", "lap s", "{:.2f}"), ("near_pct", "near wall %", "{:.1f}"), ("sat_pct", "full lock %", "{:.1f}"),
            ("osc_per_s", "swings/s", "{:.2f}"), ("d_jump_rms", "D-jump us", "{:.0f}"),
            ("rev_count", "reverses", "{:.1f}"), ("speed_mean", "motor us", "{:.0f}")]
    print(f"{'setting':28s} {'runs':>4s}  " + "  ".join(f"{h:>18s}" for _, h, _ in keys))
    for i, cfg in enumerate(configs):
        rs = results[i]
        if not rs:
            print(f"{label(cfg):28s} {0:4d}")
            continue
        cells = []
        for k, _, fmt in keys:
            vals = [r[k] for r in rs]
            mean = sum(vals) / len(vals)
            spread = f" ({fmt.format(min(vals))}-{fmt.format(max(vals))})" if len(vals) > 1 else ""
            cells.append(f"{fmt.format(mean)}{spread}".rjust(18))
        print(f"{label(cfg):28s} {len(rs):4d}  " + "  ".join(cells))
    print("(mean, with min-max over the repeats; first 2 s of each run ignored)")
    return 1 if aborted else 0


if __name__ == "__main__":
    sys.exit(main())
