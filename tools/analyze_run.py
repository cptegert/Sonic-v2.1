#!/usr/bin/env python3
"""Summarise a PID telemetry run recorded by tools/wifi_log.py.

Usage:
  ~/.platformio/penv/bin/python ~/Projects/Sonic-v2.1/tools/analyze_run.py crashlogs/run_....csv
  (no argument = newest run_*.csv in crashlogs/)

Prints: data quality, controller settings, steering error, oscillation,
saturation, P vs D activity, reversing / near-wall events, and the same
numbers for the first / middle / last third of the run (to see e.g. traction
getting worse lap after lap).
"""
import csv
import glob
import math
import os
import sys

MAX_STEER_US = 625
SAT_MARGIN_US = 5          # |steer - 1500| >= 620 -> saturated
OSC_HYST_CM = 5.0          # error must swing past +-5 cm to count as a crossing
NEAR_WALL_CM = 15


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    out = []
    for r in rows:
        try:
            out.append({k: float(v) for k, v in r.items()})
        except (TypeError, ValueError):
            continue  # truncated last row
    return out


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else 0.0


def crossings(errors):
    """Count sign changes of the error with hysteresis (= swings across the centre)."""
    state, n = 0, 0
    for e in errors:
        s = 1 if e > OSC_HYST_CM else -1 if e < -OSC_HYST_CM else 0
        if s and state and s != state:
            n += 1
        if s:
            state = s
    return n


def stats(rows):
    if len(rows) < 2:
        return None
    dur = (rows[-1]["t_ms"] - rows[0]["t_ms"]) / 1000.0 or 1e-9
    fwd = [r for r in rows if not r["rev"]]
    err = [r["error"] - r["bias"] for r in fwd]  # wall-following error without obstacle bias
    steer_dev = [abs(r["steer_us"] - 1500) for r in fwd]
    d_terms = [r["d"] for r in fwd]
    d_jumps = [abs(b - a) for a, b in zip(d_terms, d_terms[1:])]
    rev_starts = sum(1 for a, b in zip(rows, rows[1:]) if not a["rev"] and b["rev"])
    near = sum(1 for r in fwd if min(r["left"], r["right"]) < NEAR_WALL_CM)
    return {
        "dur": dur,
        "rows": len(rows),
        "err_rms": rms(err),
        "err_max": max((abs(e) for e in err), default=0),
        "osc_per_s": crossings(err) / dur,
        "sat_pct": 100.0 * sum(1 for s in steer_dev if s >= MAX_STEER_US - SAT_MARGIN_US) / max(1, len(fwd)),
        "steer_rms": rms(steer_dev),
        "p_rms": rms([r["p"] for r in fwd]),
        "d_rms": rms(d_terms),
        "d_jump_rms": rms(d_jumps),
        "i_max": max((abs(r["i"]) for r in fwd), default=0),
        "speed_mean": sum(r["speed_us"] for r in fwd) / max(1, len(fwd)),
        "rev_count": rev_starts,
        "rev_pct": 100.0 * (len(rows) - len(fwd)) / len(rows),
        "trapped": sum(1 for r in rows if r["trapped"]),
        "near_pct": 100.0 * near / max(1, len(fwd)),
    }


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        here = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "crashlogs")
        runs = sorted(glob.glob(os.path.join(here, "run_*.csv")), key=os.path.getmtime)
        if not runs:
            sys.exit("no crashlogs/run_*.csv found")
        path = runs[-1]
    rows = load(path)
    if len(rows) < 10:
        sys.exit(f"{path}: only {len(rows)} rows")

    loops = [r["loop"] for r in rows]
    expected = loops[-1] - loops[0] + 1
    cyc = sorted(r["cyc_ms"] for r in rows)
    settings = sorted({(r["kp"], r["ki"], r["kd"], r["speed_limit"]) for r in rows})

    print(f"run: {os.path.basename(path)}")
    print(f"rows {len(rows)}, {(rows[-1]['t_ms'] - rows[0]['t_ms']) / 1000:.1f} s, "
          f"lost packets {100 * (1 - len(rows) / expected):.1f}%, "
          f"loop median {cyc[len(cyc) // 2]:.0f} ms, max {cyc[-1]:.0f} ms")
    for kp, ki, kd, sp in settings:
        print(f"settings: kp={kp:.2f} ki={ki:.3f} kd={kd:.2f} speed={sp:.0f}")
    if "batt_v" in rows[0] and rows[0]["batt_v"] > 5:
        b = [r["batt_v"] for r in rows]
        print(f"battery: start {b[0]:.2f} V, end {b[-1]:.2f} V, lowest {min(b):.2f} V "
              f"({min(b) / 3:.2f} V/cell)")
    if any(r["manual"] for r in rows):
        print("NOTE: run contains manual driving rows")

    whole = stats(rows)
    n = len(rows)
    thirds = [stats(rows[:n // 3]), stats(rows[n // 3:2 * n // 3]), stats(rows[2 * n // 3:])]

    def line(label, key, fmt, unit=""):
        vals = "  ".join(fmt.format(t[key]) if t else "-" for t in thirds)
        print(f"{label:34s} {fmt.format(whole[key]):>8s}{unit:4s} | thirds: {vals}")

    print()
    print(f"{'':34s} {'whole':>8s}     | first / middle / last third")
    line("steering error RMS (cm)", "err_rms", "{:.1f}")
    line("steering error max (cm)", "err_max", "{:.0f}")
    line("swings across centre (per s)", "osc_per_s", "{:.2f}")
    line("steering RMS from centre (us)", "steer_rms", "{:.0f}")
    line("steering saturated (% of time)", "sat_pct", "{:.1f}")
    line("P term RMS (us)", "p_rms", "{:.0f}")
    line("D term RMS (us)", "d_rms", "{:.0f}")
    line("D loop-to-loop jump RMS (us)", "d_jump_rms", "{:.0f}")
    line("I term max (us)", "i_max", "{:.2f}")
    line("mean motor signal (us)", "speed_mean", "{:.0f}")
    line("reverse manoeuvres", "rev_count", "{:.0f}")
    line("time reversing (%)", "rev_pct", "{:.1f}")
    line("TRAPPED loops", "trapped", "{:.0f}")
    line(f"closer than {NEAR_WALL_CM} cm to a wall (%)", "near_pct", "{:.1f}")

    print()
    print("Hints (rough rules, look at the data before changing anything):")
    w = whole
    if w["osc_per_s"] > 1.5 and w["sat_pct"] < 10:
        print(" - frequent swings across the centre: P may be too high or D too low")
    if w["d_jump_rms"] > 0.7 * max(w["d_rms"], 1):
        print(" - D term mostly jumps loop to loop: D is reacting to sensor noise")
    if w["sat_pct"] > 15:
        print(" - steering often at full lock: curves too tight for this speed (speed, not PID)")
    if w["i_max"] < 2:
        print(" - I term is negligible (controller is effectively PD)")
    t0, t2 = thirds[0], thirds[2]
    if t0 and t2 and t2["err_rms"] > 1.3 * t0["err_rms"]:
        print(" - error grows towards the end of the run (traction / battery / heat?)")


if __name__ == "__main__":
    main()
