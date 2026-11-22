#!/usr/bin/env python3
"""Draw what the robot "thinks" the track looks like, as a page in the browser.

  ~/.platformio/penv/bin/python ~/Projects/Sonic-v2.1/tools/track_map.py [run.csv] [--no-open]
  (no file = newest crashlogs/run_*.csv; writes crashlogs/track_<run>.html)

No wheel encoder and no IMU, so the shape is reconstructed (dead reckoning) from
the steering angle and the speed command: turning = speed x steering. Speed and
steering are averaged over all regular laps first (single laps are too noisy),
then the average lap is integrated with its turning scaled to 360 deg (a circuit)
and closed by spreading the leftover error over the lap. Scale is
arbitrary (no odometry): it is the SHAPE that matters - corners, straights and
where the trouble is. Laps come from the robot's lap counter (or are recomputed).

Shows: all regular laps (thin) and their average (thick), coloured by
  - speed command, or
  - trouble: near the wall (< 15 cm), full steering lock, reversing
  - the learned speed map (factor per track part, if the recording has it)
and the 20 track parts as numbers (part 1 = right after the lap mark).
"""
import argparse
import glob
import html
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import lap_report  # noqa: E402

PARTS = 20


def integrate(raw, k, seg):
    x = y = th = 0.0
    pts = [(0.0, 0.0, seg[0])]
    length = 0.0
    for (dt, v, turn), r in zip(raw, seg[1:]):
        th += k * turn * dt
        x += v * dt * math.cos(th)
        y += v * dt * math.sin(th)
        length += abs(v) * dt
        pts.append((x, y, r))
    return pts, length


def lap_raw(seg):
    raw = []  # (dt, v, turn) per step
    for a, b in zip(seg, seg[1:]):
        dt = max(0.0, (b["t_ms"] - a["t_ms"]) / 1000.0)
        v = (a["speed_us"] - 1500) / 500.0                # -1..1 (reverse negative)
        steer = (a["steer_us"] - 1500) / 625.0             # -1 (right) .. +1 (left)
        raw.append((dt, v, v * math.tan(steer * math.radians(28))))
    return raw


def best_turn_factor(laps):
    """Turning factor k at which the laps close best (end where they started).
    Works for any track shape (also S-bends / figure-8, where a lap is not 360 deg)."""
    raws = [lap_raw(l) for l in laps]
    best = None
    for i in range(300):
        k = 0.2 * (1.025 ** i)                              # 0.2 .. ~330, log steps
        err = 0.0
        for raw, seg in zip(raws, laps):
            pts, length = integrate(raw, k, seg)
            err += math.hypot(pts[-1][0], pts[-1][1]) / max(length, 1e-6)
        err /= len(laps)
        if best is None or err < best[0]:
            best = (err, k)
    return best[1], best[0]


def reconstruct(seg, k):
    """One lap -> list of (x, y, row), with turning factor k, loop closed."""
    if len(seg) < 20:
        return None
    pts, _ = integrate(lap_raw(seg), k, seg)
    ex, ey = pts[-1][0], pts[-1][1]                         # close the loop
    n = len(pts) - 1
    return [(px - ex * i / n, py - ey * i / n, r) for i, (px, py, r) in enumerate(pts)]


def resample(pts, n=200):
    """Same number of points per lap (by time fraction) so laps can be averaged."""
    t0, t1 = pts[0][2]["t_ms"], pts[-1][2]["t_ms"]
    out, j = [], 0
    for i in range(n):
        t = t0 + (t1 - t0) * i / (n - 1)
        while j + 1 < len(pts) and pts[j + 1][2]["t_ms"] <= t:
            j += 1
        out.append(pts[j])
    return out


def colour_speed(us):
    f = max(0.0, min(1.0, (us - 1550) / 300.0))         # 1550 (slow) .. 1850 (fast)
    return f"hsl({int(220 - 220 * f)}, 75%, 50%)"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?")
    ap.add_argument("--no-open", action="store_true")
    a = ap.parse_args()
    path = a.csv or sorted(glob.glob(os.path.join(lap_report.CRASHLOGS, "run_*.csv")), key=os.path.getmtime)[-1]
    rows = lap_report.load(path)
    bounds, how = lap_report.lap_boundaries(rows)
    laps = []
    for s, e in zip(bounds, bounds[1:]):
        seg = [r for r in rows if s <= r["t_ms"] < e and not r.get("manual", 0)]
        if len(seg) > 20:
            laps.append(seg)
    if len(laps) < 2:
        raise SystemExit("need at least 2 complete laps")
    times = sorted((l[-1]["t_ms"] - l[0]["t_ms"]) / 1000 for l in laps)
    med = times[len(times) // 2]
    regular = [l for l in laps if abs((l[-1]["t_ms"] - l[0]["t_ms"]) / 1000 - med) <= 0.2 * med]
    # Average speed and steering over all regular laps (by time fraction), then
    # reconstruct that ONE average lap: individual laps are too noisy.
    N = 240
    bins = [[] for _ in range(N)]
    for l in regular:
        t0, t1 = l[0]["t_ms"], l[-1]["t_ms"]
        for r in l:
            bins[min(N - 1, int((r["t_ms"] - t0) / max(1, t1 - t0) * N))].append(r)
    avg_rows = []
    for i, b in enumerate(bins):
        if not b:
            b = bins[i - 1] if i else bins[1]
        avg_rows.append({"t_ms": med * 1000 * i / N,
                         "speed_us": sum(r["speed_us"] for r in b) / len(b),
                         "steer_us": sum(r["steer_us"] for r in b) / len(b), "_rows": b})
    raw = lap_raw(avg_rows)
    total = sum(dt * turn for dt, v, turn in raw)
    if abs(total) < 1e-6:
        raise SystemExit("no turning in the laps")
    k = (2 * math.pi if total > 0 else -2 * math.pi) / total     # circuit: 360 deg per lap
    pts = reconstruct(avg_rows, k)
    closure = 0.0
    avg = [(x, y, r["_rows"]) for x, y, r in pts]
    rs = []
    n = len(avg)
    paths = regular
    xs = [p[0] for p in avg]
    ys = [p[1] for p in avg]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    size = 640
    sc = (size - 80) / max(maxx - minx, maxy - miny, 1e-6)

    def P(x, y):
        return 40 + (x - minx) * sc, size - 40 - (y - miny) * sc

    has_map = "smf" in rows[0]
    layers = {"speed": [], "trouble": [], "map": []}
    for i in range(n - 1):
        x1, y1 = P(avg[i][0], avg[i][1])
        x2, y2 = P(avg[i + 1][0], avg[i + 1][1])
        rr = avg[i][2]
        spd = sum(r["speed_us"] for r in rr) / len(rr)
        near = sum(min(r["left"], r["right"]) < 15 for r in rr) / len(rr)
        lock = sum(abs(r["steer_us"] - 1500) >= 615 for r in rr) / len(rr)
        rev = sum(bool(r.get("rev", 0)) for r in rr) / len(rr)
        trouble = max(near, rev)
        tcol = "#d33" if trouble > 0.3 else "#e90" if (trouble > 0.1 or lock > 0.6) else "#2a2"
        seg = f'x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}"'
        layers["speed"].append(f'<line {seg} stroke="{colour_speed(spd)}"/>')
        layers["trouble"].append(f'<line {seg} stroke="{tcol}"/>')
        if has_map:
            f = sum(r["smf"] for r in rr) / len(rr)
            mcol = "#2a2" if f > 0.97 else "#e90" if f > 0.8 else "#d33"
            layers["map"].append(f'<line {seg} stroke="{mcol}"/>')
    thin = []
    for p in rs:
        pts = " ".join(f"{P(x, y)[0]:.1f},{P(x, y)[1]:.1f}" for x, y, _ in p)
        thin.append(f'<polyline points="{pts}" class="lap"/>')
    labels = []
    for k in range(PARTS):
        i = int(k * n / PARTS)
        x, y = P(avg[i][0], avg[i][1])
        labels.append(f'<text x="{x + 6:.0f}" y="{y - 6:.0f}" class="part">{k + 1}</text>')
    sx, sy = P(avg[0][0], avg[0][1])

    # ---- RING: the 20 track parts in driving order (what the robot reliably knows) ----
    parts = [[] for _ in range(PARTS)]
    for l in regular:
        t0, t1 = l[0]["t_ms"], l[-1]["t_ms"]
        for r in l:
            parts[min(PARTS - 1, int((r["t_ms"] - t0) / max(1, t1 - t0) * PARTS))].append((id(l), r))
    R, CX, CY, size_r = 230, 320, 300, 640
    ring = {"speed": [], "trouble": [], "map": []}
    glyphs, nums, details = [], [], []
    for k, pr in enumerate(parts):
        rr = [r for _, r in pr] or [{"speed_us": 1500, "steer_us": 1500, "left": 99, "right": 99}]
        steer = sum(r["steer_us"] - 1500 for r in rr) / len(rr)
        spd = sum(r["speed_us"] for r in rr) / len(rr)
        laps_in = {i for i, _ in pr} or {0}
        near = len({i for i, r in pr if min(r["left"], r["right"]) < 15}) / len(laps_in)
        rev = len({i for i, r in pr if r.get("rev", 0)}) / len(laps_in)
        lock = len({i for i, r in pr if abs(r["steer_us"] - 1500) >= 615}) / len(laps_in)
        smf = sum(r["smf"] for r in rr) / len(rr) if has_map else 1.0
        a0 = -math.pi / 2 + 2 * math.pi * k / PARTS
        a1 = -math.pi / 2 + 2 * math.pi * (k + 1) / PARTS
        p0 = (CX + R * math.cos(a0 + 0.012), CY + R * math.sin(a0 + 0.012))
        p1 = (CX + R * math.cos(a1 - 0.012), CY + R * math.sin(a1 - 0.012))
        arc = f'<path d="M{p0[0]:.1f},{p0[1]:.1f} A{R},{R} 0 0 1 {p1[0]:.1f},{p1[1]:.1f}"'
        trouble = max(near, rev)
        tcol = "#d33" if trouble > 0.3 else "#e90" if (trouble > 0.1 or lock > 0.6) else "#2a2"
        mcol = "#2a2" if smf > 0.97 else "#e90" if smf > 0.8 else "#d33"
        ring["speed"].append(f'{arc} stroke="{colour_speed(spd)}"/>')
        ring["trouble"].append(f'{arc} stroke="{tcol}"/>')
        ring["map"].append(f'{arc} stroke="{mcol}"/>')
        am = (a0 + a1) / 2
        gx, gy = CX + (R - 42) * math.cos(am), CY + (R - 42) * math.sin(am)
        sev = abs(steer) / 625
        kind = "straight" if sev < 0.12 else ("gentle " if sev < 0.4 else "tight ") + ("left" if steer > 0 else "right")
        glyph = "|" if kind == "straight" else ("&#8630;" if steer > 0 else "&#8631;")
        gsize = 14 if kind == "straight" else 16 if kind.startswith("gentle") else 22
        glyphs.append(f'<text x="{gx:.0f}" y="{gy + 6:.0f}" text-anchor="middle" class="glyph" font-size="{gsize}">{glyph}</text>')
        nx, ny = CX + (R + 26) * math.cos(am), CY + (R + 26) * math.sin(am)
        nums.append(f'<text x="{nx:.0f}" y="{ny + 4:.0f}" text-anchor="middle" class="part">{k + 1}</text>')
        details.append(f"<tr><td>{k + 1}</td><td>{kind}</td><td>{spd:.0f}</td><td>{100 * lock:.0f}</td>"
                       f"<td>{100 * near:.0f}</td><td>{100 * rev:.0f}</td><td>{smf:.2f}</td></tr>")
    mx, my = CX, CY - R
    ring_svg = (f'<svg viewBox="0 0 {size_r} 600">'
                + "".join(f'<g class="rl" id="r-{k}"{"" if k == "speed" else " style=display:none"}>{"".join(v)}</g>'
                          for k, v in ring.items())
                + "".join(glyphs) + "".join(nums)
                + f'<circle cx="{mx}" cy="{my}" r="8" fill="var(--fg)"/>'
                + f'<text x="{mx}" y="{my + 64}" text-anchor="middle" class="part">&#9650; lap mark, driving clockwise</text>'
                + f'<text x="{CX}" y="{CY}" text-anchor="middle" class="mid">{len(regular)} laps</text>'
                + f'<text x="{CX}" y="{CY + 22}" text-anchor="middle" class="part">median {med:.2f} s</text></svg>')

    name = os.path.basename(path)
    out = os.path.join(lap_report.CRASHLOGS, "track_" + name.replace(".csv", ".html"))
    layer_svg = {k: "\n".join(v) for k, v in layers.items()}
    buttons = '<button data-l="speed" class="on">Speed</button><button data-l="trouble">Trouble</button>'
    if has_map:
        buttons += '<button data-l="map">Speed map</button>'
    page = f"""<!doctype html><html><head><meta charset="utf-8"><title>Track map</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
:root {{ --bg:#fafaf8; --fg:#1d1d1b; --muted:#6b6b66; --line:#c9c9c3; --card:#fff; }}
@media (prefers-color-scheme: dark) {{ :root {{ --bg:#161615; --fg:#ececea; --muted:#9a9a95; --line:#3a3a37; --card:#1f1f1e; }} }}
body {{ background:var(--bg); color:var(--fg); font:15px/1.5 system-ui, sans-serif; margin:0; padding:16px; }}
main {{ max-width:720px; margin:auto; }}
h1 {{ font-size:20px; margin:0 0 4px; }} p {{ color:var(--muted); margin:0 0 12px; }}
svg {{ width:100%; height:auto; background:var(--card); border:1px solid var(--line); border-radius:8px; }}
.lap {{ fill:none; stroke:var(--line); stroke-width:1; }}
.layer line {{ stroke-width:7; stroke-linecap:round; }}
.part {{ fill:var(--muted); font-size:12px; }}
.btns button {{ font:inherit; padding:4px 10px; margin:0 6px 8px 0; border:1px solid var(--line); background:var(--card); color:var(--fg); border-radius:6px; cursor:pointer; }}
.btns button.on {{ border-color:var(--fg); font-weight:600; }}
.legend {{ font-size:13px; color:var(--muted); }}
.rl path {{ fill:none; stroke-width:26; }}
.glyph {{ fill:var(--fg); }} .mid {{ fill:var(--fg); font-size:20px; font-weight:600; }}
h2 {{ font-size:16px; margin:24px 0 4px; }} details {{ margin:8px 0 4px; }}
table {{ border-collapse:collapse; font-size:13px; }} td, th {{ padding:2px 8px; border-bottom:1px solid var(--line); text-align:right; }}
</style></head><body><main>
<h1>Track map - {html.escape(name)}</h1>
<p>The track as the robot knows it: the lap cut into 20 parts in driving order (as in lap_report and the speed map), from {len(paths)} regular laps (median {med:.2f} s; laps from {how}). Arrows = turn direction, bigger = tighter.</p>
<div class="btns">{buttons}</div>
{ring_svg}
<details><summary>Turn by turn</summary><table><tr><th>part</th><th>turn</th><th>speed us</th><th>full lock %</th><th>near wall %</th><th>reversing %</th><th>speed map</th></tr>{''.join(details)}</table></details>
<h2>Estimated shape (experimental)</h2>
<p>Dead reckoning from steering and speed. The steering command is not the real turning (sliding, wall contact at full lock), so tight corners can come out as loops. Use the ring above for decisions.</p>
<svg viewBox="0 0 {size} {size}">
{''.join(thin)}
<g class="layer" id="speed">{layer_svg['speed']}</g>
<g class="layer" id="trouble" style="display:none">{layer_svg['trouble']}</g>
<g class="layer" id="map" style="display:none">{layer_svg['map']}</g>
{''.join(labels)}
<circle cx="{sx:.1f}" cy="{sy:.1f}" r="7" fill="var(--fg)"/><text x="{sx + 10:.0f}" y="{sy + 18:.0f}" class="part">lap mark</text>
</svg>
<p class="legend" id="lg"></p>
</main><script>
const L = {{speed:"Speed command: blue = slow (1550 us) ... red = fast (1850 us).",
 trouble:"Trouble: red = near the wall or reversing in > 30 % of laps, orange = sometimes / mostly at full lock, green = fine.",
 map:"Learned speed map: green = full speed, orange = slowed to 80-97 %, red = slowed below 80 %."}};
const show = k => {{ document.querySelectorAll('.layer').forEach(g => g.style.display = g.id === k ? '' : 'none');
  document.querySelectorAll('.rl').forEach(g => g.style.display = g.id === 'r-' + k ? '' : 'none');
  document.querySelectorAll('.btns button').forEach(b => b.classList.toggle('on', b.dataset.l === k));
  document.getElementById('lg').textContent = L[k]; }};
document.querySelectorAll('.btns button').forEach(b => b.onclick = () => show(b.dataset.l)); show('speed');
</script></body></html>"""
    open(out, "w").write(page)
    print(f"track map: {out}")
    if not a.no_open:
        subprocess.Popen(["xdg-open", out], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":
    main()
