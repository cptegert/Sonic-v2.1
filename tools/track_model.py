#!/usr/bin/env python3
"""Track model for the dashboard: what the robot knows about the track, learned
over many laps and drives, and matched to the track you draw / describe.

Pieces:
  run_profile(rows)       one drive -> average lap, cut into N bins by lap time
                          (steering, speed, wall, trouble, speed map), regular laps only
  combine(profiles)       many drives -> one profile; each drive is shifted so its
                          turns line up with the others (older drives count less),
                          so the picture gets steadier with every lap
  shape(profile, geom)    estimated outline: turning = speed x tan(steer angle) /
                          wheelbase, one lap = 360 deg, closed with the smallest
                          change of the distances
  turns(dtheta)           list of turns: direction and degrees
  layout_points(layout)   your track: drawn points, or a description like
                          "S2 L90 S1 H1 R180 S2 L90" (S straight, L/R turn deg, H hill)
  match(profile, layout)  lines the robot's turns up with your track (both
                          directions, any start point, stretch allowed) -> every
                          robot bin gets a place on your track

Tracks are stored in tracks/<name>.json (shared in git: drawing, notes, speed test
results, and the profile of every recorded drive).
"""
import json
import math
import os
import re
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TRACKS = os.path.join(HERE, "..", "tracks")
N = 120          # bins per lap
PARTS = 20       # robot speed-map parts
NEAR_CM = 15
FULL_LOCK_US = 615
DEFAULT_GEOM = {"wheelbase_cm": 26.0, "max_steer_deg": 28.0}
CALIB_MIN_QUALITY = 0.7   # match with your track needed before it calibrates the steering
CALIB_MIN_LAPS = 8


# --------------------------------------------------------------------------- tracks
def safe_name(name):
    name = re.sub(r"[^A-Za-z0-9_-]", "", name or "")[:15]
    return name or "default"


def track_path(name):
    return os.path.join(TRACKS, safe_name(name) + ".json")


def load_track(name):
    try:
        with open(track_path(name)) as f:
            t = json.load(f)
    except (OSError, ValueError):
        t = {}
    t.setdefault("name", safe_name(name))
    t.setdefault("created", time.strftime("%Y-%m-%d %H:%M"))
    t.setdefault("notes", "")
    t.setdefault("layout", None)
    t.setdefault("runs", {})
    t.setdefault("speedtests", [])
    return t


def save_track(t):
    os.makedirs(TRACKS, exist_ok=True)
    tmp = track_path(t["name"]) + ".tmp"
    with open(tmp, "w") as f:
        json.dump(t, f, indent=1)
    os.replace(tmp, track_path(t["name"]))


def list_tracks():
    if not os.path.isdir(TRACKS):
        return []
    return sorted(f[:-5] for f in os.listdir(TRACKS) if f.endswith(".json") and not f.startswith("_"))


ALIASES = os.path.join(TRACKS, "_renamed.json")    # old track name -> new (rename on every robot)


def load_aliases():
    try:
        with open(ALIASES) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_aliases(a):
    os.makedirs(TRACKS, exist_ok=True)
    with open(ALIASES, "w") as f:
        json.dump(a, f, indent=1)


# --------------------------------------------------------------------------- laps
def lap_segments(rows):
    """Laps from the robot's lap counter: list of row lists (manual / wrong-way rows dropped,
    laps with a wrong-way mark skipped)."""
    if not rows or "lap" not in rows[0]:
        return []
    bounds = [b["t_ms"] for a, b in zip(rows, rows[1:]) if b["lap"] > a["lap"]]
    laps = []
    for s, e in zip(bounds, bounds[1:]):
        seg = [r for r in rows if s <= r["t_ms"] < e]
        if any(r.get("ww", 0) for r in seg):
            continue
        seg = [r for r in seg if not r.get("manual", 0)]
        if len(seg) > 20:
            laps.append(seg)
    return laps


def regular_laps(laps):
    if len(laps) < 2:
        return [], 0.0
    times = sorted((l[-1]["t_ms"] - l[0]["t_ms"]) / 1000 for l in laps)
    med = times[len(times) // 2]
    reg = [l for l in laps if abs((l[-1]["t_ms"] - l[0]["t_ms"]) / 1000 - med) <= 0.2 * med]
    return reg, med


def run_profile(rows):
    """Average lap of one drive, N bins by lap-time fraction. None if < 2 regular laps."""
    reg, med = regular_laps(lap_segments(rows))
    if len(reg) < 2:
        return None
    acc = [{"n": 0, "steer": 0.0, "spd": 0.0, "left": 0.0, "right": 0.0, "smf": 0.0,
            "near": set(), "lock": set(), "rev": set(), "spds": []} for _ in range(N)]
    has_smf = "smf" in rows[0]
    for li, l in enumerate(reg):
        t0, t1 = l[0]["t_ms"], l[-1]["t_ms"]
        for r in l:
            x = acc[min(N - 1, int((r["t_ms"] - t0) / max(1, t1 - t0) * N))]
            x["n"] += 1
            if not r.get("rev", 0):
                x["steer"] += r["steer_us"] - 1500
            x["spd"] += r["speed_us"]
            x["spds"].append(r["speed_us"])
            x["left"] += r["left"]
            x["right"] += r["right"]
            x["smf"] += r["smf"] if has_smf else 1.0
            if min(r["left"], r["right"]) < NEAR_CM:
                x["near"].add(li)
            if abs(r["steer_us"] - 1500) >= FULL_LOCK_US:
                x["lock"].add(li)
            if r.get("rev", 0):
                x["rev"].add(li)
    nl = len(reg)
    p = {k: [] for k in ("steer", "speed", "left", "right", "smf", "near", "lock", "rev", "speed_lo", "speed_hi")}
    for i, x in enumerate(acc):
        if not x["n"]:          # empty bin (Wi-Fi gap): copy the neighbour
            x = acc[i - 1] if i else next(a for a in acc if a["n"])
            for k in p:
                p[k].append(p[k][-1] if p[k] else 0)
            continue
        n = x["n"]
        p["steer"].append(round(x["steer"] / n / 625, 4))           # -1 right .. +1 left
        p["speed"].append(round(x["spd"] / n, 1))                   # motor us
        p["left"].append(round(x["left"] / n, 1))
        p["right"].append(round(x["right"] / n, 1))
        p["smf"].append(round(x["smf"] / n, 3))
        sp = sorted(x["spds"])
        p["speed_lo"].append(sp[len(sp) // 10])
        p["speed_hi"].append(sp[min(len(sp) - 1, len(sp) * 9 // 10)])
        for k in ("near", "lock", "rev"):
            p[k].append(round(len(x[k]) / nl, 3))
    p["laps"] = nl
    p["lap_s"] = round(med, 3)
    # seconds from driving off to the lap mark (only if recorded from the start)
    first = next((b["t_ms"] for a, b in zip(rows, rows[1:]) if b["lap"] > a["lap"]), None)
    if first and rows[0].get("loop", 999) < 40:
        after = (first - med * 1000 - rows[0]["t_ms"]) / 1000
        if 0 <= after < 40:
            p["mark_after_s"] = round(after, 2)
    p["when"] = time.strftime("%Y-%m-%d %H:%M")
    return p


# --------------------------------------------------------------------------- combine
def _corr_shift(a, b):
    """Circular shift s that best lines b up with a (b[i - s] ~ a[i])."""
    n = len(a)
    ma, mb = sum(a) / n, sum(b) / n
    best = (-2.0, 0)
    sa = math.sqrt(sum((x - ma) ** 2 for x in a)) or 1e-9
    sb = math.sqrt(sum((x - mb) ** 2 for x in b)) or 1e-9
    for s in range(n):
        c = sum((a[i] - ma) * (b[(i - s) % n] - mb) for i in range(n)) / (sa * sb)
        if c > best[0]:
            best = (c, s)
    return best[1], best[0]


def _roll(xs, s):
    n = len(xs)
    return [xs[(i - s) % n] for i in range(n)]


def combine(profiles, max_runs=12):
    """Weighted average of drives (newest first in the list). Weight = laps, each
    older drive x0.8. Drives are shifted to line up with the newest one (older
    firmware learned a new lap mark every drive)."""
    profiles = [p for p in profiles if p and len(p.get("steer", [])) == N][:max_runs]
    if not profiles:
        return None
    ref = profiles[0]["steer"]
    keys = ["steer", "speed", "left", "right", "smf", "near", "lock", "rev"]
    if all("speed_lo" in p for p in profiles):
        keys += ["speed_lo", "speed_hi"]
    tot = {k: [0.0] * N for k in keys}
    wsum, laps, lap_s, shifts = 0.0, 0, 0.0, []
    for age, p in enumerate(profiles):
        s, c = _corr_shift(ref, p["steer"]) if age else (0, 1.0)
        if age and c < 0.3:       # does not look like the same track / direction
            shifts.append(None)
            continue
        shifts.append(s)
        w = p["laps"] * (0.8 ** age)
        for k in keys:
            for i, v in enumerate(_roll(p[k], s)):
                tot[k][i] += w * v
        wsum += w
        laps += p["laps"]
        lap_s += w * p["lap_s"]
    out = {k: [round(v / wsum, 4) for v in tot[k]] for k in keys}
    out.update(laps=laps, runs=sum(1 for s in shifts if s is not None), lap_s=round(lap_s / wsum, 3))
    return out


# --------------------------------------------------------------------------- shape
def steer_angle(geom):
    g = dict(DEFAULT_GEOM, **{k: v for k, v in (geom or {}).items() if v})
    if g.get("turn_radius_cm"):     # measured turning circle at full lock (radius)
        return math.atan(g["wheelbase_cm"] / g["turn_radius_cm"]), g
    return math.radians(g["max_steer_deg"]), g


def raw_turn(p, geom=None):
    """Uncalibrated turning per bin: relative speed x tan(steering angle)."""
    dmax, g = steer_angle(geom)
    v = [max(0.05, (s - 1500) / 500) for s in p["speed"]]          # relative speed
    return [vi * math.tan(st * dmax) for vi, st in zip(v, p["steer"])], v


def dtheta_profile(p, geom=None, calib=None):
    """Heading change per bin (rad), distance per bin (relative), calibrated?
    The steering command is not the real turning: wall following zig-zags, and the
    robot steers with an offset. Without calibration the lap is scaled to 360 deg
    (or, if left and right nearly cancel, only turn directions / strengths are
    meaningful). With a calibration (from matching your drawn track) the offset is
    removed and turns come out in real degrees."""
    raw, v = raw_turn(p, geom)
    if calib and calib.get("gain"):
        dt = p["lap_s"] / N
        dth = [calib["gain"] * dt * (r - vi * calib["offset"]) for r, vi in zip(raw, v)]
        tot = sum(dth)
        if math.radians(250) < abs(tot) < math.radians(470):
            dth = [x * 2 * math.pi / abs(tot) for x in dth]
        return dth, v, True
    tot, ab = sum(raw), sum(abs(x) for x in raw) or 1e-9
    if abs(tot) >= 0.3 * ab:
        k = 2 * math.pi / abs(tot)                                # a circuit: 360 deg
    else:
        k = 4 * math.pi / ab                                      # guess: 720 deg of turning in all
    return [k * x for x in raw], v, False


def integrate(dth, ds, close=True):
    th, heads = 0.0, []
    for d in dth:
        heads.append(th + d / 2)
        th += d
    if close:   # stretch distances by (1 + a cos + b sin) so the loop ends where it started
        c = [math.cos(h) for h in heads]
        s = [math.sin(h) for h in heads]
        A = [[sum(d * ci * ci for d, ci in zip(ds, c)), sum(d * ci * si for d, ci, si in zip(ds, c, s))],
             [0.0, sum(d * si * si for d, si in zip(ds, s))]]
        A[1][0] = A[0][1]
        r = [-sum(d * ci for d, ci in zip(ds, c)), -sum(d * si for d, si in zip(ds, s))]
        det = A[0][0] * A[1][1] - A[0][1] * A[1][0]
        if abs(det) > 1e-12:
            a = (r[0] * A[1][1] - r[1] * A[0][1]) / det
            b = (A[0][0] * r[1] - A[1][0] * r[0]) / det
            ds = [d * max(0.3, 1 + a * ci + b * si) for d, ci, si in zip(ds, c, s)]
    x = y = 0.0
    pts = []
    for d, h in zip(ds, heads):
        pts.append((x, y))
        x += d * math.cos(h)
        y += d * math.sin(h)
    return pts


def normalize(pts, flip_y=True):
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    w = max(max(xs) - min(xs), max(ys) - min(ys), 1e-9)
    return [[round((x - min(xs)) / w, 4), round(((max(ys) - y) if flip_y else (y - min(ys))) / w, 4)] for x, y in pts]


def shape(p, geom=None, calib=None):
    dth, v, _ = dtheta_profile(p, geom, calib)
    return normalize(integrate(dth, v))


# --------------------------------------------------------------------------- turns
def smooth(xs, w=2):
    n = len(xs)
    return [sum(xs[(i + k) % n] for k in range(-w, w + 1)) / (2 * w + 1) for i in range(n)]


def turns(dth, min_deg=25):
    """Turns in a closed heading-change sequence: [{dir, deg, start, end}] (start/end
    = fraction of the lap). Neighbouring bits of the same direction are merged."""
    n = len(dth)
    sm = smooth(dth, max(1, n // 60))
    thr = 0.35 * 2 * math.pi / n                     # turning faster than ~1/3 of a circle
    sign = [0 if abs(x) < thr else (1 if x > 0 else -1) for x in sm]
    if all(s == sign[0] for s in sign):
        return []
    start = next(i for i in range(n) if sign[i] != sign[i - 1])   # begin at a change
    runs, cur = [], None
    for k in range(n):
        i = (start + k) % n
        if cur and sign[i] == cur["s"]:
            cur["bins"].append(i)
        else:
            cur = {"s": sign[i], "bins": [i]}
            runs.append(cur)
    # merge: short straight gaps (<= 3 % of a lap) between turns of the same direction
    merged = []
    for r in runs:
        if merged and r["s"] != 0 and merged[-1]["s"] == r["s"]:
            merged[-1]["bins"] += r["bins"]
        elif (merged and r["s"] == 0 and len(r["bins"]) <= max(1, n // 33)):
            merged[-1]["bins"] += r["bins"]
        else:
            merged.append(r)
    out = []
    for r in merged:
        if r["s"] == 0:
            continue
        deg = math.degrees(sum(dth[i] for i in r["bins"]))
        if abs(deg) >= min_deg:
            out.append({"dir": "L" if deg > 0 else "R", "deg": int(round(abs(deg) / 5) * 5),
                        "start": round(r["bins"][0] / n, 3), "end": round((r["bins"][-1] + 1) / n, 3)})
    out.sort(key=lambda t: t["start"])
    return out


# --------------------------------------------------------------------------- your track
def parse_description(text):
    """'S2 L90 S1 H1 R180' -> points (y up) + hill spans (fractions of the length).
    S<n> straight n units, H<n> hill (straight), L<deg>/R<deg> turn (radius 1, or
    L90r2 = radius 2)."""
    x = y = th = 0.0
    pts, hills, length = [(0.0, 0.0)], [], 0.0
    toks = re.findall(r"([SHLR])\s*(\d+(?:\.\d+)?)(?:\s*r\s*(\d+(?:\.\d+)?))?", text.upper().replace(",", " "))
    for kind, num, rad in toks:
        num = float(num)
        if kind in "SH":
            h0 = length
            steps = max(1, int(num * 4))
            for _ in range(steps):
                x += num / steps * math.cos(th)
                y += num / steps * math.sin(th)
                pts.append((x, y))
            length += num
            if kind == "H":
                hills.append([h0, length])
        else:
            r = float(rad) if rad else 1.0
            sgn = 1 if kind == "L" else -1
            steps = max(2, int(num / 5))
            for _ in range(steps):
                d = math.radians(num) / steps
                x += r * d * math.cos(th + sgn * d / 2)
                y += r * d * math.sin(th + sgn * d / 2)
                th += sgn * d
                pts.append((x, y))
            length += r * math.radians(num)
    if length <= 0:
        return None
    return {"points": normalize(pts), "hills": [[a / length, b / length] for a, b in hills],
            "closed_gap": round(math.hypot(x, y) / length, 3)}


def resample_closed(pts, m=N):
    """Closed polyline (normalized, y down) -> m points equally spaced along it."""
    P = [(p[0], -p[1]) for p in pts]                    # y up for maths
    if P[0] != P[-1]:
        P.append(P[0])
    seg = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(P, P[1:])]
    total = sum(seg)
    if total <= 0:
        return None
    out, j, acc = [], 0, 0.0
    for i in range(m):
        s = total * i / m
        while j < len(seg) - 1 and acc + seg[j] < s:
            acc += seg[j]
            j += 1
        f = (s - acc) / seg[j] if seg[j] else 0
        a, b = P[j], P[j + 1]
        out.append((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])))
    return out


def chaikin(pts, it=2):
    for _ in range(it):
        n = len(pts)
        new = []
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            new += [(0.75 * a[0] + 0.25 * b[0], 0.75 * a[1] + 0.25 * b[1]),
                    (0.25 * a[0] + 0.75 * b[0], 0.25 * a[1] + 0.75 * b[1])]
        pts = new
    return pts


# --------------------------------------------------------------------------- tile tracks
# Square tiles on a grid: straight, 90 deg turn, ramp (straight with a slope; rot sets
# which end is HIGH), bridge (raised straight; may sit across a straight = crossing).
# Sides: 0 north (up on the screen), 1 east, 2 south, 3 west. rot = quarter turns clockwise.
DX, DY = (0, 1, 0, -1), (-1, 0, 1, 0)
TILE_BASE = {"straight": (0, 2), "ramp": (0, 2), "bridge": (0, 2), "turn": (2, 1)}
SIDE_NAME = ("north", "east", "south", "west")


def tile_sides(t):
    a, b = TILE_BASE.get(t.get("type"), (0, 2))
    r = int(t.get("rot", 0)) % 4
    return (a + r) % 4, (b + r) % 4


def trace_tiles(tiles, start=None):
    """Follow the pieces from the start (robot placement: tile + driving direction).
    Returns {points (tile units, y down), kinds per point, closed, error, tiles, start_frac}."""
    cells = {}
    for t in tiles or []:
        if t.get("type") in TILE_BASE:
            cells.setdefault((int(t["x"]), int(t["y"])), []).append(t)
    if not cells:
        return {"points": [], "closed": False, "error": "no pieces yet", "tiles": 0}
    x0 = y0 = ent = None
    if start and (int(start.get("x", -99)), int(start.get("y", -99))) in cells:
        x0, y0 = int(start["x"]), int(start["y"])
        for t in cells[(x0, y0)]:
            sd = tile_sides(t)
            if int(start.get("dir", -1)) in sd:
                ent = sd[0] if sd[1] == int(start["dir"]) else sd[1]
                break
    if ent is None:
        (x0, y0), ts = min(cells.items())
        ent = tile_sides(ts[0])[0]
    pts, kinds = [], []
    x, y, e = x0, y0, ent
    used = set()
    for _ in range(600):
        tile = next((t for t in cells.get((x, y), []) if e in tile_sides(t)), None)
        if tile is None:
            return {"points": pts, "kinds": kinds, "closed": False, "tiles": len(used),
                    "error": f"track open: nothing connects at tile ({x},{y}) from the {SIDE_NAME[e]}"}
        if (x, y, e) in used:
            break
        used.add((x, y, e))
        sd = tile_sides(tile)
        ex = sd[1] if sd[0] == e else sd[0]
        cx, cy = x + 0.5, y + 0.5
        a = (cx + 0.5 * DX[e], cy + 0.5 * DY[e])
        b = (cx + 0.5 * DX[ex], cy + 0.5 * DY[ex])
        kind = None
        if tile["type"] == "ramp":
            kind = "up" if ex == tile_sides(tile)[0] else "down"   # first side = high end
        elif tile["type"] == "bridge":
            kind = "bridge"
        if (e + 2) % 4 == ex:                     # straight through
            seg = [(a[0] + (b[0] - a[0]) * k / 6, a[1] + (b[1] - a[1]) * k / 6) for k in range(6)]
        else:                                     # quarter circle around the shared corner
            ccx, ccy = cx + 0.5 * (DX[e] + DX[ex]), cy + 0.5 * (DY[e] + DY[ex])
            a0 = math.atan2(a[1] - ccy, a[0] - ccx)
            a1 = math.atan2(b[1] - ccy, b[0] - ccx)
            d = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
            seg = [(ccx + 0.5 * math.cos(a0 + d * k / 6), ccy + 0.5 * math.sin(a0 + d * k / 6)) for k in range(6)]
        pts += seg
        kinds += [kind] * len(seg)
        x, y, e = x + DX[ex], y + DY[ex], (ex + 2) % 4
        if (x, y) == (x0, y0) and e == ent:
            n = len(used)
            return {"points": pts, "kinds": kinds, "closed": True, "error": None, "tiles": n,
                    "start_frac": 0.5 / n}       # robot stands in the middle of the first tile
    return {"points": pts, "kinds": kinds, "closed": False, "tiles": len(used), "error": "track does not close"}


def tile_hills(pts, kinds):
    """Runs of up / bridge / down -> [[from, to, kind]] as fractions of the track length."""
    n = len(pts)
    seg = [math.hypot(pts[(i + 1) % n][0] - pts[i][0], pts[(i + 1) % n][1] - pts[i][1]) for i in range(n)]
    tot = sum(seg) or 1
    cum = [0.0]
    for d in seg:
        cum.append(cum[-1] + d / tot)
    out, i = [], 0
    while i < n:
        if kinds[i]:
            j = i
            while j < n and kinds[j] == kinds[i]:
                j += 1
            out.append([round(cum[i], 4), round(cum[j], 4), kinds[i]])
            i = j
        else:
            i += 1
    return out


def layout_points(layout, start=None):
    """Your track as M points (normalized, y down) + hill spans. Drawn points are
    smoothed a little (corners become curves, like a real track)."""
    if not layout:
        return None
    extra = {}
    if layout.get("tiles"):
        tr = trace_tiles(layout["tiles"], start or layout.get("placement"))
        if not tr["closed"]:
            return None
        pts, hills = tr["points"], tile_hills(tr["points"], tr["kinds"])
        R = resample_closed(pts, N)          # tile units are y-down like a drawing
        seg = [math.hypot(pts[(i + 1) % len(pts)][0] - pts[i][0], pts[(i + 1) % len(pts)][1] - pts[i][1])
               for i in range(len(pts))]
        first_tile = sum(seg[:6])            # 6 points per tile: robot in the middle of the first one
        extra = {"raw": [[round(x, 3), round(-y, 3)] for x, y in R],     # resampled, tile units, y down
                 "start_frac": 0.5 * first_tile / sum(seg), "tiles": tr["tiles"],
                 "length_tiles": round(sum(seg), 2)}
        return dict({"xy": normalize(R), "up": R, "hills": hills}, **extra)
    if layout.get("text"):
        d = parse_description(layout["text"])
        if not d:
            return None
        pts, hills = d["points"], d["hills"]
    else:
        pts = layout.get("points") or []
        if len(pts) < 3:
            return None
        hills = list(layout.get("hills", []))
        if layout.get("hill_pts"):          # hill = from corner point a to corner point b (driving order)
            n = len(pts)
            seg = [math.hypot(pts[(i + 1) % n][0] - pts[i][0], pts[(i + 1) % n][1] - pts[i][1]) for i in range(n)]
            tot = sum(seg) or 1
            cum = [sum(seg[:i]) / tot for i in range(n)]
            for a, b in layout["hill_pts"]:
                if 0 <= a < n and 0 <= b < n:
                    fa, fb = cum[a], cum[b]
                    hills += [[fa, fb]] if fb > fa else [[fa, 1.0], [0.0, fb]]
        pts = [list(p) for p in chaikin([tuple(p) for p in pts], 2 if layout.get("smooth", True) else 0)]
    R = resample_closed(pts, N)
    if not R:
        return None
    return {"xy": normalize(R), "up": R, "hills": hills}


def heading_changes(up):
    m = len(up)
    hd = [math.atan2(up[(i + 1) % m][1] - up[i][1], up[(i + 1) % m][0] - up[i][0]) for i in range(m)]
    return [math.atan2(math.sin(hd[i] - hd[i - 1]), math.cos(hd[i] - hd[i - 1])) for i in range(m)]


def by_distance(dth, ds, m=N):
    """Per time bin -> per equal-distance bin: turning and distance are spread over
    the distance each time bin covers. Also the distance fraction of every time bin."""
    total = sum(ds)
    out, dist = [0.0] * m, [0.0] * m
    pos = []
    s = 0.0
    for d, v in zip(dth, ds):
        a, b = s / total * m, (s + v) / total * m
        pos.append((s + v / 2) / total)
        j = int(a)
        while j < b and j < m:
            f = (min(b, j + 1) - max(a, j)) / max(b - a, 1e-12)
            out[j] += d * f
            dist[j] += v * f
            j += 1
        s += v
    return out, dist, pos


def dtw_path(a, b, band):
    """Monotonic alignment of a onto b (same length, already roughly lined up)."""
    n = len(a)
    INF = float("inf")
    D = [[INF] * (n + 1) for _ in range(n + 1)]
    D[0][0] = 0.0
    for i in range(1, n + 1):
        for j in range(max(1, i - band), min(n, i + band) + 1):
            c = (a[i - 1] - b[j - 1]) ** 2
            D[i][j] = c + min(D[i - 1][j], D[i][j - 1], D[i - 1][j - 1])
    i = j = n
    mp = [0] * n
    while i > 0 and j > 0:
        mp[i - 1] = j - 1
        step = min((D[i - 1][j - 1], 0), (D[i - 1][j], 1), (D[i][j - 1], 2))[1]
        if step == 0:
            i, j = i - 1, j - 1
        elif step == 1:
            i -= 1
        else:
            j -= 1
    return mp


def _pearson(a, b):
    n = len(a)
    ma, mb = sum(a) / n, sum(b) / n
    sa = math.sqrt(sum((x - ma) ** 2 for x in a)) or 1e-9
    sb = math.sqrt(sum((x - mb) ** 2 for x in b)) or 1e-9
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (sa * sb)


def match(p, lay, geom=None, prior=None):
    """Place every robot bin on your track. Returns {quality, reverse, shift, bin_to_xy,
    bin_to_frac, draw_bin} or None."""
    if not p or not lay:
        return None
    raw, ds = raw_turn(p, geom)          # alignment by correlation: no calibration needed
    rob, dist, pos = by_distance(raw, ds)
    rs = smooth(rob, 3)
    up = lay["up"]
    best = None
    m0 = len(up)
    if prior is not None:
        # you said where the robot stood and which way it drove: the track is traced in
        # that direction from there, and the lap mark is about mark_frac further on
        dd = smooth(heading_changes(up), 3)
        want = prior["mark_frac"] * m0
        for s_ in range(m0):
            off = abs(((-s_) % m0) - want)
            if min(off, m0 - off) > prior.get("window", 0.15) * m0:
                continue
            c_ = _pearson(rs, _roll(dd, s_))
            if best is None or c_ > best[0]:
                best = (c_, False, s_, dd)
    if best is None:
        for rev in (False, True):
            u = list(reversed(up)) if rev else up
            dd = smooth(heading_changes(u), 3)
            s, c = _corr_shift(rs, dd)            # dd rolled by s lines up with the robot
            if best is None or c > best[0]:
                best = (c, rev, s, dd)
    c, rev, s, dd = best
    ddr = _roll(dd, s)
    mp = dtw_path(rs, ddr, band=max(3, N // 10))          # robot distance bin -> drawing bin
    al = [ddr[j] for j in mp]
    q = c        # score WITHOUT stretching (stretching makes almost anything look similar)
    m = len(up)
    # calibration: robot turning = offset x distance + turning / gain. Offset from the
    # parts that are STRAIGHT on your track (any robot "turning" there is offset),
    # gain from the rest. Unsmoothed robot values, smoothed track to find straights.
    calib = None
    straight = [abs(x) < 0.3 * 2 * math.pi / m for x in al]
    if sum(straight) >= m // 10:
        tb = sum(r for r, st in zip(rob, straight) if st) / max(1e-9, sum(d for d, st in zip(dist, straight) if st))
        turn_rob = sum(abs(r - tb * d) for r, d, st in zip(rob, dist, straight) if not st)
        turn_true = sum(abs(x) for x, st in zip(al, straight) if not st)
        if turn_rob > 1e-9:
            calib = {"gain": round(turn_true / turn_rob / (p["lap_s"] / N), 5), "offset": round(tb, 5),
                     "quality": round(q, 3)}
    def to_draw(k):            # rolled & maybe reversed drawing index -> original index
        k = (k - s) % m
        return (m - 1 - k) if rev else k
    # robot time bin -> distance bin -> drawing point (interpolated on the DTW map)
    bin_to_draw = []
    for f in pos:
        x = f * m
        i0 = int(x) % m
        bin_to_draw.append(to_draw(mp[i0]))
    draw_bin = [None] * m
    for i, k in enumerate(bin_to_draw):
        draw_bin[k] = i
    last = None    # fill gaps: nearest earlier robot bin
    for k in list(range(m)) * 2:
        if draw_bin[k] is None and last is not None:
            draw_bin[k] = last
        elif draw_bin[k] is not None:
            last = draw_bin[k]
    return {"quality": round(max(0.0, q), 3), "reverse": rev, "shift": s, "calib": calib,
            "used_start": prior is not None,
            "bin_to_draw": bin_to_draw, "draw_bin": draw_bin}


def similarity_fit(src, dst):
    """Best rotation + scale + shift of the points src onto dst (same length): src mapped."""
    n = len(src)
    mx = sum(p[0] for p in src) / n; my = sum(p[1] for p in src) / n
    nx = sum(p[0] for p in dst) / n; ny = sum(p[1] for p in dst) / n
    a = b = ss = 0.0
    for (x, y), (u, v) in zip(src, dst):
        x -= mx; y -= my; u -= nx; v -= ny
        a += x * u + y * v
        b += x * v - y * u
        ss += x * x + y * y
    if ss < 1e-12:
        return None, None
    c, s_ = a / ss, b / ss                     # scale * (cos, sin)
    out = [[round(nx + c * (x - mx) - s_ * (y - my), 3), round(ny + s_ * (x - mx) + c * (y - my), 3)] for x, y in src]
    err = math.sqrt(sum((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 for p, q in zip(out, dst)) / n)
    size = math.sqrt(sum((q[0] - nx) ** 2 + (q[1] - ny) ** 2 for q in dst) / n) or 1
    return out, err / size


def backwards_lookalike(up):
    """How much the track looks the same driven backwards (turn order): 0..1.
    High (figure-8, symmetric shapes) = steering cannot tell the direction."""
    d = smooth(heading_changes(up), 3)
    back = [-x for x in reversed(d)]
    return round(max(0.0, _corr_shift(d, back)[1]), 2)


# --------------------------------------------------------------------------- summary
def parts_table(p, dth, hills_bins=None):
    rows = []
    per = N // PARTS
    for k in range(PARTS):
        b = range(k * per, (k + 1) * per)
        deg = math.degrees(sum(dth[i] for i in b))
        rows.append({
            "part": k + 1,
            "turn": "straight" if abs(deg) < 8 else f"{'left' if deg > 0 else 'right'} {abs(deg):.0f}°",
            "speed": round(sum(p["speed"][i] for i in b) / per),
            "near": round(100 * max(p["near"][i] for i in b)),
            "lock": round(100 * max(p["lock"][i] for i in b)),
            "rev": round(100 * max(p["rev"][i] for i in b)),
            "smf": round(min(p["smf"][i] for i in b), 2),
            "width": round(sum(p["left"][i] + p["right"][i] for i in b) / per),
            "hill": next((hills_bins[i] for i in b if hills_bins and hills_bins[i]), None),
        })
    return rows


def analyse(track, robot, geom=None, live_profile=None, live_key=None, calib=None):
    """Everything the dashboard shows about a track, from its stored drives (+ the live one)."""
    runs = [(k, v) for k, v in track.get("runs", {}).items() if v.get("robot") == robot]
    if live_profile and live_key:
        runs = [(k, v) for k, v in runs if k != live_key] + [(live_key, dict(live_profile, robot=robot))]
    runs.sort(key=lambda kv: kv[1].get("when", ""), reverse=True)
    p = combine([v for _, v in runs])
    layout = track.get("layout")
    newest = runs[0][1] if runs else {}
    prior, start = None, None
    if layout and layout.get("tiles") and newest.get("start"):
        start = newest["start"]               # where you put the robot for the newest drive
    lay = layout_points(layout, start)
    if lay and start and lay.get("start_frac") is not None and newest.get("mark_after_s") is not None:
        prior = {"mark_frac": (lay["start_frac"] + newest["mark_after_s"] / newest["lap_s"]) % 1.0}
    res = {"name": track["name"], "notes": track.get("notes", ""), "layout": track.get("layout"),
           "speedtests": track.get("speedtests", [])[-5:], "best_speed": track.get("best_speed"),
           "tests": track.get("tests", [])[-15:],
           "profile": None, "drawing": None}
    if lay:
        dd = heading_changes(lay["up"])
        res["drawing"] = {"xy": lay["xy"], "hills": lay["hills"], "turns": turns(dd),
                          "backwards_lookalike": backwards_lookalike(lay["up"]),
                          "raw": lay.get("raw"), "tiles": lay.get("tiles"), "length_tiles": lay.get("length_tiles"),
                          "start": start or (layout or {}).get("placement")}
    if not p:
        return res
    m = match(p, lay, geom, prior) if lay else None
    if m and m["calib"] and m["quality"] >= CALIB_MIN_QUALITY and p["laps"] >= CALIB_MIN_LAPS:
        calib = m["calib"]                    # this track calibrates the robot right now
    dth, ds, calibrated = dtheta_profile(p, geom, calib)
    hills_bins = None
    if m and lay["hills"]:
        hills_bins = [next((h[2] if len(h) > 2 else "hill" for h in lay["hills"]
                            if h[0] <= m["bin_to_draw"][i] / N <= h[1]), None) for i in range(N)]
    res["profile"] = {
        "laps": p["laps"], "runs": p["runs"], "lap_s": p["lap_s"],
        "speed": p["speed"], "near": p["near"], "lock": p["lock"], "rev": p["rev"], "smf": p["smf"],
        "shape": normalize(integrate(dth, ds)), "turns": turns(dth), "calibrated": calibrated,
        "parts": parts_table(p, dth, hills_bins),
    }
    res["match"] = m
    res["profile"]["speed_lo"] = p.get("speed_lo")
    res["profile"]["speed_hi"] = p.get("speed_hi")
    if m and lay:
        # the robot's own shape guess, turned / scaled onto your track for comparison
        target = lay.get("raw") or lay["xy"]
        est = res["profile"]["shape"]
        dst = [target[m["bin_to_draw"][i]] for i in range(N)]
        ov, err = similarity_fit(est, dst)
        if ov:
            res["overlay"] = {"points": ov, "error": round(err, 3)}
    return res


if __name__ == "__main__":      # quick check on a recording: python track_model.py run.csv ["S2 L90 ..."]
    import sys
    sys.path.insert(0, HERE)
    import lap_report
    rows = lap_report.load(sys.argv[1])
    p = run_profile(rows)
    if not p:
        sys.exit("fewer than 2 regular laps")
    dth, _, _ = dtheta_profile(p)
    print(f"{p['laps']} laps, median {p['lap_s']} s")
    print("robot turns:", " ".join(f"{t['dir']}{t['deg']}@{t['start']:.2f}" for t in turns(dth)))
    if len(sys.argv) > 2:
        lay = layout_points({"text": sys.argv[2]})
        print("your turns: ", " ".join(f"{t['dir']}{t['deg']}@{t['start']:.2f}" for t in turns(heading_changes(lay["up"]))))
        m = match(p, lay)
        print(f"match quality {m['quality']:.2f}, reverse={m['reverse']}, calib {m['calib']}")
        dth, _, _ = dtheta_profile(p, None, m["calib"])
        print("calibrated robot turns:", " ".join(f"{t['dir']}{t['deg']}@{t['start']:.2f}" for t in turns(dth)))
