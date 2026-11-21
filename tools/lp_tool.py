#!/usr/bin/env python3
"""Local positioning tools for the second delivery.

Run them on a laptop joined to the "embedded" Wi-Fi network. The camera PC
broadcasts one JSON line per frame on UDP port 5000.

  record  Place the Zumo on each waypoint and press Enter. Writes
          WaypointLap_Zumo/waypoints.h and tools/waypoints.csv.
  extract Same as record, from a log: stop for 2 s on each waypoint, P1 to
          P14 in order, while log runs. Uses the last 14 stops.
  log     Saves every pose of the marker to a CSV file until Ctrl+C.
  plot    Draws the numbered waypoints, the desired route and the logged runs,
          and prints the lap time of each run. Needs matplotlib:
          uv run --with matplotlib tools/lp_tool.py plot ...
"""
import argparse
import csv
import json
import math
import re
import select
import socket
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
HEADER = ROOT / "WaypointLap_Zumo" / "waypoints.h"
CSV = ROOT / "tools" / "waypoints.csv"
N_WAYPOINTS = 14
TURN_ENTRANCES = {1, 3, 5, 7, 9, 11, 13}   # the section from these waypoints to the next one is a turn


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def open_socket(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", port))
    return s


def read_marker(sock, marker):
    """Returns (x, y, angle) of the marker from one UDP packet, or None."""
    data, _ = sock.recvfrom(65535)
    try:
        msg = json.loads(data.decode(errors="ignore"))
    except json.JSONDecodeError:
        return None
    for m in msg.get("markers", []):
        if m.get("id") == marker:
            return float(m["x"]), float(m["y"]), float(m["angle"])
    return None


def mean_pose(samples):
    x = sum(p[0] for p in samples) / len(samples)
    y = sum(p[1] for p in samples) / len(samples)
    a = math.degrees(math.atan2(sum(math.sin(math.radians(p[2])) for p in samples),
                                sum(math.cos(math.radians(p[2])) for p in samples)))
    return x, y, a


def fit_angle(wps):
    """Finds heading = sign * angle + offset from the straight sections.

    On a straight from Pi to Pj the tractor faces Pj at both ends, so the true
    heading is atan2(yj - yi, xj - xi). Tries both signs and keeps the better one.
    """
    pairs = []
    for i, (x, y, a, turn) in enumerate(wps):
        if turn:
            continue
        xj, yj, aj, _ = wps[(i + 1) % len(wps)]
        h = math.atan2(yj - y, xj - x)
        pairs += [(h, a), (h, aj)]
    best = None
    for sign in (1, -1):
        d = [wrap(h - sign * math.radians(a)) for h, a in pairs]
        off = math.atan2(sum(map(math.sin, d)), sum(map(math.cos, d)))
        err = max(abs(wrap(e - off)) for e in d)
        if best is None or math.degrees(err) < best[2]:
            best = (sign, math.degrees(off), math.degrees(err))
    return best


def heading(angle_deg, sign, offset_deg):
    return math.radians(sign * angle_deg + offset_deg)


def estimate_marker_offset(wps, sign, offset):
    """Distance (m) from the marker forward to the turning center of the Zumo.

    In a 180-degree turn, the turning center moves only sideways from the
    entrance to the exit. The marker sits d behind the center, so it also moves
    2 d along the entrance heading. Averages d over all the 180-degree turns.
    """
    ds = []
    for i, (x, y, a, turn) in enumerate(wps):
        xj, yj, aj, _ = wps[(i + 1) % len(wps)]
        if turn and abs(wrap(math.radians(aj - a))) > math.radians(150):
            h = heading(a, sign, offset)
            ds.append(((xj - x) * math.cos(h) + (yj - y) * math.sin(h)) / 2)
    return sum(ds) / len(ds) if ds else 0.0


def to_center(x, y, a, sign, offset, d):
    h = heading(a, sign, offset)
    return x + d * math.cos(h), y + d * math.sin(h)


def read_defines():
    """ANGLE_SIGN, ANGLE_OFFSET_DEG and MARKER_OFFSET_M from waypoints.h."""
    text = HEADER.read_text()
    get = lambda name, default: float(re.search(rf"#define {name}\s+(-?[\d.]+)f", text).group(1)) if name in text else default
    return get("ANGLE_SIGN", 1.0), get("ANGLE_OFFSET_DEG", 0.0), get("MARKER_OFFSET_M", 0.0)


def finish(raw, args):
    """Fits the angle and the marker offset, moves the waypoints to the turning center, writes the files."""
    sign, offset, err = fit_angle(raw)
    d = args.marker_offset if args.marker_offset is not None else estimate_marker_offset(raw, sign, offset)
    wps = [(*to_center(x, y, a, sign, offset, d), a, turn) for x, y, a, turn in raw]
    write_files(wps, args.start, sign, offset, err, d)
    print(f"Angle fit: heading = {sign:+.0f} * angle + {offset:.1f} deg, worst error {err:.1f} deg")
    print(f"Marker offset: the turning center is {d * 1000:.0f} mm in front of the marker")
    if err > 15:
        print("WARNING: the angle fit is poor. Make sure that the Zumo faced the travel direction at each waypoint.")
    if not 0.0 <= d <= 0.10:
        print("WARNING: the marker offset is not between 0 and 100 mm. Measure it and pass --marker-offset.")
    print(f"Wrote {HEADER} and {CSV}. Rebuild and flash WaypointLap_Zumo.")


def find_stops(rows, min_s=1.5):
    """Places where the marker stayed within 8 mm and 4 degrees for min_s or more.

    Stops closer than 3 cm to the next one merge, and the last one wins:
    that is the final placement after a small correction by hand.
    """
    stops, i = [], 0
    while i < len(rows):
        j = i
        while (j + 1 < len(rows) and math.hypot(rows[j + 1][1] - rows[i][1], rows[j + 1][2] - rows[i][2]) < 0.008
               and abs(wrap(math.radians(rows[j + 1][3] - rows[i][3]))) < math.radians(4)):
            j += 1
        if rows[j][0] - rows[i][0] >= min_s:
            stop = mean_pose([r[1:] for r in rows[i:j + 1]])
            if stops and math.hypot(stop[0] - stops[-1][0], stop[1] - stops[-1][1]) < 0.03:
                stops[-1] = stop
            else:
                stops.append(stop)
        i = j + 1
    return stops


def write_files(wps, start, sign, offset, err, marker_offset):
    with open(CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["P", "x_m", "y_m", "angle_deg", "turn"])
        for i, (x, y, a, turn) in enumerate(wps):
            w.writerow([i + 1, f"{x:.3f}", f"{y:.3f}", f"{a:.1f}", int(turn)])

    n = len(wps)
    route = [(start - 1 + k) % n for k in range(n + 1)]
    lines = [
        "// Waypoint table. Written by tools/lp_tool.py from camera measurements.",
        "#pragma once",
        '#include "follow.h"',
        "",
        "// Camera angle (deg) -> heading (rad, counterclockwise from +x):",
        "//   heading = ANGLE_SIGN * angle + ANGLE_OFFSET_DEG",
        f"// Fit on the straight sections, worst error {err:.1f} deg.",
        f"#define ANGLE_SIGN        {sign:.1f}f",
        f"#define ANGLE_OFFSET_DEG  {offset:.1f}f",
        "",
        "// The turning center of the Zumo is this far in front of the marker (m).",
        "// x and y below are turning-center positions, not marker positions.",
        f"#define MARKER_OFFSET_M   {marker_offset:.3f}f",
        "",
        "static const waypoint_t WAYPOINTS[] = {",
        "    //    x (m)    y (m)  angle  turn",
    ]
    for i, (x, y, a, turn) in enumerate(wps):
        kind = "turn entrance" if turn else "turn exit"
        lines.append(f"    {{ {x:6.3f}f, {y:6.3f}f, {a:6.1f}f, {'true ' if turn else 'false'} }},   // P{i + 1} {kind}")
    lines += [
        "};",
        "",
        f"// Waypoint order of one lap: start at P{start}, go around, stop at P{start}.",
        "static const int ROUTE[] = { " + ", ".join(map(str, route)) + " };",
        "#define ROUTE_LEN ((int)(sizeof ROUTE / sizeof ROUTE[0]))",
        "",
    ]
    HEADER.write_text("\n".join(lines))


def cmd_record(args):
    sock = open_socket(args.port)
    recent, wps = [], []
    print(f"Listening on UDP {args.port} for marker {args.marker}.")
    while len(wps) < N_WAYPOINTS:
        k = len(wps) + 1
        print(f"Place the Zumo on P{k}, facing the travel direction. Press Enter to save P{k}.")
        while True:
            ready, _, _ = select.select([sock, sys.stdin], [], [])
            if sock in ready:
                p = read_marker(sock, args.marker)
                if p:
                    recent = (recent + [p])[-10:]   # average of the last 10 frames (about 2 s)
            if sys.stdin in ready:
                sys.stdin.readline()
                if not recent:
                    print("No pose of the marker yet. Is the marker in view?")
                    continue
                x, y, a = mean_pose(recent)
                wps.append((x, y, a, k in TURN_ENTRANCES))
                print(f"  P{k}: x={x:.3f} y={y:.3f} angle={a:.1f} ({len(recent)} frames)")
                recent = []
                break
    finish(wps, args)


def cmd_extract(args):
    with open(args.log) as f:
        rows = [(float(r["t_s"]), float(r["x_m"]), float(r["y_m"]), float(r["angle_deg"])) for r in csv.DictReader(f)]
    stops = find_stops(rows)
    print(f"Found {len(stops)} stops in {args.log}:")
    for k, (x, y, a) in enumerate(stops):
        use = k >= len(stops) - N_WAYPOINTS
        print(f"  stop {k + 1:2d}: x={x:.3f} y={y:.3f} angle={a:.1f}" + (f"  -> P{k - len(stops) + N_WAYPOINTS + 1}" if use else "  (not used)"))
    if len(stops) < N_WAYPOINTS:
        sys.exit(f"Need {N_WAYPOINTS} stops. Stop for 2 s on each waypoint, P1 to P14 in order.")
    raw = [(x, y, a, k + 1 in TURN_ENTRANCES) for k, (x, y, a) in enumerate(stops[-N_WAYPOINTS:])]
    finish(raw, args)


def cmd_log(args):
    sock = open_socket(args.port)
    t0 = time.time()
    n = 0
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "x_m", "y_m", "angle_deg"])
        print(f"Logging marker {args.marker} to {args.out}. Press Ctrl+C to stop.")
        try:
            while True:
                p = read_marker(sock, args.marker)
                if p:
                    w.writerow([f"{time.time() - t0:.3f}", *p])
                    n += 1
                    if n % 25 == 0:
                        print(f"  {time.time() - t0:6.1f} s  x={p[0]:.3f} y={p[1]:.3f} a={p[2]:.0f}")
        except KeyboardInterrupt:
            pass
    print(f"Saved {n} poses.")


def build_path(wps, start, step=0.01):
    """Same geometry as follow.c: lines, and arcs tangent to the previous section."""
    n = len(wps)
    route = [(start - 1 + k) % n for k in range(n + 1)]
    pts, tx, ty = [], 0.0, 0.0
    for a_i, b_i in zip(route, route[1:]):
        ax, ay, _, turn = wps[a_i]
        bx, by, _, _ = wps[b_i]
        dx, dy = bx - ax, by - ay
        side = 1.0 if tx * dy - ty * dx > 0 else -1.0
        nx, ny = -ty * side, tx * side
        ab_n = dx * nx + dy * ny
        if turn and ab_n > 0.005:
            r = (dx * dx + dy * dy) / (2 * ab_n)
            cx, cy = ax + r * nx, ay + r * ny
            a0 = math.atan2(ay - cy, ax - cx)
            sweep = math.atan2(by - cy, bx - cx) - a0
            while side > 0 and sweep <= 0:
                sweep += 2 * math.pi
            while side < 0 and sweep >= 0:
                sweep -= 2 * math.pi
            m = max(1, int(abs(sweep) * r / step))
            pts += [(cx + r * math.cos(a0 + sweep * i / m), cy + r * math.sin(a0 + sweep * i / m)) for i in range(m)]
            ux, uy = (bx - cx) / r, (by - cy) / r
            tx, ty = -uy * side, ux * side
        else:
            length = math.hypot(dx, dy)
            tx, ty = dx / length, dy / length
            pts += [(ax + tx * s * step, ay + ty * s * step) for s in range(int(length / step))]
    pts.append(wps[route[-1]][:2])
    return pts


def lap_time(rows, final):
    """Time from the first move (> 2 cm from the start) to the first arrival within 3 cm of the final point.

    The start and the final waypoint are the same point, so the arrival only
    counts after the tractor went more than 50 cm away. Returns None if the
    tractor never arrived.
    """
    x0, y0 = rows[0][1], rows[0][2]
    moved = next((r[0] for r in rows if math.hypot(r[1] - x0, r[2] - y0) > 0.02), None)
    far = next((i for i, r in enumerate(rows) if math.hypot(r[1] - x0, r[2] - y0) > 0.5), None)
    if moved is None or far is None:
        return None
    arrived = next((r[0] for r in rows[far:] if math.hypot(r[1] - final[0], r[2] - final[1]) < 0.03), None)
    return arrived - moved if arrived else None


def cmd_plot(args):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(args.waypoints) as f:
        wps = [(float(r["x_m"]), float(r["y_m"]), float(r["angle_deg"]), r["turn"] == "1") for r in csv.DictReader(f)]
    sign, offset, d = read_defines()
    path = build_path(wps, args.start)
    fig, ax = plt.subplots(figsize=(7, 8))
    ax.plot([p[0] for p in path], [p[1] for p in path], color="0.6", lw=6, alpha=0.5, label="Desired route")
    for i, (x, y, a, turn) in enumerate(wps):
        ax.plot(x, y, "o", color="tab:red" if turn else "tab:blue", ms=7)
        ax.annotate(f"P{i + 1}", (x, y), textcoords="offset points", xytext=(6, 6), fontsize=10)
    for k, run in enumerate(args.runs):
        with open(run) as f:
            rows = [(float(r["t_s"]), *to_center(float(r["x_m"]), float(r["y_m"]), float(r["angle_deg"]), sign, offset, d))
                    for r in csv.DictReader(f)]
        t = lap_time(rows, path[-1])
        label = f"Run {k + 1}" + (f" ({t:.2f} s)" if t else " (did not finish)")
        print(f"{run}: lap time {t:.2f} s" if t else f"{run}: did not finish at the final waypoint")
        ax.plot([r[1] for r in rows], [r[2] for r in rows], lw=1.5, label=label)
    ax.plot([], [], "o", color="tab:red", label="Turn entrance")
    ax.plot([], [], "o", color="tab:blue", label="Turn exit")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Local positioning map: waypoints, route and measured trajectory")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"Saved {args.out}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--marker", type=int, default=3)
    p.add_argument("--start", type=int, default=14, help="start and final waypoint number")
    sub = p.add_subparsers(dest="cmd", required=True)
    p.add_argument("--marker-offset", type=float, help="turning center in front of the marker, m (default: estimated)")
    sub.add_parser("record")
    ex = sub.add_parser("extract")
    ex.add_argument("log")
    lg = sub.add_parser("log")
    lg.add_argument("out")
    pl = sub.add_parser("plot")
    pl.add_argument("runs", nargs="*")
    pl.add_argument("--waypoints", default=str(CSV))
    pl.add_argument("--out", default="route.png")
    args = p.parse_args()
    {"record": cmd_record, "extract": cmd_extract, "log": cmd_log, "plot": cmd_plot}[args.cmd](args)


if __name__ == "__main__":
    main()
