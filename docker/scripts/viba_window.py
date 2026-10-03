#!/usr/bin/env python3
"""Place ORB-SLAM3's untimestamped stage markers on the SIMULATION timeline.

ORB-SLAM3 prints "start VIBA 1", "end VIBA 2", "Not enough motion for
initializing" and friends as bare `cout`s with no timestamp, and rcl's own log
prefixes are SYSTEM time rather than the node clock -- so none of them can be
located in simulation time directly. The stereo-inertial node therefore emits a
0.5 Hz beacon carrying both clocks:

    [INFO] [1791051807.895476625] [...]: clock beacon: sim_t=115.600000 frame=75

Each marker is then bracketed between the surrounding beacons and reported with
that bracket, so the uncertainty is explicit instead of hidden behind a
real-time-factor extrapolation (which is only as good as the RTF being
constant, and it is not).

Why this matters: the converged-window comparison starts at VIBA 2 completion.
Getting that time wrong by seconds moves the window and therefore the headline
ATE, so it has to be measured, not estimated.

    ./viba_window.py /out/eval/<run>/orb_slam3.log
    ./viba_window.py <log> --json
    ./viba_window.py <log> --emit-t-start   # just the number, for evo --t_start
"""
from __future__ import annotations

import argparse
import bisect
import json
import re
import sys

ANSI = re.compile(r"\x1b\[[0-9;]*m")
LOGSTAMP = re.compile(r"^\[[A-Z]+\]\s*\[(\d+\.\d+)\]")
BEACON = re.compile(r"clock beacon: sim_t=([0-9.]+)\s+frame=(\d+)")
# Markers worth locating. Bare couts, in ORB-SLAM3's own wording.
MARKERS = (
    "start VIBA 1", "end VIBA 1",
    "start VIBA 2", "end VIBA 2",
    "Not enough motion for initializing",
    "not IMU meas",
    "Tracking LOST",
)


def parse(path: str) -> dict:
    text = ANSI.sub("", open(path, errors="replace").read())
    lines = text.splitlines()

    # (line_index, wall) for every rcl-stamped line, and the beacons among them.
    wall_at: list[tuple[int, float]] = []
    beacons: list[tuple[float, float]] = []      # (wall, sim)
    for i, raw in enumerate(lines):
        # Strip the launch-prefix, e.g. "[stereo_inertial-1] ".
        line = re.sub(r"^\[[^\]]+\]\s*", "", raw, count=1) if raw.startswith("[") else raw
        m = LOGSTAMP.match(line)
        if m:
            w = float(m.group(1))
            wall_at.append((i, w))
            b = BEACON.search(line)
            if b:
                beacons.append((w, float(b.group(1))))
    beacons.sort()

    def wall_near(idx: int) -> float | None:
        """Wall stamp of the nearest rcl-stamped line at or before idx."""
        if not wall_at:
            return None
        ks = [k for k, _ in wall_at]
        j = bisect.bisect_right(ks, idx) - 1
        return wall_at[j][1] if j >= 0 else wall_at[0][1]

    bw = [b[0] for b in beacons]

    def bracket(wall: float | None):
        """(sim_lo, sim_hi) beacons surrounding this wall time."""
        if wall is None or not beacons:
            return (None, None)
        j = bisect.bisect_left(bw, wall)
        lo = beacons[j - 1][1] if j - 1 >= 0 else None
        hi = beacons[j][1] if j < len(beacons) else None
        return (lo, hi)

    found: dict[str, list[dict]] = {m: [] for m in MARKERS}
    for i, raw in enumerate(lines):
        for m in MARKERS:
            if m in raw:
                w = wall_near(i)
                lo, hi = bracket(w)
                found[m].append({"line": i, "wall": w, "sim_lo": lo, "sim_hi": hi})

    out: dict = {
        "log": path,
        "beacons": len(beacons),
        "sim_span": [beacons[0][1], beacons[-1][1]] if beacons else None,
        "counts": {m: len(v) for m, v in found.items()},
        "markers": {m: v[:4] for m, v in found.items() if v},
    }
    # The converged window starts once VIBA 2 has finished. Use the UPPER
    # bracket: that is the first sim time at which the stage is certainly done,
    # so the window can never accidentally include part of the correction.
    e2 = found["end VIBA 2"]
    e1 = found["end VIBA 1"]
    out["t_start_converged"] = (e2[0]["sim_hi"] or e2[0]["sim_lo"]) if e2 else None
    out["t_start_after_viba1"] = (e1[0]["sim_hi"] or e1[0]["sim_lo"]) if e1 else None
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--emit-t-start", action="store_true",
                    help="print only t_start_converged (empty if unavailable)")
    args = ap.parse_args()

    r = parse(args.log)
    if args.emit_t_start:
        t = r["t_start_converged"]
        print("" if t is None else f"{t:.6f}")
        return 0 if t is not None else 1
    if args.json:
        print(json.dumps(r, indent=2))
        return 0

    print(f"beacons: {r['beacons']}"
          + (f"  sim {r['sim_span'][0]:.2f}..{r['sim_span'][1]:.2f} s" if r["sim_span"] else ""))
    if not r["beacons"]:
        print("  NO BEACONS -- this log predates the clock beacon, so markers\n"
              "  cannot be placed on the simulation timeline. Re-run with the\n"
              "  current stereo-inertial node.", file=sys.stderr)
    print("\nmarker counts:")
    for m, c in r["counts"].items():
        if c:
            print(f"  {m:38s} {c}")
    print("\nfirst occurrences (sim-time bracket):")
    for m, v in r["markers"].items():
        e = v[0]
        lo = "?" if e["sim_lo"] is None else f"{e['sim_lo']:.2f}"
        hi = "?" if e["sim_hi"] is None else f"{e['sim_hi']:.2f}"
        print(f"  {m:38s} sim {lo} .. {hi} s")
    def fmt(v):
        return "unavailable" if v is None else f"{v:.3f} s"

    print(f"\nconverged window starts at  : {fmt(r['t_start_converged'])}"
          "   (end VIBA 2; use as evo --t_start)")
    print(f"post-VIBA-1 window starts at: {fmt(r['t_start_after_viba1'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
