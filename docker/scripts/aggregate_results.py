#!/usr/bin/env python3
"""Aggregate ATE/RPE and rate statistics across repeated runs (run on the HOST).

Reports mean and spread, because a single run's ATE is not a result -- the whole
point of repeating is to show how much it moves.

    ./aggregate_results.py docker/out/experiment_YYYYmmdd-HHMMSS
"""
from __future__ import annotations

import json
import re
import statistics
import sys
from pathlib import Path

METRIC = re.compile(r"^\s*(max|mean|median|min|rmse|sse|std)\s+([0-9.eE+-]+)\s*$")


def parse_evo(path: Path) -> dict[str, float]:
    out: dict[str, float] = {}
    if not path.exists():
        return out
    for line in path.read_text(errors="replace").splitlines():
        m = METRIC.match(line)
        if m:
            out.setdefault(m.group(1), float(m.group(2)))
    return out


def parse_rates(path: Path) -> dict[str, float]:
    out: dict[str, float] = {}
    if not path.exists():
        return out
    txt = path.read_text(errors="replace")
    m = re.search(r"real-time factor\s*:\s*([0-9.]+)", txt)
    if m:
        out["rtf"] = float(m.group(1))
    m = re.search(r"simulated duration\s*:\s*([0-9.]+)", txt)
    if m:
        out["sim_s"] = float(m.group(1))

    # Fallback: if the RUN SUMMARY is missing (the monitor was killed before it
    # could print), recover the figures from the last per-window progress line,
    # which has the form
    #   [  165.2s wall |    47.6s sim | RTF 0.288]
    if "rtf" not in out or "sim_s" not in out:
        wins = re.findall(
            r"\[\s*([0-9.]+)s wall \|\s*([0-9.]+)s sim \| RTF\s*([0-9.]+)\]", txt)
        if wins:
            _wall, sim, rtf = wins[-1]
            out.setdefault("rtf", float(rtf))
            out.setdefault("sim_s", float(sim))
    for name, hz in re.findall(r"^\s+(/\S+)\s+([0-9.]+) Hz wall", txt, re.M):
        out[name] = float(hz)
    return out


def stats(vals: list[float]) -> str:
    vals = [v for v in vals if v == v]
    if not vals:
        return "n/a"
    if len(vals) == 1:
        return f"{vals[0]:.3f} (n=1)"
    return (f"{statistics.mean(vals):.3f} +/- {statistics.stdev(vals):.3f}"
            f"  [{min(vals):.3f}..{max(vals):.3f}] n={len(vals)}")


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    root = Path(sys.argv[1])
    rundirs_file = root / "rundirs.txt"
    rundirs = [Path("docker/out/eval") / Path(p.strip()).name
               for p in rundirs_file.read_text().splitlines() if p.strip()] \
        if rundirs_file.exists() else []

    print("=" * 72)
    print(f"EXPERIMENT SUMMARY  ({root.name})")
    print("=" * 72)

    collected: dict[str, list[float]] = {}
    rates: dict[str, list[float]] = {}
    per_run: list[tuple[str, dict[str, float], dict[str, float]]] = []

    for i, rd in enumerate(rundirs, 1):
        ape_o = parse_evo(rd / "ape_orbslam3.txt")
        rpe_o = parse_evo(rd / "rpe_orbslam3.txt")
        ape_c = parse_evo(rd / "ape_covins.txt")
        rpe_c = parse_evo(rd / "rpe_covins.txt")
        ape_pi = parse_evo(rd / "ape_orbslam3_postinit.txt")
        rpe_pi = parse_evo(rd / "rpe_orbslam3_postinit.txt")
        r = parse_rates(root / f"run{i}_rates.log")

        # Prefer the POST-HOC bag analysis for RTF/duration/path: the live
        # monitor can be starved or miss discovery entirely, and Gazebo's own
        # real_time_factor field is instantaneous and bimodal.
        bagf = root / f"run{i}_bag.json"
        if bagf.exists():
            try:
                b = json.loads(bagf.read_text())
                for k_src, k_dst in (("rtf", "rtf"), ("sim_s", "sim_s"),
                                     ("path_length_m", "path_m"),
                                     ("wall_s", "wall_s")):
                    if k_src in b:
                        r[k_dst] = float(b[k_src])
            except Exception:
                pass

        init = {}
        initf = root / f"run{i}_imu_init.txt"
        if initf.exists():
            for line in initf.read_text().splitlines():
                if "=" in line:
                    k, v = line.split("=", 1)
                    try:
                        init[k.strip()] = float(v)
                    except ValueError:
                        init[k.strip()] = v.strip()
        per_run.append((f"run{i}", {
            "ape_o": ape_o.get("rmse", float('nan')),
            "rpe_o": rpe_o.get("rmse", float('nan')),
            "ape_c": ape_c.get("rmse", float('nan')),
            "rpe_c": rpe_c.get("rmse", float('nan')),
            "ape_pi": ape_pi.get("rmse", float('nan')),
            "rpe_pi": rpe_pi.get("rmse", float('nan')),
            "init_delay": init.get("imu_init_delay_s", float('nan')),
        }, r))
        for k, v in (("ORB-SLAM3 ATE rmse", ape_o.get("rmse")),
                     ("ORB-SLAM3 RPE rmse", rpe_o.get("rmse")),
                     ("ORB ATE post-IMU-init", ape_pi.get("rmse")),
                     ("ORB RPE post-IMU-init", rpe_pi.get("rmse")),
                     ("IMU init delay (s)", init.get("imu_init_delay_s")
                      if isinstance(init.get("imu_init_delay_s"), float) else None),
                     ("COVINS    ATE rmse", ape_c.get("rmse")),
                     ("COVINS    RPE rmse", rpe_c.get("rmse"))):
            if v is not None:
                collected.setdefault(k, []).append(v)
        for k, v in r.items():
            rates.setdefault(k, []).append(v)

    print("\nPER-RUN  (RMSE in metres; RTF/sim/path are POST-HOC from the bag)")
    print(f"  {'run':<6} {'ORB ATE':>8} {'ORB RPE':>8} {'postATE':>8} {'postRPE':>8}"
          f" {'IMUinit':>8} {'COV ATE':>8} {'COV RPE':>8} {'RTF':>6} {'sim s':>6} {'path m':>7}")
    for name, m, r in per_run:
        def f(x, w=8): return f"{x:{w}.3f}" if x == x else " " * (w - 3) + "n/a"
        print(f"  {name:<6}{f(m['ape_o'])}{f(m['rpe_o'])}{f(m['ape_pi'])}{f(m['rpe_pi'])}"
              f"{f(m['init_delay'])}{f(m['ape_c'])}{f(m['rpe_c'])}"
              f" {r.get('rtf', float('nan')):6.3f} {r.get('sim_s', float('nan')):6.1f}"
              f" {r.get('path_m', float('nan')):7.2f}")

    print("\nAGGREGATE  (mean +/- sample stdev [min..max])")
    for k in ("ORB-SLAM3 ATE rmse", "ORB-SLAM3 RPE rmse",
              "ORB ATE post-IMU-init", "ORB RPE post-IMU-init",
              "IMU init delay (s)",
              "COVINS    ATE rmse", "COVINS    RPE rmse"):
        if k in collected:
            print(f"  {k:<22} {stats(collected[k])}")

    print("\nMEASURED RATES / RTF  (mean +/- stdev)")
    for k in sorted(rates):
        label = "real-time factor" if k == "rtf" else ("simulated duration (s)" if k == "sim_s" else k)
        print(f"  {label:<44} {stats(rates[k])}")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    sys.exit(main())
