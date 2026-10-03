#!/usr/bin/env python3
"""Aggregate ATE/RPE and rate statistics across repeated runs (run on the HOST).

Reports mean and spread, because a single run's ATE is not a result -- the whole
point of repeating is to show how much it moves.

    ./aggregate_results.py docker/out/experiment_YYYYmmdd-HHMMSS
"""
from __future__ import annotations

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
        r = parse_rates(root / f"run{i}_rates.log")
        per_run.append((f"run{i}", {"ape_o": ape_o.get("rmse", float('nan')),
                                    "rpe_o": rpe_o.get("rmse", float('nan')),
                                    "ape_c": ape_c.get("rmse", float('nan')),
                                    "rpe_c": rpe_c.get("rmse", float('nan'))}, r))
        for k, v in (("ORB-SLAM3 ATE rmse", ape_o.get("rmse")),
                     ("ORB-SLAM3 RPE rmse", rpe_o.get("rmse")),
                     ("COVINS    ATE rmse", ape_c.get("rmse")),
                     ("COVINS    RPE rmse", rpe_c.get("rmse"))):
            if v is not None:
                collected.setdefault(k, []).append(v)
        for k, v in r.items():
            rates.setdefault(k, []).append(v)

    print("\nPER-RUN (ATE/RPE RMSE, metres)")
    print(f"  {'run':<6} {'ORB ATE':>9} {'ORB RPE':>9} {'COV ATE':>9} {'COV RPE':>9} {'RTF':>7} {'sim s':>7}")
    for name, m, r in per_run:
        def f(x): return f"{x:9.3f}" if x == x else "      n/a"
        print(f"  {name:<6}{f(m['ape_o'])}{f(m['rpe_o'])}{f(m['ape_c'])}{f(m['rpe_c'])}"
              f" {r.get('rtf', float('nan')):7.3f} {r.get('sim_s', float('nan')):7.1f}")

    print("\nAGGREGATE  (mean +/- sample stdev [min..max])")
    for k in ("ORB-SLAM3 ATE rmse", "ORB-SLAM3 RPE rmse",
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
