#!/usr/bin/env python3
"""Analytic p(used) baseline (PATCHES s63): additive logistic model over binned features.

    fit_usage.py fit  <out_model.json> <records.jsonl ...>      # train
    fit_usage.py eval <model.json>     <records.jsonl ...>      # held-out metrics

Features per (frame, landmark, camera) record (records.jsonl from ig_decomposition):
  view_angle_deg bins, depth bins, border_px bins, and n_pred_cams (1 or 2: in how
  many cameras the landmark is predicted visible at that frame -- the
  both-cameras vs one-camera term). One-hot per bin, additive in logit (a binned
  table without interactions), L2-regularized IRLS, fitted on training flights only.
The model file holds the bin edges and coefficients so C++ can apply it."""
import json, sys, collections
import numpy as np

EDGES = {"view_angle_deg": [0, 1, 2, 5, 10, 1e9], "depth": [0, 2, 4, 8, 12, 1e9], "border_px": [0, 20, 50, 100, 1e9]}
KEYS = ["view_angle_deg", "depth", "border_px"]


def load(files):
    R = [json.loads(l) for f in files for l in open(f)]
    n = collections.Counter((r["t"], r["feature_id"]) for r in R)
    for r in R:
        r["n_pred_cams"] = n[(r["t"], r["feature_id"])]
    return R


def bin_of(x, e):
    if x is None:
        return len(e) - 1  # extra "unknown" bin (e.g. view angle for non-anchored)
    return int(np.clip(np.searchsorted(e, x, side="right") - 1, 0, len(e) - 2))


def design(R):
    cols = []
    for k in KEYS:
        nb = len(EDGES[k])  # (len-1) bins + 1 unknown
        b = np.array([bin_of(r[k], EDGES[k]) for r in R])
        cols.append(np.eye(nb)[b][:, 1:])  # drop first bin (reference)
    cols.append(np.array([[1.0 if r["n_pred_cams"] == 2 else 0.0] for r in R]))
    return np.hstack([np.ones((len(R), 1))] + cols)


def fit(X, y, lam=1.0, iters=50):
    w = np.zeros(X.shape[1])
    for _ in range(iters):
        p = 1 / (1 + np.exp(-X @ w))
        W = p * (1 - p)
        H = X.T @ (X * W[:, None]) + lam * np.eye(X.shape[1])
        g = X.T @ (p - y) + lam * w
        step = np.linalg.solve(H, g)
        w -= step
        if np.abs(step).max() < 1e-9:
            break
    return w


def metrics(p, y):
    p = np.clip(p, 1e-6, 1 - 1e-6)
    ll = -np.mean(y * np.log(p) + (1 - y) * np.log(1 - p))
    return {"n": int(len(y)), "logloss": float(ll), "brier": float(np.mean((p - y) ** 2)), "mean_p": float(p.mean()), "mean_y": float(y.mean())}


if __name__ == "__main__":
    mode, path, files = sys.argv[1], sys.argv[2], sys.argv[3:]
    R = load(files)
    X, y = design(R), np.array([r["used"] for r in R], float)
    if mode == "fit":
        w = fit(X, y)
        json.dump({"edges": EDGES, "keys": KEYS, "coef": w.tolist(),
                   "layout": "intercept, then per key one-hot of bins 1..len(edges)-1 (bin len(edges)-1 = unknown), then both_cams",
                   "train_files": files, "train": metrics(1 / (1 + np.exp(-X @ w)), y)}, open(path, "w"), indent=1)
        print("fit:", json.load(open(path))["train"])
    else:
        m = json.load(open(path)); w = np.array(m["coef"])
        p = 1 / (1 + np.exp(-X @ w))
        base = metrics(np.full_like(y, np.mean(m["train"]["mean_y"])), y)
        print("held-out model   :", metrics(p, y)); print("held-out constant:", base)
        # calibration
        for a, b in [(0, .5), (.5, .8), (.8, .9), (.9, .95), (.95, 1.01)]:
            s = (p >= a) & (p < b)
            if s.sum(): print(f"   p in [{a},{b}): n={s.sum():6d} predicted {p[s].mean():.3f} observed {y[s].mean():.3f}")
        nc = np.array([r["n_pred_cams"] for r in R])
        for c in (1, 2):
            s = nc == c
            if s.sum(): print(f"   n_pred_cams={c}: n={s.sum()} predicted {p[s].mean():.3f} observed {y[s].mean():.3f}")
        va = np.array([np.nan if r["view_angle_deg"] is None else r["view_angle_deg"] for r in R])
        for a, b in [(0, 5), (5, 10), (10, 1e9)]:
            s = (va >= a) & (va < b)
            if s.sum(): print(f"   view angle [{a},{b}): n={s.sum()} predicted {p[s].mean():.3f} observed {y[s].mean():.3f}")
