#!/usr/bin/env python3
"""Per-route audit of an nnlc-extract CSV (+ the rlog tree it came from):
build commit (seg-0 initData), NNLC on/off inferred from the logged feedforward
(linear FF tracks desired lat accel almost perfectly; the NN does not), active
time, grip stats -> keep/drop verdict and a keep-list for the grip pruner.

  python3 route_audit.py lat.csv --rlogs ./data -o keep_routes.txt [--expect-commit d67c5fe]
"""
import argparse, glob, os, sys
import numpy as np, pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument("csv"); ap.add_argument("--rlogs", default=None, help="rlog root (for seg-0 initData)")
ap.add_argument("-o", "--output", default="keep_routes.txt")
ap.add_argument("--expect-commit", default=None, help="drop routes not on this commit prefix")
ap.add_argument("--min-active-s", type=float, default=120.0)
ap.add_argument("--corr-linear", type=float, default=0.975, help="corr(ff, desired) above this = linear FF (NNLC off)")
a = ap.parse_args()

df = pd.read_csv(a.csv) if a.csv.endswith(".csv") else pd.read_parquet(a.csv)
if "route_id" not in df.columns:
    sys.exit("CSV has no route_id column (use the patched extractor)")

def seg0_commit(route):
    if not a.rlogs:
        return ""
    for f in glob.glob(os.path.join(a.rlogs, "**", f"*{route}--0", "rlog.zst"), recursive=True) + \
             glob.glob(os.path.join(a.rlogs, "**", f"*{route}--0--rlog.zst"), recursive=True):
        try:
            from nnlc_tools.logreader import LogReader
            for m in LogReader(f):
                if m.which() == "initData":
                    return str(m.initData.gitCommit)[:9]
        except Exception as ex:
            return f"err:{ex}"
    return ""

keep = []
print(f"{'route':28} {'commit':10} {'FF':7} {'corr':6} {'active_s':9} {'grip_p90':8} verdict")
for route, g in df.groupby("route_id", sort=True):
    act = g[g["active"].astype(bool)]
    active_s = len(act) / 100.0
    sel = act[act["desired_lateral_accel"].abs() > 0.3]
    corr = float(sel["ff"].corr(sel["desired_lateral_accel"])) if len(sel) > 200 else float("nan")
    ff_state = "n/a" if np.isnan(corr) else ("LINEAR" if corr >= a.corr_linear else "NN")
    grip_p90 = float(np.percentile(act["steering_torque"].abs(), 90)) if len(act) else float("nan")
    commit = seg0_commit(route)
    reasons = []
    if ff_state == "NN": reasons.append("NNLC on")
    if active_s < a.min_active_s: reasons.append(f"active<{a.min_active_s:.0f}s")
    if a.expect_commit and commit and not commit.startswith(a.expect_commit): reasons.append("wrong build")
    verdict = "KEEP" if not reasons else "DROP(" + ",".join(reasons) + ")"
    if verdict == "KEEP": keep.append(route)
    print(f"{route:28} {commit or '-':10} {ff_state:7} {corr:6.3f} {active_s:9.0f} {grip_p90:8.0f} {verdict}")
open(a.output, "w").write("\n".join(keep) + ("\n" if keep else ""))
print(f"\nkeep {len(keep)}/{df['route_id'].nunique()} routes -> {a.output}")
