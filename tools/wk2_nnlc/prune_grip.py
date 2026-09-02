#!/usr/bin/env python3
"""Drop 'silent co-steer' frames from an nnlc-extract CSV: driver column torque
below the car's steeringPressed threshold but large enough to contaminate the
torque->lateral-accel mapping (WK2 drivelog: 49% of 'not pressed' active frames
carried 40-120 units of driver torque).

  python3 prune_grip.py in.csv -o out.csv [--max-torque 40] [--pad-frames 10]
"""
import argparse
import numpy as np, pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument("input"); ap.add_argument("-o", "--output", required=True)
ap.add_argument("--max-torque", type=float, default=40.0, help="|steering_torque| above this is pruned")
ap.add_argument("--pad-frames", type=int, default=10, help="also drop this many frames after each grip episode (settling)")
a = ap.parse_args()

df = pd.read_csv(a.input) if a.input.endswith(".csv") else pd.read_parquet(a.input)
grip = (df["steering_torque"].abs() > a.max_torque) | df["steering_pressed"].astype(bool)
if a.pad_frames > 0:
    g = grip.to_numpy().copy()
    idx = np.flatnonzero(g)
    for k in range(1, a.pad_frames + 1):
        j = idx + k
        g[j[j < len(g)]] = True
    grip = pd.Series(g, index=df.index)
kept = df[~grip]
print(f"rows {len(df)} -> {len(kept)}  (pruned {grip.mean()*100:.1f}%: |torque|>{a.max_torque} or pressed, +{a.pad_frames} settling frames)")
(kept.to_csv if a.output.endswith(".csv") else kept.to_parquet)(a.output, index=False)
