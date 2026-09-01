#!/usr/bin/env python3
"""Aggregate a batch replay (many segments x many models) into a per-model summary.

Input: a batch root dir laid out as <root>/<segment>/<idx>_<name>.zst, as produced by
batch_replay.sh. For each model, averages the steering-quality metrics across all
segments it ran on, so a single ranking reflects the whole drive rather than one road.

    python3 aggregate.py /data/model_replay_batch
"""
import glob
import os
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from compare_models import read_modelv2, metrics  # noqa: E402

KEYS = ["jitter_dc", "jitter_path", "center_p95", "clr_near%", "lp_conf", "dc_range"]


def main():
  root = sys.argv[1] if len(sys.argv) > 1 else "/data/model_replay_batch"
  files = sorted(glob.glob(os.path.join(root, "*", "*.zst")))
  if not files:
    print(f"no *.zst under {root}/*/")
    return

  per_model = defaultdict(list)   # model -> list of per-segment metric dicts
  segs_seen = set()
  for f in files:
    model = os.path.basename(f)[:-4]           # e.g. 067_OPM10V3
    seg = os.path.basename(os.path.dirname(f))
    segs_seen.add(seg)
    r = metrics(read_modelv2(f))
    if r is not None:
      per_model[model].append(r)

  print(f"batch root: {root}")
  print(f"segments with data: {len(segs_seen)}   models: {len(per_model)}\n")

  rows = []
  for model, ms in per_model.items():
    agg = {}
    for k in KEYS:
      vals = [m[k] for m in ms if not np.isnan(m[k])]
      agg[k] = float(np.mean(vals)) if vals else float("nan")
    agg["n_seg"] = len(ms)
    rows.append((model, agg))

  rows.sort(key=lambda kv: np.nan_to_num(kv[1]["jitter_dc"], nan=1e9))

  print(f"{'model':<24}{'nseg':>5}{'jitter_dc':>11}{'jit_path':>10}{'ctr_p95':>9}{'clr<.9%':>9}{'lp_conf':>9}{'dc_rng':>8}")
  print(f"{'':<24}{'':>5}{'(1e-4/m)':>11}{'(m)':>10}{'(m)':>9}{'%':>9}{'':>9}{'(1e-4)':>8}")
  for model, a in rows:
    print(f"{model:<24}{a['n_seg']:>5}{a['jitter_dc']:>11.3f}{a['jitter_path']:>10.4f}"
          f"{a['center_p95']:>9.3f}{a['clr_near%']:>9.2f}{a['lp_conf']:>9.3f}{a['dc_range']:>8.2f}")
  print("\nmean across all segments per model.")
  print("lower jitter_dc (plan-curvature 2-8Hz) / jit_path / clr<.9% = smoother & safer;")
  print("lower dc_rng = calmer curvature; higher lp_conf = more confident lane lines.")

  # also dump machine-readable
  import json
  out = {m: a for m, a in rows}
  with open(os.path.join(root, "summary.json"), "w") as fp:
    json.dump(out, fp, indent=1)
  print(f"\nwrote {os.path.join(root, 'summary.json')}")


if __name__ == "__main__":
  main()
