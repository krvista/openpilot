#!/usr/bin/env python3
"""Foreground driver with a single overall progress bar over (segment x model) pairs.

Runs in the foreground and shows one tqdm bar for the whole job — how many
(segment, model) replays are done, percent, and ETA. Each replay's own verbose output
(the per-model message loop, fingerprint logs) goes to a per-run log file instead of the
screen. Resumable: pairs whose output already exists advance the bar instantly.

    python3 run_all.py --routes "00000003--021caa3877 00000006--e8efa47d38" --samples 10
    python3 run_all.py --samples all --routes "00000006--e8efa47d38"   # one trip, all segs
    python3 run_all.py                                                 # all routes, 3/route

Only FULLY-CACHED models are used by default (auto-detected) so it never stalls on a
download. Override with --indices. Tail the log for detail:  tail -f <out-dir>/run.log
"""
import argparse
import glob
import os
import re
import subprocess
import sys
from pathlib import Path

try:
  from tqdm import tqdm
except Exception:
  tqdm = None

HARNESS = os.path.dirname(os.path.abspath(__file__))


def cached_indices():
  out = subprocess.run([sys.executable, f"{HARNESS}/replay_models.py", "--check"],
                       capture_output=True, text=True).stdout
  for line in out.splitlines():
    if line.startswith("fully cached indices:"):
      return [int(x) for x in line.split(":", 1)[1].replace(" ", "").split(",") if x.strip()]
  return []


def route_of(name):
  return re.sub(r"--\d+$", "", name)


def select_segments(root, routes, samples):
  segdirs = [os.path.dirname(p) for p in glob.glob(os.path.join(root, "*", "fcamera.hevc"))]
  by_route = {}
  for d in segdirs:
    by_route.setdefault(route_of(os.path.basename(d)), []).append(d)
  if routes:
    wanted = [r.strip() for r in re.split(r"[,\s]+", routes) if r.strip()]
  else:
    wanted = sorted(by_route)
  out = []
  for r in wanted:
    segs = sorted((d for d in by_route.get(r, []) if not d.endswith("--0")),
                  key=lambda d: int(d.rsplit("--", 1)[1]))
    n = len(segs)
    if n == 0:
      continue
    if str(samples) == "all":
      out.extend(segs)
    else:
      N = int(samples)
      idxs = sorted({min(k * n // (2 * N), n - 1) for k in range(1, 2 * N, 2)})
      out.extend(segs[i] for i in idxs)
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--root", default="/data/media/0/realdata")
  ap.add_argument("--routes", default="", help="space/comma-separated route ids; empty = all")
  ap.add_argument("--samples", default="3", help="segments per route: a number, or 'all'")
  ap.add_argument("--indices", default="", help="comma-separated model idxs; empty = all cached")
  ap.add_argument("--end", type=int, default=200)
  ap.add_argument("--out-dir", default="/data/model_replay_diverse")
  args = ap.parse_args()

  models = [int(x) for x in args.indices.split(",") if x.strip()] or cached_indices()
  if not models:
    print("no models (none cached and --indices empty)"); return
  segs = select_segments(args.root, args.routes, args.samples)
  if not segs:
    print("no segments selected"); return

  out_dir = Path(args.out_dir); out_dir.mkdir(parents=True, exist_ok=True)
  log_path = out_dir / "run.log"
  work = [(s, m) for s in segs for m in models]

  print(f"segments={len(segs)}  models={models}  pairs={len(work)}")
  print(f"detail log: {log_path}   (tail -f to watch)")

  bar = tqdm(work, unit="replay", dynamic_ncols=True) if tqdm else work
  done = skipped = failed = 0
  with open(log_path, "a") as log:
    for seg, m in bar:
      name = os.path.basename(seg)
      seg_out = out_dir / name
      if list(seg_out.glob(f"{m:03d}_*.zst")):
        skipped += 1
        if tqdm:
          bar.set_postfix_str(f"{name} idx={m} (cached)")
        continue
      if tqdm:
        bar.set_postfix_str(f"{name} idx={m}")
      log.write(f"\n===== {name} idx={m} =====\n"); log.flush()
      env = {**os.environ, "REPLAY_QUIET": "1"}
      r = subprocess.run([sys.executable, f"{HARNESS}/replay_models.py",
                          "--seg-dir", seg, "--indices", str(m),
                          "--end", str(args.end), "--out-dir", str(seg_out)],
                         stdout=log, stderr=subprocess.STDOUT, env=env)
      if list(seg_out.glob(f"{m:03d}_*.zst")):
        done += 1
      else:
        failed += 1
        if tqdm:
          bar.write(f"  [fail] {name} idx={m} (see {log_path})")

  print(f"\nran={done} skipped={skipped} failed={failed}")
  print("aggregating ...")
  subprocess.run([sys.executable, f"{HARNESS}/aggregate.py", str(out_dir)])


if __name__ == "__main__":
  main()
