#!/usr/bin/env python3
"""Compare candidate driving models from their replayed modelV2 streams.

Input: a directory of *.zst modelV2 logs produced by replay_models.py, all replayed
over the SAME frames. Because the frame sequence is identical, differences in the
metrics below are attributable to the MODEL alone (no build / road confound).

Metrics (steering-quality oriented, higher-is-worse unless noted):
  jitter_dc   desiredCurvature 2-8 Hz band RMS (1e-4 1/m) -- planner hi-freq noise
              the felt low-speed wobble driver: lower is smoother
  jitter_path lateral position.y 2-8 Hz band RMS (m) -- path shimmer
  center_p95  |lane-center offset| p95 (m), both lane lines confident
              (left_y+right_y)/2 at near range: how well-centered
  clr_near%   fraction of frames nearest lane line < 0.9 m (lane-hug risk)
  lp_conf     mean min(leftProb,rightProb): model lane-line confidence (higher better)
  dc_range    p95-p5 of desiredCurvature (1e-4 1/m): responsiveness / travel

Run this on the device output (works anywhere with capnp+numpy; no GPU needed).
    python3 tools/ioniq6n_model_replay/compare_models.py /data/model_replay_out
"""
import glob
import os
import sys

import numpy as np

# standalone capnp read (no tqdm/openpilot deps needed for the analysis half)
import capnp
capnp.remove_import_hook()
CER = os.path.join(os.path.dirname(__file__), "../../cereal")
log_capnp = capnp.load(os.path.join(CER, "log.capnp"), imports=[CER, os.path.join(CER, "..")])

WARMUP = 20  # skip first N modelV2 frames (feature buffer / calibration settle)


def read_modelv2(path):
  import zstandard
  dctx = zstandard.ZstdDecompressor()
  with open(path, "rb") as f:
    data = dctx.stream_reader(f).read()
  it = log_capnp.Event.read_multiple_bytes(data)
  out = []
  while True:
    try:
      ev = next(it)
    except StopIteration:
      break
    except Exception:
      break
    if ev.which() == "modelV2":
      out.append(ev.modelV2)
  return out


def band_rms(sig, dt=0.05, lo=2.0, hi=8.0):
  sig = np.asarray(sig, float)
  if len(sig) < 32:
    return np.nan
  sig = sig - np.mean(sig)
  n = len(sig)
  f = np.fft.rfftfreq(n, dt)
  P = np.abs(np.fft.rfft(sig)) ** 2 / n
  m = (f >= lo) & (f <= hi)
  return float(np.sqrt(2.0 * np.sum(P[m]) / n)) if m.any() else np.nan


def metrics(models):
  models = models[WARMUP:]
  if len(models) < 40:
    return None
  dc = []      # desiredCurvature
  py = []      # position.y[0] near
  center = []  # (leftY+rightY)/2 near
  clr = []     # min(|leftY|,|rightY|) near
  lpconf = []
  for m in models:
    # NOTE: m.action.desiredCurvature is engagement-gated — it is 0 under replay (no
    # active lateral control), so it is useless here. The model's actual steering intent
    # lives in the PLAN: planned curvature = yaw_rate / v = orientationRate.z / velocity.x.
    # Floor v to avoid low-speed blowup. This reflects what the model wants to do,
    # independent of whether openpilot was engaged.
    try:
      v = m.velocity.x[0] if len(m.velocity.x) else 0.0
      yr = m.orientationRate.z[0] if len(m.orientationRate.z) else 0.0
      dc.append(yr / max(v, 3.0))
    except Exception:
      dc.append(np.nan)
    try:
      # y[0] is always ~0 (ego-relative); use a mid-horizon point (~index 10) where
      # lateral path actually moves, so band-RMS captures real path shimmer.
      yv = m.position.y
      py.append(yv[10] if len(yv) > 10 else yv[-1])
    except Exception:
      py.append(np.nan)
    try:
      ll = m.laneLines
      lp = m.laneLineProbs
      L = ll[1].y[0]; R = ll[2].y[0]
      if lp[1] > 0.5 and lp[2] > 0.5:
        center.append((L + R) / 2.0)
        clr.append(min(abs(L), abs(R)))
      lpconf.append(min(lp[1], lp[2]))
    except Exception:
      pass
  dc = np.asarray(dc, float) * 1e4
  res = {
    "n": len(models),
    "jitter_dc": band_rms(dc / 1e4) * 1e4,
    "jitter_path": band_rms(np.asarray(py, float)),
    "center_p95": np.percentile(np.abs(center), 95) if center else np.nan,
    "clr_near%": (np.mean(np.asarray(clr) < 0.9) * 100) if clr else np.nan,
    "lp_conf": np.nanmean(lpconf) if lpconf else np.nan,
    "dc_range": (np.nanpercentile(dc, 95) - np.nanpercentile(dc, 5)),
  }
  return res


def main():
  d = sys.argv[1] if len(sys.argv) > 1 else "/data/model_replay_out"
  files = sorted(glob.glob(os.path.join(d, "*.zst")))
  if not files:
    print(f"no *.zst in {d}")
    return
  meta = os.path.join(d, "run_meta.txt")
  if os.path.exists(meta):
    print(open(meta).read())
  rows = []
  for f in files:
    name = os.path.basename(f)[:-4]
    mv = read_modelv2(f)
    r = metrics(mv)
    if r is None:
      print(f"  {name}: too few frames ({len(mv)})")
      continue
    rows.append((name, r))

  if not rows:
    return
  hdr = ["model", "n", "jitter_dc", "jitter_path", "center_p95", "clr_near%", "lp_conf", "dc_range"]
  print(f"\n{'model':<24}{'n':>5}{'jitter_dc':>11}{'jit_path':>10}{'ctr_p95':>9}{'clr<.9%':>9}{'lp_conf':>9}{'dc_rng':>8}")
  print(f"{'':<24}{'':>5}{'(1e-4/m)':>11}{'(m)':>10}{'(m)':>9}{'':>9}{'':>9}{'(1e-4)':>8}")
  # rank by jitter_dc (primary smoothness proxy) ascending
  rows.sort(key=lambda kv: (np.nan_to_num(kv[1]["jitter_dc"], nan=1e9)))
  for name, r in rows:
    print(f"{name:<24}{r['n']:>5}{r['jitter_dc']:>11.2f}{r['jitter_path']:>10.3f}"
          f"{r['center_p95']:>9.2f}{r['clr_near%']:>9.1f}{r['lp_conf']:>9.2f}{r['dc_range']:>8.1f}")
  print("\nlower jitter_dc / jitter_path / clr_near% = smoother & safer; higher lp_conf = more confident.")


if __name__ == "__main__":
  main()
