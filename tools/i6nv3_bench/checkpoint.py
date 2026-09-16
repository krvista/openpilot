#!/usr/bin/env python3
"""One route -> one checkpoint row (JSON line + markdown), single pass over the rlogs. The trend input for the October
fine-tuning (ROUTE8_REPORT §21 item 4). Metrics: build, date, boot (logger start, safety armed, cache used, FW query),
panda faults, core 0/2 p99, lane confidence, tracking error (realized - intended: straight / curves / 30-45), 30-45 km/h
hands-on share, grabs per latActive minute, post-release 0-2 s |gap|>3 share, truly hands-off |gap|>3 share, plan-quiet 1-s
transmitted-angle swing p95, laneDropout runs.
usage: PYTHONPATH=$PWD:$PWD/opendbc_repo python3 tools/i6nv3_bench/checkpoint.py <route> [--logdir DIR] [--append trend.jsonl]"""
import argparse
import glob
import json
import os
import sys
import datetime
import numpy as np
sys.path.insert(0, os.getcwd()); sys.path.insert(0, os.path.join(os.getcwd(), "opendbc_repo"))
from openpilot.tools.lib.logreader import LogReader

ap = argparse.ArgumentParser(); ap.add_argument("route"); ap.add_argument("--logdir", default="/home/user/drivelog/drivelog")
ap.add_argument("--append", default=None); a = ap.parse_args()
files = sorted(glob.glob(f"{a.logdir}/*_{a.route}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
assert files, "no segments"
out = dict(route=a.route, segs=len(files))
# state
v = 0; press = 0; tq = 0; blk = 0; lat = 0; gap = 0; gain = None; wire = None; lc = "off"; cmd = 0; kd = 0
rh = []; kh = []; last_press = -1e9; prev_press = 0; last_rel = None
band = []; rel = []; ho = []; swings = []; probs = []; track = []; cores = {0: [], 2: []}
faults = set(); t_logger = None; t_safety = None; cache = False; fwq = False; ld_runs = 0; ld_prev = False; gps = None; commit = None
for f in files:
  seg = int(f.split("--")[-2])
  for m in LogReader(f):
    w = m.which(); t = m.logMonoTime / 1e9
    if t_logger is None: t_logger = t
    if w == "initData" and commit is None:
      commit = str(m.initData.gitCommit)[:9]
    elif w == "logMessage" and seg == 0:
      s = str(m.logMessage)
      if "Using cached CarParams" in s: cache = True
      if "Getting VIN" in s: fwq = True
    elif w == "pandaStates" and len(m.pandaStates):
      ps = m.pandaStates[0]
      for x in ps.faults: faults.add(str(x))
      if t_safety is None and str(ps.safetyModel) == "hyundaiCanfd": t_safety = t
    elif w == "deviceState" and seg > 0:
      cu = list(m.deviceState.cpuUsagePercent)
      for c in cores:
        if c < len(cu): cores[c].append(cu[c])
    elif w in ("gpsLocationExternal", "gpsLocation") and gps is None:
      g = getattr(m, w)
      if g.latitude != 0: gps = (g.latitude, g.longitude, g.unixTimestampMillis)
    elif w == "sendcan":
      for c in m.sendcan:
        if c.address == 272 and len(c.dat) >= 13:
          d = bytes(c.dat); raw = ((d[10] >> 2) | (d[11] << 6)) & 0x3FFF
          if raw & 0x2000: raw -= 0x4000
          wire = raw * 0.1; gain = d[12] * 0.004; rh.append((t, wire)); rh = [h for h in rh if t - h[0] <= 1.05]
    elif w == "carControl":
      lat = m.carControl.latActive; cmd = m.carControl.actuators.steeringAngleDeg
    elif w == "controlsState":
      gap = m.controlsState.steerCmdGapDeg; kd = m.controlsState.desiredCurvature; kh.append((t, kd)); kh = [h for h in kh if t - h[0] <= 1.05]
    elif w == "onroadEvents":
      names = {str(e.name) for e in m.onroadEvents}
      ld = "laneDropout" in names
      if ld and not ld_prev: ld_runs += 1
      ld_prev = ld
    elif w == "modelV2":
      md = m.modelV2; lc = str(md.meta.laneChangeState)
      if lat and v * 3.6 >= 30 and len(md.laneLineProbs) >= 4:
        p = min(md.laneLineProbs[1], md.laneLineProbs[2]); probs.append(p)
        if p > 0.5 and not press and not blk and lc == "off":
          ll = md.laneLines; xs = np.array(ll[1].x); i15 = int(np.argmin(np.abs(xs - 15.0)))
          y0 = (ll[1].y[0] + ll[2].y[0]) / 2; c15 = (ll[1].y[i15] + ll[2].y[i15]) / 2
          p15 = float(np.interp(15.0, np.array(md.position.x), np.array(md.position.y))) if len(md.position.x) >= 20 else np.nan
          width = ll[2].y[0] - ll[1].y[0]
          if 2.7 < width < 4.0 and np.isfinite(p15):
            track.append((v * 3.6, abs(kd) * 1e3, y0 + (p15 - c15)))
    elif w == "carState":
      c = m.carState; v = c.vEgo; press = c.steeringPressed; tq = abs(c.steeringTorque); blk = c.leftBlinker or c.rightBlinker
      vk = v * 3.6
      if press: last_press = t
      if press and not prev_press and lat and not blk and lc == "off" and 30 <= vk < 45 and last_rel is not None: rel.append(t - last_rel)
      if not press and prev_press: last_rel = t
      prev_press = press
      if gain is not None and lat and not blk and lc == "off" and 30 <= vk < 45:
        band.append((int(press), tq, gain, abs(gap), t - last_press))
        if not press and len(kh) > 50 and abs(kh[-1][1] - kh[0][1]) < 0.3e-3 and abs(kd) < 0.5e-3 and len(rh) > 50:
          swings.append(max(h[1] for h in rh) - min(h[1] for h in rh))
B = np.array(band) if band else np.zeros((0, 5)); T = np.array(track) if track else np.zeros((0, 3))
def med(x): return float(np.median(x)) if len(x) else None
def pct(x, q): return float(np.percentile(x, q)) if len(x) else None
out.update(commit=commit, date=(datetime.datetime.utcfromtimestamp(gps[2] / 1000) + datetime.timedelta(hours=9)).strftime("%Y-%m-%d %H:%M KST") if gps else None,
           boot_logger_s=round(t_logger, 1) if t_logger else None, boot_ready_s=round(t_safety, 1) if t_safety else None, cache_used=cache, fw_query=fwq,
           panda_faults=sorted(faults), core0_p99=pct(cores[0], 99), core2_p99=pct(cores[2], 99),
           lane_prob_p50=med(probs), lane_both_gt08=float(np.mean(np.array(probs) > 0.8)) if probs else None,
           track_straight_med=med(np.abs(T[T[:, 1] < 0.3, 2])) if len(T) else None, track_curve_med=med(np.abs(T[T[:, 1] >= 0.5, 2])) if len(T) else None,
           track_curve_p90=pct(np.abs(T[T[:, 1] >= 0.5, 2]), 90) if len(T) else None, track_3045_med=med(np.abs(T[(T[:, 0] >= 30) & (T[:, 0] < 45), 2])) if len(T) else None,
           band_min=round(len(B) / 6000, 1), hands_on_share=float(np.mean((B[:, 0] == 0) & (B[:, 1] >= 100))) if len(B) else None,
           grabs_per_min=round(len(rel) / max(len(B) / 6000, 1e-3), 1) if len(B) else None,
           regrab_3s=float(np.mean(np.array(rel) < 3)) if rel else None,
           post_release_gap3=float(np.mean(B[(B[:, 0] == 0) & (B[:, 4] < 2), 3] > 3)) if len(B) and ((B[:, 0] == 0) & (B[:, 4] < 2)).any() else None,
           handsoff_gap3=float(np.mean(B[(B[:, 0] == 0) & (B[:, 1] < 50) & (B[:, 4] >= 3), 3] > 3)) if len(B) and ((B[:, 0] == 0) & (B[:, 1] < 50) & (B[:, 4] >= 3)).any() else None,
           swing_p95=pct(swings, 95), lane_dropout_runs=ld_runs)
print(json.dumps(out, ensure_ascii=False))
fmt = lambda x, n=2: ("-" if x is None else (f"{x:.{n}f}" if isinstance(x, float) else str(x)))
print("| " + " | ".join([out["route"], str(out["commit"]), str(out["date"]), f"{fmt(out['boot_logger_s'],1)}/{fmt(out['boot_ready_s'],1)}", "cache" if cache else ("fwq" if fwq else "-"),
      str(len(faults)), f"{fmt(out['core0_p99'],0)}/{fmt(out['core2_p99'],0)}", fmt(out['lane_both_gt08']), f"{fmt(out['track_straight_med'],3)}/{fmt(out['track_curve_med'],3)}/{fmt(out['track_curve_p90'],3)}/{fmt(out['track_3045_med'],3)}",
      fmt(out['hands_on_share']), fmt(out['grabs_per_min'],1), fmt(out['regrab_3s']), fmt(out['post_release_gap3']), fmt(out['handsoff_gap3']), fmt(out['swing_p95'],1), str(ld_runs)]) + " |")
if a.append:
  with open(a.append, "a") as fh: fh.write(json.dumps(out, ensure_ascii=False) + "\n")
