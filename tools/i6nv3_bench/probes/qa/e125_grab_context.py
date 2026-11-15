"""E125 (report §91): why does the driver take the wheel? Every EPS press onset (steeringPressed rising after >= 1.5 s
released) while op steers, >= 30 km/h, split into lane changes (blinker / lane-change state within +-3 s) and the rest.
For the rest, per speed band: road context from the model curvature (straight / entry / in curve / exit), which way the
driver moved the wheel in the next 0.7 s relative to the curve (tighten = further into the curve, widen = out of it;
straights: toward / away from the lane centre), and how far the wheel was behind op's own command at the onset.
argv: S builds_csv"""
import glob, sys
import numpy as np
S = sys.argv[1]; BUILDS = sys.argv[2].split(",")
VB = [(30, 55), (55, 70), (70, 90), (90, 200)]
P_OFF = {"98e912d": 0.0, "31520b9": -0.06, "bf36aa2": -0.08, "a0108a4": 0.0}
for b in BUILDS:
  rows = []; mins = np.zeros(len(VB)); nlc = np.zeros(len(VB))
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True)); n = len(d['T']); r = f[-12:-4]
    if str(d['commit'])[:7] != b or n < 2000: continue
    try:
      P = dict(np.load(f"{S}/la/plan_{r}.npz")); lp = np.load(f"{S}/vm/{r}.npz", allow_pickle=True)['lp']
    except Exception: continue
    T = d['T']; v = d['v']; j = np.clip(np.searchsorted(P['T'], T), 0, len(P['T']) - 1)
    l1 = np.array([np.interp(0.0, x, y) for x, y in zip(P['LX'], P['L1'])])[j]; l2 = np.array([np.interp(0.0, x, y) for x, y in zip(P['LX'], P['L2'])])[j]
    pr = np.minimum(P['PR'][:, 1], P['PR'][:, 2])[j]; lane_off = -(l1 + l2) / 2 - P_OFF.get(b, 0.0)       # < 0 = car left of centre
    off = np.interp(T, lp[:, 0], lp[:, 3]) if len(lp) > 10 else np.zeros(n)
    bl = d['lb'] | d['rb'] | (d['lcs'] > 0); cs = np.concatenate([[0], np.cumsum(bl)]); idx = np.arange(n)
    busy = (cs[np.minimum(idx + 300, n)] - cs[np.maximum(idx - 300, 0)]) > 0
    pc = np.concatenate([[0], np.cumsum(d['pressed'])])
    on = np.flatnonzero(d['pressed'][1:] & ~d['pressed'][:-1]) + 1
    km = d['k_model']
    for i, (lo, hi) in enumerate(VB):
      mins[i] += (d['lat'] & (v >= lo) & (v < hi)).sum() / 6000
    for k in on:
      if k < 300 or k + 150 >= n or not d['lat'][k] or v[k] < 30: continue
      if pc[k] - pc[k - 150] > 0: continue                      # released for 1.5 s before
      i = next((q for q, (lo, hi) in enumerate(VB) if lo <= v[k] < hi), None)
      if i is None: continue
      if busy[k]: nlc[i] += 1; continue
      k0 = np.median(km[k - 50:k]); kp = np.median(km[k - 200:k - 150]); kf = np.median(km[k + 50:k + 100]); a0 = abs(k0)
      if max(abs(k0), abs(kp), abs(kf)) < 0.7e-3: ctx = "straight"
      elif abs(k0) > abs(kp) + 0.4e-3: ctx = "entry"
      elif abs(kf) < abs(k0) - 0.4e-3 or abs(k0) < abs(kp) - 0.4e-3: ctx = "exit"
      else: ctx = "curve"
      dw = d['ang'][k + 70] - d['ang'][k]                         # driver moved the wheel
      wheel = d['ang'][k] - off[k]; cmd = d['apply'][k] - off[k]
      if ctx == "straight":
        if pr[k] < 0.5 or abs(lane_off[k]) < 0.05 or abs(dw) < 0.3: dirn = "small/unknown"
        else: dirn = "to centre" if np.sign(dw) == np.sign(lane_off[k]) * (1 if True else -1) else "away"
        # sign check: steeringAngleDeg > 0 = left turn; lane_off < 0 = car left of centre -> to centre means steer right (dw < 0)
        if dirn in ("to centre", "away"):
          dirn = "to centre" if np.sign(dw) == np.sign(lane_off[k]) else "away"
      else:
        s = np.sign(k0) if abs(k0) > 0.3e-3 else np.sign(kf)
        sw = np.sign(cmd) if abs(cmd) > 0.5 else 0.0
        if abs(dw) < 0.3 or sw == 0: dirn = "small/unknown"
        else: dirn = "tighten" if np.sign(dw) == sw else "widen"
      short = (cmd - wheel) * (np.sign(cmd) if abs(cmd) > 0.3 else 0.0)   # > 0: wheel short of op's command (toward centre)
      rows.append((i, ctx, dirn, short, abs(cmd), lane_off[k] if pr[k] >= 0.5 else np.nan, abs(d['wire'][k] - d['ang'][k])))
  print(f"=== {b}: op minutes by band {np.round(mins, 1)}")
  for i, (lo, hi) in enumerate(VB):
    R = [x for x in rows if x[0] == i]; h = mins[i] / 60
    if h <= 0: continue
    print(f"  [{lo}-{hi}] grabs {len(R) + nlc[i]:.0f} = {(len(R) + nlc[i]) / h:5.0f}/h: lane changes {nlc[i] / h:5.0f}/h, other {len(R) / h:5.0f}/h")
    for ctx in ("straight", "entry", "curve", "exit"):
      C = [x for x in R if x[1] == ctx]
      if not C: continue
      line = f"     {ctx:8s} {len(C) / h:5.0f}/h ({len(C):3d}) | "
      for dn in (("tighten", "widen", "small/unknown") if ctx != "straight" else ("to centre", "away", "small/unknown")):
        D = [x for x in C if x[2] == dn]; line += f"{dn} {len(D) / max(len(C), 1) * 100:3.0f}%  "
      sh = np.array([x[3] for x in C]); gp = np.array([x[6] for x in C]); lo_ = np.array([x[5] for x in C])
      line += f"| wheel short of cmd p50 {np.median(sh):5.2f} deg, >= 1 deg in {np.mean(sh >= 1.0) * 100:3.0f}% | |TX-wheel| p50 {np.median(gp):.1f}"
      if ctx == "straight" and np.isfinite(lo_).sum() >= 5: line += f" | lane offset p50 {np.nanmedian(lo_) * 100:5.1f} cm, |off| >= 20 cm in {np.nanmean(np.abs(lo_) >= 0.2) * 100:3.0f}%"
      print(line)
