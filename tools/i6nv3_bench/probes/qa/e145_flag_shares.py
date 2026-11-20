"""E145 (report §94): what the steering corrections did on the real drives. Per build and speed band, while op steers with
an active wire frame: share of time each CarController flag (carStateSP.steerFlags) and controlsd flag
(controlsState.steerFlags) is set, the wire request minus controlsd's command measured toward the curve (the curve trim /
entry boost / exit lead show up here), and the stored gap |wire request - wheel|. argv: S builds_csv"""
import glob
import sys
import numpy as np
S = sys.argv[1]; BUILDS = sys.argv[2].split(",")
BANDS = [(30, 55), (55, 70), (70, 90), (90, 200)]
CC = {8: "driver_pressed", 9: "eps_pressed", 10: "grip_anchor", 16: "boost_suppr", 21: "post_grip", 25: "wheel_outrun", 26: "tx_rejected", 27: "tx_saturated", 28: "hod_touch", 29: "hand_gap", 30: "mdps_comp", 31: "entry_boost/exit_lead"}
CT = {0: "lowconf_gate", 1: "lowconf_capped", 4: "conf_blend", 5: "lane_dropout", 6: "departure_hold", 8: "entry_assist", 9: "orient", 10: "orient_budget"}
for b in BUILDS:
  acc = [dict(n=0, cc={k: 0 for k in CC}, ct={k: 0 for k in CT}, d=[], dabs=[], gap=[], cmd=[]) for _ in BANDS]
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True))
    if str(d['commit'])[:7] != b: continue
    v = d['v']; lat = d['lat'].astype(bool); txa = d['tx_active'] == 2
    sc = d['sf_cc'].astype(np.int64); st = d['sf_ctl'].astype(np.int64)
    wire = np.nan_to_num(d['wire']); ap = np.nan_to_num(d['apply']); ang = np.nan_to_num(d['ang'])
    for i, (lo, hi) in enumerate(BANDS):
      m = lat & txa & (v >= lo) & (v < hi)
      o = acc[i]; o['n'] += int(m.sum())
      for k in CC: o['cc'][k] += int((((sc >> k) & 1) > 0)[m].sum())
      for k in CT: o['ct'][k] += int((((st >> k) & 1) > 0)[m].sum())
      o['d'].append((wire - ap)[m] * np.sign(ap[m] + 1e-9)); o['gap'].append(np.abs(wire - ang)[m]); o['cmd'].append(np.abs(ap[m]))
  print(f"=== {b}")
  for i, (lo, hi) in enumerate(BANDS):
    o = acc[i]
    if o['n'] < 1000: print(f"  [{lo}-{hi}] -"); continue
    dd = np.concatenate(o['d']); gap = np.concatenate(o['gap']); cmd = np.concatenate(o['cmd'])
    print(f"  [{lo}-{hi}] {o['n'] / 6000:5.1f} min | CC: " + " ".join(f"{nm} {o['cc'][k] / o['n']:.1%}" for k, nm in CC.items() if o['cc'][k] / o['n'] >= 0.001))
    print("             CTL: " + " ".join(f"{nm} {o['ct'][k] / o['n']:.1%}" for k, nm in CT.items() if o['ct'][k] / o['n'] >= 0.001))
    cv = cmd >= 1.5
    print(f"             TX - command toward the curve (deg): p10 {np.percentile(dd, 10):+.2f} p50 {np.percentile(dd, 50):+.2f} p90 {np.percentile(dd, 90):+.2f} p99 {np.percentile(dd, 99):+.2f}; with |command| >= 1.5 deg ({cv.mean():.0%} of time): p50 {np.percentile(dd[cv], 50) if cv.any() else np.nan:+.2f} p90 {np.percentile(dd[cv], 90) if cv.any() else np.nan:+.2f}"
          f" | |TX - wheel| p50 {np.percentile(gap, 50):.2f} p90 {np.percentile(gap, 90):.2f} p99 {np.percentile(gap, 99):.2f}; >= 5 deg {np.mean(gap >= 5):.1%}, >= 8 deg {np.mean(gap >= 8):.1%}")
