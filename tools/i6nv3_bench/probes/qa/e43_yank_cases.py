"""E43: timelines of the op-led yanks that were followed by a grab (new build): per 0.1 s, model action K, controlsd command
k_ctrl / angle 'apply', TX angle 'wire', measured wheel 'ang', ACI gain, driver torque, lane prob min, and the road from the
plan lanes (centre-line curvature over 10-30 m: 2nd derivative of the lane centre), plus flags."""
import sys
import numpy as np
S = sys.argv[1]
CASES = [("00000065", 212.4), ("00000072", 461.8), ("00000071", 620.5), ("00000065", 1180.8), ("00000064", 642.9), ("00000063", 236.8)]
SER = {0: "41gate", 1: "41cap", 2: "41rel", 4: "6g", 5: "drop", 6: "dep", 7: "bsm", 8: "7c", 9: "lead", 10: "budget"}
CCN = {12: "fast", 15: "accelY", 16: "boostSup", 17: "blindC", 18: "city39a", 19: "anch35b", 20: "blinkC", 21: "post29", 25: "outrun", 27: "txSat", 28: "hod51"}
for r, t0 in CASES:
  d = np.load(f"{S}/r50/{r}.npz", allow_pickle=True); P = np.load(f"{S}/la/plan_{r}.npz")
  T = d['T']; i0 = int(np.searchsorted(T, t0))
  print(f"=== {r} t {t0} v {d['v'][i0]:.0f}")
  print("   t     K*1e3  kc*1e3 apply  wire   ang   gain  tq    lm   | road k(10-30m)*1e3 off   | flags")
  for i in range(i0 - 200, i0 + 200, 10):
    j = int(np.searchsorted(P['T'], T[i])); j = min(max(j, 0), len(P['T']) - 1)
    lx = P['LX'][j]; lc = 0.5 * (P['L1'][j] + P['L2'][j])
    xs = np.array([10.0, 20.0, 30.0]); ys = np.interp(xs, lx, lc)
    kroad = -2 * (ys[2] - 2 * ys[1] + ys[0]) / 100.0     # y left-negative? sign checked vs K below
    off = -float(np.interp(0.0, lx, lc))
    sf = int(d['sf_ctl'][i]); sc = int(d['sf_cc'][i])
    fl = [nm for b, nm in SER.items() if sf >> b & 1] + [nm for b, nm in CCN.items() if sc >> b & 1]
    print(f"  {T[i]:7.1f} {d['k_model'][i] * 1e3:+6.2f} {d['k_ctrl'][i] * 1e3:+6.2f} {d['apply'][i]:+6.1f} {d['wire'][i]:+6.1f} {d['ang'][i]:+6.1f} "
          f"{d['gain'][i]:.2f} {d['tq_s'][i]:+5.0f} {d['lane_min'][i]:.2f} | {kroad * 1e3:+6.2f} {off:+.2f} pr {P['PR'][j, 1]:.2f}/{P['PR'][j, 2]:.2f} | {' '.join(fl)}")
