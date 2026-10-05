"""§33 corner inside-bias report. Inputs: <dir>/<rr>.npy from probes/corner_lanes_extract.py (modelV2 inner lane lines /
plan y at x = 0, 10, 30 m, 20 Hz) and <dir>/000000<rr>.npz from route_extract.py. Usage: corner_inside.py <dir> <rr> [<rr> ...]
Conventions verified on 2e-38: model y positive = RIGHT (left inner line ~ -1.4 m); controlsState.curvature positive = RIGHT
turn (corr with steeringAngleDeg -0.99). Lane-centre offsets are biased by the model's x = 0 outside-line widening in curves
(+18..+50 cm) — judge by the gap to the INSIDE line, not by the centre offset."""
import sys
import numpy as np

d_, routes = sys.argv[1], sys.argv[2:]
cols = []
for r in routes:
  A = np.load(f"{d_}/{r}.npy"); d = np.load(f"{d_}/000000{r}.npz", allow_pickle=True); T = d['T']
  i = np.clip(np.searchsorted(T, A[:, 0]), 0, len(T) - 1)
  busy = np.convolve((d['lb'] | d['rb'] | (d['lcs'] != 0)).astype(float), np.ones(601), 'same') > 0
  cols.append(np.column_stack([A[:, 1], A[:, 4], A[:, 8], A[:, 2], A[:, 5], A[:, 10], A[:, 11], d['v'][i], d['lat'][i],
                               d['tx_active'][i], d['pressed'][i], d['gain'][i], d['k_meas'][i], busy[i]]))
yl0, yr0, py10, yl10, yr10, pl, pr, v, lat, tx, pressed, gain, k, busy = np.concatenate(cols).T
w = yr0 - yl0
base = (pl > 0.7) & (pr > 0.7) & (v > 30) & (busy < 0.5) & (w > 2.7) & (w < 4.3)
a = np.abs(k) * (v / 3.6) ** 2; s = np.sign(k)
cats = {"op-led": (lat > 0.5) & (tx == 2) & (pressed < 0.5) & (gain >= 0.5), "driver pressing": (lat > 0.5) & (tx == 2) & (pressed > 0.5)}
for name, cat in cats.items():
  st = base & cat & (a < 0.3)
  gL, gR, W = np.median(-yl0[st]), np.median(yr0[st]), np.median(w[st])
  print(f"== {name}: straight {st.sum() / 1200:.1f} min, gap left {gL * 100:.0f} / right {gR * 100:.0f} cm, width {W * 100:.0f}")
  for lo, hi in [(0.5, 1), (1, 2), (2, 4)]:
    for nm, sg in (("LEFT", -1), ("RIGHT", 1)):
      m = base & cat & (a >= lo) & (a < hi) & (s == sg)
      if m.sum() < 150:
        continue
      gin = np.median((-yl0 if sg < 0 else yr0)[m]); gout = np.median((yr0 if sg < 0 else -yl0)[m])
      pin = np.median(((py10 - yl10) if sg < 0 else (yr10 - py10))[m])
      print(f"  {lo}-{hi} m/s2 {nm:5s} {m.sum() / 1200:4.1f} min: inside-line gap {(gin - (gL if sg < 0 else gR)) * 100:+4.0f} cm, "
            f"outside {(gout - (gR if sg < 0 else gL)) * 100:+4.0f}, width {(np.median(w[m]) - W) * 100:+3.0f}, plan@10m->inside {pin * 100:4.0f} cm")
