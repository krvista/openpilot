"""E32: same-road comparison. Reference route/window (old build) and comparison route/window (new build) are aligned by GPS:
each comparison frame is mapped to the nearest reference frame (along-track distance). For 25 m along-track bins print, side
by side: speed, road curvature (lane centre), model action K, command k_ctrl, |k_ctrl - K|, offset to lane centre toward the
curve outside (+ = outside), min lane prob, driver torque max, steerFlags (41 cap / 6g / dropout / 7c / lead budget).
Usage: e32_same_road.py <scratch> <ref route> <t0> <t1> <cmp route> <t0> <t1>"""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import bfit, interp_rows

S = sys.argv[1]


def load(r, t0, t1):
  d = np.load(f"{S}/r50/{r}.npz", allow_pickle=True); P = np.load(f"{S}/la/plan_{r}.npz")
  T = P['T']; m = (T >= t0) & (T <= t1); T = T[m]; n = len(T)
  Tr = d['T']; at = lambda k: np.interp(T, Tr, d[k].astype(float))
  LX = P['LX'][m].astype(float); LC = 0.5 * (P['L1'][m].astype(float) + P['L2'][m].astype(float))
  xs = np.tile(np.linspace(0, 20, 9), (n, 1)); cf = bfit(xs, interp_rows(xs, LX, LC), 2)
  sf = np.interp(T, Tr, d['sf_ctl'].astype(float)).astype(int)
  tqmax = np.array([np.max(np.abs(d['tq_s'][np.searchsorted(Tr, t - 0.25):np.searchsorted(Tr, t + 0.25) + 1])) for t in T])
  return dict(T=T, lat=at('gps_lat'), lon=at('gps_lon'), v=at('v'), kr=2 * cf[:, 0], e=-cf[:, 2], K=P['K'][m].astype(float),
              kc=at('k_ctrl'), lm=np.minimum(P['PR'][m][:, 1], P['PR'][m][:, 2]), tq=tqmax, sf=sf, latA=at('lat') > 0.5)


A = load(sys.argv[2], float(sys.argv[3]), float(sys.argv[4]))
B = load(sys.argv[5], float(sys.argv[6]), float(sys.argv[7]))
# along-track distance of the reference
xy = lambda D: np.stack([(D['lat'] - A['lat'][0]) * 111e3, (D['lon'] - A['lon'][0]) * 88e3], 1)
XA, XB = xy(A), xy(B)
sA = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(XA, axis=0).T))])
nn = np.array([np.argmin(np.hypot(*(XA - p).T)) for p in XB]); sB = sA[nn]
bins = np.arange(0, sA[-1], 25.0)
fl = lambda f: "".join(c for b, c in ((1, "C"), (4, "B"), (5, "D"), (8, "7"), (10, "J")) if f >> b & 1) or "-"
print(f"{'s m':>5} | {'ref ' + sys.argv[2]:^60} | {'cmp ' + sys.argv[5]:^60}")
print(f"{'':5} | {'v':>4} {'k_road':>6} {'K':>6} {'cmd':>6} {'gap':>5} {'out':>5} {'lm':>4} {'tq':>4} fl  | {'v':>4} {'k_road':>6} {'K':>6} {'cmd':>6} {'gap':>5} {'out':>5} {'lm':>4} {'tq':>4} fl")
agg = {"A": [], "B": []}
for s0 in bins:
  row = f"{s0:5.0f} |"
  for nm, D, s in (("A", A, sA), ("B", B, sB)):
    m = (s >= s0) & (s < s0 + 25)
    if not m.any():
      row += " " * 62 + "|"; continue
    sg = np.sign(np.median(D['kr'][m])) or 1.0
    out = np.max(D['e'][m] * -sg)                       # toward the outside of the local curve
    gap = np.max(np.abs(D['kc'][m] - D['K'][m]))
    agg[nm].append((out, gap, np.max(D['tq'][m]), np.median(np.abs(D['kr'][m]))))
    row += (f" {np.mean(D['v'][m]):4.0f} {np.median(D['kr'][m]) * 1e3:+6.1f} {np.median(D['K'][m]) * 1e3:+6.1f} {np.median(D['kc'][m]) * 1e3:+6.1f} "
            f"{gap * 1e3:5.1f} {out:+5.2f} {np.min(D['lm'][m]):4.2f} {np.max(D['tq'][m]):4.0f} {fl(int(np.bitwise_or.reduce(D['sf'][m]))):3} |")
  print(row)
for nm in ("A", "B"):
  x = np.array(agg[nm]); cv = x[:, 3] >= 3e-3
  print(f"{nm}: bins {len(x)}, curve bins {cv.sum()} | outside max p50/p90/max {np.median(x[cv, 0]):+.2f}/{np.percentile(x[cv, 0], 90):+.2f}/{x[cv, 0].max():+.2f} m | "
        f"cmd-model gap max p50/p90 {np.median(x[cv, 1]) * 1e3:.2f}/{np.percentile(x[cv, 1], 90) * 1e3:.2f}e-3 | driver tq>=250 in {np.mean(x[:, 2] >= 250):.0%} of bins")
