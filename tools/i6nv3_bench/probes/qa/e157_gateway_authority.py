"""E157 (report §96): how much of openpilot's request reaches the MDPS in each gateway mode (route npz v4).
  op     openpilot lateral active, factory LFA symbol != 2: the ADAS controller relays the request
  mixed  openpilot lateral active, factory LFA symbol = 2: the controller sends its own request
Per class (v >= 50 km/h, wheel not pressed):
  1. (MDPS request - wheel) binned by (openpilot request - wheel): median / p10 / p90 - does a standing difference get through?
  2. MDPS request rate binned by openpilot request rate: median and the ratio - do changes get through?
  3. request rate the MDPS is given: p50 / p90 / p99 / p99.9 (deg/s)
  4. windows of 4 s with movement: lag and slope request -> wheel, wheel -> yaw curvature, request -> yaw, model action -> yaw
argv: S route [route ...]"""
import sys
import numpy as np
S = sys.argv[1]
EB = [(-30, -8), (-8, -4), (-4, -2), (-2, -1), (-1, -0.5), (-0.5, 0.5), (0.5, 1), (1, 2), (2, 4), (4, 8), (8, 30)]
RB = [(-40, -10), (-10, -5), (-5, -2), (-2, -0.3), (-0.3, 0.3), (0.3, 2), (2, 5), (5, 10), (10, 40)]
acc = {c: dict(e=[], g=[], r=[], rc=[], cbr=[], lag={k: [] for k in ("req->wheel", "wheel->yaw", "req->yaw", "model->yaw")}, slope=[])
       for c in ("op", "mixed")}


def xlag(x, y, maxlag):
  x = x - x.mean(); best = (-2.0, 0); nx = np.linalg.norm(x) + 1e-12
  for L in range(0, maxlag):
    yy = y[L:L + len(x)]; yy = yy - yy.mean()
    c = float(np.dot(x, yy) / (nx * (np.linalg.norm(yy) + 1e-12)))
    if c > best[0]:
      best = (c, L)
  return best


for r in sys.argv[2:]:
  d = dict(np.load(f"{S}/r50/{r}.npz", allow_pickle=True)); T = d['T']; n = len(T)
  if 'cb_req' not in d:
    continue
  v = d['v']; lat = d['lat'].astype(bool); sym = d['lfa_sym'].astype(int)
  cmd = (d['cb_lvl2'].astype(int) == 2) & (d['mdps_active'].astype(int) == 2)
  cb = d['cb_req']; wire = d['wire']; wh = d['mdps_ang']; w3 = np.r_[np.full(3, wire[0]), wire[:-3]]
  dw = np.zeros(n); dw[5:] = (w3[5:] - w3[:-5]) / 0.05; dcb = np.zeros(n); dcb[5:] = (cb[5:] - cb[:-5]) / 0.05
  try:
    lk = dict(np.load(f"{S}/vm/{r}.npz", allow_pickle=True))['lk']
    ky = np.radians(np.interp(T, lk[:, 0], lk[:, 1])) / np.maximum(v / 3.6, 1.0)
  except Exception:  # noqa: BLE001
    ky = None
  for c, mk in (("op", cmd & lat & (sym != 2)), ("mixed", cmd & lat & (sym == 2))):
    m = mk & ~d['pressed'].astype(bool) & (v >= 50); A = acc[c]
    A['e'].append((w3 - wh)[m]); A['g'].append((cb - wh)[m]); A['r'].append(dw[m]); A['rc'].append(dcb[m])
    A['cbr'].append(np.abs(np.diff(cb))[mk[1:] & mk[:-1]] / 0.01)
    if ky is None:
      continue
    W = 400
    for a in range(0, n - W - 150, W // 2):
      sl = slice(a, a + W)
      if not mk[a:a + W + 150].all() or d['pressed'][sl].any() or v[sl].min() < 50 or np.std(wire[sl]) < 0.3:
        continue
      for key, x, y in (("req->wheel", wire, wh), ("wheel->yaw", wh, -ky), ("req->yaw", wire, -ky), ("model->yaw", d['k_model'], ky)):
        cc, L = xlag(x[sl], y[a:a + W + 150], 150)
        if cc > 0.8:
          A['lag'][key].append(L * 10)
          if key == "req->wheel":
            xx = x[sl] - x[sl].mean(); yy = y[a + L:a + L + W]; A['slope'].append(float(np.dot(xx, yy - yy.mean()) / np.dot(xx, xx)))
for c in ("op", "mixed"):
  A = acc[c]
  if not A['e']:
    continue
  e = np.concatenate(A['e']); g = np.concatenate(A['g']); r = np.concatenate(A['r']); rc = np.concatenate(A['rc'])
  print(f"\n=== {c}: {len(e) / 6000:.1f} min (v >= 50, not pressed)")
  print("  (op request - wheel) bin:     n   | MDPS request - wheel: median   p10    p90")
  for lo, hi in EB:
    k = (e >= lo) & (e < hi)
    if k.sum() >= 100:
      print(f"  [{lo:5.1f},{hi:5.1f})  {k.sum():7d}   |                       {np.median(g[k]):+5.2f}  {np.percentile(g[k], 10):+5.2f}  {np.percentile(g[k], 90):+5.2f}")
  print("  op request rate bin (deg/s):  n   | MDPS request rate median | ratio")
  for lo, hi in RB:
    k = (r >= lo) & (r < hi)
    if k.sum() >= 100:
      print(f"  [{lo:5.1f},{hi:5.1f})  {k.sum():7d}   | {np.median(rc[k]):+6.2f}                   | "
            f"{np.median(rc[k]) / np.median(r[k]) if abs(np.median(r[k])) > 0.2 else float('nan'):5.2f}")
  x = np.concatenate(A['cbr'])
  print("  |d(MDPS request)/dt| p50 / p90 / p99 / p99.9 deg/s: " + " / ".join(f"{q:.1f}" for q in np.percentile(x, [50, 90, 99, 99.9])))
  print("  4 s windows: " + "  ".join(f"{k} {np.median(vv):.0f} ms (n {len(vv)})" for k, vv in A['lag'].items() if vv) +
        (f" | wheel change / request change {np.median(A['slope']):.2f}" if A['slope'] else ""))
