"""E124 (report §91): is the wheel a fixed fraction of the request? Hands off (hold-compensated driver torque < 40, EPS
press flag off, no blinker, wire active), angles relative to the learned straight-ahead offset (liveParameters
angleOffsetDeg): x = TX - offset, y = wheel - offset (wheel delayed 0.25 s).
 - y / x median by |x| bin and speed band (proportionality),
 - regression slope y on x over all samples, and split by frequency: slow (< 0.1 Hz) and 0.1-1 Hz parts of both signals
   over continuous hands-off stretches >= 20 s.
argv: S builds_csv"""
import glob, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = sys.argv[2].split(",")
VB = [(30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]; XB = [0.3, 0.7, 1.2, 2.0, 3.0, 5.0, 9.0]
def bp(x, lo, hi, dt=0.01):
  X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), dt); X[(f < lo) | (f >= hi)] = 0; return np.fft.irfft(X, len(x))
for b in BUILDS:
  R = {i: [] for i in range(len(VB))}; F = {i: dict(lo=[], hi=[]) for i in range(len(VB))}
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True)); n = len(d['T']); r = f[-12:-4]
    if str(d['commit'])[:7] != b or n < 2000: continue
    try: lp = np.load(f"{S}/vm/{r}.npz", allow_pickle=True)['lp']
    except Exception: continue
    if len(lp) < 10: continue
    off = np.interp(d['T'], lp[:, 0], lp[:, 3])
    v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
    ok = d['lat'] & (dd < 40) & ~d['pressed'] & (d['tx_active'] == 2) & ~d['lb'] & ~d['rb'] & (d['gain'] >= 0.7)
    x = d['wire'] - off; y = np.roll(d['ang'], -25) - off
    e = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(int), [0]])))
    for a0, b0 in zip(e[::2], e[1::2]):
      if b0 - a0 < 300: continue
      sl = slice(a0 + 50, b0 - 50); vv = np.median(v[sl])
      i = next((k for k, (lo, hi) in enumerate(VB) if lo <= vv < hi), None)
      if i is None: continue
      R[i].append(np.c_[x[sl], y[sl]][::10])
      if b0 - a0 >= 2100:
        xs, ys = x[sl], y[sl]
        for key, (f0, f1) in (("lo", (0.0, 0.1)), ("hi", (0.1, 1.0))):
          xb, yb = bp(xs, f0, f1), bp(ys, f0, f1)
          F[i][key].append((float(np.dot(xb, yb)), float(np.dot(xb, xb))))
  print(f"=== {b}")
  for i, (lo, hi) in enumerate(VB):
    if not R[i]: continue
    X = np.concatenate(R[i]); line = f"  [{lo:3d}-{hi:<3d}] {len(X)/600:5.1f} min | slope {np.polyfit(X[:,0], X[:,1], 1)[0]:.2f} | wheel/TX by |TX-offset|: "
    for a, c in zip(XB[:-1], XB[1:]):
      q = (np.abs(X[:, 0]) >= a) & (np.abs(X[:, 0]) < c)
      line += f"{a:g}-{c:g}: {np.median(X[q, 1] / X[q, 0]):4.2f} ({q.sum()/600:4.1f}m) " if q.sum() >= 120 else f"{a:g}-{c:g}: -  "
    for key in ("lo", "hi"):
      if F[i][key]:
        A = np.array(F[i][key]); line += f"| {'<0.1 Hz' if key == 'lo' else '0.1-1 Hz'} gain {A[:,0].sum() / A[:,1].sum():.2f} "
    print(line)
