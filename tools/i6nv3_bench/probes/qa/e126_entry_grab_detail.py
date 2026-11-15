"""E126 (report §91): the corner-entry grabs of E125 in detail (press onset, no blinker / lane change within +-3 s,
>= 30 km/h, model curvature growing). In the 1.5 s before the press: was a hand already resting (hold-compensated
driver-domain torque), what authority (ACI gain) op was sending, how fast op's command and the wire request were rising,
how much the wheel moved, and the size of the corner (|command| at onset and its peak in the next 2 s). Split by
'hand resting before' (driver torque p50 >= 40) vs 'hands off'. argv: S builds_csv"""
import glob, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = sys.argv[2].split(",")
VB = [(30, 55), (55, 70), (70, 200)]
for b in BUILDS:
  rows = []; mins = np.zeros(len(VB))
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True)); n = len(d['T']); r = f[-12:-4]
    if str(d['commit'])[:7] != b or n < 2000: continue
    try: lp = np.load(f"{S}/vm/{r}.npz", allow_pickle=True)['lp']
    except Exception: continue
    T = d['T']; v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
    off = np.interp(T, lp[:, 0], lp[:, 3]) if len(lp) > 10 else np.zeros(n)
    bl = d['lb'] | d['rb'] | (d['lcs'] > 0); cs = np.concatenate([[0], np.cumsum(bl)]); idx = np.arange(n)
    busy = (cs[np.minimum(idx + 300, n)] - cs[np.maximum(idx - 300, 0)]) > 0
    pc = np.concatenate([[0], np.cumsum(d['pressed'])]); km = d['k_model']
    for i, (lo, hi) in enumerate(VB): mins[i] += (d['lat'] & (v >= lo) & (v < hi)).sum() / 6000
    for k in np.flatnonzero(d['pressed'][1:] & ~d['pressed'][:-1]) + 1:
      if k < 300 or k + 200 >= n or not d['lat'][k] or v[k] < 30 or busy[k] or pc[k] - pc[k - 150] > 0: continue
      k0 = np.median(km[k - 50:k]); kp = np.median(km[k - 200:k - 150]); kf = np.median(km[k + 50:k + 100])
      if max(abs(k0), abs(kp), abs(kf)) < 0.7e-3 or not abs(k0) > abs(kp) + 0.4e-3: continue          # entries only
      i = next((q for q, (lo, hi) in enumerate(VB) if lo <= v[k] < hi), None)
      if i is None: continue
      ap = d['apply'] - off; wr = d['wire'] - off; an = d['ang'] - off; s = np.sign(ap[k]) if abs(ap[k]) > 0.3 else np.sign(k0)
      rows.append((i, np.median(dd[k - 150:k - 30]), np.median(d['gain'][k - 60:k - 20]), (ap[k] - ap[k - 100]) * s, (wr[k] - wr[k - 100]) * s,
                   (an[k] - an[k - 100]) * s, abs(ap[k]), np.max(np.abs(ap[k:k + 200])), (ap[k] - an[k]) * s, (wr[k] - an[k]) * s,
                   v[k], np.max(dd[k:k + 100]), (an[k + 70] - an[k]) * s))
  R = np.array(rows); print(f"=== {b}: corner-entry grabs {len(R)}")
  for i, (lo, hi) in enumerate(VB):
    for nm, q in (("hand resting before (>= 40)", (R[:, 0] == i) & (R[:, 1] >= 40)), ("hands off before (< 40)", (R[:, 0] == i) & (R[:, 1] < 40))):
      X = R[q]
      if len(X) < 3: continue
      p = lambda c: np.median(X[:, c])
      print(f"  [{lo}-{hi}] {nm:28s} n={len(X):3d} ({len(X) / (mins[i] / 60):4.0f}/h) | gain before {p(2):.2f} | last 1 s: cmd +{p(3):4.1f}, TX +{p(4):4.1f}, wheel +{p(5):4.1f} deg"
            f" | at press: |cmd| {p(6):4.1f} (peak next 2 s {p(7):4.1f}), cmd-wheel {p(8):4.1f}, TX-wheel {p(9):4.1f} | driver then moved the wheel {p(12):+5.1f} deg in 0.7 s")
