"""E144 (report §94): does the car run wide in curves? A measure that needs no timing reference. Hands off (also in the
previous 2 s), no blinker / lane change within 3 s, both lane lines seen (prob >= 0.6): the car's position in the lane
(lane-centre offset from the model's lane lines at the car) in left curves and in right curves, by size of the model's
action curvature. outside drift = (offset in right curves - offset in left curves) / 2 (cm; + = toward the outside of the
curve; a constant camera / lane-centring bias cancels). By build and speed band. argv: S builds_csv"""
import glob
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = sys.argv[2].split(",")
BANDS = [(30, 55), (55, 70), (70, 90), (90, 200)]; KB = [(0.3e-3, 0.7e-3), (0.7e-3, 1.5e-3), (1.5e-3, 1.0)]
for b in BUILDS:
  A = {(i, j, s): [] for i in range(len(BANDS)) for j in range(len(KB)) for s in (-1, 1)}; ST = {i: [] for i in range(len(BANDS))}
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True)); n = len(d['T']); r = f[-12:-4]
    if str(d['commit'])[:7] != b or n < 6000: continue
    try:
      P = dict(np.load(f"{S}/la/plan_{r}.npz"))
    except Exception:  # noqa: BLE001
      continue
    T = d['T']; v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
    bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
    cs = np.concatenate([[0], np.cumsum(bl)]); idx = np.arange(n)
    busy = (cs[np.minimum(idx + 300, n)] - cs[np.maximum(idx - 300, 0)]) > 0
    hand = (dd >= 40) | d['pressed'].astype(bool)
    ch = np.concatenate([[0], np.cumsum(hand)]); handrec = (ch[np.minimum(idx + 1, n)] - ch[np.maximum(idx - 200, 0)]) > 0
    j = np.clip(np.searchsorted(P['T'], T), 0, len(P['T']) - 1)
    l1 = np.array([np.interp(0.0, x, y) for x, y in zip(P['LX'], P['L1'])])[j]
    l2 = np.array([np.interp(0.0, x, y) for x, y in zip(P['LX'], P['L2'])])[j]
    pr = np.minimum(P['PR'][:, 1], P['PR'][:, 2])[j]
    offset = -(l1 + l2) / 2          # + = the car is left of the lane centre
    width = np.abs(l1 - l2)
    ok = d['lat'].astype(bool) & ~busy & ~handrec & (pr >= 0.6) & (width > 2.6) & (width < 4.2)
    K = d['k_model']
    for i, (lo, hi) in enumerate(BANDS):
      vb = ok & (v >= lo) & (v < hi)
      ST[i].append(offset[vb & (np.abs(K) < 0.2e-3)])
      for q, (k0, k1) in enumerate(KB):
        for s in (-1, 1):
          A[(i, q, s)].append(offset[vb & (K * s >= k0) & (K * s < k1)])
  print(f"=== {b}   (per curvature bin: seconds left / right | offset in left curves, in right curves (cm) -> outside drift (cm))")
  for i, (lo, hi) in enumerate(BANDS):
    st = np.concatenate(ST[i]) if ST[i] else np.array([])
    row = [f"straight {np.mean(st) * 100:+5.1f} cm ({len(st) / 100:.0f} s)" if len(st) > 300 else "straight -"]
    for q, (k0, k1) in enumerate(KB):
      L = np.concatenate(A[(i, q, 1)]) if A[(i, q, 1)] else np.array([]); Rr = np.concatenate(A[(i, q, -1)]) if A[(i, q, -1)] else np.array([])
      nm = f"R {1 / k1:.0f}-{1 / k0:.0f} m" if k1 < 0.5 else f"R < {1 / k0:.0f} m"
      if len(L) < 300 or len(Rr) < 300:
        row.append(f"{nm}: {len(L) / 100:.0f}/{len(Rr) / 100:.0f} s -"); continue
      row.append(f"{nm}: {len(L) / 100:.0f}/{len(Rr) / 100:.0f} s | {np.mean(L) * 100:+.1f}, {np.mean(Rr) * 100:+.1f} -> {(np.mean(Rr) - np.mean(L)) * 50:+.1f}")
    print(f"  [{lo}-{hi}] " + "  ||  ".join(row))
