"""E83: straight-road lateral offset per route (lane-centre offset, op engaged, hands off, |K| small, v>=40) and the
steady TX-vs-wheel bias / model curvature bias on those samples, to locate where the left bias originates."""
import glob, sys
import numpy as np
S = sys.argv[1]
print(f"{'route':9s} {'build':8s} {'min':>5s} {'off cm':>7s} {'Kmod 1e-4':>9s} {'Kctl 1e-4':>9s} {'Kmeas 1e-4':>10s} {'TX deg':>7s} {'wheel':>7s} {'TX-wh':>6s} {'roll deg':>8s}")
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  r = f[-12:-4]
  d = dict(np.load(f, allow_pickle=True)); b = str(d['commit'])[:7]
  if b not in ("98e912d", "31520b9", "bf36aa2"):
    continue
  try:
    P = dict(np.load(f"{S}/la/plan_{r}.npz"))
  except Exception:
    continue
  T = d['T']; v = d['v']
  lc = 0.5 * (P['L1'] + P['L2']); off20 = np.array([-np.interp(0.0, x, y) for x, y in zip(P['LX'], lc)])
  off = np.interp(T, P['T'], off20); lok = np.interp(T, P['T'], np.minimum(P['PR'][:, 1], P['PR'][:, 2])) > 0.6
  st = d['lat'] & (v >= 40) & (np.abs(d['k_model']) < 0.3e-3) & ~d['pressed'] & lok & ~(d['lb'] | d['rb'])
  if st.sum() < 3000:
    continue
  roll = np.degrees(np.median(d['roll'][st])) if 'roll' in d else np.nan
  print(f"{r} {b:8s} {st.sum()/6000:5.1f} {np.median(off[st])*100:+7.1f} {np.median(d['k_model'][st])*1e4:+9.2f} {np.median(d['k_ctrl'][st])*1e4:+9.2f} {np.median(d['k_meas'][st])*1e4:+10.2f} "
        f"{np.median(d['wire'][st]):+7.2f} {np.median(d['ang'][st]):+7.2f} {np.median(d['wire'][st]-d['ang'][st]):+6.2f} {roll:+8.2f}")
