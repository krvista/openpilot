"""E165 (report §101): how does the model itself hold the lane when openpilot steers alone?
From the plan npz (20 Hz: action K, plan position, inner lane lines) on relayed, hands-off, no-blinker frames >= 50 km/h
with both inner-line probabilities >= 0.7:
 1. where the PLAN runs relative to the lane centre 0 / 20 / 40 / 60 / 80 m ahead on straights (|K| < 0.3e-3) - a plan
    that stays parallel to the lane at the car's offset means the model is not asking to re-centre;
 2. the model's steering law, least squares over |K| < 1e-3:  K = a * k_lane + b_psi * psi + b_e * e + c
    (k_lane, psi = curvature / heading of the lane centre line relative to the car from its points at 0 / 20 / 40 m,
    e = lane centre at the car, m, + = to the right) -> heading rate v * b_psi (1/s) and lateral 'spring' v^2 * b_e (1/s^2);
 3. the preview error the Phase 73c term acts on, 2 (centre(L) - plan(L)) / L^2 at L = v * 1.0 / 1.5 / 2.0 s: mean on
    straights and, signed with the curve, in curves (|K| >= 0.8e-3; - = pushes outward).
argv: S route [route ...]"""
import sys
import numpy as np
S = sys.argv[1]
for r in sys.argv[2:]:
  d = dict(np.load(f"{S}/r50/{r}.npz", allow_pickle=True)); T = d['T']
  P = dict(np.load(f"{S}/la/plan_{r}.npz")); n = len(P['T'])
  j = np.clip(np.searchsorted(T, P['T']), 0, len(T) - 1); v = d['v'][j] / 3.6; K = P['K']
  relay = ((d['cb_lvl2'] == 2) & d['lat'].astype(bool) & (d['lfa_sym'] != 2))[j] if 'cb_req' in d else d['lat'].astype(bool)[j]
  ok = relay & (np.minimum(P['PR'][:, 1], P['PR'][:, 2]) >= 0.7) & ~d['pressed'].astype(bool)[j] & ~(d['lb'] | d['rb'])[j] & (v * 3.6 >= 50)
  cen = lambda x: np.array([(np.interp(x, P['LX'][i], P['L1'][i]) + np.interp(x, P['LX'][i], P['L2'][i])) / 2 for i in range(n)])  # noqa: E731
  pln = lambda x: np.array([np.interp(x, P['PX'][i], P['PY'][i]) for i in range(n)])  # noqa: E731
  c = {x: cen(x) for x in (0.0, 20.0, 40.0, 60.0, 80.0)}
  st = ok & (np.abs(K) < 0.3e-3)
  print(f"\n== {r} ({str(d['commit'])[:7]}): {ok.sum() / 1200:.1f} min usable, straights {st.sum() / 1200:.1f} min")
  if st.sum() < 200:
    continue
  print("  1. plan minus lane centre (cm, - = left of centre): " + " | ".join(f"{x:.0f} m: {np.mean((pln(x) - c[x])[st]) * 100:+.1f}" for x in c))
  psi = (-3 * c[0.0] + 4 * c[20.0] - c[40.0]) / 40.0; kl = (c[0.0] - 2 * c[20.0] + c[40.0]) / 400.0
  m = ok & (np.abs(K) < 1.0e-3)
  X = np.stack([kl[m], psi[m], c[0.0][m], np.ones(m.sum())], 1); b, *_ = np.linalg.lstsq(X, K[m], rcond=None); vv = np.median(v[m])
  print(f"  2. K = {b[0]:.2f} k_lane + {b[1]:.3f} psi + {b[2] * 1e3:.3f}e-3 e {b[3] * 1e3:+.3f}e-3 (R2 {1 - (K[m] - X @ b).var() / K[m].var():.2f}); at {vv * 3.6:.0f} km/h: "
        f"heading rate {vv * b[1]:.2f} 1/s, lateral spring {vv ** 2 * b[2]:.3f} 1/s^2")
  cu = ok & (np.abs(K) >= 0.8e-3)
  for tla in (1.0, 1.5, 2.0):
    L = np.clip(v * tla, 8.0, 80.0)
    dk = np.array([2 * ((np.interp(L[i], P['LX'][i], P['L1'][i]) + np.interp(L[i], P['LX'][i], P['L2'][i])) / 2 - np.interp(L[i], P['PX'][i], P['PY'][i])) / L[i] ** 2 for i in range(n)])
    print(f"  3. preview {tla} s: straights {dk[st].mean() * 1e3:+.3f}e-3 (std {dk[st].std() * 1e3:.3f}) | curves, signed {np.mean(dk[cu] * np.sign(K[cu])) * 1e3 if cu.sum() > 50 else float('nan'):+.3f}e-3")
