"""E148 (report §94): at motorway speed, is the car later where the curve trim has wound the wire request away from the
command? E143's windows (hands off, 6 s, some curvature) at >= min_kph, split by how far the wire request was from
controlsd's command in the window (p90 of |wire - command|: < 1 / 1-2.5 / >= 2.5 deg). Per group: the car's lag behind the
command, late = that lag - the command's realized plan horizon, the wheel's lag behind the wire request, and the size of
the commands in those windows (small commands are the ones the MDPS follows worst). argv: S build [min_kph=70]"""
import glob
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILD = sys.argv[2]; VMIN = float(sys.argv[3]) if len(sys.argv) > 3 else 70.0
T_IDXS = np.array([10.0 * (i / 32) ** 2 for i in range(33)]); TAUS = np.arange(0.0, 1.61, 0.05); W = 600


def best_tau(sig, kp):
  e = np.sqrt(np.mean((kp - sig[None, :]) ** 2, axis=1)); j = int(np.argmin(e))
  if 0 < j < len(TAUS) - 1:
    a, b0, c = e[j - 1], e[j], e[j + 1]; den = a - 2 * b0 + c
    return TAUS[j] + (0.5 * (a - c) / den if den > 1e-12 else 0.0) * 0.05
  return TAUS[j]


def xlag(x, y, lo, hi, step=2):
  n = len(x) - hi; xs = x[:n] - x[:n].mean(); nx = np.linalg.norm(xs) + 1e-12; best = (-2.0, 0)
  for L in range(lo, hi + 1, step):
    ys = y[L:L + n] - y[L:L + n].mean(); c = float(np.dot(xs, ys) / (nx * (np.linalg.norm(ys) + 1e-12)))
    if c > best[0]: best = (c, L)
  return best[1], best[0]


rows = []
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = dict(np.load(f, allow_pickle=True)); n = len(d['T']); r = f[-12:-4]
  if str(d['commit'])[:7] != BUILD or n < 6000: continue
  P = np.load(f"{S}/la/plan_{r}.npz"); T = d['T']; v = d['v']; vm = v / 3.6
  j = np.clip(np.searchsorted(P['T'], T, side="right") - 1, 0, len(P['T']) - 1)
  kp_all = np.empty((len(TAUS), len(P['T'])))
  for q, tau in enumerate(TAUS):
    w = min(max(np.searchsorted(T_IDXS, tau, side="right") - 1, 0), 31); fr = (tau - T_IDXS[w]) / (T_IDXS[w + 1] - T_IDXS[w])
    kp_all[q] = (P['OR'][:, w] * (1 - fr) + P['OR'][:, w + 1] * fr) / np.maximum(P['VX'][:, w] * (1 - fr) + P['VX'][:, w + 1] * fr, 1.0)
  la = np.abs(vm ** 2 * d['k_meas']); hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold); bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  cs = np.concatenate([[0], np.cumsum(bl)]); idx = np.arange(n); busy = (cs[np.minimum(idx + 300, n)] - cs[np.maximum(idx - 300, 0)]) > 0
  off = d['lat'].astype(bool) & ~busy & (dd < 40) & ~d['pressed'].astype(bool)
  K = d['k_model']; kc = d['k_ctrl']; kcar = d['k_meas']; wire = np.nan_to_num(d['wire']); ap = np.nan_to_num(d['apply']); ang = np.nan_to_num(d['ang'])
  for a0 in range(200, n - W - 200, W // 2):
    sl = slice(a0, a0 + W)
    if not off[a0:a0 + W + 160].all() or (v[sl] >= VMIN).mean() < 0.8 or np.std(K[sl]) < 0.2e-3: continue
    tc = best_tau(kc[sl], kp_all[:, j[sl]]); Lc, cc = xlag(kc[a0:a0 + W + 160], kcar[a0:a0 + W + 160], 0, 160)
    if cc <= 0.7: continue
    Lw, cw = xlag(wire[a0:a0 + W + 160], ang[a0:a0 + W + 160], 0, 160)
    dev = np.abs(wire[sl] - ap[sl])
    rows.append((np.median(dev), np.percentile(dev, 90), Lc * 0.01, Lc * 0.01 - tc, np.median(v[sl]), np.median(np.abs(ap[sl])), Lw * 0.01 if cw > 0.7 else np.nan))
R = np.array(rows)
print(f"=== {BUILD}: {len(R)} windows at >= {VMIN:g} km/h")
for nm, m in (("wire within 1 deg of the command", R[:, 1] < 1.0), ("1-2.5 deg away", (R[:, 1] >= 1.0) & (R[:, 1] < 2.5)), (">= 2.5 deg away", R[:, 1] >= 2.5)):
  if m.sum() < 3:
    print(f"  {nm}: n={m.sum()}"); continue
  x = R[m]
  print(f"  {nm:32s} n={m.sum():2d}: |wire - cmd| p50 {np.median(x[:, 0]):.2f} deg | car behind command {np.median(x[:, 2]) * 1e3:.0f} ms -> late {np.median(x[:, 3]) * 1e3:+.0f} ms "
        f"[{np.percentile(x[:, 3], 25) * 1e3:+.0f} {np.percentile(x[:, 3], 75) * 1e3:+.0f}] | wheel behind wire {np.nanmedian(x[:, 6]) * 1e3:.0f} ms | {np.median(x[:, 4]):.0f} km/h, |command| p50 {np.median(x[:, 5]):.1f} deg")
