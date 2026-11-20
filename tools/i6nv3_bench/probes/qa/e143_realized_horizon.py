"""E143 (report §94): which point of the model's plan the steering command really is. The plan gives the curvature the
model intends tau seconds ahead, k_plan(t, tau) = plan yaw rate(tau) / plan speed(tau) (the series controlsd's delay match
uses). For the model's action and for controlsd's command, find the tau at which k_plan(t, tau) is closest to the signal
(least RMS difference over hands-off 6 s windows with some curvature in them): the REALIZED horizon. The design figures:
action = lateralDelay + 0.075 s; command = max(model delay, 0.3 + E(v)) ('total steering timing').
Then the car: lag of the measured curvature behind the command (cross-correlation), and
  late = (car behind command) - (command's realized horizon)      0 = the car is on the plan, + = late
argv: S builds_csv [due_lo_ms due_hi_ms]   (the optional pair keeps only windows whose designed action time is in that range -
e.g. 700 760 = the lagd seed 0.65 s, 780 900 = the learned value)"""
import glob
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = sys.argv[2].split(",")
DUE = (float(sys.argv[3]) / 1e3, float(sys.argv[4]) / 1e3) if len(sys.argv) > 4 else (0.0, 9.0)
BANDS = [(30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]
T_IDXS = np.array([10.0 * (i / 32) ** 2 for i in range(33)])
TAUS = np.arange(0.0, 1.61, 0.05); W = 600


def best_tau(sig, kp):
  """tau (s) minimising RMS(sig - k_plan(tau)); kp: [len(TAUS), W]"""
  e = np.sqrt(np.mean((kp - sig[None, :]) ** 2, axis=1))
  j = int(np.argmin(e))
  if 0 < j < len(TAUS) - 1:   # parabolic refinement
    a, b0, c = e[j - 1], e[j], e[j + 1]; den = a - 2 * b0 + c
    off = 0.5 * (a - c) / den if den > 1e-12 else 0.0
    return TAUS[j] + off * 0.05, e[j] / (np.sqrt(np.mean(sig ** 2)) + 1e-12)
  return TAUS[j], e[j] / (np.sqrt(np.mean(sig ** 2)) + 1e-12)


def xlag(x, y, lo, hi, step=2):
  n = len(x) - hi
  xs = x[:n] - x[:n].mean(); nx = np.linalg.norm(xs) + 1e-12; best = (-2.0, 0)
  for L in range(lo, hi + 1, step):
    ys = y[L:L + n] - y[L:L + n].mean()
    c = float(np.dot(xs, ys) / (nx * (np.linalg.norm(ys) + 1e-12)))
    if c > best[0]: best = (c, L)
  return best[1], best[0]


for b in BUILDS:
  R = [dict(n=0, ta=[], tc=[], ea=[], ec=[], car=[], due=[], des=[]) for _ in BANDS]
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True)); n = len(d['T']); r = f[-12:-4]
    if str(d['commit'])[:7] != b or n < 6000: continue
    try:
      P = np.load(f"{S}/la/plan_{r}.npz"); ld = np.load(f"{S}/vm/{r}.npz", allow_pickle=True)['ld']
    except Exception:  # noqa: BLE001
      continue
    if len(ld) < 10 or len(P['T']) < 100: continue
    T = d['T']; v = d['v']; vm = v / 3.6
    j = np.clip(np.searchsorted(P['T'], T, side="right") - 1, 0, len(P['T']) - 1)     # the plan frame controlsd had at T
    OR = P['OR']; VX = P['VX']
    kp_all = np.empty((len(TAUS), len(P['T'])))
    for q, tau in enumerate(TAUS):
      w = np.searchsorted(T_IDXS, tau, side="right") - 1; w = min(max(w, 0), 31)
      fr = (tau - T_IDXS[w]) / (T_IDXS[w + 1] - T_IDXS[w])
      kp_all[q] = (OR[:, w] * (1 - fr) + OR[:, w + 1] * fr) / np.maximum(VX[:, w] * (1 - fr) + VX[:, w + 1] * fr, 1.0)
    due_all = np.interp(T, ld[:, 0], ld[:, 1]) + 0.075
    la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
    bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
    cs = np.concatenate([[0], np.cumsum(bl)]); idx = np.arange(n)
    busy = (cs[np.minimum(idx + 300, n)] - cs[np.maximum(idx - 300, 0)]) > 0
    off = d['lat'].astype(bool) & ~busy & (dd < 40) & ~d['pressed'].astype(bool)
    K = d['k_model']; kc = d['k_ctrl']; kcar = d['k_meas']
    for a0 in range(200, n - W - 200, W // 2):
      sl = slice(a0, a0 + W)
      if not off[a0:a0 + W + 160].all(): continue
      bi = next((i for i, (lo, hi) in enumerate(BANDS) if ((v[sl] >= lo) & (v[sl] < hi)).mean() > 0.8), None)
      if bi is None or np.std(K[sl]) < 0.2e-3 or not (DUE[0] <= np.median(due_all[sl]) < DUE[1]): continue
      kp = kp_all[:, j[sl]]
      ta, ea = best_tau(K[sl], kp); tc, ec = best_tau(kc[sl], kp)
      Lc, cc = xlag(kc[a0:a0 + W + 160], kcar[a0:a0 + W + 160], 0, 160)
      o = R[bi]; o['n'] += 1; o['ta'].append(ta); o['tc'].append(tc); o['ea'].append(ea); o['ec'].append(ec)
      o['due'].append(float(np.median(due_all[sl])))
      if cc > 0.7: o['car'].append(Lc * 0.01); o['des'].append(Lc * 0.01 - tc)
  print(f"=== {b}")
  for i, (lo, hi) in enumerate(BANDS):
    o = R[i]
    if o['n'] < 3: print(f"  [{lo}-{hi}] windows {o['n']}"); continue
    md = lambda k: np.median(o[k]) * 1e3 if o[k] else np.nan
    q = lambda k, p: np.percentile(o[k], p) * 1e3 if o[k] else np.nan
    print(f"  [{lo}-{hi}] windows {o['n']:3d}: action designed for {md('due'):4.0f} ms, is the plan at {md('ta'):4.0f} ms [{q('ta', 25):.0f}-{q('ta', 75):.0f}] (fit err {np.median(o['ea']):.2f}) | "
          f"command is the plan at {md('tc'):4.0f} ms [{q('tc', 25):.0f}-{q('tc', 75):.0f}] (fit err {np.median(o['ec']):.2f}) | car behind command {md('car'):4.0f} ms -> late {md('des'):+5.0f} ms [{q('des', 25):+.0f} {q('des', 75):+.0f}]")
