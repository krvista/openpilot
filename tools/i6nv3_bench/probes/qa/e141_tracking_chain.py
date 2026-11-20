"""E141 (report §94): how the car follows the model's plan, continuously instead of per corner (E140's strict entry / exit
rule leaves 1-2 events on a motorway drive). Hands-off, no blinker / lane change within 3 s, 6 s windows (3 s hop) with
some curvature in them (std of the action >= 0.2e-3 /m), by build and speed band:
  due    what the model was told: its action is the planned curvature for lateralDelay + 0.075 s later
  car    lag of the car's curvature behind the action (cross-correlation peak, 0 .. 1.6 s)       late = car - due
  cmd    lag of controlsd's command behind the action (negative = controlsd reads the plan further ahead)
  wire   lag of the wire request behind the command angle (negative = CarController leads the command)
  wheel  lag of the wheel behind the wire request
  gain   slope of the car's curvature (at its own best lag) on the action: how much of the planned curvature arrives
  err    RMS(car K(t) - action K(t - due)) / RMS(action) in those windows: the plan error at the moment it was due
argv: S builds_csv"""
import glob
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = sys.argv[2].split(",")
BANDS = [(30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]
EXTRA = 0.075; W = 600


def xlag(x, y, lo, hi, step=2):
  """lag (samples) of y behind x maximising the normalised cross-correlation, searched in [lo, hi]; (lag, corr)"""
  n = len(x) - max(hi, 0) - max(-lo, 0)
  a0 = max(-lo, 0)
  xs = x[a0:a0 + n] - x[a0:a0 + n].mean(); nx = np.linalg.norm(xs) + 1e-12
  best = (-2.0, 0)
  for L in range(lo, hi + 1, step):
    ys = y[a0 + L:a0 + L + n]; ys = ys - ys.mean()
    c = float(np.dot(xs, ys) / (nx * (np.linalg.norm(ys) + 1e-12)))
    if c > best[0]:
      best = (c, L)
  return best[1], best[0]


for b in BUILDS:
  R = [dict(due=[], car=[], cmd=[], wire=[], wheel=[], gain=[], en=[], ed=[], n=0) for _ in BANDS]
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True)); n = len(d['T']); r = f[-12:-4]
    if str(d['commit'])[:7] != b or n < 6000: continue
    try:
      ld = np.load(f"{S}/vm/{r}.npz", allow_pickle=True)['ld']
    except Exception:  # noqa: BLE001
      continue
    if len(ld) < 10: continue
    T = d['T']; due_all = np.interp(T, ld[:, 0], ld[:, 1]) + EXTRA
    v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
    bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
    cs = np.concatenate([[0], np.cumsum(bl)]); idx = np.arange(n)
    busy = (cs[np.minimum(idx + 300, n)] - cs[np.maximum(idx - 300, 0)]) > 0
    off = d['lat'].astype(bool) & ~busy & (dd < 40) & ~d['pressed'].astype(bool)
    K = d['k_model']; kc = d['k_ctrl']; kcar = d['k_meas']; ap = np.nan_to_num(d['apply']); wire = np.nan_to_num(d['wire']); ang = np.nan_to_num(d['ang'])
    for a0 in range(200, n - W - 200, W // 2):
      sl = slice(a0, a0 + W)
      if not off[a0 - 60:a0 + W + 160].all(): continue
      bi = next((i for i, (lo, hi) in enumerate(BANDS) if ((v[sl] >= lo) & (v[sl] < hi)).mean() > 0.8), None)
      if bi is None or np.std(K[sl]) < 0.2e-3: continue
      ext = slice(a0 - 60, a0 + W + 160)
      Lc, cc = xlag(K[ext][60:], kcar[ext][60:], 0, 160)
      if cc < 0.7: continue
      o = R[bi]; o['n'] += 1
      due = float(np.median(due_all[sl])); o['due'].append(due); o['car'].append(Lc * 0.01)
      Lm, cm = xlag(K[ext], kc[ext], -60, 60)
      if cm > 0.7: o['cmd'].append(Lm * 0.01)
      if np.std(ap[sl]) > 0.2:
        Lw, cw = xlag(ap[ext], wire[ext], -60, 60)
        if cw > 0.7: o['wire'].append(Lw * 0.01)
        Lh, ch = xlag(wire[ext][60:], ang[ext][60:], 0, 160)
        if ch > 0.7: o['wheel'].append(Lh * 0.01)
      x = K[sl] - K[sl].mean(); y = kcar[a0 + Lc:a0 + Lc + W]; y = y - y.mean()
      o['gain'].append(float(np.dot(x, y) / (np.dot(x, x) + 1e-18)))
      sh = int(round(due * 100))
      o['en'].append(float(np.sum((kcar[a0 + sh:a0 + sh + W] - K[sl]) ** 2))); o['ed'].append(float(np.sum(K[sl] ** 2)))
  print(f"=== {b}")
  for i, (lo, hi) in enumerate(BANDS):
    o = R[i]
    if o['n'] < 3:
      print(f"  [{lo}-{hi}] windows {o['n']}"); continue
    md = lambda k: np.median(o[k]) * 1e3 if o[k] else np.nan
    late = np.array(o['car']) - np.array(o['due'])
    print(f"  [{lo}-{hi}] windows {o['n']:3d}: due {md('due'):4.0f} ms | car {md('car'):4.0f} -> late {np.median(late) * 1e3:+5.0f} ms [{np.percentile(late, 25) * 1e3:+.0f} {np.percentile(late, 75) * 1e3:+.0f}] | "
          f"cmd {md('cmd'):+5.0f} | wire {md('wire'):+5.0f} | wheel {md('wheel'):4.0f} | gain {np.median(o['gain']):.2f} | err {np.sqrt(sum(o['en']) / max(sum(o['ed']), 1e-18)):.2f}")
