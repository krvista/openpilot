"""E90: low-speed (15-40 km/h) steering events by build — what the 15-40 km/h scorecard regressions (e81) are made of.
Per build: op minutes 15-40, the share of that time in intersection turns (|K model| >= 8e-3), moderate curves (2-8e-3),
straights; per situation: hand share, lateral jerk p95, wheel 0.5-3 Hz jitter, snap count (wheel >= 40 deg/s closing a
>= 5 deg gap), and steerFlags involvement (passthrough, hand-gap cap, grip anchor, post-grip) around each snap.
Then lists every bf36aa2 snap with its context."""
import glob, sys
from collections import defaultdict
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = ("98e912d", "31520b9", "bf36aa2")
FL = dict(passthru=2, drv_pressed=8, eps_pressed=9, anchor=10, post_grip=21, hand_gap=29, txsat=27, city_rel=18)
def bp(x, lo, hi):
  X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), 0.01); X[(f < lo) | (f >= hi)] = 0; return np.fft.irfft(X, len(x))
def starts(x):
  return np.flatnonzero(np.diff(np.concatenate([[0], x.astype(int)])) == 1)
A = {b: defaultdict(list) for b in BUILDS}; SN = []
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  r = f[-12:-4]
  d = dict(np.load(f, allow_pickle=True)); b = str(d['commit'])[:7]
  if b not in BUILDS:
    continue
  try:
    V = dict(np.load(f"{S}/vm/{r}.npz", allow_pickle=True))
  except Exception:
    continue
  T = d['T']; n = len(T); v = d['v']; vm = v / 3.6
  ky = np.radians(np.interp(T, V['lk'][:, 0], V['lk'][:, 1])) / np.maximum(vm, 1.0)
  la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  hand = (dd >= 40) | d['pressed']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  lat = d['lat'] & ~bl
  K = np.abs(d['k_model'])
  jerk = np.zeros(n); jerk[10:] = np.abs((vm ** 2 * ky)[10:] - (vm ** 2 * ky)[:-10]) / 0.1
  jit = np.abs(bp(np.nan_to_num(d['ang']), 0.5, 3.0))
  ang = d['ang']; wire = d['wire']; gap = wire - ang
  rate = np.zeros(n); rate[10:] = (ang[10:] - ang[:-10]) / 0.1
  snap = lat & (np.abs(rate) >= 40) & (np.roll(np.abs(gap), 30) >= 5) & (np.sign(rate) == np.sign(np.roll(gap, 30)))
  sf = d['sf_cc'].astype(np.int64)
  low = lat & (v >= 15) & (v < 40)
  sit = np.where(K >= 8e-3, 0, np.where(K >= 2e-3, 1, 2))
  for si, nm in enumerate(("turn", "curve", "straight")):
    m = low & (sit == si)
    A[b][nm + "_min"].append(m.sum() / 6000); A[b][nm + "_hand"].append(hand[m]); A[b][nm + "_jerk"].append(jerk[m]); A[b][nm + "_jit"].append(jit[m])
    A[b][nm + "_snap"].append(len(starts(snap & m)))
  for s in starts(snap & low):
    w = slice(max(s - 100, 0), s + 1)
    fl = {k: bool(((sf[w] >> bit) & 1).any()) for k, bit in FL.items()}
    A[b]["snapflags"].append(fl)
    if b == "bf36aa2":
      SN.append((r, T[s] - T[0], v[s], d['k_model'][s] * 1e3, gap[max(s - 30, 0)], rate[s], d['tq_s'][s], dd[max(s - 50, 0):s + 1].max(), fl))
print("situation (|K model|): turn >= 8e-3, curve 2-8e-3, straight < 2e-3; op 15-40 km/h, no blinker")
for nm in ("turn", "curve", "straight"):
  print(f"== {nm}")
  for b in BUILDS:
    a = A[b]; mins = sum(a[nm + "_min"])
    if mins < 0.3:
      continue
    h = np.concatenate(a[nm + "_hand"]); j = np.concatenate(a[nm + "_jerk"]); jt = np.concatenate(a[nm + "_jit"])
    print(f"  {b}: {mins:5.1f} min | hand {h.mean():4.0%} | jerk p95 {np.percentile(j, 95):4.2f} | jitter RMS {np.sqrt(np.mean(jt ** 2)):4.2f} deg | snaps {sum(a[nm + '_snap']):3d} = {sum(a[nm + '_snap']) / mins * 60:5.0f} /h")
print("snap context (flags seen in the 1 s before the snap):")
for b in BUILDS:
  F = A[b]["snapflags"]
  if F:
    print(f"  {b}: n {len(F)} | " + " ".join(f"{k} {np.mean([x[k] for x in F]):.0%}" for k in FL))
print("bf36aa2 snaps: route t v K(e-3) gap-0.3s rate drv_tq max-hand-1s flags")
for x in SN:
  print(f"  {x[0]} {x[1]:7.1f} {x[2]:4.0f} {x[3]:+6.1f} {x[4]:+6.1f} {x[5]:+6.0f} {x[6]:+5.0f} {x[7]:5.0f} {[k for k, vv in x[8].items() if vv]}")
# hands-off snaps only (hold-compensated driver torque < 60 over the 1 s before) and hands-off tracking per situation
print("hands-off only (driver torque < 60 for 1 s before): snaps /h of op 15-40 km/h, and hands-off yaw-vs-model error RMS (e-3, model 0.375 s ahead)")
H = {b: defaultdict(float) for b in BUILDS}; HE = {b: defaultdict(list) for b in BUILDS}
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  r = f[-12:-4]
  d = dict(np.load(f, allow_pickle=True)); b = str(d['commit'])[:7]
  if b not in BUILDS:
    continue
  try:
    V = dict(np.load(f"{S}/vm/{r}.npz", allow_pickle=True))
  except Exception:
    continue
  T = d['T']; n = len(T); v = d['v']; vm = v / 3.6
  ky = np.radians(np.interp(T, V['lk'][:, 0], V['lk'][:, 1])) / np.maximum(vm, 1.0)
  la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  lat = d['lat'] & ~bl
  cs = np.concatenate([[0], np.cumsum(dd >= 60)]); idx = np.arange(n)
  quiet = (cs[idx + 1] - cs[np.maximum(idx - 99, 0)]) == 0
  ang = d['ang']; gap = d['wire'] - ang
  rate = np.zeros(n); rate[10:] = (ang[10:] - ang[:-10]) / 0.1
  snap = lat & quiet & (np.abs(rate) >= 40) & (np.roll(np.abs(gap), 30) >= 5) & (np.sign(rate) == np.sign(np.roll(gap, 30)))
  K = d['k_model']; e = np.full(n, np.nan); e[38:] = ky[38:] - K[:-38]
  for lo, hi in ((15, 40), (40, 200)):
    m = lat & (v >= lo) & (v < hi)
    H[b][f"{lo}min"] += m.sum() / 6000; H[b][f"{lo}snap"] += len(starts(snap & m))
    for nm, sm in (("curve", (np.abs(K) >= 2e-3) & (np.abs(K) < 8e-3)), ("straight", np.abs(K) < 2e-3)):
      q = m & quiet & sm & np.isfinite(e)
      HE[b][f"{lo}{nm}"].append(e[q])
for b in BUILDS:
  s = f"  {b}: "
  for lo in (15, 40):
    s += f"{lo}+ km/h: hands-off snaps {H[b][f'{lo}snap']:.0f} in {H[b][f'{lo}min']:.0f} min = {H[b][f'{lo}snap'] / max(H[b][f'{lo}min'], 1e-9) * 60:4.1f} /h"
    for nm in ("curve", "straight"):
      x = np.concatenate(HE[b][f"{lo}{nm}"]) if HE[b][f"{lo}{nm}"] else np.array([np.nan])
      s += f", {nm} err {np.sqrt(np.nanmean(x ** 2)) * 1e3:.2f}"
    s += " | "
  print(s)
