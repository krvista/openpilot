"""E100: op lane changes by build (r50 npz: lcs = laneChangeState 1 pre / 2 starting / 3 finishing). Per change (state 2
onset, op lateral active, >= 30 km/h): duration of starting + finishing, peak lateral accel and jerk (yaw curvature),
driver help (hold-compensated driver torque >= 150 for >= 0.3 s during the change), abort (back to 0/1 before 3),
wheel overshoot after finishing (sign flip of the wheel against the change direction within 3 s, > 2 deg), and lane
offset 3 s after finishing."""
import glob, sys
from collections import defaultdict
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = ("98e912d", "31520b9", "bf36aa2")
A = defaultdict(list)
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
  la = vm ** 2 * ky
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], np.abs(vm[::10] ** 2 * d['k_meas'][::10]))]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  l = d['lcs'].astype(int)
  on = np.flatnonzero((l[1:] == 2) & (l[:-1] != 2)) + 1
  for s in on:
    if not d['lat'][s] or v[s] < 30 or s + 1200 >= n:
      continue
    e = s
    while e < n - 1 and l[e] in (2, 3):
      e += 1
    reached3 = (l[s:e] == 3).any()
    w = slice(s, e)
    jerk = np.abs(np.diff(la[s:e:10])) / 0.1 if e - s > 20 else np.array([0.0])
    w2 = slice(s + 100, e)   # skip the first 1 s (the nudge that starts the change)
    help_ = np.convolve((dd[w2] >= 150).astype(float), np.ones(30), "same").max() >= 29 if e - s > 130 else False
    nudge = np.convolve((dd[s:s + 100] >= 150).astype(float), np.ones(30), "same").max() >= 29
    sgn = np.sign(np.median(d['k_model'][s:s + 100])) if True else 0
    post = d['ang'][e:e + 300]
    A[b].append(dict(dur=(T[e] - T[s]), done=reached3, la=np.abs(la[w]).max(), jerk=np.percentile(jerk, 95), help=help_,
                     v=np.median(v[w]), nudge=nudge))
for b in BUILDS:
  E = A[b]
  if not E:
    continue
  g = lambda k: np.array([x[k] for x in E], float)
  print(f"{b}: {len(E)} changes (v p50 {np.median(g('v')):.0f} km/h) | duration p50/p90 {np.median(g('dur')):.1f}/{np.percentile(g('dur'), 90):.1f} s | "
        f"peak lat accel p50/max {np.median(g('la')):.2f}/{g('la').max():.2f} m/s2 | jerk p95 p50 {np.median(g('jerk')):.2f} | driver torque in the first 1 s {g('nudge').mean():.0%}, helped after 1 s {g('help').mean():.0%}")
