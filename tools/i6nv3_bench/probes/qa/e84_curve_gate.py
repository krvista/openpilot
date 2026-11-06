"""E84: sustained-curve samples (|K model| >= 0.5e-3 held 1 s, op, hands off, no blinker) by speed band: share of time
the command is below the curve-trim arm gate (CURVE_TRIM_MIN_CMD_DEG 4 deg) and the steady wheel/command and yaw/model
delivery in command-size bins — does the 4 deg gate leave highway curves without the trim?"""
import glob, sys
import numpy as np
S = sys.argv[1]
B = [(30, 55), (55, 70), (70, 90), (90, 200)]
acc = {b: {k: [] for k in ("cmd", "wh", "tx", "K", "ky")} for b in ("98e912d", "bf36aa2")}
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  r = f[-12:-4]
  d = dict(np.load(f, allow_pickle=True)); b = str(d['commit'])[:7]
  if b not in acc:
    continue
  try:
    V = dict(np.load(f"{S}/vm/{r}.npz", allow_pickle=True))
  except Exception:
    continue
  T = d['T']; v = d['v']; vm = v / 3.6
  ky = np.radians(np.interp(T, V['lk'][:, 0], V['lk'][:, 1])) / np.maximum(vm, 1.0)
  K = d['k_model']; n = len(T)
  big = np.abs(K) >= 0.5e-3
  cs = np.concatenate([[0], np.cumsum(big)]); idx = np.arange(n)
  held = (cs[idx + 1] - cs[np.maximum(idx - 99, 0)]) >= np.minimum(idx + 1, 100)
  kc = d['k_ctrl']; mm = d['lat'] & (np.abs(kc) > 0.5e-3)
  sg = np.sign(np.sum(d['apply'][mm] * kc[mm])) if mm.sum() > 100 else -1.0  # angle sign vs curvature sign
  m = held & d['lat'] & ~d['pressed'] & ~(d['lb'] | d['rb']) & (v >= 30)
  for k, x, g in (("cmd", d['apply'], sg), ("wh", d['ang'], sg), ("tx", d['wire'], sg), ("K", K, 1.0), ("ky", ky, 1.0)):
    acc[b][k].append(np.c_[v[m], g * x[m] * np.sign(K[m])])
for b in acc:
  A = {k: np.concatenate(acc[b][k]) for k in acc[b]}
  v = A['cmd'][:, 0]
  print(f"== {b}")
  for lo, hi in B:
    s = (v >= lo) & (v < hi)
    if s.sum() < 300:
      continue
    c = A['cmd'][s, 1]; s2 = c > 0.3; s = s.copy(); s[s] = s2; c = c[s2]; w = A['wh'][s, 1]; t = A['tx'][s, 1]; K = A['K'][s, 1]; ky = A['ky'][s, 1]
    line = f"  {lo}-{hi}: {s.sum()/6000:5.1f} min | cmd<4 deg {np.mean(c < 4):4.0%} | "
    for a, z in ((0, 2), (2, 4), (4, 99)):
      q = (c >= a) & (c < z)
      if q.sum() > 100:
        line += f"cmd {a}-{z}: wheel/cmd {np.median(w[q] / c[q]):.2f} TX/cmd {np.median(t[q] / c[q]):.2f} yaw/K {np.median(ky[q] / K[q]):.2f} | "
    print(line)
