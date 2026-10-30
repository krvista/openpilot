"""E31: what keeps the command away from the model in curves? Stretches where |k_ctrl - K| > 2e-3 for >= 0.5 s with
|K| >= 2e-3, op active >= 25 km/h, no blinker/LC +-3 s (100 Hz route npz, dir r50). Per stretch: steerFlags OR, whether the
driver was on the wheel, the sign of the gap (command turning LESS than the model = 'under', MORE = 'over'), speed, lane_min,
and the 6h-1 low-pass lag estimate (tau(v) * |dK/dt|). Grouped per build by the first matching cause."""
import glob
import sys
import numpy as np
from collections import Counter, defaultdict

S = sys.argv[1]
NAMES = {0: "41gate", 1: "41cap", 2: "41rel", 4: "6g", 5: "39drop", 6: "dep", 7: "bsm", 8: "7c", 10: "budget"}
res = defaultdict(lambda: dict(n=0, h=0.0, cause=Counter(), under=Counter(), dur=defaultdict(list), ex=[]))
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit']); r = f[-12:-4]
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  sf = d['sf_ctl'].astype(np.int64); K = d['k_model']; kc = d['k_ctrl']; lm = d['lane_min']
  tq = np.abs(d['tq_s']); pr = d['pressed']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)])
  idx = np.arange(n)
  win = lambda m, w: (c(m)[np.minimum(idx + w, n)] - c(m)[np.maximum(idx - w, 0)]) > 0
  busy = win(bl, 300); hands = win(pr | (tq >= 150), 50)
  act = d['lat'] & (v >= 25) & ~busy
  res[b]['h'] += act.sum() / 360000
  big = act & (np.abs(kc - K) > 2e-3) & (np.abs(K) >= 2e-3)
  st = np.flatnonzero(big[1:] & ~big[:-1]) + 1; en = np.flatnonzero(~big[1:] & big[:-1]) + 1
  tau = np.interp(v / 3.6, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08])
  dK = np.abs(np.gradient(K, T))
  for s0 in st:
    e1 = en[en > s0]
    if not len(e1):
      continue
    e0 = e1[0]
    if T[e0] - T[s0] < 0.5:
      continue
    w = slice(max(s0 - 50, 0), e0)
    flags = int(np.bitwise_or.reduce(sf[w]))
    under = np.mean(np.abs(kc[s0:e0]) < np.abs(K[s0:e0])) > 0.5
    lp_lag = np.median(tau[s0:e0] * dK[s0:e0])
    gap = np.median(np.abs(kc[s0:e0] - K[s0:e0]))
    if hands[s0:e0].mean() > 0.5:
      cause = "driver on wheel"
    else:
      hit = [nm for bit, nm in NAMES.items() if flags >> bit & 1]
      if hit:
        cause = "+".join(hit[:2])
      elif lp_lag > 0.5 * gap:
        cause = "LP lag (fast model change)"
      else:
        cause = "none flagged"
    a = res[b]; a['n'] += 1; a['cause'][cause] += 1; a['under'][(cause, under)] += 1; a['dur'][cause].append(T[e0] - T[s0])
    a['ex'].append((T[e0] - T[s0], r, T[s0], v[s0], cause, under, gap, np.min(lm[s0:e0]), K[s0], kc[s0]))
for b, a in res.items():
  print(f"== build {b}: {a['n']} stretches ({a['n'] / max(a['h'], 1e-9):.1f}/h)")
  for cause, k in a['cause'].most_common(12):
    u = a['under'][(cause, True)]
    print(f"   {k:4d}  {cause:28} command under-turning {u}/{k}  duration p50 {np.median(a['dur'][cause]):.2f} max {max(a['dur'][cause]):.2f} s")
  a['ex'].sort(reverse=True)
  for e in a['ex'][:8]:
    print(f"     {e[1]} t {e[2]:7.1f} {e[0]:4.1f}s v {e[3]:5.1f} {e[4]:26} {'under' if e[5] else 'over '} gap {e[6] * 1e3:.2f} lm {e[7]:.2f} K {e[8] * 1e3:+.2f} cmd {e[9] * 1e3:+.2f}")
