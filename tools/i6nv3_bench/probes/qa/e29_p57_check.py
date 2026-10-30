"""E29: Phase 57 verification per build, from the 100 Hz route npz (dir r50): frames with op active, >= 25 km/h, no blinker /
LC +-3 s, driver not steering (|tq| < 150, not pressed, +-1 s).
 1 effective low-confidence cap rate: while steerFlags bit 1 (cap held the command) is set, |d apply / dt| over 0.2 s in
   deg/s, split by |model action| < 1.5e-3 (near-straight: designed 6 deg/s) and >= 3e-3 (curve: Phase 57 30 deg/s);
   plus how long the cap stayed engaged per episode.
 2 command - model gap in low-lane-confidence CURVES (lane_min < 0.5, |action| >= 2e-3): |k_ctrl - K| p50/p90, and the
   longest continuous stretch with |k_ctrl - K| > 2e-3 (the old 'held 1.5-4 s then jump' signature), per hour.
 3 the jump after a hold: |d k_ctrl| over 0.3 s right after the cap/blend releases (bit 0/1/4 falling), p90."""
import glob
import sys
import numpy as np

S = sys.argv[1]
agg = {}
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit']); r = f[-12:-4]
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  sf = d['sf_ctl'].astype(np.int64); K = d['k_model']; kc = d['k_ctrl']; ap = d['apply']; lm = d['lane_min']
  tq = np.abs(d['tq_s']); pr = d['pressed']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)])
  idx = np.arange(n)
  win = lambda m, w: (c(m)[np.minimum(idx + w, n)] - c(m)[np.maximum(idx - w, 0)]) > 0
  hands = win(pr | (tq >= 150), 100); busy = win(bl, 300)
  act = d['lat'] & (v >= 25) & ~busy
  off = act & ~hands
  a = agg.setdefault(b, dict(routes=[], act=0, off=0, rs=[], rc=[], ep=[], gap=[], long=[], jump=[]))
  a['routes'].append(r); a['act'] += act.sum(); a['off'] += off.sum()
  capped = ((sf >> 1) & 1).astype(bool)
  rate = np.abs(ap[20:] - ap[:-20]) / np.maximum(T[20:] - T[:-20], 0.05)
  rate = np.concatenate([rate, np.zeros(20)])
  m = off & capped
  a['rs'].append(rate[m & (np.abs(K) < 1.5e-3)]); a['rc'].append(rate[m & (np.abs(K) >= 3e-3)])
  # cap episode durations
  e = act & capped
  st = np.flatnonzero(e[1:] & ~e[:-1]) + 1; en = np.flatnonzero(~e[1:] & e[:-1]) + 1
  for s0 in st:
    e1 = en[en > s0]
    if len(e1):
      a['ep'].append(T[e1[0]] - T[s0])
  # gap in low-confidence curves
  lc = act & (lm < 0.5) & (np.abs(K) >= 2e-3)
  a['gap'].append(np.abs(kc - K)[lc])
  big = act & (np.abs(kc - K) > 2e-3) & (np.abs(K) >= 2e-3)
  st = np.flatnonzero(big[1:] & ~big[:-1]) + 1; en = np.flatnonzero(~big[1:] & big[:-1]) + 1
  for s0 in st:
    e1 = en[en > s0]
    if len(e1) and T[e1[0]] - T[s0] >= 0.5:
      a['long'].append(T[e1[0]] - T[s0])
  # jump after release
  held = ((sf & 0b10011) > 0)
  rel = np.flatnonzero(~held[1:] & held[:-1]) + 1
  for i in rel:
    if i + 30 < n and act[i]:
      a['jump'].append(abs(kc[i + 30] - kc[i]))
for b, a in agg.items():
  h = a['act'] / 360000
  rs = np.concatenate(a['rs']) if a['rs'] else np.zeros(0); rc = np.concatenate(a['rc']) if a['rc'] else np.zeros(0)
  gap = np.concatenate(a['gap']) if a['gap'] else np.zeros(0)
  pct = lambda x, q: np.percentile(x, q) if len(x) else float('nan')
  print(f"== build {b} ({len(a['routes'])} routes): op active {a['act'] / 6000:.0f} min, hands-off {a['off'] / 6000:.0f} min")
  print(f"   1 cap-held rate deg/s: near-straight n {len(rs)} p50 {pct(rs, 50):.2f} p90 {pct(rs, 90):.2f} | curve n {len(rc)} p50 {pct(rc, 50):.2f} p90 {pct(rc, 90):.2f}"
        f" | cap episodes {len(a['ep'])} ({len(a['ep']) / max(h, 1e-9):.0f}/h) duration p50 {pct(a['ep'], 50):.2f} p90 {pct(a['ep'], 90):.2f} s")
  print(f"   2 low-conf curve |cmd - model|: frames {len(gap)} ({len(gap) / max(a['act'], 1):.2%}) p50 {pct(gap, 50) * 1e3:.2f} p90 {pct(gap, 90) * 1e3:.2f}e-3 | "
        f"stretches > 2e-3 for >= 0.5 s: {len(a['long']) / max(h, 1e-9):.1f}/h, duration p50 {pct(a['long'], 50):.2f} max {max(a['long']) if a['long'] else 0:.2f} s")
  print(f"   3 command change 0.3 s after a cap/blend release: n {len(a['jump'])} p50 {pct(a['jump'], 50) * 1e3:.2f} p90 {pct(a['jump'], 90) * 1e3:.2f}e-3")
