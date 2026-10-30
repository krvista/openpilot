"""E35: is the Phase 39 lane-dropout HOLD right in curves? All hold frames (controlsState.steerFlags bit 5), op active,
>= 25 km/h, both builds. Truth = the car's curvature 0.6 s later (k_meas, from the wheel; includes any driver correction).
By |model action| band at the frame: share of frames where the model is closer to the car's later path than the held
command, median errors; per hold: whether the driver applied >= 200 torque within 1.5 s after the hold started and in
which direction relative to the model (angle = -curvature: torque WITH the model has the opposite sign of K).
Also model 'spikes' inside holds: |K| jumping by > 5e-3 within 0.2 s and coming back within 1 s (what the hold exists to
freeze) vs sustained turns (|K| stays beyond the held value for >= 1 s)."""
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
  sf = d['sf_ctl'].astype(np.int64); K = d['k_model']; kc = d['k_ctrl']; km = d['k_meas']; tq = d['tq_s']
  drop = ((sf >> 5) & 1).astype(bool) & d['lat'] & (v >= 25)
  fut = np.concatenate([km[60:], np.full(60, np.nan)])
  a = agg.setdefault(b, dict(frames=[], holds=[]))
  ok = drop & np.isfinite(fut)
  a['frames'].append(np.stack([np.abs(K[ok]), np.abs(K[ok] - fut[ok]), np.abs(kc[ok] - fut[ok])], 1))
  st = np.flatnonzero(drop[1:] & ~drop[:-1]) + 1
  for s0 in st:
    e0 = s0
    while e0 + 1 < n and drop[e0 + 1]:
      e0 += 1
    held = kc[s0]
    w = slice(s0, min(e0 + 100, n))
    kmax = K[w][np.argmax(np.abs(K[w] - held))]
    dev = abs(kmax - held)
    # driver reaction within 1.5 s after the hold started
    g = slice(s0, min(s0 + 150, n))
    big = np.abs(tq[g]) >= 200
    drv = 0 if not big.any() else (1 if np.sign(tq[g][np.argmax(big)]) == -np.sign(kmax - held) else -1)
    # spike vs sustained: does |K - held| stay > 2e-3 for >= 1 s after the hold start?
    far = np.abs(K[s0:min(s0 + 300, n)] - held) > 2e-3
    run = 0; best = 0
    for x in far:
      run = run + 1 if x else 0; best = max(best, run)
    kind = "sustained" if best >= 100 else ("spike" if dev > 5e-3 else "small")
    a['holds'].append((r, T[s0], T[e0] - T[s0], v[s0], held, kmax, dev, drv, kind, abs(fut[s0] - K[s0]) if np.isfinite(fut[s0]) else np.nan,
                       abs(fut[s0] - held) if np.isfinite(fut[s0]) else np.nan))
for b, a in agg.items():
  F = np.concatenate(a['frames']) if a['frames'] else np.zeros((0, 3))
  H = a['holds']
  print(f"== build {b}: hold frames {len(F)} ({len(F) / 100:.0f} s), holds {len(H)}")
  for lo, hi in ((0, 1.5e-3), (1.5e-3, 3e-3), (3e-3, 1)):
    m = (F[:, 0] >= lo) & (F[:, 0] < hi)
    if m.sum() < 20:
      continue
    print(f"   |action| {lo * 1e3:.1f}-{min(hi, 0.1) * 1e3:.1f}e-3: frames {m.sum():5d} | model closer to the car's path 0.6 s later in {np.mean(F[m, 1] < F[m, 2]):.0%} "
          f"| median error model {np.median(F[m, 1]) * 1e3:.2f} vs held {np.median(F[m, 2]) * 1e3:.2f}e-3")
  for kind in ("sustained", "spike", "small"):
    hs = [h for h in H if h[8] == kind]
    if not hs:
      continue
    w = sum(1 for h in hs if h[7] == 1); ag = sum(1 for h in hs if h[7] == -1)
    print(f"   {kind:9}: {len(hs):3d} holds | driver torque >= 200 within 1.5 s: with the model {w}, against {ag}, none {len(hs) - w - ag} | "
          f"start-frame error model/held median {np.nanmedian([h[9] for h in hs]) * 1e3:.2f}/{np.nanmedian([h[10] for h in hs]) * 1e3:.2f}e-3")
  for h in sorted(H, key=lambda h: -h[6])[:10]:
    print(f"     {h[0]} t {h[1]:7.1f} {h[2]:4.1f}s v {h[3]:5.1f} held {h[4] * 1e3:+6.2f} model max-dev {h[5] * 1e3:+6.2f} {h[8]:9} driver {('with' if h[7] == 1 else 'against' if h[7] == -1 else '-'):7}")
