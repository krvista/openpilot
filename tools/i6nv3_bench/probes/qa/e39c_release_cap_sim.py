"""E39: candidate 'release approach cap' — after the driver lets go of a grip, the wheel may approach op's command no faster
than a GAP-DEPENDENT rate. Release edge = |tq| >= 150 for >= 0.3 s, then < 60 within 0.3 s and staying < 150 for 1.5 s,
op active, >= 30 km/h, no blinker / lane change within 3 s. Counterfactual from the log: the wheel follows its logged path
but its rate (relative to the logged wheel at the release instant) is limited by cap(gap) where gap = |apply - wheel| at the
release; the shortfall in wheel angle -> curvature (drive-measured deg per 1e-3/m) -> extra lateral deviation by double
integration of v^2 * dk over 2 s. Reports per variant: peak wheel rate distribution (p50/p90/max), share of releases that
still exceed 20 deg/s, extra lateral deviation p50/p90/max (cm), and the cases with the largest cost (gap, offset to the
lane centre at the release, urgency)."""
import glob
import sys
import numpy as np

S = sys.argv[1]
DEG_PER = ([40, 60, 80, 100], [2.85, 3.12, 3.45, 3.74])     # wheel deg per 1e-3/m
VARIANTS = {  # closing-only caps
  "now (VM limits only)": None,
  "flat 10 deg/s": lambda gap: 10.0,
  "gap 3/6/12 -> 8/12/40": lambda gap: float(np.interp(gap, [3.0, 6.0, 12.0], [8.0, 12.0, 40.0])),
  "gap 3/6/12 -> 10/15/40": lambda gap: float(np.interp(gap, [3.0, 6.0, 12.0], [10.0, 15.0, 40.0])),
  "gap 4/8/14 -> 12/18/45": lambda gap: float(np.interp(gap, [4.0, 8.0, 14.0], [12.0, 18.0, 45.0])),
}
rels = []
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit']); r = f[-12:-4]
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  ang = d['ang']; ap = d['apply']; tq = np.abs(d['tq_s'])
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)]); idx = np.arange(n)
  busy = (c(bl)[np.minimum(idx + 300, n)] - c(bl)[np.maximum(idx - 300, 0)]) > 0
  act = d['lat'] & (v >= 30) & ~busy
  kc = d['k_ctrl']
  hand = (c(tq >= 60)[np.minimum(idx + 100, n)] - c(tq >= 60)[np.maximum(idx - 100, 0)]) > 0
  fitm = act & ~hand & (np.abs(kc) > 1e-3) & (np.abs(ap) > 2)
  if fitm.sum() > 200:
    A_ = np.stack([kc[fitm] * (v[fitm] / 3.6) ** 0 , kc[fitm] * (v[fitm] / 3.6) ** 2, np.ones(fitm.sum())], 1)
    coef = np.linalg.lstsq(A_, ap[fitm], rcond=None)[0]
  else:
    coef = None
  P = None
  try:
    P = np.load(f"{S}/la/plan_{r}.npz")
  except Exception:
    pass
  hi = tq >= 150
  i = 30
  while i < n - 220:
    if hi[i - 1] and not hi[i] and hi[i - 30:i].all():
      j = i + int(np.argmax(tq[i:i + 30] < 60)) if (tq[i:i + 30] < 60).any() else None
      if j is not None and act[j:j + 200].all() and not hi[j:j + 150].any():
        w = slice(j, j + 200)
        if coef is None:
          i += 150; continue
        vv_ = v[j] / 3.6
        desired = coef[0] * kc[j] + coef[1] * kc[j] * vv_ ** 2 + coef[2]
        gap_s = desired - ang[j]
        gap = abs(gap_s)
        off = np.nan
        if P is not None:
          k = int(np.searchsorted(P['T'], T[j]))
          if 0 < k < len(P['T']) and min(P['PR'][k, 1], P['PR'][k, 2]) > 0.5:
            off = -float(np.interp(0.0, P['LX'][k], 0.5 * (P['L1'][k] + P['L2'][k])))
        rels.append(dict(b=b, r=r, t=T[j], v=v[j], gap=gap, gsign=np.sign(gap_s), off=off, ang=ang[w].copy(), T=T[w].copy(), vv=v[w] / 3.6))
      i += 150
    else:
      i += 1
print(f"release edges (desired-angle gap from a per-route apply~k_ctrl fit): {len(rels)} ({sum(1 for x in rels if x['b'] == '98e912d3d')} on 98e912d, rest a0108a4)")
for name, cap in VARIANTS.items():
  peaks, extra, still = [], [], 0
  worst = []
  for x in rels:
    a = x['ang']; T = x['T']; dt = np.maximum(np.diff(T), 0.005)
    if cap is None:
      cf = a.copy()
    else:
      # CLOSING-ONLY: delay only the part of the wheel motion that closes the gap present at the release instant;
      # later plan-driven motion passes through unchanged.
      lim = cap(x['gap']); tt = T - T[0]
      closed = np.clip((a - a[0]) * x['gsign'], 0.0, x['gap'])            # how much of the initial gap the wheel had closed
      allowed = np.minimum(closed, lim * tt)                                 # what the cap lets it close by then
      cf = a - x['gsign'] * (closed - allowed)
    rate = np.abs(np.diff(cf[::10])) / np.maximum(np.diff(T[::10]), 0.05)
    pk = rate.max() if len(rate) else 0.0
    peaks.append(pk); still += pk > 20
    sh = (a - cf)                                                   # wheel-angle shortfall vs what happened
    dk = sh / np.interp(x['v'], *DEG_PER) * 1e-3
    vel = np.concatenate([[0], np.cumsum(x['vv'][1:] ** 2 * dk[1:] * dt)])
    y = np.concatenate([[0], np.cumsum(vel[1:] * dt)])
    extra.append(np.max(np.abs(y)) * 100)
    worst.append((extra[-1], x['r'], x['t'], x['v'], x['gap'], x['off'], pk))
  peaks = np.array(peaks); extra = np.array(extra)
  print(f"== {name:24}: peak wheel rate p50 {np.median(peaks):4.1f} p90 {np.percentile(peaks, 90):4.1f} max {peaks.max():4.0f} deg/s, > 20 deg/s {still / len(rels):5.1%} | "
        f"extra lateral deviation p50 {np.median(extra):4.1f} p90 {np.percentile(extra, 90):4.1f} max {extra.max():4.1f} cm")
  if cap is not None:
    for w_ in sorted(worst, reverse=True)[:3]:
      print(f"      worst: {w_[1]} t {w_[2]:7.1f} v {w_[3]:5.1f} gap {w_[4]:4.1f} deg offset {w_[5]:+.2f} m -> +{w_[0]:.1f} cm (peak {w_[6]:.0f} deg/s)")
gaps = np.array([x['gap'] for x in rels])
print(f"gap at release p50 {np.median(gaps):.1f} p90 {np.percentile(gaps, 90):.1f} max {gaps.max():.1f} deg; gaps >= 12 deg: {np.sum(gaps >= 12)}")
