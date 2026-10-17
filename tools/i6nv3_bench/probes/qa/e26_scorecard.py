"""E26: general steering-quality scorecard per build (hands-off, op active, >= 30 km/h, no blinker/LC +-3 s) from the
100 Hz route npz only (no plan needed): dirs ropm r3d r43 r46 rx r50.
 A damping/overshoot : after each command step into a curve (|k_ctrl| rising past 1.5e-3), the measured curvature's
                       overshoot beyond the command plateau (%), and the 0.5-2 Hz oscillation energy of (k_meas-k_ctrl)
 B smoothness        : steering-command rate |d apply/dt| p50/p95/p99 deg/s; 100 Hz jerk proxy (2nd diff) RMS
 C latency           : cross-correlation lag command -> wheel (apply -> ang) and model action -> command, all curves
 D actuator          : |ang| / |apply| steady ratio in curves, gain p50; saturation (|apply-ang| > 3 deg) share
 E limits binding    : share of frames where the command moved at the curvature rate limit (clip_curvature) -
                       approximated as |d k_ctrl| within 2 % of the 100 Hz max step seen on that build
 F disturbance/crown : straight-road offset not available here (needs plan) -> steering-angle bias on straights
                       (mean apply where |k_model| < 0.3e-3) as a crown/crosswind proxy, deg
 G low-confidence    : share of active frames with lane_min < 0.5 and what the command did there (|k_ctrl - k_model| p90)
 H speed dependence  : command rate p95 and |apply-ang| p50 for 30-50 / 50-70 / 70+ km/h
 I driver fights     : driver torque >= 300 onsets per hour while op active (hands may be on) and their direction vs command"""
import glob
import sys
import numpy as np

S = sys.argv[1]
builds = {}
for f in sorted(sum((glob.glob(f"{S}/{d}/0*.npz") for d in ("ropm", "r3d", "r43", "r46", "rx", "r50")), [])):
  d = np.load(f, allow_pickle=True); b = str(d['commit']); r = f[-12:-4]
  T = d['T']; v = d['v']; n = len(T)
  if n < 6000:
    continue
  tq = np.abs(d['tq_s']); pr = d['pressed']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)])
  idx = np.arange(n)
  win = lambda m, w: (c(m)[np.minimum(idx + w, n)] - c(m)[np.maximum(idx - w, 0)]) > 0
  hands = win(pr | (tq >= 150), 100); busy = win(bl, 300)
  act = d['lat'] & (v >= 30) & ~busy
  off = act & ~hands
  ap, an, kc, km, kM, g, lm = d['apply'], d['ang'], d['k_ctrl'], d['k_meas'], d['k_model'], d['gain'], d['lane_min']
  st = builds.setdefault(b, dict(routes=[], off=0, act=0, ovs=[], osc=[], rate=[], jerk=[], lag_cw=[], lag_mc=[], ratio=[], gain=[], sat=[],
                                 step=[], crown=[], lowc=0, lowc_dev=[], sp={}, fights=0, fight_dir=[0, 0]))
  st['routes'].append(r); st['off'] += off.sum(); st['act'] += act.sum()
  # A overshoot: command rises past 1.5e-3 (same sign) and holds >= 1.5 s within 20 %
  i = 100
  while i < n - 300:
    if off[i] and abs(kc[i]) >= 1.5e-3 and abs(kc[i - 50]) < 0.75e-3 and off[i:i + 300].all():
      plat = np.median(kc[i + 100:i + 250])
      if abs(plat) >= 1.5e-3 and np.all(np.abs(kc[i + 100:i + 250] - plat) < 0.2 * abs(plat)):
        pk = np.max(km[i:i + 300] * np.sign(plat))
        st['ovs'].append(pk / abs(plat) - 1.0)
        e = (km - kc)[i + 50:i + 300]; e = e - e.mean()
        F = np.abs(np.fft.rfft(e * np.hanning(len(e)))) ** 2; fr = np.fft.rfftfreq(len(e), 0.01)
        st['osc'].append(F[(fr >= 0.5) & (fr <= 2.0)].sum() / max(F[1:].sum(), 1e-18))
      i += 300
    else:
      i += 1
  # B smoothness
  dap = np.abs(np.diff(ap)) / np.maximum(np.diff(T), 0.005)
  st['rate'].append(dap[off[1:]])
  j2 = np.abs(np.diff(ap, 2)) / 0.01 ** 2
  st['jerk'].append(j2[off[2:]] / 1e3)
  # C latency by cross-correlation in curve windows (|k_ctrl| > 1e-3), 10 s windows
  for a in range(0, n - 1000, 1000):
    w = slice(a, a + 1000)
    if off[w].mean() < 0.9 or np.mean(np.abs(kc[w]) > 1e-3) < 0.3:
      continue
    x = ap[w] - ap[w].mean(); y = an[w] - an[w].mean()
    if np.std(x) < 0.5 or np.std(y) < 0.5:
      continue
    lags = np.arange(0, 80)
    cc = [np.dot(x[:len(x) - L], y[L:]) for L in lags]
    st['lag_cw'].append(lags[int(np.argmax(cc))] * 0.01)
    xm = kM[w] - kM[w].mean(); yc = kc[w] - kc[w].mean()
    if np.std(xm) > 2e-4:
      ccm = [np.dot(xm[:len(xm) - L], yc[L:]) for L in lags]
      ccm_neg = [np.dot(yc[:len(yc) - L], xm[L:]) for L in lags]       # command leading the model
      if max(ccm) >= max(ccm_neg):
        st['lag_mc'].append(lags[int(np.argmax(ccm))] * 0.01)
      else:
        st['lag_mc'].append(-lags[int(np.argmax(ccm_neg))] * 0.01)
  # D actuator
  cv = off & (np.abs(ap) > 3) & (np.abs(kc) > 1e-3)
  st['ratio'].append((np.abs(an) / np.maximum(np.abs(ap), 1e-6))[cv]); st['gain'].append(g[cv]); st['sat'].append((np.abs(ap - an) > 3)[cv])
  # E limit binding
  dk = np.abs(np.diff(kc))[off[1:]]
  st['step'].append(dk)
  # F crown proxy
  st['crown'].append(ap[off & (np.abs(kM) < 0.3e-3)])
  # G low confidence
  lc = off & (lm < 0.5)
  st['lowc'] += lc.sum(); st['lowc_dev'].append(np.abs(kc - kM)[lc])
  # H speed
  for lo, hi in ((30, 50), (50, 70), (70, 200)):
    m = off & (v >= lo) & (v < hi)
    s2 = st['sp'].setdefault((lo, hi), [[], []])
    s2[0].append(dap[m[1:]]); s2[1].append(np.abs(ap - an)[m & (np.abs(ap) > 3)])
  # I fights
  big = (tq >= 300) & act
  on = np.flatnonzero(big[1:] & ~big[:-1]) + 1
  st['fights'] += len(on)
  for i in on:
    same = np.sign(d['tq_s'][i]) == -np.sign(ap[i]) if abs(ap[i]) > 1 else None   # tq_s > 0 = left; apply > 0 = left? angle sign: + = left
    if same is not None:
      st['fight_dir'][0 if same else 1] += 1
for b, st in builds.items():
  rate = np.concatenate(st['rate']); jerk = np.concatenate(st['jerk']); ratio = np.concatenate(st['ratio']); step = np.concatenate(st['step'])
  print(f"== build {b} ({', '.join(st['routes'])}): hands-off {st['off'] / 6000:.0f} min of {st['act'] / 6000:.0f} active")
  print(f"   A overshoot: entries {len(st['ovs'])}, measured peak over command plateau p50 {np.median(st['ovs']) * 100 if st['ovs'] else 0:+.0f}% p90 "
        f"{np.percentile(st['ovs'], 90) * 100 if st['ovs'] else 0:+.0f}%; 0.5-2 Hz share of tracking error {np.median(st['osc']) if st['osc'] else 0:.2f}")
  print(f"   B smoothness: cmd rate deg/s p50 {np.median(rate):.2f} p95 {np.percentile(rate, 95):.1f} p99 {np.percentile(rate, 99):.1f}; jerk proxy RMS {np.sqrt(np.mean(jerk ** 2)):.2f}")
  print(f"   C latency: cmd->wheel p50 {np.median(st['lag_cw']) * 1e3 if st['lag_cw'] else 0:.0f} ms (n {len(st['lag_cw'])}); model->cmd p50 "
        f"{np.median(st['lag_mc']) * 1e3 if st['lag_mc'] else 0:+.0f} ms (negative = command leads the model)")
  print(f"   D actuator: wheel/cmd ratio p50 {np.median(ratio):.3f}, gain p50 {np.median(np.concatenate(st['gain'])):.2f}, |cmd-wheel|>3 deg {np.mean(np.concatenate(st['sat'])):.1%}")
  print(f"   E limits: 100 Hz |dk| p99 {np.percentile(step, 99) * 1e3:.4f}e-3, at >= 98 % of max {np.mean(step >= 0.98 * step.max()):.3%}")
  cr = np.concatenate(st['crown'])
  print(f"   F crown proxy: straight apply mean {np.mean(cr):+.2f} deg (std {np.std(cr):.2f})")
  print(f"   G low-conf: lane_min<0.5 {st['lowc'] / max(st['off'], 1):.1%} of hands-off, |cmd-model| p90 there {np.percentile(np.concatenate(st['lowc_dev']), 90) * 1e3 if st['lowc'] else 0:.2f}e-3")
  print("   H speed: " + " | ".join(f"{lo}-{hi}: rate p95 {np.percentile(np.concatenate(a[0]), 95) if sum(len(x) for x in a[0]) else 0:.1f} deg/s, |cmd-wheel| p50 "
                                   f"{np.median(np.concatenate(a[1])) if sum(len(x) for x in a[1]) else 0:.2f}" for (lo, hi), a in st['sp'].items()))
  print(f"   I fights: >= 300 Nm onsets {st['fights'] / max(st['act'] / 360000, 1e-9):.0f}/h, against the command {st['fight_dir'][0]} / with {st['fight_dir'][1]}")
