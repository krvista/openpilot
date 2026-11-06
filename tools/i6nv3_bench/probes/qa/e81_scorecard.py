"""E81: steering-quality scorecard by speed band and build (98e912d / 31520b9 / bf36aa2). Op active (lat), no blinker / LC
within 3 s. 'off' = hands off (hold-compensated driver torque < 40 and not pressed).
  time, hand-on share | weave: wheel 0.2-1 Hz RMS (deg, off), lateral-accel 0.2-1 Hz RMS (m/s2, off) | lateral jerk p95
  (m/s3, all) | intent error RMS(yaw curvature(t) - model action(t - 0.375)) (1e-3/m, off) | lag model -> yaw (ms, xcorr
  6 s windows, off) | steady-curve delivery wheel / command (off) | lane offset mean / RMS on straights (cm, off) and its
  0.2-0.6 Hz weave | line approaches /h (edge < 0.15 m, off) and left share | grabs /h (>= 150 for 0.3 s) | snaps /h |
  TX rejects /op-h (driving) | governor saturated share | |TX - command| p90 | 15-30 km/h: wheel 0.5-3 Hz jitter."""
import glob
import sys
from collections import defaultdict
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
BUILDS = ["98e912d", "31520b9", "bf36aa2"]
BANDS = [(15, 30), (30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]


def bp(x, lo, hi, dt=0.01):
  X = np.fft.rfft(x - np.mean(x)); f = np.fft.rfftfreq(len(x), dt)
  X[(f < lo) | (f >= hi)] = 0
  return np.fft.irfft(X, len(x))


def starts(m):
  return np.flatnonzero(np.diff(np.concatenate([[0], m.astype(int)])) == 1)


A = {b: [defaultdict(list) for _ in BANDS] for b in BUILDS}
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  r = f[-12:-4]
  d = dict(np.load(f, allow_pickle=True)); b = str(d['commit'])[:7]
  if b not in BUILDS or len(d['T']) < 6000:
    continue
  try:
    V = dict(np.load(f"{S}/vm/{r}.npz", allow_pickle=True)); P = dict(np.load(f"{S}/la/plan_{r}.npz"))
  except Exception:  # noqa: BLE001
    continue
  lk = V['lk']
  if len(lk) < 100:
    continue
  T = d['T']; n = len(T); v = d['v']; vm = v / 3.6; lat = d['lat']
  ky = np.radians(np.interp(T, lk[:, 0], lk[:, 1])) / np.maximum(vm, 1.0)
  K = d['k_model']; kc = d['k_ctrl']; ang = d['ang']; wire = d['wire']; ap = d['apply']
  la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  hand = (dd >= 40) | d['pressed']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  cs = lambda x: np.concatenate([[0], np.cumsum(x)]); idx = np.arange(n)
  busy = (cs(bl)[np.minimum(idx + 300, n)] - cs(bl)[np.maximum(idx - 300, 0)]) > 0
  ok = lat & ~busy
  off = ok & ~hand
  j = np.clip(np.searchsorted(P['T'], T), 0, len(P['T']) - 1)
  l1 = np.array([np.interp(0.0, x, y) for x, y in zip(P['LX'], P['L1'])])[j]
  l2 = np.array([np.interp(0.0, x, y) for x, y in zip(P['LX'], P['L2'])])[j]
  pr = np.minimum(P['PR'][:, 1], P['PR'][:, 2])[j]
  offset = -(l1 + l2) / 2
  wb = bp(np.nan_to_num(ang), 0.2, 1.0); lab = bp(np.nan_to_num(vm ** 2 * ky), 0.2, 1.0)
  ob = bp(np.nan_to_num(offset), 0.2, 0.6); jit = bp(np.nan_to_num(ang), 0.5, 3.0)
  jerk = np.zeros(n); jerk[10:] = np.abs((vm ** 2 * ky)[10:] - (vm ** 2 * ky)[:-10]) / 0.1
  ie = np.full(n, np.nan); ie[38:] = ky[38:] - K[:-38]
  edge = np.minimum(-l1, l2) - 0.95
  near = off & (pr >= 0.6) & (edge < 0.15)
  nr = np.convolve(near.astype(float), np.ones(30), "same") >= 29
  grab = np.convolve((dd >= 150).astype(float), np.ones(30), "same") >= 29
  gap = wire - ang; rate = np.zeros(n); rate[10:] = (ang[10:] - ang[:-10]) / 0.1
  snap = lat & (np.abs(rate) >= 40) & (np.roll(np.abs(gap), 30) >= 5) & (np.sign(rate) == np.sign(np.roll(gap, 30)))
  sat = ((d['sf_cc'].astype(np.int64) >> 27) & 1) > 0
  apr = np.zeros(n); apr[100:] = np.abs(ap[100:] - ap[:-100])
  steady = off & (np.abs(ap) >= 3) & (apr < 1.0)
  txa = d['tx_active'] == 2
  boot = T < T[0] + 25
  for bi, (lo, hi) in enumerate(BANDS):
    vb = (v >= lo) & (v < hi)
    m_ok = ok & vb; m_off = off & vb
    if m_ok.sum() < 300:
      continue
    R = A[b][bi]
    R['min'].append(m_ok.sum() / 6000); R['hand'].append((m_ok & hand).sum() / 6000)
    R['wb'].append(wb[m_off]); R['lab'].append(lab[m_off]); R['jerk'].append(jerk[m_ok]); R['ie'].append(ie[m_off & ~np.isnan(ie)])
    st = m_off & (pr >= 0.6) & (np.abs(K) < 0.5e-3)
    R['off'].append(offset[st]); R['ob'].append(ob[m_off & (pr >= 0.6)])
    R['near'].append(len(starts(nr & vb))); R['nearL'].append(int(sum(1 for s in starts(nr & vb) if -l1[s] < l2[s])))
    R['offmin'].append(m_off.sum() / 6000)
    R['grab'].append(len(starts(grab & m_ok))); R['snap'].append(len(starts(snap & vb & (v >= 30))))
    R['rej'].append(float(d['tx_reject'][m_ok & ~boot].sum())); R['sat'].append(sat[m_ok & txa]); R['txg'].append(np.abs(wire - ap)[m_ok & txa])
    R['deliv'].append((ang[steady & vb] / ap[steady & vb]))
    if bi == 0:
      R['jit'].append(jit[m_ok])
    W = 600
    for a0 in range(0, n - W - 150, W // 2):
      if not (off[a0:a0 + W].all() and vb[a0:a0 + W].mean() > 0.8) or np.std(K[a0:a0 + W]) < 0.2e-3:
        continue
      xs = K[a0:a0 + W] - K[a0:a0 + W].mean(); best = (-2, 0)
      for Lg in range(0, 101, 5):
        ys = ky[a0 + Lg:a0 + Lg + W] - ky[a0 + Lg:a0 + Lg + W].mean()
        cc = np.dot(xs, ys) / (np.linalg.norm(xs) * np.linalg.norm(ys) + 1e-12)
        if cc > best[0]:
          best = (cc, Lg)
      if best[0] > 0.7:
        R['lag'].append(best[1] * 10)
  # steerOverride events per op-hour (whole route)
  ev = [nm for t, nm in d['events'] if str(nm) == "steerOverride"]
  A[b][0].setdefault('so', []).append((len(ev), lat.sum() / 360000))


def cat(L):
  return np.concatenate(L) if L and any(len(x) for x in L) else np.array([np.nan])


rows = [("op min", lambda R: f"{sum(R['min']):6.1f}"), ("hand-on share", lambda R: f"{sum(R['hand']) / max(sum(R['min']), 1e-9):6.0%}"),
        ("wheel weave 0.2-1Hz deg", lambda R: f"{np.sqrt(np.nanmean(cat(R['wb']) ** 2)):6.2f}"),
        ("lat-acc weave m/s2", lambda R: f"{np.sqrt(np.nanmean(cat(R['lab']) ** 2)):6.3f}"),
        ("lat jerk p95 m/s3", lambda R: f"{np.nanpercentile(cat(R['jerk']), 95):6.2f}"),
        ("intent err 1e-3/m", lambda R: f"{np.sqrt(np.nanmean(cat(R['ie']) ** 2)) * 1e3:6.3f}"),
        ("lag model->yaw ms", lambda R: f"{np.median(R['lag']) if R['lag'] else np.nan:6.0f}"),
        ("curve delivery", lambda R: f"{np.nanmedian(cat(R['deliv'])):6.2f}"),
        ("straight offset cm", lambda R: f"{np.nanmean(cat(R['off'])) * 100:+6.1f}"),
        ("straight off RMS cm", lambda R: f"{np.sqrt(np.nanmean(cat(R['off']) ** 2)) * 100:6.1f}"),
        ("offset weave .2-.6Hz cm", lambda R: f"{np.sqrt(np.nanmean(cat(R['ob']) ** 2)) * 100:6.1f}"),
        ("line approach /h(off)", lambda R: f"{sum(R['near']) / max(sum(R['offmin']), 1e-9) * 60:6.0f}"),
        ("  of which left", lambda R: f"{sum(R['nearL']) / max(sum(R['near']), 1):6.0%}"),
        ("grabs /h", lambda R: f"{sum(R['grab']) / max(sum(R['min']), 1e-9) * 60:6.0f}"),
        ("snaps /h", lambda R: f"{sum(R['snap']) / max(sum(R['min']), 1e-9) * 60:6.0f}"),
        ("TX rejects /op-h", lambda R: f"{sum(R['rej']) / max(sum(R['min']), 1e-9) * 60:6.1f}"),
        ("governor sat share", lambda R: f"{np.nanmean(cat(R['sat'])):6.1%}"),
        ("|TX-cmd| p90 deg", lambda R: f"{np.nanpercentile(cat(R['txg']), 90):6.2f}")]
for bi, (lo, hi) in enumerate(BANDS):
  print(f"\n[{lo}-{hi} km/h]          " + "".join(f"{b:>10}" for b in BUILDS))
  for nm, fn in rows:
    vals = []
    for b in BUILDS:
      R = A[b][bi]
      try:
        vals.append(fn(R) if R['min'] else "     -")
      except Exception:  # noqa: BLE001
        vals.append("     -")
    print(f"  {nm:24}" + "".join(f"{x:>10}" for x in vals))
  if bi == 0:
    print("  wheel jitter .5-3Hz deg " + "".join(f"{np.sqrt(np.nanmean(cat(A[b][0]['jit']) ** 2)) if A[b][0]['jit'] else np.nan:>10.2f}" for b in BUILDS))
print("\nsteerOverride events /op-h: " + "  ".join(f"{b} {sum(x[0] for x in A[b][0].get('so', [])) / max(sum(x[1] for x in A[b][0].get('so', [])), 1e-9):.0f}" for b in BUILDS))
