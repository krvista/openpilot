"""E156 (report §96): who steers the car, and how well - stock LFA alone, openpilot alone, or both at once.
The MDPS is commanded by ADAS_CMD_35_10ms (0xCB) from the vehicle's ADAS controller, not by the LKAS_ALT frame on the
camera bus (route npz v4: cb_req / cb_gain / cb_lvl2, lfa_sym = HDA_LFA_SymSta, hda_icon). Each 10 ms sample is one of
  stock   the MDPS is in angle mode, openpilot lateral is not active, factory LFA symbol = 2 (capture drives, or an
          openpilot drive with lateral switched off and the factory LFA on)
  op      openpilot lateral active, factory LFA symbol != 2 -> the controller passes openpilot's request through
  mixed   openpilot lateral active AND factory LFA symbol = 2 -> the controller sends its own request
  manual  the MDPS is not in angle mode
Part 1, per route: minutes and share per class, and inside the two openpilot classes how far the request the MDPS gets
is from the one openpilot sent 30 ms earlier (pass-through check) and from the wheel.
Part 2, per class and speed band (no blinker / lane change within 3 s; 'off' = hands off): time, hands-on share, wheel
weave 0.2-1 Hz, lateral jerk p95, lag model action -> yaw (xcorr over 6 s hands-off windows), lane offset on straights
(mean, RMS), offset weave 0.2-0.6 Hz, lane offset in left and in right curves (|model K| >= 1e-3) and the drift toward
the OUTSIDE = (left-curve offset - right-curve offset) / 2 [offset + = right of the lane centre], line approaches /h, grabs /h, |MDPS request - wheel| p50 / p90 / p99, |openpilot request - wheel| p50 / p90 / p99.
argv: S route [route ...]"""
import sys
from collections import defaultdict
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
BANDS = [(15, 30), (30, 55), (55, 70), (70, 90), (90, 200)]; CL = ["stock", "op", "mixed"]


def bp(x, lo, hi, dt=0.01):
  X = np.fft.rfft(x - np.mean(x)); f = np.fft.rfftfreq(len(x), dt)
  X[(f < lo) | (f >= hi)] = 0
  return np.fft.irfft(X, len(x))


def starts(m):
  return np.flatnonzero(np.diff(np.concatenate([[0], m.astype(int)])) == 1)


def pct(x, q):
  return np.nanpercentile(x, q) if len(x) else np.nan


A = {c: [defaultdict(list) for _ in BANDS] for c in CL}
print("route     build    min | stock    op  mixed manual other | op: |MDPS req - op req| p50 / p90, |op req - wheel| p90 | "
      "mixed: the same | stock: |MDPS req - wheel| p90")
for r in sys.argv[2:]:
  d = dict(np.load(f"{S}/r50/{r}.npz", allow_pickle=True)); T = d['T']; n = len(T)
  if 'cb_req' not in d:
    print(f"{r}: route npz older than v4, skipped"); continue
  v = d['v']; vm = v / 3.6; lat = d['lat'].astype(bool); sym = d['lfa_sym'].astype(int)
  cmd = (d['cb_lvl2'].astype(int) == 2) & (d['mdps_active'].astype(int) == 2)
  cls = {"stock": cmd & ~lat & (sym == 2), "op": cmd & lat & (sym != 2), "mixed": cmd & lat & (sym == 2)}
  manual = ~cmd; other = cmd & ~lat & (sym != 2)
  cb = d['cb_req']; wire = d['wire']; wheel = d['mdps_ang']; w3 = np.r_[np.full(3, wire[0]), wire[:-3]]
  mv = v >= 15

  def gp(m):
    m = m & mv
    return (f"{pct(np.abs(cb - w3)[m], 50):5.2f} / {pct(np.abs(cb - w3)[m], 90):5.2f}, {pct(np.abs(wire - wheel)[m], 90):5.2f}"
            if m.sum() > 200 else "    - /     -,     -")
  print(f"{r} {str(d['commit'])[:7]} {n / 6000:5.1f} | " + " ".join(f"{cls[c].mean():5.0%}" for c in CL) +
        f" {manual.mean():5.0%} {other.mean():5.0%} | {gp(cls['op'])} | {gp(cls['mixed'])} | "
        f"{pct(np.abs(cb - wheel)[cls['stock'] & mv], 90):5.2f}")
  try:
    V = dict(np.load(f"{S}/vm/{r}.npz", allow_pickle=True)); P = dict(np.load(f"{S}/la/plan_{r}.npz"))
  except Exception:  # noqa: BLE001
    print("   (no plan / vehicle-model extract: part 2 skipped)"); continue
  lk = V['lk']
  if len(lk) < 100:
    continue
  ky = np.radians(np.interp(T, lk[:, 0], lk[:, 1])) / np.maximum(vm, 1.0)
  K = d['k_model']; la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold); hand = (dd >= 40) | d['pressed'].astype(bool)
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  cs = lambda x: np.concatenate([[0], np.cumsum(x)]); idx = np.arange(n)  # noqa: E731
  busy = (cs(bl)[np.minimum(idx + 300, n)] - cs(bl)[np.maximum(idx - 300, 0)]) > 0
  j = np.clip(np.searchsorted(P['T'], T), 0, len(P['T']) - 1)
  l1 = np.array([np.interp(0.0, x, y) for x, y in zip(P['LX'], P['L1'])])[j]
  l2 = np.array([np.interp(0.0, x, y) for x, y in zip(P['LX'], P['L2'])])[j]
  pr = np.minimum(P['PR'][:, 1], P['PR'][:, 2])[j]
  offset = -(l1 + l2) / 2                                     # + = the car sits right of the lane centre (model frame: y to the right)
  wb = bp(np.nan_to_num(d['ang']), 0.2, 1.0); ob = bp(np.nan_to_num(offset), 0.2, 0.6)
  lacc = vm ** 2 * ky; jerk = np.zeros(n); jerk[10:] = np.abs(lacc[10:] - lacc[:-10]) / 0.1
  edge = np.minimum(-l1, l2) - 0.95
  grab = np.convolve((dd >= 150).astype(float), np.ones(30), "same") >= 29
  for c in CL:
    ok = cls[c] & ~busy; off = ok & ~hand
    nr = np.convolve((off & (pr >= 0.6) & (edge < 0.15)).astype(float), np.ones(30), "same") >= 29
    for bi, (lo, hi) in enumerate(BANDS):
      vb = (v >= lo) & (v < hi); m_ok = ok & vb; m_off = off & vb
      if m_ok.sum() < 300:
        continue
      R = A[c][bi]
      R['min'].append(m_ok.sum() / 6000); R['hand'].append((m_ok & hand).sum() / 6000); R['offmin'].append(m_off.sum() / 6000)
      R['wb'].append(wb[m_off]); R['jerk'].append(jerk[m_ok])
      R['off'].append(offset[m_off & (pr >= 0.6) & (np.abs(K) < 0.5e-3)]); R['ob'].append(ob[m_off & (pr >= 0.6)])
      R['cl'].append(offset[m_off & (pr >= 0.6) & (K <= -1.0e-3)]); R['cr'].append(offset[m_off & (pr >= 0.6) & (K >= 1.0e-3)])  # K < 0 = left curve
      R['near'].append(len(starts(nr & vb))); R['grab'].append(len(starts(grab & m_ok)))
      R['cbg'].append(np.abs(cb - wheel)[m_off]); R['wg'].append(np.abs(wire - wheel)[m_off])
      W = 600
      for a0 in range(0, n - W - 150, W // 2):
        if not (off[a0:a0 + W].all() and vb[a0:a0 + W].mean() > 0.8) or np.std(K[a0:a0 + W]) < 0.2e-3:
          continue
        xs = K[a0:a0 + W] - K[a0:a0 + W].mean(); best = (-2, 0)
        for Lg in range(0, 121, 5):
          ys = ky[a0 + Lg:a0 + Lg + W] - ky[a0 + Lg:a0 + Lg + W].mean()
          cc = np.dot(xs, ys) / (np.linalg.norm(xs) * np.linalg.norm(ys) + 1e-12)
          if cc > best[0]:
            best = (cc, Lg)
        if best[0] > 0.7:
          R['lag'].append(best[1] * 10)


def cat(L):
  return np.concatenate(L) if L and any(len(x) for x in L) else np.array([np.nan])


rows = [("min", lambda R: f"{sum(R['min']):7.1f}"), ("hand-on share", lambda R: f"{sum(R['hand']) / max(sum(R['min']), 1e-9):7.0%}"),
        ("wheel weave 0.2-1Hz deg", lambda R: f"{np.sqrt(np.nanmean(cat(R['wb']) ** 2)):7.2f}"),
        ("lat jerk p95 m/s3", lambda R: f"{np.nanpercentile(cat(R['jerk']), 95):7.2f}"),
        ("lag model->yaw ms (n)", lambda R: f"{np.median(R['lag']) if R['lag'] else np.nan:4.0f}({len(R['lag'])})"),
        ("straight offset cm", lambda R: f"{np.nanmean(cat(R['off'])) * 100:+7.1f}"),
        ("straight off RMS cm", lambda R: f"{np.sqrt(np.nanmean(cat(R['off']) ** 2)) * 100:7.1f}"),
        ("offset weave .2-.6Hz cm", lambda R: f"{np.sqrt(np.nanmean(cat(R['ob']) ** 2)) * 100:7.1f}"),
        ("left curves: offset cm", lambda R: f"{np.nanmean(cat(R['cl'])) * 100:+7.1f}"),
        ("right curves: offset cm", lambda R: f"{np.nanmean(cat(R['cr'])) * 100:+7.1f}"),
        ("outside drift cm", lambda R: f"{(np.nanmean(cat(R['cl'])) - np.nanmean(cat(R['cr']))) * 50:+7.1f}"),
        ("curve min L / R (off)", lambda R: f"{len(cat(R['cl'])) / 6000:3.1f}/{len(cat(R['cr'])) / 6000:3.1f}"),
        ("line approach /h(off)", lambda R: f"{sum(R['near']) / max(sum(R['offmin']), 1e-9) * 60:7.0f}"),
        ("grabs /h", lambda R: f"{sum(R['grab']) / max(sum(R['min']), 1e-9) * 60:7.0f}"),
        ("|MDPS req-wheel| p50", lambda R: f"{np.nanpercentile(cat(R['cbg']), 50):7.2f}"),
        ("|MDPS req-wheel| p90", lambda R: f"{np.nanpercentile(cat(R['cbg']), 90):7.2f}"),
        ("|MDPS req-wheel| p99", lambda R: f"{np.nanpercentile(cat(R['cbg']), 99):7.2f}"),
        ("|op req-wheel| p50", lambda R: f"{np.nanpercentile(cat(R['wg']), 50):7.2f}"),
        ("|op req-wheel| p90", lambda R: f"{np.nanpercentile(cat(R['wg']), 90):7.2f}"),
        ("|op req-wheel| p99", lambda R: f"{np.nanpercentile(cat(R['wg']), 99):7.2f}")]
for bi, (lo, hi) in enumerate(BANDS):
  print(f"\n[{lo}-{hi} km/h]            " + "".join(f"{c:>10}" for c in CL))
  for nm, fn in rows:
    vals = []
    for c in CL:
      R = A[c][bi]
      try:
        vals.append(fn(R) if R['min'] and not (c == "stock" and nm.startswith("|op req")) else "      -")
      except Exception:  # noqa: BLE001
        vals.append("      -")
    print(f"  {nm:24s}" + "".join(f"{x:>10}" for x in vals))
