"""E73: build health scan beyond weave / corner timing. Per build (98e912d vs 31520b9), op-active time:
  grabs/h (driver-domain torque >= 150 for >= 0.3 s, start events) by speed band; share of op time with a hand (>= 40);
  snaps/h (wheel closes a >= 5 deg request gap at >= 40 deg/s, E50 style);
  line approaches/h (hands-off, both lines >= 0.6, car edge within 0.15 m of a line for >= 0.3 s, no blinker / LC);
  lane offset mean / RMS on hands-off straights (|K| < 0.5e-3) >= 40 km/h;
  low speed (15-30 km/h): op active minutes, passthrough / passive share (sf_cc bits 2 / 5), wheel 0.5-3 Hz jitter RMS;
  MDPS fault frames, panda TX rejects per op-hour; lane-dropout holds per hour (sf_ctl bit 5 episodes)."""
import glob
import sys
from collections import defaultdict
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
BANDS = [(15, 30), (30, 55), (55, 200)]


def bp(x, lo, hi, dt=0.01):
  X = np.fft.rfft(x - np.mean(x)); f = np.fft.rfftfreq(len(x), dt)
  X[(f < lo) | (f >= hi)] = 0
  return np.fft.irfft(X, len(x))


def starts(m):
  return np.flatnonzero(np.diff(np.concatenate([[0], m.astype(int)])) == 1)


A = defaultdict(lambda: defaultdict(float)); L = defaultdict(lambda: defaultdict(list))
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  r = f[-12:-4]
  d = dict(np.load(f, allow_pickle=True)); b = str(d['commit'])[:7]
  if b not in ("98e912d", "31520b9"):
    continue
  try:
    P = dict(np.load(f"{S}/la/plan_{r}.npz"))
  except Exception:  # noqa: BLE001
    continue
  T = d['T']; n = len(T); v = d['v']; vm = v / 3.6; lat = d['lat']
  la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  cs = lambda x: np.concatenate([[0], np.cumsum(x)]); idx = np.arange(n)
  busy = (cs(bl)[np.minimum(idx + 300, n)] - cs(bl)[np.maximum(idx - 300, 0)]) > 0
  sc = d['sf_cc'].astype(np.int64); sf = d['sf_ctl'].astype(np.int64)
  hand = (dd >= 40) | d['pressed']
  hi = dd >= 150
  run = np.convolve(hi.astype(float), np.ones(30), "same") >= 29
  for bi, (lo, hh) in enumerate(BANDS):
    m = lat & (v >= lo) & (v < hh)
    A[b][f"min{bi}"] += m.sum() / 6000
    A[b][f"hand{bi}"] += (m & hand).sum() / 6000
    A[b][f"grab{bi}"] += sum(1 for s in starts(run & m) if True)
  # snaps
  gap = d['wire'] - d['ang']; rate = np.zeros(n); rate[10:] = (d['ang'][10:] - d['ang'][:-10]) / 0.1
  snap = lat & (v >= 30) & (np.abs(rate) >= 40) & (np.roll(np.abs(gap), 30) >= 5) & (np.sign(rate) == np.sign(np.roll(gap, 30)))
  A[b]["snap"] += len(starts(snap)); A[b]["op30"] += (lat & (v >= 30)).sum() / 6000
  # lines
  j = np.clip(np.searchsorted(P['T'], T), 0, len(P['T']) - 1)
  l1 = np.array([np.interp(0.0, P['LX'][k], P['L1'][k]) for k in range(len(P['T']))])[j]
  l2 = np.array([np.interp(0.0, P['LX'][k], P['L2'][k]) for k in range(len(P['T']))])[j]
  pr = np.minimum(P['PR'][:, 1], P['PR'][:, 2])[j]
  edge = np.minimum(-l1, l2) - 0.95          # car half-width ~0.95 m (y: left negative)
  hoff = lat & ~busy & ~hand & (v >= 30) & (pr >= 0.6)
  near = hoff & (edge < 0.15)
  nr = np.convolve(near.astype(float), np.ones(30), "same") >= 29
  A[b]["line"] += len(starts(nr)); A[b]["hoff"] += hoff.sum() / 6000
  st = hoff & (np.abs(d['k_model']) < 0.5e-3) & (v >= 40)
  off = -(l1 + l2) / 2
  L[b]["off"].append(off[st])
  # low speed
  m = lat & (v >= 15) & (v < 30)
  A[b]["ls_pass"] += (m & (((sc >> 2) & 1) > 0)).sum() / 6000; A[b]["ls_passive"] += (m & (((sc >> 5) & 1) > 0)).sum() / 6000
  if m.sum() > 500:
    L[b]["ls_jit"].append(bp(np.nan_to_num(d['ang']), 0.5, 3.0)[m])
  A[b]["fault"] += (lat & (d['mdps_fault'] > 0)).sum(); A[b]["rej"] += d['tx_reject'][lat].sum(); A[b]["opall"] += lat.sum() / 6000
  A[b]["drop"] += len(starts(lat & (((sf >> 5) & 1) > 0)))
for b in ("98e912d", "31520b9"):
  a = A[b]
  print(f"== {b}: op active {a['opall']:.0f} min (>= 30 km/h {a['op30']:.0f})")
  for bi, (lo, hh) in enumerate(BANDS):
    print(f"   {lo}-{hh} km/h: {a[f'min{bi}']:5.1f} min, hand on {a[f'hand{bi}'] / max(a[f'min{bi}'], 1e-9):4.0%}, grabs {a[f'grab{bi}'] / max(a[f'min{bi}'], 1e-9) * 60:5.1f}/h")
  o = np.concatenate(L[b]['off']) if L[b]['off'] else np.zeros(1)
  print(f"   snaps {a['snap'] / a['op30'] * 60:.1f}/h | line approaches (edge < 0.15 m, hands off) {a['line'] / max(a['hoff'], 1e-9) * 60:.1f}/h of hands-off ({a['hoff']:.0f} min) "
        f"| straight offset mean {np.mean(o) * 100:+.1f} cm RMS {np.sqrt(np.mean(o ** 2)) * 100:.1f} cm (n {len(o) / 6000:.0f} min)")
  jt = np.concatenate(L[b]['ls_jit']) if L[b]['ls_jit'] else np.zeros(1)
  print(f"   15-30 km/h: passthrough {a['ls_pass']:.1f} min, angle-passive {a['ls_passive']:.1f} min, wheel 0.5-3 Hz jitter {np.sqrt(np.mean(jt ** 2)):.2f} deg "
        f"| MDPS fault frames {a['fault']:.0f}, TX rejects {a['rej'] / a['opall'] * 60:.0f}/op-h | lane-dropout holds {a['drop'] / a['opall'] * 60:.1f}/op-h")
