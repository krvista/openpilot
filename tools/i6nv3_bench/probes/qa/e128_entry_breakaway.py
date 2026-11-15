"""E128 (report §91): what makes the wheel start in a corner entry. Hands-off entries as E127 but the command only has to
grow >= 5 deg within 2.5 s (more samples), all builds pooled. Per entry: the time the wheel has moved 1 deg (t_w1, from
the moment the command moved 2 deg = 0), the mean TX rate until then, the TX - wheel gap at that moment, the MDPS output
torque and column torque at -0.5 / 0 / 0.5 / 1.0 s and at t_w1. Then: t_w1 and the gap at breakaway by TX-rate tercile
and by speed band; median torque trajectories. argv: S"""
import glob, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
VB = [(30, 55), (55, 70), (70, 200)]; rows = []
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = dict(np.load(f, allow_pickle=True)); n = len(d['T'])
  if n < 2000: continue
  v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  bad = ~d['lat'] | d['pressed'] | (dd >= 40) | d['lb'] | d['rb'] | (d['tx_active'] != 2)
  bc = np.concatenate([[0], np.cumsum(bad)]); ap, wr, an, mt, ct = d['apply'], d['wire'], d['ang'], d['mdps_tq'], d['tq_s']
  k = 150
  while k < n - 260:
    mv = ap[k] - ap[k - 100]
    if abs(mv) >= 2.0 and abs(ap[k - 1] - ap[k - 101]) < 2.0 and abs(ap[k - 100] - ap[k - 150]) < 1.0:
      s = np.sign(mv)
      if np.max((ap[k:k + 250] - ap[k - 100]) * s) >= 5.0 and bc[k + 251] - bc[k - 150] == 0 and np.sign(ap[k]) == s and abs(an[k - 100] - an[k - 150]) < 0.5:
        i = next((q for q, (lo, hi) in enumerate(VB) if lo <= v[k] < hi), None)
        w = (an[k - 100:k + 250] - an[k - 100]) * s; j = np.flatnonzero(w >= 1.0)
        if i is not None and len(j):
          jw = j[0] - 100                                   # frames relative to k
          t0tx = np.flatnonzero((wr[k - 100:k + 250] - wr[k - 100]) * s >= 0.5); jtx = (t0tx[0] - 100) if len(t0tx) else 0
          rate = (wr[k + jw] - wr[k + jtx]) * s / max((jw - jtx) / 100.0, 0.05)
          rows.append((i, jw / 100.0, (jw - jtx) / 100.0, rate, (wr[k + jw] - an[k + jw]) * s, (wr[k - 100] - an[k - 100]) * s, v[k],
                       *[mt[k + q] * s for q in (-50, 0, 50, 100)], mt[k + jw] * s, *[ct[k + q] * s for q in (-50, 0, 50, 100)], ct[k + jw] * s,
                       (ap[k + 150] - ap[k - 100]) * s, abs(an[k - 100])))
        k += 300; continue
    k += 1
R = np.array(rows); print(f"hands-off entries (command +5 deg within 2.5 s): {len(R)}")
for i, (lo, hi) in enumerate(VB):
  X = R[R[:, 0] == i]
  if len(X) < 6: continue
  p = lambda c, q=50: np.percentile(X[:, c], q)
  print(f"[{lo}-{hi}] n={len(X)} | wheel +1 deg at t = {p(1):.2f} s [{p(1,25):.2f}-{p(1,75):.2f}] after the command moved 2 deg; {p(2):.2f} s after TX started | TX rate until then {p(3):.1f} deg/s | TX-wheel gap then {p(4):.1f} [{p(4,25):.1f}-{p(4,75):.1f}] deg (before the entry {p(5):+.1f})")
  print(f"     MDPS out torque (toward the curve) at -0.5/0/+0.5/+1.0 s: {p(7):.1f} / {p(8):.1f} / {p(9):.1f} / {p(10):.1f}, at breakaway {p(11):.1f} | column torque: {p(12):.0f} / {p(13):.0f} / {p(14):.0f} / {p(15):.0f}, at breakaway {p(16):.0f}")
  o = np.argsort(X[:, 3]); t3 = np.array_split(o, 3)
  print("     by TX-rate tercile: " + " | ".join(f"rate {np.median(X[q, 3]):4.1f} deg/s -> wheel starts {np.median(X[q, 2]):.2f} s after TX, gap {np.median(X[q, 4]):.1f} deg, out tq {np.median(X[q, 11]):.1f}" for q in t3))
  o = np.argsort(X[:, 18]); t3 = np.array_split(o, 2)
  print("     by wheel angle before: " + " | ".join(f"|wheel| {np.median(X[q, 18]):.1f} deg -> starts {np.median(X[q, 2]):.2f} s, gap {np.median(X[q, 4]):.1f}" for q in t3))
print(f"corr(start delay, TX rate) {np.corrcoef(R[:, 2], R[:, 3])[0, 1]:.2f}; corr(gap at breakaway, TX rate) {np.corrcoef(R[:, 4], R[:, 3])[0, 1]:.2f}; corr(start delay, speed) {np.corrcoef(R[:, 2], R[:, 6])[0, 1]:.2f}")
# raw scatter by TX rate (all bands), and the effect of torque already present when TX starts
o = np.argsort(R[:, 3])
print("fastest 14 entries: " + " ".join(f"[{R[q,3]:.0f} deg/s: {R[q,2]:.2f} s, gap {R[q,4]:.0f}, {R[q,6]:.0f} km/h]" for q in o[-14:]))
print("slowest 10 entries: " + " ".join(f"[{R[q,3]:.1f} deg/s: {R[q,2]:.2f} s, gap {R[q,4]:.1f}]" for q in o[:10]))
for nm, c in (("MDPS out torque toward the curve at -0.5 s", 7), ("TX - wheel gap before the entry", 5)):
  o = np.argsort(R[:, c]); t3 = np.array_split(o, 3)
  print(f"by {nm}: " + " | ".join(f"{np.median(R[q, c]):+.1f} -> wheel starts {np.median(R[q, 2]):.2f} s after TX [{np.percentile(R[q, 2], 25):.2f}-{np.percentile(R[q, 2], 75):.2f}]" for q in t3))
