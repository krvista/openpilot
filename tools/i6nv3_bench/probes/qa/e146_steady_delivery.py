"""E146 (report §94): steady-curve delivery by size of the command. Hands off (also in the previous 2 s), no blinker / lane
change within 3 s, controlsd's command changed < 1 deg in the last 1 s. Per build, speed band and command size
(1.5-3 / 3-5 / >= 5 deg): seconds, wheel / command, wire request / command (the curve trim), the car's curvature / the
model action that is due now (action shifted by lateralDelay + 0.075 s), and the stored gap |wire - wheel| p50.
argv: S builds_csv"""
import glob
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = sys.argv[2].split(",")
BANDS = [(40, 55), (55, 70), (70, 90), (90, 200)]; BINS = [(1.5, 3.0), (3.0, 5.0), (5.0, 99.0)]
for b in BUILDS:
  A = {(i, j): dict(n=0, wc=[], tc=[], yk=[], gap=[]) for i in range(len(BANDS)) for j in range(len(BINS))}
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True)); n = len(d['T']); r = f[-12:-4]
    if str(d['commit'])[:7] != b or n < 3000: continue
    ld = np.load(f"{S}/vm/{r}.npz", allow_pickle=True)['ld']
    due = np.round((np.interp(d['T'], ld[:, 0], ld[:, 1]) + 0.075) * 100).astype(int) if len(ld) > 10 else np.full(n, 38)
    v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
    bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
    cs = np.concatenate([[0], np.cumsum(bl)]); idx = np.arange(n)
    busy = (cs[np.minimum(idx + 300, n)] - cs[np.maximum(idx - 300, 0)]) > 0
    hand = (dd >= 40) | d['pressed'].astype(bool)
    ch = np.concatenate([[0], np.cumsum(hand)]); handrec = (ch[np.minimum(idx + 1, n)] - ch[np.maximum(idx - 200, 0)]) > 0   # no hand in the last 2 s
    off = d['lat'].astype(bool) & ~busy & ~handrec
    ap = np.nan_to_num(d['apply']); wire = np.nan_to_num(d['wire']); ang = np.nan_to_num(d['ang'])
    apr = np.zeros(n); apr[100:] = np.abs(ap[100:] - ap[:-100])
    K = d['k_model']; kcar = d['k_meas']
    Kd = np.where(idx - due >= 0, K[np.clip(idx - due, 0, n - 1)], np.nan)     # the action that is due now
    for i, (lo, hi) in enumerate(BANDS):
      for j, (c0, c1) in enumerate(BINS):
        m = off & (v >= lo) & (v < hi) & (np.abs(ap) >= c0) & (np.abs(ap) < c1) & (apr < 1.0)
        if m.sum() == 0: continue
        o = A[(i, j)]; o['n'] += int(m.sum())
        o['wc'].append(ang[m] / ap[m]); o['tc'].append(wire[m] / ap[m]); o['gap'].append(np.abs(wire[m] - ang[m]))
        mk = m & (np.abs(Kd) > 0.2e-3)
        o['yk'].append(kcar[mk] / Kd[mk])
  print(f"=== {b}   (per command-size bin: seconds | wheel/cmd | TX/cmd | car K / action due | |TX-wheel| p50)")
  for i, (lo, hi) in enumerate(BANDS):
    row = []
    for j, (c0, c1) in enumerate(BINS):
      o = A[(i, j)]
      if o['n'] < 300: row.append(f"{c0:g}-{c1:g} deg: -"); continue
      cat = lambda k: np.concatenate(o[k])
      row.append(f"{c0:g}-{c1:g} deg: {o['n'] / 100:5.0f} s | {np.median(cat('wc')):.2f} | {np.median(cat('tc')):.2f} | {np.nanmedian(cat('yk')) if len(cat('yk')) else np.nan:.2f} | {np.median(cat('gap')):.1f}")
    print(f"  [{lo}-{hi}] " + "   ||   ".join(row))
