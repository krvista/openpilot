"""E142 (report §94): corner entries by build with a wider net than E127 (which needs >= 6 deg more command and hands off
for 4 s, and finds 1-3 entries per band on a motorway drive). Entry: op steering, >= 30 km/h, no blinker / lane change
within +-3 s, hands off from -1.5 s to 0, and at t = 0 op's command has moved 2 deg outward from its level 1 s earlier and
grows >= GROW deg more within 2.5 s (argv 3, default 3). Per build and speed band:
  n, size of the entry (command growth p50), share in which the driver pressed the wheel within 2.5 s ('grabbed'),
  and for the entries that stayed hands-off: time until the wheel has moved 1 deg (from its level at -1 s), wire request
  and wheel as a share of the command's growth at +0.5 / +1.0 / +1.5 / +2.0 s, wheel shortfall (deg) at +1.0 / +1.5 s.
argv: S builds_csv [GROW]"""
import glob
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = sys.argv[2].split(","); GROW = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
BANDS = [(30, 55), (55, 70), (70, 200)]
for b in BUILDS:
  R = [dict(n=0, grab=0, size=[], tw1=[], sh10=[], sh15=[], wire={k: [] for k in (50, 100, 150, 200)}, wheel={k: [] for k in (50, 100, 150, 200)}) for _ in BANDS]
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True)); n = len(d['T'])
    if str(d['commit'])[:7] != b or n < 3000: continue
    v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold); pressed = d['pressed'].astype(bool)
    bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
    cs = np.concatenate([[0], np.cumsum(bl)]); idx = np.arange(n)
    busy = (cs[np.minimum(idx + 300, n)] - cs[np.maximum(idx - 300, 0)]) > 0
    lat = d['lat'].astype(bool); ap = np.nan_to_num(d['apply']); wire = np.nan_to_num(d['wire']); ang = np.nan_to_num(d['ang'])
    hand = (dd >= 40) | pressed
    k = 200
    while k < n - 260:
      if lat[k - 150:k + 250].all() and v[k] >= 30 and not busy[k] and not hand[k - 150:k + 1].any():
        s = np.sign(ap[k]) if abs(ap[k]) > 0.5 else np.sign(ap[k] - ap[k - 100])
        mv = (ap[k] - ap[k - 100]) * s
        if s != 0 and mv >= 2.0 and (ap[k - 1] - ap[k - 101]) * s < 2.0 and abs(ap[k]) > abs(ap[k - 100]):
          grow = (ap[k:k + 250] * s).max() - ap[k] * s
          if grow >= GROW:
            bi = next((i for i, (lo, hi) in enumerate(BANDS) if lo <= v[k] < hi), None)
            if bi is not None:
              o = R[bi]; o['n'] += 1; o['size'].append(grow + mv)
              if pressed[k:k + 250].any() or (dd[k:k + 250] >= 40).any():
                o['grab'] += 1
              else:
                c0, w0, a0 = ap[k - 100], wire[k - 100], ang[k - 100]
                t1 = next((t for t in range(k - 100, k + 250) if (ang[t] - a0) * s >= 1.0), None)
                o['tw1'].append((t1 - k) * 0.01 if t1 is not None else np.nan)
                for q in (50, 100, 150, 200):
                  dc = (ap[k + q] - c0) * s
                  if dc > 1.0:
                    o['wire'][q].append((wire[k + q] - w0) * s / dc); o['wheel'][q].append((ang[k + q] - a0) * s / dc)
                o['sh10'].append((ap[k + 100] - c0) * s - (ang[k + 100] - a0) * s); o['sh15'].append((ap[k + 150] - c0) * s - (ang[k + 150] - a0) * s)
            k += 300; continue
      k += 5
  print(f"=== {b} (command grows >= {GROW:g} deg more)")
  for i, (lo, hi) in enumerate(BANDS):
    o = R[i]
    if o['n'] == 0:
      print(f"  [{lo}-{hi}] n=0"); continue
    md = lambda x: np.nanmedian(x) if len(x) else np.nan
    print(f"  [{lo}-{hi}] n={o['n']:3d}, command growth p50 {md(o['size']):4.1f} deg, grabbed within 2.5 s {o['grab'] / o['n']:4.0%} | hands-off {o['n'] - o['grab']:3d}: wheel moves 1 deg after {md(o['tw1']):+.2f} s | "
          "wire/cmd " + " ".join(f"{md(o['wire'][q]):.2f}" for q in (50, 100, 150, 200)) + " | wheel/cmd " + " ".join(f"{md(o['wheel'][q]):.2f}" for q in (50, 100, 150, 200))
          + f" (+0.5/1.0/1.5/2.0 s) | shortfall +1.0 s {md(o['sh10']):.1f}, +1.5 s {md(o['sh15']):.1f} deg")
