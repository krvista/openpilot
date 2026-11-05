"""E67: Phase 59 behaviour on the road (31520b9) vs the TX shaping of the previous builds, by speed band, op active with
LKAS_ALT active, no blinker. Per band: |TX - apply| p50 / p90 / p99, share of frames at the GMAX clamp (|TX - apply| >= 5.5
deg), share with the comp feedback flag, 'windup' = |TX - apply| >= 3 deg while the wheel is NOT moving toward TX (|wheel
rate| < 2 deg/s) for >= 1 s; and RELEASES: |TX| changes by >= 3 deg in 0.3 s with |apply| change < 1 deg (TX snap not from
the command), per op-minute; plus the wheel's own swing (0.2-1 Hz band RMS of the wheel angle) and its correlation with
the windup episodes (wheel band RMS within 3 s after a windup end vs elsewhere). Hands: driver-domain torque < 40 = off."""
import glob
import sys
from collections import defaultdict
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
BANDS = [(30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]
SF_MDPS_COMP = 1 << 30


def bp(x, lo, hi, dt=0.01):
  X = np.fft.rfft(x - np.mean(x)); f = np.fft.rfftfreq(len(x), dt)
  X[(f < lo) | (f >= hi)] = 0
  return np.fft.irfft(X, len(x))


A = defaultdict(lambda: defaultdict(list))
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = dict(np.load(f, allow_pickle=True)); b = str(d['commit'])[:7]
  if b not in ("98e912d", "a0108a4", "31520b9"):
    continue
  T = d['T']; n = len(T); v = d['v']; vm = v / 3.6
  la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  hand = (dd >= 40) | d['pressed']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  ok = d['lat'] & (d['tx_active'] == 2) & ~bl & np.isfinite(d['wire'])
  gap = d['wire'] - d['apply']
  rate = np.zeros(n); rate[10:] = (d['ang'][10:] - d['ang'][:-10]) / 0.1
  wb = bp(np.nan_to_num(d['ang']), 0.2, 1.0)
  # windup episodes
  wu = ok & (np.abs(gap) >= 3) & (np.abs(rate) < 2)
  e = np.flatnonzero(np.diff(np.concatenate([[0], wu.astype(int), [0]])))
  wu_ends = [b0 for a0, b0 in zip(e[::2], e[1::2]) if b0 - a0 >= 100]
  # releases: TX snaps not explained by the command
  dtx = np.zeros(n); dtx[30:] = np.abs(d['wire'][30:] - d['wire'][:-30]); dap = np.zeros(n); dap[30:] = np.abs(d['apply'][30:] - d['apply'][:-30])
  rel = ok & (dtx >= 3) & (dap < 1)
  for bi, (lo, hi) in enumerate(BANDS):
    for hs, hm in (("off", ~hand), ("on", hand)):
      m = ok & (v >= lo) & (v < hi) & hm
      if m.sum() < 500:
        continue
      R = A[(b, hs)][bi]
      R.append(dict(n=m.sum(), g=np.abs(gap[m]), clamp=(np.abs(gap[m]) >= 5.5).sum(), comp=((d['sf_cc'][m].astype(np.int64) & SF_MDPS_COMP) > 0).sum(),
                    wu=sum(1 for x in wu_ends if m[x - 1]), rel=int(np.sum(np.diff(np.concatenate([[0], rel[m].astype(int)])) == 1)),
                    wb=np.sqrt(np.mean(wb[m] ** 2)),
                    wb_after=[np.sqrt(np.mean(wb[x:x + 300] ** 2)) for x in wu_ends if m[x - 1] and x + 300 < n]))
for hs in ("off", "on"):
  print(f"===== hands {hs}")
  for bi, (lo, hi) in enumerate(BANDS):
    for b in ("a0108a4", "98e912d", "31520b9"):
      R = A[(b, hs)][bi]
      if not R:
        continue
      N = sum(x['n'] for x in R); g = np.concatenate([x['g'] for x in R]); mins = N / 6000
      wa = [y for x in R for y in x['wb_after']]
      print(f"  [{lo}-{hi}] {b}: {mins:5.1f} min | |TX-apply| p50 {np.median(g):.2f} p90 {np.percentile(g, 90):.2f} p99 {np.percentile(g, 99):.2f} deg, at GMAX {sum(x['clamp'] for x in R) / N:.1%} "
            f"| comp fb {sum(x['comp'] for x in R) / N:.0%} | windup >= 3 deg / 1 s: {sum(x['wu'] for x in R) / mins:4.2f}/min | TX snaps not from cmd {sum(x['rel'] for x in R) / mins:4.2f}/min "
            f"| wheel 0.2-1 Hz RMS {np.sqrt(np.average([x['wb'] ** 2 for x in R], weights=[x['n'] for x in R])):.2f} deg (3 s after windup {np.median(wa) if wa else np.nan:.2f})")
