"""E147 (report §94): every wheel 'snap' at motorway speed, one line each. Snap as in E81: op steering, the wheel turning
>= 40 deg/s toward a wire request that was >= 5 deg away from it 0.3 s earlier. Per snap: speed, command / wire / wheel
0.3 s before (wire - command = stored trim or boost, wire - wheel = stored gap), wheel rate and how far it went in 0.6 s,
hold-compensated driver torque before / at the snap, EPS press, blinker / lane change within 3 s, hand-gap and
boost / exit-lead flags. Tells a released stored request from a driver's own input. argv: S builds_csv [min_kph=70]"""
import glob
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]; BUILDS = sys.argv[2].split(","); VMIN = float(sys.argv[3]) if len(sys.argv) > 3 else 70.0
for b in BUILDS:
  tot = 0; mins = 0.0
  print(f"=== {b}: snaps at >= {VMIN:g} km/h")
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True)); n = len(d['T']); r = f[-12:-4]
    if str(d['commit'])[:7] != b or n < 3000: continue
    v = d['v']; vm = v / 3.6; lat = d['lat'].astype(bool)
    ang = np.nan_to_num(d['ang']); wire = np.nan_to_num(d['wire']); ap = np.nan_to_num(d['apply'])
    la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
    gap = wire - ang; rate = np.zeros(n); rate[10:] = (ang[10:] - ang[:-10]) / 0.1
    snap = lat & (np.abs(rate) >= 40) & (np.roll(np.abs(gap), 30) >= 5) & (np.sign(rate) == np.sign(np.roll(gap, 30))) & (v >= VMIN)
    st = np.flatnonzero(np.diff(np.concatenate([[0], snap.astype(int)])) == 1)
    mins += (lat & (v >= VMIN)).sum() / 6000
    bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
    for k in st:
      tot += 1; k0 = max(k - 30, 0); sc = int(d['sf_cc'][k])
      print(f"   {r} t={d['T'][k]:7.1f} s v={v[k]:5.1f} | 0.3 s before: cmd {ap[k0]:+5.1f} wire {wire[k0]:+5.1f} wheel {ang[k0]:+5.1f} (wire-cmd {wire[k0] - ap[k0]:+.1f}, wire-wheel {gap[k0]:+.1f}) | "
            f"wheel {rate[k]:+.0f} deg/s, {ang[min(k + 30, n - 1)] - ang[k0]:+.1f} deg in 0.6 s | driver torque {dd[k0]:.0f} -> {dd[k]:.0f}, pressed {int(d['pressed'][k0])} -> {int(d['pressed'][k])}, "
            f"blinker/LC {int(bl[max(k - 300, 0):k + 300].any())}, hand-gap {(sc >> 29) & 1}, boost/lead {(sc >> 31) & 1}")
  print(f"   total {tot} in {mins:.1f} op-min = {tot / max(mins, 1e-9) * 60:.1f} /h")
