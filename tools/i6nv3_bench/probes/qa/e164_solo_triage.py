"""E164 (report §100): first look at a drive in the solo profile (Phase 72) - where did nobody steer, and what happened
at the switch-overs? Route npz v5 (works on v4 too: the car controller's steerFlags and the raw gateway fields).
1. NOBODY STEERS: openpilot lateral is ON (carControl.latActive) but the MDPS is not in angle mode, or it is and the
   factory LFA is not active while openpilot holds its request passive. Share of lateral-on time by speed, split by
   reason (steerFlags: low-speed latch / scenario block, parking mode, reverse latch, angle-passive = the driver turning,
   wheel-outrun, VM reject, camera fault; 'gateway' = openpilot was requesting but the MDPS was not commanded), and the
   episodes >= 1 s above 10 km/h without a driver press: time, length, speed, reason, lane position at the end.
2. SWITCH-OVERS: every change of who commands the MDPS (relay <-> factory LFA <-> none) and every lateral on / off edge:
   the largest wheel movement within 1 s (deg in 0.2 s) and whether a button pulse / the follow band was involved.
3. HOLD-OFF: button pulses, time the factory LFA was on / active while lateral was on, relay share of the commanded time.
argv: S route [route ...]"""
import re
import sys
from collections import Counter
import numpy as np
S = sys.argv[1]
src = open("opendbc_repo/opendbc/car/hyundai/carcontroller.py").read()
SF = {m.group(1): int(m.group(2)) for m in re.finditer(r"^(SF_[A-Z_]+)\s*=\s*1 << (\d+)", src, re.M)}
REASONS = [("low-speed", ("SF_LOWSPEED_LATCH", "SF_SCENARIO_BLOCK")), ("parking", ("SF_PARKING",)), ("reverse", ("SF_REVERSE",)),
           ("driver turning", ("SF_ANGLE_PASSIVE",)), ("wheel-outrun", ("SF_WHEEL_OUTRUN",)), ("vm-reject", ("SF_VM_REJECT",)),
           ("camera", ("SF_CAM_FAULT",))]
BINS = [(5, 15), (15, 25), (25, 35), (35, 45), (45, 60), (60, 90), (90, 200)]
for r in sys.argv[2:]:
  d = dict(np.load(f"{S}/r50/{r}.npz", allow_pickle=True)); T = d['T']; n = len(T); v = d['v']
  sf = d['sf_cc'].astype(np.int64); lat = d['lat'].astype(bool); sym = d['lfa_sym'].astype(int)
  gw = d['gw_flags'].astype(np.int64) if 'gw_flags' in d else np.zeros(n, np.int64)
  cmd = (d['cb_lvl2'].astype(int) == 2) & (d['mdps_active'].astype(int) == 2)
  eff = ((sf >> SF['SF_EFF_ACTIVE']) & 1) > 0; pressed = d['pressed'].astype(bool)
  why = np.full(n, "", dtype=object)
  for nm, keys in REASONS[::-1]:
    f = np.zeros(n, bool)
    for k in keys:
      f |= ((sf >> SF[k]) & 1) > 0
    why[f & ~eff] = nm
  why[(why == "") & ~eff] = "other passive"
  why[eff & ~cmd] = "gateway"
  nobody = lat & ~(cmd & (eff | (sym == 2)))
  print(f"\n===== {r} ({str(d['commit'])[:7]}), {n / 6000:.1f} min, lateral on {lat.mean():.0%}, solo flag {np.mean((gw & 16) > 0):.0%} of frames")
  print("1. nobody steers while lateral is on - share of lateral-on time by speed, and why")
  for lo, hi in BINS:
    m = lat & (v >= lo) & (v < hi)
    if m.sum() < 300:
      continue
    c = Counter(why[m & nobody].tolist())
    print(f"   {lo:3d}-{hi:3d} km/h {m.sum() / 6000:5.1f} min: {np.mean(nobody[m]):4.0%}  " +
          ", ".join(f"{k} {c[k] / m.sum():.0%}" for k, _ in c.most_common(4)) + f" | driver pressing in {np.mean(pressed[m & nobody]) if (m & nobody).any() else 0:.0%} of it")
  idx = np.flatnonzero(nobody & (v >= 10)); eps = []
  if len(idx):
    brk = np.flatnonzero(np.diff(idx) > 20); st = np.r_[idx[0], idx[brk + 1]]; en = np.r_[idx[brk], idx[-1]]
    eps = [(a, b) for a, b in zip(st, en) if b - a >= 100 and pressed[a:b + 1].mean() < 0.2]
  print(f"   episodes >= 1 s above 10 km/h with the driver not pressing: {len(eps)}")
  for a, b in eps[:40]:
    c = Counter(why[a:b + 1].tolist()).most_common(1)[0][0]
    print(f"     t {T[a]:7.1f} {(b - a) / 100:5.1f} s  {v[a]:3.0f} -> {v[b]:3.0f} km/h  {c:14s} factory LFA symbol {Counter(sym[a:b + 1].tolist()).most_common(1)[0][0]}"
          f"  wheel {d['mdps_ang'][a]:+6.1f} -> {d['mdps_ang'][b]:+6.1f}  max |driver torque| {np.abs(d['tq_s'][a:b + 1]).max():.0f}")
  print("2. switch-overs (who commands the MDPS: R relay, F factory LFA, - none) and lateral edges")
  who = np.where(~cmd, "-", np.where(sym == 2, "F", "R")); who[~lat & cmd & (sym != 2)] = "-"
  ch = np.flatnonzero((who[1:] != who[:-1]) | (lat[1:] != lat[:-1])) + 1; last = -1e9; rows = []
  for i in ch:
    if T[i] - last < 1.0 or v[i] < 10 or i + 120 >= n:
      continue
    last = T[i]; w = d['mdps_ang']
    step = np.abs(w[i + 20:i + 120] - w[i:i + 100]).max()
    rows.append((step, T[i], v[i], f"{who[i - 1]}>{who[min(i + 30, n - 1)]}", int(lat[i - 1]), int(lat[min(i + 30, n - 1)]),
                 bool((gw[i - 50:i + 100] & 32).any()), bool((gw[i - 50:i + 100] & 64).any()), bool(pressed[i - 50:i + 100].any())))
  steps = np.array([x[0] for x in rows]) if rows else np.array([0.0])
  print(f"   {len(rows)} switch-overs above 10 km/h; largest wheel movement in 0.2 s within 1 s: p50 {np.median(steps):.1f} p90 {np.percentile(steps, 90):.1f} max {steps.max():.1f} deg")
  for x in sorted(rows, reverse=True)[:12]:
    print(f"     t {x[1]:7.1f} {x[2]:3.0f} km/h  {x[3]}  lateral {x[4]}->{x[5]}  wheel {x[0]:4.1f} deg / 0.2 s  pulse {int(x[6])} follow {int(x[7])} driver {int(x[8])}")
  pul = np.flatnonzero(((gw[1:] & 32) > 0) & ((gw[:-1] & 32) == 0)) + 1
  m = lat & cmd
  print(f"3. hold-off: {len(pul)} button pulses; while lateral on: factory LFA on {np.mean(sym[lat] > 0):.1%}, active {np.mean(sym[lat] == 2):.1%}; "
        f"of the commanded time relayed {np.mean(sym[m] != 2) if m.any() else 0:.1%}; follow band bound {np.mean((gw[lat] & 64) > 0):.1%}; hold-off paused {np.mean((gw & 128) > 0):.1%}")
