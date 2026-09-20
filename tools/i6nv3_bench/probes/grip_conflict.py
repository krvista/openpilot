"""Grip-conflict probe (09-20): "I was holding the wheel firmly and steering the turn myself, and op steered HARDER and
the car lurched" (Euljiro -> Lotte main store left turn).

Finds windows where the driver is clearly steering (pressed, |column torque| >= GRIP_NM) while lat is active, and op's
transmitted request is ahead of the wheel in the driver's own steering direction (request - wheel, signed by the
driver's torque direction, >= LEAD_DEG) with the ACI gain on the wire above the yield floor. Prints, per episode: GPS,
speed, blinker, wheel/request/gain profile, the driver torque, and the wheel-rate jump (the "lurch": max |d wheel| over
0.2 s inside the episode vs the 1 s before). Optional --near lat,lon[,radius_m] filters by GPS.
Usage: PYTHONPATH=... python probes/grip_conflict.py <route> [--near 37.5662,126.9822,200] [--list]
"""
import glob
import math
import sys

import numpy as np

from openpilot.tools.lib.logreader import LogReader

GRIP_NM, LEAD_DEG, GAIN_MIN, MIN_FRAMES = 300.0, 5.0, 0.20, 20
r = sys.argv[1]
near = None
if "--near" in sys.argv:
  p = [float(x) for x in sys.argv[sys.argv.index("--near") + 1].split(",")]
  near = (p[0], p[1], p[2] if len(p) > 2 else 200.0)
files = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
cs, cc, wr, gp, ct = [], [], [], [], []
t0 = None
for f in files:
  for m in LogReader(f):
    w = m.which(); t = m.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    if w == "carState":
      s = m.carState
      cs.append((t, s.vEgo * 3.6, s.steeringAngleDeg, s.steeringPressed, s.steeringTorque, s.leftBlinker, s.rightBlinker))
    elif w == "carControl":
      cc.append((t, m.carControl.latActive, m.carControl.actuators.steeringAngleDeg))
    elif w == "controlsState":
      ct.append((t, m.controlsState.desiredCurvature))
    elif w in ("gpsLocationExternal", "gpsLocation"):
      g = getattr(m, w)
      if g.latitude != 0.0:
        gp.append((t, g.latitude, g.longitude))
    elif w == "sendcan":
      for c in m.sendcan:
        if c.address == 272 and len(c.dat) >= 13:
          d = bytes(c.dat); raw = ((d[10] >> 2) | (d[11] << 6)) & 0x3FFF
          if raw & 0x2000:
            raw -= 0x4000
          wr.append((t, raw * 0.1, d[12] * 0.004))
CS, CC, WR = np.array(cs, float), np.array(cc, float), np.array(wr, float)
if min(len(CS), len(CC), len(WR)) == 0:
  print(f"== {r}: no data"); sys.exit(0)
T = CS[:, 0]; v = CS[:, 1]; ang = CS[:, 2]; pressed = CS[:, 3] > 0.5; tq = CS[:, 4]; lb = CS[:, 5] > 0.5; rb = CS[:, 6] > 0.5
lat = np.interp(T, CC[:, 0], CC[:, 1]) > 0.5; apply = np.interp(T, CC[:, 0], CC[:, 2])
wire = np.interp(T, WR[:, 0], WR[:, 1]); gain = np.interp(T, WR[:, 0], WR[:, 2])
if gp:
  GP = np.array(gp, float); lat_deg = np.interp(T, GP[:, 0], GP[:, 1]); lon_deg = np.interp(T, GP[:, 0], GP[:, 2])
else:
  lat_deg = lon_deg = np.zeros(len(T))
def dist_m(la, lo, la0, lo0):
  return math.hypot((la - la0) * 111320.0, (lo - lo0) * 111320.0 * math.cos(math.radians(la0)))
n = len(T)
drv = np.sign(tq)                                    # +tq = +angle = left on this car (checked 89-99 %)
lead = (wire - ang) * drv                            # request ahead of the wheel in the driver's direction
cond = lat & pressed & (np.abs(tq) >= GRIP_NM) & (lead >= LEAD_DEG) & (gain >= GAIN_MIN) & (v >= 5)
if near is not None:
  cond &= np.array([dist_m(lat_deg[i], lon_deg[i], near[0], near[1]) <= near[2] for i in range(n)])
edges = np.diff(np.concatenate([[0], cond.astype(int), [0]])); st = np.where(edges == 1)[0]; en = np.where(edges == -1)[0]
eps = []
for a, b in zip(st, en):
  if b - a < MIN_FRAMES or a < 200 or b + 100 >= n:
    continue
  if eps and a - eps[-1]["b"] < 100:                 # merge episodes closer than 1 s
    eps[-1]["b"] = b; continue
  eps.append(dict(a=a, b=b))
wrate = np.zeros(n); wrate[20:] = np.abs(ang[20:] - ang[:-20]) * 5   # deg/s over 0.2 s
print(f"== {r}: grip-conflict episodes {len(eps)} (pressed, |tq|>={GRIP_NM:.0f} Nm, request ahead of wheel >= {LEAD_DEG:.0f} deg in the driver's direction, gain >= {GAIN_MIN})")
for e in eps:
  a, b = e["a"], e["b"]; w = slice(a, b)
  pre = slice(max(a - 100, 0), a)
  blink = "L" if lb[w].mean() > 0.5 else ("R" if rb[w].mean() > 0.5 else "-")
  print(f"  seg{int((T[a] - t0) // 60):2d} t={T[a] - t0:7.1f} dur={(b - a) / 100:4.1f}s v={v[w].mean():3.0f} km/h blink={blink} gps={lat_deg[a]:.5f},{lon_deg[a]:.5f} | "
        f"wheel {ang[a]:+5.1f}->{ang[b]:+5.1f} | request {wire[a]:+5.1f}->{wire[b]:+5.1f} (lead p50 {np.median(lead[w]):4.1f} max {lead[w].max():4.1f} deg) | "
        f"gain p50/max {np.median(gain[w]):.2f}/{gain[w].max():.2f} | driver tq p50/max {np.median(np.abs(tq[w])):4.0f}/{np.abs(tq[w]).max():4.0f} Nm | "
        f"wheel rate max {wrate[w].max():4.0f} deg/s (pre {wrate[pre].max():3.0f})")
  if "--list" in sys.argv:
    for i in range(a - 50, min(b + 50, n), 10):
      print(f"      t={T[i] - t0:7.2f} v={v[i]:3.0f} wheel {ang[i]:+6.1f} req {wire[i]:+6.1f} apply {apply[i]:+6.1f} gain {gain[i]:.2f} tq {tq[i]:+5.0f} pressed {int(pressed[i])} lb {int(lb[i])}")
