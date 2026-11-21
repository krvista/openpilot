"""E163 (report §99): where is the car REALLY in its lane, and where does openpilot's model think it is?
The factory camera publishes its own lane-line positions (FR_CMR_03_50ms 0x1B5: Info_LftLnPosVal / Info_RtLnPosVal, m,
with quality), referenced to the car. modelV2's inner lane lines are in the model's (virtually shifted, Params
CameraOffset) camera frame. Same instants, straight road (|wheel| < 2 deg), >= 50 km/h, no blinker within 3 s, both of
the factory's lines at quality >= 2 and the model's inner-line probabilities >= 0.6:
  factory offset   = car's lateral position from the lane centre by the factory camera (+ = right of centre)
  model offset     = the same by the model's lines
  D = model - factory   how far RIGHT of the factory camera's reference the model's virtual camera sits
  lane width by both (sanity)
grouped by who steers (factory LFA active / openpilot relayed / manual). argv: rlog [rlog ...]"""
import sys
from collections import defaultdict
import numpy as np
from openpilot.tools.lib.logreader import LogReader


def sbits(d, start, size):
  v = (int.from_bytes(d, "little") >> start) & ((1 << size) - 1)
  return v - (1 << size) if v & (1 << (size - 1)) else v


G = defaultdict(lambda: defaultdict(list)); offs = set(); buses = defaultdict(int)
for f in sys.argv[1:]:
  fac = None; lfa = -1; lat = False; v = 0.0; ang = 0.0; blink_t = -1e9; lvl2 = 0
  try:
    for m in LogReader(f):
      t = m.logMonoTime * 1e-9; w = m.which()
      if w == "can":
        for c in m.can:
          if c.address == 437 and len(c.dat) >= 20 and c.src in (0, 1, 2):
            d = bytes(c.dat); buses[c.src] += 1
            if c.src == 1 or fac is None or fac[5] != 1:
              fac = (t, sbits(d, 29, 14) * 0.0039625, sbits(d, 101, 14) * 0.0039625, (d[3] & 7), (d[12] & 7), c.src)
          elif c.address == 480 and c.src == 1 and len(c.dat) >= 8:
            lfa = (int.from_bytes(bytes(c.dat)[:8], "little") >> 47) & 3
          elif c.address == 203 and c.src == 1 and len(c.dat) >= 9:
            lvl2 = c.dat[3] >> 4
      elif w == "carState":
        v = m.carState.vEgo * 3.6; ang = m.carState.steeringAngleDeg
        if m.carState.leftBlinker or m.carState.rightBlinker:
          blink_t = t
      elif w == "carControl":
        lat = bool(m.carControl.latActive)
      elif w == "initData":
        for e in m.initData.params.entries:
          if e.key == "CameraOffset":
            offs.add(e.value.decode() if isinstance(e.value, bytes) else str(e.value))
      elif w == "modelV2" and fac is not None and t - fac[0] < 0.1:
        md = m.modelV2
        if len(md.laneLines) < 4 or len(md.laneLineProbs) < 4:
          continue
        if min(md.laneLineProbs[1], md.laneLineProbs[2]) < 0.6 or min(fac[3], fac[4]) < 2:
          continue
        if v < 50 or abs(ang) > 2.0 or t - blink_t < 3.0:
          continue
        l1 = float(np.interp(0.0, md.laneLines[1].x, md.laneLines[1].y)); l2 = float(np.interp(0.0, md.laneLines[2].x, md.laneLines[2].y))
        who = "manual" if lvl2 != 2 else ("factory LFA" if lfa == 2 and not lat else ("openpilot relayed" if lat and lfa != 2 else ("mixed" if lat else "other")))
        g = G[who]
        g['fl'].append(fac[1]); g['fr'].append(fac[2]); g['ml'].append(l1); g['mr'].append(l2); g['v'].append(v)
  except Exception as e:  # noqa: BLE001
    print(f"({f.split('/')[-1]}: {type(e).__name__})")
print(f"CameraOffset in these logs: {sorted(offs)}; FR_CMR_03_50ms frames per bus: {dict(buses)}")
allf = np.concatenate([np.array(g['fl']) for g in G.values()]); allr = np.concatenate([np.array(g['fr']) for g in G.values()])
left_pos = np.median(allf) > 0          # factory convention: left line positive?
print(f"factory lines: left median {np.median(allf):+.2f} m, right {np.median(allr):+.2f} m -> {'y to the LEFT' if left_pos else 'y to the RIGHT'} is positive")
print("who steers          n(20 Hz)  km/h | factory offset cm: mean  p25  p75 | model offset cm: mean | D = model - factory cm: mean  p25  p75 | lane width m factory / model")
for who, g in sorted(G.items()):
  fl, fr, ml, mr = (np.array(g[k]) for k in ('fl', 'fr', 'ml', 'mr'))
  if len(fl) < 100:
    continue
  fo = ((fl + fr) / 2 if left_pos else -(fl + fr) / 2) * 100      # + = car right of the lane centre
  mo = -(ml + mr) / 2 * 100
  D = mo - fo
  print(f"{who:18s} {len(fl):7d}  {np.mean(g['v']):5.0f} | {fo.mean():+6.1f} {np.percentile(fo, 25):+6.1f} {np.percentile(fo, 75):+6.1f}       | {mo.mean():+6.1f}"
        f"                | {D.mean():+6.1f} {np.percentile(D, 25):+6.1f} {np.percentile(D, 75):+6.1f}           | {np.median(np.abs(fl - fr)):.2f} / {np.median(mr - ml):.2f}"
        f"   corr(model, factory) {np.corrcoef(mo, fo)[0, 1]:+.2f}")
