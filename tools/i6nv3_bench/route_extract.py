"""One-pass route extractor (09-23): read a route's rlogs ONCE and save every 100 Hz series the i6nv3 probes need, so
routes can be deleted from the disk-limited container right after extraction and analysed offline.

Saved (carState-rate grid, T relative to the first log message):
  v (km/h), a_ego, ang, rate, pressed, tq_s, tq_eps, lb, rb, gas, brake, lat, apply, wire (LKAS_ALT angle), gain (ACI),
  k_model, k_ctrl, k_meas, lane_min, lcs, pitch-free yaw_rate (livePose, deg/s), accel_cmd (carControl actuators.accel),
  commit, gps (lat, lon interpolated)
Usage: PYTHONPATH=... python tools/i6nv3_bench/route_extract.py <route> <out.npz>
"""
import glob
import sys

import numpy as np

from openpilot.tools.lib.logreader import LogReader

r, out = sys.argv[1], sys.argv[2]
files = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
cs, cc, wr, ct, md, lp, gp = [], [], [], [], [], [], []
t0 = None; commit = ""
for f in files:
  try:
    for m in LogReader(f):
      w = m.which(); t = m.logMonoTime * 1e-9
      if t0 is None:
        t0 = t
      if w == "carState":
        s = m.carState
        cs.append((t, s.vEgo * 3.6, s.aEgo, s.steeringAngleDeg, s.steeringRateDeg, s.steeringPressed, s.steeringTorque,
                   s.steeringTorqueEps, s.leftBlinker, s.rightBlinker, s.gasPressed, s.brakePressed))
      elif w == "carControl":
        c = m.carControl
        cc.append((t, c.latActive, c.actuators.steeringAngleDeg, c.actuators.accel))
      elif w == "controlsState":
        ct.append((t, m.controlsState.desiredCurvature, m.controlsState.curvature))
      elif w == "modelV2":
        d = m.modelV2
        lm = min(d.laneLineProbs[1], d.laneLineProbs[2]) if len(d.laneLineProbs) >= 4 else 1.0
        md.append((t, d.action.desiredCurvature, lm, int(d.meta.laneChangeState.raw)))
      elif w == "livePose":
        lp.append((t, np.degrees(m.livePose.angularVelocityDevice.z)))
      elif w in ("gpsLocationExternal", "gpsLocation"):
        g = getattr(m, w)
        if g.latitude != 0.0:
          gp.append((t, g.latitude, g.longitude))
      elif w == "initData" and not commit:
        commit = m.initData.gitCommit[:9]
      elif w == "sendcan":
        for c in m.sendcan:
          if c.address == 272 and len(c.dat) >= 13:
            dd = bytes(c.dat); raw = ((dd[10] >> 2) | (dd[11] << 6)) & 0x3FFF
            if raw & 0x2000:
              raw -= 0x4000
            wr.append((t, raw * 0.1, dd[12] * 0.004))
  except Exception as e:  # noqa: BLE001 - a truncated last segment must not lose the route
    print(f"  {f.split('/')[-1]}: {type(e).__name__}: {e}")
CS = np.array(cs, float)
if not len(CS):
  print(f"== {r}: no carState"); sys.exit(0)
T = CS[:, 0]
def res(a, col, default=0.0):
  A = np.array(a, float)
  return np.interp(T, A[:, 0], A[:, col]) if len(A) else np.full(len(T), default)
np.savez_compressed(out, T=T - t0, v=CS[:, 1], a_ego=CS[:, 2], ang=CS[:, 3], rate=CS[:, 4], pressed=CS[:, 5] > 0.5,
                    tq_s=CS[:, 6], tq_eps=CS[:, 7], lb=CS[:, 8] > 0.5, rb=CS[:, 9] > 0.5, gas=CS[:, 10] > 0.5,
                    brake=CS[:, 11] > 0.5, lat=res(cc, 1) > 0.5, apply=res(cc, 2), accel_cmd=res(cc, 3),
                    wire=res(wr, 1), gain=res(wr, 2), k_ctrl=res(ct, 1), k_meas=res(ct, 2), k_model=res(md, 1),
                    lane_min=res(md, 2, 1.0), lcs=res(md, 3), yaw=res(lp, 1), gps_lat=res(gp, 1), gps_lon=res(gp, 2),
                    commit=np.array(commit))
print(f"== {r}: {len(T) / 6000:.1f} min, commit {commit}, lat active {np.mean(res(cc, 1) > 0.5) * 100:.0f}%")
