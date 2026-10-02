"""One-pass route extractor (09-23; v2 10-02): read a route's rlogs ONCE and save every 100 Hz series the i6nv3 probes
need, so routes can be deleted from the disk-limited container right after extraction and analysed offline.

Saved (carState-rate grid, T relative to the first log message):
  v (km/h), a_ego, ang, rate, pressed, tq_s, tq_eps, lb, rb, gas, brake, bsl, bsr, lat, apply, accel_cmd, wire (LKAS_ALT
  angle), gain (ACI), tx_active (LKAS_ALT LKAS_ANGLE_ACTIVE), k_model, k_ctrl, k_meas, gap (steerCmdGapDeg), lane_min, lcs,
  yaw (deg/s), mdps_active (0xEA LKA_ANGLE_ACTIVE), mdps_fault, mdps_tq (STEERING_OUT_TORQUE), tx_reject (0x110 echoes
  with src >= 192 per frame), sd_active (selfdriveState.active), gps_lat/lon
  events: list of (t, name) for onroadEvents transitions; commit
Usage: PYTHONPATH=... python tools/i6nv3_bench/route_extract.py <route> <out.npz>
"""
import glob
import sys

import numpy as np

from openpilot.tools.lib.logreader import LogReader

r, out = sys.argv[1], sys.argv[2]
files = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
cs, cc, wr, ct, md, lp, gp, mp, rj, sd = [], [], [], [], [], [], [], [], [], []
events = []; prev_ev = set()
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
                   s.steeringTorqueEps, s.leftBlinker, s.rightBlinker, s.gasPressed, s.brakePressed, s.leftBlindspot,
                   s.rightBlindspot))
      elif w == "carControl":
        c = m.carControl
        cc.append((t, c.latActive, c.actuators.steeringAngleDeg, c.actuators.accel))
      elif w == "controlsState":
        x = m.controlsState
        ct.append((t, x.desiredCurvature, x.curvature, getattr(x, "steerCmdGapDeg", 0.0)))
      elif w == "modelV2":
        d = m.modelV2
        lm = min(d.laneLineProbs[1], d.laneLineProbs[2]) if len(d.laneLineProbs) >= 4 else 1.0
        md.append((t, d.action.desiredCurvature, lm, int(d.meta.laneChangeState.raw)))
      elif w == "livePose":
        lp.append((t, np.degrees(m.livePose.angularVelocityDevice.z)))
      elif w == "selfdriveState":
        sd.append((t, m.selfdriveState.active))
      elif w == "onroadEvents":
        cur = {str(e.name) for e in m.onroadEvents}
        for nm in cur - prev_ev:
          events.append((t - t0, nm))
        prev_ev = cur
      elif w in ("gpsLocationExternal", "gpsLocation"):
        g = getattr(m, w)
        if g.latitude != 0.0:
          gp.append((t, g.latitude, g.longitude))
      elif w == "initData" and not commit:
        commit = m.initData.gitCommit[:9]
      elif w == "can":
        nrej = 0
        for c in m.can:
          if c.address == 234 and c.src < 128 and len(c.dat) >= 24:
            dd = bytes(c.dat)
            mp.append((t, dd[18] & 0x3, (dd[18] >> 5) & 1, ((dd[8] | (dd[9] << 8)) & 0xFFF) * 0.1 - 204.8))
          elif c.address == 272 and c.src >= 192:
            nrej += 1
        if nrej:
          rj.append((t, nrej))
      elif w == "sendcan":
        for c in m.sendcan:
          if c.address == 272 and len(c.dat) >= 13:
            dd = bytes(c.dat); raw = ((dd[10] >> 2) | (dd[11] << 6)) & 0x3FFF
            if raw & 0x2000:
              raw -= 0x4000
            wr.append((t, raw * 0.1, dd[12] * 0.004, (dd[9] >> 4) & 0x3))
  except Exception as e:  # noqa: BLE001 - a truncated last segment must not lose the route
    print(f"  {f.split('/')[-1]}: {type(e).__name__}: {e}")
CS = np.array(cs, float)
if not len(CS):
  print(f"== {r}: no carState"); sys.exit(0)
T = CS[:, 0]
def res(a, col, default=0.0, kind="interp"):
  A = np.array(a, float)
  if not len(A):
    return np.full(len(T), default)
  if kind == "hold":
    idx = np.clip(np.searchsorted(A[:, 0], T, side="right") - 1, 0, len(A) - 1)
    return A[idx, col]
  return np.interp(T, A[:, 0], A[:, col])
rej = np.zeros(len(T))
if rj:
  RJ = np.array(rj, float); idx = np.clip(np.searchsorted(T, RJ[:, 0]), 0, len(T) - 1); np.add.at(rej, idx, RJ[:, 1])
np.savez_compressed(out, T=T - t0, v=CS[:, 1], a_ego=CS[:, 2], ang=CS[:, 3], rate=CS[:, 4], pressed=CS[:, 5] > 0.5,
                    tq_s=CS[:, 6], tq_eps=CS[:, 7], lb=CS[:, 8] > 0.5, rb=CS[:, 9] > 0.5, gas=CS[:, 10] > 0.5,
                    brake=CS[:, 11] > 0.5, bsl=CS[:, 12] > 0.5, bsr=CS[:, 13] > 0.5,
                    lat=res(cc, 1, kind="hold") > 0.5, apply=res(cc, 2), accel_cmd=res(cc, 3),
                    wire=res(wr, 1), gain=res(wr, 2), tx_active=res(wr, 3, kind="hold"),
                    k_ctrl=res(ct, 1), k_meas=res(ct, 2), gap=res(ct, 3), k_model=res(md, 1),
                    lane_min=res(md, 2, 1.0), lcs=res(md, 3, kind="hold"), yaw=res(lp, 1),
                    mdps_active=res(mp, 1, kind="hold"), mdps_fault=res(mp, 2, kind="hold"), mdps_tq=res(mp, 3),
                    tx_reject=rej, sd_active=res(sd, 1, kind="hold") > 0.5,
                    gps_lat=res(gp, 1), gps_lon=res(gp, 2), commit=np.array(commit),
                    events=np.array(events, dtype=object))
print(f"== {r}: {len(T) / 6000:.1f} min, commit {commit}, lat active {np.mean(res(cc, 1, kind='hold') > 0.5) * 100:.0f}%, "
      f"events {len(events)}")
