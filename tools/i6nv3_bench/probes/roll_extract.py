"""Per modelV2 frame (20 Hz): vision roll (lane-line z), device roll (deviceMotion), paramsd roll/offsets, car state.
Usage: extract.py <route id> <out.npz> [root]"""
import glob, sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
r, out = sys.argv[1], sys.argv[2]; root = sys.argv[3] if len(sys.argv) > 3 else "/home/user/drivelog/drivelog"
files = sorted(glob.glob(f"{root}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
st = dict(v=0., yaw=0., ang=0., lat=0, a=0., dm_roll=np.nan, dm_roll_std=np.nan, dm_pitch=np.nan, vp_roll=np.nan, vp_off=np.nan,
          vp_off_avg=np.nan, vp_sr=np.nan, vp_valid=0, cal_r=np.nan, cal_p=np.nan, cal_y=np.nan, cal_status=-1,
          lat_g=np.nan, lon_g=np.nan, bearing=np.nan, ax=np.nan, ay=np.nan, az=np.nan, steer_pressed=0)
rows = []; t0 = None
IDX = (0, 7, 8, 10)   # x ~ 0, 9.3, 12.2, 19 m
for f in files:
  try:
    for m in LogReader(f):
      w = m.which(); t = m.logMonoTime * 1e-9
      if t0 is None: t0 = t
      if w == "carState":
        cs = m.carState; st.update(v=cs.vEgo, yaw=cs.yawRate, ang=cs.steeringAngleDeg, a=cs.aEgo, steer_pressed=int(cs.steeringPressed))
      elif w == "carControl": st["lat"] = int(m.carControl.latActive)
      elif w == "deviceMotion":
        o = m.deviceMotion.orientationNED; st.update(dm_roll=o.x, dm_roll_std=o.xStd, dm_pitch=o.y)
      elif w == "vehicleParameters":
        p = m.vehicleParameters; st.update(vp_roll=p.roll, vp_off=p.angleOffsetDeg, vp_off_avg=p.angleOffsetAverageDeg, vp_sr=p.steerRatio, vp_valid=int(p.valid))
      elif w == "extrinsicsCalibration":
        c = m.extrinsicsCalibration; rpy = list(c.rpyCalib)
        if len(rpy) == 3: st.update(cal_r=rpy[0], cal_p=rpy[1], cal_y=rpy[2])
        st["cal_status"] = c.calStatus.raw
      elif w == "gpsLocationExternal":
        g = m.gpsLocationExternal
        if g.hasFix: st.update(lat_g=g.latitude, lon_g=g.longitude, bearing=g.bearingDeg if g.speed > 3 else np.nan)
      elif w == "accelerometer":
        v = list(m.accelerometer.acceleration.v)
        if len(v) == 3: st.update(ax=v[0], ay=v[1], az=v[2])
      elif w == "modelV2":
        d = m.modelV2; L = list(d.laneLines); E = list(d.roadEdges); pr = list(d.laneLineProbs)
        if len(L) < 4 or len(pr) < 4: continue
        rec = [t - t0] + [st[k] for k in ("v", "yaw", "ang", "lat", "a", "dm_roll", "dm_roll_std", "dm_pitch", "vp_roll", "vp_off", "vp_off_avg",
                                           "vp_sr", "vp_valid", "cal_r", "cal_p", "cal_y", "cal_status", "lat_g", "lon_g", "bearing", "ax", "ay", "az", "steer_pressed")]
        rec += [pr[1], pr[2], pr[0], pr[3]]
        for i in IDX:
          rec += [L[1].y[i], L[1].z[i], L[2].y[i], L[2].z[i], L[0].y[i], L[0].z[i], L[3].y[i], L[3].z[i], E[0].y[i], E[0].z[i], E[1].y[i], E[1].z[i]]
        rec += [d.position.y[10], d.position.z[10] if len(d.position.z) > 10 else np.nan]
        rows.append(rec)
  except Exception as e:
    print("read error", f, type(e).__name__, file=sys.stderr)
cols = ["t", "v", "yaw", "ang", "lat", "a", "dm_roll", "dm_roll_std", "dm_pitch", "vp_roll", "vp_off", "vp_off_avg", "vp_sr", "vp_valid", "cal_r", "cal_p", "cal_y",
        "cal_status", "lat_g", "lon_g", "bearing", "ax", "ay", "az", "steer_pressed", "pl", "pr", "pll", "prr"]
for i in IDX:
  cols += [f"{k}{i}" for k in ("yL", "zL", "yR", "zR", "yLL", "zLL", "yRR", "zRR", "yEL", "zEL", "yER", "zER")]
cols += ["py10", "pz10"]
A = np.array(rows, dtype=float)
np.savez(out, **{c: A[:, j] for j, c in enumerate(cols)})
print(r, A.shape, f"{A[-1,0]/60:.1f} min")
