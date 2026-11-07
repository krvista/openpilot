"""E106: is paramsd's road roll a device-mount tilt? The yaw rate (gyro, calibrated to the car) and the wheel angle do
not depend on how the device is rolled, so k_yaw - k_angle(roll = 0) = g sin(roll_true) / v^2 (+ an angle-offset term,
constant in curvature). Per route (v >= 40 km/h, |yaw rate| < 0.06 rad/s, 1 s averages): regress the residual on
g / v^2 with an intercept -> roll_true (mean over the drive), and compare with paramsd's roll averaged over the same
samples. The difference is the mount tilt + estimator bias."""
import glob, sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.hyundai.interface import CarInterface
D = sys.argv[1]
CP = CarInterface.get_non_essential_params("HYUNDAI_IONIQ_6_N"); VM = VehicleModel(CP)
for r in sys.argv[2:]:
  fs = sorted(glob.glob(f"{D}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
  cal = PoseCalibrator(); ang = None; v = 0.0; off = 0.0; roll_p = 0.0; sr = CP.steerRatio; stiff = 1.0; yr = None
  acc = []; cur = []; t_last = None
  for f in fs:
    for m in LogReader(f):
      try:
        w = m.which(); t = m.logMonoTime * 1e-9
        if w == "carState": ang = m.carState.steeringAngleDeg; v = m.carState.vEgo
        elif w == "vehicleParameters":
          p = m.vehicleParameters; off = p.angleOffsetDeg; roll_p = p.roll; sr = p.steerRatio; stiff = p.stiffnessFactor
        elif w == "extrinsicsCalibration": cal.feed_extrinsics_calibration(m.extrinsicsCalibration)
        elif w == "deviceMotion" and cal.calib_valid and ang is not None and v > 40 / 3.6:
          pose = cal.build_calibrated_pose(Pose.from_device_motion(m.deviceMotion)); yr = pose.angular_velocity.yaw
          if abs(yr) < 0.06:
            VM.update_params(max(stiff, 0.1), max(sr, 0.1))
            k_yaw = yr / v; k_ang = -VM.calc_curvature(np.radians(ang - off), v, 0.0)   # sign as controlsd: curvature = -calc_curvature(...)
            cur.append((t, k_yaw, k_ang, v, roll_p))
            if t_last is None: t_last = t
            if t - t_last >= 1.0:
              a = np.array(cur); acc.append(a.mean(0)); cur = []; t_last = t
      except Exception:  # noqa: BLE001
        pass
  A = np.array(acc)
  if len(A) < 200:
    print(r, "too few samples", len(A)); continue
  y = A[:, 1] - A[:, 2]; x = 9.81 / A[:, 3] ** 2
  # residual curvature = sin(roll_true) * g / v^2 + c   (c: angle-offset-like constant)
  X = np.c_[x, np.ones_like(x)]
  for sgn, nm in ((1.0, "k_yaw - k_ang"), ):
    s, c = np.linalg.lstsq(X, sgn * y, rcond=None)[0]
    pred = X @ np.array([s, c]); rms = np.sqrt(np.mean((sgn * y - pred) ** 2))
    print(f"{r}: n {len(A)} 1-s samples, v p50 {np.median(A[:, 3]) * 3.6:.0f} km/h | fit {nm} = sin(roll) g/v^2 + c: roll_true {np.degrees(np.arcsin(np.clip(s, -1, 1))):+.2f} deg, c {c * 1e3:+.3f}e-3 /m (= {np.degrees(c * VM.sR * VM.l):+.2f} deg wheel), resid RMS {rms * 1e3:.3f}e-3 "
          f"| paramsd roll mean {np.degrees(A[:, 4].mean()):+.2f} deg (p25/p75 {np.degrees(np.percentile(A[:, 4], 25)):+.2f}/{np.degrees(np.percentile(A[:, 4], 75)):+.2f}) | corr(residual, paramsd roll term) {np.corrcoef(y, np.sin(A[:, 4]) * x)[0, 1]:+.2f}")
  # per-sample implied roll vs paramsd roll (straights, 40-90 km/h)
  imp = np.degrees(np.arcsin(np.clip((y - c) / x, -1, 1)))
  d = imp - np.degrees(A[:, 4])
  print(f"    implied roll p25/50/75 {np.percentile(imp, 25):+.2f}/{np.median(imp):+.2f}/{np.percentile(imp, 75):+.2f} | paramsd - implied p25/50/75 {np.percentile(-d, 25):+.2f}/{np.median(-d):+.2f}/{np.percentile(-d, 75):+.2f} deg", flush=True)
