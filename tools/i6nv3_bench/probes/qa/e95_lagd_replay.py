"""E95: offline replay of the real lagd (LateralLagEstimator) on cached route inputs (e95_lagd_cache.py): why so few
estimates are accepted on this car, and what a variant of the acceptance settings would give. Per route (each route
starts like a fresh ignition with the cached valid blocks): point-okay reasons (share of op time above min speed), the
4 Hz estimate outcome (warm-up / too few okay points / lateral-accel range / no new points / NCC / confidence /
accepted), accepted per minute, and the accepted delays (median, IQR).
Variants: name:key=val;... keys: min_vego_kph, min_ncc, min_conf, okay_window, recovery, la_range, press_min_s."""
import pickle, sys
import numpy as np
from collections import Counter
import types
for _m in ("openpilot.cereal.messaging", "openpilot.common.params", "openpilot.common.realtime", "openpilot.sunnypilot.livedelay.lagd_toggle"):
  if _m not in sys.modules:   # device-only modules lagd imports at top level; the estimator itself does not use them
    _s = types.ModuleType(_m); _s.Params = object; _s.config_realtime_process = lambda *a: None; _s.LagdToggle = object
    _s.new_message = None; sys.modules[_m] = _s
from openpilot.cereal import log
import openpilot.selfdrive.locationd.lagd as L
S = sys.argv[1]; ROUTES = sys.argv[2].split(","); VARS = [a.split(":", 1) for a in sys.argv[3:]] or [["stock", ""]]
class _CP:
  steerActuatorDelay = 0.1
class Est(L.LateralLagEstimator):
  def __init__(self, cfg):
    kw = dict(min_vego=cfg.get('min_vego_kph', 80.4672) / 3.6, min_ncc=cfg.get('min_ncc', L.MIN_NCC),
              min_confidence=cfg.get('min_conf', L.MIN_CONFIDENCE), okay_window_sec=cfg.get('okay_window', L.MIN_OKAY_WINDOW_SEC),
              min_recovery_buffer_sec=cfg.get('recovery', L.MIN_RECOVERY_BUFFER_SEC))
    super().__init__(_CP(), 0.05, **kw)
    self.la_range = cfg.get('la_range', L.MIN_LAT_ACCEL_RANGE); self.press_min = cfg.get('press_min_s', 0.0)
    self.press_start = None; self.out = Counter(); self.pt = Counter(); self.acc = []
  def handle_log(self, t, which, msg):
    super().handle_log(t, which, msg)
    if which == "carState" and self.press_min > 0.0:   # only a press held >= press_min_s counts (HOD touches pass)
      if msg.steeringPressed:
        self.press_start = t if self.press_start is None else self.press_start
        self.steering_pressed = (t - self.press_start) >= self.press_min
      else:
        self.press_start = None; self.steering_pressed = False
  def update_points(self):
    if self.lat_active and self.v_ego > self.min_vego:
      tags = []
      if self.steering_pressed or self.t - self.last_steering_pressed_t < self.min_recovery_buffer_sec: tags.append("press")
      if self.steering_saturated or self.t - self.last_steering_saturated_t < self.min_recovery_buffer_sec: tags.append("sat")
      if not self.calibrator.calib_valid: tags.append("calib")
      if self.t - self.last_pose_invalid_t < self.min_recovery_buffer_sec: tags.append("pose/la")
      self.pt[tags[0] if tags else "ok"] += 1
    super().update_points()
  def update_estimate(self):
    if not self.points_enough():
      self.out["warmup"] += 1; return
    times, desired, actual, okay = self.points.get()
    if not self.points_valid(): self.out["few okay"] += 1; return
    if actual.max() - actual.min() < self.la_range: self.out["la range"] += 1; return
    if self.last_estimate_t != 0 and times[0] <= self.last_estimate_t:
      ni = next(-i for i, tt in enumerate(reversed(times)) if tt <= self.last_estimate_t)
      if ni == 0 or not np.any(okay[ni:]): self.out["no new"] += 1; return
    desired = L.masked_symmetric_moving_average(desired, okay, L.SMOOTH_K, L.SMOOTH_SIGMA)
    actual = L.masked_symmetric_moving_average(actual, okay, L.SMOOTH_K, L.SMOOTH_SIGMA)
    delay, corr, conf = self.actuator_delay(desired, actual, okay, self.dt, L.MIN_LAG, L.MAX_LAG)
    if corr < self.min_ncc: self.out["ncc"] += 1; return
    if conf < self.min_confidence: self.out["conf"] += 1; return
    self.out["ACCEPT"] += 1; self.acc.append(delay)
    self.block_avg.update(delay); self.last_estimate_t = self.t

def run(route, cfg, vb0, lag0):
  ev = pickle.load(open(f"{S}/lagd/{route}.pkl", "rb"))
  e = Est(cfg); e.reset(lag0, vb0); n = 0; t0 = ev[0][0]   # as retrieve_initial_lag: saved estimate + completed blocks
  for ts, which, b in ev:
    with log.Event.from_bytes(b) as m:
      e.handle_log(ts * 1e-9, which, getattr(m, which))
    if which == "deviceMotion":
      e.update_points(); n += 1
      if n % 5 == 0:
        e.update_estimate()
  return e, (ev[-1][0] - t0) * 1e-9

for name, spec in VARS:
  cfg = {k: float(v) for k, v in (kv.split("=") for kv in spec.split(";") if kv)}
  vb = 0; lag0 = 0.3; allacc = []
  print(f"== {name} {cfg}")
  for r in ROUTES:
    e, dur = run(r, cfg, vb, lag0)
    vb = e.block_avg.valid_blocks; allacc += e.acc
    _, _, cm_, _ = e.block_avg.get(); lag0 = cm_ if np.isfinite(cm_) else lag0
    tot_pt = sum(e.pt.values()); tot_out = sum(e.out.values())
    vm, vs, cm, cs = e.block_avg.get()
    print(f"  {r}: {dur / 60:5.1f} min, op above min speed {tot_pt * 0.05 / 60:5.1f} min | points: " + " ".join(f"{k} {v / max(tot_pt, 1):.0%}" for k, v in e.pt.most_common())
          + f" | estimates: " + " ".join(f"{k} {v}" for k, v in e.out.most_common()) + f" | accepted {e.out['ACCEPT']} ({e.out['ACCEPT'] / max(tot_pt * 0.05 / 60, 1e-9):.1f}/min)"
          f" -> valid blocks {vb}, idx {e.block_avg.idx} (lost at ignition off) | status: valid mean {vm:.3f} std {vs:.3f} -> "
          + ("ESTIMATED" if vb >= L.BLOCK_NUM_NEEDED and vs <= L.MAX_LAG_STD else ("INVALID (std)" if vb >= L.BLOCK_NUM_NEEDED else "unestimated")), flush=True)
  if allacc:
    a = np.array(allacc)
    print(f"  accepted delays: n {len(a)} median {np.median(a):.3f} IQR {np.percentile(a, 25):.3f}-{np.percentile(a, 75):.3f} s | final valid mean {vm:.3f} std {vs:.3f}")
