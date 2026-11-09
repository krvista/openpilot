"""E111: where lagd's learning data is lost, and whether a touch that does not disturb steering can be kept. Replays
the real estimator on the cached inputs (e95_lagd_cache), carrying valid blocks across routes as ignitions do. Point
losses are split: slow (< min_vego), press-now, recovery-buffer (after a press), saturated, not-turning, pose/la.
When steeringPressed is true during op & >= min_vego it records |steeringTorque| (STEER_THRESHOLD = 350 trips it) and
|la_desired - la_actual_pose| (the lateral disturbance), so a torque threshold for 'light touch' can be read from data.
Then it measures a variant where a press is 'steering' only if |torque| >= TQ or the lateral track diverged > LA: usable
okay-point-minutes and accepted estimates before/after. argv: S routes_csv [mode]"""
import pickle, sys, types
import numpy as np
from collections import Counter
for _m in ("openpilot.cereal.messaging", "openpilot.common.params", "openpilot.common.realtime", "openpilot.sunnypilot.livedelay.lagd_toggle"):
  if _m not in sys.modules:
    _s = types.ModuleType(_m); _s.Params = object; _s.config_realtime_process = lambda *a: None; _s.LagdToggle = object; sys.modules[_m] = _s
from openpilot.cereal import log
import openpilot.selfdrive.locationd.lagd as L
S = sys.argv[1]; ROUTES = sys.argv[2].split(",")
class _CP: steerActuatorDelay = 0.1
class Est(L.LateralLagEstimator):
  def __init__(self, cfg):
    super().__init__(_CP(), 0.05, min_vego=60/3.6, min_ncc=0.9, min_confidence=0.5, min_recovery_buffer_sec=1.0)
    self.mode = cfg.get("mode", "64a"); self.tq_thr = cfg.get("tq", 500.0); self.la_thr = cfg.get("la", 0.3)
    self.press_min = cfg.get("press_min", 0.5); self.press_start = None
    self.raw_pressed = False; self.tq = 0.0; self.pt = Counter(); self.out = Counter(); self.acc = []; self.touch = []
  def handle_log(self, t, which, msg):
    super().handle_log(t, which, msg)
    if which == "carState":
      self.raw_pressed = msg.steeringPressed; self.tq = abs(msg.steeringTorque)
      la_div = abs(self.desired_curvature * self.v_ego ** 2 - self.yaw_rate * self.v_ego)
      if self.mode == "stock":
        self.steering_pressed = msg.steeringPressed
      elif self.mode == "64a":
        if msg.steeringPressed:
          self.press_start = t if self.press_start is None else self.press_start
          self.steering_pressed = (t - self.press_start) >= self.press_min
        else:
          self.press_start = None; self.steering_pressed = False
      elif self.mode == "64c":
        # a press is 'steering' only if it actually disturbs lateral control (torque high OR lateral track diverged)
        disturbing = msg.steeringPressed and (self.tq >= self.tq_thr or la_div > self.la_thr)
        if disturbing:
          self.press_start = t if self.press_start is None else self.press_start
          self.steering_pressed = (t - self.press_start) >= self.press_min
        else:
          self.press_start = None; self.steering_pressed = False
      if self.raw_pressed and self.lat_active and self.v_ego > self.min_vego:
        self.touch.append((self.tq, la_div))
  def update_points(self):
    if self.lat_active and self.v_ego > self.min_vego:
      t = []
      if self.steering_pressed: t.append("press-now")
      elif self.t - self.last_steering_pressed_t < self.min_recovery_buffer_sec: t.append("recovery")
      if self.steering_saturated or self.t - self.last_steering_saturated_t < self.min_recovery_buffer_sec: t.append("sat")
      if abs(self.yaw_rate) < self.min_yr: pass
      if not self.calibrator.calib_valid: t.append("calib")
      if self.t - self.last_pose_invalid_t < self.min_recovery_buffer_sec: t.append("pose/la")
      self.pt[t[0] if t else "ok"] += 1
    elif self.lat_active:
      self.pt["slow"] += 1
    super().update_points()
  def update_estimate(self):
    if not self.points_enough(): self.out["warmup"] += 1; return
    times, desired, actual, okay = self.points.get()
    if not self.points_valid(): self.out["few-okay"] += 1; return
    if actual.max() - actual.min() < L.MIN_LAT_ACCEL_RANGE: self.out["la-range"] += 1; return
    if self.last_estimate_t != 0 and times[0] <= self.last_estimate_t:
      ni = next(-i for i, tt in enumerate(reversed(times)) if tt <= self.last_estimate_t)
      if ni == 0 or not np.any(okay[ni:]): self.out["no-new"] += 1; return
    desired = L.masked_symmetric_moving_average(desired, okay, L.SMOOTH_K, L.SMOOTH_SIGMA)
    actual = L.masked_symmetric_moving_average(actual, okay, L.SMOOTH_K, L.SMOOTH_SIGMA)
    delay, corr, conf = self.actuator_delay(desired, actual, okay, self.dt, L.MIN_LAG, L.MAX_LAG)
    if corr < self.min_ncc: self.out["ncc"] += 1; return
    if conf < self.min_confidence: self.out["conf"] += 1; return
    self.out["ACCEPT"] += 1; self.acc.append(delay); self.block_avg.update(delay); self.last_estimate_t = self.t

def run(cfg):
  vb = 0; lag0 = 0.3; allacc = []; ptot = Counter(); otot = Counter(); touch = []; mins = 0.0
  for r in ROUTES:
    ev = pickle.load(open(f"{S}/lagd/{r}.pkl", "rb"))
    e = Est(cfg); e.reset(lag0, vb); n = 0
    for ts, which, b in ev:
      with log.Event.from_bytes(b) as m:
        e.handle_log(ts * 1e-9, which, getattr(m, which))
      if which == "deviceMotion":
        e.update_points(); n += 1
        if n % 5 == 0: e.update_estimate()
    vb = e.block_avg.valid_blocks; _, _, cm, _ = e.block_avg.get(); lag0 = cm if np.isfinite(cm) else lag0
    allacc += e.acc; ptot += e.pt; otot += e.out; touch += e.touch
  vm, vs, _, _ = (lambda a: a.block_avg.get())(Est(cfg)) if False else (np.nan, np.nan, 0, 0)
  return dict(acc=allacc, pt=ptot, out=otot, touch=touch, vb=vb, lag=lag0)

print(f"routes {ROUTES}")
R = {m: run(dict(mode=m)) for m in ("stock", "64a", "64c")}
# touch stats (same for all since based on raw_pressed): take from stock
tc = np.array(R["64a"]["touch"]) if R["64a"]["touch"] else np.zeros((0, 2))
if len(tc):
  print(f"\nraw steeringPressed during op >= 60 km/h: {len(tc)} frames = {len(tc) * 0.01:.1f}s-equiv (carState 100Hz)")
  print(f"  |steeringTorque| p10/25/50/75/90 {'/'.join(f'{np.percentile(tc[:,0],p):.0f}' for p in (10,25,50,75,90))} (STEER_THRESHOLD 350)")
  print(f"  |la_desired - la_actual| p50/75/90 {np.percentile(tc[:,1],50):.2f}/{np.percentile(tc[:,1],75):.2f}/{np.percentile(tc[:,1],90):.2f} m/s2 (max_lat_accel_diff 0.6)")
  print(f"  share of pressed frames with torque < 500: {np.mean(tc[:,0]<500):.0%}, with la_div < 0.3: {np.mean(tc[:,1]<0.3):.0%}")
for m in ("stock", "64a", "64c"):
  d = R[m]; ok = d["pt"].get("ok", 0); tot = sum(d["pt"].values())
  print(f"\n== {m}: usable 'ok' points {ok*0.05/60:.1f} min / {tot*0.05/60:.1f} op-min >= 60 km/h ({ok/max(tot,1):.0%})")
  print(f"   point loss: " + " ".join(f"{k} {v/max(tot,1):.0%}" for k,v in d['pt'].most_common() if k!='ok'))
  print(f"   estimates: " + " ".join(f"{k} {v}" for k,v in d['out'].most_common()))
  a = np.array(d["acc"]); 
  print(f"   accepted {len(a)} -> valid blocks {d['vb']}/5, learned {np.median(a) if len(a) else float('nan'):.3f} s (IQR {np.percentile(a,25) if len(a) else float('nan'):.3f}-{np.percentile(a,75) if len(a) else float('nan'):.3f}) {'ESTIMATED' if d['vb']>=5 else 'unestimated'}")
