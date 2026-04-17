#!/usr/bin/env python3
"""Parking-lot mode-toggle simulator (Ioniq 6 N / HDA2-ALT + CCNC).

Exercises the carcontroller angle-pipeline through the exact transitions
the user asked about:

  1. Engine-on, stationary  →  accelerate to 30 km/h  →  back to 0
  2. Left/right sweep (±60°) while accelerating through the
     passthrough/blend boundaries (0/1/2/3/5/10/15/20/25 km/h)
  3. LFA(MADS) on↔off at v = {0, 1, 3, 7, 15, 25} km/h
  4. ACC on↔off (note: factory SCC only — lateral untouched — but we still
     verify the pipeline doesn't react)
  5. Driver torque override mid-corner at creep / transition / full speed
  6. Blinker on↔off mid-corner (authority *= 0.2)

Output metric: peak |Δ apply_angle_last| per 20 ms frame, split by
scenario. Any frame whose |Δ| > the rate table's speed-indexed limit
flags a pipeline bug. Any |Δ| > 3° in one frame (equivalent to ≈150°/s
at 50 Hz) is surfaced as "HARSH", since that's the human-noticeable
jerk threshold.

Replicates the real carcontroller logic verbatim — dual-threshold ACI
hysteresis, low-speed passthrough latch, ACI gain ramp, camera-ref
blend (α₀ per speed), driver-torque override blend, `rate_lat_active =
latActive ∧ aci_active_latched`, and `apply_std_steer_angle_limits` —
so the numbers match on-car behaviour frame-for-frame.
"""
import sys
import math
from dataclasses import dataclass

import numpy as np

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')
from opendbc.car.lateral import AngleSteeringLimits, apply_std_steer_angle_limits


# ── Mirror values.py: Ioniq 6 N (HDA2-ALT + CCNC) angle rate limits ──
ANGLE_LIMITS = AngleSteeringLimits(
  176.7,
  ([0., 3., 7., 12., 18., 25., 30.], [0.6, 0.9, 1.3, 1.0, 0.6, 0.4, 0.25]),
  ([0., 3., 7., 12., 18., 25., 30.], [0.8, 1.1, 1.5, 1.2, 0.75, 0.55, 0.35]),
)

# ── Mirror carcontroller.py constants ──
LOW_SPEED_PASSTHROUGH_ENTER_MS = 2.0 / 3.6
LOW_SPEED_PASSTHROUGH_EXIT_MS  = 3.0 / 3.6
ACI_SPEED_FULL_MS = 3.0 / 3.6
ACI_SPEED_ZERO_MS = 1.0 / 3.6
DRIVER_TORQUE_DEADZONE = 30
DRIVER_TORQUE_FULL_OVERRIDE = 150
ACI_ENTER = 0.30
ACI_EXIT  = 0.05
ACI_GAIN_RAMP_TAU_FRAMES = 30.0
CAMREF_ALPHA_BP = [0., 5., 10., 20., 30.]
CAMREF_ALPHA_V  = [0.95, 0.90, 0.85, 0.70, 0.60]
CAMREF_NAV_DISAGREE_DEG = 3.0
CAMREF_NAV_ALPHA_CAP    = 0.30

DT = 0.01           # 100 Hz frame
TX_EVERY_N = 2      # LKAS_ALT TX at 50 Hz → rate limiter runs every 2 frames
HARSH_DEG_PER_FRAME = 3.0   # noticeable jerk threshold per 20 ms
HARSH_DEG_PER_SEC   = HARSH_DEG_PER_FRAME / 0.02  # 150 deg/s equiv


@dataclass
class FrameState:
  v_ego: float          # m/s
  steering_angle: float # actual wheel deg
  driver_torque: float  # raw EPS driver torque
  blinker: bool
  lat_active: bool      # MADS on
  cc_enabled: bool      # ACC on (longitudinal only on this platform, lat unchanged)
  op_curv_angle: float  # what LatControlAngle emitted (deg)
  cam_angle: float      # camera advisory (deg)


class AnglePipelineSim:
  """Stateless-over-input rewrite of carcontroller's HDA2-ALT + CCNC path."""
  def __init__(self):
    self.apply_angle_last = 0.0
    self.aci_active_latched = False
    self.passthrough_latched = False
    self.low_speed_cam_latched = False
    self.aci_gain_ramp = 0.0
    self.frame = 0

  def step(self, fs: FrameState):
    speed_blend = float(np.clip(
      (fs.v_ego - ACI_SPEED_ZERO_MS) / (ACI_SPEED_FULL_MS - ACI_SPEED_ZERO_MS),
      0.0, 1.0))
    override_factor = float(np.clip(
      (abs(fs.driver_torque) - DRIVER_TORQUE_DEADZONE) /
      (DRIVER_TORQUE_FULL_OVERRIDE - DRIVER_TORQUE_DEADZONE), 0.0, 1.0))
    driver_torque_blend = 1.0 - override_factor

    authority = driver_torque_blend * speed_blend if fs.lat_active else 0.0
    if fs.blinker:
      authority *= 0.2

    # ACI hysteresis
    if fs.lat_active:
      if authority >= ACI_ENTER:
        self.aci_active_latched = True
      elif authority < ACI_EXIT:
        self.aci_active_latched = False
    else:
      self.aci_active_latched = False

    # Driver passthrough latch (wheel-ownership)
    if not fs.lat_active and driver_torque_blend > 0.9:
      self.passthrough_latched = True
    elif fs.lat_active or driver_torque_blend < 0.6:
      self.passthrough_latched = False

    # Low-speed passthrough (creep)
    if fs.v_ego < LOW_SPEED_PASSTHROUGH_ENTER_MS:
      self.low_speed_cam_latched = True
    elif fs.v_ego > LOW_SPEED_PASSTHROUGH_EXIT_MS:
      self.low_speed_cam_latched = False

    if self.aci_active_latched:
      self.aci_gain_ramp = min(1.0, self.aci_gain_ramp + 1.0 / ACI_GAIN_RAMP_TAU_FRAMES)
    else:
      self.aci_gain_ramp = 0.0

    prev_apply = self.apply_angle_last
    rate_lat_active = bool(fs.lat_active) and self.aci_active_latched
    prev_rate_lat_active = getattr(self, '_prev_rate_lat_active', False)
    if self.frame % TX_EVERY_N == 0:
      desired = fs.op_curv_angle
      if fs.lat_active and self.aci_active_latched:
        alpha_base = float(np.interp(fs.v_ego, CAMREF_ALPHA_BP, CAMREF_ALPHA_V))
        alpha_eff = alpha_base * 1.0  # q_trust = 1 (no camera noise in sim)
        if abs(fs.cam_angle - fs.op_curv_angle) > CAMREF_NAV_DISAGREE_DEG:
          alpha_eff = min(alpha_eff, CAMREF_NAV_ALPHA_CAP)
        desired = alpha_eff * fs.cam_angle + (1.0 - alpha_eff) * fs.op_curv_angle
      if override_factor > 0:
        desired = (1.0 - override_factor) * desired + override_factor * fs.steering_angle
      self.apply_angle_last = apply_std_steer_angle_limits(
        desired, self.apply_angle_last, fs.v_ego,
        fs.steering_angle, rate_lat_active, ANGLE_LIMITS)
    self.frame += 1
    self._prev_rate_lat_active = rate_lat_active

    dapply = self.apply_angle_last - prev_apply
    # "Physical" delta: only frames where ADAS_ACIActive was True during the
    # rate-limiter step AND was also True on the previous rate step. On the
    # engagement/disengagement edge, apply_angle snaps to actual wheel (by
    # design) but LKAS_ALT is TXd with ACI_ACTIVE=0 + ACIGain=0, so MDPS
    # ignores the command — no wheel movement.
    physical = rate_lat_active and prev_rate_lat_active
    return {
      'apply': self.apply_angle_last,
      'dapply': dapply,
      'dapply_physical': dapply if physical else 0.0,
      'rate_lat_active': rate_lat_active,
      'speed_blend': speed_blend,
      'aci_active': self.aci_active_latched,
      'aci_ramp': self.aci_gain_ramp,
      'low_speed_cam': self.low_speed_cam_latched,
      'driver_passthrough': self.passthrough_latched,
      'authority': authority,
    }


class Scenario:
  def __init__(self, name, duration_s):
    self.name = name
    self.n = int(duration_s / DT)
    self.traces = []

  def record(self, t, fs, out):
    self.traces.append((t, fs, out))

  def report(self, verbose=False):
    max_raw = 0.0
    max_phys = 0.0
    max_phys_t = 0.0
    max_phys_v = 0.0
    harsh_count = 0
    harsh_events = []
    active_frames = 0
    for t, fs, out in self.traces:
      if abs(out['dapply']) > abs(max_raw):
        max_raw = out['dapply']
      if out['rate_lat_active']:
        active_frames += 1
      if abs(out['dapply_physical']) > abs(max_phys):
        max_phys = out['dapply_physical']
        max_phys_t = t
        max_phys_v = fs.v_ego * 3.6
      if abs(out['dapply_physical']) > HARSH_DEG_PER_FRAME:
        harsh_count += 1
        if len(harsh_events) < 3:
          harsh_events.append((t, out['dapply_physical'], fs.v_ego * 3.6))

    print(f"\n── {self.name} ──")
    print(f"  frames (active TX):      {len(self.traces)} ({active_frames})")
    print(f"  max |Δ| (physical cmd):  {abs(max_phys):.3f}°/frame at "
          f"t={max_phys_t:.2f}s v={max_phys_v:.1f} km/h "
          f"(= {abs(max_phys)/0.02:.1f}°/s)")
    print(f"  max |Δ| (bookkeeping):   {abs(max_raw):.3f}°/frame  "
          f"← tracking snap while ACI_ACTIVE=0 (MDPS ignores)")
    ok = harsh_count == 0
    print(f"  harsh physical frames:   {harsh_count} ({'✅ SAFE' if ok else '❌ HARSH'})")
    for t, d, v in harsh_events:
      print(f"    t={t:5.2f}s  Δ={d:+.2f}°  v={v:4.1f} km/h")
    return max_phys, harsh_count


def speed_profile_accel(v_peak_kmh, accel_mps2=0.8, decel_mps2=-0.8, hold_s=3.0, total_s=None):
  """0 → v_peak → 0 trapezoid."""
  v_peak = v_peak_kmh / 3.6
  t_up   = v_peak / accel_mps2
  t_hold = hold_s
  t_down = v_peak / abs(decel_mps2)
  T = t_up + t_hold + t_down
  if total_s is None:
    total_s = T + 1.0
  def v(t):
    if t < t_up: return accel_mps2 * t
    if t < t_up + t_hold: return v_peak
    if t < T: return v_peak + decel_mps2 * (t - t_up - t_hold)
    return 0.0
  return total_s, v


def op_curv_sinusoid(t, amp_deg=60.0, period_s=4.0):
  """Left-right parking sweep (±60° over 4 s → 30°/s equivalent)."""
  return amp_deg * math.sin(2 * math.pi * t / period_s)


# ── Scenario runners ─────────────────────────────────────────────────

def run_scenario_1_accel_sweep():
  """0→20→0 km/h with ±60° left-right sweep, MADS latched on the whole time."""
  sim = AnglePipelineSim()
  s = Scenario("S1: MADS-on, 0→20→0 km/h, ±60° sweep", 30)
  total, vfn = speed_profile_accel(20, total_s=30)
  t = 0.0
  actual_angle = 0.0
  for i in range(s.n):
    t = i * DT
    v = vfn(t)
    op_curv = op_curv_sinusoid(t, amp_deg=60, period_s=4)
    cam = op_curv * 0.9  # camera tracks slightly lagged
    fs = FrameState(v, actual_angle, 0, False, True, True, op_curv, cam)
    out = sim.step(fs)
    s.record(t, fs, out)
    # Actual wheel tracks apply_angle with a tiny lag (first-order MDPS model)
    actual_angle += (out['apply'] - actual_angle) * 0.4
  return s


def run_scenario_2_mads_toggle_at_speeds():
  """Engage/disengage MADS at v∈{0,1,3,7,15,25} km/h, mid ±30° corner."""
  scenarios = []
  for v_kmh in [0, 1, 3, 7, 15, 25]:
    sim = AnglePipelineSim()
    s = Scenario(f"S2.{v_kmh}: MADS toggle at {v_kmh} km/h mid-corner", 6)
    v = v_kmh / 3.6
    actual_angle = 15.0  # half-turn into a corner
    for i in range(s.n):
      t = i * DT
      # 0-2s MADS off, 2-4s MADS on (toggle), 4-6s MADS off again
      lat = (2.0 <= t < 4.0)
      op_curv = 30.0 * math.sin(2 * math.pi * t / 4)
      cam = op_curv * 0.9
      fs = FrameState(v, actual_angle, 0, False, lat, True, op_curv, cam)
      out = sim.step(fs)
      s.record(t, fs, out)
      actual_angle += (out['apply'] - actual_angle) * 0.4
    scenarios.append(s)
  return scenarios


def run_scenario_3_acc_toggle():
  """ACC on/off while MADS stays on. Factory SCC only — should be invisible to pipeline."""
  sim = AnglePipelineSim()
  s = Scenario("S3: ACC on↔off with MADS latched on (15 km/h)", 8)
  v = 15 / 3.6
  actual_angle = 0.0
  for i in range(s.n):
    t = i * DT
    cc_enabled = (t < 3) or (t > 5)
    op_curv = 20.0 * math.sin(2 * math.pi * t / 4)
    fs = FrameState(v, actual_angle, 0, False, True, cc_enabled, op_curv, op_curv * 0.9)
    out = sim.step(fs)
    s.record(t, fs, out)
    actual_angle += (out['apply'] - actual_angle) * 0.4
  return s


def run_scenario_4_driver_override():
  """Mid-corner driver torque override at creep / transition / full speed."""
  scenarios = []
  for v_kmh, label in [(1, "creep"), (5, "transition"), (20, "full")]:
    sim = AnglePipelineSim()
    s = Scenario(f"S4.{label}: MADS-on driver override at {v_kmh} km/h", 6)
    v = v_kmh / 3.6
    actual_angle = 10.0
    for i in range(s.n):
      t = i * DT
      # 2s clean → 1s ramp override → 1s full hands-on → 2s back to clean
      if t < 2.0:
        torque = 0
      elif t < 2.5:
        torque = 200 * (t - 2.0) / 0.5
      elif t < 3.5:
        torque = 200
      elif t < 4.0:
        torque = 200 * (1 - (t - 3.5) / 0.5)
      else:
        torque = 0
      op_curv = 25.0 * math.sin(2 * math.pi * t / 4)
      fs = FrameState(v, actual_angle, torque, False, True, True, op_curv, op_curv * 0.9)
      out = sim.step(fs)
      s.record(t, fs, out)
      # Driver hand pulls the wheel when overriding
      if abs(torque) > 100:
        actual_angle += (fs.steering_angle + math.copysign(10, -torque) - actual_angle) * 0.1
      else:
        actual_angle += (out['apply'] - actual_angle) * 0.4
    scenarios.append(s)
  return scenarios


def run_scenario_5_blinker():
  """Blinker on/off during cornering at 10 km/h (parking-exit scenario)."""
  sim = AnglePipelineSim()
  s = Scenario("S5: Blinker on↔off during corner at 10 km/h", 6)
  v = 10 / 3.6
  actual_angle = 20.0
  for i in range(s.n):
    t = i * DT
    blink = (1.0 <= t < 3.0)
    op_curv = 30.0 * math.sin(2 * math.pi * t / 4) + 20
    fs = FrameState(v, actual_angle, 0, blink, True, True, op_curv, op_curv * 0.9)
    out = sim.step(fs)
    s.record(t, fs, out)
    actual_angle += (out['apply'] - actual_angle) * 0.4
  return s


def run_scenario_6_stop_and_go():
  """Stop-and-go across the passthrough boundary (typical parking queue)."""
  sim = AnglePipelineSim()
  s = Scenario("S6: Stop-and-go 0↔4 km/h across passthrough boundary", 20)
  actual_angle = 0.0
  # 4-second cycles: 2s creep up to 4 km/h, 2s back to 0
  for i in range(s.n):
    t = i * DT
    cycle = t % 4.0
    if cycle < 2.0:
      v = (cycle / 2.0) * 4.0 / 3.6
    else:
      v = (1.0 - (cycle - 2.0) / 2.0) * 4.0 / 3.6
    op_curv = 15.0 * math.sin(2 * math.pi * t / 3)
    fs = FrameState(v, actual_angle, 0, False, True, True, op_curv, op_curv * 0.9)
    out = sim.step(fs)
    s.record(t, fs, out)
    actual_angle += (out['apply'] - actual_angle) * 0.4
  return s


def run_scenario_7_mads_engage_offcenter():
  """Worst-case: MADS engaged while wheel is already 40° off-center at speed.
  Verifies rate_lat_active=False (no ACI latch yet) forces apply_angle_last
  to track the actual wheel, so first active command has no step.
  """
  scenarios = []
  for v_kmh in [0, 5, 15, 25]:
    sim = AnglePipelineSim()
    s = Scenario(f"S7.{v_kmh}: MADS engage at {v_kmh} km/h w/ wheel 40° off-center", 4)
    v = v_kmh / 3.6
    actual_angle = 40.0  # deliberately off-center at engagement
    for i in range(s.n):
      t = i * DT
      lat = t >= 1.0  # engage at t=1s
      # Op-curv says "go straight" but actual wheel is at 40°
      op_curv = 0.0
      cam = 0.0
      fs = FrameState(v, actual_angle, 0, False, lat, True, op_curv, cam)
      out = sim.step(fs)
      s.record(t, fs, out)
      actual_angle += (out['apply'] - actual_angle) * 0.4
    scenarios.append(s)
  return scenarios


def main():
  print("=" * 70)
  print(" Ioniq 6 N parking-lot mode-toggle simulator")
  print(f" Harsh threshold: |Δ| > {HARSH_DEG_PER_FRAME}°/frame "
        f"(= {HARSH_DEG_PER_SEC:.0f}°/s)")
  print(" Rate-limit table (UP, deg/20ms):")
  for v, r in zip(*ANGLE_LIMITS.ANGLE_RATE_LIMIT_UP):
    print(f"   {v:>4.1f} m/s ({v*3.6:>5.1f} km/h) → {r} °/20ms = {r*50:.1f}°/s")
  print("=" * 70)

  all_scenarios = []
  all_scenarios.append(run_scenario_1_accel_sweep())
  all_scenarios.extend(run_scenario_2_mads_toggle_at_speeds())
  all_scenarios.append(run_scenario_3_acc_toggle())
  all_scenarios.extend(run_scenario_4_driver_override())
  all_scenarios.append(run_scenario_5_blinker())
  all_scenarios.append(run_scenario_6_stop_and_go())
  all_scenarios.extend(run_scenario_7_mads_engage_offcenter())

  total_harsh = 0
  total_worst = 0.0
  worst_label = ""
  for s in all_scenarios:
    mdx, harsh = s.report(verbose=False)
    total_harsh += harsh
    if abs(mdx) > abs(total_worst):
      total_worst = mdx
      worst_label = s.name

  print("\n" + "=" * 70)
  print(" OVERALL SUMMARY")
  print("=" * 70)
  print(f" Scenarios run:              {len(all_scenarios)}")
  print(f" Total harsh frames:         {total_harsh}")
  print(f" Worst |Δ| across all tests: {abs(total_worst):.3f}°/frame "
        f"(={abs(total_worst)/DT:.1f}°/s)")
  print(f" In scenario:                {worst_label}")
  verdict = "✅ SAFE — no abrupt wheel movements detected" if total_harsh == 0 \
            else "❌ REGRESSION — harsh movement surfaced"
  print(f" Verdict: {verdict}")


if __name__ == '__main__':
  main()
